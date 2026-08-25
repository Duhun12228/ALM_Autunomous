#!/usr/bin/env python3
"""path_align - '경로 대비 헤딩오차'와 그걸 고치는 ALIGN 기동.

--- 왜 필요한가 -------------------------------------------------------------
전역 플래너(SmacPlannerHybrid)는 최소 선회반경 R_min 의 원호와 직선만 이어 붙여
경로를 만든다. 즉 **제자리 회전을 표현할 수 없다.** 그래서 목표 근처에서 헤딩을
크게 바꿔야 하는 경우, 애초에 그런 경로가 나오지 않거나(빈 경로 -> abort),
나오더라도 R_min 으로는 따라갈 수 없는 경로가 나온다.

4륜 독립조향은 제자리 회전(spin, CONS(9) 47°)이 되므로 원래 이런 제약이 없다.
문제는 그 능력을 **아무도 쓰라고 지시하지 않는다**는 것이었다. 기존
command_manager._select_auto 의 spin 진입 조건은

    abs(wz) >= 0.35  AND  hypot(vx,vy) <= 0.04

로, Nav2 가 내놓은 **속도명령의 모양**을 보고 사후 분류할 뿐이다. Nav2 는 경로에
제자리 회전이 없으니 그런 명령을 낼 이유가 없고, 결국 spin 은 막혔을 때 BT
리커버리로만 발동했다.

이 모듈은 순서를 뒤집는다: **경로를 직접 보고**, 내 헤딩이 경로가 요구하는
헤딩에서 너무 벌어졌으면 스스로 spin 을 걸어 고친다.

--- 왜 '모드'가 아니라 '기동'인가 -------------------------------------------
예전에 '요구 선회반경 < R_min 이면 spin' 이라는 **모드 분류**를 넣었다가 뺐다.
spin 이 되면 vx 가 0 으로 눌리고, vx 가 0 이면 조건이 다시 거짓이 되어 normal 로
돌아가고, 그러면 조건이 또 참이 된다. 실측에서 0.02 s 주기로 왕복했고 정상
선회에서도 vx 가 0.3 -> 0.04 로 무너졌다 (command_manager._select_auto 설계메모).

이 모듈은 그 함정을 세 가지로 피한다.

  1) 트리거가 **속도명령이 아니라 경로 헤딩오차**다. spin 이 되어 vx 가 0 이
     되어도 헤딩오차는 그대로다 -> 진입 조건이 자기부정을 하지 않는다.
  2) 진입 순간 **목표 헤딩을 절대각으로 래치**한다. 매 틱 재계산하지 않으므로
     기동에 명확한 끝이 있다.
  3) 진입/이탈 문턱이 다르고(히스테리시스), 이탈 뒤 쿨다운이 있다.

--- 헤딩오차의 정의 ---------------------------------------------------------
lookahead 지점의 **경로 pose 자체의 yaw** 를 목표로 삼는다. 로봇->지점 방위각
(pure-pursuit 식)이 아니다. 이유는 후진이다: Hybrid-A* 는 Reeds-Shepp 후진
구간에서도 pose.yaw 에 '차체 헤딩'을 넣는다(진행 방향이 아니라). 방위각을 쓰면
후진 구간을 헤딩오차 180° 로 오인해 불필요한 spin 을 건다.

경로가 orientation 을 채우지 않은 경우(전 pose 의 yaw 가 동일)에는 이 정의가
무의미하므로 세그먼트 방위각으로 대체한다. 어느 쪽인지는 has_orientation() 이
알려준다.
"""

import math


def wrap_pi(a):
    """각을 (-pi, pi] 로 접는다."""
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))


class PathHeadingError:
    """/plan 을 들고 있다가 (로봇 pose) -> (헤딩오차, 남은거리, 횡오차)를 준다."""

    def __init__(self, lookahead_m=1.0):
        self.lookahead_m = float(lookahead_m)
        self._pts = []              # [(x, y, yaw)]
        self._cum = []              # 누적 호길이
        self._has_orient = False

    def has_orientation(self):
        return self._has_orient

    def n_points(self):
        return len(self._pts)

    def set_path(self, pts):
        """pts: [(x, y, yaw)] (map 프레임)."""
        pts = [(float(x), float(y), float(t)) for x, y, t in pts]
        self._pts = pts
        cum = [0.0]
        for i in range(1, len(pts)):
            cum.append(cum[-1] + math.hypot(pts[i][0] - pts[i - 1][0],
                                            pts[i][1] - pts[i - 1][1]))
        self._cum = cum
        self._has_orient = self._orientations_consistent(pts)

    @staticmethod
    def _orientations_consistent(pts, tol_deg=35.0, frac=0.7):
        """경로의 pose.yaw 가 **경로 형상과 맞는가**를 본다.

        ##왜 이렇게 판정하나##
        예전에는 'yaw 값이 전부 같으면 플래너가 안 채운 것' 으로 보고 세그먼트
        방위각으로 대체했다. 그 판정이 틀렸다 — **직선 경로는 전진이든 후진이든
        yaw 가 상수**다. 그래서 정상적인 직선 후진 경로(yaw=0, 위치는 -x 로 진행)를
        '미채움' 으로 오판하고 방위각(=180°)으로 갈아끼웠고, ALIGN 이 오차 180° 를
        보고 **불필요한 제자리 회전을 명령**했다. 실측 재현: +180.0°.

        올바른 기준은 'yaw 가 변하는가' 가 아니라 **'yaw 가 진행 방향과 정합하는가'**
        다. Hybrid-A* 의 pose.yaw 는 차체 헤딩이므로, 각 점에서

            전진 구간: yaw ≈ 세그먼트 방위각
            후진 구간: yaw ≈ 세그먼트 방위각 + 180°

        둘 중 하나에 가깝다. 둘 다에서 멀면 yaw 가 안 채워진 것이다.
        (yaw 를 안 채운 경로가 우연히 +x 로 곧게 뻗으면 방위각 0 == yaw 0 이라
         '정합' 으로 판정되는데, 그 경우엔 실제로 값이 맞으므로 문제없다.)
        """
        if len(pts) < 3:
            return False
        good = total = 0
        for i in range(len(pts) - 1):
            dx = pts[i + 1][0] - pts[i][0]
            dy = pts[i + 1][1] - pts[i][1]
            if math.hypot(dx, dy) < 1e-6:      # 중복점은 방위각이 무의미
                continue
            b = math.atan2(dy, dx)
            e_fwd = abs(wrap_pi(pts[i][2] - b))
            e_rev = abs(wrap_pi(pts[i][2] - b - math.pi))
            total += 1
            if min(e_fwd, e_rev) <= math.radians(tol_deg):
                good += 1
        return total > 0 and (good / total) >= frac

    def clear(self):
        self._pts = []
        self._cum = []
        self._has_orient = False

    def endpoint(self):
        """경로의 끝점 (x, y). 경로가 없으면 None.

        ##왜 /goal_pose 를 안 쓰나## 목표가 RViz 토픽으로 올 수도, NavigateToPose
        액션으로 직접 올 수도 있다. 어느 쪽이든 /plan 의 끝점은 항상 그 목표다
        (tolerance 0.50 m 안에서). 전달 경로에 의존하지 않으려고 여기서 뽑는다.
        """
        if not self._pts:
            return None
        return (self._pts[-1][0], self._pts[-1][1])

    def evaluate(self, rx, ry, ryaw, lookahead_m=None):
        """(err_rad, remaining_m, cross_m, idx) 또는 경로가 없으면 None.

        lookahead_m 을 주면 그 틱만 다른 lookahead 를 쓴다 (정지 상태 전용).

        ##왜 정지 상태에 lookahead 를 따로 두나 (2026-08-25)##
        Hybrid-A* 는 시작 상태 theta 를 **현재 로봇 헤딩**으로 두고 R_min 원호만
        이어 붙인다. 즉 경로는 항상 로봇 헤딩에 **접해서** 출발한다. 그래서
        출발 시점에 이 함수가 낼 수 있는 헤딩오차는 호각으로 상한이 걸린다:

            |err|_max = lookahead_m / R_min      [rad]

        R_min = 1.643 m 이므로 lookahead 1.0 m 면 34.9° 가 최대다. 그 말은
        align_enter_deg(60°)가 **출발 시점에는 수학적으로 도달 불가능**이라는
        뜻이다 — 60° 를 보려면 lookahead 가 1.72 m 이상이어야 한다.
        벽을 보고 선 채 뒤쪽 목표를 받아도 ALIGN 이 안 걸리던 이유가 이것이다.

        정지 중에는 lookahead 를 3 m 로 늘려 상한을 104.6° 까지 열어 준다.
        주행 중에는 늘리면 안 된다 — 실측 헤딩오차 분포(p90 39.8°)가 전부
        lookahead 1.0 m 기준이라, 늘리면 정상 선회를 오인해 vx 가 무너진다.
        """
        n = len(self._pts)
        if n < 2:
            return None
        # ★ orientation 을 못 믿으면 **추측하지 않는다.**
        #   틀린 추측의 대가가 '불필요한 180° 제자리 회전' 이라 너무 비싸다.
        #   ALIGN 을 쉬게 두는 편이 훨씬 안전하다 (경로 추종은 MPPI 가 계속 한다).
        if not self._has_orient:
            return None
        best_i, best_d2 = 0, float("inf")
        for i, (px, py, _) in enumerate(self._pts):
            d2 = (px - rx) ** 2 + (py - ry) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        # lookahead: 최근접점에서 호길이만큼 전진한 지점
        la = self.lookahead_m if lookahead_m is None else float(lookahead_m)
        target_s = self._cum[best_i] + la
        j = best_i
        while j + 1 < n and self._cum[j] < target_s:
            j += 1
        err = wrap_pi(self._pts[j][2] - ryaw)
        remaining = self._cum[-1] - self._cum[best_i]
        return err, remaining, math.sqrt(best_d2), best_i


class AlignManeuver:
    """헤딩오차가 크면 제자리 회전으로 고치는 래치 기동.

    상태:  IDLE --(오차 지속)--> ALIGN --(목표 도달/타임아웃)--> COOLDOWN --> IDLE
    """

    IDLE = "idle"
    ALIGN = "align"
    COOLDOWN = "cooldown"

    def __init__(self, enter_deg=50.0, exit_deg=12.0, enter_dur=0.5,
                 max_sec=25.0, cooldown_sec=3.0, kp=1.2,
                 wz_min=0.15, wz_max=0.45, min_remaining_m=0.0,
                 enter_deg_stopped=None, enter_dur_stopped=None,
                 relatch_stopped=False):
        self.enter_rad = math.radians(float(enter_deg))
        self.exit_rad = math.radians(float(exit_deg))
        self.enter_dur = float(enter_dur)
        # ---- 정지 상태 전용 진입 조건 (2026-08-23) ----
        # 왜 따로 두나: 위 enter_deg(60도)가 높은 이유는 **주행 중 정상 선회를
        # 오인하지 않기 위해서**다 (실측 경로 헤딩오차 p90 = 39.8도, 30도 초과가
        # 전체 틱의 35%). 낮추면 정상 선회마다 spin 이 걸려 vx 가 무너진다 —
        # 실제로 문턱을 낮췄다가 되돌린 이력이 있다(base_control.yaml 참고).
        #
        # 그런데 **정지 상태에는 그 위험이 아예 없다.** 안 움직이고 있으니
        # 오인해서 잃을 것이 없고, 오히려 지금이 회전하기 가장 싼 순간이다.
        # 그래서 '벽을 보고 있는데 목표가 뒤' 같은 상황에서, 잘못된 방향으로
        # 굴러가기 **전에** 헤딩을 맞출 수 있다.
        # None 이면 주행 중 값과 같게 두어 예전 거동을 유지한다.
        self.enter_rad_stopped = math.radians(
            float(enter_deg if enter_deg_stopped is None else enter_deg_stopped))
        self.enter_dur_stopped = float(
            enter_dur if enter_dur_stopped is None else enter_dur_stopped)
        self.max_sec = float(max_sec)
        self.cooldown_sec = float(cooldown_sec)
        self.kp = float(kp)
        self.wz_min = float(wz_min)
        self.wz_max = float(wz_max)
        self.min_remaining_m = float(min_remaining_m)

        self.state = self.IDLE
        self.target_yaw = 0.0       # 래치된 절대 목표 헤딩 [rad, map]
        self.entry_err = 0.0
        self._hold_start = None     # 진입 조건이 연속 참이 된 시각
        self._t0 = 0.0              # ALIGN 경과 (게이트 중에는 안 흐름)
        self._cd_start = 0.0
        self.last_exit_reason = ""
        self.n_entries = 0

        # ---- 정지 중 재래치 (2026-08-25) ----
        # 진입 때 한 번만 래치하는 것이 기본이다(매 틱 재계산하면 왕복한다).
        # 그런데 **정지 상태에서는 그 왕복 위험이 없다** — 제자리에서 돌기만
        # 하므로 진입 조건이 자기부정을 하지 않는다. 그래서 정지 중에 한해
        # 목표를 갱신해, 한 번의 spin 으로 끝까지 돌 수 있게 한다.
        #
        # 왜 필요한가: 재래치가 없으면 ALIGN 1회가 도는 각도는
        #     래치각(최대 lookahead/R_min) - exit_deg
        # 뿐이다. lookahead 1.0 m 기본값에서는 34.9 - 15 = 19.9° 다. 그리고
        # 왕복 1회 비용이 dwell 5 s x 2 + 회전 1.4 s ~= 11 s 이므로, 180° 를
        # 돌려면 9사이클 x 11 s = 약 100 s 가 든다. 300 s 타임아웃의 1/3 이다.
        #
        # 폭주 방지 두 겹:
        #   (a) **회전 방향을 절대 안 바꾼다** — 진입 시 부호를 _dir 로 고정하고
        #       부호가 뒤집히면 재래치를 멈춘다. 그래야 단조 회전이 보장된다.
        #   (b) **늘리기만 한다** — 새 목표가 지금 남은 오차보다 작으면 무시한다.
        #       낡은 /plan 이 순간적으로 오차를 줄여 기동을 조기 종료시키는 것을
        #       막는다.
        # 상한은 align_max_sec 이 잡는다 (0.25 rad/s x 25 s = 358°, 약 1회전).
        self.relatch_stopped = bool(relatch_stopped)
        self._dir = 0.0             # 진입 시 회전 부호. 0 = 기동 중 아님
        self.n_relatch = 0

        if self.exit_rad >= self.enter_rad:
            raise ValueError("align_exit_deg 는 align_enter_deg 보다 작아야 한다 "
                             "(히스테리시스가 없으면 왕복한다)")
        if self.exit_rad >= self.enter_rad_stopped:
            raise ValueError("align_exit_deg 는 align_enter_deg_stopped 보다도 "
                             "작아야 한다 (정지 진입 -> 즉시 이탈 왕복 방지)")

    def reset(self):
        self.state = self.IDLE
        self._hold_start = None
        self._dir = 0.0

    def remaining_err(self, ryaw):
        return wrap_pi(self.target_yaw - ryaw)

    def update(self, now, ryaw, err, remaining_m, eligible, gated,
               stopped=False, relatch=False,
               enter_rad_override=None, enter_dur_override=None):
        """한 틱 진행.

        now         : 초
        ryaw        : 로봇 헤딩 [rad, map]
        err         : 경로 헤딩오차 [rad] (없으면 None)
        remaining_m : 경로 남은 길이 [m] (없으면 None)
        eligible    : 지금 ALIGN 을 걸어도 되는 상황인가 (auto 모드 · 주행 중 ·
                      e-stop 아님 · 경로 신선함)
        gated       : dwell/기동정렬 등으로 지금 어차피 못 움직이는가.
                      True 면 ALIGN 타임아웃이 흐르지 않는다 — 안 그러면
                      모드전환 dwell(3 s)이 타임아웃을 잡아먹는다.
        stopped     : 로봇이 사실상 정지해 있는가. True 면 진입 문턱으로
                      enter_deg_stopped / enter_dur_stopped 를 쓴다.
                      **진입 판정에만 쓰인다.**
        enter_rad_override / enter_dur_override :
                      주면 이 틱의 진입 문턱을 통째로 대체한다. 호출부가
                      경로 헤딩이 아닌 **다른 오차**(목표 직선 방위각 등)를
                      err 로 넣을 때, 그 오차에 맞는 문턱을 같이 넘기기 위한
                      것이다. 안 그러면 두 문턱 중 큰 쪽이 실효 문턱이 되어
                      호출부가 의도한 값이 조용히 무시된다.
        relatch     : 이 틱에 목표 헤딩을 갱신해도 되는가 (정지 중 전용).
                      호출부가 relatch_stopped 와 정지 여부를 함께 판단해
                      넘긴다. 진입 판정과 분리해 둔 이유는, 진입은
                      dwell 중에 막아야 하지만(재진입 루프) 재래치는
                      dwell 중에도 살아 있어야 하기 때문이다.

        반환: (active: bool, wz: float, err_left: float|None)
        """
        if self.state == self.COOLDOWN:
            if (now - self._cd_start) >= self.cooldown_sec:
                self.state = self.IDLE
                self._hold_start = None
            return False, 0.0, None

        if self.state == self.ALIGN:
            left = self.remaining_err(ryaw)
            # ---- 정지 중 재래치 (위 __init__ 주석의 근거) ----
            #   · 방향 유지(_dir)  · 늘리기만  두 조건을 모두 만족할 때만.
            if (self.relatch_stopped and relatch and err is not None
                    and self._dir != 0.0
                    and math.copysign(1.0, err) == self._dir
                    and abs(err) > abs(left)):
                self.target_yaw = wrap_pi(ryaw + err)
                self.n_relatch += 1
                left = err
            if abs(left) <= self.exit_rad:
                self._exit(now, "정렬 완료")
                return False, 0.0, left
            if not eligible:
                # e-stop 이 걸렸거나 명령이 끊겼다. 기동을 붙들고 있을 이유가 없다.
                self._exit(now, "주행 조건 해제")
                return False, 0.0, left
            if gated:
                # dwell/기동정렬로 어차피 못 도는 구간이다. 시계를 끌고 간다.
                self._t0 = now
            elif (now - self._t0) >= self.max_sec:
                self._exit(now, f"타임아웃 {self.max_sec:.0f} s")
                return False, 0.0, left
            mag = min(max(self.kp * abs(left), self.wz_min), self.wz_max)
            return True, math.copysign(mag, left), left

        # IDLE
        if not eligible or err is None:
            self._hold_start = None
            return False, 0.0, None
        if remaining_m is not None and remaining_m < self.min_remaining_m:
            self._hold_start = None
            return False, 0.0, None
        # 정지 중이면 더 민감한 문턱을 쓴다 (위 __init__ 주석의 근거).
        enter_rad = self.enter_rad_stopped if stopped else self.enter_rad
        enter_dur = self.enter_dur_stopped if stopped else self.enter_dur
        if enter_rad_override is not None:
            enter_rad = abs(float(enter_rad_override))
        if enter_dur_override is not None:
            enter_dur = max(0.0, float(enter_dur_override))
        if abs(err) < enter_rad:
            self._hold_start = None
            return False, 0.0, None
        if self._hold_start is None:
            self._hold_start = now
        if (now - self._hold_start) < enter_dur:
            return False, 0.0, None

        # ★ 진입: 목표 헤딩을 절대각으로 래치한다. 이 뒤로 경로가 바뀌어도
        #   래치는 유지된다 — 매 틱 재계산하면 왕복이 생긴다.
        self.state = self.ALIGN
        self.target_yaw = wrap_pi(ryaw + err)
        self.entry_err = err
        self._dir = math.copysign(1.0, err) if err != 0.0 else 0.0
        self._t0 = now
        self._hold_start = None
        self.n_entries += 1
        mag = min(max(self.kp * abs(err), self.wz_min), self.wz_max)
        return True, math.copysign(mag, err), err

    def _exit(self, now, reason):
        self.state = self.COOLDOWN
        self._cd_start = now
        self._hold_start = None
        self._dir = 0.0
        self.last_exit_reason = reason


# --------------------------------------------------------------------------
# 자체 시험 (ROS 없이 실행 가능):  python3 path_align.py
# --------------------------------------------------------------------------
def _selftest():
    ok = True

    def chk(name, cond, extra=""):
        nonlocal ok
        print(f"  [{'OK ' if cond else 'FAIL'}] {name} {extra}")
        ok = ok and cond

    print("PathHeadingError")
    # 직선 **전진** 경로 (+x 로 진행, yaw=0). 로봇이 90° 틀어져 있음
    pe = PathHeadingError(lookahead_m=1.0)
    pe.set_path([(float(i) * 0.1, 0.0, 0.0) for i in range(50)])
    chk("직선 전진 경로의 orientation 을 신뢰", pe.has_orientation() is True)
    r = pe.evaluate(0.0, 0.0, math.radians(90.0))
    chk("헤딩오차 -90°", abs(math.degrees(r[0]) + 90.0) < 1e-6,
        f"got {math.degrees(r[0]):.3f}")
    chk("남은거리 4.9 m", abs(r[1] - 4.9) < 1e-6, f"got {r[1]:.4f}")
    chk("횡오차 0", abs(r[2]) < 1e-9)

    # ★ 회귀 시험: 직선 **후진** 경로 (yaw=0 인데 위치는 -x 로 진행).
    #   예전 판정('yaw 가 변하지 않으면 미채움')은 이걸 방위각 180° 로 갈아끼워
    #   ALIGN 이 불필요한 제자리 회전을 명령했다.
    pe_rev = PathHeadingError(lookahead_m=1.0)
    pe_rev.set_path([(-0.1 * i, 0.0, 0.0) for i in range(30)])
    chk("직선 후진 경로의 orientation 을 신뢰", pe_rev.has_orientation() is True)
    e_rev = pe_rev.evaluate(0.0, 0.0, 0.0)[0]
    chk("직선 후진을 180° 오차로 오인하지 않음", abs(math.degrees(e_rev)) < 1e-6,
        f"got {math.degrees(e_rev):+.1f}° (예전 코드는 +180.0°)")

    # yaw 가 형상과 전혀 안 맞는 경로 = 플래너가 안 채운 것 -> 판단 보류
    pe_bad = PathHeadingError(lookahead_m=1.0)
    pe_bad.set_path([(0.0, 0.1 * i, 0.0) for i in range(30)])   # +y 로 가는데 yaw=0
    chk("미채움 orientation 감지", pe_bad.has_orientation() is False)
    chk("미채움이면 판단 보류(None)", pe_bad.evaluate(0.0, 0.0, 0.0) is None,
        "추측해서 도는 것보다 안 도는 게 낫다")

    # 곡선 경로(전진)도 정상 신뢰
    pts_c = []
    for i in range(40):
        th = 0.02 * i
        pts_c.append((math.sin(th) * 5.0, 5.0 - math.cos(th) * 5.0, th))
    pe_c = PathHeadingError(1.0); pe_c.set_path(pts_c)
    chk("곡선 전진 경로 신뢰", pe_c.has_orientation() is True)

    print("AlignManeuver")
    am = AlignManeuver(enter_deg=50.0, exit_deg=10.0, enter_dur=0.5,
                       max_sec=20.0, cooldown_sec=2.0, kp=1.2,
                       wz_min=0.15, wz_max=0.45)
    t, ryaw = 0.0, 0.0
    err = math.radians(90.0)
    a, wz, _ = am.update(t, ryaw, err, 10.0, True, False)
    chk("문턱 넘어도 즉시 진입 안 함(지속시간)", a is False)
    t = 0.6
    a, wz, _ = am.update(t, ryaw, err, 10.0, True, False)
    chk("0.5 s 지속 뒤 진입", a is True and wz > 0)
    chk("목표 헤딩 래치 = 90°", abs(math.degrees(am.target_yaw) - 90.0) < 1e-9)

    # ★ 핵심: 진입해서 vx 가 0 이 되어도(=명령 모양이 바뀌어도) 기동은 유지된다
    a, wz, _ = am.update(t + 0.02, ryaw, 0.0, 10.0, True, False)
    chk("경로오차가 0 으로 보고돼도 기동 유지 (래치)", a is True)

    # 회전 진행
    steps = 0
    while am.state == AlignManeuver.ALIGN and steps < 5000:
        t += 0.02
        a, wz, left = am.update(t, ryaw, err, 10.0, True, False)
        ryaw = wrap_pi(ryaw + wz * 0.02)
        steps += 1
    chk("목표 헤딩에 수렴", abs(math.degrees(wrap_pi(math.radians(90.0) - ryaw))) <= 10.0,
        f"최종 {math.degrees(ryaw):.2f}°")
    chk("이탈 사유 = 정렬 완료", am.last_exit_reason == "정렬 완료",
        f"'{am.last_exit_reason}'")
    chk("이탈 직후 쿨다운", am.state == AlignManeuver.COOLDOWN)

    # 쿨다운 동안은 오차가 커도 재진입 금지
    a, _, _ = am.update(t + 0.5, 0.0, math.radians(120.0), 10.0, True, False)
    chk("쿨다운 중 재진입 금지", a is False)
    a, _, _ = am.update(t + 3.0, 0.0, math.radians(120.0), 10.0, True, False)
    chk("쿨다운 뒤에도 지속시간은 다시 요구", a is False)

    # 히스테리시스 강제
    try:
        AlignManeuver(enter_deg=30.0, exit_deg=30.0)
        chk("exit >= enter 는 거부", False)
    except ValueError:
        chk("exit >= enter 는 거부", True)

    # gated 동안 타임아웃이 안 흐르는지
    am2 = AlignManeuver(enter_deg=50.0, exit_deg=10.0, enter_dur=0.0, max_sec=1.0)
    am2.update(0.0, 0.0, math.radians(90.0), 10.0, True, False)
    chk("진입", am2.state == AlignManeuver.ALIGN)
    for k in range(1, 500):          # 10 s 동안 게이트
        am2.update(k * 0.02, 0.0, math.radians(90.0), 10.0, True, True)
    chk("게이트 10 s 동안 타임아웃 안 남", am2.state == AlignManeuver.ALIGN)

    # eligible 이 꺼지면 기동 해제
    # ---- 정지 상태 진입 문턱 (2026-08-23) ----
    # 주행 중 문턱 60도, 정지 문턱 25도로 두고 '오차 35도' 를 준다.
    # 주행 중이면 안 걸리고 정지 중이면 걸려야 한다.
    ams = AlignManeuver(enter_deg=60.0, exit_deg=15.0, enter_dur=0.6,
                        enter_deg_stopped=25.0, enter_dur_stopped=0.1)
    e35 = math.radians(35.0)
    a, _, _ = ams.update(0.0, 0.0, e35, 5.0, True, False, stopped=False)
    chk("주행 중 35도는 진입 안 함(문턱 60도)", a is False)
    a, _, _ = ams.update(1.0, 0.0, e35, 5.0, True, False, stopped=False)
    chk("주행 중은 지속시간이 지나도 진입 안 함", a is False)

    ams2 = AlignManeuver(enter_deg=60.0, exit_deg=15.0, enter_dur=0.6,
                         enter_deg_stopped=25.0, enter_dur_stopped=0.1)
    a, _, _ = ams2.update(0.0, 0.0, e35, 5.0, True, False, stopped=True)
    chk("정지 중 35도도 지속시간 전에는 진입 안 함", a is False)
    a, wz, _ = ams2.update(0.15, 0.0, e35, 5.0, True, False, stopped=True)
    chk("정지 중 0.1 s 뒤 진입 (문턱 25도)", a is True and wz > 0)
    chk("정지 진입도 목표 헤딩을 절대각으로 래치",
        abs(math.degrees(ams2.target_yaw) - 35.0) < 1e-9)

    # 정지 문턱이 exit 보다 작으면 즉시 왕복하므로 거부해야 한다
    try:
        AlignManeuver(enter_deg=60.0, exit_deg=20.0, enter_deg_stopped=15.0)
        chk("exit >= enter_stopped 는 거부", False)
    except ValueError:
        chk("exit >= enter_stopped 는 거부", True)

    # 기본값(None)이면 주행 중 값과 같아야 한다 = 예전 거동 보존
    amd = AlignManeuver(enter_deg=60.0, exit_deg=15.0, enter_dur=0.6)
    chk("stopped 인자 미지정 시 주행 중 문턱과 동일",
        abs(amd.enter_rad_stopped - amd.enter_rad) < 1e-12
        and abs(amd.enter_dur_stopped - amd.enter_dur) < 1e-12)
    a, _, _ = amd.update(0.0, 0.0, e35, 5.0, True, False, stopped=True)
    a2, _, _ = amd.update(1.0, 0.0, e35, 5.0, True, False, stopped=True)
    chk("기본값에서는 정지여도 35도로 진입 안 함(회귀 0)", a is False and a2 is False)

    # ---- 정지 중 재래치 (2026-08-25) ----
    # 오차 30도로 진입 -> 돌면서 경로가 '더 돌아야 한다'(같은 방향 50도)고
    # 말하면 목표가 늘어나야 한다. 한 번의 기동으로 끝까지 도는 것이 목적이다.
    amr = AlignManeuver(enter_deg=20.0, exit_deg=5.0, enter_dur=0.0,
                        relatch_stopped=True)
    a, _, _ = amr.update(0.0, 0.0, math.radians(30.0), 9.0, True, False,
                         stopped=True, relatch=True)
    chk("재래치: 진입", a is True and abs(math.degrees(amr.target_yaw) - 30.0) < 1e-9)
    amr.update(0.1, 0.0, math.radians(50.0), 9.0, True, False,
               stopped=True, relatch=True)
    chk("재래치: 같은 방향으로 커지면 목표가 늘어난다",
        abs(math.degrees(amr.target_yaw) - 50.0) < 1e-9 and amr.n_relatch == 1)

    # 줄어드는 방향은 무시한다 (낡은 /plan 에 의한 조기 종료 방지)
    amr.update(0.2, 0.0, math.radians(10.0), 9.0, True, False,
               stopped=True, relatch=True)
    chk("재래치: 줄어드는 갱신은 무시", abs(math.degrees(amr.target_yaw) - 50.0) < 1e-9)

    # 부호가 뒤집히면 재래치를 멈춘다 (단조 회전 보장 = 왕복 방지)
    amr.update(0.3, 0.0, math.radians(-80.0), 9.0, True, False,
               stopped=True, relatch=True)
    chk("재래치: 반대 방향 갱신은 거부(단조 회전)",
        abs(math.degrees(amr.target_yaw) - 50.0) < 1e-9 and amr.n_relatch == 1)

    # relatch=False 면 예전 거동 그대로 (회귀 0)
    amn = AlignManeuver(enter_deg=20.0, exit_deg=5.0, enter_dur=0.0,
                        relatch_stopped=True)
    amn.update(0.0, 0.0, math.radians(30.0), 9.0, True, False,
               stopped=True, relatch=False)
    amn.update(0.1, 0.0, math.radians(50.0), 9.0, True, False,
               stopped=True, relatch=False)
    chk("relatch=False 면 래치 그대로", abs(math.degrees(amn.target_yaw) - 30.0) < 1e-9)
    amo = AlignManeuver(enter_deg=20.0, exit_deg=5.0, enter_dur=0.0)
    amo.update(0.0, 0.0, math.radians(30.0), 9.0, True, False,
               stopped=True, relatch=True)
    amo.update(0.1, 0.0, math.radians(50.0), 9.0, True, False,
               stopped=True, relatch=True)
    chk("relatch_stopped=False(기본) 면 래치 그대로",
        abs(math.degrees(amo.target_yaw) - 30.0) < 1e-9)

    # ---- 정지 전용 lookahead ----
    # Hybrid-A* 경로는 로봇 헤딩에 접해서 출발하므로, 헤딩오차 상한이
    # lookahead/R_min 이다. R_min=1.643 m 원호를 그려 그 상한을 확인한다.
    R = 1.643
    arc = [(R * math.sin(t), R * (1.0 - math.cos(t)), t)
           for t in [i * 0.01 for i in range(int(2.0 / (R * 0.01)) + 1)]]
    pe_arc = PathHeadingError(lookahead_m=1.0)
    pe_arc.set_path(arc)
    e1 = pe_arc.evaluate(0.0, 0.0, 0.0)[0]
    e3 = pe_arc.evaluate(0.0, 0.0, 0.0, lookahead_m=2.0)[0]
    chk("lookahead 1.0 m 헤딩오차 = 호각 1.0/R_min",
        abs(math.degrees(e1) - math.degrees(1.0 / R)) < 1.0)
    chk("lookahead 를 늘리면 오차 상한이 열린다", e3 > e1 * 1.8)
    chk("lookahead 미지정이면 기본값 그대로",
        abs(pe_arc.evaluate(0.0, 0.0, 0.0)[0] - e1) < 1e-12)

    # endpoint()
    chk("endpoint 는 경로 끝점", pe_arc.endpoint() == (arc[-1][0], arc[-1][1]))
    chk("경로 없으면 endpoint None", PathHeadingError().endpoint() is None)

    # ---- 진입 문턱 오버라이드 ----
    # 호출부가 '목표 직선 방위각' 을 err 로 넣을 때, 그 오차에 맞는 문턱(45도)을
    # 같이 넘길 수 있어야 한다. 안 넘기면 stopped 문턱(60도)이 실효 문턱이 되어
    # 호출부의 의도가 조용히 무시된다 — 실제로 그 버그를 냈었다.
    amv = AlignManeuver(enter_deg=60.0, exit_deg=15.0, enter_dur=0.6,
                        enter_deg_stopped=60.0, enter_dur_stopped=0.15)
    e45 = math.radians(45.0)
    a, _, _ = amv.update(0.0, 0.0, e45, 9.0, True, False, stopped=True)
    a2, _, _ = amv.update(1.0, 0.0, e45, 9.0, True, False, stopped=True)
    chk("오버라이드 없으면 45도는 문턱 60도에 막힌다", a is False and a2 is False)

    amv2 = AlignManeuver(enter_deg=60.0, exit_deg=15.0, enter_dur=0.6,
                         enter_deg_stopped=60.0, enter_dur_stopped=0.15)
    amv2.update(0.0, 0.0, e45, 9.0, True, False, stopped=True,
                enter_rad_override=math.radians(40.0), enter_dur_override=0.15)
    a, wz, _ = amv2.update(0.2, 0.0, e45, 9.0, True, False, stopped=True,
                           enter_rad_override=math.radians(40.0),
                           enter_dur_override=0.15)
    chk("오버라이드 40도면 45도로 진입", a is True and wz > 0)

    am3 = AlignManeuver(enter_deg=50.0, exit_deg=10.0, enter_dur=0.0)
    am3.update(0.0, 0.0, math.radians(90.0), 10.0, True, False)
    a, _, _ = am3.update(0.1, 0.0, math.radians(90.0), 10.0, False, False)
    chk("eligible 해제 시 기동 중단", a is False and am3.state == AlignManeuver.COOLDOWN)

    print("\n" + ("전부 통과" if ok else "실패 있음"))
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
