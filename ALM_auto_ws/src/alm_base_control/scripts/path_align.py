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
        if len(pts) >= 3:
            ys = [p[2] for p in pts]
            self._has_orient = (max(ys) - min(ys)) > 1e-6
        else:
            self._has_orient = False
        if not self._has_orient and len(pts) >= 2:
            # 플래너가 orientation 을 안 채웠다 -> 세그먼트 방위각으로 대체.
            fixed = []
            for i, (x, y, _) in enumerate(pts):
                j = min(i + 1, len(pts) - 1)
                k = i if j > i else max(i - 1, 0)
                dx, dy = pts[j][0] - pts[k][0], pts[j][1] - pts[k][1]
                fixed.append((x, y, math.atan2(dy, dx) if (dx or dy) else 0.0))
            self._pts = fixed

    def clear(self):
        self._pts = []
        self._cum = []
        self._has_orient = False

    def evaluate(self, rx, ry, ryaw):
        """(err_rad, remaining_m, cross_m, idx) 또는 경로가 없으면 None."""
        n = len(self._pts)
        if n < 2:
            return None
        best_i, best_d2 = 0, float("inf")
        for i, (px, py, _) in enumerate(self._pts):
            d2 = (px - rx) ** 2 + (py - ry) ** 2
            if d2 < best_d2:
                best_d2, best_i = d2, i
        # lookahead: 최근접점에서 호길이만큼 전진한 지점
        target_s = self._cum[best_i] + self.lookahead_m
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
                 wz_min=0.15, wz_max=0.45, min_remaining_m=0.0):
        self.enter_rad = math.radians(float(enter_deg))
        self.exit_rad = math.radians(float(exit_deg))
        self.enter_dur = float(enter_dur)
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

        if self.exit_rad >= self.enter_rad:
            raise ValueError("align_exit_deg 는 align_enter_deg 보다 작아야 한다 "
                             "(히스테리시스가 없으면 왕복한다)")

    def reset(self):
        self.state = self.IDLE
        self._hold_start = None

    def remaining_err(self, ryaw):
        return wrap_pi(self.target_yaw - ryaw)

    def update(self, now, ryaw, err, remaining_m, eligible, gated):
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

        반환: (active: bool, wz: float, err_left: float|None)
        """
        if self.state == self.COOLDOWN:
            if (now - self._cd_start) >= self.cooldown_sec:
                self.state = self.IDLE
                self._hold_start = None
            return False, 0.0, None

        if self.state == self.ALIGN:
            left = self.remaining_err(ryaw)
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
        if abs(err) < self.enter_rad:
            self._hold_start = None
            return False, 0.0, None
        if self._hold_start is None:
            self._hold_start = now
        if (now - self._hold_start) < self.enter_dur:
            return False, 0.0, None

        # ★ 진입: 목표 헤딩을 절대각으로 래치한다. 이 뒤로 경로가 바뀌어도
        #   래치는 유지된다 — 매 틱 재계산하면 왕복이 생긴다.
        self.state = self.ALIGN
        self.target_yaw = wrap_pi(ryaw + err)
        self.entry_err = err
        self._t0 = now
        self._hold_start = None
        self.n_entries += 1
        mag = min(max(self.kp * abs(err), self.wz_min), self.wz_max)
        return True, math.copysign(mag, err), err

    def _exit(self, now, reason):
        self.state = self.COOLDOWN
        self._cd_start = now
        self._hold_start = None
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
    # 직선 경로 (+x 방향), 로봇이 90° 틀어져 있음
    pe = PathHeadingError(lookahead_m=1.0)
    pe.set_path([(float(i) * 0.1, 0.0, 0.0) for i in range(50)])
    chk("orientation 있음으로 판정", pe.has_orientation() is False,
        "(전부 yaw=0 이므로 세그먼트 대체 경로를 탐)")
    r = pe.evaluate(0.0, 0.0, math.radians(90.0))
    chk("헤딩오차 -90°", abs(math.degrees(r[0]) + 90.0) < 1e-6,
        f"got {math.degrees(r[0]):.3f}")
    chk("남은거리 4.9 m", abs(r[1] - 4.9) < 1e-6, f"got {r[1]:.4f}")
    chk("횡오차 0", abs(r[2]) < 1e-9)

    # yaw 가 실제로 변하는 경로: 후진 구간 흉내 (pose.yaw=0 인데 -x 로 진행)
    pts = [(-0.1 * i, 0.0, 0.0) for i in range(30)]
    pts = [(x, y, t + 1e-3 * i) for i, (x, y, t) in enumerate(pts)]  # yaw 미세 변화
    pe2 = PathHeadingError(lookahead_m=1.0)
    pe2.set_path(pts)
    chk("orientation 채워짐 감지", pe2.has_orientation() is True)
    e2 = pe2.evaluate(0.0, 0.0, 0.0)[0]
    chk("후진 경로를 180° 오차로 오인하지 않음", abs(math.degrees(e2)) < 5.0,
        f"got {math.degrees(e2):.3f}°")

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
    am3 = AlignManeuver(enter_deg=50.0, exit_deg=10.0, enter_dur=0.0)
    am3.update(0.0, 0.0, math.radians(90.0), 10.0, True, False)
    a, _, _ = am3.update(0.1, 0.0, math.radians(90.0), 10.0, False, False)
    chk("eligible 해제 시 기동 중단", a is False and am3.state == AlignManeuver.COOLDOWN)

    print("\n" + ("전부 통과" if ok else "실패 있음"))
    return 0 if ok else 1


if __name__ == "__main__":
    import sys
    sys.exit(_selftest())
