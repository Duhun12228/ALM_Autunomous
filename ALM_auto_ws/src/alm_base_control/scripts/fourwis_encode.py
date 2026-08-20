#!/usr/bin/env python3
"""fourwis_encode - Nav2 twist -> STM32 4WIS 인터페이스(steer_deg/speed_rpm/mode) 변환.

STM32 의 ``FourWIS_DrivingAlgorithm`` 은 RC 조종기 규격 입력을 받습니다.
즉 바퀴별 조향각/속도(역기구학)는 STM32 가 계산하고, Jetson 은 조종기처럼
'조향각 1개 + 속도 1개 + 모드' 만 넘깁니다. 이 모듈은 그 변환만 담당합니다.

  Nav2 twist (vx, vy, wz)  ->  (steer_deg, speed_rpm, mode_id)

ROS 의존성이 없어 단독 실행으로 검증할 수 있습니다:  python3 fourwis_encode.py

── STM32 규격에서 온 제약 (코드 실측) ────────────────────────────────
  * mode: 0=정지, 1=일반, 2=자가추종(미구현→정지), 3=크랩, 4=제로턴, 그외=정지
  * 일반 주행은 내부에서 ``normalDriveMode(-steer_deg, ...)`` 로 호출된다.
    즉 **부호가 반전**되어 있어 좌회전(wz>0)을 원하면 음수를 보내야 한다.
  * 일반 주행의 ``rpm_cmd`` 는 차체 속도가 아니라 **내측 전륜 rpm** 이다.
    (좌회전이면 rpm_FL = rpm_cmd, 나머지 3륜은 여기서 비율로 스케일됨)
  * 크랩/제로턴은 ``steer_deg`` 의 **부호만** 사용한다. 조향각 크기는 무시되고
    STM32 상수(CONS(8)/CONS(9))로 고정되므로, 연속적인 방향 제어가 불가능하다.
    Jetson 이 조절할 수 있는 것은 rpm(속도 크기)과 방향(부호) 뿐이다.

── 일반 주행 기하 ────────────────────────────────────────────────────
후륜이 전륜과 반대로 rws 비율만큼 보조조향하는 4WS 모델입니다.
뒤축 중심을 원점, 전방을 +x, 좌측을 +y 로 두면 순간회전중심(ICR)은

    ICR_y = half_track + B / (tan(d) + tan(rws * d))        d = 내측 전륜 조향각

이고, 차체 중심선 위 모든 점의 전진속도는 ``vx = wz * ICR_y`` 로 같습니다.
따라서 목표 (vx, wz) 는 ``ICR_y = vx / wz`` 하나로 요약되고, 위 식을
d 에 대해 수치역산(이분법)하면 보낼 조향각이 나옵니다.

내측 전륜의 선속도는 ICR 까지의 거리로 구합니다. 유도해보면 그 거리는
    s = B / (sin(d) - cos(d) * tan(-rws * d))
와 정확히 같아서, ``v_inner = |wz| * s`` 로 간단히 떨어집니다.

  ⚠ ICR_y >= half_track 이므로 **최소 회전반경이 half_track 으로 제한**됩니다.
    그보다 급한 선회를 요청하면 최대 조향각으로 clamp 되고 실제 궤적이
    명령과 달라집니다(직진 성분이 남음). 제자리 회전은 spin 모드를 쓰세요.
"""

import math

# STM32 mode_cmd 값 (FourWIS_DrivingAlgorithm 실측)
MODE_STOP = 0
MODE_NORMAL = 1
MODE_FOLLOW = 2      # anchor 입력 없어 STM32 에서 정지 처리됨
MODE_CRAB = 3
MODE_ZERO_TURN = 4

# drive_mode 문자열(command_manager) -> STM32 mode_cmd
DRIVE_MODE_TO_ID = {
    "normal": MODE_NORMAL,
    "crab": MODE_CRAB,
    "spin": MODE_ZERO_TURN,
}

_EPS_V = 1e-3        # 이 이하는 정지로 간주 [m/s]
_EPS_W = 1e-3        # 이 이하는 회전 없음으로 간주 [rad/s]


class FourWISParams:
    """STM32 CONS 벡터 + 구동계 제원 중 Jetson 이 알아야 하는 값들.

    wheelbase_m/track_m/rws_ratio/crab_rpm_scale/zero_turn_rpm_scale 는 STM32
    CONS(2)/CONS(3)/CONS(1)/CONS(10)/CONS(11) 과 **같은 값**이어야 명령과 실제
    궤적이 일치합니다. 이 5개 기본값은 ALM07.slx 에서 읽은 실제 CONS 값입니다.
    ##CONFIRM## 나머지(구동계 제원·기구 한계)는 CONS 에 없어 실측이 필요합니다.
    """

    def __init__(
        self,
        wheelbase_m=1.0,             # CONS(2) B = 1000 mm
        track_m=0.919,               # CONS(3) T = 919 mm (전체 윤거)
        rws_ratio=0.5,               # CONS(1) RWS_percent / 100 = 50%
        wheel_radius_m=0.103,
        gear_ratio=1.0,              # 모터 rpm / 휠 rpm
        max_steer_deg=30.0,
        straight_angle_deg=2.0,      # CONS(4) 조향 데드밴드
        crab_rpm_scale=0.5,          # CONS(10)
        zero_turn_rpm_scale=0.6,     # CONS(11)
        crab_steer_sign=1.0,         # +vy 를 낼 때 보낼 steer_deg 부호
        spin_steer_sign=1.0,         # +wz 를 낼 때 보낼 steer_deg 부호
        max_rpm=3000.0,
    ):
        self.B = float(wheelbase_m)
        self.half_track = float(track_m) * 0.5
        self.rws = float(rws_ratio)
        self.r = float(wheel_radius_m)
        self.gear = float(gear_ratio)
        self.max_steer_rad = math.radians(abs(float(max_steer_deg)))
        self.straight_angle_deg = abs(float(straight_angle_deg))
        self.crab_rpm_scale = float(crab_rpm_scale)
        self.zero_rpm_scale = float(zero_turn_rpm_scale)
        self.crab_steer_sign = 1.0 if float(crab_steer_sign) >= 0 else -1.0
        self.spin_steer_sign = 1.0 if float(spin_steer_sign) >= 0 else -1.0
        self.max_rpm = abs(float(max_rpm))

        if self.r <= 0.0:
            raise ValueError("wheel_radius_m 은 0 보다 커야 합니다")
        if self.B <= 0.0:
            raise ValueError("wheelbase_m 은 0 보다 커야 합니다")

    # 회전중심 근처에서 tan 이 발산하지 않도록 rws*d 를 제한
    def _icr_y(self, d):
        """내측 전륜 조향각 d[rad] 일 때의 ICR 측방거리 [m]."""
        dr = min(self.rws * d, math.radians(89.0))
        den = math.tan(d) + math.tan(dr)
        if den <= 1e-9:
            return float("inf")
        return self.half_track + self.B / den

    def _icr_distance_to_inner_front(self, d):
        """내측 전륜 ~ ICR 거리 s [m]. v_inner = |wz| * s."""
        dr = -min(self.rws * d, math.radians(89.0))   # 후륜은 반대 방향
        den = math.sin(d) - math.cos(d) * math.tan(dr)
        if abs(den) < 1e-9:
            return float("inf")
        return abs(self.B / den)


# ---------------------------------------------------------------- 프레임 생성
# uart_protocol.md v2 와 mcu_bridge.py 의 CMD_FMT 에 대응. 실제 UART 프레임은
# mcu_bridge 가 만들지만, 오프라인 프레임 검증/테스트용으로 여기에도 둡니다.
SYNC0, SYNC1 = 0xAA, 0x55
MSG_COMMAND = 0x01


def crc16_ccitt(data):
    """CRC16-CCITT (poly 0x1021, init 0xFFFF). 범위 = [msg_type, len, payload]."""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def build_command_frame(steer_deg, speed_rpm, mode_id, flags=0x01):
    """Command 프레임 16바이트 생성 (payload 10B + sync 2 + hdr 2 + crc 2)."""
    import struct
    payload = struct.pack("<ffBB", float(steer_deg), float(speed_rpm),
                          int(mode_id) & 0xFF, int(flags) & 0xFF)
    body = bytes([MSG_COMMAND, len(payload)]) + payload
    return bytes([SYNC0, SYNC1]) + body + struct.pack(">H", crc16_ccitt(body))


def min_turn_radius(params):
    """최대 조향각에서의 최소 회전반경 [m]. 이보다 급한 선회는 불가능."""
    return params._icr_y(params.max_steer_rad)


def max_angular_speed(params, max_linear_x):
    """일반 주행에서 낼 수 있는 최대 요레이트 [rad/s].

    Nav2 의 max_angular_z 를 이 값보다 크게 잡으면 조향이 상시 포화되어
    실제 궤적이 계획과 어긋납니다.
    """
    r_min = min_turn_radius(params)
    if not math.isfinite(r_min) or r_min <= 0.0:
        return float("inf")
    return abs(max_linear_x) / r_min


def mps_to_rpm(v_mps, params):
    """휠 접지속도 [m/s] -> 모터 rpm."""
    return v_mps / (2.0 * math.pi * params.r) * 60.0 * params.gear


def wz_from_steer(steer_deg, vx, params):
    """조향 명령각 -> 그 각도로 실제 나오는 요레이트 [rad/s]. ``_encode_normal`` 의 역함수.

    조향 슬루 제한을 건 뒤 ``McuCommand.cmd_vel`` 을 재정합하는 데 쓴다.
    슬루 제한으로 ``steer_deg`` 를 깎아놓고 ``cmd_vel.angular.z`` 는 원래 목표를
    그대로 두면, 한 메시지 안의 두 필드가 서로 다른 이야기를 한다 — 로그를 읽는
    사람도, 그 값을 쓰는 노드도 속는다.

    부호 규약은 ``_encode_normal`` 과 같다: STM32 가 내부에서 부호를 반전하므로
    **전진** 좌회전(wz>0)은 음수 ``steer_deg`` 다.

    ★ ``vx`` 의 부호도 반드시 들어간다. 요레이트는 ω = v / R 이고 R 의 부호는
      조향각이 정하므로, **같은 조향각에서 후진하면 요레이트가 뒤집힌다.**
      (예전에는 ``wz`` 부호만 보고 계산해서 후진 선회가 반대로 나왔다.
       ``_encode_normal`` 과 이 함수가 **같이** 틀려 있었기 때문에
       '인코딩 -> 역산' 방식의 자체 시험은 이를 통과시켰다 — _selftest 의
       물리 독립 검증 항목 참고.)
    """
    d = math.radians(abs(float(steer_deg)))
    if d < 1e-6 or abs(vx) < 1e-9:
        return 0.0
    icr_y = params._icr_y(d)
    if not math.isfinite(icr_y) or icr_y <= 0.0:
        return 0.0
    # 전진 기준 회전 방향(조향 부호가 결정) x 진행 방향(vx 부호)
    sign = -math.copysign(1.0, float(steer_deg)) * math.copysign(1.0, float(vx))
    return math.copysign(abs(vx) / icr_y, sign)


def rpm_for_steer(steer_deg, vx, params):
    """**실제로 명령한** 조향각과 목표 차체속도 -> 내측 전륜 rpm.

    슬루 제한으로 조향각을 깎은 뒤 rpm 을 다시 계산하기 위한 함수다.
    STM32 는 받은 rpm 을 '내측 전륜' 값으로 보고 나머지 3륜을 **자기가 들고 있는
    조향각 기준으로** 스케일한다. 그래서 우리가 목표 조향각으로 계산한 rpm 을
    보내면서 조향각만 깎으면, 조향각과 rpm 이 서로 다른 선회를 가리킨다.

    실측 오차: 목표 29.5° 인데 슬루로 1° 만 나간 순간 차체속도가 -15.8% 어긋난다.
    """
    v = float(vx)
    if abs(v) < _EPS_V:
        return 0.0
    d = math.radians(abs(float(steer_deg)))
    # STM32 직진 데드밴드(CONS(4)) 이하는 STM32 가 직진으로 처리한다.
    if math.degrees(d) <= params.straight_angle_deg:
        return mps_to_rpm(v, params)
    icr_y = params._icr_y(d)
    if not math.isfinite(icr_y) or icr_y <= 0.0:
        return mps_to_rpm(v, params)
    v_inner = abs(v) * params._icr_distance_to_inner_front(d) / icr_y
    return math.copysign(mps_to_rpm(v_inner, params), v)


def slew_limit(target_deg, prev_deg, max_rate_deg_s, dt):
    """조향각 명령의 변화율을 제한한다. ``max_rate_deg_s <= 0`` 이면 제한 없음.

    왜 twist 가속제한(``max_accel_theta``)으로는 부족한가: normal 모드 조향각은
    ``wz`` 가 아니라 **비율 wz/vx** 로 결정된다. 성분별 가속제한은 이 비를 전혀
    묶지 못해서, 특히 ``vx`` 가 작은 출발 구간에서 ``dδ/dt`` 가 사실상 무한대가
    된다(실측 재현: 20 ms 만에 0° -> 30°, 1500 deg/s). 어떤 조향 모터도 못 따라간다.

    ⚠ ``max_rate_deg_s`` 를 실제 액추에이터 슬루율보다 **낮게** 잡는 것은 안전하다
      (액추에이터가 여유롭게 따라오므로 명령 = 실제가 되어 모델이 자명하게 맞다).
      **높게** 잡는 것이 위험하다(못 따라가고, 그 사실을 알 방법이 없다).
      실측 전에는 낮게 잡을 것.
    """
    if max_rate_deg_s is None or max_rate_deg_s <= 0.0 or dt <= 0.0:
        return float(target_deg)
    step = float(max_rate_deg_s) * float(dt)
    return float(max(prev_deg - step, min(float(target_deg), prev_deg + step)))


def solve_inner_front_steer(vx, wz, params):
    """목표 (vx, wz) 를 만드는 내측 전륜 조향각 [rad, 부호없음] 을 역산.

    ICR_y(d) 는 d 에 대해 단조감소하므로 이분법으로 안전하게 풀립니다.
    요청한 선회가 기하학적으로 불가능하면 최대 조향각을 돌려주고
    ``clamped=True`` 로 알립니다.
    """
    target_icr_y = abs(vx) / abs(wz)

    lo, hi = 1e-6, params.max_steer_rad
    # 최대 조향각으로도 ICR 이 목표보다 바깥이면 더 급하게 못 돈다 -> clamp
    if params._icr_y(hi) > target_icr_y:
        return hi, True
    # 아주 작은 각으로도 ICR 이 목표보다 안쪽이면 거의 직진
    if params._icr_y(lo) < target_icr_y:
        return lo, False

    for _ in range(60):
        mid = 0.5 * (lo + hi)
        if params._icr_y(mid) > target_icr_y:
            lo = mid          # 아직 덜 꺾임
        else:
            hi = mid
    return 0.5 * (lo + hi), False


def encode(vx, vy, wz, drive_mode, stopped, params):
    """twist + 모드 -> STM32 로 보낼 (steer_deg, speed_rpm, mode_id, note).

    Args:
        vx, vy, wz: 제한이 이미 적용된 목표 twist (base_link 기준).
        drive_mode: "normal" | "crab" | "spin" (command_manager 의 유효 모드).
        stopped:    하드 정지 조건이 걸렸는지. True 면 무조건 정지 프레임.
        params:     FourWISParams

    Returns:
        (steer_deg, speed_rpm, mode_id, note)
        note 는 로그용 문자열이며 문제 없으면 "" 입니다.
    """
    if stopped:
        return 0.0, 0.0, MODE_STOP, ""

    mode_id = DRIVE_MODE_TO_ID.get(drive_mode, MODE_NORMAL)

    if mode_id == MODE_CRAB:
        return _encode_crab(vx, vy, params)
    if mode_id == MODE_ZERO_TURN:
        return _encode_spin(wz, params)
    return _encode_normal(vx, wz, params)


def _finish(steer_deg, speed_rpm, mode_id, params, note=""):
    speed_rpm = max(-params.max_rpm, min(speed_rpm, params.max_rpm))
    limit_deg = math.degrees(params.max_steer_rad)
    steer_deg = max(-limit_deg, min(steer_deg, limit_deg))
    return float(steer_deg), float(speed_rpm), int(mode_id), note


def _encode_normal(vx, wz, params):
    """일반 주행: Ackermann + 후륜 보조조향의 역산."""
    moving = abs(vx) > _EPS_V
    turning = abs(wz) > _EPS_W

    if not moving and not turning:
        return _finish(0.0, 0.0, MODE_NORMAL, params)

    if not turning:
        # 순수 직진: STM32 직진 분기에서 4륜 모두 rpm_cmd 를 그대로 쓴다.
        return _finish(0.0, mps_to_rpm(vx, params), MODE_NORMAL, params)

    if not moving:
        # 전진 성분 없이 회전만 요청 -> 일반 주행으로는 불가능(제자리 회전 아님).
        # 최대 조향으로 clamp 되지만 실제로는 직진 성분이 남으므로 spin 을 써야 한다.
        d = params.max_steer_rad
        v_inner = abs(wz) * params._icr_distance_to_inner_front(d)
        steer_deg = -math.copysign(math.degrees(d), wz)
        rpm = math.copysign(mps_to_rpm(v_inner, params), 1.0)
        return _finish(steer_deg, rpm, MODE_NORMAL, params,
                       "normal 모드에서 vx≈0 회전 요청 - spin 모드가 필요합니다")

    d, clamped = solve_inner_front_steer(vx, wz, params)

    # 계산된 조향각이 STM32 직진 데드밴드(CONS(4)) 이하면 STM32 가 어차피 직진으로
    # 처리한다. Jetson 도 직진 명령으로 맞춰야 예상 rpm 과 실제 출력이 어긋나지 않는다.
    if math.degrees(d) <= params.straight_angle_deg:
        return _finish(0.0, mps_to_rpm(vx, params), MODE_NORMAL, params)

    # ★ 내측 전륜 속도는 **vx 기준**으로 계산한다 (wz 기준이 아니다).
    #   비클램프 구간에서는 icr_y(d) == |vx|/|wz| 이므로 둘이 완전히 같다.
    #   클램프 구간(요청 R < R_min)에서만 갈리는데, 그때 wz 기준으로 계산하면
    #   **낼 수 없는 요레이트를 맞추려고 전진속도를 부풀린다.**
    #   실측: vx=0.10, wz=0.10 요청(R=1.0 m < R_min 1.643) 시 rpm 12.67 vs 7.71
    #        -> 명령한 것보다 1.64 배 빠르게 달린다.
    #   기구 한계로 못 도는 것은 '덜 도는' 것으로 처리해야지 '더 빨리 가는' 것으로
    #   처리하면 안 된다. 곡률 오차는 MPPI 가 메우지만 속도 폭주는 못 메운다.
    v_inner = abs(vx) * params._icr_distance_to_inner_front(d) / params._icr_y(d)

    # ★ 부호반전: STM32 가 normalDriveMode(-steer_deg, ...) 로 호출하므로
    #   **전진** 좌회전(wz>0)을 원하면 음수를 보내야 한다.
    # ★★ vx 부호도 들어간다. ω = v/R 이고 R 부호는 조향각이 정하므로, 같은
    #     조향각에서 후진하면 요레이트가 뒤집힌다. 즉 후진에서 같은 wz 를 내려면
    #     조향 부호를 뒤집어야 한다. Hybrid-A* 가 후진(reverse_penalty 3.0)을
    #     실제로 쓰므로 이 부호가 틀리면 후진 선회가 통째로 반대로 돈다.
    steer_deg = -math.copysign(math.degrees(d), wz * math.copysign(1.0, vx))
    # 전/후진은 rpm 부호로 표현
    rpm = math.copysign(mps_to_rpm(v_inner, params), vx)

    note = ""
    if clamped:
        note = (f"선회 반경 부족: 요청 R={abs(vx / wz):.2f} m < "
                f"최소 {params._icr_y(params.max_steer_rad):.2f} m -> 조향 clamp")
    return _finish(steer_deg, rpm, MODE_NORMAL, params, note)


def _encode_crab(vx, vy, params):
    """크랩: STM32 가 고정 각도로 병진. 방향(부호)과 속도만 전달 가능."""
    speed = math.hypot(vx, vy)
    if speed <= _EPS_V or abs(vy) <= _EPS_V:
        return _finish(0.0, 0.0, MODE_CRAB, params)

    rpm = mps_to_rpm(speed, params) / max(1e-6, params.crab_rpm_scale)
    rpm = math.copysign(rpm, vy)
    steer_deg = params.crab_steer_sign * math.copysign(
        math.degrees(params.max_steer_rad), vy)
    return _finish(steer_deg, rpm, MODE_CRAB, params,
                   "크랩 각도는 STM32 고정값 - 요청 방향과 다를 수 있습니다")


def _encode_spin(wz, params):
    """제로턴: 차체 중심 기준 제자리 회전. 회전속도는 rpm 으로만 조절."""
    if abs(wz) <= _EPS_W:
        return _finish(0.0, 0.0, MODE_ZERO_TURN, params)

    # 회전중심(차체 중심) ~ 바퀴 거리
    r_spin = math.hypot(params.half_track, params.B * 0.5)
    rpm = mps_to_rpm(abs(wz) * r_spin, params) / max(1e-6, params.zero_rpm_scale)
    rpm = math.copysign(rpm, wz)
    steer_deg = params.spin_steer_sign * math.copysign(
        math.degrees(params.max_steer_rad), wz)
    return _finish(steer_deg, rpm, MODE_ZERO_TURN, params)


# ---------------------------------------------------------------- self test
def _selftest():
    """단독 실행 검증: 역산 -> 정방향 재계산으로 twist 가 복원되는지 확인."""
    ok = True

    for rws in (0.0, 0.3, 0.5, 0.6):
        p = FourWISParams(rws_ratio=rws)
        for vx in (0.1, 0.25, 0.45, -0.15):
            for wz in (0.05, 0.2, 0.5, -0.35):
                steer_deg, rpm, mode, note = encode(vx, 0.0, wz, "normal", False, p)
                if mode != MODE_NORMAL:
                    ok = False
                    print(f"  FAIL mode {mode}")
                    continue
                if note:                      # clamp 된 경우는 복원 대상 아님
                    continue
                if steer_deg == 0.0:          # 직진 데드밴드로 접힌 경우도 제외
                    continue
                # 정방향: 보낸 각도로 ICR 을 다시 계산해 vx/wz 복원
                # ★ vx 부호가 들어간다 — 같은 조향각에서 후진하면 요레이트가 뒤집힌다
                d = math.radians(abs(steer_deg))
                icr_y = p._icr_y(d)
                wz_hat = math.copysign(
                    abs(vx) / icr_y,
                    -math.copysign(1.0, steer_deg) * math.copysign(1.0, vx))
                err = abs(wz_hat - wz)
                if err > 1e-4:
                    ok = False
                    print(f"  FAIL rws={rws} vx={vx} wz={wz}: wz_hat={wz_hat:.5f}")
                # **전진** 좌회전이면 음수를 보내야 한다. 후진이면 반대.
                if vx > 0 and wz > 0 and steer_deg > 0:
                    ok = False
                    print(f"  FAIL 부호(전진): wz={wz} -> steer={steer_deg}")
                if vx < 0 and wz > 0 and steer_deg < 0:
                    ok = False
                    print(f"  FAIL 부호(후진): wz={wz} -> steer={steer_deg}")
    print("[normal] 역산<->정방향 일치 및 부호규약:", "OK" if ok else "FAIL")

    p = FourWISParams()
    # 직진
    s, rpm, m, _ = encode(0.3, 0.0, 0.0, "normal", False, p)
    straight_ok = (s == 0.0 and m == MODE_NORMAL
                   and abs(rpm - mps_to_rpm(0.3, p)) < 1e-6)
    print("[straight] steer=0, rpm=vx 환산:", "OK" if straight_ok else "FAIL")

    # 직진 데드밴드: 데드밴드 이하 조향각은 STM32 와 동일하게 직진으로 접는다
    p_db = FourWISParams(straight_angle_deg=5.0)
    d_small = math.radians(1.0)                       # 5° 데드밴드보다 작은 조향각
    vx_db, wz_db = 0.3, 0.3 / p_db._icr_y(d_small)
    s_db, rpm_db, _, _ = encode(vx_db, 0.0, wz_db, "normal", False, p_db)
    deadband_ok = (s_db == 0.0 and abs(rpm_db - mps_to_rpm(vx_db, p_db)) < 1e-6)
    print("[deadband] CONS(4) 이하 조향 -> 직진:", "OK" if deadband_ok else "FAIL")

    # 정지 게이팅
    s, rpm, m, _ = encode(0.4, 0.0, 0.3, "normal", True, p)
    stop_ok = (s == 0.0 and rpm == 0.0 and m == MODE_STOP)
    print("[stop] 하드정지 -> mode 0:", "OK" if stop_ok else "FAIL")

    # spin / crab 모드 매핑
    _, rpm_s, m_s, _ = encode(0.0, 0.0, 0.4, "spin", False, p)
    _, rpm_c, m_c, _ = encode(0.0, 0.2, 0.0, "crab", False, p)
    map_ok = (m_s == MODE_ZERO_TURN and m_c == MODE_CRAB
              and rpm_s > 0 and rpm_c > 0)
    print("[mode] spin->4, crab->3:", "OK" if map_ok else "FAIL")

    # 최소 회전반경 초과 요청은 clamp 되고 note 가 붙어야 한다
    _, _, _, note = encode(0.05, 0.0, 0.8, "normal", False, p)
    print("[clamp] 급선회 경고:", "OK" if note else "FAIL")

    # wz_from_steer 가 _encode_normal 의 정확한 역함수인지 (조향 슬루 제한 재정합용)
    inv_ok = True
    for rws in (0.0, 0.3, 0.5, 0.6):
        pi_ = FourWISParams(rws_ratio=rws)
        for vx in (0.1, 0.25, 0.45, -0.15):
            for wz in (0.05, 0.2, -0.35):
                steer_deg, _, _, note_i = encode(vx, 0.0, wz, "normal", False, pi_)
                if note_i or steer_deg == 0.0:      # clamp/데드밴드는 복원 대상 아님
                    continue
                wz_hat = wz_from_steer(steer_deg, vx, pi_)
                if abs(wz_hat - wz) > 1e-4:
                    inv_ok = False
                    print(f"  FAIL rws={rws} vx={vx} wz={wz}: wz_hat={wz_hat:.5f}")
    print("[inverse] wz_from_steer == encode 역함수:", "OK" if inv_ok else "FAIL")

    # slew_limit: 상한 이내면 그대로, 넘으면 step 만큼만
    sl_ok = (slew_limit(10.0, 0.0, 45.0, 0.02) == 0.9              # 45*0.02 = 0.9
             and slew_limit(0.5, 0.0, 45.0, 0.02) == 0.5           # 상한 이내
             and slew_limit(-10.0, 0.0, 45.0, 0.02) == -0.9
             and slew_limit(10.0, 0.0, 0.0, 0.02) == 10.0)         # 0 = 제한 없음
    print("[slew] 조향 슬루 제한:", "OK" if sl_ok else "FAIL")

    # 프레임이 uart_protocol.md 의 예시와 바이트 단위로 같아야 한다.
    # (mcu_bridge.py 의 CMD_FMT 경로와 이 도구가 갈라지지 않도록 고정)
    expect = bytes.fromhex("AA55010A000048410000964301018EDD")
    got = build_command_frame(12.5, 300.0, 1, 0x01)
    frame_ok = got == expect
    if not frame_ok:
        print("  기대:", expect.hex(" ").upper())
        print("  실제:", got.hex(" ").upper())
    print("[frame] 문서 예시 프레임과 바이트 일치:", "OK" if frame_ok else "FAIL")

    # ---- 물리 독립 검증: 요레이트 부호 ------------------------------------
    # ##왜 따로 두나## 위의 [inv] 검증은 encode() 로 만든 값을 wz_from_steer() 로
    # 되돌리는 방식이다. 두 함수가 **같은 규약으로 같이 틀리면 그대로 통과한다.**
    # 실제로 후진 조향 부호 버그가 이 방식을 통과했다.
    # 그래서 여기서는 코드를 전혀 참조하지 않고, 자전거 모델 ω = v/R 만으로
    # 기대 부호를 세워 대조한다.
    sign_ok = True
    for rws in (0.0, 0.5):
        pp = FourWISParams(rws_ratio=rws)
        for vx in (0.30, -0.30, 0.08, -0.08):
            for wz in (0.10, -0.10, 0.04, -0.04):
                steer_deg, rpm, mode, _ = encode(vx, 0.0, wz, "normal", False, pp)
                if mode != MODE_NORMAL or abs(steer_deg) < 1e-9:
                    continue
                d = math.radians(abs(steer_deg))
                # 조향 부호 -> 전진 기준 회전 방향 (STM32 내부 부호반전 규약)
                turn_fwd = -math.copysign(1.0, steer_deg)
                # 진행 방향까지 곱해야 실제 요레이트 부호가 된다
                expect = turn_fwd * math.copysign(1.0, vx)
                got = math.copysign(1.0, wz)
                if expect != got:
                    sign_ok = False
                    print(f"   [sign] vx={vx:+.2f} wz={wz:+.3f} -> steer {steer_deg:+.2f}° "
                          f"실제 요레이트 부호 {expect:+.0f}, 요청 {got:+.0f}")
                # rpm 부호는 진행 방향을 표현해야 한다
                if rpm != 0.0 and math.copysign(1.0, rpm) != math.copysign(1.0, vx):
                    sign_ok = False
                    print(f"   [sign] rpm 부호가 vx 와 다름: vx={vx:+.2f} rpm={rpm:+.1f}")
                # wz_from_steer 도 같은 부호여야 한다
                if math.copysign(1.0, wz_from_steer(steer_deg, vx, pp)) != got:
                    sign_ok = False
                    print(f"   [sign] wz_from_steer 부호 불일치: vx={vx:+.2f} steer={steer_deg:+.2f}")
    print("[sign] 요레이트 부호 (전/후진 모두, 물리 독립):", "OK" if sign_ok else "FAIL")

    # ---- 슬루 제한 후 rpm 재계산 -------------------------------------------
    # rpm_for_steer(목표조향, vx) 는 encode() 의 rpm 과 같아야 한다(같은 조향각이므로).
    rpm_ok = True
    for pp in (FourWISParams(), FourWISParams(rws_ratio=0.0)):
        for vx in (0.20, -0.07, 0.10):
            for wz in (0.10, -0.05, 0.02):
                steer_deg, rpm, mode, _ = encode(vx, 0.0, wz, "normal", False, pp)
                if mode != MODE_NORMAL:
                    continue
                again = rpm_for_steer(steer_deg, vx, pp)
                if abs(again - rpm) > 1e-6:
                    rpm_ok = False
                    print(f"   [rpm] vx={vx:+.2f} wz={wz:+.3f} steer={steer_deg:+.2f}° "
                          f"encode {rpm:.3f} != rpm_for_steer {again:.3f}")
    # 조향각을 깎으면 rpm 도 바뀌어야 한다 (안 바뀌면 재계산이 무의미)
    pz = FourWISParams()
    r_full = rpm_for_steer(-29.0, 0.10, pz)
    r_small = rpm_for_steer(-1.0, 0.10, pz)
    if abs(r_full - r_small) < 1e-3:
        rpm_ok = False
        print(f"   [rpm] 조향각이 달라도 rpm 이 같다: {r_full:.3f} vs {r_small:.3f}")
    print(f"[rpm] 슬루 후 rpm 재계산 (29°->1° 에서 {r_full:.2f}->{r_small:.2f} rpm):",
          "OK" if rpm_ok else "FAIL")

    return (ok and straight_ok and deadband_ok and stop_ok and map_ok
            and inv_ok and sl_ok and frame_ok and sign_ok and rpm_ok)


if __name__ == "__main__":
    import sys
    sys.exit(0 if _selftest() else 1)
