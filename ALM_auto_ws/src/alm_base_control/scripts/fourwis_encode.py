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

    ##CONFIRM## 기본값은 docs/ 의 잠정 수치라 실측으로 반드시 덮어써야 합니다.
    wheelbase_m/track_m/rws_ratio 는 STM32 CONS(2)/CONS(3)/CONS(1) 과
    **같은 값**이어야 명령과 실제 궤적이 일치합니다.
    """

    def __init__(
        self,
        wheelbase_m=0.9116,          # CONS(2) B  [m] (STM32 는 mm)
        track_m=1.0,                 # CONS(3) T  [m] (전체 윤거)
        rws_ratio=0.0,               # CONS(1) RWS_percent / 100
        wheel_radius_m=0.103,
        gear_ratio=1.0,              # 모터 rpm / 휠 rpm
        max_steer_deg=30.0,
        straight_angle_deg=2.0,      # CONS(4) 조향 데드밴드
        crab_rpm_scale=1.0,          # CONS(10)
        zero_turn_rpm_scale=1.0,     # CONS(11)
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
# uart_protocol.md v2 와 mcu_bridge.py 의 CMD_FMT 에 대응. 단독 도구
# (uart_teleop.py)가 ROS 없이도 같은 바이트를 만들 수 있도록 여기 둡니다.
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
    v_inner = abs(wz) * params._icr_distance_to_inner_front(d)

    # ★ 부호반전: STM32 가 normalDriveMode(-steer_deg, ...) 로 호출하므로
    #   좌회전(wz>0)을 원하면 음수를 보내야 한다.
    steer_deg = -math.copysign(math.degrees(d), wz)
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

    for rws in (0.0, 0.3, 0.6):
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
                # 정방향: 보낸 각도로 ICR 을 다시 계산해 vx/wz 복원
                d = math.radians(abs(steer_deg))
                icr_y = p._icr_y(d)
                wz_hat = math.copysign(abs(vx) / icr_y, -steer_deg)
                err = abs(wz_hat - wz)
                if err > 1e-4:
                    ok = False
                    print(f"  FAIL rws={rws} vx={vx} wz={wz}: wz_hat={wz_hat:.5f}")
                # 좌회전이면 음수를 보내야 한다
                if wz > 0 and steer_deg > 0:
                    ok = False
                    print(f"  FAIL 부호: wz={wz} -> steer={steer_deg}")
    print("[normal] 역산<->정방향 일치 및 부호규약:", "OK" if ok else "FAIL")

    p = FourWISParams()
    # 직진
    s, rpm, m, _ = encode(0.3, 0.0, 0.0, "normal", False, p)
    straight_ok = (s == 0.0 and m == MODE_NORMAL
                   and abs(rpm - mps_to_rpm(0.3, p)) < 1e-6)
    print("[straight] steer=0, rpm=vx 환산:", "OK" if straight_ok else "FAIL")

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

    # 프레임이 uart_protocol.md 의 예시와 바이트 단위로 같아야 한다.
    # (mcu_bridge.py 의 CMD_FMT 경로와 이 도구가 갈라지지 않도록 고정)
    expect = bytes.fromhex("AA55010A000048410000964301018EDD")
    got = build_command_frame(12.5, 300.0, 1, 0x01)
    frame_ok = got == expect
    if not frame_ok:
        print("  기대:", expect.hex(" ").upper())
        print("  실제:", got.hex(" ").upper())
    print("[frame] 문서 예시 프레임과 바이트 일치:", "OK" if frame_ok else "FAIL")

    return ok and straight_ok and stop_ok and map_ok and frame_ok


if __name__ == "__main__":
    import sys
    sys.exit(0 if _selftest() else 1)
