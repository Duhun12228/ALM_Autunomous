#!/usr/bin/env python3
"""command_manager - 흐름도 '⑤ 구동 명령'의 mode_manager 역할.

/cmd_vel(Nav2/teleop) + /drive_mode + /emergency_stop 를 받아
  1) drive_mode 해석 (auto -> normal/spin/crab 자동 선택, 참고 레포 로직 포팅)
  2) 모드별 twist 제약 (spin: 회전만, crab: 병진만, normal: 전후+회전)
  3) 안전 게이팅 (속도/가속 제한, cmd timeout 정지, e-stop,
                  MCU fault 반영, 오도메트리 워치독)
  4) 기구 응답 반영 (조향 슬루 제한, 기동 조향 정렬, 모드 전환 dwell)
을 수행하고 alm_msgs/McuCommand 를 /mcu/command 로 발행합니다.

바퀴별 조향각/속도 계산(역기구학)은 STM32 의 FourWIS_DrivingAlgorithm 이 담당하므로
여기서는 하지 않습니다. 다만 STM32 는 RC 조종기 규격(조향각 1개 + 속도 1개 + 모드)을
받으므로, 마지막에 twist -> (steer_deg, speed_rpm, mode_id) 변환을 수행해
McuCommand 에 함께 실어 보냅니다(변환 로직은 fourwis_encode.py).

--- 조향은 왜 특별한가 -------------------------------------------------------
구동 속도는 개루프여도 견딜 만합니다. 선회반경 R = ICR_y(δ) 는 **조향각만의 함수**라
속도가 20% 틀려도 경로의 '모양'은 같습니다 — 같은 원을 다른 속도로 돌 뿐이고,
그 오차는 MPPI 가 위치 피드백으로 메웁니다.

조향은 다릅니다. **조향각 오차는 곧 곡률 오차이고, 곡률 오차는 즉시 경로 이탈**입니다.
같은 길을 천천히 가는 게 아니라 다른 길로 갑니다. 그런데 STM32 업링크가 미구현이라
실측 조향각이 없습니다(uart_protocol.md v2 §State). 그래서 이 노드는

  (a) 조향 명령의 변화율을 액추에이터가 따라올 수 있는 값 이하로 제한하고,
  (b) 제한된 조향각에서 실제 나올 wz 를 역산해 cmd_vel 을 재정합하며,
  (c) 조향각을 모르는 구간(기동 직후·모드 전환 중)에는 속도를 0 으로 잡습니다.

(a) 의 슬루율을 실제 액추에이터보다 **낮게** 잡으면 액추에이터가 항상 여유롭게
따라오므로 '명령 = 실제'가 성립하고, 피드백 없이도 모델이 자명하게 맞습니다.
높게 잡는 것이 위험합니다. 실측 전에는 낮게 잡으세요 — docs/control_pipeline.md §7.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener

from alm_msgs.msg import DirectDrive, McuCommand, McuState
from alm_msgs.srv import ReleaseEstop

import fourwis_encode
import path_align


def clamp(v, lo, hi):
    return max(lo, min(v, hi))


class ConditionTimer:
    """조건이 required_duration 동안 연속 유지됐는지 판정 (참고 레포 conditionHeld)."""

    def __init__(self):
        self.active = False
        self.start = 0.0

    def held(self, condition, now_sec, required_duration):
        if not condition:
            self.active = False
            return False
        if not self.active:
            self.start = now_sec
            self.active = True
        return (now_sec - self.start) >= required_duration


class CommandManager(Node):
    def __init__(self):
        super().__init__("command_manager")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("cmd_vel_topic", "/cmd_vel"),
                ("drive_mode_topic", "/drive_mode"),
                ("estop_topic", "/emergency_stop"),
                ("command_topic", "/mcu/command"),
                ("mcu_state_topic", "/mcu/state"),
                # 직접 구동 명령 (cmd_arbiter 가 web 소유일 때만 낸다).
                # 이게 신선하면 twist 경로 전체를 건너뛴다 — 아래 _tick_direct.
                ("direct_topic", "/direct_drive_mux"),
                ("direct_timeout_sec", 0.5),
                # 직접 rpm 의 가속 한계 [rpm/s]. twist 의 max_accel_x 에 해당한다.
                # 0 이면 제한 없음(권장하지 않음 — 스텝 입력이 그대로 나간다).
                ("direct_rpm_accel", 400.0),
                # 감속은 가속보다 빨라야 한다. twist 경로가 soft_stop_decel(1.5)을
                # max_accel_x(1.0)보다 크게 잡은 것과 같은 이유다 — 세우는 쪽이
                # 늦으면 안 된다. 하트비트가 끊겨 cmd_arbiter 가 rpm 0 을 보낼 때
                # 실제로 걸리는 값이 이것이다.
                ("direct_rpm_decel", 600.0),
                ("odom_topic", "/odometry/filtered"),
                ("publish_rate_hz", 50.0),
                ("default_drive_mode", "auto"),
                ("enable_motors_on_start", True),
                ("cmd_timeout_sec", 0.5),
                # E-STOP 을 래치로 둘지. true 면 /emergency_stop 의 true 만 받고
                # false 는 무시한다 — 해제는 /emergency_stop/release 서비스로만.
                # false 로 두면 예전처럼 토픽 값을 그대로 따라간다(래치 아님).
                ("estop_latch", True),
                # 래치 해제 시 MCU 가 아직 fault/estop 을 보고 중이면 거부할지.
                ("estop_release_requires_clear_mcu", True),
                ("max_linear_x", 0.45),
                ("min_linear_x", -0.15),
                ("max_linear_y", 0.30),
                ("max_angular_z", 0.8),
                ("enable_rate_limit", True),
                ("max_accel_x", 1.0),
                ("max_accel_y", 1.0),
                ("max_accel_theta", 1.5),
                # 안전 (7) MCU fault, (8) odom 워치독
                ("stop_on_mcu_fault", True),
                ("odom_watchdog_sec", 0.5),   # 0 이면 비활성
                ("auto_crab_enabled", False),
                ("auto_spin_angular_threshold", 0.35),
                ("auto_spin_release_angular_threshold", 0.03),
                ("auto_spin_linear_threshold", 0.04),
                ("auto_spin_exit_linear_threshold", 0.10),
                ("auto_spin_max_angular_speed", 0.45),
                ("auto_spin_entry_duration_sec", 0.20),
                ("auto_spin_exit_duration_sec", 0.40),
                ("auto_mode_min_hold_sec", 0.80),
                ("auto_crab_lateral_threshold", 0.05),
                ("auto_crab_angular_threshold", 0.10),
                # ---- normal 모드 조향각 제한 ----
                # 한계값은 하드코딩하지 않는다. fourwis_encode 가 wheelbase/track/
                # rws_ratio/max_steer_deg 로 런타임에 계산한다 (단일 진실 공급원).
                ("steer_limit_enabled", True),
                # 이 속도 미만에서는 조향 한계 안에서 유효한 회전을 만들 수 없어
                # wz=0 으로 접는다. normal 모드는 제자리 회전을 못 하기 때문.
                ("steer_limit_min_vx", 0.03),
                # ---- 조향 슬루(변화율) 제한 ----
                # ##CONFIRM## **주행 중** 슬루율 상한. 아직 미실측이라 보수적 초기값.
                # 낮게 잡는 것은 안전(액추에이터가 여유롭게 따라옴 -> 명령=실제),
                # 높게 잡는 것이 위험(못 따라가는데 알 방법이 없음). 실측 전엔 낮게.
                #   45 deg/s = 0->30° 전환에 0.67 s, vx=0.45 에서 주행 0.30 m
                #   측정법: docs/control_pipeline.md §7.1 (steering_observer 상한탐색)
                # 0 이면 제한 없음(권장하지 않음 — 현재 코드의 기존 거동).
                ("max_steer_rate_deg_s", 45.0),
                # ---- 정지 상태 조향 슬루율 S_정지 [deg/s] ----
                # 주행 중과 물리적으로 다른 양이다. 정지 조향은 접지면 전체를 비틀어야
                # 해서 가장 느리다. 제어 루프에는 안 들어가고(정지 구간엔 명령 램프가
                # 없다) vx=0 dwell 두 개를 유도/검사하는 데만 쓴다.
                #   실측: ros2 run alm_bringup steer_bench.py  (command_manager 를 끄고)
                #   2026-08-20 실차 = 17.5 deg/s (최솟값 기준). base_control.yaml 참고.
                # 0 이면 미설정으로 보고 max_steer_rate_deg_s 로 대체한다(예전 거동).
                ("steer_rate_stopped_deg_s", 0.0),
                # ---- 기동 시 조향 정렬 dwell ----
                # 전원을 켠 시점의 조향각을 알 방법이 없다(STM32 업링크 없음).
                # 지난 종료 시 꺾여 있던 각도가 그대로 남아 있으므로, 주행을 허용하기
                # 전에 '직진 명령 + 정지' 를 최악 가동범위만큼 유지해 조향을 편다.
                #   > 0 : 이 시간[s] 사용
                #   = 0 : 자동 계산 (2 * max_steer_deg / S_정지)
                #         ★ 나누는 값은 max_steer_rate_deg_s 가 아니라 S_정지 다 —
                #           이 구간은 vx=0 이므로 주행 중 슬루율을 쓰면 과소평가된다.
                #   < 0 : 비활성
                ("startup_steer_align_sec", 0.0),
                # ---- 모드 전환 dwell ----
                # normal(±30°) <-> crab(90°) <-> spin(47°) 는 STM32 고정 자세가
                # 달라 전환 시 조향축이 크게 스윕한다. 그동안 굴러가면 의도와 전혀
                # 다른 방향으로 간다. mode_id 는 새 모드로 먼저 보내(STM32 가 미리
                # 조준할 수 있게) 속도만 0 으로 잡아둔다. 0 이면 비활성.
                ("mode_switch_dwell_sec", 0.5),
                # ---- 정지 등급 분리 ----
                # e-stop/MCU fault 는 즉시 정지(mode 0). 반면 cmd timeout·odom stale 은
                # 통신/센서 이슈이지 위험 이벤트가 아니므로 감속 램프로 세운다.
                # true 로 두면 예전처럼 둘 다 즉시 정지.
                ("hard_stop_on_timeout", False),
                # 감속 램프 정지에 쓸 감가속도 [m/s^2] (0 이면 max_accel_x 사용)
                ("soft_stop_decel", 1.5),
                # ---- MCU 업링크 감시 ----
                # STM32 업링크(State 프레임)는 uart_protocol.md v2 기준 '미구현'이다.
                # 업링크가 없으면 stop_on_mcu_fault 는 동작하지 않는 안전장치다.
                # 기동 후 이 시간 안에 /mcu/state 가 한 번도 안 오면 경고한다.
                ("mcu_state_expect_sec", 5.0),
                # 업링크를 '있어야 하는 것'으로 볼지. STM32 업링크는 아직 미구현이므로
                # 기본은 false — 없는 게 정상이고, 기동 시 1회만 상태를 알린다.
                # 업링크가 구현되면 true 로 바꿔라. 그러면 끊김이 경고로 잡힌다.
                ("expect_mcu_state", False),
                # ---- 경로 헤딩 정렬 기동 (ALIGN) ----
                # 전역 플래너는 R_min 원호/직선만 이어 붙이므로 제자리 회전을
                # 표현하지 못한다. 그래서 '경로가 요구하는 헤딩'과 실제 헤딩이 크게
                # 벌어지면, 스스로 spin 을 걸어 헤딩만 고치고 normal 로 복귀한다.
                # 4륜 독립조향의 제자리 회전 능력을 **명시적으로** 쓰는 유일한 경로다.
                # 자세한 설계 근거는 path_align.py 모듈 docstring.
                ("align_enabled", True),
                ("align_plan_topic", "/plan"),
                # 경로 위 어느 지점의 헤딩을 목표로 삼을지 [m]. 짧으면 노이즈에
                # 민감하고, 길면 코너를 미리 보고 과하게 돈다.
                ("align_lookahead_m", 1.0),
                # 정지 상태 전용 lookahead [m]. 0 이하면 위 값을 그대로 쓴다.
                # Hybrid-A* 경로는 **현재 로봇 헤딩에 접해서** 출발하므로,
                # 출발 시점 헤딩오차는 호각 lookahead/R_min 으로 상한이 걸린다
                # (1.0 m -> 34.9°). 즉 lookahead 1.0 m 에서는 align_enter_deg(60°)
                # 가 출발 시점에 **수학적으로 도달 불가능**하다. 정지 중에만
                # 늘려서 그 상한을 연다 (3.0 m -> 104.6°).
                # ⚠ 주행 중 값은 늘리지 말 것 — 실측 헤딩오차 분포가 전부
                #   lookahead 1.0 m 기준이라 정상 선회 오인이 늘어난다.
                ("align_lookahead_m_stopped", 0.0),
                # 진입/이탈 문턱 [deg]. 반드시 exit < enter (히스테리시스).
                ("align_enter_deg", 60.0),
                ("align_exit_deg", 15.0),
                # 진입 조건이 이만큼 연속 유지돼야 건다 [s]. 재계획 순간의 튐을 거른다.
                ("align_enter_hold_sec", 0.6),
                # 기동 최대 지속 [s]. 게이트(dwell) 구간에는 시계가 흐르지 않는다.
                ("align_max_sec", 25.0),
                # 이탈 뒤 재진입 금지 시간 [s]
                ("align_cooldown_sec", 3.0),
                # wz = clamp(kp * 남은오차, wz_min, spin_max_angular_z)
                ("align_kp", 1.2),
                ("align_wz_min", 0.15),
                # 경로 남은 길이가 이보다 짧으면 진입하지 않는다 [m].
                # 0 이면 목표 직전의 최종 자세 정렬도 ALIGN 이 담당한다.
                ("align_min_remaining_m", 0.0),
                # ---- 정지 상태 전용 진입 조건 ----
                # align_enter_deg(60°)가 높은 이유는 **주행 중** 정상 선회를
                # 오인하지 않기 위해서다(실측 헤딩오차 p90 39.8°, 30° 초과가
                # 틱의 35%). 정지 상태에는 그 위험이 없으므로 더 민감하게
                # 잡아, 잘못된 방향으로 굴러가기 전에 헤딩을 맞춘다.
                # 0 이하면 주행 중 값과 같게 두어 예전 거동을 유지한다.
                ("align_enter_deg_stopped", 0.0),
                ("align_enter_hold_sec_stopped", 0.0),
                # |명령 vx| 가 이보다 작으면 '정지' 로 본다 [m/s].
                # /Odometry.twist 는 FAST-LIO 가 안 채우므로(항상 0) 쓸 수 없다.
                # 직전 틱에 **실제로 내보낸** vx 를 쓴다 — 슬루 제한 덕에
                # 명령과 실제가 거의 같다는 것이 이 스택의 전제다.
                ("align_stopped_vx", 0.05),
                # ---- 정지 중 목표 헤딩 재래치 ----
                # ALIGN 1회가 도는 각도는 '래치각 - exit_deg' 뿐이다. 정지 중에는
                # 왕복 위험이 없으므로(제자리에서 돌기만 한다) 목표를 갱신해
                # 한 번의 spin 으로 끝까지 돈다. 근거: path_align.py __init__ 주석.
                ("align_relatch_stopped", True),
                # ---- 출발 전 목표 방위각 정렬 (pre-align) ----
                # 경로 헤딩만 보면 '벽을 보고 선 채 뒤쪽 목표' 를 못 고친다.
                # 경로가 내 헤딩에 접해서 출발하기 때문이다. 그래서 목표를 받고
                # **아직 한 번도 안 굴러간** 정지 상태에서는, 경로 대신
                # **목표까지의 직선 방위각**을 보고 한 번에 돌아선다.
                # 목표 좌표는 /plan 의 끝점에서 뽑는다 — 목표가 RViz 토픽으로
                # 오든 NavigateToPose 액션으로 오든 항상 유효하다.
                ("align_goal_bearing_enabled", True),
                ("align_goal_bearing_deg", 40.0),
                # 목표가 이보다 가까우면 방위각이 노이즈에 민감해 쓰지 않는다 [m].
                ("align_goal_bearing_min_dist_m", 1.5),
                # /plan 끝점이 이보다 많이 움직이면 '새 목표' 로 본다 [m].
                # tolerance(0.50 m) 안에서 끝점이 흔들리므로 넉넉히 잡는다.
                ("align_goal_jump_m", 1.0),
                # ★ '출발 구간' 의 정의: 목표를 받은 뒤 이동거리가 이보다 작은 동안 [m].
                #   **속도로 정의하면 안 된다.** 목표를 받으면 Nav2 가 즉시
                #   cmd_vel 을 내고, 가속 1.0 m/s² 라 0.15 s 만에 vx=0.15 가 된다.
                #   그런데 진입 지속시간이 0.15 s 이므로, '|vx| < 0.05' 로 정의하면
                #   **지속시간이 차기 전에 조건이 거짓이 되어 창이 사실상 0** 이다.
                #   (통합 시험에서 실제로 한 번도 안 걸렸다.)
                #   이동거리로 정의하면 0.30 m / 0.15 m/s = 2.0 s 의 실효 창이 생긴다.
                ("align_start_travel_m", 0.30),
                # /plan 이 이 시간 넘게 안 오면 '경로 없음'으로 본다 [s]
                ("align_plan_stale_sec", 2.0),
                # ---- 4WIS 변환 (STM32 CONS 와 반드시 일치시킬 것) ----
                # 기본값 출처: STM32 Simulink 모델 ALM07.slx 의 CONS 생성 서브시스템
                ("wheelbase_m", 1.0),          # CONS(2) 1000 mm
                ("track_m", 0.919),            # CONS(3) 919 mm
                ("rws_ratio", 0.5),            # CONS(1) 50%
                ("wheel_radius_m", 0.103),
                ("gear_ratio", 1.0),
                ("max_steer_deg", 30.0),
                ("straight_angle_deg", 2.0),   # CONS(4) ##CONFIRM##
                ("crab_rpm_scale", 0.5),       # CONS(10)
                ("zero_turn_rpm_scale", 0.6),  # CONS(11)
                ("crab_steer_sign", 1.0),
                ("spin_steer_sign", 1.0),
                ("max_rpm", 3000.0),
            ],
        )
        g = self.get_parameter
        self.max_lx = g("max_linear_x").value
        self.min_lx = g("min_linear_x").value
        self.max_ly = g("max_linear_y").value
        self.max_wz = g("max_angular_z").value
        self.rate_limit_on = bool(g("enable_rate_limit").value)
        self.acc_x = g("max_accel_x").value
        self.acc_y = g("max_accel_y").value
        self.acc_th = g("max_accel_theta").value
        self.cmd_timeout = g("cmd_timeout_sec").value
        self.rate = max(1.0, g("publish_rate_hz").value)
        self.stop_on_mcu_fault = bool(g("stop_on_mcu_fault").value)
        self.odom_watchdog = g("odom_watchdog_sec").value

        self.crab_enabled = g("auto_crab_enabled").value
        self.spin_ang_th = g("auto_spin_angular_threshold").value
        self.spin_rel_th = g("auto_spin_release_angular_threshold").value
        self.spin_lin_th = g("auto_spin_linear_threshold").value
        self.spin_exit_lin_th = g("auto_spin_exit_linear_threshold").value
        self.spin_max_wz = g("auto_spin_max_angular_speed").value
        self.spin_entry_dur = g("auto_spin_entry_duration_sec").value
        self.spin_exit_dur = g("auto_spin_exit_duration_sec").value
        self.mode_min_hold = g("auto_mode_min_hold_sec").value
        self.crab_lat_th = g("auto_crab_lateral_threshold").value
        self.crab_ang_th = g("auto_crab_angular_threshold").value

        self.desired_mode = g("default_drive_mode").value
        self.enabled = bool(g("enable_motors_on_start").value)

        # STM32 4WIS 인터페이스 변환 파라미터
        self.wis = fourwis_encode.FourWISParams(
            wheelbase_m=g("wheelbase_m").value,
            track_m=g("track_m").value,
            rws_ratio=g("rws_ratio").value,
            wheel_radius_m=g("wheel_radius_m").value,
            gear_ratio=g("gear_ratio").value,
            max_steer_deg=g("max_steer_deg").value,
            straight_angle_deg=g("straight_angle_deg").value,
            crab_rpm_scale=g("crab_rpm_scale").value,
            zero_turn_rpm_scale=g("zero_turn_rpm_scale").value,
            crab_steer_sign=g("crab_steer_sign").value,
            spin_steer_sign=g("spin_steer_sign").value,
            max_rpm=g("max_rpm").value,
        )

        # ---- normal 모드 조향각 제한 (기구학에서 유도) ----
        # R_min 은 '최대 조향각에서의 ICR 측방거리'다. 후륜 역조향(rws_ratio)이
        # 반영돼 있으므로 후륜 고정 가정으로 손계산한 값과 다르다.
        self.steer_limit_on = bool(g("steer_limit_enabled").value)
        self.steer_min_vx = float(g("steer_limit_min_vx").value)
        self.r_min = fourwis_encode.min_turn_radius(self.wis)

        # ---- 조향 슬루 제한 / dwell (기구 응답 시간을 명령에 반영) ----
        self.steer_rate = float(g("max_steer_rate_deg_s").value)
        # 정지 슬루율. 미설정(0)이면 주행 중 값으로 대체하되, 그건 정지 조향을
        # 과대평가하므로(정지가 더 느리다) dwell 이 짧아질 수 있다 — 실측 권장.
        _stop_rate = float(g("steer_rate_stopped_deg_s").value)
        self.steer_rate_stopped = _stop_rate if _stop_rate > 0.0 else self.steer_rate
        self.mode_dwell = max(0.0, float(g("mode_switch_dwell_sec").value))
        self.direct_timeout = float(g("direct_timeout_sec").value)
        self.direct_rpm_accel = max(0.0, float(g("direct_rpm_accel").value))
        self.direct_rpm_decel = max(0.0, float(g("direct_rpm_decel").value))
        _align = float(g("startup_steer_align_sec").value)
        if _align > 0.0:
            self.startup_align = _align
        elif _align < 0.0:
            self.startup_align = 0.0            # 비활성
        elif self.steer_rate_stopped > 0.0:
            # 자동: 최악은 한쪽 풀락에서 반대쪽 풀락까지 = 전체 가동범위.
            # vx=0 구간이므로 정지 슬루율로 나눈다.
            self.startup_align = (2.0 * math.degrees(self.wis.max_steer_rad)
                                  / self.steer_rate_stopped)
        else:
            self.startup_align = 0.0

        # ---- 정지 등급 ----
        self.hard_stop_on_timeout = bool(g("hard_stop_on_timeout").value)
        _decel = float(g("soft_stop_decel").value)
        self.soft_decel = _decel if _decel > 0.0 else float(self.acc_x)
        self.mcu_state_expect = float(g("mcu_state_expect_sec").value)
        self.expect_mcu_state = bool(g("expect_mcu_state").value)

        # 상태
        self.cmd = Twist()
        self.last_cmd_sec = 0.0
        # 직접 구동 (웹 수동주행). twist 와 **배타적**이다 — 아래 _tick 참조.
        self.direct = DirectDrive()
        self.last_direct_sec = 0.0
        self.out_rpm = 0.0
        self.last_direct_mode = None
        self.estop = False
        self.estop_latch = bool(g("estop_latch").value)
        self.estop_release_needs_clear = bool(g("estop_release_requires_clear_mcu").value)
        self.estop_reason = ""          # 무엇이 래치를 걸었는지 (해제 로그용)
        self.mcu_fault = False          # (7) MCU 가 보고한 fault/estop
        self.last_odom_sec = 0.0        # (8) odom 워치독
        self.have_odom = False
        self.sequence = 0
        self.out_vx = 0.0
        self.out_vy = 0.0
        self.out_wz = 0.0
        self.last_tick_sec = self._now()
        self.start_sec = self._now()

        # 조향 슬루 제한 상태. '지금까지 내보낸 조향 명령'이 곧 액추에이터의
        # 추정 위치다(슬루 제한을 액추에이터 실제 슬루율보다 낮게 잡는 한).
        # STM32 업링크가 없어 실측 조향각이 없으므로, 이 값이 유일한 조향 상태다.
        self.out_steer_deg = 0.0   # 마지막으로 명령한 조향각. twist/직접 경로가 공유한다
                                   # (경로가 바뀌어도 슬루가 이어져야 하므로)
        # /mcu/state 수신 감시 (업링크 미구현 경고용)
        self.last_state_sec = 0.0
        self.have_state = False
        self.warned_no_state = False
        # 유효 모드 전환 dwell
        self.last_effective = ""
        self.last_eff_switch_sec = self.start_sec

        # ---- ALIGN 기동 (경로 헤딩 정렬) ----
        self.align_on = bool(g("align_enabled").value)
        self.align_stale = float(g("align_plan_stale_sec").value)
        self.path_err = path_align.PathHeadingError(
            lookahead_m=float(g("align_lookahead_m").value))
        self.last_plan_sec = 0.0
        self.have_plan = False
        self.warned_no_orient = False
        self.align_active_prev = False

        def _pos_or_none(v):
            """0 이하면 '미설정' 으로 본다 — 그러면 주행 중 값이 그대로 쓰인다."""
            v = float(v)
            return v if v > 0.0 else None

        self.align_stopped_vx = abs(float(g("align_stopped_vx").value))
        self.align_look_stopped = float(g("align_lookahead_m_stopped").value)
        self.align_relatch = bool(g("align_relatch_stopped").value)
        # ---- 출발 전 목표 방위각 정렬 상태 ----
        self.bearing_on = bool(g("align_goal_bearing_enabled").value)
        self.bearing_rad = math.radians(abs(float(g("align_goal_bearing_deg").value)))
        self.bearing_min_dist = abs(float(g("align_goal_bearing_min_dist_m").value))
        self.goal_jump_m = abs(float(g("align_goal_jump_m").value))
        self.start_travel_m = abs(float(g("align_start_travel_m").value))
        self.goal_xy = None          # 직전 /plan 끝점
        # 목표 수락 후 이동거리 [m]. 이 값이 start_travel_m 미만인 동안을
        # '출발 구간' 으로 본다. 목표가 없으면 창을 닫아 둔다(inf).
        self.goal_travel = float("inf")
        self.goal_prealigned = True  # 이 목표를 이미 pre-align 했는가
        self.align_bearing_run = False   # 지금 기동이 방위각으로 진입했는가
        try:
            self.align = path_align.AlignManeuver(
                enter_deg=float(g("align_enter_deg").value),
                exit_deg=float(g("align_exit_deg").value),
                enter_dur=float(g("align_enter_hold_sec").value),
                max_sec=float(g("align_max_sec").value),
                cooldown_sec=float(g("align_cooldown_sec").value),
                kp=float(g("align_kp").value),
                wz_min=float(g("align_wz_min").value),
                wz_max=float(self.spin_max_wz),
                min_remaining_m=float(g("align_min_remaining_m").value),
                enter_deg_stopped=_pos_or_none(g("align_enter_deg_stopped").value),
                enter_dur_stopped=_pos_or_none(g("align_enter_hold_sec_stopped").value),
                relatch_stopped=self.align_relatch,
            )
        except ValueError as exc:
            self.get_logger().error(
                f"ALIGN 파라미터가 잘못됐습니다({exc}) -> 기동을 비활성화합니다.")
            self.align = None
            self.align_on = False
        # 로봇 자세는 TF(map->base_link)로 읽는다. /Odometry 는 odom 프레임이라
        # map 프레임인 /plan 과 직접 비교하면 map->odom 오프셋만큼 통째로 틀어진다.
        if self.align_on:
            self.tf_buf = Buffer()
            self.tf_listener = TransformListener(self.tf_buf, self)
        else:
            self.tf_buf = None

        # auto 상태머신
        self.last_auto_mode = ""
        self.last_switch_sec = 0.0
        self.spin_entry = ConditionTimer()
        self.spin_exit = ConditionTimer()

        # I/O
        self.pub = self.create_publisher(McuCommand, g("command_topic").value, 10)
        self.eff_pub = self.create_publisher(String, "/drive_mode/effective", 10)
        self.create_subscription(Twist, g("cmd_vel_topic").value, self._on_cmd, 10)
        self.create_subscription(String, g("drive_mode_topic").value, self._on_mode, 10)
        self.create_subscription(Bool, g("estop_topic").value, self._on_estop, 10)
        self.create_subscription(McuState, g("mcu_state_topic").value, self._on_mcu_state, 10)
        self.create_subscription(Odometry, g("odom_topic").value, self._on_odom, 10)
        # 직접 구동. cmd_arbiter 가 web 소유일 때만 발행하므로, 이게 신선하다는
        # 것 자체가 '지금 웹이 수동으로 몰고 있다' 는 뜻이다.
        self.create_subscription(
            DirectDrive, g("direct_topic").value, self._on_direct, 10)
        if self.align_on:
            self.create_subscription(Path, g("align_plan_topic").value, self._on_plan, 10)
        # 래치 해제는 서비스로만. 토픽에 false 를 흘려서 푸는 경로는 없다.
        self.create_service(ReleaseEstop, "/emergency_stop/release", self._on_release_estop)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        self._normalize_mode()
        self.get_logger().info(
            f"command_manager 시작: default_mode={self.desired_mode}, "
            f"limits vx[{self.min_lx},{self.max_lx}] wz±{self.max_wz}, "
            f"rate_limit={'on' if self.rate_limit_on else 'off'}, "
            f"mcu_fault_stop={self.stop_on_mcu_fault}, odom_watchdog={self.odom_watchdog}s, "
            f"estop_latch={'on' if self.estop_latch else 'off'}"
        )
        if not self.estop_latch:
            self.get_logger().warn(
                "estop_latch=false — /emergency_stop 에 false 가 오면 정지가 즉시 풀립니다. "
                "웹 UI 는 래치를 전제로 동작하므로 운용에서는 true 를 권장합니다.")

        # 기구학 한계 대비 속도 제한이 타당한지 시작 시 1회 점검
        wz_max = fourwis_encode.max_angular_speed(self.wis, self.max_lx)
        self.get_logger().info(
            f"4WIS: 최소 회전반경 {self.r_min:.3f} m, "
            f"vx={self.max_lx} m/s 에서 가능한 최대 wz={wz_max:.3f} rad/s"
        )
        if self.steer_limit_on:
            self.get_logger().info(
                f"조향 제한 ON: normal 모드 |wz| <= |vx|/{self.r_min:.3f} "
                f"(내측 전륜 {math.degrees(self.wis.max_steer_rad):.0f}° 한계, "
                f"vx={self.max_lx} 에서 {wz_max:.3f} rad/s), "
                f"vx<{self.steer_min_vx} 이면 wz=0"
            )
        else:
            self.get_logger().warn(
                "조향 제한 OFF — normal 모드에서 기구가 못 내는 곡률을 요구해도 "
                "조향각만 포화되고 twist 는 그대로 나갑니다(계획≠실제). 운용에서는 true 권장.")
        if self.max_wz > wz_max * 1.05 and not self.steer_limit_on:
            self.get_logger().warn(
                f"max_angular_z({self.max_wz})가 기구학 한계({wz_max:.3f})를 초과합니다. "
                f"일반 주행에서 조향이 상시 포화되어 실제 궤적이 계획과 어긋납니다. "
                f"Nav2/base_control 의 각속도 제한을 낮추거나 spin 모드를 쓰세요."
            )

        # ---- 조향 슬루 제한 / dwell 상태 보고 ----
        if self.steer_rate > 0.0:
            full = math.degrees(self.wis.max_steer_rad)
            self.get_logger().info(
                f"조향 슬루 제한 {self.steer_rate:.0f} deg/s "
                f"(0°→{full:.0f}° 전환 {full / self.steer_rate:.2f} s, "
                f"vx={self.max_lx} 에서 주행 {self.max_lx * full / self.steer_rate:.2f} m). "
                + (f"S_정지 실측 {self.steer_rate_stopped:.1f} deg/s. "
                   f"##CONFIRM## 주행 중 슬루율은 미실측 — docs/control_pipeline.md §7.1"
                   if self.steer_rate_stopped != self.steer_rate else
                   "##CONFIRM## 실측값 아님 — docs/control_pipeline.md §7.1 참고")
            )
        else:
            self.get_logger().warn(
                "조향 슬루 제한 OFF (max_steer_rate_deg_s=0) — 출발 선회에서 조향 명령이 "
                "1500 deg/s 로 튑니다. 어떤 조향 모터도 못 따라가고, 업링크가 없어 "
                "얼마나 못 따라갔는지 알 방법도 없습니다.")
        if self.startup_align > 0.0:
            self.get_logger().info(
                f"기동 조향 정렬: {self.startup_align:.2f} s 동안 직진(0°)+정지 유지 후 주행 허용")
        else:
            self.get_logger().warn(
                "기동 조향 정렬 OFF — 전원 투입 시 조향각을 모르는 채로 출발합니다.")
        if self.mode_dwell > 0.0:
            self.get_logger().info(
                f"모드 전환 dwell {self.mode_dwell:.2f} s (전환 중 속도 0, mode_id 는 먼저 전달)")
            # 조향각 업링크가 없어 '스윕이 안 끝났다'를 관측할 방법이 없다.
            # 그래서 기구가 요구하는 시간과 설정값을 기동 시 한 번 대조해 둔다.
            # 최악 스윕: normal 반대쪽 풀락 -> spin 제로턴 자세(47°, CONS(9)).
            if self.steer_rate_stopped > 0.0:
                need = ((math.degrees(self.wis.max_steer_rad) + 47.0)
                        / self.steer_rate_stopped)
                if self.mode_dwell < need:
                    self.get_logger().warn(
                        f"모드 전환 dwell 부족: normal<->spin 최악 스윕에 {need:.2f} s 가 "
                        f"필요한데 {self.mode_dwell:.2f} s 입니다 "
                        f"(S_정지 {self.steer_rate_stopped:.1f} deg/s 기준). 조향축이 아직 "
                        f"도는 중에 바퀴가 굴러 의도와 다른 방향으로 나갈 수 있습니다.")
        if not self.hard_stop_on_timeout:
            self.get_logger().info(
                f"정지 등급 분리 ON: e-stop/MCU fault=즉시, "
                f"cmd timeout/odom stale=감속 램프({self.soft_decel:.1f} m/s²)")
        if self.align_on:
            self.get_logger().info(
                f"ALIGN(경로 헤딩 정렬) ON: {g('align_plan_topic').value} 구독, "
                f"lookahead {self.path_err.lookahead_m:.2f} m, "
                f"진입 {math.degrees(self.align.enter_rad):.0f}° "
                f"({self.align.enter_dur:.2f} s 지속) -> "
                f"이탈 {math.degrees(self.align.exit_rad):.0f}°, "
                f"최대 {self.align.max_sec:.0f} s, 쿨다운 {self.align.cooldown_sec:.0f} s. "
                f"헤딩오차가 이보다 크면 제자리 회전으로 고칩니다.")
            # ---- 출발 시 도달 가능한 헤딩오차 상한 점검 ----
            # Hybrid-A* 경로는 현재 로봇 헤딩에 **접해서** 출발하고 R_min 원호만
            # 이어 붙이므로, 출발 시점에 경로 헤딩오차가 낼 수 있는 최대치는
            # 호각 lookahead/R_min 이다. 진입 문턱이 그보다 크면 출발 시점에는
            # **수학적으로 절대 안 걸린다** — 조용히 안 걸리는 부류의 함정이라
            # 기동 시 한 번 대조해 둔다.
            look_run = self.path_err.lookahead_m
            look_stop = (self.align_look_stopped
                         if self.align_look_stopped > 0.0 else look_run)
            cap_run = math.degrees(look_run / self.r_min)
            cap_stop = math.degrees(look_stop / self.r_min)
            self.get_logger().info(
                f"  출발 시 헤딩오차 상한(= lookahead/R_min): "
                f"주행 중 {cap_run:.1f}° (lookahead {look_run:.2f} m) / "
                f"정지 중 {cap_stop:.1f}° (lookahead {look_stop:.2f} m)")
            enter_stop_deg = math.degrees(self.align.enter_rad_stopped)
            if enter_stop_deg > cap_stop:
                need = math.radians(enter_stop_deg) * self.r_min
                self.get_logger().warn(
                    f"정지 진입 문턱 {enter_stop_deg:.0f}° 가 출발 시 상한 "
                    f"{cap_stop:.1f}° 를 넘습니다 — 출발 시점에는 경로 헤딩만으로 "
                    f"ALIGN 이 걸리지 않습니다. align_lookahead_m_stopped 를 "
                    f"{need:.2f} m 이상으로 올리거나 문턱을 낮추세요.")
            if self.bearing_on:
                self.get_logger().info(
                    f"  출발 전 방위각 정렬 ON: 목표까지 "
                    f"{self.bearing_min_dist:.1f} m 이상이고 방위각오차가 "
                    f"{math.degrees(self.bearing_rad):.0f}° 를 넘으면, 아직 안 굴러간 "
                    f"동안 **목표당 1회** 제자리 회전으로 돌아섭니다. "
                    f"경로가 내 헤딩에 접해서 출발하는 한계를 이걸로 뚫습니다.")
            else:
                self.get_logger().warn(
                    "출발 전 방위각 정렬 OFF — 벽을 보고 선 채 뒤쪽 목표를 받으면 "
                    "R_min 으로 크게 감아 도는 경로가 그대로 나옵니다.")
            self.get_logger().info(
                f"  정지 중 목표 재래치 {'ON' if self.align_relatch else 'OFF'}"
                + ("" if self.align_relatch else
                   " — 한 기동이 도는 각도가 '래치각 - 이탈각' 으로 묶입니다."))
        else:
            self.get_logger().info(
                "ALIGN OFF — 전역 경로는 R_min 원호/직선만 담으므로, 제자리 회전이 "
                "필요한 목표(접근 자세가 크게 어긋난 경우)는 실패할 수 있습니다.")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_sec = self._now()

    def _on_direct(self, msg):
        self.direct = msg
        self.last_direct_sec = self._now()

    def _on_mode(self, msg):
        self.desired_mode = msg.data
        self._normalize_mode()
        self.get_logger().info(f"drive_mode -> {self.desired_mode}")

    def _on_estop(self, msg):
        """/emergency_stop 구독. 래치 모드에서는 true 만 받는다.

        false 를 무시하는 것이 요점이다. 토픽으로 풀 수 있으면 래치가 아니고,
        아무 노드나(또는 `ros2 topic pub`) false 를 한 번 흘리는 순간 정지가
        조용히 풀린다. 해제는 /emergency_stop/release 서비스 한 곳에서만.
        """
        if msg.data:
            if not self.estop:
                self.get_logger().warn("EMERGENCY STOP 활성화 (/emergency_stop)")
                self.estop_reason = "/emergency_stop"
            self.estop = True
            return
        if self.estop_latch:
            if self.estop:
                self.get_logger().warn(
                    "/emergency_stop 에 false 가 왔지만 래치 상태라 무시합니다. "
                    "해제는 /emergency_stop/release 서비스로만 가능합니다.")
            return
        if self.estop:
            self.get_logger().warn("EMERGENCY STOP 해제 (래치 비활성 모드)")
        self.estop = False
        self.estop_reason = ""

    def _on_release_estop(self, request, response):
        """/emergency_stop/release — 래치 해제 요청.

        MCU 가 아직 fault/estop 을 보고 중이면 거부한다. 소프트웨어가 물리
        상태를 앞질러 '해제됨'을 표시하면, 조작자는 풀린 줄 알고 다음 명령을
        낸다. 물리 조건을 먼저 풀게 하는 편이 맞다.
        """
        who = (request.reason or "").strip() or "(사유 미기재)"
        if not self.estop:
            response.success = True
            response.message = "이미 해제 상태입니다."
            response.latched = False
            return response
        if self.estop_release_needs_clear and self.mcu_fault:
            self.get_logger().warn(f"E-STOP 해제 거부 (MCU fault 유지 중): {who}")
            response.success = False
            response.message = (
                "MCU 가 아직 fault/estop 을 보고하고 있습니다. "
                "물리 비상정지 스위치와 MCU fault 를 먼저 해소하세요.")
            response.latched = True
            return response
        self.get_logger().warn(
            f"E-STOP 래치 해제: {who} (걸린 원인={self.estop_reason or '알 수 없음'})")
        self.estop = False
        self.estop_reason = ""
        response.success = True
        response.message = "E-STOP 래치를 해제했습니다."
        response.latched = False
        return response

    def _on_mcu_state(self, msg: McuState):
        if not self.have_state:
            self.get_logger().info("/mcu/state 수신 시작 — MCU fault 감시가 활성화됩니다.")
        self.have_state = True
        self.last_state_sec = self._now()
        fault = bool(msg.fault or msg.emergency_stop)
        if fault and not self.mcu_fault:
            self.get_logger().error(
                f"MCU fault/estop 보고 (code={msg.fault_code}) -> 정지")
        self.mcu_fault = fault

    def _check_uplink(self, now):
        """STM32 업링크(State 프레임) 유무를 기동 시 1회 보고한다.

        uart_protocol.md v2 기준 업링크는 **미구현**이다. 그래서 기본값
        expect_mcu_state=false 에서는 '없는 게 정상' 으로 보고 INFO 로 한 번만
        알린다 — 매번 WARN 을 띄우면 진짜 경고가 묻힌다.

        ⚠ 업링크가 없어도 주행은 성립한다(설계상 그렇게 만들어져 있다):
             · odom 워치독은 /Odometry(FAST-LIO)를 본다 — /wheel_odom 불필요
             · mcu_fault 는 False 로 남아 어떤 경로도 막지 않는다
             · e-stop 래치 해제도 mcu_fault=False 라 정상 동작한다
           대가는 하나다: **MCU fault 를 알 수 없다.** 그 사실만 분명히 남긴다.

        업링크가 구현되면 expect_mcu_state=true 로 바꿔라. 그러면 끊김이 WARN 이 된다.
        """
        if self.warned_no_state or self.have_state or self.mcu_state_expect <= 0.0:
            return
        if (now - self.start_sec) < self.mcu_state_expect:
            return
        self.warned_no_state = True
        if self.expect_mcu_state:
            self.get_logger().warn(
                f"/mcu/state 가 {self.mcu_state_expect:.0f} s 동안 오지 않았습니다 "
                "— 업링크를 기대하도록 설정돼 있는데(expect_mcu_state=true) 끊겼습니다. "
                "UART/펌웨어를 확인하세요.")
        else:
            self.get_logger().info(
                "/mcu/state 없음 — STM32 업링크 미구현이 전제입니다(expect_mcu_state=false). "
                "주행은 정상 동작하며, MCU fault 감시만 비활성입니다. "
                "업링크가 생기면 expect_mcu_state=true 로 바꾸세요.")

    def _on_odom(self, _msg):
        self.last_odom_sec = self._now()
        self.have_odom = True

    def _on_plan(self, msg):
        """전역 경로 수신. ALIGN 기동이 '경로가 요구하는 헤딩'을 아는 유일한 입구."""
        pts = [(ps.pose.position.x, ps.pose.position.y,
                path_align.yaw_from_quat(ps.pose.orientation.x, ps.pose.orientation.y,
                                         ps.pose.orientation.z, ps.pose.orientation.w))
               for ps in msg.poses]
        self.path_err.set_path(pts)
        self.last_plan_sec = self._now()
        self.have_plan = len(pts) >= 2
        # ---- 새 목표 감지 (출발 전 방위각 정렬용) ----
        # /goal_pose 를 구독하지 않는다. 목표는 RViz 토픽으로도, NavigateToPose
        # 액션으로도 들어올 수 있는데 /plan 의 끝점은 어느 쪽이든 항상 그 목표다.
        # 재계획해도 끝점은 그대로이므로, 크게 튀면 새 목표로 본다.
        end = self.path_err.endpoint()
        if end is not None:
            if (self.goal_xy is None
                    or math.hypot(end[0] - self.goal_xy[0],
                                  end[1] - self.goal_xy[1]) > self.goal_jump_m):
                self.goal_travel = 0.0
                self.goal_prealigned = False
                self.get_logger().info(
                    f"새 목표 감지 ({end[0]:+.2f}, {end[1]:+.2f}) "
                    f"— 출발 전 방위각 정렬 대기")
            self.goal_xy = end
        if (self.have_plan and not self.path_err.has_orientation()
                and not self.warned_no_orient):
            self.warned_no_orient = True
            self.get_logger().warn(
                "/plan 의 pose orientation 이 경로 형상과 맞지 않습니다 — 플래너가 헤딩을 "
                "안 채운 것으로 봅니다. **ALIGN 을 쉬게 둡니다**(추측해서 돌지 않습니다). "
                "경로 추종 자체는 MPPI 가 계속 합니다.")

    def _robot_pose(self):
        """map 프레임 로봇 자세 (x, y, yaw). TF 가 없으면 None."""
        if self.tf_buf is None:
            return None
        try:
            t = self.tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y,
                path_align.yaw_from_quat(q.x, q.y, q.z, q.w))

    def _normalize_mode(self):
        if self.desired_mode == "autonomous":
            self.desired_mode = "auto"
        if self.desired_mode not in ("normal", "crab", "spin", "auto"):
            self.get_logger().warn(f"알 수 없는 모드 '{self.desired_mode}', normal 로 대체")
            self.desired_mode = "normal"

    def _update_align(self, now, effective, cmd_recent, gated):
        """경로 헤딩 정렬 기동을 한 틱 돌린다.

        반환: 기동 중이면 명령할 wz [rad/s], 아니면 None.

        ##설계메모## 이 기동은 _select_auto 의 spin 과 목적이 다르다.
          · _select_auto  : Nav2 가 낸 twist 가 '제자리 회전처럼 생겼나' 사후 분류
          · _update_align : 경로가 요구하는 헤딩과 내 헤딩을 직접 비교해 **선제** 판단
        전자는 Nav2 가 제자리 회전 명령을 낼 때만 동작하는데, 전역 플래너가
        제자리 회전을 표현하지 못하므로 사실상 BT 리커버리 때만 발동했다.
        후자가 4륜 독립조향의 제자리 회전을 실제로 쓰는 경로다.
        """
        if not self.align_on or self.align is None:
            return None

        plan_fresh = (self.have_plan and self.align_stale > 0.0
                      and (now - self.last_plan_sec) <= self.align_stale)
        pose = self._robot_pose() if plan_fresh else None

        # ---- '정지' 를 두 가지로 나눈다 ----
        #   starting    : **출발 구간인가.** 목표를 받고 아직 start_travel_m
        #     (0.30 m)만큼 안 굴러갔다. 진입 문턱·lookahead 선택에 쓴다.
        #     ##왜 속도가 아니라 이동거리인가## 목표를 받으면 Nav2 가 즉시
        #     cmd_vel 을 내고 가속 1.0 m/s² 라 0.15 s 만에 vx=0.15 가 된다.
        #     '|vx| < 0.05' 로 정의하면 진입 지속시간(0.15 s)이 차기 전에 조건이
        #     거짓이 되어 **창이 사실상 0** 이다. 통합 시험에서 실제로 한 번도
        #     안 걸렸다. 이동거리로 보면 0.30/0.15 = 2.0 s 의 실효 창이 생긴다.
        #     동시에 이 정의가 '복귀 dwell 중 재진입 루프' 도 막는다 — 창은
        #     목표당 한 번만 열리고, 굴러가면 영영 닫힌다.
        #   stopped_raw : 실제로 안 움직이는가. **재래치에만** 쓴다.
        stopped_raw = abs(self.out_vx) < self.align_stopped_vx
        starting = (self.goal_travel < self.start_travel_m) and not gated

        # ---- 경로 헤딩오차 ----
        # 출발 구간에는 lookahead 를 늘려 오차 상한을 연다.
        # 근거는 path_align.PathHeadingError.evaluate docstring.
        la = (self.align_look_stopped
              if (starting and self.align_look_stopped > 0.0) else None)
        err = remaining = None
        if pose is not None:
            ev = self.path_err.evaluate(*pose, lookahead_m=la)
            if ev is not None:
                err, remaining = ev[0], ev[1]

        # ---- 출발 전 목표 방위각 정렬 (pre-align) ----
        # 경로 헤딩만 보면 '벽을 보고 선 채 뒤쪽 목표' 를 영영 못 고친다.
        # Hybrid-A* 경로가 **내 헤딩에 접해서** 출발하기 때문이다 — 즉 경로는
        # "지금 향한 쪽으로 나가서 크게 감아 돌아라" 라고만 말한다.
        # 그래서 목표를 받고 **아직 한 번도 안 굴러간** 정지 상태에서는,
        # 경로 대신 **목표까지의 직선 방위각**을 오차로 써서 한 번에 돌아선다.
        # 그 뒤 재계획하면 Hybrid-A* 가 깨끗한 경로를 낸다.
        #   ⚠ 목표당 1회뿐이다(goal_prealigned). 주행 중에 다시 서더라도
        #     쓰지 않는다 — 코너를 도는 중이면 목표 방위각이 경로와 다른 게
        #     정상이고, 거기서 방위각을 쫓으면 벽으로 향한다.
        use_bearing = False
        if (self.bearing_on and pose is not None and starting
                and not self.goal_prealigned and self.goal_xy is not None):
            dx = self.goal_xy[0] - pose[0]
            dy = self.goal_xy[1] - pose[1]
            dist = math.hypot(dx, dy)
            berr = path_align.wrap_pi(math.atan2(dy, dx) - pose[2])
            if dist >= self.bearing_min_dist and abs(berr) >= self.bearing_rad:
                err, remaining, use_bearing = berr, dist, True

        # ALIGN 을 걸어도 되는 상황인가.
        #   · auto 모드에서만. 수동 normal/spin/crab 은 조종자가 주인이다.
        #   · cmd_recent 를 쓴다(active 가 아니라). 실패 사례에서 MPPI 가 거의
        #     0 을 뱉는데, active(=속도가 유의미) 를 요구하면 **정작 필요할 때**
        #     기동이 안 걸린다. Nav2 는 목표 수행 중에만 cmd_vel 을 계속 낸다.
        eligible = bool(
            self.enabled and cmd_recent and plan_fresh
            and pose is not None and err is not None
            and self.desired_mode == "auto"
            and not self.estop and not (self.stop_on_mcu_fault and self.mcu_fault)
        )
        ryaw = pose[2] if pose is not None else 0.0
        # 재래치는 dwell 중에도 살아 있어야 한다(그때도 경로는 갱신되므로).
        # 다만 방위각으로 진입한 기동에는 걸지 않는다 — 목표가 이미 절대
        # 방위각으로 고정돼 있어 재래치가 무의미하고, 낡은 /plan 이 그 목표를
        # 흔들 여지만 만든다.
        relatch = (self.align_relatch and stopped_raw
                   and not self.align_bearing_run)
        # 방위각을 err 로 넣을 때는 그 오차에 맞는 문턱도 같이 넘긴다.
        # 안 넘기면 enter_deg_stopped(60°)가 실효 문턱이 되어 40° 설정이
        # 조용히 무시된다.
        active, wz, left = self.align.update(
            now, ryaw, err, remaining, eligible, gated,
            stopped=starting, relatch=relatch,
            enter_rad_override=(self.bearing_rad if use_bearing else None),
            enter_dur_override=(self.align.enter_dur_stopped if use_bearing else None))

        if active and not self.align_active_prev:
            self.align_bearing_run = use_bearing
            if use_bearing:
                self.goal_prealigned = True      # 이 목표에는 다시 안 쓴다
                self.get_logger().info(
                    f"ALIGN 진입(출발 전 방위각 정렬): 목표까지 방위각오차 "
                    f"{math.degrees(self.align.entry_err):+.1f}° "
                    f"-> 목표 헤딩 {math.degrees(self.align.target_yaw):+.1f}° 로 "
                    f"제자리 회전. 다 돌면 전역경로가 다시 그려집니다.")
            else:
                self.get_logger().info(
                    f"ALIGN 진입: 경로 헤딩오차 "
                    f"{math.degrees(self.align.entry_err):+.1f}° "
                    f"(lookahead {la if la is not None else self.path_err.lookahead_m:.1f} m, "
                    f"이동 {self.goal_travel:.2f} m) "
                    f"-> 목표 헤딩 {math.degrees(self.align.target_yaw):+.1f}° 로 제자리 회전")
        elif (not active) and self.align_active_prev:
            self.align_bearing_run = False
            self.get_logger().info(
                f"ALIGN 종료({self.align.last_exit_reason}): 남은 오차 "
                f"{math.degrees(left) if left is not None else float('nan'):+.1f}° "
                f"-> normal 복귀 (재래치 누적 {self.align.n_relatch}회)")
            # auto 상태머신을 normal 로 되돌려 놓는다. 안 하면 hold_active 가
            # 남아 복귀 직후 한동안 spin 에 붙들린다.
            self.last_auto_mode = "normal"
            self.last_switch_sec = now
            self.spin_entry.active = False
            self.spin_exit.active = False
        self.align_active_prev = active

        if not active:
            return None
        self.last_auto_mode = "spin"
        return wz

    def _select_auto(self, now_sec, vx, vy, wz, active):
        if not active:
            self.spin_entry.active = False
            self.spin_exit.active = False
            return "normal"

        if not self.last_auto_mode:
            self.last_auto_mode = "normal"
            self.last_switch_sec = now_sec

        hold_active = (now_sec - self.last_switch_sec) < self.mode_min_hold
        lin = math.hypot(vx, vy)
        spin_entry_cond = abs(wz) >= self.spin_ang_th and lin <= self.spin_lin_th
        spin_exit_cond = abs(wz) <= self.spin_rel_th or lin >= self.spin_exit_lin_th

        # ##설계메모## '요구 선회반경이 R_min 보다 작으면 spin 으로 보낸다'는
        # 반경 기반 라우팅을 여기 넣었다가 뺐다. 이 상태머신은 spin 을
        # '제자리 회전'(lin <= spin_lin_th)으로 정의하는데, 반경 조건은 전진 중에도
        # 참이 될 수 있어 두 정의가 충돌한다. 실측 결과 spin↔normal 이 1틱(0.02 s)
        # 단위로 왕복했고, spin 틱마다 vx 가 0 으로 눌려 가속 램프가 되감기면서
        # 정상 선회(R=1.0 m)에서도 vx 가 0.3 -> 0.04 로 무너졌다.
        #
        # 기구가 못 내는 곡률은 모드를 바꾸지 않고 _limit_normal_wz() 가
        # wz 를 접어서 처리한다. 로봇은 요청보다 크게 돌고, 그 오차는 Nav2
        # (MPPI)가 피드백으로 메운다 — 폐루프 제어기가 원래 하는 일이다.
        # 진짜 제자리 회전이 필요하면 Nav2 는 vx≈0, wz 큰 명령을 내므로
        # 아래 기존 임계값(wz>=auto_spin_angular_threshold AND lin<=...)이 잡는다.

        if self.last_auto_mode == "spin":
            self.spin_entry.active = False
            if hold_active:
                return "spin"
            if self.spin_exit.held(spin_exit_cond, now_sec, self.spin_exit_dur):
                self.spin_exit.active = False
                return "normal"
            return "spin"

        self.spin_exit.active = False
        if (not hold_active) and self.spin_entry.held(spin_entry_cond, now_sec, self.spin_entry_dur):
            self.spin_entry.active = False
            return "spin"

        if (self.crab_enabled and abs(vy) >= self.crab_lat_th
                and abs(vx) <= self.spin_lin_th and abs(wz) <= self.crab_ang_th):
            return "crab"
        return "normal"

    def _rate_limit(self, target, current, accel, dt):
        if not self.rate_limit_on:
            return target
        max_step = accel * dt
        return clamp(target, current - max_step, current + max_step)

    def _limit_normal_wz(self, vx, wz, quiet=False):
        """normal 모드에서 조향각 한계를 넘지 않는 wz 로 접는다.

        normal 모드의 조향각은 선회반경 R = |vx| / |wz| 로만 결정된다.
        기구가 낼 수 있는 최소 반경이 R_min 이므로 한계는 그대로

            |wz| <= |vx| / R_min

        이다. R_min 은 fourwis_encode.min_turn_radius() 가 wheelbase/track/
        rws_ratio/max_steer_deg 에서 계산하므로 여기에 상수를 박지 않는다.

        vx 가 아주 작으면 한계도 0 에 수렴한다 — normal 모드는 제자리 회전을
        못 하기 때문이다. 그 구간은 wz=0 으로 접는다. 진짜 제자리 회전이
        필요하면 Nav2 가 vx≈0, wz 큰 명령을 내므로 _select_auto 의 기존
        임계값이 spin 모드로 잡아준다.

        모드는 건드리지 않는다. 여기서 spin 으로 보내려 하면 상태머신의
        spin 정의(제자리 회전)와 충돌해 1틱 단위 왕복이 생긴다 —
        _select_auto 의 설계메모 참고.
        """
        if not self.steer_limit_on or not math.isfinite(self.r_min) or self.r_min <= 0.0:
            return wz
        if abs(vx) < self.steer_min_vx:
            if abs(wz) > 1e-3 and not quiet:
                self.get_logger().warn(
                    f"normal 모드 vx={vx:.3f} m/s 는 조향으로 회전을 만들기에 너무 느립니다"
                    f" -> wz={wz:.3f} 를 0 으로 접습니다. 회전이 필요하면 spin 모드로.",
                    throttle_duration_sec=2.0)
            return 0.0
        wz_lim = abs(vx) / self.r_min
        if abs(wz) > wz_lim and not quiet:
            self.get_logger().warn(
                f"조향각 한계로 wz {wz:.3f} -> {math.copysign(wz_lim, wz):.3f} rad/s "
                f"(요구반경 {abs(vx) / max(abs(wz), 1e-9):.2f} m < R_min {self.r_min:.2f} m)",
                throttle_duration_sec=2.0)
        return clamp(wz, -wz_lim, wz_lim)

    # ---- 조향 슬루 제한 + cmd_vel 재정합 -------------------------------------
    def _apply_steer_slew(self, steer_target_deg, effective, vx, dt):
        """조향 명령의 변화율을 제한하고, 그 결과로 실제 나올 wz 를 되돌려 준다.

        Returns: (steer_deg, wz_actual_or_None)

        ★ crab/spin 에는 슬루를 **걸지 않는다** (2026-08-23 수정).
        예전에는 모드와 무관하게 slew_limit 을 통과시키고 wz 재역산만
        normal 로 제한했다. 그런데 crab/zero-turn 에서 STM32 는 steer_deg 의
        **부호만** 쓴다(uart_protocol.md v2 §mode). 부호 전용 값에 변화율
        제한을 걸면 크기가 깎이는 게 아니라 **의미가 뒤집힌다**:

            직전 -30° (normal 우선회) -> spin 좌회전 요청(+)
            20 deg/s 슬루 -> -30 -> -20 -> ... -> 0 -> ... -> +30 (3.0 s)
            그 앞 1.5 s 동안 부호가 음수 = STM32 는 **반대쪽으로 조준**한다

        조향축이 반대로 갔다가 다시 오면 최악 94°(±47°) 스윕이 되어, 모드
        전환 dwell 이 벌어 준 시간을 그대로 까먹는다. 부호는 즉시 보내고,
        물리 스윕 시간은 dwell 이 담당한다 — 그게 dwell 의 존재 이유다.

        normal 모드에서만 슬루를 걸고 wz 를 재역산한다. 거기서는 steer_deg 의
        크기가 곧 실제 조향각이라 변화율 제한이 물리적으로 의미가 있다.

        ⚠ 이 함수의 전제: max_steer_rate_deg_s 가 실제 액추에이터 슬루율보다
          **낮다**. 그래야 out_steer_deg 가 실제 조향각의 좋은 추정치가 된다
          (액추에이터가 명령을 여유롭게 따라오므로 명령 = 실제).
          업링크가 없어 이 전제를 검증할 수단이 지금은 없다 —
          docs/control_pipeline.md §7 참고.
        """
        if effective != "normal":
            # 부호 전용 값 — 위 docstring 참고. 즉시 통과시킨다.
            self.out_steer_deg = float(steer_target_deg)
            return float(steer_target_deg), None
        limited = fourwis_encode.slew_limit(
            steer_target_deg, self.out_steer_deg, self.steer_rate, dt)
        self.out_steer_deg = limited
        if self.steer_rate <= 0.0:
            return limited, None
        if abs(limited - steer_target_deg) > 1e-6:
            self.get_logger().info(
                f"조향 슬루 제한: {steer_target_deg:+.1f}° 요청 -> {limited:+.1f}° 출력 "
                f"({self.steer_rate:.0f} deg/s)", throttle_duration_sec=5.0)
        return limited, fourwis_encode.wz_from_steer(limited, vx, self.wis)

    def _tick_direct(self, now, dt):
        """웹 수동주행 — rpm 과 조향각을 그대로 받아 /mcu/command 로 낸다.

        twist 경로와 무엇이 같고 무엇이 다른가:

          그대로 적용 (물리·안전이라 단위와 무관하다)
            · E-STOP 래치 / MCU fault  -> 즉시 정지 + 모터 비활성
            · 명령 타임아웃            -> cmd_arbiter 가 이미 rpm 0 으로 만들어 보낸다.
                                          여기서는 그 값이 그대로 램프를 타고 0 이 된다
            · 기동 조향 정렬 dwell     -> 전원 투입 직후 조향각을 모르는 건 여기도 같다
            · 모드 전환 dwell          -> 조향축이 새 자세로 스윕할 시간
            · 조향 슬루 제한           -> 액추에이터가 따라올 수 있는 변화율
            · rpm 가감속 제한          -> twist 의 max_accel_x 에 해당

          적용하지 않음 (직접 명령에서는 뜻이 없다)
            · twist -> rpm 변환        -> 애초에 이 경로의 존재 이유가 그것을 건너뛰는 것
            · R_min 조향각 한계        -> |wz| <= |vx|/R_min 은 twist 를 접는 규칙이다.
                                          조향각을 직접 주는데 각도를 각도로 접을 이유가 없다.
                                          기구 한계는 아래 max_steer_deg 클램프가 본다
            · auto 모드 선택 / ALIGN   -> 사람이 모드를 고르는 경로다
            · 오도메트리 워치독        -> 자율주행이 아니다. 사람이 보고 있다

        ⚠ 이 경로는 **속도를 모른다.** rpm 이 몇 m/s 인지는 wheel_radius_m ·
          gear_ratio 가 확정돼야 알 수 있고, 그게 아직 ##CONFIRM## 이다.
          그래서 max_linear_x 같은 속도 제한이 여기서는 걸리지 않는다 —
          걸 수가 없다. 대신 rpm 자체를 max_rpm 으로 자른다.
          **잭업 상태에서 먼저 확인할 것.**
        """
        direct = self.direct
        mode_id = int(direct.mode_id)
        if mode_id not in (fourwis_encode.MODE_STOP, fourwis_encode.MODE_NORMAL,
                           fourwis_encode.MODE_CRAB, fourwis_encode.MODE_ZERO_TURN):
            self.get_logger().warn(
                f"알 수 없는 mode_id {mode_id} -> 정지로 처리", throttle_duration_sec=2.0)
            mode_id = fourwis_encode.MODE_STOP

        # ---- 하드 클램프 (기구 한계) ----
        # crab(90°)/zero-turn(47°) 은 STM32 가 고정 자세를 쓰므로 우리가 보내는
        # 각도 크기는 의미가 없다. 0 으로 보내 '이 값은 안 쓰인다'를 명시한다.
        max_steer = math.degrees(self.wis.max_steer_rad)
        if mode_id == fourwis_encode.MODE_NORMAL:
            steer_target = clamp(float(direct.steer_deg), -max_steer, max_steer)
        else:
            steer_target = 0.0
        rpm_target = clamp(float(direct.speed_rpm), -self.wis.max_rpm, self.wis.max_rpm)

        # ---- 모드 전환 dwell ----
        if mode_id != self.last_direct_mode:
            if self.last_direct_mode is not None and self.mode_dwell > 0.0:
                self.get_logger().info(
                    f"직접 모드 전환 {self.last_direct_mode} -> {mode_id}: "
                    f"{self.mode_dwell:.2f} s 속도 0 유지 (조향축 재조준)")
            self.last_direct_mode = mode_id
            self.last_eff_switch_sec = now
        in_startup = (self.startup_align > 0.0
                      and (now - self.start_sec) < self.startup_align)
        in_mode_dwell = (self.mode_dwell > 0.0
                         and (now - self.last_eff_switch_sec) < self.mode_dwell)
        if in_startup:
            steer_target = 0.0
            rpm_target = 0.0
            self.get_logger().info(
                f"기동 조향 정렬 중 ({self.startup_align - (now - self.start_sec):.1f} s 남음)",
                throttle_duration_sec=1.0)
        elif in_mode_dwell:
            # 조향은 새 자세로 계속 명령하고 구동만 세운다 — dwell 의 목적이
            # '도는 동안 굴러가지 않게' 이지 '조향을 멈추게' 가 아니다.
            rpm_target = 0.0

        # ---- 정지 조건 ----
        mcu_stop = self.stop_on_mcu_fault and self.mcu_fault
        emergency_stop = bool(self.estop or mcu_stop)
        motors_on = self.enabled and not emergency_stop
        if emergency_stop:
            rpm_target = 0.0
            mode_id = fourwis_encode.MODE_STOP
            self.out_rpm = 0.0            # 램프 없이 즉시 (twist 경로와 같은 규약)
        else:
            # 0 쪽으로 가는 중이면 감속률을 쓴다. 조작자가 스로틀을 놓았거나
            # 하트비트가 끊긴 경우가 전부 여기 걸린다.
            slowing = abs(rpm_target) < abs(self.out_rpm)
            self.out_rpm = self._rate_limit(
                rpm_target, self.out_rpm,
                self.direct_rpm_decel if slowing else self.direct_rpm_accel, dt)

        # ---- 조향 슬루 ----
        # 굴러갈 때와 정지 상태의 슬루율이 물리적으로 다르다(정지가 느리다).
        # rpm 이 0 인지로 판정한다 — 직접 경로에는 vx 가 없다.
        rate = (self.steer_rate if abs(self.out_rpm) > 1e-3
                else self.steer_rate_stopped)
        steer_deg = fourwis_encode.slew_limit(
            steer_target, self.out_steer_deg, rate, dt)
        if abs(steer_deg - steer_target) > 1e-6:
            self.get_logger().info(
                f"조향 슬루 제한: {steer_target:+.1f}° 요청 -> {steer_deg:+.1f}° 출력 "
                f"({rate:.0f} deg/s)", throttle_duration_sec=5.0)
        self.out_steer_deg = steer_deg

        # twist 경로로 돌아갔을 때 가속 램프가 이어지도록 상태를 비워 둔다.
        # (직접 주행 중에 out_vx 가 예전 값으로 남아 있으면, 자율로 넘긴 순간
        #  그 속도에서 시작하는 것처럼 램프가 계산된다)
        self.out_vx = self.out_vy = self.out_wz = 0.0

        # ---- 발행 ----
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        out = McuCommand()
        out.stamp = self.get_clock().now().to_msg()
        out.sequence = self.sequence
        # cmd_vel 은 **비운다.** rpm -> m/s 환산이 ##CONFIRM## 상수에 걸려 있어서,
        # 여기서 숫자를 채우면 '측정하려는 값을 미확정 상수로 되돌려 계산한' 것이
        # 화면에 실측처럼 표시된다. 모르는 것은 비워 두는 편이 정직하다.
        out.drive_mode = "direct"
        out.steer_deg = steer_deg
        out.speed_rpm = self.out_rpm if motors_on else 0.0
        out.mode_id = mode_id if motors_on else fourwis_encode.MODE_STOP
        out.enable_motors = motors_on
        out.emergency_stop = emergency_stop
        self.pub.publish(out)

        eff = String()
        eff.data = "direct"
        self.eff_pub.publish(eff)

    def _tick(self):
        now = self._now()
        dt = clamp(now - self.last_tick_sec, 1e-3, 0.1)
        self.last_tick_sec = now
        self._check_uplink(now)
        # 목표 수락 후 이동거리. '출발 구간' 판정의 기준이다 (속도가 아니다 —
        # align_start_travel_m 주석 참고).
        self.goal_travel += abs(self.out_vx) * dt

        # ---- 직접 구동이 살아 있으면 twist 경로 전체를 건너뛴다 ----
        # 둘을 섞지 않는 이유: twist 경로는 '요청한 속도'를 기구 상수로 rpm 으로
        # 바꾸고, 직접 경로는 rpm 을 그대로 쓴다. 한 틱 안에서 둘을 합치면
        # 같은 액추에이터에 서로 다른 단위의 명령이 겹친다.
        # cmd_arbiter 가 web 을 소유한 동안에만 이 토픽이 오므로, 신선도만으로
        # 배타가 성립한다 (동작권 판정을 여기서 다시 하지 않는다 — 단일 출처).
        if (now - self.last_direct_sec) <= self.direct_timeout:
            self._tick_direct(now, dt)
            return

        cmd_recent = (now - self.last_cmd_sec) <= self.cmd_timeout
        vx = self.cmd.linear.x if cmd_recent else 0.0
        vy = self.cmd.linear.y if cmd_recent else 0.0
        wz = self.cmd.angular.z if cmd_recent else 0.0

        nonzero = math.hypot(vx, vy) > 1e-3 or abs(wz) > 1e-3
        active = cmd_recent and nonzero

        # ---- 모드 해석 ----
        effective = self.desired_mode
        if self.desired_mode == "auto":
            effective = self._select_auto(now, vx, vy, wz, active)
            if effective != self.last_auto_mode:
                self.get_logger().info(f"auto -> {effective}")
                self.last_auto_mode = effective
                self.last_switch_sec = now

        # ---- 게이트 판정 (ALIGN 보다 **먼저**) ----
        # 기동 조향 정렬 / 모드 전환 dwell 은 둘 다 '조향축이 목표 자세로 갈 시간을
        # 벌어주는' 같은 성격이다. 속도만 0 으로 잡고 mode_id/조향 명령은 그대로
        # 내보내야 STM32 가 미리 조준할 수 있다
        # (uart_protocol.md v2: drive_mode 가 명령에 실려감).
        #
        # 이 판정을 ALIGN 앞으로 끌어올린 이유: ALIGN 은 dwell 동안 어차피 못 도는데,
        # 그 시간이 기동 타임아웃을 잡아먹으면 한 번도 못 돌고 타임아웃이 난다.
        # 여기 값은 '이전 틱까지의' 유효모드 기준이며, ALIGN 이 모드를 바꾸면
        # 아래에서 다시 계산한다.
        # dwell 중 STM32 에 넘길 조향 방향(부호만 의미 있음). None = 해당 없음.
        dwell_aim = None
        in_startup = (self.startup_align > 0.0
                      and (now - self.start_sec) < self.startup_align)
        in_mode_dwell = (self.mode_dwell > 0.0
                         and (now - self.last_eff_switch_sec) < self.mode_dwell)

        # ---- ALIGN: 경로 헤딩 정렬 기동 ----
        # 전역 경로가 요구하는 헤딩에서 크게 벌어졌으면 제자리 회전으로 고친다.
        # 여기서 effective 를 spin 으로 덮으면 아래 dwell 기록이 전환을 잡아
        # 조향축(±30° -> 47°)이 자리 잡을 시간을 자동으로 확보한다.
        align_wz = self._update_align(now, effective, cmd_recent,
                                      in_startup or in_mode_dwell)
        if align_wz is not None:
            effective = "spin"
            vx = vy = 0.0
            wz = align_wz

        # 유효 모드가 바뀐 시각을 기록 (전환 dwell 판정용)
        if effective != self.last_effective:
            if self.last_effective and self.mode_dwell > 0.0:
                self.get_logger().info(
                    f"모드 전환 {self.last_effective} -> {effective}: "
                    f"{self.mode_dwell:.2f} s 속도 0 유지 (조향축 재조준)")
            self.last_effective = effective
            self.last_eff_switch_sec = now
            # 전환이 방금 일어났으므로 dwell 을 다시 판정한다.
            in_mode_dwell = self.mode_dwell > 0.0

        if in_startup:
            # 기동 정렬 중에는 조향도 0(직진)으로 몰아 편다.
            vx = vy = wz = 0.0
            effective = "normal"
            self.get_logger().info(
                f"기동 조향 정렬 중 ({self.startup_align - (now - self.start_sec):.1f} s 남음)",
                throttle_duration_sec=1.0)
        elif in_mode_dwell:
            # ★ 속도는 전부 0 으로 죽이되, **조향 방향만은 살려서 보낸다.**
            #   (dwell_aim 에 담아 두었다가 encode 뒤에 steer_deg 부호로 넣는다.
            #    아래 'dwell 중 조향 조준' 블록 참고.)
            #
            #   왜 필요한가: uart_protocol.md v2 는 crab/zero-turn 에서
            #   "steer_deg 의 **부호만** 사용하고 크기는 무시" 한다. 그런데
            #   여기서 wz/vy 를 0 으로 만들면 encode 가 steer_deg=0.0 을 내고,
            #   **0.0 에는 부호가 없다.** spin 은 좌/우에 따라 조향 자세가
            #   ±47°(CONS(9))로 정반대라, 잘못 조준하면 최악 94° 스윕이
            #   필요해진다 — dwell 5 s 를 쓰고도 목적을 달성하지 못한다.
            #   즉 '정지한 채 미리 조준한다' 는 이 dwell 의 설계 의도 자체가
            #   성립하지 않고 있었다. (시뮬에서 안 잡힌 이유는 fake_mcu 가
            #   spin/crab 을 명령 그대로 처리하기 때문. 실차 전용 결함이다.)
            if effective == "spin":
                dwell_aim = wz
            elif effective == "crab":
                dwell_aim = vy
            # ★ 전 성분을 0 으로. 예전에는 normal 일 때만 wz 를 껐는데, 그러면
            #   spin 진입 dwell 중에 **회전이 그대로 나갔다.** spin 의 회전은
            #   바퀴 구동으로 만들어지므로, 조향축이 아직 제로턴 자세(CONS(9) 47°)로
            #   가는 중에 돌면 의도와 전혀 다른 궤적이 된다.
            #   crab 도 같다(측방 병진이 vy 로 나감). dwell 의 뜻은 '아무것도 하지
            #   않고 조향축이 자리 잡을 시간을 준다' 이므로 전부 0 이 맞다.
            #   ⚠ 시뮬레이션에서는 안 잡혔다 — fake_mcu 가 normal 모드에서만
            #     실제 조향각으로 궤적을 계산하기 때문(spin/crab 은 명령 그대로).
            #     실차에서만 드러났을 버그다.
            vx = vy = wz = 0.0

        # ---- 모드별 twist 제약 ----
        if effective == "spin":
            vx, vy = 0.0, 0.0
            wz = clamp(wz, -self.spin_max_wz, self.spin_max_wz)
        elif effective == "crab":
            wz = 0.0
        else:  # normal
            vy = 0.0

        # ---- 속도 제한 ----
        vx = clamp(vx, self.min_lx, self.max_lx)
        vy = clamp(vy, -self.max_ly, self.max_ly)
        wz = clamp(wz, -self.max_wz, self.max_wz)

        # ---- normal 모드 조향각 한계 (목표 twist) ----
        # vx 클램프 뒤에 적용해야 한계가 실제로 낼 속도 기준이 된다.
        if effective == "normal":
            wz = self._limit_normal_wz(vx, wz)

        # ---- 정지 조건 — 두 등급으로 나눈다 ----
        #
        # ① emergency (즉시 정지, mode 0):  e-stop 래치 · MCU fault
        #    위험 이벤트다. 감속 램프를 태울 이유가 없다.
        #
        # ② soft (감속 램프 후 정지):       cmd timeout · odom stale
        #    통신/센서 이슈이지 위험 이벤트가 아니다. 0.45 m/s 에서 즉시 mode 0 은
        #    과하고(STM32 의 mode 0 이 coast 인지 brake 인지도 ##CONFIRM##),
        #    무엇보다 out.cmd_vel 은 램프를 그리는데 speed_rpm 은 0 이라
        #    한 메시지 안의 두 필드가 서로 다른 이야기를 하게 된다.
        #
        # hard_stop_on_timeout=true 로 두면 예전처럼 둘 다 즉시 정지.
        odom_stale = (
            self.odom_watchdog > 0.0 and active and self.have_odom
            and (now - self.last_odom_sec) > self.odom_watchdog
        )
        if odom_stale:
            self.get_logger().warn(
                "오도메트리 stale -> 정지 (EKF/센서 확인)", throttle_duration_sec=2.0)
        mcu_stop = self.stop_on_mcu_fault and self.mcu_fault

        emergency_stop = bool(self.estop or mcu_stop)
        soft_stop = bool(odom_stale or (not cmd_recent))
        if self.hard_stop_on_timeout:
            emergency_stop = emergency_stop or soft_stop
            soft_stop = False

        if emergency_stop or soft_stop:
            vx = vy = wz = 0.0

        # ---- 가속(rate) 제한 ----
        if emergency_stop:
            # 즉시 0. 램프를 태우면 cmd_vel 과 아래 encode(stopped=True) 결과가
            # 어긋난다 (전자는 감속 중, 후자는 이미 정지).
            self.out_vx = self.out_vy = self.out_wz = 0.0
        else:
            # soft_stop 이면 감속 전용 감가속도를 쓴다 (평소 가속보다 빠르게 세움)
            ax = self.soft_decel if soft_stop else self.acc_x
            ay = self.soft_decel if soft_stop else self.acc_y
            ath = self.soft_decel if soft_stop else self.acc_th
            self.out_vx = self._rate_limit(vx, self.out_vx, ax, dt)
            self.out_vy = self._rate_limit(vy, self.out_vy, ay, dt)
            self.out_wz = self._rate_limit(wz, self.out_wz, ath, dt)

        # ---- normal 모드 조향각 한계 (가속 램프 통과 후) ----
        # 가속제한은 성분별로 걸린다. wz(1.5 rad/s²)가 vx(1.0 m/s²)보다 빨리
        # 목표에 도달하므로, 출발 직후 wz/vx 비가 순간적으로 한계를 넘는다.
        # 목표 twist 에만 걸면 이 구간을 놓친다. (quiet=True — 위에서 이미 경고함)
        if effective == "normal":
            self.out_wz = self._limit_normal_wz(self.out_vx, self.out_wz, quiet=True)

        # ---- STM32 4WIS 인터페이스로 변환 ----
        # 모터 비활성/emergency 만 mode 0(정지). soft_stop 은 램프가 끝나면
        # out_vx≈0 이 되어 encode 가 알아서 정지 프레임을 만든다.
        motors_on = self.enabled and not emergency_stop
        steer_deg, speed_rpm, mode_id, note = fourwis_encode.encode(
            self.out_vx, self.out_vy, self.out_wz,
            effective, emergency_stop or not motors_on, self.wis,
        )
        if note:
            self.get_logger().warn(note, throttle_duration_sec=2.0)

        # ---- dwell 중 조향 조준 (부호만 전달) ----
        # 속도는 0 이지만 STM32 가 고정 자세를 **어느 쪽으로** 잡아야 하는지는
        # 알려야 한다. crab/zero-turn 은 부호만 보고 크기는 무시하므로
        # (uart_protocol.md v2 §mode), 여기서 부호를 넣어 주면 dwell 동안
        # 조향축이 목표 자세로 이동한다. rpm 은 0 이라 바퀴는 돌지 않는다.
        #   ##주의## normal 모드에는 적용하지 않는다. normal 의 steer_deg 는
        #   크기가 실제 조향각이라 0(직진)이 맞고, 슬루 제한도 거기 걸려 있다.
        if (dwell_aim is not None and mode_id in
                (fourwis_encode.MODE_CRAB, fourwis_encode.MODE_ZERO_TURN)
                and abs(dwell_aim) > 1e-9 and not (emergency_stop or not motors_on)):
            sign = (self.wis.spin_steer_sign if mode_id == fourwis_encode.MODE_ZERO_TURN
                    else self.wis.crab_steer_sign)
            steer_deg = sign * math.copysign(
                math.degrees(self.wis.max_steer_rad), dwell_aim)
            speed_rpm = 0.0

        # ---- 조향 슬루 제한 + cmd_vel 재정합 ----
        # encode 가 낸 조향각은 '이 twist 를 내려면 필요한 각도'다. 기구가 그
        # 각도로 순간이동하지 못하므로 변화율을 제한하고, 제한된 각도로 실제
        # 나올 wz 를 되돌려 cmd_vel 에 다시 넣는다. 이 재정합이 없으면
        # McuCommand 안에서 cmd_vel 과 steer_deg 가 서로 다른 이야기를 한다.
        steer_deg, wz_actual = self._apply_steer_slew(steer_deg, effective, self.out_vx, dt)
        if wz_actual is not None:
            self.out_wz = wz_actual
            # ★ rpm 도 **깎인 조향각 기준으로 다시** 계산해야 한다.
            #   STM32 는 받은 rpm 을 '내측 전륜' 값으로 보고 나머지 3륜을 자기가
            #   들고 있는 조향각으로 스케일한다. 목표 조향각으로 계산한 rpm 을
            #   보내면서 조향각만 깎으면 둘이 다른 선회를 가리킨다.
            #   실측: 목표 29.5° 인데 슬루로 1° 만 나간 순간 차체속도 -15.8%.
            #   (mode 0 정지 프레임은 건드리지 않는다 — 그건 rpm 0 이 맞다.)
            if mode_id == fourwis_encode.MODE_NORMAL:
                speed_rpm = fourwis_encode.rpm_for_steer(
                    steer_deg, self.out_vx, self.wis)

        # ---- McuCommand 발행 ----
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        out = McuCommand()
        out.stamp = self.get_clock().now().to_msg()
        out.sequence = self.sequence
        out.cmd_vel.linear.x = self.out_vx
        out.cmd_vel.linear.y = self.out_vy
        out.cmd_vel.angular.z = self.out_wz
        out.drive_mode = effective
        out.steer_deg = steer_deg
        out.speed_rpm = speed_rpm
        out.mode_id = mode_id
        # e-stop/MCU fault 는 모터 비활성으로 전달.
        # odom stale/cmd timeout 은 감속 정지이지 모터 차단이 아니다 —
        # 차단해 버리면 감속 램프 자체가 실행되지 않는다.
        out.enable_motors = motors_on
        out.emergency_stop = emergency_stop
        self.pub.publish(out)

        eff = String()
        eff.data = effective
        self.eff_pub.publish(eff)


def main():
    rclpy.init()
    node = CommandManager()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
