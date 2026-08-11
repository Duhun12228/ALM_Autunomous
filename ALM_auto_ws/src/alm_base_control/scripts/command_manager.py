#!/usr/bin/env python3
"""command_manager - 흐름도 '⑤ 구동 명령'의 mode_manager 역할.

/cmd_vel(Nav2/teleop) + /drive_mode + /emergency_stop 를 받아
  1) drive_mode 해석 (auto -> normal/spin/crab 자동 선택, 참고 레포 로직 포팅)
  2) 모드별 twist 제약 (spin: 회전만, crab: 병진만, normal: 전후+회전)
  3) normal 모드 조향각 제한 (max_steer_angle_deg, 기본 30°)
  4) 안전 게이팅 (속도/가속 제한, cmd timeout 정지, e-stop,
                  MCU fault 반영, 오도메트리 워치독)
을 수행하고 alm_msgs/McuCommand 를 /mcu/command 로 발행합니다.

실제 바퀴별 조향각/속도 계산(역기구학)은 STM32 가 담당하므로 여기서는 하지 않고,
'해석된 twist + 유효 drive_mode' 만 MCU 로 넘깁니다.

--- normal 모드 조향각 제한 (바퀴별 30°) -------------------------------------
플랫폼: 조향 4륜 독립(바퀴마다 조향모터) + 구동 2축(front/rear 묶음).
바퀴 i (x_i, y_i) 의 속도벡터와 조향각은

    v_i = (vx - wz*y_i,  vy + wz*x_i),    δ_i = atan2(v_iy, v_ix)

⚠ docs/uart_protocol.md 의 '축 단위' 식 δ = atan2(vy + wz*axle_x, vx) 은 위 식에서
  -wz*y_i 항을 빼먹은 근사다. 선회반경이 클 때만 유효하고 제자리회전에선 y항이
  지배항이라 90° 라는 틀린 값이 나온다. 여기서는 위 전체 식을 쓴다.

normal 모드(vy=0)에서 조향각은 wz/vx 비(=선회반경 R=vx/wz)만으로 결정되고,
가장 크게 꺾이는 바퀴는 '선회 내측 앞바퀴'(y = sign(wz)*half_track) 다:

    tan δ_inner_front = wz*front_x / (vx - wz*half_track)

δ_inner_front <= 30° 조건을 wz 에 대해 풀면

    |wz| <= tan(30°)/(front_x + tan(30°)*half_track) * |vx| = 0.6420 * |vx|
    ⇔ 선회반경 R >= front_x/tan(30°) + half_track = 1.558 m

(축/중심선 기준 30° 로 잡으면 R=1.058 m 이지만 내측 앞바퀴가 47.6° 까지 꺾인다.
 바퀴별 기계한계가 30° 이므로 반드시 위 식을 써야 한다.)

이 상한을 (a) 목표 twist 에 한 번, (b) 가속제한 통과 후 '실제 나가는 값' 에 한 번
적용한다. (b)가 있어야 정지→선회 가속 구간에서 wz 가 vx 보다 빨리 올라가
순간적으로 30° 를 넘는 일이 없다 (vx 1.0 m/s² vs wz 1.5 rad/s² 이라 실제로 발생).

30° 로 낼 수 없는 급회전(R < auto_spin_route_radius)을 Nav2 가 요구하면 auto 모드가
spin 으로 전환해 제자리 회전으로 처리한다 — 클램프로 깎아내려 계속 under-steer 하는
것보다 낫다. 4륜 독립조향이므로 제자리회전은 앞바퀴 ±50.7°/뒷바퀴 ±31.1° 로
'구르면서' 가능하다(skid 아님). spin/crab 은 기계 한계를 그대로 쓰므로 이 30°
제한을 적용하지 않는다.

검증용으로 /steer_angle/command (Float32MultiArray, [FL, FR, RL, RR] deg) 를
발행하고, /mcu/state 의 실측 조향각이 한계를 넘으면 경고한다.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Float32MultiArray, String

from alm_msgs.msg import McuCommand, McuState


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
                ("odom_topic", "/odometry/filtered"),
                ("publish_rate_hz", 50.0),
                ("default_drive_mode", "auto"),
                ("enable_motors_on_start", True),
                ("cmd_timeout_sec", 0.5),
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
                # normal 모드 조향각 제한 (파일 상단 설명 참고)
                ("steer_limit_enabled", True),
                ("max_steer_angle_deg", 30.0),
                ("steer_front_x", 0.6106),      # URDF front_x (SINGLE SOURCE OF TRUTH)
                ("steer_rear_x", -0.3010),      # URDF rear_x
                ("steer_half_track", 0.500),    # URDF half_track
                ("steer_limit_min_vx", 0.03),
                ("steer_feedback_margin_deg", 5.0),
                ("auto_spin_route_radius", 0.53),
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

        # ---- normal 모드 조향각 제한 ----
        self.steer_limit_on = bool(g("steer_limit_enabled").value)
        self.steer_max_deg = g("max_steer_angle_deg").value
        self.front_x = g("steer_front_x").value
        self.rear_x = g("steer_rear_x").value
        self.half_track = g("steer_half_track").value
        # 바퀴 배치 [FL, FR, RL, RR] — 조향은 4륜 독립이므로 바퀴별로 각을 본다.
        self.wheels = ((self.front_x, +self.half_track), (self.front_x, -self.half_track),
                       (self.rear_x, +self.half_track), (self.rear_x, -self.half_track))
        # 가장 크게 꺾이는 바퀴 = 선회 내측 앞바퀴 (|front_x| > |rear_x|).
        #   tan δ = wz*front_x / (vx - wz*half_track)  →  |wz| <= steer_k*|vx|
        _t = math.tan(math.radians(self.steer_max_deg))
        self.steer_k = _t / (abs(self.front_x) + _t * self.half_track)
        self.steer_min_r = 1.0 / self.steer_k   # = front_x/tan δ + half_track
        self.steer_min_vx = g("steer_limit_min_vx").value
        # 실측 피드백 경고 문턱 (모드전환 과도구간 오경보 방지용 여유 포함)
        self.steer_fb_limit = math.radians(
            self.steer_max_deg + g("steer_feedback_margin_deg").value)
        self.spin_route_radius = g("auto_spin_route_radius").value

        self.desired_mode = g("default_drive_mode").value
        self.enabled = bool(g("enable_motors_on_start").value)

        # 상태
        self.cmd = Twist()
        self.last_cmd_sec = 0.0
        self.estop = False
        self.mcu_fault = False          # (7) MCU 가 보고한 fault/estop
        self.last_odom_sec = 0.0        # (8) odom 워치독
        self.have_odom = False
        self.sequence = 0
        self.out_vx = 0.0
        self.out_vy = 0.0
        self.out_wz = 0.0
        self.last_tick_sec = self._now()
        self.mcu_steer = None           # /mcu/state 실측 조향각 [앞축, 뒤축] rad

        # auto 상태머신
        self.last_auto_mode = ""
        self.last_switch_sec = 0.0
        self.spin_entry = ConditionTimer()
        self.spin_exit = ConditionTimer()
        # 유효모드 전환 시각 (조향각 피드백 검사의 과도구간 제외용)
        self.last_effective = ""
        self.last_eff_switch_sec = self._now()

        # I/O
        self.pub = self.create_publisher(McuCommand, g("command_topic").value, 10)
        self.eff_pub = self.create_publisher(String, "/drive_mode/effective", 10)
        self.steer_pub = self.create_publisher(
            Float32MultiArray, "/steer_angle/command", 10)
        self.create_subscription(Twist, g("cmd_vel_topic").value, self._on_cmd, 10)
        self.create_subscription(String, g("drive_mode_topic").value, self._on_mode, 10)
        self.create_subscription(Bool, g("estop_topic").value, self._on_estop, 10)
        self.create_subscription(McuState, g("mcu_state_topic").value, self._on_mcu_state, 10)
        self.create_subscription(Odometry, g("odom_topic").value, self._on_odom, 10)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        self._normalize_mode()
        self.get_logger().info(
            f"command_manager 시작: default_mode={self.desired_mode}, "
            f"limits vx[{self.min_lx},{self.max_lx}] wz±{self.max_wz}, "
            f"rate_limit={'on' if self.rate_limit_on else 'off'}, "
            f"mcu_fault_stop={self.stop_on_mcu_fault}, odom_watchdog={self.odom_watchdog}s"
        )
        if self.steer_limit_on:
            self.get_logger().info(
                f"normal 조향각 제한 {self.steer_max_deg:.1f}° "
                f"→ |wz| ≤ {self.steer_k:.4f}·|vx| (최소 선회반경 {self.steer_min_r:.3f} m). "
                f"R < {self.spin_route_radius:.2f} m 요구는 auto 모드가 spin 으로 처리. "
                f"spin/crab 은 제한 없음"
            )
        else:
            self.get_logger().warn("normal 조향각 제한이 꺼져 있음 (steer_limit_enabled=false)")

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_cmd(self, msg):
        self.cmd = msg
        self.last_cmd_sec = self._now()

    def _on_mode(self, msg):
        self.desired_mode = msg.data
        self._normalize_mode()
        self.get_logger().info(f"drive_mode -> {self.desired_mode}")

    def _on_estop(self, msg):
        if msg.data and not self.estop:
            self.get_logger().warn("EMERGENCY STOP 활성화 (/emergency_stop)")
        self.estop = bool(msg.data)

    def _on_mcu_state(self, msg: McuState):
        fault = bool(msg.fault or msg.emergency_stop)
        if fault and not self.mcu_fault:
            self.get_logger().error(
                f"MCU fault/estop 보고 (code={msg.fault_code}) -> 정지")
        self.mcu_fault = fault
        # 길이 2(축단위) / 4(바퀴별) 둘 다 받는다 — uart 프로토콜이 아직 축단위(2).
        if len(msg.steer_angle) >= 2:
            self.mcu_steer = [float(a) for a in msg.steer_angle]

    def _on_odom(self, _msg):
        self.last_odom_sec = self._now()
        self.have_odom = True

    def _normalize_mode(self):
        if self.desired_mode == "autonomous":
            self.desired_mode = "auto"
        if self.desired_mode not in ("normal", "crab", "spin", "auto"):
            self.get_logger().warn(f"알 수 없는 모드 '{self.desired_mode}', normal 로 대체")
            self.desired_mode = "normal"

    # -- normal 모드 조향각 제한 ------------------------------------------------
    def _steer_wz_limit(self, vx):
        """조향각 한계를 지키는 |wz| 상한. tan δ = wz·axle_x/vx 에서 유도."""
        return self.steer_k * abs(vx)

    def _needs_spin(self, vx, wz):
        """요구 선회반경이 spin_route_radius 미만이면 normal 로는 무리 → spin 대상.

        조향각 한계를 '조금' 넘는 정도(R 이 1.058 m 에 가까운 경우)는 클램프로
        살짝 완만하게 도는 편이 낫다. 정지→제자리회전→전진은 그보다 훨씬 급한
        회전을 요구할 때만 값어치가 있다. 기본값 0.53 m ≈ 최소선회반경의 1/2.
        """
        if self.spin_route_radius <= 0.0 or abs(wz) < 1e-6:
            return False
        return abs(vx) < self.spin_route_radius * abs(wz)

    def _limit_steer_wz(self, vx, wz, warn):
        """normal 모드 wz 를 조향각 한계 안으로 클램프. 반환: 제한된 wz."""
        if not self.steer_limit_on:
            return wz
        if abs(vx) < self.steer_min_vx:
            # 거의 정지: 조향각 한계 안에서 만들 수 있는 wz 가 사실상 0
            if warn and abs(wz) > 1e-3:
                self.get_logger().warn(
                    f"normal 모드 vx={vx:.3f} m/s 로는 조향각 {self.steer_max_deg:.0f}° "
                    f"안에서 회전 불가 → wz={wz:+.2f} 무시. auto 모드면 spin 으로 "
                    "전환되고, 수동 normal 이면 제자리회전이 필요합니다",
                    throttle_duration_sec=2.0)
            return 0.0
        lim = self._steer_wz_limit(vx)
        if abs(wz) > lim:
            if warn:
                self.get_logger().warn(
                    f"조향각 한계 클램프: wz {wz:+.3f} → {math.copysign(lim, wz):+.3f} "
                    f"(vx={vx:.3f}, 요구 R={abs(vx / wz):.2f} m < "
                    f"{self.steer_min_r:.2f} m)",
                    throttle_duration_sec=2.0)
            return clamp(wz, -lim, lim)
        return wz

    @staticmethod
    def _norm_steer(a):
        """조향각을 물리 표현 (-90°, +90°] 로 정규화.

        후진(vx<0)에서 atan2 는 150° 같은 값을 내는데, 이는 조향축을 -30° 로 꺾고
        바퀴를 역회전시키는 것과 같은 자세다 (uart_protocol.md '전/후진 부호 주의').
        조향각 한계를 따질 때는 이 정규화된 각을 봐야 한다.
        """
        if a > 0.5 * math.pi:
            return a - math.pi
        if a <= -0.5 * math.pi:
            return a + math.pi
        return a

    def _steer_angles(self, vx, vy, wz):
        """바퀴별 조향각 [rad] — [FL, FR, RL, RR].

        v_i = (vx - wz*y_i, vy + wz*x_i) 의 방향. normal(vy=0) / crab(wz=0) /
        spin(vx=vy=0) 전부 같은 식으로 나온다. spin 이면 앞 ±50.7°/뒤 ∓31.1°.
        """
        return [self._norm_steer(math.atan2(vy + wz * x, vx - wz * y))
                for x, y in self.wheels]

    def _check_steer_feedback(self, now, effective):
        """/mcu/state 실측 조향각이 한계를 넘는지 감시 (normal 모드에서만)."""
        if not (self.steer_limit_on and effective == "normal" and self.mcu_steer):
            return
        # 모드전환 직후엔 조향축이 아직 돌아오는 중 → 과도구간 제외
        if (now - self.last_eff_switch_sec) < 1.0:
            return
        angles = [self._norm_steer(a) for a in self.mcu_steer]
        worst = max(abs(a) for a in angles)
        if worst > self.steer_fb_limit:
            self.get_logger().error(
                f"실측 조향각 {math.degrees(worst):.1f}° 가 normal 한계 "
                f"{self.steer_max_deg:.0f}° 초과 "
                f"[{', '.join(f'{math.degrees(a):.1f}' for a in angles)}]° "
                "— STM32 역기구학/기계한계 확인",
                throttle_duration_sec=2.0)

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
        # 기존 조건(거의 정지 + 큰 회전) 또는 normal 로 낼 수 없는 급회전 요구
        tight = self.steer_limit_on and self._needs_spin(vx, wz)
        spin_entry_cond = abs(wz) >= self.spin_ang_th and (
            lin <= self.spin_lin_th or tight)
        spin_exit_cond = abs(wz) <= self.spin_rel_th or lin >= self.spin_exit_lin_th

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

    def _tick(self):
        now = self._now()
        dt = clamp(now - self.last_tick_sec, 1e-3, 0.1)
        self.last_tick_sec = now

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

        if effective != self.last_effective:
            self.last_effective = effective
            self.last_eff_switch_sec = now

        # ---- 모드별 twist 제약 ----
        # spin/crab 은 조향각 제한 미적용 (원래 조향각 그대로).
        #   spin 정상상태 = 앞 ±50.7° / 뒤 ∓31.1° (4륜 독립조향의 구름 제자리회전)
        #   ⚠ normal↔spin 전환 중에는 vx 가 0 으로 줄고 wz 가 오르는 과도구간에서
        #     조향각이 최대 ~88° 까지 스윕한다 (약 0.2 s). 조향 기구 가동범위가
        #     이보다 좁으면 전환 시 정지 dwell 이 필요하다 — docs/TODO 참고.
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

        # ---- normal 모드 조향각 제한 (1/2: 목표값) ----
        # 여기서 먼저 깎아야 아래 가속제한이 '도달 가능한 목표'를 향해 램프한다.
        # 불가능한 요구는 여기서만 경고 (아래 2/2 는 램프 커플링이라 정상 동작).
        if effective == "normal":
            wz = self._limit_steer_wz(vx, wz, warn=True)

        # ---- 하드 정지 조건 (e-stop / timeout / MCU fault / odom 워치독) ----
        # (8) 움직이려는데 오도메트리가 오래 끊기면 정지 (EKF/센서 사망 방지)
        odom_stale = (
            self.odom_watchdog > 0.0 and active and self.have_odom
            and (now - self.last_odom_sec) > self.odom_watchdog
        )
        if odom_stale:
            self.get_logger().warn(
                "오도메트리 stale -> 정지 (EKF/센서 확인)", throttle_duration_sec=2.0)
        mcu_stop = self.stop_on_mcu_fault and self.mcu_fault
        hard_stop = self.estop or mcu_stop or odom_stale or (not cmd_recent)
        if hard_stop:
            vx = vy = wz = 0.0

        # ---- 가속(rate) 제한 ----
        self.out_vx = self._rate_limit(vx, self.out_vx, self.acc_x, dt)
        self.out_vy = self._rate_limit(vy, self.out_vy, self.acc_y, dt)
        self.out_wz = self._rate_limit(wz, self.out_wz, self.acc_th, dt)

        # ---- normal 모드 조향각 제한 (2/2: 실제 나가는 값) ----
        # 가속제한은 성분별로 걸리므로 wz(1.5 rad/s²)가 vx(1.0 m/s²)보다 빨리 올라
        # 램프 구간에서 순간적으로 한계를 넘는다. 여기서 최종 보장한다.
        # 상한을 '내리는' 방향이라 급감속 쪽이므로 안전하다.
        if effective == "normal":
            self.out_wz = self._limit_steer_wz(self.out_vx, self.out_wz, warn=False)

        self._check_steer_feedback(now, effective)

        # ---- McuCommand 발행 ----
        self.sequence = (self.sequence + 1) & 0xFFFFFFFF
        out = McuCommand()
        out.stamp = self.get_clock().now().to_msg()
        out.sequence = self.sequence
        out.cmd_vel.linear.x = self.out_vx
        out.cmd_vel.linear.y = self.out_vy
        out.cmd_vel.angular.z = self.out_wz
        out.drive_mode = effective
        # e-stop/MCU fault/odom-stale 는 모터 비활성으로 전달
        out.enable_motors = self.enabled and not (self.estop or mcu_stop or odom_stale)
        out.emergency_stop = bool(self.estop or mcu_stop or odom_stale)
        self.pub.publish(out)

        eff = String()
        eff.data = effective
        self.eff_pub.publish(eff)

        # 검증용: 실제 내보낸 twist 로 STM32 가 만들 바퀴별 조향각 [FL,FR,RL,RR] (deg)
        self.steer_pub.publish(Float32MultiArray(data=[
            math.degrees(a) for a in
            self._steer_angles(self.out_vx, self.out_vy, self.out_wz)]))


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
