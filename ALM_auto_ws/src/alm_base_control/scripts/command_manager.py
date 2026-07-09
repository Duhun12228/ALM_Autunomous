#!/usr/bin/env python3
"""command_manager - 흐름도 '⑤ 구동 명령'의 mode_manager 역할.

/cmd_vel(Nav2/teleop) + /drive_mode + /emergency_stop 를 받아
  1) drive_mode 해석 (auto -> normal/spin/crab 자동 선택, 참고 레포 로직 포팅)
  2) 모드별 twist 제약 (spin: 회전만, crab: 병진만, normal: 전후+회전)
  3) 안전 게이팅 (속도/가속 제한, cmd timeout 정지, e-stop,
                  MCU fault 반영, 오도메트리 워치독)
을 수행하고 alm_msgs/McuCommand 를 /mcu/command 로 발행합니다.

실제 4WIS 바퀴별 조향각/속도 계산(역기구학)은 STM32 가 담당하므로 여기서는 하지 않고,
'해석된 twist + 유효 drive_mode' 만 MCU 로 넘깁니다.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

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
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        self._normalize_mode()
        self.get_logger().info(
            f"command_manager 시작: default_mode={self.desired_mode}, "
            f"limits vx[{self.min_lx},{self.max_lx}] wz±{self.max_wz}, "
            f"rate_limit={'on' if self.rate_limit_on else 'off'}, "
            f"mcu_fault_stop={self.stop_on_mcu_fault}, odom_watchdog={self.odom_watchdog}s"
        )

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

    def _on_odom(self, _msg):
        self.last_odom_sec = self._now()
        self.have_odom = True

    def _normalize_mode(self):
        if self.desired_mode == "autonomous":
            self.desired_mode = "auto"
        if self.desired_mode not in ("normal", "crab", "spin", "auto"):
            self.get_logger().warn(f"알 수 없는 모드 '{self.desired_mode}', normal 로 대체")
            self.desired_mode = "normal"

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
