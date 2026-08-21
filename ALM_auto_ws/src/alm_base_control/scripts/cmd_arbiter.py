#!/usr/bin/env python3
"""cmd_arbiter - 자율주행/수동 텔레옵 사이의 '동작권(ownership)' 중재기.

시리얼 포트로 내려가는 최종 경로는 하나뿐이다(mcu_bridge 가 유일한 문지기).
그 앞단에서 twist/drive_mode 를 누가 소유할지 이 노드가 정한다.

  Nav2 ─/cmd_vel───────┐
   (수동) ─/drive_mode──┤
                        ▼
  teleop ─/cmd_vel_teleop──▶ [cmd_arbiter] ─/cmd_vel_mux──▶ command_manager ─▶ mcu_bridge ─UART─▶ STM32
         ─/drive_mode_teleop─▶     │        ─/drive_mode_mux─▶
         ─서비스 set_owner ────────┘        └─/cmd_arbiter/owner (latched)

동작권 상태:
  AUTO   : 기본. Nav 소스(/cmd_vel, /drive_mode)를 통과시킨다. 텔레옵 무시.
  TELEOP : set_owner("teleop") 서비스로 진입. 텔레옵 소스를 통과시킨다.
  HELD   : TELEOP 중 텔레옵 명령(하트비트)이 teleop_timeout_sec 넘게 끊기면
           0 twist 를 계속 발행하고 그대로 '정지 유지'. 자율로 자동 복귀하지 않는다.
           set_owner("auto") 를 명시적으로 호출해야만 AUTO 로 돌아간다.

각 소스는 freshness 로 게이팅한다: 원본이 *_timeout_sec 넘게 끊기면 0 twist 를 낸다
(arbiter 가 50Hz 로 재발행하므로 command_manager 의 cmd_timeout 이 대신 못 잡아준다).

파라미터는 런타임에 `ros2 param set /cmd_arbiter <name> <value>` 로 즉시 바뀐다.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from alm_msgs.srv import SetControlOwner

OWNERS = ("auto", "teleop")


class CmdArbiter(Node):
    def __init__(self):
        super().__init__("cmd_arbiter")
        self.declare_parameters(
            namespace="",
            parameters=[
                # ---- 입력(소스) 토픽 ----
                ("cmd_vel_nav_topic", "/cmd_vel"),
                ("drive_mode_nav_topic", "/drive_mode"),
                ("cmd_vel_teleop_topic", "/cmd_vel_teleop"),
                ("drive_mode_teleop_topic", "/drive_mode_teleop"),
                # ---- 출력 토픽 (command_manager 입력) ----
                ("cmd_vel_out_topic", "/cmd_vel_mux"),
                ("drive_mode_out_topic", "/drive_mode_mux"),
                ("owner_topic", "/cmd_arbiter/owner"),
                # ---- 동작 ----
                ("default_owner", "auto"),
                ("publish_rate_hz", 50.0),
                ("nav_timeout_sec", 0.5),       # nav twist 가 이 시간 끊기면 0
                ("teleop_timeout_sec", 0.5),    # teleop twist 끊기면 HELD(0, 유지)
            ],
        )
        g = self.get_parameter
        self.default_owner = self._valid_owner(g("default_owner").value, "auto")
        self.rate = max(1.0, float(g("publish_rate_hz").value))
        self.nav_timeout = float(g("nav_timeout_sec").value)
        self.teleop_timeout = float(g("teleop_timeout_sec").value)

        # ---- 상태 ----
        self.owner = self.default_owner
        self.held = False                 # TELEOP 인데 하트비트 끊긴 상태
        self.nav_twist = Twist()
        self.nav_twist_sec = 0.0
        self.nav_mode = None
        self.teleop_twist = Twist()
        self.teleop_twist_sec = 0.0
        self.teleop_mode = None
        self.last_pub_mode = None

        # ---- I/O ----
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, g("cmd_vel_out_topic").value, 10)
        self.mode_pub = self.create_publisher(String, g("drive_mode_out_topic").value, latched)
        self.owner_pub = self.create_publisher(String, g("owner_topic").value, latched)

        self.create_subscription(Twist, g("cmd_vel_nav_topic").value, self._on_nav_cmd, 10)
        self.create_subscription(String, g("drive_mode_nav_topic").value, self._on_nav_mode, 10)
        self.create_subscription(Twist, g("cmd_vel_teleop_topic").value, self._on_teleop_cmd, 10)
        self.create_subscription(String, g("drive_mode_teleop_topic").value, self._on_teleop_mode, 10)

        self.srv = self.create_service(
            SetControlOwner, "cmd_arbiter/set_owner", self._on_set_owner)

        self.add_on_set_parameters_callback(self._on_params)
        self.timer = self.create_timer(1.0 / self.rate, self._tick)

        self._publish_owner()
        self.get_logger().info(
            f"cmd_arbiter 시작: owner={self.owner}, rate={self.rate:.0f}Hz, "
            f"nav_timeout={self.nav_timeout}s, teleop_timeout={self.teleop_timeout}s"
        )

    # ------------------------------------------------------------------ util
    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _valid_owner(self, value, fallback):
        v = str(value).strip().lower()
        if v not in OWNERS:
            self.get_logger().warn(f"알 수 없는 owner '{value}', '{fallback}' 로 대체")
            return fallback
        return v

    # ------------------------------------------------------------- callbacks
    def _on_nav_cmd(self, msg):
        self.nav_twist = msg
        self.nav_twist_sec = self._now()

    def _on_nav_mode(self, msg):
        self.nav_mode = msg.data

    def _on_teleop_cmd(self, msg):
        self.teleop_twist = msg
        self.teleop_twist_sec = self._now()

    def _on_teleop_mode(self, msg):
        self.teleop_mode = msg.data

    def _on_set_owner(self, request, response):
        owner = str(request.owner).strip().lower()
        if owner not in OWNERS:
            response.success = False
            response.message = f"owner 는 {OWNERS} 중 하나여야 합니다 (받음: '{request.owner}')"
            response.active_owner = self.owner
            return response
        if owner != self.owner:
            self.get_logger().info(f"동작권 {self.owner} -> {owner} (서비스 요청)")
        self.owner = owner
        self.held = False
        if owner == "teleop":
            # 인계 직후 한 텀은 하트비트 유예 (텔레옵이 곧 발행 시작)
            self.teleop_twist_sec = self._now()
        self._publish_owner()
        # 소유자가 바뀌면 새 소스의 최신 모드를 즉시 반영
        self._publish_mode(self._active_mode(), force=True)
        response.success = True
        response.message = "ok"
        response.active_owner = self.owner
        return response

    def _on_params(self, params):
        for p in params:
            if p.name == "default_owner":
                self.default_owner = self._valid_owner(p.value, self.default_owner)
            elif p.name == "nav_timeout_sec":
                self.nav_timeout = float(p.value)
            elif p.name == "teleop_timeout_sec":
                self.teleop_timeout = float(p.value)
            elif p.name == "publish_rate_hz":
                new_rate = max(1.0, float(p.value))
                if new_rate != self.rate:
                    self.rate = new_rate
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.rate, self._tick)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ core
    def _active_mode(self):
        return self.teleop_mode if self.owner == "teleop" else self.nav_mode

    def _tick(self):
        now = self._now()
        out = Twist()

        if self.owner == "teleop":
            fresh = (now - self.teleop_twist_sec) <= self.teleop_timeout
            if fresh:
                out = self.teleop_twist
                if self.held:
                    self.held = False
                    self.get_logger().info("텔레옵 하트비트 복구 -> 주행 재개")
                    self._publish_owner()
            else:
                # HELD: 0 twist 유지, 자율 자동복귀 없음
                if not self.held:
                    self.held = True
                    self.get_logger().warn(
                        "텔레옵 명령 끊김 -> 정지 유지(HELD). "
                        "set_owner('auto') 로만 자율 복귀.", throttle_duration_sec=2.0)
                    self._publish_owner()
        else:  # auto
            if (now - self.nav_twist_sec) <= self.nav_timeout:
                out = self.nav_twist
            # else: nav 끊김 -> 0 twist

        self.cmd_pub.publish(out)
        self._publish_mode(self._active_mode(), force=False)

    def _publish_mode(self, mode, force):
        if mode is None:
            return
        if force or mode != self.last_pub_mode:
            self.last_pub_mode = mode
            self.mode_pub.publish(String(data=mode))

    def _publish_owner(self):
        state = "teleop(held)" if (self.owner == "teleop" and self.held) else self.owner
        self.owner_pub.publish(String(data=state))


def main():
    rclpy.init()
    node = CmdArbiter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
