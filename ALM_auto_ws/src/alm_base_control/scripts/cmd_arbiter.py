#!/usr/bin/env python3
"""cmd_arbiter - 자율주행/수동 텔레옵 사이의 '동작권(ownership)' 중재기.

시리얼 포트로 내려가는 최종 경로는 하나뿐이다(mcu_bridge 가 유일한 문지기).
그 앞단에서 twist/drive_mode 를 누가 소유할지 이 노드가 정한다.

  Nav2 ─/cmd_vel───────┐
   (수동) ─/drive_mode──┤
                        ▼
  teleop ─/cmd_vel_teleop──▶ [cmd_arbiter] ─/cmd_vel_mux──▶ command_manager ─▶ mcu_bridge ─UART─▶ STM32
         ─/drive_mode_teleop─▶     │        ─/drive_mode_mux─▶
  web ───/direct_drive_web──▶      │        ─/direct_drive_mux─▶
         ─서비스 set_owner ────────┘        └─/cmd_arbiter/owner (latched)

동작권 상태:
  AUTO   : 기본. Nav 소스(/cmd_vel, /drive_mode)를 통과시킨다. 나머지 무시.
  TELEOP : set_owner("teleop") 로 진입. 키보드 텔레옵의 twist 를 통과시킨다.
  WEB    : set_owner("web") 로 진입. 웹의 **직접 rpm/조향각**을 통과시킨다.
           이 구간에는 /cmd_vel_mux 로 0 twist 만 나간다 (twist 경로를 닫는다).
  HELD   : TELEOP/WEB 중 명령(하트비트)이 *_timeout_sec 넘게 끊긴 상태.
           그대로 '정지 유지'하며 자율로 자동 복귀하지 않는다.
           set_owner("auto") 를 명시적으로 호출해야만 AUTO 로 돌아간다.
           ★ WEB 의 HELD 는 rpm 만 0 으로 하고 **조향각은 유지**한다 —
             _tick_web 주석 참고.

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

from alm_msgs.msg import DirectDrive
from alm_msgs.srv import SetControlOwner

# web 은 앞의 둘과 **입력 종류가 다르다**. auto/teleop 은 twist 를 내고,
# web 은 rpm + 조향각을 직접 낸다(DirectDrive). 그래서 web 은 teleop 의
# /cmd_vel_teleop 를 나눠 쓰지 않고 자기 토픽을 갖는다 — 같은 칸을 공유하면
# '지금 명령을 낸 게 누구인가' 가 마지막 발행자로 결정되고, 그건 동작권이라는
# 개념을 무너뜨린다.
OWNERS = ("auto", "teleop", "web")


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
                # 웹 수동주행 (직접 rpm/조향각)
                ("direct_web_topic", "/direct_drive_web"),
                # ---- 출력 토픽 (command_manager 입력) ----
                ("cmd_vel_out_topic", "/cmd_vel_mux"),
                ("drive_mode_out_topic", "/drive_mode_mux"),
                ("direct_out_topic", "/direct_drive_mux"),
                ("owner_topic", "/cmd_arbiter/owner"),
                # ---- 동작 ----
                ("default_owner", "auto"),
                ("publish_rate_hz", 50.0),
                ("nav_timeout_sec", 0.5),       # nav twist 가 이 시간 끊기면 0
                ("teleop_timeout_sec", 0.5),    # teleop twist 끊기면 HELD(0, 유지)
                ("web_timeout_sec", 0.5),       # 웹 직접명령 끊기면 HELD(rpm 0, 조향 유지)
            ],
        )
        g = self.get_parameter
        self.default_owner = self._valid_owner(g("default_owner").value, "auto")
        self.rate = max(1.0, float(g("publish_rate_hz").value))
        self.nav_timeout = float(g("nav_timeout_sec").value)
        self.teleop_timeout = float(g("teleop_timeout_sec").value)
        self.web_timeout = float(g("web_timeout_sec").value)

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
        self.web_direct = DirectDrive()
        self.web_direct_sec = 0.0

        # ---- I/O ----
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.cmd_pub = self.create_publisher(Twist, g("cmd_vel_out_topic").value, 10)
        self.mode_pub = self.create_publisher(String, g("drive_mode_out_topic").value, latched)
        self.owner_pub = self.create_publisher(String, g("owner_topic").value, latched)

        self.create_subscription(Twist, g("cmd_vel_nav_topic").value, self._on_nav_cmd, 10)
        self.create_subscription(String, g("drive_mode_nav_topic").value, self._on_nav_mode, 10)
        self.create_subscription(Twist, g("cmd_vel_teleop_topic").value, self._on_teleop_cmd, 10)
        self.create_subscription(String, g("drive_mode_teleop_topic").value, self._on_teleop_mode, 10)
        self.direct_pub = self.create_publisher(DirectDrive, g("direct_out_topic").value, 10)
        self.create_subscription(
            DirectDrive, g("direct_web_topic").value, self._on_web_direct, 10)

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

    def _on_web_direct(self, msg):
        self.web_direct = msg
        self.web_direct_sec = self._now()

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
        elif owner == "web":
            # 인계 시점의 잔상을 지운다. 이전 세션의 마지막 rpm 이 남아 있으면
            # 제어권을 잡자마자 바퀴가 도는 상태가 만들어진다.
            self.web_direct = DirectDrive()
            self.web_direct_sec = self._now()
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
            elif p.name == "web_timeout_sec":
                self.web_timeout = float(p.value)
            elif p.name == "publish_rate_hz":
                new_rate = max(1.0, float(p.value))
                if new_rate != self.rate:
                    self.rate = new_rate
                    self.timer.cancel()
                    self.timer = self.create_timer(1.0 / self.rate, self._tick)
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------------ core
    def _active_mode(self):
        if self.owner == "teleop":
            return self.teleop_mode
        if self.owner == "web":
            # 웹은 mode_id 를 DirectDrive 에 실어 보내므로 drive_mode 문자열을
            # 쓰지 않는다. command_manager 가 직접명령을 보고 판단한다.
            return "direct"
        return self.nav_mode

    def _tick(self):
        now = self._now()
        out = Twist()

        # 웹(직접 rpm/조향각)은 twist 를 쓰지 않는다. 이 구간에서는 0 twist 를
        # 계속 내보내 다른 소스가 새어 들어오지 못하게 하고, 실제 명령은
        # /direct_drive_mux 로 나간다.
        if self.owner == "web":
            self._tick_web(now)
            self.cmd_pub.publish(out)
            self._publish_mode(self._active_mode(), force=False)
            return

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

    def _tick_web(self, now):
        """웹 직접명령을 그대로(신선하면) 또는 rpm 0 으로(끊기면) 흘린다.

        ★ 끊겼을 때 **조향각은 유지하고 rpm 만 0 으로** 한다. 굴러가는 중에
          조향을 0 으로 되돌리면 그 자체가 의도치 않은 조타다 — 세우는 것이
          목적인데 세우면서 방향을 틀어버리면 안 된다. 바퀴를 멈추는 것이
          정지이고, 조향은 멈춘 뒤에 사람이 다시 정한다.
        """
        fresh = (now - self.web_direct_sec) <= self.web_timeout
        out = DirectDrive()
        out.stamp = self.get_clock().now().to_msg()
        out.steer_deg = float(self.web_direct.steer_deg)
        out.mode_id = int(self.web_direct.mode_id)
        if fresh:
            out.speed_rpm = float(self.web_direct.speed_rpm)
            if self.held:
                self.held = False
                self.get_logger().info("웹 하트비트 복구 -> 조작 재개")
                self._publish_owner()
        else:
            out.speed_rpm = 0.0
            if not self.held:
                self.held = True
                self.get_logger().warn(
                    "웹 명령 끊김 -> 정지 유지(HELD). "
                    "set_owner('auto') 로만 자율 복귀.", throttle_duration_sec=2.0)
                self._publish_owner()
        self.direct_pub.publish(out)

    def _publish_mode(self, mode, force):
        if mode is None:
            return
        if force or mode != self.last_pub_mode:
            self.last_pub_mode = mode
            self.mode_pub.publish(String(data=mode))

    def _publish_owner(self):
        state = f"{self.owner}(held)" if (self.held and self.owner in ("teleop", "web")) else self.owner
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
