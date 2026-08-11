#!/usr/bin/env python3
"""keyboard_teleop - ROS 를 거쳐 수동 조작하는 키보드 텔레옵.

시리얼 포트를 직접 열지 않는다. twist 를 /cmd_vel_teleop 로 발행하고,
cmd_arbiter 에게 동작권을 요청/반납하면 자율주행과 동일한 경로
(cmd_arbiter -> command_manager -> mcu_bridge -> UART)로 STM32 까지 내려간다.
4WIS 변환/안전 게이팅은 command_manager 한 곳에서만 한다(여기선 twist 만 쏜다).

동작권을 쥔 동안 zero twist 라도 rate 로 계속 발행 -> arbiter 하트비트 역할.
프로세스가 죽으면 arbiter 가 정지 유지(HELD)하므로 폭주하지 않는다.

키:
  t  동작권 잡기(teleop)      r  동작권 반납(auto)
  w/s  전진/후진 (누르면 증가, 떼면 0 으로 서서히 감속)
  a/d  좌/우 회전 (누르면 증가, 유지)     j/l  좌/우 게걸음(crab, 유지)
  q  전부 0 으로 서서히 감속(소프트 스톱)   z  즉시 0 리셋
  space  비상정지 ON          c  비상정지 해제
  1  normal   3  crab   4  spin   0  auto   (drive_mode 선택)
  x 또는 Ctrl-C  종료 (종료 시 0 twist 발행; 동작권 쥔 상태면 arbiter 가 정지 유지)

⚠ 바퀴가 실제로 돕니다. 먼저 동작권(t)을 잡아야 명령이 반영됩니다.
"""

import select
import sys
import termios

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String

from alm_msgs.srv import SetControlOwner

MODE_KEYS = {"1": "normal", "3": "crab", "4": "spin", "0": "auto"}


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameters(
            namespace="",
            parameters=[
                ("cmd_vel_teleop_topic", "/cmd_vel_teleop"),
                ("drive_mode_teleop_topic", "/drive_mode_teleop"),
                ("estop_topic", "/emergency_stop"),
                ("owner_topic", "/cmd_arbiter/owner"),
                ("set_owner_service", "/cmd_arbiter/set_owner"),
                ("rate_hz", 20.0),
                ("step_v", 0.05),
                ("step_w", 0.05),
                ("max_vx", 0.45),
                ("max_vy", 0.30),
                # normal(Ackermann) 조향 포화 방지용 상한. 포화경계 wz_sat=vx/R_min
                # (R_min≈2.08m) 이므로 최대 vx(0.45)에서도 안 넘도록 0.18 로 제한.
                # ※ vx 가 낮으면 경계도 낮아져 여전히 포화될 수 있음(수동 조작 한계).
                ("max_wz", 0.18),
                # w/s 를 뗐을 때 vx 가 0 으로 돌아가는 감속률 [m/s per s]
                ("decay_vx", 0.7),
                # q(소프트 스톱) 로 전부 0 으로 줄일 때의 감속률
                ("brake_lin", 1.5),   # [m/s per s]  (vx, vy)
                ("brake_ang", 2.5),   # [rad/s per s] (wz)
            ],
        )
        g = self.get_parameter
        self.rate = max(1.0, float(g("rate_hz").value))
        self.step_v = float(g("step_v").value)
        self.step_w = float(g("step_w").value)
        self.max_vx = float(g("max_vx").value)
        self.max_vy = float(g("max_vy").value)
        self.max_wz = float(g("max_wz").value)
        self.decay_vx = float(g("decay_vx").value)
        self.brake_lin = float(g("brake_lin").value)
        self.brake_ang = float(g("brake_ang").value)

        self.vx = self.vy = self.wz = 0.0
        self.mode = "normal"
        self.owner = "?"
        self.braking = False        # q: 전부 0 으로 감속 중
        self._last_t = None

        self.cmd_pub = self.create_publisher(Twist, g("cmd_vel_teleop_topic").value, 10)
        self.mode_pub = self.create_publisher(String, g("drive_mode_teleop_topic").value, 10)
        self.estop_pub = self.create_publisher(Bool, g("estop_topic").value, 10)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, g("owner_topic").value, self._on_owner, latched)

        self.cli = self.create_client(SetControlOwner, g("set_owner_service").value)

        # 터미널을 즉시입력 모드로: 캐노니컬(줄단위) + 에코 끔.
        # 에코를 꺼야 누른 글자가 화면에 찍히지 않고 상태줄만 깔끔히 갱신된다.
        # ISIG 는 남겨 Ctrl-C(SIGINT) 로 항상 빠져나올 수 있게 한다.
        self.stdin_ok = sys.stdin.isatty()
        if self.stdin_ok:
            self.fd = sys.stdin.fileno()
            self.old_term = termios.tcgetattr(self.fd)
            new = termios.tcgetattr(self.fd)
            new[3] &= ~(termios.ICANON | termios.ECHO)   # lflags
            new[6][termios.VMIN] = 0
            new[6][termios.VTIME] = 0
            termios.tcsetattr(self.fd, termios.TCSANOW, new)
        else:
            self.get_logger().warn("stdin 이 터미널이 아닙니다 - 키 입력 없이 하트비트만 발행")

        self.timer = self.create_timer(1.0 / self.rate, self._tick)
        self._print_help()

    # ------------------------------------------------------------------ keys
    def _read_keys(self):
        if not self.stdin_ok:
            return
        while select.select([sys.stdin], [], [], 0)[0]:
            c = sys.stdin.read(1)
            if c in ("x", "\x03"):
                raise KeyboardInterrupt
            elif c == "t":
                self._request_owner("teleop")
            elif c == "r":
                self._request_owner("auto")
            elif c == "w":
                self.braking = False
                self.vx = min(self.vx + self.step_v, self.max_vx)
            elif c == "s":
                self.braking = False
                self.vx = max(self.vx - self.step_v, -self.max_vx)
            elif c == "a":
                self.braking = False
                self.wz = min(self.wz + self.step_w, self.max_wz)
            elif c == "d":
                self.braking = False
                self.wz = max(self.wz - self.step_w, -self.max_wz)
            elif c == "j":
                self.braking = False
                self.vy = min(self.vy + self.step_v, self.max_vy)
            elif c == "l":
                self.braking = False
                self.vy = max(self.vy - self.step_v, -self.max_vy)
            elif c == "q":
                self.braking = True          # 전부 0 으로 서서히 감속
            elif c == "z":
                self.vx = self.vy = self.wz = 0.0
                self.braking = False
            elif c == " ":
                self.vx = self.vy = self.wz = 0.0
                self.braking = False
                self.estop_pub.publish(Bool(data=True))
                self.get_logger().warn("비상정지 ON (/emergency_stop=true)")
            elif c == "c":
                self.estop_pub.publish(Bool(data=False))
                self.get_logger().info("비상정지 해제 (/emergency_stop=false)")
            elif c in MODE_KEYS:
                self.mode = MODE_KEYS[c]
                self.mode_pub.publish(String(data=self.mode))

    def _request_owner(self, owner):
        if not self.cli.service_is_ready():
            self.get_logger().warn(
                f"cmd_arbiter set_owner 서비스 대기중 - '{owner}' 요청 보류")
            return
        # 동작권 잡을 때 현재 drive_mode 를 함께 알려 arbiter 가 즉시 반영하게 함
        if owner == "teleop":
            self.mode_pub.publish(String(data=self.mode))
        req = SetControlOwner.Request()
        req.owner = owner
        fut = self.cli.call_async(req)
        fut.add_done_callback(self._on_owner_resp)

    def _on_owner_resp(self, fut):
        try:
            res = fut.result()
        except Exception as e:                       # noqa: BLE001
            self.get_logger().error(f"set_owner 실패: {e}")
            return
        if res.success:
            self.get_logger().info(f"동작권 -> {res.active_owner}")
        else:
            self.get_logger().warn(f"set_owner 거부: {res.message}")

    def _on_owner(self, msg):
        self.owner = msg.data

    @staticmethod
    def _toward(cur, target, delta):
        """cur 를 target 방향으로 delta 만큼(넘지 않게) 이동."""
        if cur > target:
            return max(cur - delta, target)
        if cur < target:
            return min(cur + delta, target)
        return target

    # ------------------------------------------------------------------ loop
    def _tick(self):
        self._read_keys()

        now = self.get_clock().now().nanoseconds * 1e-9
        dt = (now - self._last_t) if self._last_t is not None else 1.0 / self.rate
        dt = max(1e-3, min(dt, 0.2))
        self._last_t = now

        if self.braking:
            # q: 모든 선속도/각속도를 0 으로 서서히 감속
            self.vx = self._toward(self.vx, 0.0, self.brake_lin * dt)
            self.vy = self._toward(self.vy, 0.0, self.brake_lin * dt)
            self.wz = self._toward(self.wz, 0.0, self.brake_ang * dt)
        else:
            # w/s(linear.x) 는 키를 떼면 0 으로 자연 감속(momentary).
            # 누르는 동안엔 키 반복(bump)이 감속을 앞질러 목표까지 올라간다.
            # a/d(wz), j/l(vy) 는 유지(감속 없음).
            self.vx = self._toward(self.vx, 0.0, self.decay_vx * dt)

        t = Twist()
        t.linear.x = self.vx
        t.linear.y = self.vy
        t.angular.z = self.wz
        self.cmd_pub.publish(t)
        if self.stdin_ok:
            flag = " [BRAKE]" if self.braking else ""
            sys.stdout.write(
                f"\x1b[2K\rowner={self.owner:<12} mode={self.mode:<6} "
                f"vx={self.vx:+.2f} vy={self.vy:+.2f} wz={self.wz:+.2f}{flag}")
            sys.stdout.flush()

    def _print_help(self):
        print(__doc__.split("키:")[1].split("⚠")[0])
        print(f"{self.rate:.0f}Hz 발행 중. 먼저 't' 로 동작권을 잡으세요.\n")

    # --------------------------------------------------------------- cleanup
    def shutdown(self):
        try:
            self.cmd_pub.publish(Twist())      # 마지막 0 twist
        except Exception:                        # noqa: BLE001
            pass
        if self.stdin_ok:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_term)
            print("\n\n종료. 동작권을 쥐고 있었다면 arbiter 가 정지 유지(HELD) 합니다.\n"
                  "자율 복귀: ros2 service call /cmd_arbiter/set_owner "
                  "alm_msgs/srv/SetControlOwner \"{owner: auto}\"")


def main():
    rclpy.init()
    node = KeyboardTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
