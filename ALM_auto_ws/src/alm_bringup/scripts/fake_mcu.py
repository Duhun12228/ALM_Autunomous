#!/usr/bin/env python3
"""fake_mcu — mcu_bridge 대역. **개발 전용. 실차에서 절대 함께 띄우지 말 것.**

STM32 가 물려 있지 않은 상태에서 WebUI 연동을 검증하려고 만든 노드다.
mcu_bridge 와 같은 인터페이스(/mcu/command 구독, /mcu/state + /wheel_odom 발행)를
흉내내되 시리얼 포트는 열지 않는다. 그래서 mcu_bridge 와 동시에 띄우면
같은 토픽에 두 퍼블리셔가 생겨 상태가 뒤섞인다 — launch 에서 배타적으로 고를 것.

거동:
  - /mcu/command 를 받아 그대로 되먹인다(measured = commanded × 약간의 손실).
    command_manager → 이 노드 → /mcu/state → command_manager 의 fault 감시까지
    한 바퀴가 실제로 돈다.
  - cmd 가 command_timeout_sec 넘게 없으면 command_timeout=True 로 보고하고 정지.
  - 배터리는 천천히 닳고, fault_code 는 파라미터로 주입해 UI 경보 표시를 시험한다.
  - steer_angle 은 실물과 같이 축당 1개씩 2개([front, rear])만 채운다.
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from alm_msgs.msg import McuCommand, McuState


class FakeMcu(Node):
    def __init__(self):
        super().__init__("fake_mcu")
        g = self.declare_parameter
        g("command_topic", "/mcu/command")
        g("state_topic", "/mcu/state")
        g("odom_topic", "/wheel_odom")
        g("publish_rate_hz", 10.0)
        g("command_timeout_sec", 0.5)
        g("battery_full_voltage", 25.2)
        g("battery_empty_voltage", 21.0)
        g("battery_drain_per_min", 0.4)     # [%]
        g("wheel_radius_m", 0.103)
        g("inject_fault_code", 0)           # 0 = 정상. UI 경보 시험용
        g("inject_fault_text", "")
        # MCU '자신의' E-STOP 상태(물리 버튼/내부 래치). 명령으로 받은 estop 을
        # 되비추면 안 된다 — command_manager 는 /mcu/state 의 estop 을 fault 로
        # 보고 mcu_fault 를 세우고(_on_mcu_state), 그 mcu_fault 가 다시
        # McuCommand.emergency_stop 을 켠다. 되비추는 순간 서로가 서로를
        # 유지시켜 /emergency_stop 을 false 로 내려도 영원히 안 풀린다.
        g("inject_estop", False)            # UI 안전 스트립 시험용

        p = self.get_parameter
        self.timeout = float(p("command_timeout_sec").value)
        self.wheel_radius = float(p("wheel_radius_m").value)
        self.v_full = float(p("battery_full_voltage").value)
        self.v_empty = float(p("battery_empty_voltage").value)
        self.drain = float(p("battery_drain_per_min").value)

        self.state_pub = self.create_publisher(McuState, p("state_topic").value, 10)
        self.odom_pub = self.create_publisher(Odometry, p("odom_topic").value, 10)
        self.create_subscription(
            McuCommand, p("command_topic").value, self.on_command, 10)

        self.cmd = None
        self.cmd_stamp = None
        self.sequence = 0
        self.battery_pct = 100.0
        self.pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self.last_tick = self.get_clock().now()

        rate = max(1.0, float(p("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().warn(
            "fake_mcu 기동 — 개발 전용 더미입니다. "
            "mcu_bridge 와 동시에 실행하지 마세요.")

    def on_command(self, msg):
        self.cmd = msg
        self.cmd_stamp = self.get_clock().now()

    def _command_fresh(self):
        if self.cmd is None or self.cmd_stamp is None:
            return False
        age = (self.get_clock().now() - self.cmd_stamp).nanoseconds / 1e9
        return age <= self.timeout

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds / 1e9
        self.last_tick = now
        fresh = self._command_fresh()
        own_estop = bool(self.get_parameter("inject_estop").value)

        # 명령이 신선할 때만 움직인다. 실제 MCU 의 cmd timeout 거동과 같다.
        if fresh and not own_estop and not self.cmd.emergency_stop and self.cmd.enable_motors:
            vx = self.cmd.cmd_vel.linear.x
            vy = self.cmd.cmd_vel.linear.y
            wz = self.cmd.cmd_vel.angular.z
            steer_deg = self.cmd.steer_deg
            speed_rpm = self.cmd.speed_rpm
        else:
            vx = vy = wz = steer_deg = speed_rpm = 0.0

        # 실측은 지령을 약간 못 따라간다 (구동 손실 흉내)
        measured_vx = vx * 0.96
        measured_vy = vy * 0.96
        measured_wz = wz * 0.94

        # 데드레커닝으로 /wheel_odom 을 만든다
        self.pose[2] += measured_wz * dt
        self.pose[0] += (measured_vx * math.cos(self.pose[2])
                         - measured_vy * math.sin(self.pose[2])) * dt
        self.pose[1] += (measured_vx * math.sin(self.pose[2])
                         + measured_vy * math.cos(self.pose[2])) * dt

        if abs(measured_vx) > 1e-3 or abs(measured_wz) > 1e-3:
            self.battery_pct = max(0.0, self.battery_pct - self.drain * dt / 60.0)

        self.sequence += 1
        fault_code = int(self.get_parameter("inject_fault_code").value)

        state = McuState()
        state.stamp = now.to_msg()
        state.sequence = self.sequence
        state.measured_velocity.linear.x = measured_vx
        state.measured_velocity.linear.y = measured_vy
        state.measured_velocity.angular.z = measured_wz
        state.odom_pose.x = self.pose[0]
        state.odom_pose.y = self.pose[1]
        state.odom_pose.theta = self.pose[2]

        # 조향은 축당 1개 — [front, rear]. rear 는 후륜 고정(rws_ratio=0)이라 0.
        state.steer_angle = [math.radians(steer_deg), 0.0]

        # 바퀴 각속도 [rad/s]. 회전 성분은 좌우를 반대로 벌린다.
        base = measured_vx / self.wheel_radius if self.wheel_radius else 0.0
        differential = measured_wz * 0.5 / self.wheel_radius if self.wheel_radius else 0.0
        state.wheel_speed = [
            base - differential,  # front_left
            base + differential,  # front_right
            base - differential,  # rear_left
            base + differential,  # rear_right
        ]

        ratio = self.battery_pct / 100.0
        state.battery_voltage = self.v_empty + (self.v_full - self.v_empty) * ratio
        state.battery_current = 1.2 + abs(speed_rpm) / 3000.0 * 6.0

        commanded_estop = bool(self.cmd.emergency_stop) if self.cmd else False
        state.motors_enabled = bool(
            fresh and self.cmd and self.cmd.enable_motors
            and not commanded_estop and not own_estop)
        # 보고하는 것은 '내 estop 상태'이지 받은 명령이 아니다 (위 파라미터 주석 참고)
        state.emergency_stop = own_estop
        state.command_timeout = not fresh
        state.fault = fault_code != 0
        state.fault_code = fault_code
        state.fault_text = str(self.get_parameter("inject_fault_text").value)
        self.state_pub.publish(state)

        odom = Odometry()
        odom.header.stamp = state.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.orientation = yaw_to_quaternion(self.pose[2])
        odom.twist.twist = state.measured_velocity
        self.odom_pub.publish(odom)


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def main():
    rclpy.init()
    node = FakeMcu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
