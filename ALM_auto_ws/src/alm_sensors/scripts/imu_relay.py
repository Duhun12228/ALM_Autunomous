#!/usr/bin/env python3
"""Relay Livox MID-360 built-in IMU (/livox/imu) to /imu/data for the EKF.

livox_ros_driver2 는 MID-360 내장 6축 IMU 를 sensor_msgs/Imu 로 /livox/imu 에
발행합니다. 이 IMU 는 자이로+가속도만 제공하고 방향(orientation) 추정값이 없는데,
드라이버는 orientation 을 (0,0,0,1) 단위 쿼터니언으로 채워 보냅니다. 그대로 두면
robot_localization EKF 가 가짜 orientation 을 융합해버리므로, 여기서
orientation_covariance[0] = -1 로 설정해 "orientation 없음"을 명시합니다.
(각속도 + 선가속도만 EKF 로 융합됨 → 흐름도의 '지자기 OFF · 자이로 yaw'와 동일)
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


class ImuRelay(Node):
    def __init__(self):
        super().__init__("imu_relay")
        self.declare_parameter("input_topic", "/livox/imu")
        self.declare_parameter("output_topic", "/imu/data")
        self.declare_parameter("frame_id", "")  # 빈 값이면 원본 frame 유지
        self.declare_parameter("invalidate_orientation", True)

        self.frame_id = self.get_parameter("frame_id").value
        self.invalidate = bool(self.get_parameter("invalidate_orientation").value)
        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value

        # Livox 드라이버는 best-effort 로 발행하므로 sub QoS 를 맞춤
        qos = QoSProfile(depth=20)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT

        self.pub = self.create_publisher(Imu, out_topic, 20)
        self.sub = self.create_subscription(Imu, in_topic, self.on_imu, qos)
        self.get_logger().info(f"IMU relay: {in_topic} -> {out_topic}")

    def on_imu(self, msg: Imu):
        if self.frame_id:
            msg.header.frame_id = self.frame_id
        if self.invalidate:
            # orientation 미제공 표시 (REP-145): covariance[0] = -1
            msg.orientation_covariance[0] = -1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
