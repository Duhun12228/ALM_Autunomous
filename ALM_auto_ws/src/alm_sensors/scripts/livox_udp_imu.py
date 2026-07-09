#!/usr/bin/env python3
"""Publish the Livox Mid-360 built-in IMU as sensor_msgs/Imu, parsed from UDP.

The alm_sensors point parser (livox_udp_pointcloud2.py) only decodes point
packets on port 56300->host 56301. The Mid-360 also streams its built-in IMU
to a separate host port (56401 by default, see MID360_config.json), which
nothing currently reads. This node binds that port and decodes the IMU packets
so FAST-LIO2 can run on the built-in IMU instead of the external E2BOX unit.

Livox SDK2 wire format (same 36-byte ethernet header as the point stream):
  header.data_type == 0 (kLivoxLidarImuData)
  payload = LivoxLidarImuRawPoint = 6 x float32:
    gyro_x, gyro_y, gyro_z   [rad/s]
    acc_x,  acc_y,  acc_z    [g]        (Mid-360: accel is in gravitational units)

This is test-only and does not modify alm_sensors.
"""

import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

G_TO_MPS2 = 9.80665
HEADER_SIZE = 36
IMU_DATA_TYPE = 0
IMU_PAYLOAD_FMT = "<6f"  # gyro xyz, acc xyz
IMU_PAYLOAD_SIZE = struct.calcsize(IMU_PAYLOAD_FMT)


class LivoxUdpImu(Node):
    def __init__(self):
        super().__init__("livox_udp_imu")
        self.declare_parameter("host_ip", "192.168.1.5")
        self.declare_parameter("imu_port", 56401)
        self.declare_parameter("imu_topic", "/livox/imu")
        self.declare_parameter("frame_id", "livox_frame")

        host_ip = self.get_parameter("host_ip").value
        imu_port = int(self.get_parameter("imu_port").value)
        self.frame_id = self.get_parameter("frame_id").value

        self.publisher = self.create_publisher(Imu, self.get_parameter("imu_topic").value, 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((host_ip, imu_port))
        self.sock.setblocking(False)
        self.timer = self.create_timer(0.001, self.poll)
        self.get_logger().info(f"Listening for Livox built-in IMU on {host_ip}:{imu_port}")

    def poll(self):
        packets = 0
        while packets < 256:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            packets += 1
            self.parse_packet(data)

    def parse_packet(self, packet):
        if len(packet) < HEADER_SIZE + IMU_PAYLOAD_SIZE:
            return
        # Header layout matches livox_udp_pointcloud2.py; we only need data_type.
        (_version, _length, _time_interval, _dot_num, _udp_cnt, _frame_cnt,
         data_type, _time_type) = struct.unpack_from("<BHHHHBBB", packet, 0)
        if data_type != IMU_DATA_TYPE:
            return

        gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z = struct.unpack_from(
            IMU_PAYLOAD_FMT, packet, HEADER_SIZE
        )

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # No orientation is provided by the raw IMU stream.
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = float(gyro_x)
        msg.angular_velocity.y = float(gyro_y)
        msg.angular_velocity.z = float(gyro_z)
        # Livox reports accel in g; convert to m/s^2 for a physically meaningful topic.
        # (FAST-LIO2 self-normalizes accel scale at init, so either unit works for SLAM.)
        msg.linear_acceleration.x = float(acc_x) * G_TO_MPS2
        msg.linear_acceleration.y = float(acc_y) * G_TO_MPS2
        msg.linear_acceleration.z = float(acc_z) * G_TO_MPS2
        self.publisher.publish(msg)


def main():
    rclpy.init()
    node = LivoxUdpImu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
