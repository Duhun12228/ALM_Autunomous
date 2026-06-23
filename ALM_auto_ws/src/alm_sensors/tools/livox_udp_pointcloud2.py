#!/usr/bin/env python3
import socket
import struct
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


HOST_IP = "192.168.1.5"
POINT_PORT = 56301
FRAME_ID = "livox_frame"

HEADER_SIZE = 36
POINT_HIGH_SIZE = 14
POINT_LOW_SIZE = 8


class LivoxUdpPointCloud2(Node):
    def __init__(self):
        super().__init__("livox_udp_pointcloud2")
        self.publisher = self.create_publisher(PointCloud2, "/livox/lidar", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((HOST_IP, POINT_PORT))
        self.sock.setblocking(False)
        self.points = []
        self.last_publish = time.monotonic()
        self.timer = self.create_timer(0.001, self.poll)
        self.get_logger().info(f"Listening for Livox UDP point data on {HOST_IP}:{POINT_PORT}")

    def poll(self):
        packets = 0
        while packets < 128:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            packets += 1
            self.parse_packet(data)

        now = time.monotonic()
        if self.points and now - self.last_publish >= 0.1:
            self.publish_points()
            self.points.clear()
            self.last_publish = now

    def parse_packet(self, packet):
        if len(packet) < HEADER_SIZE:
            return

        version, length, time_interval, dot_num, udp_cnt, frame_cnt, data_type, time_type = struct.unpack_from(
            "<BHHHHBBB", packet, 0
        )
        _ = version, length, time_interval, udp_cnt, frame_cnt, time_type
        payload = packet[HEADER_SIZE:]

        if data_type == 1:
            count = min(dot_num, len(payload) // POINT_HIGH_SIZE)
            for i in range(count):
                offset = i * POINT_HIGH_SIZE
                x, y, z, reflectivity, tag = struct.unpack_from("<iiiBB", payload, offset)
                _ = tag
                self.points.append((x * 0.001, y * 0.001, z * 0.001, float(reflectivity)))
        elif data_type == 2:
            count = min(dot_num, len(payload) // POINT_LOW_SIZE)
            for i in range(count):
                offset = i * POINT_LOW_SIZE
                x, y, z, reflectivity, tag = struct.unpack_from("<hhhBB", payload, offset)
                _ = tag
                self.points.append((x * 0.01, y * 0.01, z * 0.01, float(reflectivity)))

    def publish_points(self):
        header = self.create_header()
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg = point_cloud2.create_cloud(header, fields, self.points)
        self.publisher.publish(msg)

    def create_header(self):
        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = FRAME_ID
        return header


def main():
    rclpy.init()
    node = LivoxUdpPointCloud2()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
