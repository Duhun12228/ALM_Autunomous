#!/usr/bin/env python3
"""Livox Mid-360 point-cloud UDP parser -> /livox/lidar (PointCloud2).

각 점에 실제 per-point 상대시각("time" 필드, 초)을 붙인다. Livox 포인트 UDP
패킷 헤더에는 packet 의 timestamp(첫 점의 디바이스 시각, ns)와 time_interval
(패킷 내 점들의 시간 span, 0.1us 단위)이 들어 있으므로, 이를 이용해

    point_time = packet_timestamp + (i / dot_num) * span

를 계산하고, 프레임(publish 단위) 시작 시각을 빼서 0 ~ ~0.1s 범위의 상대시각을
만든다. FAST-LIO2(lidar_type=2, timestamp_unit=seconds)가 이 필드로 스캔
디스큐를 정확히 수행한다 — SDK CustomMsg 없이도 동일한 per-point 타이밍 확보.

출력 PointCloud2 필드: x, y, z, intensity, time (모두 float32).
x,y,z,intensity 만 읽는 소비자(Nav2 costmap, pointcloud_to_scan)는 영향 없음.
"""
import socket
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


HOST_IP = "192.168.1.5"
POINT_PORT = 56301
FRAME_ID = "livox_frame"

HEADER_SIZE = 36           # version..timestamp[8] 까지
TS_OFFSET = 28             # timestamp(uint64 ns) 위치
POINT_HIGH_SIZE = 14       # data_type 1: int32 x,y,z + reflectivity + tag
POINT_LOW_SIZE = 8         # data_type 2: int16 x,y,z + reflectivity + tag


class LivoxUdpPointCloud2(Node):
    def __init__(self):
        super().__init__("livox_udp_pointcloud2")
        self.publisher = self.create_publisher(PointCloud2, "/livox/lidar", 10)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((HOST_IP, POINT_PORT))
        self.sock.setblocking(False)

        self.points = []               # (x, y, z, intensity, time_offset_s)
        self.frame_last_ns = 0         # 프레임 마지막 점의 디바이스 시각(ns)
        self.frame_start_ns = None     # 프레임 첫 패킷의 디바이스 timestamp(ns)
        self.frame_start_ros = None    # 프레임 첫 패킷 수신 시각(ROS) -> header.stamp
        self.publish_period_ns = 100_000_000  # 0.1 s (10 Hz)

        self.timer = self.create_timer(0.001, self.poll)
        self.get_logger().info(
            f"Listening for Livox UDP point data on {HOST_IP}:{POINT_PORT} (with per-point time)"
        )

    def poll(self):
        packets = 0
        while packets < 128:
            try:
                data, _ = self.sock.recvfrom(2048)
            except BlockingIOError:
                break
            packets += 1
            self.parse_packet(data)

        # 프레임 시작(디바이스 시각) 기준 span 이 publish 주기를 넘으면 발행
        if self.points and self.frame_start_ns is not None:
            last_ns = self.frame_last_ns
            if last_ns - self.frame_start_ns >= self.publish_period_ns:
                self.publish_points()

    def parse_packet(self, packet):
        if len(packet) < HEADER_SIZE:
            return

        (_version, _length, time_interval, dot_num,
         _udp_cnt, _frame_cnt, data_type, _time_type) = struct.unpack_from("<BHHHHBBB", packet, 0)
        packet_ts = struct.unpack_from("<Q", packet, TS_OFFSET)[0]  # 첫 점의 시각(ns)
        span_ns = time_interval * 100.0                            # 0.1us -> ns

        payload = packet[HEADER_SIZE:]

        if data_type == 1:
            count = min(dot_num, len(payload) // POINT_HIGH_SIZE)
            fmt, size, scale = "<iiiBB", POINT_HIGH_SIZE, 0.001
        elif data_type == 2:
            count = min(dot_num, len(payload) // POINT_LOW_SIZE)
            fmt, size, scale = "<hhhBB", POINT_LOW_SIZE, 0.01
        else:
            return

        if count == 0:
            return

        if self.frame_start_ns is None:
            self.frame_start_ns = packet_ts
            self.frame_start_ros = self.get_clock().now()

        step = span_ns / dot_num if dot_num else 0.0
        base_off = packet_ts - self.frame_start_ns

        for i in range(count):
            x, y, z, reflectivity, _tag = struct.unpack_from(fmt, payload, i * size)
            t_off = (base_off + step * i) * 1e-9
            if t_off < 0.0:
                t_off = 0.0
            self.points.append((x * scale, y * scale, z * scale, float(reflectivity), t_off))

        # 이 패킷 마지막 점의 디바이스 시각(발행 트리거 판단용)
        self.frame_last_ns = packet_ts + span_ns

    def publish_points(self):
        header = PointCloud2().header
        header.stamp = self.frame_start_ros.to_msg()
        header.frame_id = FRAME_ID
        fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        msg = point_cloud2.create_cloud(header, fields, self.points)
        self.publisher.publish(msg)

        self.points = []
        self.frame_start_ns = None
        self.frame_start_ros = None


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
