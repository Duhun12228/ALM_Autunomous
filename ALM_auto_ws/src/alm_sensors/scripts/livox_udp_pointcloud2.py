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

## 자기가림 마스크 (mask_*)
본 플랫폼은 LiDAR 뒤쪽에 적재물이 상시 올라간다. 이 점들은 센서 좌표계에
고정이라 로봇이 회전해도 같은 sector 에 남으므로, 그냥 두면
  - Scan Context: 회전해도 안 움직이는 블록이 column shift argmin 을 오염
  - ICP / Nav2 costmap: 로봇 뒤에 상시 장애물
  - 매핑: 궤적을 따라 적재물이 번져 prior map 에 기록
이 된다. 그래서 발행 직전에 (방위, 수평거리, z) 조건으로 제거한다. 여기서
빼면 /livox/lidar 하나만 보는 모든 소비자에서 동시에 사라진다.

방위는 min/max 가 아니라 center + width 로 준다 — 후방은 +-180deg 불연속에
걸쳐 있어 min/max 로 두면 래핑 버그가 난다.

거리는 3D 가 아니라 **수평거리** hypot(x,y) 로 잰다. 적재물은 위아래로 뻗은
기둥 형태라 3D 거리로 자르면 바로 윗부분이 살아남는다.

튜닝: mask_debug_topic 을 주면 '잘려나간 점'이 그 토픽으로 나간다. RViz 에
/livox/lidar 와 함께 띄워 적재물만 정확히 덮이는 값을 찾을 것.
"""
import math
import socket
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


HOST_IP = "192.168.1.5"
POINT_PORT = 56301

HEADER_SIZE = 36           # version..timestamp[8] 까지
TS_OFFSET = 28             # timestamp(uint64 ns) 위치

# data_type 1: int32 x,y,z(mm) + reflectivity + tag = 14 B
# data_type 2: int16 x,y,z(cm) + reflectivity + tag = 8 B
# (numpy 구조화 dtype 은 기본이 packed 라 패킷 레이아웃과 바이트 단위로 일치한다)
POINT_DTYPE = {
    1: np.dtype([("x", "<i4"), ("y", "<i4"), ("z", "<i4"),
                 ("refl", "u1"), ("tag", "u1")]),
    2: np.dtype([("x", "<i2"), ("y", "<i2"), ("z", "<i2"),
                 ("refl", "u1"), ("tag", "u1")]),
}
POINT_SCALE = {1: 0.001, 2: 0.01}

CLOUD_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
]
# CLOUD_FIELDS 와 바이트 레이아웃이 같아야 create_cloud 가 memoryview 로 바로
# 넘어간다 (다르면 점마다 파이썬 객체를 도는 느린 경로를 탄다).
CLOUD_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                        ("intensity", "<f4"), ("time", "<f4")])


class LivoxUdpPointCloud2(Node):
    def __init__(self):
        super().__init__("livox_udp_pointcloud2")
        self.declare_parameter("frame_id", "livox_frame")
        self.declare_parameter("mask_enable", True)
        self.declare_parameter("mask_center_deg", 180.0)   # 정후방
        self.declare_parameter("mask_width_deg", 60.0)
        self.declare_parameter("mask_max_range", 1.5)      # 수평거리 m
        self.declare_parameter("mask_min_z", -10.0)        # 기본 무제한
        self.declare_parameter("mask_max_z", 10.0)
        self.declare_parameter("mask_debug_topic", "")     # 비우면 발행 안 함

        self.frame_id = self.get_parameter("frame_id").value
        self.mask_enable = bool(self.get_parameter("mask_enable").value)
        self.mask_center = math.radians(
            float(self.get_parameter("mask_center_deg").value))
        self.mask_half_width = math.radians(
            float(self.get_parameter("mask_width_deg").value)) / 2.0
        self.mask_max_range = float(self.get_parameter("mask_max_range").value)
        self.mask_min_z = float(self.get_parameter("mask_min_z").value)
        self.mask_max_z = float(self.get_parameter("mask_max_z").value)

        self.publisher = self.create_publisher(PointCloud2, "/livox/lidar", 10)
        debug_topic = self.get_parameter("mask_debug_topic").value
        self.debug_pub = self.create_publisher(
            PointCloud2, debug_topic, 10) if debug_topic else None

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind((HOST_IP, POINT_PORT))
        self.sock.setblocking(False)

        self.frames = []               # 패킷별 CLOUD_DTYPE 배열
        self.frame_last_ns = 0         # 프레임 마지막 점의 디바이스 시각(ns)
        self.frame_start_ns = None     # 프레임 첫 패킷의 디바이스 timestamp(ns)
        self.frame_start_ros = None    # 프레임 첫 패킷 수신 시각(ROS) -> header.stamp
        self.publish_period_ns = 100_000_000  # 0.1 s (10 Hz)
        self.n_total = 0               # 마스크 통계(로그용)
        self.n_dropped = 0

        self.timer = self.create_timer(0.001, self.poll)
        self.get_logger().info(
            f"Listening for Livox UDP point data on {HOST_IP}:{POINT_PORT} (with per-point time)"
        )
        if self.mask_enable:
            self.get_logger().info(
                f"자기가림 마스크 ON: 방위 {self.get_parameter('mask_center_deg').value:.0f}"
                f"+-{self.get_parameter('mask_width_deg').value / 2.0:.0f}deg, "
                f"수평거리 <= {self.mask_max_range:.2f}m, "
                f"z {self.mask_min_z:.2f}~{self.mask_max_z:.2f}m"
                + (f" (제거된 점 -> {debug_topic})" if self.debug_pub else ""))
        else:
            self.get_logger().warn("자기가림 마스크 OFF — 적재물 점이 그대로 발행됨")

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
        if self.frames and self.frame_start_ns is not None:
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

        dtype = POINT_DTYPE.get(data_type)
        if dtype is None:
            return

        payload = packet[HEADER_SIZE:]
        count = min(dot_num, len(payload) // dtype.itemsize)
        if count == 0:
            return

        if self.frame_start_ns is None:
            self.frame_start_ns = packet_ts
            self.frame_start_ros = self.get_clock().now()

        step = span_ns / dot_num if dot_num else 0.0
        base_off = packet_ts - self.frame_start_ns
        scale = POINT_SCALE[data_type]

        raw = np.frombuffer(payload, dtype, count=count)
        pts = np.empty(count, CLOUD_DTYPE)
        pts["x"] = raw["x"] * scale
        pts["y"] = raw["y"] * scale
        pts["z"] = raw["z"] * scale
        pts["intensity"] = raw["refl"]
        # point_time = packet_ts + (i/dot_num)*span - frame_start (초). 프레임 첫
        # 패킷보다 이른 시각(재정렬)이면 0 으로 눌러 음수 time 을 막는다.
        t = (base_off + step * np.arange(count, dtype=np.float64)) * 1e-9
        pts["time"] = np.maximum(t, 0.0)
        self.frames.append(pts)

        # 이 패킷 마지막 점의 디바이스 시각(발행 트리거 판단용)
        self.frame_last_ns = packet_ts + span_ns

    def masked(self, pts):
        """마스크 영역에 드는 점의 불리언 배열."""
        az = np.arctan2(pts["y"], pts["x"])
        # 중심 기준 각차를 [-pi, pi) 로 접어 후방(+-180deg) 래핑을 흡수
        delta = np.abs(np.mod(az - self.mask_center + np.pi, 2.0 * np.pi) - np.pi)
        return ((delta <= self.mask_half_width)
                & (np.hypot(pts["x"], pts["y"]) <= self.mask_max_range)
                & (pts["z"] >= self.mask_min_z)
                & (pts["z"] <= self.mask_max_z))

    def publish_points(self):
        header = PointCloud2().header
        header.stamp = self.frame_start_ros.to_msg()
        header.frame_id = self.frame_id
        pts = np.concatenate(self.frames)

        self.frames = []
        self.frame_start_ns = None
        self.frame_start_ros = None

        if self.mask_enable:
            drop = self.masked(pts)
            self.n_total += len(pts)
            self.n_dropped += int(drop.sum())
            if self.debug_pub is not None:
                self.debug_pub.publish(
                    point_cloud2.create_cloud(header, CLOUD_FIELDS, pts[drop]))
            pts = pts[~drop]
            if self.n_total >= 2_000_000:      # 대략 10초마다
                self.get_logger().info(
                    f"마스크 제거율 {100.0 * self.n_dropped / self.n_total:.1f}% "
                    f"({self.n_dropped}/{self.n_total}점)")
                self.n_total = self.n_dropped = 0
            if len(pts) == 0:
                self.get_logger().warn(
                    "마스크가 프레임의 모든 점을 제거함 — mask_width_deg/"
                    "mask_max_range 가 과도한지 확인할 것",
                    throttle_duration_sec=5.0)
                return

        self.publisher.publish(
            point_cloud2.create_cloud(header, CLOUD_FIELDS, pts))


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
