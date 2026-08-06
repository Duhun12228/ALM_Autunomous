#!/usr/bin/env python3
"""pcd_replay — 저장된 3D 맵을 라이다 스트림처럼 재생. **개발 전용.**

MID-360 이 물려 있지 않은 상태에서 WebUI 의 3D 뷰포트·로봇 마커·측위 화면을
검증하려고 만든 노드다. `alm_3d_map.pcd`(FAST-LIO 산출) 안에서 가상의 로봇을
궤적 따라 움직이며, 매 프레임 그 위치에서 보이는 점만 잘라 `/livox/lidar` 로
내보낸다. 필드 구성과 frame_id 를 livox_udp_pointcloud2.py 와 똑같이 맞췄으므로
하위 노드(pointcloud_to_scan, FAST-LIO)는 진짜 라이다와 구분하지 못한다.

  ⚠ livox_udp_pointcloud2 와 동시에 띄우지 말 것 — 같은 토픽에 두 퍼블리셔가 된다.

발행:
  /livox/lidar   PointCloud2 (x,y,z,intensity,time / frame=livox_frame)
  /Odometry      nav_msgs/Odometry (odom -> base_link)
  TF             map->odom (항등), odom->base_link (궤적)

가시성 근사는 '반경 안 + 높이 밴드' 까지만 한다. 오클루전(벽 뒤가 안 보이는 것)은
계산하지 않으므로 실제 스캔보다 점이 많고 벽 너머까지 보인다 — 화면 배선을
검증하는 용도에는 충분하고, 측위 정확도 실험에는 쓰면 안 된다.
"""

import math
import os

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster

from pcd2pgm import read_pcd_xyz

# livox_udp_pointcloud2.py 의 CLOUD_FIELDS 와 바이트 레이아웃까지 동일해야
# create_cloud 가 memoryview 고속 경로를 탄다.
CLOUD_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
]
CLOUD_DTYPE = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                        ("intensity", "<f4"), ("time", "<f4")])


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class PcdReplay(Node):
    def __init__(self):
        super().__init__("pcd_replay")
        g = self.declare_parameter
        g("pcd", "")
        g("cloud_topic", "/livox/lidar")
        g("odom_topic", "/Odometry")
        g("frame_id", "livox_frame")
        g("cloud_rate_hz", 5.0)
        g("odom_rate_hz", 10.0)
        g("max_range", 25.0)        # 이 반경 안의 점만 한 프레임에 담는다
        g("min_range", 0.4)         # 자기 차체 근처는 버린다
        g("max_points", 24000)      # MID-360 한 프레임과 비슷한 규모
        # 맵 전체를 매 프레임 훑으면 162만 점에 대해 마스킹을 도느라 CPU 를
        # 50% 넘게 먹는다 — 개발용 더미가 정작 측정 대상인 CPU 수치를
        # 오염시키고, 같은 실행자를 쓰는 odom 타이머까지 굶긴다.
        # 어차피 출력은 max_points 로 솎이므로 로드 시점에 한 번 줄여 둔다.
        g("source_max_points", 400000)
        g("z_min", -0.6)            # 센서 기준 높이 밴드
        g("z_max", 2.0)
        g("sensor_height", 0.5)     # base_link 기준 라이다 높이
        g("loop_radius", 6.0)       # 궤적 원 반지름 [m]
        g("loop_period_sec", 60.0)  # 한 바퀴 도는 시간
        g("center_x", float("nan"))  # NaN 이면 점군 중심을 쓴다
        g("center_y", float("nan"))

        p = self.get_parameter
        pcd_path = str(p("pcd").value)
        if not pcd_path or not os.path.exists(pcd_path):
            self.get_logger().error(f"param 'pcd' 로 실제 파일을 지정하세요: {pcd_path!r}")
            raise SystemExit(1)

        self.get_logger().info(f"{pcd_path} 읽는 중…")
        points = read_pcd_xyz(pcd_path)
        loaded = len(points)

        # 균등 간격 데시메이션. 무작위 표본은 밀도가 얼룩져 벽이 성겨 보인다.
        source_max = int(p("source_max_points").value)
        if 0 < source_max < loaded:
            points = points[::int(np.ceil(loaded / source_max))]
        self.points = np.ascontiguousarray(points)

        self.get_logger().warn(
            f"pcd_replay 기동 — 개발 전용 더미입니다 "
            f"({loaded:,}점 중 {len(self.points):,}점 사용). "
            "livox_udp_pointcloud2 와 동시에 실행하지 마세요.")

        cx = float(p("center_x").value)
        cy = float(p("center_y").value)
        if math.isnan(cx) or math.isnan(cy):
            cx, cy = float(self.points[:, 0].mean()), float(self.points[:, 1].mean())
        self.center = (cx, cy)

        self.max_range = float(p("max_range").value)
        self.min_range = float(p("min_range").value)
        self.max_points = int(p("max_points").value)
        self.z_min = float(p("z_min").value)
        self.z_max = float(p("z_max").value)
        self.sensor_height = float(p("sensor_height").value)
        self.loop_radius = float(p("loop_radius").value)
        self.loop_period = max(1.0, float(p("loop_period_sec").value))
        self.frame_id = str(p("frame_id").value)

        self.cloud_pub = self.create_publisher(
            PointCloud2, str(p("cloud_topic").value), 5)
        self.odom_pub = self.create_publisher(
            Odometry, str(p("odom_topic").value), 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # map->odom 은 항등으로 고정. 실제로는 transform_publisher(ICP)가 채운다.
        static_qos = QoSProfile(depth=1,
                                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.static_tf = StaticTransformBroadcaster(self, qos=static_qos)
        identity = TransformStamped()
        identity.header.stamp = self.get_clock().now().to_msg()
        identity.header.frame_id = "map"
        identity.child_frame_id = "odom"
        identity.transform.rotation.w = 1.0
        self.static_tf.sendTransform(identity)

        self.start = self.get_clock().now()
        self.create_timer(1.0 / max(0.1, float(p("cloud_rate_hz").value)),
                          self.publish_cloud)
        self.create_timer(1.0 / max(0.1, float(p("odom_rate_hz").value)),
                          self.publish_odom)

    def pose_now(self):
        """궤적 위의 (x, y, yaw). 원을 도는 동안 진행 방향을 바라본다."""
        elapsed = (self.get_clock().now() - self.start).nanoseconds / 1e9
        phase = 2.0 * math.pi * (elapsed % self.loop_period) / self.loop_period
        x = self.center[0] + self.loop_radius * math.cos(phase)
        y = self.center[1] + self.loop_radius * math.sin(phase)
        return x, y, phase + math.pi / 2.0

    def publish_cloud(self):
        x, y, yaw = self.pose_now()
        z = self.sensor_height

        # 1) 바운딩박스로 먼저 후보를 줄이고(정수 비교라 싸다), 2) 반경으로 다듬는다.
        pts = self.points
        box = ((np.abs(pts[:, 0] - x) <= self.max_range)
               & (np.abs(pts[:, 1] - y) <= self.max_range))
        near = pts[box]
        if len(near) == 0:
            return

        # 센서 원점 기준으로 옮기고 헤딩만큼 되돌린다
        dx = near[:, 0] - x
        dy = near[:, 1] - y
        dz = near[:, 2] - z
        cos_y, sin_y = math.cos(-yaw), math.sin(-yaw)
        local_x = dx * cos_y - dy * sin_y
        local_y = dx * sin_y + dy * cos_y

        distance = np.hypot(local_x, local_y)
        keep = ((distance <= self.max_range) & (distance >= self.min_range)
                & (dz >= self.z_min) & (dz <= self.z_max))
        local_x, local_y, dz = local_x[keep], local_y[keep], dz[keep]

        count = len(local_x)
        if count == 0:
            return
        if count > self.max_points:
            # 균등 솎아내기. 무작위로 뽑으면 프레임마다 점이 튀어 보인다.
            step = count / self.max_points
            index = (np.arange(self.max_points) * step).astype(np.int64)
            local_x, local_y, dz = local_x[index], local_y[index], dz[index]
            count = self.max_points

        record = np.empty(count, dtype=CLOUD_DTYPE)
        record["x"] = local_x
        record["y"] = local_y
        record["z"] = dz
        record["intensity"] = 100.0
        # 실제 라이다의 per-point time 처럼 프레임 안에서 선형 증가시킨다
        record["time"] = np.linspace(0.0, 0.1, count, dtype=np.float32)

        header = PointCloud2().header
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = self.frame_id
        self.cloud_pub.publish(
            point_cloud2.create_cloud(header, CLOUD_FIELDS, record))

    def publish_odom(self):
        x, y, yaw = self.pose_now()
        stamp = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = yaw_to_quaternion(yaw)
        # 원 궤적의 접선 속도 = 2*pi*r/T
        odom.twist.twist.linear.x = 2.0 * math.pi * self.loop_radius / self.loop_period
        odom.twist.twist.angular.z = 2.0 * math.pi / self.loop_period
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = "odom"
        tf.child_frame_id = "base_link"
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation = yaw_to_quaternion(yaw)
        self.tf_broadcaster.sendTransform(tf)


def main():
    rclpy.init()
    node = PcdReplay()
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
