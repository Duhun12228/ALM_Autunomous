#!/usr/bin/env python3
"""저장된 2D 맵(pgm+yaml)을 /map (OccupancyGrid) 으로 발행.

두 가지 모드가 있다.

    추종 모드 (기본)  maps/active.yaml 을 따라간다. 활성 맵이 바뀌면 다시 읽어
                      발행한다. WebUI 에서 맵을 고르면 2D 지도도 같이 바뀐다.
    고정 모드         `-p yaml:=<맵.yaml>` 을 주면 그 파일만 발행한다.
                      Nav2 처럼 특정 맵으로 못박아야 하는 곳에서 쓴다.

    ros2 run alm_navigation map_publisher.py                      # 추종
    ros2 run alm_navigation map_publisher.py --ros-args -p yaml:=<맵.yaml>

RViz: Fixed Frame=map, Add>Map, Topic=/map (Durability=Transient Local).

발행 규약: TRANSIENT_LOCAL + **변화 시에만**. 예전에는 680 KB 짜리 맵을 1 Hz 로
맹목 재발행했는데(약 5 Mbps), latch 가 걸려 있으면 늦게 붙는 구독자도 마지막
값을 받으므로 재발행할 이유가 없다.
"""
import os
import sys

import numpy as np
import yaml as yamllib
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(
    get_package_share_directory("alm_navigation"), "launch"))
import map_layout                                          # noqa: E402


class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_publisher")
        self.declare_parameter("yaml", "")
        self.declare_parameter("maps_root", "")
        self.declare_parameter("poll_sec", 5.0)

        self.fixed_yaml = str(self.get_parameter("yaml").value)
        self.root = str(self.get_parameter("maps_root").value) or map_layout.maps_root(
            get_package_share_directory("alm_navigation"))

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, "/map", qos)

        self.loaded = None          # (경로, mtime)
        self.tick()
        if self.fixed_yaml:
            self.get_logger().info(f"고정 모드: {self.fixed_yaml}")
        else:
            poll = max(1.0, float(self.get_parameter("poll_sec").value))
            self.create_timer(poll, self.tick)
            self.get_logger().info(f"추종 모드: {self.root}/active.yaml 을 따라갑니다")

    # ── 대상 결정 ───────────────────────────────────────────────────────
    def target(self):
        if self.fixed_yaml:
            return self.fixed_yaml
        paths = map_layout.active_map_paths(self.root)
        return paths.grid_yaml if paths else ""

    def tick(self):
        path = self.target()
        if not path or not os.path.isfile(path):
            # 활성 맵에 2D 격자가 아직 없다 — 새로 만든 맵의 정상 상태다.
            # 이전 맵의 격자를 그대로 두면 화면이 거짓말을 하므로 비운다.
            if self.loaded is not None:
                self.get_logger().info("활성 맵에 grid.yaml 이 없습니다 — 빈 맵 발행")
                self.publish_empty()
                self.loaded = None
            return
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return
        if self.loaded == (path, mtime):
            return
        if self.load_and_publish(path):
            self.loaded = (path, mtime)

    # ── 로드 ────────────────────────────────────────────────────────────
    def load_and_publish(self, ypath):
        try:
            with open(ypath) as handle:
                meta = yamllib.safe_load(handle)
            img_path = meta["image"]
            if not os.path.isabs(img_path):
                img_path = os.path.join(os.path.dirname(ypath), img_path)
            res = float(meta["resolution"])
            ox, oy, _ = meta["origin"]
            img = np.array(Image.open(img_path))          # 상단이 y_max
        except Exception as error:                        # noqa: BLE001
            self.get_logger().error(f"{ypath} 읽기 실패: {error}")
            return False

        h, w = img.shape
        # pcd2pgm 규약: 0=occupied, 205=unknown, 254=free
        grid = np.full((h, w), -1, dtype=np.int8)
        grid[img <= 50] = 100     # occupied
        grid[img >= 250] = 0      # free
        # 205 등 그 사이 = unknown(-1) 유지
        grid = np.flipud(grid)    # OccupancyGrid 는 하단이 원점 -> 상하반전

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = res
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = float(ox)
        msg.info.origin.position.y = float(oy)
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.pub.publish(msg)
        self.get_logger().info(
            f"발행: /map  {w}x{h} @ {res}m  origin=({ox},{oy})  "
            f"(occ={int((grid == 100).sum())} free={int((grid == 0).sum())} "
            f"unk={int((grid == -1).sum())})  ← {ypath}"
        )
        return True

    def publish_empty(self):
        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = 0.05
        msg.info.origin.orientation.w = 1.0
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = MapPublisher()
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
