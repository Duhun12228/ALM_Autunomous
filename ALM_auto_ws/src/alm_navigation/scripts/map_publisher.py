#!/usr/bin/env python3
"""저장된 2D 맵(pgm+yaml)을 /map (OccupancyGrid) 으로 발행 — RViz 확인용.

nav2_map_server 없이 맵을 띄워 보기 위한 경량 퍼블리셔.
    ros2 run alm_navigation map_publisher.py --ros-args -p yaml:=<맵.yaml>
RViz: Fixed Frame=map, Add>Map, Topic=/map (Durability=Transient Local).
"""
import sys
import numpy as np
import yaml as yamllib
from PIL import Image

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from nav_msgs.msg import OccupancyGrid


class MapPublisher(Node):
    def __init__(self):
        super().__init__("map_publisher")
        self.declare_parameter("yaml", "")
        ypath = self.get_parameter("yaml").value
        if not ypath:
            self.get_logger().error("param 'yaml' 필요: -p yaml:=<맵.yaml>")
            raise SystemExit(1)

        with open(ypath) as f:
            meta = yamllib.safe_load(f)
        import os
        img_path = meta["image"]
        if not os.path.isabs(img_path):
            img_path = os.path.join(os.path.dirname(ypath), img_path)
        res = float(meta["resolution"])
        ox, oy, _ = meta["origin"]

        img = np.array(Image.open(img_path))          # 상단이 y_max
        h, w = img.shape
        # pcd2pgm 규약: 0=occupied, 205=unknown, 254=free
        grid = np.full((h, w), -1, dtype=np.int8)
        grid[img <= 50] = 100     # occupied
        grid[img >= 250] = 0      # free
        # 205 등 그 사이 = unknown(-1) 유지
        grid = np.flipud(grid)    # OccupancyGrid 는 하단이 원점 -> 상하반전

        msg = OccupancyGrid()
        msg.header.frame_id = "map"
        msg.info.resolution = res
        msg.info.width = w
        msg.info.height = h
        msg.info.origin.position.x = float(ox)
        msg.info.origin.position.y = float(oy)
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.msg = msg

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, "/map", qos)
        self.timer = self.create_timer(1.0, self.tick)
        self.get_logger().info(
            f"발행: /map  {w}x{h} @ {res}m  origin=({ox},{oy})  "
            f"(occ={int((grid==100).sum())} free={int((grid==0).sum())} unk={int((grid==-1).sum())})"
        )

    def tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    node = MapPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
