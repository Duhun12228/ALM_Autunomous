#!/usr/bin/env python3
"""Republish /livox/lidar with a synthetic per-point relative-time field.

livox_udp_pointcloud2.py (alm_sensors) publishes PointCloud2 with only
x, y, z, intensity: the raw Livox UDP protocol used there is parsed without
per-point timestamps. FAST-LIO2's generic (non-AVIA) point-cloud handler
requires a "time" field (seconds since the start of the scan) to de-skew
each cloud, otherwise it falls back to a per-ring yaw-based estimate that
needs a "ring" field we don't have either.

This node does not modify alm_sensors: it only subscribes to its output
topic and adds the field FAST-LIO2 needs, on a new topic. Since points are
already batched by the publisher into ~scan_period-length bursts, offsets
are assigned evenly across that window rather than reconstructed exactly.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2


class AddTimeField(Node):
    def __init__(self):
        super().__init__("livox_add_time_field")
        self.declare_parameter("input_topic", "/livox/lidar")
        self.declare_parameter("output_topic", "/livox/lidar_timed")
        self.declare_parameter("scan_period", 0.1)

        self.output_topic = self.get_parameter("output_topic").value
        self.scan_period = float(self.get_parameter("scan_period").value)

        # x,y,z,intensity,time. FAST-LIO2 also expects a "ring" field and logs a
        # harmless "Failed to find match for field 'ring'" per scan because we omit it.
        # We deliberately do NOT add ring: solid-state Livox has no scan rings, and once
        # a valid "time" field is present FAST-LIO2 never reads ring for timing. (Mixing a
        # uint16 ring into this all-float32 cloud broke point-step alignment and caused
        # empty scans, so an all-float32 layout is kept and the cosmetic warning tolerated.)
        self.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
        ]

        input_topic = self.get_parameter("input_topic").value
        self.sub = self.create_subscription(PointCloud2, input_topic, self.on_cloud, 10)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.get_logger().info(f"Adding synthetic 'time' field: {input_topic} -> {self.output_topic}")

    def on_cloud(self, msg: PointCloud2):
        points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=False))
        count = len(points)
        if count == 0:
            return

        timed_points = [
            (float(p[0]), float(p[1]), float(p[2]), float(p[3]), (i / count) * self.scan_period)
            for i, p in enumerate(points)
        ]

        out = point_cloud2.create_cloud(msg.header, self.fields, timed_points)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = AddTimeField()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
