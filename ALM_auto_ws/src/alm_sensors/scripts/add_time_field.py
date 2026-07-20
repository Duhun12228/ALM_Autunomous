#!/usr/bin/env python3
"""Republish a Livox PointCloud2 with a per-point relative-time field.

livox_udp_pointcloud2.py (alm_sensors) publishes PointCloud2 with only
x, y, z, intensity: the raw Livox UDP protocol used there is parsed without
per-point timestamps. FAST-LIO2's generic (non-AVIA) point-cloud handler
requires a "time" field (seconds since the start of the scan) to de-skew
each cloud, otherwise it falls back to a per-ring yaw-based estimate that
needs a "ring" field we don't have either.

If the input contains the livox_ros_driver2 ``timestamp`` field (absolute
nanoseconds), it is converted to exact seconds relative to the first point.
If no point timestamp exists, offsets are assigned evenly over scan_period.
"""

import rclpy
from rclpy.executors import ExternalShutdownException
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
        self.logged_time_mode = False

        self.timed_fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name="time", offset=16, datatype=PointField.FLOAT32, count=1),
        ]
        # rosbag2/livox_ros_driver2의 native line을 SC-LIO-SAM이 이해하는 ring으로
        # 보존한다. offset 18~19는 명시적인 padding이며 create_cloud가 point_step을
        # 올바르게 계산한다. live UDP 입력처럼 line이 없으면 기존 5-field 포맷 유지.
        self.ring_timed_fields = [
            *self.timed_fields[:4],
            PointField(name="ring", offset=16, datatype=PointField.UINT16, count=1),
            PointField(name="time", offset=20, datatype=PointField.FLOAT32, count=1),
        ]

        input_topic = self.get_parameter("input_topic").value
        self.sub = self.create_subscription(PointCloud2, input_topic, self.on_cloud, 10)
        self.pub = self.create_publisher(PointCloud2, self.output_topic, 10)
        self.get_logger().info(
            f"Normalizing point time/ring fields: {input_topic} -> {self.output_topic}")

    def on_cloud(self, msg: PointCloud2):
        input_fields = {field.name for field in msg.fields}
        ring_source = "ring" if "ring" in input_fields else (
            "line" if "line" in input_fields else None)

        if "timestamp" in input_fields:
            source_fields = ["x", "y", "z", "intensity", "timestamp"]
            mode = "timestamp"
        elif "time" in input_fields:
            source_fields = ["x", "y", "z", "intensity", "time"]
            mode = "time"
        else:
            source_fields = ["x", "y", "z", "intensity"]
            mode = "synthetic"

        time_index = len(source_fields) - 1 if mode != "synthetic" else None
        if ring_source:
            source_fields.append(ring_source)
        ring_index = len(source_fields) - 1 if ring_source else None

        points = list(point_cloud2.read_points(
            msg, field_names=source_fields, skip_nans=False))
        count = len(points)
        if count == 0:
            return

        if mode == "timestamp":
            first_ns = int(points[0][time_index])
            offsets = [(int(p[time_index]) - first_ns) * 1e-9 for p in points]
        elif mode == "time":
            first_time = float(points[0][time_index])
            offsets = [float(p[time_index]) - first_time for p in points]
        else:
            offsets = [(i / count) * self.scan_period for i in range(count)]

        if ring_source:
            fields = self.ring_timed_fields
            timed_points = [
                (float(p[0]), float(p[1]), float(p[2]), float(p[3]),
                 int(p[ring_index]), offsets[i])
                for i, p in enumerate(points)
            ]
        else:
            fields = self.timed_fields
            timed_points = [
                (float(p[0]), float(p[1]), float(p[2]), float(p[3]), offsets[i])
                for i, p in enumerate(points)
            ]

        if not self.logged_time_mode:
            self.get_logger().info(
                f"Point time mode={mode}, ring={ring_source or 'synthetic downstream'}, "
                f"points={count}, span={offsets[-1]:.6f}s")
            self.logged_time_mode = True

        out = point_cloud2.create_cloud(msg.header, fields, timed_points)
        self.pub.publish(out)


def main():
    rclpy.init()
    node = AddTimeField()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
