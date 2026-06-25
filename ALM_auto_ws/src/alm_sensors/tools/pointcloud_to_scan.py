#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2


class PointCloudToScan(Node):
    def __init__(self):
        super().__init__("pointcloud_to_scan")

        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_frame_id", "")
        self.declare_parameter("min_height", -0.25)
        self.declare_parameter("max_height", 0.25)
        self.declare_parameter("angle_min", -math.pi)
        self.declare_parameter("angle_max", math.pi)
        self.declare_parameter("angle_increment", math.radians(0.5))
        self.declare_parameter("range_min", 0.05)
        self.declare_parameter("range_max", 80.0)
        self.declare_parameter("scan_time", 0.1)

        self.cloud_topic = self.get_parameter("cloud_topic").value
        self.scan_topic = self.get_parameter("scan_topic").value
        self.scan_frame_id = self.get_parameter("scan_frame_id").value
        self.min_height = float(self.get_parameter("min_height").value)
        self.max_height = float(self.get_parameter("max_height").value)
        self.angle_min = float(self.get_parameter("angle_min").value)
        self.angle_max = float(self.get_parameter("angle_max").value)
        self.angle_increment = float(self.get_parameter("angle_increment").value)
        self.range_min = float(self.get_parameter("range_min").value)
        self.range_max = float(self.get_parameter("range_max").value)
        self.scan_time = float(self.get_parameter("scan_time").value)

        if self.angle_increment <= 0.0:
            raise ValueError("angle_increment must be positive")
        if self.angle_max <= self.angle_min:
            raise ValueError("angle_max must be greater than angle_min")
        if self.max_height < self.min_height:
            raise ValueError("max_height must be greater than or equal to min_height")

        self.beam_count = int(math.ceil((self.angle_max - self.angle_min) / self.angle_increment)) + 1
        self.publisher = self.create_publisher(LaserScan, self.scan_topic, 10)
        self.subscription = self.create_subscription(PointCloud2, self.cloud_topic, self.convert, 10)
        self.last_empty_log = 0.0

        self.get_logger().info(
            f"Converting {self.cloud_topic} PointCloud2 to {self.scan_topic} LaserScan "
            f"with height [{self.min_height:.2f}, {self.max_height:.2f}] m"
        )

    def convert(self, cloud_msg):
        ranges = [math.inf] * self.beam_count
        intensities = [0.0] * self.beam_count
        accepted_points = 0

        for point in point_cloud2.read_points(
            cloud_msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True,
        ):
            x, y, z, intensity = self.unpack_point(point)

            if z < self.min_height or z > self.max_height:
                continue

            scan_range = math.hypot(x, y)
            if scan_range < self.range_min or scan_range > self.range_max:
                continue

            angle = math.atan2(y, x)
            if angle < self.angle_min or angle > self.angle_max:
                continue

            index = int((angle - self.angle_min) / self.angle_increment)
            if index < 0 or index >= self.beam_count:
                continue

            if scan_range < ranges[index]:
                ranges[index] = scan_range
                intensities[index] = intensity
            accepted_points += 1

        if accepted_points == 0:
            self.log_empty_scan()

        scan_msg = LaserScan()
        scan_msg.header.stamp = cloud_msg.header.stamp
        scan_msg.header.frame_id = self.scan_frame_id or cloud_msg.header.frame_id
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_min + self.angle_increment * (self.beam_count - 1)
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges
        scan_msg.intensities = intensities
        self.publisher.publish(scan_msg)

    def unpack_point(self, point):
        try:
            return (
                float(point["x"]),
                float(point["y"]),
                float(point["z"]),
                float(point["intensity"]),
            )
        except (TypeError, ValueError, IndexError):
            return float(point[0]), float(point[1]), float(point[2]), float(point[3])

    def log_empty_scan(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_empty_log > 2.0:
            self.get_logger().warn(
                "No points passed the scan filters. Adjust min_height/max_height "
                "or range/angle parameters if /scan stays empty."
            )
            self.last_empty_log = now


def main():
    rclpy.init()
    node = PointCloudToScan()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
