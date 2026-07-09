#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2
from tf2_ros import Buffer, TransformException, TransformListener


class PointCloudToScan(Node):
    def __init__(self):
        super().__init__("pointcloud_to_scan")

        self.declare_parameter("cloud_topic", "/livox/lidar")
        self.declare_parameter("scan_topic", "/scan")
        self.declare_parameter("scan_frame_id", "")
        # When set, points are transformed into this frame (via TF) before the
        # horizontal slice is taken. Leave empty to slice in the cloud frame.
        self.declare_parameter("target_frame", "")
        self.declare_parameter("tf_timeout", 0.1)
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
        self.target_frame = self.get_parameter("target_frame").value
        self.tf_timeout = float(self.get_parameter("tf_timeout").value)
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
        self.last_tf_log = 0.0

        self.tf_buffer = None
        self.tf_listener = None
        if self.target_frame:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)

        target = self.target_frame or "cloud frame"
        self.get_logger().info(
            f"Converting {self.cloud_topic} PointCloud2 to {self.scan_topic} LaserScan "
            f"with height [{self.min_height:.2f}, {self.max_height:.2f}] m in {target}"
        )

    def convert(self, cloud_msg):
        ranges = np.full(self.beam_count, math.inf, dtype=np.float32)
        intensities = np.zeros(self.beam_count, dtype=np.float32)

        x, y, z, intensity = self.read_xyzi(cloud_msg)
        if x.size and self.target_frame:
            x, y, z, ok = self.transform_points(cloud_msg.header, x, y, z)
            if not ok:
                return

        accepted = 0
        if x.size:
            mask = (z >= self.min_height) & (z <= self.max_height)
            scan_range = np.hypot(x, y)
            mask &= (scan_range >= self.range_min) & (scan_range <= self.range_max)
            angle = np.arctan2(y, x)
            mask &= (angle >= self.angle_min) & (angle <= self.angle_max)

            scan_range = scan_range[mask]
            angle = angle[mask]
            beam_intensity = intensity[mask]

            index = ((angle - self.angle_min) / self.angle_increment).astype(np.int64)
            in_bounds = (index >= 0) & (index < self.beam_count)
            index = index[in_bounds]
            scan_range = scan_range[in_bounds]
            beam_intensity = beam_intensity[in_bounds]

            accepted = int(index.size)
            if accepted:
                # Keep the closest return per beam: sort by range descending so
                # the nearest point is written last and wins the assignment.
                order = np.argsort(scan_range, kind="stable")[::-1]
                index = index[order]
                ranges[index] = scan_range[order].astype(np.float32)
                intensities[index] = beam_intensity[order]

        if accepted == 0:
            self.log_empty_scan()

        scan_msg = LaserScan()
        scan_msg.header.stamp = cloud_msg.header.stamp
        scan_msg.header.frame_id = (
            self.scan_frame_id or self.target_frame or cloud_msg.header.frame_id
        )
        scan_msg.angle_min = self.angle_min
        scan_msg.angle_max = self.angle_min + self.angle_increment * (self.beam_count - 1)
        scan_msg.angle_increment = self.angle_increment
        scan_msg.time_increment = 0.0
        scan_msg.scan_time = self.scan_time
        scan_msg.range_min = self.range_min
        scan_msg.range_max = self.range_max
        scan_msg.ranges = ranges.tolist()
        scan_msg.intensities = intensities.tolist()
        self.publisher.publish(scan_msg)

    def read_xyzi(self, cloud_msg):
        cloud = point_cloud2.read_points(
            cloud_msg,
            field_names=("x", "y", "z", "intensity"),
            skip_nans=True,
        )
        if cloud.size == 0:
            empty = np.empty(0, dtype=np.float64)
            return empty, empty, empty, np.empty(0, dtype=np.float32)
        x = np.asarray(cloud["x"], dtype=np.float64)
        y = np.asarray(cloud["y"], dtype=np.float64)
        z = np.asarray(cloud["z"], dtype=np.float64)
        intensity = np.asarray(cloud["intensity"], dtype=np.float32)
        return x, y, z, intensity

    def transform_points(self, header, x, y, z):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.target_frame,
                header.frame_id,
                Time.from_msg(header.stamp),
                timeout=Duration(seconds=self.tf_timeout),
            )
        except TransformException as exc:
            self.log_tf_error(str(exc))
            return x, y, z, False

        q = tf.transform.rotation
        t = tf.transform.translation
        rotation = self.quaternion_matrix(q.x, q.y, q.z, q.w)
        points = np.vstack((x, y, z))
        rotated = rotation @ points
        return rotated[0] + t.x, rotated[1] + t.y, rotated[2] + t.z, True

    @staticmethod
    def quaternion_matrix(x, y, z, w):
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return np.eye(3)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array(
            [
                [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
                [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
                [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
            ]
        )

    def log_empty_scan(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_empty_log > 2.0:
            self.get_logger().warn(
                "No points passed the scan filters. Adjust min_height/max_height "
                "or range/angle parameters if /scan stays empty."
            )
            self.last_empty_log = now

    def log_tf_error(self, message):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self.last_tf_log > 2.0:
            self.get_logger().warn(
                f"Skipping scan; transform to '{self.target_frame}' unavailable: {message}"
            )
            self.last_tf_log = now


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
