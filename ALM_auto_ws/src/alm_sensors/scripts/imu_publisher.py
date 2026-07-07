#!/usr/bin/env python3
"""Publish E2BOX/E2BDX gyro+accel data as sensor_msgs/Imu.

On startup this configures the IMU to disable the magnetometer and stream
ASCII gyroscope/accelerometer data, following the same serial protocol as
imu_control_tools/disable_imu_magnetometer.py.
"""

import argparse
import math
import sys
import time
from typing import List, Optional, Tuple

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu

try:
    import serial
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)

DEG_TO_RAD = math.pi / 180.0
G_TO_MPS2 = 9.80665

CONFIG_COMMANDS = (
    ("sem0", "magnetometer fusion OFF"),
    ("soc1", "ASCII output"),
    ("sof1", "Euler prefix"),
    ("sog1", "gyroscope XYZ output ON"),
    ("soa1", "accelerometer XYZ output ON"),
    ("som0", "magnetometer XYZ output OFF"),
)


def send_command(ser: "serial.Serial", command: str, timeout: float) -> str:
    payload = f"<{command}>".encode("ascii")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(payload)
    ser.flush()

    deadline = time.monotonic() + timeout
    response = bytearray()
    while time.monotonic() < deadline and b"<ok>" not in response:
        waiting = ser.in_waiting
        response.extend(ser.read(waiting if waiting > 0 else 1))

    return response.decode("ascii", errors="replace").strip()


def parse_six_axis_line(line: str) -> Optional[Tuple[List[float], List[float]]]:
    """Parse gyro(3), accelerometer(3) from an E2BOX ASCII line (Euler(3) is discarded)."""
    if not line.startswith("*"):
        return None

    try:
        values = [float(value.strip()) for value in line[1:].split(",")]
    except ValueError:
        return None

    if len(values) < 9:
        return None
    return values[3:6], values[6:9]


def configure_imu(ser: "serial.Serial", timeout: float) -> None:
    for command, description in CONFIG_COMMANDS:
        response = send_command(ser, command, timeout)
        if "<ok>" not in response:
            raise RuntimeError(
                f"<{command}> ({description}) did not return <ok>. Received: {response!r}"
            )
        print(f"  <{command}>: OK ({description})")


class ImuPublisher(Node):
    def __init__(self, ser: "serial.Serial", topic: str, frame_id: str):
        super().__init__("imu_publisher")
        qos_profile = QoSProfile(depth=10)
        self.ser = ser
        self.frame_id = frame_id
        self.publisher = self.create_publisher(Imu, topic, qos_profile)
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        raw_line = self.ser.readline()
        if not raw_line:
            return

        line = raw_line.decode("ascii", errors="ignore").strip()
        parsed = parse_six_axis_line(line)
        if parsed is None:
            return

        gyro, accel = parsed
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        # No orientation estimate is produced (magnetometer/Euler fusion is disabled).
        msg.orientation_covariance[0] = -1.0
        msg.angular_velocity.x = gyro[0] * DEG_TO_RAD
        msg.angular_velocity.y = gyro[1] * DEG_TO_RAD
        msg.angular_velocity.z = gyro[2] * DEG_TO_RAD
        msg.linear_acceleration.x = accel[0] * G_TO_MPS2
        msg.linear_acceleration.y = accel[1] * G_TO_MPS2
        msg.linear_acceleration.z = accel[2] * G_TO_MPS2
        self.publisher.publish(msg)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Publish E2BOX/E2BDX gyro+accel data as sensor_msgs/Imu."
    )
    parser.add_argument("--port", default="/dev/ttyUSB0", help="IMU serial device path.")
    parser.add_argument("--baudrate", type=int, default=115200, help="IMU serial baudrate.")
    parser.add_argument(
        "--timeout", type=float, default=2.0, help="Config command response timeout in seconds."
    )
    parser.add_argument("--topic", default="/imu/data", help="Topic to publish sensor_msgs/Imu on.")
    parser.add_argument("--frame-id", default="imu_link", help="Frame id for the Imu message header.")
    return parser.parse_known_args()


def main(args=None):
    cli_args, ros_args = parse_args()

    try:
        ser = serial.Serial(port=cli_args.port, baudrate=cli_args.baudrate, timeout=0.1)
    except serial.SerialException as exc:
        print(f"Serial port error: {exc}", file=sys.stderr)
        raise

    print("Configuring IMU for 6-axis (gyro+accel) output...")
    try:
        configure_imu(ser, cli_args.timeout)
    except RuntimeError as exc:
        print(f"FAIL: {exc}", file=sys.stderr)
        ser.close()
        sys.exit(1)

    rclpy.init(args=args if args is not None else ros_args)

    print("Starting imu_publisher..")
    node = ImuPublisher(ser, cli_args.topic, cli_args.frame_id)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
