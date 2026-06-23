#!/usr/bin/env python3
import argparse
import struct

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String

import serial


class EbimuSerialParser:
    BINARY_HEADER = b"\xaa\x55"
    BINARY_FRAME_SIZE = 16
    BINARY_FOOTER = b"\r\n"

    def __init__(self, serial_port):
        self.serial_port = serial_port
        self.buffer = bytearray()

    def read_values(self):
        read_size = max(self.serial_port.in_waiting, 1)
        serial_data = self.serial_port.read(read_size)

        if not serial_data:
            return None

        self.buffer.extend(serial_data)
        return self.parse_buffer()

    def parse_buffer(self):
        while self.buffer:
            header_index = self.buffer.find(self.BINARY_HEADER)

            if header_index == -1:
                values = self.parse_ascii_line()
                if values is not None:
                    return values

                if len(self.buffer) > self.BINARY_FRAME_SIZE:
                    del self.buffer[:-self.BINARY_FRAME_SIZE]

                return None

            if header_index > 0:
                del self.buffer[:header_index]

            if len(self.buffer) < self.BINARY_FRAME_SIZE:
                return None

            frame = bytes(self.buffer[: self.BINARY_FRAME_SIZE])

            if frame[-2:] == self.BINARY_FOOTER:
                del self.buffer[: self.BINARY_FRAME_SIZE]
                return struct.unpack("<fff", frame[2:14])

            del self.buffer[0]

        return None

    def parse_ascii_line(self):
        newline_index = self.buffer.find(b"\n")

        if newline_index == -1:
            return None

        line = bytes(self.buffer[: newline_index + 1]).decode("utf-8", errors="ignore").strip()
        del self.buffer[: newline_index + 1]

        if not line.startswith("*"):
            return None

        try:
            return [float(value) for value in line.replace("*", "", 1).split(",")]
        except ValueError:
            return None


class EbimuPublisher(Node):
    def __init__(self, serial_port):
        super().__init__("ebimu_publisher")
        qos_profile = QoSProfile(depth=10)

        self.publisher = self.create_publisher(String, "ebimu_data", qos_profile)
        self.parser = EbimuSerialParser(serial_port)
        self.timer = self.create_timer(0.001, self.timer_callback)

    def timer_callback(self):
        imu_values = self.parser.read_values()

        if imu_values is None:
            return

        msg = String()
        msg.data = "*" + ",".join(f"{value:.6f}" for value in imu_values)
        self.publisher.publish(msg)


def parse_args():
    parser = argparse.ArgumentParser(description="Publish EBIMU serial data as std_msgs/String.")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="EBIMU serial device path.")
    parser.add_argument("--baudrate", default=115200, type=int, help="EBIMU serial baudrate.")
    return parser.parse_known_args()


def main(args=None):
    cli_args, ros_args = parse_args()

    try:
        serial_port = serial.Serial(port=cli_args.port, baudrate=cli_args.baudrate)
    except serial.SerialException as exc:
        print(f"Serial port error: {exc}")
        raise

    rclpy.init(args=args if args is not None else ros_args)

    print("Starting ebimu_publisher..")
    node = EbimuPublisher(serial_port)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        serial_port.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
