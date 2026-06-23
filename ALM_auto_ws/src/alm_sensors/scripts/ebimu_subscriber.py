#!/usr/bin/env python3
# This is a code for wired sensors (ebimu-9dof).

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String


def data_parser(msg_data):
    msg_data = msg_data.strip()

    if msg_data.startswith("*"):
        msg_data = msg_data.replace("*", "", 1)

    words = msg_data.split(",")

    try:
        return list(map(float, words))
    except ValueError:
        return None


class EbimuSubscriber(Node):
    def __init__(self):
        super().__init__("ebimu_subscriber")
        qos_profile = QoSProfile(depth=10)
        self.subscription = self.create_subscription(String, "ebimu_data", self.callback, qos_profile)

    def callback(self, msg):
        imu_data = data_parser(msg.data)
        if imu_data is not None:
            print(imu_data)


def main(args=None):
    rclpy.init(args=args)

    print("Starting ebimu_subscriber..")
    node = EbimuSubscriber()

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
