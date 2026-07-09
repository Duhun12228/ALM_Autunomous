"""mcu_bridge 실행 - Jetson <-> STM32 UART 통신."""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def executable_path(name):
    return os.path.join(get_package_prefix("alm_mcu_interface"), "lib", "alm_mcu_interface", name)


def generate_launch_description():
    share = get_package_share_directory("alm_mcu_interface")
    default_cfg = os.path.join(share, "config", "mcu_interface.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_cfg),
            DeclareLaunchArgument("port", default_value="/dev/ttyTHS1"),
            Node(
                executable=executable_path("mcu_bridge.py"),
                name="mcu_bridge",
                output="screen",
                parameters=[
                    LaunchConfiguration("params_file"),
                    {"port": LaunchConfiguration("port")},
                ],
            ),
        ]
    )
