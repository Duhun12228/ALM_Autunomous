"""base_control 실행.

  cmd_arbiter     : 자율(/cmd_vel,/drive_mode) vs 텔레옵(/cmd_vel_teleop,...) 동작권 중재
                    -> /cmd_vel_mux + /drive_mode_mux
  command_manager : /cmd_vel_mux + /drive_mode_mux -> /mcu/command (4WIS 변환 + 안전 게이팅)
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def executable_path(name):
    return os.path.join(get_package_prefix("alm_base_control"), "lib", "alm_base_control", name)


def generate_launch_description():
    share = get_package_share_directory("alm_base_control")
    default_cfg = os.path.join(share, "config", "base_control.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument("params_file", default_value=default_cfg),
            Node(
                executable=executable_path("cmd_arbiter.py"),
                name="cmd_arbiter",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
            Node(
                executable=executable_path("command_manager.py"),
                name="command_manager",
                output="screen",
                parameters=[LaunchConfiguration("params_file")],
            ),
        ]
    )
