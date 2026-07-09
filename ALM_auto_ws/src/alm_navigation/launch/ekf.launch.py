"""robot_localization EKF: /wheel_odom + /imu/data -> /odometry/filtered, TF odom->base_link."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("alm_navigation")
    default_ekf = os.path.join(share, "config", "ekf.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params = LaunchConfiguration("ekf_params")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("ekf_params", default_value=default_ekf),
            Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_filter_node",
                output="screen",
                parameters=[params, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
