"""Livox MID-360 내장 6축 IMU relay: /livox/imu -> /imu/data.

IMU 데이터 자체는 lidar.launch.py 의 livox_ros_driver2_node 가 /livox/imu 로
발행합니다. 이 launch 는 EKF 가 쓰기 좋게 /imu/data 로 재발행(orientation 무효화)만
담당합니다. (구 EBIMU imu_publisher.py 는 더 이상 사용하지 않음)
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def executable_path(name):
    return os.path.join(get_package_prefix("alm_sensors"), "lib", "alm_sensors", name)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("input_topic", default_value="/livox/imu"),
            DeclareLaunchArgument("output_topic", default_value="/imu/data"),
            Node(
                executable=executable_path("imu_relay.py"),
                name="imu_relay",
                output="screen",
                parameters=[
                    {"input_topic": LaunchConfiguration("input_topic")},
                    {"output_topic": LaunchConfiguration("output_topic")},
                    {"frame_id": ""},
                    {"invalidate_orientation": True},
                ],
            ),
        ]
    )
