import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def executable_path(name):
    return os.path.join(get_package_prefix("alm_sensors"), "lib", "alm_sensors", name)


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("imu_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("imu_baudrate", default_value="115200"),
            DeclareLaunchArgument("imu_timeout", default_value="2.0"),
            DeclareLaunchArgument("imu_topic", default_value="/imu/data"),
            DeclareLaunchArgument("imu_frame_id", default_value="imu_link"),
            ExecuteProcess(
                cmd=[
                    executable_path("imu_publisher.py"),
                    "--port",
                    LaunchConfiguration("imu_port"),
                    "--baudrate",
                    LaunchConfiguration("imu_baudrate"),
                    "--timeout",
                    LaunchConfiguration("imu_timeout"),
                    "--topic",
                    LaunchConfiguration("imu_topic"),
                    "--frame-id",
                    LaunchConfiguration("imu_frame_id"),
                ],
                name="imu_publisher",
                output="screen",
            ),
        ]
    )
