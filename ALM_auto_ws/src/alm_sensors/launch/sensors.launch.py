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
            DeclareLaunchArgument("ebimu_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("ebimu_baudrate", default_value="115200"),
            ExecuteProcess(
                cmd=[executable_path("livox_udp_pointcloud2.py")],
                name="livox_lidar",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[
                    executable_path("ebimu_publisher.py"),
                    "--port",
                    LaunchConfiguration("ebimu_port"),
                    "--baudrate",
                    LaunchConfiguration("ebimu_baudrate"),
                ],
                name="ebimu_publisher",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[executable_path("ebimu_subscriber.py")],
                name="ebimu_subscriber",
                output="screen",
            ),
        ]
    )
