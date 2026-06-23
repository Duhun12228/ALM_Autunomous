import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import ExecuteProcess


def executable_path(name):
    return os.path.join(get_package_prefix("alm_sensors"), "lib", "alm_sensors", name)


def generate_launch_description():
    return LaunchDescription(
        [
            ExecuteProcess(
                cmd=[executable_path("livox_udp_pointcloud2.py")],
                name="livox_lidar",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[executable_path("ebimu_publisher.sh")],
                name="ebimu_publisher",
                output="screen",
            ),
            ExecuteProcess(
                cmd=[executable_path("ebimu_subscriber.sh")],
                name="ebimu_subscriber",
                output="screen",
            ),
        ]
    )
