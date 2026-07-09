"""slam_toolbox 매핑 모드. /scan + TF(odom->base_link) -> /map, TF map->odom.

전제: odom->base_link TF(EKF)와 /scan 이 이미 발행 중이어야 합니다
(alm_bringup 의 robot.launch.py 또는 ekf.launch.py + alm_sensors 로 확보).

맵 저장:
    ros2 run nav2_map_server map_saver_cli -f ~/ALM_Autunomous/maps/my_map
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("alm_navigation")
    default_params = os.path.join(share, "config", "slam_toolbox.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = LaunchConfiguration("slam_params_file")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("slam_params_file", default_value=default_params),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
