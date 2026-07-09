"""Nav2 (AMCL 위치추정 + planner/controller) - 저장된 맵 기반 자율주행.

nav2_bringup 의 bringup_launch.py 를 slam:=False 로 포함해서
map_server + amcl + planner_server + controller_server + bt_navigator 등을 올립니다.
출력은 /cmd_vel (이후 alm_base_control 이 받아 mode 처리 + MCU 전송).

전제: odom->base_link TF(EKF), /scan 이 이미 발행 중 (alm_bringup robot.launch.py).

    ros2 launch alm_navigation navigation.launch.py map:=/path/to/my_map.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    default_params = os.path.join(nav_share, "config", "nav2.yaml")
    default_map = os.path.join(nav_share, "maps", "my_map.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    nav2_bringup_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "bringup_launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_bringup_launch),
                launch_arguments={
                    "slam": "False",
                    "map": map_yaml,
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
