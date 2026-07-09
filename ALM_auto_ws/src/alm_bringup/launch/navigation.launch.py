"""자율주행 모드: 로봇 상시 스택 + AMCL 위치추정 + Nav2.

    ros2 launch alm_bringup navigation.launch.py map:=<ws>/src/alm_navigation/maps/my_map.yaml

이후 RViz 에서 2D Pose Estimate 로 초기 위치를 주고 Nav2 Goal 로 목표를 지정하거나,
    ros2 topic pub /drive_mode std_msgs/msg/String "{data: 'auto'}" -1
로 auto 모드(normal/spin 자동 선택) 자율주행을 시작합니다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def _include(pkg, launch_file, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", launch_file])
        ),
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value=""),
            _include("alm_bringup", "robot.launch.py", {"use_sim_time": use_sim_time}),
            _include("alm_navigation", "navigation.launch.py",
                     {"use_sim_time": use_sim_time, "map": map_yaml}),
        ]
    )
