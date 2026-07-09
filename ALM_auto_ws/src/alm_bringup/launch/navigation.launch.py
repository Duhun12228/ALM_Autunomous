"""자율주행 모드: 로봇 상시 스택 + FAST-LIO-Localization + Nav2.

    ros2 launch alm_bringup navigation.launch.py \
      map:=<ws>/src/alm_navigation/maps/alm_map.yaml \
      map_pcd:=<ws>/src/alm_navigation/maps/alm_3d_map.pcd

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
    map_pcd = LaunchConfiguration("map_pcd")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("map_pcd", default_value=""),
            # 측위는 FAST-LIO-Localization 담당 -> EKF 끔 (odom->base_link TF 충돌 방지)
            _include("alm_bringup", "robot.launch.py",
                     {"use_sim_time": use_sim_time, "use_ekf": "false"}),
            _include("alm_navigation", "navigation.launch.py",
                     {"use_sim_time": use_sim_time, "map": map_yaml, "map_pcd": map_pcd}),
        ]
    )
