"""자율주행 모드: 로봇 상시 스택 + FAST-LIO-Localization + Nav2.

    ros2 launch alm_bringup navigation.launch.py

맵 경로는 인자를 비워 두면 maps/active.yaml 이 가리키는 맵에서 자동으로 조립된다
(maps/<맵이름>/grid.yaml · cloud.pcd · fpfh_map). 다른 맵을 쓰려면 active.yaml 을
고치거나 인자로 직접 덮어쓴다:

    ros2 launch alm_bringup navigation.launch.py \
      map:=<ws>/src/alm_navigation/maps/<맵이름>/grid.yaml \
      map_pcd:=<ws>/src/alm_navigation/maps/<맵이름>/cloud.pcd \
      fpfh_db_prefix:=<ws>/src/alm_navigation/maps/<맵이름>/fpfh_map

관측용 RViz(경로 · global/local costmap · footprint · 2D Goal Pose 툴)를 같이 띄우려면:

    ros2 launch alm_bringup navigation.launch.py rviz:=true

FPFH+TEASER++ 자동 초기위치가 완료된 뒤 RViz에서 Nav2 Goal을 지정하거나,
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
    fpfh_db_prefix = LaunchConfiguration("fpfh_db_prefix")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value=""),
            DeclareLaunchArgument("map_pcd", default_value=""),
            DeclareLaunchArgument("fpfh_db_prefix", default_value=""),
            # rviz:=true -> 경로/코스트맵 관측용 RViz 동시 실행
            DeclareLaunchArgument("rviz", default_value="false"),
            # 측위는 FAST-LIO-Localization 담당 -> EKF 끔 (odom->base_link TF 충돌 방지)
            _include("alm_bringup", "robot.launch.py",
                     {"use_sim_time": use_sim_time, "use_ekf": "false"}),
            _include("alm_navigation", "navigation.launch.py",
                     {"use_sim_time": use_sim_time, "map": map_yaml,
                      "map_pcd": map_pcd, "fpfh_db_prefix": fpfh_db_prefix,
                      "rviz": LaunchConfiguration("rviz")}),
        ]
    )
