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

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

# 맵 경로 조립은 alm_navigation 의 공용 헬퍼 한 곳에만 있다 (단일 진실 공급원).
sys.path.insert(0, os.path.join(
    get_package_share_directory("alm_navigation"), "launch"))
import map_layout  # noqa: E402


def _include(pkg, launch_file, args=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare(pkg), "launch", launch_file])
        ),
        launch_arguments=(args or {}).items(),
    )


def generate_launch_description():
    # ★ 기본값을 "" 로 두면 안 된다 — 여기서 선언하는 순간 스코프에 빈 값이 박히고,
    #   그것이 alm_navigation 의 active.yaml 기본값을 **이긴다.**
    #   `IncludeLaunchDescription` 은 launch configuration 을 격리하지 않으므로,
    #   부모가 먼저 선언한 이름은 자식의 `DeclareLaunchArgument` 기본값을 무력화한다.
    #   (인자를 넘기지 않아도 마찬가지다 — 이미 설정돼 있기 때문이다.)
    #
    #   실제로 났던 사고 (2026-08-20): map_pcd·fpfh_db_prefix 가 "" 로 내려가
    #       teaser_fpfh_localizer fatal error: map_path and fpfh_db_prefix are required
    #       fastlio_localization: Failed to load PCD file
    #   가 났다. teaser 가 죽으니 초기위치가 안 나오고, FAST-LIO 는 영원히
    #   'Waiting for initial pose...' 에 머물러 odom TF 를 못 낸다. 그러면 Nav2 는
    #   'Invalid frame ID "odom"' 만 반복하며 아무것도 못 한다. Nav2 자체는 정상
    #   기동한 것처럼 보이므로 **원인이 잘 안 보이는 종류의 고장**이다.
    #
    #   그래서 alm_navigation 과 **같은 헬퍼**로 같은 기본값을 만든다.
    nav_share = get_package_share_directory("alm_navigation")
    active = map_layout.active_map_paths(map_layout.maps_root(nav_share))
    default_map = active.grid_yaml if active else ""
    default_map_pcd = active.cloud if active else ""
    default_fpfh_db = active.fpfh_prefix if active else ""

    use_sim_time = LaunchConfiguration("use_sim_time")
    map_yaml = LaunchConfiguration("map")
    map_pcd = LaunchConfiguration("map_pcd")
    fpfh_db_prefix = LaunchConfiguration("fpfh_db_prefix")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("map", default_value=default_map),
            DeclareLaunchArgument("map_pcd", default_value=default_map_pcd),
            DeclareLaunchArgument("fpfh_db_prefix", default_value=default_fpfh_db),
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
