"""자율주행 (단계 B): FAST-LIO-Localization + Nav2. AMCL/EKF 미사용.

구성:
  map_server            : 저장된 2D 맵(map.pgm/yaml) -> global costmap static layer
  localization.launch.py: FAST-LIO-Localization (map->odom + odom->base_link)
                          ※ AMCL + robot_localization EKF 를 대체
  nav2 navigation_launch: planner(SmacPlannerHybrid) / smoother(ConstrainedSmoother)
                          / controller(MPPI) / behavior / bt -> /cmd_vel

경로 파이프라인:
  ComputePathToPose(Hybrid-A*) -> SmoothPath(ConstrainedSmoother) -> FollowPath(MPPI)
연결은 alm_navigation/behavior_trees/navigate_*_w_smoothing.xml 이 담당하며,
bt_navigator 가 읽을 절대경로를 RewrittenYaml 로 nav2.yaml 에 주입합니다.

전제: /livox/lidar (+time) 와 /livox/imu 가 이미 발행 중
(alm_bringup/robot.launch.py -> alm_sensors/lidar.launch.py).

    ros2 launch alm_navigation navigation.launch.py \
        map:=<2D map.yaml> map_pcd:=<3D map.pcd> fpfh_db_prefix:=<DB prefix>
  초기 pose 는 FPFH+TEASER++ 전역 정합으로 자동 계산한다.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_layout  # noqa: E402  (같은 디렉터리의 공용 헬퍼)


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    default_params = os.path.join(nav_share, "config", "nav2.yaml")

    # maps/active.yaml 이 가리키는 맵에서 세 경로를 한꺼번에 얻는다.
    # (이전 기본값 my_map.yaml 은 실제로 존재한 적이 없었다.)
    maps_root = map_layout.maps_root(nav_share)
    active = map_layout.active_map_paths(maps_root)
    default_map = active.grid_yaml if active else ""
    default_map_pcd = active.cloud if active else ""
    default_fpfh_db = active.fpfh_prefix if active else ""

    bt_dir = os.path.join(nav_share, "behavior_trees")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    map_pcd = LaunchConfiguration("map_pcd")
    fpfh_db_prefix = LaunchConfiguration("fpfh_db_prefix")
    accum_frames = LaunchConfiguration("accum_frames")

    # nav2.yaml 의 default_nav_*_bt_xml 플레이스홀더를 설치된 BT 절대경로로 치환.
    # (SmoothPath 노드가 들어간 커스텀 트리 = ConstrainedSmoother 가 실제로 불리는 지점)
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": os.path.join(
                bt_dir, "navigate_to_pose_w_smoothing.xml"),
            "default_nav_through_poses_bt_xml": os.path.join(
                bt_dir, "navigate_through_poses_w_smoothing.xml"),
        },
        convert_types=True,
    )

    nav2_navigation_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )
    loc_launch = PathJoinSubstitution(
        [FindPackageShare("alm_navigation"), "launch", "localization.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map,
                                  description="2D 맵(pcd2pgm 산출 .yaml) - global costmap용"),
            DeclareLaunchArgument("map_pcd", default_value=default_map_pcd,
                                  description="3D prior 맵(.pcd) - FAST-LIO-Localization용"),
            DeclareLaunchArgument("fpfh_db_prefix", default_value=default_fpfh_db,
                                  description="map_pcd에서 생성한 FPFH DB prefix"),
            DeclareLaunchArgument("accum_frames", default_value="10"),

            # ---- 저장된 2D 맵 서버 (global costmap static layer) ----
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[configured_params, {"yaml_filename": map_yaml},
                            {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time},
                            {"autostart": True},
                            {"node_names": ["map_server"]}],
            ),

            # ---- FAST-LIO-Localization (map->odom + odom->base_link) ----
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(loc_launch),
                launch_arguments={
                    "map_pcd": map_pcd,
                    "fpfh_db_prefix": fpfh_db_prefix,
                    "accum_frames": accum_frames,
                    "auto_init": "true",
                }.items(),
            ),

            # ---- Nav2 네비게이션 코어 (planner/smoother/controller/bt) ----
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_navigation_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": configured_params,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
