"""FAST-LIO Localization (prior map.pcd 안에서 재측위) — 단계 B 측위.

전제: /livox/lidar (PointCloud2 + per-point time) 와 /livox/imu 가 이미 발행 중
(alm_bringup/robot.launch.py -> alm_sensors/lidar.launch.py).

구성:
  teaser_fpfh_localizer: FPFH 전역 대응점 + TEASER++ + 지역 GICP
                         -> /icp_result (pose)
  transform_publisher : /icp_result -> TF map->odom
  fastlio_mapping     : /livox/lidar + /livox/imu -> odom->base_link TF, /Odometry
                        (locate_in_prior_map 모드, config fastlio_relocalization.yaml)
  초기위치 DB          : fpfh_map_builder 로 prior map의 FPFH를 사전 생성

  => AMCL + robot_localization EKF 를 대체한다 (map->odom + odom->base_link 전부 담당).

초기 pose:
  auto_init:=true + fpfh_db_prefix (FPFH+TEASER++ 전역 자동 측위)
##TODO## prior map(map_pcd)을 실제 주행 매핑으로 만든 map.pcd 로 지정.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    default_config = os.path.join(nav_share, "config", "fastlio_relocalization.yaml")
    default_map = os.path.join(nav_share, "maps", "alm_3d_map.pcd")
    default_fpfh_db = os.path.join(nav_share, "maps", "fpfh_map")

    fastlio_config = LaunchConfiguration("fastlio_config")
    map_pcd = LaunchConfiguration("map_pcd")
    auto_init = LaunchConfiguration("auto_init")
    fpfh_db_prefix = LaunchConfiguration("fpfh_db_prefix")
    accum_frames = LaunchConfiguration("accum_frames")

    args = [
        DeclareLaunchArgument("fastlio_config", default_value=default_config),
        DeclareLaunchArgument("map_pcd", default_value=default_map,
                              description="prior 3D 점군맵(.pcd) 경로 (icp + fast_lio 동일해야 함)"),
        DeclareLaunchArgument("auto_init", default_value="true",
                              description="FPFH+TEASER++ 초기위치 자동특정"),
        DeclareLaunchArgument("fpfh_db_prefix", default_value=default_fpfh_db,
                              description="fpfh_map_builder 출력 prefix (확장자 제외)"),
        DeclareLaunchArgument("accum_frames", default_value="10",
                              description="전역 정합 전에 정지 상태로 누적할 LiDAR 프레임 수"),
    ]

    # /icp_result -> TF map->odom
    transform_publisher = Node(
        package="icp_relocalization",
        executable="transform_publisher",
        name="transform_publisher",
        output="screen",
        parameters=[{"map_frame_id": "map"}, {"odom_frame_id": "odom"}],
    )

    # 초기 추정 없는 전역 FPFH+TEASER++ 정합 후 지역 GICP -> /icp_result
    teaser_localizer = Node(
        package="icp_relocalization",
        executable="teaser_fpfh_localizer",
        name="teaser_fpfh_localizer",
        output="screen",
        condition=IfCondition(auto_init),
        parameters=[
            {"map_path": map_pcd},
            {"fpfh_db_prefix": fpfh_db_prefix},
            {"lidar_topic": "/livox/lidar"},
            {"map_frame_id": "map"},
            {"accum_frames": ParameterValue(accum_frames, value_type=int)},
            # DB 생성기와 반드시 같은 전처리 파라미터.
            {"feature_voxel": 0.5},
            {"normal_radius": 1.0},
            {"feature_radius": 2.5},
            {"scan_min_range": 0.5},
            {"scan_max_range": 10.0},
            {"z_min": -0.35},
            {"z_max": 1.0},
            {"min_curvature": 0.0},
            {"max_scan_features": 1500},
            # FPFH 대응점과 TEASER++ 전역 정합.
            {"feature_ratio_threshold": 0.95},
            {"min_feature_matches": 20},
            {"max_feature_matches": 400},
            {"teaser_noise_bound": 0.5},
            {"teaser_exact_clique": False},
            {"teaser_max_clique_time_limit": 2.0},
            {"min_teaser_inliers": 6},
            # TEASER 결과 독립 검증과 지역 GICP.
            {"validation_inlier_distance": 0.50},
            {"validation_min_inlier_ratio": 0.20},
            {"validation_max_rmse": 0.35},
            {"local_map_radius": 12.0},
            {"gicp_max_correspondence": 1.0},
            {"gicp_fitness_threshold": 0.30},
            # 잘못된 단발 결과를 막기 위해 새 누적 스캔 두 번에서 일치해야 한다.
            {"consistent_result_count": 2},
            {"consistency_translation": 0.50},
            {"consistency_rotation_deg": 5.0},
        ],
    )

    # FAST-LIO 측위 모드 (odom->base_link, /Odometry)
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_localization",
        output="screen",
        parameters=[fastlio_config],
    )

    # 전역 로컬라이저/fast_lio 는 map 로딩 후 시작 (transform_publisher 먼저).
    delayed = TimerAction(period=3.0, actions=[teaser_localizer, fast_lio_node])

    return LaunchDescription(args + [transform_publisher, delayed])
