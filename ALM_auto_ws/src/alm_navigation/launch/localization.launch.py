"""FAST-LIO Localization (prior map.pcd 안에서 재측위) — 단계 B 측위.

전제: /livox/lidar (PointCloud2 + per-point time) 와 /livox/imu 가 이미 발행 중
(alm_bringup/robot.launch.py -> alm_sensors/lidar.launch.py).

구성:
  icp_node            : 현재 스캔(/livox/lidar) 을 prior map.pcd 에 ICP 정합
                        -> /icp_result (pose)
  transform_publisher : /icp_result -> TF map->odom
  fastlio_mapping     : /livox/lidar + /livox/imu -> odom->base_link TF, /Odometry
                        (locate_in_prior_map 모드, config fastlio_relocalization.yaml)

  => AMCL + robot_localization EKF 를 대체한다 (map->odom + odom->base_link 전부 담당).

초기 pose: initial_x/y/z/a (yaw, rad) 인자로 주거나 RViz "2D Pose Estimate".
##TODO## prior map(map_pcd)을 실제 주행 매핑으로 만든 map.pcd 로 지정.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    default_config = os.path.join(nav_share, "config", "fastlio_relocalization.yaml")
    default_map = os.path.join(nav_share, "maps", "alm_3d_map.pcd")

    fastlio_config = LaunchConfiguration("fastlio_config")
    map_pcd = LaunchConfiguration("map_pcd")
    init_x = LaunchConfiguration("initial_x")
    init_y = LaunchConfiguration("initial_y")
    init_z = LaunchConfiguration("initial_z")
    init_a = LaunchConfiguration("initial_a")

    args = [
        DeclareLaunchArgument("fastlio_config", default_value=default_config),
        DeclareLaunchArgument("map_pcd", default_value=default_map,
                              description="prior 3D 점군맵(.pcd) 경로 (icp + fast_lio 동일해야 함)"),
        DeclareLaunchArgument("initial_x", default_value="0.0"),
        DeclareLaunchArgument("initial_y", default_value="0.0"),
        DeclareLaunchArgument("initial_z", default_value="0.0"),
        DeclareLaunchArgument("initial_a", default_value="0.0", description="초기 yaw (rad)"),
    ]

    # /icp_result -> TF map->odom
    transform_publisher = Node(
        package="icp_relocalization",
        executable="transform_publisher",
        name="transform_publisher",
        output="screen",
        parameters=[{"map_frame_id": "map"}, {"odom_frame_id": "odom"}],
    )

    # 현재 스캔을 prior map 에 ICP 정합 -> /icp_result
    icp_node = Node(
        package="icp_relocalization",
        executable="icp_node",
        name="icp_node",
        output="screen",
        parameters=[
            {"initial_x": init_x},
            {"initial_y": init_y},
            {"initial_z": init_z},
            {"initial_a": init_a},
            {"map_voxel_leaf_size": 0.5},
            {"cloud_voxel_leaf_size": 0.3},
            {"map_frame_id": "map"},
            {"solver_max_iter": 75},
            {"max_correspondence_distance": 0.1},
            {"RANSAC_outlier_rejection_threshold": 1.0},
            {"map_path": map_pcd},
            {"fitness_score_thre": 0.2},
            {"converged_count_thre": 40},
            {"pcl_type": "pointcloud"},   # livox 아님 -> PointCloud2 구독(/pointcloud2)
        ],
        remappings=[("/pointcloud2", "/livox/lidar")],
    )

    # FAST-LIO 측위 모드 (odom->base_link, /Odometry)
    fast_lio_node = Node(
        package="fast_lio",
        executable="fastlio_mapping",
        name="fastlio_localization",
        output="screen",
        parameters=[fastlio_config],
    )

    # icp/fast_lio 는 map 로딩 후 시작 (transform_publisher 먼저)
    delayed = TimerAction(period=3.0, actions=[icp_node, fast_lio_node])

    return LaunchDescription(args + [transform_publisher, delayed])
