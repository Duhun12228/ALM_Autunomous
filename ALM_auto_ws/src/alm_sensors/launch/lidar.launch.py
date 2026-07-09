"""Livox MID-360 3D LiDAR bringup (공식 livox_ros_driver2).

  livox_ros_driver2_node : /livox/lidar (PointCloud2), /livox/imu (Imu)
  pointcloud_to_laserscan: /livox/lidar -> /scan (2D LaserScan, C++ 표준 노드)
  static_transform_publisher : base_link -> livox_frame  (##TODO## 실측 마운트)

사전 준비:
  - livox_ros_driver2: https://github.com/Livox-SDK/livox_ros_driver2
  - pointcloud_to_laserscan: sudo apt install ros-humble-pointcloud-to-laserscan
  - config/MID360_config.json 의 lidar ip / host ip 를 실제 네트워크에 맞게 수정.

주의(2D scan 튜닝):
  target_frame=base_link 로 두어 높이 밴드(min/max_height)가 '지면 기준'이 됩니다.
  → 바닥점(z<min_height)은 자동 제외되어 바닥 오탐이 줄어듭니다.
  실내에서 벽이 잘 잡히도록 min_height/max_height 를 돌려보며 조정하세요.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("alm_sensors")
    default_cfg = os.path.join(share, "config", "MID360_config.json")

    user_config = LaunchConfiguration("user_config_path")
    frame_id = LaunchConfiguration("lidar_frame")

    # ##TODO## LiDAR 마운트 위치 확정 후 아래 x y z (m) 수정
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")

    return LaunchDescription(
        [
            DeclareLaunchArgument("user_config_path", default_value=default_cfg),
            DeclareLaunchArgument("lidar_frame", default_value="livox_frame"),
            DeclareLaunchArgument("lidar_x", default_value="0.0"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.5"),
            # ---- Livox 공식 드라이버 ----
            Node(
                package="livox_ros_driver2",
                executable="livox_ros_driver2_node",
                name="livox_lidar",
                output="screen",
                parameters=[
                    {"xfer_format": 0},        # 0 = PointCloud2
                    {"multi_topic": 0},
                    {"data_src": 0},           # 0 = raw lidar
                    {"publish_freq": 10.0},
                    {"output_data_type": 0},
                    {"frame_id": frame_id},
                    {"user_config_path": user_config},
                    {"cmdline_input_bd_code": "livox0000000001"},
                ],
            ),
            # ---- base_link -> livox_frame 정적 변환 (##TODO## 실측) ----
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="base_to_livox_tf",
                arguments=[
                    "--x", lidar_x, "--y", lidar_y, "--z", lidar_z,
                    "--yaw", "0", "--pitch", "0", "--roll", "0",
                    "--frame-id", "base_link", "--child-frame-id", frame_id,
                ],
            ),
            # ---- 3D PointCloud2 -> 2D LaserScan (C++ 표준, 고속) ----
            Node(
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                output="screen",
                remappings=[
                    ("cloud_in", "/livox/lidar"),
                    ("scan", "/scan"),
                ],
                parameters=[{
                    "target_frame": "base_link",   # 지면 기준으로 높이 필터
                    "transform_tolerance": 0.05,
                    # 높이 밴드(base_link 지면 기준). ##TODO## 실내 벽/장애물에 맞게 튜닝
                    "min_height": 0.20,
                    "max_height": 1.00,
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.0087,     # 0.5 deg
                    "scan_time": 0.1,
                    "range_min": 0.20,
                    "range_max": 40.0,
                    "use_inf": True,
                    "inf_epsilon": 1.0,
                    "concurrency_level": 1,
                }],
            ),
        ]
    )
