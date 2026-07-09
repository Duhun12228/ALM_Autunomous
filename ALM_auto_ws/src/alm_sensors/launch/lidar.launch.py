"""Livox MID-360 3D LiDAR + 내장 IMU bringup (UDP 직접 파싱 방식, SDK 불필요).

  livox_udp_pointcloud2 : UDP 56301 파싱 -> /livox/lidar (PointCloud2)
  livox_udp_imu         : UDP 56401 파싱 -> /livox/imu (내장 6축 IMU)
  imu_relay             : /livox/imu -> /imu/data (EKF 입력, orientation 무효화)
  pointcloud_to_scan    : /livox/lidar -> /scan (2D LaserScan, numpy)
  static_transform_publisher : base_link -> livox_frame  (##TODO## 실측 마운트)

공식 livox_ros_driver2(SDK)를 쓰지 않고, LiDAR 가 호스트로 쏘는 UDP 패킷을
직접 파싱한다. MID360_config.json 의 host_net_info 포트(56301/56401)와
host IP(192.168.1.5)가 실제 네트워크와 일치해야 한다.

주의: livox_udp_pointcloud2.py 는 HOST_IP/POINT_PORT 가 상수로 박혀 있으니
네트워크가 바뀌면 스크립트도 함께 수정할 것.
"""

import os

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def executable_path(name):
    return os.path.join(get_package_prefix("alm_sensors"), "lib", "alm_sensors", name)


def generate_launch_description():
    frame_id = LaunchConfiguration("lidar_frame")

    # ##TODO## LiDAR 마운트 위치 확정 후 아래 x y z (m) 수정
    lidar_x = LaunchConfiguration("lidar_x")
    lidar_y = LaunchConfiguration("lidar_y")
    lidar_z = LaunchConfiguration("lidar_z")

    return LaunchDescription(
        [
            DeclareLaunchArgument("lidar_frame", default_value="livox_frame"),
            DeclareLaunchArgument("lidar_x", default_value="0.0"),
            DeclareLaunchArgument("lidar_y", default_value="0.0"),
            DeclareLaunchArgument("lidar_z", default_value="0.5"),
            # ---- UDP 포인트클라우드 파서: /livox/lidar ----
            Node(
                executable=executable_path("livox_udp_pointcloud2.py"),
                name="livox_lidar",
                output="screen",
            ),
            # ---- UDP 내장 IMU 파서: /livox/imu ----
            Node(
                executable=executable_path("livox_udp_imu.py"),
                name="livox_imu",
                output="screen",
                parameters=[
                    {"host_ip": "192.168.1.5"},
                    {"imu_port": 56401},
                    {"imu_topic": "/livox/imu"},
                    {"frame_id": frame_id},
                ],
            ),
            # ---- IMU relay: /livox/imu -> /imu/data (EKF 입력) ----
            Node(
                executable=executable_path("imu_relay.py"),
                name="imu_relay",
                output="screen",
                parameters=[
                    {"input_topic": "/livox/imu"},
                    {"output_topic": "/imu/data"},
                    {"frame_id": ""},
                    {"invalidate_orientation": True},
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
            # ---- 3D PointCloud2 -> 2D LaserScan (numpy) ----
            Node(
                executable=executable_path("pointcloud_to_scan.py"),
                name="pointcloud_to_scan",
                output="screen",
                parameters=[
                    {"cloud_topic": "/livox/lidar"},
                    {"scan_topic": "/scan"},
                ],
            ),
        ]
    )
