"""FAST-LIO2 3D 매핑 모드 (Livox Mid-360 + 내장 6축 IMU).

전제: /livox/lidar (PointCloud2) 와 /livox/imu (Imu) 가 이미 발행 중이어야 한다
(alm_bringup 의 robot.launch.py -> alm_sensors/lidar.launch.py 로 확보).

이 launch 는 그 위에 3D SLAM 만 얹는다:
    fastlio_mapping : /livox/lidar + /livox/imu -> 3D odometry + 누적 점군맵
  (/livox/lidar 는 livox_udp_pointcloud2 가 실제 per-point "time" 필드까지 붙여
   발행하므로 별도 time-field 어댑터가 필요 없다.)

3D 맵 저장 (매핑 주행 후):
    ros2 service call /map_save std_srvs/srv/Trigger
  -> config 의 map_file_path (기본: alm_navigation/maps/alm_3d_map.pcd) 로 저장.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    fast_lio_share = get_package_share_directory("fast_lio")

    default_config = os.path.join(nav_share, "config", "fastlio_mid360.yaml")
    rviz_cfg = os.path.join(fast_lio_share, "rviz", "fastlio.rviz")

    config_file = LaunchConfiguration("fastlio_config")
    rviz_use = LaunchConfiguration("rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("fastlio_config", default_value=default_config),
            DeclareLaunchArgument("rviz", default_value="false",
                                  description="FAST-LIO 오도메트리/맵 시각화 RViz"),
            # FAST-LIO2 3D 매핑
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fastlio_mapping",
                output="screen",
                parameters=[config_file],
            ),
            # 시각화 (선택)
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_cfg],
                condition=IfCondition(rviz_use),
            ),
        ]
    )
