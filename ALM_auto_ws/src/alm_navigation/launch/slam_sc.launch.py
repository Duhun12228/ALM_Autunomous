"""SC-LIO-SAM 매핑 (방식 C) — 루프클로저(RS+Scan Context) 있는 3D SLAM.

전제: /livox/lidar + /livox/imu 발행 중 (alm_sensors/lidar.launch.py).
FAST-LIO2 매핑(slam.launch.py)의 대체 — 넓은 공간에서 루프클로저로 드리프트 보정.

구성:
  imu_orientation            : /livox/imu(6축) -> /livox/imu_orient (Madgwick, LIO-SAM 요구)
  lio_sam_imuPreintegration  : IMU 사전적분 odometry
  lio_sam_imageProjection    : deskew + range image (MID-360 ring 합성, ALM 패치)
  lio_sam_featureExtraction  : corner/surface 특징
  lio_sam_mapOptimization    : scan-to-map + GTSAM 팩터그래프 + RS/SC 루프클로저

맵 저장 (종료 전에):
  ros2 service call /lio_sam/save_map lio_sam/srv/SaveMap "{resolution: 0.0, destination: ''}"
  -> $HOME/ALM_Autunomous/ALM_auto_ws/src/alm_navigation/maps/sc_lio_sam/GlobalMap.pcd
  이후 pcd2pgm / sc_build_db / localization.launch 는 방식 A/B 와 동일하게 사용.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    lio_share = get_package_share_directory("lio_sam")
    default_params = os.path.join(nav_share, "config", "sc_lio_sam.yaml")

    params = LaunchConfiguration("params")
    rviz = LaunchConfiguration("rviz")
    imu_accel_scale = LaunchConfiguration("imu_accel_scale")
    n_scan = LaunchConfiguration("n_scan")

    args = [
        DeclareLaunchArgument("params", default_value=default_params),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument(
            "imu_accel_scale", default_value="1.0",
            description="IMU acceleration multiplier (livox_ros_driver2 bags use g: 9.80665)"),
        DeclareLaunchArgument(
            "n_scan", default_value="16",
            description="Range-image rows (MID-360 driver2 native line data uses 4)"),
    ]

    imu_orientation = Node(
        package="alm_sensors",
        executable="imu_orientation.py",
        name="imu_orientation",
        parameters=[{"accel_scale": imu_accel_scale}],
        output="screen",
    )

    lio_nodes = [
        # executable 내부에서 노드 이름을 정한다. 특히 imuPreintegration 프로세스는
        # IMUPreintegration + TransformFusion 두 노드를 생성하므로 __node remap 금지.
        Node(package="lio_sam", executable=exe,
             parameters=[params, {
                 "N_SCAN": ParameterValue(n_scan, value_type=int),
             }], output="screen")
        for exe in ("lio_sam_imuPreintegration", "lio_sam_imageProjection",
                    "lio_sam_featureExtraction", "lio_sam_mapOptimization")
    ]

    # 매핑 중 map == odom (루프클로저 보정은 mapOptimization 내부 pose 그래프가 담당)
    static_map_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--frame-id", "map", "--child-frame-id", "odom"],
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(lio_share, "config", "rviz2.rviz")],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription(
        args + [imu_orientation, static_map_odom] + lio_nodes + [rviz_node])
