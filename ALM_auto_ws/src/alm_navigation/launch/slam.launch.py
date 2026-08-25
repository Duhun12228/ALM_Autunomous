"""FAST-LIO2 3D 매핑 모드 (Livox Mid-360 + 내장 6축 IMU).

전제: /livox/lidar (PointCloud2) 와 /livox/imu (Imu) 가 이미 발행 중이어야 한다
(alm_bringup 의 robot.launch.py -> alm_sensors/lidar.launch.py 로 확보).

이 launch 는 그 위에 3D SLAM 만 얹는다:
    fastlio_mapping : /livox/lidar + /livox/imu -> 3D odometry + 누적 점군맵
  (/livox/lidar 는 livox_udp_pointcloud2 가 실제 per-point "time" 필드까지 붙여
   발행하므로 별도 time-field 어댑터가 필요 없다.)

동시에 scan_recorder 가 '정합된 스캔 + 그때의 센서 위치' 를 활성 맵 폴더의
scans.npz 로 남긴다. pcd2pgm 이 이걸 받아 **레이캐스팅**으로 2D 격자를 굽는다
(광선이 지나간 셀 = 자유공간). 없으면 예전 투영 방식으로 떨어지는데, 그러면
격자의 8할 이상이 미관측으로 남는다 — pcd2pgm docstring 참고.
    record:=false 로 끌 수 있다.

3D 맵 저장 (매핑 주행 후):
    ros2 service call /map_save std_srvs/srv/Trigger
  -> **활성 맵 폴더**(maps/active.yaml)의 cloud.pcd 로 저장. map_pcd 인자로
     덮어쓸 수 있다.

  맵은 폴더 하나가 곧 맵 하나다 — maps/<맵이름>/{manifest.yaml, cloud.pcd,
  scans.npz, grid.pgm+grid.yaml, fpfh_map*}. 다른 맵에 저장하려면
  maps/active.yaml 의 active 를 바꾸거나 map_pcd:= 로 지정한다.

  ##왜 launch 가 덮어쓰나## (2026-08-23)
  예전에는 fastlio_mid360.yaml 의 map_file_path 를 사람이 직접 고쳤다. 그런데
  그 값이 절대경로라 **다른 사람 홈 디렉터리(/home/kdh/...)가 박혀 있었고**,
  그 상태로 /map_save 를 부르면 조용히 실패한다(경로가 없으므로). 활성 맵을
  단일 진실 공급원으로 두고 launch 가 주입하면 그 부류의 사고가 사라진다.
  localization.launch.py 가 prior_map_path 를 다루는 방식과 같다.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_layout  # noqa: E402  (launch 디렉터리 안의 공용 모듈)


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")

    # 활성 맵 폴더에 스캔을 남긴다 — cloud.pcd 와 같은 곳이어야 짝이 맞는다.
    maps_root = map_layout.maps_root(nav_share)
    _active = map_layout.active_map_paths(maps_root)
    default_scans = _active.scans
    default_pcd = _active.cloud

    default_config = os.path.join(nav_share, "config", "fastlio_mid360.yaml")
    # 매핑 전용 RViz (Fixed Frame=odom, /Laser_map·/cloud_registered 표시).
    # fast_lio 의 rviz 파일명은 loam_livox.rviz 라 예전 fastlio.rviz 참조는 로드
    # 실패했었음 -> alm_navigation 자체 rviz 로 고정.
    rviz_cfg = os.path.join(nav_share, "rviz", "fastlio_mapping.rviz")

    config_file = LaunchConfiguration("fastlio_config")
    rviz_use = LaunchConfiguration("rviz")
    record_use = LaunchConfiguration("record")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("fastlio_config", default_value=default_config),
            DeclareLaunchArgument("rviz", default_value="false",
                                  description="FAST-LIO 오도메트리/맵 시각화 RViz"),
            DeclareLaunchArgument("record", default_value="true",
                                  description="스캔+센서위치를 scans.npz 로 기록 "
                                              "(pcd2pgm 레이캐스팅 입력)"),
            DeclareLaunchArgument("scans_out", default_value=default_scans,
                                  description="scan_recorder 출력 .npz 경로"),
            DeclareLaunchArgument("map_pcd", default_value=default_pcd,
                                  description="/map_save 가 쓸 3D 점군 경로. "
                                              "yaml 의 map_file_path 를 덮어쓴다"),
            # FAST-LIO2 3D 매핑
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                name="fastlio_mapping",
                output="screen",
                # ##중요## map_file_path 를 활성 맵으로 **덮어쓴다.** yaml 에도 같은
                # 키가 있지만 parameters 리스트는 뒤엣것이 이긴다.
                parameters=[config_file,
                            {"map_file_path": LaunchConfiguration("map_pcd")}],
            ),
            # 스캔 기록 (레이캐스팅 입력). 매핑 자체는 아무것도 바뀌지 않는다 —
            # /cloud_registered 와 /Odometry 를 구독만 한다.
            Node(
                package="alm_navigation",
                executable="scan_recorder.py",
                name="scan_recorder",
                output="screen",
                parameters=[{"out": LaunchConfiguration("scans_out")}],
                condition=IfCondition(record_use),
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
