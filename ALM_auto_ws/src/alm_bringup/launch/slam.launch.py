"""매핑 모드: 로봇 상시 스택 + FAST-LIO2 3D 매핑.

    ros2 launch alm_bringup slam.launch.py

맵 저장:
    ros2 service call /map_save std_srvs/srv/Trigger
  -> fastlio_mid360.yaml 의 map_file_path (기본: maps/alm_lab/cloud.pcd)

⚠ **새 맵을 만들 때는 매핑 전에** fastlio_mid360.yaml 의 map_file_path 를 새 맵
  폴더로 바꾸고 그 폴더에 manifest.yaml 을 두어야 한다. 안 그러면 기존 맵의
  cloud.pcd 를 덮어쓴다 (fast_lio 파라미터라 launch 인자로 치환되지 않는다).
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
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            _include("alm_bringup", "robot.launch.py", {"use_sim_time": use_sim_time}),
            _include("alm_navigation", "slam.launch.py", {"use_sim_time": use_sim_time}),
        ]
    )
