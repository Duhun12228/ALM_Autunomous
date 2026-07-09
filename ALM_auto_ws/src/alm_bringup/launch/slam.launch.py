"""매핑 모드: 로봇 상시 스택 + FAST-LIO2 3D 매핑.

    ros2 launch alm_bringup slam.launch.py

맵 저장:
    ros2 service call /map_save std_srvs/srv/Trigger
  -> <ws>/src/alm_navigation/maps/alm_3d_map.pcd
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
