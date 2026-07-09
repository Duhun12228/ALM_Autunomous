"""ALM 로봇 '상시 구동' 스택 (SLAM/Nav2 제외).

포함:
  alm_description   : robot_state_publisher (URDF TF)
  alm_sensors       : Livox MID-360 (/livox/lidar, /livox/imu) + /scan + /imu/data
  alm_navigation    : robot_localization EKF (odom->base_link)
  alm_base_control  : command_manager (/cmd_vel + /drive_mode -> /mcu/command)
  alm_mcu_interface : mcu_bridge (UART <-> STM32, /wheel_odom, /joint_states)

SLAM 이나 자율주행은 이 위에 slam.launch.py / navigation.launch.py 를 얹습니다.
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
            _include("alm_description", "description.launch.py",
                     {"use_sim_time": use_sim_time}),
            _include("alm_sensors", "sensors.launch.py"),
            _include("alm_navigation", "ekf.launch.py",
                     {"use_sim_time": use_sim_time}),
            _include("alm_base_control", "base_control.launch.py"),
            _include("alm_mcu_interface", "mcu_interface.launch.py"),
        ]
    )
