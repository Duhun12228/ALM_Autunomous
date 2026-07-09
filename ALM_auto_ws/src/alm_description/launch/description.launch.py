"""Publish the ALM 4WIS robot description and TF from the URDF.

  robot_state_publisher: base_link -> steer/wheel links (URDF 고정/조인트 TF)

실 로봇에서는 /joint_states 를 alm_mcu_interface 의 mcu_bridge 가 엔코더 값으로
발행합니다. 하드웨어 없이 RViz 로만 확인할 때는 standalone:=true 로 실행하면
joint_state_publisher_gui 로 조향/바퀴를 수동으로 움직일 수 있습니다.

    ros2 launch alm_description description.launch.py standalone:=true rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("alm_description")
    default_xacro = os.path.join(pkg_share, "urdf", "alm_robot.urdf.xacro")
    default_rviz = os.path.join(pkg_share, "rviz", "alm.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    standalone = LaunchConfiguration("standalone")
    use_rviz = LaunchConfiguration("rviz")
    model = LaunchConfiguration("model")

    robot_description = {
        "robot_description": Command(["xacro ", model]),
        "use_sim_time": use_sim_time,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("standalone", default_value="false",
                                  description="하드웨어 없이 joint_state_publisher_gui 사용"),
            DeclareLaunchArgument("rviz", default_value="false"),
            DeclareLaunchArgument("model", default_value=default_xacro),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[robot_description],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                condition=IfCondition(standalone),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2",
                arguments=["-d", PathJoinSubstitution([FindPackageShare("alm_description"), "rviz", "alm.rviz"])],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(use_rviz),
                output="screen",
            ),
        ]
    )
