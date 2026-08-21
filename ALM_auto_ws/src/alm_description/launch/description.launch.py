"""Publish the ALM 4WIS robot description and TF from the URDF.

  robot_state_publisher: base_link -> steer/wheel links (URDF 고정/조인트 TF)

실 로봇에서는 /joint_states 를 alm_mcu_interface 의 mcu_bridge 가 엔코더 값으로
발행합니다. 하드웨어 없이 RViz 로만 확인할 때는 standalone:=true 로 실행하면
joint_state_publisher_gui 로 조향/바퀴를 수동으로 움직일 수 있습니다.

    ros2 launch alm_description description.launch.py standalone:=true description_rviz:=true

★ 인자 이름이 `rviz` 가 아니라 `description_rviz` 인 이유 (2026-08-20):
  `IncludeLaunchDescription` 은 launch configuration 을 격리하지 않는다. 상위
  런치(alm_bringup/navigation.launch.py)가 `rviz` 를 먼저 선언하면 그 값이 여기까지
  새고, 아래 `DeclareLaunchArgument("rviz", ...)` 의 기본값은 '이미 설정됨'이라
  무시된다. 그래서 `rviz:=true` 로 자율주행을 띄우면 **RViz 가 두 개** 떴다:
  여기의 URDF 뷰(alm.rviz) + alm_navigation 의 관측용(navigation.rviz).
  6코어 Orin Nano 에서 RViz 2개는 그냥 낭비다. 이름을 고유하게 두어 끊는다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory("alm_description")
    default_xacro = os.path.join(pkg_share, "urdf", "alm_robot.urdf.xacro")
    default_rviz = os.path.join(pkg_share, "rviz", "alm.rviz")

    use_sim_time = LaunchConfiguration("use_sim_time")
    standalone = LaunchConfiguration("standalone")
    use_rviz = LaunchConfiguration("description_rviz")
    model = LaunchConfiguration("model")

    # ★ ParameterValue(value_type=str) 로 감싸는 것이 필수다.
    # Command() 는 xacro 가 뱉은 URDF **문자열**을 돌려주는데, launch_ros 는 파라미터
    # 값을 기본적으로 YAML 로 파싱한다. URDF 는 '<?xml ...' 로 시작하므로 YAML 이 아니고,
    # 그대로 두면 기동이 이 에러로 죽는다:
    #     Unable to parse the value of parameter robot_description as yaml.
    # value_type=str 은 "파싱하지 말고 문자열 그대로 넘겨라" 라는 뜻이다.
    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", model]), value_type=str
        ),
        "use_sim_time": use_sim_time,
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("standalone", default_value="false",
                                  description="하드웨어 없이 joint_state_publisher_gui 사용"),
            # ★ 이름 주의: `rviz` 로 되돌리지 말 것 (파일 상단 docstring 참고).
            DeclareLaunchArgument("description_rviz", default_value="false"),
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
