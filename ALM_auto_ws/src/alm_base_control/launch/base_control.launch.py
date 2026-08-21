"""base_control 실행.

  cmd_arbiter     : 자율(/cmd_vel,/drive_mode) vs 텔레옵(/cmd_vel_teleop,...) 동작권 중재
                    -> /cmd_vel_mux + /drive_mode_mux
  command_manager : /cmd_vel_mux + /drive_mode_mux -> /mcu/command (4WIS 변환 + 안전 게이팅)
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def executable_path(name):
    return os.path.join(get_package_prefix("alm_base_control"), "lib", "alm_base_control", name)


def generate_launch_description():
    share = get_package_share_directory("alm_base_control")
    default_cfg = os.path.join(share, "config", "base_control.yaml")

    return LaunchDescription(
        [
            # ★ 인자 이름에 패키지 접두어를 붙인다 — `params_file` 로 두면 안 된다.
            # `IncludeLaunchDescription` 은 launch configuration 을 **격리하지 않는다.**
            # 안에서 실행된 `DeclareLaunchArgument` 가 부모 스코프에 그대로 남아
            # 뒤따르는 형제 include 로 새고, 거기서 같은 이름을 선언해도
            # '이미 설정됨'이라 기본값이 무시된다. 먼저 선언한 쪽이 전부 이긴다.
            #
            # 실제로 났던 사고 (2026-08-20): robot.launch.py 가 이 파일을 먼저
            # include -> `params_file`=base_control.yaml 이 남음 -> 그 뒤 Nav2 와
            # mcu_interface 가 **전부 base_control.yaml 을 받았다.**
            #     controller_server: No critics defined for FollowPath
            #     lifecycle_manager_navigation: Failed to bring up all requested nodes.
            # mcu_bridge 도 조용히 잘못된 파일을 받고 있었다(에러 없이 기본값 사용).
            #
            # GroupAction(scoped=True) 로 감싸는 해법은 **쓰면 안 된다.**
            # localization.launch.py 의 TimerAction 처럼 지연 실행되는 노드는
            # 스코프가 pop 된 뒤에 설정을 읽어 'auto_init does not exist' 로 죽는다.
            # 이름을 고유하게 두는 것이 유일하게 안전한 방법이다.
            DeclareLaunchArgument("base_control_params_file", default_value=default_cfg),
            Node(
                executable=executable_path("cmd_arbiter.py"),
                name="cmd_arbiter",
                output="screen",
                parameters=[LaunchConfiguration("base_control_params_file")],
            ),
            Node(
                executable=executable_path("command_manager.py"),
                name="command_manager",
                output="screen",
                parameters=[LaunchConfiguration("base_control_params_file")],
            ),
        ]
    )
