"""WebUI 읽기 전용 연동 개발용 스택. **로봇 하드웨어 없이** 화면을 검증한다.

    ros2 launch alm_bringup webui_dev.launch.py

무엇이 진짜고 무엇이 더미인가:

  진짜 | jetson_stats_publisher  이 머신이 Jetson 이므로 CPU/GPU/온도/전력은 실측이다
  진짜 | cmd_arbiter             시리얼이 필요 없으므로 실제 노드가 돈다
  진짜 | command_manager         〃 (4WIS 변환·안전 게이팅까지 실제로 수행)
  진짜 | map_publisher           저장된 alm_map.pgm 을 /map 으로 발행
  더미 | fake_mcu                mcu_bridge 대역. /mcu/state 를 만들어낸다
  더미 | pcd_replay              라이다 대역. alm_3d_map.pcd 를 스캔처럼 재생

하드웨어가 붙으면 `use_fake_mcu:=false use_pcd_replay:=false` 로 끄고
robot.launch.py 를 대신 띄운다. 화면 코드는 그대로여야 정상이다.

  ⚠ 띄우기 전에 반드시 `ros2 node list` 로 기존 스택을 확인할 것.

     이미 robot.launch.py 나 base_control.launch.py 가 돌고 있으면 cmd_arbiter /
     command_manager 가 같은 이름으로 두 개가 되고, mcu_bridge 가 살아 있으면
     fake_mcu 와 함께 /mcu/state 를 두 노드가 발행한다. ROS 2 는 이름이 겹쳐도
     막지 않으므로 조용히 상태가 뒤섞인다 — 재현이 어려운 종류의 사고다.

       이미 떠 있다면:  ros2 launch alm_bringup webui_dev.launch.py \
                          use_base_control:=false use_fake_mcu:=false
       깨끗이 시작하려면: 기존 launch 를 먼저 종료하고 기본값으로 실행
"""

import os

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def executable_path(package, name):
    return os.path.join(get_package_prefix(package), "lib", package, name)


def generate_launch_description():
    bringup_share = get_package_share_directory("alm_bringup")
    nav_share = get_package_share_directory("alm_navigation")
    base_share = get_package_share_directory("alm_base_control")

    default_pcd = os.path.join(nav_share, "maps", "alm_3d_map.pcd")
    default_map = os.path.join(nav_share, "maps", "alm_map.yaml")
    foxglove_cfg = os.path.join(bringup_share, "config", "foxglove_webui.yaml")

    use_fake_mcu = LaunchConfiguration("use_fake_mcu")
    use_pcd_replay = LaunchConfiguration("use_pcd_replay")
    use_base_control = LaunchConfiguration("use_base_control")
    use_bridge = LaunchConfiguration("use_bridge")

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_mcu", default_value="true",
                              description="MCU 미연결 시 /mcu/state 더미 발행"),
        DeclareLaunchArgument("use_pcd_replay", default_value="true",
                              description="라이다 미연결 시 저장된 pcd 재생"),
        DeclareLaunchArgument("use_base_control", default_value="true",
                              description="cmd_arbiter + command_manager 기동"),
        DeclareLaunchArgument("use_bridge", default_value="true",
                              description="foxglove_bridge 기동"),
        DeclareLaunchArgument("pcd", default_value=default_pcd),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("lidar_z", default_value="0.5",
                              description="base_link 기준 라이다 높이 (lidar.launch.py 와 동일해야 함)"),

        # ---- 실측: Jetson 리소스 ----
        Node(
            executable=executable_path("alm_bringup", "jetson_stats_publisher.py"),
            name="jetson_stats_publisher",
            output="screen",
            parameters=[{"topic": "/alm/jetson_stats"}, {"rate_hz": 1.0}],
        ),

        # ---- 실측: 저장된 2D 맵 ----
        Node(
            executable=executable_path("alm_navigation", "map_publisher.py"),
            name="map_publisher",
            output="screen",
            parameters=[{"yaml": LaunchConfiguration("map")}],
        ),

        # ---- 실제 노드: 동작권 중재 + 명령 변환 ----
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(base_share, "launch", "base_control.launch.py")),
            condition=IfCondition(use_base_control),
        ),

        # ---- 더미: MCU ----
        Node(
            executable=executable_path("alm_bringup", "fake_mcu.py"),
            name="fake_mcu",
            output="screen",
            condition=IfCondition(use_fake_mcu),
        ),

        # ---- 더미: 라이다 ----
        Node(
            executable=executable_path("alm_navigation", "pcd_replay.py"),
            name="pcd_replay",
            output="screen",
            condition=IfCondition(use_pcd_replay),
            parameters=[
                {"pcd": LaunchConfiguration("pcd")},
                {"sensor_height": LaunchConfiguration("lidar_z")},
            ],
        ),
        # lidar.launch.py 를 안 띄우므로 base_link->livox_frame 을 여기서 채운다
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_livox_tf",
            condition=IfCondition(use_pcd_replay),
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", LaunchConfiguration("lidar_z"),
                "--yaw", "0", "--pitch", "0", "--roll", "0",
                "--frame-id", "base_link", "--child-frame-id", "livox_frame",
            ],
        ),

        # ---- 브라우저로 나가는 유일한 경로 (읽기 전용 allowlist) ----
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            condition=IfCondition(use_bridge),
            parameters=[foxglove_cfg],
        ),
    ])
