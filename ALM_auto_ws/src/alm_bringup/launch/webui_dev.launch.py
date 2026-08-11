"""WebUI 읽기 전용 연동 개발용 스택. **로봇 하드웨어 없이** 화면을 검증한다.

    ros2 launch alm_bringup webui_dev.launch.py

무엇이 진짜고 무엇이 더미인가:

  진짜 | jetson_stats_publisher  이 머신이 Jetson 이므로 CPU/GPU/온도/전력은 실측이다
  진짜 | cmd_arbiter             시리얼이 필요 없으므로 실제 노드가 돈다
  진짜 | command_manager         〃 (4WIS 변환·안전 게이팅까지 실제로 수행)
  진짜 | map_publisher           활성 맵의 grid.pgm 을 /map 으로 발행
  진짜 | map_manager             maps/ 를 스캔해 /alm/map_inventory 로 발행
  진짜 | alm_web_backend         브라우저 → 로봇 **명령** 경로 (HTTP :8081)
  더미 | fake_mcu                mcu_bridge 대역. /mcu/state 를 만들어낸다
  더미 | pcd_replay              라이다 대역. 활성 맵의 cloud.pcd 를 스캔처럼 재생
                                 **기본 꺼짐.** use_pcd_replay:=true 로만 켜진다

라이다 점군을 보려면 실물을 띄운다:

    ros2 launch alm_sensors lidar.launch.py     # 먼저
    ros2 launch alm_bringup webui_dev.launch.py # 그다음

라이다 없이 화면만 볼 거라면 use_pcd_replay:=true — 단, 그때 화면의 파란
점군은 **저장된 맵의 재생본**이지 센서 출력이 아니다.

읽기와 쓰기는 서로 다른 포트로 나간다.

    읽기  브라우저 ──WS  :8765──▶ foxglove_bridge   구독 전용 (쓰기 구조적 차단)
    쓰기  브라우저 ──HTTP:8081──▶ alm_web_backend   토큰 + 웹 제어권 락

백엔드는 ALM_WEB_TOKEN 이 없으면 기동을 거부한다 (fail-closed).

    export ALM_WEB_TOKEN=$(openssl rand -hex 16)

정말로 인증 없이 띄우려면 `use_backend_auth:=false` — 격리된 개발 환경에서만.

하드웨어가 붙으면 `use_fake_mcu:=false` 로 더미를 끄고 robot.launch.py 를
대신 띄운다. 화면 코드는 그대로여야 정상이다.

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
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

# 맵 경로 규약은 alm_navigation 이 소유한다 — 여기서 다시 정의하지 않는다.
sys.path.insert(0, os.path.join(
    get_package_share_directory("alm_navigation"), "launch"))
import map_layout  # noqa: E402


def executable_path(package, name):
    return os.path.join(get_package_prefix(package), "lib", package, name)


def generate_launch_description():
    bringup_share = get_package_share_directory("alm_bringup")
    nav_share = get_package_share_directory("alm_navigation")
    base_share = get_package_share_directory("alm_base_control")

    maps_root = map_layout.maps_root(nav_share)
    active = map_layout.active_map_paths(maps_root)
    default_pcd = active.cloud if active else ""
    default_map = active.grid_yaml if active else ""
    foxglove_cfg = os.path.join(bringup_share, "config", "foxglove_webui.yaml")

    use_fake_mcu = LaunchConfiguration("use_fake_mcu")
    use_pcd_replay = LaunchConfiguration("use_pcd_replay")
    use_base_control = LaunchConfiguration("use_base_control")
    use_bridge = LaunchConfiguration("use_bridge")
    use_backend = LaunchConfiguration("use_backend")

    return LaunchDescription([
        DeclareLaunchArgument("use_fake_mcu", default_value="true",
                              description="MCU 미연결 시 /mcu/state 더미 발행"),
        # 기본값이 false 인 이유: 가짜가 기본이면 언젠가 가짜를 실측으로 오인한다.
        # 실제로 그랬다 — 활성 맵이 alm_lab 인 채로 집에서 띄웠더니, 재생된 랩실
        # 점군이 /livox/lidar 로 흘러서 "실시간 스캔"으로 보였다. 토픽 이름만
        # 보면 구분이 안 된다. 재생은 반드시 명시해야 켜진다.
        DeclareLaunchArgument("use_pcd_replay", default_value="false",
                              description="저장된 pcd 를 /livox/lidar 로 재생 (라이다 대역). "
                                          "명시할 때만 켜진다 — 실측과 구분이 안 되기 때문"),
        DeclareLaunchArgument("use_base_control", default_value="true",
                              description="cmd_arbiter + command_manager 기동"),
        DeclareLaunchArgument("use_bridge", default_value="true",
                              description="foxglove_bridge 기동 (읽기)"),
        DeclareLaunchArgument("use_backend", default_value="true",
                              description="alm_web_backend 기동 (쓰기)"),
        DeclareLaunchArgument("backend_port", default_value="8081"),
        DeclareLaunchArgument("use_backend_auth", default_value="true",
                              description="false 면 토큰 없이 기동 — 격리 환경에서만"),
        DeclareLaunchArgument("pcd", default_value=default_pcd),
        DeclareLaunchArgument("map", default_value=default_map),
        DeclareLaunchArgument("lidar_z", default_value="0.5",
                              description="base_link 기준 라이다 높이 (lidar.launch.py 와 동일해야 함)"),

        DeclareLaunchArgument("use_voice", default_value="true",
                              description="음성 안내 (블루투스 스피커) 기동"),

        # ---- 실측: Jetson 리소스 ----
        Node(
            executable=executable_path("alm_bringup", "jetson_stats_publisher.py"),
            name="jetson_stats_publisher",
            output="screen",
            parameters=[{"topic": "/alm/jetson_stats"}, {"rate_hz": 1.0}],
        ),

        # ---- 음성 안내 ----
        # 실차에서는 화면을 못 본다. 명령이 로봇까지 갔는지 귀로 확인한다.
        # piper 나 스피커가 없어도 노드는 조용히 살아 있는다 (로그만 남긴다).
        Node(
            executable=executable_path("alm_bringup", "voice_announcer.py"),
            name="voice_announcer",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_voice")),
            parameters=[os.path.join(bringup_share, "config", "voice.yaml")],
        ),

        # ---- 실측: maps/ 자산 목록 ----
        Node(
            executable=executable_path("alm_navigation", "map_manager.py"),
            name="map_manager",
            output="screen",
            parameters=[{"maps_root": maps_root}],
        ),

        # ---- 실측: 활성 맵의 저장된 3D 점군 ----
        # 2D 는 map_publisher 가 /map 으로 내는데 3D 는 그동안 내는 노드가 없었다.
        # (pcd_replay 가 가짜 스캔으로 흘려서 있는 것처럼 보였을 뿐이다)
        Node(
            executable=executable_path("alm_navigation", "prior_cloud_publisher.py"),
            name="prior_cloud_publisher",
            output="screen",
            parameters=[{"maps_root": maps_root}],
        ),

        # ---- 실측: 저장된 2D 맵 ----
        # 추종 모드로 띄운다 (yaml 을 안 준다). 활성 맵을 바꾸면 /map 도 따라
        # 바뀌고, 격자가 없는 맵이면 빈 맵을 내보내 이전 맵이 남지 않는다.
        # 예전에는 launch 시점의 yaml 하나에 고정돼서, 맵을 바꿔도 2D 는 그대로
        # 이전 맵을 보여주고 있었다.
        Node(
            executable=executable_path("alm_navigation", "map_publisher.py"),
            name="map_publisher",
            output="screen",
            parameters=[{"maps_root": maps_root}],
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
        # map_publisher 와 같은 이유로 cloud.pcd 가 없으면 띄우지 않는다.
        *([Node(
            executable=executable_path("alm_navigation", "pcd_replay.py"),
            name="pcd_replay",
            output="screen",
            condition=IfCondition(use_pcd_replay),
            parameters=[
                {"pcd": LaunchConfiguration("pcd")},
                {"sensor_height": LaunchConfiguration("lidar_z")},
            ],
        )] if default_pcd and os.path.isfile(default_pcd) else []),
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

        # ---- 읽기: 브라우저로 나가는 텔레메트리 (구독 전용 allowlist) ----
        Node(
            package="foxglove_bridge",
            executable="foxglove_bridge",
            name="foxglove_bridge",
            output="screen",
            condition=IfCondition(use_bridge),
            parameters=[foxglove_cfg],
        ),

        # ---- 쓰기: 브라우저에서 들어오는 명령 ----
        # 토큰 유무로 인자가 갈리므로 노드를 둘로 나눈다. --allow-no-auth 는
        # ROS 파라미터가 아니라 CLI 플래그라 조건부로 붙일 방법이 이것뿐이다.
        Node(
            executable=executable_path("alm_web_backend", "web_backend.py"),
            name="alm_web_backend",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_backend, "' == 'true' and '",
                LaunchConfiguration("use_backend_auth"), "' == 'true'"])),
            arguments=["--port", LaunchConfiguration("backend_port")],
        ),
        Node(
            executable=executable_path("alm_web_backend", "web_backend.py"),
            name="alm_web_backend",
            output="screen",
            condition=IfCondition(PythonExpression([
                "'", use_backend, "' == 'true' and '",
                LaunchConfiguration("use_backend_auth"), "' != 'true'"])),
            arguments=["--port", LaunchConfiguration("backend_port"), "--allow-no-auth"],
        ),
    ])
