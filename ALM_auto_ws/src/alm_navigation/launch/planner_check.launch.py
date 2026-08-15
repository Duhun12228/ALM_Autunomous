"""planner_check — **측위 없이** 전역경로가 나오는지만 떼어내 확인한다.

    ros2 launch alm_navigation planner_check.launch.py x:=0.0 y:=0.0 yaw:=0.0
    ros2 run alm_navigation plan_probe.py --goal -12.1 2.9 0     # 다른 터미널

무엇을 확인하고 무엇을 확인하지 않는가:

  확인함   활성 맵의 grid.pgm 이 global costmap 으로 제대로 들어가는가
           SmacPlannerHybrid 가 그 costmap 위에서 실제 경로를 내는가
           ConstrainedSmoother 가 그 경로를 받아 마감하는가
           계획 시간 · 경로 길이 · 최소 선회반경이 설계값 안에 있는가
  확인안함 측위 (map->odom 은 virtual_pose_publisher 가 **지어낸다**)
           센서 (obstacle_layer 는 관측이 없어 static layer 만 남는다)
           추종 (controller_server / bt_navigator 를 띄우지 않는다)

navigation.launch.py 와의 차이는 딱 둘이다. 측위 스택 대신
virtual_pose_publisher 를 쓰고, 주행에 필요한 노드(controller/bt/behavior)를
띄우지 않는다. **파라미터 파일은 실차와 같은 nav2.yaml 을 그대로 쓴다** —
검증용으로 값을 고쳐 쓰면 무엇을 검증한 것인지 알 수 없게 된다.

  ##경고## 이 스택은 /cmd_vel 을 내지 않으므로 로봇은 움직이지 않는다.
  그래도 map->odom 은 가짜다 — 실차 스택과 **동시에** 띄우면 TF 가 싸운다.

obstacle_layer 는 관측 소스(/scan, /livox/lidar)가 없어도 조용히 비어 있는다
(expected_update_rate 기본값 0 = 만료 없음). 그래서 라이다 없이도 global
costmap 은 static + inflation 으로 정상 동작한다.
"""

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_layout  # noqa: E402  (같은 디렉터리의 공용 헬퍼)


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    default_params = os.path.join(nav_share, "config", "nav2.yaml")

    maps_root = map_layout.maps_root(nav_share)
    active = map_layout.active_map_paths(maps_root)
    default_map = active.grid_yaml if active else ""

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")

    virtual_pose = os.path.join(
        get_package_prefix("alm_navigation"), "lib", "alm_navigation",
        "virtual_pose_publisher.py")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("params_file", default_value=default_params,
                              description="실차와 같은 nav2.yaml 을 그대로 쓴다"),
        DeclareLaunchArgument("map", default_value=default_map,
                              description="2D 맵 yaml (기본: maps/active.yaml 의 활성 맵)"),
        DeclareLaunchArgument("x", default_value="0.0",
                              description="가상 초기위치 x [m] (map 기준)"),
        DeclareLaunchArgument("y", default_value="0.0"),
        DeclareLaunchArgument("yaw", default_value="0.0", description="가상 초기 자세 [rad]"),
        DeclareLaunchArgument("use_smoother", default_value="true",
                              description="ConstrainedSmoother 도 띄운다 (plan_probe --smooth)"),

        # ---- 저장된 2D 맵 (global costmap static layer) ----
        Node(
            package="nav2_map_server",
            executable="map_server",
            name="map_server",
            output="screen",
            parameters=[params_file, {"yaml_filename": map_yaml},
                        {"use_sim_time": use_sim_time}],
        ),

        # ---- 가상 초기위치: map -> odom -> base_link ----
        Node(
            executable=virtual_pose,
            name="virtual_pose_publisher",
            output="screen",
            # launch 인자는 문자열이라 그대로 넘기면 노드의 double 선언과 타입이
            # 어긋나 기동 중에 죽는다.
            parameters=[{"x": ParameterValue(LaunchConfiguration("x"), value_type=float)},
                        {"y": ParameterValue(LaunchConfiguration("y"), value_type=float)},
                        {"yaw": ParameterValue(LaunchConfiguration("yaw"), value_type=float)},
                        {"use_sim_time": use_sim_time}],
        ),

        # ---- 전역 경로계획 (global costmap 포함) ----
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            output="screen",
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        # ---- 경로 마감 ----
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_smoother")),
            parameters=[params_file, {"use_sim_time": use_sim_time}],
        ),

        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_planner_check",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time},
                        {"autostart": True},
                        # smoother 를 끄면 lifecycle_manager 가 그 노드를 기다리다
                        # 영원히 configure 에 머문다. 목록도 같이 갈라야 한다.
                        {"node_names": ["map_server", "planner_server", "smoother_server"]}],
            condition=IfCondition(LaunchConfiguration("use_smoother")),
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_planner_check",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time},
                        {"autostart": True},
                        {"node_names": ["map_server", "planner_server"]}],
            condition=UnlessCondition(LaunchConfiguration("use_smoother")),
        ),
    ])
