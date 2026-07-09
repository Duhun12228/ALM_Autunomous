"""자율주행 (단계 B): FAST-LIO-Localization + Nav2. AMCL/EKF 미사용.

구성:
  map_server            : 저장된 2D 맵(map.pgm/yaml) -> global costmap static layer
  localization.launch.py: FAST-LIO-Localization (map->odom + odom->base_link)
                          ※ AMCL + robot_localization EKF 를 대체
  nav2 navigation_launch: planner/controller/behavior/bt (본체 변경 없음) -> /cmd_vel

전제: /livox/lidar (+time) 와 /livox/imu 가 이미 발행 중
(alm_bringup/robot.launch.py -> alm_sensors/lidar.launch.py).

    ros2 launch alm_navigation navigation.launch.py \
        map:=<2D map.yaml>  map_pcd:=<3D map.pcd>  initial_a:=<yaw>
  초기 pose 는 initial_x/y/z/a 인자 또는 RViz "2D Pose Estimate".
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    default_params = os.path.join(nav_share, "config", "nav2.yaml")
    default_map = os.path.join(nav_share, "maps", "my_map.yaml")
    default_map_pcd = os.path.join(nav_share, "maps", "alm_3d_map.pcd")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    map_yaml = LaunchConfiguration("map")
    map_pcd = LaunchConfiguration("map_pcd")

    nav2_navigation_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )
    loc_launch = PathJoinSubstitution(
        [FindPackageShare("alm_navigation"), "launch", "localization.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map,
                                  description="2D 맵(pcd2pgm 산출 .yaml) - global costmap용"),
            DeclareLaunchArgument("map_pcd", default_value=default_map_pcd,
                                  description="3D prior 맵(.pcd) - FAST-LIO-Localization용"),
            DeclareLaunchArgument("initial_x", default_value="0.0"),
            DeclareLaunchArgument("initial_y", default_value="0.0"),
            DeclareLaunchArgument("initial_z", default_value="0.0"),
            DeclareLaunchArgument("initial_a", default_value="0.0"),

            # ---- 저장된 2D 맵 서버 (global costmap static layer) ----
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[params_file, {"yaml_filename": map_yaml},
                            {"use_sim_time": use_sim_time}],
            ),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_localization",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time},
                            {"autostart": True},
                            {"node_names": ["map_server"]}],
            ),

            # ---- FAST-LIO-Localization (map->odom + odom->base_link) ----
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(loc_launch),
                launch_arguments={
                    "map_pcd": map_pcd,
                    "initial_x": LaunchConfiguration("initial_x"),
                    "initial_y": LaunchConfiguration("initial_y"),
                    "initial_z": LaunchConfiguration("initial_z"),
                    "initial_a": LaunchConfiguration("initial_a"),
                }.items(),
            ),

            # ---- Nav2 네비게이션 코어 (planner/controller/bt) ----
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_navigation_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    "params_file": params_file,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
