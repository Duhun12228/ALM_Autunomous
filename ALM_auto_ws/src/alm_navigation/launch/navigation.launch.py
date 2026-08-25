"""자율주행 (단계 B): FAST-LIO-Localization + Nav2. AMCL/EKF 미사용.

구성:
  map_server            : 저장된 2D 맵(map.pgm/yaml) -> global costmap static layer
  localization.launch.py: FAST-LIO-Localization (map->odom + odom->base_link)
                          ※ AMCL + robot_localization EKF 를 대체
  nav2 navigation_launch: planner(SmacPlannerHybrid) / smoother(ConstrainedSmoother)
                          / controller(MPPI) / behavior / bt -> /cmd_vel

경로 파이프라인:
  ComputePathToPose(Hybrid-A*) -> SmoothPath(ConstrainedSmoother) -> FollowPath(MPPI)
연결은 alm_navigation/behavior_trees/navigate_*_w_smoothing.xml 이 담당하며,
bt_navigator 가 읽을 절대경로를 RewrittenYaml 로 nav2.yaml 에 주입합니다.
SmacPlannerLattice(TightSpace)의 lattice_filepath 도 같은 방식으로 주입합니다.

전제: /livox/lidar (+time) 와 /livox/imu 가 이미 발행 중
(alm_bringup/robot.launch.py -> alm_sensors/lidar.launch.py).

    ros2 launch alm_navigation navigation.launch.py \
        map:=<2D map.yaml> map_pcd:=<3D map.pcd> fpfh_db_prefix:=<DB prefix>
  초기 pose 는 FPFH+TEASER++ 전역 정합으로 자동 계산한다.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_layout  # noqa: E402  (같은 디렉터리의 공용 헬퍼)


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    default_params = os.path.join(nav_share, "config", "nav2.yaml")

    # maps/active.yaml 이 가리키는 맵에서 세 경로를 한꺼번에 얻는다.
    # (이전 기본값 my_map.yaml 은 실제로 존재한 적이 없었다.)
    maps_root = map_layout.maps_root(nav_share)
    active = map_layout.active_map_paths(maps_root)
    default_map = active.grid_yaml if active else ""
    default_map_pcd = active.cloud if active else ""
    default_fpfh_db = active.fpfh_prefix if active else ""

    bt_dir = os.path.join(nav_share, "behavior_trees")

    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("nav2_params_file")
    map_yaml = LaunchConfiguration("map")
    map_pcd = LaunchConfiguration("map_pcd")
    fpfh_db_prefix = LaunchConfiguration("fpfh_db_prefix")
    accum_frames = LaunchConfiguration("accum_frames")

    # nav2.yaml 의 default_nav_*_bt_xml 플레이스홀더를 설치된 BT 절대경로로 치환.
    # (SmoothPath 노드가 들어간 커스텀 트리 = ConstrainedSmoother 가 실제로 불리는 지점)
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key="",
        param_rewrites={
            "default_nav_to_pose_bt_xml": os.path.join(
                bt_dir, "navigate_to_pose_w_smoothing.xml"),
            "default_nav_through_poses_bt_xml": os.path.join(
                bt_dir, "navigate_through_poses_w_smoothing.xml"),
            # SmacPlannerLattice(TightSpace) 의 control set 절대경로.
            # YAML 에 $(find-pkg-share ...) 를 써도 파라미터 로딩 때는 풀리지 않으므로
            # BT 경로와 같은 방식으로 여기서 주입한다.
            "lattice_filepath": os.path.join(
                nav_share, "lattice_primitives", "alm_1.643m_diff.json"),
        },
        convert_types=True,
    )

    nav2_navigation_launch = PathJoinSubstitution(
        [FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"]
    )
    loc_launch = PathJoinSubstitution(
        [FindPackageShare("alm_navigation"), "launch", "localization.launch.py"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("nav2_params_file", default_value=default_params),
            DeclareLaunchArgument("map", default_value=default_map,
                                  description="2D 맵(pcd2pgm 산출 .yaml) - global costmap용"),
            DeclareLaunchArgument("map_pcd", default_value=default_map_pcd,
                                  description="3D prior 맵(.pcd) - FAST-LIO-Localization용"),
            DeclareLaunchArgument("fpfh_db_prefix", default_value=default_fpfh_db,
                                  description="map_pcd에서 생성한 FPFH DB prefix"),
            DeclareLaunchArgument("accum_frames", default_value="10"),
            # rviz:=true 면 자율주행 관측용 RViz 를 함께 띄운다
            # (경로 /plan · /plan_smoothed · /local_plan, global/local costmap,
            #  footprint, 2D Goal Pose 툴). 측위 확인용 localization.rviz 와 다르다.
            DeclareLaunchArgument(
                "record", default_value="true",
                description="주행 한 판을 기록하고 종료 시 자동 판정 리포트를 남긴다 "
                            "(run_recorder). logs/run_<시각>/summary.md"),
            DeclareLaunchArgument(
                "record_dir", default_value=os.path.expanduser("~/ALM_Autunomous/logs"),
                description="run_recorder 출력 디렉터리"),
            DeclareLaunchArgument("rviz", default_value="false",
                                  description="자율주행 관측용 RViz 동시 실행"),

            # ---- 저장된 2D 맵 서버 (global costmap static layer) ----
            Node(
                package="nav2_map_server",
                executable="map_server",
                name="map_server",
                output="screen",
                parameters=[configured_params, {"yaml_filename": map_yaml},
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
                    "fpfh_db_prefix": fpfh_db_prefix,
                    "accum_frames": accum_frames,
                    "auto_init": "true",
                }.items(),
            ),

            # ---- 관측용 RViz (선택) ----
            # 주행 기록 + 자동 판정. RViz 로는 안 보이는 것들을 남긴다 —
            # wz 클램프율 · 경로의 미관측 통과 비율 · spin 체류 대비 위치오차 감소 등.
            # 구독만 하므로 주행에 영향이 없다. record:=false 로 끌 수 있다.
            Node(
                package="alm_navigation",
                executable="run_recorder.py",
                name="run_recorder",
                output="screen",
                parameters=[{"out_dir": LaunchConfiguration("record_dir")},
                            {"map_yaml": LaunchConfiguration("map")}],
                condition=IfCondition(LaunchConfiguration("record")),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz2_navigation",
                output="log",
                arguments=["-d", PathJoinSubstitution(
                    [FindPackageShare("alm_navigation"), "rviz", "navigation.rviz"])],
                parameters=[{"use_sim_time": use_sim_time}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),

            # ---- Nav2 네비게이션 코어 (planner/smoother/controller/bt) ----
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav2_navigation_launch),
                launch_arguments={
                    "use_sim_time": use_sim_time,
                    # ★ 여기는 nav2_bringup(외부 패키지)의 인자 이름이므로 "params_file" 이
                    #   맞다. 위에서 우리 인자를 nav2_params_file 로 바꾼 것과 헷갈리지 말 것.
                    "params_file": configured_params,
                    "autostart": "true",
                    "use_composition": "False",
                }.items(),
            ),
        ]
    )
