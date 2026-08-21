"""FAST-LIO Localization (prior map.pcd 안에서 재측위) — 단계 B 측위.

전제: /livox/lidar (PointCloud2 + per-point time) 와 /livox/imu 가 이미 발행 중
(alm_bringup/robot.launch.py -> alm_sensors/lidar.launch.py).

구성:
  teaser_fpfh_localizer: FPFH 전역 대응점 + TEASER++ + 지역 GICP
                         -> /icp_result (pose)
  transform_publisher : /icp_result -> TF map->odom
  fastlio_mapping     : /livox/lidar + /livox/imu -> odom->base_link TF, /Odometry
                        (locate_in_prior_map 모드, config fastlio_relocalization.yaml)
  초기위치 DB          : fpfh_map_builder 로 prior map의 FPFH를 사전 생성

  => AMCL + robot_localization EKF 를 대체한다 (map->odom + odom->base_link 전부 담당).

초기 pose:
  auto_init:=true + fpfh_db_prefix (FPFH+TEASER++ 전역 자동 측위)

맵 선택:
  인자를 안 주면 maps/active.yaml 의 활성 맵에서 map_pcd 와 fpfh_db_prefix 를
  조립한다. **map_pcd 하나가 teaser_fpfh_localizer 와 fast_lio 양쪽에 동시에
  들어간다** — 둘이 서로 다른 맵을 보는 구성이 애초에 만들어지지 않게 하기
  위해서다 (fast_lio_node 주석 참조).

  웹에서 띄울 때(alm_web_backend)는 active.yaml 을 다시 읽지 않고 백엔드가
  해석한 절대경로를 인자로 명시해 넘긴다. launch 파싱 시점과 요청 시점 사이에
  활성 맵이 바뀌는 경합을 없애기 위함이다.
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import map_layout  # noqa: E402  (같은 디렉터리의 공용 헬퍼)


def read_fpfh_meta(path):
    """fpfh_map.meta 의 key=value. 없으면 빈 dict (조용히 기본값으로 간다)."""
    values = {}
    try:
        with open(path) as handle:
            for line in handle:
                key, _, value = line.strip().partition("=")
                if key:
                    values[key] = value
    except OSError:
        pass
    return values


def generate_launch_description():
    nav_share = get_package_share_directory("alm_navigation")
    default_config = os.path.join(nav_share, "config", "fastlio_relocalization.yaml")

    # 경로는 maps/active.yaml 이 가리키는 맵에서 조립한다 — 맵을 바꾸려면
    # active.yaml 한 줄만 고치면 되고, 개별 인자는 그대로 덮어쓸 수 있다.
    maps_root = map_layout.maps_root(nav_share)
    active = map_layout.active_map_paths(maps_root)
    default_map = active.cloud if active else ""
    default_fpfh_db = active.fpfh_prefix if active else ""

    fastlio_config = LaunchConfiguration("fastlio_config")
    map_pcd = LaunchConfiguration("map_pcd")
    auto_init = LaunchConfiguration("auto_init")
    fpfh_db_prefix = LaunchConfiguration("fpfh_db_prefix")
    accum_frames = LaunchConfiguration("accum_frames")

    args = [
        DeclareLaunchArgument("fastlio_config", default_value=default_config),
        DeclareLaunchArgument("map_pcd", default_value=default_map,
                              description="prior 3D 점군맵(.pcd) 경로 (icp + fast_lio 동일해야 함)"),
        DeclareLaunchArgument("auto_init", default_value="true",
                              description="FPFH+TEASER++ 초기위치 자동특정"),
        DeclareLaunchArgument("fpfh_db_prefix", default_value=default_fpfh_db,
                              description="fpfh_map_builder 출력 prefix (확장자 제외)"),
        DeclareLaunchArgument("accum_frames", default_value="10",
                              description="전역 정합 전에 정지 상태로 누적할 LiDAR 프레임 수"),
        # ⚠ 아래 다섯은 **DB 를 만들 때 쓴 값과 반드시 같아야 한다.**
        #   FPFH 는 "같은 방식으로 요약한 것끼리만" 비교가 성립한다. 맵 쪽 DB 를
        #   voxel 0.3 으로 만들고 스캔 쪽을 0.5 로 요약하면, 같은 장소를 봐도
        #   히스토그램 모양이 달라 대응점이 안 잡힌다. 그런데 노드는 정상 기동하고
        #   로그도 깨끗하다 — 그냥 영영 안 붙는다.
        #
        #   그래서 사람이 맞추게 두지 않는다. **비워 두면 fpfh_map.meta 에 기록된
        #   생성 당시 값을 읽어 쓴다** — DB 가 자기 파라미터를 들고 다니므로
        #   어긋날 수가 없다. 값을 명시하면 그것이 이긴다 (실험용 탈출구).
        DeclareLaunchArgument("feature_voxel", default_value="",
                              description="비우면 fpfh_map.meta 의 voxel 을 쓴다"),
        DeclareLaunchArgument("normal_radius", default_value="",
                              description="비우면 fpfh_map.meta 의 normal_radius 를 쓴다"),
        DeclareLaunchArgument("feature_radius", default_value="",
                              description="비우면 fpfh_map.meta 의 feature_radius 를 쓴다"),
        DeclareLaunchArgument("z_min", default_value="",
                              description="비우면 fpfh_map.meta 의 z_min 을 쓴다"),
        DeclareLaunchArgument("z_max", default_value="",
                              description="비우면 fpfh_map.meta 의 z_max 를 쓴다"),
    ]

    def build(context):
        """DB 의 전처리 파라미터를 읽어 노드를 만든다.

        OpaqueFunction 인 이유: fpfh_db_prefix 가 인자로 덮어써질 수 있으므로,
        어느 DB 를 쓰는지는 **launch 시점에야** 확정된다. 파싱 시점에 active.yaml
        만 보고 meta 를 읽으면, 다른 맵을 인자로 준 경우 엉뚱한 DB 의 파라미터를
        쓰게 된다.
        """
        prefix = context.perform_substitution(fpfh_db_prefix)
        meta = read_fpfh_meta(prefix + ".meta")

        def resolve(arg_name, meta_key, fallback):
            """인자가 비어 있으면 meta, meta 도 없으면 fallback."""
            given = context.perform_substitution(LaunchConfiguration(arg_name)).strip()
            if given:
                return float(given)
            try:
                return float(meta[meta_key])
            except (KeyError, TypeError, ValueError):
                return fallback

        features = {
            "feature_voxel": resolve("feature_voxel", "voxel", 0.5),
            "normal_radius": resolve("normal_radius", "normal_radius", 1.0),
            "feature_radius": resolve("feature_radius", "feature_radius", 2.5),
            "z_min": resolve("z_min", "z_min", -0.35),
            "z_max": resolve("z_max", "z_max", 1.0),
        }

        # /icp_result -> TF map->odom
        transform_publisher = Node(
            package="icp_relocalization",
            executable="transform_publisher",
            name="transform_publisher",
            output="screen",
            parameters=[{"map_frame_id": "map"}, {"odom_frame_id": "odom"}],
        )

        # 초기 추정 없는 전역 FPFH+TEASER++ 정합 후 지역 GICP -> /icp_result
        teaser_localizer = Node(
            package="icp_relocalization",
            executable="teaser_fpfh_localizer",
            name="teaser_fpfh_localizer",
            output="screen",
            condition=IfCondition(auto_init),
            parameters=[
                {"map_path": map_pcd},
                {"fpfh_db_prefix": fpfh_db_prefix},
                {"lidar_topic": "/livox/lidar"},
                {"map_frame_id": "map"},
                {"accum_frames": ParameterValue(accum_frames, value_type=int)},
                # DB 생성 당시 값 (인자 선언부 ⚠ 참조).
                features,
                # 아래 셋은 **스캔 쪽에만** 적용된다 (맵 DB 에는 해당 없음).
                {"scan_min_range": 0.5},
                {"scan_max_range": 10.0},
                {"min_curvature": 0.0},
                {"max_scan_features": 1500},
                # FPFH 대응점과 TEASER++ 전역 정합.
                {"feature_ratio_threshold": 0.95},
                {"min_feature_matches": 20},
                {"max_feature_matches": 400},
                {"teaser_noise_bound": 0.5},
                {"teaser_exact_clique": False},
                {"teaser_max_clique_time_limit": 2.0},
                {"min_teaser_inliers": 6},
                # TEASER 결과 독립 검증과 지역 GICP.
                {"validation_inlier_distance": 0.50},
                {"validation_min_inlier_ratio": 0.20},
                {"validation_max_rmse": 0.35},
                {"local_map_radius": 12.0},
                {"gicp_max_correspondence": 1.0},
                {"gicp_fitness_threshold": 0.30},
                # 잘못된 단발 결과를 막기 위해 새 누적 스캔 두 번에서 일치해야 한다.
                {"consistent_result_count": 2},
                {"consistency_translation": 0.50},
                {"consistency_rotation_deg": 5.0},
            ],
        )

        # FAST-LIO 측위 모드 (odom->base_link, /Odometry)
        #
        # ⚠ prior_map_path 를 map_pcd 로 **덮어쓰는 것이 핵심이다.** yaml 에도 같은
        #   키가 있지만 그건 절대경로 한 줄이라 활성 맵을 따라오지 못한다. 예전에
        #   그 상태로 두었더니 teaser_fpfh_localizer 는 cschool 로, fast_lio 는
        #   alm_lab 으로 돌 수 있는 구성이 되어 있었다 — 둘 다 정상 기동하고
        #   로그도 깨끗해서, 어긋난 것을 알 방법이 없는 종류의 고장이다.
        #
        #   parameters 리스트는 뒤엣것이 이긴다 (yaml -> 오버라이드 순서).
        fast_lio_node = Node(
            package="fast_lio",
            executable="fastlio_mapping",
            name="fastlio_localization",
            output="screen",
            parameters=[fastlio_config, {"prior_map_path": map_pcd}],
        )

        # 전역 로컬라이저/fast_lio 는 map 로딩 후 시작 (transform_publisher 먼저).
        return [transform_publisher,
                TimerAction(period=3.0, actions=[teaser_localizer, fast_lio_node])]

    return LaunchDescription(args + [OpaqueFunction(function=build)])
