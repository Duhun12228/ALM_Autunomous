"""측위 기동 전 점검.

측위는 실패해도 **조용히** 실패한다는 것이 이 파일의 존재 이유다. 짝이 안 맞는
FPFH DB 로 teaser_fpfh_localizer 를 띄우면 노드는 멀쩡히 뜨고 로그도 깨끗한데
정합만 영영 안 붙는다. 라이다가 안 떠 있어도 똑같이 조용하다. 그 상태로 몇 분을
기다리다 로그를 뒤지게 만드는 대신, 기동 **전에** 이유를 말하고 거부한다.

판정 규칙은 map_manager 것을 그대로 빌려 쓴다. 여기 다시 구현하면 언젠가 둘이
갈라져서 "화면의 맵 카드는 짝 맞음인데 측위 기동은 거부" 같은 상태가 된다.
"""

import os
import sys

from ament_index_python.packages import get_package_prefix


class PreflightError(Exception):
    """기동 전 점검 실패. 사용자에게 그대로 보여줄 문장을 담는다."""


_map_manager = None


def map_manager_module():
    """alm_navigation 의 map_manager 를 모듈로 빌려 온다.

    노드 스크립트지만 import 시점에는 아무것도 실행하지 않는다 (main() 은
    __main__ 가드 안에 있다). 설치 위치는 lib/alm_navigation 이다 — launch 와
    공유하는 map_layout 이 share/alm_navigation/launch 에 있는 것과는 다르다.
    """
    global _map_manager                                  # noqa: PLW0603
    if _map_manager is None:
        lib = os.path.join(get_package_prefix("alm_navigation"), "lib", "alm_navigation")
        if lib not in sys.path:
            sys.path.insert(0, lib)
        import map_manager                               # noqa: PLC0415
        _map_manager = map_manager
    return _map_manager


def _thousands(value):
    return f"{value:,}"


# meta 의 키 → localization.launch.py 의 인자 이름. 이름이 하나만 다른 것은
# voxel/feature_voxel 인데, DB 쪽은 '맵을 얼마로 줄였나'고 localizer 쪽은
# '특징 추출용 격자'라서 그렇게 붙어 있다. 같은 값이어야 한다.
FEATURE_PARAM_KEYS = {
    "voxel": "feature_voxel",
    "normal_radius": "normal_radius",
    "feature_radius": "feature_radius",
    "z_min": "z_min",
    "z_max": "z_max",
}


def _feature_params(meta):
    """meta 에 기록된 전처리 파라미터를 launch 인자 이름으로 돌려준다.

    읽히지 않는 항목은 **넣지 않는다.** 기본값으로 채우면 옛 형식의 meta 를 만난
    순간 조용히 잘못된 값으로 돌아간다 — 빠뜨리면 launch 의 기본값이 쓰이고,
    그건 적어도 소스에 적혀 있어서 추적할 수 있다.
    """
    params = {}
    for meta_key, arg_name in FEATURE_PARAM_KEYS.items():
        raw = meta.get(meta_key, "")
        try:
            params[arg_name] = float(raw)
        except (TypeError, ValueError):
            continue
    return params


def check_assets(paths):
    """맵 하나가 측위에 쓸 수 있는 상태인지. 못 쓰면 PreflightError.

    성공하면 화면에 그대로 올릴 요약을 돌려준다.
    """
    mm = map_manager_module()

    if not os.path.isfile(paths.cloud):
        raise PreflightError(
            f"'{paths.name}' 에 cloud.pcd 가 없습니다. 먼저 매핑하고 저장하세요.")

    missing = []
    if not os.path.isfile(paths.fpfh_meta):
        missing.append(os.path.basename(paths.fpfh_meta))
    missing += [os.path.basename(path) for path in paths.fpfh_files
                if not os.path.isfile(path)]
    if missing:
        raise PreflightError(
            f"'{paths.name}' 의 측위 DB 가 없습니다 ({', '.join(missing)}). "
            f"'FPFH 측위 DB 생성' 을 먼저 실행하세요.")

    meta = mm.read_meta(paths.fpfh_meta)
    header = mm.read_pcd_header(paths.cloud)
    cloud_points = header.get("points", 0)

    # 1) DB 가 기록해 둔 원본 경로. 가장 강한 증거다 — 남의 맵으로 만든 DB 를
    #    복사해 온 경우는 점 개수나 mtime 으로는 안 걸릴 수 있다.
    built_from = meta.get("map_path", "")
    if built_from and os.path.realpath(built_from) != os.path.realpath(paths.cloud):
        raise PreflightError(
            f"측위 DB 가 다른 맵으로 만들어졌습니다 (DB 원본: {built_from}). "
            f"'{paths.name}' 에서 FPFH DB 를 다시 만드세요.")

    # 2) 점 개수 대조 (map_manager 의 stale 판정 1번과 같은 규칙)
    try:
        built_points = int(meta.get("map_input_points", ""))
    except ValueError:
        built_points = 0
    if cloud_points and built_points and built_points != cloud_points:
        raise PreflightError(
            f"측위 DB 는 {_thousands(built_points)}점 맵으로 만들어졌는데 현재 "
            f"cloud.pcd 는 {_thousands(cloud_points)}점입니다 — "
            f"FPFH DB 를 다시 만드세요.")

    # 3) mtime 역전 (같은 규칙 2번)
    try:
        cloud_mtime = os.path.getmtime(paths.cloud)
        meta_mtime = os.path.getmtime(paths.fpfh_meta)
    except OSError as error:
        raise PreflightError(f"맵 파일을 읽을 수 없습니다: {error}") from error
    if cloud_mtime > meta_mtime + mm.MTIME_TOLERANCE_SEC:
        raise PreflightError(
            "cloud.pcd 가 측위 DB 보다 최신입니다 — 맵을 다시 저장했다면 "
            "FPFH DB 도 다시 만들어야 합니다.")

    try:
        features = int(meta.get("feature_count", ""))
    except ValueError:
        features = 0

    return {
        "map": paths.name,
        "cloud": paths.cloud,
        "fpfh_prefix": paths.fpfh_prefix,
        "cloud_points": cloud_points,
        "db_features": features,
        "db_voxel": meta.get("voxel", ""),
        # DB 를 만들 때 쓴 전처리 파라미터. localizer 에 **그대로** 넘겨야 한다 —
        # 같은 방식으로 요약한 것끼리만 FPFH 비교가 성립한다. 화면에서 voxel 을
        # 골라 DB 를 다시 만들면 이 값들도 따라 바뀌므로, 여기서 읽어 넘기는 한
        # 어긋날 수가 없다.
        "params": _feature_params(meta),
        # feature 가 지나치게 적으면 정합이 자주 튕긴다. 거부할 근거는 아니지만
        # 실패했을 때 어디를 봐야 하는지는 미리 말해 준다.
        "warning": ("측위 DB 의 feature 가 적습니다 "
                    f"({features}개) — 반복 형상이 많은 곳에서는 정합이 자주 "
                    "거절될 수 있습니다. voxel 을 줄여 다시 만드는 것을 "
                    "고려하세요.") if 0 < features < 1000 else "",
    }


def check_lidar(lidar_source):
    """RosInterface.lidar_source() 결과로 라이다 생존을 판정한다."""
    if lidar_source.get("error"):
        # 조회 자체가 실패한 것이라 '없다'고 단정하면 안 된다. 통과시킨다.
        return ""
    if not lidar_source.get("publishers"):
        raise PreflightError(
            "/livox/lidar 를 발행하는 노드가 없습니다. 먼저 라이다를 켜세요 "
            "(ros2 launch alm_sensors lidar.launch.py).")
    if lidar_source.get("replay"):
        # 거부하지는 않는다 — 재생본으로 측위 파이프라인을 시험하는 것은
        # 정당한 개발 작업이다. 다만 화면이 실측으로 오해하면 안 된다.
        return "재생본(pcd_replay)으로 측위를 돌립니다 — 실측이 아닙니다."
    return ""
