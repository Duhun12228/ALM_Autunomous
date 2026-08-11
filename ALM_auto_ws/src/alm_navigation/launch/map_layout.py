"""maps/ 디렉터리 레이아웃 해석 — **launch 파일이 아니라 공용 헬퍼다.**

`ros2 launch alm_navigation <TAB>` 목록에 뜨지만 generate_launch_description 이
없으므로 실행되지 않는다. launch 파일들과 같은 디렉터리에 두는 이유는 그래야
설치 경로(share/alm_navigation/launch/)가 하나로 정해져 launch 와 노드가
같은 모듈을 import 할 수 있기 때문이다.

    import os, sys
    sys.path.insert(0, os.path.dirname(__file__))   # launch 파일에서
    import map_layout

레이아웃:

    maps/
      active.yaml            active: <맵이름>
      <맵이름>/
        manifest.yaml        ← 이 파일이 있어야 '맵'으로 인정한다
        cloud.pcd            3D 점군 (FAST-LIO /map_save)
        grid.pgm  grid.yaml  2D 격자 (pcd2pgm)
        fpfh_map.meta  fpfh_map_{points,normals,fpfh}.pcd   측위 DB

manifest.yaml 을 요구하는 이유는 sc_lio_sam_mid360_test/ 같은 실험 잔재 폴더가
맵 목록에 섞여 들어오는 것을 막기 위해서다.
"""

import os

MANIFEST_NAME = "manifest.yaml"
ACTIVE_NAME = "active.yaml"

CLOUD_NAME = "cloud.pcd"
GRID_PGM_NAME = "grid.pgm"
GRID_YAML_NAME = "grid.yaml"
# fpfh_map_builder 의 <prefix>_points.pcd / <prefix>.meta 규약을 따른다.
FPFH_PREFIX_NAME = "fpfh_map"


class MapPaths:
    """맵 하나의 모든 경로. 파일이 실제로 있는지는 확인하지 않는다."""

    def __init__(self, root, name):
        self.name = name
        self.root = root
        self.path = os.path.join(root, name)
        self.manifest = os.path.join(self.path, MANIFEST_NAME)
        self.cloud = os.path.join(self.path, CLOUD_NAME)
        self.grid_pgm = os.path.join(self.path, GRID_PGM_NAME)
        self.grid_yaml = os.path.join(self.path, GRID_YAML_NAME)
        self.fpfh_prefix = os.path.join(self.path, FPFH_PREFIX_NAME)
        self.fpfh_meta = self.fpfh_prefix + ".meta"
        self.fpfh_files = [self.fpfh_prefix + suffix
                           for suffix in ("_points.pcd", "_normals.pcd", "_fpfh.pcd")]

    def __repr__(self):
        return f"MapPaths({self.name!r} @ {self.path!r})"


def maps_root(package_share):
    """실제로 맵이 쌓이는 디렉터리.

    주의가 필요한 지점이다. colcon --symlink-install 은 maps/ 를 **실제
    디렉터리로 만들고 그 안의 항목만 심링크**한다. 그래서 share/maps 를 그대로
    쓰면 매핑으로 소스 쪽에 새 맵 폴더가 생겨도 colcon build 를 다시 돌리기
    전까지 보이지 않는다 — 새 맵이 UI 에 안 뜨는 형태로 조용히 고장난다.

    그래서 심링크를 따라 원본 디렉터리를 찾아 되돌린다. 복사식 설치(심링크
    없음)에서는 자연히 share/maps 를 그대로 쓴다.

    ALM_MAPS_ROOT 환경변수가 있으면 그것이 최우선이다.
    """
    override = os.environ.get("ALM_MAPS_ROOT")
    if override:
        return os.path.abspath(override)

    share_maps = os.path.join(package_share, "maps")
    if not os.path.isdir(share_maps):
        return share_maps
    try:
        entries = sorted(os.listdir(share_maps))
    except OSError:
        return share_maps
    for entry in entries:
        path = os.path.join(share_maps, entry)
        if not os.path.islink(path):
            continue
        source_dir = os.path.dirname(os.path.realpath(path))
        if os.path.isdir(source_dir):
            return source_dir
    return share_maps


def map_paths(root, name):
    return MapPaths(root, name)


def list_map_names(root):
    """manifest.yaml 을 가진 하위 디렉터리만 맵으로 인정하고 이름순 정렬."""
    if not os.path.isdir(root):
        return []
    names = []
    for entry in os.listdir(root):
        if entry.startswith("."):
            continue
        if os.path.isfile(os.path.join(root, entry, MANIFEST_NAME)):
            names.append(entry)
    return sorted(names)


def active_map_name(root):
    """active.yaml 이 가리키는 맵. 없거나 가리키는 맵이 사라졌으면 첫 맵으로 대체.

    yaml 모듈에 의존하지 않는다 — launch 파싱 시점에도 안전하게 부르기 위해
    `active: <이름>` 한 줄만 직접 읽는다.
    """
    names = list_map_names(root)
    path = os.path.join(root, ACTIVE_NAME)
    if os.path.isfile(path):
        try:
            with open(path) as handle:
                for line in handle:
                    line = line.split("#", 1)[0].strip()
                    if line.startswith("active:"):
                        value = line.split(":", 1)[1].strip().strip("'\"")
                        if value in names:
                            return value
                        break
        except OSError:
            pass
    return names[0] if names else ""


def active_map_paths(root):
    """활성 맵의 경로 묶음. 맵이 하나도 없으면 None."""
    name = active_map_name(root)
    return MapPaths(root, name) if name else None
