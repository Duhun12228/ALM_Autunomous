"""maps/ 에 쓰는 유일한 곳.

읽기는 map_manager 가 /alm/map_inventory 로 이미 하고 있으므로 여기서 다시
스캔하지 않는다. 경로 조립은 alm_navigation 의 map_layout 을 그대로 빌려 쓴다
(같은 규약을 두 번 구현하면 언젠가 갈라진다).

세 가지를 조심한다.

1) **이름 검증.** 맵 이름이 그대로 디렉터리 이름이 된다. `..` 이나 `/` 가
   들어오면 maps/ 밖에 파일을 만들 수 있다. 정규식 통과 후에도 최종 경로가
   maps/ 안에 있는지 realpath 로 다시 확인한다.

2) **원자적 쓰기.** 같은 디렉터리에 임시 파일을 쓰고 os.replace 로 바꾼다.
   active.yaml 을 쓰다 중간에 죽으면 launch 와 map_manager 가 동시에 깨진다.

3) **매핑 타깃 재작성.** fastlio_mid360.yaml 의 map_file_path 는 fast_lio 자신의
   파라미터라 launch 인자 치환이 닿지 않는다. 매핑 **시작 전에** 이 한 줄을
   대상 맵으로 바꿔야 한다. 저장 후 파일을 옮기는 방식은 못 쓴다 — fast_lio 가
   config 에 적힌 경로로 이미 써버린 뒤라 남의 맵이 파괴된 다음이다.
"""

import datetime
import os
import re
import shutil
import tempfile

NAME_RE = re.compile(r"^[a-zA-Z0-9][a-zA-Z0-9_-]{1,31}$")

# fastlio_mid360.yaml 안에서 갈아끼울 줄
MAP_FILE_PATH_RE = re.compile(r"^(\s*map_file_path\s*:\s*).*$")

MANIFEST_TEMPLATE = """format_version: 1
name: {name}
label: {label}
created: "{created}"
sensor: livox_mid360
# 자산의 '사실'(점 개수·해상도·feature 수)은 여기에 적지 않는다.
# map_manager 가 파일 헤더에서 직접 읽는다 — 단일 진실 공급원.
notes: {notes}
"""


class MapWriteError(Exception):
    pass


def _quote(value):
    """YAML 스칼라로 안전하게 감싼다 (큰따옴표 + 이스케이프)."""
    text = str(value or "")
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def validate_name(name):
    name = (name or "").strip()
    if not NAME_RE.match(name):
        raise MapWriteError(
            "맵 이름은 영문·숫자로 시작하고 영문·숫자·_·- 조합 2~32자여야 합니다.")
    return name


def resolve_in_root(root, name):
    """maps/ 안에 있는 경로인지 확인하고 절대경로를 돌려준다."""
    name = validate_name(name)
    root_real = os.path.realpath(root)
    target = os.path.realpath(os.path.join(root_real, name))
    if target != root_real and not target.startswith(root_real + os.sep):
        raise MapWriteError(f"'{name}' 은 맵 디렉터리를 벗어납니다.")
    return target


def _atomic_write(path, text):
    """임시 파일 → os.replace 로 원자적 교체.

    ⚠ realpath 로 먼저 푸는 것이 핵심이다. colcon --symlink-install 은
    install/.../config/*.yaml 을 소스로 가는 **심링크**로 만든다. 그 경로에
    os.replace 를 하면 심링크가 가리키는 파일이 아니라 **심링크 자체가 일반
    파일로 교체**된다. 소스는 그대로인 채 install 만 갈라지고, 다음 colcon
    build 때 조용히 되돌아간다 — 재현이 어려운 종류의 고장이다.

    모드도 보존한다. NamedTemporaryFile 은 0600 으로 만드는데, 그대로 두면
    ros2 launch 가 파일을 못 읽는 경우가 생긴다.
    """
    path = os.path.realpath(path)
    directory = os.path.dirname(path) or "."
    os.makedirs(directory, exist_ok=True)
    try:
        mode = os.stat(path).st_mode & 0o777
    except OSError:
        mode = 0o644

    handle = tempfile.NamedTemporaryFile(
        "w", dir=directory, prefix=".tmp-", suffix=".yaml",
        delete=False, encoding="utf-8")
    try:
        handle.write(text)
        handle.flush()
        os.fsync(handle.fileno())
        handle.close()
        os.chmod(handle.name, mode)
        os.replace(handle.name, path)
    except Exception:
        handle.close()
        try:
            os.unlink(handle.name)
        except OSError:
            pass
        raise


# ── 맵 생성 ─────────────────────────────────────────────────────────────
def create_map(root, name, label="", notes=""):
    name = validate_name(name)
    path = resolve_in_root(root, name)
    if os.path.exists(path):
        raise MapWriteError(f"'{name}' 은 이미 존재합니다.")

    os.makedirs(path, exist_ok=False)
    created = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    try:
        _atomic_write(os.path.join(path, "manifest.yaml"), MANIFEST_TEMPLATE.format(
            name=name,
            label=_quote(label or name),
            created=created,
            notes=_quote(notes or "웹 UI 에서 생성 — 아직 매핑 전"),
        ))
    except Exception:
        shutil.rmtree(path, ignore_errors=True)
        raise
    return {"name": name, "path": path, "created": created}


# ── 활성 맵 전환 ────────────────────────────────────────────────────────
ACTIVE_TEMPLATE = """# map_manager 와 launch 가 읽는 활성 맵.
active: {name}
"""


def set_active_map(root, name, known_names):
    name = validate_name(name)
    if name not in known_names:
        raise MapWriteError(f"'{name}' 은 맵 목록에 없습니다 "
                            f"(manifest.yaml 이 있어야 맵으로 인정됩니다).")
    _atomic_write(os.path.join(root, "active.yaml"), ACTIVE_TEMPLATE.format(name=name))
    return name


# ── 매핑 타깃 (fastlio_mid360.yaml) ─────────────────────────────────────
def read_mapping_target(config_path):
    """지금 /map_save 가 어디로 쓰게 되어 있는지."""
    config_path = os.path.realpath(config_path)
    try:
        with open(config_path, encoding="utf-8") as handle:
            for line in handle:
                match = MAP_FILE_PATH_RE.match(line.rstrip("\n"))
                if match:
                    return line.split(":", 1)[1].strip().strip("'\"")
    except OSError as error:
        raise MapWriteError(f"{config_path} 를 읽을 수 없습니다: {error}") from error
    return ""


def set_mapping_target(config_path, cloud_path):
    """map_file_path 한 줄만 갈아끼운다. 나머지 줄과 주석은 그대로 둔다.

    통째로 yaml.safe_load → dump 하면 주석이 전부 날아간다. 이 파일은 주석이
    실질 문서라(⚠ 경고문 포함) 줄 단위 치환이 맞다.
    """
    config_path = os.path.realpath(config_path)
    try:
        with open(config_path, encoding="utf-8") as handle:
            lines = handle.read().splitlines(keepends=True)
    except OSError as error:
        raise MapWriteError(f"{config_path} 를 읽을 수 없습니다: {error}") from error

    replaced = False
    for index, line in enumerate(lines):
        stripped = line.rstrip("\n")
        match = MAP_FILE_PATH_RE.match(stripped)
        if match:
            ending = "\n" if line.endswith("\n") else ""
            lines[index] = f'{match.group(1)}"{cloud_path}"{ending}'
            replaced = True
            break
    if not replaced:
        raise MapWriteError(f"{config_path} 에 map_file_path 항목이 없습니다.")

    _atomic_write(config_path, "".join(lines))
    return cloud_path


# 매핑을 새로 시작할 때 비우는 것들. manifest.yaml 은 맵의 정체성이라 남긴다.
CLEARABLE = (
    "cloud.pcd",            # 3D 점군 (FAST-LIO /map_save)
    "scans.npz",            # 스캔+센서위치 (scan_recorder) — cloud.pcd 와 한 쌍이다.
                            # 남겨 두면 새 cloud.pcd 에 옛 스캔이 짝지어져
                            # 레이캐스팅이 엉뚱한 자유공간을 그린다.
    "grid.pgm", "grid.yaml",  # 2D 격자 (pcd2pgm)
    "fpfh_map.meta",        # 측위 DB
    "fpfh_map_points.pcd", "fpfh_map_normals.pcd", "fpfh_map_fpfh.pcd",
    ".fingerprint",         # map_manager 의 지문 캐시 (cloud.pcd 파생물)
)


def clear_map_assets(map_dir):
    """맵 폴더의 산출물을 전부 지운다. manifest.yaml 만 남는다.

    왜 시작 전에 지우나: 재매핑하면 cloud.pcd 는 새것으로 덮이지만 grid.pgm 과
    fpfh_map* 는 옛 cloud.pcd 에서 만들어진 채로 남는다. 그러면 서로 짝이 안 맞는
    자산들이 한 폴더에 섞이고, 실수로 그 DB 로 측위를 돌리면 엉뚱한 곳에
    수렴한다. map_manager 가 stale 로 표시해 주긴 하지만, 애초에 만들지 않는
    편이 낫다.

    ⚠ 되돌릴 수 없다. 호출 전에 overwrite 확인을 반드시 거칠 것.
    """
    removed = []
    for name in CLEARABLE:
        path = os.path.join(map_dir, name)
        if not os.path.lexists(path):       # 심링크도 잡는다
            continue
        try:
            os.remove(path)
            removed.append(name)
        except OSError as error:
            raise MapWriteError(f"{name} 삭제 실패: {error}") from error
    return removed


def guard_overwrite(cloud_path, overwrite):
    """이미 cloud.pcd 가 있는 맵으로 매핑을 시작하려는 경우.

    UI 실수 한 번으로 실주행 맵이 사라지는 것을 막는다. 덮어쓰려면 요청에
    overwrite:true 를 명시해야 한다.
    """
    if overwrite:
        return
    map_dir = os.path.dirname(cloud_path)
    existing = [name for name in CLEARABLE
                if name != ".fingerprint" and os.path.lexists(os.path.join(map_dir, name))]
    if not existing:
        return
    raise MapWriteError(
        f"이 맵에는 이미 만들어진 자산이 있습니다 ({', '.join(existing)}). "
        f"매핑을 시작하면 **전부 삭제**하고 처음부터 만듭니다 — "
        f"정말 지우려면 overwrite 를 명시하세요.")
