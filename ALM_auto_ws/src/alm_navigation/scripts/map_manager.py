#!/usr/bin/env python3
"""map_manager — maps/ 디렉터리의 실제 상태를 /alm/map_inventory 로 발행한다.

WebUI 가 "맵이 이미 만들어져 있는지"를 알 수 있는 유일한 경로다. 화면이
`lab_main`/`office_floor_1` 같은 지어낸 목록 대신 파일시스템의 진실을 비추게 한다.

**읽기 전용이다.** 맵을 만들거나 지우거나 활성 맵을 바꾸는 경로는 여기에 없다.

성능상 두 가지를 지킨다.
  1. **헤더만 읽는다.** cloud.pcd 는 52 MB 다. 점 개수는 ASCII 헤더의 POINTS
     한 줄에 있으므로 파일 앞 몇 KB 만 읽으면 된다.
  2. **바뀔 때만 발행한다.** map_publisher 가 680 KB 를 1 Hz 로 맹목 재발행하는
     실수를 반복하지 않는다. 폴링은 mtime/size 만 보고, 스냅샷이 달라졌을 때와
     하트비트 주기에만 publish 한다.

짝 맞음(stale) 판정:
  - fpfh_map.meta 의 map_input_points 가 cloud.pcd 헤더의 POINTS 와 다르면 stale
  - cloud.pcd 가 grid.pgm / fpfh_map.meta 보다 최신이면 stale
  - verify_fingerprint:=true 면 FNV-1a 지문까지 대조 (기본 off — 52 MB 를 파이썬
    바이트 루프로 돌면 수십 초다). 어차피 권위 있는 검증은 teaser_fpfh_localizer 가
    기동할 때 하므로, 기본값에서는 '미확인'을 미확인이라고 표시하는 편이 정직하다.
"""

import os
import sys
import threading

from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from alm_msgs.msg import MapAsset, MapEntry, MapInventory

# 공용 레이아웃 헬퍼는 share/alm_navigation/launch/ 에 설치된다 (launch 와 공유).
sys.path.insert(0, os.path.join(
    get_package_share_directory("alm_navigation"), "launch"))
import map_layout  # noqa: E402

# mtime 비교 여유. 같은 배치에서 만들어진 파일들이 1초 안쪽으로 갈리는 것을
# stale 로 오판하지 않기 위함.
MTIME_TOLERANCE_SEC = 2.0

# ---- 격자 판정 상수 (2026-08-25) ----
# map_server 판정식: occ = (255 - 픽셀값) / 255.
#   0=점유(occ 1.000) · 254=자유(occ 0.004) · 205=미관측(occ 0.19608)
# free_thresh 가 이 값보다 크면 미관측이 **자유공간으로 읽힌다.**
UNKNOWN_OCC = (255.0 - 205.0) / 255.0
# pcd2pgm 이 쓰는 값. 0.196 을 쓰면 실제값과 8e-5 차이라 경계에 걸린다.
GRID_FREE_THRESH = 0.19
# 미관측이 이 비율을 넘으면 '투영 방식으로 구운 격자' 로 본다.
# 실측: 레이캐스팅 없이 구운 cschool 87.9% · alm_lab 81.3%.
UNKNOWN_WARN_SHARE = 0.50

# ⚠ 교과서의 FNV-1a 64 오프셋(14695981039346656037 = 0xCBF29CE484222325)이 **아니다.**
# icp_relocalization/fpfh_pipeline.hpp:232 의 값은 1469598103934665603 — 표준값에서
# 한 자리가 빠진 오타다. 하지만 fpfh_map_builder 와 teaser_fpfh_localizer 가 둘 다
# 그 함수를 쓰므로 체크섬으로서는 일관되게 동작하고, .meta 에 기록된 지문도 이 값
# 기준이다. 여기서 교과서 값을 쓰면 멀쩡한 DB 를 전부 '지문 불일치'로 오판한다.
# (실측 확인: cloud.pcd → 6813c6b4179a7b63, meta 기록값과 일치)
FNV_OFFSET = 1469598103934665603
FNV_PRIME = 0x100000001B3
FNV_MASK = 0xFFFFFFFFFFFFFFFF


def thousands(value):
    return f"{value:,}"


def read_pcd_header(path):
    """PCD ASCII 헤더에서 POINTS/WIDTH/FIELDS 만 뽑는다. 본문은 읽지 않는다."""
    fields = {}
    try:
        with open(path, "rb") as handle:
            # 헤더는 규격상 11줄 남짓이다. 4 KB 면 주석이 있어도 충분하다.
            blob = handle.read(4096)
    except OSError:
        return fields
    for raw in blob.split(b"\n"):
        try:
            line = raw.decode("ascii", "replace").strip()
        except Exception:
            break
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        key = parts[0].upper()
        if key == "POINTS" and len(parts) > 1:
            try:
                fields["points"] = int(parts[1])
            except ValueError:
                pass
        elif key == "FIELDS":
            fields["fields"] = parts[1:]
        elif key == "DATA":
            break   # 여기서부터 본문
    return fields


def read_pgm_size(path):
    """P5 PGM 의 width/height. 주석 줄(#)을 건너뛴다."""
    try:
        with open(path, "rb") as handle:
            blob = handle.read(256)
    except OSError:
        return None
    tokens = []
    for raw in blob.split(b"\n"):
        line = raw.split(b"#", 1)[0]
        tokens.extend(line.split())
        if len(tokens) >= 4:
            break
    if len(tokens) < 3 or tokens[0] not in (b"P5", b"P2"):
        return None
    try:
        return int(tokens[1]), int(tokens[2])
    except ValueError:
        return None


# 파일이 안 바뀌면 다시 안 읽는다. poll_sec(기본 5 s)마다 맵마다 격자 전체를
# 읽으면 Orin Nano 에서 헛일이 된다 — 1014x671 격자 하나가 680 KB 다.
_PGM_SHARES_CACHE = {}


def read_pgm_shares(path, stat=None):
    """P5 PGM 의 점유/자유/미관측 비율. 실패하면 None.

    ##왜 재는가## '이 격자가 레이캐스팅으로 구워졌는가' 를 mtime 으로 추측하지
    않고 **직접 측정**한다. 투영 방식으로 구운 격자는 점이 찍힌 셀만 자유공간이
    되어 미관측이 8할을 넘는다(실측 cschool 87.9% / alm_lab 81.3%). 플래너가
    allow_unknown:false 이므로 그런 맵에서는 대부분의 목표에서 계획이 실패한다.

    격자는 어느 쪽으로 구웠든 똑같이 열리고 로그도 깨끗하다. 목표를 찍어봐야
    드러나므로, 여기서 미리 재서 사람에게 보여준다.

    numpy 를 쓰지 않는다 — bytes.count 가 C 속도라 100만 셀도 수 ms 다.
    ``stat`` 을 주면 (mtime, size)로 캐시해 안 바뀐 파일은 다시 읽지 않는다.
    """
    sig = None
    if stat is not None:
        sig = (stat.st_mtime_ns, stat.st_size)
        hit = _PGM_SHARES_CACHE.get(path)
        if hit and hit[0] == sig:
            return hit[1]
    try:
        with open(path, "rb") as handle:
            blob = handle.read()
    except OSError:
        return None
    # 헤더 3개 토큰(P5, w, h) + maxval 을 건너뛴다. 주석 줄은 무시.
    idx, tokens = 0, []
    while len(tokens) < 4 and idx < len(blob):
        while idx < len(blob) and blob[idx] in b" \t\r\n":
            idx += 1
        if blob[idx:idx + 1] == b"#":
            while idx < len(blob) and blob[idx] not in b"\r\n":
                idx += 1
            continue
        end = idx
        while end < len(blob) and blob[end] not in b" \t\r\n":
            end += 1
        tokens.append(blob[idx:end])
        idx = end
    if len(tokens) < 4 or tokens[0] != b"P5":
        return None
    body = blob[idx + 1:]
    total = len(body)
    if total <= 0:
        return None
    # pcd2pgm 규약: 0=점유 · 254=자유 · 205=미관측
    shares = {
        "total": total,
        "occupied": body.count(b"\x00") / total,
        "free": body.count(b"\xfe") / total,
        "unknown": body.count(b"\xcd") / total,
    }
    if sig is not None:
        _PGM_SHARES_CACHE[path] = (sig, shares)
    return shares


def read_yaml_scalars(path, keys):
    """map.yaml 에서 필요한 스칼라만. PyYAML 없이 한 줄씩 본다."""
    values = {}
    try:
        with open(path) as handle:
            for line in handle:
                line = line.split("#", 1)[0].strip()
                if ":" not in line:
                    continue
                key, _, value = line.partition(":")
                key = key.strip()
                if key in keys:
                    values[key] = value.strip().strip("'\"")
    except OSError:
        pass
    return values


def read_meta(path):
    """fpfh_map.meta 의 key=value."""
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


def file_fingerprint(path, chunk=1 << 20):
    """icp_relocalization::file_fingerprint 와 같은 FNV-1a 64. **느리다.**

    52 MB 면 수십 초 걸린다 — 반드시 백그라운드에서, 캐시와 함께 쓸 것.
    """
    digest = FNV_OFFSET
    try:
        with open(path, "rb") as handle:
            while True:
                block = handle.read(chunk)
                if not block:
                    break
                for byte in block:
                    digest = ((digest ^ byte) * FNV_PRIME) & FNV_MASK
    except OSError:
        return None
    return f"{digest:016x}"


def stat_or_none(path):
    try:
        return os.stat(path)
    except OSError:
        return None


class MapManager(Node):
    def __init__(self):
        super().__init__("map_manager")
        default_root = map_layout.maps_root(
            get_package_share_directory("alm_navigation"))

        g = self.declare_parameter
        g("maps_root", default_root)
        g("topic", "/alm/map_inventory")
        g("poll_sec", 5.0)
        g("heartbeat_sec", 30.0)
        # 52 MB FNV-1a. 켜면 백그라운드 스레드에서 1회 계산하고 캐시한다.
        g("verify_fingerprint", False)

        p = self.get_parameter
        self.root = os.path.abspath(str(p("maps_root").value))
        self.verify = bool(p("verify_fingerprint").value)

        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(MapInventory, str(p("topic").value), qos)

        self.snapshot = None
        self.fingerprints = {}          # cloud 경로 -> 지문 (계산 완료분)
        self.fingerprint_lock = threading.Lock()
        self.fingerprint_busy = set()
        self.since_publish = 0.0
        self.heartbeat = float(p("heartbeat_sec").value)

        poll = max(0.5, float(p("poll_sec").value))
        self.poll = poll
        self.create_timer(poll, self.tick)

        self.get_logger().info(
            f"maps 루트: {self.root}  "
            f"(맵 {len(map_layout.list_map_names(self.root))}개, "
            f"지문대조 {'on' if self.verify else 'off'})")
        self.tick()

    # ── 스캔 ───────────────────────────────────────────────────────────
    def tick(self):
        message, snapshot = self.build()
        changed = snapshot != self.snapshot
        self.since_publish += self.poll
        if not changed and self.since_publish < self.heartbeat:
            return
        self.snapshot = snapshot
        self.since_publish = 0.0
        message.stamp = self.get_clock().now().to_msg()
        self.pub.publish(message)
        if changed:
            active = message.active_map or "(없음)"
            self.get_logger().info(
                f"인벤토리 갱신 — 맵 {len(message.maps)}개, 활성 {active}")

    def build(self):
        message = MapInventory()
        message.root = self.root
        message.active_map = map_layout.active_map_name(self.root)
        message.fingerprint_verified = self.verify

        snapshot = [message.root, message.active_map, self.verify]
        for name in map_layout.list_map_names(self.root):
            entry, entry_key = self.scan_map(
                map_layout.map_paths(self.root, name),
                active=(name == message.active_map))
            message.maps.append(entry)
            snapshot.append(entry_key)
        return message, tuple(snapshot)

    def scan_map(self, paths, active):
        entry = MapEntry()
        entry.name = paths.name
        entry.path = paths.path
        entry.active = active

        manifest = read_yaml_scalars(paths.manifest, {"label", "created", "notes"})
        entry.label = manifest.get("label", paths.name)
        entry.created = manifest.get("created", "")
        entry.notes = manifest.get("notes", "")

        cloud_stat = stat_or_none(paths.cloud)
        cloud_points = 0
        if cloud_stat:
            cloud_points = read_pcd_header(paths.cloud).get("points", 0)

        cloud = self.scan_cloud(paths, cloud_stat, cloud_points)
        grid = self.scan_grid(paths, cloud_stat)
        fpfh = self.scan_fpfh(paths, cloud_stat, cloud_points)
        entry.assets = [cloud, grid, fpfh]
        entry.complete = all(a.present and not a.stale for a in entry.assets)

        key = (entry.name, entry.label, entry.active, entry.complete,
               tuple((a.present, a.stale, a.size_bytes,
                      round(a.modified_epoch, 3), a.detail, a.issue)
                     for a in entry.assets))
        return entry, key

    def scan_cloud(self, paths, cloud_stat, cloud_points):
        asset = MapAsset()
        asset.kind = MapAsset.KIND_CLOUD
        asset.present = cloud_stat is not None
        if not asset.present:
            return asset
        asset.size_bytes = int(cloud_stat.st_size)
        asset.modified_epoch = float(cloud_stat.st_mtime)
        asset.detail = (f"{thousands(cloud_points)} pts"
                        if cloud_points else "점 개수를 읽지 못함")
        if not cloud_points:
            asset.stale = True
            asset.issue = "PCD 헤더에서 POINTS 를 읽지 못했습니다 — 파일 손상 의심"
        return asset

    def scan_grid(self, paths, cloud_stat):
        asset = MapAsset()
        asset.kind = MapAsset.KIND_GRID
        pgm_stat = stat_or_none(paths.grid_pgm)
        yaml_stat = stat_or_none(paths.grid_yaml)
        asset.present = pgm_stat is not None and yaml_stat is not None
        if not asset.present:
            if pgm_stat or yaml_stat:
                asset.issue = "grid.pgm 과 grid.yaml 중 하나가 없습니다"
            return asset

        asset.size_bytes = int(pgm_stat.st_size)
        asset.modified_epoch = float(pgm_stat.st_mtime)

        size = read_pgm_size(paths.grid_pgm)
        meta = read_yaml_scalars(paths.grid_yaml, {"resolution", "image", "free_thresh"})
        resolution = meta.get("resolution", "?")
        asset.detail = (f"{size[0]} x {size[1]} @{resolution} m"
                        if size else f"@{resolution} m")

        # 미관측 비율을 직접 잰다 (read_pgm_shares docstring 참고).
        shares = read_pgm_shares(paths.grid_pgm, pgm_stat)
        if shares:
            asset.detail += f" · 자유 {100 * shares['free']:.0f}% / 미관측 {100 * shares['unknown']:.0f}%"

        # free_thresh 오설정 — 205(occ 0.196)가 free 로 읽히면 미관측이 통행
        # 가능이 된다. 이건 격자가 아니라 yaml 한 줄의 문제라 재굽지 않아도 된다.
        try:
            free_thresh = float(meta.get("free_thresh", GRID_FREE_THRESH))
        except (TypeError, ValueError):
            free_thresh = GRID_FREE_THRESH
        if free_thresh > UNKNOWN_OCC:
            asset.stale = True
            asset.issue = (f"grid.yaml 의 free_thresh({free_thresh})가 "
                           f"{UNKNOWN_OCC:.3f} 보다 큽니다 — 미관측(205)이 "
                           f"자유공간으로 읽힙니다. {GRID_FREE_THRESH} 로 고치세요")
            return asset

        # grid.yaml 의 image 는 폴더 안 상대경로여야 한다 (마이그레이션 누락 탐지)
        image = meta.get("image", "")
        if image and image != map_layout.GRID_PGM_NAME:
            asset.stale = True
            asset.issue = f"grid.yaml 의 image 가 '{image}' 입니다 — grid.pgm 이어야 합니다"
            return asset

        if cloud_stat and cloud_stat.st_mtime > pgm_stat.st_mtime + MTIME_TOLERANCE_SEC:
            asset.stale = True
            asset.issue = "cloud.pcd 가 2D 맵보다 최신입니다 — pcd2pgm 재실행 필요"
            return asset

        # ---- scans.npz 와의 짝 (2026-08-25 추가) ----
        # scans 는 자산 목록에 따로 넣지 않는다(MapAsset.KIND_* 는 메시지 상수라
        # 늘리면 웹 UI 까지 번진다). 대신 **격자의 속성**으로 다룬다 — 어차피
        # scans 의 존재 이유가 '이 격자를 레이캐스팅으로 굽는 것' 하나뿐이다.
        scans_stat = stat_or_none(paths.scans)
        if scans_stat is None:
            if shares and shares["unknown"] > UNKNOWN_WARN_SHARE:
                asset.stale = True
                asset.issue = (
                    f"격자의 {100 * shares['unknown']:.0f}% 가 미관측이고 "
                    f"scans.npz 도 없습니다 — 투영 방식으로 구운 격자입니다. "
                    f"플래너가 allow_unknown:false 라 대부분의 목표에서 계획이 "
                    f"실패합니다. scan_recorder 를 켠 채 재매핑하세요")
            elif shares is None or shares["unknown"] > 0.0:
                asset.issue = ("scans.npz 가 없습니다 — 레이캐스팅으로 다시 구우면 "
                               "자유공간이 넓어집니다 (scan_recorder 필요)")
            return asset

        if scans_stat.st_mtime > pgm_stat.st_mtime + MTIME_TOLERANCE_SEC:
            asset.stale = True
            asset.issue = ("scans.npz 가 2D 맵보다 최신입니다 — "
                           "pcd2pgm --scans 재실행 필요")
            return asset

        if shares and shares["unknown"] > UNKNOWN_WARN_SHARE:
            asset.stale = True
            asset.issue = (
                f"scans.npz 가 있는데도 격자의 {100 * shares['unknown']:.0f}% 가 "
                f"미관측입니다 — --scans 없이 구웠거나 스캔 커버리지가 부족합니다")
        return asset

    def scan_fpfh(self, paths, cloud_stat, cloud_points):
        asset = MapAsset()
        asset.kind = MapAsset.KIND_FPFH
        meta_stat = stat_or_none(paths.fpfh_meta)
        part_stats = [stat_or_none(path) for path in paths.fpfh_files]
        asset.present = meta_stat is not None and all(part_stats)
        if not asset.present:
            if meta_stat or any(part_stats):
                asset.issue = "FPFH DB 파일 일부가 없습니다 — fpfh_map_builder 재실행 필요"
            return asset

        asset.size_bytes = int(sum(s.st_size for s in part_stats))
        asset.modified_epoch = float(meta_stat.st_mtime)

        meta = read_meta(paths.fpfh_meta)
        features = meta.get("feature_count", "")
        try:
            asset.detail = f"{thousands(int(features))} features"
        except ValueError:
            asset.detail = "feature 수 미상"

        # 1) 만들 때 쓴 점 개수와 지금 cloud.pcd 의 점 개수 대조 (가장 싸고 확실하다)
        try:
            built_points = int(meta.get("map_input_points", ""))
        except ValueError:
            built_points = 0
        if cloud_points and built_points and built_points != cloud_points:
            asset.stale = True
            asset.issue = (f"DB 는 {thousands(built_points)}점 맵으로 만들어졌는데 "
                           f"현재 cloud.pcd 는 {thousands(cloud_points)}점입니다 — "
                           "fpfh_map_builder 재실행 필요")
            return asset

        # 2) mtime 역전
        if cloud_stat and cloud_stat.st_mtime > meta_stat.st_mtime + MTIME_TOLERANCE_SEC:
            asset.stale = True
            asset.issue = "cloud.pcd 가 DB 보다 최신입니다 — fpfh_map_builder 재실행 필요"
            return asset

        # 3) 지문 (선택). 계산 전에는 판정하지 않는다 — 모르는 것을 안다고 하지 않는다.
        expected = meta.get("map_fingerprint", "")
        if self.verify and expected and cloud_stat:
            actual = self.fingerprint_for(paths.cloud, cloud_stat)
            if actual is None:
                asset.issue = "지문 계산 중…"
            elif actual != expected:
                asset.stale = True
                asset.issue = (f"맵 지문 불일치 (DB {expected} / 실제 {actual}) — "
                               "다른 맵으로 만든 DB 입니다")
        return asset

    # ── 지문 (백그라운드 + 캐시) ────────────────────────────────────────
    def fingerprint_for(self, path, stat_result):
        """계산돼 있으면 반환, 없으면 백그라운드로 시작하고 None 을 반환."""
        key = (path, stat_result.st_size, int(stat_result.st_mtime))
        with self.fingerprint_lock:
            if key in self.fingerprints:
                return self.fingerprints[key]
            cached = self.read_fingerprint_cache(path, key)
            if cached:
                self.fingerprints[key] = cached
                return cached
            if key in self.fingerprint_busy:
                return None
            self.fingerprint_busy.add(key)

        def worker():
            self.get_logger().info(f"지문 계산 시작 (수십 초 걸립니다): {path}")
            digest = file_fingerprint(path)
            with self.fingerprint_lock:
                self.fingerprint_busy.discard(key)
                if digest:
                    self.fingerprints[key] = digest
            if digest:
                self.write_fingerprint_cache(path, key, digest)
                self.get_logger().info(f"지문 = {digest}  ({path})")

        threading.Thread(target=worker, daemon=True).start()
        return None

    @staticmethod
    def cache_path(path):
        return os.path.join(os.path.dirname(path), ".fingerprint")

    def read_fingerprint_cache(self, path, key):
        try:
            with open(self.cache_path(path)) as handle:
                size, mtime, digest = handle.read().split()
            if int(size) == key[1] and int(mtime) == key[2]:
                return digest
        except (OSError, ValueError):
            pass
        return None

    def write_fingerprint_cache(self, path, key, digest):
        try:
            with open(self.cache_path(path), "w") as handle:
                handle.write(f"{key[1]} {key[2]} {digest}\n")
        except OSError:
            pass


def main():
    rclpy.init()
    node = MapManager()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
