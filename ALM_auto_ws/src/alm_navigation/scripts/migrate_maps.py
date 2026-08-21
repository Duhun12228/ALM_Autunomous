#!/usr/bin/env python3
"""maps/ 를 평평한 파일 더미에서 **맵 폴더 하나 = 디렉터리 하나** 구조로 옮긴다.

    python3 migrate_maps.py            # 무엇을 할지만 출력 (기본: dry-run)
    python3 migrate_maps.py --apply    # 실제 이동

일회성 스크립트지만 멱등하다 — 이미 옮겼으면 아무것도 하지 않는다.

옮기는 이유는 맵이 둘 이상이 되는 순간 평평한 구조로는 "이 pgm 이 어느 pcd 의
자식인가"를 알 수 없기 때문이다. 새 구조에서는 폴더가 곧 그 답이다.

    maps/
      active.yaml            active: alm_lab
      alm_lab/
        manifest.yaml        이름·라벨·생성일 (자산 '사실'은 적지 않는다)
        cloud.pcd            <- alm_3d_map.pcd
        grid.pgm  grid.yaml  <- alm_map.pgm  alm_map.yaml
        fpfh_map.meta  fpfh_map_{points,normals,fpfh}.pcd

**FPFH DB 는 재생성하지 않아도 된다.** fpfh_map.meta 의 map_fingerprint 는
파일 '내용'의 FNV-1a 라서 경로가 바뀌어도 값이 그대로다 —
teaser_fpfh_localizer 의 기동 시 검증(verify_map_fingerprint)이 그대로 통과한다.
그래서 이 스크립트는 meta 의 map_path 만 고쳐 쓰고 fingerprint 는 손대지 않는다.
"""

import argparse
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_ROOT = os.path.join(HERE, "..", "maps")

# (기존 이름, 새 이름). 폴더 안에서는 맵 이름이 이미 폴더로 드러나므로
# 파일명을 일반화한다 — launch 가 맵 이름 하나로 전체 경로를 조립할 수 있다.
MOVES = [
    ("alm_3d_map.pcd", "cloud.pcd"),
    ("alm_map.pgm", "grid.pgm"),
    ("alm_map.yaml", "grid.yaml"),
    # FPFH 는 fpfh_map_builder 의 <prefix>_points.pcd 규약이라 prefix 를 유지한다.
    ("fpfh_map.meta", "fpfh_map.meta"),
    ("fpfh_map_points.pcd", "fpfh_map_points.pcd"),
    ("fpfh_map_normals.pcd", "fpfh_map_normals.pcd"),
    ("fpfh_map_fpfh.pcd", "fpfh_map_fpfh.pcd"),
]

# Scan Context 는 이 브랜치에서 제거됐다(bbc6dad). 읽는 코드가 없다.
# 복구가 필요하면 09e9dc3 또는 dev/sc-lio-sam 브랜치에 남아 있다.
DELETE = ["sc_db_035.npz"]

MANIFEST = """\
format_version: 1
name: {name}
label: "{label}"
created: "{created}"
sensor: livox_mid360
# 자산의 '사실'(점 개수·해상도·feature 수)은 여기에 적지 않는다.
# map_manager 가 파일 헤더에서 직접 읽는다 — 단일 진실 공급원.
notes: "{notes}"
"""

DEFAULT_NOTES = ("자기가림 마스크 OFF 상태에서 매핑 — 적재물이 궤적을 따라 "
                 "번져 기록돼 있음 (docs/TODO.md 3.5)")


class Plan:
    """무엇을 할지 모아 두었다가 --apply 일 때만 실행한다."""

    def __init__(self, apply_changes):
        self.apply = apply_changes
        self.actions = []
        self.skipped = []

    def note(self, verb, detail):
        self.actions.append(f"{verb:<10} {detail}")

    def skip(self, detail):
        self.skipped.append(detail)


def is_tracked(path):
    """git 이 추적 중인 파일이면 mv 대신 git mv 를 써야 이력이 이어진다."""
    result = subprocess.run(
        ["git", "ls-files", "--error-unmatch", path],
        cwd=os.path.dirname(path) or ".",
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    return result.returncode == 0


def move(plan, source, target):
    if is_tracked(source):
        plan.note("git mv", f"{source} -> {target}")
        if plan.apply:
            subprocess.run(["git", "mv", source, target],
                           cwd=os.path.dirname(source), check=True)
    else:
        plan.note("mv", f"{source} -> {target}")
        if plan.apply:
            # shutil.move 는 같은 파일시스템이면 rename 이라 mtime 이 보존된다.
            # map_manager 의 stale 판정이 mtime 비교에 기대므로 중요하다.
            shutil.move(source, target)


def rewrite_grid_yaml(plan, path):
    """image: alm_map.pgm -> grid.pgm. 나머지 줄은 건드리지 않는다."""
    if not os.path.exists(path):
        return
    with open(path) as handle:
        lines = handle.readlines()
    changed = False
    for index, line in enumerate(lines):
        if line.startswith("image:") and "grid.pgm" not in line:
            lines[index] = "image: grid.pgm\n"
            changed = True
    if not changed:
        plan.skip(f"grid.yaml 의 image 는 이미 grid.pgm")
        return
    plan.note("rewrite", f"{path}  image: -> grid.pgm")
    if plan.apply:
        with open(path, "w") as handle:
            handle.writelines(lines)


def rewrite_meta(plan, path, cloud_path):
    """map_path= 만 새 경로로. map_fingerprint 는 절대 건드리지 않는다."""
    if not os.path.exists(path):
        return
    with open(path) as handle:
        lines = handle.readlines()
    new_value = f"map_path={cloud_path}\n"
    changed = False
    for index, line in enumerate(lines):
        if line.startswith("map_path=") and line != new_value:
            lines[index] = new_value
            changed = True
    if not changed:
        plan.skip("fpfh_map.meta 의 map_path 는 이미 최신")
        return
    plan.note("rewrite", f"{path}  map_path= -> {cloud_path}")
    if plan.apply:
        with open(path, "w") as handle:
            handle.writelines(lines)


def write_if_absent(plan, path, content, label):
    if os.path.exists(path):
        plan.skip(f"{label} 이미 있음")
        return
    plan.note("create", path)
    if plan.apply:
        with open(path, "w") as handle:
            handle.write(content)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--root", default=DEFAULT_ROOT, help="maps/ 경로")
    parser.add_argument("--name", default="alm_lab", help="기존 자산을 담을 맵 이름")
    parser.add_argument("--label", default="연구동 1층 실주행", help="사람이 읽는 라벨")
    parser.add_argument("--notes", default=DEFAULT_NOTES)
    parser.add_argument("--apply", action="store_true",
                        help="실제로 실행 (기본은 출력만)")
    args = parser.parse_args()

    root = os.path.abspath(args.root)
    if not os.path.isdir(root):
        print(f"maps 디렉터리가 없습니다: {root}", file=sys.stderr)
        return 1

    map_dir = os.path.join(root, args.name)
    plan = Plan(args.apply)

    if not os.path.isdir(map_dir):
        plan.note("mkdir", map_dir)
        if plan.apply:
            os.makedirs(map_dir)

    created = None
    for old_name, new_name in MOVES:
        source = os.path.join(root, old_name)
        target = os.path.join(map_dir, new_name)
        if os.path.exists(target):
            plan.skip(f"{new_name} 이미 이동됨")
            if new_name == "cloud.pcd":
                created = os.path.getmtime(target)
            continue
        if not os.path.exists(source):
            plan.skip(f"{old_name} 없음 — 건너뜀")
            continue
        if new_name == "cloud.pcd":
            created = os.path.getmtime(source)
        move(plan, source, target)

    rewrite_grid_yaml(plan, os.path.join(map_dir, "grid.yaml"))
    rewrite_meta(plan, os.path.join(map_dir, "fpfh_map.meta"),
                 os.path.join(map_dir, "cloud.pcd"))

    stamp = "unknown"
    if created:
        import datetime
        stamp = datetime.datetime.fromtimestamp(created).strftime("%Y-%m-%d %H:%M")
    write_if_absent(plan, os.path.join(map_dir, "manifest.yaml"),
                    MANIFEST.format(name=args.name, label=args.label,
                                    created=stamp, notes=args.notes),
                    "manifest.yaml")
    write_if_absent(plan, os.path.join(root, "active.yaml"),
                    f"# map_manager 와 launch 가 읽는 활성 맵.\nactive: {args.name}\n",
                    "active.yaml")

    for name in DELETE:
        path = os.path.join(root, name)
        if not os.path.exists(path):
            plan.skip(f"{name} 없음 — 이미 정리됨")
            continue
        plan.note("delete", f"{path}  (Scan Context 제거로 사용처 없음)")
        if plan.apply:
            os.remove(path)

    print(f"maps 루트: {root}")
    print(f"대상 맵  : {args.name}\n")
    if plan.actions:
        print("할 일:")
        for line in plan.actions:
            print(f"  {line}")
    else:
        print("할 일 없음 — 이미 새 구조입니다.")
    if plan.skipped:
        print("\n건너뜀:")
        for line in plan.skipped:
            print(f"  - {line}")

    if plan.actions and not args.apply:
        print("\n※ dry-run 입니다. 실제로 옮기려면 --apply 를 붙이세요.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
