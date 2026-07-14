#!/usr/bin/env python3
"""prior 3D 맵(.pcd) → Scan Context DB(.npz) 오프라인 생성 (방식 B).

맵 XY 를 --step 간격 격자로 훑으며 '가상 키프레임'을 만든다: 각 격자점에서
반경 max_radius 안의 맵 점들을 키프레임 원점 기준으로 옮겨 SC 디스크립터를
계산한다. 런타임(sc_localizer.py)은 현재 스캔의 디스크립터를 이 DB 와 대조해
초기위치(x, y, yaw)를 자동 특정한다.

키프레임 유효 조건:
  - 반경 안 점 수 >= --min-points (맵 밖/미관측 영역 제외)
  - 원점 주변 --clearance-radius 안 점 수 < --clearance-max (벽/가구 내부 제외)

키프레임 z(--keyframe-z)는 측위 시작 시 센서의 map 기준 높이. FAST-LIO map
프레임은 매핑 시작 센서 위치가 원점이므로, 같은 마운트 높이면 0.0 이 맞다.

사용:
  ros2 run alm_navigation sc_build_db.py --pcd maps/alm_3d_map.pcd \
      --out maps/sc_db.npz --selftest 20
"""
import argparse
import os
import sys

import numpy as np

from pcd2pgm import read_pcd_xyz
from scan_context import SCParams, make_descriptor, match, ring_key


def build_cell_index(xy, cell):
    """점들의 2D 셀 해시 인덱스. 반환: {(cx,cy): 점 인덱스 배열}"""
    c = np.floor(xy / cell).astype(np.int64)
    order = np.lexsort((c[:, 1], c[:, 0]))
    cs = c[order]
    change = np.any(np.diff(cs, axis=0) != 0, axis=1)
    starts = np.concatenate(([0], np.nonzero(change)[0] + 1, [len(cs)]))
    return {tuple(cs[starts[i]]): order[starts[i]:starts[i + 1]]
            for i in range(len(starts) - 1)}


def points_near(index, xyz, cell, pos, radius):
    """셀 인덱스로 pos 반경 radius 안 점들을 골라 pos 기준 상대좌표로 반환."""
    cx, cy = int(np.floor(pos[0] / cell)), int(np.floor(pos[1] / cell))
    reach = int(np.ceil(radius / cell))
    idxs = [index[(i, j)]
            for i in range(cx - reach, cx + reach + 1)
            for j in range(cy - reach, cy + reach + 1) if (i, j) in index]
    if not idxs:
        return np.empty((0, 3), np.float32)
    rel = xyz[np.concatenate(idxs)] - pos[None, :]
    keep = np.hypot(rel[:, 0], rel[:, 1]) < radius
    return rel[keep]


def build_db(xyz, args, p: SCParams):
    xy = xyz[:, :2]
    min_x, min_y = xy.min(axis=0)
    max_x, max_y = xy.max(axis=0)
    index = build_cell_index(xy, args.step)

    positions, descs, keys = [], [], []
    n_grid = n_sparse = n_blocked = n_lowcov = 0
    for gx in np.arange(min_x, max_x + args.step, args.step):
        for gy in np.arange(min_y, max_y + args.step, args.step):
            n_grid += 1
            pos = np.array([gx, gy, args.keyframe_z], np.float32)
            rel = points_near(index, xyz, args.step, pos, p.max_radius)
            if len(rel) < args.min_points:
                n_sparse += 1
                continue
            near = (np.hypot(rel[:, 0], rel[:, 1]) < args.clearance_radius) \
                & (np.abs(rel[:, 2]) < 0.5)
            if near.sum() >= args.clearance_max:
                n_blocked += 1
                continue
            d = make_descriptor(rel, p)
            # 방위 커버리지가 낮으면 맵 밖(벽 너머) 유령 키프레임 -> 제외
            coverage = float((d > 0).any(axis=0).mean())
            if coverage < args.min_coverage:
                n_lowcov += 1
                continue
            positions.append(pos)
            descs.append(d)
            keys.append(ring_key(d))

    print(f"[sc_build_db] 격자 {n_grid}곳 중 키프레임 {len(positions)}개 "
          f"(점부족 제외 {n_sparse}, 장애물내부 제외 {n_blocked}, "
          f"커버리지부족 제외 {n_lowcov})")
    if not positions:
        sys.exit("키프레임 0개 — --min-points/--step/--keyframe-z 를 확인할 것")
    return (np.stack(positions), np.stack(descs).astype(np.float32),
            np.stack(keys).astype(np.float32))


def selftest(xyz, positions, descs, keys, args, p: SCParams, n_test):
    """맵에서 가상 스캔을 떠서(위치+yaw 랜덤) 매칭 정확도를 자가검증."""
    xy = xyz[:, :2]
    index = build_cell_index(xy, args.step)
    rng = np.random.default_rng(42)
    pos_tol = args.step  # 격자 간격 이내면 성공 (뒤단 ICP 수렴권)
    yaw_tol = 2.0 * p.sector_width
    n_ok = 0
    errs = []
    for t in range(n_test):
        base = positions[rng.integers(len(positions))]
        true_pos = base + np.array([rng.uniform(-args.step / 2, args.step / 2),
                                    rng.uniform(-args.step / 2, args.step / 2),
                                    0.0], np.float32)
        true_yaw = rng.uniform(-np.pi, np.pi)
        rel = points_near(index, xyz, args.step, true_pos, p.max_radius)
        # 맵 기준 상대점 → 센서 프레임 (센서가 map 대비 +yaw 회전)
        c, s = np.cos(-true_yaw), np.sin(-true_yaw)
        scan = rel.copy()
        scan[:, 0] = c * rel[:, 0] - s * rel[:, 1]
        scan[:, 1] = s * rel[:, 0] + c * rel[:, 1]

        cands = match(make_descriptor(scan, p), descs, keys, p, topk=args.topk)
        idx, yaw, dist = cands[0]
        pos_err = float(np.hypot(*(positions[idx][:2] - true_pos[:2])))
        yaw_err = abs((yaw - true_yaw + np.pi) % (2 * np.pi) - np.pi)
        ok = pos_err <= pos_tol and yaw_err <= yaw_tol
        n_ok += ok
        errs.append((pos_err, yaw_err))
        print(f"  [{t + 1:2d}] {'OK ' if ok else 'FAIL'} "
              f"pos_err={pos_err:.2f}m yaw_err={np.degrees(yaw_err):5.1f}deg "
              f"sc_dist={dist:.3f}")
    pe = np.array([e[0] for e in errs])
    ye = np.degrees([e[1] for e in errs])
    print(f"[selftest] 성공 {n_ok}/{n_test} "
          f"(pos<= {pos_tol:.2f}m & yaw<= {np.degrees(yaw_tol):.0f}deg 기준), "
          f"pos_err 중앙값 {np.median(pe):.2f}m, yaw_err 중앙값 {np.median(ye):.1f}deg")


def main():
    ap = argparse.ArgumentParser(description="prior map.pcd -> Scan Context DB(.npz)")
    ap.add_argument("--pcd", required=True, help="입력 3D 맵 .pcd")
    ap.add_argument("--out", required=True, help="출력 .npz 경로")
    ap.add_argument("--step", type=float, default=0.75, help="키프레임 격자 간격 m")
    ap.add_argument("--keyframe-z", type=float, default=0.0,
                    help="키프레임(센서) z, map 프레임 기준")
    ap.add_argument("--num-ring", type=int, default=20)
    ap.add_argument("--num-sector", type=int, default=60)
    ap.add_argument("--max-radius", type=float, default=10.0, help="SC 최대반경 m")
    ap.add_argument("--z-min", type=float, default=-0.3, help="센서기준 z밴드 하한 (바닥 제외)")
    ap.add_argument("--z-max", type=float, default=1.0,
                    help="센서기준 z밴드 상한. 반드시 천장 아래로 — 천장이 들어가면 "
                         "모든 bin 이 천장 높이로 균일해져 장소 구분이 무너진다")
    ap.add_argument("--min-points", type=int, default=2000,
                    help="키프레임 유효 최소 점 수")
    ap.add_argument("--clearance-radius", type=float, default=0.4)
    ap.add_argument("--clearance-max", type=int, default=20)
    ap.add_argument("--min-coverage", type=float, default=0.3,
                    help="키프레임 유효 최소 방위(sector) 점유율 (완전 맵밖 제외용. "
                         "빈 디스크립터 오매칭은 거리함수의 불일치 페널티가 방지)")
    ap.add_argument("--topk", type=int, default=25, help="selftest ring key 후보 수")
    ap.add_argument("--selftest", type=int, default=0,
                    help="N>0 이면 DB 생성 후 가상스캔 N회 자가검증")
    args = ap.parse_args()

    if not os.path.isfile(args.pcd):
        sys.exit(f"입력 없음: {args.pcd}")
    p = SCParams(args.num_ring, args.num_sector, args.max_radius,
                 args.z_min, args.z_max)

    print(f"[sc_build_db] 읽는 중: {args.pcd}")
    xyz = read_pcd_xyz(args.pcd)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    print(f"  포인트 {len(xyz)}개, step={args.step}m, max_radius={p.max_radius}m")

    positions, descs, keys = build_db(xyz, args, p)

    np.savez_compressed(args.out, positions=positions, descriptors=descs,
                        ring_keys=keys, **p.to_dict())
    size_kb = os.path.getsize(args.out) / 1024
    print(f"[sc_build_db] 저장: {args.out} ({size_kb:.0f} KB, "
          f"키프레임 {len(positions)}개)")

    if args.selftest > 0:
        selftest(xyz, positions, descs, keys, args, p, args.selftest)


if __name__ == "__main__":
    main()
