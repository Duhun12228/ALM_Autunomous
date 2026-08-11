#!/usr/bin/env python3
"""prior 3D 맵(.pcd) → Scan Context DB(.npz) 오프라인 생성 (방식 B).

맵 XY 를 --step 간격 격자로 훑으며 '가상 키프레임'을 만든다: 각 격자점에서
반경 max_radius 안의 맵 점들을 키프레임 원점 기준으로 옮겨 SC 디스크립터를
계산한다. 런타임(sc_localizer.py)은 현재 스캔의 디스크립터를 이 DB 와 대조해
초기위치(x, y, yaw)를 자동 특정한다.

키프레임 유효 조건:
  - 반경 안 점 수 >= --min-points (맵 밖/미관측 영역 제외)
  - 원점 주변 --clearance-radius 안 점 수 < --clearance-max (벽/가구 내부 제외)

디스크립터를 만들기 전에 맵 점군을 '그 자리에서 실제로 보이는 것'으로 깎는다
(simulate_scan): 벽 뒤 가려짐 제거 + 수직 시야각 밖 제거. 맵은 여러 위치를
합친 전지적 시점이라 이 보정이 없으면 DB 만 바깥 링이 부풀고, 위치 정보가
바깥 링에 몰려 있는 탓에 매칭이 위치만 크게 빗나간다 (방향은 맞음).

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


def apply_sensor_fov(rel, fov_down_deg, fov_up_deg):
    """가상 키프레임 점들 중 실제 라이다가 그 자리에서 볼 수 없는 것을 제거.

    맵은 여러 위치에서 찍은 스캔을 합친 것이라 근거리 저층까지 빠짐없이 차
    있지만, 실제 라이다는 수직 시야각이 제한된다 (MID-360 은 하향 약 7deg).
    수평거리 r 에서 보이는 최저 높이 = -r*tan(fov_down) — 즉 근거리일수록
    아래쪽이 통째로 안 보인다 (z=-0.3 은 r>2.4m 에서만 보임).

    이 필터가 없으면 DB 는 안쪽 링이 꽉 차고 실제 스캔은 비어서, 거리함수의
    불일치 페널티가 정답 위치에 몰린다. 그 결과 매칭이 '가까이에 아무것도
    없는 자리'(=트인 곳)로 계통적으로 밀린다 (실측: 방향은 맞고 위치만
    구조물 반대쪽으로 ~5m 이탈).
    """
    r = np.hypot(rel[:, 0], rel[:, 1])
    # 0 이하 / 90 이상은 '제한 없음' (필터 끄기). tan(0)=0 이라 그대로 쓰면
    # 하한이 0 이 되어 센서 아래를 몽땅 지워버린다.
    lo = (-r * np.tan(np.radians(fov_down_deg))
          if 0.0 < fov_down_deg < 90.0 else np.full(len(rel), -np.inf))
    hi = (r * np.tan(np.radians(fov_up_deg))
          if 0.0 < fov_up_deg < 90.0 else np.full(len(rel), np.inf))
    return rel[(rel[:, 2] > lo) & (rel[:, 2] < hi)]


def apply_visibility(rel, az_res_deg=1.0, el_res_deg=1.0,
                     tol_ratio=1.03, tol_abs=0.10):
    """가상 키프레임에서 벽 뒤에 가려져 실제로는 보이지 않는 점을 제거.

    맵은 여러 위치에서 찍은 스캔을 합친 것이라, 격자점 반경 안이면 옆방/복도
    점까지 전부 들어온다. 실제 스캔은 벽에 막혀 그 뒤를 못 본다. 이 비대칭이
    DB 의 바깥 링만 부풀리는데, 안쪽 링은 (트인 공간이라) 양쪽 다 비어 정보가
    없으므로 **위치 정보는 사실상 바깥 링에만 있다** — 즉 이 비대칭은 위치
    추정을 직격한다. 방향(yaw)은 벽의 방위만 맞으면 되므로 살아남는다.
    실측 증상: 방향은 맞는데 위치만 구조물 반대쪽으로 ~5m 이탈.

    구현: (방위, 고도) 격자로 깊이버퍼를 만들어 셀마다 최근접 거리만 남긴다
    (실제 라이다가 광선을 쏘는 것과 같은 효과). tol_* 는 벽 두께/측정오차
    여유로, 최근접면과 거의 같은 거리의 점은 같이 살린다.
    """
    if len(rel) == 0:
        return rel
    r3 = np.linalg.norm(rel, axis=1)
    ok = r3 > 1e-6
    rel, r3 = rel[ok], r3[ok]
    if len(rel) == 0:
        return rel

    az = np.arctan2(rel[:, 1], rel[:, 0]) + np.pi          # [0, 2pi)
    el = np.arcsin(np.clip(rel[:, 2] / r3, -1.0, 1.0)) + 0.5 * np.pi  # [0, pi)
    n_az = max(1, int(round(360.0 / az_res_deg)))
    n_el = max(1, int(round(180.0 / el_res_deg)))
    ai = np.minimum((az / (2.0 * np.pi) * n_az).astype(np.int64), n_az - 1)
    ei = np.minimum((el / np.pi * n_el).astype(np.int64), n_el - 1)
    cell = ai * n_el + ei

    # 셀별 최근접 거리: (cell, r) 로 정렬하면 각 셀의 첫 원소가 최근접
    order = np.lexsort((r3, cell))
    c_s, r_s = cell[order], r3[order]
    first = np.empty(len(c_s), bool)
    first[0] = True
    first[1:] = c_s[1:] != c_s[:-1]
    starts = np.flatnonzero(first)
    gmin = np.repeat(r_s[starts], np.diff(np.append(starts, len(c_s))))

    keep = np.zeros(len(r3), bool)
    keep[order[r_s <= gmin * tol_ratio + tol_abs]] = True
    return rel[keep]


def simulate_scan(rel, args):
    """가상 키프레임 점군을 '실제 라이다가 그 자리에서 보는 것'에 가깝게 만든다.

    맵(전지적 시점) 과 실제 스캔(가려짐+시야각 제한) 의 비대칭을 줄이는 게
    매칭 성능의 핵심이다. DB 생성과 selftest 가 반드시 같은 처리를 써야 한다.
    """
    rel = apply_sensor_fov(rel, args.fov_down, args.fov_up)
    if not args.no_visibility:
        rel = apply_visibility(rel, args.vis_az_res, args.vis_el_res)
    return rel


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
            # 맵을 '그 자리에서 실제로 보이는 것'으로 깎아야 런타임 스캔과 대칭
            d = make_descriptor(simulate_scan(rel, args), p)
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
    positions = np.stack(positions)
    descs = np.stack(descs).astype(np.float32)
    keys = np.stack(keys).astype(np.float32)

    # 링별 평균 점유율. 실제 스캔의 프로파일과 비슷해야 매칭이 성립한다 —
    # 안쪽 링이 DB 에서만 꽉 차 있으면 매칭이 트인 곳으로 밀린다
    # (apply_sensor_fov 주석 참고). fov 필터가 들어가면 안쪽 몇 개는 0 에 가까워야 정상.
    occ = (descs > 0).mean(axis=(0, 2))
    print(f"[sc_build_db] 링별 평균 점유율 (안쪽→바깥, 링폭 "
          f"{p.max_radius / p.num_ring:.2f}m):")
    print("  " + " ".join(f"{v:.2f}" for v in occ))
    return positions, descs, keys


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
        rel = simulate_scan(rel, args)
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
    ap.add_argument("--fov-down", type=float, default=7.0,
                    help="라이다 하향 수직 시야각(deg). MID-360 약 7 — 데이터시트로 "
                         "확인할 것. 근거리 안쪽 링에 직접 영향. 0 이면 필터 끔")
    ap.add_argument("--fov-up", type=float, default=52.0,
                    help="라이다 상향 수직 시야각(deg). MID-360 약 52")
    ap.add_argument("--no-visibility", action="store_true",
                    help="가려짐(벽 투과) 제거를 끈다. 켜두는 것이 기본 — 끄면 DB 만 "
                         "옆방 점으로 부풀어 매칭이 트인 곳으로 밀린다")
    ap.add_argument("--vis-az-res", type=float, default=1.0,
                    help="가시성 깊이버퍼 방위 해상도(deg). 작을수록 정밀/느림")
    ap.add_argument("--vis-el-res", type=float, default=1.0,
                    help="가시성 깊이버퍼 고도 해상도(deg)")
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
                        ring_keys=keys, fov_down=args.fov_down,
                        fov_up=args.fov_up,
                        visibility=not args.no_visibility, **p.to_dict())
    size_kb = os.path.getsize(args.out) / 1024
    print(f"[sc_build_db] 저장: {args.out} ({size_kb:.0f} KB, "
          f"키프레임 {len(positions)}개)")

    if args.selftest > 0:
        selftest(xyz, positions, descs, keys, args, p, args.selftest)


if __name__ == "__main__":
    main()
