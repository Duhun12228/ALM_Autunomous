#!/usr/bin/env python3
"""prior 3D 맵(.pcd) → Scan Context DB(.npz) 오프라인 생성 (방식 B).

맵 XY 를 --step 간격 격자로 훑으며 '가상 키프레임'을 만든다: 각 격자점에서
반경 max_radius 안의 맵 점들을 키프레임 원점 기준으로 옮겨 SC 디스크립터를
계산한다. 런타임(sc_localizer.py)은 현재 스캔의 디스크립터를 이 DB 와 대조해
초기위치(x, y, yaw)를 자동 특정한다.

키프레임 유효 조건:
  - 반경 안 점 수 >= --min-points (맵 밖/미관측 영역 제외)
  - 원점 주변 --clearance-radius 안 점 수 < --clearance-max (벽/가구 내부 제외)

디스크립터를 만들기 전에 (방위, 고도) 격자 깊이버퍼로 '그 위치에서 실제로 보이는
최근접 표면'만 남긴다(--no-occlusion 으로 끄면 구버전 동작). 실제 라이다는 벽 뒤를
못 보는데 가상 키프레임은 반경 안 모든 맵 점을 담기 때문이다. 다만 합성 스캔 A/B
에서는 정답률 이득이 확인되지 않았다 — 자세한 경위는 visible_surface() 주석 참고.

키프레임 z(--keyframe-z)는 측위 시작 시 센서의 map 기준 높이. FAST-LIO map
프레임은 매핑 시작 센서 위치가 원점이므로, 같은 마운트 높이면 0.0 이 맞다.

사용:
  ros2 run alm_navigation sc_build_db.py --pcd maps/alm_3d_map.pcd \
      --out maps/sc_db.npz --selftest 20
"""
import argparse
import multiprocessing
import os
import sys
import time

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


def visible_surface(rel, az_deg, el_deg, min_range=0.2):
    """키프레임 원점에서 실제로 보이는 최근접 표면만 남긴다 (각도 깊이버퍼).

    실제 라이다는 한 방향에서 첫 표면까지만 관측한다. 반면 가상 키프레임은
    반경 안의 맵 점을 전부 담아 벽 뒤까지 포함하므로, 그대로 두면 실제 스캔에는
    존재할 수 없는 bin 이 디스크립터에 남는다. (방위, 고도) 격자로 나눠 각 칸에서
    range 가 최소인 점 하나만 남겨 이를 제거한다.

    az_deg/el_deg 가 너무 작으면 점마다 칸이 달라져 아무것도 걸러지지 않고,
    너무 크면 벽면 구조까지 뭉개진다.

    ##측정## 합성 스캔 40개 x 미니 DB 500곳 A/B 에서 rank-1 정답률은 네 설정
    (없음/0.25/0.5/1.0deg) 모두 38/40 으로 차이가 없었다. sc_distance_all_shifts
    의 '한쪽만 점 있음' 페널티가 bin 이 아니라 column(sector) 단위라, 가려진
    먼 ring 이 빠져도 그 열에는 가까운 벽이 남아 페널티 경로를 타지 않기 때문.
    단 1.0deg 는 1위 거리를 0.228 -> 0.258 로 악화시켰다(구조 뭉갬). 기본값
    0.25deg 는 실제 라이다 스캔의 디스크립터 채움률(12.1%)과 가장 근접한
    값(12.2%)이라 고른 것이며, 위 실험은 정답률이 천장에 붙어 판별력이 없었다.
    실기 재검증 전까지 '물리적으로 더 옳지만 이득은 미확인' 으로 볼 것.
    """
    if len(rel) == 0:
        return rel
    # make_descriptor 가 어차피 버리는 근거리 점이 깊이버퍼를 선점하지 않도록 먼저 제거
    rel = rel[np.hypot(rel[:, 0], rel[:, 1]) > min_range]
    if len(rel) == 0:
        return rel

    # ##주의## z 밴드([z_min, z_max]) 밖 점을 여기서 미리 걸러 정렬 대상을 줄이고
    # 싶어지지만 하면 안 된다. 밴드 밖 점(천장 보 등)도 실제로는 시야를 가리므로,
    # 미리 빼면 그 뒤가 보이게 되어 결과가 달라진다. 실측: 19곳 중 14곳 불일치.

    rng = np.linalg.norm(rel, axis=1)
    az = np.degrees(np.arctan2(rel[:, 1], rel[:, 0])) + 180.0          # [0, 360)
    el = np.degrees(np.arcsin(np.clip(rel[:, 2] / rng, -1.0, 1.0))) + 90.0  # [0, 180]
    n_az = int(np.ceil(360.0 / az_deg))
    n_el = int(np.ceil(180.0 / el_deg))
    ai = np.minimum((az / az_deg).astype(np.int64), n_az - 1)
    ei = np.minimum((el / el_deg).astype(np.int64), n_el - 1)

    cell = ai * n_el + ei
    order = np.lexsort((rng, cell))            # 칸별로 묶고, 칸 안에서는 가까운 순
    cs = cell[order]
    first = np.concatenate(([True], cs[1:] != cs[:-1]))
    return rel[order[first]]


_WORK = {}   # 워커 전역. fork 로 자식에게 상속되므로 xyz/index 를 피클하지 않는다.


def _scan_column(task):
    """격자 한 열(고정 gx)을 처리. build_db 의 안쪽 루프와 동일하다.

    격자점끼리 서로 영향을 주지 않으므로 열 단위로 나눠도 결과가 같다.
    완료 순서와 관계없이 최종 DB 순서를 보존하도록 열 인덱스도 함께 반환한다.
    """
    column_i, gx = task
    xyz, index, args, p = _WORK["xyz"], _WORK["index"], _WORK["args"], _WORK["p"]
    min_y, max_y = _WORK["y_range"]

    positions, descs, keys = [], [], []
    n_grid = n_sparse = n_blocked = n_lowcov = 0
    n_raw = n_vis = 0
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
        if args.occlusion:
            # 실제 스캔처럼 가려진 점(벽 뒤)을 제거한 뒤 디스크립터를 만든다
            n_raw += len(rel)
            rel = visible_surface(rel, args.occlusion_az, args.occlusion_el)
            n_vis += len(rel)
        d = make_descriptor(rel, p)
        # 방위 커버리지가 낮으면 맵 밖(벽 너머) 유령 키프레임 -> 제외
        coverage = float((d > 0).any(axis=0).mean())
        if coverage < args.min_coverage:
            n_lowcov += 1
            continue
        positions.append(pos)
        descs.append(d)
        keys.append(ring_key(d))

    return column_i, positions, descs, keys, (n_grid, n_sparse, n_blocked,
                                              n_lowcov, n_raw, n_vis)


def _format_duration(seconds):
    """진행 로그용 초 단위 시간을 읽기 쉬운 문자열로 변환한다."""
    seconds = max(0, int(round(seconds)))
    hours, rem = divmod(seconds, 3600)
    minutes, seconds = divmod(rem, 60)
    if hours:
        return f"{hours}시간 {minutes}분 {seconds}초"
    if minutes:
        return f"{minutes}분 {seconds}초"
    return f"{seconds}초"


def _collect_column_results(iterator, total):
    """완료되는 열을 모으면서 진행률과 예상 남은 시간을 출력한다."""
    results = [None] * total
    started = time.monotonic()
    for completed, item in enumerate(iterator, 1):
        column_i, positions, descs, keys, counts = item
        results[column_i] = (positions, descs, keys, counts)
        elapsed = time.monotonic() - started
        eta = elapsed / completed * (total - completed)
        print(f"[sc_build_db] 진행 {completed}/{total}열 "
              f"({100.0 * completed / total:5.1f}%) | "
              f"경과 {_format_duration(elapsed)} | "
              f"예상 남은 시간 {_format_duration(eta)}",
              flush=True)
    return results


def build_db(xyz, args, p: SCParams):
    xy = xyz[:, :2]
    min_x, min_y = xy.min(axis=0)
    max_x, max_y = xy.max(axis=0)
    index = build_cell_index(xy, args.step)

    columns = list(np.arange(min_x, max_x + args.step, args.step))
    tasks = list(enumerate(columns))
    _WORK.update(xyz=xyz, index=index, args=args, p=p, y_range=(min_y, max_y))

    jobs = args.jobs if args.jobs > 0 else (os.cpu_count() or 1)
    jobs = max(1, min(jobs, len(columns)))
    print(f"[sc_build_db] 격자 {len(columns)}열을 프로세스 {jobs}개로 처리",
          flush=True)
    if jobs == 1:
        results = _collect_column_results(map(_scan_column, tasks), len(tasks))
    else:
        # fork 컨텍스트: xyz(맵 전체)와 index 를 copy-on-write 로 공유한다.
        # 완료된 열부터 받아 진행률을 출력하고 열 인덱스로 원래 순서를 복원한다.
        ctx = multiprocessing.get_context("fork")
        with ctx.Pool(jobs) as pool:
            iterator = pool.imap_unordered(_scan_column, tasks, chunksize=1)
            results = _collect_column_results(iterator, len(tasks))

    positions, descs, keys = [], [], []
    n_grid = n_sparse = n_blocked = n_lowcov = 0
    n_raw = n_vis = 0
    for pos_c, desc_c, key_c, counts in results:
        positions.extend(pos_c)
        descs.extend(desc_c)
        keys.extend(key_c)
        n_grid += counts[0]
        n_sparse += counts[1]
        n_blocked += counts[2]
        n_lowcov += counts[3]
        n_raw += counts[4]
        n_vis += counts[5]

    print(f"[sc_build_db] 격자 {n_grid}곳 중 키프레임 {len(positions)}개 "
          f"(점부족 제외 {n_sparse}, 장애물내부 제외 {n_blocked}, "
          f"커버리지부족 제외 {n_lowcov})")
    if args.occlusion and n_raw:
        print(f"  가림처리(az {args.occlusion_az}deg x el {args.occlusion_el}deg): "
              f"키프레임당 평균 {n_raw / max(len(positions), 1):.0f} -> "
              f"{n_vis / max(len(positions), 1):.0f}점 "
              f"(가려진 점 {100.0 * (1 - n_vis / n_raw):.1f}% 제거)")
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
        if args.occlusion:
            # 가상 스캔도 실제 라이다처럼 가려진 점을 뺀다 (yaw 회전은 range/고도
            # 를 바꾸지 않으므로 회전 전에 걸어도 결과가 같다)
            rel = visible_surface(rel, args.occlusion_az, args.occlusion_el)
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
    ap.add_argument("--z-min", type=float, default=-0.35,
                    help="센서기준 z밴드 하한. 마운트 높이보다 깊게 잡으면 바닥면이 "
                         "들어와 바깥 링이 균일하게 채워진다")
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
    ap.add_argument("--jobs", type=int, default=0,
                    help="병렬 프로세스 수 (0=코어 수, 1=순차). 결과는 값과 무관하게 동일")
    ap.add_argument("--no-occlusion", dest="occlusion", action="store_false",
                    help="가림처리 없이 반경 안 모든 맵 점 사용 (구버전 동작, A/B 비교용)")
    # 0.25deg: 실측 라이다 스캔의 디스크립터 채움률(12.1%)에 가장 근접(12.2%).
    # 1.0deg 는 벽면 구조를 뭉개 매칭 거리가 나빠지는 것을 확인했으므로 키우지 말 것.
    ap.add_argument("--occlusion-az", type=float, default=0.25,
                    help="가림처리 방위 격자 크기 deg")
    ap.add_argument("--occlusion-el", type=float, default=0.25,
                    help="가림처리 고도 격자 크기 deg")
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

    # 가림처리 설정도 함께 남긴다 (sc_localizer 는 읽지 않지만 DB 추적/비교용).
    np.savez_compressed(args.out, positions=positions, descriptors=descs,
                        ring_keys=keys,
                        occlusion=args.occlusion,
                        occlusion_az_deg=args.occlusion_az,
                        occlusion_el_deg=args.occlusion_el,
                        **p.to_dict())
    size_kb = os.path.getsize(args.out) / 1024
    print(f"[sc_build_db] 저장: {args.out} ({size_kb:.0f} KB, "
          f"키프레임 {len(positions)}개)")

    if args.selftest > 0:
        selftest(xyz, positions, descs, keys, args, p, args.selftest)


if __name__ == "__main__":
    main()
