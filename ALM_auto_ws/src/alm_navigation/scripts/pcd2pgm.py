#!/usr/bin/env python3
"""FAST-LIO 3D 점군맵(.pcd) -> Nav2 2D 점유격자(.pgm + .yaml) 변환.

두 가지 방식이 있고, **레이캐스팅이 기본이자 권장**이다.

  [레이캐스팅]  --scans <scans.npz> 를 준다 (scan_recorder.py 가 만든 것)
      광선이 지나간 셀 = 자유공간.  라이다가 10 m 앞 벽을 봤다는 것은
      "그 사이 10 m 가 비어 있다" 는 관측이기도 하다는 사실을 쓴다.

  [투영]        --scans 없이 pcd 만 준다 (예전 방식, 호환용)
      점이 찍힌 셀만 관측으로 친다. 즉 바닥이나 천장에 점이 우연히
      찍힌 셀만 자유공간이 된다.

## 왜 바꿨나 (2026-08-23)

투영 방식으로 만든 실측 맵을 세어 보면:

    cschool  격자 851 m2 = 벽 14.3 (1.7%) + 관측자유 88.6 (10.4%) + 미관측 748.5 (87.9%)
    alm_lab  격자 1701 m2 = 벽 21.5 (1.3%) + 관측자유 297.3 (17.5%) + 미관측 1382.1 (81.3%)

격자 크기는 **가장 먼 점**이 정한다(det_range 100 m 라 문틈 너머 벽까지 찍힌다).
그런데 자유공간은 **바닥/천장 점이 찍힌 셀** 뿐이라 다닌 자리 근처로 한정된다.
그 결과 격자의 8할 이상이 미관측으로 남았고, `free_thresh` 오설정과 겹쳐
플래너는 한 번도 본 적 없는 748 m2 를 통행 가능으로 알고 경로를 그렸다.

레이캐스팅을 쓰면 **시야에 든 범위 전체**가 자유공간이 되므로, 매핑 때
구석구석 밟고 다닐 필요도 줄어든다.

## 높이 처리 — 지면 기준으로 바뀌었다

예전에는 `--z-min/--z-max` 를 pcd 절대 z 로 받았다. pcd 원점은 매핑 시작
위치(라이다 높이)라 지면이 z≈-0.5 쯤에 있어서, 사람이 매번 오프셋을 암산해야
했다. 게다가 README 가 쓰던 밴드 [0.3, 0.8] 은 지면 위 0.8~1.3 m 라

    **높이 0.8 m 미만인 장애물(박스·벤치·턱)이 전부 자유공간으로 찍혔다.**

밴드 밖 점이 있는 셀은 `관측됐지만 점유 아님` = 자유공간이 되기 때문이다.
미관측이 free 로 읽히는 것보다 위험한 종류의 오류다.

그래서 이제 **지면을 자동으로 찾고, 지면 기준 높이**로 받는다:

    --obstacle-min-h 0.15   지면 위 15 cm 부터 장애물로 본다 (턱·박스도 잡힘)
    --obstacle-max-h 1.80   지면 위 1.8 m 까지 (천장·조명은 제외)

`--ground-z` 로 지면을 직접 줄 수도 있고, `--z-min/--z-max` 를 주면 예전처럼
절대 z 로 동작한다(호환용).

## 사용

    # 1) 매핑 중 (별도 터미널)
    ros2 run alm_navigation scan_recorder.py --ros-args -p out:=$MAPS/cschool/scans.npz

    # 2) 매핑 후
    ros2 run alm_navigation pcd2pgm.py \
        --pcd $MAPS/cschool/cloud.pcd --scans $MAPS/cschool/scans.npz \
        --out $MAPS/cschool/grid
"""
import argparse
import os
import sys

import numpy as np

# PGM 값. map_server 는  occ = (255 - value)/255  로 읽는다.
V_OCC = 0        # occ 1.000  -> occupied_thresh(0.65) 초과  -> 점유
V_FREE = 254     # occ 0.004  -> free_thresh(0.19) 미만      -> 자유
V_UNKNOWN = 205  # occ 0.196  -> 그 사이                     -> 미관측
# ##중요## free_thresh 는 반드시 0.19 여야 한다. 0.25 로 두면 205 가
# 0.196 < 0.25 라 **free 로 읽힌다** — 미관측이 통행 가능이 되는 버그.
FREE_THRESH = 0.19
OCCUPIED_THRESH = 0.65


def read_pcd_xyz(path):
    """binary/ascii PCD 에서 x,y,z (Nx3 float32) 를 읽는다."""
    with open(path, "rb") as f:
        fields, sizes, types, counts = [], [], [], []
        npoints = 0
        data_fmt = None
        while True:
            line = f.readline()
            if not line:
                raise RuntimeError("PCD 헤더에서 DATA 를 못 찾음")
            text = line.decode("ascii", "replace").strip()
            if text.startswith("#") or text == "":
                continue
            key, *vals = text.split()
            if key == "FIELDS":
                fields = vals
            elif key == "SIZE":
                sizes = [int(v) for v in vals]
            elif key == "TYPE":
                types = vals
            elif key == "COUNT":
                counts = [int(v) for v in vals]
            elif key == "POINTS":
                npoints = int(vals[0])
            elif key == "DATA":
                data_fmt = vals[0]
                break

        if not counts:
            counts = [1] * len(fields)
        ix, iy, iz = fields.index("x"), fields.index("y"), fields.index("z")

        if data_fmt == "ascii":
            arr = np.loadtxt(f, dtype=np.float64)
            if arr.ndim == 1:
                arr = arr[None, :]
            return arr[:, [ix, iy, iz]].astype(np.float32)

        if data_fmt != "binary":
            raise RuntimeError(f"지원 안 하는 DATA 형식: {data_fmt} (binary_compressed 등)")

        np_type = {("F", 4): "f4", ("F", 8): "f8",
                   ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4",
                   ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
        dt_fields = []
        for name, sz, tp, cnt in zip(fields, sizes, types, counts):
            base = np_type.get((tp, sz))
            if base is None:
                raise RuntimeError(f"필드 {name} 타입 미지원: {tp}{sz}")
            dt_fields.append((name, base) if cnt == 1 else (name, base, (cnt,)))
        dtype = np.dtype(dt_fields)
        rec = np.frombuffer(f.read(npoints * dtype.itemsize), dtype=dtype, count=npoints)
        return np.stack([rec["x"], rec["y"], rec["z"]], axis=1).astype(np.float32)


def estimate_ground_z(z, bin_h=0.05, rel_thresh=0.05):
    """지면 높이를 '점이 유의미하게 모인 가장 낮은 층' 으로 추정한다.

    ##주의## 최빈값(mode)을 쓰면 안 된다. 이 스택의 전제 자체가 **바닥이 잘 안
    보인다는 것**이다 — MID-360 은 아래로 -7도까지만 보고, 매끄러운 실내 바닥은
    스치는 입사각에서 반사가 거의 안 돌아온다. 반면 벽은 여러 높이에 걸쳐
    두껍게 찍힌다. 그래서 최빈값을 쓰면 **가장 낮은 벽 층**으로 끌려간다
    (합성 시험에서 실제 지면 -0.50 을 -0.30 으로 오추정했다).

    올바른 기준은 '가장 조밀한 층' 이 아니라 **'유의미한 층 중 가장 낮은 것'**
    이다. 최대 빈 대비 ``rel_thresh`` 이상인 빈 중 최저를 고른다. 성긴 반사
    노이즈는 문턱에 걸려 걸러지고, 실제 바닥은 통과한다.
    """
    lo, hi = float(z.min()), float(z.max())
    if not np.isfinite(lo) or not np.isfinite(hi) or hi - lo < bin_h:
        return lo
    nbins = max(4, int(np.ceil((hi - lo) / bin_h)))
    hist, edges = np.histogram(z, bins=nbins, range=(lo, hi))
    peak = hist.max()
    if peak <= 0:
        return lo
    ok = np.nonzero(hist >= peak * rel_thresh)[0]
    k = int(ok[0]) if ok.size else int(np.argmax(hist))
    return float(0.5 * (edges[k] + edges[k + 1]))


def raycast_free(shape, res, min_x, min_y, points, offsets, origins,
                 blocked, max_range, n_bins, progress=True):
    """스캔별 레이캐스팅으로 '광선이 지나간 셀' 마스크를 만든다.

    각 스캔에서 방위각을 ``n_bins`` 개로 나누고, 빈마다 광선 끝을 정한다.

        · 장애물(blocked) 점이 있으면 -> 그중 **최근접** 거리까지 (그 뒤는 못 봄)
        · 없으면                      -> 그 빈의 **최원** 점까지 (바닥/천장이
                                          거기까지 보였다 = 그만큼 뚫려 있다)
        · 점이 아예 없으면            -> 정보 없음, 광선을 쏘지 않는다

    끝점 자체는 자유로 칠하지 않는다(벽 셀을 지우면 안 된다). 점유는 이 함수가
    아니라 호출부가 '장애물 점이 찍힌 셀' 로 따로 정하므로, 빈으로 뭉개도
    장애물 해상도는 손해보지 않는다.
    """
    H, W = shape
    free = np.zeros((H, W), dtype=bool)
    n_scans = len(origins)
    step = 0.5 * res          # 대각선에서 셀을 건너뛰지 않도록 절반 간격

    for i in range(n_scans):
        s, e = int(offsets[i]), int(offsets[i + 1])
        if e <= s:
            continue
        p = points[s:e]
        blk = blocked[s:e]
        ox, oy = float(origins[i][0]), float(origins[i][1])

        dx = p[:, 0] - ox
        dy = p[:, 1] - oy
        rng = np.hypot(dx, dy)
        keep = (rng > 1e-3) & (rng <= max_range)
        if not keep.any():
            continue
        dx, dy, rng, blk = dx[keep], dy[keep], rng[keep], blk[keep]

        b = ((np.arctan2(dy, dx) + np.pi) / (2.0 * np.pi) * n_bins).astype(np.int32)
        np.clip(b, 0, n_bins - 1, out=b)

        # 빈별 최근접 장애물 거리 / 최원 점 거리
        near_blk = np.full(n_bins, np.inf, dtype=np.float64)
        if blk.any():
            np.minimum.at(near_blk, b[blk], rng[blk])
        far_any = np.zeros(n_bins, dtype=np.float64)
        np.maximum.at(far_any, b, rng)

        end = np.where(np.isfinite(near_blk), near_blk, far_any)
        valid = end > res
        if not valid.any():
            continue
        idx = np.nonzero(valid)[0]
        end = end[idx]
        ang = (idx.astype(np.float64) + 0.5) / n_bins * 2.0 * np.pi - np.pi

        # 끝점 한 셀 앞까지만 자유로 칠한다
        paint = np.maximum(end - res, 0.0)
        n_step = int(np.ceil(paint.max() / step)) + 1
        t = np.linspace(0.0, 1.0, max(n_step, 2))                  # (S,)
        d = paint[:, None] * t[None, :]                            # (K,S)
        px = ox + np.cos(ang)[:, None] * d
        py = oy + np.sin(ang)[:, None] * d

        col = ((px - min_x) / res).astype(np.int32).ravel()
        row = ((py - min_y) / res).astype(np.int32).ravel()
        ok = (col >= 0) & (col < W) & (row >= 0) & (row < H)
        free[row[ok], col[ok]] = True

        if progress and (i + 1) % 200 == 0:
            print(f"    레이캐스팅 {i + 1}/{n_scans} 스캔", flush=True)

    return free


def main():
    ap = argparse.ArgumentParser(description="PCD 3D 맵 -> Nav2 PGM 2D 맵")
    ap.add_argument("--pcd", required=True, help="입력 .pcd 경로")
    ap.add_argument("--out", required=True, help="출력 basename (.pgm/.yaml 생성)")
    ap.add_argument("--scans", default=None,
                    help="scan_recorder.py 가 만든 .npz. 주면 레이캐스팅 모드 (권장)")
    ap.add_argument("--resolution", type=float, default=0.05, help="m/픽셀 (기본 0.05)")

    ap.add_argument("--ground-z", type=float, default=None,
                    help="지면 z [m, pcd 프레임]. 생략하면 z 분포에서 자동 추정")
    ap.add_argument("--obstacle-min-h", type=float, default=0.15,
                    help="지면 위 이 높이부터 장애물 [m] (기본 0.15 — 낮은 턱·박스도 잡는다)")
    ap.add_argument("--obstacle-max-h", type=float, default=1.80,
                    help="지면 위 이 높이까지 장애물 [m] (기본 1.80 — 천장 제외)")
    ap.add_argument("--z-min", type=float, default=None,
                    help="[호환] 장애물 밴드 하한을 pcd 절대 z 로 지정. 주면 지면 기준 대신 이걸 쓴다")
    ap.add_argument("--z-max", type=float, default=None, help="[호환] 장애물 밴드 상한 (절대 z)")

    ap.add_argument("--min-points", type=int, default=1, help="점유 판정 셀당 최소 점 수")
    ap.add_argument("--ray-max-range", type=float, default=15.0,
                    help="레이캐스팅 최대 거리 [m]. 멀수록 입사각이 스쳐 신뢰도가 떨어진다")
    ap.add_argument("--ray-bins", type=int, default=1080,
                    help="스캔당 방위각 빈 수 (기본 1080 = 0.33도)")
    args = ap.parse_args()

    if not os.path.isfile(args.pcd):
        sys.exit(f"입력 없음: {args.pcd}")

    print(f"[pcd2pgm] 읽는 중: {args.pcd}")
    xyz = read_pcd_xyz(args.pcd)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    print(f"  포인트 {len(xyz):,}개")
    if len(xyz) == 0:
        sys.exit("점이 없습니다.")

    # ---------------------------------------------------------------- 스캔 로드
    scans = None
    if args.scans:
        if not os.path.isfile(args.scans):
            sys.exit(f"--scans 파일 없음: {args.scans}")
        scans = np.load(args.scans)
        need = {"points", "offsets", "origins"}
        if not need.issubset(set(scans.files)):
            sys.exit(f"--scans 형식이 다릅니다. 필요한 배열: {sorted(need)}")
        print(f"  스캔 {len(scans['origins'])}개 / 스캔점 {len(scans['points']):,}개 "
              f"(레이캐스팅 모드)")

    # ---------------------------------------------------------------- 높이 밴드
    z = xyz[:, 2]
    print(f"  z 분포: min={z.min():.2f} max={z.max():.2f} "
          f"p5={np.percentile(z,5):.2f} p50={np.percentile(z,50):.2f} p95={np.percentile(z,95):.2f}")

    if args.z_min is not None or args.z_max is not None:
        zlo = args.z_min if args.z_min is not None else float(z.min())
        zhi = args.z_max if args.z_max is not None else float(z.max())
        print(f"  [호환 모드] 장애물 밴드를 절대 z [{zlo:.2f}, {zhi:.2f}] 로 지정받음")
        print("    ⚠ 지면 기준이 아닙니다. 밴드보다 낮은 장애물(턱·박스)은 자유공간이 됩니다.")
    else:
        ground = args.ground_z if args.ground_z is not None else estimate_ground_z(z)
        how = "지정" if args.ground_z is not None else "자동추정"
        zlo = ground + args.obstacle_min_h
        zhi = ground + args.obstacle_max_h
        print(f"  지면 z = {ground:.3f} ({how})")
        print(f"  장애물 밴드 = 지면 위 [{args.obstacle_min_h:.2f}, {args.obstacle_max_h:.2f}] m"
              f"  -> 절대 z [{zlo:.2f}, {zhi:.2f}]")

    # ---------------------------------------------------------------- 격자 범위
    res = args.resolution
    xs, ys = [xyz[:, 0]], [xyz[:, 1]]
    if scans is not None:
        # 광선 원점도 격자 안에 들어와야 한다
        xs.append(scans["origins"][:, 0]); ys.append(scans["origins"][:, 1])
        xs.append(scans["points"][:, 0]);  ys.append(scans["points"][:, 1])
    min_x = float(min(a.min() for a in xs)); max_x = float(max(a.max() for a in xs))
    min_y = float(min(a.min() for a in ys)); max_y = float(max(a.max() for a in ys))
    W = int(np.ceil((max_x - min_x) / res)) + 1
    H = int(np.ceil((max_y - min_y) / res)) + 1
    print(f"  격자 {W} x {H} @ {res} m  (원점 {min_x:.2f}, {min_y:.2f})")

    def to_cells(p):
        c = ((p[:, 0] - min_x) / res).astype(np.int32)
        r = ((p[:, 1] - min_y) / res).astype(np.int32)
        np.clip(c, 0, W - 1, out=c); np.clip(r, 0, H - 1, out=r)
        return r, c

    # ---------------------------------------------------------------- 점유
    # 누적 맵의 장애물 점을 그대로 찍는다. 레이캐스팅 빈으로 뭉개지 않으므로
    # 장애물 해상도는 격자 해상도 그대로다.
    row, col = to_cells(xyz)
    band = (z >= zlo) & (z <= zhi)
    occ_count = np.zeros((H, W), dtype=np.int32)
    np.add.at(occ_count, (row[band], col[band]), 1)
    occupied = occ_count >= args.min_points

    # ---------------------------------------------------------------- 자유
    if scans is not None:
        sp = scans["points"]
        sz = sp[:, 2]
        s_blocked = (sz >= zlo) & (sz <= zhi)
        print(f"  레이캐스팅: 최대 {args.ray_max_range:.1f} m, 빈 {args.ray_bins}개")
        free = raycast_free((H, W), res, min_x, min_y, sp, scans["offsets"],
                            scans["origins"], s_blocked, args.ray_max_range, args.ray_bins)
        # 광선이 지나간 곳이라도 장애물 점이 있으면 장애물이 이긴다
        free &= ~occupied
    else:
        print("  ⚠ --scans 없음 -> 예전 '투영' 방식으로 진행합니다.")
        print("    점이 찍힌 셀만 관측으로 칩니다. 실측에서 격자의 80~88% 가")
        print("    미관측으로 남았습니다. scan_recorder.py 로 스캔을 남긴 뒤")
        print("    --scans 를 주면 시야에 든 범위 전체가 자유공간이 됩니다.")
        observed = np.zeros((H, W), dtype=bool)
        observed[row, col] = True
        free = observed & ~occupied

    # ---------------------------------------------------------------- 출력
    img = np.full((H, W), V_UNKNOWN, dtype=np.uint8)
    img[free] = V_FREE
    img[occupied] = V_OCC
    img = np.flipud(img)          # PGM 은 row0 이 상단

    pgm_path = args.out + ".pgm"
    yaml_path = args.out + ".yaml"
    with open(pgm_path, "wb") as f:
        f.write(f"P5\n{W} {H}\n255\n".encode("ascii"))
        f.write(img.tobytes())
    with open(yaml_path, "w") as f:
        f.write(f"image: {os.path.basename(pgm_path)}\n")
        f.write(f"resolution: {res}\n")
        f.write(f"origin: [{min_x:.4f}, {min_y:.4f}, 0.0]\n")
        f.write("negate: 0\n")
        f.write(f"occupied_thresh: {OCCUPIED_THRESH}\n")
        # ##중요## 0.25 가 아니다. map_server 판정식은 occ = (255-value)/255 이라
        # 미관측(205) 이 occ 0.19608 이 된다. free_thresh 가 0.25 면 그게
        # **free 로 읽혀** 플래너가 한 번도 본 적 없는 곳을 통행 가능으로 안다.
        # ⚠ 0.196 을 쓰지 말 것 — 실제값과 8e-5 차이라 경계에 걸린다.
        f.write(f"free_thresh: {FREE_THRESH}\n")

    n_occ, n_free = int(occupied.sum()), int(free.sum())
    n_unk = W * H - n_occ - n_free
    cell = res * res
    print(f"[pcd2pgm] 저장: {pgm_path} , {yaml_path}")
    print(f"  점유    {n_occ:8,} 셀 ({100*n_occ/(W*H):5.2f}%)  {n_occ*cell:8.1f} m2")
    print(f"  자유    {n_free:8,} 셀 ({100*n_free/(W*H):5.2f}%)  {n_free*cell:8.1f} m2")
    print(f"  미관측  {n_unk:8,} 셀 ({100*n_unk/(W*H):5.2f}%)  {n_unk*cell:8.1f} m2")
    if n_unk > 0.5 * W * H:
        print("  ⚠ 미관측이 절반을 넘습니다. 플래너가 여길 지나가지 않게 하려면")
        print("    nav2.yaml 의 SmacPlannerHybrid.allow_unknown 을 false 로 두세요.")


if __name__ == "__main__":
    main()
