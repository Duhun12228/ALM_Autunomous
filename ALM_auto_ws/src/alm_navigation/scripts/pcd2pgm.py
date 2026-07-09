#!/usr/bin/env python3
"""FAST-LIO 3D 점군맵(.pcd) -> Nav2 2D 점유격자(.pgm + .yaml) 변환.

위에서 내려다본 투영. 높이 밴드(z-min ~ z-max)로 벽/장애물 층만 골라 점유셀로
찍고, 관측된 영역은 자유공간, 스캔 안 된 곳은 unknown 으로 채운다.

  관측영역 = 모든 점을 (x,y) 로 투영한 셀
  점유셀   = z 가 밴드 안인 점이 min-points 이상 모인 셀
  자유     = 관측영역 - 점유
  unknown  = 관측 안 된 셀

pcd 는 FAST-LIO map 프레임 기준(시작 위치가 원점, z 상방). 라이다 마운트가 0.5 m
라면 지면은 z≈-0.5 근처이므로, 실행하면 출력되는 z 분포를 보고 --z-min/--z-max 를
'지면 위 0.2~1.5 m' 에 맞게 조정할 것.

사용:
  ros2 run alm_navigation pcd2pgm.py --pcd <in.pcd> --out <basename>
  (또는 직접 실행)  python3 pcd2pgm.py --pcd map.pcd --out map --z-min -0.3 --z-max 1.5
"""
import argparse
import os
import struct
import sys

import numpy as np


def read_pcd_xyz(path):
    """binary/ascii PCD 에서 x,y,z (Nx3 float32) 를 읽는다."""
    with open(path, "rb") as f:
        fields, sizes, types, counts = [], [], [], []
        npoints = 0
        data_fmt = None
        header_len = 0
        while True:
            line = f.readline()
            header_len += len(line)
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

        # binary: 각 필드를 numpy dtype 으로 조립
        np_type = {("F", 4): "f4", ("F", 8): "f8",
                   ("U", 1): "u1", ("U", 2): "u2", ("U", 4): "u4",
                   ("I", 1): "i1", ("I", 2): "i2", ("I", 4): "i4"}
        dt_fields = []
        for name, sz, tp, cnt in zip(fields, sizes, types, counts):
            base = np_type.get((tp, sz))
            if base is None:
                raise RuntimeError(f"필드 {name} 타입 미지원: {tp}{sz}")
            if cnt == 1:
                dt_fields.append((name, base))
            else:
                dt_fields.append((name, base, (cnt,)))
        dtype = np.dtype(dt_fields)
        buf = f.read(npoints * dtype.itemsize)
        rec = np.frombuffer(buf, dtype=dtype, count=npoints)
        xyz = np.stack([rec["x"], rec["y"], rec["z"]], axis=1).astype(np.float32)
        return xyz


def main():
    ap = argparse.ArgumentParser(description="PCD 3D 맵 -> Nav2 PGM 2D 맵")
    ap.add_argument("--pcd", required=True, help="입력 .pcd 경로")
    ap.add_argument("--out", required=True, help="출력 basename (.pgm/.yaml 생성)")
    ap.add_argument("--resolution", type=float, default=0.05, help="m/픽셀 (기본 0.05)")
    ap.add_argument("--z-min", type=float, default=-0.3, help="높이밴드 하한 (pcd z, m)")
    ap.add_argument("--z-max", type=float, default=1.5, help="높이밴드 상한 (pcd z, m)")
    ap.add_argument("--min-points", type=int, default=1, help="점유 판정 셀당 최소 점 수")
    args = ap.parse_args()

    if not os.path.isfile(args.pcd):
        sys.exit(f"입력 없음: {args.pcd}")

    print(f"[pcd2pgm] 읽는 중: {args.pcd}")
    xyz = read_pcd_xyz(args.pcd)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    print(f"  포인트 {len(xyz)}개")

    z = xyz[:, 2]
    print(f"  z 분포: min={z.min():.2f} max={z.max():.2f} "
          f"p5={np.percentile(z,5):.2f} p50={np.percentile(z,50):.2f} p95={np.percentile(z,95):.2f}")
    print(f"  높이밴드 [{args.z_min}, {args.z_max}] m 적용")

    res = args.resolution
    min_x, min_y = xyz[:, 0].min(), xyz[:, 1].min()
    max_x, max_y = xyz[:, 0].max(), xyz[:, 1].max()
    W = int(np.ceil((max_x - min_x) / res)) + 1
    H = int(np.ceil((max_y - min_y) / res)) + 1
    print(f"  격자 {W} x {H} @ {res} m  (원점 {min_x:.2f}, {min_y:.2f})")

    col = ((xyz[:, 0] - min_x) / res).astype(np.int32)
    row = ((xyz[:, 1] - min_y) / res).astype(np.int32)
    np.clip(col, 0, W - 1, out=col)
    np.clip(row, 0, H - 1, out=row)

    # 관측영역(모든 점) / 점유(밴드 점 카운트)
    observed = np.zeros((H, W), dtype=bool)
    observed[row, col] = True

    band = (z >= args.z_min) & (z <= args.z_max)
    occ_count = np.zeros((H, W), dtype=np.int32)
    np.add.at(occ_count, (row[band], col[band]), 1)
    occupied = occ_count >= args.min_points

    # PGM 값: 0=점유(검정), 254=자유(흰), 205=unknown(회색). row0=상단이라 y 뒤집기.
    img = np.full((H, W), 205, dtype=np.uint8)
    img[observed & ~occupied] = 254
    img[occupied] = 0
    img = np.flipud(img)

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
        f.write("occupied_thresh: 0.65\n")
        f.write("free_thresh: 0.25\n")

    n_occ = int(occupied.sum())
    n_free = int((observed & ~occupied).sum())
    print(f"[pcd2pgm] 저장: {pgm_path} , {yaml_path}")
    print(f"  점유셀 {n_occ}, 자유셀 {n_free}, unknown {W*H - n_occ - n_free}")


if __name__ == "__main__":
    main()
