#!/usr/bin/env python3
"""Scan Context 디스크립터/매칭 공용 모듈 (방식 B: 초기위치 자동특정).

스캔(로봇 중심 점군)을 극좌표 격자(num_ring 반경 × num_sector 방위)로 나누고
각 bin 에 최대 높이(z)를 기록한 2D 디스크립터로 요약한다. 로봇이 제자리에서
회전하면 디스크립터는 열(column) 방향으로 순환이동만 하므로, 열을 돌려가며
비교하면 장소 인식과 동시에 상대 yaw 를 얻는다. (Kim & Kim, IROS 2018)

  - ring key: 링(행)별 점유율 벡터 → 회전 불변. DB 후보를 빠르게 추리는 용도.
  - 본 매칭: 후보 디스크립터와 모든 column shift 에 대해 열별 코사인 거리 평균.
    최적 shift × sector 폭 = 현재 스캔의 map 기준 yaw.

sc_build_db.py(오프라인 DB 생성)와 sc_localizer.py(런타임 노드)가 공용으로 쓴다.
맵 규모가 집/공장 실내 수준(키프레임 수천 개 이하)이라 KD-tree 없이 numpy
브루트포스로 충분하다.
"""
import numpy as np


class SCParams:
    """디스크립터 파라미터. DB 생성과 런타임 매칭에서 반드시 같아야 한다."""

    def __init__(self, num_ring=20, num_sector=60, max_radius=10.0,
                 z_min=-0.3, z_max=1.0):
        self.num_ring = int(num_ring)
        self.num_sector = int(num_sector)
        self.max_radius = float(max_radius)   # 이 반경 밖 점은 무시 (실내 ~10 m)
        # 센서 기준 상대높이 밴드. z_max 는 반드시 천장 아래 — 천장이 들어가면
        # 모든 bin 이 천장 높이로 균일해져 장소 구분이 무너진다 (실측 확인).
        self.z_min = float(z_min)
        self.z_max = float(z_max)

    def to_dict(self):
        return {"num_ring": self.num_ring, "num_sector": self.num_sector,
                "max_radius": self.max_radius, "z_min": self.z_min,
                "z_max": self.z_max}

    @property
    def sector_width(self):
        return 2.0 * np.pi / self.num_sector


def make_descriptor(xyz, p: SCParams):
    """센서(키프레임) 원점 기준 점군(Nx3) → (num_ring, num_sector) 디스크립터.

    bin 값 = 그 bin 에서 관측된 최대 (z - z_min) > 0, 빈 bin = 0.
    """
    x, y, z = xyz[:, 0], xyz[:, 1], xyz[:, 2]
    r = np.hypot(x, y)
    m = (r > 0.2) & (r < p.max_radius) & (z > p.z_min) & (z < p.z_max)
    if not np.any(m):
        return np.zeros((p.num_ring, p.num_sector), np.float32)

    ring = np.minimum((r[m] / p.max_radius * p.num_ring).astype(np.int64),
                      p.num_ring - 1)
    ang = np.arctan2(y[m], x[m]) + np.pi          # [0, 2pi)
    sect = np.minimum((ang / (2.0 * np.pi) * p.num_sector).astype(np.int64),
                      p.num_sector - 1)
    desc = np.zeros((p.num_ring, p.num_sector), np.float32)
    np.maximum.at(desc, (ring, sect), (z[m] - p.z_min).astype(np.float32))
    return desc


def ring_key(desc):
    """디스크립터 → 링별 점유율 벡터 (num_ring,). 회전 불변."""
    return (desc > 0).mean(axis=1).astype(np.float32)


def sc_distance_all_shifts(query, cands):
    """query (R,S) 를 모든 column shift 로 돌려 cands (K,R,S) 와 비교.

    열별 거리:
      양쪽 다 점 있음   -> 1 - 코사인유사도
      한쪽만 점 있음    -> 1 (구조 불일치 페널티. 이게 없으면 거의 빈
                            디스크립터가 아무 스캔과도 거리 0 으로 오매칭됨)
      양쪽 다 빈 열     -> 제외 (정보 없음)
    거리 = 위 열별 거리의 평균. 반환: dists (K,), shifts (K,).
    """
    R, S = query.shape
    shifted = np.stack([np.roll(query, s, axis=1) for s in range(S)])  # (S,R,S)
    qn = np.linalg.norm(shifted, axis=1)                               # (S,S)
    cn = np.linalg.norm(cands, axis=1)                                 # (K,S)

    dots = np.einsum("src,krc->ksc", shifted, cands)                   # (K,S,S)
    denom = qn[None, :, :] * cn[:, None, :]
    both = denom > 1e-9
    either = (qn[None, :, :] > 1e-9) | (cn[:, None, :] > 1e-9)
    cos = np.where(both, dots / np.where(both, denom, 1.0), 0.0)
    percol = np.where(both, 1.0 - cos, 1.0)                            # 한쪽만=1
    n_info = either.sum(axis=2)                                        # (K,S)
    dist = np.where(n_info > 0,
                    (percol * either).sum(axis=2) / np.maximum(n_info, 1),
                    1.0)                                               # (K,S)
    shifts = dist.argmin(axis=1)
    return dist[np.arange(len(cands)), shifts], shifts


def match(query_desc, db_descs, db_keys, p: SCParams, topk=25):
    """현재 스캔 디스크립터를 DB 와 대조해 후보를 거리순으로 반환.

    반환: [(db_idx, yaw, dist), ...] — yaw 는 '스캔 좌표 → 맵 좌표' 회전(rad).
    """
    k = min(topk, len(db_descs))
    key_d = np.linalg.norm(db_keys - ring_key(query_desc)[None, :], axis=1)
    cand_idx = np.argsort(key_d)[:k]

    dists, shifts = sc_distance_all_shifts(query_desc, db_descs[cand_idx])
    order = np.argsort(dists)
    out = []
    for o in order:
        yaw = float(shifts[o]) * p.sector_width
        if yaw > np.pi:
            yaw -= 2.0 * np.pi
        out.append((int(cand_idx[o]), yaw, float(dists[o])))
    return out
