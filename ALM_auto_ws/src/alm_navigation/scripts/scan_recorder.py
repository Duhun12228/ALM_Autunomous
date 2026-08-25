#!/usr/bin/env python3
"""scan_recorder — 매핑 중 '정합된 스캔 + 그때의 센서 위치' 를 기록한다.

    ros2 run alm_navigation scan_recorder.py --ros-args \
        -p out:=/path/to/maps/cschool/scans.npz

## 왜 필요한가

`pcd2pgm` 이 3D 점군을 2D 격자로 바꿀 때, 예전에는 **점이 찍힌 셀만** 관측으로
쳤다. 그런데 라이다가 10 m 앞 벽을 봤다는 것은 **그 사이 10 m 가 비어 있다는
관측이기도 하다** — 광선이 통과했으니까. 그 정보를 통째로 버리고 바닥이나
천장에 우연히 점이 찍힌 셀만 자유공간으로 인정한 결과:

    cschool 맵 851 m2 중 관측 자유공간 88.6 m2 (10.4%)

나머지 87.9% 가 미관측으로 남았고, `free_thresh` 오설정까지 겹쳐 플래너는 그
748 m2 를 통행 가능으로 알고 경로를 그렸다.

레이캐스팅으로 고치려면 `(센서 위치, 그 위치에서 본 점들)` 쌍이 필요하다.
그런데 FAST-LIO 의 `/map_save` 는 **누적 점군 하나**만 저장해서, 어느 점이
어느 위치에서 관측됐는지 알 수 없다. 이 노드가 그 쌍을 남긴다.

## 무엇을 구독하나

    /cloud_registered   PointCloud2   odom 프레임으로 정합된 **현재 스캔**
    /Odometry           Odometry      그 시점의 센서 위치

두 토픽 모두 FAST-LIO(laserMapping)가 이미 발행하고 있다. 매핑 launch 에
이 노드만 얹으면 되고, 매핑 자체는 아무것도 바뀌지 않는다.

##중요## `/Odometry` 는 odom->**sensor** 다(base_link 가 아니다, docs/TODO.md §4).
레이캐스팅의 광선 원점은 센서 위치여야 하므로 이게 오히려 맞다. 그대로 쓴다.

## 출력

`out` 으로 지정한 `.npz` 하나:

    points      (N,3) float32  모든 스캔의 점을 이어붙인 것 (odom 프레임)
    offsets     (M+1,) int64   scan i 의 점 = points[offsets[i]:offsets[i+1]]
    origins     (M,3) float32  scan i 를 찍은 센서 위치
    stamps      (M,)  float64  scan i 의 시각 [s]

## 용량 관리

전 스캔을 원본 해상도로 모으면 수억 점이 된다. 두 손잡이로 줄인다.

## 중간 저장은 **별도 스레드**에서 한다 (2026-08-25)

예전에는 `_on_cloud` 콜백 안에서 `savez_compressed` 로 **누적 전체를 다시
압축**했다. 매핑 후반 수십 MB 구간에서 이게 수 초씩 걸리고, 그동안 콜백이
막혀 `/cloud_registered` 큐(depth 20)가 넘친다. **유실된 스캔은 레이캐스팅에
구멍을 내고 그 자리는 미관측으로 남는다** — 로그에는 아무것도 안 남으므로
격자를 열어보기 전까지 모른다. 매핑을 다시 하는 것 말고는 복구가 없다.

지금은 스냅샷만 뜨고 스레드에 넘긴다. 중간 저장은 **무압축**(`savez`)이라
CPU 도 거의 안 쓴다. 최종 저장만 압축한다.

    stride  : N 번째 스캔마다 하나만 기록 (기본 5 -> 약 2 Hz)
              레이캐스팅에 조밀한 시간해상도는 필요 없다. 0.5 s 간격이면
              같은 공간을 여러 번 훑게 되므로 충분하다.
    voxel   : 스캔 내부 다운샘플 [m] (기본 0.15)
              격자 해상도(0.05)보다 크게 잡아도 된다 — 광선은 셀을 지나가며
              칠하므로 점 하나가 셀 여러 개를 채운다.

기본값에서 10 분 매핑 = 약 1200 스캔 x 수천 점 -> 수십 MB 수준.
"""
import os
import threading
import time

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


def voxel_downsample(xyz, leaf):
    """복셀당 점 하나만 남긴다. 정렬 없이 unique 로 처리한다."""
    if leaf <= 0.0 or xyz.shape[0] == 0:
        return xyz
    keys = np.floor(xyz / leaf).astype(np.int64)
    # (i,j,k) 를 하나의 정수로 접어 unique. 좌표 범위가 커도 충돌이 없도록
    # np.unique 를 구조화 뷰로 돌린다.
    _, idx = np.unique(
        keys.view([("", keys.dtype)] * 3).reshape(-1), return_index=True)
    return xyz[np.sort(idx)]


class ScanRecorder(Node):
    def __init__(self):
        super().__init__("scan_recorder")

        self.declare_parameter("cloud_topic", "/cloud_registered")
        self.declare_parameter("odom_topic", "/Odometry")
        self.declare_parameter("out", "scans.npz")
        self.declare_parameter("stride", 5)
        self.declare_parameter("voxel", 0.15)
        # 스캔과 odom 의 시각이 이보다 벌어지면 버린다 [s].
        # 광선 원점이 틀리면 레이캐스팅이 통째로 어긋나므로 느슨하게 두지 않는다.
        self.declare_parameter("max_odom_age", 0.20)
        # 이 시간마다 중간 저장 [s]. 0 이면 종료 시에만 저장.
        # 매핑은 길고 강제종료도 흔하므로 기본으로 켜 둔다.
        self.declare_parameter("autosave_sec", 30.0)

        g = self.get_parameter
        self.out = os.path.expanduser(str(g("out").value))
        self.stride = max(1, int(g("stride").value))
        self.voxel = float(g("voxel").value)
        self.max_odom_age = float(g("max_odom_age").value)
        self.autosave_sec = float(g("autosave_sec").value)

        self._chunks = []      # [ (M,3) float32 ]
        self._origins = []     # [ (3,) ]
        self._stamps = []      # [ float ]
        self._seen = 0         # 받은 스캔 수 (stride 적용 전)
        self._dropped = 0      # odom 이 낡아 버린 스캔 수
        self._odom = None      # (t, x, y, z)
        self._last_save = time.monotonic()
        # 중간 저장은 별도 스레드. 콜백을 막지 않는 것이 목적이다(위 docstring).
        self._save_thread = None
        self._save_lock = threading.Lock()

        # FAST-LIO 는 둘 다 depth 20 의 기본(신뢰성) QoS 로 발행한다.
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=20,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE)

        self.create_subscription(Odometry, str(g("odom_topic").value), self._on_odom, qos)
        self.create_subscription(PointCloud2, str(g("cloud_topic").value), self._on_cloud, qos)

        os.makedirs(os.path.dirname(os.path.abspath(self.out)) or ".", exist_ok=True)
        self.get_logger().info(
            f"scan_recorder 시작 — {g('cloud_topic').value} + {g('odom_topic').value} "
            f"-> {self.out}  (stride={self.stride}, voxel={self.voxel} m)")
        self.get_logger().info(
            "매핑을 마치면 Ctrl+C 로 종료하세요. 그때 최종 저장됩니다 "
            f"(중간 저장 {self.autosave_sec:.0f} s 주기).")

    # ------------------------------------------------------------------ 구독
    def _on_odom(self, msg):
        p = msg.pose.pose.position
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self._odom = (t, p.x, p.y, p.z)

    def _on_cloud(self, msg):
        self._seen += 1
        if (self._seen - 1) % self.stride != 0:
            return
        if self._odom is None:
            return

        t_cloud = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        t_odom, ox, oy, oz = self._odom
        if abs(t_cloud - t_odom) > self.max_odom_age:
            self._dropped += 1
            if self._dropped % 20 == 1:
                self.get_logger().warn(
                    f"스캔-odom 시각차 {abs(t_cloud - t_odom):.3f} s > "
                    f"{self.max_odom_age:.2f} s — 이 스캔은 버립니다 "
                    f"(누적 {self._dropped}개)")
            return

        arr = point_cloud2.read_points_numpy(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        if arr.size == 0:
            return
        xyz = np.asarray(arr, dtype=np.float32).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        if xyz.shape[0] == 0:
            return
        xyz = voxel_downsample(xyz, self.voxel)

        self._chunks.append(xyz)
        self._origins.append((ox, oy, oz))
        self._stamps.append(t_cloud)

        if len(self._chunks) % 50 == 0:
            n = sum(c.shape[0] for c in self._chunks)
            self.get_logger().info(
                f"스캔 {len(self._chunks)}개 / 점 {n:,}개 기록됨")

        if self.autosave_sec > 0.0 and (time.monotonic() - self._last_save) >= self.autosave_sec:
            self._autosave()
            self._last_save = time.monotonic()

    # ------------------------------------------------------------------ 저장
    def _autosave(self):
        """중간 저장. **콜백을 막지 않는다** (모듈 docstring 참고).

        리스트만 얕게 복사해 스레드에 넘긴다 — 담긴 ndarray 는 append 뒤로
        수정되지 않으므로 복사 없이 안전하다. 앞선 저장이 아직 돌고 있으면
        이번 차례는 건너뛴다(밀린 저장을 쌓아봐야 디스크만 때린다).
        """
        if self._save_thread is not None and self._save_thread.is_alive():
            return
        if not self._chunks:
            return
        snap = (list(self._chunks), list(self._origins), list(self._stamps))
        self._save_thread = threading.Thread(
            target=self._write, args=snap, kwargs={"compress": False}, daemon=True)
        self._save_thread.start()

    def _write(self, chunks, origins, stamps, compress):
        counts = [c.shape[0] for c in chunks]
        offsets = np.zeros(len(counts) + 1, dtype=np.int64)
        np.cumsum(counts, out=offsets[1:])
        writer = np.savez_compressed if compress else np.savez
        # 저장 중 죽어도 기존 파일이 남도록 임시파일에 쓰고 교체한다.
        # 락은 tmp 파일 이름이 겹치는 것과 replace 순서 역전을 막는다.
        with self._save_lock:
            tmp = self.out + ".tmp.npz"
            try:
                writer(tmp,
                       points=np.concatenate(chunks, axis=0),
                       offsets=offsets,
                       origins=np.asarray(origins, dtype=np.float32),
                       stamps=np.asarray(stamps, dtype=np.float64))
                os.replace(tmp, self.out)
            except Exception as exc:                              # noqa: BLE001
                self.get_logger().error(f"scans 저장 실패: {exc}")
                return None
        return int(offsets[-1])

    def save(self, quiet=False):
        """최종 저장. 중간 저장 스레드가 끝나기를 기다린 뒤 **압축해서** 쓴다."""
        if self._save_thread is not None and self._save_thread.is_alive():
            self._save_thread.join(timeout=60.0)
        if not self._chunks:
            if not quiet:
                self.get_logger().warn(
                    "기록된 스캔이 없습니다 — 저장하지 않습니다. "
                    "/cloud_registered 와 /Odometry 가 나오고 있는지 확인하세요.")
            return
        total = self._write(self._chunks, self._origins, self._stamps, compress=True)
        if total is None or quiet:
            return
        mb = os.path.getsize(self.out) / 1e6
        self.get_logger().info(
            f"저장: {self.out}  스캔 {len(self._chunks)}개 / 점 {total:,}개 / {mb:.1f} MB")
        if self._dropped:
            self.get_logger().warn(
                f"⚠ odom 시각차로 버린 스캔 {self._dropped}개. 그 자리는 "
                f"레이캐스팅에서 미관측으로 남습니다.")
        self.get_logger().info(
            "다음 단계:  ros2 run alm_navigation pcd2pgm.py "
            f"--pcd <cloud.pcd> --scans {self.out} --out <basename>")


def main():
    rclpy.init()
    node = ScanRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
