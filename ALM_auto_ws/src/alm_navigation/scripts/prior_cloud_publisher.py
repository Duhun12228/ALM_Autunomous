#!/usr/bin/env python3
"""prior_cloud_publisher — 활성 맵의 저장된 3D 점군을 화면용으로 발행한다.

없어서 생긴 문제를 메꾸는 노드다. 그동안 WebUI 3D 뷰포트는 **저장된 맵을 보여준
적이 없었다.** 실시간 토픽(/livox/lidar, /cloud_registered)만 그렸고, 저장된 맵이
보이는 것처럼 느껴졌던 건 개발용 `pcd_replay` 가 cloud.pcd 를 가짜 라이다
스캔처럼 흘려보내고 있었기 때문이다. 실센서를 붙이면서 그게 꺼지자, 활성 맵을
바꿔도 3D 에는 아무 일도 일어나지 않는 상태가 드러났다.

`pcd_replay` 방식을 다시 쓰면 안 된다. 그건 저장된 맵을 **센서 입력인 척** 내보내는
것이라 FAST-LIO 나 Nav2 의 입력으로도 들어가고, 지금 무엇을 보고 있는지도 흐려진다.
여기서는 별도 토픽(/alm/prior_cloud)으로, 화면 표시 전용임을 분명히 해서 낸다.

세 가지를 지킨다.

1) **복셀 다운샘플.** 원본은 160만점 52 MB 다. 그대로 브리지로 보내면 브라우저가
   죽는다. 0.2 m 격자로 접으면 대개 수만점으로 줄어든다.
2) **latched(TRANSIENT_LOCAL) + 변화 시에만 재발행.** 저장된 맵은 변하지 않는다.
   주기 발행할 이유가 없고, 늦게 붙는 브라우저는 latch 로 받는다.
3) **활성 맵 추적.** active.yaml 이 바뀌면 새 맵을 읽어 다시 낸다. 맵을 골랐는데
   화면이 그대로면 고른 의미가 없다.
"""

import os
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import String

from ament_index_python.packages import get_package_share_directory

# 맵 경로 규약과 PCD 리더는 이미 있는 것을 쓴다 — 같은 규약을 두 번 구현하면
# 언젠가 갈라진다.
sys.path.insert(0, os.path.join(
    get_package_share_directory("alm_navigation"), "launch"))
import map_layout                                          # noqa: E402

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from pcd2pgm import read_pcd_xyz                           # noqa: E402


def voxel_downsample(points, leaf):
    """복셀당 한 점만 남긴다 (첫 점 채택 — 무게중심보다 싸고 화면용으로 충분).

    np.unique 로 한 번에 처리한다. 160만점을 파이썬 루프로 돌면 수십 초다.
    """
    if leaf <= 0 or points.shape[0] == 0:
        return points
    keys = np.floor(points / leaf).astype(np.int64)
    # (N,3) 정수 격자를 한 열로 합쳐 unique 를 한 번만 부른다
    _, index = np.unique(keys, axis=0, return_index=True)
    index.sort()                       # 원래 순서를 유지해야 공간 지역성이 남는다
    return points[index]


class PriorCloudPublisher(Node):
    def __init__(self):
        super().__init__("prior_cloud_publisher")
        g = self.declare_parameter
        g("maps_root", "")
        g("topic", "/alm/prior_cloud")
        g("frame_id", "map")
        g("voxel_leaf", 0.2)
        g("max_points", 400000)        # 다운샘플 후에도 이보다 많으면 균등 솎기
        g("poll_sec", 5.0)

        p = self.get_parameter
        root = str(p("maps_root").value) or map_layout.maps_root(
            get_package_share_directory("alm_navigation"))
        self.root = root
        self.leaf = float(p("voxel_leaf").value)
        self.max_points = int(p("max_points").value)
        self.frame_id = str(p("frame_id").value)

        latched = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(PointCloud2, str(p("topic").value), latched)
        # 화면이 "무엇을 보고 있는지" 말할 수 있어야 한다. 점군만 보내면
        # 어느 맵인지 알 방법이 없다.
        self.name_pub = self.create_publisher(String, "/alm/prior_cloud_map", latched)

        self.loaded = None             # 지금 발행 중인 맵 이름
        self.loaded_mtime = 0.0
        self.tick()
        self.create_timer(max(1.0, float(p("poll_sec").value)), self.tick)
        self.get_logger().info(
            f"prior_cloud_publisher 시작 — root={root} voxel={self.leaf} m")

    def tick(self):
        name = map_layout.active_map_name(self.root)
        if not name:
            self._publish_empty("")
            return
        paths = map_layout.map_paths(self.root, name)
        try:
            mtime = os.path.getmtime(paths.cloud)
        except OSError:
            # 맵은 있는데 cloud.pcd 가 없다. 두 경우가 섞여 있다.
            #   ① 새로 만든 맵 (아직 매핑 전)          — 정상
            #   ② 재매핑 시작으로 방금 지워진 경우      — 낡은 latch 를 비워야 한다
            #
            # ②가 문제였다. 맵 이름이 그대로라 '바뀐 것 없음'으로 판단하고
            # 넘어가서, 이미 사라진 점군이 화면에 계속 떠 있었다. latched 토픽은
            # 아무도 다시 발행하지 않으면 마지막 값이 영원히 남는다.
            # loaded_mtime 이 0 이면 이미 빈 상태를 알린 것이므로 그때만 건너뛴다.
            if self.loaded != name or self.loaded_mtime != 0.0:
                self.get_logger().info(f"'{name}' 에 cloud.pcd 가 없습니다 — 빈 맵 발행")
                self._publish_empty(name)
            return
        if name == self.loaded and mtime == self.loaded_mtime:
            return
        self._load_and_publish(name, paths.cloud, mtime)

    def _load_and_publish(self, name, path, mtime):
        self.get_logger().info(f"'{name}' 읽는 중… {path}")
        try:
            points = read_pcd_xyz(path)
        except Exception as error:                      # noqa: BLE001
            self.get_logger().error(f"{path} 읽기 실패: {error}")
            return

        raw = points.shape[0]
        points = voxel_downsample(np.asarray(points, dtype=np.float32), self.leaf)
        after_voxel = points.shape[0]
        if after_voxel > self.max_points:
            stride = int(np.ceil(after_voxel / self.max_points))
            points = points[::stride]

        self.pub.publish(self._to_msg(points))
        self.name_pub.publish(String(data=name))
        self.loaded = name
        self.loaded_mtime = mtime
        self.get_logger().info(
            f"'{name}' 발행: {raw:,} → 복셀 {after_voxel:,} → 전송 {points.shape[0]:,} 점 "
            f"({points.nbytes / 1024 / 1024:.1f} MB)")

    def _publish_empty(self, name):
        self.pub.publish(self._to_msg(np.zeros((0, 3), dtype=np.float32)))
        self.name_pub.publish(String(data=name))
        self.loaded = name
        self.loaded_mtime = 0.0

    def _to_msg(self, points):
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = 1
        msg.width = int(points.shape[0])
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
        return msg


def main():
    rclpy.init()
    node = PriorCloudPublisher()
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
