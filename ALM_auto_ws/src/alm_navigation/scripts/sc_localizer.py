#!/usr/bin/env python3
"""Scan Context 초기위치 자동특정 노드 (방식 B).

RViz "2D Pose Estimate" 를 대체한다: 현재 스캔(/livox/lidar 여러 프레임 누적)의
SC 디스크립터를 sc_build_db.py 가 만든 DB 와 대조해 (x, y, yaw) 후보를 얻고,
/initialpose 로 발행한다. 기존 icp_node 가 이를 초기추정으로 받아 ICP 정밀화
→ /icp_result → transform_publisher(TF map->odom) 로 이어진다 (방식 A 후단 재사용).

동작:
  COLLECT: accum_frames 프레임 누적 (누적 동안 로봇 정지 가정)
  MATCH  : SC 매칭 → 상위 max_candidates 후보
  WAIT   : 후보를 /initialpose 로 발행, icp_wait 초 안에 /icp_result 없으면
           다음 후보. 전부 실패하면 새 스캔으로 처음부터 재시도.
  /icp_result 수신 = 측위 성공 → 노드 종료 (icp_node 도 스스로 종료함)

디버그: /sc_candidates (PoseArray, map 프레임) 로 후보들을 RViz 에서 확인 가능.
"""
import os

import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseWithCovarianceStamped
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

from scan_context import SCParams, make_descriptor, match


class SCLocalizer(Node):
    def __init__(self):
        super().__init__("sc_localizer")
        self.declare_parameter("db_path", "")
        self.declare_parameter("lidar_topic", "/livox/lidar")
        self.declare_parameter("map_frame_id", "map")
        self.declare_parameter("accum_frames", 10)
        self.declare_parameter("topk", 25)
        self.declare_parameter("max_candidates", 5)
        self.declare_parameter("icp_wait_sec", 12.0)

        db_path = self.get_parameter("db_path").value
        if not db_path or not os.path.isfile(db_path):
            self.get_logger().fatal(
                f"SC DB 없음: '{db_path}' — sc_build_db.py 로 생성하거나 "
                "auto_init:=false 로 수동 초기화(RViz 2D Pose Estimate)를 쓸 것")
            raise SystemExit(1)

        db = np.load(db_path)
        self.positions = db["positions"]
        self.descs = db["descriptors"]
        self.keys = db["ring_keys"]
        self.p = SCParams(db["num_ring"], db["num_sector"], db["max_radius"],
                          db["z_min"], db["z_max"])
        self.get_logger().info(
            f"SC DB 로드: {db_path} (키프레임 {len(self.positions)}개, "
            f"ring {self.p.num_ring} x sector {self.p.num_sector}, "
            f"r<{self.p.max_radius}m)")

        self.accum_frames = int(self.get_parameter("accum_frames").value)
        self.topk = int(self.get_parameter("topk").value)
        self.max_candidates = int(self.get_parameter("max_candidates").value)
        self.icp_wait = float(self.get_parameter("icp_wait_sec").value)
        self.map_frame = self.get_parameter("map_frame_id").value

        self.frames = []
        self.candidates = []      # [(idx, yaw, dist)]
        self.cand_i = 0
        self.deadline = None      # WAIT 상태에서 다음 후보로 넘어갈 시각
        self.attempt = 0

        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose", 10)
        self.cand_pub = self.create_publisher(PoseArray, "/sc_candidates", 10)
        lidar_topic = self.get_parameter("lidar_topic").value
        self.cloud_sub = self.create_subscription(
            PointCloud2, lidar_topic, self.cloud_cb, 10)
        self.icp_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/icp_result", self.icp_cb, 10)
        self.timer = self.create_timer(0.5, self.tick)
        self.get_logger().info(
            f"{lidar_topic} 에서 {self.accum_frames}프레임 누적 대기 중 "
            "(누적 동안 로봇 정지 상태여야 함)")

    # -- 콜백 ---------------------------------------------------------------
    def cloud_cb(self, msg):
        if len(self.frames) >= self.accum_frames:
            return
        pts = point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)
        xyz = np.stack([pts["x"], pts["y"], pts["z"]], axis=1)
        self.frames.append(xyz.astype(np.float32))

    def icp_cb(self, msg):
        p = msg.pose.pose.position
        self.get_logger().info(
            f"ICP 수렴 — 측위 성공: x={p.x:.2f} y={p.y:.2f} z={p.z:.2f}. "
            "sc_localizer 종료")
        raise SystemExit(0)

    # -- 상태머신 -------------------------------------------------------------
    def tick(self):
        now = self.get_clock().now()
        if self.deadline is None:
            if len(self.frames) < self.accum_frames:
                return
            self.run_match()
            return
        if now >= self.deadline:
            self.cand_i += 1
            if self.cand_i >= min(self.max_candidates, len(self.candidates)):
                self.get_logger().warn(
                    f"후보 {self.cand_i}개 전부 ICP 미수렴 — 새 스캔으로 재시도 "
                    "(로봇을 조금 움직이거나 맵/DB 일치 여부 확인)")
                self.frames = []
                self.candidates = []
                self.cand_i = 0
                self.deadline = None
                return
            self.publish_candidate()

    def run_match(self):
        scan = np.concatenate(self.frames, axis=0)
        desc = make_descriptor(scan, self.p)
        self.candidates = match(desc, self.descs, self.keys, self.p, self.topk)
        self.attempt += 1
        self.cand_i = 0
        top = self.candidates[:self.max_candidates]
        self.get_logger().info(
            f"[시도 {self.attempt}] 스캔 {len(scan)}점 매칭 완료, 상위 후보: " +
            ", ".join(f"({self.positions[i][0]:.1f},{self.positions[i][1]:.1f},"
                      f"{np.degrees(y):.0f}deg d={d:.3f})" for i, y, d in top))
        self.publish_pose_array(top)
        self.publish_candidate()

    # -- 발행 ----------------------------------------------------------------
    def fill_pose(self, pose: Pose, idx, yaw):
        pos = self.positions[idx]
        pose.position.x = float(pos[0])
        pose.position.y = float(pos[1])
        pose.position.z = float(pos[2])
        pose.orientation.z = float(np.sin(yaw / 2.0))
        pose.orientation.w = float(np.cos(yaw / 2.0))

    def publish_candidate(self):
        idx, yaw, dist = self.candidates[self.cand_i]
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        self.fill_pose(msg.pose.pose, idx, yaw)
        self.pose_pub.publish(msg)
        pos = self.positions[idx]
        self.get_logger().info(
            f"후보 {self.cand_i + 1} 발행: x={pos[0]:.2f} y={pos[1]:.2f} "
            f"yaw={np.degrees(yaw):.0f}deg (sc_dist={dist:.3f}) — "
            f"ICP 수렴 {self.icp_wait:.0f}s 대기")
        self.deadline = self.get_clock().now() + rclpy.duration.Duration(
            seconds=self.icp_wait)

    def publish_pose_array(self, cands):
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = self.map_frame
        for idx, yaw, _ in cands:
            pose = Pose()
            self.fill_pose(pose, idx, yaw)
            arr.poses.append(pose)
        self.cand_pub.publish(arr)


def main():
    rclpy.init()
    try:
        rclpy.spin(SCLocalizer())
    except SystemExit:
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
