#!/usr/bin/env python3
"""plan_probe — planner_server 에 목표를 넣고 **나온 경로를 채점한다**.

    ros2 run alm_navigation plan_probe.py --goal 5.5 3.8 0
    ros2 run alm_navigation plan_probe.py --goal -14.5 -13.5 -1.57 --smooth
    ros2 run alm_navigation plan_probe.py --start 0 0 0 --goal 8.9 2.4 0

RViz 로 "경로가 그려지네" 를 보는 것과 다르다. 화면으로는 이 스택에서 정작
중요한 것들이 안 보인다.

  · 계획 시간이 max_planning_time(2.0 s) 과 재계획 주기(1 Hz) 안에 드는가
  · 경로의 최소 선회반경이 R_min(1.643 m) 을 지키는가
    → Hybrid-A* 를 쓴 이유 그 자체다. 여기가 깨지면 경로가 나와도 못 따라간다
    → cusp(전진<->후진 전환) 주변은 빼고 잰다. 거기서 경로가 접히므로 외접원
      반경이 무의미해지고, 안 빼면 Reeds-Shepp 경로가 전부 위반으로 찍힌다
  · 후진(cusp) 이 몇 번 들어갔는가 — 후진은 전진의 1/3 속도다
  · 경로가 **관측된 자유공간**을 지나는가, 아니면 미관측 영역을 가로지르는가
    → grid.pgm 원본을 직접 읽어 판정한다. costmap 이 미관측을 어떻게 해석하든
      "저기는 실제로 본 적 없는 곳" 이라는 사실은 변하지 않기 때문이다
  · 경로 위 각 점에서 벽까지 여유가 inscribed radius(0.530 m) 이상인가

--smooth 를 주면 smoother_server(ConstrainedSmoother)까지 태워 실차의 BT
파이프라인(ComputePathToPose -> SmoothPath)과 같은 순서로 채점한다.
"""

import argparse
import math
import os
import sys

import numpy as np
import rclpy
import yaml as yamllib
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, SmoothPath
from rclpy.action import ActionClient
from rclpy.node import Node

from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.join(
    get_package_share_directory("alm_navigation"), "launch"))
import map_layout                                          # noqa: E402

# nav2.yaml 과 같은 출처(URDF 실측 유도값). 여기서 새로 정하지 않는다.
R_MIN = 1.643          # SmacPlannerHybrid.minimum_turning_radius
INSCRIBED = 0.530      # footprint inscribed radius
MAX_PLANNING_TIME = 2.0


def pose_stamped(x, y, yaw, frame="map"):
    pose = PoseStamped()
    pose.header.frame_id = frame
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
    pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
    return pose


def yaw_of(pose):
    q = pose.pose.orientation
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class OccupancyMap:
    """grid.pgm 을 그대로 읽어 '경로가 어디를 지나는가' 를 판정한다.

    costmap 을 구독하지 않고 원본을 읽는 이유: costmap 은 free_thresh 해석 ·
    inflation · unknown 처리를 이미 거친 결과라, 경로가 미관측 영역을 지나도
    그 사실이 지워져 있다. 채점자는 원본을 봐야 한다.
    """

    # pcd2pgm 규약: 0=occupied, 205=unknown, 254=free
    def __init__(self, yaml_path):
        from PIL import Image
        with open(yaml_path) as handle:
            meta = yamllib.safe_load(handle)
        image_path = meta["image"]
        if not os.path.isabs(image_path):
            image_path = os.path.join(os.path.dirname(yaml_path), image_path)
        self.img = np.array(Image.open(image_path))
        self.res = float(meta["resolution"])
        self.ox, self.oy = float(meta["origin"][0]), float(meta["origin"][1])
        self.height, self.width = self.img.shape
        self.free_thresh = float(meta.get("free_thresh", 0.25))
        self.occupied_thresh = float(meta.get("occupied_thresh", 0.65))

        occupied = self.img <= 50
        self.distance = None
        try:
            from scipy import ndimage
            # 벽까지의 거리. 미관측은 '벽이 아님' 으로 두어 costmap 과 같은 눈으로 본다.
            self.distance = ndimage.distance_transform_edt(~occupied) * self.res
        except ImportError:
            pass

    def cell(self, x, y):
        col = int(round((x - self.ox) / self.res - 0.5))
        row = int(round(self.height - 1 - ((y - self.oy) / self.res - 0.5)))
        if not (0 <= col < self.width and 0 <= row < self.height):
            return None, None
        value = int(self.img[row, col])
        clearance = None if self.distance is None else float(self.distance[row, col])
        return value, clearance

    def classify(self, value):
        if value is None:
            return "밖"
        if value <= 50:
            return "점유"
        if value >= 250:
            return "자유"
        return "미관측"

    def unknown_reads_as_free(self):
        """map_server 가 미관측(205)을 '자유' 로 읽는가.

        map_server 는 occ = (255 - 화소)/255 로 환산해 free_thresh 와 비교한다.
        205 -> 0.196 이므로 free_thresh 가 0.196 보다 크면 미관측이 통째로
        자유공간이 된다 — 격자만 봐서는 드러나지 않는 종류의 함정이다.
        """
        return (255.0 - 205.0) / 255.0 < self.free_thresh


def path_metrics(poses, occ_map):
    """경로 한 줄에 대한 채점표."""
    xy = np.array([[p.pose.position.x, p.pose.position.y] for p in poses])
    out = {"poses": len(poses)}
    if len(poses) < 2:
        return out

    segments = np.linalg.norm(np.diff(xy, axis=0), axis=1)
    out["length"] = float(segments.sum())

    # cusp(진행방향 반전) 지점. Reeds-Shepp 경로에서 후진 구간의 시작점이다.
    # ##순서주의## 최소 선회반경보다 **먼저** 구한다 — 반경 계산이 이 결과로
    # cusp 주변을 걸러내야 하기 때문이다. 아래 주석 참고.
    cusp_at = []
    previous = None
    for i in range(len(poses) - 1):
        step = xy[i + 1] - xy[i]
        if np.linalg.norm(step) < 1e-6:
            continue
        heading = yaw_of(poses[i])
        forward = math.cos(heading) * step[0] + math.sin(heading) * step[1]
        if previous is not None and forward * previous < 0:
            cusp_at.append(i)          # 점 i 에서 전진<->후진이 뒤집힌다
        previous = forward
    out["cusps"] = len(cusp_at)

    # 최소 선회반경: 연속 세 점의 외접원 반경. 직선 구간(면적 0)은 무한대로 둔다.
    #
    # ##중요## cusp 주변은 제외한다. 외접원 공식은 세 점이 **하나의 원호 위에**
    # 있다고 가정하는데, cusp 을 사이에 둔 세 점은 경로가 접혀 되돌아오므로 그
    # 가정이 깨진다. 거기서 나오는 반경은 로봇이 실제로 도는 반경이 아니다 —
    # cusp 은 조향이 아니라 기어 전환이고, 로봇은 서서 방향을 바꾼다.
    #
    # 걸러내지 않으면 Reeds-Shepp 경로는 cusp 이 항상 있으므로 이 지표가 **상시**
    # R_min 위반으로 찍힌다. 실측: cschool 4개 목표에서 전체최소 0.07~0.32 m 로
    # 전부 위반 판정이었는데, cusp 을 빼면 4개 모두 정확히 1.643 m = R_min 이었다.
    # 늘 빨간불이면 진짜 위반이 생겨도 묻힌다.
    skip = set()
    for i in cusp_at:
        skip.update(range(i - 2, i + 3))   # 반전 지점 앞뒤 2점
    radii = []
    for i in range(len(xy) - 2):
        if (i + 1) in skip:                # 삼각형의 꼭짓점이 cusp 근방이면 버린다
            continue
        a, b, c = xy[i], xy[i + 1], xy[i + 2]
        ab, bc, ca = (np.linalg.norm(b - a), np.linalg.norm(c - b), np.linalg.norm(a - c))
        area2 = abs((b[0] - a[0]) * (c[1] - a[1]) - (c[0] - a[0]) * (b[1] - a[1]))
        # 점이 겹치거나 거의 일직선이면 곡률 추정이 수치적으로 무의미하다.
        if area2 < 1e-9 or min(ab, bc, ca) < 1e-6:
            continue
        radii.append(ab * bc * ca / (2.0 * area2))
    out["min_radius"] = float(min(radii)) if radii else float("inf")

    if occ_map is not None:
        counts = {"자유": 0, "미관측": 0, "점유": 0, "밖": 0}
        min_clearance = float("inf")
        worst = None
        for x, y in xy:
            value, clearance = occ_map.cell(x, y)
            counts[occ_map.classify(value)] += 1
            if clearance is not None and clearance < min_clearance:
                min_clearance, worst = clearance, (x, y)
        out["cells"] = counts
        if worst is not None:
            out["min_clearance"] = min_clearance
            out["min_clearance_at"] = worst
    return out


class PlanProbe(Node):
    def __init__(self, args):
        super().__init__("plan_probe")
        self.args = args
        self.compute = ActionClient(self, ComputePathToPose, "compute_path_to_pose")
        self.smooth = ActionClient(self, SmoothPath, "smooth_path")

    def wait_for_servers(self, timeout=20.0):
        if not self.compute.wait_for_server(timeout_sec=timeout):
            self.get_logger().error(
                "compute_path_to_pose 액션 서버가 없습니다 — planner_check.launch.py "
                "(또는 navigation.launch.py) 가 떠 있는지 확인하세요.")
            return False
        if self.args.smooth and not self.smooth.wait_for_server(timeout_sec=timeout):
            self.get_logger().error("smooth_path 액션 서버가 없습니다 (smoother_server).")
            return False
        return True

    def _send(self, client, goal_msg):
        future = client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            return None
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)
        wrapped = result_future.result()
        return None if wrapped is None else wrapped

    def plan(self, goal_xytheta, start_xytheta=None):
        goal = ComputePathToPose.Goal()
        goal.goal = pose_stamped(*goal_xytheta)
        goal.planner_id = "GridBased"
        if start_xytheta is not None:
            goal.start = pose_stamped(*start_xytheta)
            goal.use_start = True
        return self._send(self.compute, goal)

    def smooth_path(self, path):
        goal = SmoothPath.Goal()
        goal.path = path
        goal.smoother_id = "SmoothPath"
        goal.check_for_collisions = True
        return self._send(self.smooth, goal)


def duration_sec(duration):
    return duration.sec + duration.nanosec * 1e-9


def report(label, metrics, planning_time, occ_map):
    print(f"    {label:8} 점 {metrics['poses']:4d}", end="")
    if "length" not in metrics:
        print("   (경로가 너무 짧아 채점 불가)")
        return
    print(f"  길이 {metrics['length']:6.2f} m"
          f"  최소반경(cusp제외) {metrics['min_radius']:6.2f} m"
          f"  cusp {metrics['cusps']}"
          f"  계획 {planning_time * 1000:6.1f} ms")

    verdicts = []
    if metrics["min_radius"] < R_MIN - 0.05:
        verdicts.append(
            f"✗ 최소반경 {metrics['min_radius']:.2f} < R_min {R_MIN} — normal 모드로 못 따라감")
    if planning_time > MAX_PLANNING_TIME:
        verdicts.append(f"✗ 계획시간 {planning_time:.2f}s > max_planning_time {MAX_PLANNING_TIME}s")
    if "cells" in metrics:
        cells = metrics["cells"]
        total = sum(cells.values())
        unknown_ratio = cells["미관측"] / total if total else 0.0
        print(f"    {'':8} 통과 셀: 자유 {cells['자유']}  미관측 {cells['미관측']}"
              f"  점유 {cells['점유']}  맵밖 {cells['밖']}")
        if cells["점유"]:
            verdicts.append(f"✗ 점유 셀 {cells['점유']}개를 통과 — 벽을 뚫었다")
        if unknown_ratio > 0.05:
            verdicts.append(
                f"⚠ 경로의 {unknown_ratio * 100:.0f}% 가 **미관측 영역** — 매핑으로 확인한 적 "
                f"없는 공간이다"
                + (" (free_thresh 가 미관측을 자유공간으로 읽고 있다)"
                   if occ_map is not None and occ_map.unknown_reads_as_free() else ""))
    if "min_clearance" in metrics:
        x, y = metrics["min_clearance_at"]
        mark = "✗" if metrics["min_clearance"] < INSCRIBED else "·"
        print(f"    {'':8} 벽까지 최소 여유 {metrics['min_clearance']:.2f} m "
              f"@ ({x:.2f}, {y:.2f})  {mark} (inscribed {INSCRIBED})")
        if metrics["min_clearance"] < INSCRIBED:
            verdicts.append(
                f"✗ 여유 {metrics['min_clearance']:.2f} m < inscribed {INSCRIBED} m — 차체가 닿는다")
    for line in verdicts:
        print(f"    {'':8} {line}")


def main():
    parser = argparse.ArgumentParser(description="전역경로 생성 검증")
    parser.add_argument("--goal", nargs=3, type=float, action="append", metavar=("X", "Y", "YAW"),
                        required=True, help="목표 자세 (map 기준, yaw 는 rad). 여러 번 줄 수 있음")
    parser.add_argument("--start", nargs=3, type=float, metavar=("X", "Y", "YAW"),
                        help="시작 자세를 명시 (기본: TF 의 현재 로봇 위치)")
    parser.add_argument("--smooth", action="store_true",
                        help="smoother_server 까지 태워 실차 BT 와 같은 순서로 채점")
    parser.add_argument("--map", default="",
                        help="채점 기준 grid.yaml (기본: maps/active.yaml 의 활성 맵)")
    args, _ = parser.parse_known_args()

    map_yaml = args.map
    if not map_yaml:
        root = map_layout.maps_root(get_package_share_directory("alm_navigation"))
        active = map_layout.active_map_paths(root)
        map_yaml = active.grid_yaml if active else ""
    occ_map = None
    if map_yaml and os.path.isfile(map_yaml):
        try:
            occ_map = OccupancyMap(map_yaml)
        except Exception as error:                          # noqa: BLE001
            print(f"채점용 맵을 못 읽었습니다 ({map_yaml}): {error}")

    rclpy.init()
    node = PlanProbe(args)
    failures = 0
    try:
        if not node.wait_for_servers():
            return 2

        print(f"\n채점 기준 맵: {map_yaml}")
        if occ_map is not None and occ_map.unknown_reads_as_free():
            print(f"  ⚠ free_thresh={occ_map.free_thresh} 이라 map_server 는 미관측(205)을 "
                  f"**자유공간으로** 읽습니다 (0.196 이하여야 미관측으로 남습니다).")
        if args.start:
            print(f"시작: ({args.start[0]:.2f}, {args.start[1]:.2f}, "
                  f"{math.degrees(args.start[2]):.0f}°) [명시]")
        else:
            print("시작: TF 의 현재 로봇 위치 (virtual_pose_publisher 가 준 가상 위치)")

        for goal in args.goal:
            print(f"\n목표 ({goal[0]:.2f}, {goal[1]:.2f}, {math.degrees(goal[2]):.0f}°)")
            wrapped = node.plan(goal, args.start)
            if wrapped is None:
                print("    ✗ 계획 실패 — 액션이 거부되거나 응답이 없습니다")
                failures += 1
                continue
            result = wrapped.result
            if not result.path.poses:
                print(f"    ✗ 경로 없음 (status={wrapped.status})")
                failures += 1
                continue

            planning_time = duration_sec(result.planning_time)
            metrics = path_metrics(result.path.poses, occ_map)
            report("Hybrid-A*", metrics, planning_time, occ_map)

            if args.smooth:
                smoothed = node.smooth_path(result.path)
                if smoothed is None or not smoothed.result.path.poses:
                    print("    ✗ 평활화 실패 — BT 라면 Fallback 으로 원경로를 씁니다")
                    failures += 1
                else:
                    report("+Smoother", path_metrics(smoothed.result.path.poses, occ_map),
                           duration_sec(smoothed.result.smoothing_duration), occ_map)
                    if not smoothed.result.was_completed:
                        print(f"    {'':8} ⚠ 최적화가 수렴 전에 끊겼습니다 "
                              f"(max_iterations 또는 시간 제한)")
        print()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 1 if failures else 0


if __name__ == "__main__":
    sys.exit(main())
