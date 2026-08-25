#!/usr/bin/env python3
"""run_recorder — 자율주행 한 판을 기록하고 **왜 안 됐는지 자동으로 판정**한다.

    ros2 run alm_navigation run_recorder.py
    ros2 run alm_navigation run_recorder.py --ros-args -p out_dir:=~/ALM_Autunomous/logs

## 왜 필요한가

RViz 로는 이 스택에서 정작 중요한 것들이 안 보인다.

  · MPPI 가 요청한 wz 중 **몇 %가 조향 한계로 잘려나갔는지** — 화면에 없다
  · 전역경로가 **미관측 영역을 얼마나 지나는지** — 회색으로 보일 뿐이다
  · spin 에 몇 초를 썼고 그동안 **위치오차가 줄었는지**
  · dwell 로 정지해 있던 시간이 전체의 몇 %인지

게다가 RViz 는 실시간이라 놓치면 끝이고, 사람이 계속 붙어 있어야 하며,
여러 판을 비교할 수 없다. docs/TODO.md 에 이미 기록돼 있듯 **단일 실행 A/B 는
믿을 수 없다** (같은 목표가 323 s -> ABORT -> 163 s -> TIMEOUT 로 흔들린다).
판단하려면 반복 통계가 필요한데 지금까지 그것을 담을 그릇이 없었다.

## 무엇을 판정하나

주행이 끝나면(성공·중단·강제종료 무관) `summary.md` 에 소견이 남는다.
판정 규칙은 지금까지 실차에서 확인된 실패 모드를 그대로 옮긴 것이다.

    wz 클램프율 > 50%          MPPI 가 못 내는 회전을 계속 요구 (와리가리)
    경로의 미관측 통과 > 20%    맵의 가짜 자유공간 위로 계획됨
    /plan 발행 0회             전역경로 자체가 안 나옴
    spin 체류 > 5 s + 오차 안 줄어듦   spin 탈출 실패
    저속 정체 > 30%            어딘가 막혀 있음

## 강제종료 대응

`flush_sec` 마다 파일을 통째로 다시 쓴다. Ctrl+C 는 물론 `kill -9` 로 죽어도
마지막 flush 까지는 남는다 — 시그널 핸들러만 믿으면 SIGKILL 에 날아간다.

## 출력

    <out_dir>/run_<날짜시각>/summary.md    사람이 읽는 판정
    <out_dir>/<...>/metrics.json           숫자 (여러 판 비교·집계용)
"""
import json
import math
import os
import time
from datetime import datetime

import numpy as np
import rclpy
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from alm_msgs.msg import McuCommand

GOAL_STATUS = {0: "UNKNOWN", 1: "ACCEPTED", 2: "EXECUTING", 3: "CANCELING",
               4: "SUCCEEDED", 5: "CANCELED", 6: "ABORTED"}


# --------------------------------------------------------------------- 맵 읽기
def load_grid(yaml_path):
    """grid.yaml + grid.pgm 을 읽어 (값배열, 원점, 해상도) 를 준다.

    ##왜 costmap 이 아니라 원본 pgm 인가##
    costmap 은 unknown 을 어떻게 해석할지가 설정(allow_unknown, track_unknown_space)
    에 달려 있다. 우리가 알고 싶은 것은 해석이 아니라 **'저기를 실제로 본 적이
    있는가'** 라는 사실이다. 그건 원본 격자에만 있다. plan_probe.py 도 같은
    이유로 pgm 을 직접 읽는다.
    """
    with open(yaml_path, encoding="utf-8") as f:
        text = f.read()

    def field(key, default=None):
        for line in text.splitlines():
            line = line.strip()
            if line.startswith(key + ":"):
                return line.split(":", 1)[1].strip()
        return default

    img_name = (field("image") or "").strip("'\"")
    res = float(field("resolution", "0.05"))
    org = field("origin", "[0,0,0]").strip("[]")
    ox, oy = (float(v) for v in org.split(",")[:2])
    pgm = os.path.join(os.path.dirname(os.path.abspath(yaml_path)), img_name)

    with open(pgm, "rb") as f:
        data = f.read()
    i, toks = 0, []
    while len(toks) < 4:
        while data[i] in b" \t\r\n":
            i += 1
        if data[i:i + 1] == b"#":
            while data[i] not in b"\r\n":
                i += 1
            continue
        j = i
        while data[j] not in b" \t\r\n":
            j += 1
        toks.append(data[i:j])
        i = j
    i += 1
    w, h = int(toks[1]), int(toks[2])
    img = np.frombuffer(data[i:i + w * h], dtype=np.uint8).reshape(h, w)
    return np.flipud(img), (ox, oy), res     # flipud: pgm row0 = 상단


class RunRecorder(Node):
    def __init__(self):
        super().__init__("run_recorder")

        self.declare_parameter("out_dir", "~/ALM_Autunomous/logs")
        self.declare_parameter("map_yaml", "")        # 비우면 활성 맵 자동
        self.declare_parameter("flush_sec", 1.0)
        # 이 속도 미만이면 '정체' 로 센다 [m/s]. base_control 의
        # steer_limit_min_vx(0.03) 와 같은 값 — 그 아래는 조향으로 회전도 못 만든다.
        self.declare_parameter("stall_vx", 0.03)
        # 경로가 이 값(0~1) 이상 미관측을 지나면 소견을 낸다
        self.declare_parameter("warn_unknown_frac", 0.20)
        # wz 클램프율이 이 값 이상이면 소견을 낸다
        self.declare_parameter("warn_clamp_frac", 0.50)

        g = self.get_parameter
        base = os.path.expanduser(str(g("out_dir").value))
        self.dir = os.path.join(base, "run_" + datetime.now().strftime("%Y%m%d_%H%M%S"))
        os.makedirs(self.dir, exist_ok=True)
        self.flush_sec = float(g("flush_sec").value)
        self.stall_vx = float(g("stall_vx").value)
        self.warn_unknown = float(g("warn_unknown_frac").value)
        self.warn_clamp = float(g("warn_clamp_frac").value)

        # ---- 맵 (미관측 통과 판정용) ----
        self.grid = None
        my = str(g("map_yaml").value).strip()
        if not my:
            my = self._active_map_yaml()
        try:
            self.grid = load_grid(my)
            self.map_yaml = my
            self.get_logger().info(f"맵 로드: {my}  {self.grid[0].shape}")
        except Exception as exc:                                  # noqa: BLE001
            self.map_yaml = my
            self.get_logger().warn(
                f"맵을 못 읽었습니다({exc}) — '경로가 미관측을 지나는가' 판정은 생략합니다.")

        # ---- 누적 상태 ----
        self.t0 = time.time()
        self.mono0 = time.monotonic()
        self.last_flush = 0.0

        self.n_cmd = 0                 # /mcu/command 틱 수
        self.wz_req_sum = 0.0          # |요청 wz| 합 (Nav2 -> command_manager 입력)
        self.wz_act_sum = 0.0          # |실제 wz| 합 (McuCommand.cmd_vel)
        self.wz_clamped_ticks = 0      # 유의미하게 잘린 틱 수
        self.wz_req_ticks = 0          # 회전을 요청한 틱 수 (분모)
        self.wz_clamp_max = 0.0
        self.dist = 0.0                # 주행 거리 [m]
        self.stall_sec = 0.0
        self.stop_sec = 0.0            # 완전 정지(속도 0) 시간
        self.mode_sec = {}             # 유효모드별 체류 [s]
        self.mode_switches = 0
        self.cur_mode = None
        # ---- spin 구간 계측 ----
        # ##버그였던 것## 예전에는 구간 길이로 mode_sec["spin"](=전체 누적)을
        #   그대로 넣었다. 2 s 짜리 spin 이 세 번만 있어도 세 번째부터 '6 s 썼다'
        #   가 되어 판정이 오탐했다. 구간마다 진입 시각을 따로 잡는다.
        self.spin_enter_err = None     # spin 진입 시점의 목표까지 거리
        self.spin_enter_sec = None     # spin 진입 시각 (monotonic)
        self.spin_turned = 0.0         # 이번 spin 구간에서 실제로 돈 각도 [rad]
        self.last_yaw = None
        # 구간마다 (체류s, 목표거리변화m, 실제회전deg)
        self.spin_gain = []
        self.last_cmd_t = None

        self.plan_count = 0
        self.plan_unknown_fracs = []
        self.plan_len = None
        self.dev_sum = 0.0             # 경로 이탈 누적 (평균용)
        self.dev_n = 0
        self.dev_max = 0.0

        self.goal = None
        self.goal_status = "(없음)"
        self.pose = None               # (x, y, yaw) map 프레임
        self.last_pose_xy = None
        self.last_plan_pts = None

        # ---- 구독 ----
        vol = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10,
                         reliability=QoSReliabilityPolicy.RELIABLE,
                         durability=QoSDurabilityPolicy.VOLATILE)
        tl = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                        reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Twist, "/cmd_vel", self._on_nav_cmd, vol)
        self.create_subscription(McuCommand, "/mcu/command", self._on_mcu, vol)
        self.create_subscription(Path, "/plan", self._on_plan, vol)
        self.create_subscription(Odometry, "/Odometry", self._on_odom, vol)
        self.create_subscription(String, "/drive_mode/effective", self._on_mode, vol)
        self.create_subscription(PoseStamped, "/goal_pose", self._on_goal, vol)
        self.create_subscription(GoalStatusArray, "/navigate_to_pose/_action/status",
                                 self._on_status, tl)

        self.tf_buf = Buffer()
        self.tf_listener = TransformListener(self.tf_buf, self)

        self.nav_wz = 0.0              # 최근 Nav2 요청 wz
        self.nav_vx = 0.0
        self.create_timer(0.1, self._tick)
        self.get_logger().info(f"run_recorder 시작 -> {self.dir}")
        self.get_logger().info(
            f"{self.flush_sec:.0f} s 마다 저장합니다. 강제종료(kill -9)해도 남습니다.")

    # ---------------------------------------------------------------- 유틸
    def _active_map_yaml(self):
        from ament_index_python.packages import get_package_share_directory
        import sys
        share = get_package_share_directory("alm_navigation")
        sys.path.insert(0, os.path.join(share, "launch"))
        import map_layout
        return map_layout.active_map_paths(map_layout.maps_root(share)).grid_yaml

    def _robot_pose(self):
        """map->base_link. /Odometry 는 odom 프레임이라 map 프레임 /plan 과
        직접 비교하면 map->odom 오프셋만큼 통째로 틀어진다(command_manager 와 동일)."""
        try:
            tf = self.tf_buf.lookup_transform("map", "base_link", rclpy.time.Time())
        except Exception:                                          # noqa: BLE001
            return None
        t, q = tf.transform.translation, tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def _cell(self, x, y):
        img, (ox, oy), res = self.grid
        h, w = img.shape
        c, r = int((x - ox) / res), int((y - oy) / res)
        if 0 <= c < w and 0 <= r < h:
            return int(img[r, c])
        return None

    # ---------------------------------------------------------------- 구독
    def _on_nav_cmd(self, msg):
        self.nav_wz, self.nav_vx = msg.angular.z, msg.linear.x

    def _on_mcu(self, msg):
        now = time.monotonic()
        dt = 0.0 if self.last_cmd_t is None else min(now - self.last_cmd_t, 0.5)
        self.last_cmd_t = now
        self.n_cmd += 1

        req, act = abs(self.nav_wz), abs(msg.cmd_vel.angular.z)
        self.wz_req_sum += req
        self.wz_act_sum += act
        if req > 0.02:                       # 회전을 실제로 요청한 틱만 분모에 넣는다
            self.wz_req_ticks += 1
            gap = (req - act) / req
            if gap > 0.05:
                self.wz_clamped_ticks += 1
                self.wz_clamp_max = max(self.wz_clamp_max, gap)

        vx = abs(msg.cmd_vel.linear.x)
        if vx < 1e-4:
            self.stop_sec += dt
        elif vx < self.stall_vx:
            self.stall_sec += dt
        if self.cur_mode:
            self.mode_sec[self.cur_mode] = self.mode_sec.get(self.cur_mode, 0.0) + dt

    def _on_mode(self, msg):
        m = msg.data
        if m == self.cur_mode:
            return
        # spin 구간이 끝났으면 '얼마나 돌았고 목표에 얼마나 가까워졌나' 를 남긴다.
        #   · 실제 회전각 : ALIGN 이 한 기동으로 끝내는지 보는 값. 재래치가
        #     꺼져 있으면 한 구간이 '래치각 - 이탈각' 에서 잘린다.
        #   · 목표거리변화 : spin 은 제자리 회전이라 원래 안 줄어든다. 참고값.
        if self.cur_mode == "spin" and self.spin_enter_sec is not None:
            d = self._goal_dist()
            closed = (self.spin_enter_err - d
                      if (d is not None and self.spin_enter_err is not None) else None)
            self.spin_gain.append((time.monotonic() - self.spin_enter_sec,
                                   closed, math.degrees(self.spin_turned)))
            self.spin_enter_sec = self.spin_enter_err = None
        if m == "spin":
            self.spin_enter_sec = time.monotonic()
            self.spin_enter_err = self._goal_dist()
            self.spin_turned = 0.0
        if self.cur_mode is not None:
            self.mode_switches += 1
        self.cur_mode = m

    def _goal_dist(self):
        if self.goal is None or self.pose is None:
            return None
        return math.hypot(self.goal[0] - self.pose[0], self.goal[1] - self.pose[1])

    def _on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y)
        self.goal_status = "EXECUTING"

    def _on_status(self, msg):
        if msg.status_list:
            self.goal_status = GOAL_STATUS.get(msg.status_list[-1].status, "UNKNOWN")

    def _on_odom(self, msg):
        p = msg.pose.pose.position
        if self.last_pose_xy is not None:
            self.dist += math.hypot(p.x - self.last_pose_xy[0], p.y - self.last_pose_xy[1])
        self.last_pose_xy = (p.x, p.y)

    def _on_plan(self, msg):
        self.plan_count += 1
        pts = np.array([[q.pose.position.x, q.pose.position.y] for q in msg.poses])
        if pts.size == 0:
            return
        self.last_plan_pts = pts
        seg = np.hypot(*(pts[1:] - pts[:-1]).T) if len(pts) > 1 else np.array([0.0])
        self.plan_len = float(seg.sum())
        if self.grid is None:
            return
        # 이 경로가 '한 번도 본 적 없는 곳' 을 얼마나 지나는가
        vals = [self._cell(x, y) for x, y in pts]
        seen = [v for v in vals if v is not None]
        if seen:
            self.plan_unknown_fracs.append(
                sum(1 for v in seen if v == 205) / len(seen))

    # ---------------------------------------------------------------- 주기
    def _tick(self):
        self.pose = self._robot_pose() or self.pose
        # spin 중 실제 회전각 누적. wrap 차분을 더하므로 180° 를 넘겨도 맞다.
        if self.pose is not None:
            if self.last_yaw is not None and self.cur_mode == "spin":
                d = math.atan2(math.sin(self.pose[2] - self.last_yaw),
                               math.cos(self.pose[2] - self.last_yaw))
                self.spin_turned += abs(d)
            self.last_yaw = self.pose[2]
        if self.pose is not None and self.last_plan_pts is not None:
            d = np.hypot(self.last_plan_pts[:, 0] - self.pose[0],
                         self.last_plan_pts[:, 1] - self.pose[1]).min()
            self.dev_sum += float(d)
            self.dev_n += 1
            self.dev_max = max(self.dev_max, float(d))
        if (time.monotonic() - self.mono0) - self.last_flush >= self.flush_sec:
            self.last_flush = time.monotonic() - self.mono0
            self.save()

    # ---------------------------------------------------------------- 판정
    def metrics(self):
        el = max(time.monotonic() - self.mono0, 1e-6)
        clamp_frac = (self.wz_clamped_ticks / self.wz_req_ticks
                      if self.wz_req_ticks else 0.0)
        unk = (float(np.mean(self.plan_unknown_fracs))
               if self.plan_unknown_fracs else None)
        return {
            "started": datetime.fromtimestamp(self.t0).isoformat(timespec="seconds"),
            "elapsed_sec": round(el, 1),
            "goal_status": self.goal_status,
            "goal_xy": self.goal,
            "distance_m": round(self.dist, 2),
            "goal_dist_m": (round(self._goal_dist(), 2)
                            if self._goal_dist() is not None else None),
            "map_yaml": self.map_yaml,
            "plan": {
                "publishes": self.plan_count,
                "length_m": round(self.plan_len, 2) if self.plan_len else None,
                "unknown_frac": round(unk, 4) if unk is not None else None,
                "deviation_mean_m": round(self.dev_sum / self.dev_n, 3) if self.dev_n else None,
                "deviation_max_m": round(self.dev_max, 3) if self.dev_n else None,
            },
            "wz": {
                "requested_mean": round(self.wz_req_sum / self.n_cmd, 4) if self.n_cmd else None,
                "actual_mean": round(self.wz_act_sum / self.n_cmd, 4) if self.n_cmd else None,
                "clamp_frac": round(clamp_frac, 4),
                "clamp_max": round(self.wz_clamp_max, 4),
                "ticks_requesting_turn": self.wz_req_ticks,
            },
            "mode": {
                "seconds": {k: round(v, 1) for k, v in sorted(self.mode_sec.items())},
                "switches": self.mode_switches,
                "spin_segments": [
                    {"sec": round(a, 1),
                     "closed_m": None if b is None else round(b, 2),
                     "turned_deg": None if c is None else round(c, 1)}
                    for a, b, c in self.spin_gain],
            },
            "motion": {
                "stopped_sec": round(self.stop_sec, 1),
                "stalled_sec": round(self.stall_sec, 1),
                "stopped_frac": round(self.stop_sec / el, 3),
            },
        }

    def findings(self, m):
        """숫자를 사람이 읽는 소견으로. 심각도 순으로 정렬해 돌려준다."""
        out = []
        unk = m["plan"]["unknown_frac"]
        if unk is not None and unk >= self.warn_unknown:
            out.append((90, f"전역경로의 **{unk*100:.0f}%** 가 미관측 영역(205)을 지납니다. "
                            f"맵에서 '가본 적 없는 곳'을 통행 가능으로 알고 계획한 것입니다. "
                            f"grid.yaml 의 free_thresh(0.19 이어야 함)와 "
                            f"SmacPlannerHybrid.allow_unknown(false 권장)을 확인하세요."))
        cf = m["wz"]["clamp_frac"]
        if cf >= self.warn_clamp and m["wz"]["ticks_requesting_turn"] > 20:
            out.append((80, f"회전 요청의 **{cf*100:.0f}%** 가 조향 한계로 잘렸습니다 "
                            f"(요청 평균 {m['wz']['requested_mean']}, "
                            f"실제 평균 {m['wz']['actual_mean']}). MPPI 가 이 플랫폼이 "
                            f"낼 수 없는 회전을 계속 요구하고 있습니다 — 좌우 진동의 "
                            f"주 원인입니다. nav2.yaml 의 motion_model 이 Ackermann 인지, "
                            f"wz_max/wz_std 가 적절한지 확인하세요."))
        if m["plan"]["publishes"] == 0 and self.goal is not None:
            out.append((95, "**/plan 이 한 번도 발행되지 않았습니다.** 전역경로 자체가 "
                            "안 나온 것입니다. 목표가 미관측/장애물 위에 있거나, "
                            "allow_unknown=false 에서 도달 불가한 목표일 수 있습니다."))
        # ##판정 기준을 바꿨다## 예전에는 'spin 이 긴데 목표거리가 안 줄었다' 를
        #   봤는데, ALIGN 으로 도는 spin 은 **원래** 거리를 안 줄인다(제자리 회전).
        #   실제로 나쁜 것은 '오래 머물렀는데 회전도 안 했다' 이다.
        for seg in m["mode"]["spin_segments"]:
            td = seg["turned_deg"]
            if seg["sec"] >= 5.0 and td is not None and td < 10.0:
                out.append((70, f"spin 에 {seg['sec']:.0f} s 를 썼는데 실제 회전은 "
                                f"{td:.1f}° 뿐입니다. 돌지도 못하면서 모드만 붙들고 "
                                f"있었다는 뜻입니다 — 탈출 조건"
                                f"(auto_spin_release_angular_threshold)이나 "
                                f"모드 전환 dwell(mode_switch_dwell_sec)을 보세요."))
        segs = m["mode"]["spin_segments"]
        if len(segs) >= 4:
            tot = sum(s["sec"] for s in segs)
            turn = sum(s["turned_deg"] or 0.0 for s in segs)
            out.append((75, f"spin 구간이 **{len(segs)}회** 나왔습니다 "
                            f"(합계 {tot:.0f} s, 총 회전 {turn:.0f}°). 한 번에 못 돌고 "
                            f"쪼개진 것이라면 왕복마다 모드 전환 dwell 이 두 번씩 "
                            f"붙습니다 — align_relatch_stopped 가 true 인지, "
                            f"align_cooldown_sec 이 mode_switch_dwell_sec 이상인지 "
                            f"확인하세요."))
        sf = m["motion"]["stopped_frac"]
        if sf >= 0.30:
            out.append((60, f"전체 시간의 **{sf*100:.0f}%** 를 정지 상태로 보냈습니다 "
                            f"({m['motion']['stopped_sec']:.0f} s). 모드 전환 dwell"
                            f"(5 s x {m['mode']['switches']}회)이 주 원인인지 확인하세요."))
        dev = m["plan"]["deviation_max_m"]
        if dev is not None and dev > 1.0:
            out.append((50, f"경로에서 최대 {dev:.2f} m 벗어났습니다 "
                            f"(평균 {m['plan']['deviation_mean_m']:.2f} m). 추종 실패입니다."))
        if not out:
            out.append((0, "자동 판정에서 걸린 항목이 없습니다. metrics.json 의 숫자를 "
                           "직접 보거나, 여러 판을 모아 비교하세요."))
        return [t for _, t in sorted(out, key=lambda x: -x[0])]

    # ---------------------------------------------------------------- 저장
    def save(self):
        m = self.metrics()
        tmp = os.path.join(self.dir, ".metrics.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            json.dump(m, f, ensure_ascii=False, indent=2)
        os.replace(tmp, os.path.join(self.dir, "metrics.json"))

        lines = [
            f"# 주행 기록 — {m['started']}",
            "",
            f"- 결과: **{m['goal_status']}**   경과 {m['elapsed_sec']:.0f} s   "
            f"이동 {m['distance_m']:.1f} m"
            + (f"   목표까지 {m['goal_dist_m']:.1f} m" if m["goal_dist_m"] is not None else ""),
            f"- 맵: `{os.path.basename(os.path.dirname(m['map_yaml']))}`",
            "",
            "## 소견",
            "",
        ]
        lines += [f"{i}. {t}" for i, t in enumerate(self.findings(m), 1)]
        unk = m["plan"]["unknown_frac"]
        unk_txt = "—" if unk is None else f"{unk * 100:.1f}%"
        dev_m, dev_x = m["plan"]["deviation_mean_m"], m["plan"]["deviation_max_m"]
        dev_txt = "—" if dev_m is None else f"{dev_m:.2f} / {dev_x:.2f} m"
        wz_r, wz_a = m["wz"]["requested_mean"], m["wz"]["actual_mean"]
        wz_txt = "—" if wz_r is None else f"{wz_r:.3f} / {wz_a:.3f} rad/s"
        mode_txt = ("—" if not m["mode"]["seconds"] else
                    ", ".join(f"{k} {v:.0f}s" for k, v in m["mode"]["seconds"].items()))
        lines += [
            "",
            "## 숫자",
            "",
            "| 항목 | 값 |",
            "|---|---|",
            f"| /plan 발행 | {m['plan']['publishes']}회 |",
            f"| 경로의 미관측 통과 | {unk_txt} |",
            f"| 경로 이탈 (평균/최대) | {dev_txt} |",
            f"| wz 클램프율 | {m['wz']['clamp_frac']*100:.1f}%  (최대 {m['wz']['clamp_max']*100:.0f}%) |",
            f"| wz 요청/실제 평균 | {wz_txt} |",
            f"| 모드별 체류 | {mode_txt} |",
            f"| 모드 전환 | {m['mode']['switches']}회 |",
            f"| spin 구간 | " + ("—" if not m["mode"]["spin_segments"] else
                                 ", ".join(
                                     f"{s['sec']:.0f}s/{(s['turned_deg'] or 0):.0f}°"
                                     for s in m["mode"]["spin_segments"])) + " |",
            f"| 정지 / 저속정체 | {m['motion']['stopped_sec']:.0f} s / {m['motion']['stalled_sec']:.0f} s |",
            "",
            "> 이 파일은 주행 중 계속 갱신됩니다. 강제종료해도 마지막 갱신까지는 남습니다.",
        ]
        tmp = os.path.join(self.dir, ".summary.tmp")
        with open(tmp, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")
        os.replace(tmp, os.path.join(self.dir, "summary.md"))


def main():
    rclpy.init()
    node = RunRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.get_logger().info(f"최종 저장: {node.dir}/summary.md")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
