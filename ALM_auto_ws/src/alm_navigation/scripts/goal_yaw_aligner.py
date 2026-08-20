#!/usr/bin/env python3
"""goal_yaw_aligner — 목표 위치에 도착한 뒤 **제자리 회전으로 목표 자세를 맞춘다.**

── 왜 필요한가 ───────────────────────────────────────────────────────────────
SmacPlannerHybrid 는 `minimum_turning_radius`(1.643 m)를 만족하는 모션 프리미티브
로만 탐색한다. **그 집합에 제자리 회전이 없다.** 그래서

    "목표에서 0.16 m 앞인데 자세를 180° 바꿔야 한다"

같은 요구에는 **해 자체가 존재하지 않는다.** 플래너는 빈 경로(`Path is empty`)를
내고, 컨트롤러는 진척 없음을 보고하고, BT 가 리커버리를 소진한 뒤 abort 한다.
시뮬레이션 실측: 0.16 m 앞에서 180° 회전만 하면 되는데 4.05 m 를 헤매다 실패
(우회 24.6배). docs/control_pipeline.md §12.5.2

**이 플랫폼은 4륜 독립조향이라 제자리 회전(`spin`)이 되는데, 플래너가 그 능력을
쓸 수 없다.** 이 노드가 그 구멍을 메운다 — 위치는 Nav2 가, 자세는 여기가 맡는다.

── 어떻게 ────────────────────────────────────────────────────────────────────
Nav2 를 건드리지 않고 **밖에서** 관찰한다. 커스텀 BT 플러그인(C++)이 필요 없다.

    1. **/plan 의 마지막 pose** 를 목표로 삼는다.
       Hybrid-A* 는 목표 자세를 보존하므로 경로 끝 pose = 목표 pose 다.
       목표를 액션(navigate_to_pose)으로 주든 토픽으로 주든 항상 보인다는 것이
       요점이다 — 액션 goal 은 서비스라 구독할 수 없다.
       (/goal_pose 가 발행되는 환경이면 그쪽이 더 직접적이므로 함께 받는다)
    2. TF(map->base_link)로 로봇을 본다
    3. '도착 + 정지' 가 settle_sec 동안 유지되면:
           위치오차 < xy_tolerance  AND  속도 명령 ≈ 0
    4. 자세오차 > yaw_tolerance 면 nav2 /spin 액션을 상대각으로 호출
    5. 한 목표당 한 번만 (새 목표가 오면 리셋)

`spin` 액션은 behavior_server 가 `/cmd_vel` 로 내보내고, 그 뒤는 평소 경로
(cmd_arbiter -> command_manager)를 그대로 탄다. command_manager 의 auto 상태머신이
`vx≈0, |wz|>=auto_spin_angular_threshold` 를 보고 spin 모드로 라우팅한다.

── 안전 ──────────────────────────────────────────────────────────────────────
· **'정지 상태' 에서만** 동작한다. 주행 중에는 절대 끼어들지 않는다
  (속도 명령이 0 이 아니면 트리거 자체가 안 걸림).
· 위치오차가 xy_tolerance 안일 때만. 경로 중간에 잠깐 서는 것과 구분된다.
· 회전량이 max_spin_rad 를 넘으면 거부한다(오작동 방지).
· 실패해도 주행에는 영향이 없다 — 자세만 안 맞은 채로 남는다.

── 한계 ──────────────────────────────────────────────────────────────────────
· `navigate_to_pose` 액션이 SUCCEEDED 를 돌려준 **뒤에** 정렬이 진행된다.
  클라이언트가 액션 결과만 보고 다음 동작을 하면 정렬 중일 수 있다.
  ##TODO## 근본 해결은 BT 안에서 하는 것(커스텀 BT 노드) 또는
  SmacPlannerLattice 로 제자리회전 프리미티브를 넣는 것.
· `/plan` 이 한 번도 안 나오면(플래너가 전부 실패) 목표를 알 수 없어 아무것도 안 한다.
  그때는 애초에 주행 자체가 실패한 상황이다.
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
from nav2_msgs.action import Spin


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class GoalYawAligner(Node):
    def __init__(self):
        super().__init__("goal_yaw_aligner")
        g = self.declare_parameter
        g("plan_topic", "/plan")
        g("goal_topic", "/goal_pose")
        # command_manager 로 실제 나가는 속도 명령 (cmd_arbiter 출력)
        g("cmd_topic", "/cmd_vel_mux")
        g("map_frame", "map")
        g("base_frame", "base_link")
        # Nav2 general_goal_checker.xy_goal_tolerance 와 맞춘다
        g("xy_tolerance", 0.20)
        # 이보다 크게 어긋나 있으면 정렬한다. Nav2 yaw_goal_tolerance(0.25)보다
        # 살짝 작게 두어 '거의 맞은' 경우에 굳이 돌지 않게 한다.
        g("yaw_tolerance", 0.20)
        # 이 시간 동안 '도착 + 정지'가 유지되어야 정렬을 시작한다
        g("settle_sec", 2.0)
        # 속도 명령이 이 값 이하이면 정지로 본다
        g("stopped_eps", 0.02)
        # 안전 상한 — 이보다 큰 회전은 거부 (기본 π = 180°)
        g("max_spin_rad", math.pi + 0.05)
        g("enabled", True)

        p = self.get_parameter
        self.xy_tol = float(p("xy_tolerance").value)
        self.yaw_tol = float(p("yaw_tolerance").value)
        self.settle = float(p("settle_sec").value)
        self.eps = float(p("stopped_eps").value)
        self.max_spin = float(p("max_spin_rad").value)
        self.enabled = bool(p("enabled").value)
        self.map_frame = p("map_frame").value
        self.base_frame = p("base_frame").value

        self.goal = None          # (x, y, yaw)
        self.done_for_goal = True # 새 목표가 와야 False
        self.stopped_since = None
        self.busy = False
        self.last_cmd = 0.0

        self.tfbuf = Buffer()
        self.tfl = TransformListener(self.tfbuf, self)
        self.spin_client = ActionClient(self, Spin, "spin")
        self.create_subscription(Path, p("plan_topic").value, self._on_plan, 10)
        self.create_subscription(PoseStamped, p("goal_topic").value, self._on_goal, 10)
        self.create_subscription(Twist, p("cmd_topic").value, self._on_cmd, 20)
        self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f"goal_yaw_aligner 시작 — 도착 후 자세오차 > {math.degrees(self.yaw_tol):.0f}° 면 "
            f"spin 으로 정렬 (위치오차 < {self.xy_tol} m, {self.settle:.1f} s 정지 유지 시). "
            f"{'활성' if self.enabled else '비활성(enabled=false)'}")

    # ------------------------------------------------------------------ 입력
    def _on_goal(self, msg: PoseStamped):
        self.goal = (msg.pose.position.x, msg.pose.position.y,
                     yaw_of(msg.pose.orientation))
        self.done_for_goal = False
        self.stopped_since = None
        self.get_logger().info(
            f"새 목표: ({self.goal[0]:.2f}, {self.goal[1]:.2f}) "
            f"자세 {math.degrees(self.goal[2]):+.1f}°")

    def _on_plan(self, msg: Path):
        """경로의 마지막 pose = 목표 pose (Hybrid-A* 는 목표 자세를 보존한다)."""
        if not msg.poses:
            return
        q = msg.poses[-1].pose
        goal = (q.position.x, q.position.y, yaw_of(q.orientation))
        if self.goal is not None and \
           math.hypot(goal[0] - self.goal[0], goal[1] - self.goal[1]) < 0.02 and \
           abs(wrap_pi(goal[2] - self.goal[2])) < 0.02:
            return                      # 같은 목표 — 재무장하지 않는다
        self.goal = goal
        self.done_for_goal = False
        self.stopped_since = None
        self.get_logger().info(
            f"목표 갱신(/plan): ({goal[0]:.2f}, {goal[1]:.2f}) "
            f"자세 {math.degrees(goal[2]):+.1f}°")

    def _on_cmd(self, msg: Twist):
        self.last_cmd = max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z))

    # ------------------------------------------------------------------ 판정
    def _tick(self):
        if not self.enabled or self.goal is None or self.done_for_goal or self.busy:
            return
        try:
            t = self.tfbuf.lookup_transform(
                self.map_frame, self.base_frame, rclpy.time.Time())
        except Exception:
            return
        x, y = t.transform.translation.x, t.transform.translation.y
        yaw = yaw_of(t.transform.rotation)
        gx, gy, gyaw = self.goal

        arrived = math.hypot(x - gx, y - gy) <= self.xy_tol
        stopped = self.last_cmd <= self.eps
        now = self._now()
        if not (arrived and stopped):
            self.stopped_since = None
            return
        if self.stopped_since is None:
            self.stopped_since = now
            return
        if (now - self.stopped_since) < self.settle:
            return

        err = wrap_pi(gyaw - yaw)
        self.done_for_goal = True          # 목표당 한 번만
        if abs(err) <= self.yaw_tol:
            self.get_logger().info(
                f"자세 이미 정렬됨 ({math.degrees(err):+.1f}°) — spin 생략")
            return
        if abs(err) > self.max_spin:
            self.get_logger().warn(
                f"요구 회전 {math.degrees(err):+.1f}° 가 상한 "
                f"{math.degrees(self.max_spin):.0f}° 초과 — 정렬 거부")
            return
        self._do_spin(err)

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ------------------------------------------------------------------ 실행
    def _do_spin(self, err):
        if not self.spin_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().warn("spin 액션 서버 없음 — 정렬 생략 (behavior_server 확인)")
            return
        self.busy = True
        goal = Spin.Goal()
        goal.target_yaw = float(err)          # 상대각
        # 회전 + 모드전환 dwell(기본 3 s) 을 감안해 넉넉히
        goal.time_allowance.sec = 30
        self.get_logger().info(
            f"목표 자세 정렬: {math.degrees(err):+.1f}° 제자리 회전 시작")
        fut = self.spin_client.send_goal_async(goal)
        fut.add_done_callback(self._on_sent)

    def _on_sent(self, fut):
        gh = fut.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn("spin 목표가 거부됨 — 정렬 생략")
            self.busy = False
            return
        gh.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut):
        self.busy = False
        try:
            status = fut.result().status
        except Exception:
            self.get_logger().warn("spin 결과 수신 실패")
            return
        if status == 4:      # SUCCEEDED
            self.get_logger().info("목표 자세 정렬 완료")
        else:
            self.get_logger().warn(
                f"목표 자세 정렬 실패 (status={status}) — 자세만 안 맞은 채로 둡니다")


def main():
    rclpy.init()
    n = GoalYawAligner()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
