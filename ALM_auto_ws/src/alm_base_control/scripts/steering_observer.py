#!/usr/bin/env python3
"""steering_observer — 측위 헤딩만으로 '실제 조향각' 을 역추정하는 관측기. **로깅 전용.**

STM32 업링크가 미구현이라(uart_protocol.md v2 §State) 실측 조향각이 없습니다.
그런데 조향각은 곡률과 일대일 대응이고, 곡률은 로봇이 실제로 그린 궤적에
그대로 남습니다. 그래서 측위 결과만으로 되짚을 수 있습니다.

    κ = Δθ / Δs        (헤딩 변화 ÷ 이동 거리)
    R = 1 / κ
    δ = solve_inner_front_steer(vx=1, wz=κ)     # ICR_y = 1/κ = R

── 왜 '시간' 이 아니라 '거리' 로 미분하나 ────────────────────────────────────
직관적으로는 ω = dθ/dt 를 구하고 R = v/ω 로 가고 싶습니다. 그런데

  · /Odometry.twist 가 **항상 0** 입니다 (FAST-LIO 가 pose 만 채움) — v 가 없습니다.
  · pose 를 미분해 v 를 만들면, ω 도 v 도 둘 다 미분 잡음인데 그걸 또 나눕니다.

곡률은 원래 시간이 아니라 **거리**로 정의됩니다(κ = dθ/ds). 거리 기준으로 가면
속도가 아예 필요 없고, /Odometry.pose 하나로 끝납니다. 덤으로 저속에서도
동작합니다 — 느리면 창이 차는 데 오래 걸릴 뿐 정확도는 그대로입니다.
(요레이트 기반 관측기는 v→0 에서 원리적으로 무력해집니다.)

── 프레임 ────────────────────────────────────────────────────────────────────
/Odometry 는 **odom 프레임**입니다 (laserMapping.cpp: header.frame_id = odom_frame).
map→odom 이 재측위 점프를 흡수하므로 odom 프레임 헤딩은 연속입니다. 그래서
그냥 미분해도 됩니다. map 프레임 헤딩을 쓰면 재측위 때 최대 5°
(consistency_rotation_deg) 점프가 조향 킥으로 오독됩니다.

── 정확도 (σ_δ ≈ 55 · σ_θ·√2 / Δs) ──────────────────────────────────────────
    σ_θ \ Δs     0.2 m    0.5 m    1.0 m
      0.1°       0.68°    0.27°    0.14°
      0.2°       1.36°    0.54°    0.27°
      0.5°       3.39°    1.36°    0.68°
Δs=0.5 m 이면 어떤 경우에도 1.4° 이내입니다. 감시용으로 충분합니다.
(σ_θ 는 정지 상태에서 /Odometry 를 몇 분 녹화하면 측정됩니다.)

── 이 관측기가 잡는 것 / 못 잡는 것 ─────────────────────────────────────────
  ✅ 편향: max_steer_deg · rws_ratio · wheelbase/track 오차, 조향 부호 반전,
          기계 정렬 오차  → 정상 선회 구간에서 잘 보입니다
  ❌ 지연: 창(Δs)이 시간을 뭉갭니다. 조향 서보 응답 τ 는 이 방법으로 못 잽니다
  ❌ 정지 중 조향각: ds=0 이면 κ 가 정의되지 않습니다 (원리적 한계)
  ❌ gear_ratio / wheel_radius: 곡률은 속도와 무관합니다 — 직진 구간에서 별도로
     '명령 rpm ↔ 실제 속도' 를 봐야 합니다. 두 오차원이 분리되는 건 오히려 장점입니다

── 지연 정렬 ─────────────────────────────────────────────────────────────────
δ_actual(t) 는 δ_cmd(t − τ) 의 결과입니다. 정렬하지 않으면 **지연을 편향으로
오독**합니다. steer_lag_sec 로 명령을 당겨서 비교하고, 판정은
|dδ_cmd/dt| 가 작은(=지연이 간극을 안 만드는) 구간에서만 합니다.

── 사용 ──────────────────────────────────────────────────────────────────────
    ros2 run alm_base_control steering_observer.py
    ros2 topic echo /steer/observed        # [명령δ, 관측δ, 간극, κ, Δs, 유효]

**제어 경로에 아무것도 쓰지 않습니다.** 관측 결과를 보고 대책을 고르는 것은
사람의 몫입니다 — docs/control_pipeline.md §7.3.
"""

import math
import os
import sys
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from alm_msgs.msg import McuCommand


def _import_fourwis():
    """command_manager 와 같은 기구학 모듈을 쓴다 (설치본 우선, 없으면 소스 트리)."""
    cands = []
    try:
        from ament_index_python.packages import get_package_prefix
        cands.append(os.path.join(
            get_package_prefix("alm_base_control"), "lib", "alm_base_control"))
    except Exception:
        pass
    cands.append(os.path.dirname(os.path.abspath(__file__)))
    for d in cands:
        if os.path.exists(os.path.join(d, "fourwis_encode.py")):
            sys.path.insert(0, d)
            import fourwis_encode as mod
            return mod
    raise ImportError("fourwis_encode.py 를 찾지 못했습니다")


fourwis_encode = _import_fourwis()

MODE_NORMAL = 1


def yaw_from_quat(q):
    """평면 주행이므로 yaw 만 뽑는다."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class SteeringObserver(Node):
    def __init__(self):
        super().__init__("steering_observer")
        g = self.declare_parameter
        g("odom_topic", "/Odometry")
        g("command_topic", "/mcu/command")
        g("output_topic", "/steer/observed")
        # 창 이동거리 [m]. 크면 정확하고 느리게, 작으면 빠르고 시끄럽게.
        g("window_arc_m", 0.5)
        # 창을 유지할 최대 시간 [s]. 저속에서 창이 영원히 안 차는 걸 막는다.
        g("window_max_age_sec", 20.0)
        # 조향 지연 추정치 [s]. 명령을 이만큼 당겨서 비교한다. ##CONFIRM## 미측정.
        g("steer_lag_sec", 0.25)
        # 이 이상 간극이 이 시간 이상 지속되면 경고
        g("gap_warn_deg", 5.0)
        g("gap_warn_hold_sec", 3.0)
        # 판정 게이트: 명령 조향 변화율이 이보다 크면 '지연 구간' 으로 보고 편향 판정 제외
        g("bias_gate_rate_deg_s", 5.0)
        # 기구 상수 (base_control.yaml 과 같은 값이어야 한다)
        g("wheelbase_m", 1.0)
        g("track_m", 0.919)
        g("rws_ratio", 0.5)
        g("max_steer_deg", 30.0)

        p = self.get_parameter
        self.arc_min = float(p("window_arc_m").value)
        self.age_max = float(p("window_max_age_sec").value)
        self.lag = float(p("steer_lag_sec").value)
        self.gap_warn = float(p("gap_warn_deg").value)
        self.gap_hold = float(p("gap_warn_hold_sec").value)
        self.bias_gate = float(p("bias_gate_rate_deg_s").value)
        self.wis = fourwis_encode.FourWISParams(
            wheelbase_m=float(p("wheelbase_m").value),
            track_m=float(p("track_m").value),
            rws_ratio=float(p("rws_ratio").value),
            max_steer_deg=float(p("max_steer_deg").value),
        )

        # (t, x, y, yaw) 슬라이딩 창
        self.win = deque()
        # (t, steer_deg, speed_rpm, mode_id) 명령 이력
        self.cmd_hist = deque(maxlen=2000)
        self.gap_since = None

        self.pub = self.create_publisher(Float32MultiArray, p("output_topic").value, 10)
        self.create_subscription(Odometry, p("odom_topic").value, self._on_odom, 20)
        self.create_subscription(McuCommand, p("command_topic").value, self._on_cmd, 50)

        self.get_logger().info(
            f"steering_observer 시작 — 창 {self.arc_min:.2f} m, "
            f"지연 정렬 {self.lag:.3f} s (##CONFIRM##), R_min "
            f"{fourwis_encode.min_turn_radius(self.wis):.3f} m. 로깅 전용입니다.")

    # ---------------------------------------------------------------- 입력
    def _on_cmd(self, msg: McuCommand):
        t = msg.stamp.sec + msg.stamp.nanosec * 1e-9
        self.cmd_hist.append((t, float(msg.steer_deg), float(msg.speed_rpm),
                              int(msg.mode_id)))

    def _cmd_at(self, t):
        """시각 t 에 가장 가까운 명령. 없으면 None."""
        if not self.cmd_hist:
            return None
        best = min(self.cmd_hist, key=lambda c: abs(c[0] - t))
        return best if abs(best[0] - t) < 1.0 else None

    def _cmd_rate_at(self, t, span=0.3):
        """시각 t 부근의 |dδ_cmd/dt| [deg/s]. 편향 판정 게이트용."""
        seg = [c for c in self.cmd_hist if abs(c[0] - t) <= span]
        if len(seg) < 2:
            return float("inf")
        dt = seg[-1][0] - seg[0][0]
        if dt <= 1e-6:
            return float("inf")
        return abs(seg[-1][1] - seg[0][1]) / dt

    # ---------------------------------------------------------------- 관측
    def _on_odom(self, msg: Odometry):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        yaw = yaw_from_quat(msg.pose.pose.orientation)
        self.win.append((t, x, y, yaw))

        # 창을 이동거리 기준으로 자른다 (시간 기준이 아니다 — 파일 상단 참고)
        while len(self.win) > 2 and (self._arc() > self.arc_min * 1.5
                                     or (t - self.win[0][0]) > self.age_max):
            self.win.popleft()

        arc = self._arc()
        if len(self.win) < 3 or arc < self.arc_min:
            self._publish(None, None, None, 0.0, arc, valid=False)
            return

        # 창 중앙 시각의 명령과 비교 (지연만큼 당겨서)
        t_mid = 0.5 * (self.win[0][0] + self.win[-1][0])
        cmd = self._cmd_at(t_mid - self.lag)
        if cmd is None or cmd[3] != MODE_NORMAL:
            # crab/spin 은 STM32 고정 자세라 조향각-곡률 관계가 다르다 → 판정 제외
            self._publish(None, None, None, 0.0, arc, valid=False)
            return

        # 전/후진이 창 안에서 섞이면 부호가 엉킨다 → 제외
        signs = {1 if c[2] >= 0 else -1
                 for c in self.cmd_hist
                 if self.win[0][0] - self.lag <= c[0] <= self.win[-1][0] - self.lag}
        if len(signs) != 1:
            self._publish(None, None, None, 0.0, arc, valid=False)
            return
        direction = signs.pop()

        dtheta = wrap_pi(self.win[-1][3] - self.win[0][3])
        kappa = dtheta / (arc * direction)          # 부호 있는 호길이

        # κ ≈ 0 (완전 직진)이면 solve_inner_front_steer 가 |vx|/|wz| 에서 0으로 나눈다.
        # 직진은 관측기가 다룰 수 있는 정상 상태이므로 여기서 걸러 δ=0 으로 처리한다.
        if abs(kappa) < 1e-9:
            delta_obs = 0.0
        else:
            d, _clamped = fourwis_encode.solve_inner_front_steer(1.0, abs(kappa), self.wis)
            # 부호 규약을 encode 와 맞춘다: 좌회전(κ>0) → 음수 steer_deg
            delta_obs = -math.copysign(math.degrees(d), kappa)
        delta_cmd = cmd[1]
        gap = delta_obs - delta_cmd

        self._publish(delta_cmd, delta_obs, gap, kappa, arc, valid=True)
        self._judge(gap, t_mid)

    def _arc(self):
        """창 안의 누적 이동거리 [m]."""
        return sum(math.hypot(b[1] - a[1], b[2] - a[2])
                   for a, b in zip(self.win, list(self.win)[1:]))

    def _judge(self, gap, t_mid):
        """간극이 오래 지속되면 경고. 단 '지연 구간' 은 제외한다.

        δ_cmd 가 빠르게 변하는 중이면 간극의 대부분이 지연 탓이다. 그걸 편향으로
        읽고 보정하면 발진한다. |dδ_cmd/dt| 가 작은 구간에서만 판정한다.
        """
        rate = self._cmd_rate_at(t_mid - self.lag)
        if rate > self.bias_gate:
            self.gap_since = None
            return
        if abs(gap) < self.gap_warn:
            self.gap_since = None
            return
        if self.gap_since is None:
            self.gap_since = t_mid
            return
        if (t_mid - self.gap_since) >= self.gap_hold:
            self.get_logger().warn(
                f"조향 편향 의심: 정상 선회 구간에서 간극 {gap:+.1f}° 가 "
                f"{self.gap_hold:.0f} s 이상 지속 (명령 변화율 {rate:.1f} deg/s). "
                "max_steer_deg / rws_ratio / 기계 정렬을 확인하세요.",
                throttle_duration_sec=10.0)

    def _publish(self, delta_cmd, delta_obs, gap, kappa, arc, valid):
        """[명령δ, 관측δ, 간극, κ, Δs, 유효] — 전부 float32.

        유효=0 이면 앞의 값들은 의미 없습니다(창 미충족 / 정지 / normal 아님 /
        전후진 혼재). 관측 불가를 0 으로 채워 내보내면 그래프에서 '조향 0°' 로
        오독되므로 반드시 유효 플래그를 함께 보세요.
        """
        m = Float32MultiArray()
        m.data = [
            float(delta_cmd) if delta_cmd is not None else 0.0,
            float(delta_obs) if delta_obs is not None else 0.0,
            float(gap) if gap is not None else 0.0,
            float(kappa),
            float(arc),
            1.0 if valid else 0.0,
        ]
        self.pub.publish(m)


def main():
    rclpy.init()
    node = SteeringObserver()
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
