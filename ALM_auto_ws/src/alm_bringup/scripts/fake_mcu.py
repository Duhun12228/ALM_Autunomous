#!/usr/bin/env python3
"""fake_mcu — mcu_bridge 대역. **개발 전용. 실차에서 절대 함께 띄우지 말 것.**

STM32 가 물려 있지 않은 상태에서 WebUI 연동을 검증하려고 만든 노드다.
mcu_bridge 와 같은 인터페이스(/mcu/command 구독, /mcu/state + /wheel_odom 발행)를
흉내내되 시리얼 포트는 열지 않는다. 그래서 mcu_bridge 와 동시에 띄우면
같은 토픽에 두 퍼블리셔가 생겨 상태가 뒤섞인다 — launch 에서 배타적으로 고를 것.

거동:
  - /mcu/command 를 받아 그대로 되먹인다(measured = commanded × 약간의 손실).
    command_manager → 이 노드 → /mcu/state → command_manager 의 fault 감시까지
    한 바퀴가 실제로 돈다.
  - cmd 가 command_timeout_sec 넘게 없으면 command_timeout=True 로 보고하고 정지.
  - 배터리는 천천히 닳고, fault_code 는 파라미터로 주입해 UI 경보 표시를 시험한다.
  - steer_angle 은 실물과 같이 축당 1개씩 2개([front, rear])만 채운다.

--- 조향 액추에이터 모델 (steer_rate_deg_s / steer_tau_sec) --------------------
예전 이 노드는 조향 명령을 **지연 없이 그대로 되돌려줬다**. 그래서
  · 명령 대비 실제 조향 간극
  · 출발 시 조향 풀락(0°→30° 를 20 ms 에 요구하는 문제)
  · 조향 슬루 제한이 실제로 효과가 있는지
를 시뮬레이터에서 **재현할 수 없었다.** 실차 없이는 대책을 검증할 방법이 없었다는 뜻이다.

이제 조향축을 '슬루 제한 + 1차 지연' 으로 모델링한다:

    δ_act ←(슬루 max steer_rate_deg_s)← δ_cmd,  그 뒤 1차 지연 τ = steer_tau_sec

그리고 **데드레커닝을 명령 wz 가 아니라 δ_act 로 계산한다.** 즉 조향이 못 따라가면
로봇이 실제로 경로에서 벗어난다 — 그게 이 모델의 요점이다.

기본값은 실차 미측정(##CONFIRM##) 이므로 '적당히 느린' 값을 넣어 뒀다. 실측 전에는
이 값을 바꿔가며 **민감도**를 보는 용도로 쓸 것: 30/60/120/400 deg/s 를 넣어보면
"슬루율이 이 정도면 이런 증상" 을 미리 알 수 있다.
  steer_rate_deg_s = 0  → 예전 거동(즉시 반영)으로 되돌린다.

--- 업링크 없는 실차 흉내 (publish_state) ------------------------------------
uart_protocol.md v2 기준 STM32 업링크는 **미구현**이다. 실차에서는 /mcu/state 가
아예 오지 않는다. publish_state:=false 로 두면 그 조건을 그대로 재현해서,
업링크에 의존하는 코드가 조용히 망가지지 않는지 확인할 수 있다.
"""

import math
import os
import sys

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry

from alm_msgs.msg import McuCommand, McuState


def _import_fourwis():
    """command_manager 와 '같은' 기구학 모듈을 쓴다 — 복사하면 그 순간 갈라진다.

    alm_base_control 의 설치본 lib 를 먼저 보고, 없으면 소스 트리에서 찾는다
    (nav2_kinematic_check.py 와 같은 전략). 못 찾으면 None 을 돌려주고
    호출부가 '조향 모델 없음'으로 물러난다 — 더미 노드가 임포트 실패로
    죽는 것보다 낫다.
    """
    cands = []
    try:
        from ament_index_python.packages import get_package_prefix
        cands.append(os.path.join(
            get_package_prefix("alm_base_control"), "lib", "alm_base_control"))
    except Exception:
        pass
    #  <ws>/src/alm_bringup/scripts/fake_mcu.py -> <ws>/src
    src = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    cands.append(os.path.join(src, "alm_base_control", "scripts"))
    for d in cands:
        if os.path.exists(os.path.join(d, "fourwis_encode.py")):
            sys.path.insert(0, d)
            try:
                import fourwis_encode as mod
                return mod
            except Exception:
                return None
    return None


fourwis_encode = _import_fourwis()


class FakeMcu(Node):
    def __init__(self):
        super().__init__("fake_mcu")
        g = self.declare_parameter
        g("command_topic", "/mcu/command")
        g("state_topic", "/mcu/state")
        g("odom_topic", "/wheel_odom")
        g("publish_rate_hz", 10.0)
        g("command_timeout_sec", 0.5)
        g("battery_full_voltage", 25.2)
        g("battery_empty_voltage", 21.0)
        g("battery_drain_per_min", 0.4)     # [%]
        g("wheel_radius_m", 0.103)
        g("rws_ratio", 0.5)                 # ALM07.slx CONS(1). 후륜 조향각 되비추기용
        g("wheelbase_m", 1.0)               # CONS(2). δ_act -> wz 계산용
        g("track_m", 0.919)                 # CONS(3)
        g("max_steer_deg", 30.0)
        # ---- 조향 액추에이터 모델 (##CONFIRM## 실차 미측정) ----
        # 0 이면 예전 거동(즉시 반영). 실측 전에는 값을 바꿔가며 민감도를 볼 것.
        g("steer_rate_deg_s", 60.0)         # 슬루 제한 [deg/s]
        g("steer_tau_sec", 0.08)            # 1차 지연 시상수 [s]
        # 실차처럼 업링크를 아예 안 내보내는 모드 (uart_protocol.md v2 §State 미구현)
        g("publish_state", True)
        g("inject_fault_code", 0)           # 0 = 정상. UI 경보 시험용
        g("inject_fault_text", "")
        # MCU '자신의' E-STOP 상태(물리 버튼/내부 래치). 명령으로 받은 estop 을
        # 되비추면 안 된다 — command_manager 는 /mcu/state 의 estop 을 fault 로
        # 보고 mcu_fault 를 세우고(_on_mcu_state), 그 mcu_fault 가 다시
        # McuCommand.emergency_stop 을 켠다. 되비추는 순간 서로가 서로를
        # 유지시켜 /emergency_stop 을 false 로 내려도 영원히 안 풀린다.
        g("inject_estop", False)            # UI 안전 스트립 시험용

        p = self.get_parameter
        self.timeout = float(p("command_timeout_sec").value)
        self.wheel_radius = float(p("wheel_radius_m").value)
        self.rws_ratio = float(p("rws_ratio").value)
        self.v_full = float(p("battery_full_voltage").value)
        self.v_empty = float(p("battery_empty_voltage").value)
        self.drain = float(p("battery_drain_per_min").value)

        self.state_pub = self.create_publisher(McuState, p("state_topic").value, 10)
        self.odom_pub = self.create_publisher(Odometry, p("odom_topic").value, 10)
        self.create_subscription(
            McuCommand, p("command_topic").value, self.on_command, 10)

        self.cmd = None
        self.cmd_stamp = None
        self.sequence = 0
        self.battery_pct = 100.0
        self.pose = [0.0, 0.0, 0.0]  # x, y, yaw
        self.last_tick = self.get_clock().now()

        # ---- 조향 액추에이터 상태 ----
        self.steer_rate = float(p("steer_rate_deg_s").value)
        self.steer_tau = max(0.0, float(p("steer_tau_sec").value))
        self.steer_act_deg = 0.0            # 모델링된 '실제' 조향각
        self.publish_state = bool(p("publish_state").value)
        self.wis = None
        if fourwis_encode is not None:
            self.wis = fourwis_encode.FourWISParams(
                wheelbase_m=float(p("wheelbase_m").value),
                track_m=float(p("track_m").value),
                rws_ratio=self.rws_ratio,
                wheel_radius_m=self.wheel_radius,
                max_steer_deg=float(p("max_steer_deg").value),
            )

        rate = max(1.0, float(p("publish_rate_hz").value))
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().warn(
            "fake_mcu 기동 — 개발 전용 더미입니다. "
            "mcu_bridge 와 동시에 실행하지 마세요.")
        if self.wis is None:
            self.get_logger().warn(
                "fourwis_encode 를 못 찾아 조향 액추에이터 모델을 비활성화합니다 "
                "(명령 조향각이 즉시 반영되는 예전 거동). alm_base_control 빌드 확인.")
        elif self.steer_rate > 0.0:
            self.get_logger().info(
                f"조향 액추에이터 모델: 슬루 {self.steer_rate:.0f} deg/s, "
                f"지연 τ={self.steer_tau:.3f} s. "
                f"##CONFIRM## 실차 미측정 — 민감도 확인용 값입니다.")
        else:
            self.get_logger().warn(
                "steer_rate_deg_s=0 — 조향이 즉시 반영됩니다(예전 거동). "
                "조향 지연 관련 문제는 이 설정으로는 재현되지 않습니다.")
        if not self.publish_state:
            self.get_logger().warn(
                "publish_state=false — /mcu/state 를 발행하지 않습니다. "
                "STM32 업링크 미구현(실차) 조건을 그대로 재현합니다.")

    def on_command(self, msg):
        self.cmd = msg
        self.cmd_stamp = self.get_clock().now()

    def _command_fresh(self):
        if self.cmd is None or self.cmd_stamp is None:
            return False
        age = (self.get_clock().now() - self.cmd_stamp).nanoseconds / 1e9
        return age <= self.timeout

    def _step_steer(self, target_deg, dt):
        """조향축 1스텝 진행: 슬루 제한 -> 1차 지연."""
        if self.steer_rate <= 0.0 or dt <= 0.0:
            return float(target_deg)
        cur = self.steer_act_deg
        step = self.steer_rate * dt
        cur += max(-step, min(float(target_deg) - cur, step))
        if self.steer_tau > 1e-6:
            # 1차 지연을 슬루 뒤에 얹는다. dt/τ 가 1 을 넘지 않게 클램프해서
            # 큰 dt 에서 발산하지 않도록 한다.
            cur += (float(target_deg) - cur) * min(1.0, dt / self.steer_tau) * 0.5
        return cur

    def _wz_from_actual_steer(self, vx, wz_cmd):
        """실제 조향각에서 나오는 요레이트. 모델이 없으면 명령값으로 물러난다.

        spin/crab 은 STM32 가 고정 자세를 쓰므로 조향각-곡률 관계가 다르다.
        여기서는 mode_id 로 구분해 normal 에서만 조향각 기반으로 계산하고,
        나머지는 기존처럼 명령 wz 를 약간 깎아서 쓴다.
        """
        mode_id = int(self.cmd.mode_id) if self.cmd is not None else 1
        if self.wis is None or mode_id != 1:
            return wz_cmd * 0.94
        return fourwis_encode.wz_from_steer(self.steer_act_deg, vx, self.wis)

    def tick(self):
        now = self.get_clock().now()
        dt = (now - self.last_tick).nanoseconds / 1e9
        self.last_tick = now
        fresh = self._command_fresh()
        own_estop = bool(self.get_parameter("inject_estop").value)

        # 명령이 신선할 때만 움직인다. 실제 MCU 의 cmd timeout 거동과 같다.
        if fresh and not own_estop and not self.cmd.emergency_stop and self.cmd.enable_motors:
            vx = self.cmd.cmd_vel.linear.x
            vy = self.cmd.cmd_vel.linear.y
            wz = self.cmd.cmd_vel.angular.z
            steer_deg = self.cmd.steer_deg
            speed_rpm = self.cmd.speed_rpm
        else:
            vx = vy = wz = steer_deg = speed_rpm = 0.0

        # ---- 조향 액추에이터: 슬루 제한 + 1차 지연 ----
        # 명령을 지연 없이 되돌리면 조향 문제가 시뮬레이터에서 사라진다(파일 상단 참고).
        self.steer_act_deg = self._step_steer(steer_deg, dt)

        # 실측은 지령을 약간 못 따라간다 (구동 손실 흉내)
        measured_vx = vx * 0.96
        measured_vy = vy * 0.96
        # ★ 요레이트는 '명령 wz' 가 아니라 '실제 조향각' 에서 나온다.
        #   조향이 못 따라가면 로봇이 실제로 경로를 벗어나야 한다 — 그래야
        #   조향 슬루 제한 같은 대책의 효과를 시뮬레이터에서 볼 수 있다.
        measured_wz = self._wz_from_actual_steer(measured_vx, wz)

        # 데드레커닝으로 /wheel_odom 을 만든다
        self.pose[2] += measured_wz * dt
        self.pose[0] += (measured_vx * math.cos(self.pose[2])
                         - measured_vy * math.sin(self.pose[2])) * dt
        self.pose[1] += (measured_vx * math.sin(self.pose[2])
                         + measured_vy * math.cos(self.pose[2])) * dt

        if abs(measured_vx) > 1e-3 or abs(measured_wz) > 1e-3:
            self.battery_pct = max(0.0, self.battery_pct - self.drain * dt / 60.0)

        self.sequence += 1
        fault_code = int(self.get_parameter("inject_fault_code").value)

        state = McuState()
        state.stamp = now.to_msg()
        state.sequence = self.sequence
        state.measured_velocity.linear.x = measured_vx
        state.measured_velocity.linear.y = measured_vy
        state.measured_velocity.angular.z = measured_wz
        state.odom_pose.x = self.pose[0]
        state.odom_pose.y = self.pose[1]
        state.odom_pose.theta = self.pose[2]

        # 조향은 축당 1개 — [front, rear]. 후륜은 전륜의 rws_ratio 배로 역방향 조향.
        # ★ 명령이 아니라 '모델링된 실제 각도' 를 보고한다. 실물 STM32 가 엔코더를
        #   올려주게 되면 이 자리에 그 값이 들어온다.
        state.steer_angle = [math.radians(self.steer_act_deg),
                             -math.radians(self.steer_act_deg) * self.rws_ratio]

        # 바퀴 각속도 [rad/s]. 회전 성분은 좌우를 반대로 벌린다.
        base = measured_vx / self.wheel_radius if self.wheel_radius else 0.0
        differential = measured_wz * 0.5 / self.wheel_radius if self.wheel_radius else 0.0
        state.wheel_speed = [
            base - differential,  # front_left
            base + differential,  # front_right
            base - differential,  # rear_left
            base + differential,  # rear_right
        ]

        ratio = self.battery_pct / 100.0
        state.battery_voltage = self.v_empty + (self.v_full - self.v_empty) * ratio
        state.battery_current = 1.2 + abs(speed_rpm) / 3000.0 * 6.0

        commanded_estop = bool(self.cmd.emergency_stop) if self.cmd else False
        state.motors_enabled = bool(
            fresh and self.cmd and self.cmd.enable_motors
            and not commanded_estop and not own_estop)
        # 보고하는 것은 '내 estop 상태'이지 받은 명령이 아니다 (위 파라미터 주석 참고)
        state.emergency_stop = own_estop
        state.command_timeout = not fresh
        state.fault = fault_code != 0
        state.fault_code = fault_code
        state.fault_text = str(self.get_parameter("inject_fault_text").value)
        # publish_state=false 면 실차(업링크 미구현)와 같이 /mcu/state 를 안 올린다.
        #
        # ⚠ /wheel_odom 은 **막지 않는다.** 실물에서는 State 프레임에서 파생되지만,
        #   시뮬레이션에서는 이게 '세계의 진짜 위치'(sim_world 가 소비해 TF/Odometry 를
        #   만드는 입력)라서, 여기서 막으면 로봇이 아예 안 움직이게 된다.
        #   실차의 위치 피드백은 MCU 가 아니라 **FAST-LIO** 가 준다는 점을 생각하면
        #   /wheel_odom 을 계속 내보내는 쪽이 오히려 실차에 가깝다.
        #   (앞서 이걸 함께 막았다가 '명령은 나가는데 로봇이 안 움직임' 을 만들었다)
        if self.publish_state:
            self.state_pub.publish(state)

        odom = Odometry()
        odom.header.stamp = state.stamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.orientation = yaw_to_quaternion(self.pose[2])
        odom.twist.twist = state.measured_velocity
        self.odom_pub.publish(odom)


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


def main():
    rclpy.init()
    node = FakeMcu()
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
