#!/usr/bin/env python3
"""웹 수동주행(직접 rpm/조향각) 경로 시험 — 하드웨어 없이 전 구간을 돈다.

    ros2 run alm_base_control cmd_arbiter.py &
    ros2 run alm_base_control command_manager.py --ros-args \\
        --params-file <alm_base_control>/config/base_control.yaml &
    python3 direct_drive_test.py

확인하는 것은 '값이 그대로 나가는가' 가 아니라 **안전 게이트가 직접 경로에도
그대로 걸리는가** 다. 직접 명령은 twist 변환을 건너뛰므로, 게이트까지 같이
건너뛰면 그 순간 이 경로는 안전 계층이 없는 원격 액추에이터 조작이 된다.
"""
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import Bool, String
from alm_msgs.msg import DirectDrive, McuCommand
from alm_msgs.srv import ReleaseEstop, SetControlOwner


class Probe(Node):
    def __init__(self):
        super().__init__("direct_drive_test")
        self.lock = threading.Lock()
        self.last = None
        self.owner = ""
        self.create_subscription(McuCommand, "/mcu/command", self._on_cmd, 10)
        self.create_subscription(
            String, "/cmd_arbiter/owner", self._on_owner,
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.direct_pub = self.create_publisher(DirectDrive, "/direct_drive_web", 10)
        self.estop_pub = self.create_publisher(Bool, "/emergency_stop", QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL))
        self.owner_cli = self.create_client(SetControlOwner, "/cmd_arbiter/set_owner")
        self.release_cli = self.create_client(ReleaseEstop, "/emergency_stop/release")

    def _on_cmd(self, msg):
        with self.lock:
            self.last = msg

    def _on_owner(self, msg):
        self.owner = msg.data

    def latest(self):
        with self.lock:
            return self.last

    def set_owner(self, owner):
        request = SetControlOwner.Request()
        request.owner = owner
        future = self.owner_cli.call_async(request)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        return future.result()

    def release_estop(self):
        request = ReleaseEstop.Request()
        request.reason = "test"
        future = self.release_cli.call_async(request)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not future.done():
            time.sleep(0.02)
        return future.result()

    def send(self, rpm, steer, mode=1, duration=1.0, hz=20.0):
        """직접 명령을 duration 동안 계속 낸다 (하트비트가 있어야 살아 있다)."""
        end = time.monotonic() + duration
        while time.monotonic() < end:
            msg = DirectDrive()
            msg.stamp = self.get_clock().now().to_msg()
            msg.speed_rpm = float(rpm)
            msg.steer_deg = float(steer)
            msg.mode_id = int(mode)
            self.direct_pub.publish(msg)
            time.sleep(1.0 / hz)


fails = []


def check(label, ok, detail=""):
    print(f"  [{'OK ' if ok else 'FAIL'}] {label}{(' — ' + str(detail)) if detail else ''}")
    if not ok:
        fails.append(label)


def main():
    rclpy.init()
    probe = Probe()
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
    executor.add_node(probe)
    threading.Thread(target=executor.spin, daemon=True).start()

    for client, name in ((probe.owner_cli, "set_owner"), (probe.release_cli, "estop/release")):
        if not client.wait_for_service(timeout_sec=10.0):
            print(f"'{name}' 서비스가 없습니다 — cmd_arbiter / command_manager 를 먼저 띄우세요.")
            return 2

    print("\n① 동작권 없이는 직접명령이 안 나간다")
    probe.set_owner("auto")
    time.sleep(0.5)
    probe.send(500, 10, duration=1.0)
    time.sleep(0.3)
    latest = probe.latest()
    check("auto 소유 중 직접명령 무시", latest is None or latest.drive_mode != "direct",
          f"drive_mode={getattr(latest, 'drive_mode', None)}")

    print("\n② web 동작권에서 rpm/조향각이 그대로 나간다")
    response = probe.set_owner("web")
    check("set_owner('web') 수락", response is not None and response.success,
          getattr(response, "message", "무응답"))
    # 기동 정렬 dwell 이 끝나기를 기다린다 (startup_steer_align_sec)
    probe.send(0, 0, duration=6.0)
    probe.send(600, 20, duration=3.0)
    latest = probe.latest()
    check("mode_id=1 전달", latest.mode_id == 1, latest.mode_id)
    check("rpm 이 명령값에 수렴", abs(latest.speed_rpm - 600) < 30, f"{latest.speed_rpm:.0f} rpm")
    check("조향각이 명령값에 수렴", abs(latest.steer_deg - 20) < 1.0, f"{latest.steer_deg:.1f}°")
    check("drive_mode=direct", latest.drive_mode == "direct", latest.drive_mode)
    check("cmd_vel 은 비어 있다 (rpm->m/s 환산이 미확정)",
          abs(latest.cmd_vel.linear.x) < 1e-9, latest.cmd_vel.linear.x)

    print("\n③ 기구 한계로 잘린다")
    probe.send(99999, 88, duration=3.0)
    latest = probe.latest()
    check("rpm 이 max_rpm 으로 클램프", latest.speed_rpm <= 3000.0 + 1, f"{latest.speed_rpm:.0f}")
    check("조향각이 max_steer_deg 로 클램프", latest.steer_deg <= 30.0 + 0.1, f"{latest.steer_deg:.1f}°")

    print("\n④ 조향 슬루 제한이 직접 경로에도 걸린다")
    probe.send(0, 30, duration=3.0)          # 정지 상태에서 한쪽 끝
    before = probe.latest().steer_deg
    probe.send(0, -30, duration=0.4)         # 반대쪽으로 스텝 입력
    after = probe.latest().steer_deg
    moved = abs(after - before)
    # S_정지 17.5 deg/s * 0.4 s ≈ 7° 이하여야 한다 (스텝 60° 가 그대로 나가면 실패)
    check("스텝 입력이 슬루로 깎인다", moved < 15.0,
          f"0.4 s 동안 {moved:.1f}° (제한 없으면 60°)")

    print("\n⑤ 하트비트가 끊기면 rpm 0 (HELD)")
    probe.send(500, 0, duration=2.0)
    # 끊김 판정(web_timeout 0.5 s) + 감속 램프(500 rpm / direct_rpm_decel)를 기다린다.
    # 램프가 있는 것이 맞다 — 통신 끊김은 위험 이벤트가 아니라서 twist 경로도
    # soft_stop 으로 세운다. 즉시 정지는 E-STOP 의 몫이고 아래 ⑥ 에서 본다.
    time.sleep(2.5)
    latest = probe.latest()
    check("명령 끊김 -> rpm 0", abs(latest.speed_rpm) < 1e-6, f"{latest.speed_rpm:.1f}")
    check("owner 가 web(held)", "held" in probe.owner, probe.owner)

    print("\n⑥ E-STOP 이 직접 경로도 즉시 세운다")
    threading.Thread(target=probe.send, args=(800, 0), kwargs={"duration": 4.0},
                     daemon=True).start()
    time.sleep(1.5)
    probe.estop_pub.publish(Bool(data=True))
    time.sleep(0.5)
    latest = probe.latest()
    check("E-STOP -> rpm 0", abs(latest.speed_rpm) < 1e-6, f"{latest.speed_rpm:.1f}")
    check("E-STOP -> 모터 비활성", not latest.enable_motors)
    check("E-STOP -> mode 0", latest.mode_id == 0, latest.mode_id)
    check("emergency_stop 플래그", latest.emergency_stop)
    time.sleep(3.0)
    response = probe.release_estop()
    check("E-STOP 해제", response is not None and response.success,
          getattr(response, "message", "무응답"))

    probe.set_owner("auto")
    print(f"\n{'전부 통과' if not fails else '실패: ' + ', '.join(fails)}")
    rclpy.try_shutdown()
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
