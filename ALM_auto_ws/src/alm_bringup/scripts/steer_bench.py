#!/usr/bin/env python3
"""steer_bench - 정지 상태 조향 슬루율 S_정지 를 재는 벤치 도구.

--- 왜 필요한가 -------------------------------------------------------------
STM32 업링크가 미구현이라(uart_protocol.md v2 §State) 실측 조향각이 올라오지
않습니다. 그래서 조향이 얼마나 빨리 도는지를 **자동으로 잴 방법이 없습니다.**
사람이 눈으로 보고 시간을 재는 수밖에 없고, 이 스크립트는 그 절차를 자동화합니다.

이 값이 필요한 이유는 하나입니다. 아래 두 파라미터가 **vx=0 구간**이라
'천천히 주행해서 조향이 따라오게 한다'는 전략이 통하지 않기 때문입니다.

    startup_steer_align_sec : 전원 투입 시 조향각을 모르므로 전 행정(60°)을 편다
    mode_switch_dwell_sec   : normal(±30°) <-> spin(47°) 전환 시 최악 77° 스윕

--- 어떻게 재는가 -----------------------------------------------------------
    1. 평평한 바닥에 **내려놓는다**.  ★ 잭업 금지
       정지 상태 조향은 접지면 전체를 비트는 것이라 가장 느립니다. 무부하로 재면
       실제보다 훨씬 빠르게 나오고, 그 값으로 dwell 을 정하면 조향이 안 펴진 채
       출발합니다.
    2. 앞바퀴에 테이프로 눈금을 붙인다 (어디까지 돌았는지 눈으로 봐야 함).
    3. 이 스크립트를 돌리고 화면 안내대로 Enter 를 친다.

★ 손으로 돌려보는 것은 이 측정이 아닙니다. 조향 '모터'가 접지 마찰을 이기고
  돌리는 속도를 재는 것이므로, 반드시 명령을 실어서 모터가 돌리게 해야 합니다.

--- 주의 -------------------------------------------------------------------
`/mcu/command` 로 **직접** 쏘므로 command_manager 와 충돌합니다.
반드시 command_manager 를 **끄고** mcu_bridge 만 띄운 상태에서 쓰세요.
구동은 항상 speed_rpm=0 입니다 (바퀴는 굴리지 않습니다).
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node

from alm_msgs.msg import McuCommand


class SteerBench(Node):
    def __init__(self, topic, rate_hz):
        super().__init__("steer_bench")
        self.pub = self.create_publisher(McuCommand, topic, 10)
        self.seq = 0
        self.target = 0.0
        self.period = 1.0 / max(rate_hz, 1.0)
        # STM32 는 마지막 명령을 유지하지 않을 수 있으므로 계속 재전송한다.
        self.timer = self.create_timer(self.period, self._send)

    def _send(self):
        m = McuCommand()
        m.stamp = self.get_clock().now().to_msg()
        self.seq += 1
        m.sequence = self.seq
        m.drive_mode = "normal"
        m.enable_motors = True
        m.emergency_stop = False
        m.steer_deg = float(self.target)
        m.speed_rpm = 0.0            # ★ 바퀴는 절대 굴리지 않는다
        m.mode_id = 1                # 1 = 일반 조향
        self.pub.publish(m)

    def hold(self, deg, seconds):
        self.target = float(deg)
        t0 = time.time()
        while rclpy.ok() and (time.time() - t0) < seconds:
            rclpy.spin_once(self, timeout_sec=0.05)

    def command(self, deg):
        self.target = float(deg)
        rclpy.spin_once(self, timeout_sec=0.05)


def ask(prompt):
    try:
        input(prompt)
        return True
    except (EOFError, KeyboardInterrupt):
        return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--amp", type=float, default=30.0,
                    help="한쪽 최대 조향각 [deg] (기본 30 = max_steer_deg)")
    ap.add_argument("--settle", type=float, default=4.0,
                    help="시작 자세로 갈 때까지 기다릴 시간 [s]")
    ap.add_argument("--repeats", type=int, default=3, help="반복 횟수")
    ap.add_argument("--topic", default="/mcu/command")
    ap.add_argument("--rate", type=float, default=50.0)
    a = ap.parse_args()

    span = 2.0 * a.amp
    print(f"""
==============================================================
 S_정지 (정지 상태 조향 슬루율) 측정
==============================================================
 전 행정 {span:.0f}°  ({-a.amp:+.0f}° -> {a.amp:+.0f}°),  {a.repeats} 회 반복

 ★ 확인:  · 평평한 바닥에 내려놨는가 (잭업 금지)
          · 앞바퀴에 눈금 테이프를 붙였는가
          · command_manager 를 껐는가 (안 끄면 명령이 충돌한다)
          · 주변에 사람이 없는가

 구동 명령은 항상 0 rpm 입니다 — 바퀴는 굴러가지 않습니다.
==============================================================""")
    if not ask("\n준비되면 Enter (중단하려면 Ctrl-C): "):
        return 1

    rclpy.init()
    n = SteerBench(a.topic, a.rate)
    times = []
    try:
        for i in range(1, a.repeats + 1):
            print(f"\n--- {i}/{a.repeats} 회 ---")
            print(f"  시작 자세 {-a.amp:+.0f}° 로 이동 중... ({a.settle:.0f} s)")
            n.hold(-a.amp, a.settle)
            if not ask(f"  바퀴가 {-a.amp:+.0f}° 에서 **멈췄으면** Enter → 바로 반대로 돕니다: "):
                break

            n.command(+a.amp)
            t0 = time.perf_counter()
            print(f"  ▶ {a.amp:+.0f}° 명령 전송. 바퀴가 **멈추는 순간** Enter!")
            # Enter 를 기다리는 동안에도 재전송이 계속돼야 한다
            import select
            while rclpy.ok():
                rclpy.spin_once(n, timeout_sec=0.02)
                if select.select([sys.stdin], [], [], 0.0)[0]:
                    sys.stdin.readline()
                    break
            dt = time.perf_counter() - t0
            if dt < 0.05:
                print("  ⚠ 너무 빨리 눌렸습니다 — 이 회차는 버립니다.")
                continue
            s = span / dt
            times.append(s)
            print(f"  → {dt:.2f} s 만에 {span:.0f}°  =  S_정지 ≈ {s:.1f} deg/s")

        print("\n  조향을 0° 로 되돌립니다...")
        n.hold(0.0, a.settle)
    except KeyboardInterrupt:
        pass
    finally:
        n.command(0.0)
        for _ in range(10):
            rclpy.spin_once(n, timeout_sec=0.02)
        n.destroy_node()
        rclpy.shutdown()

    if not times:
        print("\n측정값 없음.")
        return 1

    times.sort()
    s_min, s_med = times[0], times[len(times) // 2]
    print(f"""
==============================================================
 결과:  {'  '.join(f'{v:.1f}' for v in times)} deg/s
        중앙값 {s_med:.1f} deg/s,  **최솟값 {s_min:.1f} deg/s**

 ★ 파라미터는 **최솟값** 기준으로 정하세요 (보수적으로).
==============================================================

 이 값으로 확인할 것 셋 — base_control.yaml
""")
    startup_need = 60.0 / s_min
    dwell_need = 77.0 / s_min
    crab_need = 120.0 / s_min
    def verdict(need, cur):
        return "OK  충분" if cur >= need else f"부족! {need:.1f} s 이상으로 올릴 것"
    print(f"  startup_steer_align_sec  현재 5.0 s : 전 행정 60° 에 {startup_need:5.1f} s 필요"
          f"  -> {verdict(startup_need, 5.0)}")
    print(f"  mode_switch_dwell_sec    현재 4.0 s : normal<->spin 77° 에 {dwell_need:5.1f} s 필요"
          f"  -> {verdict(dwell_need, 4.0)}")
    print(f"  (crab 을 쓸 경우)                    : normal<->crab 120° 에 {crab_need:5.1f} s 필요")
    print(f"""
  max_steer_rate_deg_s  현재 20.0 (가정값)
      주행 중 슬루율은 정지보다 **빠릅니다**. S_정지 {s_min:.1f} 만 넘으면
      20 은 안전합니다. 성능을 되찾으려면 max_linear_x 와 함께
      비(100 deg/m)를 유지한 채 올리세요 — docs/control_pipeline.md §6.3.

  ★ 조향이 **아예 안 돌거나 스톨**했다면 그게 가장 중요한 발견입니다.
    그러면 '정지 중 조향 금지 = 굴리면서만 조향' 이라는 설계 제약이 생기고,
    dwell 로는 못 메웁니다. 그 경우를 만나면 알려주세요 — 설계를 바꿔야 합니다.
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
