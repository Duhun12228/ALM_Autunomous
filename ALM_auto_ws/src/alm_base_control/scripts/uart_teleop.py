#!/usr/bin/env python3
"""uart_teleop - Jetson에서 STM32로 UART 명령을 직접 쏘는 수동 조작/점검 도구.

ROS 를 띄우지 않고 단독 실행됩니다. 제어단과 마주앉아 "지금 이 값 보냈다"를
바로 확인하는 용도입니다. 보내는 바이트는 mcu_bridge 가 만드는 것과 동일합니다
(fourwis_encode.build_command_frame 공유, uart_protocol.md v2).

사용법:
  # 1) 키보드 수동 조작 (WASD). 자율주행 없이 twist -> 4WIS 변환까지 그대로 탐
  python3 uart_teleop.py --port /dev/ttyTHS1

  # 2) 값을 직접 지정 ("조향 20도, 300rpm, 일반주행 줘봐")
  python3 uart_teleop.py --port /dev/ttyTHS1 --mode direct

  # 3) 정해진 시퀀스 자동 송신 (제어단이 각 모드 반응을 관찰)
  python3 uart_teleop.py --port /dev/ttyTHS1 --mode sequence

  # 4) 배선 전 프레임만 확인 (시리얼 안 엶)
  python3 uart_teleop.py --dry-run --mode sequence

키 (teleop):
  w/s  전진/후진      a/d  좌/우 회전      q/e  좌/우 게걸음(crab)
  space  즉시 정지(mode 0)                 z    속도 0 으로 리셋
  1/3/4  모드 고정: 1=일반 3=크랩 4=제로턴  0  자동선택(twist 기준)
  x 또는 Ctrl-C  종료 (종료 시 정지 프레임 전송)

⚠ 안전: 바퀴가 실제로 돕니다. 차량을 들어올리거나(잭업) 바퀴를 띄운 상태에서
   먼저 확인하세요. 종료·정지 시 mode 0 프레임을 반복 전송합니다.
"""

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import fourwis_encode as fw

try:
    import serial
except ImportError:
    serial = None

MODE_NAMES = {0: "정지", 1: "일반주행", 2: "자가추종(정지)", 3: "크랩", 4: "제로턴"}


class Link:
    """시리얼 송신 + 수신 바이트 확인. --dry-run 이면 화면 출력만."""

    def __init__(self, port, baud, dry_run):
        self.dry_run = dry_run
        self.ser = None
        self.tx_count = 0
        self.rx = bytearray()
        if dry_run:
            return
        if serial is None:
            sys.exit("pyserial 이 없습니다:  pip3 install pyserial")
        try:
            self.ser = serial.Serial(port, baud, timeout=0)
        except Exception as e:                       # noqa: BLE001
            sys.exit(f"시리얼 열기 실패 ({port}): {e}\n"
                     f"  - 권한:  sudo usermod -aG dialout $USER  (재로그인)\n"
                     f"  - Jetson THS1 이 시리얼 콘솔에 물려 있으면:\n"
                     f"      sudo systemctl disable --now nvgetty")

    def send(self, frame):
        self.tx_count += 1
        if self.ser is not None:
            self.ser.write(frame)

    def poll_rx(self):
        """STM32 가 뭔가 보내면 수집(현재 업링크는 미구현이라 보통 비어 있음)."""
        if self.ser is None:
            return b""
        try:
            n = self.ser.in_waiting
        except Exception:                            # noqa: BLE001
            return b""
        return self.ser.read(n) if n else b""

    def close(self):
        if self.ser is not None:
            self.ser.close()


def status_line(vx, vy, wz, steer, rpm, mode, frame, note):
    txt = (f"vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f} | "
           f"steer={steer:+7.2f}° rpm={rpm:+8.2f} mode={mode}({MODE_NAMES.get(mode, '?')}) | "
           f"{frame.hex(' ').upper()}")
    if note:
        txt += f"\n    ⚠ {note}"
    return txt


def send_stop(link, n=5):
    frame = fw.build_command_frame(0.0, 0.0, fw.MODE_STOP, 0x00)
    for _ in range(n):
        link.send(frame)
        time.sleep(0.02)


# ------------------------------------------------------------------ teleop
def run_teleop(link, params, args):
    if not sys.stdin.isatty():
        sys.exit("teleop 은 터미널에서 실행해야 합니다 (--mode direct/sequence 사용)")

    vx = vy = wz = 0.0
    forced_mode = None          # None 이면 twist 로 자동 선택
    period = 1.0 / args.rate
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    print(__doc__.split("키 (teleop):")[1].split("⚠ 안전")[0])
    print(f"포트: {'DRY-RUN' if link.dry_run else args.port}  "
          f"{args.rate:.0f}Hz 송신 중...\n")
    try:
        tty.setcbreak(fd)
        while True:
            t0 = time.time()
            while select.select([sys.stdin], [], [], 0)[0]:
                c = sys.stdin.read(1)
                if c in ("x", "\x03"):
                    return
                elif c == "w":
                    vx = min(vx + args.step_v, args.max_vx)
                elif c == "s":
                    vx = max(vx - args.step_v, -args.max_vx)
                elif c == "a":
                    wz = min(wz + args.step_w, args.max_wz)
                elif c == "d":
                    wz = max(wz - args.step_w, -args.max_wz)
                elif c == "q":
                    vy = min(vy + args.step_v, args.max_vy)
                elif c == "e":
                    vy = max(vy - args.step_v, -args.max_vy)
                elif c == " ":
                    vx = vy = wz = 0.0
                    forced_mode = 0
                elif c == "z":
                    vx = vy = wz = 0.0
                elif c in ("1", "3", "4"):
                    forced_mode = int(c)
                elif c == "0":
                    forced_mode = None

            drive_mode, stopped = _resolve_mode(forced_mode, vx, vy, wz)
            steer, rpm, mode, note = fw.encode(vx, vy, wz, drive_mode, stopped, params)
            frame = fw.build_command_frame(steer, rpm, mode, 0x01)
            link.send(frame)

            rx = link.poll_rx()
            line = status_line(vx, vy, wz, steer, rpm, mode, frame, note)
            if rx:
                line += f"\n    RX {len(rx)}B: {rx[:16].hex(' ').upper()}"
            sys.stdout.write("\x1b[2K\r" + line.replace("\n", "\n\x1b[2K"))
            sys.stdout.flush()

            time.sleep(max(0.0, period - (time.time() - t0)))
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        send_stop(link)
        print(f"\n\n정지 프레임 전송 완료. 총 {link.tx_count} 프레임 송신.")


def _resolve_mode(forced, vx, vy, wz):
    """forced_mode -> (drive_mode 문자열, stopped)."""
    if forced == 0:
        return "normal", True
    if forced == 3:
        return "crab", False
    if forced == 4:
        return "spin", False
    if forced == 1:
        return "normal", False
    # 자동: 제자리 회전이면 spin, 측면이면 crab, 아니면 normal
    if abs(wz) > 1e-3 and abs(vx) < 0.02 and abs(vy) < 0.02:
        return "spin", False
    if abs(vy) > 1e-3 and abs(wz) < 1e-3:
        return "crab", False
    return "normal", False


# ------------------------------------------------------------------ direct
def run_direct(link, params, args):
    """steer_deg / rpm / mode 를 직접 입력. 제어단 요청값을 그대로 재현."""
    print("직접 입력 모드. 'steer rpm mode' 를 공백으로 구분해 입력 (예: 20 300 1)")
    print("빈 줄 = 정지 프레임, 'q' = 종료. 입력한 값을 지정한 시간만큼 반복 송신합니다.\n")
    while True:
        try:
            line = input("steer rpm mode > ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if line.lower() in ("q", "quit", "exit"):
            break
        if not line:
            send_stop(link)
            print("  정지 프레임 전송")
            continue
        try:
            parts = line.split()
            steer = float(parts[0])
            rpm = float(parts[1]) if len(parts) > 1 else 0.0
            mode = int(parts[2]) if len(parts) > 2 else 1
        except (ValueError, IndexError):
            print("  형식 오류. 예: 20 300 1")
            continue

        frame = fw.build_command_frame(steer, rpm, mode, 0x01)
        print(f"  전송: steer={steer:+.2f}° rpm={rpm:+.2f} "
              f"mode={mode}({MODE_NAMES.get(mode, '?')})")
        print(f"  프레임: {frame.hex(' ').upper()}")
        if mode == 1 and steer > 0:
            print("  참고: mode 1 은 STM32 에서 부호가 반전됩니다 -> 양수 = 우회전")

        deadline = time.time() + args.hold
        while time.time() < deadline:
            link.send(frame)
            time.sleep(1.0 / args.rate)
        rx = link.poll_rx()
        if rx:
            print(f"  RX {len(rx)}B: {rx[:32].hex(' ').upper()}")
    send_stop(link)
    print(f"\n정지 프레임 전송 완료. 총 {link.tx_count} 프레임 송신.")


# ---------------------------------------------------------------- sequence
def run_sequence(link, params, args):
    """정해진 순서로 각 모드를 송신. 제어단이 바퀴 반응을 눈으로 확인."""
    steps = [
        ("정지",              0.0,  0.0,  0.0, "normal", True),
        ("직진 0.2 m/s",      0.2,  0.0,  0.0, "normal", False),
        ("좌회전 (steer 음수)", 0.2,  0.0,  0.15, "normal", False),
        ("우회전 (steer 양수)", 0.2,  0.0, -0.15, "normal", False),
        ("정지",              0.0,  0.0,  0.0, "normal", True),
        ("제로턴 좌",          0.0,  0.0,  0.3, "spin", False),
        ("제로턴 우",          0.0,  0.0, -0.3, "spin", False),
        ("크랩 좌",            0.0,  0.2,  0.0, "crab", False),
        ("크랩 우",            0.0, -0.2,  0.0, "crab", False),
        ("정지",              0.0,  0.0,  0.0, "normal", True),
    ]
    print(f"각 단계 {args.hold:.1f}초씩 송신합니다. Ctrl-C 로 중단.\n")
    try:
        for name, vx, vy, wz, mode_str, stopped in steps:
            steer, rpm, mode, note = fw.encode(vx, vy, wz, mode_str, stopped, params)
            frame = fw.build_command_frame(steer, rpm, mode, 0x01)
            print(f"[{name}]")
            print(f"   twist  vx={vx:+.2f} vy={vy:+.2f} wz={wz:+.2f}")
            print(f"   전송   steer={steer:+7.2f}° rpm={rpm:+8.2f} "
                  f"mode={mode}({MODE_NAMES.get(mode, '?')})")
            print(f"   프레임 {frame.hex(' ').upper()}")
            if note:
                print(f"   ⚠ {note}")
            deadline = time.time() + args.hold
            while time.time() < deadline:
                link.send(frame)
                time.sleep(1.0 / args.rate)
            rx = link.poll_rx()
            if rx:
                print(f"   RX {len(rx)}B: {rx[:32].hex(' ').upper()}")
            print()
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        send_stop(link)
        print(f"정지 프레임 전송 완료. 총 {link.tx_count} 프레임 송신.")


def main():
    ap = argparse.ArgumentParser(
        description="Jetson -> STM32 UART 수동 조작/점검 도구",
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="/dev/ttyTHS1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--mode", choices=["teleop", "direct", "sequence"],
                    default="teleop")
    ap.add_argument("--dry-run", action="store_true",
                    help="시리얼을 열지 않고 프레임만 출력")
    ap.add_argument("--rate", type=float, default=50.0,
                    help="송신 주기 [Hz]. STM32 timeout(200ms) 보다 빨라야 함")
    ap.add_argument("--hold", type=float, default=2.0,
                    help="direct/sequence 에서 한 값을 유지할 시간 [s]")
    ap.add_argument("--step-v", type=float, default=0.05)
    ap.add_argument("--step-w", type=float, default=0.05)
    ap.add_argument("--max-vx", type=float, default=0.45)
    ap.add_argument("--max-vy", type=float, default=0.30)
    ap.add_argument("--max-wz", type=float, default=0.80)
    # 4WIS 기하 (base_control.yaml 과 같은 값을 쓸 것)
    ap.add_argument("--wheelbase", type=float, default=0.9116)
    ap.add_argument("--track", type=float, default=1.0)
    ap.add_argument("--rws", type=float, default=0.0)
    ap.add_argument("--wheel-radius", type=float, default=0.103)
    ap.add_argument("--gear-ratio", type=float, default=1.0)
    ap.add_argument("--max-steer-deg", type=float, default=30.0)
    args = ap.parse_args()

    params = fw.FourWISParams(
        wheelbase_m=args.wheelbase, track_m=args.track, rws_ratio=args.rws,
        wheel_radius_m=args.wheel_radius, gear_ratio=args.gear_ratio,
        max_steer_deg=args.max_steer_deg)

    r_min = fw.min_turn_radius(params)
    print(f"4WIS: 최소 회전반경 {r_min:.2f} m, "
          f"vx={args.max_vx} m/s 에서 최대 wz={fw.max_angular_speed(params, args.max_vx):.2f} rad/s")
    if args.dry_run:
        print("DRY-RUN: 시리얼을 열지 않습니다.\n")
    else:
        print(f"포트 {args.port} @ {args.baud} 8N1\n")

    link = Link(args.port, args.baud, args.dry_run)
    try:
        {"teleop": run_teleop, "direct": run_direct,
         "sequence": run_sequence}[args.mode](link, params, args)
    finally:
        link.close()


if __name__ == "__main__":
    main()
