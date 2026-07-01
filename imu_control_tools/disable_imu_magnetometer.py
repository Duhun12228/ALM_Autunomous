#!/usr/bin/env python3
"""Enable or disable an E2BOX/E2BDX magnetometer over its serial port."""

import argparse
import math
import sys
import time
from typing import List, Optional, Tuple

try:
    import serial
except ImportError:
    print("ERROR: pyserial is required. Install with: pip install pyserial", file=sys.stderr)
    sys.exit(1)


def send_command(ser: "serial.Serial", command: str, timeout: float) -> str:
    payload = f"<{command}>".encode("ascii")
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    ser.write(payload)
    ser.flush()

    # The reply can arrive between streaming data or in multiple reads.
    deadline = time.monotonic() + timeout
    response = bytearray()
    while time.monotonic() < deadline and b"<ok>" not in response:
        waiting = ser.in_waiting
        response.extend(ser.read(waiting if waiting > 0 else 1))

    return response.decode("ascii", errors="replace").strip()


def send_sem_command(port: str, baudrate: int, command: str, timeout: float = 2.0) -> str:
    with serial.Serial(port, baudrate, timeout=timeout, write_timeout=timeout) as ser:
        return send_command(ser, command, timeout)


def parse_six_axis_line(line: str) -> Optional[Tuple[List[float], List[float]]]:
    """Parse Euler(3), gyro(3), accelerometer(3) from E2BOX ASCII output."""
    if not line.startswith("*"):
        return None

    try:
        values = [float(value.strip()) for value in line[1:].split(",")]
    except ValueError:
        return None

    if len(values) < 9:
        return None
    return values[3:6], values[6:9]


def axis_bar(value: float, full_scale: float, width: int = 21) -> str:
    value = max(-full_scale, min(full_scale, value))
    center = width // 2
    position = round(center + (value / full_scale) * center)
    chars = [" "] * width
    chars[center] = "|"
    chars[position] = "#"
    return "[" + "".join(chars) + "]"


def monitor_six_axis(port: str, baudrate: int, timeout: float) -> int:
    # These commands also make the streamed field order deterministic:
    # Euler XYZ, gyro XYZ, acceleration XYZ, with no magnetometer fields.
    commands = (
        ("sem0", "magnetometer fusion OFF"),
        ("soc1", "ASCII output"),
        ("sof1", "Euler prefix"),
        ("sog1", "gyroscope XYZ output ON"),
        ("soa1", "accelerometer XYZ output ON"),
        ("som0", "magnetometer XYZ output OFF"),
    )

    try:
        with serial.Serial(port, baudrate, timeout=0.1, write_timeout=timeout) as ser:
            print("Configuring IMU for a live 6-axis check...")
            for command, description in commands:
                response = send_command(ser, command, timeout)
                if "<ok>" not in response:
                    print(
                        f"FAIL: <{command}> ({description}) did not return <ok>. "
                        f"Received: {response!r}",
                        file=sys.stderr,
                    )
                    return 1
                print(f"  <{command}>: OK ({description})")

            ser.reset_input_buffer()
            print("\nMove or tilt the IMU. Press Ctrl+C to stop.")
            time.sleep(0.8)

            last_draw = 0.0
            valid_samples = 0
            while True:
                raw_line = ser.readline()
                if not raw_line:
                    continue
                line = raw_line.decode("ascii", errors="ignore").strip()
                parsed = parse_six_axis_line(line)
                if parsed is None:
                    continue

                gyro, accel = parsed
                valid_samples += 1
                now = time.monotonic()
                if now - last_draw < 0.1:
                    continue
                last_draw = now

                gyro_norm = math.sqrt(sum(value * value for value in gyro))
                accel_norm = math.sqrt(sum(value * value for value in accel))
                moving = gyro_norm > 1.0
                print("\033[2J\033[H", end="")
                print("E2BOX/E2BDX LIVE 6-AXIS CHECK")
                print("Magnetometer fusion : OFF  (<sem0> acknowledged)")
                print("Magnetometer output : OFF  (<som0> acknowledged)")
                print(f"Samples received    : {valid_samples}")
                print(f"Motion              : {'DETECTED' if moving else 'still / slow'}")
                print("\nGyroscope [deg/s]")
                for axis, value in zip("XYZ", gyro):
                    print(f"  {axis}: {value:9.2f} {axis_bar(value, 200.0)}")
                print(f"  magnitude: {gyro_norm:.2f} deg/s")
                print("\nAccelerometer [g]")
                for axis, value in zip("XYZ", accel):
                    print(f"  {axis}: {value:9.3f} {axis_bar(value, 2.0)}")
                print(f"  magnitude: {accel_norm:.3f} g (stationary is about 1 g)")
                print("\nCtrl+C: stop")
    except KeyboardInterrupt:
        print("\nLive check stopped.")
        return 0
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Disable/enable the magnetometer on E2BOX/E2BDX IMU via serial commands."
    )
    parser.add_argument(
        "--port",
        default="/dev/ttyUSB0",
        help="Serial port for IMU (default: /dev/ttyUSB0)",
    )
    parser.add_argument(
        "--baudrate",
        type=int,
        default=115200,
        help="Baudrate for IMU serial port (default: 115200)",
    )
    parser.add_argument(
        "--state",
        choices=["off", "on"],
        default="off",
        help="Choose off to send <sem0> or on to send <sem1> (default: off)",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=2.0,
        help="Serial response timeout in seconds (default: 2.0)",
    )
    parser.add_argument(
        "--monitor",
        action="store_true",
        help=(
            "configure ASCII gyro/accelerometer output with magnetometer disabled, "
            "then show a live 6-axis terminal display"
        ),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        print("ERROR: --timeout must be greater than zero.", file=sys.stderr)
        return 2

    if args.monitor:
        if args.state != "off":
            print("ERROR: --monitor cannot be combined with --state on.", file=sys.stderr)
            return 2
        return monitor_six_axis(args.port, args.baudrate, args.timeout)

    command = "sem0" if args.state == "off" else "sem1"

    print(f"Opening serial port {args.port} at {args.baudrate} baud...")
    try:
        response = send_sem_command(args.port, args.baudrate, command, timeout=args.timeout)
    except serial.SerialException as exc:
        print(f"Serial error: {exc}", file=sys.stderr)
        return 2

    if "<ok>" in response:
        print(f"SUCCESS: sent <{command}> and received <ok> response.")
        return 0
    print(f"FAIL: sent <{command}> but did not receive <ok>. Received: {response!r}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
