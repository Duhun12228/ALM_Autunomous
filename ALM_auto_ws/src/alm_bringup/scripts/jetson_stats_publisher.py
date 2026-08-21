#!/usr/bin/env python3
"""Jetson 온보드 리소스 실측치를 /alm/jetson_stats 로 발행.

WebUI 모니터링 탭이 쓰는 값(CPU/GPU/RAM/온도/전력/네트워크)의 출처다.
`tegrastats` 나 `jtop` 을 쓰지 않고 sysfs 를 직접 읽는다 — 서브프로세스를 띄우지
않으므로 1 Hz 상시 실행에도 부담이 없고, 출력 포맷이 L4T 버전마다 바뀌는 문제도
피한다. 필요한 경로가 없는 기종에서는 해당 필드만 NaN 이 되고 나머지는 정상 발행한다.

읽는 곳:
  CPU   /proc/stat                                   (샘플 2회 차분)
  GPU   /sys/devices/platform/gpu.0/load             (천분율)
  RAM   /proc/meminfo                                (MemTotal - MemAvailable)
  온도   /sys/devices/virtual/thermal/thermal_zone*/  (type 으로 cpu/gpu/soc/tj 매칭)
  전력   /sys/class/hwmon/hwmon*/ (name=ina3221)      VDD_IN 레일의 mV × mA
  네트워크 /sys/class/net/<if>/statistics/{tx,rx}_bytes (차분)

Orin Nano 에서 확인된 값: gpu.0/load 는 0~1000, thermal_zone 은 밀리도,
ina3221 in1_label=VDD_IN(보드 총 입력) / in2=VDD_CPU_GPU_CV / in3=VDD_SOC.
"""

import glob
import math
import os
import time

import rclpy
from rclpy.node import Node

from alm_msgs.msg import JetsonStats

NAN = float("nan")

THERMAL_ROOT = "/sys/devices/virtual/thermal"
GPU_LOAD_CANDIDATES = (
    "/sys/devices/platform/gpu.0/load",
    "/sys/devices/platform/bus@0/17000000.gpu/load",
    "/sys/devices/gpu.0/load",
)


def read_text(path):
    """sysfs 한 줄 읽기. 없거나 권한이 없으면 None (기종차를 조용히 흡수)."""
    try:
        with open(path) as f:
            return f.read().strip()
    except (OSError, ValueError):
        return None


def read_int(path):
    raw = read_text(path)
    if raw is None:
        return None
    try:
        return int(raw)
    except ValueError:
        return None


class CpuSampler:
    """/proc/stat 의 누적 지터를 두 샘플 차분으로 사용률로 바꾼다."""

    def __init__(self):
        self.prev = {}

    @staticmethod
    def _snapshot():
        out = {}
        try:
            with open("/proc/stat") as f:
                for line in f:
                    if not line.startswith("cpu"):
                        break
                    parts = line.split()
                    name = parts[0]
                    values = [int(v) for v in parts[1:]]
                    idle = values[3] + (values[4] if len(values) > 4 else 0)
                    out[name] = (sum(values), idle)
        except OSError:
            pass
        return out

    def sample(self):
        """(전체 %, [코어별 %]) 반환. 첫 호출은 기준점이 없어 NaN."""
        now = self._snapshot()
        total_pct = NAN
        cores = []

        for name in sorted(now, key=lambda n: (len(n), n)):
            total, idle = now[name]
            prev = self.prev.get(name)
            pct = NAN
            if prev is not None:
                d_total = total - prev[0]
                d_idle = idle - prev[1]
                if d_total > 0:
                    pct = max(0.0, min(100.0, (d_total - d_idle) / d_total * 100.0))
            if name == "cpu":
                total_pct = pct
            else:
                cores.append(pct)

        self.prev = now
        return total_pct, cores


class NetSampler:
    """인터페이스 바이트 카운터 차분 → Mbps."""

    def __init__(self, interface=""):
        self.interface = interface or self._autodetect()
        self.prev = None

    @staticmethod
    def _autodetect():
        """lo 와 다운된 링크를 뺀 첫 인터페이스. 무선을 우선한다."""
        candidates = []
        for path in sorted(glob.glob("/sys/class/net/*")):
            name = os.path.basename(path)
            if name == "lo":
                continue
            if read_text(os.path.join(path, "operstate")) != "up":
                continue
            candidates.append(name)
        for name in candidates:
            if name.startswith(("wl", "wlan")):
                return name
        return candidates[0] if candidates else ""

    def sample(self):
        if not self.interface:
            return NAN, NAN
        base = f"/sys/class/net/{self.interface}/statistics"
        tx = read_int(f"{base}/tx_bytes")
        rx = read_int(f"{base}/rx_bytes")
        now = time.monotonic()
        if tx is None or rx is None:
            return NAN, NAN

        result = (NAN, NAN)
        if self.prev is not None:
            p_tx, p_rx, p_t = self.prev
            dt = now - p_t
            if dt > 0:
                # 바이트 → Mbps. 카운터 랩어라운드/재기동이면 음수가 되므로 버린다.
                d_tx, d_rx = tx - p_tx, rx - p_rx
                if d_tx >= 0 and d_rx >= 0:
                    result = (d_tx * 8 / dt / 1e6, d_rx * 8 / dt / 1e6)
        self.prev = (tx, rx, now)
        return result


class ThermalReader:
    """thermal_zone 의 type 을 한 번만 훑어 zone 경로를 고정한다."""

    def __init__(self):
        self.zones = {}
        for path in sorted(glob.glob(os.path.join(THERMAL_ROOT, "thermal_zone*"))):
            ztype = read_text(os.path.join(path, "type"))
            if ztype:
                self.zones.setdefault(ztype, os.path.join(path, "temp"))

    def _celsius(self, ztype):
        milli = read_int(self.zones.get(ztype, ""))
        return NAN if milli is None else milli / 1000.0

    def read(self):
        soc = [self._celsius(f"soc{i}-thermal") for i in range(3)]
        soc = [v for v in soc if not math.isnan(v)]
        return (
            self._celsius("cpu-thermal"),
            self._celsius("gpu-thermal"),
            max(soc) if soc else NAN,
            self._celsius("tj-thermal"),
        )


class PowerReader:
    """ina3221 hwmon 에서 VDD_IN 레일(보드 총 입력)의 순시 전력."""

    def __init__(self, rail="VDD_IN"):
        self.volt_path = ""
        self.curr_path = ""
        for hwmon in sorted(glob.glob("/sys/class/hwmon/hwmon*")):
            if read_text(os.path.join(hwmon, "name")) != "ina3221":
                continue
            for label_path in sorted(glob.glob(os.path.join(hwmon, "in*_label"))):
                if read_text(label_path) != rail:
                    continue
                # in1_label -> in1_input / curr1_input
                index = os.path.basename(label_path)[2:-6]
                self.volt_path = os.path.join(hwmon, f"in{index}_input")
                self.curr_path = os.path.join(hwmon, f"curr{index}_input")
                return

    def read(self):
        mv = read_int(self.volt_path)
        ma = read_int(self.curr_path)
        if mv is None or ma is None:
            return NAN
        return mv / 1000.0 * ma / 1000.0


def read_gpu_percent():
    for path in GPU_LOAD_CANDIDATES:
        raw = read_int(path)
        if raw is not None:
            return max(0.0, min(100.0, raw / 10.0))  # 천분율
    return NAN


def read_memory():
    """(ram_used, ram_total, swap_used, swap_total) GB."""
    fields = {}
    try:
        with open("/proc/meminfo") as f:
            for line in f:
                key, _, rest = line.partition(":")
                fields[key] = int(rest.split()[0])  # kB
    except (OSError, ValueError, IndexError):
        return NAN, NAN, NAN, NAN

    def gb(key):
        return fields.get(key, 0) / (1024.0 * 1024.0)

    total = gb("MemTotal")
    available = gb("MemAvailable")
    swap_total = gb("SwapTotal")
    swap_free = gb("SwapFree")
    return total - available, total, swap_total - swap_free, swap_total


class JetsonStatsPublisher(Node):
    def __init__(self):
        super().__init__("jetson_stats_publisher")
        self.declare_parameter("topic", "/alm/jetson_stats")
        self.declare_parameter("rate_hz", 1.0)
        self.declare_parameter("net_interface", "")  # 빈 값이면 자동 탐지
        self.declare_parameter("power_rail", "VDD_IN")
        self.declare_parameter("power_avg_window", 10)

        topic = self.get_parameter("topic").value
        rate = max(0.1, float(self.get_parameter("rate_hz").value))

        self.cpu = CpuSampler()
        self.net = NetSampler(self.get_parameter("net_interface").value)
        self.thermal = ThermalReader()
        self.power = PowerReader(self.get_parameter("power_rail").value)
        self.power_history = []
        self.power_window = max(1, int(self.get_parameter("power_avg_window").value))

        self.pub = self.create_publisher(JetsonStats, topic, 10)
        self.timer = self.create_timer(1.0 / rate, self.tick)

        self.cpu.sample()  # 첫 차분 기준점. 다음 tick 부터 유효한 값이 나온다
        self.get_logger().info(
            f"{topic} 발행 시작 ({rate:.1f} Hz). net={self.net.interface or '없음'} "
            f"power={'VDD_IN' if self.power.volt_path else '미지원'} "
            f"thermal={len(self.thermal.zones)} zones"
        )

    def tick(self):
        msg = JetsonStats()
        msg.stamp = self.get_clock().now().to_msg()

        msg.cpu_percent, msg.cpu_core_percent = self.cpu.sample()
        msg.gpu_percent = read_gpu_percent()
        (msg.ram_used_gb, msg.ram_total_gb,
         msg.swap_used_gb, msg.swap_total_gb) = read_memory()
        (msg.temp_cpu, msg.temp_gpu,
         msg.temp_soc, msg.temp_tj) = self.thermal.read()

        watts = self.power.read()
        msg.power_w = watts
        if not math.isnan(watts):
            self.power_history.append(watts)
            del self.power_history[:-self.power_window]
        msg.power_avg_w = (
            sum(self.power_history) / len(self.power_history)
            if self.power_history else NAN
        )

        msg.net_interface = self.net.interface
        msg.net_tx_mbps, msg.net_rx_mbps = self.net.sample()

        uptime = read_text("/proc/uptime")
        msg.uptime_sec = int(float(uptime.split()[0])) if uptime else 0

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = JetsonStatsPublisher()
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
