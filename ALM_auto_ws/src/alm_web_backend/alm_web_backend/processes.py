"""launch 프로세스 슬롯 — 오래 사는 스택을 이름 하나에 하나씩 붙여 관리한다.

두 가지가 이 파일의 존재 이유다.

1) **프로세스 그룹 종료.** `ros2 launch` 는 자식 노드를 여럿 띄운다. 부모에만
   SIGINT 를 보내면 자식이 남아 좀비 스택이 된다. start_new_session=True 로
   새 세션(=프로세스 그룹)을 만들고, killpg 로 그룹 전체에 신호를 보낸다.
   launch 는 SIGINT 를 받으면 자식을 역순으로 정리하므로 SIGINT 가 먼저다.

2) **launch allowlist.** 웹에서 임의의 launch 파일을 띄울 수 있으면 그건
   원격 코드 실행이다. 여기 적힌 조합만 허용한다.

   ⚠ 특히 alm_bringup/slam.launch.py 는 **금지**다. 그것은 robot.launch.py
   (description + lidar + ekf + base_control + mcu_interface)까지 포함해서
   상시 스택이 이미 떠 있는 상태에서 부르면 mcu_bridge 가 중복 기동된다.
   /dev/ttyTHS1 을 두 프로세스가 열면 UART 프레임 동기가 깨진다.
   웹에서 필요한 것은 alm_navigation/slam.launch.py (매핑 엔진)뿐이다.
"""

import os
import signal
import subprocess
import threading
import time

from .logging_util import log

# 슬롯 이름 → (패키지, launch 파일). 이 표에 없으면 기동하지 않는다.
LAUNCH_ALLOWLIST = {
    "slam": ("alm_navigation", "slam.launch.py"),
    "localization": ("alm_navigation", "localization.launch.py"),
    # ⚠ alm_navigation/navigation.launch.py 는 **localization.launch.py 를
    #   포함한다** (map_server + 측위 + Nav2 코어). 측위만 따로 띄워 두고 그
    #   위에 Nav2 를 얹는 구성이 아니다 — 아래 EXCLUSIVE_SLOTS 참고.
    "navigation": ("alm_navigation", "navigation.launch.py"),
}

# 동시에 떠 있으면 안 되는 조합. 둘 다 fast_lio 를 띄우므로 /Odometry 와
# odom->base_link TF 를 두 노드가 동시에 낸다 — TF 트리에 부모가 둘인
# 상태가 되어 조회하는 쪽마다 다른 답을 받는다. 로그는 양쪽 다 깨끗해서
# 화면만 보고는 알 수 없다.
#
# navigation 이 localization 과도 배타인 것은 같은 이유다. navigation.launch.py
# 안에 localization.launch.py 가 들어 있어서, 측위를 띄워 둔 채 자율주행을
# 시작하면 FAST-LIO 가 두 개가 된다.
#
# ##운용메모## 그래서 '측위로 수렴을 확인한 뒤 그 위에 Nav2 만 얹기' 가 안 된다.
#   자율주행을 시작하면 초기 정합을 처음부터 다시 한다. 정합이 비싼 이 브랜치
#   에서는 실제로 불편한 지점이고, 고치려면 Nav2 코어만 띄우는 launch 를 따로
#   만들어야 한다. 지금 그것을 만들지 않은 이유는 배타 규칙이 하나 늘어날 때마다
#   '무엇과 무엇이 같이 뜨면 안 되는가' 가 사람 머릿속으로 옮겨가기 때문이다.
EXCLUSIVE_SLOTS = (("slam", "localization"),
                   ("slam", "navigation"),
                   ("localization", "navigation"))

# 이름이 비슷해서 헷갈리는데 절대 띄우면 안 되는 것들. 실수로 allowlist 에
# 추가하려 할 때 걸리도록 명시해 둔다.
LAUNCH_DENYLIST = {
    ("alm_bringup", "slam.launch.py"): "robot.launch.py 를 포함해 mcu_bridge 가 중복 기동됩니다",
    ("alm_bringup", "robot.launch.py"): "상시 스택과 시리얼 포트가 충돌합니다",
    ("alm_bringup", "navigation.launch.py"): "robot.launch.py 를 포함합니다",
}

STOP_SIGINT_WAIT = 10.0
STOP_SIGTERM_WAIT = 5.0


class ProcessError(Exception):
    pass


class LaunchSlot:
    def __init__(self, name, package, launch_file):
        self.name = name
        self.package = package
        self.launch_file = launch_file
        self.proc = None
        self.args = []
        self.started_at = 0.0
        self.log_path = ""
        # 우리가 내린 것인지, 혼자 죽은 것인지 구분한다. 혼자 죽은 것만
        # 음성으로 알려야 한다 — 종료 버튼을 누를 때마다 "process exited" 를
        # 외치면 정작 사고가 났을 때 무시하게 된다.
        self.stop_requested = False

    def is_running(self):
        return self.proc is not None and self.proc.poll() is None

    def info(self):
        running = self.is_running()
        return {
            "slot": self.name,
            "package": self.package,
            "launch_file": self.launch_file,
            "running": running,
            "pid": self.proc.pid if running else None,
            "args": list(self.args),
            "uptime_sec": round(time.monotonic() - self.started_at, 1) if running else 0.0,
            "returncode": None if running or self.proc is None else self.proc.returncode,
            "log_path": self.log_path,
        }


class ProcessManager:
    def __init__(self, log_dir, logger=None):
        self.log_dir = log_dir
        self.logger = logger
        self._lock = threading.Lock()
        self._slots = {
            name: LaunchSlot(name, package, launch_file)
            for name, (package, launch_file) in LAUNCH_ALLOWLIST.items()
        }
        os.makedirs(self.log_dir, exist_ok=True)

    def _log(self, level, text):
        log(self.logger, level, text)

    def slot(self, name):
        slot = self._slots.get(name)
        if slot is None:
            raise ProcessError(f"알 수 없는 슬롯 '{name}'. "
                               f"허용: {', '.join(sorted(self._slots))}")
        return slot

    def status(self):
        with self._lock:
            return [slot.info() for slot in self._slots.values()]

    def _conflicting_locked(self, name):
        """name 과 동시에 떠 있으면 안 되는데 지금 돌고 있는 슬롯 이름."""
        for pair in EXCLUSIVE_SLOTS:
            if name not in pair:
                continue
            for other in pair:
                if other != name and self._slots[other].is_running():
                    return other
        return None

    def conflicting(self, name):
        with self._lock:
            return self._conflicting_locked(name)

    # ── 로그 tail ───────────────────────────────────────────────────────
    # /rosout 으로는 안 보이는 것들을 위한 경로다. launch 자체가 못 뜨는 경우,
    # 노드가 맵을 읽다 죽는 경우, PCL/TEASER 가 stderr 로 뱉는 예외는 ROS 로그
    # 시스템을 거치지 않는다. 그때 화면의 /rosout 패널은 **비어 있고**, 조작자는
    # 멈춘 건지 도는 건지 알 수 없다 — 가장 알고 싶은 순간에 가장 안 보인다.
    MAX_TAIL_BYTES = 64 * 1024

    def read_log(self, name, since=0):
        """슬롯 로그를 바이트 오프셋부터 읽어 **완결된 줄만** 돌려준다.

        오프셋 기반인 이유: 줄 번호로 세면 매번 파일을 처음부터 읽어야 한다.
        마지막 줄이 아직 개행으로 안 끝났으면 그 줄은 남겨두고 오프셋도 그
        앞까지만 올린다 — 반쪽짜리 줄을 화면에 뿌리지 않기 위해서다.
        """
        slot = self.slot(name)
        path = slot.log_path
        if not path or not os.path.isfile(path):
            return {"slot": name, "lines": [], "offset": 0, "log_path": path,
                    "running": slot.is_running(), "truncated": False}

        size = os.path.getsize(path)
        start = max(0, int(since or 0))
        truncated = False
        if start > size:
            # 새 기동으로 로그 파일이 바뀌었다. 처음부터 다시 읽는다.
            start = 0
        if size - start > self.MAX_TAIL_BYTES:
            start = size - self.MAX_TAIL_BYTES
            truncated = True

        try:
            with open(path, "rb") as handle:
                handle.seek(start)
                chunk = handle.read(self.MAX_TAIL_BYTES)
        except OSError as error:
            raise ProcessError(f"로그를 읽을 수 없습니다: {error}") from error

        cut = chunk.rfind(b"\n") + 1            # 0 이면 완결된 줄이 없다
        offset = start + cut
        text = chunk[:cut].decode("utf-8", errors="replace")
        lines = text.splitlines()
        if truncated and lines:
            # 앞을 잘랐다면 첫 줄은 중간부터일 수 있다. 버리는 편이 정직하다.
            lines = lines[1:]
        return {"slot": name, "lines": lines, "offset": offset, "log_path": path,
                "running": slot.is_running(), "truncated": truncated}

    # ── 기동 ────────────────────────────────────────────────────────────
    def start(self, name, launch_args=None, env=None):
        launch_args = list(launch_args or [])
        with self._lock:
            slot = self.slot(name)
            if slot.is_running():
                raise ProcessError(f"'{name}' 이 이미 실행 중입니다 (pid={slot.proc.pid}). "
                                   f"먼저 종료하세요.")

            key = (slot.package, slot.launch_file)
            if key in LAUNCH_DENYLIST:
                raise ProcessError(f"{key[0]}/{key[1]} 는 웹에서 기동할 수 없습니다 — "
                                   f"{LAUNCH_DENYLIST[key]}")

            other = self._conflicting_locked(name)
            if other is not None:
                raise ProcessError(
                    f"'{other}' 가 실행 중입니다. '{name}' 과 동시에 띄울 수 "
                    f"없습니다 — 둘 다 FAST-LIO 를 띄워서 /Odometry 와 "
                    f"odom→base_link TF 가 겹칩니다. 먼저 '{other}' 를 종료하세요.")

            argv = ["ros2", "launch", slot.package, slot.launch_file] + launch_args
            stamp = time.strftime("%Y%m%d-%H%M%S")
            log_path = os.path.join(self.log_dir, f"{name}-{stamp}.log")
            handle = open(log_path, "wb", buffering=0)

            self._log("info", f"launch 기동: {' '.join(argv)} → {log_path}")
            try:
                slot.proc = subprocess.Popen(
                    argv,
                    stdout=handle,
                    stderr=subprocess.STDOUT,
                    stdin=subprocess.DEVNULL,
                    # 자체 프로세스 그룹. 종료 시 그룹 전체에 신호를 보내기 위함.
                    start_new_session=True,
                    env={**os.environ, **(env or {})},
                )
            except OSError as error:
                handle.close()
                raise ProcessError(f"기동 실패: {error}") from error
            finally:
                # 자식이 fd 를 물고 있으므로 부모 쪽은 닫아도 된다.
                handle.close()

            slot.args = launch_args
            slot.started_at = time.monotonic()
            slot.log_path = log_path
            slot.stop_requested = False
            return slot.info()

    # ── 종료 ────────────────────────────────────────────────────────────
    def stop(self, name):
        with self._lock:
            slot = self.slot(name)
            if not slot.is_running():
                return {**slot.info(), "already_stopped": True}
            # 우리가 내리는 것이다 — 음성 안내가 '혼자 죽었다'고 오해하면 안 된다.
            slot.stop_requested = True
            proc = slot.proc

        # 락을 놓고 기다린다 — 최대 15초를 락 안에서 붙들면 다른 요청이 전부 막힌다.
        pgid = os.getpgid(proc.pid)
        self._log("info", f"'{name}' 종료 요청 → SIGINT to pgid {pgid}")
        for sig, wait in ((signal.SIGINT, STOP_SIGINT_WAIT),
                          (signal.SIGTERM, STOP_SIGTERM_WAIT)):
            try:
                os.killpg(pgid, sig)
            except ProcessLookupError:
                break
            if self._wait_exit(proc, wait):
                break
            self._log("warn", f"'{name}' 이 {sig.name} 후 {wait}초 안에 안 죽었습니다.")
        else:
            try:
                self._log("error", f"'{name}' SIGKILL")
                os.killpg(pgid, signal.SIGKILL)
            except ProcessLookupError:
                pass
            self._wait_exit(proc, 3.0)

        with self._lock:
            return {**slot.info(), "already_stopped": False}

    @staticmethod
    def _wait_exit(proc, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                return True
            time.sleep(0.1)
        return proc.poll() is not None

    def stop_all(self):
        for name, slot in self._slots.items():
            if slot.is_running():
                try:
                    self.stop(name)
                except Exception as error:      # noqa: BLE001 - 종료 경로는 삼킨다
                    self._log("error", f"'{name}' 종료 실패: {error}")
