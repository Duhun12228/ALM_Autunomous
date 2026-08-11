#!/usr/bin/env python3
"""음성 안내 — 실차에서 화면을 못 볼 때 로봇이 직접 말한다.

왜 있는가
---------
젯슨을 차에 넣으면 조작자는 화면에서 떨어져 있다. 웹에서 매핑 시작을 눌렀을 때
요청이 로봇까지 갔는지, 프로세스가 정말 떴는지 확인할 방법이 브라우저 토스트뿐인데
차 옆에 서서 태블릿을 들여다보고 있을 수는 없다. 그래서 귀로 확인한다.

왜 노드 하나가 오디오를 독점하는가
---------------------------------
발화 지점은 여럿이다(백엔드 · command_manager · 안전 토픽). 각자 piper 를 부르면
① 호출한 스레드가 오디오에 묶이고 ② 여러 발화가 스피커를 동시에 잡아 겹치고
③ 백엔드를 재기동할 때마다 소리가 끊긴다. **장치를 만지는 곳은 한 군데여야 한다.**

설계에서 지킨 것
---------------
- **오디오 실패가 로봇에 영향을 주지 않는다.** 모든 재생은 워커 스레드에서 돌고,
  subprocess 는 전부 타임아웃이 있고, 예외는 삼키되 로그는 속도 제한한다.
- **piper 를 상주시키지 않는다.** 모델 63 MB + onnxruntime 이면 RSS 가 200~300 MB 다.
  대신 텍스트 해시로 WAV 를 캐시한다. 어휘가 좁아서 곧 전부 캐시에 남는다.
- **안전 사건은 웹을 거치지 않는다.** /emergency_stop 을 직접 구독한다 — 웹이
  죽었거나 물리 버튼으로 눌린 정지도 알려야 한다.
"""
import hashlib
import os
import re
import shutil
import subprocess
import threading
import time
import wave
from heapq import heapify, heappop, heappush

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Bool

from alm_msgs.msg import McuState, Speech

# 큐 상한. 밀린 안내를 무한정 쌓아두면 한참 전 사건을 지금 사건인 양 말하게 된다.
MAX_QUEUE = 8
# 같은 실패를 매번 찍으면 로그가 그것만 남는다. 종류당 이 간격으로 한 번.
LOG_THROTTLE_SEC = 60.0
# 기동 시 미리 구워둘 문구. 첫 발화의 950 ms 합성 지연을 없앤다.
PREWARM = (
    "Starting mapping", "Mapping started", "Mapping failed to start",
    "Stopping mapping", "Mapping stopped",
    "Saving map", "Map saved", "Map save failed",
    "Building two D map", "Two D map ready", "Two D map failed",
    "Building localization database", "Localization database ready",
    "Localization database failed",
    "Emergency stop", "Emergency stop released", "M C U fault",
    "S L A M process exited", "Voice ready",
)


class VoiceAnnouncer(Node):
    def __init__(self):
        super().__init__("voice_announcer")

        home = os.path.expanduser("~")
        self.declare_parameter("enabled", True)
        self.declare_parameter("speaker_mac", "")
        self.declare_parameter("fallback_sink", "")
        self.declare_parameter("piper_bin", f"{home}/.local/share/alm-voice/piper/piper")
        self.declare_parameter("piper_model",
                               f"{home}/.local/share/alm-voice/voices/en_US-lessac-low.onnx")
        self.declare_parameter("cache_dir", f"{home}/.cache/alm_voice")
        self.declare_parameter("lead_silence_ms", 300)
        self.declare_parameter("min_interval_sec", 0.4)
        self.declare_parameter("prewarm", True)
        self.declare_parameter("announce_estop", True)

        self.cache_dir = self._param("cache_dir")
        os.makedirs(self.cache_dir, exist_ok=True)

        # ── 큐 상태 ─────────────────────────────────────────────────────
        # heapq 로 (우선순위 역순, 시퀀스) 정렬. 시퀀스가 있어야 같은 우선순위
        # 안에서 도착 순서가 지켜진다.
        self._queue = []
        self._seq = 0
        self._lock = threading.Condition()
        self._playing = None          # 현재 재생 중인 subprocess
        self._stop = threading.Event()
        self._log_at = {}
        self._last_spoke = 0.0

        # ── 오디오 대상 ─────────────────────────────────────────────────
        self._sink = None
        self._sink_checked_at = 0.0
        self._bt_next_try = 0.0
        self._bt_backoff = 5.0
        self.speaker_mac = self._param("speaker_mac") or self._autodetect_speaker()

        # ── 안전 사건 직접 구독 ─────────────────────────────────────────
        # /emergency_stop 은 latched(TRANSIENT_LOCAL) 다. 발행자와 맞지 않으면
        # 아예 안 받으므로 여기서도 같은 내구성을 쓴다.
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=QoSReliabilityPolicy.RELIABLE,
                             history=QoSHistoryPolicy.KEEP_LAST)
        self._estop_was = None
        self._fault_was = None
        if self._param("announce_estop"):
            self.create_subscription(Bool, "/emergency_stop", self._on_estop, latched)
            self.create_subscription(McuState, "/mcu/state", self._on_mcu, 10)

        self.create_subscription(Speech, "/alm/say", self._on_say, 10)

        self._worker = threading.Thread(target=self._run, daemon=True)
        self._worker.start()
        self.create_timer(10.0, self._watchdog)

        self.get_logger().info(
            f"voice_announcer 시작 — 스피커={self.speaker_mac or '자동탐지 실패'} "
            f"캐시={self.cache_dir}")
        if self._param("prewarm"):
            threading.Thread(target=self._prewarm, daemon=True).start()

    # ── 파라미터 ────────────────────────────────────────────────────────
    def _param(self, name):
        return self.get_parameter(name).value

    # ── 수신 ────────────────────────────────────────────────────────────
    def _on_say(self, msg):
        self.say(msg.text, msg.priority, msg.key, msg.interrupt)

    def _on_estop(self, msg):
        """상승 에지에서만 말한다. latched 토픽이라 접속할 때마다 마지막 값이
        다시 오는데, 그때마다 "Emergency stop" 을 외치면 안 된다."""
        now = bool(msg.data)
        if self._estop_was is None:
            self._estop_was = now       # 첫 수신은 현재 상태를 받아적기만 한다
            return
        if now and not self._estop_was:
            self.say("Emergency stop", Speech.PRIORITY_SAFETY, "estop", interrupt=True)
        elif not now and self._estop_was:
            self.say("Emergency stop released", Speech.PRIORITY_SAFETY, "estop")
        self._estop_was = now

    def _on_mcu(self, msg):
        fault = bool(getattr(msg, "fault", False))
        if self._fault_was is None:
            self._fault_was = fault
            return
        if fault and not self._fault_was:
            self.say("M C U fault", Speech.PRIORITY_SAFETY, "mcu_fault", interrupt=True)
        self._fault_was = fault

    # ── 큐 넣기 ─────────────────────────────────────────────────────────
    def say(self, text, priority=Speech.PRIORITY_NORMAL, key="", interrupt=False):
        text = (text or "").strip()
        if not text or not self._param("enabled"):
            return
        with self._lock:
            if key:
                # 같은 key 가 이미 큐에 있으면 갈아끼운다. 반복되는 안내가 큐를
                # 채워 다른 발화를 밀어내는 것을 막는다.
                # ⚠ 걸러낸 리스트는 더 이상 힙이 아니다 — heapify 로 되돌린다.
                pruned = [item for item in self._queue if item[3] != key]
                if len(pruned) != len(self._queue):
                    self._queue = pruned
                    heapify(self._queue)
            self._seq += 1
            # heapq 는 최소 힙이라 우선순위를 음수로 넣어야 높은 것이 먼저 나온다
            heappush(self._queue, (-int(priority), self._seq, text, key, bool(interrupt)))
            if len(self._queue) > MAX_QUEUE:
                # 넘치면 **가장 낮은 우선순위**를 버린다 (음수라 max 가 최저 우선순위).
                # 안전 안내는 남는다. remove 도 힙을 깨므로 다시 heapify.
                self._queue.remove(max(self._queue))
                heapify(self._queue)
            self._lock.notify()
        if interrupt:
            self._kill_playing()

    def _kill_playing(self):
        proc = self._playing
        if proc and proc.poll() is None:
            try:
                proc.terminate()
            except OSError:
                pass

    # ── 워커 ────────────────────────────────────────────────────────────
    def _run(self):
        while not self._stop.is_set():
            with self._lock:
                while not self._queue and not self._stop.is_set():
                    self._lock.wait(0.5)
                if self._stop.is_set():
                    return
                _, _, text, _, _ = heappop(self._queue)

            gap = self._param("min_interval_sec") - (time.monotonic() - self._last_spoke)
            if gap > 0:
                time.sleep(gap)
            try:
                self._speak(text)
            except Exception as error:               # noqa: BLE001
                # 오디오 실패는 로봇 동작과 무관해야 한다. 절대 워커를 죽이지 않는다.
                self._throttled("speak", f"발화 실패: {error}")
            self._last_spoke = time.monotonic()

    def _speak(self, text):
        wav = self._wav_for(text)
        if not wav:
            return
        sink = self._resolve_sink()
        cmd = ["paplay"]
        if sink:
            cmd.append(f"--device={sink}")
        cmd.append(wav)
        try:
            self._playing = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                                             stderr=subprocess.PIPE)
            _, err = self._playing.communicate(timeout=15)
            if self._playing.returncode not in (0, -15):   # -15 = 선점당한 것
                self._throttled("paplay", f"재생 실패: {err.decode(errors='replace').strip()}")
                self._sink = None      # 싱크가 사라졌을 수 있다. 다음에 다시 찾는다
        except subprocess.TimeoutExpired:
            self._kill_playing()
            self._throttled("paplay_timeout", "재생이 15초를 넘겨 중단했다")
        except FileNotFoundError:
            self._throttled("paplay_missing", "paplay 가 없다 — 음성 안내를 건너뛴다")
        finally:
            self._playing = None

    # ── 합성 · 캐시 ─────────────────────────────────────────────────────
    def _wav_for(self, text):
        """캐시에 있으면 그대로, 없으면 piper 로 굽는다. 실패하면 None."""
        digest = hashlib.sha1(text.encode("utf-8")).hexdigest()[:16]
        path = os.path.join(self.cache_dir, f"{digest}.wav")
        if os.path.exists(path) and os.path.getsize(path) > 44:
            return path

        piper = self._param("piper_bin")
        model = self._param("piper_model")
        if not (os.path.exists(piper) and os.path.exists(model)):
            self._throttled("piper_missing",
                            f"piper 가 없다 ({piper}) — scripts/install_piper.sh 를 실행할 것")
            return None

        raw = path + ".raw"
        try:
            subprocess.run([piper, "--model", model, "--output_file", raw],
                           input=text.encode("utf-8"), timeout=20,
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, check=True)
        except (subprocess.SubprocessError, OSError) as error:
            self._throttled("piper_run", f"합성 실패: {error}")
            self._unlink(raw)
            return None

        try:
            self._prepend_silence(raw, path, int(self._param("lead_silence_ms")))
        except Exception as error:                   # noqa: BLE001
            # 무음 삽입이 실패해도 소리는 나야 한다. 원본을 그대로 쓴다.
            self._throttled("silence", f"무음 삽입 실패({error}) — 원본 사용")
            try:
                shutil.move(raw, path)
            except OSError:
                return None
        finally:
            self._unlink(raw)
        return path if os.path.exists(path) else None

    @staticmethod
    def _prepend_silence(src, dst, millis):
        """앞에 무음을 붙인다.

        A2DP 스피커는 절전에서 깨어나는 데 시간이 걸려서 **첫 음절이 잘린다.**
        잘려도 되는 소리를 먼저 보내면 정작 하려던 말은 온전히 들린다.
        """
        with wave.open(src, "rb") as w:
            params = w.getparams()
            frames = w.readframes(w.getnframes())
        pad = b"\x00" * (int(params.framerate * millis / 1000)
                         * params.sampwidth * params.nchannels)
        tmp = dst + ".part"
        with wave.open(tmp, "wb") as out:
            out.setparams(params)
            out.writeframes(pad + frames)
        os.replace(tmp, dst)             # 원자적 — 반쯤 쓰인 캐시를 남기지 않는다

    @staticmethod
    def _unlink(path):
        try:
            os.unlink(path)
        except OSError:
            pass

    def _prewarm(self):
        """고정 어휘를 미리 구워둔다. 첫 발화가 950 ms 늦는 것을 없앤다."""
        made = 0
        for text in PREWARM:
            if self._stop.is_set():
                return
            digest = hashlib.sha1(text.encode("utf-8")).hexdigest()[:16]
            if os.path.exists(os.path.join(self.cache_dir, f"{digest}.wav")):
                continue
            if self._wav_for(text):
                made += 1
        if made:
            self.get_logger().info(f"음성 캐시 {made}개 생성 완료")

    # ── 싱크 · 블루투스 ─────────────────────────────────────────────────
    def _resolve_sink(self):
        """BT 싱크 이름은 고정이 아니다. 매번 찾되 3초는 캐시한다."""
        now = time.monotonic()
        if self._sink and (now - self._sink_checked_at) < 3.0:
            return self._sink
        self._sink_checked_at = now

        sinks = self._run_text(["pactl", "list", "sinks", "short"])
        if sinks is None:
            return None
        if self.speaker_mac:
            # 접두사가 오디오 서버마다 다르다: PulseAudio 는 bluez_sink.<MAC>.a2dp_sink,
            # PipeWire 는 bluez_output.<MAC>.a2dp-sink. 이 젯슨은 둘 다 떠 있고
            # 실제 서버는 pulse 다. 어느 쪽이 되든 잡히도록 **MAC 으로** 찾는다.
            want = self.speaker_mac.replace(":", "_").upper()
            for line in sinks.splitlines():
                parts = line.split("\t")
                if len(parts) > 1 and want in parts[1].upper():
                    self._sink = parts[1]
                    return self._sink
        self._sink = self._param("fallback_sink") or None
        return self._sink

    def _watchdog(self):
        """스피커가 절전으로 끊긴다. 싱크가 없으면 다시 붙인다 (지수 백오프)."""
        if not self._param("enabled") or not self.speaker_mac:
            return
        if self._resolve_sink() and self._sink and "bluez" in self._sink:
            self._bt_backoff = 5.0       # 붙었으면 백오프를 되돌린다
            return
        now = time.monotonic()
        if now < self._bt_next_try:
            return
        self._bt_next_try = now + self._bt_backoff
        self._bt_backoff = min(self._bt_backoff * 2, 60.0)
        # connect 는 스피커가 꺼져 있으면 수 초를 소모한다. 타이머 스레드를
        # 붙잡아 두지 않도록 따로 던진다.
        threading.Thread(target=self._bt_connect, daemon=True).start()

    def _bt_connect(self):
        out = self._run_text(["bluetoothctl", "connect", self.speaker_mac], timeout=20)
        if out and "successful" in out.lower():
            self.get_logger().info(f"블루투스 스피커 연결됨 ({self.speaker_mac})")
            self._sink = None
            self._sink_checked_at = 0.0
        else:
            self._throttled("bt_connect", f"스피커 연결 실패 ({self.speaker_mac}) — 전원 확인")

    def _autodetect_speaker(self):
        """페어링된 장치 중 Audio Sink 를 가진 첫 장치. MAC 을 설정에 박지 않기 위함."""
        devices = self._run_text(["bluetoothctl", "devices"], timeout=10)
        if not devices:
            return ""
        for line in devices.splitlines():
            match = re.match(r"Device ([0-9A-F:]{17}) (.+)", line.strip(), re.I)
            if not match:
                continue
            info = self._run_text(["bluetoothctl", "info", match.group(1)], timeout=10) or ""
            if "Audio Sink" in info:
                self.get_logger().info(f"스피커 자동탐지: {match.group(2)} ({match.group(1)})")
                return match.group(1)
        return ""

    # ── 잡동사니 ────────────────────────────────────────────────────────
    def _run_text(self, cmd, timeout=8):
        try:
            done = subprocess.run(cmd, capture_output=True, timeout=timeout, check=False)
            return done.stdout.decode(errors="replace")
        except (subprocess.SubprocessError, OSError) as error:
            self._throttled(f"run:{cmd[0]}", f"{cmd[0]} 실행 실패: {error}")
            return None

    def _throttled(self, kind, text):
        """같은 실패를 매초 찍으면 로그가 그것만 남아 진짜 문제를 가린다."""
        now = time.monotonic()
        if now - self._log_at.get(kind, -1e9) < LOG_THROTTLE_SEC:
            return
        self._log_at[kind] = now
        self.get_logger().warn(text)

    def destroy_node(self):
        self._stop.set()
        with self._lock:
            self._lock.notify_all()
        self._kill_playing()
        return super().destroy_node()


def main():
    rclpy.init()
    node = VoiceAnnouncer()
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
