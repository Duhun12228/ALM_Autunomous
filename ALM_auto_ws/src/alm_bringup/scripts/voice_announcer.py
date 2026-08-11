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

# 스피커 버튼 수신은 선택 기능이다. python3-dbus 나 PyGObject 가 없는 환경에서도
# 음성 안내 본체는 그대로 돌아야 하므로 import 실패를 여기서 흡수한다.
try:
    import dbus
    import dbus.mainloop.glib
    import dbus.service
    from gi.repository import GLib

    HAVE_DBUS = True
except ImportError:                                  # pragma: no cover
    HAVE_DBUS = False

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

# ── 스피커 버튼 (AVRCP) ────────────────────────────────────────────────
# 이 커널에는 uinput 이 없다 (CONFIG_INPUT_UINPUT 미설정, 모듈도 없음). 그래서
# BlueZ 의 기본 경로인 "AVRCP 키 → 가상 키보드 → /dev/input/event*" 가 통째로
# 막혀 있다. 커널을 다시 빌드하지 않는 한 evdev 로는 못 받는다.
#
# 대신 org.bluez.Media1.RegisterPlayer 로 플레이어를 등록하면, 스피커가 보낸
# PASS THROUGH 가 이 객체의 MPRIS 메서드 호출로 들어온다. root 는 필요 없다 —
# bluetoothd 가 우리 unique name(:1.xxx) 으로 콜백하기 때문이다.
BT_ADAPTER = "/org/bluez/hci0"
BT_PLAYER_PATH = "/org/alm/voice_button"
MPRIS_PLAYER = "org.mpris.MediaPlayer2.Player"
# 등록 인자 타입이 까다롭다. Position 을 uint32/uint64 로 주면 등록 자체가
# InvalidArguments 로 거부되고, 그러면 콜백이 영영 안 온다 (증상은 '무반응').
BT_PLAYER_PROPS = {
    "PlaybackStatus": "playing",
    "Identity": "ALM Voice",
    "LoopStatus": "None",
    "Shuffle": False,                # 반드시 bool
    "Position": 0,                   # 반드시 int64
    "CanPlay": True,
    "CanPause": True,
    "CanControl": True,
}
# 실측: 이 스피커는 버튼 하나로 PLAY 와 PAUSE 를 **번갈아** 보낸다 (자체적으로
# 재생 상태를 토글해 추적하기 때문). Play() 만 구현하면 정확히 절반을 놓친다.
# 최소 누름 간격은 약 500 ms 였으므로 200 ms 디바운스면 중복만 걸러진다.
BT_REGISTER_RETRY_SEC = 15.0

# 주소를 또박또박 읽기 위한 표. "192" 를 "one hundred ninety two" 로 읽으면
# 받아적을 수가 없다. 한 자리씩 읽는다.
DIGIT_WORDS = {"0": "zero", "1": "one", "2": "two", "3": "three", "4": "four",
               "5": "five", "6": "six", "7": "seven", "8": "eight", "9": "nine"}
# 이보다 긴 SSID 는 한 글자씩 읽으면 너무 길어진다. 통째로 넘겨 piper 가 읽게 둔다.
SSID_SPELL_MAX = 16


if HAVE_DBUS:
    class ButtonPlayer(dbus.service.Object):
        """BlueZ 가 AVRCP 패스스루를 이 객체의 메서드로 전달한다.

        MPRIS 플레이어 흉내만 낸다 — 실제로 뭘 재생하지는 않는다. 필요한 것은
        '버튼이 눌렸다'는 사실 하나뿐이다.
        """

        def __init__(self, bus, on_press, debounce):
            super().__init__(conn=bus, object_path=BT_PLAYER_PATH)
            self._on_press = on_press
            self._debounce = debounce
            self._last = 0.0

        def _fire(self, key):
            now = time.monotonic()
            if now - self._last < self._debounce():
                return
            self._last = now
            try:
                self._on_press(key)
            except Exception:                        # noqa: BLE001, S110
                pass          # 콜백이 터져도 D-Bus 루프는 살아 있어야 한다

        # 여섯 메서드의 몸통이 같지만 루프로 만들어 붙일 수는 없다 —
        # dbus.service 의 메타클래스가 **클래스 생성 시점에** 클래스 딕셔너리를
        # 훑어 메서드 표를 만들기 때문에, 나중에 setattr 한 것은 등록되지 않는다.
        @dbus.service.method(MPRIS_PLAYER)
        def Play(self):                              # noqa: N802 - D-Bus 규약
            self._fire("PLAY")

        @dbus.service.method(MPRIS_PLAYER)
        def Pause(self):                             # noqa: N802
            self._fire("PAUSE")

        @dbus.service.method(MPRIS_PLAYER)
        def PlayPause(self):                         # noqa: N802
            self._fire("PLAYPAUSE")

        @dbus.service.method(MPRIS_PLAYER)
        def Stop(self):                              # noqa: N802
            self._fire("STOP")

        @dbus.service.method(MPRIS_PLAYER)
        def Next(self):                              # noqa: N802
            self._fire("NEXT")

        @dbus.service.method(MPRIS_PLAYER)
        def Previous(self):                          # noqa: N802
            self._fire("PREVIOUS")


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
        self.declare_parameter("button_enabled", True)
        # 비어 있으면 누를 때마다 네트워크 상태(SSID + IPv4)를 읽는다.
        # 문자열을 넣으면 그 문장만 말한다.
        self.declare_parameter("button_text", "")
        self.declare_parameter("button_debounce_sec", 0.2)
        self.declare_parameter("button_length_scale", 1.35)

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

        # ── 스피커 버튼 ─────────────────────────────────────────────────
        self._button_loop = None
        self._button_bus = None
        if self._param("button_enabled"):
            if HAVE_DBUS:
                threading.Thread(target=self._button_thread, daemon=True).start()
            else:
                self.get_logger().warn(
                    "python3-dbus / PyGObject 가 없어 스피커 버튼 수신을 건너뜁니다")

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

    # ── 스피커 버튼 ─────────────────────────────────────────────────────
    def _button_thread(self):
        """BlueZ 에 플레이어를 등록하고 GLib 루프를 돈다.

        전용 스레드인 이유: dbus-python 콜백은 GLib 메인루프가 돌아야 오는데,
        메인 스레드는 이미 rclpy 가 쓰고 있다.
        """
        dbus.mainloop.glib.DBusGMainLoop(set_as_default=True)
        try:
            self._button_bus = dbus.SystemBus()
            player = ButtonPlayer(self._button_bus, self._on_button,
                                  lambda: float(self._param("button_debounce_sec")))
        except Exception as error:                   # noqa: BLE001
            self.get_logger().warn(f"버튼 수신 준비 실패: {error}")
            return

        # 어댑터가 아직 안 올라왔거나 bluetoothd 가 재기동 중일 수 있다. 될
        # 때까지 재시도하되, 노드 종료 요청이 오면 즉시 빠져나온다.
        while not self._stop.is_set():
            if self._register_player():
                break
            if self._stop.wait(BT_REGISTER_RETRY_SEC):
                return

        self._button_loop = GLib.MainLoop()
        try:
            self._button_loop.run()
        except Exception as error:                   # noqa: BLE001
            self.get_logger().warn(f"버튼 루프 종료: {error}")
        finally:
            del player

    def _register_player(self):
        try:
            media = dbus.Interface(
                self._button_bus.get_object("org.bluez", BT_ADAPTER), "org.bluez.Media1")
            media.RegisterPlayer(
                dbus.ObjectPath(BT_PLAYER_PATH),
                dbus.Dictionary({
                    "PlaybackStatus": BT_PLAYER_PROPS["PlaybackStatus"],
                    "Identity": BT_PLAYER_PROPS["Identity"],
                    "LoopStatus": BT_PLAYER_PROPS["LoopStatus"],
                    "Shuffle": dbus.Boolean(BT_PLAYER_PROPS["Shuffle"]),
                    "Position": dbus.Int64(BT_PLAYER_PROPS["Position"]),
                    "CanPlay": dbus.Boolean(BT_PLAYER_PROPS["CanPlay"]),
                    "CanPause": dbus.Boolean(BT_PLAYER_PROPS["CanPause"]),
                    "CanControl": dbus.Boolean(BT_PLAYER_PROPS["CanControl"]),
                }, signature="sv"))
        except Exception as error:                   # noqa: BLE001
            self._throttled("bt_player", f"스피커 버튼 등록 실패 — 재시도합니다: {error}")
            return False
        fixed = self._param("button_text")
        self.get_logger().info(
            f'스피커 버튼 수신 준비 완료 — 누르면 "{fixed}"' if fixed
            else "스피커 버튼 수신 준비 완료 — 누르면 네트워크 상태를 읽습니다")
        return True

    def _on_button(self, key):
        """버튼 1회 = 콜백 1회. PLAY/PAUSE 구분은 의미가 없어 로그에만 남긴다."""
        text = self._param("button_text") or self._network_phrase()
        self.get_logger().info(f"스피커 버튼 (AVRCP {key}) → {text}")
        # key 를 주지 않는다. 연달아 누르면 큐에서 갈아끼워져 한 번만 들리는데,
        # 안 들려서 다시 누른 사람에게는 그게 '고장' 으로 보인다.
        # 주소는 받아적는 정보라 기본 속도로는 빠르다 — 느리게 읽는다.
        self.say(text, Speech.PRIORITY_NORMAL, scale=self._button_scale())

    def _button_scale(self):
        """1.0 이하면 기본 속도로 둔다 (캐시 키에도 안 들어간다)."""
        scale = float(self._param("button_length_scale"))
        return scale if scale > 1.0 else None

    # ── 네트워크 보고 ───────────────────────────────────────────────────
    def _network_phrase(self):
        """"지금 이 젯슨에 어떻게 접속하나" 를 한 문장으로.

        `hostname -I` 를 그대로 읽지 않는 이유: 여기에는 IPv6 도 섞여 나오는데
        (실측 4개 중 2개), 128 비트 주소를 한 자리씩 읽으면 40초가 넘는다.
        받아적을 수 있는 것은 IPv4 뿐이라 IPv4 만 읽는다.
        """
        addrs = self._ipv4_addresses()
        wireless_ip = next((ip for is_wireless, _, ip in addrs if is_wireless), "")
        wired_ip = next((ip for is_wireless, _, ip in addrs if not is_wireless), "")

        ssid = self._wifi_ssid()
        if not ssid:
            # 붙은 줄 알았는데 아니었던 경우가 제일 헷갈린다. 명시적으로 말한다.
            parts = ["Wifi not connected"]
            # 그래도 유선이 살아 있으면 접속할 길은 있다. 알려주는 편이 낫다.
            if wired_ip:
                parts.append(f"Wired {self._spell_ip(wired_ip)}")
            return ". ".join(parts)

        # 주소는 하나만 읽는다. 무선·유선을 다 읽으면 한 번 누를 때 13초가 넘어
        # 끝까지 듣고 있기 힘들다.
        parts = [f"Wifi {self._spell_ssid(ssid)}"]
        # 연결은 됐는데 주소를 못 받은 상태(DHCP 실패)가 실제로 있다.
        # 그때 주소를 통째로 빼먹으면 스피커가 고장 난 것처럼 들린다.
        parts.append(f"Address {self._spell_ip(wireless_ip)}" if wireless_ip
                     else "No address")
        return ". ".join(parts)

    def _wifi_ssid(self):
        out = self._run_text(["iwgetid", "-r"], timeout=3)
        if out and out.strip():
            return out.strip()
        # iwgetid 는 wext 기반이라 못 잡는 드라이버가 있다. NetworkManager 로 재확인.
        out = self._run_text(["nmcli", "-t", "-f", "ACTIVE,SSID", "dev", "wifi"], timeout=5)
        for line in (out or "").splitlines():
            if line.startswith("yes:"):
                return line[4:].strip()
        return ""

    def _ipv4_addresses(self):
        """[(무선인가, 인터페이스, 주소)] — 무선 먼저, 루프백 제외."""
        out = self._run_text(["ip", "-4", "-o", "addr", "show"], timeout=3)
        found = []
        for line in (out or "").splitlines():
            fields = line.split()
            if len(fields) < 4 or fields[1] == "lo":
                continue
            iface = fields[1]
            ip = fields[3].split("/")[0]
            wireless = os.path.exists(f"/sys/class/net/{iface}/wireless")
            found.append((wireless, iface, ip))
        found.sort(key=lambda item: not item[0])
        return found

    @staticmethod
    def _spell_ip(ip):
        """'192.168.1.5' → 'one nine two dot one six eight dot one dot five'"""
        return " dot ".join(" ".join(DIGIT_WORDS.get(digit, digit) for digit in octet)
                            for octet in ip.split("."))

    @staticmethod
    def _spell_ssid(ssid):
        """받아적어야 하는 이름이라 한 글자씩 읽는다.

        비 ASCII(한글 등)가 섞이면 영어 음성으로는 뭉개지므로 이름을 포기하고
        연결 사실만 알린다 — 틀린 이름을 말하는 것보다 낫다.
        """
        if not ssid.isascii():
            return "connected"
        if len(ssid) > SSID_SPELL_MAX:
            return ssid
        spelled = []
        for char in ssid:
            if char.isdigit():
                spelled.append(DIGIT_WORDS[char])
            elif char.isalpha():
                spelled.append(char.upper())
            elif char == "-":
                spelled.append("dash")
            elif char == "_":
                spelled.append("underscore")
            else:
                spelled.append(char)
        return " ".join(spelled)

    # ── 큐 넣기 ─────────────────────────────────────────────────────────
    def say(self, text, priority=Speech.PRIORITY_NORMAL, key="", interrupt=False,
            scale=None):
        """scale 은 piper 의 length_scale — 1보다 크면 느려진다. None 이면 기본 속도."""
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
            heappush(self._queue,
                     (-int(priority), self._seq, text, key, bool(interrupt), scale))
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
                _, _, text, _, _, scale = heappop(self._queue)

            gap = self._param("min_interval_sec") - (time.monotonic() - self._last_spoke)
            if gap > 0:
                time.sleep(gap)
            try:
                self._speak(text, scale)
            except Exception as error:               # noqa: BLE001
                # 오디오 실패는 로봇 동작과 무관해야 한다. 절대 워커를 죽이지 않는다.
                self._throttled("speak", f"발화 실패: {error}")
            self._last_spoke = time.monotonic()

    def _speak(self, text, scale=None):
        wav = self._wav_for(text, scale)
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
    def _cache_path(self, text, scale=None):
        """속도가 키에 들어가야 한다. 같은 문장을 다른 속도로 구우면 먼저 구운
        쪽이 계속 재생된다."""
        stamp = text if scale is None else f"{text}@{float(scale):.2f}"
        digest = hashlib.sha1(stamp.encode("utf-8")).hexdigest()[:16]
        return os.path.join(self.cache_dir, f"{digest}.wav")

    def _wav_for(self, text, scale=None):
        """캐시에 있으면 그대로, 없으면 piper 로 굽는다. 실패하면 None."""
        path = self._cache_path(text, scale)
        if os.path.exists(path) and os.path.getsize(path) > 44:
            return path

        piper = self._param("piper_bin")
        model = self._param("piper_model")
        if not (os.path.exists(piper) and os.path.exists(model)):
            self._throttled("piper_missing",
                            f"piper 가 없다 ({piper}) — scripts/install_piper.sh 를 실행할 것")
            return None

        raw = path + ".raw"
        argv = [piper, "--model", model, "--output_file", raw]
        if scale is not None:
            argv += ["--length_scale", f"{float(scale):.2f}"]
        try:
            subprocess.run(argv,
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
        # (문장, 속도). 버튼 문구는 고정이 아니다(주소가 바뀌면 문장도 바뀐다).
        # 지금 값으로 한 번 구워두면 첫 누름이 즉답이 된다 — 주소가 바뀐 뒤
        # 첫 누름만 950 ms 늦고, 그다음부터는 다시 캐시에 걸린다.
        items = [(text, None) for text in PREWARM]
        if self._param("button_enabled"):
            items.append((self._param("button_text") or self._network_phrase(),
                          self._button_scale()))
        for text, scale in items:
            if self._stop.is_set():
                return
            if os.path.exists(self._cache_path(text, scale)):
                continue
            if self._wav_for(text, scale):
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
        self._unregister_player()
        return super().destroy_node()

    def _unregister_player(self):
        """등록을 남긴 채 죽으면 BlueZ 에 유령 플레이어가 남는다. 어댑터당
        플레이어는 하나뿐이라, 다음 기동이 등록에 실패한다."""
        if self._button_loop is not None:
            try:
                self._button_loop.quit()
            except Exception:                        # noqa: BLE001, S110
                pass
        if self._button_bus is None:
            return
        try:
            media = dbus.Interface(
                self._button_bus.get_object("org.bluez", BT_ADAPTER), "org.bluez.Media1")
            media.UnregisterPlayer(dbus.ObjectPath(BT_PLAYER_PATH))
        except Exception:                            # noqa: BLE001, S110
            pass


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
