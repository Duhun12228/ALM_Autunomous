"""웹 세션 제어권 — 접속한 여러 브라우저 중 '누가 조작하는가'.

주의: 이것은 cmd_arbiter 의 **동작권(ownership)과 다른 축**이다.

    웹 세션 제어권 : 브라우저 여럿 중 누가 명령을 낼 수 있나   ← 이 파일
    동작권         : 로봇이 자율을 따르나 텔레옵을 따르나      ← cmd_arbiter

둘을 하나로 합치면 다중 접속에서 반드시 사고가 난다. 관전자 3명이 붙어 있는데
그중 아무나 매핑을 중단시킬 수 있으면 안 되고, 반대로 웹 세션을 쥐었다고
로봇이 자동으로 텔레옵 모드가 되어서도 안 된다.

리스(lease) 방식인 이유: 브라우저가 크래시하면 release 를 못 보낸다. 명시
반납만 인정하면 락이 영구히 잠겨 아무도 조작할 수 없게 된다. 그래서 하트비트가
TTL 동안 끊기면 자동으로 풀린다.

on_change 콜백은 제어권이 바뀐 것을 밖에 알린다 (음성 안내가 이걸 쓴다).
사건은 셋이고, **반납과 만료는 반드시 구분한다** — 반납은 조작자가 끝낸 것이고,
만료는 쥔 채로 연결이 끊긴 것이다. 실차에서 의미가 전혀 다르다.

    acquired  획득
    released  명시 반납 (버튼, 탭 닫기)
    expired   리스 만료 (브라우저 크래시, Wi-Fi 이탈)
"""

import secrets
import threading
import time


class SessionLock:
    def __init__(self, ttl_sec=15.0, on_change=None):
        self.ttl = float(ttl_sec)
        self.on_change = on_change
        self._lock = threading.Lock()
        self._holder = None      # session_id
        self._label = ""         # 사람이 읽을 접속자 표시 (IP 등)
        self._expires = 0.0

    # ── 내부 ────────────────────────────────────────────────────────────
    def _expired_locked(self):
        return self._holder is not None and time.monotonic() >= self._expires

    def _clear_locked(self):
        """보유자를 비우고 그 label 을 돌려준다. 비어 있었으면 None."""
        if self._holder is None:
            return None
        label = self._label
        self._holder = None
        self._label = ""
        self._expires = 0.0
        return label

    def _emit(self, event, label):
        """**반드시 락 밖에서** 부른다.

        콜백은 ROS 퍼블리시까지 내려간다. 뮤텍스를 쥔 채로 부르면 세션 락이
        오디오 경로에 묶인다 — status() 는 붙어 있는 브라우저마다 폴링하므로
        그 대가를 모든 조회가 나눠 문다.

        호출 스레드는 정해져 있지 않다. HTTP 핸들러일 수도 있고 만료를 감지한
        주기 타이머일 수도 있다. 콜백은 그 둘 어디서 불려도 안전해야 한다.
        """
        if self.on_change is None:
            return
        try:
            self.on_change(event, label)
        except Exception:                                # noqa: BLE001
            # 알림이 실패했다고 제어권 관리가 흔들려서는 안 된다.
            pass

    def _sync(self):
        """만료된 리스를 정리한다. 공개 메서드는 전부 여기서 시작한다.

        정리와 그 뒤의 본 작업이 원자적이지 않지만 문제되지 않는다 — 정리는
        멱등이고(_holder 를 None 으로 만드는 스레드는 뮤텍스상 하나뿐이라
        콜백도 한 번만 뜬다), 본 작업은 각자 다시 락을 잡고 조건을 재확인한다.
        """
        with self._lock:
            label = self._clear_locked() if self._expired_locked() else None
        if label is not None:
            self._emit("expired", label)

    # ── 공개 API ────────────────────────────────────────────────────────
    def poll(self):
        """주기 타이머용 — 만료를 **제때** 감지하기 위한 유일한 경로.

        만료 정리는 원래 누군가 호출해 줄 때 곁다리로만 일어난다. 그런데 락을
        쥔 브라우저가 크래시하고 그게 유일한 접속자였다면 아무도 호출하지
        않는다. 그러면 로봇은 조용히 제어권을 버린 채 아무 표시도 내지 않는다.
        실차에서 제일 알고 싶은 사건이 제일 조용한 셈이라, 여기서 깨운다.
        """
        self._sync()

    def acquire(self, label=""):
        """비어 있으면 새 세션을 발급한다. 이미 유효한 보유자가 있으면 None."""
        self._sync()
        with self._lock:
            if self._holder is not None:
                return None
            self._holder = secrets.token_urlsafe(16)
            self._label = label
            self._expires = time.monotonic() + self.ttl
            session_id = self._holder
        self._emit("acquired", label)
        return session_id

    def heartbeat(self, session_id):
        """리스 갱신. 보유자가 아니면 False."""
        self._sync()
        with self._lock:
            if session_id and self._holder == session_id:
                self._expires = time.monotonic() + self.ttl
                return True
            return False

    def release(self, session_id):
        # _sync 를 먼저 부르는 이유: 이미 만료된 리스를 뒤늦게 반납하면
        # "released" 가 아니라 "expired" 로 알려야 사실에 맞다. 연결이 20초
        # 끊겼다 돌아와서 누른 반납 버튼은, 실제로는 18초 전에 이미 풀린 것이다.
        self._sync()
        with self._lock:
            if not (session_id and self._holder == session_id):
                return False
            label = self._clear_locked()
        self._emit("released", label)
        return True

    def holds(self, session_id):
        """이 세션이 지금 제어권을 쥐고 있는가. 만료된 락은 자동으로 비운다."""
        self._sync()
        with self._lock:
            return bool(session_id) and self._holder == session_id

    def status(self):
        self._sync()
        with self._lock:
            held = self._holder is not None
            return {
                "held": held,
                "label": self._label if held else "",
                "expires_in": round(max(0.0, self._expires - time.monotonic()), 1) if held else 0.0,
                "ttl_sec": self.ttl,
            }

    def force_release(self):
        """운용자가 강제로 비우는 경로 (관리 목적). 지금은 쓰지 않는다."""
        with self._lock:
            label = self._clear_locked()
        if label is None:
            return False
        self._emit("released", label)
        return True
