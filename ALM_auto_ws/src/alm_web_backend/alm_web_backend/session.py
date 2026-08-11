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
"""

import secrets
import threading
import time


class SessionLock:
    def __init__(self, ttl_sec=15.0):
        self.ttl = float(ttl_sec)
        self._lock = threading.Lock()
        self._holder = None      # session_id
        self._label = ""         # 사람이 읽을 접속자 표시 (IP 등)
        self._expires = 0.0

    # ── 내부 ────────────────────────────────────────────────────────────
    def _expired_locked(self):
        return self._holder is not None and time.monotonic() >= self._expires

    def _clear_if_expired_locked(self):
        if self._expired_locked():
            self._holder = None
            self._label = ""
            self._expires = 0.0
            return True
        return False

    # ── 공개 API ────────────────────────────────────────────────────────
    def acquire(self, label=""):
        """비어 있으면 새 세션을 발급한다. 이미 유효한 보유자가 있으면 None."""
        with self._lock:
            self._clear_if_expired_locked()
            if self._holder is not None:
                return None
            self._holder = secrets.token_urlsafe(16)
            self._label = label
            self._expires = time.monotonic() + self.ttl
            return self._holder

    def heartbeat(self, session_id):
        """리스 갱신. 보유자가 아니면 False."""
        with self._lock:
            self._clear_if_expired_locked()
            if session_id and self._holder == session_id:
                self._expires = time.monotonic() + self.ttl
                return True
            return False

    def release(self, session_id):
        with self._lock:
            if session_id and self._holder == session_id:
                self._holder = None
                self._label = ""
                self._expires = 0.0
                return True
            return False

    def holds(self, session_id):
        """이 세션이 지금 제어권을 쥐고 있는가. 만료된 락은 자동으로 비운다."""
        with self._lock:
            self._clear_if_expired_locked()
            return bool(session_id) and self._holder == session_id

    def status(self):
        with self._lock:
            self._clear_if_expired_locked()
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
            self._holder = None
            self._label = ""
            self._expires = 0.0
