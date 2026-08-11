"""HTTP 표면 — 라우팅, 인증, 세션 게이팅.

왜 프레임워크를 안 쓰나: 젯슨에 fastapi/uvicorn/aiohttp/flask 가 하나도 없다.
로봇 온보드에 pip 의존성을 새로 심으면 재현성이 나빠지고, 여기 있는 것은
엔드포인트 열댓 개짜리 어댑터지 웹앱이 아니다. 게다가 uvicorn 의 asyncio 루프와
rclpy 실행기를 한 프로세스에 엮는 것보다, 실행기(메인 스레드) + 스레드 풀
HTTP 서버 쪽이 훨씬 단순하다.

인증은 fail-closed 다. ALM_WEB_TOKEN 이 없으면 기동 자체를 거부한다
(--allow-no-auth 를 명시해야만 예외). "일단 열어두고 나중에 잠그자"는 그대로
남는다는 것을 여러 번 봤다.

게이팅 규칙:
  · 모든 요청       → Bearer 토큰 필요
  · 상태를 바꾸는 것 → 그 위에 X-ALM-Session 이 현재 보유자와 일치해야 함
  · E-STOP 만 예외  → 토큰만 있으면 누구나. 정지가 락 때문에 막히면 안 된다
"""

import hmac
import json
import os
import threading
import time
import traceback
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

from .logging_util import log as log_line

MAX_BODY = 64 * 1024
SSE_KEEPALIVE_SEC = 15.0


class ApiError(Exception):
    def __init__(self, status, message):
        super().__init__(message)
        self.status = status
        self.message = message


class Router:
    """(메서드, 경로 패턴) → 핸들러. 패턴은 '/api/jobs/{id}/cancel' 형태."""

    def __init__(self):
        self._routes = []

    def add(self, method, pattern, handler, *, needs_lock=False):
        parts = [p for p in pattern.strip("/").split("/") if p != ""]
        self._routes.append((method, parts, handler, needs_lock))

    def match(self, method, path):
        parts = [p for p in path.strip("/").split("/") if p != ""]
        allowed = set()
        for route_method, route_parts, handler, needs_lock in self._routes:
            if len(route_parts) != len(parts):
                continue
            params = {}
            for expected, actual in zip(route_parts, parts):
                if expected.startswith("{") and expected.endswith("}"):
                    params[expected[1:-1]] = actual
                elif expected != actual:
                    break
            else:
                allowed.add(route_method)
                if route_method == method:
                    return handler, params, needs_lock
        if allowed:
            raise ApiError(405, f"{method} 는 이 경로에서 지원하지 않습니다 "
                                f"(가능: {', '.join(sorted(allowed))})")
        return None, None, None


class Backend:
    """라우팅 대상이 되는 애플리케이션 상태. 실제 핸들러는 api.py 가 등록한다."""

    def __init__(self, *, token, session, router, logger=None, cors_origins=("*",)):
        self.token = token
        self.session = session
        self.router = router
        self.logger = logger
        self.cors_origins = list(cors_origins)
        self.started_at = time.time()

    def log(self, level, text):
        log_line(self.logger, level, text)

    def check_token(self, header_value):
        if not self.token:
            return True                      # --allow-no-auth 로 명시한 경우
        if not header_value or not header_value.startswith("Bearer "):
            return False
        given = header_value[len("Bearer "):].strip()
        return hmac.compare_digest(given, self.token)

    def allow_origin(self, origin):
        if "*" in self.cors_origins:
            return "*"
        if origin and origin in self.cors_origins:
            return origin
        return self.cors_origins[0] if self.cors_origins else ""


class Handler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"
    server_version = "alm_web_backend"
    sys_version = ""

    backend = None      # ThreadingHTTPServer 가 주입한다

    # ── 로깅 ────────────────────────────────────────────────────────────
    def log_message(self, fmt, *args):
        # 기본 구현은 stderr 로 직접 쓴다. ROS 로거로 돌린다.
        self.backend.log("debug", f"{self.address_string()} {fmt % args}")

    def log_error(self, fmt, *args):
        self.backend.log("warn", f"{self.address_string()} {fmt % args}")

    # ── 응답 헬퍼 ───────────────────────────────────────────────────────
    def _cors_headers(self):
        origin = self.backend.allow_origin(self.headers.get("Origin", ""))
        if origin:
            self.send_header("Access-Control-Allow-Origin", origin)
            self.send_header("Vary", "Origin")
        self.send_header("Access-Control-Allow-Headers",
                         "Authorization, Content-Type, X-ALM-Session")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, PUT, OPTIONS")
        self.send_header("Access-Control-Max-Age", "600")

    def send_json(self, status, payload):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self._cors_headers()
        self.end_headers()
        self.wfile.write(body)

    def send_error_json(self, status, message):
        self.send_json(status, {"error": message, "status": status})

    # ── 메서드 진입점 ───────────────────────────────────────────────────
    def do_OPTIONS(self):       # noqa: N802 - BaseHTTPRequestHandler 규약
        self.send_response(204)
        self.send_header("Content-Length", "0")
        self._cors_headers()
        self.end_headers()

    def do_GET(self):           # noqa: N802
        self._dispatch("GET")

    def do_POST(self):          # noqa: N802
        self._dispatch("POST")

    def do_PUT(self):           # noqa: N802
        self._dispatch("PUT")

    def _read_body(self):
        length = self.headers.get("Content-Length")
        if not length:
            return {}
        try:
            size = int(length)
        except ValueError:
            raise ApiError(400, "Content-Length 가 올바르지 않습니다.")
        if size > MAX_BODY:
            raise ApiError(413, f"요청 본문이 너무 큽니다 (최대 {MAX_BODY} 바이트).")
        if size <= 0:
            return {}
        raw = self.rfile.read(size)
        if not raw.strip():
            return {}
        try:
            data = json.loads(raw.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as error:
            raise ApiError(400, f"JSON 파싱 실패: {error}") from error
        if not isinstance(data, dict):
            raise ApiError(400, "요청 본문은 JSON 객체여야 합니다.")
        return data

    def _dispatch(self, method):
        backend = self.backend
        parsed = urlparse(self.path)
        query = {k: v[0] for k, v in parse_qs(parsed.query).items()}
        try:
            if not backend.check_token(self.headers.get("Authorization", "")):
                raise ApiError(401, "토큰이 없거나 올바르지 않습니다.")

            handler, params, needs_lock = backend.router.match(method, parsed.path)
            if handler is None:
                raise ApiError(404, f"알 수 없는 경로: {parsed.path}")

            session_id = self.headers.get("X-ALM-Session", "")
            if needs_lock and not backend.session.holds(session_id):
                status = backend.session.status()
                raise ApiError(409, "제어권이 없습니다. "
                                    + ("다른 접속자가 조작 중입니다."
                                       if status["held"] else "먼저 제어권을 확보하세요."))

            body = self._read_body() if method in ("POST", "PUT") else {}
            result = handler(Request(self, params, query, body, session_id))
            if result is None:
                result = {"ok": True}
            if isinstance(result, StreamResponse):
                result.run(self)
            else:
                self.send_json(200, result)
        except ApiError as error:
            self.send_error_json(error.status, error.message)
        except BrokenPipeError:
            pass                            # 브라우저가 먼저 끊었다
        except Exception as error:          # noqa: BLE001
            backend.log("error", f"처리 실패 {method} {parsed.path}: {error}\n"
                                 f"{traceback.format_exc()}")
            try:
                self.send_error_json(500, f"{type(error).__name__}: {error}")
            except BrokenPipeError:
                pass


class Request:
    """핸들러가 받는 요청 묶음."""

    def __init__(self, handler, params, query, body, session_id):
        self.handler = handler
        self.params = params or {}
        self.query = query or {}
        self.body = body or {}
        self.session_id = session_id
        self.client = handler.client_address[0] if handler.client_address else ""

    def str_field(self, name, default=None, required=False):
        value = self.body.get(name, default)
        if value is None or value == "":
            if required:
                raise ApiError(400, f"'{name}' 값이 필요합니다.")
            return default
        if not isinstance(value, str):
            raise ApiError(400, f"'{name}' 은 문자열이어야 합니다.")
        return value.strip()

    def bool_field(self, name, default=False):
        value = self.body.get(name, default)
        if isinstance(value, bool):
            return value
        raise ApiError(400, f"'{name}' 은 true/false 여야 합니다.")

    def float_field(self, name, default):
        value = self.body.get(name, default)
        try:
            return float(value)
        except (TypeError, ValueError):
            raise ApiError(400, f"'{name}' 은 숫자여야 합니다.") from None

    def int_field(self, name, default):
        value = self.body.get(name, default)
        try:
            return int(value)
        except (TypeError, ValueError):
            raise ApiError(400, f"'{name}' 은 정수여야 합니다.") from None

    def int_query(self, name, default=0):
        try:
            return int(self.query.get(name, default))
        except (TypeError, ValueError):
            return default


class StreamResponse:
    """SSE. 브라우저는 fetch 스트리밍으로, CLI 는 curl -N 으로 읽는다.

    EventSource 를 쓰지 않는 이유: EventSource 는 커스텀 헤더를 못 붙여서
    Bearer 토큰을 쿼리스트링에 실어야 한다. 토큰이 접속 로그에 남는다.
    """

    def __init__(self, generator):
        self.generator = generator

    def run(self, handler):
        handler.send_response(200)
        handler.send_header("Content-Type", "text/event-stream; charset=utf-8")
        handler.send_header("Cache-Control", "no-store")
        handler.send_header("Connection", "close")
        handler.send_header("X-Accel-Buffering", "no")
        handler._cors_headers()
        handler.end_headers()
        try:
            for event, payload in self.generator:
                chunk = (f"event: {event}\n"
                         f"data: {json.dumps(payload, ensure_ascii=False)}\n\n")
                handler.wfile.write(chunk.encode("utf-8"))
                handler.wfile.flush()
        except (BrokenPipeError, ConnectionResetError):
            pass
        handler.close_connection = True


def serve(backend, host, port):
    """HTTP 서버를 데몬 스레드에서 돌린다. (rclpy 는 메인 스레드가 spin)"""

    handler_cls = type("BoundHandler", (Handler,), {"backend": backend})
    httpd = ThreadingHTTPServer((host, port), handler_cls)
    httpd.daemon_threads = True
    thread = threading.Thread(target=httpd.serve_forever, name="alm-web-http", daemon=True)
    thread.start()
    return httpd


def load_token(explicit=None, allow_no_auth=False):
    """ALM_WEB_TOKEN 을 읽는다. 없으면 기동 거부 (fail-closed)."""
    token = (explicit or os.environ.get("ALM_WEB_TOKEN") or "").strip()
    if token:
        if len(token) < 8:
            raise SystemExit("ALM_WEB_TOKEN 이 너무 짧습니다 (8자 이상). "
                             "예: export ALM_WEB_TOKEN=$(openssl rand -hex 16)")
        return token
    if allow_no_auth:
        return ""
    raise SystemExit(
        "ALM_WEB_TOKEN 이 설정되지 않았습니다. 이 백엔드는 로봇에 명령을 보내므로\n"
        "인증 없이 기동하지 않습니다.\n\n"
        "  export ALM_WEB_TOKEN=$(openssl rand -hex 16)\n\n"
        "정말로 인증 없이 띄우려면 --allow-no-auth 를 명시하세요 "
        "(격리된 개발 환경에서만).")
