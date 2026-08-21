"""한 번 돌고 끝나는 작업 — pcd2pgm, fpfh_map_builder.

이 둘은 ROS 서비스가 아니라 **argparse CLI 스크립트**다. 그래서 subprocess 로
돌리고 stdout 을 줄 단위로 받아 브라우저에 흘린다.

진행률을 지어내지 않는 것이 요점이다. 목업 UI 는 `setTimeout` 으로 25% → 50%
→ 75% 를 그렸지만, 실제 스크립트는 퍼센트를 내지 않는다. 그래서 여기서도
퍼센트를 만들지 않고 **실제 출력 줄**을 그대로 보낸다. 화면은 "무엇을 하는
중인지"를 스크립트가 말한 그대로 보여주면 된다.

로그는 링버퍼(기본 500줄)에만 남긴다. fpfh_map_builder 는 점군이 크면 수천
줄을 뱉는데, 전부 메모리에 이고 있을 이유가 없다.
"""

import os
import secrets
import subprocess
import threading
import time
from collections import deque

MAX_LINES = 500


class JobError(Exception):
    pass


class Job:
    def __init__(self, kind, argv, map_name=""):
        self.id = f"{kind}-{secrets.token_hex(4)}"
        self.kind = kind
        self.argv = list(argv)
        self.map_name = map_name
        self.state = "running"          # running | succeeded | failed | cancelled
        self.returncode = None
        self.started_at = time.time()
        self.ended_at = None
        self.error = ""
        self.lines = deque(maxlen=MAX_LINES)
        self.line_count = 0             # 링버퍼에서 잘려나간 것 포함한 총 줄 수
        self._proc = None
        self._cond = threading.Condition()

    # ── 상태 ────────────────────────────────────────────────────────────
    def snapshot(self, since=0):
        """since 번째 줄 이후만 반환. 폴링/SSE 양쪽이 같은 규약을 쓴다."""
        with self._cond:
            dropped = self.line_count - len(self.lines)
            start = max(since, dropped)
            offset = start - dropped
            return {
                "id": self.id,
                "kind": self.kind,
                "map": self.map_name,
                "state": self.state,
                "returncode": self.returncode,
                "error": self.error,
                "started_at": self.started_at,
                "ended_at": self.ended_at,
                "elapsed_sec": round((self.ended_at or time.time()) - self.started_at, 1),
                "line_count": self.line_count,
                "dropped": dropped,
                "lines": list(self.lines)[offset:] if offset < len(self.lines) else [],
                "argv": self.argv,
            }

    def _append(self, text):
        with self._cond:
            self.lines.append(text)
            self.line_count += 1
            self._cond.notify_all()

    def _finish(self, state, returncode=None, error=""):
        with self._cond:
            self.state = state
            self.returncode = returncode
            self.error = error
            self.ended_at = time.time()
            self._cond.notify_all()

    def wait_for_change(self, seen_lines, seen_state, timeout):
        """SSE 가 쓰는 대기. 새 줄이 생기거나 상태가 바뀔 때까지 막힌다."""
        with self._cond:
            self._cond.wait_for(
                lambda: self.line_count != seen_lines or self.state != seen_state,
                timeout=timeout)

    def cancel(self):
        proc = self._proc
        if proc is None or proc.poll() is not None:
            return False
        try:
            os.killpg(os.getpgid(proc.pid), 15)
        except (ProcessLookupError, PermissionError):
            return False
        return True


class JobManager:
    def __init__(self, logger=None, keep=20):
        self.logger = logger
        self._lock = threading.Lock()
        self._jobs = {}
        self._order = deque()
        self._keep = keep
        # 잡이 끝났을 때 부를 콜백 (job 하나를 받는다). 음성 안내가 이걸 쓴다.
        # HTTP 응답은 이미 나간 뒤라, 완료를 알리려면 이 경로밖에 없다.
        self.on_finish = None

    def get(self, job_id):
        with self._lock:
            job = self._jobs.get(job_id)
        if job is None:
            raise JobError(f"작업 '{job_id}' 을(를) 찾을 수 없습니다.")
        return job

    def list(self):
        with self._lock:
            return [self._jobs[i].snapshot(since=10 ** 9) for i in self._order]

    def active_for(self, map_name):
        """같은 맵에서 이미 돌고 있는 작업. 동시 실행을 막기 위해 본다."""
        with self._lock:
            for job in self._jobs.values():
                if job.state == "running" and job.map_name == map_name:
                    return job
        return None

    def start(self, kind, argv, map_name="", cwd=None, env=None):
        busy = self.active_for(map_name)
        if busy is not None:
            raise JobError(f"'{map_name}' 에서 이미 {busy.kind} 작업이 실행 중입니다 "
                           f"(id={busy.id}). 끝난 뒤에 다시 시도하세요.")

        job = Job(kind, argv, map_name)
        with self._lock:
            self._jobs[job.id] = job
            self._order.append(job.id)
            while len(self._order) > self._keep:
                stale = self._order.popleft()
                if self._jobs.get(stale) and self._jobs[stale].state == "running":
                    self._order.appendleft(stale)   # 돌고 있는 건 안 버린다
                    break
                self._jobs.pop(stale, None)

        thread = threading.Thread(target=self._run, args=(job, cwd, env),
                                  name=f"job-{job.id}", daemon=True)
        thread.start()
        return job

    def _run(self, job, cwd, env):
        if self.logger:
            self.logger.info(f"작업 시작 {job.id}: {' '.join(job.argv)}")
        job._append(f"$ {' '.join(job.argv)}")
        try:
            proc = subprocess.Popen(
                job.argv,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                stdin=subprocess.DEVNULL,
                cwd=cwd,
                env={**os.environ, **(env or {})},
                bufsize=1,
                universal_newlines=True,
                errors="replace",
                start_new_session=True,     # cancel() 이 그룹째 죽일 수 있도록
            )
        except OSError as error:
            job._append(f"[오류] 실행할 수 없습니다: {error}")
            job._finish("failed", error=str(error))
            return

        job._proc = proc
        try:
            for line in proc.stdout:
                job._append(line.rstrip("\n"))
        except Exception as error:          # noqa: BLE001
            job._append(f"[오류] 출력 읽기 실패: {error}")
        finally:
            proc.stdout.close()
        returncode = proc.wait()

        if returncode == 0:
            job._append("[완료] 정상 종료")
            job._finish("succeeded", returncode)
        elif returncode < 0:
            job._append(f"[중단] 신호 {-returncode} 로 종료")
            job._finish("cancelled", returncode)
        else:
            job._append(f"[실패] 종료 코드 {returncode}")
            job._finish("failed", returncode)

        if self.logger:
            self.logger.info(f"작업 종료 {job.id}: {job.state} (rc={returncode})")

        # 콜백이 터져도 잡 상태는 이미 확정됐다. 여기서 예외가 올라가면 워커
        # 스레드만 죽고 아무도 모른다 — 삼키고 로그만 남긴다.
        if self.on_finish:
            try:
                self.on_finish(job)
            except Exception as error:      # noqa: BLE001
                if self.logger:
                    self.logger.warning(f"작업 완료 콜백 실패 {job.id}: {error}")
