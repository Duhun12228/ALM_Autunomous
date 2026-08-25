"""엔드포인트 정의 — 각 버튼이 로봇에서 무엇을 하는지가 여기 다 있다.

경로 규약:
  · needs_lock=True  → X-ALM-Session 이 현재 보유자와 일치해야 함
  · needs_lock=False → 토큰만 있으면 됨 (조회 + E-STOP)

E-STOP 이 needs_lock=False 인 것은 의도적이다. 제어권을 남이 쥐고 있다고
로봇을 못 세우면 그게 사고다.
"""

import math
import os
import time

from .http_server import ApiError, StreamResponse
from . import localization, maps_write, navigation
from .jobs import JobError
from .logging_util import log
from .processes import ProcessError
from .ros_iface import NavRejected, RosTimeout

# 사용자가 보낼 수 있는 수치의 상한/하한. 스크립트에 이상한 값이 들어가
# 몇 시간 도는 작업이 되는 것을 막는다.
CLAMPS = {
    "resolution": (0.01, 1.0),
    "z_min": (-10.0, 10.0),
    "z_max": (-10.0, 20.0),
    # 지면 기준 장애물 밴드 (pcd2pgm 의 권장 경로). 절대 z 가 아니라 '지면 위
    # 몇 m' 이므로 라이다 마운트 높이에 영향받지 않는다.
    "obstacle_min_h": (0.0, 3.0),
    "obstacle_max_h": (0.1, 10.0),
    "ground_z": (-10.0, 10.0),
    # ★ 1 -> 2 (2026-08-25). 밴드가 지면 위 0.15~1.80 m 로 넓어져 밴드에 드는
    #   점이 크게 늘었는데 1점이면 점유 셀이 된다. 레이캐스팅은
    #   free &= ~occupied 라 광선이 지나가도 안 지워진다 — 노이즈 한 점이
    #   영구 장애물로 남는다.
    "min_points": (1, 1000),
    "voxel": (0.05, 5.0),
    "normal_radius": (0.1, 10.0),
    "feature_radius": (0.1, 20.0),
}


def _clamp(name, value):
    low, high = CLAMPS[name]
    return type(low)(max(low, min(high, value)))


class Api:
    def __init__(self, *, ros, session, processes, jobs, maps_root, map_layout,
                 fastlio_config, exe_paths, logger=None):
        self.ros = ros
        self.session = session
        self.processes = processes
        self.jobs = jobs
        self.maps_root = maps_root
        self.map_layout = map_layout
        self.fastlio_config = fastlio_config
        self.exe = exe_paths
        self.logger = logger
        # 잡은 HTTP 요청이 끝난 **뒤에** 완료된다. 그 시점에 말하려면 콜백이 필요하다.
        self.jobs.on_finish = self._job_finished
        # 제어권도 같은 이유다. 만료는 HTTP 요청이 아예 없는 채로 일어난다.
        self.session.on_change = self._session_changed
        # 측위 수렴도 마찬가지다. 기동 요청은 3초면 끝나고, 정합은 그 뒤에
        # 수십 초에 걸쳐 일어난다.
        self.ros.on_localized = self._localized
        # 주행 종료는 이 중에서도 가장 늦게 온다 — 목표를 받는 요청은 수십 ms,
        # 도착은 몇 분 뒤다.
        self.ros.on_nav_finished = self._nav_finished

    # ── 음성 문구 ───────────────────────────────────────────────────────
    # 약어는 띄어쓴다 — 붙여 쓰면 합성기가 뭉갠다 ("SLAM" → "슬램" 이 아니라 잡음).
    # 맵 이름은 읽지 않는다. "alm_lab" 이 "a l m underscore lab" 이 되어 방해만 된다.
    JOB_VOICE = {
        "pcd2pgm": ("Building two D map", "Two D map ready", "Two D map failed"),
        "fpfh": ("Building localization database",
                 "Localization database ready", "Localization database failed"),
    }

    # 슬롯이 **혼자 죽었을 때** 외치는 문구. 슬롯 이름을 그대로 읽히면
    # "localization" 이 뭉개지므로 표로 둔다 (약어는 띄어쓰기 규칙과 같은 이유).
    DIED_VOICE = {
        "slam": "S L A M process exited",
        "localization": "Localization process exited",
        "navigation": "Navigation process exited",
    }

    # 미션이 끝났을 때의 문구와 로그 등급. 성공과 실패를 귀로 구분해야 한다 —
    # 실차에서 화면을 못 볼 때 이게 유일한 신호다.
    NAV_END_VOICE = {
        "succeeded": ("Goal reached", "info"),
        "failed": ("Navigation failed", "warn"),
        "canceled": ("Navigation canceled", "info"),
        "paused": ("Navigation paused", "info"),
    }

    # 반납과 만료를 굳이 다른 문구로 나눈다. 반납은 조작자가 끝낸 것이고,
    # 만료는 **쥔 채로 연결이 끊긴** 것이다 — 귀로 구분돼야 한다.
    SESSION_VOICE = {
        "acquired": ("Control acquired", "info", "웹 제어권 획득"),
        "released": ("Control released", "info", "웹 제어권 반납"),
        "expired": ("Control timed out", "warn",
                    "웹 제어권 리스 만료 (하트비트 끊김)"),
    }

    def _session_changed(self, event, label):
        """SessionLock 이 부른다. 스레드는 정해져 있지 않다 — HTTP 핸들러일
        수도 있고, 만료를 감지한 주기 타이머일 수도 있다."""
        entry = self.SESSION_VOICE.get(event)
        if entry is None:
            return
        phrase, level, message = entry
        self._log(level, f"{message}: {label}" if label else message)
        # label 은 읽지 않는다 — 기본값이 접속 IP 라 자릿수를 하나씩 읽으면
        # 5초가 넘고, 정작 알고 싶은 정보도 아니다.
        #
        # E-STOP 처럼 priority 2 + interrupt 로 올리지 않는다. 제어권이 풀린다고
        # 로봇이 위험해지는 게 아니라 명령을 안 받을 뿐이고, 선점은 안전 사건
        # 전용으로 남겨둔다.
        self.ros.say(phrase, key="session")

    def _localized(self, result):
        """teaser_fpfh_localizer 가 /icp_result 를 냈다. rclpy 콜백 스레드다."""
        self._log("info",
                  f"측위 수렴: x={result['x']} y={result['y']} "
                  f"yaw={result['yaw_deg']}deg")
        # 실차에서 이게 가장 기다리는 소리다 — 초기위치가 잡혀야 주행을 시작한다.
        self.ros.say("Localization converged", key="localization")

    def _nav_finished(self, state, message, snapshot):
        """미션이 끝났다 (도달·중단·실패). rclpy 실행기 스레드다."""
        phrase, level = self.NAV_END_VOICE.get(state, ("Navigation ended", "info"))
        done = snapshot.get("index", 0)
        total = snapshot.get("total") or len(snapshot.get("points", []))
        self._log(level, f"자율주행 {state}: {message} ({done}/{total} 목표)")
        # 실패는 우선순위를 올린다. 로봇이 서 있는데 왜 서 있는지 모르는 시간이
        # 가장 위험하다 — 조작자가 다가가서 들여다보는 동안 다시 움직일 수도 있다.
        self.ros.say(phrase, priority=2 if state == "failed" else 1, key="navigation")

    def check_processes(self):
        """프로세스가 **혼자 죽은 것**을 잡아 알린다. 주기 타이머가 부른다.

        실차에서 가장 알고 싶은 사건이다 — 매핑 중에 FAST-LIO 가 죽으면 조작자는
        계속 돌고 있다고 믿으며 주행한다. 화면을 안 보면 알 방법이 없다.
        /api/health 는 브라우저가 열려 있을 때만 폴링되므로 여기에 의존할 수 없다.
        """
        for slot_info in self.processes.status():
            name = slot_info["slot"]
            slot = self.processes.slot(name)
            was = getattr(slot, "_voice_was_running", False)
            now = bool(slot_info["running"])
            slot._voice_was_running = now
            # stop() 을 거친 종료는 ProcessManager 가 표시한다. 그게 아닌데
            # 사라졌다면 스스로 죽은 것이다.
            if was and not now and not getattr(slot, "stop_requested", False):
                self.ros.say(self.DIED_VOICE.get(name, "Process exited"),
                             priority=2, key=f"died:{name}")
                self._log("warn", f"'{name}' 프로세스가 요청 없이 종료했습니다")

    def _job_finished(self, job):
        started, ok, failed = self.JOB_VOICE.get(job.kind, ("", "", ""))
        if not ok:
            return
        if job.state == "succeeded":
            self.ros.say(ok)
        elif job.state == "failed":
            self.ros.say(failed)
        # cancelled 는 조작자가 스스로 취소한 것이라 알릴 이유가 없다

    # ── 등록 ────────────────────────────────────────────────────────────
    def register(self, router):
        add = router.add
        add("GET", "/api/health", self.health)
        add("GET", "/api/limits", self.limits)

        add("GET", "/api/session", self.session_status)
        add("POST", "/api/session/acquire", self.session_acquire)
        add("POST", "/api/session/heartbeat", self.session_heartbeat)
        add("POST", "/api/session/release", self.session_release)

        # E-STOP: 락 예외 (위 도입부 참조)
        add("POST", "/api/estop", self.estop)
        add("POST", "/api/estop/release", self.estop_release, needs_lock=True)

        add("POST", "/api/mapping/start", self.mapping_start, needs_lock=True)
        add("POST", "/api/mapping/stop", self.mapping_stop, needs_lock=True)
        add("POST", "/api/mapping/save", self.mapping_save, needs_lock=True)

        add("GET", "/api/localization", self.localization_status)
        add("GET", "/api/localization/log", self.localization_log)
        add("POST", "/api/localization/start", self.localization_start, needs_lock=True)
        add("POST", "/api/localization/stop", self.localization_stop, needs_lock=True)

        add("GET", "/api/navigation", self.navigation_status)
        add("GET", "/api/navigation/log", self.navigation_log)
        add("POST", "/api/navigation/start", self.navigation_start, needs_lock=True)
        add("POST", "/api/navigation/stop", self.navigation_stop, needs_lock=True)
        add("POST", "/api/navigation/goal", self.navigation_goal, needs_lock=True)
        add("POST", "/api/navigation/pause", self.navigation_pause, needs_lock=True)
        add("POST", "/api/navigation/resume", self.navigation_resume, needs_lock=True)
        add("POST", "/api/navigation/cancel", self.navigation_cancel, needs_lock=True)

        add("GET", "/api/manual", self.manual_status)
        add("POST", "/api/manual/acquire", self.manual_acquire, needs_lock=True)
        add("POST", "/api/manual/release", self.manual_release, needs_lock=True)
        add("POST", "/api/manual/command", self.manual_command, needs_lock=True)
        add("POST", "/api/manual/stop", self.manual_stop, needs_lock=True)

        add("GET", "/api/jobs", self.job_list)
        add("GET", "/api/jobs/{id}", self.job_get)
        add("GET", "/api/jobs/{id}/stream", self.job_stream)
        add("POST", "/api/jobs/{id}/cancel", self.job_cancel, needs_lock=True)
        add("POST", "/api/jobs/pcd2pgm", self.job_pcd2pgm, needs_lock=True)
        add("POST", "/api/jobs/fpfh", self.job_fpfh, needs_lock=True)

        add("POST", "/api/maps", self.map_create, needs_lock=True)
        add("PUT", "/api/maps/active", self.map_set_active, needs_lock=True)

    # ── 상태 ────────────────────────────────────────────────────────────
    def health(self, _request):
        return {
            "ok": True,
            "uptime_sec": round(time.time() - self.ros.started_wall, 1),
            "maps_root": self.maps_root,
            "active_map": self.map_layout.active_map_name(self.maps_root),
            "session": self.session.status(),
            "processes": self.processes.status(),
            "map_save_available": self.ros.map_save_available(),
            # 슬롯 표에는 없지만 그래프에는 있는 측위 노드. CLI 로 먼저 띄운
            # 경우가 이것이고, 화면이 '측위 안 돌고 있음'이라고 말하면 안 된다.
            "localization_nodes": self.ros.running_nodes(self.LOCALIZATION_NODES),
            # 자율주행도 같은 이유로 그래프를 본다. 다만 여기서는 '떠 있는가'
            # 보다 '목표를 받을 수 있는가' 가 실질적인 질문이라 액션 준비 상태와
            # map->odom TF 를 함께 내려준다 — 화면이 시작 버튼을 살릴지 정하는
            # 근거가 이 둘이다.
            "navigation_nodes": self.ros.running_nodes(self.NAVIGATION_NODES),
            "navigation_ready": self.ros.nav_action_ready(),
            "localized": self.ros.nav_tf_ready(),
            "mapping_target": maps_write.read_mapping_target(self.fastlio_config),
            # 화면의 점군이 실측인지 재생본인지. 토픽 이름으로는 구분이 안 된다.
            "lidar_source": self.ros.lidar_source(),
        }

    def limits(self, _request):
        """command_manager 의 실제 속도 한계. UI 하드코딩을 없애기 위한 것."""
        try:
            return {"limits": self.ros.limits(), "source": "command_manager"}
        except RosTimeout as error:
            raise ApiError(503, str(error)) from error

    # ── 세션 ────────────────────────────────────────────────────────────
    def session_status(self, _request):
        return self.session.status()

    def session_acquire(self, request):
        # 로그와 음성은 SessionLock 의 on_change 가 낸다 (_session_changed).
        # 여기서 또 내면 만료 경로만 빠진 반쪽짜리가 되고, 언젠가 갈라진다.
        label = request.str_field("label", default="") or request.client
        session_id = self.session.acquire(label)
        if session_id is None:
            raise ApiError(409, "다른 접속자가 제어권을 쥐고 있습니다.")
        return {"session_id": session_id, **self.session.status()}

    def session_heartbeat(self, request):
        if not self.session.heartbeat(request.session_id):
            raise ApiError(409, "제어권이 만료되었거나 다른 접속자에게 넘어갔습니다.")
        return self.session.status()

    def session_release(self, request):
        released = self.session.release(request.session_id)
        return {"released": released, **self.session.status()}

    # ── E-STOP ──────────────────────────────────────────────────────────
    def estop(self, request):
        """정지는 락 없이도 가능하다 (의도적)."""
        self.ros.trigger_estop()
        self._log("warn", f"E-STOP 요청 (from {request.client})")
        return {"estop": True,
                "message": "정지 명령을 발행했습니다. /mcu/state 로 반영을 확인하세요."}

    def estop_release(self, request):
        reason = request.str_field("reason", default="") or f"web:{request.client}"
        try:
            result = self.ros.release_estop(reason)
        except RosTimeout as error:
            raise ApiError(503, str(error)) from error
        if not result["success"]:
            # 거부는 오류가 아니라 정상적인 안전 응답이다 — 사유를 그대로 올린다.
            raise ApiError(409, result["message"])
        return result

    # ── 매핑 ────────────────────────────────────────────────────────────
    def _map_paths(self, name):
        names = self.map_layout.list_map_names(self.maps_root)
        if name not in names:
            raise ApiError(404, f"맵 '{name}' 이 없습니다. "
                                f"(있는 맵: {', '.join(names) or '없음'})")
        return self.map_layout.map_paths(self.maps_root, name)

    def mapping_start(self, request):
        name = request.str_field("map", required=True)
        overwrite = request.bool_field("overwrite", False)
        paths = self._map_paths(name)

        try:
            maps_write.guard_overwrite(paths.cloud, overwrite)
        except maps_write.MapWriteError as error:
            raise ApiError(409, str(error)) from error

        # 시작 전에 이 맵의 산출물을 전부 비운다. 재매핑하면 cloud.pcd 만 덮이고
        # grid.pgm / fpfh_map* 는 옛 점군에서 만들어진 채 남아, 짝이 안 맞는
        # 자산이 한 폴더에 섞인다. 그 DB 로 측위를 돌리면 엉뚱한 곳에 수렴한다.
        try:
            cleared = maps_write.clear_map_assets(paths.path)
        except maps_write.MapWriteError as error:
            raise ApiError(500, str(error)) from error
        if cleared:
            self._log("warn", f"'{name}' 기존 자산 삭제: {', '.join(cleared)}")

        # ⚠ 순서가 중요하다. fast_lio 는 기동 시점에 map_file_path 를 읽으므로
        #   launch 를 띄우기 **전에** 바꿔야 한다. 저장 후 파일을 옮기는 방식은
        #   못 쓴다 — 그때는 이미 config 가 가리키던 남의 맵을 덮어쓴 뒤다.
        try:
            target = maps_write.set_mapping_target(self.fastlio_config, paths.cloud)
        except maps_write.MapWriteError as error:
            raise ApiError(500, str(error)) from error
        self._log("info", f"매핑 타깃 → {target}")

        # 접수 즉시 한 번. launch 기동은 몇 초 걸리므로, 이게 없으면 그동안
        # 버튼이 먹었는지 알 길이 없다.
        self.ros.say("Starting mapping", key="mapping")
        try:
            info = self.processes.start("slam")
        except ProcessError as error:
            self.ros.say("Mapping failed to start", key="mapping")
            raise ApiError(409, str(error)) from error
        self.ros.say("Mapping started", key="mapping")
        return {"map": name, "target": target, "cleared": cleared, "process": info}

    def mapping_stop(self, _request):
        # 종료는 SIGINT → 최대 15초다. 접수와 완료를 나눠 알린다.
        self.ros.say("Stopping mapping", key="mapping")
        try:
            info = self.processes.stop("slam")
        except ProcessError as error:
            raise ApiError(409, str(error)) from error
        self.ros.say("Mapping stopped", key="mapping")
        return {"process": info}

    def mapping_save(self, _request):
        """/map_save 호출. 어디에 저장되는지는 config 가 정한다 (mapping_start 참조).

        ⚠ 알려진 상류 함정: FAST-LIO 의 map_save_callback 은 누적 점군이 비어
        있으면 pcl::IOException 을 잡지 않아 **노드가 그대로 죽는다**(exit -6).
        서비스는 영영 응답하지 않으므로, 슬롯이 죽는 것을 보고 즉시 포기한다.
        """
        target = maps_write.read_mapping_target(self.fastlio_config)
        slot = self.processes.slot("slam")
        was_running = slot.is_running()

        def died():
            return was_running and not slot.is_running()

        # 큰 맵은 수십 초 걸린다. 저장이 시작됐다는 것부터 알린다.
        self.ros.say("Saving map", key="save")
        try:
            result = self.ros.map_save(abort_if=died)
        except RosTimeout as error:
            self.ros.say("Map save failed", key="save")
            if died():
                raise ApiError(500,
                               "저장 도중 매핑 노드가 종료했습니다. 누적된 점군이 "
                               "비어 있으면 FAST-LIO 가 이렇게 죽습니다 — 먼저 "
                               "주행해서 맵을 쌓았는지 확인하세요.") from error
            raise ApiError(503, f"{error} (FAST-LIO 가 떠 있는지 확인하세요)") from error
        if not result["success"]:
            self.ros.say("Map save failed", key="save")
            raise ApiError(500, result["message"] or "/map_save 가 실패를 보고했습니다.")
        self.ros.say("Map saved", key="save")
        # 저장된 파일의 사실(점 개수 등)은 여기서 지어내지 않는다.
        # map_manager 가 5초 안에 헤더를 읽어 /alm/map_inventory 로 알린다.
        return {"saved_to": target, "message": result["message"]}

    # ── 측위 ────────────────────────────────────────────────────────────
    # localization.launch.py 가 띄우는 노드들. 이름이 바뀌면 여기도 바꿔야 한다
    # (중복 기동 감지가 조용히 무력해지는 종류의 어긋남이다).
    LOCALIZATION_NODES = ("teaser_fpfh_localizer", "fastlio_localization")

    def localization_status(self, _request):
        slot = self.processes.slot("localization")
        return {
            "process": slot.info(),
            # 슬롯이 비어 있어도 그래프에 떠 있을 수 있다 (CLI 기동).
            "nodes": self.ros.running_nodes(self.LOCALIZATION_NODES),
            "active_map": self.map_layout.active_map_name(self.maps_root),
            # 이 슬롯이 어떤 맵으로 떠 있는지. args 를 그대로 되돌려주는 이유는
            # 기동 뒤에 활성 맵이 바뀌었을 수 있기 때문이다 — 지금 돌고 있는
            # 프로세스의 진실은 active.yaml 이 아니라 기동 인자에 있다.
            "result": self.ros.last_icp_result(),
        }

    def localization_log(self, request):
        """슬롯 로그 tail. /rosout 에 안 나오는 실패를 보기 위한 경로다.

        기동 자체가 실패하거나 노드가 맵을 읽다 죽으면 ROS 로그는 한 줄도
        안 남는다. 그때 화면의 /rosout 패널은 비어 있고, 조작자는 '멈춘 건지
        도는 건지'를 알 수 없다.
        """
        try:
            return self.processes.read_log("localization",
                                           since=request.int_query("since", 0))
        except ProcessError as error:
            raise ApiError(404, str(error)) from error

    def localization_start(self, request):
        """활성 맵으로 측위 스택을 띄운다.

        맵을 인자로 안 주면 active.yaml 을 따른다. 다만 **여기서 해석한
        절대경로를 launch 인자로 명시 전달**한다 — launch 가 스스로 active.yaml 을
        다시 읽게 두면, 요청과 파싱 사이에 활성 맵이 바뀌었을 때 화면이 보고한
        맵과 실제로 뜬 맵이 갈라진다.
        """
        name = (request.str_field("map", default="")
                or self.map_layout.active_map_name(self.maps_root))
        if not name:
            raise ApiError(409, "활성 맵이 없습니다. 맵을 먼저 만들고 선택하세요.")
        paths = self._map_paths(name)

        # 기동 전 점검. 실패는 전부 409 — 서버 오류가 아니라 '지금은 못 한다'다.
        try:
            summary = localization.check_assets(paths)
            lidar_note = localization.check_lidar(self.ros.lidar_source())
        except localization.PreflightError as error:
            raise ApiError(409, str(error)) from error

        conflict = self.processes.conflicting("localization")
        if conflict:
            raise ApiError(409, f"'{conflict}' 가 실행 중입니다. 먼저 종료하세요 — "
                                f"FAST-LIO 가 두 개 뜨면 /Odometry 와 TF 가 겹칩니다.")

        # 슬롯 표만으로는 부족하다. CLI 로 먼저 띄워놓고 웹을 여는 경우 슬롯은
        # 비어 있어서 여기까지 통과하고, 그러면 fast_lio 두 개가 odom→base_link
        # 를 동시에 내는 상태가 만들어진다 — 로그는 양쪽 다 깨끗하다.
        already = self.ros.running_nodes(self.LOCALIZATION_NODES)
        if already:
            raise ApiError(409,
                           f"측위 노드가 이미 떠 있습니다 ({', '.join(already)}). "
                           f"웹 밖에서 기동한 것이라면 그 터미널에서 먼저 종료하세요 "
                           f"— FAST-LIO 가 두 개면 /Odometry 와 TF 가 겹칩니다.")

        accum = max(1, min(60, request.int_field("accum_frames", 10)))
        launch_args = [
            f"map_pcd:={paths.cloud}",
            f"fpfh_db_prefix:={paths.fpfh_prefix}",
            "auto_init:=true",
            f"accum_frames:={accum}",
        ]
        # DB 가 자기 전처리 파라미터를 들고 다닌다. 화면에서 voxel 을 바꿔 DB 를
        # 다시 만들어도 localizer 가 자동으로 따라온다 — 이게 없으면 DB 는 0.3,
        # localizer 는 0.5 로 도는 조용한 고장이 난다.
        launch_args += [f"{name}:={value}"
                        for name, value in sorted(summary["params"].items())]

        # 이전 기동의 성공이 남아 있으면 화면이 '이미 수렴했다'고 거짓말한다.
        self.ros.clear_icp_result()

        self.ros.say("Starting localization", key="localization")
        try:
            info = self.processes.start("localization", launch_args)
        except ProcessError as error:
            self.ros.say("Localization failed to start", key="localization")
            raise ApiError(409, str(error)) from error
        self.ros.say("Localization started", key="localization")
        self._log("info", f"측위 기동: 맵 '{name}' "
                          f"(feature {summary['db_features']}개, "
                          f"{summary['cloud_points']:,}점, {accum}프레임 누적)")

        notes = [note for note in (summary["warning"], lidar_note) if note]
        return {"map": name, "process": info, "summary": summary,
                "accum_frames": accum, "notes": notes,
                "message": "로봇을 정지 상태로 두세요 — 정합 전에 라이다 "
                           f"{accum}프레임을 누적합니다."}

    def localization_stop(self, _request):
        try:
            info = self.processes.stop("localization")
        except ProcessError as error:
            raise ApiError(409, str(error)) from error

        # 슬롯이 비어 있었다면 **아무것도 안 내린 것**이다. 그런데도 "종료했다"고
        # 말하면 조작자는 측위가 멈춘 줄 안다 — CLI 로 띄운 스택이 그대로 도는
        # 채로. 그건 화면이 하는 거짓말 중 가장 나쁜 종류다.
        if info.get("already_stopped"):
            leftover = self.ros.running_nodes(self.LOCALIZATION_NODES)
            if leftover:
                raise ApiError(409,
                               f"웹이 띄운 측위가 없습니다. 지금 도는 것은 웹 밖에서 "
                               f"기동한 것입니다 ({', '.join(leftover)}) — 그 터미널에서 "
                               f"Ctrl-C 하세요.")
            return {"process": info, "message": "실행 중인 측위가 없습니다."}

        self.ros.clear_icp_result()
        self.ros.say("Localization stopped", key="localization")
        return {"process": info}

    # ── 자율주행 ────────────────────────────────────────────────────────
    # navigation.launch.py 가 띄우는 것 중 **이 스택에만 있는** 노드.
    # 측위 노드(teaser/fastlio)는 여기 넣지 않는다 — 넣으면 측위만 띄운
    # 상태를 '자율주행 중' 으로 오독한다.
    NAVIGATION_NODES = ("bt_navigator", "controller_server", "planner_server",
                        "smoother_server", "behavior_server", "waypoint_follower")

    def navigation_status(self, _request):
        slot = self.processes.slot("navigation")
        return {
            "process": slot.info(),
            "nodes": self.ros.running_nodes(self.NAVIGATION_NODES),
            "mission": self.ros.nav_status(),
            "active_map": self.map_layout.active_map_name(self.maps_root),
        }

    def navigation_log(self, request):
        try:
            return self.processes.read_log("navigation",
                                           since=request.int_query("since", 0))
        except ProcessError as error:
            raise ApiError(404, str(error)) from error

    def navigation_start(self, request):
        """자율주행 스택(map_server + 측위 + Nav2)을 띄운다.

        측위 기동과 사전 점검이 같은 이유로 같은 모양이다 — 이 launch 는
        localization.launch.py 를 **포함하므로**, 맵 자산이 안 맞으면 Nav2 까지
        올라간 뒤에야 정합이 조용히 안 붙는다.
        """
        name = (request.str_field("map", default="")
                or self.map_layout.active_map_name(self.maps_root))
        if not name:
            raise ApiError(409, "활성 맵이 없습니다. 맵을 먼저 만들고 선택하세요.")
        paths = self._map_paths(name)

        if not os.path.isfile(paths.grid_yaml):
            raise ApiError(409,
                           f"'{name}' 에 2D 격자(grid.yaml)가 없습니다 — Nav2 의 "
                           f"global costmap 이 이걸 씁니다. '2D 맵 생성' 을 "
                           f"먼저 실행하세요.")
        try:
            summary = localization.check_assets(paths)
            lidar_note = localization.check_lidar(self.ros.lidar_source())
        except localization.PreflightError as error:
            raise ApiError(409, str(error)) from error

        conflict = self.processes.conflicting("navigation")
        if conflict:
            raise ApiError(409, f"'{conflict}' 가 실행 중입니다. 먼저 종료하세요 — "
                                f"자율주행 스택은 측위를 안에 포함하므로 "
                                f"FAST-LIO 가 두 개 뜹니다.")
        already = self.ros.running_nodes(self.LOCALIZATION_NODES
                                         + self.NAVIGATION_NODES)
        if already:
            raise ApiError(409,
                           f"측위/자율주행 노드가 이미 떠 있습니다 "
                           f"({', '.join(already)}). 웹 밖에서 기동한 것이라면 "
                           f"그 터미널에서 먼저 종료하세요.")

        accum = max(1, min(60, request.int_field("accum_frames", 10)))
        launch_args = [
            f"map:={paths.grid_yaml}",
            f"map_pcd:={paths.cloud}",
            f"fpfh_db_prefix:={paths.fpfh_prefix}",
            f"accum_frames:={accum}",
        ]
        # 측위와 같은 이유로 DB 의 전처리 파라미터를 그대로 물려준다
        # (localization_start 주석 참고).
        launch_args += [f"{key}:={value}"
                        for key, value in sorted(summary["params"].items())]

        self.ros.clear_icp_result()
        self.ros.say("Starting navigation", key="navigation")
        try:
            info = self.processes.start("navigation", launch_args)
        except ProcessError as error:
            self.ros.say("Navigation failed to start", key="navigation")
            raise ApiError(409, str(error)) from error

        self._log("info", f"자율주행 기동: 맵 '{name}' (격자 {os.path.basename(paths.grid_yaml)}, "
                          f"feature {summary['db_features']}개, {accum}프레임 누적)")
        notes = [note for note in (summary["warning"], lidar_note) if note]
        return {"map": name, "process": info, "summary": summary,
                "accum_frames": accum, "notes": notes,
                "message": "로봇을 정지 상태로 두세요 — 초기 정합이 끝나야 "
                           "목표를 보낼 수 있습니다."}

    def navigation_stop(self, _request):
        # 미션이 돌고 있으면 먼저 세운다. 프로세스만 내리면 목표를 받은 채로
        # 죽는 것이라, 마지막 /cmd_vel 이 그대로 남는 상황을 만들 수 있다.
        # (command_manager 의 cmd_timeout 0.5 s 가 결국 세우긴 하지만, 세우는
        #  주체가 '타임아웃' 인 것과 '취소' 인 것은 다르다.)
        if self.ros.nav_busy():
            try:
                self.ros.nav_cancel(keep=False)
            except NavRejected as error:
                self._log("warn", f"종료 전 미션 취소 실패: {error}")

        try:
            info = self.processes.stop("navigation")
        except ProcessError as error:
            raise ApiError(409, str(error)) from error

        if info.get("already_stopped"):
            leftover = self.ros.running_nodes(self.NAVIGATION_NODES)
            if leftover:
                raise ApiError(409,
                               f"웹이 띄운 자율주행이 없습니다. 지금 도는 것은 웹 "
                               f"밖에서 기동한 것입니다 ({', '.join(leftover)}) — "
                               f"그 터미널에서 Ctrl-C 하세요.")
            return {"process": info, "message": "실행 중인 자율주행이 없습니다."}

        # force — Nav2 를 방금 죽였으므로 결과 future 는 안 온다 (nav_clear 주석)
        self.ros.nav_clear(force=True)
        self.ros.clear_icp_result()
        self.ros.say("Navigation stopped", key="navigation")
        return {"process": info}

    def _nav_preflight(self):
        try:
            navigation.check_stack(
                action_ready=self.ros.nav_action_ready(),
                tf_ready=self.ros.nav_tf_ready(),
                slot_running=self.processes.slot("navigation").is_running(),
                external_nodes=self.ros.running_nodes(self.LOCALIZATION_NODES))
        except navigation.PreflightError as error:
            raise ApiError(409, str(error)) from error

    def navigation_goal(self, request):
        """목표를 보낸다. points 하나면 단일 목표, 여럿이면 웨이포인트 미션."""
        self._nav_preflight()
        try:
            points = navigation.parse_points(request.body.get("points"))
        except navigation.PreflightError as error:
            raise ApiError(400, str(error)) from error

        if self.ros.nav_busy():
            raise ApiError(409, "이미 주행 중입니다. 중단하거나 일시정지한 뒤 "
                                "새 목표를 보내세요.")

        try:
            status = self.ros.nav_send(points)
        except NavRejected as error:
            self._log("warn", f"목표 거부: {error}")
            raise ApiError(409, str(error)) from error

        summary = navigation.describe(points)
        self._log("info", f"자율주행 목표 전송: {summary} "
                          f"(직선 {status['distance_estimate_m']} m)")
        self.ros.say("Goal accepted", key="navigation")
        return {"mission": status, "message": f"목표를 전송했습니다 — {summary}"}

    def navigation_pause(self, _request):
        """현재 목표를 취소하되 남은 웨이포인트는 남긴다.

        Nav2 에는 일시정지가 없다. 그래서 '취소하고 남은 목록을 기억' 이
        일시정지의 실제 구현이고, resume 은 남은 목록으로 **새 목표를 보내는**
        것이다. 재개하면 경로를 처음부터 다시 계획한다 — 세운 자리에서
        이어붙이는 것이 아니다.
        """
        try:
            status = self.ros.nav_cancel(keep=True)
        except NavRejected as error:
            raise ApiError(409, str(error)) from error
        remaining = len(status["remaining"])
        self._log("info", f"자율주행 일시정지 — 남은 목표 {remaining}개")
        return {"mission": status,
                "message": f"주행을 세웠습니다. 남은 목표 {remaining}개를 유지합니다."}

    def navigation_resume(self, _request):
        self._nav_preflight()
        status = self.ros.nav_status()
        if status["state"] != "paused":
            raise ApiError(409, "일시정지 상태가 아닙니다.")
        remaining = status["remaining"]
        if not remaining:
            raise ApiError(409, "남은 목표가 없습니다.")

        try:
            status = self.ros.nav_send(remaining, start_index=status["index"],
                                       resume=True)
        except NavRejected as error:
            raise ApiError(409, str(error)) from error
        self._log("info", f"자율주행 재개 — 남은 목표 {len(remaining)}개")
        self.ros.say("Resuming navigation", key="navigation")
        return {"mission": status,
                "message": f"남은 목표 {len(remaining)}개로 새 경로를 계획합니다."}

    def navigation_cancel(self, _request):
        """미션을 폐기한다. 일시정지와 달리 남은 목표를 버린다."""
        if not self.ros.nav_busy():
            # 이미 끝났거나 세워진 미션을 지우는 것도 '중단' 이다. 화면의
            # 중단 버튼이 상태에 따라 다른 오류를 뱉으면 조작자만 헷갈린다.
            status = self.ros.nav_clear()
            return {"mission": status, "message": "진행 중인 미션이 없습니다."}
        try:
            status = self.ros.nav_cancel(keep=False)
        except NavRejected as error:
            raise ApiError(409, str(error)) from error
        # 여기서 nav_clear() 로 지우지 않는다. 결과 콜백이 아직 안 왔을 수 있고
        # (nav_cancel 은 2초까지만 기다린다), 무엇보다 'canceled' 라는 사실이
        # 화면에 한 번은 보여야 한다. 다음 목표를 보내면 그때 덮인다.
        self._log("warn", "자율주행 미션 중단")
        return {"mission": status, "message": "미션을 중단했습니다."}

    # ── 수동주행 (직접 rpm/조향각) ──────────────────────────────────────
    #
    # 이 경로는 twist 를 쓰지 않는다. 이유와 안전 설계는 alm_msgs/DirectDrive.msg
    # 와 command_manager._tick_direct 의 docstring 에 있다. 요약하면:
    # 변환 상수가 미확정이라 twist 로는 무엇이 나가는지 모르고, 그 상수를
    # **측정하려면** 이 경로가 필요하다.
    #
    # 여기(backend)는 얇게 유지한다. 속도/각도 한계, 슬루, E-STOP, 타임아웃은
    # 전부 command_manager 가 본다 — 두 곳에서 자르면 어느 쪽이 잘랐는지
    # 화면에서 알 수 없게 된다. 여기서 하는 것은 **형식 검증**뿐이다.
    MANUAL_MODES = (0, 1, 3, 4)

    def manual_status(self, _request):
        status = self.ros.manual_status()
        try:
            limits = self.ros.limits()
        except RosTimeout:
            limits = {}
        return {**status, "limits": limits}

    def manual_acquire(self, _request):
        """동작권을 web 으로 가져온다. 이때부터 직접명령이 통과한다."""
        try:
            result = self.ros.set_owner("web")
        except RosTimeout as error:
            raise ApiError(503, str(error)) from error
        if not result["success"]:
            raise ApiError(409, result["message"])
        self._log("warn", "웹 수동주행 동작권 획득 — 직접 rpm/조향각 명령이 활성화됩니다")
        self.ros.say("Manual control active", priority=2, key="manual")
        return {**result, **self.ros.manual_status()}

    def manual_release(self, _request):
        # 스트림을 먼저 끊고 동작권을 넘긴다. 순서가 반대면 자율로 넘어간
        # 직후에 남은 직접명령이 한 틱 더 나갈 수 있다.
        self.ros.manual_release()
        try:
            result = self.ros.set_owner("auto")
        except RosTimeout as error:
            raise ApiError(503, str(error)) from error
        self._log("info", "웹 수동주행 동작권 반납 -> auto")
        self.ros.say("Manual control released", key="manual")
        return {**result, **self.ros.manual_status()}

    def manual_command(self, request):
        """rpm + 조향각 + 모드. 브라우저가 이걸 주기적으로 보내는 것이 데드맨이다."""
        owner = self.ros.owner()
        if not owner.startswith("web"):
            raise ApiError(409,
                           f"동작권이 web 이 아닙니다 (현재 '{owner or '알 수 없음'}'). "
                           f"'수동 조작 시작' 을 먼저 누르세요.")
        mode_id = request.int_field("mode_id", 1)
        if mode_id not in self.MANUAL_MODES:
            raise ApiError(400, f"mode_id 는 {self.MANUAL_MODES} 중 하나여야 합니다.")
        speed_rpm = request.float_field("speed_rpm", 0.0)
        steer_deg = request.float_field("steer_deg", 0.0)
        for name, value in (("speed_rpm", speed_rpm), ("steer_deg", steer_deg)):
            if not math.isfinite(value):
                raise ApiError(400, f"'{name}' 이 유한한 값이 아닙니다.")
        # 여기서 한계로 자르지 않는다 — command_manager 가 자른다(위 주석).
        # 다만 터무니없는 값은 형식 오류로 본다.
        if abs(speed_rpm) > 100000 or abs(steer_deg) > 360:
            raise ApiError(400, "값의 자릿수가 비정상입니다 — 입력을 확인하세요.")
        return self.ros.manual_command(speed_rpm, steer_deg, mode_id)

    def manual_stop(self, _request):
        """구동만 세운다. 동작권은 유지 — 다시 밀면 바로 간다."""
        status = self.ros.manual_stop()
        self._log("info", "웹 수동주행 정지 (동작권 유지)")
        return status

    # ── 작업 (subprocess) ───────────────────────────────────────────────
    def job_pcd2pgm(self, request):
        name = request.str_field("map", required=True)
        paths = self._map_paths(name)
        if not os.path.isfile(paths.cloud):
            raise ApiError(409, f"'{name}' 에 cloud.pcd 가 없습니다. 먼저 3D 맵을 저장하세요.")

        argv = [
            self.exe["pcd2pgm"],
            "--pcd", paths.cloud,
            "--out", os.path.join(paths.path, "grid"),
            "--resolution", str(_clamp("resolution", request.float_field("resolution", 0.05))),
            "--min-points", str(_clamp("min_points", request.int_field("min_points", 2))),
        ]

        # ---- 높이 밴드 (2026-08-25 수정) ----
        # ##왜 이렇게 바뀌었나## 예전에는 --z-min/--z-max 를 **무조건** 넘겼다.
        #   그런데 pcd2pgm 은 이 인자가 하나라도 오면 절대 z 를 쓰는 **호환
        #   모드로 떨어진다.** 즉 지면 자동추정이 웹 경로에서 통째로 죽어 있었다.
        #   기본값 -0.3 의 정당성은 '라이다 마운트 0.5 m' 가정에 전부 걸려
        #   있는데 그 TF 는 아직 추정값이다(docs/TODO.md). 마운트가 0.7 m 면
        #   밴드 하한이 지면 위 0.4 m 가 되어 그보다 낮은 턱·박스가 전부
        #   자유공간으로 찍힌다. 미관측이 아니라 **가짜 자유공간**이라 더 나쁘다.
        #
        # 이제 절대 z 는 **요청에 명시적으로 있을 때만** 넘긴다. 없으면
        # pcd2pgm 이 지면을 자동 추정하고 지면 기준 밴드를 쓴다.
        has_abs = ("z_min" in request.body) or ("z_max" in request.body)
        if has_abs:
            z_min = _clamp("z_min", request.float_field("z_min", -0.3))
            z_max = _clamp("z_max", request.float_field("z_max", 1.5))
            if z_max <= z_min:
                raise ApiError(400, "z_max 는 z_min 보다 커야 합니다.")
            argv += ["--z-min", str(z_min), "--z-max", str(z_max)]
            self._log("warn",
                      f"'{name}' 2D 변환에 절대 z 밴드 [{z_min}, {z_max}] 가 지정됐습니다 — "
                      f"지면 기준이 아니므로 밴드보다 낮은 장애물(턱·박스)이 "
                      f"자유공간으로 찍힙니다. 라이다 마운트 높이가 확정되기 "
                      f"전에는 비워 두고 자동 추정을 쓰세요.")
        else:
            lo = _clamp("obstacle_min_h", request.float_field("obstacle_min_h", 0.15))
            hi = _clamp("obstacle_max_h", request.float_field("obstacle_max_h", 1.80))
            if hi <= lo:
                raise ApiError(400, "obstacle_max_h 는 obstacle_min_h 보다 커야 합니다.")
            argv += ["--obstacle-min-h", str(lo), "--obstacle-max-h", str(hi)]
            if "ground_z" in request.body:
                argv += ["--ground-z",
                         str(_clamp("ground_z", request.float_field("ground_z", 0.0)))]
        # 레이캐스팅 입력. scan_recorder 가 매핑 중에 남긴 것이며, 있으면 반드시
        # 넘겨야 한다 — 빼면 '점이 찍힌 셀만 자유공간' 인 옛 투영 방식으로
        # 조용히 떨어지고, 격자의 8할 이상이 미관측으로 남는다(실측 cschool
        # 87.9%). 플래너가 allow_unknown:false 라 그런 맵에서는 대부분의 목표에서
        # 계획이 실패한다. 로그도 깨끗해서 격자를 열어보기 전까지 모른다.
        if os.path.isfile(paths.scans):
            argv += ["--scans", paths.scans]
        else:
            self._log("warn",
                      f"'{name}' 에 scans.npz 가 없어 투영 방식으로 2D 맵을 만듭니다 — "
                      f"자유공간이 크게 과소평가됩니다. scan_recorder 를 켠 채"
                      f"(slam.launch.py record:=true, 기본값) 다시 매핑하세요.")
        return self._start_job("pcd2pgm", argv, name)

    def job_fpfh(self, request):
        name = request.str_field("map", required=True)
        paths = self._map_paths(name)
        if not os.path.isfile(paths.cloud):
            raise ApiError(409, f"'{name}' 에 cloud.pcd 가 없습니다. 먼저 3D 맵을 저장하세요.")

        argv = [
            self.exe["fpfh_map_builder"],
            "--map", paths.cloud,
            "--output-prefix", paths.fpfh_prefix,
            "--voxel", str(_clamp("voxel", request.float_field("voxel", 0.5))),
            "--normal-radius",
            str(_clamp("normal_radius", request.float_field("normal_radius", 1.0))),
            "--feature-radius",
            str(_clamp("feature_radius", request.float_field("feature_radius", 2.5))),
            "--z-min", str(_clamp("z_min", request.float_field("z_min", -0.35))),
            "--z-max", str(_clamp("z_max", request.float_field("z_max", 1.0))),
        ]
        return self._start_job("fpfh", argv, name)

    def _start_job(self, kind, argv, map_name):
        if not os.path.isfile(argv[0]):
            raise ApiError(500, f"실행 파일을 찾을 수 없습니다: {argv[0]} "
                                f"(colcon build 를 다시 하세요)")
        try:
            job = self.jobs.start(kind, argv, map_name=map_name)
        except JobError as error:
            raise ApiError(409, str(error)) from error
        # 잡은 수십 초가 걸린다. 접수됐다는 것부터 알려야 조작자가 기다릴 수 있다.
        started = self.JOB_VOICE.get(kind, ("",))[0]
        if started:
            self.ros.say(started)
        return job.snapshot(since=0)

    def job_list(self, _request):
        return {"jobs": self.jobs.list()}

    def job_get(self, request):
        try:
            job = self.jobs.get(request.params["id"])
        except JobError as error:
            raise ApiError(404, str(error)) from error
        return job.snapshot(since=request.int_query("since", 0))

    def job_cancel(self, request):
        try:
            job = self.jobs.get(request.params["id"])
        except JobError as error:
            raise ApiError(404, str(error)) from error
        return {"cancelled": job.cancel(), **job.snapshot(since=10 ** 9)}

    def job_stream(self, request):
        """SSE. UI 는 폴링(?since=)을 쓰고, 이건 curl -N 디버깅용이다."""
        try:
            job = self.jobs.get(request.params["id"])
        except JobError as error:
            raise ApiError(404, str(error)) from error

        def generate():
            seen = request.int_query("since", 0)
            state = ""
            while True:
                snapshot = job.snapshot(since=seen)
                if snapshot["lines"] or snapshot["state"] != state:
                    yield "progress", snapshot
                    seen = snapshot["line_count"]
                    state = snapshot["state"]
                if state in ("succeeded", "failed", "cancelled"):
                    yield "done", job.snapshot(since=10 ** 9)
                    return
                job.wait_for_change(seen, state, timeout=5.0)

        return StreamResponse(generate())

    # ── 맵 쓰기 ─────────────────────────────────────────────────────────
    def map_create(self, request):
        name = request.str_field("name", required=True)
        try:
            created = maps_write.create_map(
                self.maps_root, name,
                label=request.str_field("label", default="") or "",
                notes=request.str_field("notes", default="") or "")
        except maps_write.MapWriteError as error:
            raise ApiError(409, str(error)) from error
        self._log("info", f"맵 생성: {created['path']}")
        # 목록은 map_manager 가 5초 안에 다시 스캔해 /alm/map_inventory 로 알린다.
        return created

    def map_set_active(self, request):
        name = request.str_field("name", required=True)
        try:
            active = maps_write.set_active_map(
                self.maps_root, name, self.map_layout.list_map_names(self.maps_root))
        except maps_write.MapWriteError as error:
            raise ApiError(409, str(error)) from error
        self._log("info", f"활성 맵 → {active}")

        # 매핑 타깃도 같이 옮긴다. 활성 맵을 고른다는 것은 "지금 이 맵으로
        # 작업한다"는 뜻인데, /map_save 목적지만 예전 맵에 남아 있으면 저장이
        # 엉뚱한 폴더로 떨어진다. 선택은 하나여야 한다.
        target, target_note = self._follow_mapping_target(active)

        # 무엇이 즉시 따라오고 무엇이 안 따라오는지 구분해서 알린다.
        # "반영 안 됨"으로 뭉뚱그리면 조작자가 화면을 못 믿게 된다.
        return {
            "active": active,
            "mapping_target": target,
            "follows": ["map_publisher(/map)", "prior_cloud_publisher(/alm/prior_cloud)",
                        "map_manager(/alm/map_inventory)", "fastlio map_file_path"],
            "note": "2D·3D 저장 맵은 몇 초 안에 따라옵니다. "
                    "이미 실행 중인 SLAM/측위/Nav2 프로세스는 다음 기동부터 반영됩니다."
                    + (f" {target_note}" if target_note else ""),
        }

    def _follow_mapping_target(self, name):
        """활성 맵을 따라 fastlio 의 map_file_path 를 옮긴다.

        SLAM 이 도는 중이면 건드리지 않는다. fast_lio 는 기동 시점에 그 값을
        읽어 메모리에 들고 있으므로, 지금 파일을 고쳐 봐야 실제 저장 위치는
        안 바뀐다. 파일만 바꾸면 화면이 보고하는 목적지와 실제 목적지가
        갈라져서, 저장한 뒤에야 어긋난 걸 알게 된다.
        """
        if self.processes.slot("slam").is_running():
            current = maps_write.read_mapping_target(self.fastlio_config)
            self._log("warn", f"SLAM 실행 중 — 매핑 타깃은 {current} 로 유지")
            return current, ("SLAM 이 실행 중이라 저장 위치는 바뀌지 않습니다 "
                             "— 지금 세션은 시작할 때 정한 맵에 저장됩니다.")
        try:
            paths = self.map_layout.map_paths(self.maps_root, name)
            target = maps_write.set_mapping_target(self.fastlio_config, paths.cloud)
        except maps_write.MapWriteError as error:
            # 활성 맵 전환 자체는 이미 성공했다. 여기서 500 을 던지면 조작자는
            # 전환이 통째로 실패한 줄 안다 — 사실과 다르다.
            self._log("error", f"매핑 타깃 갱신 실패: {error}")
            return "", f"매핑 타깃 갱신에 실패했습니다: {error}"
        self._log("info", f"매핑 타깃 → {target}")
        return target, ""

    # ── 잡동사니 ────────────────────────────────────────────────────────
    def _log(self, level, text):
        log(self.logger, level, text)
