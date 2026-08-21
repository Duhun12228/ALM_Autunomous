"""rclpy 쪽 접점 — 토픽 발행과 서비스 호출만.

스레딩 규약이 이 파일의 핵심이다.

    메인 스레드 : MultiThreadedExecutor.spin()  (rclpy 콜백이 도는 곳)
    HTTP 스레드 : ThreadingHTTPServer 의 요청 핸들러 → 여기 메서드를 부른다

그래서 HTTP 스레드에서 spin_until_future_complete() 를 부르면 안 된다 —
같은 노드를 두 곳에서 spin 하게 되어 콜백이 유실되거나 데드락이 난다.
대신 call_async() 로 던지고 future.done() 을 폴링한다. 실행기는 메인 스레드가
계속 돌리고 있으므로 결과는 정상적으로 채워진다.

서비스 클라이언트는 ReentrantCallbackGroup 에 둔다. 기본
MutuallyExclusiveCallbackGroup 이면 요청 두 개가 겹쳤을 때 뒤엣것이 앞엣것의
응답을 기다리다 함께 멈춘다.
"""

import math
import threading
import time

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from alm_msgs.msg import Speech
from alm_msgs.srv import ReleaseEstop


class RosTimeout(Exception):
    """서비스가 시간 안에 응답하지 않음 (노드가 없거나 멈춰 있음)."""


class RosInterface(Node):
    def __init__(self):
        super().__init__("alm_web_backend")
        self._group = ReentrantCallbackGroup()
        self.started_wall = time.time()

        # E-STOP 은 놓치면 안 되는 명령이다. 늦게 뜨는 구독자(command_manager
        # 재시작 등)도 마지막 값을 받도록 TRANSIENT_LOCAL 로 둔다.
        estop_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._estop_pub = self.create_publisher(Bool, "/emergency_stop", estop_qos)

        self._release_cli = self.create_client(
            ReleaseEstop, "/emergency_stop/release", callback_group=self._group)
        self._map_save_cli = self.create_client(
            Trigger, "/map_save", callback_group=self._group)
        self._limits_cli = self.create_client(
            GetParameters, "/command_manager/get_parameters", callback_group=self._group)

        # 음성 안내. 실차에서는 화면을 못 보므로 명령이 닿았는지 귀로 확인한다.
        # 구독자(voice_announcer)가 없어도 발행은 그냥 버려진다 — 확인하지 않는다.
        self._say_pub = self.create_publisher(Speech, "/alm/say", 10)

        # 측위 수렴. HTTP 요청이 이미 끝난 **뒤에** 오는 사건이라 구독이 필요하다
        # (잡 완료 콜백과 같은 이유). teaser_fpfh_localizer 가 성공했을 때 딱
        # 한 번 낸다 — 그 뒤로는 스스로 유휴 상태가 되므로 재발행도 없다.
        self._icp_lock = threading.Lock()
        self._last_icp = None
        self.on_localized = None
        self._icp_sub = self.create_subscription(
            PoseWithCovarianceStamped, "/icp_result", self._on_icp_result, 10,
            callback_group=self._group)

        self._limits_cache = None
        self._limits_cache_at = 0.0

    # ── 음성 안내 ───────────────────────────────────────────────────────
    def say(self, text, priority=Speech.PRIORITY_NORMAL, key="", interrupt=False):
        """발화 요청. **절대 예외를 밖으로 내보내지 않는다.**

        음성은 부가 기능이다. 이것 때문에 API 응답이 실패하거나 늦어지면
        본말이 뒤집힌다. E-STOP 은 voice_announcer 가 /emergency_stop 을 직접
        구독해 알리므로 여기서 보내지 않는다 (중복 방지).
        """
        try:
            msg = Speech()
            msg.text = text
            msg.priority = int(priority)
            msg.key = key
            msg.interrupt = bool(interrupt)
            self._say_pub.publish(msg)
        except Exception:                                # noqa: BLE001
            pass

    # ── 측위 결과 ───────────────────────────────────────────────────────
    def _on_icp_result(self, msg):
        pose = msg.pose.pose
        q = pose.orientation
        # yaw 만 뽑는다. roll/pitch 는 화면에서 쓸 데가 없고, 평면 위 로봇에서는
        # 사실상 0 이다.
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        result = {
            "x": round(pose.position.x, 3),
            "y": round(pose.position.y, 3),
            "z": round(pose.position.z, 3),
            "yaw_deg": round(math.degrees(yaw), 1),
            "frame_id": msg.header.frame_id,
            "stamp_wall": time.time(),
        }
        with self._icp_lock:
            self._last_icp = result

        # 콜백은 **락 밖에서** 부른다 (session.py 의 _emit 과 같은 규약).
        # 여기서 발화까지 하므로 뮤텍스를 쥔 채로 부르면 오디오 경로에 묶인다.
        callback = self.on_localized
        if callback is None:
            return
        try:
            callback(result)
        except Exception:                                # noqa: BLE001
            pass

    def last_icp_result(self):
        with self._icp_lock:
            return dict(self._last_icp) if self._last_icp else None

    def clear_icp_result(self):
        """측위를 새로 시작할 때 부른다. 안 지우면 이전 기동의 성공이 남아
        있어서 화면이 '이미 수렴했다'고 거짓말한다."""
        with self._icp_lock:
            self._last_icp = None

    # ── 공통 ────────────────────────────────────────────────────────────
    def _call(self, client, request, timeout=5.0, what="", abort_if=None):
        """HTTP 스레드에서 안전하게 서비스를 부른다 (§파일 도입부 참조).

        abort_if 는 "기다려봐야 소용없는 상황"을 알려주는 술어다. 서비스를
        제공하던 프로세스가 죽으면 future 는 영원히 done() 이 되지 않으므로,
        타임아웃 전체를 기다리는 대신 즉시 포기한다. (FAST-LIO 는 빈 맵을
        저장하려 하면 pcl::IOException 을 안 잡고 죽는다 — 그 상황이 이것이다.)
        """
        if not client.service_is_ready():
            # 서비스가 아직 광고 안 됐을 수 있으니 잠깐 기다린다.
            deadline = time.monotonic() + min(timeout, 2.0)
            while time.monotonic() < deadline and not client.service_is_ready():
                time.sleep(0.05)
        if not client.service_is_ready():
            raise RosTimeout(f"{what or client.srv_name} 서비스를 찾을 수 없습니다. "
                             f"해당 노드가 떠 있는지 확인하세요.")

        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            if abort_if is not None and abort_if():
                future.cancel()
                raise RosTimeout(
                    f"{what or client.srv_name} 를 제공하던 프로세스가 응답 도중 "
                    f"종료했습니다.")
            time.sleep(0.02)
        future.cancel()
        raise RosTimeout(f"{what or client.srv_name} 응답이 {timeout:.0f}초 안에 오지 않았습니다.")

    # ── E-STOP ──────────────────────────────────────────────────────────
    def trigger_estop(self):
        """/emergency_stop 에 true 를 발행한다.

        false 를 발행하는 경로는 **이 클래스에 존재하지 않는다.** 해제는
        command_manager 의 래치 해제 서비스로만 가능해야 하고, 그래야 '무엇이
        정지를 풀 수 있는가'가 코드 한 곳에서만 결정된다.
        """
        self._estop_pub.publish(Bool(data=True))
        self.get_logger().warn("웹 요청으로 E-STOP 발행")

    def release_estop(self, reason=""):
        request = ReleaseEstop.Request()
        request.reason = reason or "web"
        response = self._call(self._release_cli, request, timeout=5.0, what="E-STOP 해제")
        return {
            "success": bool(response.success),
            "message": response.message,
            "latched": bool(response.latched),
        }

    # ── 맵 저장 ─────────────────────────────────────────────────────────
    def map_save(self, timeout=120.0, abort_if=None):
        """FAST-LIO 의 /map_save. 점군이 크면 수십 초가 걸린다."""
        response = self._call(self._map_save_cli, Trigger.Request(),
                              timeout=timeout, what="/map_save", abort_if=abort_if)
        return {"success": bool(response.success), "message": response.message}

    def map_save_available(self):
        return self._map_save_cli.service_is_ready()

    # ── 라이다 출처 ─────────────────────────────────────────────────────
    # 재생본과 실측이 같은 토픽(/livox/lidar)으로 흐른다. 토픽 이름만 보면
    # 구분이 안 되고, 실제로 집에서 랩실 맵 재생본을 실시간 스캔으로 오인했다.
    # 누가 발행하는지는 노드 이름으로만 알 수 있다.
    #
    # foxglove 의 connectionGraph 로 브라우저가 직접 알아내려 했지만, 브리지가
    # 빈 그래프를 한 번 보내고 그 뒤로 갱신을 안 준다(3.4.2 기준). 그래서
    # 백엔드가 rclpy 로 직접 조회해 알려준다.
    LIDAR_TOPIC = "/livox/lidar"
    REPLAY_NODES = ("pcd_replay",)

    def lidar_source(self):
        try:
            infos = self.get_publishers_info_by_topic(self.LIDAR_TOPIC)
        except Exception as error:                      # noqa: BLE001
            # 조회 실패가 화면을 멈추게 하면 안 된다. 모른다고 말한다.
            return {"topic": self.LIDAR_TOPIC, "publishers": [],
                    "replay": None, "error": str(error)}
        names = sorted({info.node_name for info in infos})
        return {
            "topic": self.LIDAR_TOPIC,
            "publishers": names,
            "replay": any(name in self.REPLAY_NODES for name in names),
        }

    # ── 그래프 조회 ─────────────────────────────────────────────────────
    def running_nodes(self, wanted):
        """wanted 중 지금 ROS 그래프에 있는 노드 이름.

        ProcessManager 는 **자기가 띄운 것만** 안다. CLI 로 먼저 launch 를
        띄워놓고 웹을 여는 것은 흔한 순서인데, 그 경우 슬롯은 비어 있으므로
        중복 기동을 막지 못한다. 프로세스 표가 아니라 그래프에 물어야 한다.
        """
        try:
            names = {name for name, _ in self.get_node_names_and_namespaces()}
        except Exception:                                # noqa: BLE001
            return []
        return sorted(name for name in wanted if name in names)

    # ── command_manager 속도 한계 ───────────────────────────────────────
    LIMIT_KEYS = ("max_linear_x", "min_linear_x", "max_linear_y", "max_angular_z",
                  "auto_crab_enabled", "cmd_timeout_sec", "estop_latch")

    def limits(self, max_age=30.0):
        """command_manager 의 실제 파라미터를 읽는다.

        UI 가 0.45 / -0.15 / 0.8 / 0.3 을 JS 에 하드코딩해 두고 있었는데, 지금은
        우연히 값이 맞을 뿐이다. 로봇 쪽만 바꾸면 화면이 조용히 어긋난다.
        """
        now = time.monotonic()
        if self._limits_cache is not None and (now - self._limits_cache_at) < max_age:
            return self._limits_cache

        request = GetParameters.Request()
        request.names = list(self.LIMIT_KEYS)
        response = self._call(self._limits_cli, request, timeout=3.0,
                              what="command_manager 파라미터")

        out = {}
        for name, value in zip(self.LIMIT_KEYS, response.values):
            # ParameterType: 1=BOOL 2=INTEGER 3=DOUBLE 4=STRING, 0=NOT_SET
            if value.type == 1:
                out[name] = bool(value.bool_value)
            elif value.type == 2:
                out[name] = int(value.integer_value)
            elif value.type == 3:
                out[name] = float(value.double_value)
            elif value.type == 4:
                out[name] = value.string_value
            else:
                out[name] = None
        self._limits_cache = out
        self._limits_cache_at = now
        return out


def spin_in_background(node):
    """MultiThreadedExecutor 를 만들어 돌린다. 호출한 스레드를 막는다."""
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    return executor
