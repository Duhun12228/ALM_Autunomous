"""rclpy 쪽 접점 — 토픽 발행, 서비스 호출, Nav2 액션.

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

액션(§자율주행)도 같은 규약을 따른다. 다만 서비스와 달리 **HTTP 요청이 끝난
뒤에도 계속 진행되는 것**이라 상태를 여기 들고 있어야 한다: 목표 수락은 몇십
ms 만에 끝나지만 주행은 몇 분이 걸리고, 그동안 화면은 폴링으로 물어본다.
피드백·결과 콜백은 실행기 스레드에서 오고 조회는 HTTP 스레드에서 하므로
_nav_lock 으로 묶는다.
"""

import math
import threading
import time

import rclpy
import tf2_ros
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from alm_msgs.msg import Speech
from alm_msgs.srv import ReleaseEstop

from . import navigation


class RosTimeout(Exception):
    """서비스가 시간 안에 응답하지 않음 (노드가 없거나 멈춰 있음)."""


class NavRejected(Exception):
    """Nav2 가 목표를 거절했거나 전송이 실패함."""


# 미션이 없는 상태. dict(_NAV_IDLE) 로 복사해 쓴다.
#
# state 의 의미:
#   idle       미션 없음
#   pending    목표를 보냈고 수락을 기다리는 중
#   active     주행 중
#   paused     조작자가 세웠다. points[index:] 가 남아 있고 resume 으로 이어간다
#   succeeded  전 목표 도달
#   failed     Nav2 가 abort (경로 없음·리커버리 소진 등)
#   canceled   조작자가 미션을 폐기
_NAV_IDLE = {
    "state": "idle",
    "kind": "",              # "" | "pose" | "waypoints"
    "points": [],            # 미션 전체 목록 (일시정지 후 재개의 근거)
    "index": 0,              # 지금 향하고 있는 목표의 전체 목록상 위치 (0-based)
    "sent_index": 0,         # 이번 전송의 첫 목표가 전체 목록에서 몇 번째였나
    "distance_remaining_m": None,   # NavigateToPose 만 준다
    "eta_sec": None,                # 〃
    "recoveries": 0,                # 〃
    "distance_estimate_m": None,    # 직선 합. 실제 경로장이 아니다 (하한)
    "missed": [],            # FollowWaypoints 가 못 간 목표 인덱스
    "message": "",
    "started_wall": 0.0,
    "updated_wall": 0.0,
    "pending_cancel": "",    # "pause" | "cancel" — 결과 콜백이 상태를 정할 때 쓴다
}


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

        # ---- 자율주행 ----
        # TF 는 '측위가 수렴했나' 를 묻는 가장 정직한 방법이다. /icp_result 는
        # 성공 시 딱 한 번만 오므로, 백엔드가 그 뒤에 재시작하면 영영 못 본다.
        # map->odom 이 지금 조회되는가는 재시작과 무관하게 참이다.
        self._tf_buffer = tf2_ros.Buffer()
        # spin_thread=False — 이 노드는 이미 MultiThreadedExecutor 가 돌린다.
        # True 로 두면 리스너가 자기 실행기를 하나 더 만들어 같은 노드를 두 곳에서
        # spin 하게 된다 (이 파일 도입부의 그 함정이다).
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, self, spin_thread=False)

        self._nav_lock = threading.Lock()
        self._nav = dict(_NAV_IDLE)
        self._nav_goal_handle = None
        self.on_nav_finished = None

        self._nav_clients = {
            "pose": ActionClient(self, NavigateToPose, self.NAV_ACTION_POSE,
                                 callback_group=self._group),
            "waypoints": ActionClient(self, FollowWaypoints, self.NAV_ACTION_WAYPOINTS,
                                      callback_group=self._group),
        }

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
                  "cmd_timeout_sec", "estop_latch",
                  "auto_crab_enabled")

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


    # ── 자율주행 (Nav2 액션) ────────────────────────────────────────────
    NAV_ACTION_POSE = "/navigate_to_pose"
    NAV_ACTION_WAYPOINTS = "/follow_waypoints"

    # map->odom 이 이 시간보다 오래됐으면 '측위 살아있음' 으로 보지 않는다.
    # tf2 버퍼는 마지막 값을 캐시 기간(기본 10 s) 동안 들고 있으므로, 나이를
    # 안 보면 측위가 죽은 뒤에도 한동안 준비된 것처럼 보인다.
    NAV_TF_MAX_AGE_SEC = 5.0
    NAV_ACCEPT_TIMEOUT = 5.0
    NAV_CANCEL_TIMEOUT = 5.0

    def nav_action_ready(self, kind=None):
        """Nav2 액션 서버가 광고 중인가. kind 를 주면 그것만 본다."""
        kinds = (kind,) if kind else tuple(self._nav_clients)
        return all(self._nav_clients[name].server_is_ready() for name in kinds)

    def nav_tf_ready(self):
        """map->odom 이 지금 조회되는가 = 초기 정합이 붙었는가.

        can_transform 만으로는 부족하다 — 측위가 죽어도 버퍼에 남은 마지막
        값으로 True 가 나온다. 스탬프 나이까지 본다.
        """
        try:
            if not self._tf_buffer.can_transform("map", "odom", rclpy.time.Time()):
                return False
            transform = self._tf_buffer.lookup_transform(
                "map", "odom", rclpy.time.Time())
        except Exception:                                # noqa: BLE001
            return False
        stamp = transform.header.stamp
        stamp_sec = stamp.sec + stamp.nanosec * 1e-9
        if stamp_sec <= 0.0:
            # static transform 으로 들어온 경우 스탬프가 0 이다. 나이를 못 재므로
            # 있는 것만으로 인정한다 — 없는 것보다는 확실히 낫다.
            return True
        now = self.get_clock().now().nanoseconds * 1e-9
        return (now - stamp_sec) <= self.NAV_TF_MAX_AGE_SEC

    def nav_status(self):
        with self._nav_lock:
            snapshot = dict(self._nav)
        snapshot.pop("pending_cancel", None)
        snapshot["remaining"] = snapshot["points"][snapshot["index"]:]
        snapshot["total"] = len(snapshot["points"])
        snapshot["action_ready"] = self.nav_action_ready()
        snapshot["tf_ready"] = self.nav_tf_ready()
        return snapshot

    def nav_busy(self):
        with self._nav_lock:
            return self._nav["state"] in ("pending", "active")

    def _pose_stamped(self, point):
        pose = PoseStamped()
        pose.header.frame_id = navigation.GOAL_FRAME
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(point["x"])
        pose.pose.position.y = float(point["y"])
        z, w = navigation.yaw_to_quaternion(point["yaw_deg"])
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose

    def nav_send(self, points, *, start_index=0, resume=False):
        """목표를 보내고 **수락까지 확인한 뒤** 돌아온다.

        수락을 기다리는 이유: 여기서 안 기다리면 HTTP 응답은 성공인데 Nav2 는
        거절한 상태가 만들어진다. 화면은 'RUNNING' 을 띄우고 로봇은 서 있는다.
        수락은 보통 수십 ms 라 요청을 붙들어도 비싸지 않다.

        목표 하나면 NavigateToPose 를 쓴다. FollowWaypoints 는 남은 거리와
        도착 예정 시간을 **주지 않기 때문**이다 (피드백이 current_waypoint 뿐).
        하나짜리 미션에서까지 그 정보를 버릴 이유가 없다.
        """
        if not points:
            raise NavRejected("보낼 목표가 없습니다.")
        kind = "pose" if len(points) == 1 else "waypoints"
        client = self._nav_clients[kind]
        if not client.server_is_ready():
            raise NavRejected(
                f"{self.NAV_ACTION_POSE if kind == 'pose' else self.NAV_ACTION_WAYPOINTS} "
                f"액션 서버가 없습니다. Nav2 가 떠 있는지 확인하세요.")

        if kind == "pose":
            goal = NavigateToPose.Goal()
            goal.pose = self._pose_stamped(points[0])
        else:
            goal = FollowWaypoints.Goal()
            goal.poses = [self._pose_stamped(point) for point in points]

        with self._nav_lock:
            # 전체 목록은 재개의 근거다. 이어가는 전송이면 앞부분을 보존한다.
            previous = self._nav["points"] if resume else []
            self._nav = {
                **_NAV_IDLE,
                "state": "pending",
                "kind": kind,
                "points": (previous[:start_index] + list(points)) if resume else list(points),
                "index": start_index,
                "sent_index": start_index,
                "distance_estimate_m": navigation.path_length(points),
                "message": "목표 수락을 기다리는 중",
                "started_wall": time.time(),
                "updated_wall": time.time(),
            }

        send_future = client.send_goal_async(
            goal, feedback_callback=self._nav_on_feedback)

        deadline = time.monotonic() + self.NAV_ACCEPT_TIMEOUT
        while time.monotonic() < deadline and not send_future.done():
            time.sleep(0.02)
        if not send_future.done():
            send_future.cancel()
            self._nav_finish("failed", "목표 전송이 응답하지 않습니다 (Nav2 무응답).")
            raise NavRejected(
                f"Nav2 가 {self.NAV_ACCEPT_TIMEOUT:.0f}초 안에 목표를 받지 않았습니다.")

        try:
            handle = send_future.result()
        except Exception as error:                       # noqa: BLE001
            self._nav_finish("failed", f"목표 전송 실패: {error}")
            raise NavRejected(f"목표 전송 실패: {error}") from error

        if handle is None or not handle.accepted:
            # 거절은 오류가 아니라 판단이다. Nav2 는 목표가 코스트맵 밖이거나
            # 서버가 비활성일 때 거절한다.
            self._nav_finish("failed", "Nav2 가 목표를 거절했습니다.")
            raise NavRejected(
                "Nav2 가 목표를 거절했습니다 — 목표가 맵 밖이거나 코스트맵에서 "
                "점유 상태일 수 있습니다.")

        with self._nav_lock:
            self._nav_goal_handle = handle
            self._nav["state"] = "active"
            self._nav["message"] = "주행 중"
            self._nav["updated_wall"] = time.time()

        handle.get_result_async().add_done_callback(self._nav_on_result)
        return self.nav_status()

    def _nav_on_feedback(self, message):
        """실행기 스레드. 액션 종류마다 오는 것이 다르다."""
        feedback = message.feedback
        with self._nav_lock:
            if self._nav["state"] not in ("pending", "active"):
                return                    # 이미 끝났거나 세운 미션의 뒷북
            if self._nav["kind"] == "pose":
                self._nav["distance_remaining_m"] = round(
                    float(feedback.distance_remaining), 2)
                eta = feedback.estimated_time_remaining
                self._nav["eta_sec"] = round(eta.sec + eta.nanosec * 1e-9, 1)
                self._nav["recoveries"] = int(feedback.number_of_recoveries)
            else:
                # current_waypoint 는 **이번에 보낸 목록** 기준이다. 재개하면
                # 앞부분을 안 보냈으므로 전체 목록 위치로 옮겨야 한다.
                self._nav["index"] = (self._nav["sent_index"]
                                      + int(feedback.current_waypoint))
            self._nav["updated_wall"] = time.time()

    def _nav_on_result(self, future):
        """실행기 스레드. 미션의 끝."""
        try:
            outcome = future.result()
        except Exception as error:                       # noqa: BLE001
            self._nav_finish("failed", f"결과 수신 실패: {error}")
            return

        status = outcome.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            missed = list(getattr(outcome.result, "missed_waypoints", []) or [])
            if missed:
                # FollowWaypoints 는 일부를 못 가도 SUCCEEDED 로 끝낸다.
                # 그걸 '성공' 이라고만 표시하면 화면이 거짓말을 한다.
                self._nav_finish(
                    "failed",
                    f"목표 {len(missed)}개를 건너뛰었습니다 "
                    f"(번호 {', '.join(str(i + 1) for i in missed)}).",
                    missed=missed)
            else:
                self._nav_finish("succeeded", "전 목표에 도달했습니다.")
        elif status == GoalStatus.STATUS_CANCELED:
            with self._nav_lock:
                paused = self._nav["pending_cancel"] == "pause"
            if paused:
                self._nav_finish("paused", "일시정지 — 남은 목표를 유지합니다.")
            else:
                self._nav_finish("canceled", "미션을 중단했습니다.")
        else:
            # ABORTED. 이 플랫폼에서 가장 흔한 원인은 '경로가 아예 안 나오는
            # 목표' 다 (docs/control_pipeline.md §9.3-K). 그걸 짚어 준다.
            self._nav_finish(
                "failed",
                "Nav2 가 목표를 포기했습니다 (abort). 경로를 못 찾았거나 "
                "리커버리를 모두 소진했습니다 — 목표가 벽에 너무 가깝거나, "
                "최소 선회반경 1.643 m 로는 들어갈 수 없는 자리일 수 있습니다.")

    def _nav_finish(self, state, message, missed=()):
        with self._nav_lock:
            self._nav["state"] = state
            self._nav["message"] = message
            self._nav["missed"] = list(missed)
            self._nav["pending_cancel"] = ""
            self._nav["distance_remaining_m"] = None
            self._nav["eta_sec"] = None
            self._nav["updated_wall"] = time.time()
            self._nav_goal_handle = None
            if state == "succeeded":
                self._nav["index"] = len(self._nav["points"])
            snapshot = dict(self._nav)

        # 콜백은 락 밖에서 (_on_icp_result 와 같은 규약 — 발화가 걸릴 수 있다)
        callback = self.on_nav_finished
        if callback is None:
            return
        try:
            callback(state, message, snapshot)
        except Exception:                                # noqa: BLE001
            pass

    def nav_cancel(self, *, keep):
        """현재 목표를 취소한다. keep=True 면 남은 목표를 보존한다(일시정지).

        취소 요청만 보내고 상태 전이는 **결과 콜백에 맡긴다.** 여기서 곧바로
        'paused' 로 써 버리면, 취소가 실제로 안 먹었을 때(이미 도착 직전이라
        SUCCEEDED 로 끝나는 경우) 화면만 세워지고 로봇은 계속 간다.
        """
        with self._nav_lock:
            handle = self._nav_goal_handle
            if handle is None or self._nav["state"] not in ("pending", "active"):
                raise NavRejected("진행 중인 미션이 없습니다.")
            self._nav["pending_cancel"] = "pause" if keep else "cancel"
            self._nav["message"] = "취소 요청을 보냈습니다"

        cancel_future = handle.cancel_goal_async()
        deadline = time.monotonic() + self.NAV_CANCEL_TIMEOUT
        while time.monotonic() < deadline and not cancel_future.done():
            time.sleep(0.02)
        if not cancel_future.done():
            cancel_future.cancel()
            raise NavRejected(
                f"취소 요청이 {self.NAV_CANCEL_TIMEOUT:.0f}초 안에 응답하지 "
                f"않았습니다. 로봇이 계속 주행 중일 수 있습니다 — "
                f"멈춰야 하면 E-STOP 을 쓰세요.")

        # 결과 콜백이 곧 상태를 확정한다. 그때까지 잠깐 기다려 준다 —
        # 여기서 바로 돌아가면 화면이 아직 'active' 인 스냅샷을 받는다.
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline and self.nav_busy():
            time.sleep(0.02)
        return self.nav_status()

    def nav_clear(self, *, force=False):
        """미션 기록을 지운다.

        force 는 **스택을 통째로 내릴 때만** 쓴다. Nav2 프로세스가 죽으면 결과
        future 는 영영 완료되지 않아 상태가 'active' 에 멈춘다 — 그때 지우기를
        거절하면 다음 기동까지 화면이 유령 미션을 들고 있게 된다.
        """
        with self._nav_lock:
            if not force and self._nav["state"] in ("pending", "active"):
                raise NavRejected("진행 중인 미션은 지울 수 없습니다. 먼저 중단하세요.")
            self._nav = dict(_NAV_IDLE)
            self._nav_goal_handle = None
        return self.nav_status()


def spin_in_background(node):
    """MultiThreadedExecutor 를 만들어 돌린다. 호출한 스레드를 막는다."""
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    return executor
