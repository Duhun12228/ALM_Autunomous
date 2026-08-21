#!/usr/bin/env python3
"""virtual_pose_publisher — 측위를 **손으로 대신 준다**. 검증/개발 전용.

왜 필요한가. Nav2 의 전역 costmap 과 planner 는 측위 결과가 아니라 **TF** 를
본다. `map -> odom -> base_link` 만 있으면 경로계획은 성립한다. 그런데 이
스택에서 그 TF 를 만드는 것은 FAST-LIO-Localization(teaser_fpfh_localizer +
fastlio) 이고, 그게 초기위치를 못 잡으면 TF 가 없어서 **경로가 나오는지조차
확인할 수 없다.** 측위 문제와 경로계획 문제가 한 덩어리로 묶여 있는 상태다.

이 노드는 그 덩어리를 자른다. 가상의 초기위치를 map 좌표로 직접 박아
`map -> odom` 을 발행한다. 그러면 측위 없이도 planner_server 를 진짜로 돌려
"이 맵에서 전역경로가 나오는가"만 따로 확인할 수 있다.

  ##경고## 이건 **거짓 측위**다. 로봇이 실제로 거기 있다는 근거가 하나도 없다.
  실차 주행 스택(navigation.launch.py)에는 절대 넣지 마라. 넣으면 Nav2 는
  가짜 위치를 믿고 진짜 바퀴를 굴린다.

TF 분담은 실제 스택과 똑같이 맞춘다.

    map -> odom        이 노드 (= teaser_fpfh_localizer + transform_publisher 자리)
    odom -> base_link  이 노드 (= fastlio 자리, publish_odom_to_base:=false 로 끌 수 있음)

그래서 fastlio 만 따로 띄워 실제 주행 odometry 를 쓰면서 초기위치만 가상으로
주는 조합도 된다 — `publish_odom_to_base:=false` 로 두면 odom->base_link 는
fastlio 것을 쓰고, 이 노드는 map->odom 보정만 담당한다.

RViz/Foxglove 의 "2D Pose Estimate"(/initialpose)로 가상 위치를 옮길 수 있다.
그래서 여러 시작점에서 경로를 뽑아보는 게 쉽다.
"""

import math

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformBroadcaster, TransformListener


def quat_from_yaw(yaw):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def quat_inv(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def quat_rotate(q, v):
    """q * (v,0) * q^-1 의 벡터부."""
    x, y, z, w = q
    vx, vy, vz = v
    # t = 2 * (q_vec x v)
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def transform_inv(translation, rotation):
    inv_rotation = quat_inv(rotation)
    inv_translation = quat_rotate(inv_rotation, tuple(-c for c in translation))
    return inv_translation, inv_rotation


def transform_mul(ta, qa, tb, qb):
    """(A∘B): A 로 표현된 좌표계 위에 B 를 얹는다."""
    return tuple(a + r for a, r in zip(ta, quat_rotate(qa, tb))), quat_mul(qa, qb)


class VirtualPosePublisher(Node):
    def __init__(self):
        super().__init__("virtual_pose_publisher")
        g = self.declare_parameter
        g("x", 0.0)
        g("y", 0.0)
        g("yaw", 0.0)                    # rad, map 기준
        g("map_frame", "map")
        g("odom_frame", "odom")
        g("base_frame", "base_link")
        g("rate_hz", 20.0)
        # 단독 검증이면 true (이 노드가 TF 사슬 전체를 만든다).
        # fastlio 를 함께 띄우면 false — odom->base_link 를 두 노드가 내면 TF 가 싸운다.
        g("publish_odom_to_base", True)
        g("publish_odometry", True)      # /Odometry — controller/velocity_smoother 가 구독
        g("odom_topic", "/Odometry")

        p = self.get_parameter
        self.map_frame = str(p("map_frame").value)
        self.odom_frame = str(p("odom_frame").value)
        self.base_frame = str(p("base_frame").value)
        self.own_odom = bool(p("publish_odom_to_base").value)

        # map->odom. odom->base_link 를 이 노드가 항등으로 내므로 초기값은 그대로
        # 가상 초기위치가 된다.
        self.t_map_odom = (float(p("x").value), float(p("y").value), 0.0)
        self.q_map_odom = quat_from_yaw(float(p("yaw").value))

        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_pub = None
        if bool(p("publish_odometry").value):
            self.odom_pub = self.create_publisher(Odometry, str(p("odom_topic").value), 10)

        # RViz 의 "2D Pose Estimate" 는 기본적으로 신뢰성 없음(BEST_EFFORT)이 아니라
        # 기본 QoS 로 보낸다. 놓치면 안 되는 단발 메시지라 depth 를 넉넉히 둔다.
        self.create_subscription(
            PoseWithCovarianceStamped, "/initialpose", self.on_initialpose,
            QoSProfile(depth=10,
                       history=QoSHistoryPolicy.KEEP_LAST,
                       reliability=QoSReliabilityPolicy.RELIABLE,
                       durability=QoSDurabilityPolicy.VOLATILE))

        period = 1.0 / max(1.0, float(p("rate_hz").value))
        self.create_timer(period, self.tick)

        self.get_logger().warn(
            "가상 초기위치로 TF 를 만듭니다 — 실제 측위가 아닙니다. "
            f"map->odom=({self.t_map_odom[0]:.2f}, {self.t_map_odom[1]:.2f}, "
            f"yaw {math.degrees(float(p('yaw').value)):.1f}°), "
            f"odom->base_link {'항등 발행' if self.own_odom else '외부(fastlio) 사용'}")

    # ── /initialpose 로 가상 위치 이동 ────────────────────────────────────
    def on_initialpose(self, msg):
        if msg.header.frame_id not in ("", self.map_frame):
            self.get_logger().warn(
                f"/initialpose 의 frame_id 가 '{msg.header.frame_id}' 입니다 — "
                f"'{self.map_frame}' 만 받습니다. 무시합니다.")
            return

        pose = msg.pose.pose
        t_map_base = (pose.position.x, pose.position.y, pose.position.z)
        q_map_base = (pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w)

        if self.own_odom:
            # odom->base_link 가 항등이므로 map->odom = map->base_link.
            t_odom_base, q_odom_base = (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)
        else:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.odom_frame, self.base_frame, rclpy.time.Time())
            except Exception as error:                    # noqa: BLE001
                self.get_logger().warn(
                    f"{self.odom_frame}->{self.base_frame} TF 가 없어 /initialpose 를 "
                    f"반영하지 못했습니다: {error}")
                return
            v, r = tf.transform.translation, tf.transform.rotation
            t_odom_base = (v.x, v.y, v.z)
            q_odom_base = (r.x, r.y, r.z, r.w)

        # map->odom = (map->base) ∘ (odom->base)^-1
        t_inv, q_inv = transform_inv(t_odom_base, q_odom_base)
        self.t_map_odom, self.q_map_odom = transform_mul(
            t_map_base, q_map_base, t_inv, q_inv)
        self.get_logger().info(
            f"가상 위치 이동: map->base_link = ({t_map_base[0]:.2f}, {t_map_base[1]:.2f}, "
            f"yaw {math.degrees(self._yaw(q_map_base)):.1f}°)")

    @staticmethod
    def _yaw(q):
        x, y, z, w = q
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    # ── 주기 발행 ────────────────────────────────────────────────────────
    def tick(self):
        now = self.get_clock().now().to_msg()
        transforms = [self._tf(now, self.map_frame, self.odom_frame,
                               self.t_map_odom, self.q_map_odom)]
        if self.own_odom:
            transforms.append(self._tf(now, self.odom_frame, self.base_frame,
                                       (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)))
        self.tf_broadcaster.sendTransform(transforms)

        if self.odom_pub is not None and self.own_odom:
            odom = Odometry()
            odom.header.stamp = now
            odom.header.frame_id = self.odom_frame
            odom.child_frame_id = self.base_frame
            odom.pose.pose.orientation.w = 1.0
            self.odom_pub.publish(odom)

    @staticmethod
    def _tf(stamp, parent, child, translation, rotation):
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = parent
        tf.child_frame_id = child
        tf.transform.translation.x, tf.transform.translation.y, tf.transform.translation.z = (
            float(c) for c in translation)
        (tf.transform.rotation.x, tf.transform.rotation.y,
         tf.transform.rotation.z, tf.transform.rotation.w) = (float(c) for c in rotation)
        return tf


def main():
    rclpy.init()
    node = VirtualPosePublisher()
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
