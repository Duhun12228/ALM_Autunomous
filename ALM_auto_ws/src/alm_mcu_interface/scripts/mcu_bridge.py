#!/usr/bin/env python3
"""mcu_bridge - Jetson <-> STM32 UART 통신 브리지 (순수 전송 계층, 기구학 없음).

  다운링크: /mcu/command (alm_msgs/McuCommand) --UART--> STM32
  업링크:   STM32 --UART--> /mcu/state (McuState) + /wheel_odom (Odometry) + /joint_states

프레임 포맷과 바이트 레이아웃은 docs/uart_protocol.md 에 정의되어 있으며,
STM32 펌웨어는 그 규격에 맞춰 파싱/생성하면 됩니다.

  Frame: 0xAA 0x55 | msg_type(1) | len(1) | payload(len) | crc16(2, big-endian)
  CRC16-CCITT(0x1021, init 0xFFFF) over [msg_type, len, payload].
"""

import math
import struct

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

from alm_msgs.msg import McuCommand, McuState

try:
    import serial
except ImportError:
    serial = None

SYNC0, SYNC1 = 0xAA, 0x55
MSG_COMMAND = 0x01
MSG_STATE = 0x02

# payload struct (little-endian). docs/uart_protocol.md 와 반드시 일치.
CMD_FMT = "<fffBBI"          # vx, vy, wz, drive_mode, flags, sequence  = 18 bytes
STATE_FMT = "<I14fBH"        # seq, [odom_x,y,th, vx,vy,wz, steerF,steerR,
                             #        wFL,wFR,wRL,wRR, batt_v,batt_c], flags, fault = 63 bytes
CMD_LEN = struct.calcsize(CMD_FMT)
STATE_LEN = struct.calcsize(STATE_FMT)

MODE_TO_ID = {"normal": 0, "crab": 1, "spin": 2, "auto": 3}

STEER_JOINTS = ["front_left_steer_joint", "front_right_steer_joint",
                "rear_left_steer_joint", "rear_right_steer_joint"]
WHEEL_JOINTS = ["front_left_wheel_joint", "front_right_wheel_joint",
                "rear_left_wheel_joint", "rear_right_wheel_joint"]


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc


def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


class McuBridge(Node):
    def __init__(self):
        super().__init__("mcu_bridge")
        g = self.declare_parameter
        self.port = g("port", "/dev/ttyTHS1").value
        self.baud = g("baudrate", 115200).value
        self.ser_timeout = g("serial_timeout", 0.02).value
        self.reconnect_period = g("reconnect_period_sec", 1.0).value
        self.odom_frame = g("odom_frame", "odom").value
        self.base_frame = g("base_frame", "base_link").value
        self.publish_joints = g("publish_joint_states", True).value
        poll_hz = g("poll_rate_hz", 200.0).value
        cmd_topic = g("command_topic", "/mcu/command").value
        state_topic = g("state_topic", "/mcu/state").value
        odom_topic = g("odom_topic", "/wheel_odom").value

        self.ser = None
        self.rx = bytearray()
        self.wheel_pos = [0.0, 0.0, 0.0, 0.0]   # joint_states 적분용
        self.last_state_time = None

        self.state_pub = self.create_publisher(McuState, state_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, odom_topic, 10)
        self.joint_pub = (self.create_publisher(JointState, "/joint_states", 10)
                          if self.publish_joints else None)
        self.create_subscription(McuCommand, cmd_topic, self.on_command, 10)

        self.create_timer(1.0 / max(1.0, poll_hz), self.poll_serial)
        self.create_timer(self.reconnect_period, self.ensure_serial)
        self.ensure_serial()
        self.get_logger().info(
            f"mcu_bridge: port={self.port}@{self.baud} "
            f"(CMD {CMD_LEN}B / STATE {STATE_LEN}B)"
        )

    # ---------------- serial 관리 ----------------
    def ensure_serial(self):
        if self.ser is not None and self.ser.is_open:
            return
        if serial is None:
            self.get_logger().error("pyserial 미설치: pip install pyserial", once=True)
            return
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=self.ser_timeout)
            self.rx.clear()
            self.get_logger().info(f"시리얼 연결됨: {self.port}")
        except serial.SerialException as e:
            self.ser = None
            self.get_logger().warn(f"시리얼 열기 실패 ({self.port}): {e}", throttle_duration_sec=5.0)

    def _write_frame(self, msg_type: int, payload: bytes):
        if self.ser is None or not self.ser.is_open:
            return
        body = bytes([msg_type, len(payload)]) + payload
        crc = crc16_ccitt(body)
        frame = bytes([SYNC0, SYNC1]) + body + struct.pack(">H", crc)
        try:
            self.ser.write(frame)
        except serial.SerialException as e:
            self.get_logger().warn(f"시리얼 쓰기 실패: {e}")
            self.ser = None

    # ---------------- 다운링크 (command -> STM32) ----------------
    def on_command(self, msg: McuCommand):
        flags = (0x01 if msg.enable_motors else 0) | (0x02 if msg.emergency_stop else 0)
        mode_id = MODE_TO_ID.get(msg.drive_mode, 0)
        payload = struct.pack(
            CMD_FMT,
            float(msg.cmd_vel.linear.x),
            float(msg.cmd_vel.linear.y),
            float(msg.cmd_vel.angular.z),
            mode_id, flags,
            int(msg.sequence) & 0xFFFFFFFF,
        )
        self._write_frame(MSG_COMMAND, payload)

    # ---------------- 업링크 (STM32 -> state) ----------------
    def poll_serial(self):
        if self.ser is None or not self.ser.is_open:
            return
        try:
            n = self.ser.in_waiting
            chunk = self.ser.read(n if n > 0 else 1)
        except (OSError, serial.SerialException) as e:
            self.get_logger().warn(f"시리얼 읽기 실패: {e}")
            self.ser = None
            return
        if chunk:
            self.rx.extend(chunk)
            self._parse()

    def _parse(self):
        buf = self.rx
        while True:
            # sync 탐색
            start = buf.find(bytes([SYNC0, SYNC1]))
            if start < 0:
                if len(buf) > 1:
                    del buf[:-1]
                return
            if start > 0:
                del buf[:start]
            if len(buf) < 4:            # sync(2)+type(1)+len(1)
                return
            msg_type = buf[2]
            length = buf[3]
            frame_len = 4 + length + 2  # + crc
            if len(buf) < frame_len:
                return
            body = bytes(buf[2:4 + length])
            crc_rx = struct.unpack(">H", bytes(buf[4 + length:6 + length]))[0]
            if crc16_ccitt(body) != crc_rx:
                del buf[:2]             # sync 무효, 한 칸 전진 후 재탐색
                continue
            payload = bytes(buf[4:4 + length])
            del buf[:frame_len]
            if msg_type == MSG_STATE and length == STATE_LEN:
                self._handle_state(payload)

    def _handle_state(self, payload: bytes):
        v = struct.unpack(STATE_FMT, payload)
        seq = v[0]
        (ox, oy, oth, vx, vy, wz,
         steer_f, steer_r, w_fl, w_fr, w_rl, w_rr,
         batt_v, batt_c) = v[1:15]
        flags = v[15]
        fault_code = v[16]
        now = self.get_clock().now()

        st = McuState()
        st.stamp = now.to_msg()
        st.sequence = seq
        st.measured_velocity.linear.x = vx
        st.measured_velocity.linear.y = vy
        st.measured_velocity.angular.z = wz
        st.odom_pose.x = ox
        st.odom_pose.y = oy
        st.odom_pose.theta = oth
        st.steer_angle = [steer_f, steer_r]
        st.wheel_speed = [w_fl, w_fr, w_rl, w_rr]
        st.battery_voltage = batt_v
        st.battery_current = batt_c
        st.motors_enabled = bool(flags & 0x01)
        st.emergency_stop = bool(flags & 0x02)
        st.command_timeout = bool(flags & 0x04)
        st.fault = bool(flags & 0x08)
        st.fault_code = fault_code
        self.state_pub.publish(st)

        odom = Odometry()
        odom.header.stamp = st.stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = ox
        odom.pose.pose.position.y = oy
        odom.pose.pose.orientation = yaw_to_quat(oth)
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz
        # 속도 융합 위주 → pose 공분산은 크게, twist 는 작게
        odom.pose.covariance[0] = 0.05
        odom.pose.covariance[7] = 0.05
        odom.pose.covariance[35] = 0.1
        odom.twist.covariance[0] = 0.02
        odom.twist.covariance[7] = 0.02
        odom.twist.covariance[35] = 0.04
        self.odom_pub.publish(odom)

        if self.joint_pub is not None:
            self._publish_joints(now, steer_f, steer_r, [w_fl, w_fr, w_rl, w_rr])

    def _publish_joints(self, now, steer_f, steer_r, wheel_speeds):
        dt = 0.0
        if self.last_state_time is not None:
            dt = (now - self.last_state_time).nanoseconds * 1e-9
            dt = min(max(dt, 0.0), 0.2)
        self.last_state_time = now
        for i, ws in enumerate(wheel_speeds):
            self.wheel_pos[i] += ws * dt

        js = JointState()
        js.header.stamp = now.to_msg()
        js.name = STEER_JOINTS + WHEEL_JOINTS
        # 2축 조향: 앞축 각도는 앞 두 바퀴에, 뒤축 각도는 뒤 두 바퀴에 동일 적용(평행 근사)
        js.position = [steer_f, steer_f, steer_r, steer_r] + list(self.wheel_pos)
        self.joint_pub.publish(js)


def main():
    rclpy.init()
    node = McuBridge()
    try:
        rclpy.spin(node)
    finally:
        if node.ser is not None and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
