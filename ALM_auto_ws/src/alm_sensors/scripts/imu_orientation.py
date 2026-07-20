#!/usr/bin/env python3
"""6축 IMU 에 orientation 을 합성해 재발행 (Madgwick, 자력계 없음).

MID-360 내장 IMU(/livox/imu)는 gyro+accel 만 있고 orientation 이 없다.
LIO-SAM 은 스캔 deskew 초기값으로 orientation(roll/pitch)을 요구하므로
Madgwick 필터로 합성해 /livox/imu_orient 로 재발행한다.
(ros-humble-imu-filter-madgwick 대체 — apt 설치에 sudo 필요해서 자체 구현)

yaw 는 자력계가 없어 gyro 적분만으로 드리프트한다 → LIO-SAM 설정에서
useImuHeadingInitialization: false 필수 (roll/pitch 만 신뢰).
"""
import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu


class MadgwickNode(Node):
    def __init__(self):
        super().__init__("imu_orientation")
        self.declare_parameter("input_topic", "/livox/imu")
        self.declare_parameter("output_topic", "/livox/imu_orient")
        self.declare_parameter("beta", 0.1)   # Madgwick 수렴 게인
        # 실센서 UDP 파서는 이미 m/s^2. livox_ros_driver2 bag은 g 단위이므로
        # bag 재생 시 9.80665로 설정한다.
        self.declare_parameter("accel_scale", 1.0)

        self.beta = float(self.get_parameter("beta").value)
        self.accel_scale = float(self.get_parameter("accel_scale").value)
        self.q = [1.0, 0.0, 0.0, 0.0]  # w, x, y, z (body->world)
        self.initialized = False
        self.last_t = None

        out = self.get_parameter("output_topic").value
        self.pub = self.create_publisher(Imu, out, 50)
        self.sub = self.create_subscription(
            Imu, self.get_parameter("input_topic").value, self.cb, 50)
        self.get_logger().info(f"Madgwick(6축) {self.sub.topic_name} -> {out}, "
                               f"beta={self.beta}, accel_scale={self.accel_scale}")

    def cb(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        dt = 0.005 if self.last_t is None else min(max(t - self.last_t, 1e-4), 0.05)
        self.last_t = t

        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        ax = msg.linear_acceleration.x * self.accel_scale
        ay = msg.linear_acceleration.y * self.accel_scale
        az = msg.linear_acceleration.z * self.accel_scale
        # Madgwick 보정에서는 아래에서 단위벡터가 필요하지만, 발행값은 반드시
        # 센서의 물리 단위(m/s^2)를 보존해야 한다.
        output_ax, output_ay, output_az = ax, ay, az

        # 시작 자세가 기울어진 상태여도 중력을 운동 가속도로 오인하지 않도록
        # 첫 유효 accel에서 roll/pitch를 즉시 초기화한다. yaw는 0으로 둔다.
        if not self.initialized:
            accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
            if accel_norm > 1e-3:
                roll = math.atan2(ay, az)
                pitch = math.atan2(-ax, math.sqrt(ay * ay + az * az))
                cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
                cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
                self.q = [cr * cp, sr * cp, cr * sp, -sr * sp]
                self.initialized = True
                self.get_logger().info(
                    f"Initial tilt from accel: roll={math.degrees(roll):.2f}deg, "
                    f"pitch={math.degrees(pitch):.2f}deg")

        q0, q1, q2, q3 = self.q

        # gyro 에 의한 자세 미분
        qd0 = 0.5 * (-q1 * gx - q2 * gy - q3 * gz)
        qd1 = 0.5 * (q0 * gx + q2 * gz - q3 * gy)
        qd2 = 0.5 * (q0 * gy - q1 * gz + q3 * gx)
        qd3 = 0.5 * (q0 * gz + q1 * gy - q2 * gx)

        # accel(중력 방향) 보정 — 경사하강 1스텝
        norm = math.sqrt(ax * ax + ay * ay + az * az)
        if norm > 1e-3:
            norm_ax, norm_ay, norm_az = ax / norm, ay / norm, az / norm
            f0 = 2.0 * (q1 * q3 - q0 * q2) - norm_ax
            f1 = 2.0 * (q0 * q1 + q2 * q3) - norm_ay
            f2 = 2.0 * (0.5 - q1 * q1 - q2 * q2) - norm_az
            s0 = -2.0 * q2 * f0 + 2.0 * q1 * f1
            s1 = 2.0 * q3 * f0 + 2.0 * q0 * f1 - 4.0 * q1 * f2
            s2 = -2.0 * q0 * f0 + 2.0 * q3 * f1 - 4.0 * q2 * f2
            s3 = 2.0 * q1 * f0 + 2.0 * q2 * f1
            snorm = math.sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3)
            if snorm > 1e-9:
                qd0 -= self.beta * s0 / snorm
                qd1 -= self.beta * s1 / snorm
                qd2 -= self.beta * s2 / snorm
                qd3 -= self.beta * s3 / snorm

        q0 += qd0 * dt
        q1 += qd1 * dt
        q2 += qd2 * dt
        q3 += qd3 * dt
        n = math.sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3)
        self.q = [q0 / n, q1 / n, q2 / n, q3 / n]

        out = Imu()
        out.header = msg.header
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration.x = output_ax
        out.linear_acceleration.y = output_ay
        out.linear_acceleration.z = output_az
        scale2 = self.accel_scale * self.accel_scale
        out.linear_acceleration_covariance = [
            value * scale2 if value >= 0.0 else value
            for value in msg.linear_acceleration_covariance
        ]
        out.orientation.w = self.q[0]
        out.orientation.x = self.q[1]
        out.orientation.y = self.q[2]
        out.orientation.z = self.q[3]
        # roll/pitch 는 수렴, yaw 는 드리프트 -> yaw 분산 크게
        out.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 1.0]
        self.pub.publish(out)


def main():
    rclpy.init()
    try:
        rclpy.spin(MadgwickNode())
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
