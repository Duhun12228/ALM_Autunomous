#!/usr/bin/env python3
"""Nav2 액션 서버 대역 — alm_web_backend 의 자율주행 경로를 하드웨어 없이 검증한다.

제공하는 것:
  /navigate_to_pose  (NavigateToPose)  distance_remaining 을 줄여가며 피드백
  /follow_waypoints  (FollowWaypoints) current_waypoint 를 올려가며 피드백
  map -> odom TF                        '측위 수렴' 을 흉내낸다

인자:
  --fail        목표를 abort 로 끝낸다 (경로 없음 재현)
  --reject      목표를 거절한다
  --no-tf       TF 를 안 낸다 (측위 미수렴 재현)
  --step SEC    목표 하나에 걸리는 시간
"""
import argparse
import sys
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from geometry_msgs.msg import TransformStamped
from nav2_msgs.action import FollowWaypoints, NavigateToPose
from tf2_ros import TransformBroadcaster


class FakeNav2(Node):
    def __init__(self, args):
        super().__init__("fake_nav2")
        self.args = args
        group = ReentrantCallbackGroup()
        self._pose_srv = ActionServer(
            self, NavigateToPose, "/navigate_to_pose",
            execute_callback=self._run_pose,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=group)
        self._wp_srv = ActionServer(
            self, FollowWaypoints, "/follow_waypoints",
            execute_callback=self._run_waypoints,
            goal_callback=self._on_goal, cancel_callback=self._on_cancel,
            callback_group=group)
        if not args.no_tf:
            self._tf = TransformBroadcaster(self)
            self.create_timer(0.1, self._send_tf)
        self.get_logger().info(
            f"fake_nav2 준비: step={args.step}s fail={args.fail} reject={args.reject} "
            f"tf={not args.no_tf}")

    def _send_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.rotation.w = 1.0
        self._tf.sendTransform(tf)

    def _on_goal(self, _goal):
        return GoalResponse.REJECT if self.args.reject else GoalResponse.ACCEPT

    def _on_cancel(self, _goal):
        self.get_logger().info("취소 요청 수락")
        return CancelResponse.ACCEPT

    def _sleep_steps(self, handle, steps, feedback_for):
        """steps 번 쉬면서 피드백을 보낸다. 취소되면 False."""
        for i in range(steps):
            for _ in range(10):
                if handle.is_cancel_requested:
                    return False
                time.sleep(self.args.step / 10.0)
            handle.publish_feedback(feedback_for(i + 1))
        return True

    def _run_pose(self, handle):
        goal = handle.request.pose.pose.position
        self.get_logger().info(f"NavigateToPose 시작 x={goal.x:.2f} y={goal.y:.2f}")
        steps = 5

        def feedback(done):
            msg = NavigateToPose.Feedback()
            msg.distance_remaining = float(steps - done)
            msg.estimated_time_remaining.sec = int((steps - done) * self.args.step)
            msg.number_of_recoveries = 0
            return msg

        if not self._sleep_steps(handle, steps, feedback):
            handle.canceled()
            self.get_logger().info("NavigateToPose 취소됨")
            return NavigateToPose.Result()
        if self.args.fail:
            handle.abort()
            self.get_logger().info("NavigateToPose abort")
            return NavigateToPose.Result()
        handle.succeed()
        self.get_logger().info("NavigateToPose 성공")
        return NavigateToPose.Result()

    def _run_waypoints(self, handle):
        poses = handle.request.poses
        self.get_logger().info(f"FollowWaypoints 시작 {len(poses)}개")

        def feedback(done):
            msg = FollowWaypoints.Feedback()
            msg.current_waypoint = min(done, len(poses) - 1)
            return msg

        if not self._sleep_steps(handle, len(poses), feedback):
            handle.canceled()
            self.get_logger().info("FollowWaypoints 취소됨")
            return FollowWaypoints.Result()
        result = FollowWaypoints.Result()
        if self.args.fail:
            # 실제 Nav2 도 일부를 못 가면 SUCCEEDED + missed 로 끝낸다
            result.missed_waypoints = [len(poses) - 1]
        handle.succeed()
        self.get_logger().info(f"FollowWaypoints 성공 missed={list(result.missed_waypoints)}")
        return result


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--step", type=float, default=1.0)
    parser.add_argument("--fail", action="store_true")
    parser.add_argument("--reject", action="store_true")
    parser.add_argument("--no-tf", action="store_true")
    args = parser.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])

    rclpy.init()
    node = FakeNav2(args)
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
