#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
ZERO_POSITION = [0.0] * 6


class PiperZero(Node):
    def __init__(self):
        super().__init__("piper_zero")
        self.latest_feedback = None
        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_feedback, 10
        )
        self.move_j_pub = self.create_publisher(JointState, "/control/move_j", 10)

    def _on_feedback(self, msg):
        self.latest_feedback = msg

    def wait_for_feedback(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.latest_feedback is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_feedback is None:
            raise RuntimeError("No /feedback/joint_states received.")
        return self.current_joints()

    def current_joints(self):
        positions = dict(zip(self.latest_feedback.name, self.latest_feedback.position))
        missing = [name for name in JOINT_NAMES if name not in positions]
        if missing:
            raise RuntimeError(
                f"Joint feedback is incomplete. Missing {missing}; got {self.latest_feedback.name}"
            )

        joints = [float(positions[name]) for name in JOINT_NAMES]
        if any(not math.isfinite(value) for value in joints):
            raise RuntimeError("Feedback contains non-finite joint values.")
        return joints

    def wait_for_move_subscriber(self, timeout):
        deadline = time.monotonic() + timeout
        while (
            rclpy.ok()
            and self.move_j_pub.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.move_j_pub.get_subscription_count() == 0:
            raise RuntimeError("No subscriber found on /control/move_j.")

    def publish_zero(self):
        target = JointState()
        target.header.stamp = self.get_clock().now().to_msg()
        target.name = JOINT_NAMES
        target.position = ZERO_POSITION
        self.move_j_pub.publish(target)
        self.get_logger().info(
            "Sent zero target: "
            + ", ".join(f"{name}=0.0000" for name in JOINT_NAMES)
        )

    def wait_until_zero(self, tolerance, timeout):
        deadline = time.monotonic() + timeout
        last_joints = self.current_joints()

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            last_joints = self.current_joints()
            if max(abs(value) for value in last_joints) <= tolerance:
                return True, last_joints

        return False, last_joints


def parse_args():
    parser = argparse.ArgumentParser(description="Move all Piper arm joints to zero.")
    parser.add_argument("--feedback-timeout", type=float, default=5.0)
    parser.add_argument("--subscriber-timeout", type=float, default=5.0)
    parser.add_argument("--motion-timeout", type=float, default=30.0)
    parser.add_argument("--tolerance", type=float, default=0.03)
    return parser.parse_args()


def main():
    args = parse_args()

    rclpy.init()
    node = PiperZero()
    try:
        current = node.wait_for_feedback(args.feedback_timeout)
        node.get_logger().info(
            "Current joints: "
            + ", ".join(f"{name}={value:.4f}" for name, value in zip(JOINT_NAMES, current))
        )

        node.wait_for_move_subscriber(args.subscriber_timeout)
        node.publish_zero()

        reached, final_joints = node.wait_until_zero(args.tolerance, args.motion_timeout)
        node.get_logger().info(
            "Final joints: "
            + ", ".join(f"{name}={value:.4f}" for name, value in zip(JOINT_NAMES, final_joints))
        )

        if not reached:
            raise RuntimeError(
                f"Zero target was sent, but joints did not reach tolerance "
                f"{args.tolerance} rad within {args.motion_timeout} s."
            )

        node.get_logger().info("Piper joints reached zero position.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
