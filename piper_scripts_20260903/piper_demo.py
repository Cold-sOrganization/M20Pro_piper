#!/usr/bin/env python3
import argparse
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
HOME_POSITION = [0.0] * 6
GRIPPER_NAME = "gripper"

JOINT_LIMITS = {
    "joint1": (-2.617994, 2.617994),
    "joint2": (0.0, 3.141593),
    "joint3": (-2.967060, 0.0),
    "joint4": (-1.745330, 1.745330),
    "joint5": (-1.221730, 1.221730),
    "joint6": (-2.094395, 2.094395),
}


class PiperDemo(Node):
    def __init__(self):
        super().__init__("piper_demo")
        self.latest_feedback = None
        self.feedback_sub = self.create_subscription(
            JointState, "/feedback/joint_states", self._on_feedback, 10
        )
        self.move_j_pub = self.create_publisher(JointState, "/control/move_j", 10)
        self.joint_states_pub = self.create_publisher(
            JointState, "/control/joint_states", 10
        )

    def _on_feedback(self, msg):
        self.latest_feedback = msg

    def wait_for_feedback(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.latest_feedback is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_feedback is None:
            raise RuntimeError("No /feedback/joint_states received.")
        return self.current_arm_positions()

    def feedback_positions(self):
        if self.latest_feedback is None:
            return {}
        return dict(zip(self.latest_feedback.name, self.latest_feedback.position))

    def current_arm_positions(self):
        positions = self.feedback_positions()
        missing = [name for name in JOINT_NAMES if name not in positions]
        if missing:
            raise RuntimeError(
                f"Joint feedback is incomplete. Missing {missing}; got {self.latest_feedback.name}"
            )
        return [float(positions[name]) for name in JOINT_NAMES]

    def wait_for_publisher_subscriber(self, publisher, topic, timeout):
        deadline = time.monotonic() + timeout
        while (
            rclpy.ok()
            and publisher.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError(f"No subscriber found on {topic}.")

    def publish_arm_target(self, positions):
        validate_arm_target(positions)
        target = JointState()
        target.header.stamp = self.get_clock().now().to_msg()
        target.name = JOINT_NAMES
        target.position = [float(value) for value in positions]
        self.move_j_pub.publish(target)
        self.get_logger().info(
            "Sent /control/move_j target: "
            + ", ".join(f"{name}={value:.4f}" for name, value in zip(target.name, target.position))
        )

    def publish_gripper_target(self, width, effort):
        width = clamp(width, 0.0, 0.1)
        effort = clamp(effort, 0.5, 3.0)

        target = JointState()
        target.header.stamp = self.get_clock().now().to_msg()
        target.name = [GRIPPER_NAME]
        target.position = [width]
        target.effort = [effort]
        self.joint_states_pub.publish(target)
        self.get_logger().info(
            f"Sent /control/joint_states gripper target: width={width:.3f} m, effort={effort:.2f} N"
        )

    def wait_until_arm_close(self, target, tolerance, timeout):
        deadline = time.monotonic() + timeout
        last_positions = self.current_arm_positions()

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            last_positions = self.current_arm_positions()
            errors = [abs(current - goal) for current, goal in zip(last_positions, target)]
            if max(errors) <= tolerance:
                return True, last_positions

        return False, last_positions

    def sleep_with_spin(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, remaining)))


def clamp(value, low, high):
    return max(low, min(high, float(value)))


def validate_arm_target(positions):
    if len(positions) != len(JOINT_NAMES):
        raise ValueError(f"Expected {len(JOINT_NAMES)} arm positions, got {len(positions)}.")

    for name, value in zip(JOINT_NAMES, positions):
        if not math.isfinite(value):
            raise ValueError(f"{name} target is not finite: {value}")
        low, high = JOINT_LIMITS[name]
        if value < low or value > high:
            raise ValueError(f"{name} target {value} is outside [{low}, {high}].")


def interpolate(start, end, step_count):
    if step_count <= 0:
        return [end]
    points = []
    for step in range(1, step_count + 1):
        ratio = step / float(step_count)
        points.append([a + (b - a) * ratio for a, b in zip(start, end)])
    return points


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move Piper to zero, raise slightly, then gently cycle the gripper."
    )
    parser.add_argument("--feedback-timeout", type=float, default=5.0)
    parser.add_argument("--subscriber-timeout", type=float, default=5.0)
    parser.add_argument("--motion-timeout", type=float, default=30.0)
    parser.add_argument("--tolerance", type=float, default=0.03)
    parser.add_argument("--raise-joint2", type=float, default=0.30)
    parser.add_argument("--raise-joint3", type=float, default=-0.30)
    parser.add_argument("--raise-steps", type=int, default=6)
    parser.add_argument("--raise-step-delay", type=float, default=1.0)
    parser.add_argument("--gripper-open", type=float, default=0.04)
    parser.add_argument("--gripper-closed", type=float, default=0.01)
    parser.add_argument("--gripper-effort", type=float, default=0.8)
    parser.add_argument("--gripper-cycles", type=int, default=3)
    parser.add_argument("--gripper-delay", type=float, default=1.2)
    parser.add_argument("--skip-gripper", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()

    raised_position = [
        0.0,
        clamp(args.raise_joint2, 0.0, 0.35),
        clamp(args.raise_joint3, -0.35, 0.0),
        0.0,
        0.0,
        0.0,
    ]
    validate_arm_target(HOME_POSITION)
    validate_arm_target(raised_position)

    rclpy.init()
    node = PiperDemo()

    try:
        current = node.wait_for_feedback(args.feedback_timeout)
        node.get_logger().info(
            "Current joints: "
            + ", ".join(f"{name}={value:.4f}" for name, value in zip(JOINT_NAMES, current))
        )

        if any(not math.isfinite(value) for value in current):
            raise RuntimeError("Feedback contains non-finite joint values.")

        node.wait_for_publisher_subscriber(
            node.move_j_pub, "/control/move_j", args.subscriber_timeout
        )
        node.publish_arm_target(HOME_POSITION)

        reached, final_positions = node.wait_until_arm_close(
            HOME_POSITION, args.tolerance, args.motion_timeout
        )
        node.get_logger().info(
            "After zero: "
            + ", ".join(f"{name}={value:.4f}" for name, value in zip(JOINT_NAMES, final_positions))
        )
        if not reached:
            raise RuntimeError(
                f"Zero target was sent, but joints did not reach tolerance {args.tolerance} rad "
                f"within {args.motion_timeout} s."
            )

        node.get_logger().info(
            "Raising slowly to: "
            + ", ".join(f"{name}={value:.4f}" for name, value in zip(JOINT_NAMES, raised_position))
        )
        for point in interpolate(HOME_POSITION, raised_position, max(1, args.raise_steps)):
            node.publish_arm_target(point)
            node.sleep_with_spin(max(0.2, args.raise_step_delay))

        reached, final_positions = node.wait_until_arm_close(
            raised_position, args.tolerance, args.motion_timeout
        )
        node.get_logger().info(
            "After raise: "
            + ", ".join(f"{name}={value:.4f}" for name, value in zip(JOINT_NAMES, final_positions))
        )
        if not reached:
            raise RuntimeError(
                f"Raised target was sent, but joints did not reach tolerance {args.tolerance} rad "
                f"within {args.motion_timeout} s."
            )

        if args.skip_gripper:
            node.get_logger().info("Skipping gripper demo.")
            return

        node.wait_for_publisher_subscriber(
            node.joint_states_pub, "/control/joint_states", args.subscriber_timeout
        )
        if GRIPPER_NAME not in node.feedback_positions():
            node.get_logger().warning(
                "Feedback does not include gripper. Gripper commands require effector_type=agx_gripper."
            )

        open_width = clamp(args.gripper_open, 0.0, 0.05)
        closed_width = clamp(args.gripper_closed, 0.0, open_width)
        effort = clamp(args.gripper_effort, 0.5, 1.0)

        for cycle in range(max(0, args.gripper_cycles)):
            node.get_logger().info(f"Gripper cycle {cycle + 1}/{args.gripper_cycles}: open")
            node.publish_gripper_target(open_width, effort)
            node.sleep_with_spin(max(0.5, args.gripper_delay))

            node.get_logger().info(f"Gripper cycle {cycle + 1}/{args.gripper_cycles}: close")
            node.publish_gripper_target(closed_width, effort)
            node.sleep_with_spin(max(0.5, args.gripper_delay))

        node.publish_gripper_target(open_width, effort)
        node.get_logger().info("Demo complete.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
