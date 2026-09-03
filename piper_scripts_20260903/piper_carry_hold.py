#!/usr/bin/env python3
import argparse
import math
import subprocess
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
GRIPPER_NAME = "gripper"
CARRY_POSITION = [0.0, 1.05, -1.10, 0.0, 1.10, 0.0]

JOINT_LIMITS = {
    "joint1": (-2.617994, 2.617994),
    "joint2": (0.0, 3.141593),
    "joint3": (-2.967060, 0.0),
    "joint4": (-1.745330, 1.745330),
    "joint5": (-1.221730, 1.221730),
    "joint6": (-2.094395, 2.094395),
}


class PiperCarryHold(Node):
    def __init__(self):
        super().__init__("piper_carry_hold")
        self.latest_feedback = None
        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_feedback, 10
        )
        self.move_j_pub = self.create_publisher(JointState, "/control/move_j", 10)
        self.gripper_pub = self.create_publisher(
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

    def feedback_positions(self):
        positions = dict(zip(self.latest_feedback.name, self.latest_feedback.position))
        missing = [name for name in JOINT_NAMES if name not in positions]
        if missing:
            raise RuntimeError(f"Joint feedback is incomplete. Missing: {missing}")

        joints = [float(positions[name]) for name in JOINT_NAMES]
        if any(not math.isfinite(value) for value in joints):
            raise RuntimeError("Joint feedback contains a non-finite value.")

        gripper = positions.get(GRIPPER_NAME)
        if gripper is not None:
            gripper = clamp_gripper(gripper)
        return joints, gripper

    def wait_for_subscriber(self, publisher, topic, timeout):
        deadline = time.monotonic() + timeout
        while (
            rclpy.ok()
            and publisher.get_subscription_count() == 0
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(self, timeout_sec=0.1)
        if publisher.get_subscription_count() == 0:
            raise RuntimeError(f"No subscriber found on {topic}.")

    def publish_arm(self, joints):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = [float(value) for value in joints]
        self.move_j_pub.publish(msg)

    def publish_gripper(self, width, effort):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [GRIPPER_NAME]
        msg.position = [clamp_gripper(width)]
        msg.effort = [clamp_effort(effort)]
        self.gripper_pub.publish(msg)

    def publish_carry_target(self, gripper_width, gripper_effort):
        self.publish_arm(CARRY_POSITION)
        self.publish_gripper(gripper_width, gripper_effort)

    def sleep_with_spin(self, seconds):
        deadline = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, remaining)))


def clamp_gripper(width):
    return max(0.0, min(0.1, float(width)))


def clamp_effort(effort):
    return max(0.5, min(3.0, float(effort)))


def validate_carry_position():
    for name, value in zip(JOINT_NAMES, CARRY_POSITION):
        low, high = JOINT_LIMITS[name]
        if value < low or value > high:
            raise RuntimeError(f"Carry target {name}={value} is outside [{low}, {high}].")


def smootherstep(ratio):
    ratio = max(0.0, min(1.0, ratio))
    return ratio * ratio * ratio * (ratio * (ratio * 6.0 - 15.0) + 10.0)


def smooth_joint_points(start, target, speed, rate):
    max_delta = max(abs(a - b) for a, b in zip(start, target))
    duration = max(0.5, max_delta / speed)
    steps = max(1, int(math.ceil(duration * rate)))
    points = []
    for step in range(1, steps + 1):
        blend = smootherstep(step / float(steps))
        points.append([a + (b - a) * blend for a, b in zip(start, target)])
    return points, duration


def smooth_scalar_points(start, target, speed, rate):
    delta = target - start
    if abs(delta) < 1e-6:
        return [target], 0.0
    duration = max(0.5, abs(delta) / speed)
    steps = max(1, int(math.ceil(duration * rate)))
    return [
        start + delta * smootherstep(step / float(steps))
        for step in range(1, steps + 1)
    ], duration


def close_gripper(node, start_width, target_width, speed, rate, effort):
    widths, duration = smooth_scalar_points(start_width, target_width, speed, rate)
    node.get_logger().info(
        f"Closing gripper from {start_width:.4f} m to {target_width:.4f} m "
        f"over {duration:.2f} s at {rate:.1f} Hz."
    )
    for index, width in enumerate(widths):
        node.publish_gripper(width, effort)
        node.sleep_with_spin(1.0 / rate)
        if index % max(1, int(rate)) == 0 or index == len(widths) - 1:
            node.get_logger().info(
                f"Gripper step {index + 1}/{len(widths)}: {width:.4f} m"
            )


def move_to_carry(node, current, speed, rate, gripper_width, gripper_effort):
    points, duration = smooth_joint_points(current, CARRY_POSITION, speed, rate)
    node.get_logger().info(
        f"Moving smoothly to carry pose over {duration:.2f} s at {rate:.1f} Hz."
    )
    for index, point in enumerate(points):
        node.publish_arm(point)
        node.publish_gripper(gripper_width, gripper_effort)
        node.sleep_with_spin(1.0 / rate)
        if index % max(1, int(rate)) == 0 or index == len(points) - 1:
            node.get_logger().info(f"Carry approach step {index + 1}/{len(points)}")


def hold_until_ctrl_c(node, rate, gripper_width, gripper_effort):
    period = 1.0 / rate
    node.get_logger().info(
        "Carry pose reached and held. Press Ctrl+C to run the zero script and exit."
    )
    while rclpy.ok():
        node.publish_carry_target(gripper_width, gripper_effort)
        node.sleep_with_spin(period)


def run_zero_script(path, tolerance, motion_timeout):
    command = [
        sys.executable,
        path,
        "--tolerance",
        str(tolerance),
        "--motion-timeout",
        str(motion_timeout),
    ]
    print("Running zero script: " + " ".join(command), flush=True)
    subprocess.run(command, check=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description=(
            "Slowly enter the Piper carry pose, hold it, then run the zero script "
            "on Ctrl+C."
        )
    )
    parser.add_argument(
        "--arm-speed",
        type=float,
        default=0.25,
        help="Average arm transition speed in rad/s.",
    )
    parser.add_argument(
        "--command-rate",
        type=float,
        default=20.0,
        help="Smooth command publishing rate in Hz.",
    )
    parser.add_argument("--gripper-width", type=float, default=0.0)
    parser.add_argument(
        "--gripper-speed",
        type=float,
        default=0.04,
        help="Average gripper transition speed in m/s.",
    )
    parser.add_argument("--gripper-effort", type=float, default=1.0)
    parser.add_argument("--hold-rate", type=float, default=2.0)
    parser.add_argument("--feedback-timeout", type=float, default=5.0)
    parser.add_argument("--subscriber-timeout", type=float, default=5.0)
    parser.add_argument("--zero-script", default="/home/user/piper_zero.py")
    parser.add_argument("--zero-tolerance", type=float, default=0.03)
    parser.add_argument("--zero-motion-timeout", type=float, default=30.0)
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def validate_args(args):
    positive = {
        "--arm-speed": args.arm_speed,
        "--command-rate": args.command_rate,
        "--gripper-speed": args.gripper_speed,
        "--hold-rate": args.hold_rate,
        "--feedback-timeout": args.feedback_timeout,
        "--subscriber-timeout": args.subscriber_timeout,
        "--zero-tolerance": args.zero_tolerance,
        "--zero-motion-timeout": args.zero_motion_timeout,
    }
    for name, value in positive.items():
        if value <= 0:
            raise SystemExit(f"{name} must be positive.")
    if not 0.0 <= args.gripper_width <= 0.1:
        raise SystemExit("--gripper-width must be in [0.0, 0.1] m.")
    if not 0.5 <= args.gripper_effort <= 3.0:
        raise SystemExit("--gripper-effort must be in [0.5, 3.0].")


def main():
    args = parse_args()
    validate_args(args)
    validate_carry_position()

    run_zero_after_ctrl_c = False
    rclpy.init()
    node = PiperCarryHold()
    try:
        node.wait_for_feedback(args.feedback_timeout)
        current_joints, current_gripper = node.feedback_positions()
        node.get_logger().info(
            "Current joints: "
            + ", ".join(
                f"{name}={value:.4f}"
                for name, value in zip(JOINT_NAMES, current_joints)
            )
        )
        node.get_logger().info(
            "Carry target: "
            + ", ".join(
                f"{name}={value:.4f}"
                for name, value in zip(JOINT_NAMES, CARRY_POSITION)
            )
        )

        if args.dry_run:
            node.get_logger().info("Dry run complete; no motion commands sent.")
            return

        node.wait_for_subscriber(
            node.move_j_pub, "/control/move_j", args.subscriber_timeout
        )
        node.wait_for_subscriber(
            node.gripper_pub, "/control/joint_states", args.subscriber_timeout
        )

        if current_gripper is None:
            current_gripper = 0.1
            node.get_logger().warning(
                "No gripper position in feedback; using 0.1000 m as the gradual-close start."
            )

        try:
            close_gripper(
                node,
                current_gripper,
                args.gripper_width,
                args.gripper_speed,
                args.command_rate,
                args.gripper_effort,
            )
            move_to_carry(
                node,
                current_joints,
                args.arm_speed,
                args.command_rate,
                args.gripper_width,
                args.gripper_effort,
            )
            hold_until_ctrl_c(
                node,
                args.hold_rate,
                args.gripper_width,
                args.gripper_effort,
            )
        except KeyboardInterrupt:
            node.get_logger().info("Ctrl+C received.")
            run_zero_after_ctrl_c = True
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if run_zero_after_ctrl_c:
        run_zero_script(
            args.zero_script, args.zero_tolerance, args.zero_motion_timeout
        )


if __name__ == "__main__":
    main()
