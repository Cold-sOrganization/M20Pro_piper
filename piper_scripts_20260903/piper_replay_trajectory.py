#!/usr/bin/env python3
import argparse
import json
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
GRIPPER_NAME = "gripper"

JOINT_LIMITS = {
    "joint1": (-2.617994, 2.617994),
    "joint2": (0.0, 3.141593),
    "joint3": (-2.967060, 0.0),
    "joint4": (-1.745330, 1.745330),
    "joint5": (-1.221730, 1.221730),
    "joint6": (-2.094395, 2.094395),
}


class TrajectoryReplayer(Node):
    def __init__(self):
        super().__init__("piper_trajectory_replayer")
        self.latest_feedback = None
        self.create_subscription(
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

    def current_joints(self):
        positions = dict(zip(self.latest_feedback.name, self.latest_feedback.position))
        missing = [name for name in JOINT_NAMES if name not in positions]
        if missing:
            raise RuntimeError(f"Joint feedback is incomplete. Missing: {missing}")
        return [float(positions[name]) for name in JOINT_NAMES]

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
        self.get_logger().info(
            "Sent arm target: "
            + ", ".join(f"{name}={value:.4f}" for name, value in zip(JOINT_NAMES, joints))
        )

    def publish_gripper(self, width, effort):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [GRIPPER_NAME]
        msg.position = [max(0.0, min(0.1, float(width)))]
        msg.effort = [max(0.5, min(3.0, float(effort)))]
        self.joint_states_pub.publish(msg)

    def sleep_with_spin(self, seconds):
        deadline = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < deadline:
            remaining = deadline - time.monotonic()
            rclpy.spin_once(self, timeout_sec=min(0.1, max(0.0, remaining)))


def validate_sample(sample):
    joints = sample.get("joints")
    if not isinstance(joints, list) or len(joints) != 6:
        raise ValueError("Each sample must contain six joint values.")
    for name, value in zip(JOINT_NAMES, joints):
        if not math.isfinite(float(value)):
            raise ValueError(f"{name} value is not finite: {value}")
        low, high = JOINT_LIMITS[name]
        if float(value) < low or float(value) > high:
            raise ValueError(f"{name} value {value} is outside [{low}, {high}].")
    return [float(value) for value in joints]


def load_trajectory(path, max_step):
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)
    samples = data.get("samples", [])
    if not samples:
        raise ValueError("Trajectory contains no samples.")

    cleaned = []
    last_joints = None
    last_t = None
    for sample in samples:
        joints = validate_sample(sample)
        t = float(sample.get("t", 0.0))
        if last_t is not None and t < last_t:
            raise ValueError("Sample timestamps must be nondecreasing.")
        if last_joints is not None:
            max_delta = max(abs(a - b) for a, b in zip(joints, last_joints))
            if max_delta > max_step:
                raise ValueError(
                    f"Trajectory step is too large: {max_delta:.3f} rad > {max_step:.3f} rad."
                )
        cleaned.append(
            {
                "t": t,
                "joints": joints,
                "gripper": sample.get("gripper"),
            }
        )
        last_joints = joints
        last_t = t
    return cleaned


def interpolate(start, end, max_step):
    max_delta = max(abs(a - b) for a, b in zip(start, end))
    steps = max(1, int(math.ceil(max_delta / max_step)))
    points = []
    for step in range(1, steps + 1):
        ratio = step / float(steps)
        points.append([a + (b - a) * ratio for a, b in zip(start, end)])
    return points


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay a recorded Piper joint trajectory through /control/move_j."
    )
    parser.add_argument(
        "--trajectory",
        default="/home/user/piper_recorded_trajectory.json",
        help="Recorded trajectory JSON path.",
    )
    parser.add_argument(
        "--playback-speed",
        type=float,
        default=None,
        help="Playback speed multiplier: 1.0 original speed, 0.5 half speed, 2.0 double speed.",
    )
    parser.add_argument(
        "--speed-scale",
        type=float,
        default=1.5,
        help="Legacy slowdown factor. Use --playback-speed for new commands.",
    )
    parser.add_argument("--feedback-timeout", type=float, default=5.0)
    parser.add_argument("--subscriber-timeout", type=float, default=5.0)
    parser.add_argument("--start-tolerance", type=float, default=0.25)
    parser.add_argument("--max-start-distance", type=float, default=0.8)
    parser.add_argument("--approach-step", type=float, default=0.05)
    parser.add_argument("--approach-delay", type=float, default=0.8)
    parser.add_argument("--max-step", type=float, default=0.08)
    parser.add_argument("--gripper-effort", type=float, default=1.5)
    parser.add_argument("--skip-gripper", action="store_true")
    parser.add_argument("--dry-run", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.playback_speed is not None:
        if args.playback_speed <= 0:
            raise SystemExit("--playback-speed must be positive.")
        delay_scale = 1.0 / args.playback_speed
    else:
        if args.speed_scale < 1.0:
            raise SystemExit("--speed-scale must be >= 1.0. Use >1.0 for slower playback.")
        delay_scale = args.speed_scale
    if args.start_tolerance <= 0:
        raise SystemExit("--start-tolerance must be positive.")
    if args.max_start_distance <= 0:
        raise SystemExit("--max-start-distance must be positive.")
    if args.approach_step <= 0:
        raise SystemExit("--approach-step must be positive.")
    if args.approach_delay <= 0:
        raise SystemExit("--approach-delay must be positive.")
    if args.max_step <= 0:
        raise SystemExit("--max-step must be positive.")

    trajectory = load_trajectory(args.trajectory, args.max_step)
    first = trajectory[0]["joints"]
    gripper_samples = sum(1 for sample in trajectory if sample["gripper"] is not None)
    gripper_values = [
        float(sample["gripper"]) for sample in trajectory if sample["gripper"] is not None
    ]

    rclpy.init()
    node = TrajectoryReplayer()
    try:
        node.wait_for_feedback(args.feedback_timeout)
        current = node.current_joints()
        start_error = max(abs(a - b) for a, b in zip(current, first))
        node.get_logger().info(f"Loaded {len(trajectory)} points from {args.trajectory}")
        node.get_logger().info(
            f"Playback speed: {1.0 / delay_scale:.2f}x"
        )
        if gripper_samples == 0 and not args.skip_gripper:
            node.get_logger().warning(
                "Trajectory has no gripper samples, so replay will only move the arm."
            )
        elif gripper_values and not args.skip_gripper:
            node.get_logger().info(
                f"Trajectory gripper range: {min(gripper_values):.4f} to "
                f"{max(gripper_values):.4f} m across {gripper_samples} samples"
            )
        node.get_logger().info(f"Distance to first point: {start_error:.3f} rad")
        if start_error > args.max_start_distance:
            raise RuntimeError(
                f"Current pose is too far from the recorded start pose for automatic approach: "
                f"{start_error:.3f} rad > {args.max_start_distance:.3f} rad."
            )

        if args.dry_run:
            node.get_logger().info("Dry run complete; no motion commands sent.")
            return

        node.wait_for_subscriber(node.move_j_pub, "/control/move_j", args.subscriber_timeout)
        if not args.skip_gripper:
            node.wait_for_subscriber(
                node.joint_states_pub, "/control/joint_states", args.subscriber_timeout
            )

        if start_error > args.start_tolerance:
            approach_points = interpolate(current, first, args.approach_step)
            node.get_logger().info(
                f"Approaching recorded start pose in {len(approach_points)} slow steps."
            )
            for index, point in enumerate(approach_points):
                node.publish_arm(point)
                node.sleep_with_spin(args.approach_delay)
                if index % 5 == 0 or index == len(approach_points) - 1:
                    node.get_logger().info(
                        f"Approach step {index + 1}/{len(approach_points)}"
                    )

        previous_t = trajectory[0]["t"]
        for index, sample in enumerate(trajectory):
            delay = (sample["t"] - previous_t) * delay_scale
            node.sleep_with_spin(delay)
            node.publish_arm(sample["joints"])
            if not args.skip_gripper and sample["gripper"] is not None:
                node.publish_gripper(sample["gripper"], args.gripper_effort)
            previous_t = sample["t"]
            if index % 20 == 0:
                if sample["gripper"] is None or args.skip_gripper:
                    node.get_logger().info(
                        f"Replayed {index + 1}/{len(trajectory)} points"
                    )
                else:
                    node.get_logger().info(
                        f"Replayed {index + 1}/{len(trajectory)} points, "
                        f"gripper={float(sample['gripper']):.4f} m"
                    )

        node.get_logger().info("Replay complete.")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
