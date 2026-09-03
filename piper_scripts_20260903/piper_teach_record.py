#!/usr/bin/env python3
import argparse
import json
import math
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from agx_arm_msgs.msg import AgxArmStatus
except Exception:
    AgxArmStatus = None


JOINT_NAMES = [f"joint{i}" for i in range(1, 7)]
GRIPPER_NAME = "gripper"


class TeachRecorder(Node):
    def __init__(self):
        super().__init__("piper_teach_recorder")
        self.latest_joint_state = None
        self.latest_status = None
        self.create_subscription(
            JointState, "/feedback/joint_states", self._on_joint_state, 10
        )
        if AgxArmStatus is not None:
            self.create_subscription(
                AgxArmStatus, "/feedback/arm_status", self._on_arm_status, 10
            )

    def _on_joint_state(self, msg):
        self.latest_joint_state = msg

    def _on_arm_status(self, msg):
        self.latest_status = msg

    def wait_for_feedback(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and self.latest_joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_joint_state is None:
            raise RuntimeError("No /feedback/joint_states received.")
        if GRIPPER_NAME not in self.latest_joint_state.name:
            self.get_logger().warning(
                "Feedback does not include gripper. Start the Piper ROS node with "
                "effector_type=agx_gripper if you want to record gripper motion."
            )

    def wait_for_teach_mode(self, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.is_teach_mode():
                return
        raise RuntimeError("Teach mode was not detected before timeout.")

    def is_teach_mode(self):
        if self.latest_status is None:
            return False
        return self.latest_status.ctrl_mode == 2 or self.latest_status.teach_status == 1

    def current_sample(self, start_time):
        msg = self.latest_joint_state
        positions = dict(zip(msg.name, msg.position))
        missing = [name for name in JOINT_NAMES if name not in positions]
        if missing:
            raise RuntimeError(f"Joint feedback is incomplete. Missing: {missing}")

        joints = [float(positions[name]) for name in JOINT_NAMES]
        if any(not math.isfinite(value) for value in joints):
            raise RuntimeError("Joint feedback contains non-finite values.")

        sample = {
            "t": round(time.monotonic() - start_time, 4),
            "joints": joints,
        }
        if GRIPPER_NAME in positions:
            gripper = float(positions[GRIPPER_NAME])
            if math.isfinite(gripper):
                sample["gripper"] = gripper
        return sample


def parse_args():
    parser = argparse.ArgumentParser(
        description="Record Piper teaching/dragging trajectory from /feedback/joint_states."
    )
    parser.add_argument(
        "--output",
        default="/home/user/piper_recorded_trajectory.json",
        help="Output trajectory JSON path.",
    )
    parser.add_argument("--duration", type=float, default=20.0)
    parser.add_argument("--rate", type=float, default=20.0)
    parser.add_argument("--feedback-timeout", type=float, default=5.0)
    parser.add_argument("--start-delay", type=float, default=3.0)
    parser.add_argument(
        "--wait-teach",
        action="store_true",
        help="Wait until /feedback/arm_status reports teach mode before recording.",
    )
    parser.add_argument("--teach-timeout", type=float, default=30.0)
    return parser.parse_args()


def main():
    args = parse_args()
    if args.duration <= 0:
        raise SystemExit("--duration must be positive.")
    if args.rate <= 0 or args.rate > 100:
        raise SystemExit("--rate must be in (0, 100].")

    rclpy.init()
    node = TeachRecorder()
    try:
        node.wait_for_feedback(args.feedback_timeout)
        if args.wait_teach:
            if AgxArmStatus is None:
                raise RuntimeError("agx_arm_msgs/AgxArmStatus is unavailable.")
            node.get_logger().info("Waiting for teach mode...")
            node.wait_for_teach_mode(args.teach_timeout)

        if args.start_delay > 0:
            node.get_logger().info(f"Recording starts in {args.start_delay:.1f} s.")
            end_delay = time.monotonic() + args.start_delay
            while rclpy.ok() and time.monotonic() < end_delay:
                rclpy.spin_once(node, timeout_sec=0.1)

        period = 1.0 / args.rate
        samples = []
        start_time = time.monotonic()
        next_sample_time = start_time
        end_time = start_time + args.duration

        node.get_logger().info(
            f"Recording {args.duration:.1f} s at {args.rate:.1f} Hz to {args.output}"
        )
        while rclpy.ok() and time.monotonic() < end_time:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_sample_time:
                samples.append(node.current_sample(start_time))
                next_sample_time += period

        data = {
            "format": "piper_joint_trajectory_v1",
            "created_unix": time.time(),
            "joint_names": JOINT_NAMES,
            "rate_hz": args.rate,
            "samples": samples,
        }

        os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
        tmp_path = args.output + ".tmp"
        with open(tmp_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2)
            f.write("\n")
        os.replace(tmp_path, args.output)
        node.get_logger().info(f"Saved {len(samples)} samples to {args.output}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
