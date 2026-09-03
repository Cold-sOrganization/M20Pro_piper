#!/usr/bin/env python3
"""Supervised Piper joint-6 out-and-back motion test for ROS 2 Foxy."""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from agx_arm_msgs.msg import AgxArmStatus


JOINT_NAMES = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
ARM_NODE_NAME = "agx_arm_ctrl_single_node"


class PiperSwingTest(Node):
    def __init__(self):
        super().__init__("piper_joint6_swing_test")
        self.positions = None
        self.feedback_time = 0.0
        self.arm_status = None
        self.motion_started = False

        self.create_subscription(
            JointState, "/feedback/joint_states", self._joint_callback, 10
        )
        self.create_subscription(
            AgxArmStatus, "/feedback/arm_status", self._status_callback, 10
        )
        self.command_pub = self.create_publisher(
            JointState, "/control/move_j", 10
        )
        self.enable_client = self.create_client(SetBool, "/enable_agx_arm")
        self.gate_client = self.create_client(SetBool, "/control_enable")

    def _joint_callback(self, msg):
        if len(msg.name) != len(msg.position):
            return
        values = dict(zip(msg.name, msg.position))
        if not all(name in values for name in JOINT_NAMES):
            return
        positions = [float(values[name]) for name in JOINT_NAMES]
        if not all(math.isfinite(value) for value in positions):
            return
        self.positions = positions
        self.feedback_time = time.monotonic()

    def _status_callback(self, msg):
        self.arm_status = msg

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def require_single_arm_node(self):
        self.spin_for(4.0)
        matches = [
            (name, namespace)
            for name, namespace in self.get_node_names_and_namespaces()
            if name == ARM_NODE_NAME
        ]
        if len(matches) != 1:
            raise RuntimeError(
                "Expected exactly one {} node, found {}. Stop duplicate nodes."
                .format(ARM_NODE_NAME, len(matches))
            )

    def require_feedback(self, timeout=8.0):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            fresh = time.monotonic() - self.feedback_time < 0.5
            if self.positions is not None and self.arm_status is not None and fresh:
                break
        else:
            raise RuntimeError(
                "No fresh Piper feedback. Wait for 'Agx_arm feedback is ready'."
            )

        if int(self.arm_status.err_status) != 0:
            raise RuntimeError(
                "Piper err_status is {}, refusing motion."
                .format(self.arm_status.err_status)
            )
        if any(bool(value) for value in self.arm_status.joint_angle_limit):
            raise RuntimeError("A joint angle limit flag is active, refusing motion.")
        if any(bool(value) for value in self.arm_status.communication_status_joint):
            raise RuntimeError("A joint communication error flag is active.")

    def call_set_bool(self, client, value, label, timeout=10.0):
        if not client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError("{} service is unavailable.".format(label))
        request = SetBool.Request()
        request.data = bool(value)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise RuntimeError("{} service timed out.".format(label))
        response = future.result()
        if response is None or not response.success:
            message = "no response" if response is None else response.message
            raise RuntimeError("{} failed: {}".format(label, message))
        print("{}: {}".format(label, response.message), flush=True)

    def publish_target(self, positions):
        if len(positions) != 6:
            raise RuntimeError("A complete six-joint target is required.")
        message = JointState()
        message.name = list(JOINT_NAMES)
        message.position = [float(value) for value in positions]
        for _ in range(5):
            message.header.stamp = self.get_clock().now().to_msg()
            self.command_pub.publish(message)
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_until_target(self, target, timeout=12.0, tolerance=0.02):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.positions is None:
                continue
            error = max(abs(a - b) for a, b in zip(self.positions, target))
            if error <= tolerance:
                return
        raise RuntimeError("Motion did not reach target before timeout.")

    def best_effort_safe_stop(self):
        for client, value, label in (
            (self.gate_client, False, "close control gate"),
            (self.enable_client, False, "disable Piper"),
        ):
            try:
                self.call_set_bool(client, value, label, timeout=4.0)
            except Exception as exc:
                print("WARNING: {}".format(exc), file=sys.stderr, flush=True)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Move Piper joint6 out and back under supervision."
    )
    parser.add_argument(
        "--delta",
        type=float,
        default=0.15,
        help="Joint6 relative excursion in radians (default: 0.15).",
    )
    parser.add_argument(
        "--hold",
        type=float,
        default=1.5,
        help="Seconds to hold at the excursion target (default: 1.5).",
    )
    parser.add_argument(
        "--execute",
        action="store_true",
        help="Required safety acknowledgement; without it no motion occurs.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    if not args.execute:
        print("Refusing motion: add --execute after clearing the workspace.")
        return 2
    if not 0.05 <= abs(args.delta) <= 0.20:
        print("Refusing motion: --delta must be between 0.05 and 0.20 rad.")
        return 2
    if not 0.0 <= args.hold <= 5.0:
        print("Refusing motion: --hold must be between 0 and 5 seconds.")
        return 2

    rclpy.init()
    node = PiperSwingTest()
    normal_return_completed = False
    try:
        node.require_single_arm_node()
        node.require_feedback()
        start = list(node.positions)
        target = list(start)

        delta = args.delta
        if not -1.5 <= start[5] + delta <= 1.5:
            delta = -delta
        if not -1.5 <= start[5] + delta <= 1.5:
            raise RuntimeError("Joint6 start position is outside the test envelope.")
        target[5] += delta

        print("Start : " + " ".join("{:.4f}".format(x) for x in start))
        print("Target: " + " ".join("{:.4f}".format(x) for x in target))
        print("Joint6 excursion: {:.4f} rad ({:.1f} deg)".format(
            delta, math.degrees(delta)
        ))
        print("Motion begins in 3 seconds. Press Ctrl+C to abort.", flush=True)
        node.spin_for(3.0)

        node.call_set_bool(node.enable_client, True, "enable Piper")
        node.call_set_bool(node.gate_client, True, "open control gate")
        node.motion_started = True

        print("Moving to excursion target...", flush=True)
        node.publish_target(target)
        node.wait_until_target(target)
        node.spin_for(args.hold)

        print("Returning to the captured start position...", flush=True)
        node.publish_target(start)
        node.wait_until_target(start)
        normal_return_completed = True
        print("Return completed.", flush=True)
        return 0
    except KeyboardInterrupt:
        print("Interrupted by operator.", file=sys.stderr, flush=True)
        return 130
    except Exception as exc:
        print("ERROR: {}".format(exc), file=sys.stderr, flush=True)
        return 1
    finally:
        if node.motion_started or normal_return_completed:
            node.best_effort_safe_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
