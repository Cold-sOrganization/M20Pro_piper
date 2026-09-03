#!/usr/bin/env python3
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


delta = float(sys.argv[1]) if len(sys.argv) > 1 else 0.02
if abs(delta) > 0.02:
    raise SystemExit("拒绝执行：首次演示位移绝对值不得超过 0.02 rad")

rclpy.init()
node = Node("piper_safe_joint6_step")
latest = None


def on_feedback(msg):
    global latest
    latest = msg


sub = node.create_subscription(JointState, "/feedback/joint_states", on_feedback, 10)
pub = node.create_publisher(JointState, "/control/move_j", 10)

deadline = time.monotonic() + 5.0
while rclpy.ok() and latest is None and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

if latest is None:
    raise SystemExit("错误：5 秒内未收到 /feedback/joint_states")

positions = dict(zip(latest.name, latest.position))
names = [f"joint{i}" for i in range(1, 7)]
if any(name not in positions for name in names):
    raise SystemExit(f"错误：反馈关节不完整：{latest.name}")

target = JointState()
target.header.stamp = node.get_clock().now().to_msg()
target.name = names
target.position = [float(positions[name]) for name in names]
target.position[5] += delta

deadline = time.monotonic() + 3.0
while rclpy.ok() and pub.get_subscription_count() == 0 and time.monotonic() < deadline:
    rclpy.spin_once(node, timeout_sec=0.1)

if pub.get_subscription_count() == 0:
    raise SystemExit("错误：未发现 /control/move_j 订阅者")

pub.publish(target)
time.sleep(1.0)
print("已发送完整六轴目标：", list(zip(target.name, target.position)))

node.destroy_node()
rclpy.shutdown()
