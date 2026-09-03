#!/usr/bin/env python3
import json
import socket
import threading

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool


class PiperGosBridge(Node):
    def __init__(self):
        super().__init__("piper_tcp_bridge_gos")
        self._client = None
        self._client_lock = threading.Lock()
        self._command_pub = self.create_publisher(JointState, "/control/joint_states", 10)
        self.create_subscription(JointState, "/feedback/joint_states", self._feedback, 10)
        self._enable_client = self.create_client(
            SetBool, "/enable_agx_arm", callback_group=ReentrantCallbackGroup()
        )
        threading.Thread(target=self._server, daemon=True).start()

    def _send(self, payload):
        data = (json.dumps(payload, separators=(",", ":")) + "\n").encode()
        with self._client_lock:
            if self._client:
                try:
                    self._client.sendall(data)
                except OSError:
                    try:
                        self._client.close()
                    except OSError:
                        pass
                    self._client = None

    def _feedback(self, msg):
        self._send(
            {
                "type": "feedback",
                "names": list(msg.name),
                "positions": list(msg.position),
                "velocities": list(msg.velocity),
                "efforts": list(msg.effort),
            }
        )

    def _server(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(("10.21.31.104", 29500))
        server.listen(1)
        self.get_logger().info("Listening for middle Piper bridge on 10.21.31.104:29500")
        while rclpy.ok():
            client, address = server.accept()
            self.get_logger().info(f"Middle Piper bridge connected from {address}")
            with self._client_lock:
                if self._client:
                    self._client.close()
                self._client = client
            try:
                reader = client.makefile("r", encoding="utf-8")
                for line in reader:
                    self._handle(json.loads(line))
            except Exception as exc:
                self.get_logger().warn(f"Middle bridge disconnected: {exc}")
            finally:
                with self._client_lock:
                    if self._client is client:
                        self._client = None
                client.close()

    def _handle(self, payload):
        if payload.get("type") == "command":
            names = payload.get("names", [])
            positions = payload.get("positions", [])
            if len(names) != 6 or len(positions) != 6:
                return
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = [str(v) for v in names]
            msg.position = [float(v) for v in positions]
            self._command_pub.publish(msg)
        elif payload.get("type") == "enable":
            request_id = payload.get("id", "")
            if not self._enable_client.wait_for_service(timeout_sec=1.0):
                self._send(
                    {"type": "service_response", "id": request_id, "success": False,
                     "message": "GOS /enable_agx_arm 服务不可用"}
                )
                return
            request = SetBool.Request()
            request.data = bool(payload.get("data"))
            future = self._enable_client.call_async(request)

            def done(done_future):
                try:
                    remote = done_future.result()
                    response = {"type": "service_response", "id": request_id,
                                "success": bool(remote.success), "message": remote.message}
                except Exception as exc:
                    response = {"type": "service_response", "id": request_id,
                                "success": False, "message": str(exc)}
                self._send(response)

            future.add_done_callback(done)


def main(args=None):
    rclpy.init(args=args)
    node = PiperGosBridge()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
