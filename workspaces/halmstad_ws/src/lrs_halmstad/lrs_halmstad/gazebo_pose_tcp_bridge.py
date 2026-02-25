#!/usr/bin/env python3

import socketserver
import threading
import time
import math
from dataclasses import dataclass
from typing import Dict

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node


@dataclass
class PoseSample:
    x: float
    y: float
    z: float
    yaw: float
    updated_monotonic: float


class _PoseTcpServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, server_address, bridge_node):
        self.bridge_node = bridge_node
        super().__init__(server_address, _PoseRequestHandler)


class _PoseRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        while rclpy.ok():
            try:
                raw = self.rfile.readline()
            except OSError:
                break

            if not raw:
                break

            command = raw.decode("ascii", errors="ignore").strip().upper()
            if command == "GET":
                line = self.server.bridge_node.build_snapshot_line()
                self.wfile.write((line + "\n").encode("ascii"))
                self.wfile.flush()
            else:
                self.wfile.write(b"ERR unsupported command\n")
                self.wfile.flush()


class GazeboPoseTcpBridge(Node):
    def __init__(self):
        super().__init__("gazebo_pose_tcp_bridge")

        self.declare_parameter("odom_topic", "/a201_0000/platform/odom")
        self.declare_parameter("model_name", "robot")
        self.declare_parameter("odom_topics", ["/a201_0000/platform/odom", "/dji0/pose_cmd/odom"])
        self.declare_parameter("model_names", ["robot", "dji0"])
        self.declare_parameter("bind_host", "127.0.0.1")
        self.declare_parameter("port", 5555)
        self.declare_parameter("stale_timeout_sec", 2.0)

        self._odom_topic = self.get_parameter("odom_topic").get_parameter_value().string_value
        self._model_name = self.get_parameter("model_name").get_parameter_value().string_value
        self._odom_topics = [str(v) for v in self.get_parameter("odom_topics").value if str(v)]
        self._model_names = [str(v) for v in self.get_parameter("model_names").value if str(v)]
        self._bind_host = self.get_parameter("bind_host").get_parameter_value().string_value
        self._port = int(self.get_parameter("port").get_parameter_value().integer_value)
        self._stale_timeout_sec = float(self.get_parameter("stale_timeout_sec").get_parameter_value().double_value)

        self._latest_poses: Dict[str, PoseSample] = {}
        self._lock = threading.Lock()
        self._odom_subs = []
        pairs = self._resolve_odom_model_pairs()
        for model_name, odom_topic in pairs:
            sub = self.create_subscription(Odometry, odom_topic, self._make_odom_callback(model_name), 10)
            self._odom_subs.append(sub)

        self._server = _PoseTcpServer((self._bind_host, self._port), self)
        self._server_thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._server_thread.start()

        self.get_logger().info(
            f"Serving Gazebo pose snapshot TCP bridge on {self._bind_host}:{self._port} "
            f"from odom/model pairs: {pairs}"
        )

    def _resolve_odom_model_pairs(self):
        if self._odom_topics and self._model_names:
            if len(self._odom_topics) != len(self._model_names):
                self.get_logger().warn(
                    "Parameter length mismatch: odom_topics and model_names must have the same length. "
                    "Falling back to odom_topic/model_name."
                )
            else:
                return list(zip(self._model_names, self._odom_topics))
        return [(self._model_name, self._odom_topic)]

    def _make_odom_callback(self, model_name: str):
        def _on_odom(msg: Odometry) -> None:
            self._update_pose_sample(model_name, msg)

        return _on_odom

    def _update_pose_sample(self, model_name: str, msg: Odometry) -> None:
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        # Use projected body-forward heading instead of raw Euler yaw extraction.
        # This is more stable when small roll/pitch is present.
        qx = float(orientation.x)
        qy = float(orientation.y)
        qz = float(orientation.z)
        qw = float(orientation.w)
        qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if qnorm > 0.0:
            qx /= qnorm
            qy /= qnorm
            qz /= qnorm
            qw /= qnorm
        fx = 1.0 - 2.0 * (qy * qy + qz * qz)
        fy = 2.0 * (qx * qy + qw * qz)
        yaw = math.atan2(fy, fx)
        sample = PoseSample(
            x=float(position.x),
            y=float(position.y),
            z=float(position.z),
            yaw=float(yaw),
            updated_monotonic=time.monotonic(),
        )
        with self._lock:
            self._latest_poses[model_name] = sample

    def build_snapshot_line(self) -> str:
        now = time.monotonic()
        with self._lock:
            entries = []
            for name, sample in self._latest_poses.items():
                if self._stale_timeout_sec > 0 and (now - sample.updated_monotonic) > self._stale_timeout_sec:
                    continue
                entries.append((name, sample))

        entries.sort(key=lambda item: item[0])
        parts = [str(len(entries))]
        for name, sample in entries:
            parts.extend([
                name,
                f"{sample.x:.6f}",
                f"{sample.y:.6f}",
                f"{sample.z:.6f}",
                f"{sample.yaw:.6f}",
            ])
        return " ".join(parts)

    def destroy_node(self):
        if hasattr(self, "_server"):
            self._server.shutdown()
            self._server.server_close()
        if hasattr(self, "_server_thread") and self._server_thread.is_alive():
            self._server_thread.join(timeout=1.0)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GazeboPoseTcpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
