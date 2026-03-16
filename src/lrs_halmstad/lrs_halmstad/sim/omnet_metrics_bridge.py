#!/usr/bin/env python3
"""
omnet_metrics_bridge — ROS2 node that connects to the OMNeT++ OmnetMetricsServer
TCP server and republishes network metrics as ROS2 topics.

OMNeT sends one ASCII line per update interval:
    <simtime_s> <distance_m> <rssi_dbm> <snir_db> <per> <radio_distance_m>

Published topics:
    /omnet/link_distance      (std_msgs/Float64)  — metres (geometric, from Gazebo positions)
    /omnet/rssi_dbm           (std_msgs/Float64)  — dBm
    /omnet/snir_db            (std_msgs/Float64)  — dB
    /omnet/packet_error_rate  (std_msgs/Float64)  — 0..1
    /omnet/sim_time           (std_msgs/Float64)  — OMNeT simulation time (s)
    /omnet/radio_distance     (std_msgs/Float64)  — metres (FSPL-inverted from RSSI only)
"""

import socket
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class OmnetMetricsBridge(Node):
    def __init__(self):
        super().__init__("omnet_metrics_bridge")

        self.declare_parameter("omnet_host", "127.0.0.1")
        self.declare_parameter("omnet_port", 5556)
        self.declare_parameter("reconnect_interval_s", 2.0)

        self._host = self.get_parameter("omnet_host").get_parameter_value().string_value
        self._port = int(self.get_parameter("omnet_port").get_parameter_value().integer_value)
        self._reconnect_interval = float(
            self.get_parameter("reconnect_interval_s").get_parameter_value().double_value
        )

        self._pub_distance      = self.create_publisher(Float64, "/omnet/link_distance", 10)
        self._pub_rssi          = self.create_publisher(Float64, "/omnet/rssi_dbm", 10)
        self._pub_snir          = self.create_publisher(Float64, "/omnet/snir_db", 10)
        self._pub_per           = self.create_publisher(Float64, "/omnet/packet_error_rate", 10)
        self._pub_simtime       = self.create_publisher(Float64, "/omnet/sim_time", 10)
        self._pub_radio_dist    = self.create_publisher(Float64, "/omnet/radio_distance", 10)

        self._sock: socket.socket | None = None
        self._stop_event = threading.Event()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

        self.get_logger().info(
            f"OMNeT metrics bridge started — connecting to {self._host}:{self._port}"
        )

    # ── background TCP thread ──────────────────────────────────────────────

    def _run_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._connect()
                self._receive_loop()
            except ConnectionRefusedError:
                if not self._stop_event.is_set():
                    self.get_logger().debug(
                        f"OMNeT server not available at {self._host}:{self._port}; "
                        f"retrying in {self._reconnect_interval:.1f}s"
                    )
            except Exception as exc:
                if not self._stop_event.is_set():
                    self.get_logger().warning(
                        f"OMNeT bridge error: {exc}; retrying in "
                        f"{self._reconnect_interval:.1f}s"
                    )
            self._close()
            if not self._stop_event.is_set():
                self._stop_event.wait(self._reconnect_interval)

    def _connect(self) -> None:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(5.0)
        sock.connect((self._host, self._port))
        sock.settimeout(None)  # blocking reads from here
        self._sock = sock
        self.get_logger().info(
            f"Connected to OMNeT metrics server at {self._host}:{self._port}"
        )

    def _receive_loop(self) -> None:
        assert self._sock is not None
        buf = b""
        while not self._stop_event.is_set():
            chunk = self._sock.recv(256)
            if not chunk:
                raise ConnectionResetError("OMNeT closed the connection")
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle_line(line.decode("ascii", errors="ignore").strip())

    def _handle_line(self, line: str) -> None:
        if not line:
            return
        parts = line.split()
        if len(parts) != 6:
            self.get_logger().debug(f"Unexpected metrics line: {line!r}")
            return
        try:
            sim_time, distance, rssi, snir, per, radio_dist = (float(p) for p in parts)
        except ValueError:
            self.get_logger().debug(f"Could not parse metrics line: {line!r}")
            return

        now = self.get_clock().now().to_msg()

        def _pub(pub, val: float) -> None:
            msg = Float64()
            msg.data = val
            pub.publish(msg)

        _pub(self._pub_simtime,    sim_time)
        _pub(self._pub_distance,   distance)
        _pub(self._pub_rssi,       rssi)
        _pub(self._pub_snir,       snir)
        _pub(self._pub_per,        per)
        _pub(self._pub_radio_dist, radio_dist)

    def _close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None

    # ── lifecycle ──────────────────────────────────────────────────────────

    def destroy_node(self) -> None:
        self._stop_event.set()
        self._close()
        if self._thread.is_alive():
            self._thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OmnetMetricsBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
