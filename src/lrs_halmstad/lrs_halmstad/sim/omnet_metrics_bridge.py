#!/usr/bin/env python3
"""
omnet_metrics_bridge — ROS2 node that connects to the OMNeT++ OmnetMetricsServer
TCP server and republishes network metrics as ROS2 topics.

Legacy OMNeT metrics servers sent:
    <simtime_s> <distance_m> <rssi_dbm> <snir_db> <per> <radio_distance_m>
The current LoRa metrics server sends:
    <simtime_s> <link_distance_m> <rssi_dbm> <snir_db> <per> <radio_distance_m> <pdr> <latency_s> <jitter_s>

Published topics:
    /omnet/rssi_dbm           (std_msgs/Float64)  — dBm
    /omnet/snir_db            (std_msgs/Float64)  — dB
    /omnet/packet_error_rate  (std_msgs/Float64)  — 0..1
    /omnet/packet_delivery_ratio (std_msgs/Float64) — 0..1
    /omnet/latency_s          (std_msgs/Float64)  — seconds
    /omnet/jitter_s           (std_msgs/Float64)  — seconds
    /omnet/sim_time           (std_msgs/Float64)  — OMNeT simulation time (s)
    /omnet/radio_distance     (std_msgs/Float64)  — RSSI/path-loss-derived metres, or decoded link payload when RSSI distance is disabled
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
        self.declare_parameter("read_timeout_s", 5.0)

        self._host = self.get_parameter("omnet_host").get_parameter_value().string_value
        self._port = int(self.get_parameter("omnet_port").get_parameter_value().integer_value)
        self._reconnect_interval = float(
            self.get_parameter("reconnect_interval_s").get_parameter_value().double_value
        )
        self._read_timeout = float(
            self.get_parameter("read_timeout_s").get_parameter_value().double_value
        )
        if self._read_timeout <= 0.0:
            self._read_timeout = 5.0

        self._pub_rssi          = self.create_publisher(Float64, "/omnet/rssi_dbm", 5)
        self._pub_snir          = self.create_publisher(Float64, "/omnet/snir_db", 5)
        self._pub_per           = self.create_publisher(Float64, "/omnet/packet_error_rate", 1)
        self._pub_pdr           = self.create_publisher(Float64, "/omnet/packet_delivery_ratio", 1)
        self._pub_latency       = self.create_publisher(Float64, "/omnet/latency_s", 10)
        self._pub_jitter        = self.create_publisher(Float64, "/omnet/jitter_s", 10)
        self._pub_simtime       = self.create_publisher(Float64, "/omnet/sim_time", 10)
        self._pub_radio_dist    = self.create_publisher(Float64, "/omnet/radio_distance", 5)

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
        sock.settimeout(self._read_timeout)
        self._sock = sock
        self.get_logger().info(
            f"Connected to OMNeT metrics server at {self._host}:{self._port}"
        )

    def _receive_loop(self) -> None:
        assert self._sock is not None
        buf = b""
        last_line_at = time.monotonic()
        while not self._stop_event.is_set():
            try:
                chunk = self._sock.recv(256)
            except socket.timeout as exc:
                age = time.monotonic() - last_line_at
                raise TimeoutError(
                    f"no OMNeT metrics received for {age:.1f}s after connecting"
                ) from exc
            if not chunk:
                raise ConnectionResetError("OMNeT closed the connection")
            buf += chunk
            while b"\n" in buf:
                line, buf = buf.split(b"\n", 1)
                self._handle_line(line.decode("ascii", errors="ignore").strip())
                last_line_at = time.monotonic()

    def _handle_line(self, line: str) -> None:
        if not line:
            return
        parts = line.split()
        if len(parts) not in (6, 8, 9):
            self.get_logger().debug(f"Unexpected metrics line: {line!r}")
            return
        try:
            values = [float(p) for p in parts]
        except ValueError:
            self.get_logger().debug(f"Could not parse metrics line: {line!r}")
            return
        if len(values) == 8:
            link_dist = float("nan")
            sim_time, rssi, snir, per, radio_dist, pdr, latency_s, jitter_s = values
        elif len(values) == 9:
            sim_time, link_dist, rssi, snir, per, radio_dist = values[:6]
            pdr, latency_s, jitter_s = values[6:]
        else:
            sim_time, link_dist, rssi, snir, per, radio_dist = values
            pdr = latency_s = jitter_s = float("nan")

        def _pub(pub, val: float) -> None:
            msg = Float64()
            msg.data = val
            pub.publish(msg)

        _pub(self._pub_simtime,    sim_time)
        _pub(self._pub_rssi,       rssi)
        _pub(self._pub_snir,       snir)
        _pub(self._pub_per,        per)
        _pub(self._pub_pdr,        pdr)
        _pub(self._pub_latency,    latency_s)
        _pub(self._pub_jitter,     jitter_s)
        if radio_dist != radio_dist and link_dist == link_dist:
            radio_dist = link_dist
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
