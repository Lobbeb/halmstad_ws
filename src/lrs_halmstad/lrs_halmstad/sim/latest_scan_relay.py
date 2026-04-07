#!/usr/bin/env python3

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class LatestScanRelay(Node):
    def __init__(self) -> None:
        super().__init__("latest_scan_relay")

        self.declare_parameter("input_topic", "/scan")
        self.declare_parameter("output_topic", "/scan_relay")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("max_age_s", 1.0)
        self.declare_parameter("restamp", True)
        self.declare_parameter("startup_hold_s", 0.0)

        input_topic = str(self.get_parameter("input_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        publish_hz = max(0.1, float(self.get_parameter("publish_hz").value))
        self.max_age_s = max(0.0, float(self.get_parameter("max_age_s").value))
        self.restamp = bool(self.get_parameter("restamp").value)
        startup_hold_s = max(0.0, float(self.get_parameter("startup_hold_s").value))

        sensor_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.latest_scan: LaserScan | None = None
        self.latest_received_ns: int | None = None
        self.ready_after_ns = self.get_clock().now().nanoseconds + int(startup_hold_s * 1e9)

        self.create_subscription(LaserScan, input_topic, self._on_scan, sensor_qos)
        self.publisher = self.create_publisher(LaserScan, output_topic, sensor_qos)
        self.create_timer(1.0 / publish_hz, self._on_timer)

        self.get_logger().info(
            f"Relaying latest scan from {input_topic} to {output_topic} at {publish_hz:.2f} Hz "
            f"(restamp={self.restamp}, max_age_s={self.max_age_s:.2f}, startup_hold_s={startup_hold_s:.2f})"
        )

    def _on_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.latest_received_ns = self.get_clock().now().nanoseconds

    def _on_timer(self) -> None:
        if self.latest_scan is None or self.latest_received_ns is None:
            return

        now_ns = self.get_clock().now().nanoseconds
        if now_ns < self.ready_after_ns:
            return

        age_s = (now_ns - self.latest_received_ns) / 1e9
        if self.max_age_s > 0.0 and age_s > self.max_age_s:
            return

        msg = copy.deepcopy(self.latest_scan)
        if self.restamp:
            msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LatestScanRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
