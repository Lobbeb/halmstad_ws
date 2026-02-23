#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class DriveLidar(Node):
    def __init__(self):
        super().__init__("drive_lidar")

        self.declare_parameter("cmd_vel_topic", "/x2_ugv/cmd_vel")
        self.declare_parameter("scan_topic", "/x2_ugv/scan")
        self.declare_parameter(
            "fallback_scan_topics",
            [
                "/x2_ugv/scan",
                "/scan",
                "/lidar",
                "/x2_ugv/lidar",
                "/world/default/model/x2_ugv/link/base/sensor/lidar/scan",
                "/world/walls/model/x2_ugv/link/base/sensor/lidar/scan",
                "/world/default/model/x2_ugv/model/X2/link/base/sensor/lidar/scan",
                "/world/walls/model/x2_ugv/model/X2/link/base/sensor/lidar/scan",
                "/world/default/model/x2_ugv/link/base/sensor/lidar/lidar",
                "/world/walls/model/x2_ugv/link/base/sensor/lidar/lidar",
            ],
        )
        self.declare_parameter("control_rate_hz", 10.0)
        self.declare_parameter("linear_speed", 0.55)
        self.declare_parameter("creep_speed", 0.18)
        self.declare_parameter("turn_speed", 0.9)
        self.declare_parameter("ugv_mps", -1.0)
        self.declare_parameter("startup_forward_sec", 1.0)
        self.declare_parameter("startup_forward_speed", 0.25)
        self.declare_parameter("safety_distance", 3.0)
        self.declare_parameter("critical_distance", 1.5)
        self.declare_parameter("front_half_angle_deg", 45.0)
        self.declare_parameter("side_min_angle_deg", 35.0)
        self.declare_parameter("side_max_angle_deg", 120.0)
        self.declare_parameter("scan_timeout_sec", 1.0)

        self._cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self._scan_topic = self.get_parameter("scan_topic").value
        fallback_topics_value = self.get_parameter("fallback_scan_topics").value
        if isinstance(fallback_topics_value, str):
            fallback_topics = [fallback_topics_value]
        else:
            fallback_topics = list(fallback_topics_value)
        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._linear_speed = float(self.get_parameter("linear_speed").value)
        self._creep_speed = float(self.get_parameter("creep_speed").value)
        self._turn_speed = float(self.get_parameter("turn_speed").value)
        ugv_mps = float(self.get_parameter("ugv_mps").value)
        self._startup_forward_sec = float(self.get_parameter("startup_forward_sec").value)
        self._startup_forward_speed = float(self.get_parameter("startup_forward_speed").value)
        self._safety_distance = float(self.get_parameter("safety_distance").value)
        self._critical_distance = float(self.get_parameter("critical_distance").value)
        self._front_half_angle = math.radians(float(self.get_parameter("front_half_angle_deg").value))
        self._side_min_angle = math.radians(float(self.get_parameter("side_min_angle_deg").value))
        self._side_max_angle = math.radians(float(self.get_parameter("side_max_angle_deg").value))
        self._scan_timeout_sec = float(self.get_parameter("scan_timeout_sec").value)

        if ugv_mps > 0.0:
            # Preserve behavior shape while allowing a single speed override from the launcher.
            base_linear = max(self._linear_speed, 1e-6)
            creep_ratio = self._creep_speed / base_linear
            startup_ratio = self._startup_forward_speed / base_linear
            self._linear_speed = ugv_mps
            self._creep_speed = max(0.05, ugv_mps * creep_ratio)
            self._startup_forward_speed = max(0.05, ugv_mps * startup_ratio)

        self._scan_topics = []
        for topic in [self._scan_topic, *fallback_topics]:
            if topic and topic not in self._scan_topics:
                self._scan_topics.append(topic)

        self._cmd_pub = self.create_publisher(Twist, self._cmd_vel_topic, 10)
        self._scan_subs = [
            self.create_subscription(
                LaserScan,
                topic,
                self._make_scan_callback(topic),
                qos_profile_sensor_data,
            )
            for topic in self._scan_topics
        ]

        self._latest_scan: Optional[LaserScan] = None
        self._start_time = self.get_clock().now()
        self._latest_scan_stamp = self.get_clock().now()
        self._missing_scan_logged = False
        self._scan_source_topic: Optional[str] = None

        timer_period = 1.0 / max(self._control_rate_hz, 1.0)
        self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            f"drive_lidar active: scan topics={self._scan_topics}, cmd_vel={self._cmd_vel_topic}, "
            f"linear={self._linear_speed:.2f}m/s, turn={self._turn_speed:.2f}rad/s, "
            f"safety={self._safety_distance:.2f}m, critical={self._critical_distance:.2f}m"
        )

    def _make_scan_callback(self, topic: str):
        def _on_scan(msg: LaserScan):
            self._latest_scan = msg
            self._latest_scan_stamp = self.get_clock().now()
            self._missing_scan_logged = False
            if self._scan_source_topic != topic:
                self._scan_source_topic = topic
                self.get_logger().info(f"Receiving LaserScan from {topic}")

        return _on_scan

    def _sector_min(self, scan: LaserScan, angle_start: float, angle_end: float) -> float:
        if not scan.ranges:
            return math.inf

        if angle_start > angle_end:
            angle_start, angle_end = angle_end, angle_start

        angle_start = max(angle_start, scan.angle_min)
        angle_end = min(angle_end, scan.angle_max)
        if angle_end < angle_start:
            return math.inf

        inc = scan.angle_increment
        if inc <= 0.0:
            return math.inf

        i0 = int((angle_start - scan.angle_min) / inc)
        i1 = int((angle_end - scan.angle_min) / inc)
        i0 = max(0, min(i0, len(scan.ranges) - 1))
        i1 = max(0, min(i1, len(scan.ranges) - 1))
        if i1 < i0:
            i0, i1 = i1, i0

        minimum = math.inf
        for i in range(i0, i1 + 1):
            r = float(scan.ranges[i])
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                minimum = min(minimum, r)
        return minimum

    def _publish_cmd(self, linear_x: float, angular_z: float):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self._cmd_pub.publish(msg)

    def _on_timer(self):
        elapsed = (self.get_clock().now() - self._start_time).nanoseconds / 1e9
        if elapsed < max(0.0, self._startup_forward_sec):
            self._publish_cmd(self._startup_forward_speed, 0.0)
            return

        now = self.get_clock().now()
        if self._latest_scan is None or (now - self._latest_scan_stamp).nanoseconds / 1e9 > self._scan_timeout_sec:
            self._publish_cmd(0.0, 0.0)
            if not self._missing_scan_logged:
                self.get_logger().warn(
                    f"No recent LaserScan on any of {self._scan_topics}; holding position. "
                    "Check ros_gz bridge lidar topic."
                )
                self._missing_scan_logged = True
            return

        scan = self._latest_scan
        front_min = self._sector_min(scan, -self._front_half_angle, self._front_half_angle)
        left_min = self._sector_min(scan, self._side_min_angle, self._side_max_angle)
        right_min = self._sector_min(scan, -self._side_max_angle, -self._side_min_angle)

        linear_x = 0.0
        angular_z = 0.0

        # Reactive policy: move forward when clear, otherwise turn toward the side with more free space.
        if front_min < self._critical_distance:
            angular_z = self._turn_speed if left_min > right_min else -self._turn_speed
        elif front_min < self._safety_distance:
            linear_x = self._creep_speed
            angular_z = 0.65 * self._turn_speed if left_min > right_min else -0.65 * self._turn_speed
        else:
            linear_x = self._linear_speed

            left_crowded = left_min < self._safety_distance * 0.75
            right_crowded = right_min < self._safety_distance * 0.75
            if left_crowded and not right_crowded:
                angular_z = -0.25 * self._turn_speed
            elif right_crowded and not left_crowded:
                angular_z = 0.25 * self._turn_speed

        self._publish_cmd(linear_x, angular_z)

    def stop(self):
        if not rclpy.ok():
            return
        try:
            self._publish_cmd(0.0, 0.0)
        except Exception:
            # ROS context may already be shutting down during Ctrl+C.
            pass


def main():
    rclpy.init()
    node = DriveLidar()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


if __name__ == "__main__":
    main()
