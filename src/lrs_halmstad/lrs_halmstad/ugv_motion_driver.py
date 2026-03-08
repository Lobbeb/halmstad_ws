#!/usr/bin/env python3

import math
import time

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

from .ugv_motion_profile import MotionProfileConfig, build_segments


class UgvMotionDriver(Node):
    def __init__(self) -> None:
        super().__init__("ugv_motion_driver")

        self.declare_parameter("cmd_topic", "/a201_0000/cmd_vel")
        self.declare_parameter("start_delay_s", 0.0)
        self.declare_parameter("motion_profile", "default")
        self.declare_parameter("turn_pattern", "alternate")

        self.declare_parameter("forward_speed", 1.5)
        self.declare_parameter("forward_time_s", 0.75)
        self.declare_parameter("turn_speed", 1.9)
        self.declare_parameter("turn_time_s", 0.85)
        self.declare_parameter("cycles", 120)

        self.declare_parameter("pub_rate_hz", 12.0)
        self.declare_parameter("startup_pad_s", 0.7)

        self.declare_parameter("variation_enable", True)
        self.declare_parameter("variation_amplitude", 0.20)
        self.declare_parameter("pause_every_n", 0)
        self.declare_parameter("pause_time_s", 0.0)
        self.declare_parameter("obstacle_avoid_enable", False)
        self.declare_parameter("scan_topic", "/a201_0000/sensors/lidar2d_0/scan")
        self.declare_parameter("scan_timeout_s", 0.75)
        self.declare_parameter("stop_on_scan_timeout", False)
        self.declare_parameter("front_sector_deg", 70.0)
        self.declare_parameter("side_sector_min_deg", 20.0)
        self.declare_parameter("side_sector_max_deg", 100.0)
        self.declare_parameter("stop_distance_m", 1.25)
        self.declare_parameter("clear_distance_m", 1.80)
        self.declare_parameter("avoid_turn_speed", 0.80)
        self.declare_parameter("min_valid_scan_points", 3)

        # Lightweight readiness gate to avoid sending motion commands before the
        # UGV controller subscriber is ready/active. This is especially useful
        # in the follow launch where the stack comes up quickly.
        self.declare_parameter("ready_check_enable", False)
        self.declare_parameter("ready_require_cmd_subscriber", True)
        self.declare_parameter("ready_require_odom_flow", False)
        self.declare_parameter("ready_odom_topic", "")
        self.declare_parameter("ready_timeout_s", 8.0)
        self.declare_parameter("ready_poll_hz", 10.0)
        self.declare_parameter("ready_settle_s", 0.25)

        self.cmd_topic = str(self.get_parameter("cmd_topic").value)
        self.start_delay_s = max(0.0, float(self.get_parameter("start_delay_s").value))
        self.motion_profile = str(self.get_parameter("motion_profile").value).strip().lower()
        self.turn_pattern = str(self.get_parameter("turn_pattern").value).strip().lower()

        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.forward_time_s = float(self.get_parameter("forward_time_s").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.turn_time_s = float(self.get_parameter("turn_time_s").value)
        self.cycles = max(0, int(self.get_parameter("cycles").value))

        self.pub_rate_hz = max(1e-3, float(self.get_parameter("pub_rate_hz").value))
        self.startup_pad_s = max(0.0, float(self.get_parameter("startup_pad_s").value))

        self.variation_enable = bool(self.get_parameter("variation_enable").value)
        self.variation_amplitude = float(self.get_parameter("variation_amplitude").value)
        self.pause_every_n = max(0, int(self.get_parameter("pause_every_n").value))
        self.pause_time_s = max(0.0, float(self.get_parameter("pause_time_s").value))
        self.obstacle_avoid_enable = bool(self.get_parameter("obstacle_avoid_enable").value)
        self.scan_topic = str(self.get_parameter("scan_topic").value)
        self.scan_timeout_s = max(0.0, float(self.get_parameter("scan_timeout_s").value))
        self.stop_on_scan_timeout = bool(self.get_parameter("stop_on_scan_timeout").value)
        self.front_sector_deg = max(1.0, float(self.get_parameter("front_sector_deg").value))
        self.side_sector_min_deg = max(0.0, float(self.get_parameter("side_sector_min_deg").value))
        self.side_sector_max_deg = max(
            self.side_sector_min_deg,
            float(self.get_parameter("side_sector_max_deg").value),
        )
        self.stop_distance_m = max(0.05, float(self.get_parameter("stop_distance_m").value))
        self.clear_distance_m = max(self.stop_distance_m, float(self.get_parameter("clear_distance_m").value))
        self.avoid_turn_speed = abs(float(self.get_parameter("avoid_turn_speed").value))
        self.min_valid_scan_points = max(1, int(self.get_parameter("min_valid_scan_points").value))

        self.ready_check_enable = bool(self.get_parameter("ready_check_enable").value)
        self.ready_require_cmd_subscriber = bool(self.get_parameter("ready_require_cmd_subscriber").value)
        self.ready_require_odom_flow = bool(self.get_parameter("ready_require_odom_flow").value)
        self.ready_odom_topic = str(self.get_parameter("ready_odom_topic").value)
        self.ready_timeout_s = max(0.0, float(self.get_parameter("ready_timeout_s").value))
        self.ready_poll_hz = max(1e-3, float(self.get_parameter("ready_poll_hz").value))
        self.ready_settle_s = max(0.0, float(self.get_parameter("ready_settle_s").value))

        self.front_sector_half_rad = math.radians(0.5 * self.front_sector_deg)
        self.side_sector_min_rad = math.radians(self.side_sector_min_deg)
        self.side_sector_max_rad = math.radians(self.side_sector_max_deg)
        self._pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self._ready_odom_seen = False
        self._ready_odom_sub = None
        self._latest_scan_stamp_s = None
        self._front_min_range_m = float("inf")
        self._left_clearance_m = float("inf")
        self._right_clearance_m = float("inf")
        self._avoid_active = False
        self._avoid_turn_sign = 1.0
        self._last_scan_timeout_log_s = 0.0
        if self.ready_check_enable and self.ready_require_odom_flow and self.ready_odom_topic:
            self._ready_odom_sub = self.create_subscription(
                Odometry, self.ready_odom_topic, self._on_ready_odom, 10
            )
        self._scan_sub = None
        if self.obstacle_avoid_enable and self.scan_topic:
            self._scan_sub = self.create_subscription(
                LaserScan, self.scan_topic, self._on_scan, qos_profile_sensor_data
            )

    def _build_segments(self):
        return build_segments(
            MotionProfileConfig(
                motion_profile=self.motion_profile,
                turn_pattern=self.turn_pattern,
                forward_speed=self.forward_speed,
                forward_time_s=self.forward_time_s,
                turn_speed=self.turn_speed,
                turn_time_s=self.turn_time_s,
                cycles=self.cycles,
                variation_enable=self.variation_enable,
                variation_amplitude=self.variation_amplitude,
                pause_every_n=self.pause_every_n,
                pause_time_s=self.pause_time_s,
            )
        )

    def _publish_cmd(self, vx: float, wz: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        msg.twist.angular.x = 0.0
        msg.twist.angular.y = 0.0
        msg.twist.angular.z = float(wz)
        self._pub.publish(msg)

    def _on_ready_odom(self, _msg: Odometry) -> None:
        self._ready_odom_seen = True

    @staticmethod
    def _mean(values: List[float]) -> float:
        if not values:
            return float("inf")
        return sum(values) / float(len(values))

    def _on_scan(self, msg: LaserScan) -> None:
        front_ranges: List[float] = []
        left_ranges: List[float] = []
        right_ranges: List[float] = []
        min_valid_range = max(0.05, float(msg.range_min))
        max_valid_range = float(msg.range_max) if math.isfinite(msg.range_max) and msg.range_max > 0.0 else float("inf")
        angle = float(msg.angle_min)
        increment = float(msg.angle_increment)

        for distance in msg.ranges:
            rng = float(distance)
            if not math.isfinite(rng) or rng < min_valid_range or rng > max_valid_range:
                angle += increment
                continue

            wrapped_angle = math.atan2(math.sin(angle), math.cos(angle))
            if abs(wrapped_angle) <= self.front_sector_half_rad:
                front_ranges.append(rng)
            if self.side_sector_min_rad <= wrapped_angle <= self.side_sector_max_rad:
                left_ranges.append(rng)
            elif -self.side_sector_max_rad <= wrapped_angle <= -self.side_sector_min_rad:
                right_ranges.append(rng)
            angle += increment

        self._latest_scan_stamp_s = time.monotonic()
        self._front_min_range_m = min(front_ranges) if len(front_ranges) >= self.min_valid_scan_points else float("inf")
        self._left_clearance_m = self._mean(left_ranges)
        self._right_clearance_m = self._mean(right_ranges)

    def _scan_is_fresh(self) -> bool:
        if self._latest_scan_stamp_s is None:
            return False
        if self.scan_timeout_s <= 0.0:
            return True
        return (time.monotonic() - self._latest_scan_stamp_s) <= self.scan_timeout_s

    def _choose_avoid_turn_sign(self) -> float:
        left = self._left_clearance_m
        right = self._right_clearance_m
        if math.isfinite(left) and math.isfinite(right):
            if left > (right + 0.10):
                return 1.0
            if right > (left + 0.10):
                return -1.0
        elif math.isfinite(left):
            return 1.0
        elif math.isfinite(right):
            return -1.0
        return self._avoid_turn_sign

    def _apply_obstacle_avoidance(self, vx: float, wz: float) -> Tuple[float, float]:
        if not self.obstacle_avoid_enable:
            return vx, wz

        scan_fresh = self._scan_is_fresh()
        if not scan_fresh:
            if self.stop_on_scan_timeout:
                now_s = time.monotonic()
                if (now_s - self._last_scan_timeout_log_s) >= 1.0:
                    self.get_logger().warn(
                        f"LiDAR scan timeout on {self.scan_topic}; holding UGV motion"
                    )
                    self._last_scan_timeout_log_s = now_s
                self._avoid_active = False
                return 0.0, 0.0
            return vx, wz

        front_blocked = self._front_min_range_m <= self.stop_distance_m
        front_cleared = self._front_min_range_m >= self.clear_distance_m
        was_active = self._avoid_active

        if front_blocked:
            self._avoid_active = True
        elif self._avoid_active and not front_cleared:
            self._avoid_active = True
        else:
            self._avoid_active = False

        if not self._avoid_active:
            if was_active:
                self.get_logger().info(
                    f"Obstacle clear ahead ({self._front_min_range_m:.2f} m); resuming scripted motion"
                )
            return vx, wz

        self._avoid_turn_sign = self._choose_avoid_turn_sign()
        avoid_wz = self._avoid_turn_sign * self.avoid_turn_speed
        if not was_active:
            turn_dir = "left" if self._avoid_turn_sign > 0.0 else "right"
            self.get_logger().warn(
                "Front obstacle detected at "
                f"{self._front_min_range_m:.2f} m; turning {turn_dir} "
                f"(left_clearance={self._left_clearance_m:.2f} m, "
                f"right_clearance={self._right_clearance_m:.2f} m)"
            )
        return 0.0, avoid_wz

    def _cmd_has_subscriber(self) -> bool:
        return len(self.get_subscriptions_info_by_topic(self.cmd_topic)) > 0

    def _wait_for_ready(self) -> None:
        if not self.ready_check_enable:
            return

        # If neither condition is enabled, the gate has nothing to do.
        if not self.ready_require_cmd_subscriber and not self.ready_require_odom_flow:
            self.get_logger().info("UGV readiness gate enabled but no checks selected; continuing immediately")
            return

        deadline = time.monotonic() + self.ready_timeout_s if self.ready_timeout_s > 0.0 else None
        poll_dt = 1.0 / self.ready_poll_hz
        last_log_t = 0.0
        stable_t0 = None
        self.get_logger().info(
            "Waiting for UGV readiness: "
            f"cmd_subscriber={self.ready_require_cmd_subscriber}, "
            f"odom_flow={self.ready_require_odom_flow}"
            + (f" on {self.ready_odom_topic}" if self.ready_require_odom_flow and self.ready_odom_topic else "")
        )

        while rclpy.ok():
            now = time.monotonic()
            if deadline is not None and now > deadline:
                break

            rclpy.spin_once(self, timeout_sec=0.0)

            cmd_ok = (not self.ready_require_cmd_subscriber) or self._cmd_has_subscriber()
            odom_ok = (not self.ready_require_odom_flow) or self._ready_odom_seen

            if cmd_ok and odom_ok:
                if stable_t0 is None:
                    stable_t0 = now
                if (now - stable_t0) >= self.ready_settle_s:
                    self.get_logger().info("UGV readiness gate passed; starting motion")
                    return
            else:
                stable_t0 = None

            if (now - last_log_t) >= 1.0:
                status = []
                if self.ready_require_cmd_subscriber:
                    status.append(f"cmd_subscriber={'yes' if cmd_ok else 'no'}")
                if self.ready_require_odom_flow:
                    status.append(f"odom_flow={'yes' if odom_ok else 'no'}")
                self.get_logger().info("UGV readiness pending: " + ", ".join(status))
                last_log_t = now

            time.sleep(poll_dt)

        # Fail-open to preserve motion behavior even if graph introspection or
        # timing differs across machines. The driver will still apply startup_pad.
        self.get_logger().warn(
            f"UGV readiness gate timed out after {self.ready_timeout_s:.1f}s; continuing anyway"
        )

    def run(self) -> int:
        if self.start_delay_s > 0.0:
            self.get_logger().info(f"Start delay {self.start_delay_s:.1f}s before UGV motion")
            time.sleep(self.start_delay_s)

        self._wait_for_ready()

        segments = self._build_segments()
        est_motion_s = sum(float(segment.duration_s) for segment in segments)
        self.get_logger().info(
            f"UGV motion: profile={self.motion_profile} turn_pattern={self.turn_pattern} "
            f"topic={self.cmd_topic} cycles={self.cycles} est_duration~{est_motion_s:.1f}s"
        )
        self.get_logger().info(
            f"v_fwd={self.forward_speed:.3f} t_fwd={self.forward_time_s:.3f} "
            f"v_turn={self.turn_speed:.3f} t_turn={self.turn_time_s:.3f} pub_rate={self.pub_rate_hz:.1f}Hz"
        )
        if self.obstacle_avoid_enable:
            self.get_logger().info(
                "UGV obstacle avoidance enabled: "
                f"scan_topic={self.scan_topic} stop={self.stop_distance_m:.2f}m "
                f"clear={self.clear_distance_m:.2f}m sector={self.front_sector_deg:.1f}deg "
                f"avoid_turn={self.avoid_turn_speed:.2f}rad/s"
            )

        dt = 1.0 / self.pub_rate_hz

        if self.startup_pad_s > 0.0:
            t_end = time.monotonic() + self.startup_pad_s
            while time.monotonic() < t_end:
                self._publish_cmd(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(dt)

        for segment in segments:
            t_end = time.monotonic() + max(0.0, float(segment.duration_s))
            while time.monotonic() < t_end:
                rclpy.spin_once(self, timeout_sec=0.0)
                cmd_vx, cmd_wz = self._apply_obstacle_avoidance(segment.vx, segment.wz)
                self._publish_cmd(cmd_vx, cmd_wz)
                time.sleep(dt)

        for _ in range(max(3, int(math.ceil(0.3 / dt)))):
            self._publish_cmd(0.0, 0.0)
            rclpy.spin_once(self, timeout_sec=0.0)
            time.sleep(dt)

        self.get_logger().info("UGV motion complete")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvMotionDriver()
    rc = 0
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
