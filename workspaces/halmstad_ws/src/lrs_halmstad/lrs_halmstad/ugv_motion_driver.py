#!/usr/bin/env python3

import math
import time
from typing import List, Tuple

import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node


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

        self.ready_check_enable = bool(self.get_parameter("ready_check_enable").value)
        self.ready_require_cmd_subscriber = bool(self.get_parameter("ready_require_cmd_subscriber").value)
        self.ready_require_odom_flow = bool(self.get_parameter("ready_require_odom_flow").value)
        self.ready_odom_topic = str(self.get_parameter("ready_odom_topic").value)
        self.ready_timeout_s = max(0.0, float(self.get_parameter("ready_timeout_s").value))
        self.ready_poll_hz = max(1e-3, float(self.get_parameter("ready_poll_hz").value))
        self.ready_settle_s = max(0.0, float(self.get_parameter("ready_settle_s").value))

        self.variation_amplitude = max(0.0, min(0.45, self.variation_amplitude))

        self._apply_motion_profile()
        self._pub = self.create_publisher(TwistStamped, self.cmd_topic, 10)
        self._ready_odom_seen = False
        self._ready_odom_sub = None
        if self.ready_check_enable and self.ready_require_odom_flow and self.ready_odom_topic:
            self._ready_odom_sub = self.create_subscription(
                Odometry, self.ready_odom_topic, self._on_ready_odom, 10
            )

    def _apply_motion_profile(self) -> None:
        if self.motion_profile in ("fast_wide", "far_fast", "wide_fast"):
            self.forward_speed *= 1.8
            self.forward_time_s *= 1.6
            self.turn_speed *= 1.2
            self.turn_time_s *= 0.9
            self.cycles = max(self.cycles, 12)
        elif self.motion_profile in ("fast", "speed"):
            self.forward_speed *= 1.8
            self.turn_speed *= 1.2
        elif self.motion_profile in ("wide", "far"):
            self.forward_time_s *= 1.8
            self.cycles = max(self.cycles, 12)

    def _build_segments(self) -> List[Tuple[str, float, float, float]]:
        segments: List[Tuple[str, float, float, float]] = []
        for i in range(self.cycles):
            if self.variation_enable:
                # Deterministic modulation for repeatable but less robotic paths.
                fwd_var = 0.65 * math.sin(0.73 * i) + 0.35 * math.sin(0.19 * i + 1.10)
                turn_var = 0.60 * math.cos(0.61 * i + 0.40) + 0.40 * math.sin(0.27 * i + 0.90)
                fs = max(0.40, 1.0 + self.variation_amplitude * fwd_var)
                ts = max(0.40, 1.0 + self.variation_amplitude * turn_var)
            else:
                fs = 1.0
                ts = 1.0

            segments.append(("fwd", self.forward_speed * fs, 0.0, self.forward_time_s))
            turn_sign = -1.0 if (self.turn_pattern in ("alternate", "zigzag", "zig-zag") and (i % 2)) else 1.0
            segments.append(("turn", 0.0, turn_sign * self.turn_speed * ts, self.turn_time_s))
            if self.pause_every_n > 0 and ((i + 1) % self.pause_every_n == 0) and self.pause_time_s > 0.0:
                segments.append(("pause", 0.0, 0.0, self.pause_time_s))

        return segments

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
        est_motion_s = sum(float(secs) for _, _, _, secs in segments)
        self.get_logger().info(
            f"UGV motion: profile={self.motion_profile} turn_pattern={self.turn_pattern} "
            f"topic={self.cmd_topic} cycles={self.cycles} est_duration~{est_motion_s:.1f}s"
        )
        self.get_logger().info(
            f"v_fwd={self.forward_speed:.3f} t_fwd={self.forward_time_s:.3f} "
            f"v_turn={self.turn_speed:.3f} t_turn={self.turn_time_s:.3f} pub_rate={self.pub_rate_hz:.1f}Hz"
        )

        dt = 1.0 / self.pub_rate_hz

        if self.startup_pad_s > 0.0:
            t_end = time.monotonic() + self.startup_pad_s
            while time.monotonic() < t_end:
                self._publish_cmd(0.0, 0.0)
                rclpy.spin_once(self, timeout_sec=0.0)
                time.sleep(dt)

        for _name, vx, wz, secs in segments:
            t_end = time.monotonic() + max(0.0, float(secs))
            while time.monotonic() < t_end:
                self._publish_cmd(vx, wz)
                rclpy.spin_once(self, timeout_sec=0.0)
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
