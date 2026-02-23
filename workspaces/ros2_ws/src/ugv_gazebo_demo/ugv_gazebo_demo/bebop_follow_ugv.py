#!/usr/bin/env python3

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _wrap_to_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm > 0.0:
        x /= norm
        y /= norm
        z /= norm
        w /= norm
    fx = 1.0 - 2.0 * (y * y + z * z)
    fy = 2.0 * (x * y + w * z)
    return math.atan2(fy, fx)


class BebopFollowUgv(Node):
    def __init__(self):
        super().__init__("bebop_follow_ugv")

        self.declare_parameter("ugv_odom_topic", "/x2_ugv/odom")
        self.declare_parameter("uav_odom_topic", "/bebop1/odom")
        self.declare_parameter("uav_cmd_vel_topic", "/bebop1/cmd_vel")
        self.declare_parameter("uav_enable_topic", "/bebop1/enable")
        self.declare_parameter("control_rate_hz", 20.0)
        self.declare_parameter("follow_distance_m", 10.0)
        self.declare_parameter("hover_height_above_ugv_m", 10.0)
        self.declare_parameter("takeoff_height_above_ugv_m", 10.0)
        self.declare_parameter("kp_xy", 1.0)
        self.declare_parameter("kp_z", 1.0)
        self.declare_parameter("kp_yaw", 1.2)
        self.declare_parameter("use_ugv_velocity_feedforward", True)
        self.declare_parameter("ugv_velocity_feedforward_gain", 1.0)
        self.declare_parameter("max_xy_speed_mps", 2.0)
        self.declare_parameter("max_z_speed_mps", 2.0)
        self.declare_parameter("max_yaw_rate_rps", 1.5)
        self.declare_parameter("odom_timeout_sec", 1.0)
        self.declare_parameter("takeoff_speed_mps", 1.2)
        self.declare_parameter("takeoff_duration_s", 0.0)
        self.declare_parameter("use_odom_vertical_control", True)
        self.declare_parameter("min_takeoff_climb_for_vertical_feedback_m", 0.5)
        self.declare_parameter("follow_ready_topic", "/bebop1/follow_ready")

        self._ugv_odom_topic = self.get_parameter("ugv_odom_topic").value
        self._uav_odom_topic = self.get_parameter("uav_odom_topic").value
        self._uav_cmd_vel_topic = self.get_parameter("uav_cmd_vel_topic").value
        self._uav_enable_topic = self.get_parameter("uav_enable_topic").value
        self._control_rate_hz = float(self.get_parameter("control_rate_hz").value)
        self._follow_distance = float(self.get_parameter("follow_distance_m").value)
        self._hover_height = float(self.get_parameter("hover_height_above_ugv_m").value)
        self._takeoff_height = float(self.get_parameter("takeoff_height_above_ugv_m").value)
        self._kp_xy = float(self.get_parameter("kp_xy").value)
        self._kp_z = float(self.get_parameter("kp_z").value)
        self._kp_yaw = float(self.get_parameter("kp_yaw").value)
        self._use_ugv_velocity_ff = bool(self.get_parameter("use_ugv_velocity_feedforward").value)
        self._ugv_velocity_ff_gain = float(self.get_parameter("ugv_velocity_feedforward_gain").value)
        self._max_xy_speed = float(self.get_parameter("max_xy_speed_mps").value)
        self._max_z_speed = float(self.get_parameter("max_z_speed_mps").value)
        self._max_yaw_rate = float(self.get_parameter("max_yaw_rate_rps").value)
        self._odom_timeout_sec = float(self.get_parameter("odom_timeout_sec").value)
        self._takeoff_speed = float(self.get_parameter("takeoff_speed_mps").value)
        self._takeoff_duration_override = float(self.get_parameter("takeoff_duration_s").value)
        self._use_odom_vertical_control = bool(self.get_parameter("use_odom_vertical_control").value)
        self._min_takeoff_climb_for_vertical_feedback = float(
            self.get_parameter("min_takeoff_climb_for_vertical_feedback_m").value
        )
        self._follow_ready_topic = str(self.get_parameter("follow_ready_topic").value)
        if self._takeoff_duration_override > 0.0:
            self._takeoff_duration_s = max(0.1, self._takeoff_duration_override)
            self._takeoff_speed = self._takeoff_height / self._takeoff_duration_s
        else:
            self._takeoff_duration_s = self._takeoff_height / max(0.1, self._takeoff_speed)

        self._ugv_odom: Optional[Odometry] = None
        self._uav_odom: Optional[Odometry] = None
        now = self.get_clock().now()
        self._ugv_stamp = now
        self._uav_stamp = now
        self._takeoff_complete = False
        self._missing_odom_logged = False
        self._control_enabled = False
        self._takeoff_start_time = None
        self._takeoff_start_z: Optional[float] = None
        self._vertical_control_active = self._use_odom_vertical_control

        self._cmd_pub = self.create_publisher(Twist, self._uav_cmd_vel_topic, 10)
        self._enable_pub = self.create_publisher(Bool, self._uav_enable_topic, 10)
        ready_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._ready_pub = self.create_publisher(Bool, self._follow_ready_topic, ready_qos)
        self._ready_pub.publish(Bool(data=False))
        odom_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._ugv_sub = self.create_subscription(Odometry, self._ugv_odom_topic, self._on_ugv_odom, odom_qos)
        self._uav_sub = self.create_subscription(Odometry, self._uav_odom_topic, self._on_uav_odom, odom_qos)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        period = 1.0 / max(1.0, self._control_rate_hz)
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            "bebop_follow_ugv active: "
            f"ugv_odom={self._ugv_odom_topic}, uav_odom={self._uav_odom_topic}, "
            f"uav_cmd={self._uav_cmd_vel_topic}, uav_enable={self._uav_enable_topic}, "
            f"behind={self._follow_distance:.1f}m, hover_height={self._hover_height:.1f}m, "
            f"takeoff_height={self._takeoff_height:.1f}m, takeoff_speed={self._takeoff_speed:.2f}m/s, "
            f"takeoff_duration={self._takeoff_duration_s:.1f}s, "
            f"vel_ff={'on' if self._use_ugv_velocity_ff else 'off'}({self._ugv_velocity_ff_gain:.2f}), "
            f"max_xy={self._max_xy_speed:.2f}m/s, "
            f"vertical_control={'on' if self._vertical_control_active else 'off'}"
        )

    def _on_set_parameters(self, params):
        for p in params:
            if p.name == "follow_distance_m":
                try:
                    new_val = float(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="follow_distance_m must be numeric")
                if not math.isfinite(new_val) or new_val < 0.0:
                    return SetParametersResult(successful=False, reason="follow_distance_m must be >= 0")
                self._follow_distance = new_val
                self.get_logger().info(f"Updated follow_distance_m -> {self._follow_distance:.2f} m")
            elif p.name == "hover_height_above_ugv_m":
                try:
                    new_val = float(p.value)
                except Exception:
                    return SetParametersResult(successful=False, reason="hover_height_above_ugv_m must be numeric")
                if not math.isfinite(new_val) or new_val < 0.0:
                    return SetParametersResult(successful=False, reason="hover_height_above_ugv_m must be >= 0")
                self._hover_height = new_val
                self.get_logger().info(f"Updated hover_height_above_ugv_m -> {self._hover_height:.2f} m")
        return SetParametersResult(successful=True)

    def _on_ugv_odom(self, msg: Odometry):
        self._ugv_odom = msg
        self._ugv_stamp = self.get_clock().now()
        self._missing_odom_logged = False

    def _on_uav_odom(self, msg: Odometry):
        self._uav_odom = msg
        self._uav_stamp = self.get_clock().now()
        self._missing_odom_logged = False

    def _publish_cmd(self, vx_body: float, vy_body: float, vz: float, yaw_rate: float):
        msg = Twist()
        msg.linear.x = float(vx_body)
        msg.linear.y = float(vy_body)
        msg.linear.z = float(vz)
        msg.angular.z = float(yaw_rate)
        self._cmd_pub.publish(msg)

    def _on_timer(self):
        now = self.get_clock().now()
        fresh_odom = not (
            self._ugv_odom is None
            or self._uav_odom is None
            or (now - self._ugv_stamp).nanoseconds / 1e9 > self._odom_timeout_sec
            or (now - self._uav_stamp).nanoseconds / 1e9 > self._odom_timeout_sec
        )
        if not fresh_odom:
            # Do not arm until both odometry streams are live.
            self._ready_pub.publish(Bool(data=False))
            if not self._control_enabled:
                self._enable_pub.publish(Bool(data=False))
            else:
                self._enable_pub.publish(Bool(data=True))
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
            if not self._missing_odom_logged:
                self.get_logger().warn(
                    f"Waiting for fresh odom on {self._ugv_odom_topic} and {self._uav_odom_topic}; holding."
                )
                self._missing_odom_logged = True
            return

        if not self._control_enabled:
            self._enable_pub.publish(Bool(data=True))
            self._control_enabled = True
            self.get_logger().info("Fresh odom detected. Enabling follower control.")
        else:
            self._enable_pub.publish(Bool(data=True))

        ugv_pos = self._ugv_odom.pose.pose.position
        ugv_q = self._ugv_odom.pose.pose.orientation
        ugv_yaw = _yaw_from_quaternion(ugv_q.x, ugv_q.y, ugv_q.z, ugv_q.w)
        ugv_twist = self._ugv_odom.twist.twist.linear

        uav_pos = self._uav_odom.pose.pose.position
        uav_q = self._uav_odom.pose.pose.orientation
        uav_yaw = _yaw_from_quaternion(uav_q.x, uav_q.y, uav_q.z, uav_q.w)

        # Target point: follow behind UGV heading and keep altitude offset above UGV.
        target_x = ugv_pos.x - self._follow_distance * math.cos(ugv_yaw)
        target_y = ugv_pos.y - self._follow_distance * math.sin(ugv_yaw)
        target_z = ugv_pos.z + self._hover_height
        target_yaw = ugv_yaw

        # Stage 1: time-based takeoff before engaging horizontal motion.
        if self._takeoff_start_z is None:
            self._takeoff_start_z = uav_pos.z
            self.get_logger().info(f"Takeoff start z={self._takeoff_start_z:.2f}")
        if self._takeoff_start_time is None:
            self._takeoff_start_time = now

        if not self._takeoff_complete:
            elapsed = (now - self._takeoff_start_time).nanoseconds / 1e9
            if elapsed < self._takeoff_duration_s:
                yaw_rate = _clamp(
                    self._kp_yaw * _wrap_to_pi(target_yaw - uav_yaw),
                    -self._max_yaw_rate,
                    self._max_yaw_rate,
                )
                vz = _clamp(self._takeoff_speed, 0.0, self._max_z_speed)
                self._ready_pub.publish(Bool(data=False))
                self._publish_cmd(0.0, 0.0, vz, yaw_rate)
                return
            self._takeoff_complete = True
            climbed = abs(uav_pos.z - self._takeoff_start_z)
            if self._vertical_control_active and climbed < self._min_takeoff_climb_for_vertical_feedback:
                self._vertical_control_active = False
                self.get_logger().warn(
                    f"Disabling vertical hold: odom z climb during takeoff was only {climbed:.2f} m "
                    f"(threshold {self._min_takeoff_climb_for_vertical_feedback:.2f} m)."
                )
            self.get_logger().info("Takeoff complete. Switching to follow mode.")
            self._ready_pub.publish(Bool(data=True))

        ex = target_x - uav_pos.x
        ey = target_y - uav_pos.y
        ez = target_z - uav_pos.z
        eyaw = _wrap_to_pi(target_yaw - uav_yaw)

        # Position-feedback term (world frame).
        vx_world_fb = self._kp_xy * ex
        vy_world_fb = self._kp_xy * ey
        # Feed forward the UGV velocity (odom twist is typically in child/body frame).
        if self._use_ugv_velocity_ff:
            c_ugv = math.cos(ugv_yaw)
            s_ugv = math.sin(ugv_yaw)
            ugv_vx_world = c_ugv * ugv_twist.x - s_ugv * ugv_twist.y
            ugv_vy_world = s_ugv * ugv_twist.x + c_ugv * ugv_twist.y
            vx_world_cmd = vx_world_fb + self._ugv_velocity_ff_gain * ugv_vx_world
            vy_world_cmd = vy_world_fb + self._ugv_velocity_ff_gain * ugv_vy_world
        else:
            vx_world_cmd = vx_world_fb
            vy_world_cmd = vy_world_fb
        vx_world = _clamp(vx_world_cmd, -self._max_xy_speed, self._max_xy_speed)
        vy_world = _clamp(vy_world_cmd, -self._max_xy_speed, self._max_xy_speed)
        if self._vertical_control_active:
            vz = _clamp(self._kp_z * ez, -self._max_z_speed, self._max_z_speed)
        else:
            # Odom z can be pinned at 0 in some nested-model setups; hold z command at zero in follow mode.
            vz = 0.0
        yaw_rate = _clamp(self._kp_yaw * eyaw, -self._max_yaw_rate, self._max_yaw_rate)

        # Convert world-frame XY velocity to UAV body frame.
        c = math.cos(uav_yaw)
        s = math.sin(uav_yaw)
        vx_body = c * vx_world + s * vy_world
        vy_body = -s * vx_world + c * vy_world

        self._ready_pub.publish(Bool(data=True))
        self._publish_cmd(vx_body, vy_body, vz, yaw_rate)

    def stop(self):
        if not rclpy.ok():
            return
        try:
            self._publish_cmd(0.0, 0.0, 0.0, 0.0)
        except Exception:
            pass


def main():
    rclpy.init()
    node = BebopFollowUgv()
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
