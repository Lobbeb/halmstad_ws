#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, String

from lrs_halmstad.follow_debug import FollowDebugPublishers
from lrs_halmstad.follow_math import (
    Pose2D,
    camera_xy_from_uav_pose,
    clamp_point_to_radius,
    coerce_bool,
    compute_leader_look_target,
    solve_yaw_to_target,
    wrap_pi,
    yaw_from_quat,
)


def compute_rate_limited_axis_value(
    current_value: float,
    target_value: float,
    *,
    tick_hz: float,
    max_speed_per_s: float,
    gain: float,
    snap_tolerance: float = 0.01,
) -> float:
    if tick_hz <= 0.0:
        return float(target_value)

    error = float(target_value) - float(current_value)
    if abs(error) <= max(0.0, float(snap_tolerance)):
        return float(target_value)

    axis_speed_per_s = min(
        max(float(max_speed_per_s), 0.0),
        max(float(gain), 0.0) * abs(error),
    )
    step_limit = axis_speed_per_s / float(tick_hz) if axis_speed_per_s > 0.0 else 0.0
    if step_limit <= 0.0:
        return float(current_value)
    if abs(error) <= step_limit:
        return float(target_value)
    return float(current_value) + math.copysign(step_limit, error)


class FollowUavOdom(Node):
    def __init__(self):
        super().__init__("follow_uav")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("world", "orchard")
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("leader_odom_topic", "/a201_0000/platform/odom/filtered")

        self.declare_parameter("tick_hz", 10.0, dyn_num)
        self.declare_parameter("d_target", 7.0, dyn_num)
        self.declare_parameter("d_max", 100.0)
        self.declare_parameter("z_alt", 7.0, dyn_num)
        self.declare_parameter("d_euclidean", 0.0, dyn_num)
        self.declare_parameter("manual_override_enable", False)

        self.declare_parameter("seed_uav_cmd_on_start", True)
        self.declare_parameter("uav_start_x", -2.0)
        self.declare_parameter("uav_start_y", 0.0)
        self.declare_parameter("uav_start_yaw_deg", 0.0)
        self.declare_parameter("startup_nudge_enable", False)
        self.declare_parameter("startup_nudge_dx_m", 0.30)
        self.declare_parameter("startup_nudge_dy_m", 0.0)

        self.declare_parameter("follow_yaw", False)
        self.declare_parameter("pose_timeout_s", 3.0)
        self.declare_parameter("min_cmd_period_s", 0.05)
        self.declare_parameter("smooth_alpha", 1.0)
        self.declare_parameter("follow_speed_mps", 1.0)
        self.declare_parameter("follow_speed_gain", 2.0)
        self.declare_parameter("follow_z_speed_mps", 1.0)
        self.declare_parameter("follow_z_speed_gain", 2.0)
        self.declare_parameter("cmd_xy_deadband_m", 0.02)
        self.declare_parameter("follow_yaw_rate_rad_s", 1.0)
        self.declare_parameter("follow_yaw_rate_gain", 2.0)
        self.declare_parameter("yaw_deadband_rad", 0.01)
        self.declare_parameter("yaw_update_xy_gate_m", 0.0)

        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("publish_events", False)
        self.declare_parameter("publish_metrics", False)
        self.declare_parameter("metrics_prefix", "/coord")
        self.declare_parameter("camera_x_offset_m", 0.0)
        self.declare_parameter("camera_y_offset_m", 0.0)
        self.declare_parameter("camera_z_offset_m", 0.27)
        self.declare_parameter("leader_look_target_x_m", 0.0)
        self.declare_parameter("leader_look_target_y_m", 0.0)

        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.leader_odom_topic = str(self.get_parameter("leader_odom_topic").value)

        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.d_target = float(self.get_parameter("d_target").value)
        self.d_max = float(self.get_parameter("d_max").value)
        self.z_alt = float(self.get_parameter("z_alt").value)
        self.d_euclidean = float(self.get_parameter("d_euclidean").value)
        self.manual_override_enable = coerce_bool(self.get_parameter("manual_override_enable").value)

        self.seed_uav_cmd_on_start = coerce_bool(self.get_parameter("seed_uav_cmd_on_start").value)
        self.uav_start_x = float(self.get_parameter("uav_start_x").value)
        self.uav_start_y = float(self.get_parameter("uav_start_y").value)
        self.uav_start_yaw = math.radians(float(self.get_parameter("uav_start_yaw_deg").value))
        self.startup_nudge_enable = coerce_bool(self.get_parameter("startup_nudge_enable").value)
        self.startup_nudge_dx_m = float(self.get_parameter("startup_nudge_dx_m").value)
        self.startup_nudge_dy_m = float(self.get_parameter("startup_nudge_dy_m").value)

        self.follow_yaw = coerce_bool(self.get_parameter("follow_yaw").value)
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.min_cmd_period_s = float(self.get_parameter("min_cmd_period_s").value)
        self.smooth_alpha = float(self.get_parameter("smooth_alpha").value)
        self.follow_speed_mps = float(self.get_parameter("follow_speed_mps").value)
        self.follow_speed_gain = float(self.get_parameter("follow_speed_gain").value)
        self.follow_z_speed_mps = float(self.get_parameter("follow_z_speed_mps").value)
        self.follow_z_speed_gain = float(self.get_parameter("follow_z_speed_gain").value)
        self.cmd_xy_deadband_m = float(self.get_parameter("cmd_xy_deadband_m").value)
        self.follow_yaw_rate_rad_s = float(self.get_parameter("follow_yaw_rate_rad_s").value)
        self.follow_yaw_rate_gain = float(self.get_parameter("follow_yaw_rate_gain").value)
        self.yaw_deadband_rad = float(self.get_parameter("yaw_deadband_rad").value)
        self.yaw_update_xy_gate_m = float(self.get_parameter("yaw_update_xy_gate_m").value)

        self.event_topic = str(self.get_parameter("event_topic").value)
        self.publish_events = coerce_bool(self.get_parameter("publish_events").value)
        self.publish_metrics = coerce_bool(self.get_parameter("publish_metrics").value)
        self.metrics_prefix = str(self.get_parameter("metrics_prefix").value).rstrip("/") or "/coord"
        self.camera_x_offset_m = float(self.get_parameter("camera_x_offset_m").value)
        self.camera_y_offset_m = float(self.get_parameter("camera_y_offset_m").value)
        self.camera_z_offset_m = float(self.get_parameter("camera_z_offset_m").value)
        self.leader_look_target_x_m = float(self.get_parameter("leader_look_target_x_m").value)
        self.leader_look_target_y_m = float(self.get_parameter("leader_look_target_y_m").value)

        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.d_target <= 0.0 or self.d_max <= 0.0:
            raise ValueError("d_target and d_max must be > 0")
        if (
            self.follow_speed_mps < 0.0
            or self.follow_speed_gain < 0.0
            or self.follow_z_speed_mps < 0.0
            or self.follow_z_speed_gain < 0.0
            or self.follow_yaw_rate_rad_s < 0.0
            or self.follow_yaw_rate_gain < 0.0
        ):
            raise ValueError(
                "follow_speed_mps, follow_speed_gain, follow_z_speed_mps, "
                "follow_z_speed_gain, follow_yaw_rate_rad_s, and "
                "follow_yaw_rate_gain must be >= 0"
            )

        if self.d_euclidean <= 0.0:
            self.d_euclidean = math.hypot(self.d_target, self.z_alt)
        self.d_euclidean = max(self.d_euclidean, 0.01)
        self.d_euclidean_xy_ratio = self.d_target / self.d_euclidean
        self.d_euclidean_z_ratio = self.z_alt / self.d_euclidean
        self.base_d_target = self.d_target
        self.base_d_euclidean = self.d_euclidean
        self.base_follow_speed_mps = self.follow_speed_mps

        self.have_ugv = False
        self.ugv_pose = Pose2D(0.0, 0.0, 0.0)
        self.ugv_z = 0.0
        self.ugv_follow_heading = 0.0
        self.last_ugv_stamp: Optional[Time] = None

        self.uav_cmd = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.uav_cmd_z = self.z_alt
        self.have_uav_cmd = bool(self.seed_uav_cmd_on_start)

        self.uav_actual = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.uav_actual_z = self.z_alt
        self.have_uav_actual = False
        self.last_uav_actual_time: Optional[Time] = None
        self.last_debug_actual_yaw_unwrapped: Optional[float] = None

        self.current_leader_distance_xy_m = max(
            0.01,
            math.hypot(self.d_target - self.camera_x_offset_m, self.camera_y_offset_m),
        )
        self.current_leader_distance_3d_m = math.hypot(self.current_leader_distance_xy_m, self.z_alt)
        self.startup_nudge_pending = (
            self.startup_nudge_enable
            and math.hypot(self.startup_nudge_dx_m, self.startup_nudge_dy_m) > 1e-9
        )
        self.last_cmd_time: Optional[Time] = None

        self.leader_sub = self.create_subscription(
            Odometry,
            self.leader_odom_topic,
            self.on_leader_odom,
            10,
        )
        self.uav_pose_sub = self.create_subscription(
            PoseStamped,
            f"/{self.uav_name}/pose",
            self.on_uav_pose,
            10,
        )

        self.pose_pub = self.create_publisher(PoseStamped, f"/{self.uav_name}/pose_cmd", 10)
        self.pose_odom_pub = self.create_publisher(Odometry, f"/{self.uav_name}/pose_cmd/odom", 10)
        self.uav_cmd_pub = self.create_publisher(
            Joy,
            f"/{self.uav_name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw",
            10,
        )
        self.events_pub = (
            self.create_publisher(String, self.event_topic, 10)
            if self.publish_events
            else None
        )
        self.metric_dist_pub = (
            self.create_publisher(Float32, f"{self.metrics_prefix}/follow_dist_cmd", 10)
            if self.publish_metrics
            else None
        )
        self.metric_err_pub = (
            self.create_publisher(Float32, f"{self.metrics_prefix}/follow_tracking_error_cmd", 10)
            if self.publish_metrics
            else None
        )
        self.debug_pubs = FollowDebugPublishers(self, self.uav_name)

        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"[follow_uav_odom] Started: world={self.world}, uav={self.uav_name}, "
            f"leader_odom={self.leader_odom_topic}, tick={self.tick_hz}Hz, "
            f"d_target={self.d_target}, d_max={self.d_max}, z_alt={self.z_alt}, "
            f"d_euclidean={self.d_euclidean}, follow_speed_mps={self.follow_speed_mps}, "
            f"manual_override_enable={self.manual_override_enable}, "
            f"follow_speed_gain={self.follow_speed_gain}, "
            f"follow_z_speed_mps={self.follow_z_speed_mps}, "
            f"follow_z_speed_gain={self.follow_z_speed_gain}, "
            f"follow_yaw_rate_rad_s={self.follow_yaw_rate_rad_s}, "
            f"follow_yaw_rate_gain={self.follow_yaw_rate_gain}, "
            f"smooth_alpha={self.smooth_alpha}, "
            f"camera_offset_m=({self.camera_x_offset_m}, {self.camera_y_offset_m}, {self.camera_z_offset_m}), "
            f"leader_look_target_xy_m=({self.leader_look_target_x_m}, {self.leader_look_target_y_m}), "
            f"startup_nudge={'on' if self.startup_nudge_pending else 'off'}"
        )
        self.emit_event("FOLLOW_ODOM_NODE_START")

    def emit_event(self, text: str) -> None:
        if not self.publish_events:
            return
        msg = String()
        msg.data = text
        self.events_pub.publish(msg)

    def _on_set_parameters(self, params):
        updates = {}
        runtime_updates = {}
        bool_updates = {}
        for param in params:
            if param.name == "manual_override_enable":
                bool_updates[param.name] = coerce_bool(param.value)
                continue
            try:
                value = float(param.value)
            except Exception as exc:
                if param.name in (
                    "d_euclidean",
                    "d_target",
                    "z_alt",
                    "follow_speed_mps",
                    "follow_speed_gain",
                    "follow_z_speed_mps",
                    "follow_z_speed_gain",
                    "follow_yaw_rate_rad_s",
                    "follow_yaw_rate_gain",
                    "cmd_xy_deadband_m",
                    "yaw_deadband_rad",
                    "yaw_update_xy_gate_m",
                    "smooth_alpha",
                ):
                    return SetParametersResult(successful=False, reason=f"invalid {param.name}: {exc}")
                continue

            if param.name in ("d_euclidean", "d_target", "z_alt"):
                if value <= 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be > 0")
                updates[param.name] = value
            elif param.name in (
                "follow_speed_mps",
                "follow_speed_gain",
                "follow_z_speed_mps",
                "follow_z_speed_gain",
                "follow_yaw_rate_rad_s",
                "follow_yaw_rate_gain",
                "cmd_xy_deadband_m",
                "yaw_deadband_rad",
                "yaw_update_xy_gate_m",
            ):
                if value < 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be >= 0")
                runtime_updates[param.name] = value
            elif param.name == "smooth_alpha":
                if not 0.0 <= value <= 1.0:
                    return SetParametersResult(successful=False, reason="smooth_alpha must be within [0, 1]")
                runtime_updates[param.name] = value

        if not updates and not runtime_updates and not bool_updates:
            return SetParametersResult(successful=True)

        if updates:
            if "d_target" in updates or "z_alt" in updates:
                new_d_target = updates.get("d_target", self.d_target)
                new_z_alt = updates.get("z_alt", self.z_alt)
                new_d_euclidean = math.hypot(new_d_target, new_z_alt)
            else:
                new_d_euclidean = updates["d_euclidean"]
                new_d_target = new_d_euclidean * self.d_euclidean_xy_ratio
                new_z_alt = new_d_euclidean * self.d_euclidean_z_ratio

            if new_d_target >= self.d_max:
                return SetParametersResult(successful=False, reason="d_target must remain < d_max")

            self.d_target = max(new_d_target, 0.01)
            self.z_alt = max(new_z_alt, 0.01)
            self.d_euclidean = max(new_d_euclidean, 0.01)
            self.d_euclidean_xy_ratio = self.d_target / self.d_euclidean
            self.d_euclidean_z_ratio = self.z_alt / self.d_euclidean
            self._update_follow_response_from_distance()
            self._apply_runtime_distance_update()

        if runtime_updates:
            for name, value in runtime_updates.items():
                setattr(self, name, float(value))
        if bool_updates:
            self.manual_override_enable = bool_updates["manual_override_enable"]
        update_parts = []
        if runtime_updates:
            update_parts.append(", ".join(f"{name}={value}" for name, value in sorted(runtime_updates.items())))
        if bool_updates:
            update_parts.append(f"manual_override_enable={self.manual_override_enable}")
        if update_parts:
            self.get_logger().info(f"[follow_uav_odom] Runtime parameter update: {', '.join(update_parts)}")
        return SetParametersResult(successful=True)

    def _current_uav_pose(self) -> Pose2D:
        if self.have_uav_actual:
            return self.uav_actual
        if self.have_uav_cmd:
            return self.uav_cmd
        return Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)

    def _current_uav_z(self) -> float:
        if self.have_uav_actual:
            return self.uav_actual_z
        return self.uav_cmd_z

    def _use_cmd_state_for_control(self) -> bool:
        if not self.have_uav_cmd:
            return False
        if not self.have_uav_actual or self.last_uav_actual_time is None:
            return True
        if self.last_cmd_time is None:
            return False
        return self.last_cmd_time.nanoseconds > self.last_uav_actual_time.nanoseconds

    def _control_uav_pose(self) -> Pose2D:
        if self._use_cmd_state_for_control():
            return self.uav_cmd
        return self._current_uav_pose()

    def _control_uav_z(self) -> float:
        if self._use_cmd_state_for_control():
            return self.uav_cmd_z
        return self._current_uav_z()

    def on_leader_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        vx_body = float(msg.twist.twist.linear.x)
        vy_body = float(msg.twist.twist.linear.y)
        vx_world = math.cos(yaw) * vx_body - math.sin(yaw) * vy_body
        vy_world = math.sin(yaw) * vx_body + math.cos(yaw) * vy_body
        speed_world = math.hypot(vx_world, vy_world)

        self.ugv_pose = Pose2D(float(p.x), float(p.y), yaw)
        self.ugv_z = float(p.z)
        self.ugv_follow_heading = math.atan2(vy_world, vx_world) if speed_world > 0.05 else yaw
        self.have_ugv = True
        try:
            self.last_ugv_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_ugv_stamp = self.get_clock().now()

    def on_uav_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_actual = Pose2D(float(p.x), float(p.y), yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)))
        self.uav_actual_z = float(p.z)
        self.have_uav_actual = True
        try:
            self.last_uav_actual_time = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_actual_time = self.get_clock().now()

    def ugv_pose_is_fresh(self, now: Time) -> bool:
        if not self.have_ugv or self.last_ugv_stamp is None:
            return False
        age_s = (now - self.last_ugv_stamp).nanoseconds * 1e-9
        return age_s <= self.pose_timeout_s

    def can_send_command_now(self, now: Time) -> bool:
        if self.last_cmd_time is None:
            return True
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        return dt >= self.min_cmd_period_s

    def publish_legacy_uav_command(self, x: float, y: float, z: float, yaw_rad: float) -> None:
        msg = Joy()
        msg.axes = [float(x), float(y), float(z), float(yaw_rad)]
        self.uav_cmd_pub.publish(msg)

    def publish_pose_cmd(self, x: float, y: float, z: float, yaw_rad: float) -> None:
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)
        qx, qy, qz, qw = (0.0, 0.0, math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad))
        ps.pose.orientation.x = qx
        ps.pose.orientation.y = qy
        ps.pose.orientation.z = qz
        ps.pose.orientation.w = qw
        self.pose_pub.publish(ps)

    def publish_pose_cmd_odom(self, x: float, y: float, z: float, yaw_rad: float) -> None:
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = f"{self.uav_name}/base_link"
        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(z)
        qx, qy, qz, qw = (0.0, 0.0, math.sin(0.5 * yaw_rad), math.cos(0.5 * yaw_rad))
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        self.pose_odom_pub.publish(odom)
        self.uav_cmd_z = float(z)

    def _camera_xy_from_uav_pose(self, x: float, y: float, yaw: float):
        return camera_xy_from_uav_pose(
            x,
            y,
            yaw,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )

    def _current_follow_geometry(self):
        current_uav = self._current_uav_pose()
        camera_x, camera_y = self._camera_xy_from_uav_pose(current_uav.x, current_uav.y, current_uav.yaw)
        horizontal_distance = math.hypot(self.ugv_pose.x - camera_x, self.ugv_pose.y - camera_y)
        horizontal_distance = max(horizontal_distance, 1e-3)
        distance_3d = math.hypot(horizontal_distance, self._current_uav_z() - self.ugv_z)
        self.current_leader_distance_xy_m = horizontal_distance
        self.current_leader_distance_3d_m = distance_3d
        return horizontal_distance, distance_3d

    def _leader_look_target_xy(self) -> tuple[float, float]:
        target_x, target_y, _target_z = compute_leader_look_target(
            self.ugv_pose.x,
            self.ugv_pose.y,
            self.ugv_pose.yaw,
            self.ugv_z,
            self.leader_look_target_x_m,
            self.leader_look_target_y_m,
            0.0,
        )
        return target_x, target_y

    def _compute_anchor_target(self) -> Pose2D:
        xt = self.ugv_pose.x - self.d_target * math.cos(self.ugv_follow_heading)
        yt = self.ugv_pose.y - self.d_target * math.sin(self.ugv_follow_heading)
        xt, yt = clamp_point_to_radius(self.ugv_pose.x, self.ugv_pose.y, xt, yt, self.d_max)
        target_x, target_y = self._leader_look_target_xy()
        yaw_cmd = solve_yaw_to_target(
            xt,
            yt,
            target_x,
            target_y,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        ) if self.follow_yaw else self._current_uav_pose().yaw
        return Pose2D(xt, yt, yaw_cmd)

    def _anchor_errors(self, anchor_target: Pose2D, current_uav: Pose2D):
        dx = current_uav.x - anchor_target.x
        dy = current_uav.y - anchor_target.y
        anchor_distance_error = math.hypot(dx, dy)

        c = math.cos(self.ugv_follow_heading)
        s = math.sin(self.ugv_follow_heading)
        actual_rel_x = c * (current_uav.x - self.ugv_pose.x) + s * (current_uav.y - self.ugv_pose.y)
        actual_rel_y = -s * (current_uav.x - self.ugv_pose.x) + c * (current_uav.y - self.ugv_pose.y)
        target_rel_x = c * (anchor_target.x - self.ugv_pose.x) + s * (anchor_target.y - self.ugv_pose.y)
        target_rel_y = -s * (anchor_target.x - self.ugv_pose.x) + c * (anchor_target.y - self.ugv_pose.y)
        return (
            anchor_distance_error,
            actual_rel_x - target_rel_x,
            actual_rel_y - target_rel_y,
        )

    def _update_follow_response_from_distance(self) -> None:
        # Yaw response is intentionally independent of follow-distance retargeting.
        return

    def _effective_follow_speed_mps(self, anchor_distance_error: float) -> float:
        return min(
            self.follow_speed_mps,
            self.follow_speed_gain * max(anchor_distance_error, 0.0),
        )

    def _compute_command_z(self, current_z: float) -> float:
        return compute_rate_limited_axis_value(
            current_z,
            self.z_alt,
            tick_hz=self.tick_hz,
            max_speed_per_s=self.follow_z_speed_mps,
            gain=self.follow_z_speed_gain,
        )

    @staticmethod
    def _unwrap_angle_near(raw_angle: float, reference_angle: Optional[float]) -> float:
        if reference_angle is None:
            return float(raw_angle)
        return float(reference_angle + wrap_pi(raw_angle - reference_angle))

    def _publish_follow_debug(
        self,
        anchor_target: Pose2D,
        *,
        current_uav: Pose2D,
        target_yaw: float,
        yaw_error: float,
        yaw_error_raw: float,
        yaw_target_unwrapped: float,
        yaw_actual_unwrapped: float,
        yaw_wrap_correction: float,
        yaw_step_limit: float,
        yaw_cmd_delta: float,
        yaw_mode: str,
    ) -> None:
        now_msg = self.get_clock().now().to_msg()
        anchor_distance_error, anchor_along_error, anchor_cross_error = self._anchor_errors(anchor_target, current_uav)
        self.debug_pubs.publish(
            stamp=now_msg,
            frame_id="map",
            anchor_target=anchor_target,
            d_target=self.d_target,
            z_alt=self.z_alt,
            d_euclidean=self.d_euclidean,
            actual_xy_distance=self.current_leader_distance_xy_m,
            actual_distance_3d=self.current_leader_distance_3d_m,
            actual_yaw=current_uav.yaw,
            target_yaw=target_yaw,
            xy_distance_error=self.current_leader_distance_xy_m - self.d_target,
            anchor_distance_error=anchor_distance_error,
            anchor_along_error=anchor_along_error,
            anchor_cross_error=anchor_cross_error,
            yaw_error=yaw_error,
            yaw_target_raw=target_yaw,
            yaw_target_unwrapped=yaw_target_unwrapped,
            yaw_actual_unwrapped=yaw_actual_unwrapped,
            yaw_error_raw=yaw_error_raw,
            yaw_wrap_correction=yaw_wrap_correction,
            yaw_wrap_active=1.0 if abs(yaw_wrap_correction) > 1e-3 else 0.0,
            yaw_step_limit=yaw_step_limit,
            yaw_cmd_delta=yaw_cmd_delta,
            yaw_mode=yaw_mode,
        )

    def _apply_runtime_distance_update(self) -> None:
        if not self.have_ugv:
            return
        now = self.get_clock().now()
        current_uav = self._control_uav_pose()
        current_uav_z = self._control_uav_z()
        target_x, target_y = self._leader_look_target_xy()
        yaw_cmd = solve_yaw_to_target(
            current_uav.x,
            current_uav.y,
            target_x,
            target_y,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )
        z_cmd = self._compute_command_z(current_uav_z)
        self.publish_legacy_uav_command(current_uav.x, current_uav.y, z_cmd, yaw_cmd)
        self.publish_pose_cmd(current_uav.x, current_uav.y, z_cmd, yaw_cmd)
        self.publish_pose_cmd_odom(current_uav.x, current_uav.y, z_cmd, yaw_cmd)
        self.uav_cmd = Pose2D(current_uav.x, current_uav.y, yaw_cmd)
        self.have_uav_cmd = True
        self.last_cmd_time = now

    def _maybe_publish_startup_nudge(self, now: Time) -> bool:
        if not self.startup_nudge_pending:
            return False
        current_uav = self._control_uav_pose()
        xn = current_uav.x + self.startup_nudge_dx_m
        yn = current_uav.y + self.startup_nudge_dy_m
        self.publish_legacy_uav_command(xn, yn, self.z_alt, current_uav.yaw)
        self.publish_pose_cmd(xn, yn, self.z_alt, current_uav.yaw)
        self.publish_pose_cmd_odom(xn, yn, self.z_alt, current_uav.yaw)
        self.uav_cmd = Pose2D(xn, yn, current_uav.yaw)
        self.have_uav_cmd = True
        self.last_cmd_time = now
        self.startup_nudge_pending = False
        self.emit_event("FOLLOW_STARTUP_NUDGE")
        return True

    def publish_metrics_cmd(self, leader: Pose2D, cmd_x: float, cmd_y: float) -> None:
        if not self.publish_metrics:
            return
        d_cmd = math.hypot(cmd_x - leader.x, cmd_y - leader.y)
        err = abs(d_cmd - self.d_target)
        msg1 = Float32()
        msg1.data = float(d_cmd)
        self.metric_dist_pub.publish(msg1)
        msg2 = Float32()
        msg2.data = float(err)
        self.metric_err_pub.publish(msg2)

    def on_tick(self) -> None:
        now = self.get_clock().now()
        if self.manual_override_enable:
            return
        current_uav = self._control_uav_pose()
        current_uav_z = self._control_uav_z()

        if self.can_send_command_now(now) and self._maybe_publish_startup_nudge(now):
            return
        if not self.ugv_pose_is_fresh(now):
            return
        if not self.can_send_command_now(now):
            return

        anchor_target = self._compute_anchor_target()
        xt, yt = anchor_target.x, anchor_target.y

        if (self.have_uav_actual or self.have_uav_cmd) and self.smooth_alpha < 1.0:
            a = self.smooth_alpha
            xt = a * xt + (1.0 - a) * current_uav.x
            yt = a * yt + (1.0 - a) * current_uav.y

        anchor_distance_error, _, _ = self._anchor_errors(anchor_target, current_uav)
        effective_follow_speed_mps = self._effective_follow_speed_mps(anchor_distance_error)
        step_limit = effective_follow_speed_mps / self.tick_hz if effective_follow_speed_mps > 0.0 else 0.0
        if step_limit > 0.0:
            dx = xt - current_uav.x
            dy = yt - current_uav.y
            step = math.hypot(dx, dy)
            if step > step_limit and step > 1e-9:
                s = step_limit / step
                xt = current_uav.x + dx * s
                yt = current_uav.y + dy * s

        cmd_xy_delta = math.hypot(xt - current_uav.x, yt - current_uav.y)
        target_x, target_y = self._leader_look_target_xy()
        yaw_target = solve_yaw_to_target(
            xt,
            yt,
            target_x,
            target_y,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )
        yaw_error_raw = yaw_target - current_uav.yaw
        yaw_error = wrap_pi(yaw_error_raw)
        yaw_actual_unwrapped = self._unwrap_angle_near(
            current_uav.yaw,
            self.last_debug_actual_yaw_unwrapped,
        )
        self.last_debug_actual_yaw_unwrapped = yaw_actual_unwrapped
        yaw_target_unwrapped = yaw_actual_unwrapped + yaw_error
        yaw_wrap_correction = yaw_error_raw - yaw_error
        # Yaw must be solved from the same XY pose we command in this tick.
        # Otherwise a large XY step on sharp turns can leave the camera at the
        # new position with a yaw that was computed for the old position.
        yaw_rate_rad_s = min(
            max(self.follow_yaw_rate_rad_s, 0.0),
            max(self.follow_yaw_rate_gain, 0.0) * abs(yaw_error),
        )
        yaw_step_limit = yaw_rate_rad_s / self.tick_hz if yaw_rate_rad_s > 0.0 else 0.0

        if self.yaw_update_xy_gate_m > 0.0 and cmd_xy_delta < self.yaw_update_xy_gate_m:
            yaw_cmd = current_uav.yaw
            yaw_mode = "xy_gate_hold"
        elif self.yaw_deadband_rad > 0.0 and abs(yaw_error) < self.yaw_deadband_rad:
            yaw_cmd = current_uav.yaw
            yaw_mode = "deadband_hold"
        elif yaw_step_limit > 0.0 and abs(yaw_error) > yaw_step_limit:
            yaw_cmd = wrap_pi(current_uav.yaw + math.copysign(yaw_step_limit, yaw_error))
            yaw_mode = "rate_limited"
        else:
            yaw_cmd = yaw_target
            yaw_mode = "direct_target"

        yaw_cmd_delta = wrap_pi(yaw_cmd - current_uav.yaw)

        if self.cmd_xy_deadband_m > 0.0 and cmd_xy_delta < self.cmd_xy_deadband_m:
            xt = current_uav.x
            yt = current_uav.y

        z_cmd = self._compute_command_z(current_uav_z)
        self.publish_legacy_uav_command(xt, yt, z_cmd, yaw_cmd)
        self.publish_pose_cmd(xt, yt, z_cmd, yaw_cmd)
        self.publish_pose_cmd_odom(xt, yt, z_cmd, yaw_cmd)
        self.publish_metrics_cmd(self.ugv_pose, xt, yt)

        self.uav_cmd = Pose2D(xt, yt, yaw_cmd)
        self.have_uav_cmd = True
        self.last_cmd_time = now

        self._current_follow_geometry()
        self._publish_follow_debug(
            anchor_target,
            current_uav=current_uav,
            target_yaw=yaw_target,
            yaw_error=yaw_error,
            yaw_error_raw=yaw_error_raw,
            yaw_target_unwrapped=yaw_target_unwrapped,
            yaw_actual_unwrapped=yaw_actual_unwrapped,
            yaw_wrap_correction=yaw_wrap_correction,
            yaw_step_limit=yaw_step_limit,
            yaw_cmd_delta=yaw_cmd_delta,
            yaw_mode=yaw_mode,
        )


def main(args=None):
    rclpy.init(args=args)
    node = FollowUavOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.emit_event("FOLLOW_ODOM_NODE_SHUTDOWN")
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
