#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from lrs_halmstad.common.ros_params import declare_yaml_param, required_param_value
from lrs_halmstad.follow.follow_core import FollowControllerCoreMixin
from lrs_halmstad.follow.follow_math import (
    Pose2D,
    clamp_point_to_radius,
    coerce_bool,
    horizontal_distance_for_euclidean,
    solve_yaw_to_target,
    wrap_pi,
    yaw_from_quat,
)


def move_towards(value: float, target: float, max_step: float) -> float:
    delta = float(target) - float(value)
    if abs(delta) <= max_step:
        return float(target)
    return float(value) + math.copysign(float(max_step), delta)


class FollowUav(FollowControllerCoreMixin, Node):
    """Pose/estimate-mode UAV follow controller with odom-style geometry."""

    def __init__(self):
        super().__init__("follow_uav")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("world", "warehouse")
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("leader_input_type", "pose")
        self.declare_parameter("leader_pose_topic", "/coord/leader_estimate")

        declare_yaml_param(self, "tick_hz", descriptor=dyn_num)
        declare_yaml_param(self, "d_target", descriptor=dyn_num)
        declare_yaml_param(self, "xy_anchor_max", descriptor=dyn_num)
        declare_yaml_param(self, "seed_uav_cmd_on_start")
        declare_yaml_param(self, "uav_start_x", descriptor=dyn_num)
        declare_yaml_param(self, "uav_start_y", descriptor=dyn_num)
        declare_yaml_param(self, "uav_start_z", descriptor=dyn_num)
        declare_yaml_param(self, "uav_start_yaw_deg", descriptor=dyn_num)
        declare_yaml_param(self, "require_uav_actual_before_motion")
        declare_yaml_param(self, "use_uav_actual_z_on_start")
        declare_yaml_param(self, "start_delay_s", descriptor=dyn_num)
        declare_yaml_param(self, "follow_yaw")
        declare_yaml_param(self, "pose_timeout_s", descriptor=dyn_num)
        declare_yaml_param(self, "min_cmd_period_s", descriptor=dyn_num)
        declare_yaml_param(self, "follow_speed_mps", descriptor=dyn_num)
        declare_yaml_param(self, "follow_speed_gain", descriptor=dyn_num)
        declare_yaml_param(self, "reverse_speed_feedforward_gain", descriptor=dyn_num)
        declare_yaml_param(self, "reverse_speed_min_mps", descriptor=dyn_num)
        declare_yaml_param(self, "leader_velocity_alpha", descriptor=dyn_num)
        declare_yaml_param(self, "leader_velocity_max_mps", descriptor=dyn_num)
        declare_yaml_param(self, "follow_z_offset_m", descriptor=dyn_num)
        declare_yaml_param(self, "d_target_slew_mps", descriptor=dyn_num)
        declare_yaml_param(self, "follow_z_slew_mps", descriptor=dyn_num)
        declare_yaml_param(self, "leader_heading_offset_deg", descriptor=dyn_num)
        declare_yaml_param(self, "leader_motion_heading_min_speed_mps", descriptor=dyn_num)
        declare_yaml_param(self, "leader_motion_heading_min_delta_m", descriptor=dyn_num)
        declare_yaml_param(self, "leader_motion_heading_alpha", descriptor=dyn_num)
        declare_yaml_param(self, "follow_yaw_rate_rad_s", descriptor=dyn_num)
        declare_yaml_param(self, "follow_yaw_rate_gain", descriptor=dyn_num)
        declare_yaml_param(self, "publish_pose_cmd_topics")

        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        leader_input_type_raw = str(self.get_parameter("leader_input_type").value).strip().lower()
        if leader_input_type_raw == "estimate":
            leader_input_type_raw = "pose"
        if leader_input_type_raw != "pose":
            raise ValueError(
                "follow_uav only supports 'pose'/'estimate'; "
                f"got leader_input_type={self.get_parameter('leader_input_type').value!r}; "
                "use follow_uav_odom for odom mode"
            )
        self.leader_input_type = leader_input_type_raw
        self.leader_pose_topic = str(self.get_parameter("leader_pose_topic").value)

        self.tick_hz = float(required_param_value(self, "tick_hz"))
        self.d_target = float(required_param_value(self, "d_target"))
        self.xy_anchor_max = float(required_param_value(self, "xy_anchor_max"))
        self.seed_uav_cmd_on_start = coerce_bool(required_param_value(self, "seed_uav_cmd_on_start"))
        self.uav_start_x = float(required_param_value(self, "uav_start_x"))
        self.uav_start_y = float(required_param_value(self, "uav_start_y"))
        self.uav_start_z = float(required_param_value(self, "uav_start_z"))
        self.uav_start_yaw = math.radians(float(required_param_value(self, "uav_start_yaw_deg")))
        self.require_uav_actual_before_motion = coerce_bool(
            required_param_value(self, "require_uav_actual_before_motion")
        )
        self.use_uav_actual_z_on_start = coerce_bool(
            required_param_value(self, "use_uav_actual_z_on_start")
        )
        self.start_delay_s = max(0.0, float(required_param_value(self, "start_delay_s")))
        self.follow_yaw = coerce_bool(required_param_value(self, "follow_yaw"))
        self.pose_timeout_s = float(required_param_value(self, "pose_timeout_s"))
        self.min_cmd_period_s = float(required_param_value(self, "min_cmd_period_s"))
        self.follow_speed_mps = float(required_param_value(self, "follow_speed_mps"))
        self.follow_speed_gain = float(required_param_value(self, "follow_speed_gain"))
        self.reverse_speed_feedforward_gain = float(
            required_param_value(self, "reverse_speed_feedforward_gain")
        )
        self.reverse_speed_min_mps = float(required_param_value(self, "reverse_speed_min_mps"))
        self.leader_velocity_alpha = float(required_param_value(self, "leader_velocity_alpha"))
        self.leader_velocity_max_mps = float(required_param_value(self, "leader_velocity_max_mps"))
        self.follow_z_offset_m = float(required_param_value(self, "follow_z_offset_m"))
        self.d_target_slew_mps = float(required_param_value(self, "d_target_slew_mps"))
        self.follow_z_slew_mps = float(required_param_value(self, "follow_z_slew_mps"))
        self.leader_heading_offset_rad = math.radians(
            float(required_param_value(self, "leader_heading_offset_deg"))
        )
        self.leader_motion_heading_min_speed_mps = float(
            required_param_value(self, "leader_motion_heading_min_speed_mps")
        )
        self.leader_motion_heading_min_delta_m = float(
            required_param_value(self, "leader_motion_heading_min_delta_m")
        )
        self.leader_motion_heading_alpha = float(
            required_param_value(self, "leader_motion_heading_alpha")
        )
        self.follow_yaw_rate_rad_s = float(required_param_value(self, "follow_yaw_rate_rad_s"))
        self.follow_yaw_rate_gain = float(required_param_value(self, "follow_yaw_rate_gain"))
        self.publish_pose_cmd_topics = coerce_bool(required_param_value(self, "publish_pose_cmd_topics"))

        self._validate_parameters()

        self.have_ugv = False
        self.ugv_pose = Pose2D(0.0, 0.0, 0.0)
        self.ugv_z = 0.0
        self.ugv_estimate_yaw = 0.0
        self.ugv_follow_heading = 0.0
        self.ugv_follow_heading_source = "startup"
        self.leader_frame_id = "map"
        self.last_ugv_stamp: Optional[Time] = None
        self._leader_heading_ref_xy: Optional[tuple[float, float]] = None
        self._leader_heading_ref_stamp: Optional[Time] = None
        self._leader_motion_last_xy: Optional[tuple[float, float]] = None
        self._leader_motion_last_stamp: Optional[Time] = None
        self._leader_velocity_x_mps = 0.0
        self._leader_velocity_y_mps = 0.0
        self._leader_heading_dir_xy: Optional[tuple[float, float]] = None
        self._last_logged_heading_source = ""

        self.uav_cmd = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.uav_cmd_z = self.uav_start_z
        self.have_uav_cmd = bool(self.seed_uav_cmd_on_start)

        self.uav_actual = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.uav_actual_z = self.uav_start_z
        self.have_uav_actual = False
        self.last_uav_actual_time: Optional[Time] = None
        self._latched_uav_actual_z_on_start = False

        self.last_cmd_time: Optional[Time] = None
        self._start_time: Optional[Time] = None
        self._startup_hold_logged = False
        self._waiting_for_actual_pose_logged = False
        self._stale_hold_logged = False
        self._active_d_target = self.d_target
        self._active_follow_z_offset_m = self.follow_z_offset_m
        self._last_follow_slew_time: Optional[Time] = None

        self.leader_sub = self.create_subscription(
            PoseStamped,
            self.leader_pose_topic,
            self.on_leader_pose,
            1,
        )
        self.uav_pose_sub = self.create_subscription(
            PoseStamped,
            f"/{self.uav_name}/pose",
            self.on_uav_pose,
            10,
        )
        self.uav_cmd_pub = self.create_publisher(
            Joy,
            f"/{self.uav_name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw",
            10,
        )
        self.pose_pub = (
            self.create_publisher(PoseStamped, f"/{self.uav_name}/pose_cmd", 10)
            if self.publish_pose_cmd_topics
            else None
        )
        self.pose_odom_pub = (
            self.create_publisher(Odometry, f"/{self.uav_name}/pose_cmd/odom", 10)
            if self.publish_pose_cmd_topics
            else None
        )

        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"[follow_uav] Started: world={self.world}, uav={self.uav_name}, "
            f"leader_pose={self.leader_pose_topic}, tick={self.tick_hz}Hz, "
            f"d_target={self.d_target:.2f}, xy_anchor_max={self.xy_anchor_max:.2f}, "
            f"follow_z_offset_m={self.follow_z_offset_m:.2f}, "
            f"reverse_ff_gain={self.reverse_speed_feedforward_gain:.2f}, "
            f"velocity_alpha={self.leader_velocity_alpha:.2f}, "
            f"d_target_slew={self.d_target_slew_mps:.2f}m/s, "
            f"follow_z_slew={self.follow_z_slew_mps:.2f}m/s, "
            f"leader_heading_offset_deg={math.degrees(self.leader_heading_offset_rad):.1f}, "
            f"motion_heading_min_speed={self.leader_motion_heading_min_speed_mps:.2f}m/s, "
            f"follow_yaw={self.follow_yaw}, publish_pose_cmd_topics={self.publish_pose_cmd_topics}, "
            f"uav_start=({self.uav_start_x:.2f},{self.uav_start_y:.2f},"
            f"{self.uav_start_z:.2f},{math.degrees(self.uav_start_yaw):.1f}deg)"
        )

    def _validate_parameters(self) -> None:
        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.d_target <= 0.0:
            raise ValueError("d_target must be > 0")
        if self.xy_anchor_max <= 0.0:
            raise ValueError("xy_anchor_max must be > 0")
        if self.pose_timeout_s <= 0.0:
            raise ValueError("pose_timeout_s must be > 0")
        if self.min_cmd_period_s < 0.0:
            raise ValueError("min_cmd_period_s must be >= 0")
        if (
            self.follow_speed_mps < 0.0
            or self.follow_speed_gain < 0.0
            or self.reverse_speed_feedforward_gain < 0.0
            or self.reverse_speed_min_mps < 0.0
            or self.leader_velocity_max_mps < 0.0
            or self.follow_z_offset_m < 0.0
            or self.d_target_slew_mps < 0.0
            or self.follow_z_slew_mps < 0.0
            or self.leader_motion_heading_min_speed_mps < 0.0
            or self.leader_motion_heading_min_delta_m < 0.0
            or self.follow_yaw_rate_rad_s < 0.0
            or self.follow_yaw_rate_gain < 0.0
        ):
            raise ValueError(
                "follow speed and reverse feed-forward parameters, follow_z_offset_m, d_target_slew_mps, "
                "follow_z_slew_mps, leader motion heading thresholds, follow_yaw_rate_rad_s, "
                "and follow_yaw_rate_gain must be >= 0"
            )
        if self.follow_z_offset_m > self.d_target:
            raise ValueError("follow_z_offset_m must be <= d_target to preserve the 3D follow distance")
        if not (0.0 <= self.leader_motion_heading_alpha <= 1.0):
            raise ValueError("leader_motion_heading_alpha must be within [0, 1]")
        if not (0.0 <= self.leader_velocity_alpha <= 1.0):
            raise ValueError("leader_velocity_alpha must be within [0, 1]")

    def _on_set_parameters(self, params):
        numeric_nonnegative = {
            "follow_speed_mps",
            "follow_speed_gain",
            "reverse_speed_feedforward_gain",
            "reverse_speed_min_mps",
            "leader_velocity_max_mps",
            "leader_motion_heading_min_speed_mps",
            "leader_motion_heading_min_delta_m",
            "follow_yaw_rate_rad_s",
            "follow_yaw_rate_gain",
            "min_cmd_period_s",
            "start_delay_s",
            "follow_z_offset_m",
            "d_target_slew_mps",
            "follow_z_slew_mps",
        }
        numeric_positive = {
            "d_target",
            "xy_anchor_max",
            "tick_hz",
            "pose_timeout_s",
        }
        updates = {}

        for param in params:
            if param.name in numeric_nonnegative or param.name in numeric_positive:
                try:
                    value = float(param.value)
                except Exception as exc:
                    return SetParametersResult(successful=False, reason=f"invalid {param.name}: {exc}")
                if param.name in numeric_positive and value <= 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be > 0")
                if param.name in numeric_nonnegative and value < 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be >= 0")
                updates[param.name] = value
            elif param.name == "leader_heading_offset_deg":
                try:
                    updates[param.name] = float(param.value)
                except Exception as exc:
                    return SetParametersResult(successful=False, reason=f"invalid {param.name}: {exc}")
            elif param.name == "follow_yaw":
                updates[param.name] = coerce_bool(param.value)
            elif param.name in {"leader_motion_heading_alpha", "leader_velocity_alpha"}:
                try:
                    value = float(param.value)
                except Exception as exc:
                    return SetParametersResult(successful=False, reason=f"invalid {param.name}: {exc}")
                if not (0.0 <= value <= 1.0):
                    return SetParametersResult(successful=False, reason=f"{param.name} must be within [0, 1]")
                updates[param.name] = value

        if not updates:
            return SetParametersResult(successful=True)

        next_d_target = float(updates.get("d_target", self.d_target))
        next_z_offset = float(updates.get("follow_z_offset_m", self.follow_z_offset_m))
        if next_z_offset > next_d_target:
            return SetParametersResult(
                successful=False,
                reason="follow_z_offset_m must be <= d_target to preserve the 3D follow distance",
            )

        for name, value in updates.items():
            if name == "leader_heading_offset_deg":
                self.leader_heading_offset_rad = math.radians(float(value))
            else:
                setattr(self, name, value)

        """ self.get_logger().info(
            "[follow_uav] Runtime parameter update: "
            + ", ".join(f"{name}={value}" for name, value in sorted(updates.items()))
        ) """
        return SetParametersResult(successful=True)

    def _set_leader_follow_heading(self, yaw: float, source: str) -> None:
        self.ugv_follow_heading = wrap_pi(yaw)
        self.ugv_follow_heading_source = source
        if source != self._last_logged_heading_source:
            self.get_logger().info(
                f"[follow_uav] Leader follow heading source: {source}"
            )
            self._last_logged_heading_source = source

    def _set_leader_heading_dir(self, dir_x: float, dir_y: float) -> None:
        norm = math.hypot(dir_x, dir_y)
        if norm <= 1e-6:
            return
        next_x = float(dir_x / norm)
        next_y = float(dir_y / norm)
        if self._leader_heading_dir_xy is not None and self.leader_motion_heading_alpha < 1.0:
            prev_x, prev_y = self._leader_heading_dir_xy
            alpha = self.leader_motion_heading_alpha
            blend_x = (1.0 - alpha) * prev_x + alpha * next_x
            blend_y = (1.0 - alpha) * prev_y + alpha * next_y
            blend_norm = math.hypot(blend_x, blend_y)
            if blend_norm > 1e-6:
                next_x = float(blend_x / blend_norm)
                next_y = float(blend_y / blend_norm)
        self._leader_heading_dir_xy = (next_x, next_y)
        self._set_leader_follow_heading(math.atan2(next_y, next_x), "motion_heading")

    def _startup_heading_from_view(self, leader_x: float, leader_y: float) -> Optional[float]:
        if not self.have_uav_actual:
            return None
        dx = float(leader_x) - self.uav_actual.x
        dy = float(leader_y) - self.uav_actual.y
        if math.hypot(dx, dy) <= 1e-6:
            return None
        return math.atan2(dy, dx)

    def _update_leader_follow_heading(
        self,
        leader_x: float,
        leader_y: float,
        estimate_yaw: float,
        stamp: Time,
    ) -> None:
        if self._leader_heading_ref_xy is None or self._leader_heading_ref_stamp is None:
            self._leader_heading_ref_xy = (float(leader_x), float(leader_y))
            self._leader_heading_ref_stamp = stamp
            self._leader_motion_last_xy = (float(leader_x), float(leader_y))
            self._leader_motion_last_stamp = stamp
            startup_yaw = self._startup_heading_from_view(leader_x, leader_y)
            if startup_yaw is not None:
                self._leader_heading_dir_xy = (math.cos(startup_yaw), math.sin(startup_yaw))
                self._set_leader_follow_heading(startup_yaw, "view_line_startup")
            else:
                self._leader_heading_dir_xy = (math.cos(estimate_yaw), math.sin(estimate_yaw))
                self._set_leader_follow_heading(estimate_yaw, "estimate_yaw_startup")
            return

        if self._leader_motion_last_xy is None or self._leader_motion_last_stamp is None:
            self._leader_motion_last_xy = (float(leader_x), float(leader_y))
            self._leader_motion_last_stamp = stamp
            return

        dt_s = (stamp - self._leader_motion_last_stamp).nanoseconds * 1e-9
        last_x, last_y = self._leader_motion_last_xy
        self._leader_motion_last_xy = (float(leader_x), float(leader_y))
        self._leader_motion_last_stamp = stamp
        if dt_s <= 1e-3:
            return
        if dt_s > self.pose_timeout_s:
            self._leader_heading_ref_xy = (float(leader_x), float(leader_y))
            self._leader_heading_ref_stamp = stamp
            self._leader_velocity_x_mps = 0.0
            self._leader_velocity_y_mps = 0.0
            return

        meas_vx_mps = (float(leader_x) - last_x) / dt_s
        meas_vy_mps = (float(leader_y) - last_y) / dt_s
        meas_speed_mps = math.hypot(meas_vx_mps, meas_vy_mps)
        if self.leader_velocity_max_mps > 0.0 and meas_speed_mps > self.leader_velocity_max_mps:
            scale = self.leader_velocity_max_mps / meas_speed_mps
            meas_vx_mps *= scale
            meas_vy_mps *= scale
            meas_speed_mps = self.leader_velocity_max_mps
        alpha = self.leader_velocity_alpha
        self._leader_velocity_x_mps = (
            (1.0 - alpha) * self._leader_velocity_x_mps + alpha * meas_vx_mps
        )
        self._leader_velocity_y_mps = (
            (1.0 - alpha) * self._leader_velocity_y_mps + alpha * meas_vy_mps
        )
        ref_x, ref_y = self._leader_heading_ref_xy
        dx = float(leader_x) - ref_x
        dy = float(leader_y) - ref_y
        distance_m = math.hypot(dx, dy)
        if (
            distance_m >= self.leader_motion_heading_min_delta_m
            and meas_speed_mps >= self.leader_motion_heading_min_speed_mps
        ):
            self._set_leader_heading_dir(dx, dy)
            self._leader_heading_ref_xy = (float(leader_x), float(leader_y))
            self._leader_heading_ref_stamp = stamp
            return

        if self.ugv_follow_heading_source == "estimate_yaw_startup":
            startup_yaw = self._startup_heading_from_view(leader_x, leader_y)
            if startup_yaw is not None:
                self._leader_heading_dir_xy = (math.cos(startup_yaw), math.sin(startup_yaw))
                self._set_leader_follow_heading(startup_yaw, "view_line_startup")
                return

        if self._leader_heading_dir_xy is not None:
            dir_x, dir_y = self._leader_heading_dir_xy
            self._set_leader_follow_heading(
                math.atan2(dir_y, dir_x),
                self.ugv_follow_heading_source or "held_heading",
            )

    def on_leader_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        try:
            stamp = Time.from_msg(msg.header.stamp)
        except (ValueError, TypeError, AttributeError):
            stamp = self.get_clock().now()

        self.ugv_pose = Pose2D(float(p.x), float(p.y), yaw)
        self.ugv_z = float(p.z)
        self.ugv_estimate_yaw = yaw
        self.leader_frame_id = msg.header.frame_id or "map"
        self._update_leader_follow_heading(float(p.x), float(p.y), yaw, stamp)
        self.have_ugv = True
        self.last_ugv_stamp = stamp

    def on_uav_pose(self, msg: PoseStamped) -> None:
        super().on_uav_pose(msg)
        if not self.use_uav_actual_z_on_start or self._latched_uav_actual_z_on_start:
            return

        self.uav_start_z = self.uav_actual_z
        self.uav_cmd_z = self.uav_actual_z
        self._latched_uav_actual_z_on_start = True
        self.get_logger().info(
            f"[follow_uav] Using current UAV z as startup altitude: "
            f"z={self.uav_start_z:.2f}, follow_z_offset_m={self.follow_z_offset_m:.2f}"
        )

    def _uav_actual_is_fresh(self, now: Time) -> bool:
        if not self.have_uav_actual or self.last_uav_actual_time is None:
            return False
        age_s = (now - self.last_uav_actual_time).nanoseconds * 1e-9
        return age_s <= self.pose_timeout_s

    def _target_uav_z(self) -> float:
        return self.ugv_z + self._active_follow_z_offset_m

    def _slew_follow_targets(self, now: Time) -> None:
        desired_d = float(self.d_target)
        desired_z = min(float(self.follow_z_offset_m), desired_d)

        if self._last_follow_slew_time is None:
            self._last_follow_slew_time = now
            self._active_follow_z_offset_m = min(self._active_follow_z_offset_m, self._active_d_target)
            return

        dt_s = max(0.0, (now - self._last_follow_slew_time).nanoseconds * 1e-9)
        self._last_follow_slew_time = now
        if dt_s <= 0.0:
            return
        dt_s = min(dt_s, 1.0)

        d_slew_target = max(desired_d, self._active_follow_z_offset_m)
        if self.d_target_slew_mps > 0.0:
            self._active_d_target = move_towards(
                self._active_d_target,
                d_slew_target,
                self.d_target_slew_mps * dt_s,
            )
        else:
            self._active_d_target = d_slew_target

        desired_z = min(desired_z, self._active_d_target)
        if self.follow_z_slew_mps > 0.0:
            self._active_follow_z_offset_m = move_towards(
                self._active_follow_z_offset_m,
                desired_z,
                self.follow_z_slew_mps * dt_s,
            )
        else:
            self._active_follow_z_offset_m = desired_z
        self._active_follow_z_offset_m = min(self._active_follow_z_offset_m, self._active_d_target)

    def _compute_anchor_pose(self, target_horizontal_distance: float) -> Pose2D:
        heading = wrap_pi(self.ugv_follow_heading + self.leader_heading_offset_rad)
        anchor_x = self.ugv_pose.x - target_horizontal_distance * math.cos(heading)
        anchor_y = self.ugv_pose.y - target_horizontal_distance * math.sin(heading)
        anchor_x, anchor_y = clamp_point_to_radius(
            self.ugv_pose.x,
            self.ugv_pose.y,
            anchor_x,
            anchor_y,
            self.xy_anchor_max,
        )
        yaw_target = solve_yaw_to_target(anchor_x, anchor_y, self.ugv_pose.x, self.ugv_pose.y)
        return Pose2D(anchor_x, anchor_y, yaw_target)

    def _effective_follow_speed_mps(self, current_uav: Pose2D, anchor_pose: Pose2D) -> float:
        error = math.hypot(anchor_pose.x - current_uav.x, anchor_pose.y - current_uav.y)
        leader_to_uav_x = current_uav.x - self.ugv_pose.x
        leader_to_uav_y = current_uav.y - self.ugv_pose.y
        leader_to_uav_distance = math.hypot(leader_to_uav_x, leader_to_uav_y)
        closing_speed_mps = 0.0
        if leader_to_uav_distance > 1e-6:
            closing_speed_mps = max(
                0.0,
                (
                    self._leader_velocity_x_mps * leader_to_uav_x
                    + self._leader_velocity_y_mps * leader_to_uav_y
                )
                / leader_to_uav_distance,
            )
        if closing_speed_mps < self.reverse_speed_min_mps:
            closing_speed_mps = 0.0
        return min(
            self.follow_speed_mps,
            self.follow_speed_gain * max(error, 0.0)
            + self.reverse_speed_feedforward_gain * closing_speed_mps,
        )

    def _compute_command_yaw(self, current_yaw: float, target_yaw: float) -> float:
        if not self.follow_yaw:
            return current_yaw
        yaw_error = wrap_pi(target_yaw - current_yaw)
        yaw_rate_rad_s = min(
            max(self.follow_yaw_rate_rad_s, 0.0),
            max(self.follow_yaw_rate_gain, 0.0) * abs(yaw_error),
        )
        yaw_step_limit = yaw_rate_rad_s / self.tick_hz if yaw_rate_rad_s > 0.0 else 0.0
        if yaw_step_limit > 0.0 and abs(yaw_error) > yaw_step_limit:
            return wrap_pi(current_yaw + math.copysign(yaw_step_limit, yaw_error))
        return wrap_pi(target_yaw)

    def _publish_current_pose_hold(self, now: Time) -> bool:
        if not self._uav_actual_is_fresh(now):
            return False
        if not self.can_send_command_now(now):
            return True

        x_cmd = self.uav_actual.x
        y_cmd = self.uav_actual.y
        z_cmd = self.uav_actual_z
        yaw_cmd = self.uav_actual.yaw

        self.publish_legacy_uav_command(x_cmd, y_cmd, z_cmd, yaw_cmd)
        self.publish_pose_cmd(x_cmd, y_cmd, z_cmd, yaw_cmd)
        self.publish_pose_cmd_odom(x_cmd, y_cmd, z_cmd, yaw_cmd)

        self.uav_cmd = Pose2D(x_cmd, y_cmd, yaw_cmd)
        self.have_uav_cmd = True
        self.last_cmd_time = now
        return True


    def on_tick(self) -> None:
        now = self.get_clock().now()
        if self.require_uav_actual_before_motion and not self.have_uav_actual:
            if not self._waiting_for_actual_pose_logged:
                self.get_logger().info("[follow_uav] Waiting for UAV pose before motion")
                self._waiting_for_actual_pose_logged = True
            return
        self._waiting_for_actual_pose_logged = False

        if self.start_delay_s > 0.0:
            if self._start_time is None:
                self._start_time = now
            elapsed_s = max(0.0, (now - self._start_time).nanoseconds * 1e-9)
            if elapsed_s < self.start_delay_s:
                if not self._startup_hold_logged:
                    self.get_logger().info(
                        f"[follow_uav] Holding commands for {self.start_delay_s:.1f}s"
                    )
                    self._startup_hold_logged = True
                return
            self._startup_hold_logged = False

        if not self.ugv_pose_is_fresh(now):
            if self._publish_current_pose_hold(now):
                if not self._stale_hold_logged:
                    self.get_logger().info("[follow_uav] Holding current UAV pose until leader estimate is fresh")
                    self._stale_hold_logged = True
            return
        self._stale_hold_logged = False

        if not self.can_send_command_now(now):
            return

        self._slew_follow_targets(now)
        current_uav = self._current_uav_pose()
        target_uav_z = self._target_uav_z()
        vertical_delta = target_uav_z - self.ugv_z
        target_horizontal_distance = horizontal_distance_for_euclidean(self._active_d_target, vertical_delta)
        anchor_pose = self._compute_anchor_pose(target_horizontal_distance)

        dx = anchor_pose.x - current_uav.x
        dy = anchor_pose.y - current_uav.y
        distance_to_anchor = math.hypot(dx, dy)
        speed_mps = self._effective_follow_speed_mps(current_uav, anchor_pose)
        step_distance = speed_mps / self.tick_hz if self.tick_hz > 0.0 else speed_mps
        if distance_to_anchor < 0.2 or distance_to_anchor <= 1e-9 or step_distance <= 0.0:
            x_cmd = current_uav.x
            y_cmd = current_uav.y
        else:
            step_distance = min(step_distance, distance_to_anchor)
            scale = step_distance / distance_to_anchor
            x_cmd = current_uav.x + dx * scale
            y_cmd = current_uav.y + dy * scale

        z_cmd = target_uav_z
        yaw_target = solve_yaw_to_target(x_cmd, y_cmd, self.ugv_pose.x, self.ugv_pose.y)
        yaw_cmd = self._compute_command_yaw(current_uav.yaw, yaw_target)

        self.publish_legacy_uav_command(x_cmd, y_cmd, z_cmd, yaw_cmd)
        self.publish_pose_cmd(x_cmd, y_cmd, z_cmd, yaw_cmd)
        self.publish_pose_cmd_odom(x_cmd, y_cmd, z_cmd, yaw_cmd)

        self.uav_cmd = Pose2D(x_cmd, y_cmd, yaw_cmd)
        self.have_uav_cmd = True
        self.last_cmd_time = now


def main(args=None):
    rclpy.init(args=args)
    node = FollowUav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
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
