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

from lrs_halmstad.common.node_mixins import EventEmitterMixin
from lrs_halmstad.common.ros_params import declare_yaml_param, required_param_value
from lrs_halmstad.follow.follow_core import (
    FollowControllerCoreMixin,
    compute_rate_limited_axis_value,
)
from lrs_halmstad.follow.follow_debug import FollowDebugPublishers
from lrs_halmstad.follow.follow_math import (
    Pose2D,
    clamp_point_to_radius,
    coerce_bool,
    compute_leader_look_target,
    solve_yaw_to_target,
    wrap_pi,
    yaw_from_quat,
    horizontal_distance_for_euclidean,
    vertical_distance_for_euclidean,
)

class FollowUavOdom(EventEmitterMixin, FollowControllerCoreMixin, Node):
    def __init__(self):
        super().__init__("follow_uav")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("world", "orchard")
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("leader_odom_topic", "/a201_0000/amcl_pose_odom")

        declare_yaml_param(self, "tick_hz", descriptor=dyn_num)
        declare_yaml_param(self, "d_target", descriptor=dyn_num)
        self.declare_parameter("d_max", 20.0)
        declare_yaml_param(self, "xy_anchor_max", descriptor=dyn_num)
        declare_yaml_param(self, "xy_min", descriptor=dyn_num)
        declare_yaml_param(self, "z_min", descriptor=dyn_num)
        self.declare_parameter("z_max", 0.0, dyn_num)
        self.declare_parameter("xy_max", 0.0, dyn_num)
        self.declare_parameter("d_euclidean", 0.0, dyn_num)
        declare_yaml_param(self, "seed_uav_cmd_on_start")
        declare_yaml_param(self, "uav_start_x")
        declare_yaml_param(self, "uav_start_y")
        declare_yaml_param(self, "uav_start_z")
        declare_yaml_param(self, "uav_start_yaw_deg")

        declare_yaml_param(self, "follow_yaw")
        declare_yaml_param(self, "pose_timeout_s")
        declare_yaml_param(self, "min_cmd_period_s")
        declare_yaml_param(self, "follow_speed_mps")
        declare_yaml_param(self, "follow_speed_gain")
        declare_yaml_param(self, "uav_xy_command_mode")
        declare_yaml_param(self, "uav_xy_command_step_scale")
        declare_yaml_param(self, "follow_z_speed_mps")
        declare_yaml_param(self, "follow_z_speed_gain")
        declare_yaml_param(self, "follow_yaw_rate_rad_s")
        declare_yaml_param(self, "follow_yaw_rate_gain")

        declare_yaml_param(self, "event_topic")
        declare_yaml_param(self, "publish_events")
        declare_yaml_param(self, "publish_metrics")
        declare_yaml_param(self, "publish_debug_topics")
        declare_yaml_param(self, "publish_pose_cmd_topics")
        declare_yaml_param(self, "metrics_prefix")
        declare_yaml_param(self, "camera_x_offset_m")
        declare_yaml_param(self, "camera_y_offset_m")
        declare_yaml_param(self, "camera_z_offset_m")
        declare_yaml_param(self, "leader_look_target_x_m")
        declare_yaml_param(self, "leader_look_target_y_m")
        self.declare_parameter("start_delay_s", 0.0)

        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.leader_odom_topic = str(self.get_parameter("leader_odom_topic").value)

        self.tick_hz = float(required_param_value(self, "tick_hz"))
        self.d_target = float(required_param_value(self, "d_target"))
        legacy_d_max = float(self.get_parameter("d_max").value)
        xy_anchor_max_param = float(required_param_value(self, "xy_anchor_max"))
        self.xy_anchor_max = xy_anchor_max_param if xy_anchor_max_param > 0.0 else legacy_d_max
        self.xy_min = float(required_param_value(self, "xy_min"))
        self.z_min = float(required_param_value(self, "z_min"))
        self.z_max = float(self.get_parameter("z_max").value)
        self.xy_max = float(self.get_parameter("xy_max").value)
        legacy_d_target = float(self.get_parameter("d_euclidean").value)
        if legacy_d_target > 0.0:
            self.d_target = legacy_d_target
        self.d_target = max(self.d_target, 0.01)
        self.xy_target = 0.0
        self.seed_uav_cmd_on_start = coerce_bool(required_param_value(self, "seed_uav_cmd_on_start"))
        self.uav_start_x = float(required_param_value(self, "uav_start_x"))
        self.uav_start_y = float(required_param_value(self, "uav_start_y"))
        self.uav_start_z = float(required_param_value(self, "uav_start_z"))
        self.uav_start_yaw = math.radians(float(required_param_value(self, "uav_start_yaw_deg")))

        self.follow_yaw = coerce_bool(required_param_value(self, "follow_yaw"))
        self.pose_timeout_s = float(required_param_value(self, "pose_timeout_s"))
        self.min_cmd_period_s = float(required_param_value(self, "min_cmd_period_s"))
        self.follow_speed_mps = float(required_param_value(self, "follow_speed_mps"))
        self.follow_speed_gain = float(required_param_value(self, "follow_speed_gain"))
        self.uav_xy_command_mode = str(required_param_value(self, "uav_xy_command_mode")).strip().lower()
        self.uav_xy_command_step_scale = float(required_param_value(self, "uav_xy_command_step_scale"))
        self.follow_z_speed_mps = float(required_param_value(self, "follow_z_speed_mps"))
        self.follow_z_speed_gain = float(required_param_value(self, "follow_z_speed_gain"))
        self.follow_yaw_rate_rad_s = float(required_param_value(self, "follow_yaw_rate_rad_s"))
        self.follow_yaw_rate_gain = float(required_param_value(self, "follow_yaw_rate_gain"))

        self.event_topic = str(required_param_value(self, "event_topic"))
        self.publish_events = coerce_bool(required_param_value(self, "publish_events"))
        self.publish_metrics = coerce_bool(required_param_value(self, "publish_metrics"))
        self.publish_debug_topics = coerce_bool(required_param_value(self, "publish_debug_topics"))
        self.publish_pose_cmd_topics = coerce_bool(required_param_value(self, "publish_pose_cmd_topics"))
        self.metrics_prefix = str(required_param_value(self, "metrics_prefix")).rstrip("/") or "/coord"
        self.camera_x_offset_m = float(required_param_value(self, "camera_x_offset_m"))
        self.camera_y_offset_m = float(required_param_value(self, "camera_y_offset_m"))
        self.camera_z_offset_m = float(required_param_value(self, "camera_z_offset_m"))
        self.leader_look_target_x_m = float(required_param_value(self, "leader_look_target_x_m"))
        self.leader_look_target_y_m = float(required_param_value(self, "leader_look_target_y_m"))
        self.start_delay_s = max(0.0, float(self.get_parameter("start_delay_s").value))
        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.z_min < 0.0:
            raise ValueError("z_min must be >= 0")
        if self.xy_min < 0.0:
            raise ValueError("xy_min must be >= 0")
        if self.z_max > 0.0 and self.z_max < self.z_min:
            raise ValueError("z_max must be >= z_min when enabled")
        if self.xy_max < 0.0:
            raise ValueError("xy_max must be >= 0")
        if self.xy_max > 0.0 and self.xy_max < self.xy_min:
            raise ValueError("xy_max must be >= xy_min when enabled")
        if self.xy_anchor_max <= 0.0:
            raise ValueError("xy_anchor_max must be > 0")
        if (
            self.follow_speed_mps < 0.0
            or self.follow_speed_gain < 0.0
            or self.uav_xy_command_step_scale < 0.0
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
        if self.uav_xy_command_mode not in ("direct", "controller_step"):
            raise ValueError("uav_xy_command_mode must be 'direct' or 'controller_step'")

        self._refresh_xy_target()
        self.have_ugv = False
        self.ugv_pose = Pose2D(0.0, 0.0, 0.0)
        self.ugv_z = 0.0
        self.ugv_follow_heading = 0.0
        self.last_ugv_stamp: Optional[Time] = None

        self.uav_cmd = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.uav_cmd_z = self.uav_start_z
        self.have_uav_cmd = bool(self.seed_uav_cmd_on_start)

        self.uav_actual = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.uav_actual_z = self.uav_start_z
        self.have_uav_actual = False
        self.last_uav_actual_time: Optional[Time] = None
        self.last_debug_actual_yaw_unwrapped: Optional[float] = None

        self.current_leader_distance_xy_m = max(0.01, self.xy_target)
        self.current_leader_distance_3d_m = math.hypot(self.current_leader_distance_xy_m, self.uav_start_z)
        self.last_cmd_time: Optional[Time] = None
        self._start_time: Optional[Time] = None
        self._startup_hold_logged = False

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
        self.uav_cmd_pub = self.create_publisher(
            Joy,
            f"/{self.uav_name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw",
            10,
        )
        self._setup_event_emitter(self.event_topic, self.publish_events)
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
        self.debug_pubs = (
            FollowDebugPublishers(self, self.uav_name)
            if self.publish_debug_topics
            else None
        )

        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"[follow_uav_odom] Started: world={self.world}, uav={self.uav_name}, "
            f"leader_odom={self.leader_odom_topic}, tick={self.tick_hz}Hz, "
            f"d_target={self.d_target}, xy_target={self.xy_target}, xy_min={self.xy_min}, "
            f"xy_anchor_max={self.xy_anchor_max}, z_min={self.z_min}, z_max={self.z_max}, xy_max={self.xy_max}, "
            f"follow_speed_mps={self.follow_speed_mps}, "
            f"follow_speed_gain={self.follow_speed_gain}, "
            f"uav_xy_command_mode={self.uav_xy_command_mode}, "
            f"uav_xy_command_step_scale={self.uav_xy_command_step_scale}, "
            f"follow_z_speed_mps={self.follow_z_speed_mps}, "
            f"follow_z_speed_gain={self.follow_z_speed_gain}, "
            f"follow_yaw_rate_rad_s={self.follow_yaw_rate_rad_s}, "
            f"follow_yaw_rate_gain={self.follow_yaw_rate_gain}, "
            f"publish_debug_topics={self.publish_debug_topics}, publish_pose_cmd_topics={self.publish_pose_cmd_topics}, "
            f"camera_offset_m=({self.camera_x_offset_m}, {self.camera_y_offset_m}, {self.camera_z_offset_m}), "
            f"leader_look_target_xy_m=({self.leader_look_target_x_m}, {self.leader_look_target_y_m}), "
            f"uav_start=({self.uav_start_x:.2f},{self.uav_start_y:.2f},{self.uav_start_z:.2f},{math.degrees(self.uav_start_yaw):.1f}deg)"
        )
        self.emit_event("FOLLOW_ODOM_NODE_START")

    def _on_set_parameters(self, params):
        updates = {}
        runtime_updates = {}
        for param in params:
            try:
                value = float(param.value)
            except Exception as exc:
                if param.name in (
                    "d_euclidean",
                    "d_target",
                    "xy_min",
                    "z_min",
                    "z_max",
                    "xy_max",
                    "follow_speed_mps",
                    "follow_speed_gain",
                    "follow_z_speed_mps",
                    "follow_z_speed_gain",
                    "follow_yaw_rate_rad_s",
                    "follow_yaw_rate_gain",
                ):
                    return SetParametersResult(successful=False, reason=f"invalid {param.name}: {exc}")
                continue

            if param.name in ("d_euclidean", "d_target", "xy_min", "z_min", "z_max", "xy_max"):
                if param.name in ("xy_min", "z_min", "z_max", "xy_max"):
                    if value < 0.0:
                        return SetParametersResult(successful=False, reason=f"{param.name} must be >= 0")
                elif value <= 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be > 0")
                updates[param.name] = value
            elif param.name in (
                "follow_speed_mps",
                "follow_speed_gain",
                "follow_z_speed_mps",
                "follow_z_speed_gain",
                "follow_yaw_rate_rad_s",
                "follow_yaw_rate_gain",
            ):
                if value < 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be >= 0")
                runtime_updates[param.name] = value

        if not updates and not runtime_updates:
            return SetParametersResult(successful=True)

        if updates:
            new_z_min = updates.get("z_min", self.z_min)
            new_z_max = updates.get("z_max", self.z_max)
            new_xy_min = updates.get("xy_min", self.xy_min)
            new_xy_max = updates.get("xy_max", self.xy_max)
            if new_z_max > 0.0 and new_z_max < new_z_min:
                return SetParametersResult(successful=False, reason="z_max must be >= z_min")
            if new_xy_max > 0.0 and new_xy_max < new_xy_min:
                return SetParametersResult(successful=False, reason="xy_max must be >= xy_min")
            explicit_d_target = updates.get("d_target")
            legacy_d_target = updates.get("d_euclidean")
            if explicit_d_target is not None and legacy_d_target is not None:
                if not math.isclose(
                    explicit_d_target,
                    legacy_d_target,
                    rel_tol=1e-6,
                    abs_tol=1e-6,
                ):
                    return SetParametersResult(
                        successful=False,
                        reason="d_target and d_euclidean conflict",
                    )
            new_d_target = (
                explicit_d_target
                if explicit_d_target is not None
                else legacy_d_target
                if legacy_d_target is not None
                else self.d_target
            )
            derived_z_cap = vertical_distance_for_euclidean(new_d_target, new_xy_min)
            effective_z_cap = max(new_z_min, derived_z_cap)
            if new_z_max > 0.0:
                effective_z_cap = min(effective_z_cap, new_z_max)
            preferred_z = min(max(new_z_min, self.uav_start_z), effective_z_cap)
            effective_xy_target = horizontal_distance_for_euclidean(new_d_target, preferred_z)
            derived_xy_cap = horizontal_distance_for_euclidean(new_d_target, new_z_min)
            if derived_xy_cap > 0.0:
                effective_xy_target = min(effective_xy_target, derived_xy_cap)
            if new_xy_max > 0.0:
                effective_xy_target = min(effective_xy_target, new_xy_max)
            effective_xy_target = max(effective_xy_target, new_xy_min)

            if effective_xy_target >= self.xy_anchor_max:
                return SetParametersResult(
                    successful=False,
                    reason="bounded xy target must remain < xy_anchor_max",
                )

            self.d_target = max(new_d_target, 0.01)
            self.z_min = max(new_z_min, 0.0)
            self.z_max = max(new_z_max, 0.0)
            self.xy_min = max(new_xy_min, 0.0)
            self.xy_max = max(new_xy_max, 0.0)
            self._refresh_xy_target()
            self._apply_runtime_distance_update()

        if runtime_updates:
            for name, value in runtime_updates.items():
                setattr(self, name, float(value))
        update_parts = []
        if runtime_updates:
            update_parts.append(", ".join(f"{name}={value}" for name, value in sorted(runtime_updates.items())))
        if updates:
            update_parts.append(
                f"d_target={self.d_target:.2f}, xy_target={self.xy_target:.2f}, xy_min={self.xy_min:.2f}, "
                f"z_min={self.z_min:.2f}, z_max={self.z_max:.2f}, xy_max={self.xy_max:.2f}"
            )
        if update_parts:
            self.get_logger().info(f"[follow_uav_odom] Runtime parameter update: {', '.join(update_parts)}")
        return SetParametersResult(successful=True)

    def _current_uav_z(self) -> float:
        # Odom mode always has uav_cmd_z seeded at startup; drop the have_uav_cmd
        # guard and uav_start_z fallback that the mixin carries for the generic case.
        if self.have_uav_actual:
            return self.uav_actual_z
        return self.uav_cmd_z

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

    def _current_follow_geometry(self):
        current_uav = self._current_uav_pose()
        # Body-motion geometry is defined from the UAV body pose to the leader pose.
        # Camera-relative geometry stays in camera_tracker.py.
        horizontal_distance = math.hypot(self.ugv_pose.x - current_uav.x, self.ugv_pose.y - current_uav.y)
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

    def _distance_targets_for_geometry(self, leader_z: float) -> tuple[float, float]:
        return self._nominal_target_pair(leader_z)

    def _compute_anchor_target(self, target_horizontal_distance: float) -> Pose2D:
        xt = self.ugv_pose.x - target_horizontal_distance * math.cos(self.ugv_follow_heading)
        yt = self.ugv_pose.y - target_horizontal_distance * math.sin(self.ugv_follow_heading)
        xt, yt = clamp_point_to_radius(
            self.ugv_pose.x,
            self.ugv_pose.y,
            xt,
            yt,
            self.xy_anchor_max,
        )
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

    def _effective_follow_speed_mps(self, anchor_distance_error: float) -> float:
        return min(
            self.follow_speed_mps,
            self.follow_speed_gain * max(anchor_distance_error, 0.0),
        )

    def _compute_command_z(self, current_z: float, target_z: float) -> float:
        return compute_rate_limited_axis_value(
            current_z,
            target_z,
            tick_hz=self.tick_hz,
            max_speed_per_s=self.follow_z_speed_mps,
            gain=self.follow_z_speed_gain,
        )

    def _publish_follow_debug(
        self,
        anchor_target: Pose2D,
        *,
        target_horizontal_distance: float,
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
        if self.debug_pubs is None:
            return
        now_msg = self.get_clock().now().to_msg()
        anchor_distance_error, anchor_along_error, anchor_cross_error = self._anchor_errors(anchor_target, current_uav)
        self.debug_pubs.publish(
            stamp=now_msg,
            frame_id="map",
            anchor_target=anchor_target,
            xy_target=target_horizontal_distance,
            z_min=self.z_min,
            d_target=self.d_target,
            actual_xy_distance=self.current_leader_distance_xy_m,
            actual_distance_3d=self.current_leader_distance_3d_m,
            actual_yaw=current_uav.yaw,
            target_yaw=target_yaw,
            xy_distance_error=self.current_leader_distance_xy_m - target_horizontal_distance,
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
        horizontal_distance, _distance_3d = self._current_follow_geometry()
        target_x, target_y = self._leader_look_target_xy()
        yaw_cmd = solve_yaw_to_target(
            current_uav.x,
            current_uav.y,
            target_x,
            target_y,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )
        _target_horizontal_distance, z_target = self._distance_targets_for_geometry(self.ugv_z)
        z_cmd = self._compute_command_z(
            current_uav_z,
            z_target,
        )
        self.publish_legacy_uav_command(current_uav.x, current_uav.y, z_cmd, yaw_cmd)
        self.publish_pose_cmd(current_uav.x, current_uav.y, z_cmd, yaw_cmd)
        self.publish_pose_cmd_odom(current_uav.x, current_uav.y, z_cmd, yaw_cmd)
        self.uav_cmd = Pose2D(current_uav.x, current_uav.y, yaw_cmd)
        self.have_uav_cmd = True
        self.last_cmd_time = now

    def publish_metrics_cmd(self, leader: Pose2D, cmd_x: float, cmd_y: float) -> None:
        if not self.publish_metrics:
            return
        d_cmd = math.hypot(cmd_x - leader.x, cmd_y - leader.y)
        target_horizontal_distance, _target_z = self._distance_targets_for_geometry(self.ugv_z)
        err = abs(d_cmd - target_horizontal_distance)
        msg1 = Float32()
        msg1.data = float(d_cmd)
        self.metric_dist_pub.publish(msg1)
        msg2 = Float32()
        msg2.data = float(err)
        self.metric_err_pub.publish(msg2)

    def on_tick(self) -> None:
        now = self.get_clock().now()
        if self.start_delay_s > 0.0:
            if self._start_time is None:
                self._start_time = now
            elapsed_s = max(0.0, (now - self._start_time).nanoseconds * 1e-9)
            if elapsed_s < self.start_delay_s:
                if not self._startup_hold_logged:
                    self.get_logger().info(
                        f"[follow_uav_odom] Start delay {self.start_delay_s:.1f}s before UAV follow motion"
                    )
                    self._startup_hold_logged = True
                return
        current_uav = self._control_uav_pose()
        current_uav_z = self._control_uav_z()
        current_horizontal_distance, _current_distance_3d = self._current_follow_geometry()

        if not self.ugv_pose_is_fresh(now):
            return
        if not self.can_send_command_now(now):
            return

        target_horizontal_distance, z_target = self._distance_targets_for_geometry(self.ugv_z)
        anchor_target = self._compute_anchor_target(target_horizontal_distance)
        xt, yt = anchor_target.x, anchor_target.y

        anchor_distance_error, _, _ = self._anchor_errors(anchor_target, current_uav)
        effective_follow_speed_mps = self._effective_follow_speed_mps(anchor_distance_error)
        xt, yt = self.compute_uav_xy_command(
            current_uav,
            xt,
            yt,
            speed_mps=effective_follow_speed_mps,
        )

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

        if yaw_step_limit > 0.0 and abs(yaw_error) > yaw_step_limit:
            yaw_cmd = wrap_pi(current_uav.yaw + math.copysign(yaw_step_limit, yaw_error))
            yaw_mode = "rate_limited"
        else:
            yaw_cmd = yaw_target
            yaw_mode = "direct_target"

        yaw_cmd_delta = wrap_pi(yaw_cmd - current_uav.yaw)

        z_cmd = self._compute_command_z(
            current_uav_z,
            z_target,
        )
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
            target_horizontal_distance=target_horizontal_distance,
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
