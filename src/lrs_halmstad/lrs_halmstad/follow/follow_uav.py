#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from lrs_halmstad.common.ros_params import declare_yaml_param, required_param_value
from lrs_halmstad.follow.follow_core import (
    FollowControllerCoreMixin,
    compute_rate_limited_axis_value,
)
from lrs_halmstad.follow.follow_debug import FollowDebugPublishers
from lrs_halmstad.follow.follow_math import (
    Pose2D,
    clamp_mag,
    clamp_point_to_radius,
    coerce_bool,
    compute_leader_look_target,
    horizontal_distance_for_euclidean,
    solve_yaw_to_target,
    wrap_pi,
    yaw_from_quat,
)


class FollowUav(FollowControllerCoreMixin, Node):
    """Pose/estimate-mode UAV follow controller."""

    def __init__(self):
        super().__init__("follow_uav")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        # ---------- Parameters ----------
        self.declare_parameter("world", "warehouse")
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("leader_input_type", "pose")
        self.declare_parameter("leader_pose_topic", "/coord/leader_estimate")
        declare_yaml_param(self, "leader_actual_heading_enable")
        declare_yaml_param(self, "leader_actual_heading_topic")

        declare_yaml_param(self, "tick_hz")
        declare_yaml_param(self, "d_target", descriptor=dyn_num)
        self.declare_parameter("d_max", 15.0)
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

        # Freshness and service pacing
        declare_yaml_param(self, "pose_timeout_s")   # stale if odom older than this
        declare_yaml_param(self, "min_cmd_period_s") # don't command faster than this even if tick is high

        declare_yaml_param(self, "follow_speed_mps")
        declare_yaml_param(self, "follow_speed_gain")
        declare_yaml_param(self, "uav_xy_command_mode")
        declare_yaml_param(self, "uav_xy_command_step_scale")
        declare_yaml_param(self, "follow_z_speed_mps")
        declare_yaml_param(self, "follow_z_speed_gain")
        declare_yaml_param(self, "search_z_speed_scale")
        declare_yaml_param(self, "search_z_gain_scale")
        declare_yaml_param(self, "search_min_motion_scale")
        declare_yaml_param(self, "search_delay_s")
        declare_yaml_param(self, "follow_yaw_rate_rad_s")
        declare_yaml_param(self, "follow_yaw_rate_gain")
        declare_yaml_param(self, "follow_yaw_accel_rad_s2")
        declare_yaml_param(self, "camera_x_offset_m")
        declare_yaml_param(self, "camera_y_offset_m")
        declare_yaml_param(self, "camera_z_offset_m")
        declare_yaml_param(self, "leader_look_target_x_m")
        declare_yaml_param(self, "leader_look_target_y_m")
        declare_yaml_param(self, "leader_status_topic")
        declare_yaml_param(self, "publish_debug_topics")
        declare_yaml_param(self, "publish_pose_cmd_topics")
        declare_yaml_param(self, "quality_scale_enable")
        declare_yaml_param(self, "quality_conf_min")
        declare_yaml_param(self, "quality_conf_good")
        declare_yaml_param(self, "quality_latency_ref_ms")
        declare_yaml_param(self, "quality_min_motion_scale")
        declare_yaml_param(self, "quality_hold_step_scale")
        declare_yaml_param(self, "state_machine_enable")
        declare_yaml_param(self, "state_debounce_ticks")
        declare_yaml_param(self, "traj_enable")
        declare_yaml_param(self, "traj_rel_frame_enable")
        declare_yaml_param(self, "traj_pos_gain")
        declare_yaml_param(self, "traj_max_speed_mps")
        declare_yaml_param(self, "traj_max_accel_mps2")
        declare_yaml_param(self, "traj_reset_on_yaw_jump_rad")
        # Estimate-mode already receives a tracker-stabilized pose yaw from
        # leader_estimator. Re-deriving heading again here adds noise to the
        # anchor frame, so keep this opt-in.
        declare_yaml_param(self, "estimate_heading_from_motion_enable")
        declare_yaml_param(self, "estimate_heading_min_speed_mps")
        declare_yaml_param(self, "estimate_heading_max_dt_s")

        # ---------- Read params ----------
        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        leader_input_type_raw = str(self.get_parameter("leader_input_type").value).strip().lower()
        self.leader_input_type = leader_input_type_raw
        self.leader_pose_topic = str(self.get_parameter("leader_pose_topic").value)
        self.leader_actual_heading_enable = coerce_bool(
            required_param_value(self, "leader_actual_heading_enable")
        )
        self.leader_actual_heading_topic = str(
            required_param_value(self, "leader_actual_heading_topic")
        )

        if self.leader_input_type == "estimate":
            self.leader_input_type = "pose"
        if self.leader_input_type != "pose":
            raise ValueError(
                "follow_uav only supports 'pose'/'estimate'; "
                f"got leader_input_type={leader_input_type_raw!r}; "
                "use follow_uav_odom for odom mode"
            )

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
        self.search_z_speed_scale = float(required_param_value(self, "search_z_speed_scale"))
        self.search_z_gain_scale = float(required_param_value(self, "search_z_gain_scale"))
        self.search_min_motion_scale = float(required_param_value(self, "search_min_motion_scale"))
        self.search_delay_s = float(required_param_value(self, "search_delay_s"))
        self.follow_yaw_rate_rad_s = float(required_param_value(self, "follow_yaw_rate_rad_s"))
        self.follow_yaw_rate_gain = float(required_param_value(self, "follow_yaw_rate_gain"))
        self.follow_yaw_accel_rad_s2 = float(required_param_value(self, "follow_yaw_accel_rad_s2"))
        self.camera_x_offset_m = float(required_param_value(self, "camera_x_offset_m"))
        self.camera_y_offset_m = float(required_param_value(self, "camera_y_offset_m"))
        self.camera_z_offset_m = float(required_param_value(self, "camera_z_offset_m"))
        self.leader_look_target_x_m = float(required_param_value(self, "leader_look_target_x_m"))
        self.leader_look_target_y_m = float(required_param_value(self, "leader_look_target_y_m"))
        self.leader_status_topic = str(required_param_value(self, "leader_status_topic"))
        self.publish_debug_topics = coerce_bool(required_param_value(self, "publish_debug_topics"))
        self.publish_pose_cmd_topics = coerce_bool(required_param_value(self, "publish_pose_cmd_topics"))
        self.quality_scale_enable = coerce_bool(required_param_value(self, "quality_scale_enable"))
        self.quality_conf_min = float(required_param_value(self, "quality_conf_min"))
        self.quality_conf_good = float(required_param_value(self, "quality_conf_good"))
        self.quality_latency_ref_ms = float(required_param_value(self, "quality_latency_ref_ms"))
        self.quality_min_motion_scale = float(required_param_value(self, "quality_min_motion_scale"))
        self.quality_hold_step_scale = float(required_param_value(self, "quality_hold_step_scale"))
        self.state_machine_enable = coerce_bool(required_param_value(self, "state_machine_enable"))
        self.state_debounce_ticks = int(required_param_value(self, "state_debounce_ticks"))
        self.traj_enable = coerce_bool(required_param_value(self, "traj_enable"))
        self.traj_rel_frame_enable = coerce_bool(required_param_value(self, "traj_rel_frame_enable"))
        self.traj_pos_gain = float(required_param_value(self, "traj_pos_gain"))
        self.traj_max_speed_mps = float(required_param_value(self, "traj_max_speed_mps"))
        self.traj_max_accel_mps2 = float(required_param_value(self, "traj_max_accel_mps2"))
        self.traj_reset_on_yaw_jump_rad = float(required_param_value(self, "traj_reset_on_yaw_jump_rad"))
        self.estimate_heading_from_motion_enable = coerce_bool(required_param_value(self, "estimate_heading_from_motion_enable"))
        self.estimate_heading_min_speed_mps = float(required_param_value(self, "estimate_heading_min_speed_mps"))
        self.estimate_heading_max_dt_s = float(required_param_value(self, "estimate_heading_max_dt_s"))
        self._refresh_xy_target()
        self.current_leader_distance_xy_m = max(0.01, self.xy_target)
        self.current_leader_distance_3d_m = math.hypot(
            self.current_leader_distance_xy_m,
            max(0.0, self.uav_start_z),
        )
        self.ugv_z = 0.0
        self.uav_actual_z = self.uav_start_z
        self.uav_cmd_z = self.uav_start_z
        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.xy_anchor_max <= 0.0:
            raise ValueError("xy_anchor_max must be > 0")
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
        if self.xy_anchor_max <= self.xy_target:
            self.get_logger().warn(
                f"xy_anchor_max ({self.xy_anchor_max}) <= xy_target ({self.xy_target}). "
                f"Leash will trigger/clamp often. Recommend xy_anchor_max > xy_target."
            )
        if self.follow_speed_mps < 0.0:
            raise ValueError("follow_speed_mps must be >= 0")
        if self.follow_speed_gain < 0.0:
            raise ValueError("follow_speed_gain must be >= 0")
        if self.uav_xy_command_mode not in ("direct", "controller_step"):
            raise ValueError("uav_xy_command_mode must be 'direct' or 'controller_step'")
        if self.uav_xy_command_step_scale < 0.0:
            raise ValueError("uav_xy_command_step_scale must be >= 0")
        if self.follow_z_speed_mps < 0.0:
            raise ValueError("follow_z_speed_mps must be >= 0")
        if self.follow_z_speed_gain < 0.0:
            raise ValueError("follow_z_speed_gain must be >= 0")
        if self.search_z_speed_scale < 0.0:
            raise ValueError("search_z_speed_scale must be >= 0")
        if self.search_z_gain_scale < 0.0:
            raise ValueError("search_z_gain_scale must be >= 0")
        if not (0.0 <= self.search_min_motion_scale <= 1.0):
            raise ValueError("search_min_motion_scale must be in [0,1]")
        if self.search_delay_s < 0.0:
            raise ValueError("search_delay_s must be >= 0")
        if self.follow_yaw_rate_rad_s < 0.0:
            raise ValueError("follow_yaw_rate_rad_s must be >= 0")
        if self.follow_yaw_rate_gain < 0.0:
            raise ValueError("follow_yaw_rate_gain must be >= 0")
        if self.follow_yaw_accel_rad_s2 < 0.0:
            raise ValueError("follow_yaw_accel_rad_s2 must be >= 0")
        if self.quality_conf_good <= self.quality_conf_min:
            raise ValueError("quality_conf_good must be > quality_conf_min")
        if self.quality_latency_ref_ms <= 0.0:
            raise ValueError("quality_latency_ref_ms must be > 0")
        if not (0.0 <= self.quality_min_motion_scale <= 1.0):
            raise ValueError("quality_min_motion_scale must be in [0,1]")
        if not (0.0 <= self.quality_hold_step_scale <= 1.0):
            raise ValueError("quality_hold_step_scale must be in [0,1]")
        if self.state_debounce_ticks < 1:
            raise ValueError("state_debounce_ticks must be >= 1")
        if self.traj_pos_gain < 0.0:
            raise ValueError("traj_pos_gain must be >= 0")
        if self.traj_max_speed_mps < 0.0:
            raise ValueError("traj_max_speed_mps must be >= 0")
        if self.traj_max_accel_mps2 < 0.0:
            raise ValueError("traj_max_accel_mps2 must be >= 0")
        if self.traj_reset_on_yaw_jump_rad < 0.0:
            raise ValueError("traj_reset_on_yaw_jump_rad must be >= 0")
        if self.estimate_heading_min_speed_mps < 0.0:
            raise ValueError("estimate_heading_min_speed_mps must be >= 0")
        if self.estimate_heading_max_dt_s <= 0.0:
            raise ValueError("estimate_heading_max_dt_s must be > 0")

        # ---------- State ----------
        self.have_ugv = False
        self.have_seen_ugv_pose = False
        self.ugv_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_ugv_stamp: Optional[Time] = None
        self.last_actual_heading_yaw: Optional[float] = None
        self.last_actual_heading_stamp: Optional[Time] = None

        # Commanded UAV pose (deterministic internal state)
        self.uav_cmd = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.have_uav_cmd = bool(self.seed_uav_cmd_on_start)
        self.uav_actual = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.have_uav_actual = False
        self.last_uav_actual_time: Optional[Time] = None
        self.last_debug_actual_yaw_unwrapped: Optional[float] = None

        self.last_cmd_time: Optional[Time] = None
        self.last_leader_status_fields = {}
        self.last_leader_status_rx: Optional[Time] = None
        self.follow_state = "INIT"
        self._desired_follow_state = "INIT"
        self._state_good_ticks = 0
        self._state_bad_ticks = 0
        self.search_bad_state_since: Optional[Time] = None
        self.search_bad_state_name = ""
        self.traj_rel_vx = 0.0
        self.traj_rel_vy = 0.0
        self.traj_last_leader_yaw: Optional[float] = None
        self.yaw_rate_cmd_rad_s = 0.0
        self.leader_motion_prev_xy: Optional[Tuple[float, float]] = None
        self.leader_motion_prev_stamp: Optional[Time] = None
        self.leader_motion_vx = 0.0
        self.leader_motion_vy = 0.0
        self.leader_motion_speed_mps = 0.0
        self.leader_motion_heading_yaw: Optional[float] = None

        # ---------- ROS I/O ----------
        self.leader_sub = self.create_subscription(
            PoseStamped,
            self.leader_pose_topic,
            self.on_leader_pose,
            1,
        )
        leader_desc = f"pose:{self.leader_pose_topic}"
        self.leader_actual_heading_sub = None
        if self.leader_actual_heading_enable:
            self.leader_actual_heading_sub = self.create_subscription(
                Odometry,
                self.leader_actual_heading_topic,
                self.on_leader_actual_heading_odom,
                10,
            )
            leader_desc += f", actual_heading:{self.leader_actual_heading_topic}"
        self.leader_status_sub = self.create_subscription(
            String,
            self.leader_status_topic,
            self.on_leader_status,
            1,
        )

        self.pose_pub = (
            self.create_publisher(
                PoseStamped,
                f"/{self.uav_name}/pose_cmd",
                10,
            )
            if self.publish_pose_cmd_topics
            else None
        )
        self.debug_pubs = (
            FollowDebugPublishers(self, self.uav_name)
            if self.publish_debug_topics
            else None
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
        self.pose_odom_pub = (
            self.create_publisher(
                Odometry,
                f"/{self.uav_name}/pose_cmd/odom",
                10,
            )
            if self.publish_pose_cmd_topics
            else None
        )

        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"[follow_uav] Started: world={self.world}, uav={self.uav_name}, "
            f"leader_input={leader_desc}, tick={self.tick_hz}Hz, "
            f"d_target={self.d_target}, xy_target={self.xy_target}, xy_min={self.xy_min}, "
            f"xy_anchor_max={self.xy_anchor_max}, z_min={self.z_min}, z_max={self.z_max}, xy_max={self.xy_max}, "
            f"pose_timeout_s={self.pose_timeout_s}, min_cmd_period_s={self.min_cmd_period_s}, "
            f"follow_speed_mps={self.follow_speed_mps}, "
            f"follow_speed_gain={self.follow_speed_gain}, "
            f"uav_xy_command_mode={self.uav_xy_command_mode}, "
            f"uav_xy_command_step_scale={self.uav_xy_command_step_scale}, "
            f"follow_z_speed_mps={self.follow_z_speed_mps}, "
            f"follow_z_speed_gain={self.follow_z_speed_gain}, "
            f"search_z_speed_scale={self.search_z_speed_scale}, "
            f"search_z_gain_scale={self.search_z_gain_scale}, "
            f"search_min_motion_scale={self.search_min_motion_scale}, "
            f"search_delay_s={self.search_delay_s}, "
            f"seed_uav_cmd_on_start={self.seed_uav_cmd_on_start}, "
            f"uav_start=({self.uav_start_x:.2f},{self.uav_start_y:.2f},{self.uav_start_z:.2f},{math.degrees(self.uav_start_yaw):.1f}deg), "
            f"follow_yaw_rate_rad_s={self.follow_yaw_rate_rad_s}, "
            f"follow_yaw_rate_gain={self.follow_yaw_rate_gain}, "
            f"follow_yaw_accel_rad_s2={self.follow_yaw_accel_rad_s2}, "
            f"camera_offset_m=({self.camera_x_offset_m}, {self.camera_y_offset_m}, {self.camera_z_offset_m}), "
            f"leader_look_target_xy_m=({self.leader_look_target_x_m}, {self.leader_look_target_y_m}), "
            f"quality_scale_enable={self.quality_scale_enable}, leader_status_topic={self.leader_status_topic}, "
            f"publish_debug_topics={self.publish_debug_topics}, publish_pose_cmd_topics={self.publish_pose_cmd_topics}, "
            f"status_timeout_s={self.pose_timeout_s}, quality_conf=[{self.quality_conf_min},{self.quality_conf_good}], "
            f"quality_latency_ref_ms={self.quality_latency_ref_ms}, quality_min_motion_scale={self.quality_min_motion_scale}, "
            f"quality_hold_step_scale={self.quality_hold_step_scale}, "
            f"state_machine_enable={self.state_machine_enable}, state_debounce_ticks={self.state_debounce_ticks}, "
            f"traj_enable={self.traj_enable}, traj_rel_frame_enable={self.traj_rel_frame_enable}, "
            f"traj_pos_gain={self.traj_pos_gain}, "
            f"traj_max_speed_mps={self.traj_max_speed_mps}, traj_max_accel_mps2={self.traj_max_accel_mps2}, "
            f"traj_reset_on_yaw_jump_rad={self.traj_reset_on_yaw_jump_rad}, "
            f"estimate_heading_from_motion_enable={self.estimate_heading_from_motion_enable}, "
            f"leader_actual_heading_enable={self.leader_actual_heading_enable}, "
            f"leader_actual_heading_topic={self.leader_actual_heading_topic}, "
            f"estimate_heading_min_speed_mps={self.estimate_heading_min_speed_mps}, "
            f"estimate_heading_max_dt_s={self.estimate_heading_max_dt_s}, "
            f"uav_cmd_topic=/{self.uav_name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
        )

    def _on_set_parameters(self, params):
        updates = {}
        for param in params:
            if param.name not in ("d_euclidean", "d_target", "xy_min", "z_min", "z_max", "xy_max"):
                continue
            try:
                value = float(param.value)
            except Exception as exc:
                return SetParametersResult(successful=False, reason=f"invalid {param.name}: {exc}")
            if param.name in ("xy_min", "z_min", "z_max", "xy_max"):
                if value < 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be >= 0")
            elif value <= 0.0:
                return SetParametersResult(successful=False, reason=f"{param.name} must be > 0")
            updates[param.name] = value

        if not updates:
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

        update_parts = []
        if updates:
            update_parts.append(
                f"d_target={self.d_target:.2f}, xy_target={self.xy_target:.2f}, xy_min={self.xy_min:.2f}, "
                f"z_min={self.z_min:.2f}, z_max={self.z_max:.2f}, xy_max={self.xy_max:.2f}"
            )
        if update_parts:
            self.get_logger().info("[follow_uav] Runtime parameter update: " + ", ".join(update_parts))
        return SetParametersResult(successful=True)

    def on_leader_status(self, msg: String):
        fields = {}
        for tok in msg.data.split():
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            fields[k.strip()] = v.strip()
        self.last_leader_status_fields = fields
        self.last_leader_status_rx = self.get_clock().now()

    def _status_float(self, key: str, default: float = -1.0) -> float:
        raw = self.last_leader_status_fields.get(key)
        if raw is None:
            return default
        if raw == "inf":
            return float("inf")
        try:
            return float(raw)
        except Exception:
            return default

    def _quality_scale_from_status(self, now: Time) -> Tuple[float, str]:
        if not self.quality_scale_enable:
            return 1.0, "disabled"
        if self.last_leader_status_rx is None:
            return 1.0, "no_status"
        if self.pose_timeout_s > 0.0:
            age_s = (now - self.last_leader_status_rx).nanoseconds * 1e-9
            if age_s > self.pose_timeout_s:
                return 0.0, "status_stale"

        state = self.last_leader_status_fields.get("state", "")
        conf = self._status_float("conf", -1.0)
        latency_ms = self._status_float("latency_ms", -1.0)

        hard_bad_states = {
            "STALE", "DECODE_FAIL", "NO_DET", "YOLO_DISABLED", "REJECT", "waiting_for_image",
            "waiting_for_camera_info", "waiting_for_uav_pose", "stale_image"
        }
        soft_states = {"HOLD", "DEBOUNCE_HOLD", "REJECT_HOLD", "REJECT_DEBOUNCE_HOLD", "REACQUIRE"}

        if state in hard_bad_states:
            return 0.0, f"state:{state}"

        if conf < 0.0:
            q_conf = 0.6 if state in soft_states else 1.0
        else:
            q_conf = (conf - self.quality_conf_min) / max(1e-6, (self.quality_conf_good - self.quality_conf_min))
            q_conf = max(0.0, min(1.0, q_conf))

        if latency_ms < 0.0 or not math.isfinite(latency_ms):
            q_lat = 1.0
        else:
            q_lat = 1.0 / (1.0 + max(0.0, latency_ms) / self.quality_latency_ref_ms)

        q_state = 0.45 if state in soft_states else 1.0
        q = q_conf * q_lat * q_state
        if q > 0.0:
            q = max(self.quality_min_motion_scale, q)
        q = max(0.0, min(1.0, q))
        return q, (state if state else "ok")

    def _leader_status_state(self) -> str:
        return str(self.last_leader_status_fields.get("state", "")).strip()

    def _freeze_yaw_for_status(self) -> bool:
        return self._leader_status_state() in {"REJECT_HOLD", "REJECT_DEBOUNCE_HOLD"}

    def _should_search_for_target(self, now: Time, current_horizontal_distance: float) -> bool:
        if not self.have_ugv or self.last_ugv_stamp is None:
            self.search_bad_state_since = None
            self.search_bad_state_name = ""
            return False
        st = self._leader_status_state()
        bad_states = {
            "STALE",
            "DECODE_FAIL",
            "NO_DET",
            "YOLO_DISABLED",
            "waiting_for_image",
            "waiting_for_camera_info",
            "waiting_for_uav_pose",
            "stale_image",
        }
        if st not in bad_states:
            self.search_bad_state_since = None
            self.search_bad_state_name = ""
            return False
        if self.search_bad_state_name != st or self.search_bad_state_since is None:
            self.search_bad_state_name = st
            self.search_bad_state_since = now
            return False
        if self.search_delay_s > 0.0:
            bad_age_s = (now - self.search_bad_state_since).nanoseconds * 1e-9
            if bad_age_s < self.search_delay_s:
                return False
        age_s = (now - self.last_ugv_stamp).nanoseconds * 1e-9
        if age_s > self.pose_timeout_s:
            return False
        search_xy_trigger_m = max(2.0, self.xy_min + 1.0)
        return current_horizontal_distance <= search_xy_trigger_m

    def _set_follow_state(self, new_state: str) -> None:
        if new_state == self.follow_state:
            return
        self.follow_state = new_state

    def _update_follow_state(self, desired_state: str) -> None:
        if not self.state_machine_enable:
            self._set_follow_state(desired_state)
            return

        if desired_state == self._desired_follow_state:
            if desired_state in ("TRACK", "REACQUIRE"):
                self._state_good_ticks += 1
                self._state_bad_ticks = 0
                if self._state_good_ticks >= self.state_debounce_ticks:
                    self._set_follow_state(desired_state)
            else:
                self._state_bad_ticks += 1
                self._state_good_ticks = 0
                if self._state_bad_ticks >= self.state_debounce_ticks:
                    self._set_follow_state(desired_state)
        else:
            self._desired_follow_state = desired_state
            if desired_state in ("TRACK", "REACQUIRE"):
                self._state_good_ticks = 1
                self._state_bad_ticks = 0
            else:
                self._state_bad_ticks = 1
                self._state_good_ticks = 0

    def _classify_follow_state(self, quality_scale: float) -> str:
        if not self.quality_scale_enable:
            return "TRACK"

        st = self._leader_status_state()
        hard_bad_states = {
            "STALE", "DECODE_FAIL", "NO_DET", "YOLO_DISABLED",
            "waiting_for_image", "waiting_for_camera_info", "waiting_for_uav_pose", "stale_image"
        }
        soft_states = {"HOLD", "DEBOUNCE_HOLD", "REJECT_HOLD", "REJECT_DEBOUNCE_HOLD", "REACQUIRE", "REJECT"}

        if quality_scale <= self.quality_hold_step_scale:
            return "HOLD"
        if st in hard_bad_states:
            return "DEGRADED"
        if st in soft_states:
            return "REACQUIRE"
        return "TRACK"

    def _to_leader_frame(self, leader: Pose2D, x_world: float, y_world: float) -> Tuple[float, float]:
        dx = x_world - leader.x
        dy = y_world - leader.y
        c = math.cos(leader.yaw)
        s = math.sin(leader.yaw)
        # Rotate world offset into leader frame.
        return (c * dx + s * dy, -s * dx + c * dy)

    def _from_leader_frame(self, leader: Pose2D, x_rel: float, y_rel: float) -> Tuple[float, float]:
        c = math.cos(leader.yaw)
        s = math.sin(leader.yaw)
        return (
            leader.x + c * x_rel - s * y_rel,
            leader.y + s * x_rel + c * y_rel,
        )

    def _reset_traj_state(self) -> None:
        self.traj_rel_vx = 0.0
        self.traj_rel_vy = 0.0
        self.traj_last_leader_yaw = None

    def _reset_yaw_state(self) -> None:
        self.yaw_rate_cmd_rad_s = 0.0

    def _control_dt_s(self, now: Time) -> float:
        dt = 1.0 / self.tick_hz
        if self.last_cmd_time is not None:
            dt = max(1e-3, (now - self.last_cmd_time).nanoseconds * 1e-9)
        return min(dt, 1.0)

    def _update_leader_motion_model(self, pose: Pose2D, stamp: Optional[Time]) -> None:
        if stamp is None:
            return
        if self.leader_motion_prev_xy is None or self.leader_motion_prev_stamp is None:
            self.leader_motion_prev_xy = (pose.x, pose.y)
            self.leader_motion_prev_stamp = stamp
            if self.leader_motion_heading_yaw is None:
                self.leader_motion_heading_yaw = pose.yaw
            return

        dt = (stamp - self.leader_motion_prev_stamp).nanoseconds * 1e-9
        if not math.isfinite(dt) or dt <= 1e-6:
            self.leader_motion_prev_xy = (pose.x, pose.y)
            self.leader_motion_prev_stamp = stamp
            return
        # Refresh motion heading on a much coarser interval so anchor direction
        # is based on a meaningful displacement, not frame-to-frame jitter.
        if dt < 1.0:
            return
        if dt > self.estimate_heading_max_dt_s:
            self.leader_motion_prev_xy = (pose.x, pose.y)
            self.leader_motion_prev_stamp = stamp
            return

        dx = pose.x - self.leader_motion_prev_xy[0]
        dy = pose.y - self.leader_motion_prev_xy[1]
        rvx = dx / dt
        rvy = dy / dt
        self.leader_motion_vx = rvx
        self.leader_motion_vy = rvy
        self.leader_motion_speed_mps = math.hypot(self.leader_motion_vx, self.leader_motion_vy)
        if self.leader_motion_speed_mps >= self.estimate_heading_min_speed_mps:
            raw_yaw = math.atan2(self.leader_motion_vy, self.leader_motion_vx)
            if self.leader_motion_heading_yaw is not None:
                # Motion direction reverses by pi when the UGV backs up; the
                # follow anchor should stay in the same leader-frame direction.
                if math.cos(raw_yaw - self.leader_motion_heading_yaw) < 0.0:
                    raw_yaw = wrap_pi(raw_yaw + math.pi)
            self.leader_motion_heading_yaw = raw_yaw

        self.leader_motion_prev_xy = (pose.x, pose.y)
        self.leader_motion_prev_stamp = stamp

    def _leader_pose_for_follow(self) -> Pose2D:
        yaw, _source, _estimate_yaw, _actual_yaw = self._leader_heading_details_for_follow()
        return Pose2D(self.ugv_pose.x, self.ugv_pose.y, yaw)

    def _leader_estimate_heading_source(self) -> str:
        return str(self.last_leader_status_fields.get("heading_src", "")).strip().lower()

    def _leader_heading_details_for_follow(self) -> Tuple[float, str, Optional[float], Optional[float]]:
        estimate_yaw = self.ugv_pose.yaw
        actual_yaw = None
        if self.leader_actual_heading_enable and self.leader_actual_heading_is_fresh():
            actual_yaw = self.last_actual_heading_yaw
            return actual_yaw, "actual_heading", estimate_yaw, actual_yaw
        estimate_heading_source = self._leader_estimate_heading_source()
        motion_heading_ok = self.leader_motion_heading_yaw is not None
        if motion_heading_ok and self.estimate_heading_from_motion_enable:
            return self.leader_motion_heading_yaw, "motion_heading", estimate_yaw, actual_yaw
        if estimate_heading_source not in {"", "none", "fallback"}:
            return estimate_yaw, f"estimate_{estimate_heading_source}", estimate_yaw, actual_yaw
        if estimate_heading_source == "fallback":
            return estimate_yaw, "estimate_fallback_pose_yaw", estimate_yaw, actual_yaw
        return estimate_yaw, "estimate_pose_yaw", estimate_yaw, actual_yaw

    def _shape_target_trajectory(
        self,
        leader: Pose2D,
        xt: float,
        yt: float,
        now: Time,
        quality_scale: float,
        *,
        search_active: bool = False,
    ) -> Tuple[float, float]:
        if not self.traj_enable or self.leader_input_type != "pose":
            return xt, yt
        if self.traj_max_speed_mps <= 0.0 or self.traj_pos_gain <= 0.0:
            return xt, yt
        current_uav = self._current_uav_pose()
        if not (self.have_uav_actual or self.have_uav_cmd):
            self._reset_traj_state()
            return xt, yt

        dt = self._control_dt_s(now)

        if self.traj_last_leader_yaw is not None and self.traj_reset_on_yaw_jump_rad > 0.0:
            dyaw_leader = abs(wrap_pi(leader.yaw - self.traj_last_leader_yaw))
            if dyaw_leader > self.traj_reset_on_yaw_jump_rad:
                self._reset_traj_state()

        if self.traj_rel_frame_enable:
            cur_rel_x, cur_rel_y = self._to_leader_frame(leader, current_uav.x, current_uav.y)
            des_rel_x, des_rel_y = self._to_leader_frame(leader, xt, yt)
        else:
            cur_rel_x, cur_rel_y = current_uav.x, current_uav.y
            des_rel_x, des_rel_y = xt, yt

        ex = des_rel_x - cur_rel_x
        ey = des_rel_y - cur_rel_y
        err_mag = math.hypot(ex, ey)

        q_scale = 1.0
        if self.quality_scale_enable:
            q_scale = max(self.quality_min_motion_scale, quality_scale)
        if search_active:
            q_scale = max(q_scale, self.search_min_motion_scale)

        vmax = self.traj_max_speed_mps * q_scale
        amax = self.traj_max_accel_mps2 * q_scale

        vdx = self.traj_pos_gain * ex
        vdy = self.traj_pos_gain * ey
        vdx, vdy = clamp_mag(vdx, vdy, vmax)

        if amax > 0.0:
            dvx = vdx - self.traj_rel_vx
            dvy = vdy - self.traj_rel_vy
            dvx, dvy = clamp_mag(dvx, dvy, amax * dt)
            self.traj_rel_vx += dvx
            self.traj_rel_vy += dvy
        else:
            self.traj_rel_vx, self.traj_rel_vy = vdx, vdy

        step_x = self.traj_rel_vx * dt
        step_y = self.traj_rel_vy * dt

        # Prevent overshoot relative to the desired target.
        if (ex * (ex - step_x) + ey * (ey - step_y)) < 0.0:
            nxt_x, nxt_y = des_rel_x, des_rel_y
            self.traj_rel_vx = 0.0
            self.traj_rel_vy = 0.0
        else:
            nxt_x = cur_rel_x + step_x
            nxt_y = cur_rel_y + step_y

        self.traj_last_leader_yaw = leader.yaw
        if self.traj_rel_frame_enable:
            return self._from_leader_frame(leader, nxt_x, nxt_y)
        return nxt_x, nxt_y

    def on_leader_pose(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))

        self.ugv_pose = Pose2D(float(p.x), float(p.y), float(yaw))
        self.ugv_z = float(p.z)
        self.have_ugv = True
        self.have_seen_ugv_pose = True

        try:
            self.last_ugv_stamp = Time.from_msg(msg.header.stamp)
        except (ValueError, TypeError, AttributeError):
            self.last_ugv_stamp = self.get_clock().now()
        self._update_leader_motion_model(self.ugv_pose, self.last_ugv_stamp)

    def on_leader_actual_heading_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.last_actual_heading_yaw = yaw_from_quat(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        try:
            self.last_actual_heading_stamp = Time.from_msg(msg.header.stamp)
        except (ValueError, TypeError, AttributeError):
            self.last_actual_heading_stamp = self.get_clock().now()

    def leader_actual_heading_is_fresh(self) -> bool:
        if self.last_actual_heading_yaw is None or self.last_actual_heading_stamp is None:
            return False
        now = self.get_clock().now()
        age = (now - self.last_actual_heading_stamp).nanoseconds * 1e-9
        return age <= self.pose_timeout_s

    def _leader_look_target_xy(self, leader_for_follow: Pose2D) -> Tuple[float, float]:
        target_x, target_y, _target_z = compute_leader_look_target(
            leader_for_follow.x,
            leader_for_follow.y,
            leader_for_follow.yaw,
            self.ugv_z,
            self.leader_look_target_x_m,
            self.leader_look_target_y_m,
            0.0,
        )
        return target_x, target_y

    def _search_target_pair(self, leader_z: float) -> Tuple[float, float]:
        target_z = self._effective_z_cap(leader_z)
        target_xy = self._bounded_xy_target(
            horizontal_distance_for_euclidean(self.d_target, target_z - leader_z),
            leader_z,
        )
        return target_xy, target_z

    def _distance_targets_for_geometry(
        self,
        leader_z: float,
        *,
        search_active: bool = False,
    ) -> Tuple[float, float]:
        if search_active:
            return self._search_target_pair(leader_z)
        return self._nominal_target_pair(leader_z)

    def _target_xy_for_z(self, leader_z: float, uav_z: float) -> float:
        return self._bounded_xy_target(
            horizontal_distance_for_euclidean(self.d_target, uav_z - leader_z),
            leader_z,
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
        if self.have_ugv:
            leader_follow_yaw, leader_heading_source, leader_estimate_yaw, leader_actual_heading_yaw = (
                self._leader_heading_details_for_follow()
            )
            leader_for_follow = Pose2D(self.ugv_pose.x, self.ugv_pose.y, leader_follow_yaw)
        else:
            leader_heading_source = "startup_seed"
            leader_follow_yaw = current_uav.yaw
            leader_estimate_yaw = None
            leader_actual_heading_yaw = None
            nominal_horizontal_target = self._nominal_horizontal_follow_distance()
            leader_for_follow = Pose2D(
                self.uav_start_x + nominal_horizontal_target,
                self.uav_start_y,
                current_uav.yaw,
            )
        anchor_distance_error, anchor_along_error, anchor_cross_error = self._anchor_errors(
            leader_for_follow, anchor_target, current_uav
        )
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
            leader_heading_source=leader_heading_source,
            leader_follow_yaw=leader_follow_yaw,
            leader_estimate_yaw=leader_estimate_yaw,
            leader_actual_heading_yaw=leader_actual_heading_yaw,
        )

    def _anchor_errors(
        self, leader_for_follow: Pose2D, anchor_target: Pose2D, current_uav: Pose2D
    ) -> Tuple[float, float, float]:
        actual_rel_x, actual_rel_y = self._to_leader_frame(
            leader_for_follow, current_uav.x, current_uav.y
        )
        target_rel_x, target_rel_y = self._to_leader_frame(
            leader_for_follow, anchor_target.x, anchor_target.y
        )
        anchor_along_error = actual_rel_x - target_rel_x
        anchor_cross_error = actual_rel_y - target_rel_y
        anchor_distance_error = math.hypot(
            current_uav.x - anchor_target.x,
            current_uav.y - anchor_target.y,
        )
        return anchor_distance_error, anchor_along_error, anchor_cross_error

    def _effective_follow_speed_mps(self, anchor_distance_error: float) -> float:
        return min(
            self.follow_speed_mps,
            self.follow_speed_gain * max(anchor_distance_error, 0.0),
        )

    def _compute_command_z(
        self,
        current_z: float,
        target_z: float,
        speed_scale: float = 1.0,
        gain_scale: float = 1.0,
    ) -> float:
        return compute_rate_limited_axis_value(
            current_z,
            target_z,
            tick_hz=self.tick_hz,
            max_speed_per_s=self.follow_z_speed_mps * max(0.0, speed_scale),
            gain=self.follow_z_speed_gain * max(0.0, gain_scale),
        )

    def _current_follow_geometry(self) -> Tuple[float, float]:
        if self.have_ugv:
            leader_x = self.ugv_pose.x
            leader_y = self.ugv_pose.y
            leader_z = self.ugv_z
        else:
            leader_x = self.uav_start_x + self._nominal_horizontal_follow_distance()
            leader_y = self.uav_start_y
            leader_z = 0.0

        current_uav = self._current_uav_pose()
        uav_x = current_uav.x
        uav_y = current_uav.y
        uav_z = self._current_uav_z()

        # Body-motion geometry is defined from the UAV body pose to the leader pose.
        # Camera-relative geometry stays in camera_tracker.py.
        horizontal_distance = math.hypot(leader_x - uav_x, leader_y - uav_y)
        if horizontal_distance < 1e-3:
            horizontal_distance = 1e-3
        distance_3d = math.hypot(horizontal_distance, uav_z - leader_z)

        self.current_leader_distance_xy_m = horizontal_distance
        self.current_leader_distance_3d_m = distance_3d
        return horizontal_distance, distance_3d

    def _compute_anchor_target(self, target_horizontal_distance: float) -> Pose2D:
        leader_for_follow = self._leader_pose_for_follow()
        xt = leader_for_follow.x - target_horizontal_distance * math.cos(leader_for_follow.yaw)
        yt = leader_for_follow.y - target_horizontal_distance * math.sin(leader_for_follow.yaw)

        xt, yt = clamp_point_to_radius(
            leader_for_follow.x,
            leader_for_follow.y,
            xt,
            yt,
            self.xy_anchor_max,
        )

        if self.follow_yaw:
            target_x, target_y = self._leader_look_target_xy(leader_for_follow)
            yaw_cmd = solve_yaw_to_target(
                xt,
                yt,
                target_x,
                target_y,
                self.camera_x_offset_m,
                self.camera_y_offset_m,
            )
        else:
            yaw_cmd = self._current_uav_pose().yaw

        return Pose2D(xt, yt, yaw_cmd)

    def on_tick(self):
        now = self.get_clock().now()
        current_uav = self._control_uav_pose()
        current_uav_z = self._control_uav_z()
        control_dt = self._control_dt_s(now)
        current_horizontal_distance, _current_distance_3d = self._current_follow_geometry()
        search_active = self._should_search_for_target(now, current_horizontal_distance)

        if not self.ugv_pose_is_fresh(now):
            self._reset_yaw_state()
            self._update_follow_state("HOLD")
            return

        if not self.can_send_command_now(now):
            return

        quality_scale, _quality_reason = self._quality_scale_from_status(now)
        desired_follow_state = self._classify_follow_state(quality_scale)
        self._update_follow_state(desired_follow_state)

        target_horizontal_distance, z_target = self._distance_targets_for_geometry(
            self.ugv_z,
            search_active=search_active,
        )
        if (self.follow_state == "HOLD" or self._freeze_yaw_for_status()) and not search_active:
            z_cmd = current_uav_z
        else:
            search_climbing = search_active and z_target > current_uav_z + 0.01
            z_cmd = self._compute_command_z(
                current_uav_z,
                z_target,
                speed_scale=self.search_z_speed_scale if search_climbing else 1.0,
                gain_scale=self.search_z_gain_scale if search_climbing else 1.0,
            )
        target_horizontal_distance = self._target_xy_for_z(self.ugv_z, z_cmd)
        leader_for_follow = self._leader_pose_for_follow()
        anchor_target = self._compute_anchor_target(target_horizontal_distance)
        xt = anchor_target.x
        yt = anchor_target.y

        if self.follow_state == "HOLD" and not search_active:
            self._reset_traj_state()
            self._reset_yaw_state()
            xt = current_uav.x
            yt = current_uav.y
        if self.follow_state != "HOLD" or search_active:
            xt, yt = self._shape_target_trajectory(
                leader_for_follow,
                xt,
                yt,
                now,
                quality_scale,
                search_active=search_active,
            )

        anchor_distance_error, _, _ = self._anchor_errors(leader_for_follow, anchor_target, current_uav)
        effective_follow_speed_mps = self._effective_follow_speed_mps(anchor_distance_error)
        if self.quality_scale_enable:
            motion_quality_scale = quality_scale
            if search_active:
                motion_quality_scale = max(motion_quality_scale, self.search_min_motion_scale)
            effective_follow_speed_mps *= max(0.0, min(1.0, motion_quality_scale))
        xt, yt = self.compute_uav_xy_command(
            current_uav,
            xt,
            yt,
            speed_mps=effective_follow_speed_mps,
        )

        cmd_xy_delta = math.hypot(xt - current_uav.x, yt - current_uav.y)
        target_x, target_y = self._leader_look_target_xy(leader_for_follow)
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
        yaw_error_mag = abs(yaw_error)
        yaw_rate_target = min(
            max(self.follow_yaw_rate_rad_s, 0.0),
            max(self.follow_yaw_rate_gain, 0.0) * yaw_error_mag,
        )
        if self.quality_scale_enable:
            yaw_rate_target *= max(0.0, min(1.0, quality_scale))
        yaw_rate_target = math.copysign(yaw_rate_target, yaw_error)
        yaw_accel_rad_s2 = self.follow_yaw_accel_rad_s2
        if yaw_accel_rad_s2 > 0.0:
            max_rate_delta = yaw_accel_rad_s2 * control_dt
            rate_delta = yaw_rate_target - self.yaw_rate_cmd_rad_s
            if rate_delta > max_rate_delta:
                rate_delta = max_rate_delta
            elif rate_delta < -max_rate_delta:
                rate_delta = -max_rate_delta
            self.yaw_rate_cmd_rad_s += rate_delta
        else:
            self.yaw_rate_cmd_rad_s = yaw_rate_target
        yaw_step_limit = abs(self.yaw_rate_cmd_rad_s) * control_dt if abs(self.yaw_rate_cmd_rad_s) > 0.0 else 0.0

        if search_active:
            self._reset_yaw_state()
            yaw_cmd = current_uav.yaw
            yaw_mode = "search_hold"
        elif self.follow_state == "HOLD":
            self._reset_yaw_state()
            yaw_cmd = current_uav.yaw
            yaw_mode = "hold_state"
        elif self._freeze_yaw_for_status():
            self._reset_yaw_state()
            yaw_cmd = current_uav.yaw
            yaw_mode = "status_hold"
        elif not self.follow_yaw:
            self._reset_yaw_state()
            yaw_cmd = current_uav.yaw
            yaw_mode = "follow_yaw_disabled"
        elif yaw_step_limit > 0.0 and abs(yaw_error) > yaw_step_limit:
            yaw_cmd = wrap_pi(current_uav.yaw + math.copysign(yaw_step_limit, yaw_error))
            yaw_mode = "accel_rate_limited"
        else:
            self._reset_yaw_state()
            yaw_cmd = yaw_target
            yaw_mode = "direct_target"

        yaw_cmd_delta = wrap_pi(yaw_cmd - current_uav.yaw)

        self.publish_legacy_uav_command(xt, yt, z_cmd, yaw_cmd)
        self.publish_pose_cmd(xt, yt, z_cmd, yaw_cmd)
        self.publish_pose_cmd_odom(xt, yt, z_cmd, yaw_cmd)

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
    node = FollowUav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
