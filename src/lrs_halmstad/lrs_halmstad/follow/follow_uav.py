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

from lrs_halmstad.follow.follow_debug import FollowDebugPublishers
from lrs_halmstad.follow.follow_math import (
    Pose2D,
    camera_xy_from_uav_pose,
    clamp_mag,
    clamp_point_to_radius,
    coerce_bool,
    compute_leader_look_target,
    quat_from_yaw,
    solve_yaw_to_target,
    wrap_pi,
    yaw_from_quat,
)


class FollowUav(Node):
    """Pose/estimate-mode UAV follow controller."""

    def __init__(self):
        super().__init__("follow_uav")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        # ---------- Parameters ----------
        self.declare_parameter("world", "warehouse")
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("leader_input_type", "pose")
        self.declare_parameter("leader_pose_topic", "/coord/leader_estimate")
        self.declare_parameter("leader_actual_heading_enable", True)
        self.declare_parameter("leader_actual_heading_topic", "/a201_0000/amcl_pose_odom")

        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("d_target", 7.0, dyn_num)
        self.declare_parameter("d_max", 15.0)
        self.declare_parameter("z_alt", 7.0, dyn_num)
        self.declare_parameter("d_euclidean", 0.0, dyn_num)
        self.declare_parameter("manual_override_enable", False)
        self.declare_parameter("seed_uav_cmd_on_start", True)
        self.declare_parameter("startup_reposition_enable", False)
        self.declare_parameter("startup_reposition_tol_m", 0.25)
        self.declare_parameter("uav_start_x", -2.0)
        self.declare_parameter("uav_start_y", 0.0)
        self.declare_parameter("uav_start_yaw_deg", 0.0)

        self.declare_parameter("follow_yaw", True)

        # Freshness and service pacing
        self.declare_parameter("pose_timeout_s", 0.75)   # stale if odom older than this
        self.declare_parameter("min_cmd_period_s", 0.10) # don't command faster than this even if tick is high

        # Smoothing (alpha=1.0 -> no smoothing; 0.0 -> freeze)
        self.declare_parameter("smooth_alpha", 1.0)
        self.declare_parameter("follow_speed_mps", 0.5)
        self.declare_parameter("follow_speed_gain", 2.0)
        self.declare_parameter("follow_z_speed_mps", 1.0)
        self.declare_parameter("follow_z_speed_gain", 2.0)
        self.declare_parameter("cmd_xy_deadband_m", 0.0)
        self.declare_parameter("follow_yaw_rate_rad_s", 1.0)
        self.declare_parameter("follow_yaw_rate_gain", 2.0)
        self.declare_parameter("follow_yaw_accel_rad_s2", 8.0)
        self.declare_parameter("follow_yaw_accel_boost_ref_rad", 0.6)
        self.declare_parameter("follow_yaw_accel_boost_max_scale", 2.0)
        self.declare_parameter("yaw_deadband_rad", 0.0)
        self.declare_parameter("yaw_update_xy_gate_m", 0.0)
        self.declare_parameter("camera_x_offset_m", 0.0)
        self.declare_parameter("camera_y_offset_m", 0.0)
        self.declare_parameter("camera_z_offset_m", 0.27)
        self.declare_parameter("leader_look_target_x_m", 0.0)
        self.declare_parameter("leader_look_target_y_m", 0.0)
        self.declare_parameter("leader_status_topic", "/coord/leader_estimate_status")
        self.declare_parameter("publish_debug_topics", True)
        self.declare_parameter("publish_pose_cmd_topics", True)
        self.declare_parameter("quality_scale_enable", True)
        self.declare_parameter("quality_status_timeout_s", 1.5)
        self.declare_parameter("quality_conf_min", 0.15)
        self.declare_parameter("quality_conf_good", 0.65)
        self.declare_parameter("quality_latency_ref_ms", 180.0)
        self.declare_parameter("quality_min_step_scale", 0.25)
        self.declare_parameter("quality_hold_step_scale", 0.08)
        self.declare_parameter("quality_deadband_boost_m", 0.05)
        self.declare_parameter("state_machine_enable", True)
        self.declare_parameter("state_ok_debounce_ticks", 2)
        self.declare_parameter("state_bad_debounce_ticks", 2)
        self.declare_parameter("traj_enable", True)
        self.declare_parameter("traj_rel_frame_enable", True)
        self.declare_parameter("traj_rel_smooth_alpha", 0.35)
        self.declare_parameter("traj_pos_gain", 2.0)
        self.declare_parameter("traj_max_speed_mps", 1.5)
        self.declare_parameter("traj_max_accel_mps2", 3.0)
        self.declare_parameter("traj_error_boost_ref_m", 2.5)
        self.declare_parameter("traj_error_boost_max_scale", 1.8)
        self.declare_parameter("traj_quality_min_scale", 0.35)
        self.declare_parameter("traj_reset_on_yaw_jump_rad", 0.7)
        # Estimate-mode already receives a tracker-stabilized pose yaw from
        # leader_estimator. Re-deriving heading again here adds noise to the
        # anchor frame, so keep this opt-in.
        self.declare_parameter("estimate_heading_from_motion_enable", False)
        self.declare_parameter("estimate_heading_alpha", 0.35)
        self.declare_parameter("estimate_heading_min_speed_mps", 0.15)
        self.declare_parameter("estimate_heading_max_dt_s", 1.0)

        # ---------- Read params ----------
        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        leader_input_type_raw = str(self.get_parameter("leader_input_type").value).strip().lower()
        self.leader_input_type = leader_input_type_raw
        self.leader_pose_topic = str(self.get_parameter("leader_pose_topic").value)
        self.leader_actual_heading_enable = coerce_bool(
            self.get_parameter("leader_actual_heading_enable").value
        )
        self.leader_actual_heading_topic = str(
            self.get_parameter("leader_actual_heading_topic").value
        )

        if self.leader_input_type == "estimate":
            self.leader_input_type = "pose"
        if self.leader_input_type != "pose":
            raise ValueError(
                "follow_uav only supports 'pose'/'estimate'; "
                f"got leader_input_type={leader_input_type_raw!r}; "
                "use follow_uav_odom for odom mode"
            )

        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.d_target = float(self.get_parameter("d_target").value)
        self.d_max = float(self.get_parameter("d_max").value)
        self.z_alt = float(self.get_parameter("z_alt").value)
        self.d_euclidean = float(self.get_parameter("d_euclidean").value)
        self.manual_override_enable = coerce_bool(self.get_parameter("manual_override_enable").value)
        self.seed_uav_cmd_on_start = coerce_bool(self.get_parameter("seed_uav_cmd_on_start").value)
        self.startup_reposition_enable = coerce_bool(self.get_parameter("startup_reposition_enable").value)
        self.startup_reposition_tol_m = float(self.get_parameter("startup_reposition_tol_m").value)
        self.uav_start_x = float(self.get_parameter("uav_start_x").value)
        self.uav_start_y = float(self.get_parameter("uav_start_y").value)
        self.uav_start_yaw = math.radians(float(self.get_parameter("uav_start_yaw_deg").value))

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
        self.follow_yaw_accel_rad_s2 = float(self.get_parameter("follow_yaw_accel_rad_s2").value)
        self.follow_yaw_accel_boost_ref_rad = float(
            self.get_parameter("follow_yaw_accel_boost_ref_rad").value
        )
        self.follow_yaw_accel_boost_max_scale = float(
            self.get_parameter("follow_yaw_accel_boost_max_scale").value
        )
        self.yaw_deadband_rad = float(self.get_parameter("yaw_deadband_rad").value)
        self.yaw_update_xy_gate_m = float(self.get_parameter("yaw_update_xy_gate_m").value)
        self.camera_x_offset_m = float(self.get_parameter("camera_x_offset_m").value)
        self.camera_y_offset_m = float(self.get_parameter("camera_y_offset_m").value)
        self.camera_z_offset_m = float(self.get_parameter("camera_z_offset_m").value)
        self.leader_look_target_x_m = float(self.get_parameter("leader_look_target_x_m").value)
        self.leader_look_target_y_m = float(self.get_parameter("leader_look_target_y_m").value)
        self.base_follow_speed_mps = self.follow_speed_mps
        self.leader_status_topic = str(self.get_parameter("leader_status_topic").value)
        self.publish_debug_topics = coerce_bool(self.get_parameter("publish_debug_topics").value)
        self.publish_pose_cmd_topics = coerce_bool(self.get_parameter("publish_pose_cmd_topics").value)
        self.quality_scale_enable = coerce_bool(self.get_parameter("quality_scale_enable").value)
        self.quality_status_timeout_s = float(self.get_parameter("quality_status_timeout_s").value)
        self.quality_conf_min = float(self.get_parameter("quality_conf_min").value)
        self.quality_conf_good = float(self.get_parameter("quality_conf_good").value)
        self.quality_latency_ref_ms = float(self.get_parameter("quality_latency_ref_ms").value)
        self.quality_min_step_scale = float(self.get_parameter("quality_min_step_scale").value)
        self.quality_hold_step_scale = float(self.get_parameter("quality_hold_step_scale").value)
        self.quality_deadband_boost_m = float(self.get_parameter("quality_deadband_boost_m").value)
        self.state_machine_enable = coerce_bool(self.get_parameter("state_machine_enable").value)
        self.state_ok_debounce_ticks = int(self.get_parameter("state_ok_debounce_ticks").value)
        self.state_bad_debounce_ticks = int(self.get_parameter("state_bad_debounce_ticks").value)
        self.traj_enable = coerce_bool(self.get_parameter("traj_enable").value)
        self.traj_rel_frame_enable = coerce_bool(self.get_parameter("traj_rel_frame_enable").value)
        self.traj_rel_smooth_alpha = float(self.get_parameter("traj_rel_smooth_alpha").value)
        self.traj_pos_gain = float(self.get_parameter("traj_pos_gain").value)
        self.traj_max_speed_mps = float(self.get_parameter("traj_max_speed_mps").value)
        self.traj_max_accel_mps2 = float(self.get_parameter("traj_max_accel_mps2").value)
        self.traj_error_boost_ref_m = float(self.get_parameter("traj_error_boost_ref_m").value)
        self.traj_error_boost_max_scale = float(self.get_parameter("traj_error_boost_max_scale").value)
        self.traj_quality_min_scale = float(self.get_parameter("traj_quality_min_scale").value)
        self.traj_reset_on_yaw_jump_rad = float(self.get_parameter("traj_reset_on_yaw_jump_rad").value)
        self.estimate_heading_from_motion_enable = coerce_bool(self.get_parameter("estimate_heading_from_motion_enable").value)
        self.estimate_heading_alpha = float(self.get_parameter("estimate_heading_alpha").value)
        self.estimate_heading_min_speed_mps = float(self.get_parameter("estimate_heading_min_speed_mps").value)
        self.estimate_heading_max_dt_s = float(self.get_parameter("estimate_heading_max_dt_s").value)
        self.camera_look_target_z_m = 0.0
        self.current_leader_distance_xy_m = max(
            0.01,
            math.hypot(self.d_target - self.camera_x_offset_m, self.camera_y_offset_m),
        )
        self.current_leader_distance_3d_m = math.hypot(
            self.current_leader_distance_xy_m,
            max(0.0, self.z_alt - self.camera_z_offset_m - self.camera_look_target_z_m),
        )
        self.ugv_z = 0.0
        self.uav_actual_z = self.z_alt
        self.uav_cmd_z = self.z_alt
        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.d_max <= 0.0 or self.d_target <= 0.0:
            raise ValueError("d_max and d_target must be > 0")
        if self.d_max <= self.d_target:
            self.get_logger().warn(
                f"d_max ({self.d_max}) <= d_target ({self.d_target}). "
                f"Leash will trigger/clamp often. Recommend d_max > d_target."
            )
        if self.startup_reposition_tol_m < 0.0:
            raise ValueError("startup_reposition_tol_m must be >= 0")
        if self.d_euclidean <= 0.0:
            self.d_euclidean = math.hypot(self.d_target, self.z_alt)
        self.d_euclidean = max(self.d_euclidean, 0.01)
        self.d_euclidean_xy_ratio = self.d_target / self.d_euclidean
        self.d_euclidean_z_ratio = self.z_alt / self.d_euclidean
        if not (0.0 <= self.smooth_alpha <= 1.0):
            raise ValueError("smooth_alpha must be in [0,1]")
        if self.follow_speed_mps < 0.0:
            raise ValueError("follow_speed_mps must be >= 0")
        if self.follow_speed_gain < 0.0:
            raise ValueError("follow_speed_gain must be >= 0")
        if self.follow_z_speed_mps < 0.0:
            raise ValueError("follow_z_speed_mps must be >= 0")
        if self.follow_z_speed_gain < 0.0:
            raise ValueError("follow_z_speed_gain must be >= 0")
        if self.cmd_xy_deadband_m < 0.0:
            raise ValueError("cmd_xy_deadband_m must be >= 0")
        if self.follow_yaw_rate_rad_s < 0.0:
            raise ValueError("follow_yaw_rate_rad_s must be >= 0")
        if self.follow_yaw_rate_gain < 0.0:
            raise ValueError("follow_yaw_rate_gain must be >= 0")
        if self.follow_yaw_accel_rad_s2 < 0.0:
            raise ValueError("follow_yaw_accel_rad_s2 must be >= 0")
        if self.follow_yaw_accel_boost_ref_rad < 0.0:
            raise ValueError("follow_yaw_accel_boost_ref_rad must be >= 0")
        if self.follow_yaw_accel_boost_max_scale < 1.0:
            raise ValueError("follow_yaw_accel_boost_max_scale must be >= 1")
        if self.yaw_deadband_rad < 0.0:
            raise ValueError("yaw_deadband_rad must be >= 0")
        if self.yaw_update_xy_gate_m < 0.0:
            raise ValueError("yaw_update_xy_gate_m must be >= 0")
        if self.quality_status_timeout_s < 0.0:
            raise ValueError("quality_status_timeout_s must be >= 0")
        if self.quality_conf_good <= self.quality_conf_min:
            raise ValueError("quality_conf_good must be > quality_conf_min")
        if self.quality_latency_ref_ms <= 0.0:
            raise ValueError("quality_latency_ref_ms must be > 0")
        if not (0.0 <= self.quality_min_step_scale <= 1.0):
            raise ValueError("quality_min_step_scale must be in [0,1]")
        if not (0.0 <= self.quality_hold_step_scale <= 1.0):
            raise ValueError("quality_hold_step_scale must be in [0,1]")
        if self.quality_deadband_boost_m < 0.0:
            raise ValueError("quality_deadband_boost_m must be >= 0")
        if self.state_ok_debounce_ticks < 1:
            raise ValueError("state_ok_debounce_ticks must be >= 1")
        if self.state_bad_debounce_ticks < 1:
            raise ValueError("state_bad_debounce_ticks must be >= 1")
        if not (0.0 <= self.traj_rel_smooth_alpha <= 1.0):
            raise ValueError("traj_rel_smooth_alpha must be in [0,1]")
        if self.traj_pos_gain < 0.0:
            raise ValueError("traj_pos_gain must be >= 0")
        if self.traj_max_speed_mps < 0.0:
            raise ValueError("traj_max_speed_mps must be >= 0")
        if self.traj_max_accel_mps2 < 0.0:
            raise ValueError("traj_max_accel_mps2 must be >= 0")
        if self.traj_error_boost_ref_m < 0.0:
            raise ValueError("traj_error_boost_ref_m must be >= 0")
        if self.traj_error_boost_max_scale < 1.0:
            raise ValueError("traj_error_boost_max_scale must be >= 1")
        if not (0.0 <= self.traj_quality_min_scale <= 1.0):
            raise ValueError("traj_quality_min_scale must be in [0,1]")
        if self.traj_reset_on_yaw_jump_rad < 0.0:
            raise ValueError("traj_reset_on_yaw_jump_rad must be >= 0")
        if not (0.0 <= self.estimate_heading_alpha <= 1.0):
            raise ValueError("estimate_heading_alpha must be in [0,1]")
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
        self.traj_rel_target_ema: Optional[Tuple[float, float]] = None
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
            10,
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
            10,
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
            f"d_target={self.d_target}, d_max={self.d_max}, z={self.z_alt}, "
            f"d_euclidean={self.d_euclidean}, "
            f"manual_override_enable={self.manual_override_enable}, "
            f"pose_timeout_s={self.pose_timeout_s}, min_cmd_period_s={self.min_cmd_period_s}, "
            f"smooth_alpha={self.smooth_alpha}, follow_speed_mps={self.follow_speed_mps}, "
            f"follow_speed_gain={self.follow_speed_gain}, "
            f"follow_z_speed_mps={self.follow_z_speed_mps}, "
            f"follow_z_speed_gain={self.follow_z_speed_gain}, "
            f"seed_uav_cmd_on_start={self.seed_uav_cmd_on_start}, "
            f"startup_reposition_enable={self.startup_reposition_enable}, "
            f"uav_start=({self.uav_start_x:.2f},{self.uav_start_y:.2f},{math.degrees(self.uav_start_yaw):.1f}deg), "
            f"cmd_xy_deadband_m={self.cmd_xy_deadband_m}, follow_yaw_rate_rad_s={self.follow_yaw_rate_rad_s}, "
            f"follow_yaw_rate_gain={self.follow_yaw_rate_gain}, "
            f"follow_yaw_accel_rad_s2={self.follow_yaw_accel_rad_s2}, "
            f"follow_yaw_accel_boost_ref_rad={self.follow_yaw_accel_boost_ref_rad}, "
            f"follow_yaw_accel_boost_max_scale={self.follow_yaw_accel_boost_max_scale}, "
            f"yaw_deadband_rad={self.yaw_deadband_rad}, yaw_update_xy_gate_m={self.yaw_update_xy_gate_m}, "
            f"camera_offset_m=({self.camera_x_offset_m}, {self.camera_y_offset_m}, {self.camera_z_offset_m}), "
            f"leader_look_target_xy_m=({self.leader_look_target_x_m}, {self.leader_look_target_y_m}), "
            f"quality_scale_enable={self.quality_scale_enable}, leader_status_topic={self.leader_status_topic}, "
            f"publish_debug_topics={self.publish_debug_topics}, publish_pose_cmd_topics={self.publish_pose_cmd_topics}, "
            f"quality_status_timeout_s={self.quality_status_timeout_s}, quality_conf=[{self.quality_conf_min},{self.quality_conf_good}], "
            f"quality_latency_ref_ms={self.quality_latency_ref_ms}, quality_min_step_scale={self.quality_min_step_scale}, "
            f"quality_hold_step_scale={self.quality_hold_step_scale}, quality_deadband_boost_m={self.quality_deadband_boost_m}, "
            f"state_machine_enable={self.state_machine_enable}, state_debounce=({self.state_ok_debounce_ticks},{self.state_bad_debounce_ticks}), "
            f"traj_enable={self.traj_enable}, traj_rel_frame_enable={self.traj_rel_frame_enable}, "
            f"traj_rel_smooth_alpha={self.traj_rel_smooth_alpha}, traj_pos_gain={self.traj_pos_gain}, "
            f"traj_max_speed_mps={self.traj_max_speed_mps}, traj_max_accel_mps2={self.traj_max_accel_mps2}, "
            f"traj_error_boost_ref_m={self.traj_error_boost_ref_m}, "
            f"traj_error_boost_max_scale={self.traj_error_boost_max_scale}, "
            f"traj_quality_min_scale={self.traj_quality_min_scale}, traj_reset_on_yaw_jump_rad={self.traj_reset_on_yaw_jump_rad}, "
            f"estimate_heading_from_motion_enable={self.estimate_heading_from_motion_enable}, "
            f"leader_actual_heading_enable={self.leader_actual_heading_enable}, "
            f"leader_actual_heading_topic={self.leader_actual_heading_topic}, "
            f"estimate_heading_alpha={self.estimate_heading_alpha}, estimate_heading_min_speed_mps={self.estimate_heading_min_speed_mps}, "
            f"estimate_heading_max_dt_s={self.estimate_heading_max_dt_s}, "
            f"uav_cmd_topic=/{self.uav_name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
        )

    def _on_set_parameters(self, params):
        updates = {}
        bool_updates = {}
        for param in params:
            if param.name == "manual_override_enable":
                bool_updates[param.name] = coerce_bool(param.value)
                continue
            if param.name not in ("d_euclidean", "d_target", "z_alt"):
                continue
            try:
                value = float(param.value)
            except Exception as exc:
                return SetParametersResult(successful=False, reason=f"invalid {param.name}: {exc}")
            if value <= 0.0:
                return SetParametersResult(successful=False, reason=f"{param.name} must be > 0")
            updates[param.name] = value

        if not updates and not bool_updates:
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

        if bool_updates:
            self.manual_override_enable = bool_updates["manual_override_enable"]

        update_parts = []
        if updates:
            update_parts.append(
                f"d_target={self.d_target:.2f}, z_alt={self.z_alt:.2f}, d_euclidean={self.d_euclidean:.2f}"
            )
        if bool_updates:
            update_parts.append(f"manual_override_enable={self.manual_override_enable}")
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
        if self.quality_status_timeout_s > 0.0:
            age_s = (now - self.last_leader_status_rx).nanoseconds * 1e-9
            if age_s > self.quality_status_timeout_s:
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
            q = max(self.quality_min_step_scale, q)
        q = max(0.0, min(1.0, q))
        return q, (state if state else "ok")

    def _leader_status_state(self) -> str:
        return str(self.last_leader_status_fields.get("state", "")).strip()

    def _freeze_yaw_for_status(self) -> bool:
        return self._leader_status_state() in {"REJECT_HOLD", "REJECT_DEBOUNCE_HOLD"}

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
                if self._state_good_ticks >= self.state_ok_debounce_ticks:
                    self._set_follow_state(desired_state)
            else:
                self._state_bad_ticks += 1
                self._state_good_ticks = 0
                if self._state_bad_ticks >= self.state_bad_debounce_ticks:
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
        self.traj_rel_target_ema = None
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

    @staticmethod
    def _error_boost_scale(error_mag: float, ref_mag: float, max_scale: float) -> float:
        if ref_mag <= 1e-6 or max_scale <= 1.0:
            return 1.0
        alpha = max(0.0, min(1.0, error_mag / ref_mag))
        return 1.0 + (max_scale - 1.0) * alpha

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
        if dt > self.estimate_heading_max_dt_s:
            self.leader_motion_prev_xy = (pose.x, pose.y)
            self.leader_motion_prev_stamp = stamp
            return

        dx = pose.x - self.leader_motion_prev_xy[0]
        dy = pose.y - self.leader_motion_prev_xy[1]
        rvx = dx / dt
        rvy = dy / dt
        a = self.estimate_heading_alpha
        if self.leader_motion_prev_xy is None or a >= 1.0:
            self.leader_motion_vx = rvx
            self.leader_motion_vy = rvy
        else:
            self.leader_motion_vx = a * rvx + (1.0 - a) * self.leader_motion_vx
            self.leader_motion_vy = a * rvy + (1.0 - a) * self.leader_motion_vy
        self.leader_motion_speed_mps = math.hypot(self.leader_motion_vx, self.leader_motion_vy)
        if self.leader_motion_speed_mps >= self.estimate_heading_min_speed_mps:
            raw_yaw = math.atan2(self.leader_motion_vy, self.leader_motion_vx)
            if self.leader_motion_heading_yaw is None or a >= 1.0:
                self.leader_motion_heading_yaw = raw_yaw
            else:
                dyaw = wrap_pi(raw_yaw - self.leader_motion_heading_yaw)
                self.leader_motion_heading_yaw = wrap_pi(self.leader_motion_heading_yaw + a * dyaw)

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
        if motion_heading_ok and estimate_heading_source in {"", "none", "fallback"}:
            return self.leader_motion_heading_yaw, "motion_heading_fallback", estimate_yaw, actual_yaw
        return estimate_yaw, "estimate_pose_yaw", estimate_yaw, actual_yaw

    def _shape_target_trajectory(self, leader: Pose2D, xt: float, yt: float, now: Time, quality_scale: float) -> Tuple[float, float]:
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

        if self.traj_rel_target_ema is None or self.traj_rel_smooth_alpha >= 1.0:
            ema_x, ema_y = des_rel_x, des_rel_y
        else:
            a = self.traj_rel_smooth_alpha
            ema_x = a * des_rel_x + (1.0 - a) * self.traj_rel_target_ema[0]
            ema_y = a * des_rel_y + (1.0 - a) * self.traj_rel_target_ema[1]
        self.traj_rel_target_ema = (ema_x, ema_y)

        ex = ema_x - cur_rel_x
        ey = ema_y - cur_rel_y
        err_mag = math.hypot(ex, ey)

        q_scale = 1.0
        if self.quality_scale_enable:
            q_scale = max(self.traj_quality_min_scale, quality_scale)

        jump_boost = self._error_boost_scale(
            err_mag,
            self.traj_error_boost_ref_m,
            self.traj_error_boost_max_scale,
        )
        vmax = self.traj_max_speed_mps * q_scale * jump_boost
        amax = self.traj_max_accel_mps2 * q_scale * jump_boost

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

        # Prevent overshoot relative to the smoothed target.
        if (ex * (ex - step_x) + ey * (ey - step_y)) < 0.0:
            nxt_x, nxt_y = ema_x, ema_y
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
        except Exception:
            self.last_ugv_stamp = self.get_clock().now()
        self._update_leader_motion_model(self.ugv_pose, self.last_ugv_stamp)

    def on_leader_actual_heading_odom(self, msg: Odometry):
        q = msg.pose.pose.orientation
        self.last_actual_heading_yaw = yaw_from_quat(
            float(q.x), float(q.y), float(q.z), float(q.w)
        )
        try:
            self.last_actual_heading_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_actual_heading_stamp = self.get_clock().now()

    def on_uav_pose(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        self.uav_actual = Pose2D(float(p.x), float(p.y), float(yaw))
        self.uav_actual_z = float(p.z)
        self.have_uav_actual = True
        try:
            self.last_uav_actual_time = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_actual_time = self.get_clock().now()

    def _current_uav_pose(self) -> Pose2D:
        if self.have_uav_actual:
            return self.uav_actual
        if self.have_uav_cmd:
            return self.uav_cmd
        return Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)

    def _current_uav_z(self) -> float:
        if self.have_uav_actual:
            return self.uav_actual_z
        if self.have_uav_cmd:
            return self.uav_cmd_z
        return self.z_alt

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

    def ugv_pose_is_fresh(self, now: Time) -> bool:
        if not self.have_ugv or self.last_ugv_stamp is None:
            return False
        age = (now - self.last_ugv_stamp).nanoseconds * 1e-9
        return age <= self.pose_timeout_s

    def leader_actual_heading_is_fresh(self) -> bool:
        if self.last_actual_heading_yaw is None or self.last_actual_heading_stamp is None:
            return False
        now = self.get_clock().now()
        age = (now - self.last_actual_heading_stamp).nanoseconds * 1e-9
        return age <= self.pose_timeout_s

    def can_send_command_now(self, now: Time) -> bool:
        if self.last_cmd_time is None:
            return True
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        return dt >= self.min_cmd_period_s

    def publish_legacy_uav_command(self, x: float, y: float, z: float, yaw_rad: float):
        msg = Joy()
        msg.axes = [float(x), float(y), float(z), float(yaw_rad)]
        self.uav_cmd_pub.publish(msg)

    def publish_pose_cmd(self, x: float, y: float, z: float, yaw_rad: float):
        if self.pose_pub is None:
            return
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)

        quat = quat_from_yaw(float(yaw_rad))
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])

        self.pose_pub.publish(ps)

    def publish_pose_cmd_odom(self, x: float, y: float, z: float, yaw_rad: float):
        if self.pose_odom_pub is None:
            self.uav_cmd_z = float(z)
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = f"{self.uav_name}/base_link"

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(z)

        quat = quat_from_yaw(float(yaw_rad))
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])

        # Teleport/set-pose control has no reliable twist here; leave zeros.
        self.pose_odom_pub.publish(odom)
        self.uav_cmd_z = float(z)

    def _camera_xy_from_uav_pose(self, x: float, y: float, yaw: float) -> Tuple[float, float]:
        return camera_xy_from_uav_pose(
            x,
            y,
            yaw,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )

    def _solve_yaw_to_target(self, uav_x: float, uav_y: float, target_x: float, target_y: float) -> float:
        return solve_yaw_to_target(
            uav_x,
            uav_y,
            target_x,
            target_y,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )

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
            leader_for_follow = Pose2D(
                self.uav_start_x + self.d_target, self.uav_start_y, current_uav.yaw
            )
        anchor_distance_error, anchor_along_error, anchor_cross_error = self._anchor_errors(
            leader_for_follow, anchor_target, current_uav
        )
        self.debug_pubs.publish(
            stamp=now_msg,
            frame_id="map",
            anchor_target=anchor_target,
            d_target=self.d_target,
            z_alt=self.z_alt,
            d_euclidean=math.hypot(self.d_target, self.z_alt),
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
            leader_heading_source=leader_heading_source,
            leader_follow_yaw=leader_follow_yaw,
            leader_estimate_yaw=leader_estimate_yaw,
            leader_actual_heading_yaw=leader_actual_heading_yaw,
        )

    def _effective_follow_speed_mps(self, anchor_distance_error: float) -> float:
        return min(
            self.follow_speed_mps,
            self.follow_speed_gain * max(anchor_distance_error, 0.0),
        )

    def _compute_command_z(self, current_z: float) -> float:
        if self.tick_hz <= 0.0:
            return float(self.z_alt)
        z_error = float(self.z_alt) - float(current_z)
        if abs(z_error) <= 0.01:
            return float(self.z_alt)
        z_speed_mps = min(
            max(self.follow_z_speed_mps, 0.0),
            max(self.follow_z_speed_gain, 0.0) * abs(z_error),
        )
        if z_speed_mps <= 0.0:
            return float(current_z)
        step_limit = z_speed_mps / self.tick_hz
        if abs(z_error) <= step_limit:
            return float(self.z_alt)
        return float(current_z) + math.copysign(step_limit, z_error)

    def _current_follow_geometry(self) -> Tuple[float, float]:
        if self.have_ugv:
            leader_x = self.ugv_pose.x
            leader_y = self.ugv_pose.y
            leader_z = self.ugv_z
        else:
            leader_x = self.uav_start_x + self.d_target
            leader_y = self.uav_start_y
            leader_z = 0.0

        current_uav = self._current_uav_pose()
        uav_x = current_uav.x
        uav_y = current_uav.y
        camera_x, camera_y = self._camera_xy_from_uav_pose(uav_x, uav_y, current_uav.yaw)
        uav_z = self._current_uav_z()

        horizontal_distance = math.hypot(leader_x - camera_x, leader_y - camera_y)
        if horizontal_distance < 1e-3:
            horizontal_distance = max(
                0.01,
                math.hypot(self.d_target - self.camera_x_offset_m, self.camera_y_offset_m),
            )
        distance_3d = math.hypot(horizontal_distance, uav_z - leader_z)

        self.current_leader_distance_xy_m = horizontal_distance
        self.current_leader_distance_3d_m = distance_3d
        return horizontal_distance, distance_3d

    def _compute_anchor_target(self) -> Pose2D:
        leader_for_follow = self._leader_pose_for_follow()
        xt = leader_for_follow.x - self.d_target * math.cos(leader_for_follow.yaw)
        yt = leader_for_follow.y - self.d_target * math.sin(leader_for_follow.yaw)

        xt, yt = clamp_point_to_radius(leader_for_follow.x, leader_for_follow.y, xt, yt, self.d_max)

        if self.follow_yaw:
            target_x, target_y = self._leader_look_target_xy(leader_for_follow)
            yaw_cmd = self._solve_yaw_to_target(xt, yt, target_x, target_y)
        else:
            yaw_cmd = self._current_uav_pose().yaw

        return Pose2D(xt, yt, yaw_cmd)

    def _maybe_publish_startup_reposition(self, now: Time) -> bool:
        if not self.startup_reposition_enable:
            return False
        if self.have_seen_ugv_pose:
            return False
        current_uav = self._current_uav_pose()
        current_z = self._current_uav_z()
        dx = self.uav_start_x - current_uav.x
        dy = self.uav_start_y - current_uav.y
        dz = self.z_alt - current_z
        if math.hypot(dx, dy) <= self.startup_reposition_tol_m and abs(dz) <= self.startup_reposition_tol_m:
            return False
        self.publish_legacy_uav_command(self.uav_start_x, self.uav_start_y, self.z_alt, self.uav_start_yaw)
        self.publish_pose_cmd(self.uav_start_x, self.uav_start_y, self.z_alt, self.uav_start_yaw)
        self.publish_pose_cmd_odom(self.uav_start_x, self.uav_start_y, self.z_alt, self.uav_start_yaw)
        self.uav_cmd = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.have_uav_cmd = True
        self.last_cmd_time = now
        return True

    def on_tick(self):
        now = self.get_clock().now()
        if self.manual_override_enable:
            self._reset_yaw_state()
            self._set_follow_state("MANUAL")
            return
        current_uav = self._control_uav_pose()
        current_uav_z = self._control_uav_z()
        control_dt = self._control_dt_s(now)

        if not self.ugv_pose_is_fresh(now):
            if self.can_send_command_now(now) and self._maybe_publish_startup_reposition(now):
                return
            self._reset_yaw_state()
            self._update_follow_state("HOLD")
            return

        if not self.can_send_command_now(now):
            return

        quality_scale, _quality_reason = self._quality_scale_from_status(now)
        desired_follow_state = self._classify_follow_state(quality_scale)
        self._update_follow_state(desired_follow_state)

        leader_for_follow = self._leader_pose_for_follow()
        anchor_target = self._compute_anchor_target()
        xt = anchor_target.x
        yt = anchor_target.y

        if self.follow_state == "HOLD":
            self._reset_traj_state()
            self._reset_yaw_state()
            xt = current_uav.x
            yt = current_uav.y
        elif (self.have_uav_actual or self.have_uav_cmd) and self.smooth_alpha < 1.0:
            a = self.smooth_alpha
            xt = a * xt + (1.0 - a) * current_uav.x
            yt = a * yt + (1.0 - a) * current_uav.y

        if self.follow_state != "HOLD":
            xt, yt = self._shape_target_trajectory(leader_for_follow, xt, yt, now, quality_scale)

        anchor_distance_error, _, _ = self._anchor_errors(leader_for_follow, anchor_target, current_uav)
        effective_follow_speed_mps = self._effective_follow_speed_mps(anchor_distance_error)
        if self.quality_scale_enable:
            effective_follow_speed_mps *= max(0.0, min(1.0, quality_scale))
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
        target_x, target_y = self._leader_look_target_xy(leader_for_follow)
        yaw_target = self._solve_yaw_to_target(
            xt,
            yt,
            target_x,
            target_y,
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
        yaw_accel_scale = self._error_boost_scale(
            yaw_error_mag,
            self.follow_yaw_accel_boost_ref_rad,
            self.follow_yaw_accel_boost_max_scale,
        )
        yaw_accel_rad_s2 = self.follow_yaw_accel_rad_s2 * yaw_accel_scale
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

        if self.follow_state == "HOLD":
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
        elif self.yaw_update_xy_gate_m > 0.0 and cmd_xy_delta < self.yaw_update_xy_gate_m:
            self._reset_yaw_state()
            yaw_cmd = current_uav.yaw
            yaw_mode = "xy_gate_hold"
        elif self.yaw_deadband_rad > 0.0 and abs(yaw_error) < self.yaw_deadband_rad:
            self._reset_yaw_state()
            yaw_cmd = current_uav.yaw
            yaw_mode = "deadband_hold"
        elif yaw_step_limit > 0.0 and abs(yaw_error) > yaw_step_limit:
            yaw_cmd = wrap_pi(current_uav.yaw + math.copysign(yaw_step_limit, yaw_error))
            yaw_mode = "accel_rate_limited"
        else:
            self._reset_yaw_state()
            yaw_cmd = yaw_target
            yaw_mode = "direct_target"

        yaw_cmd_delta = wrap_pi(yaw_cmd - current_uav.yaw)

        cmd_xy_deadband_m = self.cmd_xy_deadband_m
        if self.quality_scale_enable:
            cmd_xy_deadband_m += self.quality_deadband_boost_m * max(0.0, 1.0 - quality_scale)
        if cmd_xy_deadband_m > 0.0 and cmd_xy_delta < cmd_xy_deadband_m:
            xt = current_uav.x
            yt = current_uav.y

        z_cmd = self._compute_command_z(current_uav_z)
        self.publish_legacy_uav_command(xt, yt, z_cmd, yaw_cmd)
        self.publish_pose_cmd(xt, yt, z_cmd, yaw_cmd)
        self.publish_pose_cmd_odom(xt, yt, z_cmd, yaw_cmd)

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
