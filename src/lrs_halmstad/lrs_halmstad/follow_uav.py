




#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
from rcl_interfaces.msg import ParameterDescriptor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String, Float32, Float64

from ros_gz_interfaces.srv import SetEntityPose


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    # Local yaw-only helpers avoid tf_transformations/transforms3d runtime issues
    # with newer NumPy, while covering this node's current 2D heading needs.
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def quat_from_camera_pitch_deg(pitch_deg: float, yaw_rad: float = 0.0) -> Tuple[float, float, float, float]:
    # Match command.py/simulator behavior:
    # pitch command is interpreted as camera tilt in degrees from horizontal,
    # so world pitch uses -pitch_deg; yaw follows UAV yaw.
    pitch = -math.radians(float(pitch_deg))
    cy = math.cos(float(yaw_rad) * 0.5)
    sy = math.sin(float(yaw_rad) * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    # roll=0
    return (-sp * sy, sp * cy, cp * sy, cp * cy)


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def clamp_mag(x: float, y: float, max_norm: float) -> Tuple[float, float]:
    n = math.hypot(x, y)
    if n <= max_norm or n < 1e-9:
        return x, y
    s = max_norm / n
    return x * s, y * s


def clamp_point_to_radius(center_x: float, center_y: float, x: float, y: float, r: float) -> Tuple[float, float]:
    """
    If (x,y) is farther than r from (center_x, center_y), project it onto the circle of radius r.
    """
    dx = x - center_x
    dy = y - center_y
    d = math.hypot(dx, dy)
    if d <= r or d < 1e-9:
        return x, y
    s = r / d
    return center_x + dx * s, center_y + dy * s


class FollowUav(Node):
    """
    Stage 2 (B1): Follow + Leash controller (robust).
    - Reads UGV odom (default: /a201_0000/platform/odom/filtered)
    - Computes target behind UGV at d_target
    - Enforces leash by clamping target within d_max around UGV (geometrically correct)
    - Commands UAV using /world/<world>/set_pose (Gazebo SetEntityPose)
    - Publishes commanded pose for bagging (/dji0/pose_cmd)
    - Publishes events to EVENT_TOPIC (default /coord/events)
    - Safety: pose freshness timeout -> HOLD (no new commands) + POSE_STALE event
    - Reliability: tracks service future to avoid request backlog
    - Optional smoothing (alpha=1.0 means off)
    """

    def __init__(self):
        super().__init__("follow_uav")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        # ---------- Parameters ----------
        self.declare_parameter("world", "orchard")
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("uav_backend", "setpose")
        self.declare_parameter("camera_name", "camera0")
        self.declare_parameter("camera_pitch_deg", -45.0)
        self.declare_parameter("camera_z_offset_m", 0.27)
        self.declare_parameter("controller_cmd_topic", "")
        self.declare_parameter("controller_pan_topic", "")
        self.declare_parameter("controller_tilt_topic", "")
        self.declare_parameter("controller_seed_x", -2.0)
        self.declare_parameter("controller_seed_y", 0.0)
        self.declare_parameter("controller_seed_yaw_deg", 0.0)
        self.declare_parameter("controller_publish_gimbal", True)
        # Camera control path:
        # - separate_model: drive <uav>_camera0 with set_pose (teleport stack)
        # - gimbal_only: publish pan/tilt topics only (integrated camera/gimbal stack)
        # - auto: start with separate_model in setpose backend, fail over to gimbal_only after repeated camera set_pose failures
        self.declare_parameter("camera_pose_control_mode", "auto")
        self.declare_parameter("camera_pose_failover_failures", 12)
        self.declare_parameter("camera_lock_enable", True)
        self.declare_parameter("camera_lock_gain", 0.6)
        self.declare_parameter("camera_lock_deadband_deg", 1.0)
        self.declare_parameter("camera_lock_pan_limit_deg", 65.0)
        self.declare_parameter("camera_lock_max_step_deg", 4.0)
        self.declare_parameter("camera_lock_hold_on_bad_state", True)
        self.declare_parameter("camera_lock_tilt_enable", True)
        self.declare_parameter("camera_lock_tilt_gain_per_px", 0.02)
        self.declare_parameter("camera_lock_tilt_deadband_px", 10.0)
        self.declare_parameter("camera_lock_tilt_min_deg", -88.0)
        self.declare_parameter("camera_lock_tilt_max_deg", -20.0)
        self.declare_parameter("camera_lock_tilt_max_step_deg", 2.0)
        self.declare_parameter("camera_lock_tilt_hold_on_bad_state", True)
        self.declare_parameter("camera_lock_pan_reset_on_reacquire", False)
        self.declare_parameter("camera_lock_search_enable", True)
        self.declare_parameter("camera_lock_search_rate_deg_s", 18.0)
        self.declare_parameter("camera_lock_search_hold_s", 0.6)
        self.declare_parameter("under_target_guard_enable", True)
        self.declare_parameter("under_target_err_v_ratio", 0.33)
        self.declare_parameter("under_target_range_m", 2.2)
        self.declare_parameter("under_target_hold_xy", True)
        self.declare_parameter("under_target_hold_yaw", True)
        # Structured reacquire sequence on true target loss:
        # PAN (local) -> TILT (vertical sweep) -> YAW (wide search) -> HOLD dwell -> PAN ...
        # with optional long fallback HOLD after extended unsuccessful search.
        self.declare_parameter("reacquire_sequence_enable", True)
        self.declare_parameter("reacquire_pan_phase_s", 2.0)
        self.declare_parameter("reacquire_tilt_phase_s", 2.5)
        self.declare_parameter("reacquire_yaw_phase_s", 8.0)
        self.declare_parameter("reacquire_pan_local_span_deg", 28.0)
        self.declare_parameter("reacquire_tilt_local_span_deg", 12.0)
        self.declare_parameter("reacquire_pan_rate_deg_s", 24.0)
        self.declare_parameter("reacquire_tilt_rate_deg_s", 22.0)
        self.declare_parameter("reacquire_yaw_rate_deg_s", 45.0)
        self.declare_parameter("reacquire_hold_phase_s", 1.5)
        self.declare_parameter("reacquire_fallback_hold_s", 30.0)
        self.declare_parameter("reacquire_hold_after_search", True)
        self.declare_parameter("reacquire_search_hold_xy", True)
        self.declare_parameter("reacquire_search_hold_yaw_before_yaw_phase", True)
        self.declare_parameter("reacquire_commit_enable", True)
        self.declare_parameter("reacquire_commit_s", 1.5)
        self.declare_parameter("reacquire_commit_quality_floor", 0.22)
        self.declare_parameter("reacquire_startup_guard_enable", True)
        self.declare_parameter("reacquire_first_lock_gate_enable", True)
        self.declare_parameter("reacquire_first_lock_min_visible_frames", 3)
        self.declare_parameter("reacquire_relock_min_visible_frames", 3)
        self.declare_parameter("reacquire_yaw_assist_delay_s", 4.0)
        self.declare_parameter("reacquire_xy_assist_delay_s", 7.0)
        self.declare_parameter("controller_profile_enable", True)
        self.declare_parameter("controller_profile_smooth_alpha", 0.85)
        self.declare_parameter("controller_profile_max_step_m_per_tick", 0.30)
        self.declare_parameter("controller_profile_cmd_xy_deadband_m", 0.05)
        self.declare_parameter("controller_profile_yaw_deadband_rad", 0.08)
        self.declare_parameter("controller_profile_min_cmd_period_s", 0.12)
        self.declare_parameter("leader_input_type", "odom")
        self.declare_parameter("leader_odom_topic", "/a201_0000/platform/odom/filtered")
        self.declare_parameter("leader_pose_topic", "/coord/leader_estimate")

        self.declare_parameter("tick_hz", 5.0, dyn_num)
        self.declare_parameter("d_target", 5.0)
        self.declare_parameter("d_max", 15.0)
        self.declare_parameter("z_alt", 10.0)

        self.declare_parameter("follow_yaw", True)

        # Freshness and service pacing
        self.declare_parameter("pose_timeout_s", 0.75)   # stale if odom older than this
        self.declare_parameter("min_cmd_period_s", 0.10) # don't command faster than this even if tick is high

        # Smoothing (alpha=1.0 -> no smoothing; 0.0 -> freeze)
        self.declare_parameter("smooth_alpha", 1.0)
        self.declare_parameter("max_step_m_per_tick", 0.5, dyn_num)
        self.declare_parameter("cmd_xy_deadband_m", 0.0, dyn_num)
        self.declare_parameter("max_yaw_step_rad_per_tick", 0.0, dyn_num)
        self.declare_parameter("yaw_deadband_rad", 0.0, dyn_num)
        self.declare_parameter("yaw_update_xy_gate_m", 0.0, dyn_num)
        self.declare_parameter("leader_status_topic", "/coord/leader_estimate_status")
        self.declare_parameter("quality_scale_enable", True)
        self.declare_parameter("quality_status_timeout_s", 1.5, dyn_num)
        self.declare_parameter("quality_conf_min", 0.15, dyn_num)
        self.declare_parameter("quality_conf_good", 0.65, dyn_num)
        self.declare_parameter("quality_latency_ref_ms", 180.0, dyn_num)
        self.declare_parameter("quality_min_step_scale", 0.25, dyn_num)
        self.declare_parameter("quality_hold_step_scale", 0.08, dyn_num)
        self.declare_parameter("quality_deadband_boost_m", 0.05, dyn_num)
        self.declare_parameter("state_machine_enable", True)
        self.declare_parameter("state_ok_debounce_ticks", 2, dyn_num)
        self.declare_parameter("state_bad_debounce_ticks", 2, dyn_num)
        self.declare_parameter("track_only_motion_enable", True)
        self.declare_parameter("track_only_allow_reacquire_motion", True)
        self.declare_parameter("track_only_hold_yaw_enable", True)
        self.declare_parameter("traj_enable", True)
        self.declare_parameter("traj_rel_frame_enable", True)
        self.declare_parameter("traj_rel_smooth_alpha", 0.35, dyn_num)
        self.declare_parameter("traj_pos_gain", 2.0, dyn_num)
        self.declare_parameter("traj_max_speed_mps", 1.5, dyn_num)
        self.declare_parameter("traj_max_accel_mps2", 3.0, dyn_num)
        self.declare_parameter("traj_quality_min_scale", 0.35, dyn_num)
        self.declare_parameter("traj_reset_on_yaw_jump_rad", 0.7, dyn_num)
        self.declare_parameter("estimate_heading_from_motion_enable", True)
        self.declare_parameter("estimate_heading_alpha", 0.35, dyn_num)
        self.declare_parameter("estimate_heading_min_speed_mps", 0.15, dyn_num)
        self.declare_parameter("estimate_heading_max_dt_s", 1.0, dyn_num)
        self.declare_parameter("stale_predict_enable", True)
        self.declare_parameter("stale_predict_timeout_s", 6.0, dyn_num)
        self.declare_parameter("stale_predict_quality_floor", 0.20, dyn_num)
        self.declare_parameter("xy_motion_enable", True)
        self.declare_parameter("publish_debug_status", True)
        self.declare_parameter("debug_status_topic", "/coord/follow_debug_status")

        # Events
        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("publish_events", True)

        # ---------- Read params ----------
        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.uav_backend = str(self.get_parameter("uav_backend").value).strip().lower()
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.camera_pitch_deg = float(self.get_parameter("camera_pitch_deg").value)
        self.camera_z_offset_m = float(self.get_parameter("camera_z_offset_m").value)
        self.controller_cmd_topic = str(self.get_parameter("controller_cmd_topic").value).strip()
        self.controller_pan_topic = str(self.get_parameter("controller_pan_topic").value).strip()
        self.controller_tilt_topic = str(self.get_parameter("controller_tilt_topic").value).strip()
        self.controller_seed_x = float(self.get_parameter("controller_seed_x").value)
        self.controller_seed_y = float(self.get_parameter("controller_seed_y").value)
        self.controller_seed_yaw_deg = float(self.get_parameter("controller_seed_yaw_deg").value)
        self.controller_publish_gimbal = bool(self.get_parameter("controller_publish_gimbal").value)
        self.camera_pose_control_mode = str(self.get_parameter("camera_pose_control_mode").value).strip().lower()
        self.camera_pose_failover_failures = int(self.get_parameter("camera_pose_failover_failures").value)
        self.camera_lock_enable = bool(self.get_parameter("camera_lock_enable").value)
        self.camera_lock_gain = float(self.get_parameter("camera_lock_gain").value)
        self.camera_lock_deadband_deg = float(self.get_parameter("camera_lock_deadband_deg").value)
        self.camera_lock_pan_limit_deg = float(self.get_parameter("camera_lock_pan_limit_deg").value)
        self.camera_lock_max_step_deg = float(self.get_parameter("camera_lock_max_step_deg").value)
        self.camera_lock_hold_on_bad_state = bool(self.get_parameter("camera_lock_hold_on_bad_state").value)
        self.camera_lock_tilt_enable = bool(self.get_parameter("camera_lock_tilt_enable").value)
        self.camera_lock_tilt_gain_per_px = float(self.get_parameter("camera_lock_tilt_gain_per_px").value)
        self.camera_lock_tilt_deadband_px = float(self.get_parameter("camera_lock_tilt_deadband_px").value)
        self.camera_lock_tilt_min_deg = float(self.get_parameter("camera_lock_tilt_min_deg").value)
        self.camera_lock_tilt_max_deg = float(self.get_parameter("camera_lock_tilt_max_deg").value)
        self.camera_lock_tilt_max_step_deg = float(self.get_parameter("camera_lock_tilt_max_step_deg").value)
        self.camera_lock_tilt_hold_on_bad_state = bool(self.get_parameter("camera_lock_tilt_hold_on_bad_state").value)
        self.camera_lock_pan_reset_on_reacquire = bool(self.get_parameter("camera_lock_pan_reset_on_reacquire").value)
        self.camera_lock_search_enable = bool(self.get_parameter("camera_lock_search_enable").value)
        self.camera_lock_search_rate_deg_s = float(self.get_parameter("camera_lock_search_rate_deg_s").value)
        self.camera_lock_search_hold_s = float(self.get_parameter("camera_lock_search_hold_s").value)
        self.under_target_guard_enable = bool(self.get_parameter("under_target_guard_enable").value)
        self.under_target_err_v_ratio = float(self.get_parameter("under_target_err_v_ratio").value)
        self.under_target_range_m = float(self.get_parameter("under_target_range_m").value)
        self.under_target_hold_xy = bool(self.get_parameter("under_target_hold_xy").value)
        self.under_target_hold_yaw = bool(self.get_parameter("under_target_hold_yaw").value)
        self.reacquire_sequence_enable = bool(self.get_parameter("reacquire_sequence_enable").value)
        self.reacquire_pan_phase_s = float(self.get_parameter("reacquire_pan_phase_s").value)
        self.reacquire_tilt_phase_s = float(self.get_parameter("reacquire_tilt_phase_s").value)
        self.reacquire_yaw_phase_s = float(self.get_parameter("reacquire_yaw_phase_s").value)
        self.reacquire_pan_local_span_deg = float(self.get_parameter("reacquire_pan_local_span_deg").value)
        self.reacquire_tilt_local_span_deg = float(self.get_parameter("reacquire_tilt_local_span_deg").value)
        self.reacquire_pan_rate_deg_s = float(self.get_parameter("reacquire_pan_rate_deg_s").value)
        self.reacquire_tilt_rate_deg_s = float(self.get_parameter("reacquire_tilt_rate_deg_s").value)
        self.reacquire_yaw_rate_deg_s = float(self.get_parameter("reacquire_yaw_rate_deg_s").value)
        self.reacquire_hold_phase_s = float(self.get_parameter("reacquire_hold_phase_s").value)
        self.reacquire_fallback_hold_s = float(self.get_parameter("reacquire_fallback_hold_s").value)
        self.reacquire_hold_after_search = bool(self.get_parameter("reacquire_hold_after_search").value)
        self.reacquire_search_hold_xy = bool(self.get_parameter("reacquire_search_hold_xy").value)
        self.reacquire_search_hold_yaw_before_yaw_phase = bool(
            self.get_parameter("reacquire_search_hold_yaw_before_yaw_phase").value
        )
        self.reacquire_commit_enable = bool(self.get_parameter("reacquire_commit_enable").value)
        self.reacquire_commit_s = float(self.get_parameter("reacquire_commit_s").value)
        self.reacquire_commit_quality_floor = float(self.get_parameter("reacquire_commit_quality_floor").value)
        self.reacquire_startup_guard_enable = bool(self.get_parameter("reacquire_startup_guard_enable").value)
        self.reacquire_first_lock_gate_enable = bool(self.get_parameter("reacquire_first_lock_gate_enable").value)
        self.reacquire_first_lock_min_visible_frames = int(
            self.get_parameter("reacquire_first_lock_min_visible_frames").value
        )
        self.reacquire_relock_min_visible_frames = int(
            self.get_parameter("reacquire_relock_min_visible_frames").value
        )
        self.reacquire_yaw_assist_delay_s = float(self.get_parameter("reacquire_yaw_assist_delay_s").value)
        self.reacquire_xy_assist_delay_s = float(self.get_parameter("reacquire_xy_assist_delay_s").value)
        self.controller_profile_enable = bool(self.get_parameter("controller_profile_enable").value)
        self.controller_profile_smooth_alpha = float(self.get_parameter("controller_profile_smooth_alpha").value)
        self.controller_profile_max_step_m_per_tick = float(self.get_parameter("controller_profile_max_step_m_per_tick").value)
        self.controller_profile_cmd_xy_deadband_m = float(self.get_parameter("controller_profile_cmd_xy_deadband_m").value)
        self.controller_profile_yaw_deadband_rad = float(self.get_parameter("controller_profile_yaw_deadband_rad").value)
        self.controller_profile_min_cmd_period_s = float(self.get_parameter("controller_profile_min_cmd_period_s").value)
        self.camera_model_name = f"{self.uav_name}_{self.camera_name}"
        self.leader_input_type = str(self.get_parameter("leader_input_type").value).strip().lower()
        self.leader_odom_topic = str(self.get_parameter("leader_odom_topic").value)
        self.leader_pose_topic = str(self.get_parameter("leader_pose_topic").value)

        if self.uav_backend not in ("setpose", "controller"):
            raise ValueError("uav_backend must be 'setpose' or 'controller'")
        if self.camera_pose_control_mode not in ("auto", "separate_model", "gimbal_only"):
            raise ValueError("camera_pose_control_mode must be one of: auto, separate_model, gimbal_only")
        if self.camera_pose_failover_failures < 1:
            raise ValueError("camera_pose_failover_failures must be >= 1")
        if not self.controller_cmd_topic:
            self.controller_cmd_topic = f"/{self.uav_name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
        if not self.controller_pan_topic:
            self.controller_pan_topic = f"/{self.uav_name}/update_pan"
        if not self.controller_tilt_topic:
            self.controller_tilt_topic = f"/{self.uav_name}/update_tilt"

        if self.leader_input_type == "estimate":
            self.leader_input_type = "pose"
        if self.leader_input_type not in ("odom", "pose"):
            raise ValueError("leader_input_type must be 'odom', 'pose', or 'estimate'")

        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.d_target = float(self.get_parameter("d_target").value)
        self.d_max = float(self.get_parameter("d_max").value)
        self.z_alt = float(self.get_parameter("z_alt").value)

        self.follow_yaw = bool(self.get_parameter("follow_yaw").value)

        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.min_cmd_period_s = float(self.get_parameter("min_cmd_period_s").value)
        self.smooth_alpha = float(self.get_parameter("smooth_alpha").value)
        self.max_step_m_per_tick = float(self.get_parameter("max_step_m_per_tick").value)
        self.cmd_xy_deadband_m = float(self.get_parameter("cmd_xy_deadband_m").value)
        self.max_yaw_step_rad_per_tick = float(self.get_parameter("max_yaw_step_rad_per_tick").value)
        self.yaw_deadband_rad = float(self.get_parameter("yaw_deadband_rad").value)
        self.yaw_update_xy_gate_m = float(self.get_parameter("yaw_update_xy_gate_m").value)
        self.leader_status_topic = str(self.get_parameter("leader_status_topic").value)
        self.quality_scale_enable = bool(self.get_parameter("quality_scale_enable").value)
        self.quality_status_timeout_s = float(self.get_parameter("quality_status_timeout_s").value)
        self.quality_conf_min = float(self.get_parameter("quality_conf_min").value)
        self.quality_conf_good = float(self.get_parameter("quality_conf_good").value)
        self.quality_latency_ref_ms = float(self.get_parameter("quality_latency_ref_ms").value)
        self.quality_min_step_scale = float(self.get_parameter("quality_min_step_scale").value)
        self.quality_hold_step_scale = float(self.get_parameter("quality_hold_step_scale").value)
        self.quality_deadband_boost_m = float(self.get_parameter("quality_deadband_boost_m").value)
        self.state_machine_enable = bool(self.get_parameter("state_machine_enable").value)
        self.state_ok_debounce_ticks = int(self.get_parameter("state_ok_debounce_ticks").value)
        self.state_bad_debounce_ticks = int(self.get_parameter("state_bad_debounce_ticks").value)
        self.track_only_motion_enable = bool(self.get_parameter("track_only_motion_enable").value)
        self.track_only_allow_reacquire_motion = bool(self.get_parameter("track_only_allow_reacquire_motion").value)
        self.track_only_hold_yaw_enable = bool(self.get_parameter("track_only_hold_yaw_enable").value)
        self.traj_enable = bool(self.get_parameter("traj_enable").value)
        self.traj_rel_frame_enable = bool(self.get_parameter("traj_rel_frame_enable").value)
        self.traj_rel_smooth_alpha = float(self.get_parameter("traj_rel_smooth_alpha").value)
        self.traj_pos_gain = float(self.get_parameter("traj_pos_gain").value)
        self.traj_max_speed_mps = float(self.get_parameter("traj_max_speed_mps").value)
        self.traj_max_accel_mps2 = float(self.get_parameter("traj_max_accel_mps2").value)
        self.traj_quality_min_scale = float(self.get_parameter("traj_quality_min_scale").value)
        self.traj_reset_on_yaw_jump_rad = float(self.get_parameter("traj_reset_on_yaw_jump_rad").value)
        self.estimate_heading_from_motion_enable = bool(self.get_parameter("estimate_heading_from_motion_enable").value)
        self.estimate_heading_alpha = float(self.get_parameter("estimate_heading_alpha").value)
        self.estimate_heading_min_speed_mps = float(self.get_parameter("estimate_heading_min_speed_mps").value)
        self.estimate_heading_max_dt_s = float(self.get_parameter("estimate_heading_max_dt_s").value)
        self.stale_predict_enable = bool(self.get_parameter("stale_predict_enable").value)
        self.stale_predict_timeout_s = float(self.get_parameter("stale_predict_timeout_s").value)
        self.stale_predict_quality_floor = float(self.get_parameter("stale_predict_quality_floor").value)
        self.xy_motion_enable = bool(self.get_parameter("xy_motion_enable").value)
        self.publish_debug_status = bool(self.get_parameter("publish_debug_status").value)
        self.debug_status_topic = str(self.get_parameter("debug_status_topic").value).strip() or "/coord/follow_debug_status"
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.publish_events = bool(self.get_parameter("publish_events").value)

        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.camera_z_offset_m < 0.0:
            raise ValueError("camera_z_offset_m must be >= 0")
        if self.d_max <= 0.0 or self.d_target <= 0.0:
            raise ValueError("d_max and d_target must be > 0")
        if self.d_max <= self.d_target:
            self.get_logger().warn(
                f"d_max ({self.d_max}) <= d_target ({self.d_target}). "
                f"Leash will trigger/clamp often. Recommend d_max > d_target."
            )
        if not (0.0 <= self.smooth_alpha <= 1.0):
            raise ValueError("smooth_alpha must be in [0,1]")
        if self.max_step_m_per_tick < 0.0:
            raise ValueError("max_step_m_per_tick must be >= 0")
        if self.cmd_xy_deadband_m < 0.0:
            raise ValueError("cmd_xy_deadband_m must be >= 0")
        if self.max_yaw_step_rad_per_tick < 0.0:
            raise ValueError("max_yaw_step_rad_per_tick must be >= 0")
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
        if self.stale_predict_timeout_s < 0.0:
            raise ValueError("stale_predict_timeout_s must be >= 0")
        if not (0.0 <= self.stale_predict_quality_floor <= 1.0):
            raise ValueError("stale_predict_quality_floor must be in [0,1]")
        if not (0.0 <= self.controller_profile_smooth_alpha <= 1.0):
            raise ValueError("controller_profile_smooth_alpha must be in [0,1]")
        if self.controller_profile_max_step_m_per_tick < 0.0:
            raise ValueError("controller_profile_max_step_m_per_tick must be >= 0")
        if self.controller_profile_cmd_xy_deadband_m < 0.0:
            raise ValueError("controller_profile_cmd_xy_deadband_m must be >= 0")
        if self.controller_profile_yaw_deadband_rad < 0.0:
            raise ValueError("controller_profile_yaw_deadband_rad must be >= 0")
        if self.controller_profile_min_cmd_period_s < 0.0:
            raise ValueError("controller_profile_min_cmd_period_s must be >= 0")
        if self.camera_lock_gain < 0.0:
            raise ValueError("camera_lock_gain must be >= 0")
        if self.camera_lock_deadband_deg < 0.0:
            raise ValueError("camera_lock_deadband_deg must be >= 0")
        if self.camera_lock_pan_limit_deg < 0.0:
            raise ValueError("camera_lock_pan_limit_deg must be >= 0")
        if self.camera_lock_max_step_deg < 0.0:
            raise ValueError("camera_lock_max_step_deg must be >= 0")
        if self.camera_lock_search_rate_deg_s < 0.0:
            raise ValueError("camera_lock_search_rate_deg_s must be >= 0")
        if self.camera_lock_search_hold_s < 0.0:
            raise ValueError("camera_lock_search_hold_s must be >= 0")
        if self.camera_lock_tilt_gain_per_px < 0.0:
            raise ValueError("camera_lock_tilt_gain_per_px must be >= 0")
        if self.camera_lock_tilt_deadband_px < 0.0:
            raise ValueError("camera_lock_tilt_deadband_px must be >= 0")
        if self.camera_lock_tilt_max_step_deg < 0.0:
            raise ValueError("camera_lock_tilt_max_step_deg must be >= 0")
        if self.camera_lock_tilt_min_deg > self.camera_lock_tilt_max_deg:
            raise ValueError("camera_lock_tilt_min_deg must be <= camera_lock_tilt_max_deg")
        if not (0.0 <= self.under_target_err_v_ratio <= 1.0):
            raise ValueError("under_target_err_v_ratio must be in [0,1]")
        if self.under_target_range_m < 0.0:
            raise ValueError("under_target_range_m must be >= 0")
        if self.reacquire_pan_phase_s < 0.0:
            raise ValueError("reacquire_pan_phase_s must be >= 0")
        if self.reacquire_tilt_phase_s < 0.0:
            raise ValueError("reacquire_tilt_phase_s must be >= 0")
        if self.reacquire_yaw_phase_s < 0.0:
            raise ValueError("reacquire_yaw_phase_s must be >= 0")
        if self.reacquire_pan_local_span_deg < 0.0:
            raise ValueError("reacquire_pan_local_span_deg must be >= 0")
        if self.reacquire_tilt_local_span_deg < 0.0:
            raise ValueError("reacquire_tilt_local_span_deg must be >= 0")
        if self.reacquire_pan_rate_deg_s < 0.0:
            raise ValueError("reacquire_pan_rate_deg_s must be >= 0")
        if self.reacquire_tilt_rate_deg_s < 0.0:
            raise ValueError("reacquire_tilt_rate_deg_s must be >= 0")
        if self.reacquire_yaw_rate_deg_s < 0.0:
            raise ValueError("reacquire_yaw_rate_deg_s must be >= 0")
        if self.reacquire_hold_phase_s < 0.0:
            raise ValueError("reacquire_hold_phase_s must be >= 0")
        if self.reacquire_fallback_hold_s < 0.0:
            raise ValueError("reacquire_fallback_hold_s must be >= 0")
        if self.reacquire_commit_s < 0.0:
            raise ValueError("reacquire_commit_s must be >= 0")
        if not (0.0 <= self.reacquire_commit_quality_floor <= 1.0):
            raise ValueError("reacquire_commit_quality_floor must be in [0,1]")
        if self.reacquire_first_lock_min_visible_frames < 1:
            raise ValueError("reacquire_first_lock_min_visible_frames must be >= 1")
        if self.reacquire_relock_min_visible_frames < 1:
            raise ValueError("reacquire_relock_min_visible_frames must be >= 1")
        if self.reacquire_yaw_assist_delay_s < 0.0:
            raise ValueError("reacquire_yaw_assist_delay_s must be >= 0")
        if self.reacquire_xy_assist_delay_s < 0.0:
            raise ValueError("reacquire_xy_assist_delay_s must be >= 0")
        if self.reacquire_xy_assist_delay_s < self.reacquire_yaw_assist_delay_s:
            self.get_logger().warn(
                "[follow_uav] reacquire_xy_assist_delay_s < reacquire_yaw_assist_delay_s; clamping xy delay to yaw delay."
            )
            self.reacquire_xy_assist_delay_s = self.reacquire_yaw_assist_delay_s
        self.publish_gimbal_cmd = self.controller_publish_gimbal or self.camera_lock_enable

        # Apply conservative controller-backend tuning only when values still look like
        # baseline defaults. Explicit user overrides remain untouched.
        self.controller_profile_applied = []
        if self.uav_backend == "controller" and self.controller_profile_enable:
            if self.smooth_alpha >= 0.999 and self.controller_profile_smooth_alpha < self.smooth_alpha:
                self.smooth_alpha = self.controller_profile_smooth_alpha
                self.controller_profile_applied.append("smooth_alpha")
            if self.max_step_m_per_tick >= 0.5 and self.controller_profile_max_step_m_per_tick < self.max_step_m_per_tick:
                self.max_step_m_per_tick = self.controller_profile_max_step_m_per_tick
                self.controller_profile_applied.append("max_step_m_per_tick")
            if self.cmd_xy_deadband_m <= 1e-9 and self.controller_profile_cmd_xy_deadband_m > self.cmd_xy_deadband_m:
                self.cmd_xy_deadband_m = self.controller_profile_cmd_xy_deadband_m
                self.controller_profile_applied.append("cmd_xy_deadband_m")
            if self.yaw_deadband_rad <= 0.050001 and self.controller_profile_yaw_deadband_rad > self.yaw_deadband_rad:
                self.yaw_deadband_rad = self.controller_profile_yaw_deadband_rad
                self.controller_profile_applied.append("yaw_deadband_rad")
            if self.min_cmd_period_s <= 0.100001 and self.controller_profile_min_cmd_period_s > self.min_cmd_period_s:
                self.min_cmd_period_s = self.controller_profile_min_cmd_period_s
                self.controller_profile_applied.append("min_cmd_period_s")

        # ---------- State ----------
        self.have_ugv = False
        self.ugv_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_ugv_stamp: Optional[Time] = None
        self.stale_latched = False
        self.stale_predict_active = False

        # Commanded UAV pose (deterministic internal state)
        self.uav_cmd = Pose2D(0.0, 0.0, 0.0)
        self.have_uav_cmd = False
        if self.uav_backend == "controller":
            # Seed internal state to match simulator backend initial state.
            self.uav_cmd = Pose2D(
                self.controller_seed_x,
                self.controller_seed_y,
                math.radians(self.controller_seed_yaw_deg),
            )
            self.have_uav_cmd = True

        self.last_cmd_time: Optional[Time] = None
        self.pending_futures = []
        self.camera_setpose_failed_latched = False
        self.camera_setpose_failures_total = 0
        self.camera_setpose_failures_consecutive = 0
        self.camera_setpose_failover_latched = False
        self.camera_setpose_runtime_enabled = (self.camera_pose_control_mode != "gimbal_only")
        self.startup_seed_sent = False
        self.last_leader_status_fields = {}
        self.last_leader_status_rx: Optional[Time] = None
        self.sel_visible_streak = 0
        self.reacquire_first_lock_achieved = not self.reacquire_first_lock_gate_enable
        self.quality_hold_latched = False
        self.camera_pan_cmd_deg = 0.0
        self.camera_tilt_cmd_deg = max(self.camera_lock_tilt_min_deg, min(self.camera_lock_tilt_max_deg, self.camera_pitch_deg))
        self.camera_lock_latched = False
        self.camera_lock_search_latched = False
        self.camera_lock_search_dir = 1.0
        self.camera_lock_last_good_time: Optional[Time] = None
        self.camera_lock_last_search_update: Optional[Time] = None
        self.follow_state = "INIT"
        self._desired_follow_state = "INIT"
        self._state_good_ticks = 0
        self._state_bad_ticks = 0
        self.track_only_hold_latched = False
        self.xy_motion_hold_latched = False
        self.under_target_guard_latched = False
        self.reacquire_seq_active = False
        self.reacquire_seq_phase = "IDLE"
        self.reacquire_seq_phase_start: Optional[Time] = None
        self.reacquire_seq_last_update: Optional[Time] = None
        self.reacquire_seq_pan_center_deg = 0.0
        self.reacquire_seq_tilt_center_deg = 0.0
        self.reacquire_seq_pan_dir = 1.0
        self.reacquire_seq_tilt_dir = 1.0
        self.reacquire_seq_yaw_dir = 1.0
        self.reacquire_seq_yaw_cmd: Optional[float] = None
        self.reacquire_seq_started_at: Optional[Time] = None
        self.reacquire_seq_fallback_hold = False
        self.reacquire_seq_allow_yaw_assist = False
        self.reacquire_seq_allow_xy_assist = False
        self.reacquire_commit_until: Optional[Time] = None
        self.reacquire_commit_latched = False
        self.traj_rel_target_ema: Optional[Tuple[float, float]] = None
        self.traj_rel_vx = 0.0
        self.traj_rel_vy = 0.0
        self.traj_last_leader_yaw: Optional[float] = None
        self.leader_motion_prev_xy: Optional[Tuple[float, float]] = None
        self.leader_motion_prev_stamp: Optional[Time] = None
        self.leader_motion_vx = 0.0
        self.leader_motion_vy = 0.0
        self.leader_motion_speed_mps = 0.0
        self.leader_motion_heading_yaw: Optional[float] = None

        # ---------- ROS I/O ----------
        if self.leader_input_type == "odom":
            self.leader_sub = self.create_subscription(
                Odometry,
                self.leader_odom_topic,
                self.on_leader_odom,
                10,
            )
            leader_desc = f"odom:{self.leader_odom_topic}"
        else:
            self.leader_sub = self.create_subscription(
                PoseStamped,
                self.leader_pose_topic,
                self.on_leader_pose,
                10,
            )
            leader_desc = f"pose:{self.leader_pose_topic}"
        self.leader_status_sub = self.create_subscription(
            String,
            self.leader_status_topic,
            self.on_leader_status,
            10,
        )

        self.pose_pub = self.create_publisher(
            PoseStamped,
            f"/{self.uav_name}/pose_cmd",
            10,
        )
        self.pose_odom_pub = self.create_publisher(
            Odometry,
            f"/{self.uav_name}/pose_cmd/odom",
            10,
        )
        self.controller_cmd_pub = None
        self.controller_pan_pub = None
        self.controller_tilt_pub = None
        if self.uav_backend == "controller":
            self.controller_cmd_pub = self.create_publisher(
                Joy,
                self.controller_cmd_topic,
                10,
            )
        if self.publish_gimbal_cmd:
            # Keep pan/tilt topics available in both backends so estimator can
            # consume pan for camera-geometry compensation consistently.
            self.controller_pan_pub = self.create_publisher(Float64, self.controller_pan_topic, 10)
            self.controller_tilt_pub = self.create_publisher(Float64, self.controller_tilt_topic, 10)

        self.events_pub = self.create_publisher(String, self.event_topic, 10)
        self.debug_status_pub = self.create_publisher(String, self.debug_status_topic, 10) if self.publish_debug_status else None
        # Metrics (for offline evaluation)
        self.declare_parameter("publish_metrics", True)
        self.declare_parameter("metrics_prefix", "/coord")

        self.publish_metrics = bool(self.get_parameter("publish_metrics").value)
        self.metrics_prefix = str(self.get_parameter("metrics_prefix").value).rstrip("/")
        if self.metrics_prefix == "":
            self.metrics_prefix = "/coord"

        self.metric_dist_pub = self.create_publisher(
            Float32, f"{self.metrics_prefix}/follow_dist_cmd", 10
        )
        self.metric_err_pub = self.create_publisher(
            Float32, f"{self.metrics_prefix}/follow_tracking_error_cmd", 10
        )

        self.cli = None
        if self.uav_backend == "setpose":
            srv_name = f"/world/{self.world}/set_pose"
            self.cli = self.create_client(SetEntityPose, srv_name)
            self.get_logger().info(f"[follow_uav] Waiting for service: {srv_name}")
            while not self.cli.wait_for_service(timeout_sec=1.0):
                self.get_logger().info("[follow_uav] Service not available, waiting again...")
            # In observation-only runs, estimator may need a valid commanded UAV pose
            # and camera orientation before any leader estimate exists.
            seed_yaw = math.radians(self.controller_seed_yaw_deg)
            self.uav_cmd = Pose2D(self.controller_seed_x, self.controller_seed_y, seed_yaw)
            self.have_uav_cmd = True
            self.pending_futures = [
                self.set_entity_pose_async(self.uav_name, self.uav_cmd.x, self.uav_cmd.y, self.z_alt, self.uav_cmd.yaw),
            ]
            if self._camera_setpose_enabled():
                self.pending_futures.append(
                    self.set_camera_pose_async(
                        self.uav_cmd.x,
                        self.uav_cmd.y,
                        self.z_alt,
                        self.uav_cmd.yaw,
                        self.camera_pan_cmd_deg,
                        self.camera_tilt_cmd_deg,
                    )
                )
            if self.publish_gimbal_cmd and self.controller_pan_pub is not None and self.controller_tilt_pub is not None:
                pan_msg = Float64()
                pan_msg.data = float(self.camera_pan_cmd_deg)
                self.controller_pan_pub.publish(pan_msg)
                tilt_msg = Float64()
                tilt_msg.data = float(self.camera_tilt_cmd_deg)
                self.controller_tilt_pub.publish(tilt_msg)
            self.publish_pose_cmd(self.uav_cmd.x, self.uav_cmd.y, self.z_alt, self.uav_cmd.yaw)
            self.publish_pose_cmd_odom(self.uav_cmd.x, self.uav_cmd.y, self.z_alt, self.uav_cmd.yaw)
            self.last_cmd_time = self.get_clock().now()
            self.startup_seed_sent = True
            self.emit_event("FOLLOW_STARTUP_SEED_SETPOSE")

        dt = 1.0 / self.tick_hz
        self.timer = self.create_timer(dt, self.on_tick)

        self.get_logger().info(
            f"[follow_uav] Started: world={self.world}, uav={self.uav_name}, backend={self.uav_backend}, "
            f"camera_model={self.camera_model_name}, camera_pitch_deg={self.camera_pitch_deg}, "
            f"camera_z_offset_m={self.camera_z_offset_m}, "
            f"camera_lock_enable={self.camera_lock_enable}, camera_lock_gain={self.camera_lock_gain}, "
            f"camera_lock_deadband_deg={self.camera_lock_deadband_deg}, camera_lock_pan_limit_deg={self.camera_lock_pan_limit_deg}, "
            f"camera_lock_max_step_deg={self.camera_lock_max_step_deg}, camera_lock_hold_on_bad_state={self.camera_lock_hold_on_bad_state}, "
            f"camera_lock_tilt_enable={self.camera_lock_tilt_enable}, camera_lock_tilt_gain_per_px={self.camera_lock_tilt_gain_per_px}, "
            f"camera_lock_tilt_deadband_px={self.camera_lock_tilt_deadband_px}, "
            f"camera_lock_tilt_range=[{self.camera_lock_tilt_min_deg},{self.camera_lock_tilt_max_deg}], "
            f"camera_lock_tilt_max_step_deg={self.camera_lock_tilt_max_step_deg}, "
            f"camera_lock_tilt_hold_on_bad_state={self.camera_lock_tilt_hold_on_bad_state}, "
            f"under_target_guard_enable={self.under_target_guard_enable}, "
            f"under_target_err_v_ratio={self.under_target_err_v_ratio}, under_target_range_m={self.under_target_range_m}, "
            f"under_target_hold_xy={self.under_target_hold_xy}, under_target_hold_yaw={self.under_target_hold_yaw}, "
            f"reacquire_sequence_enable={self.reacquire_sequence_enable}, "
            f"reacquire_pan_phase_s={self.reacquire_pan_phase_s}, reacquire_tilt_phase_s={self.reacquire_tilt_phase_s}, "
            f"reacquire_yaw_phase_s={self.reacquire_yaw_phase_s}, reacquire_pan_local_span_deg={self.reacquire_pan_local_span_deg}, "
            f"reacquire_tilt_local_span_deg={self.reacquire_tilt_local_span_deg}, "
            f"reacquire_pan_rate_deg_s={self.reacquire_pan_rate_deg_s}, reacquire_tilt_rate_deg_s={self.reacquire_tilt_rate_deg_s}, "
            f"reacquire_yaw_rate_deg_s={self.reacquire_yaw_rate_deg_s}, "
            f"reacquire_hold_phase_s={self.reacquire_hold_phase_s}, reacquire_fallback_hold_s={self.reacquire_fallback_hold_s}, "
            f"reacquire_hold_after_search={self.reacquire_hold_after_search}, reacquire_search_hold_xy={self.reacquire_search_hold_xy}, "
            f"reacquire_search_hold_yaw_before_yaw_phase={self.reacquire_search_hold_yaw_before_yaw_phase}, "
            f"reacquire_commit_enable={self.reacquire_commit_enable}, reacquire_commit_s={self.reacquire_commit_s}, "
            f"reacquire_commit_quality_floor={self.reacquire_commit_quality_floor}, "
            f"reacquire_startup_guard_enable={self.reacquire_startup_guard_enable}, "
            f"reacquire_first_lock_gate_enable={self.reacquire_first_lock_gate_enable}, "
            f"reacquire_first_lock_min_visible_frames={self.reacquire_first_lock_min_visible_frames}, "
            f"reacquire_relock_min_visible_frames={self.reacquire_relock_min_visible_frames}, "
            f"reacquire_yaw_assist_delay_s={self.reacquire_yaw_assist_delay_s}, "
            f"reacquire_xy_assist_delay_s={self.reacquire_xy_assist_delay_s}, "
            f"publish_gimbal_cmd={self.publish_gimbal_cmd}, "
            f"camera_pose_control_mode={self.camera_pose_control_mode}, "
            f"camera_pose_failover_failures={self.camera_pose_failover_failures}, "
            f"camera_setpose_enabled_initial={self._camera_setpose_enabled()}, "
            f"leader_input={leader_desc}, tick={self.tick_hz}Hz, "
            f"d_target={self.d_target}, d_max={self.d_max}, z={self.z_alt}, "
            f"pose_timeout_s={self.pose_timeout_s}, min_cmd_period_s={self.min_cmd_period_s}, "
            f"smooth_alpha={self.smooth_alpha}, max_step_m_per_tick={self.max_step_m_per_tick}, "
            f"cmd_xy_deadband_m={self.cmd_xy_deadband_m}, max_yaw_step_rad_per_tick={self.max_yaw_step_rad_per_tick}, "
            f"yaw_deadband_rad={self.yaw_deadband_rad}, yaw_update_xy_gate_m={self.yaw_update_xy_gate_m}, "
            f"quality_scale_enable={self.quality_scale_enable}, leader_status_topic={self.leader_status_topic}, "
            f"quality_status_timeout_s={self.quality_status_timeout_s}, quality_conf=[{self.quality_conf_min},{self.quality_conf_good}], "
            f"quality_latency_ref_ms={self.quality_latency_ref_ms}, quality_min_step_scale={self.quality_min_step_scale}, "
            f"quality_hold_step_scale={self.quality_hold_step_scale}, quality_deadband_boost_m={self.quality_deadband_boost_m}, "
            f"state_machine_enable={self.state_machine_enable}, state_debounce=({self.state_ok_debounce_ticks},{self.state_bad_debounce_ticks}), "
            f"track_only_motion_enable={self.track_only_motion_enable}, "
            f"track_only_allow_reacquire_motion={self.track_only_allow_reacquire_motion}, "
            f"xy_motion_enable={self.xy_motion_enable}, "
            f"track_only_hold_yaw_enable={self.track_only_hold_yaw_enable}, "
            f"traj_enable={self.traj_enable}, traj_rel_frame_enable={self.traj_rel_frame_enable}, "
            f"traj_rel_smooth_alpha={self.traj_rel_smooth_alpha}, traj_pos_gain={self.traj_pos_gain}, "
            f"traj_max_speed_mps={self.traj_max_speed_mps}, traj_max_accel_mps2={self.traj_max_accel_mps2}, "
            f"traj_quality_min_scale={self.traj_quality_min_scale}, traj_reset_on_yaw_jump_rad={self.traj_reset_on_yaw_jump_rad}, "
            f"estimate_heading_from_motion_enable={self.estimate_heading_from_motion_enable}, "
            f"estimate_heading_alpha={self.estimate_heading_alpha}, estimate_heading_min_speed_mps={self.estimate_heading_min_speed_mps}, "
            f"estimate_heading_max_dt_s={self.estimate_heading_max_dt_s}, "
            f"stale_predict_enable={self.stale_predict_enable}, stale_predict_timeout_s={self.stale_predict_timeout_s}, "
            f"stale_predict_quality_floor={self.stale_predict_quality_floor}, "
            f"controller_cmd_topic={self.controller_cmd_topic}, controller_pan_topic={self.controller_pan_topic}, "
            f"controller_tilt_topic={self.controller_tilt_topic}, controller_seed=({self.controller_seed_x:.2f},{self.controller_seed_y:.2f},{self.controller_seed_yaw_deg:.1f}deg), "
            f"controller_profile_applied={','.join(self.controller_profile_applied) if self.controller_profile_applied else 'none'}, "
            f"event_topic={self.event_topic}, "
            f"debug_status_topic={self.debug_status_topic if self.debug_status_pub is not None else '<disabled>'}"
        )
        self.emit_event("FOLLOW_NODE_START")
        self.emit_event(f"FOLLOW_BACKEND:{self.uav_backend}")
        if self._camera_setpose_enabled():
            self.emit_event("CAMERA_CONTROL_PATH:separate_model")
        else:
            self.emit_event("CAMERA_CONTROL_PATH:gimbal_only")
        if self.controller_profile_applied:
            self.emit_event(f"FOLLOW_CONTROLLER_PROFILE_APPLIED:{','.join(self.controller_profile_applied)}")

    def emit_event(self, s: str):
        if not self.publish_events:
            return
        msg = String()
        msg.data = s
        self.events_pub.publish(msg)

    def on_leader_status(self, msg: String):
        fields = {}
        for tok in msg.data.split():
            if "=" not in tok:
                continue
            k, v = tok.split("=", 1)
            fields[k.strip()] = v.strip()
        self.last_leader_status_fields = fields
        self.last_leader_status_rx = self.get_clock().now()
        sel_visible = self._parse_status_bool(fields.get("sel_visible"))
        if sel_visible is True:
            self.sel_visible_streak += 1
            if (
                self.reacquire_first_lock_gate_enable
                and (not self.reacquire_first_lock_achieved)
                and self.sel_visible_streak >= self.reacquire_first_lock_min_visible_frames
            ):
                self.reacquire_first_lock_achieved = True
                self.emit_event("REACQUIRE_FIRST_LOCK_READY")
        else:
            self.sel_visible_streak = 0

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

    def _parse_status_bool(self, raw: Optional[str]) -> Optional[bool]:
        if raw is None:
            return None
        rv = str(raw).strip().lower()
        if rv in ("1", "true", "yes", "on"):
            return True
        if rv in ("0", "false", "no", "off"):
            return False
        return None

    def _leader_status_sel_visible(self) -> Optional[bool]:
        return self._parse_status_bool(self.last_leader_status_fields.get("sel_visible"))

    def _leader_target_visible(self, now: Time) -> bool:
        if self.last_leader_status_rx is None:
            return False
        if self.quality_status_timeout_s > 0.0:
            age_s = (now - self.last_leader_status_rx).nanoseconds * 1e-9
            if age_s > self.quality_status_timeout_s:
                return False
        sel_visible = self._leader_status_sel_visible()
        if sel_visible is not None:
            return sel_visible

        st = self._leader_status_state()
        return st not in {
            "STALE",
            "DECODE_FAIL",
            "NO_DET",
            "YOLO_DISABLED",
            "HOLD",
            "DEBOUNCE_HOLD",
            "REJECT",
            "REJECT_HOLD",
            "REJECT_DEBOUNCE_HOLD",
            "waiting_for_image",
            "waiting_for_camera_info",
            "waiting_for_uav_pose",
            "stale_image",
        }

    def _start_reacquire_commit(self, now: Time, reason: str) -> None:
        if not self.reacquire_commit_enable or self.reacquire_commit_s <= 0.0:
            return
        self.reacquire_commit_until = now + Duration(seconds=self.reacquire_commit_s)
        if not self.reacquire_commit_latched:
            self.emit_event(f"REACQUIRE_COMMIT_ENTER:{reason}")
            self.reacquire_commit_latched = True

    def _reacquire_commit_active(self, now: Time) -> bool:
        if not self.reacquire_commit_enable or self.reacquire_commit_s <= 0.0:
            if self.reacquire_commit_latched:
                self.emit_event("REACQUIRE_COMMIT_EXIT:disabled")
                self.reacquire_commit_latched = False
            self.reacquire_commit_until = None
            return False
        if self.reacquire_commit_until is None:
            return False
        if now <= self.reacquire_commit_until:
            return True
        self.reacquire_commit_until = None
        if self.reacquire_commit_latched:
            self.emit_event("REACQUIRE_COMMIT_EXIT:timeout")
            self.reacquire_commit_latched = False
        return False

    def _quality_scale_from_status(self, now: Time) -> Tuple[float, str]:
        if not self.quality_scale_enable or self.leader_input_type != "pose":
            return 1.0, "disabled"
        if self.last_leader_status_rx is None:
            return 1.0, "no_status"
        if self.quality_status_timeout_s > 0.0:
            age_s = (now - self.last_leader_status_rx).nanoseconds * 1e-9
            if age_s > self.quality_status_timeout_s:
                return 0.0, "status_stale"

        state = self.last_leader_status_fields.get("state", "")
        sel_visible = self._leader_status_sel_visible()
        commit_active = self._reacquire_commit_active(now)
        if sel_visible is True:
            self._start_reacquire_commit(now, "sel_visible")
            commit_active = self._reacquire_commit_active(now)
        conf = self._status_float("conf", -1.0)
        latency_ms = self._status_float("latency_ms", -1.0)

        hard_bad_states = {
            "STALE", "DECODE_FAIL", "NO_DET", "YOLO_DISABLED", "REJECT", "waiting_for_image",
            "waiting_for_camera_info", "waiting_for_uav_pose", "stale_image"
        }
        soft_states = {"HOLD", "DEBOUNCE_HOLD", "REJECT_HOLD", "REJECT_DEBOUNCE_HOLD", "REACQUIRE"}

        # sel_visible=0 means true loss for lock/search decisions even if periodic
        # status-heartbeat messages report state=running.
        if sel_visible is False:
            if commit_active:
                return max(self.reacquire_commit_quality_floor, self.quality_min_step_scale), "commit_grace"
            return 0.0, "sel_visible:0"

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
        if commit_active:
            q = max(q, self.reacquire_commit_quality_floor)
        if q > 0.0:
            q = max(self.quality_min_step_scale, q)
        q = max(0.0, min(1.0, q))
        return q, (state if state else "ok")

    def _leader_status_state(self) -> str:
        return str(self.last_leader_status_fields.get("state", "")).strip()

    def _leader_status_is_bad_for_camera_lock(self) -> bool:
        sel_visible = self._leader_status_sel_visible()
        if sel_visible is False:
            return True
        st = self._leader_status_state()
        return st in {
            "STALE",
            "DECODE_FAIL",
            "NO_DET",
            "YOLO_DISABLED",
            "HOLD",
            "DEBOUNCE_HOLD",
            "REJECT",
            "REJECT_HOLD",
            "REJECT_DEBOUNCE_HOLD",
            "waiting_for_image",
            "waiting_for_camera_info",
            "waiting_for_uav_pose",
            "stale_image",
        }

    def _camera_lock_search_step(self, now: Time) -> float:
        if not self.camera_lock_search_enable or self.camera_lock_search_rate_deg_s <= 0.0:
            return self.camera_pan_cmd_deg
        if self.camera_lock_last_search_update is None:
            self.camera_lock_last_search_update = now
            return self.camera_pan_cmd_deg

        dt = (now - self.camera_lock_last_search_update).nanoseconds * 1e-9
        if not math.isfinite(dt) or dt <= 0.0:
            dt = 1.0 / max(1.0, self.tick_hz)
        if dt > 0.5:
            dt = 0.5
        self.camera_lock_last_search_update = now

        step = self.camera_lock_search_rate_deg_s * dt * self.camera_lock_search_dir
        new_pan = self.camera_pan_cmd_deg + step
        hit_limit = False
        if new_pan > self.camera_lock_pan_limit_deg:
            new_pan = self.camera_lock_pan_limit_deg
            hit_limit = True
        elif new_pan < -self.camera_lock_pan_limit_deg:
            new_pan = -self.camera_lock_pan_limit_deg
            hit_limit = True
        self.camera_pan_cmd_deg = new_pan
        if hit_limit:
            self.camera_lock_search_dir *= -1.0
        return self.camera_pan_cmd_deg

    def _update_camera_pan_lock(self, now: Time) -> float:
        if not self.camera_lock_enable:
            if self.camera_lock_latched:
                self.emit_event("CAMERA_LOCK_EXIT")
                self.camera_lock_latched = False
            if self.camera_lock_search_latched:
                self.emit_event("CAMERA_LOCK_SEARCH_EXIT")
                self.camera_lock_search_latched = False
            self.camera_pan_cmd_deg = 0.0
            return 0.0

        use_builtin_search = self.camera_lock_search_enable and not (
            self.reacquire_sequence_enable and self.leader_input_type == "pose"
        )

        if self.last_leader_status_rx is None:
            if use_builtin_search:
                if not self.camera_lock_search_latched:
                    self.emit_event("CAMERA_LOCK_SEARCH_ENTER")
                    self.camera_lock_search_latched = True
                return self._camera_lock_search_step(now)
            return self.camera_pan_cmd_deg
        age_s = (now - self.last_leader_status_rx).nanoseconds * 1e-9
        if self.quality_status_timeout_s > 0.0 and age_s > self.quality_status_timeout_s:
            if self.camera_lock_latched:
                self.emit_event("CAMERA_LOCK_EXIT")
                self.camera_lock_latched = False
            if use_builtin_search:
                if not self.camera_lock_search_latched:
                    self.emit_event("CAMERA_LOCK_SEARCH_ENTER")
                    self.camera_lock_search_latched = True
                return self._camera_lock_search_step(now)
            if self.camera_lock_hold_on_bad_state:
                return self.camera_pan_cmd_deg
            self.camera_pan_cmd_deg = 0.0
            return 0.0

        if self._leader_status_is_bad_for_camera_lock():
            if self.camera_lock_latched:
                self.emit_event("CAMERA_LOCK_EXIT")
                self.camera_lock_latched = False
            if use_builtin_search:
                if self.camera_lock_last_good_time is None:
                    if not self.camera_lock_search_latched:
                        self.emit_event("CAMERA_LOCK_SEARCH_ENTER")
                        self.camera_lock_search_latched = True
                    return self._camera_lock_search_step(now)
                bad_age = (now - self.camera_lock_last_good_time).nanoseconds * 1e-9
                if bad_age >= self.camera_lock_search_hold_s:
                    if not self.camera_lock_search_latched:
                        self.emit_event("CAMERA_LOCK_SEARCH_ENTER")
                        self.camera_lock_search_latched = True
                    return self._camera_lock_search_step(now)
            if self.camera_lock_hold_on_bad_state:
                return self.camera_pan_cmd_deg
            self.camera_pan_cmd_deg = 0.0
            return 0.0

        self.camera_lock_last_good_time = now
        self.camera_lock_last_search_update = now
        if self.camera_lock_search_latched:
            self.emit_event("CAMERA_LOCK_SEARCH_EXIT")
            self.camera_lock_search_latched = False

        if self.camera_lock_pan_reset_on_reacquire and self._leader_status_state() == "REACQUIRE":
            self.camera_pan_cmd_deg = 0.0
            return 0.0

        # Camera lock should use image-space bearing error, not absolute world bearing.
        bearing_deg = self._status_float("bearing_img_deg", float("nan"))
        if not math.isfinite(bearing_deg):
            bearing_deg = self._status_float("bearing_deg", 0.0)
        if not math.isfinite(bearing_deg):
            return self.camera_pan_cmd_deg
        if abs(bearing_deg) <= self.camera_lock_deadband_deg:
            if self.camera_lock_latched:
                self.emit_event("CAMERA_LOCK_EXIT")
                self.camera_lock_latched = False
            return self.camera_pan_cmd_deg

        if not self.camera_lock_latched:
            self.emit_event("CAMERA_LOCK_ENTER")
            self.camera_lock_latched = True
        desired = self.camera_pan_cmd_deg + self.camera_lock_gain * bearing_deg
        desired = max(-self.camera_lock_pan_limit_deg, min(self.camera_lock_pan_limit_deg, desired))
        delta = desired - self.camera_pan_cmd_deg
        if self.camera_lock_max_step_deg > 0.0:
            delta = max(-self.camera_lock_max_step_deg, min(self.camera_lock_max_step_deg, delta))
        self.camera_pan_cmd_deg += delta
        self.camera_pan_cmd_deg = max(-self.camera_lock_pan_limit_deg, min(self.camera_lock_pan_limit_deg, self.camera_pan_cmd_deg))
        return self.camera_pan_cmd_deg

    def _reacquire_seq_set_phase(self, now: Time, phase: str) -> None:
        if self.reacquire_seq_phase == phase:
            return
        self.reacquire_seq_phase = phase
        self.reacquire_seq_phase_start = now
        self.emit_event(f"REACQUIRE_SEQ_PHASE:{phase}")

    def _reacquire_seq_reset(self, reason: str = "") -> None:
        if self.reacquire_seq_fallback_hold:
            self.emit_event("REACQUIRE_SEQ_FALLBACK_HOLD_EXIT")
        if self.reacquire_seq_active:
            suffix = f":{reason}" if reason else ""
            self.emit_event(f"REACQUIRE_SEQ_EXIT{suffix}")
        self.reacquire_seq_active = False
        self.reacquire_seq_phase = "IDLE"
        self.reacquire_seq_phase_start = None
        self.reacquire_seq_last_update = None
        self.reacquire_seq_pan_center_deg = self.camera_pan_cmd_deg
        self.reacquire_seq_tilt_center_deg = self.camera_tilt_cmd_deg
        self.reacquire_seq_pan_dir = 1.0
        self.reacquire_seq_tilt_dir = 1.0
        self.reacquire_seq_yaw_dir = 1.0
        self.reacquire_seq_yaw_cmd = None
        self.reacquire_seq_started_at = None
        self.reacquire_seq_fallback_hold = False
        self.reacquire_seq_allow_yaw_assist = False
        self.reacquire_seq_allow_xy_assist = False

    def _update_reacquire_sequence(self, now: Time, yaw_cmd: float) -> Tuple[float, float, float, bool, bool, str]:
        if not self.camera_lock_enable:
            self._reacquire_seq_reset("camera_lock_disabled")
            return self.camera_pan_cmd_deg, self.camera_tilt_cmd_deg, yaw_cmd, False, False, "IDLE"
        if not (self.reacquire_sequence_enable and self.leader_input_type == "pose"):
            self._reacquire_seq_reset("disabled")
            return self.camera_pan_cmd_deg, self.camera_tilt_cmd_deg, yaw_cmd, False, False, "IDLE"
        if self.reacquire_startup_guard_enable and self.last_leader_status_rx is None:
            self._reacquire_seq_reset("startup_guard")
            return self.camera_pan_cmd_deg, self.camera_tilt_cmd_deg, yaw_cmd, False, False, "IDLE"
        if self.reacquire_first_lock_gate_enable and (not self.reacquire_first_lock_achieved):
            self._reacquire_seq_reset("first_lock_gate")
            return self.camera_pan_cmd_deg, self.camera_tilt_cmd_deg, yaw_cmd, False, False, "IDLE"

        if self._leader_target_visible(now):
            self._reacquire_seq_reset("visible")
            return self.camera_pan_cmd_deg, self.camera_tilt_cmd_deg, yaw_cmd, False, False, "IDLE"
        if self._reacquire_commit_active(now):
            # Short grace window after a visibility hit to avoid immediately
            # re-entering search on a single-frame dropout.
            self._reacquire_seq_reset("commit_grace")
            return self.camera_pan_cmd_deg, self.camera_tilt_cmd_deg, yaw_cmd, False, False, "IDLE"

        if not self.reacquire_seq_active:
            self.reacquire_seq_active = True
            self.reacquire_seq_last_update = now
            self.reacquire_seq_pan_center_deg = self.camera_pan_cmd_deg
            self.reacquire_seq_tilt_center_deg = self.camera_tilt_cmd_deg
            self.reacquire_seq_pan_dir = 1.0
            self.reacquire_seq_tilt_dir = 1.0
            self.reacquire_seq_yaw_dir = 1.0
            self.reacquire_seq_yaw_cmd = yaw_cmd
            self.reacquire_seq_started_at = now
            self.reacquire_seq_fallback_hold = False
            self.emit_event("REACQUIRE_SEQ_ENTER")
            self._reacquire_seq_set_phase(now, "PAN")

        if self.reacquire_seq_last_update is None:
            dt = 1.0 / max(1.0, self.tick_hz)
        else:
            dt = (now - self.reacquire_seq_last_update).nanoseconds * 1e-9
            if not math.isfinite(dt) or dt <= 0.0:
                dt = 1.0 / max(1.0, self.tick_hz)
            if dt > 0.5:
                dt = 0.5
        self.reacquire_seq_last_update = now
        phase_start = self.reacquire_seq_phase_start if self.reacquire_seq_phase_start is not None else now
        phase_age = (now - phase_start).nanoseconds * 1e-9
        seq_start = self.reacquire_seq_started_at if self.reacquire_seq_started_at is not None else now
        seq_age = (now - seq_start).nanoseconds * 1e-9
        if not math.isfinite(seq_age) or seq_age < 0.0:
            seq_age = 0.0
        self.reacquire_seq_allow_yaw_assist = seq_age >= self.reacquire_yaw_assist_delay_s
        self.reacquire_seq_allow_xy_assist = seq_age >= self.reacquire_xy_assist_delay_s

        phase = self.reacquire_seq_phase
        hold_xy = True
        hold_yaw = True

        if phase == "PAN":
            span = min(self.camera_lock_pan_limit_deg, max(0.0, self.reacquire_pan_local_span_deg))
            lo = max(-self.camera_lock_pan_limit_deg, self.reacquire_seq_pan_center_deg - span)
            hi = min(self.camera_lock_pan_limit_deg, self.reacquire_seq_pan_center_deg + span)
            step = self.reacquire_pan_rate_deg_s * dt * self.reacquire_seq_pan_dir
            new_pan = self.camera_pan_cmd_deg + step
            if new_pan > hi:
                new_pan = hi
                self.reacquire_seq_pan_dir *= -1.0
            elif new_pan < lo:
                new_pan = lo
                self.reacquire_seq_pan_dir *= -1.0
            self.camera_pan_cmd_deg = new_pan
            if self.reacquire_pan_phase_s > 0.0 and phase_age >= self.reacquire_pan_phase_s:
                self._reacquire_seq_set_phase(now, "TILT")
                self.reacquire_seq_tilt_center_deg = self.camera_tilt_cmd_deg
                phase = self.reacquire_seq_phase

        if phase == "TILT":
            # Keep pan near search-center while sweeping tilt to recover vertical FOV loss.
            pan_err = self.reacquire_seq_pan_center_deg - self.camera_pan_cmd_deg
            pan_step = self.reacquire_pan_rate_deg_s * dt
            pan_step = max(0.2, pan_step)
            self.camera_pan_cmd_deg += max(-pan_step, min(pan_step, pan_err))
            self.camera_pan_cmd_deg = max(-self.camera_lock_pan_limit_deg, min(self.camera_lock_pan_limit_deg, self.camera_pan_cmd_deg))

            # Keep tilt sweep local around loss-time tilt to avoid large unnecessary vertical swings.
            span = max(0.0, self.reacquire_tilt_local_span_deg)
            lo_tilt = max(self.camera_lock_tilt_min_deg, self.reacquire_seq_tilt_center_deg - span)
            hi_tilt = min(self.camera_lock_tilt_max_deg, self.reacquire_seq_tilt_center_deg + span)
            if hi_tilt < lo_tilt:
                hi_tilt = lo_tilt
            tilt_step = self.reacquire_tilt_rate_deg_s * dt * self.reacquire_seq_tilt_dir
            new_tilt = self.camera_tilt_cmd_deg + tilt_step
            if new_tilt > hi_tilt:
                new_tilt = hi_tilt
                self.reacquire_seq_tilt_dir *= -1.0
            elif new_tilt < lo_tilt:
                new_tilt = lo_tilt
                self.reacquire_seq_tilt_dir *= -1.0
            self.camera_tilt_cmd_deg = new_tilt
            if self.reacquire_tilt_phase_s > 0.0 and phase_age >= self.reacquire_tilt_phase_s:
                if (
                    self.reacquire_seq_allow_yaw_assist
                    and self.reacquire_yaw_phase_s > 0.0
                    and self.reacquire_yaw_rate_deg_s > 0.0
                ):
                    self._reacquire_seq_set_phase(now, "YAW")
                    phase = self.reacquire_seq_phase
                elif self.reacquire_hold_after_search:
                    self._reacquire_seq_set_phase(now, "HOLD")
                    phase = self.reacquire_seq_phase
                else:
                    self._reacquire_seq_set_phase(now, "PAN")
                    phase = self.reacquire_seq_phase

        if phase == "YAW":
            hold_yaw = not self.reacquire_seq_allow_yaw_assist
            # Full-range pan sweep while yaw rotates to cover 360-like search.
            step = self.reacquire_pan_rate_deg_s * dt * self.reacquire_seq_pan_dir
            new_pan = self.camera_pan_cmd_deg + step
            if new_pan > self.camera_lock_pan_limit_deg:
                new_pan = self.camera_lock_pan_limit_deg
                self.reacquire_seq_pan_dir *= -1.0
            elif new_pan < -self.camera_lock_pan_limit_deg:
                new_pan = -self.camera_lock_pan_limit_deg
                self.reacquire_seq_pan_dir *= -1.0
            self.camera_pan_cmd_deg = new_pan

            # Bias tilt toward baseline pitch during wide yaw search.
            tilt_target = max(self.camera_lock_tilt_min_deg, min(self.camera_lock_tilt_max_deg, self.camera_pitch_deg))
            tilt_step = max(0.2, self.reacquire_tilt_rate_deg_s * dt)
            self.camera_tilt_cmd_deg += max(-tilt_step, min(tilt_step, tilt_target - self.camera_tilt_cmd_deg))
            self.camera_tilt_cmd_deg = max(self.camera_lock_tilt_min_deg, min(self.camera_lock_tilt_max_deg, self.camera_tilt_cmd_deg))

            if self.reacquire_seq_yaw_cmd is None:
                self.reacquire_seq_yaw_cmd = yaw_cmd
            if self.reacquire_seq_allow_yaw_assist:
                dyaw = math.radians(self.reacquire_yaw_rate_deg_s) * dt * self.reacquire_seq_yaw_dir
                self.reacquire_seq_yaw_cmd = wrap_pi(self.reacquire_seq_yaw_cmd + dyaw)
                yaw_cmd = self.reacquire_seq_yaw_cmd

            if self.reacquire_yaw_phase_s > 0.0 and phase_age >= self.reacquire_yaw_phase_s:
                if self.reacquire_hold_after_search:
                    self._reacquire_seq_set_phase(now, "HOLD")
                    phase = self.reacquire_seq_phase
                else:
                    self._reacquire_seq_set_phase(now, "PAN")
                    phase = self.reacquire_seq_phase

        if phase == "HOLD":
            hold_xy = True
            hold_yaw = True
            if self.reacquire_hold_after_search and not self.reacquire_seq_fallback_hold:
                if self.reacquire_fallback_hold_s > 0.0 and seq_age >= self.reacquire_fallback_hold_s:
                    self.reacquire_seq_fallback_hold = True
                    self.emit_event("REACQUIRE_SEQ_FALLBACK_HOLD_ENTER")
                elif self.reacquire_hold_phase_s <= 0.0 or phase_age >= self.reacquire_hold_phase_s:
                    self._reacquire_seq_set_phase(now, "PAN")
                    phase = self.reacquire_seq_phase
        if (
            self.reacquire_seq_allow_xy_assist
            and (not self.reacquire_seq_fallback_hold)
            and phase in ("PAN", "TILT", "YAW")
        ):
            hold_xy = False

        return self.camera_pan_cmd_deg, self.camera_tilt_cmd_deg, yaw_cmd, hold_xy, hold_yaw, self.reacquire_seq_phase

    def _update_camera_tilt_lock(self, now: Time) -> float:
        if not self.camera_lock_tilt_enable:
            self.camera_tilt_cmd_deg = max(
                self.camera_lock_tilt_min_deg,
                min(self.camera_lock_tilt_max_deg, self.camera_pitch_deg),
            )
            return self.camera_tilt_cmd_deg

        if self.last_leader_status_rx is None:
            if not self.camera_lock_tilt_hold_on_bad_state:
                self.camera_tilt_cmd_deg = max(
                    self.camera_lock_tilt_min_deg,
                    min(self.camera_lock_tilt_max_deg, self.camera_pitch_deg),
                )
            return self.camera_tilt_cmd_deg

        age_s = (now - self.last_leader_status_rx).nanoseconds * 1e-9
        if self.quality_status_timeout_s > 0.0 and age_s > self.quality_status_timeout_s:
            if not self.camera_lock_tilt_hold_on_bad_state:
                self.camera_tilt_cmd_deg = max(
                    self.camera_lock_tilt_min_deg,
                    min(self.camera_lock_tilt_max_deg, self.camera_pitch_deg),
                )
            return self.camera_tilt_cmd_deg

        if self._leader_status_is_bad_for_camera_lock():
            if not self.camera_lock_tilt_hold_on_bad_state:
                self.camera_tilt_cmd_deg = max(
                    self.camera_lock_tilt_min_deg,
                    min(self.camera_lock_tilt_max_deg, self.camera_pitch_deg),
                )
            return self.camera_tilt_cmd_deg

        err_v_px = self._status_float("err_v_px", float("nan"))
        if not math.isfinite(err_v_px):
            return self.camera_tilt_cmd_deg

        if abs(err_v_px) <= self.camera_lock_tilt_deadband_px:
            return self.camera_tilt_cmd_deg

        # err_v_px > 0 means target is below image center -> tilt down (more negative deg).
        desired = self.camera_tilt_cmd_deg - self.camera_lock_tilt_gain_per_px * err_v_px
        desired = max(self.camera_lock_tilt_min_deg, min(self.camera_lock_tilt_max_deg, desired))
        delta = desired - self.camera_tilt_cmd_deg
        if self.camera_lock_tilt_max_step_deg > 0.0:
            delta = max(-self.camera_lock_tilt_max_step_deg, min(self.camera_lock_tilt_max_step_deg, delta))
        self.camera_tilt_cmd_deg += delta
        self.camera_tilt_cmd_deg = max(self.camera_lock_tilt_min_deg, min(self.camera_lock_tilt_max_deg, self.camera_tilt_cmd_deg))
        return self.camera_tilt_cmd_deg

    def _set_follow_state(self, new_state: str) -> None:
        if new_state == self.follow_state:
            return
        self.follow_state = new_state
        self.emit_event(f"FOLLOW_STATE_{new_state}")

    def _update_follow_state(self, desired_state: str) -> None:
        if not self.state_machine_enable or self.leader_input_type != "pose":
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
        if self.leader_input_type != "pose":
            return "TRACK"
        if not self.quality_scale_enable:
            return "TRACK"

        now = self.get_clock().now()
        sel_visible = self._leader_status_sel_visible()
        if sel_visible is True:
            if self.sel_visible_streak >= self.reacquire_relock_min_visible_frames:
                return "TRACK"
            return "REACQUIRE"
        if sel_visible is False and self._reacquire_commit_active(now):
            return "REACQUIRE"
        if sel_visible is False:
            return "DEGRADED"

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
        if self.leader_input_type != "pose" or not self.estimate_heading_from_motion_enable:
            return self.ugv_pose
        yaw = self.ugv_pose.yaw
        if self.leader_motion_heading_yaw is not None:
            yaw = self.leader_motion_heading_yaw
        return Pose2D(self.ugv_pose.x, self.ugv_pose.y, yaw)

    def _shape_target_trajectory(self, leader: Pose2D, xt: float, yt: float, now: Time, quality_scale: float) -> Tuple[float, float]:
        if not self.traj_enable or self.leader_input_type != "pose":
            return xt, yt
        if self.traj_max_speed_mps <= 0.0 or self.traj_pos_gain <= 0.0:
            return xt, yt
        if not self.have_uav_cmd:
            self._reset_traj_state()
            return xt, yt

        dt = 1.0 / self.tick_hz
        if self.last_cmd_time is not None:
            dt = max(1e-3, (now - self.last_cmd_time).nanoseconds * 1e-9)
        dt = min(dt, 1.0)

        if self.traj_last_leader_yaw is not None and self.traj_reset_on_yaw_jump_rad > 0.0:
            dyaw_leader = abs(wrap_pi(leader.yaw - self.traj_last_leader_yaw))
            if dyaw_leader > self.traj_reset_on_yaw_jump_rad:
                self._reset_traj_state()

        if self.traj_rel_frame_enable:
            cur_rel_x, cur_rel_y = self._to_leader_frame(leader, self.uav_cmd.x, self.uav_cmd.y)
            des_rel_x, des_rel_y = self._to_leader_frame(leader, xt, yt)
        else:
            cur_rel_x, cur_rel_y = self.uav_cmd.x, self.uav_cmd.y
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

        q_scale = 1.0
        if self.quality_scale_enable:
            q_scale = max(self.traj_quality_min_scale, quality_scale)

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

    def on_leader_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))

        self.ugv_pose = Pose2D(float(p.x), float(p.y), float(yaw))
        self.have_ugv = True

        try:
            self.last_ugv_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_ugv_stamp = self.get_clock().now()
        self._update_leader_motion_model(self.ugv_pose, self.last_ugv_stamp)

    def on_leader_pose(self, msg: PoseStamped):
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))

        self.ugv_pose = Pose2D(float(p.x), float(p.y), float(yaw))
        self.have_ugv = True

        try:
            self.last_ugv_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_ugv_stamp = self.get_clock().now()
        self._update_leader_motion_model(self.ugv_pose, self.last_ugv_stamp)



    def ugv_pose_is_fresh(self, now: Time) -> bool:
        if not self.have_ugv or self.last_ugv_stamp is None:
            return False
        age = (now - self.last_ugv_stamp).nanoseconds * 1e-9
        return age <= self.pose_timeout_s

    def _predict_leader_pose(self, now: Time) -> Optional[Pose2D]:
        # Short dropout fallback that uses only previously estimated leader motion.
        if self.leader_input_type != "pose" or not self.stale_predict_enable:
            return None
        if not self.have_ugv or self.last_ugv_stamp is None:
            return None
        age = (now - self.last_ugv_stamp).nanoseconds * 1e-9
        if not math.isfinite(age) or age <= 0.0:
            return None
        if age > self.stale_predict_timeout_s:
            return None
        if not (math.isfinite(self.leader_motion_vx) and math.isfinite(self.leader_motion_vy)):
            return None
        px = self.ugv_pose.x + self.leader_motion_vx * age
        py = self.ugv_pose.y + self.leader_motion_vy * age
        pyaw = self.leader_motion_heading_yaw if self.leader_motion_heading_yaw is not None else self.ugv_pose.yaw
        return Pose2D(px, py, pyaw)

    def can_send_command_now(self, now: Time) -> bool:
        if self.last_cmd_time is None:
            return True
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        return dt >= self.min_cmd_period_s

    def _set_entity_pose_quat_async(
        self,
        entity_name: str,
        x: float,
        y: float,
        z: float,
        quat: Tuple[float, float, float, float],
        label: str,
    ):
        req = SetEntityPose.Request()
        req.entity.id = 0
        req.entity.name = entity_name
        req.entity.type = req.entity.MODEL

        req.pose.position.x = float(x)
        req.pose.position.y = float(y)
        req.pose.position.z = float(z)

        req.pose.orientation.x = float(quat[0])
        req.pose.orientation.y = float(quat[1])
        req.pose.orientation.z = float(quat[2])
        req.pose.orientation.w = float(quat[3])

        fut = self.cli.call_async(req)

        def _done_cb(f):
            try:
                resp = f.result()  # may raise
            except Exception as e:
                self.get_logger().warn(f"[follow_uav] set_pose failed for {label} '{entity_name}': {e}")
                if label == "camera":
                    self.camera_setpose_failures_total += 1
                    self.camera_setpose_failures_consecutive += 1
                return
            if not getattr(resp, "success", False):
                if label == "camera":
                    self.camera_setpose_failures_total += 1
                    self.camera_setpose_failures_consecutive += 1
                    if not self.camera_setpose_failed_latched:
                        self.get_logger().warn(
                            f"[follow_uav] set_pose failed for camera model '{entity_name}'. "
                            "Camera may not be spawned or named differently."
                        )
                        self.camera_setpose_failed_latched = True
                    if (
                        self.camera_pose_control_mode == "auto"
                        and self.camera_setpose_runtime_enabled
                        and self.camera_setpose_failures_consecutive >= self.camera_pose_failover_failures
                    ):
                        self.camera_setpose_runtime_enabled = False
                        if not self.camera_setpose_failover_latched:
                            self.emit_event("CAMERA_SETPOSE_FAILOVER_GIMBAL_ONLY")
                            self.get_logger().warn(
                                "[follow_uav] camera set_pose failed repeatedly; "
                                "switching to gimbal-only camera control path for this run."
                            )
                            self.camera_setpose_failover_latched = True
                else:
                    self.get_logger().warn(
                        f"[follow_uav] set_pose reported failure for {label} '{entity_name}'"
                    )
            elif label == "camera":
                self.camera_setpose_failures_consecutive = 0
                if self.camera_setpose_failed_latched:
                    self.get_logger().info(
                        f"[follow_uav] camera set_pose recovered for '{entity_name}'"
                    )
                    self.camera_setpose_failed_latched = False

        fut.add_done_callback(_done_cb)
        return fut

    def set_entity_pose_async(self, entity_name: str, x: float, y: float, z: float, yaw_rad: float):
        if self.cli is None:
            return None
        quat = quat_from_yaw(float(yaw_rad))
        return self._set_entity_pose_quat_async(entity_name, x, y, z, quat, "uav")

    def set_camera_pose_async(
        self,
        x: float,
        y: float,
        z: float,
        yaw_rad: float,
        pan_deg: float = 0.0,
        tilt_deg: Optional[float] = None,
    ):
        if self.cli is None:
            return None
        use_tilt_deg = self.camera_tilt_cmd_deg if tilt_deg is None else float(tilt_deg)
        quat = quat_from_camera_pitch_deg(use_tilt_deg, yaw_rad + math.radians(pan_deg))
        return self._set_entity_pose_quat_async(
            self.camera_model_name,
            x,
            y,
            z - self.camera_z_offset_m,
            quat,
            "camera",
        )

    def _camera_setpose_enabled(self) -> bool:
        if self.uav_backend != "setpose":
            return False
        if self.camera_pose_control_mode == "gimbal_only":
            return False
        if self.camera_pose_control_mode == "separate_model":
            return True
        return self.camera_setpose_runtime_enabled

    def _has_pending_pose_requests(self) -> bool:
        if not self.pending_futures:
            return False
        self.pending_futures = [f for f in self.pending_futures if f is not None and not f.done()]
        return bool(self.pending_futures)

    def send_controller_command(
        self,
        x: float,
        y: float,
        z: float,
        yaw_rad: float,
        pan_deg: float = 0.0,
        tilt_deg: Optional[float] = None,
    ) -> None:
        if self.controller_cmd_pub is None:
            return

        dx = 0.0
        dy = 0.0
        if self.have_uav_cmd:
            dx = float(x - self.uav_cmd.x)
            dy = float(y - self.uav_cmd.y)

        cmd = Joy()
        cmd.axes = [dx, dy, float(z), float(yaw_rad)]
        self.controller_cmd_pub.publish(cmd)

        if self.publish_gimbal_cmd and self.controller_pan_pub is not None and self.controller_tilt_pub is not None:
            pan_msg = Float64()
            pan_msg.data = float(pan_deg)
            self.controller_pan_pub.publish(pan_msg)

            tilt_msg = Float64()
            tilt_msg.data = float(self.camera_tilt_cmd_deg if tilt_deg is None else tilt_deg)
            self.controller_tilt_pub.publish(tilt_msg)

    def publish_pose_cmd(self, x: float, y: float, z: float, yaw_rad: float):
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


    def publish_metrics_cmd(self, leader: Pose2D, cmd_x: float, cmd_y: float) -> None:
        if not self.publish_metrics:
            return
        d_cmd = math.hypot(cmd_x - leader.x, cmd_y - leader.y)
        err = abs(d_cmd - self.d_target)

        m1 = Float32()
        m1.data = float(d_cmd)
        self.metric_dist_pub.publish(m1)

        m2 = Float32()
        m2.data = float(err)
        self.metric_err_pub.publish(m2)

    def _publish_follow_debug_status(
        self,
        now: Time,
        leader: Optional[Pose2D],
        cmd_x: float,
        cmd_y: float,
        yaw_cmd: float,
        pan_cmd_deg: float,
        tilt_cmd_deg: float,
        quality_scale: float,
        quality_tag: str,
        note: str = "",
        track_motion_hold: bool = False,
        predicted_mode: bool = False,
        under_target_hold: bool = False,
        search_phase: str = "IDLE",
        reacquire_commit: bool = False,
        reacquire_fallback_hold: bool = False,
    ) -> None:
        if self.debug_status_pub is None:
            return

        prev_x = self.uav_cmd.x if self.have_uav_cmd else cmd_x
        prev_y = self.uav_cmd.y if self.have_uav_cmd else cmd_y
        dx = cmd_x - prev_x
        dy = cmd_y - prev_y
        leader_state = self._leader_status_state()
        bearing_img_deg = self._status_float("bearing_img_deg", float("nan"))
        err_u_px = self._status_float("err_u_px", float("nan"))
        err_v_px = self._status_float("err_v_px", float("nan"))
        sel_class = str(self.last_leader_status_fields.get("sel_class", "na"))
        sel_visible = str(self.last_leader_status_fields.get("sel_visible", "na"))
        sel_changed = str(self.last_leader_status_fields.get("sel_changed", "na"))
        relock_ready = 1 if self.sel_visible_streak >= self.reacquire_relock_min_visible_frames else 0
        first_lock_ready = 1 if self.reacquire_first_lock_achieved else 0

        rel_cmd_x = float("nan")
        rel_cmd_y = float("nan")
        leader_x = float("nan")
        leader_y = float("nan")
        leader_yaw_deg = float("nan")
        if leader is not None:
            leader_x = leader.x
            leader_y = leader.y
            leader_yaw_deg = math.degrees(leader.yaw)
            rel_cmd_x, rel_cmd_y = self._to_leader_frame(leader, cmd_x, cmd_y)

        msg = String()
        cam_path = "separate_model" if self._camera_setpose_enabled() else "gimbal_only"
        msg.data = (
            f"state={self.follow_state} leader_state={leader_state or 'na'} "
            f"xy_motion_enable={1 if self.xy_motion_enable else 0} track_motion_hold={1 if track_motion_hold else 0} "
            f"cam_path={cam_path} "
            f"predicted_mode={1 if predicted_mode else 0} "
            f"search_phase={search_phase} "
            f"first_lock_ready={first_lock_ready} sel_visible_streak={self.sel_visible_streak} relock_ready={relock_ready} "
            f"reacquire_yaw_assist={1 if self.reacquire_seq_allow_yaw_assist else 0} "
            f"reacquire_xy_assist={1 if self.reacquire_seq_allow_xy_assist else 0} "
            f"reacquire_commit={1 if reacquire_commit else 0} "
            f"reacquire_fallback_hold={1 if reacquire_fallback_hold else 0} "
            f"quality_scale={quality_scale:.3f} quality_tag={quality_tag} "
            f"leader_x={leader_x:.3f} leader_y={leader_y:.3f} leader_yaw_deg={leader_yaw_deg:.1f} "
            f"cmd_x={cmd_x:.3f} cmd_y={cmd_y:.3f} cmd_yaw_deg={math.degrees(yaw_cmd):.1f} "
            f"cmd_dx={dx:.3f} cmd_dy={dy:.3f} rel_cmd_x={rel_cmd_x:.3f} rel_cmd_y={rel_cmd_y:.3f} "
            f"pan_cmd_deg={pan_cmd_deg:.2f} tilt_cmd_deg={tilt_cmd_deg:.2f} under_target_hold={1 if under_target_hold else 0} "
            f"bearing_img_deg={bearing_img_deg:.2f} err_u_px={err_u_px:.2f} err_v_px={err_v_px:.2f} "
            f"sel_visible={sel_visible} sel_changed={sel_changed} sel_class={sel_class} note={note or 'na'} "
            f"t={now.nanoseconds}"
        )
        self.debug_status_pub.publish(msg)

    def _issue_hold_command(self, now: Time, reason: str) -> None:
        """
        Publish a stable HOLD command while waiting for fresh estimates.
        This keeps command/log streams alive and allows camera-lock search
        to continue scanning for reacquire.
        """
        if self.uav_backend == "setpose" and self._has_pending_pose_requests():
            return
        if not self.can_send_command_now(now):
            return

        if self.have_uav_cmd:
            hold_pose = Pose2D(self.uav_cmd.x, self.uav_cmd.y, self.uav_cmd.yaw)
        else:
            hold_pose = Pose2D(
                self.controller_seed_x,
                self.controller_seed_y,
                math.radians(self.controller_seed_yaw_deg),
            )
            self.uav_cmd = hold_pose
            self.have_uav_cmd = True
        if self.xy_motion_hold_latched:
            self.emit_event("FOLLOW_XY_DISABLED_EXIT")
            self.xy_motion_hold_latched = False

        camera_pan_deg = self._update_camera_pan_lock(now)
        camera_tilt_deg = self._update_camera_tilt_lock(now)
        search_phase = self.reacquire_seq_phase if self.reacquire_seq_active else "IDLE"
        hold_yaw_cmd = hold_pose.yaw
        reacquire_hold_xy = False
        reacquire_hold_yaw = False
        if self.reacquire_sequence_enable and self.leader_input_type == "pose":
            camera_pan_deg, camera_tilt_deg, hold_yaw_cmd, reacquire_hold_xy, reacquire_hold_yaw, search_phase = (
                self._update_reacquire_sequence(now, hold_pose.yaw)
            )
            if not reacquire_hold_yaw:
                hold_pose = Pose2D(hold_pose.x, hold_pose.y, hold_yaw_cmd)
        quality_scale, quality_tag = self._quality_scale_from_status(now)
        leader_for_debug = self.ugv_pose if self.have_ugv else None

        if self.uav_backend == "setpose":
            self.pending_futures = [
                self.set_entity_pose_async(self.uav_name, hold_pose.x, hold_pose.y, self.z_alt, hold_pose.yaw),
            ]
            if self._camera_setpose_enabled():
                self.pending_futures.append(
                    self.set_camera_pose_async(
                        hold_pose.x,
                        hold_pose.y,
                        self.z_alt,
                        hold_pose.yaw,
                        camera_pan_deg,
                        camera_tilt_deg,
                    )
                )
            if self.publish_gimbal_cmd and self.controller_pan_pub is not None and self.controller_tilt_pub is not None:
                pan_msg = Float64()
                pan_msg.data = float(camera_pan_deg)
                self.controller_pan_pub.publish(pan_msg)
                tilt_msg = Float64()
                tilt_msg.data = float(camera_tilt_deg)
                self.controller_tilt_pub.publish(tilt_msg)
        else:
            self.pending_futures = []
            self.send_controller_command(hold_pose.x, hold_pose.y, self.z_alt, hold_pose.yaw, camera_pan_deg, camera_tilt_deg)

        self.publish_pose_cmd(hold_pose.x, hold_pose.y, self.z_alt, hold_pose.yaw)
        self.publish_pose_cmd_odom(hold_pose.x, hold_pose.y, self.z_alt, hold_pose.yaw)
        self._publish_follow_debug_status(
            now=now,
            leader=leader_for_debug,
            cmd_x=hold_pose.x,
            cmd_y=hold_pose.y,
            yaw_cmd=hold_pose.yaw,
            pan_cmd_deg=camera_pan_deg,
            tilt_cmd_deg=camera_tilt_deg,
            quality_scale=quality_scale,
            quality_tag=quality_tag,
            note=f"hold:{reason}",
            track_motion_hold=True,
            predicted_mode=False,
            under_target_hold=False,
            search_phase=search_phase,
            reacquire_commit=self._reacquire_commit_active(now),
            reacquire_fallback_hold=self.reacquire_seq_fallback_hold,
        )
        self.last_cmd_time = now
        self.emit_event(f"FOLLOW_HOLD_TICK:{reason}")

    def on_tick(self):
        now = self.get_clock().now()

        predicted_mode = False
        # Need fresh leader pose, or short-term predicted fallback in observation mode.
        if not self.ugv_pose_is_fresh(now):
            pred = self._predict_leader_pose(now)
            if pred is None:
                if self.stale_predict_active:
                    self.emit_event("POSE_STALE_PREDICT_EXIT")
                    self.stale_predict_active = False
                self._update_follow_state("HOLD")
                if not self.stale_latched:
                    self.emit_event("POSE_STALE_HOLD_ENTER")
                    self.stale_latched = True
                self._issue_hold_command(now, "pose_stale")
                return
            predicted_mode = True
            self.ugv_pose = pred
            self.have_ugv = True
            if not self.stale_predict_active:
                self.emit_event("POSE_STALE_PREDICT_ENTER")
                self.stale_predict_active = True
            if self.stale_latched:
                self.emit_event("POSE_STALE_HOLD_EXIT")
                self.stale_latched = False
        else:
            if self.stale_predict_active:
                self.emit_event("POSE_STALE_PREDICT_EXIT")
                self.stale_predict_active = False
            if self.stale_latched:
                self.emit_event("POSE_STALE_HOLD_EXIT")
                self.stale_latched = False

        # Avoid backlog only for set_pose backend requests.
        if self.uav_backend == "setpose" and self._has_pending_pose_requests():
            return

        # Rate-limit commands independently of tick rate
        if not self.can_send_command_now(now):
            return

        ugv = self.ugv_pose
        leader_for_follow = self._leader_pose_for_follow()
        quality_scale, quality_tag = self._quality_scale_from_status(now)
        if predicted_mode and self.leader_input_type == "pose":
            quality_scale = max(quality_scale, self.stale_predict_quality_floor)
            quality_tag = f"{quality_tag}:stale_predict"
        desired_follow_state = self._classify_follow_state(quality_scale)
        self._update_follow_state(desired_follow_state)

        # Desired target behind UGV
        xt = leader_for_follow.x - self.d_target * math.cos(leader_for_follow.yaw)
        yt = leader_for_follow.y - self.d_target * math.sin(leader_for_follow.yaw)

        # Leash clamp: ensure target is within d_max of UGV
        xt2, yt2 = clamp_point_to_radius(leader_for_follow.x, leader_for_follow.y, xt, yt, self.d_max)
        if (xt2, yt2) != (xt, yt):
            self.emit_event("LEASH_CLAMPED")
        xt, yt = xt2, yt2

        # Optional smoothing on commanded XY (helps jitter; default alpha=1.0 => no smoothing)
        if self.have_uav_cmd and self.smooth_alpha < 1.0:
            a = self.smooth_alpha
            xt = a * xt + (1.0 - a) * self.uav_cmd.x
            yt = a * yt + (1.0 - a) * self.uav_cmd.y

        # Phase 2: relative-frame trajectory shaping (vel/accel limited), preserving the
        # same leash target geometry while smoothing how we move the command toward it.
        xt, yt = self._shape_target_trajectory(leader_for_follow, xt, yt, now, quality_scale)

        # Confidence/latency-aware command shaping for estimate-mode: preserve follow+leash
        # semantics while reducing aggression on noisy/intermittent perception.
        eff_step_limit = self.max_step_m_per_tick
        eff_yaw_step_limit = self.max_yaw_step_rad_per_tick
        eff_deadband = self.cmd_xy_deadband_m
        if self.leader_input_type == "pose" and self.quality_scale_enable:
            if eff_step_limit > 0.0:
                eff_step_limit *= quality_scale
            if eff_yaw_step_limit > 0.0:
                eff_yaw_step_limit *= quality_scale
            eff_deadband += (1.0 - quality_scale) * self.quality_deadband_boost_m

            quality_hold = self.have_uav_cmd and quality_scale <= self.quality_hold_step_scale
            if quality_hold:
                xt = self.uav_cmd.x
                yt = self.uav_cmd.y
                if not self.quality_hold_latched:
                    self.emit_event(f"FOLLOW_QUALITY_HOLD_ENTER:{quality_tag}")
                    self.quality_hold_latched = True
            elif self.quality_hold_latched:
                self.emit_event("FOLLOW_QUALITY_HOLD_EXIT")
                self.quality_hold_latched = False

        # In observation-only chain, only move when estimate state is TRACK (post-debounce).
        # During REACQUIRE/HOLD/DEGRADED we keep XY fixed unless explicit structured-search
        # assist is armed. This avoids startup/search oscillation.
        reacquire_search_motion_assist = (
            self.reacquire_sequence_enable
            and self.leader_input_type == "pose"
            and self.reacquire_seq_active
            and self.reacquire_seq_allow_xy_assist
            and (not self.reacquire_seq_fallback_hold)
            and self.reacquire_seq_phase in ("PAN", "TILT", "YAW")
        )
        gate_block_states = {"HOLD", "DEGRADED", "REACQUIRE"}
        if reacquire_search_motion_assist:
            gate_block_states.discard("DEGRADED")
            if self.track_only_allow_reacquire_motion:
                gate_block_states.discard("REACQUIRE")
        track_motion_hold = (
            self.leader_input_type == "pose"
            and self.track_only_motion_enable
            and self.follow_state in gate_block_states
            and self.have_uav_cmd
        )
        if track_motion_hold:
            xt = self.uav_cmd.x
            yt = self.uav_cmd.y
            if not self.track_only_hold_latched:
                self.emit_event(f"FOLLOW_TRACK_GATE_HOLD_ENTER:{self.follow_state}")
                self.track_only_hold_latched = True
        elif self.track_only_hold_latched:
            self.emit_event("FOLLOW_TRACK_GATE_HOLD_EXIT")
            self.track_only_hold_latched = False

        if not self.xy_motion_enable:
            if self.have_uav_cmd:
                xt = self.uav_cmd.x
                yt = self.uav_cmd.y
            else:
                xt = self.controller_seed_x
                yt = self.controller_seed_y
            if not self.xy_motion_hold_latched:
                self.emit_event("FOLLOW_XY_DISABLED_ENTER")
                self.xy_motion_hold_latched = True
        elif self.xy_motion_hold_latched:
            self.emit_event("FOLLOW_XY_DISABLED_EXIT")
            self.xy_motion_hold_latched = False

        # Clamp step-to-step commanded motion to avoid snapping on estimate jumps.
        if self.have_uav_cmd and eff_step_limit > 0.0:
            dx = xt - self.uav_cmd.x
            dy = yt - self.uav_cmd.y
            step = math.hypot(dx, dy)
            if step > eff_step_limit and step > 1e-9:
                s = eff_step_limit / step
                xt = self.uav_cmd.x + dx * s
                yt = self.uav_cmd.y + dy * s
                self.emit_event("FOLLOW_STEP_CLAMPED")

        self.publish_metrics_cmd(ugv, xt, yt)
        cmd_xy_delta = 0.0
        if self.have_uav_cmd:
            cmd_xy_delta = math.hypot(xt - self.uav_cmd.x, yt - self.uav_cmd.y)

        yaw_cmd = leader_for_follow.yaw if self.follow_yaw else (self.uav_cmd.yaw if self.have_uav_cmd else leader_for_follow.yaw)
        if track_motion_hold and self.track_only_hold_yaw_enable and self.have_uav_cmd:
            yaw_cmd = self.uav_cmd.yaw
        elif self.follow_state in ("HOLD", "DEGRADED") and self.have_uav_cmd:
            yaw_cmd = self.uav_cmd.yaw
        elif self.have_uav_cmd and self.leader_input_type == "pose" and self.quality_scale_enable and quality_scale <= self.quality_hold_step_scale:
            yaw_cmd = self.uav_cmd.yaw
        elif (
            self.have_uav_cmd
            and self.leader_input_type == "pose"
            and self.yaw_update_xy_gate_m > 0.0
            and cmd_xy_delta < self.yaw_update_xy_gate_m
        ):
            # Avoid yaw chattering when the XY command is effectively stationary.
            yaw_cmd = self.uav_cmd.yaw
        elif self.have_uav_cmd and self.yaw_deadband_rad > 0.0:
            if abs(wrap_pi(yaw_cmd - self.uav_cmd.yaw)) < self.yaw_deadband_rad:
                yaw_cmd = self.uav_cmd.yaw
        if self.have_uav_cmd and eff_yaw_step_limit > 0.0:
            dyaw = wrap_pi(yaw_cmd - self.uav_cmd.yaw)
            if abs(dyaw) > eff_yaw_step_limit:
                yaw_cmd = wrap_pi(self.uav_cmd.yaw + math.copysign(eff_yaw_step_limit, dyaw))
                self.emit_event("FOLLOW_YAW_CLAMPED")

        if self.have_uav_cmd and eff_deadband > 0.0:
            if math.hypot(xt - self.uav_cmd.x, yt - self.uav_cmd.y) < eff_deadband:
                xt = self.uav_cmd.x
                yt = self.uav_cmd.y

        under_target_hold = False
        if self.under_target_guard_enable and self.have_uav_cmd:
            err_v_px = self._status_float("err_v_px", float("nan"))
            img_h = self._status_float("img_h", float("nan"))
            range_m = self._status_float("range_m", float("nan"))
            vertical_low = (
                math.isfinite(err_v_px)
                and math.isfinite(img_h)
                and img_h > 1.0
                and err_v_px >= (self.under_target_err_v_ratio * img_h)
            )
            range_close = math.isfinite(range_m) and range_m > 0.0 and range_m <= self.under_target_range_m
            under_target_hold = bool(vertical_low and range_close)
            if under_target_hold:
                if self.under_target_hold_xy:
                    xt = self.uav_cmd.x
                    yt = self.uav_cmd.y
                if self.under_target_hold_yaw:
                    yaw_cmd = self.uav_cmd.yaw
                if not self.under_target_guard_latched:
                    self.emit_event("FOLLOW_UNDER_TARGET_GUARD_ENTER")
                    self.under_target_guard_latched = True
            elif self.under_target_guard_latched:
                self.emit_event("FOLLOW_UNDER_TARGET_GUARD_EXIT")
                self.under_target_guard_latched = False
        elif self.under_target_guard_latched:
            self.emit_event("FOLLOW_UNDER_TARGET_GUARD_EXIT")
            self.under_target_guard_latched = False

        camera_pan_deg = self._update_camera_pan_lock(now)
        camera_tilt_deg = self._update_camera_tilt_lock(now)
        search_phase = self.reacquire_seq_phase if self.reacquire_seq_active else "IDLE"
        reacquire_hold_xy = False
        reacquire_hold_yaw = False
        if self.reacquire_sequence_enable and self.leader_input_type == "pose":
            camera_pan_deg, camera_tilt_deg, yaw_cmd, reacquire_hold_xy, reacquire_hold_yaw, search_phase = (
                self._update_reacquire_sequence(now, yaw_cmd)
            )
            reacquire_search_motion_assist = (
                self.follow_state == "DEGRADED"
                and self.reacquire_seq_allow_xy_assist
                and (search_phase in ("PAN", "TILT", "YAW"))
                and (not self.reacquire_seq_fallback_hold)
            )
            if reacquire_search_motion_assist:
                # Motion assist is only allowed after the configured persistence delay.
                reacquire_hold_xy = False
            if reacquire_hold_xy and self.have_uav_cmd:
                xt = self.uav_cmd.x
                yt = self.uav_cmd.y
            if reacquire_hold_yaw and self.have_uav_cmd:
                yaw_cmd = self.uav_cmd.yaw

        if self.uav_backend == "setpose":
            # Command UAV model and its separately spawned gimbal/camera model.
            self.pending_futures = [
                self.set_entity_pose_async(self.uav_name, xt, yt, self.z_alt, yaw_cmd),
            ]
            if self._camera_setpose_enabled():
                self.pending_futures.append(
                    self.set_camera_pose_async(xt, yt, self.z_alt, yaw_cmd, camera_pan_deg, camera_tilt_deg)
                )
            if self.publish_gimbal_cmd and self.controller_pan_pub is not None and self.controller_tilt_pub is not None:
                pan_msg = Float64()
                pan_msg.data = float(camera_pan_deg)
                self.controller_pan_pub.publish(pan_msg)
                tilt_msg = Float64()
                tilt_msg.data = float(camera_tilt_deg)
                self.controller_tilt_pub.publish(tilt_msg)
        else:
            self.pending_futures = []
            self.send_controller_command(xt, yt, self.z_alt, yaw_cmd, camera_pan_deg, camera_tilt_deg)

        # Publish commanded pose for rosbag/metrics
        self.publish_pose_cmd(xt, yt, self.z_alt, yaw_cmd)
        self.publish_pose_cmd_odom(xt, yt, self.z_alt, yaw_cmd)
        self._publish_follow_debug_status(
            now=now,
            leader=leader_for_follow,
            cmd_x=xt,
            cmd_y=yt,
            yaw_cmd=yaw_cmd,
            pan_cmd_deg=camera_pan_deg,
            tilt_cmd_deg=camera_tilt_deg,
            quality_scale=quality_scale,
            quality_tag=quality_tag,
            note=(
                "track_reacquire_search" if search_phase != "IDLE"
                else ("track_under_target_guard" if under_target_hold else "track")
            ),
            track_motion_hold=(track_motion_hold or (not self.xy_motion_enable) or reacquire_hold_xy),
            predicted_mode=predicted_mode,
            under_target_hold=under_target_hold,
            search_phase=search_phase,
            reacquire_commit=self._reacquire_commit_active(now),
            reacquire_fallback_hold=self.reacquire_seq_fallback_hold,
        )

        # Update internal command state
        self.uav_cmd = Pose2D(xt, yt, yaw_cmd)
        self.have_uav_cmd = True
        self.last_cmd_time = now

        self.emit_event("FOLLOW_TICK")
    
    

def main(args=None):
    rclpy.init(args=args)
    node = FollowUav()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.emit_event("FOLLOW_NODE_SHUTDOWN")
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
