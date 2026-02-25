




#!/usr/bin/env python3
import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import ParameterDescriptor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Float32

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


def quat_from_camera_pitch_deg(pitch_deg: float) -> Tuple[float, float, float, float]:
    # Match command.py behavior: camera world pitch uses -pitch_deg.
    half = -0.5 * math.radians(float(pitch_deg))
    return (0.0, math.sin(half), 0.0, math.cos(half))


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
        self.declare_parameter("camera_name", "camera0")
        self.declare_parameter("camera_pitch_deg", -45.0)
        self.declare_parameter("camera_z_offset_m", 0.27)
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

        # Events
        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("publish_events", True)

        # ---------- Read params ----------
        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.camera_name = str(self.get_parameter("camera_name").value)
        self.camera_pitch_deg = float(self.get_parameter("camera_pitch_deg").value)
        self.camera_z_offset_m = float(self.get_parameter("camera_z_offset_m").value)
        self.camera_model_name = f"{self.uav_name}_{self.camera_name}"
        self.leader_input_type = str(self.get_parameter("leader_input_type").value).strip().lower()
        self.leader_odom_topic = str(self.get_parameter("leader_odom_topic").value)
        self.leader_pose_topic = str(self.get_parameter("leader_pose_topic").value)

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

        # ---------- State ----------
        self.have_ugv = False
        self.ugv_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_ugv_stamp: Optional[Time] = None
        self.stale_latched = False

        # Commanded UAV pose (deterministic internal state)
        self.uav_cmd = Pose2D(0.0, 0.0, 0.0)
        self.have_uav_cmd = False

        self.last_cmd_time: Optional[Time] = None
        self.pending_futures = []
        self.camera_setpose_failed_latched = False
        self.last_leader_status_fields = {}
        self.last_leader_status_rx: Optional[Time] = None
        self.quality_hold_latched = False
        self.follow_state = "INIT"
        self._desired_follow_state = "INIT"
        self._state_good_ticks = 0
        self._state_bad_ticks = 0
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

        self.events_pub = self.create_publisher(String, self.event_topic, 10)
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

        srv_name = f"/world/{self.world}/set_pose"
        self.cli = self.create_client(SetEntityPose, srv_name)
        self.get_logger().info(f"[follow_uav] Waiting for service: {srv_name}")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("[follow_uav] Service not available, waiting again...")

        dt = 1.0 / self.tick_hz
        self.timer = self.create_timer(dt, self.on_tick)

        self.get_logger().info(
            f"[follow_uav] Started: world={self.world}, uav={self.uav_name}, "
            f"camera_model={self.camera_model_name}, camera_pitch_deg={self.camera_pitch_deg}, "
            f"camera_z_offset_m={self.camera_z_offset_m}, "
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
            f"traj_enable={self.traj_enable}, traj_rel_frame_enable={self.traj_rel_frame_enable}, "
            f"traj_rel_smooth_alpha={self.traj_rel_smooth_alpha}, traj_pos_gain={self.traj_pos_gain}, "
            f"traj_max_speed_mps={self.traj_max_speed_mps}, traj_max_accel_mps2={self.traj_max_accel_mps2}, "
            f"traj_quality_min_scale={self.traj_quality_min_scale}, traj_reset_on_yaw_jump_rad={self.traj_reset_on_yaw_jump_rad}, "
            f"estimate_heading_from_motion_enable={self.estimate_heading_from_motion_enable}, "
            f"estimate_heading_alpha={self.estimate_heading_alpha}, estimate_heading_min_speed_mps={self.estimate_heading_min_speed_mps}, "
            f"estimate_heading_max_dt_s={self.estimate_heading_max_dt_s}, "
            f"event_topic={self.event_topic}"
        )
        self.emit_event("FOLLOW_NODE_START")

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
        if not self.quality_scale_enable or self.leader_input_type != "pose":
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
                return
            if not getattr(resp, "success", False):
                if label == "camera":
                    if not self.camera_setpose_failed_latched:
                        self.get_logger().warn(
                            f"[follow_uav] set_pose failed for camera model '{entity_name}'. "
                            "Camera may not be spawned or named differently."
                        )
                        self.camera_setpose_failed_latched = True
                else:
                    self.get_logger().warn(
                        f"[follow_uav] set_pose reported failure for {label} '{entity_name}'"
                    )
            elif label == "camera" and self.camera_setpose_failed_latched:
                self.get_logger().info(
                    f"[follow_uav] camera set_pose recovered for '{entity_name}'"
                )
                self.camera_setpose_failed_latched = False

        fut.add_done_callback(_done_cb)
        return fut

    def set_entity_pose_async(self, entity_name: str, x: float, y: float, z: float, yaw_rad: float):
        quat = quat_from_yaw(float(yaw_rad))
        return self._set_entity_pose_quat_async(entity_name, x, y, z, quat, "uav")

    def set_camera_pose_async(self, x: float, y: float, z: float):
        quat = quat_from_camera_pitch_deg(self.camera_pitch_deg)
        return self._set_entity_pose_quat_async(
            self.camera_model_name,
            x,
            y,
            z - self.camera_z_offset_m,
            quat,
            "camera",
        )

    def _has_pending_pose_requests(self) -> bool:
        if not self.pending_futures:
            return False
        self.pending_futures = [f for f in self.pending_futures if f is not None and not f.done()]
        return bool(self.pending_futures)

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

    def on_tick(self):
        now = self.get_clock().now()

        # Need fresh UGV pose
        if not self.ugv_pose_is_fresh(now):
            self._update_follow_state("HOLD")
            if not self.stale_latched:
                self.emit_event("POSE_STALE_HOLD_ENTER")
                self.stale_latched = True
            return
        else:
            if self.stale_latched:
                self.emit_event("POSE_STALE_HOLD_EXIT")
                self.stale_latched = False

        # Avoid backlog: if previous set_pose requests are still pending, skip this tick.
        if self._has_pending_pose_requests():
            return

        # Rate-limit commands independently of tick rate
        if not self.can_send_command_now(now):
            return

        ugv = self.ugv_pose
        leader_for_follow = self._leader_pose_for_follow()
        quality_scale, quality_tag = self._quality_scale_from_status(now)
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
        if self.follow_state in ("HOLD", "DEGRADED") and self.have_uav_cmd:
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

        # Command UAV model and its separately spawned gimbal/camera model.
        self.pending_futures = [
            self.set_entity_pose_async(self.uav_name, xt, yt, self.z_alt, yaw_cmd),
            self.set_camera_pose_async(xt, yt, self.z_alt),
        ]

        # Publish commanded pose for rosbag/metrics
        self.publish_pose_cmd(xt, yt, self.z_alt, yaw_cmd)
        self.publish_pose_cmd_odom(xt, yt, self.z_alt, yaw_cmd)

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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
