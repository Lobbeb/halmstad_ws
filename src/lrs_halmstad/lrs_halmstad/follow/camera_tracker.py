#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Float32, Float64, String
from tf_transformations import quaternion_from_euler

from lrs_halmstad.follow.follow_math import (
    Pose2D,
    camera_xy_from_uav_pose,
    coerce_bool,
    compute_camera_tilt_deg,
    compute_leader_look_target,
    wrap_pi,
    yaw_from_quat,
)
from lrs_halmstad.perception.detection_protocol import Detection2D, decode_detection_payload


def should_prefer_command_pose(
    *,
    have_cmd: bool,
    cmd_stamp_ns: Optional[int],
    have_actual: bool,
    actual_stamp_ns: Optional[int],
    now_ns: int,
    pose_timeout_s: float,
) -> bool:
    if not have_cmd or cmd_stamp_ns is None:
        return False
    if now_ns - cmd_stamp_ns > pose_timeout_s * 1e9:
        return False
    if not have_actual or actual_stamp_ns is None:
        return True
    if now_ns - actual_stamp_ns > pose_timeout_s * 1e9:
        return True
    return cmd_stamp_ns >= actual_stamp_ns


def apply_deadband_command(
    target_value: float,
    previous_value: Optional[float],
    deadband_value: float,
) -> float:
    if previous_value is None or deadband_value <= 0.0:
        return float(target_value)
    if abs(float(target_value) - float(previous_value)) < deadband_value:
        return float(previous_value)
    return float(target_value)


TRACKABLE_ESTIMATOR_STATES = frozenset({"OK"})


def parse_status_field(status_line: str, key: str) -> Optional[str]:
    prefix = f"{key}="
    for token in status_line.split():
        if token.startswith(prefix):
            return token[len(prefix):]
    return None

class CameraTracker(Node):
    def __init__(self):
        super().__init__("camera_tracker")

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("leader_input_type", "odom")
        self.declare_parameter("leader_odom_topic", "/a201_0000/platform/odom/filtered")
        self.declare_parameter("leader_pose_topic", "/coord/leader_estimate")
        self.declare_parameter("leader_status_topic", "/coord/leader_estimate_status")
        self.declare_parameter("leader_detection_topic", "/coord/leader_detection")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("uav_camera_mode", "integrated_joint")
        self.declare_parameter("camera_mount_pitch_deg", 45.0)
        self.declare_parameter("uav_pose_cmd_topic", "")
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("pose_timeout_s", 3.0)
        self.declare_parameter("camera_x_offset_m", 0.0)
        self.declare_parameter("camera_y_offset_m", 0.0)
        self.declare_parameter("camera_z_offset_m", 0.27)
        self.declare_parameter("leader_look_target_x_m", 0.0)
        self.declare_parameter("leader_look_target_y_m", 0.0)
        self.declare_parameter("camera_look_target_z_m", 0.0)
        self.declare_parameter("camera_yaw_offset_deg", 0.0)
        self.declare_parameter("camera_pan_sign", 1.0)
        self.declare_parameter("default_pan_deg", 0.0)
        self.declare_parameter("pan_enable", True)
        self.declare_parameter("default_tilt_deg", 0.0)
        self.declare_parameter("tilt_enable", True)
        self.declare_parameter("tilt_deadband_deg", 0.0)
        self.declare_parameter("image_center_correction_enable", False)
        self.declare_parameter("image_center_correction_timeout_s", 0.25)
        self.declare_parameter("image_center_deadband_deg", 0.75)
        self.declare_parameter("pan_image_center_gain", 0.0)
        self.declare_parameter("pan_image_center_max_deg", 12.0)
        self.declare_parameter("tilt_image_center_gain", 0.0)
        self.declare_parameter("tilt_image_center_max_deg", 8.0)
        self.declare_parameter("publish_debug_topics", True)

        self.uav_name = str(self.get_parameter("uav_name").value)
        self.leader_input_type = str(self.get_parameter("leader_input_type").value).strip().lower()
        self.leader_odom_topic = str(self.get_parameter("leader_odom_topic").value)
        self.leader_pose_topic = str(self.get_parameter("leader_pose_topic").value)
        self.leader_status_topic = str(self.get_parameter("leader_status_topic").value).strip()
        self.leader_detection_topic = str(self.get_parameter("leader_detection_topic").value).strip()
        self.camera_info_topic = (
            str(self.get_parameter("camera_info_topic").value).strip()
            or f"/{self.uav_name}/camera0/camera_info"
        )
        self.uav_camera_mode = str(self.get_parameter("uav_camera_mode").value).strip().lower()
        self.camera_mount_pitch_deg = float(self.get_parameter("camera_mount_pitch_deg").value)
        self.uav_pose_cmd_topic = str(self.get_parameter("uav_pose_cmd_topic").value).strip()
        self.tick_hz = max(1.0, float(self.get_parameter("tick_hz").value))
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.camera_x_offset_m = float(self.get_parameter("camera_x_offset_m").value)
        self.camera_y_offset_m = float(self.get_parameter("camera_y_offset_m").value)
        self.camera_z_offset_m = float(self.get_parameter("camera_z_offset_m").value)
        self.leader_look_target_x_m = float(self.get_parameter("leader_look_target_x_m").value)
        self.leader_look_target_y_m = float(self.get_parameter("leader_look_target_y_m").value)
        self.camera_look_target_z_m = float(self.get_parameter("camera_look_target_z_m").value)
        self.camera_yaw_offset_deg = float(self.get_parameter("camera_yaw_offset_deg").value)
        self.camera_pan_sign = float(self.get_parameter("camera_pan_sign").value)
        self.default_pan_deg = float(self.get_parameter("default_pan_deg").value)
        self.pan_enable = coerce_bool(self.get_parameter("pan_enable").value)
        self.default_tilt_deg = float(self.get_parameter("default_tilt_deg").value)
        self.tilt_enable = coerce_bool(self.get_parameter("tilt_enable").value)
        self.tilt_deadband_deg = max(0.0, float(self.get_parameter("tilt_deadband_deg").value))
        self.image_center_correction_enable = coerce_bool(
            self.get_parameter("image_center_correction_enable").value
        )
        self.image_center_correction_timeout_s = max(
            0.0, float(self.get_parameter("image_center_correction_timeout_s").value)
        )
        self.image_center_deadband_deg = max(0.0, float(self.get_parameter("image_center_deadband_deg").value))
        self.pan_image_center_gain = float(self.get_parameter("pan_image_center_gain").value)
        self.pan_image_center_max_deg = max(0.0, float(self.get_parameter("pan_image_center_max_deg").value))
        self.tilt_image_center_gain = float(self.get_parameter("tilt_image_center_gain").value)
        self.tilt_image_center_max_deg = max(0.0, float(self.get_parameter("tilt_image_center_max_deg").value))
        self.publish_debug_topics = coerce_bool(self.get_parameter("publish_debug_topics").value)

        if self.leader_input_type == "estimate":
            self.leader_input_type = "pose"
        if self.leader_input_type not in ("odom", "pose"):
            raise ValueError("leader_input_type must be 'odom', 'pose', or 'estimate'")
        if self.uav_camera_mode == "integrated":
            self.uav_camera_mode = "integrated_joint"
        if self.uav_camera_mode == "detached":
            self.uav_camera_mode = "detached_model"
        if not self.uav_pose_cmd_topic:
            self.uav_pose_cmd_topic = f"/{self.uav_name}/pose_cmd"

        self.have_leader = False
        self.leader_pose = Pose2D(0.0, 0.0, 0.0)
        self.leader_z = 0.0
        self.last_leader_stamp: Optional[Time] = None
        self.last_leader_status_state: Optional[str] = None
        self.last_leader_status_stamp: Optional[Time] = None
        self.last_trackable_leader_pose: Optional[Pose2D] = None
        self.last_trackable_leader_z: Optional[float] = None
        self.last_detection: Optional[Detection2D] = None
        self.last_detection_stamp: Optional[Time] = None
        self.camera_fx: Optional[float] = None
        self.camera_fy: Optional[float] = None
        self.camera_cx: Optional[float] = None
        self.camera_cy: Optional[float] = None

        self.have_uav_actual = False
        self.uav_actual_pose = Pose2D(0.0, 0.0, 0.0)
        self.uav_actual_z = 0.0
        self.last_uav_actual_stamp: Optional[Time] = None

        self.have_uav_cmd = False
        self.uav_cmd_pose = Pose2D(0.0, 0.0, 0.0)
        self.uav_cmd_z = 0.0
        self.last_uav_cmd_stamp: Optional[Time] = None
        self.last_tilt_cmd_deg: Optional[float] = None
        self.last_pan_cmd_deg: Optional[float] = None

        if self.leader_input_type == "odom":
            self.leader_sub = self.create_subscription(
                Odometry,
                self.leader_odom_topic,
                self.on_leader_odom,
                10,
            )
        else:
            self.leader_sub = self.create_subscription(
                PoseStamped,
                self.leader_pose_topic,
                self.on_leader_pose,
                10,
            )
            self.leader_status_sub = self.create_subscription(
                String,
                self.leader_status_topic,
                self.on_leader_status,
                10,
            ) if self.leader_status_topic else None
        self.leader_detection_sub = (
            self.create_subscription(
                String,
                self.leader_detection_topic,
                self.on_leader_detection,
                10,
            )
            if self.image_center_correction_enable and self.leader_detection_topic
            else None
        )
        camera_info_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.camera_info_sub = (
            self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.on_camera_info,
                camera_info_qos,
            )
            if self.image_center_correction_enable
            else None
        )
        camera_info_sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self.camera_info_sensor_sub = (
            self.create_subscription(
                CameraInfo,
                self.camera_info_topic,
                self.on_camera_info,
                camera_info_sensor_qos,
            )
            if self.image_center_correction_enable
            else None
        )

        self.uav_sub = self.create_subscription(
            PoseStamped,
            f"/{self.uav_name}/pose",
            self.on_uav_pose,
            10,
        )
        self.uav_cmd_sub = self.create_subscription(
            PoseStamped,
            self.uav_pose_cmd_topic,
            self.on_uav_pose_cmd,
            10,
        )
        self.target_camera_pose_pub = (
            self.create_publisher(
                PoseStamped,
                f"/{self.uav_name}/camera/target/center_pose",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.target_look_at_point_pub = (
            self.create_publisher(
                PointStamped,
                f"/{self.uav_name}/camera/target/look_at_point",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.target_camera_world_yaw_pub = (
            self.create_publisher(
                Float32,
                f"/{self.uav_name}/camera/target/world_yaw_rad",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.tilt_mode_pub = (
            self.create_publisher(
                String,
                f"/{self.uav_name}/camera/debug/tilt_mode",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.tilt_target_cmd_pub = (
            self.create_publisher(
                Float32,
                f"/{self.uav_name}/camera/debug/tilt_target_cmd_deg",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.tilt_hdist_pub = (
            self.create_publisher(
                Float32,
                f"/{self.uav_name}/camera/debug/tilt_target_horizontal_distance_m",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.tilt_vdelta_pub = (
            self.create_publisher(
                Float32,
                f"/{self.uav_name}/camera/debug/tilt_target_vertical_delta_m",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.tracking_uav_pose_source_pub = (
            self.create_publisher(
                String,
                f"/{self.uav_name}/camera/debug/tracking_uav_pose_source",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.image_error_x_pub = (
            self.create_publisher(
                Float32,
                f"/{self.uav_name}/camera/debug/image_error_x_deg",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.image_error_y_pub = (
            self.create_publisher(
                Float32,
                f"/{self.uav_name}/camera/debug/image_error_y_deg",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.pan_image_correction_pub = (
            self.create_publisher(
                Float32,
                f"/{self.uav_name}/camera/debug/pan_image_correction_deg",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.image_correction_state_pub = (
            self.create_publisher(
                String,
                f"/{self.uav_name}/camera/debug/image_correction_state",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.tilt_image_correction_pub = (
            self.create_publisher(
                Float32,
                f"/{self.uav_name}/camera/debug/tilt_image_correction_deg",
                10,
            )
            if self.publish_debug_topics
            else None
        )
        self.tilt_pub = self.create_publisher(Float64, f"/{self.uav_name}/update_tilt", 10)
        self.pan_pub = self.create_publisher(Float64, f"/{self.uav_name}/update_pan", 10)
        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)
        self.get_logger().info(
            f"[camera_tracker] Started: uav={self.uav_name}, leader_input={self.leader_input_type}, "
            f"pan_enable={self.pan_enable}, default_pan_deg={self.default_pan_deg}, "
            f"tilt_enable={self.tilt_enable}, default_tilt_deg={self.default_tilt_deg}, "
            f"tilt_deadband_deg={self.tilt_deadband_deg}, "
            f"image_center_correction_enable={self.image_center_correction_enable}, "
            f"publish_debug_topics={self.publish_debug_topics}, "
            f"leader_look_target_m=({self.leader_look_target_x_m}, "
            f"{self.leader_look_target_y_m}, {self.camera_look_target_z_m}), "
            f"camera_mode={self.uav_camera_mode}, camera_mount_pitch_deg={self.camera_mount_pitch_deg}, "
            f"uav_pose_cmd_topic={self.uav_pose_cmd_topic}"
        )

    def on_leader_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.leader_pose = Pose2D(float(p.x), float(p.y), yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)))
        self.leader_z = float(p.z)
        self.have_leader = True
        try:
            self.last_leader_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_leader_stamp = self.get_clock().now()

    def on_leader_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.leader_pose = Pose2D(float(p.x), float(p.y), yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)))
        self.leader_z = float(p.z)
        self.have_leader = True
        try:
            self.last_leader_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_leader_stamp = self.get_clock().now()

    def on_uav_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_actual_pose = Pose2D(
            float(p.x),
            float(p.y),
            yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.uav_actual_z = float(p.z)
        self.have_uav_actual = True
        try:
            self.last_uav_actual_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_actual_stamp = self.get_clock().now()

    def on_uav_pose_cmd(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_cmd_pose = Pose2D(
            float(p.x),
            float(p.y),
            yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.uav_cmd_z = float(p.z)
        self.have_uav_cmd = True
        try:
            self.last_uav_cmd_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_cmd_stamp = self.get_clock().now()

    def on_leader_status(self, msg: String) -> None:
        state = parse_status_field(msg.data, "state")
        if not state:
            return
        self.last_leader_status_state = state.strip().upper()
        self.last_leader_status_stamp = self.get_clock().now()

    def on_leader_detection(self, msg: String) -> None:
        try:
            parsed = decode_detection_payload(msg.data)
        except Exception:
            return
        self.last_detection = parsed.detection
        self.last_detection_stamp = self.get_clock().now()

    def on_camera_info(self, msg: CameraInfo) -> None:
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        if fx <= 0.0 or fy <= 0.0:
            return
        self.camera_fx = fx
        self.camera_fy = fy
        self.camera_cx = float(msg.k[2])
        self.camera_cy = float(msg.k[5])

    def leader_pose_is_fresh(self, now: Time) -> bool:
        if not self.have_leader or self.last_leader_stamp is None:
            return False
        age_s = (now - self.last_leader_stamp).nanoseconds * 1e-9
        return age_s <= self.pose_timeout_s

    def leader_status_is_fresh(self, now: Time) -> bool:
        if self.last_leader_status_stamp is None:
            return False
        age_s = (now - self.last_leader_status_stamp).nanoseconds * 1e-9
        return age_s <= self.pose_timeout_s

    def detection_is_fresh(self, now: Time) -> bool:
        if self.last_detection is None or self.last_detection_stamp is None:
            return False
        age_s = (now - self.last_detection_stamp).nanoseconds * 1e-9
        return age_s <= self.image_center_correction_timeout_s

    def leader_pose_is_trackable(self, now: Time) -> bool:
        if not self.leader_pose_is_fresh(now):
            return False
        if self.leader_input_type != "pose":
            return True
        if self.leader_status_topic:
            if self.last_leader_status_state is None or not self.leader_status_is_fresh(now):
                return False
        return self.last_leader_status_state in TRACKABLE_ESTIMATOR_STATES

    def leader_pose_is_tilt_trackable(self, now: Time) -> bool:
        # Keep vertical framing tied to the freshest estimate pose even when the
        # estimator status briefly drops out. Pan remains on the stricter
        # trackable gate to avoid yaw swings from weak estimates.
        return self.leader_pose_is_fresh(now)

    def _prefer_uav_cmd_pose(self, now: Time) -> bool:
        return should_prefer_command_pose(
            have_cmd=self.have_uav_cmd,
            cmd_stamp_ns=(
                None if self.last_uav_cmd_stamp is None else int(self.last_uav_cmd_stamp.nanoseconds)
            ),
            have_actual=self.have_uav_actual,
            actual_stamp_ns=(
                None
                if self.last_uav_actual_stamp is None
                else int(self.last_uav_actual_stamp.nanoseconds)
            ),
            now_ns=int(now.nanoseconds),
            pose_timeout_s=self.pose_timeout_s,
        )

    def _tracking_uav_pose(self, now: Time) -> Optional[Pose2D]:
        if self._prefer_uav_cmd_pose(now):
            return self.uav_cmd_pose
        if self.have_uav_actual:
            return self.uav_actual_pose
        if self.have_uav_cmd:
            return self.uav_cmd_pose
        return None

    def _tracking_uav_z(self, now: Time) -> Optional[float]:
        if self._prefer_uav_cmd_pose(now):
            return self.uav_cmd_z
        if self.have_uav_actual:
            return self.uav_actual_z
        if self.have_uav_cmd:
            return self.uav_cmd_z
        return None

    def _tracking_uav_pose_source(self, now: Time) -> str:
        if self._prefer_uav_cmd_pose(now):
            return "cmd_pose"
        if self.have_uav_actual:
            return "actual_pose"
        if self.have_uav_cmd:
            return "cmd_pose"
        return "none"

    @staticmethod
    def _clamp_symmetric(value: float, limit: float) -> float:
        limit = max(0.0, float(limit))
        if limit <= 0.0:
            return 0.0
        return max(-limit, min(limit, float(value)))

    def _image_center_corrections(
        self,
        now: Time,
    ) -> tuple[float, float, Optional[float], Optional[float]]:
        if not self.image_center_correction_enable:
            return 0.0, 0.0, None, None
        if not self.detection_is_fresh(now):
            return 0.0, 0.0, None, None
        if (
            self.camera_fx is None
            or self.camera_fy is None
            or self.camera_cx is None
            or self.camera_cy is None
            or self.last_detection is None
        ):
            return 0.0, 0.0, None, None

        det = self.last_detection
        err_x_deg = math.degrees(math.atan2((det.u - self.camera_cx) / self.camera_fx, 1.0))
        err_y_deg = math.degrees(math.atan2((det.v - self.camera_cy) / self.camera_fy, 1.0))
        if abs(err_x_deg) < self.image_center_deadband_deg:
            err_x_deg = 0.0
        if abs(err_y_deg) < self.image_center_deadband_deg:
            err_y_deg = 0.0

        pan_corr_deg = self._clamp_symmetric(
            -self.pan_image_center_gain * err_x_deg,
            self.pan_image_center_max_deg,
        )
        tilt_corr_deg = self._clamp_symmetric(
            -self.tilt_image_center_gain * err_y_deg,
            self.tilt_image_center_max_deg,
        )
        return pan_corr_deg, tilt_corr_deg, err_x_deg, err_y_deg

    def _leader_look_target_xyz_for(self, leader_pose: Pose2D, leader_z: float) -> tuple[float, float, float]:
        return compute_leader_look_target(
            leader_pose.x,
            leader_pose.y,
            leader_pose.yaw,
            leader_z,
            self.leader_look_target_x_m,
            self.leader_look_target_y_m,
            self.camera_look_target_z_m,
        )

    def _leader_look_target_xyz(self) -> tuple[float, float, float]:
        return self._leader_look_target_xyz_for(self.leader_pose, self.leader_z)

    def _compute_tilt_deg_for_target(
        self,
        uav_pose: Pose2D,
        uav_z: float,
        leader_pose: Pose2D,
        leader_z: float,
    ) -> float:
        target_x, target_y, target_z = self._leader_look_target_xyz_for(leader_pose, leader_z)
        return compute_camera_tilt_deg(
            uav_pose.x,
            uav_pose.y,
            uav_z,
            uav_pose.yaw,
            target_x,
            target_y,
            target_z,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
            self.camera_z_offset_m,
            0.0,
        )

    def _compute_tilt_deg(self, uav_pose: Pose2D, uav_z: float) -> float:
        return self._compute_tilt_deg_for_target(uav_pose, uav_z, self.leader_pose, self.leader_z)

    def _command_tilt_deg(self, uav_pose: Pose2D, uav_z: float) -> float:
        if not self.tilt_enable:
            return self.default_tilt_deg
        target_tilt_deg = self._compute_tilt_deg(uav_pose, uav_z)
        if self.uav_camera_mode == "integrated_joint":
            # The attached gimbal pitch command is relative to the fixed camera
            # mount angle baked into the spawned UAV model.
            return target_tilt_deg + self.camera_mount_pitch_deg
        return target_tilt_deg

    def _compute_pan_deg_for_target(
        self,
        uav_pose: Pose2D,
        leader_pose: Pose2D,
        leader_z: float,
    ) -> float:
        if not self.pan_enable:
            return self.default_pan_deg
        camera_x, camera_y = camera_xy_from_uav_pose(
            uav_pose.x,
            uav_pose.y,
            uav_pose.yaw,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )
        target_x, target_y, _ = self._leader_look_target_xyz_for(leader_pose, leader_z)
        target_yaw = math.atan2(target_y - camera_y, target_x - camera_x)
        # Pan command is the raw residual from current UAV yaw to target yaw.
        # The simulator applies camera_pan_sign and camera_yaw_offset_deg when
        # converting this command into the rendered camera pose.
        pan_rad = wrap_pi(target_yaw - uav_pose.yaw)
        return math.degrees(pan_rad)

    def _compute_pan_deg(self, uav_pose: Pose2D) -> float:
        return self._compute_pan_deg_for_target(uav_pose, self.leader_pose, self.leader_z)

    def _publish_camera_debug_for_target(
        self,
        uav_pose: Pose2D,
        uav_z: float,
        leader_pose: Pose2D,
        leader_z: float,
    ) -> None:
        if (
            self.target_camera_pose_pub is None
            or self.target_look_at_point_pub is None
            or self.target_camera_world_yaw_pub is None
        ):
            return
        camera_x, camera_y = camera_xy_from_uav_pose(
            uav_pose.x,
            uav_pose.y,
            uav_pose.yaw,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )
        target_x, target_y, target_z = self._leader_look_target_xyz_for(leader_pose, leader_z)
        target_yaw = math.atan2(target_y - camera_y, target_x - camera_x)
        target_tilt = self._compute_tilt_deg_for_target(uav_pose, uav_z, leader_pose, leader_z)
        quat = quaternion_from_euler(0.0, -math.radians(target_tilt), target_yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = float(camera_x)
        pose_msg.pose.position.y = float(camera_y)
        pose_msg.pose.position.z = float(uav_z - self.camera_z_offset_m)
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.target_camera_pose_pub.publish(pose_msg)

        point_msg = PointStamped()
        point_msg.header.stamp = pose_msg.header.stamp
        point_msg.header.frame_id = "odom"
        point_msg.point.x = float(target_x)
        point_msg.point.y = float(target_y)
        point_msg.point.z = float(target_z)
        self.target_look_at_point_pub.publish(point_msg)

        yaw_msg = Float32()
        yaw_msg.data = float(target_yaw)
        self.target_camera_world_yaw_pub.publish(yaw_msg)

    def _publish_camera_debug(self, uav_pose: Pose2D, uav_z: float) -> None:
        self._publish_camera_debug_for_target(uav_pose, uav_z, self.leader_pose, self.leader_z)

    def _publish_tilt_debug(
        self,
        *,
        uav_pose: Pose2D,
        uav_z: float,
        uav_pose_source: str,
        tilt_mode: str,
        target_cmd_deg: Optional[float],
        leader_pose: Optional[Pose2D],
        leader_z: Optional[float],
    ) -> None:
        if (
            self.tilt_mode_pub is None
            or self.tilt_target_cmd_pub is None
            or self.tilt_hdist_pub is None
            or self.tilt_vdelta_pub is None
            or self.tracking_uav_pose_source_pub is None
        ):
            return

        mode_msg = String()
        mode_msg.data = str(tilt_mode)
        self.tilt_mode_pub.publish(mode_msg)

        src_msg = String()
        src_msg.data = str(uav_pose_source)
        self.tracking_uav_pose_source_pub.publish(src_msg)

        target_cmd_msg = Float32()
        target_cmd_msg.data = float("nan") if target_cmd_deg is None else float(target_cmd_deg)
        self.tilt_target_cmd_pub.publish(target_cmd_msg)

        hdist_msg = Float32()
        vdelta_msg = Float32()
        if leader_pose is None or leader_z is None:
            hdist_msg.data = float("nan")
            vdelta_msg.data = float("nan")
        else:
            camera_x, camera_y = camera_xy_from_uav_pose(
                uav_pose.x,
                uav_pose.y,
                uav_pose.yaw,
                self.camera_x_offset_m,
                self.camera_y_offset_m,
            )
            target_x, target_y, target_z = self._leader_look_target_xyz_for(leader_pose, leader_z)
            hdist_msg.data = float(math.hypot(target_x - camera_x, target_y - camera_y))
            vdelta_msg.data = float(target_z - (uav_z - self.camera_z_offset_m))
        self.tilt_hdist_pub.publish(hdist_msg)
        self.tilt_vdelta_pub.publish(vdelta_msg)

    def _publish_image_correction_debug(
        self,
        *,
        pan_correction_deg: float,
        tilt_correction_deg: float,
        err_x_deg: Optional[float],
        err_y_deg: Optional[float],
    ) -> None:
        if (
            self.image_error_x_pub is None
            or self.image_error_y_pub is None
            or self.pan_image_correction_pub is None
            or self.tilt_image_correction_pub is None
        ):
            return

        err_x_msg = Float32()
        err_x_msg.data = float("nan") if err_x_deg is None else float(err_x_deg)
        self.image_error_x_pub.publish(err_x_msg)

        err_y_msg = Float32()
        err_y_msg.data = float("nan") if err_y_deg is None else float(err_y_deg)
        self.image_error_y_pub.publish(err_y_msg)

        pan_msg = Float32()
        pan_msg.data = float(pan_correction_deg)
        self.pan_image_correction_pub.publish(pan_msg)

        tilt_msg = Float32()
        tilt_msg.data = float(tilt_correction_deg)
        self.tilt_image_correction_pub.publish(tilt_msg)

    def on_tick(self) -> None:
        now = self.get_clock().now()
        uav_pose = self._tracking_uav_pose(now)
        uav_z = self._tracking_uav_z(now)
        if uav_pose is None or uav_z is None:
            return
        uav_pose_source = self._tracking_uav_pose_source(now)
        pan_image_correction_deg, tilt_image_correction_deg, image_err_x_deg, image_err_y_deg = (
            self._image_center_corrections(now)
        )
        leader_trackable = self.leader_pose_is_trackable(now)
        leader_tilt_trackable = self.leader_pose_is_tilt_trackable(now)
        tracked_leader_pose: Optional[Pose2D] = None
        tracked_leader_z: Optional[float] = None
        tilt_leader_pose: Optional[Pose2D] = None
        tilt_leader_z: Optional[float] = None
        if leader_trackable:
            self.last_trackable_leader_pose = Pose2D(
                self.leader_pose.x,
                self.leader_pose.y,
                self.leader_pose.yaw,
            )
            self.last_trackable_leader_z = float(self.leader_z)
            tracked_leader_pose = self.last_trackable_leader_pose
            tracked_leader_z = self.last_trackable_leader_z
        if leader_tilt_trackable:
            tilt_leader_pose = Pose2D(
                self.leader_pose.x,
                self.leader_pose.y,
                self.leader_pose.yaw,
            )
            tilt_leader_z = float(self.leader_z)

        if tracked_leader_pose is not None and tracked_leader_z is not None:
            self._publish_camera_debug_for_target(uav_pose, uav_z, tracked_leader_pose, tracked_leader_z)

        tilt_msg = Float64()
        tilt_mode = "default"
        target_tilt_cmd_deg: Optional[float] = None
        if self.tilt_enable and tilt_leader_pose is not None and tilt_leader_z is not None:
            raw_tilt_cmd = self._compute_tilt_deg_for_target(
                uav_pose,
                uav_z,
                tilt_leader_pose,
                tilt_leader_z,
            )
            if self.uav_camera_mode == "integrated_joint":
                raw_tilt_cmd += self.camera_mount_pitch_deg
            raw_tilt_cmd += tilt_image_correction_deg
            target_tilt_cmd_deg = float(raw_tilt_cmd)
            tilt_mode = "track"
            if (
                self.last_tilt_cmd_deg is not None
                and self.tilt_deadband_deg > 0.0
                and abs(raw_tilt_cmd - self.last_tilt_cmd_deg) < self.tilt_deadband_deg
            ):
                tilt_mode = "track_deadband_hold"
            tilt_msg.data = apply_deadband_command(
                raw_tilt_cmd,
                self.last_tilt_cmd_deg,
                self.tilt_deadband_deg,
            )
        elif self.tilt_enable and self.last_tilt_cmd_deg is not None:
            tilt_mode = "hold_last"
            target_tilt_cmd_deg = float(self.last_tilt_cmd_deg)
            tilt_msg.data = float(self.last_tilt_cmd_deg)
        else:
            target_tilt_cmd_deg = float(self.default_tilt_deg)
            tilt_msg.data = float(self.default_tilt_deg)
        self.tilt_pub.publish(tilt_msg)
        self.last_tilt_cmd_deg = float(tilt_msg.data)
        self._publish_tilt_debug(
            uav_pose=uav_pose,
            uav_z=uav_z,
            uav_pose_source=uav_pose_source,
            tilt_mode=tilt_mode,
            target_cmd_deg=target_tilt_cmd_deg,
            leader_pose=tilt_leader_pose,
            leader_z=tilt_leader_z,
        )
        self._publish_image_correction_debug(
            pan_correction_deg=pan_image_correction_deg,
            tilt_correction_deg=tilt_image_correction_deg,
            err_x_deg=image_err_x_deg,
            err_y_deg=image_err_y_deg,
        )

        pan_msg = Float64()
        if self.pan_enable and tracked_leader_pose is not None and tracked_leader_z is not None:
            pan_msg.data = float(
                self._compute_pan_deg_for_target(
                    uav_pose,
                    tracked_leader_pose,
                    tracked_leader_z,
                )
                + pan_image_correction_deg
            )
        elif self.pan_enable and self.last_pan_cmd_deg is not None:
            pan_msg.data = float(self.last_pan_cmd_deg)
        else:
            pan_msg.data = float(self.default_pan_deg)
        self.pan_pub.publish(pan_msg)
        self.last_pan_cmd_deg = float(pan_msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTracker()
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
