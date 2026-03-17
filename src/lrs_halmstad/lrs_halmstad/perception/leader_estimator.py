#!/usr/bin/env python3
from __future__ import annotations

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String
from vision_msgs.msg import Detection2DArray

from lrs_halmstad.common.selected_target_state import (
    SelectedTargetState,
    encode_selected_target_state_msg,
)
from lrs_halmstad.follow.follow_math import coerce_bool, quat_from_yaw, wrap_pi, yaw_from_quat
from lrs_halmstad.perception.detection_protocol import Detection2D, decode_detection_payload
from lrs_halmstad.perception.detection_status import overlay_lines_from_status
from lrs_halmstad.perception.yolo_common import cv2, image_to_bgr, order_quad

@dataclass
class Pose2D:
    x: float
    y: float
    z: float
    yaw: float


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


class LeaderEstimator(Node):
    """Estimator-only node that turns external detections into a world-frame leader pose."""

    def __init__(self):
        super().__init__("leader_estimator")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("uav_pose_topic", "")
        self.declare_parameter("camera_tilt_topic", "")
        self.declare_parameter("camera_world_yaw_topic", "")
        self.declare_parameter("external_detection_topic", "/coord/leader_detection")
        self.declare_parameter("external_detection_status_topic", "/coord/leader_detection_status")
        self.declare_parameter("out_topic", "/coord/leader_estimate")
        self.declare_parameter("status_topic", "/coord/leader_estimate_status")
        self.declare_parameter("fault_status_topic", "/coord/leader_estimate_fault")
        self.declare_parameter("estimate_error_topic", "/coord/leader_estimate_error")
        self.declare_parameter("selected_target_topic", "/coord/leader_selected_target")
        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("debug_image_topic", "/coord/leader_debug_image")

        self.declare_parameter("publish_status", True)
        self.declare_parameter("publish_fault_status", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("publish_selected_target", True)
        self.declare_parameter("publish_events", False)

        self.declare_parameter("leader_actual_pose_enable", True)
        self.declare_parameter("leader_actual_pose_topic", "/a201_0000/amcl_pose_odom")
        self.declare_parameter("leader_actual_pose_timeout_s", 2.0, dyn_num)

        self.declare_parameter("est_hz", 20.0, dyn_num)
        self.declare_parameter("image_timeout_s", 1.0, dyn_num)
        self.declare_parameter("uav_pose_timeout_s", 1.0, dyn_num)
        self.declare_parameter("external_detection_timeout_s", 1.0, dyn_num)
        self.declare_parameter("camera_orientation_timeout_s", 1.0, dyn_num)

        self.declare_parameter("constant_range_m", 5.0, dyn_num)
        self.declare_parameter("range_mode", "auto")
        self.declare_parameter("use_depth_range", True)
        self.declare_parameter("depth_scale", 0.001, dyn_num)
        self.declare_parameter("depth_min_m", 0.2, dyn_num)
        self.declare_parameter("depth_max_m", 100.0, dyn_num)
        self.declare_parameter("target_ground_z_m", 0.0, dyn_num)
        self.declare_parameter("ground_min_range_m", 0.25, dyn_num)
        self.declare_parameter("ground_max_range_m", 50.0, dyn_num)

        self.declare_parameter("cam_yaw_offset_deg", 0.0, dyn_num)
        self.declare_parameter("cam_pitch_offset_deg", 0.0, dyn_num)
        self.declare_parameter("cam_roll_offset_deg", 0.0, dyn_num)
        self.declare_parameter("cam_x_offset_m", 0.0, dyn_num)
        self.declare_parameter("cam_y_offset_m", 0.0, dyn_num)
        self.declare_parameter("cam_z_offset_m", 0.0, dyn_num)

        self.uav_name = str(self.get_parameter("uav_name").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value).strip() or f"/{self.uav_name}/camera0/image_raw"
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value).strip() or f"/{self.uav_name}/camera0/camera_info"
        self.depth_topic = str(self.get_parameter("depth_topic").value).strip()
        self.uav_pose_topic = str(self.get_parameter("uav_pose_topic").value).strip() or f"/{self.uav_name}/pose"
        self.camera_tilt_topic = (
            str(self.get_parameter("camera_tilt_topic").value).strip()
            or f"/{self.uav_name}/follow/actual/tilt_deg"
        )
        self.camera_world_yaw_topic = (
            str(self.get_parameter("camera_world_yaw_topic").value).strip()
            or f"/{self.uav_name}/camera/actual/world_yaw_rad"
        )
        self.external_detection_topic = str(self.get_parameter("external_detection_topic").value).strip()
        self.external_detection_status_topic = str(self.get_parameter("external_detection_status_topic").value).strip()
        self.out_topic = str(self.get_parameter("out_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.fault_status_topic = str(self.get_parameter("fault_status_topic").value).strip()
        self.estimate_error_topic = str(self.get_parameter("estimate_error_topic").value).strip()
        self.selected_target_topic = str(self.get_parameter("selected_target_topic").value).strip()
        self.event_topic = str(self.get_parameter("event_topic").value).strip()
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value).strip()

        self.publish_status = coerce_bool(self.get_parameter("publish_status").value)
        self.publish_fault_status = coerce_bool(self.get_parameter("publish_fault_status").value)
        self.publish_debug_image = coerce_bool(self.get_parameter("publish_debug_image").value)
        self.publish_selected_target = coerce_bool(self.get_parameter("publish_selected_target").value)
        self.publish_events = coerce_bool(self.get_parameter("publish_events").value)

        self.leader_actual_pose_enable = coerce_bool(self.get_parameter("leader_actual_pose_enable").value)
        self.leader_actual_pose_topic = str(self.get_parameter("leader_actual_pose_topic").value).strip()
        self.leader_actual_pose_timeout_s = float(self.get_parameter("leader_actual_pose_timeout_s").value)

        self.est_hz = float(self.get_parameter("est_hz").value)
        self.image_timeout_s = float(self.get_parameter("image_timeout_s").value)
        self.uav_pose_timeout_s = float(self.get_parameter("uav_pose_timeout_s").value)
        self.external_detection_timeout_s = float(self.get_parameter("external_detection_timeout_s").value)
        self.camera_orientation_timeout_s = float(self.get_parameter("camera_orientation_timeout_s").value)

        self.constant_range_m = float(self.get_parameter("constant_range_m").value)
        self.range_mode = str(self.get_parameter("range_mode").value).strip().lower()
        self.use_depth_range = coerce_bool(self.get_parameter("use_depth_range").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.depth_min_m = float(self.get_parameter("depth_min_m").value)
        self.depth_max_m = float(self.get_parameter("depth_max_m").value)
        self.target_ground_z_m = float(self.get_parameter("target_ground_z_m").value)
        self.ground_min_range_m = float(self.get_parameter("ground_min_range_m").value)
        self.ground_max_range_m = float(self.get_parameter("ground_max_range_m").value)

        self.cam_yaw_offset_rad = math.radians(float(self.get_parameter("cam_yaw_offset_deg").value))
        self.cam_pitch_offset_rad = math.radians(float(self.get_parameter("cam_pitch_offset_deg").value))
        self.cam_roll_offset_rad = math.radians(float(self.get_parameter("cam_roll_offset_deg").value))
        self.cam_x_offset_m = float(self.get_parameter("cam_x_offset_m").value)
        self.cam_y_offset_m = float(self.get_parameter("cam_y_offset_m").value)
        self.cam_z_offset_m = float(self.get_parameter("cam_z_offset_m").value)

        if self.est_hz <= 0.0:
            raise ValueError("est_hz must be > 0")
        if self.image_timeout_s <= 0.0:
            raise ValueError("image_timeout_s must be > 0")
        if self.uav_pose_timeout_s <= 0.0:
            raise ValueError("uav_pose_timeout_s must be > 0")
        if self.external_detection_timeout_s <= 0.0:
            raise ValueError("external_detection_timeout_s must be > 0")
        if self.camera_orientation_timeout_s <= 0.0:
            raise ValueError("camera_orientation_timeout_s must be > 0")
        if self.leader_actual_pose_timeout_s <= 0.0:
            raise ValueError("leader_actual_pose_timeout_s must be > 0")
        if self.constant_range_m <= 0.0:
            raise ValueError("constant_range_m must be > 0")
        if self.range_mode not in ("auto", "depth", "ground", "const"):
            raise ValueError("range_mode must be one of: auto, depth, ground, const")

        self.camera_model: Optional[CameraModel] = None
        self.uav_pose: Optional[Pose2D] = None
        self.last_uav_pose_stamp: Optional[Time] = None
        self.last_image_msg: Optional[Image] = None
        self.last_image_stamp: Optional[Time] = None
        self.last_image_recv_walltime: Optional[float] = None
        self.last_depth_msg: Optional[Image] = None
        self.last_depth_stamp: Optional[Time] = None
        self.last_camera_tilt_deg: Optional[float] = None
        self.last_camera_tilt_stamp: Optional[Time] = None
        self.last_camera_world_yaw_rad: Optional[float] = None
        self.last_camera_world_yaw_stamp: Optional[Time] = None
        self.last_external_det: Optional[Detection2D] = None
        self.last_external_det_stamp: Optional[Time] = None
        self.last_external_det_rx_stamp: Optional[Time] = None
        self.last_external_det_status_text: str = ""
        self.last_external_det_status_stamp: Optional[Time] = None
        self.last_actual_leader_pose: Optional[Pose2D] = None
        self.last_actual_leader_pose_stamp: Optional[Time] = None
        self.last_estimate_pose: Optional[Pose2D] = None
        self.last_estimate_stamp: Optional[Time] = None
        self.last_estimate_track_id: Optional[int] = None

        self.last_estimate_error_dx_m: Optional[float] = None
        self.last_estimate_error_dy_m: Optional[float] = None
        self.last_estimate_error_m: Optional[float] = None
        self.prev_heading_yaw: Optional[float] = None

        self.last_det_conf: float = -1.0
        self.last_latency_ms: float = -1.0
        self.last_range_source: str = "none"
        self.last_range_used_m: float = -1.0
        self.last_bearing_used_deg: float = 0.0
        self.last_heading_source: str = "none"
        self.last_reject_reason: str = "none"
        self.last_camera_orientation_source: str = "static"
        self.last_fault_state: str = "none"
        self.last_fault_reason: str = "none"
        self.last_fault_stamp: Optional[Time] = None
        self.last_debug_state: str = "INIT"
        self.last_debug_det: Optional[Detection2D] = None
        self.last_camera_frame_id: str = ""

        # ---- estimate caching (prevent re-projection of stale detections) ----
        self._last_projected_det_rx_ns: int = -1
        self._cached_estimate_pose: Optional[Pose2D] = None
        self._cached_estimate_track_id: Optional[int] = None

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.on_image, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.on_depth, 10) if self.depth_topic else None
        self.uav_pose_sub = self.create_subscription(PoseStamped, self.uav_pose_topic, self.on_uav_pose, 10)
        self.camera_tilt_sub = (
            self.create_subscription(Float32, self.camera_tilt_topic, self.on_camera_tilt, 10)
            if self.camera_tilt_topic
            else None
        )
        self.camera_world_yaw_sub = (
            self.create_subscription(Float32, self.camera_world_yaw_topic, self.on_camera_world_yaw, 10)
            if self.camera_world_yaw_topic
            else None
        )
        self.external_detection_sub = self.create_subscription(String, self.external_detection_topic, self.on_external_detection, 10)
        self.external_detection_status_sub = self.create_subscription(
            String,
            self.external_detection_status_topic,
            self.on_external_detection_status,
            10,
        )
        self.actual_leader_pose_sub = (
            self.create_subscription(Odometry, self.leader_actual_pose_topic, self.on_actual_leader_pose, 10)
            if self.leader_actual_pose_enable
            else None
        )

        self.pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.events_pub = self.create_publisher(String, self.event_topic, 10)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 2) if self.publish_debug_image else None
        self.selected_target_pub = (
            self.create_publisher(Detection2DArray, self.selected_target_topic, 10)
            if self.publish_selected_target and self.selected_target_topic
            else None
        )
        self.estimate_error_pub = (
            self.create_publisher(Vector3Stamped, self.estimate_error_topic, 10)
            if self.leader_actual_pose_enable
            else None
        )
        fault_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.fault_status_pub = (
            self.create_publisher(String, self.fault_status_topic, fault_qos)
            if self.publish_fault_status
            else None
        )

        self.timer = self.create_timer(1.0 / self.est_hz, self.on_tick)

        self.get_logger().info(
            "[leader_estimator] Started: "
            f"image={self.camera_topic}, camera_info={self.camera_info_topic}, depth={self.depth_topic or 'disabled'}, "
            f"uav_pose={self.uav_pose_topic}, detection={self.external_detection_topic}, "
            f"detection_status={self.external_detection_status_topic}, out={self.out_topic}, "
            f"selected_target={self.selected_target_topic or 'disabled'}, "
            f"est_hz={self.est_hz}Hz, range_mode={self.range_mode}, constant_range_m={self.constant_range_m:.2f}"
        )
        self.publish_fault_status_msg(self._fault_line("none", "none", self.get_clock().now()))
        self.emit_event("ESTIMATOR_NODE_START")

    def emit_event(self, name: str) -> None:
        if not self.publish_events:
            return
        msg = String()
        msg.data = str(name)
        self.events_pub.publish(msg)

    def publish_status_msg(self, text: str) -> None:
        if not self.publish_status:
            return
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_fault_status_msg(self, text: str) -> None:
        if not self.publish_fault_status or self.fault_status_pub is None:
            return
        msg = String()
        msg.data = text
        self.fault_status_pub.publish(msg)

    def _fault_age_ms(self, now: Time) -> float:
        if self.last_fault_stamp is None:
            return -1.0
        return max(0.0, (now - self.last_fault_stamp).nanoseconds * 1e-6)

    def _detector_age_ms(self, now: Time) -> float:
        if self.last_external_det_stamp is None:
            return -1.0
        return max(0.0, (now - self.last_external_det_stamp).nanoseconds * 1e-6)

    def _detector_reason(self, now: Time) -> str:
        if self.last_external_det_stamp is None:
            return "none"
        return "ok" if self.external_detection_fresh(now) else "stale"

    def _fault_line(self, state: str, reason: str, now: Time) -> str:
        return (
            f"state={state} reason={reason} fault_age_ms={self._fault_age_ms(now):.1f} "
            f"detector=external detector_reason={self._detector_reason(now)} detector_age_ms={self._detector_age_ms(now):.1f} "
            f"conf={self.last_det_conf:.3f} latency_ms={self.last_latency_ms:.1f} reject_reason={self.last_reject_reason}"
        )

    def _record_fault(self, state: str, reason: str, now: Time) -> None:
        self.last_fault_state = str(state).strip() or "unknown"
        self.last_fault_reason = str(reason).strip() or "none"
        self.last_fault_stamp = now
        self.publish_fault_status_msg(self._fault_line(self.last_fault_state, self.last_fault_reason, now))

    def on_image(self, msg: Image) -> None:
        self.last_image_msg = msg
        self.last_image_recv_walltime = time.monotonic()
        if msg.header.frame_id:
            self.last_camera_frame_id = str(msg.header.frame_id)
        try:
            self.last_image_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_image_stamp = self.get_clock().now()

    def on_depth(self, msg: Image) -> None:
        self.last_depth_msg = msg
        try:
            self.last_depth_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_depth_stamp = self.get_clock().now()

    def on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) < 9:
            return
        fx = float(msg.k[0])
        fy = float(msg.k[4])
        if fx <= 0.0 or fy <= 0.0:
            return
        if msg.header.frame_id:
            self.last_camera_frame_id = str(msg.header.frame_id)
        self.camera_model = CameraModel(
            fx=fx,
            fy=fy,
            cx=float(msg.k[2]),
            cy=float(msg.k[5]),
            width=int(msg.width),
            height=int(msg.height),
        )

    def on_uav_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_pose = Pose2D(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            yaw=yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        try:
            self.last_uav_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_pose_stamp = self.get_clock().now()

    def on_actual_leader_pose(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.last_actual_leader_pose = Pose2D(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            yaw=yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        try:
            self.last_actual_leader_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_actual_leader_pose_stamp = self.get_clock().now()

    def on_external_detection(self, msg: String) -> None:
        try:
            det_msg = decode_detection_payload(msg.data)
        except ValueError:
            return
        now = self.get_clock().now()
        stamp_ns = det_msg.stamp_ns
        if stamp_ns > 0:
            stamp = Time(nanoseconds=stamp_ns, clock_type=self.get_clock().clock_type)
        else:
            stamp = now
        # Only latch valid detections so that NO_DET frames do not erase the
        # last real detection before external_detection_timeout_s expires.
        if det_msg.detection is not None:
            self.last_external_det_stamp = stamp
            self.last_external_det_rx_stamp = now
            self.last_external_det = det_msg.detection

    def on_external_detection_status(self, msg: String) -> None:
        self.last_external_det_status_text = str(msg.data)
        self.last_external_det_status_stamp = self.get_clock().now()

    def on_camera_tilt(self, msg: Float32) -> None:
        self.last_camera_tilt_deg = float(msg.data)
        self.last_camera_tilt_stamp = self.get_clock().now()

    def on_camera_world_yaw(self, msg: Float32) -> None:
        self.last_camera_world_yaw_rad = wrap_pi(float(msg.data))
        self.last_camera_world_yaw_stamp = self.get_clock().now()

    def _is_fresh(self, last_stamp: Optional[Time], timeout_s: float, now: Time) -> bool:
        if last_stamp is None:
            return False
        return (now - last_stamp).nanoseconds * 1e-9 <= timeout_s

    def image_fresh(self, now: Time) -> bool:
        return self._is_fresh(self.last_image_stamp, self.image_timeout_s, now)

    def uav_pose_fresh(self, now: Time) -> bool:
        return self.uav_pose is not None and self._is_fresh(self.last_uav_pose_stamp, self.uav_pose_timeout_s, now)

    def actual_leader_pose_fresh(self, now: Time) -> bool:
        if not self.leader_actual_pose_enable:
            return False
        return self._is_fresh(self.last_actual_leader_pose_stamp, self.leader_actual_pose_timeout_s, now)

    def external_detection_fresh(self, now: Time) -> bool:
        return self._is_fresh(self.last_external_det_rx_stamp, self.external_detection_timeout_s, now)

    def _camera_orientation(self, now: Time) -> Tuple[float, float, str]:
        assert self.uav_pose is not None
        tilt_live = (
            self.last_camera_tilt_deg is not None
            and self._is_fresh(self.last_camera_tilt_stamp, self.camera_orientation_timeout_s, now)
        )
        yaw_live = (
            self.last_camera_world_yaw_rad is not None
            and self._is_fresh(self.last_camera_world_yaw_stamp, self.camera_orientation_timeout_s, now)
        )
        pitch_rad = math.radians(float(self.last_camera_tilt_deg)) if tilt_live else self.cam_pitch_offset_rad
        center_world_yaw = (
            float(self.last_camera_world_yaw_rad)
            if yaw_live
            else wrap_pi(self.uav_pose.yaw + self.cam_yaw_offset_rad)
        )
        if tilt_live and yaw_live:
            source = "live"
        elif tilt_live:
            source = "live_tilt"
        elif yaw_live:
            source = "live_yaw"
        else:
            source = "static"
        return pitch_rad, center_world_yaw, source

    def _cam_world_xy(self) -> Tuple[float, float]:
        assert self.uav_pose is not None
        cy = math.cos(self.uav_pose.yaw)
        sy = math.sin(self.uav_pose.yaw)
        return (
            self.uav_pose.x + self.cam_x_offset_m * cy - self.cam_y_offset_m * sy,
            self.uav_pose.y + self.cam_x_offset_m * sy + self.cam_y_offset_m * cy,
        )

    def _image_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        return image_to_bgr(msg)

    def _depth_to_array_m(self, msg: Image) -> Optional[np.ndarray]:
        try:
            h = int(msg.height)
            w = int(msg.width)
            if h <= 0 or w <= 0:
                return None
            if msg.encoding in ("32FC1", "32FC"):
                return np.frombuffer(msg.data, dtype=np.float32).reshape((h, w)).copy()
            if msg.encoding in ("16UC1", "16UC"):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w)).astype(np.float32)
                return arr * self.depth_scale
        except Exception:
            return None
        return None

    def _depth_range_at(self, det: Detection2D) -> Optional[float]:
        if not self.use_depth_range or self.last_depth_msg is None:
            return None
        depth = self._depth_to_array_m(self.last_depth_msg)
        if depth is None:
            return None
        u = int(round(det.u))
        v = int(round(det.v))
        h, w = depth.shape[:2]
        if u < 0 or u >= w or v < 0 or v >= h:
            return None
        patch = depth[max(0, v - 2):min(h, v + 3), max(0, u - 2):min(w, u + 3)]
        patch = patch[np.isfinite(patch)]
        patch = patch[(patch >= self.depth_min_m) & (patch <= self.depth_max_m)]
        if patch.size == 0:
            return None
        return float(np.median(patch))

    def _ground_projection_pixel(self, det: Detection2D) -> Tuple[float, float]:
        if det.obb_corners is not None and len(det.obb_corners) == 4:
            try:
                ordered = self._order_quad(np.asarray(det.obb_corners, dtype=np.float64))
                bottom_edge_idx = max(
                    range(4),
                    key=lambda idx: float(ordered[idx, 1] + ordered[(idx + 1) % 4, 1]),
                )
                p1 = ordered[bottom_edge_idx]
                p2 = ordered[(bottom_edge_idx + 1) % 4]
                return (0.5 * float(p1[0] + p2[0]), 0.5 * float(p1[1] + p2[1]))
            except Exception:
                pass
        try:
            x1, _, x2, y2 = det.bbox
            return (0.5 * float(x1 + x2), float(y2))
        except Exception:
            return (float(det.u), float(det.v))

    def _ground_range_from_pixel(
        self,
        u: float,
        v: float,
        cam: CameraModel,
        camera_pitch_rad: float,
    ) -> Optional[float]:
        assert self.uav_pose is not None
        x_n = (u - cam.cx) / cam.fx
        y_n = (v - cam.cy) / cam.fy
        pitch_img = math.atan2(y_n, math.sqrt(1.0 + x_n * x_n))
        cam_world_z = self.uav_pose.z + self.cam_z_offset_m
        dz = self.target_ground_z_m - cam_world_z
        elev = float(camera_pitch_rad) + pitch_img
        tan_elev = math.tan(elev)
        if abs(tan_elev) < 1e-3:
            return None
        horiz_range = dz / tan_elev
        if not math.isfinite(horiz_range) or horiz_range <= 0.0:
            return None
        if horiz_range < self.ground_min_range_m:
            return None
        if self.ground_max_range_m > 0.0 and horiz_range > self.ground_max_range_m:
            return None
        return float(horiz_range)

    @staticmethod
    def _order_quad(points: np.ndarray) -> np.ndarray:
        return order_quad(points)

    def _unwrap_angle_near(self, raw_angle: float, reference_angle: Optional[float]) -> float:
        if reference_angle is None:
            return float(raw_angle)
        return float(reference_angle + wrap_pi(raw_angle - reference_angle))

    def _ground_point_from_pixel(
        self,
        u: float,
        v: float,
        cam: CameraModel,
        camera_pitch_rad: float,
        camera_center_world_yaw: float,
    ) -> Optional[Tuple[float, float]]:
        assert self.uav_pose is not None
        x_n = (u - cam.cx) / cam.fx
        y_n = (v - cam.cy) / cam.fy
        bearing_world = float(camera_center_world_yaw) - math.atan2(x_n, 1.0)
        pitch_img = math.atan2(y_n, math.sqrt(1.0 + x_n * x_n))
        cam_world_z = self.uav_pose.z + self.cam_z_offset_m
        dz = self.target_ground_z_m - cam_world_z
        elev = float(camera_pitch_rad) + pitch_img
        tan_elev = math.tan(elev)
        if abs(tan_elev) < 1e-3:
            return None
        horiz_range = dz / tan_elev
        if not math.isfinite(horiz_range) or horiz_range <= 0.0:
            return None
        cam_world_x, cam_world_y = self._cam_world_xy()
        return (
            cam_world_x + horiz_range * math.cos(bearing_world),
            cam_world_y + horiz_range * math.sin(bearing_world),
        )

    def _obb_heading_from_corners(
        self,
        corners: Tuple[Tuple[float, float], ...],
        cam: CameraModel,
        now: Time,
    ) -> Optional[float]:
        camera_pitch_rad, camera_center_world_yaw, _orientation_source = self._camera_orientation(now)
        ordered = self._order_quad(np.asarray(corners, dtype=np.float64))
        edges = np.roll(ordered, -1, axis=0) - ordered
        lengths = np.linalg.norm(edges, axis=1)
        if lengths[0] >= lengths[1]:
            axis_p1 = 0.5 * (ordered[0] + ordered[1])
            axis_p2 = 0.5 * (ordered[2] + ordered[3])
        else:
            axis_p1 = 0.5 * (ordered[1] + ordered[2])
            axis_p2 = 0.5 * (ordered[3] + ordered[0])
        gp1 = self._ground_point_from_pixel(
            float(axis_p1[0]),
            float(axis_p1[1]),
            cam,
            camera_pitch_rad,
            camera_center_world_yaw,
        )
        gp2 = self._ground_point_from_pixel(
            float(axis_p2[0]),
            float(axis_p2[1]),
            cam,
            camera_pitch_rad,
            camera_center_world_yaw,
        )
        if gp1 is None or gp2 is None:
            return None
        dx = gp2[0] - gp1[0]
        dy = gp2[1] - gp1[1]
        if math.hypot(dx, dy) <= 1e-6:
            return None
        yaw_a = math.atan2(dy, dx)
        yaw_b = wrap_pi(yaw_a + math.pi)
        if self.prev_heading_yaw is None:
            return yaw_a
        yaw_a = self._unwrap_angle_near(yaw_a, self.prev_heading_yaw)
        yaw_b = self._unwrap_angle_near(yaw_b, self.prev_heading_yaw)
        return yaw_a if abs(yaw_a - self.prev_heading_yaw) <= abs(yaw_b - self.prev_heading_yaw) else yaw_b

    def _ground_reject_reason(self, x: float, y: float, now: Time, track_id: Optional[int]) -> str:
        if self.last_estimate_pose is None or self.last_estimate_stamp is None:
            return "none"
        age_s = max(0.0, (now - self.last_estimate_stamp).nanoseconds * 1e-9)
        freshness_window_s = max(self.external_detection_timeout_s, self.image_timeout_s, self.uav_pose_timeout_s)
        if age_s > freshness_window_s:
            return "none"
        if (
            track_id is not None
            and self.last_estimate_track_id is not None
            and track_id != self.last_estimate_track_id
        ):
            return "none"
        jump_m = math.hypot(x - self.last_estimate_pose.x, y - self.last_estimate_pose.y)
        max_jump_m = 3.0 + 2.0 * age_s
        if jump_m > max_jump_m:
            return "ground_jump"
        return "none"

    def _estimate_range_from_detection(
        self,
        det: Detection2D,
        cam: CameraModel,
        now: Time,
    ) -> Tuple[float, str, float, str, str]:
        assert self.uav_pose is not None
        camera_pitch_rad, camera_center_world_yaw, orientation_source = self._camera_orientation(now)
        center_x_n = (det.u - cam.cx) / cam.fx
        center_bearing_world = camera_center_world_yaw - math.atan2(center_x_n, 1.0)
        center_bearing = wrap_pi(center_bearing_world - self.uav_pose.yaw)
        depth_range = self._depth_range_at(det)
        ground_u: Optional[float] = None
        ground_v: Optional[float] = None
        ground_range: Optional[float] = None
        ground_bearing: Optional[float] = None
        reject_reason = "none"
        cam_world_x, cam_world_y = self._cam_world_xy()
        if self.range_mode in ("ground", "auto"):
            ground_u, ground_v = self._ground_projection_pixel(det)
            ground_x_n = (ground_u - cam.cx) / cam.fx
            ground_bearing_world = camera_center_world_yaw - math.atan2(ground_x_n, 1.0)
            ground_bearing = wrap_pi(ground_bearing_world - self.uav_pose.yaw)
            ground_range = self._ground_range_from_pixel(ground_u, ground_v, cam, camera_pitch_rad)
            if ground_range is not None and ground_bearing is not None:
                ground_x = cam_world_x + ground_range * math.cos(ground_bearing_world)
                ground_y = cam_world_y + ground_range * math.sin(ground_bearing_world)
                reject_reason = self._ground_reject_reason(ground_x, ground_y, now, det.track_id)
                if reject_reason != "none":
                    ground_range = None
                    ground_bearing = None
        if self.range_mode == "depth":
            if depth_range is None:
                raise ValueError("depth_range_invalid")
            range_m = depth_range
            range_source = "depth"
        elif self.range_mode == "ground":
            if ground_range is None:
                self.last_reject_reason = reject_reason
                raise ValueError(reject_reason if reject_reason != "none" else "ground_range_invalid")
            range_m = ground_range
            range_source = "ground"
        elif self.range_mode == "const":
            range_m = self.constant_range_m
            range_source = "const"
        else:
            if depth_range is not None:
                range_m = depth_range
                range_source = "depth"
            elif ground_range is not None:
                range_m = ground_range
                range_source = "ground"
            else:
                range_m = self.constant_range_m
                range_source = "const"

        bearing = ground_bearing if range_source == "ground" and ground_bearing is not None else center_bearing
        return float(range_m), str(range_source), float(bearing), str(reject_reason), str(orientation_source)

    def _selected_target_bbox(self, det: Detection2D) -> Tuple[float, float, float, float, float]:
        if det.obb_corners is not None and len(det.obb_corners) == 4:
            ordered = self._order_quad(np.asarray(det.obb_corners, dtype=np.float64))
            center = np.mean(ordered, axis=0)
            edge01 = ordered[1] - ordered[0]
            edge12 = ordered[2] - ordered[1]
            len01 = float(np.linalg.norm(edge01))
            len12 = float(np.linalg.norm(edge12))
            if len01 >= len12:
                size_x = len01
                size_y = len12
                theta = math.atan2(float(edge01[1]), float(edge01[0]))
            else:
                size_x = len12
                size_y = len01
                theta = math.atan2(float(edge12[1]), float(edge12[0]))
            return (
                float(center[0]),
                float(center[1]),
                float(size_x),
                float(size_y),
                float(theta),
            )
        x1, y1, x2, y2 = det.bbox
        return (
            0.5 * float(x1 + x2),
            0.5 * float(y1 + y2),
            max(0.0, float(x2 - x1)),
            max(0.0, float(y2 - y1)),
            0.0,
        )

    def _selected_target_state(self, now: Time, det: Optional[Detection2D]) -> SelectedTargetState:
        stamp = (
            self.last_external_det_stamp.to_msg()
            if det is not None and self.last_external_det_stamp is not None
            else now.to_msg()
        )
        frame_id = self.last_camera_frame_id
        if det is None:
            return SelectedTargetState(
                stamp=stamp,
                frame_id=frame_id,
                valid=False,
            )

        center_x, center_y, size_x, size_y, theta = self._selected_target_bbox(det)
        projected_range_m: Optional[float] = None
        if self.camera_model is not None and self.image_fresh(now) and self.uav_pose_fresh(now):
            try:
                range_m, range_source, _bearing, _reject_reason, _orientation_source = self._estimate_range_from_detection(
                    det,
                    self.camera_model,
                    now,
                )
                if range_source in ("ground", "depth"):
                    projected_range_m = float(range_m)
            except Exception:
                projected_range_m = None

        cls_label = str(det.cls_name or "")
        if not cls_label and det.cls_id is not None:
            cls_label = str(det.cls_id)
        track_id = "" if det.track_id is None else str(int(det.track_id))
        track_age_s = float(det.track_age_s) if det.track_age_s > 0.0 else None
        return SelectedTargetState(
            stamp=stamp,
            frame_id=frame_id,
            valid=True,
            bbox_center_x_px=center_x,
            bbox_center_y_px=center_y,
            bbox_size_x_px=size_x,
            bbox_size_y_px=size_y,
            bbox_theta_rad=theta,
            confidence=float(det.conf),
            class_id=cls_label,
            track_id=track_id,
            projected_range_m=projected_range_m,
            track_age_s=track_age_s,
        )

    def _publish_selected_target(self, now: Time, det: Optional[Detection2D]) -> None:
        if self.selected_target_pub is None:
            return
        state = self._selected_target_state(now, det)
        self.selected_target_pub.publish(encode_selected_target_state_msg(state))

    def _estimate_from_detection(self, det: Detection2D, cam: CameraModel, now: Time) -> Tuple[Pose2D, str]:
        range_m, range_source, bearing, reject_reason, camera_orientation_source = self._estimate_range_from_detection(det, cam, now)
        cam_world_x, cam_world_y = self._cam_world_xy()
        x = cam_world_x + range_m * math.cos(self.uav_pose.yaw + bearing)
        y = cam_world_y + range_m * math.sin(self.uav_pose.yaw + bearing)

        heading_source = "fallback"
        yaw = self.prev_heading_yaw if self.prev_heading_yaw is not None else self.uav_pose.yaw
        if det.obb_heading_yaw is not None:
            yaw = det.obb_heading_yaw
            heading_source = "obb"
        elif det.obb_corners is not None:
            obb_heading = self._obb_heading_from_corners(det.obb_corners, cam, now)
            if obb_heading is not None:
                yaw = obb_heading
                det.obb_heading_yaw = obb_heading
                heading_source = "obb"

        self.last_det_conf = float(det.conf)
        self.last_range_source = range_source
        self.last_range_used_m = float(range_m)
        self.last_bearing_used_deg = math.degrees(bearing)
        self.last_heading_source = heading_source
        self.last_reject_reason = reject_reason
        self.last_camera_orientation_source = camera_orientation_source
        return Pose2D(x=x, y=y, z=self.target_ground_z_m, yaw=yaw), range_source

    def _publish_estimate(self, pose: Pose2D, now: Time, track_id: Optional[int]) -> None:
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(pose.x)
        msg.pose.position.y = float(pose.y)
        msg.pose.position.z = float(pose.z)
        qx, qy, qz, qw = quat_from_yaw(pose.yaw)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pub.publish(msg)
        self.last_estimate_pose = pose
        self.last_estimate_stamp = now
        self.last_estimate_track_id = track_id
        self.prev_heading_yaw = pose.yaw
        self._publish_estimate_error(pose, now)

    def _publish_estimate_error(self, est_pose: Pose2D, now: Time) -> None:
        if not self.leader_actual_pose_enable or self.estimate_error_pub is None:
            self.last_estimate_error_dx_m = None
            self.last_estimate_error_dy_m = None
            self.last_estimate_error_m = None
            return
        if not self.actual_leader_pose_fresh(now) or self.last_actual_leader_pose is None:
            self.last_estimate_error_dx_m = None
            self.last_estimate_error_dy_m = None
            self.last_estimate_error_m = None
            return
        dx = est_pose.x - self.last_actual_leader_pose.x
        dy = est_pose.y - self.last_actual_leader_pose.y
        planar = math.hypot(dx, dy)
        self.last_estimate_error_dx_m = dx
        self.last_estimate_error_dy_m = dy
        self.last_estimate_error_m = planar
        msg = Vector3Stamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.vector.x = float(dx)
        msg.vector.y = float(dy)
        msg.vector.z = float(planar)
        self.estimate_error_pub.publish(msg)

    def _status_line(self, state: str, reason: str, now: Time) -> str:
        reason = str(reason).strip() or "none"
        err_dx = "na" if self.last_estimate_error_dx_m is None else f"{self.last_estimate_error_dx_m:.2f}"
        err_dy = "na" if self.last_estimate_error_dy_m is None else f"{self.last_estimate_error_dy_m:.2f}"
        err_planar = "na" if self.last_estimate_error_m is None else f"{self.last_estimate_error_m:.2f}"
        return (
            f"state={state} reason={reason} "
            f"detector_reason={self._detector_reason(now)} detector_age_ms={self._detector_age_ms(now):.1f} "
            f"conf={self.last_det_conf:.3f} latency_ms={self.last_latency_ms:.1f} "
            f"range_src={self.last_range_source} range_mode={self.range_mode} range_m={self.last_range_used_m:.2f} "
            f"bearing_deg={self.last_bearing_used_deg:.1f} reject_reason={self.last_reject_reason} "
            f"cam_orient_src={self.last_camera_orientation_source} "
            f"heading_src={self.last_heading_source} "
            f"err_dx_m={err_dx} err_dy_m={err_dy} err_planar_m={err_planar}"
        )

    def _detection_status_overlay_lines(self, now: Time) -> list[str]:
        if not self.last_external_det_status_text:
            return ["det_state=na reason=status_missing"]
        if not self._is_fresh(self.last_external_det_status_stamp, self.external_detection_timeout_s, now):
            return ["det_state=na reason=status_stale"]
        return overlay_lines_from_status(self.last_external_det_status_text) or ["det_state=na reason=status_invalid"]

    def _bgr_to_image_msg(self, img_bgr: np.ndarray) -> Optional[Image]:
        if img_bgr.ndim != 3 or img_bgr.shape[2] != 3:
            return None
        msg = Image()
        if self.last_image_msg is not None:
            msg.header.stamp = self.last_image_msg.header.stamp
            msg.header.frame_id = self.last_image_msg.header.frame_id
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = ""
        msg.height = int(img_bgr.shape[0])
        msg.width = int(img_bgr.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = int(img_bgr.shape[1] * 3)
        msg.data = np.ascontiguousarray(img_bgr).tobytes()
        return msg

    def _debug_color(self, state: str) -> Tuple[int, int, int]:
        if state == "OK":
            return (0, 220, 0)
        if state == "NO_DET":
            return (0, 165, 255)
        return (0, 0, 255)

    def _estimate_error_line(self) -> str:
        err_dx = "na" if self.last_estimate_error_dx_m is None else f"{self.last_estimate_error_dx_m:.2f}"
        err_dy = "na" if self.last_estimate_error_dy_m is None else f"{self.last_estimate_error_dy_m:.2f}"
        err_planar = "na" if self.last_estimate_error_m is None else f"{self.last_estimate_error_m:.2f}"
        return f"err=({err_dx},{err_dy},{err_planar})"

    def _publish_debug_image(self, state: str, det: Optional[Detection2D]) -> None:
        if not self.publish_debug_image or self.debug_image_pub is None or self.last_image_msg is None:
            return
        img_bgr = self._image_to_bgr(self.last_image_msg)
        if img_bgr is None:
            return
        try:
            out = img_bgr.copy()
            if cv2 is not None:
                color = self._debug_color(state)
                if det is not None:
                    x1, y1, x2, y2 = det.bbox
                    cv2.rectangle(out, (int(round(x1)), int(round(y1))), (int(round(x2)), int(round(y2))), color, 2)
                    if det.obb_corners is not None:
                        pts = np.asarray(det.obb_corners, dtype=np.int32).reshape((-1, 1, 2))
                        cv2.polylines(out, [pts], True, (255, 255, 0), 2, cv2.LINE_AA)
                    cv2.circle(out, (int(round(det.u)), int(round(det.v))), 3, color, -1)
                lines = [
                    f"state={state} conf={self.last_det_conf:.2f} latency_ms={self.last_latency_ms:.1f}",
                    f"est_range={self.last_range_used_m:.2f}m src={self.last_range_source} bearing={self.last_bearing_used_deg:.1f}deg",
                    f"cam_orient_src={self.last_camera_orientation_source}",
                    f"heading_src={self.last_heading_source}",
                ]
                lines.extend(self._detection_status_overlay_lines(self.get_clock().now()))
                if self.last_estimate_pose is not None:
                    lines.append(f"est=({self.last_estimate_pose.x:.2f},{self.last_estimate_pose.y:.2f},{self.last_estimate_pose.yaw:.2f})")
                    lines.append(self._estimate_error_line())
                y = 20
                for line in lines:
                    cv2.putText(out, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 3, cv2.LINE_AA)
                    cv2.putText(out, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
                    y += 18
            msg = self._bgr_to_image_msg(out)
            if msg is not None:
                self.debug_image_pub.publish(msg)
        except Exception as exc:
            self.get_logger().warn(f"[leader_estimator] debug image publish failed: {exc}")

    def on_tick(self) -> None:
        now = self.get_clock().now()
        det = self.last_external_det if self.external_detection_fresh(now) else None
        self.last_latency_ms = self._detector_age_ms(now)
        self.last_reject_reason = "none"
        self.last_debug_state = "INIT"
        self.last_debug_det = det
        self._publish_selected_target(now, det)

        # ---- estimate caching: skip re-projection of stale detections ----
        det_rx_ns = (self.last_external_det_rx_stamp.nanoseconds
                     if self.last_external_det_rx_stamp is not None else -1)
        detection_is_new = (det is not None
                            and det_rx_ns != self._last_projected_det_rx_ns)

        if not detection_is_new and det is not None and self._cached_estimate_pose is not None:
            # Same detection as last projection — republish cached estimate
            self._publish_estimate(
                self._cached_estimate_pose, now, det.track_id)
            self.publish_status_msg(
                self._status_line("OK", "estimate_cached", now))
            self.last_debug_state = "OK"
            self._publish_debug_image("OK", det)
            return

        if self.camera_model is None:
            self.last_det_conf = -1.0
            self.last_range_source = "none"
            self.last_range_used_m = -1.0
            self.last_heading_source = "none"
            self.last_camera_orientation_source = "static"
            self.publish_status_msg(self._status_line("STALE", "camera_info_missing", now))
            self._record_fault("STALE", "camera_info_missing", now)
            self._publish_debug_image("STALE", det)
            return
        if not self.image_fresh(now):
            self.last_det_conf = -1.0
            self.last_range_source = "none"
            self.last_range_used_m = -1.0
            self.last_heading_source = "none"
            self.last_camera_orientation_source = "static"
            self.publish_status_msg(self._status_line("STALE", "image_stale", now))
            self._record_fault("STALE", "image_stale", now)
            self._publish_debug_image("STALE", det)
            return
        if not self.uav_pose_fresh(now):
            self.last_det_conf = -1.0
            self.last_range_source = "none"
            self.last_range_used_m = -1.0
            self.last_heading_source = "none"
            self.last_camera_orientation_source = "static"
            self.publish_status_msg(self._status_line("STALE", "uav_pose_stale", now))
            self._record_fault("STALE", "uav_pose_stale", now)
            self._publish_debug_image("STALE", det)
            return
        if det is None:
            self.last_det_conf = -1.0
            self.last_range_source = "none"
            self.last_range_used_m = -1.0
            self.last_heading_source = "none"
            self.last_camera_orientation_source = "static"
            reason = "no_detection" if self.last_external_det_stamp is not None else "detection_missing"
            self.publish_status_msg(self._status_line("NO_DET", reason, now))
            self._record_fault("NO_DET", reason, now)
            self._publish_debug_image("NO_DET", None)
            return

        try:
            pose, _ = self._estimate_from_detection(det, self.camera_model, now)
        except ValueError as exc:
            reason = str(exc) or "estimate_failed"
            self.last_heading_source = "none"
            self.publish_status_msg(self._status_line("STALE", reason, now))
            self._record_fault("STALE", reason, now)
            self._publish_debug_image("STALE", det)
            return
        except Exception as exc:
            reason = f"estimate_failed:{exc}"
            self.last_heading_source = "none"
            self.publish_status_msg(self._status_line("STALE", reason, now))
            self._record_fault("STALE", reason, now)
            self._publish_debug_image("STALE", det)
            return

        self._cached_estimate_pose = pose
        self._cached_estimate_track_id = det.track_id
        self._last_projected_det_rx_ns = det_rx_ns

        self._publish_estimate(pose, now, det.track_id)
        self.publish_status_msg(self._status_line("OK", "none", now))
        self.last_debug_state = "OK"
        self.last_debug_det = det
        self._publish_debug_image("OK", det)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.emit_event("ESTIMATOR_NODE_SHUTDOWN")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
