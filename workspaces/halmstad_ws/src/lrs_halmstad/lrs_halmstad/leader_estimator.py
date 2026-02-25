#!/usr/bin/env python3
from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from rcl_interfaces.msg import ParameterDescriptor

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None

try:
    from ultralytics import YOLO  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    YOLO = None


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


@dataclass
class Detection2D:
    u: float
    v: float
    conf: float
    bbox: Tuple[float, float, float, float]
    cls_id: Optional[int] = None
    cls_name: str = ""


def clamp01(x: float) -> float:
    return max(0.0, min(1.0, float(x)))


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    # Local yaw-only helpers avoid tf_transformations/transforms3d runtime issues
    # with newer NumPy, while covering this node's current 2D heading needs.
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


class LeaderEstimator(Node):
    """
    Vision-based leader estimator (YOLO in this node only).
    - Subscribes to camera image (+ camera_info, optional depth) and UAV pose
    - Publishes leader estimate as PoseStamped on /coord/leader_estimate
    - Publishes status string on /coord/leader_estimate_status including confidence + latency
    - Minimal bring-up path: bearing-only with constant range
    - Optional depth-based range if a depth topic is configured and available
    """

    def __init__(self):
        super().__init__("leader_estimator")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        # Parameters
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("depth_topic", "")
        self.declare_parameter("uav_pose_topic", "")
        self.declare_parameter("out_topic", "/coord/leader_estimate")
        self.declare_parameter("status_topic", "/coord/leader_estimate_status")
        self.declare_parameter("publish_status", True)

        self.declare_parameter("est_hz", 5.0, dyn_num)
        self.declare_parameter("image_timeout_s", 1.0)
        self.declare_parameter("uav_pose_timeout_s", 1.0)

        # YOLO and detection config
        self.declare_parameter("yolo_weights", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("conf_threshold", 0.25, dyn_num)
        self.declare_parameter("iou_threshold", 0.45, dyn_num)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("target_class_name", "")
        self.declare_parameter("smooth_alpha", 0.35, dyn_num)
        self.declare_parameter("max_hold_frames", 3, dyn_num)
        self.declare_parameter("max_hold_s", 0.5, dyn_num)
        self.declare_parameter("xy_smooth_alpha", 0.5, dyn_num)
        self.declare_parameter("max_xy_jump_m", 6.0, dyn_num)
        self.declare_parameter("max_target_speed_mps", 8.0, dyn_num)
        self.declare_parameter("max_bearing_jump_deg", 40.0, dyn_num)
        self.declare_parameter("ok_debounce_frames", 2, dyn_num)
        self.declare_parameter("bad_debounce_frames", 2, dyn_num)
        self.declare_parameter("bbox_continuity_weight", 0.15, dyn_num)
        self.declare_parameter("bbox_continuity_class_bonus", 0.05, dyn_num)
        self.declare_parameter("bbox_continuity_max_px", 400.0, dyn_num)
        self.declare_parameter("tracker_enable", True)
        self.declare_parameter("tracker_alpha", 0.45, dyn_num)
        self.declare_parameter("tracker_beta", 0.10, dyn_num)
        self.declare_parameter("tracker_min_gain_scale", 0.35, dyn_num)
        self.declare_parameter("tracker_quality_latency_ref_ms", 180.0, dyn_num)
        self.declare_parameter("latency_comp_enable", True)
        self.declare_parameter("latency_comp_scale", 1.0, dyn_num)
        self.declare_parameter("latency_comp_max_s", 0.25, dyn_num)

        # Range and geometry
        self.declare_parameter("constant_range_m", 8.0, dyn_num)
        self.declare_parameter("range_mode", "auto")  # auto|depth|ground|const
        self.declare_parameter("use_depth_range", True)
        self.declare_parameter("depth_scale", 0.001, dyn_num)  # for 16UC1 mm -> m
        self.declare_parameter("depth_min_m", 0.2, dyn_num)
        self.declare_parameter("depth_max_m", 100.0, dyn_num)
        self.declare_parameter("target_ground_z_m", 0.0, dyn_num)
        self.declare_parameter("ground_min_range_m", 2.0, dyn_num)
        self.declare_parameter("ground_max_range_m", 50.0, dyn_num)
        self.declare_parameter("cam_yaw_offset_deg", 0.0, dyn_num)
        self.declare_parameter("cam_pitch_offset_deg", -90.0, dyn_num)
        self.declare_parameter("cam_roll_offset_deg", 0.0, dyn_num)
        self.declare_parameter("cam_x_offset_m", 0.0, dyn_num)
        self.declare_parameter("cam_y_offset_m", 0.0, dyn_num)
        self.declare_parameter("cam_z_offset_m", 0.0, dyn_num)
        self.declare_parameter("bootstrap_uav_pose_enabled", True)
        self.declare_parameter("bootstrap_uav_x", 0.0, dyn_num)
        self.declare_parameter("bootstrap_uav_y", 0.0, dyn_num)
        self.declare_parameter("bootstrap_uav_yaw", 0.0, dyn_num)

        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("publish_events", True)

        # Read params
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value) or f"/{self.uav_name}/camera0/image_raw"
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value) or f"/{self.uav_name}/camera0/camera_info"
        self.depth_topic = str(self.get_parameter("depth_topic").value)
        self.uav_pose_topic = str(self.get_parameter("uav_pose_topic").value) or f"/{self.uav_name}/pose_cmd"
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.publish_status = bool(self.get_parameter("publish_status").value)

        self.est_hz = float(self.get_parameter("est_hz").value)
        self.image_timeout_s = float(self.get_parameter("image_timeout_s").value)
        self.uav_pose_timeout_s = float(self.get_parameter("uav_pose_timeout_s").value)

        self.yolo_weights = str(self.get_parameter("yolo_weights").value)
        self.device = str(self.get_parameter("device").value)
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value).strip()
        self.smooth_alpha = float(self.get_parameter("smooth_alpha").value)
        self.max_hold_frames = int(self.get_parameter("max_hold_frames").value)
        self.max_hold_s = float(self.get_parameter("max_hold_s").value)
        self.xy_smooth_alpha = float(self.get_parameter("xy_smooth_alpha").value)
        self.max_xy_jump_m = float(self.get_parameter("max_xy_jump_m").value)
        self.max_target_speed_mps = float(self.get_parameter("max_target_speed_mps").value)
        self.max_bearing_jump_deg = float(self.get_parameter("max_bearing_jump_deg").value)
        self.ok_debounce_frames = int(self.get_parameter("ok_debounce_frames").value)
        self.bad_debounce_frames = int(self.get_parameter("bad_debounce_frames").value)
        self.bbox_continuity_weight = float(self.get_parameter("bbox_continuity_weight").value)
        self.bbox_continuity_class_bonus = float(self.get_parameter("bbox_continuity_class_bonus").value)
        self.bbox_continuity_max_px = float(self.get_parameter("bbox_continuity_max_px").value)
        self.tracker_enable = bool(self.get_parameter("tracker_enable").value)
        self.tracker_alpha = float(self.get_parameter("tracker_alpha").value)
        self.tracker_beta = float(self.get_parameter("tracker_beta").value)
        self.tracker_min_gain_scale = float(self.get_parameter("tracker_min_gain_scale").value)
        self.tracker_quality_latency_ref_ms = float(self.get_parameter("tracker_quality_latency_ref_ms").value)
        self.latency_comp_enable = bool(self.get_parameter("latency_comp_enable").value)
        self.latency_comp_scale = float(self.get_parameter("latency_comp_scale").value)
        self.latency_comp_max_s = float(self.get_parameter("latency_comp_max_s").value)

        self.constant_range_m = float(self.get_parameter("constant_range_m").value)
        self.range_mode = str(self.get_parameter("range_mode").value).strip().lower()
        self.use_depth_range = bool(self.get_parameter("use_depth_range").value)
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
        self.bootstrap_uav_pose_enabled = bool(self.get_parameter("bootstrap_uav_pose_enabled").value)
        self.bootstrap_uav_x = float(self.get_parameter("bootstrap_uav_x").value)
        self.bootstrap_uav_y = float(self.get_parameter("bootstrap_uav_y").value)
        self.bootstrap_uav_yaw = float(self.get_parameter("bootstrap_uav_yaw").value)

        self.event_topic = str(self.get_parameter("event_topic").value)
        self.publish_events = bool(self.get_parameter("publish_events").value)

        if self.est_hz <= 0.0:
            raise ValueError("est_hz must be > 0")
        if self.constant_range_m <= 0.0:
            raise ValueError("constant_range_m must be > 0")
        if not (0.0 <= self.smooth_alpha <= 1.0):
            raise ValueError("smooth_alpha must be in [0,1]")
        if self.max_hold_frames < 0:
            raise ValueError("max_hold_frames must be >= 0")
        if self.max_hold_s < 0.0:
            raise ValueError("max_hold_s must be >= 0")
        if not (0.0 <= self.xy_smooth_alpha <= 1.0):
            raise ValueError("xy_smooth_alpha must be in [0,1]")
        if self.range_mode not in ("auto", "depth", "ground", "const"):
            raise ValueError("range_mode must be one of: auto, depth, ground, const")
        if self.ground_min_range_m < 0.0:
            raise ValueError("ground_min_range_m must be >= 0")
        if self.ground_max_range_m < 0.0:
            raise ValueError("ground_max_range_m must be >= 0")
        if self.ground_max_range_m > 0.0 and self.ground_max_range_m < self.ground_min_range_m:
            raise ValueError("ground_max_range_m must be >= ground_min_range_m (or 0 to disable)")
        if self.ok_debounce_frames < 1:
            raise ValueError("ok_debounce_frames must be >= 1")
        if self.bad_debounce_frames < 1:
            raise ValueError("bad_debounce_frames must be >= 1")
        if self.tracker_alpha < 0.0 or self.tracker_alpha > 1.0:
            raise ValueError("tracker_alpha must be in [0,1]")
        if self.tracker_beta < 0.0 or self.tracker_beta > 1.0:
            raise ValueError("tracker_beta must be in [0,1]")
        if not (0.0 <= self.tracker_min_gain_scale <= 1.0):
            raise ValueError("tracker_min_gain_scale must be in [0,1]")
        if self.tracker_quality_latency_ref_ms <= 1e-3:
            raise ValueError("tracker_quality_latency_ref_ms must be > 0")
        if self.latency_comp_scale < 0.0:
            raise ValueError("latency_comp_scale must be >= 0")
        if self.latency_comp_max_s < 0.0:
            raise ValueError("latency_comp_max_s must be >= 0")

        # State
        self.have_uav_pose = False
        self.have_real_uav_pose = False
        self.uav_pose = Pose2D(0.0, 0.0, 0.0, 0.0)
        self.last_uav_pose_stamp: Optional[Time] = None
        self.uav_pose_source = "none"
        if self.bootstrap_uav_pose_enabled:
            self.have_uav_pose = True
            self.uav_pose = Pose2D(self.bootstrap_uav_x, self.bootstrap_uav_y, 10.0, self.bootstrap_uav_yaw)
            self.last_uav_pose_stamp = self.get_clock().now()
            self.uav_pose_source = "bootstrap"

        self.camera_model: Optional[CameraModel] = None
        self.last_image_msg: Optional[Image] = None
        self.last_image_stamp: Optional[Time] = None
        self.last_depth_msg: Optional[Image] = None
        self.last_depth_stamp: Optional[Time] = None
        self.last_image_recv_walltime: Optional[float] = None

        self.last_pub_time: Optional[Time] = None
        self.last_status: Optional[str] = None
        self.last_det_conf: float = -1.0
        self.last_latency_ms: float = -1.0
        self.last_range_source: str = "none"

        self.prev_estimate_xy: Optional[Tuple[float, float]] = None
        self.prev_estimate_stamp: Optional[Time] = None
        self.prev_heading_yaw: Optional[float] = None
        self.last_good_estimate: Optional[Pose2D] = None
        self.last_good_estimate_stamp: Optional[Time] = None
        self.hold_frames_left = 0
        self.smoothed_bearing: Optional[float] = None
        self.smoothed_range_m: Optional[float] = None
        self.smoothed_xy: Optional[Tuple[float, float]] = None
        self.last_bearing_rad: Optional[float] = None
        self.last_range_used_m: float = -1.0
        self.last_bearing_used_deg: float = 0.0
        self.last_reject_reason: str = "none"
        self.good_det_streak = 0
        self.bad_det_streak = 0
        self.track_latched = False

        self.last_det_center: Optional[Tuple[float, float]] = None
        self.last_det_cls_id: Optional[int] = None

        self.track_x: float = 0.0
        self.track_y: float = 0.0
        self.track_vx: float = 0.0
        self.track_vy: float = 0.0
        self.track_stamp: Optional[Time] = None
        self.track_valid: bool = False

        self.yolo_model = None
        self.yolo_ready = False
        self.yolo_error: Optional[str] = None
        self._init_yolo()

        # I/O
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.on_image, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        if self.depth_topic:
            self.depth_sub = self.create_subscription(Image, self.depth_topic, self.on_depth, 10)
        else:
            self.depth_sub = None
        self.uav_pose_sub = self.create_subscription(PoseStamped, self.uav_pose_topic, self.on_uav_pose, 10)
        self.pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.events_pub = self.create_publisher(String, self.event_topic, 10)

        self.timer = self.create_timer(1.0 / self.est_hz, self.on_tick)
        self.status_timer = self.create_timer(1.0, self.on_status_tick)

        self.get_logger().info(f"[leader_estimator] Using image_topic={self.camera_topic}")
        self.get_logger().info(f"[leader_estimator] Using camera_info_topic={self.camera_info_topic}")
        self.get_logger().info(f"[leader_estimator] Using depth_topic={self.depth_topic or '<disabled>'}")
        self.get_logger().info(f"[leader_estimator] Using uav_pose_topic={self.uav_pose_topic}")
        self.get_logger().info("[leader_estimator] Subscribed ok")

        self.get_logger().info(
            "[leader_estimator] Started: "
            f"image={self.camera_topic}, camera_info={self.camera_info_topic}, depth={self.depth_topic or 'disabled'}, "
            f"uav_pose={self.uav_pose_topic}, out={self.out_topic}, est_hz={self.est_hz}Hz, "
            f"range=constant({self.constant_range_m:.2f}m){'+depth' if self.use_depth_range and self.depth_topic else ''}, "
            f"yolo_weights={self.yolo_weights or '<auto/none>'}, device={self.device}, "
            f"smooth_alpha={self.smooth_alpha}, max_hold_frames={self.max_hold_frames}, max_hold_s={self.max_hold_s}, "
            f"xy_smooth_alpha={self.xy_smooth_alpha}, range_mode={self.range_mode}, "
            f"cam_pitch_deg={math.degrees(self.cam_pitch_offset_rad):.1f}, cam_yaw_deg={math.degrees(self.cam_yaw_offset_rad):.1f}, "
            f"tracker={self.tracker_enable} a={self.tracker_alpha} b={self.tracker_beta}, "
            f"debounce(ok={self.ok_debounce_frames},bad={self.bad_debounce_frames})"
        )
        self.emit_event("ESTIMATOR_NODE_START")

    def _init_yolo(self) -> None:
        if YOLO is None:
            self.yolo_error = "ultralytics_not_installed"
            return
        if not self.yolo_weights:
            self.yolo_error = "yolo_weights_not_set"
            return
        if not os.path.isfile(self.yolo_weights):
            self.yolo_error = f"file_not_found:{self.yolo_weights}"
            self.get_logger().warn(f"[leader_estimator] YOLO weights file not found: {self.yolo_weights}")
            return
        try:
            self.yolo_model = YOLO(self.yolo_weights)
            self.yolo_ready = True
            self.get_logger().info(f"[leader_estimator] YOLO loaded: {self.yolo_weights}")
        except Exception as e:
            self.yolo_error = f"yolo_load_failed:{e}"
            self.get_logger().warn(f"[leader_estimator] Failed to load YOLO weights '{self.yolo_weights}': {e}")

    def emit_event(self, s: str) -> None:
        if not self.publish_events:
            return
        msg = String()
        msg.data = s
        self.events_pub.publish(msg)

    def publish_status_msg(self, s: str) -> None:
        if not self.publish_status:
            return
        msg = String()
        msg.data = s
        self.status_pub.publish(msg)

    def on_image(self, msg: Image) -> None:
        self.last_image_msg = msg
        self.last_image_recv_walltime = time.monotonic()
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
        cx = float(msg.k[2])
        cy = float(msg.k[5])
        if fx <= 0.0 or fy <= 0.0:
            return
        self.camera_model = CameraModel(
            fx=fx,
            fy=fy,
            cx=cx,
            cy=cy,
            width=int(msg.width),
            height=int(msg.height),
        )

    def on_uav_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))

        self.uav_pose = Pose2D(float(p.x), float(p.y), float(p.z), float(yaw))
        self.have_uav_pose = True
        self.have_real_uav_pose = True
        self.uav_pose_source = "pose_cmd"

        try:
            self.last_uav_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_pose_stamp = self.get_clock().now()

    def _is_fresh(self, last_stamp: Optional[Time], timeout_s: float, now: Time) -> bool:
        if last_stamp is None:
            return False
        age_s = (now - last_stamp).nanoseconds * 1e-9
        return age_s <= timeout_s

    def image_fresh(self, now: Time) -> bool:
        return self._is_fresh(self.last_image_stamp, self.image_timeout_s, now)

    def uav_pose_fresh(self, now: Time) -> bool:
        if self.have_real_uav_pose:
            return self.have_uav_pose and self._is_fresh(self.last_uav_pose_stamp, self.uav_pose_timeout_s, now)
        if self.bootstrap_uav_pose_enabled and self.have_uav_pose:
            # Keep bootstrap pose available until the first pose_cmd arrives to break startup deadlock.
            self.last_uav_pose_stamp = now
            self.uav_pose_source = "bootstrap"
            return True
        return False

    def _image_to_bgr(self, msg: Image) -> Optional[np.ndarray]:
        try:
            h = int(msg.height)
            w = int(msg.width)
            if h <= 0 or w <= 0:
                return None
            if msg.encoding == "bgr8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                return arr.copy()
            if msg.encoding == "rgb8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
                if cv2 is not None:
                    return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
                return arr[:, :, ::-1].copy()
            if msg.encoding == "bgra8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                return arr[:, :, :3].copy()
            if msg.encoding == "rgba8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
                rgb = arr[:, :, :3]
                if cv2 is not None:
                    return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                return rgb[:, :, ::-1].copy()
            if msg.encoding == "mono8":
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
                if cv2 is not None:
                    return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
                return np.stack([arr, arr, arr], axis=-1)
        except Exception as e:
            self.get_logger().warn(f"[leader_estimator] Failed image decode: {e}")
            return None
        return None

    def _depth_to_array_m(self, msg: Image) -> Optional[np.ndarray]:
        try:
            h = int(msg.height)
            w = int(msg.width)
            if h <= 0 or w <= 0:
                return None
            if msg.encoding in ("32FC1", "32FC"):
                arr = np.frombuffer(msg.data, dtype=np.float32).reshape((h, w))
                return arr.copy()
            if msg.encoding in ("16UC1", "16UC"):
                arr = np.frombuffer(msg.data, dtype=np.uint16).reshape((h, w)).astype(np.float32)
                arr *= float(self.depth_scale)
                return arr
        except Exception as e:
            self.get_logger().warn(f"[leader_estimator] Failed depth decode: {e}")
            return None
        return None

    def _pick_detection(self, img_bgr: np.ndarray) -> Optional[Detection2D]:
        if not self.yolo_ready or self.yolo_model is None:
            return None
        try:
            results = self.yolo_model.predict(
                source=img_bgr,
                verbose=False,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
            )
        except Exception as e:
            self.yolo_error = f"infer_failed:{e}"
            self.get_logger().warn(f"[leader_estimator] YOLO inference failed: {e}")
            return None
        if not results:
            return None

        result = results[0]
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return None

        names = getattr(result, "names", {}) or {}
        best: Optional[Detection2D] = None
        best_score = -1e9
        for b in boxes:
            try:
                xyxy = b.xyxy[0].tolist()
                x1, y1, x2, y2 = [float(v) for v in xyxy]
                conf = float(b.conf[0]) if getattr(b, "conf", None) is not None else 0.0
                cls_id = int(b.cls[0]) if getattr(b, "cls", None) is not None else None
                cls_name = str(names.get(cls_id, "")) if cls_id is not None else ""
            except Exception:
                continue

            if self.target_class_id >= 0 and cls_id != self.target_class_id:
                continue
            if self.target_class_name and cls_name.lower() != self.target_class_name.lower():
                continue

            u = 0.5 * (x1 + x2)
            v = 0.5 * (y1 + y2)
            cand = Detection2D(u=u, v=v, conf=conf, bbox=(x1, y1, x2, y2), cls_id=cls_id, cls_name=cls_name)
            score = cand.conf
            if self.last_det_center is not None:
                du = cand.u - self.last_det_center[0]
                dv = cand.v - self.last_det_center[1]
                dpx = math.hypot(du, dv)
                if self.bbox_continuity_max_px > 0.0 and dpx > self.bbox_continuity_max_px:
                    score -= 1.0
                if self.bbox_continuity_weight > 0.0:
                    score -= self.bbox_continuity_weight * (dpx / max(1.0, self.bbox_continuity_max_px))
                if self.last_det_cls_id is not None and cand.cls_id == self.last_det_cls_id:
                    score += self.bbox_continuity_class_bonus
            if best is None or score > best_score:
                best = cand
                best_score = score

        return best

    def _estimate_quality_scale(self, conf: float, latency_ms: float) -> float:
        conf_floor = max(1e-3, min(0.95, self.conf_threshold))
        q_conf = clamp01((conf - conf_floor) / max(1e-6, 1.0 - conf_floor))
        q_lat = 1.0 / (1.0 + max(0.0, latency_ms) / max(1.0, self.tracker_quality_latency_ref_ms))
        q = q_conf * q_lat
        return max(self.tracker_min_gain_scale, q)

    def _tracker_predict_to(self, t: Time) -> None:
        if not self.track_valid or self.track_stamp is None:
            return
        dt = (t - self.track_stamp).nanoseconds * 1e-9
        if not math.isfinite(dt) or dt <= 1e-6:
            self.track_stamp = t
            return
        if dt > 1.0:
            dt = 1.0
        self.track_x += self.track_vx * dt
        self.track_y += self.track_vy * dt
        self.track_stamp = t

    def _tracker_update(self, meas_x: float, meas_y: float, t: Time, quality_scale: float) -> Tuple[float, float]:
        if not self.tracker_enable:
            return meas_x, meas_y
        if not self.track_valid or self.track_stamp is None:
            self.track_x = meas_x
            self.track_y = meas_y
            self.track_vx = 0.0
            self.track_vy = 0.0
            self.track_stamp = t
            self.track_valid = True
            return self.track_x, self.track_y

        dt = (t - self.track_stamp).nanoseconds * 1e-9
        if not math.isfinite(dt) or dt < 0.0:
            dt = 0.0
        if dt > 1.0:
            dt = 1.0

        # Predict
        px = self.track_x + self.track_vx * dt
        py = self.track_y + self.track_vy * dt
        rx = meas_x - px
        ry = meas_y - py

        a = clamp01(self.tracker_alpha * quality_scale)
        b = clamp01(self.tracker_beta * quality_scale)
        self.track_x = px + a * rx
        self.track_y = py + a * ry
        if dt > 1e-6:
            self.track_vx = self.track_vx + (b / dt) * rx
            self.track_vy = self.track_vy + (b / dt) * ry
        self.track_stamp = t
        return self.track_x, self.track_y

    def _tracker_pose_with_prediction(self, base_pose: Pose2D, now: Time, latency_s: float) -> Pose2D:
        if not self.tracker_enable:
            return base_pose
        self._tracker_predict_to(now)
        if not self.track_valid:
            return base_pose

        pred_horizon = 0.0
        if self.latency_comp_enable:
            pred_horizon = min(self.latency_comp_max_s, max(0.0, latency_s) * self.latency_comp_scale)
        x = self.track_x + self.track_vx * pred_horizon
        y = self.track_y + self.track_vy * pred_horizon

        yaw = base_pose.yaw
        speed = math.hypot(self.track_vx, self.track_vy)
        if speed > 0.05:
            yaw = math.atan2(self.track_vy, self.track_vx)
        return Pose2D(x=x, y=y, z=base_pose.z, yaw=yaw)

    def _sample_depth_range(self, det: Detection2D) -> Optional[float]:
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

        # Median in a small patch is more robust than single-pixel sampling.
        r = 2
        u0 = max(0, u - r)
        u1 = min(w, u + r + 1)
        v0 = max(0, v - r)
        v1 = min(h, v + r + 1)
        patch = depth[v0:v1, u0:u1]
        patch = patch[np.isfinite(patch)]
        patch = patch[(patch >= self.depth_min_m) & (patch <= self.depth_max_m)]
        if patch.size == 0:
            return None
        return float(np.median(patch))

    def _ground_range_from_pixel(self, det: Detection2D, cam: CameraModel) -> Optional[float]:
        # Approximate camera projection with yaw/pitch offsets and a ground-plane target (z=target_ground_z_m).
        x_n = (det.u - cam.cx) / cam.fx
        y_n = (det.v - cam.cy) / cam.fy
        yaw_img = math.atan2(x_n, 1.0)
        pitch_img = math.atan2(y_n, math.sqrt(1.0 + x_n * x_n))

        cam_world_z = self.uav_pose.z + self.cam_z_offset_m
        dz = self.target_ground_z_m - cam_world_z  # usually negative (ground below camera)
        elev = self.cam_pitch_offset_rad + pitch_img
        tan_elev = math.tan(elev)
        if abs(tan_elev) < 1e-3:
            return None
        # horizontal range from camera to ground intersection
        horiz_range = dz / tan_elev
        if not math.isfinite(horiz_range) or horiz_range <= 0.0:
            return None
        return float(horiz_range)

    def _estimate_pose_from_detection(
        self,
        det: Detection2D,
        cam: CameraModel,
        now: Time,
    ) -> Tuple[Pose2D, float, str]:
        x_n = (det.u - cam.cx) / cam.fx
        _y_n = (det.v - cam.cy) / cam.fy
        bearing_img = math.atan2(x_n, 1.0)
        bearing = self.cam_yaw_offset_rad + bearing_img

        depth_range = self._sample_depth_range(det) if self.use_depth_range else None
        ground_range = self._ground_range_from_pixel(det, cam)
        if ground_range is not None:
            if ground_range < self.ground_min_range_m:
                ground_range = None
            elif self.ground_max_range_m > 0.0 and ground_range > self.ground_max_range_m:
                ground_range = None
        range_m = self.constant_range_m
        range_source = "const"
        mode = self.range_mode
        if mode == "depth":
            if depth_range is not None:
                range_m = depth_range
                range_source = "depth"
        elif mode == "ground":
            if ground_range is not None:
                range_m = ground_range
                range_source = "ground"
        elif mode == "const":
            pass
        else:  # auto
            if depth_range is not None:
                range_m = depth_range
                range_source = "depth"
            elif ground_range is not None:
                range_m = ground_range
                range_source = "ground"

        # Simple EMA smoothing on bearing/range to reduce estimate jitter.
        a = self.smooth_alpha
        if self.smoothed_bearing is None or a >= 1.0:
            self.smoothed_bearing = bearing
        else:
            self.smoothed_bearing = a * bearing + (1.0 - a) * self.smoothed_bearing
        if self.smoothed_range_m is None or a >= 1.0:
            self.smoothed_range_m = range_m
        else:
            self.smoothed_range_m = a * range_m + (1.0 - a) * self.smoothed_range_m

        bearing = self.smoothed_bearing
        range_m = self.smoothed_range_m

        cam_world_x = self.uav_pose.x + self.cam_x_offset_m * math.cos(self.uav_pose.yaw) - self.cam_y_offset_m * math.sin(self.uav_pose.yaw)
        cam_world_y = self.uav_pose.y + self.cam_x_offset_m * math.sin(self.uav_pose.yaw) + self.cam_y_offset_m * math.cos(self.uav_pose.yaw)

        x = cam_world_x + range_m * math.cos(self.uav_pose.yaw + bearing)
        y = cam_world_y + range_m * math.sin(self.uav_pose.yaw + bearing)

        if self.smoothed_xy is None or self.xy_smooth_alpha >= 1.0:
            self.smoothed_xy = (x, y)
        else:
            ax = self.xy_smooth_alpha
            self.smoothed_xy = (
                ax * x + (1.0 - ax) * self.smoothed_xy[0],
                ax * y + (1.0 - ax) * self.smoothed_xy[1],
            )
        x, y = self.smoothed_xy

        yaw = self.prev_heading_yaw if self.prev_heading_yaw is not None else (self.uav_pose.yaw + bearing)
        if self.prev_estimate_xy is not None and self.prev_estimate_stamp is not None:
            dt = (now - self.prev_estimate_stamp).nanoseconds * 1e-9
            if dt > 1e-6:
                dx = x - self.prev_estimate_xy[0]
                dy = y - self.prev_estimate_xy[1]
                if math.hypot(dx, dy) > 0.05:
                    yaw = math.atan2(dy, dx)

        self.last_range_used_m = float(range_m)
        self.last_bearing_used_deg = math.degrees(bearing)
        pose = Pose2D(x=x, y=y, z=self.target_ground_z_m, yaw=yaw)
        return pose, range_m, range_source

    def _publish_estimate(self, est_pose: Pose2D, image_stamp: Time, now: Time) -> None:
        ps = PoseStamped()
        ps.header.stamp = now.to_msg()
        ps.header.frame_id = "map"

        ps.pose.position.x = float(est_pose.x)
        ps.pose.position.y = float(est_pose.y)
        ps.pose.position.z = float(est_pose.z)

        quat = quat_from_yaw(est_pose.yaw)
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])

        self.pub.publish(ps)
        self.last_pub_time = now
        self.prev_estimate_xy = (est_pose.x, est_pose.y)
        self.prev_estimate_stamp = now
        self.prev_heading_yaw = est_pose.yaw

    def _status_line(self, state: str, now: Time, extra: str = "") -> str:
        image_age_s = float("inf")
        image_age_ms = float("inf")
        pose_age = float("inf")
        if self.last_image_stamp is not None:
            image_age_s = (now - self.last_image_stamp).nanoseconds * 1e-9
            image_age_ms = image_age_s * 1000.0
        if self.last_uav_pose_stamp is not None:
            pose_age = (now - self.last_uav_pose_stamp).nanoseconds * 1e-9
        yolo_state = "enabled" if self.yolo_ready else "disabled"
        yolo_reason = self.yolo_error if self.yolo_error is not None else "ok"
        suffix = f" {extra}" if extra else ""
        img_wall_age_ms = -1.0
        if self.last_image_recv_walltime is not None:
            img_wall_age_ms = (time.monotonic() - self.last_image_recv_walltime) * 1000.0
        return (
            f"state={state} last_img_age_ms={image_age_ms if math.isfinite(image_age_ms) else 'inf'} "
            f"last_img_recv_wall_age_ms={img_wall_age_ms:.1f} "
            f"yolo={yolo_state} yolo_reason={yolo_reason} device={self.device} "
            f"conf={self.last_det_conf:.3f} latency_ms={self.last_latency_ms:.1f} "
            f"range_src={self.last_range_source} range_mode={self.range_mode} range_m={self.last_range_used_m:.2f} "
            f"bearing_deg={self.last_bearing_used_deg:.1f} reject_reason={self.last_reject_reason} "
            f"img_age_s={image_age_s if math.isfinite(image_age_s) else 'inf'} "
            f"uav_pose_age_s={pose_age:.2f} uav_pose_src={self.uav_pose_source}{suffix}"
        )

    def _publish_held_estimate(self, now: Time) -> bool:
        if self.last_good_estimate is None or self.hold_frames_left <= 0:
            return False
        if self.last_good_estimate_stamp is not None and self.max_hold_s > 0.0:
            hold_age_s = (now - self.last_good_estimate_stamp).nanoseconds * 1e-9
            if hold_age_s > self.max_hold_s:
                self.hold_frames_left = 0
                return False
        image_stamp = self.last_image_stamp if self.last_image_stamp is not None else now
        self.last_det_conf = -1.0
        self.last_latency_ms = max(0.0, (now - image_stamp).nanoseconds * 1e-6)
        self.last_range_source = "hold"
        self.last_reject_reason = "hold_reuse"
        hold_pose = self.last_good_estimate
        if self.track_valid:
            self._tracker_predict_to(now)
            speed = math.hypot(self.track_vx, self.track_vy)
            hold_yaw = hold_pose.yaw
            if speed > 0.05:
                hold_yaw = math.atan2(self.track_vy, self.track_vx)
            hold_pose = Pose2D(self.track_x, self.track_y, hold_pose.z, hold_yaw)
        self._publish_estimate(hold_pose, image_stamp, now)
        self.hold_frames_left -= 1
        return True

    def _estimate_is_sane(self, est_pose: Pose2D, bearing_rad: float, now: Time) -> Tuple[bool, str]:
        if self.prev_estimate_xy is None or self.prev_estimate_stamp is None:
            return True, "none"
        dt = max(1e-6, (now - self.prev_estimate_stamp).nanoseconds * 1e-9)
        dx = est_pose.x - self.prev_estimate_xy[0]
        dy = est_pose.y - self.prev_estimate_xy[1]
        jump = math.hypot(dx, dy)
        if self.max_xy_jump_m > 0.0 and jump > self.max_xy_jump_m:
            return False, "jump_xy"
        speed = jump / dt
        if self.max_target_speed_mps > 0.0 and speed > self.max_target_speed_mps:
            return False, "speed"
        if self.last_bearing_rad is not None and self.max_bearing_jump_deg > 0.0:
            d = abs((bearing_rad - self.last_bearing_rad + math.pi) % (2.0 * math.pi) - math.pi)
            if math.degrees(d) > self.max_bearing_jump_deg:
                return False, "bearing_jump"
        return True, "none"

    def _current_state(self, now: Time) -> str:
        if self.last_image_msg is None:
            return "waiting_for_image"
        if self.camera_model is None:
            return "waiting_for_camera_info"
        if not self.uav_pose_fresh(now):
            return "waiting_for_uav_pose"
        if not self.image_fresh(now):
            return "stale_image"
        return "running"

    def on_status_tick(self) -> None:
        now = self.get_clock().now()
        self.publish_status_msg(self._status_line(self._current_state(now), now))

    def on_tick(self) -> None:
        now = self.get_clock().now()
        image_ok = self.image_fresh(now)
        pose_ok = self.uav_pose_fresh(now)
        cam_ok = self.camera_model is not None

        if not cam_ok or not image_ok or not pose_ok:
            reasons = []
            if not cam_ok:
                reasons.append("no_camera_info")
            if not image_ok:
                reasons.append("image_stale")
            if not pose_ok:
                reasons.append("uav_pose_stale")
            state = "STALE"
            self.last_det_conf = -1.0
            self.last_latency_ms = -1.0
            self.last_range_source = "none"
            self.publish_status_msg(self._status_line(state, now, extra=f"reason={','.join(reasons)}"))
            if self.last_status != state:
                self.emit_event("ESTIMATE_STALE")
                self.last_status = state
            return

        if self.last_image_msg is None or self.camera_model is None:
            return

        img_bgr = self._image_to_bgr(self.last_image_msg)
        if img_bgr is None:
            self.last_det_conf = -1.0
            self.last_latency_ms = -1.0
            self.last_range_source = "none"
            state = "DECODE_FAIL"
            self.publish_status_msg(self._status_line(state, now))
            if self.last_status != state:
                self.emit_event("ESTIMATE_STALE")
                self.last_status = state
            return

        det = self._pick_detection(img_bgr)
        if det is None:
            self.bad_det_streak += 1
            self.good_det_streak = 0
            if self.bad_det_streak >= self.bad_debounce_frames:
                self.track_latched = False
            if self.yolo_ready and self._publish_held_estimate(now):
                state = "HOLD" if self.bad_det_streak >= self.bad_debounce_frames else "DEBOUNCE_HOLD"
                self.publish_status_msg(self._status_line(state, now))
                if self.last_status != state:
                    self.emit_event("ESTIMATE_OK")
                    self.last_status = state
                return
            self.last_det_conf = -1.0
            self.last_latency_ms = -1.0
            self.last_range_source = "none"
            state = "NO_DET" if self.yolo_ready else "YOLO_DISABLED"
            self.publish_status_msg(self._status_line(state, now))
            if self.last_status != state:
                self.emit_event("ESTIMATE_STALE")
                self.last_status = state
            return

        est_pose, _range_m, range_source = self._estimate_pose_from_detection(det, self.camera_model, now)
        cand_bearing_rad = math.radians(self.last_bearing_used_deg)
        sane, reject_reason = self._estimate_is_sane(est_pose, cand_bearing_rad, now)
        if not sane:
            self.bad_det_streak += 1
            self.good_det_streak = 0
            if self.bad_det_streak >= self.bad_debounce_frames:
                self.track_latched = False
            self.last_reject_reason = reject_reason
            if self._publish_held_estimate(now):
                state = "REJECT_HOLD" if self.bad_det_streak >= self.bad_debounce_frames else "REJECT_DEBOUNCE_HOLD"
                self.publish_status_msg(self._status_line(state, now))
                if self.last_status != state:
                    self.emit_event("ESTIMATE_OK")
                    self.last_status = state
                return
            self.last_det_conf = -1.0
            self.last_latency_ms = -1.0
            self.last_range_source = "reject"
            state = "REJECT"
            self.publish_status_msg(self._status_line(state, now))
            if self.last_status != state:
                self.emit_event("ESTIMATE_STALE")
                self.last_status = state
            return

        self.good_det_streak += 1
        self.bad_det_streak = 0
        self.last_det_center = (det.u, det.v)
        self.last_det_cls_id = det.cls_id

        self.last_reject_reason = "none"
        self.last_det_conf = float(det.conf)
        self.last_range_source = range_source
        image_stamp = self.last_image_stamp if self.last_image_stamp is not None else now
        self.last_latency_ms = max(0.0, (now - image_stamp).nanoseconds * 1e-6)

        quality_scale = self._estimate_quality_scale(self.last_det_conf, self.last_latency_ms)
        tx, ty = self._tracker_update(est_pose.x, est_pose.y, now, quality_scale)
        tracked_pose = Pose2D(tx, ty, est_pose.z, est_pose.yaw)
        tracked_pose = self._tracker_pose_with_prediction(tracked_pose, now, self.last_latency_ms * 1e-3)

        if not self.track_latched and self.good_det_streak < self.ok_debounce_frames:
            self.last_good_estimate = tracked_pose
            self.last_good_estimate_stamp = now
            self.hold_frames_left = self.max_hold_frames
            state = "REACQUIRE"
            if self._publish_held_estimate(now):
                self.publish_status_msg(self._status_line(state, now))
            else:
                self.publish_status_msg(self._status_line(state, now))
            if self.last_status != state:
                self.emit_event("ESTIMATE_OK")
                self.last_status = state
            self.last_bearing_rad = cand_bearing_rad
            return

        self.track_latched = True
        self.last_good_estimate = tracked_pose
        self.last_good_estimate_stamp = now
        self.hold_frames_left = self.max_hold_frames
        self._publish_estimate(tracked_pose, image_stamp, now)
        self.last_bearing_rad = cand_bearing_rad

        state = "OK"
        cls_txt = det.cls_name if det.cls_name else (str(det.cls_id) if det.cls_id is not None else "na")
        self.publish_status_msg(self._status_line(state, now, extra=f"class={cls_txt}"))
        if self.last_status != state:
            self.emit_event("ESTIMATE_OK")
            self.last_status = state



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
