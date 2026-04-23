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
from std_msgs.msg import Float32, Float64, String
from vision_msgs.msg import Detection2DArray

from lrs_halmstad.common.node_mixins import EventEmitterMixin
from lrs_halmstad.common.selected_target_state import (
    SelectedTargetState,
    encode_selected_target_state_msg,
)
from lrs_halmstad.common.ros_params import yaml_param
from lrs_halmstad.follow.follow_math import (
    Pose3D,
    coerce_bool,
    horizontal_distance_for_euclidean,
    quat_from_yaw,
    wrap_pi,
    yaw_from_quat,
)
from lrs_halmstad.perception.detection_protocol import Detection2D, decode_detection_payload
from lrs_halmstad.perception.detection_status import overlay_lines_from_status, parse_status_line
from lrs_halmstad.perception.yolo_common import cv2, image_to_bgr, order_quad


@dataclass
class CameraModel:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


class LeaderEstimator(EventEmitterMixin, Node):
    """Estimator-only node that turns external detections into a world-frame leader pose."""

    def __init__(self):
        super().__init__("leader_estimator")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.uav_name = str(self.declare_parameter("uav_name", "dji0").value)
        self.camera_topic = str(self.declare_parameter("camera_topic", "").value).strip() or f"/{self.uav_name}/camera0/image_raw"
        self.camera_info_topic = str(self.declare_parameter("camera_info_topic", "").value).strip() or f"/{self.uav_name}/camera0/camera_info"
        self.depth_topic = str(self.declare_parameter("depth_topic", "").value).strip()
        self.uav_pose_topic = str(self.declare_parameter("uav_pose_topic", "").value).strip() or f"/{self.uav_name}/pose"
        self.external_detection_topic = str(yaml_param(self, "external_detection_topic")).strip()
        self.external_detection_status_topic = str(yaml_param(self, "external_detection_status_topic")).strip()
        self.out_topic = str(yaml_param(self, "out_topic")).strip()
        self.status_topic = str(yaml_param(self, "status_topic")).strip()
        self.distance_status_topic = "/coord/leader_distance_debug"
        self.fault_status_topic = str(yaml_param(self, "fault_status_topic")).strip()
        self.estimate_error_topic = str(yaml_param(self, "estimate_error_topic")).strip()
        self.event_topic = str(yaml_param(self, "event_topic")).strip()
        self.debug_image_topic = str(yaml_param(self, "debug_image_topic")).strip()
        self.selected_target_topic = "/coord/leader_selected_target"
        self.declare_parameter("publish_selected_target", True)
        self.publish_selected_target = coerce_bool(self.get_parameter("publish_selected_target").value)
        _ct = self.declare_parameter("camera_tilt_topic", "").value
        self.camera_tilt_topic = str(_ct).strip() or f"/{self.uav_name}/follow/actual/tilt_deg"
        _cy = self.declare_parameter("camera_world_yaw_topic", "").value
        self.camera_world_yaw_topic = str(_cy).strip() or f"/{self.uav_name}/camera0/actual/world_yaw_rad"

        self.publish_status = coerce_bool(yaml_param(self, "publish_status"))
        self.publish_fault_status = coerce_bool(yaml_param(self, "publish_fault_status"))
        self.publish_debug_image = coerce_bool(yaml_param(self, "publish_debug_image"))
        self.publish_events = coerce_bool(yaml_param(self, "publish_events"))

        self.leader_actual_pose_enable = coerce_bool(yaml_param(self, "leader_actual_pose_enable"))
        self.leader_actual_pose_topic = str(yaml_param(self, "leader_actual_pose_topic")).strip()
        self.leader_actual_pose_timeout_s = float(yaml_param(self, "leader_actual_pose_timeout_s", descriptor=dyn_num))

        self.est_hz = float(yaml_param(self, "est_hz", descriptor=dyn_num))
        self.image_timeout_s = float(yaml_param(self, "image_timeout_s", descriptor=dyn_num))
        self.uav_pose_timeout_s = float(yaml_param(self, "uav_pose_timeout_s", descriptor=dyn_num))
        self.external_detection_timeout_s = float(yaml_param(self, "external_detection_timeout_s", descriptor=dyn_num))
        self.external_detection_max_latency_ms = float(
            yaml_param(self, "external_detection_max_latency_ms", descriptor=dyn_num)
        )

        self.d_target = float(yaml_param(self, "d_target", descriptor=dyn_num))
        self.declare_parameter("constant_range_m", 5.0, dyn_num)
        self.range_mode = str(yaml_param(self, "range_mode")).strip().lower()
        self.depth_scale = float(yaml_param(self, "depth_scale", descriptor=dyn_num))
        self.depth_timeout_s = float(yaml_param(self, "depth_timeout_s", descriptor=dyn_num))
        self.depth_patch_min_valid_px = max(1, int(yaml_param(self, "depth_patch_min_valid_px", descriptor=dyn_num)))
        self.depth_min_m = float(yaml_param(self, "depth_min_m", descriptor=dyn_num))
        self.depth_max_m = float(yaml_param(self, "depth_max_m", descriptor=dyn_num))
        self.depth_max_estimate_jump_m = float(yaml_param(self, "depth_max_estimate_jump_m", descriptor=dyn_num))
        self.depth_percentile = max(1.0, min(99.0, float(yaml_param(self, "depth_percentile", descriptor=dyn_num))))
        self.target_ground_z_m = float(yaml_param(self, "target_ground_z_m", descriptor=dyn_num))
        self.radio_range_topic = str(self.declare_parameter("radio_range_topic", "/omnet/radio_distance").value).strip()
        self.radio_range_timeout_s = float(self.declare_parameter("radio_range_timeout_s", 0.5).value)

        self.cam_yaw_offset_rad = math.radians(float(yaml_param(self, "cam_yaw_offset_deg", descriptor=dyn_num)))
        self.cam_pitch_offset_rad = math.radians(float(yaml_param(self, "cam_pitch_offset_deg", descriptor=dyn_num)))
        self.cam_roll_offset_rad = math.radians(float(yaml_param(self, "cam_roll_offset_deg", descriptor=dyn_num)))
        self.cam_x_offset_m = float(yaml_param(self, "cam_x_offset_m", descriptor=dyn_num))
        self.cam_y_offset_m = float(yaml_param(self, "cam_y_offset_m", descriptor=dyn_num))
        self.cam_z_offset_m = float(yaml_param(self, "cam_z_offset_m", descriptor=dyn_num))

        self.constant_range_m = float(self.get_parameter("constant_range_m").value)

        if self.est_hz <= 0.0:
            raise ValueError("est_hz must be > 0")
        if self.image_timeout_s <= 0.0:
            raise ValueError("image_timeout_s must be > 0")
        if self.depth_timeout_s <= 0.0:
            raise ValueError("depth_timeout_s must be > 0")
        if self.uav_pose_timeout_s <= 0.0:
            raise ValueError("uav_pose_timeout_s must be > 0")
        if self.external_detection_timeout_s <= 0.0:
            raise ValueError("external_detection_timeout_s must be > 0")
        if self.external_detection_max_latency_ms < 0.0:
            raise ValueError("external_detection_max_latency_ms must be >= 0")
        if self.leader_actual_pose_timeout_s <= 0.0:
            raise ValueError("leader_actual_pose_timeout_s must be > 0")
        if self.constant_range_m <= 0.0 and self.d_target <= 0.0:
            raise ValueError("either d_target or constant_range_m must be > 0")
        if self.range_mode not in ("auto", "depth", "const", "radio"):
            raise ValueError("range_mode must be one of: auto, depth, const, radio")

        self.camera_model: Optional[CameraModel] = None
        self.uav_pose: Optional[Pose3D] = None
        self.last_uav_pose_stamp: Optional[Time] = None
        self.last_image_msg: Optional[Image] = None
        self.last_image_stamp: Optional[Time] = None
        self.last_image_recv_walltime: Optional[float] = None
        self.last_depth_msg: Optional[Image] = None
        self.last_depth_stamp: Optional[Time] = None
        self.last_external_det: Optional[Detection2D] = None
        self.last_external_det_stamp: Optional[Time] = None
        self.last_external_det_rx_stamp: Optional[Time] = None
        self.last_external_det_publish_latency_ms: float = -1.0
        self.last_external_det_reject_reason: str = "none"
        self.last_external_det_status_text: str = ""
        self.last_camera_frame_id: str = ""
        self.last_camera_tilt_deg: Optional[float] = None
        self.last_camera_tilt_stamp: Optional[Time] = None
        self.last_camera_world_yaw_rad: Optional[float] = None
        self.last_camera_world_yaw_stamp: Optional[Time] = None
        self.last_external_det_status_stamp: Optional[Time] = None
        self.last_follow_debug_heading_yaw: Optional[float] = None
        self.last_follow_debug_heading_source: str = ""
        self.last_follow_debug_heading_stamp: Optional[Time] = None
        self.last_actual_leader_pose: Optional[Pose3D] = None
        self.last_actual_leader_pose_stamp: Optional[Time] = None
        self.last_radio_range_m: Optional[float] = None
        self.last_radio_range_stamp: Optional[Time] = None
        self.last_estimate_pose: Optional[Pose3D] = None
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
        self.last_fault_state: str = "none"
        self.last_fault_reason: str = "none"
        self.last_fault_stamp: Optional[Time] = None
        self.last_debug_state: str = "INIT"
        self.last_debug_det: Optional[Detection2D] = None
        self.audit_ticks_total = 0
        self.audit_ok_total = 0
        self.audit_no_det_total = 0
        self.audit_stale_total = 0
        self.audit_estimate_fail_total = 0

        # ---- estimate caching (prevent re-projection of stale detections) ----
        self._last_projected_det_rx_ns: int = -1
        self._cached_estimate_pose: Optional[Pose2D] = None
        self._cached_estimate_track_id: Optional[int] = None

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.on_image, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.depth_sub = self.create_subscription(Image, self.depth_topic, self.on_depth, 10) if self.depth_topic else None
        self.uav_pose_sub = self.create_subscription(PoseStamped, self.uav_pose_topic, self.on_uav_pose, 10)
        self.external_detection_sub = self.create_subscription(String, self.external_detection_topic, self.on_external_detection, 1)
        self.external_detection_status_sub = self.create_subscription(
            String,
            self.external_detection_status_topic,
            self.on_external_detection_status,
            1,
        )
        self.follow_debug_heading_source_sub = self.create_subscription(
            String,
            f"/{self.uav_name}/follow/debug/leader_heading_source",
            self.on_follow_debug_heading_source,
            10,
        )
        self.follow_debug_heading_yaw_sub = self.create_subscription(
            Float32,
            f"/{self.uav_name}/follow/debug/leader_follow_yaw_rad",
            self.on_follow_debug_heading_yaw,
            10,
        )
        self.actual_leader_pose_sub = (
            self.create_subscription(Odometry, self.leader_actual_pose_topic, self.on_actual_leader_pose, 10)
            if self.leader_actual_pose_enable
            else None
        )
        self.radio_range_sub = (
            self.create_subscription(Float64, self.radio_range_topic, self.on_radio_range, 10)
            if self.radio_range_topic
            else None
        )
        self.camera_tilt_sub = self.create_subscription(Float32, self.camera_tilt_topic, self.on_camera_tilt, 10)
        self.camera_world_yaw_sub = self.create_subscription(Float32, self.camera_world_yaw_topic, self.on_camera_world_yaw, 10)

        self.pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.distance_status_pub = self.create_publisher(String, self.distance_status_topic, 10)
        self._setup_event_emitter(self.event_topic, self.publish_events)
        self.debug_image_pub = self.create_publisher(Image, self.debug_image_topic, 2) if self.publish_debug_image else None
        self.selected_target_pub = (
            self.create_publisher(Detection2DArray, self.selected_target_topic, 10)
            if self.publish_selected_target
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
            f"est_hz={self.est_hz}Hz, range_mode={self.range_mode}, const_target_m={self._constant_target_range()[0]:.2f}, "
            f"radio_range={self.radio_range_topic or 'disabled'}, "
            f"distance_status={self.distance_status_topic}"
        )
        self.publish_fault_status_msg(self._fault_line("none", "none", self.get_clock().now()))
        self.emit_event("ESTIMATOR_NODE_START")

    def publish_status_msg(self, text: str) -> None:
        if not self.publish_status:
            return
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def publish_distance_status_msg(self, now: Time) -> None:
        msg = String()
        msg.data = self._distance_status_line(now)
        self.distance_status_pub.publish(msg)

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

    def _detector_publish_latency_ms(self) -> float:
        return float(self.last_external_det_publish_latency_ms)

    def _detector_reason(self, now: Time) -> str:
        if self.last_external_det_stamp is None:
            return self.last_external_det_reject_reason if self.last_external_det_reject_reason != "none" else "none"
        return "ok" if self.external_detection_fresh(now) else "stale"

    def _detector_status_fields(self, now: Time) -> dict[str, str]:
        if not self.last_external_det_status_text:
            return {}
        if not self._is_fresh(self.last_external_det_status_stamp, self.external_detection_timeout_s, now):
            return {}
        return parse_status_line(self.last_external_det_status_text)

    def _detector_state(self, now: Time) -> str:
        return self._detector_status_fields(now).get("state", "na")

    def _detector_status_reason(self, now: Time) -> str:
        return self._detector_status_fields(now).get("reason", "na")

    def _no_det_origin(self, now: Time) -> str:
        if self.last_external_det_reject_reason != "none":
            return "detector_message_rejected"
        fields = self._detector_status_fields(now)
        if not fields:
            return "detector_missing" if self.last_external_det_stamp is None else "detector_stale"
        det_state = fields.get("state", "").strip().upper()
        if det_state == "NO_DET":
            return "tracker_no_det"
        if det_state == "OK":
            return "detector_stale_to_estimator"
        if det_state == "DECODE_FAIL":
            return "tracker_decode_fail"
        if det_state == "YOLO_DISABLED":
            return "tracker_disabled"
        return f"tracker_{det_state.lower()}" if det_state else "detector_unknown"

    def _record_audit_state(self, state: str) -> None:
        self.audit_ticks_total += 1
        state = str(state).strip().upper()
        if state == "OK":
            self.audit_ok_total += 1
        elif state == "NO_DET":
            self.audit_no_det_total += 1
        elif state == "STALE":
            self.audit_stale_total += 1
        else:
            self.audit_estimate_fail_total += 1

    def _detection_publish_latency_ms(self, metadata: dict[str, object]) -> Optional[float]:
        raw = metadata.get("camera_to_publish_latency_ms")
        try:
            value = float(raw)
        except Exception:
            return None
        return value if math.isfinite(value) and value >= 0.0 else None

    def _detection_exceeds_latency_budget(self, metadata: dict[str, object]) -> bool:
        if self.external_detection_max_latency_ms <= 0.0:
            return False
        if bool(metadata.get("stale_detection", False)):
            return True
        latency_ms = self._detection_publish_latency_ms(metadata)
        return latency_ms is not None and latency_ms > self.external_detection_max_latency_ms

    def _fault_line(self, state: str, reason: str, now: Time) -> str:
        return (
            f"state={state} reason={reason} fault_age_ms={self._fault_age_ms(now):.1f} "
            f"detector=external detector_reason={self._detector_reason(now)} detector_age_ms={self._detector_age_ms(now):.1f} "
            f"detector_publish_latency_ms={self._detector_publish_latency_ms():.1f} "
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
        self.uav_pose = Pose3D(
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
        self.last_actual_leader_pose = Pose3D(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            yaw=yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        try:
            self.last_actual_leader_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_actual_leader_pose_stamp = self.get_clock().now()

    def on_radio_range(self, msg: Float64) -> None:
        val = float(msg.data)
        if math.isfinite(val) and val > 0.0:
            self.last_radio_range_m = val
            self.last_radio_range_stamp = self.get_clock().now()

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
        latency_ms = self._detection_publish_latency_ms(det_msg.metadata)
        self.last_external_det_publish_latency_ms = latency_ms if latency_ms is not None else -1.0
        # Only latch valid detections so that NO_DET frames do not erase the
        # last real detection before external_detection_timeout_s expires.
        if det_msg.detection is not None:
            if self._detection_exceeds_latency_budget(det_msg.metadata):
                self.last_external_det_reject_reason = "detection_publish_latency_exceeded"
                return
            self.last_external_det_stamp = stamp
            self.last_external_det_rx_stamp = now
            self.last_external_det = det_msg.detection
            self.last_external_det_reject_reason = "none"

    def on_camera_tilt(self, msg: Float32) -> None:
        self.last_camera_tilt_deg = float(msg.data)
        self.last_camera_tilt_stamp = self.get_clock().now()

    def on_camera_world_yaw(self, msg: Float32) -> None:
        self.last_camera_world_yaw_rad = wrap_pi(float(msg.data))
        self.last_camera_world_yaw_stamp = self.get_clock().now()

    def on_external_detection_status(self, msg: String) -> None:
        self.last_external_det_status_text = str(msg.data)
        self.last_external_det_status_stamp = self.get_clock().now()

    def on_follow_debug_heading_source(self, msg: String) -> None:
        self.last_follow_debug_heading_source = str(msg.data).strip()
        self.last_follow_debug_heading_stamp = self.get_clock().now()

    def on_follow_debug_heading_yaw(self, msg: Float32) -> None:
        self.last_follow_debug_heading_yaw = float(msg.data)
        self.last_follow_debug_heading_stamp = self.get_clock().now()

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
        return self._is_fresh(self.last_external_det_stamp, self.external_detection_timeout_s, now)

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

    def depth_fresh(self, now: Time) -> bool:
        return self._is_fresh(self.last_depth_stamp, self.depth_timeout_s, now)

    def radio_range_fresh(self, now: Time) -> bool:
        return self._is_fresh(self.last_radio_range_stamp, self.radio_range_timeout_s, now)

    def _depth_range_at(self, det: Detection2D, now: Time) -> Optional[float]:
        if self.last_depth_msg is None or not self.depth_fresh(now):
            return None
        depth = self._depth_to_array_m(self.last_depth_msg)
        if depth is None:
            return None
        h, w = depth.shape[:2]
        # Use the inner 50% of the bounding box for depth sampling when available.
        # This scales with detection size (robust at all distances) and avoids
        # bbox edge pixels that often land on background or drone body.
        if det.bbox is not None:
            x1, y1, x2, y2 = det.bbox
            bw = max(1.0, x2 - x1)
            bh = max(1.0, y2 - y1)
            mx = bw * 0.25
            my = bh * 0.25
            pu1 = max(0, int(round(x1 + mx)))
            pu2 = min(w, int(round(x2 - mx)))
            pv1 = max(0, int(round(y1 + my)))
            pv2 = min(h, int(round(y2 - my)))
        else:
            u = int(round(det.u))
            v = int(round(det.v))
            if u < 0 or u >= w or v < 0 or v >= h:
                return None
            pu1, pu2 = max(0, u - 2), min(w, u + 3)
            pv1, pv2 = max(0, v - 2), min(h, v + 3)
        patch = depth[pv1:pv2, pu1:pu2]
        patch = patch[np.isfinite(patch)]
        patch = patch[(patch >= self.depth_min_m) & (patch <= self.depth_max_m)]
        if patch.size < self.depth_patch_min_valid_px:
            return None
        return float(np.percentile(patch, self.depth_percentile))

    def _depth_reject_reason(
        self,
        x: float,
        y: float,
        depth_range: float,
        now: Time,
        track_id: Optional[int],
    ) -> str:
        if self.depth_max_estimate_jump_m <= 0.0:
            return "none"
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
        if jump_m > self.depth_max_estimate_jump_m:
            return "depth_jump"
        return "none"

    @staticmethod
    def _order_quad(points: np.ndarray) -> np.ndarray:
        return order_quad(points)

    def _unwrap_angle_near(self, raw_angle: float, reference_angle: Optional[float]) -> float:
        if reference_angle is None:
            return float(raw_angle)
        return float(reference_angle + wrap_pi(raw_angle - reference_angle))

    def _resolve_bidirectional_heading(self, raw_yaw: float) -> float:
        yaw_a = float(raw_yaw)
        yaw_b = wrap_pi(yaw_a + math.pi)
        if self.prev_heading_yaw is None:
            return yaw_a
        yaw_a = self._unwrap_angle_near(yaw_a, self.prev_heading_yaw)
        yaw_b = self._unwrap_angle_near(yaw_b, self.prev_heading_yaw)
        return yaw_a if abs(yaw_a - self.prev_heading_yaw) <= abs(yaw_b - self.prev_heading_yaw) else yaw_b

    def _ground_point_from_pixel(self, u: float, v: float, cam: CameraModel) -> Optional[Tuple[float, float]]:
        assert self.uav_pose is not None
        x_n = (u - cam.cx) / cam.fx
        y_n = (v - cam.cy) / cam.fy
        bearing = self.cam_yaw_offset_rad - math.atan2(x_n, 1.0)
        pitch_img = math.atan2(-y_n, math.sqrt(1.0 + x_n * x_n))
        cam_world_z = self.uav_pose.z + self.cam_z_offset_m
        dz = self.target_ground_z_m - cam_world_z
        elev = self.cam_pitch_offset_rad + pitch_img
        tan_elev = math.tan(elev)
        if abs(tan_elev) < 1e-3:
            return None
        horiz_range = dz / tan_elev
        if not math.isfinite(horiz_range) or horiz_range <= 0.0:
            return None
        cam_world_x, cam_world_y = self._cam_world_xy()
        return (
            cam_world_x + horiz_range * math.cos(self.uav_pose.yaw + bearing),
            cam_world_y + horiz_range * math.sin(self.uav_pose.yaw + bearing),
        )

    def _obb_heading_from_corners(
        self,
        corners: Tuple[Tuple[float, float], ...],
        cam: CameraModel,
    ) -> Optional[float]:
        ordered = self._order_quad(np.asarray(corners, dtype=np.float64))
        edges = np.roll(ordered, -1, axis=0) - ordered
        lengths = np.linalg.norm(edges, axis=1)
        if lengths[0] >= lengths[1]:
            axis_p1 = 0.5 * (ordered[0] + ordered[1])
            axis_p2 = 0.5 * (ordered[2] + ordered[3])
        else:
            axis_p1 = 0.5 * (ordered[1] + ordered[2])
            axis_p2 = 0.5 * (ordered[3] + ordered[0])
        gp1 = self._ground_point_from_pixel(float(axis_p1[0]), float(axis_p1[1]), cam)
        gp2 = self._ground_point_from_pixel(float(axis_p2[0]), float(axis_p2[1]), cam)
        if gp1 is None or gp2 is None:
            return None
        dx = gp2[0] - gp1[0]
        dy = gp2[1] - gp1[1]
        if math.hypot(dx, dy) <= 1e-6:
            return None
        return self._resolve_bidirectional_heading(math.atan2(dy, dx))

    def _constant_target_range(self) -> tuple[float, str]:
        if self.d_target > 0.0:
            vertical_delta = 0.0
            if self.uav_pose is not None:
                vertical_delta = self.uav_pose.z - self.target_ground_z_m
            return float(horizontal_distance_for_euclidean(self.d_target, vertical_delta)), "const_target"
        return float(self.constant_range_m), "const_fallback"

    def _radio_range_as_horizontal(self, now: Time) -> Optional[float]:
        """Convert the OMNeT FSPL radio distance (Euclidean) to a horizontal range."""
        if self.last_radio_range_m is None or not self.radio_range_fresh(now):
            return None
        if self.uav_pose is None:
            return None
        vertical_delta = self.uav_pose.z - self.target_ground_z_m
        horiz = horizontal_distance_for_euclidean(self.last_radio_range_m, vertical_delta)
        return float(horiz) if horiz > 0.0 else None

    def _selected_target_bbox(self, det: Detection2D):
        if det.obb_corners is not None and len(det.obb_corners) == 4:
            corners = det.obb_corners
            x1 = min(c[0] for c in corners)
            x2 = max(c[0] for c in corners)
            y1 = min(c[1] for c in corners)
            y2 = max(c[1] for c in corners)
            return (0.5 * float(x1 + x2), 0.5 * float(y1 + y2), max(0.0, float(x2 - x1)), max(0.0, float(y2 - y1)), 0.0)
        if det.bbox is not None and len(det.bbox) == 4:
            x1, y1, x2, y2 = det.bbox
            return (0.5 * float(x1 + x2), 0.5 * float(y1 + y2), max(0.0, float(x2 - x1)), max(0.0, float(y2 - y1)), 0.0)
        return (float(det.u), float(det.v), 0.0, 0.0, 0.0)

    def _publish_selected_target(self, now: Time, det: Optional[Detection2D], projected_range_m: Optional[float] = None) -> None:
        if self.selected_target_pub is None:
            return
        stamp = (
            self.last_external_det_stamp.to_msg()
            if det is not None and self.last_external_det_stamp is not None
            else now.to_msg()
        )
        frame_id = self.last_camera_frame_id
        if det is None:
            state = SelectedTargetState(stamp=stamp, frame_id=frame_id, valid=False)
        else:
            center_x, center_y, size_x, size_y, theta = self._selected_target_bbox(det)
            cls_label = str(det.cls_name or "")
            if not cls_label and det.cls_id is not None:
                cls_label = str(det.cls_id)
            track_id = "" if det.track_id is None else str(int(det.track_id))
            track_age_s = float(det.track_age_s) if det.track_age_s > 0.0 else None
            state = SelectedTargetState(
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
        self.selected_target_pub.publish(encode_selected_target_state_msg(state))

    def _estimate_from_detection(self, det: Detection2D, cam: CameraModel, now: Time) -> Tuple[Pose3D, str]:
        assert self.uav_pose is not None
        center_x_n = (det.u - cam.cx) / cam.fx
        center_bearing = self.cam_yaw_offset_rad - math.atan2(center_x_n, 1.0)
        depth_range = self._depth_range_at(det, now)
        cam_world_x, cam_world_y = self._cam_world_xy()
        depth_reject_reason = "none"
        if depth_range is not None:
            depth_x = cam_world_x + depth_range * math.cos(self.uav_pose.yaw + center_bearing)
            depth_y = cam_world_y + depth_range * math.sin(self.uav_pose.yaw + center_bearing)
            depth_reject_reason = self._depth_reject_reason(
                depth_x,
                depth_y,
                depth_range,
                now,
                det.track_id,
            )
            if depth_reject_reason != "none":
                depth_range = None
        radio_range = self._radio_range_as_horizontal(now)
        if self.range_mode == "depth":
            if depth_range is None:
                raise ValueError(depth_reject_reason if depth_reject_reason != "none" else "depth_range_invalid")
            range_m = depth_range
            range_source = "depth"
        elif self.range_mode == "radio":
            if radio_range is None:
                raise ValueError("radio_range_stale_or_unavailable")
            range_m = radio_range
            range_source = "radio"
        elif self.range_mode == "const":
            range_m, range_source = self._constant_target_range()
        else:  # auto: prefer depth, then radio, fall back to const
            if depth_range is not None:
                range_m = depth_range
                range_source = "depth"
            elif radio_range is not None:
                range_m = radio_range
                range_source = "radio"
            else:
                range_m, range_source = self._constant_target_range()

        bearing = center_bearing
        x = cam_world_x + range_m * math.cos(self.uav_pose.yaw + bearing)
        y = cam_world_y + range_m * math.sin(self.uav_pose.yaw + bearing)

        heading_source = "fallback"
        yaw = self.prev_heading_yaw if self.prev_heading_yaw is not None else self.uav_pose.yaw
        if det.obb_heading_yaw is not None:
            yaw = self._resolve_bidirectional_heading(det.obb_heading_yaw)
            heading_source = "obb"
        elif det.obb_corners is not None:
            obb_heading = self._obb_heading_from_corners(det.obb_corners, cam)
            if obb_heading is not None:
                yaw = obb_heading
                det.obb_heading_yaw = obb_heading
                heading_source = "obb"

        self.last_det_conf = float(det.conf)
        self.last_range_source = range_source
        self.last_range_used_m = float(range_m)
        self.last_bearing_used_deg = math.degrees(bearing)
        self.last_heading_source = heading_source
        self.last_reject_reason = depth_reject_reason
        return Pose3D(x=x, y=y, z=self.target_ground_z_m, yaw=yaw), range_source

    def _publish_estimate(self, pose: Pose3D, now: Time, track_id: Optional[int]) -> None:
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

    def _publish_estimate_error(self, est_pose: Pose3D, now: Time) -> None:
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

    def _status_line(self, state: str, reason: str, now: Time, *, audit_origin: str = "none") -> str:
        reason = str(reason).strip() or "none"
        err_dx = "na" if self.last_estimate_error_dx_m is None else f"{self.last_estimate_error_dx_m:.2f}"
        err_dy = "na" if self.last_estimate_error_dy_m is None else f"{self.last_estimate_error_dy_m:.2f}"
        err_planar = "na" if self.last_estimate_error_m is None else f"{self.last_estimate_error_m:.2f}"
        est_d, est_xy = self._estimated_distance_values(now)
        real_d, real_xy, real_z = self._actual_distance_values(now)
        est_d_text = "na" if est_d is None else f"{est_d:.2f}"
        est_xy_text = "na" if est_xy is None else f"{est_xy:.2f}"
        real_d_text = "na" if real_d is None else f"{real_d:.2f}"
        real_xy_text = "na" if real_xy is None else f"{real_xy:.2f}"
        real_z_text = "na" if real_z is None else f"{real_z:.2f}"
        return (
            f"state={state} reason={reason} "
            f"audit_origin={audit_origin} "
            f"detector_state={self._detector_state(now)} detector_status_reason={self._detector_status_reason(now)} "
            f"detector_reason={self._detector_reason(now)} detector_age_ms={self._detector_age_ms(now):.1f} "
            f"detector_publish_latency_ms={self._detector_publish_latency_ms():.1f} "
            f"conf={self.last_det_conf:.3f} latency_ms={self.last_latency_ms:.1f} "
            f"range_src={self.last_range_source} range_mode={self.range_mode} range_m={self.last_range_used_m:.2f} "
            f"bearing_deg={self.last_bearing_used_deg:.1f} reject_reason={self.last_reject_reason} "
            f"heading_src={self.last_heading_source} "
            f"audit_ticks={self.audit_ticks_total} audit_ok={self.audit_ok_total} "
            f"audit_no_det={self.audit_no_det_total} audit_stale={self.audit_stale_total} "
            f"audit_estimate_fail={self.audit_estimate_fail_total} "
            f"est_d_m={est_d_text} est_xy_m={est_xy_text} "
            f"real_d_m={real_d_text} real_xy_m={real_xy_text} real_z_m={real_z_text} "
            f"err_dx_m={err_dx} err_dy_m={err_dy} err_planar_m={err_planar}"
        )

    def _distance_status_line(self, now: Time) -> str:
        est_d, est_xy = self._estimated_distance_values(now)
        real_d, real_xy, real_z = self._actual_distance_values(now)
        range_m_text = "na" if self.last_range_used_m < 0.0 else f"{self.last_range_used_m:.2f}"
        est_d_text = "na" if est_d is None else f"{est_d:.2f}"
        est_xy_text = "na" if est_xy is None else f"{est_xy:.2f}"
        real_d_text = "na" if real_d is None else f"{real_d:.2f}"
        real_xy_text = "na" if real_xy is None else f"{real_xy:.2f}"
        real_z_text = "na" if real_z is None else f"{real_z:.2f}"
        err_d_text = "na" if est_d is None or real_d is None else f"{(est_d - real_d):.2f}"
        err_xy_text = "na" if est_xy is None or real_xy is None else f"{(est_xy - real_xy):.2f}"
        return (
            f"range_src={self.last_range_source} range_m={range_m_text} "
            f"est_d_m={est_d_text} est_xy_m={est_xy_text} "
            f"real_d_m={real_d_text} real_xy_m={real_xy_text} real_z_m={real_z_text} "
            f"err_d_m={err_d_text} err_xy_m={err_xy_text}"
        )

    def _detection_status_overlay_lines(self, now: Time) -> list[str]:
        if not self.last_external_det_status_text:
            return ["src=none"]
        if not self._is_fresh(self.last_external_det_status_stamp, self.external_detection_timeout_s, now):
            return ["src=none"]
        return overlay_lines_from_status(self.last_external_det_status_text) or ["src=none"]

    def _detection_task_label(self, now: Time) -> str:
        if not self.last_external_det_status_text:
            return "na"
        if not self._is_fresh(self.last_external_det_status_stamp, self.external_detection_timeout_s, now):
            return "na"
        fields = parse_status_line(self.last_external_det_status_text)
        return fields.get("task", "na")

    def _resolved_debug_heading(self, now: Time) -> tuple[Optional[float], str]:
        if (
            self.last_follow_debug_heading_yaw is not None
            and self._is_fresh(self.last_follow_debug_heading_stamp, self.external_detection_timeout_s, now)
        ):
            src = self.last_follow_debug_heading_source or "follow_debug"
            return self.last_follow_debug_heading_yaw, src
        if self.last_estimate_pose is not None:
            return self.last_estimate_pose.yaw, self.last_heading_source or "estimate_pose"
        return None, "none"

    def _heading_direction_label(self, leader_pose: Optional[Pose3D], heading_yaw: Optional[float]) -> str:
        if leader_pose is None or self.uav_pose is None or heading_yaw is None:
            return "na"
        away_from_uav_yaw = math.atan2(
            leader_pose.y - self.uav_pose.y,
            leader_pose.x - self.uav_pose.x,
        )
        delta = wrap_pi(heading_yaw - away_from_uav_yaw)
        abs_delta = abs(delta)
        straight_thresh = math.radians(45.0)
        reverse_thresh = math.radians(135.0)
        if abs_delta <= straight_thresh:
            return "forward"
        if abs_delta >= reverse_thresh:
            return "reverse"
        if delta > 0.0:
            return "left"
        return "right"

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

    def _actual_pose_line(self, now: Time) -> str:
        if self.last_actual_leader_pose is None or not self.actual_leader_pose_fresh(now):
            return "actual: na"
        return (
            f"actual: ("
            f"{self.last_actual_leader_pose.x:.2f},"
            f"{self.last_actual_leader_pose.y:.2f},"
            f"{self.last_actual_leader_pose.yaw:.2f})"
        )

    def _estimated_distance_values(self, now: Time) -> tuple[Optional[float], Optional[float]]:
        if self.uav_pose is None or not self.uav_pose_fresh(now):
            return None, None
        if self.last_estimate_pose is None:
            return None, None
        dx = self.last_estimate_pose.x - self.uav_pose.x
        dy = self.last_estimate_pose.y - self.uav_pose.y
        dz = self.last_estimate_pose.z - self.uav_pose.z
        est_xy = math.hypot(dx, dy)
        est_d = math.hypot(est_xy, dz)
        return est_d, est_xy

    def _estimated_distance_lines(self, now: Time) -> list[str]:
        est_d, est_xy = self._estimated_distance_values(now)
        if est_d is None or est_xy is None:
            return ["est_d: na", "est_xy: na"]
        return [
            f"est_d: {est_d:.2f}m",
            f"est_xy: {est_xy:.2f}m",
        ]

    def _actual_distance_values(self, now: Time) -> tuple[Optional[float], Optional[float], Optional[float]]:
        if self.uav_pose is None or not self.uav_pose_fresh(now):
            return None, None, None
        if self.last_actual_leader_pose is None or not self.actual_leader_pose_fresh(now):
            return None, None, None
        dx = self.last_actual_leader_pose.x - self.uav_pose.x
        dy = self.last_actual_leader_pose.y - self.uav_pose.y
        dz = self.last_actual_leader_pose.z - self.uav_pose.z
        real_xy = math.hypot(dx, dy)
        real_z = abs(dz)
        real_d = math.hypot(real_xy, real_z)
        return real_d, real_xy, real_z

    def _actual_range_lines(self, now: Time) -> list[str]:
        real_d, real_xy, real_z = self._actual_distance_values(now)
        if real_d is None or real_xy is None or real_z is None:
            return ["real_d: na", "real_xy: na", "real_z: na"]
        return [
            f"real_d: {real_d:.2f}m",
            f"real_xy: {real_xy:.2f}m",
            f"real_z: {real_z:.2f}m",
        ]

    def _draw_debug_text(
        self,
        img_bgr: np.ndarray,
        text: str,
        x: int,
        y: int,
        *,
        align_right: bool = False,
    ) -> None:
        draw_x = int(x)
        if align_right:
            (text_w, _), _baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            draw_x = max(0, int(x) - int(text_w))
        cv2.putText(img_bgr, text, (draw_x, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 3, cv2.LINE_AA)
        cv2.putText(img_bgr, text, (draw_x, int(y)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    def _draw_debug_lines(
        self,
        img_bgr: np.ndarray,
        lines: list[str],
        x: int,
        y_start: int,
        *,
        align_right: bool = False,
    ) -> None:
        y = int(y_start)
        for line in lines:
            self._draw_debug_text(img_bgr, line, x, y, align_right=align_right)
            y += 18

    def _publish_debug_image(self, state: str, reason: str, det: Optional[Detection2D]) -> None:
        if not self.publish_debug_image or self.debug_image_pub is None or self.last_image_msg is None:
            return
        img_bgr = self._image_to_bgr(self.last_image_msg)
        if img_bgr is None:
            return
        try:
            out = img_bgr.copy()
            if cv2 is not None:
                now = self.get_clock().now()
                color = self._debug_color(state)
                if det is not None:
                    x1, y1, x2, y2 = det.bbox
                    cv2.rectangle(out, (int(round(x1)), int(round(y1))), (int(round(x2)), int(round(y2))), color, 2)
                    if det.obb_corners is not None:
                        pts = np.asarray(det.obb_corners, dtype=np.int32).reshape((-1, 1, 2))
                        cv2.polylines(out, [pts], True, (255, 255, 0), 2, cv2.LINE_AA)
                    cv2.circle(out, (int(round(det.u)), int(round(det.v))), 3, color, -1)
                resolved_heading_yaw, resolved_heading_src = self._resolved_debug_heading(now)
                resolved_heading_label = self._heading_direction_label(self.last_estimate_pose, resolved_heading_yaw)
                h, w = out.shape[:2]
                estimate_lines = []
                if self.last_estimate_pose is not None:
                    estimate_lines.append(
                        f"est: ({self.last_estimate_pose.x:.2f},{self.last_estimate_pose.y:.2f},{self.last_estimate_pose.yaw:.2f})"
                    )
                estimate_lines.append(self._actual_pose_line(now))
                estimate_lines.extend(self._estimated_distance_lines(now))
                estimate_lines.extend(self._actual_range_lines(now))
                estimate_lines.extend([
                    f"range_src: {self.last_range_source}",
                    f"bearing: {self.last_bearing_used_deg:.1f}deg",
                    f"est_heading: {resolved_heading_label}",
                    f"heading_src: {resolved_heading_src}",
                ])

                detection_lines = [
                    f"task: {self._detection_task_label(now)}",
                    f"state: {state}",
                    f"reason: {reason}",
                    f"conf: {self.last_det_conf:.2f}",
                ]
                detection_lines.extend(self._detection_status_overlay_lines(now))

                self._draw_debug_lines(out, estimate_lines, 8, 20)
                self._draw_debug_lines(out, detection_lines, w - 8, 20, align_right=True)
                self._draw_debug_text(out, f"latency_ms: {self.last_latency_ms:.1f}", 8, h - 12)
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

        # ---- estimate caching: skip re-projection of stale detections ----
        det_rx_ns = (self.last_external_det_rx_stamp.nanoseconds
                     if self.last_external_det_rx_stamp is not None else -1)
        detection_is_new = (det is not None
                            and det_rx_ns != self._last_projected_det_rx_ns)

        if not detection_is_new and det is not None and self._cached_estimate_pose is not None:
            # Same detection as last projection — republish cached estimate
            self._publish_estimate(
                self._cached_estimate_pose, now, det.track_id)
            self._record_audit_state("OK")
            self.publish_status_msg(
                self._status_line("OK", "estimate_cached", now, audit_origin="estimate_cached"))
            self.last_debug_state = "OK"
            self._publish_debug_image("OK", "estimate_cached", det)
            return

        if self.camera_model is None:
            self.last_det_conf = -1.0
            self.last_range_source = "none"
            self.last_range_used_m = -1.0
            self.last_heading_source = "none"
            self._record_audit_state("STALE")
            self.publish_status_msg(self._status_line("STALE", "camera_info_missing", now, audit_origin="estimator_prereq"))
            self.publish_distance_status_msg(now)
            self._record_fault("STALE", "camera_info_missing", now)
            self._publish_debug_image("STALE", "camera_info_missing", det)
            return
        if not self.image_fresh(now):
            self.last_det_conf = -1.0
            self.last_range_source = "none"
            self.last_range_used_m = -1.0
            self.last_heading_source = "none"
            self._record_audit_state("STALE")
            self.publish_status_msg(self._status_line("STALE", "image_stale", now, audit_origin="estimator_prereq"))
            self.publish_distance_status_msg(now)
            self._record_fault("STALE", "image_stale", now)
            self._publish_debug_image("STALE", "image_stale", det)
            return
        if not self.uav_pose_fresh(now):
            self.last_det_conf = -1.0
            self.last_range_source = "none"
            self.last_range_used_m = -1.0
            self.last_heading_source = "none"
            self._record_audit_state("STALE")
            self.publish_status_msg(self._status_line("STALE", "uav_pose_stale", now, audit_origin="estimator_prereq"))
            self.publish_distance_status_msg(now)
            self._record_fault("STALE", "uav_pose_stale", now)
            self._publish_debug_image("STALE", "uav_pose_stale", det)
            return
        if det is None:
            self.last_det_conf = -1.0
            self.last_range_source = "none"
            self.last_range_used_m = -1.0
            self.last_heading_source = "none"
            reason = "no_detection" if self.last_external_det_stamp is not None else "detection_missing"
            if self.last_external_det_reject_reason != "none":
                reason = self.last_external_det_reject_reason
            self._record_audit_state("NO_DET")
            self.publish_status_msg(self._status_line("NO_DET", reason, now, audit_origin=self._no_det_origin(now)))
            self.publish_distance_status_msg(now)
            self._record_fault("NO_DET", reason, now)
            self._publish_debug_image("NO_DET", reason, None)
            self._publish_selected_target(now, None)
            return

        try:
            pose, _ = self._estimate_from_detection(det, self.camera_model, now)
        except ValueError as exc:
            reason = str(exc) or "estimate_failed"
            self.last_heading_source = "none"
            self._record_audit_state("STALE")
            self.publish_status_msg(self._status_line("STALE", reason, now, audit_origin="estimator_reject"))
            self.publish_distance_status_msg(now)
            self._record_fault("STALE", reason, now)
            self._publish_debug_image("STALE", reason, det)
            return
        except Exception as exc:
            reason = f"estimate_failed:{exc}"
            self.last_heading_source = "none"
            self._record_audit_state("STALE")
            self.publish_status_msg(self._status_line("STALE", reason, now, audit_origin="estimator_exception"))
            self.publish_distance_status_msg(now)
            self._record_fault("STALE", reason, now)
            self._publish_debug_image("STALE", reason, det)
            return

        projected_range_m = float(self.last_range_used_m) if self.last_range_source == "depth" else None
        self._publish_selected_target(now, det, projected_range_m)
        self._cached_estimate_pose = pose
        self._cached_estimate_track_id = det.track_id
        self._last_projected_det_rx_ns = det_rx_ns

        self._publish_estimate(pose, now, det.track_id)
        self._record_audit_state("OK")
        self.publish_status_msg(self._status_line("OK", "none", now, audit_origin="estimate_ok"))
        self.publish_distance_status_msg(now)
        self.last_debug_state = "OK"
        self.last_debug_det = det
        self._publish_debug_image("OK", "none", det)


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
