#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Float32, String
from vision_msgs.msg import Detection2DArray

from lrs_halmstad.common.selected_target_state import (
    SelectedTargetState,
    decode_selected_target_state_msg,
)
from lrs_halmstad.common.visual_target_estimate import (
    VisualTargetEstimate,
    decode_visual_target_estimate_msg,
)
from lrs_halmstad.follow.follow_math import coerce_bool
from lrs_halmstad.perception.yolo_common import cv2, image_to_bgr


class VisualFollowController(Node):
    """Step 2 test-mode visual-follow controller driven by a clean selected-target state."""

    def __init__(self) -> None:
        super().__init__("visual_follow_controller")

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("selected_target_topic", "/coord/leader_selected_target_filtered")
        self.declare_parameter("target_estimate_topic", "/coord/leader_visual_target_estimate")
        self.declare_parameter("out_topic", "/coord/leader_visual_control")
        self.declare_parameter("status_topic", "/coord/leader_visual_control_status")
        self.declare_parameter("debug_image_topic", "/coord/leader_visual_control_debug_image")
        self.declare_parameter("publish_status", True)
        self.declare_parameter("publish_debug_topics", True)
        self.declare_parameter("publish_debug_image", True)
        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("target_estimate_timeout_s", 1.0)
        self.declare_parameter("prefer_target_estimate", True)
        self.declare_parameter("loss_hold_timeout_s", 0.35)
        self.declare_parameter("distance_signal_mode", "auto")
        self.declare_parameter("prefer_intrinsics_normalization", True)
        self.declare_parameter("target_range_m", 7.0)
        self.declare_parameter("target_area_norm", 0.05)
        self.declare_parameter("yaw_gain", 2.0)
        self.declare_parameter("forward_range_gain", 0.75)
        self.declare_parameter("forward_area_gain", 6.0)
        self.declare_parameter("yaw_error_deadband_norm", 0.01)
        self.declare_parameter("range_error_deadband_m", 0.25)
        self.declare_parameter("area_error_deadband_norm", 0.015)
        self.declare_parameter("min_area_norm_valid", 0.002)
        self.declare_parameter("range_valid_min_m", 0.5)
        self.declare_parameter("range_valid_max_m", 25.0)
        self.declare_parameter("auto_range_reacquire_ticks", 3)
        self.declare_parameter("auto_range_loss_ticks", 4)
        self.declare_parameter("yaw_rate_max_rad_s", 1.0)
        self.declare_parameter("forward_speed_max_mps", 2.0)
        self.declare_parameter("yaw_slew_rate_rad_s2", 3.0)
        self.declare_parameter("forward_slew_rate_mps2", 2.0)
        self.declare_parameter("yaw_release_rate_rad_s2", 6.0)
        self.declare_parameter("forward_release_rate_mps2", 4.0)

        self.uav_name = str(self.get_parameter("uav_name").value).strip() or "dji0"
        self.camera_topic = (
            str(self.get_parameter("camera_topic").value).strip()
            or f"/{self.uav_name}/camera0/image_raw"
        )
        self.camera_info_topic = (
            str(self.get_parameter("camera_info_topic").value).strip()
            or f"/{self.uav_name}/camera0/camera_info"
        )
        self.selected_target_topic = str(self.get_parameter("selected_target_topic").value).strip()
        self.target_estimate_topic = str(self.get_parameter("target_estimate_topic").value).strip()
        self.out_topic = str(self.get_parameter("out_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.debug_image_topic = str(self.get_parameter("debug_image_topic").value).strip()
        self.publish_status = coerce_bool(self.get_parameter("publish_status").value)
        self.publish_debug_topics = coerce_bool(self.get_parameter("publish_debug_topics").value)
        self.publish_debug_image = coerce_bool(self.get_parameter("publish_debug_image").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.target_estimate_timeout_s = float(self.get_parameter("target_estimate_timeout_s").value)
        self.prefer_target_estimate = coerce_bool(self.get_parameter("prefer_target_estimate").value)
        self.loss_hold_timeout_s = float(self.get_parameter("loss_hold_timeout_s").value)
        self.distance_signal_mode = str(self.get_parameter("distance_signal_mode").value).strip().lower()
        self.prefer_intrinsics_normalization = coerce_bool(
            self.get_parameter("prefer_intrinsics_normalization").value
        )
        self.target_range_m = float(self.get_parameter("target_range_m").value)
        self.target_area_norm = float(self.get_parameter("target_area_norm").value)
        self.yaw_gain = float(self.get_parameter("yaw_gain").value)
        self.forward_range_gain = float(self.get_parameter("forward_range_gain").value)
        self.forward_area_gain = float(self.get_parameter("forward_area_gain").value)
        self.yaw_error_deadband_norm = float(self.get_parameter("yaw_error_deadband_norm").value)
        self.range_error_deadband_m = float(self.get_parameter("range_error_deadband_m").value)
        self.area_error_deadband_norm = float(self.get_parameter("area_error_deadband_norm").value)
        self.min_area_norm_valid = float(self.get_parameter("min_area_norm_valid").value)
        self.range_valid_min_m = float(self.get_parameter("range_valid_min_m").value)
        self.range_valid_max_m = float(self.get_parameter("range_valid_max_m").value)
        self.auto_range_reacquire_ticks = int(self.get_parameter("auto_range_reacquire_ticks").value)
        self.auto_range_loss_ticks = int(self.get_parameter("auto_range_loss_ticks").value)
        self.yaw_rate_max = float(self.get_parameter("yaw_rate_max_rad_s").value)
        self.forward_speed_max = float(self.get_parameter("forward_speed_max_mps").value)
        self.yaw_slew_rate = float(self.get_parameter("yaw_slew_rate_rad_s2").value)
        self.forward_slew_rate = float(self.get_parameter("forward_slew_rate_mps2").value)
        self.yaw_release_rate = float(self.get_parameter("yaw_release_rate_rad_s2").value)
        self.forward_release_rate = float(self.get_parameter("forward_release_rate_mps2").value)

        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.target_timeout_s <= 0.0:
            raise ValueError("target_timeout_s must be > 0")
        if self.target_estimate_timeout_s <= 0.0:
            raise ValueError("target_estimate_timeout_s must be > 0")
        if self.loss_hold_timeout_s < 0.0:
            raise ValueError("loss_hold_timeout_s must be >= 0")
        if self.distance_signal_mode not in ("auto", "range", "area"):
            raise ValueError("distance_signal_mode must be one of: auto, range, area")
        if self.target_range_m <= 0.0:
            raise ValueError("target_range_m must be > 0")
        if self.target_area_norm <= 0.0:
            raise ValueError("target_area_norm must be > 0")
        if self.min_area_norm_valid < 0.0:
            raise ValueError("min_area_norm_valid must be >= 0")
        if self.range_valid_min_m <= 0.0 or self.range_valid_max_m <= self.range_valid_min_m:
            raise ValueError("range_valid_min_m/range_valid_max_m must define a positive valid range window")
        if self.auto_range_reacquire_ticks < 1 or self.auto_range_loss_ticks < 1:
            raise ValueError("auto_range_reacquire_ticks and auto_range_loss_ticks must be >= 1")
        if self.yaw_gain < 0.0 or self.forward_range_gain < 0.0 or self.forward_area_gain < 0.0:
            raise ValueError("gains must be >= 0")
        if self.yaw_rate_max < 0.0 or self.forward_speed_max < 0.0:
            raise ValueError("command limits must be >= 0")
        if (
            self.yaw_slew_rate < 0.0
            or self.forward_slew_rate < 0.0
            or self.yaw_release_rate < 0.0
            or self.forward_release_rate < 0.0
        ):
            raise ValueError("slew/release limits must be >= 0")

        self.camera_fx: Optional[float] = None
        self.camera_fy: Optional[float] = None
        self.camera_cx: Optional[float] = None
        self.camera_cy: Optional[float] = None
        self.camera_width: Optional[int] = None
        self.camera_height: Optional[int] = None

        self.last_target = SelectedTargetState(
            stamp=self.get_clock().now().to_msg(),
            frame_id="",
            valid=False,
        )
        self.last_target_stamp: Optional[Time] = None
        self.last_target_rx: Optional[Time] = None
        self.last_target_estimate = VisualTargetEstimate(
            stamp=self.get_clock().now().to_msg(),
            frame_id="",
            valid=False,
        )
        self.last_target_estimate_stamp: Optional[Time] = None
        self.last_target_estimate_rx: Optional[Time] = None
        self.last_image_msg: Optional[Image] = None
        self.last_image_stamp: Optional[Time] = None
        self.last_cmd_forward = 0.0
        self.last_cmd_yaw = 0.0
        self.last_valid_forward = 0.0
        self.last_valid_yaw = 0.0
        self.last_valid_target_time: Optional[Time] = None
        self.last_tick_time: Optional[Time] = None
        self.last_status_state = "INIT"
        self.active_distance_source = "none"
        self.range_reacquire_streak = 0
        self.range_loss_streak = 0

        camera_info_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

        self.create_subscription(Detection2DArray, self.selected_target_topic, self.on_selected_target, 10)
        self.create_subscription(Odometry, self.target_estimate_topic, self.on_target_estimate, 10)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, camera_info_qos)
        self.create_subscription(Image, self.camera_topic, self.on_image, 10)

        self.command_pub = self.create_publisher(TwistStamped, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10) if self.publish_status else None
        self.debug_image_pub = (
            self.create_publisher(Image, self.debug_image_topic, 2)
            if self.publish_debug_image and self.debug_image_topic
            else None
        )

        self.err_x_raw_pub = None
        self.err_y_raw_pub = None
        self.err_x_norm_pub = None
        self.err_y_norm_pub = None
        self.range_pub = None
        self.area_pub = None
        self.forward_pub = None
        self.yaw_pub = None
        self.mode_pub = None
        if self.publish_debug_topics:
            self.err_x_raw_pub = self.create_publisher(Float32, f"{self.out_topic}/debug/error_x_raw_px", 10)
            self.err_y_raw_pub = self.create_publisher(Float32, f"{self.out_topic}/debug/error_y_raw_px", 10)
            self.err_x_norm_pub = self.create_publisher(Float32, f"{self.out_topic}/debug/error_x_norm", 10)
            self.err_y_norm_pub = self.create_publisher(Float32, f"{self.out_topic}/debug/error_y_norm", 10)
            self.range_pub = self.create_publisher(Float32, f"{self.out_topic}/debug/projected_range_m", 10)
            self.area_pub = self.create_publisher(Float32, f"{self.out_topic}/debug/area_norm", 10)
            self.forward_pub = self.create_publisher(Float32, f"{self.out_topic}/debug/forward_speed_mps", 10)
            self.yaw_pub = self.create_publisher(Float32, f"{self.out_topic}/debug/yaw_rate_rad_s", 10)
            self.mode_pub = self.create_publisher(String, f"{self.out_topic}/debug/mode", 10)

        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)
        self.get_logger().info(
            "[visual_follow_controller] Started: "
            f"selected_target={self.selected_target_topic}, target_estimate={self.target_estimate_topic}, camera={self.camera_topic}, "
            f"camera_info={self.camera_info_topic}, out={self.out_topic}, "
            f"distance_signal_mode={self.distance_signal_mode}"
        )

    def on_selected_target(self, msg: Detection2DArray) -> None:
        self.last_target = decode_selected_target_state_msg(msg)
        self.last_target_rx = self.get_clock().now()
        try:
            self.last_target_stamp = Time.from_msg(self.last_target.stamp)
        except Exception:
            self.last_target_stamp = self.last_target_rx

    def on_target_estimate(self, msg: Odometry) -> None:
        self.last_target_estimate = decode_visual_target_estimate_msg(msg)
        self.last_target_estimate_rx = self.get_clock().now()
        try:
            self.last_target_estimate_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_target_estimate_stamp = self.last_target_estimate_rx

    def on_camera_info(self, msg: CameraInfo) -> None:
        if len(msg.k) >= 9:
            fx = float(msg.k[0])
            fy = float(msg.k[4])
            if fx > 0.0 and fy > 0.0:
                self.camera_fx = fx
                self.camera_fy = fy
                self.camera_cx = float(msg.k[2])
                self.camera_cy = float(msg.k[5])
        self.camera_width = int(msg.width) if int(msg.width) > 0 else self.camera_width
        self.camera_height = int(msg.height) if int(msg.height) > 0 else self.camera_height

    def on_image(self, msg: Image) -> None:
        self.last_image_msg = msg
        self.camera_width = int(msg.width) if int(msg.width) > 0 else self.camera_width
        self.camera_height = int(msg.height) if int(msg.height) > 0 else self.camera_height
        try:
            self.last_image_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_image_stamp = self.get_clock().now()

    def _target_is_fresh(self, now: Time) -> bool:
        if self.last_target_stamp is None:
            return False
        return (now - self.last_target_stamp).nanoseconds * 1e-9 <= self.target_timeout_s

    def _target_estimate_is_fresh(self, now: Time) -> bool:
        if self.last_target_estimate_stamp is None:
            return False
        return (now - self.last_target_estimate_stamp).nanoseconds * 1e-9 <= self.target_estimate_timeout_s

    def _dt_s(self, now: Time) -> float:
        if self.last_tick_time is None:
            return 1.0 / self.tick_hz
        dt = (now - self.last_tick_time).nanoseconds * 1e-9
        return max(1.0 / self.tick_hz, min(0.5, dt))

    def _image_geometry(self) -> Tuple[Optional[float], Optional[float], Optional[float], Optional[float], str]:
        if (
            self.prefer_intrinsics_normalization
            and self.camera_fx is not None
            and self.camera_fy is not None
            and self.camera_cx is not None
            and self.camera_cy is not None
        ):
            return self.camera_cx, self.camera_cy, self.camera_fx, self.camera_fy, "intrinsics"
        if self.camera_width is not None and self.camera_height is not None:
            half_w = max(1.0, 0.5 * float(self.camera_width))
            half_h = max(1.0, 0.5 * float(self.camera_height))
            return half_w, half_h, half_w, half_h, "image_size"
        return None, None, None, None, "missing"

    def _raw_and_normalized_errors(
        self,
        target: SelectedTargetState,
    ) -> Tuple[float, float, float, float, str]:
        center_u, center_v, scale_x, scale_y, norm_mode = self._image_geometry()
        if center_u is None or center_v is None or scale_x is None or scale_y is None:
            raise RuntimeError("image_geometry_missing")
        raw_x = float(target.bbox_center_x_px) - float(center_u)
        raw_y = float(target.bbox_center_y_px) - float(center_v)
        norm_x = raw_x / float(scale_x)
        norm_y = raw_y / float(scale_y)
        return raw_x, raw_y, norm_x, norm_y, norm_mode

    def _raw_and_normalized_errors_from_estimate(
        self,
        estimate: VisualTargetEstimate,
    ) -> Tuple[float, float, float, float, str]:
        if not estimate.valid or not math.isfinite(estimate.rel_z_m) or estimate.rel_z_m <= 0.0:
            raise RuntimeError("target_estimate_invalid")
        norm_x = float(estimate.rel_x_m) / float(estimate.rel_z_m)
        norm_y = float(estimate.rel_y_m) / float(estimate.rel_z_m)
        center_u, center_v, scale_x, scale_y, norm_mode = self._image_geometry()
        if center_u is not None and center_v is not None and scale_x is not None and scale_y is not None:
            raw_x = norm_x * float(scale_x)
            raw_y = norm_y * float(scale_y)
            return raw_x, raw_y, norm_x, norm_y, f"estimate_{norm_mode}"
        return norm_x, norm_y, norm_x, norm_y, "estimate"

    @staticmethod
    def _clamp_symmetric(value: float, limit: float) -> float:
        limit = max(0.0, float(limit))
        return max(-limit, min(limit, float(value)))

    @staticmethod
    def _slew(current: float, target: float, rise_limit_per_s: float, fall_limit_per_s: float, dt_s: float) -> float:
        if rise_limit_per_s <= 0.0 and fall_limit_per_s <= 0.0:
            return float(target)
        delta = float(target) - float(current)
        limit_per_s = float(rise_limit_per_s) if abs(float(target)) >= abs(float(current)) else float(fall_limit_per_s)
        if limit_per_s <= 0.0:
            return float(target)
        max_delta = limit_per_s * max(0.0, float(dt_s))
        if abs(delta) <= max_delta:
            return float(target)
        return float(current) + math.copysign(max_delta, delta)

    def _area_norm(self, target: SelectedTargetState) -> Optional[float]:
        if self.camera_width is None or self.camera_height is None:
            return None
        image_area = float(self.camera_width * self.camera_height)
        if image_area <= 0.0:
            return None
        area = max(0.0, float(target.bbox_size_x_px)) * max(0.0, float(target.bbox_size_y_px))
        return float(area / image_area)

    def _range_available(self, projected_range_m: Optional[float]) -> bool:
        if projected_range_m is None or not math.isfinite(projected_range_m):
            return False
        return self.range_valid_min_m <= float(projected_range_m) <= self.range_valid_max_m

    def _area_available(self, area_norm: Optional[float]) -> bool:
        return area_norm is not None and math.isfinite(area_norm) and area_norm >= self.min_area_norm_valid

    def _select_distance_source(self, range_available: bool, area_available: bool) -> str:
        if self.distance_signal_mode == "range":
            self.active_distance_source = "range" if range_available else "none"
            self.range_reacquire_streak = 0
            self.range_loss_streak = 0
            return self.active_distance_source

        if self.distance_signal_mode == "area":
            self.active_distance_source = "area" if area_available else "none"
            self.range_reacquire_streak = 0
            self.range_loss_streak = 0
            return self.active_distance_source

        active = self.active_distance_source
        if active not in ("range", "area"):
            if range_available:
                active = "range"
            elif area_available:
                active = "area"
            else:
                active = "none"
            self.active_distance_source = active
            self.range_reacquire_streak = 0
            self.range_loss_streak = 0
            return active

        if active == "range":
            self.range_reacquire_streak = 0
            if range_available:
                self.range_loss_streak = 0
                return active
            self.range_loss_streak += 1
            if self.range_loss_streak >= self.auto_range_loss_ticks:
                active = "area" if area_available else "none"
                self.active_distance_source = active
                self.range_loss_streak = 0
            return self.active_distance_source

        if area_available:
            if range_available:
                self.range_reacquire_streak += 1
                if self.range_reacquire_streak >= self.auto_range_reacquire_ticks:
                    self.active_distance_source = "range"
                    self.range_reacquire_streak = 0
                    self.range_loss_streak = 0
            else:
                self.range_reacquire_streak = 0
            return self.active_distance_source

        self.range_reacquire_streak = 0
        if range_available:
            self.active_distance_source = "range"
        else:
            self.active_distance_source = "none"
        self.range_loss_streak = 0
        return self.active_distance_source

    def _distance_signal(
        self,
        target: SelectedTargetState,
        area_norm: Optional[float],
    ) -> Tuple[str, str, Optional[float], Optional[float], float, bool, bool]:
        projected_range = target.projected_range_m
        range_available = self._range_available(projected_range)
        area_available = self._area_available(area_norm)
        active_source = self._select_distance_source(range_available, area_available)

        if self.distance_signal_mode == "range":
            if range_available:
                range_error = float(projected_range) - self.target_range_m
                if abs(range_error) < self.range_error_deadband_m:
                    range_error = 0.0
                forward = self.forward_range_gain * range_error
                return "range", active_source, float(projected_range), area_norm, forward, range_available, area_available
            return "range_unavailable", active_source, None, area_norm, 0.0, range_available, area_available

        if self.distance_signal_mode == "area":
            if not area_available:
                return "area_unavailable", active_source, projected_range if range_available else None, None, 0.0, range_available, area_available
            target_area_proxy = math.sqrt(self.target_area_norm)
            current_area_proxy = math.sqrt(area_norm)
            area_error = target_area_proxy - current_area_proxy
            if abs(area_error) < self.area_error_deadband_norm:
                area_error = 0.0
            forward = self.forward_area_gain * area_error
            return "area", active_source, projected_range if range_available else None, area_norm, forward, range_available, area_available

        if active_source == "range" and range_available:
            range_error = float(projected_range) - self.target_range_m
            if abs(range_error) < self.range_error_deadband_m:
                range_error = 0.0
            forward = self.forward_range_gain * range_error
            return "range", active_source, float(projected_range), area_norm, forward, range_available, area_available

        if active_source == "range":
            return "range_hold", active_source, None, area_norm if area_available else None, 0.0, range_available, area_available

        if active_source == "area" and area_available:
            target_area_proxy = math.sqrt(self.target_area_norm)
            current_area_proxy = math.sqrt(area_norm)
            area_error = target_area_proxy - current_area_proxy
            if abs(area_error) < self.area_error_deadband_norm:
                area_error = 0.0
            forward = self.forward_area_gain * area_error
            return "area", active_source, projected_range if range_available else None, area_norm, forward, range_available, area_available

        return "none", active_source, projected_range if range_available else None, area_norm if area_available else None, 0.0, range_available, area_available

    def _hold_decay_factor(self, now: Time) -> float:
        if self.last_valid_target_time is None or self.loss_hold_timeout_s <= 0.0:
            return 0.0
        age_s = max(0.0, (now - self.last_valid_target_time).nanoseconds * 1e-9)
        if age_s >= self.loss_hold_timeout_s:
            return 0.0
        return max(0.0, 1.0 - (age_s / self.loss_hold_timeout_s))

    @staticmethod
    def _rotated_box_corners(
        center_x: float,
        center_y: float,
        size_x: float,
        size_y: float,
        theta: float,
    ) -> np.ndarray:
        half_x = 0.5 * float(size_x)
        half_y = 0.5 * float(size_y)
        corners = np.asarray(
            [
                [-half_x, -half_y],
                [half_x, -half_y],
                [half_x, half_y],
                [-half_x, half_y],
            ],
            dtype=np.float64,
        )
        c = math.cos(theta)
        s = math.sin(theta)
        rot = np.asarray([[c, -s], [s, c]], dtype=np.float64)
        rotated = corners @ rot.T
        rotated[:, 0] += float(center_x)
        rotated[:, 1] += float(center_y)
        return rotated

    def _publish_debug_scalars(
        self,
        *,
        raw_x_px: float,
        raw_y_px: float,
        norm_x: float,
        norm_y: float,
        projected_range_m: Optional[float],
        area_norm: Optional[float],
        forward_speed_mps: float,
        yaw_rate_rad_s: float,
        mode: str,
        active_source: str,
        range_available: bool,
        area_available: bool,
    ) -> None:
        if (
            self.err_x_raw_pub is None
            or self.err_y_raw_pub is None
            or self.err_x_norm_pub is None
            or self.err_y_norm_pub is None
            or self.range_pub is None
            or self.area_pub is None
            or self.forward_pub is None
            or self.yaw_pub is None
            or self.mode_pub is None
        ):
            return

        def publish_scalar(pub, value: Optional[float]) -> None:
            msg = Float32()
            msg.data = float("nan") if value is None else float(value)
            pub.publish(msg)

        publish_scalar(self.err_x_raw_pub, raw_x_px)
        publish_scalar(self.err_y_raw_pub, raw_y_px)
        publish_scalar(self.err_x_norm_pub, norm_x)
        publish_scalar(self.err_y_norm_pub, norm_y)
        publish_scalar(self.range_pub, projected_range_m)
        publish_scalar(self.area_pub, area_norm)
        publish_scalar(self.forward_pub, forward_speed_mps)
        publish_scalar(self.yaw_pub, yaw_rate_rad_s)
        mode_msg = String()
        mode_msg.data = (
            f"mode={mode} active_source={active_source} "
            f"range_available={'true' if range_available else 'false'} "
            f"area_available={'true' if area_available else 'false'} "
            f"range_reacquire_streak={self.range_reacquire_streak} "
            f"range_loss_streak={self.range_loss_streak}"
        )
        self.mode_pub.publish(mode_msg)

    def _publish_status(
        self,
        *,
        state: str,
        reason: str,
        norm_mode: str,
        distance_mode: str,
        active_source: str,
        range_available: bool,
        area_available: bool,
        raw_x_px: float,
        raw_y_px: float,
        norm_x: float,
        norm_y: float,
        projected_range_m: Optional[float],
        area_norm: Optional[float],
        yaw_rate_rad_s: float,
        forward_speed_mps: float,
        target: SelectedTargetState,
    ) -> None:
        if self.status_pub is None:
            return
        track_id = target.track_id or (self.last_target_estimate.track_id if self.last_target_estimate.valid else "none")
        confidence = target.confidence if target.valid else 0.0
        msg = String()
        msg.data = (
            f"state={state} "
            f"reason={reason} "
            f"norm_mode={norm_mode} "
            f"distance_mode={distance_mode} "
            f"active_source={active_source} "
            f"requested_source={self.distance_signal_mode} "
            f"range_available={'true' if range_available else 'false'} "
            f"area_available={'true' if area_available else 'false'} "
            f"range_reacquire_streak={self.range_reacquire_streak} "
            f"range_loss_streak={self.range_loss_streak} "
            f"track_id={track_id or 'none'} "
            f"conf={confidence:.3f} "
            f"err_x_raw_px={raw_x_px:.2f} "
            f"err_y_raw_px={raw_y_px:.2f} "
            f"err_x_norm={norm_x:.4f} "
            f"err_y_norm={norm_y:.4f} "
            f"projected_range_m={'na' if projected_range_m is None else f'{projected_range_m:.2f}'} "
            f"area_norm={'na' if area_norm is None else f'{area_norm:.4f}'} "
            f"yaw_rate_rad_s={yaw_rate_rad_s:.3f} "
            f"forward_speed_mps={forward_speed_mps:.3f}"
        )
        self.status_pub.publish(msg)

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

    def _publish_debug_image(
        self,
        *,
        state: str,
        reason: str,
        target: SelectedTargetState,
        raw_x_px: float,
        raw_y_px: float,
        norm_x: float,
        norm_y: float,
        distance_mode: str,
        active_source: str,
        range_available: bool,
        area_available: bool,
        yaw_rate_rad_s: float,
        forward_speed_mps: float,
        projected_range_m: Optional[float],
        area_norm: Optional[float],
    ) -> None:
        if self.debug_image_pub is None or self.last_image_msg is None:
            return
        img_bgr = image_to_bgr(self.last_image_msg)
        if img_bgr is None or cv2 is None:
            return

        out = img_bgr.copy()
        desired_u = int(round(self.camera_cx if self.camera_cx is not None else 0.5 * out.shape[1]))
        desired_v = int(round(self.camera_cy if self.camera_cy is not None else 0.5 * out.shape[0]))
        cv2.drawMarker(out, (desired_u, desired_v), (0, 255, 255), cv2.MARKER_CROSS, 18, 2, cv2.LINE_AA)

        if target.valid:
            corners = self._rotated_box_corners(
                target.bbox_center_x_px,
                target.bbox_center_y_px,
                target.bbox_size_x_px,
                target.bbox_size_y_px,
                target.bbox_theta_rad,
            )
            pts = np.asarray(corners, dtype=np.int32).reshape((-1, 1, 2))
            cv2.polylines(out, [pts], True, (0, 220, 0), 2, cv2.LINE_AA)
            cx = int(round(target.bbox_center_x_px))
            cy = int(round(target.bbox_center_y_px))
            cv2.circle(out, (cx, cy), 4, (0, 220, 0), -1)
            cv2.line(out, (desired_u, desired_v), (cx, cy), (0, 165, 255), 2, cv2.LINE_AA)

        lines = [
            f"state={state} reason={reason}",
            f"err_px=({raw_x_px:.1f},{raw_y_px:.1f}) err_norm=({norm_x:.3f},{norm_y:.3f})",
            f"dist={distance_mode} active={active_source} range_ok={int(range_available)} area_ok={int(area_available)}",
            f"range={'na' if projected_range_m is None else f'{projected_range_m:.2f}'} area={'na' if area_norm is None else f'{area_norm:.4f}'}",
            f"cmd_fwd={forward_speed_mps:.2f} cmd_yaw={yaw_rate_rad_s:.2f}",
        ]
        y = 20
        for line in lines:
            cv2.putText(out, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (20, 20, 20), 3, cv2.LINE_AA)
            cv2.putText(out, line, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)
            y += 18

        msg = self._bgr_to_image_msg(out)
        if msg is not None:
            self.debug_image_pub.publish(msg)

    def _publish_command(self, now: Time, forward_speed_mps: float, yaw_rate_rad_s: float) -> None:
        msg = TwistStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = f"{self.uav_name}/base_link"
        msg.twist.linear.x = float(forward_speed_mps)
        msg.twist.angular.z = float(yaw_rate_rad_s)
        self.command_pub.publish(msg)

    def on_tick(self) -> None:
        now = self.get_clock().now()
        dt_s = self._dt_s(now)
        target = self.last_target if self._target_is_fresh(now) else SelectedTargetState(
            stamp=now.to_msg(),
            frame_id="",
            valid=False,
        )
        target_estimate = (
            self.last_target_estimate
            if self.prefer_target_estimate and self._target_estimate_is_fresh(now) and self.last_target_estimate.valid
            else VisualTargetEstimate(stamp=now.to_msg(), frame_id="", valid=False)
        )

        raw_x_px = 0.0
        raw_y_px = 0.0
        norm_x = 0.0
        norm_y = 0.0
        norm_mode = "missing"
        projected_range_m = None
        area_norm = None
        desired_forward = 0.0
        desired_yaw = 0.0
        state = "LOST"
        reason = "target_missing"
        distance_mode = "none"
        active_source = self.active_distance_source
        range_available = False
        area_available = False

        if target_estimate.valid:
            try:
                raw_x_px, raw_y_px, norm_x, norm_y, norm_mode = self._raw_and_normalized_errors_from_estimate(
                    target_estimate
                )
                area_norm = self._area_norm(target) if target.valid else None
                projected_range_m = float(target_estimate.rel_z_m)
                distance_mode = "estimate"
                active_source = "estimate"
                self.active_distance_source = "none"
                self.range_reacquire_streak = 0
                self.range_loss_streak = 0
                range_available = self._range_available(projected_range_m)
                area_available = self._area_available(area_norm)
                if range_available:
                    range_error = projected_range_m - self.target_range_m
                    if abs(range_error) < self.range_error_deadband_m:
                        range_error = 0.0
                    desired_forward = self.forward_range_gain * range_error
                else:
                    desired_forward = 0.0
                if abs(norm_x) < self.yaw_error_deadband_norm:
                    norm_x = 0.0
                desired_yaw = -self.yaw_gain * norm_x
                desired_yaw = self._clamp_symmetric(desired_yaw, self.yaw_rate_max)
                desired_forward = self._clamp_symmetric(desired_forward, self.forward_speed_max)
                state = "TRACK" if range_available else "TRACK_YAW_ONLY"
                reason = "none"
                self.last_valid_forward = desired_forward
                self.last_valid_yaw = desired_yaw
                self.last_valid_target_time = now
            except RuntimeError:
                state = "WAIT_ESTIMATE"
                reason = "target_estimate_invalid"
                desired_forward = 0.0
                desired_yaw = 0.0
        elif target.valid:
            try:
                raw_x_px, raw_y_px, norm_x, norm_y, norm_mode = self._raw_and_normalized_errors(target)
                area_norm = self._area_norm(target)
                projected_range_m = target.projected_range_m
                (
                    distance_mode,
                    active_source,
                    projected_range_m,
                    area_norm,
                    desired_forward,
                    range_available,
                    area_available,
                ) = self._distance_signal(
                    target,
                    area_norm,
                )
                if abs(norm_x) < self.yaw_error_deadband_norm:
                    norm_x = 0.0
                desired_yaw = -self.yaw_gain * norm_x
                desired_yaw = self._clamp_symmetric(desired_yaw, self.yaw_rate_max)
                desired_forward = self._clamp_symmetric(desired_forward, self.forward_speed_max)
                state = "TRACK" if distance_mode in ("range", "area") else "TRACK_YAW_ONLY"
                reason = "selected_target_fallback"
                self.last_valid_forward = desired_forward
                self.last_valid_yaw = desired_yaw
                self.last_valid_target_time = now
            except RuntimeError:
                state = "WAIT_CAMERA"
                reason = "image_geometry_missing"
                desired_forward = 0.0
                desired_yaw = 0.0
        else:
            hold_factor = self._hold_decay_factor(now)
            if hold_factor > 0.0:
                state = "HOLD"
                reason = "short_loss_hold"
                distance_mode = "hold"
                active_source = self.active_distance_source
                desired_forward = self.last_valid_forward * hold_factor
                desired_yaw = self.last_valid_yaw * hold_factor

        cmd_forward = self._slew(
            self.last_cmd_forward,
            desired_forward,
            self.forward_slew_rate,
            self.forward_release_rate,
            dt_s,
        )
        cmd_yaw = self._slew(
            self.last_cmd_yaw,
            desired_yaw,
            self.yaw_slew_rate,
            self.yaw_release_rate,
            dt_s,
        )

        self._publish_command(now, cmd_forward, cmd_yaw)
        self._publish_debug_scalars(
            raw_x_px=raw_x_px,
            raw_y_px=raw_y_px,
            norm_x=norm_x,
            norm_y=norm_y,
            projected_range_m=projected_range_m,
            area_norm=area_norm,
            forward_speed_mps=cmd_forward,
            yaw_rate_rad_s=cmd_yaw,
            mode=distance_mode,
            active_source=active_source,
            range_available=range_available,
            area_available=area_available,
        )
        self._publish_status(
            state=state,
            reason=reason,
            norm_mode=norm_mode,
            distance_mode=distance_mode,
            active_source=active_source,
            range_available=range_available,
            area_available=area_available,
            raw_x_px=raw_x_px,
            raw_y_px=raw_y_px,
            norm_x=norm_x,
            norm_y=norm_y,
            projected_range_m=projected_range_m,
            area_norm=area_norm,
            yaw_rate_rad_s=cmd_yaw,
            forward_speed_mps=cmd_forward,
            target=target,
        )
        self._publish_debug_image(
            state=state,
            reason=reason,
            target=target,
            raw_x_px=raw_x_px,
            raw_y_px=raw_y_px,
            norm_x=norm_x,
            norm_y=norm_y,
            distance_mode=distance_mode,
            active_source=active_source,
            range_available=range_available,
            area_available=area_available,
            yaw_rate_rad_s=cmd_yaw,
            forward_speed_mps=cmd_forward,
            projected_range_m=projected_range_m,
            area_norm=area_norm,
        )

        self.last_cmd_forward = cmd_forward
        self.last_cmd_yaw = cmd_yaw
        self.last_tick_time = now
        self.last_status_state = state


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualFollowController()
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
