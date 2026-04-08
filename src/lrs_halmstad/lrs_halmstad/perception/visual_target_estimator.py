#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional, Tuple

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from lrs_halmstad.common.selected_target_state import (
    SelectedTargetState,
    decode_selected_target_state_msg,
)
from lrs_halmstad.common.visual_target_estimate import (
    VisualTargetEstimate,
    encode_visual_target_estimate_msg,
)
from lrs_halmstad.follow.follow_math import coerce_bool


class VisualTargetEstimator(Node):
    """Sensors-inspired relative-state estimator for the visual-follow controller."""

    def __init__(self) -> None:
        super().__init__("visual_target_estimator")

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("selected_target_topic", "/coord/leader_selected_target_filtered")
        self.declare_parameter("camera_info_topic", "")
        self.declare_parameter("out_topic", "/coord/leader_visual_target_estimate")
        self.declare_parameter("status_topic", "/coord/leader_visual_target_estimate_status")
        self.declare_parameter("publish_status", True)
        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("prediction_max_gap_s", 0.35)
        self.declare_parameter("prefer_intrinsics_normalization", True)
        self.declare_parameter("range_signal_mode", "auto")
        self.declare_parameter("reference_range_m", 7.0)
        self.declare_parameter("reference_area_norm", 0.009)
        self.declare_parameter("min_area_norm_valid", 0.002)
        self.declare_parameter("projected_range_min_m", 0.5)
        self.declare_parameter("projected_range_max_m", 25.0)
        self.declare_parameter("auto_projected_reacquire_ticks", 3)
        self.declare_parameter("auto_projected_loss_ticks", 4)
        self.declare_parameter("position_alpha_projected", 0.65)
        self.declare_parameter("position_alpha_area", 0.45)
        self.declare_parameter("velocity_beta_projected", 0.35)
        self.declare_parameter("velocity_beta_area", 0.18)
        self.declare_parameter("max_velocity_mps", 8.0)
        self.declare_parameter("degraded_after_s", 0.75)
        self.declare_parameter("hard_lost_after_s", 1.35)
        self.declare_parameter("continuity_hit_gain", 0.18)
        self.declare_parameter("continuity_miss_decay_per_s", 1.10)
        self.declare_parameter("predicted_velocity_decay", 0.92)
        self.declare_parameter("degraded_velocity_decay", 0.65)
        self.declare_parameter("visibility_center_weight", 0.35)
        self.declare_parameter("visibility_edge_weight", 0.40)
        self.declare_parameter("visibility_area_weight", 0.25)
        self.declare_parameter("visibility_center_soft_radius_norm", 0.60)
        self.declare_parameter("visibility_edge_soft_margin_norm", 0.18)

        self.uav_name = str(self.get_parameter("uav_name").value).strip() or "dji0"
        self.selected_target_topic = str(self.get_parameter("selected_target_topic").value).strip()
        self.camera_info_topic = (
            str(self.get_parameter("camera_info_topic").value).strip()
            or f"/{self.uav_name}/camera0/camera_info"
        )
        self.out_topic = str(self.get_parameter("out_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.publish_status = coerce_bool(self.get_parameter("publish_status").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.prediction_max_gap_s = float(self.get_parameter("prediction_max_gap_s").value)
        self.prefer_intrinsics_normalization = coerce_bool(
            self.get_parameter("prefer_intrinsics_normalization").value
        )
        self.range_signal_mode = str(self.get_parameter("range_signal_mode").value).strip().lower()
        self.reference_range_m = float(self.get_parameter("reference_range_m").value)
        self.reference_area_norm = float(self.get_parameter("reference_area_norm").value)
        self.min_area_norm_valid = float(self.get_parameter("min_area_norm_valid").value)
        self.projected_range_min_m = float(self.get_parameter("projected_range_min_m").value)
        self.projected_range_max_m = float(self.get_parameter("projected_range_max_m").value)
        self.auto_projected_reacquire_ticks = int(
            self.get_parameter("auto_projected_reacquire_ticks").value
        )
        self.auto_projected_loss_ticks = int(self.get_parameter("auto_projected_loss_ticks").value)
        self.position_alpha_projected = float(self.get_parameter("position_alpha_projected").value)
        self.position_alpha_area = float(self.get_parameter("position_alpha_area").value)
        self.velocity_beta_projected = float(self.get_parameter("velocity_beta_projected").value)
        self.velocity_beta_area = float(self.get_parameter("velocity_beta_area").value)
        self.max_velocity_mps = float(self.get_parameter("max_velocity_mps").value)
        self.degraded_after_s = float(self.get_parameter("degraded_after_s").value)
        self.hard_lost_after_s = float(self.get_parameter("hard_lost_after_s").value)
        self.continuity_hit_gain = float(self.get_parameter("continuity_hit_gain").value)
        self.continuity_miss_decay_per_s = float(self.get_parameter("continuity_miss_decay_per_s").value)
        self.predicted_velocity_decay = float(self.get_parameter("predicted_velocity_decay").value)
        self.degraded_velocity_decay = float(self.get_parameter("degraded_velocity_decay").value)
        self.visibility_center_weight = float(self.get_parameter("visibility_center_weight").value)
        self.visibility_edge_weight = float(self.get_parameter("visibility_edge_weight").value)
        self.visibility_area_weight = float(self.get_parameter("visibility_area_weight").value)
        self.visibility_center_soft_radius_norm = float(
            self.get_parameter("visibility_center_soft_radius_norm").value
        )
        self.visibility_edge_soft_margin_norm = float(
            self.get_parameter("visibility_edge_soft_margin_norm").value
        )

        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.target_timeout_s <= 0.0:
            raise ValueError("target_timeout_s must be > 0")
        if self.prediction_max_gap_s < 0.0:
            raise ValueError("prediction_max_gap_s must be >= 0")
        if self.degraded_after_s < self.prediction_max_gap_s:
            raise ValueError("degraded_after_s must be >= prediction_max_gap_s")
        if self.hard_lost_after_s < self.degraded_after_s:
            raise ValueError("hard_lost_after_s must be >= degraded_after_s")
        if self.range_signal_mode not in ("auto", "projected", "area"):
            raise ValueError("range_signal_mode must be one of: auto, projected, area")
        if self.reference_range_m <= 0.0 or self.reference_area_norm <= 0.0:
            raise ValueError("reference_range_m and reference_area_norm must be > 0")
        if self.min_area_norm_valid < 0.0:
            raise ValueError("min_area_norm_valid must be >= 0")
        if (
            self.projected_range_min_m <= 0.0
            or self.projected_range_max_m <= self.projected_range_min_m
        ):
            raise ValueError("projected_range_min_m/projected_range_max_m must define a valid range window")
        if self.auto_projected_reacquire_ticks < 1 or self.auto_projected_loss_ticks < 1:
            raise ValueError("auto_projected_reacquire_ticks/auto_projected_loss_ticks must be >= 1")
        if self.max_velocity_mps < 0.0:
            raise ValueError("max_velocity_mps must be >= 0")
        if self.continuity_hit_gain < 0.0:
            raise ValueError("continuity_hit_gain must be >= 0")
        if self.continuity_miss_decay_per_s < 0.0:
            raise ValueError("continuity_miss_decay_per_s must be >= 0")
        if not (0.0 <= self.predicted_velocity_decay <= 1.0):
            raise ValueError("predicted_velocity_decay must be within [0, 1]")
        if not (0.0 <= self.degraded_velocity_decay <= 1.0):
            raise ValueError("degraded_velocity_decay must be within [0, 1]")
        if self.visibility_center_weight < 0.0 or self.visibility_edge_weight < 0.0 or self.visibility_area_weight < 0.0:
            raise ValueError("visibility weights must be >= 0")
        if self.visibility_center_soft_radius_norm <= 0.0:
            raise ValueError("visibility_center_soft_radius_norm must be > 0")
        if self.visibility_edge_soft_margin_norm <= 0.0:
            raise ValueError("visibility_edge_soft_margin_norm must be > 0")

        self.camera_fx: Optional[float] = None
        self.camera_fy: Optional[float] = None
        self.camera_cx: Optional[float] = None
        self.camera_cy: Optional[float] = None
        self.camera_width: Optional[int] = None
        self.camera_height: Optional[int] = None
        self.camera_frame_id: str = ""

        now_msg = self.get_clock().now().to_msg()
        self.last_target = SelectedTargetState(stamp=now_msg, frame_id="", valid=False)
        self.last_target_stamp: Optional[Time] = None
        self.last_target_rx: Optional[Time] = None

        self.estimate = VisualTargetEstimate(stamp=now_msg, frame_id="", valid=False)
        self.last_estimate_update: Optional[Time] = None
        self.last_measurement_time: Optional[Time] = None
        self.last_tick_time: Optional[Time] = None
        self.active_range_source = "none"
        self.projected_reacquire_streak = 0
        self.projected_loss_streak = 0
        self.last_status_state = "INIT"
        self.continuity_score = 0.0
        self.consecutive_hits = 0
        self.consecutive_misses = 0
        self.last_visibility_score = 0.0

        self.create_subscription(Detection2DArray, self.selected_target_topic, self.on_selected_target, 1)
        self.create_subscription(CameraInfo, self.camera_info_topic, self.on_camera_info, 10)
        self.out_pub = self.create_publisher(Odometry, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10) if self.publish_status else None
        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)

        self.get_logger().info(
            "[visual_target_estimator] Started: "
            f"selected_target={self.selected_target_topic}, camera_info={self.camera_info_topic}, "
            f"out={self.out_topic}, range_signal_mode={self.range_signal_mode}"
        )

    def on_selected_target(self, msg: Detection2DArray) -> None:
        self.last_target = decode_selected_target_state_msg(msg)
        self.last_target_rx = self.get_clock().now()
        try:
            self.last_target_stamp = Time.from_msg(self.last_target.stamp)
        except Exception:
            self.last_target_stamp = self.last_target_rx

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
        self.camera_frame_id = str(msg.header.frame_id or self.camera_frame_id)

    def _dt_s(self, now: Time) -> float:
        if self.last_tick_time is None:
            return 1.0 / self.tick_hz
        dt_s = (now - self.last_tick_time).nanoseconds * 1e-9
        return max(1.0 / self.tick_hz, min(0.5, dt_s))

    def _is_fresh(self, stamp: Optional[Time], timeout_s: float, now: Time) -> bool:
        if stamp is None:
            return False
        return (now - stamp).nanoseconds * 1e-9 <= timeout_s

    def _current_target(self, now: Time) -> SelectedTargetState:
        if self.last_target.valid and self._is_fresh(self.last_target_stamp, self.target_timeout_s, now):
            return self.last_target
        return SelectedTargetState(stamp=now.to_msg(), frame_id="", valid=False)

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

    def _area_norm(self, target: SelectedTargetState) -> Optional[float]:
        if self.camera_width is None or self.camera_height is None:
            return None
        image_area = float(self.camera_width * self.camera_height)
        if image_area <= 0.0:
            return None
        area = max(0.0, float(target.bbox_size_x_px)) * max(0.0, float(target.bbox_size_y_px))
        return area / image_area

    def _projected_range_available(self, range_m: Optional[float]) -> bool:
        if range_m is None or not math.isfinite(range_m):
            return False
        return self.projected_range_min_m <= float(range_m) <= self.projected_range_max_m

    def _area_available(self, area_norm: Optional[float]) -> bool:
        return area_norm is not None and math.isfinite(area_norm) and area_norm >= self.min_area_norm_valid

    def _range_from_area(self, area_norm: Optional[float]) -> Optional[float]:
        if not self._area_available(area_norm):
            return None
        area_ratio = self.reference_area_norm / max(float(area_norm), 1e-9)
        return self.reference_range_m * math.sqrt(max(area_ratio, 1e-9))

    def _visibility_score(
        self,
        target: SelectedTargetState,
        area_norm: Optional[float],
    ) -> float:
        weighted_score = 0.0
        total_weight = 0.0

        if self.camera_width is not None and self.camera_height is not None:
            half_w = max(1.0, 0.5 * float(self.camera_width))
            half_h = max(1.0, 0.5 * float(self.camera_height))
            norm_x = abs((float(target.bbox_center_x_px) - half_w) / half_w)
            norm_y = abs((float(target.bbox_center_y_px) - half_h) / half_h)
            center_radius = math.hypot(norm_x, norm_y)
            center_score = self._clamp01(
                1.0 - (center_radius / max(self.visibility_center_soft_radius_norm, 1e-6))
            )

            left_margin = float(target.bbox_center_x_px) - 0.5 * max(0.0, float(target.bbox_size_x_px))
            right_margin = float(self.camera_width) - (
                float(target.bbox_center_x_px) + 0.5 * max(0.0, float(target.bbox_size_x_px))
            )
            top_margin = float(target.bbox_center_y_px) - 0.5 * max(0.0, float(target.bbox_size_y_px))
            bottom_margin = float(self.camera_height) - (
                float(target.bbox_center_y_px) + 0.5 * max(0.0, float(target.bbox_size_y_px))
            )
            edge_margin_norm = min(
                max(0.0, left_margin) / half_w,
                max(0.0, right_margin) / half_w,
                max(0.0, top_margin) / half_h,
                max(0.0, bottom_margin) / half_h,
            )
            edge_score = self._clamp01(
                edge_margin_norm / max(self.visibility_edge_soft_margin_norm, 1e-6)
            )

            weighted_score += self.visibility_center_weight * center_score
            total_weight += self.visibility_center_weight
            weighted_score += self.visibility_edge_weight * edge_score
            total_weight += self.visibility_edge_weight

        if self._area_available(area_norm):
            area_score = self._clamp01(
                math.sqrt(float(area_norm) / max(self.reference_area_norm, 1e-9))
            )
            weighted_score += self.visibility_area_weight * area_score
            total_weight += self.visibility_area_weight

        if total_weight <= 1e-9:
            return 1.0
        return self._clamp01(weighted_score / total_weight)

    def _select_range_source(self, projected_available: bool, area_available: bool) -> str:
        if self.range_signal_mode == "projected":
            self.active_range_source = "projected" if projected_available else "none"
            self.projected_reacquire_streak = 0
            self.projected_loss_streak = 0
            return self.active_range_source

        if self.range_signal_mode == "area":
            self.active_range_source = "area" if area_available else "none"
            self.projected_reacquire_streak = 0
            self.projected_loss_streak = 0
            return self.active_range_source

        active = self.active_range_source
        if active not in ("projected", "area"):
            active = "projected" if projected_available else ("area" if area_available else "none")
            self.active_range_source = active
            self.projected_reacquire_streak = 0
            self.projected_loss_streak = 0
            return active

        if active == "projected":
            self.projected_reacquire_streak = 0
            if projected_available:
                self.projected_loss_streak = 0
                return active
            self.projected_loss_streak += 1
            if self.projected_loss_streak >= self.auto_projected_loss_ticks:
                self.active_range_source = "area" if area_available else "none"
                self.projected_loss_streak = 0
            return self.active_range_source

        if area_available:
            if projected_available:
                self.projected_reacquire_streak += 1
                if self.projected_reacquire_streak >= self.auto_projected_reacquire_ticks:
                    self.active_range_source = "projected"
                    self.projected_reacquire_streak = 0
                    self.projected_loss_streak = 0
            else:
                self.projected_reacquire_streak = 0
            return self.active_range_source

        self.projected_reacquire_streak = 0
        self.projected_loss_streak = 0
        self.active_range_source = "projected" if projected_available else "none"
        return self.active_range_source

    def _measurement_from_target(
        self,
        target: SelectedTargetState,
    ) -> Tuple[Optional[tuple[float, float, float]], str, str, Optional[float], Optional[float], bool, bool]:
        center_u, center_v, scale_x, scale_y, norm_mode = self._image_geometry()
        if center_u is None or center_v is None or scale_x is None or scale_y is None:
            raise RuntimeError("image_geometry_missing")

        raw_x = float(target.bbox_center_x_px) - float(center_u)
        raw_y = float(target.bbox_center_y_px) - float(center_v)
        norm_x = raw_x / float(scale_x)
        norm_y = raw_y / float(scale_y)
        area_norm = self._area_norm(target)
        projected_range_m = target.projected_range_m
        projected_available = self._projected_range_available(projected_range_m)
        area_available = self._area_available(area_norm)
        active_source = self._select_range_source(projected_available, area_available)

        range_m: Optional[float]
        if active_source == "projected" and projected_available:
            range_m = float(projected_range_m)
        elif active_source == "area":
            range_m = self._range_from_area(area_norm)
        else:
            range_m = None

        if range_m is None or not math.isfinite(range_m) or range_m <= 0.0:
            return None, active_source, norm_mode, projected_range_m, area_norm, projected_available, area_available

        measurement = (
            float(norm_x * range_m),
            float(norm_y * range_m),
            float(range_m),
        )
        return measurement, active_source, norm_mode, projected_range_m, area_norm, projected_available, area_available

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        limit = max(0.0, float(limit))
        return max(-limit, min(limit, float(value)))

    @staticmethod
    def _clamp01(value: float) -> float:
        return max(0.0, min(1.0, float(value)))

    def _bump_continuity_on_measurement(self, target: SelectedTargetState) -> None:
        same_track = bool(
            target.track_id
            and self.estimate.track_id
            and str(target.track_id) == str(self.estimate.track_id)
        )
        base_gain = self.continuity_hit_gain * (0.50 + 0.50 * max(0.0, min(1.0, float(target.confidence))))
        if same_track:
            self.consecutive_hits += 1
            self.consecutive_misses = 0
            self.continuity_score = self._clamp01(self.continuity_score + base_gain)
        else:
            self.consecutive_hits = 1
            self.consecutive_misses = 0
            self.continuity_score = self._clamp01(max(0.20, 0.35 + 0.45 * float(target.confidence)))

    def _degrade_continuity_for_gap(self, dt_s: float) -> None:
        self.consecutive_hits = 0
        self.consecutive_misses += 1
        self.continuity_score = self._clamp01(
            self.continuity_score - self.continuity_miss_decay_per_s * max(0.0, dt_s)
        )

    def _tracked_quality(self, target: SelectedTargetState, visibility_score: float) -> float:
        return self._clamp01(
            0.18
            + 0.34 * float(target.confidence)
            + 0.23 * self.continuity_score
            + 0.25 * visibility_score
        )

    def _gap_quality(self, gap_s: float, *, degraded: bool) -> float:
        visibility_factor = 0.65 + 0.35 * self.last_visibility_score
        if degraded:
            horizon = max(1e-3, self.hard_lost_after_s - self.degraded_after_s)
            progress = max(0.0, min(1.0, (gap_s - self.degraded_after_s) / horizon))
            return self._clamp01(
                (1.0 - progress) * (0.18 + 0.55 * self.continuity_score) * visibility_factor
            )
        horizon = max(1e-3, self.prediction_max_gap_s)
        progress = max(0.0, min(1.0, gap_s / horizon))
        return self._clamp01(
            (1.0 - 0.60 * progress) * (0.35 + 0.55 * self.continuity_score) * visibility_factor
        )

    def _publish_status(
        self,
        *,
        state: str,
        reason: str,
        norm_mode: str,
        target: SelectedTargetState,
        measurement_source: str,
        projected_range_m: Optional[float],
        area_norm: Optional[float],
        range_available: bool,
        area_available: bool,
    ) -> None:
        if self.status_pub is None:
            return

        range_m = self.estimate.range_m if self.estimate.valid else None
        bearing_rad = self.estimate.bearing_rad if self.estimate.valid else math.nan
        elevation_rad = self.estimate.elevation_rad if self.estimate.valid else math.nan
        msg = String()
        msg.data = (
            f"state={state} "
            f"reason={reason} "
            f"norm_mode={norm_mode} "
            f"measurement_source={measurement_source} "
            f"requested_source={self.range_signal_mode} "
            f"projected_available={'true' if range_available else 'false'} "
            f"area_available={'true' if area_available else 'false'} "
            f"projected_reacquire_streak={self.projected_reacquire_streak} "
            f"projected_loss_streak={self.projected_loss_streak} "
            f"track_id={self.estimate.track_id or target.track_id or 'none'} "
            f"conf={target.confidence:.3f} "
            f"quality={self.estimate.quality:.3f} "
            f"mode={self.estimate.mode or state} "
            f"target_age_s={'na' if not math.isfinite(self.estimate.target_age_s) else f'{self.estimate.target_age_s:.2f}'} "
            f"predicted_age_s={self.estimate.predicted_age_s:.2f} "
            f"continuity={self.estimate.continuity_score:.3f} "
            f"visibility={self.last_visibility_score:.3f} "
            f"hits={self.estimate.consecutive_hits} "
            f"misses={self.estimate.consecutive_misses} "
            f"range_m={'na' if range_m is None or not math.isfinite(range_m) else f'{range_m:.2f}'} "
            f"projected_range_m={'na' if projected_range_m is None or not math.isfinite(projected_range_m) else f'{projected_range_m:.2f}'} "
            f"area_norm={'na' if area_norm is None or not math.isfinite(area_norm) else f'{area_norm:.4f}'} "
            f"bearing_rad={'na' if not math.isfinite(bearing_rad) else f'{bearing_rad:.4f}'} "
            f"elevation_rad={'na' if not math.isfinite(elevation_rad) else f'{elevation_rad:.4f}'} "
            f"rel_x_m={'na' if not self.estimate.valid else f'{self.estimate.rel_x_m:.3f}'} "
            f"rel_y_m={'na' if not self.estimate.valid else f'{self.estimate.rel_y_m:.3f}'} "
            f"rel_z_m={'na' if not self.estimate.valid else f'{self.estimate.rel_z_m:.3f}'} "
            f"vel_x_mps={'na' if not self.estimate.valid else f'{self.estimate.rel_vx_mps:.3f}'} "
            f"vel_y_mps={'na' if not self.estimate.valid else f'{self.estimate.rel_vy_mps:.3f}'} "
            f"vel_z_mps={'na' if not self.estimate.valid else f'{self.estimate.rel_vz_mps:.3f}'}"
        )
        self.status_pub.publish(msg)

    def _publish_invalid(self, now: Time) -> None:
        self.estimate = VisualTargetEstimate(
            stamp=now.to_msg(),
            frame_id=self.camera_frame_id,
            valid=False,
            mode="LOST",
            quality=0.0,
            target_age_s=math.nan,
            predicted_age_s=0.0,
            continuity_score=self.continuity_score,
            consecutive_hits=self.consecutive_hits,
            consecutive_misses=self.consecutive_misses,
        )
        self.out_pub.publish(encode_visual_target_estimate_msg(self.estimate))

    def on_tick(self) -> None:
        now = self.get_clock().now()
        dt_s = self._dt_s(now)
        target = self._current_target(now)
        state = "LOST"
        reason = "target_missing"
        norm_mode = "missing"
        measurement_source = self.active_range_source
        projected_range_m = None
        area_norm = None
        range_available = False
        area_available = False

        measurement: Optional[tuple[float, float, float]] = None
        if target.valid:
            try:
                (
                    measurement,
                    measurement_source,
                    norm_mode,
                    projected_range_m,
                    area_norm,
                    range_available,
                    area_available,
                ) = self._measurement_from_target(target)
                if measurement is None:
                    state = "DEGRADED"
                    reason = "range_unavailable"
            except RuntimeError:
                state = "WAIT_CAMERA"
                reason = "image_geometry_missing"

        if measurement is not None:
            mx, my, mz = measurement
            self._bump_continuity_on_measurement(target)
            visibility_score = self._visibility_score(target, area_norm)
            self.last_visibility_score = visibility_score
            if (
                not self.estimate.valid
                or self.last_estimate_update is None
                or (
                    target.track_id
                    and self.estimate.track_id
                    and str(target.track_id) != str(self.estimate.track_id)
                )
            ):
                self.estimate = VisualTargetEstimate(
                    stamp=now.to_msg(),
                    frame_id=self.camera_frame_id or target.frame_id,
                    valid=True,
                    track_id=str(target.track_id or ""),
                    rel_x_m=float(mx),
                    rel_y_m=float(my),
                    rel_z_m=float(mz),
                    rel_vx_mps=0.0,
                    rel_vy_mps=0.0,
                    rel_vz_mps=0.0,
                    mode="TRACKED",
                    quality=self._tracked_quality(target, visibility_score),
                    target_age_s=0.0,
                    predicted_age_s=0.0,
                    continuity_score=self.continuity_score,
                    consecutive_hits=self.consecutive_hits,
                    consecutive_misses=self.consecutive_misses,
                )
                state = "TRACKED"
                reason = "seed"
            else:
                pred_x = self.estimate.rel_x_m + self.estimate.rel_vx_mps * dt_s
                pred_y = self.estimate.rel_y_m + self.estimate.rel_vy_mps * dt_s
                pred_z = self.estimate.rel_z_m + self.estimate.rel_vz_mps * dt_s

                alpha = self.position_alpha_projected if measurement_source == "projected" else self.position_alpha_area
                beta = self.velocity_beta_projected if measurement_source == "projected" else self.velocity_beta_area

                err_x = float(mx) - float(pred_x)
                err_y = float(my) - float(pred_y)
                err_z = float(mz) - float(pred_z)

                next_x = float(pred_x + alpha * err_x)
                next_y = float(pred_y + alpha * err_y)
                next_z = max(1e-3, float(pred_z + alpha * err_z))

                next_vx = float(self.estimate.rel_vx_mps + beta * (err_x / max(dt_s, 1e-3)))
                next_vy = float(self.estimate.rel_vy_mps + beta * (err_y / max(dt_s, 1e-3)))
                next_vz = float(self.estimate.rel_vz_mps + beta * (err_z / max(dt_s, 1e-3)))
                next_vx = self._clamp(next_vx, self.max_velocity_mps)
                next_vy = self._clamp(next_vy, self.max_velocity_mps)
                next_vz = self._clamp(next_vz, self.max_velocity_mps)

                self.estimate = VisualTargetEstimate(
                    stamp=now.to_msg(),
                    frame_id=self.camera_frame_id or target.frame_id or self.estimate.frame_id,
                    valid=True,
                    track_id=str(target.track_id or self.estimate.track_id),
                    rel_x_m=next_x,
                    rel_y_m=next_y,
                    rel_z_m=next_z,
                    rel_vx_mps=next_vx,
                    rel_vy_mps=next_vy,
                    rel_vz_mps=next_vz,
                    mode="TRACKED",
                    quality=self._tracked_quality(target, visibility_score),
                    target_age_s=0.0,
                    predicted_age_s=0.0,
                    continuity_score=self.continuity_score,
                    consecutive_hits=self.consecutive_hits,
                    consecutive_misses=self.consecutive_misses,
                )
                state = "TRACKED"
                reason = "update"

            self.last_estimate_update = now
            self.last_measurement_time = now
            self.last_status_state = state
            self.out_pub.publish(encode_visual_target_estimate_msg(self.estimate))
            self._publish_status(
                state=state,
                reason=reason,
                norm_mode=norm_mode,
                target=target,
                measurement_source=measurement_source,
                projected_range_m=projected_range_m,
                area_norm=area_norm,
                range_available=range_available,
                area_available=area_available,
            )
        elif (
            self.estimate.valid
            and self.last_measurement_time is not None
            and self.hard_lost_after_s > 0.0
        ):
            gap_s = max(0.0, (now - self.last_measurement_time).nanoseconds * 1e-9)
            if gap_s <= self.hard_lost_after_s:
                self._degrade_continuity_for_gap(dt_s)
                degraded = gap_s > self.prediction_max_gap_s
                vel_decay = self.degraded_velocity_decay if degraded else self.predicted_velocity_decay
                next_vx = float(self.estimate.rel_vx_mps * vel_decay)
                next_vy = float(self.estimate.rel_vy_mps * vel_decay)
                next_vz = float(self.estimate.rel_vz_mps * vel_decay)
                next_x = float(self.estimate.rel_x_m + next_vx * dt_s)
                next_y = float(self.estimate.rel_y_m + next_vy * dt_s)
                next_z = max(1e-3, float(self.estimate.rel_z_m + next_vz * dt_s))
                state = "DEGRADED" if degraded else "PREDICTED"
                reason = "measurement_gap_degraded" if degraded else "measurement_gap"
                self.estimate = VisualTargetEstimate(
                    stamp=now.to_msg(),
                    frame_id=self.estimate.frame_id or self.camera_frame_id,
                    valid=True,
                    track_id=self.estimate.track_id,
                    rel_x_m=next_x,
                    rel_y_m=next_y,
                    rel_z_m=next_z,
                    rel_vx_mps=next_vx,
                    rel_vy_mps=next_vy,
                    rel_vz_mps=next_vz,
                    mode=state,
                    quality=self._gap_quality(gap_s, degraded=degraded),
                    target_age_s=gap_s,
                    predicted_age_s=gap_s,
                    continuity_score=self.continuity_score,
                    consecutive_hits=self.consecutive_hits,
                    consecutive_misses=self.consecutive_misses,
                )
                self.last_estimate_update = now
                self.last_status_state = state
                self.out_pub.publish(encode_visual_target_estimate_msg(self.estimate))
                self._publish_status(
                    state=state,
                    reason=reason,
                    norm_mode=norm_mode,
                    target=target,
                    measurement_source=self.active_range_source,
                    projected_range_m=projected_range_m,
                    area_norm=area_norm,
                    range_available=range_available,
                    area_available=area_available,
                )
            else:
                self._degrade_continuity_for_gap(dt_s)
                self._publish_invalid(now)
                self.last_status_state = state
                self._publish_status(
                    state=state,
                    reason=reason,
                    norm_mode=norm_mode,
                    target=target,
                    measurement_source=self.active_range_source,
                    projected_range_m=projected_range_m,
                    area_norm=area_norm,
                    range_available=range_available,
                    area_available=area_available,
                )
        else:
            if target.valid:
                self._degrade_continuity_for_gap(dt_s)
            self._publish_invalid(now)
            self.last_status_state = state
            self._publish_status(
                state=state,
                reason=reason,
                norm_mode=norm_mode,
                target=target,
                measurement_source=self.active_range_source,
                projected_range_m=projected_range_m,
                area_norm=area_norm,
                range_available=range_available,
                area_available=area_available,
            )

        self.last_tick_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualTargetEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
