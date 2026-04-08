#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray

from lrs_halmstad.common.selected_target_state import (
    SelectedTargetState,
    decode_selected_target_state_msg,
    encode_selected_target_state_msg,
)
from lrs_halmstad.follow.follow_math import coerce_bool, wrap_pi


@dataclass
class FilterDecision:
    output_state: SelectedTargetState
    output_mode: str
    confidence_bucket: str
    accepted: bool
    reject_reason: str
    same_track: bool
    center_jump_px: Optional[float]
    area_ratio: Optional[float]
    range_jump_m: Optional[float]
    lost_age_s: float
    stable_age_s: float


class SelectedTargetFilter(Node):
    """ByteTrack-inspired trust/smoothing layer between selected target and controller."""

    def __init__(self) -> None:
        super().__init__("selected_target_filter")

        self.declare_parameter("in_topic", "/coord/leader_selected_target")
        self.declare_parameter("out_topic", "/coord/leader_selected_target_filtered")
        self.declare_parameter("status_topic", "/coord/leader_selected_target_filtered_status")
        self.declare_parameter("publish_status", True)
        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("target_timeout_s", 1.0)
        self.declare_parameter("hold_timeout_s", 0.45)
        self.declare_parameter("prediction_enable", True)
        self.declare_parameter("prediction_max_gap_s", 0.20)
        self.declare_parameter("max_predict_center_shift_px", 120.0)
        self.declare_parameter("strong_confidence_threshold", 0.65)
        self.declare_parameter("weak_confidence_threshold", 0.35)
        self.declare_parameter("min_confidence_threshold", 0.05)
        self.declare_parameter("track_switch_min_confidence", 0.60)
        self.declare_parameter("track_switch_min_age_s", 0.20)
        self.declare_parameter("same_track_center_jump_px_max", 140.0)
        self.declare_parameter("track_switch_center_jump_px_max", 260.0)
        self.declare_parameter("same_track_area_ratio_max", 2.5)
        self.declare_parameter("track_switch_area_ratio_max", 3.5)
        self.declare_parameter("range_jump_max_m", 4.0)
        self.declare_parameter("hold_last_range_on_missing", True)
        self.declare_parameter("low_conf_reacquire_enable", True)
        self.declare_parameter("low_conf_reacquire_timeout_s", 0.90)
        self.declare_parameter("low_conf_reacquire_center_jump_px_max", 240.0)
        self.declare_parameter("low_conf_reacquire_area_ratio_max", 4.5)
        self.declare_parameter("low_conf_reacquire_range_jump_max_m", 6.0)
        self.declare_parameter("center_alpha_strong", 0.65)
        self.declare_parameter("center_alpha_weak", 0.35)
        self.declare_parameter("size_alpha_strong", 0.55)
        self.declare_parameter("size_alpha_weak", 0.30)
        self.declare_parameter("range_alpha_strong", 0.60)
        self.declare_parameter("range_alpha_weak", 0.30)

        self.in_topic = str(self.get_parameter("in_topic").value).strip()
        self.out_topic = str(self.get_parameter("out_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.publish_status = coerce_bool(self.get_parameter("publish_status").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.target_timeout_s = float(self.get_parameter("target_timeout_s").value)
        self.hold_timeout_s = float(self.get_parameter("hold_timeout_s").value)
        self.prediction_enable = coerce_bool(self.get_parameter("prediction_enable").value)
        self.prediction_max_gap_s = float(self.get_parameter("prediction_max_gap_s").value)
        self.max_predict_center_shift_px = float(self.get_parameter("max_predict_center_shift_px").value)
        self.strong_confidence_threshold = float(self.get_parameter("strong_confidence_threshold").value)
        self.weak_confidence_threshold = float(self.get_parameter("weak_confidence_threshold").value)
        self.min_confidence_threshold = float(self.get_parameter("min_confidence_threshold").value)
        self.track_switch_min_confidence = float(self.get_parameter("track_switch_min_confidence").value)
        self.track_switch_min_age_s = float(self.get_parameter("track_switch_min_age_s").value)
        self.same_track_center_jump_px_max = float(self.get_parameter("same_track_center_jump_px_max").value)
        self.track_switch_center_jump_px_max = float(self.get_parameter("track_switch_center_jump_px_max").value)
        self.same_track_area_ratio_max = float(self.get_parameter("same_track_area_ratio_max").value)
        self.track_switch_area_ratio_max = float(self.get_parameter("track_switch_area_ratio_max").value)
        self.range_jump_max_m = float(self.get_parameter("range_jump_max_m").value)
        self.hold_last_range_on_missing = coerce_bool(self.get_parameter("hold_last_range_on_missing").value)
        self.low_conf_reacquire_enable = coerce_bool(
            self.get_parameter("low_conf_reacquire_enable").value
        )
        self.low_conf_reacquire_timeout_s = float(
            self.get_parameter("low_conf_reacquire_timeout_s").value
        )
        self.low_conf_reacquire_center_jump_px_max = float(
            self.get_parameter("low_conf_reacquire_center_jump_px_max").value
        )
        self.low_conf_reacquire_area_ratio_max = float(
            self.get_parameter("low_conf_reacquire_area_ratio_max").value
        )
        self.low_conf_reacquire_range_jump_max_m = float(
            self.get_parameter("low_conf_reacquire_range_jump_max_m").value
        )
        self.center_alpha_strong = float(self.get_parameter("center_alpha_strong").value)
        self.center_alpha_weak = float(self.get_parameter("center_alpha_weak").value)
        self.size_alpha_strong = float(self.get_parameter("size_alpha_strong").value)
        self.size_alpha_weak = float(self.get_parameter("size_alpha_weak").value)
        self.range_alpha_strong = float(self.get_parameter("range_alpha_strong").value)
        self.range_alpha_weak = float(self.get_parameter("range_alpha_weak").value)

        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.target_timeout_s <= 0.0:
            raise ValueError("target_timeout_s must be > 0")
        if self.hold_timeout_s < 0.0:
            raise ValueError("hold_timeout_s must be >= 0")
        if self.prediction_max_gap_s < 0.0:
            raise ValueError("prediction_max_gap_s must be >= 0")
        if self.low_conf_reacquire_timeout_s < 0.0:
            raise ValueError("low_conf_reacquire_timeout_s must be >= 0")
        if not (0.0 <= self.min_confidence_threshold <= self.weak_confidence_threshold <= self.strong_confidence_threshold <= 1.0):
            raise ValueError("confidence thresholds must satisfy min <= weak <= strong and all be within [0, 1]")
        if self.track_switch_min_confidence < 0.0 or self.track_switch_min_confidence > 1.0:
            raise ValueError("track_switch_min_confidence must be within [0, 1]")

        now_msg = self.get_clock().now().to_msg()
        self.last_raw_target = SelectedTargetState(stamp=now_msg, frame_id="", valid=False)
        self.last_raw_stamp: Optional[Time] = None
        self.filtered_target = SelectedTargetState(stamp=now_msg, frame_id="", valid=False)
        self.last_output = self.filtered_target
        self.last_accept_target: Optional[SelectedTargetState] = None
        self.last_accept_time: Optional[Time] = None
        self.prev_accept_target: Optional[SelectedTargetState] = None
        self.prev_accept_time: Optional[Time] = None
        self.track_stable_since: Optional[Time] = None
        self.current_track_id: str = ""
        self.center_vx_px_s = 0.0
        self.center_vy_px_s = 0.0
        self.continuity_score = 0.0
        self.consecutive_hits = 0
        self.consecutive_misses = 0
        self.audit_ticks_total = 0
        self.audit_accept_total = 0
        self.audit_predicted_total = 0
        self.audit_held_total = 0
        self.audit_invalid_total = 0
        self.last_decision = FilterDecision(
            output_state=self.filtered_target,
            output_mode="invalid",
            confidence_bucket="none",
            accepted=False,
            reject_reason="none",
            same_track=False,
            center_jump_px=None,
            area_ratio=None,
            range_jump_m=None,
            lost_age_s=0.0,
            stable_age_s=0.0,
        )

        self.create_subscription(Detection2DArray, self.in_topic, self.on_raw_target, 1)
        self.out_pub = self.create_publisher(Detection2DArray, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10) if self.publish_status else None
        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)

        self.get_logger().info(
            "[selected_target_filter] Started: "
            f"in={self.in_topic}, out={self.out_topic}, status={self.status_topic}, "
            f"hold_timeout_s={self.hold_timeout_s:.2f}, prediction_enable={self.prediction_enable}"
        )

    def on_raw_target(self, msg: Detection2DArray) -> None:
        self.last_raw_target = decode_selected_target_state_msg(msg)
        try:
            self.last_raw_stamp = Time.from_msg(self.last_raw_target.stamp)
        except Exception:
            self.last_raw_stamp = self.get_clock().now()

    def _is_fresh(self, stamp: Optional[Time], timeout_s: float, now: Time) -> bool:
        if stamp is None:
            return False
        return (now - stamp).nanoseconds * 1e-9 <= timeout_s

    def _raw_target(self, now: Time) -> SelectedTargetState:
        if self.last_raw_target.valid and self._is_fresh(self.last_raw_stamp, self.target_timeout_s, now):
            return self.last_raw_target
        return SelectedTargetState(stamp=now.to_msg(), frame_id="", valid=False)

    @staticmethod
    def _area_px(state: SelectedTargetState) -> float:
        return max(0.0, float(state.bbox_size_x_px)) * max(0.0, float(state.bbox_size_y_px))

    def _area_ratio(self, raw: SelectedTargetState, ref: SelectedTargetState) -> Optional[float]:
        raw_area = self._area_px(raw)
        ref_area = self._area_px(ref)
        if raw_area <= 1e-6 or ref_area <= 1e-6:
            return None
        return max(raw_area, ref_area) / min(raw_area, ref_area)

    def _center_jump_px(self, raw: SelectedTargetState, ref: SelectedTargetState) -> Optional[float]:
        if not raw.valid or not ref.valid:
            return None
        return math.hypot(
            float(raw.bbox_center_x_px) - float(ref.bbox_center_x_px),
            float(raw.bbox_center_y_px) - float(ref.bbox_center_y_px),
        )

    def _range_jump_m(self, raw: SelectedTargetState, ref: SelectedTargetState) -> Optional[float]:
        if raw.projected_range_m is None or ref.projected_range_m is None:
            return None
        return abs(float(raw.projected_range_m) - float(ref.projected_range_m))

    def _confidence_bucket(self, target: SelectedTargetState) -> str:
        conf = float(target.confidence)
        if conf >= self.strong_confidence_threshold:
            return "strong"
        if conf >= self.weak_confidence_threshold:
            return "weak"
        if conf >= self.min_confidence_threshold:
            return "low"
        return "reject"

    @staticmethod
    def _same_track(raw: SelectedTargetState, ref: SelectedTargetState) -> bool:
        if not raw.valid or not ref.valid:
            return False
        if not raw.track_id or not ref.track_id:
            return False
        return str(raw.track_id) == str(ref.track_id)

    def _track_stable_age_s(self, now: Time, track_id: str) -> float:
        if not track_id or self.track_stable_since is None or track_id != self.current_track_id:
            return 0.0
        return max(0.0, (now - self.track_stable_since).nanoseconds * 1e-9)

    @staticmethod
    def _smooth(prev: float, new: float, alpha: float) -> float:
        alpha = max(0.0, min(1.0, float(alpha)))
        return (1.0 - alpha) * float(prev) + alpha * float(new)

    @staticmethod
    def _smooth_angle(prev: float, new: float, alpha: float) -> float:
        alpha = max(0.0, min(1.0, float(alpha)))
        return wrap_pi(float(prev) + alpha * wrap_pi(float(new) - float(prev)))

    def _copy_state(self, state: SelectedTargetState, *, stamp=None, valid=None) -> SelectedTargetState:
        return replace(
            state,
            stamp=state.stamp if stamp is None else stamp,
            valid=state.valid if valid is None else valid,
        )

    def _accept_track_switch(self, raw: SelectedTargetState, center_jump_px: Optional[float], area_ratio: Optional[float], range_jump_m: Optional[float]) -> tuple[bool, str]:
        if float(raw.confidence) < self.track_switch_min_confidence:
            return False, "switch_conf_low"
        if raw.track_age_s is not None and float(raw.track_age_s) < self.track_switch_min_age_s:
            return False, "switch_track_age_low"
        if center_jump_px is not None and center_jump_px > self.track_switch_center_jump_px_max:
            return False, "switch_center_jump"
        if area_ratio is not None and area_ratio > self.track_switch_area_ratio_max:
            return False, "switch_area_jump"
        if range_jump_m is not None and range_jump_m > self.range_jump_max_m:
            return False, "switch_range_jump"
        return True, "none"

    def _smooth_target(
        self,
        raw: SelectedTargetState,
        same_track: bool,
        confidence_bucket: str,
        reference: Optional[SelectedTargetState] = None,
    ) -> SelectedTargetState:
        reference = self.filtered_target if reference is None else reference
        if not reference.valid or not same_track:
            return self._copy_state(raw)

        center_alpha = self.center_alpha_strong if confidence_bucket == "strong" else self.center_alpha_weak
        size_alpha = self.size_alpha_strong if confidence_bucket == "strong" else self.size_alpha_weak
        range_alpha = self.range_alpha_strong if confidence_bucket == "strong" else self.range_alpha_weak

        projected_range = raw.projected_range_m
        if projected_range is None and self.hold_last_range_on_missing:
            projected_range = reference.projected_range_m
        elif projected_range is not None and reference.projected_range_m is not None:
            projected_range = self._smooth(reference.projected_range_m, projected_range, range_alpha)

        return SelectedTargetState(
            stamp=raw.stamp,
            frame_id=raw.frame_id or reference.frame_id,
            valid=True,
            bbox_center_x_px=self._smooth(reference.bbox_center_x_px, raw.bbox_center_x_px, center_alpha),
            bbox_center_y_px=self._smooth(reference.bbox_center_y_px, raw.bbox_center_y_px, center_alpha),
            bbox_size_x_px=max(0.0, self._smooth(reference.bbox_size_x_px, raw.bbox_size_x_px, size_alpha)),
            bbox_size_y_px=max(0.0, self._smooth(reference.bbox_size_y_px, raw.bbox_size_y_px, size_alpha)),
            bbox_theta_rad=self._smooth_angle(reference.bbox_theta_rad, raw.bbox_theta_rad, size_alpha),
            confidence=float(raw.confidence),
            class_id=raw.class_id or reference.class_id,
            track_id=raw.track_id or reference.track_id,
            projected_range_m=projected_range,
            track_age_s=raw.track_age_s if raw.track_age_s is not None else reference.track_age_s,
        )

    def _recent_accept_reference(self, now: Time) -> Optional[SelectedTargetState]:
        if (
            not self.low_conf_reacquire_enable
            or self.last_accept_target is None
            or self.last_accept_time is None
            or not self.last_accept_target.valid
        ):
            return None
        age_s = max(0.0, (now - self.last_accept_time).nanoseconds * 1e-9)
        if age_s > self.low_conf_reacquire_timeout_s:
            return None
        return self.last_accept_target

    def _can_low_conf_reacquire(
        self,
        raw: SelectedTargetState,
        reference: Optional[SelectedTargetState],
    ) -> tuple[bool, str, Optional[float], Optional[float], Optional[float], bool]:
        if reference is None or not reference.valid:
            return False, "bootstrap_conf_low", None, None, None, False

        if raw.track_id and reference.track_id and str(raw.track_id) != str(reference.track_id):
            return False, "low_conf_track_mismatch", None, None, None, False

        center_jump_px = self._center_jump_px(raw, reference)
        area_ratio = self._area_ratio(raw, reference)
        range_jump_m = self._range_jump_m(raw, reference)
        if center_jump_px is not None and center_jump_px > self.low_conf_reacquire_center_jump_px_max:
            return False, "low_conf_center_jump", center_jump_px, area_ratio, range_jump_m, False
        if area_ratio is not None and area_ratio > self.low_conf_reacquire_area_ratio_max:
            return False, "low_conf_area_jump", center_jump_px, area_ratio, range_jump_m, False
        if range_jump_m is not None and range_jump_m > self.low_conf_reacquire_range_jump_max_m:
            return False, "low_conf_range_jump", center_jump_px, area_ratio, range_jump_m, False
        return True, "none", center_jump_px, area_ratio, range_jump_m, self._same_track(raw, reference)

    def _update_track_velocity(self, now: Time, accepted: SelectedTargetState) -> None:
        if (
            self.last_accept_target is None
            or self.last_accept_time is None
            or not accepted.valid
            or not self.last_accept_target.valid
        ):
            self.center_vx_px_s = 0.0
            self.center_vy_px_s = 0.0
            return
        if not self._same_track(accepted, self.last_accept_target):
            self.center_vx_px_s = 0.0
            self.center_vy_px_s = 0.0
            return
        dt_s = max(1e-3, (now - self.last_accept_time).nanoseconds * 1e-9)
        self.center_vx_px_s = (
            float(accepted.bbox_center_x_px) - float(self.last_accept_target.bbox_center_x_px)
        ) / dt_s
        self.center_vy_px_s = (
            float(accepted.bbox_center_y_px) - float(self.last_accept_target.bbox_center_y_px)
        ) / dt_s

    def _predict_target(self, now: Time) -> SelectedTargetState:
        if not self.filtered_target.valid or self.last_accept_time is None:
            return self._copy_state(self.filtered_target, stamp=now.to_msg())
        gap_s = max(0.0, (now - self.last_accept_time).nanoseconds * 1e-9)
        if not self.prediction_enable or gap_s <= 0.0 or gap_s > self.prediction_max_gap_s:
            return self._copy_state(self.filtered_target, stamp=now.to_msg())

        shift_x = self.center_vx_px_s * gap_s
        shift_y = self.center_vy_px_s * gap_s
        shift_mag = math.hypot(shift_x, shift_y)
        if shift_mag > self.max_predict_center_shift_px and shift_mag > 1e-6:
            scale = self.max_predict_center_shift_px / shift_mag
            shift_x *= scale
            shift_y *= scale
        return SelectedTargetState(
            stamp=now.to_msg(),
            frame_id=self.filtered_target.frame_id,
            valid=True,
            bbox_center_x_px=float(self.filtered_target.bbox_center_x_px) + float(shift_x),
            bbox_center_y_px=float(self.filtered_target.bbox_center_y_px) + float(shift_y),
            bbox_size_x_px=float(self.filtered_target.bbox_size_x_px),
            bbox_size_y_px=float(self.filtered_target.bbox_size_y_px),
            bbox_theta_rad=float(self.filtered_target.bbox_theta_rad),
            confidence=float(self.filtered_target.confidence),
            class_id=self.filtered_target.class_id,
            track_id=self.filtered_target.track_id,
            projected_range_m=self.filtered_target.projected_range_m,
            track_age_s=self.filtered_target.track_age_s,
        )

    def _invalid_state(self, now: Time) -> SelectedTargetState:
        return SelectedTargetState(stamp=now.to_msg(), frame_id="", valid=False)

    def _handle_gap(self, now: Time, confidence_bucket: str, reject_reason: str) -> FilterDecision:
        if self.last_accept_time is None or not self.filtered_target.valid:
            invalid = self._invalid_state(now)
            return FilterDecision(
                output_state=invalid,
                output_mode="invalid",
                confidence_bucket=confidence_bucket,
                accepted=False,
                reject_reason=reject_reason,
                same_track=False,
                center_jump_px=None,
                area_ratio=None,
                range_jump_m=None,
                lost_age_s=0.0,
                stable_age_s=0.0,
            )

        lost_age_s = max(0.0, (now - self.last_accept_time).nanoseconds * 1e-9)
        stable_age_s = self._track_stable_age_s(now, self.filtered_target.track_id)
        if lost_age_s > self.hold_timeout_s:
            invalid = self._invalid_state(now)
            return FilterDecision(
                output_state=invalid,
                output_mode="invalid",
                confidence_bucket=confidence_bucket,
                accepted=False,
                reject_reason=reject_reason,
                same_track=False,
                center_jump_px=None,
                area_ratio=None,
                range_jump_m=None,
                lost_age_s=lost_age_s,
                stable_age_s=stable_age_s,
            )

        predicted = self._predict_target(now)
        output_mode = (
            "predicted"
            if self.prediction_enable and lost_age_s <= self.prediction_max_gap_s and predicted.valid
            else "held"
        )
        return FilterDecision(
            output_state=predicted if predicted.valid else self._copy_state(self.filtered_target, stamp=now.to_msg()),
            output_mode=output_mode,
            confidence_bucket=confidence_bucket,
            accepted=False,
            reject_reason=reject_reason,
            same_track=False,
            center_jump_px=None,
            area_ratio=None,
            range_jump_m=None,
            lost_age_s=lost_age_s,
            stable_age_s=stable_age_s,
        )

    def _process_raw_target(self, raw: SelectedTargetState, now: Time) -> FilterDecision:
        if not raw.valid:
            return self._handle_gap(now, "none", "raw_invalid")

        confidence_bucket = self._confidence_bucket(raw)
        if confidence_bucket == "reject":
            return self._handle_gap(now, confidence_bucket, "confidence_too_low")

        if not self.filtered_target.valid:
            if confidence_bucket in ("strong", "weak"):
                accepted = self._copy_state(raw)
                stable_age_s = float(raw.track_age_s) if raw.track_age_s is not None else 0.0
                return FilterDecision(
                    output_state=accepted,
                    output_mode="raw" if confidence_bucket == "strong" else "weak",
                    confidence_bucket=confidence_bucket,
                    accepted=True,
                    reject_reason="none",
                    same_track=False,
                    center_jump_px=None,
                    area_ratio=None,
                    range_jump_m=None,
                    lost_age_s=0.0,
                    stable_age_s=stable_age_s,
                )
            if confidence_bucket == "low":
                reference = self._recent_accept_reference(now)
                (
                    can_reacquire,
                    reacquire_reason,
                    center_jump_px,
                    area_ratio,
                    range_jump_m,
                    same_track,
                ) = self._can_low_conf_reacquire(raw, reference)
                if can_reacquire and reference is not None:
                    accepted = self._smooth_target(
                        raw,
                        same_track=same_track,
                        confidence_bucket="weak",
                        reference=reference,
                    )
                    stable_age_s = self._track_stable_age_s(now, accepted.track_id)
                    return FilterDecision(
                        output_state=accepted,
                        output_mode="reacquired",
                        confidence_bucket=confidence_bucket,
                        accepted=True,
                        reject_reason="none",
                        same_track=same_track,
                        center_jump_px=center_jump_px,
                        area_ratio=area_ratio,
                        range_jump_m=range_jump_m,
                        lost_age_s=0.0,
                        stable_age_s=stable_age_s,
                    )
                return self._handle_gap(now, confidence_bucket, reacquire_reason)
            return self._handle_gap(now, confidence_bucket, "bootstrap_conf_low")

        same_track = self._same_track(raw, self.filtered_target)
        center_jump_px = self._center_jump_px(raw, self.filtered_target)
        area_ratio = self._area_ratio(raw, self.filtered_target)
        range_jump_m = self._range_jump_m(raw, self.filtered_target)

        if same_track:
            if center_jump_px is not None and center_jump_px > self.same_track_center_jump_px_max:
                return self._handle_gap(now, confidence_bucket, "same_track_center_jump")
            if area_ratio is not None and area_ratio > self.same_track_area_ratio_max:
                return self._handle_gap(now, confidence_bucket, "same_track_area_jump")
            if range_jump_m is not None and range_jump_m > self.range_jump_max_m:
                return self._handle_gap(now, confidence_bucket, "same_track_range_jump")
            if confidence_bucket in ("strong", "weak", "low"):
                accepted = self._smooth_target(raw, same_track=True, confidence_bucket=confidence_bucket)
                stable_age_s = self._track_stable_age_s(now, accepted.track_id)
                return FilterDecision(
                    output_state=accepted,
                    output_mode="filtered" if confidence_bucket == "strong" else "weak",
                    confidence_bucket=confidence_bucket,
                    accepted=True,
                    reject_reason="none",
                    same_track=True,
                    center_jump_px=center_jump_px,
                    area_ratio=area_ratio,
                    range_jump_m=range_jump_m,
                    lost_age_s=0.0,
                    stable_age_s=stable_age_s,
                )
            return self._handle_gap(now, confidence_bucket, "same_track_conf_reject")

        can_switch, switch_reason = self._accept_track_switch(raw, center_jump_px, area_ratio, range_jump_m)
        if can_switch:
            accepted = self._copy_state(raw)
            stable_age_s = float(raw.track_age_s) if raw.track_age_s is not None else 0.0
            return FilterDecision(
                output_state=accepted,
                output_mode="raw",
                confidence_bucket=confidence_bucket,
                accepted=True,
                reject_reason="none",
                same_track=False,
                center_jump_px=center_jump_px,
                area_ratio=area_ratio,
                range_jump_m=range_jump_m,
                lost_age_s=0.0,
                stable_age_s=stable_age_s,
            )
        return self._handle_gap(now, confidence_bucket, switch_reason)

    def _publish_status(self, decision: FilterDecision, raw: SelectedTargetState, now: Time) -> None:
        if self.status_pub is None:
            return
        msg = String()
        msg.data = (
            f"state={'VALID' if decision.output_state.valid else 'INVALID'} "
            f"output_mode={decision.output_mode} "
            f"confidence_bucket={decision.confidence_bucket} "
            f"accepted={'true' if decision.accepted else 'false'} "
            f"reject_reason={decision.reject_reason} "
            f"raw_valid={'true' if raw.valid else 'false'} "
            f"same_track={'true' if decision.same_track else 'false'} "
            f"track_id={decision.output_state.track_id or 'none'} "
            f"conf={decision.output_state.confidence:.3f} "
            f"track_age_s={'na' if decision.output_state.track_age_s is None else f'{decision.output_state.track_age_s:.2f}'} "
            f"stable_age_s={decision.stable_age_s:.2f} "
            f"lost_age_s={decision.lost_age_s:.2f} "
            f"continuity={self.continuity_score:.3f} "
            f"hits={self.consecutive_hits} "
            f"misses={self.consecutive_misses} "
            f"audit_ticks={self.audit_ticks_total} "
            f"audit_accept={self.audit_accept_total} "
            f"audit_predicted={self.audit_predicted_total} "
            f"audit_held={self.audit_held_total} "
            f"audit_invalid={self.audit_invalid_total} "
            f"center_jump_px={'na' if decision.center_jump_px is None else f'{decision.center_jump_px:.1f}'} "
            f"area_ratio={'na' if decision.area_ratio is None else f'{decision.area_ratio:.2f}'} "
            f"range_jump_m={'na' if decision.range_jump_m is None else f'{decision.range_jump_m:.2f}'} "
            f"stamp_age_ms={0.0 if self.last_raw_stamp is None else max(0.0, (now - self.last_raw_stamp).nanoseconds * 1e-6):.1f}"
        )
        self.status_pub.publish(msg)

    def _commit_decision(self, decision: FilterDecision, now: Time) -> None:
        self.audit_ticks_total += 1
        self.last_output = decision.output_state
        self.last_decision = decision
        if decision.accepted and decision.output_state.valid:
            self.audit_accept_total += 1
            self.consecutive_hits += 1
            self.consecutive_misses = 0
            continuity_gain = 0.18 * (0.50 + 0.50 * max(0.0, min(1.0, float(decision.output_state.confidence))))
            self.continuity_score = min(1.0, self.continuity_score + continuity_gain)
            self.filtered_target = decision.output_state
            self._update_track_velocity(now, decision.output_state)
            self.prev_accept_target = self.last_accept_target
            self.prev_accept_time = self.last_accept_time
            self.last_accept_target = decision.output_state
            self.last_accept_time = now
            track_id = decision.output_state.track_id or ""
            if track_id != self.current_track_id or self.track_stable_since is None:
                self.current_track_id = track_id
                self.track_stable_since = now
        elif not decision.output_state.valid and decision.output_mode == "invalid":
            self.audit_invalid_total += 1
            self.consecutive_hits = 0
            self.consecutive_misses += 1
            self.continuity_score = max(0.0, self.continuity_score - 0.35)
            self.filtered_target = decision.output_state
            self.current_track_id = ""
            self.track_stable_since = None
            self.center_vx_px_s = 0.0
            self.center_vy_px_s = 0.0
        else:
            if decision.output_mode == "predicted":
                self.audit_predicted_total += 1
            elif decision.output_mode == "held":
                self.audit_held_total += 1
            self.consecutive_hits = 0
            self.consecutive_misses += 1
            self.continuity_score = max(0.0, self.continuity_score - 0.12)

    def on_tick(self) -> None:
        now = self.get_clock().now()
        raw = self._raw_target(now)
        decision = self._process_raw_target(raw, now)
        if decision.output_state.valid and decision.output_state.stamp.sec == 0 and decision.output_state.stamp.nanosec == 0:
            decision = replace(decision, output_state=self._copy_state(decision.output_state, stamp=now.to_msg()))
        self._commit_decision(decision, now)
        self.out_pub.publish(encode_selected_target_state_msg(decision.output_state))
        self._publish_status(decision, raw, now)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SelectedTargetFilter()
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
