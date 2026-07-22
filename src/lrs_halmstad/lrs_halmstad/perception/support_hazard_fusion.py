"""Deterministic conservative multi-source hazard fusion at dji0."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
from functools import partial
import math
from typing import Optional

from builtin_interfaces.msg import Duration, Time
from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header


MAP_FRAME = "map"
DEFAULT_DJI1_TOPIC = "/coord/support/dji1/aerial_hazards"
DEFAULT_DJI2_TOPIC = "/coord/support/dji2/aerial_hazards"
DEFAULT_OUTPUT_TOPIC = "/coord/dji0/aerial_hazards"
DEFAULT_DJI2_ENABLE = False
DEFAULT_STALE_TIMEOUT_S = 0.75
DEFAULT_MAX_SOURCE_AGE_S = 0.75
DEFAULT_SOURCE_TIMEOUT_S = 0.75
DEFAULT_MAX_COVARIANCE = 4.0
DEFAULT_PUBLISH_RATE_HZ = 10.0
DEFAULT_DIAGNOSTIC_PERIOD_S = 5.0
DEFAULT_ASSOCIATION_TIME_WINDOW_S = 0.75
DEFAULT_ASSOCIATION_CHI2_XY = 5.991
DEFAULT_ASSOCIATION_MAX_DISTANCE_M = 1.5
DEFAULT_ASSOCIATION_REQUIRE_FOOTPRINT_OVERLAP = False
DEFAULT_CONFIRMATION_WINDOW_S = 2.0
DEFAULT_SINGLE_SOURCE_CONFIRM_HITS = 2
DEFAULT_TRACK_TIMEOUT_S = 2.0
DEFAULT_MAX_TRACK_COUNT = 256
DEFAULT_CONFLICT_MAX_DIMENSION_RATIO = 3.0
DEFAULT_QUALITY_WEIGHT_CONFIDENCE = 0.35
DEFAULT_QUALITY_WEIGHT_FRESHNESS = 0.25
DEFAULT_QUALITY_WEIGHT_UNCERTAINTY = 0.25
DEFAULT_QUALITY_WEIGHT_VIEW = 0.15
DEFAULT_QUALITY_WEIGHT_COMMUNICATION = 0.0
DEFAULT_SELECTION_SCORE_EPSILON = 0.01
DEFAULT_SOURCE_COMMUNICATION_QUALITY = 1.0
DEFAULT_SOURCE_COMMUNICATION_PENALTY = 0.0
QUALITY_WEIGHT_TOLERANCE = 1.0e-6
_NANOSECONDS_PER_SECOND = 1_000_000_000


def _stamp_ns(stamp: Time) -> int:
    return int(stamp.sec) * _NANOSECONDS_PER_SECOND + int(stamp.nanosec)


def _duration_ns(duration: Duration) -> int:
    return int(duration.sec) * _NANOSECONDS_PER_SECOND + int(duration.nanosec)


def _time_from_ns(stamp_ns: int) -> Time:
    if stamp_ns < 0:
        raise ValueError("timestamp must be non-negative")
    return Time(
        sec=stamp_ns // _NANOSECONDS_PER_SECOND,
        nanosec=stamp_ns % _NANOSECONDS_PER_SECOND,
    )


def _finite_positive(values) -> bool:
    return all(math.isfinite(float(value)) and float(value) > 0.0 for value in values)


def _class_key(hazard: AerialHazard) -> str:
    if not hazard.detection.results:
        return ""
    return str(hazard.detection.results[0].hypothesis.class_id).strip().casefold()


def _xy(hazard: AerialHazard) -> tuple[float, float]:
    center = hazard.detection.bbox.center.position
    return float(center.x), float(center.y)


def _xy_covariance(hazard: AerialHazard) -> tuple[float, float, float, float]:
    covariance = hazard.detection.results[0].pose.covariance
    return (
        float(covariance[0]),
        float(covariance[1]),
        float(covariance[6]),
        float(covariance[7]),
    )


def _yaw(hazard: AerialHazard) -> float:
    orientation = hazard.detection.bbox.center.orientation
    return math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )


def _footprints_overlap(left: AerialHazard, right: AerialHazard) -> bool:
    """Return whether two oriented XY rectangles overlap using SAT."""
    left_x, left_y = _xy(left)
    right_x, right_y = _xy(right)
    left_yaw = _yaw(left)
    right_yaw = _yaw(right)
    left_axes = (
        (math.cos(left_yaw), math.sin(left_yaw)),
        (-math.sin(left_yaw), math.cos(left_yaw)),
    )
    right_axes = (
        (math.cos(right_yaw), math.sin(right_yaw)),
        (-math.sin(right_yaw), math.cos(right_yaw)),
    )
    delta_x = right_x - left_x
    delta_y = right_y - left_y
    left_half = (
        0.5 * float(left.detection.bbox.size.x),
        0.5 * float(left.detection.bbox.size.y),
    )
    right_half = (
        0.5 * float(right.detection.bbox.size.x),
        0.5 * float(right.detection.bbox.size.y),
    )

    for axis_x, axis_y in left_axes + right_axes:
        center_distance = abs(delta_x * axis_x + delta_y * axis_y)
        left_radius = sum(
            left_half[index]
            * abs(left_axes[index][0] * axis_x + left_axes[index][1] * axis_y)
            for index in range(2)
        )
        right_radius = sum(
            right_half[index]
            * abs(right_axes[index][0] * axis_x + right_axes[index][1] * axis_y)
            for index in range(2)
        )
        if center_distance > left_radius + right_radius:
            return False
    return True


def _dimension_ratio(left: AerialHazard, right: AerialHazard) -> float:
    ratios = []
    for left_value, right_value in (
        (left.detection.bbox.size.x, right.detection.bbox.size.x),
        (left.detection.bbox.size.y, right.detection.bbox.size.y),
        (left.detection.bbox.size.z, right.detection.bbox.size.z),
    ):
        low = min(float(left_value), float(right_value))
        high = max(float(left_value), float(right_value))
        ratios.append(high / low)
    return max(ratios)


def validate_hazard(
    hazard: AerialHazard,
    *,
    array_header: Header,
    now_ns: int,
    stale_timeout_s: float,
    max_covariance: float,
    max_source_age_s: Optional[float] = None,
) -> Optional[str]:
    """Return a rejection reason, or None when a hazard is safe to fuse."""
    if array_header.frame_id != MAP_FRAME:
        return "array_not_in_map"

    array_stamp_ns = _stamp_ns(array_header.stamp)
    detection = hazard.detection
    detection_stamp_ns = _stamp_ns(detection.header.stamp)
    first_seen_ns = _stamp_ns(hazard.first_seen)
    last_seen_ns = _stamp_ns(hazard.last_seen)
    if min(array_stamp_ns, detection_stamp_ns, first_seen_ns, last_seen_ns) <= 0:
        return "missing_stamp"
    if detection.header.frame_id != MAP_FRAME:
        return "detection_not_in_map"
    if first_seen_ns > last_seen_ns:
        return "invalid_track_time_order"

    source_age_limit_s = (
        float(stale_timeout_s)
        if max_source_age_s is None
        else float(max_source_age_s)
    )
    source_age_ns = now_ns - detection_stamp_ns
    if source_age_ns < 0:
        return "future_stamp"
    if source_age_ns > int(round(source_age_limit_s * _NANOSECONDS_PER_SECOND)):
        return "source_age_exceeded"

    stale_timeout_ns = int(round(float(stale_timeout_s) * _NANOSECONDS_PER_SECOND))
    for stamp_ns in (array_stamp_ns, detection_stamp_ns, last_seen_ns):
        age_ns = now_ns - stamp_ns
        if age_ns < 0:
            return "future_stamp"
        if age_ns > stale_timeout_ns:
            return "stale"

    ttl_ns = _duration_ns(hazard.ttl)
    if ttl_ns < 0:
        return "invalid_ttl"
    if now_ns - last_seen_ns > ttl_ns:
        return "expired"

    if not str(detection.id).strip():
        return "missing_track_id"
    if not _finite_positive(
        (detection.bbox.size.x, detection.bbox.size.y, detection.bbox.size.z)
    ):
        return "invalid_dimensions"
    center = detection.bbox.center.position
    if not all(math.isfinite(float(value)) for value in (center.x, center.y, center.z)):
        return "invalid_position"
    orientation = detection.bbox.center.orientation
    quaternion = tuple(
        float(value)
        for value in (orientation.x, orientation.y, orientation.z, orientation.w)
    )
    if not all(math.isfinite(value) for value in quaternion):
        return "invalid_orientation"
    if sum(value * value for value in quaternion) <= 0.0:
        return "invalid_orientation"
    if not detection.results:
        return "missing_confidence"

    result = detection.results[0]
    if not str(result.hypothesis.class_id).strip():
        return "missing_class"
    confidence = float(result.hypothesis.score)
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        return "invalid_confidence"
    support_quality = float(hazard.support_quality)
    if not math.isfinite(support_quality) or not 0.0 <= support_quality <= 1.0:
        return "invalid_support_quality"

    if hazard.state not in (
        AerialHazard.TENTATIVE,
        AerialHazard.CONFIRMED,
        AerialHazard.CONFLICT,
    ):
        return "unsupported_state"

    covariance = [float(value) for value in result.pose.covariance]
    if len(covariance) != 36 or not all(math.isfinite(value) for value in covariance):
        return "invalid_covariance"
    if not any(value != 0.0 for value in covariance):
        return "zero_covariance"
    if any(covariance[index] < 0.0 for index in (0, 7, 14, 21, 28, 35)):
        return "negative_covariance_diagonal"
    if max(abs(value) for value in covariance) > float(max_covariance):
        return "excessive_covariance"

    return None


@dataclass(frozen=True)
class StoredHazard:
    hazard: AerialHazard
    array_stamp_ns: int
    source_id: str
    source_order: int

    @property
    def detection_id(self) -> str:
        return str(self.hazard.detection.id)

    @property
    def last_seen_ns(self) -> int:
        return _stamp_ns(self.hazard.last_seen)

    @property
    def acquisition_stamp_ns(self) -> int:
        return _stamp_ns(self.hazard.detection.header.stamp)


@dataclass(frozen=True)
class QualityTerms:
    confidence: float
    freshness: float
    uncertainty: float
    view: float
    communication: float
    score: float


@dataclass
class FusionDiagnostics:
    stale_source_rejections: int = 0
    dropped_source_evidence: int = 0
    expired_source_evidence: int = 0
    source_timeouts: int = 0
    no_fresh_source_available: int = 0
    communication_adjusted_selections: int = 0
    expired_tracks: int = 0
    selected_sources: dict[str, int] = field(default_factory=dict)
    last_selection_terms: dict[str, QualityTerms] = field(default_factory=dict)


@dataclass
class FusionTrack:
    stable_id: str
    class_key: str
    first_seen_ns: int
    last_update_ns: int
    evidence: dict[str, StoredHazard] = field(default_factory=dict)
    source_track_ids: dict[str, str] = field(default_factory=dict)
    source_hit_counts: dict[str, int] = field(default_factory=dict)
    source_last_hit_ns: dict[str, int] = field(default_factory=dict)
    contributing_sources: set[str] = field(default_factory=set)


class HazardFusionCore:
    """Associate validated snapshots into bounded conservative dji0 tracks."""

    def __init__(
        self,
        *,
        stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S,
        max_source_age_s: float = DEFAULT_MAX_SOURCE_AGE_S,
        source_timeout_s: float = DEFAULT_SOURCE_TIMEOUT_S,
        max_covariance: float = DEFAULT_MAX_COVARIANCE,
        association_time_window_s: float = DEFAULT_ASSOCIATION_TIME_WINDOW_S,
        association_chi2_xy: float = DEFAULT_ASSOCIATION_CHI2_XY,
        association_max_distance_m: float = DEFAULT_ASSOCIATION_MAX_DISTANCE_M,
        association_require_footprint_overlap: bool = (
            DEFAULT_ASSOCIATION_REQUIRE_FOOTPRINT_OVERLAP
        ),
        confirmation_window_s: float = DEFAULT_CONFIRMATION_WINDOW_S,
        single_source_confirm_hits: int = DEFAULT_SINGLE_SOURCE_CONFIRM_HITS,
        track_timeout_s: float = DEFAULT_TRACK_TIMEOUT_S,
        max_track_count: int = DEFAULT_MAX_TRACK_COUNT,
        conflict_max_dimension_ratio: float = DEFAULT_CONFLICT_MAX_DIMENSION_RATIO,
        quality_weight_confidence: float = DEFAULT_QUALITY_WEIGHT_CONFIDENCE,
        quality_weight_freshness: float = DEFAULT_QUALITY_WEIGHT_FRESHNESS,
        quality_weight_uncertainty: float = DEFAULT_QUALITY_WEIGHT_UNCERTAINTY,
        quality_weight_view: float = DEFAULT_QUALITY_WEIGHT_VIEW,
        quality_weight_communication: float = DEFAULT_QUALITY_WEIGHT_COMMUNICATION,
        selection_score_epsilon: float = DEFAULT_SELECTION_SCORE_EPSILON,
        source_communication_quality: Optional[dict[str, float]] = None,
        source_communication_penalty: Optional[dict[str, float]] = None,
        source_order: tuple[str, ...] = ("dji1", "dji2"),
    ) -> None:
        self.stale_timeout_s = float(stale_timeout_s)
        self.max_source_age_s = float(max_source_age_s)
        self.source_timeout_s = float(source_timeout_s)
        self.max_covariance = float(max_covariance)
        self.association_time_window_s = float(association_time_window_s)
        self.association_chi2_xy = float(association_chi2_xy)
        self.association_max_distance_m = float(association_max_distance_m)
        self.association_require_footprint_overlap = bool(
            association_require_footprint_overlap
        )
        self.confirmation_window_s = float(confirmation_window_s)
        self.single_source_confirm_hits = int(single_source_confirm_hits)
        self.track_timeout_s = float(track_timeout_s)
        self.max_track_count = int(max_track_count)
        self.conflict_max_dimension_ratio = float(conflict_max_dimension_ratio)
        self.quality_weights = (
            float(quality_weight_confidence),
            float(quality_weight_freshness),
            float(quality_weight_uncertainty),
            float(quality_weight_view),
            float(quality_weight_communication),
        )
        self.selection_score_epsilon = float(selection_score_epsilon)
        self.source_order = tuple(source_order)
        quality_overrides = source_communication_quality or {}
        penalty_overrides = source_communication_penalty or {}
        self.source_communication_quality = {
            source_id: float(
                quality_overrides.get(source_id, DEFAULT_SOURCE_COMMUNICATION_QUALITY)
            )
            for source_id in self.source_order
        }
        self.source_communication_penalty = {
            source_id: float(
                penalty_overrides.get(source_id, DEFAULT_SOURCE_COMMUNICATION_PENALTY)
            )
            for source_id in self.source_order
        }
        self._validate_configuration()
        self._tracks: dict[str, FusionTrack] = {}
        self._next_track_index = 1
        self._last_source_message_ns: dict[str, Optional[int]] = {
            source_id: None for source_id in self.source_order
        }
        self._timed_out_sources: set[str] = set()
        self._tracks_without_fresh_evidence: set[str] = set()
        self.diagnostics = FusionDiagnostics(
            selected_sources={source_id: 0 for source_id in self.source_order}
        )

    @property
    def track_count(self) -> int:
        return len(self._tracks)

    def _validate_configuration(self) -> None:
        positive_values = {
            "stale_timeout_s": self.stale_timeout_s,
            "max_source_age_s": self.max_source_age_s,
            "source_timeout_s": self.source_timeout_s,
            "max_covariance": self.max_covariance,
            "association_time_window_s": self.association_time_window_s,
            "association_chi2_xy": self.association_chi2_xy,
            "association_max_distance_m": self.association_max_distance_m,
            "confirmation_window_s": self.confirmation_window_s,
            "track_timeout_s": self.track_timeout_s,
        }
        for name, value in positive_values.items():
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f"{name} must be finite and greater than zero")
        if self.single_source_confirm_hits < 2:
            raise ValueError("single_source_confirm_hits must be at least 2")
        if self.max_track_count < 1:
            raise ValueError("max_track_count must be at least 1")
        if (
            not math.isfinite(self.conflict_max_dimension_ratio)
            or self.conflict_max_dimension_ratio < 1.0
        ):
            raise ValueError("conflict_max_dimension_ratio must be finite and at least 1")
        if not self.source_order or len(set(self.source_order)) != len(self.source_order):
            raise ValueError("source_order must contain unique source IDs")
        if not all(math.isfinite(weight) and weight >= 0.0 for weight in self.quality_weights):
            raise ValueError("quality weights must be finite and non-negative")
        if abs(sum(self.quality_weights) - 1.0) > QUALITY_WEIGHT_TOLERANCE:
            raise ValueError("quality weights must sum to one")
        if (
            not math.isfinite(self.selection_score_epsilon)
            or self.selection_score_epsilon < 0.0
        ):
            raise ValueError("selection_score_epsilon must be finite and non-negative")
        for label, values in (
            ("source communication quality", self.source_communication_quality),
            ("source communication penalty", self.source_communication_penalty),
        ):
            if not all(
                math.isfinite(value) and 0.0 <= value <= 1.0
                for value in values.values()
            ):
                raise ValueError(f"{label} values must be in [0,1]")

    def replace_source(
        self,
        source_id: str,
        message: AerialHazardArray,
        *,
        now_ns: int,
    ) -> int:
        """Atomically replace one source snapshot and associate its observations."""
        if source_id not in self.source_order:
            raise ValueError(f"unknown source_id: {source_id}")
        self._prune_inactive_evidence(now_ns=now_ns)
        self._cleanup_tracks(now_ns=now_ns)
        self._last_source_message_ns[source_id] = now_ns
        self._timed_out_sources.discard(source_id)
        source_order = self.source_order.index(source_id)
        accepted_by_id: dict[str, StoredHazard] = {}
        if message.header.frame_id == MAP_FRAME and _stamp_ns(message.header.stamp) > 0:
            for hazard in message.hazards:
                reason = validate_hazard(
                    hazard,
                    array_header=message.header,
                    now_ns=now_ns,
                    stale_timeout_s=self.stale_timeout_s,
                    max_covariance=self.max_covariance,
                    max_source_age_s=self.max_source_age_s,
                )
                if reason is not None:
                    self._record_rejection(reason)
                    continue
                candidate = StoredHazard(
                    hazard=hazard,
                    array_stamp_ns=_stamp_ns(message.header.stamp),
                    source_id=source_id,
                    source_order=source_order,
                )
                current = accepted_by_id.get(candidate.detection_id)
                if current is None or self._prefer_candidate(
                    candidate,
                    current,
                    now_ns=now_ns,
                ):
                    accepted_by_id[candidate.detection_id] = candidate

        old_evidence = {
            track_id: track.evidence.get(source_id)
            for track_id, track in self._tracks.items()
        }
        assignments: dict[str, StoredHazard] = {}
        assigned_tracks: set[str] = set()
        for candidate in sorted(
            accepted_by_id.values(),
            key=lambda item: (item.detection_id, _class_key(item.hazard), _xy(item.hazard)),
        ):
            track = self._find_track(
                candidate,
                assigned_tracks=assigned_tracks,
                old_evidence=old_evidence,
            )
            if track is None:
                track = self._create_track(
                    candidate,
                    now_ns=now_ns,
                    protected_track_ids=assigned_tracks,
                )
            if track is None:
                continue
            assignments[track.stable_id] = candidate
            assigned_tracks.add(track.stable_id)

        for track_id, track in self._tracks.items():
            removed = track.evidence.pop(source_id, None)
            if removed is not None and track_id not in assignments:
                self.diagnostics.dropped_source_evidence += 1
        for track_id, candidate in assignments.items():
            track = self._tracks[track_id]
            previous = old_evidence.get(track_id)
            self._update_source_hit_count(track, candidate, previous)
            track.evidence[source_id] = candidate
            track.source_track_ids[source_id] = candidate.detection_id
            track.first_seen_ns = min(
                track.first_seen_ns, _stamp_ns(candidate.hazard.first_seen)
            )
            track.last_update_ns = max(track.last_update_ns, candidate.last_seen_ns)
            track.contributing_sources.update(candidate.hazard.source_uavs)
            track.contributing_sources.add(source_id)
            self._remove_duplicate_source_mapping(track, candidate)

        self._cleanup_tracks(now_ns=now_ns)
        return len(assignments)

    def selected_hazards(self, *, now_ns: int) -> list[AerialHazard]:
        self._prune_inactive_evidence(now_ns=now_ns)
        self._cleanup_tracks(now_ns=now_ns)
        active = {
            track_id: self._active_evidence(track, now_ns=now_ns)
            for track_id, track in self._tracks.items()
        }
        for track_id, evidence in active.items():
            if evidence:
                self._tracks_without_fresh_evidence.discard(track_id)
            elif track_id not in self._tracks_without_fresh_evidence:
                self.diagnostics.no_fresh_source_available += 1
                self._tracks_without_fresh_evidence.add(track_id)
        active = {track_id: evidence for track_id, evidence in active.items() if evidence}
        conflicts = self._conflicting_track_ids(active)
        output: list[AerialHazard] = []
        for track_id in sorted(active):
            track = self._tracks[track_id]
            evidence = active[track_id]
            selected = self._select_representative(evidence, now_ns=now_ns)
            terms = self._quality_terms(selected, now_ns)
            state = (
                AerialHazard.CONFIRMED
                if self._track_is_confirmed(track, evidence, now_ns=now_ns)
                else AerialHazard.TENTATIVE
            )
            if track_id in conflicts or any(
                candidate.hazard.state == AerialHazard.CONFLICT for candidate in evidence
            ):
                state = AerialHazard.CONFLICT
            fused = copy.deepcopy(selected.hazard)
            fused.detection.id = track.stable_id
            fused.source_uavs = sorted(track.contributing_sources, key=self._source_sort_key)
            fused.state = state
            fused.first_seen = _time_from_ns(track.first_seen_ns)
            fused.support_quality = float(terms.score)
            if state == AerialHazard.CONFLICT:
                fused.provenance = "dji0_multi_conflict"
            elif state == AerialHazard.CONFIRMED and len(track.contributing_sources) > 1:
                fused.provenance = "rgbd_multi_confirmed"
            elif state == AerialHazard.CONFIRMED:
                fused.provenance = "dji0_repeated_confirmed"
            else:
                fused.provenance = "dji0_tentative"
            output.append(fused)
            self.diagnostics.selected_sources[selected.source_id] += 1
            self.diagnostics.last_selection_terms[selected.source_id] = terms
            if (
                self.quality_weights[4] > 0.0
                and any(
                    self._communication_score(candidate.source_id)
                    < DEFAULT_SOURCE_COMMUNICATION_QUALITY
                    for candidate in evidence
                )
            ):
                self.diagnostics.communication_adjusted_selections += 1
        return output

    def build_output(self, *, now_ns: int) -> AerialHazardArray:
        message = AerialHazardArray()
        message.header = Header(stamp=_time_from_ns(now_ns), frame_id=MAP_FRAME)
        message.hazards = self.selected_hazards(now_ns=now_ns)
        return message

    def _find_track(
        self,
        candidate: StoredHazard,
        *,
        assigned_tracks: set[str],
        old_evidence: dict[str, Optional[StoredHazard]],
    ) -> Optional[FusionTrack]:
        direct_matches: list[tuple[tuple, FusionTrack]] = []
        spatial_matches: list[tuple[tuple, FusionTrack]] = []
        for track_id in sorted(self._tracks):
            if track_id in assigned_tracks:
                continue
            track = self._tracks[track_id]
            if track.class_key != _class_key(candidate.hazard):
                continue
            old_same_source = old_evidence.get(track_id)
            known_detection_id = track.source_track_ids.get(candidate.source_id)
            if known_detection_id is not None and known_detection_id != candidate.detection_id:
                continue
            score = self._best_association_score(track, candidate)
            if score is None and old_same_source is not None:
                score = self._pair_association_score(old_same_source, candidate)
            if score is None and known_detection_id == candidate.detection_id:
                direct_matches.append(
                    ((2.0, math.inf, math.inf, track.stable_id), track)
                )
                continue
            if score is None:
                continue
            entry = (score + (track.stable_id,), track)
            if known_detection_id == candidate.detection_id:
                direct_matches.append(entry)
            else:
                spatial_matches.append(entry)
        matches = direct_matches or spatial_matches
        return min(matches, key=lambda item: item[0])[1] if matches else None

    def _best_association_score(
        self, track: FusionTrack, candidate: StoredHazard
    ) -> Optional[tuple[float, float, float]]:
        scores = [
            score
            for evidence in track.evidence.values()
            if (score := self._pair_association_score(evidence, candidate)) is not None
        ]
        return min(scores) if scores else None

    def _pair_association_score(
        self, left: StoredHazard, right: StoredHazard
    ) -> Optional[tuple[float, float, float]]:
        if _class_key(left.hazard) != _class_key(right.hazard):
            return None
        time_delta_s = abs(left.last_seen_ns - right.last_seen_ns) / _NANOSECONDS_PER_SECOND
        if time_delta_s > self.association_time_window_s:
            return None
        if self.association_require_footprint_overlap and not _footprints_overlap(
            left.hazard, right.hazard
        ):
            return None

        left_x, left_y = _xy(left.hazard)
        right_x, right_y = _xy(right.hazard)
        delta_x = right_x - left_x
        delta_y = right_y - left_y
        euclidean = math.hypot(delta_x, delta_y)
        mahalanobis = self._mahalanobis_xy(left.hazard, right.hazard, delta_x, delta_y)
        if mahalanobis is not None:
            if mahalanobis > self.association_chi2_xy:
                return None
            return 0.0, mahalanobis, euclidean
        if euclidean > self.association_max_distance_m:
            return None
        return 1.0, euclidean, euclidean

    @staticmethod
    def _mahalanobis_xy(
        left: AerialHazard,
        right: AerialHazard,
        delta_x: float,
        delta_y: float,
    ) -> Optional[float]:
        left_xx, left_xy, left_yx, left_yy = _xy_covariance(left)
        right_xx, right_xy, right_yx, right_yy = _xy_covariance(right)
        covariance_xx = left_xx + right_xx
        covariance_yy = left_yy + right_yy
        covariance_xy = 0.5 * (left_xy + left_yx + right_xy + right_yx)
        determinant = covariance_xx * covariance_yy - covariance_xy * covariance_xy
        if covariance_xx <= 0.0 or covariance_yy <= 0.0 or determinant <= 1.0e-12:
            return None
        value = (
            covariance_yy * delta_x * delta_x
            - 2.0 * covariance_xy * delta_x * delta_y
            + covariance_xx * delta_y * delta_y
        ) / determinant
        if not math.isfinite(value) or value < 0.0:
            return None
        return value

    def _create_track(
        self,
        candidate: StoredHazard,
        *,
        now_ns: int,
        protected_track_ids: set[str],
    ) -> Optional[FusionTrack]:
        if not self._make_capacity(
            now_ns=now_ns,
            protected_track_ids=protected_track_ids,
        ):
            return None
        stable_id = f"dji0-hazard-{self._next_track_index:06d}"
        self._next_track_index += 1
        first_seen_ns = _stamp_ns(candidate.hazard.first_seen)
        track = FusionTrack(
            stable_id=stable_id,
            class_key=_class_key(candidate.hazard),
            first_seen_ns=first_seen_ns,
            last_update_ns=candidate.last_seen_ns,
        )
        self._tracks[stable_id] = track
        return track

    def _make_capacity(
        self,
        *,
        now_ns: int,
        protected_track_ids: set[str],
    ) -> bool:
        self._cleanup_tracks(now_ns=now_ns)
        if len(self._tracks) < self.max_track_count:
            return True
        candidates = [
            track
            for track in self._tracks.values()
            if track.stable_id not in protected_track_ids
        ]
        if not candidates:
            return False
        victim = min(
            candidates,
            key=lambda track: (
                bool(self._active_evidence(track, now_ns=now_ns)),
                track.last_update_ns,
                track.stable_id,
            ),
        )
        del self._tracks[victim.stable_id]
        self._tracks_without_fresh_evidence.discard(victim.stable_id)
        return True

    def _update_source_hit_count(
        self,
        track: FusionTrack,
        candidate: StoredHazard,
        previous: Optional[StoredHazard],
    ) -> None:
        source_id = candidate.source_id
        last_hit_ns = track.source_last_hit_ns.get(source_id)
        if previous is not None:
            last_hit_ns = previous.last_seen_ns
        if last_hit_ns is None or candidate.last_seen_ns <= last_hit_ns:
            count = track.source_hit_counts.get(source_id, 0)
            if last_hit_ns is None:
                count = 1
        else:
            gap_s = (candidate.last_seen_ns - last_hit_ns) / _NANOSECONDS_PER_SECOND
            count = track.source_hit_counts.get(source_id, 0) + 1
            if gap_s > self.confirmation_window_s:
                count = 1
        track.source_hit_counts[source_id] = count
        track.source_last_hit_ns[source_id] = max(
            candidate.last_seen_ns,
            track.source_last_hit_ns.get(source_id, 0),
        )

    def _remove_duplicate_source_mapping(
        self, selected_track: FusionTrack, candidate: StoredHazard
    ) -> None:
        for track in self._tracks.values():
            if track.stable_id == selected_track.stable_id:
                continue
            if track.source_track_ids.get(candidate.source_id) == candidate.detection_id:
                track.source_track_ids.pop(candidate.source_id, None)

    def _active_evidence(
        self, track: FusionTrack, *, now_ns: int
    ) -> list[StoredHazard]:
        del now_ns
        return sorted(
            track.evidence.values(),
            key=lambda evidence: (evidence.source_order, evidence.detection_id),
        )

    def _prune_inactive_evidence(self, *, now_ns: int) -> None:
        source_timeout_ns = int(round(self.source_timeout_s * _NANOSECONDS_PER_SECOND))
        timed_out_sources = {
            source_id
            for source_id, received_ns in self._last_source_message_ns.items()
            if received_ns is not None and now_ns - received_ns > source_timeout_ns
        }
        for source_id in timed_out_sources - self._timed_out_sources:
            self.diagnostics.source_timeouts += 1
        self._timed_out_sources.update(timed_out_sources)

        for track in self._tracks.values():
            for source_id, evidence in list(track.evidence.items()):
                if source_id in timed_out_sources:
                    del track.evidence[source_id]
                    self.diagnostics.dropped_source_evidence += 1
                    continue
                reason = validate_hazard(
                    evidence.hazard,
                    array_header=Header(
                        stamp=_time_from_ns(evidence.array_stamp_ns),
                        frame_id=MAP_FRAME,
                    ),
                    now_ns=now_ns,
                    stale_timeout_s=self.stale_timeout_s,
                    max_covariance=self.max_covariance,
                    max_source_age_s=self.max_source_age_s,
                )
                if reason is None:
                    continue
                del track.evidence[source_id]
                self.diagnostics.dropped_source_evidence += 1
                if reason == "expired":
                    self.diagnostics.expired_source_evidence += 1
                self._record_rejection(reason)

    def _record_rejection(self, reason: str) -> None:
        if reason in ("stale", "source_age_exceeded"):
            self.diagnostics.stale_source_rejections += 1

    def _track_is_confirmed(
        self,
        track: FusionTrack,
        evidence: list[StoredHazard],
        *,
        now_ns: int,
    ) -> bool:
        active_sources = {candidate.source_id for candidate in evidence}
        if len(active_sources) >= 2:
            return True
        confirmation_window_ns = int(
            round(self.confirmation_window_s * _NANOSECONDS_PER_SECOND)
        )
        return any(
            track.source_hit_counts.get(source_id, 0)
            >= self.single_source_confirm_hits
            and now_ns - track.source_last_hit_ns.get(source_id, 0)
            <= confirmation_window_ns
            for source_id in active_sources
        )

    def _cleanup_tracks(self, *, now_ns: int) -> None:
        timeout_ns = int(round(self.track_timeout_s * _NANOSECONDS_PER_SECOND))
        expired = [
            track_id
            for track_id, track in self._tracks.items()
            if now_ns - track.last_update_ns > timeout_ns
        ]
        for track_id in expired:
            del self._tracks[track_id]
            self._tracks_without_fresh_evidence.discard(track_id)
            self.diagnostics.expired_tracks += 1

    def _conflicting_track_ids(
        self, active: dict[str, list[StoredHazard]]
    ) -> set[str]:
        conflict_ids: set[str] = set()
        track_ids = sorted(active)
        for left_index, left_id in enumerate(track_ids):
            for right_id in track_ids[left_index + 1:]:
                if self._tracks_conflict(active[left_id], active[right_id]):
                    conflict_ids.update((left_id, right_id))
        return conflict_ids

    def _tracks_conflict(
        self, left_evidence: list[StoredHazard], right_evidence: list[StoredHazard]
    ) -> bool:
        if len({item.source_id for item in left_evidence + right_evidence}) < 2:
            return False
        for left in left_evidence:
            for right in right_evidence:
                time_delta_s = (
                    abs(left.last_seen_ns - right.last_seen_ns)
                    / _NANOSECONDS_PER_SECOND
                )
                if time_delta_s > self.association_time_window_s:
                    continue
                if not _footprints_overlap(left.hazard, right.hazard):
                    continue
                class_mismatch = _class_key(left.hazard) != _class_key(right.hazard)
                geometry_mismatch = (
                    _dimension_ratio(left.hazard, right.hazard)
                    > self.conflict_max_dimension_ratio
                )
                association_failed = self._pair_association_score(left, right) is None
                if class_mismatch or geometry_mismatch or association_failed:
                    return True
        return False

    def _communication_score(self, source_id: str) -> float:
        quality = self.source_communication_quality[source_id]
        penalty = self.source_communication_penalty[source_id]
        return min(1.0, max(0.0, quality - penalty))

    def _quality_terms(self, candidate: StoredHazard, now_ns: int) -> QualityTerms:
        confidence = float(candidate.hazard.detection.results[0].hypothesis.score)
        age_s = max(
            0.0,
            (now_ns - candidate.acquisition_stamp_ns) / _NANOSECONDS_PER_SECOND,
        )
        freshness = max(0.0, 1.0 - age_s / self.max_source_age_s)
        covariance = candidate.hazard.detection.results[0].pose.covariance
        max_xy_variance = max(float(covariance[0]), float(covariance[7]))
        uncertainty = max(0.0, 1.0 - max_xy_variance / self.max_covariance)
        view = float(candidate.hazard.support_quality)
        communication = self._communication_score(candidate.source_id)
        values = (confidence, freshness, uncertainty, view, communication)
        score = min(
            1.0,
            max(
                0.0,
                sum(weight * value for weight, value in zip(self.quality_weights, values)),
            ),
        )
        return QualityTerms(
            confidence=confidence,
            freshness=freshness,
            uncertainty=uncertainty,
            view=view,
            communication=communication,
            score=score,
        )

    def _quality_score(self, candidate: StoredHazard, now_ns: int) -> float:
        return self._quality_terms(candidate, now_ns).score

    def _prefer_candidate(
        self,
        candidate: StoredHazard,
        current: StoredHazard,
        *,
        now_ns: int,
    ) -> bool:
        candidate_score = self._quality_score(candidate, now_ns)
        current_score = self._quality_score(current, now_ns)
        if candidate_score > current_score + self.selection_score_epsilon:
            return True
        if current_score > candidate_score + self.selection_score_epsilon:
            return False
        candidate_key = (
            candidate.acquisition_stamp_ns,
            candidate.last_seen_ns,
            _class_key(candidate.hazard),
            _xy(candidate.hazard),
        )
        current_key = (
            current.acquisition_stamp_ns,
            current.last_seen_ns,
            _class_key(current.hazard),
            _xy(current.hazard),
        )
        return candidate_key > current_key

    def _select_representative(
        self,
        evidence: list[StoredHazard],
        *,
        now_ns: int,
    ) -> StoredHazard:
        ordered = sorted(
            evidence,
            key=lambda candidate: (candidate.source_order, candidate.detection_id),
        )
        selected = ordered[0]
        selected_score = self._quality_score(selected, now_ns)
        for candidate in ordered[1:]:
            candidate_score = self._quality_score(candidate, now_ns)
            if candidate_score > selected_score + self.selection_score_epsilon:
                selected = candidate
                selected_score = candidate_score
        return selected

    def diagnostic_summary(self) -> str:
        selected = ",".join(
            f"{source_id}:{self.diagnostics.selected_sources[source_id]}"
            for source_id in self.source_order
        )
        communication = ",".join(
            f"{source_id}:{self._communication_score(source_id):.2f}"
            for source_id in self.source_order
        )
        return (
            f"tracks={self.track_count} selected=[{selected}] "
            f"stale_rejected={self.diagnostics.stale_source_rejections} "
            f"dropped={self.diagnostics.dropped_source_evidence} "
            f"expired_evidence={self.diagnostics.expired_source_evidence} "
            f"source_timeouts={self.diagnostics.source_timeouts} "
            f"no_fresh={self.diagnostics.no_fresh_source_available} "
            f"expired_tracks={self.diagnostics.expired_tracks} "
            f"configured_comm=[{communication}]"
        )

    def _source_sort_key(self, source_id: str) -> tuple[int, str]:
        try:
            return self.source_order.index(source_id), source_id
        except ValueError:
            return len(self.source_order), source_id


class SupportHazardFusion(Node):
    """Fuse typed map-frame hazards from dji1 and optional dji2 evidence."""

    def __init__(self) -> None:
        super().__init__("support_hazard_fusion")

        self.dji1_topic = self._string_parameter("dji1_topic", DEFAULT_DJI1_TOPIC)
        self.dji2_topic = self._string_parameter("dji2_topic", DEFAULT_DJI2_TOPIC)
        self.output_topic = self._string_parameter("output_topic", DEFAULT_OUTPUT_TOPIC)
        self.dji2_enable = bool(
            self.declare_parameter("dji2_enable", DEFAULT_DJI2_ENABLE).value
        )
        self.stale_timeout_s = float(
            self.declare_parameter("stale_timeout_s", DEFAULT_STALE_TIMEOUT_S).value
        )
        self.max_covariance = float(
            self.declare_parameter("max_covariance", DEFAULT_MAX_COVARIANCE).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ).value
        )
        self.diagnostic_period_s = float(
            self.declare_parameter(
                "diagnostic_period_s", DEFAULT_DIAGNOSTIC_PERIOD_S
            ).value
        )
        core_parameters = {
            "max_source_age_s": DEFAULT_MAX_SOURCE_AGE_S,
            "source_timeout_s": DEFAULT_SOURCE_TIMEOUT_S,
            "association_time_window_s": DEFAULT_ASSOCIATION_TIME_WINDOW_S,
            "association_chi2_xy": DEFAULT_ASSOCIATION_CHI2_XY,
            "association_max_distance_m": DEFAULT_ASSOCIATION_MAX_DISTANCE_M,
            "association_require_footprint_overlap": DEFAULT_ASSOCIATION_REQUIRE_FOOTPRINT_OVERLAP,
            "confirmation_window_s": DEFAULT_CONFIRMATION_WINDOW_S,
            "single_source_confirm_hits": DEFAULT_SINGLE_SOURCE_CONFIRM_HITS,
            "track_timeout_s": DEFAULT_TRACK_TIMEOUT_S,
            "max_track_count": DEFAULT_MAX_TRACK_COUNT,
            "conflict_max_dimension_ratio": DEFAULT_CONFLICT_MAX_DIMENSION_RATIO,
            "quality_weight_confidence": DEFAULT_QUALITY_WEIGHT_CONFIDENCE,
            "quality_weight_freshness": DEFAULT_QUALITY_WEIGHT_FRESHNESS,
            "quality_weight_uncertainty": DEFAULT_QUALITY_WEIGHT_UNCERTAINTY,
            "quality_weight_view": DEFAULT_QUALITY_WEIGHT_VIEW,
            "quality_weight_communication": DEFAULT_QUALITY_WEIGHT_COMMUNICATION,
            "selection_score_epsilon": DEFAULT_SELECTION_SCORE_EPSILON,
        }
        configured = {
            name: self.declare_parameter(name, default).value
            for name, default in core_parameters.items()
        }
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and greater than zero")
        if (
            not math.isfinite(self.diagnostic_period_s)
            or self.diagnostic_period_s <= 0.0
        ):
            raise ValueError("diagnostic_period_s must be finite and greater than zero")

        source_communication_quality = {
            source_id: float(
                self.declare_parameter(
                    f"{source_id}_communication_quality",
                    DEFAULT_SOURCE_COMMUNICATION_QUALITY,
                ).value
            )
            for source_id in ("dji1", "dji2")
        }
        source_communication_penalty = {
            source_id: float(
                self.declare_parameter(
                    f"{source_id}_communication_penalty",
                    DEFAULT_SOURCE_COMMUNICATION_PENALTY,
                ).value
            )
            for source_id in ("dji1", "dji2")
        }

        self._core = HazardFusionCore(
            stale_timeout_s=self.stale_timeout_s,
            max_covariance=self.max_covariance,
            source_communication_quality=source_communication_quality,
            source_communication_penalty=source_communication_penalty,
            **configured,
        )
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._publisher = self.create_publisher(AerialHazardArray, self.output_topic, qos)
        self.create_subscription(
            AerialHazardArray,
            self.dji1_topic,
            partial(self._on_source, "dji1"),
            qos,
        )
        if self.dji2_enable:
            self.create_subscription(
                AerialHazardArray,
                self.dji2_topic,
                partial(self._on_source, "dji2"),
                qos,
            )
        self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.create_timer(self.diagnostic_period_s, self._on_diagnostics)
        self.get_logger().info(
            "[support_hazard_fusion] Started: "
            f"dji1={self.dji1_topic}, dji2={'on' if self.dji2_enable else 'off'}, "
            f"output={self.output_topic}, tracks<={self._core.max_track_count}, "
            f"association=({self._core.association_time_window_s:.2f}s,"
            f"chi2={self._core.association_chi2_xy:.3f}), "
            f"source_age<={self._core.max_source_age_s:.2f}s, "
            f"source_timeout={self._core.source_timeout_s:.2f}s, "
            f"communication=configured-only(weight={self._core.quality_weights[4]:.2f}), "
            f"publish_rate_hz={self.publish_rate_hz:.1f}"
        )

    def _string_parameter(self, name: str, default: str) -> str:
        value = str(self.declare_parameter(name, default).value).strip()
        return value or default

    def _on_source(self, source_id: str, message: AerialHazardArray) -> None:
        self._core.replace_source(
            source_id,
            message,
            now_ns=int(self.get_clock().now().nanoseconds),
        )

    def _on_timer(self) -> None:
        self._publisher.publish(
            self._core.build_output(now_ns=int(self.get_clock().now().nanoseconds))
        )

    def _on_diagnostics(self) -> None:
        self.get_logger().info(
            f"[support_hazard_fusion] {self._core.diagnostic_summary()}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupportHazardFusion()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
