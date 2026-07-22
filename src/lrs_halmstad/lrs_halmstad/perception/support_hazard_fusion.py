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
DEFAULT_MAX_COVARIANCE = 4.0
DEFAULT_PUBLISH_RATE_HZ = 10.0
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
    confirmed: bool = False


class HazardFusionCore:
    """Associate validated snapshots into bounded conservative dji0 tracks."""

    def __init__(
        self,
        *,
        stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S,
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
        source_order: tuple[str, ...] = ("dji1", "dji2"),
    ) -> None:
        self.stale_timeout_s = float(stale_timeout_s)
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
        )
        self.source_order = tuple(source_order)
        self._validate_configuration()
        self._tracks: dict[str, FusionTrack] = {}
        self._next_track_index = 1

    @property
    def track_count(self) -> int:
        return len(self._tracks)

    def _validate_configuration(self) -> None:
        positive_values = {
            "stale_timeout_s": self.stale_timeout_s,
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
        self._cleanup_tracks(now_ns=now_ns)
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
                )
                if reason is not None:
                    continue
                candidate = StoredHazard(
                    hazard=hazard,
                    array_stamp_ns=_stamp_ns(message.header.stamp),
                    source_id=source_id,
                    source_order=source_order,
                )
                current = accepted_by_id.get(candidate.detection_id)
                if current is None or self._evidence_selection_key(
                    candidate, now_ns
                ) > self._evidence_selection_key(current, now_ns):
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

        for track in self._tracks.values():
            track.evidence.pop(source_id, None)
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
            if track.source_hit_counts[source_id] >= self.single_source_confirm_hits:
                track.confirmed = True
            self._remove_duplicate_source_mapping(track, candidate)

        self._promote_multi_source_tracks(now_ns=now_ns)
        self._cleanup_tracks(now_ns=now_ns)
        return len(assignments)

    def selected_hazards(self, *, now_ns: int) -> list[AerialHazard]:
        self._cleanup_tracks(now_ns=now_ns)
        active = {
            track_id: self._active_evidence(track, now_ns=now_ns)
            for track_id, track in self._tracks.items()
        }
        active = {track_id: evidence for track_id, evidence in active.items() if evidence}
        conflicts = self._conflicting_track_ids(active)
        output: list[AerialHazard] = []
        for track_id in sorted(active):
            track = self._tracks[track_id]
            evidence = active[track_id]
            selected = max(
                evidence,
                key=lambda candidate: self._evidence_selection_key(candidate, now_ns),
            )
            state = AerialHazard.CONFIRMED if track.confirmed else AerialHazard.TENTATIVE
            if track_id in conflicts or any(
                candidate.hazard.state == AerialHazard.CONFLICT for candidate in evidence
            ):
                state = AerialHazard.CONFLICT
            fused = copy.deepcopy(selected.hazard)
            fused.detection.id = track.stable_id
            fused.source_uavs = sorted(track.contributing_sources, key=self._source_sort_key)
            fused.state = state
            fused.first_seen = _time_from_ns(track.first_seen_ns)
            fused.support_quality = float(self._quality_score(selected, now_ns))
            if state == AerialHazard.CONFLICT:
                fused.provenance = "dji0_multi_conflict"
            elif state == AerialHazard.CONFIRMED and len(track.contributing_sources) > 1:
                fused.provenance = "rgbd_multi_confirmed"
            elif state == AerialHazard.CONFIRMED:
                fused.provenance = "dji0_repeated_confirmed"
            else:
                fused.provenance = "dji0_tentative"
            output.append(fused)
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

    def _promote_multi_source_tracks(self, *, now_ns: int) -> None:
        for track in self._tracks.values():
            active_sources = {
                evidence.source_id
                for evidence in self._active_evidence(track, now_ns=now_ns)
            }
            if len(active_sources) >= 2:
                track.confirmed = True

    def _active_evidence(
        self, track: FusionTrack, *, now_ns: int
    ) -> list[StoredHazard]:
        active = []
        for evidence in track.evidence.values():
            header = Header(
                stamp=_time_from_ns(evidence.array_stamp_ns),
                frame_id=MAP_FRAME,
            )
            if validate_hazard(
                evidence.hazard,
                array_header=header,
                now_ns=now_ns,
                stale_timeout_s=self.stale_timeout_s,
                max_covariance=self.max_covariance,
            ) is None:
                active.append(evidence)
        return active

    def _cleanup_tracks(self, *, now_ns: int) -> None:
        timeout_ns = int(round(self.track_timeout_s * _NANOSECONDS_PER_SECOND))
        expired = [
            track_id
            for track_id, track in self._tracks.items()
            if now_ns - track.last_update_ns > timeout_ns
        ]
        for track_id in expired:
            del self._tracks[track_id]

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

    def _quality_score(self, candidate: StoredHazard, now_ns: int) -> float:
        confidence = float(candidate.hazard.detection.results[0].hypothesis.score)
        age_s = max(0.0, (now_ns - candidate.last_seen_ns) / _NANOSECONDS_PER_SECOND)
        freshness = max(0.0, 1.0 - age_s / self.stale_timeout_s)
        covariance = candidate.hazard.detection.results[0].pose.covariance
        max_xy_variance = max(float(covariance[0]), float(covariance[7]))
        uncertainty = max(0.0, 1.0 - max_xy_variance / self.max_covariance)
        view = float(candidate.hazard.support_quality)
        terms = (confidence, freshness, uncertainty, view)
        return min(1.0, max(0.0, sum(w * term for w, term in zip(self.quality_weights, terms))))

    def _evidence_selection_key(
        self, candidate: StoredHazard, now_ns: int
    ) -> tuple[float, int, float, int, str]:
        return (
            self._quality_score(candidate, now_ns),
            candidate.last_seen_ns,
            float(candidate.hazard.detection.results[0].hypothesis.score),
            -candidate.source_order,
            candidate.detection_id,
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
        core_parameters = {
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
        }
        configured = {
            name: self.declare_parameter(name, default).value
            for name, default in core_parameters.items()
        }
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and greater than zero")

        self._core = HazardFusionCore(
            stale_timeout_s=self.stale_timeout_s,
            max_covariance=self.max_covariance,
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
        self.get_logger().info(
            "[support_hazard_fusion] Started: "
            f"dji1={self.dji1_topic}, dji2={'on' if self.dji2_enable else 'off'}, "
            f"output={self.output_topic}, tracks<={self._core.max_track_count}, "
            f"association=({self._core.association_time_window_s:.2f}s,"
            f"chi2={self._core.association_chi2_xy:.3f}), "
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
