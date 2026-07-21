"""Conservative typed support-hazard selection at dji0."""

from __future__ import annotations

import math
from dataclasses import dataclass
from functools import partial
from typing import Optional

import rclpy
from builtin_interfaces.msg import Duration, Time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header

from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray


MAP_FRAME = "map"
DEFAULT_DJI1_TOPIC = "/coord/support/dji1/aerial_hazards"
DEFAULT_DJI2_TOPIC = "/coord/support/dji2/aerial_hazards"
DEFAULT_OUTPUT_TOPIC = "/coord/dji0/aerial_hazards"
DEFAULT_STALE_TIMEOUT_S = 0.75
DEFAULT_MAX_COVARIANCE = 4.0
DEFAULT_PUBLISH_RATE_HZ = 10.0


def _stamp_ns(stamp: Time) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _duration_ns(duration: Duration) -> int:
    return int(duration.sec) * 1_000_000_000 + int(duration.nanosec)


def _time_from_ns(stamp_ns: int) -> Time:
    if stamp_ns < 0:
        raise ValueError("timestamp must be non-negative")
    return Time(sec=stamp_ns // 1_000_000_000, nanosec=stamp_ns % 1_000_000_000)


def _finite_positive(values) -> bool:
    return all(math.isfinite(float(value)) and float(value) > 0.0 for value in values)


def validate_hazard(
    hazard: AerialHazard,
    *,
    array_header: Header,
    now_ns: int,
    stale_timeout_s: float,
    max_covariance: float,
) -> Optional[str]:
    """Return a rejection reason, or None when a hazard is safe to select."""
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

    stale_timeout_ns = int(round(float(stale_timeout_s) * 1_000_000_000.0))
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
    if not detection.results:
        return "missing_confidence"

    result = detection.results[0]
    confidence = float(result.hypothesis.score)
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        return "invalid_confidence"
    support_quality = float(hazard.support_quality)
    if not math.isfinite(support_quality) or not 0.0 <= support_quality <= 1.0:
        return "invalid_support_quality"

    if hazard.state not in (AerialHazard.TENTATIVE, AerialHazard.CONFIRMED):
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
    def last_seen_ns(self) -> int:
        return _stamp_ns(self.hazard.last_seen)


class HazardFusionCore:
    """Keep one acceptable pass-through hazard per source and stable track ID."""

    def __init__(
        self,
        *,
        stale_timeout_s: float = DEFAULT_STALE_TIMEOUT_S,
        max_covariance: float = DEFAULT_MAX_COVARIANCE,
        source_order: tuple[str, ...] = ("dji1", "dji2"),
    ) -> None:
        self.stale_timeout_s = float(stale_timeout_s)
        self.max_covariance = float(max_covariance)
        self.source_order = tuple(source_order)
        self._source_tracks: dict[str, dict[str, StoredHazard]] = {
            source_id: {} for source_id in self.source_order
        }

    def replace_source(
        self,
        source_id: str,
        message: AerialHazardArray,
        *,
        now_ns: int,
    ) -> int:
        """Replace one source snapshot; invalid or absent hazards are removed."""
        source_order = self.source_order.index(source_id)
        accepted: dict[str, StoredHazard] = {}
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
                current = accepted.get(hazard.detection.id)
                if current is None or self._selection_key(candidate) > self._selection_key(current):
                    accepted[hazard.detection.id] = candidate

        self._source_tracks[source_id] = accepted
        return len(accepted)

    def selected_hazards(self, *, now_ns: int) -> list[AerialHazard]:
        selected: dict[str, StoredHazard] = {}
        for source_tracks in self._source_tracks.values():
            for track_id, candidate in source_tracks.items():
                if validate_hazard(
                    candidate.hazard,
                    array_header=Header(
                        stamp=_time_from_ns(candidate.array_stamp_ns),
                        frame_id=MAP_FRAME,
                    ),
                    now_ns=now_ns,
                    stale_timeout_s=self.stale_timeout_s,
                    max_covariance=self.max_covariance,
                ) is not None:
                    continue
                current = selected.get(track_id)
                if current is None or self._selection_key(candidate) > self._selection_key(current):
                    selected[track_id] = candidate
        return [selected[track_id].hazard for track_id in sorted(selected)]

    def build_output(self, *, now_ns: int) -> AerialHazardArray:
        message = AerialHazardArray()
        message.header = Header(stamp=_time_from_ns(now_ns), frame_id=MAP_FRAME)
        message.hazards = self.selected_hazards(now_ns=now_ns)
        return message

    @staticmethod
    def _selection_key(candidate: StoredHazard) -> tuple[int, int, float, int]:
        state_rank = int(candidate.hazard.state == AerialHazard.CONFIRMED)
        quality = float(candidate.hazard.support_quality)
        return state_rank, candidate.last_seen_ns, quality, -candidate.source_order


class SupportHazardFusion(Node):
    """Validate and conservatively select typed hazards from dji1 and dji2."""

    def __init__(self) -> None:
        super().__init__("support_hazard_fusion")

        self.dji1_topic = str(
            self.declare_parameter("dji1_topic", DEFAULT_DJI1_TOPIC).value
        ).strip() or DEFAULT_DJI1_TOPIC
        self.dji2_topic = str(
            self.declare_parameter("dji2_topic", DEFAULT_DJI2_TOPIC).value
        ).strip() or DEFAULT_DJI2_TOPIC
        self.output_topic = str(
            self.declare_parameter("output_topic", DEFAULT_OUTPUT_TOPIC).value
        ).strip() or DEFAULT_OUTPUT_TOPIC
        self.stale_timeout_s = float(
            self.declare_parameter("stale_timeout_s", DEFAULT_STALE_TIMEOUT_S).value
        )
        self.max_covariance = float(
            self.declare_parameter("max_covariance", DEFAULT_MAX_COVARIANCE).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", DEFAULT_PUBLISH_RATE_HZ).value
        )
        if not math.isfinite(self.stale_timeout_s) or self.stale_timeout_s <= 0.0:
            raise ValueError("stale_timeout_s must be finite and greater than zero")
        if not math.isfinite(self.max_covariance) or self.max_covariance <= 0.0:
            raise ValueError("max_covariance must be finite and greater than zero")
        if not math.isfinite(self.publish_rate_hz) or self.publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and greater than zero")

        self._core = HazardFusionCore(
            stale_timeout_s=self.stale_timeout_s,
            max_covariance=self.max_covariance,
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
        self.create_subscription(
            AerialHazardArray,
            self.dji2_topic,
            partial(self._on_source, "dji2"),
            qos,
        )
        self.create_timer(1.0 / self.publish_rate_hz, self._on_timer)
        self.get_logger().info(
            "[support_hazard_fusion] Started: "
            f"inputs=({self.dji1_topic},{self.dji2_topic}), output={self.output_topic}, "
            f"stale_timeout_s={self.stale_timeout_s:.2f}, "
            f"max_covariance={self.max_covariance:.3f}, "
            f"publish_rate_hz={self.publish_rate_hz:.1f}"
        )

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
