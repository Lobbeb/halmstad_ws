"""Publish one deterministic, map-frame aerial hazard for contract testing."""

from __future__ import annotations

import math
import sys
from typing import Sequence

import rclpy
from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Quaternion
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Header
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose

from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray


MAP_FRAME = "map"
DEFAULT_TOPIC = "/coord/support/dji1/aerial_hazards"
DEFAULT_COVARIANCE = [
    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.10, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.10, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.10,
]


def _time_from_ns(stamp_ns: int) -> Time:
    if stamp_ns < 0:
        raise ValueError("timestamp must be non-negative")
    msg = Time()
    msg.sec = stamp_ns // 1_000_000_000
    msg.nanosec = stamp_ns % 1_000_000_000
    return msg


def _duration_from_s(seconds: float) -> Duration:
    if not math.isfinite(seconds) or seconds < 0.0:
        raise ValueError("duration must be finite and non-negative")
    total_ns = int(round(seconds * 1_000_000_000.0))
    msg = Duration()
    msg.sec = total_ns // 1_000_000_000
    msg.nanosec = total_ns % 1_000_000_000
    return msg


def _yaw_quaternion(yaw_rad: float) -> Quaternion:
    if not math.isfinite(yaw_rad):
        raise ValueError("yaw must be finite")
    half_yaw = yaw_rad * 0.5
    msg = Quaternion()
    msg.z = math.sin(half_yaw)
    msg.w = math.cos(half_yaw)
    return msg


def _validate_dimensions(dimensions: Sequence[float]) -> tuple[float, float, float]:
    if len(dimensions) != 3:
        raise ValueError("dimensions must contain exactly three values")
    values = tuple(float(value) for value in dimensions)
    if not all(math.isfinite(value) and value > 0.0 for value in values):
        raise ValueError("dimensions must be finite and greater than zero")
    return values


def _validate_confidence(confidence: float) -> float:
    confidence = float(confidence)
    if not math.isfinite(confidence) or not 0.0 <= confidence <= 1.0:
        raise ValueError("confidence must be finite and in [0, 1]")
    return confidence


def _validate_covariance(covariance: Sequence[float]) -> list[float]:
    if len(covariance) != 36:
        raise ValueError("covariance must contain exactly 36 values")
    values = [float(value) for value in covariance]
    if not all(math.isfinite(value) for value in values):
        raise ValueError("covariance must contain only finite values")
    if not any(value != 0.0 for value in values):
        raise ValueError("covariance must be nonzero")
    if any(values[index] < 0.0 for index in (0, 7, 14, 21, 28, 35)):
        raise ValueError("covariance diagonal must be non-negative")
    return values


def activity_state(now_ns: int, start_ns: int, active_duration_s: float) -> str:
    """Return inactive, active, or expired using simulation-clock nanoseconds."""
    if now_ns < start_ns:
        return "inactive"
    if active_duration_s <= 0.0:
        return "active"
    duration_ns = int(round(active_duration_s * 1_000_000_000.0))
    return "active" if now_ns - start_ns < duration_ns else "expired"


def build_hazard(
    *,
    source_uav: str,
    class_name: str,
    stable_track_id: str,
    center_xyz: Sequence[float],
    yaw_rad: float,
    dimensions_xyz: Sequence[float],
    confidence: float,
    covariance: Sequence[float],
    state: int,
    first_seen_ns: int,
    last_seen_ns: int,
    ttl_s: float,
    support_quality: float = 1.0,
    provenance: str = "synthetic",
) -> AerialHazard:
    """Construct a fully populated, deterministic map-frame hazard message."""
    if not str(source_uav).strip():
        raise ValueError("source_uav must not be empty")
    if not str(class_name).strip():
        raise ValueError("class_name must not be empty")
    if not str(stable_track_id).strip():
        raise ValueError("stable_track_id must not be empty")
    if len(center_xyz) != 3:
        raise ValueError("center_xyz must contain exactly three values")
    center = tuple(float(value) for value in center_xyz)
    if not all(math.isfinite(value) for value in center):
        raise ValueError("center_xyz must contain only finite values")
    dimensions = _validate_dimensions(dimensions_xyz)
    confidence = _validate_confidence(confidence)
    covariance = _validate_covariance(covariance)
    if state not in (AerialHazard.TENTATIVE, AerialHazard.CONFIRMED, AerialHazard.CONFLICT):
        raise ValueError("state must be TENTATIVE, CONFIRMED, or CONFLICT")
    if first_seen_ns < 0 or last_seen_ns < 0 or last_seen_ns < first_seen_ns:
        raise ValueError("timestamps must be ordered and non-negative")
    if not math.isfinite(support_quality) or not 0.0 <= support_quality <= 1.0:
        raise ValueError("support_quality must be finite and in [0, 1]")
    ttl = _duration_from_s(float(ttl_s))

    detection = Detection3D()
    detection.header = Header(stamp=_time_from_ns(last_seen_ns), frame_id=MAP_FRAME)
    detection.id = str(stable_track_id)
    detection.bbox.center.position.x = center[0]
    detection.bbox.center.position.y = center[1]
    detection.bbox.center.position.z = center[2]
    detection.bbox.center.orientation = _yaw_quaternion(yaw_rad)
    detection.bbox.size.x = dimensions[0]
    detection.bbox.size.y = dimensions[1]
    detection.bbox.size.z = dimensions[2]

    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = str(class_name)
    result.hypothesis.score = confidence
    result.pose.pose.position.x = center[0]
    result.pose.pose.position.y = center[1]
    result.pose.pose.position.z = center[2]
    result.pose.pose.orientation = _yaw_quaternion(yaw_rad)
    result.pose.covariance = covariance
    detection.results = [result]

    hazard = AerialHazard()
    hazard.detection = detection
    hazard.source_uavs = [str(source_uav)]
    hazard.state = state
    hazard.first_seen = _time_from_ns(first_seen_ns)
    hazard.last_seen = _time_from_ns(last_seen_ns)
    hazard.ttl = ttl
    hazard.support_quality = float(support_quality)
    hazard.provenance = str(provenance)
    return hazard


class SyntheticHazardPublisher(Node):
    """Publish one configured hazard while it is active on the simulation clock."""

    def __init__(self) -> None:
        super().__init__("synthetic_hazard_publisher")

        self.declare_parameter("topic", DEFAULT_TOPIC)
        self.declare_parameter("source_uav", "dji1")
        self.declare_parameter("class_name", "hazard")
        self.declare_parameter("stable_track_id", "synthetic_hazard_0")
        self.declare_parameter("center_x", 0.0)
        self.declare_parameter("center_y", 0.0)
        self.declare_parameter("center_z", 0.0)
        self.declare_parameter("yaw", 0.0)
        self.declare_parameter("dimension_x", 1.0)
        self.declare_parameter("dimension_y", 1.0)
        self.declare_parameter("dimension_z", 1.0)
        self.declare_parameter("confidence", 0.9)
        self.declare_parameter("covariance", DEFAULT_COVARIANCE)
        self.declare_parameter("state", int(AerialHazard.CONFIRMED))
        self.declare_parameter("start_delay_s", 0.0)
        self.declare_parameter("publish_rate_hz", 1.0)
        self.declare_parameter("ttl_s", 5.0)
        self.declare_parameter("active_duration_s", 0.0)
        self.declare_parameter("publish_empty_after_active_duration", True)
        self.declare_parameter("support_quality", 1.0)
        self.declare_parameter("provenance", "synthetic")

        self.topic = str(self.get_parameter("topic").value).strip() or DEFAULT_TOPIC
        self.source_uav = str(self.get_parameter("source_uav").value).strip()
        self.class_name = str(self.get_parameter("class_name").value).strip()
        self.stable_track_id = str(self.get_parameter("stable_track_id").value).strip()
        self.center_xyz = tuple(
            float(self.get_parameter(name).value)
            for name in ("center_x", "center_y", "center_z")
        )
        self.yaw_rad = float(self.get_parameter("yaw").value)
        self.dimensions_xyz = tuple(
            float(self.get_parameter(name).value)
            for name in ("dimension_x", "dimension_y", "dimension_z")
        )
        self.confidence = float(self.get_parameter("confidence").value)
        self.covariance = list(self.get_parameter("covariance").value)
        self.state = int(self.get_parameter("state").value)
        self.start_delay_s = float(self.get_parameter("start_delay_s").value)
        publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.ttl_s = float(self.get_parameter("ttl_s").value)
        self.active_duration_s = float(self.get_parameter("active_duration_s").value)
        self.publish_empty_after_active_duration = bool(
            self.get_parameter("publish_empty_after_active_duration").value
        )
        self.support_quality = float(self.get_parameter("support_quality").value)
        self.provenance = str(self.get_parameter("provenance").value)

        if not math.isfinite(publish_rate_hz) or publish_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be finite and greater than zero")
        if not math.isfinite(self.start_delay_s) or self.start_delay_s < 0.0:
            raise ValueError("start_delay_s must be finite and non-negative")
        if not math.isfinite(self.active_duration_s) or self.active_duration_s < 0.0:
            raise ValueError("active_duration_s must be finite and non-negative")
        if not str(self.topic).startswith("/"):
            raise ValueError("topic must be an absolute ROS topic")

        self._start_ns: int | None = None
        self._first_seen_ns: int | None = None
        self.publisher = self.create_publisher(AerialHazardArray, self.topic, 10)
        self.create_timer(1.0 / publish_rate_hz, self._on_timer)

    def _on_timer(self) -> None:
        now_ns = int(self.get_clock().now().nanoseconds)
        if self._start_ns is None:
            self._start_ns = now_ns + int(round(self.start_delay_s * 1_000_000_000.0))

        state = activity_state(now_ns, self._start_ns, self.active_duration_s)
        if state == "inactive":
            return
        if state == "expired" and not self.publish_empty_after_active_duration:
            return

        message = AerialHazardArray()
        message.header = Header(stamp=_time_from_ns(now_ns), frame_id=MAP_FRAME)
        if state == "active":
            if self._first_seen_ns is None:
                self._first_seen_ns = now_ns
            message.hazards = [
                build_hazard(
                    source_uav=self.source_uav,
                    class_name=self.class_name,
                    stable_track_id=self.stable_track_id,
                    center_xyz=self.center_xyz,
                    yaw_rad=self.yaw_rad,
                    dimensions_xyz=self.dimensions_xyz,
                    confidence=self.confidence,
                    covariance=self.covariance,
                    state=self.state,
                    first_seen_ns=self._first_seen_ns,
                    last_seen_ns=now_ns,
                    ttl_s=self.ttl_s,
                    support_quality=self.support_quality,
                    provenance=self.provenance,
                )
            ]
        self.publisher.publish(message)


def main(args=None) -> None:
    if "--help" in sys.argv[1:] or "-h" in sys.argv[1:]:
        print(
            "Publishes a deterministic map-frame AerialHazardArray. "
            "Configure it with ROS parameters, for example:\n"
            "  --ros-args -p topic:=/coord/support/dji2/aerial_hazards "
            "-p active_duration_s:=10.0"
        )
        return
    rclpy.init(args=args)
    node = SyntheticHazardPublisher()
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
