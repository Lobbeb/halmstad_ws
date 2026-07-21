from __future__ import annotations

import rclpy
from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose

from lrs_halmstad.perception.dji0_to_ugv_forwarder import Dji0ToUgvForwarder
from lrs_halmstad.perception.support_hazard_fusion import HazardFusionCore
from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray


def _time(ns: int) -> Time:
    return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)


def _hazard(
    *,
    track_id: str = "track-1",
    state: int = AerialHazard.TENTATIVE,
    last_seen_ns: int = 10_000_000_000,
    ttl_s: float = 5.0,
    confidence: float = 0.8,
    support_quality: float = 0.5,
    covariance_value: float = 0.25,
) -> AerialHazard:
    detection = Detection3D()
    detection.header = Header(stamp=_time(last_seen_ns), frame_id="map")
    detection.id = track_id
    detection.bbox.center.position = Point(x=1.0, y=2.0, z=0.5)
    detection.bbox.size.x = 1.0
    detection.bbox.size.y = 1.0
    detection.bbox.size.z = 1.0

    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = "person"
    result.hypothesis.score = confidence
    result.pose.covariance = [
        covariance_value if index in (0, 7, 14, 21, 28, 35) else 0.0
        for index in range(36)
    ]
    detection.results = [result]

    hazard = AerialHazard()
    hazard.detection = detection
    hazard.source_uavs = ["support"]
    hazard.state = state
    hazard.first_seen = _time(last_seen_ns - 1_000_000_000)
    hazard.last_seen = _time(last_seen_ns)
    ttl_ns = int(ttl_s * 1_000_000_000)
    hazard.ttl = Duration(sec=ttl_ns // 1_000_000_000, nanosec=ttl_ns % 1_000_000_000)
    hazard.support_quality = support_quality
    hazard.provenance = "test"
    return hazard


def _array(*hazards: AerialHazard, stamp_ns: int = 10_000_000_000) -> AerialHazardArray:
    message = AerialHazardArray()
    message.header = Header(stamp=_time(stamp_ns), frame_id="map")
    message.hazards = list(hazards)
    return message


def _core(**kwargs) -> HazardFusionCore:
    return HazardFusionCore(stale_timeout_s=2.0, max_covariance=4.0, **kwargs)


def test_dji1_only():
    core = _core()
    core.replace_source("dji1", _array(_hazard()), now_ns=10_500_000_000)

    output = core.build_output(now_ns=10_500_000_000)

    assert [hazard.detection.id for hazard in output.hazards] == ["track-1"]


def test_dji2_only():
    core = _core()
    core.replace_source("dji2", _array(_hazard(track_id="track-2")), now_ns=10_500_000_000)

    output = core.build_output(now_ns=10_500_000_000)

    assert [hazard.detection.id for hazard in output.hazards] == ["track-2"]


def test_confirmed_is_preferred_over_fresher_tentative():
    core = _core()
    core.replace_source(
        "dji1",
        _array(_hazard(state=AerialHazard.TENTATIVE, last_seen_ns=11_000_000_000)),
        now_ns=11_500_000_000,
    )
    core.replace_source(
        "dji2",
        _array(
            _hazard(
                state=AerialHazard.CONFIRMED,
                last_seen_ns=10_500_000_000,
                support_quality=0.1,
            )
        ),
        now_ns=11_500_000_000,
    )

    output = core.build_output(now_ns=11_500_000_000)

    assert output.hazards[0].state == AerialHazard.CONFIRMED
    assert output.hazards[0].last_seen == _time(10_500_000_000)


def test_stale_input_is_rejected():
    core = _core()
    core.replace_source("dji1", _array(_hazard()), now_ns=13_000_000_001)

    assert core.build_output(now_ns=13_000_000_001).hazards == []


def test_expired_input_is_rejected():
    core = _core()
    core.replace_source(
        "dji1",
        _array(_hazard(ttl_s=1.0)),
        now_ns=10_500_000_000,
    )

    assert core.build_output(now_ns=11_500_000_001).hazards == []


def test_high_covariance_is_rejected():
    core = _core()
    core.replace_source(
        "dji1",
        _array(_hazard(covariance_value=5.0)),
        now_ns=10_500_000_000,
    )

    assert core.build_output(now_ns=10_500_000_000).hazards == []


def test_tie_is_deterministic_in_dji1_source_order():
    core = _core()
    dji1_hazard = _hazard(support_quality=0.5)
    dji2_hazard = _hazard(support_quality=0.5)
    dji1_hazard.provenance = "dji1"
    dji2_hazard.provenance = "dji2"
    core.replace_source("dji2", _array(dji2_hazard), now_ns=10_500_000_000)
    core.replace_source("dji1", _array(dji1_hazard), now_ns=10_500_000_000)

    output = core.build_output(now_ns=10_500_000_000)

    assert output.hazards[0].provenance == "dji1"


def test_empty_output_after_expiry():
    core = _core()
    core.replace_source(
        "dji1",
        _array(_hazard(ttl_s=1.0)),
        now_ns=10_500_000_000,
    )
    assert len(core.build_output(now_ns=10_500_000_000).hazards) == 1

    assert core.build_output(now_ns=11_000_000_001).hazards == []


class _FakePublisher:
    def __init__(self):
        self.last_message = None

    def publish(self, message):
        self.last_message = message


def test_forwarding_preserves_typed_message_without_mutation():
    message = _array(_hazard())
    publisher = _FakePublisher()
    forwarder = object.__new__(Dji0ToUgvForwarder)
    forwarder._out_hazard_pub = publisher

    forwarder._on_hazard(message)

    assert publisher.last_message is message
    assert publisher.last_message.header.frame_id == "map"
    assert publisher.last_message.hazards[0].detection.id == "track-1"


def test_legacy_forwarder_defaults_remain_enabled_and_typed_path_is_off():
    rclpy.init(args=[])
    node = Dji0ToUgvForwarder()
    try:
        assert node.hazard_forward_enable is False
        assert node.in_detection_topic == "/coord/dji0/leader_detection"
        assert node.out_detection_topic == "/coord/ugv/leader_detection"
        assert node._out_hazard_pub is None
    finally:
        node.destroy_node()
        rclpy.shutdown()
