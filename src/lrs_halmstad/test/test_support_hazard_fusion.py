from __future__ import annotations

import math

from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Point
from lrs_halmstad.perception.dji0_to_ugv_forwarder import Dji0ToUgvForwarder
from lrs_halmstad.perception.support_hazard_fusion import (
    HazardFusionCore,
    SupportHazardFusion,
)
from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray
import pytest
import rclpy
from std_msgs.msg import Header
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose


SECOND = 1_000_000_000


def _time(ns: int) -> Time:
    return Time(sec=ns // SECOND, nanosec=ns % SECOND)


def _hazard(
    *,
    track_id: str = "source-track-1",
    source: str = "dji1",
    class_id: str = "person",
    state: int = AerialHazard.TENTATIVE,
    last_seen_ns: int = 10 * SECOND,
    acquisition_ns: int | None = None,
    ttl_s: float = 5.0,
    confidence: float = 0.8,
    support_quality: float = 0.5,
    xy_variance: float = 0.25,
    x: float = 1.0,
    y: float = 2.0,
    size_x: float = 1.0,
    size_y: float = 1.0,
    size_z: float = 1.0,
) -> AerialHazard:
    if acquisition_ns is None:
        acquisition_ns = last_seen_ns
    detection = Detection3D()
    detection.header = Header(stamp=_time(acquisition_ns), frame_id="map")
    detection.id = track_id
    detection.bbox.center.position = Point(x=x, y=y, z=0.5)
    detection.bbox.center.orientation.w = 1.0
    detection.bbox.size.x = size_x
    detection.bbox.size.y = size_y
    detection.bbox.size.z = size_z

    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = class_id
    result.hypothesis.score = confidence
    result.pose.pose.position = Point(x=x, y=y, z=0.5)
    result.pose.pose.orientation.w = 1.0
    result.pose.covariance = [0.0] * 36
    for index in (0, 7):
        result.pose.covariance[index] = xy_variance
    for index in (14, 21, 28, 35):
        result.pose.covariance[index] = 0.25
    detection.results = [result]

    hazard = AerialHazard()
    hazard.detection = detection
    hazard.source_uavs = [source]
    hazard.state = state
    hazard.first_seen = _time(last_seen_ns - SECOND)
    hazard.last_seen = _time(last_seen_ns)
    ttl_ns = int(ttl_s * SECOND)
    hazard.ttl = Duration(sec=ttl_ns // SECOND, nanosec=ttl_ns % SECOND)
    hazard.support_quality = support_quality
    hazard.provenance = f"test_{source}"
    return hazard


def _array(*hazards: AerialHazard, stamp_ns: int | None = None) -> AerialHazardArray:
    message = AerialHazardArray()
    if stamp_ns is None:
        stamp_ns = max((_stamp(hazard.last_seen) for hazard in hazards), default=10 * SECOND)
    message.header = Header(stamp=_time(stamp_ns), frame_id="map")
    message.hazards = list(hazards)
    return message


def _stamp(stamp: Time) -> int:
    return int(stamp.sec) * SECOND + int(stamp.nanosec)


def _core(**kwargs) -> HazardFusionCore:
    defaults = {
        "stale_timeout_s": 2.0,
        "max_source_age_s": 2.0,
        "source_timeout_s": 2.0,
        "max_covariance": 4.0,
        "track_timeout_s": 3.0,
    }
    defaults.update(kwargs)
    return HazardFusionCore(**defaults)


def test_first_dji1_evidence_is_tentative_with_dji0_stable_id():
    core = _core()
    core.replace_source("dji1", _array(_hazard()), now_ns=10_500_000_000)

    output = core.build_output(now_ns=10_500_000_000)

    assert len(output.hazards) == 1
    assert output.hazards[0].detection.id == "dji0-hazard-000001"
    assert output.hazards[0].state == AerialHazard.TENTATIVE
    assert output.hazards[0].source_uavs == ["dji1"]


def test_dji2_only_logic_remains_available_for_opt_in_use():
    core = _core()
    core.replace_source(
        "dji2",
        _array(_hazard(track_id="dji2-track", source="dji2")),
        now_ns=10_500_000_000,
    )

    output = core.build_output(now_ns=10_500_000_000)

    assert len(output.hazards) == 1
    assert output.hazards[0].source_uavs == ["dji2"]


def test_same_object_from_two_uavs_associates_and_confirms():
    core = _core()
    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1", x=4.0, y=5.0)),
        now_ns=10_300_000_000,
    )
    core.replace_source(
        "dji2",
        _array(
            _hazard(
                track_id="other-source-id",
                source="dji2",
                last_seen_ns=10_100_000_000,
                x=4.2,
                y=5.1,
            )
        ),
        now_ns=10_300_000_000,
    )

    output = core.build_output(now_ns=10_300_000_000)

    assert len(output.hazards) == 1
    assert output.hazards[0].state == AerialHazard.CONFIRMED
    assert output.hazards[0].source_uavs == ["dji1", "dji2"]
    assert output.hazards[0].provenance == "rgbd_multi_confirmed"


def test_two_distinct_nearby_objects_remain_separate():
    core = _core(association_chi2_xy=5.991)
    first = _hazard(track_id="a", x=1.0, y=1.0, xy_variance=0.05, size_x=0.4, size_y=0.4)
    second = _hazard(track_id="b", x=2.0, y=1.0, xy_variance=0.05, size_x=0.4, size_y=0.4)

    core.replace_source("dji1", _array(first, second), now_ns=10_200_000_000)

    assert len(core.build_output(now_ns=10_200_000_000).hazards) == 2


def test_observations_outside_timestamp_window_do_not_associate():
    core = _core(association_time_window_s=0.75)
    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1", last_seen_ns=10 * SECOND)),
        now_ns=10_100_000_000,
    )
    core.replace_source(
        "dji2",
        _array(
            _hazard(
                track_id="dji2-id",
                source="dji2",
                last_seen_ns=11 * SECOND,
                x=1.05,
            )
        ),
        now_ns=11_100_000_000,
    )

    assert len(core.build_output(now_ns=11_100_000_000).hazards) == 2


def test_class_mismatch_does_not_associate():
    core = _core()
    core.replace_source("dji1", _array(_hazard(class_id="person", x=0.0)), now_ns=10_200_000_000)
    core.replace_source(
        "dji2",
        _array(_hazard(source="dji2", class_id="vehicle", x=5.0)),
        now_ns=10_200_000_000,
    )

    output = core.build_output(now_ns=10_200_000_000)

    assert len(output.hazards) == 2
    assert {hazard.detection.results[0].hypothesis.class_id for hazard in output.hazards} == {
        "person",
        "vehicle",
    }
    assert all(hazard.state == AerialHazard.TENTATIVE for hazard in output.hazards)


def test_stale_source_is_rejected():
    core = _core()
    core.replace_source("dji1", _array(_hazard()), now_ns=12 * SECOND + 1)

    assert core.build_output(now_ns=12 * SECOND + 1).hazards == []


def test_one_source_dropout_retains_track_and_complete_sources():
    core = _core()
    core.replace_source("dji1", _array(_hazard(source="dji1")), now_ns=10_200_000_000)
    core.replace_source(
        "dji2",
        _array(_hazard(track_id="dji2-id", source="dji2", x=1.1)),
        now_ns=10_200_000_000,
    )
    core.replace_source("dji2", _array(stamp_ns=10_400_000_000), now_ns=10_400_000_000)

    output = core.build_output(now_ns=10_400_000_000)

    assert len(output.hazards) == 1
    assert output.hazards[0].state == AerialHazard.TENTATIVE
    assert output.hazards[0].source_uavs == ["dji1", "dji2"]


def test_stable_track_identity_across_updates():
    core = _core()
    core.replace_source("dji1", _array(_hazard(x=1.0)), now_ns=10_100_000_000)
    first_id = core.build_output(now_ns=10_100_000_000).hazards[0].detection.id
    core.replace_source(
        "dji1",
        _array(_hazard(last_seen_ns=10_500_000_000, x=1.2)),
        now_ns=10_600_000_000,
    )

    assert core.build_output(now_ns=10_600_000_000).hazards[0].detection.id == first_id


def test_repeated_single_source_evidence_promotes_confirmation():
    core = _core(single_source_confirm_hits=2, confirmation_window_s=1.0)
    core.replace_source("dji1", _array(_hazard()), now_ns=10_100_000_000)
    assert core.build_output(now_ns=10_100_000_000).hazards[0].state == AerialHazard.TENTATIVE

    core.replace_source(
        "dji1",
        _array(_hazard(last_seen_ns=10_500_000_000, x=1.05)),
        now_ns=10_600_000_000,
    )

    assert core.build_output(now_ns=10_600_000_000).hazards[0].state == AerialHazard.CONFIRMED


def test_overlapping_incompatible_cross_uav_evidence_is_conflict():
    core = _core()
    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1", class_id="person", x=3.0, y=4.0)),
        now_ns=10_200_000_000,
    )
    core.replace_source(
        "dji2",
        _array(
            _hazard(
                track_id="vehicle-id",
                source="dji2",
                class_id="vehicle",
                x=3.2,
                y=4.0,
            )
        ),
        now_ns=10_200_000_000,
    )

    output = core.build_output(now_ns=10_200_000_000)

    assert len(output.hazards) == 2
    assert all(hazard.state == AerialHazard.CONFLICT for hazard in output.hazards)


def test_ttl_expiry_stops_output_and_track_timeout_cleans_store():
    core = _core(stale_timeout_s=5.0, track_timeout_s=2.0)
    core.replace_source(
        "dji1",
        _array(_hazard(ttl_s=1.0)),
        now_ns=10_500_000_000,
    )
    assert len(core.build_output(now_ns=10_500_000_000).hazards) == 1

    assert core.build_output(now_ns=11 * SECOND + 1).hazards == []
    assert core.track_count == 1
    core.build_output(now_ns=12 * SECOND + 1)
    assert core.track_count == 0


def test_deterministic_order_is_independent_of_input_array_order():
    hazards = (
        _hazard(track_id="z", x=10.0, size_x=0.5, size_y=0.5),
        _hazard(track_id="a", x=0.0, size_x=0.5, size_y=0.5),
    )
    left = _core()
    right = _core()
    left.replace_source("dji1", _array(*hazards), now_ns=10_200_000_000)
    right.replace_source("dji1", _array(*reversed(hazards)), now_ns=10_200_000_000)

    left_output = left.build_output(now_ns=10_200_000_000).hazards
    right_output = right.build_output(now_ns=10_200_000_000).hazards

    assert [hazard.detection.id for hazard in left_output] == [
        hazard.detection.id for hazard in right_output
    ]
    assert [hazard.detection.bbox.center.position.x for hazard in left_output] == [
        hazard.detection.bbox.center.position.x for hazard in right_output
    ]


def test_covariance_is_selected_verbatim_and_never_shrunk():
    core = _core()
    preferred = _hazard(
        source="dji1",
        confidence=1.0,
        support_quality=1.0,
        xy_variance=0.8,
    )
    secondary = _hazard(
        track_id="dji2-id",
        source="dji2",
        confidence=0.4,
        support_quality=0.0,
        xy_variance=0.2,
        x=1.1,
    )
    expected_covariance = list(preferred.detection.results[0].pose.covariance)
    core.replace_source("dji1", _array(preferred), now_ns=10_200_000_000)
    core.replace_source("dji2", _array(secondary), now_ns=10_200_000_000)

    fused = core.build_output(now_ns=10_200_000_000).hazards[0]

    assert list(fused.detection.results[0].pose.covariance) == expected_covariance
    assert fused.detection.results[0].pose.covariance[0] == pytest.approx(0.8)


def test_freshest_equivalent_source_is_selected_by_acquisition_age():
    core = _core()
    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1", last_seen_ns=10_000_000_000, x=1.0)),
        now_ns=10_500_000_000,
    )
    core.replace_source(
        "dji2",
        _array(
            _hazard(
                track_id="dji2-id",
                source="dji2",
                last_seen_ns=10_400_000_000,
                x=1.1,
            )
        ),
        now_ns=10_500_000_000,
    )

    fused = core.build_output(now_ns=10_500_000_000).hazards[0]

    assert fused.detection.bbox.center.position.x == pytest.approx(1.1)


def test_source_older_than_explicit_age_gate_is_rejected_and_counted():
    core = _core(max_source_age_s=0.5)

    accepted = core.replace_source(
        "dji1",
        _array(
            _hazard(
                last_seen_ns=10_500_000_000,
                acquisition_ns=10_000_000_000,
            )
        ),
        now_ns=10_600_000_000,
    )

    assert accepted == 0
    assert core.build_output(now_ns=10_600_000_000).hazards == []
    assert core.diagnostics.stale_source_rejections == 1


def test_high_quality_slightly_older_source_beats_fresher_uncertain_source():
    core = _core(
        quality_weight_confidence=0.25,
        quality_weight_freshness=0.25,
        quality_weight_uncertainty=0.40,
        quality_weight_view=0.10,
    )
    core.replace_source(
        "dji1",
        _array(
            _hazard(
                source="dji1",
                last_seen_ns=10_300_000_000,
                confidence=0.95,
                support_quality=1.0,
                xy_variance=0.05,
                x=1.0,
            )
        ),
        now_ns=10_600_000_000,
    )
    core.replace_source(
        "dji2",
        _array(
            _hazard(
                track_id="dji2-id",
                source="dji2",
                last_seen_ns=10_500_000_000,
                confidence=0.5,
                support_quality=0.3,
                xy_variance=3.0,
                x=1.1,
            )
        ),
        now_ns=10_600_000_000,
    )

    fused = core.build_output(now_ns=10_600_000_000).hazards[0]

    assert fused.detection.bbox.center.position.x == pytest.approx(1.0)
    assert fused.detection.results[0].pose.covariance[0] == pytest.approx(0.05)


def test_configured_communication_penalty_changes_source_choice():
    core = _core(
        quality_weight_confidence=0.2,
        quality_weight_freshness=0.1,
        quality_weight_uncertainty=0.1,
        quality_weight_view=0.1,
        quality_weight_communication=0.5,
        source_communication_penalty={"dji1": 0.8, "dji2": 0.0},
    )
    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1", x=1.0)),
        now_ns=10_200_000_000,
    )
    core.replace_source(
        "dji2",
        _array(_hazard(track_id="dji2-id", source="dji2", x=1.1)),
        now_ns=10_200_000_000,
    )

    fused = core.build_output(now_ns=10_200_000_000).hazards[0]

    assert fused.detection.bbox.center.position.x == pytest.approx(1.1)
    assert core.diagnostics.communication_adjusted_selections == 1


def test_no_live_metric_mode_uses_only_configured_source_quality():
    core = _core(
        quality_weight_confidence=0.2,
        quality_weight_freshness=0.1,
        quality_weight_uncertainty=0.1,
        quality_weight_view=0.1,
        quality_weight_communication=0.5,
        source_communication_quality={"dji1": 0.2, "dji2": 1.0},
    )
    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1", x=1.0)),
        now_ns=10_200_000_000,
    )
    core.replace_source(
        "dji2",
        _array(_hazard(track_id="dji2-id", source="dji2", x=1.1)),
        now_ns=10_200_000_000,
    )

    fused = core.build_output(now_ns=10_200_000_000).hazards[0]

    assert fused.detection.bbox.center.position.x == pytest.approx(1.1)
    assert not hasattr(core, "communication_subscription")


def test_source_timeout_drops_one_uav_while_other_remains_fresh():
    core = _core(source_timeout_s=0.5, single_source_confirm_hits=3)
    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1")),
        now_ns=10_100_000_000,
    )
    core.replace_source(
        "dji2",
        _array(_hazard(track_id="dji2-id", source="dji2", x=1.1)),
        now_ns=10_100_000_000,
    )
    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1", last_seen_ns=10_500_000_000, x=1.05)),
        now_ns=10_500_000_000,
    )

    output = core.build_output(now_ns=10_700_000_000)

    assert len(output.hazards) == 1
    assert output.hazards[0].source_uavs == ["dji1", "dji2"]
    assert core.diagnostics.source_timeouts == 1


def test_stale_second_source_does_not_keep_confirmation_alive():
    core = _core(max_source_age_s=0.5, single_source_confirm_hits=3)
    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1", last_seen_ns=10_400_000_000)),
        now_ns=10_500_000_000,
    )
    core.replace_source(
        "dji2",
        _array(
            _hazard(
                track_id="dji2-id",
                source="dji2",
                last_seen_ns=10_400_000_000,
                x=1.1,
            )
        ),
        now_ns=10_500_000_000,
    )
    assert core.build_output(now_ns=10_500_000_000).hazards[0].state == AerialHazard.CONFIRMED

    core.replace_source(
        "dji1",
        _array(_hazard(source="dji1", last_seen_ns=10_900_000_000, x=1.05)),
        now_ns=11_000_000_000,
    )
    fused = core.build_output(now_ns=11_000_000_000).hazards[0]

    assert fused.state == AerialHazard.TENTATIVE
    assert fused.source_uavs == ["dji1", "dji2"]


def test_conflict_clears_when_incompatible_source_becomes_stale():
    core = _core(max_source_age_s=0.5, single_source_confirm_hits=3)
    core.replace_source(
        "dji1",
        _array(
            _hazard(
                source="dji1",
                class_id="person",
                last_seen_ns=10_400_000_000,
                x=3.0,
                y=4.0,
            )
        ),
        now_ns=10_500_000_000,
    )
    core.replace_source(
        "dji2",
        _array(
            _hazard(
                track_id="vehicle-id",
                source="dji2",
                class_id="vehicle",
                last_seen_ns=10_400_000_000,
                x=3.2,
                y=4.0,
            )
        ),
        now_ns=10_500_000_000,
    )
    assert all(
        hazard.state == AerialHazard.CONFLICT
        for hazard in core.build_output(now_ns=10_500_000_000).hazards
    )

    core.replace_source(
        "dji1",
        _array(
            _hazard(
                source="dji1",
                class_id="person",
                last_seen_ns=10_900_000_000,
                x=3.0,
                y=4.0,
            )
        ),
        now_ns=11_000_000_000,
    )
    output = core.build_output(now_ns=11_000_000_000)

    assert len(output.hazards) == 1
    assert output.hazards[0].state == AerialHazard.TENTATIVE


def test_score_epsilon_uses_deterministic_source_order_for_near_tie():
    parameters = {
        "quality_weight_confidence": 0.0,
        "quality_weight_freshness": 1.0,
        "quality_weight_uncertainty": 0.0,
        "quality_weight_view": 0.0,
        "selection_score_epsilon": 0.1,
    }
    outputs = []
    for source_order in (("dji1", "dji2"), ("dji2", "dji1")):
        core = _core(**parameters)
        hazards = {
            "dji1": _hazard(
                source="dji1", last_seen_ns=10_300_000_000, x=1.0
            ),
            "dji2": _hazard(
                track_id="dji2-id",
                source="dji2",
                last_seen_ns=10_400_000_000,
                x=1.1,
            ),
        }
        for source_id in source_order:
            core.replace_source(
                source_id,
                _array(hazards[source_id]),
                now_ns=10_500_000_000,
            )
        outputs.append(
            core.build_output(now_ns=10_500_000_000)
            .hazards[0]
            .detection.bbox.center.position.x
        )

    assert outputs == pytest.approx([1.0, 1.0])


def test_no_fresh_source_diagnostic_is_bounded_per_transition():
    core = _core(max_source_age_s=0.5)
    core.replace_source(
        "dji1",
        _array(_hazard(last_seen_ns=10_000_000_000)),
        now_ns=10_100_000_000,
    )

    assert core.build_output(now_ns=10_600_000_000).hazards == []
    assert core.build_output(now_ns=10_700_000_000).hazards == []
    assert core.diagnostics.no_fresh_source_available == 1


def test_euclidean_fallback_associates_when_xy_covariance_is_singular():
    core = _core(association_max_distance_m=1.5)
    first = _hazard(source="dji1", xy_variance=0.0, x=1.0)
    second = _hazard(
        track_id="dji2-id",
        source="dji2",
        xy_variance=0.0,
        x=2.0,
    )
    core.replace_source("dji1", _array(first), now_ns=10_200_000_000)
    core.replace_source("dji2", _array(second), now_ns=10_200_000_000)

    assert len(core.build_output(now_ns=10_200_000_000).hazards) == 1


def test_optional_footprint_overlap_gate_prevents_association():
    core = _core(association_require_footprint_overlap=True)
    first = _hazard(source="dji1", x=1.0, size_x=0.2, size_y=0.2)
    second = _hazard(
        track_id="dji2-id",
        source="dji2",
        x=1.5,
        size_x=0.2,
        size_y=0.2,
    )
    core.replace_source("dji1", _array(first), now_ns=10_200_000_000)
    core.replace_source("dji2", _array(second), now_ns=10_200_000_000)

    assert len(core.build_output(now_ns=10_200_000_000).hazards) == 2


def test_track_store_is_bounded_without_unbounded_history():
    core = _core(max_track_count=2)
    hazards = [
        _hazard(track_id=f"track-{index}", x=float(index) * 10.0, size_x=0.5, size_y=0.5)
        for index in range(3)
    ]

    accepted = core.replace_source("dji1", _array(*hazards), now_ns=10_200_000_000)

    assert accepted == 2
    assert core.track_count == 2


def test_quality_weights_must_sum_to_one():
    with pytest.raises(ValueError, match="sum to one"):
        _core(
            quality_weight_confidence=0.5,
            quality_weight_freshness=0.5,
            quality_weight_uncertainty=0.5,
            quality_weight_view=0.5,
        )


def test_malformed_position_is_rejected_without_crashing():
    core = _core()
    malformed = _hazard()
    malformed.detection.bbox.center.position.x = math.nan

    core.replace_source("dji1", _array(malformed), now_ns=10_200_000_000)

    assert core.build_output(now_ns=10_200_000_000).hazards == []


def test_runtime_typed_dji2_subscription_defaults_off():
    rclpy.init(args=[])
    node = SupportHazardFusion()
    try:
        assert node.dji2_enable is False
    finally:
        node.destroy_node()
        rclpy.shutdown()


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
    assert publisher.last_message.hazards[0].detection.id == "source-track-1"


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
