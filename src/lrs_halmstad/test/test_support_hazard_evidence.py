from __future__ import annotations

import copy
import json

from builtin_interfaces.msg import Duration, Time
from geometry_msgs.msg import Point
from lrs_halmstad.tools.support_hazard_evidence import (
    DJI0_TOPIC,
    DJI1_TOPIC,
    DJI2_TOPIC,
    EvidenceCollector,
    EvidenceExpectations,
    UGV_TOPIC,
    write_evidence,
)
from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray
from std_msgs.msg import Header
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose


SECOND = 1_000_000_000


def _time(nanoseconds: int) -> Time:
    return Time(sec=nanoseconds // SECOND, nanosec=nanoseconds % SECOND)


def _hazard(
    *,
    track_id: str,
    source_uavs: list[str],
    state: int,
    x: float,
    class_id: str = 'hazard',
    covariance: float = 0.25,
    stamp_ns: int = 10 * SECOND,
) -> AerialHazard:
    detection = Detection3D()
    detection.header = Header(stamp=_time(stamp_ns), frame_id='map')
    detection.id = track_id
    detection.bbox.center.position = Point(x=x, y=5.0, z=0.5)
    detection.bbox.center.orientation.w = 1.0
    detection.bbox.size.x = 1.0
    detection.bbox.size.y = 1.0
    detection.bbox.size.z = 1.0
    result = ObjectHypothesisWithPose()
    result.hypothesis.class_id = class_id
    result.hypothesis.score = 0.9
    result.pose.pose = detection.bbox.center
    result.pose.covariance = [0.0] * 36
    for index in (0, 7, 14, 21, 28, 35):
        result.pose.covariance[index] = covariance
    detection.results = [result]
    hazard = AerialHazard()
    hazard.detection = detection
    hazard.source_uavs = source_uavs
    hazard.state = state
    hazard.first_seen = _time(stamp_ns)
    hazard.last_seen = _time(stamp_ns)
    hazard.ttl = Duration(sec=2)
    hazard.support_quality = 0.9
    hazard.provenance = 'test'
    return hazard


def _array(*hazards: AerialHazard, stamp_ns: int = 10_100_000_000) -> AerialHazardArray:
    message = AerialHazardArray()
    message.header = Header(stamp=_time(stamp_ns), frame_id='map')
    message.hazards = list(hazards)
    return message


def test_confirmation_flow_source_retention_selection_and_covariance():
    collector = EvidenceCollector()
    dji1 = _array(
        _hazard(
            track_id='dji1-track',
            source_uavs=['dji1'],
            state=AerialHazard.TENTATIVE,
            x=4.0,
        )
    )
    dji2 = _array(
        _hazard(
            track_id='dji2-track',
            source_uavs=['dji2'],
            state=AerialHazard.TENTATIVE,
            x=4.2,
        )
    )
    tentative = copy.deepcopy(dji1)
    tentative.hazards[0].detection.id = 'dji0-hazard-000001'
    confirmed = copy.deepcopy(tentative)
    confirmed.hazards[0].state = AerialHazard.CONFIRMED
    confirmed.hazards[0].source_uavs = ['dji1', 'dji2']

    collector.add(DJI1_TOPIC, dji1, 1)
    collector.add(DJI0_TOPIC, tentative, 2)
    collector.add(UGV_TOPIC, copy.deepcopy(tentative), 3)
    collector.add(DJI2_TOPIC, dji2, 4)
    collector.add(DJI0_TOPIC, confirmed, 5)
    collector.add(UGV_TOPIC, copy.deepcopy(confirmed), 6)

    summary = collector.summarize(
        EvidenceExpectations(
            require_dji2=True,
            expected_state=AerialHazard.CONFIRMED,
            expected_sources=('dji1', 'dji2'),
            expected_selected_source='dji1',
            require_confirmation_promotion=True,
        )
    )

    assert summary['status'] == 'pass'
    assert summary['typed_flow_complete'] is True
    assert summary['selected_source_latest'] == 'dji1'
    assert summary['confirmation_promotion_seen'] is True
    assert summary['covariance_preserved_from_source'] is True
    assert summary['dji0_to_ugv_forwarding_preserved'] is True


def test_conflict_and_expiry_are_packaged_without_navigation_claims():
    collector = EvidenceCollector()
    dji1_hazard = _hazard(
        track_id='dji1-person',
        source_uavs=['dji1'],
        state=AerialHazard.TENTATIVE,
        x=4.0,
    )
    dji2_hazard = _hazard(
        track_id='dji2-vehicle',
        source_uavs=['dji2'],
        state=AerialHazard.TENTATIVE,
        x=4.1,
        class_id='vehicle',
    )
    conflict_left = copy.deepcopy(dji1_hazard)
    conflict_left.state = AerialHazard.CONFLICT
    conflict_left.detection.id = 'dji0-hazard-000001'
    conflict_right = copy.deepcopy(dji2_hazard)
    conflict_right.state = AerialHazard.CONFLICT
    conflict_right.detection.id = 'dji0-hazard-000002'
    conflict = _array(conflict_left, conflict_right)
    empty = _array(stamp_ns=11 * SECOND)

    collector.add(DJI1_TOPIC, _array(dji1_hazard), 1)
    collector.add(DJI2_TOPIC, _array(dji2_hazard), 2)
    collector.add(DJI0_TOPIC, conflict, 3)
    collector.add(UGV_TOPIC, copy.deepcopy(conflict), 4)
    collector.add(DJI0_TOPIC, empty, 5)
    collector.add(UGV_TOPIC, copy.deepcopy(empty), 6)

    summary = collector.summarize(
        EvidenceExpectations(
            require_dji2=True,
            expected_state=AerialHazard.CONFLICT,
            minimum_hazard_count=2,
            require_conflict=True,
            require_expiry=True,
        )
    )

    assert summary['status'] == 'pass'
    assert summary['conflict_seen'] is True
    assert summary['expiry_empty_array_seen'] is True
    assert any('does not establish detector accuracy' in item for item in summary['limitations'])
    assert any('No closed-loop navigation' in item for item in summary['limitations'])


def test_sample_storage_is_bounded_and_overflow_fails_validation():
    collector = EvidenceCollector(max_samples_per_topic=1)
    message = _array(
        _hazard(
            track_id='dji1-track',
            source_uavs=['dji1'],
            state=AerialHazard.TENTATIVE,
            x=4.0,
        )
    )
    collector.add(DJI1_TOPIC, message, 1)
    collector.add(DJI1_TOPIC, message, 2)

    summary = collector.summarize(EvidenceExpectations())

    assert collector.total_counts[DJI1_TOPIC] == 2
    assert len(collector.samples[DJI1_TOPIC]) == 1
    assert summary['dropped_sample_counts'][DJI1_TOPIC] == 1
    assert 'sample_limit_exceeded' in summary['failures']


def test_evidence_writer_creates_machine_readable_outputs_and_figure(tmp_path):
    collector = EvidenceCollector()
    message = _array(
        _hazard(
            track_id='dji1-track',
            source_uavs=['dji1'],
            state=AerialHazard.TENTATIVE,
            x=4.0,
        )
    )
    collector.add(DJI1_TOPIC, message, 1)
    summary = collector.summarize(EvidenceExpectations())

    write_evidence(tmp_path, summary, collector.timeline_rows())

    assert json.loads((tmp_path / 'summary.json').read_text())['schema_version'] == 1
    assert (tmp_path / 'summary.csv').is_file()
    assert (tmp_path / 'timeline.csv').is_file()
    assert '<svg' in (tmp_path / 'timeline.svg').read_text()
