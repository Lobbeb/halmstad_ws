from builtin_interfaces.msg import Time

import pytest

from lrs_halmstad.tools.synthetic_hazard_publisher import (
    DEFAULT_COVARIANCE,
    activity_state,
    build_hazard,
)
from lrs_halmstad_interfaces.msg import AerialHazard


def _build(**overrides):
    values = {
        "source_uav": "dji2",
        "class_name": "person",
        "stable_track_id": "track-7",
        "center_xyz": (1.0, 2.0, 3.0),
        "yaw_rad": 0.5,
        "dimensions_xyz": (0.5, 0.6, 1.7),
        "confidence": 0.8,
        "covariance": DEFAULT_COVARIANCE,
        "state": AerialHazard.CONFIRMED,
        "first_seen_ns": 10_000_000_000,
        "last_seen_ns": 11_000_000_000,
        "ttl_s": 4.0,
    }
    values.update(overrides)
    return build_hazard(**values)


def test_valid_message_construction():
    hazard = _build()

    assert hazard.detection.header.frame_id == "map"
    assert hazard.detection.header.stamp == Time(sec=11, nanosec=0)
    assert hazard.detection.bbox.size.x == pytest.approx(0.5)
    assert hazard.detection.bbox.size.y == pytest.approx(0.6)
    assert hazard.detection.bbox.size.z == pytest.approx(1.7)
    assert hazard.detection.results[0].hypothesis.class_id == "person"
    assert hazard.detection.results[0].hypothesis.score == pytest.approx(0.8)


def test_stable_id_state_and_source_are_preserved():
    hazard = _build()

    assert hazard.detection.id == "track-7"
    assert hazard.source_uavs == ["dji2"]
    assert hazard.state == AerialHazard.CONFIRMED


def test_covariance_is_nonzero_and_ttl_is_converted():
    hazard = _build(ttl_s=2.25)

    assert any(value != 0.0 for value in hazard.detection.results[0].pose.covariance)
    assert hazard.ttl.sec == 2
    assert hazard.ttl.nanosec == 250_000_000


@pytest.mark.parametrize(
    "field,value",
    [
        ("dimensions_xyz", (1.0, 0.0, 1.0)),
        ("dimensions_xyz", (-1.0, 1.0, 1.0)),
        ("dimensions_xyz", (1.0, 1.0)),
        ("confidence", -0.01),
        ("confidence", 1.01),
    ],
)
def test_invalid_dimensions_or_confidence_are_rejected(field, value):
    with pytest.raises(ValueError):
        _build(**{field: value})


def test_activity_is_deterministic_before_during_and_after_window():
    assert activity_state(9_999_999_999, 10_000_000_000, 2.0) == "inactive"
    assert activity_state(10_000_000_000, 10_000_000_000, 2.0) == "active"
    assert activity_state(11_999_999_999, 10_000_000_000, 2.0) == "active"
    assert activity_state(12_000_000_000, 10_000_000_000, 2.0) == "expired"
    assert activity_state(100, 0, 0.0) == "active"
