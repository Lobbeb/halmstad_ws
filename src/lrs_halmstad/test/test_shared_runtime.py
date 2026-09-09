"""Non-interactive compatibility checks: no ROS nodes, sockets or simulation."""
from collections import deque
from types import SimpleNamespace
from unittest.mock import Mock
import math

import pytest
from rclpy.time import Time

from lrs_halmstad.follow.follow_uav import FollowUav
from lrs_halmstad.perception.leader_estimator import LeaderEstimator
from lrs_halmstad.sim.omnet_metrics_bridge import OmnetMetricsBridge


def metrics_sink():
    names = ("simtime", "distance", "rssi", "snir", "per", "radio_dist", "pdr", "latency", "jitter")
    sink = SimpleNamespace(get_logger=lambda: Mock())
    for name in names:
        setattr(sink, "_pub_" + name, Mock())
    return sink


@pytest.mark.parametrize("line, distance, pdr", [
    ("1 20 -70 10 .1 30", 20.0, None),
    ("1 -70 10 .1 30 .9 .02 .003", None, .9),
    ("1 20 -70 10 .1 30 .9 .02 .003", 20.0, .9),
])
def test_metrics_formats_keep_fields_separate(line, distance, pdr):
    sink = metrics_sink()
    OmnetMetricsBridge._handle_line(sink, line)
    assert sink._pub_radio_dist.publish.call_args.args[0].data == 30.0
    assert sink._pub_rssi.publish.call_args.args[0].data == -70.0
    for name, expected in (("distance", distance), ("pdr", pdr)):
        actual = getattr(sink, "_pub_" + name).publish.call_args.args[0].data
        assert math.isnan(actual) if expected is None else actual == expected


@pytest.mark.parametrize("line", [
    "1 20 -70 10 .1 nan",
    "1 20 -70 10 .1 nan .9 .02 .003",
])
def test_missing_radio_never_uses_geometric_distance(line):
    sink = metrics_sink()
    OmnetMetricsBridge._handle_line(sink, line)
    assert sink._pub_distance.publish.call_args.args[0].data == 20.0
    assert math.isnan(sink._pub_radio_dist.publish.call_args.args[0].data)


@pytest.mark.parametrize("line", ["", "1 2 3", "1 20 bad 10 .1 30"])
def test_malformed_metrics_publish_nothing(line):
    sink = metrics_sink()
    OmnetMetricsBridge._handle_line(sink, line)
    for name, publisher in vars(sink).items():
        if name.startswith("_pub_"):
            publisher.publish.assert_not_called()


def follow_state(d_rate=0.0, z_rate=0.0):
    return SimpleNamespace(
        d_target=8.0, follow_z_offset_m=4.0,
        d_target_slew_mps=d_rate, follow_z_slew_mps=z_rate,
        _active_d_target=10.0, _active_follow_z_offset_m=7.0,
        _last_follow_slew_time=None,
    )


def test_zero_slew_preserves_immediate_updates_even_on_first_tick():
    state = follow_state()
    FollowUav._slew_follow_targets(state, Time(seconds=1))
    assert (state._active_d_target, state._active_follow_z_offset_m) == (8.0, 4.0)
    state.d_target, state.follow_z_offset_m = 5.0, 3.0
    FollowUav._slew_follow_targets(state, Time(seconds=1))
    assert (state._active_d_target, state._active_follow_z_offset_m) == (5.0, 3.0)


def test_slew_limits_changes_and_keeps_height_within_standoff():
    state = follow_state(2.0, 1.0)
    FollowUav._slew_follow_targets(state, Time(seconds=1))
    FollowUav._slew_follow_targets(state, Time(seconds=1.5))
    assert state._active_d_target == 9.0
    assert state._active_follow_z_offset_m == 6.5
    state.d_target, state.follow_z_offset_m = 3.0, 2.0
    for tick in range(2, 10):
        FollowUav._slew_follow_targets(state, Time(seconds=tick))
        assert state._active_follow_z_offset_m <= state._active_d_target


def test_radio_readiness_checks_freshness_warmup_samples_and_stability():
    state = SimpleNamespace(
        radio_range_fresh=lambda now: True,
        first_radio_range_stamp=Time(seconds=1),
        radio_range_warmup_s=0.0,
        radio_range_sample_count=1, radio_range_min_samples=1,
        radio_range_stability_max_delta_m=0.0,
        radio_range_stability_window=2,
        radio_range_window=deque([10.0, 10.5]),
    )
    assert LeaderEstimator.radio_range_ready(state, Time(seconds=1))
    state.radio_range_warmup_s = 2.0
    assert not LeaderEstimator.radio_range_ready(state, Time(seconds=2))
    state.radio_range_min_samples = 2
    assert not LeaderEstimator.radio_range_ready(state, Time(seconds=3))
    state.radio_range_sample_count = 2
    state.radio_range_stability_max_delta_m = .2
    assert not LeaderEstimator.radio_range_ready(state, Time(seconds=3))
    state.radio_range_window = deque([10.0, 10.1])
    assert LeaderEstimator.radio_range_ready(state, Time(seconds=3))
    state.radio_range_fresh = lambda now: False
    assert not LeaderEstimator.radio_range_ready(state, Time(seconds=3))
