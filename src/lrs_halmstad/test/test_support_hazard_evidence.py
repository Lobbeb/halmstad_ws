from __future__ import annotations

import copy
import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from builtin_interfaces.msg import Duration, Time
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point, TransformStamped
from lrs_halmstad.tools.support_hazard_evidence import (
    _clearing_mechanism,
    _global_robot_footprint,
    _request_costmap_snapshot,
    _set_aerial_layer,
    baseline_repeatability,
    build_parser,
    discrete_hausdorff_distance,
    DJI0_TOPIC,
    DJI1_TOPIC,
    DJI2_TOPIC,
    effective_hazard_geometry,
    EvidenceCollector,
    EvidenceExpectations,
    FeedbackRecord,
    GridSnapshot,
    load_nav2_inflation_config,
    nav2_cost_class,
    path_cost_exposure,
    path_hazard_metrics,
    PlannerEvidenceCollector,
    PlanRecord,
    PoseRecord,
    RequestedGoalRecord,
    relevant_costmap_delta,
    resolve_planar_tf_pose,
    RuntimeEvidenceCollector,
    RuntimeEvidenceNode,
    TransformRecord,
    MissionStatusRecord,
    segment_crosses_lethal_cost,
    settled_baseline_selection,
    summarize_runtime_evidence,
    trajectory_footprint_overlap,
    UGV_TOPIC,
    write_evidence,
    write_planner_evidence,
    write_runtime_evidence,
)
from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray
from lrs_halmstad.sim.simulation_uav_localization import load_calibration_points
from lrs_halmstad.tools.support_world_clearance import (
    MODEL_TOPIC, _model_world_pose, registration,
)
from nav2_msgs.msg import Costmap, CostmapUpdate
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from vision_msgs.msg import Detection3D, ObjectHypothesisWithPose


SECOND = 1_000_000_000
REPO_ROOT = Path(__file__).resolve().parents[3]


def test_runtime_footprint_diagnostic_detects_body_overlap_without_center_crossing():
    footprint = _global_robot_footprint(
        str(REPO_ROOT / 'src/lrs_halmstad/config/nav2_baylands_large_map.yaml')
    )
    geometry = {
        'center_x': 0.0, 'center_y': 0.0, 'yaw': 0.0,
        'effective_size_x': 4.0, 'effective_size_y': 4.0,
    }
    poses = [
        SimpleNamespace(x=3.0, y=0.0, yaw=0.0, received_ns=1, source_stamp_ns=1),
        SimpleNamespace(x=2.4, y=0.0, yaw=0.0, received_ns=2, source_stamp_ns=2),
    ]

    result = trajectory_footprint_overlap(poses, geometry, footprint)

    assert not path_hazard_metrics([(3.0, 0.0), (2.4, 0.0)], geometry)[
        'crosses_effective_hazard'
    ]
    assert result['overlapping_recorded_pose_count'] == 1
    assert result['first_overlap_received_ns'] == 2
    assert result['maximum_overlap_area_m2'] == pytest.approx(0.1472)
    assert result['overlap_events'][0]['source_stamp_ns'] == 2


def test_world_clearance_requires_named_model_pose_and_uses_distinct_control_points():
    unnamed = TFMessage(transforms=[TransformStamped()])
    assert _model_world_pose(unnamed) is None
    named = TransformStamped()
    named.header.frame_id = 'baylands'
    named.child_frame_id = 'a201_0000/robot'
    named.transform.translation.x = -146.0
    named.transform.translation.y = 91.0
    assert _model_world_pose(TFMessage(transforms=[named])) == (-146.0, 91.0)

    points = load_calibration_points(
        str(REPO_ROOT / 'maps/waypoints_baylands_groups.csv'), 'parkinglot_west'
    )
    distinct, _, residuals, held_out, margin = registration(points)
    assert len(distinct) == 21
    assert len(residuals) == len(held_out) == 21
    assert margin == max(max(value for _, value in residuals),
                         max(value for _, value in held_out))
    assert 1.2 < margin < 1.25


def test_world_evaluation_topic_has_no_operational_consumer():
    roots = (
        REPO_ROOT / 'src/lrs_halmstad/launch',
        REPO_ROOT / 'src/lrs_halmstad/lrs_halmstad/perception',
        REPO_ROOT / 'src/lrs_halmstad/lrs_halmstad/nav',
        REPO_ROOT / 'src/lrs_halmstad_nav_plugins/src',
    )
    for root in roots:
        for source in root.rglob('*'):
            if source.suffix in ('.py', '.cpp', '.hpp'):
                assert MODEL_TOPIC not in source.read_text(encoding='utf-8')
    evaluator = (REPO_ROOT / 'src/lrs_halmstad/lrs_halmstad/tools/'
                 'support_world_clearance.py').read_text(encoding='utf-8')
    assert 'create_subscription' not in evaluator
    assert 'create_publisher' not in evaluator
    assert 'rclpy.init' not in evaluator


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

    assert summary['status'] == 'pass', summary['failures']
    assert summary['typed_flow_complete'] is True
    assert summary['selected_source_latest'] == 'dji1'
    assert summary['confirmation_promotion_seen'] is True
    assert summary['covariance_preserved_from_source'] is True
    assert summary['dji0_to_ugv_forwarding_preserved'] is True


def test_forwarding_evidence_tolerates_independent_subscriber_sampling():
    collector = EvidenceCollector()
    forwarded = _array(_hazard(
        track_id='forwarded', source_uavs=['dji1'],
        state=AerialHazard.CONFIRMED, x=4.0,
    ))
    captured_only_at_ugv = _array(_hazard(
        track_id='missed-at-dji0', source_uavs=['dji1'],
        state=AerialHazard.CONFIRMED, x=5.0,
    ))
    collector.add(DJI0_TOPIC, forwarded, 1)
    collector.add(UGV_TOPIC, captured_only_at_ugv, 2)
    collector.add(UGV_TOPIC, copy.deepcopy(forwarded), 3)

    assert collector._forwarding_preserved() is True
    assert collector._forwarding_match_count() == 1


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


def test_effective_geometry_applies_covariance_to_both_sides():
    hazard = _hazard(
        track_id='route-hazard',
        source_uavs=['dji1'],
        state=AerialHazard.CONFIRMED,
        x=-72.0,
        covariance=0.25,
    )
    hazard.detection.bbox.size.x = 2.0
    hazard.detection.bbox.size.y = 2.0

    geometry = effective_hazard_geometry(hazard, covariance_sigma_scale=2.0)

    assert geometry['uncertainty_per_side_m'] == 1.0
    assert geometry['effective_size_x'] == 4.0
    assert geometry['effective_size_y'] == 4.0


def test_plan_geometry_detects_route_crossing_and_material_change():
    geometry = {
        'center_x': 0.0, 'center_y': 0.0, 'yaw': 0.0,
        'effective_size_x': 4.0, 'effective_size_y': 4.0,
    }
    baseline = [(0.0, 5.0), (0.0, -5.0)]
    detour = [(0.0, 5.0), (3.0, 3.0), (3.0, -3.0), (0.0, -5.0)]

    assert path_hazard_metrics(baseline, geometry)['crosses_effective_hazard']
    assert not path_hazard_metrics(detour, geometry)['crosses_effective_hazard']
    assert discrete_hausdorff_distance(baseline, detour) >= 3.0


def test_costmap_full_and_update_are_distinguished_and_reconstructed():
    collector = PlannerEvidenceCollector()
    full = Costmap()
    full.metadata.resolution = 1.0
    full.metadata.size_x = 5
    full.metadata.size_y = 5
    full.data = [0] * 25
    collector.add_full_costmap(full, 10)
    update = CostmapUpdate(x=2, y=2, size_x=1, size_y=1, data=[254])
    collector.add_costmap_update(update, 20)
    geometry = {
        'center_x': 2.5, 'center_y': 2.5, 'yaw': 0.0,
        'effective_size_x': 1.0, 'effective_size_y': 1.0,
    }

    delta = relevant_costmap_delta(collector.costmaps[0], collector.costmaps[1], geometry)

    assert collector.costmap_full_count == 1
    assert collector.costmap_update_count == 1
    assert delta['affected_cells'] == 1
    assert delta['lethal_cells'] == 1
    assert delta['relevant_cost_values'] == [254]
    assert delta['affected_cell_values'][0]['current_cost'] == 254
    assert segment_crosses_lethal_cost([(0.0, 2.5), (5.0, 2.5)], collector.costmaps[1])


def test_runtime_costmap_capture_crops_large_grid_and_maps_global_updates():
    geometry = {
        'center_x': 50.5, 'center_y': 50.5, 'yaw': 0.0,
        'effective_size_x': 2.0, 'effective_size_y': 2.0,
    }
    collector = RuntimeEvidenceCollector(
        crop_geometry=geometry, crop_margin_m=1.0,
    )
    full = Costmap()
    full.metadata.resolution = 1.0
    full.metadata.size_x = 100
    full.metadata.size_y = 100
    full.data = [0] * 10_000
    collector.add_full_costmap(full, 10)
    baseline = collector.costmaps[-1]
    collector.add_costmap_update(
        CostmapUpdate(x=50, y=50, size_x=1, size_y=1, data=[254]), 20
    )

    assert baseline.size_x == 5
    assert baseline.size_y == 5
    assert len(baseline.data) == 25
    delta = relevant_costmap_delta(baseline, collector.costmaps[-1], geometry)
    assert delta['lethal_cells'] == 1
    assert collector.costmaps[-1].source_kind == 'update_hazard_crop'


def test_runtime_evidence_uses_explicit_nonzero_simulation_clock():
    fake_node = SimpleNamespace(_runtime_sim_time_ns=0)
    clock = Clock(clock=_time(12_345_000_000))

    RuntimeEvidenceNode._on_runtime_clock(fake_node, clock)

    assert RuntimeEvidenceNode._evidence_now_ns(fake_node) == 12_345_000_000
    collector = RuntimeEvidenceCollector()
    collector.add_full_costmap(Costmap(), 0)
    assert not collector.costmaps


def test_costmap_delta_distinguishes_lethal_core_and_graded_inflation_halo():
    baseline = GridSnapshot(1, 'full', 1.0, 7, 7, 0.0, 0.0, bytes([0] * 49))
    data = bytearray([0] * 49)
    data[3 * 7 + 3] = 254
    data[3 * 7 + 4] = 100
    candidate = GridSnapshot(2, 'update', 1.0, 7, 7, 0.0, 0.0, bytes(data))
    geometry = {
        'center_x': 3.5, 'center_y': 3.5, 'yaw': 0.0,
        'effective_size_x': 1.0, 'effective_size_y': 1.0,
    }

    delta = relevant_costmap_delta(baseline, candidate, geometry, 2.0)

    assert delta['hazard_footprint_lethal_cells'] == 1
    assert delta['inflation_halo_nonzero_cells'] == 1
    assert delta['analysis_region_affected_cells'] == 2
    assert {item['region'] for item in delta['affected_cell_values']} == {
        'covariance_footprint', 'inflation_halo'
    }

    exposure = path_cost_exposure(
        [(3.5, 3.5), (5.5, 3.5)], candidate, geometry,
        {**geometry, 'effective_size_x': 5.0, 'effective_size_y': 5.0}, baseline,
    )
    assert exposure['crosses_lethal_costmap_cell'] is True
    assert exposure['graded_inflated_cost_unique_cell_count'] == 1
    assert exposure['graded_inflated_cost_path_length_m'] > 0.0


def test_nav2_cost_classification_distinguishes_253_254_and_255():
    assert nav2_cost_class(0) == 'free'
    assert nav2_cost_class(253) == 'graded'
    assert nav2_cost_class(254) == 'lethal'
    assert nav2_cost_class(255) == 'no_information'

    for cost, expected in ((253, False), (254, True), (255, False)):
        snapshot = GridSnapshot(1, 'full', 1.0, 1, 1, 0.0, 0.0, bytes([cost]))
        assert segment_crosses_lethal_cost(
            [(0.1, 0.5), (0.9, 0.5)], snapshot
        ) is expected


def test_no_information_startup_transition_is_not_an_aerial_effect():
    baseline = GridSnapshot(1, 'full', 1.0, 1, 1, 0.0, 0.0, bytes([255]))
    settled = GridSnapshot(2, 'update', 1.0, 1, 1, 0.0, 0.0, bytes([0]))
    geometry = {
        'center_x': 0.5, 'center_y': 0.5, 'yaw': 0.0,
        'effective_size_x': 1.0, 'effective_size_y': 1.0,
    }

    delta = relevant_costmap_delta(baseline, settled, geometry)

    assert delta['analysis_region_raw_changed_cells'] == 1
    assert delta['no_information_transition_cells'] == 1
    assert delta['analysis_region_affected_cells'] == 0
    assert delta['lethal_cells'] == 0


def test_baseline_selection_requires_consecutive_known_stable_regions():
    geometry = {
        'center_x': 0.5, 'center_y': 0.5, 'yaw': 0.0,
        'effective_size_x': 1.0, 'effective_size_y': 1.0,
    }
    unknown = GridSnapshot(1, 'full', 1.0, 1, 1, 0.0, 0.0, bytes([255]))
    known_once = GridSnapshot(2, 'update', 1.0, 1, 1, 0.0, 0.0, bytes([0]))
    known_twice = GridSnapshot(3, 'update', 1.0, 1, 1, 0.0, 0.0, bytes([0]))

    missing, incomplete = settled_baseline_selection(
        [unknown, known_once], geometry, 0.0
    )
    selected, complete = settled_baseline_selection(
        [unknown, known_once, known_twice], geometry, 0.0
    )

    assert missing is None
    assert incomplete['status'] == 'unsettled'
    assert selected == known_twice
    assert complete['status'] == 'settled'
    assert complete['maximum_consecutive_stable_snapshots'] == 2
    assert complete['observations'][0]['no_information_cell_count'] == 1


def test_aerial_layer_parameter_client_uses_jazzy_wait_for_services():
    class CompletedFuture:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace(results=[SimpleNamespace(successful=True)])

    class ParameterClient:
        def __init__(self):
            self.wait_called = False

        def wait_for_services(self, timeout_sec):
            self.wait_called = True
            return timeout_sec == 2.0

        def set_parameters(self, parameters):
            assert parameters[0].name == 'aerial_support_layer.enabled'
            assert parameters[0].value is True
            return CompletedFuture()

    client = ParameterClient()
    node = SimpleNamespace(layer_parameters=client)

    assert _set_aerial_layer(node, True, 2.0)
    assert client.wait_called


def test_costmap_service_seeds_a_full_snapshot_for_deterministic_settling():
    message = Costmap()
    message.metadata.resolution = 1.0
    message.metadata.size_x = 1
    message.metadata.size_y = 1
    message.data = [0]

    class CompletedFuture:
        def done(self):
            return True

        def result(self):
            return SimpleNamespace(map=message)

    class CostmapClient:
        def wait_for_service(self, timeout_sec):
            return timeout_sec == 2.0

        def call_async(self, request):
            return CompletedFuture()

    collector = PlannerEvidenceCollector()
    node = SimpleNamespace(
        costmap_client=CostmapClient(),
        collector=collector,
        get_clock=lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=10)),
    )

    assert _request_costmap_snapshot(node, 2.0)
    assert collector.latest_costmap().source_kind == 'service'
    assert collector.latest_costmap().data == bytes([0])


def test_nav2_config_provenance_and_inflation_are_loaded_from_yaml(tmp_path):
    config = tmp_path / 'nav2.yaml'
    config.write_text("""
global_costmap:
  global_costmap:
    ros__parameters:
      footprint: "[[0.55, 0.45], [0.55, -0.45], [-0.55, -0.45], [-0.55, 0.45]]"
      plugins: [aerial_support_layer, inflation_layer]
      aerial_support_layer:
        topic: /coord/ugv/aerial_hazards
        target_frame: map
        min_confidence: 0.35
        max_observation_age_s: 1.0
        default_ttl_s: 2.0
        max_xy_variance_m2: 1.0
        confirmed_cost: 254
        tentative_cost: 200
        conflict_cost: 220
        covariance_sigma_scale: 2.0
        min_footprint_size_m: 0.3
        subscription_depth: 10
      inflation_layer:
        inflation_radius: 1.25
        cost_scaling_factor: 3.5
local_costmap:
  local_costmap:
    ros__parameters:
      footprint: "[[0.55, 0.45], [0.55, -0.45], [-0.55, -0.45], [-0.55, 0.45]]"
      global_frame: odom
      robot_base_frame: base_link
      plugins: [aerial_support_layer, inflation_layer]
      aerial_support_layer:
        topic: /coord/ugv/aerial_hazards
        target_frame: odom
        min_confidence: 0.35
        max_observation_age_s: 1.0
        default_ttl_s: 2.0
        max_xy_variance_m2: 1.0
        confirmed_cost: 254
        tentative_cost: 200
        conflict_cost: 220
        covariance_sigma_scale: 2.0
        min_footprint_size_m: 0.3
        subscription_depth: 10
      inflation_layer:
        inflation_radius: 0.8
        cost_scaling_factor: 4.0
controller_server:
  ros__parameters:
    general_goal_checker:
      xy_goal_tolerance: 0.75
      yaw_goal_tolerance: 1.25
      plugin: nav2_controller::SimpleGoalChecker
      stateful: true
""")

    loaded = load_nav2_inflation_config(config)

    assert loaded['source_yaml'] == str(config.resolve())
    assert len(loaded['source_sha256']) == 64
    assert loaded['inflation_radius_m'] == 1.25
    assert loaded['cost_scaling_factor'] == 3.5
    assert loaded['aerial_min_confidence'] == 0.35
    assert loaded['global_footprint_padding_m'] == pytest.approx(0.01)
    assert loaded['global_footprint_padded'][0] == pytest.approx([0.56, 0.46])
    assert loaded['local_costmap_frame'] == 'odom'
    assert loaded['local_aerial_layer_configured'] is True
    assert loaded['local_aerial_target_frame'] == 'odom'
    assert loaded['goal_checker_xy_tolerance_m'] == 0.75
    assert loaded['goal_checker_yaw_tolerance_rad'] == 1.25
    assert loaded['goal_checker_plugin'] == 'nav2_controller::SimpleGoalChecker'
    assert loaded['goal_checker_stateful'] is True


def test_baylands_global_inflation_is_derived_from_the_actual_config():
    loaded = load_nav2_inflation_config(
        REPO_ROOT / 'src/lrs_halmstad/config/nav2_baylands_large_map.yaml'
    )

    assert loaded['inflation_radius_m'] == 0.95
    assert loaded['cost_scaling_factor'] == 3.0
    assert loaded['aerial_covariance_sigma_scale'] == 2.0
    assert loaded['global_costmap_rolling_window'] is False
    assert loaded['global_costmap_resolution_m'] == 0.2
    assert loaded['local_aerial_layer_configured'] is True
    assert loaded['local_aerial_target_frame'] == 'odom'
    assert loaded['goal_checker_xy_tolerance_m'] == 1.0
    assert loaded['goal_checker_yaw_tolerance_rad'] == 2.5
    assert loaded['goal_checker_stateful'] is True


def test_planner_cli_keeps_nav2_config_separate_from_ros_params_file():
    parser = build_parser()
    parsed, ros_args = parser.parse_known_args([
        'planner-live', '--scenario', 'baseline', '--map', '/tmp/map.yaml',
        '--nav2-config', '/tmp/nav2.yaml', '--start-x', '0', '--start-y', '0',
        '--goal-x', '1', '--goal-y', '1', '--hazard-x', '0.5', '--hazard-y', '0.5',
        '--output', '/tmp/evidence', '--ros-args', '--params-file', '/tmp/ros.yaml',
    ])

    assert parsed.nav2_config == Path('/tmp/nav2.yaml')
    assert ros_args == ['--ros-args', '--params-file', '/tmp/ros.yaml']


def test_runtime_cli_keeps_full_mission_inputs_separate_from_ros_args():
    parser = build_parser()
    parsed, ros_args = parser.parse_known_args([
        'runtime-live', '--scenario', 'valid', '--map', '/tmp/map.yaml',
        '--nav2-config', '/tmp/nav2.yaml', '--baseline-evidence', '/tmp/baseline',
        '--start-x', '0', '--start-y', '5', '--goal-x', '0', '--goal-y', '-5',
        '--hazard-x', '0', '--hazard-y', '0', '--output', '/tmp/runtime',
        '--ros-args', '-p', 'use_sim_time:=true',
    ])

    assert parsed.scenario == 'valid'
    assert parsed.baseline_evidence == Path('/tmp/baseline')
    assert ros_args == ['--ros-args', '-p', 'use_sim_time:=true']


def test_baseline_repeatability_requires_all_results_and_stable_geometry():
    stable = [
        PlanRecord(f'baseline_repeat_{index}', index, index + 1, 0.01, 0, '',
                   ((0.0, 0.0), (1.0, 1.0)))
        for index in range(1, 4)
    ]

    result = baseline_repeatability(stable, required_count=3, tolerance_m=0.05)
    missing = baseline_repeatability(stable[:2], required_count=3, tolerance_m=0.05)
    changed = list(stable)
    changed[-1] = PlanRecord(
        'baseline_repeat_3', 3, 4, 0.01, 0, '', ((0.0, 0.0), (2.0, 1.0))
    )

    assert result['status'] == 'pass'
    assert result['successful_result_count'] == 3
    assert result['timing_is_acceptance_criterion'] is False
    assert missing['status'] == 'fail'
    assert 'baseline_request_count_incomplete' in missing['failures']
    assert baseline_repeatability(
        changed, required_count=3, tolerance_m=0.05
    )['status'] == 'fail'


def test_missing_typed_evidence_cannot_pass():
    summary = EvidenceCollector().summarize(EvidenceExpectations())

    assert summary['status'] == 'fail'
    assert 'typed_flow_incomplete' in summary['failures']


def test_hazard_timeline_records_timestamps_quality_covariance_and_geometry():
    collector = PlannerEvidenceCollector()
    hazard = _hazard(
        track_id='route-hazard', source_uavs=['dji1'],
        state=AerialHazard.CONFIRMED, x=-72.0,
    )
    hazard.detection.bbox.size.x = 2.0
    hazard.detection.bbox.size.y = 2.0
    collector.add(DJI1_TOPIC, _array(hazard), 11 * SECOND)

    row = collector.hazard_rows()[0]

    assert row['track_id'] == 'route-hazard'
    assert row['acquisition_ns'] == 10 * SECOND
    assert row['publication_ns'] == 10_100_000_000
    assert row['observation_ns'] == 10 * SECOND
    assert row['support_quality'] == 0.9
    assert row['age_at_receipt_s'] == 1.0
    assert row['ttl_s'] == 2.0
    assert row['covariance_x_m2'] == 0.25
    assert row['effective_size_x_m'] == 4.0


def test_planner_writer_creates_structured_plans_timelines_and_overlay(tmp_path):
    geometry = {
        'center_x': 0.0, 'center_y': 0.0, 'yaw': 0.0,
        'nominal_size_x': 2.0, 'nominal_size_y': 2.0,
        'effective_size_x': 4.0, 'effective_size_y': 4.0,
    }
    plan = PlanRecord(
        label='baseline', requested_ns=1, received_ns=2,
        planning_time_s=0.01, error_code=0, error_message='',
        points=((0.0, 5.0), (0.0, -5.0)),
    )
    summary = {
        'schema_version': 2,
        'hazard_geometry': geometry,
        'planner': {'plans': [{
            'label': plan.label,
            'points': [list(point) for point in plan.points],
        }]},
    }

    write_planner_evidence(
        tmp_path,
        summary,
        [{'topic': DJI1_TOPIC, 'snapshot_kind': 'hazard'}],
        [{'label': 'baseline', 'affected_cells': 0}],
    )

    assert json.loads((tmp_path / 'summary.json').read_text())['schema_version'] == 2
    assert json.loads((tmp_path / 'plans.json').read_text())[0]['label'] == 'baseline'
    assert 'snapshot_kind' in (tmp_path / 'hazard_timeline.csv').read_text()
    assert 'affected_cells' in (tmp_path / 'costmap_timeline.csv').read_text()
    assert '<svg' in (tmp_path / 'planner_overlay.svg').read_text()


def _runtime_grid(received_ns, *, marked=False):
    size = 12
    values = [0] * (size * size)
    if marked:
        for row in range(size):
            for column in range(size):
                x = -6.0 + (column + 0.5)
                y = -6.0 + (row + 0.5)
                if abs(x) <= 2.0 and abs(y) <= 2.0:
                    values[row * size + column] = 254
                elif abs(x) <= 3.0 and abs(y) <= 3.0:
                    values[row * size + column] = 120
    return GridSnapshot(
        received_ns, 'full', 1.0, size, size, -6.0, -6.0, bytes(values)
    )


def _tf(
    received_ns,
    source_stamp_ns,
    parent,
    child,
    x,
    y=0.0,
    yaw=0.0,
    *,
    is_static=False,
):
    return TransformRecord(
        received_ns, source_stamp_ns, parent, child, x, y, yaw, is_static
    )


def test_common_time_tf_resolver_uses_exact_same_timestamp_chain():
    result = resolve_planar_tf_pose(
        [
            _tf(9 * SECOND, 10 * SECOND, 'map', 'odom', 2.0),
            _tf(9 * SECOND, 10 * SECOND, 'odom', 'base_link', 3.0),
        ],
        parent_frame='map', child_frame='base_link', at_received_ns=10 * SECOND,
    )

    assert result['status'] == 'resolved'
    assert result['common_time_ns'] == 10 * SECOND
    assert result['x'] == 5.0
    assert [item['method'] for item in result['transform_chain']] == ['exact', 'exact']


def test_common_time_tf_resolver_interpolates_future_stamped_map_to_odom():
    result = resolve_planar_tf_pose(
        [
            _tf(9 * SECOND, 9 * SECOND, 'map', 'odom', 0.0),
            _tf(9_500_000_000, 11 * SECOND, 'map', 'odom', 2.0),
            _tf(9_900_000_000, 10 * SECOND, 'odom', 'base_link', 10.0),
        ],
        parent_frame='map', child_frame='base_link', at_received_ns=10 * SECOND,
    )

    assert result['status'] == 'resolved'
    assert result['common_time_ns'] == 10 * SECOND
    assert result['x'] == 11.0
    assert result['transform_chain'][0]['method'] == 'interpolated'
    assert result['transform_chain'][0]['upper_source_stamp_ns'] == 11 * SECOND
    assert result['transform_chain'][1]['method'] == 'exact'


def test_common_time_tf_resolver_handles_mismatched_overlapping_histories():
    result = resolve_planar_tf_pose(
        [
            _tf(8 * SECOND, 8 * SECOND, 'map', 'odom', 0.0),
            _tf(9 * SECOND, 10 * SECOND, 'map', 'odom', 2.0),
            _tf(9 * SECOND, 9 * SECOND, 'odom', 'base_link', 10.0),
            _tf(9_500_000_000, 11 * SECOND, 'odom', 'base_link', 12.0),
        ],
        parent_frame='map', child_frame='base_link', at_received_ns=10 * SECOND,
    )

    assert result['status'] == 'resolved'
    assert result['common_time_ns'] == 10 * SECOND
    assert result['x'] == 13.0
    assert [item['method'] for item in result['transform_chain']] == [
        'exact', 'interpolated'
    ]


def test_common_time_tf_resolver_interpolates_all_bracketing_samples():
    result = resolve_planar_tf_pose(
        [
            _tf(8 * SECOND, 8 * SECOND, 'map', 'odom', 0.0),
            _tf(8_500_000_000, 10 * SECOND, 'map', 'odom', 2.0),
            _tf(8 * SECOND, 8 * SECOND, 'odom', 'base_link', 2.0),
            _tf(8_500_000_000, 10 * SECOND, 'odom', 'base_link', 4.0),
        ],
        parent_frame='map', child_frame='base_link', at_received_ns=9 * SECOND,
    )

    assert result['status'] == 'resolved'
    assert result['common_time_ns'] == 9 * SECOND
    assert result['x'] == 4.0
    assert all(item['method'] == 'interpolated' for item in result['transform_chain'])


def test_common_time_tf_resolver_rejects_histories_without_common_time():
    result = resolve_planar_tf_pose(
        [
            _tf(8 * SECOND, 8 * SECOND, 'map', 'odom', 1.0),
            _tf(9 * SECOND, 9 * SECOND, 'odom', 'base_link', 2.0),
        ],
        parent_frame='map', child_frame='base_link', at_received_ns=10 * SECOND,
    )

    assert result['status'] == 'unavailable'
    assert result['failure_reason'] == 'no_common_tf_time'


def test_common_time_tf_resolver_rejects_stale_and_wide_interpolation():
    stale = resolve_planar_tf_pose(
        [_tf(8 * SECOND, 8 * SECOND, 'map', 'base_link', 1.0)],
        parent_frame='map', child_frame='base_link', at_received_ns=10 * SECOND,
    )
    wide = resolve_planar_tf_pose(
        [
            _tf(8 * SECOND, 8 * SECOND, 'map', 'odom', 0.0),
            _tf(9 * SECOND, 12 * SECOND, 'map', 'odom', 4.0),
            _tf(9 * SECOND, 10 * SECOND, 'odom', 'base_link', 1.0),
        ],
        parent_frame='map', child_frame='base_link', at_received_ns=10 * SECOND,
    )

    assert stale['status'] == 'unavailable'
    assert stale['failure_reason'] == 'common_tf_time_too_old'
    assert wide['status'] == 'unavailable'
    assert wide['failure_reason'] == 'interpolation_gap_too_large'


def test_common_time_tf_resolver_interpolates_yaw_across_wraparound():
    result = resolve_planar_tf_pose(
        [
            _tf(8 * SECOND, 8 * SECOND, 'map', 'base_link', 0.0,
                yaw=math.radians(179.0)),
            _tf(8_500_000_000, 10 * SECOND, 'map', 'base_link', 0.0,
                yaw=math.radians(-179.0)),
        ],
        parent_frame='map', child_frame='base_link', at_received_ns=9 * SECOND,
    )

    assert result['status'] == 'resolved'
    assert math.isclose(abs(result['yaw']), math.pi, abs_tol=1.0e-12)


def test_common_time_tf_resolver_excludes_samples_received_after_success():
    result = resolve_planar_tf_pose(
        [
            _tf(9_900_000_000, 9_900_000_000, 'map', 'base_link', 1.0),
            _tf(10_100_000_000, 10 * SECOND, 'map', 'base_link', 99.0),
        ],
        parent_frame='map', child_frame='base_link', at_received_ns=10 * SECOND,
    )

    assert result['status'] == 'resolved'
    assert result['x'] == 1.0
    assert result['newest_used_tf_received_ns'] == 9_900_000_000


def test_common_time_tf_resolver_rejects_zero_success_or_dynamic_stamp():
    invalid_success = resolve_planar_tf_pose(
        [], parent_frame='map', child_frame='base_link', at_received_ns=0,
    )
    invalid_stamp = resolve_planar_tf_pose(
        [_tf(SECOND, 0, 'map', 'base_link', 1.0)],
        parent_frame='map', child_frame='base_link', at_received_ns=2 * SECOND,
    )

    assert invalid_success['failure_reason'] == 'invalid_action_success_timestamp'
    assert invalid_stamp['failure_reason'] == 'no_transform_chain_available_at_success'
    assert invalid_stamp['invalid_dynamic_stamp_count'] == 1


def _runtime_collector(*, clearing=False):
    collector = RuntimeEvidenceCollector()
    collector.costmaps.extend([
        _runtime_grid(200_000_000),
        _runtime_grid(500_000_000),
        _runtime_grid(3 * SECOND, marked=True),
    ])
    if clearing:
        collector.costmaps.append(_runtime_grid(6 * SECOND))
    collector.costmap_full_count = len(collector.costmaps)
    collector.costmap_count = len(collector.costmaps)
    collector.status_events.extend([
        MissionStatusRecord(1 * SECOND, 'goal-1', GoalStatus.STATUS_ACCEPTED),
        MissionStatusRecord(1_100_000_000, 'goal-1', GoalStatus.STATUS_EXECUTING),
        MissionStatusRecord(10 * SECOND, 'goal-1', GoalStatus.STATUS_SUCCEEDED),
    ])
    collector.requested_goals.append(RequestedGoalRecord(
        900_000_000, 0, 'map', 0.0, -5.0, 0.0
    ))
    collector.transforms.append(TransformRecord(
        9_900_000_000, 9_900_000_000, 'map', 'base_link',
        0.0, -5.0, 0.0,
    ))
    collector.automatic_plans.extend([
        PlanRecord('automatic_0001', 1_500_000_000, 1_500_000_000, 0.0, 0, '',
                   ((0.0, 5.0), (0.0, -5.0))),
        PlanRecord('automatic_0002', 4 * SECOND, 4 * SECOND, 0.0, 0, '',
                   ((0.0, 5.0), (3.0, 2.5), (3.0, -2.5), (0.0, -5.0))),
        PlanRecord('automatic_0003', 7 * SECOND, 7 * SECOND, 0.0, 0, '',
                   ((3.0, -1.0), (0.0, -5.0))),
    ])
    collector.plan_topic_count = len(collector.automatic_plans)
    collector.poses.extend([
        PoseRecord(1 * SECOND, 0.0, 5.0, 0.0),
        PoseRecord(3 * SECOND, 3.0, 2.5, 0.0),
        PoseRecord(6 * SECOND, 3.0, -2.5, 0.0),
        PoseRecord(
            10 * SECOND, 0.0, -5.0, 0.0, 9_950_000_000,
            'map', 'base_link', 'amcl_pose',
        ),
        PoseRecord(
            11 * SECOND, 1.0, -5.0, 0.0, 10_950_000_000,
            'map', 'base_link', 'amcl_pose',
        ),
    ])
    collector.feedback.extend([
        FeedbackRecord(
            8 * SECOND,
            'goal-1',
            PoseRecord(
                8 * SECOND, 0.0, -4.5, 0.0, 7_950_000_000,
                'map', 'base_link', 'navigate_to_pose_feedback',
            ),
            0.5,
            0,
        ),
        FeedbackRecord(
            9_800_000_000,
            'goal-1',
            PoseRecord(
                9_800_000_000, 1.2, -4.8, 0.0, 9_750_000_000,
                'map', 'base_link', 'navigate_to_pose_feedback',
            ),
            1.3,
            0,
        ),
    ])
    hazard = _hazard(
        track_id='runtime-hazard', source_uavs=['dji1'],
        state=AerialHazard.CONFIRMED, x=0.0, stamp_ns=2 * SECOND,
    )
    hazard.detection.bbox.center.position.y = 0.0
    hazard.detection.results[0].pose.pose.position.y = 0.0
    hazard.detection.bbox.size.x = 2.0
    hazard.detection.bbox.size.y = 2.0
    message = _array(hazard, stamp_ns=2_100_000_000)
    for topic in (DJI1_TOPIC, DJI0_TOPIC, UGV_TOPIC):
        collector.add(topic, copy.deepcopy(message), 2_200_000_000)
    if clearing:
        empty = _array(stamp_ns=5_500_000_000)
        for topic in (DJI1_TOPIC, DJI0_TOPIC, UGV_TOPIC):
            collector.add(topic, copy.deepcopy(empty), 5_500_000_000)
    return collector


def _summarize_runtime_collector(
    collector,
    *,
    scenario='clearing',
    stateful=True,
    baseline_trajectory=None,
):
    nav2_config = load_nav2_inflation_config(
        REPO_ROOT / 'src/lrs_halmstad/config/nav2_baylands_large_map.yaml'
    )
    baseline_bundle = None
    if scenario != 'baseline':
        baseline_bundle = {
            'root': '/tmp/baseline',
            'summary': {'status': 'pass'},
            'plans': [{
                'label': 'baseline',
                'crosses_covariance_footprint': True,
            }],
            'trajectory': (
                baseline_trajectory
                if baseline_trajectory is not None
                else [(0.0, 5.0), (0.0, -5.0)]
            ),
        }
    return summarize_runtime_evidence(
        collector,
        scenario=scenario,
        start=(0.0, 5.0, 0.0),
        goal=(0.0, -5.0, 0.0),
        geometry={
            'center_x': 0.0, 'center_y': 0.0, 'yaw': 0.0,
            'nominal_size_x': 2.0, 'nominal_size_y': 2.0,
            'variance_x': 0.25, 'variance_y': 0.25,
            'covariance_sigma_scale': 2.0,
            'effective_size_x': 4.0, 'effective_size_y': 4.0,
        },
        nav2_config=nav2_config,
        map_path=REPO_ROOT / 'maps/baylands.yaml',
        layer_enabled=scenario != 'baseline',
        baseline_bundle=baseline_bundle,
        minimum_path_change_m=0.5,
        minimum_trajectory_change_m=0.75,
        maximum_plan_tracking_error_m=2.0,
        goal_tolerance_m=1.0,
        yaw_goal_tolerance_rad=2.5,
        runtime_goal_checker={
            'goal_checker_plugins': ['general_goal_checker'],
            'plugin': 'nav2_controller::SimpleGoalChecker',
            'xy_goal_tolerance_m': 1.0,
            'yaw_goal_tolerance_rad': 2.5,
            'stateful': stateful,
            'source': 'test',
        },
    )


def test_runtime_summary_requires_passive_replan_motion_and_clearing(tmp_path):
    collector = _runtime_collector(clearing=True)
    nav2_config = load_nav2_inflation_config(
        REPO_ROOT / 'src/lrs_halmstad/config/nav2_baylands_large_map.yaml'
    )
    baseline_bundle = {
        'root': '/tmp/baseline',
        'summary': {'status': 'pass'},
        'plans': [{'label': 'baseline', 'crosses_covariance_footprint': True}],
        'trajectory': [(0.0, 5.0), (0.0, -5.0)],
    }
    summary, trajectory = summarize_runtime_evidence(
        collector,
        scenario='clearing',
        start=(0.0, 5.0, 0.0),
        goal=(0.0, -5.0, 0.0),
        geometry={
            'center_x': 0.0, 'center_y': 0.0, 'yaw': 0.0,
            'nominal_size_x': 2.0, 'nominal_size_y': 2.0,
            'variance_x': 0.25, 'variance_y': 0.25,
            'covariance_sigma_scale': 2.0,
            'effective_size_x': 4.0, 'effective_size_y': 4.0,
        },
        nav2_config=nav2_config,
        map_path=REPO_ROOT / 'maps/baylands.yaml',
        layer_enabled=True,
        baseline_bundle=baseline_bundle,
        minimum_path_change_m=0.5,
        minimum_trajectory_change_m=0.75,
        maximum_plan_tracking_error_m=2.0,
        goal_tolerance_m=1.0,
        yaw_goal_tolerance_rad=2.5,
        runtime_goal_checker={
            'goal_checker_plugins': ['general_goal_checker'],
            'plugin': 'nav2_controller::SimpleGoalChecker',
            'xy_goal_tolerance_m': 1.0,
            'yaw_goal_tolerance_rad': 2.5,
            'stateful': True,
            'source': 'test',
        },
    )

    assert summary['status'] == 'pass', summary['failures']
    assert summary['configuration']['manual_planner_requests_issued_by_evidence'] == 0
    assert summary['planner']['hazard_active']['crosses_lethal_costmap_cell'] is False
    assert summary['costmap']['first_mark_delta']['hazard_footprint_lethal_cells'] > 0
    assert summary['costmap']['first_mark_delta']['inflation_halo_nonzero_cells'] > 0
    assert summary['costmap']['clearing']['observed_mechanism'] == 'explicit_empty_snapshot'
    assert summary['mission']['terminal_status'] == 'SUCCEEDED'
    assert summary['mission']['action_success_received_ns'] == 10 * SECOND
    assert summary['mission']['tf_resolution_at_action_success']['status'] == 'resolved'
    assert summary['mission']['tf_pose_at_common_time']['xy_error_m'] == 0.0
    assert math.isclose(
        summary['mission']['action_feedback_pose_at_or_before_success']['xy_error_m'],
        math.hypot(1.2, 0.2),
    )
    feedback_xy = summary['mission']['stateful_xy_diagnostics']['action_feedback']
    assert feedback_xy['first_pose_within_xy_tolerance']['xy_error_m'] == 0.5
    assert feedback_xy['later_pose_outside_xy_tolerance'] is True
    assert math.isclose(feedback_xy['maximum_later_xy_error_m'], math.hypot(1.2, 0.2))
    assert summary['mission']['amcl_pose_at_or_before_success']['xy_error_m'] == 0.0
    assert summary['mission']['later_shutdown_amcl_pose']['xy_error_m'] == 1.0
    assert summary['mission']['runtime_goal_checker']['stateful'] is True
    assert summary['mission']['goal_checker_evidence']['classification'] == (
        'within_xy_tolerance_at_success'
    )
    assert summary['planner']['automatic_mission_replanning']['observed'] is True
    assert summary['trajectory']['physical_detour_observed'] is True
    assert summary['trajectory']['first_physical_deviation_after_replan'] is not None
    assert summary['costmap']['clearing'][
        'explicit_empty_propagation_complete'
    ] is True
    assert summary['costmap']['navigation_continued_after_clear'] is True
    assert summary['configuration']['requested_action_goal']['matches_configured_goal'] is True
    assert summary['configuration']['requested_action_goal'][
        'xy_error_to_configured_goal_m'
    ] == 0.0
    assert summary['configuration']['requested_action_goal'][
        'yaw_error_to_configured_goal_rad'
    ] == 0.0
    assert summary['trajectory']['crosses_covariance_footprint'] is False
    assert summary['trajectory']['active_hazard_interval'][
        'crosses_covariance_footprint'
    ] is False
    assert len(trajectory) == 4
    write_runtime_evidence(tmp_path, summary, collector, trajectory)
    for filename in (
        'summary.json', 'plans.json', 'hazard_timeline.csv',
        'costmap_timeline.csv', 'mission_timeline.csv', 'trajectory.csv',
        'planner_overlay.svg', 'runtime_overlay.svg',
    ):
        assert (tmp_path / filename).is_file()


def test_clearing_allows_return_through_hazard_region_only_after_clear():
    collector = _runtime_collector(clearing=True)
    collector.poses.insert(3, PoseRecord(7 * SECOND, 1.0, -1.0, 0.0))

    summary, _ = _summarize_runtime_collector(collector, scenario='clearing')

    assert summary['status'] == 'pass', summary['failures']
    assert summary['trajectory']['crosses_covariance_footprint'] is True
    assert summary['trajectory']['active_hazard_interval'][
        'crosses_covariance_footprint'
    ] is False
    assert summary['trajectory']['active_hazard_interval']['clear_received_ns'] == 6 * SECOND
    assert 'ugv_trajectory_crosses_effective_hazard_while_active' not in summary[
        'failures'
    ]


def test_runtime_summary_rejects_center_crossing_while_hazard_is_active():
    collector = _runtime_collector(clearing=True)
    collector.poses.insert(2, PoseRecord(4 * SECOND, 0.0, 0.0, 0.0))

    summary, _ = _summarize_runtime_collector(collector, scenario='clearing')

    assert summary['status'] == 'fail'
    assert summary['trajectory']['active_hazard_interval'][
        'crosses_covariance_footprint'
    ] is True
    assert 'ugv_trajectory_crosses_effective_hazard_while_active' in summary[
        'failures'
    ]


def test_runtime_summary_cannot_pass_without_terminal_mission_evidence():
    collector = _runtime_collector()
    collector.status_events.pop()
    nav2_config = load_nav2_inflation_config(
        REPO_ROOT / 'src/lrs_halmstad/config/nav2_baylands_large_map.yaml'
    )
    summary, _ = summarize_runtime_evidence(
        collector,
        scenario='valid', start=(0.0, 5.0, 0.0), goal=(0.0, -5.0, 0.0),
        geometry={
            'center_x': 0.0, 'center_y': 0.0, 'yaw': 0.0,
            'nominal_size_x': 2.0, 'nominal_size_y': 2.0,
            'variance_x': 0.25, 'variance_y': 0.25,
            'covariance_sigma_scale': 2.0,
            'effective_size_x': 4.0, 'effective_size_y': 4.0,
        },
        nav2_config=nav2_config, map_path=REPO_ROOT / 'maps/baylands.yaml',
        layer_enabled=True,
        baseline_bundle={
            'root': '/tmp/baseline', 'summary': {'status': 'pass'},
            'plans': [{'label': 'baseline', 'crosses_covariance_footprint': True}],
            'trajectory': [(0.0, 5.0), (0.0, -5.0)],
        },
        minimum_path_change_m=0.5, minimum_trajectory_change_m=0.75,
        maximum_plan_tracking_error_m=2.0,
        goal_tolerance_m=1.0,
        yaw_goal_tolerance_rad=2.5,
        runtime_goal_checker={
            'goal_checker_plugins': ['general_goal_checker'],
            'plugin': 'nav2_controller::SimpleGoalChecker',
            'xy_goal_tolerance_m': 1.0,
            'yaw_goal_tolerance_rad': 2.5,
            'stateful': True,
            'source': 'test',
        },
    )

    assert summary['status'] == 'inconclusive'
    assert 'navigate_to_pose_goal_not_completed' in summary['failures']
    assert 'terminal_status_missing' in summary['inconclusive_reasons']


def test_stateful_goal_checker_accepts_only_a_proven_xy_entry_before_drift():
    collector = _runtime_collector(clearing=True)
    collector.transforms.clear()
    collector.transforms.append(TransformRecord(
        9_900_000_000, 9_900_000_000, 'map', 'base_link',
        1.2, -5.0, 0.0,
    ))

    summary, _ = _summarize_runtime_collector(collector)

    evidence = summary['mission']['goal_checker_evidence']
    assert summary['status'] == 'pass', summary['failures']
    assert evidence['classification'] == 'stateful_xy_latched_after_proven_entry'
    assert evidence['first_proven_xy_entry']['xy_error_m'] == 0.5
    assert evidence['xy_entry_proof_source'] == 'navigate_to_pose_feedback'


def test_action_success_is_not_vetoed_by_independent_map_frame_xy_error():
    collector = _runtime_collector(clearing=True)
    collector.transforms.clear()
    collector.transforms.append(TransformRecord(
        9_900_000_000, 9_900_000_000, 'map', 'base_link',
        1.2, -5.0, 0.0,
    ))
    collector.feedback.clear()
    collector.feedback.append(FeedbackRecord(
        9_800_000_000,
        'goal-1',
        PoseRecord(
            9_800_000_000, 1.5, -5.0, 0.0, 9_750_000_000,
            'map', 'base_link', 'navigate_to_pose_feedback',
        ),
        1.5,
        0,
    ))

    summary, _ = _summarize_runtime_collector(collector)

    assert summary['status'] == 'pass', summary['failures']
    assert summary['mission']['goal_checker_evidence']['classification'] == (
        'success_without_proven_xy_entry'
    )
    diagnostic = summary['mission']['localization_diagnostic']
    assert diagnostic['classification'] == 'limitation'
    assert diagnostic['independent_map_frame_xy_outside_controller_tolerance'] is True
    assert diagnostic['map_frame_tf_xy_error_m'] == 1.2
    assert summary['mission']['goal_checker_evidence']['diagnostic_only'] is True
    assert summary['mission']['terminal_status'] == 'SUCCEEDED'


def test_goal_checker_reports_insufficient_pose_evidence_separately():
    collector = _runtime_collector(clearing=True)
    collector.transforms.clear()
    collector.feedback.clear()

    summary, _ = _summarize_runtime_collector(collector)

    assert summary['status'] == 'pass', summary['failures']
    assert summary['mission']['goal_checker_evidence']['classification'] == (
        'insufficient_or_inconsistent_pose_evidence'
    )
    assert summary['mission']['localization_diagnostic']['classification'] == 'unavailable'


def test_non_stateful_map_frame_discrepancy_remains_diagnostic():
    collector = _runtime_collector(clearing=True)
    collector.transforms.clear()
    collector.transforms.append(TransformRecord(
        9_900_000_000, 9_900_000_000, 'map', 'base_link',
        1.2, -5.0, 0.0,
    ))

    summary, _ = _summarize_runtime_collector(collector, stateful=False)

    assert summary['status'] == 'fail'
    assert summary['mission']['goal_checker_evidence']['classification'] == (
        'outside_xy_tolerance_at_success'
    )


def test_action_failure_still_fails_with_good_map_frame_pose():
    collector = _runtime_collector(clearing=True)
    collector.status_events.pop()
    collector.status_events.append(MissionStatusRecord(
        10 * SECOND, 'goal-1', GoalStatus.STATUS_ABORTED
    ))
    summary, _ = _summarize_runtime_collector(collector)
    assert summary['status'] == 'fail'
    assert 'navigate_to_pose_goal_not_succeeded' in summary['failures']


def test_baseline_cannot_pass_with_ambiguous_goal_identity():
    collector = _runtime_collector()
    collector.status_events.append(MissionStatusRecord(
        5 * SECOND, 'goal-2', GoalStatus.STATUS_EXECUTING
    ))
    summary, _ = _summarize_runtime_collector(collector, scenario='baseline')
    assert summary['status'] == 'fail'
    assert 'navigate_to_pose_goal_identity_ambiguous' in summary['failures']


def test_baseline_cannot_pass_without_physical_motion():
    collector = _runtime_collector()
    collector.poses.clear()
    collector.poses.extend([
        PoseRecord(2 * SECOND, 0.0, 5.0, 0.0),
        PoseRecord(9 * SECOND, 0.0, 5.0, 0.0),
    ])
    summary, _ = _summarize_runtime_collector(collector, scenario='baseline')
    assert summary['status'] == 'fail'
    assert 'ugv_trajectory_missing' in summary['failures']


def test_automatic_replanning_rejects_an_ambiguous_goal_lifetime():
    collector = _runtime_collector(clearing=True)
    collector.status_events.append(MissionStatusRecord(
        5 * SECOND, 'goal-2', GoalStatus.STATUS_ACCEPTED,
    ))

    summary, _ = _summarize_runtime_collector(collector)

    replanning = summary['planner']['automatic_mission_replanning']
    assert replanning['observed'] is False
    assert replanning['other_goal_ids_during_mission'] == ['goal-2']
    assert 'automatic_replan_goal_identity_ambiguous' in summary['failures']


def test_terminal_goal_history_does_not_make_replanning_ambiguous():
    collector = _runtime_collector(clearing=True)
    collector.status_events.append(MissionStatusRecord(
        5 * SECOND, 'old-goal', GoalStatus.STATUS_SUCCEEDED,
    ))

    summary, _ = _summarize_runtime_collector(collector)

    replanning = summary['planner']['automatic_mission_replanning']
    assert replanning['observed'] is True
    assert replanning['other_goal_ids_during_mission'] == []


def test_ordinary_plan_traffic_without_post_mark_change_is_not_replanning():
    collector = _runtime_collector(clearing=True)
    unchanged = collector.automatic_plans[0].points
    collector.automatic_plans[1] = PlanRecord(
        'automatic_0002', 4 * SECOND, 4 * SECOND, 0.0, 0, '', unchanged,
    )
    collector.automatic_plans[2] = PlanRecord(
        'automatic_0003', 7 * SECOND, 7 * SECOND, 0.0, 0, '', unchanged,
    )

    summary, _ = _summarize_runtime_collector(collector)

    assert summary['planner']['automatic_mission_replanning']['observed'] is False
    assert 'automatic_post_mark_replan_not_observed' in summary['failures']


def test_pre_replan_motion_cannot_count_as_a_physical_detour():
    collector = _runtime_collector(clearing=True)
    collector.poses[2] = PoseRecord(6 * SECOND, 0.0, -2.5, 0.0)

    summary, _ = _summarize_runtime_collector(collector)

    assert summary['trajectory']['hausdorff_distance_from_baseline_m'] > 0.75
    assert summary['trajectory']['first_physical_deviation_after_replan'] is None
    assert summary['trajectory']['physical_detour_observed'] is False
    assert 'ugv_trajectory_did_not_materially_deviate' in summary['failures']


def test_clearing_requires_ordered_empty_propagation_through_every_stage():
    collector = _runtime_collector(clearing=True)
    collector.samples[DJI1_TOPIC].pop()

    summary, _ = _summarize_runtime_collector(collector)

    clearing = summary['costmap']['clearing']
    assert clearing['explicit_empty_propagation_complete'] is False
    assert clearing['source_explicit_empty_ns'] is None
    assert clearing['fusion_explicit_empty_ns'] == 5_500_000_000
    assert clearing['forwarded_explicit_empty_ns'] == 5_500_000_000
    assert 'explicit_empty_clearing_propagation_incomplete' in summary['failures']


def test_clearing_cannot_pass_without_prior_marking():
    collector = _runtime_collector(clearing=True)
    collector.costmaps[2] = _runtime_grid(3 * SECOND)

    summary, _ = _summarize_runtime_collector(collector)

    assert summary['costmap']['first_mark_ns'] is None
    assert summary['costmap']['clearing']['observed_mechanism'] == 'not_observed'
    assert 'aerial_costmap_mark_not_observed' in summary['failures']


def test_pre_hazard_plan_precedes_forwarded_hazard_even_when_mark_is_delayed():
    collector = _runtime_collector(clearing=True)
    collector.automatic_plans.insert(1, PlanRecord(
        'early_hazard_response', 2_500_000_000, 2_500_000_000, 0.0, 0, '',
        ((0.0, 5.0), (3.0, 2.5), (3.0, -2.5), (0.0, -5.0)),
    ))
    collector.plan_topic_count += 1

    summary, _ = _summarize_runtime_collector(collector)

    assert summary['planner']['pre_hazard']['result_ns'] == 1_500_000_000
    assert summary['planner']['pre_hazard']['crosses_covariance_footprint'] is True


def test_valid_fails_if_aerial_costmap_clears_before_mission_completion():
    collector = _runtime_collector()
    collector.costmaps.append(_runtime_grid(6 * SECOND))
    collector.costmap_count += 1
    collector.costmap_full_count += 1

    summary, _ = _summarize_runtime_collector(collector, scenario='valid')

    assert summary['costmap']['first_mark_ns'] == 3 * SECOND
    assert summary['costmap']['first_clear_ns'] == 6 * SECOND
    assert summary['costmap']['clearing']['source_explicit_empty_ns'] is None
    assert 'aerial_costmap_cleared_during_active_hazard' in summary['failures']


def test_clearing_expiry_and_silence_mechanisms_remain_distinct():
    collector = _runtime_collector()

    age_expiry = _clearing_mechanism(collector, 5 * SECOND, 2.0)
    ttl_expiry = _clearing_mechanism(collector, 5 * SECOND, 10.0)
    source_silence = _clearing_mechanism(collector, 3 * SECOND, 10.0)

    assert age_expiry['observed_mechanism'] == 'observation_age_expiry'
    assert ttl_expiry['observed_mechanism'] == 'ttl_expiry'
    assert source_silence['observed_mechanism'] == 'source_silence'
    for result in (age_expiry, ttl_expiry, source_silence):
        assert result['explicit_empty_propagation_complete'] is False
