#!/usr/bin/env python3
"""Bounded live and rosbag evidence checks for the typed support-hazard chain."""

from __future__ import annotations

import argparse
import ast
from collections import deque
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import heapq
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Iterable
import xml.etree.ElementTree as ET

from geometry_msgs.msg import PoseStamped
from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.msg import Costmap, CostmapUpdate
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path as NavPath
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
import yaml


DJI1_TOPIC = '/coord/support/dji1/aerial_hazards'
DJI2_TOPIC = '/coord/support/dji2/aerial_hazards'
DJI0_TOPIC = '/coord/dji0/aerial_hazards'
UGV_TOPIC = '/coord/ugv/aerial_hazards'
HAZARD_TOPICS = (DJI1_TOPIC, DJI2_TOPIC, DJI0_TOPIC, UGV_TOPIC)
COSTMAP_TOPICS = (
    '/a201_0000/global_costmap/costmap_raw',
    '/a201_0000/global_costmap/costmap',
)
STATE_NAMES = {
    AerialHazard.TENTATIVE: 'TENTATIVE',
    AerialHazard.CONFIRMED: 'CONFIRMED',
    AerialHazard.CONFLICT: 'CONFLICT',
}
FREE_SPACE = 0
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255


def stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def _float_tuple(values: Iterable[float]) -> tuple[float, ...]:
    return tuple(float(value) for value in values)


def hazard_geometry_signature(hazard: AerialHazard) -> tuple[Any, ...]:
    detection = hazard.detection
    center = detection.bbox.center.position
    orientation = detection.bbox.center.orientation
    size = detection.bbox.size
    class_id = (
        str(detection.results[0].hypothesis.class_id)
        if detection.results
        else ''
    )
    covariance = (
        _float_tuple(detection.results[0].pose.covariance)
        if detection.results
        else ()
    )
    return (
        class_id,
        _float_tuple((center.x, center.y, center.z)),
        _float_tuple((orientation.x, orientation.y, orientation.z, orientation.w)),
        _float_tuple((size.x, size.y, size.z)),
        covariance,
    )


@dataclass(frozen=True)
class CapturedSample:
    topic: str
    received_ns: int
    message: AerialHazardArray


@dataclass(frozen=True)
class EvidenceExpectations:
    require_dji2: bool = False
    expected_state: int | None = None
    expected_sources: tuple[str, ...] = ()
    expected_selected_source: str = ''
    minimum_hazard_count: int = 1
    require_confirmation_promotion: bool = False
    require_conflict: bool = False
    require_expiry: bool = False
    require_costmap: bool = False
    require_typed_flow: bool = True
    require_forwarding: bool = True
    require_covariance_match: bool = True
    required_topics: tuple[str, ...] = ()
    max_age_s: float = 1.0


@dataclass(frozen=True)
class GridSnapshot:
    received_ns: int
    source_kind: str
    resolution: float
    size_x: int
    size_y: int
    origin_x: float
    origin_y: float
    data: bytes | tuple[int, ...]


@dataclass(frozen=True)
class PlanRecord:
    label: str
    requested_ns: int
    received_ns: int
    planning_time_s: float
    error_code: int
    error_message: str
    points: tuple[tuple[float, float], ...]


def effective_hazard_geometry(
    hazard: AerialHazard,
    *,
    covariance_sigma_scale: float = 2.0,
) -> dict[str, float]:
    detection = hazard.detection
    covariance = detection.results[0].pose.covariance if detection.results else [0.0] * 36
    variance = max(float(covariance[0]), float(covariance[7]))
    uncertainty = float(covariance_sigma_scale) * math.sqrt(max(0.0, variance))
    orientation = detection.bbox.center.orientation
    yaw = math.atan2(
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
        1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
    )
    return {
        'center_x': float(detection.bbox.center.position.x),
        'center_y': float(detection.bbox.center.position.y),
        'yaw': yaw,
        'nominal_size_x': float(detection.bbox.size.x),
        'nominal_size_y': float(detection.bbox.size.y),
        'variance_x': float(covariance[0]),
        'variance_y': float(covariance[7]),
        'covariance_sigma_scale': float(covariance_sigma_scale),
        'uncertainty_per_side_m': uncertainty,
        'effective_size_x': float(detection.bbox.size.x) + 2.0 * uncertainty,
        'effective_size_y': float(detection.bbox.size.y) + 2.0 * uncertainty,
    }


def path_length(points: Iterable[tuple[float, float]]) -> float:
    values = list(points)
    return sum(
        math.hypot(x1 - x0, y1 - y0)
        for (x0, y0), (x1, y1) in zip(values, values[1:])
    )


def _point_to_rect_distance(
    point: tuple[float, float], geometry: dict[str, float]
) -> float:
    dx = point[0] - geometry['center_x']
    dy = point[1] - geometry['center_y']
    cosine = math.cos(geometry['yaw'])
    sine = math.sin(geometry['yaw'])
    local_x = cosine * dx + sine * dy
    local_y = -sine * dx + cosine * dy
    outside_x = max(abs(local_x) - 0.5 * geometry['effective_size_x'], 0.0)
    outside_y = max(abs(local_y) - 0.5 * geometry['effective_size_y'], 0.0)
    return math.hypot(outside_x, outside_y)


def segment_crosses_effective_hazard(
    start: tuple[float, float],
    end: tuple[float, float],
    geometry: dict[str, float],
) -> bool:
    distance = math.hypot(end[0] - start[0], end[1] - start[1])
    sample_step = max(min(geometry['effective_size_x'], geometry['effective_size_y']) / 20.0, 0.05)
    samples = max(1, int(math.ceil(distance / sample_step)))
    return any(
        _point_to_rect_distance(
            (
                start[0] + (end[0] - start[0]) * index / samples,
                start[1] + (end[1] - start[1]) * index / samples,
            ),
            geometry,
        ) <= 1.0e-9
        for index in range(samples + 1)
    )


def path_hazard_metrics(
    points: Iterable[tuple[float, float]], geometry: dict[str, float]
) -> dict[str, Any]:
    values = list(points)
    sampled_points = list(values)
    for start, end in zip(values, values[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        samples = max(1, int(math.ceil(distance / 0.05)))
        sampled_points.extend(
            (
                start[0] + (end[0] - start[0]) * index / samples,
                start[1] + (end[1] - start[1]) * index / samples,
            )
            for index in range(1, samples)
        )
    minimum_distance = min(
        (_point_to_rect_distance(point, geometry) for point in sampled_points),
        default=None,
    )
    crosses = any(
        segment_crosses_effective_hazard(start, end, geometry)
        for start, end in zip(values, values[1:])
    )
    return {
        'minimum_distance_to_effective_hazard_m': minimum_distance,
        'crosses_effective_hazard': crosses,
    }


def expanded_hazard_geometry(
    geometry: dict[str, float], inflation_radius_m: float
) -> dict[str, float]:
    """Return the covariance footprint expanded by the configured Nav2 radius."""
    expanded = dict(geometry)
    expanded['effective_size_x'] = geometry['effective_size_x'] + 2.0 * inflation_radius_m
    expanded['effective_size_y'] = geometry['effective_size_y'] + 2.0 * inflation_radius_m
    return expanded


def load_nav2_inflation_config(path: Path) -> dict[str, Any]:
    """Load the global-costmap inflation settings from the actual Nav2 YAML."""
    resolved = path.expanduser().resolve()
    with resolved.open(encoding='utf-8') as stream:
        document = yaml.safe_load(stream)
    if not isinstance(document, dict):
        raise ValueError(f'Nav2 configuration is not a mapping: {resolved}')

    candidates = [document]
    candidates.extend(value for value in document.values() if isinstance(value, dict))
    parameters = None
    for candidate in candidates:
        try:
            parameters = candidate['global_costmap']['global_costmap']['ros__parameters']
        except (KeyError, TypeError):
            continue
        break
    if not isinstance(parameters, dict):
        raise ValueError(f'global_costmap ROS parameters not found in {resolved}')
    inflation = parameters.get('inflation_layer')
    if not isinstance(inflation, dict):
        raise ValueError(f'global_costmap inflation_layer not found in {resolved}')
    radius = float(inflation['inflation_radius'])
    scaling = float(inflation['cost_scaling_factor'])
    if not math.isfinite(radius) or radius < 0.0:
        raise ValueError('global inflation_radius must be finite and non-negative')
    if not math.isfinite(scaling) or scaling <= 0.0:
        raise ValueError('global cost_scaling_factor must be finite and positive')
    aerial = parameters.get('aerial_support_layer')
    if not isinstance(aerial, dict):
        raise ValueError(f'global_costmap aerial_support_layer not found in {resolved}')
    return {
        'source_yaml': str(resolved),
        'source_sha256': hashlib.sha256(resolved.read_bytes()).hexdigest(),
        'inflation_radius_m': radius,
        'cost_scaling_factor': scaling,
        'aerial_min_confidence': float(aerial['min_confidence']),
        'aerial_max_observation_age_s': float(aerial['max_observation_age_s']),
        'aerial_covariance_sigma_scale': float(aerial['covariance_sigma_scale']),
    }


def discrete_hausdorff_distance(
    first: Iterable[tuple[float, float]], second: Iterable[tuple[float, float]]
) -> float | None:
    a = list(first)
    b = list(second)
    if not a or not b:
        return None

    def directed(source, target):
        return max(
            min(math.hypot(px - qx, py - qy) for qx, qy in target)
            for px, py in source
        )

    return max(directed(a, b), directed(b, a))


def costmap_value(snapshot: GridSnapshot, x: float, y: float) -> int | None:
    mx = int(math.floor((x - snapshot.origin_x) / snapshot.resolution))
    my = int(math.floor((y - snapshot.origin_y) / snapshot.resolution))
    if mx < 0 or my < 0 or mx >= snapshot.size_x or my >= snapshot.size_y:
        return None
    return int(snapshot.data[my * snapshot.size_x + mx])


def nav2_cost_class(cost: int) -> str:
    """Classify an unsigned Nav2 cost without conflating unknown and lethal."""
    value = int(cost)
    if value == FREE_SPACE:
        return 'free'
    if 1 <= value < LETHAL_OBSTACLE:
        return 'graded'
    if value == LETHAL_OBSTACLE:
        return 'lethal'
    if value == NO_INFORMATION:
        return 'no_information'
    raise ValueError(f'Nav2 cost must be in [0, 255], got {value}')


def segment_crosses_lethal_cost(
    points: Iterable[tuple[float, float]], snapshot: GridSnapshot
) -> bool:
    values = list(points)
    for start, end in zip(values, values[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        samples = max(1, int(math.ceil(distance / max(snapshot.resolution * 0.5, 0.01))))
        for index in range(samples + 1):
            x = start[0] + (end[0] - start[0]) * index / samples
            y = start[1] + (end[1] - start[1]) * index / samples
            value = costmap_value(snapshot, x, y)
            if value is not None and nav2_cost_class(value) == 'lethal':
                return True
    return False


def path_cost_exposure(
    points: Iterable[tuple[float, float]],
    snapshot: GridSnapshot,
    hazard_geometry: dict[str, float],
    analysis_geometry: dict[str, float],
    baseline_snapshot: GridSnapshot | None = None,
) -> dict[str, Any]:
    """Measure lethal and graded cost exposure inside the hazard analysis region."""
    values = list(points)
    graded_length = 0.0
    graded_samples = 0
    graded_cells: set[tuple[int, int]] = set()
    lethal_intersection = False
    for start, end in zip(values, values[1:]):
        distance = math.hypot(end[0] - start[0], end[1] - start[1])
        samples = max(1, int(math.ceil(distance / max(snapshot.resolution * 0.5, 0.01))))
        step_length = distance / samples
        for index in range(samples):
            fraction = (index + 0.5) / samples
            x = start[0] + (end[0] - start[0]) * fraction
            y = start[1] + (end[1] - start[1]) * fraction
            if _point_to_rect_distance((x, y), analysis_geometry) > 1.0e-9:
                continue
            cost = costmap_value(snapshot, x, y)
            if cost is None:
                continue
            baseline_cost = (
                costmap_value(baseline_snapshot, x, y)
                if baseline_snapshot is not None else 0
            )
            mx = int(math.floor((x - snapshot.origin_x) / snapshot.resolution))
            my = int(math.floor((y - snapshot.origin_y) / snapshot.resolution))
            cost_class = nav2_cost_class(cost)
            if cost_class == 'lethal':
                lethal_intersection = True
            elif (
                cost_class == 'graded'
                and baseline_cost is not None
                and cost != baseline_cost
                and _point_to_rect_distance((x, y), hazard_geometry) > 1.0e-9
            ):
                graded_samples += 1
                graded_cells.add((mx, my))
                graded_length += step_length
    return {
        'crosses_lethal_costmap_cell': lethal_intersection,
        'graded_inflated_cost_sample_count': graded_samples,
        'graded_inflated_cost_unique_cell_count': len(graded_cells),
        'graded_inflated_cost_path_length_m': graded_length,
    }


def _path_geometry_hash(points: Iterable[tuple[float, float]]) -> str:
    canonical = [[round(x, 6), round(y, 6)] for x, y in points]
    return hashlib.sha256(
        json.dumps(canonical, separators=(',', ':')).encode('utf-8')
    ).hexdigest()


def baseline_repeatability(
    records: Iterable[PlanRecord], *, required_count: int, tolerance_m: float
) -> dict[str, Any]:
    """Summarize repeated baseline requests; missing evidence always fails."""
    selected = [record for record in records if record.label.startswith('baseline')]
    successful = [
        record for record in selected if record.error_code == 0 and len(record.points) >= 2
    ]
    deviations = [
        discrete_hausdorff_distance(first.points, second.points)
        for index, first in enumerate(successful)
        for second in successful[index + 1:]
    ]
    maximum_deviation = max(
        (value for value in deviations if value is not None), default=None
    )
    failures = []
    if len(selected) != required_count:
        failures.append('baseline_request_count_incomplete')
    if len(successful) != required_count:
        failures.append('baseline_success_count_incomplete')
    if required_count > 1 and maximum_deviation is None:
        failures.append('baseline_deviation_unavailable')
    elif maximum_deviation is not None and maximum_deviation > tolerance_m:
        failures.append('baseline_path_not_repeatable')
    return {
        'status': 'pass' if not failures else 'fail',
        'required_request_count': required_count,
        'observed_request_count': len(selected),
        'successful_result_count': len(successful),
        'tolerance_m': tolerance_m,
        'maximum_pairwise_path_deviation_m': maximum_deviation,
        'requests': [{
            'label': record.label,
            'error_code': record.error_code,
            'path_pose_count': len(record.points),
            'path_length_m': path_length(record.points),
            'geometry_sha256': _path_geometry_hash(record.points),
            'reported_planning_time_s': record.planning_time_s,
            'response_latency_s': (record.received_ns - record.requested_ns) * 1.0e-9,
        } for record in selected],
        'timing_is_acceptance_criterion': False,
        'failures': failures,
    }


def relevant_costmap_delta(
    baseline: GridSnapshot,
    candidate: GridSnapshot,
    geometry: dict[str, float],
    inflation_radius_m: float = 0.0,
) -> dict[str, Any]:
    if (
        baseline.resolution != candidate.resolution
        or baseline.size_x != candidate.size_x
        or baseline.size_y != candidate.size_y
        or baseline.origin_x != candidate.origin_x
        or baseline.origin_y != candidate.origin_y
    ):
        return {
            'comparable': False,
            'affected_cells': 0,
            'lethal_cells': 0,
            'maximum_cost': None,
            'inflation_halo_nonzero_cells': 0,
            'analysis_region_affected_cells': 0,
            'analysis_region_raw_changed_cells': 0,
            'no_information_transition_cells': 0,
        }
    analysis_geometry = expanded_hazard_geometry(geometry, inflation_radius_m)
    half_diagonal = 0.5 * math.hypot(
        analysis_geometry['effective_size_x'], analysis_geometry['effective_size_y']
    )
    min_x = int(math.floor(
        (geometry['center_x'] - half_diagonal - candidate.origin_x)
        / candidate.resolution
    ))
    max_x = int(math.ceil(
        (geometry['center_x'] + half_diagonal - candidate.origin_x)
        / candidate.resolution
    ))
    min_y = int(math.floor(
        (geometry['center_y'] - half_diagonal - candidate.origin_y)
        / candidate.resolution
    ))
    max_y = int(math.ceil(
        (geometry['center_y'] + half_diagonal - candidate.origin_y)
        / candidate.resolution
    ))
    core_affected = 0
    core_lethal = 0
    core_current_lethal = 0
    core_values = []
    analysis_values = []
    affected_values = []
    halo_affected = 0
    halo_nonzero = 0
    halo_values = []
    raw_changed = 0
    no_information_transitions = 0
    core_classes = {name: 0 for name in ('free', 'graded', 'lethal', 'no_information')}
    halo_classes = {name: 0 for name in ('free', 'graded', 'lethal', 'no_information')}
    for my in range(max(0, min_y), min(candidate.size_y, max_y + 1)):
        for mx in range(max(0, min_x), min(candidate.size_x, max_x + 1)):
            wx = candidate.origin_x + (mx + 0.5) * candidate.resolution
            wy = candidate.origin_y + (my + 0.5) * candidate.resolution
            if _point_to_rect_distance((wx, wy), analysis_geometry) > 1.0e-9:
                continue
            index = my * candidate.size_x + mx
            value = int(candidate.data[index])
            baseline_value = int(baseline.data[index])
            value_class = nav2_cost_class(value)
            baseline_class = nav2_cost_class(baseline_value)
            in_core = _point_to_rect_distance((wx, wy), geometry) <= 1.0e-9
            analysis_values.append(value)
            if in_core:
                core_values.append(value)
                core_classes[value_class] += 1
                if value_class == 'lethal':
                    core_current_lethal += 1
            else:
                halo_values.append(value)
                halo_classes[value_class] += 1
            if value != baseline_value:
                raw_changed += 1
                involves_no_information = (
                    value_class == 'no_information'
                    or baseline_class == 'no_information'
                )
                if involves_no_information:
                    no_information_transitions += 1
                elif in_core:
                    core_affected += 1
                    if value_class == 'lethal':
                        core_lethal += 1
                else:
                    halo_affected += 1
                    if value_class == 'graded':
                        halo_nonzero += 1
                affected_values.append({
                    'cell_x': mx,
                    'cell_y': my,
                    'world_x': wx,
                    'world_y': wy,
                    'region': 'covariance_footprint' if in_core else 'inflation_halo',
                    'baseline_cost': baseline_value,
                    'baseline_cost_class': baseline_class,
                    'current_cost': value,
                    'current_cost_class': value_class,
                    'change_kind': (
                        'no_information_transition'
                        if involves_no_information else 'comparable_cost_change'
                    ),
                })
    return {
        'comparable': True,
        # Compatibility keys retain their original covariance-footprint meaning.
        'affected_cells': core_affected,
        'lethal_cells': core_lethal,
        'maximum_cost': max(core_values) if core_values else None,
        'relevant_cost_values': sorted(set(core_values)),
        'hazard_footprint_affected_cells': core_affected,
        'hazard_footprint_lethal_cells': core_lethal,
        'hazard_footprint_current_lethal_cells': core_current_lethal,
        'hazard_footprint_cost_values': sorted(set(core_values)),
        'hazard_footprint_cost_class_counts': core_classes,
        'inflation_halo_affected_cells': halo_affected,
        'inflation_halo_nonzero_cells': halo_nonzero,
        'inflation_halo_cost_values': sorted(set(halo_values)),
        'inflation_halo_cost_class_counts': halo_classes,
        'analysis_region_affected_cells': core_affected + halo_affected,
        'analysis_region_raw_changed_cells': raw_changed,
        'no_information_transition_cells': no_information_transitions,
        'analysis_region_cost_class_counts': {
            name: core_classes[name] + halo_classes[name] for name in core_classes
        },
        'analysis_region_maximum_cost': max(analysis_values) if analysis_values else None,
        'affected_cell_values': affected_values,
    }


def settled_baseline_selection(
    snapshots: Iterable[GridSnapshot],
    geometry: dict[str, float],
    inflation_radius_m: float,
    *,
    required_consecutive: int = 2,
) -> tuple[GridSnapshot | None, dict[str, Any]]:
    """Select consecutive identical, fully known relevant-region snapshots."""
    if required_consecutive < 2:
        raise ValueError('required_consecutive must be at least two')
    analysis_geometry = expanded_hazard_geometry(geometry, inflation_radius_m)
    observations = []
    previous_signature = None
    consecutive = 0
    selected = None
    maximum_consecutive = 0
    for snapshot in snapshots:
        half_diagonal = 0.5 * math.hypot(
            analysis_geometry['effective_size_x'],
            analysis_geometry['effective_size_y'],
        )
        min_x = int(math.floor(
            (geometry['center_x'] - half_diagonal - snapshot.origin_x)
            / snapshot.resolution
        ))
        max_x = int(math.ceil(
            (geometry['center_x'] + half_diagonal - snapshot.origin_x)
            / snapshot.resolution
        ))
        min_y = int(math.floor(
            (geometry['center_y'] - half_diagonal - snapshot.origin_y)
            / snapshot.resolution
        ))
        max_y = int(math.ceil(
            (geometry['center_y'] + half_diagonal - snapshot.origin_y)
            / snapshot.resolution
        ))
        region_values = bytearray()
        for my in range(max(0, min_y), min(snapshot.size_y, max_y + 1)):
            for mx in range(max(0, min_x), min(snapshot.size_x, max_x + 1)):
                wx = snapshot.origin_x + (mx + 0.5) * snapshot.resolution
                wy = snapshot.origin_y + (my + 0.5) * snapshot.resolution
                if _point_to_rect_distance((wx, wy), analysis_geometry) <= 1.0e-9:
                    region_values.append(int(snapshot.data[my * snapshot.size_x + mx]))
        no_information_count = region_values.count(NO_INFORMATION)
        signature = hashlib.sha256(bytes(region_values)).hexdigest()
        fully_known = bool(region_values) and no_information_count == 0
        if fully_known and signature == previous_signature:
            consecutive += 1
        elif fully_known:
            consecutive = 1
        else:
            consecutive = 0
        previous_signature = signature if fully_known else None
        maximum_consecutive = max(maximum_consecutive, consecutive)
        observations.append({
            'received_ns': snapshot.received_ns,
            'source_kind': snapshot.source_kind,
            'relevant_cell_count': len(region_values),
            'no_information_cell_count': no_information_count,
            'fully_known': fully_known,
            'region_sha256': signature,
            'consecutive_stable_count': consecutive,
        })
        if consecutive >= required_consecutive:
            selected = snapshot
    return selected, {
        'status': 'settled' if selected is not None else 'unsettled',
        'required_consecutive_snapshots': required_consecutive,
        'maximum_consecutive_stable_snapshots': maximum_consecutive,
        'selected_received_ns': selected.received_ns if selected else None,
        'observation_count': len(observations),
        'observations': observations,
    }


class EvidenceCollector:
    """Keep bounded topic samples and derive defensible contract-level evidence."""

    def __init__(self, *, max_samples_per_topic: int = 5000) -> None:
        if max_samples_per_topic < 1:
            raise ValueError('max_samples_per_topic must be at least one')
        self.max_samples_per_topic = int(max_samples_per_topic)
        self.samples = {
            topic: deque(maxlen=self.max_samples_per_topic)
            for topic in HAZARD_TOPICS
        }
        self.total_counts = {topic: 0 for topic in HAZARD_TOPICS}
        self.dropped_counts = {topic: 0 for topic in HAZARD_TOPICS}
        self.costmap_count = 0

    def add(self, topic: str, message: AerialHazardArray, received_ns: int) -> None:
        if topic not in self.samples:
            raise ValueError(f'unsupported hazard topic: {topic}')
        queue = self.samples[topic]
        if len(queue) == queue.maxlen:
            self.dropped_counts[topic] += 1
        queue.append(
            CapturedSample(
                topic=topic,
                received_ns=int(received_ns),
                message=message,
            )
        )
        self.total_counts[topic] += 1

    def add_costmap(self) -> None:
        self.costmap_count += 1

    def _ordered(self, topic: str) -> list[CapturedSample]:
        return sorted(self.samples[topic], key=lambda sample: sample.received_ns)

    def _source_geometry(self) -> dict[tuple[Any, ...], set[str]]:
        matches: dict[tuple[Any, ...], set[str]] = {}
        for source_id, topic in (('dji1', DJI1_TOPIC), ('dji2', DJI2_TOPIC)):
            for sample in self.samples[topic]:
                for hazard in sample.message.hazards:
                    matches.setdefault(hazard_geometry_signature(hazard), set()).add(
                        source_id
                    )
        return matches

    def _selected_sources(self) -> tuple[dict[str, int], str, int]:
        geometry_sources = self._source_geometry()
        counts: dict[str, int] = {'dji1': 0, 'dji2': 0, 'ambiguous': 0, 'unmatched': 0}
        latest_source = ''
        latest_time = -1
        matched_hazards = 0
        for sample in self._ordered(DJI0_TOPIC):
            for hazard in sample.message.hazards:
                sources = geometry_sources.get(hazard_geometry_signature(hazard), set())
                if len(sources) == 1:
                    selected = next(iter(sources))
                    counts[selected] += 1
                    matched_hazards += 1
                    if sample.received_ns >= latest_time:
                        latest_time = sample.received_ns
                        latest_source = selected
                elif sources:
                    counts['ambiguous'] += 1
                else:
                    counts['unmatched'] += 1
        return counts, latest_source, matched_hazards

    def _confirmation_promotion(self) -> bool:
        states_by_track: dict[str, list[int]] = {}
        for sample in self._ordered(DJI0_TOPIC):
            for hazard in sample.message.hazards:
                states_by_track.setdefault(str(hazard.detection.id), []).append(
                    int(hazard.state)
                )
        for states in states_by_track.values():
            try:
                tentative_index = states.index(AerialHazard.TENTATIVE)
                confirmed_index = states.index(AerialHazard.CONFIRMED)
            except ValueError:
                continue
            if tentative_index < confirmed_index:
                return True
        return False

    def _expiry_cleared(self) -> bool:
        saw_nonempty = False
        for sample in self._ordered(UGV_TOPIC):
            if sample.message.hazards:
                saw_nonempty = True
            elif saw_nonempty:
                return True
        return False

    def _forwarding_preserved(self) -> bool | None:
        dji0_messages = [
            sample.message for sample in self.samples[DJI0_TOPIC]
            if sample.message.hazards
        ]
        ugv_messages = [
            sample.message for sample in self.samples[UGV_TOPIC]
            if sample.message.hazards
        ]
        if not dji0_messages or not ugv_messages:
            return None
        return any(
            dji0_message == ugv_message
            for dji0_message in dji0_messages
            for ugv_message in ugv_messages
        )

    def _forwarding_match_count(self) -> int:
        dji0_messages = [
            sample.message for sample in self.samples[DJI0_TOPIC]
            if sample.message.hazards
        ]
        return sum(
            1 for sample in self.samples[UGV_TOPIC]
            if sample.message.hazards
            and any(sample.message == message for message in dji0_messages)
        )

    def _age_metrics(self) -> tuple[float | None, int]:
        ages = []
        invalid_count = 0
        for sample in self.samples[DJI0_TOPIC]:
            publication_ns = stamp_ns(sample.message.header.stamp)
            for hazard in sample.message.hazards:
                age_s = (
                    publication_ns - stamp_ns(hazard.detection.header.stamp)
                ) * 1.0e-9
                if not math.isfinite(age_s) or age_s < 0.0:
                    invalid_count += 1
                else:
                    ages.append(age_s)
        return (max(ages) if ages else None), invalid_count

    def _covariance_preserved(self) -> bool | None:
        geometry_sources = self._source_geometry()
        fused = [
            hazard
            for sample in self.samples[DJI0_TOPIC]
            for hazard in sample.message.hazards
        ]
        if not fused or not geometry_sources:
            return None
        return all(hazard_geometry_signature(hazard) in geometry_sources for hazard in fused)

    def timeline_rows(self) -> list[dict[str, Any]]:
        all_samples = [sample for queue in self.samples.values() for sample in queue]
        if not all_samples:
            return []
        start_ns = min(sample.received_ns for sample in all_samples)
        geometry_sources = self._source_geometry()
        rows = []
        for sample in sorted(all_samples, key=lambda item: (item.received_ns, item.topic)):
            states = sorted(
                {
                    STATE_NAMES.get(int(hazard.state), str(int(hazard.state)))
                    for hazard in sample.message.hazards
                }
            )
            source_uavs = sorted(
                {
                    str(source)
                    for hazard in sample.message.hazards
                    for source in hazard.source_uavs
                }
            )
            selected = set()
            if sample.topic == DJI0_TOPIC:
                for hazard in sample.message.hazards:
                    matches = geometry_sources.get(hazard_geometry_signature(hazard), set())
                    selected.add(next(iter(matches)) if len(matches) == 1 else 'unknown')
            rows.append(
                {
                    'time_s': (sample.received_ns - start_ns) * 1.0e-9,
                    'topic': sample.topic,
                    'hazard_count': len(sample.message.hazards),
                    'states': ','.join(states),
                    'source_uavs': ','.join(source_uavs),
                    'selected_sources': ','.join(sorted(selected)),
                }
            )
        return rows

    def summarize(self, expectations: EvidenceExpectations) -> dict[str, Any]:
        required_topics = list(expectations.required_topics)
        if not required_topics:
            required_topics = [DJI1_TOPIC, DJI0_TOPIC, UGV_TOPIC]
            if expectations.require_dji2:
                required_topics.insert(1, DJI2_TOPIC)
        nonempty_counts = {
            topic: sum(1 for sample in queue if sample.message.hazards)
            for topic, queue in self.samples.items()
        }
        typed_flow = all(nonempty_counts[topic] > 0 for topic in required_topics)
        dji0_hazards = [
            hazard
            for sample in self.samples[DJI0_TOPIC]
            for hazard in sample.message.hazards
        ]
        states_seen = sorted(
            {STATE_NAMES.get(int(hazard.state), str(int(hazard.state))) for hazard in dji0_hazards}
        )
        source_uavs_seen = sorted(
            {str(source) for hazard in dji0_hazards for source in hazard.source_uavs}
        )
        selected_counts, selected_latest, matched_hazards = self._selected_sources()
        max_age_s, invalid_age_count = self._age_metrics()
        covariance_preserved = self._covariance_preserved()
        forwarding_preserved = self._forwarding_preserved()
        promotion = self._confirmation_promotion()
        expiry_cleared = self._expiry_cleared()
        conflict_seen = 'CONFLICT' in states_seen
        maximum_hazard_count = max(
            (len(sample.message.hazards) for sample in self.samples[DJI0_TOPIC]),
            default=0,
        )

        failures = []
        if expectations.require_typed_flow and not typed_flow:
            failures.append('typed_flow_incomplete')
        if expectations.require_forwarding and forwarding_preserved is not True:
            failures.append('dji0_to_ugv_forwarding_not_preserved')
        if expectations.require_covariance_match and covariance_preserved is not True:
            failures.append('selected_covariance_not_matched_to_source')
        if invalid_age_count:
            failures.append('invalid_acquisition_age')
        if max_age_s is not None and max_age_s > expectations.max_age_s:
            failures.append('acquisition_age_exceeded')
        if maximum_hazard_count < expectations.minimum_hazard_count:
            failures.append('minimum_hazard_count_not_met')
        if expectations.expected_state is not None:
            expected_name = STATE_NAMES[expectations.expected_state]
            if expected_name not in states_seen:
                failures.append(f'expected_state_not_seen:{expected_name}')
        if expectations.expected_sources and not set(expectations.expected_sources).issubset(
            source_uavs_seen
        ):
            failures.append('expected_source_uavs_not_retained')
        if (
            expectations.expected_selected_source
            and selected_latest != expectations.expected_selected_source
        ):
            failures.append('expected_selected_source_not_latest')
        if expectations.require_confirmation_promotion and not promotion:
            failures.append('confirmation_promotion_not_seen')
        if expectations.require_conflict and not conflict_seen:
            failures.append('conflict_not_seen')
        if expectations.require_expiry and not expiry_cleared:
            failures.append('expiry_empty_array_not_seen')
        if expectations.require_costmap and self.costmap_count < 1:
            failures.append('costmap_topic_not_recorded')
        if any(self.dropped_counts.values()):
            failures.append('sample_limit_exceeded')

        return {
            'schema_version': 1,
            'status': 'pass' if not failures else 'fail',
            'validated_scope': 'typed_support_hazard_contract',
            'topic_counts': dict(self.total_counts),
            'nonempty_topic_counts': nonempty_counts,
            'dropped_sample_counts': dict(self.dropped_counts),
            'typed_flow_complete': typed_flow,
            'states_seen': states_seen,
            'confirmation_promotion_seen': promotion,
            'conflict_seen': conflict_seen,
            'expiry_empty_array_seen': expiry_cleared,
            'source_uavs_seen': source_uavs_seen,
            'selected_source_counts': selected_counts,
            'selected_source_latest': selected_latest or 'unknown',
            'selected_source_matched_hazard_count': matched_hazards,
            'covariance_preserved_from_source': covariance_preserved,
            'dji0_to_ugv_forwarding_preserved': forwarding_preserved,
            'dji0_to_ugv_exact_nonempty_match_count': self._forwarding_match_count(),
            'maximum_dji0_hazard_count': maximum_hazard_count,
            'maximum_acquisition_age_s': max_age_s,
            'invalid_acquisition_age_count': invalid_age_count,
            'costmap_message_count': self.costmap_count,
            'failures': failures,
            'limitations': [
                'Contract evidence does not establish detector accuracy.',
                'Costmap topic presence alone does not establish navigation behavior.',
                'No closed-loop navigation or quantitative safety claim is inferred.',
            ],
        }


class PlannerEvidenceCollector(EvidenceCollector):
    """Extend typed-flow evidence with bounded raw costmaps and planner results."""

    def __init__(self, *, max_samples_per_topic: int = 5000, max_costmaps: int = 4) -> None:
        super().__init__(max_samples_per_topic=max_samples_per_topic)
        self.costmaps: deque[GridSnapshot] = deque(maxlen=max_costmaps)
        self.costmap_full_count = 0
        self.costmap_update_count = 0
        self.plan_topic_count = 0
        self.plan_records: list[PlanRecord] = []

    def add_full_costmap(
        self, message: Costmap, received_ns: int, *, source_kind: str = 'full'
    ) -> None:
        metadata = message.metadata
        expected = int(metadata.size_x) * int(metadata.size_y)
        if len(message.data) != expected or expected == 0:
            return
        self.costmap_count += 1
        self.costmap_full_count += 1
        self.costmaps.append(GridSnapshot(
            received_ns=int(received_ns),
            source_kind=source_kind,
            resolution=float(metadata.resolution),
            size_x=int(metadata.size_x),
            size_y=int(metadata.size_y),
            origin_x=float(metadata.origin.position.x),
            origin_y=float(metadata.origin.position.y),
            data=bytes(message.data),
        ))

    def add_costmap_update(self, message: CostmapUpdate, received_ns: int) -> None:
        if not self.costmaps:
            return
        previous = self.costmaps[-1]
        width = int(message.size_x)
        height = int(message.size_y)
        if width * height != len(message.data):
            return
        if int(message.x) + width > previous.size_x or int(message.y) + height > previous.size_y:
            return
        data = bytearray(previous.data)
        for row in range(height):
            source_start = row * width
            target_start = (int(message.y) + row) * previous.size_x + int(message.x)
            data[target_start:target_start + width] = bytes(
                message.data[source_start:source_start + width]
            )
        self.costmap_count += 1
        self.costmap_update_count += 1
        self.costmaps.append(GridSnapshot(
            received_ns=int(received_ns),
            source_kind='update',
            resolution=previous.resolution,
            size_x=previous.size_x,
            size_y=previous.size_y,
            origin_x=previous.origin_x,
            origin_y=previous.origin_y,
            data=bytes(data),
        ))

    def latest_costmap(self) -> GridSnapshot | None:
        return self.costmaps[-1] if self.costmaps else None

    def add_plan(self, record: PlanRecord) -> None:
        self.plan_records.append(record)

    def add_plan_topic_message(self) -> None:
        self.plan_topic_count += 1

    def hazard_rows(self, *, covariance_sigma_scale: float = 2.0) -> list[dict[str, Any]]:
        samples = [sample for queue in self.samples.values() for sample in queue]
        if not samples:
            return []
        start_ns = min(sample.received_ns for sample in samples)
        rows: list[dict[str, Any]] = []
        for sample in sorted(samples, key=lambda value: (value.received_ns, value.topic)):
            publication_ns = stamp_ns(sample.message.header.stamp)
            if not sample.message.hazards:
                rows.append({
                    'time_s': (sample.received_ns - start_ns) * 1.0e-9,
                    'topic': sample.topic,
                    'snapshot_kind': 'empty',
                })
                continue
            for hazard in sample.message.hazards:
                detection = hazard.detection
                acquisition_ns = stamp_ns(detection.header.stamp)
                geometry = effective_hazard_geometry(
                    hazard, covariance_sigma_scale=covariance_sigma_scale
                )
                result = detection.results[0] if detection.results else None
                rows.append({
                    'time_s': (sample.received_ns - start_ns) * 1.0e-9,
                    'topic': sample.topic,
                    'snapshot_kind': 'hazard',
                    'track_id': str(detection.id),
                    'source_uavs': ','.join(str(item) for item in hazard.source_uavs),
                    'state': STATE_NAMES.get(int(hazard.state), str(int(hazard.state))),
                    'class_id': str(result.hypothesis.class_id) if result else '',
                    'confidence': float(result.hypothesis.score) if result else None,
                    'support_quality': float(hazard.support_quality),
                    'acquisition_ns': acquisition_ns,
                    'publication_ns': publication_ns,
                    'observation_ns': stamp_ns(hazard.last_seen),
                    'received_ns': sample.received_ns,
                    'age_at_publication_s': (publication_ns - acquisition_ns) * 1.0e-9,
                    'age_at_receipt_s': (sample.received_ns - acquisition_ns) * 1.0e-9,
                    'ttl_s': stamp_ns(hazard.ttl) * 1.0e-9,
                    'covariance_x_m2': geometry['variance_x'],
                    'covariance_y_m2': geometry['variance_y'],
                    'nominal_size_x_m': geometry['nominal_size_x'],
                    'nominal_size_y_m': geometry['nominal_size_y'],
                    'effective_size_x_m': geometry['effective_size_x'],
                    'effective_size_y_m': geometry['effective_size_y'],
                    'center_x': geometry['center_x'],
                    'center_y': geometry['center_y'],
                    'provenance': str(hazard.provenance),
                })
        return rows


def plan_record_dict(
    record: PlanRecord,
    *,
    geometry: dict[str, float],
    inflation_geometry: dict[str, float],
    costmap: GridSnapshot | None,
    baseline_costmap: GridSnapshot | None,
) -> dict[str, Any]:
    result = {
        'label': record.label,
        'request_ns': record.requested_ns,
        'result_ns': record.received_ns,
        'response_latency_s': (record.received_ns - record.requested_ns) * 1.0e-9,
        'reported_planning_time_s': record.planning_time_s,
        'error_code': record.error_code,
        'error_message': record.error_message,
        'path_pose_count': len(record.points),
        'path_length_m': path_length(record.points),
        'path_geometry_sha256': _path_geometry_hash(record.points),
        'points': [list(point) for point in record.points],
    }
    result.update(path_hazard_metrics(record.points, geometry))
    result['minimum_distance_to_covariance_footprint_m'] = result[
        'minimum_distance_to_effective_hazard_m'
    ]
    result['crosses_covariance_footprint'] = result['crosses_effective_hazard']
    inflation_metrics = path_hazard_metrics(record.points, inflation_geometry)
    result['minimum_distance_to_inflation_region_m'] = inflation_metrics[
        'minimum_distance_to_effective_hazard_m'
    ]
    result['crosses_inflation_region'] = inflation_metrics['crosses_effective_hazard']
    if costmap is not None and record.points:
        result.update(path_cost_exposure(
            record.points,
            costmap,
            geometry,
            inflation_geometry,
            baseline_costmap,
        ))
    else:
        result.update({
            'crosses_lethal_costmap_cell': None,
            'graded_inflated_cost_sample_count': None,
            'graded_inflated_cost_unique_cell_count': None,
            'graded_inflated_cost_path_length_m': None,
        })
    return result


def _planner_overlay_svg(
    plans: list[dict[str, Any]], geometry: dict[str, float]
) -> str:
    all_points = [tuple(point) for plan in plans for point in plan.get('points', [])]
    half_x = 0.5 * geometry['effective_size_x']
    half_y = 0.5 * geometry['effective_size_y']
    all_points.extend([
        (geometry['center_x'] - half_x, geometry['center_y'] - half_y),
        (geometry['center_x'] + half_x, geometry['center_y'] + half_y),
    ])
    if not all_points:
        return '<svg xmlns="http://www.w3.org/2000/svg" width="900" height="650"/>\n'
    min_x = min(point[0] for point in all_points) - 2.0
    max_x = max(point[0] for point in all_points) + 2.0
    min_y = min(point[1] for point in all_points) - 2.0
    max_y = max(point[1] for point in all_points) + 2.0
    width, height, pad = 900, 650, 45

    def project(point):
        x = pad + (point[0] - min_x) / max(max_x - min_x, 1.0e-9) * (width - 2 * pad)
        y = height - pad - (point[1] - min_y) / max(max_y - min_y, 1.0e-9) * (height - 2 * pad)
        return x, y

    colors = {'baseline': '#377eb8', 'hazard_active': '#e41a1c', 'post_clear': '#4daf4a'}
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="20" y="25" font-family="sans-serif" font-size="16">Planner-only evidence</text>',
    ]
    nominal = dict(geometry)
    nominal['effective_size_x'] = geometry['nominal_size_x']
    nominal['effective_size_y'] = geometry['nominal_size_y']
    for footprint, color, label in (
        (geometry, '#ff9896', 'effective'),
        (nominal, '#ff0000', 'nominal'),
    ):
        hx = 0.5 * footprint['effective_size_x']
        hy = 0.5 * footprint['effective_size_y']
        corners = []
        cosine = math.cos(footprint['yaw'])
        sine = math.sin(footprint['yaw'])
        for local_x, local_y in ((-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy)):
            corners.append(project((
                footprint['center_x'] + cosine * local_x - sine * local_y,
                footprint['center_y'] + sine * local_x + cosine * local_y,
            )))
        encoded = ' '.join(f'{x:.2f},{y:.2f}' for x, y in corners)
        parts.append(
            f'<polygon points="{encoded}" fill="none" stroke="{color}" '
            f'stroke-width="3"><title>{label} hazard footprint</title></polygon>'
        )
    for plan in plans:
        points = [project(tuple(point)) for point in plan.get('points', [])]
        if not points:
            continue
        encoded = ' '.join(f'{x:.2f},{y:.2f}' for x, y in points)
        color = colors.get(str(plan.get('label')), '#555555')
        parts.append(
            f'<polyline points="{encoded}" fill="none" stroke="{color}" stroke-width="3"/>'
        )
    parts.append('</svg>')
    return '\n'.join(parts) + '\n'


def write_planner_evidence(
    output_dir: Path,
    summary: dict[str, Any],
    hazard_rows: list[dict[str, Any]],
    costmap_rows: list[dict[str, Any]],
) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    targets = (
        output_dir / 'summary.json',
        output_dir / 'hazard_timeline.csv',
        output_dir / 'costmap_timeline.csv',
        output_dir / 'plans.json',
        output_dir / 'planner_overlay.svg',
    )
    if any(path.exists() for path in targets):
        raise FileExistsError(f'evidence output already exists in {output_dir}')
    summary['generated_at'] = datetime.now(timezone.utc).isoformat()
    (output_dir / 'summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n', encoding='utf-8'
    )
    (output_dir / 'plans.json').write_text(
        json.dumps(summary.get('planner', {}).get('plans', []), indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    for filename, rows in (
        ('hazard_timeline.csv', hazard_rows),
        ('costmap_timeline.csv', costmap_rows),
    ):
        with (output_dir / filename).open('w', newline='', encoding='utf-8') as stream:
            fieldnames = sorted({key for row in rows for key in row})
            writer = csv.DictWriter(stream, fieldnames=fieldnames)
            if fieldnames:
                writer.writeheader()
                writer.writerows(rows)
    (output_dir / 'planner_overlay.svg').write_text(
        _planner_overlay_svg(
            summary.get('planner', {}).get('plans', []), summary['hazard_geometry']
        ),
        encoding='utf-8',
    )


def _flatten_summary(summary: dict[str, Any]) -> dict[str, Any]:
    return {
        key: json.dumps(value, sort_keys=True) if isinstance(value, (dict, list)) else value
        for key, value in summary.items()
    }


def _timeline_svg(rows: list[dict[str, Any]]) -> str:
    width = 960
    height = 280
    left = 280
    right = 30
    topics = list(HAZARD_TOPICS)
    maximum_time = max((float(row['time_s']) for row in rows), default=1.0)
    maximum_time = max(maximum_time, 1.0e-6)
    lane_height = 50
    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">',
        '<rect width="100%" height="100%" fill="white"/>',
        '<text x="20" y="24" font-family="sans-serif" font-size="16">'
        'Typed hazard evidence timeline</text>',
    ]
    for index, topic in enumerate(topics):
        y = 55 + index * lane_height
        parts.append(
            f'<text x="10" y="{y + 5}" font-family="monospace" font-size="11">{topic}</text>'
        )
        parts.append(
            f'<line x1="{left}" y1="{y}" x2="{width - right}" y2="{y}" stroke="#999"/>'
        )
    for row in rows:
        topic_index = topics.index(str(row['topic']))
        y = 55 + topic_index * lane_height
        x = left + (width - left - right) * float(row['time_s']) / maximum_time
        count = int(row['hazard_count'])
        color = '#2b8cbe' if count else '#bdbdbd'
        if 'CONFLICT' in str(row['states']):
            color = '#d7301f'
        elif 'CONFIRMED' in str(row['states']):
            color = '#238b45'
        parts.append(f'<circle cx="{x:.2f}" cy="{y}" r="4" fill="{color}"/>')
    parts.append(
        f'<text x="{left}" y="{height - 12}" font-family="sans-serif" font-size="11">0 s</text>'
    )
    parts.append(
        f'<text x="{width - right - 55}" y="{height - 12}" '
        f'font-family="sans-serif" font-size="11">{maximum_time:.2f} s</text>'
    )
    parts.append('</svg>')
    return '\n'.join(parts) + '\n'


def write_evidence(output_dir: Path, summary: dict[str, Any], rows: list[dict[str, Any]]) -> None:
    output_dir.mkdir(parents=True, exist_ok=True)
    targets = (
        output_dir / 'summary.json',
        output_dir / 'summary.csv',
        output_dir / 'timeline.csv',
        output_dir / 'timeline.svg',
    )
    if any(path.exists() for path in targets):
        raise FileExistsError(f'evidence output already exists in {output_dir}')
    summary['generated_at'] = datetime.now(timezone.utc).isoformat()
    (output_dir / 'summary.json').write_text(
        json.dumps(summary, indent=2, sort_keys=True) + '\n',
        encoding='utf-8',
    )
    flat = _flatten_summary(summary)
    with (output_dir / 'summary.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=list(flat))
        writer.writeheader()
        writer.writerow(flat)
    timeline_fields = (
        'time_s',
        'topic',
        'hazard_count',
        'states',
        'source_uavs',
        'selected_sources',
    )
    with (output_dir / 'timeline.csv').open('w', newline='', encoding='utf-8') as stream:
        writer = csv.DictWriter(stream, fieldnames=timeline_fields)
        writer.writeheader()
        writer.writerows(rows)
    (output_dir / 'timeline.svg').write_text(_timeline_svg(rows), encoding='utf-8')


class LiveEvidenceNode(Node):
    def __init__(self, collector: EvidenceCollector) -> None:
        super().__init__('support_hazard_evidence')
        self.collector = collector
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._subscriptions = [
            self.create_subscription(
                AerialHazardArray,
                topic,
                lambda message, topic=topic: self.collector.add(
                    topic,
                    message,
                    int(self.get_clock().now().nanoseconds),
                ),
                qos,
            )
            for topic in HAZARD_TOPICS
        ]


class PlannerEvidenceNode(LiveEvidenceNode):
    def __init__(self, collector: PlannerEvidenceCollector, namespace: str) -> None:
        super().__init__(collector)
        self.collector = collector
        prefix = '/' + namespace.strip('/')
        full_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        update_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._costmap_subscription = self.create_subscription(
            Costmap,
            f'{prefix}/global_costmap/costmap_raw',
            lambda message: collector.add_full_costmap(
                message, int(self.get_clock().now().nanoseconds)
            ),
            full_qos,
        )
        self._costmap_update_subscription = self.create_subscription(
            CostmapUpdate,
            f'{prefix}/global_costmap/costmap_raw_updates',
            lambda message: collector.add_costmap_update(
                message, int(self.get_clock().now().nanoseconds)
            ),
            update_qos,
        )
        self._plan_subscription = self.create_subscription(
            NavPath,
            f'{prefix}/plan',
            lambda message: collector.add_plan_topic_message(),
            update_qos,
        )
        self.planner_client = ActionClient(
            self, ComputePathToPose, f'{prefix}/compute_path_to_pose'
        )
        self.layer_parameters = AsyncParameterClient(
            self, f'{prefix}/global_costmap/global_costmap'
        )
        self.costmap_client = self.create_client(
            GetCostmap, f'{prefix}/global_costmap/get_costmap'
        )


def _spin_until(node: Node, predicate, deadline: float) -> bool:
    while time.monotonic() < deadline:
        if predicate():
            return True
        rclpy.spin_once(node, timeout_sec=0.1)
    return bool(predicate())


def _pose(node: Node, x: float, y: float, yaw: float) -> PoseStamped:
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(float(yaw) * 0.5)
    pose.pose.orientation.w = math.cos(float(yaw) * 0.5)
    return pose


def _request_plan(
    node: PlannerEvidenceNode,
    *,
    label: str,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    timeout_s: float,
) -> PlanRecord:
    requested_ns = int(node.get_clock().now().nanoseconds)
    goal_message = ComputePathToPose.Goal()
    goal_message.start = _pose(node, *start)
    goal_message.goal = _pose(node, *goal)
    goal_message.planner_id = 'GridBased'
    goal_message.use_start = True
    send_future = node.planner_client.send_goal_async(goal_message)
    deadline = time.monotonic() + timeout_s
    if not _spin_until(node, send_future.done, deadline):
        return PlanRecord(label, requested_ns, int(node.get_clock().now().nanoseconds), 0.0,
                          207, 'goal request timed out', ())
    goal_handle = send_future.result()
    if goal_handle is None or not goal_handle.accepted:
        return PlanRecord(label, requested_ns, int(node.get_clock().now().nanoseconds), 0.0,
                          200, 'planner rejected goal', ())
    result_future = goal_handle.get_result_async()
    if not _spin_until(node, result_future.done, deadline):
        goal_handle.cancel_goal_async()
        return PlanRecord(label, requested_ns, int(node.get_clock().now().nanoseconds), 0.0,
                          207, 'planner result timed out', ())
    wrapped = result_future.result()
    result = wrapped.result
    received_ns = int(node.get_clock().now().nanoseconds)
    points = tuple(
        (float(item.pose.position.x), float(item.pose.position.y))
        for item in result.path.poses
    )
    return PlanRecord(
        label=label,
        requested_ns=requested_ns,
        received_ns=received_ns,
        planning_time_s=stamp_ns(result.planning_time) * 1.0e-9,
        error_code=int(result.error_code),
        error_message=str(result.error_msg),
        points=points,
    )


def _set_aerial_layer(node: PlannerEvidenceNode, enabled: bool, timeout_s: float) -> bool:
    if not node.layer_parameters.wait_for_services(timeout_sec=timeout_s):
        return False
    future = node.layer_parameters.set_parameters([
        Parameter('aerial_support_layer.enabled', Parameter.Type.BOOL, enabled)
    ])
    if not _spin_until(node, future.done, time.monotonic() + timeout_s):
        return False
    response = future.result()
    results = response.results if response is not None else ()
    return bool(results) and all(result.successful for result in results)


def _request_costmap_snapshot(node: PlannerEvidenceNode, timeout_s: float) -> bool:
    if not node.costmap_client.wait_for_service(timeout_sec=timeout_s):
        return False
    future = node.costmap_client.call_async(GetCostmap.Request())
    if not _spin_until(node, future.done, time.monotonic() + timeout_s):
        return False
    response = future.result()
    if response is None:
        return False
    node.collector.add_full_costmap(
        response.map,
        int(node.get_clock().now().nanoseconds),
        source_kind='service',
    )
    return True


def _has_nonempty(collector: EvidenceCollector, topic: str) -> bool:
    return any(sample.message.hazards for sample in collector.samples[topic])


def _has_empty_after_nonempty(collector: EvidenceCollector, topic: str) -> bool:
    seen = False
    for sample in collector._ordered(topic):
        if sample.message.hazards:
            seen = True
        elif seen:
            return True
    return False


def _costmap_rows(
    snapshots: list[tuple[str, GridSnapshot | None]],
    baseline: GridSnapshot,
    geometry: dict[str, float],
    inflation_radius_m: float,
) -> list[dict[str, Any]]:
    rows = []
    for label, snapshot in snapshots:
        if snapshot is None:
            continue
        delta = relevant_costmap_delta(
            baseline, snapshot, geometry, inflation_radius_m
        )
        rows.append({
            'label': label,
            'received_ns': snapshot.received_ns,
            'source_kind': snapshot.source_kind,
            'resolution': snapshot.resolution,
            'origin_x': snapshot.origin_x,
            'origin_y': snapshot.origin_y,
            'comparable': delta['comparable'],
            'affected_cells': delta['affected_cells'],
            'lethal_cells': delta['lethal_cells'],
            'maximum_cost': delta['maximum_cost'],
            'hazard_footprint_affected_cells': delta.get(
                'hazard_footprint_affected_cells', 0
            ),
            'hazard_footprint_lethal_cells': delta.get(
                'hazard_footprint_lethal_cells', 0
            ),
            'inflation_halo_affected_cells': delta.get(
                'inflation_halo_affected_cells', 0
            ),
            'inflation_halo_nonzero_cells': delta.get(
                'inflation_halo_nonzero_cells', 0
            ),
            'analysis_region_affected_cells': delta.get(
                'analysis_region_affected_cells', 0
            ),
            'analysis_region_raw_changed_cells': delta.get(
                'analysis_region_raw_changed_cells', 0
            ),
            'no_information_transition_cells': delta.get(
                'no_information_transition_cells', 0
            ),
            'relevant_cost_values_json': json.dumps(delta.get('relevant_cost_values', [])),
            'inflation_halo_cost_values_json': json.dumps(
                delta.get('inflation_halo_cost_values', [])
            ),
            'hazard_footprint_cost_class_counts_json': json.dumps(
                delta.get('hazard_footprint_cost_class_counts', {})
            ),
            'inflation_halo_cost_class_counts_json': json.dumps(
                delta.get('inflation_halo_cost_class_counts', {})
            ),
            'affected_cell_values_json': json.dumps(delta.get('affected_cell_values', [])),
        })
    return rows


def _clearing_mechanism(
    collector: PlannerEvidenceCollector,
    first_clear_ns: int | None,
    max_observation_age_s: float,
) -> dict[str, Any]:
    explicit_empty = _has_empty_after_nonempty(collector, UGV_TOPIC)
    ugv_nonempty = [
        (sample, hazard)
        for sample in collector.samples[UGV_TOPIC]
        for hazard in sample.message.hazards
    ]
    source_nonempty = [
        sample for sample in collector.samples[DJI1_TOPIC] if sample.message.hazards
    ]
    source_silence = bool(source_nonempty) and not _has_empty_after_nonempty(
        collector, DJI1_TOPIC
    )
    mechanism = 'not_observed'
    ttl_deadline_ns = None
    age_deadline_ns = None
    if ugv_nonempty:
        _, hazard = ugv_nonempty[-1]
        ttl_deadline_ns = stamp_ns(hazard.last_seen) + stamp_ns(hazard.ttl)
        age_deadline_ns = stamp_ns(hazard.detection.header.stamp) + int(
            max_observation_age_s * 1.0e9
        )
    if explicit_empty and first_clear_ns is not None:
        mechanism = 'explicit_empty_snapshot'
    elif ugv_nonempty and first_clear_ns is not None:
        if age_deadline_ns <= ttl_deadline_ns and first_clear_ns >= age_deadline_ns:
            mechanism = 'observation_age_expiry'
        elif first_clear_ns >= ttl_deadline_ns:
            mechanism = 'ttl_expiry'
        elif source_silence:
            mechanism = 'source_silence'
    return {
        'observed_mechanism': mechanism,
        'explicit_empty_snapshot_seen': explicit_empty,
        'source_silence_seen': source_silence,
        'observation_age_deadline_ns': age_deadline_ns,
        'ttl_deadline_ns': ttl_deadline_ns,
        'first_observed_clear_ns': first_clear_ns,
    }


def _ros_package_version(package_name: str) -> str | None:
    package_xml = Path('/opt/ros/jazzy/share') / package_name / 'package.xml'
    try:
        root = ET.parse(package_xml).getroot()
    except (FileNotFoundError, ET.ParseError):
        return None
    version = root.findtext('version')
    return version.strip() if version else None


def _run_planner_live(args: argparse.Namespace, ros_args: list[str]) -> int:
    if args.scenario not in {
        'baseline', 'valid', 'clearing', 'off_route', 'low_confidence', 'stale',
        'layer_disabled',
    }:
        raise ValueError(f'unsupported planner scenario: {args.scenario}')
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        raise ValueError('--timeout-s must be finite and greater than zero')
    if args.baseline_repeat_count < 2:
        raise ValueError('--baseline-repeat-count must be at least two')
    if (
        not math.isfinite(args.baseline_settle_timeout_s)
        or args.baseline_settle_timeout_s <= 0.0
    ):
        raise ValueError('--baseline-settle-timeout-s must be finite and positive')
    if args.baseline_stable_snapshots < 2:
        raise ValueError('--baseline-stable-snapshots must be at least two')
    nav2_inflation = load_nav2_inflation_config(args.nav2_config)
    collector = PlannerEvidenceCollector(max_samples_per_topic=args.max_samples_per_topic)
    rclpy.init(args=ros_args)
    node = PlannerEvidenceNode(collector, args.namespace)
    deadline = time.monotonic() + float(args.timeout_s)
    failures: list[str] = []
    covariance_sigma_scale = nav2_inflation['aerial_covariance_sigma_scale']
    uncertainty_per_side = covariance_sigma_scale * math.sqrt(0.25)
    geometry = {
        'center_x': float(args.hazard_x),
        'center_y': float(args.hazard_y),
        'yaw': 0.0,
        'nominal_size_x': 2.0,
        'nominal_size_y': 2.0,
        'variance_x': 0.25,
        'variance_y': 0.25,
        'covariance_sigma_scale': covariance_sigma_scale,
        'uncertainty_per_side_m': uncertainty_per_side,
        'effective_size_x': 2.0 + 2.0 * uncertainty_per_side,
        'effective_size_y': 2.0 + 2.0 * uncertainty_per_side,
    }
    inflation_geometry = expanded_hazard_geometry(
        geometry, nav2_inflation['inflation_radius_m']
    )
    start = (float(args.start_x), float(args.start_y), float(args.start_yaw))
    goal = (float(args.goal_x), float(args.goal_y), float(args.goal_yaw))
    baseline_map = None
    active_map = None
    clear_map = None
    first_mark_ns = None
    first_clear_ns = None
    plan_maps: dict[str, GridSnapshot | None] = {}
    baseline_stability: dict[str, Any] = {
        'status': 'not_evaluated',
        'required_consecutive_snapshots': args.baseline_stable_snapshots,
        'observations': [],
    }
    costmap_service_snapshot_count = 0
    try:
        if not node.planner_client.wait_for_server(timeout_sec=min(20.0, args.timeout_s)):
            failures.append('planner_action_unavailable')
        for _ in range(args.baseline_stable_snapshots):
            if _request_costmap_snapshot(
                node, min(5.0, max(0.1, deadline - time.monotonic()))
            ):
                costmap_service_snapshot_count += 1
            else:
                break
        settle_deadline = min(
            deadline, time.monotonic() + args.baseline_settle_timeout_s
        )
        settled = _spin_until(
            node,
            lambda: settled_baseline_selection(
                collector.costmaps,
                geometry,
                nav2_inflation['inflation_radius_m'],
                required_consecutive=args.baseline_stable_snapshots,
            )[0] is not None,
            settle_deadline,
        )
        baseline_map, baseline_stability = settled_baseline_selection(
            collector.costmaps,
            geometry,
            nav2_inflation['inflation_radius_m'],
            required_consecutive=args.baseline_stable_snapshots,
        )
        if not collector.costmaps:
            failures.append('full_costmap_unavailable')
        elif not settled or baseline_map is None:
            failures.append('baseline_costmap_not_settled')
        if not failures:
            repeat_count = args.baseline_repeat_count if args.scenario == 'baseline' else 1
            for index in range(repeat_count):
                label = 'baseline' if index == 0 else f'baseline_repeat_{index + 1}'
                baseline_plan = _request_plan(
                    node, label=label, start=start, goal=goal,
                    timeout_s=min(15.0, max(1.0, deadline - time.monotonic())),
                )
                collector.add_plan(baseline_plan)
                current_map = collector.latest_costmap() or baseline_map
                if index == 0:
                    baseline_map = current_map
                plan_maps[label] = current_map

        if args.scenario != 'baseline' and not failures:
            if args.scenario != 'layer_disabled':
                if not _set_aerial_layer(node, True, min(10.0, args.timeout_s)):
                    failures.append('aerial_layer_enable_failed')
            if not _spin_until(node, lambda: _has_nonempty(collector, DJI1_TOPIC), deadline):
                failures.append('dji1_hazard_not_observed')
            expects_mark = args.scenario in {'valid', 'clearing', 'off_route'}
            if expects_mark and baseline_map is not None:
                marked = _spin_until(
                    node,
                    lambda: (
                        collector.latest_costmap() is not None
                        and relevant_costmap_delta(
                            baseline_map, collector.latest_costmap(), geometry,
                            nav2_inflation['inflation_radius_m'],
                        )['affected_cells'] > 0
                        and relevant_costmap_delta(
                            baseline_map, collector.latest_costmap(), geometry,
                            nav2_inflation['inflation_radius_m'],
                        )['lethal_cells'] > 0
                    ),
                    deadline,
                )
                if not marked:
                    failures.append('aerial_costmap_mark_not_observed')
                else:
                    first_mark_ns = collector.latest_costmap().received_ns
            else:
                quiet_deadline = min(deadline, time.monotonic() + 2.5)
                _spin_until(node, lambda: False, quiet_deadline)
            active_map = collector.latest_costmap()
            active_plan = _request_plan(
                node, label='hazard_active', start=start, goal=goal,
                timeout_s=min(15.0, max(1.0, deadline - time.monotonic())),
            )
            collector.add_plan(active_plan)
            plan_maps['hazard_active'] = active_map

            if args.scenario == 'clearing' and baseline_map is not None:
                cleared = _spin_until(
                    node,
                    lambda: (
                        _has_empty_after_nonempty(collector, UGV_TOPIC)
                        and collector.latest_costmap() is not None
                        and relevant_costmap_delta(
                            baseline_map, collector.latest_costmap(), geometry,
                            nav2_inflation['inflation_radius_m'],
                        )['analysis_region_affected_cells'] == 0
                    ),
                    deadline,
                )
                if not cleared:
                    failures.append('aerial_costmap_clear_not_observed')
                clear_map = collector.latest_costmap()
                if cleared and clear_map is not None:
                    first_clear_ns = clear_map.received_ns
                post_clear = _request_plan(
                    node, label='post_clear', start=start, goal=goal,
                    timeout_s=min(15.0, max(1.0, deadline - time.monotonic())),
                )
                collector.add_plan(post_clear)
                plan_maps['post_clear'] = clear_map
    finally:
        records = list(collector.plan_records)
        plans = [
            plan_record_dict(
                record,
                geometry=geometry,
                inflation_geometry=inflation_geometry,
                costmap=plan_maps.get(record.label),
                baseline_costmap=baseline_map,
            )
            for record in records
        ]
        for plan in plans:
            if plan['error_code'] != 0 or plan['path_pose_count'] < 2:
                failures.append(f"planner_failed:{plan['label']}")
        by_label = {record.label: record for record in records}
        baseline_record = by_label.get('baseline')
        active_record = by_label.get('hazard_active')
        post_record = by_label.get('post_clear')
        active_change = (
            discrete_hausdorff_distance(baseline_record.points, active_record.points)
            if baseline_record and active_record else None
        )
        post_change = (
            discrete_hausdorff_distance(baseline_record.points, post_record.points)
            if baseline_record and post_record else None
        )
        active_delta = (
            relevant_costmap_delta(
                baseline_map, active_map, geometry,
                nav2_inflation['inflation_radius_m'],
            )
            if baseline_map is not None and active_map is not None else None
        )
        clear_delta = (
            relevant_costmap_delta(
                baseline_map, clear_map, geometry,
                nav2_inflation['inflation_radius_m'],
            )
            if baseline_map is not None and clear_map is not None else None
        )
        repeatability = baseline_repeatability(
            records,
            required_count=(args.baseline_repeat_count if args.scenario == 'baseline' else 1),
            tolerance_m=args.baseline_path_tolerance_m,
        )
        if args.scenario == 'baseline':
            typed_expectations = EvidenceExpectations(
                minimum_hazard_count=0,
                require_typed_flow=False,
                require_forwarding=False,
                require_covariance_match=False,
            )
        elif args.scenario == 'stale':
            typed_expectations = EvidenceExpectations(
                minimum_hazard_count=0,
                required_topics=(DJI1_TOPIC,),
                require_forwarding=False,
                require_covariance_match=False,
            )
        else:
            typed_expectations = EvidenceExpectations()
        typed = collector.summarize(typed_expectations)
        dji1_hazards = [
            (sample, hazard)
            for sample in collector.samples[DJI1_TOPIC]
            for hazard in sample.message.hazards
        ]
        dji1_confidences = [
            float(hazard.detection.results[0].hypothesis.score)
            for _, hazard in dji1_hazards if hazard.detection.results
        ]
        dji1_ages_at_receipt = [
            (sample.received_ns - stamp_ns(hazard.detection.header.stamp)) * 1.0e-9
            for sample, hazard in dji1_hazards
        ]
        if args.scenario == 'baseline':
            if repeatability['status'] != 'pass':
                failures.extend(repeatability['failures'])
            if baseline_map is None:
                failures.append('baseline_costmap_missing')
            else:
                for label, snapshot in plan_maps.items():
                    if not label.startswith('baseline') or snapshot is None:
                        continue
                    delta = relevant_costmap_delta(
                        baseline_map, snapshot, geometry,
                        nav2_inflation['inflation_radius_m'],
                    )
                    if (
                        not delta['comparable']
                        or delta['analysis_region_affected_cells'] != 0
                    ):
                        failures.append('baseline_aerial_cost_change_observed')
        elif args.scenario in {'valid', 'clearing'}:
            if typed['status'] != 'pass':
                failures.extend('hazard_flow:' + item for item in typed['failures'])
            if active_change is None or active_change < args.minimum_path_change_m:
                failures.append('active_path_did_not_change')
            baseline_plan = next(
                (item for item in plans if item['label'] == 'baseline'), {}
            )
            active_plan = next(
                (item for item in plans if item['label'] == 'hazard_active'), {}
            )
            if not baseline_plan.get('crosses_effective_hazard'):
                failures.append('baseline_path_misses_effective_hazard')
            if active_plan.get('crosses_effective_hazard'):
                failures.append('active_path_crosses_effective_hazard')
            if active_plan.get('crosses_lethal_costmap_cell'):
                failures.append('active_path_crosses_lethal_cost')
            if active_delta is None or active_delta['lethal_cells'] < 1:
                failures.append('lethal_aerial_cells_not_observed')
            if (
                active_delta is None
                or active_delta['inflation_halo_nonzero_cells'] < 1
            ):
                failures.append('graded_inflation_halo_not_observed')
            if args.scenario == 'clearing':
                clearing = _clearing_mechanism(
                    collector,
                    first_clear_ns,
                    nav2_inflation['aerial_max_observation_age_s'],
                )
                if clearing['observed_mechanism'] != 'explicit_empty_snapshot':
                    failures.append('explicit_clearing_mechanism_not_observed')
                if (
                    clear_delta is None
                    or not clear_delta['comparable']
                    or clear_delta['analysis_region_affected_cells'] != 0
                ):
                    failures.append('post_clear_aerial_costs_remain')
                if post_change is None or post_change > args.control_path_tolerance_m:
                    failures.append('post_clear_path_not_near_baseline')
        elif args.scenario == 'off_route':
            if typed['status'] != 'pass':
                failures.extend('hazard_flow:' + item for item in typed['failures'])
            if active_change is None or active_change > args.control_path_tolerance_m:
                failures.append('off_route_hazard_changed_path')
        elif args.scenario in {'low_confidence', 'stale', 'layer_disabled'}:
            if typed['status'] != 'pass':
                failures.extend('hazard_flow:' + item for item in typed['failures'])
            if args.scenario == 'stale' and (
                _has_nonempty(collector, DJI0_TOPIC) or _has_nonempty(collector, UGV_TOPIC)
            ):
                failures.append('stale_hazard_reached_operational_output')
            if args.scenario == 'stale' and not any(
                age > nav2_inflation['aerial_max_observation_age_s']
                for age in dji1_ages_at_receipt
            ):
                failures.append('stale_input_age_not_demonstrated')
            if args.scenario == 'low_confidence' and not (
                dji1_confidences
                and max(dji1_confidences) < nav2_inflation['aerial_min_confidence']
            ):
                failures.append('low_confidence_threshold_not_demonstrated')
            if (
                active_delta is None
                or not active_delta['comparable']
                or active_delta['analysis_region_affected_cells'] != 0
            ):
                failures.append('negative_control_marked_costmap')
            if active_change is None or active_change > args.control_path_tolerance_m:
                failures.append('negative_control_changed_path')
        costmap_rows = (
            _costmap_rows(
                list(plan_maps.items()),
                baseline_map,
                geometry,
                nav2_inflation['inflation_radius_m'],
            )
            if baseline_map is not None else []
        )
        summary = {
            'schema_version': 3,
            'status': 'pass' if not failures else 'fail',
            'validated_scope': 'planner_only_typed_hazard_costmap_response',
            'scenario': args.scenario,
            'time_model': 'wall_time',
            'configuration': {
                'map': str(args.map),
                'nav2_config': nav2_inflation['source_yaml'],
                'nav2_config_sha256': nav2_inflation['source_sha256'],
                'namespace': '/' + args.namespace.strip('/'),
                'planner_action': '/' + args.namespace.strip('/') + '/compute_path_to_pose',
                'aerial_layer_parameter': (
                    '/' + args.namespace.strip('/')
                    + '/global_costmap/global_costmap:aerial_support_layer.enabled'
                ),
                'nav2_bringup_version': _ros_package_version('nav2_bringup'),
            },
            'start': {'x': start[0], 'y': start[1], 'yaw': start[2]},
            'goal': {'x': goal[0], 'y': goal[1], 'yaw': goal[2]},
            'hazard_geometry': {
                **geometry,
                'nominal_footprint': {
                    'size_x_m': geometry['nominal_size_x'],
                    'size_y_m': geometry['nominal_size_y'],
                },
                'covariance_expansion': {
                    'variance_x_m2': geometry['variance_x'],
                    'variance_y_m2': geometry['variance_y'],
                    'sigma_scale': geometry['covariance_sigma_scale'],
                    'per_side_m': geometry['uncertainty_per_side_m'],
                },
                'covariance_expanded_footprint': {
                    'size_x_m': geometry['effective_size_x'],
                    'size_y_m': geometry['effective_size_y'],
                },
                'nav2_inflation': {
                    'inflation_radius_m': nav2_inflation['inflation_radius_m'],
                    'cost_scaling_factor': nav2_inflation['cost_scaling_factor'],
                },
                'inflation_expanded_analysis_region': {
                    'size_x_m': inflation_geometry['effective_size_x'],
                    'size_y_m': inflation_geometry['effective_size_y'],
                },
            },
            'hazard_flow': typed,
            'aerial_layer_configuration': {
                'initial_enabled': False,
                'runtime_enabled_for_scenario': args.scenario not in {
                    'baseline', 'layer_disabled'
                },
                'min_confidence': nav2_inflation['aerial_min_confidence'],
                'max_observation_age_s': nav2_inflation[
                    'aerial_max_observation_age_s'
                ],
                'covariance_sigma_scale': nav2_inflation[
                    'aerial_covariance_sigma_scale'
                ],
            },
            'scenario_input_observations': {
                'dji1_confidences': dji1_confidences,
                'configured_aerial_min_confidence': nav2_inflation[
                    'aerial_min_confidence'
                ],
                'dji1_ages_at_receipt_s': dji1_ages_at_receipt,
                'configured_aerial_max_observation_age_s': nav2_inflation[
                    'aerial_max_observation_age_s'
                ],
            },
            'costmap': {
                'full_message_count': collector.costmap_full_count,
                'update_message_count': collector.costmap_update_count,
                'service_snapshot_count': costmap_service_snapshot_count,
                'baseline_stability': baseline_stability,
                'resolution': baseline_map.resolution if baseline_map else None,
                'origin': [baseline_map.origin_x, baseline_map.origin_y] if baseline_map else None,
                'active_region': active_delta,
                'post_clear_region': clear_delta,
                'first_observed_mark_ns': first_mark_ns,
                'first_observed_clear_ns': first_clear_ns,
            },
            'planner': {
                'plans': plans,
                'plan_topic_message_count': collector.plan_topic_count,
                'active_hausdorff_from_baseline_m': active_change,
                'post_clear_hausdorff_from_baseline_m': post_change,
                'baseline_repeatability': repeatability,
            },
            'clearing': _clearing_mechanism(
                collector,
                first_clear_ns,
                nav2_inflation['aerial_max_observation_age_s'],
            ),
            'failures': sorted(set(failures)),
            'claim_boundaries': [
                'No physical UGV motion was commanded or observed.',
                'ComputePathToPose evidence does not prove NavigateToPose BT replanning.',
                'No goal-completion or real-motion safety claim is made.',
            ],
        }
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    write_planner_evidence(args.output, summary, collector.hazard_rows(), costmap_rows)
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary['status'] == 'pass' else 1


@dataclass(frozen=True)
class OfflineMap:
    width: int
    height: int
    resolution: float
    origin_x: float
    origin_y: float
    occupied_threshold: float
    free_threshold: float
    pixels: bytes

    def world_to_cell(self, x: float, y: float) -> tuple[int, int]:
        return (
            int(math.floor((x - self.origin_x) / self.resolution)),
            int(math.floor((y - self.origin_y) / self.resolution)),
        )

    def cell_to_world(self, x: int, y: int) -> tuple[float, float]:
        return (
            self.origin_x + (x + 0.5) * self.resolution,
            self.origin_y + (y + 0.5) * self.resolution,
        )

    def pixel(self, x: int, y: int) -> int:
        row = self.height - 1 - y
        return int(self.pixels[row * self.width + x])

    def occupied(self, x: int, y: int) -> bool:
        probability = (255.0 - self.pixel(x, y)) / 255.0
        return probability >= self.occupied_threshold

    def unknown(self, x: int, y: int) -> bool:
        probability = (255.0 - self.pixel(x, y)) / 255.0
        return self.free_threshold <= probability < self.occupied_threshold


def _simple_map_yaml(path: Path) -> dict[str, Any]:
    values: dict[str, Any] = {}
    for line in path.read_text(encoding='utf-8').splitlines():
        stripped = line.split('#', 1)[0].strip()
        if not stripped or ':' not in stripped:
            continue
        key, raw = stripped.split(':', 1)
        values[key.strip()] = raw.strip()
    required = {'image', 'resolution', 'origin', 'occupied_thresh', 'free_thresh'}
    missing = sorted(required - set(values))
    if missing:
        raise ValueError(f"map YAML is missing: {', '.join(missing)}")
    return values


def _read_pgm(path: Path) -> tuple[int, int, bytes]:
    raw = path.read_bytes()
    position = 0

    def token() -> bytes:
        nonlocal position
        while position < len(raw):
            if raw[position:position + 1] == b'#':
                while position < len(raw) and raw[position:position + 1] not in b'\r\n':
                    position += 1
            elif raw[position:position + 1].isspace():
                position += 1
            else:
                break
        start = position
        while position < len(raw) and not raw[position:position + 1].isspace():
            position += 1
        return raw[start:position]

    magic = token()
    width = int(token())
    height = int(token())
    maximum = int(token())
    if magic != b'P5' or maximum != 255:
        raise ValueError('only 8-bit binary PGM maps are supported')
    if raw[position:position + 2] == b'\r\n':
        position += 2
    elif position < len(raw) and raw[position:position + 1].isspace():
        position += 1
    else:
        raise ValueError('PGM header is not followed by pixel data')
    pixels = raw[position:]
    if len(pixels) != width * height:
        raise ValueError('PGM pixel count does not match its header')
    return width, height, pixels


def load_offline_map(path: Path) -> OfflineMap:
    yaml_path = path.expanduser().resolve()
    values = _simple_map_yaml(yaml_path)
    image_value = str(values['image']).strip('"\'')
    image_path = Path(image_value)
    if not image_path.is_absolute():
        image_path = yaml_path.parent / image_path
    width, height, pixels = _read_pgm(image_path.resolve())
    origin = ast.literal_eval(str(values['origin']))
    return OfflineMap(
        width=width,
        height=height,
        resolution=float(values['resolution']),
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        occupied_threshold=float(values['occupied_thresh']),
        free_threshold=float(values['free_thresh']),
        pixels=pixels,
    )


def offline_astar(
    grid: OfflineMap,
    start_world: tuple[float, float],
    goal_world: tuple[float, float],
    *,
    blocked_geometry: dict[str, float] | None = None,
    extra_margin_m: float = 0.0,
    search_margin_m: float = 25.0,
) -> list[tuple[float, float]]:
    start = grid.world_to_cell(*start_world)
    goal = grid.world_to_cell(*goal_world)
    margin_cells = int(math.ceil(search_margin_m / grid.resolution))
    min_x = max(0, min(start[0], goal[0]) - margin_cells)
    max_x = min(grid.width - 1, max(start[0], goal[0]) + margin_cells)
    min_y = max(0, min(start[1], goal[1]) - margin_cells)
    max_y = min(grid.height - 1, max(start[1], goal[1]) + margin_cells)

    def blocked(cell: tuple[int, int]) -> bool:
        x, y = cell
        if x < min_x or x > max_x or y < min_y or y > max_y or grid.occupied(x, y):
            return True
        if blocked_geometry is None:
            return False
        expanded = dict(blocked_geometry)
        expanded['effective_size_x'] += 2.0 * extra_margin_m
        expanded['effective_size_y'] += 2.0 * extra_margin_m
        return _point_to_rect_distance(grid.cell_to_world(x, y), expanded) <= 1.0e-9

    if blocked(start) or blocked(goal):
        return []
    frontier: list[tuple[float, tuple[int, int]]] = [(0.0, start)]
    cost = {start: 0.0}
    parent: dict[tuple[int, int], tuple[int, int]] = {}
    neighbours = (
        (-1, -1, math.sqrt(2.0)), (0, -1, 1.0), (1, -1, math.sqrt(2.0)),
        (-1, 0, 1.0), (1, 0, 1.0),
        (-1, 1, math.sqrt(2.0)), (0, 1, 1.0), (1, 1, math.sqrt(2.0)),
    )
    while frontier:
        _, current = heapq.heappop(frontier)
        if current == goal:
            cells = [current]
            while current != start:
                current = parent[current]
                cells.append(current)
            cells.reverse()
            return [grid.cell_to_world(x, y) for x, y in cells]
        for dx, dy, step in neighbours:
            candidate = (current[0] + dx, current[1] + dy)
            if blocked(candidate):
                continue
            unknown_penalty = 2.0 if grid.unknown(*candidate) else 1.0
            new_cost = cost[current] + step * unknown_penalty
            if new_cost >= cost.get(candidate, math.inf):
                continue
            cost[candidate] = new_cost
            parent[candidate] = current
            heuristic = math.hypot(candidate[0] - goal[0], candidate[1] - goal[1])
            heapq.heappush(frontier, (new_cost + heuristic, candidate))
    return []


def _run_map_check(args: argparse.Namespace) -> int:
    grid = load_offline_map(args.map)
    nav2_config = load_nav2_inflation_config(args.nav2_config)
    uncertainty = args.covariance_sigma_scale * math.sqrt(
        max(args.variance_x, args.variance_y)
    )
    geometry = {
        'center_x': args.hazard_x,
        'center_y': args.hazard_y,
        'yaw': args.hazard_yaw,
        'nominal_size_x': args.hazard_size_x,
        'nominal_size_y': args.hazard_size_y,
        'variance_x': args.variance_x,
        'variance_y': args.variance_y,
        'covariance_sigma_scale': args.covariance_sigma_scale,
        'uncertainty_per_side_m': uncertainty,
        'effective_size_x': args.hazard_size_x + 2.0 * uncertainty,
        'effective_size_y': args.hazard_size_y + 2.0 * uncertainty,
    }
    start = (args.start_x, args.start_y)
    goal = (args.goal_x, args.goal_y)
    baseline = offline_astar(grid, start, goal)
    active = offline_astar(
        grid,
        start,
        goal,
        blocked_geometry=geometry,
        extra_margin_m=nav2_config['inflation_radius_m'],
    )
    baseline_metrics = path_hazard_metrics(baseline, geometry)
    active_metrics = path_hazard_metrics(active, geometry)
    failures = []
    if not baseline:
        failures.append('baseline_path_unavailable')
    if not baseline_metrics['crosses_effective_hazard']:
        failures.append('candidate_does_not_intersect_baseline')
    if not active:
        failures.append('detour_path_unavailable')
    if active_metrics['crosses_effective_hazard']:
        failures.append('detour_crosses_effective_hazard')
    summary = {
        'schema_version': 2,
        'status': 'pass' if not failures else 'fail',
        'validated_scope': 'offline_baylands_map_feasibility',
        'map': {
            'yaml': str(args.map.expanduser().resolve()),
            'resolution': grid.resolution,
            'origin': [grid.origin_x, grid.origin_y],
            'size_cells': [grid.width, grid.height],
        },
        'start': {'x': args.start_x, 'y': args.start_y},
        'goal': {'x': args.goal_x, 'y': args.goal_y},
        'hazard_geometry': geometry,
        'nav2_configuration': nav2_config,
        'nav2_inflation_radius_m': nav2_config['inflation_radius_m'],
        'baseline': {
            'path_pose_count': len(baseline),
            'path_length_m': path_length(baseline),
            **baseline_metrics,
        },
        'hazard_active': {
            'path_pose_count': len(active),
            'path_length_m': path_length(active),
            **active_metrics,
        },
        'path_hausdorff_distance_m': discrete_hausdorff_distance(baseline, active),
        'failures': failures,
        'limitations': [
            'Offline A* is a feasibility check and is not Navfn output.',
            'The active search conservatively blocks the effective footprint '
            'plus inflation radius.',
            'Unknown cells are permitted with a penalty, matching the configured '
            'allow_unknown intent.',
        ],
    }
    output_dir = args.output.expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    summary_path = output_dir / 'map_check.json'
    overlay_path = output_dir / 'map_check.svg'
    if summary_path.exists() or overlay_path.exists():
        raise FileExistsError(f'map-check output already exists in {output_dir}')
    summary['generated_at'] = datetime.now(timezone.utc).isoformat()
    summary_path.write_text(json.dumps(summary, indent=2, sort_keys=True) + '\n')
    overlay_path.write_text(_planner_overlay_svg([
        {'label': 'baseline', 'points': baseline},
        {'label': 'hazard_active', 'points': active},
    ], geometry), encoding='utf-8')
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary['status'] == 'pass' else 1


def _state_value(value: str) -> int | None:
    text = str(value).strip().upper()
    if not text:
        return None
    matches = {name: state for state, name in STATE_NAMES.items()}
    if text not in matches:
        raise argparse.ArgumentTypeError('state must be TENTATIVE, CONFIRMED, or CONFLICT')
    return matches[text]


def _expectations(args: argparse.Namespace) -> EvidenceExpectations:
    expected_sources = tuple(
        item.strip() for item in str(args.expected_sources).split(',') if item.strip()
    )
    return EvidenceExpectations(
        require_dji2=bool(args.require_dji2),
        expected_state=args.expected_state,
        expected_sources=expected_sources,
        expected_selected_source=str(args.expected_selected_source).strip(),
        minimum_hazard_count=int(args.minimum_hazard_count),
        require_confirmation_promotion=bool(args.require_confirmation_promotion),
        require_conflict=bool(args.require_conflict),
        require_expiry=bool(args.require_expiry),
        require_costmap=bool(getattr(args, 'require_costmap', False)),
        max_age_s=float(args.max_age_s),
    )


def _add_expectation_arguments(parser: argparse.ArgumentParser) -> None:
    parser.add_argument('--require-dji2', action='store_true')
    parser.add_argument('--expected-state', type=_state_value, default=None)
    parser.add_argument('--expected-sources', default='')
    parser.add_argument('--expected-selected-source', choices=('', 'dji1', 'dji2'), default='')
    parser.add_argument('--minimum-hazard-count', type=int, default=1)
    parser.add_argument('--require-confirmation-promotion', action='store_true')
    parser.add_argument('--require-conflict', action='store_true')
    parser.add_argument('--require-expiry', action='store_true')
    parser.add_argument('--max-age-s', type=float, default=1.0)
    parser.add_argument('--max-samples-per-topic', type=int, default=5000)
    parser.add_argument('--output', type=Path, required=True)


def _run_live(args: argparse.Namespace, ros_args: list[str]) -> int:
    if not math.isfinite(args.timeout_s) or args.timeout_s <= 0.0:
        raise ValueError('--timeout-s must be finite and greater than zero')
    collector = EvidenceCollector(max_samples_per_topic=args.max_samples_per_topic)
    expectations = _expectations(args)
    rclpy.init(args=ros_args)
    node = LiveEvidenceNode(collector)
    deadline = time.monotonic() + float(args.timeout_s)
    summary = collector.summarize(expectations)
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            summary = collector.summarize(expectations)
            if summary['status'] == 'pass':
                break
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    write_evidence(args.output, summary, collector.timeline_rows())
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary['status'] == 'pass' else 1


def _resolve_bag_dir(path: Path) -> Path:
    candidate = path.expanduser().resolve()
    if (candidate / 'bag').is_dir():
        candidate = candidate / 'bag'
    if not candidate.is_dir():
        raise FileNotFoundError(f'bag directory not found: {candidate}')
    return candidate


def _run_bag(args: argparse.Namespace) -> int:
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    collector = EvidenceCollector(max_samples_per_topic=args.max_samples_per_topic)
    bag_dir = _resolve_bag_dir(args.bag)
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=''),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr',
            output_serialization_format='cdr',
        ),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    message_types = {}
    for topic in HAZARD_TOPICS:
        if topic in type_map:
            message_types[topic] = get_message(type_map[topic])
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic in message_types:
            collector.add(
                topic,
                deserialize_message(data, message_types[topic]),
                int(timestamp_ns),
            )
        elif topic in COSTMAP_TOPICS:
            collector.add_costmap()
    summary = collector.summarize(_expectations(args))
    summary['bag_directory'] = str(bag_dir)
    write_evidence(args.output, summary, collector.timeline_rows())
    print(json.dumps(summary, indent=2, sort_keys=True))
    return 0 if summary['status'] == 'pass' else 1


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description='Validate and package bounded typed support-hazard evidence.'
    )
    subparsers = parser.add_subparsers(dest='mode', required=True)
    live = subparsers.add_parser('live', help='Observe bounded live typed topics.')
    live.add_argument('--timeout-s', type=float, default=12.0)
    _add_expectation_arguments(live)
    bag = subparsers.add_parser('bag', help='Analyze an existing support_hazard bag.')
    bag.add_argument('--bag', type=Path, required=True)
    bag.add_argument('--require-costmap', action='store_true')
    _add_expectation_arguments(bag)
    planner = subparsers.add_parser(
        'planner-live', help='Run bounded ComputePathToPose and costmap evidence.'
    )
    planner.add_argument('--scenario', required=True)
    planner.add_argument('--namespace', default='a201_0000')
    planner.add_argument('--map', type=Path, required=True)
    planner.add_argument('--nav2-config', type=Path, required=True)
    planner.add_argument('--start-x', type=float, required=True)
    planner.add_argument('--start-y', type=float, required=True)
    planner.add_argument('--start-yaw', type=float, default=0.0)
    planner.add_argument('--goal-x', type=float, required=True)
    planner.add_argument('--goal-y', type=float, required=True)
    planner.add_argument('--goal-yaw', type=float, default=0.0)
    planner.add_argument('--hazard-x', type=float, required=True)
    planner.add_argument('--hazard-y', type=float, required=True)
    planner.add_argument('--timeout-s', type=float, default=50.0)
    planner.add_argument('--minimum-path-change-m', type=float, default=0.5)
    planner.add_argument('--control-path-tolerance-m', type=float, default=0.25)
    planner.add_argument('--baseline-repeat-count', type=int, default=3)
    planner.add_argument('--baseline-path-tolerance-m', type=float, default=0.05)
    planner.add_argument('--baseline-settle-timeout-s', type=float, default=12.0)
    planner.add_argument('--baseline-stable-snapshots', type=int, default=2)
    planner.add_argument('--max-samples-per-topic', type=int, default=5000)
    planner.add_argument('--output', type=Path, required=True)
    map_check = subparsers.add_parser(
        'map-check', help='Check the fixed Baylands candidate without ROS runtime.'
    )
    map_check.add_argument('--map', type=Path, required=True)
    map_check.add_argument('--nav2-config', type=Path, required=True)
    map_check.add_argument('--output', type=Path, required=True)
    map_check.add_argument('--start-x', type=float, required=True)
    map_check.add_argument('--start-y', type=float, required=True)
    map_check.add_argument('--goal-x', type=float, required=True)
    map_check.add_argument('--goal-y', type=float, required=True)
    map_check.add_argument('--hazard-x', type=float, required=True)
    map_check.add_argument('--hazard-y', type=float, required=True)
    map_check.add_argument('--hazard-yaw', type=float, default=0.0)
    map_check.add_argument('--hazard-size-x', type=float, default=2.0)
    map_check.add_argument('--hazard-size-y', type=float, default=2.0)
    map_check.add_argument('--variance-x', type=float, default=0.25)
    map_check.add_argument('--variance-y', type=float, default=0.25)
    map_check.add_argument('--covariance-sigma-scale', type=float, default=2.0)
    return parser


def main(args=None) -> None:
    parser = build_parser()
    parsed, ros_args = parser.parse_known_args(sys.argv[1:] if args is None else args)
    try:
        if parsed.mode == 'live':
            status = _run_live(parsed, ros_args)
        elif parsed.mode == 'bag':
            status = _run_bag(parsed)
        elif parsed.mode == 'planner-live':
            status = _run_planner_live(parsed, ros_args)
        else:
            status = _run_map_check(parsed)
    except (FileExistsError, FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))
    raise SystemExit(status)


if __name__ == '__main__':
    main()
