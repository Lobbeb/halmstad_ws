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

from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.action._navigate_to_pose import NavigateToPose_FeedbackMessage
from nav2_msgs.msg import Costmap, CostmapUpdate
from nav2_msgs.srv import GetCostmap
from nav_msgs.msg import Path as NavPath
from rcl_interfaces.msg import ParameterType
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosgraph_msgs.msg import Clock
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from tf2_msgs.msg import TFMessage
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
GOAL_STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
    GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
    GoalStatus.STATUS_EXECUTING: 'EXECUTING',
    GoalStatus.STATUS_CANCELING: 'CANCELING',
    GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
    GoalStatus.STATUS_CANCELED: 'CANCELED',
    GoalStatus.STATUS_ABORTED: 'ABORTED',
}
FREE_SPACE = 0
LETHAL_OBSTACLE = 254
NO_INFORMATION = 255
DEFAULT_MAX_SUCCESS_TF_AGE_S = 1.0
DEFAULT_MAX_TF_INTERPOLATION_GAP_S = 2.0


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


@dataclass(frozen=True)
class PoseRecord:
    received_ns: int
    x: float
    y: float
    yaw: float
    source_stamp_ns: int = 0
    frame_id: str = ''
    child_frame_id: str = ''
    source: str = 'amcl_pose'


@dataclass(frozen=True)
class FeedbackRecord:
    received_ns: int
    goal_id: str
    pose: PoseRecord
    distance_remaining_m: float
    number_of_recoveries: int


@dataclass(frozen=True)
class TransformRecord:
    received_ns: int
    source_stamp_ns: int
    parent_frame: str
    child_frame: str
    x: float
    y: float
    yaw: float
    is_static: bool = False


@dataclass(frozen=True)
class RequestedGoalRecord:
    received_ns: int
    source_stamp_ns: int
    frame_id: str
    x: float
    y: float
    yaw: float


@dataclass(frozen=True)
class MissionStatusRecord:
    received_ns: int
    goal_id: str
    status: int


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


def _oriented_vertices(
    vertices: Iterable[tuple[float, float]], x: float, y: float, yaw: float
) -> list[tuple[float, float]]:
    cosine, sine = math.cos(yaw), math.sin(yaw)
    return [
        (x + cosine * px - sine * py, y + sine * px + cosine * py)
        for px, py in vertices
    ]


def _convex_intersection_area(
    polygon: list[tuple[float, float]],
    clipper: list[tuple[float, float]],
) -> float:
    """Clip a robot polygon to the counter-clockwise hazard rectangle."""
    result = polygon
    for start, end in zip(clipper, clipper[1:] + clipper[:1]):
        def side(point):
            return ((end[0] - start[0]) * (point[1] - start[1])
                    - (end[1] - start[1]) * (point[0] - start[0]))
        clipped = []
        for first, second in zip(result, result[1:] + result[:1]):
            first_side, second_side = side(first), side(second)
            if (first_side >= 0.0) != (second_side >= 0.0):
                ratio = first_side / (first_side - second_side)
                clipped.append((
                    first[0] + ratio * (second[0] - first[0]),
                    first[1] + ratio * (second[1] - first[1]),
                ))
            if second_side >= 0.0:
                clipped.append(second)
        result = clipped
        if not result:
            return 0.0
    return 0.5 * abs(sum(
        first[0] * second[1] - second[0] * first[1]
        for first, second in zip(result, result[1:] + result[:1])
    ))


def _point_segment_distance(point, start, end) -> float:
    dx, dy = end[0] - start[0], end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared <= 0.0:
        return math.hypot(point[0] - start[0], point[1] - start[1])
    ratio = min(1.0, max(0.0, (
        (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
    ) / length_squared))
    return math.hypot(
        point[0] - (start[0] + ratio * dx),
        point[1] - (start[1] + ratio * dy),
    )


def _polygon_clearance(
    first: list[tuple[float, float]], second: list[tuple[float, float]]
) -> float:
    if _convex_intersection_area(first, second) > 1.0e-9:
        return 0.0
    return min(
        _point_segment_distance(point, start, end)
        for polygon, other in ((first, second), (second, first))
        for point in polygon
        for start, end in zip(other, other[1:] + other[:1])
    )


def _parse_footprint(parameters: dict[str, Any]) -> tuple[
    list[tuple[float, float]], list[tuple[float, float]], float
]:
    value = parameters.get('footprint', '[]')
    vertices = ast.literal_eval(value) if isinstance(value, str) else value
    footprint = [(float(x), float(y)) for x, y in vertices]
    if len(footprint) < 3 or not all(
        math.isfinite(x) and math.isfinite(y) for x, y in footprint
    ):
        raise ValueError('costmap footprint must have at least three finite vertices')
    # Nav2 1.3.10 Costmap2DROS defaults footprint_padding to 0.01 and
    # padFootprint moves each nonzero coordinate outward by that amount.
    padding = float(parameters.get('footprint_padding', 0.01))
    if not math.isfinite(padding) or padding < 0.0:
        raise ValueError('costmap footprint_padding must be finite and non-negative')
    padded = [(
        x + (padding if x > 0.0 else -padding if x < 0.0 else 0.0),
        y + (padding if y > 0.0 else -padding if y < 0.0 else 0.0),
    ) for x, y in footprint]
    return footprint, padded, padding


def _global_robot_footprint(nav2_yaml: str) -> list[tuple[float, float]]:
    document = yaml.safe_load(Path(nav2_yaml).read_text(encoding='utf-8'))
    parameters = document['global_costmap']['global_costmap']['ros__parameters']
    _, padded, _ = _parse_footprint(parameters)
    return padded


def trajectory_footprint_overlap(
    poses, geometry: dict[str, float], footprint: list[tuple[float, float]]
) -> dict[str, Any]:
    """Diagnostic overlap of recorded map-frame robot poses with the virtual core."""
    half_x = 0.5 * geometry['effective_size_x']
    half_y = 0.5 * geometry['effective_size_y']
    hazard = _oriented_vertices(
        [(-half_x, -half_y), (half_x, -half_y),
         (half_x, half_y), (-half_x, half_y)],
        geometry['center_x'], geometry['center_y'], geometry['yaw'],
    )
    overlaps = []
    clearances = []
    for pose in poses:
        robot = _oriented_vertices(footprint, pose.x, pose.y, pose.yaw)
        area = _convex_intersection_area(robot, hazard)
        clearance = _polygon_clearance(robot, hazard)
        clearances.append(clearance)
        if area > 1.0e-9:
            overlaps.append({
                'received_ns': pose.received_ns,
                'source_stamp_ns': pose.source_stamp_ns,
                'pose': {'x': pose.x, 'y': pose.y, 'yaw': pose.yaw},
                'footprint_polygon': [list(point) for point in robot],
                'hazard_polygon': [list(point) for point in hazard],
                'overlap_area_m2': area,
                'minimum_clearance_m': clearance,
            })
    return {
        'source': 'recorded_amcl_map_frame_poses_and_global_costmap_yaml_footprint',
        'recorded_pose_count': len(poses),
        'overlapping_recorded_pose_count': len(overlaps),
        'first_overlap_received_ns': overlaps[0]['received_ns'] if overlaps else None,
        'last_overlap_received_ns': overlaps[-1]['received_ns'] if overlaps else None,
        'maximum_overlap_area_m2': max(
            (item['overlap_area_m2'] for item in overlaps), default=0.0
        ),
        'minimum_recorded_pose_clearance_m': min(clearances, default=None),
        'overlap_events': overlaps,
        'evaluated_footprint_vertices': footprint,
        'continuous_swept_footprint_not_proven': True,
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
    try:
        local_parameters = document['local_costmap']['local_costmap']['ros__parameters']
    except (KeyError, TypeError):
        local_parameters = None
    if not isinstance(local_parameters, dict):
        raise ValueError(f'local_costmap ROS parameters not found in {resolved}')
    global_footprint, global_padded_footprint, global_padding = _parse_footprint(parameters)
    local_footprint, local_padded_footprint, local_padding = _parse_footprint(
        local_parameters
    )
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
    local_aerial = local_parameters.get('aerial_support_layer')
    if not isinstance(local_aerial, dict):
        raise ValueError(f'local_costmap aerial_support_layer not found in {resolved}')
    shared_aerial_keys = (
        'topic', 'max_observation_age_s', 'default_ttl_s', 'min_confidence',
        'max_xy_variance_m2', 'confirmed_cost', 'tentative_cost', 'conflict_cost',
        'covariance_sigma_scale', 'min_footprint_size_m', 'subscription_depth',
    )
    if any(local_aerial.get(key) != aerial.get(key) for key in shared_aerial_keys):
        raise ValueError(
            f'local/global aerial_support_layer contracts differ in {resolved}'
        )
    controller_parameters = None
    for candidate in candidates:
        try:
            controller_parameters = candidate['controller_server']['ros__parameters']
        except (KeyError, TypeError):
            continue
        break
    if not isinstance(controller_parameters, dict):
        raise ValueError(f'controller_server ROS parameters not found in {resolved}')
    goal_checker = controller_parameters.get('general_goal_checker')
    required_goal_checker = ('plugin', 'xy_goal_tolerance', 'yaw_goal_tolerance', 'stateful')
    if not isinstance(goal_checker, dict) or any(
        key not in goal_checker for key in required_goal_checker
    ):
        raise ValueError(f'general_goal_checker configuration incomplete in {resolved}')
    rolling_value = parameters.get('rolling_window', False)
    if isinstance(rolling_value, str):
        rolling_value = rolling_value.strip().lower() in ('true', 'yes', '1', 'on')
    return {
        'source_yaml': str(resolved),
        'source_sha256': hashlib.sha256(resolved.read_bytes()).hexdigest(),
        'inflation_radius_m': radius,
        'cost_scaling_factor': scaling,
        'aerial_min_confidence': float(aerial['min_confidence']),
        'aerial_max_observation_age_s': float(aerial['max_observation_age_s']),
        'aerial_covariance_sigma_scale': float(aerial['covariance_sigma_scale']),
        'global_costmap_rolling_window': bool(rolling_value),
        'global_costmap_resolution_m': (
            float(parameters['resolution']) if 'resolution' in parameters else None
        ),
        'global_costmap_frame': str(parameters.get('global_frame', 'map')),
        'global_robot_base_frame': str(parameters.get('robot_base_frame', 'base_link')),
        'global_footprint_unpadded': [list(point) for point in global_footprint],
        'global_footprint_padding_m': global_padding,
        'global_footprint_padded': [list(point) for point in global_padded_footprint],
        'local_costmap_frame': str(local_parameters.get('global_frame', 'odom')),
        'local_robot_base_frame': str(local_parameters.get('robot_base_frame', 'base_link')),
        'local_footprint_unpadded': [list(point) for point in local_footprint],
        'local_footprint_padding_m': local_padding,
        'local_footprint_padded': [list(point) for point in local_padded_footprint],
        'local_inflation_radius_m': float(
            local_parameters['inflation_layer']['inflation_radius']
        ),
        'local_cost_scaling_factor': float(
            local_parameters['inflation_layer']['cost_scaling_factor']
        ),
        'local_aerial_layer_configured': (
            'aerial_support_layer' in local_parameters.get('plugins', [])
        ),
        'local_aerial_target_frame': str(local_aerial.get('target_frame', '')),
        'goal_checker_xy_tolerance_m': float(goal_checker['xy_goal_tolerance']),
        'goal_checker_yaw_tolerance_rad': float(goal_checker['yaw_goal_tolerance']),
        'goal_checker_plugin': str(goal_checker['plugin']),
        'goal_checker_stateful': bool(goal_checker['stateful']),
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


def directed_path_distance(
    source: Iterable[tuple[float, float]], target: Iterable[tuple[float, float]]
) -> float | None:
    source_points = list(source)
    target_points = list(target)
    if not source_points or not target_points:
        return None
    return max(
        min(math.hypot(px - qx, py - qy) for qx, qy in target_points)
        for px, py in source_points
    )


def point_to_path_distance(
    point: tuple[float, float], path: Iterable[tuple[float, float]]
) -> float | None:
    points = list(path)
    if not points:
        return None
    if len(points) == 1:
        return math.hypot(point[0] - points[0][0], point[1] - points[0][1])

    distances = []
    for start, end in zip(points, points[1:]):
        dx = end[0] - start[0]
        dy = end[1] - start[1]
        length_squared = dx * dx + dy * dy
        if length_squared <= 0.0:
            distances.append(math.hypot(point[0] - start[0], point[1] - start[1]))
            continue
        fraction = (
            (point[0] - start[0]) * dx + (point[1] - start[1]) * dy
        ) / length_squared
        fraction = min(1.0, max(0.0, fraction))
        closest = (start[0] + fraction * dx, start[1] + fraction * dy)
        distances.append(math.hypot(point[0] - closest[0], point[1] - closest[1]))
    return min(distances)


def stationary_periods(
    poses: Iterable[PoseRecord],
    *,
    speed_threshold_mps: float = 0.05,
    minimum_duration_s: float = 0.5,
) -> list[dict[str, Any]]:
    records = list(poses)
    periods: list[dict[str, Any]] = []
    period_start_ns = None
    period_end_ns = None
    for first, second in zip(records, records[1:]):
        elapsed_s = (second.received_ns - first.received_ns) * 1.0e-9
        if elapsed_s <= 0.0:
            continue
        speed = math.hypot(second.x - first.x, second.y - first.y) / elapsed_s
        if speed <= speed_threshold_mps:
            if period_start_ns is None:
                period_start_ns = first.received_ns
            period_end_ns = second.received_ns
        elif period_start_ns is not None and period_end_ns is not None:
            duration_s = (period_end_ns - period_start_ns) * 1.0e-9
            if duration_s >= minimum_duration_s:
                periods.append({
                    'start_ns': period_start_ns,
                    'end_ns': period_end_ns,
                    'duration_s': duration_s,
                })
            period_start_ns = None
            period_end_ns = None
    if period_start_ns is not None and period_end_ns is not None:
        duration_s = (period_end_ns - period_start_ns) * 1.0e-9
        if duration_s >= minimum_duration_s:
            periods.append({
                'start_ns': period_start_ns,
                'end_ns': period_end_ns,
                'duration_s': duration_s,
            })
    return periods


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


class RuntimeEvidenceCollector(PlannerEvidenceCollector):
    """Bound full-runtime mission, trajectory, plan, hazard, and costmap evidence."""

    def __init__(
        self,
        *,
        max_samples_per_topic: int = 5000,
        max_costmaps: int = 900,
        max_poses: int = 30000,
        max_plans: int = 2000,
        max_status_events: int = 2000,
        crop_geometry: dict[str, float] | None = None,
        crop_margin_m: float = 1.0,
    ) -> None:
        super().__init__(
            max_samples_per_topic=max_samples_per_topic,
            max_costmaps=max_costmaps,
        )
        self.poses: deque[PoseRecord] = deque(maxlen=max_poses)
        self.feedback: deque[FeedbackRecord] = deque(maxlen=max_poses)
        self.transforms: deque[TransformRecord] = deque(maxlen=max_poses * 4)
        self.requested_goals: deque[RequestedGoalRecord] = deque(maxlen=100)
        self.automatic_plans: deque[PlanRecord] = deque(maxlen=max_plans)
        self.status_events: deque[MissionStatusRecord] = deque(
            maxlen=max_status_events
        )
        self._last_status_by_goal: dict[str, int] = {}
        self.pose_dropped_count = 0
        self.feedback_dropped_count = 0
        self.transform_dropped_count = 0
        self.requested_goal_dropped_count = 0
        self.plan_dropped_count = 0
        self.status_dropped_count = 0
        self.amcl_warnings: deque[str] = deque(maxlen=10)
        self.amcl_warning_count = 0
        self.crop_geometry = crop_geometry
        self.crop_margin_m = max(0.0, float(crop_margin_m))
        self._crop_source_x = 0
        self._crop_source_y = 0
        self._source_size_x = 0
        self._source_size_y = 0

    def add(self, topic: str, message: AerialHazardArray, received_ns: int) -> None:
        if received_ns > 0:
            super().add(topic, message, received_ns)

    def add_full_costmap(
        self, message: Costmap, received_ns: int, *, source_kind: str = 'full'
    ) -> None:
        """Keep only the fixed hazard-region crop from large runtime costmaps."""
        if received_ns <= 0:
            return
        if self.crop_geometry is None:
            super().add_full_costmap(message, received_ns, source_kind=source_kind)
            return
        metadata = message.metadata
        source_size_x = int(metadata.size_x)
        source_size_y = int(metadata.size_y)
        expected = source_size_x * source_size_y
        resolution = float(metadata.resolution)
        if len(message.data) != expected or expected == 0 or resolution <= 0.0:
            return
        geometry = self.crop_geometry
        yaw = float(geometry.get('yaw', 0.0))
        half_x = 0.5 * float(geometry['effective_size_x'])
        half_y = 0.5 * float(geometry['effective_size_y'])
        extent_x = abs(math.cos(yaw)) * half_x + abs(math.sin(yaw)) * half_y
        extent_y = abs(math.sin(yaw)) * half_x + abs(math.cos(yaw)) * half_y
        extent_x += self.crop_margin_m
        extent_y += self.crop_margin_m
        source_origin_x = float(metadata.origin.position.x)
        source_origin_y = float(metadata.origin.position.y)
        min_x = int(math.floor(
            (float(geometry['center_x']) - extent_x - source_origin_x) / resolution
        ))
        max_x = int(math.ceil(
            (float(geometry['center_x']) + extent_x - source_origin_x) / resolution
        ))
        min_y = int(math.floor(
            (float(geometry['center_y']) - extent_y - source_origin_y) / resolution
        ))
        max_y = int(math.ceil(
            (float(geometry['center_y']) + extent_y - source_origin_y) / resolution
        ))
        min_x = max(0, min(source_size_x, min_x))
        max_x = max(0, min(source_size_x, max_x))
        min_y = max(0, min(source_size_y, min_y))
        max_y = max(0, min(source_size_y, max_y))
        crop_size_x = max_x - min_x
        crop_size_y = max_y - min_y
        if crop_size_x <= 0 or crop_size_y <= 0:
            return
        cropped = bytearray(crop_size_x * crop_size_y)
        source = bytes(message.data)
        for row in range(crop_size_y):
            source_start = (min_y + row) * source_size_x + min_x
            target_start = row * crop_size_x
            cropped[target_start:target_start + crop_size_x] = source[
                source_start:source_start + crop_size_x
            ]
        self._crop_source_x = min_x
        self._crop_source_y = min_y
        self._source_size_x = source_size_x
        self._source_size_y = source_size_y
        self.costmap_count += 1
        self.costmap_full_count += 1
        self.costmaps.append(GridSnapshot(
            received_ns=int(received_ns),
            source_kind=f'{source_kind}_hazard_crop',
            resolution=resolution,
            size_x=crop_size_x,
            size_y=crop_size_y,
            origin_x=source_origin_x + min_x * resolution,
            origin_y=source_origin_y + min_y * resolution,
            data=bytes(cropped),
        ))

    def add_costmap_update(self, message: CostmapUpdate, received_ns: int) -> None:
        if received_ns <= 0:
            return
        if self.crop_geometry is None:
            super().add_costmap_update(message, received_ns)
            return
        if not self.costmaps:
            return
        previous = self.costmaps[-1]
        update_x = int(message.x)
        update_y = int(message.y)
        update_width = int(message.size_x)
        update_height = int(message.size_y)
        if update_width * update_height != len(message.data):
            return
        if (
            update_x < 0 or update_y < 0
            or update_x + update_width > self._source_size_x
            or update_y + update_height > self._source_size_y
        ):
            return
        left = max(update_x, self._crop_source_x)
        right = min(update_x + update_width, self._crop_source_x + previous.size_x)
        bottom = max(update_y, self._crop_source_y)
        top = min(update_y + update_height, self._crop_source_y + previous.size_y)
        if left >= right or bottom >= top:
            return
        data = bytearray(previous.data)
        update_data = bytes(message.data)
        width = right - left
        for source_row in range(bottom, top):
            update_start = (
                (source_row - update_y) * update_width + left - update_x
            )
            target_start = (
                (source_row - self._crop_source_y) * previous.size_x
                + left - self._crop_source_x
            )
            data[target_start:target_start + width] = update_data[
                update_start:update_start + width
            ]
        self.costmap_count += 1
        self.costmap_update_count += 1
        self.costmaps.append(GridSnapshot(
            received_ns=int(received_ns),
            source_kind='update_hazard_crop',
            resolution=previous.resolution,
            size_x=previous.size_x,
            size_y=previous.size_y,
            origin_x=previous.origin_x,
            origin_y=previous.origin_y,
            data=bytes(data),
        ))

    def add_pose(self, message: PoseWithCovarianceStamped, received_ns: int) -> None:
        if received_ns <= 0:
            return
        if len(self.poses) == self.poses.maxlen:
            self.pose_dropped_count += 1
        orientation = message.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.poses.append(PoseRecord(
            received_ns=int(received_ns),
            x=float(message.pose.pose.position.x),
            y=float(message.pose.pose.position.y),
            yaw=float(yaw),
            source_stamp_ns=stamp_ns(message.header.stamp),
            frame_id=str(message.header.frame_id),
            child_frame_id='base_link',
            source='amcl_pose',
        ))

    def add_feedback(
        self, message: NavigateToPose_FeedbackMessage, received_ns: int
    ) -> None:
        if received_ns <= 0:
            return
        if len(self.feedback) == self.feedback.maxlen:
            self.feedback_dropped_count += 1
        current = message.feedback.current_pose
        orientation = current.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.feedback.append(FeedbackRecord(
            received_ns=int(received_ns),
            goal_id=bytes(message.goal_id.uuid).hex(),
            pose=PoseRecord(
                received_ns=int(received_ns),
                x=float(current.pose.position.x),
                y=float(current.pose.position.y),
                yaw=float(yaw),
                source_stamp_ns=stamp_ns(current.header.stamp),
                frame_id=str(current.header.frame_id),
                child_frame_id='base_link',
                source='navigate_to_pose_feedback',
            ),
            distance_remaining_m=float(message.feedback.distance_remaining),
            number_of_recoveries=int(message.feedback.number_of_recoveries),
        ))

    def add_requested_route(self, message: NavPath, received_ns: int) -> None:
        if received_ns <= 0 or not message.poses:
            return
        if len(self.requested_goals) == self.requested_goals.maxlen:
            self.requested_goal_dropped_count += 1
        pose = message.poses[-1]
        orientation = pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self.requested_goals.append(RequestedGoalRecord(
            received_ns=int(received_ns),
            source_stamp_ns=stamp_ns(pose.header.stamp),
            frame_id=str(pose.header.frame_id or message.header.frame_id),
            x=float(pose.pose.position.x),
            y=float(pose.pose.position.y),
            yaw=float(yaw),
        ))

    def add_tf(self, message: TFMessage, received_ns: int, *, is_static: bool) -> None:
        if received_ns <= 0:
            return
        for item in message.transforms:
            parent = str(item.header.frame_id).strip('/')
            child = str(item.child_frame_id).strip('/')
            if (parent, child) not in {
                ('map', 'odom'), ('odom', 'base_link'), ('map', 'base_link')
            }:
                continue
            if len(self.transforms) == self.transforms.maxlen:
                self.transform_dropped_count += 1
            rotation = item.transform.rotation
            yaw = math.atan2(
                2.0 * (rotation.w * rotation.z + rotation.x * rotation.y),
                1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z),
            )
            self.transforms.append(TransformRecord(
                received_ns=int(received_ns),
                source_stamp_ns=stamp_ns(item.header.stamp),
                parent_frame=parent,
                child_frame=child,
                x=float(item.transform.translation.x),
                y=float(item.transform.translation.y),
                yaw=float(yaw),
                is_static=bool(is_static),
            ))

    def add_automatic_plan(self, message: NavPath, received_ns: int) -> None:
        if received_ns <= 0:
            return
        self.add_plan_topic_message()
        if len(self.automatic_plans) == self.automatic_plans.maxlen:
            self.plan_dropped_count += 1
        points = tuple(
            (float(item.pose.position.x), float(item.pose.position.y))
            for item in message.poses
        )
        self.automatic_plans.append(PlanRecord(
            label=f'automatic_{self.plan_topic_count:04d}',
            requested_ns=int(received_ns),
            received_ns=int(received_ns),
            planning_time_s=0.0,
            error_code=0 if points else 1,
            error_message='' if points else 'empty plan topic message',
            points=points,
        ))

    def add_status(self, message: GoalStatusArray, received_ns: int) -> None:
        if received_ns <= 0:
            return
        for item in message.status_list:
            goal_id = bytes(item.goal_info.goal_id.uuid).hex()
            status = int(item.status)
            if self._last_status_by_goal.get(goal_id) == status:
                continue
            self._last_status_by_goal[goal_id] = status
            if len(self.status_events) == self.status_events.maxlen:
                self.status_dropped_count += 1
            self.status_events.append(MissionStatusRecord(
                received_ns=int(received_ns),
                goal_id=goal_id,
                status=status,
            ))

    def add_rosout(self, message: Log) -> None:
        if message.level >= Log.WARN and 'amcl' in message.name.lower():
            self.record_amcl_warning(str(message.msg))

    def record_amcl_warning(self, message: str) -> None:
        self.amcl_warning_count += 1
        if message not in self.amcl_warnings:
            self.amcl_warnings.append(message)

    def selected_mission(self) -> dict[str, Any]:
        events = sorted(self.status_events, key=lambda item: item.received_ns)
        live = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING)
        goal_id = next((item.goal_id for item in events if item.status in live), '')
        selected = [item for item in events if item.goal_id == goal_id]
        start_ns = next(
            (item.received_ns for item in selected if item.status in live), None
        )
        terminal_states = (
            GoalStatus.STATUS_SUCCEEDED,
            GoalStatus.STATUS_CANCELED,
            GoalStatus.STATUS_ABORTED,
        )
        terminal = next(
            (item for item in reversed(selected) if item.status in terminal_states), None
        )
        terminal_statuses = sorted({item.status for item in selected if item.status in terminal_states})
        completion_ns = terminal.received_ns if terminal else None
        concurrent_states = (*live, GoalStatus.STATUS_CANCELING)
        other_goal_ids = sorted({
            item.goal_id
            for item in events
            if goal_id
            and item.goal_id != goal_id
            and item.status in concurrent_states
            and start_ns is not None
            and item.received_ns >= start_ns
            and (completion_ns is None or item.received_ns <= completion_ns)
        })
        return {
            'goal_id': goal_id,
            'start_ns': start_ns,
            'completion_ns': completion_ns,
            'terminal_status': (
                GOAL_STATUS_NAMES.get(terminal.status, str(terminal.status))
                if terminal else None
            ),
            'succeeded': bool(terminal and terminal.status == GoalStatus.STATUS_SUCCEEDED),
            'contradictory_terminal_statuses': len(terminal_statuses) > 1,
            'other_goal_ids_during_mission': other_goal_ids,
            'single_active_goal_observed': bool(goal_id and not other_goal_ids),
            'transitions': [
                {
                    'received_ns': item.received_ns,
                    'status': GOAL_STATUS_NAMES.get(item.status, str(item.status)),
                }
                for item in selected
            ],
        }


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
                    self._evidence_now_ns(),
                ),
                qos,
            )
            for topic in HAZARD_TOPICS
        ]

    def _evidence_now_ns(self) -> int:
        return int(self.get_clock().now().nanoseconds)


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


class RuntimeEvidenceNode(LiveEvidenceNode):
    """Passively observe one full NavigateToPose experiment."""

    def __init__(self, collector: RuntimeEvidenceCollector, namespace: str) -> None:
        self._runtime_sim_time_ns = 0
        super().__init__(collector)
        self.collector = collector
        prefix = '/' + namespace.strip('/')
        clock_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        full_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        stream_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        tf_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._runtime_subscriptions = [
            self.create_subscription(
                Clock,
                '/clock',
                self._on_runtime_clock,
                clock_qos,
            ),
            self.create_subscription(
                Log, '/rosout', collector.add_rosout, stream_qos
            ),
            self.create_subscription(
                Costmap,
                f'{prefix}/global_costmap/costmap_raw',
                lambda message: collector.add_full_costmap(
                    message, self._evidence_now_ns()
                ),
                full_qos,
            ),
            self.create_subscription(
                CostmapUpdate,
                f'{prefix}/global_costmap/costmap_raw_updates',
                lambda message: collector.add_costmap_update(
                    message, self._evidence_now_ns()
                ),
                stream_qos,
            ),
            self.create_subscription(
                NavPath,
                f'{prefix}/plan',
                lambda message: collector.add_automatic_plan(
                    message, self._evidence_now_ns()
                ),
                stream_qos,
            ),
            self.create_subscription(
                NavPath,
                f'{prefix}/planned_path',
                lambda message: collector.add_requested_route(
                    message, self._evidence_now_ns()
                ),
                stream_qos,
            ),
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'{prefix}/amcl_pose',
                lambda message: collector.add_pose(
                    message, self._evidence_now_ns()
                ),
                stream_qos,
            ),
            self.create_subscription(
                GoalStatusArray,
                f'{prefix}/navigate_to_pose/_action/status',
                lambda message: collector.add_status(
                    message, self._evidence_now_ns()
                ),
                stream_qos,
            ),
            self.create_subscription(
                NavigateToPose_FeedbackMessage,
                f'{prefix}/navigate_to_pose/_action/feedback',
                lambda message: collector.add_feedback(
                    message, self._evidence_now_ns()
                ),
                stream_qos,
            ),
            self.create_subscription(
                TFMessage,
                f'{prefix}/tf',
                lambda message: collector.add_tf(
                    message, self._evidence_now_ns(), is_static=False
                ),
                tf_qos,
            ),
            self.create_subscription(
                TFMessage,
                f'{prefix}/tf_static',
                lambda message: collector.add_tf(
                    message, self._evidence_now_ns(), is_static=True
                ),
                full_qos,
            ),
            self.create_subscription(
                TFMessage,
                '/tf',
                lambda message: collector.add_tf(
                    message, self._evidence_now_ns(), is_static=False
                ),
                tf_qos,
            ),
            self.create_subscription(
                TFMessage,
                '/tf_static',
                lambda message: collector.add_tf(
                    message, self._evidence_now_ns(), is_static=True
                ),
                full_qos,
            ),
        ]
        self.costmap_client = self.create_client(
            GetCostmap, f'{prefix}/global_costmap/get_costmap'
        )
        self.layer_parameters = AsyncParameterClient(
            self, f'{prefix}/global_costmap/global_costmap'
        )
        self.controller_parameters = AsyncParameterClient(
            self, f'{prefix}/controller_server'
        )

    def _on_runtime_clock(self, message: Clock) -> None:
        self._runtime_sim_time_ns = stamp_ns(message.clock)

    def _evidence_now_ns(self) -> int:
        return self._runtime_sim_time_ns


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


def _get_aerial_layer_enabled(
    node: RuntimeEvidenceNode, timeout_s: float
) -> bool | None:
    if not node.layer_parameters.wait_for_services(timeout_sec=timeout_s):
        return None
    future = node.layer_parameters.get_parameters(['aerial_support_layer.enabled'])
    if not _spin_until(node, future.done, time.monotonic() + timeout_s):
        return None
    response = future.result()
    values = response.values if response is not None else ()
    if not values:
        return None
    return bool(values[0].bool_value)


def _get_runtime_goal_checker(
    node: RuntimeEvidenceNode, timeout_s: float
) -> dict[str, Any] | None:
    if not node.controller_parameters.wait_for_services(timeout_sec=timeout_s):
        return None
    names = [
        'goal_checker_plugins',
        'general_goal_checker.plugin',
        'general_goal_checker.xy_goal_tolerance',
        'general_goal_checker.yaw_goal_tolerance',
        'general_goal_checker.stateful',
    ]
    future = node.controller_parameters.get_parameters(names)
    if not _spin_until(node, future.done, time.monotonic() + timeout_s):
        return None
    response = future.result()
    values = response.values if response is not None else ()
    if len(values) != len(names):
        return None
    if (
        values[0].type != ParameterType.PARAMETER_STRING_ARRAY
        or values[1].type != ParameterType.PARAMETER_STRING
        or values[2].type != ParameterType.PARAMETER_DOUBLE
        or values[3].type != ParameterType.PARAMETER_DOUBLE
        or values[4].type != ParameterType.PARAMETER_BOOL
    ):
        return None
    return {
        'goal_checker_plugins': list(values[0].string_array_value),
        'plugin': str(values[1].string_value),
        'xy_goal_tolerance_m': float(values[2].double_value),
        'yaw_goal_tolerance_rad': float(values[3].double_value),
        'stateful': bool(values[4].bool_value),
        'source': 'controller_server runtime parameters',
    }


def _request_costmap_snapshot(node: PlannerEvidenceNode, timeout_s: float) -> bool:
    if not node.costmap_client.wait_for_service(timeout_sec=timeout_s):
        return False
    future = node.costmap_client.call_async(GetCostmap.Request())
    if not _spin_until(node, future.done, time.monotonic() + timeout_s):
        return False
    response = future.result()
    if response is None:
        return False
    received_ns = (
        node._evidence_now_ns()
        if hasattr(node, '_evidence_now_ns')
        else int(node.get_clock().now().nanoseconds)
    )
    if isinstance(node.collector, RuntimeEvidenceCollector) and received_ns <= 0:
        return False
    node.collector.add_full_costmap(
        response.map,
        received_ns,
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


def _first_empty_after_nonempty(
    collector: EvidenceCollector, topic: str
) -> CapturedSample | None:
    saw_nonempty = False
    for sample in collector._ordered(topic):
        if sample.message.hazards:
            saw_nonempty = True
        elif saw_nonempty:
            return sample
    return None


def _clearing_mechanism(
    collector: PlannerEvidenceCollector,
    first_clear_ns: int | None,
    max_observation_age_s: float,
) -> dict[str, Any]:
    source_empty = _first_empty_after_nonempty(collector, DJI1_TOPIC)
    fused_empty = _first_empty_after_nonempty(collector, DJI0_TOPIC)
    forwarded_empty = _first_empty_after_nonempty(collector, UGV_TOPIC)
    explicit_empty_propagation = bool(
        source_empty
        and fused_empty
        and forwarded_empty
        and source_empty.received_ns <= fused_empty.received_ns
        and fused_empty.received_ns <= forwarded_empty.received_ns
        and first_clear_ns is not None
        and forwarded_empty.received_ns <= first_clear_ns
    )
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
    if explicit_empty_propagation:
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
        'explicit_empty_snapshot_seen': explicit_empty_propagation,
        'source_explicit_empty_ns': (
            source_empty.received_ns if source_empty else None
        ),
        'fusion_explicit_empty_ns': (
            fused_empty.received_ns if fused_empty else None
        ),
        'forwarded_explicit_empty_ns': (
            forwarded_empty.received_ns if forwarded_empty else None
        ),
        'explicit_empty_propagation_complete': explicit_empty_propagation,
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


def _path_for_snapshot(
    snapshots: Iterable[GridSnapshot], received_ns: int
) -> GridSnapshot | None:
    ordered = sorted(snapshots, key=lambda item: item.received_ns)
    earlier = [item for item in ordered if item.received_ns <= received_ns]
    if earlier:
        return earlier[-1]
    return ordered[0] if ordered else None


def _runtime_baseline_bundle(path: Path | None) -> dict[str, Any] | None:
    if path is None:
        return None
    root = path.expanduser().resolve()
    summary_path = root / 'summary.json'
    plans_path = root / 'plans.json'
    trajectory_path = root / 'trajectory.csv'
    if not summary_path.is_file() or not plans_path.is_file() or not trajectory_path.is_file():
        raise FileNotFoundError(
            f'baseline evidence must contain summary.json, plans.json, and trajectory.csv: {root}'
        )
    summary = json.loads(summary_path.read_text(encoding='utf-8'))
    plans = json.loads(plans_path.read_text(encoding='utf-8'))
    with trajectory_path.open(newline='', encoding='utf-8') as stream:
        trajectory = [
            (float(row['x']), float(row['y']))
            for row in csv.DictReader(stream)
        ]
    return {
        'root': str(root),
        'summary': summary,
        'plans': plans,
        'trajectory': trajectory,
    }


def _runtime_costmap_analysis(
    collector: RuntimeEvidenceCollector,
    geometry: dict[str, float],
    inflation_radius_m: float,
) -> dict[str, Any]:
    snapshots = sorted(collector.costmaps, key=lambda item: item.received_ns)
    first_hazard_ns = min(
        (
            sample.received_ns
            for sample in collector.samples[UGV_TOPIC]
            if sample.message.hazards
        ),
        default=None,
    )
    baseline_candidates = [
        item for item in snapshots
        if first_hazard_ns is None or item.received_ns < first_hazard_ns
    ]
    baseline, settling = settled_baseline_selection(
        baseline_candidates,
        geometry,
        inflation_radius_m,
        required_consecutive=2,
    )
    first_mark = None
    first_mark_delta = None
    first_clear = None
    if baseline is not None and first_hazard_ns is not None:
        for snapshot in snapshots:
            if snapshot.received_ns < first_hazard_ns:
                continue
            delta = relevant_costmap_delta(
                baseline, snapshot, geometry, inflation_radius_m
            )
            if (
                delta['comparable']
                and delta.get('hazard_footprint_lethal_cells', 0) > 0
            ):
                first_mark = snapshot
                first_mark_delta = delta
                break
        if first_mark is not None:
            for snapshot in snapshots:
                if snapshot.received_ns <= first_mark.received_ns:
                    continue
                delta = relevant_costmap_delta(
                    baseline, snapshot, geometry, inflation_radius_m
                )
                if delta['comparable'] and delta['affected_cells'] == 0:
                    first_clear = snapshot
                    break
    return {
        'baseline': baseline,
        'baseline_settling': settling,
        'first_hazard_ns': first_hazard_ns,
        'first_mark': first_mark,
        'first_mark_delta': first_mark_delta,
        'first_clear': first_clear,
        'snapshots': snapshots,
    }


def _runtime_plan_dicts(
    collector: RuntimeEvidenceCollector,
    *,
    mission: dict[str, Any],
    costmaps: dict[str, Any],
    geometry: dict[str, float],
    inflation_geometry: dict[str, float],
    minimum_path_change_m: float,
) -> tuple[list[dict[str, Any]], dict[str, Any]]:
    start_ns = mission.get('start_ns')
    completion_ns = mission.get('completion_ns')
    records = [
        item for item in collector.automatic_plans
        if start_ns is not None
        and item.received_ns >= start_ns
        and (completion_ns is None or item.received_ns <= completion_ns)
        and item.points
    ]
    mark = costmaps['first_mark']
    clear = costmaps['first_clear']
    first_hazard_ns = costmaps['first_hazard_ns']
    if first_hazard_ns is None:
        pre_hazard = records[0] if records else None
    else:
        before = [item for item in records if item.received_ns < first_hazard_ns]
        pre_hazard = before[-1] if before else None
    changed = None
    changed_distance = None
    if mark is not None and pre_hazard is not None:
        for item in records:
            if item.received_ns <= mark.received_ns:
                continue
            distance = discrete_hausdorff_distance(pre_hazard.points, item.points)
            if distance is not None and distance >= minimum_path_change_m:
                changed = item
                changed_distance = distance
                break
    post_clear = None
    if clear is not None:
        post_clear = next(
            (item for item in records if item.received_ns > clear.received_ns), None
        )

    labelled: list[tuple[str, PlanRecord]] = []
    if pre_hazard is not None:
        labelled.append(('baseline', pre_hazard))
    if changed is not None:
        labelled.append(('hazard_active', changed))
    if post_clear is not None:
        labelled.append(('post_clear', post_clear))
    selected_ids = {id(record) for _, record in labelled}
    for record in records:
        if id(record) not in selected_ids:
            labelled.append((record.label, record))

    plans = []
    for label, record in labelled:
        relabelled = PlanRecord(
            label, record.requested_ns, record.received_ns,
            record.planning_time_s, record.error_code, record.error_message,
            record.points,
        )
        plans.append(plan_record_dict(
            relabelled,
            geometry=geometry,
            inflation_geometry=inflation_geometry,
            costmap=_path_for_snapshot(costmaps['snapshots'], record.received_ns),
            baseline_costmap=costmaps['baseline'],
        ))
    selected = {
        'pre_hazard': next((item for item in plans if item['label'] == 'baseline'), None),
        'hazard_active': next(
            (item for item in plans if item['label'] == 'hazard_active'), None
        ),
        'post_clear': next(
            (item for item in plans if item['label'] == 'post_clear'), None
        ),
        'material_change_m': changed_distance,
        'automatic_plan_count_during_goal': len(records),
    }
    return plans, selected


def _compose_planar_pose(
    first: tuple[float, float, float], second: tuple[float, float, float]
) -> tuple[float, float, float]:
    """Compose parent->middle and middle->child planar transforms."""
    x0, y0, yaw0 = first
    x1, y1, yaw1 = second
    return (
        x0 + math.cos(yaw0) * x1 - math.sin(yaw0) * y1,
        y0 + math.sin(yaw0) * x1 + math.cos(yaw0) * y1,
        math.atan2(math.sin(yaw0 + yaw1), math.cos(yaw0 + yaw1)),
    )


def _inverse_planar_pose(
    transform: tuple[float, float, float]
) -> tuple[float, float, float]:
    x, y, yaw = transform
    inverse_yaw = -yaw
    return (
        -(math.cos(inverse_yaw) * x - math.sin(inverse_yaw) * y),
        -(math.sin(inverse_yaw) * x + math.cos(inverse_yaw) * y),
        inverse_yaw,
    )


def resolve_planar_tf_pose(
    transforms: Iterable[TransformRecord],
    *,
    parent_frame: str,
    child_frame: str,
    at_received_ns: int,
    max_age_s: float = DEFAULT_MAX_SUCCESS_TF_AGE_S,
    max_interpolation_gap_s: float = DEFAULT_MAX_TF_INTERPOLATION_GAP_S,
) -> dict[str, Any]:
    """Resolve one planar TF chain at a bounded common source timestamp."""
    start = parent_frame.strip('/')
    target = child_frame.strip('/')
    base_result = {
        'status': 'unavailable',
        'frame_id': start,
        'child_frame_id': target,
        'action_success_received_ns': int(at_received_ns),
        'max_tf_age_s': float(max_age_s),
        'max_interpolation_gap_s': float(max_interpolation_gap_s),
    }
    if at_received_ns <= 0:
        return {**base_result, 'failure_reason': 'invalid_action_success_timestamp'}
    if not math.isfinite(max_age_s) or max_age_s < 0.0:
        raise ValueError('max_age_s must be finite and non-negative')
    if not math.isfinite(max_interpolation_gap_s) or max_interpolation_gap_s < 0.0:
        raise ValueError('max_interpolation_gap_s must be finite and non-negative')

    dynamic_by_stamp: dict[
        tuple[str, str], dict[int, TransformRecord]
    ] = {}
    static_by_edge: dict[tuple[str, str], TransformRecord] = {}
    invalid_dynamic_stamp_count = 0
    for item in transforms:
        if item.received_ns <= 0 or item.received_ns > at_received_ns:
            continue
        key = (item.parent_frame.strip('/'), item.child_frame.strip('/'))
        if not all(key):
            continue
        if item.is_static:
            previous = static_by_edge.get(key)
            if previous is None or item.received_ns >= previous.received_ns:
                static_by_edge[key] = item
            continue
        if item.source_stamp_ns <= 0:
            invalid_dynamic_stamp_count += 1
            continue
        by_stamp = dynamic_by_stamp.setdefault(key, {})
        previous = by_stamp.get(item.source_stamp_ns)
        if previous is None or item.received_ns >= previous.received_ns:
            by_stamp[item.source_stamp_ns] = item

    dynamic = {
        key: sorted(records.values(), key=lambda item: item.source_stamp_ns)
        for key, records in dynamic_by_stamp.items()
    }
    edges = sorted(set(dynamic) | set(static_by_edge))
    graph: dict[str, list[tuple[str, tuple[str, str], bool]]] = {}
    for key in edges:
        parent, child = key
        graph.setdefault(parent, []).append((child, key, False))
        graph.setdefault(child, []).append((parent, key, True))

    frame_count = len({frame for edge in edges for frame in edge})
    queue = deque([(start, (), (start,))])
    paths: list[tuple[tuple[str, tuple[str, str], bool], ...]] = []
    while queue:
        frame, path, visited = queue.popleft()
        if frame == target:
            paths.append(path)
            continue
        if len(visited) > frame_count:
            continue
        for adjacent, key, inverse in graph.get(frame, ()):
            if adjacent in visited:
                continue
            queue.append((adjacent, path + ((adjacent, key, inverse),), visited + (adjacent,)))

    if not paths:
        return {
            **base_result,
            'failure_reason': 'no_transform_chain_available_at_success',
            'invalid_dynamic_stamp_count': invalid_dynamic_stamp_count,
        }

    max_age_ns = int(round(max_age_s * 1.0e9))
    max_gap_ns = int(round(max_interpolation_gap_s * 1.0e9))
    candidates: list[dict[str, Any]] = []
    rejected_paths: list[dict[str, Any]] = []
    for path in paths:
        frame_path = [start] + [item[0] for item in path]
        dynamic_histories = [dynamic[key] for _, key, _ in path if key in dynamic]
        if dynamic_histories:
            earliest_common_ns = max(history[0].source_stamp_ns for history in dynamic_histories)
            latest_common_ns = min(
                at_received_ns,
                *(history[-1].source_stamp_ns for history in dynamic_histories),
            )
        else:
            earliest_common_ns = at_received_ns
            latest_common_ns = at_received_ns
        if latest_common_ns < earliest_common_ns:
            rejected_paths.append({
                'frame_path': frame_path,
                'reason': 'no_common_tf_time',
                'earliest_common_ns': earliest_common_ns,
                'latest_common_ns': latest_common_ns,
            })
            continue
        common_time_ns = latest_common_ns
        age_ns = at_received_ns - common_time_ns
        if age_ns > max_age_ns:
            rejected_paths.append({
                'frame_path': frame_path,
                'reason': 'common_tf_time_too_old',
                'common_time_ns': common_time_ns,
                'age_s': age_ns * 1.0e-9,
            })
            continue

        accumulated = (0.0, 0.0, 0.0)
        chain: list[dict[str, Any]] = []
        used_records: list[TransformRecord] = []
        rejection: dict[str, Any] | None = None
        current_frame = start
        for adjacent, key, inverse in path:
            if key in static_by_edge and key not in dynamic:
                record = static_by_edge[key]
                value = (record.x, record.y, record.yaw)
                used_records.append(record)
                detail = {
                    'parent_frame': key[0],
                    'child_frame': key[1],
                    'traversal_parent_frame': current_frame,
                    'traversal_child_frame': adjacent,
                    'method': 'static',
                    'source_stamp_ns': record.source_stamp_ns,
                    'received_ns': record.received_ns,
                    'static': True,
                }
            else:
                history = dynamic[key]
                exact = next(
                    (item for item in history if item.source_stamp_ns == common_time_ns),
                    None,
                )
                if exact is not None:
                    value = (exact.x, exact.y, exact.yaw)
                    used_records.append(exact)
                    detail = {
                        'parent_frame': key[0],
                        'child_frame': key[1],
                        'traversal_parent_frame': current_frame,
                        'traversal_child_frame': adjacent,
                        'method': 'exact',
                        'source_stamp_ns': exact.source_stamp_ns,
                        'received_ns': exact.received_ns,
                        'static': False,
                    }
                else:
                    lower = next(
                        (item for item in reversed(history)
                         if item.source_stamp_ns < common_time_ns),
                        None,
                    )
                    upper = next(
                        (item for item in history
                         if item.source_stamp_ns > common_time_ns),
                        None,
                    )
                    if lower is None or upper is None:
                        rejection = {
                            'frame_path': frame_path,
                            'reason': 'common_tf_time_not_bracketed',
                            'edge': list(key),
                            'common_time_ns': common_time_ns,
                        }
                        break
                    gap_ns = upper.source_stamp_ns - lower.source_stamp_ns
                    if gap_ns > max_gap_ns:
                        rejection = {
                            'frame_path': frame_path,
                            'reason': 'interpolation_gap_too_large',
                            'edge': list(key),
                            'gap_s': gap_ns * 1.0e-9,
                        }
                        break
                    fraction = (
                        (common_time_ns - lower.source_stamp_ns) / gap_ns
                    )
                    yaw_delta = math.atan2(
                        math.sin(upper.yaw - lower.yaw),
                        math.cos(upper.yaw - lower.yaw),
                    )
                    value = (
                        lower.x + fraction * (upper.x - lower.x),
                        lower.y + fraction * (upper.y - lower.y),
                        math.atan2(
                            math.sin(lower.yaw + fraction * yaw_delta),
                            math.cos(lower.yaw + fraction * yaw_delta),
                        ),
                    )
                    used_records.extend((lower, upper))
                    detail = {
                        'parent_frame': key[0],
                        'child_frame': key[1],
                        'traversal_parent_frame': current_frame,
                        'traversal_child_frame': adjacent,
                        'method': 'interpolated',
                        'source_stamp_ns': common_time_ns,
                        'lower_source_stamp_ns': lower.source_stamp_ns,
                        'lower_received_ns': lower.received_ns,
                        'upper_source_stamp_ns': upper.source_stamp_ns,
                        'upper_received_ns': upper.received_ns,
                        'interpolation_fraction': fraction,
                        'static': False,
                    }
            if inverse:
                value = _inverse_planar_pose(value)
            accumulated = _compose_planar_pose(accumulated, value)
            chain.append(detail)
            current_frame = adjacent
        if rejection is not None:
            rejected_paths.append(rejection)
            continue

        candidates.append({
            **base_result,
            'status': 'resolved',
            'x': accumulated[0],
            'y': accumulated[1],
            'yaw': accumulated[2],
            'common_time_ns': common_time_ns,
            'common_time_offset_from_success_s': (
                common_time_ns - at_received_ns
            ) * 1.0e-9,
            'tf_age_s': age_ns * 1.0e-9,
            'interpolation_used': any(
                item['method'] == 'interpolated' for item in chain
            ),
            'newest_used_tf_received_ns': max(
                (item.received_ns for item in used_records), default=None
            ),
            'oldest_used_tf_received_ns': min(
                (item.received_ns for item in used_records), default=None
            ),
            'transform_chain': chain,
            'temporal_policy': (
                'latest common source timestamp not after action success; '
                'exact samples or bounded interpolation only'
            ),
            'invalid_dynamic_stamp_count': invalid_dynamic_stamp_count,
        })

    if not candidates:
        reasons = {item['reason'] for item in rejected_paths}
        failure_reason = (
            next(iter(reasons)) if len(reasons) == 1
            else 'no_defensible_common_tf_time'
        )
        return {
            **base_result,
            'failure_reason': failure_reason,
            'invalid_dynamic_stamp_count': invalid_dynamic_stamp_count,
            'rejected_paths': rejected_paths,
        }
    return max(
        candidates,
        key=lambda item: (
            item['common_time_ns'],
            -len(item['transform_chain']),
            not item['interpolation_used'],
        ),
    )


def _pose_evaluation(
    pose: dict[str, Any] | PoseRecord | None,
    goal: tuple[float, float, float],
    *,
    action_success_received_ns: int | None = None,
) -> dict[str, Any] | None:
    if pose is None:
        return None
    if isinstance(pose, PoseRecord):
        result = {
            'x': pose.x,
            'y': pose.y,
            'yaw': pose.yaw,
            'frame_id': pose.frame_id,
            'child_frame_id': pose.child_frame_id,
            'received_ns': pose.received_ns,
            'source_stamp_ns': pose.source_stamp_ns,
            'source': pose.source,
        }
    else:
        result = dict(pose)
    result['xy_error_m'] = math.hypot(result['x'] - goal[0], result['y'] - goal[1])
    result['yaw_error_rad'] = abs(math.atan2(
        math.sin(result['yaw'] - goal[2]), math.cos(result['yaw'] - goal[2])
    ))
    if action_success_received_ns is not None:
        received_ns = result.get('received_ns')
        source_stamp_ns = result.get('source_stamp_ns')
        result['received_time_offset_from_action_success_s'] = (
            (received_ns - action_success_received_ns) * 1.0e-9
            if received_ns is not None else None
        )
        result['source_stamp_offset_from_action_success_s'] = (
            (source_stamp_ns - action_success_received_ns) * 1.0e-9
            if source_stamp_ns is not None and source_stamp_ns > 0 else None
        )
    return result


def _xy_tolerance_history_diagnostics(
    poses: Iterable[dict[str, Any] | None],
    tolerance_m: float,
) -> dict[str, Any]:
    observed = [pose for pose in poses if pose is not None]
    first_index = next(
        (
            index for index, pose in enumerate(observed)
            if pose['xy_error_m'] <= tolerance_m
        ),
        None,
    )
    if first_index is None:
        return {
            'observed_pose_count': len(observed),
            'first_pose_within_xy_tolerance': None,
            'later_pose_count': 0,
            'maximum_later_xy_error_m': None,
            'later_pose_outside_xy_tolerance': False,
        }
    later = observed[first_index + 1:]
    return {
        'observed_pose_count': len(observed),
        'first_pose_within_xy_tolerance': observed[first_index],
        'later_pose_count': len(later),
        'maximum_later_xy_error_m': max(
            (pose['xy_error_m'] for pose in later), default=None
        ),
        'later_pose_outside_xy_tolerance': any(
            pose['xy_error_m'] > tolerance_m for pose in later
        ),
    }


def _goal_checker_evidence(
    *,
    mission: dict[str, Any],
    tf_pose: dict[str, Any] | None,
    feedback_history: Iterable[dict[str, Any] | None],
    amcl_history: Iterable[dict[str, Any] | None],
    runtime_goal_checker: dict[str, Any] | None,
    xy_tolerance_m: float,
    yaw_tolerance_rad: float | None,
) -> dict[str, Any]:
    feedback = [pose for pose in feedback_history if pose is not None]
    amcl = [pose for pose in amcl_history if pose is not None]

    def valid_history_pose(pose: dict[str, Any]) -> bool:
        return bool(
            pose.get('frame_id', '').strip('/') == 'map'
            and pose.get('child_frame_id', '').strip('/') == 'base_link'
            and int(pose.get('received_ns') or 0) > 0
            and int(pose.get('source_stamp_ns') or 0) > 0
        )

    valid_feedback = [pose for pose in feedback if valid_history_pose(pose)]
    valid_amcl = [pose for pose in amcl if valid_history_pose(pose)]
    first_feedback_entry = next(
        (
            pose for pose in valid_feedback
            if pose['xy_error_m'] <= xy_tolerance_m
        ),
        None,
    )
    base = {
        'diagnostic_only': True,
        'accepted': False,
        'classification': 'insufficient_or_inconsistent_pose_evidence',
        'xy_tolerance_m': xy_tolerance_m,
        'yaw_tolerance_rad': yaw_tolerance_rad,
        'stateful': (
            runtime_goal_checker.get('stateful')
            if runtime_goal_checker is not None else None
        ),
        'valid_action_feedback_pose_count': len(valid_feedback),
        'invalid_action_feedback_pose_count': len(feedback) - len(valid_feedback),
        'valid_amcl_pose_count': len(valid_amcl),
        'invalid_amcl_pose_count': len(amcl) - len(valid_amcl),
        'first_proven_xy_entry': first_feedback_entry,
        'xy_entry_proof_source': (
            'navigate_to_pose_feedback' if first_feedback_entry else None
        ),
    }
    if not mission.get('succeeded'):
        return {**base, 'classification': 'mission_not_succeeded'}
    if tf_pose is None or yaw_tolerance_rad is None:
        return base
    if (
        tf_pose.get('frame_id', '').strip('/') != 'map'
        or tf_pose.get('child_frame_id', '').strip('/') != 'base_link'
    ):
        return base
    if tf_pose['yaw_error_rad'] > yaw_tolerance_rad:
        return {**base, 'classification': 'outside_yaw_tolerance_at_success'}
    if tf_pose['xy_error_m'] <= xy_tolerance_m:
        return {
            **base,
            'accepted': True,
            'classification': 'within_xy_tolerance_at_success',
        }
    if not (runtime_goal_checker and runtime_goal_checker.get('stateful')):
        return {**base, 'classification': 'outside_xy_tolerance_at_success'}
    if first_feedback_entry is not None:
        return {
            **base,
            'accepted': True,
            'classification': 'stateful_xy_latched_after_proven_entry',
        }
    if valid_feedback:
        return {**base, 'classification': 'success_without_proven_xy_entry'}
    return base


def _map_odom_drift(
    transforms: Iterable[TransformRecord], start_ns: int | None,
    completion_ns: int | None,
) -> dict[str, Any]:
    samples = sorted(
        (
            item for item in transforms
            if item.parent_frame.strip('/') == 'map'
            and item.child_frame.strip('/') == 'odom'
            and not item.is_static
            and start_ns is not None and completion_ns is not None
            and start_ns <= item.received_ns <= completion_ns
        ),
        key=lambda item: item.received_ns,
    )
    if len(samples) < 2:
        return {'sample_count': len(samples), 'translation_change_m': None,
                'yaw_change_rad': None}
    first, last = samples[0], samples[-1]
    return {
        'sample_count': len(samples),
        'first_received_ns': first.received_ns,
        'last_received_ns': last.received_ns,
        'translation_change_m': math.hypot(last.x - first.x, last.y - first.y),
        'yaw_change_rad': abs(math.atan2(
            math.sin(last.yaw - first.yaw), math.cos(last.yaw - first.yaw)
        )),
    }


def summarize_runtime_evidence(
    collector: RuntimeEvidenceCollector,
    *,
    scenario: str,
    start: tuple[float, float, float],
    goal: tuple[float, float, float],
    geometry: dict[str, float],
    nav2_config: dict[str, Any],
    map_path: Path,
    layer_enabled: bool | None,
    baseline_bundle: dict[str, Any] | None,
    minimum_path_change_m: float,
    minimum_trajectory_change_m: float,
    maximum_plan_tracking_error_m: float,
    goal_tolerance_m: float,
    yaw_goal_tolerance_rad: float | None = None,
    runtime_goal_checker: dict[str, Any] | None = None,
    runtime_profile: str = 'authoritative_full',
) -> tuple[dict[str, Any], list[dict[str, Any]]]:
    inflation_radius_m = float(nav2_config['inflation_radius_m'])
    inflation_geometry = expanded_hazard_geometry(geometry, inflation_radius_m)
    mission = collector.selected_mission()
    costmaps = _runtime_costmap_analysis(
        collector, geometry, inflation_radius_m
    )
    plans, selected_plans = _runtime_plan_dicts(
        collector,
        mission=mission,
        costmaps=costmaps,
        geometry=geometry,
        inflation_geometry=inflation_geometry,
        minimum_path_change_m=minimum_path_change_m,
    )
    start_ns = mission.get('start_ns')
    completion_ns = mission.get('completion_ns')
    action_success_ns = completion_ns if mission.get('succeeded') else None
    poses = [
        item for item in collector.poses
        if start_ns is not None
        and item.received_ns >= start_ns
        and (completion_ns is None or item.received_ns <= completion_ns)
    ]
    trajectory = [(item.x, item.y) for item in poses]
    trajectory_geometry = path_hazard_metrics(trajectory, geometry)
    trajectory_inflation = path_hazard_metrics(trajectory, inflation_geometry)
    mark_bound_ns = (
        costmaps['first_mark'].received_ns if costmaps['first_mark'] is not None else None
    )
    clear_bound_ns = (
        costmaps['first_clear'].received_ns if costmaps['first_clear'] is not None else None
    )
    footprint_poses = [
        item for item in poses
        if mark_bound_ns is None or (
            item.received_ns >= mark_bound_ns
            and (clear_bound_ns is None or item.received_ns < clear_bound_ns)
        )
    ]
    active_trajectory_geometry = path_hazard_metrics(
        [(item.x, item.y) for item in footprint_poses], geometry
    )
    footprint_overlap = trajectory_footprint_overlap(
        footprint_poses, geometry, _global_robot_footprint(nav2_config['source_yaml'])
    )
    footprint_overlap['evaluation_window'] = (
        'entire_mission_without_observed_aerial_mark'
        if mark_bound_ns is None else 'aerial_mark_active_interval'
    )
    footprint_overlap['mark_received_ns'] = mark_bound_ns
    footprint_overlap['clear_received_ns'] = clear_bound_ns
    driven_distance_m = path_length(trajectory)
    amcl_at_success = max(poses, key=lambda item: item.received_ns) if poses else None
    later_amcl = next(
        (item for item in reversed(collector.poses)
         if completion_ns is not None and item.received_ns >= completion_ns),
        amcl_at_success,
    )
    feedback_at_success = next(
        (item for item in reversed(collector.feedback)
         if item.goal_id == mission.get('goal_id')
         and completion_ns is not None and item.received_ns <= completion_ns),
        None,
    )
    tf_resolution_at_success = (
        resolve_planar_tf_pose(
            collector.transforms,
            parent_frame='map', child_frame='base_link',
            at_received_ns=action_success_ns,
        ) if action_success_ns is not None else None
    )
    tf_at_success = (
        tf_resolution_at_success
        if tf_resolution_at_success
        and tf_resolution_at_success.get('status') == 'resolved'
        else None
    )
    tf_goal_pose = _pose_evaluation(tf_at_success, goal)
    feedback_goal_pose = _pose_evaluation(
        feedback_at_success.pose if feedback_at_success else None,
        goal,
        action_success_received_ns=action_success_ns,
    )
    amcl_goal_pose = _pose_evaluation(
        amcl_at_success,
        goal,
        action_success_received_ns=action_success_ns,
    )
    later_amcl_goal_pose = _pose_evaluation(
        later_amcl,
        goal,
        action_success_received_ns=action_success_ns,
    )
    feedback_history = [
        _pose_evaluation(
            item.pose,
            goal,
            action_success_received_ns=action_success_ns,
        )
        for item in sorted(collector.feedback, key=lambda item: item.received_ns)
        if item.goal_id == mission.get('goal_id')
        and start_ns is not None
        and item.received_ns >= start_ns
        and (completion_ns is None or item.received_ns <= completion_ns)
    ]
    amcl_history = [
        _pose_evaluation(
            item,
            goal,
            action_success_received_ns=action_success_ns,
        )
        for item in poses
    ]
    stateful_xy_diagnostics = {
        'interpretation_limit': (
            'Observed samples can show an enter-then-drift pattern but do not expose '
            'the SimpleGoalChecker internal XY latch.'
        ),
        'action_feedback': _xy_tolerance_history_diagnostics(
            feedback_history, goal_tolerance_m
        ),
        'amcl': _xy_tolerance_history_diagnostics(
            amcl_history, goal_tolerance_m
        ),
    }
    goal_checker_evidence = _goal_checker_evidence(
        mission=mission,
        tf_pose=tf_goal_pose,
        feedback_history=feedback_history,
        amcl_history=amcl_history,
        runtime_goal_checker=runtime_goal_checker,
        xy_tolerance_m=goal_tolerance_m,
        yaw_tolerance_rad=yaw_goal_tolerance_rad,
    )
    map_frame_outside_xy_tolerance = bool(
        tf_goal_pose is not None and tf_goal_pose['xy_error_m'] > goal_tolerance_m
    )
    map_odom_drift = _map_odom_drift(
        collector.transforms, start_ns, completion_ns
    )
    requested_goal = next(
        (item for item in reversed(collector.requested_goals)
         if start_ns is None or item.received_ns <= start_ns),
        collector.requested_goals[-1] if collector.requested_goals else None,
    )
    requested_goal_xy_error_m = (
        math.hypot(requested_goal.x - goal[0], requested_goal.y - goal[1])
        if requested_goal else None
    )
    requested_goal_yaw_error_rad = (
        abs(math.atan2(
            math.sin(requested_goal.yaw - goal[2]),
            math.cos(requested_goal.yaw - goal[2]),
        ))
        if requested_goal else None
    )
    requested_goal_matches_configuration = bool(
        requested_goal
        and requested_goal.frame_id.strip('/') == 'map'
        and requested_goal_xy_error_m <= 1.0e-6
        and requested_goal_yaw_error_rad <= 1.0e-6
    )
    baseline_trajectory = baseline_bundle['trajectory'] if baseline_bundle else []
    baseline_trajectory_geometry = path_hazard_metrics(
        baseline_trajectory, geometry
    ) if baseline_trajectory else None
    trajectory_change_m = discrete_hausdorff_distance(
        baseline_trajectory, trajectory
    ) if baseline_trajectory and trajectory else None
    first_mark = costmaps['first_mark']
    first_clear = costmaps['first_clear']
    changed_plan = selected_plans['hazard_active']
    mark_ns = first_mark.received_ns if first_mark else None
    automatic_replanning_observed = bool(
        changed_plan is not None
        and mark_ns is not None
        and start_ns is not None
        and changed_plan['result_ns'] > mark_ns >= start_ns
        and (completion_ns is None or changed_plan['result_ns'] <= completion_ns)
        and mission.get('single_active_goal_observed')
    )
    post_replan_poses = [
        item for item in poses
        if changed_plan is not None and item.received_ns >= changed_plan['result_ns']
    ]
    post_replan_deviations = [
        (item, point_to_path_distance((item.x, item.y), baseline_trajectory))
        for item in post_replan_poses
    ] if baseline_trajectory else []
    first_physical_deviation = next(
        (
            (item, distance)
            for item, distance in post_replan_deviations
            if distance is not None and distance >= minimum_trajectory_change_m
        ),
        None,
    )
    physical_detour_observed = bool(
        first_physical_deviation
        and trajectory_change_m is not None
        and trajectory_change_m >= minimum_trajectory_change_m
    )
    post_mark_plan_points = [
        tuple(point)
        for plan in plans
        if first_mark is not None and plan['result_ns'] >= first_mark.received_ns
        for point in plan['points']
    ]
    plan_tracking_error_m = directed_path_distance(
        ((item.x, item.y) for item in post_replan_poses),
        post_mark_plan_points,
    )
    for event in footprint_overlap['overlap_events']:
        event_ns = event['received_ns']
        event['time_from_mission_start_s'] = (
            (event_ns - start_ns) * 1.0e-9 if start_ns is not None else None
        )
        active_plan = max(
            (item for item in plans if item['result_ns'] <= event_ns),
            key=lambda item: item['result_ns'], default=None,
        )
        if active_plan is not None:
            event['active_plan'] = {
                'label': active_plan['label'],
                'result_ns': active_plan['result_ns'],
                'age_s': (event_ns - active_plan['result_ns']) * 1.0e-9,
                'distance_from_pose_to_plan_m': point_to_path_distance(
                    (event['pose']['x'], event['pose']['y']), active_plan['points']
                ),
                'minimum_centerline_clearance_to_core_m': active_plan[
                    'minimum_distance_to_covariance_footprint_m'
                ],
                'crosses_lethal_costmap_cell': active_plan[
                    'crosses_lethal_costmap_cell'
                ],
            }
        snapshot = max(
            (item for item in costmaps['snapshots'] if item.received_ns <= event_ns),
            key=lambda item: item.received_ns, default=None,
        )
        if snapshot is not None and costmaps['baseline'] is not None:
            delta = relevant_costmap_delta(
                costmaps['baseline'], snapshot, geometry, inflation_radius_m
            )
            event['costmap'] = {
                'snapshot_received_ns': snapshot.received_ns,
                'snapshot_age_s': (event_ns - snapshot.received_ns) * 1.0e-9,
                'source_kind': snapshot.source_kind,
                'comparable': delta['comparable'],
                'lethal_core_cell_count': delta.get(
                    'hazard_footprint_current_lethal_cells', 0
                ),
                'graded_inflation_cell_count': delta.get(
                    'inflation_halo_nonzero_cells', 0
                ),
            }
        hazard_sample = max(
            (
                item for item in collector.samples[UGV_TOPIC]
                if item.received_ns <= event_ns
            ),
            key=lambda item: item.received_ns, default=None,
        )
        if hazard_sample is not None:
            hazard = hazard_sample.message.hazards[0] if hazard_sample.message.hazards else None
            event['active_ugv_hazard'] = {
                'message_received_ns': hazard_sample.received_ns,
                'message_age_s': (event_ns - hazard_sample.received_ns) * 1.0e-9,
                'nonempty': hazard is not None,
                'array_stamp_ns': stamp_ns(hazard_sample.message.header.stamp),
            }
            if hazard is not None:
                last_seen_ns = stamp_ns(hazard.last_seen)
                event['active_ugv_hazard'].update({
                    'track_id': str(hazard.detection.id),
                    'state': int(hazard.state),
                    'last_seen_ns': last_seen_ns,
                    'ttl_ns': stamp_ns(hazard.ttl),
                    'ttl_deadline_ns': last_seen_ns + stamp_ns(hazard.ttl),
                    'geometry': effective_hazard_geometry(
                        hazard,
                        covariance_sigma_scale=(
                            nav2_config['aerial_covariance_sigma_scale']
                        ),
                    ),
                })
    baseline_plans = baseline_bundle['plans'] if baseline_bundle else []
    baseline_reference_plan = next(
        (item for item in baseline_plans if item.get('label') == 'baseline'),
        baseline_plans[0] if baseline_plans else None,
    )

    typed = collector.summarize(EvidenceExpectations(
        expected_state=(
            None if scenario == 'baseline' else AerialHazard.CONFIRMED
        ),
        expected_sources=(() if scenario == 'baseline' else ('dji1',)),
        expected_selected_source=(None if scenario == 'baseline' else 'dji1'),
        minimum_hazard_count=(0 if scenario == 'baseline' else 1),
        require_typed_flow=(scenario != 'baseline'),
        require_forwarding=(scenario != 'baseline'),
        require_covariance_match=(scenario != 'baseline'),
        max_age_s=float(nav2_config['aerial_max_observation_age_s']),
    ))
    mark_delta = costmaps['first_mark_delta'] or {}
    clearing = _clearing_mechanism(
        collector,
        first_clear.received_ns if first_clear else None,
        float(nav2_config['aerial_max_observation_age_s']),
    )
    post_clear_poses = [
        item for item in poses
        if first_clear is not None and item.received_ns >= first_clear.received_ns
    ]
    post_clear_driven_distance_m = path_length(
        (item.x, item.y) for item in post_clear_poses
    )
    navigation_continued_after_clear = bool(
        first_clear is not None
        and completion_ns is not None
        and first_clear.received_ns < completion_ns
        and len(post_clear_poses) >= 2
        and post_clear_driven_distance_m > 0.05
    )
    failures: list[str] = []
    if not mission['goal_id']:
        failures.append('navigate_to_pose_goal_not_observed')
    if mission.get('start_ns') is not None and mission['start_ns'] <= 0:
        failures.append('invalid_runtime_timestamp')
    if mission.get('completion_ns') is not None and mission['completion_ns'] <= 0:
        failures.append('invalid_runtime_timestamp')
    if mission.get('completion_ns') is None:
        failures.append('navigate_to_pose_goal_not_completed')
    elif not mission['succeeded']:
        failures.append('navigate_to_pose_goal_not_succeeded')
    if mission.get('other_goal_ids_during_mission'):
        failures.append('navigate_to_pose_goal_identity_ambiguous')
    if mission.get('contradictory_terminal_statuses'):
        failures.append('navigate_to_pose_terminal_status_contradictory')
    if requested_goal is None:
        failures.append('requested_action_goal_not_observed')
    elif not requested_goal_matches_configuration:
        failures.append('requested_action_goal_mismatch')
    if runtime_goal_checker is None:
        failures.append('runtime_goal_checker_parameters_unavailable')
    elif (
        'general_goal_checker' not in runtime_goal_checker.get('goal_checker_plugins', ())
        or runtime_goal_checker.get('stateful') != nav2_config['goal_checker_stateful']
        or runtime_goal_checker.get('plugin') != nav2_config['goal_checker_plugin']
        or runtime_goal_checker.get('xy_goal_tolerance_m') != goal_tolerance_m
        or runtime_goal_checker.get('yaw_goal_tolerance_rad') != yaw_goal_tolerance_rad
    ):
        failures.append('runtime_goal_checker_parameter_mismatch')
    if len(trajectory) < 2 or driven_distance_m < 1.0:
        failures.append('ugv_trajectory_missing')
    if selected_plans['automatic_plan_count_during_goal'] < 1:
        failures.append('active_mission_plan_not_observed')
    if costmaps['baseline'] is None:
        failures.append('stable_pre_hazard_costmap_missing')
    if any((
        collector.pose_dropped_count,
        collector.plan_dropped_count,
        collector.status_dropped_count,
        *collector.dropped_counts.values(),
    )):
        failures.append('bounded_evidence_limit_exceeded')

    expected_layer = scenario != 'baseline'
    if layer_enabled is None:
        failures.append('aerial_layer_runtime_parameter_unavailable')
    elif layer_enabled != expected_layer:
        failures.append('aerial_layer_runtime_parameter_mismatch')

    if scenario == 'baseline':
        if _has_nonempty(collector, UGV_TOPIC):
            failures.append('baseline_received_operational_hazard')
        baseline_plan = selected_plans['pre_hazard']
        if baseline_plan is None or not baseline_plan['crosses_covariance_footprint']:
            failures.append('baseline_plan_misses_candidate_hazard')
        if not trajectory_geometry['crosses_effective_hazard']:
            failures.append('baseline_trajectory_misses_candidate_hazard')
    else:
        failures.extend(f'hazard_flow:{item}' for item in typed['failures'])
        if baseline_bundle is None:
            failures.append('baseline_runtime_evidence_missing')
        elif baseline_bundle['summary'].get('status') != 'pass':
            failures.append('baseline_runtime_evidence_not_passed')
        if first_mark is None:
            failures.append('aerial_costmap_mark_not_observed')
        if mark_delta.get('hazard_footprint_lethal_cells', 0) < 1:
            failures.append('lethal_hazard_core_not_observed')
        if mark_delta.get('inflation_halo_nonzero_cells', 0) < 1:
            failures.append('graded_inflation_halo_not_observed')
        if (
            scenario == 'valid'
            and first_clear is not None
            and (completion_ns is None or first_clear.received_ns < completion_ns)
            and (
                clearing['source_explicit_empty_ns'] is None
                or clearing['source_explicit_empty_ns'] > first_clear.received_ns
            )
        ):
            failures.append('aerial_costmap_cleared_during_active_hazard')
        pre_hazard = selected_plans['pre_hazard']
        active = selected_plans['hazard_active']
        if pre_hazard is None:
            failures.append('pre_hazard_active_goal_plan_missing')
        elif not pre_hazard['crosses_covariance_footprint']:
            failures.append('pre_hazard_plan_misses_effective_hazard')
        if active is None:
            failures.append('automatic_post_mark_replan_not_observed')
        elif not mission.get('single_active_goal_observed'):
            failures.append('automatic_replan_goal_identity_ambiguous')
        elif not automatic_replanning_observed:
            failures.append('automatic_replan_timing_invalid')
        elif active['crosses_lethal_costmap_cell'] is not False:
            failures.append('automatic_replan_crosses_lethal_cost')
        if active_trajectory_geometry['crosses_effective_hazard']:
            failures.append('ugv_trajectory_crosses_effective_hazard_while_active')
        if footprint_overlap['overlapping_recorded_pose_count'] > 0:
            failures.append('ugv_footprint_intersects_effective_hazard')
        if baseline_reference_plan is None or not baseline_reference_plan.get(
            'crosses_covariance_footprint', False
        ):
            failures.append('baseline_reference_plan_not_hazard_relevant')
        if (
            baseline_trajectory_geometry is None
            or not baseline_trajectory_geometry['crosses_effective_hazard']
        ):
            failures.append('baseline_reference_trajectory_not_hazard_relevant')
        if not physical_detour_observed:
            failures.append('ugv_trajectory_did_not_materially_deviate')
        if (
            plan_tracking_error_m is None
            or plan_tracking_error_m > maximum_plan_tracking_error_m
        ):
            failures.append('ugv_trajectory_did_not_follow_replanned_corridor')
        if scenario == 'clearing':
            if first_clear is None:
                failures.append('aerial_costmap_clear_not_observed')
            if not clearing['explicit_empty_propagation_complete']:
                failures.append('explicit_empty_clearing_propagation_incomplete')
            if not navigation_continued_after_clear:
                failures.append('navigation_did_not_continue_after_clear')
            if selected_plans['post_clear'] is None:
                failures.append('post_clear_active_goal_plan_missing')

    replanning_latency_s = (
        (changed_plan['result_ns'] - mark_ns) * 1.0e-9
        if changed_plan is not None and mark_ns is not None else None
    )
    map_resolved = map_path.expanduser().resolve()
    map_values = _simple_map_yaml(map_resolved)
    map_image = Path(str(map_values['image']).strip('"\''))
    if not map_image.is_absolute():
        map_image = map_resolved.parent / map_image
    map_image = map_image.resolve()
    stationary = stationary_periods(poses)
    inconclusive_reasons = []
    if not mission.get('goal_id'):
        inconclusive_reasons.append('mission_identity_missing')
    if mission.get('completion_ns') is None:
        inconclusive_reasons.append('terminal_status_missing')
    if runtime_goal_checker is None:
        inconclusive_reasons.append('runtime_goal_checker_parameters_missing')
    if 'bounded_evidence_limit_exceeded' in failures:
        inconclusive_reasons.append('bounded_evidence_limit_exceeded')
    result_classification = (
        'pass' if not failures
        else 'inconclusive' if inconclusive_reasons
        else 'fail'
    )
    summary = {
        'schema_version': 7,
        'status': result_classification,
        'classification': result_classification,
        'inconclusive_reasons': inconclusive_reasons,
        'scenario': scenario,
        'validated_scope': {
            'authoritative_full': 'full_baylands_navigate_to_pose_runtime',
            'downstream_track_a': 'baylands_downstream_support_navigation_runtime',
            'reduced_resource_diagnostic': 'non_authoritative_reduced_resource_track_a_diagnostic',
        }[runtime_profile],
        'time_model': 'explicit_best_effort_simulation_clock_with_wall_time_timeout',
        'configuration': {
            'runtime_profile': runtime_profile,
            'authoritative_full_runtime': runtime_profile == 'authoritative_full',
            'authoritative_downstream_track_a': runtime_profile == 'downstream_track_a',
            'start': {'x': start[0], 'y': start[1], 'yaw': start[2]},
            'goal': {'x': goal[0], 'y': goal[1], 'yaw': goal[2]},
            'requested_action_goal': (
                {
                    'x': requested_goal.x,
                    'y': requested_goal.y,
                    'yaw': requested_goal.yaw,
                    'frame_id': requested_goal.frame_id,
                    'source_stamp_ns': requested_goal.source_stamp_ns,
                    'received_ns': requested_goal.received_ns,
                    'source': 'ugv_nav2_driver planned_path',
                    'xy_error_to_configured_goal_m': requested_goal_xy_error_m,
                    'yaw_error_to_configured_goal_rad': requested_goal_yaw_error_rad,
                    'matches_configured_goal': requested_goal_matches_configuration,
                }
                if requested_goal else None
            ),
            'map_yaml': str(map_resolved),
            'map_sha256': hashlib.sha256(map_resolved.read_bytes()).hexdigest(),
            'map_image': str(map_image),
            'map_image_sha256': hashlib.sha256(map_image.read_bytes()).hexdigest(),
            'nav2': nav2_config,
            'aerial_layer_enabled_requested': expected_layer,
            'aerial_layer_enabled_observed': layer_enabled,
            'manual_planner_requests_issued_by_evidence': 0,
            'costmap_capture': {
                'scope': 'hazard_region_crop',
                'margin_beyond_covariance_footprint_m': inflation_radius_m + 1.0,
                'reason': 'bound memory while retaining core, inflation, and clearing evidence',
            },
        },
        'hazard_geometry': {
            **geometry,
            'covariance_expansion_model': 'nominal + 2*sigma on each side',
            'inflation_radius_m': inflation_radius_m,
            'inflation_expanded_size_x': inflation_geometry['effective_size_x'],
            'inflation_expanded_size_y': inflation_geometry['effective_size_y'],
        },
        'mission': {
            **mission,
            'action_success_received_ns': action_success_ns,
            'duration_s': (
                (completion_ns - start_ns) * 1.0e-9
                if start_ns is not None and completion_ns is not None else None
            ),
            'goal_tolerance_m': goal_tolerance_m,
            'yaw_goal_tolerance_rad': yaw_goal_tolerance_rad,
            'tf_resolution_at_action_success': tf_resolution_at_success,
            'tf_pose_at_common_time': tf_goal_pose,
            'action_feedback_pose_at_or_before_success': feedback_goal_pose,
            'feedback_distance_remaining_m': (
                feedback_at_success.distance_remaining_m
                if feedback_at_success else None
            ),
            'feedback_number_of_recoveries': (
                feedback_at_success.number_of_recoveries
                if feedback_at_success else None
            ),
            'amcl_pose_at_or_before_success': amcl_goal_pose,
            'later_shutdown_amcl_pose': later_amcl_goal_pose,
            'comparison_frame': 'map',
            'compared_child_frame': 'base_link',
            'runtime_goal_checker': runtime_goal_checker,
            'goal_checker_evidence': goal_checker_evidence,
            'success_contract': 'matching NavigateToPose action SUCCEEDED with unambiguous goal and physical UGV motion',
            'localization_diagnostic': {
                'independent_map_frame_xy_outside_controller_tolerance': map_frame_outside_xy_tolerance,
                'map_frame_tf_xy_error_m': (
                    tf_goal_pose['xy_error_m'] if tf_goal_pose else None
                ),
                'amcl_xy_error_m': (
                    amcl_goal_pose['xy_error_m'] if amcl_goal_pose else None
                ),
                'map_to_odom_drift': map_odom_drift,
                'amcl_warnings': list(collector.amcl_warnings),
                'amcl_warning_count': collector.amcl_warning_count,
                'amcl_warning_source': 'rosout' if collector.amcl_warnings else 'not_observed',
                'classification': (
                    'limitation' if map_frame_outside_xy_tolerance
                    else 'unavailable' if tf_goal_pose is None else 'within_tolerance'
                ),
            },
            'stateful_xy_diagnostics': stateful_xy_diagnostics,
            'stateful_goal_checker_semantics': (
                'Once XY first passes, XY remains latched and is not rechecked '
                'while yaw is evaluated until the goal checker is reset'
                if runtime_goal_checker and runtime_goal_checker.get('stateful') else
                'XY and yaw are checked together'
            ),
            'goal_checker_interpretation_limit': (
                'Static semantics do not prove this run legitimately reached the goal; '
                'compare TF, feedback, and AMCL timing around the success transition.'
            ),
        },
        'hazard_flow': typed,
        'costmap': {
            'full_message_count': collector.costmap_full_count,
            'update_message_count': collector.costmap_update_count,
            'stable_baseline': costmaps['baseline_settling'],
            'first_hazard_ns': costmaps['first_hazard_ns'],
            'first_mark_ns': mark_ns,
            'first_mark_latency_s': (
                (mark_ns - costmaps['first_hazard_ns']) * 1.0e-9
                if mark_ns is not None and costmaps['first_hazard_ns'] is not None
                else None
            ),
            'first_mark_delta': mark_delta,
            'first_clear_ns': first_clear.received_ns if first_clear else None,
            'clearing_latency_s': (
                (first_clear.received_ns - mark_ns) * 1.0e-9
                if first_clear is not None and mark_ns is not None else None
            ),
            'restored_to_pre_hazard_in_analysis_region': first_clear is not None,
            'post_clear_pose_count': len(post_clear_poses),
            'post_clear_driven_distance_m': post_clear_driven_distance_m,
            'navigation_continued_after_clear': navigation_continued_after_clear,
            'preservation_scope': (
                'exact pre-hazard cell values across the covariance footprint, '
                'inflation radius, and one-metre margin'
            ),
            'clearing': clearing,
        },
        'planner': {
            'evidence_source': 'passive /plan subscription during active NavigateToPose',
            'manual_compute_path_requests': 0,
            'automatic_mission_replanning': {
                'observed': automatic_replanning_observed,
                'basis': (
                    'materially changed passive /plan output after the aerial mark '
                    'inside one unambiguous NavigateToPose mission lifetime; the '
                    'evidence process issued zero ComputePath requests'
                ),
                'goal_id': mission['goal_id'],
                'single_active_goal_observed': mission.get(
                    'single_active_goal_observed', False
                ),
                'other_goal_ids_during_mission': mission.get(
                    'other_goal_ids_during_mission', []
                ),
                'mark_ns': mark_ns,
                'changed_plan_ns': (
                    changed_plan['result_ns'] if changed_plan else None
                ),
                'material_change_m': selected_plans['material_change_m'],
            },
            'plans': plans,
            **selected_plans,
            'replanning_latency_from_first_mark_s': replanning_latency_s,
        },
        'trajectory': {
            'pose_count': len(trajectory),
            'active_hazard_interval': {
                'mark_received_ns': mark_bound_ns,
                'clear_received_ns': clear_bound_ns,
                'pose_count': len(footprint_poses),
                'minimum_distance_to_covariance_footprint_m': (
                    active_trajectory_geometry[
                        'minimum_distance_to_effective_hazard_m'
                    ]
                ),
                'crosses_covariance_footprint': active_trajectory_geometry[
                    'crosses_effective_hazard'
                ],
            },
            'virtual_core_robot_footprint_diagnostic': footprint_overlap,
            'driven_distance_m': driven_distance_m,
            'minimum_distance_to_covariance_footprint_m': trajectory_geometry[
                'minimum_distance_to_effective_hazard_m'
            ],
            'crosses_covariance_footprint': trajectory_geometry[
                'crosses_effective_hazard'
            ],
            'minimum_distance_to_inflation_region_m': trajectory_inflation[
                'minimum_distance_to_effective_hazard_m'
            ],
            'crosses_inflation_region': trajectory_inflation['crosses_effective_hazard'],
            'hausdorff_distance_from_baseline_m': trajectory_change_m,
            'physical_detour_observed': physical_detour_observed,
            'samples_before_replan': sum(
                1 for item in poses
                if changed_plan is not None
                and item.received_ns < changed_plan['result_ns']
            ),
            'samples_at_or_after_replan': len(post_replan_poses),
            'first_physical_deviation_after_replan': (
                {
                    'received_ns': first_physical_deviation[0].received_ns,
                    'time_from_replan_s': (
                        first_physical_deviation[0].received_ns
                        - changed_plan['result_ns']
                    ) * 1.0e-9,
                    'x': first_physical_deviation[0].x,
                    'y': first_physical_deviation[0].y,
                    'distance_from_baseline_trajectory_m': (
                        first_physical_deviation[1]
                    ),
                }
                if first_physical_deviation and changed_plan else None
            ),
            'maximum_post_replan_deviation_from_baseline_m': max(
                (
                    distance for _, distance in post_replan_deviations
                    if distance is not None
                ),
                default=None,
            ),
            'maximum_distance_to_post_mark_plan_history_m': plan_tracking_error_m,
            'maximum_plan_tracking_error_m': maximum_plan_tracking_error_m,
            'stationary_periods': stationary,
            'stationary_total_s': sum(item['duration_s'] for item in stationary),
            'baseline_evidence': baseline_bundle['root'] if baseline_bundle else None,
        },
        'dropped_evidence': {
            'poses': collector.pose_dropped_count,
            'feedback': collector.feedback_dropped_count,
            'transforms': collector.transform_dropped_count,
            'requested_goals': collector.requested_goal_dropped_count,
            'plans': collector.plan_dropped_count,
            'statuses': collector.status_dropped_count,
            'hazards': dict(collector.dropped_counts),
        },
        'failures': failures,
        'limitations': [
            'Synthetic hazards substitute for the deferred support-UAV detector.',
            'A passing run establishes one configured Baylands mission, not general safety.',
            'No perception accuracy or EiraX rolling-costmap behavior is inferred.',
            'Independent current map-frame localization accuracy is reported separately from Nav2 mission success.',
        ],
    }
    trajectory_rows = [
        {
            'received_ns': item.received_ns,
            'time_from_mission_start_s': (
                (item.received_ns - start_ns) * 1.0e-9 if start_ns is not None else None
            ),
            'x': item.x,
            'y': item.y,
            'yaw': item.yaw,
        }
        for item in poses
    ]
    return summary, trajectory_rows


def write_runtime_evidence(
    output_dir: Path,
    summary: dict[str, Any],
    collector: RuntimeEvidenceCollector,
    trajectory_rows: list[dict[str, Any]],
) -> None:
    geometry = summary['hazard_geometry']
    baseline_snapshot = _runtime_costmap_analysis(
        collector, geometry, float(geometry['inflation_radius_m'])
    )['baseline']
    if baseline_snapshot is None:
        costmap_rows = []
    else:
        costmap_rows = _costmap_rows(
            [
                (f'snapshot_{index:04d}', snapshot)
                for index, snapshot in enumerate(collector.costmaps, start=1)
            ],
            baseline_snapshot,
            geometry,
            float(geometry['inflation_radius_m']),
        )
    write_planner_evidence(
        output_dir,
        summary,
        collector.hazard_rows(
            covariance_sigma_scale=float(
                summary['configuration']['nav2']['aerial_covariance_sigma_scale']
            )
        ),
        costmap_rows,
    )
    with (output_dir / 'trajectory.csv').open('w', newline='', encoding='utf-8') as stream:
        fields = ('received_ns', 'time_from_mission_start_s', 'x', 'y', 'yaw')
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(trajectory_rows)
    mission_rows = [
        {
            'received_ns': item.received_ns,
            'goal_id': item.goal_id,
            'status': GOAL_STATUS_NAMES.get(item.status, str(item.status)),
        }
        for item in collector.status_events
    ]
    with (output_dir / 'mission_timeline.csv').open(
        'w', newline='', encoding='utf-8'
    ) as stream:
        writer = csv.DictWriter(
            stream, fieldnames=('received_ns', 'goal_id', 'status')
        )
        writer.writeheader()
        writer.writerows(mission_rows)
    overlay_plans = list(summary['planner']['plans'])
    overlay_plans.append({
        'label': 'ugv_trajectory',
        'points': [[row['x'], row['y']] for row in trajectory_rows],
    })
    (output_dir / 'runtime_overlay.svg').write_text(
        _planner_overlay_svg(overlay_plans, geometry), encoding='utf-8'
    )


def _run_runtime_live(args: argparse.Namespace, ros_args: list[str]) -> int:
    if args.scenario not in ('baseline', 'valid', 'clearing'):
        raise ValueError('runtime scenario must be baseline, valid, or clearing')
    nav2_config = load_nav2_inflation_config(args.nav2_config)
    if nav2_config['global_costmap_rolling_window']:
        raise ValueError(
            'runtime profile requires the fixed Baylands global costmap; '
            'rolling grid correction is pending'
        )
    baseline_bundle = _runtime_baseline_bundle(
        args.baseline_evidence if args.scenario != 'baseline' else None
    )
    uncertainty_x = args.covariance_sigma_scale * math.sqrt(args.variance_x)
    uncertainty_y = args.covariance_sigma_scale * math.sqrt(args.variance_y)
    geometry = {
        'center_x': float(args.hazard_x),
        'center_y': float(args.hazard_y),
        'yaw': float(args.hazard_yaw),
        'nominal_size_x': float(args.hazard_size_x),
        'nominal_size_y': float(args.hazard_size_y),
        'variance_x': float(args.variance_x),
        'variance_y': float(args.variance_y),
        'covariance_sigma_scale': float(args.covariance_sigma_scale),
        'effective_size_x': float(args.hazard_size_x) + 2.0 * uncertainty_x,
        'effective_size_y': float(args.hazard_size_y) + 2.0 * uncertainty_y,
    }
    collector = RuntimeEvidenceCollector(
        max_samples_per_topic=args.max_samples_per_topic,
        max_costmaps=args.max_costmaps,
        max_poses=args.max_poses,
        max_plans=args.max_plans,
        crop_geometry=geometry,
        crop_margin_m=float(nav2_config['inflation_radius_m']) + 1.0,
    )
    rclpy.init(args=ros_args)
    node = RuntimeEvidenceNode(collector, args.namespace)
    layer_enabled = None
    runtime_goal_checker = None
    try:
        service_deadline = time.monotonic() + min(180.0, args.timeout_s)
        while time.monotonic() < service_deadline and rclpy.ok():
            if _request_costmap_snapshot(node, 1.0):
                break
            rclpy.spin_once(node, timeout_sec=0.1)
        layer_enabled = _get_aerial_layer_enabled(node, 10.0)
        runtime_goal_checker = _get_runtime_goal_checker(node, 10.0)
        deadline = time.monotonic() + args.timeout_s
        terminal_seen_at = None
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            mission = collector.selected_mission()
            if mission['completion_ns'] is not None and terminal_seen_at is None:
                terminal_seen_at = time.monotonic()
            if (
                terminal_seen_at is not None
                and time.monotonic() - terminal_seen_at >= args.post_terminal_s
            ):
                break
        summary, trajectory_rows = summarize_runtime_evidence(
            collector,
            scenario=args.scenario,
            start=(args.start_x, args.start_y, args.start_yaw),
            goal=(args.goal_x, args.goal_y, args.goal_yaw),
            geometry=geometry,
            nav2_config=nav2_config,
            map_path=args.map,
            layer_enabled=layer_enabled,
            baseline_bundle=baseline_bundle,
            minimum_path_change_m=args.minimum_path_change_m,
            minimum_trajectory_change_m=args.minimum_trajectory_change_m,
            maximum_plan_tracking_error_m=args.maximum_plan_tracking_error_m,
            goal_tolerance_m=(
                args.goal_tolerance_m
                if args.goal_tolerance_m is not None
                else float(
                    runtime_goal_checker['xy_goal_tolerance_m']
                    if runtime_goal_checker is not None
                    else nav2_config['goal_checker_xy_tolerance_m']
                )
            ),
            yaw_goal_tolerance_rad=float(
                runtime_goal_checker['yaw_goal_tolerance_rad']
                if runtime_goal_checker is not None
                else nav2_config['goal_checker_yaw_tolerance_rad']
            ),
            runtime_goal_checker=runtime_goal_checker,
            runtime_profile=args.runtime_profile,
        )
        if collector.selected_mission()['completion_ns'] is None:
            if 'runtime_timeout_before_terminal_status' not in summary['failures']:
                summary['failures'].append('runtime_timeout_before_terminal_status')
            if 'terminal_status_missing' not in summary['inconclusive_reasons']:
                summary['inconclusive_reasons'].append('terminal_status_missing')
            summary['status'] = 'inconclusive'
            summary['classification'] = 'inconclusive'
        write_runtime_evidence(args.output, summary, collector, trajectory_rows)
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if summary['status'] == 'pass' else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def _run_runtime_bag(args: argparse.Namespace) -> int:
    """Recompute a historical runtime verdict from its recorded ROS messages."""
    import rosbag2_py
    from rosidl_runtime_py.utilities import get_message

    recording_root = args.recording_root.expanduser().resolve()
    source_analysis = recording_root.parent / 'analysis'
    source = json.loads((source_analysis / 'summary.json').read_text(encoding='utf-8'))
    scenario = source.get('scenario')
    if scenario not in ('baseline', 'valid', 'clearing'):
        raise ValueError('runtime-bag requires a supported authoritative scenario')
    source_profile = source.get('configuration', {}).get('runtime_profile')
    if source_profile not in ('authoritative_full', 'downstream_track_a'):
        raise ValueError('runtime-bag requires an authoritative Track A recording')
    configuration = source['configuration']
    nav2_config_path = Path(configuration['nav2']['source_yaml'])
    nav2_config = load_nav2_inflation_config(nav2_config_path)
    recorded_nav2 = dict(configuration['nav2'])
    for key, value in nav2_config.items():
        if key not in recorded_nav2 and key in {
            'global_costmap_frame', 'global_robot_base_frame',
            'global_footprint_unpadded', 'global_footprint_padding_m',
            'global_footprint_padded', 'local_costmap_frame',
            'local_robot_base_frame', 'local_footprint_unpadded',
            'local_footprint_padding_m', 'local_footprint_padded',
            'local_inflation_radius_m', 'local_cost_scaling_factor',
            'local_aerial_layer_configured', 'local_aerial_target_frame',
        }:
            recorded_nav2[key] = value
    if nav2_config != recorded_nav2:
        raise ValueError('Nav2 configuration differs from the original live analysis')
    map_path = Path(configuration['map_yaml'])
    if hashlib.sha256(map_path.read_bytes()).hexdigest() != configuration['map_sha256']:
        raise ValueError('map YAML differs from the original live analysis')
    if hashlib.sha256(Path(configuration['map_image']).read_bytes()).hexdigest() != configuration['map_image_sha256']:
        raise ValueError('map image differs from the original live analysis')
    geometry = source['hazard_geometry']
    collector = RuntimeEvidenceCollector(
        crop_geometry=geometry,
        crop_margin_m=float(nav2_config['inflation_radius_m']) + 1.0,
    )
    bag_dir = recording_root / 'bag'
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr'
        ),
    )
    prefix = '/a201_0000'
    handlers = {
        **{topic: (AerialHazardArray, lambda msg, now, topic=topic: collector.add(topic, msg, now))
           for topic in HAZARD_TOPICS},
        f'{prefix}/global_costmap/costmap_raw': (Costmap, collector.add_full_costmap),
        f'{prefix}/global_costmap/costmap_raw_updates': (CostmapUpdate, collector.add_costmap_update),
        f'{prefix}/plan': (NavPath, collector.add_automatic_plan),
        f'{prefix}/planned_path': (NavPath, collector.add_requested_route),
        f'{prefix}/amcl_pose': (PoseWithCovarianceStamped, collector.add_pose),
        f'{prefix}/navigate_to_pose/_action/status': (GoalStatusArray, collector.add_status),
        f'{prefix}/navigate_to_pose/_action/feedback': (NavigateToPose_FeedbackMessage, collector.add_feedback),
        f'{prefix}/tf': (TFMessage, lambda msg, now: collector.add_tf(msg, now, is_static=False)),
        f'{prefix}/tf_static': (TFMessage, lambda msg, now: collector.add_tf(msg, now, is_static=True)),
        '/tf': (TFMessage, lambda msg, now: collector.add_tf(msg, now, is_static=False)),
        '/tf_static': (TFMessage, lambda msg, now: collector.add_tf(msg, now, is_static=True)),
        '/rosout': (Log, lambda msg, now: collector.add_rosout(msg)),
    }
    types = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    required = {
        '/clock', f'{prefix}/plan', f'{prefix}/amcl_pose',
        f'{prefix}/navigate_to_pose/_action/status',
        f'{prefix}/navigate_to_pose/_action/feedback',
        f'{prefix}/global_costmap/costmap_raw',
    }
    if missing := required - types.keys():
        raise ValueError(f'runtime recording is missing required topics: {sorted(missing)}')
    for topic, (message_type, _) in handlers.items():
        if topic in types and get_message(types[topic]) != message_type:
            raise ValueError(f'unexpected recorded message type for {topic}: {types[topic]}')
    now_ns = 0
    message_count = 0
    completion_ns = source.get('mission', {}).get('completion_ns')
    replay_cutoff_ns = (
        int(completion_ns) + 2_000_000_000
        if completion_ns is not None else None
    )
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic == '/clock':
            now_ns = stamp_ns(deserialize_message(data, Clock).clock)
            if replay_cutoff_ns is not None and now_ns > replay_cutoff_ns:
                break
        elif topic in handlers and now_ns > 0:
            message_type, handler = handlers[topic]
            handler(deserialize_message(data, message_type), now_ns)
            message_count += 1
    if not collector.amcl_warnings:
        for log_path in sorted((recording_root.parent / 'logs').glob('*.log')):
            for line in log_path.read_text(encoding='utf-8', errors='replace').splitlines():
                if 'amcl' in line.lower() and (
                    '[WARN]' in line or 'observations were not in the map' in line
                ):
                    collector.record_amcl_warning(line.strip())
    runtime_goal_checker = source['mission']['runtime_goal_checker']
    start = configuration['start']
    goal = configuration['goal']
    baseline_bundle = _runtime_baseline_bundle(args.baseline_evidence)
    if scenario != 'baseline' and baseline_bundle is None:
        raise ValueError('valid and clearing bag reanalysis require --baseline-evidence')
    summary, trajectory_rows = summarize_runtime_evidence(
        collector,
        scenario=scenario,
        start=(start['x'], start['y'], start['yaw']),
        goal=(goal['x'], goal['y'], goal['yaw']),
        geometry=geometry, nav2_config=nav2_config, map_path=map_path,
        layer_enabled=configuration['aerial_layer_enabled_observed'],
        baseline_bundle=baseline_bundle,
        minimum_path_change_m=0.5,
        minimum_trajectory_change_m=0.75,
        maximum_plan_tracking_error_m=2.0,
        goal_tolerance_m=runtime_goal_checker['xy_goal_tolerance_m'],
        yaw_goal_tolerance_rad=runtime_goal_checker['yaw_goal_tolerance_rad'],
        runtime_goal_checker=runtime_goal_checker,
        runtime_profile=source_profile,
    )
    summary['reanalysis'] = {
        'source_analysis': str(source_analysis),
        'source_recording': str(bag_dir),
        'source_summary_sha256': hashlib.sha256(
            (source_analysis / 'summary.json').read_bytes()
        ).hexdigest(),
        'recorded_messages_processed': message_count,
        'replay_cutoff_ns': replay_cutoff_ns,
        'runtime_parameters_from_original_live_capture': True,
    }
    if not types.get('/rosout') and collector.amcl_warnings:
        summary['mission']['localization_diagnostic']['amcl_warning_source'] = 'recorded_runtime_logs'
    write_runtime_evidence(args.output, summary, collector, trajectory_rows)
    print(json.dumps({
        'status': summary['status'], 'failures': summary['failures'],
        'inconclusive_reasons': summary['inconclusive_reasons'],
        'output': str(args.output), 'recorded_messages_processed': message_count,
    }, indent=2))
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
    runtime = subparsers.add_parser(
        'runtime-live',
        help='Passively capture one Baylands NavigateToPose mission.',
    )
    runtime.add_argument('--scenario', choices=('baseline', 'valid', 'clearing'), required=True)
    runtime.add_argument('--namespace', default='a201_0000')
    runtime.add_argument('--map', type=Path, required=True)
    runtime.add_argument('--nav2-config', type=Path, required=True)
    runtime.add_argument('--baseline-evidence', type=Path)
    runtime.add_argument(
        '--runtime-profile',
        choices=('authoritative_full', 'downstream_track_a', 'reduced_resource_diagnostic'),
        default='authoritative_full',
    )
    runtime.add_argument('--start-x', type=float, required=True)
    runtime.add_argument('--start-y', type=float, required=True)
    runtime.add_argument('--start-yaw', type=float, default=0.0)
    runtime.add_argument('--goal-x', type=float, required=True)
    runtime.add_argument('--goal-y', type=float, required=True)
    runtime.add_argument('--goal-yaw', type=float, default=0.0)
    runtime.add_argument('--hazard-x', type=float, required=True)
    runtime.add_argument('--hazard-y', type=float, required=True)
    runtime.add_argument('--hazard-yaw', type=float, default=0.0)
    runtime.add_argument('--hazard-size-x', type=float, default=2.0)
    runtime.add_argument('--hazard-size-y', type=float, default=2.0)
    runtime.add_argument('--variance-x', type=float, default=0.25)
    runtime.add_argument('--variance-y', type=float, default=0.25)
    runtime.add_argument('--covariance-sigma-scale', type=float, default=2.0)
    runtime.add_argument('--minimum-path-change-m', type=float, default=0.5)
    runtime.add_argument('--minimum-trajectory-change-m', type=float, default=0.75)
    runtime.add_argument('--maximum-plan-tracking-error-m', type=float, default=2.0)
    runtime.add_argument(
        '--goal-tolerance-m', type=float,
        help='Override the controller YAML general_goal_checker XY tolerance.',
    )
    runtime.add_argument('--timeout-s', type=float, default=300.0)
    runtime.add_argument('--post-terminal-s', type=float, default=3.0)
    runtime.add_argument('--max-samples-per-topic', type=int, default=5000)
    runtime.add_argument('--max-costmaps', type=int, default=900)
    runtime.add_argument('--max-poses', type=int, default=30000)
    runtime.add_argument('--max-plans', type=int, default=2000)
    runtime.add_argument('--output', type=Path, required=True)
    runtime_bag = subparsers.add_parser(
        'runtime-bag', help='Reanalyze an authoritative runtime bag offline.'
    )
    runtime_bag.add_argument('--recording-root', type=Path, required=True)
    runtime_bag.add_argument('--baseline-evidence', type=Path)
    runtime_bag.add_argument('--output', type=Path, required=True)
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
        elif parsed.mode == 'runtime-live':
            status = _run_runtime_live(parsed, ros_args)
        elif parsed.mode == 'runtime-bag':
            status = _run_runtime_bag(parsed)
        else:
            status = _run_map_check(parsed)
    except (FileExistsError, FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))
    raise SystemExit(status)


if __name__ == '__main__':
    main()
