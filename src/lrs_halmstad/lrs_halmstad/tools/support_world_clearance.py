"""Offline-only Gazebo-world clearance check for Track A recordings.

This module never creates a ROS node or publishes a topic. World poses are
read from a completed bag and cannot affect the recorded navigation run.
"""

from __future__ import annotations

import argparse
import ast
import hashlib
import json
import math
from pathlib import Path

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosgraph_msgs.msg import Clock
from tf2_msgs.msg import TFMessage
import yaml

from lrs_halmstad.sim.simulation_uav_localization import (
    fit_world_to_map, load_calibration_points, world_point_to_map,
)
from lrs_halmstad.tools.support_hazard_evidence import path_hazard_metrics


MODEL_TOPIC = '/model/a201_0000/robot/pose'
LEGACY_TOPIC = '/world/baylands/dynamic_pose/info'
MODEL_FRAME = 'a201_0000/robot'
WORLD_FRAME = 'baylands'


def registration(points):
    """Fit all distinct pairs and use maximum leave-one-out error as margin."""
    unique = []
    seen = set()
    for point in points:
        key = (point.world_x, point.world_y, point.map_x, point.map_y)
        if key not in seen:
            unique.append(point)
            seen.add(key)
    if len(unique) < 3:
        raise ValueError('at least three distinct world/map pairs are required')
    fit = fit_world_to_map(unique)
    residuals = []
    held_out = []
    for index, point in enumerate(unique):
        x, y, _ = world_point_to_map(
            (point.world_x, point.world_y, point.world_z), fit.yaw_rad,
            (fit.translation_x_m, fit.translation_y_m, fit.translation_z_m),
        )
        residuals.append((point.place, math.hypot(x - point.map_x, y - point.map_y)))
        alternate = fit_world_to_map(unique[:index] + unique[index + 1:])
        x, y, _ = world_point_to_map(
            (point.world_x, point.world_y, point.world_z), alternate.yaw_rad,
            (alternate.translation_x_m, alternate.translation_y_m, alternate.translation_z_m),
        )
        held_out.append((point.place, math.hypot(x - point.map_x, y - point.map_y)))
    margin = max(max(error for _, error in residuals), max(error for _, error in held_out))
    return unique, fit, residuals, held_out, margin


def map_to_world(x, y, fit):
    dx = x - fit.translation_x_m
    dy = y - fit.translation_y_m
    cosine = math.cos(fit.yaw_rad)
    sine = math.sin(fit.yaw_rad)
    return cosine * dx + sine * dy, -sine * dx + cosine * dy


def _hull(points):
    values = sorted(set(points))
    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    lower = []
    for point in values:
        while len(lower) >= 2 and cross(lower[-2], lower[-1], point) <= 0:
            lower.pop()
        lower.append(point)
    upper = []
    for point in reversed(values):
        while len(upper) >= 2 and cross(upper[-2], upper[-1], point) <= 0:
            upper.pop()
        upper.append(point)
    return lower[:-1] + upper[:-1]


def _inside_hull(point, hull):
    return all(
        (b[0] - a[0]) * (point[1] - a[1])
        - (b[1] - a[1]) * (point[0] - a[0]) >= -1.0e-6
        for a, b in zip(hull, hull[1:] + hull[:1])
    )


def _model_world_pose(message):
    matches = [
        item for item in message.transforms
        if item.header.frame_id == WORLD_FRAME and item.child_frame_id == MODEL_FRAME
    ]
    if len(matches) != 1:
        return None
    return matches[0].transform.translation.x, matches[0].transform.translation.y


def _recorded_world_poses(bag_dir, start_ns, completion_ns, allow_legacy):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id='mcap'),
        rosbag2_py.ConverterOptions(
            input_serialization_format='cdr', output_serialization_format='cdr',
        ),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    topic = MODEL_TOPIC if MODEL_TOPIC in types else LEGACY_TOPIC if allow_legacy and LEGACY_TOPIC in types else None
    if topic is None:
        return None, [], 'world_model_pose_topic_missing'
    if types[topic] != 'tf2_msgs/msg/TFMessage':
        return topic, [], 'world_model_pose_type_mismatch'
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/clock', topic]))
    now_ns = 0
    poses = []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name == '/clock':
            clock = deserialize_message(data, Clock).clock
            now_ns = clock.sec * 1_000_000_000 + clock.nanosec
            if now_ns > completion_ns + 1_000_000_000:
                break
            continue
        if not start_ns - 500_000_000 <= now_ns <= completion_ns + 500_000_000:
            continue
        message = deserialize_message(data, TFMessage)
        if topic == MODEL_TOPIC:
            point = _model_world_pose(message)
        else:
            # Old recordings lost entity names in this bridge conversion. The
            # first pose is only a candidate and can never establish PASS.
            point = None if not message.transforms else (
                message.transforms[0].transform.translation.x,
                message.transforms[0].transform.translation.y,
            )
        if point is not None and all(math.isfinite(value) for value in point):
            poses.append((now_ns, *point))
    return topic, poses, None


def evaluate(analysis_dir: Path, recording_dir: Path, waypoint_csv: Path, *, allow_legacy=False):
    summary = json.loads((analysis_dir / 'summary.json').read_text(encoding='utf-8'))
    if summary.get('scenario') != 'valid':
        raise ValueError('world clearance evaluation requires a valid scenario')
    mission = summary['mission']
    if mission.get('start_ns') is None or mission.get('completion_ns') is None:
        raise ValueError('completed mission timestamps are required')
    nav2_path = Path(summary['configuration']['nav2']['source_yaml'])
    nav2_bytes = nav2_path.read_bytes()
    if hashlib.sha256(nav2_bytes).hexdigest() != summary['configuration']['nav2']['source_sha256']:
        raise ValueError('Nav2 YAML differs from recorded configuration')
    nav2 = yaml.safe_load(nav2_bytes)
    footprint_text = nav2['global_costmap']['global_costmap']['ros__parameters']['footprint']
    footprint = ast.literal_eval(footprint_text) if isinstance(footprint_text, str) else footprint_text
    robot_radius = max(math.hypot(float(x), float(y)) for x, y in footprint)
    unique, fit, residuals, held_out, margin = registration(
        load_calibration_points(str(waypoint_csv), 'parkinglot_west')
    )
    geometry = dict(summary['hazard_geometry'])
    geometry['center_x'], geometry['center_y'] = map_to_world(
        geometry['center_x'], geometry['center_y'], fit
    )
    geometry['yaw'] -= fit.yaw_rad
    topic, poses, topic_error = _recorded_world_poses(
        recording_dir / 'bag', mission['start_ns'], mission['completion_ns'], allow_legacy
    )
    path = [(x, y) for _, x, y in poses]
    metrics = path_hazard_metrics(path, geometry)
    nominal = metrics['minimum_distance_to_effective_hazard_m']
    conservative = None if nominal is None else nominal - margin - robot_radius
    gaps = [(b[0] - a[0]) / 1.0e9 for a, b in zip(poses, poses[1:])]
    hull = _hull([(point.world_x, point.world_y) for point in unique])
    hazard_inside_hull = _inside_hull((geometry['center_x'], geometry['center_y']), hull)
    outside_count = sum(not _inside_hull(point, hull) for point in path)
    reasons = []
    if topic_error:
        reasons.append(topic_error)
    if topic == LEGACY_TOPIC:
        reasons.append('global_pose_index_has_no_recorded_entity_identity')
    if not poses or poses[0][0] > mission['start_ns'] + 500_000_000 or poses[-1][0] < mission['completion_ns'] - 500_000_000:
        reasons.append('world_trajectory_does_not_cover_mission')
    if gaps and max(gaps) > 0.5:
        reasons.append('world_trajectory_gap_over_0_5_s')
    if not hazard_inside_hull or outside_count:
        reasons.append('evaluation_outside_calibration_hull')
    if max(error for _, error in held_out) > 1.25:
        reasons.append('rigid_registration_residual_over_1_25_m')
    if conservative is None or conservative <= 0.0:
        reasons.append('uncertainty_adjusted_clearance_not_proven')
    if summary.get('status') != 'pass':
        reasons.append('operational_valid_analysis_not_passed')
    return {
        'status': 'pass' if not reasons else 'inconclusive',
        'reasons': reasons,
        'scope': 'offline_evaluation_only_no_operational_subscriptions_or_publications',
        'source_topic': topic,
        'source_identity_proven': topic == MODEL_TOPIC,
        'world_pose_count': len(poses),
        'world_pose_max_gap_s': max(gaps, default=None),
        'hazard_inside_calibration_hull': hazard_inside_hull,
        'outside_calibration_hull_pose_count': outside_count,
        'registration': {
            'waypoint_csv': str(waypoint_csv.resolve()),
            'waypoint_csv_sha256': hashlib.sha256(waypoint_csv.read_bytes()).hexdigest(),
            'control_point_count': len(unique),
            'world_to_map_yaw_rad': fit.yaw_rad,
            'world_to_map_translation_x_m': fit.translation_x_m,
            'world_to_map_translation_y_m': fit.translation_y_m,
            'maximum_fit_residual_m': max(error for _, error in residuals),
            'maximum_leave_one_out_residual_m': max(error for _, error in held_out),
            'conservative_empirical_margin_m': margin,
            'residuals_m': dict(residuals),
            'leave_one_out_residuals_m': dict(held_out),
        },
        'hazard_world_geometry': geometry,
        'robot_footprint_circumscribed_radius_m': robot_radius,
        'nominal_centerline_clearance_m': nominal,
        'nominal_body_clearance_m': None if nominal is None else nominal - robot_radius,
        'uncertainty_adjusted_body_clearance_m': conservative,
        'crosses_effective_hazard_centerline': metrics['crosses_effective_hazard'],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--analysis', type=Path, required=True)
    parser.add_argument('--recording', type=Path, required=True)
    parser.add_argument('--waypoints', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--allow-legacy-global-index', action='store_true')
    args = parser.parse_args()
    result = evaluate(
        args.analysis, args.recording, args.waypoints,
        allow_legacy=args.allow_legacy_global_index,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        parser.error(f'refusing to overwrite evaluation evidence: {args.output}')
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({key: result[key] for key in (
        'status', 'reasons', 'source_topic', 'nominal_centerline_clearance_m',
        'uncertainty_adjusted_body_clearance_m',
    )}, indent=2))
    raise SystemExit(0 if result['status'] == 'pass' else 1)


if __name__ == '__main__':
    main()
