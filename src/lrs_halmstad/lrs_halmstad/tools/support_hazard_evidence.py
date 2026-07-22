#!/usr/bin/env python3
"""Bounded live and rosbag evidence checks for the typed support-hazard chain."""

from __future__ import annotations

import argparse
from collections import deque
import csv
from dataclasses import dataclass
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time
from typing import Any, Iterable

from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message, serialize_message


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


def array_signature(message: AerialHazardArray) -> bytes:
    return bytes(serialize_message(message))


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
    max_age_s: float = 1.0


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
        dji0_signatures = {
            array_signature(sample.message) for sample in self.samples[DJI0_TOPIC]
        }
        ugv_samples = list(self.samples[UGV_TOPIC])
        if not dji0_signatures or not ugv_samples:
            return None
        return all(
            array_signature(sample.message) in dji0_signatures
            for sample in ugv_samples
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
        if not typed_flow:
            failures.append('typed_flow_incomplete')
        if forwarding_preserved is not True:
            failures.append('dji0_to_ugv_forwarding_not_preserved')
        if covariance_preserved is not True:
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
    return parser


def main(args=None) -> None:
    parser = build_parser()
    parsed, ros_args = parser.parse_known_args(sys.argv[1:] if args is None else args)
    try:
        status = _run_live(parsed, ros_args) if parsed.mode == 'live' else _run_bag(parsed)
    except (FileExistsError, FileNotFoundError, ValueError) as exc:
        parser.error(str(exc))
    raise SystemExit(status)


if __name__ == '__main__':
    main()
