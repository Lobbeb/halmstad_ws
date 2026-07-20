#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
from bisect import bisect_left
from pathlib import Path
from typing import Any, Callable


"""Summarize Baylands Results campaign runs.

Event rules used here are intentionally simple and configurable:
- stuck event: command target exists, but UAV position speed stays below the
  threshold for more than --stuck-duration seconds while target error remains.
- sharp turn: yaw-rate exceeds --sharp-turn-yaw-rate.
- oscillation: repeated sign changes in follow/cross-track error inside
  --oscillation-window seconds.
- divergence: follow error exceeds --divergence-threshold.
- target loss: estimate status stays unhealthy beyond --target-loss-timeout.
- bridge loss: visual bridge status stays unhealthy beyond --target-loss-timeout.
- stable follow: follow error stays below --stable-follow-error-threshold for
  at least --stable-follow-min-duration seconds.

The script only parses fields that exist in recorded ROS messages, status lines,
JSON payloads, detector CSVs, or campaign metadata. Missing topics/fields become NA.
"""


NA = "NA"

RUN_COLUMNS = [
    "condition",
    "run_id",
    "world",
    "waypoint",
    "nav2_goals",
    "duration_s",
    "recorded_duration_s",
    "analysed_duration_s",
    "warmup_s",
    "completion_status",
    "failure_reason",
    "valid_for_results",
    "missing_required_topics_count",
    "representative_run_candidate",
    "detector_backend",
    "model_path",
    "model_filename",
    "model_domain",
    "yolo_control_mode",
    "visual_follow_logic",
    "support_backend",
    "support_model_path",
    "record_profile",
    "visual_reacquire_assist_enable",
    "visual_reacquire_active_ratio",
    "visual_reacquire_count",
    "visual_reacquire_total_time_s",
    "visual_reacquire_recovered_count",
    "visual_reacquire_unrecovered_loss_count",
    "time_to_first_valid_detection_s",
    "time_to_first_stable_follow_s",
    "route_completion_ratio",
    "mean_follow_error_m",
    "median_follow_error_m",
    "p95_follow_error_m",
    "max_follow_error_m",
    "mean_3d_separation_m",
    "p95_3d_separation_m",
    "mean_yaw_error_rad",
    "p95_yaw_error_rad",
    "setpoint_hz",
    "setpoint_jitter_ms",
    "command_update_gap_mean_ms",
    "command_update_gap_p95_ms",
    "command_dropout_count",
    "detector_ok_ratio",
    "no_det_ratio",
    "stale_ratio",
    "detector_latency_mean_ms",
    "detector_latency_p50_ms",
    "detector_latency_p95_ms",
    "detection_confidence_mean",
    "detection_confidence_p50",
    "detection_confidence_p95",
    "detection_confidence_min",
    "target_loss_count",
    "target_loss_total_duration_s",
    "reacquisition_count",
    "estimate_fresh_ratio",
    "estimate_stale_ratio",
    "estimate_age_mean_ms",
    "estimate_age_p95_ms",
    "estimate_update_hz",
    "estimate_gap_p95_ms",
    "continuity_mean",
    "continuity_tracked_ratio",
    "visual_target_fresh_ratio",
    "follow_point_update_hz",
    "planned_target_update_hz",
    "bridge_ok_ratio",
    "bridge_loss_count",
    "bridge_loss_total_duration_s",
    "actuation_jitter_ms",
    "support_ok_ratio",
    "support_no_det_ratio",
    "support_no_input_ratio",
    "support_selected_dji1_ratio",
    "support_selected_dji2_ratio",
    "support_source_switch_count",
    "support_dji1_fresh_ratio",
    "support_dji2_fresh_ratio",
    "support_dji1_ok_ratio",
    "support_dji2_ok_ratio",
    "ugv_awareness_msg_count",
    "ugv_advisory_msg_count",
    "ugv_forwarded_detection_count",
    "stuck_event_count",
    "total_stuck_duration_s",
    "sharp_turn_count",
    "oscillation_count",
    "divergence_event_count",
    "max_no_command_gap_s",
    "max_no_estimate_gap_s",
    "omnet_rssi_mean",
    "omnet_rssi_p50",
    "omnet_snir_mean",
    "omnet_snir_p50",
    "omnet_per_mean",
    "omnet_per_p95",
    "omnet_radio_distance_mean",
    "omnet_radio_distance_p95",
    "omnet_delivery_ratio",
]

TEXT_COLUMNS = {
    "condition",
    "run_id",
    "world",
    "waypoint",
    "nav2_goals",
    "completion_status",
    "failure_reason",
    "detector_backend",
    "model_path",
    "model_filename",
    "model_domain",
    "yolo_control_mode",
    "visual_follow_logic",
    "support_backend",
    "support_model_path",
    "record_profile",
    "visual_reacquire_assist_enable",
    "representative_run_candidate",
}

AGG_COLUMNS = [
    "condition",
    "metric",
    "valid_run_count",
    "numeric_count",
    "mean",
    "median",
    "std",
    "min",
    "max",
    "p95",
    "completion_count",
    "failure_count",
]

BAD_STATES = {
    "",
    "NO_DET",
    "NO_INPUT",
    "STALE",
    "LOST",
    "INVALID",
    "INVALID_SUMMARY",
    "WAIT_CAMERA",
    "WAIT_POSE",
    "WAIT_START",
    "YOLO_DISABLED",
    "DECODE_FAIL",
}

HEALTHY_STATES = {
    "OK",
    "VALID",
    "TRACKED",
    "ACTIVE",
    "PREDICTED",
    "DEGRADED",
    "HOLD",
    "WAITING",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize Baylands Results campaign bags.")
    parser.add_argument("path", help="One run folder, condition folder, or full bags/results folder")
    parser.add_argument("--out", default="", help="Output directory for runs.csv and aggregate.csv")
    parser.add_argument("--warmup", type=float, default=15.0, help="Seconds to discard from the start of each bag")
    parser.add_argument("--stuck-speed-threshold", type=float, default=0.05, help="Speed below this is stuck-like")
    parser.add_argument("--stuck-duration", type=float, default=3.0, help="Minimum stuck segment duration")
    parser.add_argument("--divergence-threshold", type=float, default=15.0, help="Follow error threshold for divergence")
    parser.add_argument("--target-loss-timeout", type=float, default=10.0, help="Unhealthy status duration before target/bridge loss")
    parser.add_argument("--sharp-turn-yaw-rate", type=float, default=1.0, help="Yaw-rate threshold for sharp turn events")
    parser.add_argument("--oscillation-window", type=float, default=5.0, help="Window for repeated sign-change oscillations")
    parser.add_argument("--stable-follow-error-threshold", type=float, default=2.0, help="Follow error threshold for stable-follow timing")
    parser.add_argument("--stable-follow-min-duration", type=float, default=5.0, help="Minimum sustained duration for stable-follow timing")
    return parser.parse_args()


def safe_float(value: Any) -> float:
    if value is None:
        return math.nan
    text = str(value).strip()
    if not text or text.lower() in {"na", "nan", "none", "null", "unknown"}:
        return math.nan
    try:
        return float(text)
    except Exception:
        return math.nan


def is_finite(value: Any) -> bool:
    try:
        return math.isfinite(float(value))
    except Exception:
        return False


def finite_values(values: list[Any]) -> list[float]:
    return [float(v) for v in (safe_float(item) for item in values) if math.isfinite(v)]


def fmt(value: Any, *, text: bool = False) -> str:
    if value is None:
        return "" if text else NA
    if isinstance(value, str):
        return value if (text or value) else NA
    number = safe_float(value)
    if not math.isfinite(number):
        return NA
    if abs(number - round(number)) < 1e-9 and abs(number) < 1e12:
        return str(int(round(number)))
    return f"{number:.6g}"


def mean(values: list[Any]) -> float:
    data = finite_values(values)
    return statistics.fmean(data) if data else math.nan


def median(values: list[Any]) -> float:
    data = finite_values(values)
    return statistics.median(data) if data else math.nan


def percentile(values: list[Any], q: float) -> float:
    data = sorted(finite_values(values))
    if not data:
        return math.nan
    if len(data) == 1:
        return data[0]
    rank = max(0.0, min(1.0, q / 100.0)) * (len(data) - 1)
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return data[lo]
    frac = rank - lo
    return data[lo] * (1.0 - frac) + data[hi] * frac


def ratio(count: int, total: int) -> float:
    return float(count) / float(total) if total > 0 else math.nan


def parse_kv(line: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in str(line).split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key.strip()] = value.strip()
    return fields


def read_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as handle:
            data = json.load(handle)
        return data if isinstance(data, dict) else {}
    except Exception:
        return {}


def infer_domain(path: str) -> str:
    name = Path(str(path)).name.lower()
    if "baylands" in name:
        return "baylands"
    if "warehouse" in name:
        return "warehouse"
    return "unknown" if name else ""


def discover_runs(path: Path) -> list[Path]:
    path = path.expanduser().resolve()
    if path.name == "bag" and path.is_dir():
        return [path.parent]
    if (path / "bag").is_dir() or (path / "campaign_run.json").is_file():
        return [path]
    runs: set[Path] = set()
    for metadata in path.rglob("bag/metadata.yaml"):
        runs.add(metadata.parent.parent)
    for metadata in path.rglob("campaign_run.json"):
        runs.add(metadata.parent)
    return sorted(runs)


def default_out_dir(input_path: Path) -> Path:
    path = input_path.expanduser().resolve()
    if path.name == "summary":
        return path
    return path / "summary"


def detector_csv_stats(run_dir: Path) -> dict[str, Any]:
    csv_path = run_dir / "detector.csv"
    if not csv_path.is_file():
        return {}
    try:
        with csv_path.open("r", encoding="utf-8", newline="") as handle:
            rows = list(csv.DictReader(handle))
    except Exception:
        return {}
    latencies = [safe_float(row.get("camera_to_publish_latency_ms")) for row in rows]
    stale = [str(row.get("stale_detection", "")).lower() in ("1", "true", "yes") for row in rows]
    valid = [str(row.get("valid_detection", "")).lower() in ("1", "true", "yes") for row in rows]
    backends = [str(row.get("backend", "")).strip() for row in rows if str(row.get("backend", "")).strip()]
    total = len(rows)
    return {
        "detector_backend": backends[0] if backends else "",
        "detector_latency_mean_ms": mean(latencies),
        "detector_latency_p50_ms": percentile(latencies, 50.0),
        "detector_latency_p95_ms": percentile(latencies, 95.0),
        "stale_ratio": ratio(sum(1 for item in stale if item), total),
        "detector_ok_ratio": ratio(sum(1 for item in valid if item), total),
        "no_det_ratio": ratio(sum(1 for item in valid if not item), total),
    }


def yaw_from_quat(q: Any) -> float:
    x = safe_float(getattr(q, "x", 0.0))
    y = safe_float(getattr(q, "y", 0.0))
    z = safe_float(getattr(q, "z", 0.0))
    w = safe_float(getattr(q, "w", 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_pi(value: float) -> float:
    while value > math.pi:
        value -= 2.0 * math.pi
    while value < -math.pi:
        value += 2.0 * math.pi
    return value


def pose_from_msg(msg: Any) -> tuple[float, float, float, float] | None:
    try:
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        position = pose.position
        yaw = yaw_from_quat(pose.orientation)
        return (float(position.x), float(position.y), float(position.z), yaw)
    except Exception:
        return None


def joy_setpoint_from_msg(msg: Any) -> tuple[float, float, float, float] | None:
    try:
        axes = list(msg.axes)
        if len(axes) < 4:
            return None
        return (float(axes[0]), float(axes[1]), float(axes[2]), float(axes[3]))
    except Exception:
        return None


def string_data(msg: Any) -> str:
    return str(getattr(msg, "data", "") or "")


def nearest_sample(samples: list[tuple[float, tuple[float, float, float, float]]], t: float):
    if not samples:
        return None
    times = [item[0] for item in samples]
    idx = bisect_left(times, t)
    candidates = []
    if idx < len(samples):
        candidates.append(samples[idx])
    if idx > 0:
        candidates.append(samples[idx - 1])
    return min(candidates, key=lambda item: abs(item[0] - t)) if candidates else None


def sample_pose_error(
    pose_samples: list[tuple[float, tuple[float, float, float, float]]],
    target_samples: list[tuple[float, tuple[float, float, float, float]]],
    *,
    max_dt_s: float = 0.5,
) -> tuple[list[float], list[float], list[float]]:
    xy_errors: list[float] = []
    yaw_errors: list[float] = []
    sep3d: list[float] = []
    for t, pose in pose_samples:
        target_item = nearest_sample(target_samples, t)
        if target_item is None or abs(target_item[0] - t) > max_dt_s:
            continue
        target = target_item[1]
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        dz = target[2] - pose[2]
        xy_errors.append(math.hypot(dx, dy))
        sep3d.append(math.sqrt(dx * dx + dy * dy + dz * dz))
        yaw_errors.append(abs(wrap_pi(target[3] - pose[3])))
    return xy_errors, yaw_errors, sep3d


def sample_pose_error_timed(
    pose_samples: list[tuple[float, tuple[float, float, float, float]]],
    target_samples: list[tuple[float, tuple[float, float, float, float]]],
    *,
    max_dt_s: float = 0.5,
) -> tuple[list[tuple[float, float]], list[tuple[float, float]], list[tuple[float, float]]]:
    xy_errors: list[tuple[float, float]] = []
    yaw_errors: list[tuple[float, float]] = []
    sep3d: list[tuple[float, float]] = []
    for t, pose in pose_samples:
        target_item = nearest_sample(target_samples, t)
        if target_item is None or abs(target_item[0] - t) > max_dt_s:
            continue
        target = target_item[1]
        dx = target[0] - pose[0]
        dy = target[1] - pose[1]
        dz = target[2] - pose[2]
        xy_errors.append((t, math.hypot(dx, dy)))
        sep3d.append((t, math.sqrt(dx * dx + dy * dy + dz * dz)))
        yaw_errors.append((t, abs(wrap_pi(target[3] - pose[3]))))
    return xy_errors, yaw_errors, sep3d


def time_to_first_stable_follow(
    samples: list[tuple[float, float]],
    *,
    warmup_s: float,
    threshold_m: float,
    min_duration_s: float,
) -> float:
    if len(samples) < 2:
        return math.nan
    active_start: float | None = None
    last_t: float | None = None
    for t, value in sorted(samples):
        if not math.isfinite(value):
            continue
        if value <= threshold_m:
            if active_start is None or (last_t is not None and t - last_t > 1.0):
                active_start = t
            if active_start is not None and t - active_start >= min_duration_s:
                return max(0.0, active_start - warmup_s)
        else:
            active_start = None
        last_t = t
    return math.nan


def time_to_first_status(
    samples: list[tuple[float, dict[str, str]]],
    *,
    warmup_s: float,
    predicate: Callable[[dict[str, str]], bool],
) -> float:
    for t, fields in sorted(samples, key=lambda item: item[0]):
        if predicate(fields):
            return max(0.0, t - warmup_s)
    return math.nan


def interval_stats(times: list[float]) -> dict[str, float]:
    samples = sorted(float(t) for t in times if math.isfinite(float(t)))
    if len(samples) < 2:
        return {
            "hz": math.nan,
            "jitter_ms": math.nan,
            "gap_mean_ms": math.nan,
            "gap_p95_ms": math.nan,
            "dropout_count": math.nan,
            "max_gap_s": math.nan,
        }
    gaps = [b - a for a, b in zip(samples, samples[1:]) if b > a]
    if not gaps:
        return {
            "hz": math.nan,
            "jitter_ms": math.nan,
            "gap_mean_ms": math.nan,
            "gap_p95_ms": math.nan,
            "dropout_count": math.nan,
            "max_gap_s": math.nan,
        }
    span = samples[-1] - samples[0]
    hz = (len(samples) - 1) / span if span > 0 else math.nan
    med_gap = statistics.median(gaps)
    dropout_threshold = max(1.0, 3.0 * med_gap)
    return {
        "hz": hz,
        "jitter_ms": statistics.pstdev(gaps) * 1000.0 if len(gaps) > 1 else 0.0,
        "gap_mean_ms": mean(gaps) * 1000.0,
        "gap_p95_ms": percentile(gaps, 95.0) * 1000.0,
        "dropout_count": float(sum(1 for gap in gaps if gap > dropout_threshold)),
        "max_gap_s": max(gaps),
    }


def state_of(fields: dict[str, str]) -> str:
    return str(fields.get("state", "")).strip().upper()


def status_ratio(samples: list[tuple[float, dict[str, str]]], predicate: Callable[[str], bool]) -> float:
    states = [state_of(fields) for _, fields in samples]
    return ratio(sum(1 for state in states if predicate(state)), len(states))


def count_segments(
    samples: list[tuple[float, dict[str, str]]],
    predicate: Callable[[dict[str, str]], bool],
    *,
    min_duration_s: float,
) -> tuple[float, float]:
    if len(samples) < 2:
        return math.nan, math.nan
    count = 0
    total = 0.0
    start: float | None = None
    last_t = samples[0][0]
    for t, fields in samples:
        bad = predicate(fields)
        if bad and start is None:
            start = t
        elif not bad and start is not None:
            duration = max(0.0, last_t - start)
            if duration >= min_duration_s:
                count += 1
                total += duration
            start = None
        last_t = t
    if start is not None:
        duration = max(0.0, last_t - start)
        if duration >= min_duration_s:
            count += 1
            total += duration
    return float(count), total


def reacquisition_count(samples: list[tuple[float, dict[str, str]]]) -> float:
    if len(samples) < 2:
        return math.nan
    count = 0
    was_bad = False
    for _, fields in samples:
        healthy = state_of(fields) in HEALTHY_STATES
        if healthy and was_bad:
            count += 1
        was_bad = not healthy
    return float(count)


def stuck_events(
    pose_samples: list[tuple[float, tuple[float, float, float, float]]],
    target_samples: list[tuple[float, tuple[float, float, float, float]]],
    *,
    command_error_threshold_m: float,
    actual_speed_threshold_mps: float,
    min_duration_s: float,
) -> tuple[float, float]:
    if len(pose_samples) < 2 or not target_samples:
        return math.nan, math.nan
    event_count = 0
    total_duration = 0.0
    active_start: float | None = None
    active_end: float | None = None
    for (t0, p0), (t1, p1) in zip(pose_samples, pose_samples[1:]):
        dt = t1 - t0
        if dt <= 0.0 or dt > 2.0:
            continue
        target_item = nearest_sample(target_samples, t1)
        if target_item is None or abs(target_item[0] - t1) > 0.75:
            continue
        target = target_item[1]
        command_error = math.hypot(target[0] - p1[0], target[1] - p1[1])
        speed = math.hypot(p1[0] - p0[0], p1[1] - p0[1]) / dt
        stuck = command_error >= command_error_threshold_m and speed <= actual_speed_threshold_mps
        if stuck:
            if active_start is None:
                active_start = t0
            active_end = t1
        elif active_start is not None and active_end is not None:
            duration = active_end - active_start
            if duration >= min_duration_s:
                event_count += 1
                total_duration += duration
            active_start = None
            active_end = None
    if active_start is not None and active_end is not None:
        duration = active_end - active_start
        if duration >= min_duration_s:
            event_count += 1
            total_duration += duration
    return float(event_count), total_duration


def sharp_turn_count(samples: list[tuple[float, tuple[float, float, float, float]]], threshold_rad_s: float) -> float:
    if len(samples) < 2:
        return math.nan
    count = 0
    active = False
    for (t0, p0), (t1, p1) in zip(samples, samples[1:]):
        dt = t1 - t0
        if dt <= 0.0 or dt > 2.0:
            continue
        rate = abs(wrap_pi(p1[3] - p0[3])) / dt
        if rate >= threshold_rad_s and not active:
            count += 1
            active = True
        elif rate < threshold_rad_s * 0.5:
            active = False
    return float(count)


def oscillation_count(samples: list[tuple[float, float]], *, threshold: float = 0.05, window_s: float) -> float:
    filtered = [(t, v) for t, v in samples if math.isfinite(v) and abs(v) >= threshold]
    if len(filtered) < 4:
        return math.nan
    changes: list[float] = []
    last_sign = 1 if filtered[0][1] > 0 else -1
    for t, value in filtered[1:]:
        sign = 1 if value > 0 else -1
        if sign != last_sign:
            changes.append(t)
            last_sign = sign
    count = 0
    bucket: list[float] = []
    for t in changes:
        bucket = [item for item in bucket if t - item <= window_s]
        bucket.append(t)
        if len(bucket) >= 3:
            count += 1
            bucket = []
    return float(count)


def threshold_event_count(values: list[tuple[float, float]], threshold: float) -> float:
    if not values:
        return math.nan
    count = 0
    active = False
    for _, value in values:
        over = math.isfinite(value) and value >= threshold
        if over and not active:
            count += 1
            active = True
        elif not over:
            active = False
    return float(count)


def valid_detection_payload(line: str) -> tuple[bool | None, float]:
    try:
        payload = json.loads(line)
    except Exception:
        return None, math.nan
    if not isinstance(payload, dict):
        return None, math.nan
    valid = bool(payload.get("valid", False))
    return valid, safe_float(payload.get("conf"))


def first_present(*values: Any) -> Any:
    for value in values:
        if value not in ("", None, NA):
            return value
    return ""


def condition_key(condition: Any) -> str:
    text = str(condition or "").strip().upper()
    if text.startswith("C1"):
        return "C1"
    if text.startswith("C2"):
        return "C2"
    if text.startswith("C3"):
        return "C3"
    if text.startswith("C4"):
        return "C4"
    if text.startswith("C5"):
        return "C5"
    return text


def required_topic_groups(condition: Any) -> list[tuple[str, ...]]:
    common = [
        ("/dji0/pose",),
        ("/dji0/pose_cmd", "/dji0/pose_cmd/odom"),
        ("/a201_0000/amcl_pose_odom", "/a201_0000/platform/odom/filtered"),
    ]
    key = condition_key(condition)
    if key == "C1":
        return common + [
            ("/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw",),
        ]
    if key == "C2":
        return common + [
            ("/coord/leader_detection",),
            ("/coord/leader_detection_status",),
            ("/coord/leader_estimate",),
            ("/coord/leader_estimate_status",),
        ]
    if key == "C3":
        return common + [
            ("/coord/leader_detection",),
            ("/coord/leader_detection_status",),
            ("/coord/leader_estimate",),
            ("/coord/leader_estimate_status",),
            ("/coord/leader_visual_target_estimate_status",),
            ("/coord/leader_follow_point_status",),
            ("/coord/leader_planned_target_status",),
            ("/coord/leader_visual_actuation_bridge_status",),
        ]
    if key == "C4":
        return common + [
            ("/dji1/pose",),
            ("/dji2/pose",),
            ("/coord/support/dji1/leader_detection_status",),
            ("/coord/support/dji2/leader_detection_status",),
            ("/coord/dji0/leader_detection_status",),
            ("/coord/dji0/support_observation_summary",),
            ("/coord/ugv/leader_detection_status",),
            ("/coord/ugv/support_observation_summary",),
            ("/coord/ugv/support_awareness_status",),
            ("/coord/ugv/support_path_advisory",),
            ("/coord/support/camera_scan_status",),
        ]
    if key == "C5":
        return required_topic_groups("C2") + [
            ("/omnet/sim_time",),
            ("/omnet/rssi_dbm",),
            ("/omnet/snir_db",),
            ("/omnet/packet_error_rate",),
            ("/omnet/radio_distance",),
        ]
    return common


def missing_required_topics_count(condition: Any, topics: set[str]) -> int:
    missing = 0
    for group in required_topic_groups(condition):
        if not any(topic in topics for topic in group):
            missing += 1
    return missing


def valid_for_results(row: dict[str, Any], *, missing_topics: int) -> float:
    if missing_topics > 0:
        return 0.0
    recorded_duration = safe_float(row.get("recorded_duration_s"))
    analysed_duration = safe_float(row.get("analysed_duration_s"))
    expected_duration = safe_float(row.get("duration_s"))
    warmup = safe_float(row.get("warmup_s"))
    if not math.isfinite(recorded_duration) or recorded_duration <= 0.0:
        return 0.0
    if not math.isfinite(analysed_duration) or analysed_duration <= 0.0:
        return 0.0
    if math.isfinite(expected_duration) and math.isfinite(warmup):
        expected_analysed = max(0.0, expected_duration - warmup)
        minimum_usable = min(30.0, max(5.0, expected_analysed * 0.25))
        if analysed_duration < minimum_usable:
            return 0.0
    if row.get("completion_status") == "failed" and analysed_duration < 5.0:
        return 0.0
    return 1.0


def summarize_bag(run_dir: Path, args: argparse.Namespace) -> dict[str, str]:
    campaign = read_json(run_dir / "campaign_run.json")
    record_meta = read_json(run_dir / "metadata.json")
    row: dict[str, Any] = {column: NA for column in RUN_COLUMNS}

    model_path = first_present(campaign.get("model_path"), campaign.get("yolo_weights_path"), campaign.get("support_model_path"))
    support_model_path = first_present(campaign.get("support_model_path"), campaign.get("support_onnx_model_path"), campaign.get("support_yolo_weights_path"))
    row.update(
        {
            "condition": first_present(campaign.get("condition"), record_meta.get("tag", "").split("_")[0], run_dir.parent.name),
            "run_id": first_present(campaign.get("run_id"), run_dir.name),
            "world": first_present(campaign.get("world"), record_meta.get("world")),
            "waypoint": first_present(campaign.get("waypoint")),
            "nav2_goals": first_present(campaign.get("nav2_goals"), campaign.get("route")),
            "duration_s": first_present(campaign.get("duration_s")),
            "warmup_s": first_present(campaign.get("warmup_s"), args.warmup),
            "completion_status": first_present(campaign.get("completion_status"), "unknown"),
            "failure_reason": first_present(campaign.get("failure_reason")),
            "detector_backend": first_present(campaign.get("detector_backend")),
            "model_path": model_path,
            "model_filename": first_present(campaign.get("model_filename"), Path(str(model_path)).name if model_path else ""),
            "model_domain": first_present(campaign.get("model_domain"), campaign.get("detector_model_training_domain"), infer_domain(str(model_path))),
            "yolo_control_mode": first_present(campaign.get("yolo_control_mode")),
            "visual_follow_logic": first_present(campaign.get("visual_follow_logic")),
            "support_backend": first_present(campaign.get("support_backend"), campaign.get("support_detector_backend")),
            "support_model_path": support_model_path,
            "record_profile": first_present(campaign.get("record_profile"), record_meta.get("profile")),
            "visual_reacquire_assist_enable": first_present(campaign.get("visual_reacquire_assist_enable")),
        }
    )

    for key, value in detector_csv_stats(run_dir).items():
        if value not in ("", None):
            row[key] = value

    bag_dir = run_dir / "bag"
    if not bag_dir.is_dir():
        missing_topics = missing_required_topics_count(row.get("condition"), set())
        row["missing_required_topics_count"] = float(missing_topics)
        row["valid_for_results"] = 0.0
        return {column: fmt(row.get(column), text=column in TEXT_COLUMNS) for column in RUN_COLUMNS}

    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception:
        row["failure_reason"] = first_present(row.get("failure_reason"), "rosbag2_py_unavailable")
        missing_topics = missing_required_topics_count(row.get("condition"), set())
        row["missing_required_topics_count"] = float(missing_topics)
        row["valid_for_results"] = 0.0
        return {column: fmt(row.get(column), text=column in TEXT_COLUMNS) for column in RUN_COLUMNS}

    reader = rosbag2_py.SequentialReader()
    try:
        reader.open(
            rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
    except Exception as exc:
        row["failure_reason"] = first_present(row.get("failure_reason"), f"bag_open_failed:{exc}")
        missing_topics = missing_required_topics_count(row.get("condition"), set())
        row["missing_required_topics_count"] = float(missing_topics)
        row["valid_for_results"] = 0.0
        return {column: fmt(row.get(column), text=column in TEXT_COLUMNS) for column in RUN_COLUMNS}

    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    recorded_topics = set(type_map)
    missing_topics = missing_required_topics_count(row.get("condition"), recorded_topics)
    row["missing_required_topics_count"] = float(missing_topics)
    msg_type_cache: dict[str, Any] = {}

    start_ns: int | None = None
    raw_min_t = math.nan
    raw_max_t = math.nan
    kept_min_t = math.nan
    kept_max_t = math.nan
    times: dict[str, list[float]] = {}
    status: dict[str, list[tuple[float, dict[str, str]]]] = {}
    pose: dict[str, list[tuple[float, tuple[float, float, float, float]]]] = {}
    floats: dict[str, list[tuple[float, float]]] = {}
    leader_detection_conf: list[float] = []
    support_summaries: list[tuple[float, dict[str, Any]]] = []
    ugv_summaries: list[tuple[float, dict[str, Any]]] = []
    omnet_rssi: list[float] = []
    omnet_snir: list[float] = []
    omnet_per: list[float] = []
    omnet_radio_distance: list[float] = []
    leader_detection_valid_times: list[float] = []
    ugv_awareness_count = 0
    ugv_advisory_count = 0
    ugv_forwarded_detection_count = 0

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if start_ns is None:
            start_ns = int(timestamp_ns)
        rel_t = (int(timestamp_ns) - start_ns) * 1e-9
        raw_min_t = rel_t if not math.isfinite(raw_min_t) else min(raw_min_t, rel_t)
        raw_max_t = rel_t if not math.isfinite(raw_max_t) else max(raw_max_t, rel_t)
        if rel_t < args.warmup:
            continue
        kept_min_t = rel_t if not math.isfinite(kept_min_t) else min(kept_min_t, rel_t)
        kept_max_t = rel_t if not math.isfinite(kept_max_t) else max(kept_max_t, rel_t)
        times.setdefault(topic, []).append(rel_t)

        msg_type_name = type_map.get(topic)
        if not msg_type_name:
            continue
        try:
            if msg_type_name not in msg_type_cache:
                msg_type_cache[msg_type_name] = get_message(msg_type_name)
            msg = deserialize_message(data, msg_type_cache[msg_type_name])
        except Exception:
            continue

        if topic.endswith("_status") or topic in {"/coord/ugv/support_path_advisory", "/coord/support/camera_scan_status"}:
            status.setdefault(topic, []).append((rel_t, parse_kv(string_data(msg))))

        if topic in {
            "/dji0/pose",
            "/dji0/pose_cmd",
            "/dji0/pose_cmd/odom",
            "/dji1/pose",
            "/dji2/pose",
            "/dji1/pose_cmd",
            "/dji2/pose_cmd",
            "/coord/leader_estimate",
            "/a201_0000/amcl_pose_odom",
            "/a201_0000/platform/odom/filtered",
        }:
            item = pose_from_msg(msg)
            if item is not None:
                pose.setdefault(topic, []).append((rel_t, item))

        if topic.startswith("/dji0/follow/") or topic.startswith("/omnet/"):
            value = safe_float(getattr(msg, "data", None))
            if math.isfinite(value):
                floats.setdefault(topic, []).append((rel_t, value))

        if topic in {"/coord/leader_detection", "/coord/dji0/leader_detection"}:
            valid, conf = valid_detection_payload(string_data(msg))
            if valid:
                leader_detection_valid_times.append(rel_t)
            if valid and math.isfinite(conf):
                leader_detection_conf.append(conf)
        elif topic == "/coord/dji0/support_observation_summary":
            try:
                payload = json.loads(string_data(msg))
                if isinstance(payload, dict):
                    support_summaries.append((rel_t, payload))
            except Exception:
                pass
        elif topic == "/coord/ugv/support_observation_summary":
            try:
                payload = json.loads(string_data(msg))
                if isinstance(payload, dict):
                    ugv_summaries.append((rel_t, payload))
            except Exception:
                pass
        elif topic == "/coord/ugv/support_awareness_status":
            ugv_awareness_count += 1
        elif topic == "/coord/ugv/support_path_advisory":
            ugv_advisory_count += 1
        elif topic == "/coord/ugv/leader_detection":
            valid, _ = valid_detection_payload(string_data(msg))
            if valid:
                ugv_forwarded_detection_count += 1
        elif topic == "/omnet/rssi_dbm":
            omnet_rssi.append(safe_float(getattr(msg, "data", None)))
        elif topic == "/omnet/snir_db":
            omnet_snir.append(safe_float(getattr(msg, "data", None)))
        elif topic == "/omnet/packet_error_rate":
            omnet_per.append(safe_float(getattr(msg, "data", None)))
        elif topic == "/omnet/radio_distance":
            omnet_radio_distance.append(safe_float(getattr(msg, "data", None)))

    if math.isfinite(raw_min_t) and math.isfinite(raw_max_t):
        row["recorded_duration_s"] = max(0.0, raw_max_t - raw_min_t)
    if math.isfinite(kept_min_t) and math.isfinite(kept_max_t):
        row["analysed_duration_s"] = max(0.0, kept_max_t - kept_min_t)

    follow_error_samples = floats.get("/dji0/follow/error/xy_distance_m", [])
    yaw_error_samples = [(t, abs(v)) for t, v in floats.get("/dji0/follow/error/yaw_rad", [])]
    sep3d_samples = floats.get("/dji0/follow/actual/distance_3d_m", [])
    xy_errors = [v for _, v in follow_error_samples]
    yaw_errors = [v for _, v in yaw_error_samples]
    sep3d = [v for _, v in sep3d_samples]
    anchor_cross = floats.get("/dji0/follow/error/anchor_cross_m", [])

    dji0_pose = pose.get("/dji0/pose", [])
    command_pose = pose.get("/dji0/pose_cmd", []) or pose.get("/dji0/pose_cmd/odom", [])
    ugv_pose = pose.get("/a201_0000/amcl_pose_odom", []) or pose.get("/a201_0000/platform/odom/filtered", [])
    if not xy_errors:
        computed_xy_samples, computed_yaw_samples, computed_sep_samples = sample_pose_error_timed(dji0_pose, command_pose)
        follow_error_samples = computed_xy_samples
        xy_errors = [v for _, v in computed_xy_samples]
        if not yaw_errors:
            yaw_error_samples = computed_yaw_samples
            yaw_errors = [v for _, v in computed_yaw_samples]
        if not sep3d:
            sep3d_samples = computed_sep_samples
            sep3d = [v for _, v in computed_sep_samples]
    if not sep3d:
        _, _, computed_sep_samples = sample_pose_error_timed(dji0_pose, ugv_pose)
        sep3d_samples = computed_sep_samples
        sep3d = [v for _, v in computed_sep_samples]

    row["mean_follow_error_m"] = mean(xy_errors)
    row["median_follow_error_m"] = median(xy_errors)
    row["p95_follow_error_m"] = percentile(xy_errors, 95.0)
    row["max_follow_error_m"] = max(finite_values(xy_errors), default=math.nan)
    row["mean_3d_separation_m"] = mean(sep3d)
    row["p95_3d_separation_m"] = percentile(sep3d, 95.0)
    row["mean_yaw_error_rad"] = mean(yaw_errors)
    row["p95_yaw_error_rad"] = percentile(yaw_errors, 95.0)
    row["time_to_first_stable_follow_s"] = time_to_first_stable_follow(
        follow_error_samples,
        warmup_s=args.warmup,
        threshold_m=args.stable_follow_error_threshold,
        min_duration_s=args.stable_follow_min_duration,
    )

    command_times = (
        times.get("/dji0/psdk_ros2/flight_control_setpoint_ENUposition_yaw", [])
        or times.get("/dji0/pose_cmd", [])
        or times.get("/dji0/pose_cmd/odom", [])
    )
    command_stats = interval_stats(command_times)
    row["setpoint_hz"] = command_stats["hz"]
    row["setpoint_jitter_ms"] = command_stats["jitter_ms"]
    row["command_update_gap_mean_ms"] = command_stats["gap_mean_ms"]
    row["command_update_gap_p95_ms"] = command_stats["gap_p95_ms"]
    row["command_dropout_count"] = command_stats["dropout_count"]
    row["max_no_command_gap_s"] = command_stats["max_gap_s"]

    detection_statuses = status.get("/coord/leader_detection_status", []) or status.get("/coord/dji0/leader_detection_status", [])
    if detection_statuses:
        row["time_to_first_valid_detection_s"] = time_to_first_status(
            detection_statuses,
            warmup_s=args.warmup,
            predicate=lambda fields: state_of(fields) == "OK",
        )
        row["detector_ok_ratio"] = status_ratio(detection_statuses, lambda state: state == "OK")
        row["no_det_ratio"] = status_ratio(detection_statuses, lambda state: state == "NO_DET")
        if not is_finite(row.get("stale_ratio")):
            row["stale_ratio"] = status_ratio(detection_statuses, lambda state: state == "STALE")
        latencies = [safe_float(fields.get("latency_ms")) for _, fields in detection_statuses]
        if not is_finite(row.get("detector_latency_mean_ms")):
            row["detector_latency_mean_ms"] = mean(latencies)
            row["detector_latency_p50_ms"] = percentile(latencies, 50.0)
            row["detector_latency_p95_ms"] = percentile(latencies, 95.0)
        status_conf = [safe_float(fields.get("conf")) for _, fields in detection_statuses]
        leader_detection_conf.extend([value for value in status_conf if math.isfinite(value) and value >= 0.0])
    if not is_finite(row.get("time_to_first_valid_detection_s")) and leader_detection_valid_times:
        row["time_to_first_valid_detection_s"] = max(0.0, min(leader_detection_valid_times) - args.warmup)

    row["detection_confidence_mean"] = mean(leader_detection_conf)
    row["detection_confidence_p50"] = percentile(leader_detection_conf, 50.0)
    row["detection_confidence_p95"] = percentile(leader_detection_conf, 95.0)
    finite_conf = finite_values(leader_detection_conf)
    row["detection_confidence_min"] = min(finite_conf) if finite_conf else math.nan

    estimate_statuses = status.get("/coord/leader_estimate_status", [])
    if estimate_statuses:
        row["estimate_fresh_ratio"] = status_ratio(estimate_statuses, lambda state: state == "OK")
        row["estimate_stale_ratio"] = status_ratio(estimate_statuses, lambda state: state in {"NO_DET", "STALE", "LOST"})
        ages = [
            first_present(fields.get("estimate_age_ms"), fields.get("detector_age_ms"), fields.get("latency_ms"))
            for _, fields in estimate_statuses
        ]
        row["estimate_age_mean_ms"] = mean(ages)
        row["estimate_age_p95_ms"] = percentile(ages, 95.0)
        row["target_loss_count"], row["target_loss_total_duration_s"] = count_segments(
            estimate_statuses,
            lambda fields: state_of(fields) in {"NO_DET", "STALE", "LOST", "INVALID"},
            min_duration_s=args.target_loss_timeout,
        )
        row["reacquisition_count"] = reacquisition_count(estimate_statuses)

    estimate_stats = interval_stats(times.get("/coord/leader_estimate", []))
    row["estimate_update_hz"] = estimate_stats["hz"]
    row["estimate_gap_p95_ms"] = estimate_stats["gap_p95_ms"]
    row["max_no_estimate_gap_s"] = estimate_stats["max_gap_s"]

    reacquire_statuses = status.get("/coord/visual_reacquire_assist_status", [])
    if reacquire_statuses:
        row["visual_reacquire_active_ratio"] = status_ratio(
            reacquire_statuses,
            lambda state: state == "ACTIVE",
        )
        counts = [safe_float(fields.get("reacquisition_count")) for _, fields in reacquire_statuses]
        totals = [safe_float(fields.get("reacquisition_total_time_s")) for _, fields in reacquire_statuses]
        recovered = [safe_float(fields.get("recovered_to_visual_count")) for _, fields in reacquire_statuses]
        unrecovered = [safe_float(fields.get("unrecovered_loss_count")) for _, fields in reacquire_statuses]
        finite_counts = finite_values(counts)
        finite_totals = finite_values(totals)
        finite_recovered = finite_values(recovered)
        finite_unrecovered = finite_values(unrecovered)
        row["visual_reacquire_count"] = max(finite_counts) if finite_counts else math.nan
        row["visual_reacquire_total_time_s"] = max(finite_totals) if finite_totals else math.nan
        row["visual_reacquire_recovered_count"] = max(finite_recovered) if finite_recovered else math.nan
        row["visual_reacquire_unrecovered_loss_count"] = max(finite_unrecovered) if finite_unrecovered else math.nan

    visual_statuses = status.get("/coord/leader_visual_target_estimate_status", [])
    selected_filtered_statuses = status.get("/coord/leader_selected_target_filtered_status", [])
    continuity_samples = [safe_float(fields.get("continuity")) for _, fields in (visual_statuses or selected_filtered_statuses)]
    row["continuity_mean"] = mean(continuity_samples)
    if visual_statuses:
        row["continuity_tracked_ratio"] = status_ratio(visual_statuses, lambda state: state == "TRACKED")
        row["visual_target_fresh_ratio"] = status_ratio(visual_statuses, lambda state: state in {"TRACKED", "PREDICTED", "DEGRADED"})
    elif selected_filtered_statuses:
        row["continuity_tracked_ratio"] = status_ratio(selected_filtered_statuses, lambda state: state == "VALID")

    row["follow_point_update_hz"] = interval_stats(times.get("/coord/leader_follow_point", []))["hz"]
    row["planned_target_update_hz"] = interval_stats(times.get("/coord/leader_planned_target", []))["hz"]

    bridge_statuses = status.get("/coord/leader_visual_actuation_bridge_status", [])
    if bridge_statuses:
        row["bridge_ok_ratio"] = status_ratio(bridge_statuses, lambda state: state in {"ACTIVE", "PREDICTED", "DEGRADED", "HOLD", "OK"})
        row["bridge_loss_count"], row["bridge_loss_total_duration_s"] = count_segments(
            bridge_statuses,
            lambda fields: state_of(fields) in BAD_STATES,
            min_duration_s=args.target_loss_timeout,
        )
        bridge_dt = [safe_float(fields.get("dt_s")) for _, fields in bridge_statuses]
        row["actuation_jitter_ms"] = statistics.pstdev(finite_values(bridge_dt)) * 1000.0 if len(finite_values(bridge_dt)) > 1 else math.nan

    summaries = support_summaries or ugv_summaries
    if summaries:
        states = [str(payload.get("state", "")).upper() for _, payload in summaries]
        selected = [str(payload.get("selected_source", "")) for _, payload in summaries]
        row["support_ok_ratio"] = ratio(states.count("OK"), len(states))
        row["support_no_det_ratio"] = ratio(states.count("NO_DET"), len(states))
        row["support_no_input_ratio"] = ratio(states.count("NO_INPUT"), len(states))
        row["support_selected_dji1_ratio"] = ratio(selected.count("dji1"), len(selected))
        row["support_selected_dji2_ratio"] = ratio(selected.count("dji2"), len(selected))
        switches = 0
        last_source = ""
        dji_fresh: dict[str, list[bool]] = {"dji1": [], "dji2": []}
        dji_ok: dict[str, list[bool]] = {"dji1": [], "dji2": []}
        for _, payload in summaries:
            current = str(payload.get("selected_source", ""))
            if current not in {"", "none"}:
                if last_source and current != last_source:
                    switches += 1
                last_source = current
            sources = payload.get("sources", [])
            if isinstance(sources, list):
                for source in sources:
                    if not isinstance(source, dict):
                        continue
                    source_id = str(source.get("source_id", ""))
                    if source_id in dji_fresh:
                        dji_fresh[source_id].append(bool(source.get("fresh", False)))
                        dji_ok[source_id].append(str(source.get("state", "")).upper() == "OK")
        row["support_source_switch_count"] = float(switches)
        row["support_dji1_fresh_ratio"] = ratio(sum(dji_fresh["dji1"]), len(dji_fresh["dji1"]))
        row["support_dji2_fresh_ratio"] = ratio(sum(dji_fresh["dji2"]), len(dji_fresh["dji2"]))
        row["support_dji1_ok_ratio"] = ratio(sum(dji_ok["dji1"]), len(dji_ok["dji1"]))
        row["support_dji2_ok_ratio"] = ratio(sum(dji_ok["dji2"]), len(dji_ok["dji2"]))

    row["ugv_awareness_msg_count"] = float(ugv_awareness_count) if ugv_awareness_count else math.nan
    row["ugv_advisory_msg_count"] = float(ugv_advisory_count) if ugv_advisory_count else math.nan
    row["ugv_forwarded_detection_count"] = float(ugv_forwarded_detection_count) if ugv_forwarded_detection_count else math.nan

    target_samples = command_pose or ugv_pose
    row["stuck_event_count"], row["total_stuck_duration_s"] = stuck_events(
        dji0_pose,
        target_samples,
        command_error_threshold_m=max(0.75, args.divergence_threshold * 0.1),
        actual_speed_threshold_mps=args.stuck_speed_threshold,
        min_duration_s=args.stuck_duration,
    )
    row["sharp_turn_count"] = sharp_turn_count(dji0_pose or command_pose, args.sharp_turn_yaw_rate)
    row["oscillation_count"] = oscillation_count(anchor_cross, window_s=args.oscillation_window)
    row["divergence_event_count"] = threshold_event_count(follow_error_samples, args.divergence_threshold)

    row["omnet_rssi_mean"] = mean(omnet_rssi)
    row["omnet_rssi_p50"] = percentile(omnet_rssi, 50.0)
    row["omnet_snir_mean"] = mean(omnet_snir)
    row["omnet_snir_p50"] = percentile(omnet_snir, 50.0)
    row["omnet_per_mean"] = mean(omnet_per)
    row["omnet_per_p95"] = percentile(omnet_per, 95.0)
    row["omnet_radio_distance_mean"] = mean(omnet_radio_distance)
    row["omnet_radio_distance_p95"] = percentile(omnet_radio_distance, 95.0)
    row["omnet_delivery_ratio"] = 1.0 - mean(omnet_per) if finite_values(omnet_per) else math.nan
    row["valid_for_results"] = valid_for_results(row, missing_topics=missing_topics)

    return {column: fmt(row.get(column), text=column in TEXT_COLUMNS) for column in RUN_COLUMNS}


def aggregate_rows(rows: list[dict[str, str]]) -> list[dict[str, str]]:
    by_condition: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        by_condition.setdefault(row.get("condition", NA), []).append(row)
    aggregate: list[dict[str, str]] = []
    numeric_columns = [column for column in RUN_COLUMNS if column not in TEXT_COLUMNS]
    for condition, group in sorted(by_condition.items()):
        completion_count = sum(1 for row in group if row.get("completion_status") == "completed")
        failure_count = sum(
            1
            for row in group
            if row.get("completion_status") == "failed" or row.get("failure_reason") not in {"", NA}
        )
        for metric in numeric_columns:
            values = finite_values([row.get(metric) for row in group])
            aggregate.append(
                {
                    "condition": condition,
                    "metric": metric,
                    "valid_run_count": str(len(group)),
                    "numeric_count": str(len(values)),
                    "mean": fmt(mean(values)),
                    "median": fmt(median(values)),
                    "std": fmt(statistics.stdev(values) if len(values) > 1 else (0.0 if values else math.nan)),
                    "min": fmt(min(values) if values else math.nan),
                    "max": fmt(max(values) if values else math.nan),
                    "p95": fmt(percentile(values, 95.0)),
                    "completion_count": str(completion_count),
                    "failure_count": str(failure_count),
                }
            )
    return aggregate


def mark_representative_runs(rows: list[dict[str, str]]) -> None:
    by_condition: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        by_condition.setdefault(row.get("condition", NA), []).append(row)
    for group in by_condition.values():
        candidates = [
            row
            for row in group
            if safe_float(row.get("valid_for_results")) >= 0.5
            and math.isfinite(safe_float(row.get("mean_follow_error_m")))
        ]
        if len(candidates) < 2:
            continue
        median_error = median([row.get("mean_follow_error_m") for row in candidates])
        if not math.isfinite(median_error):
            continue
        selected = min(
            candidates,
            key=lambda row: (
                abs(safe_float(row.get("mean_follow_error_m")) - median_error),
                str(row.get("run_id", "")),
            ),
        )
        for row in group:
            row["representative_run_candidate"] = "1" if row is selected else "0"


def write_csv(path: Path, rows: list[dict[str, str]], columns: list[str]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=columns)
        writer.writeheader()
        for row in rows:
            writer.writerow({column: row.get(column, NA) for column in columns})


def write_summary_metadata(path: Path, args: argparse.Namespace, run_count: int) -> None:
    payload = {
        "schema": "lrs.results_summary.v1",
        "run_count": run_count,
        "warmup_s": args.warmup,
        "event_rules": {
            "stuck_event": "command target exists; UAV speed below stuck_speed_threshold for longer than stuck_duration",
            "sharp_turn": "yaw-rate exceeds sharp_turn_yaw_rate",
            "oscillation": "at least three follow/cross-track error sign changes inside oscillation_window",
            "divergence": "follow error exceeds divergence_threshold",
            "target_loss": "leader estimate status unhealthy beyond target_loss_timeout",
            "bridge_loss": "visual bridge status unhealthy beyond target_loss_timeout",
            "stable_follow": "follow error stays below stable_follow_error_threshold for at least stable_follow_min_duration",
            "representative_run_candidate": "per condition, valid run with mean_follow_error_m closest to the valid-run median",
            "valid_for_results": "false when required topic groups are missing, no usable recording duration exists, or an immediate failed run has less than 5 s analysed data",
        },
        "thresholds": {
            "stuck_speed_threshold": args.stuck_speed_threshold,
            "stuck_duration": args.stuck_duration,
            "divergence_threshold": args.divergence_threshold,
            "target_loss_timeout": args.target_loss_timeout,
            "sharp_turn_yaw_rate": args.sharp_turn_yaw_rate,
            "oscillation_window": args.oscillation_window,
            "stable_follow_error_threshold": args.stable_follow_error_threshold,
            "stable_follow_min_duration": args.stable_follow_min_duration,
        },
    }
    with path.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")


def main() -> None:
    args = parse_args()
    input_path = Path(args.path)
    run_dirs = discover_runs(input_path)
    rows = [summarize_bag(run_dir, args) for run_dir in run_dirs]
    mark_representative_runs(rows)
    aggregate = aggregate_rows(rows)

    out_dir = Path(args.out).expanduser().resolve() if args.out else default_out_dir(input_path)
    write_csv(out_dir / "runs.csv", rows, RUN_COLUMNS)
    write_csv(out_dir / "aggregate.csv", aggregate, AGG_COLUMNS)
    write_summary_metadata(out_dir / "summary_metadata.json", args, len(rows))
    print(f"runs_csv={out_dir / 'runs.csv'}")
    print(f"aggregate_csv={out_dir / 'aggregate.csv'}")
    print(f"summary_metadata={out_dir / 'summary_metadata.json'}")


if __name__ == "__main__":
    main()
