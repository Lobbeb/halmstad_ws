#!/usr/bin/env python3
"""Plot OMNeT/LoRa metrics from ROS 2 bags.

This script intentionally does not make XY trajectory plots. Use
plot_trajectory_paths.py for trajectory figures.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import re
from dataclasses import dataclass
from pathlib import Path
from statistics import fmean
from typing import Any


@dataclass
class PoseSample:
    t: float
    x: float
    y: float
    z: float


@dataclass
class RunData:
    route: str
    rep: str
    lora_mode: str
    bag_dir: Path
    ugv: list[PoseSample]
    uav: list[PoseSample]
    estimate: list[PoseSample]
    metrics: dict[str, list[tuple[float, float]]]
    distances: list[tuple[float, float, float, float]]
    metrics_csv: Path | None = None


ROS_METRIC_TOPICS = {
    "rssi_dbm": "/omnet/rssi_dbm",
    "snir_db": "/omnet/snir_db",
    "per": "/omnet/packet_error_rate",
    "pdr": "/omnet/packet_delivery_ratio",
    "latency_s": "/omnet/latency_s",
    "jitter_s": "/omnet/jitter_s",
    "radio_distance_m": "/omnet/radio_distance",
}
METRIC_NAMES = ["link_distance_m", *ROS_METRIC_TOPICS.keys()]
CSV_TO_METRIC = {
    "link_distance_m": "link_distance_m",
    "rssi_dbm": "rssi_dbm",
    "snir_db": "snir_db",
    "packet_error_rate": "per",
    "packet_delivery_ratio": "pdr",
    "radio_distance_m": "radio_distance_m",
    "latency_s": "latency_s",
    "jitter_s": "jitter_s",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot separated OMNeT network metric figures from ROS 2 bags."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--run-dir", help="Run directory containing bag/.")
    source.add_argument("--bag", help="ROS 2 bag directory.")
    source.add_argument("--results-dir", help="Results directory containing repXX/RNN_route/bag folders.")
    parser.add_argument("--metrics-csv", help="Offline OMNeT CSV from run_omnet_bag_replay.sh.")
    parser.add_argument(
        "--offline-omnet",
        action="store_true",
        help="With --results-dir, discover offline_omnet/*/network_metrics.csv files and plot those.",
    )
    parser.add_argument("--ugv-topic", default="/a201_0000/ground_truth/odom")
    parser.add_argument("--uav-topic", default="/dji0/pose")
    parser.add_argument("--estimate-topic", default="/coord/leader_estimate")
    parser.add_argument("--warmup", type=float, default=0.0)
    parser.add_argument("--out", help="Output file/stem, or output directory for --results-dir.")
    parser.add_argument("--pdf", action="store_true", help="Also write PDF.")
    parser.add_argument("--dpi", type=int, default=220)
    parser.add_argument("--width", type=float, default=6.6)
    parser.add_argument("--height", type=float, default=3.6)
    parser.add_argument("--font-size", type=float, default=11.0)
    parser.add_argument("--nearest-max-gap-s", type=float, default=1.0)
    parser.add_argument("--overview-only", action="store_true", help="For --results-dir, skip per-run plots.")
    parser.add_argument(
        "--reps",
        help="Comma-separated repetitions to include with --results-dir, e.g. rep01 or rep01,rep03.",
    )
    parser.add_argument(
        "--lora-mode",
        choices=("auto", "simplex", "duplex", "rep-map"),
        default="auto",
        help="Label mode. rep-map means rep01/rep03=simplex and rep02=duplex.",
    )
    return parser.parse_args()


def normalize_rep_label(value: str) -> str:
    value = str(value).strip().lower()
    if not value:
        return value
    if value.isdigit():
        return f"rep{int(value):02d}"
    match = re.fullmatch(r"rep0*(\d+)", value)
    if match:
        return f"rep{int(match.group(1)):02d}"
    return value


def selected_reps(value: str | None) -> set[str]:
    if not value:
        return set()
    return {normalize_rep_label(part) for part in value.split(",") if part.strip()}


def configure_plot_style(args: argparse.Namespace) -> None:
    try:
        import matplotlib as mpl
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    mpl.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["DejaVu Serif", "Times New Roman", "Times"],
            "font.size": args.font_size,
            "axes.labelsize": args.font_size,
            "axes.titlesize": args.font_size,
            "xtick.labelsize": args.font_size,
            "ytick.labelsize": args.font_size,
            "legend.fontsize": args.font_size - 1,
            "axes.linewidth": 0.8,
            "grid.linewidth": 0.45,
            "lines.linewidth": 2.0,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "savefig.bbox": "tight",
        }
    )


def route_label_from_run_dir(run_dir: Path) -> str:
    selected_match = re.match(r"^C\d+_Route[A-Z]_(.+?)_r\d+__", run_dir.name)
    if selected_match:
        return selected_match.group(1).replace("_", " ")
    return re.sub(r"^R\d+_", "", run_dir.name).replace("_", " ")


def rep_label_from_run_dir(run_dir: Path) -> str:
    selected_match = re.match(r"^(C\d+)_Route[A-Z]_.+?_(r\d+)__", run_dir.name)
    if selected_match:
        return f"{selected_match.group(1)}_{selected_match.group(2)}"
    for parent in run_dir.parents:
        if re.fullmatch(r"rep\d+", parent.name):
            return parent.name
    return run_dir.parent.name


def infer_lora_mode(rep: str, bag_dir: Path, requested: str) -> str:
    rep_l = rep.lower()
    if requested in {"simplex", "duplex"}:
        return requested
    if requested == "rep-map":
        if rep_l == "rep02":
            return "duplex"
        if rep_l in {"rep01", "rep03"}:
            return "simplex"
        return "unknown"

    path_l = str(bag_dir).lower()
    if "duplex" in path_l:
        return "duplex"
    if "simplex" in path_l:
        return "simplex"
    if rep_l == "rep02":
        return "duplex"
    if rep_l in {"rep01", "rep03"}:
        return "simplex"
    return "unknown"


def infer_lora_mode_from_csv(csv_path: Path, requested: str) -> str:
    if requested in {"simplex", "duplex"}:
        return requested
    name = csv_path.parent.name.lower()
    if "duplex" in name:
        return "duplex"
    if "lora" in name:
        return "simplex"
    return "unknown"


def resolve_single_bag(args: argparse.Namespace) -> tuple[str, str, Path]:
    if args.bag:
        bag_dir = Path(args.bag).expanduser().resolve()
        return route_label_from_run_dir(bag_dir.parent), rep_label_from_run_dir(bag_dir.parent), bag_dir
    run_dir = Path(args.run_dir).expanduser().resolve()
    return route_label_from_run_dir(run_dir), rep_label_from_run_dir(run_dir), run_dir / "bag"


def discover_result_bags(results_dir: Path) -> list[tuple[str, str, Path]]:
    runs: list[tuple[str, str, Path]] = []
    for bag_dir in sorted(results_dir.glob("rep*/R*/bag")):
        if not bag_dir.is_dir():
            continue
        if not (bag_dir / "metadata.yaml").is_file() and not list(bag_dir.glob("*.mcap")):
            continue
        run_dir = bag_dir.parent
        if ".replaced_" in run_dir.name:
            continue
        runs.append((route_label_from_run_dir(run_dir), rep_label_from_run_dir(run_dir), bag_dir))
    return runs


def discover_offline_metric_runs(results_dir: Path) -> list[tuple[str, str, Path, Path]]:
    runs: list[tuple[str, str, Path, Path]] = []
    for csv_path in sorted(results_dir.glob("**/offline_omnet/*/network_metrics.csv")):
        if not csv_path.is_file():
            continue
        run_dir = csv_path.parents[2]
        if ".replaced_" in run_dir.name:
            continue
        bag_dir = run_dir / "bag"
        if not bag_dir.is_dir():
            continue
        if not (bag_dir / "metadata.yaml").is_file() and not list(bag_dir.glob("*.mcap")):
            continue
        runs.append((route_label_from_run_dir(run_dir), rep_label_from_run_dir(run_dir), bag_dir, csv_path))
    return runs


def pose_from_msg(msg: Any) -> PoseSample | None:
    try:
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        p = pose.position
        return PoseSample(0.0, float(p.x), float(p.y), float(p.z))
    except Exception:
        return None


def read_bag(
    route: str,
    rep: str,
    lora_mode: str,
    bag_dir: Path,
    args: argparse.Namespace,
    *,
    read_ros_metrics: bool = True,
) -> RunData:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:
        raise SystemExit(
            "ROS bag reading requires a sourced ROS 2 environment "
            f"(rosbag2_py/rclpy unavailable): {exc}"
        )

    if not bag_dir.is_dir():
        raise SystemExit(f"Bag directory does not exist: {bag_dir}")

    ugv_topic_candidates = [
        args.ugv_topic,
        "/a201_0000/ground_truth/odom",
        "/a201_0000/platform/odom/filtered",
        "/a201_0000/amcl_pose_odom",
        "/a201_0000/platform/odom",
    ]
    topic_names = {args.uav_topic, args.estimate_topic, *ugv_topic_candidates}
    if read_ros_metrics:
        topic_names.update(ROS_METRIC_TOPICS.values())
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    actual_ugv_topic = next((topic for topic in ugv_topic_candidates if topic in type_map), args.ugv_topic)
    topic_names.add(actual_ugv_topic)
    required_topics = {actual_ugv_topic, args.uav_topic, args.estimate_topic}
    if read_ros_metrics:
        required_topics.update(ROS_METRIC_TOPICS.values())
    missing = sorted(topic for topic in required_topics if topic not in type_map)
    if missing:
        print(f"[plot_network_metrics] Warning: {bag_dir} missing: {', '.join(missing)}")

    msg_type_cache: dict[str, Any] = {}
    ugv: list[PoseSample] = []
    uav: list[PoseSample] = []
    estimate: list[PoseSample] = []
    metrics: dict[str, list[tuple[float, float]]] = {name: [] for name in METRIC_NAMES}
    start_ns: int | None = None

    metric_topic_to_name = {topic: name for name, topic in ROS_METRIC_TOPICS.items()}
    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        timestamp_ns = int(timestamp_ns)
        if start_ns is None:
            start_ns = timestamp_ns
        rel_t = (timestamp_ns - start_ns) * 1e-9
        if rel_t < args.warmup or topic not in topic_names:
            continue

        msg_type_name = type_map.get(topic)
        if not msg_type_name:
            continue
        try:
            if msg_type_name not in msg_type_cache:
                msg_type_cache[msg_type_name] = get_message(msg_type_name)
            msg = deserialize_message(data, msg_type_cache[msg_type_name])
        except Exception:
            continue

        if topic in (actual_ugv_topic, args.uav_topic, args.estimate_topic):
            pose = pose_from_msg(msg)
            if pose is None:
                continue
            pose.t = rel_t
            if topic == actual_ugv_topic:
                ugv.append(pose)
            elif topic == args.uav_topic:
                uav.append(pose)
            else:
                estimate.append(pose)
            continue

        metric_name = metric_topic_to_name.get(topic)
        if metric_name and hasattr(msg, "data"):
            value = float(msg.data)
            if math.isfinite(value):
                if metric_name in {"latency_s", "jitter_s"}:
                    value *= 1000.0
                metrics[metric_name].append((rel_t, value))

    distances = compute_distances(ugv, uav, args.nearest_max_gap_s)
    return RunData(route, rep, lora_mode, bag_dir, ugv, uav, estimate, metrics, distances)


def read_metrics_csv(csv_path: Path) -> dict[str, list[tuple[float, float]]]:
    metrics: dict[str, list[tuple[float, float]]] = {name: [] for name in METRIC_NAMES}
    with csv_path.open("r", encoding="utf-8", newline="") as handle:
        reader = csv.DictReader(handle)
        for row in reader:
            try:
                t = float(row.get("sim_time_s") or row.get("wall_time_s") or "nan")
            except ValueError:
                continue
            if not math.isfinite(t):
                continue
            for csv_name, metric_name in CSV_TO_METRIC.items():
                try:
                    value = float(row.get(csv_name, "nan"))
                except ValueError:
                    continue
                if not math.isfinite(value):
                    continue
                if metric_name in {"latency_s", "jitter_s"}:
                    value *= 1000.0
                metrics[metric_name].append((t, value))
    return metrics


def nearest_pose(samples: list[PoseSample], t: float, max_gap_s: float) -> PoseSample | None:
    if not samples:
        return None
    times = [sample.t for sample in samples]
    idx = bisect.bisect_left(times, t)
    candidates = []
    if idx < len(samples):
        candidates.append(samples[idx])
    if idx > 0:
        candidates.append(samples[idx - 1])
    best = min(candidates, key=lambda sample: abs(sample.t - t), default=None)
    if best is None or abs(best.t - t) > max_gap_s:
        return None
    return best


def compute_distances(
    ugv: list[PoseSample],
    uav: list[PoseSample],
    max_gap_s: float,
) -> list[tuple[float, float, float, float]]:
    distances: list[tuple[float, float, float, float]] = []
    for uav_sample in uav:
        ugv_sample = nearest_pose(ugv, uav_sample.t, max_gap_s)
        if ugv_sample is None:
            continue
        dx = uav_sample.x - ugv_sample.x
        dy = uav_sample.y - ugv_sample.y
        dz = uav_sample.z - ugv_sample.z
        xy = math.hypot(dx, dy)
        d3 = math.sqrt(dx * dx + dy * dy + dz * dz)
        distances.append((uav_sample.t, d3, xy, dz))
    return distances


def metric_values(data: RunData, name: str) -> list[float]:
    return [value for _t, value in data.metrics.get(name, []) if math.isfinite(value)]


def mean_or_nan(values: list[float]) -> float:
    return fmean(values) if values else float("nan")


def out_stem_for_run(data: RunData, out_root: Path) -> Path:
    route = re.sub(r"[^A-Za-z0-9_.-]+", "_", data.route).strip("_") or "route"
    return out_root / f"{data.rep}_{data.lora_mode}_{route}_network_metrics"


def single_out_stem(args: argparse.Namespace, data: RunData) -> Path:
    if args.out:
        out = Path(args.out).expanduser().resolve()
        if out.suffix.lower() in {".png", ".pdf", ".csv"}:
            return out.with_suffix("")
        return out
    return data.bag_dir.parent / "plots" / "network_metrics"


def save_fig(fig: Any, stem: Path, args: argparse.Namespace) -> None:
    stem.parent.mkdir(parents=True, exist_ok=True)
    png = stem.with_suffix(".png")
    fig.tight_layout()
    fig.savefig(png, dpi=args.dpi)
    print(f"png={png}")
    if args.pdf:
        pdf = stem.with_suffix(".pdf")
        fig.savefig(pdf)
        print(f"pdf={pdf}")


def save_metric_fig(fig: Any, stem: Path, suffix: str, args: argparse.Namespace) -> None:
    save_fig(fig, stem.with_name(f"{stem.name}_{suffix}"), args)


def expand_flat_ylim(ax: Any, values: list[float], *, floor_zero: bool = False) -> None:
    finite = [v for v in values if math.isfinite(v)]
    if not finite:
        return
    v_min = min(finite)
    v_max = max(finite)
    if floor_zero:
        v_min = min(0.0, v_min)
    if abs(v_max - v_min) < 1e-9:
        pad = max(1.0, abs(v_max) * 0.05)
    else:
        pad = (v_max - v_min) * 0.08
    ax.set_ylim(v_min - pad, v_max + pad)


def title_prefix(data: RunData) -> str:
    return f"{data.rep} {data.route} ({data.lora_mode})"


def plot_time_series(
    data: RunData,
    stem: Path,
    args: argparse.Namespace,
    metric_name: str,
    suffix: str,
    title: str,
    ylabel: str,
    color: str,
    *,
    scale: float = 1.0,
    floor_zero: bool = False,
) -> bool:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    series = data.metrics.get(metric_name, [])
    if not series:
        print(f"skip={stem.with_name(f'{stem.name}_{suffix}.png')} reason=no_{metric_name}_samples")
        return False

    fig, ax = plt.subplots(figsize=(args.width, args.height))
    x = [t for t, _v in series]
    y = [v * scale for _t, v in series]
    ax.plot(x, y, color=color, marker="o", markersize=2.6, markeredgewidth=0.0, label=title)
    ax.set_title(f"{title_prefix(data)}: {title}")
    ax.set_xlabel("time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.28)
    expand_flat_ylim(ax, y, floor_zero=floor_zero)
    ax.legend(frameon=False, loc="best")
    save_metric_fig(fig, stem, suffix, args)
    plt.close(fig)
    return True


def plot_distance(data: RunData, stem: Path, args: argparse.Namespace) -> bool:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    if (
        not data.distances
        and not data.metrics.get("radio_distance_m")
        and not data.metrics.get("link_distance_m")
    ):
        print(f"skip={stem.with_name(f'{stem.name}_distance.png')} reason=no_distance_samples")
        return False

    fig, ax = plt.subplots(figsize=(args.width, args.height))
    y_values: list[float] = []
    if data.distances:
        t = [row[0] for row in data.distances]
        d3 = [row[1] for row in data.distances]
        xy = [row[2] for row in data.distances]
        dz = [row[3] for row in data.distances]
        ax.plot(t, d3, color="#4C78A8", label="true 3D distance")
        ax.plot(t, xy, color="#72B7B2", linestyle="--", label="horizontal distance")
        ax.plot(t, dz, color="#E45756", linestyle=":", label="height delta")
        y_values.extend(d3)
        y_values.extend(xy)
        y_values.extend(dz)
    link = data.metrics.get("link_distance_m", [])
    if link:
        y = [v for _t, v in link]
        ax.plot([t for t, _v in link], y, color="#9D755D", linestyle="-.", alpha=0.9, label="OMNeT link distance")
        y_values.extend(y)
    radio = data.metrics.get("radio_distance_m", [])
    if radio:
        y = [v for _t, v in radio]
        ax.plot([t for t, _v in radio], y, color="#F58518", marker="o", markersize=2.4, alpha=0.9, label="radio distance")
        y_values.extend(y)
    ax.set_title(f"{title_prefix(data)}: distance")
    ax.set_xlabel("time (s)")
    ax.set_ylabel("distance (m)")
    ax.grid(True, alpha=0.28)
    expand_flat_ylim(ax, y_values, floor_zero=True)
    ax.legend(frameon=False, loc="best")
    save_metric_fig(fig, stem, "distance", args)
    plt.close(fig)
    return True


def metric_vs_distance_points(data: RunData, metric_name: str, args: argparse.Namespace) -> tuple[list[float], list[float]]:
    dist_t = [row[0] for row in data.distances]
    dist_d = [row[1] for row in data.distances]
    xs: list[float] = []
    ys: list[float] = []
    for metric_t, value in data.metrics.get(metric_name, []):
        nearest_idx = bisect.bisect_left(dist_t, metric_t)
        candidates = []
        if nearest_idx < len(dist_t):
            candidates.append(nearest_idx)
        if nearest_idx > 0:
            candidates.append(nearest_idx - 1)
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(dist_t[i] - metric_t))
        if abs(dist_t[best] - metric_t) <= args.nearest_max_gap_s:
            xs.append(dist_d[best])
            ys.append(value)
    return xs, ys


def plot_metric_vs_distance(
    data: RunData,
    stem: Path,
    args: argparse.Namespace,
    metric_name: str,
    suffix: str,
    title: str,
    ylabel: str,
    color: str,
    *,
    scale: float = 1.0,
    floor_zero: bool = False,
) -> bool:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    if not data.distances or not data.metrics.get(metric_name):
        print(f"skip={stem.with_name(f'{stem.name}_{suffix}.png')} reason=no_distance_or_{metric_name}_samples")
        return False
    x, y_raw = metric_vs_distance_points(data, metric_name, args)
    y = [v * scale for v in y_raw]
    if not x:
        print(f"skip={stem.with_name(f'{stem.name}_{suffix}.png')} reason=no_nearest_distance_pairs")
        return False

    fig, ax = plt.subplots(figsize=(args.width, args.height))
    ax.scatter(x, y, s=22, alpha=0.72, color=color, edgecolors="none", label=title)
    ax.set_title(f"{title_prefix(data)}: {title} vs distance")
    ax.set_xlabel("true UAV-UGV distance (m)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.28)
    expand_flat_ylim(ax, y, floor_zero=floor_zero)
    ax.legend(frameon=False, loc="best")
    save_metric_fig(fig, stem, suffix, args)
    plt.close(fig)
    return True


def plot_run(data: RunData, stem: Path, args: argparse.Namespace) -> None:
    plot_distance(data, stem, args)
    plot_time_series(data, stem, args, "link_distance_m", "link_distance", "OMNeT link distance", "distance (m)", "#9D755D", floor_zero=True)
    plot_time_series(data, stem, args, "radio_distance_m", "radio_distance", "radio distance", "distance (m)", "#F58518", floor_zero=True)
    plot_time_series(data, stem, args, "rssi_dbm", "rssi", "RSSI", "RSSI (dBm)", "#4C78A8")
    plot_time_series(data, stem, args, "snir_db", "snir", "SNIR", "SNIR (dB)", "#54A24B")
    plot_time_series(data, stem, args, "per", "per", "PER", "PER (%)", "#E45756", scale=100.0, floor_zero=True)
    plot_time_series(data, stem, args, "pdr", "pdr", "PDR", "PDR (%)", "#54A24B", scale=100.0, floor_zero=True)
    plot_time_series(data, stem, args, "latency_s", "latency", "latency", "latency (ms)", "#B279A2", floor_zero=True)
    plot_time_series(data, stem, args, "jitter_s", "jitter", "jitter", "jitter (ms)", "#FF9DA6", floor_zero=True)
    plot_metric_vs_distance(data, stem, args, "rssi_dbm", "rssi_vs_distance", "RSSI", "RSSI (dBm)", "#4C78A8")
    plot_metric_vs_distance(data, stem, args, "per", "per_vs_distance", "PER", "PER (%)", "#E45756", scale=100.0, floor_zero=True)
    plot_metric_vs_distance(data, stem, args, "radio_distance_m", "radio_vs_true_distance", "radio distance", "radio distance (m)", "#F58518", floor_zero=True)


def write_summary_csv(data_runs: list[RunData], csv_path: Path) -> None:
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    with csv_path.open("w", encoding="utf-8", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(
            [
                "rep",
                "route",
                "lora_mode",
                "samples_distance",
                "true_distance_m",
                "horizontal_distance_m",
                "height_delta_m",
                "radio_distance_m",
                "link_distance_m",
                "rssi_dbm",
                "snir_db",
                "latency_ms",
                "jitter_ms",
                "per",
                "pdr",
            ]
        )
        for data in data_runs:
            writer.writerow(
                [
                    data.rep,
                    data.route,
                    data.lora_mode,
                    len(data.distances),
                    mean_or_nan([row[1] for row in data.distances]),
                    mean_or_nan([row[2] for row in data.distances]),
                    mean_or_nan([row[3] for row in data.distances]),
                    mean_or_nan(metric_values(data, "radio_distance_m")),
                    mean_or_nan(metric_values(data, "link_distance_m")),
                    mean_or_nan(metric_values(data, "rssi_dbm")),
                    mean_or_nan(metric_values(data, "snir_db")),
                    mean_or_nan(metric_values(data, "latency_s")),
                    mean_or_nan(metric_values(data, "jitter_s")),
                    mean_or_nan(metric_values(data, "per")),
                    mean_or_nan(metric_values(data, "pdr")),
                ]
            )
    print(f"csv={csv_path}")


def plot_overview_metric_vs_distance(
    data_runs: list[RunData],
    stem: Path,
    args: argparse.Namespace,
    metric_name: str,
    suffix: str,
    title: str,
    ylabel: str,
    *,
    scale: float = 1.0,
    floor_zero: bool = False,
) -> None:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    fig, ax = plt.subplots(figsize=(args.width, args.height))
    colors = ["#4C78A8", "#F58518", "#54A24B", "#B279A2", "#E45756", "#72B7B2", "#FF9DA6", "#9D755D"]
    markers = {"simplex": "o", "duplex": "s", "unknown": "^"}
    y_all: list[float] = []

    for idx, data in enumerate(data_runs):
        if not data.distances or not data.metrics.get(metric_name):
            continue
        label = f"{data.rep} {data.lora_mode} {data.route}"
        color = colors[idx % len(colors)]
        x, y_raw = metric_vs_distance_points(data, metric_name, args)
        y = [v * scale for v in y_raw]
        if not x:
            continue
        y_all.extend(y)
        ax.scatter(
            x,
            y,
            s=18,
            alpha=0.62,
            color=color,
            marker=markers.get(data.lora_mode, "^"),
            edgecolors="none",
            label=label,
        )

    ax.set_title(f"{title} vs distance")
    ax.set_xlabel("true UAV-UGV distance (m)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.28)
    expand_flat_ylim(ax, y_all, floor_zero=floor_zero)
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(frameon=False, loc="best", ncol=1)
    save_metric_fig(fig, stem, suffix, args)
    plt.close(fig)


def plot_overviews(data_runs: list[RunData], stem: Path, args: argparse.Namespace) -> None:
    plot_overview_metric_vs_distance(data_runs, stem, args, "rssi_dbm", "rssi_vs_distance", "RSSI", "RSSI (dBm)")
    plot_overview_metric_vs_distance(data_runs, stem, args, "per", "per_vs_distance", "PER", "PER (%)", scale=100.0, floor_zero=True)
    plot_overview_metric_vs_distance(data_runs, stem, args, "link_distance_m", "link_vs_true_distance", "OMNeT link distance", "link distance (m)", floor_zero=True)
    plot_overview_metric_vs_distance(data_runs, stem, args, "radio_distance_m", "radio_vs_true_distance", "radio distance", "radio distance (m)", floor_zero=True)
    plot_overview_metric_vs_distance(data_runs, stem, args, "latency_s", "latency_vs_distance", "latency", "latency (ms)", floor_zero=True)


def main() -> None:
    args = parse_args()
    if args.results_dir:
        results_dir = Path(args.results_dir).expanduser().resolve()
        out_root = Path(args.out).expanduser().resolve() if args.out else results_dir / "plots" / "network"
        out_root.mkdir(parents=True, exist_ok=True)

        if args.offline_omnet:
            offline_runs = discover_offline_metric_runs(results_dir)
            rep_filter = selected_reps(args.reps)
            if rep_filter:
                offline_runs = [
                    run for run in offline_runs if normalize_rep_label(run[1]) in rep_filter
                ]
            if not offline_runs:
                raise SystemExit(f"No offline_omnet/*/network_metrics.csv files found under: {results_dir}")
            data_runs = []
            for route, rep, bag_dir, metrics_csv in offline_runs:
                mode = infer_lora_mode_from_csv(metrics_csv, args.lora_mode)
                data = read_bag(route, rep, mode, bag_dir, args, read_ros_metrics=False)
                data.metrics = read_metrics_csv(metrics_csv)
                data.metrics_csv = metrics_csv
                data_runs.append(data)
            write_summary_csv(data_runs, out_root / "network_metrics_summary.csv")
            if not args.overview_only:
                for data in data_runs:
                    plot_run(data, out_stem_for_run(data, out_root), args)
            plot_overviews(data_runs, out_root / "network_metrics_overview", args)
            return

        runs = discover_result_bags(results_dir)
        rep_filter = selected_reps(args.reps)
        if rep_filter:
            runs = [run for run in runs if normalize_rep_label(run[1]) in rep_filter]
        if not runs:
            raise SystemExit(f"No repXX/RNN_route/bag directories found under: {results_dir}")
        data_runs = [
            read_bag(route, rep, infer_lora_mode(rep, bag_dir, args.lora_mode), bag_dir, args)
            for route, rep, bag_dir in runs
        ]
        write_summary_csv(data_runs, out_root / "network_metrics_summary.csv")
        if not args.overview_only:
            for data in data_runs:
                plot_run(data, out_stem_for_run(data, out_root), args)
        plot_overviews(data_runs, out_root / "network_metrics_overview", args)
        return

    route, rep, bag_dir = resolve_single_bag(args)
    data = read_bag(route, rep, infer_lora_mode(rep, bag_dir, args.lora_mode), bag_dir, args)
    if args.metrics_csv:
        metrics_csv = Path(args.metrics_csv).expanduser().resolve()
        data.metrics = read_metrics_csv(metrics_csv)
    plot_run(data, single_out_stem(args, data), args)


if __name__ == "__main__":
    main()
