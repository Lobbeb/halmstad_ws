#!/usr/bin/env python3
"""Plot OMNeT/LoRa metrics and trajectory paths from ROS 2 bags."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
import re
import sys
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
    yaw: float | None = None


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
RADIO_DERIVED_DISTANCE_METRIC = "radio_derived_distance_m"
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

THESIS_ROUTE_BY_NAME = {
    "rotundan": "A",
    "road_to_west": "B",
    "road to west": "B",
    "parkinglot_west": "C",
    "parkinglot west": "C",
    "road_to_spawn": "D",
    "road to spawn": "D",
    "spawn": "E",
    "road_to_east": "F",
    "road to east": "F",
    "parkinglot_east": "G",
    "parkinglot east": "G",
    "road_to_strip": "H",
    "road to strip": "H",
    "strip": "I",
}

C2_REP_LABELS = {
    "rep01": "Simplex",
    "rep02": "Duplex",
    "rep03": "Distance Sweep",
}

# MasterThesis/classicthesis-config.tex sets the one-column text block to 312 pt.
SINGLE_FIGURE_WIDTH_IN = 312.0 / 72.27
SINGLE_FIGURE_HEIGHT_IN = 3.3
TALL_FIGURE_HEIGHT_IN = 6.6

THREE_D_PATH_DEFAULTS = {
    "marker_size": (("--marker-size",), 7.0),
    "view_azim": (("--view-azim",), 45.0),
    "view_elev": (("--view-elev",), 22.5),
    "axis_padding_fraction": (("--axis-padding-fraction",), 0.07),
    "labelpad": (("--labelpad",), 2.0),
    "z_labelpad": (("--z-labelpad",), 0.0),
    "legend_borderaxespad": (("--legend-borderaxespad",), 1.75),
    "figure_right": (("--figure-right",), 0.95),
    "figure_left": (("--figure-left",), 0.15),
    "figure_top": (("--figure-top",), 1.0),
    "figure_bottom": (("--figure-bottom",), 0.0),
}

TWO_D_PATH_DEFAULTS = {
    "marker_size": (("--marker-size",), 7.0),
    "axis_padding_fraction": (("--axis-padding-fraction",), 0.05),
    "labelpad": (("--labelpad",), 2.0),
    "legend_borderaxespad": (("--legend-borderaxespad",), 0.0),
    "legend_y": (("--legend-y",), 1.1),
}


class DefaultsHelpFormatter(argparse.HelpFormatter):
    def _get_help_string(self, action: argparse.Action) -> str:
        help_text = action.help or ""
        if action.default is argparse.SUPPRESS:
            return help_text
        if not action.option_strings:
            return help_text
        if action.default is None and help_text != "Value.":
            return help_text
        if "%(default)" not in help_text and "default" not in help_text.lower():
            suffix = "(default: %(default)s)"
            return f"{help_text} {suffix}" if help_text else suffix
        return help_text


def option_was_provided(argv: list[str], option_names: tuple[str, ...]) -> bool:
    return any(arg == option or arg.startswith(f"{option}=") for arg in argv for option in option_names)


def apply_dimension_defaults(args: argparse.Namespace, argv: list[str]) -> None:

    for dest, (options, value) in (THREE_D_PATH_DEFAULTS.items() if args.dimension == "3d" else TWO_D_PATH_DEFAULTS.items()):
        if not option_was_provided(argv, options):
            setattr(args, dest, value)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot OMNeT network metrics and trajectory path figures from ROS 2 bags.",
        epilog=(
            "When --dimension 3d is used, omitted layout defaults become: "
            "--marker-size 7, --view-azim 45, --view-elev 22.5, "
            "--axis-padding-fraction 0.025, --labelpad 2, --z-labelpad 0, "
            "--legend-borderaxespad 1.75, --figure-left 0.15, "
            "--figure-right 0.95, --figure-top 1.0, --figure-bottom 0.0."
        ),
        formatter_class=DefaultsHelpFormatter,
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
    parser.add_argument("--width", type=float, default=SINGLE_FIGURE_WIDTH_IN)
    parser.add_argument("--height", type=float, default=SINGLE_FIGURE_HEIGHT_IN)
    parser.add_argument("--z-max", type=float)
    parser.add_argument("--z-min", type=float)
    parser.add_argument("--font-size", type=float, default=11.0)
    parser.add_argument("--usetex", action="store_true", help="Render plot text with LaTeX/newtx fonts. Requires latex on PATH.")
    parser.add_argument("--labelpad", type=float, default=2.0)
    parser.add_argument("--z-labelpad", type=float, default=0.0)
    parser.add_argument("--legend-x", type=float, default=0.5)
    parser.add_argument("--legend-y", type=float, default=1.08)
    parser.add_argument("--legend-ncol", type=int, default=0, help="0 means automatic.")
    parser.add_argument("--legend-handlelength", type=float, default=1.0)
    parser.add_argument("--legend-columnspacing", type=float, default=0.8)
    parser.add_argument("--legend-borderaxespad", type=float, default=0.2)
    parser.add_argument("--figure-left", type=float)
    parser.add_argument("--figure-right", type=float)
    parser.add_argument("--figure-bottom", type=float)
    parser.add_argument("--figure-top", type=float)
    parser.add_argument("--figure-wspace", type=float)
    parser.add_argument("--figure-hspace", type=float)
    parser.add_argument("--no-tight-layout", action="store_true")
    parser.add_argument("--nearest-max-gap-s", type=float, default=1.0)
    parser.add_argument(
        "--plottype",
        "--plot-type",
        dest="plot_type",
        choices=("path", "network", "both"),
        default="network",
        help="Which figure family to generate. Default: network.",
    )
    parser.add_argument(
        "--dimension",
        choices=("2d", "3d"),
        default="2d",
        help="Trajectory plot dimension. Network figures remain 2D. Default: 2d.",
    )
    parser.add_argument(
        "--trajectory-plane",
        choices=("xy", "xz", "yz"),
        default="xy",
        help="2D trajectory plane used when --dimension 2d. Default: xy.",
    )
    parser.add_argument("--axis-padding-fraction", type=float, default=0.04)
    parser.add_argument("--trajectory-linewidth", type=float, default=1.4)
    parser.add_argument("--ugv-linewidth", type=float)
    parser.add_argument("--uav-linewidth", type=float)
    parser.add_argument("--estimate-linewidth", type=float)
    parser.add_argument("--ugv-linestyle", default="-")
    parser.add_argument("--uav-linestyle", default="--")
    parser.add_argument("--estimate-linestyle", default=":")
    parser.add_argument("--ugv-marker", default="o")
    parser.add_argument("--uav-marker", default="^")
    parser.add_argument("--estimate-marker", default="s")
    parser.add_argument("--marker-count", type=int, default=8)
    parser.add_argument("--marker-size", type=float, default=4.0)
    parser.add_argument(
        "--heading-markers",
        choices=("none", "ugv", "uav", "both"),
        default="none",
        help="Use rotated triangle markers from pose yaw in 2D XY path plots.",
    )
    parser.add_argument("--heading-marker-size", type=float, default=5.0)
    parser.add_argument(
        "--timestamp-links",
        choices=("none", "ugv", "estimate", "both"),
        default="both",
        help="In 3D path plots, draw dashed links from UAV marker timestamps to matching UGV and/or estimate samples.",
    )
    parser.add_argument("--timestamp-link-linewidth", type=float, default=0.6)
    parser.add_argument("--timestamp-link-alpha", type=float, default=0.55)
    parser.add_argument("--timestamp-link-linestyle", default="--")
    parser.add_argument("--path-alpha", type=float, default=0.95)
    parser.add_argument("--view-elev", type=float, default=25.0, help="3D view elevation.")
    parser.add_argument("--view-azim", type=float, default=-60.0, help="3D view azimuth.")
    parser.add_argument(
        "--figures",
        choices=("combined", "separate", "both"),
        default="combined",
        help="Per-run figure set to write when --per-run-plots is used. Default: combined.",
    )
    parser.add_argument(
        "--per-run-plots",
        action="store_true",
        help="With --results-dir, also write per-run figures. Default is campaign overview only.",
    )
    parser.add_argument(
        "--stack-routes",
        action="store_true",
        help="With --results-dir and path plots, write side-by-side UGV/UAV stacked route overviews.",
    )
    parser.add_argument(
        "--rep-average-plots",
        action="store_true",
        help="With --results-dir, write one averaged combined time-series figure per repetition.",
    )
    parser.add_argument(
        "--average-bin-s",
        type=float,
        default=5.0,
        help="Time bin size for --rep-average-plots. Default: 5 s.",
    )
    parser.add_argument("--overview-only", action="store_true", help="Deprecated alias for the default campaign-only behavior.")
    parser.add_argument("--summary-only", action="store_true", help="Write summary CSV and extremes table without generating figures.")
    parser.add_argument(
        "--overview-set",
        choices=("thesis", "all", "both"),
        default="thesis",
        help="Campaign overview figure set. thesis writes the compact signal/loss figure. Default: thesis.",
    )
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
    parser.add_argument(
        "--networks",
        help="Comma-separated offline OMNeT network folder names to include, e.g. lora-duplex.",
    )
    parser.add_argument(
        "--rank-limit",
        type=int,
        default=5,
        help="Rows per best/worst metric table in network_metrics_extremes.md. Default: 5.",
    )
    for action in parser._actions:
        if action.help is None or action.help == "":
            action.help = "Value."
    argv = sys.argv[1:]
    args = parser.parse_args(argv)
    apply_dimension_defaults(args, argv)
    return args


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


def selected_networks(value: str | None) -> set[str]:
    if not value:
        return set()
    return {part.strip().lower() for part in value.split(",") if part.strip()}


def configure_plot_style(args: argparse.Namespace) -> None:
    try:
        import matplotlib as mpl
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    mpl.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Nimbus Roman", "Liberation Serif", "Times New Roman", "Times", "DejaVu Serif"],
            "text.usetex": args.usetex,
            "text.latex.preamble": r"\usepackage[T1]{fontenc}\usepackage{newtxtext}\usepackage{newtxmath}" if args.usetex else "",
            "font.size": args.font_size,
            "axes.labelsize": args.font_size,
            "axes.titlesize": args.font_size,
            "xtick.labelsize": args.font_size,
            "ytick.labelsize": args.font_size,
            "legend.fontsize": args.font_size - 1,
            "axes.labelpad": args.labelpad,
            "axes.linewidth": 0.8,
            "grid.linewidth": 0.45,
            "lines.linewidth": 2.0,
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )


def single_figsize(args: argparse.Namespace) -> tuple[float, float]:
    return (args.width, args.height)


def tall_figsize(args: argparse.Namespace, rows: int) -> tuple[float, float]:
    if rows <= 1:
        return single_figsize(args)
    return (args.width, max(args.height * rows, TALL_FIGURE_HEIGHT_IN))


def per_axis_label(args: argparse.Namespace) -> str:
    return r"PER (\%)" if args.usetex else "PER (%)"


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
        yaw = yaw_from_quat(pose.orientation) if hasattr(pose, "orientation") else None
        return PoseSample(0.0, float(p.x), float(p.y), float(p.z), yaw)
    except Exception:
        return None


def yaw_from_quat(q: Any) -> float:
    x = float(getattr(q, "x", 0.0))
    y = float(getattr(q, "y", 0.0))
    z = float(getattr(q, "z", 0.0))
    w = float(getattr(q, "w", 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


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
    required_topics = {actual_ugv_topic, args.uav_topic}
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


def compute_estimate_errors(
    ugv: list[PoseSample],
    estimate: list[PoseSample],
    max_gap_s: float,
) -> list[tuple[float, float]]:
    errors: list[tuple[float, float]] = []
    for estimate_sample in estimate:
        ugv_sample = nearest_pose(ugv, estimate_sample.t, max_gap_s)
        if ugv_sample is None:
            continue
        dx = estimate_sample.x - ugv_sample.x
        dy = estimate_sample.y - ugv_sample.y
        dz = estimate_sample.z - ugv_sample.z
        errors.append((estimate_sample.t, math.sqrt(dx * dx + dy * dy + dz * dz)))
    return errors


def compute_radio_distance_errors(data: RunData, max_gap_s: float) -> list[tuple[float, float]]:
    distance_times = [row[0] for row in data.distances]
    errors: list[tuple[float, float]] = []
    for metric_t, radio_distance in radio_derived_distance_series(data):
        nearest_idx = bisect.bisect_left(distance_times, metric_t)
        candidates = []
        if nearest_idx < len(distance_times):
            candidates.append(nearest_idx)
        if nearest_idx > 0:
            candidates.append(nearest_idx - 1)
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(distance_times[i] - metric_t))
        if abs(distance_times[best] - metric_t) <= max_gap_s:
            errors.append((metric_t, abs(radio_distance - data.distances[best][1])))
    return errors


def radio_derived_distance_metric(data: RunData) -> str:
    """Choose the plotted d_r source for the active communication mode."""
    if "duplex" in data.lora_mode.lower():
        return "link_distance_m"
    return "radio_distance_m"


def radio_derived_distance_series(data: RunData) -> list[tuple[float, float]]:
    return data.metrics.get(radio_derived_distance_metric(data), [])


def metric_series(data: RunData, name: str) -> list[tuple[float, float]]:
    if name == RADIO_DERIVED_DISTANCE_METRIC:
        return radio_derived_distance_series(data)
    return data.metrics.get(name, [])


def metric_values(data: RunData, name: str) -> list[float]:
    return [value for _t, value in metric_series(data, name) if math.isfinite(value)]


def mean_or_nan(values: list[float]) -> float:
    return fmean(values) if values else float("nan")


def thesis_route_letter_from_name(route: str) -> str | None:
    key = route.strip().lower().replace("-", "_")
    return THESIS_ROUTE_BY_NAME.get(key) or THESIS_ROUTE_BY_NAME.get(key.replace("_", " "))


def display_condition_label(data: RunData) -> str:
    return C2_REP_LABELS.get(data.rep.lower(), data.lora_mode)


def out_stem_for_run(data: RunData, out_root: Path) -> Path:
    route = re.sub(r"[^A-Za-z0-9_.-]+", "_", data.route).strip("_") or "route"
    return out_root / data.rep / f"{data.rep}_{data.lora_mode}_{route}_network_metrics"


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
    if not args.no_tight_layout:
        fig.tight_layout()
    adjust_kwargs = {
        key: value
        for key, value in {
            "left": args.figure_left,
            "right": args.figure_right,
            "bottom": args.figure_bottom,
            "top": args.figure_top,
            "wspace": args.figure_wspace,
            "hspace": args.figure_hspace,
        }.items()
        if value is not None
    }
    if adjust_kwargs:
        fig.subplots_adjust(**adjust_kwargs)
    fig.savefig(png, dpi=args.dpi)
    print(png, flush=True)
    if args.pdf:
        pdf = stem.with_suffix(".pdf")
        fig.savefig(pdf)
        print(pdf, flush=True)


def save_metric_fig(fig: Any, stem: Path, suffix: str, args: argparse.Namespace) -> None:
    save_fig(fig, stem.with_name(f"{stem.name}_{suffix}"), args)


def legend_outside(ax: Any, handles: list[Any] | None = None, labels: list[str] | None = None, *, ncol: int = 2) -> None:
    if handles is None or labels is None:
        handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, frameon=False, loc="upper center", bbox_to_anchor=(0.5, 1.25), ncol=ncol)


def legend_right(ax: Any) -> None:
    handles, labels = ax.get_legend_handles_labels()
    if handles:
        ax.legend(handles, labels, frameon=False, loc="center left", bbox_to_anchor=(1.02, 0.5), ncol=1)


def legend_ncol(args: argparse.Namespace, handles: list[Any]) -> int:
    if args.legend_ncol > 0:
        return args.legend_ncol
    return max(1, len(handles))


def route_legend_handles(data: RunData, handles: list[Any], labels: list[str]) -> tuple[list[Any], list[str]]:
    try:
        from matplotlib.lines import Line2D
    except Exception:
        return handles, labels
    route_label = f"Route {route_code(data)}:"
    route_handle = Line2D([], [], color="none", linestyle="", marker="", label=route_label)
    return [route_handle, *handles], [route_label, *labels]


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
    lower = max(0.0, v_min - pad) if floor_zero else v_min - pad
    ax.set_ylim(lower, v_max + pad)


def expand_per_ylim(ax: Any, values: list[float]) -> None:
    """Leave visual clearance below zero while retaining nonnegative PER ticks."""
    expand_flat_ylim(ax, values, floor_zero=True)
    upper = ax.get_ylim()[1]
    nonnegative_ticks = [tick for tick in ax.get_yticks() if 0.0 <= tick <= upper]
    ax.set_ylim(-0.5, upper)
    ax.set_yticks(nonnegative_ticks)


def title_prefix(data: RunData) -> str:
    code = route_code(data)
    route = f"Route {code}: {data.route}" if re.fullmatch(r"[A-I]", code) else data.route
    return f"{route} ({display_condition_label(data)})"


def route_code(data: RunData) -> str:
    run_name = data.bag_dir.parent.name
    match = re.match(r"^R0*([1-9])(?:_|$)", run_name)
    if match:
        return chr(ord("A") + int(match.group(1)) - 1)
    letter = thesis_route_letter_from_name(data.route)
    if letter:
        return letter
    return re.sub(r"[^A-Za-z0-9]+", "_", data.route).strip("_") or "route"


def campaign_code(data: RunData) -> str:
    for part in data.bag_dir.parts:
        if re.fullmatch(r"C\d+", part):
            return part
    match = re.match(r"^(C\d+)_", data.bag_dir.parent.name)
    if match:
        return match.group(1)
    return data.rep.split("_", 1)[0] if data.rep.lower().startswith("c") else "campaign"


def compact_route_label(data_runs: list[RunData]) -> str:
    codes = sorted({route_code(data) for data in data_runs})
    if not codes:
        return "routes"
    if len(codes) == 1:
        return codes[0]
    if all(re.fullmatch(r"[A-I]", code) for code in codes):
        values = [ord(code) for code in codes]
        if values == list(range(values[0], values[-1] + 1)):
            return f"{codes[0]}-{codes[-1]}"
        return ", ".join(codes)
    return f"{codes[0]}-{codes[-1]}"


def output_roots(args: argparse.Namespace, results_dir: Path) -> tuple[Path, Path]:
    if args.out:
        network_root = Path(args.out).expanduser().resolve()
    else:
        network_root = results_dir / "plots" / "network"
    path_root = network_root.parent / "path" if network_root.name == "network" else network_root / "path"
    network_root.mkdir(parents=True, exist_ok=True)
    path_root.mkdir(parents=True, exist_ok=True)
    return network_root, path_root


def out_stem_for_path_run(data: RunData, out_root: Path) -> Path:
    route = re.sub(r"[^A-Za-z0-9_.-]+", "_", data.route).strip("_") or "route"
    return out_root / data.rep / f"{data.rep}_{data.lora_mode}_{route}_trajectory"


def path_dimension_root(out_root: Path, args: argparse.Namespace) -> Path:
    return out_root / "3d" if args.dimension == "3d" else out_root


def single_path_out_stem(args: argparse.Namespace, data: RunData) -> Path:
    if args.out:
        out = Path(args.out).expanduser().resolve()
        if out.suffix.lower() in {".png", ".pdf"}:
            return out.with_suffix("")
        return path_dimension_root(out.with_name(f"{out.name}_trajectory"), args)
    return path_dimension_root(data.bag_dir.parent / "plots" / "trajectory", args)


def project_pose(sample: PoseSample, plane: str) -> tuple[float, float]:
    if plane == "xz":
        return sample.x, sample.z
    if plane == "yz":
        return sample.y, sample.z
    return sample.x, sample.y


def marker_indices_by_distance(samples: list[PoseSample], args: argparse.Namespace) -> list[int]:
    count = max(0, args.marker_count)
    if not samples or count <= 0:
        return []
    if len(samples) <= count:
        return list(range(len(samples)))
    distances = [0.0]
    for prev, cur in zip(samples, samples[1:]):
        distances.append(distances[-1] + math.sqrt((cur.x - prev.x) ** 2 + (cur.y - prev.y) ** 2 + (cur.z - prev.z) ** 2))
    total = distances[-1]
    if total <= 0.0:
        step = (len(samples) - 1) / max(count - 1, 1)
        return sorted({round(idx * step) for idx in range(count)})
    indices = set()
    seg = 0
    for idx in range(count):
        target = total * idx / max(count - 1, 1)
        while seg + 1 < len(distances) and distances[seg + 1] < target:
            seg += 1
        candidates = [seg]
        if seg + 1 < len(distances):
            candidates.append(seg + 1)
        indices.add(min(candidates, key=lambda item: abs(distances[item] - target)))
    return sorted(indices)


def path_plot_kwargs(args: argparse.Namespace, linestyle: str, linewidth: float, marker: str, alpha: float | None = None) -> dict[str, Any]:
    kwargs: dict[str, Any] = {
        "linestyle": linestyle,
        "linewidth": linewidth,
        "alpha": args.path_alpha if alpha is None else alpha,
    }
    if marker:
        kwargs.update(
            {
                "marker": marker,
                "markevery": None,
                "markersize": args.marker_size,
                "markeredgewidth": 0.0,
            }
        )
    return kwargs


def heading_triangle_marker(yaw: float) -> Any:
    from matplotlib.path import Path as MplPath

    base_vertices = [(1.0, 0.0), (-0.55, 0.45), (-0.55, -0.45), (1.0, 0.0)]
    c = math.cos(yaw)
    s = math.sin(yaw)
    vertices = [(x * c - y * s, x * s + y * c) for x, y in base_vertices]
    return MplPath(vertices, [MplPath.MOVETO, MplPath.LINETO, MplPath.LINETO, MplPath.CLOSEPOLY])


def use_heading_markers(series_name: str, args: argparse.Namespace) -> bool:
    if args.dimension != "2d" or args.trajectory_plane != "xy":
        return False
    if args.heading_markers in {"ugv", "both"} and series_name == "ugv":
        return True
    if args.heading_markers in {"uav", "both"} and series_name == "uav":
        return True
    return False


def plot_heading_markers(
    ax: Any,
    samples: list[PoseSample],
    indices: list[int],
    *,
    color: str,
    size: float,
    alpha: float,
    zorder: float,
) -> None:
    marker_area = size * size
    for idx in indices:
        if idx < 0 or idx >= len(samples):
            continue
        sample = samples[idx]
        if sample.yaw is None or not math.isfinite(sample.yaw):
            continue
        x, y = project_pose(sample, "xy")
        ax.scatter(
            [x],
            [y],
            marker=heading_triangle_marker(sample.yaw),
            s=marker_area,
            facecolor=color,
            edgecolor=color,
            linewidths=0.0,
            alpha=alpha,
            zorder=zorder,
        )


def path_linewidth(args: argparse.Namespace, value: float | None) -> float:
    return args.trajectory_linewidth if value is None else value


def apply_2d_path_limits(ax: Any, series: list[list[PoseSample]], args: argparse.Namespace) -> None:
    points = [project_pose(sample, args.trajectory_plane) for samples in series for sample in samples]
    if not points:
        return
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    xmin, xmax = min(xs), max(xs)
    ymin, ymax = min(ys), max(ys)
    span = max(xmax - xmin, ymax - ymin, 1.0)
    pad = span * max(args.axis_padding_fraction, 0.0)
    half = span * 0.5 + pad
    ax.set_xlim((xmin + xmax) * 0.5 - half, (xmin + xmax) * 0.5 + half)
    ax.set_ylim((ymin + ymax) * 0.5 - half, (ymin + ymax) * 0.5 + half)


def apply_3d_path_limits(ax: Any, series: list[list[PoseSample]], args: argparse.Namespace) -> None:
    samples = [sample for samples in series for sample in samples]
    if not samples:
        return
    mins = [min(getattr(sample, axis) for sample in samples) for axis in ("x", "y", "z")]
    maxs = [max(getattr(sample, axis) for sample in samples) for axis in ("x", "y", "z")]
    span = max(maxs[idx] - mins[idx] for idx in range(3))
    if span <= 0.0:
        span = 1.0
    pad = span * max(args.axis_padding_fraction, 0.0)
    half = span * 0.5 + pad
    mids = [(mins[idx] + maxs[idx]) * 0.5 for idx in range(3)]
    ax.set_xlim(mids[0] - half, mids[0] + half)
    ax.set_ylim(mids[1] - half, mids[1] + half)
    z_min = args.z_min if args.z_min is not None else mins[2]
    z_max = args.z_max if args.z_max is not None else max(maxs[2] + pad, z_min + 1.0)
    ax.set_zlim(z_min, z_max)


def plot_pose_series_2d(
    ax: Any,
    samples: list[PoseSample],
    args: argparse.Namespace,
    *,
    series_name: str,
    label: str,
    color: str,
    linestyle: str,
    linewidth: float,
    marker: str,
    alpha: float | None = None,
    zorder: float = 2.0,
) -> None:
    if not samples:
        return
    xs, ys = zip(*(project_pose(sample, args.trajectory_plane) for sample in samples))
    indices = marker_indices_by_distance(samples, args)
    draw_heading_markers = use_heading_markers(series_name, args)
    kwargs = path_plot_kwargs(args, linestyle, linewidth, "" if draw_heading_markers else marker, alpha)
    if marker and not draw_heading_markers:
        kwargs["markevery"] = indices
    ax.plot(xs, ys, label=label, color=color, zorder=zorder, **kwargs)
    if draw_heading_markers:
        plot_heading_markers(
            ax,
            samples,
            indices,
            color=color,
            size=args.heading_marker_size,
            alpha=args.path_alpha if alpha is None else alpha,
            zorder=zorder + 0.2,
        )


def plot_pose_series_3d(
    ax: Any,
    samples: list[PoseSample],
    args: argparse.Namespace,
    *,
    label: str,
    color: str,
    linestyle: str,
    linewidth: float,
    marker: str,
    alpha: float | None = None,
    zorder: float = 2.0,
) -> None:
    if not samples:
        return
    kwargs = path_plot_kwargs(args, linestyle, linewidth, marker, alpha)
    if marker:
        kwargs["markevery"] = marker_indices_by_distance(samples, args)
    ax.plot([s.x for s in samples], [s.y for s in samples], [s.z for s in samples], label=label, color=color, zorder=zorder, **kwargs)


def plot_timestamp_links_3d(ax: Any, data: RunData, args: argparse.Namespace) -> None:
    if args.timestamp_links == "none" or not data.uav:
        return
    indices = marker_indices_by_distance(data.uav, args)
    targets: list[tuple[str, list[PoseSample], str, float]] = []
    if args.timestamp_links in {"ugv", "both"}:
        targets.append(("UGV", data.ugv, "#777777", 1.4))
    if args.timestamp_links in {"estimate", "both"}:
        targets.append(("Estimate", data.estimate, "#F58518", 1.5))
    for idx in indices:
        if idx < 0 or idx >= len(data.uav):
            continue
        uav = data.uav[idx]
        for _label, samples, color, zorder in targets:
            target = nearest_pose(samples, uav.t, args.nearest_max_gap_s)
            if target is None:
                continue
            ax.plot(
                [uav.x, target.x],
                [uav.y, target.y],
                [uav.z, target.z],
                color=color,
                linestyle=args.timestamp_link_linestyle,
                linewidth=args.timestamp_link_linewidth,
                alpha=args.timestamp_link_alpha,
                zorder=zorder,
            )


def plot_path_run(data: RunData, stem: Path, args: argparse.Namespace) -> None:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    if not data.ugv and not data.uav and not data.estimate:
        print(f"skip={stem.with_name(f'{stem.name}_path.png')} reason=no_path_samples")
        return
    if args.dimension == "3d":
        fig = plt.figure(figsize=single_figsize(args))
        if args.figure_right is None or args.figure_top is None:
            fig.subplots_adjust(
                right=args.figure_right if args.figure_right is not None else 0.84,
                top=args.figure_top if args.figure_top is not None else 0.86,
                left=args.figure_left if args.figure_left is not None else 0.02,
                bottom=args.figure_bottom if args.figure_bottom is not None else 0.02,
            )
        ax = fig.add_subplot(111, projection="3d")
        ax.set_xlabel("X (m)", labelpad=args.labelpad)
        ax.set_ylabel("Y (m)", labelpad=args.labelpad)
        ax.set_zlabel("Z (m)", labelpad=args.z_labelpad)
        ax.view_init(elev=args.view_elev, azim=args.view_azim)
        apply_3d_path_limits(ax, [data.ugv, data.uav, data.estimate], args)
        plot_timestamp_links_3d(ax, data, args)
        plot_pose_series_3d(ax, data.ugv, args, label="UGV", color="#333333", linestyle=args.ugv_linestyle, linewidth=path_linewidth(args, args.ugv_linewidth), marker=args.ugv_marker, zorder=2.0)
        plot_pose_series_3d(ax, data.uav, args, label="UAV", color="#4C78A8", linestyle=args.uav_linestyle, linewidth=path_linewidth(args, args.uav_linewidth), marker=args.uav_marker, zorder=3.0)
        plot_pose_series_3d(ax, data.estimate, args, label="Estimate", color="#F58518", linestyle=args.estimate_linestyle, linewidth=path_linewidth(args, args.estimate_linewidth), marker=args.estimate_marker, alpha=0.8, zorder=4.0)
        handles, labels = ax.get_legend_handles_labels()
        if handles:
            ax.legend(
                handles,
                labels,
                frameon=False,
                loc="upper center",
                bbox_to_anchor=(args.legend_x, args.legend_y),
                ncol=legend_ncol(args, handles),
                handlelength=args.legend_handlelength,
                columnspacing=args.legend_columnspacing,
                borderaxespad=args.legend_borderaxespad,
            )
    else:
        fig, ax = plt.subplots(figsize=single_figsize(args))
        plot_pose_series_2d(ax, data.ugv, args, series_name="ugv", label="UGV", color="#333333", linestyle=args.ugv_linestyle, linewidth=path_linewidth(args, args.ugv_linewidth), marker=args.ugv_marker, zorder=2.0)
        plot_pose_series_2d(ax, data.uav, args, series_name="uav", label="UAV", color="#4C78A8", linestyle=args.uav_linestyle, linewidth=path_linewidth(args, args.uav_linewidth), marker=args.uav_marker, alpha=0.8, zorder=3.0)
        plot_pose_series_2d(ax, data.estimate, args, series_name="estimate", label="Estimate", color="#F58518", linestyle=args.estimate_linestyle, linewidth=path_linewidth(args, args.estimate_linewidth), marker=args.estimate_marker, alpha=0.8, zorder=4.0)
        labels = {
            "xy": ("X (m)", "Y (m)"),
            "xz": ("X (m)", "Z (m)"),
            "yz": ("Y (m)", "Z (m)"),
        }
        ax.set_xlabel(labels[args.trajectory_plane][0], labelpad=args.labelpad)
        ax.set_ylabel(labels[args.trajectory_plane][1], labelpad=args.labelpad)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.28)
        apply_2d_path_limits(ax, [data.ugv, data.uav, data.estimate], args)
        handles, labels = ax.get_legend_handles_labels()
        handles, labels = route_legend_handles(data, handles, labels)
        if handles:
            ax.legend(
                handles,
                labels,
                frameon=False,
                loc="upper center",
                bbox_to_anchor=(args.legend_x, args.legend_y),
                ncol=legend_ncol(args, handles),
                handlelength=args.legend_handlelength,
                columnspacing=args.legend_columnspacing,
                borderaxespad=args.legend_borderaxespad,
            )
        #legend_outside(ax)
    suffix = "path_3d" if args.dimension == "3d" else f"path_{args.trajectory_plane}"
    previous_no_tight_layout = args.no_tight_layout
    if args.dimension == "3d":
        args.no_tight_layout = True
    try:
        save_metric_fig(fig, stem, suffix, args)
    finally:
        args.no_tight_layout = previous_no_tight_layout
    plt.close(fig)


def plot_path_runs(data_runs: list[RunData], out_root: Path, args: argparse.Namespace) -> None:
    out_root = path_dimension_root(out_root, args)
    for data in data_runs:
        plot_path_run(data, out_stem_for_path_run(data, out_root), args)


def route_sort_key(data: RunData) -> tuple[int, str]:
    code = route_code(data)
    if re.fullmatch(r"[A-Z]", code):
        return ord(code) - ord("A"), data.route
    return 99, data.route


def short_route_label(data: RunData) -> str:
    code = route_code(data)
    if re.fullmatch(r"[A-Z]", code):
        return f"Route {code}: {data.route.title()}"
    return data.route.title()


def resample_pose_path(samples: list[PoseSample], count: int = 300) -> list[PoseSample]:
    if not samples:
        return []
    if len(samples) == 1 or count <= 1:
        return [samples[0]]
    distances = [0.0]
    for prev, cur in zip(samples, samples[1:]):
        distances.append(distances[-1] + math.sqrt((cur.x - prev.x) ** 2 + (cur.y - prev.y) ** 2 + (cur.z - prev.z) ** 2))
    total = distances[-1]
    if total <= 0.0:
        return [samples[min(round(idx * (len(samples) - 1) / (count - 1)), len(samples) - 1)] for idx in range(count)]
    out: list[PoseSample] = []
    seg = 1
    for idx in range(count):
        target = total * idx / (count - 1)
        while seg < len(distances) and distances[seg] < target:
            seg += 1
        if seg >= len(distances):
            out.append(samples[-1])
            continue
        p0 = samples[seg - 1]
        p1 = samples[seg]
        d0 = distances[seg - 1]
        d1 = distances[seg]
        ratio = 0.0 if d1 <= d0 else (target - d0) / (d1 - d0)
        yaw = p1.yaw if p1.yaw is not None else p0.yaw
        out.append(
            PoseSample(
                p0.t + (p1.t - p0.t) * ratio,
                p0.x + (p1.x - p0.x) * ratio,
                p0.y + (p1.y - p0.y) * ratio,
                p0.z + (p1.z - p0.z) * ratio,
                yaw,
            )
        )
    return out


def average_pose_paths(paths: list[list[PoseSample]], count: int = 300) -> list[PoseSample]:
    resampled = [resample_pose_path(path, count) for path in paths if path]
    if not resampled:
        return []
    out: list[PoseSample] = []
    for idx in range(count):
        rows = [path[idx] for path in resampled if idx < len(path)]
        if not rows:
            continue
        out.append(
            PoseSample(
                sum(row.t for row in rows) / len(rows),
                sum(row.x for row in rows) / len(rows),
                sum(row.y for row in rows) / len(rows),
                sum(row.z for row in rows) / len(rows),
                None,
            )
        )
    return out


def set_common_2d_limits(axes: list[Any]) -> None:
    limits = []
    for ax in axes:
        xmin, xmax = ax.get_xlim()
        ymin, ymax = ax.get_ylim()
        if all(math.isfinite(value) for value in (xmin, xmax, ymin, ymax)):
            limits.append((xmin, xmax, ymin, ymax))
    if not limits:
        return
    xmin = min(item[0] for item in limits)
    xmax = max(item[1] for item in limits)
    ymin = min(item[2] for item in limits)
    ymax = max(item[3] for item in limits)
    span = max(xmax - xmin, ymax - ymin, 1.0)
    pad = span * 0.02
    half = span * 0.5 + pad
    x_mid = (xmin + xmax) * 0.5
    y_mid = (ymin + ymax) * 0.5
    for ax in axes:
        ax.set_xlim(x_mid - half, x_mid + half)
        ax.set_ylim(y_mid - half, y_mid + half)


def stacked_route_stem(out_root: Path, scope: str, args: argparse.Namespace) -> Path:
    if args.dimension == "3d":
        return out_root / "3d" / scope / "all_routes_trajectories_stacked_3d"
    suffix = "" if args.trajectory_plane == "xy" else f"_{args.trajectory_plane}"
    return out_root / scope / f"all_routes_trajectories_stacked{suffix}"


def grouped_route_paths(data_runs: list[RunData]) -> dict[str, tuple[RunData, list[PoseSample], list[PoseSample]]]:
    routes: dict[str, list[RunData]] = {}
    for data in sorted(data_runs, key=route_sort_key):
        routes.setdefault(route_code(data), []).append(data)
    out: dict[str, tuple[RunData, list[PoseSample], list[PoseSample]]] = {}
    for code, runs in routes.items():
        ugv = average_pose_paths([run.ugv for run in runs if run.ugv])
        uav = average_pose_paths([run.uav for run in runs if run.uav])
        if ugv or uav:
            out[code] = (runs[0], ugv, uav)
    return out


def plot_stacked_routes_3d(data_runs: list[RunData], out_root: Path, scope: str, args: argparse.Namespace) -> None:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    colors = ["#4C78A8", "#F58518", "#54A24B", "#B279A2", "#E45756", "#72B7B2", "#FF9DA6", "#9D755D", "#BAB0AC"]
    route_paths = grouped_route_paths(data_runs)
    if not route_paths:
        print("skip=stacked_routes_3d reason=no_path_samples")
        return

    fig = plt.figure(figsize=(max(args.width * 2.0, 6.6), max(args.height * 1.35, 4.4)))
    ax = fig.add_subplot(111, projection="3d")
    route_handles: list[Any] = []
    route_labels: list[str] = []
    all_series: list[list[PoseSample]] = []

    for idx, (_code, (data, ugv, uav)) in enumerate(route_paths.items()):
        color = colors[idx % len(colors)]
        label = short_route_label(data)
        if ugv:
            plot_pose_series_3d(
                ax,
                ugv,
                args,
                label=f"{label} UGV",
                color=color,
                linestyle=args.ugv_linestyle,
                linewidth=path_linewidth(args, args.ugv_linewidth),
                marker="",
                alpha=0.95,
                zorder=2.0,
            )
            all_series.append(ugv)
        if uav:
            plot_pose_series_3d(
                ax,
                uav,
                args,
                label=f"{label} UAV",
                color=color,
                linestyle=args.uav_linestyle,
                linewidth=path_linewidth(args, args.uav_linewidth),
                marker="",
                alpha=0.8,
                zorder=3.0,
            )
            all_series.append(uav)
        route_handles.append(ax.plot([], [], [], color=color, linestyle="-", linewidth=1.4)[0])
        route_labels.append(label)

    ax.set_xlabel("X (m)", labelpad=args.labelpad)
    ax.set_ylabel("Y (m)", labelpad=args.labelpad)
    ax.set_zlabel("Z (m)", labelpad=args.z_labelpad)
    ax.view_init(elev=args.view_elev, azim=args.view_azim)
    apply_3d_path_limits(ax, all_series, args)
    ax.grid(True, alpha=0.28)
    if route_handles:
        ax.legend(
            route_handles,
            route_labels,
            frameon=False,
            loc="upper center",
            bbox_to_anchor=(args.legend_x, args.legend_y),
            ncol=args.legend_ncol if args.legend_ncol > 0 else min(len(route_handles), 5),
            handlelength=args.legend_handlelength,
            columnspacing=args.legend_columnspacing,
            borderaxespad=args.legend_borderaxespad,
        )
    previous_no_tight_layout = args.no_tight_layout
    args.no_tight_layout = True
    try:
        save_fig(fig, stacked_route_stem(out_root, scope, args), args)
    finally:
        args.no_tight_layout = previous_no_tight_layout
    plt.close(fig)


def plot_stacked_routes(data_runs: list[RunData], out_root: Path, scope: str, args: argparse.Namespace) -> None:
    if args.dimension == "3d":
        plot_stacked_routes_3d(data_runs, out_root, scope, args)
        return
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    colors = ["#4C78A8", "#F58518", "#54A24B", "#B279A2", "#E45756", "#72B7B2", "#FF9DA6", "#9D755D", "#BAB0AC"]
    route_paths = grouped_route_paths(data_runs)

    fig, axes = plt.subplots(1, 2, figsize=(max(args.width * 2.0, 6.6), args.height), sharex=False, sharey=False)
    route_handles: list[Any] = []
    route_labels: list[str] = []
    plotted = 0

    for idx, (_code, (data, ugv, uav)) in enumerate(route_paths.items()):
        color = colors[idx % len(colors)]
        if not ugv and not uav:
            continue
        label = short_route_label(data)
        if ugv:
            plot_pose_series_2d(
                axes[0],
                ugv,
                args,
                series_name="ugv",
                label=label,
                color=color,
                linestyle=args.ugv_linestyle,
                linewidth=path_linewidth(args, args.ugv_linewidth),
                marker="",
                alpha=0.95,
                zorder=2.0,
            )
            plotted += 1
        if uav:
            plot_pose_series_2d(
                axes[1],
                uav,
                args,
                series_name="uav",
                label=label,
                color=color,
                linestyle=args.uav_linestyle,
                linewidth=path_linewidth(args, args.uav_linewidth),
                marker="",
                alpha=0.8,
                zorder=3.0,
            )
            plotted += 1
        route_handles.append(axes[1].plot([], [], color=color, linestyle="-", linewidth=1.4)[0])
        route_labels.append(label)

    if plotted == 0:
        plt.close(fig)
        print("skip=stacked_routes reason=no_path_samples")
        return

    axis_labels = {
        "xy": ("X (m)", "Y (m)"),
        "xz": ("X (m)", "Z (m)"),
        "yz": ("Y (m)", "Z (m)"),
    }
    xlabel, ylabel = axis_labels[args.trajectory_plane]
    for ax, panel_label in zip(axes, ("UGV", "UAV")):
        ax.set_title(panel_label, pad=8)
        ax.set_xlabel(xlabel, labelpad=args.labelpad)
        ax.set_ylabel(ylabel, labelpad=args.labelpad)
        ax.set_aspect("equal", adjustable="box")
        ax.grid(True, alpha=0.28)
    set_common_2d_limits(list(axes))
    if route_handles:
        fig.legend(
            route_handles,
            route_labels,
            frameon=False,
            loc="upper center",
            bbox_to_anchor=(args.legend_x, args.legend_y),
            ncol=args.legend_ncol if args.legend_ncol > 0 else min(len(route_handles), 9),
            handlelength=args.legend_handlelength,
            handletextpad=0.25,
            columnspacing=args.legend_columnspacing,
            borderaxespad=args.legend_borderaxespad,
        )
    save_fig(fig, stacked_route_stem(out_root, scope, args), args)
    plt.close(fig)


def safe_scope_name(value: str) -> str:
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", value).strip("_") or "runs"


def plot_stacked_route_groups(data_runs: list[RunData], out_root: Path, args: argparse.Namespace) -> None:
    if not data_runs:
        return
    reps: dict[str, list[RunData]] = {}
    for data in data_runs:
        reps.setdefault(data.rep, []).append(data)
    if len(reps) > 1:
        plot_stacked_routes(data_runs, out_root, "all_reps", args)
    for rep, runs in sorted(reps.items()):
        plot_stacked_routes(runs, out_root, safe_scope_name(rep), args)


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

    series = metric_series(data, metric_name)
    if not series:
        print(f"skip={stem.with_name(f'{stem.name}_{suffix}.png')} reason=no_{metric_name}_samples")
        return False

    fig, ax = plt.subplots(figsize=single_figsize(args))
    x = [t for t, _v in series]
    y = [v * scale for _t, v in series]
    ax.plot(x, y, color=color, marker="o", markersize=2.6, markeredgewidth=0.0, label=title)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.28)
    expand_flat_ylim(ax, y, floor_zero=floor_zero)
    if metric_name not in {"rssi_dbm", "per"}:
        legend_outside(ax)
    save_metric_fig(fig, stem, suffix, args)
    plt.close(fig)
    return True


def plot_distance(data: RunData, stem: Path, args: argparse.Namespace) -> bool:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    if not data.distances and not radio_derived_distance_series(data):
        print(f"skip={stem.with_name(f'{stem.name}_distance.png')} reason=no_distance_samples")
        return False

    fig, ax = plt.subplots(figsize=single_figsize(args))
    y_values: list[float] = []
    if data.distances:
        t = [row[0] for row in data.distances]
        d3 = [row[1] for row in data.distances]
        xy = [row[2] for row in data.distances]
        dz = [row[3] for row in data.distances]
        ax.plot(t, d3, color="#4C78A8", label="True 3D Distance")
        ax.plot(t, xy, color="#72B7B2", linestyle="--", label="Horizontal Distance")
        ax.plot(t, dz, color="#E45756", linestyle=":", label="Height Delta")
        y_values.extend(d3)
        y_values.extend(xy)
        y_values.extend(dz)
    radio = radio_derived_distance_series(data)
    if radio:
        y = [v for _t, v in radio]
        ax.plot([t for t, _v in radio], y, color="#F58518", marker="o", markersize=2.4, alpha=0.9, label="Radio Distance")
        y_values.extend(y)
    ax.set_xlabel("Time (s)")
    ax.set_ylabel("Distance (m)")
    ax.grid(True, alpha=0.28)
    expand_flat_ylim(ax, y_values, floor_zero=True)
    legend_outside(ax)
    save_metric_fig(fig, stem, "distance", args)
    plt.close(fig)
    return True


def metric_vs_distance_points(data: RunData, metric_name: str, args: argparse.Namespace) -> tuple[list[float], list[float]]:
    dist_t = [row[0] for row in data.distances]
    dist_d = [row[1] for row in data.distances]
    xs: list[float] = []
    ys: list[float] = []
    for metric_t, value in metric_series(data, metric_name):
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

    if not data.distances or not metric_series(data, metric_name):
        print(f"skip={stem.with_name(f'{stem.name}_{suffix}.png')} reason=no_distance_or_{metric_name}_samples")
        return False
    x, y_raw = metric_vs_distance_points(data, metric_name, args)
    y = [v * scale for v in y_raw]
    if not x:
        print(f"skip={stem.with_name(f'{stem.name}_{suffix}.png')} reason=no_nearest_distance_pairs")
        return False

    fig, ax = plt.subplots(figsize=single_figsize(args))
    ax.scatter(x, y, s=22, alpha=0.72, color=color, edgecolors="none", label=title)
    ax.set_xlabel("True UAV-UGV Distance (m)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.28)
    expand_flat_ylim(ax, y, floor_zero=floor_zero)
    if metric_name not in {"rssi_dbm", "per"}:
        legend_outside(ax)
    save_metric_fig(fig, stem, suffix, args)
    plt.close(fig)
    return True


def add_metric_line(
    ax: Any,
    data: RunData,
    metric_name: str,
    label: str,
    color: str,
    *,
    scale: float = 1.0,
    linestyle: str = "-",
    marker: str | None = None,
    alpha: float = 0.95,
) -> list[float]:
    series = metric_series(data, metric_name)
    if not series:
        return []
    x = [t for t, _v in series]
    y = [v * scale for _t, v in series]
    ax.plot(
        x,
        y,
        color=color,
        linestyle=linestyle,
        marker=marker or "",
        markersize=2.2 if marker else 0.0,
        markeredgewidth=0.0,
        alpha=alpha,
        label=label,
    )
    return y


def set_panel_empty(ax: Any, label: str) -> None:
    ax.text(0.5, 0.5, f"no {label} samples", ha="center", va="center", transform=ax.transAxes)
    ax.set_yticks([])


def plot_combined_run(data: RunData, stem: Path, args: argparse.Namespace) -> None:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=tall_figsize(args, 2))

    # Distance panel.
    ax = axes[0]
    y_values: list[float] = []
    if data.distances:
        t = [row[0] for row in data.distances]
        d3 = [row[1] for row in data.distances]
        ax.plot(t, d3, color="#4C78A8", label=r"$d$")
        y_values.extend(d3)
        if normalize_rep_label(data.rep) == "rep03":
            dz = [row[3] for row in data.distances]
            ax.plot(t, dz, color="#E45756", linestyle=":", label=r"$\Delta z$")
            y_values.extend(dz)
    y_values.extend(add_metric_line(ax, data, RADIO_DERIVED_DISTANCE_METRIC, r"$d_r$", "#F58518", alpha=0.9))
    radio_errors = compute_radio_distance_errors(data, args.nearest_max_gap_s)
    if radio_errors:
        y = [value for _t, value in radio_errors]
        ax.plot([t for t, _value in radio_errors], y, color="#333333", linestyle=":", alpha=0.85, label=r"$\Delta d$")
        y_values.extend(y)
    ax.set_ylabel("Distance (m)")
    if y_values:
        expand_flat_ylim(ax, y_values, floor_zero=True)
        legend_right(ax)
    else:
        set_panel_empty(ax, "distance")

    # RSSI and PER panel. SNIR, latency, PDR, and jitter stay in CSV only.
    ax = axes[1]
    y_values = []
    y_values.extend(add_metric_line(ax, data, "rssi_dbm", "RSSI", "#4C78A8", alpha=0.8))
    ax.set_ylabel("RSSI (dBm)")
    if y_values:
        expand_flat_ylim(ax, y_values)
    else:
        set_panel_empty(ax, "RSSI")
    ax.set_xlabel("Simulation Time (s)")
    ax2 = ax.twinx()
    per_values = add_metric_line(ax2, data, "per", "PER", "#E45756", scale=100.0, marker="o", alpha=0.8)
    ax2.set_ylabel(per_axis_label(args), labelpad=args.labelpad + 6.0)
    if per_values:
        expand_per_ylim(ax2, per_values)

    for ax in axes:
        ax.grid(True, alpha=0.28)

    save_metric_fig(fig, stem, "combined", args)
    plt.close(fig)


def plot_separate_run(data: RunData, stem: Path, args: argparse.Namespace) -> None:
    plot_distance(data, stem, args)
    plot_time_series(data, stem, args, RADIO_DERIVED_DISTANCE_METRIC, "radio_distance", "Radio Distance", "Distance (m)", "#F58518", floor_zero=True)
    plot_time_series(data, stem, args, "rssi_dbm", "rssi", "RSSI", "RSSI (dBm)", "#4C78A8")
    plot_time_series(data, stem, args, "per", "per", "PER", per_axis_label(args), "#E45756", scale=100.0, floor_zero=True)
    plot_metric_vs_distance(data, stem, args, "rssi_dbm", "rssi_vs_distance", "RSSI", "RSSI (dBm)", "#4C78A8")
    plot_metric_vs_distance(data, stem, args, "per", "per_vs_distance", "PER", per_axis_label(args), "#E45756", scale=100.0, floor_zero=True)
    plot_metric_vs_distance(data, stem, args, RADIO_DERIVED_DISTANCE_METRIC, "radio_vs_true_distance", "Radio Distance", "Radio Distance (m)", "#F58518", floor_zero=True)


def plot_run(data: RunData, stem: Path, args: argparse.Namespace) -> None:
    if args.figures in {"combined", "both"}:
        plot_combined_run(data, stem, args)
    if args.figures in {"separate", "both"}:
        plot_separate_run(data, stem, args)


def binned_mean_from_runs(
    runs: list[RunData],
    series_fn: Any,
    bin_s: float,
) -> tuple[list[float], list[float]]:
    per_run: list[dict[int, float]] = []
    for data in runs:
        buckets: dict[int, list[float]] = {}
        for t, value in series_fn(data):
            if not (math.isfinite(t) and math.isfinite(value)):
                continue
            idx = int(t // bin_s)
            buckets.setdefault(idx, []).append(value)
        if buckets:
            per_run.append({idx: fmean(values) for idx, values in buckets.items() if values})
    if not per_run:
        return [], []
    indices = sorted(set().union(*(run_bins.keys() for run_bins in per_run)))
    x: list[float] = []
    y: list[float] = []
    for idx in indices:
        values = [run_bins[idx] for run_bins in per_run if idx in run_bins]
        if values:
            x.append(idx * bin_s)
            y.append(fmean(values))
    return x, y


def plot_rep_average_combined(rep: str, data_runs: list[RunData], stem: Path, args: argparse.Namespace) -> None:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    bin_s = max(args.average_bin_s, 0.1)
    fig, axes = plt.subplots(2, 1, sharex=True, figsize=tall_figsize(args, 2))

    def dist_col(col: int) -> Any:
        return lambda data: [(row[0], row[col]) for row in data.distances]

    def metric(name: str, scale: float = 1.0) -> Any:
        return lambda data: [(t, value * scale) for t, value in metric_series(data, name)]

    ax = axes[0]
    y_values: list[float] = []
    x, y = binned_mean_from_runs(data_runs, dist_col(1), bin_s)
    if y:
        ax.plot(x, y, color="#4C78A8", label=r"$d$")
        y_values.extend(y)
    if normalize_rep_label(rep) == "rep03":
        x, y = binned_mean_from_runs(data_runs, dist_col(3), bin_s)
        if y:
            ax.plot(x, y, color="#E45756", linestyle=":", label=r"$\Delta z$")
            y_values.extend(y)
    x, y = binned_mean_from_runs(data_runs, metric(RADIO_DERIVED_DISTANCE_METRIC), bin_s)
    if y:
        ax.plot(x, y, color="#F58518", alpha=0.9, label=r"$d_r$")
        y_values.extend(y)
    x, y = binned_mean_from_runs(
        data_runs,
        lambda data: compute_radio_distance_errors(data, args.nearest_max_gap_s),
        bin_s,
    )
    if y:
        ax.plot(x, y, color="#333333", linestyle=":", alpha=0.85, label=r"$\Delta d$")
        y_values.extend(y)
    ax.set_ylabel("Distance (m)")
    if y_values:
        expand_flat_ylim(ax, y_values, floor_zero=True)
        legend_right(ax)
    else:
        set_panel_empty(ax, "distance")

    ax = axes[1]
    x, y = binned_mean_from_runs(data_runs, metric("rssi_dbm"), bin_s)
    if y:
        ax.plot(x, y, color="#4C78A8", alpha=0.85, label="RSSI")
        expand_flat_ylim(ax, y)
    else:
        set_panel_empty(ax, "RSSI")
    ax.set_ylabel("RSSI (dBm)")
    ax.set_xlabel("Simulation Time (s)")
    ax2 = ax.twinx()
    x2, y2 = binned_mean_from_runs(data_runs, metric("per", 100.0), bin_s)
    if y2:
        ax2.plot(x2, y2, color="#E45756", marker="o", markersize=2.6, markeredgewidth=0.0, alpha=0.85, label="PER")
        expand_per_ylim(ax2, y2)
    ax2.set_ylabel(per_axis_label(args), labelpad=args.labelpad + 6.0)

    for ax in axes:
        ax.grid(True, alpha=0.28)
    save_metric_fig(fig, stem, "average_combined", args)
    plt.close(fig)


def plot_rep_average_groups(data_runs: list[RunData], out_root: Path, args: argparse.Namespace) -> None:
    groups: dict[str, list[RunData]] = {}
    for data in data_runs:
        groups.setdefault(normalize_rep_label(data.rep), []).append(data)
    for rep, runs in sorted(groups.items()):
        stem = out_root / "combined" / f"{display_condition_label(runs[0]).replace(' ', '_')}_network_metrics"
        plot_rep_average_combined(rep, runs, stem, args)


def write_summary_csv(data_runs: list[RunData], csv_path: Path, args: argparse.Namespace) -> None:
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
                "leader_estimate_error_m",
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
                    mean_or_nan([value for _t, value in compute_estimate_errors(data.ugv, data.estimate, args.nearest_max_gap_s)]),
                    mean_or_nan(metric_values(data, "radio_distance_m")),
                    mean_or_nan(metric_values(data, "link_distance_m")),
                    mean_or_nan(metric_values(data, "rssi_dbm")),
                    mean_or_nan(metric_values(data, "snir_db")),
                    mean_or_nan([value * 1000.0 for value in metric_values(data, "latency_s")]),
                    mean_or_nan([value * 1000.0 for value in metric_values(data, "jitter_s")]),
                    mean_or_nan(metric_values(data, "per")),
                    mean_or_nan(metric_values(data, "pdr")),
                ]
            )
    print(csv_path)


def summary_row(data: RunData, args: argparse.Namespace) -> dict[str, float | str | int]:
    return {
        "campaign": campaign_code(data),
        "rep": data.rep,
        "route": route_code(data),
        "route_name": data.route,
        "lora_mode": data.lora_mode,
        "samples_distance": len(data.distances),
        "true_distance_m": mean_or_nan([row[1] for row in data.distances]),
        "leader_estimate_error_m": mean_or_nan(
            [value for _t, value in compute_estimate_errors(data.ugv, data.estimate, args.nearest_max_gap_s)]
        ),
        "rssi_dbm": mean_or_nan(metric_values(data, "rssi_dbm")),
        "snir_db": mean_or_nan(metric_values(data, "snir_db")),
        "latency_ms": mean_or_nan([value * 1000.0 for value in metric_values(data, "latency_s")]),
        "jitter_ms": mean_or_nan([value * 1000.0 for value in metric_values(data, "jitter_s")]),
        "per": mean_or_nan(metric_values(data, "per")),
        "pdr": mean_or_nan(metric_values(data, "pdr")),
    }


def fmt_table_value(value: float | str | int, metric: str | None = None) -> str:
    if isinstance(value, str):
        return value
    if isinstance(value, int):
        return str(value)
    if not math.isfinite(value):
        return "n/a"
    if metric == "per":
        return f"{value * 100.0:.2f}%"
    if metric == "pdr":
        return f"{value:.3f}"
    if metric == "rssi_dbm":
        return f"{value:.1f}"
    if metric in {"latency_ms", "jitter_ms"}:
        return f"{value:.2f}"
    return f"{value:.2f}"


def write_metric_table(handle: Any, title: str, rows: list[dict[str, float | str | int]], metric: str, *, high_is_best: bool, limit: int) -> None:
    valid = [row for row in rows if isinstance(row.get(metric), float) and math.isfinite(row[metric])]  # type: ignore[arg-type]
    if not valid:
        handle.write(f"### {title}\n\nNo finite `{metric}` values.\n\n")
        return
    ordered_best = sorted(valid, key=lambda row: row[metric], reverse=high_is_best)[:limit]
    ordered_worst = sorted(valid, key=lambda row: row[metric], reverse=not high_is_best)[:limit]

    def write_rows(label: str, selected: list[dict[str, float | str | int]]) -> None:
        handle.write(f"#### {label}\n\n")
        handle.write("| campaign | route | run | mode | value | distance m | latency ms | PER | PDR |\n")
        handle.write("| --- | --- | --- | --- | ---: | ---: | ---: | ---: | ---: |\n")
        for row in selected:
            handle.write(
                "| "
                f"{row['campaign']} | Route {row['route']} | {row['rep']} | {row['lora_mode']} | "
                f"{fmt_table_value(row[metric], metric)} | "
                f"{fmt_table_value(row['true_distance_m'])} | "
                f"{fmt_table_value(row['latency_ms'], 'latency_ms')} | "
                f"{fmt_table_value(row['per'], 'per')} | "
                f"{fmt_table_value(row['pdr'], 'pdr')} |\n"
            )
        handle.write("\n")

    handle.write(f"### {title}\n\n")
    write_rows("Best", ordered_best)
    write_rows("Worst", ordered_worst)


def write_extremes_markdown(data_runs: list[RunData], md_path: Path, args: argparse.Namespace) -> None:
    limit = max(1, args.rank_limit)
    rows = [summary_row(data, args) for data in data_runs]
    md_path.parent.mkdir(parents=True, exist_ok=True)
    with md_path.open("w", encoding="utf-8") as handle:
        handle.write("# Network Metric Extremes\n\n")
        handle.write(
            "Best/worst tables are ranked by per-run mean values. RSSI and PDR are better when higher; "
            "PER and latency are better when lower.\n\n"
        )
        for campaign in sorted({str(row["campaign"]) for row in rows}):
            campaign_rows = [row for row in rows if row["campaign"] == campaign]
            handle.write(f"## {campaign}\n\n")
            write_metric_table(handle, "RSSI", campaign_rows, "rssi_dbm", high_is_best=True, limit=limit)
            write_metric_table(handle, "PER", campaign_rows, "per", high_is_best=False, limit=limit)
            write_metric_table(handle, "Latency", campaign_rows, "latency_ms", high_is_best=False, limit=limit)
            write_metric_table(handle, "PDR", campaign_rows, "pdr", high_is_best=True, limit=limit)
    print(md_path)


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

    fig, ax = plt.subplots(figsize=single_figsize(args))
    colors = ["#4C78A8", "#F58518", "#54A24B", "#B279A2", "#E45756", "#72B7B2", "#FF9DA6", "#9D755D"]
    markers = {"simplex": "o", "duplex": "s", "unknown": "^"}
    y_all: list[float] = []

    rep_style = all(re.fullmatch(r"rep\d+", data.rep.lower()) for data in data_runs)
    grouped: dict[tuple[str, str], list[tuple[RunData, list[float], list[float]]]] = {}
    for data in data_runs:
        group_label = display_condition_label(data) if rep_style else data.lora_mode
        group_key = (group_label, data.lora_mode)
        x, y_raw = metric_vs_distance_points(data, metric_name, args)
        y = [v * scale for v in y_raw]
        if not x:
            continue
        grouped.setdefault(group_key, []).append((data, x, y))

    for idx, ((group_label, mode), entries) in enumerate(sorted(grouped.items())):
        color = colors[idx % len(colors)]
        label_runs = [entry[0] for entry in entries]
        label = f"{group_label}: {compact_route_label(label_runs)}"
        plotted_label = False
        for _data, x, y in entries:
            y_all.extend(y)
            ax.scatter(
                x,
                y,
                s=18,
                alpha=0.55,
                color=color,
                marker=markers.get(mode, "^"),
                edgecolors="none",
                label=label if not plotted_label else None,
            )
            plotted_label = True

    ax.set_xlabel("True UAV-UGV Distance (m)")
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.28)
    expand_flat_ylim(ax, y_all, floor_zero=floor_zero)
    handles, labels = ax.get_legend_handles_labels()
    if handles and metric_name not in {"rssi_dbm", "per"}:
        legend_outside(ax)
    save_metric_fig(fig, stem, suffix, args)
    plt.close(fig)


def scatter_overview_metric(
    ax: Any,
    data_runs: list[RunData],
    args: argparse.Namespace,
    metric_name: str,
    *,
    scale: float = 1.0,
    floor_zero: bool = False,
    marker_override: str | None = None,
    label_groups: bool = True,
    alpha: float = 0.55,
) -> list[float]:
    colors = ["#4C78A8", "#F58518", "#54A24B", "#B279A2", "#E45756", "#72B7B2", "#FF9DA6", "#9D755D"]
    markers = {"simplex": "o", "duplex": "s", "unknown": "^"}
    rep_style = all(re.fullmatch(r"rep\d+", data.rep.lower()) for data in data_runs)
    grouped: dict[tuple[str, str], list[tuple[RunData, list[float], list[float]]]] = {}
    y_all: list[float] = []

    for data in data_runs:
        group_label = display_condition_label(data) if rep_style else data.lora_mode
        group_key = (group_label, data.lora_mode)
        x, y_raw = metric_vs_distance_points(data, metric_name, args)
        y = [v * scale for v in y_raw]
        if x:
            grouped.setdefault(group_key, []).append((data, x, y))

    for idx, ((group_label, mode), entries) in enumerate(sorted(grouped.items())):
        color = colors[idx % len(colors)]
        label = f"{group_label}: {compact_route_label([entry[0] for entry in entries])}"
        plotted_label = False
        for _data, x, y in entries:
            y_all.extend(y)
            marker = marker_override or markers.get(mode, "^")
            ax.scatter(
                x,
                y,
                s=18,
                alpha=alpha,
                color=color,
                marker=marker,
                edgecolors=color if marker == "x" else "none",
                label=label if label_groups and not plotted_label else None,
            )
            plotted_label = True

    if y_all:
        expand_flat_ylim(ax, y_all, floor_zero=floor_zero)
    return y_all


def plot_thesis_overviews(data_runs: list[RunData], stem: Path, args: argparse.Namespace) -> None:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    fig, ax = plt.subplots(1, 1, figsize=single_figsize(args))
    ax_per = ax.twinx()
    scatter_overview_metric(ax, data_runs, args, "rssi_dbm", label_groups=False)
    per_values = scatter_overview_metric(
        ax_per,
        data_runs,
        args,
        "per",
        scale=100.0,
        floor_zero=True,
        label_groups=False,
        marker_override="o",
        alpha=0.75,
    )
    if per_values:
        expand_per_ylim(ax_per, per_values)
    ax.set_xlabel("True UAV-UGV Distance (m)")
    ax.set_ylabel("RSSI (dBm)")
    ax_per.set_ylabel(per_axis_label(args), labelpad=args.labelpad + 6.0)
    ax.grid(True, alpha=0.28)
    save_metric_fig(fig, stem, "signal_loss_summary", args)
    plt.close(fig)


def plot_overviews(data_runs: list[RunData], stem: Path, args: argparse.Namespace) -> None:
    if args.overview_set in {"thesis", "both"}:
        plot_thesis_overviews(data_runs, stem, args)
    if args.overview_set in {"all", "both"}:
        plot_overview_metric_vs_distance(data_runs, stem, args, "rssi_dbm", "rssi_vs_distance", "RSSI", "RSSI (dBm)")
        plot_overview_metric_vs_distance(data_runs, stem, args, "per", "per_vs_distance", "PER", per_axis_label(args), scale=100.0, floor_zero=True)
        plot_overview_metric_vs_distance(data_runs, stem, args, RADIO_DERIVED_DISTANCE_METRIC, "radio_vs_true_distance", "Radio Distance", "Radio Distance (m)", floor_zero=True)


def main() -> None:
    args = parse_args()
    do_network = args.plot_type in {"network", "both"}
    do_path = args.plot_type in {"path", "both"}
    if args.results_dir:
        results_dir = Path(args.results_dir).expanduser().resolve()
        out_root, path_root = output_roots(args, results_dir)

        if args.offline_omnet:
            offline_runs = discover_offline_metric_runs(results_dir)
            network_filter = selected_networks(args.networks)
            if network_filter:
                offline_runs = [
                    run for run in offline_runs if run[3].parent.name.split("_sf", 1)[0].lower() in network_filter
                ]
            rep_filter = selected_reps(args.reps)
            if rep_filter:
                offline_runs = [
                    run for run in offline_runs if normalize_rep_label(run[1]) in rep_filter
                ]
            if not offline_runs:
                raise SystemExit(f"No offline_omnet/*/network_metrics.csv files found under: {results_dir}")
            if do_path and not do_network and not args.summary_only:
                data_runs = [
                    read_bag(
                        route,
                        rep,
                        infer_lora_mode_from_csv(metrics_csv, args.lora_mode),
                        bag_dir,
                        args,
                        read_ros_metrics=False,
                    )
                    for route, rep, bag_dir, metrics_csv in offline_runs
                ]
                plot_path_runs(data_runs, path_root, args)
                if args.stack_routes:
                    plot_stacked_route_groups(data_runs, path_root, args)
                return
            data_runs = []
            for route, rep, bag_dir, metrics_csv in offline_runs:
                mode = infer_lora_mode_from_csv(metrics_csv, args.lora_mode)
                data = read_bag(route, rep, mode, bag_dir, args, read_ros_metrics=False)
                data.metrics = read_metrics_csv(metrics_csv)
                data.metrics_csv = metrics_csv
                data_runs.append(data)
            if do_network:
                combined_root = out_root / "combined"
                write_summary_csv(data_runs, combined_root / "network_metrics_summary.csv", args)
                write_extremes_markdown(data_runs, combined_root / "network_metrics_extremes.md", args)
                if args.summary_only:
                    return
                if args.per_run_plots and not args.overview_only:
                    for data in data_runs:
                        plot_run(data, out_stem_for_run(data, out_root), args)
                if args.rep_average_plots:
                    plot_rep_average_groups(data_runs, out_root, args)
                if not args.rep_average_plots:
                    plot_overviews(data_runs, combined_root / "network_metrics_overview", args)
            if do_path and not args.summary_only:
                plot_path_runs(data_runs, path_root, args)
                if args.stack_routes:
                    plot_stacked_route_groups(data_runs, path_root, args)
            return

        runs = discover_result_bags(results_dir)
        rep_filter = selected_reps(args.reps)
        if rep_filter:
            runs = [run for run in runs if normalize_rep_label(run[1]) in rep_filter]
        if not runs:
            raise SystemExit(f"No repXX/RNN_route/bag directories found under: {results_dir}")
        if do_path and not do_network and not args.summary_only:
            data_runs = [
                read_bag(
                    route,
                    rep,
                    infer_lora_mode(rep, bag_dir, args.lora_mode),
                    bag_dir,
                    args,
                    read_ros_metrics=False,
                )
                for route, rep, bag_dir in runs
            ]
            plot_path_runs(data_runs, path_root, args)
            if args.stack_routes:
                plot_stacked_route_groups(data_runs, path_root, args)
            return
        data_runs = [
            read_bag(route, rep, infer_lora_mode(rep, bag_dir, args.lora_mode), bag_dir, args, read_ros_metrics=do_network)
            for route, rep, bag_dir in runs
        ]
        if do_network:
            combined_root = out_root / "combined"
            write_summary_csv(data_runs, combined_root / "network_metrics_summary.csv", args)
            write_extremes_markdown(data_runs, combined_root / "network_metrics_extremes.md", args)
            if args.summary_only:
                return
            if args.per_run_plots and not args.overview_only:
                for data in data_runs:
                    plot_run(data, out_stem_for_run(data, out_root), args)
            if args.rep_average_plots:
                plot_rep_average_groups(data_runs, out_root, args)
            if not args.rep_average_plots:
                plot_overviews(data_runs, combined_root / "network_metrics_overview", args)
        if do_path and not args.summary_only:
            plot_path_runs(data_runs, path_root, args)
            if args.stack_routes:
                plot_stacked_route_groups(data_runs, path_root, args)
        return

    route, rep, bag_dir = resolve_single_bag(args)
    data = read_bag(
        route,
        rep,
        infer_lora_mode(rep, bag_dir, args.lora_mode),
        bag_dir,
        args,
        read_ros_metrics=do_network and not args.metrics_csv,
    )
    if args.metrics_csv:
        metrics_csv = Path(args.metrics_csv).expanduser().resolve()
        data.metrics = read_metrics_csv(metrics_csv)
    if do_network:
        plot_run(data, single_out_stem(args, data), args)
    if do_path:
        plot_path_run(data, single_path_out_stem(args, data), args)


if __name__ == "__main__":
    main()
