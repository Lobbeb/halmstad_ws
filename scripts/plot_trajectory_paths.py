#!/usr/bin/env python3
"""Plot pose trajectories from a ROS 2 bag run folder.

This is a small handoff utility for comparing a reference path, UAV path,
and optional estimated/predicted paths. It intentionally stays independent
from the thesis-specific C1 plotting scripts.
"""
from __future__ import annotations

import argparse
import math
import re
from pathlib import Path
from typing import Any

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

REFERENCE_TOPIC_FALLBACKS = [
    "/a201_0000/ground_truth/odom",
    "/a201_0000/platform/odom/filtered",
    "/a201_0000/amcl_pose_odom",
    "/a201_0000/platform/odom",
]

SINGLE_FIGURE_WIDTH_IN = 3.3
SINGLE_FIGURE_HEIGHT_IN = 3.3
WIDE_FIGURE_WIDTH_IN = 6.6
WIDE_FIGURE_HEIGHT_IN = 3.4
AXIS_LIMIT_PAD_FRACTION = 0.03
PATH_MARKER_COUNT = 8
PathPoint = tuple[float, float, float, float | None]


def parse_bool(value: str) -> bool:
    lowered = str(value).strip().lower()
    if lowered in {"1", "true", "yes", "y", "on"}:
        return True
    if lowered in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError("expected true or false")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Plot reference/UAV/optional estimate trajectories from a ROS 2 bag."
    )
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--run-dir", help="Run directory containing a bag/ subdirectory.")
    source.add_argument("--bag", help="ROS 2 bag directory, for example run/bag.")
    source.add_argument(
        "--results-dir",
        help=(
            "Results directory containing repXX/<route>/bag folders. "
            "Writes one merged route sheet per route."
        ),
    )
    parser.add_argument(
        "--reference-topic",
        default="/a201_0000/ground_truth/odom",
        help="Reference/UGV path topic. Default: /a201_0000/ground_truth/odom",
    )
    parser.add_argument(
        "--uav-topic",
        default="/dji0/pose",
        help="UAV/follower path topic. Default: /dji0/pose",
    )
    parser.add_argument(
        "--estimated-topic",
        action="append",
        default=[],
        help=(
            "Optional estimated/predicted path topic. May be repeated. "
            "Use either /topic or Label=/topic."
        ),
    )
    parser.add_argument(
        "--extra-topic",
        action="append",
        default=[],
        help="Additional path series in Label=/topic form. May be repeated.",
    )
    parser.add_argument("--reference-label", default="UGV")
    parser.add_argument("--uav-label", default="UAV")
    parser.add_argument("--warmup", type=float, default=0.0, help="Seconds to skip from bag start.")
    parser.add_argument("--title", default="", help="Optional plot title.")
    parser.add_argument(
        "--out",
        help=(
            "Single-bag output stem/file, or batch output directory when "
            "--results-dir is used. PNG is written by default."
        ),
    )
    parser.add_argument("--pdf", action="store_true", help="Also write PDF output.")
    parser.add_argument("--dpi", type=int, default=220)
    parser.add_argument("--width", type=float, default=SINGLE_FIGURE_WIDTH_IN, help="Single-panel figure width in inches.")
    parser.add_argument("--height", type=float, default=SINGLE_FIGURE_HEIGHT_IN, help="Single-panel figure height in inches.")
    parser.add_argument("--font-size", type=float, default=11.0, help="Base plot font size.")
    parser.add_argument(
        "--plane",
        choices=["xy", "xz", "yz"],
        default="xy",
        help="Coordinate plane to plot. Default: xy",
    )
    parser.add_argument(
        "--batch-plots",
        choices=["average", "normal", "both"],
        default="both",
        help=(
            "When --results-dir is used, choose which route sheets to write. "
            "average draws mean paths across repetitions; "
            "normal draws all requested topics for every repetition. Default: both."
        ),
    )
    parser.add_argument(
        "--reps",
        help="Comma-separated repetitions to include with --results-dir, e.g. rep01 or rep01,rep03.",
    )
    parser.add_argument(
        "--stack-routes",
        action="store_true",
        help="With --results-dir, also write one overview with all route trajectories overlaid.",
    )
    parser.add_argument(
        "--average",
        type=parse_bool,
        default=None,
        metavar="true|false",
        help=(
            "Legacy alias for --batch-plots. true writes only average plots; "
            "false writes only normal plots."
        ),
    )
    parser.add_argument(
        "--no-equal-aspect",
        action="store_true",
        help="Do not force equal x/y aspect ratio.",
    )
    parser.add_argument(
        "--trajectory-style",
        choices=["markers", "lines"],
        default="markers",
        help="markers draws sparse position markers; lines restores the older continuous-line style.",
    )
    parser.add_argument(
        "--heading-markers",
        choices=["none", "ugv", "uav", "both"],
        default="none",
        help="Use rotated triangle markers from pose yaw in XY marker plots. Default: none.",
    )
    parser.add_argument(
        "--heading-marker-size",
        type=float,
        default=5.0,
        help="Size of rotated heading triangle markers. Default: 5.0.",
    )
    args = parser.parse_args()
    if args.average is not None:
        args.batch_plots = "average" if args.average else "normal"
    if args.out and Path(args.out).suffix.lower() == ".pdf":
        args.pdf = True
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


def rep_output_scope(value: str | None) -> str:
    reps = sorted(selected_reps(value))
    if not reps:
        return "all_reps"
    return "_".join(reps)


def yaw_from_quat(q: Any) -> float:
    x = float(getattr(q, "x", 0.0))
    y = float(getattr(q, "y", 0.0))
    z = float(getattr(q, "z", 0.0))
    w = float(getattr(q, "w", 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def point_from_position(position: Any, plane: str) -> tuple[float, float]:
    x = float(position.x)
    y = float(position.y)
    z = float(position.z)
    if plane == "xz":
        return x, z
    if plane == "yz":
        return y, z
    return x, y


def point_from_pose(pose: Any, plane: str) -> tuple[float, float, float | None]:
    x, y = point_from_position(pose.position, plane)
    yaw = yaw_from_quat(pose.orientation) if plane == "xy" and hasattr(pose, "orientation") else None
    return x, y, yaw


def pose_points_from_msg(msg: Any, plane: str) -> list[tuple[float, float, float | None]]:
    """Return one or more 2D projected points from common pose-like ROS messages."""
    try:
        if hasattr(msg, "poses"):
            points: list[tuple[float, float, float | None]] = []
            for pose_stamped in msg.poses:
                points.append(point_from_pose(pose_stamped.pose, plane))
            return points
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        return [point_from_pose(pose, plane)]
    except Exception:
        return []


def parse_series_arg(value: str, fallback_label: str) -> tuple[str, str]:
    if "=" in value:
        label, topic = value.split("=", 1)
        return label.strip() or fallback_label, topic.strip()
    topic = value.strip()
    label = topic.rsplit("/", 1)[-1] or fallback_label
    return label, topic


def resolve_bag_dir(args: argparse.Namespace) -> Path:
    if args.bag:
        return Path(args.bag).expanduser().resolve()
    run_dir = Path(args.run_dir).expanduser().resolve()
    return run_dir / "bag"


def route_label_from_run_dir(run_dir: Path) -> str:
    """Return a stable route label from known Baylands run-folder names."""
    name = run_dir.name
    selected_match = re.match(r"^C\d+_Route[A-Z]_(.+?)_r\d+__", name)
    if selected_match:
        return selected_match.group(1)
    return re.sub(r"^R\d+_", "", name)


def thesis_route_letter_from_name(route: str) -> str | None:
    key = route.strip().lower().replace("-", "_")
    return THESIS_ROUTE_BY_NAME.get(key) or THESIS_ROUTE_BY_NAME.get(key.replace("_", " "))


def route_code_from_run_dir(run_dir: Path) -> str | None:
    numbered_match = re.match(r"^R0*([1-9])(?:_|$)", run_dir.name)
    if numbered_match:
        return chr(ord("A") + int(numbered_match.group(1)) - 1)
    return None


def display_route_title(route: str, code: str | None = None) -> str:
    letter = code or thesis_route_letter_from_name(route)
    name = route.replace("_", " ").title()
    return f"Route {letter}: {name}" if letter else name


def display_rep_label(rep: str) -> str:
    return C2_REP_LABELS.get(rep.lower(), rep)


def rep_label_from_run_dir(run_dir: Path) -> str:
    selected_match = re.match(r"^(C\d+)_Route[A-Z]_.+?_(r\d+)__", run_dir.name)
    if selected_match:
        return f"{selected_match.group(1)}_{selected_match.group(2)}"
    for parent in run_dir.parents:
        if re.fullmatch(r"rep\d+", parent.name):
            return parent.name
    return run_dir.parent.name


def discover_results_runs(results_dir: Path) -> dict[str, list[tuple[str, str, Path]]]:
    groups: dict[str, list[tuple[str, str, Path]]] = {}
    for bag_dir in sorted(results_dir.glob("**/bag")):
        if not bag_dir.is_dir():
            continue
        if not (bag_dir / "metadata.yaml").is_file() and not list(bag_dir.glob("bag_*.mcap")):
            continue
        if "offline_omnet" in bag_dir.parts:
            continue
        run_dir = bag_dir.parent
        route = route_label_from_run_dir(run_dir)
        rep = rep_label_from_run_dir(run_dir)
        code = route_code_from_run_dir(run_dir) or thesis_route_letter_from_name(route) or route
        groups.setdefault(route, []).append((code, rep, bag_dir))
    for runs in groups.values():
        runs.sort(key=lambda item: (item[0], item[1]))
    return dict(sorted(groups.items(), key=lambda item: item[1][0][0] if item[1] else item[0]))


def point_x(point: PathPoint) -> float:
    return point[1]


def point_y(point: PathPoint) -> float:
    return point[2]


def point_yaw(point: PathPoint) -> float | None:
    return point[3] if len(point) > 3 else None


def path_xs(series: list[PathPoint]) -> list[float]:
    return [point_x(point) for point in series]


def path_ys(series: list[PathPoint]) -> list[float]:
    return [point_y(point) for point in series]


def marker_indices_by_distance(series: list[PathPoint], count: int = PATH_MARKER_COUNT) -> list[int]:
    if not series or count <= 0:
        return []
    if len(series) <= count:
        return list(range(len(series)))

    distances = [0.0]
    for prev, cur in zip(series, series[1:]):
        distances.append(distances[-1] + math.hypot(point_x(cur) - point_x(prev), point_y(cur) - point_y(prev)))
    total = distances[-1]
    if total <= 0.0:
        step = (len(series) - 1) / max(count - 1, 1)
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


def marker_for_topic(topic: str, args: argparse.Namespace) -> str:
    if topic == args.uav_topic:
        return "^"
    return "o"


def topic_uses_heading_markers(topic: str, args: argparse.Namespace) -> bool:
    if args.plane != "xy" or args.trajectory_style != "markers":
        return False
    if args.heading_markers in {"ugv", "both"} and topic == args.reference_topic:
        return True
    if args.heading_markers in {"uav", "both"} and topic == args.uav_topic:
        return True
    return False


def heading_triangle_marker(yaw: float) -> Any:
    from matplotlib.path import Path as MplPath

    base_vertices = [(1.0, 0.0), (-0.55, 0.45), (-0.55, -0.45), (1.0, 0.0)]
    c = math.cos(yaw)
    s = math.sin(yaw)
    vertices = [(x * c - y * s, x * s + y * c) for x, y in base_vertices]
    return MplPath(vertices, [MplPath.MOVETO, MplPath.LINETO, MplPath.LINETO, MplPath.CLOSEPOLY])


def plot_heading_markers(
    ax: Any,
    series: list[PathPoint],
    indices: list[int],
    *,
    color: Any,
    marker_size: float,
    zorder: float | None = None,
) -> None:
    for idx in indices:
        if idx < 0 or idx >= len(series):
            continue
        point = series[idx]
        yaw = point_yaw(point)
        if yaw is None or not math.isfinite(yaw):
            continue
        ax.scatter(
            [point_x(point)],
            [point_y(point)],
            marker=heading_triangle_marker(yaw),
            s=marker_size * marker_size,
            facecolor=color,
            edgecolor=color,
            linewidths=0.0,
            zorder=zorder,
        )



def plot_path_series(
    ax: Any,
    series: list[PathPoint],
    *,
    topic: str,
    args: argparse.Namespace,
    markers: bool = True,
    **kwargs: Any,
) -> None:
    marker_color = kwargs.get("color")
    marker_indices = marker_indices_by_distance(series)
    use_heading_markers = markers and topic_uses_heading_markers(topic, args)
    kwargs["linestyle"] = ("-")
    if markers and args.trajectory_style == "markers" and not use_heading_markers:
        kwargs.update(
            {
                "marker": marker_for_topic(topic, args),
                "markevery": marker_indices,
                "markersize": 4.0,
                "markerfacecolor": marker_color,
                "markeredgecolor": marker_color,
            }
        )
    ax.plot(path_xs(series), path_ys(series), **kwargs)
    if use_heading_markers:
        base_zorder = kwargs.get("zorder")
        marker_zorder = None if base_zorder is None else float(base_zorder) + 0.2
        plot_heading_markers(
            ax,
            series,
            marker_indices,
            color=marker_color,
            marker_size=args.heading_marker_size,
            zorder=marker_zorder,
        )


def resample_path(
    series: list[PathPoint],
    samples: int,
) -> list[PathPoint]:
    if not series:
        return []
    if samples <= 1 or len(series) == 1:
        return [series[0]]

    distances = [0.0]
    for prev, cur in zip(series, series[1:]):
        distances.append(distances[-1] + math.hypot(point_x(cur) - point_x(prev), point_y(cur) - point_y(prev)))

    total = distances[-1]
    if total <= 0.0:
        return [series[0]] * samples

    out: list[PathPoint] = []
    seg = 1
    for idx in range(samples):
        target = total * idx / (samples - 1)
        while seg < len(distances) - 1 and distances[seg] < target:
            seg += 1
        d0 = distances[seg - 1]
        d1 = distances[seg]
        p0 = series[seg - 1]
        p1 = series[seg]
        ratio = 0.0 if d1 <= d0 else (target - d0) / (d1 - d0)
        out.append((
            p0[0] + (p1[0] - p0[0]) * ratio,
            point_x(p0) + (point_x(p1) - point_x(p0)) * ratio,
            point_y(p0) + (point_y(p1) - point_y(p0)) * ratio,
            None,
        ))
    return out


def average_paths(
    paths: list[list[PathPoint]],
    samples: int = 300,
) -> tuple[list[PathPoint], int]:
    resampled = [resample_path(path, samples) for path in paths if path]
    if not resampled:
        return [], 0
    out: list[PathPoint] = []
    for idx in range(samples):
        ts = [path[idx][0] for path in resampled if idx < len(path)]
        xs = [point_x(path[idx]) for path in resampled if idx < len(path)]
        ys = [point_y(path[idx]) for path in resampled if idx < len(path)]
        if xs and ys:
            out.append((sum(ts) / len(ts), sum(xs) / len(xs), sum(ys) / len(ys), None))
    return out, len(resampled)


def batch_out_dir(args: argparse.Namespace, results_dir: Path) -> Path:
    base = Path(args.out).expanduser().resolve() if args.out else results_dir / "plots" / "trajectory"
    out = base / rep_output_scope(args.reps)
    out.mkdir(parents=True, exist_ok=True)
    return out


def configure_plot_style(args: argparse.Namespace) -> None:
    try:
        import matplotlib as mpl
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    mpl.rcParams.update(
        {
            "font.family": "serif",
            "font.serif": ["Nimbus Roman", "Liberation Serif", "Times New Roman", "Times", "DejaVu Serif"],
            "font.size": args.font_size,
            "axes.labelsize": args.font_size,
            "axes.titlesize": args.font_size,
            "xtick.labelsize": args.font_size,
            "ytick.labelsize": args.font_size,
            "legend.fontsize": args.font_size,
            "axes.linewidth": 0.8,
            "grid.linewidth": 0.45,
            "lines.linewidth": 2.0,
            "lines.solid_capstyle": "round",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
        }
    )

def set_common_xy_limits(axes: list[Any]) -> None:
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
    span = max(xmax - xmin, ymax - ymin)
    if span <= 0.0:
        span = 1.0
    pad = span * AXIS_LIMIT_PAD_FRACTION
    half = (span * 0.5) + pad
    x_mid = (xmin + xmax) * 0.5
    y_mid = (ymin + ymax) * 0.5
    for ax in axes:
        ax.set_xlim(x_mid - half, x_mid + half)
        ax.set_ylim(y_mid - half, y_mid + half)


def finish_axes(ax: Any, args: argparse.Namespace, title: str = "", legend_outside: bool = True) -> None:
    axis_labels = {
        "xy": ("x (m)", "y (m)"),
        "xz": ("x (m)", "z (m)"),
        "yz": ("y (m)", "z (m)"),
    }
    xlabel, ylabel = axis_labels[args.plane]
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True, alpha=0.28)
    if not args.no_equal_aspect:
        ax.set_aspect("equal", adjustable="box")
        set_common_xy_limits([ax])
    ax.legend(
        frameon=False, 
        loc="center",
        bbox_to_anchor=(0.5, 1.10),
        ncol=2,
        handlelength=0.75,
        handletextpad=0.25,
        columnspacing=0.5,
        borderaxespad=0.5,
    )


def save_figure(fig: Any, png_path: Path, pdf_path: Path, args: argparse.Namespace) -> None:
    png_path.parent.mkdir(parents=True, exist_ok=True)
    if args.pdf:
        pdf_path.parent.mkdir(parents=True, exist_ok=True)
    if getattr(fig, "legends", None):
        fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.90))
    else:
        fig.tight_layout()
    fig.savefig(png_path, dpi=args.dpi)
    if args.pdf:
        fig.savefig(pdf_path)


def out_paths(out_arg: str) -> tuple[Path, Path]:
    out = Path(out_arg).expanduser().resolve()
    if out.suffix.lower() in {".png", ".pdf"}:
        stem = out.with_suffix("")
    else:
        stem = out
    stem.parent.mkdir(parents=True, exist_ok=True)
    return stem.with_suffix(".png"), stem.with_suffix(".pdf")


def default_single_out(bag_dir: Path, plane: str) -> str:
    plane_suffix = "" if plane == "xy" else f"_{plane}"
    return str(bag_dir.parent / "plots" / "trajectory" / f"trajectory{plane_suffix}")


def read_paths(
    bag_dir: Path,
    topic_labels: dict[str, str],
    reference_topic: str,
    warmup_s: float,
    plane: str,
) -> dict[str, list[PathPoint]]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:
        raise SystemExit(
            "ROS bag reading requires a sourced ROS 2 environment "
            f"(rosbag2_py/rclpy unavailable): {exc}"
        )

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}

    read_topic_to_requested: dict[str, str] = {}
    requested_to_actual: dict[str, str] = {}
    for requested_topic in topic_labels:
        actual_topic = requested_topic
        if requested_topic == reference_topic and requested_topic not in type_map:
            actual_topic = next((topic for topic in REFERENCE_TOPIC_FALLBACKS if topic in type_map), requested_topic)
            if actual_topic != requested_topic:
                print(
                    f"[plot_trajectory_paths] Using reference fallback for {bag_dir}: "
                    f"{actual_topic}"
                )
        read_topic_to_requested[actual_topic] = requested_topic
        requested_to_actual[requested_topic] = actual_topic

    missing = [topic for topic, actual_topic in requested_to_actual.items() if actual_topic not in type_map]
    if missing:
        print("[plot_trajectory_paths] Warning: missing topics:", ", ".join(missing))

    msg_type_cache: dict[str, Any] = {}
    points = {topic: [] for topic in topic_labels}
    start_ns: int | None = None

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if start_ns is None:
            start_ns = int(timestamp_ns)
        rel_t = (int(timestamp_ns) - start_ns) * 1e-9
        if rel_t < warmup_s or topic not in read_topic_to_requested:
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
        requested_topic = read_topic_to_requested[topic]
        points[requested_topic].extend((rel_t, x, y, yaw) for x, y, yaw in pose_points_from_msg(msg, plane))

    return points


def plot_paths(
    topic_labels: dict[str, str],
    points: dict[str, list[PathPoint]],
    args: argparse.Namespace,
) -> None:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    png_path, pdf_path = out_paths(args.out)
    fig, ax = plt.subplots(figsize=(args.width, args.height))
    styles = [
        {"color": "black", "linewidth": 1.2, "linestyle": (0, (3, 2))},
        {"color": "#4C78A8", "linewidth": 1.2, "linestyle": (0, (3, 2))},
        {"color": "#F58518", "linewidth": 1.2, "linestyle": (0, (3, 2))},
        {"color": "#54A24B", "linewidth": 1.2, "linestyle": (0, (3, 2))},
        {"color": "#B279A2", "linewidth": 1.2, "linestyle": (0, (3, 2))},
    ]

    plotted = 0
    for idx, (topic, label) in enumerate(topic_labels.items()):
        series = points.get(topic, [])
        if not series:
            print(f"[plot_trajectory_paths] Warning: no samples for {label} ({topic})")
            continue
        plot_path_series(
            ax,
            series,
            topic=topic,
            args=args,
            label=label,
            **styles[min(idx, len(styles) - 1)],
        )
        plotted += 1

    if plotted == 0:
        raise SystemExit("No requested path topics contained plottable pose samples.")

    finish_axes(ax, args, args.title)
    save_figure(fig, png_path, pdf_path, args)
    plt.close(fig)
    print(f"png={png_path}")
    if args.pdf:
        print(f"pdf={pdf_path}")


def read_results_group(
    route: str,
    runs: list[tuple[str, str, Path]],
    topic_labels: dict[str, str],
    warmup_s: float,
    plane: str,
    args: argparse
) -> list[tuple[str, dict[str, list[PathPoint]]]]:
    run_points = []
    for _code, rep, bag_dir in runs:
        points = read_paths(bag_dir, topic_labels, args.reference_topic, warmup_s, plane)
        for topic, label in topic_labels.items():
            if not points.get(topic):
                print(
                    f"[plot_trajectory_paths] Warning: no samples for "
                    f"{route} {rep} {label} ({topic})"
                )
        run_points.append((rep, points))
    return run_points


def route_output_paths(out_dir: Path, route: str, mode: str, plane: str) -> tuple[Path, Path]:
    safe_route = re.sub(r"[^A-Za-z0-9_.-]+", "_", route).strip("_") or "route"
    plane_suffix = "" if plane == "xy" else f"_{plane}"
    stem = out_dir / f"{safe_route}_trajectories_{mode}{plane_suffix}"
    return stem.with_suffix(".png"), stem.with_suffix(".pdf")


def route_stack_output_paths(out_dir: Path, mode: str, plane: str) -> tuple[Path, Path]:
    plane_suffix = "" if plane == "xy" else f"_{plane}"
    stem = out_dir / f"all_routes_trajectories_{mode}{plane_suffix}"
    return stem.with_suffix(".png"), stem.with_suffix(".pdf")


def route_title(route: str, mode: str, args: argparse.Namespace, code: str | None = None) -> str:
    base = display_route_title(route, code)
    prefix = f"{args.title} - " if args.title else ""
    suffix = "mean reference" if mode == "average" else "all repetitions"
    return f"{prefix}{base} ({suffix})"


def short_route_label(route: str, code: str | None = None) -> str:
    letter = code or thesis_route_letter_from_name(route)
    if letter:
        return f"Route {letter}"
    return display_route_title(route)


def plot_results_group_mode(
    route: str,
    code: str,
    run_points: list[tuple[str, dict[str, list[PathPoint]]]],
    topic_labels: dict[str, str],
    args: argparse.Namespace,
    out_dir: Path,
    mode: str,
) -> None:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    fig, ax = plt.subplots(figsize=(args.width, args.height))
    colors = [
        "#4C78A8",
        "#F58518",
        "#54A24B",
        "#B279A2",
        "#E45756",
        "#72B7B2",
        "#FF9DA6",
        "#9D755D",
    ]
    plotted_any = False
    average_note = ""

    if mode == "average":
        reference_sources = [
            (rep, points.get(args.reference_topic, []))
            for rep, points in run_points
            if points.get(args.reference_topic)
        ]
        missing_reference_reps = [
            rep for rep, points in run_points if not points.get(args.reference_topic)
        ]
        if missing_reference_reps:
            print(
                f"[plot_trajectory_paths] Warning: {route} average ignores missing "
                f"{args.reference_label} reps: {', '.join(missing_reference_reps)}"
            )
        mean_path, mean_count = average_paths([path for _rep, path in reference_sources])
        count_notes = [f"reference_n={mean_count}"]
        if mean_path:
            plot_path_series(
                ax,
                mean_path,
                topic=args.reference_topic,
                args=args,
                label=f"{args.reference_label} Mean (n={mean_count})",
                color="black",
                linewidth=1.5,
                zorder=1,
            )
            plotted_any = True

        for topic_idx, (topic, label) in enumerate(topic_labels.items()):
            if topic == args.reference_topic:
                continue
            sources = [
                (rep, points.get(topic, []))
                for rep, points in run_points
                if points.get(topic)
            ]
            missing_reps = [rep for rep, points in run_points if not points.get(topic)]
            if missing_reps:
                print(
                    f"[plot_trajectory_paths] Warning: {route} average ignores missing "
                    f"{label} reps: {', '.join(missing_reps)}"
                )
            mean_series, count = average_paths([path for _rep, path in sources])
            count_notes.append(f"{label}_n={count}")
            if not mean_series:
                continue
            color = colors[(topic_idx - 1) % len(colors)]
            linewidth = 1.4
            plot_path_series(
                ax,
                mean_series,
                topic=topic,
                args=args,
                label=f"{label} Mean (n={count})",
                color=color,
                linewidth=linewidth,
                alpha=0.95,
            )
            plotted_any = True
        average_note = " " + " ".join(count_notes)
    elif mode == "normal":
        for rep_idx, (rep, points) in enumerate(run_points):
            color = colors[rep_idx % len(colors)]
            for topic_idx, (topic, label) in enumerate(topic_labels.items()):
                series = points.get(topic, [])
                if not series:
                    continue
                if topic == args.reference_topic:
                    line_color = "#333333"
                    linewidth = 1.1
                    alpha = 0.55
                else:
                    line_color = color
                    linewidth = 1.2
                    alpha = 0.9
                plot_path_series(
                    ax,
                    series,
                    topic=topic,
                    args=args,
                    label=f"{label} {display_rep_label(rep)}",
                    color=line_color,
                    linewidth=linewidth,
                    alpha=alpha,
                )
                plotted_any = True
    else:
        raise ValueError(f"unknown plot mode: {mode}")

    if not plotted_any:
        plt.close(fig)
        print(f"[plot_trajectory_paths] Warning: no plottable samples for {route} ({mode})")
        return

    finish_axes(ax, args, route_title(route, mode, args, code), legend_outside=True)
    png_path, pdf_path = route_output_paths(out_dir, route, mode, args.plane)
    save_figure(fig, png_path, pdf_path, args)
    plt.close(fig)
    print(f"route={route} reps={len(run_points)} mode={mode}{average_note}")
    print(f"png={png_path}")
    if args.pdf:
        print(f"pdf={pdf_path}")


def plot_results_group(
    route: str,
    runs: list[tuple[str, str, Path]],
    topic_labels: dict[str, str],
    args: argparse.Namespace,
    out_dir: Path,
) -> None:
    run_points = read_results_group(route, runs, topic_labels, args.warmup, args.plane, args)
    code = runs[0][0] if runs else thesis_route_letter_from_name(route) or route
    modes = ["average", "normal"] if args.batch_plots == "both" else [args.batch_plots]
    for mode in modes:
        plot_results_group_mode(route, code, run_points, topic_labels, args, out_dir, mode)


def plot_stacked_routes(
    route_points: dict[str, tuple[str, list[tuple[str, dict[str, list[PathPoint]]]]]],
    args: argparse.Namespace,
    out_dir: Path,
) -> None:
    configure_plot_style(args)
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    colors = [
        "#4C78A8",
        "#F58518",
        "#54A24B",
        "#B279A2",
        "#E45756",
        "#72B7B2",
        "#FF9DA6",
        "#9D755D",
        "#BAB0AC",
    ]

    stacked_width = max(args.width * 2.0, WIDE_FIGURE_WIDTH_IN)
    stacked_height = max(args.height, WIDE_FIGURE_HEIGHT_IN)
    fig, axes = plt.subplots(1, 2, figsize=(stacked_width, stacked_height), sharex=False, sharey=False)
    stack_specs = [
        (axes[0], args.reference_topic, "UGV", 1.4),
        (axes[1], args.uav_topic, "UAV", 1.4),
    ]
    route_handles: list[Any] = []
    route_labels: list[str] = []
    plotted_total = 0

    for idx, (route, (code, run_points)) in enumerate(route_points.items()):
        color = colors[idx % len(colors)]
        route_label = short_route_label(route, code)
        route_plotted = False
        for ax, topic, panel_label, linewidth in stack_specs:
            paths = [points.get(topic, []) for _rep, points in run_points if points.get(topic)]
            mean_path, _count = average_paths(paths)
            if not mean_path:
                continue
            plot_path_series(
                ax,
                mean_path,
                topic=topic,
                args=args,
                markers=False,
                color=color,
                linewidth=linewidth,
                alpha=0.95,
            )
            plotted_total += 1
            route_plotted = True
        if route_plotted:
            route_handles.append(
                axes[1].plot(
                    [],
                    [],
                    color=color,
                    linestyle="-",
                    linewidth=1.4,
                )[0]
            )
            route_labels.append(route_label)

    if plotted_total == 0:
        plt.close(fig)
        print("[plot_trajectory_paths] Warning: no plottable samples for stacked route overview")
        return

    axis_labels = {
        "xy": ("x (m)", "y (m)"),
        "xz": ("x (m)", "z (m)"),
        "yz": ("y (m)", "z (m)"),
    }
    xlabel, ylabel = axis_labels[args.plane]
    for ax, _topic, panel_label, _linewidth in stack_specs:
        ax.set_title(panel_label, pad=8)
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        ax.grid(True, alpha=0.28)
        if not args.no_equal_aspect:
            ax.set_aspect("equal", adjustable="box")
    set_common_xy_limits(list(axes))

    if route_handles:
        ncol = min(len(route_handles), 9)
        fig.legend(
            route_handles,
            route_labels,
            frameon=False,
            loc="upper center",
            bbox_to_anchor=(0.5, 1.03),
            ncol=ncol,
            handlelength=1.0,
            handletextpad=0.25,
            columnspacing=0.75,
            borderaxespad=0.75,
        )

    png_path, pdf_path = route_stack_output_paths(out_dir, "stacked", args.plane)
    save_figure(fig, png_path, pdf_path, args)
    plt.close(fig)
    print(f"stacked_routes={len(route_handles)}")
    print(f"png={png_path}")
    if args.pdf:
        print(f"pdf={pdf_path}")


def main() -> None:
    args = parse_args()
    topic_labels: dict[str, str] = {
        args.reference_topic: args.reference_label,
        args.uav_topic: args.uav_label,
    }
    for item in args.estimated_topic:
        label, topic = parse_series_arg(item, "Estimated")
        topic_labels[topic] = label
    for item in args.extra_topic:
        label, topic = parse_series_arg(item, "Extra")
        topic_labels[topic] = label

    if args.results_dir:
        results_dir = Path(args.results_dir).expanduser().resolve()
        if not results_dir.is_dir():
            raise SystemExit(f"Results directory does not exist: {results_dir}")
        groups = discover_results_runs(results_dir)
        rep_filter = selected_reps(args.reps)
        if rep_filter:
            groups = {
                route: [
                    (code, rep, bag_dir)
                    for code, rep, bag_dir in runs
                    if normalize_rep_label(rep) in rep_filter
                ]
                for route, runs in groups.items()
            }
            groups = {route: runs for route, runs in groups.items() if runs}
        if not groups:
            raise SystemExit(f"No repXX/RNN_route/bag directories found under: {results_dir}")
        out_dir = batch_out_dir(args, results_dir)
        route_points: dict[str, tuple[str, list[tuple[str, dict[str, list[PathPoint]]]]]] = {}
        for route, runs in groups.items():
            run_points = read_results_group(route, runs, topic_labels, args.warmup, args.plane, args)
            code = runs[0][0] if runs else thesis_route_letter_from_name(route) or route
            route_points[route] = (code, run_points)
            modes = ["average", "normal"] if args.batch_plots == "both" else [args.batch_plots]
            for mode in modes:
                plot_results_group_mode(route, code, run_points, topic_labels, args, out_dir, mode)
        if args.stack_routes:
            plot_stacked_routes(route_points, args, out_dir)
        return

    bag_dir = resolve_bag_dir(args)
    if not bag_dir.is_dir():
        raise SystemExit(f"Bag directory does not exist: {bag_dir}")
    if not args.out:
        args.out = default_single_out(bag_dir, args.plane)

    points = read_paths(bag_dir, topic_labels, args.reference_topic, args.warmup, args.plane)

    plot_paths(topic_labels, points, args)
    


if __name__ == "__main__":
    main()
