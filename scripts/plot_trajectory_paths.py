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
    parser.add_argument("--reference-label", default="Reference / UGV")
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
    parser.add_argument("--width", type=float, default=6.5, help="Figure width in inches.")
    parser.add_argument("--height", type=float, default=5.0, help="Figure height in inches.")
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
    args = parser.parse_args()
    if args.average is not None:
        args.batch_plots = "average" if args.average else "normal"
    if args.out and Path(args.out).suffix.lower() == ".pdf":
        args.pdf = True
    if not args.results_dir and not args.out:
        parser.error("--out is required unless --results-dir is used")
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


def pose_points_from_msg(msg: Any, plane: str) -> list[tuple[float, float]]:
    """Return one or more 2D projected points from common pose-like ROS messages."""
    try:
        if hasattr(msg, "poses"):
            points: list[tuple[float, float]] = []
            for pose_stamped in msg.poses:
                p = pose_stamped.pose.position
                points.append(point_from_position(p, plane))
            return points
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        p = pose.position
        return [point_from_position(p, plane)]
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


def rep_label_from_run_dir(run_dir: Path) -> str:
    selected_match = re.match(r"^(C\d+)_Route[A-Z]_.+?_(r\d+)__", run_dir.name)
    if selected_match:
        return f"{selected_match.group(1)}_{selected_match.group(2)}"
    for parent in run_dir.parents:
        if re.fullmatch(r"rep\d+", parent.name):
            return parent.name
    return run_dir.parent.name


def discover_results_runs(results_dir: Path) -> dict[str, list[tuple[str, Path]]]:
    groups: dict[str, list[tuple[str, Path]]] = {}
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
        groups.setdefault(route, []).append((rep, bag_dir))
    for runs in groups.values():
        runs.sort(key=lambda item: item[0])
    return dict(sorted(groups.items()))


def resample_path(
    series: list[tuple[float, float]],
    samples: int,
) -> list[tuple[float, float]]:
    if not series:
        return []
    if samples <= 1 or len(series) == 1:
        return [series[0]]

    distances = [0.0]
    for prev, cur in zip(series, series[1:]):
        distances.append(distances[-1] + math.hypot(cur[0] - prev[0], cur[1] - prev[1]))

    total = distances[-1]
    if total <= 0.0:
        return [series[0]] * samples

    out: list[tuple[float, float]] = []
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
        out.append((p0[0] + (p1[0] - p0[0]) * ratio, p0[1] + (p1[1] - p0[1]) * ratio))
    return out


def average_paths(
    paths: list[list[tuple[float, float]]],
    samples: int = 300,
) -> tuple[list[tuple[float, float]], int]:
    resampled = [resample_path(path, samples) for path in paths if path]
    if not resampled:
        return [], 0
    out: list[tuple[float, float]] = []
    for idx in range(samples):
        xs = [path[idx][0] for path in resampled if idx < len(path)]
        ys = [path[idx][1] for path in resampled if idx < len(path)]
        if xs and ys:
            out.append((sum(xs) / len(xs), sum(ys) / len(ys)))
    return out, len(resampled)


def batch_out_dir(args: argparse.Namespace, results_dir: Path) -> Path:
    out = Path(args.out).expanduser().resolve() if args.out else results_dir / "plots"
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
            "font.serif": ["DejaVu Serif", "Times New Roman", "Times"],
            "font.size": args.font_size,
            "axes.labelsize": args.font_size,
            "axes.titlesize": args.font_size,
            "xtick.labelsize": args.font_size,
            "ytick.labelsize": args.font_size,
            "legend.fontsize": args.font_size,
            "axes.linewidth": 0.8,
            "grid.linewidth": 0.45,
            "lines.solid_capstyle": "round",
            "pdf.fonttype": 42,
            "ps.fonttype": 42,
            "savefig.bbox": "tight",
        }
    )


def set_equal_xy_limits(ax: Any) -> None:
    xmin, xmax = ax.get_xlim()
    ymin, ymax = ax.get_ylim()
    if not all(math.isfinite(value) for value in (xmin, xmax, ymin, ymax)):
        return
    x_span = xmax - xmin
    y_span = ymax - ymin
    span = max(x_span, y_span)
    if span <= 0.0:
        span = 1.0
    pad = span * 0.06
    half = (span * 0.5) + pad
    ax.set_xlim(((xmin + xmax) * 0.5) - half, ((xmin + xmax) * 0.5) + half)
    ax.set_ylim(((ymin + ymax) * 0.5) - half, ((ymin + ymax) * 0.5) + half)


def finish_axes(ax: Any, args: argparse.Namespace, title: str = "", legend_outside: bool = False) -> None:
    if title:
        ax.set_title(title, pad=8)
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
        set_equal_xy_limits(ax)
    if legend_outside:
        ax.legend(frameon=False, loc="center left", bbox_to_anchor=(1.02, 0.5))
    else:
        ax.legend(frameon=False, loc="best")


def save_figure(fig: Any, png_path: Path, pdf_path: Path, args: argparse.Namespace) -> None:
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


def read_paths(
    bag_dir: Path,
    topic_labels: dict[str, str],
    warmup_s: float,
    plane: str,
) -> dict[str, list[tuple[float, float]]]:
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
    missing = [topic for topic in topic_labels if topic not in type_map]
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
        if rel_t < warmup_s or topic not in topic_labels:
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
        points[topic].extend(pose_points_from_msg(msg, plane))

    return points


def plot_paths(
    topic_labels: dict[str, str],
    points: dict[str, list[tuple[float, float]]],
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
        {"color": "black", "linewidth": 2.8, "linestyle": "-"},
        {"color": "#4C78A8", "linewidth": 2.3, "linestyle": "-"},
        {"color": "#F58518", "linewidth": 2.3, "linestyle": "-"},
        {"color": "#54A24B", "linewidth": 2.3, "linestyle": "-"},
        {"color": "#B279A2", "linewidth": 2.3, "linestyle": "-"},
    ]

    plotted = 0
    for idx, (topic, label) in enumerate(topic_labels.items()):
        series = points.get(topic, [])
        if not series:
            print(f"[plot_trajectory_paths] Warning: no samples for {label} ({topic})")
            continue
        xs = [p[0] for p in series]
        ys = [p[1] for p in series]
        ax.plot(xs, ys, label=label, **styles[min(idx, len(styles) - 1)])
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
    runs: list[tuple[str, Path]],
    topic_labels: dict[str, str],
    warmup_s: float,
    plane: str,
) -> list[tuple[str, dict[str, list[tuple[float, float]]]]]:
    run_points = []
    for rep, bag_dir in runs:
        points = read_paths(bag_dir, topic_labels, warmup_s, plane)
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


def route_title(route: str, mode: str, args: argparse.Namespace) -> str:
    base = route.replace("_", " ").title()
    prefix = f"{args.title} - " if args.title else ""
    suffix = "mean reference" if mode == "average" else "all repetitions"
    return f"{prefix}{base} ({suffix})"


def plot_results_group_mode(
    route: str,
    run_points: list[tuple[str, dict[str, list[tuple[float, float]]]]],
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
    topic_linestyles = ["-", "--", ":", "-."]
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
            ax.plot(
                [p[0] for p in mean_path],
                [p[1] for p in mean_path],
                label=f"{args.reference_label} mean (n={mean_count})",
                color="black",
                linewidth=3.0,
                linestyle="-",
                zorder=4,
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
            ax.plot(
                [p[0] for p in mean_series],
                [p[1] for p in mean_series],
                label=f"{label} mean (n={count})",
                color=color,
                linewidth=2.6,
                linestyle=topic_linestyles[topic_idx % len(topic_linestyles)],
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
                    linewidth = 1.8
                    alpha = 0.55
                else:
                    line_color = color
                    linewidth = 2.2
                    alpha = 0.9
                ax.plot(
                    [p[0] for p in series],
                    [p[1] for p in series],
                    label=f"{label} {rep}",
                    color=line_color,
                    linewidth=linewidth,
                    linestyle=topic_linestyles[topic_idx % len(topic_linestyles)],
                    alpha=alpha,
                )
                plotted_any = True
    else:
        raise ValueError(f"unknown plot mode: {mode}")

    if not plotted_any:
        plt.close(fig)
        print(f"[plot_trajectory_paths] Warning: no plottable samples for {route} ({mode})")
        return

    finish_axes(ax, args, route_title(route, mode, args), legend_outside=True)
    png_path, pdf_path = route_output_paths(out_dir, route, mode, args.plane)
    save_figure(fig, png_path, pdf_path, args)
    plt.close(fig)
    print(f"route={route} reps={len(run_points)} mode={mode}{average_note}")
    print(f"png={png_path}")
    if args.pdf:
        print(f"pdf={pdf_path}")


def plot_results_group(
    route: str,
    runs: list[tuple[str, Path]],
    topic_labels: dict[str, str],
    args: argparse.Namespace,
    out_dir: Path,
) -> None:
    run_points = read_results_group(route, runs, topic_labels, args.warmup, args.plane)
    modes = ["average", "normal"] if args.batch_plots == "both" else [args.batch_plots]
    for mode in modes:
        plot_results_group_mode(route, run_points, topic_labels, args, out_dir, mode)


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
                    (rep, bag_dir)
                    for rep, bag_dir in runs
                    if normalize_rep_label(rep) in rep_filter
                ]
                for route, runs in groups.items()
            }
            groups = {route: runs for route, runs in groups.items() if runs}
        if not groups:
            raise SystemExit(f"No repXX/RNN_route/bag directories found under: {results_dir}")
        out_dir = batch_out_dir(args, results_dir)
        for route, runs in groups.items():
            plot_results_group(route, runs, topic_labels, args, out_dir)
        return

    bag_dir = resolve_bag_dir(args)
    if not bag_dir.is_dir():
        raise SystemExit(f"Bag directory does not exist: {bag_dir}")

    points = read_paths(bag_dir, topic_labels, args.warmup, args.plane)
    plot_paths(topic_labels, points, args)


if __name__ == "__main__":
    main()
