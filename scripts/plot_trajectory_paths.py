#!/usr/bin/env python3
"""Plot x-y trajectories from a ROS 2 bag run folder.

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
            "--results-dir is used. Both PNG and PDF are written."
        ),
    )
    parser.add_argument("--dpi", type=int, default=220)
    parser.add_argument("--width", type=float, default=6.5, help="Figure width in inches.")
    parser.add_argument("--height", type=float, default=5.0, help="Figure height in inches.")
    parser.add_argument(
        "--no-equal-aspect",
        action="store_true",
        help="Do not force equal x/y aspect ratio.",
    )
    args = parser.parse_args()
    if not args.results_dir and not args.out:
        parser.error("--out is required unless --results-dir is used")
    return args


def yaw_from_quat(q: Any) -> float:
    x = float(getattr(q, "x", 0.0))
    y = float(getattr(q, "y", 0.0))
    z = float(getattr(q, "z", 0.0))
    w = float(getattr(q, "w", 1.0))
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def pose_xy_from_msg(msg: Any) -> list[tuple[float, float]]:
    """Return one or more x-y points from common pose-like ROS messages."""
    try:
        if hasattr(msg, "poses"):
            points: list[tuple[float, float]] = []
            for pose_stamped in msg.poses:
                p = pose_stamped.pose.position
                points.append((float(p.x), float(p.y)))
            return points
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        p = pose.position
        return [(float(p.x), float(p.y))]
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
    """Return a stable route label from names like R01_rotundan."""
    name = run_dir.name
    return re.sub(r"^R\d+_", "", name)


def rep_label_from_run_dir(run_dir: Path) -> str:
    for parent in run_dir.parents:
        if re.fullmatch(r"rep\d+", parent.name):
            return parent.name
    return run_dir.parent.name


def discover_results_runs(results_dir: Path) -> dict[str, list[tuple[str, Path]]]:
    groups: dict[str, list[tuple[str, Path]]] = {}
    for bag_dir in sorted(results_dir.glob("rep*/R*/bag")):
        if not bag_dir.is_dir():
            continue
        if not (bag_dir / "metadata.yaml").is_file() and not list(bag_dir.glob("bag_*.mcap")):
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
) -> list[tuple[float, float]]:
    resampled = [resample_path(path, samples) for path in paths if path]
    if not resampled:
        return []
    out: list[tuple[float, float]] = []
    for idx in range(samples):
        xs = [path[idx][0] for path in resampled if idx < len(path)]
        ys = [path[idx][1] for path in resampled if idx < len(path)]
        if xs and ys:
            out.append((sum(xs) / len(xs), sum(ys) / len(ys)))
    return out


def batch_out_dir(args: argparse.Namespace, results_dir: Path) -> Path:
    out = Path(args.out).expanduser().resolve() if args.out else results_dir / "plots"
    out.mkdir(parents=True, exist_ok=True)
    return out


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
        points[topic].extend(pose_xy_from_msg(msg))

    return points


def plot_paths(
    topic_labels: dict[str, str],
    points: dict[str, list[tuple[float, float]]],
    args: argparse.Namespace,
) -> None:
    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required: {exc}")

    png_path, pdf_path = out_paths(args.out)
    fig, ax = plt.subplots(figsize=(args.width, args.height))
    styles = [
        {"color": "black", "linewidth": 2.0, "linestyle": "--"},
        {"color": "#4C78A8", "linewidth": 1.8, "linestyle": "-"},
        {"color": "#F58518", "linewidth": 1.8, "linestyle": "-"},
        {"color": "#54A24B", "linewidth": 1.8, "linestyle": "-"},
        {"color": "#B279A2", "linewidth": 1.8, "linestyle": "-"},
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

    if args.title:
        ax.set_title(args.title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.25, linewidth=0.6)
    if not args.no_equal_aspect:
        ax.set_aspect("equal", adjustable="box")
    ax.legend(frameon=False, loc="best")
    fig.tight_layout()
    fig.savefig(png_path, dpi=args.dpi)
    #fig.savefig(pdf_path)
    plt.close(fig)
    print(f"png={png_path}")
    #print(f"pdf={pdf_path}")


def plot_results_group(
    route: str,
    runs: list[tuple[str, Path]],
    topic_labels: dict[str, str],
    args: argparse.Namespace,
    out_dir: Path,
) -> None:
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
    linestyles = ["-", "--", ":", "-."]
    reference_paths: list[list[tuple[float, float]]] = []
    plotted_any = False

    for idx, (rep, bag_dir) in enumerate(runs):
        color = colors[idx % len(colors)]
        points = read_paths(bag_dir, topic_labels, args.warmup)
        reference_series = points.get(args.reference_topic, [])
        if reference_series:
            reference_paths.append(reference_series)
        else:
            print(
                f"[plot_trajectory_paths] Warning: no samples for "
                f"{route} {rep} {args.reference_label} ({args.reference_topic})"
            )

        for topic_idx, (topic, label) in enumerate(topic_labels.items()):
            if topic == args.reference_topic:
                continue
            series = points.get(topic, [])
            if not series:
                print(
                    f"[plot_trajectory_paths] Warning: no samples for "
                    f"{route} {rep} {label} ({topic})"
                )
                continue
            xs = [p[0] for p in series]
            ys = [p[1] for p in series]
            ax.plot(
                xs,
                ys,
                label=f"{label} {rep}",
                color=color,
                linewidth=1.6,
                linestyle=linestyles[topic_idx % len(linestyles)],
                alpha=0.85,
            )
            plotted_any = True

    mean_reference = average_paths(reference_paths)
    if mean_reference:
        xs = [p[0] for p in mean_reference]
        ys = [p[1] for p in mean_reference]
        ax.plot(xs, ys, label="UGV mean", color="black", linewidth=2.4)
        plotted_any = True

    if not plotted_any:
        plt.close(fig)
        print(f"[plot_trajectory_paths] Warning: no plottable samples for {route}")
        return

    title = f"{args.title} - {route}" if args.title else route
    ax.set_title(title)
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(True, alpha=0.25, linewidth=0.6)
    if not args.no_equal_aspect:
        ax.set_aspect("equal", adjustable="box")
    ax.legend(frameon=False, loc="center left", bbox_to_anchor=(1.02, 0.5))
    fig.tight_layout()
    safe_route = re.sub(r"[^A-Za-z0-9_.-]+", "_", route).strip("_") or "route"
    png_path = out_dir / f"{safe_route}_trajectories.png"
    #pdf_path = out_dir / f"{safe_route}_trajectories.pdf"
    fig.savefig(png_path, dpi=args.dpi, bbox_inches="tight")
    #fig.savefig(pdf_path, bbox_inches="tight")
    plt.close(fig)
    print(f"route={route} reps={len(runs)}")
    print(f"png={png_path}")
    #print(f"pdf={pdf_path}")


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
        if not groups:
            raise SystemExit(f"No repXX/RNN_route/bag directories found under: {results_dir}")
        out_dir = batch_out_dir(args, results_dir)
        for route, runs in groups.items():
            plot_results_group(route, runs, topic_labels, args, out_dir)
        return

    bag_dir = resolve_bag_dir(args)
    if not bag_dir.is_dir():
        raise SystemExit(f"Bag directory does not exist: {bag_dir}")

    points = read_paths(bag_dir, topic_labels, args.warmup)
    plot_paths(topic_labels, points, args)


if __name__ == "__main__":
    main()
