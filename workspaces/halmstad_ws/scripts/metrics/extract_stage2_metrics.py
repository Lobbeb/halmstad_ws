#!/usr/bin/env python3
import argparse
import csv
import math
import os
from pathlib import Path

import rclpy.serialization
from rosidl_runtime_py.utilities import get_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, TopicMetadata


def read_messages(bag_path: Path, topics_of_interest: set):
    """
    Yields tuples: (topic, msg, t_sec)
    t_sec is bag timestamp in seconds (float).
    """
    storage_options = StorageOptions(uri=str(bag_path), storage_id="")  # auto-detect
    converter_options = ConverterOptions(
        input_serialization_format="cdr",
        output_serialization_format="cdr",
    )
    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    # Build type map
    topic_types = {}
    for t in reader.get_all_topics_and_types():
        # t is TopicMetadata(name, type, serialization_format, offered_qos_profiles)
        topic_types[t.name] = t.type

    # Ensure topics exist
    missing = [t for t in topics_of_interest if t not in topic_types]
    return reader, topic_types, missing


def extract_xy_from_odom(msg):
    p = msg.pose.pose.position
    return float(p.x), float(p.y)


def extract_xy_from_pose_stamped(msg):
    p = msg.pose.position
    return float(p.x), float(p.y)


def nearest_neighbor_align(t_a, xya, t_b, xyb, max_dt=0.10):
    """
    Align series A to B by nearest timestamp within max_dt.
    Returns list of (t, ax, ay, bx, by) aligned on A timestamps.
    Assumes t_a and t_b sorted.
    """
    out = []
    j = 0
    n_b = len(t_b)
    for i, ta in enumerate(t_a):
        # advance j while next is closer
        while j + 1 < n_b and abs(t_b[j + 1] - ta) <= abs(t_b[j] - ta):
            j += 1
        if n_b == 0:
            break
        if abs(t_b[j] - ta) <= max_dt:
            ax, ay = xya[i]
            bx, by = xyb[j]
            out.append((ta, ax, ay, bx, by))
    return out


def rate_hz(count, t0, t1):
    dt = max(1e-9, (t1 - t0))
    return float(count) / dt


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs_root", default=str(Path.home() / "halmstad_ws" / "runs"))
    ap.add_argument("--uav", default="dji0")
    ap.add_argument("--bag_dirname", default="bag", help="subfolder name inside each run (default: bag)")
    ap.add_argument("--d_target", type=float, default=3.0)
    ap.add_argument("--d_max", type=float, default=8.0)
    ap.add_argument("--eps", type=float, default=0.1)
    ap.add_argument("--align_dt", type=float, default=0.10)
    ap.add_argument("--out_summary", default="")
    args = ap.parse_args()

    runs_root = Path(args.runs_root)
    out_summary = Path(args.out_summary) if args.out_summary else (runs_root / "stage2_baseline_summary.csv")

    odom_topic = "/a201_0000/platform/odom"
    pose_cmd_topic = f"/{args.uav}/pose_cmd"
    events_topic = "/coord/events"

    topics = {odom_topic, pose_cmd_topic, events_topic}

    summary_rows = []

    run_dirs = sorted([p for p in runs_root.iterdir() if p.is_dir() and p.name.startswith("run_")])

    for run_dir in run_dirs:
        bag_path = run_dir / args.bag_dirname
        if not bag_path.exists():
            # try to auto-detect a bag directory
            candidates = [p for p in run_dir.iterdir() if p.is_dir() and ("bag" in p.name or "rosbag2" in p.name)]
            if candidates:
                bag_path = sorted(candidates)[0]
            else:
                print(f"[WARN] {run_dir.name}: no bag directory found, skipping")
                continue

        # Read messages
        reader, topic_types, missing = read_messages(bag_path, topics)
        if missing:
            print(f"[WARN] {run_dir.name}: missing topics {missing}, skipping")
            continue

        # Prepare message classes
        msg_cls = {name: get_message(typ) for name, typ in topic_types.items() if name in topics}

        t_odom, xy_odom = [], []
        t_cmd, xy_cmd = [], []
        t_events = []
        follow_tick = 0
        stale_hold = 0
        leash_clamped = 0

        # Iterate bag
        while reader.has_next():
            topic, data, t_ns = reader.read_next()
            if topic not in topics:
                continue
            t_sec = float(t_ns) * 1e-9
            msg = rclpy.serialization.deserialize_message(data, msg_cls[topic])

            if topic == odom_topic:
                t_odom.append(t_sec)
                xy_odom.append(extract_xy_from_odom(msg))
            elif topic == pose_cmd_topic:
                t_cmd.append(t_sec)
                xy_cmd.append(extract_xy_from_pose_stamped(msg))
            elif topic == events_topic:
                t_events.append(t_sec)
                s = msg.data
                if "FOLLOW_TICK" in s:
                    follow_tick += 1
                if "POSE_STALE_HOLD" in s:
                    stale_hold += 1
                if "LEASH_CLAMPED" in s:
                    leash_clamped += 1

        if len(t_odom) == 0 or len(t_cmd) == 0:
            print(f"[WARN] {run_dir.name}: no odom or no pose_cmd, skipping")
            continue

        # Compute run duration (use overlap window)
        t0 = max(min(t_odom), min(t_cmd))
        t1 = min(max(t_odom), max(t_cmd))
        if t1 <= t0:
            print(f"[WARN] {run_dir.name}: no overlapping time window, skipping")
            continue

        # Trim to overlap
        odom_trim = [(t, xy) for t, xy in zip(t_odom, xy_odom) if t0 <= t <= t1]
        cmd_trim  = [(t, xy) for t, xy in zip(t_cmd,  xy_cmd)  if t0 <= t <= t1]

        t_odom2 = [t for t, _ in odom_trim]
        xy_odom2 = [xy for _, xy in odom_trim]
        t_cmd2 = [t for t, _ in cmd_trim]
        xy_cmd2 = [xy for _, xy in cmd_trim]

        aligned = nearest_neighbor_align(t_cmd2, xy_cmd2, t_odom2, xy_odom2, max_dt=args.align_dt)
        if len(aligned) < 10:
            print(f"[WARN] {run_dir.name}: too few aligned samples ({len(aligned)}), skipping")
            continue

        # Distances + errors
        dists = []
        errs = []
        violations = 0

        for (t, ux, uy, gx, gy) in aligned:
            d = math.hypot(ux - gx, uy - gy)
            e = d - args.d_target
            dists.append(d)
            errs.append(e)
            if d > (args.d_max + args.eps):
                violations += 1

        n = len(errs)
        mean_abs_err = sum(abs(e) for e in errs) / n
        max_abs_err = max(abs(e) for e in errs)
        rmse_err = math.sqrt(sum(e * e for e in errs) / n)

        max_dist = max(dists)
        viol_pct = 100.0 * violations / n

        # Rates
        pose_cmd_rate = rate_hz(len(t_cmd2), t0, t1)
        follow_tick_rate = rate_hz(follow_tick, t0, t1)

        row = {
            "run_id": run_dir.name,
            "bag_path": str(bag_path),
            "uav": args.uav,
            "d_target": args.d_target,
            "d_max": args.d_max,
            "align_dt_s": args.align_dt,
            "duration_s": (t1 - t0),
            "mean_abs_err_m": mean_abs_err,
            "max_abs_err_m": max_abs_err,
            "rmse_err_m": rmse_err,
            "max_dist_m": max_dist,
            "leash_violation_pct": viol_pct,
            "pose_cmd_rate_hz": pose_cmd_rate,
            "follow_tick_rate_hz": follow_tick_rate,
            "stale_hold_count": stale_hold,
            "leash_clamped_count": leash_clamped,
            "follow_tick_count": follow_tick,
            "aligned_samples": n,
        }
        summary_rows.append(row)

        # Per-run metrics.csv
        per_run_csv = run_dir / "metrics.csv"
        with per_run_csv.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(row.keys()))
            w.writeheader()
            w.writerow(row)

        print(f"[OK] {run_dir.name}: wrote {per_run_csv}")

    # Summary CSV
    if summary_rows:
        with out_summary.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
            w.writeheader()
            for r in summary_rows:
                w.writerow(r)
        print(f"[OK] wrote summary: {out_summary}")
    else:
        print("[WARN] no runs processed successfully")


if __name__ == "__main__":
    main()
