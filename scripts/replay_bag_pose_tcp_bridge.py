#!/usr/bin/env python3
"""Serve recorded UGV/UAV bag poses using the OMNeT pose TCP protocol."""

from __future__ import annotations

import argparse
import bisect
import math
import socketserver
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


@dataclass(frozen=True)
class PoseSample:
    t: float
    x: float
    y: float
    z: float
    yaw: float


class PoseTrack:
    def __init__(self, samples: list[PoseSample]):
        if not samples:
            raise ValueError("pose track has no samples")
        self.samples = samples
        self.times = [sample.t for sample in samples]

    def at(self, t: float) -> PoseSample:
        idx = bisect.bisect_left(self.times, t)
        if idx <= 0:
            return self.samples[0]
        if idx >= len(self.samples):
            return self.samples[-1]
        before = self.samples[idx - 1]
        after = self.samples[idx]
        if abs(before.t - t) <= abs(after.t - t):
            return before
        return after


class PoseReplayState:
    def __init__(self, tracks: dict[str, PoseTrack], speed: float, hold_last: bool):
        self.tracks = tracks
        self.speed = speed
        self.hold_last = hold_last
        self.started_at = time.monotonic()
        self.duration = max(track.samples[-1].t for track in tracks.values())

    def replay_time(self) -> float:
        t = max(0.0, (time.monotonic() - self.started_at) * self.speed)
        if self.hold_last:
            return min(t, self.duration)
        return t

    def snapshot_line(self) -> str:
        t = self.replay_time()
        if not self.hold_last and t > self.duration:
            return "0"
        parts = [str(len(self.tracks))]
        for name in sorted(self.tracks):
            sample = self.tracks[name].at(t)
            parts.extend(
                [
                    name,
                    f"{sample.x:.6f}",
                    f"{sample.y:.6f}",
                    f"{sample.z:.6f}",
                    f"{sample.yaw:.6f}",
                ]
            )
        return " ".join(parts)


class PoseTcpServer(socketserver.ThreadingTCPServer):
    allow_reuse_address = True
    daemon_threads = True

    def __init__(self, address: tuple[str, int], state: PoseReplayState):
        self.state = state
        super().__init__(address, PoseRequestHandler)


class PoseRequestHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        while True:
            raw = self.rfile.readline()
            if not raw:
                break
            command = raw.decode("ascii", errors="ignore").strip().upper()
            if command == "GET":
                self.wfile.write((self.server.state.snapshot_line() + "\n").encode("ascii"))
            else:
                self.wfile.write(b"ERR unsupported command\n")
            self.wfile.flush()


def yaw_from_quaternion(q: Any) -> float:
    qx = float(q.x)
    qy = float(q.y)
    qz = float(q.z)
    qw = float(q.w)
    qnorm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if qnorm > 0.0:
        qx /= qnorm
        qy /= qnorm
        qz /= qnorm
        qw /= qnorm
    fx = 1.0 - 2.0 * (qy * qy + qz * qz)
    fy = 2.0 * (qx * qy + qw * qz)
    return math.atan2(fy, fx)


def pose_from_msg(msg: Any) -> tuple[Any, Any] | None:
    try:
        pose = msg.pose.pose if hasattr(msg.pose, "pose") else msg.pose
        return pose.position, pose.orientation
    except Exception:
        return None


def read_tracks(
    bag_dir: Path,
    topic_to_model: dict[str, str],
    start_s: float,
    duration_s: float | None,
) -> dict[str, PoseTrack]:
    try:
        import rosbag2_py
        from rclpy.serialization import deserialize_message
        from rosidl_runtime_py.utilities import get_message
    except Exception as exc:
        raise SystemExit(
            "ROS bag reading requires a sourced ROS 2 environment "
            f"(rosbag2_py/rclpy unavailable): {exc}"
        ) from exc

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id=""),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
    )
    type_map = {topic.name: topic.type for topic in reader.get_all_topics_and_types()}
    missing = sorted(topic for topic in topic_to_model if topic not in type_map)
    if missing:
        raise SystemExit(f"Bag is missing required pose topics: {', '.join(missing)}")

    msg_type_cache: dict[str, Any] = {}
    samples_by_model: dict[str, list[PoseSample]] = {model: [] for model in topic_to_model.values()}
    first_ns: int | None = None
    end_s = start_s + duration_s if duration_s is not None else None

    while reader.has_next():
        topic, data, timestamp_ns = reader.read_next()
        if topic not in topic_to_model:
            continue
        timestamp_ns = int(timestamp_ns)
        if first_ns is None:
            first_ns = timestamp_ns
        rel_t = (timestamp_ns - first_ns) * 1e-9
        if rel_t < start_s:
            continue
        if end_s is not None and rel_t > end_s:
            continue

        msg_type_name = type_map[topic]
        if msg_type_name not in msg_type_cache:
            msg_type_cache[msg_type_name] = get_message(msg_type_name)
        msg = deserialize_message(data, msg_type_cache[msg_type_name])
        pose = pose_from_msg(msg)
        if pose is None:
            continue
        position, orientation = pose
        samples_by_model[topic_to_model[topic]].append(
            PoseSample(
                t=rel_t - start_s,
                x=float(position.x),
                y=float(position.y),
                z=float(position.z),
                yaw=yaw_from_quaternion(orientation),
            )
        )

    tracks: dict[str, PoseTrack] = {}
    for model, samples in samples_by_model.items():
        if not samples:
            raise SystemExit(f"No usable pose samples for model '{model}'")
        tracks[model] = PoseTrack(samples)
    return tracks


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, help="ROS 2 bag directory")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=5555)
    parser.add_argument("--ugv-topic", default="/a201_0000/ground_truth/odom")
    parser.add_argument("--uav-topic", default="/dji0/pose")
    parser.add_argument("--ugv-model", default="robot")
    parser.add_argument("--uav-model", default="dji0")
    parser.add_argument("--speed", type=float, default=1.0, help="Replay speed relative to bag time")
    parser.add_argument("--start-s", type=float, default=0.0)
    parser.add_argument("--duration-s", type=float)
    parser.add_argument("--no-hold-last", action="store_true")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if args.speed <= 0.0:
        raise SystemExit("--speed must be > 0")
    bag_dir = Path(args.bag).expanduser().resolve()
    topic_to_model = {
        args.ugv_topic: args.ugv_model,
        args.uav_topic: args.uav_model,
    }
    tracks = read_tracks(bag_dir, topic_to_model, args.start_s, args.duration_s)
    state = PoseReplayState(tracks, args.speed, hold_last=not args.no_hold_last)
    print(
        f"Serving bag poses from {bag_dir} on {args.host}:{args.port}; "
        f"duration={state.duration:.3f}s speed={args.speed}",
        flush=True,
    )
    server = PoseTcpServer((args.host, args.port), state)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        server.server_close()


if __name__ == "__main__":
    main()
