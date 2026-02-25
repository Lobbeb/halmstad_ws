#!/usr/bin/env python3
from __future__ import annotations

import json
import math
import sys
from pathlib import Path
from typing import Dict, Any, List, Tuple

import yaml


def path_length(points: List[Tuple[float, float, float]]) -> float:
    if len(points) < 2:
        return 0.0
    dist = 0.0
    for (x1, y1, z1), (x2, y2, z2) in zip(points, points[1:]):
        dist += math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2 + (z2 - z1) ** 2)
    return dist


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: analyze_run.py <run_dir>")
        return 2

    run_dir = Path(sys.argv[1]).expanduser().resolve()
    meta_path = run_dir / "meta.yaml"
    bag_dir = run_dir / "bag"

    if not meta_path.exists():
        print(f"ERROR: missing {meta_path}")
        return 2
    if not bag_dir.exists():
        print(f"ERROR: missing {bag_dir}")
        return 2

    meta = yaml.safe_load(meta_path.read_text()) or {}

    # NOTE: This is a “restore baseline” analyzer.
    # Your earlier docs mention direct rosbag parsing; we keep this simple and robust for now.
    # Next iteration: use rosbag2_py to parse Pose/Odometry streams.
    results: Dict[str, Any] = {
        "run_id": meta.get("run_id"),
        "timestamp": meta.get("timestamp"),
        "condition": meta.get("condition"),
        "world": meta.get("world"),
        "uav_name": meta.get("uav_name"),
        "with_cameras": meta.get("with_cameras"),
        "notes": "Analyzer stub restored. Extend with rosbag2_py parsing next.",
    }

    out_path = run_dir / "results.json"
    out_path.write_text(json.dumps(results, indent=2))
    print(json.dumps(results, indent=2))
    print(f"✅ wrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
