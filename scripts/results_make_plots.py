#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Any


PLOTS = [
    ("mean_follow_error_m", "Follow error by condition", "follow_error_by_condition.png"),
    ("estimate_fresh_ratio", "Estimate freshness by condition", "estimate_freshness_by_condition.png"),
    ("detector_ok_ratio", "Detector OK ratio by condition", "detector_ok_ratio_by_condition.png"),
    ("no_det_ratio", "Detector NO_DET ratio by condition", "detector_no_det_ratio_by_condition.png"),
    ("stale_ratio", "Detector stale ratio by condition", "detector_stale_ratio_by_condition.png"),
    ("support_ok_ratio", "Support OK ratio by condition", "support_ok_ratio_by_condition.png"),
    ("support_no_det_ratio", "Support NO_DET ratio by condition", "support_no_det_ratio_by_condition.png"),
    ("support_no_input_ratio", "Support NO_INPUT ratio by condition", "support_no_input_ratio_by_condition.png"),
    ("divergence_event_count", "Divergence events by condition", "divergence_events_by_condition.png"),
    ("setpoint_jitter_ms", "Setpoint jitter by condition", "setpoint_jitter_by_condition.png"),
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Make simple plotting-ready PNGs from Results runs.csv.")
    parser.add_argument("runs_csv", help="Path to runs.csv")
    parser.add_argument("--out", default="", help="Output directory, default <runs_csv parent>/plots")
    return parser.parse_args()


def safe_float(value: Any) -> float:
    try:
        text = str(value).strip()
        if text in {"", "NA", "nan", "NaN"}:
            return math.nan
        return float(text)
    except Exception:
        return math.nan


def mean(values: list[float]) -> float:
    finite = [value for value in values if math.isfinite(value)]
    return sum(finite) / len(finite) if finite else math.nan


def main() -> None:
    args = parse_args()
    runs_csv = Path(args.runs_csv).expanduser().resolve()
    out_dir = Path(args.out).expanduser().resolve() if args.out else runs_csv.parent / "plots"
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        import matplotlib.pyplot as plt
    except Exception as exc:
        raise SystemExit(f"matplotlib is required for plotting: {exc}")

    with runs_csv.open("r", encoding="utf-8", newline="") as handle:
        rows = list(csv.DictReader(handle))

    conditions = sorted({row.get("condition", "") for row in rows if row.get("condition", "")})
    for metric, title, filename in PLOTS:
        values = [mean([safe_float(row.get(metric)) for row in rows if row.get("condition") == condition]) for condition in conditions]
        if not any(math.isfinite(value) for value in values):
            continue
        plt.figure(figsize=(max(5.0, 1.2 * len(conditions)), 3.5))
        plt.bar(conditions, [value if math.isfinite(value) else 0.0 for value in values], color="#4c78a8")
        plt.title(title)
        plt.ylabel(metric)
        plt.xlabel("condition")
        plt.tight_layout()
        plt.savefig(out_dir / filename, dpi=160)
        plt.close()

    print(f"plots_dir={out_dir}")


if __name__ == "__main__":
    main()
