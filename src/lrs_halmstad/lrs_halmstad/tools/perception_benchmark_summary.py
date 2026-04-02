from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

from lrs_halmstad.perception.runtime_metrics import summarize_csv_rows


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Summarize detector benchmark CSV output.")
    parser.add_argument("csv_paths", nargs="+", help="Benchmark CSV files to summarize")
    return parser.parse_args()


def _fmt(value: float) -> str:
    if value is None or not math.isfinite(float(value)):
        return "na"
    return f"{float(value):.2f}"


def main() -> None:
    args = parse_args()
    header = (
        "| file | samples | infer mean ms | infer median ms | infer p95 ms | "
        "latency mean ms | latency median ms | latency p95 ms | detector hz | dropped | stale |"
    )
    print(header)
    print("|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|")
    for csv_path in args.csv_paths:
        path = Path(csv_path).expanduser().resolve()
        with open(path, "r", encoding="utf-8", newline="") as fh:
            rows = list(csv.DictReader(fh))
        stats = summarize_csv_rows(rows)
        print(
            "| "
            + " | ".join(
                [
                    path.name,
                    str(int(stats["samples"])),
                    _fmt(stats["inference_mean_ms"]),
                    _fmt(stats["inference_median_ms"]),
                    _fmt(stats["inference_p95_ms"]),
                    _fmt(stats["latency_mean_ms"]),
                    _fmt(stats["latency_median_ms"]),
                    _fmt(stats["latency_p95_ms"]),
                    _fmt(stats["detector_hz"]),
                    str(int(stats["dropped_frames_total"])),
                    str(int(stats["stale_detections_total"])),
                ]
            )
            + " |"
        )


if __name__ == "__main__":
    main()
