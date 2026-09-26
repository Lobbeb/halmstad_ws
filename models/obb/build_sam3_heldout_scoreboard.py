#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


HELDOUT_RUNS = {
    "v10-tuned-no-art": "baylands_sam3_obb_generalization_art",
    "v10-tuned-no-playground": "baylands_sam3_obb_generalization_playground",
    "v10-tuned-no-road-to-art": "baylands_sam3_obb_generalization_road_to_art",
    "v10-tuned-no-rotundan": "baylands_sam3_obb_generalization_rotundan",
}
SAM3_DATASET_PREFIX = "baylands_sam3_obb_"
LEGACY_DATASET_PREFIX = "baylands_"
STATUS_COLUMNS = [
    "matched",
    "no_prediction",
    "multi_prediction",
    "false_positive_empty",
    "correct_empty",
    "under_prediction",
]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build held-out scenario scoreboard for leave-one-scenario-out SAM3 OBB runs."
    )
    parser.add_argument(
        "--exp-root",
        type=Path,
        default=Path("/home/ruben/halmstad_ws/models/obb/experiments/sam3_hybrid_full"),
    )
    parser.add_argument("--out-dir", type=Path, default=None)
    return parser.parse_args()


def load_json(path: Path) -> dict[str, Any]:
    try:
        with path.open("r", encoding="utf-8") as f:
            payload = json.load(f)
    except (OSError, json.JSONDecodeError) as exc:
        print(f"Warning: skipping unreadable JSON {path}: {exc}")
        return {}
    return payload if isinstance(payload, dict) else {}


def as_float(value: Any) -> float | None:
    try:
        value = float(value)
    except (TypeError, ValueError):
        return None
    if math.isnan(value):
        return None
    return value


def metric_dict(payload: dict[str, Any]) -> dict[str, Any]:
    metrics = payload.get("metrics", payload.get("results", payload))
    return metrics if isinstance(metrics, dict) else {}


def find_metric(metrics: dict[str, Any], wanted: str) -> float | None:
    wanted_l = wanted.lower()
    items = [(str(k).lower(), v) for k, v in metrics.items()]

    if wanted_l in {"map50-95", "map50_95"}:
        for key, value in items:
            if "map50-95" in key or "map_50-95" in key:
                return as_float(value)
    elif wanted_l == "map50":
        for key, value in items:
            if "map50" in key and "map50-95" not in key:
                return as_float(value)
    elif wanted_l == "precision":
        for key, value in items:
            if "precision" in key:
                return as_float(value)
    elif wanted_l == "recall":
        for key, value in items:
            if "recall" in key:
                return as_float(value)
    return None


def legacy_dataset_name(dataset: str) -> str:
    if dataset.startswith(SAM3_DATASET_PREFIX):
        return LEGACY_DATASET_PREFIX + dataset[len(SAM3_DATASET_PREFIX):]
    return dataset


def sam3_dataset_name(dataset: str) -> str:
    if dataset.startswith(LEGACY_DATASET_PREFIX) and not dataset.startswith(SAM3_DATASET_PREFIX):
        return SAM3_DATASET_PREFIX + dataset[len(LEGACY_DATASET_PREFIX):]
    return dataset


def dataset_candidates(dataset: str) -> list[str]:
    candidates = [dataset, legacy_dataset_name(dataset), sam3_dataset_name(dataset)]
    return list(dict.fromkeys(candidates))


def short_dataset(dataset: str) -> str:
    return legacy_dataset_name(dataset).replace("baylands_generalization_", "")


def model_version(run: str) -> str:
    for prefix in ("baylands-leader-", "baylands-generalized-", "baylands-original"):
        if run.startswith(prefix):
            return run[len(prefix):]
    return run


def first_existing(paths: list[Path]) -> Path | None:
    for path in paths:
        if path.exists():
            return path
    return None


def collect_val_metrics(exp_root: Path, run: str, dataset: str) -> tuple[dict[str, Any], str]:
    candidates = [
        exp_root / "runs" / "val" / run / f"{dataset_candidate}_test" / "metrics.json"
        for dataset_candidate in dataset_candidates(dataset)
    ]
    path = first_existing(candidates)
    if path is None:
        return {}, ""
    return metric_dict(load_json(path)), str(path)


def collect_summary(exp_root: Path, run: str, dataset: str) -> tuple[dict[str, Any], str]:
    candidates = [
        exp_root / "runs" / "prediction_analysis" / run / dataset_candidate / "summary.json"
        for dataset_candidate in dataset_candidates(dataset)
    ]
    path = first_existing(candidates)
    if path is None:
        return {}, ""
    return load_json(path), str(path)


def status_count(summary: dict[str, Any], key: str) -> int:
    counts = summary.get("status_counts", {})
    if not isinstance(counts, dict):
        return 0
    return int(counts.get(key, 0) or 0)


def issue_rate(summary: dict[str, Any]) -> float | None:
    images = as_float(summary.get("images"))
    if not images:
        return None

    issues = (
        status_count(summary, "no_prediction")
        + status_count(summary, "multi_prediction")
        + status_count(summary, "false_positive_empty")
        + status_count(summary, "under_prediction")
    )
    return issues / images


def prediction_score(iou: float | None, issues: float | None) -> float | None:
    if iou is None or issues is None:
        return None
    return max(0.0, iou * (1.0 - issues))


def collect_row(exp_root: Path, run: str, dataset: str) -> dict[str, Any]:
    metrics, metrics_file = collect_val_metrics(exp_root, run, dataset)
    summary, summary_file = collect_summary(exp_root, run, dataset)

    iou = as_float(summary.get("mean_best_iou_non_empty"))
    issues = issue_rate(summary)
    issue_total = sum(
        status_count(summary, key)
        for key in ("no_prediction", "multi_prediction", "false_positive_empty", "under_prediction")
    )

    row: dict[str, Any] = {
        "run": run,
        "model_version": model_version(run),
        "heldout": short_dataset(dataset),
        "heldout_dataset": dataset,
        "heldout_mAP50-95": find_metric(metrics, "mAP50-95"),
        "heldout_mAP50": find_metric(metrics, "mAP50"),
        "heldout_precision": find_metric(metrics, "precision"),
        "heldout_recall": find_metric(metrics, "recall"),
        "pred_score": prediction_score(iou, issues),
        "mean_iou": iou,
        "issue_rate": issues,
        "issue_total": issue_total,
        "images": int(summary.get("images", 0) or 0),
        "metrics_file": metrics_file,
        "summary_file": summary_file,
    }

    for key in STATUS_COLUMNS:
        row[key] = status_count(summary, key)

    return row


def fmt_md(value: Any) -> str:
    if value is None:
        return ""
    if isinstance(value, float):
        return f"{value:.5f}"
    return str(value)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return

    fieldnames = [
        "rank",
        "model_version",
        "heldout",
        "heldout_mAP50-95",
        "heldout_mAP50",
        "heldout_precision",
        "heldout_recall",
        "pred_score",
        "mean_iou",
        "issue_rate",
        "issue_total",
        "images",
        *STATUS_COLUMNS,
        "run",
        "heldout_dataset",
        "metrics_file",
        "summary_file",
    ]

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def write_markdown(path: Path, rows: list[dict[str, Any]]) -> None:
    columns = [
        "rank",
        "model_version",
        "heldout",
        "heldout_mAP50-95",
        "pred_score",
        "mean_iou",
        "issue_rate",
        "issue_total",
        "images",
        "matched",
        "no_prediction",
        "multi_prediction",
        "false_positive_empty",
        "correct_empty",
        "under_prediction",
    ]

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write("| " + " | ".join(columns) + " |\n")
        f.write("| " + " | ".join(["---"] * len(columns)) + " |\n")
        for row in rows:
            f.write("| " + " | ".join(fmt_md(row.get(column)) for column in columns) + " |\n")


def main() -> int:
    args = parse_args()
    exp_root = args.exp_root.expanduser().resolve()
    out_dir = args.out_dir.expanduser().resolve() if args.out_dir else exp_root / "scoreboards" / "heldout"

    rows = [
        collect_row(exp_root, run, dataset)
        for run, dataset in HELDOUT_RUNS.items()
    ]
    rows.sort(
        key=lambda row: (
            as_float(row.get("heldout_mAP50-95")) is not None,
            as_float(row.get("heldout_mAP50-95")) or -1.0,
            as_float(row.get("pred_score")) or -1.0,
        ),
        reverse=True,
    )
    for rank, row in enumerate(rows, start=1):
        row["rank"] = rank

    write_csv(out_dir / "heldout_scoreboard.csv", rows)
    write_markdown(out_dir / "heldout_scoreboard.md", rows)

    print(f"Wrote: {out_dir / 'heldout_scoreboard.csv'}")
    print(f"Wrote: {out_dir / 'heldout_scoreboard.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
