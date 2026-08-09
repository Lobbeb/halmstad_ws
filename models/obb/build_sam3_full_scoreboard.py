#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any


MAIN_DATASET = "baylands_sam3_obb_super_75_15_10"
GENERALIZATION_DATASETS = [
    "baylands_sam3_obb_generalization_art",
    "baylands_sam3_obb_generalization_playground",
    "baylands_sam3_obb_generalization_road_to_art",
    "baylands_sam3_obb_generalization_rotundan",
]
DATASETS = [MAIN_DATASET] + GENERALIZATION_DATASETS
SAM3_DATASET_PREFIX = "baylands_sam3_obb_"
LEGACY_DATASET_PREFIX = "baylands_"

RUN_ALIASES = {
    # Local eval was once launched with 020 in the run name, but the trusted
    # cloud config/metrics and local weight are the 025 mosaic run.
    "baylands-leader-v7c-ft-mosaic020-lowerase": "baylands-leader-v7c-ft-mosaic025-lowerase",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build SAM3 hybrid scoreboards from training metrics, local val metrics, and prediction analysis."
    )
    parser.add_argument(
        "--exp-root",
        type=Path,
        default=Path("/home/ruben/halmstad_ws/models/obb/experiments/sam3_hybrid_full"),
    )
    parser.add_argument("--prediction-analysis-root", type=Path, default=None)
    parser.add_argument("--runs", nargs="*", default=None)
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
    else:
        for key, value in items:
            if key == wanted_l:
                return as_float(value)

    return None


def mean(values: list[float | None]) -> float | None:
    clean = [float(v) for v in values if v is not None and not math.isnan(float(v))]
    if not clean:
        return None
    return sum(clean) / len(clean)


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


def metric_score(metrics: dict[str, Any]) -> float | None:
    map5095 = find_metric(metrics, "mAP50-95")
    map50 = find_metric(metrics, "mAP50")
    recall = find_metric(metrics, "recall")
    precision = find_metric(metrics, "precision")
    box_loss = find_metric(metrics, "val/box_loss") or 0.0
    angle_loss = find_metric(metrics, "val/angle_loss") or 0.0

    required = [map5095, map50, recall, precision]
    if any(v is None for v in required):
        return None

    return (
        0.55 * float(map5095)
        + 0.20 * float(map50)
        + 0.15 * float(recall)
        + 0.10 * float(precision)
        - 0.025 * box_loss
        - 0.025 * angle_loss
    )


def eval_score(row: dict[str, Any]) -> float | None:
    main_test_map = as_float(row.get("main_test_mAP50-95"))
    gen_map = as_float(row.get("gen_avg_mAP50-95"))
    train_map = as_float(row.get("train_mAP50-95"))
    pred_avg = as_float(row.get("pred_avg_score"))

    if main_test_map is None or gen_map is None:
        return None

    return (
        0.45 * main_test_map
        + 0.35 * gen_map
        + 0.10 * (train_map or 0.0)
        + 0.10 * (pred_avg or 0.0)
    )


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
    legacy_name = legacy_dataset_name(dataset)
    if legacy_name == legacy_dataset_name(MAIN_DATASET):
        return "main"
    return legacy_name.replace("baylands_generalization_", "gen_")


def run_candidates(run: str) -> list[str]:
    aliases = [alias for alias, canonical in RUN_ALIASES.items() if canonical == run]
    return [run] + sorted(aliases)


def model_version(run: str) -> str:
    for prefix in ("baylands-leader-", "baylands-generalized-", "baylands-original"):
        if run.startswith(prefix):
            return run[len(prefix):]
    return run


def collect_train_metrics(exp_root: Path, run: str) -> tuple[dict[str, Any], str]:
    path = exp_root / "metrics" / f"{run}.json"
    if not path.exists():
        return {}, ""
    payload = load_json(path)
    return metric_dict(payload), str(path)


def collect_local_val_metrics(exp_root: Path, run: str, dataset: str, split: str) -> dict[str, Any]:
    for candidate in run_candidates(run):
        for dataset_candidate in dataset_candidates(dataset):
            name = f"{dataset_candidate}_{split}"
            path = exp_root / "runs" / "val" / candidate / name / "metrics.json"
            if path.exists():
                return metric_dict(load_json(path))
    return {}


def add_metric_columns(row: dict[str, Any], prefix: str, metrics: dict[str, Any]) -> None:
    if not metrics:
        return
    row[f"{prefix}_mAP50-95"] = find_metric(metrics, "mAP50-95")
    row[f"{prefix}_mAP50"] = find_metric(metrics, "mAP50")
    row[f"{prefix}_precision"] = find_metric(metrics, "precision")
    row[f"{prefix}_recall"] = find_metric(metrics, "recall")
    row[f"{prefix}_metric_score"] = metric_score(metrics)


def add_prediction_columns(analysis_root: Path, row: dict[str, Any], run: str, dataset: str) -> None:
    path = None
    for candidate in run_candidates(run):
        for dataset_candidate in dataset_candidates(dataset):
            candidate_path = analysis_root / candidate / dataset_candidate / "summary.json"
            if candidate_path.exists():
                path = candidate_path
                break
        if path is not None:
            break
    if path is None:
        return

    summary = load_json(path)
    prefix = f"{short_dataset(dataset)}_pred"
    iou = as_float(summary.get("mean_best_iou_non_empty"))
    issues = issue_rate(summary)

    row[f"{prefix}_iou"] = iou
    row[f"{prefix}_issue_rate"] = issues
    row[f"{prefix}_score"] = prediction_score(iou, issues)
    row[f"{prefix}_no_prediction"] = status_count(summary, "no_prediction")
    row[f"{prefix}_multi_prediction"] = status_count(summary, "multi_prediction")
    row[f"{prefix}_false_positive_empty"] = status_count(summary, "false_positive_empty")


def collect_run(exp_root: Path, analysis_root: Path, run: str) -> dict[str, Any]:
    config_path = exp_root / "configs" / f"{run}.json"
    config = load_json(config_path) if config_path.exists() else {}

    train_metrics, train_metrics_file = collect_train_metrics(exp_root, run)
    row: dict[str, Any] = {
        "run": run,
        "model_version": model_version(run),
        "model": config.get("model", ""),
        "data": config.get("data", ""),
        "lr0": as_float(config.get("lr0")),
        "mosaic": as_float(config.get("mosaic")),
        "scale": as_float(config.get("scale")),
        "degrees": as_float(config.get("degrees")),
        "epoch": find_metric(train_metrics, "epoch"),
        "metrics_file": train_metrics_file,
    }

    add_metric_columns(row, "train", train_metrics)

    main_val_metrics = collect_local_val_metrics(exp_root, run, MAIN_DATASET, "val")
    main_test_metrics = collect_local_val_metrics(exp_root, run, MAIN_DATASET, "test")
    add_metric_columns(row, "main_val", main_val_metrics)
    add_metric_columns(row, "main_test", main_test_metrics)

    gen_maps: list[float | None] = []
    gen_metric_scores: list[float | None] = []
    gen_pred_scores: list[float | None] = []

    for dataset in GENERALIZATION_DATASETS:
        prefix = short_dataset(dataset)
        metrics = collect_local_val_metrics(exp_root, run, dataset, "test")
        add_metric_columns(row, prefix, metrics)
        gen_maps.append(row.get(f"{prefix}_mAP50-95"))
        gen_metric_scores.append(row.get(f"{prefix}_metric_score"))

    for dataset in DATASETS:
        add_prediction_columns(analysis_root, row, run, dataset)
        if dataset in GENERALIZATION_DATASETS:
            gen_pred_scores.append(row.get(f"{short_dataset(dataset)}_pred_score"))

    row["train_score"] = metric_score(train_metrics)
    row["gen_avg_mAP50-95"] = mean(gen_maps)
    row["gen_avg_metric_score"] = mean(gen_metric_scores)
    row["gen_avg_pred_score"] = mean(gen_pred_scores)
    row["pred_avg_score"] = mean([row.get("main_pred_score"), row.get("gen_avg_pred_score")])
    row["eval_score"] = eval_score(row)
    return row


def fmt_md(value: Any, column: str) -> str:
    if value is None:
        return ""
    if column == "epoch":
        value_float = as_float(value)
        return "" if value_float is None else str(int(round(value_float)))
    if isinstance(value, float):
        return f"{value:.5f}"
    return str(value)


def write_csv(path: Path, rows: list[dict[str, Any]]) -> None:
    if not rows:
        return

    preferred = [
        "rank",
        "model_version",
        "eval_score",
        "train_score",
        "train_mAP50-95",
        "main_test_mAP50-95",
        "gen_avg_mAP50-95",
        "pred_avg_score",
        "main_pred_score",
        "gen_avg_pred_score",
        "main_val_mAP50-95",
        "train_mAP50",
        "train_precision",
        "train_recall",
        "main_test_mAP50",
        "main_test_precision",
        "main_test_recall",
        "gen_art_mAP50-95",
        "gen_playground_mAP50-95",
        "gen_road_to_art_mAP50-95",
        "gen_rotundan_mAP50-95",
        "epoch",
        "lr0",
        "mosaic",
        "scale",
        "degrees",
        "model",
        "data",
        "run",
        "metrics_file",
    ]
    extra = sorted({key for row in rows for key in row if key not in preferred})
    fieldnames = preferred + extra

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
        "eval_score",
        "train_score",
        "train_mAP50-95",
        "main_test_mAP50-95",
        "gen_avg_mAP50-95",
        "pred_avg_score",
        "main_pred_score",
        "gen_avg_pred_score",
        "main_val_mAP50-95",
        "lr0",
        "mosaic",
        "epoch",
    ]
    header = (
        '| <div style="width:32px">#</div> | '
        '<div style="width:180px">model version</div> |                                        '
        '<div style="width:50px">eval score</div>  |                                             '
        '<div style="width:50px">train score</div> |                                             '
        '<div style="width:70px">train mAP50-95</div>  |                                         '
        '<div style="width:80px">main test mAP50-95</div> |                                      '
        '<div style="width:80px">gen avg mAP50-95</div>  |                                       '
        '<div style="width:60px">pred avg score</div>  |                                         '
        '<div style="width:70px">main pred score</div>  |                                         '
        '<div style="width:70px">gen avg pred score</div>  |                                    '
        '<div style="width:80px">main val mAP50-95</div>  |                                      '
        '<div style="width:50px">lr0</div>  |                                                    '
        '<div style="width:50px">mosaic</div>  |                                                  '
        '<div style="width:40px">epoch</div>  |'
    )
    alignment = "| ---: | :--- | ---: | :---: | :---: | :---: | :---: | :---: | :---: | :---: | :--- | ---: | :---: | :--- |"

    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        f.write(header + "\n")
        f.write(alignment + "\n")
        for row in rows:
            f.write("| " + " | ".join(fmt_md(row.get(column), column) for column in columns) + " |\n")


def main() -> int:
    args = parse_args()
    exp_root = args.exp_root.expanduser().resolve()
    analysis_root = (
        args.prediction_analysis_root.expanduser().resolve()
        if args.prediction_analysis_root
        else exp_root / "runs" / "prediction_analysis"
    )
    out_dir = args.out_dir.expanduser().resolve() if args.out_dir else exp_root / "scoreboards"

    if args.runs:
        runs = set(args.runs)
    else:
        runs = {
            path.stem
            for path in (exp_root / "metrics").glob("*.json")
            if not path.stem.endswith(" copy")
        }

    if args.runs is None and analysis_root.is_dir():
        runs.update(RUN_ALIASES.get(path.name, path.name) for path in analysis_root.iterdir() if path.is_dir())

    rows = [collect_run(exp_root, analysis_root, run) for run in sorted(runs)]
    rows.sort(
        key=lambda row: (
            as_float(row.get("eval_score")) is not None,
            as_float(row.get("eval_score")) or -1.0,
            as_float(row.get("train_score")) or -1.0,
        ),
        reverse=True,
    )
    for rank, row in enumerate(rows, start=1):
        row["rank"] = rank

    write_csv(out_dir / "full_scoreboard.csv", rows)
    write_markdown(out_dir / "full_scoreboard.md", rows)

    print(f"Wrote: {out_dir / 'full_scoreboard.csv'}")
    print(f"Wrote: {out_dir / 'full_scoreboard.md'}")
    if rows:
        best_eval = next((row for row in rows if row.get("eval_score") is not None), None)
        if best_eval:
            print(f"Best eval: {best_eval['run']} eval_score={best_eval['eval_score']:.6f}")
        print(f"Best training metrics row: {max(rows, key=lambda row: as_float(row.get('train_score')) or -1.0)['run']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
