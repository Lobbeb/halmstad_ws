#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any


METRIC_KEYS = [
    "precision",
    "recall",
    "mAP50",
    "mAP50-95",
    "val/box_loss",
    "val/cls_loss",
    "val/dfl_loss",
    "val/angle_loss",
    "train/box_loss",
    "train/cls_loss",
    "train/dfl_loss",
    "train/angle_loss",
    "epoch",
    "lr",
]


CONFIG_KEYS = [
    "model",
    "data",
    "epochs",
    "patience",
    "batch",
    "imgsz",
    "optimizer",
    "lr0",
    "lrf",
    "cos_lr",
    "hsv_h",
    "hsv_s",
    "hsv_v",
    "degrees",
    "translate",
    "scale",
    "mosaic",
    "mixup",
    "cutmix",
    "copy_paste",
    "erasing",
    "close_mosaic",
]


def as_float(value: Any) -> float | None:
    if value is None:
        return None

    if isinstance(value, str):
        if value.lower() in {"none", "nan", ""}:
            return None

    try:
        return float(value)
    except (TypeError, ValueError):
        return None


def load_json(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return json.load(f)


def find_value(data: dict[str, Any], key: str) -> Any:
    if key in data:
        return data[key]

    # Some exported files may wrap results.
    for wrapper in ["results", "metrics", "config", "args"]:
        value = data.get(wrapper)
        if isinstance(value, dict) and key in value:
            return value[key]

    return None


def score_run(row: dict[str, Any]) -> float:
    """
    Simple ranking score.
    Prioritizes mAP50-95, then recall, then mAP50,
    and mildly penalizes box/angle loss.
    """
    map5095 = as_float(row.get("mAP50-95")) or 0.0
    map50 = as_float(row.get("mAP50")) or 0.0
    recall = as_float(row.get("recall")) or 0.0

    box_loss = as_float(row.get("val/box_loss")) or 0.0
    angle_loss = as_float(row.get("val/angle_loss")) or 0.0

    return (
        0.55 * map5095
        + 0.20 * map50
        + 0.20 * recall
        - 0.025 * box_loss
        - 0.025 * angle_loss
    )


def collect_runs(metrics_dir: Path, configs_dir: Path | None) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []

    for metrics_path in sorted(metrics_dir.glob("*.json")):
        run_name = metrics_path.stem
        metrics = load_json(metrics_path)

        config = {}
        if configs_dir is not None:
            config_path = configs_dir / f"{run_name}.json"
            if config_path.exists():
                config = load_json(config_path)

        row: dict[str, Any] = {
            "run": run_name,
            "metrics_file": str(metrics_path),
        }

        for key in METRIC_KEYS:
            row[key] = find_value(metrics, key)

        for key in CONFIG_KEYS:
            row[key] = find_value(config, key)

        row["score"] = round(score_run(row), 6)
        rows.append(row)

    rows.sort(key=lambda r: as_float(r.get("score")) or -999, reverse=True)
    return rows


def write_csv(rows: list[dict[str, Any]], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)

    fieldnames = ["run", "score"] + METRIC_KEYS + CONFIG_KEYS + ["metrics_file"]

    with out_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})


def md_value(value: Any) -> str:
    if value is None:
        return ""

    number = as_float(value)
    if number is not None:
        return f"{number:.6g}"

    return str(value)


def write_markdown(rows: list[dict[str, Any]], out_path: Path) -> None:
    out_path.parent.mkdir(parents=True, exist_ok=True)

    columns = [
        "run",
        "score",
        "mAP50",
        "mAP50-95",
        "precision",
        "recall",
        "val/box_loss",
        "val/angle_loss",
        "epoch",
        "model",
        "lr0",
        "mosaic",
        "scale",
        "degrees",
    ]

    with out_path.open("w", encoding="utf-8") as f:
        f.write("| " + " | ".join(columns) + " |\n")
        f.write("| " + " | ".join(["---"] * len(columns)) + " |\n")

        for row in rows:
            f.write("| " + " | ".join(md_value(row.get(c)) for c in columns) + " |\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--metrics-dir", required=True, type=Path)
    parser.add_argument("--configs-dir", type=Path, default=None)
    parser.add_argument("--out-dir", required=True, type=Path)

    args = parser.parse_args()

    rows = collect_runs(args.metrics_dir, args.configs_dir)

    csv_path = args.out_dir / "scoreboard.csv"
    md_path = args.out_dir / "scoreboard.md"

    write_csv(rows, csv_path)
    write_markdown(rows, md_path)

    print(f"Wrote: {csv_path}")
    print(f"Wrote: {md_path}")

    if rows:
        print()
        print("Best run:")
        best = rows[0]
        print(f"  run:       {best.get('run')}")
        print(f"  score:     {best.get('score')}")
        print(f"  mAP50:     {best.get('mAP50')}")
        print(f"  mAP50-95:  {best.get('mAP50-95')}")
        print(f"  precision: {best.get('precision')}")
        print(f"  recall:    {best.get('recall')}")


if __name__ == "__main__":
    main()

