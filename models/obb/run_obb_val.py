#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path
from typing import Any

def parse_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    value_l = value.strip().lower()
    if value_l in {"1", "true", "yes", "y", "on"}:
        return True
    if value_l in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected boolean value, got: {value}")


def parse_compile(value: str | bool) -> bool | str:
    if isinstance(value, bool):
        return value
    value_l = value.strip().lower()
    if value_l in {"1", "true", "yes", "y", "on"}:
        return True
    if value_l in {"0", "false", "no", "n", "off"}:
        return False
    return value


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Ultralytics OBB val and persist metrics.json.")
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--data", required=True, type=Path)
    parser.add_argument("--dataset", required=True)
    parser.add_argument("--split", default="val", choices=["train", "val", "test"])
    parser.add_argument("--project", required=True, type=Path)
    parser.add_argument("--name", required=True)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--iou", type=float, default=0.7)
    parser.add_argument("--single-cls", action="store_true")
    parser.add_argument("--plots", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--agnostic-nms", type=parse_bool, default=True)
    parser.add_argument("--compile", type=parse_compile, default=True)
    return parser.parse_args()


def json_safe(value: Any) -> Any:
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [json_safe(v) for v in value]
    if hasattr(value, "item"):
        try:
            return value.item()
        except Exception:
            pass
    try:
        return float(value)
    except (TypeError, ValueError):
        return str(value)


def image_metric_rows(image_metrics: Any) -> list[dict[str, Any]]:
    if not image_metrics:
        return []

    if isinstance(image_metrics, dict):
        items = image_metrics.items()
    else:
        items = enumerate(image_metrics)

    rows: list[dict[str, Any]] = []
    for image, metrics in items:
        row: dict[str, Any] = {"image": str(image)}
        safe_metrics = json_safe(metrics)
        if isinstance(safe_metrics, dict):
            row.update(safe_metrics)
        else:
            row["value"] = safe_metrics
        rows.append(row)
    return rows


def write_image_metrics(save_dir: Path, metrics: Any) -> tuple[str, str]:
    image_metrics = getattr(getattr(metrics, "box", None), "image_metrics", None)
    rows = image_metric_rows(image_metrics)
    if not rows:
        return "", ""

    json_path = save_dir / "image_metrics.json"
    csv_path = save_dir / "image_metrics.csv"
    json_path.write_text(json.dumps(rows, indent=2), encoding="utf-8")

    preferred = ["image", "precision", "recall", "f1", "tp", "fp", "fn"]
    extra = sorted({key for row in rows for key in row if key not in preferred})
    fieldnames = [key for key in preferred if any(key in row for row in rows)] + extra

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for row in rows:
            writer.writerow({key: row.get(key, "") for key in fieldnames})

    return str(json_path), str(csv_path)


def main() -> int:
    args = parse_args()

    from ultralytics import YOLO

    model_path = args.model.expanduser().resolve()
    data_path = args.data.expanduser().resolve()
    project = args.project.expanduser().resolve()

    model = YOLO(str(model_path))
    metrics = model.val(
        data=str(data_path),
        task="obb",
        split=args.split,
        imgsz=args.imgsz,
        device=args.device,
        project=str(project),
        name=args.name,
        exist_ok=True,
        single_cls=args.single_cls,
        iou=args.iou,
        plots=args.plots,
        agnostic_nms=args.agnostic_nms,
        compile=args.compile,
    )

    save_dir = Path(metrics.save_dir)
    metrics_json = save_dir / "metrics.json"
    image_metrics_json, image_metrics_csv = write_image_metrics(save_dir, metrics)
    payload = {
        "dataset": args.dataset,
        "split": args.split,
        "model": str(model_path),
        "data": str(data_path),
        "save_dir": str(save_dir),
        "image_metrics_json": image_metrics_json,
        "image_metrics_csv": image_metrics_csv,
        "val_args": {
            "imgsz": args.imgsz,
            "device": args.device,
            "iou": args.iou,
            "single_cls": args.single_cls,
            "plots": args.plots,
            "agnostic_nms": args.agnostic_nms,
            "compile": args.compile,
        },
        "metrics": json_safe(metrics.results_dict),
    }

    metrics_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Saved metrics: {metrics_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
