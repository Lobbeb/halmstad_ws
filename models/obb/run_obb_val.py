#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

from ultralytics import YOLO


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


def main() -> int:
    args = parse_args()

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
    )

    save_dir = Path(metrics.save_dir)
    metrics_json = save_dir / "metrics.json"
    payload = {
        "dataset": args.dataset,
        "split": args.split,
        "model": str(model_path),
        "data": str(data_path),
        "save_dir": str(save_dir),
        "metrics": json_safe(metrics.results_dict),
    }

    metrics_json.write_text(json.dumps(payload, indent=2), encoding="utf-8")
    print(f"Saved metrics: {metrics_json}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
