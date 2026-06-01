#!/usr/bin/env python3
from __future__ import annotations

import argparse
from pathlib import Path


def parse_bool(value: str | bool) -> bool:
    if isinstance(value, bool):
        return value
    value_l = value.strip().lower()
    if value_l in {"1", "true", "yes", "y", "on"}:
        return True
    if value_l in {"0", "false", "no", "n", "off"}:
        return False
    raise argparse.ArgumentTypeError(f"Expected boolean value, got: {value}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run Ultralytics OBB prediction.")
    parser.add_argument("--model", required=True, type=Path)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--project", required=True, type=Path)
    parser.add_argument("--name", required=True)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--imgsz", type=int, default=640)
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--max-det", type=int, default=300)
    parser.add_argument("--save", type=parse_bool, default=True)
    parser.add_argument("--save-txt", type=parse_bool, default=True)
    parser.add_argument("--save-conf", type=parse_bool, default=True)
    parser.add_argument("--exist-ok", type=parse_bool, default=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()

    from ultralytics import YOLO

    model_path = args.model.expanduser().resolve()
    source = args.source.expanduser().resolve()
    project = args.project.expanduser().resolve()

    if not source.exists():
        raise FileNotFoundError(f"Prediction source does not exist: {source}")

    model = YOLO(str(model_path))
    model.predict(
        source=str(source),
        task="obb",
        imgsz=args.imgsz,
        conf=args.conf,
        max_det=args.max_det,
        device=args.device,
        save=args.save,
        save_txt=args.save_txt,
        save_conf=args.save_conf,
        project=str(project),
        name=args.name,
        exist_ok=args.exist_ok,
    )

    print(f"Saved predictions: {project / args.name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
