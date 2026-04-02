from __future__ import annotations

import argparse
import json
import os
import shutil
from pathlib import Path

from lrs_halmstad.perception.yolo_common import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export an Ultralytics YOLO model to ONNX for runtime deployment.")
    parser.add_argument("--weights", required=True, help="Input .pt weights path")
    parser.add_argument("--out", required=True, help="Output .onnx path")
    parser.add_argument("--imgsz", type=int, default=640, help="Export input size")
    parser.add_argument("--opset", type=int, default=17, help="ONNX opset")
    parser.add_argument("--dynamic", action="store_true", help="Enable dynamic input shapes")
    parser.add_argument("--simplify", action="store_true", help="Enable graph simplification")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if YOLO is None:
        raise RuntimeError("ultralytics_not_installed")

    weights = os.path.expanduser(str(args.weights).strip())
    out_path = Path(os.path.expanduser(str(args.out).strip())).resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    model = YOLO(weights)
    exported = model.export(
        format="onnx",
        imgsz=int(args.imgsz),
        opset=int(args.opset),
        dynamic=bool(args.dynamic),
        simplify=bool(args.simplify),
        nms=False,
    )
    exported_path = Path(str(exported)).resolve()
    if exported_path != out_path:
        shutil.copy2(str(exported_path), str(out_path))

    manifest = {
        "source_weights": str(Path(weights).resolve()),
        "exported_onnx": str(out_path),
        "imgsz": int(args.imgsz),
        "opset": int(args.opset),
        "dynamic": bool(args.dynamic),
        "simplify": bool(args.simplify),
        "nms": False,
    }
    manifest_path = out_path.with_suffix(out_path.suffix + ".manifest.json")
    with open(manifest_path, "w", encoding="utf-8") as fh:
        json.dump(manifest, fh, indent=2, sort_keys=True)

    print(str(out_path))
    print(str(manifest_path))


if __name__ == "__main__":
    main()
