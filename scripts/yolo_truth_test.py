#!/usr/bin/env python3
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

try:
    import cv2  # type: ignore
except Exception as exc:  # pragma: no cover
    raise RuntimeError(f"OpenCV is required: {exc}") from exc

try:
    from ultralytics import YOLO  # type: ignore
except Exception as exc:  # pragma: no cover
    raise RuntimeError(f"Ultralytics is required: {exc}") from exc


@dataclass
class FrameCapture:
    bgr: Optional[np.ndarray] = None
    stamp_ns: int = 0
    encoding: str = ""


def image_to_bgr(msg: Image) -> Optional[np.ndarray]:
    if msg.height <= 0 or msg.width <= 0:
        return None
    if len(msg.data) == 0:
        return None

    enc = str(msg.encoding).lower()
    h = int(msg.height)
    w = int(msg.width)

    if enc == "bgr8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
        return arr.copy()
    if enc == "rgb8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 3))
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc == "bgra8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    if enc == "rgba8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w, 4))
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if enc == "mono8":
        arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((h, w))
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)

    # Unsupported encoding
    return None


class OneShotImage(Node):
    def __init__(self, topic: str):
        super().__init__("yolo_truth_test_capture")
        self.capture = FrameCapture()
        self.sub = self.create_subscription(Image, topic, self._on_img, 10)

    def _on_img(self, msg: Image) -> None:
        bgr = image_to_bgr(msg)
        if bgr is None:
            return
        self.capture.bgr = bgr
        self.capture.encoding = str(msg.encoding)
        self.capture.stamp_ns = int(msg.header.stamp.sec) * 1_000_000_000 + int(msg.header.stamp.nanosec)


def run_model(model_path: str, bgr: np.ndarray, conf: float, iou: float, imgsz: int) -> tuple[int, str]:
    model = YOLO(model_path)
    results = model.predict(
        source=bgr,
        verbose=False,
        conf=conf,
        iou=iou,
        imgsz=imgsz,
    )
    if not results:
        return 0, "no_results"
    r = results[0]
    boxes = getattr(r, "boxes", None)
    if boxes is None or len(boxes) == 0:
        return 0, "no_boxes"

    names = getattr(r, "names", {}) or {}
    parts = []
    n = len(boxes)
    for i in range(min(n, 5)):
        b = boxes[i]
        conf_i = float(b.conf[0]) if getattr(b, "conf", None) is not None else -1.0
        cls_i = int(b.cls[0]) if getattr(b, "cls", None) is not None else -1
        cls_name = str(names.get(cls_i, cls_i))
        parts.append(f"{cls_name}@{conf_i:.3f}")
    return n, ", ".join(parts)


def main() -> int:
    ap = argparse.ArgumentParser(description="Capture one ROS camera frame and run YOLO truth test on it.")
    ap.add_argument("--topic", default="/dji0/camera0/image_raw")
    ap.add_argument("--model", required=True, help="Primary model path (.pt)")
    ap.add_argument("--compare-model", default="", help="Optional second model path for A/B")
    ap.add_argument("--out", default="/tmp/ros_truth_frame.png", help="Saved frame path")
    ap.add_argument("--timeout-s", type=float, default=8.0)
    ap.add_argument("--conf", type=float, default=0.05)
    ap.add_argument("--iou", type=float, default=0.50)
    ap.add_argument("--imgsz", type=int, default=640)
    args = ap.parse_args()

    out_path = Path(args.out).expanduser()
    out_path.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init(args=None)
    node = OneShotImage(args.topic)
    deadline = time.monotonic() + max(0.5, float(args.timeout_s))
    try:
        while rclpy.ok() and node.capture.bgr is None and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if node.capture.bgr is None:
        print(f"[truth-test] ERROR: no frame received from topic {args.topic} within {args.timeout_s:.1f}s")
        return 2

    cv2.imwrite(str(out_path), node.capture.bgr)
    print(f"[truth-test] saved frame: {out_path}")
    print(f"[truth-test] encoding={node.capture.encoding} stamp_ns={node.capture.stamp_ns}")

    n1, info1 = run_model(args.model, node.capture.bgr, args.conf, args.iou, args.imgsz)
    print(f"[truth-test] model={args.model} boxes={n1} details={info1}")

    if args.compare_model:
        n2, info2 = run_model(args.compare_model, node.capture.bgr, args.conf, args.iou, args.imgsz)
        print(f"[truth-test] model={args.compare_model} boxes={n2} details={info2}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
