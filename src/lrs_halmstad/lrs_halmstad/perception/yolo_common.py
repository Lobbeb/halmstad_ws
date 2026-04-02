from __future__ import annotations

import os
from pathlib import Path
from typing import Optional

import numpy as np
from ament_index_python.packages import get_package_share_path
from rclpy.time import Time
from sensor_msgs.msg import Image

from lrs_halmstad.common.paths import package_source_root, workspace_root

try:
    import cv2  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    cv2 = None

try:
    from ultralytics import YOLO  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    YOLO = None

try:
    import torch  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    torch = None


def default_models_root(module_file: str) -> str:
    configured_root = os.environ.get("LRS_HALMSTAD_MODELS_ROOT", "").strip()
    if configured_root:
        return os.path.expanduser(configured_root)
    try:
        pkg_share = get_package_share_path("lrs_halmstad").resolve()
        for parent in pkg_share.parents:
            candidate = parent / "src" / "lrs_halmstad"
            if candidate.is_dir():
                return str((parent / "models").resolve())
    except Exception:
        pass
    return str((workspace_root(module_file) / "models").resolve())


def package_config_root(module_file: str) -> Path:
    try:
        return (get_package_share_path("lrs_halmstad").resolve() / "config").resolve()
    except Exception:
        return (package_source_root(module_file) / "config").resolve()


def resolve_yolo_weights_path(raw_path: str, models_root: str) -> str:
    if not raw_path:
        return ""
    expanded = os.path.expanduser(raw_path)
    if os.path.isabs(expanded):
        return expanded
    return os.path.join(models_root, expanded)


def resolve_onnx_model_path(raw_path: str, models_root: str, yolo_weights_path: str = "") -> str:
    expanded = os.path.expanduser(str(raw_path or "").strip())
    if expanded:
        if os.path.isabs(expanded):
            return expanded
        return os.path.join(models_root, expanded)

    weights_path = os.path.expanduser(str(yolo_weights_path or "").strip())
    if weights_path.lower().endswith(".onnx"):
        return weights_path
    if weights_path.lower().endswith(".pt"):
        candidate = os.path.splitext(weights_path)[0] + ".onnx"
        if os.path.isfile(candidate):
            return candidate
    return ""


def resolve_tracker_config(raw_path: str, module_file: str) -> str:
    config_root = package_config_root(module_file)
    if not raw_path:
        return str(config_root / "trackers" / "bytetrack.yaml")
    expanded = Path(os.path.expanduser(raw_path))
    if expanded.is_absolute():
        return str(expanded)
    candidates = [
        config_root / "trackers" / expanded,
        config_root / expanded,
    ]
    for candidate in candidates:
        if candidate.is_file():
            return str(candidate)
    return str((config_root / expanded).resolve())


def load_ultralytics_model(weights_path: str):
    if YOLO is None:
        raise RuntimeError("ultralytics_not_installed")
    return YOLO(weights_path)


def normalize_inference_device(raw_device: str) -> str:
    requested = str(raw_device or "").strip()
    return requested or "auto"


def cuda_available() -> bool:
    if torch is None:
        return False
    try:
        return bool(torch.cuda.is_available() and torch.cuda.device_count() > 0)
    except Exception:
        return False


def resolve_inference_device(raw_device: str) -> tuple[str, Optional[str]]:
    requested = normalize_inference_device(raw_device)
    requested_lower = requested.lower()

    if requested_lower in {"auto", "gpu"}:
        if cuda_available():
            return "0", None
        return "cpu", "cuda_unavailable_auto_fallback"

    if requested_lower == "cuda":
        if cuda_available():
            return "0", None
        return "cpu", "cuda_unavailable_cpu_fallback"

    if requested_lower.startswith("cuda:") or requested_lower.isdigit():
        if cuda_available():
            return requested_lower, None
        return "cpu", f"{requested_lower}_unavailable_cpu_fallback"

    if requested_lower == "cpu":
        return "cpu", None

    return requested, None


def image_to_bgr(msg: Image) -> Optional[np.ndarray]:
    if cv2 is None:
        return None
    try:
        if msg.encoding.lower() in ("bgr8", "rgb8"):
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, 3))
            if msg.encoding.lower() == "rgb8":
                arr = cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
            return arr.copy()
        if msg.encoding.lower() == "mono8":
            arr = np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width))
            return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    except Exception:
        return None
    return None


def order_quad(points: np.ndarray) -> np.ndarray:
    center = np.mean(points, axis=0)
    angles = np.arctan2(points[:, 1] - center[1], points[:, 0] - center[0])
    ordered = points[np.argsort(angles)]
    start = int(np.argmin(ordered[:, 0] + ordered[:, 1]))
    return np.roll(ordered, -start, axis=0)


def stamp_ns(msg: Image) -> int:
    try:
        return int(Time.from_msg(msg.header.stamp).nanoseconds)
    except Exception:
        return 0
