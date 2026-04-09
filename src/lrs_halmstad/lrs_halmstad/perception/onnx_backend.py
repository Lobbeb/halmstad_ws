from __future__ import annotations

import ast
import math
import os
import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from lrs_halmstad.perception.detection_protocol import Detection2D
from lrs_halmstad.perception.yolo_common import cv2, order_quad

try:
    import onnxruntime as ort  # type: ignore
except Exception:  # pragma: no cover - optional dependency
    ort = None


def onnxruntime_available() -> bool:
    return ort is not None


def resolve_onnx_backend_name(raw_backend: str) -> str:
    backend = str(raw_backend or "").strip().lower()
    if backend in {"onnx", "onnxruntime"}:
        return "onnx_cpu"
    return backend


def select_onnx_providers(backend: str) -> tuple[list[str], str]:
    backend = resolve_onnx_backend_name(backend)
    if ort is None:
        raise RuntimeError("onnxruntime_not_installed")
    available = list(ort.get_available_providers())
    if backend == "onnx_directml":
        if "DmlExecutionProvider" in available:
            return ["DmlExecutionProvider", "CPUExecutionProvider"], "DmlExecutionProvider"
        return ["CPUExecutionProvider"], "CPUExecutionProvider"
    return ["CPUExecutionProvider"], "CPUExecutionProvider"


def _parse_names_metadata(raw_names: Optional[str]) -> dict[int, str]:
    text = str(raw_names or "").strip()
    if not text:
        return {}
    try:
        parsed = ast.literal_eval(text)
    except Exception:
        return {}
    if isinstance(parsed, dict):
        result: dict[int, str] = {}
        for key, value in parsed.items():
            try:
                result[int(key)] = str(value)
            except Exception:
                continue
        return result
    if isinstance(parsed, (list, tuple)):
        return {int(idx): str(value) for idx, value in enumerate(parsed)}
    return {}


def _clip_boxes_xyxy(boxes: np.ndarray, width: int, height: int) -> np.ndarray:
    boxes[:, 0] = np.clip(boxes[:, 0], 0.0, max(0.0, float(width - 1)))
    boxes[:, 1] = np.clip(boxes[:, 1], 0.0, max(0.0, float(height - 1)))
    boxes[:, 2] = np.clip(boxes[:, 2], 0.0, max(0.0, float(width - 1)))
    boxes[:, 3] = np.clip(boxes[:, 3], 0.0, max(0.0, float(height - 1)))
    return boxes


def _xywh_to_xyxy(boxes_xywh: np.ndarray) -> np.ndarray:
    out = np.empty_like(boxes_xywh, dtype=np.float32)
    out[:, 0] = boxes_xywh[:, 0] - 0.5 * boxes_xywh[:, 2]
    out[:, 1] = boxes_xywh[:, 1] - 0.5 * boxes_xywh[:, 3]
    out[:, 2] = boxes_xywh[:, 0] + 0.5 * boxes_xywh[:, 2]
    out[:, 3] = boxes_xywh[:, 1] + 0.5 * boxes_xywh[:, 3]
    return out


def _bbox_iou(one: np.ndarray, many: np.ndarray) -> np.ndarray:
    x1 = np.maximum(one[0], many[:, 0])
    y1 = np.maximum(one[1], many[:, 1])
    x2 = np.minimum(one[2], many[:, 2])
    y2 = np.minimum(one[3], many[:, 3])
    inter = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    area_one = np.maximum(0.0, one[2] - one[0]) * np.maximum(0.0, one[3] - one[1])
    area_many = np.maximum(0.0, many[:, 2] - many[:, 0]) * np.maximum(0.0, many[:, 3] - many[:, 1])
    denom = np.maximum(1e-6, area_one + area_many - inter)
    return inter / denom


def _nms_by_class(boxes_xyxy: np.ndarray, scores: np.ndarray, classes: np.ndarray, iou_threshold: float) -> np.ndarray:
    keep: list[int] = []
    for cls_id in np.unique(classes):
        cls_mask = classes == cls_id
        cls_indices = np.where(cls_mask)[0]
        order = cls_indices[np.argsort(scores[cls_indices])[::-1]]
        while order.size > 0:
            current = int(order[0])
            keep.append(current)
            if order.size == 1:
                break
            ious = _bbox_iou(boxes_xyxy[current], boxes_xyxy[order[1:]])
            order = order[1:][ious <= float(iou_threshold)]
    return np.asarray(sorted(set(keep)), dtype=np.int64)


def _xywhr_to_corners(x: float, y: float, w: float, h: float, angle_rad: float) -> tuple[tuple[float, float], ...]:
    dx = float(w) * 0.5
    dy = float(h) * 0.5
    corners = np.asarray(
        [
            [-dx, -dy],
            [dx, -dy],
            [dx, dy],
            [-dx, dy],
        ],
        dtype=np.float32,
    )
    c = math.cos(float(angle_rad))
    s = math.sin(float(angle_rad))
    rot = np.asarray([[c, -s], [s, c]], dtype=np.float32)
    rotated = corners @ rot.T
    rotated[:, 0] += float(x)
    rotated[:, 1] += float(y)
    ordered = order_quad(rotated.astype(np.float64))
    return tuple((float(px), float(py)) for px, py in ordered)


@dataclass
class OnnxInferenceResult:
    detections: list[Detection2D]
    provider: str
    infer_end_perf_ns: int
    postprocess_end_perf_ns: int
    raw_prediction_count: int = 0
    conf_pass_count: int = 0
    class_pass_count: int = 0
    nms_keep_count: int = 0
    raw_best_conf: float = -1.0
    class_best_conf: float = -1.0
    target_raw_count: int = 0
    target_best_conf: float = -1.0
    target_best_area_norm: float = -1.0
    target_best_u_norm: float = -1.0
    target_best_v_norm: float = -1.0
    target_best_center_error_norm: float = -1.0


class OnnxYoloRuntime:
    def __init__(
        self,
        *,
        onnx_model: str,
        backend: str,
        imgsz: int,
        conf_threshold: float,
        iou_threshold: float,
        target_class_id: int,
        target_class_name: str,
        task_type: str,
    ) -> None:
        if cv2 is None:
            raise RuntimeError("opencv_not_installed")
        if ort is None:
            raise RuntimeError("onnxruntime_not_installed")
        self.onnx_model = os.path.expanduser(str(onnx_model).strip())
        if not self.onnx_model:
            raise ValueError("onnx_model must be set for ONNX backends")
        if not os.path.isfile(self.onnx_model):
            raise ValueError(f"onnx model not found: {self.onnx_model}")

        providers, requested_provider = select_onnx_providers(backend)
        self.session = ort.InferenceSession(self.onnx_model, providers=providers)
        self.provider = self.session.get_providers()[0] if self.session.get_providers() else requested_provider
        self.input_name = self.session.get_inputs()[0].name
        self.input_shape = self.session.get_inputs()[0].shape
        self.imgsz = int(max(32, imgsz))
        self.conf_threshold = float(conf_threshold)
        self.iou_threshold = float(iou_threshold)
        self.target_class_id = int(target_class_id)
        self.target_class_name = str(target_class_name or "").strip()
        self.task_type = str(task_type or "detection").strip().lower()

        meta = self.session.get_modelmeta().custom_metadata_map or {}
        self.class_names = _parse_names_metadata(meta.get("names"))
        meta_task = str(meta.get("task", "")).strip().lower()
        if meta_task:
            self.task_type = "obb" if meta_task == "obb" else "detection"

    def predict(self, img_bgr: np.ndarray) -> OnnxInferenceResult:
        inp, ratio, pad = self._preprocess(img_bgr)
        outputs = self.session.run(None, {self.input_name: inp})
        infer_end_perf_ns = time.perf_counter_ns()
        detections = self._postprocess(outputs, img_bgr.shape[:2], ratio, pad)
        postprocess_end_perf_ns = time.perf_counter_ns()
        return OnnxInferenceResult(
            detections=detections["detections"],
            provider=self.provider,
            infer_end_perf_ns=infer_end_perf_ns,
            postprocess_end_perf_ns=postprocess_end_perf_ns,
            raw_prediction_count=int(detections["raw_prediction_count"]),
            conf_pass_count=int(detections["conf_pass_count"]),
            class_pass_count=int(detections["class_pass_count"]),
            nms_keep_count=int(detections["nms_keep_count"]),
            raw_best_conf=float(detections["raw_best_conf"]),
            class_best_conf=float(detections["class_best_conf"]),
            target_raw_count=int(detections["target_raw_count"]),
            target_best_conf=float(detections["target_best_conf"]),
            target_best_area_norm=float(detections["target_best_area_norm"]),
            target_best_u_norm=float(detections["target_best_u_norm"]),
            target_best_v_norm=float(detections["target_best_v_norm"]),
            target_best_center_error_norm=float(detections["target_best_center_error_norm"]),
        )

    def _preprocess(self, img_bgr: np.ndarray) -> tuple[np.ndarray, float, tuple[float, float]]:
        src_h, src_w = img_bgr.shape[:2]
        target_h = self.imgsz
        target_w = self.imgsz
        if len(self.input_shape) >= 4:
            shape_h = self.input_shape[2]
            shape_w = self.input_shape[3]
            if isinstance(shape_h, int) and shape_h > 0:
                target_h = int(shape_h)
            if isinstance(shape_w, int) and shape_w > 0:
                target_w = int(shape_w)

        ratio = min(float(target_h) / max(1, src_h), float(target_w) / max(1, src_w))
        new_w = max(1, int(round(src_w * ratio)))
        new_h = max(1, int(round(src_h * ratio)))
        resized = cv2.resize(img_bgr, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
        pad_w = float(target_w - new_w) / 2.0
        pad_h = float(target_h - new_h) / 2.0
        top = int(round(pad_h - 0.1))
        bottom = int(round(pad_h + 0.1))
        left = int(round(pad_w - 0.1))
        right = int(round(pad_w + 0.1))
        padded = cv2.copyMakeBorder(
            resized,
            top,
            bottom,
            left,
            right,
            cv2.BORDER_CONSTANT,
            value=(114, 114, 114),
        )
        rgb = cv2.cvtColor(padded, cv2.COLOR_BGR2RGB)
        tensor = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32) / 255.0
        return tensor[None, ...], float(ratio), (float(left), float(top))

    def _postprocess(
        self,
        outputs: list[np.ndarray],
        original_hw: tuple[int, int],
        ratio: float,
        pad: tuple[float, float],
    ) -> dict[str, object]:
        preds = self._normalize_prediction_array(outputs)
        if preds.size == 0:
            return {
                "detections": [],
                "raw_prediction_count": 0,
                "conf_pass_count": 0,
                "class_pass_count": 0,
                "nms_keep_count": 0,
                "raw_best_conf": -1.0,
                "class_best_conf": -1.0,
            }
        orig_h, orig_w = original_hw
        raw_prediction_count = int(preds.shape[0])
        frame_area = max(1.0, float(orig_w * orig_h))
        target_raw_count = 0
        target_best_conf = -1.0
        target_best_area_norm = -1.0
        target_best_u_norm = -1.0
        target_best_v_norm = -1.0
        target_best_center_error_norm = -1.0

        if self.task_type == "obb":
            if preds.shape[1] < 6:
                return {
                    "detections": [],
                    "raw_prediction_count": raw_prediction_count,
                    "conf_pass_count": 0,
                    "class_pass_count": 0,
                    "nms_keep_count": 0,
                    "raw_best_conf": -1.0,
                    "class_best_conf": -1.0,
                    "target_raw_count": target_raw_count,
                    "target_best_conf": target_best_conf,
                    "target_best_area_norm": target_best_area_norm,
                    "target_best_u_norm": target_best_u_norm,
                    "target_best_v_norm": target_best_v_norm,
                    "target_best_center_error_norm": target_best_center_error_norm,
                }
            boxes_xywh = preds[:, :4].astype(np.float32)
            class_scores = preds[:, 4:-1].astype(np.float32)
            angle = preds[:, -1].astype(np.float32)
        else:
            if preds.shape[1] < 5:
                return {
                    "detections": [],
                    "raw_prediction_count": raw_prediction_count,
                    "conf_pass_count": 0,
                    "class_pass_count": 0,
                    "nms_keep_count": 0,
                    "raw_best_conf": -1.0,
                    "class_best_conf": -1.0,
                    "target_raw_count": target_raw_count,
                    "target_best_conf": target_best_conf,
                    "target_best_area_norm": target_best_area_norm,
                    "target_best_u_norm": target_best_u_norm,
                    "target_best_v_norm": target_best_v_norm,
                    "target_best_center_error_norm": target_best_center_error_norm,
                }
            boxes_xywh = preds[:, :4].astype(np.float32)
            class_scores = preds[:, 4:].astype(np.float32)
            angle = None

        if class_scores.size == 0:
            return {
                "detections": [],
                "raw_prediction_count": raw_prediction_count,
                "conf_pass_count": 0,
                "class_pass_count": 0,
                "nms_keep_count": 0,
                "raw_best_conf": -1.0,
                "class_best_conf": -1.0,
                "target_raw_count": target_raw_count,
                "target_best_conf": target_best_conf,
                "target_best_area_norm": target_best_area_norm,
                "target_best_u_norm": target_best_u_norm,
                "target_best_v_norm": target_best_v_norm,
                "target_best_center_error_norm": target_best_center_error_norm,
            }
        cls_ids = np.argmax(class_scores, axis=1).astype(np.int64)
        confs = class_scores[np.arange(class_scores.shape[0]), cls_ids]
        raw_best_conf = float(np.max(confs)) if confs.size else -1.0
        target_mask = np.ones_like(cls_ids, dtype=bool)
        if self.target_class_id >= 0:
            target_mask &= cls_ids == self.target_class_id
        if self.target_class_name:
            target_mask &= np.asarray(
                [
                    str(self.class_names.get(int(cls_id), "")).lower() == self.target_class_name.lower()
                    for cls_id in cls_ids
                ],
                dtype=bool,
            )
        target_raw_count = int(np.count_nonzero(target_mask))

        boxes_xyxy_all = _xywh_to_xyxy(boxes_xywh.copy())
        boxes_xyxy_all[:, [0, 2]] -= float(pad[0])
        boxes_xyxy_all[:, [1, 3]] -= float(pad[1])
        boxes_xyxy_all[:, :4] /= max(1e-6, float(ratio))
        boxes_xyxy_all = _clip_boxes_xyxy(boxes_xyxy_all, orig_w, orig_h)

        boxes_xywh_all = boxes_xywh.copy()
        boxes_xywh_all[:, 0] = (boxes_xywh_all[:, 0] - float(pad[0])) / max(1e-6, float(ratio))
        boxes_xywh_all[:, 1] = (boxes_xywh_all[:, 1] - float(pad[1])) / max(1e-6, float(ratio))
        boxes_xywh_all[:, 2] = boxes_xywh_all[:, 2] / max(1e-6, float(ratio))
        boxes_xywh_all[:, 3] = boxes_xywh_all[:, 3] / max(1e-6, float(ratio))

        if target_raw_count > 0:
            target_indices = np.flatnonzero(target_mask)
            best_target_idx = int(target_indices[np.argmax(confs[target_indices])])
            target_best_conf = float(confs[best_target_idx])
            x1, y1, x2, y2 = [float(v) for v in boxes_xyxy_all[best_target_idx]]
            u = 0.5 * (x1 + x2)
            v = 0.5 * (y1 + y2)
            target_best_area_norm = max(0.0, (x2 - x1) * (y2 - y1)) / frame_area
            target_best_u_norm = u / max(1.0, float(orig_w))
            target_best_v_norm = v / max(1.0, float(orig_h))
            half_diag = max(1e-6, math.hypot(0.5 * float(orig_w), 0.5 * float(orig_h)))
            target_best_center_error_norm = math.hypot(
                u - 0.5 * float(orig_w),
                v - 0.5 * float(orig_h),
            ) / half_diag

        conf_valid = confs >= float(self.conf_threshold)
        conf_pass_count = int(np.count_nonzero(conf_valid))
        valid = conf_valid.copy()
        valid &= target_mask
        class_pass_count = int(np.count_nonzero(valid))
        if not np.any(valid):
            return {
                "detections": [],
                "raw_prediction_count": raw_prediction_count,
                "conf_pass_count": conf_pass_count,
                "class_pass_count": class_pass_count,
                "nms_keep_count": 0,
                "raw_best_conf": raw_best_conf,
                "class_best_conf": -1.0,
                "target_raw_count": target_raw_count,
                "target_best_conf": target_best_conf,
                "target_best_area_norm": target_best_area_norm,
                "target_best_u_norm": target_best_u_norm,
                "target_best_v_norm": target_best_v_norm,
                "target_best_center_error_norm": target_best_center_error_norm,
            }

        boxes_xywh = boxes_xywh_all[valid]
        cls_ids = cls_ids[valid]
        confs = confs[valid]
        class_best_conf = float(np.max(confs)) if confs.size else -1.0
        if angle is not None:
            angle = angle[valid]
        boxes_xyxy = boxes_xyxy_all[valid]

        keep = _nms_by_class(boxes_xyxy, confs, cls_ids, self.iou_threshold)
        detections: list[Detection2D] = []
        for idx in keep.tolist():
            cls_id = int(cls_ids[idx])
            cls_name = str(self.class_names.get(cls_id, ""))
            x1, y1, x2, y2 = [float(v) for v in boxes_xyxy[idx]]
            kwargs = {
                "u": 0.5 * (x1 + x2),
                "v": 0.5 * (y1 + y2),
                "conf": float(confs[idx]),
                "bbox": (x1, y1, x2, y2),
                "cls_id": cls_id,
                "cls_name": cls_name,
                "source": "onnx",
            }
            if angle is not None:
                cx, cy, w, h = [float(v) for v in boxes_xywh[idx]]
                theta = float(angle[idx])
                kwargs["obb_corners"] = _xywhr_to_corners(cx, cy, w, h, theta)
                kwargs["obb_heading_yaw"] = theta
            detections.append(Detection2D(**kwargs))
        return {
            "detections": detections,
            "raw_prediction_count": raw_prediction_count,
            "conf_pass_count": conf_pass_count,
            "class_pass_count": class_pass_count,
            "nms_keep_count": int(len(keep)),
            "raw_best_conf": raw_best_conf,
            "class_best_conf": class_best_conf,
            "target_raw_count": target_raw_count,
            "target_best_conf": target_best_conf,
            "target_best_area_norm": target_best_area_norm,
            "target_best_u_norm": target_best_u_norm,
            "target_best_v_norm": target_best_v_norm,
            "target_best_center_error_norm": target_best_center_error_norm,
        }

    def _normalize_prediction_array(self, outputs: list[np.ndarray]) -> np.ndarray:
        arrays = [np.asarray(out) for out in outputs if isinstance(out, np.ndarray)]
        if not arrays:
            return np.empty((0, 0), dtype=np.float32)
        pred = max(arrays, key=lambda arr: arr.size)
        if pred.ndim == 4 and pred.shape[0] == 1:
            pred = pred[0]
        if pred.ndim == 3 and pred.shape[0] == 1:
            pred = pred[0]
        if pred.ndim == 3:
            pred = pred.reshape(pred.shape[0], -1)
        if pred.ndim != 2:
            return np.empty((0, 0), dtype=np.float32)

        # Ultralytics raw ONNX exports are typically (C, N) and need transposing.
        if pred.shape[0] < pred.shape[1]:
            pred = pred.T
        return pred.astype(np.float32, copy=False)
