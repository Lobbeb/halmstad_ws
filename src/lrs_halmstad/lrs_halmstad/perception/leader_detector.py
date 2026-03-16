#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from typing import Optional, Tuple

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from std_msgs.msg import String

from lrs_halmstad.common.node_mixins import EventEmitterMixin
from lrs_halmstad.common.ros_params import yaml_param
from lrs_halmstad.perception.detection_protocol import Detection2D
from lrs_halmstad.perception.detection_status import DetectionNodeMixin, DetectionStatusPublisher
from lrs_halmstad.perception.yolo_common import (
    YOLO,
    default_models_root,
    image_to_bgr,
    load_ultralytics_model,
    order_quad,
    resolve_yolo_weights_path,
)


class LeaderDetector(DetectionNodeMixin, EventEmitterMixin, Node):
    def __init__(self):
        super().__init__("leader_detector")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.uav_name = str(self.declare_parameter("uav_name", "dji0").value)
        self.camera_topic = str(self.declare_parameter("camera_topic", "").value) or f"/{self.uav_name}/camera0/image_raw"
        self.out_topic = str(yaml_param(self, "out_topic"))
        self.status_topic = str(yaml_param(self, "status_topic"))
        self.models_root = os.path.expanduser(str(self.declare_parameter("models_root", default_models_root(__file__)).value).strip())
        self.yolo_weights = resolve_yolo_weights_path(
            str(self.declare_parameter("yolo_weights", "").value).strip(),
            self.models_root,
        )
        self.device = str(yaml_param(self, "device"))
        self.conf_threshold = float(yaml_param(self, "conf_threshold", descriptor=dyn_num))
        self.iou_threshold = float(yaml_param(self, "iou_threshold", descriptor=dyn_num))
        self.imgsz = int(yaml_param(self, "imgsz"))
        self.target_class_id = int(yaml_param(self, "target_class_id"))
        self.target_class_name = str(yaml_param(self, "target_class_name")).strip()
        self.bbox_continuity_weight = float(yaml_param(self, "bbox_continuity_weight", descriptor=dyn_num))
        self.bbox_continuity_class_bonus = float(yaml_param(self, "bbox_continuity_class_bonus", descriptor=dyn_num))
        self.bbox_continuity_max_px = float(yaml_param(self, "bbox_continuity_max_px", descriptor=dyn_num))
        self.predict_hz = float(yaml_param(self, "predict_hz", descriptor=dyn_num))
        self.event_topic = str(yaml_param(self, "event_topic"))
        self.publish_events = bool(yaml_param(self, "publish_events"))

        if self.predict_hz <= 0.0:
            raise ValueError("predict_hz must be > 0")

        self.last_det_center: Optional[Tuple[float, float]] = None
        self.last_det_cls_id: Optional[int] = None
        self.last_predict_time: Optional[Time] = None
        self.yolo_model = None
        self.yolo_ready = False
        self.yolo_error: Optional[str] = None
        self.task_type = self._resolve_task_type()
        self._init_yolo()

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.on_image, 10)
        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.status_helper = DetectionStatusPublisher(self, self.status_topic)
        self._setup_event_emitter(self.event_topic, self.publish_events)

        self.get_logger().info(
            "[leader_detector] Started: "
            f"image={self.camera_topic}, out={self.out_topic}, status={self.status_topic}, "
            f"yolo_weights={self.yolo_weights or '<none>'}, "
            f"device={self.device}, predict_hz={self.predict_hz}"
        )
        self.emit_event("DETECTOR_NODE_START")

    def _init_yolo_ultralytics(self) -> bool:
        try:
            self.yolo_model = load_ultralytics_model(self.yolo_weights)
            self.yolo_ready = True
            self.task_type = self._resolve_task_type()
            return True
        except RuntimeError as exc:
            self.yolo_error = str(exc)
            return False
        except Exception as exc:
            self.yolo_error = f"yolo_load_failed_ultralytics:{exc}"
            self.get_logger().warn(f"[leader_detector] Failed to load ultralytics weights '{self.yolo_weights}': {exc}")
            return False

    def _init_yolo(self) -> None:
        if not self.yolo_weights:
            self.yolo_error = "yolo_weights_not_set"
            return
        if not os.path.isfile(self.yolo_weights):
            self.yolo_error = f"file_not_found:{self.yolo_weights}"
            self.get_logger().warn(f"[leader_detector] YOLO weights file not found: {self.yolo_weights}")
            return
        self._init_yolo_ultralytics()

    def _resolve_task_type(self) -> str:
        model_task = getattr(self.yolo_model, "task", None)
        if isinstance(model_task, str):
            task = model_task.strip().lower()
            if task == "obb":
                return "obb"
            if task in {"detect", "detection"}:
                return "detection"
        weights_lower = str(self.yolo_weights or "").lower()
        if "/obb/" in weights_lower or weights_lower.endswith("-obb.pt") or weights_lower.endswith("_obb.pt"):
            return "obb"
        return "detection"

    def _score_candidate(self, cand: Detection2D) -> float:
        score = cand.conf
        if self.last_det_center is not None:
            du = cand.u - self.last_det_center[0]
            dv = cand.v - self.last_det_center[1]
            dpx = math.hypot(du, dv)
            if self.bbox_continuity_max_px > 0.0 and dpx > self.bbox_continuity_max_px:
                score -= 1.0
            if self.bbox_continuity_weight > 0.0:
                score -= self.bbox_continuity_weight * (dpx / max(1.0, self.bbox_continuity_max_px))
            if self.last_det_cls_id is not None and cand.cls_id == self.last_det_cls_id:
                score += self.bbox_continuity_class_bonus
        return score

    def _candidate_ok(self, cls_id: Optional[int], cls_name: str) -> bool:
        if self.target_class_id >= 0 and cls_id != self.target_class_id:
            return False
        if self.target_class_name and cls_name.lower() != self.target_class_name.lower():
            return False
        return True

    def _pick_detection_ultralytics_obb(self, result) -> Optional[Detection2D]:
        obb = getattr(result, "obb", None)
        if obb is None or len(obb) == 0:
            return None
        names = getattr(result, "names", {}) or {}

        best: Optional[Detection2D] = None
        best_score = -1e9
        for b in obb:
            try:
                xyxy = b.xyxy[0].tolist()
                x1, y1, x2, y2 = [float(v) for v in xyxy]
                conf = float(b.conf[0]) if getattr(b, "conf", None) is not None else 0.0
                cls_id = int(b.cls[0]) if getattr(b, "cls", None) is not None else None
                cls_name = str(names.get(cls_id, "")) if cls_id is not None else ""
                corners_arr = np.asarray(b.xyxyxyxy[0].tolist(), dtype=np.float64).reshape(4, 2)
                corners = tuple((float(pt[0]), float(pt[1])) for pt in order_quad(corners_arr))
            except Exception:
                continue
            if not self._candidate_ok(cls_id, cls_name):
                continue
            cand = Detection2D(
                u=0.5 * (x1 + x2),
                v=0.5 * (y1 + y2),
                conf=conf,
                bbox=(x1, y1, x2, y2),
                cls_id=cls_id,
                cls_name=cls_name,
                obb_corners=corners,
                source="detector",
            )
            score = self._score_candidate(cand)
            if best is None or score > best_score:
                best = cand
                best_score = score
        return best

    def _pick_detection_ultralytics(self, img_bgr: np.ndarray) -> Optional[Detection2D]:
        try:
            results = self.yolo_model.predict(
                source=img_bgr,
                verbose=False,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
            )
        except Exception as exc:
            self.yolo_error = f"infer_failed_ultralytics:{exc}"
            self.get_logger().warn(f"[leader_detector] YOLO inference failed (ultralytics): {exc}")
            return None
        if not results:
            return None

        result = results[0]
        obb = getattr(result, "obb", None)
        if obb is not None and len(obb) > 0:
            return self._pick_detection_ultralytics_obb(result)

        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return None
        names = getattr(result, "names", {}) or {}

        best: Optional[Detection2D] = None
        best_score = -1e9
        for b in boxes:
            try:
                xyxy = b.xyxy[0].tolist()
                x1, y1, x2, y2 = [float(v) for v in xyxy]
                conf = float(b.conf[0]) if getattr(b, "conf", None) is not None else 0.0
                cls_id = int(b.cls[0]) if getattr(b, "cls", None) is not None else None
                cls_name = str(names.get(cls_id, "")) if cls_id is not None else ""
            except Exception:
                continue
            if not self._candidate_ok(cls_id, cls_name):
                continue
            cand = Detection2D(
                u=0.5 * (x1 + x2),
                v=0.5 * (y1 + y2),
                conf=conf,
                bbox=(x1, y1, x2, y2),
                cls_id=cls_id,
                cls_name=cls_name,
                source="detector",
            )
            score = self._score_candidate(cand)
            if best is None or score > best_score:
                best = cand
                best_score = score
        return best

    def _pick_detection(self, img_bgr: np.ndarray) -> Optional[Detection2D]:
        if not self.yolo_ready or self.yolo_model is None:
            return None
        return self._pick_detection_ultralytics(img_bgr)

    def on_image(self, msg: Image) -> None:
        now = self.get_clock().now()
        if self.last_predict_time is not None:
            dt = (now - self.last_predict_time).nanoseconds * 1e-9
            if dt < (1.0 / self.predict_hz):
                return
        self.last_predict_time = now

        img_bgr = image_to_bgr(msg)
        if img_bgr is None:
            self._publish_detection(msg, None)
            self._publish_status("DECODE_FAIL", "image_decode_failed", None)
            return
        if not self.yolo_ready or self.yolo_model is None:
            self._publish_detection(msg, None)
            self._publish_status("YOLO_DISABLED", self.yolo_error or "yolo_not_ready", None)
            return
        det = self._pick_detection(img_bgr)
        if det is not None:
            self.last_det_center = (det.u, det.v)
            self.last_det_cls_id = det.cls_id
            self.yolo_error = None
        self._publish_detection(msg, det)
        if det is not None:
            self._publish_status("OK", "none", det)
        else:
            self._publish_status("NO_DET", self.yolo_error or "no_detection", None)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.emit_event("DETECTOR_NODE_SHUTDOWN")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
