#!/usr/bin/env python3
from __future__ import annotations

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
    resolve_tracker_config,
    resolve_yolo_weights_path,
    stamp_ns,
)


class LeaderTracker(DetectionNodeMixin, EventEmitterMixin, Node):
    """Ultralytics track-mode wrapper publishing tracked detections on /coord/leader_detection."""

    def __init__(self):
        super().__init__("leader_tracker")
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
        self.device = str(yaml_param(self, "device")).strip() or "cpu"
        self.conf_threshold = float(yaml_param(self, "conf_threshold", descriptor=dyn_num))
        self.iou_threshold = float(yaml_param(self, "iou_threshold", descriptor=dyn_num))
        self.imgsz = int(yaml_param(self, "imgsz"))
        self.target_class_id = int(yaml_param(self, "target_class_id"))
        self.target_class_name = str(yaml_param(self, "target_class_name")).strip()
        self.predict_hz = float(yaml_param(self, "predict_hz", descriptor=dyn_num))
        self.tracker_config = resolve_tracker_config(
            str(yaml_param(self, "tracker_config")).strip(),
            __file__,
        )
        self.event_topic = str(yaml_param(self, "event_topic"))
        self.publish_events = bool(yaml_param(self, "publish_events"))

        if self.predict_hz <= 0.0:
            raise ValueError("predict_hz must be > 0")
        if not self.yolo_weights:
            raise ValueError("yolo_weights must be set for leader_tracker")
        if not os.path.isfile(self.yolo_weights):
            raise ValueError(f"leader_tracker weights not found: {self.yolo_weights}")
        if not os.path.isfile(self.tracker_config):
            raise ValueError(f"leader_tracker tracker_config not found: {self.tracker_config}")
        if YOLO is None:
            raise ValueError("ultralytics is required for leader_tracker")

        self.model = load_ultralytics_model(self.yolo_weights)
        self.task_type = self._resolve_task_type()
        self.last_predict_time: Optional[Time] = None
        self.active_track_id: Optional[int] = None
        self.active_track_hits = 0
        self.active_track_first_seen_ns: Optional[int] = None
        self.last_infer_reason = "none"

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.on_image, 10)
        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.status_helper = DetectionStatusPublisher(self, self.status_topic)
        self._setup_event_emitter(self.event_topic, self.publish_events)

        self.get_logger().info(
            "[leader_tracker] Started: "
            f"image={self.camera_topic}, out={self.out_topic}, status={self.status_topic}, "
            f"weights={self.yolo_weights}, tracker={self.tracker_config}, device={self.device}, predict_hz={self.predict_hz}"
        )
        self.emit_event("TRACKER_NODE_START")

    def _resolve_task_type(self) -> str:
        model_task = getattr(self.model, "task", None)
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

    @staticmethod
    def _extract_track_id(item) -> Optional[int]:
        raw = getattr(item, "id", None)
        if raw is None:
            return None
        try:
            if hasattr(raw, "tolist"):
                raw = raw.tolist()
            if isinstance(raw, (list, tuple)):
                raw = raw[0]
            return int(raw)
        except Exception:
            return None

    def _candidate_ok(self, cls_id: Optional[int], cls_name: str) -> bool:
        if self.target_class_id >= 0 and cls_id != self.target_class_id:
            return False
        if self.target_class_name and cls_name.lower() != self.target_class_name.lower():
            return False
        return True

    def _collect_obb_candidates(self, result) -> list[Detection2D]:
        obb = getattr(result, "obb", None)
        if obb is None or len(obb) == 0:
            return []
        names = getattr(result, "names", {}) or {}
        candidates: list[Detection2D] = []
        for item in obb:
            try:
                xyxy = item.xyxy[0].tolist()
                x1, y1, x2, y2 = [float(v) for v in xyxy]
                conf = float(item.conf[0]) if getattr(item, "conf", None) is not None else 0.0
                cls_id = int(item.cls[0]) if getattr(item, "cls", None) is not None else None
                cls_name = str(names.get(cls_id, "")) if cls_id is not None else ""
                corners_arr = np.asarray(item.xyxyxyxy[0].tolist(), dtype=np.float64).reshape(4, 2)
                corners = tuple((float(pt[0]), float(pt[1])) for pt in order_quad(corners_arr))
                track_id = self._extract_track_id(item)
            except Exception:
                continue
            if not self._candidate_ok(cls_id, cls_name):
                continue
            candidates.append(
                Detection2D(
                    u=0.5 * (x1 + x2),
                    v=0.5 * (y1 + y2),
                    conf=conf,
                    bbox=(x1, y1, x2, y2),
                    cls_id=cls_id,
                    cls_name=cls_name,
                    track_id=track_id,
                    obb_corners=corners,
                    source="tracker",
                )
            )
        return candidates

    def _collect_box_candidates(self, result) -> list[Detection2D]:
        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            return []
        names = getattr(result, "names", {}) or {}
        candidates: list[Detection2D] = []
        for item in boxes:
            try:
                xyxy = item.xyxy[0].tolist()
                x1, y1, x2, y2 = [float(v) for v in xyxy]
                conf = float(item.conf[0]) if getattr(item, "conf", None) is not None else 0.0
                cls_id = int(item.cls[0]) if getattr(item, "cls", None) is not None else None
                cls_name = str(names.get(cls_id, "")) if cls_id is not None else ""
                track_id = self._extract_track_id(item)
            except Exception:
                continue
            if not self._candidate_ok(cls_id, cls_name):
                continue
            candidates.append(
                Detection2D(
                    u=0.5 * (x1 + x2),
                    v=0.5 * (y1 + y2),
                    conf=conf,
                    bbox=(x1, y1, x2, y2),
                    cls_id=cls_id,
                    cls_name=cls_name,
                    track_id=track_id,
                    source="tracker",
                )
            )
        return candidates

    def _reset_track_state(self) -> None:
        self.active_track_id = None
        self.active_track_hits = 0
        self.active_track_first_seen_ns = None

    def _track_age_s(self, stamp_ns_value: int) -> float:
        if self.active_track_first_seen_ns is None:
            return 0.0
        return max(0.0, (int(stamp_ns_value) - int(self.active_track_first_seen_ns)) * 1e-9)

    def _annotate_track_metadata(self, det: Detection2D, stamp_ns_value: int) -> Detection2D:
        if det.track_id is None:
            det.track_hits = 0
            det.track_age_s = 0.0
            det.track_state = "raw"
            det.track_switched = False
            self._reset_track_state()
            return det

        prev_track_id = self.active_track_id
        track_changed = prev_track_id is not None and det.track_id != prev_track_id
        if det.track_id != prev_track_id:
            self.active_track_id = det.track_id
            self.active_track_hits = 1
            self.active_track_first_seen_ns = int(stamp_ns_value)
        else:
            self.active_track_hits += 1

        det.track_hits = self.active_track_hits
        det.track_age_s = self._track_age_s(stamp_ns_value)
        det.track_state = "reacquire" if self.active_track_hits <= 1 else "tracked"
        det.track_switched = track_changed
        return det

    def _choose_candidate(self, candidates: list[Detection2D], stamp_ns_value: int) -> Optional[Detection2D]:
        if not candidates:
            self._reset_track_state()
            return None
        if self.active_track_id is not None:
            matches = [cand for cand in candidates if cand.track_id == self.active_track_id]
            if matches:
                best = max(matches, key=lambda cand: cand.conf)
                return self._annotate_track_metadata(best, stamp_ns_value)
        tracked = [cand for cand in candidates if cand.track_id is not None]
        if tracked:
            best = max(tracked, key=lambda cand: cand.conf)
            return self._annotate_track_metadata(best, stamp_ns_value)
        best = max(candidates, key=lambda cand: cand.conf)
        return self._annotate_track_metadata(best, stamp_ns_value)

    def _track_detection(self, img_bgr: np.ndarray, stamp_ns_value: int) -> Optional[Detection2D]:
        try:
            results = self.model.track(
                source=img_bgr,
                persist=True,
                tracker=self.tracker_config,
                verbose=False,
                conf=self.conf_threshold,
                iou=self.iou_threshold,
                imgsz=self.imgsz,
                device=self.device,
            )
        except Exception as exc:
            self.last_infer_reason = f"infer_failed:{exc}"
            self.get_logger().warn(f"[leader_tracker] Ultralytics track() failed: {exc}")
            return None
        if not results:
            self.last_infer_reason = "no_detection"
            self._reset_track_state()
            return None
        result = results[0]
        candidates = self._collect_obb_candidates(result)
        if not candidates:
            candidates = self._collect_box_candidates(result)
        det = self._choose_candidate(candidates, stamp_ns_value)
        self.last_infer_reason = "none" if det is not None else "no_detection"
        return det

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
        det = self._track_detection(img_bgr, stamp_ns(msg))
        self._publish_detection(msg, det)
        if det is not None:
            self._publish_status("OK", "none", det)
        else:
            self._publish_status("NO_DET", self.last_infer_reason, None)


def main(args=None):
    rclpy.init(args=args)
    node = LeaderTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.emit_event("TRACKER_NODE_SHUTDOWN")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
