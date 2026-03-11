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

from lrs_halmstad.perception.detection_protocol import Detection2D, encode_detection_payload
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


class LeaderTracker(Node):
    """Ultralytics track-mode wrapper publishing tracked detections on /coord/leader_detection."""

    def __init__(self):
        super().__init__("leader_tracker")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("camera_topic", "")
        self.declare_parameter("out_topic", "/coord/leader_detection")
        self.declare_parameter("models_root", default_models_root(__file__))
        self.declare_parameter("yolo_weights", "")
        self.declare_parameter("device", "cpu")
        self.declare_parameter("conf_threshold", 0.05, dyn_num)
        self.declare_parameter("iou_threshold", 0.45, dyn_num)
        self.declare_parameter("imgsz", 640)
        self.declare_parameter("target_class_id", -1)
        self.declare_parameter("target_class_name", "")
        self.declare_parameter("predict_hz", 20.0, dyn_num)
        self.declare_parameter("tracker_config", "botsort.yaml")
        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("publish_events", False)

        self.uav_name = str(self.get_parameter("uav_name").value)
        self.camera_topic = str(self.get_parameter("camera_topic").value) or f"/{self.uav_name}/camera0/image_raw"
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.models_root = os.path.expanduser(str(self.get_parameter("models_root").value).strip())
        self.yolo_weights = resolve_yolo_weights_path(
            str(self.get_parameter("yolo_weights").value).strip(),
            self.models_root,
        )
        self.device = str(self.get_parameter("device").value).strip() or "cpu"
        self.conf_threshold = float(self.get_parameter("conf_threshold").value)
        self.iou_threshold = float(self.get_parameter("iou_threshold").value)
        self.imgsz = int(self.get_parameter("imgsz").value)
        self.target_class_id = int(self.get_parameter("target_class_id").value)
        self.target_class_name = str(self.get_parameter("target_class_name").value).strip()
        self.predict_hz = float(self.get_parameter("predict_hz").value)
        self.tracker_config = resolve_tracker_config(
            str(self.get_parameter("tracker_config").value).strip(),
            __file__,
        )
        self.event_topic = str(self.get_parameter("event_topic").value)
        self.publish_events = bool(self.get_parameter("publish_events").value)

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
        self.last_predict_time: Optional[Time] = None
        self.active_track_id: Optional[int] = None

        self.image_sub = self.create_subscription(Image, self.camera_topic, self.on_image, 10)
        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.events_pub = self.create_publisher(String, self.event_topic, 10)

        self.get_logger().info(
            "[leader_tracker] Started: "
            f"image={self.camera_topic}, out={self.out_topic}, "
            f"weights={self.yolo_weights}, tracker={self.tracker_config}, device={self.device}, predict_hz={self.predict_hz}"
        )
        self.emit_event("TRACKER_NODE_START")

    def emit_event(self, name: str) -> None:
        if not self.publish_events:
            return
        msg = String()
        msg.data = str(name)
        self.events_pub.publish(msg)

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

    def _choose_candidate(self, candidates: list[Detection2D]) -> Optional[Detection2D]:
        if not candidates:
            self.active_track_id = None
            return None
        if self.active_track_id is not None:
            matches = [cand for cand in candidates if cand.track_id == self.active_track_id]
            if matches:
                return max(matches, key=lambda cand: cand.conf)
        tracked = [cand for cand in candidates if cand.track_id is not None]
        if tracked:
            best = max(tracked, key=lambda cand: cand.conf)
            self.active_track_id = best.track_id
            return best
        self.active_track_id = None
        return max(candidates, key=lambda cand: cand.conf)

    def _track_detection(self, img_bgr: np.ndarray) -> Optional[Detection2D]:
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
            self.get_logger().warn(f"[leader_tracker] Ultralytics track() failed: {exc}")
            return None
        if not results:
            self.active_track_id = None
            return None
        result = results[0]
        candidates = self._collect_obb_candidates(result)
        if not candidates:
            candidates = self._collect_box_candidates(result)
        return self._choose_candidate(candidates)

    def _publish_detection(self, msg: Image, det: Optional[Detection2D]) -> None:
        out = String()
        out.data = encode_detection_payload(stamp_ns(msg), det)
        self.pub.publish(out)

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
            return
        det = self._track_detection(img_bgr)
        self._publish_detection(msg, det)


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
