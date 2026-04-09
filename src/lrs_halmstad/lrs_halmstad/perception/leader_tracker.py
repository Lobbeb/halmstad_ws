#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String

from lrs_halmstad.common.node_mixins import EventEmitterMixin
from lrs_halmstad.common.ros_params import yaml_param
from lrs_halmstad.perception.detection_protocol import Detection2D
from lrs_halmstad.perception.detection_status import DetectionNodeMixin, DetectionStatusPublisher
from lrs_halmstad.follow.follow_math import coerce_bool
from lrs_halmstad.perception.onnx_backend import OnnxYoloRuntime, resolve_onnx_backend_name
from lrs_halmstad.perception.runtime_metrics import (
    CsvBenchmarkLogger,
    FrameTiming,
    LatestItemWorker,
    RollingRateTracker,
    perf_counter_ns,
)
from lrs_halmstad.perception.yolo_common import (
    YOLO,
    default_models_root,
    image_to_bgr,
    load_ultralytics_model,
    normalize_inference_device,
    order_quad,
    resolve_inference_device,
    resolve_onnx_model_path,
    resolve_tracker_config,
    resolve_yolo_weights_path,
    stamp_ns,
)


@dataclass
class PendingImage:
    msg: Image
    recv_ros_ns: int
    recv_perf_ns: int
    frame_index: int


class LeaderTracker(DetectionNodeMixin, EventEmitterMixin, Node):
    """Tracker-backed detector with optional ONNX detection backend and async frame dropping."""

    def __init__(self):
        super().__init__("leader_tracker")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.uav_name = str(self.declare_parameter("uav_name", "dji0").value)
        self.camera_topic = str(self.declare_parameter("camera_topic", "").value) or f"/{self.uav_name}/camera0/image_raw"
        self.out_topic = str(yaml_param(self, "out_topic"))
        self.status_topic = str(yaml_param(self, "status_topic"))
        self.models_root = os.path.expanduser(
            str(self.declare_parameter("models_root", default_models_root(__file__)).value).strip()
        )
        self.yolo_weights = resolve_yolo_weights_path(
            str(self.declare_parameter("yolo_weights", "").value).strip(),
            self.models_root,
        )
        self.backend = resolve_onnx_backend_name(str(yaml_param(self, "backend", descriptor=dyn_num)))
        self.onnx_model = resolve_onnx_model_path(
            str(self.declare_parameter("onnx_model", "").value).strip(),
            self.models_root,
            self.yolo_weights,
        )
        self.device_requested = normalize_inference_device(str(yaml_param(self, "device")))
        if self.backend == "ultralytics_cpu" and self.device_requested == "auto":
            self.device_requested = "cpu"
            self.backend = "ultralytics"
        self.device, self.device_fallback_reason = resolve_inference_device(self.device_requested)
        self.conf_threshold = float(yaml_param(self, "conf_threshold", descriptor=dyn_num))
        self.iou_threshold = float(yaml_param(self, "iou_threshold", descriptor=dyn_num))
        self.imgsz = int(yaml_param(self, "imgsz"))
        self.target_class_id = int(yaml_param(self, "target_class_id"))
        self.target_class_name = str(yaml_param(self, "target_class_name")).strip()
        self.predict_hz = float(yaml_param(self, "predict_hz", descriptor=dyn_num))
        self.tracker_config = resolve_tracker_config(str(yaml_param(self, "tracker_config")).strip(), __file__)
        self.event_topic = str(yaml_param(self, "event_topic"))
        self.publish_events = coerce_bool(yaml_param(self, "publish_events"))
        self.async_inference = coerce_bool(yaml_param(self, "async_inference"))
        self.latest_frame_only = coerce_bool(yaml_param(self, "latest_frame_only"))
        self.stale_detection_threshold_ms = float(yaml_param(self, "stale_detection_threshold_ms", descriptor=dyn_num))
        self.metrics_window_s = float(yaml_param(self, "metrics_window_s", descriptor=dyn_num))
        self.benchmark_csv_path = str(yaml_param(self, "benchmark_csv_path")).strip()
        self.image_qos_depth = max(1, int(yaml_param(self, "image_qos_depth", descriptor=dyn_num)))
        self.image_qos_reliability = str(yaml_param(self, "image_qos_reliability")).strip().lower()
        self.pseudo_track_max_center_jump_px = float(
            yaml_param(self, "pseudo_track_max_center_jump_px", descriptor=dyn_num)
        )

        if self.predict_hz <= 0.0:
            raise ValueError("predict_hz must be > 0")
        if self.metrics_window_s <= 0.0:
            raise ValueError("metrics_window_s must be > 0")
        if self.stale_detection_threshold_ms < 0.0:
            raise ValueError("stale_detection_threshold_ms must be >= 0")
        if self.backend not in ("ultralytics", "onnx_cpu", "onnx_directml"):
            raise ValueError("backend must be one of: ultralytics, onnx_cpu, onnx_directml")
        if self.async_inference and not self.latest_frame_only:
            self.get_logger().warn(
                "[leader_tracker] async_inference requires latest-frame-only semantics; forcing latest_frame_only=true"
            )
            self.latest_frame_only = True

        self.model = None
        self.onnx_runtime: Optional[OnnxYoloRuntime] = None
        self.task_type = self._resolve_task_type("")
        self.current_provider = self.device if self.backend == "ultralytics" else ""
        self.yolo_error: Optional[str] = None
        self.yolo_ready = False

        self.last_predict_perf_ns: int = 0
        self.active_track_id: Optional[int] = None
        self.active_track_hits = 0
        self.active_track_first_seen_ns: Optional[int] = None
        self.pseudo_track_id = 1000
        self.last_selected_center: Optional[tuple[float, float]] = None
        self.last_selected_cls_id: Optional[int] = None
        self.frame_index = 0
        self.dropped_frame_count = 0
        self.stale_detection_count = 0
        self.last_infer_reason = "none"
        self.last_selection_reason = "none"
        self.last_raw_output_count = 0
        self.last_conf_pass_count = 0
        self.last_class_pass_count = 0
        self.last_nms_keep_count = 0
        self.last_candidate_count = 0
        self.last_raw_best_conf = -1.0
        self.last_class_best_conf = -1.0
        self.last_target_raw_count = 0
        self.last_target_best_conf = -1.0
        self.last_target_best_area_norm = -1.0
        self.last_target_best_u_norm = -1.0
        self.last_target_best_v_norm = -1.0
        self.last_target_best_center_error_norm = -1.0
        self.audit_frames_total = 0
        self.audit_ok_total = 0
        self.audit_no_det_total = 0
        self.audit_decode_fail_total = 0
        self.audit_yolo_disabled_total = 0
        self.audit_infer_fail_total = 0
        self.publish_rate_tracker = RollingRateTracker(self.metrics_window_s)
        self.benchmark_logger = CsvBenchmarkLogger(self.benchmark_csv_path) if self.benchmark_csv_path else None
        self.worker: Optional[LatestItemWorker] = None

        self._init_runtime()

        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.on_image,
            self._image_qos_profile(),
        )
        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.status_helper = DetectionStatusPublisher(self, self.status_topic)
        self._setup_event_emitter(self.event_topic, self.publish_events)

        if self.device_fallback_reason is not None and self.backend == "ultralytics":
            self.get_logger().warn(
                f"[leader_tracker] Requested device '{self.device_requested}' is unavailable; using '{self.device}' instead"
            )
        if self.async_inference:
            self.worker = LatestItemWorker(self._process_pending_image, thread_name="leader_tracker_worker")

        self.get_logger().info(
            "[leader_tracker] Started: "
            f"image={self.camera_topic}, out={self.out_topic}, status={self.status_topic}, "
            f"backend={self.backend}, provider={self.current_provider or 'none'}, "
            f"weights={self.yolo_weights or '<none>'}, onnx_model={self.onnx_model or '<none>'}, "
            f"tracker={self.tracker_config}, predict_hz={self.predict_hz}, "
            f"async_inference={self.async_inference}, latest_frame_only={self.latest_frame_only}"
        )
        self.emit_event("TRACKER_NODE_START")

    def close(self) -> None:
        if self.worker is not None:
            self.worker.close()
            self.worker = None

    def _image_qos_profile(self) -> QoSProfile:
        reliability = (
            QoSReliabilityPolicy.RELIABLE
            if self.image_qos_reliability == "reliable"
            else QoSReliabilityPolicy.BEST_EFFORT
        )
        return QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=self.image_qos_depth,
            reliability=reliability,
            durability=QoSDurabilityPolicy.VOLATILE,
        )

    def _resolve_task_type(self, model_hint_path: str) -> str:
        model_task = getattr(self.model, "task", None)
        if isinstance(model_task, str):
            task = model_task.strip().lower()
            if task == "obb":
                return "obb"
            if task in {"detect", "detection"}:
                return "detection"
        weights_lower = str(model_hint_path or self.yolo_weights or self.onnx_model or "").lower()
        if "/obb/" in weights_lower or weights_lower.endswith("-obb.pt") or weights_lower.endswith("_obb.pt") or weights_lower.endswith("-obb.onnx") or weights_lower.endswith("_obb.onnx"):
            return "obb"
        return "detection"

    def _init_runtime(self) -> None:
        if self.backend == "ultralytics":
            if not self.yolo_weights:
                raise ValueError("yolo_weights must be set for leader_tracker")
            if not os.path.isfile(self.yolo_weights):
                raise ValueError(f"leader_tracker weights not found: {self.yolo_weights}")
            if not os.path.isfile(self.tracker_config):
                raise ValueError(f"leader_tracker tracker_config not found: {self.tracker_config}")
            if YOLO is None:
                raise ValueError("ultralytics is required for leader_tracker")
            self.model = load_ultralytics_model(self.yolo_weights)
            self.task_type = self._resolve_task_type(self.yolo_weights)
            self.current_provider = self.device
            self.yolo_ready = True
            return

        self.task_type = self._resolve_task_type(self.onnx_model)
        self.onnx_runtime = OnnxYoloRuntime(
            onnx_model=self.onnx_model,
            backend=self.backend,
            imgsz=self.imgsz,
            conf_threshold=self.conf_threshold,
            iou_threshold=self.iou_threshold,
            target_class_id=self.target_class_id,
            target_class_name=self.target_class_name,
            task_type=self.task_type,
        )
        self.current_provider = self.onnx_runtime.provider
        self.yolo_ready = True

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

    def _score_untracked_candidate(self, det: Detection2D) -> float:
        score = float(det.conf)
        if self.last_selected_center is not None:
            du = float(det.u) - self.last_selected_center[0]
            dv = float(det.v) - self.last_selected_center[1]
            dpx = math.hypot(du, dv)
            score -= 0.15 * (dpx / max(1.0, self.pseudo_track_max_center_jump_px))
            if dpx > self.pseudo_track_max_center_jump_px:
                score -= 1.0
        if self.last_selected_cls_id is not None and det.cls_id == self.last_selected_cls_id:
            score += 0.05
        return score

    def _assign_pseudo_track_id(self, det: Detection2D) -> Detection2D:
        if self.active_track_id is None or self.last_selected_center is None:
            self.pseudo_track_id += 1
            det.track_id = self.pseudo_track_id
            return det
        dpx = math.hypot(float(det.u) - self.last_selected_center[0], float(det.v) - self.last_selected_center[1])
        if dpx <= self.pseudo_track_max_center_jump_px:
            det.track_id = self.active_track_id
            return det
        self.pseudo_track_id += 1
        det.track_id = self.pseudo_track_id
        return det

    def _choose_candidate(self, candidates: list[Detection2D], stamp_ns_value: int) -> Optional[Detection2D]:
        if not candidates:
            self._reset_track_state()
            self.last_selection_reason = "no_candidates"
            return None
        if self.active_track_id is not None:
            matches = [cand for cand in candidates if cand.track_id == self.active_track_id]
            if matches:
                best = max(matches, key=lambda cand: cand.conf)
                self.last_selection_reason = "active_track_match"
                return self._annotate_track_metadata(best, stamp_ns_value)
        tracked = [cand for cand in candidates if cand.track_id is not None]
        if tracked:
            best = max(tracked, key=lambda cand: cand.conf)
            self.last_selection_reason = "best_tracked_candidate"
            return self._annotate_track_metadata(best, stamp_ns_value)

        best = max(candidates, key=self._score_untracked_candidate)
        best = self._assign_pseudo_track_id(best)
        self.last_selection_reason = "pseudo_track_candidate"
        return self._annotate_track_metadata(best, stamp_ns_value)

    def _track_detection_ultralytics(
        self,
        img_bgr: np.ndarray,
        stamp_ns_value: int,
        timing: FrameTiming,
    ) -> Optional[Detection2D]:
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
            timing.infer_end_ros_ns = int(self.get_clock().now().nanoseconds)
            timing.infer_end_perf_ns = perf_counter_ns()
            timing.postprocess_end_ros_ns = timing.infer_end_ros_ns
            timing.postprocess_end_perf_ns = timing.infer_end_perf_ns
            return None
        timing.infer_end_ros_ns = int(self.get_clock().now().nanoseconds)
        timing.infer_end_perf_ns = perf_counter_ns()
        self.last_raw_output_count = 0
        self.last_conf_pass_count = 0
        self.last_class_pass_count = 0
        self.last_nms_keep_count = 0
        self.last_candidate_count = 0
        self.last_raw_best_conf = -1.0
        self.last_class_best_conf = -1.0
        self.last_target_raw_count = 0
        self.last_target_best_conf = -1.0
        self.last_target_best_area_norm = -1.0
        self.last_target_best_u_norm = -1.0
        self.last_target_best_v_norm = -1.0
        self.last_target_best_center_error_norm = -1.0
        if not results:
            self.last_infer_reason = "no_result"
            self.last_selection_reason = "no_result"
            self._reset_track_state()
            timing.postprocess_end_ros_ns = int(self.get_clock().now().nanoseconds)
            timing.postprocess_end_perf_ns = perf_counter_ns()
            return None
        result = results[0]
        raw_obb_count = 0 if getattr(result, "obb", None) is None else int(len(result.obb))
        raw_box_count = 0 if getattr(result, "boxes", None) is None else int(len(result.boxes))
        candidates = self._collect_obb_candidates(result)
        if not candidates:
            candidates = self._collect_box_candidates(result)
        self.last_raw_output_count = raw_obb_count + raw_box_count
        self.last_conf_pass_count = self.last_raw_output_count
        self.last_class_pass_count = len(candidates)
        self.last_nms_keep_count = len(candidates)
        self.last_candidate_count = len(candidates)
        self.last_raw_best_conf = max((float(cand.conf) for cand in candidates), default=-1.0)
        self.last_class_best_conf = self.last_raw_best_conf
        self.last_target_raw_count = len(candidates)
        self.last_target_best_conf = self.last_class_best_conf
        if candidates:
            best_target = max(candidates, key=lambda cand: float(cand.conf))
            x1, y1, x2, y2 = [float(v) for v in best_target.bbox]
            u = 0.5 * (x1 + x2)
            v = 0.5 * (y1 + y2)
            self.last_target_best_area_norm = max(0.0, (x2 - x1) * (y2 - y1)) / max(1.0, float(img_bgr.shape[0] * img_bgr.shape[1]))
            self.last_target_best_u_norm = u / max(1.0, float(img_bgr.shape[1]))
            self.last_target_best_v_norm = v / max(1.0, float(img_bgr.shape[0]))
            half_diag = max(1e-6, math.hypot(0.5 * float(img_bgr.shape[1]), 0.5 * float(img_bgr.shape[0])))
            self.last_target_best_center_error_norm = math.hypot(
                u - 0.5 * float(img_bgr.shape[1]),
                v - 0.5 * float(img_bgr.shape[0]),
            ) / half_diag
        det = self._choose_candidate(candidates, stamp_ns_value)
        if det is not None:
            self.last_infer_reason = "none"
        elif self.last_raw_output_count <= 0:
            self.last_infer_reason = "no_result"
        elif self.last_class_pass_count <= 0:
            self.last_infer_reason = "no_candidates_after_class_filter"
        else:
            self.last_infer_reason = "candidate_selection_failed"
        timing.postprocess_end_ros_ns = int(self.get_clock().now().nanoseconds)
        timing.postprocess_end_perf_ns = perf_counter_ns()
        return det

    def _track_detection_onnx(
        self,
        img_bgr: np.ndarray,
        stamp_ns_value: int,
        timing: FrameTiming,
    ) -> Optional[Detection2D]:
        if self.onnx_runtime is None:
            self.last_infer_reason = "onnx_runtime_not_ready"
            return None
        try:
            result = self.onnx_runtime.predict(img_bgr)
        except Exception as exc:
            self.last_infer_reason = f"infer_failed:{exc}"
            self.get_logger().warn(f"[leader_tracker] ONNX inference failed: {exc}")
            timing.infer_end_ros_ns = int(self.get_clock().now().nanoseconds)
            timing.infer_end_perf_ns = perf_counter_ns()
            timing.postprocess_end_ros_ns = timing.infer_end_ros_ns
            timing.postprocess_end_perf_ns = timing.infer_end_perf_ns
            return None
        self.current_provider = result.provider
        if timing.infer_start_ros_ns > 0 and timing.infer_start_perf_ns > 0:
            infer_delta_ns = max(0, int(result.infer_end_perf_ns) - int(timing.infer_start_perf_ns))
            post_delta_ns = max(0, int(result.postprocess_end_perf_ns) - int(timing.infer_start_perf_ns))
            timing.infer_end_ros_ns = timing.infer_start_ros_ns + infer_delta_ns
            timing.postprocess_end_ros_ns = timing.infer_start_ros_ns + post_delta_ns
        else:
            timing.infer_end_ros_ns = int(self.get_clock().now().nanoseconds)
            timing.postprocess_end_ros_ns = timing.infer_end_ros_ns
        timing.infer_end_perf_ns = int(result.infer_end_perf_ns)
        timing.postprocess_end_perf_ns = int(result.postprocess_end_perf_ns)
        candidates = list(result.detections)
        for det in candidates:
            det.source = "tracker"
        self.last_raw_output_count = int(result.raw_prediction_count)
        self.last_conf_pass_count = int(result.conf_pass_count)
        self.last_class_pass_count = int(result.class_pass_count)
        self.last_nms_keep_count = int(result.nms_keep_count)
        self.last_candidate_count = len(candidates)
        self.last_raw_best_conf = float(result.raw_best_conf)
        self.last_class_best_conf = float(result.class_best_conf)
        self.last_target_raw_count = int(result.target_raw_count)
        self.last_target_best_conf = float(result.target_best_conf)
        self.last_target_best_area_norm = float(result.target_best_area_norm)
        self.last_target_best_u_norm = float(result.target_best_u_norm)
        self.last_target_best_v_norm = float(result.target_best_v_norm)
        self.last_target_best_center_error_norm = float(result.target_best_center_error_norm)
        det = self._choose_candidate(candidates, stamp_ns_value)
        if det is not None:
            self.last_infer_reason = "none"
        elif self.last_raw_output_count <= 0:
            self.last_infer_reason = "no_raw_predictions"
        elif self.last_conf_pass_count <= 0:
            self.last_infer_reason = "no_candidates_above_conf"
        elif self.last_class_pass_count <= 0:
            self.last_infer_reason = "no_candidates_after_class_filter"
        elif self.last_nms_keep_count <= 0:
            self.last_infer_reason = "no_candidates_after_nms"
        else:
            self.last_infer_reason = "candidate_selection_failed"
        return det

    def _track_detection(self, img_bgr: np.ndarray, stamp_ns_value: int, timing: FrameTiming) -> Optional[Detection2D]:
        if self.backend == "ultralytics":
            return self._track_detection_ultralytics(img_bgr, stamp_ns_value, timing)
        return self._track_detection_onnx(img_bgr, stamp_ns_value, timing)

    def _rate_gate_allows_inference(self) -> bool:
        if self.last_predict_perf_ns <= 0:
            return True
        dt_s = (perf_counter_ns() - self.last_predict_perf_ns) * 1e-9
        return dt_s >= (1.0 / self.predict_hz)

    def on_image(self, msg: Image) -> None:
        pending = PendingImage(
            msg=msg,
            recv_ros_ns=int(self.get_clock().now().nanoseconds),
            recv_perf_ns=perf_counter_ns(),
            frame_index=self.frame_index,
        )
        self.frame_index += 1

        if not self.async_inference:
            self._process_pending_image(pending)
            return

        if self.worker is None:
            return
        replaced = self.worker.submit(pending)
        if replaced and self.latest_frame_only:
            self.dropped_frame_count += 1

    def _status_extras(self, timing: FrameTiming) -> dict[str, object]:
        return {
            "backend": self.backend,
            "provider": self.current_provider or "none",
            "async": self.async_inference,
            "latest_frame_only": self.latest_frame_only,
            "inference_ms": timing.inference_ms if math.isfinite(timing.inference_ms) else -1.0,
            "postprocess_ms": timing.postprocess_ms if math.isfinite(timing.postprocess_ms) else -1.0,
            "latency_ms": (
                timing.camera_to_publish_latency_ms
                if math.isfinite(timing.camera_to_publish_latency_ms)
                else -1.0
            ),
            "detector_hz": timing.detector_hz,
            "dropped_frames": self.dropped_frame_count,
            "stale_detections": self.stale_detection_count,
            "frame_index": timing.frame_index,
            "raw_outputs": self.last_raw_output_count,
            "raw_best_conf": self.last_raw_best_conf,
            "conf_pass": self.last_conf_pass_count,
            "class_best_conf": self.last_class_best_conf,
            "target_raw_count": self.last_target_raw_count,
            "target_best_conf": self.last_target_best_conf,
            "target_best_area_norm": self.last_target_best_area_norm,
            "target_best_u_norm": self.last_target_best_u_norm,
            "target_best_v_norm": self.last_target_best_v_norm,
            "target_best_center_error_norm": self.last_target_best_center_error_norm,
            "class_pass": self.last_class_pass_count,
            "nms_keep": self.last_nms_keep_count,
            "candidates": self.last_candidate_count,
            "selection": self.last_selection_reason,
            "audit_frames": self.audit_frames_total,
            "audit_ok": self.audit_ok_total,
            "audit_no_det": self.audit_no_det_total,
            "audit_decode_fail": self.audit_decode_fail_total,
            "audit_yolo_disabled": self.audit_yolo_disabled_total,
            "audit_infer_fail": self.audit_infer_fail_total,
        }

    def _publish_result(
        self,
        pending: PendingImage,
        det: Optional[Detection2D],
        timing: FrameTiming,
        *,
        state: str,
        reason: str,
    ) -> None:
        self.audit_frames_total += 1
        timing.publish_ros_ns = int(self.get_clock().now().nanoseconds)
        timing.publish_perf_ns = perf_counter_ns()
        timing.provider = self.current_provider or ""
        self.publish_rate_tracker.record(timing.publish_ros_ns)
        timing.detector_hz = self.publish_rate_tracker.hz(timing.publish_ros_ns)
        timing.dropped_frames_total = self.dropped_frame_count
        if (
            self.stale_detection_threshold_ms > 0.0
            and math.isfinite(timing.camera_to_publish_latency_ms)
            and timing.camera_to_publish_latency_ms > self.stale_detection_threshold_ms
        ):
            self.stale_detection_count += 1
            timing.stale_detection = True
        timing.stale_detections_total = self.stale_detection_count
        timing.valid_detection = det is not None
        timing.reason = reason
        infer_failed = "infer_failed" in str(reason)
        if state == "OK":
            self.audit_ok_total += 1
        elif state == "NO_DET":
            self.audit_no_det_total += 1
        elif state == "DECODE_FAIL":
            self.audit_decode_fail_total += 1
        elif state == "YOLO_DISABLED":
            self.audit_yolo_disabled_total += 1
        if infer_failed:
            self.audit_infer_fail_total += 1

        if det is not None:
            self.last_selected_center = (det.u, det.v)
            self.last_selected_cls_id = det.cls_id

        self._publish_detection(pending.msg, det, metadata=timing.metadata())
        self._publish_status(state, reason, det, extras=self._status_extras(timing))
        if self.benchmark_logger is not None:
            self.benchmark_logger.log(timing)

    def _process_pending_image(self, pending: PendingImage) -> None:
        try:
            if not self._rate_gate_allows_inference():
                return
            self.last_predict_perf_ns = perf_counter_ns()
            timing = FrameTiming(
                source_stamp_ns=stamp_ns(pending.msg),
                frame_recv_ros_ns=int(pending.recv_ros_ns),
                frame_recv_perf_ns=int(pending.recv_perf_ns),
                backend=self.backend,
                provider=self.current_provider or "",
                frame_index=int(pending.frame_index),
            )
            if not self.yolo_ready:
                self._publish_result(
                    pending,
                    None,
                    timing,
                    state="YOLO_DISABLED",
                    reason=self.yolo_error or "yolo_not_ready",
                )
                return

            img_bgr = image_to_bgr(pending.msg)
            if img_bgr is None:
                self._publish_result(
                    pending,
                    None,
                    timing,
                    state="DECODE_FAIL",
                    reason="image_decode_failed",
                )
                return

            timing.infer_start_ros_ns = int(self.get_clock().now().nanoseconds)
            timing.infer_start_perf_ns = perf_counter_ns()
            det = self._track_detection(img_bgr, stamp_ns(pending.msg), timing)
            if timing.infer_end_ros_ns <= 0:
                timing.infer_end_ros_ns = int(self.get_clock().now().nanoseconds)
            if timing.infer_end_perf_ns <= 0:
                timing.infer_end_perf_ns = perf_counter_ns()
            if timing.postprocess_end_ros_ns <= 0:
                timing.postprocess_end_ros_ns = int(self.get_clock().now().nanoseconds)
            if timing.postprocess_end_perf_ns <= 0:
                timing.postprocess_end_perf_ns = perf_counter_ns()

            if det is not None:
                self._publish_result(pending, det, timing, state="OK", reason="none")
                return
            self._publish_result(
                pending,
                None,
                timing,
                state="NO_DET",
                reason=self.last_infer_reason or "no_detection",
            )
        except Exception as exc:
            self.get_logger().warn(f"[leader_tracker] Processing failed: {exc}")


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
        try:
            node.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
