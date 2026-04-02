#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Optional, Tuple

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
    resolve_yolo_weights_path,
    stamp_ns,
)


@dataclass
class PendingImage:
    msg: Image
    recv_ros_ns: int
    recv_perf_ns: int
    frame_index: int


class LeaderDetector(DetectionNodeMixin, EventEmitterMixin, Node):
    def __init__(self):
        super().__init__("leader_detector")
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
        self.bbox_continuity_weight = float(yaml_param(self, "bbox_continuity_weight", descriptor=dyn_num))
        self.bbox_continuity_class_bonus = float(yaml_param(self, "bbox_continuity_class_bonus", descriptor=dyn_num))
        self.bbox_continuity_max_px = float(yaml_param(self, "bbox_continuity_max_px", descriptor=dyn_num))
        self.predict_hz = float(yaml_param(self, "predict_hz", descriptor=dyn_num))
        self.event_topic = str(yaml_param(self, "event_topic"))
        self.publish_events = bool(yaml_param(self, "publish_events"))
        self.async_inference = bool(yaml_param(self, "async_inference"))
        self.latest_frame_only = bool(yaml_param(self, "latest_frame_only"))
        self.stale_detection_threshold_ms = float(yaml_param(self, "stale_detection_threshold_ms", descriptor=dyn_num))
        self.metrics_window_s = float(yaml_param(self, "metrics_window_s", descriptor=dyn_num))
        self.benchmark_csv_path = str(yaml_param(self, "benchmark_csv_path")).strip()
        self.image_qos_depth = max(1, int(yaml_param(self, "image_qos_depth", descriptor=dyn_num)))
        self.image_qos_reliability = str(yaml_param(self, "image_qos_reliability")).strip().lower()

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
                "[leader_detector] async_inference requires latest-frame-only semantics; forcing latest_frame_only=true"
            )
            self.latest_frame_only = True

        self.last_det_center: Optional[Tuple[float, float]] = None
        self.last_det_cls_id: Optional[int] = None
        self.last_predict_perf_ns: int = 0
        self.frame_index = 0
        self.dropped_frame_count = 0
        self.stale_detection_count = 0
        self.last_infer_reason = "none"
        self.current_provider = self.device if self.backend == "ultralytics" else ""
        self.publish_rate_tracker = RollingRateTracker(self.metrics_window_s)
        self.benchmark_logger = CsvBenchmarkLogger(self.benchmark_csv_path) if self.benchmark_csv_path else None
        self.worker: Optional[LatestItemWorker] = None

        self.yolo_model = None
        self.yolo_ready = False
        self.yolo_error: Optional[str] = None
        self.task_type = self._resolve_task_type("")
        self.onnx_runtime: Optional[OnnxYoloRuntime] = None
        self._init_runtime()

        if self.device_fallback_reason is not None and self.backend == "ultralytics":
            self.get_logger().warn(
                f"[leader_detector] Requested device '{self.device_requested}' is unavailable; using '{self.device}' instead"
            )

        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.on_image,
            self._image_qos_profile(),
        )
        self.pub = self.create_publisher(String, self.out_topic, 10)
        self.status_helper = DetectionStatusPublisher(self, self.status_topic)
        self._setup_event_emitter(self.event_topic, self.publish_events)

        if self.async_inference:
            self.worker = LatestItemWorker(self._process_pending_image, thread_name="leader_detector_worker")

        self.get_logger().info(
            "[leader_detector] Started: "
            f"image={self.camera_topic}, out={self.out_topic}, status={self.status_topic}, "
            f"backend={self.backend}, provider={self.current_provider or 'none'}, "
            f"yolo_weights={self.yolo_weights or '<none>'}, onnx_model={self.onnx_model or '<none>'}, "
            f"predict_hz={self.predict_hz}, async_inference={self.async_inference}, latest_frame_only={self.latest_frame_only}"
        )
        self.emit_event("DETECTOR_NODE_START")

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
        model_task = getattr(self.yolo_model, "task", None)
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

    def _init_yolo_ultralytics(self) -> bool:
        try:
            self.yolo_model = load_ultralytics_model(self.yolo_weights)
            self.yolo_ready = True
            self.task_type = self._resolve_task_type(self.yolo_weights)
            self.current_provider = self.device
            return True
        except RuntimeError as exc:
            self.yolo_error = str(exc)
            return False
        except Exception as exc:
            self.yolo_error = f"yolo_load_failed_ultralytics:{exc}"
            self.get_logger().warn(f"[leader_detector] Failed to load ultralytics weights '{self.yolo_weights}': {exc}")
            return False

    def _init_runtime(self) -> None:
        if self.backend == "ultralytics":
            if not self.yolo_weights:
                self.yolo_error = "yolo_weights_not_set"
                return
            if not os.path.isfile(self.yolo_weights):
                self.yolo_error = f"file_not_found:{self.yolo_weights}"
                self.get_logger().warn(f"[leader_detector] YOLO weights file not found: {self.yolo_weights}")
                return
            self._init_yolo_ultralytics()
            return

        self.task_type = self._resolve_task_type(self.onnx_model)
        try:
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
            self.yolo_ready = True
            self.current_provider = self.onnx_runtime.provider
        except Exception as exc:
            self.yolo_error = f"onnx_runtime_init_failed:{exc}"
            self.get_logger().warn(f"[leader_detector] Failed to initialise ONNX backend '{self.onnx_model}': {exc}")

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

    def _pick_detection_ultralytics(self, img_bgr: np.ndarray, timing: FrameTiming) -> Optional[Detection2D]:
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
            timing.infer_end_ros_ns = int(self.get_clock().now().nanoseconds)
            timing.infer_end_perf_ns = perf_counter_ns()
            timing.postprocess_end_ros_ns = timing.infer_end_ros_ns
            timing.postprocess_end_perf_ns = timing.infer_end_perf_ns
            return None
        timing.infer_end_ros_ns = int(self.get_clock().now().nanoseconds)
        timing.infer_end_perf_ns = perf_counter_ns()
        if not results:
            timing.postprocess_end_ros_ns = int(self.get_clock().now().nanoseconds)
            timing.postprocess_end_perf_ns = perf_counter_ns()
            return None

        result = results[0]
        obb = getattr(result, "obb", None)
        if obb is not None and len(obb) > 0:
            det = self._pick_detection_ultralytics_obb(result)
            timing.postprocess_end_ros_ns = int(self.get_clock().now().nanoseconds)
            timing.postprocess_end_perf_ns = perf_counter_ns()
            return det

        boxes = getattr(result, "boxes", None)
        if boxes is None or len(boxes) == 0:
            timing.postprocess_end_ros_ns = int(self.get_clock().now().nanoseconds)
            timing.postprocess_end_perf_ns = perf_counter_ns()
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
        timing.postprocess_end_ros_ns = int(self.get_clock().now().nanoseconds)
        timing.postprocess_end_perf_ns = perf_counter_ns()
        return best

    def _pick_detection_onnx(self, img_bgr: np.ndarray, timing: FrameTiming) -> Optional[Detection2D]:
        if self.onnx_runtime is None:
            self.yolo_error = "onnx_runtime_not_ready"
            return None
        try:
            result = self.onnx_runtime.predict(img_bgr)
        except Exception as exc:
            self.yolo_error = f"infer_failed_onnx:{exc}"
            self.get_logger().warn(f"[leader_detector] ONNX inference failed: {exc}")
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
        if not result.detections:
            return None
        best = max(result.detections, key=self._score_candidate)
        best.source = "detector"
        return best

    def _pick_detection(self, img_bgr: np.ndarray, timing: FrameTiming) -> Optional[Detection2D]:
        if not self.yolo_ready:
            return None
        if self.backend == "ultralytics":
            return self._pick_detection_ultralytics(img_bgr, timing)
        return self._pick_detection_onnx(img_bgr, timing)

    def _rate_gate_allows_inference(self) -> bool:
        if self.last_predict_perf_ns <= 0:
            return True
        dt_s = (perf_counter_ns() - self.last_predict_perf_ns) * 1e-9
        return dt_s >= (1.0 / self.predict_hz)

    def on_image(self, msg: Image) -> None:
        recv_ros_ns = int(self.get_clock().now().nanoseconds)
        pending = PendingImage(
            msg=msg,
            recv_ros_ns=recv_ros_ns,
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
        }

    def _log_timing(self, timing: FrameTiming) -> None:
        if self.benchmark_logger is not None:
            self.benchmark_logger.log(timing)

    def _publish_result(
        self,
        pending: PendingImage,
        det: Optional[Detection2D],
        timing: FrameTiming,
        *,
        state: str,
        reason: str,
    ) -> None:
        publish_ros_ns = int(self.get_clock().now().nanoseconds)
        timing.publish_ros_ns = publish_ros_ns
        timing.publish_perf_ns = perf_counter_ns()
        timing.provider = self.current_provider or ""
        self.publish_rate_tracker.record(publish_ros_ns)
        timing.detector_hz = self.publish_rate_tracker.hz(publish_ros_ns)
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

        metadata = timing.metadata()
        self._publish_detection(pending.msg, det, metadata=metadata)
        self._publish_status(state, reason, det, extras=self._status_extras(timing))
        self._log_timing(timing)

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
            det = self._pick_detection(img_bgr, timing)
            if timing.infer_end_ros_ns <= 0:
                timing.infer_end_ros_ns = int(self.get_clock().now().nanoseconds)
            if timing.infer_end_perf_ns <= 0:
                timing.infer_end_perf_ns = perf_counter_ns()
            if timing.postprocess_end_ros_ns <= 0:
                timing.postprocess_end_ros_ns = int(self.get_clock().now().nanoseconds)
            if timing.postprocess_end_perf_ns <= 0:
                timing.postprocess_end_perf_ns = perf_counter_ns()

            if det is not None:
                self.last_det_center = (det.u, det.v)
                self.last_det_cls_id = det.cls_id
                self.yolo_error = None
                self._publish_result(pending, det, timing, state="OK", reason="none")
                return

            self._publish_result(
                pending,
                None,
                timing,
                state="NO_DET",
                reason=self.yolo_error or "no_detection",
            )
        except Exception as exc:
            self.get_logger().warn(f"[leader_detector] Processing failed: {exc}")


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
        try:
            node.close()
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
