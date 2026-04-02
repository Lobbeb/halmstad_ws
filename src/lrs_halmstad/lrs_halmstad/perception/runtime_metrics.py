from __future__ import annotations

import csv
import math
import os
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Optional


@dataclass
class FrameTiming:
    source_stamp_ns: int
    frame_recv_ros_ns: int
    frame_recv_perf_ns: int
    backend: str
    provider: str = ""
    frame_index: int = 0
    infer_start_ros_ns: int = 0
    infer_end_ros_ns: int = 0
    infer_start_perf_ns: int = 0
    infer_end_perf_ns: int = 0
    postprocess_end_ros_ns: int = 0
    postprocess_end_perf_ns: int = 0
    publish_ros_ns: int = 0
    publish_perf_ns: int = 0
    detector_hz: float = 0.0
    dropped_frames_total: int = 0
    stale_detections_total: int = 0
    stale_detection: bool = False
    valid_detection: bool = False
    reason: str = "none"

    @property
    def inference_ms(self) -> float:
        if self.infer_start_perf_ns <= 0 or self.infer_end_perf_ns <= 0:
            return math.nan
        return max(0.0, (self.infer_end_perf_ns - self.infer_start_perf_ns) * 1e-6)

    @property
    def postprocess_ms(self) -> float:
        if self.infer_end_perf_ns <= 0 or self.postprocess_end_perf_ns <= 0:
            return math.nan
        return max(0.0, (self.postprocess_end_perf_ns - self.infer_end_perf_ns) * 1e-6)

    @property
    def receive_to_publish_ms(self) -> float:
        if self.frame_recv_perf_ns <= 0 or self.publish_perf_ns <= 0:
            return math.nan
        return max(0.0, (self.publish_perf_ns - self.frame_recv_perf_ns) * 1e-6)

    @property
    def camera_to_publish_latency_ms(self) -> float:
        if self.source_stamp_ns <= 0 or self.publish_ros_ns <= 0:
            return math.nan
        return max(0.0, (self.publish_ros_ns - self.source_stamp_ns) * 1e-6)

    def metadata(self) -> dict[str, object]:
        return {
            "frame_index": int(self.frame_index),
            "backend": str(self.backend),
            "provider": str(self.provider),
            "frame_recv_ros_ns": int(self.frame_recv_ros_ns),
            "infer_start_ros_ns": int(self.infer_start_ros_ns),
            "infer_end_ros_ns": int(self.infer_end_ros_ns),
            "postprocess_end_ros_ns": int(self.postprocess_end_ros_ns),
            "publish_ros_ns": int(self.publish_ros_ns),
            "inference_ms": float(self.inference_ms) if math.isfinite(self.inference_ms) else None,
            "postprocess_ms": float(self.postprocess_ms) if math.isfinite(self.postprocess_ms) else None,
            "camera_to_publish_latency_ms": (
                float(self.camera_to_publish_latency_ms)
                if math.isfinite(self.camera_to_publish_latency_ms)
                else None
            ),
            "receive_to_publish_ms": (
                float(self.receive_to_publish_ms)
                if math.isfinite(self.receive_to_publish_ms)
                else None
            ),
            "detector_hz": float(self.detector_hz),
            "dropped_frames_total": int(self.dropped_frames_total),
            "stale_detections_total": int(self.stale_detections_total),
            "stale_detection": bool(self.stale_detection),
            "valid_detection": bool(self.valid_detection),
            "reason": str(self.reason),
        }

    def csv_row(self) -> dict[str, object]:
        row = self.metadata().copy()
        row["source_stamp_ns"] = int(self.source_stamp_ns)
        return row


class RollingRateTracker:
    def __init__(self, window_s: float = 5.0):
        self.window_ns = max(1, int(max(0.1, float(window_s)) * 1e9))
        self.samples_ns: deque[int] = deque()

    def record(self, stamp_ns: int) -> None:
        stamp_ns = int(stamp_ns)
        if stamp_ns <= 0:
            return
        self.samples_ns.append(stamp_ns)
        self._trim(stamp_ns)

    def hz(self, now_ns: Optional[int] = None) -> float:
        if not self.samples_ns:
            return 0.0
        ref_ns = int(now_ns) if now_ns is not None else int(self.samples_ns[-1])
        self._trim(ref_ns)
        if len(self.samples_ns) <= 1:
            return 0.0
        span_ns = max(1, self.samples_ns[-1] - self.samples_ns[0])
        return max(0.0, (len(self.samples_ns) - 1) / (span_ns * 1e-9))

    def _trim(self, ref_ns: int) -> None:
        cutoff = int(ref_ns) - self.window_ns
        while self.samples_ns and self.samples_ns[0] < cutoff:
            self.samples_ns.popleft()


class CsvBenchmarkLogger:
    FIELDNAMES = [
        "source_stamp_ns",
        "frame_index",
        "backend",
        "provider",
        "frame_recv_ros_ns",
        "infer_start_ros_ns",
        "infer_end_ros_ns",
        "postprocess_end_ros_ns",
        "publish_ros_ns",
        "inference_ms",
        "postprocess_ms",
        "camera_to_publish_latency_ms",
        "receive_to_publish_ms",
        "detector_hz",
        "dropped_frames_total",
        "stale_detections_total",
        "stale_detection",
        "valid_detection",
        "reason",
    ]

    def __init__(self, csv_path: str):
        self.csv_path = os.path.expanduser(str(csv_path).strip())
        self._lock = threading.Lock()
        self._header_written = os.path.isfile(self.csv_path) and os.path.getsize(self.csv_path) > 0
        os.makedirs(os.path.dirname(self.csv_path) or ".", exist_ok=True)

    def log(self, timing: FrameTiming) -> None:
        row = timing.csv_row()
        with self._lock:
            with open(self.csv_path, "a", encoding="utf-8", newline="") as fh:
                writer = csv.DictWriter(fh, fieldnames=self.FIELDNAMES)
                if not self._header_written:
                    writer.writeheader()
                    self._header_written = True
                writer.writerow(row)


class LatestItemWorker:
    """Single-worker latest-item processor.

    Only one pending item is retained. When a new item arrives while another is already
    pending, the older pending item is replaced.
    """

    def __init__(self, process_fn, *, thread_name: str):
        self._process_fn = process_fn
        self._cond = threading.Condition()
        self._pending = None
        self._stop = False
        self._thread = threading.Thread(target=self._run, name=thread_name, daemon=True)
        self._thread.start()

    def submit(self, item) -> bool:
        with self._cond:
            replaced = self._pending is not None
            self._pending = item
            self._cond.notify()
            return replaced

    def close(self, timeout_s: float = 2.0) -> None:
        with self._cond:
            self._stop = True
            self._pending = None
            self._cond.notify_all()
        self._thread.join(timeout=max(0.0, float(timeout_s)))

    def _run(self) -> None:
        while True:
            with self._cond:
                while self._pending is None and not self._stop:
                    self._cond.wait(timeout=0.25)
                if self._stop:
                    return
                item = self._pending
                self._pending = None
            try:
                self._process_fn(item)
            except Exception:
                # Keep the worker alive so a single bad frame does not permanently
                # kill perception.
                continue


def percentile(values: list[float], q: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(float(v) for v in values)
    if len(ordered) == 1:
        return ordered[0]
    rank = max(0.0, min(1.0, float(q) / 100.0)) * (len(ordered) - 1)
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    frac = rank - lower
    return ordered[lower] * (1.0 - frac) + ordered[upper] * frac


def summarize_csv_rows(rows: list[dict[str, str]]) -> dict[str, float]:
    inference_ms = [float(r["inference_ms"]) for r in rows if str(r.get("inference_ms", "")).strip()]
    latency_ms = [
        float(r["camera_to_publish_latency_ms"])
        for r in rows
        if str(r.get("camera_to_publish_latency_ms", "")).strip()
    ]
    publish_ros_ns = [int(r["publish_ros_ns"]) for r in rows if str(r.get("publish_ros_ns", "")).strip()]

    stats = {
        "samples": float(len(rows)),
        "inference_mean_ms": statistics.fmean(inference_ms) if inference_ms else math.nan,
        "inference_median_ms": statistics.median(inference_ms) if inference_ms else math.nan,
        "inference_p95_ms": percentile(inference_ms, 95.0),
        "latency_mean_ms": statistics.fmean(latency_ms) if latency_ms else math.nan,
        "latency_median_ms": statistics.median(latency_ms) if latency_ms else math.nan,
        "latency_p95_ms": percentile(latency_ms, 95.0),
        "detector_hz": 0.0,
        "dropped_frames_total": 0.0,
        "stale_detections_total": 0.0,
    }
    if len(publish_ros_ns) > 1:
        span_ns = max(1, max(publish_ros_ns) - min(publish_ros_ns))
        stats["detector_hz"] = max(0.0, (len(publish_ros_ns) - 1) / (span_ns * 1e-9))
    if rows:
        stats["dropped_frames_total"] = float(max(int(r.get("dropped_frames_total", "0") or 0) for r in rows))
        stats["stale_detections_total"] = float(max(int(r.get("stale_detections_total", "0") or 0) for r in rows))
    return stats


def perf_counter_ns() -> int:
    return time.perf_counter_ns()
