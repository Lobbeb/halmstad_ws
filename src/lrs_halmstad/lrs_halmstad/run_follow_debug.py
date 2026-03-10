#!/usr/bin/env python3
import argparse
import math
from datetime import datetime
from pathlib import Path
from collections import deque
from dataclasses import dataclass
from typing import Deque, Optional, TextIO

import rclpy
from geometry_msgs.msg import PoseStamped
from rcl_interfaces.msg import ParameterType, ParameterValue
from rclpy.node import Node
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Float32, Float64, String
from lrs_halmstad.follow_math import wrap_pi, yaw_from_quat


def parse_status_line(line: str) -> dict[str, str]:
    fields: dict[str, str] = {}
    for token in line.split():
        if "=" not in token:
            continue
        key, value = token.split("=", 1)
        fields[key] = value
    return fields


def param_value_to_python(value: ParameterValue):
    kind = value.type
    if kind == ParameterType.PARAMETER_BOOL:
        return value.bool_value
    if kind == ParameterType.PARAMETER_INTEGER:
        return value.integer_value
    if kind == ParameterType.PARAMETER_DOUBLE:
        return value.double_value
    if kind == ParameterType.PARAMETER_STRING:
        return value.string_value
    if kind == ParameterType.PARAMETER_BYTE_ARRAY:
        return list(value.byte_array_value)
    if kind == ParameterType.PARAMETER_BOOL_ARRAY:
        return list(value.bool_array_value)
    if kind == ParameterType.PARAMETER_INTEGER_ARRAY:
        return list(value.integer_array_value)
    if kind == ParameterType.PARAMETER_DOUBLE_ARRAY:
        return list(value.double_array_value)
    if kind == ParameterType.PARAMETER_STRING_ARRAY:
        return list(value.string_array_value)
    return None


@dataclass
class Sample:
    t: float
    value: float


class FollowDebugNode(Node):
    def __init__(self, uav_name: str, window_s: float, report_hz: float, log_file: str):
        super().__init__("follow_debug")
        self.uav_name = uav_name
        self.window_s = max(0.2, float(window_s))
        self.report_period_s = 1.0 / max(0.2, float(report_hz))
        self.history_limit = max(20, int(math.ceil(self.window_s * 40.0)))
        self.log_file_handle: Optional[TextIO] = self._open_log_file(log_file)

        self.estimator_status: dict[str, str] = {}
        self.estimator_fault: dict[str, str] = {}
        self.estimator_status_rx_s: Optional[float] = None
        self.estimator_fault_rx_s: Optional[float] = None

        self.follow_params: dict[str, object] = {}
        self.camera_params: dict[str, object] = {}
        self.follow_param_client = AsyncParameterClient(self, "/follow_uav")
        self.camera_param_client = AsyncParameterClient(self, "/camera_tracker")
        self.follow_param_names = [
            "follow_yaw",
            "manual_override_enable",
            "leader_input_type",
            "startup_reposition_enable",
        ]
        self.camera_param_names = [
            "pan_enable",
            "tilt_enable",
            "uav_camera_mode",
        ]
        self._follow_params_pending = False
        self._camera_params_pending = False

        self.pose_cmd_yaw_hist: Deque[Sample] = deque(maxlen=self.history_limit)
        self.pose_actual_yaw_hist: Deque[Sample] = deque(maxlen=self.history_limit)
        self.pan_hist: Deque[Sample] = deque(maxlen=self.history_limit)
        self.tilt_hist: Deque[Sample] = deque(maxlen=self.history_limit)
        self.follow_target_yaw_hist: Deque[Sample] = deque(maxlen=self.history_limit)
        self.follow_actual_yaw_hist: Deque[Sample] = deque(maxlen=self.history_limit)
        self.follow_error_yaw_hist: Deque[Sample] = deque(maxlen=self.history_limit)
        self.pose_cmd_xy: Optional[tuple[float, float]] = None
        self.pose_actual_xy: Optional[tuple[float, float]] = None
        self.anchor_target_xy: Optional[tuple[float, float]] = None
        self.follow_target_d_target_m: Optional[float] = None
        self.follow_actual_xy_distance_m: Optional[float] = None
        self.follow_actual_distance_3d_m: Optional[float] = None
        self.follow_error_xy_distance_m: Optional[float] = None
        self.follow_error_anchor_distance_m: Optional[float] = None
        self.follow_error_anchor_along_m: Optional[float] = None
        self.follow_error_anchor_cross_m: Optional[float] = None

        self.last_yaw_mode: Optional[str] = None
        self.last_yaw_mode_rx_s: Optional[float] = None

        self.create_subscription(String, "/coord/leader_estimate_status", self.on_estimator_status, 10)
        self.create_subscription(String, "/coord/leader_estimate_fault", self.on_estimator_fault, 10)
        self.create_subscription(PoseStamped, f"/{self.uav_name}/pose_cmd", self.on_pose_cmd, 10)
        self.create_subscription(PoseStamped, f"/{self.uav_name}/pose", self.on_pose_actual, 10)
        self.create_subscription(
            PoseStamped,
            f"/{self.uav_name}/follow/target/anchor_pose",
            self.on_follow_target_anchor,
            10,
        )
        self.create_subscription(Float64, f"/{self.uav_name}/update_pan", self.on_pan, 10)
        self.create_subscription(Float64, f"/{self.uav_name}/update_tilt", self.on_tilt, 10)
        self.create_subscription(
            Float32,
            f"/{self.uav_name}/follow/target/d_target_m",
            self.on_follow_target_d_target,
            10,
        )
        self.create_subscription(
            Float32,
            f"/{self.uav_name}/follow/actual/xy_distance_m",
            self.on_follow_actual_xy_distance,
            10,
        )
        self.create_subscription(
            Float32,
            f"/{self.uav_name}/follow/actual/distance_3d_m",
            self.on_follow_actual_distance_3d,
            10,
        )
        self.create_subscription(
            Float32,
            f"/{self.uav_name}/follow/error/xy_distance_m",
            self.on_follow_error_xy_distance,
            10,
        )
        self.create_subscription(
            Float32,
            f"/{self.uav_name}/follow/error/anchor_distance_m",
            self.on_follow_error_anchor_distance,
            10,
        )
        self.create_subscription(
            Float32,
            f"/{self.uav_name}/follow/error/anchor_along_m",
            self.on_follow_error_anchor_along,
            10,
        )
        self.create_subscription(
            Float32,
            f"/{self.uav_name}/follow/error/anchor_cross_m",
            self.on_follow_error_anchor_cross,
            10,
        )
        self.create_subscription(Float32, f"/{self.uav_name}/follow/target/yaw_rad", self.on_follow_target_yaw, 10)
        self.create_subscription(Float32, f"/{self.uav_name}/follow/actual/yaw_rad", self.on_follow_actual_yaw, 10)
        self.create_subscription(Float32, f"/{self.uav_name}/follow/error/yaw_rad", self.on_follow_error_yaw, 10)
        self.create_subscription(String, f"/{self.uav_name}/follow/debug/yaw_mode", self.on_yaw_mode, 10)

        self.param_timer = self.create_timer(2.0, self.refresh_params)
        self.report_timer = self.create_timer(self.report_period_s, self.report)
        self.get_logger().info(
            f"[follow_debug] Watching uav={self.uav_name}, window_s={self.window_s:.1f}, "
            f"topics=estimator+follow+camera+xy"
        )
        if self.log_file_handle is not None:
            self._emit_text(
                f"[follow_debug] Logging runtime analysis to {self.log_file_handle.name}"
            )

    def _default_log_file(self) -> str:
        log_dir = Path(__file__).resolve().parents[3] / "debug_logs" / "follow_debug"
        log_dir.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        return str(log_dir / f"follow_debug_{stamp}.log")

    def _open_log_file(self, log_file: str) -> Optional[TextIO]:
        path = str(log_file).strip() if log_file else self._default_log_file()
        try:
            out_path = Path(path)
            out_path.parent.mkdir(parents=True, exist_ok=True)
            return out_path.open("a", encoding="utf-8", buffering=1)
        except Exception as exc:
            self.get_logger().warn(f"[follow_debug] Could not open log file '{path}': {exc}")
            return None

    def _emit_text(self, text: str) -> None:
        print(text, flush=True)
        if self.log_file_handle is not None:
            try:
                self.log_file_handle.write(text + "\n")
            except Exception as exc:
                self.get_logger().warn(f"[follow_debug] Failed writing log file: {exc}")
                try:
                    self.log_file_handle.close()
                except Exception:
                    pass
                self.log_file_handle = None

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def append_sample(self, history: Deque[Sample], value: float) -> None:
        now_s = self.now_s()
        history.append(Sample(now_s, float(value)))
        cutoff = now_s - (self.window_s * 2.0)
        while history and history[0].t < cutoff:
            history.popleft()

    def on_estimator_status(self, msg: String) -> None:
        self.estimator_status = parse_status_line(msg.data)
        self.estimator_status_rx_s = self.now_s()

    def on_estimator_fault(self, msg: String) -> None:
        self.estimator_fault = parse_status_line(msg.data)
        self.estimator_fault_rx_s = self.now_s()

    def on_pose_cmd(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        self.pose_cmd_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.append_sample(self.pose_cmd_yaw_hist, yaw)

    def on_pose_actual(self, msg: PoseStamped) -> None:
        q = msg.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        self.pose_actual_xy = (float(msg.pose.position.x), float(msg.pose.position.y))
        self.append_sample(self.pose_actual_yaw_hist, yaw)

    def on_follow_target_anchor(self, msg: PoseStamped) -> None:
        self.anchor_target_xy = (float(msg.pose.position.x), float(msg.pose.position.y))

    def on_pan(self, msg: Float64) -> None:
        self.append_sample(self.pan_hist, float(msg.data))

    def on_tilt(self, msg: Float64) -> None:
        self.append_sample(self.tilt_hist, float(msg.data))

    def on_follow_target_d_target(self, msg: Float32) -> None:
        self.follow_target_d_target_m = float(msg.data)

    def on_follow_actual_xy_distance(self, msg: Float32) -> None:
        self.follow_actual_xy_distance_m = float(msg.data)

    def on_follow_actual_distance_3d(self, msg: Float32) -> None:
        self.follow_actual_distance_3d_m = float(msg.data)

    def on_follow_error_xy_distance(self, msg: Float32) -> None:
        self.follow_error_xy_distance_m = float(msg.data)

    def on_follow_error_anchor_distance(self, msg: Float32) -> None:
        self.follow_error_anchor_distance_m = float(msg.data)

    def on_follow_error_anchor_along(self, msg: Float32) -> None:
        self.follow_error_anchor_along_m = float(msg.data)

    def on_follow_error_anchor_cross(self, msg: Float32) -> None:
        self.follow_error_anchor_cross_m = float(msg.data)

    def on_follow_target_yaw(self, msg: Float32) -> None:
        self.append_sample(self.follow_target_yaw_hist, float(msg.data))

    def on_follow_actual_yaw(self, msg: Float32) -> None:
        self.append_sample(self.follow_actual_yaw_hist, float(msg.data))

    def on_follow_error_yaw(self, msg: Float32) -> None:
        self.append_sample(self.follow_error_yaw_hist, float(msg.data))

    def on_yaw_mode(self, msg: String) -> None:
        self.last_yaw_mode = msg.data.strip()
        self.last_yaw_mode_rx_s = self.now_s()

    def refresh_params(self) -> None:
        if not self._follow_params_pending and self.follow_param_client.services_are_ready():
            future = self.follow_param_client.get_parameters(self.follow_param_names)
            self._follow_params_pending = True
            future.add_done_callback(self.on_follow_params)
        if not self._camera_params_pending and self.camera_param_client.services_are_ready():
            future = self.camera_param_client.get_parameters(self.camera_param_names)
            self._camera_params_pending = True
            future.add_done_callback(self.on_camera_params)

    def on_follow_params(self, future) -> None:
        self._follow_params_pending = False
        try:
            result = future.result()
        except Exception:
            return
        if result is None:
            return
        self.follow_params = {
            name: param_value_to_python(value)
            for name, value in zip(self.follow_param_names, result.values)
        }

    def on_camera_params(self, future) -> None:
        self._camera_params_pending = False
        try:
            result = future.result()
        except Exception:
            return
        if result is None:
            return
        self.camera_params = {
            name: param_value_to_python(value)
            for name, value in zip(self.camera_param_names, result.values)
        }

    def recent_delta(self, history: Deque[Sample], wrap: bool = False) -> Optional[float]:
        if len(history) < 2:
            return None
        now_s = self.now_s()
        start = None
        end = history[-1]
        cutoff = now_s - self.window_s
        for sample in history:
            if sample.t >= cutoff:
                start = sample
                break
        if start is None:
            start = history[0]
        delta = end.value - start.value
        return wrap_pi(delta) if wrap else delta

    def latest(self, history: Deque[Sample]) -> Optional[float]:
        if not history:
            return None
        return history[-1].value

    def fmt_deg(self, value: Optional[float], radians: bool = False) -> str:
        if value is None:
            return "na"
        if radians:
            value = math.degrees(value)
        return f"{value:.1f}deg"

    def fmt_m(self, value: Optional[float]) -> str:
        if value is None:
            return "na"
        return f"{value:.2f}m"

    def fmt_xy(self, value: Optional[tuple[float, float]]) -> str:
        if value is None:
            return "(na, na)"
        return f"({value[0]:.2f}, {value[1]:.2f})"

    def estimator_state(self) -> str:
        return self.estimator_status.get("state", "none")

    def estimator_reason(self) -> str:
        reason = self.estimator_status.get("state_reason", "")
        if reason and reason != "none":
            return reason
        reason = self.estimator_fault.get("reason", "")
        return reason or "none"

    def classify(self) -> tuple[str, list[str]]:
        warnings: list[str] = []
        body_cmd_delta = self.recent_delta(self.pose_cmd_yaw_hist, wrap=True)
        body_actual_delta = self.recent_delta(self.pose_actual_yaw_hist, wrap=True)
        pan_delta = self.recent_delta(self.pan_hist, wrap=False)
        tilt_delta = self.recent_delta(self.tilt_hist, wrap=False)

        body_active = body_cmd_delta is not None and abs(math.degrees(body_cmd_delta)) > 3.0
        body_actual_active = body_actual_delta is not None and abs(math.degrees(body_actual_delta)) > 3.0
        pan_active = pan_delta is not None and abs(pan_delta) > 2.0
        tilt_active = tilt_delta is not None and abs(tilt_delta) > 2.0

        estimator_state = self.estimator_state()
        bad_estimator_state = estimator_state in {
            "NO_DET",
            "STALE",
            "DECODE_FAIL",
            "YOLO_DISABLED",
            "REJECT",
            "REJECT_HOLD",
            "REJECT_DEBOUNCE_HOLD",
            "HOLD",
            "DEBOUNCE_HOLD",
        }

        if body_active and pan_active:
            cause = "camera pan and UAV body yaw are both moving"
        elif body_active:
            cause = "UAV body yaw command is moving"
        elif pan_active or tilt_active:
            cause = "camera-only pan/tilt is moving"
        elif body_actual_active:
            cause = "UAV body is drifting/rotating without a large new yaw command"
        else:
            cause = "no significant turn command detected"

        if bad_estimator_state and (pan_active or tilt_active):
            warnings.append(f"camera still moving while estimator state={estimator_state}")
        if bad_estimator_state and body_active:
            warnings.append(f"body yaw command still moving while estimator state={estimator_state}")
        if self.follow_params.get("follow_yaw") is False and body_active:
            warnings.append("follow_yaw=false but body yaw command is still changing")
        if self.follow_params.get("startup_reposition_enable") and not self.follow_params.get("manual_override_enable", False):
            warnings.append("startup_reposition_enable=true on /follow_uav")
        if (
            estimator_state == "OK"
            and self.follow_error_anchor_cross_m is not None
            and self.follow_error_anchor_along_m is not None
            and abs(self.follow_error_anchor_cross_m) > 2.0
            and abs(self.follow_error_anchor_cross_m) > max(1.5 * abs(self.follow_error_anchor_along_m), 1.0)
        ):
            warnings.append("anchor cross-track error dominates; possible mirrored heading/anchor frame")
        return cause, warnings

    def report(self) -> None:
        cause, warnings = self.classify()
        state = self.estimator_state()
        reason = self.estimator_reason()
        conf = self.estimator_status.get("conf", "na")
        reject = self.estimator_status.get("reject_reason", "na")

        body_cmd_delta = self.recent_delta(self.pose_cmd_yaw_hist, wrap=True)
        body_actual_delta = self.recent_delta(self.pose_actual_yaw_hist, wrap=True)
        pan_delta = self.recent_delta(self.pan_hist, wrap=False)
        tilt_delta = self.recent_delta(self.tilt_hist, wrap=False)
        pan_now = self.latest(self.pan_hist)
        tilt_now = self.latest(self.tilt_hist)

        follow_yaw = self.follow_params.get("follow_yaw", "na")
        manual_override = self.follow_params.get("manual_override_enable", "na")
        startup_reposition = self.follow_params.get("startup_reposition_enable", "na")
        pan_enable = self.camera_params.get("pan_enable", "na")
        tilt_enable = self.camera_params.get("tilt_enable", "na")
        yaw_mode = self.last_yaw_mode or "na"
        anchor_xy = self.anchor_target_xy
        actual_xy = self.pose_actual_xy
        cmd_xy = self.pose_cmd_xy
        anchor_dx = None
        anchor_dy = None
        if anchor_xy is not None and actual_xy is not None:
            anchor_dx = anchor_xy[0] - actual_xy[0]
            anchor_dy = anchor_xy[1] - actual_xy[1]
        timestamp = datetime.now().strftime("%H:%M:%S")

        lines = [
            f"[{timestamp}] Follow Debug",
            f"  Estimator: state={state} reason={reason} conf={conf} reject={reject}",
            f"  Motion: {cause}",
            (
                "  Body yaw: "
                f"cmd_delta={self.fmt_deg(body_cmd_delta, radians=True)} "
                f" actual_delta={self.fmt_deg(body_actual_delta, radians=True)} "
                f" follow_yaw={follow_yaw} yaw_mode={yaw_mode}"
            ),
            (
                "  Camera: "
                f"pan={self.fmt_deg(pan_now)} delta={self.fmt_deg(pan_delta)} "
                f" tilt={self.fmt_deg(tilt_now)} delta={self.fmt_deg(tilt_delta)} "
                f" pan_enable={pan_enable} tilt_enable={tilt_enable}"
            ),
            (
                "  XY: "
                f"anchor={self.fmt_xy(anchor_xy)} actual={self.fmt_xy(actual_xy)} "
                f"cmd={self.fmt_xy(cmd_xy)} delta_to_anchor={self.fmt_xy(None if anchor_dx is None or anchor_dy is None else (anchor_dx, anchor_dy))}"
            ),
            (
                "  Anchor: "
                f"d_target={self.fmt_m(self.follow_target_d_target_m)} "
                f"actual_xy={self.fmt_m(self.follow_actual_xy_distance_m)} "
                f"actual_3d={self.fmt_m(self.follow_actual_distance_3d_m)} "
                f"xy_err={self.fmt_m(self.follow_error_xy_distance_m)} "
                f"anchor_dist={self.fmt_m(self.follow_error_anchor_distance_m)} "
                f"along={self.fmt_m(self.follow_error_anchor_along_m)} "
                f"cross={self.fmt_m(self.follow_error_anchor_cross_m)}"
            ),
            (
                "  Control: "
                f"manual_override={manual_override} "
                f"startup_reposition={startup_reposition}"
            ),
        ]
        for warning in warnings:
            lines.append(f"  Warning: {warning}")
        self._emit_text("\n".join(lines))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Analyze follow/camera turning causes during runtime.")
    parser.add_argument("--uav-name", default="dji0", help="UAV namespace/name. Default: dji0")
    parser.add_argument("--window-s", type=float, default=1.5, help="Analysis window in seconds. Default: 1.5")
    parser.add_argument("--report-hz", type=float, default=1.0, help="How often to print diagnoses. Default: 1.0")
    parser.add_argument(
        "--log-file",
        default="",
        help="Optional path to save the runtime analysis log. Default: <workspace>/debug_logs/follow_debug/<timestamp>.log",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    rclpy.init()
    node = FollowDebugNode(
        uav_name=str(args.uav_name).strip() or "dji0",
        window_s=float(args.window_s),
        report_hz=float(args.report_hz),
        log_file=str(args.log_file),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node.log_file_handle is not None:
                node.log_file_handle.close()
        except Exception:
            pass
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
