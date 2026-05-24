#!/usr/bin/env python3
import argparse
import math
import random
import select
import sys
import termios
import tty
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterType, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient
from std_msgs.msg import Float64

@dataclass
class FollowControlConfig:
    mode: str
    node_name: str
    timeout_s: float
    step_z_min: float
    step_d_target: float
    step_heading_deg: float
    step_pan_deg: float
    step_tilt_deg: float
    interval_s: float
    start_delay_s: float
    count: int
    min_z_min: float
    max_z_min: float
    min_d_target: float
    max_d_target: float
    focus_min: float
    focus_max: float
    focus_weight: float
    decimals: int
    seed: Optional[int]
    # Gimbal sweep fields (random mode)
    uav_name: str = "dji0"
    gimbal_enable: bool = False
    gimbal_only: bool = False          # skip d_target/z_min, only sweep gimbal
    gimbal_interval_s: float = 0.0    # 0 = use interval_s
    tilt_center_deg: float = -45.0
    tilt_amplitude_deg: float = 15.0
    tilt_min_deg: float = -85.0
    tilt_max_deg: float = 0.0
    pan_center_deg: float = 0.0
    pan_amplitude_deg: float = 20.0
    pan_min_deg: float = -45.0
    pan_max_deg: float = 45.0
    gimbal_publish_hz: float = 5.0
    opposite_settle_s: float = 8.0


def sample_biased_range(
    rng: random.Random,
    value_min: float,
    value_max: float,
    focus_min: float,
    focus_max: float,
    focus_weight: float,
) -> float:
    if value_max <= value_min:
        return float(value_min)

    low_end = min(value_max, focus_min)
    mid_start = max(value_min, focus_min)
    mid_end = min(value_max, focus_max)
    high_start = max(value_min, focus_max)

    side_weight = max(0.0, 1.0 - focus_weight) / 2.0
    buckets = []

    if low_end > value_min:
        buckets.append((side_weight, value_min, low_end))
    if mid_end > mid_start:
        buckets.append((focus_weight, mid_start, mid_end))
    if value_max > high_start:
        buckets.append((side_weight, high_start, value_max))

    if not buckets:
        return rng.uniform(value_min, value_max)

    total_weight = sum(weight for weight, _, _ in buckets)
    pick = rng.uniform(0.0, total_weight)
    running = 0.0
    for weight, bucket_min, bucket_max in buckets:
        running += weight
        if pick <= running:
            return rng.uniform(bucket_min, bucket_max)
    return rng.uniform(buckets[-1][1], buckets[-1][2])


def extract_set_result(result):
    if isinstance(result, SetParametersResult):
        return result
    if isinstance(result, SetParametersAtomically.Response):
        return result.result
    if hasattr(result, "result") and isinstance(result.result, SetParametersResult):
        return result.result
    return None


def clamp(value: float, value_min: float, value_max: float) -> float:
    return min(max(float(value), float(value_min)), float(value_max))


def normalize_angle_deg(angle: float) -> float:
    angle = float(angle)
    while angle > 180.0:
        angle -= 360.0
    while angle <= -180.0:
        angle += 360.0
    return angle


def shortest_angle_delta_deg(target: float, current: float) -> float:
    return normalize_angle_deg(float(target) - float(current))


def move_towards(value: float, target: float, max_step: float) -> float:
    delta = float(target) - float(value)
    if abs(delta) <= max_step:
        return float(target)
    return float(value) + math.copysign(float(max_step), delta)


def move_angle_towards(value: float, target: float, max_step: float) -> float:
    delta = shortest_angle_delta_deg(target, value)
    if abs(delta) <= max_step:
        return normalize_angle_deg(target)
    return normalize_angle_deg(float(value) + math.copysign(float(max_step), delta))


def parameter_value_to_float(value) -> Optional[float]:
    if value.type == ParameterType.PARAMETER_DOUBLE:
        return float(value.double_value)
    if value.type == ParameterType.PARAMETER_INTEGER:
        return float(value.integer_value)
    return None


class RandomFollowParams(Node):
    def __init__(self, config: FollowControlConfig) -> None:
        super().__init__("random_follow_params")
        self.config = config
        self.rng = random.Random(config.seed)
        self.param_client = AsyncParameterClient(self, config.node_name) if not config.gimbal_only else None
        self.pending_future = None
        self.applied_count = 0
        t0 = time.monotonic() + max(config.start_delay_s, 0.0)
        self.next_update_wall = t0
        self.next_gimbal_wall = t0
        self.timer = self.create_timer(0.2, self._tick)

        # Gimbal publishers
        self._tilt_pub = None
        self._pan_pub = None
        if config.gimbal_enable or config.gimbal_only:
            self._tilt_pub = self.create_publisher(Float64, f"/{config.uav_name}/update_tilt", 10)
            self._pan_pub = self.create_publisher(Float64, f"/{config.uav_name}/update_pan", 10)

        self._validate_config()
        if not config.gimbal_only:
            self._wait_for_follow_node()

        seed_text = "time-based" if config.seed is None else str(config.seed)
        gimbal_interval = config.gimbal_interval_s if config.gimbal_interval_s > 0.0 else config.interval_s
        if not config.gimbal_only:
            self.get_logger().info(
                "Random follow param sweep ready: "
                f"target={config.node_name} "
                f"interval={config.interval_s:.1f}s "
                f"d_target=[{config.min_d_target}, {config.max_d_target}] "
                f"focus=[{config.focus_min}, {config.focus_max}] "
                f"focus_weight={config.focus_weight:.2f} "
                f"count={'infinite' if config.count <= 0 else config.count} "
                f"seed={seed_text}"
            )
        if config.gimbal_enable or config.gimbal_only:
            self.get_logger().info(
                f"Gimbal sweep: uav={config.uav_name} interval={gimbal_interval:.1f}s "
                f"tilt={config.tilt_center_deg:.1f}±{config.tilt_amplitude_deg:.1f}° "
                f"[{config.tilt_min_deg}, {config.tilt_max_deg}] "
                f"pan={config.pan_center_deg:.1f}±{config.pan_amplitude_deg:.1f}° "
                f"[{config.pan_min_deg}, {config.pan_max_deg}]"
            )
        if config.start_delay_s > 0.0:
            self.get_logger().info(f"Initial delay {config.start_delay_s:.1f}s before first random update")

    def _validate_config(self) -> None:
        cfg = self.config
        if cfg.interval_s <= 0.0:
            raise ValueError("interval must be > 0")
        if cfg.timeout_s <= 0.0:
            raise ValueError("timeout must be > 0")
        if not cfg.gimbal_only:
            if cfg.min_d_target <= 0.0 or cfg.max_d_target <= 0.0 or cfg.max_d_target < cfg.min_d_target:
                raise ValueError("d_target range must satisfy 0 < min <= max")
            if cfg.focus_max < cfg.focus_min:
                raise ValueError("focus range must satisfy focus_min <= focus_max")
            if not 0.0 <= cfg.focus_weight <= 1.0:
                raise ValueError("focus_weight must be within [0, 1]")
        if cfg.decimals < 0:
            raise ValueError("decimals must be >= 0")

    def _wait_for_follow_node(self) -> None:
        while rclpy.ok() and not self.param_client.wait_for_services(timeout_sec=self.config.timeout_s):
            self.get_logger().info(f"Waiting for parameter services on {self.config.node_name} ...")

    def _sample_angle(self, center: float, amplitude: float, lo: float, hi: float) -> float:
        raw = center + self.rng.uniform(-amplitude, amplitude)
        return max(lo, min(hi, round(raw, self.config.decimals)))

    def _tick(self) -> None:
        now = time.monotonic()

        # Gimbal sweep tick (independent interval)
        gimbal_interval = (
            self.config.gimbal_interval_s if self.config.gimbal_interval_s > 0.0 else self.config.interval_s
        )
        if (self.config.gimbal_enable or self.config.gimbal_only) and now >= self.next_gimbal_wall:
            self.next_gimbal_wall = now + gimbal_interval
            tilt = self._sample_angle(
                self.config.tilt_center_deg,
                self.config.tilt_amplitude_deg,
                self.config.tilt_min_deg,
                self.config.tilt_max_deg,
            )
            pan = self._sample_angle(
                self.config.pan_center_deg,
                self.config.pan_amplitude_deg,
                self.config.pan_min_deg,
                self.config.pan_max_deg,
            )
            tilt_msg = Float64()
            tilt_msg.data = float(tilt)
            self._tilt_pub.publish(tilt_msg)
            pan_msg = Float64()
            pan_msg.data = float(pan)
            self._pan_pub.publish(pan_msg)
            self.get_logger().info(f"Gimbal: tilt={tilt:.1f}° pan={pan:.1f}°")

        if self.config.gimbal_only:
            return

        # Follow distance tick.
        if self.pending_future is not None:
            return
        if self.config.count > 0 and self.applied_count >= self.config.count:
            return
        if now < self.next_update_wall:
            return

        d_target = round(
            sample_biased_range(
                self.rng,
                self.config.min_d_target,
                self.config.max_d_target,
                self.config.focus_min,
                self.config.focus_max,
                self.config.focus_weight,
            ),
            self.config.decimals,
        )

        params = [
            Parameter("d_target", Parameter.Type.DOUBLE, float(d_target)),
        ]
        self.pending_future = self.param_client.set_parameters_atomically(params)
        self.pending_future.add_done_callback(
            lambda future, d_target=d_target: self._on_set_complete(
                future,
                d_target,
            )
        )

    def _on_set_complete(self, future, d_target: float) -> None:
        self.pending_future = None
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to update follow params: {exc}")
            self.next_update_wall = time.monotonic() + self.config.interval_s
            return

        set_result = extract_set_result(result)
        if set_result is None or not set_result.successful:
            reason = set_result.reason if set_result is not None else "unknown failure"
            self.get_logger().error(f"Follow param update rejected: {reason}")
            self.next_update_wall = time.monotonic() + self.config.interval_s
            return

        self.applied_count += 1
        self.next_update_wall = time.monotonic() + self.config.interval_s
        self.get_logger().info(f"[{self.applied_count}] d_target={d_target:.2f}")

        if self.config.count > 0 and self.applied_count >= self.config.count:
            self.get_logger().info("Random follow param sweep complete")
            rclpy.shutdown()


class ScriptedFollowParams(Node):
    def __init__(self, config: FollowControlConfig) -> None:
        super().__init__("scripted_follow_params")
        self.config = config
        self.param_client = AsyncParameterClient(self, config.node_name)
        self.pending_future = None
        self.heading_supported = True
        self.applied_count = 0
        self.step_index = 0
        self.current_d_target = float(config.min_d_target)
        self.current_z_offset_m = float(config.min_z_min)
        self.current_heading_deg = 0.0
        self.current_pan_deg = clamp(config.pan_center_deg, config.pan_min_deg, config.pan_max_deg)
        self.current_tilt_deg = clamp(config.tilt_center_deg, config.tilt_min_deg, config.tilt_max_deg)
        self.target_step: Optional[dict[str, float | str]] = None
        self.next_update_wall = time.monotonic() + max(config.start_delay_s, 0.0)
        self.next_gimbal_publish_wall = 0.0
        self._tilt_pub = self.create_publisher(Float64, f"/{config.uav_name}/update_tilt", 10)
        self._pan_pub = self.create_publisher(Float64, f"/{config.uav_name}/update_pan", 10)
        self.pattern = self._build_pattern()

        self._validate_config()
        self._wait_for_follow_node()
        self._probe_heading_support()
        self.get_logger().info(
            "Scripted follow sweep ready: "
            f"target={config.node_name} uav={config.uav_name} interval={config.interval_s:.1f}s "
            f"d_target=[{config.min_d_target:.1f}, {config.max_d_target:.1f}] "
            f"heading_step={config.step_heading_deg:.1f}° pan_step={config.step_pan_deg:.1f}° "
            f"tilt_step={config.step_tilt_deg:.1f}° "
            f"steps={len(self.pattern)} count={'infinite' if config.count <= 0 else config.count}"
        )
        if config.start_delay_s > 0.0:
            self.get_logger().info(f"Initial delay {config.start_delay_s:.1f}s before first scripted update")
        self.timer = self.create_timer(0.1, self._tick)

    def _validate_config(self) -> None:
        cfg = self.config
        if cfg.interval_s <= 0.0:
            raise ValueError("interval must be > 0")
        if cfg.timeout_s <= 0.0:
            raise ValueError("timeout must be > 0")
        if cfg.min_d_target <= 0.0 or cfg.max_d_target < cfg.min_d_target:
            raise ValueError("d_target range must satisfy 0 < min <= max")
        if cfg.pan_max_deg < cfg.pan_min_deg:
            raise ValueError("pan bounds must satisfy pan-min <= pan-max")
        if cfg.tilt_max_deg < cfg.tilt_min_deg:
            raise ValueError("tilt bounds must satisfy tilt-min <= tilt-max")
        if cfg.gimbal_publish_hz <= 0.0:
            raise ValueError("gimbal-publish-hz must be > 0")
        if cfg.step_d_target <= 0.0:
            raise ValueError("step-d-target must be > 0")
        if cfg.step_z_min <= 0.0:
            raise ValueError("step-z-offset must be > 0")
        if cfg.min_z_min < 0.0:
            raise ValueError("min-z-offset must be >= 0")
        if cfg.min_d_target < cfg.min_z_min:
            raise ValueError("min-d-target must be >= min-z-offset")
        if cfg.step_heading_deg <= 0.0:
            raise ValueError("step-heading must be > 0")
        if cfg.step_pan_deg <= 0.0:
            raise ValueError("step-pan must be > 0")
        if cfg.step_tilt_deg <= 0.0:
            raise ValueError("step-tilt must be > 0")
        if cfg.opposite_settle_s < 0.0:
            raise ValueError("opposite-settle must be >= 0")
        if not self.pattern:
            raise ValueError("scripted pattern is empty")

    def _wait_for_follow_node(self) -> None:
        while rclpy.ok() and not self.param_client.wait_for_services(timeout_sec=self.config.timeout_s):
            self.get_logger().info(f"Waiting for parameter services on {self.config.node_name} ...")

    def _probe_heading_support(self) -> None:
        future = self.param_client.get_parameters(["leader_heading_offset_deg"])
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > self.config.timeout_s:
                self.heading_supported = False
                self.get_logger().warn(
                    "Timed out reading leader_heading_offset_deg; scripted mode will vary d_target/gimbal only"
                )
                return
            rclpy.spin_once(self, timeout_sec=0.05)

        try:
            result = future.result()
        except Exception as exc:
            self.heading_supported = False
            self.get_logger().warn(
                "Could not read leader_heading_offset_deg; scripted mode will vary d_target/gimbal only "
                f"({exc})"
            )
            return

        values = result.values if hasattr(result, "values") else None
        heading = parameter_value_to_float(values[0]) if values else None
        if heading is None:
            self.heading_supported = False
            self.get_logger().warn(
                "leader_heading_offset_deg is not available on the follow node; "
                "scripted mode will vary d_target/gimbal only"
            )
            return

        self.current_heading_deg = normalize_angle_deg(heading)
        self.heading_supported = True

    def _build_pattern(self) -> list[dict[str, float | str]]:
        cfg = self.config

        span = cfg.max_d_target - cfg.min_d_target
        distances = [
            cfg.min_d_target,
            cfg.min_d_target + span * 0.25,
            cfg.min_d_target + span * 0.50,
            cfg.max_d_target,
        ]

        headings = [
            0.0,
            180.0,
            -90.0,
            90.0,
            -135.0,
            45.0,
            -45.0,
            135.0
        ]

        image_modes = [
            ("center", cfg.pan_center_deg, cfg.tilt_center_deg),
            ("upper_right", cfg.pan_center_deg - cfg.pan_amplitude_deg, cfg.tilt_center_deg + cfg.tilt_amplitude_deg),
            ("upper_left", cfg.pan_center_deg + cfg.pan_amplitude_deg, cfg.tilt_center_deg + cfg.tilt_amplitude_deg),
            ("lower_right", cfg.pan_center_deg - cfg.pan_amplitude_deg, cfg.tilt_center_deg - cfg.tilt_amplitude_deg),
            ("lower_left", cfg.pan_center_deg + cfg.pan_amplitude_deg, cfg.tilt_center_deg - cfg.tilt_amplitude_deg),
        ]

        pattern: list[dict[str, float | str]] = []

        z_min = float(cfg.min_z_min)      # should be 3.0
        z_settle = 7.0

        for distance in distances:
            d_target = round(distance, cfg.decimals)
            z_max = float(distance)

            # Build z sweep: start low, move up to max=d_target.
            z_levels = [
                z_min,
                z_min + (z_max - z_min) * (1/3),
                z_min + (z_max - z_min) * (2/3),
                z_max,
            ]

            # Remove duplicate z values after rounding.
            z_levels = list(dict.fromkeys(round(z, cfg.decimals) for z in z_levels))

            # Clamp settle z so it is always valid for this d_target.
            z_settle_clamped = round(clamp(z_settle, z_min, z_max), cfg.decimals)

            for heading in headings:
                for z_offset in z_levels:
                    for image_name, pan, tilt in image_modes:
                        pattern.append(
                            {
                                "d_target": d_target,
                                "follow_z_offset_m": z_offset,
                                "heading": normalize_angle_deg(heading),
                                "pan": clamp(pan, cfg.pan_min_deg, cfg.pan_max_deg),
                                "tilt": clamp(tilt, cfg.tilt_min_deg, cfg.tilt_max_deg),
                                "corner": image_name,
                            }
                        )

                # After reaching max z for this heading, settle back to 7 m.
                # This gives the controller time to return to a neutral/default altitude
                # before the next heading/d_target block starts.
                pattern.append(
                    {
                        "d_target": d_target,
                        "follow_z_offset_m": z_settle_clamped,
                        "heading": normalize_angle_deg(heading),
                        "pan": clamp(cfg.pan_center_deg, cfg.pan_min_deg, cfg.pan_max_deg),
                        "tilt": clamp(cfg.tilt_center_deg, cfg.tilt_min_deg, cfg.tilt_max_deg),
                        "corner": "settle_center",
                    }
                )

        return pattern

    def _publish_gimbal(self) -> None:
        pan_msg = Float64()
        pan_msg.data = float(self.current_pan_deg)
        self._pan_pub.publish(pan_msg)

        tilt_msg = Float64()
        tilt_msg.data = float(self.current_tilt_deg)
        self._tilt_pub.publish(tilt_msg)

        self.next_gimbal_publish_wall = time.monotonic() + (1.0 / self.config.gimbal_publish_hz)

    def _tick(self) -> None:
        now = time.monotonic()

        if self.config.count > 0 and self.applied_count >= self.config.count:
            return

        if now >= self.next_update_wall:
            self.target_step = self.pattern[self.step_index % len(self.pattern)]
            self.step_index += 1
            self.applied_count += 1
            self.next_update_wall = now + self.config.interval_s
            self.get_logger().info(
                f"[{self.applied_count}] target "
                f"d_target={float(self.target_step['d_target']):.2f} "
                f"z_offset={float(self.target_step['follow_z_offset_m']):.2f} "
                f"heading={float(self.target_step['heading']):.1f}° "
                f"image_pos={self.target_step['corner']}"
            )

        if self.target_step is None:
            return

        target = self.target_step
        

        self.current_d_target = move_towards(
            self.current_d_target,
            float(target["d_target"]),
            self.config.step_d_target,
            )
        self.current_z_offset_m = move_towards(
            self.current_z_offset_m,
            float(target["follow_z_offset_m"]),
            self.config.step_z_min,
        )
        self.current_heading_deg = move_angle_towards(
            self.current_heading_deg,
            float(target["heading"]),
            self.config.step_heading_deg,
        )
        self.current_pan_deg = move_towards(
            self.current_pan_deg,
            float(target["pan"]),
            self.config.step_pan_deg,
        )
        self.current_tilt_deg = move_towards(
            self.current_tilt_deg,
            float(target["tilt"]),
            self.config.step_tilt_deg,
        )
        step = dict(target)
        step["d_target"] = self.current_d_target
        step["follow_z_offset_m"] = self.current_z_offset_m
        step["heading"] = self.current_heading_deg
        step["pan"] = self.current_pan_deg
        step["tilt"] = self.current_tilt_deg
        if now >= self.next_gimbal_publish_wall:
            self._publish_gimbal()
        if self.pending_future is None:
            self._set_follow_step(step, include_heading=self.heading_supported)

    def _set_follow_step(
        self,
        step: dict[str, float | str],
        include_heading: bool,
    ) -> None:
        params = [
            Parameter("d_target", Parameter.Type.DOUBLE, float(step["d_target"])),
            Parameter("follow_z_offset_m", Parameter.Type.DOUBLE, float(step["follow_z_offset_m"])),
        ]
        if include_heading:
            params.append(
                Parameter("leader_heading_offset_deg", Parameter.Type.DOUBLE, float(step["heading"]))
            )
        self.pending_future = self.param_client.set_parameters_atomically(params)
        self.pending_future.add_done_callback(
            lambda future, step=step, include_heading=include_heading: self._on_set_complete(
                future,
                step,
                include_heading,
            )
        )

    def _on_set_complete(
        self,
        future,
        step: dict[str, float | str],
        included_heading: bool,
    ) -> None:
        self.pending_future = None
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to update scripted follow params: {exc}")
            self.next_update_wall = time.monotonic() + self.config.interval_s
            return

        set_result = extract_set_result(result)
        if set_result is None or not set_result.successful:
            reason = set_result.reason if set_result is not None else "unknown failure"
            if included_heading:
                self.heading_supported = False
                self.get_logger().warn(
                    "leader_heading_offset_deg was rejected; continuing scripted mode with d_target/gimbal only "
                    f"({reason})"
                )
                self._set_follow_step(step, include_heading=False)
                return

            self.get_logger().error(f"Scripted follow param update rejected: {reason}")
            return

        if self.config.count > 0 and self.applied_count >= self.config.count:
            self.get_logger().info("Scripted follow sweep complete")
            rclpy.shutdown()


class KeyboardFollowParams(Node):
    def __init__(self, config: FollowControlConfig) -> None:
        super().__init__("keyboard_follow_params")
        self.config = config
        self.param_client = AsyncParameterClient(self, config.node_name)
        self.pending_get = None
        self.pending_set = None
        self.current_d_target: Optional[float] = None
        self.current_z_offset_m: Optional[float] = None
        self.current_heading_offset_deg: Optional[float] = None
        self.heading_supported = False
        self.z_offset_supported = False
        self.current_pan_deg = clamp(config.pan_center_deg, config.pan_min_deg, config.pan_max_deg)
        self.current_tilt_deg = clamp(config.tilt_center_deg, config.tilt_min_deg, config.tilt_max_deg)
        self.gimbal_active = False
        self.next_gimbal_publish_wall = 0.0
        self._tilt_pub = self.create_publisher(Float64, f"/{config.uav_name}/update_tilt", 10)
        self._pan_pub = self.create_publisher(Float64, f"/{config.uav_name}/update_pan", 10)
        if not sys.stdin.isatty():
            raise RuntimeError("keyboard mode requires an interactive terminal")
        self.stdin_fd = sys.stdin.fileno()
        self.stdin_attrs = termios.tcgetattr(self.stdin_fd)
        self._validate_config()
        self._wait_for_follow_node()
        self._enter_raw_mode()
        self._print_help()
        self.timer = self.create_timer(0.05, self._tick)
        self._request_current_values()

    def _validate_config(self) -> None:
        cfg = self.config
        if cfg.timeout_s <= 0.0:
            raise ValueError("timeout must be > 0")
        if cfg.step_d_target <= 0.0:
            raise ValueError("step-d-target must be > 0")
        if cfg.step_z_min <= 0.0:
            raise ValueError("step-z-offset must be > 0")
        if cfg.min_z_min < 0.0:
            raise ValueError("min-z-offset must be >= 0")
        if cfg.step_heading_deg <= 0.0:
            raise ValueError("step-heading must be > 0")
        if cfg.step_pan_deg <= 0.0:
            raise ValueError("step-pan must be > 0")
        if cfg.step_tilt_deg <= 0.0:
            raise ValueError("step-tilt must be > 0")
        if cfg.min_d_target <= 0.0 or cfg.max_d_target < cfg.min_d_target:
            raise ValueError("d_target bounds must satisfy 0 < min <= max")
        if cfg.pan_max_deg < cfg.pan_min_deg:
            raise ValueError("pan bounds must satisfy pan-min <= pan-max")
        if cfg.tilt_max_deg < cfg.tilt_min_deg:
            raise ValueError("tilt bounds must satisfy tilt-min <= tilt-max")
        if cfg.gimbal_publish_hz <= 0.0:
            raise ValueError("gimbal-publish-hz must be > 0")

    def _wait_for_follow_node(self) -> None:
        while rclpy.ok() and not self.param_client.wait_for_services(timeout_sec=self.config.timeout_s):
            self.get_logger().info(f"Waiting for parameter services on {self.config.node_name} ...")

    def _enter_raw_mode(self) -> None:
        tty.setcbreak(self.stdin_fd)

    def restore_terminal(self) -> None:
        try:
            termios.tcsetattr(self.stdin_fd, termios.TCSADRAIN, self.stdin_attrs)
        except Exception:
            pass

    def destroy_node(self):
        self.restore_terminal()
        return super().destroy_node()

    def _print_help(self) -> None:
        print(
            "\nFollow keyboard control\n"
            f"  target node: {self.config.node_name}\n"
            f"  w/s: d_target -/+ {self.config.step_d_target:g} m\n"
            f"  z/x: follow_z_offset_m -/+ {self.config.step_z_min:g} m "
            f"(clamped to [{self.config.min_z_min:g}, d_target])\n"
            f"  a/d: leader_heading_offset_deg -/+ {self.config.step_heading_deg:g} deg\n"
            f"  j/l: gimbal pan -/+ {self.config.step_pan_deg:g} deg\n"
            f"  i/k: gimbal tilt + / - {self.config.step_tilt_deg:g} deg\n"
            "  c: center gimbal\n"
            "  p: print current values\n"
            "  r: refresh current values\n"
            "  h: show help\n"
            "  q: quit\n"
            "  Ctrl-C also exits\n",
            flush=True,
        )

    def _tick(self) -> None:
        now = time.monotonic()
        if self.gimbal_active and now >= self.next_gimbal_publish_wall:
            self._publish_gimbal()

        if self.pending_get is not None or self.pending_set is not None:
            return

        ready, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not ready:
            return

        char = sys.stdin.read(1)
        if not char:
            return

        if char == "q":
            self.get_logger().info("Follow parameter control exiting")
            rclpy.shutdown()
            return
        if char == "h":
            self._print_help()
            return
        if char == "r":
            self._request_current_values()
            return
        if char == "p":
            self._print_status()
            return

        if char in ("j", "l", "i", "k", "c"):
            self._handle_gimbal_key(char)
            return

        if char in ("s", "w"):
            if self.current_d_target is None:
                self.get_logger().warn("Current d_target is not loaded yet; refreshing")
                self._request_current_values()
                return

            delta = self.config.step_d_target if char == "s" else -self.config.step_d_target
            d_target = clamp(
                self.current_d_target + delta,
                self.config.min_d_target,
                self.config.max_d_target,
            )

            # Keep z valid if d_target is reduced below the current z offset.
            params = {"d_target": d_target}
            if self.current_z_offset_m is not None:
                z_offset = clamp(self.current_z_offset_m, self.config.min_z_min, d_target)
                params["follow_z_offset_m"] = z_offset

            self._set_follow_params(params)

            return
        if char in ("z", "x"):
            if self.current_z_offset_m is None:
                if not self.z_offset_supported:
                    self.get_logger().warn(
                        "follow_z_offset_m is not available on this follow node"
                    )
                    return
                self.get_logger().warn("Current follow_z_offset_m is not loaded yet; refreshing")
                self._request_current_values()
                return

            if self.current_d_target is None:
                self.get_logger().warn("Current d_target is not loaded yet; refreshing")
                self._request_current_values()
                return

            delta = self.config.step_z_min if char == "x" else -self.config.step_z_min
            z_offset = clamp(
                self.current_z_offset_m + delta,
                self.config.min_z_min,
                self.current_d_target,
            )
            self._set_follow_param("follow_z_offset_m", z_offset)
            return
        
        if char in ("a", "d"):
            if self.current_heading_offset_deg is None:
                if not self.heading_supported:
                    self.get_logger().warn(
                        "leader_heading_offset_deg is not available on this follow node; "
                        "it works with odom follow mode"
                    )
                    return
                self.get_logger().warn("Current leader_heading_offset_deg is not loaded yet; refreshing")
                self._request_current_values()
                return
            delta = self.config.step_heading_deg if char == "d" else -self.config.step_heading_deg
            heading = normalize_angle_deg(self.current_heading_offset_deg + delta)
            self._set_follow_param("leader_heading_offset_deg", heading)
            return

    def _request_current_values(self) -> None:
        future = self.param_client.get_parameters(
            ["d_target", "follow_z_offset_m", "leader_heading_offset_deg"]
        )
        self.pending_get = future
        future.add_done_callback(self._on_get_complete)

    def _on_get_complete(self, future) -> None:
        self.pending_get = None
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to read current follow params: {exc}")
            return

        values = None
        if isinstance(result, GetParameters.Response):
            values = result.values
        elif hasattr(result, "values"):
            values = result.values

        if values is None or len(values) != 3:
            self.get_logger().error("Follow param query returned an unexpected response")
            return

        d_target = parameter_value_to_float(values[0])
        z_offset = parameter_value_to_float(values[1])
        heading = parameter_value_to_float(values[2])

        if d_target is None:
            self.get_logger().error("Follow node did not return a numeric d_target parameter")
            return

        self.current_d_target = d_target

        if z_offset is None:
            self.current_z_offset_m = None
            self.z_offset_supported = False
        else:
            self.current_z_offset_m = clamp(
                z_offset,
                self.config.min_z_min,
                self.current_d_target,
            )
            self.z_offset_supported = True

        if heading is None:
            self.current_heading_offset_deg = None
            self.heading_supported = False
        else:
            self.current_heading_offset_deg = normalize_angle_deg(heading)
            self.heading_supported = True

        self._print_status(prefix="Current")

    def _set_follow_param(self, name: str, value: float) -> None:
        self._set_follow_params({name: value})


    def _set_follow_params(self, values: dict[str, float]) -> None:
        params = [
            Parameter(name, Parameter.Type.DOUBLE, float(value))
            for name, value in values.items()
        ]
        future = self.param_client.set_parameters_atomically(params)
        self.pending_set = future
        future.add_done_callback(
            lambda done, values=values: self._on_set_complete(done, values)
        )

    def _on_set_complete(self, future, values: dict[str, float]) -> None:
        self.pending_set = None
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f"Failed to update follow params: {exc}")
            return

        set_result = extract_set_result(result)
        if set_result is None or not set_result.successful:
            reason = set_result.reason if set_result is not None else "unknown failure"
            self.get_logger().error(f"Follow param update rejected: {reason}")
            self._request_current_values()
            return

        if "d_target" in values:
            self.current_d_target = values["d_target"]

        if "follow_z_offset_m" in values:
            self.current_z_offset_m = values["follow_z_offset_m"]
            self.z_offset_supported = True

        if "leader_heading_offset_deg" in values:
            self.current_heading_offset_deg = normalize_angle_deg(values["leader_heading_offset_deg"])
            self.heading_supported = True

        self._print_status(prefix="Applied")

    def _handle_gimbal_key(self, char: str) -> None:
        if char == "j":
            self.current_pan_deg += self.config.step_pan_deg
        elif char == "l":
            self.current_pan_deg -= self.config.step_pan_deg
        elif char == "i":
            self.current_tilt_deg += self.config.step_tilt_deg
        elif char == "k":
            self.current_tilt_deg -= self.config.step_tilt_deg
        elif char == "c":
            self.current_pan_deg = self.config.pan_center_deg
            self.current_tilt_deg = self.config.tilt_center_deg

        self.current_pan_deg = clamp(self.current_pan_deg, self.config.pan_min_deg, self.config.pan_max_deg)
        self.current_tilt_deg = clamp(self.current_tilt_deg, self.config.tilt_min_deg, self.config.tilt_max_deg)
        self.gimbal_active = True
        self._publish_gimbal()
        self._print_status(prefix="Gimbal")

    def _publish_gimbal(self) -> None:
        pan_msg = Float64()
        pan_msg.data = float(self.current_pan_deg)
        self._pan_pub.publish(pan_msg)

        tilt_msg = Float64()
        tilt_msg.data = float(self.current_tilt_deg)
        self._tilt_pub.publish(tilt_msg)

        self.next_gimbal_publish_wall = time.monotonic() + (1.0 / self.config.gimbal_publish_hz)

    def _print_status(self, prefix: str = "Status") -> None:
        heading = (
            f"{self.current_heading_offset_deg:.1f} deg"
            if self.current_heading_offset_deg is not None
            else "not available"
        )
        d_target = f"{self.current_d_target:.2f} m" if self.current_d_target is not None else "unknown"
        z_offset = (
            f"{self.current_z_offset_m:.2f} m"
            if self.current_z_offset_m is not None
            else "not available"
        )

        print(
            f"{prefix}: d_target={d_target}, "
            f"follow_z_offset={z_offset}, "
            f"leader_heading_offset={heading}, "
            f"pan={self.current_pan_deg:.1f} deg, "
            f"tilt={self.current_tilt_deg:.1f} deg",
            flush=True,
        )


def parse_args() -> FollowControlConfig:
    raw_argv = list(sys.argv[1:])
    if raw_argv and raw_argv[0] in ("keyboard", "params", "random", "scripted"):
        raw_argv = ["--mode", raw_argv[0], *raw_argv[1:]]

    parser = argparse.ArgumentParser(
        description="Keyboard, random, or scripted runtime tuning for the follow controller"
    )
    parser.add_argument(
        "--mode",
        choices=("keyboard", "params", "random", "scripted"),
        default="keyboard",
        help=(
            "keyboard/params: keyboard follow and gimbal tuning; "
            "random: random d_target/gimbal updates; "
            "scripted: deterministic distance/heading/gimbal pattern for dataset collection"
        ),
    )
    parser.add_argument("--node", default="/follow_uav", help="Target follow node, default: /follow_uav")
    parser.add_argument("--timeout", type=float, default=1.0, help="Service wait timeout in seconds, default: 1")
    parser.add_argument("--step-z-min", type=float, default=4.0, help="Keyboard/scripted follow_z_offset_m change per step, default: 1")
    parser.add_argument("--step-z-alt", dest="step_z_min", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--step-d-target", type=float, default=1.0, help="Keyboard/scripted d_target change per step, default: 1")
    parser.add_argument("--step-heading", type=float, default=5.0, help="Keyboard heading-offset change per keypress, default: 15 deg")
    parser.add_argument("--step-pan", type=float, default=5.0, help="Keyboard gimbal pan change per keypress, default: 5 deg; scripted default: 3")
    parser.add_argument("--step-tilt", type=float, default=2.0, help="Keyboard gimbal tilt change per keypress, default: 5 deg; scripted default: 2")
    parser.add_argument("--interval", type=float, default=1.0, help="Seconds between random/scripted update steps, default: 10; scripted default: 2")
    parser.add_argument("--start-delay", type=float, default=0.0, help="Initial delay before the first random/scripted update, default: 0")
    parser.add_argument("--count", type=int, default=0, help="Number of random/scripted updates to apply, default: 0 (infinite)")
    parser.add_argument("--min-z-min",type=float, default=8.0, help="Minimum follow_z_offset_m, default: 3")
    parser.add_argument("--max-z-min", type=float, default=24.0, help=argparse.SUPPRESS)
    parser.add_argument("--min-z-alt", dest="min_z_min", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--max-z-alt", dest="max_z_min", type=float, help=argparse.SUPPRESS)
    parser.add_argument("--min-d-target", type=float, default=8.0, help="Minimum d_target in random mode, default: 5; scripted default: 6")
    parser.add_argument("--max-d-target", type=float, default=100.0, help="Maximum d_target in random/keyboard mode, default: 500")
    parser.add_argument("--focus-min", type=float, default=40.0, help="Lower edge of the preferred random sampling band, default: 40")
    parser.add_argument("--focus-max", type=float, default=160.0, help="Upper edge of the preferred random sampling band, default: 160")
    parser.add_argument("--focus-weight", type=float, default=0.75, help="Probability mass to place in the preferred random band, default: 0.7")
    parser.add_argument("--decimals", type=int, default=2, help="Decimal places to keep in random mode, default: 2")
    parser.add_argument("--seed", type=int, default=None, help="Optional RNG seed for repeatability")
    # Gimbal sweep options (random mode)
    parser.add_argument("--uav-name", default="dji0", help="UAV name for gimbal topics, default: dji0")
    parser.add_argument("--gimbal", action="store_true", help="Also randomly sweep pan/tilt alongside d_target")
    parser.add_argument("--gimbal-only", action="store_true", help="Only sweep pan/tilt, skip d_target params")
    parser.add_argument("--gimbal-interval", type=float, default=4.0, help="Gimbal update interval in seconds (default: same as --interval)")
    parser.add_argument("--tilt-center", type=float, default=-0.0, help="Tilt centre in degrees, default: -45")
    parser.add_argument("--tilt-amplitude", type=float, default=180.0, help="Tilt random amplitude in degrees, default: 15")
    parser.add_argument("--tilt-min", type=float, default=-75.0, help="Tilt lower limit in degrees, default: -75")
    parser.add_argument("--tilt-max", type=float, default=30.0, help="Tilt upper limit in degrees, default: -15")
    parser.add_argument("--pan-center", type=float, default=0.0, help="Pan centre in degrees, default: 0")
    parser.add_argument("--pan-amplitude", type=float, default=360.0, help="Pan random amplitude in degrees, default: 20")
    parser.add_argument("--pan-min", type=float, default=-60.0, help="Pan lower limit in degrees, default: -45")
    parser.add_argument("--pan-max", type=float, default=60.0, help="Pan upper limit in degrees, default: 45")
    parser.add_argument("--gimbal-publish-hz", type=float, default=10.0, help="Keyboard/scripted gimbal command repeat rate, default: 5")
    parser.add_argument("--opposite-settle", type=float, default=5.0, help="Scripted hold time after reaching 180 deg opposite-side targets, default: 20")
    args = parser.parse_args(raw_argv)
    mode = "keyboard" if args.mode == "params" else args.mode
    has_arg = lambda name: any(item == name or item.startswith(f"{name}=") for item in raw_argv)
    # Scripted mode
    if mode == "scripted":
        if not has_arg("--min-d-target"):
            args.min_d_target = 6.0
        if not has_arg("--max-d-target"):
            args.max_d_target = 500.0
        if not has_arg("--interval"):
            args.interval = 3.0
        if not has_arg("--step-heading"):
            args.step_heading = 15.0
        if not has_arg("--step-pan"):
            args.step_pan = 5.0
        if not has_arg("--step-tilt"):
            args.step_tilt = (1 / 4) * args.step_pan 
        if not has_arg("--pan-amplitude"):
            args.pan_amplitude = 135.0
        if not has_arg("--tilt-amplitude"):
            args.tilt_amplitude = (1 / 4) * args.pan_amplitude
        if not has_arg("--gimbal-publish-hz"):
            args.gimbal_publish_hz = 10.0
    return FollowControlConfig(
        mode=mode,
        node_name=args.node,
        timeout_s=float(args.timeout),
        step_z_min=float(args.step_z_min),
        step_d_target=float(args.step_d_target),
        step_heading_deg=float(args.step_heading),
        step_pan_deg=float(args.step_pan),
        step_tilt_deg=float(args.step_tilt),
        interval_s=float(args.interval),
        start_delay_s=float(args.start_delay),
        count=int(args.count),
        min_z_min=float(args.min_z_min),
        max_z_min=float(args.max_z_min),
        min_d_target=float(args.min_d_target),
        max_d_target=float(args.max_d_target),
        focus_min=float(args.focus_min),
        focus_max=float(args.focus_max),
        focus_weight=float(args.focus_weight),
        decimals=int(args.decimals),
        seed=args.seed,
        uav_name=args.uav_name,
        gimbal_enable=args.gimbal or args.gimbal_only,
        gimbal_only=args.gimbal_only,
        gimbal_interval_s=float(args.gimbal_interval),
        tilt_center_deg=float(args.tilt_center),
        tilt_amplitude_deg=float(args.tilt_amplitude),
        tilt_min_deg=float(args.tilt_min),
        tilt_max_deg=float(args.tilt_max),
        pan_center_deg=float(args.pan_center),
        pan_amplitude_deg=float(args.pan_amplitude),
        pan_min_deg=float(args.pan_min),
        pan_max_deg=float(args.pan_max),
        gimbal_publish_hz=float(args.gimbal_publish_hz),
        opposite_settle_s=float(args.opposite_settle),
    )


def main() -> None:
    config = parse_args()
    rclpy.init()
    node = None
    try:
        if config.mode == "random":
            node = RandomFollowParams(config)
        elif config.mode == "scripted":
            node = ScriptedFollowParams(config)
        else:
            node = KeyboardFollowParams(config)
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
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
