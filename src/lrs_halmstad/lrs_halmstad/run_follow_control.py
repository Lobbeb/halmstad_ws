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
from rcl_interfaces.msg import SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.parameter_client import AsyncParameterClient


@dataclass
class FollowControlConfig:
    mode: str
    node_name: str
    timeout_s: float
    step_z_alt: float
    step_d_target: float
    interval_s: float
    start_delay_s: float
    count: int
    min_z_alt: float
    max_z_alt: float
    min_d_target: float
    max_d_target: float
    focus_min: float
    focus_max: float
    focus_weight: float
    decimals: int
    seed: Optional[int]


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


class RandomFollowParams(Node):
    def __init__(self, config: FollowControlConfig) -> None:
        super().__init__("random_follow_params")
        self.config = config
        self.rng = random.Random(config.seed)
        self.param_client = AsyncParameterClient(self, config.node_name)
        self.pending_future = None
        self.applied_count = 0
        self.next_update_wall = time.monotonic() + max(config.start_delay_s, 0.0)
        self.timer = self.create_timer(0.2, self._tick)

        self._validate_config()
        self._wait_for_follow_node()

        seed_text = "time-based" if config.seed is None else str(config.seed)
        self.get_logger().info(
            "Random follow param sweep ready: "
            f"target={config.node_name} "
            f"interval={config.interval_s:.1f}s "
            f"z_alt=[{config.min_z_alt}, {config.max_z_alt}] "
            f"d_target=[{config.min_d_target}, {config.max_d_target}] "
            f"focus=[{config.focus_min}, {config.focus_max}] "
            f"focus_weight={config.focus_weight:.2f} "
            f"count={'infinite' if config.count <= 0 else config.count} "
            f"seed={seed_text}"
        )
        if config.start_delay_s > 0.0:
            self.get_logger().info(f"Initial delay {config.start_delay_s:.1f}s before first random update")

    def _validate_config(self) -> None:
        cfg = self.config
        if cfg.interval_s <= 0.0:
            raise ValueError("interval must be > 0")
        if cfg.timeout_s <= 0.0:
            raise ValueError("timeout must be > 0")
        if cfg.min_z_alt <= 0.0 or cfg.max_z_alt <= 0.0 or cfg.max_z_alt < cfg.min_z_alt:
            raise ValueError("z_alt range must satisfy 0 < min <= max")
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

    def _tick(self) -> None:
        if self.pending_future is not None:
            return
        if self.config.count > 0 and self.applied_count >= self.config.count:
            return
        if time.monotonic() < self.next_update_wall:
            return

        z_alt = round(
            sample_biased_range(
                self.rng,
                self.config.min_z_alt,
                self.config.max_z_alt,
                self.config.focus_min,
                self.config.focus_max,
                self.config.focus_weight,
            ),
            self.config.decimals,
        )
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
        d_euclidean = math.hypot(d_target, z_alt)

        params = [
            Parameter("d_target", Parameter.Type.DOUBLE, float(d_target)),
            Parameter("z_alt", Parameter.Type.DOUBLE, float(z_alt)),
        ]
        self.pending_future = self.param_client.set_parameters_atomically(params)
        self.pending_future.add_done_callback(
            lambda future, d_target=d_target, z_alt=z_alt, d_euclidean=d_euclidean: self._on_set_complete(
                future,
                d_target,
                z_alt,
                d_euclidean,
            )
        )

    def _on_set_complete(self, future, d_target: float, z_alt: float, d_euclidean: float) -> None:
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
        self.get_logger().info(
            f"[{self.applied_count}] d_target={d_target:.2f} z_alt={z_alt:.2f} "
            f"d_euclidean={d_euclidean:.2f}"
        )

        if self.config.count > 0 and self.applied_count >= self.config.count:
            self.get_logger().info("Random follow param sweep complete")
            rclpy.shutdown()


class KeyboardFollowParams(Node):
    def __init__(self, config: FollowControlConfig) -> None:
        super().__init__("keyboard_follow_params")
        self.config = config
        self.param_client = AsyncParameterClient(self, config.node_name)
        self.pending_get = None
        self.pending_set = None
        self.current_z_alt: Optional[float] = None
        self.current_d_target: Optional[float] = None
        if not sys.stdin.isatty():
            raise RuntimeError("params mode requires an interactive terminal")
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
        if cfg.step_z_alt <= 0.0:
            raise ValueError("step-z-alt must be > 0")
        if cfg.step_d_target <= 0.0:
            raise ValueError("step-d-target must be > 0")
        if cfg.min_z_alt <= 0.0 or cfg.max_z_alt < cfg.min_z_alt:
            raise ValueError("z_alt bounds must satisfy 0 < min <= max")
        if cfg.min_d_target <= 0.0 or cfg.max_d_target < cfg.min_d_target:
            raise ValueError("d_target bounds must satisfy 0 < min <= max")

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
            "\nFollow parameter control\n"
            f"  target node: {self.config.node_name}\n"
            f"  w: z_alt +{self.config.step_z_alt:g}\n"
            f"  s: z_alt -{self.config.step_z_alt:g}\n"
            f"  d: d_target +{self.config.step_d_target:g}\n"
            f"  a: d_target -{self.config.step_d_target:g}\n"
            "  r: refresh current values\n"
            "  h: show help\n"
            "  q: quit\n"
            "  Ctrl-C also exits\n",
            flush=True,
        )

    def _tick(self) -> None:
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

        if self.current_z_alt is None or self.current_d_target is None:
            self.get_logger().warn("Current follow params not loaded yet; press 'r' in a moment")
            self._request_current_values()
            return

        z_alt = self.current_z_alt
        d_target = self.current_d_target

        if char == "w":
            z_alt += self.config.step_z_alt
        elif char == "s":
            z_alt -= self.config.step_z_alt
        elif char == "d":
            d_target += self.config.step_d_target
        elif char == "a":
            d_target -= self.config.step_d_target
        else:
            return

        z_alt = min(max(z_alt, self.config.min_z_alt), self.config.max_z_alt)
        d_target = min(max(d_target, self.config.min_d_target), self.config.max_d_target)
        self._set_follow_values(d_target=d_target, z_alt=z_alt)

    def _request_current_values(self) -> None:
        future = self.param_client.get_parameters(["d_target", "z_alt"])
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

        if values is None or len(values) != 2:
            self.get_logger().error("Follow param query returned an unexpected response")
            return

        d_target = float(values[0].double_value)
        z_alt = float(values[1].double_value)
        self.current_d_target = d_target
        self.current_z_alt = z_alt
        print(f"Current values: d_target={d_target:.2f} z_alt={z_alt:.2f}", flush=True)

    def _set_follow_values(self, d_target: float, z_alt: float) -> None:
        params = [
            Parameter("d_target", Parameter.Type.DOUBLE, float(d_target)),
            Parameter("z_alt", Parameter.Type.DOUBLE, float(z_alt)),
        ]
        future = self.param_client.set_parameters_atomically(params)
        self.pending_set = future
        future.add_done_callback(
            lambda done, d_target=d_target, z_alt=z_alt: self._on_set_complete(done, d_target, z_alt)
        )

    def _on_set_complete(self, future, d_target: float, z_alt: float) -> None:
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

        self.current_d_target = d_target
        self.current_z_alt = z_alt
        print(f"Applied: d_target={d_target:.2f} z_alt={z_alt:.2f}", flush=True)


def parse_args() -> FollowControlConfig:
    parser = argparse.ArgumentParser(
        description="Keyboard or random runtime tuning for the follow controller"
    )
    parser.add_argument(
        "--mode",
        choices=("keyboard", "params", "random"),
        default="keyboard",
        help="keyboard/params: keyboard d_target/z_alt tuning; random: random d_target/z_alt updates",
    )
    parser.add_argument("--node", default="/follow_uav", help="Target follow node, default: /follow_uav")
    parser.add_argument("--timeout", type=float, default=1.0, help="Service wait timeout in seconds, default: 1")
    parser.add_argument("--step-z-alt", type=float, default=5.0, help="Keyboard z_alt change per keypress, default: 5")
    parser.add_argument("--step-d-target", type=float, default=5.0, help="Keyboard d_target change per keypress, default: 5")
    parser.add_argument("--interval", type=float, default=10.0, help="Seconds between random updates, default: 10")
    parser.add_argument("--start-delay", type=float, default=0.0, help="Initial delay before the first random update, default: 0")
    parser.add_argument("--count", type=int, default=0, help="Number of random updates to apply, default: 0 (infinite)")
    parser.add_argument("--min-z-alt", type=float, default=2.0, help="Minimum z_alt in params/random mode, default: 2")
    parser.add_argument("--max-z-alt", type=float, default=40.0, help="Maximum z_alt in params/random mode, default: 40")
    parser.add_argument("--min-d-target", type=float, default=1.0, help="Minimum d_target in params/random mode, default: 1")
    parser.add_argument("--max-d-target", type=float, default=20.0, help="Maximum d_target in params/random mode, default: 20")
    parser.add_argument("--focus-min", type=float, default=5.0, help="Lower edge of the preferred random sampling band, default: 5")
    parser.add_argument("--focus-max", type=float, default=15.0, help="Upper edge of the preferred random sampling band, default: 15")
    parser.add_argument("--focus-weight", type=float, default=0.7, help="Probability mass to place in the preferred random band, default: 0.7")
    parser.add_argument("--decimals", type=int, default=2, help="Decimal places to keep in random mode, default: 2")
    parser.add_argument("--seed", type=int, default=None, help="Optional RNG seed for repeatability")
    args = parser.parse_args()
    mode = "keyboard" if args.mode == "params" else args.mode
    return FollowControlConfig(
        mode=mode,
        node_name=args.node,
        timeout_s=float(args.timeout),
        step_z_alt=float(args.step_z_alt),
        step_d_target=float(args.step_d_target),
        interval_s=float(args.interval),
        start_delay_s=float(args.start_delay),
        count=int(args.count),
        min_z_alt=float(args.min_z_alt),
        max_z_alt=float(args.max_z_alt),
        min_d_target=float(args.min_d_target),
        max_d_target=float(args.max_d_target),
        focus_min=float(args.focus_min),
        focus_max=float(args.focus_max),
        focus_weight=float(args.focus_weight),
        decimals=int(args.decimals),
        seed=args.seed,
    )


def main() -> None:
    config = parse_args()
    rclpy.init()
    node = None
    try:
        if config.mode == "random":
            node = RandomFollowParams(config)
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
