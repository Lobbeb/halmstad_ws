import argparse
import math
import sys
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import Joy
from std_msgs.msg import Float32, Float64

from lrs_halmstad.follow.follow_math import wrap_pi, yaw_from_quat


@dataclass
class PoseState:
    x: float
    y: float
    z: float
    yaw: float


def _guess_workspace_root() -> Path:
    search_roots = [Path.cwd(), Path(__file__).resolve()]
    for root in search_roots:
        for parent in [root, *root.parents]:
            if (parent / "src" / "lrs_halmstad").exists():
                return parent
    return Path.cwd()


def _default_log_path(uav_name: str) -> Path:
    workspace_root = _guess_workspace_root()
    log_dir = workspace_root / "debug_logs" / "uav_commands"
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return log_dir / f"uav_command_log_{uav_name}_{timestamp}.log"


class UavCommandLogger(Node):
    def __init__(
        self,
        uav_name: str,
        log_path: Path,
        include_camera: bool,
        timeout_s: Optional[float],
        leader_estimate_topic: str,
        leader_actual_topic: str,
    ):
        super().__init__("uav_command_logger")
        self.uav_name = uav_name.strip().strip("/")
        self.include_camera = include_camera
        self.log_path = log_path
        self.log_path.parent.mkdir(parents=True, exist_ok=True)
        self.log_fp = self.log_path.open("w", encoding="utf-8", buffering=1)

        self.flush_delay_s = 0.05
        self.flush_due_ns: Optional[int] = None
        self.pending_sources: set[str] = set()

        self.body_topic = f"/{self.uav_name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
        self.pose_topic = f"/{self.uav_name}/pose"
        self.pan_topic = f"/{self.uav_name}/update_pan"
        self.tilt_topic = f"/{self.uav_name}/update_tilt"
        self.actual_pan_topic = f"/{self.uav_name}/follow/actual/pan_deg"
        self.actual_tilt_topic = f"/{self.uav_name}/follow/actual/tilt_deg"
        self.leader_follow_yaw_topic = f"/{self.uav_name}/follow/debug/leader_follow_yaw_rad"
        self.leader_estimate_yaw_topic = f"/{self.uav_name}/follow/debug/leader_estimate_yaw_rad"
        self.leader_estimate_topic = str(leader_estimate_topic).strip() or "/coord/leader_estimate"
        self.leader_actual_topic = str(leader_actual_topic).strip() or "/a201_0000/amcl_pose_odom"

        self.last_body_time = None
        self.last_body_dt_ms: Optional[float] = None
        self.prev_body_cmd: Optional[PoseState] = None
        self.body_cmd: Optional[PoseState] = None
        self.actual_uav_pose: Optional[PoseState] = None
        self.actual_uav_prev_pose: Optional[PoseState] = None
        self.actual_uav_prev_stamp_s: Optional[float] = None
        self.actual_uav_speed_mps: Optional[float] = None
        self.cmd_pan_deg: Optional[float] = None
        self.cmd_tilt_deg: Optional[float] = None
        self.actual_pan_deg: Optional[float] = None
        self.actual_tilt_deg: Optional[float] = None
        self.leader_estimate_pose: Optional[PoseState] = None
        self.leader_actual_pose: Optional[PoseState] = None
        self.leader_follow_yaw_dbg_rad: Optional[float] = None
        self.leader_estimate_yaw_dbg_rad: Optional[float] = None

        self.body_sub = self.create_subscription(Joy, self.body_topic, self._on_body_cmd, 50)
        self.pose_sub = self.create_subscription(PoseStamped, self.pose_topic, self._on_actual_uav_pose, 50)
        self.leader_estimate_sub = self.create_subscription(
            PoseStamped,
            self.leader_estimate_topic,
            self._on_leader_estimate_pose,
            20,
        )
        self.leader_actual_sub = self.create_subscription(
            Odometry,
            self.leader_actual_topic,
            self._on_leader_actual_pose,
            20,
        )
        self.leader_follow_yaw_sub = self.create_subscription(
            Float32,
            self.leader_follow_yaw_topic,
            self._on_leader_follow_yaw_dbg,
            20,
        )
        self.leader_estimate_yaw_sub = self.create_subscription(
            Float32,
            self.leader_estimate_yaw_topic,
            self._on_leader_estimate_yaw_dbg,
            20,
        )

        self.pan_sub = None
        self.tilt_sub = None
        self.actual_pan_sub = None
        self.actual_tilt_sub = None
        if self.include_camera:
            self.pan_sub = self.create_subscription(Float64, self.pan_topic, self._on_pan_cmd, 50)
            self.tilt_sub = self.create_subscription(Float64, self.tilt_topic, self._on_tilt_cmd, 50)
            self.actual_pan_sub = self.create_subscription(
                Float32, self.actual_pan_topic, self._on_actual_pan, 50
            )
            self.actual_tilt_sub = self.create_subscription(
                Float32, self.actual_tilt_topic, self._on_actual_tilt, 50
            )

        self.flush_timer = self.create_timer(self.flush_delay_s, self._maybe_flush_snapshot)
        if timeout_s is not None and timeout_s > 0.0:
            self.create_timer(float(timeout_s), self._shutdown_once)

        self._write_block(
            [
                f"# Logging UAV commands to {self.log_path}",
                f"# Body topic: {self.body_topic}",
                f"# UAV pose topic: {self.pose_topic}",
                f"# Leader estimate topic: {self.leader_estimate_topic}",
                f"# Leader actual topic (AMCL): {self.leader_actual_topic}",
                f"# Follow debug follow-yaw topic: {self.leader_follow_yaw_topic}",
                f"# Follow debug heading topic: {self.leader_estimate_yaw_topic}",
                (
                    f"# Camera topics: {self.pan_topic}, {self.tilt_topic}, "
                    f"{self.actual_pan_topic}, {self.actual_tilt_topic}"
                    if self.include_camera
                    else "# Camera logging disabled"
                ),
            ]
        )

    def destroy_node(self):
        try:
            if not self.log_fp.closed:
                self.log_fp.flush()
                self.log_fp.close()
        finally:
            super().destroy_node()

    def _shutdown_once(self):
        self._write_block(["# Timeout reached, stopping logger"])
        rclpy.shutdown()

    def _stamp(self) -> str:
        return datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

    def _write_block(self, lines: list[str]) -> None:
        block = "\n".join(lines)
        print(block)
        print()
        self.log_fp.write(block + "\n\n")
        self.log_fp.flush()

    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _mark_dirty(self, source: str) -> None:
        self.pending_sources.add(source)
        self.flush_due_ns = self._now_ns() + int(self.flush_delay_s * 1e9)

    @staticmethod
    def _pose_from_pose_stamped(msg: PoseStamped) -> PoseState:
        p = msg.pose.position
        q = msg.pose.orientation
        return PoseState(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            yaw=yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )

    @staticmethod
    def _pose_from_odom(msg: Odometry) -> PoseState:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return PoseState(
            x=float(p.x),
            y=float(p.y),
            z=float(p.z),
            yaw=yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )

    @staticmethod
    def _fmt_num(value: Optional[float], unit: str = "", decimals: int = 2) -> str:
        if value is None or not math.isfinite(value):
            return "na"
        return f"{value:.{decimals}f}{unit}"

    @staticmethod
    def _fmt_signed(value: Optional[float], unit: str = "", decimals: int = 2) -> str:
        if value is None or not math.isfinite(value):
            return "na"
        return f"{value:+.{decimals}f}{unit}"

    def _body_summary_lines(self) -> list[str]:
        if self.body_cmd is None:
            return [
                "STEP: X=na Y=na Yaw=na dt=na",
                "NOW: X=na Y=na Yaw=na Speed=na",
            ]

        if self.prev_body_cmd is None:
            step_dx = step_dy = step_dyaw_deg = None
        else:
            step_dx = self.body_cmd.x - self.prev_body_cmd.x
            step_dy = self.body_cmd.y - self.prev_body_cmd.y
            step_dyaw_deg = math.degrees(wrap_pi(self.body_cmd.yaw - self.prev_body_cmd.yaw))

        if self.actual_uav_pose is None:
            dx = dy = dyaw_deg = None
        else:
            dx = self.body_cmd.x - self.actual_uav_pose.x
            dy = self.body_cmd.y - self.actual_uav_pose.y
            dyaw_deg = math.degrees(wrap_pi(self.body_cmd.yaw - self.actual_uav_pose.yaw))

        dt_ms = self._fmt_num(self.last_body_dt_ms, "ms", decimals=1)
        return [
            (
                f"STEP: X={self._fmt_signed(step_dx, 'm')} "
                f"Y={self._fmt_signed(step_dy, 'm')} "
                f"Yaw={self._fmt_signed(step_dyaw_deg, 'deg')} "
                f"dt={dt_ms}"
            ),
            (
                f"NOW: X={self._fmt_signed(dx, 'm')} "
                f"Y={self._fmt_signed(dy, 'm')} "
                f"Yaw={self._fmt_signed(dyaw_deg, 'deg')} "
                f"Speed={self._fmt_num(self.actual_uav_speed_mps, 'm/s')}"
            ),
        ]

    def _camera_delta_summary(self, name: str, cmd_deg: Optional[float], actual_deg: Optional[float]) -> str:
        delta = None
        if cmd_deg is not None and actual_deg is not None:
            delta = cmd_deg - actual_deg
        return f"{name}: {self._fmt_signed(delta, ' deg')}"

    def _direction_label(self, leader_pose: Optional[PoseState], heading_yaw: Optional[float]) -> str:
        if leader_pose is None or self.actual_uav_pose is None or heading_yaw is None:
            return "na"

        away_from_uav_yaw = math.atan2(
            leader_pose.y - self.actual_uav_pose.y,
            leader_pose.x - self.actual_uav_pose.x,
        )
        delta = wrap_pi(heading_yaw - away_from_uav_yaw)
        abs_delta = abs(delta)
        straight_thresh = math.radians(45.0)
        reverse_thresh = math.radians(135.0)

        if abs_delta <= straight_thresh:
            return "Straight"
        if abs_delta >= reverse_thresh:
            return "Reversing"
        if delta > 0.0:
            return "Turning Left"
        return "Turning Right"

    def _resolved_leader_estimate_yaw(self) -> tuple[Optional[float], str]:
        if self.leader_follow_yaw_dbg_rad is not None:
            return self.leader_follow_yaw_dbg_rad, "follow_debug_follow_yaw"
        if self.leader_estimate_yaw_dbg_rad is not None:
            return self.leader_estimate_yaw_dbg_rad, "follow_debug"
        if self.leader_estimate_pose is not None:
            return self.leader_estimate_pose.yaw, "estimate_pose"
        return None, "na"

    def _resolved_leader_actual_heading_yaw(self) -> tuple[Optional[float], str]:
        if self.leader_actual_pose is not None:
            return self.leader_actual_pose.yaw, "amcl_pose"
        return None, "na"

    def _heading_summary(self) -> list[str]:
        actual_yaw, actual_src = self._resolved_leader_actual_heading_yaw()
        est_yaw, est_src = self._resolved_leader_estimate_yaw()
        actual_dir = self._direction_label(
            self.leader_actual_pose,
            actual_yaw,
        )
        est_dir = self._direction_label(
            self.leader_estimate_pose,
            est_yaw,
        )
        return [
            f"ACTUAL DIR [{actual_src}]: {actual_dir}",
            f"EST. DIR [{est_src}]: {est_dir}",
        ]

    def _emit_snapshot(self) -> None:
        sources = ",".join(sorted(self.pending_sources)) if self.pending_sources else "unknown"
        lines = [
            f"[{self._stamp()}] UAV Command Snapshot ({sources})",
        ]
        for body_line in self._body_summary_lines():
            lines.append(f"  {body_line}")
        if self.include_camera:
            lines.append(f"  {self._camera_delta_summary('PAN', self.cmd_pan_deg, self.actual_pan_deg)}")
            lines.append(f"  {self._camera_delta_summary('TILT', self.cmd_tilt_deg, self.actual_tilt_deg)}")
        for heading_line in self._heading_summary():
            lines.append(f"  {heading_line}")
        self._write_block(lines)
        self.pending_sources.clear()
        self.flush_due_ns = None

    def _maybe_flush_snapshot(self) -> None:
        if not self.pending_sources or self.flush_due_ns is None:
            return
        if self._now_ns() < self.flush_due_ns:
            return
        self._emit_snapshot()

    def _on_body_cmd(self, msg: Joy) -> None:
        axes = list(msg.axes)
        if len(axes) < 4:
            self._write_block([f"[{self._stamp()}] BODY_CMD malformed axes={axes}"])
            return

        now = self.get_clock().now()
        self.last_body_dt_ms = None
        if self.last_body_time is not None:
            self.last_body_dt_ms = (now - self.last_body_time).nanoseconds / 1e6
        self.last_body_time = now

        new_cmd = PoseState(
            x=float(axes[0]),
            y=float(axes[1]),
            z=float(axes[2]),
            yaw=float(axes[3]),
        )
        self.prev_body_cmd = self.body_cmd
        self.body_cmd = new_cmd
        self._mark_dirty("body")

    def _on_actual_uav_pose(self, msg: PoseStamped) -> None:
        pose = self._pose_from_pose_stamped(msg)
        stamp_s = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        if stamp_s <= 0.0:
            stamp_s = self._now_ns() * 1e-9
        if self.actual_uav_prev_pose is not None and self.actual_uav_prev_stamp_s is not None:
            dt = stamp_s - self.actual_uav_prev_stamp_s
            if math.isfinite(dt) and dt > 1e-6:
                dx = pose.x - self.actual_uav_prev_pose.x
                dy = pose.y - self.actual_uav_prev_pose.y
                self.actual_uav_speed_mps = math.hypot(dx, dy) / dt
        self.actual_uav_prev_pose = pose
        self.actual_uav_prev_stamp_s = stamp_s
        self.actual_uav_pose = pose

    def _on_leader_estimate_pose(self, msg: PoseStamped) -> None:
        self.leader_estimate_pose = self._pose_from_pose_stamped(msg)

    def _on_leader_actual_pose(self, msg: Odometry) -> None:
        self.leader_actual_pose = self._pose_from_odom(msg)

    def _on_leader_follow_yaw_dbg(self, msg: Float32) -> None:
        self.leader_follow_yaw_dbg_rad = float(msg.data)

    def _on_leader_estimate_yaw_dbg(self, msg: Float32) -> None:
        self.leader_estimate_yaw_dbg_rad = float(msg.data)

    def _on_pan_cmd(self, msg: Float64) -> None:
        self.cmd_pan_deg = float(msg.data)

    def _on_tilt_cmd(self, msg: Float64) -> None:
        self.cmd_tilt_deg = float(msg.data)

    def _on_actual_pan(self, msg: Float32) -> None:
        self.actual_pan_deg = float(msg.data)

    def _on_actual_tilt(self, msg: Float32) -> None:
        self.actual_tilt_deg = float(msg.data)


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Print and save coalesced UAV command snapshots with movement deltas."
    )
    parser.add_argument("--uav-name", default="dji0", help="UAV namespace, default: dji0")
    parser.add_argument(
        "--log-file",
        default="",
        help="Optional explicit log file path. Defaults to debug_logs/uav_commands/<timestamp>.log",
    )
    parser.add_argument(
        "--include-camera",
        action="store_true",
        help="Also log pan/tilt command deltas using /<uav>/update_* and /<uav>/follow/actual/* topics",
    )
    parser.add_argument(
        "--leader-estimate-topic",
        default="/coord/leader_estimate",
        help="Leader estimate topic used for estimated heading direction. Default: /coord/leader_estimate",
    )
    parser.add_argument(
        "--leader-actual-topic",
        default="/a201_0000/amcl_pose_odom",
        help="Actual UGV odom topic used for actual heading direction. Default: /a201_0000/amcl_pose_odom",
    )
    parser.add_argument(
        "--timeout-s",
        type=float,
        default=0.0,
        help="Optional auto-stop timeout in seconds",
    )
    return parser.parse_args(argv)


def main():
    cli_args = remove_ros_args(args=sys.argv)[1:]
    args = _parse_args(cli_args)
    log_path = Path(args.log_file).expanduser() if args.log_file else _default_log_path(args.uav_name)

    rclpy.init(args=sys.argv)
    node = UavCommandLogger(
        uav_name=args.uav_name,
        log_path=log_path,
        include_camera=bool(args.include_camera),
        timeout_s=args.timeout_s if args.timeout_s > 0.0 else None,
        leader_estimate_topic=str(args.leader_estimate_topic),
        leader_actual_topic=str(args.leader_actual_topic),
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        node.destroy_node()


if __name__ == "__main__":
    main()
