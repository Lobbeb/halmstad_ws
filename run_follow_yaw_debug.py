#!/usr/bin/env python3
import argparse
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from std_msgs.msg import Float32, String


@dataclass
class DebugState:
    uav_x: Optional[float] = None
    uav_y: Optional[float] = None
    uav_pose_yaw: Optional[float] = None
    follow_actual_yaw: Optional[float] = None
    ugv_x: Optional[float] = None
    ugv_y: Optional[float] = None
    camera_actual_yaw: Optional[float] = None
    camera_error_yaw: Optional[float] = None
    yaw_error: Optional[float] = None
    yaw_error_raw: Optional[float] = None
    yaw_wrap_correction: Optional[float] = None
    yaw_wrap_active: Optional[float] = None
    yaw_step_limit: Optional[float] = None
    yaw_cmd_delta: Optional[float] = None
    yaw_actual_unwrapped: Optional[float] = None
    yaw_target_unwrapped: Optional[float] = None
    yaw_mode: Optional[str] = None
    last_render_uav_x: Optional[float] = None
    last_render_uav_y: Optional[float] = None
    last_render_ugv_x: Optional[float] = None
    last_render_ugv_y: Optional[float] = None
    last_render_actual_yaw: Optional[float] = None
    last_render_time_s: Optional[float] = None


class FollowYawDebugWatch(Node):
    def __init__(self, uav_name: str, ugv_odom_topic: str, rate_hz: float):
        super().__init__("follow_yaw_debug_watch")
        self.uav_name = uav_name
        self.ugv_odom_topic = ugv_odom_topic
        self.state = DebugState()

        self.create_subscription(
            PoseStamped,
            f"/{self.uav_name}/pose",
            self._on_uav_pose,
            10,
        )
        self.create_subscription(
            Odometry,
            self.ugv_odom_topic,
            self._on_ugv_odom,
            10,
        )
        self._float_topic("follow/actual/yaw_rad", "follow_actual_yaw")
        self._float_topic("camera/actual/world_yaw_rad", "camera_actual_yaw")
        self._float_topic("camera/error/world_yaw_rad", "camera_error_yaw")
        self._float_topic("follow/error/yaw_rad", "yaw_error")
        self._float_topic("follow/debug/yaw_error_raw_rad", "yaw_error_raw")
        self._float_topic("follow/debug/yaw_wrap_correction_rad", "yaw_wrap_correction")
        self._float_topic("follow/debug/yaw_wrap_active", "yaw_wrap_active")
        self._float_topic("follow/debug/yaw_step_limit_rad", "yaw_step_limit")
        self._float_topic("follow/debug/yaw_cmd_delta_rad", "yaw_cmd_delta")
        self._float_topic("follow/debug/yaw_actual_unwrapped_rad", "yaw_actual_unwrapped")
        self._float_topic("follow/debug/yaw_target_unwrapped_rad", "yaw_target_unwrapped")
        self.create_subscription(
            String,
            f"/{self.uav_name}/follow/debug/yaw_mode",
            self._on_yaw_mode,
            10,
        )

        self.create_timer(1.0 / max(rate_hz, 1.0), self._render)

    def _float_topic(self, suffix: str, field_name: str) -> None:
        self.create_subscription(
            Float32,
            f"/{self.uav_name}/{suffix}",
            lambda msg, name=field_name: setattr(self.state, name, float(msg.data)),
            10,
        )

    def _on_yaw_mode(self, msg: String) -> None:
        self.state.yaw_mode = msg.data

    def _on_uav_pose(self, msg: PoseStamped) -> None:
        self.state.uav_x = float(msg.pose.position.x)
        self.state.uav_y = float(msg.pose.position.y)
        q = msg.pose.orientation
        siny_cosp = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy_cosp = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        self.state.uav_pose_yaw = math.atan2(siny_cosp, cosy_cosp)

    def _on_ugv_odom(self, msg: Odometry) -> None:
        self.state.ugv_x = float(msg.pose.pose.position.x)
        self.state.ugv_y = float(msg.pose.pose.position.y)

    @staticmethod
    def _fmt(value: Optional[float], width: int = 8) -> str:
        if value is None:
            return " " * (width - 3) + "n/a"
        return f"{value:{width}.3f}"

    @staticmethod
    def _fmt_precise(value: Optional[float], width: int = 9) -> str:
        if value is None:
            return " " * (width - 3) + "n/a"
        return f"{value:{width}.4f}"

    @staticmethod
    def _fmt_flag(value: Optional[float]) -> str:
        if value is None:
            return "n/a"
        return "1" if value >= 0.5 else "0"

    @staticmethod
    def _xy_delta(x: Optional[float], y: Optional[float], last_x: Optional[float], last_y: Optional[float]) -> Optional[float]:
        if x is None or y is None or last_x is None or last_y is None:
            return None
        return math.hypot(x - last_x, y - last_y)

    def _render(self) -> None:
        s = self.state
        now = self.get_clock().now().nanoseconds * 1e-9
        uav_xy_delta = self._xy_delta(s.uav_x, s.uav_y, s.last_render_uav_x, s.last_render_uav_y)
        ugv_xy_delta = self._xy_delta(s.ugv_x, s.ugv_y, s.last_render_ugv_x, s.last_render_ugv_y)
        actual_yaw_delta = None
        actual_yaw_rate = None
        if s.uav_pose_yaw is not None and s.last_render_actual_yaw is not None:
            actual_yaw_delta = math.atan2(
                math.sin(s.uav_pose_yaw - s.last_render_actual_yaw),
                math.cos(s.uav_pose_yaw - s.last_render_actual_yaw),
            )
        if actual_yaw_delta is not None and s.last_render_time_s is not None:
            dt = now - s.last_render_time_s
            if dt > 1e-6:
                actual_yaw_rate = actual_yaw_delta / dt
                if uav_xy_delta is not None:
                    uav_xy_delta = uav_xy_delta / dt
                if ugv_xy_delta is not None:
                    ugv_xy_delta = ugv_xy_delta / dt

        rel_range = None
        rel_forward = None
        rel_lateral = None
        camera_bearing = None
        if None not in (s.uav_x, s.uav_y, s.ugv_x, s.ugv_y):
            dx = float(s.ugv_x - s.uav_x)
            dy = float(s.ugv_y - s.uav_y)
            rel_range = math.hypot(dx, dy)
            if s.uav_pose_yaw is not None:
                c = math.cos(s.uav_pose_yaw)
                sb = math.sin(s.uav_pose_yaw)
                rel_forward = c * dx + sb * dy
                rel_lateral = -sb * dx + c * dy
            if s.camera_actual_yaw is not None:
                camera_bearing = math.atan2(
                    math.sin(math.atan2(dy, dx) - s.camera_actual_yaw),
                    math.cos(math.atan2(dy, dx) - s.camera_actual_yaw),
                )
        print("=" * 72)
        print(f"Follow yaw debug: {self.uav_name}   t={now:10.3f}s")
        print("")
        print(
            "xy motion   "
            f"uav_rate={self._fmt_precise(uav_xy_delta)}  "
            f"ugv_rate={self._fmt_precise(ugv_xy_delta)}"
        )
        print(
            "target geom "
            f"range={self._fmt_precise(rel_range)}  "
            f"fwd={self._fmt_precise(rel_forward)}  "
            f"lat={self._fmt_precise(rel_lateral)}  "
            f"cam_bear={self._fmt_precise(camera_bearing)}"
        )
        print(
            "body yaw    "
            f"actual={self._fmt(s.uav_pose_yaw)}  "
            f"follow_dbg={self._fmt(s.follow_actual_yaw)}  "
            f"camera={self._fmt(s.camera_actual_yaw)}  "
            f"cam_err={self._fmt(s.camera_error_yaw)}"
        )
        print(
            "body motion "
            f"actual_d={self._fmt_precise(actual_yaw_delta)}  "
            f"actual_rate={self._fmt_precise(actual_yaw_rate)}"
        )
        print(
            "yaw error   "
            f"wrapped={self._fmt(s.yaw_error)}  "
            f"raw={self._fmt(s.yaw_error_raw)}  "
            f"wrap_corr={self._fmt(s.yaw_wrap_correction)}  "
            f"wrap={self._fmt_flag(s.yaw_wrap_active)}"
        )
        print(
            "yaw cmd     "
            f"delta={self._fmt_precise(s.yaw_cmd_delta)}  "
            f"step_lim={self._fmt_precise(s.yaw_step_limit)}  "
            f"mode={s.yaw_mode or 'n/a'}"
        )
        print(
            "yaw unwrap  "
            f"actual={self._fmt(s.yaw_actual_unwrapped)}  "
            f"target={self._fmt(s.yaw_target_unwrapped)}"
        )
        print("")
        print("wrap event: wrap=1 and wrap_corr near +/-6.283")
        print("sign check: yaw_cmd_delta should match wrapped yaw error, not raw yaw error")
        print("cam_bear: target bearing in camera frame, positive=left, negative=right")
        print("")
        if s.uav_x is not None and s.uav_y is not None:
            s.last_render_uav_x = s.uav_x
            s.last_render_uav_y = s.uav_y
        if s.ugv_x is not None and s.ugv_y is not None:
            s.last_render_ugv_x = s.ugv_x
            s.last_render_ugv_y = s.ugv_y
        if s.uav_pose_yaw is not None:
            s.last_render_actual_yaw = s.uav_pose_yaw
            s.last_render_time_s = now


def main() -> None:
    parser = argparse.ArgumentParser(description="Compact terminal watcher for follow yaw wrap debugging")
    parser.add_argument("--uav", default="dji0", help="UAV name prefix, default: dji0")
    parser.add_argument(
        "--ugv-odom",
        default="/a201_0000/platform/odom",
        help="UGV odom topic, default: /a201_0000/platform/odom",
    )
    parser.add_argument("--rate", type=float, default=5.0, help="Refresh rate in Hz, default: 5")
    args = parser.parse_args()

    rclpy.init()
    node = FollowYawDebugWatch(args.uav, args.ugv_odom, args.rate)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
