#!/usr/bin/env python3
import argparse
import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from tf2_ros import Buffer, TransformException, TransformListener


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float
    frame_id: str


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


class PoseSourceDebugWatch(Node):
    def __init__(self, ugv_namespace: str, target_frame: str, rate_hz: float):
        super().__init__("pose_source_debug_watch", namespace=ugv_namespace)

        self.ugv_namespace = ugv_namespace.strip("/")
        self.target_frame = target_frame
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self.raw_odom: Optional[Pose2D] = None
        self.filtered_odom: Optional[Pose2D] = None
        self.amcl_pose: Optional[Pose2D] = None

        self.create_subscription(Odometry, "platform/odom", self._on_raw_odom, 10)
        self.create_subscription(Odometry, "platform/odom/filtered", self._on_filtered_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, "amcl_pose", self._on_amcl_pose, 10)
        self.create_timer(1.0 / max(rate_hz, 1.0), self._render)

    def _on_raw_odom(self, msg: Odometry) -> None:
        self.raw_odom = self._pose_from_odom(msg)

    def _on_filtered_odom(self, msg: Odometry) -> None:
        self.filtered_odom = self._pose_from_odom(msg)

    def _on_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.amcl_pose = Pose2D(
            x=float(p.x),
            y=float(p.y),
            yaw=yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
            frame_id=msg.header.frame_id or self.target_frame,
        )

    @staticmethod
    def _pose_from_odom(msg: Odometry) -> Pose2D:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return Pose2D(
            x=float(p.x),
            y=float(p.y),
            yaw=yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
            frame_id=msg.header.frame_id or "odom",
        )

    def _transform_pose(self, pose: Optional[Pose2D], target_frame: str) -> Optional[Pose2D]:
        if pose is None:
            return None
        if pose.frame_id == target_frame:
            return pose
        try:
            tf = self.tf_buffer.lookup_transform(
                target_frame,
                pose.frame_id,
                Time(),
            )
        except TransformException:
            return None

        tx = float(tf.transform.translation.x)
        ty = float(tf.transform.translation.y)
        q = tf.transform.rotation
        tf_yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)
        x = tx + c * pose.x - s * pose.y
        y = ty + s * pose.x + c * pose.y
        yaw = wrap_pi(tf_yaw + pose.yaw)
        return Pose2D(x=x, y=y, yaw=yaw, frame_id=target_frame)

    def _lookup_transform_pose(self, target_frame: str, source_frame: str) -> Optional[Pose2D]:
        if not source_frame or source_frame == target_frame:
            return Pose2D(0.0, 0.0, 0.0, target_frame)
        try:
            tf = self.tf_buffer.lookup_transform(target_frame, source_frame, Time())
        except TransformException:
            return None
        q = tf.transform.rotation
        return Pose2D(
            x=float(tf.transform.translation.x),
            y=float(tf.transform.translation.y),
            yaw=yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
            frame_id=target_frame,
        )

    @staticmethod
    def _fmt(value: Optional[float], width: int = 9) -> str:
        if value is None:
            return " " * (width - 3) + "n/a"
        return f"{value:{width}.4f}"

    @staticmethod
    def _fmt_pose(label: str, pose: Optional[Pose2D]) -> str:
        if pose is None:
            return f"{label:<11} n/a"
        return (
            f"{label:<11} frame={pose.frame_id:<6} "
            f"x={pose.x:8.3f} y={pose.y:8.3f} yaw={pose.yaw:8.3f}"
        )

    @staticmethod
    def _pose_error(a: Optional[Pose2D], b: Optional[Pose2D]) -> tuple[Optional[float], Optional[float]]:
        if a is None or b is None:
            return None, None
        return math.hypot(a.x - b.x, a.y - b.y), wrap_pi(a.yaw - b.yaw)

    def _render(self) -> None:
        raw_in_target = self._transform_pose(self.raw_odom, self.target_frame)
        filtered_in_target = self._transform_pose(self.filtered_odom, self.target_frame)
        amcl_in_target = self._transform_pose(self.amcl_pose, self.target_frame)

        raw_err_xy, raw_err_yaw = self._pose_error(raw_in_target, amcl_in_target)
        filtered_err_xy, filtered_err_yaw = self._pose_error(filtered_in_target, amcl_in_target)

        source_frame = None
        if self.raw_odom is not None:
            source_frame = self.raw_odom.frame_id
        elif self.filtered_odom is not None:
            source_frame = self.filtered_odom.frame_id
        map_from_odom = self._lookup_transform_pose(self.target_frame, source_frame) if source_frame else None

        now = self.get_clock().now().nanoseconds * 1e-9
        print("=" * 72)
        print(
            f"Pose source debug: {self.ugv_namespace or '/'}   "
            f"t={now:10.3f}s   target_frame={self.target_frame}"
        )
        print("")
        print(self._fmt_pose("raw odom", self.raw_odom))
        print(self._fmt_pose("filtered", self.filtered_odom))
        print(self._fmt_pose("amcl", self.amcl_pose))
        print("")
        print(self._fmt_pose("raw->target", raw_in_target))
        print(self._fmt_pose("filtered->t", filtered_in_target))
        print(self._fmt_pose("amcl->target", amcl_in_target))
        print(self._fmt_pose("tf tgt<-src", map_from_odom))
        print("")
        print(
            "raw vs amcl "
            f"xy={self._fmt(raw_err_xy)}  yaw={self._fmt(raw_err_yaw)}"
        )
        print(
            "filt vs amcl"
            f" xy={self._fmt(filtered_err_xy)}  yaw={self._fmt(filtered_err_yaw)}"
        )
        print("")
        print("If raw/filtered drift away from amcl during turns, the follower is likely chasing the wrong frame.")
        print("")


def main() -> None:
    parser = argparse.ArgumentParser(description="Compare UGV pose sources in a common frame")
    parser.add_argument("--ugv", default="a201_0000", help="UGV namespace, default: a201_0000")
    parser.add_argument("--target-frame", default="map", help="Comparison frame, default: map")
    parser.add_argument("--rate", type=float, default=5.0, help="Refresh rate in Hz, default: 5")
    args = parser.parse_args()

    rclpy.init()
    node = PoseSourceDebugWatch(args.ugv, args.target_frame, args.rate)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
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
