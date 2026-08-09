#!/usr/bin/env python3

from __future__ import annotations

import math
import threading
from dataclasses import dataclass
from typing import Optional

import gz.transport13 as gz_transport
from geometry_msgs.msg import PoseStamped
from gz.msgs10 import pose_v_pb2
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node

from lrs_halmstad.common.world_names import gazebo_world_name


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def _wrap_pi(angle: float) -> float:
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


@dataclass
class PoseState:
    stamp_sec: int
    stamp_nsec: int
    x: float
    y: float
    z: float
    qx: float
    qy: float
    qz: float
    qw: float
    yaw: float
    vx_body: float
    vy_body: float
    vz: float
    wz: float


class GazeboModelPoseBridge(Node):
    def __init__(self) -> None:
        super().__init__("gazebo_model_pose_bridge")

        self.declare_parameter("world", "warehouse")
        self.declare_parameter("model_name", "a201_0000/robot")
        self.declare_parameter("pose_topic", "ground_truth/pose")
        self.declare_parameter("odom_topic", "ground_truth/odom")
        self.declare_parameter("frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")
        self.declare_parameter("publish_hz", 0.0)

        self.world = str(self.get_parameter("world").value).strip()
        self.gz_world = gazebo_world_name(self.world)
        self.model_name = str(self.get_parameter("model_name").value).strip()
        self.pose_topic = str(self.get_parameter("pose_topic").value).strip()
        self.odom_topic = str(self.get_parameter("odom_topic").value).strip()
        self.frame_id = str(self.get_parameter("frame_id").value).strip() or "odom"
        self.child_frame_id = (
            str(self.get_parameter("child_frame_id").value).strip() or "base_link"
        )
        self.publish_hz = max(0.0, float(self.get_parameter("publish_hz").value))
        self._min_publish_period_s = 1.0 / self.publish_hz if self.publish_hz > 0.0 else 0.0

        self.pose_pub = self.create_publisher(PoseStamped, self.pose_topic, 10)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)

        self._lock = threading.Lock()
        self._prev_pose: Optional[PoseState] = None
        self._received_first_pose = False
        self._last_source_topic = ""

        self._gz_node = gz_transport.Node()
        self._subscribed_topics = []
        for topic in (
            f"/world/{self.gz_world}/dynamic_pose/info",
            f"/world/{self.gz_world}/pose/info",
        ):
            try:
                ok = self._gz_node.subscribe(
                    pose_v_pb2.Pose_V,
                    topic,
                    self._make_pose_callback(topic),
                )
            except Exception as exc:
                self.get_logger().warn(
                    f"Failed to subscribe to Gazebo pose topic '{topic}': {exc}"
                )
                ok = False
            if ok:
                self._subscribed_topics.append(topic)

        if not self._subscribed_topics:
            raise RuntimeError(
                f"Could not subscribe to any Gazebo pose topic for world '{self.gz_world}'"
            )

        self.get_logger().info(
            f"[gazebo_model_pose_bridge] Tracking model '{self.model_name}' from {self._subscribed_topics} "
            f"and publishing pose={self.pose_topic} odom={self.odom_topic} frame_id={self.frame_id} "
            f"publish_hz={self.publish_hz:.1f}"
        )

    def _make_pose_callback(self, source_topic: str):
        def _on_pose_v(msg: pose_v_pb2.Pose_V) -> None:
            self._on_pose_v(source_topic, msg)

        return _on_pose_v

    def _on_pose_v(self, source_topic: str, msg: pose_v_pb2.Pose_V) -> None:
        matching_pose = None
        for pose in msg.pose:
            if pose.name == self.model_name:
                matching_pose = pose
                break

        if matching_pose is None:
            return

        stamp_sec = int(msg.header.stamp.sec)
        stamp_nsec = int(msg.header.stamp.nsec)

        x = float(matching_pose.position.x)
        y = float(matching_pose.position.y)
        z = float(matching_pose.position.z)
        qx = float(matching_pose.orientation.x)
        qy = float(matching_pose.orientation.y)
        qz = float(matching_pose.orientation.z)
        qw = float(matching_pose.orientation.w)
        yaw = _yaw_from_quat(qx, qy, qz, qw)

        vx_body = vy_body = vz = wz = 0.0
        with self._lock:
            previous = self._prev_pose
            if previous is not None:
                dt = (
                    float(stamp_sec - previous.stamp_sec)
                    + float(stamp_nsec - previous.stamp_nsec) * 1e-9
                )
                if dt <= 1e-9:
                    return
                if self._min_publish_period_s > 0.0 and dt < self._min_publish_period_s:
                    return
                if dt > 1e-6:
                    vx_world = (x - previous.x) / dt
                    vy_world = (y - previous.y) / dt
                    vz = (z - previous.z) / dt
                    # nav_msgs/Odometry twist is expected to be in the child/body frame.
                    vx_body = math.cos(yaw) * vx_world + math.sin(yaw) * vy_world
                    vy_body = -math.sin(yaw) * vx_world + math.cos(yaw) * vy_world
                    wz = _wrap_pi(yaw - previous.yaw) / dt
            current = PoseState(
                stamp_sec=stamp_sec,
                stamp_nsec=stamp_nsec,
                x=x,
                y=y,
                z=z,
                qx=qx,
                qy=qy,
                qz=qz,
                qw=qw,
                yaw=yaw,
                vx_body=vx_body,
                vy_body=vy_body,
                vz=vz,
                wz=wz,
            )
            self._prev_pose = current
            self._last_source_topic = source_topic

        self._publish_state(current)

        if not self._received_first_pose:
            self._received_first_pose = True
            self.get_logger().info(
                f"[gazebo_model_pose_bridge] First pose for '{self.model_name}' received from {source_topic}: "
                f"x={x:.3f} y={y:.3f} z={z:.3f} yaw={yaw:.3f}"
            )

    def _publish_state(self, state: PoseState) -> None:
        stamp = self.get_clock().now().to_msg()
        if state.stamp_sec != 0 or state.stamp_nsec != 0:
            stamp.sec = int(state.stamp_sec)
            stamp.nanosec = int(state.stamp_nsec)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = stamp
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = state.x
        pose_msg.pose.position.y = state.y
        pose_msg.pose.position.z = state.z
        pose_msg.pose.orientation.x = state.qx
        pose_msg.pose.orientation.y = state.qy
        pose_msg.pose.orientation.z = state.qz
        pose_msg.pose.orientation.w = state.qw
        self.pose_pub.publish(pose_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = self.frame_id
        odom_msg.child_frame_id = self.child_frame_id
        odom_msg.pose.pose = pose_msg.pose
        odom_msg.twist.twist.linear.x = state.vx_body
        odom_msg.twist.twist.linear.y = state.vy_body
        odom_msg.twist.twist.linear.z = state.vz
        odom_msg.twist.twist.angular.z = state.wz
        self.odom_pub.publish(odom_msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GazeboModelPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
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
