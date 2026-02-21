#!/usr/bin/env python3
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from tf_transformations import quaternion_from_euler, euler_from_quaternion


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


class LeaderEstimator(Node):
    """
    Stage 2.2: Estimator interface node placeholder (future vision-based estimator), ground-truth based).
    - Subscribes to UGV odom (default: /a201_0000/platform/odom)
    - Publishes leader estimate as PoseStamped on /coord/leader_estimate
    - Throttles publish rate to est_hz (default 5 Hz)
    - Health logic: if odom is stale -> stop publishing and emit status/event
    """

    def __init__(self):
        super().__init__("leader_estimator")

        # Parameters
        self.declare_parameter("ugv_odom_topic", "/a201_0000/platform/odom")
        self.declare_parameter("out_topic", "/coord/leader_estimate")
        self.declare_parameter("status_topic", "/coord/leader_estimate_status")
        self.declare_parameter("publish_status", True)

        self.declare_parameter("est_hz", 5.0)
        self.declare_parameter("pose_timeout_s", 2.0)

        self.declare_parameter("event_topic", "/coord/events")
        self.declare_parameter("publish_events", True)

        # Read params
        self.ugv_odom_topic = str(self.get_parameter("ugv_odom_topic").value)
        self.out_topic = str(self.get_parameter("out_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)
        self.publish_status = bool(self.get_parameter("publish_status").value)

        self.est_hz = float(self.get_parameter("est_hz").value)
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)

        self.event_topic = str(self.get_parameter("event_topic").value)
        self.publish_events = bool(self.get_parameter("publish_events").value)

        if self.est_hz <= 0.0:
            raise ValueError("est_hz must be > 0")

        # State
        self.have_ugv = False
        self.ugv_pose = Pose2D(0.0, 0.0, 0.0)
        self.last_ugv_stamp: Optional[Time] = None

        self.last_pub_time: Optional[Time] = None
        # self.stale_mode = False  # latch so we don't spam status/events
        self.last_status: Optional[str] = None

        # I/O
        self.sub = self.create_subscription(Odometry, self.ugv_odom_topic, self.on_odom, 10)
        self.pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.events_pub = self.create_publisher(String, self.event_topic, 10)

        self.timer = self.create_timer(1.0 / self.est_hz, self.on_tick)

        self.get_logger().info(
            f"[leader_estimator] Started: odom={self.ugv_odom_topic} -> out={self.out_topic}, "
            f"est_hz={self.est_hz}Hz, pose_timeout_s={self.pose_timeout_s}, "
            f"status_topic={self.status_topic}, event_topic={self.event_topic}"
        )
        self.emit_event("ESTIMATOR_NODE_START")
        
    def emit_event(self, s: str) -> None:
        if not self.publish_events:
            return
        msg = String()
        msg.data = s
        self.events_pub.publish(msg)

    def publish_status_msg(self, s: str) -> None:
        if not self.publish_status:
            return
        msg = String()
        msg.data = s
        self.status_pub.publish(msg)

    def on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        (_, _, yaw) = euler_from_quaternion([q.x, q.y, q.z, q.w])

        self.ugv_pose = Pose2D(float(p.x), float(p.y), float(yaw))
        self.have_ugv = True

        try:
            self.last_ugv_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_ugv_stamp = self.get_clock().now()

    def ugv_fresh(self, now: Time) -> bool:
        if (not self.have_ugv) or (self.last_ugv_stamp is None):
            return False
        age = (now - self.last_ugv_stamp).nanoseconds * 1e-9
        return age <= self.pose_timeout_s

    def publish_estimate(self, now: Time) -> None:
        ps = PoseStamped()
        ps.header.stamp = now.to_msg()
        ps.header.frame_id = "map"

        ps.pose.position.x = self.ugv_pose.x
        ps.pose.position.y = self.ugv_pose.y
        ps.pose.position.z = 0.0

        quat = quaternion_from_euler(0.0, 0.0, self.ugv_pose.yaw)
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])

        self.pub.publish(ps)

    def on_tick(self) -> None:
        now = self.get_clock().now()

        
        fresh = self.ugv_fresh(now)

        age_s = float("inf")
        if self.have_ugv and self.last_ugv_stamp is not None:
            age_s = (now - self.last_ugv_stamp).nanoseconds * 1e-9

        status = f"OK age={age_s:.2f}s" if fresh else f"STALE age={age_s:.2f}s"
        self.publish_status_msg(status)


        # Only emit events on transitions (avoid spam)
        state = "OK" if fresh else "STALE"
        if state != self.last_status:
            if state == "STALE":
                self.emit_event("ESTIMATE_STALE")
            else:
                self.emit_event("ESTIMATE_OK")
            self.last_status = state

        # Only publish estimate when fresh
        if not fresh:
            return

        self.publish_estimate(now)
        self.last_pub_time = now



def main(args=None):
    rclpy.init(args=args)
    node = LeaderEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.emit_event("ESTIMATOR_NODE_SHUTDOWN")
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
