#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64
from tf_transformations import quaternion_from_euler

from lrs_halmstad.follow_math import (
    Pose2D,
    camera_xy_from_uav_pose,
    compute_camera_tilt_deg,
    wrap_pi,
    yaw_from_quat,
)


class CameraTracker(Node):
    def __init__(self):
        super().__init__("camera_tracker")

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("leader_input_type", "odom")
        self.declare_parameter("leader_odom_topic", "/a201_0000/platform/odom/filtered")
        self.declare_parameter("leader_pose_topic", "/coord/leader_estimate")
        self.declare_parameter("uav_camera_mode", "integrated_joint")
        self.declare_parameter("camera_mount_pitch_deg", 45.0)
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("pose_timeout_s", 3.0)
        self.declare_parameter("camera_x_offset_m", 0.0)
        self.declare_parameter("camera_y_offset_m", 0.0)
        self.declare_parameter("camera_z_offset_m", 0.27)
        self.declare_parameter("camera_look_target_z_m", 0.0)
        self.declare_parameter("camera_yaw_offset_deg", 0.0)
        self.declare_parameter("camera_pan_sign", 1.0)
        self.declare_parameter("default_pan_deg", 0.0)
        self.declare_parameter("pan_enable", False)
        self.declare_parameter("default_tilt_deg", 0.0)
        self.declare_parameter("tilt_enable", True)

        self.uav_name = str(self.get_parameter("uav_name").value)
        self.leader_input_type = str(self.get_parameter("leader_input_type").value).strip().lower()
        self.leader_odom_topic = str(self.get_parameter("leader_odom_topic").value)
        self.leader_pose_topic = str(self.get_parameter("leader_pose_topic").value)
        self.uav_camera_mode = str(self.get_parameter("uav_camera_mode").value).strip().lower()
        self.camera_mount_pitch_deg = float(self.get_parameter("camera_mount_pitch_deg").value)
        self.tick_hz = max(1.0, float(self.get_parameter("tick_hz").value))
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.camera_x_offset_m = float(self.get_parameter("camera_x_offset_m").value)
        self.camera_y_offset_m = float(self.get_parameter("camera_y_offset_m").value)
        self.camera_z_offset_m = float(self.get_parameter("camera_z_offset_m").value)
        self.camera_look_target_z_m = float(self.get_parameter("camera_look_target_z_m").value)
        self.camera_yaw_offset_deg = float(self.get_parameter("camera_yaw_offset_deg").value)
        self.camera_pan_sign = float(self.get_parameter("camera_pan_sign").value)
        self.default_pan_deg = float(self.get_parameter("default_pan_deg").value)
        self.pan_enable = bool(self.get_parameter("pan_enable").value)
        self.default_tilt_deg = float(self.get_parameter("default_tilt_deg").value)
        self.tilt_enable = bool(self.get_parameter("tilt_enable").value)

        if self.leader_input_type == "estimate":
            self.leader_input_type = "pose"
        if self.leader_input_type not in ("odom", "pose"):
            raise ValueError("leader_input_type must be 'odom', 'pose', or 'estimate'")
        if self.uav_camera_mode == "integrated":
            self.uav_camera_mode = "integrated_joint"
        if self.uav_camera_mode == "detached":
            self.uav_camera_mode = "detached_model"

        self.have_leader = False
        self.leader_pose = Pose2D(0.0, 0.0, 0.0)
        self.leader_z = 0.0
        self.last_leader_stamp: Optional[Time] = None

        self.have_uav = False
        self.uav_pose = Pose2D(0.0, 0.0, 0.0)
        self.uav_z = 0.0

        if self.leader_input_type == "odom":
            self.leader_sub = self.create_subscription(
                Odometry,
                self.leader_odom_topic,
                self.on_leader_odom,
                10,
            )
        else:
            self.leader_sub = self.create_subscription(
                PoseStamped,
                self.leader_pose_topic,
                self.on_leader_pose,
                10,
            )

        self.uav_sub = self.create_subscription(
            PoseStamped,
            f"/{self.uav_name}/pose",
            self.on_uav_pose,
            10,
        )
        self.target_camera_pose_pub = self.create_publisher(
            PoseStamped,
            f"/{self.uav_name}/camera/target/center_pose",
            10,
        )
        self.target_look_at_point_pub = self.create_publisher(
            PointStamped,
            f"/{self.uav_name}/camera/target/look_at_point",
            10,
        )
        self.target_camera_world_yaw_pub = self.create_publisher(
            Float32,
            f"/{self.uav_name}/camera/target/world_yaw_rad",
            10,
        )
        self.tilt_pub = self.create_publisher(Float64, f"/{self.uav_name}/update_tilt", 10)
        self.pan_pub = self.create_publisher(Float64, f"/{self.uav_name}/update_pan", 10)
        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)
        self.get_logger().info(
            f"[camera_tracker] Started: uav={self.uav_name}, leader_input={self.leader_input_type}, "
            f"pan_enable={self.pan_enable}, default_pan_deg={self.default_pan_deg}, "
            f"tilt_enable={self.tilt_enable}, default_tilt_deg={self.default_tilt_deg}, "
            f"camera_mode={self.uav_camera_mode}, camera_mount_pitch_deg={self.camera_mount_pitch_deg}"
        )

    def on_leader_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.leader_pose = Pose2D(float(p.x), float(p.y), yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)))
        self.leader_z = float(p.z)
        self.have_leader = True
        try:
            self.last_leader_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_leader_stamp = self.get_clock().now()

    def on_leader_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.leader_pose = Pose2D(float(p.x), float(p.y), yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)))
        self.leader_z = float(p.z)
        self.have_leader = True
        try:
            self.last_leader_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_leader_stamp = self.get_clock().now()

    def on_uav_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_pose = Pose2D(float(p.x), float(p.y), yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)))
        self.uav_z = float(p.z)
        self.have_uav = True

    def leader_pose_is_fresh(self, now: Time) -> bool:
        if not self.have_leader or self.last_leader_stamp is None:
            return False
        age_s = (now - self.last_leader_stamp).nanoseconds * 1e-9
        return age_s <= self.pose_timeout_s

    def _compute_tilt_deg(self) -> float:
        return compute_camera_tilt_deg(
            self.uav_pose.x,
            self.uav_pose.y,
            self.uav_z,
            self.uav_pose.yaw,
            self.leader_pose.x,
            self.leader_pose.y,
            self.leader_z,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
            self.camera_z_offset_m,
            self.camera_look_target_z_m,
        )

    def _command_tilt_deg(self) -> float:
        if not self.tilt_enable:
            return self.default_tilt_deg
        target_tilt_deg = self._compute_tilt_deg()
        if self.uav_camera_mode == "integrated_joint":
            # The attached gimbal pitch command is relative to the fixed camera
            # mount angle baked into the spawned UAV model.
            return target_tilt_deg + self.camera_mount_pitch_deg
        return target_tilt_deg

    def _compute_pan_deg(self) -> float:
        if not self.pan_enable:
            return self.default_pan_deg
        camera_x, camera_y = camera_xy_from_uav_pose(
            self.uav_pose.x,
            self.uav_pose.y,
            self.uav_pose.yaw,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )
        target_yaw = math.atan2(self.leader_pose.y - camera_y, self.leader_pose.x - camera_x)
        # Pan command is the raw residual from current UAV yaw to target yaw.
        # The simulator applies camera_pan_sign and camera_yaw_offset_deg when
        # converting this command into the rendered camera pose.
        pan_rad = wrap_pi(target_yaw - self.uav_pose.yaw)
        return math.degrees(pan_rad)

    def _publish_camera_debug(self) -> None:
        camera_x, camera_y = camera_xy_from_uav_pose(
            self.uav_pose.x,
            self.uav_pose.y,
            self.uav_pose.yaw,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )
        target_yaw = math.atan2(self.leader_pose.y - camera_y, self.leader_pose.x - camera_x)
        target_tilt = self._compute_tilt_deg()
        quat = quaternion_from_euler(0.0, -math.radians(target_tilt), target_yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = float(camera_x)
        pose_msg.pose.position.y = float(camera_y)
        pose_msg.pose.position.z = float(self.uav_z - self.camera_z_offset_m)
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.target_camera_pose_pub.publish(pose_msg)

        point_msg = PointStamped()
        point_msg.header.stamp = pose_msg.header.stamp
        point_msg.header.frame_id = "odom"
        point_msg.point.x = float(self.leader_pose.x)
        point_msg.point.y = float(self.leader_pose.y)
        point_msg.point.z = float(self.leader_z)
        self.target_look_at_point_pub.publish(point_msg)

        yaw_msg = Float32()
        yaw_msg.data = float(target_yaw)
        self.target_camera_world_yaw_pub.publish(yaw_msg)

    def on_tick(self) -> None:
        now = self.get_clock().now()
        if not self.have_uav:
            return
        leader_fresh = self.leader_pose_is_fresh(now)
        if leader_fresh:
            self._publish_camera_debug()

        tilt_msg = Float64()
        if self.tilt_enable and leader_fresh:
            tilt_msg.data = float(self._command_tilt_deg())
        else:
            tilt_msg.data = float(self.default_tilt_deg)
        self.tilt_pub.publish(tilt_msg)

        pan_msg = Float64()
        if self.pan_enable and leader_fresh:
            pan_msg.data = float(self._compute_pan_deg())
        else:
            pan_msg.data = float(self.default_pan_deg)
        self.pan_pub.publish(pan_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
