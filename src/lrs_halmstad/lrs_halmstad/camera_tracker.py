#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float64, String

from lrs_halmstad.follow_math import (
    Pose2D,
    camera_xy_from_uav_pose,
    coerce_bool,
    compute_camera_tilt_deg,
    compute_leader_look_target,
    wrap_pi,
    yaw_from_quat,
)


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    half_roll = 0.5 * float(roll)
    half_pitch = 0.5 * float(pitch)
    half_yaw = 0.5 * float(yaw)

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    w = cr * cp * cy + sr * sp * sy
    return (x, y, z, w)


def should_prefer_command_pose(
    *,
    have_cmd: bool,
    cmd_stamp_ns: Optional[int],
    have_actual: bool,
    actual_stamp_ns: Optional[int],
    now_ns: int,
    pose_timeout_s: float,
) -> bool:
    if not have_cmd or cmd_stamp_ns is None:
        return False
    if now_ns - cmd_stamp_ns > pose_timeout_s * 1e9:
        return False
    if not have_actual or actual_stamp_ns is None:
        return True
    if now_ns - actual_stamp_ns > pose_timeout_s * 1e9:
        return True
    return cmd_stamp_ns >= actual_stamp_ns


def apply_deadband_command(
    target_value: float,
    previous_value: Optional[float],
    deadband_value: float,
) -> float:
    if previous_value is None or deadband_value <= 0.0:
        return float(target_value)
    if abs(float(target_value) - float(previous_value)) < deadband_value:
        return float(previous_value)
    return float(target_value)


TRACKABLE_ESTIMATOR_STATES = frozenset({"OK"})


def parse_status_field(status_line: str, key: str) -> Optional[str]:
    prefix = f"{key}="
    for token in status_line.split():
        if token.startswith(prefix):
            return token[len(prefix):]
    return None

class CameraTracker(Node):
    def __init__(self):
        super().__init__("camera_tracker")

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("leader_input_type", "odom")
        self.declare_parameter("leader_odom_topic", "/a201_0000/platform/odom/filtered")
        self.declare_parameter("leader_pose_topic", "/coord/leader_estimate")
        self.declare_parameter("leader_status_topic", "/coord/leader_estimate_status")
        self.declare_parameter("uav_camera_mode", "integrated_joint")
        self.declare_parameter("camera_mount_pitch_deg", 45.0)
        self.declare_parameter("uav_pose_cmd_topic", "")
        self.declare_parameter("tick_hz", 10.0)
        self.declare_parameter("pose_timeout_s", 3.0)
        self.declare_parameter("camera_x_offset_m", 0.0)
        self.declare_parameter("camera_y_offset_m", 0.0)
        self.declare_parameter("camera_z_offset_m", 0.27)
        self.declare_parameter("leader_look_target_x_m", 0.0)
        self.declare_parameter("leader_look_target_y_m", 0.0)
        self.declare_parameter("camera_look_target_z_m", 0.0)
        self.declare_parameter("camera_yaw_offset_deg", 0.0)
        self.declare_parameter("camera_pan_sign", 1.0)
        self.declare_parameter("default_pan_deg", 0.0)
        self.declare_parameter("pan_enable", True)
        self.declare_parameter("default_tilt_deg", 0.0)
        self.declare_parameter("tilt_enable", True)
        self.declare_parameter("tilt_deadband_deg", 0.0)

        self.uav_name = str(self.get_parameter("uav_name").value)
        self.leader_input_type = str(self.get_parameter("leader_input_type").value).strip().lower()
        self.leader_odom_topic = str(self.get_parameter("leader_odom_topic").value)
        self.leader_pose_topic = str(self.get_parameter("leader_pose_topic").value)
        self.leader_status_topic = str(self.get_parameter("leader_status_topic").value).strip()
        self.uav_camera_mode = str(self.get_parameter("uav_camera_mode").value).strip().lower()
        self.camera_mount_pitch_deg = float(self.get_parameter("camera_mount_pitch_deg").value)
        self.uav_pose_cmd_topic = str(self.get_parameter("uav_pose_cmd_topic").value).strip()
        self.tick_hz = max(1.0, float(self.get_parameter("tick_hz").value))
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.camera_x_offset_m = float(self.get_parameter("camera_x_offset_m").value)
        self.camera_y_offset_m = float(self.get_parameter("camera_y_offset_m").value)
        self.camera_z_offset_m = float(self.get_parameter("camera_z_offset_m").value)
        self.leader_look_target_x_m = float(self.get_parameter("leader_look_target_x_m").value)
        self.leader_look_target_y_m = float(self.get_parameter("leader_look_target_y_m").value)
        self.camera_look_target_z_m = float(self.get_parameter("camera_look_target_z_m").value)
        self.camera_yaw_offset_deg = float(self.get_parameter("camera_yaw_offset_deg").value)
        self.camera_pan_sign = float(self.get_parameter("camera_pan_sign").value)
        self.default_pan_deg = float(self.get_parameter("default_pan_deg").value)
        self.pan_enable = coerce_bool(self.get_parameter("pan_enable").value)
        self.default_tilt_deg = float(self.get_parameter("default_tilt_deg").value)
        self.tilt_enable = coerce_bool(self.get_parameter("tilt_enable").value)
        self.tilt_deadband_deg = max(0.0, float(self.get_parameter("tilt_deadband_deg").value))

        if self.leader_input_type == "estimate":
            self.leader_input_type = "pose"
        if self.leader_input_type not in ("odom", "pose"):
            raise ValueError("leader_input_type must be 'odom', 'pose', or 'estimate'")
        if self.uav_camera_mode == "integrated":
            self.uav_camera_mode = "integrated_joint"
        if self.uav_camera_mode == "detached":
            self.uav_camera_mode = "detached_model"
        if not self.uav_pose_cmd_topic:
            self.uav_pose_cmd_topic = f"/{self.uav_name}/pose_cmd"

        self.have_leader = False
        self.leader_pose = Pose2D(0.0, 0.0, 0.0)
        self.leader_z = 0.0
        self.last_leader_stamp: Optional[Time] = None
        self.last_leader_status_state: Optional[str] = None
        self.last_leader_status_stamp: Optional[Time] = None
        self.last_trackable_leader_pose: Optional[Pose2D] = None
        self.last_trackable_leader_z: Optional[float] = None

        self.have_uav_actual = False
        self.uav_actual_pose = Pose2D(0.0, 0.0, 0.0)
        self.uav_actual_z = 0.0
        self.last_uav_actual_stamp: Optional[Time] = None

        self.have_uav_cmd = False
        self.uav_cmd_pose = Pose2D(0.0, 0.0, 0.0)
        self.uav_cmd_z = 0.0
        self.last_uav_cmd_stamp: Optional[Time] = None
        self.last_tilt_cmd_deg: Optional[float] = None
        self.last_pan_cmd_deg: Optional[float] = None

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
            self.leader_status_sub = self.create_subscription(
                String,
                self.leader_status_topic,
                self.on_leader_status,
                10,
            ) if self.leader_status_topic else None

        self.uav_sub = self.create_subscription(
            PoseStamped,
            f"/{self.uav_name}/pose",
            self.on_uav_pose,
            10,
        )
        self.uav_cmd_sub = self.create_subscription(
            PoseStamped,
            self.uav_pose_cmd_topic,
            self.on_uav_pose_cmd,
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
            f"tilt_deadband_deg={self.tilt_deadband_deg}, "
            f"leader_look_target_m=({self.leader_look_target_x_m}, "
            f"{self.leader_look_target_y_m}, {self.camera_look_target_z_m}), "
            f"camera_mode={self.uav_camera_mode}, camera_mount_pitch_deg={self.camera_mount_pitch_deg}, "
            f"uav_pose_cmd_topic={self.uav_pose_cmd_topic}"
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
        self.uav_actual_pose = Pose2D(
            float(p.x),
            float(p.y),
            yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.uav_actual_z = float(p.z)
        self.have_uav_actual = True
        try:
            self.last_uav_actual_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_actual_stamp = self.get_clock().now()

    def on_uav_pose_cmd(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_cmd_pose = Pose2D(
            float(p.x),
            float(p.y),
            yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.uav_cmd_z = float(p.z)
        self.have_uav_cmd = True
        try:
            self.last_uav_cmd_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_cmd_stamp = self.get_clock().now()

    def on_leader_status(self, msg: String) -> None:
        state = parse_status_field(msg.data, "state")
        if not state:
            return
        self.last_leader_status_state = state.strip().upper()
        self.last_leader_status_stamp = self.get_clock().now()

    def leader_pose_is_fresh(self, now: Time) -> bool:
        if not self.have_leader or self.last_leader_stamp is None:
            return False
        age_s = (now - self.last_leader_stamp).nanoseconds * 1e-9
        return age_s <= self.pose_timeout_s

    def leader_status_is_fresh(self, now: Time) -> bool:
        if self.last_leader_status_stamp is None:
            return False
        age_s = (now - self.last_leader_status_stamp).nanoseconds * 1e-9
        return age_s <= self.pose_timeout_s

    def leader_pose_is_trackable(self, now: Time) -> bool:
        if not self.leader_pose_is_fresh(now):
            return False
        if self.leader_input_type != "pose":
            return True
        if self.leader_status_topic:
            if self.last_leader_status_state is None or not self.leader_status_is_fresh(now):
                return False
        return self.last_leader_status_state in TRACKABLE_ESTIMATOR_STATES

    def _prefer_uav_cmd_pose(self, now: Time) -> bool:
        return should_prefer_command_pose(
            have_cmd=self.have_uav_cmd,
            cmd_stamp_ns=(
                None if self.last_uav_cmd_stamp is None else int(self.last_uav_cmd_stamp.nanoseconds)
            ),
            have_actual=self.have_uav_actual,
            actual_stamp_ns=(
                None
                if self.last_uav_actual_stamp is None
                else int(self.last_uav_actual_stamp.nanoseconds)
            ),
            now_ns=int(now.nanoseconds),
            pose_timeout_s=self.pose_timeout_s,
        )

    def _tracking_uav_pose(self, now: Time) -> Optional[Pose2D]:
        if self._prefer_uav_cmd_pose(now):
            return self.uav_cmd_pose
        if self.have_uav_actual:
            return self.uav_actual_pose
        if self.have_uav_cmd:
            return self.uav_cmd_pose
        return None

    def _tracking_uav_z(self, now: Time) -> Optional[float]:
        if self._prefer_uav_cmd_pose(now):
            return self.uav_cmd_z
        if self.have_uav_actual:
            return self.uav_actual_z
        if self.have_uav_cmd:
            return self.uav_cmd_z
        return None

    def _leader_look_target_xyz_for(self, leader_pose: Pose2D, leader_z: float) -> tuple[float, float, float]:
        return compute_leader_look_target(
            leader_pose.x,
            leader_pose.y,
            leader_pose.yaw,
            leader_z,
            self.leader_look_target_x_m,
            self.leader_look_target_y_m,
            self.camera_look_target_z_m,
        )

    def _leader_look_target_xyz(self) -> tuple[float, float, float]:
        return self._leader_look_target_xyz_for(self.leader_pose, self.leader_z)

    def _compute_tilt_deg_for_target(
        self,
        uav_pose: Pose2D,
        uav_z: float,
        leader_pose: Pose2D,
        leader_z: float,
    ) -> float:
        target_x, target_y, target_z = self._leader_look_target_xyz_for(leader_pose, leader_z)
        return compute_camera_tilt_deg(
            uav_pose.x,
            uav_pose.y,
            uav_z,
            uav_pose.yaw,
            target_x,
            target_y,
            target_z,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
            self.camera_z_offset_m,
            0.0,
        )

    def _compute_tilt_deg(self, uav_pose: Pose2D, uav_z: float) -> float:
        return self._compute_tilt_deg_for_target(uav_pose, uav_z, self.leader_pose, self.leader_z)

    def _command_tilt_deg(self, uav_pose: Pose2D, uav_z: float) -> float:
        if not self.tilt_enable:
            return self.default_tilt_deg
        target_tilt_deg = self._compute_tilt_deg(uav_pose, uav_z)
        if self.uav_camera_mode == "integrated_joint":
            # The attached gimbal pitch command is relative to the fixed camera
            # mount angle baked into the spawned UAV model.
            return target_tilt_deg + self.camera_mount_pitch_deg
        return target_tilt_deg

    def _compute_pan_deg_for_target(
        self,
        uav_pose: Pose2D,
        leader_pose: Pose2D,
        leader_z: float,
    ) -> float:
        if not self.pan_enable:
            return self.default_pan_deg
        camera_x, camera_y = camera_xy_from_uav_pose(
            uav_pose.x,
            uav_pose.y,
            uav_pose.yaw,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )
        target_x, target_y, _ = self._leader_look_target_xyz_for(leader_pose, leader_z)
        target_yaw = math.atan2(target_y - camera_y, target_x - camera_x)
        # Pan command is the raw residual from current UAV yaw to target yaw.
        # The simulator applies camera_pan_sign and camera_yaw_offset_deg when
        # converting this command into the rendered camera pose.
        pan_rad = wrap_pi(target_yaw - uav_pose.yaw)
        return math.degrees(pan_rad)

    def _compute_pan_deg(self, uav_pose: Pose2D) -> float:
        return self._compute_pan_deg_for_target(uav_pose, self.leader_pose, self.leader_z)

    def _publish_camera_debug_for_target(
        self,
        uav_pose: Pose2D,
        uav_z: float,
        leader_pose: Pose2D,
        leader_z: float,
    ) -> None:
        camera_x, camera_y = camera_xy_from_uav_pose(
            uav_pose.x,
            uav_pose.y,
            uav_pose.yaw,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )
        target_x, target_y, target_z = self._leader_look_target_xyz_for(leader_pose, leader_z)
        target_yaw = math.atan2(target_y - camera_y, target_x - camera_x)
        target_tilt = self._compute_tilt_deg_for_target(uav_pose, uav_z, leader_pose, leader_z)
        quat = quaternion_from_euler(0.0, -math.radians(target_tilt), target_yaw)

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "odom"
        pose_msg.pose.position.x = float(camera_x)
        pose_msg.pose.position.y = float(camera_y)
        pose_msg.pose.position.z = float(uav_z - self.camera_z_offset_m)
        pose_msg.pose.orientation.x = float(quat[0])
        pose_msg.pose.orientation.y = float(quat[1])
        pose_msg.pose.orientation.z = float(quat[2])
        pose_msg.pose.orientation.w = float(quat[3])
        self.target_camera_pose_pub.publish(pose_msg)

        point_msg = PointStamped()
        point_msg.header.stamp = pose_msg.header.stamp
        point_msg.header.frame_id = "odom"
        point_msg.point.x = float(target_x)
        point_msg.point.y = float(target_y)
        point_msg.point.z = float(target_z)
        self.target_look_at_point_pub.publish(point_msg)

        yaw_msg = Float32()
        yaw_msg.data = float(target_yaw)
        self.target_camera_world_yaw_pub.publish(yaw_msg)

    def _publish_camera_debug(self, uav_pose: Pose2D, uav_z: float) -> None:
        self._publish_camera_debug_for_target(uav_pose, uav_z, self.leader_pose, self.leader_z)

    def on_tick(self) -> None:
        now = self.get_clock().now()
        uav_pose = self._tracking_uav_pose(now)
        uav_z = self._tracking_uav_z(now)
        if uav_pose is None or uav_z is None:
            return
        leader_trackable = self.leader_pose_is_trackable(now)
        tracked_leader_pose: Optional[Pose2D] = None
        tracked_leader_z: Optional[float] = None
        if leader_trackable:
            self.last_trackable_leader_pose = Pose2D(
                self.leader_pose.x,
                self.leader_pose.y,
                self.leader_pose.yaw,
            )
            self.last_trackable_leader_z = float(self.leader_z)
            tracked_leader_pose = self.last_trackable_leader_pose
            tracked_leader_z = self.last_trackable_leader_z

        if tracked_leader_pose is not None and tracked_leader_z is not None:
            self._publish_camera_debug_for_target(uav_pose, uav_z, tracked_leader_pose, tracked_leader_z)

        tilt_msg = Float64()
        if self.tilt_enable and tracked_leader_pose is not None and tracked_leader_z is not None:
            raw_tilt_cmd = self._compute_tilt_deg_for_target(
                uav_pose,
                uav_z,
                tracked_leader_pose,
                tracked_leader_z,
            )
            if self.uav_camera_mode == "integrated_joint":
                raw_tilt_cmd += self.camera_mount_pitch_deg
            tilt_msg.data = apply_deadband_command(
                raw_tilt_cmd,
                self.last_tilt_cmd_deg,
                self.tilt_deadband_deg,
            )
        elif self.tilt_enable and self.last_tilt_cmd_deg is not None:
            tilt_msg.data = float(self.last_tilt_cmd_deg)
        else:
            tilt_msg.data = float(self.default_tilt_deg)
        self.tilt_pub.publish(tilt_msg)
        self.last_tilt_cmd_deg = float(tilt_msg.data)

        pan_msg = Float64()
        if self.pan_enable and tracked_leader_pose is not None and tracked_leader_z is not None:
            pan_msg.data = float(
                self._compute_pan_deg_for_target(
                    uav_pose,
                    tracked_leader_pose,
                    tracked_leader_z,
                )
            )
        elif self.pan_enable and self.last_pan_cmd_deg is not None:
            pan_msg.data = float(self.last_pan_cmd_deg)
        else:
            pan_msg.data = float(self.default_pan_deg)
        self.pan_pub.publish(pan_msg)
        self.last_pan_cmd_deg = float(pan_msg.data)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTracker()
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
