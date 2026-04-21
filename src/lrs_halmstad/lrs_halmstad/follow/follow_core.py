#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.time import Time
from sensor_msgs.msg import Joy

from lrs_halmstad.follow.follow_math import (
    Pose2D,
    camera_xy_from_uav_pose,
    horizontal_distance_for_euclidean,
    quat_from_yaw,
    vertical_distance_for_euclidean,
    wrap_pi,
    yaw_from_quat,
)


def compute_rate_limited_axis_value(
    current_value: float,
    target_value: float,
    *,
    tick_hz: float,
    max_speed_per_s: float,
    gain: float,
    snap_tolerance: float = 0.01,
) -> float:
    if tick_hz <= 0.0:
        return float(target_value)

    error = float(target_value) - float(current_value)
    if abs(error) <= max(0.0, float(snap_tolerance)):
        return float(target_value)

    axis_speed_per_s = min(
        max(float(max_speed_per_s), 0.0),
        max(float(gain), 0.0) * abs(error),
    )
    step_limit = axis_speed_per_s / float(tick_hz) if axis_speed_per_s > 0.0 else 0.0
    if step_limit <= 0.0:
        return float(current_value)
    if abs(error) <= step_limit:
        return float(target_value)
    return float(current_value) + math.copysign(step_limit, error)


def compute_controller_style_xy_command(
    current_x: float,
    current_y: float,
    target_x: float,
    target_y: float,
    *,
    tick_hz: float,
    speed_mps: float,
    step_scale: float = 1.0,
    snap_tolerance: float = 1e-6,
) -> tuple[float, float]:
    dx = float(target_x) - float(current_x)
    dy = float(target_y) - float(current_y)
    distance = math.hypot(dx, dy)
    if distance <= max(0.0, float(snap_tolerance)):
        return float(target_x), float(target_y)
    if tick_hz <= 0.0:
        return float(target_x), float(target_y)

    step_distance = max(0.0, float(speed_mps)) * max(0.0, float(step_scale)) / float(tick_hz)
    if step_distance <= 0.0:
        return float(current_x), float(current_y)
    if distance <= step_distance:
        return float(target_x), float(target_y)

    scale = step_distance / distance
    return float(current_x) + dx * scale, float(current_y) + dy * scale


class FollowControllerCoreMixin:
    def on_uav_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_actual = Pose2D(
            float(p.x),
            float(p.y),
            yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.uav_actual_z = float(p.z)
        self.have_uav_actual = True
        try:
            self.last_uav_actual_time = Time.from_msg(msg.header.stamp)
        except Exception as e:
            # FIX: Warn the operator but don't spam the console (throttle to 1 sec)
            self.get_logger().warn(
                f"UAV Pose timestamp missing or invalid: {e}. Falling back to local clock! Check network sync.", 
                throttle_duration_sec=1.0
            )
            self.last_uav_actual_time = self.get_clock().now()

    def _current_uav_pose(self) -> Pose2D:
        if self.have_uav_actual:
            return self.uav_actual
        if self.have_uav_cmd:
            return self.uav_cmd
        return Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)

    def _current_uav_z(self) -> float:
        if self.have_uav_actual:
            return self.uav_actual_z
        if self.have_uav_cmd:
            return self.uav_cmd_z
        return self.uav_start_z

    def _use_cmd_state_for_control(self) -> bool:
        if not self.have_uav_cmd:
            return False
        if not self.have_uav_actual or self.last_uav_actual_time is None:
            return True
            
        # Check if the user explicitly set 'use_actual_pose_for_control' to True in the YAML
        use_actual = getattr(self, "use_actual_pose_for_control", False)
        if use_actual:
            return False  # False means "DO NOT use cmd state, use actual state instead"
            
        # Default/Simulation behavior: use the command state if it is newer
        if self.last_cmd_time is None:
            return False
        return self.last_cmd_time.nanoseconds > self.last_uav_actual_time.nanoseconds
    
    def _control_uav_pose(self) -> Pose2D:
        if self._use_cmd_state_for_control():
            return self.uav_cmd
        return self._current_uav_pose()

    def _control_uav_z(self) -> float:
        if self._use_cmd_state_for_control():
            return self.uav_cmd_z
        return self._current_uav_z()

    def ugv_pose_is_fresh(self, now: Time) -> bool:
        if not self.have_ugv or self.last_ugv_stamp is None:
            return False
        age_s = (now - self.last_ugv_stamp).nanoseconds * 1e-9
        return age_s <= self.pose_timeout_s

    def can_send_command_now(self, now: Time) -> bool:
        if self.last_cmd_time is None:
            return True
        dt = (now - self.last_cmd_time).nanoseconds * 1e-9
        return dt >= self.min_cmd_period_s

    def compute_uav_xy_command(
        self,
        current_uav: Pose2D,
        target_x: float,
        target_y: float,
        *,
        speed_mps: float,
    ) -> tuple[float, float]:
        mode = str(getattr(self, "uav_xy_command_mode", "direct")).strip().lower()
        step_scale = 1.0
        if mode == "controller_step":
            step_scale = float(getattr(self, "uav_xy_command_step_scale", 0.6))
        return compute_controller_style_xy_command(
            current_uav.x,
            current_uav.y,
            target_x,
            target_y,
            tick_hz=self.tick_hz,
            speed_mps=speed_mps,
            step_scale=step_scale,
        )

    def publish_legacy_uav_command(self, x: float, y: float, z: float, yaw_rad: float) -> None:
        msg = Joy()
        msg.axes = [float(x), float(y), float(z), float(yaw_rad)]
        self.uav_cmd_pub.publish(msg)

    def publish_pose_cmd(self, x: float, y: float, z: float, yaw_rad: float) -> None:
        if self.pose_pub is None:
            return
        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = "map"
        ps.pose.position.x = float(x)
        ps.pose.position.y = float(y)
        ps.pose.position.z = float(z)

        quat = quat_from_yaw(float(yaw_rad))
        ps.pose.orientation.x = float(quat[0])
        ps.pose.orientation.y = float(quat[1])
        ps.pose.orientation.z = float(quat[2])
        ps.pose.orientation.w = float(quat[3])

        self.pose_pub.publish(ps)

    def publish_pose_cmd_odom(self, x: float, y: float, z: float, yaw_rad: float) -> None:
        if self.pose_odom_pub is None:
            self.uav_cmd_z = float(z)
            return
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "map"
        odom.child_frame_id = f"{self.uav_name}/base_link"

        odom.pose.pose.position.x = float(x)
        odom.pose.pose.position.y = float(y)
        odom.pose.pose.position.z = float(z)

        quat = quat_from_yaw(float(yaw_rad))
        odom.pose.pose.orientation.x = float(quat[0])
        odom.pose.pose.orientation.y = float(quat[1])
        odom.pose.pose.orientation.z = float(quat[2])
        odom.pose.pose.orientation.w = float(quat[3])

        self.pose_odom_pub.publish(odom)
        self.uav_cmd_z = float(z)

    def _camera_xy_from_uav_pose(self, x: float, y: float, yaw: float):
        return camera_xy_from_uav_pose(
            x,
            y,
            yaw,
            self.camera_x_offset_m,
            self.camera_y_offset_m,
        )

    def _effective_xy_cap(self, leader_z: float) -> float:
        cap = horizontal_distance_for_euclidean(self.d_target, self.z_min - leader_z)
        if self.xy_max > 0.0:
            cap = min(cap, self.xy_max)
        return max(0.0, float(cap))

    def _effective_z_cap(self, leader_z: float) -> float:
        cap = leader_z + vertical_distance_for_euclidean(self.d_target, self.xy_min)
        if self.z_max > 0.0:
            cap = min(cap, self.z_max)
        return max(self.z_min, float(cap))

    def _bounded_xy_target(self, xy_distance: float, leader_z: float = 0.0) -> float:
        bounded = max(0.0, float(xy_distance))
        xy_cap = self._effective_xy_cap(leader_z)
        if xy_cap > 0.0:
            bounded = min(bounded, xy_cap)
        bounded = max(bounded, self.xy_min)
        return bounded

    def _bounded_z_target(self, z_value: float, leader_z: float = 0.0) -> float:
        bounded = max(self.z_min, float(z_value))
        bounded = min(bounded, self._effective_z_cap(leader_z))
        return bounded

    def _nominal_uav_z_target(self, leader_z: float = 0.0) -> float:
        return self._bounded_z_target(self.uav_start_z, leader_z)

    def _refresh_xy_target(self) -> None:
        self.xy_target, _target_z = self._nominal_target_pair(0.0)

    def _nominal_horizontal_follow_distance(self) -> float:
        xy_target, _target_z = self._nominal_target_pair(0.0)
        return xy_target

    def _nominal_target_pair(self, leader_z: float):
        nominal_z_target = self._nominal_uav_z_target(leader_z)
        nominal_xy_target = horizontal_distance_for_euclidean(
            self.d_target,
            nominal_z_target - leader_z,
        )
        bounded_xy_target = self._bounded_xy_target(nominal_xy_target, leader_z)

        if math.isclose(bounded_xy_target, nominal_xy_target, rel_tol=1e-6, abs_tol=1e-6):
            target_z = nominal_z_target
            target_xy = nominal_xy_target
        else:
            target_z = self._bounded_z_target(
                leader_z + vertical_distance_for_euclidean(self.d_target, bounded_xy_target),
                leader_z,
            )
            target_xy = self._bounded_xy_target(
                horizontal_distance_for_euclidean(self.d_target, target_z - leader_z),
                leader_z,
            )
        return target_xy, target_z

    @staticmethod
    def _unwrap_angle_near(raw_angle: float, reference_angle: Optional[float]) -> float:
        if reference_angle is None:
            return float(raw_angle)
        return float(reference_angle + wrap_pi(raw_angle - reference_angle))
