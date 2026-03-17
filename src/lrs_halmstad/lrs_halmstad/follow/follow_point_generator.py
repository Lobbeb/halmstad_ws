#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

from lrs_halmstad.common.visual_target_estimate import (
    VisualTargetEstimate,
    decode_visual_target_estimate_msg,
)
from lrs_halmstad.follow.follow_math import (
    Pose2D,
    camera_xy_from_uav_pose,
    coerce_bool,
    quat_from_yaw,
    solve_yaw_to_target,
    wrap_pi,
    yaw_from_quat,
)
from nav_msgs.msg import Odometry


class FollowPointGenerator(Node):
    """Generate a world-frame follow point from the estimator-backed target state."""

    def __init__(self) -> None:
        super().__init__("follow_point_generator")

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("target_estimate_topic", "/coord/leader_visual_target_estimate")
        self.declare_parameter("target_pose_topic", "/coord/leader_estimate")
        self.declare_parameter("prefer_target_pose_position", True)
        self.declare_parameter("uav_pose_topic", "")
        self.declare_parameter("camera_pose_topic", "")
        self.declare_parameter("out_topic", "/coord/leader_follow_point")
        self.declare_parameter("status_topic", "/coord/leader_follow_point_status")
        self.declare_parameter("publish_status", True)
        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("target_estimate_timeout_s", 1.0)
        self.declare_parameter("uav_pose_timeout_s", 0.5)
        self.declare_parameter("camera_pose_timeout_s", 0.5)
        self.declare_parameter("hold_timeout_s", 0.5)
        self.declare_parameter("follow_distance_m", 7.0)
        self.declare_parameter("lateral_offset_m", 0.0)
        self.declare_parameter("lookahead_horizon_s", 0.25)
        self.declare_parameter("min_target_speed_mps", 0.25)
        self.declare_parameter("prefer_target_pose_heading", True)
        self.declare_parameter("target_pose_timeout_s", 0.75)
        self.declare_parameter("heading_dir_alpha", 0.25)
        self.declare_parameter("heading_hold_timeout_s", 1.0)
        self.declare_parameter("target_velocity_alpha", 0.35)
        self.declare_parameter("point_alpha", 0.55)
        self.declare_parameter("max_follow_point_jump_m", 2.0)
        self.declare_parameter("use_current_altitude", True)
        self.declare_parameter("fixed_z_m", 7.0)
        self.declare_parameter("camera_x_offset_m", 0.0)
        self.declare_parameter("camera_y_offset_m", 0.0)

        self.uav_name = str(self.get_parameter("uav_name").value).strip() or "dji0"
        self.target_estimate_topic = str(self.get_parameter("target_estimate_topic").value).strip()
        self.target_pose_topic = str(self.get_parameter("target_pose_topic").value).strip()
        self.prefer_target_pose_position = coerce_bool(
            self.get_parameter("prefer_target_pose_position").value
        )
        self.uav_pose_topic = (
            str(self.get_parameter("uav_pose_topic").value).strip() or f"/{self.uav_name}/pose"
        )
        self.camera_pose_topic = (
            str(self.get_parameter("camera_pose_topic").value).strip()
            or f"/{self.uav_name}/camera/actual/center_pose"
        )
        self.out_topic = str(self.get_parameter("out_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.publish_status = coerce_bool(self.get_parameter("publish_status").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.target_estimate_timeout_s = float(self.get_parameter("target_estimate_timeout_s").value)
        self.uav_pose_timeout_s = float(self.get_parameter("uav_pose_timeout_s").value)
        self.camera_pose_timeout_s = float(self.get_parameter("camera_pose_timeout_s").value)
        self.hold_timeout_s = float(self.get_parameter("hold_timeout_s").value)
        self.follow_distance_m = float(self.get_parameter("follow_distance_m").value)
        self.lateral_offset_m = float(self.get_parameter("lateral_offset_m").value)
        self.lookahead_horizon_s = float(self.get_parameter("lookahead_horizon_s").value)
        self.min_target_speed_mps = float(self.get_parameter("min_target_speed_mps").value)
        self.prefer_target_pose_heading = coerce_bool(
            self.get_parameter("prefer_target_pose_heading").value
        )
        self.target_pose_timeout_s = float(self.get_parameter("target_pose_timeout_s").value)
        self.heading_dir_alpha = float(self.get_parameter("heading_dir_alpha").value)
        self.heading_hold_timeout_s = float(self.get_parameter("heading_hold_timeout_s").value)
        self.target_velocity_alpha = float(self.get_parameter("target_velocity_alpha").value)
        self.point_alpha = float(self.get_parameter("point_alpha").value)
        self.max_follow_point_jump_m = float(self.get_parameter("max_follow_point_jump_m").value)
        self.use_current_altitude = coerce_bool(self.get_parameter("use_current_altitude").value)
        self.fixed_z_m = float(self.get_parameter("fixed_z_m").value)
        self.camera_x_offset_m = float(self.get_parameter("camera_x_offset_m").value)
        self.camera_y_offset_m = float(self.get_parameter("camera_y_offset_m").value)

        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.target_estimate_timeout_s <= 0.0:
            raise ValueError("target_estimate_timeout_s must be > 0")
        if self.uav_pose_timeout_s <= 0.0 or self.camera_pose_timeout_s <= 0.0:
            raise ValueError("pose timeouts must be > 0")
        if self.hold_timeout_s < 0.0:
            raise ValueError("hold_timeout_s must be >= 0")
        if self.follow_distance_m <= 0.0:
            raise ValueError("follow_distance_m must be > 0")
        if self.lookahead_horizon_s < 0.0:
            raise ValueError("lookahead_horizon_s must be >= 0")
        if self.target_pose_timeout_s <= 0.0:
            raise ValueError("target_pose_timeout_s must be > 0")
        if not (0.0 <= self.heading_dir_alpha <= 1.0):
            raise ValueError("heading_dir_alpha must be within [0, 1]")
        if self.heading_hold_timeout_s < 0.0:
            raise ValueError("heading_hold_timeout_s must be >= 0")
        if not (0.0 <= self.target_velocity_alpha <= 1.0):
            raise ValueError("target_velocity_alpha must be within [0, 1]")
        if not (0.0 <= self.point_alpha <= 1.0):
            raise ValueError("point_alpha must be within [0, 1]")
        if self.max_follow_point_jump_m < 0.0:
            raise ValueError("max_follow_point_jump_m must be >= 0")

        now_msg = self.get_clock().now().to_msg()
        self.last_target_estimate = VisualTargetEstimate(stamp=now_msg, frame_id="", valid=False)
        self.last_target_estimate_stamp: Optional[Time] = None
        self.last_target_pose_xy: Optional[tuple[float, float]] = None
        self.last_target_pose_yaw: Optional[float] = None
        self.last_target_pose_stamp: Optional[Time] = None
        self.uav_pose = Pose2D(0.0, 0.0, 0.0)
        self.uav_z = self.fixed_z_m
        self.last_uav_pose_stamp: Optional[Time] = None
        self.camera_pose = Pose2D(0.0, 0.0, 0.0)
        self.camera_z = self.fixed_z_m
        self.camera_frame_id = "map"
        self.last_camera_pose_stamp: Optional[Time] = None
        self.last_tick_time: Optional[Time] = None

        self.last_target_world_xy: Optional[tuple[float, float]] = None
        self.last_target_world_stamp: Optional[Time] = None
        self.target_world_vx_mps = 0.0
        self.target_world_vy_mps = 0.0
        self.last_heading_dir_xy: Optional[tuple[float, float]] = None
        self.last_heading_time: Optional[Time] = None

        self.last_follow_point_msg: Optional[PoseStamped] = None
        self.last_follow_point_stamp: Optional[Time] = None
        self.last_valid_follow_point_time: Optional[Time] = None

        self.create_subscription(Odometry, self.target_estimate_topic, self.on_target_estimate, 10)
        if self.target_pose_topic:
            self.create_subscription(PoseStamped, self.target_pose_topic, self.on_target_pose, 10)
        self.create_subscription(PoseStamped, self.uav_pose_topic, self.on_uav_pose, 10)
        self.create_subscription(PoseStamped, self.camera_pose_topic, self.on_camera_pose, 10)
        self.follow_point_pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10) if self.publish_status else None
        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)

        self.get_logger().info(
            "[follow_point_generator] Started: "
            f"target_estimate={self.target_estimate_topic}, camera_pose={self.camera_pose_topic}, "
            f"uav_pose={self.uav_pose_topic}, out={self.out_topic}"
        )

    def on_target_estimate(self, msg: Odometry) -> None:
        self.last_target_estimate = decode_visual_target_estimate_msg(msg)
        try:
            self.last_target_estimate_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_target_estimate_stamp = self.get_clock().now()

    def on_target_pose(self, msg: PoseStamped) -> None:
        self.last_target_pose_xy = (
            float(msg.pose.position.x),
            float(msg.pose.position.y),
        )
        q = msg.pose.orientation
        self.last_target_pose_yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        try:
            self.last_target_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_target_pose_stamp = self.get_clock().now()

    def on_uav_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_pose = Pose2D(
            float(p.x),
            float(p.y),
            yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.uav_z = float(p.z)
        try:
            self.last_uav_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_uav_pose_stamp = self.get_clock().now()

    def on_camera_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.camera_pose = Pose2D(
            float(p.x),
            float(p.y),
            yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.camera_z = float(p.z)
        self.camera_frame_id = str(msg.header.frame_id or self.camera_frame_id)
        try:
            self.last_camera_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_camera_pose_stamp = self.get_clock().now()

    def _is_fresh(self, stamp: Optional[Time], timeout_s: float, now: Time) -> bool:
        if stamp is None:
            return False
        return (now - stamp).nanoseconds * 1e-9 <= timeout_s

    def _dt_s(self, now: Time) -> float:
        if self.last_tick_time is None:
            return 1.0 / self.tick_hz
        dt_s = (now - self.last_tick_time).nanoseconds * 1e-9
        return max(1.0 / self.tick_hz, min(0.5, dt_s))

    def _publish_status(
        self,
        *,
        now: Time,
        state: str,
        reason: str,
        policy_mode: str,
        target_x: Optional[float],
        target_y: Optional[float],
        target_speed_mps: float,
        follow_x: Optional[float],
        follow_y: Optional[float],
        follow_z: Optional[float],
        follow_yaw: Optional[float],
    ) -> None:
        if self.status_pub is None:
            return
        estimate_age_ms = (
            float("nan")
            if self.last_target_estimate_stamp is None
            else max(0.0, (now - self.last_target_estimate_stamp).nanoseconds * 1e-6)
        )
        msg = String()
        msg.data = (
            f"state={state} "
            f"reason={reason} "
            f"policy_mode={policy_mode} "
            f"estimate_age_ms={'na' if not math.isfinite(estimate_age_ms) else f'{estimate_age_ms:.1f}'} "
            f"track_id={self.last_target_estimate.track_id or 'none'} "
            f"target_speed_mps={target_speed_mps:.3f} "
            f"target_x={'na' if target_x is None else f'{target_x:.3f}'} "
            f"target_y={'na' if target_y is None else f'{target_y:.3f}'} "
            f"follow_x={'na' if follow_x is None else f'{follow_x:.3f}'} "
            f"follow_y={'na' if follow_y is None else f'{follow_y:.3f}'} "
            f"follow_z={'na' if follow_z is None else f'{follow_z:.3f}'} "
            f"follow_yaw={'na' if follow_yaw is None else f'{follow_yaw:.3f}'}"
        )
        self.status_pub.publish(msg)

    def _target_world_xy(
        self,
        estimate: VisualTargetEstimate,
        now: Time,
    ) -> Optional[tuple[float, float]]:
        if not estimate.valid or not math.isfinite(estimate.range_m) or estimate.range_m <= 0.0:
            return None
        if (
            self.prefer_target_pose_position
            and self.last_target_pose_xy is not None
            and self._is_fresh(self.last_target_pose_stamp, self.target_pose_timeout_s, now)
        ):
            return self.last_target_pose_xy
        camera_pose, _, _ = self._camera_pose_for_follow(self.get_clock().now())
        bearing_world = wrap_pi(camera_pose.yaw - math.atan2(float(estimate.rel_x_m), float(estimate.rel_z_m)))
        range_m = float(estimate.range_m)
        return (
            float(camera_pose.x + range_m * math.cos(bearing_world)),
            float(camera_pose.y + range_m * math.sin(bearing_world)),
        )

    def _camera_pose_for_follow(self, now: Time) -> tuple[Pose2D, float, str]:
        if self._is_fresh(self.last_camera_pose_stamp, self.camera_pose_timeout_s, now):
            return self.camera_pose, self.camera_z, "actual"
        if self._is_fresh(self.last_uav_pose_stamp, self.uav_pose_timeout_s, now):
            cam_x, cam_y = camera_xy_from_uav_pose(
                self.uav_pose.x,
                self.uav_pose.y,
                self.uav_pose.yaw,
                self.camera_x_offset_m,
                self.camera_y_offset_m,
            )
            return Pose2D(cam_x, cam_y, self.uav_pose.yaw), self.uav_z, "derived"
        return self.camera_pose, self.camera_z, "missing"

    def _update_target_velocity(self, now: Time, target_xy: tuple[float, float]) -> float:
        if self.last_target_world_xy is None or self.last_target_world_stamp is None:
            self.last_target_world_xy = target_xy
            self.last_target_world_stamp = now
            self.target_world_vx_mps = 0.0
            self.target_world_vy_mps = 0.0
            return 0.0

        dt_s = max(1e-3, (now - self.last_target_world_stamp).nanoseconds * 1e-9)
        meas_vx = (float(target_xy[0]) - float(self.last_target_world_xy[0])) / dt_s
        meas_vy = (float(target_xy[1]) - float(self.last_target_world_xy[1])) / dt_s
        alpha = self.target_velocity_alpha
        self.target_world_vx_mps = (1.0 - alpha) * self.target_world_vx_mps + alpha * meas_vx
        self.target_world_vy_mps = (1.0 - alpha) * self.target_world_vy_mps + alpha * meas_vy
        self.last_target_world_xy = target_xy
        self.last_target_world_stamp = now
        return math.hypot(self.target_world_vx_mps, self.target_world_vy_mps)

    def _smooth_follow_xy(self, raw_x: float, raw_y: float) -> tuple[float, float]:
        if self.last_follow_point_msg is None:
            return raw_x, raw_y
        prev_x = float(self.last_follow_point_msg.pose.position.x)
        prev_y = float(self.last_follow_point_msg.pose.position.y)
        alpha = self.point_alpha
        smooth_x = (1.0 - alpha) * prev_x + alpha * raw_x
        smooth_y = (1.0 - alpha) * prev_y + alpha * raw_y
        jump = math.hypot(smooth_x - prev_x, smooth_y - prev_y)
        if self.max_follow_point_jump_m > 0.0 and jump > self.max_follow_point_jump_m:
            scale = self.max_follow_point_jump_m / max(jump, 1e-6)
            smooth_x = prev_x + (smooth_x - prev_x) * scale
            smooth_y = prev_y + (smooth_y - prev_y) * scale
        return smooth_x, smooth_y

    def _update_heading_dir(self, now: Time, dir_x: float, dir_y: float) -> None:
        norm = math.hypot(dir_x, dir_y)
        if norm <= 1e-6:
            return
        next_x = float(dir_x / norm)
        next_y = float(dir_y / norm)
        if self.last_heading_dir_xy is not None and self.heading_dir_alpha < 1.0:
            prev_x, prev_y = self.last_heading_dir_xy
            blend_x = (1.0 - self.heading_dir_alpha) * prev_x + self.heading_dir_alpha * next_x
            blend_y = (1.0 - self.heading_dir_alpha) * prev_y + self.heading_dir_alpha * next_y
            blend_norm = math.hypot(blend_x, blend_y)
            if blend_norm > 1e-6:
                next_x = float(blend_x / blend_norm)
                next_y = float(blend_y / blend_norm)
        self.last_heading_dir_xy = (next_x, next_y)
        self.last_heading_time = now

    def _held_heading_dir(self, now: Time) -> Optional[tuple[float, float]]:
        if self.last_heading_dir_xy is None:
            return None
        if not self._is_fresh(self.last_heading_time, self.heading_hold_timeout_s, now):
            return None
        return self.last_heading_dir_xy

    def _anchor_heading_dir(self, target_x: float, target_y: float) -> Optional[tuple[float, float]]:
        if self.last_follow_point_msg is None:
            return None
        prev_x = float(self.last_follow_point_msg.pose.position.x)
        prev_y = float(self.last_follow_point_msg.pose.position.y)
        dir_x = float(target_x) - prev_x
        dir_y = float(target_y) - prev_y
        norm = math.hypot(dir_x, dir_y)
        if norm <= 1e-6:
            return None
        return float(dir_x / norm), float(dir_y / norm)

    def _target_pose_heading_dir(self, now: Time) -> Optional[tuple[float, float]]:
        if not self.prefer_target_pose_heading:
            return None
        if self.last_target_pose_yaw is None:
            return None
        if not self._is_fresh(self.last_target_pose_stamp, self.target_pose_timeout_s, now):
            return None
        return math.cos(self.last_target_pose_yaw), math.sin(self.last_target_pose_yaw)

    def _select_follow_heading(
        self,
        *,
        now: Time,
        pred_target_x: float,
        pred_target_y: float,
        target_speed_mps: float,
    ) -> tuple[float, float, str]:
        pose_heading = self._target_pose_heading_dir(now)
        if pose_heading is not None:
            self._update_heading_dir(now, pose_heading[0], pose_heading[1])
            return float(self.last_heading_dir_xy[0]), float(self.last_heading_dir_xy[1]), "target_pose_heading"

        if target_speed_mps >= self.min_target_speed_mps:
            dir_x = self.target_world_vx_mps / max(target_speed_mps, 1e-6)
            dir_y = self.target_world_vy_mps / max(target_speed_mps, 1e-6)
            self._update_heading_dir(now, dir_x, dir_y)
            return float(self.last_heading_dir_xy[0]), float(self.last_heading_dir_xy[1]), "motion_heading"

        held_heading = self._held_heading_dir(now)
        if held_heading is not None:
            return float(held_heading[0]), float(held_heading[1]), "held_heading"

        anchor_heading = self._anchor_heading_dir(pred_target_x, pred_target_y)
        if anchor_heading is not None:
            self._update_heading_dir(now, anchor_heading[0], anchor_heading[1])
            return float(anchor_heading[0]), float(anchor_heading[1]), "anchor_heading"

        view_dx = pred_target_x - self.uav_pose.x
        view_dy = pred_target_y - self.uav_pose.y
        view_dist = math.hypot(view_dx, view_dy)
        if view_dist <= 1e-6:
            dir_x = math.cos(self.uav_pose.yaw)
            dir_y = math.sin(self.uav_pose.yaw)
        else:
            dir_x = view_dx / view_dist
            dir_y = view_dy / view_dist
        return dir_x, dir_y, "view_line"

    def _publish_follow_point(self, now: Time, x: float, y: float, z: float, yaw_rad: float) -> PoseStamped:
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.camera_frame_id or "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        qx, qy, qz, qw = quat_from_yaw(float(yaw_rad))
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        self.follow_point_pub.publish(msg)
        self.last_follow_point_msg = msg
        self.last_follow_point_stamp = now
        self.last_valid_follow_point_time = now
        return msg

    def on_tick(self) -> None:
        now = self.get_clock().now()
        state = "INVALID"
        reason = "target_missing"
        policy_mode = "none"
        camera_pose_src = "missing"
        target_x = None
        target_y = None
        target_speed_mps = 0.0
        follow_x = None
        follow_y = None
        follow_z = None
        follow_yaw = None

        estimate_fresh = self.last_target_estimate.valid and self._is_fresh(
            self.last_target_estimate_stamp, self.target_estimate_timeout_s, now
        )
        uav_pose_fresh = self._is_fresh(self.last_uav_pose_stamp, self.uav_pose_timeout_s, now)
        camera_pose, camera_z, camera_pose_src = self._camera_pose_for_follow(now)
        camera_pose_ready = camera_pose_src in ("actual", "derived")

        if estimate_fresh and uav_pose_fresh and camera_pose_ready:
            self.camera_pose = camera_pose
            self.camera_z = camera_z
            target_xy = self._target_world_xy(self.last_target_estimate, now)
            if target_xy is not None:
                target_x, target_y = target_xy
                target_speed_mps = self._update_target_velocity(now, target_xy)
                pred_target_x = float(target_x + self.target_world_vx_mps * self.lookahead_horizon_s)
                pred_target_y = float(target_y + self.target_world_vy_mps * self.lookahead_horizon_s)
                dir_x, dir_y, policy_mode = self._select_follow_heading(
                    now=now,
                    pred_target_x=pred_target_x,
                    pred_target_y=pred_target_y,
                    target_speed_mps=target_speed_mps,
                )

                perp_x = -dir_y
                perp_y = dir_x
                raw_follow_x = pred_target_x - self.follow_distance_m * dir_x + self.lateral_offset_m * perp_x
                raw_follow_y = pred_target_y - self.follow_distance_m * dir_y + self.lateral_offset_m * perp_y
                follow_x, follow_y = self._smooth_follow_xy(raw_follow_x, raw_follow_y)
                follow_z = self.uav_z if self.use_current_altitude else self.fixed_z_m
                yaw_cmd = solve_yaw_to_target(
                    follow_x,
                    follow_y,
                    pred_target_x,
                    pred_target_y,
                    self.camera_x_offset_m,
                    self.camera_y_offset_m,
                )
                follow_yaw = wrap_pi(float(yaw_cmd))
                self._publish_follow_point(now, follow_x, follow_y, follow_z, follow_yaw)
                state = "ACTIVE"
                reason = "none"
            else:
                reason = "target_projection_invalid"
        elif self.last_follow_point_msg is not None and self.last_valid_follow_point_time is not None:
            age_s = max(0.0, (now - self.last_valid_follow_point_time).nanoseconds * 1e-9)
            if age_s <= self.hold_timeout_s:
                held = PoseStamped()
                held.header.stamp = now.to_msg()
                held.header.frame_id = self.last_follow_point_msg.header.frame_id
                held.pose = self.last_follow_point_msg.pose
                self.follow_point_pub.publish(held)
                self.last_follow_point_msg = held
                self.last_follow_point_stamp = now
                follow_x = float(held.pose.position.x)
                follow_y = float(held.pose.position.y)
                follow_z = float(held.pose.position.z)
                follow_yaw = yaw_from_quat(
                    float(held.pose.orientation.x),
                    float(held.pose.orientation.y),
                    float(held.pose.orientation.z),
                    float(held.pose.orientation.w),
                )
                state = "HOLD"
                reason = "short_loss_hold"
                policy_mode = "hold"
        else:
            if not uav_pose_fresh:
                reason = "uav_pose_missing"
            elif not camera_pose_ready:
                reason = "camera_pose_missing"

        self._publish_status(
            now=now,
            state=state,
            reason=f"{reason}:{camera_pose_src}",
            policy_mode=policy_mode,
            target_x=target_x,
            target_y=target_y,
            target_speed_mps=target_speed_mps,
            follow_x=follow_x,
            follow_y=follow_y,
            follow_z=follow_z,
            follow_yaw=follow_yaw,
        )
        self.last_tick_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowPointGenerator()
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
