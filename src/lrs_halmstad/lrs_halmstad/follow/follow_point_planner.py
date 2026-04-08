#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.time import Time
from std_msgs.msg import String

from lrs_halmstad.common.visual_target_estimate import (
    VisualTargetEstimate,
    decode_visual_target_estimate_msg,
)
from lrs_halmstad.follow.follow_math import (
    Pose2D,
    clamp_point_to_radius,
    coerce_bool,
    quat_from_yaw,
    wrap_pi,
    yaw_from_quat,
)


class FollowPointPlanner(Node):
    """Convert raw follow points into a smoother planned pose target."""

    def __init__(self) -> None:
        super().__init__("follow_point_planner")

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("follow_point_topic", "/coord/leader_follow_point")
        self.declare_parameter("target_estimate_topic", "/coord/leader_visual_target_estimate")
        self.declare_parameter("uav_pose_topic", "")
        self.declare_parameter("out_topic", "/coord/leader_planned_target")
        self.declare_parameter("status_topic", "/coord/leader_planned_target_status")
        self.declare_parameter("publish_status", True)
        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("follow_point_timeout_s", 0.5)
        self.declare_parameter("target_estimate_timeout_s", 1.0)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("hold_timeout_s", 0.5)
        self.declare_parameter("xy_alpha", 0.45)
        self.declare_parameter("yaw_alpha", 0.55)
        self.declare_parameter("max_planned_step_m", 0.40)
        self.declare_parameter("max_planned_yaw_step_rad", 0.12)
        self.declare_parameter("seed_from_uav_pose", True)
        self.declare_parameter("z_alpha", 0.45)
        self.declare_parameter("max_planned_z_step_m", 0.40)
        self.declare_parameter("use_follow_point_altitude", True)
        self.declare_parameter("fixed_z_m", 7.0)
        self.declare_parameter("stale_fp_thresh_m", 0.05)
        self.declare_parameter("stale_alpha_scale", 0.3)
        self.declare_parameter("predicted_xy_alpha_scale", 0.70)
        self.declare_parameter("degraded_xy_alpha_scale", 0.45)
        self.declare_parameter("predicted_yaw_alpha_scale", 0.80)
        self.declare_parameter("degraded_yaw_alpha_scale", 0.55)
        self.declare_parameter("degraded_step_scale", 0.65)

        self.uav_name = str(self.get_parameter("uav_name").value).strip() or "dji0"
        self.follow_point_topic = str(self.get_parameter("follow_point_topic").value).strip()
        self.target_estimate_topic = str(self.get_parameter("target_estimate_topic").value).strip()
        self.uav_pose_topic = (
            str(self.get_parameter("uav_pose_topic").value).strip()
            or f"/{self.uav_name}/pose"
        )
        self.out_topic = str(self.get_parameter("out_topic").value).strip()
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.publish_status = coerce_bool(self.get_parameter("publish_status").value)
        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.follow_point_timeout_s = float(self.get_parameter("follow_point_timeout_s").value)
        self.target_estimate_timeout_s = float(self.get_parameter("target_estimate_timeout_s").value)
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.hold_timeout_s = float(self.get_parameter("hold_timeout_s").value)
        self.xy_alpha = float(self.get_parameter("xy_alpha").value)
        self.yaw_alpha = float(self.get_parameter("yaw_alpha").value)
        self.max_planned_step_m = float(self.get_parameter("max_planned_step_m").value)
        self.max_planned_yaw_step_rad = float(self.get_parameter("max_planned_yaw_step_rad").value)
        self.seed_from_uav_pose = coerce_bool(self.get_parameter("seed_from_uav_pose").value)
        self.z_alpha = float(self.get_parameter("z_alpha").value)
        self.max_planned_z_step_m = float(self.get_parameter("max_planned_z_step_m").value)
        self.use_follow_point_altitude = coerce_bool(self.get_parameter("use_follow_point_altitude").value)
        self.fixed_z_m = float(self.get_parameter("fixed_z_m").value)
        self.stale_fp_thresh_m = float(self.get_parameter("stale_fp_thresh_m").value)
        self.stale_alpha_scale = max(0.0, min(1.0, float(self.get_parameter("stale_alpha_scale").value)))
        self.predicted_xy_alpha_scale = max(
            0.0, min(1.0, float(self.get_parameter("predicted_xy_alpha_scale").value))
        )
        self.degraded_xy_alpha_scale = max(
            0.0, min(1.0, float(self.get_parameter("degraded_xy_alpha_scale").value))
        )
        self.predicted_yaw_alpha_scale = max(
            0.0, min(1.0, float(self.get_parameter("predicted_yaw_alpha_scale").value))
        )
        self.degraded_yaw_alpha_scale = max(
            0.0, min(1.0, float(self.get_parameter("degraded_yaw_alpha_scale").value))
        )
        self.degraded_step_scale = max(0.0, min(1.0, float(self.get_parameter("degraded_step_scale").value)))

        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.follow_point_timeout_s <= 0.0:
            raise ValueError("follow_point_timeout_s must be > 0")
        if self.target_estimate_timeout_s <= 0.0:
            raise ValueError("target_estimate_timeout_s must be > 0")
        if self.pose_timeout_s <= 0.0:
            raise ValueError("pose_timeout_s must be > 0")
        if self.hold_timeout_s < 0.0:
            raise ValueError("hold_timeout_s must be >= 0")
        if not (0.0 <= self.xy_alpha <= 1.0):
            raise ValueError("xy_alpha must be within [0, 1]")
        if not (0.0 <= self.yaw_alpha <= 1.0):
            raise ValueError("yaw_alpha must be within [0, 1]")
        if self.max_planned_step_m < 0.0:
            raise ValueError("max_planned_step_m must be >= 0")
        if self.max_planned_yaw_step_rad < 0.0:
            raise ValueError("max_planned_yaw_step_rad must be >= 0")

        self.have_pose = False
        self.uav_pose = Pose2D(0.0, 0.0, 0.0)
        self.uav_z = 0.0
        self.last_pose_stamp: Optional[Time] = None
        self.planned_z: float = 0.0
        self._planned_z_seeded = False

        self.last_follow_point: Optional[PoseStamped] = None
        self.last_follow_point_stamp: Optional[Time] = None
        self.last_target_estimate = VisualTargetEstimate(stamp=self.get_clock().now().to_msg(), frame_id="", valid=False)
        self.last_target_estimate_stamp: Optional[Time] = None
        self.last_planned_target: Optional[PoseStamped] = None
        self.last_planned_target_time: Optional[Time] = None
        self._prev_fp_xy: Optional[tuple[float, float]] = None

        self.create_subscription(PoseStamped, self.follow_point_topic, self.on_follow_point, 1)
        self.create_subscription(Odometry, self.target_estimate_topic, self.on_target_estimate, 1)
        self.create_subscription(PoseStamped, self.uav_pose_topic, self.on_uav_pose, 10)
        self.planned_target_pub = self.create_publisher(PoseStamped, self.out_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10) if self.publish_status else None
        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)

        self.get_logger().info(
            "[follow_point_planner] Started: "
            f"follow_point={self.follow_point_topic}, uav_pose={self.uav_pose_topic}, out={self.out_topic}"
        )

    def on_follow_point(self, msg: PoseStamped) -> None:
        self.last_follow_point = msg
        try:
            self.last_follow_point_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_follow_point_stamp = self.get_clock().now()

    def on_target_estimate(self, msg: Odometry) -> None:
        self.last_target_estimate = decode_visual_target_estimate_msg(msg)
        try:
            self.last_target_estimate_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_target_estimate_stamp = self.get_clock().now()

    def on_uav_pose(self, msg: PoseStamped) -> None:
        p = msg.pose.position
        q = msg.pose.orientation
        self.uav_pose = Pose2D(
            float(p.x),
            float(p.y),
            yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w)),
        )
        self.uav_z = float(p.z)
        self.have_pose = True
        try:
            self.last_pose_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_pose_stamp = self.get_clock().now()

    def _is_fresh(self, stamp: Optional[Time], timeout_s: float, now: Time) -> bool:
        if stamp is None:
            return False
        return (now - stamp).nanoseconds * 1e-9 <= timeout_s

    @staticmethod
    def _clamp_symmetric(value: float, limit: float) -> float:
        limit = max(0.0, float(limit))
        if limit <= 0.0:
            return 0.0
        return max(-limit, min(limit, float(value)))

    def _decode_target(
        self,
        msg: Optional[PoseStamped],
    ) -> tuple[Optional[float], Optional[float], Optional[float], Optional[float], str]:
        if msg is None:
            return None, None, None, None, "map"
        pose = msg.pose
        q = pose.orientation
        yaw_rad = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        z_value = float(pose.position.z)
        if not math.isfinite(z_value):
            z_value = self.uav_z
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(z_value),
            float(yaw_rad),
            str(msg.header.frame_id or "map"),
        )

    def _base_plan(self) -> tuple[float, float, float, float, str]:
        if self.last_planned_target is not None:
            x, y, z, yaw_rad, frame_id = self._decode_target(self.last_planned_target)
            if x is not None and y is not None and z is not None and yaw_rad is not None:
                return float(x), float(y), float(z), float(yaw_rad), frame_id
        if self.seed_from_uav_pose and self.have_pose:
            return self.uav_pose.x, self.uav_pose.y, self.uav_z, self.uav_pose.yaw, "map"
        x, y, z, yaw_rad, frame_id = self._decode_target(self.last_follow_point)
        return (
            float(x or 0.0),
            float(y or 0.0),
            float(z if z is not None else self.uav_z),
            float(yaw_rad or 0.0),
            frame_id,
        )

    def _publish_target(
        self,
        *,
        now: Time,
        frame_id: str,
        x: float,
        y: float,
        z: float,
        yaw_rad: float,
        refresh_valid_time: bool = True,
    ) -> None:
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = str(frame_id or "map")
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        qx, qy, qz, qw = quat_from_yaw(float(yaw_rad))
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        self.planned_target_pub.publish(msg)
        self.last_planned_target = msg
        if refresh_valid_time:
            self.last_planned_target_time = now

    def _publish_status(
        self,
        *,
        now: Time,
        state: str,
        reason: str,
        planner_mode: str,
        estimate_mode: str,
        estimate_quality: float,
        raw_x: Optional[float],
        raw_y: Optional[float],
        raw_z: Optional[float],
        raw_yaw: Optional[float],
        planned_x: Optional[float],
        planned_y: Optional[float],
        planned_z: Optional[float],
        planned_yaw: Optional[float],
        step_xy_m: float,
        step_yaw_rad: float,
    ) -> None:
        if self.status_pub is None:
            return
        pose_age_ms = (
            float("nan")
            if self.last_pose_stamp is None
            else max(0.0, (now - self.last_pose_stamp).nanoseconds * 1e-6)
        )
        follow_point_age_ms = (
            float("nan")
            if self.last_follow_point_stamp is None
            else max(0.0, (now - self.last_follow_point_stamp).nanoseconds * 1e-6)
        )
        target_estimate_age_ms = (
            float("nan")
            if self.last_target_estimate_stamp is None
            else max(0.0, (now - self.last_target_estimate_stamp).nanoseconds * 1e-6)
        )
        msg = String()
        msg.data = (
            f"state={state} "
            f"reason={reason} "
            f"planner_mode={planner_mode} "
            f"pose_age_ms={'na' if not math.isfinite(pose_age_ms) else f'{pose_age_ms:.1f}'} "
            f"follow_point_age_ms={'na' if not math.isfinite(follow_point_age_ms) else f'{follow_point_age_ms:.1f}'} "
            f"target_estimate_age_ms={'na' if not math.isfinite(target_estimate_age_ms) else f'{target_estimate_age_ms:.1f}'} "
            f"estimate_mode={estimate_mode} "
            f"estimate_quality={estimate_quality:.3f} "
            f"raw_x={'na' if raw_x is None else f'{raw_x:.3f}'} "
            f"raw_y={'na' if raw_y is None else f'{raw_y:.3f}'} "
            f"raw_z={'na' if raw_z is None else f'{raw_z:.3f}'} "
            f"raw_yaw={'na' if raw_yaw is None else f'{raw_yaw:.3f}'} "
            f"planned_x={'na' if planned_x is None else f'{planned_x:.3f}'} "
            f"planned_y={'na' if planned_y is None else f'{planned_y:.3f}'} "
            f"planned_z={'na' if planned_z is None else f'{planned_z:.3f}'} "
            f"planned_yaw={'na' if planned_yaw is None else f'{planned_yaw:.3f}'} "
            f"step_xy_m={step_xy_m:.3f} "
            f"step_yaw_rad={step_yaw_rad:.3f}"
        )
        self.status_pub.publish(msg)

    def on_tick(self) -> None:
        now = self.get_clock().now()
        pose_fresh = self.have_pose and self._is_fresh(self.last_pose_stamp, self.pose_timeout_s, now)
        follow_point_fresh = self._is_fresh(self.last_follow_point_stamp, self.follow_point_timeout_s, now)
        estimate_fresh = (
            self.last_target_estimate.valid
            and self._is_fresh(self.last_target_estimate_stamp, self.target_estimate_timeout_s, now)
        )
        hold_active = self._is_fresh(self.last_planned_target_time, self.hold_timeout_s, now)
        estimate_mode = self.last_target_estimate.mode if estimate_fresh else "LOST"
        estimate_quality = self.last_target_estimate.quality if estimate_fresh else 0.0

        state = "WAIT_POSE"
        reason = "pose_missing"
        planner_mode = "none"
        raw_x = raw_y = raw_z = raw_yaw = None
        planned_x = planned_y = planned_z = planned_yaw = None
        step_xy_m = 0.0
        step_yaw_rad = 0.0

        if pose_fresh:
            state = "INVALID"
            reason = "follow_point_stale"
            if follow_point_fresh:
                raw_x, raw_y, raw_z, raw_yaw, frame_id = self._decode_target(self.last_follow_point)
                if raw_x is not None and raw_y is not None and raw_z is not None and raw_yaw is not None:
                    base_x, base_y, _, base_yaw, _ = self._base_plan()
                    # Adaptive alpha: reduce when follow point is stationary (cached upstream)
                    eff_alpha = self.xy_alpha
                    eff_yaw_alpha = self.yaw_alpha
                    step_limit = self.max_planned_step_m
                    yaw_step_limit = self.max_planned_yaw_step_rad
                    if estimate_mode == "PREDICTED":
                        eff_alpha *= self.predicted_xy_alpha_scale
                        eff_yaw_alpha *= self.predicted_yaw_alpha_scale
                    elif estimate_mode == "DEGRADED":
                        eff_alpha *= self.degraded_xy_alpha_scale
                        eff_yaw_alpha *= self.degraded_yaw_alpha_scale
                        step_limit *= self.degraded_step_scale
                        yaw_step_limit *= self.degraded_step_scale
                    if estimate_mode in ("PREDICTED", "DEGRADED"):
                        eff_alpha *= max(0.35, min(1.0, estimate_quality if estimate_quality > 0.0 else 0.5))
                        eff_yaw_alpha *= max(0.45, min(1.0, estimate_quality if estimate_quality > 0.0 else 0.6))
                    if self._prev_fp_xy is not None:
                        fp_delta = math.hypot(float(raw_x) - self._prev_fp_xy[0],
                                              float(raw_y) - self._prev_fp_xy[1])
                        if fp_delta < self.stale_fp_thresh_m:
                            eff_alpha *= self.stale_alpha_scale
                    self._prev_fp_xy = (float(raw_x), float(raw_y))
                    interp_x = base_x + eff_alpha * (float(raw_x) - base_x)
                    interp_y = base_y + eff_alpha * (float(raw_y) - base_y)
                    if step_limit > 0.0:
                        interp_x, interp_y = clamp_point_to_radius(
                            base_x,
                            base_y,
                            interp_x,
                            interp_y,
                            step_limit,
                        )
                    step_xy_m = math.hypot(interp_x - base_x, interp_y - base_y)

                    yaw_error = wrap_pi(float(raw_yaw) - base_yaw)
                    step_yaw_rad = self._clamp_symmetric(
                        eff_yaw_alpha * yaw_error,
                        yaw_step_limit,
                    )
                    planned_x = float(interp_x)
                    planned_y = float(interp_y)
                    if not self._planned_z_seeded and self.have_pose:
                        self.planned_z = self.uav_z
                        self._planned_z_seeded = True
                    if math.isfinite(float(raw_z)):
                        step_z = float(raw_z) - self.planned_z
                        if abs(step_z) > self.max_planned_z_step_m:
                            step_z = math.copysign(self.max_planned_z_step_m, step_z)
                        self.planned_z = self.planned_z + self.z_alpha * step_z
                    planned_z = self.planned_z
                    planned_yaw = wrap_pi(base_yaw + step_yaw_rad)
                    self._publish_target(
                        now=now,
                        frame_id=frame_id,
                        x=planned_x,
                        y=planned_y,
                        z=planned_z,
                        yaw_rad=planned_yaw,
                        refresh_valid_time=True,
                    )
                    state = "DEGRADED" if estimate_mode == "DEGRADED" else ("PREDICTED" if estimate_mode == "PREDICTED" else "ACTIVE")
                    reason = "follow_point_degraded" if estimate_mode == "DEGRADED" else ("follow_point_predicted" if estimate_mode == "PREDICTED" else "follow_point")
                    planner_mode = "degraded" if estimate_mode == "DEGRADED" else ("predicted" if estimate_mode == "PREDICTED" else "interpolate")
                else:
                    reason = "follow_point_invalid"
            elif hold_active and self.last_planned_target is not None:
                planned_x, planned_y, planned_z, planned_yaw, frame_id = self._decode_target(
                    self.last_planned_target
                )
                if (
                    planned_x is not None
                    and planned_y is not None
                    and planned_z is not None
                    and planned_yaw is not None
                ):
                    self._publish_target(
                        now=now,
                        frame_id=frame_id,
                        x=planned_x,
                        y=planned_y,
                        z=planned_z,
                        yaw_rad=planned_yaw,
                        refresh_valid_time=False,
                    )
                    state = "HOLD"
                    reason = "follow_point_stale"
                    planner_mode = "hold"

        self._publish_status(
            now=now,
            state=state,
            reason=reason,
            planner_mode=planner_mode,
            estimate_mode=estimate_mode,
            estimate_quality=estimate_quality,
            raw_x=raw_x,
            raw_y=raw_y,
            raw_z=raw_z,
            raw_yaw=raw_yaw,
            planned_x=planned_x,
            planned_y=planned_y,
            planned_z=planned_z,
            planned_yaw=planned_yaw,
            step_xy_m=step_xy_m,
            step_yaw_rad=step_yaw_rad,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FollowPointPlanner()
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
