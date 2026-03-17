#!/usr/bin/env python3
from __future__ import annotations

import math
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Joy
from std_msgs.msg import String

from lrs_halmstad.follow.follow_math import Pose2D, coerce_bool, quat_from_yaw, wrap_pi, yaw_from_quat


class VisualActuationBridge(Node):
    """Bridge bounded visual-follow commands into the existing pose-like UAV command interface."""

    def __init__(self) -> None:
        super().__init__("visual_actuation_bridge")

        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("input_mode", "auto")
        self.declare_parameter("visual_control_topic", "/coord/leader_visual_control")
        self.declare_parameter("follow_point_topic", "/coord/leader_follow_point")
        self.declare_parameter("planned_target_topic", "/coord/leader_planned_target")
        self.declare_parameter("uav_pose_topic", "")
        self.declare_parameter("out_topic", "")
        self.declare_parameter("pose_cmd_topic", "")
        self.declare_parameter("status_topic", "/coord/leader_visual_actuation_bridge_status")
        self.declare_parameter("tick_hz", 20.0)
        self.declare_parameter("control_timeout_s", 0.5)
        self.declare_parameter("follow_point_timeout_s", 0.5)
        self.declare_parameter("planned_target_timeout_s", 0.5)
        self.declare_parameter("pose_timeout_s", 0.5)
        self.declare_parameter("forward_scale", 1.0)
        self.declare_parameter("yaw_scale", 1.0)
        self.declare_parameter("max_xy_step_m", 0.25)
        self.declare_parameter("max_yaw_step_rad", 0.20)
        self.declare_parameter("publish_pose_cmd_mirror", True)
        self.declare_parameter("use_current_altitude", True)
        self.declare_parameter("fixed_z_m", 7.0)

        self.uav_name = str(self.get_parameter("uav_name").value).strip() or "dji0"
        self.input_mode = str(self.get_parameter("input_mode").value).strip().lower() or "auto"
        self.visual_control_topic = str(self.get_parameter("visual_control_topic").value).strip()
        self.follow_point_topic = str(self.get_parameter("follow_point_topic").value).strip()
        self.planned_target_topic = str(self.get_parameter("planned_target_topic").value).strip()
        self.uav_pose_topic = (
            str(self.get_parameter("uav_pose_topic").value).strip()
            or f"/{self.uav_name}/pose"
        )
        self.out_topic = (
            str(self.get_parameter("out_topic").value).strip()
            or f"/{self.uav_name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw"
        )
        self.pose_cmd_topic = (
            str(self.get_parameter("pose_cmd_topic").value).strip()
            or f"/{self.uav_name}/pose_cmd"
        )
        self.status_topic = str(self.get_parameter("status_topic").value).strip()
        self.tick_hz = float(self.get_parameter("tick_hz").value)
        self.control_timeout_s = float(self.get_parameter("control_timeout_s").value)
        self.follow_point_timeout_s = float(self.get_parameter("follow_point_timeout_s").value)
        self.planned_target_timeout_s = float(self.get_parameter("planned_target_timeout_s").value)
        self.pose_timeout_s = float(self.get_parameter("pose_timeout_s").value)
        self.forward_scale = float(self.get_parameter("forward_scale").value)
        self.yaw_scale = float(self.get_parameter("yaw_scale").value)
        self.max_xy_step_m = float(self.get_parameter("max_xy_step_m").value)
        self.max_yaw_step_rad = float(self.get_parameter("max_yaw_step_rad").value)
        self.publish_pose_cmd_mirror = coerce_bool(self.get_parameter("publish_pose_cmd_mirror").value)
        self.use_current_altitude = coerce_bool(self.get_parameter("use_current_altitude").value)
        self.fixed_z_m = float(self.get_parameter("fixed_z_m").value)

        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.input_mode not in ("auto", "control", "follow_point", "planned_target"):
            raise ValueError("input_mode must be one of: auto, control, follow_point, planned_target")
        if self.control_timeout_s < 0.0:
            raise ValueError("control_timeout_s must be >= 0")
        if self.follow_point_timeout_s < 0.0:
            raise ValueError("follow_point_timeout_s must be >= 0")
        if self.planned_target_timeout_s < 0.0:
            raise ValueError("planned_target_timeout_s must be >= 0")
        if self.pose_timeout_s <= 0.0:
            raise ValueError("pose_timeout_s must be > 0")
        if self.max_xy_step_m < 0.0:
            raise ValueError("max_xy_step_m must be >= 0")
        if self.max_yaw_step_rad < 0.0:
            raise ValueError("max_yaw_step_rad must be >= 0")

        self.have_pose = False
        self.uav_pose = Pose2D(0.0, 0.0, 0.0)
        self.uav_z = self.fixed_z_m
        self.last_pose_stamp: Optional[Time] = None

        self.last_control_forward = 0.0
        self.last_control_yaw = 0.0
        self.last_control_stamp: Optional[Time] = None
        self.last_follow_point: Optional[PoseStamped] = None
        self.last_follow_point_stamp: Optional[Time] = None
        self.last_planned_target: Optional[PoseStamped] = None
        self.last_planned_target_stamp: Optional[Time] = None
        self.last_tick_time: Optional[Time] = None

        self.create_subscription(TwistStamped, self.visual_control_topic, self.on_visual_control, 10)
        self.create_subscription(PoseStamped, self.follow_point_topic, self.on_follow_point, 10)
        self.create_subscription(PoseStamped, self.planned_target_topic, self.on_planned_target, 10)
        self.create_subscription(PoseStamped, self.uav_pose_topic, self.on_uav_pose, 10)
        self.uav_cmd_pub = self.create_publisher(Joy, self.out_topic, 10)
        self.pose_cmd_pub = (
            self.create_publisher(PoseStamped, self.pose_cmd_topic, 10)
            if self.publish_pose_cmd_mirror
            else None
        )
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)

        self.get_logger().info(
            "[visual_actuation_bridge] Started: "
            f"input_mode={self.input_mode}, visual_control={self.visual_control_topic}, "
            f"follow_point={self.follow_point_topic}, planned_target={self.planned_target_topic}, "
            f"uav_pose={self.uav_pose_topic}, "
            f"out={self.out_topic}, pose_cmd={self.pose_cmd_topic}, "
            f"publish_pose_cmd_mirror={self.publish_pose_cmd_mirror}"
        )

    def on_visual_control(self, msg: TwistStamped) -> None:
        self.last_control_forward = float(msg.twist.linear.x)
        self.last_control_yaw = float(msg.twist.angular.z)
        try:
            self.last_control_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_control_stamp = self.get_clock().now()

    def on_follow_point(self, msg: PoseStamped) -> None:
        self.last_follow_point = msg
        try:
            self.last_follow_point_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_follow_point_stamp = self.get_clock().now()

    def on_planned_target(self, msg: PoseStamped) -> None:
        self.last_planned_target = msg
        try:
            self.last_planned_target_stamp = Time.from_msg(msg.header.stamp)
        except Exception:
            self.last_planned_target_stamp = self.get_clock().now()

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

    def _dt_s(self, now: Time) -> float:
        if self.last_tick_time is None:
            return 1.0 / self.tick_hz
        dt_s = (now - self.last_tick_time).nanoseconds * 1e-9
        return max(1.0 / self.tick_hz, min(0.5, dt_s))

    @staticmethod
    def _clamp_symmetric(value: float, limit: float) -> float:
        limit = max(0.0, float(limit))
        if limit <= 0.0:
            return 0.0
        return max(-limit, min(limit, float(value)))

    def _publish_pose_cmd(self, now: Time, x: float, y: float, z: float, yaw_rad: float) -> None:
        if self.pose_cmd_pub is None:
            return
        msg = PoseStamped()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        qx, qy, qz, qw = quat_from_yaw(float(yaw_rad))
        msg.pose.orientation.x = float(qx)
        msg.pose.orientation.y = float(qy)
        msg.pose.orientation.z = float(qz)
        msg.pose.orientation.w = float(qw)
        self.pose_cmd_pub.publish(msg)

    def _publish_status(
        self,
        *,
        now: Time,
        state: str,
        reason: str,
        active_input: str,
        forward_cmd_mps: float,
        yaw_rate_cmd_rad_s: float,
        dt_s: float,
        pose_x: Optional[float],
        pose_y: Optional[float],
        pose_z: Optional[float],
        pose_yaw: Optional[float],
        target_x: Optional[float],
        target_y: Optional[float],
        target_z: Optional[float],
        target_yaw: Optional[float],
        step_xy_m: float,
        step_yaw_rad: float,
    ) -> None:
        pose_age_ms = (
            float("nan")
            if self.last_pose_stamp is None
            else max(0.0, (now - self.last_pose_stamp).nanoseconds * 1e-6)
        )
        control_age_ms = (
            float("nan")
            if self.last_control_stamp is None
            else max(0.0, (now - self.last_control_stamp).nanoseconds * 1e-6)
        )
        follow_point_age_ms = (
            float("nan")
            if self.last_follow_point_stamp is None
            else max(0.0, (now - self.last_follow_point_stamp).nanoseconds * 1e-6)
        )
        planned_target_age_ms = (
            float("nan")
            if self.last_planned_target_stamp is None
            else max(0.0, (now - self.last_planned_target_stamp).nanoseconds * 1e-6)
        )
        msg = String()
        msg.data = (
            f"input_mode={self.input_mode} "
            f"active_input={active_input} "
            f"state={state} "
            f"reason={reason} "
            f"pose_age_ms={'na' if not math.isfinite(pose_age_ms) else f'{pose_age_ms:.1f}'} "
            f"control_age_ms={'na' if not math.isfinite(control_age_ms) else f'{control_age_ms:.1f}'} "
            f"follow_point_age_ms={'na' if not math.isfinite(follow_point_age_ms) else f'{follow_point_age_ms:.1f}'} "
            f"planned_target_age_ms={'na' if not math.isfinite(planned_target_age_ms) else f'{planned_target_age_ms:.1f}'} "
            f"forward_cmd_mps={forward_cmd_mps:.3f} "
            f"yaw_rate_cmd_rad_s={yaw_rate_cmd_rad_s:.3f} "
            f"dt_s={dt_s:.3f} "
            f"step_xy_m={step_xy_m:.3f} "
            f"step_yaw_rad={step_yaw_rad:.3f} "
            f"pose_x={'na' if pose_x is None else f'{pose_x:.3f}'} "
            f"pose_y={'na' if pose_y is None else f'{pose_y:.3f}'} "
            f"pose_z={'na' if pose_z is None else f'{pose_z:.3f}'} "
            f"pose_yaw={'na' if pose_yaw is None else f'{pose_yaw:.3f}'} "
            f"target_x={'na' if target_x is None else f'{target_x:.3f}'} "
            f"target_y={'na' if target_y is None else f'{target_y:.3f}'} "
            f"target_z={'na' if target_z is None else f'{target_z:.3f}'} "
            f"target_yaw={'na' if target_yaw is None else f'{target_yaw:.3f}'}"
        )
        self.status_pub.publish(msg)

    @staticmethod
    def _target_from_pose_msg(
        msg: Optional[PoseStamped],
        fallback_z: float,
    ) -> tuple[Optional[float], Optional[float], Optional[float], Optional[float]]:
        if msg is None:
            return None, None, None, None
        pose = msg.pose
        q = pose.orientation
        yaw_rad = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        z_value = float(pose.position.z)
        if not math.isfinite(z_value):
            z_value = fallback_z
        return (
            float(pose.position.x),
            float(pose.position.y),
            float(z_value),
            float(yaw_rad),
        )

    def on_tick(self) -> None:
        now = self.get_clock().now()
        dt_s = self._dt_s(now)
        pose_fresh = self.have_pose and self._is_fresh(self.last_pose_stamp, self.pose_timeout_s, now)
        control_fresh = self._is_fresh(self.last_control_stamp, self.control_timeout_s, now)
        follow_point_fresh = self._is_fresh(self.last_follow_point_stamp, self.follow_point_timeout_s, now)
        planned_target_fresh = self._is_fresh(self.last_planned_target_stamp, self.planned_target_timeout_s, now)

        state = "WAIT_POSE"
        reason = "pose_missing"
        active_input = "none"
        forward_cmd = 0.0
        yaw_rate_cmd = 0.0
        target_x = None
        target_y = None
        target_z = None
        target_yaw = None
        step_xy_m = 0.0
        step_yaw_rad = 0.0

        if pose_fresh:
            state = "HOLD"
            reason = "input_stale"

            if self.input_mode == "planned_target":
                use_planned_target = planned_target_fresh
                use_follow_point = False
                active_input = "planned_target"
                if not planned_target_fresh:
                    reason = "planned_target_stale"
            elif self.input_mode == "follow_point":
                use_planned_target = False
                use_follow_point = follow_point_fresh
                active_input = "follow_point"
                if not follow_point_fresh:
                    reason = "follow_point_stale"
            elif self.input_mode == "control":
                use_planned_target = False
                use_follow_point = False
            else:
                use_planned_target = planned_target_fresh
                use_follow_point = not use_planned_target and follow_point_fresh
                if not use_planned_target and not use_follow_point:
                    if self.last_planned_target_stamp is not None:
                        reason = "planned_target_stale"
                    elif self.last_follow_point_stamp is not None:
                        reason = "follow_point_stale"
                    else:
                        reason = "spatial_target_missing"

            if use_planned_target:
                active_input = "planned_target"
                plan_x, plan_y, plan_z, plan_yaw = self._target_from_pose_msg(
                    self.last_planned_target,
                    self.uav_z if self.use_current_altitude else self.fixed_z_m,
                )
                if (
                    plan_x is not None
                    and plan_y is not None
                    and plan_z is not None
                    and plan_yaw is not None
                ):
                    dx = float(plan_x) - self.uav_pose.x
                    dy = float(plan_y) - self.uav_pose.y
                    dist_xy = math.hypot(dx, dy)
                    if self.max_xy_step_m > 0.0 and dist_xy > self.max_xy_step_m:
                        scale = self.max_xy_step_m / max(dist_xy, 1e-9)
                        dx *= scale
                        dy *= scale
                        step_xy_m = self.max_xy_step_m
                    else:
                        step_xy_m = dist_xy
                    yaw_error = wrap_pi(float(plan_yaw) - self.uav_pose.yaw)
                    step_yaw_rad = self._clamp_symmetric(yaw_error, self.max_yaw_step_rad)

                    target_x = self.uav_pose.x + dx
                    target_y = self.uav_pose.y + dy
                    target_z = self.uav_z if self.use_current_altitude else float(plan_z)
                    target_yaw = wrap_pi(self.uav_pose.yaw + step_yaw_rad)
                    if step_xy_m > 1e-6 or abs(step_yaw_rad) > 1e-6:
                        state = "ACTIVE"
                        reason = "planned_target"
                    else:
                        reason = "planned_target_hold"
                else:
                    reason = "planned_target_invalid"
            elif use_follow_point:
                active_input = "follow_point"
                if follow_point_fresh:
                    follow_x, follow_y, follow_z, follow_yaw = self._target_from_pose_msg(
                        self.last_follow_point,
                        self.uav_z if self.use_current_altitude else self.fixed_z_m,
                    )
                    if (
                        follow_x is not None
                        and follow_y is not None
                        and follow_z is not None
                        and follow_yaw is not None
                    ):
                        dx = float(follow_x) - self.uav_pose.x
                        dy = float(follow_y) - self.uav_pose.y
                        dist_xy = math.hypot(dx, dy)
                        if self.max_xy_step_m > 0.0 and dist_xy > self.max_xy_step_m:
                            scale = self.max_xy_step_m / max(dist_xy, 1e-9)
                            dx *= scale
                            dy *= scale
                            step_xy_m = self.max_xy_step_m
                        else:
                            step_xy_m = dist_xy
                        yaw_error = wrap_pi(float(follow_yaw) - self.uav_pose.yaw)
                        step_yaw_rad = self._clamp_symmetric(yaw_error, self.max_yaw_step_rad)

                        target_x = self.uav_pose.x + dx
                        target_y = self.uav_pose.y + dy
                        target_z = self.uav_z if self.use_current_altitude else float(follow_z)
                        target_yaw = wrap_pi(self.uav_pose.yaw + step_yaw_rad)
                        if step_xy_m > 1e-6 or abs(step_yaw_rad) > 1e-6:
                            state = "ACTIVE"
                            reason = "follow_point"
                        else:
                            reason = "follow_point_hold"
                    else:
                        reason = "follow_point_invalid"
                else:
                    reason = "follow_point_stale"
            elif self.input_mode == "control":
                active_input = "control"
                reason = "control_stale"
                if control_fresh:
                    forward_cmd = self.last_control_forward * self.forward_scale
                    yaw_rate_cmd = self.last_control_yaw * self.yaw_scale
                    if abs(forward_cmd) > 1e-6 or abs(yaw_rate_cmd) > 1e-6:
                        state = "ACTIVE"
                        reason = "none"
                    else:
                        reason = "zero_command"

                yaw_step_rad = yaw_rate_cmd * dt_s
                yaw_step_rad = self._clamp_symmetric(yaw_step_rad, self.max_yaw_step_rad)
                yaw_mid = self.uav_pose.yaw + 0.5 * yaw_step_rad
                step_x = math.cos(yaw_mid) * forward_cmd * dt_s
                step_y = math.sin(yaw_mid) * forward_cmd * dt_s
                step_xy_m = math.hypot(step_x, step_y)
                if self.max_xy_step_m > 0.0 and step_xy_m > self.max_xy_step_m:
                    scale = self.max_xy_step_m / step_xy_m
                    step_x *= scale
                    step_y *= scale
                    step_xy_m = self.max_xy_step_m
                step_yaw_rad = yaw_step_rad

                target_x = self.uav_pose.x + step_x
                target_y = self.uav_pose.y + step_y
                target_z = self.uav_z if self.use_current_altitude else self.fixed_z_m
                target_yaw = wrap_pi(self.uav_pose.yaw + yaw_step_rad)

            if target_x is None or target_y is None or target_z is None or target_yaw is None:
                target_x = self.uav_pose.x
                target_y = self.uav_pose.y
                target_z = self.uav_z if self.use_current_altitude else self.fixed_z_m
                target_yaw = self.uav_pose.yaw

            joy = Joy()
            joy.axes = [float(target_x), float(target_y), float(target_z), float(target_yaw)]
            self.uav_cmd_pub.publish(joy)
            self._publish_pose_cmd(now, target_x, target_y, target_z, target_yaw)

        self._publish_status(
            now=now,
            state=state,
            reason=reason,
            active_input=active_input,
            forward_cmd_mps=forward_cmd,
            yaw_rate_cmd_rad_s=yaw_rate_cmd,
            dt_s=dt_s,
            pose_x=self.uav_pose.x if pose_fresh else None,
            pose_y=self.uav_pose.y if pose_fresh else None,
            pose_z=self.uav_z if pose_fresh else None,
            pose_yaw=self.uav_pose.yaw if pose_fresh else None,
            target_x=target_x,
            target_y=target_y,
            target_z=target_z,
            target_yaw=target_yaw,
            step_xy_m=step_xy_m,
            step_yaw_rad=step_yaw_rad,
        )
        self.last_tick_time = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisualActuationBridge()
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
