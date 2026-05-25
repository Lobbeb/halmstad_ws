#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Joy

from lrs_halmstad.common.ros_params import declare_yaml_param, required_param_value
from lrs_halmstad.follow.follow_core import FollowControllerCoreMixin
from lrs_halmstad.follow.follow_math import (
    Pose2D,
    clamp_point_to_radius,
    coerce_bool,
    horizontal_distance_for_euclidean,
    rotate_body_offset,
    solve_yaw_to_target,
    wrap_pi,
    yaw_from_quat,
)


class FollowUavOdom(FollowControllerCoreMixin, Node):
    def __init__(self):
        super().__init__("follow_uav")
        dyn_num = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("world", "orchard")
        self.declare_parameter("uav_name", "dji0")
        self.declare_parameter("leader_odom_topic", "/a201_0000/amcl_pose_odom")

        declare_yaml_param(self, "tick_hz", descriptor=dyn_num)
        declare_yaml_param(self, "d_target", descriptor=dyn_num)
        declare_yaml_param(self, "xy_anchor_max", descriptor=dyn_num)
        declare_yaml_param(self, "seed_uav_cmd_on_start")
        declare_yaml_param(self, "uav_start_x", descriptor=dyn_num)
        declare_yaml_param(self, "uav_start_y", descriptor=dyn_num)
        declare_yaml_param(self, "uav_start_z", descriptor=dyn_num)
        declare_yaml_param(self, "uav_start_yaw_deg", descriptor=dyn_num)
        declare_yaml_param(self, "require_uav_actual_before_motion")
        declare_yaml_param(self, "use_uav_actual_z_on_start")
        declare_yaml_param(self, "start_delay_s", descriptor=dyn_num)
        declare_yaml_param(self, "follow_yaw")
        declare_yaml_param(self, "pose_timeout_s", descriptor=dyn_num)
        declare_yaml_param(self, "min_cmd_period_s", descriptor=dyn_num)
        declare_yaml_param(self, "follow_speed_mps", descriptor=dyn_num)
        declare_yaml_param(self, "follow_speed_gain", descriptor=dyn_num)
        declare_yaml_param(self, "follow_z_offset_m", descriptor=dyn_num)
        declare_yaml_param(self, "follow_yaw_rate_rad_s", descriptor=dyn_num)
        declare_yaml_param(self, "follow_yaw_rate_gain", descriptor=dyn_num)
        declare_yaml_param(self, "publish_pose_cmd_topics")
        declare_yaml_param(self, "leader_heading_offset_deg", descriptor=dyn_num)
        self.declare_parameter("forward_offset_m", 0.0, dyn_num)
        self.declare_parameter("lateral_offset_m", 0.0, dyn_num)

        self.world = str(self.get_parameter("world").value)
        self.uav_name = str(self.get_parameter("uav_name").value)
        self.leader_odom_topic = str(self.get_parameter("leader_odom_topic").value)

        self.tick_hz = float(required_param_value(self, "tick_hz"))
        self.d_target = float(required_param_value(self, "d_target"))
        self.xy_anchor_max = float(required_param_value(self, "xy_anchor_max"))
        self.seed_uav_cmd_on_start = coerce_bool(required_param_value(self, "seed_uav_cmd_on_start"))
        self.uav_start_x = float(required_param_value(self, "uav_start_x"))
        self.uav_start_y = float(required_param_value(self, "uav_start_y"))
        self.uav_start_z = float(required_param_value(self, "uav_start_z"))
        self.uav_start_yaw = math.radians(float(required_param_value(self, "uav_start_yaw_deg")))
        self.require_uav_actual_before_motion = coerce_bool(
            required_param_value(self, "require_uav_actual_before_motion")
        )
        self.use_uav_actual_z_on_start = coerce_bool(
            required_param_value(self, "use_uav_actual_z_on_start")
        )
        self.start_delay_s = max(0.0, float(required_param_value(self, "start_delay_s")))
        self.follow_yaw = coerce_bool(required_param_value(self, "follow_yaw"))
        self.pose_timeout_s = float(required_param_value(self, "pose_timeout_s"))
        self.min_cmd_period_s = float(required_param_value(self, "min_cmd_period_s"))
        self.follow_speed_mps = float(required_param_value(self, "follow_speed_mps"))
        self.follow_speed_gain = float(required_param_value(self, "follow_speed_gain"))
        self.follow_z_offset_m = float(required_param_value(self, "follow_z_offset_m"))
        self.follow_yaw_rate_rad_s = float(required_param_value(self, "follow_yaw_rate_rad_s"))
        self.follow_yaw_rate_gain = float(required_param_value(self, "follow_yaw_rate_gain"))
        self.publish_pose_cmd_topics = coerce_bool(required_param_value(self, "publish_pose_cmd_topics"))
        self.leader_heading_offset_rad = math.radians(
            float(required_param_value(self, "leader_heading_offset_deg"))
        )
        self.forward_offset_m = float(self.get_parameter("forward_offset_m").value)
        self.lateral_offset_m = float(self.get_parameter("lateral_offset_m").value)

        self._validate_parameters()

        self.have_ugv = False
        self.ugv_pose = Pose2D(0.0, 0.0, 0.0)
        self.ugv_z = 0.0
        self.ugv_follow_heading = 0.0
        self.ugv_follow_heading_source = "pose_yaw"
        self.leader_frame_id = "map"
        self.last_ugv_stamp: Optional[Time] = None

        self.uav_cmd = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.uav_cmd_z = self.uav_start_z
        self.have_uav_cmd = bool(self.seed_uav_cmd_on_start)

        self.uav_actual = Pose2D(self.uav_start_x, self.uav_start_y, self.uav_start_yaw)
        self.uav_actual_z = self.uav_start_z
        self.have_uav_actual = False
        self.last_uav_actual_time: Optional[Time] = None
        self._latched_uav_actual_z_on_start = False

        self.last_cmd_time: Optional[Time] = None
        self._start_time: Optional[Time] = None
        self._startup_hold_logged = False
        self._waiting_for_actual_pose_logged = False

        self.leader_sub = self.create_subscription(
            Odometry,
            self.leader_odom_topic,
            self.on_leader_odom,
            10,
        )
        self.uav_pose_sub = self.create_subscription(
            PoseStamped,
            f"/{self.uav_name}/pose",
            self.on_uav_pose,
            10,
        )
        self.uav_cmd_pub = self.create_publisher(
            Joy,
            f"/{self.uav_name}/psdk_ros2/flight_control_setpoint_ENUposition_yaw",
            10,
        )
        self.pose_pub = (
            self.create_publisher(PoseStamped, f"/{self.uav_name}/pose_cmd", 10)
            if self.publish_pose_cmd_topics
            else None
        )
        self.pose_odom_pub = (
            self.create_publisher(Odometry, f"/{self.uav_name}/pose_cmd/odom", 10)
            if self.publish_pose_cmd_topics
            else None
        )
        """ self.anchor_point_pub = self.create_publisher(
            PointStamped,
            f"/{self.uav_name}/follow/target/anchor_point",
            10,
        ) """

        self.timer = self.create_timer(1.0 / self.tick_hz, self.on_tick)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self.get_logger().info(
            f"[follow_uav_odom] Started: world={self.world}, uav={self.uav_name}, "
            f"leader_odom={self.leader_odom_topic}, tick={self.tick_hz}Hz, "
            f"d_target={self.d_target:.2f}, xy_anchor_max={self.xy_anchor_max:.2f}, "
            f"follow_z_offset_m={self.follow_z_offset_m:.2f}, "
            f"follow_yaw={self.follow_yaw}, publish_pose_cmd_topics={self.publish_pose_cmd_topics}, "
            f"slot_offset_xy_m=({self.forward_offset_m:.2f}, {self.lateral_offset_m:.2f}), "
            f"leader_heading_offset_deg={math.degrees(self.leader_heading_offset_rad):.1f}, "
            f"uav_start=({self.uav_start_x:.2f},{self.uav_start_y:.2f},"
            f"{self.uav_start_z:.2f},{math.degrees(self.uav_start_yaw):.1f}deg)"
        )

    def _validate_parameters(self) -> None:
        if self.tick_hz <= 0.0:
            raise ValueError("tick_hz must be > 0")
        if self.d_target <= 0.0:
            raise ValueError("d_target must be > 0")
        if self.xy_anchor_max <= 0.0:
            raise ValueError("xy_anchor_max must be > 0")
        if self.pose_timeout_s <= 0.0:
            raise ValueError("pose_timeout_s must be > 0")
        if self.min_cmd_period_s < 0.0:
            raise ValueError("min_cmd_period_s must be >= 0")
        if (
            self.follow_speed_mps < 0.0
            or self.follow_speed_gain < 0.0
            or self.follow_z_offset_m < 0.0
            or self.follow_yaw_rate_rad_s < 0.0
            or self.follow_yaw_rate_gain < 0.0
        ):
            raise ValueError(
                "follow_speed_mps, follow_speed_gain, follow_z_offset_m, follow_yaw_rate_rad_s, "
                "and follow_yaw_rate_gain must be >= 0"
            )
        if self.follow_z_offset_m > self.d_target:
            raise ValueError("follow_z_offset_m must be <= d_target to preserve the 3D follow distance")

    def _on_set_parameters(self, params):
        numeric_nonnegative = {
            "follow_speed_mps",
            "follow_speed_gain",
            "follow_yaw_rate_rad_s",
            "follow_yaw_rate_gain",
            "min_cmd_period_s",
            "start_delay_s",
            "follow_z_offset_m",
        }
        numeric_positive = {
            "d_target",
            "xy_anchor_max",
            "tick_hz",
            "pose_timeout_s",
        }
        updates = {}

        for param in params:
            if param.name in numeric_nonnegative or param.name in numeric_positive:
                try:
                    value = float(param.value)
                except Exception as exc:
                    return SetParametersResult(successful=False, reason=f"invalid {param.name}: {exc}")
                if param.name in numeric_positive and value <= 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be > 0")
                if param.name in numeric_nonnegative and value < 0.0:
                    return SetParametersResult(successful=False, reason=f"{param.name} must be >= 0")
                updates[param.name] = value
            elif param.name == "leader_heading_offset_deg":
                try:
                    updates["leader_heading_offset_rad"] = math.radians(float(param.value))
                except Exception as exc:
                    return SetParametersResult(successful=False, reason=f"invalid leader_heading_offset_deg: {exc}")
            elif param.name == "follow_yaw":
                updates[param.name] = coerce_bool(param.value)

        if not updates:
            return SetParametersResult(successful=True)

        next_d_target = float(updates.get("d_target", self.d_target))
        next_z_offset = float(updates.get("follow_z_offset_m", self.follow_z_offset_m))
        if next_z_offset > next_d_target:
            return SetParametersResult(
                successful=False,
                reason="follow_z_offset_m must be <= d_target to preserve the 3D follow distance",
            )

        for name, value in updates.items():
            setattr(self, name, value)

        """ self.get_logger().info(
            "[follow_uav_odom] Runtime parameter update: "
            + ", ".join(f"{name}={value}" for name, value in sorted(updates.items()))
        ) """
        return SetParametersResult(successful=True)

    def _current_uav_z(self) -> float:
        if self.have_uav_actual:
            return self.uav_actual_z
        return self.uav_cmd_z

    def _target_uav_z(self) -> float:
        return self.ugv_z + self.follow_z_offset_m

    def on_leader_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = yaw_from_quat(float(q.x), float(q.y), float(q.z), float(q.w))
        vx_body = float(msg.twist.twist.linear.x)
        vy_body = float(msg.twist.twist.linear.y)
        vx_world = math.cos(yaw) * vx_body - math.sin(yaw) * vy_body
        vy_world = math.sin(yaw) * vx_body + math.cos(yaw) * vy_body
        speed_world = math.hypot(vx_world, vy_world)

        self.ugv_pose = Pose2D(float(p.x), float(p.y), yaw)
        self.ugv_z = float(p.z)
        self.leader_frame_id = msg.header.frame_id or "map"
        if vx_body < -0.05:
            heading = yaw
            self.ugv_follow_heading_source = "reverse_body_yaw"
        elif speed_world > 0.05:
            heading = math.atan2(vy_world, vx_world)
            self.ugv_follow_heading_source = "motion_heading"
        else:
            heading = yaw
            self.ugv_follow_heading_source = "pose_yaw"
        self.ugv_follow_heading = wrap_pi(heading + self.leader_heading_offset_rad)
        self.have_ugv = True
        try:
            self.last_ugv_stamp = Time.from_msg(msg.header.stamp)
        except (ValueError, TypeError, AttributeError):
            self.last_ugv_stamp = self.get_clock().now()

    def on_uav_pose(self, msg: PoseStamped) -> None:
        super().on_uav_pose(msg)
        if not self.use_uav_actual_z_on_start or self._latched_uav_actual_z_on_start:
            return

        self.uav_start_z = self.uav_actual_z
        self.uav_cmd_z = self.uav_actual_z
        self._latched_uav_actual_z_on_start = True
        self.get_logger().info(
            f"[follow_uav_odom] Using current UAV z as startup altitude: "
            f"z={self.uav_start_z:.2f}, follow_z_offset_m={self.follow_z_offset_m:.2f}"
        )

    def _compute_anchor_pose(self, target_horizontal_distance: float) -> Pose2D:
        heading = self.ugv_follow_heading
        anchor_x = self.ugv_pose.x - target_horizontal_distance * math.cos(heading)
        anchor_y = self.ugv_pose.y - target_horizontal_distance * math.sin(heading)
        anchor_x += self.forward_offset_m * math.cos(heading) - self.lateral_offset_m * math.sin(heading)
        anchor_y += self.forward_offset_m * math.sin(heading) + self.lateral_offset_m * math.cos(heading)
        anchor_x, anchor_y = clamp_point_to_radius(
            self.ugv_pose.x,
            self.ugv_pose.y,
            anchor_x,
            anchor_y,
            self.xy_anchor_max,
        )
        yaw_target = solve_yaw_to_target(anchor_x, anchor_y, self.ugv_pose.x, self.ugv_pose.y)
        return Pose2D(anchor_x, anchor_y, yaw_target)

    def publish_anchor_point(self, anchor_pose: Pose2D) -> None:
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.leader_frame_id or "map"
        msg.point.x = float(anchor_pose.x)
        msg.point.y = float(anchor_pose.y)
        msg.point.z = float(self.ugv_z)
        self.anchor_point_pub.publish(msg)

    def _effective_follow_speed_mps(self, current_uav: Pose2D, anchor_pose: Pose2D) -> float:
        error = math.hypot(anchor_pose.x - current_uav.x, anchor_pose.y - current_uav.y)
        return min(
            self.follow_speed_mps,
            self.follow_speed_gain * max(error, 0.0),
        )

    def _compute_command_yaw(self, current_yaw: float, target_yaw: float) -> float:
        if not self.follow_yaw:
            return current_yaw
        yaw_error = wrap_pi(target_yaw - current_yaw)
        yaw_rate_rad_s = min(
            max(self.follow_yaw_rate_rad_s, 0.0),
            max(self.follow_yaw_rate_gain, 0.0) * abs(yaw_error),
        )
        yaw_step_limit = yaw_rate_rad_s / self.tick_hz if yaw_rate_rad_s > 0.0 else 0.0
        if yaw_step_limit > 0.0 and abs(yaw_error) > yaw_step_limit:
            return wrap_pi(current_yaw + math.copysign(yaw_step_limit, yaw_error))
        return wrap_pi(target_yaw)

    def on_tick(self) -> None:
        now = self.get_clock().now()
        if self.require_uav_actual_before_motion and not self.have_uav_actual:
            if not self._waiting_for_actual_pose_logged:
                self.get_logger().info("[follow_uav_odom] Waiting for UAV pose before motion")
                self._waiting_for_actual_pose_logged = True
            return
        self._waiting_for_actual_pose_logged = False

        if self.start_delay_s > 0.0:
            if self._start_time is None:
                self._start_time = now
            elapsed_s = max(0.0, (now - self._start_time).nanoseconds * 1e-9)
            if elapsed_s < self.start_delay_s:
                if not self._startup_hold_logged:
                    self.get_logger().info(
                        f"[follow_uav_odom] Holding commands for {self.start_delay_s:.1f}s"
                    )
                    self._startup_hold_logged = True
                return
            self._startup_hold_logged = False

        if not self.ugv_pose_is_fresh(now):
            return
        if not self.can_send_command_now(now):
            return

        current_uav = self._current_uav_pose()
        target_uav_z = self._target_uav_z()
        vertical_delta = target_uav_z - self.ugv_z
        target_horizontal_distance = horizontal_distance_for_euclidean(self.d_target, vertical_delta)
        anchor_pose = self._compute_anchor_pose(target_horizontal_distance)
        #self.publish_anchor_point(anchor_pose)

        dx = anchor_pose.x - current_uav.x
        dy = anchor_pose.y - current_uav.y
        distance_to_anchor = math.hypot(dx, dy)
        speed_mps = self._effective_follow_speed_mps(current_uav, anchor_pose)
        step_distance = speed_mps / self.tick_hz if self.tick_hz > 0.0 else speed_mps
        if distance_to_anchor < 0.2 or distance_to_anchor <= 1e-9 or step_distance <= 0.0:
            x_cmd = current_uav.x
            y_cmd = current_uav.y
        else:
            step_distance = min(step_distance, distance_to_anchor)
            scale = step_distance / distance_to_anchor
            x_cmd = current_uav.x + dx * scale
            y_cmd = current_uav.y + dy * scale

        z_cmd = target_uav_z
        yaw_target = solve_yaw_to_target(x_cmd, y_cmd, self.ugv_pose.x, self.ugv_pose.y)
        yaw_cmd = self._compute_command_yaw(current_uav.yaw, yaw_target)

        self.publish_legacy_uav_command(x_cmd, y_cmd, z_cmd, yaw_cmd)
        self.publish_pose_cmd(x_cmd, y_cmd, z_cmd, yaw_cmd)
        self.publish_pose_cmd_odom(x_cmd, y_cmd, z_cmd, yaw_cmd)

        self.uav_cmd = Pose2D(x_cmd, y_cmd, yaw_cmd)
        self.have_uav_cmd = True
        self.last_cmd_time = now


def main(args=None):
    rclpy.init(args=args)
    node = FollowUavOdom()
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
