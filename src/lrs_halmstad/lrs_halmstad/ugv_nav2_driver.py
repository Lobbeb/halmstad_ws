#!/usr/bin/env python3

import math
import random
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
import yaml
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from .follow_math import coerce_bool
from .ugv_motion_profile import (
    MotionProfileConfig,
    MotionWaypoint,
    build_route_waypoints,
    build_segments,
    integrate_segments,
    normalize_angle,
)


@dataclass
class Pose2DState:
    x: float
    y: float
    yaw: float
    frame_id: str
    stamp_ns: int
    received_monotonic_s: float


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)
    return quaternion


def rotate_offset(x_local: float, y_local: float, yaw: float) -> tuple[float, float]:
    return (
        x_local * math.cos(yaw) - y_local * math.sin(yaw),
        x_local * math.sin(yaw) + y_local * math.cos(yaw),
    )


class UgvNav2Driver(Node):
    def __init__(self) -> None:
        super().__init__("ugv_nav2_driver")

        self.declare_parameter("start_delay_s", 0.0)
        self.declare_parameter("motion_profile", "default")
        self.declare_parameter("turn_pattern", "alternate")
        self.declare_parameter("forward_speed", 0.75)
        self.declare_parameter("forward_time_s", 0.90)
        self.declare_parameter("turn_speed", 0.55)
        self.declare_parameter("turn_time_s", 0.65)
        self.declare_parameter("cycles", 120)
        self.declare_parameter("variation_enable", True)
        self.declare_parameter("variation_amplitude", 0.08)
        self.declare_parameter("pause_every_n", 0)
        self.declare_parameter("pause_time_s", 0.0)

        self.declare_parameter("pose_topic", "amcl_pose")
        self.declare_parameter("pose_topic_type", "pose_with_covariance")
        self.declare_parameter("set_initial_pose_enable", True)
        self.declare_parameter("initial_pose_topic", "initialpose")
        self.declare_parameter("initial_pose_frame_id", "map")
        self.declare_parameter("initial_pose_x", 0.0)
        self.declare_parameter("initial_pose_y", 0.0)
        self.declare_parameter("initial_pose_yaw_deg", 0.0)
        self.declare_parameter("initial_pose_covariance_xy", 0.25)
        self.declare_parameter("initial_pose_covariance_yaw", 0.068)
        self.declare_parameter("initial_pose_publish_hz", 1.0)
        self.declare_parameter("initial_pose_timeout_s", 15.0)
        self.declare_parameter("initial_pose_skip_wait_s", 0.5)
        self.declare_parameter("goal_action_name", "navigate_to_pose")
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("goal_server_timeout_s", 20.0)
        self.declare_parameter("goal_result_timeout_s", 120.0)
        self.declare_parameter("goal_start_delay_s", 2.0)
        self.declare_parameter("goal_reject_retry_count", 5)
        self.declare_parameter("goal_reject_retry_delay_s", 1.0)
        self.declare_parameter("goal_sequence_csv", "")
        self.declare_parameter("goal_sequence_file", "")
        self.declare_parameter("goal_sequence_randomize", True)
        self.declare_parameter("goal_sequence_random_reverse", True)
        self.declare_parameter("goal_sequence_relative_to_current_pose", False)
        self.declare_parameter("goal_sequence_seed", -1)
        self.declare_parameter("pose_timeout_s", 10.0)
        self.declare_parameter("pose_stale_timeout_s", 2.0)
        self.declare_parameter("pause_after_goal_s", 0.0)
        self.declare_parameter("path_topic", "planned_path")
        self.declare_parameter("min_goal_xy_delta_m", 0.05)
        self.declare_parameter("min_goal_yaw_delta_deg", 2.0)
        self.declare_parameter("continue_on_goal_failure", False)

        self.start_delay_s = max(0.0, float(self.get_parameter("start_delay_s").value))
        self.motion_profile = str(self.get_parameter("motion_profile").value)
        self.turn_pattern = str(self.get_parameter("turn_pattern").value)
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        self.forward_time_s = float(self.get_parameter("forward_time_s").value)
        self.turn_speed = float(self.get_parameter("turn_speed").value)
        self.turn_time_s = float(self.get_parameter("turn_time_s").value)
        self.cycles = max(0, int(self.get_parameter("cycles").value))
        self.variation_enable = coerce_bool(self.get_parameter("variation_enable").value)
        self.variation_amplitude = float(self.get_parameter("variation_amplitude").value)
        self.pause_every_n = max(0, int(self.get_parameter("pause_every_n").value))
        self.pause_time_s = max(0.0, float(self.get_parameter("pause_time_s").value))

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.pose_topic_type = str(self.get_parameter("pose_topic_type").value).strip().lower()
        self.set_initial_pose_enable = coerce_bool(self.get_parameter("set_initial_pose_enable").value)
        self.initial_pose_topic = str(self.get_parameter("initial_pose_topic").value)
        self.initial_pose_frame_id = str(self.get_parameter("initial_pose_frame_id").value)
        self.initial_pose_x = float(self.get_parameter("initial_pose_x").value)
        self.initial_pose_y = float(self.get_parameter("initial_pose_y").value)
        self.initial_pose_yaw_deg = float(self.get_parameter("initial_pose_yaw_deg").value)
        self.initial_pose_covariance_xy = max(0.0, float(self.get_parameter("initial_pose_covariance_xy").value))
        self.initial_pose_covariance_yaw = max(0.0, float(self.get_parameter("initial_pose_covariance_yaw").value))
        self.initial_pose_publish_hz = max(0.1, float(self.get_parameter("initial_pose_publish_hz").value))
        self.initial_pose_timeout_s = max(0.0, float(self.get_parameter("initial_pose_timeout_s").value))
        self.initial_pose_skip_wait_s = max(0.0, float(self.get_parameter("initial_pose_skip_wait_s").value))
        self.goal_action_name = str(self.get_parameter("goal_action_name").value)
        self.goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        self.goal_server_timeout_s = max(0.0, float(self.get_parameter("goal_server_timeout_s").value))
        self.goal_result_timeout_s = max(0.0, float(self.get_parameter("goal_result_timeout_s").value))
        self.goal_start_delay_s = max(0.0, float(self.get_parameter("goal_start_delay_s").value))
        self.goal_reject_retry_count = max(0, int(self.get_parameter("goal_reject_retry_count").value))
        self.goal_reject_retry_delay_s = max(0.0, float(self.get_parameter("goal_reject_retry_delay_s").value))
        self.goal_sequence_csv = str(self.get_parameter("goal_sequence_csv").value).strip()
        self.goal_sequence_file = str(self.get_parameter("goal_sequence_file").value).strip()
        self.goal_sequence_randomize = coerce_bool(self.get_parameter("goal_sequence_randomize").value)
        self.goal_sequence_random_reverse = coerce_bool(self.get_parameter("goal_sequence_random_reverse").value)
        self.goal_sequence_relative_to_current_pose = coerce_bool(
            self.get_parameter("goal_sequence_relative_to_current_pose").value
        )
        self.goal_sequence_seed = int(self.get_parameter("goal_sequence_seed").value)
        self.pose_timeout_s = max(0.0, float(self.get_parameter("pose_timeout_s").value))
        self.pose_stale_timeout_s = max(0.0, float(self.get_parameter("pose_stale_timeout_s").value))
        self.pause_after_goal_s = max(0.0, float(self.get_parameter("pause_after_goal_s").value))
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.min_goal_xy_delta_m = max(0.0, float(self.get_parameter("min_goal_xy_delta_m").value))
        self.min_goal_yaw_delta_rad = math.radians(max(0.0, float(self.get_parameter("min_goal_yaw_delta_deg").value)))
        self.continue_on_goal_failure = coerce_bool(self.get_parameter("continue_on_goal_failure").value)

        self._latest_pose: Optional[Pose2DState] = None
        self._active_goal_handle = None
        self._pose_message_count = 0
        self._initial_pose_pub = (
            self.create_publisher(PoseWithCovarianceStamped, self.initial_pose_topic, 10)
            if self.initial_pose_topic
            else None
        )
        self._path_pub = self.create_publisher(Path, self.path_topic, 10) if self.path_topic else None
        self._nav_client = ActionClient(self, NavigateToPose, self.goal_action_name)
        self._amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        if self.pose_topic_type in ("pose", "pose_with_covariance", "posewithcovariancestamped"):
            self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self._on_pose,
                self._amcl_pose_qos,
            )
        elif self.pose_topic_type in ("odom", "odometry"):
            self.create_subscription(Odometry, self.pose_topic, self._on_odom, 10)
        else:
            raise ValueError(
                "pose_topic_type must be one of: pose_with_covariance, pose, odometry"
            )

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self._latest_pose = Pose2DState(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=float(yaw),
            frame_id=msg.header.frame_id,
            stamp_ns=(int(msg.header.stamp.sec) * 1_000_000_000) + int(msg.header.stamp.nanosec),
            received_monotonic_s=time.monotonic(),
        )
        self._pose_message_count += 1

    def _on_odom(self, msg: Odometry) -> None:
        orientation = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
            1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z),
        )
        self._latest_pose = Pose2DState(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=float(yaw),
            frame_id=msg.header.frame_id,
            stamp_ns=(int(msg.header.stamp.sec) * 1_000_000_000) + int(msg.header.stamp.nanosec),
            received_monotonic_s=time.monotonic(),
        )
        self._pose_message_count += 1

    def _build_initial_pose_msg(self) -> PoseWithCovarianceStamped:
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.initial_pose_frame_id
        msg.pose.pose.position.x = float(self.initial_pose_x)
        msg.pose.pose.position.y = float(self.initial_pose_y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(math.radians(self.initial_pose_yaw_deg))
        msg.pose.covariance[0] = self.initial_pose_covariance_xy
        msg.pose.covariance[7] = self.initial_pose_covariance_xy
        msg.pose.covariance[35] = self.initial_pose_covariance_yaw
        return msg

    def _maybe_set_initial_pose(self) -> None:
        if not self.set_initial_pose_enable:
            return
        if self.pose_topic_type not in ("pose", "pose_with_covariance", "posewithcovariancestamped"):
            self.get_logger().info(
                f"Automatic initial pose skipped because pose_topic_type='{self.pose_topic_type}' is not localization-based"
            )
            return
        if self._initial_pose_pub is None:
            self.get_logger().warn("Initial pose auto-set enabled but no initial_pose_topic configured; skipping")
            return

        # If localization is already publishing, do not overwrite it unless the
        # operator explicitly changes the parameters and restarts the node.
        if self.initial_pose_skip_wait_s > 0.0:
            deadline = time.monotonic() + self.initial_pose_skip_wait_s
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._latest_pose is not None:
                    self.get_logger().info(
                        f"Localization already has a pose on '{self.pose_topic}'; skipping automatic initial pose"
                    )
                    return

        initial_pose_msg = self._build_initial_pose_msg()
        start_count = self._pose_message_count
        publish_period_s = 1.0 / self.initial_pose_publish_hz
        deadline = time.monotonic() + self.initial_pose_timeout_s if self.initial_pose_timeout_s > 0.0 else None

        self.get_logger().info(
            "Setting initial pose from driver: "
            f"x={self.initial_pose_x:.2f} y={self.initial_pose_y:.2f} "
            f"yaw={self.initial_pose_yaw_deg:.1f}deg frame={self.initial_pose_frame_id}"
        )

        while rclpy.ok():
            initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
            self._initial_pose_pub.publish(initial_pose_msg)

            wait_until = time.monotonic() + publish_period_s
            while rclpy.ok() and time.monotonic() < wait_until:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._pose_message_count > start_count and self._latest_pose is not None:
                    self.get_logger().info(f"Received localization pose on '{self.pose_topic}' after initial pose publish")
                    return

            if deadline is not None and time.monotonic() > deadline:
                raise RuntimeError(
                    f"Timed out after {self.initial_pose_timeout_s:.1f}s waiting for localization pose "
                    f"on '{self.pose_topic}' after publishing initial pose"
                )

    def _wait_for_pose(self) -> Pose2DState:
        deadline = time.monotonic() + self.pose_timeout_s if self.pose_timeout_s > 0.0 else None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_pose is not None:
                return self._latest_pose
            if deadline is not None and time.monotonic() > deadline:
                raise RuntimeError(
                    f"Timed out waiting for UGV pose on topic '{self.pose_topic}'"
                )

        raise RuntimeError("ROS shutdown while waiting for UGV pose")

    def _ensure_pose_is_fresh(self) -> Pose2DState:
        pose = self._wait_for_pose()
        if self.pose_stale_timeout_s <= 0.0:
            return pose

        # Freshness should be based on when this node actually received a pose
        # message, not on header stamp subtraction. AMCL pose stamps are in sim
        # time, and the node clock may still be on wall time during startup.
        pose_age_s = max(0.0, time.monotonic() - pose.received_monotonic_s)
        if pose_age_s <= self.pose_stale_timeout_s:
            return pose

        raise RuntimeError(
            f"UGV pose on '{self.pose_topic}' is stale ({pose_age_s:.2f}s old)"
        )

    def _wait_for_goal_server(self) -> None:
        if self._nav_client.wait_for_server(timeout_sec=self.goal_server_timeout_s):
            return
        raise RuntimeError(
            f"Timed out waiting for Nav2 action server '{self.goal_action_name}'"
        )

    def _settle_before_goals(self) -> None:
        if self.goal_start_delay_s <= 0.0:
            return

        self.get_logger().info(
            f"Waiting {self.goal_start_delay_s:.1f}s for Nav2 TF/costmaps to settle before sending goals"
        )
        deadline = time.monotonic() + self.goal_start_delay_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)

    def _build_explicit_waypoints(self, start_pose: Pose2DState):
        if not self.goal_sequence_csv:
            return []

        base_waypoints = []
        for raw_entry in self.goal_sequence_csv.split(";"):
            entry = raw_entry.strip()
            if not entry:
                continue
            parts = [part.strip() for part in entry.split(",")]
            if len(parts) != 3:
                raise ValueError(
                    "goal_sequence_csv must use 'x,y,yaw_deg;x,y,yaw_deg' format"
                )

            x_val = float(parts[0])
            y_val = float(parts[1])
            yaw_rad = math.radians(float(parts[2]))

            if self.goal_sequence_relative_to_current_pose:
                dx, dy = rotate_offset(x_val, y_val, start_pose.yaw)
                goal_x = start_pose.x + dx
                goal_y = start_pose.y + dy
                goal_yaw = normalize_angle(start_pose.yaw + yaw_rad)
            else:
                goal_x = x_val
                goal_y = y_val
                goal_yaw = yaw_rad

            base_waypoints.append(
                MotionWaypoint(
                    segment_name="route_goal",
                    x=goal_x,
                    y=goal_y,
                    yaw=goal_yaw,
                    duration_s=0.0,
                )
            )

        waypoints = []
        for _ in range(max(1, self.cycles)):
            for waypoint in base_waypoints:
                waypoints.append(
                    MotionWaypoint(
                        segment_name=waypoint.segment_name,
                        x=waypoint.x,
                        y=waypoint.y,
                        yaw=waypoint.yaw,
                        duration_s=waypoint.duration_s,
                    )
                )
        return waypoints

    def _build_file_waypoints(self, start_pose: Pose2DState):
        if not self.goal_sequence_file:
            return []

        with open(self.goal_sequence_file, "r", encoding="utf-8") as handle:
            data = yaml.safe_load(handle) or {}

        raw_waypoints = data.get("waypoints")
        if not isinstance(raw_waypoints, list) or len(raw_waypoints) < 2:
            raise ValueError("goal_sequence_file must contain at least two waypoints")

        seed = self.goal_sequence_seed if self.goal_sequence_seed >= 0 else int(time.time_ns() & 0xFFFFFFFF)
        rng = random.Random(seed)
        ordered_waypoints = list(raw_waypoints)
        fixed_last_waypoint = ordered_waypoints[-1]
        randomizable_waypoints = ordered_waypoints[:-1]

        if self.goal_sequence_randomize and len(randomizable_waypoints) > 1:
            rotate_by = rng.randrange(len(randomizable_waypoints))
            randomizable_waypoints = randomizable_waypoints[rotate_by:] + randomizable_waypoints[:rotate_by]

        if self.goal_sequence_random_reverse and len(randomizable_waypoints) > 1 and rng.random() < 0.5:
            randomizable_waypoints.reverse()

        ordered_waypoints = randomizable_waypoints + [fixed_last_waypoint]

        base_waypoints = []
        count = len(ordered_waypoints)
        for index, waypoint in enumerate(ordered_waypoints):
            if not isinstance(waypoint, dict):
                raise ValueError("goal_sequence_file waypoints must be mappings with x/y")

            x_val = float(waypoint["x"])
            y_val = float(waypoint["y"])
            yaw_deg = waypoint.get("yaw_deg")

            if yaw_deg is None:
                next_waypoint = ordered_waypoints[(index + 1) % count]
                next_x = float(next_waypoint["x"])
                next_y = float(next_waypoint["y"])
                yaw_rad = math.atan2(next_y - y_val, next_x - x_val)
            else:
                yaw_rad = math.radians(float(yaw_deg))

            if self.goal_sequence_relative_to_current_pose:
                dx, dy = rotate_offset(x_val, y_val, start_pose.yaw)
                goal_x = start_pose.x + dx
                goal_y = start_pose.y + dy
                goal_yaw = normalize_angle(start_pose.yaw + yaw_rad)
            else:
                goal_x = x_val
                goal_y = y_val
                goal_yaw = yaw_rad

            base_waypoints.append(
                MotionWaypoint(
                    segment_name="route_goal",
                    x=goal_x,
                    y=goal_y,
                    yaw=goal_yaw,
                    duration_s=0.0,
                )
            )

        waypoints = []
        for _ in range(max(1, self.cycles)):
            for waypoint in base_waypoints:
                waypoints.append(
                    MotionWaypoint(
                        segment_name=waypoint.segment_name,
                        x=waypoint.x,
                        y=waypoint.y,
                        yaw=waypoint.yaw,
                        duration_s=waypoint.duration_s,
                    )
                )
        return waypoints

    def _build_waypoints(self, start_pose: Pose2DState):
        explicit_waypoints = self._build_explicit_waypoints(start_pose)
        if explicit_waypoints:
            return [], explicit_waypoints

        file_waypoints = self._build_file_waypoints(start_pose)
        if file_waypoints:
            return [], file_waypoints

        route_waypoints = build_route_waypoints(
            self.motion_profile,
            start_pose.x,
            start_pose.y,
            self.cycles,
        )
        if route_waypoints:
            return [], route_waypoints

        segments = build_segments(
            MotionProfileConfig(
                motion_profile=self.motion_profile,
                turn_pattern=self.turn_pattern,
                forward_speed=self.forward_speed,
                forward_time_s=self.forward_time_s,
                turn_speed=self.turn_speed,
                turn_time_s=self.turn_time_s,
                cycles=self.cycles,
                variation_enable=self.variation_enable,
                variation_amplitude=self.variation_amplitude,
                pause_every_n=self.pause_every_n,
                pause_time_s=self.pause_time_s,
            )
        )
        return segments, integrate_segments(start_pose.x, start_pose.y, start_pose.yaw, segments)

    def _filtered_waypoints(self, waypoints):
        filtered = []
        last_goal = None

        for waypoint in waypoints:
            if waypoint.segment_name == "pause":
                filtered.append(waypoint)
                continue

            if last_goal is None:
                filtered.append(waypoint)
                last_goal = waypoint
                continue

            dx = waypoint.x - last_goal.x
            dy = waypoint.y - last_goal.y
            dyaw = normalize_angle(waypoint.yaw - last_goal.yaw)
            if math.hypot(dx, dy) < self.min_goal_xy_delta_m and abs(dyaw) < self.min_goal_yaw_delta_rad:
                continue

            filtered.append(waypoint)
            last_goal = waypoint

        return filtered

    def _publish_path(self, frame_id: str, waypoints) -> None:
        if self._path_pub is None:
            return

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id

        for waypoint in waypoints:
            if waypoint.segment_name == "pause":
                continue

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = float(waypoint.x)
            pose.pose.position.y = float(waypoint.y)
            pose.pose.orientation = yaw_to_quaternion(waypoint.yaw)
            path.poses.append(pose)

        self._path_pub.publish(path)

    def _send_goal(self, frame_id: str, x: float, y: float, yaw: float) -> bool:
        for attempt in range(self.goal_reject_retry_count + 1):
            goal = NavigateToPose.Goal()
            goal.pose.header.stamp = self.get_clock().now().to_msg()
            goal.pose.header.frame_id = frame_id
            goal.pose.pose.position.x = float(x)
            goal.pose.pose.position.y = float(y)
            goal.pose.pose.orientation = yaw_to_quaternion(yaw)

            send_future = self._nav_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                if attempt >= self.goal_reject_retry_count:
                    self.get_logger().error(
                        f"Nav2 rejected goal x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg"
                    )
                    return False

                self.get_logger().warn(
                    f"Nav2 rejected goal x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg; "
                    f"retry {attempt + 1}/{self.goal_reject_retry_count} after {self.goal_reject_retry_delay_s:.1f}s"
                )
                deadline = time.monotonic() + self.goal_reject_retry_delay_s
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                continue

            self._active_goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            timeout = self.goal_result_timeout_s if self.goal_result_timeout_s > 0.0 else None
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
            if not result_future.done():
                self.get_logger().error(
                    f"Nav2 goal timed out after {self.goal_result_timeout_s:.1f}s; cancelling"
                )
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                self._active_goal_handle = None
                return False

            result = result_future.result()
            self._active_goal_handle = None
            if result is None:
                self.get_logger().error("Nav2 goal returned no result")
                return False

            if result.status != GoalStatus.STATUS_SUCCEEDED:
                self.get_logger().error(
                    f"Nav2 goal failed with status={result.status} "
                    f"for x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg"
                )
                return False

            return True

        return False

    def cancel_active_goal(self) -> None:
        if self._active_goal_handle is None:
            return
        cancel_future = self._active_goal_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=2.0)
        self._active_goal_handle = None

    def run(self) -> int:
        if self.start_delay_s > 0.0:
            self.get_logger().info(f"Start delay {self.start_delay_s:.1f}s before Nav2 UGV motion")
            time.sleep(self.start_delay_s)

        self._maybe_set_initial_pose()
        start_pose = self._ensure_pose_is_fresh()
        self._wait_for_goal_server()
        self._settle_before_goals()

        segments, waypoints = self._build_waypoints(start_pose)
        filtered_waypoints = self._filtered_waypoints(waypoints)
        goal_frame_id = self.goal_frame_id or start_pose.frame_id
        if segments:
            est_motion_s = sum(float(segment.duration_s) for segment in segments)
            self.get_logger().info(
                f"UGV Nav2 motion: profile={self.motion_profile} turn_pattern={self.turn_pattern} "
                f"pose_topic={self.pose_topic} action={self.goal_action_name} cycles={self.cycles} "
                f"est_duration~{est_motion_s:.1f}s"
            )
        else:
            self.get_logger().info(
                f"UGV Nav2 motion: profile={self.motion_profile} pose_topic={self.pose_topic} "
                f"action={self.goal_action_name} loops={self.cycles} "
                f"route_mode={'file_waypoints' if self.goal_sequence_file else 'explicit_waypoints'}"
            )
        self.get_logger().info(
            f"Generated {len(filtered_waypoints)} motion waypoints in frame '{goal_frame_id}'"
        )
        self._publish_path(goal_frame_id, filtered_waypoints)

        for index, waypoint in enumerate(filtered_waypoints):
            if waypoint.segment_name == "pause":
                if waypoint.duration_s > 0.0:
                    self.get_logger().info(
                        f"Waypoint {index + 1}/{len(filtered_waypoints)}: pause {waypoint.duration_s:.2f}s"
                    )
                    time.sleep(waypoint.duration_s)
                continue

            self.get_logger().info(
                f"Waypoint {index + 1}/{len(filtered_waypoints)}: "
                f"{waypoint.segment_name} -> x={waypoint.x:.2f} y={waypoint.y:.2f} "
                f"yaw={math.degrees(waypoint.yaw):.1f}deg"
            )
            success = self._send_goal(goal_frame_id, waypoint.x, waypoint.y, waypoint.yaw)
            if not success:
                if self.continue_on_goal_failure:
                    self.get_logger().warn("Continuing to next Nav2 waypoint after failure")
                    continue
                return 2

            if self.pause_after_goal_s > 0.0:
                time.sleep(self.pause_after_goal_s)

        self.get_logger().info("UGV Nav2 motion complete")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvNav2Driver()
    rc = 0
    try:
        rc = node.run()
    except KeyboardInterrupt:
        node.cancel_active_goal()
        rc = 130
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
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
