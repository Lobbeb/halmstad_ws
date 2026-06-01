#!/usr/bin/env python3

import math
import random
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
import yaml
from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from lifecycle_msgs.msg import State
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParameters
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from lrs_halmstad.common.ros_params import yaml_param
from lrs_halmstad.follow.follow_math import coerce_bool
from lrs_halmstad.nav.ugv_motion_profile import (
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


def yaw_from_wxyz(values) -> float:
    if not isinstance(values, (list, tuple)) or len(values) != 4:
        raise ValueError("RViz waypoint orientation must be [w, x, y, z]")
    qw = float(values[0])
    qx = float(values[1])
    qy = float(values[2])
    qz = float(values[3])
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


class UgvNav2Driver(Node):
    def __init__(self) -> None:
        super().__init__("ugv_nav2_driver")

        self.start_delay_s = max(0.0, float(yaml_param(self, "start_delay_s")))
        self.motion_profile = str(yaml_param(self, "motion_profile"))
        self.turn_pattern = str(yaml_param(self, "turn_pattern"))
        self.forward_speed = float(yaml_param(self, "forward_speed"))
        self.forward_time_s = float(yaml_param(self, "forward_time_s"))
        self.turn_speed = float(yaml_param(self, "turn_speed"))
        self.turn_time_s = float(yaml_param(self, "turn_time_s"))
        self.cycles = max(0, int(yaml_param(self, "cycles")))
        self.variation_enable = coerce_bool(yaml_param(self, "variation_enable"))
        self.variation_amplitude = float(yaml_param(self, "variation_amplitude"))
        self.pause_every_n = max(0, int(yaml_param(self, "pause_every_n")))
        self.pause_time_s = max(0.0, float(yaml_param(self, "pause_time_s")))

        self.pose_topic = str(yaml_param(self, "pose_topic"))
        self.pose_topic_type = str(yaml_param(self, "pose_topic_type")).strip().lower()
        self.set_initial_pose_enable = coerce_bool(yaml_param(self, "set_initial_pose_enable"))
        self.initial_pose_topic = str(yaml_param(self, "initial_pose_topic"))
        self.initial_pose_frame_id = str(yaml_param(self, "initial_pose_frame_id"))
        self.initial_pose_x = float(yaml_param(self, "initial_pose_x"))
        self.initial_pose_y = float(yaml_param(self, "initial_pose_y"))
        self.initial_pose_yaw_deg = float(yaml_param(self, "initial_pose_yaw_deg"))
        self.initial_pose_covariance_xy = max(0.0, float(yaml_param(self, "initial_pose_covariance_xy")))
        self.initial_pose_covariance_yaw = max(0.0, float(yaml_param(self, "initial_pose_covariance_yaw")))
        self.initial_pose_publish_hz = max(0.1, float(yaml_param(self, "initial_pose_publish_hz")))
        self.initial_pose_timeout_s = max(0.0, float(yaml_param(self, "initial_pose_timeout_s")))
        self.initial_pose_skip_wait_s = max(0.0, float(yaml_param(self, "initial_pose_skip_wait_s")))
        self.goal_action_name = str(yaml_param(self, "goal_action_name"))
        self.goal_frame_id = str(yaml_param(self, "goal_frame_id"))
        self.goal_server_timeout_s = max(0.0, float(yaml_param(self, "goal_server_timeout_s")))
        self.goal_result_timeout_s = max(0.0, float(yaml_param(self, "goal_result_timeout_s")))
        self.goal_start_delay_s = max(0.0, float(yaml_param(self, "goal_start_delay_s")))
        self.declare_parameter("nav2_required_lifecycle_nodes", ["map_server", "amcl", "controller_server", "bt_navigator"])
        required_lifecycle_nodes = self.get_parameter("nav2_required_lifecycle_nodes").value
        if isinstance(required_lifecycle_nodes, str):
            required_lifecycle_nodes = [part.strip() for part in required_lifecycle_nodes.split(",") if part.strip()]
        self.nav2_required_lifecycle_nodes = [str(node).strip() for node in (required_lifecycle_nodes or []) if str(node).strip()]
        self.goal_reject_retry_count = max(0, int(yaml_param(self, "goal_reject_retry_count")))
        self.goal_reject_retry_delay_s = max(0.0, float(yaml_param(self, "goal_reject_retry_delay_s")))
        self.goal_sequence_csv = str(yaml_param(self, "goal_sequence_csv")).strip()
        self.goal_sequence_file = str(yaml_param(self, "goal_sequence_file")).strip()
        self.goal_sequence_randomize = coerce_bool(yaml_param(self, "goal_sequence_randomize"))
        self.goal_sequence_random_reverse = coerce_bool(yaml_param(self, "goal_sequence_random_reverse"))
        self.goal_sequence_relative_to_current_pose = coerce_bool(
            yaml_param(self, "goal_sequence_relative_to_current_pose")
        )
        self.goal_sequence_seed = int(yaml_param(self, "goal_sequence_seed"))
        self.pose_timeout_s = max(0.0, float(yaml_param(self, "pose_timeout_s")))
        self.pose_stale_timeout_s = max(0.0, float(yaml_param(self, "pose_stale_timeout_s")))
        self.pause_after_goal_s = max(0.0, float(yaml_param(self, "pause_after_goal_s")))
        self.path_topic = str(yaml_param(self, "path_topic"))
        self.min_goal_xy_delta_m = max(0.0, float(yaml_param(self, "min_goal_xy_delta_m")))
        self.min_goal_yaw_delta_rad = math.radians(
            max(0.0, float(yaml_param(self, "min_goal_yaw_delta_deg")))
        )
        self.continue_on_goal_failure = coerce_bool(yaml_param(self, "continue_on_goal_failure"))
        self.lidar_settings_file = str(yaml_param(self, "lidar_settings_file")).strip()
        self.pc2ls_node_name = str(yaml_param(self, "pc2ls_node_name")).strip()
        self.pc2ls_update_timeout_s = max(0.0, float(yaml_param(self, "pc2ls_update_timeout_s")))

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
        self._amcl_pose_volatile_qos = QoSProfile(
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._lifecycle_clients: dict[str, any] = {}
        self._pc2ls_client = None
        self._lidar_settings = self._load_lidar_settings()
        self._pose_subscriptions = []

        if self.pose_topic_type in ("pose", "pose_with_covariance", "posewithcovariancestamped"):
            self._pose_subscriptions.append(self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self._on_pose,
                self._amcl_pose_qos,
            ))
            self._pose_subscriptions.append(self.create_subscription(
                PoseWithCovarianceStamped,
                self.pose_topic,
                self._on_pose,
                self._amcl_pose_volatile_qos,
            ))
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
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0
        msg.header.frame_id = self.initial_pose_frame_id
        msg.pose.pose.position.x = float(self.initial_pose_x)
        msg.pose.pose.position.y = float(self.initial_pose_y)
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = yaw_to_quaternion(math.radians(self.initial_pose_yaw_deg))
        msg.pose.covariance[0] = self.initial_pose_covariance_xy
        msg.pose.covariance[7] = self.initial_pose_covariance_xy
        msg.pose.covariance[35] = self.initial_pose_covariance_yaw
        return msg

    def _initial_pose_state(self) -> Pose2DState:
        return Pose2DState(
            x=float(self.initial_pose_x),
            y=float(self.initial_pose_y),
            yaw=math.radians(self.initial_pose_yaw_deg),
            frame_id=self.initial_pose_frame_id,
            stamp_ns=0,
            received_monotonic_s=time.monotonic(),
        )

    def _maybe_set_initial_pose(self, force_publish: bool = False) -> None:
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

        # If localization is already publishing, do not overwrite it unless this
        # call is explicitly synchronizing after Nav2 lifecycle activation.
        if not force_publish and self.initial_pose_skip_wait_s > 0.0:
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
            initial_pose_msg.header.stamp.sec = 0
            initial_pose_msg.header.stamp.nanosec = 0
            self._initial_pose_pub.publish(initial_pose_msg)

            wait_until = time.monotonic() + publish_period_s
            while rclpy.ok() and time.monotonic() < wait_until:
                rclpy.spin_once(self, timeout_sec=0.1)
                if self._pose_message_count > start_count and self._latest_pose is not None:
                    self.get_logger().info(f"Received localization pose on '{self.pose_topic}' after initial pose publish")
                    return

            if deadline is not None and time.monotonic() > deadline:
                self.get_logger().warn(
                    f"Timed out after {self.initial_pose_timeout_s:.1f}s waiting for localization pose "
                    f"on '{self.pose_topic}' after publishing initial pose; continuing with startup"
                )
                return

    def _wait_for_pose(self) -> Pose2DState:
        deadline = time.monotonic() + self.pose_timeout_s if self.pose_timeout_s > 0.0 else None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_pose is not None:
                return self._latest_pose
            if deadline is not None and time.monotonic() > deadline:
                if self.set_initial_pose_enable and self.pose_topic_type in (
                    "pose",
                    "pose_with_covariance",
                    "posewithcovariancestamped",
                ):
                    self.get_logger().warn(
                        f"Timed out waiting for UGV pose on topic '{self.pose_topic}'; "
                        "using configured initial pose as startup fallback"
                    )
                    return self._initial_pose_state()
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
        wait_chunk_s = self.goal_server_timeout_s if self.goal_server_timeout_s > 0.0 else 5.0

        while rclpy.ok():
            if self._nav_client.wait_for_server(timeout_sec=wait_chunk_s):
                self.get_logger().info(
                    f"Connected to Nav2 action server '{self.goal_action_name}'"
                )
                return

            self.get_logger().warn(
                f"Timed out waiting {wait_chunk_s:.1f}s for Nav2 action server '{self.goal_action_name}'; retrying"
            )

        raise RuntimeError(
            f"ROS shutdown while waiting for Nav2 action server '{self.goal_action_name}'"
        )

    def _lifecycle_client(self, node_name: str):
        client = self._lifecycle_clients.get(node_name)
        if client is not None:
            return client
        client = self.create_client(GetState, f"{node_name}/get_state")
        self._lifecycle_clients[node_name] = client
        return client

    def _wait_for_lifecycle_node_active(self, node_name: str) -> None:
        wait_chunk_s = self.goal_server_timeout_s if self.goal_server_timeout_s > 0.0 else 5.0
        client = self._lifecycle_client(node_name)

        while rclpy.ok():
            if not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn(
                    f"Timed out waiting for Nav2 lifecycle service '{node_name}/get_state'; retrying"
                )
                continue

            future = client.call_async(GetState.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=wait_chunk_s)
            if not future.done():
                self.get_logger().warn(
                    f"Timed out waiting {wait_chunk_s:.1f}s for Nav2 lifecycle state from '{node_name}'; retrying"
                )
                continue

            response = future.result()
            if response is None:
                self.get_logger().warn(
                    f"Nav2 lifecycle service '{node_name}/get_state' returned no state; retrying"
                )
                continue

            state = response.current_state
            if state.id == State.PRIMARY_STATE_ACTIVE:
                self.get_logger().info(f"Confirmed Nav2 lifecycle node '{node_name}' is active")
                return

            self.get_logger().warn(
                f"Nav2 lifecycle node '{node_name}' is '{state.label}' ({state.id}); waiting for active"
            )
            deadline = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)

        raise RuntimeError(f"ROS shutdown while waiting for Nav2 lifecycle node '{node_name}' to become active")

    def _wait_for_nav2_active(self) -> None:
        for node_name in self.nav2_required_lifecycle_nodes:
            self._wait_for_lifecycle_node_active(node_name)

    def _default_lidar_settings_file(self) -> str:
        try:
            share_dir = get_package_share_directory("lrs_halmstad")
        except PackageNotFoundError:
            return ""
        candidate = f"{share_dir}/config/baylands_route_lidar.yaml"
        return candidate

    def _load_lidar_settings(self) -> dict:
        if self.lidar_settings_file.lower() in ("none", "false", "disabled"):
            return {}

        settings_file = self.lidar_settings_file or self._default_lidar_settings_file()
        if not settings_file:
            return {}

        try:
            with open(settings_file, "r", encoding="utf-8") as handle:
                data = yaml.safe_load(handle) or {}
        except FileNotFoundError:
            self.get_logger().warn(f"Lidar settings file not found: {settings_file}")
            return {}
        except Exception as exc:
            self.get_logger().warn(f"Failed to read lidar settings file '{settings_file}': {exc}")
            return {}

        waypoints = data.get("waypoints") or {}
        if not isinstance(waypoints, dict):
            self.get_logger().warn(f"Ignoring malformed waypoint lidar settings in {settings_file}")
            return {}

        self.get_logger().info(f"Loaded waypoint lidar settings from {settings_file}")
        return waypoints

    def _pc2ls_param_name(self, config_key: str) -> str:
        if config_key.startswith("pc2ls_"):
            return config_key[len("pc2ls_") :]
        return config_key

    def _pc2ls_set_parameters_client(self):
        if self._pc2ls_client is not None:
            return self._pc2ls_client
        if not self.pc2ls_node_name:
            return None
        self._pc2ls_client = self.create_client(SetParameters, f"{self.pc2ls_node_name}/set_parameters")
        return self._pc2ls_client

    def _apply_waypoint_lidar_settings(self, waypoint_name: str) -> None:
        if not waypoint_name or waypoint_name == "route_goal":
            return
        raw_settings = self._lidar_settings.get(waypoint_name)
        if not raw_settings:
            return
        if not isinstance(raw_settings, dict):
            self.get_logger().warn(f"Ignoring malformed lidar settings for waypoint '{waypoint_name}'")
            return

        supported_keys = {
            "pc2ls_min_height",
            "pc2ls_max_height",
            "min_height",
            "max_height",
        }
        params = []
        log_parts = []
        for key, value in raw_settings.items():
            if key not in supported_keys:
                continue
            param_name = self._pc2ls_param_name(str(key))
            param_value = float(value)
            params.append(
                Parameter(
                    name=param_name,
                    value=ParameterValue(
                        type=ParameterType.PARAMETER_DOUBLE,
                        double_value=param_value,
                    ),
                )
            )
            log_parts.append(f"{param_name}={param_value:g}")

        if not params:
            return

        client = self._pc2ls_set_parameters_client()
        if client is None:
            self.get_logger().warn(
                f"Waypoint '{waypoint_name}' has lidar settings but pc2ls_node_name is empty"
            )
            return
        wait_deadline = time.monotonic() + self.pc2ls_update_timeout_s
        while rclpy.ok():
            wait_timeout = min(0.5, max(0.0, wait_deadline - time.monotonic()))
            if client.wait_for_service(timeout_sec=wait_timeout):
                break
            if time.monotonic() >= wait_deadline:
                self.get_logger().warn(
                    f"Could not update lidar settings for waypoint '{waypoint_name}': "
                    f"{self.pc2ls_node_name}/set_parameters unavailable"
                )
                return
            self.get_logger().warn(
                f"Waiting for {self.pc2ls_node_name}/set_parameters before applying lidar settings for '{waypoint_name}'"
            )
            rclpy.spin_once(self, timeout_sec=0.1)

        if not rclpy.ok():
            self.get_logger().warn(
                f"Could not update lidar settings for waypoint '{waypoint_name}': "
                "ROS shutdown"
            )
            return

        request = SetParameters.Request()
        request.parameters = params
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.pc2ls_update_timeout_s)
        if not future.done() or future.result() is None:
            self.get_logger().warn(f"Timed out updating lidar settings for waypoint '{waypoint_name}'")
            return

        failed = [result.reason for result in future.result().results if not result.successful]
        if failed:
            self.get_logger().warn(
                f"Failed updating lidar settings for waypoint '{waypoint_name}': {'; '.join(failed)}"
            )
            return

        self.get_logger().info(
            f"Applied lidar settings at waypoint '{waypoint_name}': {', '.join(log_parts)}"
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

        file_frame_id = str(data.get("frame_id") or "").strip()
        goal_sequence_relative = self.goal_sequence_relative_to_current_pose
        if file_frame_id:
            normalized_file_frame = file_frame_id.lstrip("/")
            normalized_goal_frame = self.goal_frame_id.lstrip("/")
            if normalized_file_frame in {"map", "odom"} or (
                normalized_goal_frame and normalized_file_frame == normalized_goal_frame
            ):
                if goal_sequence_relative:
                    self.get_logger().warn(
                        "goal_sequence_relative_to_current_pose=true was requested, but "
                        f"goal_sequence_file '{self.goal_sequence_file}' declares frame_id='{file_frame_id}'. "
                        "Treating these file waypoints as absolute goals."
                    )
                goal_sequence_relative = False

        raw_waypoints = data.get("waypoints")
        preserve_file_order = "source_start_waypoint" in data or "source_start_index" in data
        if isinstance(raw_waypoints, dict):
            ordered_waypoints = []
            for name, waypoint in raw_waypoints.items():
                if not isinstance(waypoint, dict):
                    raise ValueError("RViz goal_sequence_file waypoints must be mappings")
                pose = waypoint.get("pose")
                if not isinstance(pose, (list, tuple)) or len(pose) < 2:
                    raise ValueError("RViz goal_sequence_file waypoints must contain pose [x, y, z]")
                parsed_waypoint = {
                    "name": str(name),
                    "x": float(pose[0]),
                    "y": float(pose[1]),
                }
                orientation = waypoint.get("orientation")
                if orientation is not None:
                    parsed_waypoint["yaw_rad"] = yaw_from_wxyz(orientation)
                ordered_waypoints.append(parsed_waypoint)
        elif isinstance(raw_waypoints, list):
            ordered_waypoints = list(raw_waypoints)
        else:
            raise ValueError("goal_sequence_file must contain a waypoint list or RViz waypoint mapping")

        if len(ordered_waypoints) < 1:
            raise ValueError("goal_sequence_file must contain at least one waypoint")

        seed = self.goal_sequence_seed if self.goal_sequence_seed >= 0 else int(time.time_ns() & 0xFFFFFFFF)
        rng = random.Random(seed)
        fixed_last_waypoint = ordered_waypoints[-1]
        randomizable_waypoints = ordered_waypoints[:-1]

        if preserve_file_order:
            self.get_logger().info(
                f"Preserving sliced waypoint file order from '{self.goal_sequence_file}'"
            )
        elif len(randomizable_waypoints) > 1:
            randomizable_waypoints = self._reorder_file_waypoints_for_forward_start(
                randomizable_waypoints,
                start_pose,
                rng,
            )

        ordered_waypoints = randomizable_waypoints + [fixed_last_waypoint]

        base_waypoints = []
        count = len(ordered_waypoints)
        for index, waypoint in enumerate(ordered_waypoints):
            if not isinstance(waypoint, dict):
                raise ValueError("goal_sequence_file waypoints must be mappings with x/y")

            x_val = float(waypoint["x"])
            y_val = float(waypoint["y"])
            yaw_rad = waypoint.get("yaw_rad")
            yaw_deg = waypoint.get("yaw_deg")

            if yaw_rad is not None:
                yaw_rad = float(yaw_rad)
            elif yaw_deg is None:
                next_waypoint = ordered_waypoints[(index + 1) % count]
                next_x = float(next_waypoint["x"])
                next_y = float(next_waypoint["y"])
                yaw_rad = math.atan2(next_y - y_val, next_x - x_val)
            else:
                yaw_rad = math.radians(float(yaw_deg))

            if goal_sequence_relative:
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
                    segment_name=str(waypoint.get("name") or "route_goal"),
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

    def _reorder_file_waypoints_for_forward_start(
        self,
        randomizable_waypoints,
        start_pose: Pose2DState,
        rng: random.Random,
    ):
        if len(randomizable_waypoints) <= 1:
            return list(randomizable_waypoints)

        start_indices = self._forward_start_indices(randomizable_waypoints, start_pose)
        if start_indices:
            if self.goal_sequence_randomize:
                rotate_by = rng.choice(start_indices)
            else:
                rotate_by = start_indices[0]
        elif self.goal_sequence_randomize:
            rotate_by = rng.randrange(len(randomizable_waypoints))
        else:
            rotate_by = 0

        ordered = list(randomizable_waypoints[rotate_by:]) + list(randomizable_waypoints[:rotate_by])

        if self.goal_sequence_random_reverse and len(ordered) > 1 and rng.random() < 0.5:
            ordered = [ordered[0]] + list(reversed(ordered[1:]))

        return ordered

    def _forward_start_indices(self, waypoints, start_pose: Pose2DState):
        heading_x = math.cos(start_pose.yaw)
        heading_y = math.sin(start_pose.yaw)
        forward_indices = []

        for index, waypoint in enumerate(waypoints):
            if not isinstance(waypoint, dict):
                continue
            dx = float(waypoint["x"]) - start_pose.x
            dy = float(waypoint["y"]) - start_pose.y
            forward_projection = dx * heading_x + dy * heading_y
            if forward_projection > 0.0:
                forward_indices.append(index)

        return forward_indices

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

    def _filtered_waypoints(self, waypoints, start_pose: Optional[Pose2DState] = None):
        filtered = []
        last_goal = None

        for waypoint in waypoints:
            if waypoint.segment_name == "pause":
                filtered.append(waypoint)
                continue

            if last_goal is None:
                if start_pose is not None:
                    dx = waypoint.x - start_pose.x
                    dy = waypoint.y - start_pose.y
                    dyaw = normalize_angle(waypoint.yaw - start_pose.yaw)
                    if math.hypot(dx, dy) < self.min_goal_xy_delta_m and abs(dyaw) < self.min_goal_yaw_delta_rad:
                        self.get_logger().info(
                            f"Skipping first waypoint '{waypoint.segment_name}' because the UGV is already there"
                        )
                        last_goal = waypoint
                        continue
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
        # Keep the debug path in "latest transform" time so RViz and Nav2
        # inspection stay aligned with whichever map->odom transform is
        # currently available.
        path.header.stamp.sec = 0
        path.header.stamp.nanosec = 0
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
            # Use zero-time goals so Nav2 transforms them with the latest
            # available map->odom TF instead of requiring an exact future stamp.
            goal.pose.header.stamp.sec = 0
            goal.pose.header.stamp.nanosec = 0
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
                self._wait_for_nav2_active()
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
                nav2_result = getattr(result, "result", None)
                error_code = getattr(nav2_result, "error_code", None)
                error_msg = getattr(nav2_result, "error_msg", "")
                detail = ""
                if error_code is not None:
                    detail = f" error_code={error_code}"
                if error_msg:
                    detail = f"{detail} error_msg='{error_msg}'"
                self.get_logger().error(
                    f"Nav2 goal failed with status={result.status} "
                    f"for x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg{detail}"
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

        self._wait_for_goal_server()
        self._wait_for_nav2_active()
        self._maybe_set_initial_pose(force_publish=True)
        start_pose = self._ensure_pose_is_fresh()
        self._settle_before_goals()

        segments, waypoints = self._build_waypoints(start_pose)
        filtered_waypoints = self._filtered_waypoints(waypoints, start_pose)
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

            self._apply_waypoint_lidar_settings(waypoint.segment_name)

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
