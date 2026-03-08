#!/usr/bin/env python3

import math
import time
from dataclasses import dataclass
from typing import List, Optional

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Path
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from .ugv_motion_profile import normalize_angle


@dataclass
class Pose2DState:
    x: float
    y: float
    yaw: float
    frame_id: str


@dataclass
class GoalSpec:
    x: float
    y: float
    yaw: float


def yaw_to_quaternion(yaw: float) -> Quaternion:
    quaternion = Quaternion()
    quaternion.z = math.sin(0.5 * yaw)
    quaternion.w = math.cos(0.5 * yaw)
    return quaternion


def quaternion_to_yaw(quaternion: Quaternion) -> float:
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z),
    )


def rotate_offset(x_local: float, y_local: float, yaw: float) -> tuple[float, float]:
    return (
        x_local * math.cos(yaw) - y_local * math.sin(yaw),
        x_local * math.sin(yaw) + y_local * math.cos(yaw),
    )


class UgvNav2GoalTester(Node):
    def __init__(self) -> None:
        super().__init__("ugv_nav2_goal_tester")

        self.declare_parameter("pose_topic", "amcl_pose")
        self.declare_parameter("goal_action_name", "navigate_to_pose")
        self.declare_parameter("goal_frame_id", "map")
        self.declare_parameter("goal_server_timeout_s", 20.0)
        self.declare_parameter("goal_result_timeout_s", 120.0)
        self.declare_parameter("goal_start_delay_s", 2.0)
        self.declare_parameter("goal_reject_retry_count", 5)
        self.declare_parameter("goal_reject_retry_delay_s", 1.0)
        self.declare_parameter("pose_timeout_s", 10.0)
        self.declare_parameter("path_topic", "test_goal_path")
        self.declare_parameter("pattern", "square")
        self.declare_parameter("pattern_size_m", 2.0)
        self.declare_parameter("loop_count", 1)
        self.declare_parameter("pause_after_goal_s", 0.0)
        self.declare_parameter("relative_to_current_pose", True)
        self.declare_parameter("goal_sequence_csv", "")
        self.declare_parameter("line_return_enable", True)

        self.pose_topic = str(self.get_parameter("pose_topic").value)
        self.goal_action_name = str(self.get_parameter("goal_action_name").value)
        self.goal_frame_id = str(self.get_parameter("goal_frame_id").value)
        self.goal_server_timeout_s = max(0.0, float(self.get_parameter("goal_server_timeout_s").value))
        self.goal_result_timeout_s = max(0.0, float(self.get_parameter("goal_result_timeout_s").value))
        self.goal_start_delay_s = max(0.0, float(self.get_parameter("goal_start_delay_s").value))
        self.goal_reject_retry_count = max(0, int(self.get_parameter("goal_reject_retry_count").value))
        self.goal_reject_retry_delay_s = max(0.0, float(self.get_parameter("goal_reject_retry_delay_s").value))
        self.pose_timeout_s = max(0.0, float(self.get_parameter("pose_timeout_s").value))
        self.path_topic = str(self.get_parameter("path_topic").value)
        self.pattern = str(self.get_parameter("pattern").value).strip().lower()
        self.pattern_size_m = max(0.1, float(self.get_parameter("pattern_size_m").value))
        self.loop_count = max(1, int(self.get_parameter("loop_count").value))
        self.pause_after_goal_s = max(0.0, float(self.get_parameter("pause_after_goal_s").value))
        self.relative_to_current_pose = bool(self.get_parameter("relative_to_current_pose").value)
        self.goal_sequence_csv = str(self.get_parameter("goal_sequence_csv").value).strip()
        self.line_return_enable = bool(self.get_parameter("line_return_enable").value)

        self._latest_pose: Optional[Pose2DState] = None
        self._amcl_pose_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self._pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, self.pose_topic, self._on_pose, self._amcl_pose_qos
        )
        self._nav_client = ActionClient(self, NavigateToPose, self.goal_action_name)
        self._path_pub = self.create_publisher(Path, self.path_topic, 10) if self.path_topic else None

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_pose = Pose2DState(
            x=float(msg.pose.pose.position.x),
            y=float(msg.pose.pose.position.y),
            yaw=quaternion_to_yaw(msg.pose.pose.orientation),
            frame_id=msg.header.frame_id,
        )

    def _wait_for_pose(self) -> Pose2DState:
        deadline = time.monotonic() + self.pose_timeout_s if self.pose_timeout_s > 0.0 else None
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if self._latest_pose is not None:
                return self._latest_pose
            if deadline is not None and time.monotonic() > deadline:
                raise RuntimeError(f"Timed out waiting for pose on '{self.pose_topic}'")
        raise RuntimeError("ROS shutdown while waiting for UGV pose")

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

    def _parse_goal_sequence(self, base_pose: Pose2DState) -> List[GoalSpec]:
        goals: List[GoalSpec] = []
        if not self.goal_sequence_csv:
            return goals

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
            yaw_deg = float(parts[2])
            yaw_rad = math.radians(yaw_deg)

            if self.relative_to_current_pose:
                dx, dy = rotate_offset(x_val, y_val, base_pose.yaw)
                goals.append(
                    GoalSpec(
                        x=base_pose.x + dx,
                        y=base_pose.y + dy,
                        yaw=normalize_angle(base_pose.yaw + yaw_rad),
                    )
                )
            else:
                goals.append(GoalSpec(x=x_val, y=y_val, yaw=yaw_rad))

        return goals

    def _build_pattern_goals(self, base_pose: Pose2DState) -> List[GoalSpec]:
        if self.pattern == "square":
            local_goals = [
                (self.pattern_size_m, 0.0, 0.0),
                (self.pattern_size_m, self.pattern_size_m, 90.0),
                (0.0, self.pattern_size_m, 180.0),
                (0.0, 0.0, -90.0),
            ]
        elif self.pattern == "line":
            local_goals = [(self.pattern_size_m, 0.0, 0.0)]
            if self.line_return_enable:
                local_goals.append((0.0, 0.0, 180.0))
        else:
            raise ValueError("pattern must be one of: square, line")

        goals: List[GoalSpec] = []
        for x_local, y_local, yaw_deg in local_goals:
            if self.relative_to_current_pose:
                dx, dy = rotate_offset(x_local, y_local, base_pose.yaw)
                goals.append(
                    GoalSpec(
                        x=base_pose.x + dx,
                        y=base_pose.y + dy,
                        yaw=normalize_angle(base_pose.yaw + math.radians(yaw_deg)),
                    )
                )
            else:
                goals.append(
                    GoalSpec(
                        x=x_local,
                        y=y_local,
                        yaw=math.radians(yaw_deg),
                    )
                )
        return goals

    def _build_goals(self, base_pose: Pose2DState) -> List[GoalSpec]:
        goals = self._parse_goal_sequence(base_pose)
        if not goals:
            goals = self._build_pattern_goals(base_pose)
        return goals * self.loop_count

    def _publish_path(self, frame_id: str, goals: List[GoalSpec]) -> None:
        if self._path_pub is None:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = frame_id
        for goal in goals:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = goal.x
            pose.pose.position.y = goal.y
            pose.pose.orientation = yaw_to_quaternion(goal.yaw)
            path.poses.append(pose)
        self._path_pub.publish(path)

    def _send_goal(self, goal: GoalSpec, frame_id: str, index: int, total: int) -> bool:
        self.get_logger().info(
            f"Sending goal {index}/{total}: x={goal.x:.2f} y={goal.y:.2f} "
            f"yaw={math.degrees(goal.yaw):.1f}deg"
        )

        for attempt in range(self.goal_reject_retry_count + 1):
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.header.frame_id = frame_id
            goal_msg.pose.pose.position.x = goal.x
            goal_msg.pose.pose.position.y = goal.y
            goal_msg.pose.pose.orientation = yaw_to_quaternion(goal.yaw)

            send_future = self._nav_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_future)
            goal_handle = send_future.result()
            if goal_handle is None or not goal_handle.accepted:
                if attempt >= self.goal_reject_retry_count:
                    self.get_logger().error(f"Nav2 rejected goal {index}/{total}")
                    return False

                self.get_logger().warn(
                    f"Nav2 rejected goal {index}/{total}; "
                    f"retry {attempt + 1}/{self.goal_reject_retry_count} after {self.goal_reject_retry_delay_s:.1f}s"
                )
                deadline = time.monotonic() + self.goal_reject_retry_delay_s
                while rclpy.ok() and time.monotonic() < deadline:
                    rclpy.spin_once(self, timeout_sec=0.1)
                continue

            result_future = goal_handle.get_result_async()
            timeout = self.goal_result_timeout_s if self.goal_result_timeout_s > 0.0 else None
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=timeout)
            if not result_future.done():
                self.get_logger().error(
                    f"Nav2 goal {index}/{total} timed out after {self.goal_result_timeout_s:.1f}s"
                )
                cancel_future = goal_handle.cancel_goal_async()
                rclpy.spin_until_future_complete(self, cancel_future, timeout_sec=5.0)
                return False

            result = result_future.result()
            if result is None or result.status != GoalStatus.STATUS_SUCCEEDED:
                status = "none" if result is None else str(result.status)
                self.get_logger().error(f"Nav2 goal {index}/{total} failed with status={status}")
                return False
            return True

        return False

    def run(self) -> int:
        base_pose = self._wait_for_pose()
        self._wait_for_goal_server()
        self._settle_before_goals()
        goal_frame_id = self.goal_frame_id or base_pose.frame_id
        goals = self._build_goals(base_pose)
        self._publish_path(goal_frame_id, goals)

        self.get_logger().info(
            f"Nav2 goal test: pattern={self.pattern} loop_count={self.loop_count} "
            f"goals={len(goals)} frame={goal_frame_id}"
        )

        for index, goal in enumerate(goals, start=1):
            if not self._send_goal(goal, goal_frame_id, index, len(goals)):
                return 2
            if self.pause_after_goal_s > 0.0:
                time.sleep(self.pause_after_goal_s)

        self.get_logger().info("Nav2 goal test complete")
        return 0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = UgvNav2GoalTester()
    rc = 0
    try:
        rc = node.run()
    except KeyboardInterrupt:
        rc = 130
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(rc)


if __name__ == "__main__":
    main()
