from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node

from ros_gz_interfaces.srv import SetEntityPose
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header


@dataclass
class PoseRPY:
    x: float
    y: float
    z: float
    roll: float
    pitch: float
    yaw: float


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> Quaternion:
    """Convert RPY to quaternion (ROS standard)."""
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class GzSetPoseClient(Node):
    """
    Thin client for /world/<world>/set_pose using ros_gz_interfaces/srv/SetEntityPose.
    """

    def __init__(self, world: str):
        super().__init__("gz_set_pose_client")
        self._world = world
        self._srv_name = f"/world/{world}/set_pose"
        self._client = self.create_client(SetEntityPose, self._srv_name)

    @property
    def srv_name(self) -> str:
        return self._srv_name

    def wait(self, timeout_s: float = 10.0) -> bool:
        return self._client.wait_for_service(timeout_sec=timeout_s)

    def set_pose(
        self,
        entity_name: str,
        pose_rpy: PoseRPY,
        frame_id: str = "world",
    ) -> bool:
        req = SetEntityPose.Request()
        req.entity.name = entity_name

        req.pose.header = Header()
        req.pose.header.frame_id = frame_id
        req.pose.pose = Pose()
        req.pose.pose.position = Point(x=pose_rpy.x, y=pose_rpy.y, z=pose_rpy.z)
        req.pose.pose.orientation = quaternion_from_euler(
            pose_rpy.roll, pose_rpy.pitch, pose_rpy.yaw
        )

        future = self._client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is None:
            self.get_logger().error("SetEntityPose call failed or timed out")
            return False

        # SetEntityPose returns empty response; success is inferred by transport completion
        return True


def grid_points(
    x_min: float, x_max: float, x_steps: int,
    y_min: float, y_max: float, y_steps: int,
) -> list[Tuple[float, float]]:
    if x_steps < 2 or y_steps < 2:
        raise ValueError("x_steps and y_steps must be >= 2")

    xs = [x_min + i * (x_max - x_min) / (x_steps - 1) for i in range(x_steps)]
    ys = [y_min + j * (y_max - y_min) / (y_steps - 1) for j in range(y_steps)]

    pts: list[Tuple[float, float]] = []
    for y in ys:
        for x in xs:
            pts.append((x, y))
    return pts
