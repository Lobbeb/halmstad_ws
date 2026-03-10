import math
from dataclasses import dataclass
from typing import Tuple


@dataclass
class Pose2D:
    x: float
    y: float
    yaw: float


def coerce_bool(value) -> bool:
    if isinstance(value, bool):
        return value
    if value is None:
        return False
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in ("", "0", "false", "no", "off", "n", "f"):
            return False
        if normalized in ("1", "true", "yes", "on", "y", "t"):
            return True
    return bool(value)


def quat_from_yaw(yaw: float) -> Tuple[float, float, float, float]:
    half = 0.5 * float(yaw)
    return (0.0, 0.0, math.sin(half), math.cos(half))


def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    return (float(a) + math.pi) % (2.0 * math.pi) - math.pi


def clamp_mag(x: float, y: float, max_norm: float) -> Tuple[float, float]:
    n = math.hypot(x, y)
    if n <= max_norm or n < 1e-9:
        return x, y
    s = max_norm / n
    return x * s, y * s


def clamp_point_to_radius(center_x: float, center_y: float, x: float, y: float, r: float) -> Tuple[float, float]:
    dx = x - center_x
    dy = y - center_y
    d = math.hypot(dx, dy)
    if d <= r or d < 1e-9:
        return x, y
    s = r / d
    return center_x + dx * s, center_y + dy * s


def rotate_body_offset(x_body: float, y_body: float, yaw: float) -> Tuple[float, float]:
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (c * x_body - s * y_body, s * x_body + c * y_body)


def compute_leader_look_target(
    leader_x: float,
    leader_y: float,
    leader_yaw: float,
    leader_z: float,
    leader_look_target_x_m: float = 0.0,
    leader_look_target_y_m: float = 0.0,
    leader_look_target_z_m: float = 0.0,
) -> Tuple[float, float, float]:
    offset_x, offset_y = rotate_body_offset(
        leader_look_target_x_m,
        leader_look_target_y_m,
        leader_yaw,
    )
    return (
        leader_x + offset_x,
        leader_y + offset_y,
        leader_z + leader_look_target_z_m,
    )


def camera_xy_from_uav_pose(
    uav_x: float,
    uav_y: float,
    uav_yaw: float,
    camera_x_offset_m: float,
    camera_y_offset_m: float = 0.0,
) -> Tuple[float, float]:
    off_x, off_y = rotate_body_offset(camera_x_offset_m, camera_y_offset_m, uav_yaw)
    return (uav_x + off_x, uav_y + off_y)


def solve_yaw_to_target(
    uav_x: float,
    uav_y: float,
    target_x: float,
    target_y: float,
    camera_x_offset_m: float,
    camera_y_offset_m: float = 0.0,
) -> float:
    yaw = math.atan2(target_y - uav_y, target_x - uav_x)
    for _ in range(3):
        cam_x, cam_y = camera_xy_from_uav_pose(
            uav_x,
            uav_y,
            yaw,
            camera_x_offset_m,
            camera_y_offset_m,
        )
        dx = target_x - cam_x
        dy = target_y - cam_y
        if math.hypot(dx, dy) <= 1e-9:
            break
        yaw = math.atan2(dy, dx)
    return yaw


def compute_camera_tilt_deg(
    uav_x: float,
    uav_y: float,
    uav_z: float,
    uav_yaw: float,
    target_x: float,
    target_y: float,
    target_z: float,
    camera_x_offset_m: float,
    camera_y_offset_m: float = 0.0,
    camera_z_offset_m: float = 0.0,
    camera_look_target_z_m: float = 0.0,
) -> float:
    camera_x, camera_y = camera_xy_from_uav_pose(
        uav_x,
        uav_y,
        uav_yaw,
        camera_x_offset_m,
        camera_y_offset_m,
    )
    horizontal_distance = max(math.hypot(target_x - camera_x, target_y - camera_y), 1e-3)
    camera_z = uav_z - camera_z_offset_m
    vertical_drop = max(0.0, camera_z - (target_z + camera_look_target_z_m))
    tilt_deg = -math.degrees(math.atan2(vertical_drop, horizontal_distance))
    return max(-89.0, min(89.0, tilt_deg))
