"""Simulation-only UAV localization for timestamped camera projection.

The node consumes only the UAV simulator's Gazebo-world pose contracts.  It
does not subscribe to UGV localization, odometry, pose, or ground truth.
"""

from collections import deque
from dataclasses import dataclass
import csv
import math
from statistics import median
from typing import Iterable, Sequence

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


@dataclass(frozen=True)
class CalibrationPoint:
    place: str
    world_x: float
    world_y: float
    world_z: float
    map_x: float
    map_y: float


@dataclass(frozen=True)
class WorldMapTransform:
    yaw_rad: float
    translation_x_m: float
    translation_y_m: float
    translation_z_m: float
    point_count: int
    max_fit_error_m: float
    validation_error_m: float | None


def normalize_frame_id(frame_id: str) -> str:
    """Return a tf2-compatible frame ID while preserving its hierarchy."""
    normalized = str(frame_id).strip().lstrip('/')
    if not normalized:
        raise ValueError('frame ID must not be empty')
    if normalized.endswith('/') or '//' in normalized:
        raise ValueError(f'invalid frame ID: {frame_id!r}')
    return normalized


def quaternion_from_euler(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def quaternion_multiply(
    left: Sequence[float], right: Sequence[float]
) -> tuple[float, float, float, float]:
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def normalized_quaternion(values: Sequence[float]) -> tuple[float, float, float, float]:
    norm = math.sqrt(sum(float(value) ** 2 for value in values))
    if not math.isfinite(norm) or norm <= 1.0e-12:
        raise ValueError('quaternion must be finite and nonzero')
    return tuple(float(value) / norm for value in values)


def quaternion_inverse(values: Sequence[float]) -> tuple[float, float, float, float]:
    x, y, z, w = normalized_quaternion(values)
    return (-x, -y, -z, w)


def rotate_vector(values: Sequence[float], vector: Sequence[float]) -> tuple[float, float, float]:
    quaternion = normalized_quaternion(values)
    pure = (float(vector[0]), float(vector[1]), float(vector[2]), 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, pure), quaternion_inverse(quaternion)
    )
    return rotated[0], rotated[1], rotated[2]


def load_calibration_points(csv_path: str, group: str) -> list[CalibrationPoint]:
    points: list[CalibrationPoint] = []
    with open(csv_path, newline='', encoding='utf-8') as csv_file:
        for row in csv.DictReader(csv_file):
            if row.get('group', '').strip() != group:
                continue
            points.append(CalibrationPoint(
                place=row['place'].strip(),
                world_x=float(row['x']),
                world_y=float(row['y']),
                world_z=float(row['z']),
                map_x=float(row['amcl_x']),
                map_y=float(row['amcl_y']),
            ))
    return points


def fit_world_to_map(
    points: Iterable[CalibrationPoint],
    *,
    map_z_m: float = 0.0,
    validation_place: str = '',
) -> WorldMapTransform:
    """Fit a planar rigid Gazebo-world to Nav2-map transform.

    The optional validation point is held out from the fit, so its error is an
    independent check against the stored simulation/map measurement.
    """
    all_points = list(points)
    validation = [point for point in all_points if point.place == validation_place]
    fit_points = [point for point in all_points if point.place != validation_place]
    if not validation_place:
        fit_points = all_points
    if len(fit_points) < 2:
        raise ValueError('at least two calibration points are required')
    if validation_place and len(validation) != 1:
        raise ValueError(f'expected one validation point named {validation_place!r}')

    world_mean_x = sum(point.world_x for point in fit_points) / len(fit_points)
    world_mean_y = sum(point.world_y for point in fit_points) / len(fit_points)
    map_mean_x = sum(point.map_x for point in fit_points) / len(fit_points)
    map_mean_y = sum(point.map_y for point in fit_points) / len(fit_points)
    dot = 0.0
    cross = 0.0
    for point in fit_points:
        world_x = point.world_x - world_mean_x
        world_y = point.world_y - world_mean_y
        map_x = point.map_x - map_mean_x
        map_y = point.map_y - map_mean_y
        dot += world_x * map_x + world_y * map_y
        cross += world_x * map_y - world_y * map_x
    if abs(dot) + abs(cross) <= 1.0e-12:
        raise ValueError('calibration points do not constrain a planar transform')

    yaw = math.atan2(cross, dot)
    cosine = math.cos(yaw)
    sine = math.sin(yaw)
    translation_x = map_mean_x - (cosine * world_mean_x - sine * world_mean_y)
    translation_y = map_mean_y - (sine * world_mean_x + cosine * world_mean_y)
    translation_z = float(map_z_m) - median(point.world_z for point in fit_points)

    def error(point: CalibrationPoint) -> float:
        map_x, map_y, _ = world_point_to_map(
            (point.world_x, point.world_y, point.world_z),
            yaw,
            (translation_x, translation_y, translation_z),
        )
        return math.hypot(map_x - point.map_x, map_y - point.map_y)

    fit_errors = [error(point) for point in fit_points]
    validation_error = error(validation[0]) if validation else None
    return WorldMapTransform(
        yaw_rad=yaw,
        translation_x_m=translation_x,
        translation_y_m=translation_y,
        translation_z_m=translation_z,
        point_count=len(fit_points),
        max_fit_error_m=max(fit_errors),
        validation_error_m=validation_error,
    )


def world_point_to_map(
    point: Sequence[float],
    yaw_rad: float,
    translation: Sequence[float],
) -> tuple[float, float, float]:
    cosine = math.cos(yaw_rad)
    sine = math.sin(yaw_rad)
    return (
        cosine * point[0] - sine * point[1] + translation[0],
        sine * point[0] + cosine * point[1] + translation[1],
        point[2] + translation[2],
    )


def stamp_nanoseconds(message: PoseStamped) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


def build_tf_chain(
    body_pose: PoseStamped,
    camera_gazebo_pose: PoseStamped,
    calibration: WorldMapTransform,
    *,
    map_frame: str,
    body_frame: str,
    optical_frame: str,
    base_to_gimbal_xyz_m: Sequence[float],
    camera_sensor_xyz_m: Sequence[float],
    optical_rpy_rad: Sequence[float],
) -> tuple[TransformStamped, TransformStamped]:
    """Build map→body→ROS-optical transforms at the body-pose timestamp."""
    if stamp_nanoseconds(body_pose) <= 0 or stamp_nanoseconds(camera_gazebo_pose) <= 0:
        raise ValueError('pose timestamps must be nonzero')

    map_frame = normalize_frame_id(map_frame)
    body_frame = normalize_frame_id(body_frame)
    optical_frame = normalize_frame_id(optical_frame)
    body_q_world = normalized_quaternion((
        body_pose.pose.orientation.x,
        body_pose.pose.orientation.y,
        body_pose.pose.orientation.z,
        body_pose.pose.orientation.w,
    ))
    camera_q_world = normalized_quaternion((
        camera_gazebo_pose.pose.orientation.x,
        camera_gazebo_pose.pose.orientation.y,
        camera_gazebo_pose.pose.orientation.z,
        camera_gazebo_pose.pose.orientation.w,
    ))

    map_position = world_point_to_map(
        (
            body_pose.pose.position.x,
            body_pose.pose.position.y,
            body_pose.pose.position.z,
        ),
        calibration.yaw_rad,
        (
            calibration.translation_x_m,
            calibration.translation_y_m,
            calibration.translation_z_m,
        ),
    )
    map_q_world = quaternion_from_euler(0.0, 0.0, calibration.yaw_rad)
    body_q_map = normalized_quaternion(quaternion_multiply(map_q_world, body_q_world))

    map_to_body = TransformStamped()
    map_to_body.header.stamp = body_pose.header.stamp
    map_to_body.header.frame_id = map_frame
    map_to_body.child_frame_id = body_frame
    map_to_body.transform.translation.x = map_position[0]
    map_to_body.transform.translation.y = map_position[1]
    map_to_body.transform.translation.z = map_position[2]
    map_to_body.transform.rotation.x = body_q_map[0]
    map_to_body.transform.rotation.y = body_q_map[1]
    map_to_body.transform.rotation.z = body_q_map[2]
    map_to_body.transform.rotation.w = body_q_map[3]

    camera_q_body = normalized_quaternion(
        quaternion_multiply(quaternion_inverse(body_q_world), camera_q_world)
    )
    sensor_offset_body = rotate_vector(camera_q_body, camera_sensor_xyz_m)
    optical_q_camera = quaternion_from_euler(*optical_rpy_rad)
    optical_q_body = normalized_quaternion(
        quaternion_multiply(camera_q_body, optical_q_camera)
    )

    body_to_optical = TransformStamped()
    body_to_optical.header.stamp = body_pose.header.stamp
    body_to_optical.header.frame_id = body_frame
    body_to_optical.child_frame_id = optical_frame
    body_to_optical.transform.translation.x = base_to_gimbal_xyz_m[0] + sensor_offset_body[0]
    body_to_optical.transform.translation.y = base_to_gimbal_xyz_m[1] + sensor_offset_body[1]
    body_to_optical.transform.translation.z = base_to_gimbal_xyz_m[2] + sensor_offset_body[2]
    body_to_optical.transform.rotation.x = optical_q_body[0]
    body_to_optical.transform.rotation.y = optical_q_body[1]
    body_to_optical.transform.rotation.z = optical_q_body[2]
    body_to_optical.transform.rotation.w = optical_q_body[3]
    return map_to_body, body_to_optical


class SimulationUavLocalization(Node):
    """Publish a Task 5 simulation-only dji1 TF chain."""

    OPERATIONAL_INPUT_PARAMETERS = ('pose_topic', 'camera_pose_topic')

    def __init__(self):
        super().__init__('simulation_uav_localization')
        self.declare_parameter('pose_topic', '/dji1/pose')
        self.declare_parameter('camera_pose_topic', '/dji1/camera0/actual/center_pose')
        self.declare_parameter('world_frame', 'gazebo_world')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('body_frame', 'dji1/base_link')
        self.declare_parameter('optical_frame', 'dji1/camera0/image_optical_frame')
        self.declare_parameter('sync_queue_size', 20)
        self.declare_parameter('sync_tolerance_s', 0.03)
        self.declare_parameter('calibration_mode', 'manifest')
        self.declare_parameter('calibration_csv', '')
        self.declare_parameter('calibration_group', 'parkinglot_west')
        self.declare_parameter('calibration_map_z_m', 0.0)
        self.declare_parameter('calibration_validation_place', 'parkinglot_west_0')
        self.declare_parameter('calibration_min_points', 10)
        self.declare_parameter('calibration_max_fit_error_m', 1.25)
        self.declare_parameter('calibration_max_validation_error_m', 0.75)
        self.declare_parameter('world_to_map_yaw_rad', 0.0)
        self.declare_parameter('world_to_map_x_m', 0.0)
        self.declare_parameter('world_to_map_y_m', 0.0)
        self.declare_parameter('world_to_map_z_m', 0.0)
        self.declare_parameter('base_to_gimbal_x_m', 0.0)
        self.declare_parameter('base_to_gimbal_y_m', 0.0)
        self.declare_parameter('base_to_gimbal_z_m', -0.1)
        self.declare_parameter('camera_sensor_x_m', 0.027)
        self.declare_parameter('camera_sensor_y_m', 0.0)
        self.declare_parameter('camera_sensor_z_m', -0.027)
        self.declare_parameter('optical_roll_rad', -math.pi / 2.0)
        self.declare_parameter('optical_pitch_rad', 0.0)
        self.declare_parameter('optical_yaw_rad', -math.pi / 2.0)

        self.world_frame = normalize_frame_id(self.get_parameter('world_frame').value)
        self.map_frame = normalize_frame_id(self.get_parameter('map_frame').value)
        self.body_frame = normalize_frame_id(self.get_parameter('body_frame').value)
        self.optical_frame = normalize_frame_id(self.get_parameter('optical_frame').value)
        self.sync_queue_size = max(2, int(self.get_parameter('sync_queue_size').value))
        self.sync_tolerance_ns = int(
            max(0.0, float(self.get_parameter('sync_tolerance_s').value)) * 1.0e9
        )
        self.base_to_gimbal = tuple(float(self.get_parameter(name).value) for name in (
            'base_to_gimbal_x_m', 'base_to_gimbal_y_m', 'base_to_gimbal_z_m'
        ))
        self.camera_sensor = tuple(float(self.get_parameter(name).value) for name in (
            'camera_sensor_x_m', 'camera_sensor_y_m', 'camera_sensor_z_m'
        ))
        self.optical_rpy = tuple(float(self.get_parameter(name).value) for name in (
            'optical_roll_rad', 'optical_pitch_rad', 'optical_yaw_rad'
        ))
        self.calibration = self._load_calibration()
        self._body_queue: deque[PoseStamped] = deque(maxlen=self.sync_queue_size)
        self._camera_queue: deque[PoseStamped] = deque(maxlen=self.sync_queue_size)
        self._last_published_stamp_ns = -1
        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('pose_topic').value), self._body_callback, 20
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('camera_pose_topic').value),
            self._camera_callback,
            20,
        )
        self.get_logger().info(
            'Simulation-only localization ready: '
            f'{self.map_frame} -> {self.body_frame} -> {self.optical_frame}; '
            f'world_to_map=(yaw={self.calibration.yaw_rad:.6f}, '
            f'x={self.calibration.translation_x_m:.3f}, '
            f'y={self.calibration.translation_y_m:.3f}, '
            f'z={self.calibration.translation_z_m:.3f}), '
            f'max_fit_error={self.calibration.max_fit_error_m:.3f}m, '
            f'validation_error={self.calibration.validation_error_m}'
        )

    def _load_calibration(self) -> WorldMapTransform:
        mode = str(self.get_parameter('calibration_mode').value).strip().lower()
        if mode == 'explicit':
            return WorldMapTransform(
                yaw_rad=float(self.get_parameter('world_to_map_yaw_rad').value),
                translation_x_m=float(self.get_parameter('world_to_map_x_m').value),
                translation_y_m=float(self.get_parameter('world_to_map_y_m').value),
                translation_z_m=float(self.get_parameter('world_to_map_z_m').value),
                point_count=0,
                max_fit_error_m=0.0,
                validation_error_m=None,
            )
        if mode != 'manifest':
            raise ValueError("calibration_mode must be 'manifest' or 'explicit'")

        csv_path = str(self.get_parameter('calibration_csv').value).strip()
        if not csv_path:
            raise ValueError('calibration_csv is required in manifest mode')
        points = load_calibration_points(
            csv_path, str(self.get_parameter('calibration_group').value).strip()
        )
        calibration = fit_world_to_map(
            points,
            map_z_m=float(self.get_parameter('calibration_map_z_m').value),
            validation_place=str(self.get_parameter('calibration_validation_place').value).strip(),
        )
        minimum_points = max(2, int(self.get_parameter('calibration_min_points').value))
        if calibration.point_count < minimum_points:
            raise ValueError(
                f'calibration has {calibration.point_count} points; {minimum_points} required'
            )
        max_fit_error = float(self.get_parameter('calibration_max_fit_error_m').value)
        if calibration.max_fit_error_m > max_fit_error:
            raise ValueError(
                f'calibration fit error {calibration.max_fit_error_m:.3f}m exceeds '
                f'{max_fit_error:.3f}m'
            )
        max_validation_error = float(
            self.get_parameter('calibration_max_validation_error_m').value
        )
        if (
            calibration.validation_error_m is not None
            and calibration.validation_error_m > max_validation_error
        ):
            raise ValueError(
                f'calibration validation error {calibration.validation_error_m:.3f}m exceeds '
                f'{max_validation_error:.3f}m'
            )
        return calibration

    def _valid_pose(self, message: PoseStamped, label: str) -> bool:
        try:
            frame = normalize_frame_id(message.header.frame_id)
        except ValueError as error:
            self.get_logger().warn(f'Rejecting {label}: {error}', throttle_duration_sec=5.0)
            return False
        if frame != self.world_frame:
            self.get_logger().warn(
                f'Rejecting {label}: frame {frame!r} is not {self.world_frame!r}',
                throttle_duration_sec=5.0,
            )
            return False
        if stamp_nanoseconds(message) <= 0:
            self.get_logger().warn(
                f'Rejecting {label}: missing simulation timestamp', throttle_duration_sec=5.0
            )
            return False
        return True

    def _body_callback(self, message: PoseStamped) -> None:
        if self._valid_pose(message, 'UAV body pose'):
            self._body_queue.append(message)
            self._publish_closest_pair()

    def _camera_callback(self, message: PoseStamped) -> None:
        if self._valid_pose(message, 'camera pose'):
            self._camera_queue.append(message)
            self._publish_closest_pair()

    def _publish_closest_pair(self) -> None:
        if not self._body_queue or not self._camera_queue:
            return
        best: tuple[int, int, int] | None = None
        for body_index, body in enumerate(self._body_queue):
            body_stamp = stamp_nanoseconds(body)
            for camera_index, camera in enumerate(self._camera_queue):
                difference = abs(body_stamp - stamp_nanoseconds(camera))
                if best is None or difference < best[0]:
                    best = (difference, body_index, camera_index)
        if best is None or best[0] > self.sync_tolerance_ns:
            return
        _, body_index, camera_index = best
        body = self._body_queue[body_index]
        camera = self._camera_queue[camera_index]
        body_stamp = stamp_nanoseconds(body)
        del self._body_queue[body_index]
        del self._camera_queue[camera_index]
        if body_stamp <= self._last_published_stamp_ns:
            return
        try:
            transforms = build_tf_chain(
                body,
                camera,
                self.calibration,
                map_frame=self.map_frame,
                body_frame=self.body_frame,
                optical_frame=self.optical_frame,
                base_to_gimbal_xyz_m=self.base_to_gimbal,
                camera_sensor_xyz_m=self.camera_sensor,
                optical_rpy_rad=self.optical_rpy,
            )
        except ValueError as error:
            self.get_logger().warn(
                f'Rejecting synchronized UAV pose pair: {error}', throttle_duration_sec=5.0
            )
            return
        self._broadcaster.sendTransform(transforms)
        self._last_published_stamp_ns = body_stamp


def main(args=None):
    rclpy.init(args=args)
    node = SimulationUavLocalization()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
