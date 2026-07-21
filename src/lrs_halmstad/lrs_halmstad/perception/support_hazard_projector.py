"""Project one dji1 detector observation into a typed map-frame hazard."""

from __future__ import annotations

from collections import OrderedDict
from dataclasses import dataclass
import math
import sys
from typing import Generic, Sequence, TypeVar

import numpy as np
import rclpy
from builtin_interfaces.msg import Time as TimeMessage
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from lrs_halmstad.perception.detection_protocol import Detection2D, decode_detection_payload
from lrs_halmstad.sim.simulation_uav_localization import (
    normalize_frame_id,
    normalized_quaternion,
    rotate_vector,
)
from lrs_halmstad.tools.synthetic_hazard_publisher import build_hazard
from lrs_halmstad_interfaces.msg import AerialHazard, AerialHazardArray


NANOSECONDS_PER_SECOND = 1_000_000_000
T = TypeVar('T')


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class DepthSample:
    depth_m: float
    pixel_u: float
    pixel_v: float
    valid_pixel_count: int


@dataclass
class SourceTrack:
    track_id: str
    first_seen_ns: int
    last_seen_ns: int
    hits: int


def time_message_from_ns(stamp_ns: int) -> TimeMessage:
    if stamp_ns < 0:
        raise ValueError('timestamp must be non-negative')
    message = TimeMessage()
    message.sec = stamp_ns // NANOSECONDS_PER_SECOND
    message.nanosec = stamp_ns % NANOSECONDS_PER_SECOND
    return message


def message_stamp_ns(message) -> int:
    return (
        int(message.header.stamp.sec) * NANOSECONDS_PER_SECOND
        + int(message.header.stamp.nanosec)
    )


def camera_intrinsics(message: CameraInfo) -> CameraIntrinsics:
    if len(message.k) != 9:
        raise ValueError('CameraInfo.k must contain nine values')
    fx = float(message.k[0])
    fy = float(message.k[4])
    cx = float(message.k[2])
    cy = float(message.k[5])
    width = int(message.width)
    height = int(message.height)
    if not all(math.isfinite(value) for value in (fx, fy, cx, cy)):
        raise ValueError('camera intrinsics must be finite')
    if fx <= 0.0 or fy <= 0.0 or width <= 0 or height <= 0:
        raise ValueError('camera focal lengths and dimensions must be positive')
    return CameraIntrinsics(fx, fy, cx, cy, width, height)


def back_project_optical(
    pixel_u: float,
    pixel_v: float,
    depth_m: float,
    intrinsics: CameraIntrinsics,
) -> tuple[float, float, float]:
    """Back-project using ROS optical axes: +x right, +y down, +z forward."""
    if not all(math.isfinite(value) for value in (pixel_u, pixel_v, depth_m)):
        raise ValueError('pixel and depth values must be finite')
    if depth_m <= 0.0:
        raise ValueError('depth must be positive')
    return (
        (pixel_u - intrinsics.cx) * depth_m / intrinsics.fx,
        (pixel_v - intrinsics.cy) * depth_m / intrinsics.fy,
        depth_m,
    )


def depth_image_to_meters(
    message: Image,
    *,
    supported_encoding: str,
    depth_scale: float,
) -> np.ndarray:
    encoding = str(message.encoding).strip()
    if encoding != supported_encoding or encoding != '32FC1':
        raise ValueError(f'unsupported depth encoding: {encoding!r}')
    if not math.isfinite(depth_scale) or depth_scale <= 0.0:
        raise ValueError('depth_scale must be finite and positive')
    height = int(message.height)
    width = int(message.width)
    step = int(message.step)
    if height <= 0 or width <= 0 or step < width * 4 or step % 4 != 0:
        raise ValueError('invalid depth image dimensions or step')
    if len(message.data) < height * step:
        raise ValueError('depth image data is truncated')
    dtype = np.dtype('>f4' if message.is_bigendian else '<f4')
    row_width = step // 4
    values = np.frombuffer(message.data, dtype=dtype, count=height * row_width)
    return values.reshape(height, row_width)[:, :width].astype(np.float32) * depth_scale


def sample_depth_patch(
    depth_m: np.ndarray,
    detection: Detection2D,
    *,
    inner_fraction: float,
    minimum_valid_pixels: int,
    minimum_depth_m: float,
    maximum_depth_m: float,
    statistic: str,
    percentile: float,
) -> DepthSample:
    if depth_m.ndim != 2 or depth_m.size == 0:
        raise ValueError('depth image must be a nonempty 2D array')
    if not 0.0 < inner_fraction <= 1.0:
        raise ValueError('inner_fraction must be in (0, 1]')
    if minimum_valid_pixels <= 0:
        raise ValueError('minimum_valid_pixels must be positive')
    if not 0.0 < minimum_depth_m < maximum_depth_m:
        raise ValueError('depth limits must satisfy 0 < minimum < maximum')
    statistic = str(statistic).strip().lower()
    if statistic not in ('median', 'percentile'):
        raise ValueError("statistic must be 'median' or 'percentile'")
    if not 0.0 <= percentile <= 100.0:
        raise ValueError('percentile must be in [0, 100]')

    height, width = depth_m.shape
    x1, y1, x2, y2 = (float(value) for value in detection.bbox)
    if not all(math.isfinite(value) for value in (x1, y1, x2, y2)):
        raise ValueError('detection bbox must be finite')
    if x2 <= x1 or y2 <= y1:
        raise ValueError('detection bbox dimensions must be positive')
    center_u = 0.5 * (x1 + x2)
    center_v = 0.5 * (y1 + y2)
    half_width = 0.5 * (x2 - x1) * inner_fraction
    half_height = 0.5 * (y2 - y1) * inner_fraction
    column_start = max(0, int(math.floor(center_u - half_width)))
    column_end = min(width, int(math.ceil(center_u + half_width)))
    row_start = max(0, int(math.floor(center_v - half_height)))
    row_end = min(height, int(math.ceil(center_v + half_height)))
    if column_end <= column_start or row_end <= row_start:
        raise ValueError('detection bbox does not overlap the depth image')

    patch = depth_m[row_start:row_end, column_start:column_end]
    valid = patch[
        np.isfinite(patch)
        & (patch > 0.0)
        & (patch >= minimum_depth_m)
        & (patch <= maximum_depth_m)
    ]
    if valid.size < minimum_valid_pixels:
        raise ValueError(
            f'insufficient valid depth pixels: {valid.size} < {minimum_valid_pixels}'
        )
    selected = (
        float(np.median(valid))
        if statistic == 'median'
        else float(np.percentile(valid, percentile))
    )
    return DepthSample(selected, center_u, center_v, int(valid.size))


def nearest_cached(
    cache: OrderedDict[int, T], target_stamp_ns: int, tolerance_ns: int
) -> tuple[int, T] | None:
    if not cache:
        return None
    stamp = min(cache, key=lambda candidate: (abs(candidate - target_stamp_ns), candidate))
    if abs(stamp - target_stamp_ns) > tolerance_ns:
        return None
    return stamp, cache[stamp]


def observation_is_stale(now_ns: int, observation_ns: int, stale_timeout_s: float) -> bool:
    if observation_ns <= 0 or now_ns < observation_ns:
        return True
    if not math.isfinite(stale_timeout_s) or stale_timeout_s < 0.0:
        raise ValueError('stale_timeout_s must be finite and non-negative')
    return now_ns - observation_ns > int(round(stale_timeout_s * NANOSECONDS_PER_SECOND))


def track_expired(now_ns: int, last_seen_ns: int, ttl_s: float) -> bool:
    if not math.isfinite(ttl_s) or ttl_s <= 0.0:
        raise ValueError('ttl_s must be finite and positive')
    return now_ns - last_seen_ns > int(round(ttl_s * NANOSECONDS_PER_SECOND))


def transform_optical_point(
    point_optical: Sequence[float], transform: TransformStamped
) -> tuple[float, float, float]:
    rotation = transform.transform.rotation
    rotated = rotate_vector(
        normalized_quaternion((rotation.x, rotation.y, rotation.z, rotation.w)),
        point_optical,
    )
    translation = transform.transform.translation
    return (
        rotated[0] + float(translation.x),
        rotated[1] + float(translation.y),
        rotated[2] + float(translation.z),
    )


def lookup_transform_at(
    buffer,
    *,
    target_frame: str,
    source_frame: str,
    acquisition_stamp_ns: int,
    timeout_s: float,
    clock_type,
):
    """Perform only a timestamped lookup; never substitute latest TF."""
    if acquisition_stamp_ns <= 0:
        raise ValueError('acquisition timestamp must be nonzero')
    return buffer.lookup_transform(
        normalize_frame_id(target_frame),
        normalize_frame_id(source_frame),
        Time(nanoseconds=acquisition_stamp_ns, clock_type=clock_type),
        timeout=Duration(seconds=float(timeout_s)),
    )


def projection_covariance(
    *,
    depth_m: float,
    pixel_u: float,
    pixel_v: float,
    intrinsics: CameraIntrinsics,
    map_from_optical_quaternion: Sequence[float],
    depth_stddev_base_m: float,
    depth_stddev_per_m: float,
    image_stddev_px: float,
    transform_stddev_m: float,
    orientation_stddev_rad: float,
) -> list[float]:
    values = (
        depth_m,
        depth_stddev_base_m,
        depth_stddev_per_m,
        image_stddev_px,
        transform_stddev_m,
        orientation_stddev_rad,
    )
    if not all(math.isfinite(value) and value >= 0.0 for value in values):
        raise ValueError('covariance assumptions must be finite and non-negative')
    if depth_m <= 0.0:
        raise ValueError('depth must be positive')
    depth_stddev = depth_stddev_base_m + depth_stddev_per_m * depth_m
    normalized_u = (pixel_u - intrinsics.cx) / intrinsics.fx
    normalized_v = (pixel_v - intrinsics.cy) / intrinsics.fy
    optical_covariance = np.diag([
        (depth_m * image_stddev_px / intrinsics.fx) ** 2
        + (normalized_u * depth_stddev) ** 2,
        (depth_m * image_stddev_px / intrinsics.fy) ** 2
        + (normalized_v * depth_stddev) ** 2,
        depth_stddev ** 2,
    ])
    quaternion = normalized_quaternion(map_from_optical_quaternion)
    basis = np.column_stack([
        rotate_vector(quaternion, (1.0, 0.0, 0.0)),
        rotate_vector(quaternion, (0.0, 1.0, 0.0)),
        rotate_vector(quaternion, (0.0, 0.0, 1.0)),
    ])
    map_covariance = (
        basis @ optical_covariance @ basis.T
        + np.eye(3, dtype=np.float64) * transform_stddev_m ** 2
    )
    covariance = [0.0] * 36
    for row in range(3):
        for column in range(3):
            covariance[row * 6 + column] = float(map_covariance[row, column])
    orientation_variance = max(orientation_stddev_rad ** 2, 1.0e-9)
    covariance[21] = orientation_variance
    covariance[28] = orientation_variance
    covariance[35] = orientation_variance
    return covariance


def stable_source_track_id(
    source_uav: str,
    class_name: str,
    detector_track_id: int | None,
    configured_track_id: str,
) -> str:
    configured = str(configured_track_id).strip()
    if configured:
        return configured
    if detector_track_id is not None and detector_track_id >= 0:
        return f'{source_uav}:{detector_track_id}'
    return f'{source_uav}:{class_name}:0'


def build_projected_hazard(
    *,
    source_uav: str,
    class_name: str,
    track_id: str,
    map_position: Sequence[float],
    dimensions_xyz: Sequence[float],
    class_yaw_rad: float,
    confidence: float,
    covariance: Sequence[float],
    state: int,
    first_seen_ns: int,
    acquisition_stamp_ns: int,
    ttl_s: float,
    support_quality: float,
    provenance: str,
) -> AerialHazard:
    return build_hazard(
        source_uav=source_uav,
        class_name=class_name,
        stable_track_id=track_id,
        center_xyz=map_position,
        yaw_rad=class_yaw_rad,
        dimensions_xyz=dimensions_xyz,
        confidence=confidence,
        covariance=covariance,
        state=state,
        first_seen_ns=first_seen_ns,
        last_seen_ns=acquisition_stamp_ns,
        ttl_s=ttl_s,
        support_quality=support_quality,
        provenance=provenance,
    )


class SupportHazardProjector(Node):
    """dji1-only RGB-D projector with timestamped tf2 transformation."""

    OPERATIONAL_INPUT_PARAMETERS = (
        'rgb_topic', 'depth_topic', 'camera_info_topic', 'detector_topic'
    )

    def __init__(self) -> None:
        super().__init__('support_hazard_projector')
        self.declare_parameter('source_uav', 'dji1')
        self.declare_parameter('rgb_topic', '/dji1/camera0/image_raw')
        self.declare_parameter('depth_topic', '/dji1/camera0/depth_image')
        self.declare_parameter('camera_info_topic', '/dji1/camera0/camera_info')
        self.declare_parameter('detector_topic', '/coord/support/dji1/hazard_detection')
        self.declare_parameter('output_topic', '/coord/support/dji1/aerial_hazards')
        self.declare_parameter('target_frame', 'map')
        self.declare_parameter('optical_frame', 'dji1/camera0/image_optical_frame')
        self.declare_parameter('target_class', 'ugv')
        self.declare_parameter('minimum_confidence', 0.5)
        self.declare_parameter('stable_track_id', '')
        self.declare_parameter('sync_queue_size', 30)
        self.declare_parameter('sync_tolerance_s', 0.02)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('supported_depth_encoding', '32FC1')
        self.declare_parameter('depth_scale', 1.0)
        self.declare_parameter('inner_box_fraction', 0.5)
        self.declare_parameter('minimum_valid_pixels', 20)
        self.declare_parameter('minimum_depth_m', 0.2)
        self.declare_parameter('maximum_depth_m', 100.0)
        self.declare_parameter('depth_statistic', 'median')
        self.declare_parameter('depth_percentile', 50.0)
        self.declare_parameter('transform_timeout_s', 0.2)
        self.declare_parameter('ttl_s', 1.0)
        self.declare_parameter('confirm_hits', 3)
        self.declare_parameter('expiry_check_rate_hz', 10.0)
        self.declare_parameter('summary_period_s', 5.0)
        self.declare_parameter('depth_stddev_base_m', 0.25)
        self.declare_parameter('depth_stddev_per_m', 0.02)
        self.declare_parameter('image_stddev_px', 2.0)
        self.declare_parameter('transform_stddev_m', 1.25)
        self.declare_parameter('orientation_stddev_rad', 0.35)
        self.declare_parameter('class_dimension_x_m', 1.0)
        self.declare_parameter('class_dimension_y_m', 1.0)
        self.declare_parameter('class_dimension_z_m', 1.8)
        self.declare_parameter('class_yaw_rad', 0.0)
        self.declare_parameter('provenance', 'rgbd_projector:dji1')

        self.source_uav = str(self.get_parameter('source_uav').value).strip()
        self.target_frame = normalize_frame_id(self.get_parameter('target_frame').value)
        self.optical_frame = normalize_frame_id(self.get_parameter('optical_frame').value)
        self.target_class = str(self.get_parameter('target_class').value).strip()
        self.minimum_confidence = float(self.get_parameter('minimum_confidence').value)
        self.configured_track_id = str(self.get_parameter('stable_track_id').value).strip()
        self.queue_size = max(3, int(self.get_parameter('sync_queue_size').value))
        self.sync_tolerance_ns = int(
            float(self.get_parameter('sync_tolerance_s').value) * NANOSECONDS_PER_SECOND
        )
        self.stale_timeout_s = float(self.get_parameter('stale_timeout_s').value)
        self.supported_depth_encoding = str(
            self.get_parameter('supported_depth_encoding').value
        ).strip()
        self.depth_scale = float(self.get_parameter('depth_scale').value)
        self.inner_box_fraction = float(self.get_parameter('inner_box_fraction').value)
        self.minimum_valid_pixels = int(self.get_parameter('minimum_valid_pixels').value)
        self.minimum_depth_m = float(self.get_parameter('minimum_depth_m').value)
        self.maximum_depth_m = float(self.get_parameter('maximum_depth_m').value)
        self.depth_statistic = str(self.get_parameter('depth_statistic').value).strip()
        self.depth_percentile = float(self.get_parameter('depth_percentile').value)
        self.transform_timeout_s = float(self.get_parameter('transform_timeout_s').value)
        self.ttl_s = float(self.get_parameter('ttl_s').value)
        self.confirm_hits = max(1, int(self.get_parameter('confirm_hits').value))
        self.class_dimensions = tuple(float(self.get_parameter(name).value) for name in (
            'class_dimension_x_m', 'class_dimension_y_m', 'class_dimension_z_m'
        ))
        self.class_yaw_rad = float(self.get_parameter('class_yaw_rad').value)
        self.provenance = str(self.get_parameter('provenance').value).strip()
        self.covariance_parameters = {
            name: float(self.get_parameter(name).value)
            for name in (
                'depth_stddev_base_m', 'depth_stddev_per_m', 'image_stddev_px',
                'transform_stddev_m', 'orientation_stddev_rad'
            )
        }
        self._validate_parameters()

        self.rgb_cache: OrderedDict[int, Image] = OrderedDict()
        self.depth_cache: OrderedDict[int, Image] = OrderedDict()
        self.camera_info_cache: OrderedDict[int, CameraInfo] = OrderedDict()
        self.track: SourceTrack | None = None
        self.empty_published = False
        self.counters = {
            'detections_received': 0,
            'synchronization_rejection': 0,
            'invalid_depth': 0,
            'missing_transform': 0,
            'successful_projections': 0,
            'stale_observations': 0,
            'published_hazards': 0,
        }
        self._last_summary = None

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            AerialHazardArray, str(self.get_parameter('output_topic').value), 10
        )
        self.create_subscription(
            Image, str(self.get_parameter('rgb_topic').value), self._on_rgb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Image, str(self.get_parameter('depth_topic').value), self._on_depth,
            qos_profile_sensor_data
        )
        self.create_subscription(
            CameraInfo, str(self.get_parameter('camera_info_topic').value),
            self._on_camera_info, qos_profile_sensor_data
        )
        self.create_subscription(
            String, str(self.get_parameter('detector_topic').value), self._on_detection, 10
        )
        expiry_rate = float(self.get_parameter('expiry_check_rate_hz').value)
        self.create_timer(1.0 / expiry_rate, self._expire_track)
        self.create_timer(float(self.get_parameter('summary_period_s').value), self._log_summary)
        self.get_logger().info(
            f'dji1 RGB-D hazard projector ready: detector='
            f'{self.get_parameter("detector_topic").value}, output='
            f'{self.get_parameter("output_topic").value}, frame={self.target_frame}, '
            f'class={self.target_class}, encoding={self.supported_depth_encoding}'
        )

    def _validate_parameters(self) -> None:
        if self.source_uav != 'dji1':
            raise ValueError('Task 5 supports source_uav=dji1 only')
        if not self.target_class:
            raise ValueError('target_class must not be empty')
        if not 0.0 <= self.minimum_confidence <= 1.0:
            raise ValueError('minimum_confidence must be in [0, 1]')
        if self.sync_tolerance_ns < 0:
            raise ValueError('sync_tolerance_s must be non-negative')
        if self.ttl_s <= 0.0 or self.transform_timeout_s < 0.0:
            raise ValueError('ttl_s must be positive and transform_timeout_s non-negative')
        if not all(math.isfinite(value) and value > 0.0 for value in self.class_dimensions):
            raise ValueError('class dimensions must be finite and positive')
        if self.supported_depth_encoding != '32FC1':
            raise ValueError('Task 5 runtime evidence supports only 32FC1 depth')
        expiry_rate = float(self.get_parameter('expiry_check_rate_hz').value)
        summary_period = float(self.get_parameter('summary_period_s').value)
        if expiry_rate <= 0.0 or summary_period <= 0.0:
            raise ValueError('timer rates must be positive')

    def _cache(self, cache: OrderedDict[int, T], message: T) -> None:
        stamp = message_stamp_ns(message)
        if stamp <= 0:
            return
        cache[stamp] = message
        cache.move_to_end(stamp)
        while len(cache) > self.queue_size:
            cache.popitem(last=False)

    def _on_rgb(self, message: Image) -> None:
        self._cache(self.rgb_cache, message)

    def _on_depth(self, message: Image) -> None:
        self._cache(self.depth_cache, message)

    def _on_camera_info(self, message: CameraInfo) -> None:
        self._cache(self.camera_info_cache, message)

    def _synchronized_messages(self, acquisition_ns: int):
        matched = [
            nearest_cached(cache, acquisition_ns, self.sync_tolerance_ns)
            for cache in (self.rgb_cache, self.depth_cache, self.camera_info_cache)
        ]
        if any(item is None for item in matched):
            return None
        stamps = [item[0] for item in matched]
        if max(stamps) - min(stamps) > self.sync_tolerance_ns:
            return None
        return matched[0][1], matched[1][1], matched[2][1]

    def _frames_match(self, messages: Sequence) -> bool:
        try:
            return all(
                normalize_frame_id(message.header.frame_id) == self.optical_frame
                for message in messages
            )
        except ValueError:
            return False

    def _publish(self, acquisition_ns: int, hazards: Sequence[AerialHazard]) -> None:
        message = AerialHazardArray()
        message.header = Header(
            stamp=time_message_from_ns(acquisition_ns), frame_id=self.target_frame
        )
        message.hazards = list(hazards)
        self.publisher.publish(message)

    def _on_detection(self, message: String) -> None:
        self.counters['detections_received'] += 1
        try:
            detection_message = decode_detection_payload(message.data)
        except ValueError:
            self.counters['synchronization_rejection'] += 1
            return
        detection = detection_message.detection
        if detection is None:
            return
        acquisition_ns = int(detection_message.stamp_ns)
        now_ns = int(self.get_clock().now().nanoseconds)
        if observation_is_stale(now_ns, acquisition_ns, self.stale_timeout_s):
            self.counters['stale_observations'] += 1
            return
        if (
            not math.isfinite(detection.conf)
            or detection.conf < self.minimum_confidence
            or detection.conf > 1.0
        ):
            return
        if self.target_class and detection.cls_name.lower() != self.target_class.lower():
            return
        synchronized = self._synchronized_messages(acquisition_ns)
        if synchronized is None or not self._frames_match(synchronized):
            self.counters['synchronization_rejection'] += 1
            return
        _, depth_message, camera_info_message = synchronized
        try:
            intrinsics = camera_intrinsics(camera_info_message)
            depth = depth_image_to_meters(
                depth_message,
                supported_encoding=self.supported_depth_encoding,
                depth_scale=self.depth_scale,
            )
            sample = sample_depth_patch(
                depth,
                detection,
                inner_fraction=self.inner_box_fraction,
                minimum_valid_pixels=self.minimum_valid_pixels,
                minimum_depth_m=self.minimum_depth_m,
                maximum_depth_m=self.maximum_depth_m,
                statistic=self.depth_statistic,
                percentile=self.depth_percentile,
            )
            optical_point = back_project_optical(
                sample.pixel_u, sample.pixel_v, sample.depth_m, intrinsics
            )
        except ValueError as error:
            self.counters['invalid_depth'] += 1
            self.get_logger().warn(
                f'discarding hazard detection: {error}', throttle_duration_sec=5.0
            )
            return
        try:
            transform = lookup_transform_at(
                self.tf_buffer,
                target_frame=self.target_frame,
                source_frame=self.optical_frame,
                acquisition_stamp_ns=acquisition_ns,
                timeout_s=self.transform_timeout_s,
                clock_type=self.get_clock().clock_type,
            )
        except (TransformException, ValueError) as error:
            self.counters['missing_transform'] += 1
            self.get_logger().warn(
                f'discarding hazard detection: timestamped TF unavailable: {error}',
                throttle_duration_sec=5.0,
            )
            return

        map_position = transform_optical_point(optical_point, transform)
        rotation = transform.transform.rotation
        covariance = projection_covariance(
            depth_m=sample.depth_m,
            pixel_u=sample.pixel_u,
            pixel_v=sample.pixel_v,
            intrinsics=intrinsics,
            map_from_optical_quaternion=(rotation.x, rotation.y, rotation.z, rotation.w),
            **self.covariance_parameters,
        )
        track_id = stable_source_track_id(
            self.source_uav,
            self.target_class,
            detection.track_id,
            self.configured_track_id,
        )
        if (
            self.track is None
            or self.track.track_id != track_id
            or track_expired(acquisition_ns, self.track.last_seen_ns, self.ttl_s)
        ):
            self.track = SourceTrack(track_id, acquisition_ns, acquisition_ns, 1)
        else:
            self.track.last_seen_ns = acquisition_ns
            self.track.hits += 1
        state = (
            AerialHazard.CONFIRMED
            if self.track.hits >= self.confirm_hits
            else AerialHazard.TENTATIVE
        )
        hazard = build_projected_hazard(
            source_uav=self.source_uav,
            class_name=self.target_class,
            track_id=track_id,
            map_position=map_position,
            dimensions_xyz=self.class_dimensions,
            class_yaw_rad=self.class_yaw_rad,
            confidence=float(detection.conf),
            covariance=covariance,
            state=state,
            first_seen_ns=self.track.first_seen_ns,
            acquisition_stamp_ns=acquisition_ns,
            ttl_s=self.ttl_s,
            support_quality=float(detection.conf),
            provenance=self.provenance,
        )
        self._publish(acquisition_ns, [hazard])
        self.empty_published = False
        self.counters['successful_projections'] += 1
        self.counters['published_hazards'] += 1

    def _expire_track(self) -> None:
        if self.track is None or self.empty_published:
            return
        now_ns = int(self.get_clock().now().nanoseconds)
        if not track_expired(now_ns, self.track.last_seen_ns, self.ttl_s):
            return
        self._publish(now_ns, [])
        self.track = None
        self.empty_published = True

    def _log_summary(self) -> None:
        snapshot = tuple(self.counters.items())
        if snapshot == self._last_summary:
            return
        self._last_summary = snapshot
        self.get_logger().info(
            'projector counters: '
            + ' '.join(f'{name}={value}' for name, value in self.counters.items())
        )


def main(args=None) -> None:
    if '--help' in sys.argv[1:] or '-h' in sys.argv[1:]:
        print(
            'Projects timestamped dji1 RGB-D detections into '
            'lrs_halmstad_interfaces/AerialHazardArray. Configure with ROS parameters.'
        )
        return
    rclpy.init(args=args)
    node = SupportHazardProjector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
