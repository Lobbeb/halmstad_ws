from collections import OrderedDict
import inspect
import math

import numpy as np
import pytest
from geometry_msgs.msg import TransformStamped
from rclpy.clock import ClockType
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException

from lrs_halmstad.perception.detection_protocol import Detection2D
from lrs_halmstad.perception.support_hazard_projector import (
    CameraIntrinsics,
    SupportHazardProjector,
    back_project_optical,
    build_projected_hazard,
    camera_intrinsics,
    depth_image_to_meters,
    lookup_transform_at,
    nearest_cached,
    observation_is_stale,
    projection_covariance,
    sample_depth_patch,
    stable_source_track_id,
    track_expired,
    transform_optical_point,
)
from lrs_halmstad.sim.simulation_uav_localization import quaternion_from_euler
from lrs_halmstad_interfaces.msg import AerialHazard


def _intrinsics():
    return CameraIntrinsics(100.0, 100.0, 50.0, 40.0, 100, 80)


def _detection(bbox=(20.0, 20.0, 80.0, 60.0), confidence=0.8):
    return Detection2D(
        u=50.0,
        v=40.0,
        conf=confidence,
        bbox=bbox,
        cls_id=0,
        cls_name='ugv',
    )


def _depth_image(values, encoding='32FC1', scale_bytes=None):
    values = np.asarray(values, dtype=np.float32)
    message = Image()
    message.height, message.width = values.shape
    message.encoding = encoding
    message.is_bigendian = False
    message.step = values.shape[1] * 4 if scale_bytes is None else scale_bytes
    message.data = values.tobytes()
    return message


def _transform(translation=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    message = TransformStamped()
    message.header.frame_id = 'map'
    message.child_frame_id = 'dji1/camera0/image_optical_frame'
    message.transform.translation.x = translation[0]
    message.transform.translation.y = translation[1]
    message.transform.translation.z = translation[2]
    quaternion = quaternion_from_euler(*rpy)
    message.transform.rotation.x = quaternion[0]
    message.transform.rotation.y = quaternion[1]
    message.transform.rotation.z = quaternion[2]
    message.transform.rotation.w = quaternion[3]
    return message


def test_camera_back_projection_and_verified_optical_axes():
    intrinsics = _intrinsics()
    assert back_project_optical(50.0, 40.0, 5.0, intrinsics) == pytest.approx(
        (0.0, 0.0, 5.0)
    )
    assert back_project_optical(60.0, 50.0, 5.0, intrinsics) == pytest.approx(
        (0.5, 0.5, 5.0)
    )
    assert transform_optical_point((0.5, 0.5, 5.0), _transform()) == pytest.approx(
        (0.5, 0.5, 5.0)
    )


def test_camera_info_validation():
    message = CameraInfo()
    message.width = 100
    message.height = 80
    message.k = [100.0, 0.0, 50.0, 0.0, 100.0, 40.0, 0.0, 0.0, 1.0]
    assert camera_intrinsics(message) == _intrinsics()
    message.k[0] = 0.0
    with pytest.raises(ValueError):
        camera_intrinsics(message)


def test_supported_depth_encoding_conversion_and_unsupported_rejection():
    values = np.array([[1.0, 2.0], [3.0, np.inf]], dtype=np.float32)
    converted = depth_image_to_meters(
        _depth_image(values), supported_encoding='32FC1', depth_scale=1.0
    )
    assert converted[:2, :2] == pytest.approx(values)
    with pytest.raises(ValueError, match='unsupported'):
        depth_image_to_meters(
            _depth_image(values, encoding='16UC1'),
            supported_encoding='32FC1',
            depth_scale=1.0,
        )


def test_depth_patch_uses_inner_box_and_robust_statistic():
    depth = np.full((80, 100), np.inf, dtype=np.float32)
    depth[30:50, 35:65] = 5.0
    depth[35, 40] = 50.0
    depth[36, 40] = np.nan
    depth[37, 40] = 0.0
    sample = sample_depth_patch(
        depth,
        _detection(),
        inner_fraction=0.5,
        minimum_valid_pixels=20,
        minimum_depth_m=0.2,
        maximum_depth_m=20.0,
        statistic='median',
        percentile=50.0,
    )
    assert sample.depth_m == pytest.approx(5.0)
    assert sample.pixel_u == pytest.approx(50.0)
    assert sample.pixel_v == pytest.approx(40.0)
    assert sample.valid_pixel_count >= 20


def test_depth_patch_rejects_invalid_count_and_range():
    invalid = np.full((80, 100), np.inf, dtype=np.float32)
    invalid[40, 50] = 5.0
    with pytest.raises(ValueError, match='insufficient'):
        sample_depth_patch(
            invalid,
            _detection(),
            inner_fraction=0.5,
            minimum_valid_pixels=2,
            minimum_depth_m=0.2,
            maximum_depth_m=20.0,
            statistic='median',
            percentile=50.0,
        )
    out_of_range = np.full((80, 100), 100.0, dtype=np.float32)
    with pytest.raises(ValueError, match='insufficient'):
        sample_depth_patch(
            out_of_range,
            _detection(),
            inner_fraction=0.5,
            minimum_valid_pixels=1,
            minimum_depth_m=0.2,
            maximum_depth_m=20.0,
            statistic='percentile',
            percentile=25.0,
        )


def test_synchronization_tolerance_is_bounded_and_deterministic():
    cache = OrderedDict([(900, 'old'), (1100, 'new')])
    assert nearest_cached(cache, 1000, 100) == (900, 'old')
    assert nearest_cached(cache, 1000, 99) is None


def test_stale_and_ttl_behavior():
    assert not observation_is_stale(1_400_000_000, 1_000_000_000, 0.5)
    assert observation_is_stale(1_600_000_000, 1_000_000_000, 0.5)
    assert observation_is_stale(900_000_000, 1_000_000_000, 0.5)
    assert not track_expired(2_000_000_000, 1_000_000_000, 1.0)
    assert track_expired(2_000_000_001, 1_000_000_000, 1.0)


class _RecordingBuffer:
    def __init__(self, result=None, error=None):
        self.result = result
        self.error = error
        self.requested_time = None

    def lookup_transform(self, target, source, requested_time, timeout):
        self.requested_time = requested_time
        if self.error is not None:
            raise self.error
        return self.result


def test_transform_lookup_uses_acquisition_timestamp_and_rejects_unavailable():
    transform = _transform()
    buffer = _RecordingBuffer(result=transform)
    result = lookup_transform_at(
        buffer,
        target_frame='map',
        source_frame='dji1/camera0/image_optical_frame',
        acquisition_stamp_ns=5_123_456_789,
        timeout_s=0.2,
        clock_type=ClockType.ROS_TIME,
    )
    assert result is transform
    assert buffer.requested_time.nanoseconds == 5_123_456_789
    unavailable = _RecordingBuffer(error=TransformException('missing'))
    with pytest.raises(TransformException):
        lookup_transform_at(
            unavailable,
            target_frame='map',
            source_frame='dji1/camera0/image_optical_frame',
            acquisition_stamp_ns=5_123_456_789,
            timeout_s=0.2,
            clock_type=ClockType.ROS_TIME,
        )


def test_known_synthetic_geometry_produces_expected_map_position():
    optical = back_project_optical(50.0, 40.0, 5.0, _intrinsics())
    transform = _transform(translation=(10.0, 20.0, 3.0), rpy=(0.0, 0.0, math.pi / 2.0))
    assert transform_optical_point(optical, transform) == pytest.approx((10.0, 20.0, 8.0))


def _covariance(depth):
    return projection_covariance(
        depth_m=depth,
        pixel_u=60.0,
        pixel_v=45.0,
        intrinsics=_intrinsics(),
        map_from_optical_quaternion=(0.0, 0.0, 0.0, 1.0),
        depth_stddev_base_m=0.25,
        depth_stddev_per_m=0.02,
        image_stddev_px=2.0,
        transform_stddev_m=1.25,
        orientation_stddev_rad=0.35,
    )


def test_covariance_is_nonzero_and_grows_with_depth():
    near = _covariance(2.0)
    far = _covariance(20.0)
    assert len(near) == 36
    assert all(near[index] > 0.0 for index in (0, 7, 14, 21, 28, 35))
    assert far[0] > near[0]
    assert far[7] > near[7]
    assert far[14] > near[14]


def test_typed_hazard_fields_and_stable_source_id():
    track_id = stable_source_track_id('dji1', 'ugv', None, '')
    assert track_id == 'dji1:ugv:0'
    assert stable_source_track_id('dji1', 'ugv', 42, '') == 'dji1:42'
    hazard = build_projected_hazard(
        source_uav='dji1',
        class_name='ugv',
        track_id=track_id,
        map_position=(1.0, 2.0, 0.9),
        dimensions_xyz=(1.0, 1.0, 1.8),
        class_yaw_rad=0.0,
        confidence=0.8,
        covariance=_covariance(5.0),
        state=AerialHazard.TENTATIVE,
        first_seen_ns=4_000_000_000,
        acquisition_stamp_ns=5_000_000_000,
        ttl_s=1.0,
        support_quality=0.8,
        provenance='rgbd_projector:dji1',
    )
    assert hazard.detection.header.frame_id == 'map'
    assert hazard.detection.header.stamp.sec == 5
    assert hazard.detection.id == track_id
    assert hazard.source_uavs == ['dji1']
    assert hazard.detection.results[0].hypothesis.class_id == 'ugv'
    assert hazard.detection.results[0].hypothesis.score == pytest.approx(0.8)
    assert hazard.first_seen.sec == 4
    assert hazard.last_seen.sec == 5
    assert hazard.ttl.sec == 1
    assert hazard.state == AerialHazard.TENTATIVE
    assert hazard.provenance == 'rgbd_projector:dji1'


def test_projector_has_no_ugv_pose_or_odometry_dependency():
    assert SupportHazardProjector.OPERATIONAL_INPUT_PARAMETERS == (
        'rgb_topic', 'depth_topic', 'camera_info_topic', 'detector_topic'
    )
    constructor = inspect.getsource(SupportHazardProjector.__init__).lower()
    for forbidden in ('/ugv', '/a201', 'amcl_pose', 'odom_topic', 'ground_truth'):
        assert forbidden not in constructor
