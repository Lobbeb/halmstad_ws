import inspect
import math
from pathlib import Path

import pytest
from geometry_msgs.msg import PoseStamped

from lrs_halmstad.sim.simulation_uav_localization import (
    CalibrationPoint,
    SimulationUavLocalization,
    WorldMapTransform,
    build_tf_chain,
    fit_world_to_map,
    load_calibration_points,
    normalize_frame_id,
    quaternion_from_euler,
    rotate_vector,
    stamp_nanoseconds,
    world_point_to_map,
)
from lrs_halmstad.sim.simulator import Simulator


def _simulator_pitch_fixture(mount_pitch_deg=30.0):
    simulator = Simulator.__new__(Simulator)
    simulator.camera_mount_pitch_deg = mount_pitch_deg
    simulator.gimbal_pitch_min = -math.pi / 2.0
    simulator.gimbal_pitch_max = math.pi / 2.0
    return simulator


def test_gimbal_joint_command_matches_fixed_camera_mount():
    simulator = _simulator_pitch_fixture()

    joint_command = simulator._gimbal_pitch_cmd_rad(-62.0)

    assert math.degrees(joint_command) == pytest.approx(-32.0)
    assert 30.0 - math.degrees(joint_command) == pytest.approx(62.0)


def test_reported_camera_tilt_matches_physical_joint_orientation():
    simulator = _simulator_pitch_fixture()

    reported_tilt = simulator._absolute_camera_tilt_deg(-62.0)
    physical_down_pitch = 30.0 - math.degrees(
        simulator._gimbal_pitch_cmd_rad(-62.0)
    )

    assert reported_tilt == pytest.approx(-62.0)
    assert -reported_tilt == pytest.approx(physical_down_pitch)


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]


def _pose(stamp_ns, position=(0.0, 0.0, 0.0), rpy=(0.0, 0.0, 0.0)):
    message = PoseStamped()
    message.header.stamp.sec = stamp_ns // 1_000_000_000
    message.header.stamp.nanosec = stamp_ns % 1_000_000_000
    message.header.frame_id = 'gazebo_world'
    message.pose.position.x, message.pose.position.y, message.pose.position.z = position
    quaternion = quaternion_from_euler(*rpy)
    message.pose.orientation.x = quaternion[0]
    message.pose.orientation.y = quaternion[1]
    message.pose.orientation.z = quaternion[2]
    message.pose.orientation.w = quaternion[3]
    return message


def _identity_calibration():
    return WorldMapTransform(0.0, 0.0, 0.0, 0.0, 2, 0.0, None)


def _chain(body=None, camera=None, calibration=None):
    return build_tf_chain(
        body or _pose(1_250_000_000),
        camera or _pose(1_250_000_000),
        calibration or _identity_calibration(),
        map_frame='map',
        body_frame='dji1/base_link',
        optical_frame='dji1/camera0/image_optical_frame',
        base_to_gimbal_xyz_m=(0.0, 0.0, -0.1),
        camera_sensor_xyz_m=(0.027, 0.0, -0.027),
        optical_rpy_rad=(-math.pi / 2.0, 0.0, -math.pi / 2.0),
    )


def test_frame_name_normalization():
    assert normalize_frame_id('/dji1/camera0/image_optical_frame') == (
        'dji1/camera0/image_optical_frame'
    )
    with pytest.raises(ValueError):
        normalize_frame_id('///')
    with pytest.raises(ValueError):
        normalize_frame_id('dji1//camera')


def test_camera_extrinsics_match_model_and_ros_optical_axes():
    model = (REPOSITORY_ROOT / 'src/lrs_halmstad/xacro/lrs_model.xacro').read_text()
    camera = (
        REPOSITORY_ROOT / 'src/lrs_halmstad/xacro/lrs_camera_gimbal.sdf.xacro'
    ).read_text()
    optical_reference = (
        REPOSITORY_ROOT / 'src/lrs_halmstad/urdf/lrs_camera_gimbal.urdf.xacro'
    ).read_text()
    assert '<xacro:arg name="camera_z_offset" default="-0.1"/>' in model
    assert '>0.027 0 -0.027 ${sensor_roll_offset}' in camera
    assert 'rpy="${-M_PI/2} 0.0 ${-M_PI/2}"' in optical_reference

    _, body_to_optical = _chain()
    translation = body_to_optical.transform.translation
    assert (translation.x, translation.y, translation.z) == pytest.approx(
        (0.027, 0.0, -0.127)
    )
    rotation = body_to_optical.transform.rotation
    forward_in_body = rotate_vector(
        (rotation.x, rotation.y, rotation.z, rotation.w), (0.0, 0.0, 1.0)
    )
    assert forward_in_body == pytest.approx((1.0, 0.0, 0.0), abs=1.0e-12)


def test_world_to_map_transform_math():
    yaw = math.radians(30.0)
    translation = (4.0, -2.0, 0.5)
    source = [(0.0, 0.0, 1.0), (2.0, 0.0, 1.2), (0.0, 3.0, 0.8)]
    points = []
    for index, world in enumerate(source):
        mapped = world_point_to_map(world, yaw, translation)
        points.append(CalibrationPoint(
            str(index), world[0], world[1], world[2], mapped[0], mapped[1]
        ))
    fitted = fit_world_to_map(points, map_z_m=1.5)
    assert fitted.yaw_rad == pytest.approx(yaw)
    assert fitted.translation_x_m == pytest.approx(translation[0])
    assert fitted.translation_y_m == pytest.approx(translation[1])
    assert fitted.translation_z_m == pytest.approx(0.5)
    assert fitted.max_fit_error_m < 1.0e-12


def test_known_route_world_point_maps_to_stored_map_measurement():
    manifest = REPOSITORY_ROOT / 'maps/waypoints_baylands_groups.csv'
    points = load_calibration_points(str(manifest), 'parkinglot_west')
    calibration = fit_world_to_map(
        points, map_z_m=0.0, validation_place='parkinglot_west_0'
    )
    held_out = next(point for point in points if point.place == 'parkinglot_west_0')
    mapped = world_point_to_map(
        (held_out.world_x, held_out.world_y, held_out.world_z),
        calibration.yaw_rad,
        (
            calibration.translation_x_m,
            calibration.translation_y_m,
            calibration.translation_z_m,
        ),
    )
    assert math.hypot(mapped[0] - held_out.map_x, mapped[1] - held_out.map_y) < 0.75
    assert calibration.max_fit_error_m < 1.25


def test_tf_chain_preserves_timestamp_and_has_expected_frames():
    body = _pose(9_876_543_210, position=(1.0, 2.0, 3.0), rpy=(0.0, 0.0, 0.4))
    camera = _pose(9_876_543_210, rpy=(0.0, -0.3, 0.5))
    map_to_body, body_to_optical = _chain(body=body, camera=camera)
    assert stamp_nanoseconds(body) == 9_876_543_210
    assert map_to_body.header.stamp == body.header.stamp
    assert body_to_optical.header.stamp == body.header.stamp
    assert map_to_body.header.frame_id == 'map'
    assert map_to_body.child_frame_id == body_to_optical.header.frame_id == 'dji1/base_link'
    assert body_to_optical.child_frame_id == 'dji1/camera0/image_optical_frame'


def test_localization_has_no_ugv_operational_input_dependency():
    assert SimulationUavLocalization.OPERATIONAL_INPUT_PARAMETERS == (
        'pose_topic', 'camera_pose_topic'
    )
    constructor = inspect.getsource(SimulationUavLocalization.__init__).lower()
    assert constructor.count('create_subscription(') == 2
    for forbidden in ('/ugv', '/a201', 'amcl_pose', 'odom_topic'):
        assert forbidden not in constructor
