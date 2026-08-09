import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _gazebo_world_name(world_sub):
    return PythonExpression([
        "'office_construction' if '",
        world_sub,
        "' == 'construction' else '",
        world_sub,
        "'",
    ])


def _default_world_value(world_sub, warehouse_value: str, default_value: str, baylands_value: str | None = None):
    if baylands_value is None:
        baylands_value = default_value
    return PythonExpression([
        "'",
        warehouse_value,
        "' if '",
        world_sub,
        "'.startswith('warehouse') else '",
        baylands_value,
        "' if '",
        world_sub,
        "'.startswith('baylands') else '",
        default_value,
        "'",
    ])


def generate_launch_description():
    world_arg = DeclareLaunchArgument('world', default_value='baylands')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    uav_mode_arg = DeclareLaunchArgument('uav_mode', default_value='teleport')
    x_arg = DeclareLaunchArgument(
        'x',
        default_value=_default_world_value(
            LaunchConfiguration('world'),
            '-7.0',
            '-7.0',
            baylands_value='-21.085738068',
        ),
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value=_default_world_value(
            LaunchConfiguration('world'),
            '0.0',
            '0.0',
            baylands_value='-54.861874768',
        ),
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='7.0',
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw',
        default_value='0.0',
    )
    camera_name_arg = DeclareLaunchArgument('camera_name', default_value='camera0')
    uav_camera_mode_arg = DeclareLaunchArgument('uav_camera_mode', default_value='integrated_joint')
    camera_pitch_offset_deg_arg = DeclareLaunchArgument('camera_pitch_offset_deg', default_value='45.0')
    camera_update_rate_arg = DeclareLaunchArgument('camera_update_rate', default_value='10')
    bridge_depth_arg = DeclareLaunchArgument('bridge_depth', default_value='false')
    bridge_gimbal_arg = DeclareLaunchArgument('bridge_gimbal', default_value='false')
    with_laser_arg = DeclareLaunchArgument('with_laser', default_value='false')
    bridge_laser_arg = DeclareLaunchArgument('bridge_laser', default_value='false')
    laser_name_arg = DeclareLaunchArgument('laser_name', default_value='laser0')
    laser_update_rate_arg = DeclareLaunchArgument('laser_update_rate', default_value='10')
    laser_min_range_arg = DeclareLaunchArgument('laser_min_range', default_value='0.2')
    laser_max_range_arg = DeclareLaunchArgument('laser_max_range', default_value='25.0')
    laser_angle_deg_arg = DeclareLaunchArgument('laser_angle_deg', default_value='180')
    laser_x_arg = DeclareLaunchArgument('laser_x', default_value='0.0')
    laser_y_arg = DeclareLaunchArgument('laser_y', default_value='0.0')
    laser_z_arg = DeclareLaunchArgument('laser_z', default_value='0.5')
    laser_sensor_x_arg = DeclareLaunchArgument('laser_sensor_x', default_value='0.0')
    laser_sensor_y_arg = DeclareLaunchArgument('laser_sensor_y', default_value='0.0')
    laser_sensor_z_arg = DeclareLaunchArgument('laser_sensor_z', default_value='0.0')
    laser_rpy_arg = DeclareLaunchArgument('laser_rpy', default_value='0 0 0')
    laser_frame_id_arg = DeclareLaunchArgument('laser_frame_id', default_value='laser0_laser')
    share_dir = get_package_share_directory('lrs_halmstad')
    gz_world = _gazebo_world_name(LaunchConfiguration('world'))

    uav_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, 'spawn_robot.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'name': LaunchConfiguration('uav_name'),
            'type': 'm100',
            'uav_mode': LaunchConfiguration('uav_mode'),
            'with_camera': 'true',
            'bridge_camera': 'true',
            'bridge_depth': LaunchConfiguration('bridge_depth'),
            'bridge_gimbal': LaunchConfiguration('bridge_gimbal'),
            'camera_pitch_offset_deg': LaunchConfiguration('camera_pitch_offset_deg'),
            'camera_update_rate': LaunchConfiguration('camera_update_rate'),
            'camera_name': LaunchConfiguration('camera_name'),
            'with_laser': LaunchConfiguration('with_laser'),
            'bridge_laser': LaunchConfiguration('bridge_laser'),
            'laser_name': LaunchConfiguration('laser_name'),
            'laser_update_rate': LaunchConfiguration('laser_update_rate'),
            'laser_min_range': LaunchConfiguration('laser_min_range'),
            'laser_max_range': LaunchConfiguration('laser_max_range'),
            'laser_angle_deg': LaunchConfiguration('laser_angle_deg'),
            'laser_x': LaunchConfiguration('laser_x'),
            'laser_y': LaunchConfiguration('laser_y'),
            'laser_z': LaunchConfiguration('laser_z'),
            'laser_sensor_x': LaunchConfiguration('laser_sensor_x'),
            'laser_sensor_y': LaunchConfiguration('laser_sensor_y'),
            'laser_sensor_z': LaunchConfiguration('laser_sensor_z'),
            'laser_rpy': LaunchConfiguration('laser_rpy'),
            'laser_frame_id': LaunchConfiguration('laser_frame_id'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'Y': LaunchConfiguration('yaw'),
        }.items(),
    )

    set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='uav_set_pose_bridge',
        arguments=[
            ['/world/', gz_world, '/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        ],
        output='screen',
    )

    return LaunchDescription([
        world_arg,
        uav_name_arg,
        uav_mode_arg,
        x_arg,
        y_arg,
        z_arg,
        yaw_arg,
        camera_name_arg,
        uav_camera_mode_arg,
        camera_pitch_offset_deg_arg,
        camera_update_rate_arg,
        bridge_depth_arg,
        bridge_gimbal_arg,
        with_laser_arg,
        bridge_laser_arg,
        laser_name_arg,
        laser_update_rate_arg,
        laser_min_range_arg,
        laser_max_range_arg,
        laser_angle_deg_arg,
        laser_x_arg,
        laser_y_arg,
        laser_z_arg,
        laser_sensor_x_arg,
        laser_sensor_y_arg,
        laser_sensor_z_arg,
        laser_rpy_arg,
        laser_frame_id_arg,
        set_pose_bridge,
        uav_spawn,
    ])
