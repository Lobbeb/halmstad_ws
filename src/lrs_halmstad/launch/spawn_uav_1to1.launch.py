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
    world_arg = DeclareLaunchArgument('world', default_value='warehouse')
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
            'bridge_gimbal': 'true',
            'camera_pitch_offset_deg': LaunchConfiguration('camera_pitch_offset_deg'),
            'camera_update_rate': LaunchConfiguration('camera_update_rate'),
            'camera_name': LaunchConfiguration('camera_name'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
            'Y': LaunchConfiguration('yaw'),
        }.items(),
    )

    set_pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
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
        set_pose_bridge,
        uav_spawn,
    ])
