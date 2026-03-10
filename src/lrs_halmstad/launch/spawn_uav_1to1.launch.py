import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
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


def generate_launch_description():
    world_arg = DeclareLaunchArgument('world', default_value='warehouse')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    uav_mode_arg = DeclareLaunchArgument('uav_mode', default_value='teleport')
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='-2.0',
    )
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
    )
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='7.0',
    )
    camera_name_arg = DeclareLaunchArgument('camera_name', default_value='camera0')
    uav_camera_mode_arg = DeclareLaunchArgument('uav_camera_mode', default_value='detached_model')
    camera_pitch_offset_deg_arg = DeclareLaunchArgument('camera_pitch_offset_deg', default_value='45.0')
    camera_sensor_roll_deg_arg = DeclareLaunchArgument('camera_sensor_roll_deg', default_value='0.0')
    camera_sensor_pitch_deg_arg = DeclareLaunchArgument('camera_sensor_pitch_deg', default_value='0.0')
    camera_sensor_yaw_deg_arg = DeclareLaunchArgument('camera_sensor_yaw_deg', default_value='0.0')
    detached_spawn_delay_arg = DeclareLaunchArgument(
        'detached_spawn_delay_s',
        default_value='0.25',
        description='Delay between UAV body, detached camera, and camera bridge startup',
    )

    share_dir = get_package_share_directory('lrs_halmstad')
    gz_world = _gazebo_world_name(LaunchConfiguration('world'))
    integrated_camera_for_mode = PythonExpression([
        "'true' if '",
        LaunchConfiguration('uav_camera_mode'),
        "'.strip().lower() in ('integrated', 'integrated_joint') else 'false'",
    ])
    detached_camera_condition = IfCondition(PythonExpression([
        "'",
        LaunchConfiguration('uav_camera_mode'),
        "'.strip().lower() in ('detached', 'detached_model')",
    ]))

    uav_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, 'spawn_robot.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'name': LaunchConfiguration('uav_name'),
            'type': 'm100',
            'uav_mode': LaunchConfiguration('uav_mode'),
            'with_camera': integrated_camera_for_mode,
            'bridge_camera': integrated_camera_for_mode,
            'bridge_gimbal': integrated_camera_for_mode,
            'camera_pitch_offset_deg': LaunchConfiguration('camera_pitch_offset_deg'),
            'camera_name': LaunchConfiguration('camera_name'),
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
        }.items(),
    )

    camera_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(share_dir, 'spawn_gimbal.launch.py')),
        condition=detached_camera_condition,
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'name': LaunchConfiguration('uav_name'),
            'type': 'm100',
            'camera_name': LaunchConfiguration('camera_name'),
            'camera_sensor_roll_deg': LaunchConfiguration('camera_sensor_roll_deg'),
            'camera_sensor_pitch_deg': LaunchConfiguration('camera_sensor_pitch_deg'),
            'camera_sensor_yaw_deg': LaunchConfiguration('camera_sensor_yaw_deg'),
            'bridge_camera': 'false',
            'x': LaunchConfiguration('x'),
            'y': LaunchConfiguration('y'),
            'z': LaunchConfiguration('z'),
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

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        condition=detached_camera_condition,
        arguments=[
            [
                '/',
                LaunchConfiguration('uav_name'),
                '/',
                LaunchConfiguration('camera_name'),
                '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            ],
            [
                '/',
                LaunchConfiguration('uav_name'),
                '/',
                LaunchConfiguration('camera_name'),
                '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            ],
        ],
        output='screen',
    )

    camera_spawn_delayed = TimerAction(
        period=LaunchConfiguration('detached_spawn_delay_s'),
        actions=[camera_spawn],
    )

    camera_bridge_delayed = TimerAction(
        period=PythonExpression([
            str(2.0), " * ", LaunchConfiguration('detached_spawn_delay_s')
        ]),
        actions=[camera_bridge],
    )

    return LaunchDescription([
        world_arg,
        uav_name_arg,
        uav_mode_arg,
        x_arg,
        y_arg,
        z_arg,
        camera_name_arg,
        uav_camera_mode_arg,
        camera_pitch_offset_deg_arg,
        camera_sensor_roll_deg_arg,
        camera_sensor_pitch_deg_arg,
        camera_sensor_yaw_deg_arg,
        detached_spawn_delay_arg,
        set_pose_bridge,
        uav_spawn,
        camera_spawn_delayed,
        camera_bridge_delayed,
    ])
