import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml


def _support_instance(
    *,
    instance_id,
    share_dir,
    params_file,
    world,
    uav_mode,
    leader_odom_topic,
    uav_name,
    start_x,
    start_y,
    start_z,
    start_yaw_deg,
    d_target,
    forward_offset_m,
    lateral_offset_m,
    with_camera,
    bridge_gimbal,
    camera_pitch_offset_deg,
    camera_update_rate,
):
    spawn_group = GroupAction([
        PushRosNamespace(f'support_follow_spawn_{instance_id}'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(share_dir, 'spawn_robot.launch.py')
            ),
            launch_arguments={
                'world': world,
                'name': uav_name,
                'type': 'm100',
                'uav_mode': uav_mode,
                'with_camera': with_camera,
                'bridge_camera': with_camera,
                'bridge_gimbal': bridge_gimbal,
                'camera_pitch_offset_deg': camera_pitch_offset_deg,
                'camera_update_rate': camera_update_rate,
                'x': start_x,
                'y': start_y,
                'z': start_z,
                'Y': start_yaw_deg,
            }.items(),
        ),
    ])

    simulator_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={},
        key_rewrites={'uav_simulator': f'support_follow_{instance_id}_simulator'},
        convert_types=True,
    )

    follow_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'uav_start_x': start_x,
            'uav_start_y': start_y,
            'uav_start_z': start_z,
            'uav_start_yaw_deg': start_yaw_deg,
            'event_topic': f'/coord/support/{instance_id}/follow_odom/events',
            'follow_yaw': 'true',
            'publish_debug_topics': 'true',
            'publish_pose_cmd_topics': 'true',
            'd_target': d_target,
            'forward_offset_m': forward_offset_m,
            'lateral_offset_m': lateral_offset_m,
        },
        key_rewrites={'follow_uav': f'support_follow_{instance_id}_odom_controller'},
        convert_types=True,
    )

    simulator_node = Node(
        package='lrs_halmstad',
        executable='simulator',
        name=f'support_follow_{instance_id}_simulator',
        output='screen',
        parameters=[
            simulator_params,
            {
                'use_sim_time': True,
                'world': world,
                'uav_name': uav_name,
                'camera_mode': 'integrated_joint',
                'start_x': start_x,
                'start_y': start_y,
                'start_z': start_z,
                'start_yaw_deg': start_yaw_deg,
            },
        ],
    )

    follow_node = Node(
        package='lrs_halmstad',
        executable='follow_uav_odom',
        name=f'support_follow_{instance_id}_odom_controller',
        output='screen',
        parameters=[
            follow_params,
            {
                'use_sim_time': True,
                'world': world,
                'uav_name': uav_name,
                'leader_odom_topic': leader_odom_topic,
                'uav_start_x': start_x,
                'uav_start_y': start_y,
                'uav_start_z': start_z,
                'uav_start_yaw_deg': start_yaw_deg,
                'event_topic': f'/coord/support/{instance_id}/follow_odom/events',
                'follow_yaw': True,
                'publish_debug_topics': True,
                'publish_pose_cmd_topics': True,
                'd_target': d_target,
                'forward_offset_m': forward_offset_m,
                'lateral_offset_m': lateral_offset_m,
            },
        ],
    )

    return [spawn_group, simulator_node, follow_node]


def _slot_axis_expression(
    *,
    axis: str,
    leader_start_x,
    leader_start_y,
    leader_start_yaw_deg,
    forward_offset_m,
    lateral_offset_m,
):
    if axis == 'x':
        return PythonExpression([
            '(',
            leader_start_x,
            ') + math.cos(math.radians(',
            leader_start_yaw_deg,
            ')) * (',
            forward_offset_m,
            ') - math.sin(math.radians(',
            leader_start_yaw_deg,
            ')) * (',
            lateral_offset_m,
            ')',
        ], ['math'])
    if axis == 'y':
        return PythonExpression([
            '(',
            leader_start_y,
            ') + math.sin(math.radians(',
            leader_start_yaw_deg,
            ')) * (',
            forward_offset_m,
            ') + math.cos(math.radians(',
            leader_start_yaw_deg,
            ')) * (',
            lateral_offset_m,
            ')',
        ], ['math'])
    raise ValueError(f"Unsupported slot axis: {axis}")


def generate_launch_description():
    share_dir = get_package_share_directory('lrs_halmstad')
    default_params_file = os.path.join(share_dir, 'config', 'run_follow_defaults.yaml')

    world_arg = DeclareLaunchArgument('world', default_value='warehouse')
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Parameter YAML reused from the trusted 1-to-1 baseline.',
    )
    leader_pose_topic_arg = DeclareLaunchArgument(
        'leader_pose_topic',
        default_value='/dji0/pose',
        description='Actual pose topic for the main UAV (UAV1).',
    )
    leader_odom_topic_arg = DeclareLaunchArgument(
        'leader_odom_topic',
        default_value='/dji0/pose/odom',
        description='Synthetic odom topic generated from UAV1 actual pose.',
    )
    leader_nominal_z_arg = DeclareLaunchArgument(
        'leader_nominal_z',
        default_value='7.0',
        description='Nominal main-UAV altitude used to derive support hover altitude.',
    )
    leader_start_x_arg = DeclareLaunchArgument(
        'leader_start_x',
        default_value='-7.0',
        description='Expected main-UAV startup X used to place support UAVs into their initial slot geometry.',
    )
    leader_start_y_arg = DeclareLaunchArgument(
        'leader_start_y',
        default_value='0.0',
        description='Expected main-UAV startup Y used to place support UAVs into their initial slot geometry.',
    )
    leader_start_yaw_deg_arg = DeclareLaunchArgument(
        'leader_start_yaw_deg',
        default_value='0.0',
        description='Expected main-UAV startup yaw used to rotate the support slot geometry.',
    )
    support_vertical_offset_m_arg = DeclareLaunchArgument(
        'support_vertical_offset_m',
        default_value='0.0',
        description='Explicit shared altitude offset for both support UAVs relative to UAV1 nominal altitude.',
    )
    uav_mode_arg = DeclareLaunchArgument(
        'uav_mode',
        default_value='teleport',
        description='UAV mode passed to the support spawn path.',
    )
    support_with_camera_arg = DeclareLaunchArgument(
        'support_with_camera',
        default_value='false',
        description='Attach and bridge camera sensors for support UAVs so observation overlays can subscribe to image topics.',
    )
    support_camera_pitch_offset_deg_arg = DeclareLaunchArgument(
        'support_camera_pitch_offset_deg',
        default_value='30.0',
        description='Support UAV camera mount pitch offset in degrees when support_with_camera:=true.',
    )
    support_camera_update_rate_arg = DeclareLaunchArgument(
        'support_camera_update_rate',
        default_value='5',
        description='Support UAV camera sensor update rate in Hz when support_with_camera:=true.',
    )
    support_bridge_gimbal_arg = DeclareLaunchArgument(
        'support_bridge_gimbal',
        default_value='false',
        description='Bridge support UAV gimbal joint commands when optional camera scanning is enabled.',
    )
    dji1_name_arg = DeclareLaunchArgument('dji1_name', default_value='dji1')
    dji1_d_target_arg = DeclareLaunchArgument(
        'dji1_d_target',
        default_value='8.0',
        description='3D follow distance for the first support UAV.',
    )
    dji1_forward_offset_m_arg = DeclareLaunchArgument(
        'dji1_forward_offset_m',
        default_value='4.0',
        description='Forward slot offset for the first support UAV in the leader-heading frame.',
    )
    dji1_lateral_offset_m_arg = DeclareLaunchArgument(
        'dji1_lateral_offset_m',
        default_value='2.5',
        description='Left/right slot offset for the first support UAV in the leader-heading frame.',
    )
    dji1_start_x_arg = DeclareLaunchArgument(
        'dji1_start_x',
        default_value=_slot_axis_expression(
            axis='x',
            leader_start_x=LaunchConfiguration('leader_start_x'),
            leader_start_y=LaunchConfiguration('leader_start_y'),
            leader_start_yaw_deg=LaunchConfiguration('leader_start_yaw_deg'),
            forward_offset_m=LaunchConfiguration('dji1_forward_offset_m'),
            lateral_offset_m=LaunchConfiguration('dji1_lateral_offset_m'),
        ),
    )
    dji1_start_y_arg = DeclareLaunchArgument(
        'dji1_start_y',
        default_value=_slot_axis_expression(
            axis='y',
            leader_start_x=LaunchConfiguration('leader_start_x'),
            leader_start_y=LaunchConfiguration('leader_start_y'),
            leader_start_yaw_deg=LaunchConfiguration('leader_start_yaw_deg'),
            forward_offset_m=LaunchConfiguration('dji1_forward_offset_m'),
            lateral_offset_m=LaunchConfiguration('dji1_lateral_offset_m'),
        ),
    )
    dji1_start_z_arg = DeclareLaunchArgument(
        'dji1_start_z',
        default_value=PythonExpression([
            LaunchConfiguration('leader_nominal_z'),
            ' + ',
            LaunchConfiguration('support_vertical_offset_m'),
        ]),
    )
    dji1_start_yaw_deg_arg = DeclareLaunchArgument(
        'dji1_start_yaw_deg',
        default_value=LaunchConfiguration('leader_start_yaw_deg'),
    )
    dji2_name_arg = DeclareLaunchArgument('dji2_name', default_value='dji2')
    dji2_d_target_arg = DeclareLaunchArgument(
        'dji2_d_target',
        default_value='8.0',
        description='3D follow distance for the second support UAV.',
    )
    dji2_forward_offset_m_arg = DeclareLaunchArgument(
        'dji2_forward_offset_m',
        default_value='4.0',
        description='Forward slot offset for the second support UAV in the leader-heading frame.',
    )
    dji2_lateral_offset_m_arg = DeclareLaunchArgument(
        'dji2_lateral_offset_m',
        default_value='-2.5',
        description='Left/right slot offset for the second support UAV in the leader-heading frame.',
    )
    dji2_start_x_arg = DeclareLaunchArgument(
        'dji2_start_x',
        default_value=_slot_axis_expression(
            axis='x',
            leader_start_x=LaunchConfiguration('leader_start_x'),
            leader_start_y=LaunchConfiguration('leader_start_y'),
            leader_start_yaw_deg=LaunchConfiguration('leader_start_yaw_deg'),
            forward_offset_m=LaunchConfiguration('dji2_forward_offset_m'),
            lateral_offset_m=LaunchConfiguration('dji2_lateral_offset_m'),
        ),
    )
    dji2_start_y_arg = DeclareLaunchArgument(
        'dji2_start_y',
        default_value=_slot_axis_expression(
            axis='y',
            leader_start_x=LaunchConfiguration('leader_start_x'),
            leader_start_y=LaunchConfiguration('leader_start_y'),
            leader_start_yaw_deg=LaunchConfiguration('leader_start_yaw_deg'),
            forward_offset_m=LaunchConfiguration('dji2_forward_offset_m'),
            lateral_offset_m=LaunchConfiguration('dji2_lateral_offset_m'),
        ),
    )
    dji2_start_z_arg = DeclareLaunchArgument(
        'dji2_start_z',
        default_value=PythonExpression([
            LaunchConfiguration('leader_nominal_z'),
            ' + ',
            LaunchConfiguration('support_vertical_offset_m'),
        ]),
    )
    dji2_start_yaw_deg_arg = DeclareLaunchArgument(
        'dji2_start_yaw_deg',
        default_value=LaunchConfiguration('leader_start_yaw_deg'),
    )

    leader_pose_to_odom = Node(
        package='lrs_halmstad',
        executable='pose_cmd_to_odom',
        name='support_follow_dji0_pose_to_odom',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'pose_topic': LaunchConfiguration('leader_pose_topic'),
            'odom_topic': LaunchConfiguration('leader_odom_topic'),
            'frame_id': 'map',
            'child_frame_id': 'base_link',
            'copy_header_stamp': True,
        }],
    )
    support_dji1_actions = _support_instance(
        instance_id='dji1',
        share_dir=share_dir,
        params_file=LaunchConfiguration('params_file'),
        world=LaunchConfiguration('world'),
        uav_mode=LaunchConfiguration('uav_mode'),
        leader_odom_topic=LaunchConfiguration('leader_odom_topic'),
        uav_name=LaunchConfiguration('dji1_name'),
        start_x=LaunchConfiguration('dji1_start_x'),
        start_y=LaunchConfiguration('dji1_start_y'),
        start_z=LaunchConfiguration('dji1_start_z'),
        start_yaw_deg=LaunchConfiguration('dji1_start_yaw_deg'),
        d_target=LaunchConfiguration('dji1_d_target'),
        forward_offset_m=LaunchConfiguration('dji1_forward_offset_m'),
        lateral_offset_m=LaunchConfiguration('dji1_lateral_offset_m'),
        with_camera=LaunchConfiguration('support_with_camera'),
        bridge_gimbal=LaunchConfiguration('support_bridge_gimbal'),
        camera_pitch_offset_deg=LaunchConfiguration('support_camera_pitch_offset_deg'),
        camera_update_rate=LaunchConfiguration('support_camera_update_rate'),
    )
    support_dji2_actions = _support_instance(
        instance_id='dji2',
        share_dir=share_dir,
        params_file=LaunchConfiguration('params_file'),
        world=LaunchConfiguration('world'),
        uav_mode=LaunchConfiguration('uav_mode'),
        leader_odom_topic=LaunchConfiguration('leader_odom_topic'),
        uav_name=LaunchConfiguration('dji2_name'),
        start_x=LaunchConfiguration('dji2_start_x'),
        start_y=LaunchConfiguration('dji2_start_y'),
        start_z=LaunchConfiguration('dji2_start_z'),
        start_yaw_deg=LaunchConfiguration('dji2_start_yaw_deg'),
        d_target=LaunchConfiguration('dji2_d_target'),
        forward_offset_m=LaunchConfiguration('dji2_forward_offset_m'),
        lateral_offset_m=LaunchConfiguration('dji2_lateral_offset_m'),
        with_camera=LaunchConfiguration('support_with_camera'),
        bridge_gimbal=LaunchConfiguration('support_bridge_gimbal'),
        camera_pitch_offset_deg=LaunchConfiguration('support_camera_pitch_offset_deg'),
        camera_update_rate=LaunchConfiguration('support_camera_update_rate'),
    )

    return LaunchDescription([
        world_arg,
        params_file_arg,
        leader_pose_topic_arg,
        leader_odom_topic_arg,
        leader_nominal_z_arg,
        leader_start_x_arg,
        leader_start_y_arg,
        leader_start_yaw_deg_arg,
        support_vertical_offset_m_arg,
        uav_mode_arg,
        support_with_camera_arg,
        support_camera_pitch_offset_deg_arg,
        support_camera_update_rate_arg,
        support_bridge_gimbal_arg,
        dji1_name_arg,
        dji1_d_target_arg,
        dji1_forward_offset_m_arg,
        dji1_lateral_offset_m_arg,
        dji1_start_x_arg,
        dji1_start_y_arg,
        dji1_start_z_arg,
        dji1_start_yaw_deg_arg,
        dji2_name_arg,
        dji2_d_target_arg,
        dji2_forward_offset_m_arg,
        dji2_lateral_offset_m_arg,
        dji2_start_x_arg,
        dji2_start_y_arg,
        dji2_start_z_arg,
        dji2_start_yaw_deg_arg,
        leader_pose_to_odom,
        *support_dji1_actions,
        *support_dji2_actions,
    ])
