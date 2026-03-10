from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare


def _default_world_value(world_sub, orchard_value: str, walls_value: str, warehouse_value: str, default_value: str = '0.0'):
    return PythonExpression([
        "'",
        warehouse_value,
        "' if '",
        world_sub,
        "'.startswith('warehouse') else '",
        walls_value,
        "' if '",
        world_sub,
        "' == 'walls' else '",
        orchard_value,
        "' if '",
        world_sub,
        "' == 'orchard' else '",
        default_value,
        "'",
    ])


def generate_launch_description():
    params_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'run_follow_defaults.yaml']
    )
    warehouse_waypoints_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'warehouse_waypoints.yaml']
    )
    follow_launch = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'run_follow_motion.launch.py']
    )

    params_file_arg = DeclareLaunchArgument('params_file', default_value=params_default)
    world_arg = DeclareLaunchArgument('world', default_value='warehouse')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    leader_mode_arg = DeclareLaunchArgument('leader_mode', default_value='odom')
    leader_perception_enable_arg = DeclareLaunchArgument('leader_perception_enable', default_value='false')
    start_estimator_arg = DeclareLaunchArgument('start_leader_estimator', default_value='auto')
    startup_reposition_enable_arg = DeclareLaunchArgument('startup_reposition_enable', default_value='false')
    follow_yaw_arg = DeclareLaunchArgument('follow_yaw', default_value='false')
    uav_start_x_arg = DeclareLaunchArgument(
        'uav_start_x',
        default_value='-2.0',
    )
    uav_start_y_arg = DeclareLaunchArgument(
        'uav_start_y',
        default_value='0.0',
    )
    uav_start_z_arg = DeclareLaunchArgument('uav_start_z', default_value='7.0')
    uav_start_yaw_deg_arg = DeclareLaunchArgument('uav_start_yaw_deg', default_value='0.0')
    camera_mount_pitch_deg_arg = DeclareLaunchArgument('camera_mount_pitch_deg', default_value='45.0')
    camera_default_tilt_deg_arg = DeclareLaunchArgument('camera_default_tilt_deg', default_value='-45.0')
    pan_enable_arg = DeclareLaunchArgument('pan_enable', default_value='')
    tilt_enable_arg = DeclareLaunchArgument('tilt_enable', default_value='')
    camera_yaw_offset_deg_arg = DeclareLaunchArgument('camera_yaw_offset_deg', default_value='0.0')
    camera_pan_sign_arg = DeclareLaunchArgument('camera_pan_sign', default_value='1.0')
    start_uav_simulator_arg = DeclareLaunchArgument('start_uav_simulator', default_value='true')
    uav_camera_mode_arg = DeclareLaunchArgument('uav_camera_mode', default_value='detached_model')
    ugv_mode_arg = DeclareLaunchArgument(
        'ugv_mode',
        default_value='nav2',
        description="UGV mobility backend: 'nav2' or 'external' when another Nav2 goal source is used",
    )
    ugv_set_initial_pose_arg = DeclareLaunchArgument('ugv_set_initial_pose', default_value='true')
    ugv_initial_pose_x_arg = DeclareLaunchArgument(
        'ugv_initial_pose_x',
        default_value=_default_world_value(LaunchConfiguration('world'), '0.449', '-0.048', '0.0'),
    )
    ugv_initial_pose_y_arg = DeclareLaunchArgument(
        'ugv_initial_pose_y',
        default_value=_default_world_value(LaunchConfiguration('world'), '0.139', '-0.179', '0.0'),
    )
    ugv_initial_pose_yaw_deg_arg = DeclareLaunchArgument(
        'ugv_initial_pose_yaw_deg',
        default_value=_default_world_value(LaunchConfiguration('world'), '-4.6', '-53.9', '0.0'),
    )
    ugv_goal_sequence_csv_arg = DeclareLaunchArgument(
        'ugv_goal_sequence_csv',
        default_value='',
    )
    ugv_goal_sequence_file_arg = DeclareLaunchArgument(
        'ugv_goal_sequence_file',
        default_value=warehouse_waypoints_default,
    )
    yolo_weights_arg = DeclareLaunchArgument(
        'yolo_weights',
        default_value='',
    )
    leader_actual_pose_topic_arg = DeclareLaunchArgument(
        'leader_actual_pose_topic',
        default_value='/a201_0000/amcl_pose_odom',
    )
    leader_actual_pose_enable_arg = DeclareLaunchArgument(
        'leader_actual_pose_enable',
        default_value='true',
    )
    leader_range_mode_arg = DeclareLaunchArgument('leader_range_mode', default_value='ground')
    leader_constant_range_m_arg = DeclareLaunchArgument('leader_constant_range_m', default_value='5.0')
    target_class_name_arg = DeclareLaunchArgument('target_class_name', default_value='')
    target_class_id_arg = DeclareLaunchArgument('target_class_id', default_value='-1')
    yolo_device_arg = DeclareLaunchArgument('yolo_device', default_value='cpu')
    event_topic_arg = DeclareLaunchArgument('event_topic', default_value='/coord/events')
    ugv_start_delay_arg = DeclareLaunchArgument('ugv_start_delay_s', default_value='0.0')

    follow = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(follow_launch),
        launch_arguments={
            'params_file': LaunchConfiguration('params_file'),
            'world': LaunchConfiguration('world'),
            'uav_name': LaunchConfiguration('uav_name'),
            'leader_mode': LaunchConfiguration('leader_mode'),
            'leader_perception_enable': LaunchConfiguration('leader_perception_enable'),
            'start_leader_estimator': LaunchConfiguration('start_leader_estimator'),
            'startup_reposition_enable': LaunchConfiguration('startup_reposition_enable'),
            'follow_yaw': LaunchConfiguration('follow_yaw'),
            'uav_start_x': LaunchConfiguration('uav_start_x'),
            'uav_start_y': LaunchConfiguration('uav_start_y'),
            'uav_start_z': LaunchConfiguration('uav_start_z'),
            'uav_start_yaw_deg': LaunchConfiguration('uav_start_yaw_deg'),
            'camera_mount_pitch_deg': LaunchConfiguration('camera_mount_pitch_deg'),
            'camera_default_tilt_deg': LaunchConfiguration('camera_default_tilt_deg'),
            'pan_enable': LaunchConfiguration('pan_enable'),
            'tilt_enable': LaunchConfiguration('tilt_enable'),
            'camera_yaw_offset_deg': LaunchConfiguration('camera_yaw_offset_deg'),
            'camera_pan_sign': LaunchConfiguration('camera_pan_sign'),
            'start_uav_simulator': LaunchConfiguration('start_uav_simulator'),
            'uav_camera_mode': LaunchConfiguration('uav_camera_mode'),
            'ugv_mode': LaunchConfiguration('ugv_mode'),
            'ugv_set_initial_pose': LaunchConfiguration('ugv_set_initial_pose'),
            'ugv_initial_pose_x': LaunchConfiguration('ugv_initial_pose_x'),
            'ugv_initial_pose_y': LaunchConfiguration('ugv_initial_pose_y'),
            'ugv_initial_pose_yaw_deg': LaunchConfiguration('ugv_initial_pose_yaw_deg'),
            'ugv_goal_sequence_csv': LaunchConfiguration('ugv_goal_sequence_csv'),
            'ugv_goal_sequence_file': LaunchConfiguration('ugv_goal_sequence_file'),
            'ugv_namespace': 'a201_0000',
            'leader_uav_pose_topic': ['/', LaunchConfiguration('uav_name'), '/pose'],
            'yolo_weights': LaunchConfiguration('yolo_weights'),
            'leader_actual_pose_topic': LaunchConfiguration('leader_actual_pose_topic'),
            'leader_actual_pose_enable': LaunchConfiguration('leader_actual_pose_enable'),
            'leader_range_mode': LaunchConfiguration('leader_range_mode'),
            'leader_constant_range_m': LaunchConfiguration('leader_constant_range_m'),
            'target_class_name': LaunchConfiguration('target_class_name'),
            'target_class_id': LaunchConfiguration('target_class_id'),
            'yolo_device': LaunchConfiguration('yolo_device'),
            'event_topic': LaunchConfiguration('event_topic'),
            'ugv_start_delay_s': LaunchConfiguration('ugv_start_delay_s'),
        }.items(),
    )

    return LaunchDescription([
        params_file_arg,
        world_arg,
        uav_name_arg,
        leader_mode_arg,
        leader_perception_enable_arg,
        start_estimator_arg,
        startup_reposition_enable_arg,
        follow_yaw_arg,
        uav_start_x_arg,
        uav_start_y_arg,
        uav_start_z_arg,
        uav_start_yaw_deg_arg,
        camera_mount_pitch_deg_arg,
        camera_default_tilt_deg_arg,
        pan_enable_arg,
        tilt_enable_arg,
        camera_yaw_offset_deg_arg,
        camera_pan_sign_arg,
        start_uav_simulator_arg,
        uav_camera_mode_arg,
        ugv_mode_arg,
        ugv_set_initial_pose_arg,
        ugv_initial_pose_x_arg,
        ugv_initial_pose_y_arg,
        ugv_initial_pose_yaw_deg_arg,
        ugv_goal_sequence_csv_arg,
        ugv_goal_sequence_file_arg,
        yolo_weights_arg,
        leader_actual_pose_topic_arg,
        leader_actual_pose_enable_arg,
        leader_range_mode_arg,
        leader_constant_range_m_arg,
        target_class_name_arg,
        target_class_id_arg,
        yolo_device_arg,
        event_topic_arg,
        ugv_start_delay_arg,
        follow,
    ])
