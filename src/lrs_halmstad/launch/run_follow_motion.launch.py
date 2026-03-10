from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _estimator_condition():
    start_arg = LaunchConfiguration('start_leader_estimator')
    leader_mode = LaunchConfiguration('leader_mode')
    perception = LaunchConfiguration('leader_perception_enable')
    yolo_weights = LaunchConfiguration('yolo_weights')
    return IfCondition(
        PythonExpression([
            "(",
            "'", start_arg, "'.lower() in ('1','true','yes','on')",
            ") or (",
            "'", start_arg, "'.lower() == 'auto' and (",
            "'", leader_mode, "'.lower() in ('pose','estimate')",
            " or ",
            "'", perception, "'.lower() in ('1','true','yes','on')",
            " or ",
            "'", yolo_weights, "'.strip() != ''",
            "))",
        ])
    )


def _nav2_ugv_condition():
    ugv_mode = LaunchConfiguration('ugv_mode')
    return IfCondition(
        PythonExpression([
            "'",
            ugv_mode,
            "'.lower() == 'nav2'",
        ])
    )


def _external_ugv_condition():
    ugv_mode = LaunchConfiguration('ugv_mode')
    return IfCondition(
        PythonExpression([
            "'",
            ugv_mode,
            "'.lower() in ('external','none')",
        ])
    )


def _leader_odom_condition():
    leader_mode = LaunchConfiguration('leader_mode')
    return IfCondition(
        PythonExpression([
            "'",
            leader_mode,
            "'.lower() == 'odom'",
        ])
    )


def _leader_nonodom_condition():
    leader_mode = LaunchConfiguration('leader_mode')
    return IfCondition(
        PythonExpression([
            "'",
            leader_mode,
            "'.lower() != 'odom'",
        ])
    )


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


def _bool_param(name: str) -> ParameterValue:
    return ParameterValue(LaunchConfiguration(name), value_type=bool)


def _optional_bool_from_launch(context, name: str):
    raw = LaunchConfiguration(name).perform(context).strip().lower()
    if raw == "":
        return None
    if raw in ("1", "true", "yes", "on"):
        return True
    if raw in ("0", "false", "no", "off"):
        return False
    raise ValueError(
        f"{name} must be one of true/false/1/0/yes/no/on/off when provided; got {raw!r}"
    )


def _build_camera_tracker_node(context, *args, **kwargs):
    camera_params = {
        'uav_name': LaunchConfiguration('uav_name'),
        'leader_input_type': LaunchConfiguration('leader_mode'),
        'leader_odom_topic': LaunchConfiguration('ugv_odom_topic'),
        'leader_pose_topic': LaunchConfiguration('leader_pose_topic'),
        'leader_status_topic': '/coord/leader_estimate_status',
        'uav_camera_mode': LaunchConfiguration('uav_camera_mode'),
        'camera_mount_pitch_deg': LaunchConfiguration('camera_mount_pitch_deg'),
        'default_tilt_deg': LaunchConfiguration('camera_default_tilt_deg'),
        'camera_yaw_offset_deg': LaunchConfiguration('camera_yaw_offset_deg'),
        'camera_pan_sign': LaunchConfiguration('camera_pan_sign'),
    }
    pan_enable = _optional_bool_from_launch(context, 'pan_enable')
    if pan_enable is not None:
        camera_params['pan_enable'] = pan_enable
    tilt_enable = _optional_bool_from_launch(context, 'tilt_enable')
    if tilt_enable is not None:
        camera_params['tilt_enable'] = tilt_enable
    return [
        Node(
            package='lrs_halmstad',
            executable='camera_tracker',
            name='camera_tracker',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                camera_params,
            ],
        )
    ]


def generate_launch_description():
    params_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'run_follow_defaults.yaml']
    )
    warehouse_waypoints_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'warehouse_waypoints.yaml']
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_default,
        description='Parameter YAML for simulator, camera_tracker, leader_estimator, follow_uav, and ugv_nav2_driver',
    )
    world_arg = DeclareLaunchArgument('world', default_value='warehouse')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    leader_mode_arg = DeclareLaunchArgument('leader_mode', default_value='odom')
    leader_perception_enable_arg = DeclareLaunchArgument('leader_perception_enable', default_value='false')
    start_estimator_arg = DeclareLaunchArgument(
        'start_leader_estimator',
        default_value='auto',
        description="auto|true|false; auto starts estimator for pose/estimate, perception mode, or when yolo_weights is set",
    )
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
    ugv_namespace_arg = DeclareLaunchArgument('ugv_namespace', default_value='a201_0000')
    ugv_mode_arg = DeclareLaunchArgument(
        'ugv_mode',
        default_value='nav2',
        description="UGV mobility backend: 'nav2' for built-in NavigateToPose goals, or 'external' to leave UGV motion to another Nav2 goal source",
    )
    ugv_set_initial_pose_arg = DeclareLaunchArgument(
        'ugv_set_initial_pose',
        default_value='true',
        description='When ugv_mode:=nav2, publish /initialpose and wait for amcl_pose before sending goals',
    )
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
    leader_pose_topic_arg = DeclareLaunchArgument(
        'leader_pose_topic',
        default_value='/coord/leader_estimate',
    )
    ugv_odom_topic_arg = DeclareLaunchArgument(
        'ugv_odom_topic',
        default_value=['/', LaunchConfiguration('ugv_namespace'), '/amcl_pose_odom'],
    )
    leader_actual_pose_topic_arg = DeclareLaunchArgument(
        'leader_actual_pose_topic',
        default_value=['/', LaunchConfiguration('ugv_namespace'), '/amcl_pose_odom'],
    )
    leader_actual_pose_enable_arg = DeclareLaunchArgument(
        'leader_actual_pose_enable',
        default_value='true',
    )
    leader_image_topic_arg = DeclareLaunchArgument('leader_image_topic', default_value=['/', LaunchConfiguration('uav_name'), '/camera0/image_raw'])
    leader_camera_info_topic_arg = DeclareLaunchArgument('leader_camera_info_topic', default_value=['/', LaunchConfiguration('uav_name'), '/camera0/camera_info'])
    leader_depth_topic_arg = DeclareLaunchArgument('leader_depth_topic', default_value='')
    leader_uav_pose_topic_arg = DeclareLaunchArgument('leader_uav_pose_topic', default_value=['/', LaunchConfiguration('uav_name'), '/pose'])
    leader_range_mode_arg = DeclareLaunchArgument('leader_range_mode', default_value='ground')
    leader_constant_range_m_arg = DeclareLaunchArgument('leader_constant_range_m', default_value='5.0')
    target_class_name_arg = DeclareLaunchArgument('target_class_name', default_value='')
    target_class_id_arg = DeclareLaunchArgument('target_class_id', default_value='-1')
    yolo_weights_arg = DeclareLaunchArgument(
        'yolo_weights',
        default_value='',
    )
    yolo_device_arg = DeclareLaunchArgument('yolo_device', default_value='cpu')
    event_topic_arg = DeclareLaunchArgument('event_topic', default_value='/coord/events')
    ugv_start_delay_arg = DeclareLaunchArgument('ugv_start_delay_s', default_value='0.0')

    simulator_node = Node(
        package='lrs_halmstad',
        executable='simulator',
        name='uav_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_uav_simulator')),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'world': LaunchConfiguration('world'),
                'uav_name': LaunchConfiguration('uav_name'),
                'camera_mode': LaunchConfiguration('uav_camera_mode'),
                'start_x': LaunchConfiguration('uav_start_x'),
                'start_y': LaunchConfiguration('uav_start_y'),
                'start_z': LaunchConfiguration('uav_start_z'),
                'start_yaw_deg': LaunchConfiguration('uav_start_yaw_deg'),
                'camera_mount_pitch_deg': LaunchConfiguration('camera_mount_pitch_deg'),
                'camera_yaw_offset_deg': LaunchConfiguration('camera_yaw_offset_deg'),
                'camera_pan_sign': LaunchConfiguration('camera_pan_sign'),
            },
        ],
    )

    estimator_node = Node(
        package='lrs_halmstad',
        executable='leader_estimator',
        name='leader_estimator',
        output='screen',
        condition=_estimator_condition(),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'uav_name': LaunchConfiguration('uav_name'),
                'camera_topic': LaunchConfiguration('leader_image_topic'),
                'camera_info_topic': LaunchConfiguration('leader_camera_info_topic'),
                'depth_topic': LaunchConfiguration('leader_depth_topic'),
                'uav_pose_topic': LaunchConfiguration('leader_uav_pose_topic'),
                'range_mode': LaunchConfiguration('leader_range_mode'),
                'constant_range_m': LaunchConfiguration('leader_constant_range_m'),
                'leader_actual_pose_topic': LaunchConfiguration('leader_actual_pose_topic'),
                'leader_actual_pose_enable': _bool_param('leader_actual_pose_enable'),
                'target_class_name': LaunchConfiguration('target_class_name'),
                'target_class_id': LaunchConfiguration('target_class_id'),
                'device': LaunchConfiguration('yolo_device'),
                'yolo_weights': LaunchConfiguration('yolo_weights'),
                'event_topic': LaunchConfiguration('event_topic'),
            },
        ],
    )

    follow_odom_node = Node(
        package='lrs_halmstad',
        executable='follow_uav_odom',
        name='follow_uav',
        output='screen',
        condition=_leader_odom_condition(),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'world': LaunchConfiguration('world'),
                'uav_name': LaunchConfiguration('uav_name'),
                'leader_odom_topic': LaunchConfiguration('ugv_odom_topic'),
                'z_alt': LaunchConfiguration('uav_start_z'),
                'event_topic': LaunchConfiguration('event_topic'),
                'follow_yaw': _bool_param('follow_yaw'),
            },
        ],
    )

    follow_estimate_node = Node(
        package='lrs_halmstad',
        executable='follow_uav',
        name='follow_uav',
        output='screen',
        condition=_leader_nonodom_condition(),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'world': LaunchConfiguration('world'),
                'uav_name': LaunchConfiguration('uav_name'),
                'leader_input_type': LaunchConfiguration('leader_mode'),
                'leader_pose_topic': LaunchConfiguration('leader_pose_topic'),
                'z_alt': LaunchConfiguration('uav_start_z'),
                'follow_yaw': _bool_param('follow_yaw'),
                'startup_reposition_enable': _bool_param('startup_reposition_enable'),
            },
        ],
    )

    camera_tracker_node = OpaqueFunction(function=_build_camera_tracker_node)

    ugv_nav2_node = Node(
        package='lrs_halmstad',
        executable='ugv_nav2_driver',
        name='ugv_nav2_driver',
        namespace=LaunchConfiguration('ugv_namespace'),
        output='screen',
        condition=_nav2_ugv_condition(),
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'start_delay_s': LaunchConfiguration('ugv_start_delay_s'),
                'set_initial_pose_enable': _bool_param('ugv_set_initial_pose'),
                'initial_pose_x': LaunchConfiguration('ugv_initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('ugv_initial_pose_y'),
                'initial_pose_yaw_deg': LaunchConfiguration('ugv_initial_pose_yaw_deg'),
                'goal_sequence_csv': LaunchConfiguration('ugv_goal_sequence_csv'),
                'goal_sequence_file': LaunchConfiguration('ugv_goal_sequence_file'),
            },
        ],
    )

    ugv_amcl_to_odom_node = Node(
        package='lrs_halmstad',
        executable='pose_cov_to_odom',
        name='ugv_amcl_to_odom',
        namespace=LaunchConfiguration('ugv_namespace'),
        output='screen',
        parameters=[
            {
                'pose_topic': 'amcl_pose',
                'odom_topic': 'amcl_pose_odom',
                'frame_id': 'map',
                'child_frame_id': 'base_link',
                'copy_header_stamp': True,
            },
        ],
    )

    ugv_nav2_delayed_start = TimerAction(
        period=0.1,
        actions=[
            LogInfo(
                msg='[run_follow_motion] Starting Nav2-backed UGV motion driver',
                condition=_nav2_ugv_condition(),
            ),
            ugv_nav2_node,
        ],
    )

    ugv_external_info = TimerAction(
        period=0.1,
        actions=[
            LogInfo(
                msg='[run_follow_motion] UGV mobility backend disabled; expecting external Nav2 goal source',
                condition=_external_ugv_condition(),
            ),
        ],
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
        ugv_namespace_arg,
        ugv_mode_arg,
        ugv_set_initial_pose_arg,
        ugv_initial_pose_x_arg,
        ugv_initial_pose_y_arg,
        ugv_initial_pose_yaw_deg_arg,
        ugv_goal_sequence_csv_arg,
        ugv_goal_sequence_file_arg,
        leader_pose_topic_arg,
        ugv_odom_topic_arg,
        leader_actual_pose_topic_arg,
        leader_actual_pose_enable_arg,
        leader_image_topic_arg,
        leader_camera_info_topic_arg,
        leader_depth_topic_arg,
        leader_uav_pose_topic_arg,
        leader_range_mode_arg,
        leader_constant_range_m_arg,
        target_class_name_arg,
        target_class_id_arg,
        yolo_weights_arg,
        yolo_device_arg,
        event_topic_arg,
        ugv_start_delay_arg,
        simulator_node,
        ugv_amcl_to_odom_node,
        estimator_node,
        follow_odom_node,
        follow_estimate_node,
        camera_tracker_node,
        ugv_nav2_delayed_start,
        ugv_external_info,
    ])
