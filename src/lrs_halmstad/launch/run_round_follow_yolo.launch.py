from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _estimator_condition():
    start_arg = LaunchConfiguration('start_leader_estimator')
    leader_mode = LaunchConfiguration('leader_mode')
    perception = LaunchConfiguration('leader_perception_enable')
    yolo_version = LaunchConfiguration('yolo_version')
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
            "'", yolo_version, "'.strip() != ''",
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


def generate_launch_description():
    yolo_models_dir = '/home/ruben/halmstad_ws/models'
    params_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'run_round_follow_defaults.yaml']
    )
    warehouse_waypoints_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'warehouse_waypoints.yaml']
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_default,
        description='Parameter YAML for leader_estimator, follow_uav, and the selected UGV driver',
    )
    world_arg = DeclareLaunchArgument('world', default_value='orchard')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    leader_mode_arg = DeclareLaunchArgument(
        'leader_mode',
        default_value='estimate',
        description="Leader input for follow_uav: 'estimate' (via /coord topics), 'pose', or 'odom'",
    )
    leader_perception_enable_arg = DeclareLaunchArgument(
        'leader_perception_enable',
        default_value='true',
        description='Enable perception/YOLO-oriented estimator startup in auto mode',
    )
    start_estimator_arg = DeclareLaunchArgument(
        'start_leader_estimator',
        default_value='auto',
        description="auto|true|false; auto starts estimator for pose/estimate, perception mode, or when yolo_version is set",
    )
    uav_start_x_arg = DeclareLaunchArgument('uav_start_x', default_value='-2.0')
    uav_start_y_arg = DeclareLaunchArgument('uav_start_y', default_value='0.0')
    uav_start_z_arg = DeclareLaunchArgument('uav_start_z', default_value='7.0')
    uav_start_yaw_deg_arg = DeclareLaunchArgument('uav_start_yaw_deg', default_value='0.0')
    camera_mount_pitch_deg_arg = DeclareLaunchArgument('camera_mount_pitch_deg', default_value='45.0')
    start_uav_simulator_arg = DeclareLaunchArgument('start_uav_simulator', default_value='true')
    uav_camera_mode_arg = DeclareLaunchArgument('uav_camera_mode', default_value='integrated_joint')
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
        default_value=['/', LaunchConfiguration('ugv_namespace'), '/platform/odom'],
    )
    leader_image_topic_arg = DeclareLaunchArgument('leader_image_topic', default_value=['/', LaunchConfiguration('uav_name'), '/camera0/image_raw'])
    leader_camera_info_topic_arg = DeclareLaunchArgument('leader_camera_info_topic', default_value=['/', LaunchConfiguration('uav_name'), '/camera0/camera_info'])
    leader_depth_topic_arg = DeclareLaunchArgument('leader_depth_topic', default_value='')
    leader_uav_pose_topic_arg = DeclareLaunchArgument('leader_uav_pose_topic', default_value=['/', LaunchConfiguration('uav_name'), '/pose'])
    leader_range_mode_arg = DeclareLaunchArgument('leader_range_mode', default_value='ground')
    leader_constant_range_m_arg = DeclareLaunchArgument('leader_constant_range_m', default_value='5.0')
    target_class_name_arg = DeclareLaunchArgument('target_class_name', default_value='')
    target_class_id_arg = DeclareLaunchArgument('target_class_id', default_value='-1')
    yolo_version_arg = DeclareLaunchArgument(
        'yolo_version',
        default_value='detection/yolo26/yolo26l.pt',
        description=(
            f"YOLO weights path relative to {yolo_models_dir} "
            "(for example detection/yolo5/yolov5su.pt or detection/yolo26/yolo26l.pt)"
        ),
    )
    yolo_weights_path = PathJoinSubstitution([yolo_models_dir, LaunchConfiguration('yolo_version')])
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
                'target_class_name': LaunchConfiguration('target_class_name'),
                'target_class_id': LaunchConfiguration('target_class_id'),
                'device': LaunchConfiguration('yolo_device'),
                'yolo_weights': yolo_weights_path,
                'yolo_backend': 'ultralytics',
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
            },
        ],
    )

    camera_tracker_node = Node(
        package='lrs_halmstad',
        executable='camera_tracker',
        name='camera_tracker',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'uav_name': LaunchConfiguration('uav_name'),
                'leader_input_type': LaunchConfiguration('leader_mode'),
                'leader_odom_topic': LaunchConfiguration('ugv_odom_topic'),
                'leader_pose_topic': LaunchConfiguration('leader_pose_topic'),
                'uav_camera_mode': LaunchConfiguration('uav_camera_mode'),
                'camera_mount_pitch_deg': LaunchConfiguration('camera_mount_pitch_deg'),
            },
        ],
    )

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
                'set_initial_pose_enable': LaunchConfiguration('ugv_set_initial_pose'),
                'initial_pose_x': LaunchConfiguration('ugv_initial_pose_x'),
                'initial_pose_y': LaunchConfiguration('ugv_initial_pose_y'),
                'initial_pose_yaw_deg': LaunchConfiguration('ugv_initial_pose_yaw_deg'),
                'goal_sequence_csv': LaunchConfiguration('ugv_goal_sequence_csv'),
                'goal_sequence_file': LaunchConfiguration('ugv_goal_sequence_file'),
            },
        ],
    )

    ugv_nav2_delayed_start = TimerAction(
        period=0.1,
        actions=[
            LogInfo(
                msg='[run_round_follow_yolo] Starting Nav2-backed UGV motion driver',
                condition=_nav2_ugv_condition(),
            ),
            ugv_nav2_node,
        ],
    )

    return LaunchDescription([
        params_file_arg,
        world_arg,
        uav_name_arg,
        leader_mode_arg,
        leader_perception_enable_arg,
        start_estimator_arg,
        uav_start_x_arg,
        uav_start_y_arg,
        uav_start_z_arg,
        uav_start_yaw_deg_arg,
        camera_mount_pitch_deg_arg,
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
        leader_image_topic_arg,
        leader_camera_info_topic_arg,
        leader_depth_topic_arg,
        leader_uav_pose_topic_arg,
        leader_range_mode_arg,
        leader_constant_range_m_arg,
        target_class_name_arg,
        target_class_id_arg,
        yolo_version_arg,
        yolo_device_arg,
        event_topic_arg,
        ugv_start_delay_arg,
        simulator_node,
        estimator_node,
        follow_odom_node,
        follow_estimate_node,
        camera_tracker_node,
        ugv_nav2_delayed_start,
    ])
