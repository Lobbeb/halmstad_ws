import yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from nav2_common.launch import RewrittenYaml
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
    start_bridge = LaunchConfiguration('start_visual_actuation_bridge')
    return IfCondition(
        PythonExpression([
            "'",
            leader_mode,
            "'.lower() == 'odom' and '",
            start_bridge,
            "'.lower() not in ('1','true','yes','on')",
        ])
    )


def _leader_nonodom_condition():
    leader_mode = LaunchConfiguration('leader_mode')
    start_bridge = LaunchConfiguration('start_visual_actuation_bridge')
    return IfCondition(
        PythonExpression([
            "'",
            leader_mode,
            "'.lower() != 'odom' and '",
            start_bridge,
            "'.lower() not in ('1','true','yes','on')",
        ])
    )


def _visual_pipeline_condition():
    start_visual = LaunchConfiguration('start_visual_follow_controller')
    start_follow_point = LaunchConfiguration('start_visual_follow_point_generator')
    start_planner = LaunchConfiguration('start_visual_follow_planner')
    start_bridge = LaunchConfiguration('start_visual_actuation_bridge')
    return IfCondition(
        PythonExpression([
            "(",
            "'",
            start_visual,
            "'.lower() in ('1','true','yes','on')",
            ") or (",
            "'",
            start_follow_point,
            "'.lower() in ('1','true','yes','on')",
            ") or (",
            "'",
            start_planner,
            "'.lower() in ('1','true','yes','on')",
            ") or (",
            "'",
            start_bridge,
            "'.lower() in ('1','true','yes','on')",
            ")",
        ])
    )


def _visual_controller_condition():
    start_visual = LaunchConfiguration('start_visual_follow_controller')
    start_bridge = LaunchConfiguration('start_visual_actuation_bridge')
    start_follow_point = LaunchConfiguration('start_visual_follow_point_generator')
    start_planner = LaunchConfiguration('start_visual_follow_planner')
    return IfCondition(
        PythonExpression([
            "(",
            "'",
            start_visual,
            "'.lower() in ('1','true','yes','on')",
            ") or (",
            "'",
            start_bridge,
            "'.lower() in ('1','true','yes','on') and '",
            start_follow_point,
            "'.lower() not in ('1','true','yes','on') and '",
            start_planner,
            "'.lower() not in ('1','true','yes','on')",
            ")",
        ])
    )


def _follow_point_generator_condition():
    start_follow_point = LaunchConfiguration('start_visual_follow_point_generator')
    start_planner = LaunchConfiguration('start_visual_follow_planner')
    return IfCondition(
        PythonExpression([
            "'",
            start_follow_point,
            "'.lower() in ('1','true','yes','on') or '",
            start_planner,
            "'.lower() in ('1','true','yes','on')",
        ])
    )


def _follow_planner_condition():
    start_planner = LaunchConfiguration('start_visual_follow_planner')
    return IfCondition(
        PythonExpression([
            "'",
            start_planner,
            "'.lower() in ('1','true','yes','on')",
        ])
    )


def _external_perception_condition(node_name: str):
    enabled = LaunchConfiguration('external_detection_enable')
    selected = LaunchConfiguration('external_detection_node')
    return IfCondition(
        PythonExpression([
            "'",
            enabled,
            "'.lower() in ('1','true','yes','on') and '",
            selected,
            "'.lower() == '",
            node_name.lower(),
            "'",
        ])
    )


def _default_world_value(
    world_sub,
    orchard_value: str,
    walls_value: str,
    warehouse_value: str,
    default_value: str = '0.0',
):
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
        'use_sim_time': True,
        'uav_name': LaunchConfiguration('uav_name'),
        'leader_input_type': LaunchConfiguration('leader_mode'),
        'leader_odom_topic': LaunchConfiguration('ugv_odom_topic'),
        'leader_pose_topic': LaunchConfiguration('leader_pose_topic'),
        'leader_actual_pose_topic': LaunchConfiguration('camera_leader_actual_pose_topic'),
        'leader_status_topic': '/coord/leader_estimate_status',
        'uav_camera_mode': LaunchConfiguration('uav_camera_mode'),
        'camera_mount_pitch_deg': LaunchConfiguration('camera_mount_pitch_deg'),
        'default_tilt_deg': LaunchConfiguration('camera_default_tilt_deg'),
        'camera_yaw_offset_deg': LaunchConfiguration('camera_yaw_offset_deg'),
        'camera_pan_sign': LaunchConfiguration('camera_pan_sign'),
        'actual_pose_reacquire_enable': _bool_param('camera_actual_pose_reacquire_enable'),
        'publish_debug_topics': _bool_param('publish_camera_debug_topics'),
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
                camera_params,
                LaunchConfiguration('params_file'),
            ],
        )
    ]


def _load_node_params_from_yaml(context, node_name: str) -> dict:
    params_file = LaunchConfiguration('params_file').perform(context).strip()
    if not params_file:
        return {}
    with open(params_file, 'r', encoding='utf-8') as fh:
        data = yaml.safe_load(fh) or {}
    merged = {}
    merged.update(((data.get('/**') or {}).get('ros__parameters')) or {})
    merged.update(((data.get(node_name) or {}).get('ros__parameters')) or {})
    return merged


def _launch_bool(context, name: str) -> bool:
    raw = LaunchConfiguration(name).perform(context).strip().lower()
    return raw in ('1', 'true', 'yes', 'on')


def _build_ugv_nav2_node(context, *args, **kwargs):
    nav2_params = _load_node_params_from_yaml(context, 'ugv_nav2_driver')
    nav2_params.update(
        {
            'use_sim_time': True,
            'start_delay_s': float(LaunchConfiguration('ugv_start_delay_s').perform(context)),
            'set_initial_pose_enable': _launch_bool(context, 'ugv_set_initial_pose'),
            'initial_pose_x': float(LaunchConfiguration('ugv_initial_pose_x').perform(context)),
            'initial_pose_y': float(LaunchConfiguration('ugv_initial_pose_y').perform(context)),
            'initial_pose_yaw_deg': float(LaunchConfiguration('ugv_initial_pose_yaw_deg').perform(context)),
            'goal_sequence_csv': LaunchConfiguration('ugv_goal_sequence_csv').perform(context),
            'goal_sequence_file': LaunchConfiguration('ugv_goal_sequence_file').perform(context),
        }
    )
    return [
        Node(
            package='lrs_halmstad',
            executable='ugv_nav2_driver',
            name='ugv_nav2_driver',
            namespace=LaunchConfiguration('ugv_namespace').perform(context),
            output='screen',
            condition=_nav2_ugv_condition(),
            parameters=[nav2_params],
        )
    ]


def _build_omnet_nodes(context, *args, **kwargs):
    if not _launch_bool(context, 'start_omnet_bridge'):
        return []
    uav_name = LaunchConfiguration('uav_name').perform(context)
    ugv_ns = LaunchConfiguration('ugv_namespace').perform(context)
    port = int(LaunchConfiguration('omnet_bridge_port').perform(context))
    return [
        # Converts /dji0/pose (actual Gazebo pose) â†’ Odometry for the pose bridge.
        # Uses the true simulator position rather than the commanded pose so OMNeT
        # node positions stay accurate even when publish_pose_cmd_topics:=false.
        Node(
            package='lrs_halmstad',
            executable='pose_cmd_to_odom',
            name='omnet_uav_pose_to_odom',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'pose_topic': f'/{uav_name}/pose',
                'odom_topic': f'/{uav_name}/pose/odom',
                'frame_id': 'map',
                'child_frame_id': 'base_link',
                'copy_header_stamp': True,
            }],
        ),
        # TCP server (port 5555): serves Gazebo UGV+UAV poses to OMNeT GazeboPositionScheduler.
        Node(
            package='lrs_halmstad',
            executable='gazebo_pose_tcp_bridge',
            name='omnet_tcp_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'port': port,
                'odom_topics': [f'/{ugv_ns}/amcl_pose_odom', f'/{uav_name}/pose/odom'],
                'model_names': ['robot', uav_name],
                'auto_discover_pose_cmd_odom': False,
            }],
        ),
        # TCP client (port 5556): receives live network metrics from OMNeT OmnetMetricsServer
        # and republishes as /omnet/* ROS2 topics.
        Node(
            package='lrs_halmstad',
            executable='omnet_metrics_bridge',
            name='omnet_metrics_bridge',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'omnet_host': '127.0.0.1',
                'omnet_port': 5556,
            }],
        ),
    ]


def _build_visual_actuation_bridge_node(context, *args, **kwargs):
    if not _launch_bool(context, 'start_visual_actuation_bridge'):
        return []
    input_mode = 'auto'
    if _launch_bool(context, 'start_visual_follow_planner'):
        input_mode = 'planned_target'
    elif _launch_bool(context, 'start_visual_follow_point_generator'):
        input_mode = 'follow_point'
    elif _launch_bool(context, 'start_visual_follow_controller'):
        input_mode = 'control'
    return [
        Node(
            package='lrs_halmstad',
            executable='visual_actuation_bridge',
            name='visual_actuation_bridge',
            output='screen',
            parameters=[
                {
                    'use_sim_time': True,
                    'uav_name': LaunchConfiguration('uav_name'),
                    'input_mode': ParameterValue(input_mode, value_type=str),
                    'visual_control_topic': LaunchConfiguration('leader_visual_control_topic'),
                    'follow_point_topic': LaunchConfiguration('leader_follow_point_topic'),
                    'planned_target_topic': LaunchConfiguration('leader_planned_target_topic'),
                    'uav_pose_topic': LaunchConfiguration('leader_uav_pose_topic'),
                    'status_topic': LaunchConfiguration('leader_visual_actuation_bridge_status_topic'),
                },
                LaunchConfiguration('params_file'),
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
        description='Parameter YAML for simulator, leader_detector, leader_tracker, camera_tracker, leader_estimator, follow_uav, and ugv_nav2_driver',
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
    follow_yaw_arg = DeclareLaunchArgument('follow_yaw', default_value='true')
    publish_follow_debug_topics_arg = DeclareLaunchArgument('publish_follow_debug_topics', default_value='true')
    publish_pose_cmd_topics_arg = DeclareLaunchArgument('publish_pose_cmd_topics', default_value='true')
    uav_start_x_arg = DeclareLaunchArgument(
        'uav_start_x',
        default_value='-7.0',
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
    publish_camera_debug_topics_arg = DeclareLaunchArgument('publish_camera_debug_topics', default_value='true')
    camera_yaw_offset_deg_arg = DeclareLaunchArgument('camera_yaw_offset_deg', default_value='0.0')
    camera_pan_sign_arg = DeclareLaunchArgument('camera_pan_sign', default_value='1.0')
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
    ugv_use_amcl_odom_fallback_arg = DeclareLaunchArgument(
        'ugv_use_amcl_odom_fallback',
        default_value='true',
        description='Publish fallback platform odom topics and odom->base_link TF from AMCL when sim odom is unavailable',
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
    camera_leader_actual_pose_topic_arg = DeclareLaunchArgument(
        'camera_leader_actual_pose_topic',
        default_value=['/', LaunchConfiguration('ugv_namespace'), '/platform/odom/filtered'],
        description='Odom-frame leader pose used by camera_tracker for camera-only reacquisition.',
    )
    leader_actual_pose_enable_arg = DeclareLaunchArgument(
        'leader_actual_pose_enable',
        default_value='true',
    )
    camera_actual_pose_reacquire_enable_arg = DeclareLaunchArgument(
        'camera_actual_pose_reacquire_enable',
        default_value='true',
        description='Allow camera_tracker to use leader_actual_pose_topic for camera-only reacquisition when estimator pose is unavailable.',
    )
    leader_actual_heading_enable_arg = DeclareLaunchArgument(
        'leader_actual_heading_enable',
        default_value='false',
    )
    leader_actual_heading_topic_arg = DeclareLaunchArgument(
        'leader_actual_heading_topic',
        default_value=LaunchConfiguration('leader_actual_pose_topic'),
    )
    external_detection_enable_arg = DeclareLaunchArgument(
        'external_detection_enable',
        default_value='false',
        description='Start the external leader_detector node that publishes /coord/leader_detection',
    )
    external_detection_node_arg = DeclareLaunchArgument(
        'external_detection_node',
        default_value='detector',
        description="Perception node to run when external_detection_enable:=true: detector|tracker",
    )
    external_detection_topic_arg = DeclareLaunchArgument(
        'external_detection_topic',
        default_value='/coord/leader_detection',
    )
    leader_image_topic_arg = DeclareLaunchArgument(
        'leader_image_topic',
        default_value=['/', LaunchConfiguration('uav_name'), '/camera0/image_raw'],
    )
    leader_camera_info_topic_arg = DeclareLaunchArgument(
        'leader_camera_info_topic',
        default_value=['/', LaunchConfiguration('uav_name'), '/camera0/camera_info'],
    )
    leader_depth_topic_arg = DeclareLaunchArgument(
        'leader_depth_topic',
        default_value=['/', LaunchConfiguration('uav_name'), '/camera0/depth_image'],
    )
    leader_uav_pose_topic_arg = DeclareLaunchArgument(
        'leader_uav_pose_topic',
        default_value=['/', LaunchConfiguration('uav_name'), '/pose'],
    )
    leader_range_mode_arg = DeclareLaunchArgument('leader_range_mode', default_value='auto')
    target_class_name_arg = DeclareLaunchArgument('target_class_name', default_value='')
    target_class_id_arg = DeclareLaunchArgument('target_class_id', default_value='-1')
    yolo_weights_arg = DeclareLaunchArgument(
        'yolo_weights',
        default_value='',
    )
    yolo_device_arg = DeclareLaunchArgument('yolo_device', default_value='auto')
    detector_backend_arg = DeclareLaunchArgument('detector_backend', default_value='ultralytics')
    detector_onnx_model_arg = DeclareLaunchArgument('detector_onnx_model', default_value='')
    detector_async_inference_arg = DeclareLaunchArgument('detector_async_inference', default_value='true')
    detector_latest_frame_only_arg = DeclareLaunchArgument('detector_latest_frame_only', default_value='true')
    detector_stale_detection_threshold_ms_arg = DeclareLaunchArgument(
        'detector_stale_detection_threshold_ms',
        default_value='500.0',
    )
    detector_metrics_window_s_arg = DeclareLaunchArgument('detector_metrics_window_s', default_value='5.0')
    detector_benchmark_csv_path_arg = DeclareLaunchArgument('detector_benchmark_csv_path', default_value='')
    detector_image_qos_depth_arg = DeclareLaunchArgument('detector_image_qos_depth', default_value='1')
    detector_image_qos_reliability_arg = DeclareLaunchArgument(
        'detector_image_qos_reliability',
        default_value='best_effort',
    )
    tracker_config_arg = DeclareLaunchArgument('tracker_config', default_value='bytetrack.yaml')
    event_topic_arg = DeclareLaunchArgument('event_topic', default_value='/coord/events')
    ugv_start_delay_arg = DeclareLaunchArgument('ugv_start_delay_s', default_value='0.0')
    start_omnet_bridge_arg = DeclareLaunchArgument(
        'start_omnet_bridge',
        default_value='false',
        description='Start the Gazeboâ†’OMNeT TCP pose bridge on omnet_bridge_port',
    )
    omnet_bridge_port_arg = DeclareLaunchArgument(
        'omnet_bridge_port',
        default_value='5555',
        description='TCP port for the OMNeT pose bridge (must match omnetpp.ini gazeboScheduler.port)',
    )

    # Visual follow pipeline args
    start_visual_follow_controller_arg = DeclareLaunchArgument(
        'start_visual_follow_controller',
        default_value='false',
        description='Start the optional image-space visual follow test controller',
    )
    start_visual_actuation_bridge_arg = DeclareLaunchArgument(
        'start_visual_actuation_bridge',
        default_value='false',
        description='Start bridge that converts visual-follow commands into the UAV actuation path; disables follow_uav/follow_uav_odom',
    )
    start_visual_follow_point_generator_arg = DeclareLaunchArgument(
        'start_visual_follow_point_generator',
        default_value='false',
        description='Start the follow-point generator that turns the visual target estimate into a spatial follow goal',
    )
    start_visual_follow_planner_arg = DeclareLaunchArgument(
        'start_visual_follow_planner',
        default_value='false',
        description='Start the planner that smooths the follow point into a planned pose target for the bridge',
    )
    leader_selected_target_topic_arg = DeclareLaunchArgument(
        'leader_selected_target_topic',
        default_value='/coord/leader_selected_target',
    )
    leader_selected_target_filtered_topic_arg = DeclareLaunchArgument(
        'leader_selected_target_filtered_topic',
        default_value='/coord/leader_selected_target_filtered',
    )
    leader_selected_target_filtered_status_topic_arg = DeclareLaunchArgument(
        'leader_selected_target_filtered_status_topic',
        default_value='/coord/leader_selected_target_filtered_status',
    )
    leader_visual_target_estimate_topic_arg = DeclareLaunchArgument(
        'leader_visual_target_estimate_topic',
        default_value='/coord/leader_visual_target_estimate',
    )
    leader_visual_target_estimate_status_topic_arg = DeclareLaunchArgument(
        'leader_visual_target_estimate_status_topic',
        default_value='/coord/leader_visual_target_estimate_status',
    )
    leader_follow_point_topic_arg = DeclareLaunchArgument(
        'leader_follow_point_topic',
        default_value='/coord/leader_follow_point',
    )
    leader_follow_point_status_topic_arg = DeclareLaunchArgument(
        'leader_follow_point_status_topic',
        default_value='/coord/leader_follow_point_status',
    )
    leader_planned_target_topic_arg = DeclareLaunchArgument(
        'leader_planned_target_topic',
        default_value='/coord/leader_planned_target',
    )
    leader_planned_target_status_topic_arg = DeclareLaunchArgument(
        'leader_planned_target_status_topic',
        default_value='/coord/leader_planned_target_status',
    )
    leader_visual_control_topic_arg = DeclareLaunchArgument(
        'leader_visual_control_topic',
        default_value='/coord/leader_visual_control',
    )
    leader_visual_control_status_topic_arg = DeclareLaunchArgument(
        'leader_visual_control_status_topic',
        default_value='/coord/leader_visual_control_status',
    )
    leader_visual_actuation_bridge_status_topic_arg = DeclareLaunchArgument(
        'leader_visual_actuation_bridge_status_topic',
        default_value='/coord/leader_visual_actuation_bridge_status',
    )
    leader_camera_actual_pose_topic_arg = DeclareLaunchArgument(
        'leader_camera_actual_pose_topic',
        default_value=['/', LaunchConfiguration('uav_name'), '/camera/actual/center_pose'],
    )

    simulator_node = Node(
        package='lrs_halmstad',
        executable='simulator',
        name='uav_simulator',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_uav_simulator')),
        parameters=[
            {
                'use_sim_time': True,
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
            LaunchConfiguration('params_file'),
        ],
    )

    detector_runtime_params = RewrittenYaml(
        source_file=LaunchConfiguration('params_file'),
        param_rewrites={
            'backend': LaunchConfiguration('detector_backend'),
            'onnx_model': LaunchConfiguration('detector_onnx_model'),
            'async_inference': LaunchConfiguration('detector_async_inference'),
            'latest_frame_only': LaunchConfiguration('detector_latest_frame_only'),
            'stale_detection_threshold_ms': LaunchConfiguration('detector_stale_detection_threshold_ms'),
            'metrics_window_s': LaunchConfiguration('detector_metrics_window_s'),
            'benchmark_csv_path': LaunchConfiguration('detector_benchmark_csv_path'),
            'image_qos_depth': LaunchConfiguration('detector_image_qos_depth'),
            'image_qos_reliability': LaunchConfiguration('detector_image_qos_reliability'),
            'tracker_config': LaunchConfiguration('tracker_config'),
        },
        convert_types=True,
    )

    detector_node = Node(
        package='lrs_halmstad',
        executable='leader_detector',
        name='leader_detector',
        output='screen',
        condition=_external_perception_condition('detector'),
        parameters=[
            detector_runtime_params,
            {
                'use_sim_time': True,
                'uav_name': LaunchConfiguration('uav_name'),
                'camera_topic': LaunchConfiguration('leader_image_topic'),
                'out_topic': LaunchConfiguration('external_detection_topic'),
                'target_class_name': LaunchConfiguration('target_class_name'),
                'target_class_id': LaunchConfiguration('target_class_id'),
                'device': LaunchConfiguration('yolo_device'),
                'yolo_weights': LaunchConfiguration('yolo_weights'),
                'backend': LaunchConfiguration('detector_backend'),
                'onnx_model': LaunchConfiguration('detector_onnx_model'),
                'async_inference': _bool_param('detector_async_inference'),
                'latest_frame_only': _bool_param('detector_latest_frame_only'),
                'stale_detection_threshold_ms': LaunchConfiguration('detector_stale_detection_threshold_ms'),
                'metrics_window_s': LaunchConfiguration('detector_metrics_window_s'),
                'benchmark_csv_path': LaunchConfiguration('detector_benchmark_csv_path'),
                'image_qos_depth': LaunchConfiguration('detector_image_qos_depth'),
                'image_qos_reliability': LaunchConfiguration('detector_image_qos_reliability'),
                'event_topic': LaunchConfiguration('event_topic'),
            },
        ],
    )

    tracker_node = Node(
        package='lrs_halmstad',
        executable='leader_tracker',
        name='leader_tracker',
        output='screen',
        condition=_external_perception_condition('tracker'),
        parameters=[
            detector_runtime_params,
            {
                'use_sim_time': True,
                'uav_name': LaunchConfiguration('uav_name'),
                'camera_topic': LaunchConfiguration('leader_image_topic'),
                'out_topic': LaunchConfiguration('external_detection_topic'),
                'target_class_name': LaunchConfiguration('target_class_name'),
                'target_class_id': LaunchConfiguration('target_class_id'),
                'device': LaunchConfiguration('yolo_device'),
                'yolo_weights': LaunchConfiguration('yolo_weights'),
                'backend': LaunchConfiguration('detector_backend'),
                'onnx_model': LaunchConfiguration('detector_onnx_model'),
                'async_inference': _bool_param('detector_async_inference'),
                'latest_frame_only': _bool_param('detector_latest_frame_only'),
                'stale_detection_threshold_ms': LaunchConfiguration('detector_stale_detection_threshold_ms'),
                'metrics_window_s': LaunchConfiguration('detector_metrics_window_s'),
                'benchmark_csv_path': LaunchConfiguration('detector_benchmark_csv_path'),
                'image_qos_depth': LaunchConfiguration('detector_image_qos_depth'),
                'image_qos_reliability': LaunchConfiguration('detector_image_qos_reliability'),
                'tracker_config': LaunchConfiguration('tracker_config'),
                'event_topic': LaunchConfiguration('event_topic'),
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
            {
                'use_sim_time': True,
                'uav_name': LaunchConfiguration('uav_name'),
                'camera_topic': LaunchConfiguration('leader_image_topic'),
                'camera_info_topic': LaunchConfiguration('leader_camera_info_topic'),
                'depth_topic': LaunchConfiguration('leader_depth_topic'),
                'uav_pose_topic': LaunchConfiguration('leader_uav_pose_topic'),
                'range_mode': LaunchConfiguration('leader_range_mode'),
                'leader_actual_pose_topic': LaunchConfiguration('leader_actual_pose_topic'),
                'leader_actual_pose_enable': _bool_param('leader_actual_pose_enable'),
                'external_detection_topic': LaunchConfiguration('external_detection_topic'),
                'external_detection_max_latency_ms': LaunchConfiguration('detector_stale_detection_threshold_ms'),
                'event_topic': LaunchConfiguration('event_topic'),
            },
            LaunchConfiguration('params_file'),
        ],
    )

    follow_odom_node = Node(
        package='lrs_halmstad',
        executable='follow_uav_odom',
        name='follow_uav',
        output='screen',
        condition=_leader_odom_condition(),
        parameters=[
            {
                'use_sim_time': True,
                'world': LaunchConfiguration('world'),
                'uav_name': LaunchConfiguration('uav_name'),
                'leader_odom_topic': LaunchConfiguration('ugv_odom_topic'),
                'uav_start_z': LaunchConfiguration('uav_start_z'),
                'event_topic': LaunchConfiguration('event_topic'),
                'follow_yaw': _bool_param('follow_yaw'),
                'publish_debug_topics': _bool_param('publish_follow_debug_topics'),
                'publish_pose_cmd_topics': _bool_param('publish_pose_cmd_topics'),
            },
            LaunchConfiguration('params_file'),
        ],
    )

    follow_estimate_node = Node(
        package='lrs_halmstad',
        executable='follow_uav',
        name='follow_uav',
        output='screen',
        condition=_leader_nonodom_condition(),
        parameters=[
            {
                'use_sim_time': True,
                'world': LaunchConfiguration('world'),
                'uav_name': LaunchConfiguration('uav_name'),
                'leader_input_type': LaunchConfiguration('leader_mode'),
                'leader_pose_topic': LaunchConfiguration('leader_pose_topic'),
                'leader_actual_heading_enable': _bool_param('leader_actual_heading_enable'),
                'leader_actual_heading_topic': LaunchConfiguration('leader_actual_heading_topic'),
                'uav_start_z': LaunchConfiguration('uav_start_z'),
                'follow_yaw': _bool_param('follow_yaw'),
                'publish_debug_topics': _bool_param('publish_follow_debug_topics'),
                'publish_pose_cmd_topics': _bool_param('publish_pose_cmd_topics'),
            },
            LaunchConfiguration('params_file'),
        ],
    )

    camera_tracker_node = OpaqueFunction(function=_build_camera_tracker_node)

    selected_target_filter_node = Node(
        package='lrs_halmstad',
        executable='selected_target_filter',
        name='selected_target_filter',
        output='screen',
        condition=_visual_pipeline_condition(),
        parameters=[
            {
                'use_sim_time': True,
                'in_topic': LaunchConfiguration('leader_selected_target_topic'),
                'out_topic': LaunchConfiguration('leader_selected_target_filtered_topic'),
                'status_topic': LaunchConfiguration('leader_selected_target_filtered_status_topic'),
            },
            LaunchConfiguration('params_file'),
        ],
    )

    visual_target_estimator_node = Node(
        package='lrs_halmstad',
        executable='visual_target_estimator',
        name='visual_target_estimator',
        output='screen',
        condition=_visual_pipeline_condition(),
        parameters=[
            {
                'use_sim_time': True,
                'uav_name': LaunchConfiguration('uav_name'),
                'selected_target_topic': LaunchConfiguration('leader_selected_target_filtered_topic'),
                'camera_info_topic': LaunchConfiguration('leader_camera_info_topic'),
                'out_topic': LaunchConfiguration('leader_visual_target_estimate_topic'),
                'status_topic': LaunchConfiguration('leader_visual_target_estimate_status_topic'),
            },
            LaunchConfiguration('params_file'),
        ],
    )

    follow_point_generator_node = Node(
        package='lrs_halmstad',
        executable='follow_point_generator',
        name='follow_point_generator',
        output='screen',
        condition=_follow_point_generator_condition(),
        parameters=[
            {
                'use_sim_time': True,
                'uav_name': LaunchConfiguration('uav_name'),
                'target_estimate_topic': LaunchConfiguration('leader_visual_target_estimate_topic'),
                'uav_pose_topic': LaunchConfiguration('leader_uav_pose_topic'),
                'camera_pose_topic': LaunchConfiguration('leader_camera_actual_pose_topic'),
                'out_topic': LaunchConfiguration('leader_follow_point_topic'),
                'status_topic': LaunchConfiguration('leader_follow_point_status_topic'),
            },
            LaunchConfiguration('params_file'),
        ],
    )

    follow_point_planner_node = Node(
        package='lrs_halmstad',
        executable='follow_point_planner',
        name='follow_point_planner',
        output='screen',
        condition=_follow_planner_condition(),
        parameters=[
            {
                'use_sim_time': True,
                'uav_name': LaunchConfiguration('uav_name'),
                'follow_point_topic': LaunchConfiguration('leader_follow_point_topic'),
                'uav_pose_topic': LaunchConfiguration('leader_uav_pose_topic'),
                'out_topic': LaunchConfiguration('leader_planned_target_topic'),
                'status_topic': LaunchConfiguration('leader_planned_target_status_topic'),
            },
            LaunchConfiguration('params_file'),
        ],
    )

    visual_follow_controller_node = Node(
        package='lrs_halmstad',
        executable='visual_follow_controller',
        name='visual_follow_controller',
        output='screen',
        condition=_visual_controller_condition(),
        parameters=[
            {
                'use_sim_time': True,
                'uav_name': LaunchConfiguration('uav_name'),
                'camera_topic': LaunchConfiguration('leader_image_topic'),
                'camera_info_topic': LaunchConfiguration('leader_camera_info_topic'),
                'selected_target_topic': LaunchConfiguration('leader_selected_target_filtered_topic'),
                'target_estimate_topic': LaunchConfiguration('leader_visual_target_estimate_topic'),
                'out_topic': LaunchConfiguration('leader_visual_control_topic'),
                'status_topic': LaunchConfiguration('leader_visual_control_status_topic'),
            },
            LaunchConfiguration('params_file'),
        ],
    )

    visual_actuation_bridge_node = OpaqueFunction(function=_build_visual_actuation_bridge_node)

    ugv_nav2_node = OpaqueFunction(function=_build_ugv_nav2_node)

    ugv_amcl_to_odom_node = Node(
        package='lrs_halmstad',
        executable='pose_cov_to_odom',
        name='ugv_amcl_to_odom',
        namespace=LaunchConfiguration('ugv_namespace'),
        output='screen',
        parameters=[
            {
                'use_sim_time': True,
                'pose_topic': 'amcl_pose',
                'odom_topic': 'amcl_pose_odom',
                'frame_id': 'map',
                'child_frame_id': 'base_link',
                'copy_header_stamp': True,
            },
        ],
    )

    ugv_amcl_to_platform_odom_node = Node(
        package='lrs_halmstad',
        executable='pose_cov_to_odom',
        name='ugv_amcl_to_platform_odom',
        namespace=LaunchConfiguration('ugv_namespace'),
        output='screen',
        condition=IfCondition(LaunchConfiguration('ugv_use_amcl_odom_fallback')),
        parameters=[
            {
                'pose_topic': 'amcl_pose',
                'odom_topic': 'platform/odom',
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
                'copy_header_stamp': True,
            },
        ],
    )

    ugv_amcl_to_platform_filtered_odom_node = Node(
        package='lrs_halmstad',
        executable='pose_cov_to_odom',
        name='ugv_amcl_to_platform_filtered_odom',
        namespace=LaunchConfiguration('ugv_namespace'),
        output='screen',
        condition=IfCondition(LaunchConfiguration('ugv_use_amcl_odom_fallback')),
        parameters=[
            {
                'pose_topic': 'amcl_pose',
                'odom_topic': 'platform/odom/filtered',
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
                'copy_header_stamp': True,
            },
        ],
    )

    ugv_platform_odom_to_tf_node = Node(
        package='lrs_halmstad',
        executable='odom_to_tf',
        name='ugv_platform_odom_to_tf',
        namespace=LaunchConfiguration('ugv_namespace'),
        output='screen',
        condition=IfCondition(LaunchConfiguration('ugv_use_amcl_odom_fallback')),
        parameters=[
            {
                'odom_topic': 'platform/odom/filtered',
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
                'copy_header_stamp': True,
            },
        ],
    )

    omnet_nodes = OpaqueFunction(function=_build_omnet_nodes)

    ugv_nav2_delayed_start = TimerAction(
        period=0.1,
        actions=[
            LogInfo(
                msg='[run_follow] Starting Nav2-backed UGV motion driver',
                condition=_nav2_ugv_condition(),
            ),
            ugv_nav2_node,
        ],
    )

    ugv_external_info = TimerAction(
        period=0.1,
        actions=[
            LogInfo(
                msg='[run_follow] UGV mobility backend disabled; expecting external Nav2 goal source',
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
        follow_yaw_arg,
        publish_follow_debug_topics_arg,
        publish_pose_cmd_topics_arg,
        uav_start_x_arg,
        uav_start_y_arg,
        uav_start_z_arg,
        uav_start_yaw_deg_arg,
        camera_mount_pitch_deg_arg,
        camera_default_tilt_deg_arg,
        pan_enable_arg,
        tilt_enable_arg,
        publish_camera_debug_topics_arg,
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
        ugv_use_amcl_odom_fallback_arg,
        leader_pose_topic_arg,
        ugv_odom_topic_arg,
        leader_actual_pose_topic_arg,
        camera_leader_actual_pose_topic_arg,
        leader_actual_pose_enable_arg,
        camera_actual_pose_reacquire_enable_arg,
        leader_actual_heading_enable_arg,
        leader_actual_heading_topic_arg,
        external_detection_enable_arg,
        external_detection_node_arg,
        external_detection_topic_arg,
        leader_image_topic_arg,
        leader_camera_info_topic_arg,
        leader_depth_topic_arg,
        leader_uav_pose_topic_arg,
        leader_range_mode_arg,
        target_class_name_arg,
        target_class_id_arg,
        yolo_weights_arg,
        yolo_device_arg,
        detector_backend_arg,
        detector_onnx_model_arg,
        detector_async_inference_arg,
        detector_latest_frame_only_arg,
        detector_stale_detection_threshold_ms_arg,
        detector_metrics_window_s_arg,
        detector_benchmark_csv_path_arg,
        detector_image_qos_depth_arg,
        detector_image_qos_reliability_arg,
        tracker_config_arg,
        event_topic_arg,
        ugv_start_delay_arg,
        start_omnet_bridge_arg,
        omnet_bridge_port_arg,
        start_visual_follow_controller_arg,
        start_visual_actuation_bridge_arg,
        start_visual_follow_point_generator_arg,
        start_visual_follow_planner_arg,
        leader_selected_target_topic_arg,
        leader_selected_target_filtered_topic_arg,
        leader_selected_target_filtered_status_topic_arg,
        leader_visual_target_estimate_topic_arg,
        leader_visual_target_estimate_status_topic_arg,
        leader_follow_point_topic_arg,
        leader_follow_point_status_topic_arg,
        leader_planned_target_topic_arg,
        leader_planned_target_status_topic_arg,
        leader_visual_control_topic_arg,
        leader_visual_control_status_topic_arg,
        leader_visual_actuation_bridge_status_topic_arg,
        leader_camera_actual_pose_topic_arg,
        simulator_node,
        ugv_amcl_to_odom_node,
        ugv_amcl_to_platform_odom_node,
        ugv_amcl_to_platform_filtered_odom_node,
        ugv_platform_odom_to_tf_node,
        detector_node,
        tracker_node,
        estimator_node,
        selected_target_filter_node,
        visual_target_estimator_node,
        follow_point_generator_node,
        follow_point_planner_node,
        visual_follow_controller_node,
        visual_actuation_bridge_node,
        follow_odom_node,
        follow_estimate_node,
        camera_tracker_node,
        omnet_nodes,
        ugv_nav2_delayed_start,
        ugv_external_info,
    ])
