from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
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


def _controller_backend_condition():
    backend = LaunchConfiguration('uav_backend')
    return IfCondition(
        PythonExpression([
            "'", backend, "'.lower() == 'controller'",
        ])
    )


def _simulator_failfast_condition():
    return _controller_backend_condition()


def generate_launch_description():
    params_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'run_round_follow_defaults.yaml']
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_default,
        description='Parameter YAML for leader_estimator, follow_uav, and UGV motion driver',
    )
    world_arg = DeclareLaunchArgument('world', default_value='orchard')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    leader_mode_arg = DeclareLaunchArgument('leader_mode', default_value='odom')
    leader_dependency_mode_arg = DeclareLaunchArgument(
        'leader_dependency_mode',
        default_value='ugv_state',
        description='Leader dependency policy: uav_only (estimate/observation path) or ugv_state (debug compatibility)',
    )
    uav_backend_arg = DeclareLaunchArgument(
        'uav_backend',
        default_value='setpose',
        description="UAV backend mode: setpose (baseline) or controller (simulator command interface)",
    )
    leader_perception_enable_arg = DeclareLaunchArgument('leader_perception_enable', default_value='false')
    start_estimator_arg = DeclareLaunchArgument(
        'start_leader_estimator',
        default_value='auto',
        description="auto|true|false; auto starts estimator for pose/estimate, perception mode, or when yolo_weights is set",
    )
    ugv_cmd_topic_arg = DeclareLaunchArgument('ugv_cmd_topic', default_value='/a201_0000/cmd_vel')
    ugv_odom_topic_arg = DeclareLaunchArgument('ugv_odom_topic', default_value='/a201_0000/platform/odom')
    leader_image_topic_arg = DeclareLaunchArgument('leader_image_topic', default_value=['/', LaunchConfiguration('uav_name'), '/camera0/image_raw'])
    leader_camera_info_topic_arg = DeclareLaunchArgument('leader_camera_info_topic', default_value=['/', LaunchConfiguration('uav_name'), '/camera0/camera_info'])
    leader_camera_tilt_topic_arg = DeclareLaunchArgument('leader_camera_tilt_topic', default_value=['/', LaunchConfiguration('uav_name'), '/update_tilt'])
    leader_depth_topic_arg = DeclareLaunchArgument('leader_depth_topic', default_value='')
    leader_uav_pose_topic_arg = DeclareLaunchArgument('leader_uav_pose_topic', default_value=['/', LaunchConfiguration('uav_name'), '/pose_cmd'])
    backend_frame_id_arg = DeclareLaunchArgument('backend_frame_id', default_value='map')
    backend_initial_x_arg = DeclareLaunchArgument('backend_initial_x', default_value='-2.0')
    backend_initial_y_arg = DeclareLaunchArgument('backend_initial_y', default_value='0.0')
    backend_initial_z_arg = DeclareLaunchArgument('backend_initial_z', default_value='5.0')
    backend_initial_yaw_deg_arg = DeclareLaunchArgument('backend_initial_yaw_deg', default_value='0.0')
    backend_initial_pan_deg_arg = DeclareLaunchArgument('backend_initial_pan_deg', default_value='0.0')
    backend_initial_tilt_deg_arg = DeclareLaunchArgument('backend_initial_tilt_deg', default_value='-45.0')
    yolo_weights_arg = DeclareLaunchArgument('yolo_weights', default_value='')
    yolo_device_arg = DeclareLaunchArgument('yolo_device', default_value='cpu')
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.12',
        description='YOLO confidence threshold for leader_estimator.',
    )
    iou_threshold_arg = DeclareLaunchArgument(
        'iou_threshold',
        default_value='0.45',
        description='YOLO NMS IoU threshold for leader_estimator.',
    )
    target_class_id_arg = DeclareLaunchArgument(
        'target_class_id',
        default_value='-1',
        description='Optional class-id filter for leader_estimator (-1 disables).',
    )
    target_class_name_arg = DeclareLaunchArgument(
        'target_class_name',
        default_value='',
        description='Optional class-name filter for leader_estimator (empty disables).',
    )
    target_class_names_arg = DeclareLaunchArgument(
        'target_class_names',
        default_value='',
        description='Optional comma-separated class-name allowlist for leader_estimator.',
    )
    target_mode_arg = DeclareLaunchArgument(
        'target_mode',
        default_value='standard',
        description='Target selector mode: standard|proxy_coco',
    )
    target_coco_class_id_arg = DeclareLaunchArgument(
        'target_coco_class_id',
        default_value='-1',
        description='Proxy COCO class-id filter in proxy_coco mode (-1 disables).',
    )
    target_coco_class_name_arg = DeclareLaunchArgument(
        'target_coco_class_name',
        default_value='cell phone',
        description='Proxy COCO class-name filter in proxy_coco mode.',
    )
    target_coco_class_names_arg = DeclareLaunchArgument(
        'target_coco_class_names',
        default_value='',
        description='Proxy COCO class-name allowlist in proxy_coco mode (comma-separated).',
    )
    proxy_center_weight_arg = DeclareLaunchArgument(
        'proxy_center_weight',
        default_value='0.25',
        description='Proxy mode center-bias score weight.',
    )
    proxy_center_max_px_arg = DeclareLaunchArgument(
        'proxy_center_max_px',
        default_value='900.0',
        description='Proxy mode center-bias normalization distance in pixels.',
    )
    proxy_switch_max_px_arg = DeclareLaunchArgument(
        'proxy_switch_max_px',
        default_value='300.0',
        description='Proxy mode target-switch guard distance in pixels.',
    )
    proxy_switch_min_conf_arg = DeclareLaunchArgument(
        'proxy_switch_min_conf',
        default_value='0.35',
        description='Proxy mode minimum confidence required for far target switch.',
    )
    proxy_switch_force_local_arg = DeclareLaunchArgument(
        'proxy_switch_force_local',
        default_value='false',
        description='Proxy mode: if true, reject far target switches regardless of confidence.',
    )
    proxy_use_geom_filters_arg = DeclareLaunchArgument(
        'proxy_use_geom_filters',
        default_value='false',
        description='Proxy mode: if true, apply standard geometry filters (off recommended for proxy retention).',
    )
    track_only_allow_reacquire_motion_arg = DeclareLaunchArgument(
        'track_only_allow_reacquire_motion',
        default_value='true',
        description='If true, allow UAV XY motion in REACQUIRE; hold only in HOLD/DEGRADED.',
    )
    camera_pose_control_mode_arg = DeclareLaunchArgument(
        'camera_pose_control_mode',
        default_value='auto',
        description='Camera control path: auto|separate_model|gimbal_only.',
    )
    camera_pose_failover_failures_arg = DeclareLaunchArgument(
        'camera_pose_failover_failures',
        default_value='12',
        description='In auto mode, switch to gimbal_only after this many consecutive camera set_pose failures.',
    )
    camera_lock_enable_arg = DeclareLaunchArgument(
        'camera_lock_enable',
        default_value='true',
        description='Enable camera lock/search pan controller in follow_uav.',
    )
    camera_lock_tilt_enable_arg = DeclareLaunchArgument(
        'camera_lock_tilt_enable',
        default_value='true',
        description='Enable camera tilt lock controller in follow_uav.',
    )
    camera_lock_tilt_gain_per_px_arg = DeclareLaunchArgument(
        'camera_lock_tilt_gain_per_px',
        default_value='0.02',
        description='Tilt lock proportional gain in deg/pixel for vertical image error.',
    )
    camera_lock_tilt_deadband_px_arg = DeclareLaunchArgument(
        'camera_lock_tilt_deadband_px',
        default_value='10.0',
        description='Tilt lock deadband in vertical pixels.',
    )
    camera_lock_tilt_min_deg_arg = DeclareLaunchArgument(
        'camera_lock_tilt_min_deg',
        default_value='-88.0',
        description='Minimum tilt command in degrees (more negative looks farther down).',
    )
    camera_lock_tilt_max_deg_arg = DeclareLaunchArgument(
        'camera_lock_tilt_max_deg',
        default_value='-20.0',
        description='Maximum tilt command in degrees.',
    )
    camera_lock_tilt_max_step_deg_arg = DeclareLaunchArgument(
        'camera_lock_tilt_max_step_deg',
        default_value='2.0',
        description='Maximum per-tick tilt command change in degrees.',
    )
    under_target_guard_enable_arg = DeclareLaunchArgument(
        'under_target_guard_enable',
        default_value='true',
        description='Enable near-under-target guard to prevent loss when target is too low/close in frame.',
    )
    under_target_err_v_ratio_arg = DeclareLaunchArgument(
        'under_target_err_v_ratio',
        default_value='0.33',
        description='Under-target trigger threshold as fraction of image height for err_v_px.',
    )
    under_target_range_m_arg = DeclareLaunchArgument(
        'under_target_range_m',
        default_value='2.2',
        description='Under-target trigger threshold for estimated range_m.',
    )
    under_target_hold_xy_arg = DeclareLaunchArgument(
        'under_target_hold_xy',
        default_value='true',
        description='In under-target guard, hold XY command.',
    )
    under_target_hold_yaw_arg = DeclareLaunchArgument(
        'under_target_hold_yaw',
        default_value='true',
        description='In under-target guard, hold yaw command.',
    )
    reacquire_sequence_enable_arg = DeclareLaunchArgument(
        'reacquire_sequence_enable',
        default_value='true',
        description='Enable structured reacquire sequence PAN->TILT->YAW->HOLD.',
    )
    reacquire_pan_phase_s_arg = DeclareLaunchArgument(
        'reacquire_pan_phase_s',
        default_value='2.0',
        description='Duration of local pan-search phase (s).',
    )
    reacquire_tilt_phase_s_arg = DeclareLaunchArgument(
        'reacquire_tilt_phase_s',
        default_value='2.5',
        description='Duration of vertical tilt-sweep phase (s).',
    )
    reacquire_yaw_phase_s_arg = DeclareLaunchArgument(
        'reacquire_yaw_phase_s',
        default_value='8.0',
        description='Duration of wide yaw-search phase (s).',
    )
    reacquire_pan_local_span_deg_arg = DeclareLaunchArgument(
        'reacquire_pan_local_span_deg',
        default_value='28.0',
        description='Half-span of local pan search around loss bearing (deg).',
    )
    reacquire_tilt_local_span_deg_arg = DeclareLaunchArgument(
        'reacquire_tilt_local_span_deg',
        default_value='12.0',
        description='Half-span of local tilt search around loss tilt (deg).',
    )
    reacquire_pan_rate_deg_s_arg = DeclareLaunchArgument(
        'reacquire_pan_rate_deg_s',
        default_value='24.0',
        description='Pan search speed (deg/s).',
    )
    reacquire_tilt_rate_deg_s_arg = DeclareLaunchArgument(
        'reacquire_tilt_rate_deg_s',
        default_value='22.0',
        description='Tilt sweep speed (deg/s).',
    )
    reacquire_yaw_rate_deg_s_arg = DeclareLaunchArgument(
        'reacquire_yaw_rate_deg_s',
        default_value='45.0',
        description='Yaw search speed during wide sweep phase (deg/s).',
    )
    reacquire_hold_phase_s_arg = DeclareLaunchArgument(
        'reacquire_hold_phase_s',
        default_value='1.5',
        description='Short HOLD dwell at end of search cycle before restarting PAN (s).',
    )
    reacquire_fallback_hold_s_arg = DeclareLaunchArgument(
        'reacquire_fallback_hold_s',
        default_value='30.0',
        description='Total unsuccessful search time before terminal fallback HOLD (s, 0=disabled).',
    )
    reacquire_hold_after_search_arg = DeclareLaunchArgument(
        'reacquire_hold_after_search',
        default_value='true',
        description='Enable HOLD stage after PAN/TILT/YAW (dwell + optional long fallback hold).',
    )
    reacquire_search_hold_xy_arg = DeclareLaunchArgument(
        'reacquire_search_hold_xy',
        default_value='true',
        description='Hold XY during structured search phases.',
    )
    reacquire_search_hold_yaw_before_yaw_phase_arg = DeclareLaunchArgument(
        'reacquire_search_hold_yaw_before_yaw_phase',
        default_value='true',
        description='Hold yaw in PAN/TILT phases before YAW search starts.',
    )
    reacquire_commit_enable_arg = DeclareLaunchArgument(
        'reacquire_commit_enable',
        default_value='true',
        description='Enable short commit window after visibility hit before restarting search.',
    )
    reacquire_commit_s_arg = DeclareLaunchArgument(
        'reacquire_commit_s',
        default_value='1.5',
        description='Reacquire commit duration after sel_visible hit (s).',
    )
    reacquire_commit_quality_floor_arg = DeclareLaunchArgument(
        'reacquire_commit_quality_floor',
        default_value='0.22',
        description='Minimum quality scale during reacquire commit grace.',
    )
    reacquire_startup_guard_enable_arg = DeclareLaunchArgument(
        'reacquire_startup_guard_enable',
        default_value='true',
        description='Prevent structured search before first estimator status arrives.',
    )
    reacquire_first_lock_gate_enable_arg = DeclareLaunchArgument(
        'reacquire_first_lock_gate_enable',
        default_value='true',
        description='Block structured reacquire search until first stable sel_visible lock is observed.',
    )
    reacquire_first_lock_min_visible_frames_arg = DeclareLaunchArgument(
        'reacquire_first_lock_min_visible_frames',
        default_value='3',
        description='Consecutive sel_visible=1 status frames required to arm first lock gate.',
    )
    reacquire_relock_min_visible_frames_arg = DeclareLaunchArgument(
        'reacquire_relock_min_visible_frames',
        default_value='3',
        description='Consecutive sel_visible=1 frames required before returning to TRACK after loss.',
    )
    reacquire_yaw_assist_delay_s_arg = DeclareLaunchArgument(
        'reacquire_yaw_assist_delay_s',
        default_value='4.0',
        description='Loss persistence time before yaw assist can activate in structured reacquire.',
    )
    reacquire_xy_assist_delay_s_arg = DeclareLaunchArgument(
        'reacquire_xy_assist_delay_s',
        default_value='7.0',
        description='Loss persistence time before XY motion assist can activate in structured reacquire.',
    )
    follow_yaw_arg = DeclareLaunchArgument(
        'follow_yaw',
        default_value='true',
        description='Enable follow_uav yaw tracking to leader heading.',
    )
    xy_motion_enable_arg = DeclareLaunchArgument(
        'xy_motion_enable',
        default_value='true',
        description='Enable UAV XY translation command updates (set false for camera-lock isolation).',
    )
    publish_follow_debug_status_arg = DeclareLaunchArgument(
        'publish_follow_debug_status',
        default_value='true',
        description='Publish structured follow debug status topic.',
    )
    follow_debug_status_topic_arg = DeclareLaunchArgument(
        'follow_debug_status_topic',
        default_value='/coord/follow_debug_status',
        description='Follow debug status topic name.',
    )
    event_topic_arg = DeclareLaunchArgument('event_topic', default_value='/coord/events')
    # Fallback delay only. The primary startup gating now happens inside
    # ugv_motion_driver via a lightweight readiness check (cmd subscriber + odom flow).
    ugv_start_delay_arg = DeclareLaunchArgument('ugv_start_delay_s', default_value='0.0')
    ugv_ready_require_leader_flow_arg = DeclareLaunchArgument('ugv_ready_require_leader_flow', default_value='false')
    ugv_ready_leader_topic_arg = DeclareLaunchArgument('ugv_ready_leader_topic', default_value='/coord/leader_estimate')
    ugv_ready_timeout_arg = DeclareLaunchArgument('ugv_ready_timeout_s', default_value='8.0')
    shutdown_when_ugv_done_arg = DeclareLaunchArgument(
        'shutdown_when_ugv_done',
        default_value='false',
        description='If true, stop this launch when ugv_motion_driver exits.',
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
                'camera_tilt_topic': LaunchConfiguration('leader_camera_tilt_topic'),
                'depth_topic': LaunchConfiguration('leader_depth_topic'),
                'uav_pose_topic': LaunchConfiguration('leader_uav_pose_topic'),
                'device': LaunchConfiguration('yolo_device'),
                'yolo_weights': LaunchConfiguration('yolo_weights'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'iou_threshold': LaunchConfiguration('iou_threshold'),
                'target_class_id': LaunchConfiguration('target_class_id'),
                'target_class_name': LaunchConfiguration('target_class_name'),
                'target_class_names': LaunchConfiguration('target_class_names'),
                'target_mode': LaunchConfiguration('target_mode'),
                'target_coco_class_id': LaunchConfiguration('target_coco_class_id'),
                'target_coco_class_name': LaunchConfiguration('target_coco_class_name'),
                'target_coco_class_names': LaunchConfiguration('target_coco_class_names'),
                'proxy_center_weight': LaunchConfiguration('proxy_center_weight'),
                'proxy_center_max_px': LaunchConfiguration('proxy_center_max_px'),
                'proxy_switch_max_px': LaunchConfiguration('proxy_switch_max_px'),
                'proxy_switch_min_conf': LaunchConfiguration('proxy_switch_min_conf'),
                'proxy_switch_force_local': LaunchConfiguration('proxy_switch_force_local'),
                'proxy_use_geom_filters': LaunchConfiguration('proxy_use_geom_filters'),
                'event_topic': LaunchConfiguration('event_topic'),
            },
        ],
    )

    follow_node = Node(
        package='lrs_halmstad',
        executable='follow_uav',
        name='follow_uav',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'world': LaunchConfiguration('world'),
                'uav_name': LaunchConfiguration('uav_name'),
                'leader_input_type': LaunchConfiguration('leader_mode'),
                'leader_odom_topic': LaunchConfiguration('ugv_odom_topic'),
                'event_topic': LaunchConfiguration('event_topic'),
                'uav_backend': LaunchConfiguration('uav_backend'),
                'controller_seed_x': LaunchConfiguration('backend_initial_x'),
                'controller_seed_y': LaunchConfiguration('backend_initial_y'),
                'controller_seed_yaw_deg': LaunchConfiguration('backend_initial_yaw_deg'),
                'track_only_allow_reacquire_motion': LaunchConfiguration('track_only_allow_reacquire_motion'),
                'camera_pose_control_mode': LaunchConfiguration('camera_pose_control_mode'),
                'camera_pose_failover_failures': LaunchConfiguration('camera_pose_failover_failures'),
                'camera_lock_enable': LaunchConfiguration('camera_lock_enable'),
                'camera_lock_tilt_enable': LaunchConfiguration('camera_lock_tilt_enable'),
                'camera_lock_tilt_gain_per_px': LaunchConfiguration('camera_lock_tilt_gain_per_px'),
                'camera_lock_tilt_deadband_px': LaunchConfiguration('camera_lock_tilt_deadband_px'),
                'camera_lock_tilt_min_deg': LaunchConfiguration('camera_lock_tilt_min_deg'),
                'camera_lock_tilt_max_deg': LaunchConfiguration('camera_lock_tilt_max_deg'),
                'camera_lock_tilt_max_step_deg': LaunchConfiguration('camera_lock_tilt_max_step_deg'),
                'under_target_guard_enable': LaunchConfiguration('under_target_guard_enable'),
                'under_target_err_v_ratio': LaunchConfiguration('under_target_err_v_ratio'),
                'under_target_range_m': LaunchConfiguration('under_target_range_m'),
                'under_target_hold_xy': LaunchConfiguration('under_target_hold_xy'),
                'under_target_hold_yaw': LaunchConfiguration('under_target_hold_yaw'),
                'reacquire_sequence_enable': LaunchConfiguration('reacquire_sequence_enable'),
                'reacquire_pan_phase_s': LaunchConfiguration('reacquire_pan_phase_s'),
                'reacquire_tilt_phase_s': LaunchConfiguration('reacquire_tilt_phase_s'),
                'reacquire_yaw_phase_s': LaunchConfiguration('reacquire_yaw_phase_s'),
                'reacquire_pan_local_span_deg': LaunchConfiguration('reacquire_pan_local_span_deg'),
                'reacquire_tilt_local_span_deg': LaunchConfiguration('reacquire_tilt_local_span_deg'),
                'reacquire_pan_rate_deg_s': LaunchConfiguration('reacquire_pan_rate_deg_s'),
                'reacquire_tilt_rate_deg_s': LaunchConfiguration('reacquire_tilt_rate_deg_s'),
                'reacquire_yaw_rate_deg_s': LaunchConfiguration('reacquire_yaw_rate_deg_s'),
                'reacquire_hold_phase_s': LaunchConfiguration('reacquire_hold_phase_s'),
                'reacquire_fallback_hold_s': LaunchConfiguration('reacquire_fallback_hold_s'),
                'reacquire_hold_after_search': LaunchConfiguration('reacquire_hold_after_search'),
                'reacquire_search_hold_xy': LaunchConfiguration('reacquire_search_hold_xy'),
                'reacquire_search_hold_yaw_before_yaw_phase': LaunchConfiguration('reacquire_search_hold_yaw_before_yaw_phase'),
                'reacquire_commit_enable': LaunchConfiguration('reacquire_commit_enable'),
                'reacquire_commit_s': LaunchConfiguration('reacquire_commit_s'),
                'reacquire_commit_quality_floor': LaunchConfiguration('reacquire_commit_quality_floor'),
                'reacquire_startup_guard_enable': LaunchConfiguration('reacquire_startup_guard_enable'),
                'reacquire_first_lock_gate_enable': LaunchConfiguration('reacquire_first_lock_gate_enable'),
                'reacquire_first_lock_min_visible_frames': LaunchConfiguration('reacquire_first_lock_min_visible_frames'),
                'reacquire_relock_min_visible_frames': LaunchConfiguration('reacquire_relock_min_visible_frames'),
                'reacquire_yaw_assist_delay_s': LaunchConfiguration('reacquire_yaw_assist_delay_s'),
                'reacquire_xy_assist_delay_s': LaunchConfiguration('reacquire_xy_assist_delay_s'),
                'follow_yaw': LaunchConfiguration('follow_yaw'),
                'xy_motion_enable': LaunchConfiguration('xy_motion_enable'),
                'publish_debug_status': LaunchConfiguration('publish_follow_debug_status'),
                'debug_status_topic': LaunchConfiguration('follow_debug_status_topic'),
            },
        ],
    )

    simulator_backend_node = Node(
        package='lrs_halmstad',
        executable='simulator',
        name='simulator',
        namespace=LaunchConfiguration('uav_name'),
        output='screen',
        condition=_controller_backend_condition(),
        parameters=[
            {
                'world': LaunchConfiguration('world'),
                'name': LaunchConfiguration('uav_name'),
                'frame_id': LaunchConfiguration('backend_frame_id'),
                'initial_x': LaunchConfiguration('backend_initial_x'),
                'initial_y': LaunchConfiguration('backend_initial_y'),
                'initial_z': LaunchConfiguration('backend_initial_z'),
                'initial_yaw_deg': LaunchConfiguration('backend_initial_yaw_deg'),
                'initial_pan_deg': LaunchConfiguration('backend_initial_pan_deg'),
                'initial_tilt_deg': LaunchConfiguration('backend_initial_tilt_deg'),
            },
        ],
    )

    ugv_node = Node(
        package='lrs_halmstad',
        executable='ugv_motion_driver',
        name='ugv_motion_driver',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'cmd_topic': LaunchConfiguration('ugv_cmd_topic'),
                'start_delay_s': LaunchConfiguration('ugv_start_delay_s'),
                'ready_check_enable': True,
                'ready_require_cmd_subscriber': True,
                'ready_require_odom_flow': True,
                'ready_odom_topic': LaunchConfiguration('ugv_odom_topic'),
                'ready_require_leader_flow': LaunchConfiguration('ugv_ready_require_leader_flow'),
                'ready_leader_topic': LaunchConfiguration('ugv_ready_leader_topic'),
                'ready_timeout_s': LaunchConfiguration('ugv_ready_timeout_s'),
            },
        ],
    )

    ugv_delayed_start = TimerAction(
        period=0.1,
        actions=[
            LogInfo(msg='[run_round_follow_motion] Starting UGV motion driver (follow stack already launched)'),
            ugv_node,
        ],
    )

    control_chain_log = LogInfo(
        msg=[
            '[run_round_follow_motion] Control chain: leader_mode=',
            LaunchConfiguration('leader_mode'),
            ' dependency=',
            LaunchConfiguration('leader_dependency_mode'),
            ' -> follow_uav intent=/',
            LaunchConfiguration('uav_name'),
            '/pose_cmd -> backend=',
            LaunchConfiguration('uav_backend'),
            ' (no manual UAV command path in this launch)',
        ],
    )

    shutdown_on_ugv_done = RegisterEventHandler(
        OnProcessExit(
            target_action=ugv_node,
            on_exit=[
                LogInfo(msg='[run_round_follow_motion] ugv_motion_driver exited; shutting down launch'),
                EmitEvent(event=Shutdown(reason='UGV motion complete')),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('shutdown_when_ugv_done')),
    )
    shutdown_on_simulator_exit = RegisterEventHandler(
        OnProcessExit(
            target_action=simulator_backend_node,
            on_exit=[
                LogInfo(msg='[run_round_follow_motion] simulator backend exited in controller mode; shutting down launch'),
                EmitEvent(event=Shutdown(reason='Simulator backend exited')),
            ],
        ),
        condition=_simulator_failfast_condition(),
    )

    return LaunchDescription([
        params_file_arg,
        world_arg,
        uav_name_arg,
        leader_mode_arg,
        leader_dependency_mode_arg,
        uav_backend_arg,
        leader_perception_enable_arg,
        start_estimator_arg,
        ugv_cmd_topic_arg,
        ugv_odom_topic_arg,
        leader_image_topic_arg,
        leader_camera_info_topic_arg,
        leader_camera_tilt_topic_arg,
        leader_depth_topic_arg,
        leader_uav_pose_topic_arg,
        backend_frame_id_arg,
        backend_initial_x_arg,
        backend_initial_y_arg,
        backend_initial_z_arg,
        backend_initial_yaw_deg_arg,
        backend_initial_pan_deg_arg,
        backend_initial_tilt_deg_arg,
        yolo_weights_arg,
        yolo_device_arg,
        conf_threshold_arg,
        iou_threshold_arg,
        target_class_id_arg,
        target_class_name_arg,
        target_class_names_arg,
        target_mode_arg,
        target_coco_class_id_arg,
        target_coco_class_name_arg,
        target_coco_class_names_arg,
        proxy_center_weight_arg,
        proxy_center_max_px_arg,
        proxy_switch_max_px_arg,
        proxy_switch_min_conf_arg,
        proxy_switch_force_local_arg,
        proxy_use_geom_filters_arg,
        track_only_allow_reacquire_motion_arg,
        camera_pose_control_mode_arg,
        camera_pose_failover_failures_arg,
        camera_lock_enable_arg,
        camera_lock_tilt_enable_arg,
        camera_lock_tilt_gain_per_px_arg,
        camera_lock_tilt_deadband_px_arg,
        camera_lock_tilt_min_deg_arg,
        camera_lock_tilt_max_deg_arg,
        camera_lock_tilt_max_step_deg_arg,
        under_target_guard_enable_arg,
        under_target_err_v_ratio_arg,
        under_target_range_m_arg,
        under_target_hold_xy_arg,
        under_target_hold_yaw_arg,
        reacquire_sequence_enable_arg,
        reacquire_pan_phase_s_arg,
        reacquire_tilt_phase_s_arg,
        reacquire_yaw_phase_s_arg,
        reacquire_pan_local_span_deg_arg,
        reacquire_tilt_local_span_deg_arg,
        reacquire_pan_rate_deg_s_arg,
        reacquire_tilt_rate_deg_s_arg,
        reacquire_yaw_rate_deg_s_arg,
        reacquire_hold_phase_s_arg,
        reacquire_fallback_hold_s_arg,
        reacquire_hold_after_search_arg,
        reacquire_search_hold_xy_arg,
        reacquire_search_hold_yaw_before_yaw_phase_arg,
        reacquire_commit_enable_arg,
        reacquire_commit_s_arg,
        reacquire_commit_quality_floor_arg,
        reacquire_startup_guard_enable_arg,
        reacquire_first_lock_gate_enable_arg,
        reacquire_first_lock_min_visible_frames_arg,
        reacquire_relock_min_visible_frames_arg,
        reacquire_yaw_assist_delay_s_arg,
        reacquire_xy_assist_delay_s_arg,
        follow_yaw_arg,
        xy_motion_enable_arg,
        publish_follow_debug_status_arg,
        follow_debug_status_topic_arg,
        event_topic_arg,
        ugv_start_delay_arg,
        ugv_ready_require_leader_flow_arg,
        ugv_ready_leader_topic_arg,
        ugv_ready_timeout_arg,
        shutdown_when_ugv_done_arg,
        control_chain_log,
        estimator_node,
        follow_node,
        simulator_backend_node,
        ugv_delayed_start,
        shutdown_on_ugv_done,
        shutdown_on_simulator_exit,
    ])
