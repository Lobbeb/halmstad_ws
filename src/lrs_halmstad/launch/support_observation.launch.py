import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _support_detector_instance(
    *,
    instance_id,
    params_file,
    uav_name,
    camera_name,
    detector_backend,
    detector_onnx_model,
    yolo_weights,
):
    node_name = f'support_{instance_id}_leader_detector'
    detector_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'event_topic': f'/coord/support/{instance_id}/leader_detection_events',
            'out_topic': f'/coord/support/{instance_id}/leader_detection',
            'status_topic': f'/coord/support/{instance_id}/leader_detection_status',
            'backend': detector_backend,
            'onnx_model': detector_onnx_model,
            'async_inference': LaunchConfiguration('detector_async_inference'),
            'latest_frame_only': LaunchConfiguration('detector_latest_frame_only'),
            'stale_detection_threshold_ms': LaunchConfiguration('detector_stale_detection_threshold_ms'),
            'metrics_window_s': LaunchConfiguration('detector_metrics_window_s'),
            'benchmark_csv_path': LaunchConfiguration('detector_benchmark_csv_path'),
            'image_qos_depth': LaunchConfiguration('detector_image_qos_depth'),
            'image_qos_reliability': LaunchConfiguration('detector_image_qos_reliability'),
            'target_class_name': LaunchConfiguration('target_class_name'),
            'target_class_id': LaunchConfiguration('target_class_id'),
            'device': LaunchConfiguration('yolo_device'),
            'yolo_weights': yolo_weights,
        },
        key_rewrites={'leader_detector': node_name},
        convert_types=True,
    )

    return Node(
        package='lrs_halmstad',
        executable='leader_detector',
        name=node_name,
        output='screen',
        parameters=[
            detector_params,
            {
                'use_sim_time': True,
                'uav_name': uav_name,
                'camera_topic': ['/', uav_name, '/', camera_name, '/image_raw'],
            },
        ],
    )


def generate_launch_description():
    share_dir = get_package_share_directory('lrs_halmstad')
    default_params_file = os.path.join(share_dir, 'config', 'run_follow_defaults.yaml')

    world_arg = DeclareLaunchArgument(
        'world',
        default_value='warehouse',
        description='Unused operator-facing symmetry argument so the overlay matches repo run patterns.',
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Parameter YAML reused from the trusted detector runtime defaults.',
    )
    camera_name_arg = DeclareLaunchArgument('camera_name', default_value='camera0')
    dji1_name_arg = DeclareLaunchArgument('dji1_name', default_value='dji1')
    dji2_name_arg = DeclareLaunchArgument('dji2_name', default_value='dji2')
    detector_backend_arg = DeclareLaunchArgument('detector_backend', default_value='onnx_cpu')
    detector_onnx_model_arg = DeclareLaunchArgument('detector_onnx_model', default_value='')
    yolo_weights_arg = DeclareLaunchArgument('yolo_weights', default_value='')
    support_detector_backend_arg = DeclareLaunchArgument(
        'support_detector_backend',
        default_value=LaunchConfiguration('detector_backend'),
        description='Shared detector backend for support UAVs unless overridden per UAV.',
    )
    support_detector_onnx_model_arg = DeclareLaunchArgument(
        'support_detector_onnx_model',
        default_value=LaunchConfiguration('detector_onnx_model'),
        description='Shared support detector ONNX model unless overridden per UAV.',
    )
    support_yolo_weights_arg = DeclareLaunchArgument(
        'support_yolo_weights',
        default_value=LaunchConfiguration('yolo_weights'),
        description='Shared support Ultralytics weights unless overridden per UAV.',
    )
    dji1_detector_backend_arg = DeclareLaunchArgument(
        'dji1_detector_backend',
        default_value=LaunchConfiguration('support_detector_backend'),
    )
    dji1_detector_onnx_model_arg = DeclareLaunchArgument(
        'dji1_detector_onnx_model',
        default_value=LaunchConfiguration('support_detector_onnx_model'),
    )
    dji1_yolo_weights_arg = DeclareLaunchArgument(
        'dji1_yolo_weights',
        default_value=LaunchConfiguration('support_yolo_weights'),
    )
    dji2_detector_backend_arg = DeclareLaunchArgument(
        'dji2_detector_backend',
        default_value=LaunchConfiguration('support_detector_backend'),
    )
    dji2_detector_onnx_model_arg = DeclareLaunchArgument(
        'dji2_detector_onnx_model',
        default_value=LaunchConfiguration('support_detector_onnx_model'),
    )
    dji2_yolo_weights_arg = DeclareLaunchArgument(
        'dji2_yolo_weights',
        default_value=LaunchConfiguration('support_yolo_weights'),
    )
    yolo_device_arg = DeclareLaunchArgument('yolo_device', default_value='auto')
    detector_async_inference_arg = DeclareLaunchArgument('detector_async_inference', default_value='true')
    detector_latest_frame_only_arg = DeclareLaunchArgument('detector_latest_frame_only', default_value='true')
    detector_stale_detection_threshold_ms_arg = DeclareLaunchArgument(
        'detector_stale_detection_threshold_ms',
        default_value='500.0',
    )
    detector_metrics_window_s_arg = DeclareLaunchArgument(
        'detector_metrics_window_s',
        default_value='5.0',
    )
    detector_benchmark_csv_path_arg = DeclareLaunchArgument(
        'detector_benchmark_csv_path',
        default_value='',
    )
    detector_image_qos_depth_arg = DeclareLaunchArgument('detector_image_qos_depth', default_value='1')
    detector_image_qos_reliability_arg = DeclareLaunchArgument(
        'detector_image_qos_reliability',
        default_value='best_effort',
    )
    target_class_name_arg = DeclareLaunchArgument('target_class_name', default_value='')
    target_class_id_arg = DeclareLaunchArgument('target_class_id', default_value='-1')
    support_mux_enable_arg = DeclareLaunchArgument('support_mux_enable', default_value='true')
    support_mux_publish_rate_hz_arg = DeclareLaunchArgument('support_mux_publish_rate_hz', default_value='10.0')
    support_mux_source_stale_timeout_s_arg = DeclareLaunchArgument(
        'support_mux_source_stale_timeout_s',
        default_value='0.75',
    )
    support_mux_out_detection_topic_arg = DeclareLaunchArgument(
        'support_mux_out_detection_topic',
        default_value='/coord/dji0/leader_detection',
    )
    support_mux_out_status_topic_arg = DeclareLaunchArgument(
        'support_mux_out_status_topic',
        default_value='/coord/dji0/leader_detection_status',
    )
    support_mux_out_summary_topic_arg = DeclareLaunchArgument(
        'support_mux_out_summary_topic',
        default_value='/coord/dji0/support_observation_summary',
    )
    support_mux_relation_source_arg = DeclareLaunchArgument(
        'support_mux_relation_source',
        default_value='odom',
    )
    support_mux_relation_quality_arg = DeclareLaunchArgument(
        'support_mux_relation_quality',
        default_value='not_evaluated',
    )
    support_mux_relation_note_arg = DeclareLaunchArgument(
        'support_mux_relation_note',
        default_value='support_slots',
    )
    hazard_fusion_enable_arg = DeclareLaunchArgument(
        'hazard_fusion_enable',
        default_value='false',
        description='Enable the typed dji1/dji2 to dji0 hazard selector.',
    )
    hazard_fusion_dji1_topic_arg = DeclareLaunchArgument(
        'hazard_fusion_dji1_topic',
        default_value='/coord/support/dji1/aerial_hazards',
    )
    hazard_fusion_dji2_topic_arg = DeclareLaunchArgument(
        'hazard_fusion_dji2_topic',
        default_value='/coord/support/dji2/aerial_hazards',
    )
    hazard_fusion_out_topic_arg = DeclareLaunchArgument(
        'hazard_fusion_out_topic',
        default_value='/coord/dji0/aerial_hazards',
    )
    hazard_fusion_stale_timeout_s_arg = DeclareLaunchArgument(
        'hazard_fusion_stale_timeout_s',
        default_value='0.75',
    )
    hazard_fusion_max_covariance_arg = DeclareLaunchArgument(
        'hazard_fusion_max_covariance',
        default_value='4.0',
    )
    hazard_fusion_publish_rate_hz_arg = DeclareLaunchArgument(
        'hazard_fusion_publish_rate_hz',
        default_value='10.0',
    )
    ugv_forward_enable_arg = DeclareLaunchArgument('ugv_forward_enable', default_value='true')
    ugv_forward_owner_arg = DeclareLaunchArgument('ugv_forward_owner', default_value='dji0')
    ugv_forward_stage_arg = DeclareLaunchArgument('ugv_forward_stage', default_value='dji0_to_ugv')
    ugv_forward_out_detection_topic_arg = DeclareLaunchArgument(
        'ugv_forward_out_detection_topic',
        default_value='/coord/ugv/leader_detection',
    )
    ugv_forward_out_status_topic_arg = DeclareLaunchArgument(
        'ugv_forward_out_status_topic',
        default_value='/coord/ugv/leader_detection_status',
    )
    ugv_forward_out_summary_topic_arg = DeclareLaunchArgument(
        'ugv_forward_out_summary_topic',
        default_value='/coord/ugv/support_observation_summary',
    )
    hazard_forward_enable_arg = DeclareLaunchArgument(
        'hazard_forward_enable',
        default_value='false',
        description='Enable typed dji0 to UGV hazard forwarding.',
    )
    hazard_in_topic_arg = DeclareLaunchArgument(
        'hazard_in_topic',
        default_value='/coord/dji0/aerial_hazards',
    )
    hazard_out_topic_arg = DeclareLaunchArgument(
        'hazard_out_topic',
        default_value='/coord/ugv/aerial_hazards',
    )
    start_ugv_support_awareness_arg = DeclareLaunchArgument(
        'start_ugv_support_awareness',
        default_value='true',
        description='Publish a compact UGV-side status line from the structured support summary.',
    )
    ugv_support_awareness_status_topic_arg = DeclareLaunchArgument(
        'ugv_support_awareness_status_topic',
        default_value='/coord/ugv/support_awareness_status',
    )
    support_awareness_publish_advisory_arg = DeclareLaunchArgument(
        'support_awareness_publish_advisory',
        default_value='true',
        description='Publish status-only UGV advisory text for future replanning/costmap integration.',
    )
    ugv_support_path_advisory_topic_arg = DeclareLaunchArgument(
        'ugv_support_path_advisory_topic',
        default_value='/coord/ugv/support_path_advisory',
    )
    support_camera_scan_enable_arg = DeclareLaunchArgument(
        'support_camera_scan_enable',
        default_value='false',
        description='Optionally sweep support-UAV camera pan/tilt commands for data collection.',
    )
    support_camera_scan_uavs_arg = DeclareLaunchArgument('support_camera_scan_uavs', default_value='dji1,dji2')
    support_camera_scan_yaw_center_deg_arg = DeclareLaunchArgument(
        'support_camera_scan_yaw_center_deg',
        default_value='0.0',
    )
    support_camera_scan_yaw_amplitude_deg_arg = DeclareLaunchArgument(
        'support_camera_scan_yaw_amplitude_deg',
        default_value='35.0',
    )
    support_camera_scan_period_s_arg = DeclareLaunchArgument(
        'support_camera_scan_period_s',
        default_value='8.0',
    )
    support_camera_scan_pan_phase_offsets_deg_arg = DeclareLaunchArgument(
        'support_camera_scan_pan_phase_offsets_deg',
        default_value='',
        description='Comma-separated per-UAV pan phase offsets in degrees, for example 0,180 for mirrored sweeps.',
    )
    support_camera_scan_pitch_deg_arg = DeclareLaunchArgument(
        'support_camera_scan_pitch_deg',
        default_value='-20.0',
    )
    support_camera_scan_pitch_amplitude_deg_arg = DeclareLaunchArgument(
        'support_camera_scan_pitch_amplitude_deg',
        default_value='0.0',
        description='Optional sinusoidal pitch amplitude in degrees; 0 keeps a fixed tilt.',
    )
    support_camera_scan_pitch_period_s_arg = DeclareLaunchArgument(
        'support_camera_scan_pitch_period_s',
        default_value='0.0',
        description='Optional pitch sweep period in seconds; 0 reuses the pan period.',
    )
    support_camera_scan_pitch_phase_offsets_deg_arg = DeclareLaunchArgument(
        'support_camera_scan_pitch_phase_offsets_deg',
        default_value='',
        description='Comma-separated per-UAV pitch phase offsets in degrees.',
    )
    support_camera_scan_rate_hz_arg = DeclareLaunchArgument(
        'support_camera_scan_rate_hz',
        default_value='10.0',
    )

    support_dji1_detector = _support_detector_instance(
        instance_id='dji1',
        params_file=LaunchConfiguration('params_file'),
        uav_name=LaunchConfiguration('dji1_name'),
        camera_name=LaunchConfiguration('camera_name'),
        detector_backend=LaunchConfiguration('dji1_detector_backend'),
        detector_onnx_model=LaunchConfiguration('dji1_detector_onnx_model'),
        yolo_weights=LaunchConfiguration('dji1_yolo_weights'),
    )
    support_dji2_detector = _support_detector_instance(
        instance_id='dji2',
        params_file=LaunchConfiguration('params_file'),
        uav_name=LaunchConfiguration('dji2_name'),
        camera_name=LaunchConfiguration('camera_name'),
        detector_backend=LaunchConfiguration('dji2_detector_backend'),
        detector_onnx_model=LaunchConfiguration('dji2_detector_onnx_model'),
        yolo_weights=LaunchConfiguration('dji2_yolo_weights'),
    )

    support_detection_mux = Node(
        package='lrs_halmstad',
        executable='support_detection_mux',
        name='support_detection_mux',
        output='screen',
        condition=IfCondition(LaunchConfiguration('support_mux_enable')),
        parameters=[
            {
                'use_sim_time': True,
                'source_a_id': LaunchConfiguration('dji1_name'),
                'source_b_id': LaunchConfiguration('dji2_name'),
                'publish_rate_hz': LaunchConfiguration('support_mux_publish_rate_hz'),
                'source_stale_timeout_s': LaunchConfiguration('support_mux_source_stale_timeout_s'),
                'out_detection_topic': LaunchConfiguration('support_mux_out_detection_topic'),
                'out_status_topic': LaunchConfiguration('support_mux_out_status_topic'),
                'out_summary_topic': LaunchConfiguration('support_mux_out_summary_topic'),
                'relation_source': LaunchConfiguration('support_mux_relation_source'),
                'relation_quality': LaunchConfiguration('support_mux_relation_quality'),
                'relation_note': LaunchConfiguration('support_mux_relation_note'),
            }
        ],
    )

    support_hazard_fusion = Node(
        package='lrs_halmstad',
        executable='support_hazard_fusion',
        name='support_hazard_fusion',
        output='screen',
        condition=IfCondition(LaunchConfiguration('hazard_fusion_enable')),
        parameters=[
            {
                'use_sim_time': True,
                'dji1_topic': LaunchConfiguration('hazard_fusion_dji1_topic'),
                'dji2_topic': LaunchConfiguration('hazard_fusion_dji2_topic'),
                'output_topic': LaunchConfiguration('hazard_fusion_out_topic'),
                'stale_timeout_s': LaunchConfiguration('hazard_fusion_stale_timeout_s'),
                'max_covariance': LaunchConfiguration('hazard_fusion_max_covariance'),
                'publish_rate_hz': LaunchConfiguration('hazard_fusion_publish_rate_hz'),
            }
        ],
    )

    dji0_to_ugv_forwarder = Node(
        package='lrs_halmstad',
        executable='dji0_to_ugv_forwarder',
        name='dji0_to_ugv_forwarder',
        output='screen',
        condition=IfCondition(LaunchConfiguration('ugv_forward_enable')),
        parameters=[
            {
                'use_sim_time': True,
                'in_detection_topic': LaunchConfiguration('support_mux_out_detection_topic'),
                'in_status_topic': LaunchConfiguration('support_mux_out_status_topic'),
                'in_summary_topic': LaunchConfiguration('support_mux_out_summary_topic'),
                'out_detection_topic': LaunchConfiguration('ugv_forward_out_detection_topic'),
                'out_status_topic': LaunchConfiguration('ugv_forward_out_status_topic'),
                'out_summary_topic': LaunchConfiguration('ugv_forward_out_summary_topic'),
                'awareness_enable': LaunchConfiguration('start_ugv_support_awareness'),
                'out_awareness_status_topic': LaunchConfiguration('ugv_support_awareness_status_topic'),
                'publish_advisory': LaunchConfiguration('support_awareness_publish_advisory'),
                'out_advisory_topic': LaunchConfiguration('ugv_support_path_advisory_topic'),
                'forward_owner': LaunchConfiguration('ugv_forward_owner'),
                'forward_stage': LaunchConfiguration('ugv_forward_stage'),
                'hazard_forward_enable': LaunchConfiguration('hazard_forward_enable'),
                'in_hazard_topic': LaunchConfiguration('hazard_in_topic'),
                'out_hazard_topic': LaunchConfiguration('hazard_out_topic'),
            }
        ],
    )
    support_camera_scanner = Node(
        package='lrs_halmstad',
        executable='support_camera_scanner',
        name='support_camera_scanner',
        output='screen',
        condition=IfCondition(LaunchConfiguration('support_camera_scan_enable')),
        parameters=[
            {
                'use_sim_time': True,
                'uav_names': LaunchConfiguration('support_camera_scan_uavs'),
                'yaw_center_deg': LaunchConfiguration('support_camera_scan_yaw_center_deg'),
                'yaw_amplitude_deg': LaunchConfiguration('support_camera_scan_yaw_amplitude_deg'),
                'period_s': LaunchConfiguration('support_camera_scan_period_s'),
                'pan_phase_offsets_deg': LaunchConfiguration('support_camera_scan_pan_phase_offsets_deg'),
                'pitch_deg': LaunchConfiguration('support_camera_scan_pitch_deg'),
                'pitch_amplitude_deg': LaunchConfiguration('support_camera_scan_pitch_amplitude_deg'),
                'pitch_period_s': LaunchConfiguration('support_camera_scan_pitch_period_s'),
                'pitch_phase_offsets_deg': LaunchConfiguration('support_camera_scan_pitch_phase_offsets_deg'),
                'rate_hz': LaunchConfiguration('support_camera_scan_rate_hz'),
            }
        ],
    )

    return LaunchDescription([
        world_arg,
        params_file_arg,
        camera_name_arg,
        dji1_name_arg,
        dji2_name_arg,
        detector_backend_arg,
        detector_onnx_model_arg,
        yolo_weights_arg,
        support_detector_backend_arg,
        support_detector_onnx_model_arg,
        support_yolo_weights_arg,
        dji1_detector_backend_arg,
        dji1_detector_onnx_model_arg,
        dji1_yolo_weights_arg,
        dji2_detector_backend_arg,
        dji2_detector_onnx_model_arg,
        dji2_yolo_weights_arg,
        yolo_device_arg,
        detector_async_inference_arg,
        detector_latest_frame_only_arg,
        detector_stale_detection_threshold_ms_arg,
        detector_metrics_window_s_arg,
        detector_benchmark_csv_path_arg,
        detector_image_qos_depth_arg,
        detector_image_qos_reliability_arg,
        target_class_name_arg,
        target_class_id_arg,
        support_mux_enable_arg,
        support_mux_publish_rate_hz_arg,
        support_mux_source_stale_timeout_s_arg,
        support_mux_out_detection_topic_arg,
        support_mux_out_status_topic_arg,
        support_mux_out_summary_topic_arg,
        support_mux_relation_source_arg,
        support_mux_relation_quality_arg,
        support_mux_relation_note_arg,
        hazard_fusion_enable_arg,
        hazard_fusion_dji1_topic_arg,
        hazard_fusion_dji2_topic_arg,
        hazard_fusion_out_topic_arg,
        hazard_fusion_stale_timeout_s_arg,
        hazard_fusion_max_covariance_arg,
        hazard_fusion_publish_rate_hz_arg,
        ugv_forward_enable_arg,
        ugv_forward_owner_arg,
        ugv_forward_stage_arg,
        ugv_forward_out_detection_topic_arg,
        ugv_forward_out_status_topic_arg,
        ugv_forward_out_summary_topic_arg,
        hazard_forward_enable_arg,
        hazard_in_topic_arg,
        hazard_out_topic_arg,
        start_ugv_support_awareness_arg,
        ugv_support_awareness_status_topic_arg,
        support_awareness_publish_advisory_arg,
        ugv_support_path_advisory_topic_arg,
        support_camera_scan_enable_arg,
        support_camera_scan_uavs_arg,
        support_camera_scan_yaw_center_deg_arg,
        support_camera_scan_yaw_amplitude_deg_arg,
        support_camera_scan_period_s_arg,
        support_camera_scan_pan_phase_offsets_deg_arg,
        support_camera_scan_pitch_deg_arg,
        support_camera_scan_pitch_amplitude_deg_arg,
        support_camera_scan_pitch_period_s_arg,
        support_camera_scan_pitch_phase_offsets_deg_arg,
        support_camera_scan_rate_hz_arg,
        support_dji1_detector,
        support_dji2_detector,
        support_detection_mux,
        support_hazard_fusion,
        dji0_to_ugv_forwarder,
        support_camera_scanner,
    ])
