"""Lightweight Baylands planner validation for the typed aerial-hazard chain."""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction
from launch.conditions import IfCondition
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


SCENARIOS = {
    'baseline',
    'valid',
    'clearing',
    'off_route',
    'low_confidence',
    'stale',
    'layer_disabled',
}


def _scenario_settings(name: str) -> dict[str, object]:
    settings: dict[str, object] = {
        'publish': name != 'baseline',
        'hazard_x': -72.0,
        'hazard_y': 195.5,
        'confidence': 0.9,
        'stamp_offset_s': 0.0,
        'start_delay_s': 2.0,
        'active_duration_s': 60.0,
        'publish_empty': False,
    }
    if name == 'clearing':
        settings['start_delay_s'] = 10.0
        settings['active_duration_s'] = 35.0
        settings['publish_empty'] = True
    elif name == 'off_route':
        settings['hazard_x'] = -60.0
    elif name == 'low_confidence':
        settings['confidence'] = 0.10
    elif name == 'stale':
        settings['stamp_offset_s'] = -5.0
    return settings


def _launch_setup(context, *args, **kwargs):
    scenario = LaunchConfiguration('scenario').perform(context).strip()
    if scenario not in SCENARIOS:
        raise RuntimeError(
            f"Unknown planner validation scenario '{scenario}'. "
            f"Choose one of: {', '.join(sorted(SCENARIOS))}"
        )
    settings = _scenario_settings(scenario)
    namespace = LaunchConfiguration('namespace').perform(context).strip().strip('/')
    if not namespace:
        raise RuntimeError('namespace must not be empty')

    params_file = LaunchConfiguration('params_file')
    rewritten_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites={
            'global_costmap.global_costmap.ros__parameters.aerial_support_layer.enabled':
                'false',
        },
        convert_types=True,
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        namespace=namespace,
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map'),
        }],
    )
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        namespace=namespace,
        name='planner_server',
        output='screen',
        parameters=[rewritten_params, {'use_sim_time': False}],
    )
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        namespace=namespace,
        name='lifecycle_manager_support_planner',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'bond_timeout': 4.0,
            'node_names': ['map_server', 'planner_server'],
        }],
    )
    fixed_robot_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='support_planner_fixed_robot_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('start_x'),
            '--y', LaunchConfiguration('start_y'),
            '--z', '0.0',
            '--yaw', LaunchConfiguration('start_yaw'),
            '--pitch', '0.0',
            '--roll', '0.0',
            '--frame-id', 'map',
            '--child-frame-id', 'base_link',
        ],
        parameters=[{'use_sim_time': False}],
    )
    fusion = Node(
        package='lrs_halmstad',
        executable='support_hazard_fusion',
        name='support_planner_hazard_fusion',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'dji1_topic': '/coord/support/dji1/aerial_hazards',
            'dji2_enable': False,
            'output_topic': '/coord/dji0/aerial_hazards',
        }],
    )
    forwarder = Node(
        package='lrs_halmstad',
        executable='dji0_to_ugv_forwarder',
        name='support_planner_dji0_to_ugv_forwarder',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'awareness_enable': False,
            'publish_advisory': False,
            'hazard_forward_enable': True,
            'in_hazard_topic': '/coord/dji0/aerial_hazards',
            'out_hazard_topic': '/coord/ugv/aerial_hazards',
        }],
    )

    actions = [
        map_server,
        planner_server,
        lifecycle_manager,
        fixed_robot_tf,
        fusion,
        forwarder,
    ]
    if bool(settings['publish']):
        actions.append(Node(
            package='lrs_halmstad',
            executable='synthetic_hazard_publisher',
            name='support_planner_synthetic_hazard',
            output='screen',
            parameters=[{
                'use_sim_time': False,
                'topic': '/coord/support/dji1/aerial_hazards',
                'source_uav': 'dji1',
                'class_name': 'hazard',
                'stable_track_id': f'planner_{scenario}_hazard',
                'center_x': float(settings['hazard_x']),
                'center_y': float(settings['hazard_y']),
                'center_z': 0.5,
                'yaw': 0.0,
                'dimension_x': 2.0,
                'dimension_y': 2.0,
                'dimension_z': 1.0,
                'confidence': float(settings['confidence']),
                'covariance': [
                    0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.25, 0.0, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.04, 0.0, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.04, 0.0,
                    0.0, 0.0, 0.0, 0.0, 0.0, 0.04,
                ],
                'state': 1,
                'start_delay_s': float(settings['start_delay_s']),
                'publish_rate_hz': 5.0,
                'ttl_s': 4.0,
                'active_duration_s': float(settings['active_duration_s']),
                'publish_empty_after_active_duration': bool(settings['publish_empty']),
                'stamp_offset_s': float(settings['stamp_offset_s']),
                'support_quality': 1.0,
                'provenance': f'planner_validation:{scenario}',
            }],
        ))

    actions.append(Node(
        package='lrs_halmstad',
        executable='support_hazard_evidence',
        name='support_planner_evidence',
        output='screen',
        arguments=[
            'planner-live',
            '--scenario', scenario,
            '--output', LaunchConfiguration('output'),
            '--namespace', namespace,
            '--map', LaunchConfiguration('map'),
            '--nav2-config', LaunchConfiguration('params_file'),
            '--start-x', LaunchConfiguration('start_x'),
            '--start-y', LaunchConfiguration('start_y'),
            '--start-yaw', LaunchConfiguration('start_yaw'),
            '--goal-x', LaunchConfiguration('goal_x'),
            '--goal-y', LaunchConfiguration('goal_y'),
            '--goal-yaw', LaunchConfiguration('goal_yaw'),
            '--hazard-x', str(settings['hazard_x']),
            '--hazard-y', str(settings['hazard_y']),
            '--timeout-s', LaunchConfiguration('timeout_s'),
        ],
        parameters=[{'use_sim_time': False}],
        on_exit=EmitEvent(event=Shutdown(reason='planner evidence completed')),
    ))

    actions.append(Node(
        package='rviz2',
        executable='rviz2',
        name='support_planner_rviz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
        arguments=['-d', LaunchConfiguration('rviz_config')],
        parameters=[{'use_sim_time': False}],
    ))
    return actions


def generate_launch_description():
    share_dir = get_package_share_directory('lrs_halmstad')
    return LaunchDescription([
        DeclareLaunchArgument('scenario', default_value='valid'),
        DeclareLaunchArgument('namespace', default_value='a201_0000'),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(share_dir, 'maps', 'baylands.yaml')
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(share_dir, 'config', 'nav2_baylands_large_map.yaml'),
        ),
        DeclareLaunchArgument('start_x', default_value='-71.39979517989994'),
        DeclareLaunchArgument('start_y', default_value='205.6352521524901'),
        DeclareLaunchArgument('start_yaw', default_value='-1.6133110777'),
        DeclareLaunchArgument('goal_x', default_value='-72.53159610937462'),
        DeclareLaunchArgument('goal_y', default_value='185.3861158209806'),
        DeclareLaunchArgument('goal_yaw', default_value='0.0066198909'),
        DeclareLaunchArgument('timeout_s', default_value='75.0'),
        DeclareLaunchArgument('output', default_value='/tmp/support_planner_validation'),
        DeclareLaunchArgument('rviz', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(
                share_dir, 'config', 'rviz_configs', 'localization_testing.rviz'
            ),
        ),
        OpaqueFunction(function=_launch_setup),
    ])
