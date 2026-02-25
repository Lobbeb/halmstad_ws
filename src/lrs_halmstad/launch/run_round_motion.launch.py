from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_default = PathJoinSubstitution(
        [FindPackageShare('lrs_halmstad'), 'config', 'run_round_motion_defaults.yaml']
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=params_default,
        description='Parameter YAML for UAV sweep and UGV motion nodes',
    )
    world_arg = DeclareLaunchArgument('world', default_value='orchard')
    uav_name_arg = DeclareLaunchArgument('uav_name', default_value='dji0')
    ugv_cmd_topic_arg = DeclareLaunchArgument('ugv_cmd_topic', default_value='/a201_0000/cmd_vel')
    uav_log_csv_arg = DeclareLaunchArgument('uav_log_csv', default_value='')

    uav_node = Node(
        package='lrs_halmstad',
        executable='uav_setpose_sweep',
        name='uav_setpose_sweep',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'world': LaunchConfiguration('world'),
                'uav_name': LaunchConfiguration('uav_name'),
                'log_csv': LaunchConfiguration('uav_log_csv'),
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
            },
        ],
    )

    start_ugv_after_uav = RegisterEventHandler(
        OnProcessExit(
            target_action=uav_node,
            on_exit=[
                LogInfo(msg='[run_round_motion] UAV sweep finished; starting UGV motion driver'),
                ugv_node,
            ],
        )
    )

    return LaunchDescription(
        [
            params_file_arg,
            world_arg,
            uav_name_arg,
            ugv_cmd_topic_arg,
            uav_log_csv_arg,
            uav_node,
            start_ugv_after_uav,
        ]
    )
