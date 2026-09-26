import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg_lrs = get_package_share_directory('lrs_halmstad')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')

    uav = LaunchConfiguration('uav')
    laser_name = LaunchConfiguration('laser_name')
    scan_topic = LaunchConfiguration('scan_topic')
    slam_params_file = LaunchConfiguration('slam_params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sync = LaunchConfiguration('sync')
    autostart = LaunchConfiguration('autostart')
    use_lifecycle_manager = LaunchConfiguration('use_lifecycle_manager')
    slam_max_laser_range = LaunchConfiguration('slam_max_laser_range')

    rewritten_parameters = RewrittenYaml(
        source_file=slam_params_file,
        root_key=uav,
        param_rewrites={
            'map_name': ['/', uav, '/map'],
            'scan_topic': scan_topic,
            'max_laser_range': slam_max_laser_range,
            'map_frame': 'map',
            'odom_frame': 'odom',
            'base_frame': 'base_link',
        },
        convert_types=True,
    )

    pose_to_odom = Node(
        package='lrs_halmstad',
        executable='pose_cmd_to_odom',
        name='uav_pose_to_odom',
        namespace=uav,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'pose_topic': 'pose',
            'odom_topic': 'pose/odom',
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'copy_header_stamp': True,
        }],
    )

    odom_to_tf = Node(
        package='lrs_halmstad',
        executable='odom_to_tf',
        name='uav_odom_to_tf',
        namespace=uav,
        output='screen',
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_topic': 'pose/odom',
            'frame_id': 'odom',
            'child_frame_id': 'base_link',
            'copy_header_stamp': True,
        }],
    )

    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='uav_laser_static_tf',
        namespace=uav,
        output='screen',
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        arguments=[
            '--x', LaunchConfiguration('laser_x'),
            '--y', LaunchConfiguration('laser_y'),
            '--z', LaunchConfiguration('laser_z'),
            '--roll', LaunchConfiguration('laser_roll'),
            '--pitch', LaunchConfiguration('laser_pitch'),
            '--yaw', LaunchConfiguration('laser_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', [laser_name, '_laser'],
        ],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    launch_slam_sync = PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_sync_launch.py'])
    launch_slam_async = PathJoinSubstitution([pkg_slam_toolbox, 'launch', 'online_async_launch.py'])

    slam_group = GroupAction([
        PushRosNamespace(uav),
        SetRemap('/tf', ['/', uav, '/tf']),
        SetRemap('/tf_static', ['/', uav, '/tf_static']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam_sync),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('autostart', autostart),
                ('use_lifecycle_manager', use_lifecycle_manager),
                ('slam_params_file', rewritten_parameters),
            ],
            condition=IfCondition(sync),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_slam_async),
            launch_arguments=[
                ('use_sim_time', use_sim_time),
                ('autostart', autostart),
                ('use_lifecycle_manager', use_lifecycle_manager),
                ('slam_params_file', rewritten_parameters),
            ],
            condition=UnlessCondition(sync),
        ),
    ])

    return LaunchDescription([
        DeclareLaunchArgument('uav', default_value='dji0'),
        DeclareLaunchArgument('laser_name', default_value='laser0'),
        DeclareLaunchArgument('scan_topic', default_value='/dji0/laser0/scan'),
        DeclareLaunchArgument(
            'slam_params_file',
            default_value=os.path.join(pkg_lrs, 'config', 'slam.yaml'),
        ),
        DeclareLaunchArgument('use_sim_time', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('sync', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('autostart', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('use_lifecycle_manager', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('slam_max_laser_range', default_value='25.0'),
        DeclareLaunchArgument('laser_x', default_value='0.0'),
        DeclareLaunchArgument('laser_y', default_value='0.0'),
        DeclareLaunchArgument('laser_z', default_value='0.5'),
        DeclareLaunchArgument('laser_roll', default_value='0.0'),
        DeclareLaunchArgument('laser_pitch', default_value='0.0'),
        DeclareLaunchArgument('laser_yaw', default_value='0.0'),
        pose_to_odom,
        odom_to_tf,
        laser_static_tf,
        slam_group,
    ])
