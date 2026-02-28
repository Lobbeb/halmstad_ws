import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def _gazebo_world_name(world_sub):
    return PythonExpression([
        "'office_construction' if '",
        world_sub,
        "' == 'construction' else '",
        world_sub,
        "'",
    ])


ARGUMENTS = [
    DeclareLaunchArgument(
        'rviz',
        default_value='false',
        choices=['true', 'false'],
        description='Start rviz.',
    ),
    DeclareLaunchArgument(
        'gui',
        default_value='true',
        choices=['true', 'false'],
        description='Start Gazebo GUI.',
    ),
    DeclareLaunchArgument(
        'world',
        default_value='orchard',
        choices=['construction', 'office', 'orchard', 'pipeline', 'solar_farm', 'warehouse'],
        description='Gazebo World',
    ),
    DeclareLaunchArgument(
        'setup_path',
        default_value=[EnvironmentVariable('HOME'), '/clearpath/'],
        description='Clearpath setup path',
    ),
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='use_sim_time',
    ),
    DeclareLaunchArgument(
        'auto_start',
        default_value='true',
        choices=['true', 'false'],
        description='Auto-start Gazebo simulation',
    ),
]

for pose_element in ['x', 'y', 'yaw']:
    ARGUMENTS.append(
        DeclareLaunchArgument(
            pose_element,
            default_value='0.0',
            description=f'{pose_element} component of the robot pose.',
        )
    )

ARGUMENTS.append(
    DeclareLaunchArgument(
        'z',
        default_value='0.3',
        description='z component of the robot pose.',
    )
)


def _gz_launch(context, *args, **kwargs):
    pkg_lrs_halmstad = get_package_share_directory('lrs_halmstad')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gui_config_path = os.path.join(pkg_lrs_halmstad, 'config', 'gui.config')

    auto_start_option = ''
    if LaunchConfiguration('auto_start').perform(context) == 'true':
        auto_start_option = ' -r'

    gz_args = [
        LaunchConfiguration('world'),
        '.sdf',
        auto_start_option,
        ' -v 4',
    ]
    if LaunchConfiguration('gui').perform(context) == 'true':
        gz_args.extend([
            ' --gui-config ',
            gui_config_path,
        ])
    else:
        gz_args.append(' -s')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([gz_sim_launch]),
        launch_arguments=[
            ('gz_args', gz_args),
            ('on_exit_shutdown', 'true'),
        ],
    )

    return [gz_sim]


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])

    ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')
    packages_paths = [
        os.path.join(prefix, 'share')
        for prefix in ament_prefix_path.split(':')
        if prefix
    ]

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(pkg_clearpath_gz, 'worlds') + ':',
            os.path.join(pkg_clearpath_gz, 'meshes') + ':',
            ':' + ':'.join(packages_paths),
        ],
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='clock_bridge',
        output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
    )

    robot_spawn = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([robot_spawn_launch]),
        launch_arguments=[
            ('use_sim_time', LaunchConfiguration('use_sim_time')),
            ('setup_path', LaunchConfiguration('setup_path')),
            ('world', _gazebo_world_name(LaunchConfiguration('world'))),
            ('rviz', LaunchConfiguration('rviz')),
            ('x', LaunchConfiguration('x')),
            ('y', LaunchConfiguration('y')),
            ('z', LaunchConfiguration('z')),
            ('yaw', LaunchConfiguration('yaw')),
        ],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(OpaqueFunction(function=_gz_launch))
    ld.add_action(clock_bridge)
    ld.add_action(robot_spawn)
    return ld
