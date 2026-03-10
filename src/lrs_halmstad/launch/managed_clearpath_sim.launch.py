import os
from pathlib import Path
from ament_index_python.packages import get_package_prefix, get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.actions import OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


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
        default_value='warehouse',
        description='Gazebo World',
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
    default_value = '0.0'
    ARGUMENTS.append(
        DeclareLaunchArgument(
            pose_element,
            default_value=default_value,
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
    world_name = LaunchConfiguration('world').perform(context)

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


def _default_clearpath_setup_path(pkg_lrs_halmstad: str) -> str:
    configured_path = os.environ.get("LRS_CLEARPATH_SETUP_PATH", "").strip()
    if configured_path:
        return os.path.expanduser(configured_path)
    share_path = Path(pkg_lrs_halmstad).resolve()
    for parent in share_path.parents:
        candidate = parent / "src" / "lrs_halmstad" / "clearpath"
        if candidate.is_dir():
            return str(candidate)
    if len(share_path.parents) >= 4:
        return str(share_path.parents[3] / "src" / "lrs_halmstad" / "clearpath")
    return str((share_path.parent / "clearpath").resolve())


def generate_launch_description():
    pkg_clearpath_gz = get_package_share_directory('clearpath_gz')
    pkg_lrs_halmstad = get_package_share_directory('lrs_halmstad')
    pkg_gui_plugins_prefix = get_package_prefix('lrs_halmstad_gui_plugins')
    robot_spawn_launch = PathJoinSubstitution([pkg_clearpath_gz, 'launch', 'robot_spawn.launch.py'])
    pkg_share_root = os.path.dirname(pkg_lrs_halmstad)
    ament_prefix_path = os.getenv('AMENT_PREFIX_PATH', '')
    packages_paths = [
        os.path.join(prefix, 'share')
        for prefix in ament_prefix_path.split(':')
        if prefix
    ]

    gz_sim_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            pkg_share_root + ':',
            os.path.join(pkg_lrs_halmstad, 'models') + ':',
            os.path.join(pkg_lrs_halmstad, 'worlds') + ':',
            os.path.join(pkg_clearpath_gz, 'worlds') + ':',
            os.path.join(pkg_clearpath_gz, 'meshes') + ':',
            ':' + ':'.join(packages_paths),
        ],
    )

    arguments = ARGUMENTS + [
        DeclareLaunchArgument(
            'setup_path',
            default_value=_default_clearpath_setup_path(pkg_lrs_halmstad),
            description='Clearpath setup path',
        ),
    ]

    gz_gui_plugin_path = SetEnvironmentVariable(
        name='GZ_GUI_PLUGIN_PATH',
        value=[
            os.path.join(pkg_gui_plugins_prefix, 'lib'),
            ':',
            EnvironmentVariable('GZ_GUI_PLUGIN_PATH', default_value=''),
        ],
    )

    gazebo_world_name = SetEnvironmentVariable(
        name='LRS_GAZEBO_WORLD',
        value=_gazebo_world_name(LaunchConfiguration('world')),
    )
    ugv_spawn_x = SetEnvironmentVariable(
        name='LRS_UGV_SPAWN_X',
        value=LaunchConfiguration('x'),
    )
    ugv_spawn_y = SetEnvironmentVariable(
        name='LRS_UGV_SPAWN_Y',
        value=LaunchConfiguration('y'),
    )
    ugv_spawn_z = SetEnvironmentVariable(
        name='LRS_UGV_SPAWN_Z',
        value=LaunchConfiguration('z'),
    )
    ugv_spawn_yaw = SetEnvironmentVariable(
        name='LRS_UGV_SPAWN_YAW',
        value=LaunchConfiguration('yaw'),
    )

    clock_bridge = RosGzBridge(
        bridge_name='clock_bridge',
        use_composition=False,
        extra_bridge_params={
            'bridges': {
                'bridge_0': {
                    'ros_topic_name': '/clock_raw',
                    'ros_type_name': 'rosgraph_msgs/msg/Clock',
                    'gz_topic_name': '/clock',
                    'gz_type_name': 'gz.msgs.Clock',
                    'direction': 'GZ_TO_ROS',
                    'lazy': False,
                },
            },
            'bridge_names': ['bridge_0'],
        },
    )

    clock_guard = Node(
        package='lrs_halmstad',
        executable='clock_guard',
        name='clock_guard',
        output='screen',
        parameters=[{
            'input_topic': '/clock_raw',
            'output_topic': '/clock',
        }],
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

    ld = LaunchDescription(arguments)
    ld.add_action(gz_sim_resource_path)
    ld.add_action(gz_gui_plugin_path)
    ld.add_action(gazebo_world_name)
    ld.add_action(ugv_spawn_x)
    ld.add_action(ugv_spawn_y)
    ld.add_action(ugv_spawn_z)
    ld.add_action(ugv_spawn_yaw)
    ld.add_action(OpaqueFunction(function=_gz_launch))
    ld.add_action(clock_bridge)
    ld.add_action(clock_guard)
    ld.add_action(robot_spawn)
    return ld
