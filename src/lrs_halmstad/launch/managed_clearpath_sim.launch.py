import hashlib
import math
import os
import re
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
    DeclareLaunchArgument(
        'real_time_factor',
        default_value='1.0',
        description='Gazebo target real-time factor. >1.0 runs faster than real time if the machine can keep up.',
    ),
    DeclareLaunchArgument(
        'rtf',
        default_value='',
        description='Alias for real_time_factor.',
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

ARGUMENTS.extend([
    DeclareLaunchArgument(
        'view_follow_spawn',
        default_value='false',
        choices=['true', 'false'],
        description='Start the Gazebo GUI camera near the UGV spawn pose.',
    ),
    DeclareLaunchArgument(
        'view_distance',
        default_value='6.0',
        description='Distance behind the spawn pose for the initial Gazebo GUI camera.',
    ),
    DeclareLaunchArgument(
        'view_height',
        default_value='6.0',
        description='Height above the spawn pose for the initial Gazebo GUI camera.',
    ),
    DeclareLaunchArgument(
        'view_pitch',
        default_value='0.5',
        description='Pitch angle in radians for the initial Gazebo GUI camera.',
    ),
    DeclareLaunchArgument(
        'view_yaw_offset',
        default_value='0.0',
        description='Yaw offset in radians applied to the initial Gazebo GUI camera.',
    ),
])


def _gz_launch(context, *args, **kwargs):
    pkg_lrs_halmstad = get_package_share_directory('lrs_halmstad')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    world_name = LaunchConfiguration('world').perform(context)
    real_time_factor = _resolve_real_time_factor(context)
    world_sdf_path = _resolve_world_sdf_path(pkg_lrs_halmstad, world_name)
    world_launch_path = _prepare_world_launch_path(context, world_sdf_path, world_name, real_time_factor)

    gz_sim_launch = PathJoinSubstitution([pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py'])
    gui_config_path = _prepare_gui_config_path(context, os.path.join(pkg_lrs_halmstad, 'config', 'gui.config'))

    auto_start_option = ''
    if LaunchConfiguration('auto_start').perform(context) == 'true':
        auto_start_option = ' -r'

    gz_args = [
        world_launch_path,
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


def _resolve_real_time_factor(context) -> float:
    raw_alias = LaunchConfiguration('rtf').perform(context).strip()
    raw_value = raw_alias or LaunchConfiguration('real_time_factor').perform(context).strip()
    if not raw_value:
        return 1.0
    try:
        value = float(raw_value)
    except ValueError as exc:
        raise RuntimeError(f"Invalid real_time_factor/rtf value: '{raw_value}'") from exc
    if value <= 0.0:
        raise RuntimeError(f"real_time_factor/rtf must be > 0, got {value}")
    return value


def _resolve_world_sdf_path(pkg_lrs_halmstad: str, world_name: str) -> Path:
    candidate = Path(os.path.expanduser(world_name))
    if candidate.is_absolute():
        if not candidate.is_file():
            raise RuntimeError(f"World file not found: {candidate}")
        return candidate.resolve()

    world_file = world_name if world_name.endswith('.sdf') else f"{world_name}.sdf"
    candidate = (Path(pkg_lrs_halmstad) / 'worlds' / world_file).resolve()
    if not candidate.is_file():
        raise RuntimeError(f"World file not found: {candidate}")
    return candidate


def _prepare_world_launch_path(context, world_sdf_path: Path, world_name: str, real_time_factor: float) -> str:
    sdf_text = world_sdf_path.read_text(encoding='utf-8')

    patched_text = sdf_text
    changed = False

    if abs(real_time_factor - 1.0) > 1e-9:
        patched_text, count = re.subn(
            r'(<real_time_factor>\s*)([^<]+)(\s*</real_time_factor>)',
            rf'\g<1>{real_time_factor}\g<3>',
            patched_text,
            count=1,
        )
        if count != 1:
            raise RuntimeError(
                f"Could not patch real_time_factor in world file: {world_sdf_path}"
            )
        changed = True

    if not changed:
        return str(world_sdf_path)

    tmp_dir = Path('/tmp/halmstad_ws/generated_worlds')
    tmp_dir.mkdir(parents=True, exist_ok=True)
    source_hash = hashlib.sha1(str(world_sdf_path.resolve()).encode('utf-8')).hexdigest()[:10]
    rtf_token = f"{real_time_factor:g}".replace('.', 'p')
    tmp_path = tmp_dir / f"{Path(world_name).stem}_{source_hash}_rtf_{rtf_token}.sdf"
    if not tmp_path.is_file() or tmp_path.read_text(encoding='utf-8') != patched_text:
        tmp_path.write_text(patched_text, encoding='utf-8')
    return str(tmp_path)


def _parse_float_arg(context, name: str) -> float:
    raw_value = LaunchConfiguration(name).perform(context).strip()
    try:
        return float(raw_value)
    except ValueError as exc:
        raise RuntimeError(f"Invalid {name} value: '{raw_value}'") from exc


def _prepare_gui_config_path(context, base_gui_config_path: str) -> str:
    if LaunchConfiguration('view_follow_spawn').perform(context) != 'true':
        return base_gui_config_path

    spawn_x = _parse_float_arg(context, 'x')
    spawn_y = _parse_float_arg(context, 'y')
    spawn_z = _parse_float_arg(context, 'z')
    spawn_yaw = _parse_float_arg(context, 'yaw')
    view_distance = _parse_float_arg(context, 'view_distance')
    view_height = _parse_float_arg(context, 'view_height')
    view_pitch = _parse_float_arg(context, 'view_pitch')
    view_yaw_offset = _parse_float_arg(context, 'view_yaw_offset')

    camera_yaw = spawn_yaw + view_yaw_offset
    camera_x = spawn_x - view_distance * math.cos(camera_yaw)
    camera_y = spawn_y - view_distance * math.sin(camera_yaw)
    camera_z = spawn_z + view_height
    camera_pose = f"{camera_x:.6f} {camera_y:.6f} {camera_z:.6f} 0 {view_pitch:.6f} {camera_yaw:.6f}"

    gui_text = Path(base_gui_config_path).read_text(encoding='utf-8')
    patched_text, count = re.subn(
        r'(<camera_pose>\s*)([^<]+)(\s*</camera_pose>)',
        rf'\g<1>{camera_pose}\g<3>',
        gui_text,
        count=1,
    )
    if count != 1:
        raise RuntimeError(f"Could not patch camera_pose in GUI config: {base_gui_config_path}")

    tmp_dir = Path('/tmp/halmstad_ws/generated_gui_configs')
    tmp_dir.mkdir(parents=True, exist_ok=True)
    source_hash = hashlib.sha1(str(Path(base_gui_config_path).resolve()).encode('utf-8')).hexdigest()[:10]
    pose_hash = hashlib.sha1(camera_pose.encode('utf-8')).hexdigest()[:10]
    tmp_path = tmp_dir / f"gui_{source_hash}_{pose_hash}.config"
    if not tmp_path.is_file() or tmp_path.read_text(encoding='utf-8') != patched_text:
        tmp_path.write_text(patched_text, encoding='utf-8')
    return str(tmp_path)


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
