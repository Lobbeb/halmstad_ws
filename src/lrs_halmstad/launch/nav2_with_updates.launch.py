import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace, SetRemap

from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument(
        "setup_path",
        default_value="/etc/clearpath/",
        description="Clearpath setup path",
    ),
    DeclareLaunchArgument(
        "scan_topic",
        default_value="",
        description="Override the default laserscan topic",
    ),
    DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="",
        description="Optional pointcloud topic to convert into a LaserScan before Nav2",
    ),
    DeclareLaunchArgument(
        "use_pointcloud_to_laserscan",
        default_value="true",
        choices=["true", "false"],
        description="Convert an incoming point cloud into a 2D LaserScan before Nav2",
    ),
    DeclareLaunchArgument("pc2ls_min_height", default_value="-0.4"),
    DeclareLaunchArgument("pc2ls_max_height", default_value="0.4"),
    DeclareLaunchArgument("pc2ls_angle_min", default_value="-3.141592653589793"),
    DeclareLaunchArgument("pc2ls_angle_max", default_value="3.141592653589793"),
    DeclareLaunchArgument("pc2ls_angle_increment", default_value="0.006981317007977318"),
    DeclareLaunchArgument("pc2ls_scan_time", default_value="0.05"),
    DeclareLaunchArgument("pc2ls_range_min", default_value="0.5"),
    DeclareLaunchArgument("pc2ls_range_max", default_value="25.0"),
    DeclareLaunchArgument("pc2ls_queue_size", default_value="40"),
    DeclareLaunchArgument("pc2ls_target_frame", default_value=""),
    DeclareLaunchArgument("pc2ls_transform_tolerance", default_value="0.2"),
    DeclareLaunchArgument("pc2ls_use_inf", default_value="true", choices=["true", "false"]),
    DeclareLaunchArgument(
        "use_scan_relay",
        default_value="false",
        choices=["true", "false"],
        description="Relay the latest scan at a slower, stable rate for Nav2 consumers.",
    ),
    DeclareLaunchArgument("scan_relay_hz", default_value="8.0"),
    DeclareLaunchArgument("scan_relay_max_age_s", default_value="0.5"),
    DeclareLaunchArgument("scan_relay_restamp", default_value="true", choices=["true", "false"]),
    DeclareLaunchArgument("scan_relay_stamp_offset_s", default_value="0.0"),
    DeclareLaunchArgument("scan_relay_start_delay_s", default_value="0.0"),
    DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Override the default Nav2 params YAML",
    ),
    DeclareLaunchArgument(
        "start_teleop_base",
        default_value="true",
        choices=["true", "false"],
        description="Start Clearpath teleop_base so twist_mux forwards Nav2 cmd_vel to platform/cmd_vel.",
    ),
    DeclareLaunchArgument(
        "start_collision_monitor",
        default_value="false",
        choices=["true", "false"],
        description="Start Nav2 collision_monitor. Disabled by default for lighter simulation runs.",
    ),
]


def _find_call_start(lines, marker_index):
    for index in range(marker_index, -1, -1):
        stripped = lines[index].strip()
        if stripped in ("Node(", "ComposableNode("):
            return index
    raise RuntimeError("Could not find opennav_docking launch call start")


def _find_call_end(lines, start_index):
    depth = 0
    for index in range(start_index, len(lines)):
        depth += lines[index].count("(")
        depth -= lines[index].count(")")
        if depth <= 0 and index > start_index:
            return index + 1
    raise RuntimeError("Could not find opennav_docking launch call end")


def _remove_launch_call_for_package(lines, package_name):
    while True:
        marker_index = next(
            (
                index
                for index, line in enumerate(lines)
                if f"package='{package_name}'" in line
            ),
            None,
        )
        if marker_index is None:
            return lines
        start_index = _find_call_start(lines, marker_index)
        end_index = _find_call_end(lines, start_index)
        del lines[start_index:end_index]


def _remap_velocity_smoother_output_to_cmd_vel(lines):
    marker = "package='nav2_velocity_smoother'"
    marker_index = next(
        (index for index, line in enumerate(lines) if marker in line),
        None,
    )
    while marker_index is not None:
        start_index = _find_call_start(lines, marker_index)
        end_index = _find_call_end(lines, start_index)
        for index in range(start_index, end_index):
            if "[('cmd_vel', 'cmd_vel_nav')]" in lines[index]:
                lines[index] = lines[index].replace(
                    "[('cmd_vel', 'cmd_vel_nav')]",
                    "[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]",
                )
                break
        marker_index = next(
            (
                index
                for index, line in enumerate(lines[end_index:], start=end_index)
                if marker in line
            ),
            None,
        )
    return lines


def _navigation_launch_without_docking(pkg_nav2_bringup, include_collision_monitor=False):
    source = Path(pkg_nav2_bringup) / "launch" / "navigation_launch.py"
    destination = Path("/tmp/halmstad_ws/navigation_launch_no_docking.py")
    lines = source.read_text(encoding="utf-8").splitlines(keepends=True)

    lines = [line for line in lines if line.strip() != "'docking_server',"]
    if not include_collision_monitor:
        lines = [line for line in lines if line.strip() != "'collision_monitor',"]

    _remove_launch_call_for_package(lines, "opennav_docking")
    if not include_collision_monitor:
        _remove_launch_call_for_package(lines, "nav2_collision_monitor")
        _remap_velocity_smoother_output_to_cmd_vel(lines)

    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("".join(lines), encoding="utf-8")
    return str(destination)


def launch_setup(context, *args, **kwargs):
    pkg_clearpath_nav2_demos = get_package_share_directory("clearpath_nav2_demos")
    pkg_clearpath_control = get_package_share_directory("clearpath_control")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    setup_path = LaunchConfiguration("setup_path")
    scan_topic = LaunchConfiguration("scan_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    use_pointcloud_to_laserscan = LaunchConfiguration("use_pointcloud_to_laserscan")
    pc2ls_min_height = LaunchConfiguration("pc2ls_min_height")
    pc2ls_max_height = LaunchConfiguration("pc2ls_max_height")
    pc2ls_angle_min = LaunchConfiguration("pc2ls_angle_min")
    pc2ls_angle_max = LaunchConfiguration("pc2ls_angle_max")
    pc2ls_angle_increment = LaunchConfiguration("pc2ls_angle_increment")
    pc2ls_scan_time = LaunchConfiguration("pc2ls_scan_time")
    pc2ls_range_min = LaunchConfiguration("pc2ls_range_min")
    pc2ls_range_max = LaunchConfiguration("pc2ls_range_max")
    pc2ls_queue_size = LaunchConfiguration("pc2ls_queue_size")
    pc2ls_target_frame = LaunchConfiguration("pc2ls_target_frame")
    pc2ls_transform_tolerance = LaunchConfiguration("pc2ls_transform_tolerance")
    pc2ls_use_inf = LaunchConfiguration("pc2ls_use_inf")
    use_scan_relay = LaunchConfiguration("use_scan_relay")
    scan_relay_hz = LaunchConfiguration("scan_relay_hz")
    scan_relay_max_age_s = LaunchConfiguration("scan_relay_max_age_s")
    scan_relay_restamp = LaunchConfiguration("scan_relay_restamp")
    scan_relay_stamp_offset_s = LaunchConfiguration("scan_relay_stamp_offset_s")
    scan_relay_start_delay_s = LaunchConfiguration("scan_relay_start_delay_s")
    params_file = LaunchConfiguration("params_file")
    start_teleop_base = LaunchConfiguration("start_teleop_base")
    start_collision_monitor = LaunchConfiguration("start_collision_monitor")

    config = read_yaml(os.path.join(setup_path.perform(context), "robot.yaml"))
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    eval_scan_topic = scan_topic.perform(context)
    if len(eval_scan_topic) == 0:
        eval_scan_topic = f"/{namespace}/sensors/lidar2d_0/scan"
    eval_pointcloud_topic = pointcloud_topic.perform(context)
    if len(eval_pointcloud_topic) == 0 and eval_scan_topic in (
        f"/{namespace}/sensors/lidar3d_0/scan_from_points",
        f"/{namespace}/sensors/lidar3d_0/scan_from_points_relay",
    ):
        eval_pointcloud_topic = f"/{namespace}/sensors/lidar3d_0/points"

    eval_params_file = params_file.perform(context)
    if len(eval_params_file) == 0:
        eval_params_file = os.path.join(
            pkg_clearpath_nav2_demos, "config", platform_model, "nav2.yaml"
        )

    launch_nav2 = _navigation_launch_without_docking(
        pkg_nav2_bringup,
        include_collision_monitor=start_collision_monitor.perform(context) == "true",
    )
    launch_teleop_base = PathJoinSubstitution([pkg_clearpath_control, "launch", "teleop_base.launch.py"])

    converter_node = None
    if use_pointcloud_to_laserscan.perform(context) == "true" and len(eval_pointcloud_topic) != 0:
        converter_node = Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            name="pointcloud_to_laserscan",
            output="screen",
            remappings=[
                ("cloud_in", eval_pointcloud_topic),
                ("scan", eval_scan_topic),
            ],
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "min_height": pc2ls_min_height,
                    "max_height": pc2ls_max_height,
                    "angle_min": pc2ls_angle_min,
                    "angle_max": pc2ls_angle_max,
                    "angle_increment": pc2ls_angle_increment,
                    "scan_time": pc2ls_scan_time,
                    "range_min": pc2ls_range_min,
                    "range_max": pc2ls_range_max,
                    "queue_size": pc2ls_queue_size,
                    "target_frame": pc2ls_target_frame,
                    "transform_tolerance": pc2ls_transform_tolerance,
                    "use_inf": pc2ls_use_inf,
                }
            ],
        )

    relay_node = None
    if use_scan_relay.perform(context) == "true":
        relay_input_topic = eval_scan_topic
        eval_scan_topic = f"{relay_input_topic}_relay"
        relay_node = Node(
            package="lrs_halmstad",
            executable="latest_scan_relay",
            name="latest_scan_relay",
            output="screen",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "input_topic": relay_input_topic,
                    "output_topic": eval_scan_topic,
                    "publish_hz": scan_relay_hz,
                    "max_age_s": scan_relay_max_age_s,
                    "restamp": scan_relay_restamp,
                    "stamp_offset_s": scan_relay_stamp_offset_s,
                    "startup_hold_s": scan_relay_start_delay_s,
                }
            ],
        )

    rewritten_parameters = RewrittenYaml(
        source_file=eval_params_file,
        param_rewrites={
            "scan.topic": eval_scan_topic,
            "pointcloud.topic": eval_pointcloud_topic,
        },
        convert_types=True,
    )
    group_actions = [
        PushRosNamespace(namespace),
        SetRemap("/" + namespace + "/odom", "/" + namespace + "/platform/odom"),
    ]
    if converter_node is not None:
        group_actions.append(converter_node)
    if relay_node is not None:
        group_actions.append(relay_node)
    group_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_nav2),
            launch_arguments=[
                ("use_sim_time", use_sim_time),
                ("params_file", rewritten_parameters),
                ("use_composition", "False"),
                ("namespace", namespace),
            ],
        )
    )
    group_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_teleop_base),
            launch_arguments=[
                ("setup_path", setup_path),
                ("use_sim_time", use_sim_time),
            ],
            condition=IfCondition(start_teleop_base),
        )
    )

    nav2 = GroupAction(group_actions)

    return [nav2]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
