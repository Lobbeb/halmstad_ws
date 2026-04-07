import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace

from nav2_common.launch import RewrittenYaml


ARGUMENTS = [
    DeclareLaunchArgument("use_sim_time", default_value="false", choices=["true", "false"]),
    DeclareLaunchArgument("setup_path", default_value="/etc/clearpath/"),
    DeclareLaunchArgument("scan_topic", default_value=""),
    DeclareLaunchArgument("pointcloud_topic", default_value=""),
    DeclareLaunchArgument(
        "use_pointcloud_to_laserscan",
        default_value="false",
        choices=["true", "false"],
    ),
    DeclareLaunchArgument("pc2ls_min_height", default_value="-0.35"),
    DeclareLaunchArgument("pc2ls_max_height", default_value="0.05"),
    DeclareLaunchArgument("pc2ls_angle_min", default_value="-3.141592653589793"),
    DeclareLaunchArgument("pc2ls_angle_max", default_value="3.141592653589793"),
    DeclareLaunchArgument("pc2ls_angle_increment", default_value="0.006981317007977318"),
    DeclareLaunchArgument("pc2ls_scan_time", default_value="0.05"),
    DeclareLaunchArgument("pc2ls_range_min", default_value="0.5"),
    DeclareLaunchArgument("pc2ls_range_max", default_value="30.0"),
    DeclareLaunchArgument("pc2ls_queue_size", default_value="1"),
    DeclareLaunchArgument("pc2ls_target_frame", default_value=""),
    DeclareLaunchArgument("pc2ls_transform_tolerance", default_value="0.05"),
    DeclareLaunchArgument("pc2ls_use_inf", default_value="true", choices=["true", "false"]),
]


def launch_setup(context, *args, **kwargs):
    pkg_clearpath_nav2_demos = get_package_share_directory("clearpath_nav2_demos")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    setup_path = LaunchConfiguration("setup_path")
    map_path = LaunchConfiguration("map")
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

    config = read_yaml(os.path.join(setup_path.perform(context), "robot.yaml"))
    clearpath_config = ClearpathConfig(config)

    namespace = clearpath_config.system.namespace
    platform_model = clearpath_config.platform.get_platform_model()

    eval_scan_topic = scan_topic.perform(context)
    if len(eval_scan_topic) == 0:
        eval_scan_topic = f"/{namespace}/sensors/lidar2d_0/scan"
    eval_pointcloud_topic = pointcloud_topic.perform(context)
    if len(eval_pointcloud_topic) == 0 and eval_scan_topic == f"/{namespace}/sensors/lidar3d_0/scan_from_points":
        eval_pointcloud_topic = f"/{namespace}/sensors/lidar3d_0/points"

    file_parameters = PathJoinSubstitution(
        [pkg_clearpath_nav2_demos, "config", platform_model, "localization.yaml"]
    )

    rewritten_parameters = RewrittenYaml(
        source_file=file_parameters,
        param_rewrites={"scan_topic": eval_scan_topic},
        convert_types=True,
    )

    launch_localization = PathJoinSubstitution(
        [pkg_nav2_bringup, "launch", "localization_launch.py"]
    )

    group_actions = [PushRosNamespace(namespace)]

    if use_pointcloud_to_laserscan.perform(context) == "true" and len(eval_pointcloud_topic) != 0:
        group_actions.append(
            Node(
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
        )

    group_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(launch_localization),
            launch_arguments=[
                ("namespace", namespace),
                ("map", map_path),
                ("use_sim_time", use_sim_time),
                ("params_file", rewritten_parameters),
            ],
        )
    )

    return [GroupAction(group_actions)]


def generate_launch_description():
    pkg_clearpath_nav2_demos = get_package_share_directory("clearpath_nav2_demos")

    map_arg = DeclareLaunchArgument(
        "map",
        default_value=PathJoinSubstitution([pkg_clearpath_nav2_demos, "maps", "warehouse.yaml"]),
        description="Full path to map yaml file to load",
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(map_arg)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
