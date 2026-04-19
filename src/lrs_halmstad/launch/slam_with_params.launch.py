import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
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
        "autostart",
        default_value="true",
        choices=["true", "false"],
        description="Automatically startup the slamtoolbox. Ignored when use_lifecycle_manager is true.",
    ),
    DeclareLaunchArgument(
        "use_lifecycle_manager",
        default_value="false",
        choices=["true", "false"],
        description="Enable bond connection during node activation",
    ),
    DeclareLaunchArgument(
        "sync",
        default_value="true",
        choices=["true", "false"],
        description="Use synchronous SLAM",
    ),
    DeclareLaunchArgument(
        "scan_topic",
        default_value="",
        description="Override the default laserscan topic",
    ),
    DeclareLaunchArgument(
        "pointcloud_topic",
        default_value="",
        description="Optional pointcloud topic to convert into a LaserScan before SLAM",
    ),
    DeclareLaunchArgument(
        "slam_params_file",
        default_value="",
        description="Override the default SLAM params YAML",
    ),
    DeclareLaunchArgument(
        "use_pointcloud_to_laserscan",
        default_value="false",
        choices=["true", "false"],
        description="Convert an incoming point cloud into a 2D LaserScan before SLAM",
    ),
    DeclareLaunchArgument("pc2ls_min_height", default_value="-0.35"),
    DeclareLaunchArgument("pc2ls_max_height", default_value="0.05"),
    DeclareLaunchArgument("pc2ls_angle_min", default_value="-3.141592653589793"),
    DeclareLaunchArgument("pc2ls_angle_max", default_value="3.141592653589793"),
    DeclareLaunchArgument("pc2ls_angle_increment", default_value="0.006981317007977318"),
    DeclareLaunchArgument("pc2ls_scan_time", default_value="0.05"),
    DeclareLaunchArgument("pc2ls_range_min", default_value="0.5"),
    DeclareLaunchArgument("pc2ls_range_max", default_value="25.0"),
    DeclareLaunchArgument("pc2ls_queue_size", default_value="20"),
    DeclareLaunchArgument("pc2ls_target_frame", default_value="odom"),
    DeclareLaunchArgument("pc2ls_transform_tolerance", default_value="0.2"),
    DeclareLaunchArgument("pc2ls_use_inf", default_value="true", choices=["true", "false"]),
    DeclareLaunchArgument(
        "slam_max_laser_range",
        default_value="25.0",
        description="Maximum scan range used internally by slam_toolbox rasterization",
    ),
    DeclareLaunchArgument(
        "use_scan_relay",
        default_value="false",
        choices=["true", "false"],
        description="Relay the latest scan to a lower-rate freshest-only topic before SLAM",
    ),
    DeclareLaunchArgument(
        "scan_relay_hz",
        default_value="2.0",
        description="Publish rate for the optional latest-scan relay",
    ),
    DeclareLaunchArgument(
        "scan_relay_max_age_s",
        default_value="1.0",
        description="Maximum age of a relayed scan before it is dropped",
    ),
    DeclareLaunchArgument(
        "scan_relay_restamp",
        default_value="true",
        choices=["true", "false"],
        description="Restamp relayed scans with the current ROS clock time",
    ),
    DeclareLaunchArgument(
        "scan_relay_start_delay_s",
        default_value="0.0",
        description="Hold relay output for this many seconds after startup while keeping only the freshest scan",
    ),
]


def launch_setup(context, *args, **kwargs):
    pkg_clearpath_nav2_demos = get_package_share_directory("clearpath_nav2_demos")
    pkg_slam_toolbox = get_package_share_directory("slam_toolbox")

    use_sim_time = LaunchConfiguration("use_sim_time")
    setup_path = LaunchConfiguration("setup_path")
    autostart = LaunchConfiguration("autostart")
    use_lifecycle_manager = LaunchConfiguration("use_lifecycle_manager")
    sync = LaunchConfiguration("sync")
    scan_topic = LaunchConfiguration("scan_topic")
    pointcloud_topic = LaunchConfiguration("pointcloud_topic")
    slam_params_file = LaunchConfiguration("slam_params_file")
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
    slam_max_laser_range = LaunchConfiguration("slam_max_laser_range")
    use_scan_relay = LaunchConfiguration("use_scan_relay")
    scan_relay_hz = LaunchConfiguration("scan_relay_hz")
    scan_relay_max_age_s = LaunchConfiguration("scan_relay_max_age_s")
    scan_relay_restamp = LaunchConfiguration("scan_relay_restamp")
    scan_relay_start_delay_s = LaunchConfiguration("scan_relay_start_delay_s")

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
                    "startup_hold_s": scan_relay_start_delay_s,
                }
            ],
        )

    eval_slam_params = slam_params_file.perform(context)
    if len(eval_slam_params) == 0:
        eval_slam_params = os.path.join(
            pkg_clearpath_nav2_demos, "config", platform_model, "slam.yaml"
        )

    rewritten_parameters = RewrittenYaml(
        source_file=eval_slam_params,
        root_key=namespace,
        param_rewrites={
            "map_name": "/" + namespace + "/map",
            "scan_topic": eval_scan_topic,
            "max_laser_range": slam_max_laser_range,
        },
        convert_types=True,
    )

    launch_slam_sync = PathJoinSubstitution([pkg_slam_toolbox, "launch", "online_sync_launch.py"])
    launch_slam_async = PathJoinSubstitution([pkg_slam_toolbox, "launch", "online_async_launch.py"])

    group_actions = [
        PushRosNamespace(namespace),
        SetRemap("/tf", "/" + namespace + "/tf"),
        SetRemap("/tf_static", "/" + namespace + "/tf_static"),
    ]

    if relay_node is not None:
        group_actions.append(relay_node)

    if converter_node is not None:
        group_actions.append(converter_node)

    group_actions.extend(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_slam_sync),
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                    ("autostart", autostart),
                    ("use_lifecycle_manager", use_lifecycle_manager),
                    ("slam_params_file", rewritten_parameters),
                ],
                condition=IfCondition(sync),
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_slam_async),
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                    ("autostart", autostart),
                    ("use_lifecycle_manager", use_lifecycle_manager),
                    ("slam_params_file", rewritten_parameters),
                ],
                condition=UnlessCondition(sync),
            ),
        ]
    )

    slam = GroupAction(group_actions)

    return [slam]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
