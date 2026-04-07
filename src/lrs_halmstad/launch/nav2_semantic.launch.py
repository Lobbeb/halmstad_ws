import os

from ament_index_python.packages import get_package_share_directory

from clearpath_config.clearpath_config import ClearpathConfig
from clearpath_config.common.utils.yaml import read_yaml

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node, PushRosNamespace, SetRemap


ARGUMENTS = [
    DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        choices=["true", "false"],
        description="Use sim time",
    ),
    DeclareLaunchArgument(
        "setup_path",
        default_value="/etc/clearpath/",
        description="Clearpath setup path",
    ),
    DeclareLaunchArgument(
        "params_file",
        default_value="",
        description="Override the semantic Nav2 params YAML",
    ),
    DeclareLaunchArgument(
        "autostart",
        default_value="true",
        choices=["true", "false"],
        description="Automatically startup Nav2",
    ),
    DeclareLaunchArgument(
        "segmentation_input_topic",
        default_value="sensors/camera_0/color/image",
        description="RGB image topic consumed by the semantic segmentation node",
    ),
    DeclareLaunchArgument(
        "segmentation_publish_overlay",
        default_value="true",
        choices=["true", "false"],
        description="Publish the color overlay topic for RViz / debugging",
    ),
]


def launch_setup(context, *args, **kwargs):
    pkg_lrs_halmstad = get_package_share_directory("lrs_halmstad")
    pkg_nav2_bringup = get_package_share_directory("nav2_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    setup_path = LaunchConfiguration("setup_path")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    segmentation_input_topic = LaunchConfiguration("segmentation_input_topic")
    segmentation_publish_overlay = LaunchConfiguration("segmentation_publish_overlay")

    config = read_yaml(os.path.join(setup_path.perform(context), "robot.yaml"))
    clearpath_config = ClearpathConfig(config)
    namespace = clearpath_config.system.namespace

    eval_params_file = params_file.perform(context)
    if len(eval_params_file) == 0:
        eval_params_file = os.path.join(pkg_lrs_halmstad, "config", "nav2_semantic_params.yaml")

    launch_nav2 = PathJoinSubstitution([pkg_nav2_bringup, "launch", "navigation_launch.py"])

    semantic_bringup = GroupAction(
        [
            PushRosNamespace(namespace),
            SetRemap("/" + namespace + "/odom", "/" + namespace + "/platform/odom"),
            SetRemap("/tf", "/" + namespace + "/tf"),
            SetRemap("/tf_static", "/" + namespace + "/tf_static"),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="map_to_odom_static",
                output="screen",
                arguments=[
                    "--x", "0",
                    "--y", "0",
                    "--z", "0",
                    "--roll", "0",
                    "--pitch", "0",
                    "--yaw", "0",
                    "--frame-id", "map",
                    "--child-frame-id", "odom",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
            ),
            Node(
                package="semantic_segmentation_node",
                executable="segmentation_node",
                name="semantic_segmentation_node",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": use_sim_time,
                        "input_topic": segmentation_input_topic,
                        "mask_topic": "segmentation/mask",
                        "confidence_topic": "segmentation/confidence",
                        "label_info_topic": "segmentation/label_info",
                        "overlay_topic": "segmentation/overlay",
                        "publish_overlay": segmentation_publish_overlay,
                    }
                ],
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(launch_nav2),
                launch_arguments=[
                    ("use_sim_time", use_sim_time),
                    ("params_file", eval_params_file),
                    ("use_composition", "False"),
                    ("namespace", namespace),
                    ("autostart", autostart),
                ],
            ),
        ]
    )

    return [semantic_bringup]


def generate_launch_description():
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(OpaqueFunction(function=launch_setup))
    return ld
