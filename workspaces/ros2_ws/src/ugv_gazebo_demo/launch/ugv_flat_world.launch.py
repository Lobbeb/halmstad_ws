from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable, SetLaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
import tempfile


def _world_name_from_arg(world_arg: str) -> str:
    value = (world_arg or "").strip()
    if not value:
        return "default"
    candidate = os.path.basename(value)
    if candidate.endswith(".sdf") or candidate.endswith(".world"):
        candidate = os.path.splitext(candidate)[0]
    return candidate or "default"


def _create_bridge_node(context):
    world_name = _world_name_from_arg(LaunchConfiguration("world").perform(context))
    bebop_img = f"/world/{world_name}/model/bebop1/model/parrot_bebop_2/link/body/sensor/rgb_camera_sensor/image"
    bebop_info = f"/world/{world_name}/model/bebop1/model/parrot_bebop_2/link/body/sensor/rgb_camera_sensor/camera_info"
    ugv_scan = f"/world/{world_name}/model/x2_ugv/link/base/sensor/lidar/scan"
    ugv_scan_nested = f"/world/{world_name}/model/x2_ugv/model/X2/link/base/sensor/lidar/scan"

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        parameters=[{"expand_gz_topic_names": True}],
        arguments=[
            "/model/x2_ugv/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/model/bebop1/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry",
            "/x2_ugv/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/bebop1/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist",
            "/bebop1/enable@std_msgs/msg/Bool]gz.msgs.Boolean",
            f"{bebop_img}@sensor_msgs/msg/Image[gz.msgs.Image",
            f"{bebop_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo",
            "/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{ugv_scan}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
            f"{ugv_scan_nested}@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan",
        ],
        remappings=[
            ("/model/x2_ugv/odometry", "/x2_ugv/odom"),
            ("/model/bebop1/odometry", "/bebop1/odom"),
            (bebop_img, "/bebop1/camera/image"),
            (bebop_info, "/bebop1/camera/camera_info"),
            ("/scan", "/x2_ugv/scan"),
            (ugv_scan, "/x2_ugv/scan"),
            (ugv_scan_nested, "/x2_ugv/scan"),
        ],
        output="screen",
    )
    return [bridge]


def _prepare_runtime_world(context):
    world_path = LaunchConfiguration("world").perform(context)
    spawn_bebop = LaunchConfiguration("spawn_bebop").perform(context).strip().lower() in ("1", "true", "yes", "on")
    bebop_spawn_x = LaunchConfiguration("bebop_spawn_x").perform(context)
    bebop_spawn_y = LaunchConfiguration("bebop_spawn_y").perform(context)
    bebop_spawn_z = LaunchConfiguration("bebop_spawn_z").perform(context)
    pkg_share = get_package_share_directory("ugv_gazebo_demo")
    ugv_model_sdf = os.path.join(pkg_share, "models", "ugv_x2_with_odom", "model.sdf")
    bebop_model_sdf = os.path.join(pkg_share, "models", "bebop_with_control", "model.sdf")

    # Read with utf-8-sig to handle BOM in some world files (e.g., default.sdf).
    with open(world_path, "r", encoding="utf-8-sig") as f:
        world_sdf = f.read()

    if "</world>" not in world_sdf:
        raise RuntimeError(f"Invalid world SDF (missing </world>): {world_path}")

    includes = []
    if 'name="x2_ugv"' not in world_sdf:
        includes.append(
            """    <include>
      <uri>file://{ugv_model_sdf}</uri>
      <name>x2_ugv</name>
      <pose>0 0 0.1 0 0 0</pose>
    </include>""".format(ugv_model_sdf=ugv_model_sdf)
        )

    if spawn_bebop and 'name="bebop1"' not in world_sdf:
        includes.append(
            f"""    <include>
      <uri>file://{bebop_model_sdf}</uri>
      <name>bebop1</name>
      <pose>{bebop_spawn_x} {bebop_spawn_y} {bebop_spawn_z} 0 0 0</pose>
    </include>"""
        )

    if not includes:
        return [SetLaunchConfiguration("world_runtime", world_path)]

    patched_sdf = world_sdf.replace("</world>", "\n" + "\n".join(includes) + "\n  </world>", 1)

    world_base = os.path.splitext(os.path.basename(world_path))[0]
    runtime_path = os.path.join(tempfile.gettempdir(), f"ugv_gazebo_demo_{world_base}_runtime.sdf")
    with open(runtime_path, "w", encoding="utf-8") as f:
        f.write(patched_sdf)

    return [SetLaunchConfiguration("world_runtime", runtime_path)]


def generate_launch_description():
    pkg_share = get_package_share_directory("ugv_gazebo_demo")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")

    default_world = os.path.join(os.path.expanduser("~"), "ros2_ws", "worlds", "default.sdf")
    world_runtime = LaunchConfiguration("world_runtime")

    extra_model_paths = [
        os.path.join(os.path.expanduser("~"), "ws_bebop", "src", "bebop_ros", "bebop_gz", "models"),
        os.path.join(os.path.expanduser("~"), "ros2_ws", "models"),
        os.path.join(pkg_share, "models"),
    ]
    extra_model_path = os.pathsep.join([path for path in extra_model_paths if os.path.isdir(path)])

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": ["-r ", world_runtime],
            "on_exit_shutdown": "true",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "world",
            default_value=default_world,
            description="Absolute path to the SDF/world file to load in Gazebo",
        ),
        DeclareLaunchArgument(
            "world_runtime",
            default_value=default_world,
            description="Internal: runtime world path with embedded robot includes for reset-safe respawn.",
        ),
        DeclareLaunchArgument(
            "spawn_bebop",
            default_value="false",
            description="Spawn one bebop1 drone with multicopter control plugins.",
        ),
        DeclareLaunchArgument(
            "bebop_spawn_x",
            default_value="-10.0",
            description="Initial bebop1 spawn x in world frame (meters).",
        ),
        DeclareLaunchArgument(
            "bebop_spawn_y",
            default_value="0.0",
            description="Initial bebop1 spawn y in world frame (meters).",
        ),
        DeclareLaunchArgument(
            "bebop_spawn_z",
            default_value="0.15",
            description="Initial bebop1 spawn z in world frame (meters).",
        ),
        DeclareLaunchArgument(
            "show_bebop_camera",
            default_value="false",
            description="Deprecated compatibility arg (Bebop camera now shown in Gazebo ImageDisplay panel).",
        ),
        SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", extra_model_path),
        OpaqueFunction(function=_prepare_runtime_world),
        gazebo,
        OpaqueFunction(function=_create_bridge_node),
    ])
