#import logging
#logging.root.setLevel(logging.DEBUG)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, PythonExpression


def _gazebo_world_name(world_sub):
    return PythonExpression([
        "'office_construction' if '",
        world_sub,
        "' == 'construction' else '",
        world_sub,
        "'",
    ])


world_arg = DeclareLaunchArgument('world', default_value='warehouse',
                      description='Gazebo World')
uav_mode_arg = DeclareLaunchArgument('uav_mode', default_value='teleport',
                      description='UAV mode: teleport (deterministic) or physics')
camera_mode_arg = DeclareLaunchArgument(
    'camera_mode',
    default_value='detached',
    choices=['integrated', 'detached'],
    description='Camera mode: integrated (camera link in UAV model) or detached (separate camera model)'
)
detached_spawn_delay_arg = DeclareLaunchArgument(
    'detached_spawn_delay_s',
    default_value='0.25',
    description='Delay between UAV body and detached camera spawns to match GUI spawner behavior',
)

def generate_launch_description():
    gz_world = _gazebo_world_name(LaunchConfiguration('world'))
    with_camera_for_uav = PythonExpression([
        "'false' if '", LaunchConfiguration('camera_mode'), "' == 'detached' else 'true'"
    ])
    use_detached_camera = PythonExpression([
        "'",
        LaunchConfiguration('camera_mode'),
        "' == 'detached'"
    ])

    dji0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji0",
                          "type": "m100",
                          "uav_mode": LaunchConfiguration('uav_mode'),
                          "with_camera": with_camera_for_uav,
                          "camera_name": "camera0",
                          "x": '0.0',
                          "y": '0.0',
                          "z": '2.27',
                          "world": LaunchConfiguration('world')
                          }.items(),
    )


    dji1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji1",
                          "type": "m100",
                          "uav_mode": LaunchConfiguration('uav_mode'),
                          "with_camera": with_camera_for_uav,
                          "camera_name": "camera0",
                          "x": '0.0',
                          "y": '0.0',
                          "z": '3.27',
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )

    dji2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji2",
                          "type": "m100",
                          "uav_mode": LaunchConfiguration('uav_mode'),
                          "with_camera": with_camera_for_uav,
                          "camera_name": "camera0",
                          "x": '0.0',
                          "y": '0.0',
                          "z": '4.27',
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )

    dji0_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={"name": "dji0",
                          "type": "m100",
                          "camera_name": "camera0",
                          "bridge_camera": "false",
                          "x": '0.0',
                          "y": '0.0',
                          "z": '2.27',
                          "world": LaunchConfiguration('world')}.items(),
        condition=IfCondition(use_detached_camera),
    )

    dji1_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={"name": "dji1",
                          "type": "m100",
                          "camera_name": "camera0",
                          "bridge_camera": "false",
                          "x": '0.0',
                          "y": '0.0',
                          "z": '3.27',
                          "world": LaunchConfiguration('world')}.items(),
        condition=IfCondition(use_detached_camera),
    )

    dji2_camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={"name": "dji2",
                          "type": "m100",
                          "camera_name": "camera0",
                          "bridge_camera": "false",
                          "x": '0.0',
                          "y": '0.0',
                          "z": '4.27',
                          "world": LaunchConfiguration('world')}.items(),
        condition=IfCondition(use_detached_camera),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
#        arguments= ['/world/orchard/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        arguments=[
            ['/world/', gz_world, '/set_pose@ros_gz_interfaces/srv/SetEntityPose']
        ],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/dji0/camera0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji0/camera0/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/dji1/camera0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji1/camera0/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/dji2/camera0/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji2/camera0/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
        ],
        output='screen',
    )
    
    
    dji0_camera_delayed = TimerAction(
        period=LaunchConfiguration('detached_spawn_delay_s'),
        actions=[dji0_camera],
    )
    dji1_delayed = TimerAction(
        period=PythonExpression([str(2.0), " * ", LaunchConfiguration('detached_spawn_delay_s')]),
        actions=[dji1],
    )
    dji1_camera_delayed = TimerAction(
        period=PythonExpression([str(3.0), " * ", LaunchConfiguration('detached_spawn_delay_s')]),
        actions=[dji1_camera],
    )
    dji2_delayed = TimerAction(
        period=PythonExpression([str(4.0), " * ", LaunchConfiguration('detached_spawn_delay_s')]),
        actions=[dji2],
    )
    dji2_camera_delayed = TimerAction(
        period=PythonExpression([str(5.0), " * ", LaunchConfiguration('detached_spawn_delay_s')]),
        actions=[dji2_camera],
    )
    camera_bridge_delayed = TimerAction(
        period=PythonExpression([str(6.0), " * ", LaunchConfiguration('detached_spawn_delay_s')]),
        actions=[camera_bridge],
    )

    return LaunchDescription([
        world_arg,
        uav_mode_arg,
        camera_mode_arg,
        detached_spawn_delay_arg,
        bridge,
        dji0,
        dji0_camera_delayed,
        dji1_delayed,
        dji1_camera_delayed,
        dji2_delayed,
        dji2_camera_delayed,
        camera_bridge_delayed,
    ])
