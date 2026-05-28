#import logging
#logging.root.setLevel(logging.DEBUG)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
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
    default_value='integrated',
    description='Integrated camera mode only; detached camera mode has been removed from simulation'
)
camera_update_rate_arg = DeclareLaunchArgument(
    'camera_update_rate',
    default_value='20',
    description='UAV camera sensor update rate in Hz'
)
dji0_x_arg = DeclareLaunchArgument('dji0_x', default_value='0.0')
dji0_y_arg = DeclareLaunchArgument('dji0_y', default_value='0.0')
dji0_z_arg = DeclareLaunchArgument('dji0_z', default_value='2.27')
dji0_yaw_arg = DeclareLaunchArgument('dji0_yaw', default_value='0.0')
dji1_x_arg = DeclareLaunchArgument('dji1_x', default_value='0.0')
dji1_y_arg = DeclareLaunchArgument('dji1_y', default_value='0.0')
dji1_z_arg = DeclareLaunchArgument('dji1_z', default_value='3.27')
dji1_yaw_arg = DeclareLaunchArgument('dji1_yaw', default_value='0.0')
dji2_x_arg = DeclareLaunchArgument('dji2_x', default_value='0.0')
dji2_y_arg = DeclareLaunchArgument('dji2_y', default_value='0.0')
dji2_z_arg = DeclareLaunchArgument('dji2_z', default_value='4.27')
dji2_yaw_arg = DeclareLaunchArgument('dji2_yaw', default_value='0.0')

def generate_launch_description():
    gz_world = _gazebo_world_name(LaunchConfiguration('world'))

    dji0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji0",
                          "type": "m100",
                          "uav_mode": LaunchConfiguration('uav_mode'),
                          "with_camera": "true",
                          "camera_name": "camera0",
                          "camera_update_rate": LaunchConfiguration('camera_update_rate'),
                          "x": LaunchConfiguration('dji0_x'),
                          "y": LaunchConfiguration('dji0_y'),
                          "z": LaunchConfiguration('dji0_z'),
                          "Y": LaunchConfiguration('dji0_yaw'),
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
                          "with_camera": "true",
                          "camera_name": "camera0",
                          "camera_update_rate": LaunchConfiguration('camera_update_rate'),
                          "x": LaunchConfiguration('dji1_x'),
                          "y": LaunchConfiguration('dji1_y'),
                          "z": LaunchConfiguration('dji1_z'),
                          "Y": LaunchConfiguration('dji1_yaw'),
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
                          "with_camera": "true",
                          "camera_name": "camera0",
                          "camera_update_rate": LaunchConfiguration('camera_update_rate'),
                          "x": LaunchConfiguration('dji2_x'),
                          "y": LaunchConfiguration('dji2_y'),
                          "z": LaunchConfiguration('dji2_z'),
                          "Y": LaunchConfiguration('dji2_yaw'),
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='support_uav_set_pose_bridge',
#        arguments= ['/world/orchard/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        arguments=[
            ['/world/', gz_world, '/set_pose@ros_gz_interfaces/srv/SetEntityPose']
        ],
        output='screen'
    )

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='support_uav_camera_bridge',
        arguments=[
            '/dji0/camera0/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji0/camera0/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/dji0/camera0/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji1/camera0/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji1/camera0/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/dji1/camera0/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji2/camera0/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/dji2/camera0/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo',
            '/dji2/camera0/depth_image@sensor_msgs/msg/Image@ignition.msgs.Image',
        ],
        remappings=[
            ('/dji0/camera0/image', '/dji0/camera0/image_raw'),
            ('/dji1/camera0/image', '/dji1/camera0/image_raw'),
            ('/dji2/camera0/image', '/dji2/camera0/image_raw'),
        ],
        output='screen',
    )
    return LaunchDescription([
        world_arg,
        uav_mode_arg,
        camera_mode_arg,
        camera_update_rate_arg,
        dji0_x_arg,
        dji0_y_arg,
        dji0_z_arg,
        dji0_yaw_arg,
        dji1_x_arg,
        dji1_y_arg,
        dji1_z_arg,
        dji1_yaw_arg,
        dji2_x_arg,
        dji2_y_arg,
        dji2_z_arg,
        dji2_yaw_arg,
        bridge,
        dji0,
        dji1,
        dji2,
        camera_bridge,
    ])
