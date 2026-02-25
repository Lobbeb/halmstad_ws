#import logging
#logging.root.setLevel(logging.DEBUG)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression


world_arg = DeclareLaunchArgument('world', default_value='warehouse',
                      choices=[
                          'construction',
                          'office',
                          'orchard',
                          'pipeline',
                          'solar_farm',
                          'warehouse',
                      ],
                      description='Gazebo World')
uav_mode_arg = DeclareLaunchArgument('uav_mode', default_value='teleport',
                      description='UAV mode: teleport (deterministic) or physics')

def generate_launch_description():
    teleport_cond = IfCondition(PythonExpression(["'", LaunchConfiguration('uav_mode'), "' == 'teleport'"]))
    physics_cond = IfCondition(PythonExpression(["'", LaunchConfiguration('uav_mode'), "' == 'physics'"]))
    
    dji0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji0",
                          "type": "m100",
                          "uav_mode": LaunchConfiguration('uav_mode'),
                          "camera_name": "camera0",
                          "z": "2.27",
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
                          "camera_name": "camera0",
                          "z": "3.27",
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
                          "camera_name": "camera0",
                          "z": "4.27",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji0gimbal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={
            "name": "dji0",
            "camera_name": "camera0",
            "type": "m100",
            "z": "2.0",
            "world": LaunchConfiguration('world'),
        }.items(),
        condition=teleport_cond,
    )

    dji1gimbal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={
            "name": "dji1",
            "camera_name": "camera0",
            "type": "m100",
            "z": "3.0",
            "world": LaunchConfiguration('world'),
        }.items(),
        condition=teleport_cond,
    )

    dji2gimbal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={
            "name": "dji2",
            "camera_name": "camera0",
            "type": "m100",
            "z": "4.0",
            "world": LaunchConfiguration('world'),
        }.items(),
        condition=teleport_cond,
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
#        arguments= ['/world/orchard/set_pose@ros_gz_interfaces/srv/SetEntityPose'],
        arguments=[
            ['/world/', LaunchConfiguration('world'),'/set_pose@ros_gz_interfaces/srv/SetEntityPose']
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
        condition=physics_cond
    )
    
    
    return LaunchDescription([
        world_arg,
        uav_mode_arg,
        bridge,
        camera_bridge,
        dji0,
        dji0gimbal,
        dji1,
        dji1gimbal,
        dji2,
        dji2gimbal,
    ])
