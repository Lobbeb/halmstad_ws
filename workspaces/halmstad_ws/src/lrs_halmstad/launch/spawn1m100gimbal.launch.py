import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_m100_with_gimbal(name, z, world):
    res = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": name,
                          "type": "m100",
                          "with_camera": "true",
                          "world": world,
                          "z": f'{z}'
                          }.items(),
    )
    return res;

def get_m100(name, z, world):
    res = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": name,
                          "type": "m100",
                          "world": world,
                          "z": f'{z}'
                          }.items(),
    )
    return res;

def get_gimbal(name, z, world):
    res = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={"name": name,
                          "type": "m100",
                          "world": world,
                          "camera_name": "camera0",
                          "z": f'{z}'
                          }.items(),
    )
    return res;

def generate_launch_description():
    name_arg = DeclareLaunchArgument(
        'name',
        default_value='dji0',
        description='UAV base name')
    world_arg = DeclareLaunchArgument(
        'world',
        default_value='orchard',
        description='Gazebo World')
    name = LaunchConfiguration('name')
    world = LaunchConfiguration('world')
    
    dji0 = get_m100(name, 9.0, world)

    gimb0 = get_gimbal(name, 9.0, world)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/world/', world, '/set_pose@ros_gz_interfaces/srv/SetEntityPose']
        ],
        output='screen'
    )
    
    
    return LaunchDescription([
        name_arg,
        world_arg,
        bridge,
        dji0,
        gimb0
    ])
