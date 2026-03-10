import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _gazebo_world_name(world_sub):
    return PythonExpression([
        "'office_construction' if '",
        world_sub,
        "' == 'construction' else '",
        world_sub,
        "'",
    ])


def get_m100_with_gimbal(name, x, y, z, world):
    res = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": name,
                          "type": "m100",
                          "with_camera": "true",
                          "bridge_camera": "true",
                          "camera_name": "camera0",
                          "world": world,
                          "x": x,
                          "y": y,
                          "z": z
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
        default_value='warehouse',
        description='Gazebo World')
    x_arg = DeclareLaunchArgument(
        'x',
        default_value='-2.0',
        description='UAV spawn x position')
    y_arg = DeclareLaunchArgument(
        'y',
        default_value='0.0',
        description='UAV spawn y position')
    z_arg = DeclareLaunchArgument(
        'z',
        default_value='5.0',
        description='UAV spawn z position')
    name = LaunchConfiguration('name')
    world = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    gz_world = _gazebo_world_name(world)
    
    dji0 = get_m100_with_gimbal(name, x, y, z, world)

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/world/', gz_world, '/set_pose@ros_gz_interfaces/srv/SetEntityPose']
        ],
        output='screen'
    )
    
    
    return LaunchDescription([
        name_arg,
        world_arg,
        x_arg,
        y_arg,
        z_arg,
        bridge,
        dji0
    ])
