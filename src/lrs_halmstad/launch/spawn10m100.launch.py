import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration


world_arg = DeclareLaunchArgument('world', default_value='orchard',
                      choices=[
                          'construction',
                          'office',
                          'orchard',
                          'pipeline',
                          'solar_farm',
                          'warehouse',
                      ],
                      description='Gazebo World')

def generate_launch_description():
    
    dji0 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji0",
                          "type": "m100",
                          "z": "0.0",
                          "world": LaunchConfiguration('world')
                          }.items(),
    )
    
    dji0gimbal = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_gimbal.launch.py')),
        launch_arguments={"name": "dji0gimbal",
                          "type": "m100",
                          "z": "0.0",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji1 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji1",
                          "type": "m100",
                          "z": "1.0",
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
                          "z": "2.0",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji3",
                          "type": "m100",
                          "z": "3.0",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji4 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji4",
                          "type": "m100",
                          "z": "4.0",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji5 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji5",
                          "type": "m100",
                          "z": "5.0",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji6 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji6",
                          "type": "m100",
                          "z": "6.0",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji7 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji7",
                          "type": "m100",
                          "z": "7.0",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji8 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji8",
                          "type": "m100",
                          "z": "8.0",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )
    
    dji9 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": "dji9",
                          "type": "m100",
                          "z": "9.0",
                          "world": LaunchConfiguration('world'),
                          }.items(),
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/world/', LaunchConfiguration('world'), '/set_pose@ros_gz_interfaces/srv/SetEntityPose']
        ],
        output='screen'
    )
    
    
    return LaunchDescription([
        world_arg,
        bridge,
        dji0,
        dji0gimbal,
        dji1,
        dji2,
        dji3,
        dji4,
        dji5,
        dji6,
        dji7,
        dji8,
        dji9
    ])
