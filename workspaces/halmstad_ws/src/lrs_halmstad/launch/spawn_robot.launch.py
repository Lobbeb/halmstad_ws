import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import ThisLaunchFileDir, LaunchConfiguration
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path

#from lrs_util import XacroContents
import launch_ros
#import xacro

def generate_launch_description():
    urdf_path = get_package_share_path('lrs_halmstad') / 'urdf'
    sdf_path = get_package_share_path('lrs_halmstad') / 'sdf'
    default_urdf_model_path = urdf_path / 'lrs_piraya.urdf.xacro'
    default_sdf_model_path = sdf_path / 'lrs_piraya.sdf'

    world_arg = DeclareLaunchArgument(name='world', default_value='empty',
                                      description='World to spawn in')
    
    name_arg = DeclareLaunchArgument(name='name', default_value='piraya0',
                                     description='Name of model')
    
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_sdf_model_path),
                                      description='Absolute path to robot file')

    type_arg = DeclareLaunchArgument(name='type', default_value="piraya",
                                     description='Type of model')
    
    uav_mode_arg = DeclareLaunchArgument(name='uav_mode', default_value="teleport",
                                         description='UAV mode: teleport or physics')

    with_camera_arg = DeclareLaunchArgument(name='with_camera', default_value="false",
                                            description='Attach camera/gimbal to the model')
    
    camera_name_arg = DeclareLaunchArgument(name='camera_name', default_value="camera0",
                                            description='Attached camera name')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    R = LaunchConfiguration('R')
    P = LaunchConfiguration('P')
    Y = LaunchConfiguration('Y')
    with_camera_for_mode = PythonExpression([
        "'true' if '", LaunchConfiguration('uav_mode'), "' == 'physics' else 'false'"
    ])

    generate_sdf_exe = os.path.join(
        get_package_prefix('lrs_halmstad'),
        'lib',
        'lrs_halmstad',
        'generate_sdf',
    )

    spawn_node = Node(
            package="ros_gz_sim",
            executable='create',
            name='spawn_entity',
            arguments=[ 
                '-world', LaunchConfiguration('world'),               
                '-name', LaunchConfiguration('name'),
#                '-robot_namespace', LaunchConfiguration('name'),
#                '-topic', ['/', LaunchConfiguration('name'), '/robot_description']
#                '-file', LaunchConfiguration('model'),
#                '-file', "/tmp/gen.sdf",
                '-string', Command([
                    generate_sdf_exe, " ", "--ros-args",
                    " -p type:=", LaunchConfiguration('type'),
                    " -p name:=", LaunchConfiguration('name'),
                    " -p robot:=True",
                    " -p with_camera:=", with_camera_for_mode,
                    " -p camera_name:=", LaunchConfiguration('camera_name')
                ]),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y'),
                '--ros-args', '--log-level', 'info'
            ]
        )

    

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value="0.0"),
        DeclareLaunchArgument('y', default_value="0.0"),
        DeclareLaunchArgument('z', default_value="0.0"),
        DeclareLaunchArgument('R', default_value="0.0"),
        DeclareLaunchArgument('P', default_value="0.0"),
        DeclareLaunchArgument('Y', default_value="0.0"),

        type_arg,
        uav_mode_arg,
        with_camera_arg,
        camera_name_arg,
        model_arg,
        world_arg,        
        name_arg,
        spawn_node
    ])
