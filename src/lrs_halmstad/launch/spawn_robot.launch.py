import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
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


def _gazebo_world_name(world_sub):
    return PythonExpression([
        "'office_construction' if '",
        world_sub,
        "' == 'construction' else '",
        world_sub,
        "'",
    ])

def generate_launch_description():
    urdf_path = get_package_share_path('lrs_halmstad') / 'urdf'
    sdf_path = get_package_share_path('lrs_halmstad') / 'sdf'
    default_urdf_model_path = urdf_path / 'lrs_camera.urdf.xacro'
    default_sdf_model_path = sdf_path / 'lrs_camera.sdf'

    world_arg = DeclareLaunchArgument(name='world', default_value='empty',
                                      description='World to spawn in')
    
    name_arg = DeclareLaunchArgument(name='name', default_value='m100',
                                     description='Name of model')
    
    model_arg = DeclareLaunchArgument(name='model', default_value=str(default_sdf_model_path),
                                      description='Absolute path to robot file')

    type_arg = DeclareLaunchArgument(name='type', default_value="m100",
                                     description='Type of model')
    
    uav_mode_arg = DeclareLaunchArgument(name='uav_mode', default_value="teleport",
                                         description='UAV mode: teleport or physics')

    with_camera_arg = DeclareLaunchArgument(name='with_camera', default_value="false",
                                            description='Attach camera/gimbal to the model')

    bridge_camera_arg = DeclareLaunchArgument(
        name='bridge_camera',
        default_value="false",
        description='Bridge /<name>/<camera_name> image + camera_info topics to ROS'
    )
    bridge_gimbal_arg = DeclareLaunchArgument(
        name='bridge_gimbal',
        default_value="true",
        description='Bridge gimbal joint command topic(s) between ROS and Gazebo'
    )
    camera_pitch_offset_deg_arg = DeclareLaunchArgument(
        name='camera_pitch_offset_deg',
        default_value="45.0",
        description='Attached camera mount pitch offset in degrees',
    )
    camera_name_arg = DeclareLaunchArgument(name='camera_name', default_value="camera0",
                                            description='Attached camera name')

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    R = LaunchConfiguration('R')
    P = LaunchConfiguration('P')
    Y = LaunchConfiguration('Y')
    gz_world = _gazebo_world_name(LaunchConfiguration('world'))
    # Historical behavior attached camera automatically in physics mode only.
    # Keep that default, but honor an explicit with_camera:=true in teleport mode too.
    with_camera_for_mode = PythonExpression([
        "'true' if (",
        "'", LaunchConfiguration('with_camera'), "'.lower() in ('1','true','yes','on')",
        " or ",
        "'", LaunchConfiguration('uav_mode'), "' == 'physics'",
        ") else 'false'"
    ])
    model_static_for_mode = PythonExpression([
        "'true' if '", LaunchConfiguration('uav_mode'), "' == 'teleport' else 'false'"
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
                '-world', gz_world,
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
                    " -p model_static:=", model_static_for_mode,
                    " -p camera_pitch_offset_deg:=", LaunchConfiguration('camera_pitch_offset_deg'),
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

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/', LaunchConfiguration('name'), '/', LaunchConfiguration('camera_name'),
             '/image_raw@sensor_msgs/msg/Image@ignition.msgs.Image'],
            ['/', LaunchConfiguration('name'), '/', LaunchConfiguration('camera_name'),
             '/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo'],
        ],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'", LaunchConfiguration('bridge_camera'), "'.lower() in ('1','true','yes','on')"
        ])),
    )

    gimbal_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            ['/model/', LaunchConfiguration('name'),
             '/joint/', LaunchConfiguration('name'),
             '_gimbal_joint/pitch/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'],
            ['/model/', LaunchConfiguration('name'),
             '/joint/', LaunchConfiguration('name'),
             '_gimbal_joint/yaw/cmd_pos@std_msgs/msg/Float64@ignition.msgs.Double'],
        ],
        output='screen',
        condition=IfCondition(PythonExpression([
            "'",
            LaunchConfiguration('bridge_gimbal'),
            "'.lower() in ('1','true','yes','on') and '",
            with_camera_for_mode,
            "' == 'true'"
        ])),
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
        bridge_camera_arg,
        bridge_gimbal_arg,
        camera_pitch_offset_deg_arg,
        camera_name_arg,
        model_arg,
        world_arg,        
        name_arg,
        spawn_node,
        camera_bridge,
        gimbal_bridge,
    ])
