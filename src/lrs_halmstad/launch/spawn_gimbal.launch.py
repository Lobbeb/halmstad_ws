import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from ament_index_python.packages import get_package_prefix, get_package_share_path


def _gazebo_world_name(world_sub):
    return PythonExpression([
        "'office_construction' if '",
        world_sub,
        "' == 'construction' else '",
        world_sub,
        "'",
    ])


def generate_launch_description():
    xacro_path = get_package_share_path('lrs_halmstad') / 'xacro'
    default_xacro_file = xacro_path / 'lrs_gimbal.xacro'

    world_arg = DeclareLaunchArgument(name='world', default_value='warehouse',
                                      description='World to spawn in')

    name_arg = DeclareLaunchArgument(name='name', default_value='dji0',
                                     description='Name of model')

    camera_name_arg = DeclareLaunchArgument(name='camera_name', default_value='camera0',
                                            description='Name of camera')

    type_arg = DeclareLaunchArgument(name='type', default_value="m100",
                                     description='Type of model')

    xacro_file_arg = DeclareLaunchArgument(
        name='xacro_file',
        default_value=str(default_xacro_file),
        description='Absolute path to detached camera xacro file',
    )

    camera_update_rate_arg = DeclareLaunchArgument(name='camera_update_rate', default_value="30",
                                                   description='Camera update rate')
    camera_sensor_roll_deg_arg = DeclareLaunchArgument(
        name='camera_sensor_roll_deg',
        default_value='0.0',
        description='Detached camera sensor roll offset in degrees',
    )
    camera_sensor_pitch_deg_arg = DeclareLaunchArgument(
        name='camera_sensor_pitch_deg',
        default_value='0.0',
        description='Detached camera sensor pitch offset in degrees',
    )
    camera_sensor_yaw_deg_arg = DeclareLaunchArgument(
        name='camera_sensor_yaw_deg',
        default_value='0.0',
        description='Detached camera sensor yaw offset in degrees',
    )
    bridge_camera_arg = DeclareLaunchArgument(
        name='bridge_camera',
        default_value='true',
        description='Bridge detached camera image_raw and camera_info topics to ROS',
    )

    gz_world = _gazebo_world_name(LaunchConfiguration('world'))
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
                '-name', [LaunchConfiguration('name'), "_", LaunchConfiguration('camera_name')],
                '-robot_namespace', LaunchConfiguration('name'),
                '-string', Command([
                    generate_sdf_exe, " ", "--ros-args",
                    " -p type:=", LaunchConfiguration('type'),
                    " -p robot_name:=", LaunchConfiguration('name'),
                    " -p camera_name:=", LaunchConfiguration('camera_name'),
                    " -p xacro_file:=", LaunchConfiguration('xacro_file'),
                    " -p gimbal:=True",
                    " -p camera_update_rate:=", LaunchConfiguration('camera_update_rate'),
                    " -p camera_sensor_roll_deg:=", LaunchConfiguration('camera_sensor_roll_deg'),
                    " -p camera_sensor_pitch_deg:=", LaunchConfiguration('camera_sensor_pitch_deg'),
                    " -p camera_sensor_yaw_deg:=", LaunchConfiguration('camera_sensor_yaw_deg'),
                ]),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),
                '-R', LaunchConfiguration('R'),
                '-P', LaunchConfiguration('P'),
                '-Y', LaunchConfiguration('Y'),
                '--ros-args', '--log-level', 'spawn_entity:=debug'
            ]
        )


    bridge = Node(
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
            "'",
            LaunchConfiguration('bridge_camera'),
            "'.lower() in ('1','true','yes','on')",
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
        xacro_file_arg,
        world_arg,        
        name_arg,
        camera_name_arg,
        camera_update_rate_arg,
        camera_sensor_roll_deg_arg,
        camera_sensor_pitch_deg_arg,
        camera_sensor_yaw_deg_arg,
        bridge_camera_arg,
        spawn_node,
        bridge
    ])
