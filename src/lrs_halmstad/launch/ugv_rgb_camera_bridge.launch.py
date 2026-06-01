from launch import LaunchDescription
from launch_ros.actions import Node
from ros_gz_bridge.actions import RosGzBridge


def generate_launch_description():
    camera_bridge = RosGzBridge(
        bridge_name='ugv_camera_0_rgb_bridge',
        use_composition=False,
        extra_bridge_params={
            'bridges': {
                'image': {
                    'ros_topic_name': '/a201_0000/sensors/camera_0/color/image',
                    'ros_type_name': 'sensor_msgs/msg/Image',
                    'gz_topic_name': '/a201_0000/sensors/camera_0/image',
                    'gz_type_name': 'gz.msgs.Image',
                    'direction': 'GZ_TO_ROS',
                    'lazy': True,
                },
                'camera_info': {
                    'ros_topic_name': '/a201_0000/sensors/camera_0/color/camera_info',
                    'ros_type_name': 'sensor_msgs/msg/CameraInfo',
                    'gz_topic_name': '/a201_0000/sensors/camera_0/camera_info',
                    'gz_type_name': 'gz.msgs.CameraInfo',
                    'direction': 'GZ_TO_ROS',
                    'lazy': True,
                },
            },
            'bridge_names': ['image', 'camera_info'],
        },
    )

    camera_static_tf = Node(
        name='camera_0_static_tf',
        executable='static_transform_publisher',
        package='tf2_ros',
        namespace='a201_0000',
        output='screen',
        arguments=[
            '--frame-id', 'camera_0_link',
            '--child-frame-id', 'a201_0000/robot/base_link/camera_0',
        ],
        remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')],
        parameters=[{'use_sim_time': True}],
    )
    return LaunchDescription([
        #camera_bridge,         <--- KEEP OFF UNLESS NEEDED
        camera_static_tf,
    ])
