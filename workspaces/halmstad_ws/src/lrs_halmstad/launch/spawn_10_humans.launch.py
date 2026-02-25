import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def get_human(name, x, y, z):
    res = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lrs_halmstad'),
                'spawn_robot.launch.py')),
        launch_arguments={"name": name,
                          "type": "human",
                          "x": f'{x}',
                          "y": f'{y}',
                          "z": f'{z}',
                          }.items(),
    )
    return res;

def generate_launch_description():
    z = 2.0
    x = 0.0
    y = 0.0
    human0 = get_human("human0", x+0.0, y+0.0, z)
    human1 = get_human("human1", x+1.0, y+0.0, z)
    human2 = get_human("human2", x+2.0, y+0.0, z)
    human3 = get_human("human3", x+3.0, y+0.0, z)
    human4 = get_human("human4", x+4.0, y+0.0, z)
    human5 = get_human("human5", x+5.0, y+0.0, z)
    human6 = get_human("human6", x+6.0, y+0.0, z)
    human7 = get_human("human7", x+7.0, y+0.0, z)
    human8 = get_human("human8", x+8.0, y+0.0, z)
    human9 = get_human("human9", x+9.0, y+0.0, z)

    return LaunchDescription([
        human0,
        human1,
        human2,
        human3,
        human4,
        human5,
        human6,
        human7,
        human8,
        human9,
    ])
    
