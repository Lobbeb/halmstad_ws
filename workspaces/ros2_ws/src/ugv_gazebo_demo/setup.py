from setuptools import setup, find_packages
from glob import glob
import os

package_name = "ugv_gazebo_demo"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "models", "bebop_with_control"), glob("models/bebop_with_control/*")),
        (os.path.join("share", package_name, "models", "ugv_husky_with_odom"), glob("models/ugv_husky_with_odom/*")),
        (os.path.join("share", package_name, "models", "ugv_x2_with_odom"), glob("models/ugv_x2_with_odom/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "drive_ugv = ugv_gazebo_demo.drive_ugv:main",
            "drive_lidar = ugv_gazebo_demo.drive_lidar:main",
            "bebop_follow_ugv = ugv_gazebo_demo.bebop_follow_ugv:main",
            "gazebo_pose_tcp_bridge = ugv_gazebo_demo.gazebo_pose_tcp_bridge:main",
        ],
    },
)
