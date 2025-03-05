from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

pkg_radiacode_driver = get_package_share_directory("radiacode_driver")

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="radiacode_driver",
                executable="radiacode_node.py",
                name="radiacode_node",
                output="screen",
                parameters=[os.path.join(pkg_radiacode_driver, 'config', 'usb.yaml')]
            ),
        ]
    )