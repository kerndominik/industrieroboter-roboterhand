"""Start only the robot hand GUI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='robothand_gui', executable='robothand_gui', name='robothand_gui',
             output='screen'),
    ])
