"""Visualize the robot hand in RViz2 (without hardware).

robot_state_publisher publishes the TF transforms of the URDF model. The joint
angles come from joint_state_publisher_gui (sliders) by default, so the model
can be explored without the real hand. Set jsp_gui:=false if another node
(e.g. robothand_interface) publishes /joint_states.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('robothand_description')

    jsp_gui = LaunchConfiguration('jsp_gui')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')

    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value=os.path.join(pkg_share, 'urdf', 'robothand.urdf.xacro'),
            description='Absolute path to the robot xacro/URDF file'),
        DeclareLaunchArgument('jsp_gui', default_value='true',
                              description='Start joint_state_publisher_gui (sliders)'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument(
            'rviz_config', default_value=os.path.join(pkg_share, 'rviz', 'robothand.rviz'),
            description='RViz2 config file'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_share, 'launch', 'description.launch.py')),
            launch_arguments={'model': LaunchConfiguration('model')}.items(),
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            condition=IfCondition(jsp_gui),
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(rviz),
        ),
    ])
