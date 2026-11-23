"""Start robot_state_publisher with the robot hand URDF (publishes TF)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('robothand_description')
    default_model = os.path.join(pkg_share, 'urdf', 'robothand.urdf.xacro')

    model = LaunchConfiguration('model')
    use_sim_time = LaunchConfiguration('use_sim_time')

    robot_description = ParameterValue(Command(['xacro ', model]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('model', default_value=default_model,
                              description='Absolute path to the robot xacro/URDF file'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': use_sim_time,
                'publish_frequency': 30.0,
            }],
        ),
    ])
