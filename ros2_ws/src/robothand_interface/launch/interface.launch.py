"""Start only the robot hand hardware interface."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_params = os.path.join(get_package_share_directory('robothand_interface'),
                                  'config', 'robothand_interface.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('simulate', default_value='false'),
        Node(
            package='robothand_interface',
            executable='robothand_interface',
            name='robothand_interface',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'port': LaunchConfiguration('port'),
                    'simulate': ParameterValue(LaunchConfiguration('simulate'), value_type=bool),
                },
            ],
        ),
    ])
