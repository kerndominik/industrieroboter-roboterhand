"""Start the complete robot hand system.

    ros2 launch robothand_bringup robothand.launch.py                  # real hand on /dev/ttyUSB0
    ros2 launch robothand_bringup robothand.launch.py simulate:=true   # without hardware
    ros2 launch robothand_bringup robothand.launch.py port:=/dev/ttyACM0 rviz:=false

Nodes:
  robot_state_publisher   URDF -> TF          (robothand_description)
  robothand_interface     ROS <-> Arduino/ESP32 (robothand_interface)
  robothand_gui           control / plots / DH (robothand_gui)      [gui:=true]
  rviz2                   3D view              (robothand_description) [rviz:=true]
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory('robothand_description')
    interface_share = get_package_share_directory('robothand_interface')

    args = [
        DeclareLaunchArgument('simulate', default_value='false',
                              description='true: emulated microcontroller, no hardware needed'),
        DeclareLaunchArgument('port', default_value='/dev/ttyUSB0',
                              description='Serial port (or pyserial URL, e.g. '
                                          'socket://localhost:5555)'),
        DeclareLaunchArgument('baudrate', default_value='115200'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(interface_share, 'config',
                                                         'robothand_interface.yaml'),
                              description='Parameters of robothand_interface (calibration)'),
        DeclareLaunchArgument('gui', default_value='true', description='Start robothand_gui'),
        DeclareLaunchArgument('rviz', default_value='true', description='Start RViz2'),
        DeclareLaunchArgument('rviz_config',
                              default_value=os.path.join(description_share, 'rviz',
                                                         'robothand.rviz')),
    ]

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(description_share, 'launch', 'description.launch.py')))

    interface = Node(
        package='robothand_interface',
        executable='robothand_interface',
        name='robothand_interface',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'port': LaunchConfiguration('port'),
                'baudrate': ParameterValue(LaunchConfiguration('baudrate'), value_type=int),
                'simulate': ParameterValue(LaunchConfiguration('simulate'), value_type=bool),
            },
        ],
    )

    gui = Node(
        package='robothand_gui',
        executable='robothand_gui',
        name='robothand_gui',
        output='screen',
        condition=IfCondition(LaunchConfiguration('gui')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(args + [robot_state_publisher, interface, gui, rviz])
