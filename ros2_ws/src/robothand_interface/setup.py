import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robothand_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robothand Team',
    maintainer_email='robothand@todo.todo',
    description='ROS 2 hardware interface of the robot hand (Arduino Mega / ESP32 over USB serial)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robothand_interface = robothand_interface.interface_node:main',
            'fake_mcu = robothand_interface.fake_mcu:main',
        ],
    },
)
