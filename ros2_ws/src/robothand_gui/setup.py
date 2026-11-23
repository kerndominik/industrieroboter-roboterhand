import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robothand_gui'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Robothand Team',
    maintainer_email='robothand@todo.todo',
    description='Qt GUI of the robot hand: control, live plots, Denavit-Hartenberg kinematics',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robothand_gui = robothand_gui.app:main',
        ],
    },
)
