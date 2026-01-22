from setuptools import setup
import os
from glob import glob

package_name = 'robot_control_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name, f'{package_name}/nodes'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ROS2 interface for robot control using camera input and ArUco markers',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_control_node = robot_control_interface.nodes.camera_control_node:main',
        ],
    },
)
