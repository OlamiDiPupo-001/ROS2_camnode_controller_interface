from setuptools import setup
from glob import glob
import os

package_name = 'camera_click_teleop'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages', ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='youremail@example.com',
    description='ROS2 camera click teleoperation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'click_teleop_node = camera_click_teleop.click_teleop_node:main',
            'aruco_teleop_node = camera_click_teleop.aruco_teleop_node:main',
            'turtle_rviz = camera_click_teleop.turtle_rviz_node:main',
        ],
    },
)

