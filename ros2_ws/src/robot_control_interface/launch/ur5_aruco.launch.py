#!/usr/bin/env python3
"""
Launch file for UR5 with ArUco control
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Start UR5 simulation in Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('ur_simulation_gazebo'),
                    'launch',
                    'ur5.launch.py'
                ])
            ]),
            launch_arguments={
                'ur_type': 'ur5',
                'use_fake_hardware': 'false',
                'launch_rviz': 'false'
            }.items()
        ),
        
        # Start camera driver
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[{
                'video_device': '/dev/video0',
                'image_width': 640,
                'image_height': 480,
                'framerate': 30.0,
                'pixel_format': 'yuyv',
                'camera_frame_id': 'camera_link'
            }]
        ),
        
        # Start our control node with ArUco
        Node(
            package='robot_control_interface',
            executable='camera_control_node',
            name='camera_control_node',
            parameters=[{
                'robot_type': 'ur5',
                'use_aruco': True,
                'camera_topic': '/image_raw'
            }],
            output='screen'
        )
    ])
