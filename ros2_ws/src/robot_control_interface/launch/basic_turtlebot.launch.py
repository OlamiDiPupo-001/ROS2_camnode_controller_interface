#!/usr/bin/env python3
"""
Launch file for basic TurtleBot control
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start TurtleBot3 simulation
        ExecuteProcess(
            cmd=['ros2', 'launch', 'turtlebot3_gazebo', 'turtlebot3_world.launch.py'],
            output='screen',
            shell=True
        ),
        
        # Start camera driver (USB camera)
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
        
        # Start our control node
        Node(
            package='robot_control_interface',
            executable='camera_control_node',
            name='camera_control_node',
            parameters=[{
                'robot_type': 'turtlebot',
                'use_aruco': False,
                'camera_topic': '/image_raw'
            }]
        )
    ])
