from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            output='screen',
            parameters=[{'image_size': [640, 480]}]
        ),
        Node(
            package='camera_click_teleop',
            executable='click_teleop_node',
            output='screen'
        ),
    ])

