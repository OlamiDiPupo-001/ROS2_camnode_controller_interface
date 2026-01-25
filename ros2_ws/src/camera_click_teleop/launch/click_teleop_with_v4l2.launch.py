from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        output='screen'
    )

    click_teleop_node = Node(
        package='camera_click_teleop',
        executable='click_teleop_node',
        output='screen'
    )

    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        output='screen'
    )
    rviz_node = Node(
    	package='rviz2',
    	executable='rviz2',
    	output='screen'
    )

    return LaunchDescription([
        camera_node,
        click_teleop_node,
        turtlesim_node,
        rviz_node
    ])

