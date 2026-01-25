from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Camera node (publishes /image_raw)
    camera_node = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='camera',
        output='screen',
        parameters=[
            {'image_size': [640, 480]},
            {'pixel_format': 'YUYV'}
        ]
    )

    # ArUco teleoperation node
    aruco_teleop_node = Node(
        package='camera_click_teleop',
        executable='aruco_teleop_node',
        name='aruco_teleop',
        output='screen'
    )

    # Mobile robot simulation (Turtlesim)
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
        output='screen'
    )

    # RViz (visualization only)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        camera_node,
        aruco_teleop_node,
        turtlesim_node
    ])

