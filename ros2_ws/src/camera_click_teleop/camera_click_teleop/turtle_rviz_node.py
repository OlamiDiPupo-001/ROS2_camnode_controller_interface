#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from visualization_msgs.msg import Marker
from geometry_msgs.msg import TransformStamped
import tf2_ros
import math

class TurtleRviz(Node):
    def __init__(self):
        super().__init__('turtle_rviz_node')

        self.sub = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10
        )

        self.marker_pub = self.create_publisher(
            Marker,
            '/turtle_marker',
            10
        )

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

    def pose_callback(self, msg):
        # --- TF ---
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'turtle1'
        t.transform.translation.x = msg.x
        t.transform.translation.y = msg.y
        t.transform.translation.z = 0.0

        qz = math.sin(msg.theta / 2.0)
        qw = math.cos(msg.theta / 2.0)
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(t)

        # --- Marker ---
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = t.header.stamp
        marker.ns = 'turtle'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = msg.x
        marker.pose.position.y = msg.y
        marker.pose.position.z = 0.0
        marker.pose.orientation = t.transform.rotation

        marker.scale.x = 0.5
        marker.scale.y = 0.3
        marker.scale.z = 0.2

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = TurtleRviz()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

