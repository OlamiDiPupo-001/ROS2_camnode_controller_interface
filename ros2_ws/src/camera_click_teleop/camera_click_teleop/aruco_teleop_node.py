#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist

from cv_bridge import CvBridge
import cv2
import numpy as np


class ArucoTeleop(Node):

    def __init__(self):
        super().__init__('aruco_teleop_node')

        # Subscribers & Publishers
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/turtle1/cmd_vel',
            10
        )

        self.bridge = CvBridge()

        # ArUco setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(
            cv2.aruco.DICT_4X4_50
        )
        self.aruco_params = cv2.aruco.DetectorParameters()

        self.get_logger().info("ArUco Teleop Node Started")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        height, width, _ = frame.shape
        image_center_y = height // 2

        # Detect ArUco markers
        corners, ids, rejected = cv2.aruco.detectMarkers(
            frame,
            self.aruco_dict,
            parameters=self.aruco_params
        )

        cmd = Twist()

        if ids is not None:
            # Take first detected marker
            marker_corners = corners[0][0]

            # Compute marker center
            marker_center_y = int(marker_corners[:, 1].mean())

            # Draw marker and center line
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            cv2.line(
                frame,
                (0, image_center_y),
                (width, image_center_y),
                (0, 255, 0),
                2
            )

            # Control logic
            error = image_center_y - marker_center_y

            if abs(error) < 20:
                cmd.linear.x = 0.0
            elif error > 0:
                cmd.linear.x = 1.0   # move forward
            else:
                cmd.linear.x = -1.0  # move backward

        else:
            # No marker → stop
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

        # Display image
        cv2.imshow("ArUco Teleop", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTeleop()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

