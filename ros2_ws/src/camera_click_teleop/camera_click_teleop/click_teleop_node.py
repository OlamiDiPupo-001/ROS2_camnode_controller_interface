#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image



class ClickTeleop(Node):
    def __init__(self):
        super().__init__('click_teleop')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(Image, '/image_raw', self.image_callback, 10)
        self.img = None
        self.window_name = "Camera Click Teleop"
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)
        self.get_logger().info("Click Teleop Node started")

    def image_callback(self, msg):
        self.img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        cv2.imshow(self.window_name, self.img)
        cv2.waitKey(1)

    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and self.img is not None:
            height, width, _ = self.img.shape
            twist = Twist()
            if y < height // 2:
                twist.linear.x = 0.2
                self.get_logger().info("Moving Forward")
            else:
                twist.linear.x = -0.2
                self.get_logger().info("Moving Backward")
            self.pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ClickTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

