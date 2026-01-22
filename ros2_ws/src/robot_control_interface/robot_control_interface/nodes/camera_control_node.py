#!/usr/bin/env python3
"""
Camera Control Node for Robot Interface
Basic version: Click to control robot movement
Advanced version: ArUco marker detection
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np

class CameraControlNode(Node):
    def __init__(self):
        super().__init__('camera_control_node')
        
        # Parameters
        self.declare_parameter('robot_type', 'turtlebot')  # or 'ur5'
        self.declare_parameter('use_aruco', False)
        self.declare_parameter('camera_topic', '/camera/image_raw')
        
        self.robot_type = self.get_parameter('robot_type').value
        self.use_aruco = self.get_parameter('use_aruco').value
        self.camera_topic = self.get_parameter('camera_topic').value
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        self.center_x = 320  # Default, will be updated
        self.center_y = 240
        
        # ArUco detector setup
        if self.use_aruco:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Publisher for robot control
        if self.robot_type == 'turtlebot':
            self.control_pub = self.create_publisher(Twist, '/cmd_vel', 10)
            self.get_logger().info("Control node ready for TurtleBot")
        else:  # UR5
            self.control_pub = self.create_publisher(
                JointTrajectory, 
                '/scaled_joint_trajectory_controller/joint_trajectory', 
                10
            )
            self.get_logger().info("Control node ready for UR5")
        
        # Image subscriber
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Mouse callback for basic version
        if not self.use_aruco:
            cv2.namedWindow('Camera View')
            cv2.setMouseCallback('Camera View', self.mouse_callback)
        
        self.get_logger().info(f"Camera Control Node started with robot_type={self.robot_type}, use_aruco={self.use_aruco}")
    
    def mouse_callback(self, event, x, y, flags, param):
        """Handle mouse clicks for basic version"""
        if event == cv2.EVENT_LBUTTONDOWN:
            self.control_robot_from_click(x, y)
    
    def control_robot_from_click(self, x, y):
        """Control robot based on click position"""
        if self.robot_type == 'turtlebot':
            # TurtleBot control
            msg = Twist()
            if y < self.center_y:  # Above center
                msg.linear.x = 0.2  # Move forward
                self.get_logger().info("Moving FORWARD")
            else:  # Below center
                msg.linear.x = -0.2  # Move backward
                self.get_logger().info("Moving BACKWARD")
            self.control_pub.publish(msg)
        else:
            # UR5 control - rotate base joint
            msg = JointTrajectory()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                              'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            
            point = JointTrajectoryPoint()
            if y < self.center_y:  # Above center
                # Rotate base joint +30 degrees
                point.positions = [0.5236, -1.57, 1.57, -1.57, -1.57, 0.0]
                self.get_logger().info("Rotating UR5 base FORWARD (+30°)")
            else:  # Below center
                # Rotate base joint -30 degrees
                point.positions = [-0.5236, -1.57, 1.57, -1.57, -1.57, 0.0]
                self.get_logger().info("Rotating UR5 base BACKWARD (-30°)")
            
            point.time_from_start.sec = 2
            msg.points.append(point)
            self.control_pub.publish(msg)
    
    def detect_aruco_markers(self, cv_image):
        """Detect ArUco markers and return their centers"""
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params
        )
        
        if ids is not None:
            centers = []
            for i in range(len(ids)):
                # Calculate center of marker
                center_x = int(np.mean(corners[i][0][:, 0]))
                center_y = int(np.mean(corners[i][0][:, 1]))
                centers.append((center_x, center_y, ids[i][0]))
                
                # Draw marker
                cv2.aruco.drawDetectedMarkers(cv_image, corners, ids)
                cv2.circle(cv_image, (center_x, center_y), 5, (0, 255, 0), -1)
                cv2.putText(cv_image, f"ID: {ids[i][0]}", 
                           (center_x + 10, center_y), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            
            return centers, cv_image
        
        return [], cv_image
    
    def control_robot_from_aruco(self, center_y):
        """Control robot based on ArUco marker position"""
        if self.robot_type == 'turtlebot':
            msg = Twist()
            if center_y < self.center_y:  # Marker above center
                msg.linear.x = 0.15
                self.get_logger().info(f"ArUco detected ABOVE center: Moving FORWARD")
            else:  # Marker below center
                msg.linear.x = -0.15
                self.get_logger().info(f"ArUco detected BELOW center: Moving BACKWARD")
            self.control_pub.publish(msg)
        else:
            # UR5 control
            msg = JointTrajectory()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.joint_names = ['shoulder_pan_joint', 'shoulder_lift_joint', 
                              'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint']
            
            point = JointTrajectoryPoint()
            if center_y < self.center_y:  # Marker above center
                point.positions = [0.4, -1.57, 1.57, -1.57, -1.57, 0.0]
            else:  # Marker below center
                point.positions = [-0.4, -1.57, 1.57, -1.57, -1.57, 0.0]
            
            point.time_from_start.sec = 2
            msg.points.append(point)
            self.control_pub.publish(msg)
    
    def image_callback(self, msg):
        """Process incoming images"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.center_y, self.center_x = cv_image.shape[0] // 2, cv_image.shape[1] // 2
            
            # Draw center line
            cv2.line(cv_image, (0, self.center_y), (cv_image.shape[1], self.center_y), 
                    (0, 0, 255), 2)
            cv2.putText(cv_image, f"Robot: {self.robot_type.upper()}", 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            if self.use_aruco:
                # ArUco detection mode
                centers, cv_image = self.detect_aruco_markers(cv_image)
                if centers:
                    # Control based on first detected marker
                    center_x, center_y, marker_id = centers[0]
                    self.control_robot_from_aruco(center_y)
                    cv2.putText(cv_image, f"ArUco ID: {marker_id}", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                else:
                    cv2.putText(cv_image, "No ArUco markers detected", 
                               (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                # Click mode - just display instructions
                cv2.putText(cv_image, "Click ABOVE line to move FORWARD", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(cv_image, "Click BELOW line to move BACKWARD", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
            # Display image
            cv2.imshow('Camera View', cv_image)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")
    
    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = CameraControlNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
