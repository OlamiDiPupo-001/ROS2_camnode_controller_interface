#!/bin/bash

echo "Installing ROS2 Humble dependencies..."

# Install ROS2 packages
sudo apt-get update
sudo apt-get install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-camera-calibration-parsers \
    ros-humble-usb-cam \
    ros-humble-ur-moveit \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-turtlebot3-gazebo \
    ros-humble-turtlebot3

# Install Python packages
pip3 install opencv-python opencv-contrib-python

echo "Installation complete!"
