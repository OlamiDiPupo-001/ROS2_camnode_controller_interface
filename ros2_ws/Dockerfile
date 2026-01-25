# Use official ROS 2 Humble base image
FROM ros:humble-ros-core

# Install dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-opencv \
    python3-colcon-common-extensions \
    ros-humble-turtlesim \
    ros-humble-v4l2-camera \
    ros-humble-cv-bridge \
    ros-humble-image-tools \
    ros-humble-rviz2 \
    git \
 && rm -rf /var/lib/apt/lists/*

# Install Python packages
RUN pip3 install numpy opencv-contrib-python

# Create workspace inside container
RUN mkdir -p /root/ros2_ws/src
WORKDIR /root/ros2_ws

# Copy your ROS2 package into container
COPY /src/camera_click_teleop src/camera_click_teleop

# Build the ROS2 workspace
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

# Source ROS 2 and workspace setup
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source install/setup.bash && bash"]

