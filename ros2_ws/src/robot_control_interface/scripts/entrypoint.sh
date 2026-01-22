#!/bin/bash

# Source ROS and workspace
source /opt/ros/humble/setup.bash
source /ros_ws/install/setup.bash

# Set TurtleBot3 model if needed
export TURTLEBOT3_MODEL=waffle

# Execute the command passed to docker run
exec "$@"
