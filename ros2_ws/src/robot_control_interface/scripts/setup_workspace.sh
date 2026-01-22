#!/bin/bash

echo "Setting up workspace..."

# Source ROS2
source /opt/ros/humble/setup.bash

# Navigate to workspace
cd ~/ros2_ws

# Build the package
colcon build --symlink-install

# Source the workspace
source install/setup.bash

echo "Workspace setup complete!"
