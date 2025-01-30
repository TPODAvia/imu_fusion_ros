#!/bin/bash
set -e

# Source ROS setup
source /opt/ros/humble/setup.bash

# Build if not yet built
cd /ros2_ws
colcon build --symlink-install || true

# Source the workspace
source install/setup.bash

# Launch your node
exec ros2 launch imu_fusion_ros imu_fusion.launch.py