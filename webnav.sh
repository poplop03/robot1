#!/bin/bash

# Exit on any error
set -e

# Handle Ctrl+C
cleanup() {
  echo "Caught Ctrl+C! Stopping Node.js server and ROS..."
  kill $NODE_PID
  exit 0
}
trap cleanup SIGINT

# Source ROS environment
source /opt/ros/noetic/setup.bash
source ~/ROS1/robot1/devel/setup.bash

# Start Node.js server in background
echo "Starting Node.js server..."
cd ~/react-ui-ros
npm start &
NODE_PID=$!  # Capture Node.js PID

# Return to previous directory
cd -

# Start ROS launch (this blocks)
echo "Starting ROS launch..."
roslaunch robot1_bringup web_nav.launch

# After launch exits, cleanup in case trap didn’t trigger
cleanup

