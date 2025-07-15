#!/bin/bash
# This script is the entrypoint for the ROS 1 Docker container.

# Source the ROS 1 Noetic installation to set up the environment.
source /opt/ros/noetic/setup.bash

# Source the local catkin workspace to make our built packages available.
if [ -f /catkin_ws/devel/setup.bash ]; then
  source /catkin_ws/devel/setup.bash
fi

# Execute the command passed to the docker run command.
exec "$@"