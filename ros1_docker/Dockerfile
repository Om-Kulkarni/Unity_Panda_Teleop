# Base image: ROS Noetic desktop-full
# This image includes:
# - ROS core packages
# - RViz for visualization
# - Gazebo for simulation
# - Other GUI tools and dependencies
FROM osrf/ros:noetic-desktop-full

# Configure shell and environment
# - Use bash as the default shell for ROS compatibility
# - Set ROS_DISTRO for package installations and setup scripts
# - Disable the ROS 1 End-of-Life warning dialog
SHELL ["/bin/bash", "-c"]
ENV ROS_DISTRO=noetic
ENV DISABLE_ROS1_EOL_WARNINGS=1

# Install required packages:
# - git: For cloning repositories
# - cmake: Build system required by catkin
# - python3-catkin-tools: Modern build tool for ROS (replaces catkin_make)
# - ros-*-libfranka: Core library for controlling Franka Emika robots
# - ros-*-moveit: Motion planning framework
# - ros-*-ros-control: Hardware interface controllers
# - ros-*-ros-controllers: Standard robot controllers
# - ros-*-boost-sml: State machine library required by franka_gazebo
# - dos2unix: Line ending conversion tool for cross-platform compatibility
RUN apt-get update && apt-get install -y \
    git \
    cmake \
    python3-catkin-tools \
    ros-${ROS_DISTRO}-libfranka \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-boost-sml \
    dos2unix \
    && rm -rf /var/lib/apt/lists/*

# Create and set up the catkin workspace directory
WORKDIR /catkin_ws

# Set up the ROS workspace:
# 1. Source the ROS environment
# 2. Create the src directory
# 3. Copy the local Franka ROS package from ROS/src
# 4. Copy the panda_moveit_config package from ROS/src 
# 5. Copy the panda_moveit package for Unity-ROS pick and place integration
# 6. Copy the ros_tcp_endpoint package for Unity-ROS communication
# 7. Fix line endings in ros_tcp_endpoint Python files (Windows -> Unix)
# 8. Install dependencies using rosdep (skipping libfranka and boost_sml as they're installed above)
# 9. Build the workspace using catkin
RUN mkdir -p src
COPY ./ROS/src/franka_ros /catkin_ws/src/franka_ros
COPY ./ROS/src/panda_moveit_config /catkin_ws/src/panda_moveit_config
COPY ./ROS/src/panda_moveit /catkin_ws/src/panda_moveit
COPY ./ROS/src/ros_tcp_endpoint /catkin_ws/src/ros_tcp_endpoint

# Fix line endings and Python shebangs for ros_tcp_endpoint to ensure cross-platform compatibility
RUN find /catkin_ws/src/ros_tcp_endpoint -name "*.py" -type f -exec dos2unix {} \; && \
    find /catkin_ws/src/ros_tcp_endpoint -name "*.py" -type f -exec sed -i '1s|^#!/usr/bin/env python$|#!/usr/bin/env python3|' {} \;

RUN . /opt/ros/noetic/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro ${ROS_DISTRO} --skip-keys "libfranka boost_sml" && \
    catkin build 

# Set up the container's entry point
# 1. Copy the custom entrypoint script that sets up the ROS environment
# 2. Make it executable
COPY ros1_docker/entrypoint.sh /catkin_ws/entrypoint.sh
RUN chmod +x /catkin_ws/entrypoint.sh

# Configure container startup:
# - ENTRYPOINT: Use our custom script to ensure proper ROS environment setup
# - CMD: Default to an interactive bash shell when no command is provided
ENTRYPOINT ["/catkin_ws/entrypoint.sh"]
CMD ["bash"]