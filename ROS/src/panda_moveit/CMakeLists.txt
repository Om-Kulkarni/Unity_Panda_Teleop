cmake_minimum_required(VERSION 3.0.2)
project(panda_moveit)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  moveit_msgs
  sensor_msgs
  trajectory_msgs
  ros_tcp_endpoint
  message_generation
  roslaunch
)

## Check for launch files
roslaunch_add_file_check(launch)

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg)

## Generate services in the 'srv' folder  
add_service_files(DIRECTORY srv)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  moveit_msgs
  sensor_msgs
  trajectory_msgs
)

## Declare a catkin package
catkin_package(
  CATKIN_DEPENDS 
    rospy
    std_msgs
    geometry_msgs
    moveit_msgs
    sensor_msgs
    trajectory_msgs
    ros_tcp_endpoint
    message_runtime
)

## Install Python scripts
catkin_install_python(PROGRAMS
  scripts/panda_mover.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Install launch files
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)

## Install config files if any
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
