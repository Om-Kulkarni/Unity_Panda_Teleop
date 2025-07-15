# Panda MoveIt Package

This package provides MoveIt integration for the Franka Panda robot with Unity support.

## Overview

The `panda_moveit` package enables pick and place operations for the Franka Panda robot using MoveIt motion planning. It's designed to work with Unity through ROS-TCP-Endpoint for VR robotics applications.

## Features

- **Pick and Place Planning**: Complete pick and place trajectory planning using MoveIt
- **7-DOF Support**: Full support for the Franka Panda's 7 degrees of freedom
- **Unity Integration**: Designed to work seamlessly with Unity robotics projects
- **ROS Service Interface**: Clean service-based API for trajectory planning

## Dependencies

- ROS Noetic
- MoveIt
- panda_moveit_config
- ros_tcp_endpoint
- moveit_commander

## Usage

### Basic Launch

To start the Panda MoveIt service:

```bash
roslaunch panda_moveit panda_moveit_service.launch
```

### Service Interface

The package provides a `panda_moveit` service that accepts:

**Request:**
- `PandaMoveitJoints joints_input`: Current joint positions (7 DOF)
- `geometry_msgs/Pose pick_pose`: Target pick pose
- `geometry_msgs/Pose place_pose`: Target place pose

**Response:**
- `moveit_msgs/RobotTrajectory[] trajectories`: Array of 4 trajectories:
  1. Pre-grasp (move above object)
  2. Grasp (move down to object)
  3. Pick up (lift object)
  4. Place (move to placement location)

### Integration with Unity

This package is designed to work with the Unity ROS-TCP-Endpoint. The Unity side should:

1. Connect to ROS using ROSConnection
2. Send current joint states and desired pick/place poses
3. Execute the returned trajectories on the Unity robot model
4. Handle gripper actions between trajectory segments

## File Structure

```
panda_moveit/
├── CMakeLists.txt
├── package.xml
├── README.md
├── msg/
│   └── PandaMoveitJoints.msg
├── srv/
│   └── PandaMoverService.srv
├── scripts/
│   └── panda_mover.py
└── launch/
    └── panda_moveit_service.launch
```

## Notes

- The package uses the existing `panda_moveit_config` for robot configuration
- Gripper control is handled externally (in Unity)
- The service operates in "fake" execution mode for Unity integration
- Joint names follow the standard Panda convention: panda_joint1 through panda_joint7
