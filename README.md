# Unity_Panda_Teleop
A VR teleoperation project for the Franka Emika Panda robot using Unity and ROS. This project enables real-time control of the Panda robot's end-effector through VR controllers with full 6DOF (position and orientation) manipulation.

A demonstration video of the Panda VR Teleoperation system is available below:

## Panda VR Teleoperation Demo

**Watch the Panda VR Teleoperation demo:**

<p align="center">
  <a href="files/Franka_VR_Teleop.mp4">
    <img src="files/Franka_VR_Teleop_thumbnail.png" alt="Panda VR Teleop Demo Thumbnail" width="480" />
  </a>
</p>

[▶️ Click here to download or view the demo video (MP4)](files/Franka_VR_Teleop.mp4)

If the video does not play in your browser, right-click the link above and choose "Save link as..." to download.

## Features
- Real-time VR teleoperation of Franka Panda robot
- Full 6DOF end-effector control (position + rotation)
- MoveIt-based inverse kinematics solving
- ROS 1 integration with Unity
- Docker-based development environment
- Smooth VR controller interaction with grab-and-move interface

## Project Structure

```
Unity_Panda_Teleop/
├── Panda_Teleop/             # Unity VR Project
│   ├── Assets/
│   │   ├── Scripts/
│   │   │   └── PandaVRTeleoperator.cs  # Main VR teleoperation script
│   │   ├── RosMessages/      # Generated ROS message types
│   │   └── Scenes/           # Unity scenes with Panda robot
├── ROS/
│   └── src/
│       ├── panda_moveit/     # Custom MoveIt integration package
│       │   ├── scripts/
│       │   │   ├── panda_mover.py        # Pick-and-place service (legacy)
│       │   │   └── panda_teleop_server.py # Real-time IK solver service
│       │   ├── srv/
│       │   │   ├── PandaMoverService.srv   # Pick-and-place service definition
│       │   │   └── PandaIKSolver.srv       # Real-time IK service definition
│       │   ├── msg/
│       │   │   └── PandaMoveitJoints.msg   # Joint state message
│       │   └── launch/
│       │       └── panda_unity_integration.launch  # Streamlined launch file
│       ├── panda_moveit_config/  # MoveIt configuration for Panda
│       └── ros_tcp_endpoint/     # Unity-ROS communication bridge
└── ros1_docker/              # ROS 1 Noetic Docker setup
```

## How VR Teleoperation Works

### Control Method
- **Grab Interface**: Use VR controllers to directly grab and move the robot's end-effector
- **6DOF Control**: Full position (X, Y, Z) and rotation (Roll, Pitch, Yaw) control
- **Real-time IK**: MoveIt solves inverse kinematics at 30Hz for smooth operation
- **Collision Checking**: Optional collision detection during movement

### Technical Flow
```
VR Controller Movement → Unity PandaVRTeleoperator → ROS TCP → panda_teleop_server → MoveIt IK Solver → Joint Commands → Robot Movement
```

### Key Components
1. **PandaVRTeleoperator.cs**: Unity script handling VR input and ROS communication
2. **panda_teleop_server.py**: ROS service providing real-time IK solving
3. **PandaIKSolver.srv**: Service definition for fast IK requests
4. **MoveIt Integration**: Robust inverse kinematics and motion planning

## Setup Instructions

### Prerequisites
- Docker Desktop with WSL2 backend
- Unity 2022.3 LTS or later
- VR headset compatible with Unity XR (Meta Quest, HTC Vive, etc.)
- Git (for cloning repositories)
- Windows 11 (for WSLg GUI support)

### Clone Required ROS Packages

Before building the Docker image, you need to clone the required ROS packages into your `ROS/src` directory:

```bash
# Navigate to the ROS source directory
cd ROS/src/

# Clone Panda MoveIt configuration
git clone https://github.com/moveit/panda_moveit_config.git

# Clone Unity ROS TCP Endpoint for Unity-ROS communication
git clone https://github.com/Unity-Technologies/ROS-TCP-Endpoint.git ros_tcp_endpoint
```

**Note**: The `panda_moveit` package is already included in this repository and contains the custom teleoperation services.

### Troubleshooting ROS TCP Endpoint

**Line Ending Issues:**
The `ros_tcp_endpoint` package from GitHub may have Windows-style (CRLF) line endings that can cause errors. If you encounter issues like `/usr/bin/env: 'python\r': No such file or directory`, fix them with:

```bash
# Convert line endings and fix shebang lines in Python files
find ROS/src/ros_tcp_endpoint -name "*.py" -exec dos2unix {} \;
find ROS/src/ros_tcp_endpoint -name "*.py" -exec sed -i '1s|^#!/usr/bin/env python$|#!/usr/bin/env python3|' {} \;
```

## Docker Instructions

### ROS 1 Noetic (Franka Panda Teleoperation)

1. **Setup the Workspace**
   Ensure you have cloned the required ROS packages (see "Clone Required ROS Packages" section above) into your `ROS/src` directory:
   - `panda_moveit_config/` - MoveIt configuration for Panda
   - `ros_tcp_endpoint/` - Unity-ROS communication bridge
   - `panda_moveit/` - Custom teleoperation services (included in repo)

2. **Build the Docker Image**
   ```bash
   # Build from the project root directory
   docker build -t panda-teleop-noetic -f ros1_docker/Dockerfile .
   ```

3. **Start the Docker Container**
   Start a detached container with GUI support and port forwarding:
   ```powershell
   docker run -d --name panda_teleop_container `
                -it `
                -p 10000:10000 `
                -p 5005:5005 `
                -v /run/desktop/mnt/host/wslg/.X11-unix:/tmp/.X11-unix `
                -v /run/desktop/mnt/host/wslg:/mnt/wslg `
                -v ${PWD}/ROS/src/panda_moveit:/catkin_ws/src/panda_moveit `
                -e DISPLAY=:0 `
                -e WAYLAND_DISPLAY=wayland-0 `
                -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir `
                -e PULSE_SERVER=/mnt/wslg/PulseServer `
                panda-teleop-noetic `
                tail -f /dev/null
   ```

   **Port forwarding explained:**
   - `-p 10000:10000`: Unity-ROS TCP communication port
   - `-p 5005:5005`: Additional communication/debugging port

4. **Launch the Panda VR Teleoperation Service**
   Open a terminal and enter the Docker container:
   ```bash
   # Enter the container
   docker exec -it panda_teleop_container /bin/bash

   # Inside the container, launch the VR teleoperation integration
   . devel/setup.bash
   roslaunch panda_moveit panda_unity_integration.launch
   ```

   This streamlined launch file starts:
   - ROS core
   - MoveIt planning server for the Panda robot (without RViz by default)
   - Unity-ROS TCP communication endpoint
   - Real-time IK solver service for VR teleoperation
   - Essential robot state publishers and TF broadcasters

### Unity Setup

1. **Import Robot URDF**
   - The Panda robot URDF should already be configured in the Unity project
   - If needed, generate ROS messages: `Robotics -> Generate ROS Messages`
   - Navigate to `ROS/src/panda_moveit/msg/` and `ROS/src/panda_moveit/srv/`
   - Build the required message types for teleoperation

2. **Configure VR**
   - Set up your VR headset with Unity XR
   - Ensure XR Interaction Toolkit is installed
   - Configure VR controllers in the Unity scene

3. **Setup PandaVRTeleoperator**
   - Attach the `PandaVRTeleoperator.cs` script to a GameObject in your scene
   - Assign the Panda robot GameObject in the inspector
   - The script will auto-configure the end-effector grab interactable
   - Adjust control parameters as needed:
     - `positionSmoothingFactor`: Controls position responsiveness (default: 10)
     - `rotationSmoothingFactor`: Controls rotation responsiveness (default: 5)
     - `updateRate`: IK solving frequency in Hz (default: 30)
     - `minimumMovementThreshold`: Minimum movement to trigger IK (default: 0.001m)
     - `minimumRotationThreshold`: Minimum rotation to trigger IK (default: 1°)

## Starting the VR Teleoperation

### Step-by-Step Launch Process

1. **Launch the Teleoperation Services**
   ```bash
   # Enter the container
   docker exec -it panda_teleop_container /bin/bash

   # Launch the streamlined teleoperation services
   . devel/setup.bash
   roslaunch panda_moveit panda_unity_integration.launch
   ```

3. **Start Unity VR**
   - Put on your VR headset
   - Open the Unity project (`Panda_Teleop`)
   - Load the teleoperation scene
   - Press Play to start the VR simulation

4. **Begin Teleoperation**
   - In VR, locate the Panda robot's end-effector (hand)
   - Use your VR controller to grab the end-effector
   - Move and rotate your controller to control the robot
   - The robot will follow your movements in real-time with smooth IK solving

### What Happens During Teleoperation

When you grab the end-effector:
1. **Grab Detection**: XR Grab Interactable detects controller interaction
2. **Offset Calculation**: System calculates grab offset for smooth control
3. **Real-time Updates**: At 30Hz, Unity sends target poses to ROS
4. **IK Solving**: MoveIt solves inverse kinematics for the target pose
5. **Joint Control**: Solved joint angles are applied to the robot
6. **Visual Feedback**: Robot moves smoothly following your VR controller

### Control Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| Position Smoothing | 10.0 | Higher = more responsive position |
| Rotation Smoothing | 5.0 | Higher = more responsive rotation |
| Update Rate | 30 Hz | IK solving frequency |
| Movement Threshold | 0.001m | Minimum movement to trigger update |
| Rotation Threshold | 1° | Minimum rotation to trigger update |

## Troubleshooting

### Connection Issues
**If Unity can't connect to ROS:**
- Verify Docker container is running: `docker ps`
- Check ROS services: `docker exec -it panda_teleop_container rosnode list`
- Ensure port 10000 is properly forwarded
- Check Unity Console for ROS TCP connection messages

### Performance Issues
**If teleoperation is laggy:**
- Increase `updateRate` for more responsive control (may increase CPU usage)
- Decrease `positionSmoothingFactor` and `rotationSmoothingFactor` for smoother movement
- Disable collision checking in the PandaVRTeleoperator inspector
- Adjust movement/rotation thresholds to reduce unnecessary IK requests

### Robot Movement Issues
**If robot doesn't move or moves incorrectly:**
- Verify robot URDF is properly loaded in Unity
- Check joint names match between Unity and ROS
- Ensure end-effector transform is correctly identified
- Monitor ROS logs for IK solver errors: `docker exec -it panda_teleop_container rostopic echo /rosout`

### VR Setup Issues
**If VR controllers don't work:**
- Verify XR Interaction Toolkit is properly configured
- Check VR headset tracking and controller pairing
- Ensure XRGrabInteractable is attached to the end-effector
- Test VR interaction with other Unity objects first

## Architecture Details

### Services
- **`panda_ik_solver`**: Real-time IK solving service (50ms timeout)
- **`panda_moveit`**: Legacy pick-and-place service (still available)

### Key Messages
- **`PandaIKSolverRequest`**: Target pose + current joints + options
- **`PandaIKSolverResponse`**: Success flag + joint solution + error message
- **`PandaMoveitJoints`**: Joint angle array message

### Performance Optimizations
- **Fast IK solving**: 50ms timeout with 3 planning attempts
- **Movement filtering**: Only sends requests when movement exceeds thresholds
- **Smooth interpolation**: Applies smoothing to VR controller input
- **Collision checking toggle**: Can disable for faster performance

This teleoperation system provides intuitive, real-time control of the Franka Panda robot through VR, making it ideal for research, training, and remote manipulation tasks.
