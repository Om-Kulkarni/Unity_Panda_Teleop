# Digital Twin Utils

A ROS package containing utility scripts and tools for digital twin functionality.

## Overview

This package provides utility functions and scripts that support the digital twin implementation for robotic systems. It includes helper functions for data processing, communication utilities, and other supporting tools.

## Dependencies

- ROS (tested with ROS Noetic)
- Python 3
- rospy
- std_msgs
- geometry_msgs
- sensor_msgs

## Package Structure

```
digital_twin_utils/
├── CMakeLists.txt      # Build configuration
├── package.xml         # Package metadata
├── README.md          # This file
├── scripts/           # Python executable scripts
├── src/               # Source code (Python modules)
└── launch/            # Launch files
```

## Installation

1. Clone this package into your catkin workspace:
```bash
cd ~/catkin_ws/src
# (assuming this is already part of your workspace)
```

2. Build the package:
```bash
cd ~/catkin_ws
catkin_make
```

3. Source the workspace:
```bash
source devel/setup.bash
```

## Usage

This is a utility package that will be used by other packages in the digital twin system. Add specific usage instructions here as you develop utility scripts.

## Contributing

When adding new utility scripts:
1. Place executable Python scripts in the `scripts/` directory
2. Place Python modules/libraries in the `src/digital_twin_utils/` directory
3. Add launch files in the `launch/` directory
4. Update this README with usage instructions

## License

MIT License
