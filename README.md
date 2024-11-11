# 2025 Lunabotics Competition - RAER Software Repo

This repo houses the ROS2 - Humble workspace for our submission in the 2024 NASA RMC (Regolith Mining Competition)


### Structure

The source code of the project follows this structure:

```
LunaboticsBurm                                   <--- Root of the ROS2 Workspace
├── launch
│   └── launchfile-1.xml                         <--- ROS2 launch files with custom parameters
└── src                                          <--- ROS2 Packages
    ├── ROS2-Package-1
    │   ├── arduino_files                        <--- Microcontroller companion code
    │   │   ├── platform-1                       <--- Platform.io project parent directory
    │   │   │   ├── include
    │   │   │   ├── lib
    │   │   │   ├── platformio.ini
    │   │   │   └── src
    │   │   │       └── main.cpp
    │   │   └── platform-2                       <--- Platform.io project parent directory
    │   └── ROS2-Package-1                       <--- ROS2 Package source code
    │       ├── node1.py                         <--- ROS2 Node
    │       └── node2.py                         <--- ROS2 Node
    └── ROS2-Package-2      
```

## Code Overview

### ROS2 Packages:

1. arduino_serial_package - This package is responsible for communicating with various microcontrollers over serial. This includes drivetrain and linear actuator signaling
2. teleop_input - This package contains modified teleop_twist nodes for the specific functional requirment of the robot

## Setup Details

The setup for each node and Arduino program is documented in each respective README file.

1. For nodes, this is found at the package level.
2. For Arduino files, this is found at the project level.
