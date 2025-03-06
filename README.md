

# RAER Presents:
 ```
    ____    ____   ____    ____                         ///,        ////
   / __ )  /  _/  / __ \  / __ \                        \  /,      /  >.
  / __  |  / /   / /_/ / / / / /                         \  /,   _/  /.
 / /_/ / _/ / _ / _, _/ / /_/ /                           \_  /_/   /.
/_____(_)___/(_)_/ |_(_)_____(_)                           \__/_   <
Basic Industrial Regolith Driver                           /<<< \_\_
....................................                      /,)^>>_._ \
...........................                               (/   \\ /\\\
.............                                                  // ````
....                                                          ((`
```
This repo houses RAER's (Robotic Association at Embry-Riddle) entrance into the 2025 NASA Lunabotics Challenge.

## 🎯 Objectives
The purpose of this robot is to navigate simulated lunar terrain, excavate lunar regolith simulant, and construct a berm in a fully autonomous manner. See [*Lunabotics Guidebook 2025*](https://www.nasa.gov/wp-content/uploads/2024/08/lunaboticsguidebook-2025.pdf?emrc=2a35f5?emrc=2a35f5)

*The following information is subject to change*
## ⚙️ Hardware Description
1. Raspberry Pi 4B
2. Livox MID360 LiDAR
3. Intel RealSense D456 Stereo Camera
4. Clearcore Motion Controller
5. ClearPath MCVC Motors
6. Custom AVR -Based Micro-Controller Expansion Boards

## 💻 Software Description
BIRD is built on ROS2 (Humble) and Ubuntu 22.04 (Jammy Jellyfish). It is comprised of the following nodes and features:
1. Custom Hardware Drivers
2. Serial Communication Nodes
3. [Livox ROS2 Driver](https://github.com/Livox-SDK/livox_ros_driver2)
4. [RealSense ROS2 Driver](https://github.com/IntelRealSense/realsense-ros)
5. [IMU Madgwick Filters](https://wiki.ros.org/imu_filter_madgwick)
6. Unit Conversion Nodes
7. [State Kalman Filtering (Robot Localization)](https://docs.ros.org/en/melodic/api/robot_localization/html/index.html)
8. [3D SLAM](https://github.com/hku-mars/FAST_LIO/tree/ROS2)
9. [Nav2](https://docs.nav2.org/)
10. Custom State Machine and Fault Detection System