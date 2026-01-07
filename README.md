# 18-DOF Spider Robot - ROS Melodic Workspace

[![ROS](https://img.shields.io/badge/ROS-Melodic-blue.svg)](http://wiki.ros.org/melodic)
[![Ubuntu](https://img.shields.io/badge/Ubuntu-18.04-orange.svg)](https://releases.ubuntu.com/18.04/)
[![Gazebo](https://img.shields.io/badge/Gazebo-9.0-green.svg)](http://gazebosim.org/)

## Overview

Autonomous 18-DOF hexapod (spider) robot workspace developed with **ROS Melodic Morenia** on **Ubuntu 18.04 (Bionic Beaver)**, featuring full Gazebo simulation and hardware integration with NVIDIA Jetson Nano Eagle-101 and HI-WONDER LSC-32 servo driver.

## Features

- ✅ **18 Degrees of Freedom**: 6 legs × 3 joints (coxa, thigh, tibia)
- ✅ **ROS Melodic Integration**: Full ROS control architecture with effort controllers
- ✅ **Gazebo Simulation**: Accurate physics simulation with tripod gait locomotion
- ✅ **RViz Visualization**: Real-time joint state monitoring
- ✅ **Teleop Control**: Keyboard-based manual control via `teleop_twist_keyboard`
- ✅ **Hardware Bridge**: Serial communication for LSC-32 servo driver
- ✅ **Walker Node**: Autonomous gait generation for complex terrain navigation

## Hardware Components

- **Platform**: NVIDIA Jetson Nano Eagle-101
- **OS**: Ubuntu 18.04 LTS (Bionic Beaver)
- **Servo Controller**: HI-WONDER LSC-32-v1.3 (32-channel)
- **Servos**: 18× RDS3115 MG Digital Servos (15kg-cm torque)
- **Power**: 6V/20A regulated DC supply (servos), 5V/4A (Jetson)
- **Communication**: UART serial (115200 baud, /dev/ttyTHS1)

## Software Stack

- **ROS Distribution**: Melodic Morenia
- **Simulation**: Gazebo 9.0
- **Visualization**: RViz
- **Control**: ros_control, effort_controllers, position_controllers
- **Packages**:
  - `phantomx_description`: URDF models and meshes
  - `phantomx_gazebo`: Simulation launch files and walker node
  - `phantomx_control`: Controller configurations
  - `hexapod`: Gait generation algorithms
  - `Hopper_ROS`: Hexapod utilities

## Installation

### Prerequisites

```bash
sudo apt update
sudo apt install ros-melodic-desktop-full ros-melodic-gazebo-ros-pkgs \
  ros-melodic-ros-control ros-melodic-ros-controllers \
  ros-melodic-effort-controllers ros-melodic-teleop-twist-keyboard -y
Clone and Build
bash
# Clone this repository
cd ~
git clone https://github.com/namanharshwal/spider-robot-ros-melodic.git catkin_ws
cd catkin_ws

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build workspace
catkin_make
source devel/setup.bash
Usage
Simulation (Gazebo + Teleop)
bash
# Terminal 1: Launch simulation
roslaunch phantomx_gazebo phantomx_gazebo.launch

# Terminal 2: Teleop control
rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/phantomx/cmd_vel
Controls: W (forward), A (left), S (back), D (right), Space (stop)

With RViz Visualization
bash
# Terminal 3: RViz
rosrun rviz rviz
# Add: RobotModel, JointState (topic: /joint_states)
Project Structure
text
catkin_ws/
├── src/
│   ├── phantomx_description/       # URDF, meshes, xacro files
│   ├── phantomx_gazebo/            # Gazebo launch files, walker node
│   ├── phantomx_control/           # Controller configurations
│   ├── hexapod/                    # Gait generation algorithms
│   └── Hopper_ROS/                 # Hexapod utilities
├── .gitignore
└── README.md
Applications
🔍 Search and rescue in uneven terrain

🏭 Industrial inspection in confined spaces

🌾 Agricultural monitoring

🤖 Robotics research and education

🚀 Autonomous exploration

Acknowledgments
Built upon and integrated with cutting-edge open-source repositories:

HumaRobotics/phantomx_gazebo — Full 18-DOF hexapod simulation

PeterL328/hexapod — Hexapod gait generation and IK

dmweis/Hopper_ROS — Hexapod locomotion framework

Author
Naman Harshwal
Robotics Engineer | ROS Developer
Connect with me on LinkedIn


