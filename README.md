# TOS Teachbot ROS2 Package

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![ROS2 Jazzy](https://img.shields.io/badge/ROS2-Jazzy-blue.svg)](https://docs.ros.org/en/jazzy/)
[![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20Jammy-orange.svg)](https://ubuntu.com/)
[![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04%20Noble-orange.svg)](https://ubuntu.com/)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10+-green.svg)](https://python.org/)
[![License](https://img.shields.io/badge/License-Proprietary-red.svg)](https://tosrobotics.com/)

> **ROS2 driver for the TOS Teachbot — a 6-DOF robotic arm teaching device with high-resolution encoders and intuitive control inputs.**

---

## 📋 Table of Contents

- [Features](#-features)
- [Architecture](#-architecture)
- [Quick Start](#-quick-start)
- [Installation](#-installation)
- [Usage](#-usage)
- [Configuration](#-configuration)
- [API Reference](#-api-reference)
- [Troubleshooting](#-troubleshooting)

---

## ✨ Features

| Feature | Description |
|---------|-------------|
| 🎯 **High-Precision Encoders** | AksIM-2 17-bit absolute encoders (131,072 counts/rev) on all 6 joints |
| ⚡ **250 Hz Publishing Rate** | Low-latency joint state updates for real-time applications |
| 🔧 **Forward Kinematics** | Built-in FK computation for TCP position and orientation |
| 🎮 **Control Inputs** | Trigger potentiometer + 2 programmable buttons |
| 🤖 **RViz Integration** | Launch with UR5e visualization for immediate feedback |
| ⚙️ **Fully Configurable** | YAML-based calibration and parameter tuning |

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                         TOS Teachbot Hardware                        │
├──────────────────────────────┬──────────────────────────────────────┤
│   AksIM-2 Encoders (J1-J6)   │      RS-485 Interface                │
│   TCP Ports 5004-5009        │      TCP Port 5011                   │
└──────────────┬───────────────┴──────────────────┬───────────────────┘
               │                                   │
               ▼                                   ▼
┌──────────────────────────────────────────────────────────────────────┐
│                      teachbot_publisher Node                          │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────────┐   │
│  │ EncoderListener │  │  RS485Listener  │  │ ForwardKinematics   │   │
│  │  (6 threads)    │  │   (1 thread)    │  │                     │   │
│  └────────┬────────┘  └────────┬────────┘  └──────────┬──────────┘   │
│           │                    │                      │              │
│           └────────────────────┴──────────────────────┘              │
│                                │                                      │
│                         Timer @ 250 Hz                                │
└────────────────────────────────┼─────────────────────────────────────┘
                                 │
                                 ▼
              ┌──────────────────┴──────────────────┐
              │                                     │
              ▼                                     ▼
   /teachbot/joint_states              /teachbot/state
   (sensor_msgs/JointState)            (TeachbotState)
```

---

## 🚀 Quick Start

```bash
# Terminal 1: Launch the teachbot node
source ~/ros2_ws/install/setup.bash
ros2 launch teachbot_ros teachbot.launch.py

# Terminal 2: Monitor joint states (radians)
ros2 topic echo /teachbot/joint_states

# Terminal 3: Monitor full state (degrees, FK, buttons, pot)
ros2 topic echo /teachbot/state

# Verify publishing rate (~250 Hz expected)
ros2 topic hz /teachbot/joint_states
```

### With RViz Visualization

```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```

Launches the teachbot publisher with a UR5e robot model in RViz for real-time visualization.

---

## 📦 Installation

### Prerequisites

| Requirement | Supported Versions |
|-------------|-------------------|
| **Ubuntu** | 22.04 Jammy, 24.04 Noble |
| **ROS2** | Humble, Jazzy |
| **Python** | 3.10+ |

### Option 1: Quick Install (Recommended)

```bash
cd ~/ros2_ws/src/teachbot_ros
chmod +x install_teachbot_ros.sh
./install_teachbot_ros.sh
```

### Option 2: Manual Installation

```bash
# Source ROS2 (use your installed version)
source /opt/ros/humble/setup.bash   # For Humble on Ubuntu 22.04
# OR
source /opt/ros/jazzy/setup.bash    # For Jazzy on Ubuntu 24.04

# Create workspace (if needed)
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build interfaces first (custom messages)
colcon build --packages-select teachbot_interfaces
source install/setup.bash

# Build the main package
colcon build --packages-select teachbot_ros
source install/setup.bash
```

### Option 3: Clone from GitHub

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
git clone https://github.com/TOSTeun/teachbot_ros.git
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 🎮 Usage

### Launch Files

| Launch File | Description |
|-------------|-------------|
| `teachbot.launch.py` | Headless mode — publishes topics only |
| `teachbot_rviz.launch.py` | With RViz visualization |

### Running the Node

```bash
# Source workspace (add to ~/.bashrc for convenience)
source ~/ros2_ws/install/setup.bash

# Launch with default parameters
ros2 launch teachbot_ros teachbot.launch.py

# Launch with custom IP address
ros2 launch teachbot_ros teachbot.launch.py remote_ip:=192.168.1.100

# Run node directly
ros2 run teachbot_ros teachbot_publisher
```

---

## ⚙️ Configuration

### Parameter File

The default configuration is in `config/teachbot_params.yaml`. Override by passing a custom config:

```bash
ros2 launch teachbot_ros teachbot.launch.py config_file:=/path/to/my_config.yaml
```

### Available Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `remote_ip` | string | `192.168.100.152` | Teachbot IP address (TCP server mode) |
| `start_port` | int | `5004` | First encoder TCP port (J1) |
| `ee_port` | int | `5011` | RS-485 trigger/button port |
| `robot_model` | string | `Robot_200_200_mm` | Forward kinematics model |
| `publish_rate` | float | `250.0` | Topic publishing rate (Hz) |
| `dof` | int | `6` | Degrees of freedom |

### Calibration Parameters

| Parameter | Type | Description |
|-----------|------|-------------|
| `position_offsets` | int[6] | Encoder raw values at zero position |
| `degree_offsets` | float[6] | Additional angle offsets (degrees) |
| `joint_scale_factors` | float[6] | Direction multipliers (+1 or -1) |
| `pot_idle` | int | Raw potentiometer value at 0% |
| `pot_full` | int | Raw potentiometer value at 100% |
| `joint_names` | string[6] | Joint names for JointState message |

---

## 📡 API Reference

### Published Topics

| Topic | Type | Rate | Description |
|-------|------|------|-------------|
| `/teachbot/joint_states` | `sensor_msgs/JointState` | 250 Hz | Standard ROS joint states (radians) |
| `/teachbot/state` | `teachbot_interfaces/TeachbotState` | 250 Hz | Full teachbot state |

### TeachbotState Message

```yaml
# Header with timestamp
std_msgs/Header header

# Joint angles in degrees (6 DOF)
float64[6] joint_angles_deg

# TCP position from forward kinematics (mm)
float64 tcp_x
float64 tcp_y
float64 tcp_z

# TCP orientation (degrees)  
float64 tcp_rx
float64 tcp_ry
float64 tcp_rz

# Gripper/control inputs
int32 pot_raw           # Raw potentiometer value (0-1023)
int32 pot_percent       # Mapped percentage (0-100)
bool btn1               # Button 1 state
bool btn2               # Button 2 state

# Encoder diagnostics
bool[6] encoder_errors
bool[6] encoder_warnings
float64[6] encoder_frequencies

# Model info
string robot_model
```

### Node Parameters

Query current parameters:

```bash
ros2 param list /teachbot_publisher
ros2 param get /teachbot_publisher remote_ip
```

---

## 🌐 Network Setup

The Teachbot operates as a **TCP server**. This ROS2 node acts as a **TCP client**.

### Connection Diagram

```
┌──────────────────┐         TCP          ┌──────────────────┐
│   Your PC        │◄────────────────────►│    Teachbot      │
│  192.168.100.x   │   Ports 5004-5009    │  192.168.100.152 │
│                  │   Port 5011          │   (TCP Server)   │
│ teachbot_publisher│                      │                  │
│   (TCP Client)   │                      │                  │
└──────────────────┘                      └──────────────────┘
```

### Setup Steps

1. Configure Teachbot in **TCP Server** mode
2. Note the Teachbot's IP address (default: `192.168.100.152`)
3. Configure your PC on the same subnet (e.g., `192.168.100.x`)
4. Update `remote_ip` parameter if different from default
5. Verify connectivity: `ping 192.168.100.152`

### Required Ports

| Ports | Protocol | Data |
|-------|----------|------|
| 5004-5009 | TCP | AksIM-2 Encoder streams (J1-J6) |
| 5011 | TCP | RS-485: Potentiometer + Buttons |

---

## 🔧 Troubleshooting

### Connection Issues

| Problem | Solution |
|---------|----------|
| No data received | Check Teachbot power and TCP server mode |
| Connection timeout | Verify IP: `ping 192.168.100.152` |
| Port unreachable | Test: `nc -zv 192.168.100.152 5004` |
| Auto-reconnecting | Normal behavior if Teachbot reboots |

### Calibration Issues

| Problem | Solution |
|---------|----------|
| Wrong joint directions | Flip sign in `joint_scale_factors` (±1) |
| Wrong zero position | Recalibrate `position_offsets` |
| RViz pose incorrect | Check both offsets and scale factors |

### Build Issues

```bash
# Ensure ROS2 is sourced
source /opt/ros/humble/setup.bash

# Clean rebuild
cd ~/ros2_ws
rm -rf build install log
colcon build
source install/setup.bash
```

---

## 📁 Package Structure

```
teachbot_ros/
├── teachbot_ros/
│   ├── __init__.py
│   ├── teachbot_publisher.py    # Main ROS2 node
│   ├── listeners.py             # TCP listener threads  
│   └── kinematics.py            # Forward kinematics engine
├── config/
│   └── teachbot_params.yaml     # Default configuration
├── launch/
│   ├── teachbot.launch.py       # Headless launch
│   └── teachbot_rviz.launch.py  # Launch with RViz
├── rviz/
│   └── teachbot.rviz            # RViz configuration
├── utils/
│   └── teachbot_viewer.py       # Standalone viewer utility
├── test/                        # Unit tests
├── package.xml
├── setup.py
├── setup.cfg
├── install_teachbot_ros.sh      # Quick install script
├── INSTALL.md                   # Installation guide
└── README.md                    # This file
```

---

## 📦 Dependencies

### ROS2 Packages

```bash
# For ROS2 Humble (Ubuntu 22.04)
sudo apt install ros-humble-ur-description

# For ROS2 Jazzy (Ubuntu 24.04)
sudo apt install ros-jazzy-ur-description
```

### Python Packages

- `numpy` — Matrix operations for FK
- `rclpy` — ROS2 Python client library

---

## 📄 License

Proprietary — © TOS Robotics. All rights reserved.

---

## 🔗 Links

- **Repository:** [github.com/TOSTeun/teachbot_ros](https://github.com/TOSTeun/teachbot_ros)
- **TOS Robotics:** [tosrobotics.com](https://tosrobotics.com)

---

<div align="center">
  <i>Built by TOS Robotics</i>
</div>

