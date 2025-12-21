# Teachbot ROS2 Installation Guide

## Prerequisites

- **Ubuntu 22.04** (tested)
- **ROS2 Humble** installed and sourced
- Python 3.10+

---

## Quick Install (3 steps)

### Step 1: Extract the package

```bash
unzip teachbot_ros_package.zip -d ~/ros2_ws/src/
```

### Step 2: Run the install script

```bash
cd ~/ros2_ws/src/teachbot_ros
chmod +x install_teachbot_ros.sh
./install_teachbot_ros.sh
```

### Step 3: Source your workspace

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Running the Teachbot Node

### Option A: Headless (no visualization)

```bash
ros2 launch teachbot_ros teachbot.launch.py
```

Publishes joint states and teachbot state to ROS topics only.

### Option B: With RViz visualization

```bash
ros2 launch teachbot_ros teachbot_rviz.launch.py
```

Opens RViz with the robot model and live joint visualization.

---

## Verify Installation

Check that topics are being published:

```bash
ros2 topic list | grep teachbot
```

Expected output:
```
/joint_states
/teachbot/state
```

Echo joint states:
```bash
ros2 topic echo /joint_states
```

---

## Network Requirements

The teachbot node listens on these UDP ports:

| Port | Data |
|------|------|
| 5004-5009 | Encoder values (J1-J6) |
| 5011 | RS-485 trigger/buttons |

Make sure these ports are not in use by other applications.

---

## Troubleshooting

### "Address already in use" error
Another application is using the UDP ports. Close `teachbot_viewer.py` or any other conflicting application.

### Robot not moving in RViz
- Check that encoder data is being received: `ros2 topic hz /joint_states`
- Verify UDP packets are arriving on ports 5004-5009

### Build errors
Make sure ROS2 Humble is sourced before building:
```bash
source /opt/ros/humble/setup.bash
cd ~/ros2_ws
colcon build
```

---

## File Structure

```
teachbot_ros_package.zip
├── teachbot_interfaces/     # Custom message definitions
│   └── msg/TeachbotState.msg
└── teachbot_ros/            # Main ROS2 package
    ├── teachbot_ros/        # Python source
    │   ├── teachbot_publisher.py
    │   ├── listeners.py
    │   └── kinematics.py
    ├── launch/              # Launch files
    ├── urdf/                # Robot model
    ├── rviz/                # RViz config
    ├── config/              # Parameters
    ├── install_teachbot_ros.sh
    └── README.md            # Detailed documentation
```

---

## For More Details

See `README.md` in the teachbot_ros package for:
- Complete topic/message documentation
- Parameter configuration
- Development notes
