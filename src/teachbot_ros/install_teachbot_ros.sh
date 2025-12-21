#!/bin/bash
# install_teachbot_ros.sh
# Installation script for TOS Teachbot ROS2 node
#
# Supported:
#   - Ubuntu 22.04 (Jammy) + ROS2 Humble
#   - Ubuntu 24.04 (Noble) + ROS2 Jazzy
#
# Usage:
#   chmod +x install_teachbot_ros.sh
#   ./install_teachbot_ros.sh

set -e

echo "=========================================="
echo "TOS Teachbot ROS2 Node Installation"
echo "=========================================="
echo ""

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Detect ROS2 installation
echo -e "${YELLOW}[1/6] Detecting ROS2 installation...${NC}"
ROS_DISTRO=""
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    ROS_DISTRO="jazzy"
    source /opt/ros/jazzy/setup.bash
    echo -e "${GREEN}✓ ROS2 Jazzy found${NC}"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
    ROS_DISTRO="humble"
    source /opt/ros/humble/setup.bash
    echo -e "${GREEN}✓ ROS2 Humble found${NC}"
else
    echo -e "${RED}✗ ROS2 not found!${NC}"
    echo "Please install ROS2 Humble or Jazzy first:"
    echo ""
    echo "  For Ubuntu 22.04 (Humble):"
    echo "    sudo apt install ros-humble-desktop python3-colcon-common-extensions"
    echo ""
    echo "  For Ubuntu 24.04 (Jazzy):"
    echo "    sudo apt install ros-jazzy-desktop python3-colcon-common-extensions"
    exit 1
fi

# Check for colcon
echo -e "${YELLOW}[2/6] Checking colcon...${NC}"
if command -v colcon &> /dev/null; then
    echo -e "${GREEN}✓ colcon found${NC}"
else
    echo -e "${YELLOW}Installing colcon...${NC}"
    sudo apt update
    sudo apt install -y python3-colcon-common-extensions
fi

# Check for numpy (required for kinematics)
echo -e "${YELLOW}[3/6] Checking Python dependencies...${NC}"
python3 -c "import numpy" 2>/dev/null || {
    echo -e "${YELLOW}Installing numpy...${NC}"
    pip3 install numpy
}
python3 -c "import yaml" 2>/dev/null || {
    echo -e "${YELLOW}Installing PyYAML...${NC}"
    pip3 install pyyaml
}
echo -e "${GREEN}✓ Python dependencies OK${NC}"

# Create workspace if it doesn't exist
WORKSPACE_DIR="${HOME}/ros2_ws"
echo -e "${YELLOW}[4/6] Setting up workspace at ${WORKSPACE_DIR}...${NC}"

if [ ! -d "${WORKSPACE_DIR}/src" ]; then
    mkdir -p "${WORKSPACE_DIR}/src"
    echo -e "${GREEN}✓ Created workspace directory${NC}"
else
    echo -e "${GREEN}✓ Workspace directory exists${NC}"
fi

# Get the directory where this script is located
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Check if we're running from within ros2_ws already
if [[ "${SCRIPT_DIR}" == *"ros2_ws/src"* ]]; then
    echo -e "${GREEN}✓ Already in ros2_ws, skipping copy${NC}"
else
    # Copy packages to workspace
    echo -e "${YELLOW}Copying packages to workspace...${NC}"
    
    # Copy teachbot_interfaces if it exists in script dir
    if [ -d "${SCRIPT_DIR}/../teachbot_interfaces" ]; then
        cp -r "${SCRIPT_DIR}/../teachbot_interfaces" "${WORKSPACE_DIR}/src/"
    fi
    
    # Copy teachbot_ros if it exists in script dir
    if [ -d "${SCRIPT_DIR}/../teachbot_ros" ]; then
        cp -r "${SCRIPT_DIR}/../teachbot_ros" "${WORKSPACE_DIR}/src/"
    fi
fi

# Build the workspace
echo -e "${YELLOW}[5/6] Building ROS2 packages...${NC}"
cd "${WORKSPACE_DIR}"
source /opt/ros/${ROS_DISTRO}/setup.bash

# Build interfaces first (needed by teachbot_ros)
echo "Building teachbot_interfaces..."
colcon build --packages-select teachbot_interfaces
source install/setup.bash

# Build the main package
echo "Building teachbot_ros..."
colcon build --packages-select teachbot_ros

echo -e "${GREEN}✓ Build complete${NC}"

# Add to bashrc if not already there
echo -e "${YELLOW}[6/6] Updating shell configuration...${NC}"
BASHRC_LINE="source ${WORKSPACE_DIR}/install/setup.bash"
if grep -q "ros2_ws/install/setup.bash" ~/.bashrc; then
    echo -e "${GREEN}✓ Already in .bashrc${NC}"
else
    echo "" >> ~/.bashrc
    echo "# TOS Teachbot ROS2 workspace" >> ~/.bashrc
    echo "${BASHRC_LINE}" >> ~/.bashrc
    echo -e "${GREEN}✓ Added to .bashrc${NC}"
fi

echo ""
echo "=========================================="
echo -e "${GREEN}Installation Complete!${NC}"
echo "=========================================="
echo ""
echo "To use the teachbot node:"
echo ""
echo "  1. Source the workspace (or open a new terminal):"
echo "     source ${WORKSPACE_DIR}/install/setup.bash"
echo ""
echo "  2. Run the node:"
echo "     ros2 run teachbot_ros teachbot_publisher"
echo ""
echo "  3. Or use the launch file:"
echo "     ros2 launch teachbot_ros teachbot.launch.py"
echo ""
echo "  4. View topics:"
echo "     ros2 topic list"
echo "     ros2 topic echo /teachbot/joint_states"
echo "     ros2 topic echo /teachbot/state"
echo ""
echo "  5. Check publishing rate:"
echo "     ros2 topic hz /teachbot/joint_states"
echo ""
echo "Configuration file:"
echo "  ${WORKSPACE_DIR}/install/teachbot_ros/share/teachbot_ros/config/teachbot_params.yaml"
echo ""
echo "To modify parameters, copy the config file and launch with:"
echo "  ros2 launch teachbot_ros teachbot.launch.py config_file:=/path/to/your/config.yaml"
echo ""
