#!/usr/bin/env python3
# teachbot.launch.py
"""
Launch file for TOS Teachbot ROS2 node.

Usage:
    ros2 launch teachbot_ros teachbot.launch.py
    ros2 launch teachbot_ros teachbot.launch.py remote_ip:=192.168.100.152
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directory
    pkg_share = get_package_share_directory('teachbot_ros')
    
    # Default config file path
    default_config = os.path.join(pkg_share, 'config', 'teachbot_params.yaml')
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )
    
    remote_ip_arg = DeclareLaunchArgument(
        'remote_ip',
        default_value='',
        description='Override teachbot IP address (empty = use config file value)'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='',
        description='Override publish rate in Hz (empty = use config file value)'
    )
    
    # Create the node
    teachbot_node = Node(
        package='teachbot_ros',
        executable='teachbot_publisher',
        name='teachbot_publisher',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        # Remap topics if needed
        remappings=[
            # Example: ('/teachbot/joint_states', '/my_robot/joint_states')
        ]
    )
    
    return LaunchDescription([
        config_file_arg,
        remote_ip_arg,
        publish_rate_arg,
        teachbot_node
    ])
