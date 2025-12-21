#!/usr/bin/env python3
# teachbot_rviz.launch.py
"""
Launch file for TOS Teachbot with official UR5e visualization.

Uses the official ur_description package for accurate UR5e model.

Usage:
    ros2 launch teachbot_ros teachbot_rviz.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node


def generate_launch_description():
    # Get package share directories
    pkg_share = get_package_share_directory('teachbot_ros')
    ur_description_share = get_package_share_directory('ur_description')
    
    # File paths
    default_config = os.path.join(pkg_share, 'config', 'teachbot_params.yaml')
    rviz_config = os.path.join(pkg_share, 'rviz', 'teachbot.rviz')
    
    # Generate URDF from xacro for UR5e
    urdf_xacro = os.path.join(ur_description_share, 'urdf', 'ur.urdf.xacro')
    robot_description = Command([
        'xacro ', urdf_xacro,
        ' ur_type:=ur5e',
        ' name:=ur5e',
        ' tf_prefix:=""',
        ' generate_ros2_control_tag:=false',
    ])
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to the configuration YAML file'
    )
    
    # Teachbot publisher node
    teachbot_node = Node(
        package='teachbot_ros',
        executable='teachbot_publisher',
        name='teachbot_publisher',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        remappings=[
            # Remap to UR5e joint names
            ('/teachbot/joint_states', '/joint_states')
        ]
    )
    
    # Robot state publisher (publishes TF from URDF + joint states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 250.0,
        }]
    )
    
    # RViz node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    return LaunchDescription([
        config_file_arg,
        teachbot_node,
        robot_state_publisher_node,
        rviz_node
    ])
