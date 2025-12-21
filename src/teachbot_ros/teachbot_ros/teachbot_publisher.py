#!/usr/bin/env python3
# teachbot_publisher.py
"""
ROS2 Node for publishing TOS Teachbot joint states and control inputs.

Publishes:
  - /teachbot/joint_states (sensor_msgs/JointState) @ 250 Hz
  - /teachbot/state (teachbot_interfaces/TeachbotState) @ 250 Hz
"""

from __future__ import annotations

import math
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

from teachbot_interfaces.msg import TeachbotState

from .listeners import EncoderListener, RS485Listener, MAX_POSITION, pot_raw_to_percent
from .kinematics import ForwardKinematics, get_forward_kinematics


class TeachbotPublisher(Node):
    """ROS2 node that publishes teachbot joint states at 250 Hz."""

    def __init__(self):
        super().__init__('teachbot_publisher')
        
        # Declare parameters with defaults
        self.declare_parameter('remote_ip', '192.168.100.152')
        self.declare_parameter('start_port', 5004)
        self.declare_parameter('ee_port', 5011)
        self.declare_parameter('robot_model', 'Robot_200_200_mm')
        self.declare_parameter('publish_rate', 250.0)
        self.declare_parameter('dof', 6)
        
        # Calibration parameters (lists)
        self.declare_parameter('position_offsets', [46000, 99320, 96150, 56200, 81850, 44400])
        self.declare_parameter('degree_offsets', [0.0, -5.0, -5.0, 0.0, 0.0, 0.0])
        self.declare_parameter('joint_scale_factors', [-1.0, 1.0, 1.0, 1.0, -1.0, 1.0])
        
        # Potentiometer calibration
        self.declare_parameter('pot_idle', 811)
        self.declare_parameter('pot_full', 58)
        
        # Joint names for JointState message
        self.declare_parameter('joint_names', ['joint_1', 'joint_2', 'joint_3', 
                                                'joint_4', 'joint_5', 'joint_6'])
        
        # Get parameters
        self._remote_ip = self.get_parameter('remote_ip').value
        self._start_port = self.get_parameter('start_port').value
        self._ee_port = self.get_parameter('ee_port').value
        self._robot_model = self.get_parameter('robot_model').value
        self._publish_rate = self.get_parameter('publish_rate').value
        self._dof = self.get_parameter('dof').value
        
        self._position_offsets = list(self.get_parameter('position_offsets').value)
        self._degree_offsets = list(self.get_parameter('degree_offsets').value)
        self._joint_scale_factors = list(self.get_parameter('joint_scale_factors').value)
        
        self._pot_idle = self.get_parameter('pot_idle').value
        self._pot_full = self.get_parameter('pot_full').value
        self._joint_names = list(self.get_parameter('joint_names').value)
        
        # State tracking for angle unwrapping
        self._prev_angles: List[Optional[float]] = [None] * self._dof
        self._joint_angles: List[float] = [0.0] * self._dof
        
        # Initialize forward kinematics
        try:
            self._fk = get_forward_kinematics(self._robot_model)
            self.get_logger().info(f'Loaded FK for model: {self._robot_model}')
        except KeyError as e:
            self.get_logger().warn(f'FK not available: {e}')
            self._fk = None
        
        # Create publishers
        self._joint_state_pub = self.create_publisher(
            JointState, 
            '/teachbot/joint_states', 
            10
        )
        self._state_pub = self.create_publisher(
            TeachbotState, 
            '/teachbot/state', 
            10
        )
        
        # Start TCP listeners (connect to teachbot server)
        self._encoder_threads: List[EncoderListener] = []
        for i in range(self._dof):
            port = self._start_port + i
            listener = EncoderListener(self._remote_ip, port)
            listener.start()
            self._encoder_threads.append(listener)
            self.get_logger().info(f'Started encoder TCP client for {self._remote_ip}:{port}')
        
        self._rs485_thread = RS485Listener(self._remote_ip, self._ee_port)
        self._rs485_thread.start()
        self.get_logger().info(f'Started RS-485 TCP client for {self._remote_ip}:{self._ee_port}')
        
        # Create timer for publishing
        timer_period = 1.0 / self._publish_rate
        self._timer = self.create_timer(timer_period, self._publish_callback)
        
        self.get_logger().info(
            f'Teachbot publisher started @ {self._publish_rate} Hz, connecting to {self._remote_ip}'
        )

    def _compute_joint_angle(self, idx: int, snap: dict) -> float:
        """Compute calibrated joint angle with unwrapping."""
        offset = self._position_offsets[idx] if idx < len(self._position_offsets) else 0
        corrected_counts = (snap["pos"] - offset) & (MAX_POSITION - 1)
        raw_angle = corrected_counts * 360.0 / MAX_POSITION
        
        # Apply scale factor
        scale = self._joint_scale_factors[idx] if idx < len(self._joint_scale_factors) else 1.0
        raw_angle *= scale
        
        # Apply degree offset
        deg_offset = self._degree_offsets[idx] if idx < len(self._degree_offsets) else 0.0
        raw_angle += deg_offset
        
        # Unwrap angle to prevent discontinuities
        prev = self._prev_angles[idx]
        if prev is not None:
            diff = raw_angle - prev
            while diff > 180.0:
                diff -= 360.0
            while diff < -180.0:
                diff += 360.0
            angle = prev + diff
        else:
            angle = raw_angle
            while angle > 180.0:
                angle -= 360.0
            while angle < -180.0:
                angle += 360.0
        
        self._prev_angles[idx] = angle
        return angle

    def _publish_callback(self):
        """Timer callback to publish joint states."""
        now = self.get_clock().now().to_msg()
        
        # Collect encoder data
        encoder_errors = []
        encoder_warnings = []
        encoder_freqs = []
        
        for idx, listener in enumerate(self._encoder_threads):
            snap = listener.snapshot()
            self._joint_angles[idx] = self._compute_joint_angle(idx, snap)
            encoder_errors.append(snap["error"])
            encoder_warnings.append(snap["warning"])
            encoder_freqs.append(snap["freq"])
        
        # Collect RS-485 data
        rs485_snap = self._rs485_thread.snapshot()
        pot_raw = rs485_snap["pot"]
        pot_percent = pot_raw_to_percent(pot_raw, self._pot_idle, self._pot_full)
        btn1 = rs485_snap["btn1"]
        btn2 = rs485_snap["btn2"]
        
        # Compute FK
        tcp_x = tcp_y = tcp_z = 0.0
        tcp_rx = tcp_ry = tcp_rz = 0.0
        if self._fk is not None:
            try:
                tcp = self._fk.compute(self._joint_angles[:6])
                tcp_x, tcp_y, tcp_z = tcp["x"], tcp["y"], tcp["z"]
                tcp_rx, tcp_ry, tcp_rz = tcp["rx"], tcp["ry"], tcp["rz"]
            except Exception as e:
                self.get_logger().debug(f'FK computation failed: {e}')
        
        # Publish JointState message
        joint_state_msg = JointState()
        joint_state_msg.header.stamp = now
        joint_state_msg.header.frame_id = 'teachbot_base'
        joint_state_msg.name = self._joint_names[:self._dof]
        # Convert degrees to radians for standard ROS convention
        joint_state_msg.position = [math.radians(a) for a in self._joint_angles]
        joint_state_msg.velocity = []  # Not available from encoders
        joint_state_msg.effort = []    # Not available
        
        self._joint_state_pub.publish(joint_state_msg)
        
        # Publish TeachbotState message
        state_msg = TeachbotState()
        state_msg.header.stamp = now
        state_msg.header.frame_id = 'teachbot_base'
        state_msg.robot_model = self._robot_model
        
        # Joint angles in degrees
        state_msg.joint_angles_deg = self._joint_angles[:6]
        
        # TCP pose
        state_msg.tcp_x = tcp_x
        state_msg.tcp_y = tcp_y
        state_msg.tcp_z = tcp_z
        state_msg.tcp_rx = tcp_rx
        state_msg.tcp_ry = tcp_ry
        state_msg.tcp_rz = tcp_rz
        
        # Control inputs
        state_msg.pot_raw = pot_raw
        state_msg.pot_percent = pot_percent
        state_msg.btn1 = btn1
        state_msg.btn2 = btn2
        
        # Encoder status
        state_msg.encoder_errors = encoder_errors[:6]
        state_msg.encoder_warnings = encoder_warnings[:6]
        state_msg.encoder_frequencies = encoder_freqs[:6]
        
        self._state_pub.publish(state_msg)

    def destroy_node(self):
        """Clean up listeners on shutdown."""
        self.get_logger().info('Shutting down teachbot publisher...')
        
        for listener in self._encoder_threads:
            listener.stop()
            listener.join(timeout=1.0)
        
        self._rs485_thread.stop()
        self._rs485_thread.join(timeout=1.0)
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = TeachbotPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
