# kinematics.py
"""
Forward kinematics for 6-DOF robot arms using DH parameters.

Coordinate system follows ROS REP-103 standard (right-handed):
- X+ = forward (direction robot faces when J1=0)
- Y+ = left
- Z+ = up

This matches RViz and standard ROS tools.
"""

from __future__ import annotations

import os
from typing import Dict, List, Optional
import numpy as np
import yaml


def dh_matrix(theta0: float, joint_angle: float, d: float, a: float, alpha: float) -> np.ndarray:
    """
    Compute the 4x4 DH transformation matrix.
    
    Args:
        theta0: Joint angle offset (radians)
        joint_angle: Joint angle (radians)
        d: Link offset along Z
        a: Link length along X
        alpha: Link twist (radians)
    
    Returns:
        4x4 transformation matrix
    """
    theta = joint_angle + theta0
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    
    return np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,   sa,       ca,      d     ],
        [0,   0,        0,       1     ]
    ])


def calc_fk_robot_200_200(joint_angles: List[float]) -> Dict[str, float]:
    """
    Forward kinematics for Robot_200_200_mm (TOS teachbot).
    
    Uses ROS standard coordinate convention:
    - X+ = forward (away from operator)
    - Y+ = left
    - Z+ = up
    
    Robot dimensions:
    - Upper arm (a2): 200mm
    - Forearm (d4): 200mm
    - Flange (d6): 60mm
    
    Args:
        joint_angles: List of 6 joint angles in degrees
        
    Returns:
        Dict with x, y, z (mm), w, p, r (degrees)
    """
    j1 = np.deg2rad(joint_angles[0])
    j2 = np.deg2rad(joint_angles[1])
    j3 = np.deg2rad(joint_angles[2])
    j4 = np.deg2rad(joint_angles[3])
    j5 = np.deg2rad(joint_angles[4])
    j6 = np.deg2rad(joint_angles[5])
    
    # DH parameters for Robot_200_200_mm
    matrix1 = dh_matrix(0, j1, 0, 0, np.pi/2)
    matrix2 = dh_matrix(np.pi/2, -j2, 0, 200, 0)
    matrix3 = dh_matrix(0, j3, 0, 0, -np.pi/2)
    matrix4 = dh_matrix(0, j4, -200, 0, np.pi/2)
    matrix5 = dh_matrix(0, j5, 0, 0, -np.pi/2)
    matrix6 = dh_matrix(0, j6, -60, 0, np.pi)
    
    T = matrix1 @ matrix2 @ matrix3 @ matrix4 @ matrix5 @ matrix6
    
    # ROS standard convention: use DH output directly
    # X+ = forward, Y+ = left, Z+ = up
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    
    # Extract orientation (Roll, Pitch, Yaw) - ZYX Euler angles
    # This gives orientation relative to base frame
    w = np.rad2deg(np.arctan2(T[2, 1], T[2, 2]))   # Roll (rotation about X)
    p = np.rad2deg(-np.arcsin(np.clip(T[2, 0], -1.0, 1.0)))  # Pitch (rotation about Y)
    r = np.rad2deg(np.arctan2(T[1, 0], T[0, 0]))   # Yaw (rotation about Z)
    
    return {"x": x, "y": y, "z": z, "w": w, "p": p, "r": r}


def calc_fk_cobot_200_200(joint_angles: List[float]) -> Dict[str, float]:
    """Forward kinematics for Cobot_200_200_mm (UR-style cobot)."""
    j1 = np.deg2rad(joint_angles[0])
    j2 = np.deg2rad(joint_angles[1])
    j3 = np.deg2rad(joint_angles[2])
    j4 = np.deg2rad(joint_angles[3])
    j5 = np.deg2rad(joint_angles[4])
    j6 = np.deg2rad(joint_angles[5])
    
    matrix1 = dh_matrix(0, j1, 80, 0, np.pi/2)
    matrix2 = dh_matrix(np.pi/2, -j2, 0, 200, 0)
    matrix3 = dh_matrix(0, j3, 0, 200, 0)
    matrix4 = dh_matrix(0, j4, 60, 0, np.pi/2)
    matrix5 = dh_matrix(0, j5, 60, 0, -np.pi/2)
    matrix6 = dh_matrix(0, j6, 40, 0, 0)
    
    T = matrix1 @ matrix2 @ matrix3 @ matrix4 @ matrix5 @ matrix6
    
    # ROS standard convention
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    w = np.rad2deg(np.arctan2(T[2, 1], T[2, 2]))
    p = np.rad2deg(-np.arcsin(np.clip(T[2, 0], -1.0, 1.0)))
    r = np.rad2deg(np.arctan2(T[1, 0], T[0, 0]))
    
    return {"x": x, "y": y, "z": z, "w": w, "p": p, "r": r}


# Model Registry
KINEMATICS_FUNCTIONS = {
    "Robot_200_200_mm": calc_fk_robot_200_200,
    "Cobot_200_200_mm": calc_fk_cobot_200_200,
}


def get_forward_kinematics_function(model_name: str):
    """Get the forward kinematics function for a model."""
    if model_name not in KINEMATICS_FUNCTIONS:
        available = ", ".join(KINEMATICS_FUNCTIONS.keys())
        raise KeyError(f"Unknown model '{model_name}'. Available: {available}")
    return KINEMATICS_FUNCTIONS[model_name]


def compute_fk(model_name: str, joint_angles: List[float]) -> Dict[str, float]:
    """Compute forward kinematics for a given model and joint angles."""
    fk_func = get_forward_kinematics_function(model_name)
    return fk_func(joint_angles)


class ForwardKinematics:
    """Wrapper class for forward kinematics computation."""
    
    def __init__(self, model_name: str):
        self.model_name = model_name
        self._fk_func = get_forward_kinematics_function(model_name)
    
    def compute(self, joint_angles_deg: List[float]) -> Dict[str, float]:
        """Compute FK and return position/orientation."""
        result = self._fk_func(joint_angles_deg)
        return {
            "x": result["x"],
            "y": result["y"],
            "z": result["z"],
            "rx": result["w"],
            "ry": result["p"],
            "rz": result["r"]
        }


def get_forward_kinematics(model_name: str) -> ForwardKinematics:
    """Get a ForwardKinematics instance for the given model."""
    return ForwardKinematics(model_name)
