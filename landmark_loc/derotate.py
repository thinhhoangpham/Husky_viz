"""Gravity de-rotation: remove the robot's roll/pitch from a point cloud so a
tilted scan does not masquerade as sloped terrain. Yaw is left alone (the map
frame's heading comes from the compass yaw elsewhere).

Measured motivation: on the lake slope the robot pitches ~17 deg; at 20 m that
injects a ~6 m false height ramp, larger than the map's true 2.4 m relief. See
docs/superpowers/specs/2026-08-16-terrain-grid-localization-design.md section 2.
"""
import math

import numpy as np


def roll_pitch_from_quat(x, y, z, w):
    """(roll, pitch) in radians from a quaternion, aerospace convention.
    Roll is rotation about x, pitch about y. Yaw is intentionally not returned.
    """
    # roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis), clamped for numerical safety at the poles
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def level_rotation(roll, pitch):
    """(3,3) matrix R such that R @ p removes the given roll and pitch from a
    body-frame point p, leaving a gravity-aligned frame. Yaw is untouched.

    We undo pitch then roll: R = Rx(-roll) @ Ry(pitch).
    """
    cr, sr = math.cos(-roll), math.sin(-roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]], dtype=float)
    return rx @ ry


def derotate_cloud(points, roll, pitch):
    """Apply level_rotation to an (N,3) array. Empty passes through as (0,3)."""
    p = np.asarray(points, dtype=float)
    if len(p) == 0:
        return p.reshape(-1, 3)
    return (level_rotation(roll, pitch) @ p.T).T
