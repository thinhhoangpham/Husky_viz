"""Dependency-free 2D geometry: rigid (Kabsch/Umeyama) transform.

Shared by solve.py (final N-point pose refit) and constellation.py (seed
transforms). Kept import-free so both can use it without a cycle.
"""
import math
import numpy as np


def rigid_transform_2d(src_xy, dst_xy):
    src = np.asarray(src_xy, float)
    dst = np.asarray(dst_xy, float)
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    s0, d0 = src - cs, dst - cd
    H = s0.T @ d0
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:      # reflection guard
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    yaw = math.atan2(R[1, 0], R[0, 0])
    t = cd - R @ cs
    resid = (R @ s0.T).T + cd - dst
    rms = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1)))) if len(dst) else 0.0
    return float(t[0]), float(t[1]), yaw, rms
