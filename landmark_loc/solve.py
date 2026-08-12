"""Associate observed landmarks to catalog landmarks and solve the robot pose.

Association is now constellation-based (see `constellation.match`): observations
are identified by their prior-invariant pairwise geometry rather than by
nearest-neighbor-under-prior, so it survives a badly drifted prior. The pose is
the 2D rigid transform (Umeyama/Kabsch) mapping observed (robot-frame) points
onto their matched (map-frame) points; that transform IS the robot's map pose.
A fit with < 2 correspondences or RMS residual above the gate is rejected
(returns None) so a bad scan cannot corrupt the downstream EKF.
"""
import math
import numpy as np

from landmark_loc import constellation


def _to_map(o, prior_xyz):
    x, y, yaw = prior_xyz
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * o.x - s * o.y, y + s * o.x + c * o.y)


def associate(observations, gated_landmarks, prior_xyz, dist_gate):
    pairs = []
    for o in observations:
        mx, my = _to_map(o, prior_xyz)
        best, best_d = None, dist_gate
        for lm in gated_landmarks:
            if lm.identity != o.identity:
                continue
            d = math.hypot(lm.x - mx, lm.y - my)
            if d <= best_d:
                best, best_d = lm, d
        if best is not None:
            pairs.append((o, best))
    return pairs


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


def solve_pose(observations, gated_landmarks, prior_xyz, dist_gate, residual_gate,
                max_prior_dist=5.0):
    pairs = constellation.match(observations, gated_landmarks, prior_xyz, dist_gate,
                                 max_prior_dist)
    if len(pairs) < 2:
        return None
    src = np.array([[o.x, o.y] for o, _ in pairs])
    dst = np.array([[lm.x, lm.y] for _, lm in pairs])
    x, y, yaw, rms = rigid_transform_2d(src, dst)
    if rms > residual_gate:
        return None
    return (x, y, yaw, rms, len(pairs))
