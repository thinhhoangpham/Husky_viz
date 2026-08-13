"""Fit a known rectangle footprint to a lidar cluster (robot-frame xy) by ICP.

The lidar sees only the near face(s) of an object; for an elongated bench/table
that is a near-edge line or L-shape, not the full footprint, so the visible-
points centroid is a biased estimate of the true center. We instead register the
object's KNOWN rectangle outline to the points and read off the true center and
orientation. Import-free of ROS.
"""
import math
import numpy as np

_MIN_PTS = 6
_MAX_ITERS = 20
_CONV_EPS = 1e-3   # metres; stop when the update translation is tiny


def _pca_yaw(xy):
    c = xy.mean(axis=0)
    u, s, vt = np.linalg.svd(xy - c)
    v = vt[0]                      # principal (long-axis) direction
    return math.atan2(v[1], v[0])


def _rect_segments(cx, cy, yaw, length, width):
    """Return the 4 edges as (p0, p1) segment endpoints in world (robot) frame."""
    hl, hw = length / 2.0, width / 2.0
    corners = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]], float)
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    w = (R @ corners.T).T + np.array([cx, cy])
    return [(w[0], w[1]), (w[1], w[2]), (w[2], w[3]), (w[3], w[0])]


def _closest_on_segment(p, a, b):
    ab = b - a
    t = np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12)
    t = min(1.0, max(0.0, t))
    return a + t * ab


def _nearest_outline_pts(pts, cx, cy, yaw, length, width):
    segs = _rect_segments(cx, cy, yaw, length, width)
    out = np.empty_like(pts)
    for i, p in enumerate(pts):
        best, bd = None, 1e18
        for a, b in segs:
            q = _closest_on_segment(p, a, b)
            d = np.dot(p - q, p - q)
            if d < bd:
                bd, best = d, q
        out[i] = best
    return out


def fit_rectangle(points_xy, length, width):
    pts = np.asarray(points_xy, float).reshape(-1, 2)
    if len(pts) < _MIN_PTS:
        return 0.0, 0.0, 0.0, False
    cx, cy = pts.mean(axis=0)
    yaw = _pca_yaw(pts)
    for _ in range(_MAX_ITERS):
        targets = _nearest_outline_pts(pts, cx, cy, yaw, length, width)
        # Solve rigid (Umeyama, no scale) mapping current model->targets, i.e.
        # move the rectangle so its outline sits under the points. We compute the
        # transform that best moves pts onto targets, then apply the INVERSE to the
        # rectangle pose (equivalently move the rectangle toward the points).
        src_c = pts.mean(axis=0)
        dst_c = targets.mean(axis=0)
        H = (pts - src_c).T @ (targets - dst_c)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        dyaw = math.atan2(R[1, 0], R[0, 0])
        t = dst_c - R @ src_c
        # R, t map pts -> targets. The rectangle pose must move by the INVERSE
        # of that transform (moving the model toward the points is the opposite
        # of moving the points onto the model), or the update runs backwards and
        # diverges.
        R_inv = R.T
        t_inv = -R_inv @ t
        new_c = R_inv @ np.array([cx, cy]) + t_inv
        step = math.hypot(new_c[0] - cx, new_c[1] - cy)
        cx, cy = float(new_c[0]), float(new_c[1])
        yaw += -dyaw
        if step < _CONV_EPS:
            break
    return cx, cy, yaw, True
