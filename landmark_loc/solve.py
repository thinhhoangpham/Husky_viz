"""Associate observed landmarks to catalog landmarks and solve the robot pose.

EXPERIMENTAL: `solve_pose` currently uses odom-guess nearest-neighbor
association (`associate`) with a one-to-one dedup guard, not constellation
matching. The odom-based prior is assumed to be only a few metres off, so each
observed landmark's correct map landmark is simply the nearest same-type
landmark under the prior; `constellation.match` is left intact (and still
covered by its own tests) so this can be flipped back or combined later. The
pose is the 2D rigid transform (Umeyama/Kabsch) mapping observed (robot-frame)
points onto their matched (map-frame) points; that transform IS the robot's
map pose. A fit with < 3 correspondences or RMS residual above the gate is
rejected (returns None) so a bad scan cannot corrupt the downstream EKF. Fewer
than 3 correspondences is geometrically ambiguous under reflection (a 2-point
fit can yield a confidently-wrong, flipped pose with low residual), so 3 is
the floor that removes that ambiguity.
"""
import math
import numpy as np

from landmark_loc import constellation
from landmark_loc.geom import rigid_transform_2d


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


def _dedupe_one_to_one(pairs, observations, prior_xyz):
    """If two observations claim the same map landmark, keep only the closer one
    (smaller obs->landmark distance in the map frame). Returns filtered pairs."""
    best_for_lm = {}   # id(landmark) -> (dist, (obs, lm))
    for o, lm in pairs:
        mx, my = _to_map(o, prior_xyz)
        d = math.hypot(lm.x - mx, lm.y - my)
        key = id(lm)
        if key not in best_for_lm or d < best_for_lm[key][0]:
            best_for_lm[key] = (d, (o, lm))
    return [v[1] for v in best_for_lm.values()]


def solve_pose(observations, gated_landmarks, prior_xyz, dist_gate, residual_gate,
                max_prior_dist=5.0):
    pairs = associate(observations, gated_landmarks, prior_xyz, dist_gate)
    pairs = _dedupe_one_to_one(pairs, observations, prior_xyz)
    if len(pairs) < 3:
        return None
    src = np.array([[o.x, o.y] for o, _ in pairs])
    dst = np.array([[lm.x, lm.y] for _, lm in pairs])
    x, y, yaw, rms = rigid_transform_2d(src, dst)
    if rms > residual_gate:
        return None
    return (x, y, yaw, rms, len(pairs))
