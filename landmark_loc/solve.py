"""Associate observed landmarks to catalog landmarks and solve the robot pose.

`solve_pose` identifies observed landmarks via RANSAC constellation (shape)
matching (`constellation.match`), which uses frame-invariant pairwise
distances between observed landmarks rather than a nearest-neighbor search
under the pose prior. This makes identification robust to several metres of
prior/odom drift -- the prior is only used as a wide final sanity check inside
`constellation.match`. `associate` (plain nearest-neighbor under the prior)
is kept for legacy/characterization tests but is no longer used by
`solve_pose`. The pose is the 2D rigid transform (Umeyama/Kabsch) mapping
observed (robot-frame) points onto their matched (map-frame) points; that
transform IS the robot's map pose. A fit with < 3 correspondences or RMS
residual above the gate is rejected (returns None) so a bad scan cannot
corrupt the downstream EKF. Fewer than 3 correspondences is geometrically
ambiguous under reflection (a 2-point fit can yield a confidently-wrong,
flipped pose with low residual), so 3 is the floor that removes that
ambiguity.
"""
import math
import numpy as np

from landmark_loc import constellation
from landmark_loc import constellation_typeless
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
                max_prior_dist=5.0, matcher="typed"):
    if matcher not in ("typed", "typeless"):
        import warnings
        warnings.warn("solve_pose: unknown matcher %r, defaulting to 'typed'" % (matcher,))
        matcher = "typed"
    mod = constellation_typeless if matcher == "typeless" else constellation
    pairs = mod.match(observations, gated_landmarks, prior_xyz, dist_gate,
                       max_prior_dist)
    if len(pairs) < 3:
        return None
    src = np.array([[o.x, o.y] for o, _ in pairs])
    dst = np.array([[lm.x, lm.y] for _, lm in pairs])
    x, y, yaw, rms = rigid_transform_2d(src, dst)
    if rms > residual_gate:
        return None
    return (x, y, yaw, rms, len(pairs))
