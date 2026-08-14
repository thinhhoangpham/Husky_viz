"""Types-stripped control: the RANSAC constellation matcher with the landmark
IDENTITY constraint removed, for the identity-vs-geometry experiment.

Identical to landmark_loc.constellation in every respect EXCEPT that it ignores
landmark identity: any observed pair may seed against any distance-compatible
catalog pair, and an observation may be an inlier of the nearest catalog landmark
of ANY type. The yaw constraint (geometric, not semantic) is kept, so IDENTITY is
the single difference vs. the typed matcher. See
docs/superpowers/specs/2026-08-13-typeless-matcher-control.md.

NOT used in the production localization path -- experiment harness only.
"""
import math

from landmark_loc.geom import rigid_transform_2d

# Constants identical to landmark_loc.constellation.
_INLIER_TOL = 0.5
_MIN_INLIERS = 3
_PRIOR_SANITY = 15.0
_YAW_TOL = 0.35


def _dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _obs_pair_dists(observations):
    d = {}
    n = len(observations)
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            d[(i, j)] = _dist(oi.x, oi.y, oj.x, oj.y)
    return d


def _cat_pairs(gated):
    """ALL catalog pairs (a, b, distance), NO type key. (Coupling point 1 removed.)"""
    out = []
    m = len(gated)
    for i in range(m):
        for j in range(i + 1, m):
            a, b = gated[i], gated[j]
            out.append((a, b, _dist(a.x, a.y, b.x, b.y)))
    return out


def _yaw_diff_ok(oi, oj, ci, cj):
    """Identical to the typed matcher: yaw is geometric, kept in both arms."""
    ys = (oi.yaw, oj.yaw, ci.yaw, cj.yaw)
    if any(y is None for y in ys):
        return True
    d_obs = (oi.yaw - oj.yaw) % math.pi
    d_cat = (ci.yaw - cj.yaw) % math.pi
    dd = abs(d_obs - d_cat) % math.pi
    return min(dd, math.pi - dd) <= _YAW_TOL


def _score_transform(tx, ty, yaw, observations, gated):
    """Project every observation; assign to the nearest catalog landmark within
    _INLIER_TOL of ANY type (coupling point 4 removed). Yaw check kept."""
    c, s = math.cos(yaw), math.sin(yaw)
    cand = []
    for k, o in enumerate(observations):
        mx = tx + c * o.x - s * o.y
        my = ty + s * o.x + c * o.y
        best, bd = None, _INLIER_TOL
        for lm in gated:
            # NOTE: no `lm.identity != o.identity` filter -- types ignored.
            if o.yaw is not None and lm.yaw is not None:
                map_yaw = o.yaw + yaw
                dd = abs(map_yaw - lm.yaw) % math.pi
                ang = min(dd, math.pi - dd)
                if ang > _YAW_TOL:
                    continue
            d = math.hypot(lm.x - mx, lm.y - my)
            if d <= bd:
                best, bd = lm, d
        if best is not None:
            cand.append((k, best, bd))
    best_for_lm = {}
    for k, lm, d in cand:
        key = id(lm)
        if key not in best_for_lm or d < best_for_lm[key][2]:
            best_for_lm[key] = (k, lm, d)
    return [(observations[k], lm) for (k, lm, d) in best_for_lm.values()]


def match(observations, gated_landmarks, prior_xyz, tol, max_prior_dist=5.0):
    if len(observations) < _MIN_INLIERS or len(gated_landmarks) < _MIN_INLIERS:
        return []
    obs_d = _obs_pair_dists(observations)
    cat_pairs = _cat_pairs(gated_landmarks)
    n = len(observations)
    best_inliers = []
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            dij = obs_d[(i, j)]
            for (a, b, dab) in cat_pairs:                 # coupling point 2: no type key
                if abs(dab - dij) > tol:
                    continue
                for cat_i, cat_j in ((a, b), (b, a)):     # coupling point 3: always both
                    if not _yaw_diff_ok(oi, oj, cat_i, cat_j):
                        continue
                    tx, ty, yaw, _ = rigid_transform_2d(
                        [[oi.x, oi.y], [oj.x, oj.y]],
                        [[cat_i.x, cat_i.y], [cat_j.x, cat_j.y]])
                    inliers = _score_transform(tx, ty, yaw,
                                               observations, gated_landmarks)
                    if len(inliers) > len(best_inliers):
                        best_inliers = inliers
    if len(best_inliers) < _MIN_INLIERS:
        return []
    src = [[o.x, o.y] for o, _ in best_inliers]
    dst = [[lm.x, lm.y] for _, lm in best_inliers]
    px, py, _, _ = rigid_transform_2d(src, dst)
    if math.hypot(px - prior_xyz[0], py - prior_xyz[1]) > _PRIOR_SANITY:
        return []
    return best_inliers
