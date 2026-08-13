"""Prior-free landmark identification by RANSAC constellation (shape) matching.

Each observation is identified by the robot-FRAME-INVARIANT pairwise distances
between the observed landmarks, matched against the catalog under a type
constraint. Because a distance between two points does not change when the pose
prior drifts, identification survives a badly drifted prior -- unlike the
nearest-neighbor-under-prior association it replaces (solve.associate).

Algorithm: RANSAC over seed pairs. Every observed pair (i, j) whose distance
matches some same-type catalog pair (a, b) within tol is a candidate seed. Each
seed determines a full rigid transform (via geom.rigid_transform_2d on just the
two seed points), which is then scored by projecting EVERY observation through
it and counting inliers (observations landing within _INLIER_TOL of a same-type,
not-yet-claimed catalog landmark). The seed with the most whole-set inliers wins.
Identification therefore depends only on the SHAPE of the observed set, never on
the prior.

The prior is used ONLY once, at the very end, as a WIDE sanity check: the pose
implied by refitting the winning inlier set is compared to prior_xyz, and the
match is rejected if that implied pose is farther than _PRIOR_SANITY (15 m) away.
This catches self-consistent-but-wrong matches (e.g. a repeated constellation
far away) without threatening drift-immunity, since realistic short-term drift
(a few meters) is well inside 15 m.

Drop-in for solve.associate: match(observations, gated_landmarks, prior_xyz, tol)
-> list of (Observation, MapLandmark).
"""
import math

from landmark_loc.geom import rigid_transform_2d

_INLIER_TOL = 0.5       # TIGHT: a correct transform lands obs within this of catalog
_MIN_INLIERS = 3        # 3 non-collinear correspondences pin pose (no reflection flip)
_PRIOR_SANITY = 15.0    # WIDE final-only guard; tolerates ~4m drift with margin


def _dist(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def _obs_pair_dists(observations):
    """Map (i, j) -> distance for observed landmarks, i < j."""
    d = {}
    n = len(observations)
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            d[(i, j)] = _dist(oi.x, oi.y, oj.x, oj.y)
    return d


def _cat_pair_index(gated):
    """Map frozenset({identity_a, identity_b}) -> list of (a, b, distance)
    catalog pairs (a, b are MapLandmark), so a typed observed pair can look up
    candidate catalog pairs of the same identity combination quickly."""
    idx = {}
    m = len(gated)
    for i in range(m):
        for j in range(i + 1, m):
            a, b = gated[i], gated[j]
            key = frozenset((a.identity, b.identity))
            idx.setdefault(key, []).append((a, b, _dist(a.x, a.y, b.x, b.y)))
    return idx


def _seed_orientations(oi, oj, a, b):
    """Yield (cat_for_i, cat_for_j) orientations of a catalog pair (a, b) that are
    type-consistent with observations (oi, oj). If oi, oj are the same type, both
    orientations are valid; otherwise only the one whose identities line up."""
    if oi.identity != oj.identity:
        if oi.identity == a.identity and oj.identity == b.identity:
            yield (a, b)
        elif oi.identity == b.identity and oj.identity == a.identity:
            yield (b, a)
        return
    # same type: both orientations are type-consistent
    yield (a, b)
    yield (b, a)


def _score_transform(tx, ty, yaw, observations, gated):
    """Project every observation through (tx,ty,yaw); assign it to the nearest
    same-type catalog landmark within _INLIER_TOL, one-to-one (a landmark is taken
    by its closest observation). Returns list[(Observation, MapLandmark)]."""
    c, s = math.cos(yaw), math.sin(yaw)
    # candidate (obs_index, landmark, dist) within tol
    cand = []
    for k, o in enumerate(observations):
        mx = tx + c * o.x - s * o.y
        my = ty + s * o.x + c * o.y
        best, bd = None, _INLIER_TOL
        for lm in gated:
            if lm.identity != o.identity:
                continue
            d = math.hypot(lm.x - mx, lm.y - my)
            if d <= bd:
                best, bd = lm, d
        if best is not None:
            cand.append((k, best, bd))
    # one-to-one: each landmark kept by its closest observation
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
    cat_idx = _cat_pair_index(gated_landmarks)
    n = len(observations)
    best_inliers = []
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            key = frozenset((oi.identity, oj.identity))
            for (a, b, dab) in cat_idx.get(key, ()):
                if abs(dab - obs_d[(i, j)]) > tol:       # seed_tol on pair distance
                    continue
                for cat_i, cat_j in _seed_orientations(oi, oj, a, b):
                    tx, ty, yaw, _ = rigid_transform_2d(
                        [[oi.x, oi.y], [oj.x, oj.y]],
                        [[cat_i.x, cat_i.y], [cat_j.x, cat_j.y]])
                    inliers = _score_transform(tx, ty, yaw,
                                               observations, gated_landmarks)
                    if len(inliers) > len(best_inliers):
                        best_inliers = inliers
    if len(best_inliers) < _MIN_INLIERS:
        return []
    # WIDE prior sanity on the refit pose
    src = [[o.x, o.y] for o, _ in best_inliers]
    dst = [[lm.x, lm.y] for _, lm in best_inliers]
    px, py, _, _ = rigid_transform_2d(src, dst)
    if math.hypot(px - prior_xyz[0], py - prior_xyz[1]) > _PRIOR_SANITY:
        return []
    return best_inliers
