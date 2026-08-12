"""Prior-free landmark identification by constellation (shape) matching.

Each observation is identified by the robot-FRAME-INVARIANT pairwise distances
between the observed landmarks, matched against the catalog under a type
constraint. Because a distance between two points does not change when the pose
prior drifts, identification survives a badly drifted prior -- unlike the
nearest-neighbor-under-prior association it replaces (solve.associate). The prior
is consulted only to break ties between equally-good catalog constellations.

Drop-in for solve.associate: match(observations, gated_landmarks, prior_xyz, tol)
-> list of (Observation, MapLandmark).
"""
import math


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


def _grow(i, j, cat_i, cat_j, observations, obs_d, gated, tol):
    """Extend a seed (obs i->cat_i, obs j->cat_j) by assigning every other
    observation to the UNIQUE same-type catalog landmark whose distance to both
    seed landmarks matches the observed distances within tol. Observations with
    zero or more than one candidate are skipped (a partial constellation is valid
    as long as >=2 correspondences remain). Returns dict obs_index -> MapLandmark.
    """
    assign = {i: cat_i, j: cat_j}
    used = {id(cat_i), id(cat_j)}
    for k in range(len(observations)):
        if k in assign:
            continue
        ok = observations[k]
        dki = obs_d[(min(k, i), max(k, i))]
        dkj = obs_d[(min(k, j), max(k, j))]
        cand = [lm for lm in gated
                if lm.identity == ok.identity and id(lm) not in used
                and abs(_dist(lm.x, lm.y, cat_i.x, cat_i.y) - dki) <= tol
                and abs(_dist(lm.x, lm.y, cat_j.x, cat_j.y) - dkj) <= tol]
        if len(cand) == 1:
            assign[k] = cand[0]
            used.add(id(cand[0]))
    return assign


def match(observations, gated_landmarks, prior_xyz, tol):
    if len(observations) < 2 or len(gated_landmarks) < 2:
        return []
    obs_d = _obs_pair_dists(observations)
    cat_idx = _cat_pair_index(gated_landmarks)
    n = len(observations)
    best = {}
    for i in range(n):
        for j in range(i + 1, n):
            oi, oj = observations[i], observations[j]
            key = frozenset((oi.identity, oj.identity))
            for (a, b, dab) in cat_idx.get(key, ()):
                if abs(dab - obs_d[(i, j)]) > tol:
                    continue
                for cat_i, cat_j in _seed_orientations(oi, oj, a, b):
                    assign = _grow(i, j, cat_i, cat_j, observations, obs_d,
                                   gated_landmarks, tol)
                    if len(assign) > len(best):
                        best = assign
    if len(best) < 2:
        return []
    return [(observations[k], best[k]) for k in sorted(best)]
