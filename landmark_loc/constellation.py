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


def _centroid(assign):
    xs = [lm.x for lm in assign.values()]
    ys = [lm.y for lm in assign.values()]
    return (sum(xs) / len(xs), sum(ys) / len(ys))


def _prior_dist(assign, prior_xyz):
    cx, cy = _centroid(assign)
    return math.hypot(cx - prior_xyz[0], cy - prior_xyz[1])


def _implied_heading(observations, assign):
    """Compute implied robot heading from observations and their assigned landmarks.

    For two or more observations, try to find a consistent heading. Returns the
    heading implied by the first two assigned observations, or None if inconsistent.
    """
    indices = sorted(assign.keys())
    if len(indices) < 2:
        return None

    # Use the first two observations to compute heading
    i, j = indices[0], indices[1]
    oi, oj = observations[i], observations[j]
    lm_i, lm_j = assign[i], assign[j]

    # Delta in robot frame
    delta_rx = oj.x - oi.x
    delta_ry = oj.y - oi.y

    # Delta in world frame
    delta_wx = lm_j.x - lm_i.x
    delta_wy = lm_j.y - lm_i.y

    # If deltas are too small, can't determine heading
    if math.hypot(delta_rx, delta_ry) < 1e-6:
        return None

    # Implied heading: angle from robot-frame delta to world-frame delta
    theta = math.atan2(delta_wy, delta_wx) - math.atan2(delta_ry, delta_rx)

    # Normalize to [-π, π]
    while theta > math.pi:
        theta -= 2 * math.pi
    while theta < -math.pi:
        theta += 2 * math.pi

    return theta


def match(observations, gated_landmarks, prior_xyz, tol):
    if len(observations) < 2 or len(gated_landmarks) < 2:
        return []
    obs_d = _obs_pair_dists(observations)
    cat_idx = _cat_pair_index(gated_landmarks)
    n = len(observations)
    candidates = []
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
                    candidates.append(assign)
    candidates = [a for a in candidates if len(a) >= 2]
    if not candidates:
        return []
    # Filter out assignments with heading inconsistent with prior (within π/2 threshold)
    max_heading_diff = math.pi / 2
    filtered = []
    for a in candidates:
        implied = _implied_heading(observations, a)
        if implied is None:
            # Can't determine heading (too few/small observations), accept tentatively
            filtered.append(a)
        else:
            # Check heading difference from prior
            prior_heading = prior_xyz[2]
            diff = abs(implied - prior_heading)
            # Normalize difference to [0, π]
            while diff > math.pi:
                diff = 2 * math.pi - diff
            if diff <= max_heading_diff:
                filtered.append(a)
    if not filtered:
        return []
    candidates = filtered
    best_size = max(len(a) for a in candidates)
    top = [a for a in candidates if len(a) == best_size]
    # dedupe identical assignments (same obj set) so a true single winner isn't
    # treated as a tie
    uniq = []
    for a in top:
        sig = frozenset((k, id(v)) for k, v in a.items())
        if sig not in {frozenset((k, id(v)) for k, v in u.items()) for u in uniq}:
            uniq.append(a)
    chosen = min(uniq, key=lambda a: _prior_dist(a, prior_xyz))
    return [(observations[k], chosen[k]) for k in sorted(chosen)]
