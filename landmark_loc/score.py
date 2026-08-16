"""Best-score classification of a lidar cluster into a park landmark type.

WHAT THIS IS, AND HOW IT DIFFERS FROM THE CASCADE
-------------------------------------------------
`landmark_loc.classify.classify_cluster` is a FIRST-MATCH-WINS cascade: types
are tested in a fixed order and the first rule that fires wins. That makes
ordering load-bearing -- "tree wins first" is a comment in that module, and it
is literally true: the tree gate runs before every other gate, so a wide canopy
can never be tested against the bin/bench/table rules at all.

This module removes ordering entirely. EVERY type scores the cluster, the
HIGHEST score wins, and if even the best score falls below that type's
confidence floor the cluster is "unknown" and is dropped. Ordering is replaced
by having to actually OUTSCORE the alternatives.

REJECTION IS STILL MANDATORY (see landmark_loc/detector.py)
-----------------------------------------------------------
A detector that always picks the best of N classes turns every ground-scatter
fragment into phantom furniture. Two things preserve rejection here:

  1. HARD VETOES, which are not scores at all (see `veto`). A vetoed cluster is
     unknown no matter how well it would have scored.
  2. PER-TYPE CONFIDENCE FLOORS (`ParkType.score_floor`). The winner must clear
     its own floor, not a global one, because the types differ wildly in how
     discriminative their evidence is.

WHY ONE FORMULA DOES NOT FIT ALL TYPES
--------------------------------------
The cascade's rules are not all the same KIND of rule, so scoring them with one
formula would misrepresent them:

  * BAND types (trash_bin_1, bench, garden_table) are genuine size bands: the
    cascade accepts them inside a window on foot_major / height / aspect. The
    honest continuous analogue is distance to the band CENTRE, normalised by
    the band half-width -- 1.0 dead centre, 0.0 at the edge (`band_score`).

  * PROFILE types (lamp, tree) are PREDICATES, not windows. "A thin post rising
    above HIGH_Z" and "a wide canopy over a trunk" each have ONE discriminating
    quantity with a one-sided threshold, and no meaningful centre: a lamp post
    is not "wrong" for being thinner than typical, and a canopy is not "wrong"
    for being wider than typical -- both are MORE convincing. So these score by
    MARGIN past the threshold (`margin_score`), which is monotone in the
    evidence, where a distance-to-centre score would perversely punish the
    clearest examples.

The per-type NUMBERS (band centres, half-widths, margins, floors) live in the
type registry, `map_tools.park_types.ParkType` -- not here. This module holds
only the machinery. Adding a type is a registry entry.
"""
import math

from landmark_loc import classify, shapefeat
from map_tools.park_types import PARK_TYPES

UNKNOWN = "unknown"

#: types this module can score, in registry order (order is NOT significant --
#: that is the entire point of this detector; it is fixed only so that an exact
#: score tie breaks deterministically instead of by dict iteration accident).
SCOREABLE = tuple(t for t in PARK_TYPES if t.score_family is not None)


def veto(cluster):
    """Return a reason string if the cluster cannot be ANY ground type, else None.

    These are HARD prerequisites, deliberately not folded into the scores:

      * too few points to measure a shape at all (shapefeat._MIN_SHAPE_PTS).
        There is nothing to be uncertain ABOUT below this -- the features are
        undefined, not weak.
      * the ground-anchoring gate (classify._GROUND_Z_MAX). This encodes
        PHYSICS: a bench, bin, table and lamp all rest ON the ground. The
        measured separation is not marginal -- real ground objects sit at
        z_min -0.48..-0.35 while floating tree-canopy phantoms sit at
        z_min 3.7..4.5 -- so softening it into a score would resurrect exactly
        the phantom furniture that gate was added to kill.

    As in the cascade, the ground veto does NOT apply to the tree path: a tree
    is identified by its canopy profile, and a cluster holding only the canopy
    is still legitimately a tree sighting. `score_cluster` therefore applies
    this veto per-type, not once up front.
    """
    pts = cluster.points
    if pts is None or len(pts) < shapefeat._MIN_SHAPE_PTS:
        return "too-few-points"
    if float(pts[:, 2].min()) >= classify._GROUND_Z_MAX:
        return "not-ground-anchored"
    return None


def band_score(value, centre, half_width):
    """1.0 at the band centre, falling linearly to 0.0 at +/- half_width.

    Clamped at 0.0 outside the band, so a quantity far outside contributes no
    negative evidence beyond "no support" -- the combination in
    `band_type_score` is a MINIMUM, so one out-of-band quantity is already
    fatal and letting it go negative would add nothing but scale distortion.
    """
    if half_width <= 0:
        return 0.0
    return max(0.0, 1.0 - abs(value - centre) / half_width)


def margin_score(value, threshold, margin, above=True):
    """Score how far `value` sits past `threshold`, saturating at `margin`.

    `above=True`  -> evidence grows as value rises above threshold (tree canopy
                     width past its 2.0 m floor).
    `above=False` -> evidence grows as value falls below threshold (lamp post
                     width under its 0.35 m cap).

    0.0 on the wrong side of the threshold, 1.0 once `margin` past it. This is
    monotone in the evidence, which is the whole reason profile types do not
    use `band_score` (see the module docstring).
    """
    if margin <= 0:
        return 0.0
    delta = (value - threshold) if above else (threshold - value)
    if delta <= 0:
        return 0.0
    return min(1.0, delta / margin)


def _features(cluster):
    """The quantities the band types are scored on: foot_major, height, aspect."""
    pts = cluster.points
    foot_major, foot_minor = shapefeat.foot_extents(pts)
    height = cluster.height
    if height is None:
        height = float(pts[:, 2].max() - pts[:, 2].min())
    return {
        "major": float(foot_major),
        "height": float(height),
        "aspect": float(foot_major / max(foot_minor, 1e-6)),
    }


def band_type_score(park_type, feats):
    """Combine a band type's per-quantity scores with a MINIMUM.

    A minimum, not a mean: the band bounds are the cascade's ACCEPTANCE
    conditions, which were conjunctive ("short AND compact AND not elongated").
    Averaging would let a superb footprint match buy forgiveness for a height
    or aspect that the cascade would have rejected outright -- e.g. the
    elongated ground fragments [1]/[3], whose foot_major and height both sit
    comfortably inside the bin band and are excluded ONLY by aspect. Under a
    mean they would score ~0.6 and become phantom bins; under a minimum their
    out-of-band aspect pins them at 0.0.
    """
    bands = park_type.score_bands
    return min(band_score(feats[q], centre, half)
               for q, (centre, half) in bands.items())


def _canopy_width(pts):
    """Widest horizontal band width at/above classify._TREE_CANOPY_MIN_Z.

    The tree PREDICATE asks whether ANY such band reaches the 2.0 m floor; the
    tree SCORE asks how far the best one exceeds it, so this returns the max
    rather than short-circuiting on the first qualifying band.
    """
    if pts is None or len(pts) == 0:
        return 0.0
    top = float(pts[:, 2].max())
    best = 0.0
    z = classify._TREE_CANOPY_MIN_Z
    while z < top:
        best = max(best, classify._band_width(pts, z, z + classify._TREE_BAND))
        z += classify._TREE_BAND
    return best


def profile_type_score(park_type, cluster):
    """Score a PROFILE type (lamp / postescable / tree) by margin on its discriminator."""
    pts = cluster.points
    if park_type.identity == "tree":
        # Margin of the widest canopy band past the 2.0 m canopy floor.
        return margin_score(_canopy_width(pts),
                            classify._TREE_CANOPY_MIN_WIDTH,
                            park_type.score_margin, above=True)
    if park_type.identity in ("lamp", "postescable"):
        # Both are "a thin post rising high" -- the thin-high-band predicate is
        # a PREREQUISITE, not a soft term: without a thin band above HIGH_Z
        # there is no post, and "post width" is then measuring something that
        # is not a post at all.
        #
        # postescable (a power-line pole) is scored with the LAMP's post-width
        # margin/cap as an APPROXIMATION: this is the only profile discriminator
        # score.py has, and the registry entry's own score_margin (0.35) is
        # copied from the lamp for exactly that reason. This affects only the
        # opt-in `score` detector seam exercised by this module -- the
        # descriptor/anchor detector added later in this plan does not call
        # score.py at all, so this approximation never reaches it. Do not read
        # this branch as a claim that a pylon and a street lamp are the same
        # shape; it is a stopgap discriminator, not a shape model.
        if not shapefeat.has_thin_high_band(pts):
            return 0.0
        return margin_score(shapefeat.post_width(pts),
                            classify._LAMP_POST_MAX,
                            park_type.score_margin, above=False)
    raise ValueError("no profile scorer for %r" % park_type.identity)


def score_cluster(cluster):
    """Score every type against the cluster: {identity: score in [0, 1]}.

    Vetoed types score 0.0 (never a negative or a None -- callers compare
    scores, and the contract that a score is always a number in [0, 1] is what
    lets the marker/diag formatting be uniform). The ground/min-points veto
    applies to every type EXCEPT tree; see `veto`.
    """
    reason = veto(cluster)
    scores = {}
    for t in SCOREABLE:
        vetoed = reason is not None and not (
            t.identity == "tree" and reason == "not-ground-anchored")
        if vetoed:
            scores[t.identity] = 0.0
        elif t.score_family == "profile":
            scores[t.identity] = float(profile_type_score(t, cluster))
        else:
            scores[t.identity] = float(band_type_score(t, _features(cluster)))
    return scores


def classify_cluster(cluster):
    """Best-score label for one cluster: (identity, confidence).

    Returns ("unknown", best_score) when the winner fails to clear its own
    `score_floor`, so the caller still learns HOW close the rejection was --
    the diagnostics and RViz labels show that number, which is what makes a
    too-tight floor visible in a live run instead of silent.
    """
    scores = score_cluster(cluster)
    if not scores:
        return UNKNOWN, 0.0
    # max() over registry order: ties break toward the earlier registry entry,
    # deterministically. Ties are not expected -- both formulas are continuous
    # -- but "deterministic" beats "whatever dict order happened to be".
    best = max(SCOREABLE, key=lambda t: scores[t.identity])
    best_score = scores[best.identity]
    if best_score < best.score_floor or best_score <= 0.0:
        return UNKNOWN, best_score
    return best.identity, best_score
