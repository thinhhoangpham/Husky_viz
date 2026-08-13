# landmark_loc/tests/test_association_duplicate.py
"""Characterization test for the deferred one-to-one association gap.

`solve.associate` matches each observation independently to its nearest
same-identity map landmark, with NO one-to-one guard. Two different observations
can therefore associate to the SAME map landmark ("duplicate association"). When
that happened at the `solve.solve_pose` level, it could return a result with
n>=2 that was really pinned on only ONE distinct landmark -- a phantom,
potentially confidently-wrong fix that still cleared the n>=2 and residual
gates.

That solve_pose-level defect no longer applies: `solve_pose` now routes
through `constellation.match` (RANSAC constellation matching), which is
one-to-one BY CONSTRUCTION -- `_score_transform` keeps, for each candidate map
landmark, only its closest observation, so two observations can never both
count as correspondences to the same map landmark. With only two total
landmarks in this scenario, `constellation.match`'s 3-correspondence floor
(`_MIN_INLIERS`) also can't be met, so `solve_pose` returns None regardless.
The `associate()` characterization below remains as documentation of that
raw (non-deduped) nearest-neighbor function's behavior; it is no longer used
by `solve_pose`.

These tests are TEST-ONLY; no production code is modified by them.
"""
import math

from landmark_loc.solve import associate, solve_pose
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


# ---------------------------------------------------------------------------
# Shared geometry (see the block comment in the first test for the full "why").
#
# Two SAME-identity benches, a tight 4 m cluster, placed 20 m in front of the
# robot. The robot truly sits at the world origin looking down +x (yaw 0) and
# observes each bench from that TRUE pose. We then hand `associate` a prior whose
# YAW is wrong by 0.15 rad. Because a yaw error rotates every observation about
# the robot, a landmark at range R swings by ~R*err metres -- here ~20*0.15 = 3 m,
# which exceeds the 4 m bench separation enough to push BOTH map-projected
# observations onto the nearer single bench (bench_2). A uniform position shift
# would NOT do this: it moves both observations by the same vector and cancels
# out. Angular error is the physical trigger.
# ---------------------------------------------------------------------------
_BENCH_1 = MapLandmark("bench_1", "bench", 20.0, 0.0)
_BENCH_2 = MapLandmark("bench_2", "bench", 20.0, 4.0)
_LANDMARKS = [_BENCH_1, _BENCH_2]

_TRUE_POSE = (0.0, 0.0, 0.0)      # robot's real map pose (test knowledge only)
_YAW_ERROR = 0.15                 # rad of yaw error injected into the prior
_PRIOR = (0.0, 0.0, _YAW_ERROR)   # same position, wrong heading
_DIST_GATE = 10.0                 # generous gate; both benches are reachable


def _observe_from_true_pose(lm, true_xyz):
    """Project a map landmark into the robot frame at the TRUE pose (test only).

    This is what the robot's lidar would actually report -- it has no yaw error.
    The yaw error lives only in the PRIOR handed to associate(), exactly as in
    the field where the EKF prior can be stale/wrong while the sensor is honest.
    """
    x, y, yaw = true_xyz
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = lm.x - x, lm.y - y
    return Observation(lm.identity, c * dx - s * dy, s * dx + c * dy)


def _observations():
    return [_observe_from_true_pose(_BENCH_1, _TRUE_POSE),
            _observe_from_true_pose(_BENCH_2, _TRUE_POSE)]


def test_associate_can_produce_duplicate_map_landmark():
    """associate() maps two distinct observations onto the SAME MapLandmark.

    Geometry / numbers used and WHY they trigger it:
      - Two benches 4 m apart at (20, 0) and (20, 4): a tight, same-identity
        cluster like the real park's clustered benches.
      - Robot truly at (0, 0, yaw=0); both benches observed honestly from there.
      - Prior yaw wrong by +0.15 rad. Rotating the two honest observations by
        that error about the robot swings each ~20 m * 0.15 rad ~= 3 m in the
        +y direction. Bench_1's projection lands near y~3 and bench_2's near
        y~7; both are now closer to bench_2 (y=4) than to bench_1 (y=0), so the
        independent nearest-neighbour search binds BOTH to bench_2.
      - dist_gate = 10 m is loose enough that neither projection is rejected.
    A uniform (translation-only) prior error was tried mentally and does not
    work: it shifts both observations by the same vector, preserving their
    relative order, so each still keeps its own nearest bench.
    """
    obs = _observations()
    pairs = associate(obs, _LANDMARKS, _PRIOR, _DIST_GATE)

    # Every observation associated (both within the gate).
    assert len(pairs) == 2, f"expected both observations gated in, got {pairs}"

    matched_names = [lm.name for _, lm in pairs]
    matched_ids = {id(lm) for _, lm in pairs}

    # The defect: two observation->landmark pairs share ONE map landmark.
    assert len(set(matched_names)) < len(matched_names), (
        f"expected a duplicate association, got distinct matches {matched_names}"
    )
    assert len(matched_ids) == 1, (
        f"expected both pairs to point at the same MapLandmark object, "
        f"got {matched_names}"
    )
    # Concretely: both bind to bench_2.
    assert matched_names == ["bench_2", "bench_2"]


def test_duplicate_association_dropped_by_dedup_not_phantom_fit():
    """solve_pose now routes through constellation.match, which is one-to-one
    BY CONSTRUCTION, so the two-benches-onto-one duplicate characterized above
    (a defect of the raw associate() function) can never reach solve_pose as a
    phantom fit pinned on one landmark. With only two same-identity benches in
    this scene, constellation.match also can't clear its own 3-correspondence
    floor (_MIN_INLIERS), so solve_pose returns None either way."""
    obs = _observations()
    for rg in (1.0, 5.0):
        out = solve_pose(obs, _LANDMARKS, _PRIOR, dist_gate=_DIST_GATE, residual_gate=rg)
        assert out is None, (
            f"expected dedup to drop below the 3-correspondence floor, got {out}"
        )
