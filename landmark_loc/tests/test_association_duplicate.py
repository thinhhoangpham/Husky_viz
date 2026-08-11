# landmark_loc/tests/test_association_duplicate.py
"""Characterization test for the deferred one-to-one association gap.

`solve.associate` matches each observation independently to its nearest
same-identity map landmark, with NO one-to-one guard. Two different observations
can therefore associate to the SAME map landmark ("duplicate association"). When
that happens, `solve.solve_pose` can return a result with n>=2 that is really
pinned on only ONE distinct landmark -- a phantom, potentially confidently-wrong
fix that still clears the n>=2 and residual gates.

These tests turn that hypothetical into a concrete, reproducible demonstration.
They are TEST-ONLY; no production code is modified. The guard remains deferred.
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


def test_duplicate_association_can_pass_gates():
    """The duplicate slips through solve_pose's n>=2 + residual gates.

    Same inputs as above. Because both observations bind to bench_2, solve_pose
    fits a rigid transform whose destination points are bench_2 TWICE -- a single
    distinct landmark. With a lenient residual_gate it still returns a pose with
    n == 2, i.e. it looks like a two-landmark fix but rests on one point.

    Measured outcome (documented here so the demonstration is self-contained):
      - The returned pose is (x=0.0, y=2.0, yaw=0.0), rms=2.0, n=2.
      - The TRUE pose is (0, 0, 0), so the phantom fix is 2.0 m off in y while
        reporting n=2 and clearing the gate.
      - residual_gate must be >= the rms (2.0) for it to pass: a tight gate
        (e.g. 1.0) rejects it (rms 2.0 > 1.0 -> None). So the gap is only
        exploitable when the residual gate is loose relative to the cluster
        spacing -- which is exactly the regime a wide tolerance would create.
    """
    obs = _observations()

    # A tight residual gate DOES catch this particular phantom.
    strict = solve_pose(obs, _LANDMARKS, _PRIOR,
                        dist_gate=_DIST_GATE, residual_gate=1.0)
    assert strict is None, "tight residual gate should reject this phantom"

    # A lenient residual gate lets the phantom through with n>=2.
    lenient = solve_pose(obs, _LANDMARKS, _PRIOR,
                        dist_gate=_DIST_GATE, residual_gate=5.0)
    assert lenient is not None, "lenient gate: expected the phantom to pass"
    x, y, yaw, rms, n = lenient

    # It reports two correspondences despite resting on ONE distinct landmark.
    assert n == 2

    # And it is wrong: ~2 m off the true pose (0, 0, 0) in y.
    true_x, true_y, _ = _TRUE_POSE
    assert abs(y - true_y) > 1.0, (
        f"expected a confidently-wrong fix; got y={y} vs true {true_y}"
    )
    assert math.hypot(x - true_x, y - true_y) > 1.0
