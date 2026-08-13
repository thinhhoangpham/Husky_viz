# landmark_loc/tests/test_solve.py
import math
import numpy as np
from landmark_loc import solve
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def _obs_from_truth(true_xyz, landmarks):
    """Project map landmarks into the robot frame at the TRUE pose (test only)."""
    x, y, yaw = true_xyz
    c, s = math.cos(-yaw), math.sin(-yaw)
    obs = []
    for lm in landmarks:
        dx, dy = lm.x - x, lm.y - y
        obs.append(Observation(lm.identity, c * dx - s * dy, s * dx + c * dy))
    return obs


def test_rigid_transform_recovers_known_pose():
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    true = (2.0, -1.0, 0.5)
    obs = _obs_from_truth(true, lms)
    src = np.array([[o.x, o.y] for o in obs])
    dst = np.array([[l.x, l.y] for l in lms])
    x, y, yaw, rms = solve.rigid_transform_2d(src, dst)
    assert abs(x - 2.0) < 1e-6 and abs(y + 1.0) < 1e-6
    assert abs((yaw - 0.5 + math.pi) % (2 * math.pi) - math.pi) < 1e-6
    assert rms < 1e-6


def test_solve_pose_rejects_when_too_few_matches():
    lms = [MapLandmark("a", "bench", 5.0, 1.0)]
    obs = _obs_from_truth((0, 0, 0), lms)
    out = solve.solve_pose(obs, lms, prior_xyz=(0, 0, 0),
                          dist_gate=1.0, residual_gate=0.5)
    assert out is None  # only 1 correspondence


def test_solve_pose_rejects_high_residual():
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    obs = _obs_from_truth((2.0, -1.0, 0.5), lms)
    obs[0].x += 3.0  # corrupt one observation badly
    out = solve.solve_pose(obs, lms, prior_xyz=(2.0, -1.0, 0.5),
                          dist_gate=5.0, residual_gate=0.3)
    assert out is None  # residual too high


def test_solve_pose_recovers_under_small_prior_error():
    # The nearest-neighbor matcher (associate) relies on the prior being only a
    # few metres off, so each observation's map-frame projection still lands
    # closest to its OWN true landmark. Here the prior is off by (0.3, 0.2, 0.05)
    # from truth -- realistic short-term odom drift -- and the fit should still
    # recover the true pose exactly once the rigid transform is solved.
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    true = (2.0, -1.0, 0.5)
    obs = _obs_from_truth(true, lms)
    small_wrong_prior = (2.3, -0.8, 0.55)  # a few decimeters/radians off
    out = solve.solve_pose(obs, lms, prior_xyz=small_wrong_prior,
                           dist_gate=1.0, residual_gate=0.5)
    assert out is not None
    x, y, yaw, rms, n = out
    assert n == 3 and rms < 1e-6
    assert abs(x - 2.0) < 1e-6 and abs(y + 1.0) < 1e-6


def test_solve_pose_fails_under_large_prior_error():
    # With a badly wrong prior (large odom drift), nearest-neighbor association
    # under the prior projects observations far from their true landmarks, so
    # the tight dist_gate rejects them and solve_pose returns None. This is the
    # tradeoff versus constellation matching: association now depends on the
    # prior being accurate to within dist_gate, not just within max_prior_dist.
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    true = (2.0, -1.0, 0.5)
    obs = _obs_from_truth(true, lms)
    wrong_prior = (6.0, 3.0, 1.2)  # far off from truth
    out = solve.solve_pose(obs, lms, prior_xyz=wrong_prior,
                           dist_gate=0.3, residual_gate=0.5)
    assert out is None


def test_solve_pose_accepts_clean_fit():
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    obs = _obs_from_truth((2.0, -1.0, 0.5), lms)
    out = solve.solve_pose(obs, lms, prior_xyz=(2.0, -1.0, 0.5),
                          dist_gate=1.0, residual_gate=0.3)
    assert out is not None
    x, y, yaw, rms, n = out
    assert abs(x - 2.0) < 0.05 and abs(y + 1.0) < 0.05 and n == 3
