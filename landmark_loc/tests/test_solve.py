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


def test_solve_pose_matcher_recovers_under_wrong_prior():
    # "wrong" but within realistic short-term odom drift of the true
    # constellation centroid (~4.67, 1.0) -- i.e. within max_prior_dist
    # (default 5m) of the correct constellation, per the primary prior
    # filter in constellation.match. See test_far_constellation_rejected_by_prior_filter
    # for the case where the prior is farther than max_prior_dist.
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    true = (2.0, -1.0, 0.5)
    obs = _obs_from_truth(true, lms)
    wrong_prior = (6.0, 3.0, 1.2)  # off from truth, but realistically close
    out = solve.solve_pose(obs, lms, prior_xyz=wrong_prior,
                           dist_gate=0.3, residual_gate=0.5)
    assert out is not None
    x, y, yaw, rms, n = out
    assert n == 3 and rms < 1e-6
    assert abs(x - 2.0) < 1e-6 and abs(y + 1.0) < 1e-6


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
