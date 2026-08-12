import math
from landmark_loc import constellation
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def _observe_from_true_pose(lm, true_xyz):
    """Project a map landmark into the robot frame at the TRUE pose (test only)."""
    x, y, yaw = true_xyz
    c, s = math.cos(-yaw), math.sin(-yaw)
    dx, dy = lm.x - x, lm.y - y
    return Observation(lm.identity, c * dx - s * dy, s * dx + c * dy)


_LMS = [
    MapLandmark("bench_1", "bench", 5.0, 1.0),
    MapLandmark("lamp_1", "lamp", 6.0, -2.0),
    MapLandmark("table_1", "garden_table", 3.0, 4.0),
]


def test_clean_three_landmark_match():
    true = (2.0, -1.0, 0.5)
    obs = [_observe_from_true_pose(lm, true) for lm in _LMS]
    pairs = constellation.match(obs, _LMS, prior_xyz=true, tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    assert got == {"bench": "bench_1", "lamp": "lamp_1", "garden_table": "table_1"}
    assert len(pairs) == 3


def test_drift_immunity_prior_8m_off():
    true = (2.0, -1.0, 0.5)
    obs = [_observe_from_true_pose(lm, true) for lm in _LMS]
    bad_prior = (true[0] + 8.0, true[1] - 8.0, true[2] + 0.4)  # far off
    pairs = constellation.match(obs, _LMS, prior_xyz=bad_prior, tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    assert got == {"bench": "bench_1", "lamp": "lamp_1", "garden_table": "table_1"}
