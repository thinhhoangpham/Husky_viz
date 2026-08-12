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


def test_one_observation_returns_empty():
    obs = [_observe_from_true_pose(_LMS[0], (0, 0, 0))]
    assert constellation.match(obs, _LMS, prior_xyz=(0, 0, 0), tol=0.3) == []


def test_two_distinct_type_unique_pair_matches():
    two = [_LMS[0], _LMS[1]]  # bench + lamp, distinct types
    obs = [_observe_from_true_pose(lm, (0, 0, 0)) for lm in two]
    pairs = constellation.match(obs, _LMS, prior_xyz=(0, 0, 0), tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    assert got == {"bench": "bench_1", "lamp": "lamp_1"}


def test_no_match_shape_absent_returns_empty():
    # observations form a triangle with side lengths that exist in NO catalog trio
    obs = [Observation("bench", 0.0, 0.0),
           Observation("lamp", 100.0, 0.0),      # 100 m apart: no catalog pair
           Observation("garden_table", 0.0, 100.0)]
    assert constellation.match(obs, _LMS, prior_xyz=(0, 0, 0), tol=0.3) == []


def test_type_constraint_blocks_geometric_lookalike():
    # a lamp observation sitting exactly where a catalog BENCH is must NOT pair to
    # it; only lamp catalog entries are eligible.
    cat = [MapLandmark("bench_x", "bench", 5.0, 0.0),
           MapLandmark("lamp_far", "lamp", 40.0, 0.0)]
    obs = [Observation("lamp", 5.0, 0.0), Observation("bench", 40.0, 0.0)]
    pairs = constellation.match(obs, cat, prior_xyz=(0, 0, 0), tol=0.3)
    # geometry alone would swap them; type forbids it -> the only type-consistent
    # assignment (lamp->lamp_far, bench->bench_x) has the wrong distance, so empty.
    assert pairs == []


def test_ambiguity_resolved_by_prior():
    # two identical bench+lamp constellations, 50 m apart; prior sits next to the
    # SECOND one -> matcher must pick the second.
    cat = [MapLandmark("bench_a", "bench", 0.0, 0.0),
           MapLandmark("lamp_a", "lamp", 3.0, 0.0),
           MapLandmark("bench_b", "bench", 50.0, 0.0),
           MapLandmark("lamp_b", "lamp", 53.0, 0.0)]
    # robot at second cluster, observing bench_b + lamp_b from true pose (50,0,0)
    obs = [_observe_from_true_pose(cat[2], (50.0, 0.0, 0.0)),
           _observe_from_true_pose(cat[3], (50.0, 0.0, 0.0))]
    pairs = constellation.match(obs, cat, prior_xyz=(50.0, 0.0, 0.0), tol=0.3)
    got = {lm.name for _, lm in pairs}
    assert got == {"bench_b", "lamp_b"}


def test_collinear_triple_still_matches_or_empty():
    # three landmarks in a straight line, distinct types (so type still pins them)
    cat = [MapLandmark("bench_1", "bench", 0.0, 0.0),
           MapLandmark("lamp_1", "lamp", 5.0, 0.0),
           MapLandmark("table_1", "garden_table", 10.0, 0.0)]
    obs = [_observe_from_true_pose(lm, (0.0, 0.0, 0.3)) for lm in cat]
    pairs = constellation.match(obs, cat, prior_xyz=(0.0, 0.0, 0.3), tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    # distinct types make even a collinear set unambiguous
    assert got == {"bench": "bench_1", "lamp": "lamp_1", "garden_table": "table_1"}
