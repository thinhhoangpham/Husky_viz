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


def test_drift_immunity_prior_off():
    # The prior must be off by less than max_prior_dist (default 5m) from the
    # correct constellation's centroid for the match to survive the primary
    # filter -- that filter is what makes far-away wrong matches physically
    # impossible in the first place (see the bug this guards against in
    # test_far_constellation_rejected_by_prior_filter). "Drift immunity" here
    # means the SHAPE match still succeeds despite a realistically-drifted
    # prior, not that the prior can be arbitrarily wrong.
    true = (2.0, -1.0, 0.5)
    obs = [_observe_from_true_pose(lm, true) for lm in _LMS]
    bad_prior = (true[0] + 2.0, true[1] - 2.0, true[2] + 0.4)  # realistic drift
    pairs = constellation.match(obs, _LMS, prior_xyz=bad_prior, tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    assert got == {"bench": "bench_1", "lamp": "lamp_1", "garden_table": "table_1"}


def test_one_observation_returns_empty():
    obs = [_observe_from_true_pose(_LMS[0], (0, 0, 0))]
    assert constellation.match(obs, _LMS, prior_xyz=(0, 0, 0), tol=0.3) == []


def test_two_landmark_observation_returns_empty():
    # Only 2 observed landmarks: below the >=3 minimum needed to remove the
    # 2-point reflection ambiguity, so the matcher must return nothing rather
    # than risk a confidently-wrong (possibly flipped) pose.
    two = [_LMS[0], _LMS[1]]  # bench + lamp, distinct types
    obs = [_observe_from_true_pose(lm, (0, 0, 0)) for lm in two]
    pairs = constellation.match(obs, _LMS, prior_xyz=(2.0, 0.0, 0.0), tol=0.3)
    assert pairs == []


def test_three_distinct_type_unique_triple_matches():
    obs = [_observe_from_true_pose(lm, (0, 0, 0)) for lm in _LMS]
    # prior must be within max_prior_dist of the constellation centroid
    pairs = constellation.match(obs, _LMS, prior_xyz=(2.0, 0.0, 0.0), tol=0.3)
    got = {o.identity: lm.name for o, lm in pairs}
    assert got == {"bench": "bench_1", "lamp": "lamp_1", "garden_table": "table_1"}


def test_no_match_shape_absent_returns_empty():
    # observations form a triangle with side lengths that exist in NO catalog trio
    obs = [Observation("bench", 0.0, 0.0),
           Observation("lamp", 100.0, 0.0),      # 100 m apart: no catalog pair
           Observation("garden_table", 0.0, 100.0)]
    assert constellation.match(obs, _LMS, prior_xyz=(0, 0, 0), tol=0.3) == []


def test_type_constraint_blocks_geometric_lookalike():
    # Type constraint must forbid swapping landmarks based on position alone.
    # Create a scenario where the only type-consistent assignment fails distance check.
    # Observations: lamp and bench are 35 m apart.
    # Catalog: two clusters with different inter-landmark distances.
    # The swapped assignment (lamp->bench, bench->lamp) is type-illegal.
    # The type-consistent assignment (lamp->lamp, bench->bench) exists but with
    # a different pair distance (40 m), so fails the tolerance check.
    cat = [MapLandmark("bench_x", "bench", 5.0, 0.0),
           MapLandmark("lamp_y", "lamp", 45.0, 0.0)]  # distance 40 m
    obs = [Observation("lamp", 5.0, 0.0), Observation("bench", 40.0, 0.0)]  # distance 35 m
    pairs = constellation.match(obs, cat, prior_xyz=(0, 0, 0), tol=0.3)
    # The only type-consistent assignment (lamp->lamp_y, bench->bench_x) has
    # catalog distance 40m but observed distance 35m, which exceeds tol=0.3, so rejected.
    assert pairs == []


def test_ambiguity_resolved_by_prior():
    # two identical bench+lamp+table constellations, 50 m apart; prior sits next
    # to the SECOND one -> matcher must pick the second. Three landmarks per
    # cluster (not two) so the match clears the >=3 minimum.
    cat = [MapLandmark("bench_a", "bench", 0.0, 0.0),
           MapLandmark("lamp_a", "lamp", 3.0, 0.0),
           MapLandmark("table_a", "garden_table", 0.0, 4.0),
           MapLandmark("bench_b", "bench", 50.0, 0.0),
           MapLandmark("lamp_b", "lamp", 53.0, 0.0),
           MapLandmark("table_b", "garden_table", 50.0, 4.0)]
    # robot at second cluster, observing bench_b + lamp_b + table_b from true
    # pose (50,0,0)
    obs = [_observe_from_true_pose(cat[3], (50.0, 0.0, 0.0)),
           _observe_from_true_pose(cat[4], (50.0, 0.0, 0.0)),
           _observe_from_true_pose(cat[5], (50.0, 0.0, 0.0))]
    pairs = constellation.match(obs, cat, prior_xyz=(50.0, 0.0, 0.0), tol=0.3)
    got = {lm.name for _, lm in pairs}
    assert got == {"bench_b", "lamp_b", "table_b"}


def test_far_constellation_rejected_by_prior_filter():
    # Regression test for the bug where a larger (more landmarks) but far-away
    # constellation beat a smaller/equal correct one because size was compared
    # BEFORE the prior was consulted. Two catalog clusters have the exact same
    # bench/lamp/table shape (so both grow to size 3 and tie on size); one sits
    # near the true pose/prior, the other ~13m away. The prior filter must
    # reject the far one outright so only the near one survives.
    near = [MapLandmark("bench_near", "bench", 0.0, 0.0),
            MapLandmark("lamp_near", "lamp", 3.0, 0.0),
            MapLandmark("table_near", "garden_table", 0.0, 4.0)]
    far = [MapLandmark("bench_far", "bench", 13.0, 0.0),
           MapLandmark("lamp_far", "lamp", 16.0, 0.0),
           MapLandmark("table_far", "garden_table", 13.0, 4.0)]
    cat = near + far
    true = (0.0, 0.0, 0.0)
    obs = [_observe_from_true_pose(lm, true) for lm in near]
    pairs = constellation.match(obs, cat, prior_xyz=true, tol=0.3)
    got = {lm.name for _, lm in pairs}
    assert got == {"bench_near", "lamp_near", "table_near"}


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
