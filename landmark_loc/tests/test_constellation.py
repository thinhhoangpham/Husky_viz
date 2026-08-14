import math
from landmark_loc.constellation import match
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def _obs(ident, x, y):
    return Observation(ident, x, y)


def _lm(name, ident, x, y):
    return MapLandmark(name, ident, x, y)


# A map-frame catalog constellation (3 distinct-type landmarks) and the robot
# observing them from a KNOWN true pose. Observations are the catalog points
# expressed in the robot frame at that true pose.
def _scene(true_x, true_y, true_yaw):
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    c, s = math.cos(-true_yaw), math.sin(-true_yaw)
    obs = []
    for lm in cat:
        dx, dy = lm.x - true_x, lm.y - true_y
        obs.append(_obs(lm.identity, c * dx - s * dy, s * dx + c * dy))
    return obs, cat


def test_clean_match_at_correct_prior():
    obs, cat = _scene(0.0, 0.0, 0.0)
    pairs = match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    names = sorted(lm.name for _, lm in pairs)
    assert names == ["benchB", "binC", "lampA"]


def test_drift_immunity_prior_4m_off():
    # THE key test: the prior is 4m off, but identification must still be correct
    # because RANSAC uses the seed shape, not the prior.
    obs, cat = _scene(0.0, 0.0, 0.0)
    pairs = match(obs, cat, (4.0, -3.0, 0.0), 1.0)   # prior 5m off
    names = sorted(lm.name for _, lm in pairs)
    assert names == ["benchB", "binC", "lampA"]


def test_drift_immunity_prior_10m_off():
    obs, cat = _scene(0.0, 0.0, 0.0)
    pairs = match(obs, cat, (10.0, 0.0, 0.0), 1.0)   # 10m off, still under 15m sanity
    names = sorted(lm.name for _, lm in pairs)
    assert names == ["benchB", "binC", "lampA"]


def test_dense_same_type_picks_max_inlier_arrangement():
    # Two lamps + a bench + a bin. A tie on the lamp alone would defeat _grow;
    # RANSAC's whole-set inliers resolve it.
    cat = [_lm("lamp1", "lamp", 10.0, 0.0),
           _lm("lamp2", "lamp", 12.5, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    # robot at origin observing all four honestly
    obs = [_obs(lm.identity, lm.x, lm.y) for lm in cat]  # true_pose=(0,0,0)
    pairs = match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    assert len(pairs) >= 3
    # every observation maps to the correctly-positioned catalog landmark
    for o, lm in pairs:
        assert abs(o.x - lm.x) < 0.6 and abs(o.y - lm.y) < 0.6


def test_too_few_consistent_returns_empty():
    # Only 2 landmarks visible -> cannot reach 3 inliers -> []
    cat = [_lm("lampA", "lamp", 10.0, 0.0), _lm("benchB", "bench", 13.0, 4.0)]
    obs = [_obs(lm.identity, lm.x, lm.y) for lm in cat]
    assert match(obs, cat, (0.0, 0.0, 0.0), 1.0) == []


def test_absurd_pose_rejected_by_sanity():
    # A self-consistent 3-landmark scene whose implied robot pose is >15m from the
    # prior must be rejected.
    obs, cat = _scene(0.0, 0.0, 0.0)
    assert match(obs, cat, (100.0, 0.0, 0.0), 1.0) == []


def _o(ident, x, y, yaw=None):
    return Observation(ident, x, y, yaw)


def _l(name, ident, x, y, yaw=None):
    return MapLandmark(name, ident, x, y, yaw)


def test_yaw_diff_rejects_wrong_orientation_pair():
    # Two benches whose catalog yaw-difference is ~0 (parallel). An observed pair
    # with the SAME distance but a 90deg yaw-difference must NOT seed-match them.
    cat = [_l("bA", "bench", 10.0, 0.0, 0.0), _l("bB", "bench", 13.0, 0.0, 0.0),
           _l("lC", "lamp", 10.0, 5.0, None)]
    # observed: correct positions but bench yaws differ by 90deg (wrong)
    obs = [_o("bench", 10.0, 0.0, 0.0), _o("bench", 13.0, 0.0, math.pi / 2),
           _o("lamp", 10.0, 5.0, None)]
    pairs = match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    names = sorted(l.name for _, l in pairs)
    # the bench-bench yaw mismatch blocks that seed; with only 1 usable
    # correspondence type left, cannot reach 3 -> []
    assert pairs == [] or "bB" not in names


def test_yaw_diff_frame_invariant_still_matches_when_rotated():
    # Genuine rigid rotation of the WHOLE scene (positions AND yaws) by `rot`
    # about the origin. A rigid rotation must not break the match: the
    # recovered transform un-rotates the scene, so map_yaw = o.yaw +
    # transform_yaw ends up back at the catalog yaw for every typed object.
    cat = [_l("bA", "bench", 10.0, 0.0, 0.2), _l("bB", "bench", 13.0, 0.0, 0.2),
           _l("tC", "trash_bin_1", 8.0, 5.0, None)]
    rot = 0.7
    c, s = math.cos(rot), math.sin(rot)

    def _rotate(lm):
        ox = c * lm.x - s * lm.y
        oy = s * lm.x + c * lm.y
        oyaw = None if lm.yaw is None else lm.yaw + rot
        return _o(lm.identity, ox, oy, oyaw)

    obs = [_rotate(lm) for lm in cat]
    pairs = match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    assert len(pairs) >= 3
    names = sorted(l.name for _, l in pairs)
    assert names == ["bA", "bB", "tC"]


def test_none_yaw_pairs_match_distance_only():
    # all round types (no yaw) -> behaves exactly like distance-only today
    cat = [_l("l1", "lamp", 10.0, 0.0), _l("l2", "lamp", 12.5, 0.0),
           _l("t1", "trash_bin_1", 8.0, 5.0)]
    obs = [_o(l.identity, l.x, l.y) for l in cat]
    pairs = match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    assert len(pairs) >= 3
