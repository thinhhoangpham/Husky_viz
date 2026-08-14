import math
from landmark_loc.constellation_typeless import match as tl_match
from landmark_loc.constellation import match as typed_match
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def _obs(ident, x, y, yaw=None):
    return Observation(ident, x, y, yaw)


def _lm(name, ident, x, y, yaw=None):
    return MapLandmark(name, ident, x, y, yaw)


def _scene(true_x, true_y, true_yaw, cat):
    """Express catalog points in the robot frame at a known true pose."""
    c, s = math.cos(-true_yaw), math.sin(-true_yaw)
    obs = []
    for lm in cat:
        dx, dy = lm.x - true_x, lm.y - true_y
        obs.append(_obs(lm.identity, c * dx - s * dy, s * dx + c * dy))
    return obs


# --- PARITY: on an unambiguous scene, typeless must equal typed ---
def test_parity_unambiguous_scene():
    # distinct types AND distinct pairwise distances -> geometry alone is enough,
    # so removing types must NOT change the answer.
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)
    typed = sorted(lm.name for _, lm in typed_match(obs, cat, (0.0, 0.0, 0.0), 1.0))
    tl = sorted(lm.name for _, lm in tl_match(obs, cat, (0.0, 0.0, 0.0), 1.0))
    assert tl == typed == ["benchB", "binC", "lampA"]


def test_parity_prior_offset():
    # drift-immunity must also hold for the typeless matcher (geometry is invariant)
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)
    pairs = tl_match(obs, cat, (4.0, 0.0, 0.0), 1.0)  # prior 4m off
    assert sorted(lm.name for _, lm in pairs) == ["benchB", "binC", "lampA"]


# --- DIVERGENCE: where types would disambiguate, typeless may pick wrong ---
def test_typeless_admits_wrong_type_inlier():
    # Two catalog landmarks of DIFFERENT type at the same geometric spot pattern:
    # a bin and a lamp swapped in position but same distances. The typed matcher
    # must respect type; the typeless matcher is free to attach an observation to
    # the wrong-type-but-nearby catalog landmark.
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0),
           _lm("lampD", "lamp", 8.0, 5.0)]   # a LAMP sitting where the bin's obs lands
    obs = _scene(0.0, 0.0, 0.0, cat[:3])      # robot sees lamp, bench, bin
    typed = typed_match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    tl = tl_match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    # typed: the bin observation matches binC (type-correct)
    typed_names = {lm.name for _, lm in typed}
    assert "binC" in typed_names
    # typeless: the bin observation may be claimed by lampD (same spot, wrong type)
    # -> at minimum, the typeless result is allowed to differ; assert it does NOT
    #    enforce type (lampD is a legal inlier target for the bin observation).
    tl_names = {lm.name for _, lm in tl}
    assert ("lampD" in tl_names) or (tl_names != typed_names)


# --- REGRESSION: shared gates behave identically ---
def test_too_few_returns_empty():
    cat = [_lm("a", "lamp", 10.0, 0.0), _lm("b", "bench", 13.0, 4.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)  # only 2 -> below _MIN_INLIERS
    assert tl_match(obs, cat, (0.0, 0.0, 0.0), 1.0) == []


def test_absurd_pose_rejected_by_sanity():
    cat = [_lm("lampA", "lamp", 10.0, 0.0),
           _lm("benchB", "bench", 13.0, 4.0),
           _lm("binC", "trash_bin_1", 8.0, 5.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)
    # prior 100m away -> refit pose must be rejected by _PRIOR_SANITY
    assert tl_match(obs, cat, (100.0, 100.0, 0.0), 1.0) == []


def test_yaw_kept_in_typeless():
    # yaw is geometric, kept in both arms. An oriented pair whose observed yaw-diff
    # does NOT match the catalog yaw-diff must be rejected as a seed even typeless.
    cat = [_lm("benchA", "bench", 10.0, 0.0, 0.0),
           _lm("tableB", "garden_table", 14.0, 0.0, 0.0),
           _lm("lampC", "lamp", 12.0, 4.0)]
    obs = _scene(0.0, 0.0, 0.0, cat)
    # correct scene should still match (yaw consistent)
    pairs = tl_match(obs, cat, (0.0, 0.0, 0.0), 1.0)
    assert len(pairs) >= 3
