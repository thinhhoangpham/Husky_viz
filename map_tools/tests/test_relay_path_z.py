"""Tests for the path/pose/marker z relay's pure logic (no ROS master needed).

The module lives in scripts/, which is not a package, so it is loaded by path
-- same approach as test_relay_costmap_z.py.
"""
import importlib.util
import os

import numpy as np
import pytest

_SCRIPT = os.path.join(os.path.dirname(__file__), "..", "..", "scripts",
                       "relay_path_z.py")

rospy = pytest.importorskip("rospy",
                            reason="ROS not on this interpreter's path")


def _load():
    spec = importlib.util.spec_from_file_location("relay_path_z", _SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


rpz = _load()

_MAPS = os.path.join(os.path.dirname(__file__), "..", "..", "maps")


# --- dtm_height_at ---------------------------------------------------------

def _grid():
    """4x4 DTM, resolution 1.0, origin (0, 0). Row 0 = lowest y."""
    return np.array([[1.0, 2.0, 3.0, 4.0],
                     [5.0, 6.0, 7.0, 8.0],
                     [9.0, 10.0, 11.0, 12.0],
                     [13.0, 14.0, 15.0, 16.0]], dtype=np.float32)


def test_height_is_read_at_the_right_cell():
    assert rpz.dtm_height_at(_grid(), 1.0, 0.0, 0.0, 2.5, 1.5) == pytest.approx(7.0)


def test_row_zero_is_lowest_y():
    """Row 0 = LOWEST y, matching the .npy layout documented in the DTM yaml."""
    assert rpz.dtm_height_at(_grid(), 1.0, 0.0, 0.0, 0.5, 0.5) == pytest.approx(1.0)
    assert rpz.dtm_height_at(_grid(), 1.0, 0.0, 0.0, 0.5, 3.5) == pytest.approx(13.0)


def test_off_grid_returns_none_not_zero():
    """None is 'no terrain data', which is NOT 'height zero'."""
    assert rpz.dtm_height_at(_grid(), 1.0, 0.0, 0.0, 99.0, 0.5) is None
    assert rpz.dtm_height_at(_grid(), 1.0, 0.0, 0.0, 0.5, -5.0) is None


def test_nan_cell_returns_none():
    z = _grid()
    z[1, 2] = np.nan
    assert rpz.dtm_height_at(z, 1.0, 0.0, 0.0, 2.5, 1.5) is None


def test_non_finite_coordinate_returns_none():
    assert rpz.dtm_height_at(_grid(), 1.0, 0.0, 0.0, float("nan"), 1.0) is None
    assert rpz.dtm_height_at(_grid(), 1.0, 0.0, 0.0, 1.0, float("inf")) is None


def test_negative_origin_does_not_fold_cells_onto_zero():
    """int() truncation rounds toward zero, which on the negative side of the
    map folds the cell below the origin onto cell 0 and reads the WRONG
    terrain. The lake DTM's origin is (-49.75, -25.0), so this is not
    hypothetical -- it affects every plan in the western half of that world."""
    z = _grid()
    # origin -2.0: x=-1.5 is cell 0, x=-0.5 is cell 1. Truncation gives 0 for both.
    assert rpz.dtm_height_at(z, 1.0, -2.0, -2.0, -1.5, -1.5) == pytest.approx(1.0)
    assert rpz.dtm_height_at(z, 1.0, -2.0, -2.0, -0.5, -1.5) == pytest.approx(2.0)


def test_returns_a_python_float():
    v = rpz.dtm_height_at(_grid(), 1.0, 0.0, 0.0, 0.5, 0.5)
    assert type(v) is float


def test_empty_grid_returns_none():
    assert rpz.dtm_height_at(np.zeros((0, 0)), 1.0, 0.0, 0.0, 0.0, 0.0) is None


# --- drape_heights ---------------------------------------------------------

def _ramp(x, y):
    """Terrain that rises with x; None outside 0 <= x < 10."""
    return None if not (0.0 <= x < 10.0) else float(x)


def test_each_point_gets_its_own_height():
    """The whole point of a per-point relay: the route DRAPES over relief
    instead of sitting on one flat plane the way a costmap must."""
    out = rpz.drape_heights([(1.0, 0.0), (2.0, 0.0), (3.0, 0.0)], _ramp)
    assert out == pytest.approx([1.0, 2.0, 3.0])


def test_z_offset_lifts_every_point():
    out = rpz.drape_heights([(1.0, 0.0), (2.0, 0.0)], _ramp, z_offset=0.15)
    assert out == pytest.approx([1.15, 2.15])


def test_gap_inherits_the_last_valid_height():
    """An off-mesh excursion renders as a flat bridge at the level of the
    ground the path left, not a spike to zero."""
    out = rpz.drape_heights(
        [(1.0, 0.0), (2.0, 0.0), (99.0, 0.0), (3.0, 0.0)], _ramp)
    assert out == pytest.approx([1.0, 2.0, 2.0, 3.0])


def test_leading_gap_inherits_the_first_valid_height():
    out = rpz.drape_heights([(99.0, 0.0), (-5.0, 0.0), (4.0, 0.0)], _ramp)
    assert out == pytest.approx([4.0, 4.0, 4.0])


def test_trailing_gap_holds_the_last_height():
    out = rpz.drape_heights([(4.0, 0.0), (99.0, 0.0), (99.0, 0.0)], _ramp)
    assert out == pytest.approx([4.0, 4.0, 4.0])


def test_no_valid_sample_anywhere_returns_none():
    """None tells the caller to publish the message UNCHANGED, so a plan is
    never silently missing from RViz."""
    assert rpz.drape_heights([(99.0, 0.0), (-5.0, 0.0)], _ramp) is None


def test_empty_input_returns_none():
    assert rpz.drape_heights([], _ramp) is None


def test_output_length_always_matches_input():
    xys = [(1.0, 0.0), (99.0, 0.0), (2.0, 0.0), (-7.0, 0.0)]
    assert len(rpz.drape_heights(xys, _ramp)) == len(xys)


def test_a_gap_never_produces_zero():
    """Regression on the actual bug: z=0 is four metres underground in the
    lake world, so a gap must never fall back to it."""
    out = rpz.drape_heights([(5.0, 0.0), (99.0, 0.0)], _ramp)
    assert all(v != 0.0 for v in out)


# --- drape_path / drape_pose ----------------------------------------------

def _path(xys, frame="map"):
    from nav_msgs.msg import Path
    from geometry_msgs.msg import PoseStamped
    p = Path()
    p.header.frame_id = frame
    for x, y in xys:
        ps = PoseStamped()
        ps.pose.position.x, ps.pose.position.y = x, y
        ps.pose.orientation.w = 1.0
        p.poses.append(ps)
    return p


def test_path_poses_get_per_point_heights():
    out = rpz.drape_path(_path([(1.0, 0.0), (3.0, 0.0)]), _ramp)
    assert [p.pose.position.z for p in out.poses] == pytest.approx([1.0, 3.0])


def test_path_input_is_never_mutated():
    """The input is shared with every other subscriber in the process and with
    rospy's own buffers -- mutating it would corrupt what move_base sees."""
    src = _path([(1.0, 0.0), (3.0, 0.0)])
    rpz.drape_path(src, _ramp)
    assert [p.pose.position.z for p in src.poses] == pytest.approx([0.0, 0.0])


def test_path_carries_everything_else_through_unchanged():
    src = _path([(1.0, 2.0)], frame="odom")
    out = rpz.drape_path(src, _ramp)
    assert out.header.frame_id == "odom"
    assert out.poses[0].pose.position.x == pytest.approx(1.0)
    assert out.poses[0].pose.position.y == pytest.approx(2.0)
    assert out.poses[0].pose.orientation.w == pytest.approx(1.0)


def test_path_with_no_terrain_anywhere_is_returned_unchanged():
    src = _path([(99.0, 0.0)])
    assert rpz.drape_path(src, _ramp) is src


def test_empty_path_is_returned_unchanged():
    src = _path([])
    assert rpz.drape_path(src, _ramp) is src


def test_pose_is_draped():
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.header.frame_id = "map"
    ps.pose.position.x = 4.0
    out = rpz.drape_pose(ps, _ramp, z_offset=0.15)
    assert out.pose.position.z == pytest.approx(4.15)
    assert ps.pose.position.z == pytest.approx(0.0)


def test_pose_off_the_dtm_is_returned_unchanged():
    from geometry_msgs.msg import PoseStamped
    ps = PoseStamped()
    ps.pose.position.x = 99.0
    assert rpz.drape_pose(ps, _ramp) is ps


# --- out_topic_for / split_topics -----------------------------------------

def test_output_topic_always_differs_from_the_input():
    """Republishing onto a topic move_base consumes is the one thing this node
    must never do."""
    for t in ("/move_base/NavfnROS/plan", "/move_base/current_goal"):
        assert rpz.out_topic_for(t) != t


def test_empty_suffix_is_rejected():
    with pytest.raises(ValueError):
        rpz.out_topic_for("/move_base/NavfnROS/plan", "")


def test_default_topics_are_the_ones_measured_broken():
    defaults = (rpz.split_topics(rpz.DEFAULT_PATH_TOPICS)
                + rpz.split_topics(rpz.DEFAULT_POSE_TOPICS))
    assert "/move_base/NavfnROS/plan" in defaults
    assert "/move_base/DWAPlannerROS/local_plan" in defaults
    assert "/move_base/current_goal" in defaults


def test_markers_are_not_relayed_by_default():
    """/landmark_observed_markers already carries a real 3-D z (the localizer
    publishes in the lidar frame at cluster_top + 0.5), so relaying it would
    add terrain height to a height that already includes it."""
    assert rpz.split_topics(rpz.DEFAULT_MARKER_TOPICS) == []


def test_split_topics_drops_blanks_and_whitespace():
    assert rpz.split_topics(" /a , ,/b ") == ["/a", "/b"]
    assert rpz.split_topics("") == []


# --- against the real lake DTM --------------------------------------------

def _lake():
    npy = os.path.join(_MAPS, "lake_dtm.npy")
    if not os.path.exists(npy):
        pytest.skip("maps/lake_dtm.npy not present")
    res, ox, oy = rpz.load_dtm_meta(npy)
    return np.load(npy), res, ox, oy


def test_lake_heights_are_inside_the_extractors_reported_range():
    """A draped plan must land in the terrain band the extractor recorded
    (z_min 3.504688 .. z_max 5.927002), never at the z=0 this node exists to
    fix."""
    z, res, ox, oy = _lake()
    def sample(x, y):
        return rpz.dtm_height_at(z, res, ox, oy, x, y)
    xys = [(x, 0.0) for x in np.arange(-45.0, 45.0, 1.0)]
    out = rpz.drape_heights(xys, sample)
    assert out is not None
    assert min(out) >= 3.504688 - 1e-3
    assert max(out) <= 5.927002 + 1e-3


def test_lake_plan_actually_varies_in_height():
    """If every point came out the same the drape would be pointless -- this
    is what a per-point relay buys over the costmap's single flat sheet."""
    z, res, ox, oy = _lake()
    def sample(x, y):
        return rpz.dtm_height_at(z, res, ox, oy, x, y)
    out = rpz.drape_heights([(x, 0.0) for x in np.arange(-45.0, 45.0, 1.0)],
                            sample)
    assert max(out) - min(out) > 0.5


def test_lake_meta_matches_the_extractor_yaml():
    z, res, ox, oy = _lake()
    assert res == pytest.approx(0.25)
    assert ox == pytest.approx(-49.75)
    assert oy == pytest.approx(-25.0)
    assert z.shape == (200, 398)


def test_load_dtm_meta_raises_when_the_yaml_is_absent(tmp_path):
    with pytest.raises(IOError):
        rpz.load_dtm_meta(str(tmp_path / "nope.npy"))


def test_resolve_dtm_path_is_absolute_and_world_selective():
    assert rpz.resolve_dtm_path("lake").endswith("lake_dtm.npy")
    assert os.path.isabs(rpz.resolve_dtm_path("park"))
    assert rpz.resolve_dtm_path("park", "~/x.npy") == os.path.abspath(
        os.path.expanduser("~/x.npy"))
