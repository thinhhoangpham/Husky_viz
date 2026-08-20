"""Tests for the costmap z relay's pure logic (no ROS master needed).

The module lives in scripts/, which is not a package, so it is loaded by path
-- same approach as test_publish_dtm_cloud.py.
"""
import importlib.util
import os

import numpy as np
import pytest

_SCRIPT = os.path.join(os.path.dirname(__file__), "..", "..", "scripts",
                       "relay_costmap_z.py")

rospy = pytest.importorskip("rospy",
                            reason="ROS not on this interpreter's path")


def _load():
    spec = importlib.util.spec_from_file_location("relay_costmap_z", _SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


rcz = _load()


# --- dtm_min_z -------------------------------------------------------------

def test_min_is_the_minimum_finite_height():
    z = np.array([[5.0, 3.25], [4.0, 9.0]], dtype=np.float32)
    assert rcz.dtm_min_z(z) == pytest.approx(3.25)


def test_nan_cells_do_not_drag_the_minimum_down():
    """NaN means 'no mesh covered this cell', NOT a height of zero. If NaNs
    were treated as low ground the sheet would sink to 0 in any world with
    off-mesh void -- which is every world here."""
    z = np.array([[np.nan, 4.5], [np.nan, 6.0]], dtype=np.float32)
    assert rcz.dtm_min_z(z) == pytest.approx(4.5)


def test_all_nan_grid_falls_back_to_zero():
    z = np.full((3, 3), np.nan, dtype=np.float32)
    assert rcz.dtm_min_z(z) == 0.0


def test_empty_grid_falls_back_to_zero():
    assert rcz.dtm_min_z(np.zeros((0, 0), dtype=np.float32)) == 0.0


def test_negative_heights_are_honoured():
    """Nothing forces terrain to be above zero; the minimum must not be
    clamped."""
    z = np.array([[-2.5, 1.0]], dtype=np.float32)
    assert rcz.dtm_min_z(z) == pytest.approx(-2.5)


def test_infinity_is_excluded_like_nan():
    z = np.array([[np.inf, 7.0], [-np.inf, 8.0]], dtype=np.float32)
    assert rcz.dtm_min_z(z) == pytest.approx(7.0)


def test_returns_a_python_float_not_a_numpy_scalar():
    """The value goes straight into a ROS message field; a numpy scalar
    serialises inconsistently across versions."""
    assert type(rcz.dtm_min_z(np.array([[1.0]], dtype=np.float32))) is float


# --- real DTMs -------------------------------------------------------------

_MAPS = os.path.join(os.path.dirname(__file__), "..", "..", "maps")


@pytest.mark.parametrize("world,expected", [
    ("park", 2.985557),
    ("lake", 3.504688),
])
def test_real_dtm_minimum_matches_the_extractor_yaml(world, expected):
    """The relay's z must equal the z_min the DTM extractor recorded, or the
    sheet is not where the yaml says the terrain starts."""
    path = os.path.join(_MAPS, "%s_dtm.npy" % world)
    if not os.path.exists(path):
        pytest.skip("no %s DTM checked out" % world)
    assert rcz.dtm_min_z(np.load(path)) == pytest.approx(expected, abs=1e-5)


def test_real_dtm_minimum_is_at_or_below_every_finite_cell():
    """The whole point of choosing the minimum: the sheet may never poke up
    through the terrain anywhere."""
    path = os.path.join(_MAPS, "lake_dtm.npy")
    if not os.path.exists(path):
        pytest.skip("no lake DTM checked out")
    z = np.load(path)
    assert (z[np.isfinite(z)] >= rcz.dtm_min_z(z)).all()


# --- resolve_dtm_path ------------------------------------------------------

def test_world_selects_that_worlds_dtm():
    assert rcz.resolve_dtm_path("lake").endswith("maps/lake_dtm.npy")
    assert rcz.resolve_dtm_path("park").endswith("maps/park_dtm.npy")


def test_explicit_dtm_path_overrides_world():
    got = rcz.resolve_dtm_path("park", dtm_path="/tmp/other_dtm.npy")
    assert got == "/tmp/other_dtm.npy"


def test_dtm_path_is_user_expanded():
    got = rcz.resolve_dtm_path("park", dtm_path="~/x_dtm.npy")
    assert got == os.path.join(os.path.expanduser("~"), "x_dtm.npy")


def test_resolved_path_is_absolute():
    assert os.path.isabs(rcz.resolve_dtm_path("lake"))


def test_maps_dir_can_be_overridden():
    got = rcz.resolve_dtm_path("lake", maps_dir="/srv/maps")
    assert got == "/srv/maps/lake_dtm.npy"


# --- shift_grid_z ----------------------------------------------------------

def _grid():
    from nav_msgs.msg import OccupancyGrid
    g = OccupancyGrid()
    g.header.frame_id = "map"
    g.info.resolution = 0.15
    g.info.width, g.info.height = 3, 2
    g.info.origin.position.x = -55.4915
    g.info.origin.position.y = -30.9713
    g.info.origin.position.z = 0.0
    g.info.origin.orientation.w = 1.0
    g.data = [0, 100, -1, 0, -1, 100]
    return g


def test_z_is_set_on_the_output():
    out = rcz.shift_grid_z(_grid(), 3.504688)
    assert out.info.origin.position.z == pytest.approx(3.504688)


def test_input_message_is_never_mutated():
    """move_base and other subscribers share this message object; writing z
    through it would corrupt the costmap they read."""
    src = _grid()
    rcz.shift_grid_z(src, 9.0)
    assert src.info.origin.position.z == 0.0


def test_everything_except_z_is_carried_through_unchanged():
    src = _grid()
    out = rcz.shift_grid_z(src, 3.5)
    assert out.header.frame_id == src.header.frame_id
    assert out.info.resolution == src.info.resolution
    assert (out.info.width, out.info.height) == (src.info.width, src.info.height)
    assert out.info.origin.position.x == src.info.origin.position.x
    assert out.info.origin.position.y == src.info.origin.position.y
    assert out.info.origin.orientation.w == src.info.origin.orientation.w


def test_occupancy_values_are_identical_including_unknown():
    """Water must stay UNKNOWN (-1) through the relay -- never rewritten to
    free or lethal."""
    src = _grid()
    out = rcz.shift_grid_z(src, 3.5)
    assert list(out.data) == list(src.data)
    assert -1 in list(out.data)


def test_relay_is_idempotent_in_z():
    once = rcz.shift_grid_z(_grid(), 3.5)
    twice = rcz.shift_grid_z(once, 3.5)
    assert twice.info.origin.position.z == pytest.approx(3.5)
    assert list(twice.data) == list(once.data)


def test_default_topics_differ():
    """The output must never land on the topic move_base consumes."""
    assert rcz.DEFAULT_OUT_TOPIC != rcz.DEFAULT_IN_TOPIC


# --- follow-robot mode: window_min_z ---------------------------------------
#
# The LOCAL costmap rolls with the robot, so a single global-minimum z would
# sink further below the ground the robot is actually on as it climbs. These
# cover the windowed sample that replaces it.

def _ramp_dtm():
    """A 10x10 grid at 1 m cells whose height equals its column index.

    origin (0, 0), so world x in [0, 10) and cell (r, c) has height c. A pure
    x-ramp makes the expected minimum over any window trivially readable.
    """
    return np.tile(np.arange(10, dtype=np.float32), (10, 1))


def test_window_min_is_the_lowest_cell_the_window_covers():
    """The window, not the single cell under the robot: the sheet has to clear
    the LOWEST ground it overlaps or it cuts through the slope in front."""
    z = _ramp_dtm()
    # robot at x=5.5 -> centre cell 5; a 4 m window spans x in [3.5, 7.5),
    # i.e. columns 3..7, whose minimum height is 3.
    got = rcz.window_min_z(z, 1.0, 0.0, 0.0, robot_x=5.5, robot_y=5.5,
                           window=4.0)
    assert got == pytest.approx(3.0)


def test_window_min_tracks_the_robot_uphill():
    """The whole reason this mode exists: as the robot climbs, its sheet must
    come up with it instead of staying pinned to the map minimum."""
    z = _ramp_dtm()
    low = rcz.window_min_z(z, 1.0, 0.0, 0.0, 2.5, 5.0, window=2.0)
    high = rcz.window_min_z(z, 1.0, 0.0, 0.0, 8.5, 5.0, window=2.0)
    assert high > low


def test_window_min_never_exceeds_any_covered_cell():
    """The invariant inherited from the global relay: the sheet may sit below
    the terrain, never poke up through it."""
    z = _ramp_dtm()
    window = 6.0
    got = rcz.window_min_z(z, 1.0, 0.0, 0.0, 5.5, 5.5, window=window)
    half = window / 2.0
    covered = z[:, int(5.5 - half):int(5.5 + half)]
    assert (covered >= got).all()


def test_window_is_clipped_to_the_grid_not_wrapped():
    """A robot near the edge must sample the cells that exist. Negative
    indices would wrap to the far side of the map and read unrelated ground."""
    z = _ramp_dtm()
    # robot at the low-x edge; the window runs off the grid on the left.
    got = rcz.window_min_z(z, 1.0, 0.0, 0.0, 0.5, 0.5, window=6.0)
    assert got == pytest.approx(0.0)


def test_window_min_ignores_nan_cells():
    """NaN is 'no mesh here', not low ground -- same rule as dtm_min_z."""
    z = _ramp_dtm().copy()
    z[:, 3] = np.nan
    got = rcz.window_min_z(z, 1.0, 0.0, 0.0, 5.5, 5.5, window=4.0)
    assert got == pytest.approx(4.0)


def test_window_of_all_nan_returns_none():
    """Off-mesh: there is no terrain to sit on. None means 'no answer', which
    the caller turns into 'hold the last good z' rather than dropping the
    sheet to 0 and flashing it underground for a frame."""
    z = np.full((10, 10), np.nan, dtype=np.float32)
    assert rcz.window_min_z(z, 1.0, 0.0, 0.0, 5.0, 5.0, window=4.0) is None


def test_robot_entirely_off_the_grid_returns_none():
    z = _ramp_dtm()
    assert rcz.window_min_z(z, 1.0, 0.0, 0.0, 500.0, 500.0, window=4.0) is None


def test_non_finite_robot_position_returns_none():
    """A bad tf lookup must not be laundered into a plausible height."""
    z = _ramp_dtm()
    assert rcz.window_min_z(z, 1.0, 0.0, 0.0, float("nan"), 5.0, 4.0) is None


def test_window_min_returns_a_python_float():
    z = _ramp_dtm()
    assert type(rcz.window_min_z(z, 1.0, 0.0, 0.0, 5.5, 5.5, 4.0)) is float


def test_negative_origin_does_not_fold_cells_onto_zero():
    """floor, not truncation: on the negative side of the map, int() rounds
    toward zero and silently reads the wrong row/col."""
    z = _ramp_dtm()
    # origin -5 => world x=-4.5 is column 0.
    got = rcz.window_min_z(z, 1.0, -5.0, -5.0, -4.5, 0.0, window=2.0)
    assert got == pytest.approx(0.0)


def test_window_min_on_the_real_lake_dtm_beats_the_global_minimum():
    """On real terrain the follow-robot sheet must sit closer to the ground
    under the robot than the global minimum does -- that is the entire gain.
    The lake has up to 2.0 m of relief inside a single 10 m window, so this is
    not a rounding-level difference."""
    path = os.path.join(_MAPS, "lake_dtm.npy")
    if not os.path.exists(path):
        pytest.skip("no lake DTM checked out")
    z = np.load(path)
    res, ox, oy = 0.25, -49.75, -25.0
    # A cell high on the terrain, chosen from the data rather than hardcoded.
    rows, cols = np.where(np.isfinite(z) & (z > np.nanpercentile(z, 95)))
    r, c = int(rows[0]), int(cols[0])
    x = ox + (c + 0.5) * res
    y = oy + (r + 0.5) * res
    local = rcz.window_min_z(z, res, ox, oy, x, y, window=10.0)
    assert local is not None
    assert local > rcz.dtm_min_z(z)
    assert local <= float(z[r, c])


# --- load_dtm_meta / grid_window_metres ------------------------------------

def test_load_dtm_meta_reads_the_sibling_yaml():
    """The .npy has no geometry of its own; without the .yaml the window would
    be sampled at the wrong place on the map entirely."""
    path = os.path.join(_MAPS, "lake_dtm.npy")
    if not os.path.exists(path):
        pytest.skip("no lake DTM checked out")
    assert rcz.load_dtm_meta(path) == (0.25, -49.75, -25.0)


def test_load_dtm_meta_raises_when_the_yaml_is_absent():
    with pytest.raises(IOError):
        rcz.load_dtm_meta("/nonexistent/nowhere_dtm.npy")


def test_load_dtm_meta_rejects_an_incomplete_yaml(tmp_path):
    """A yaml missing origin_x would otherwise silently default and shift the
    sampled window by tens of metres."""
    npy = tmp_path / "x_dtm.npy"
    (tmp_path / "x_dtm.yaml").write_text("resolution: 0.25\norigin_y: -1.0\n")
    with pytest.raises(ValueError):
        rcz.load_dtm_meta(str(npy))


def test_grid_window_metres_uses_the_larger_side():
    """The sampled window must cover the whole sheet, so a non-square costmap
    is sized by its longer edge."""
    g = _grid()
    g.info.resolution = 0.05
    g.info.width, g.info.height = 200, 100
    assert rcz.grid_window_metres(g) == pytest.approx(10.0)


def test_grid_window_metres_matches_the_configured_local_costmap():
    """config/costmap_local_gps.yaml is 10x10 m at 0.05 m; the relay reads the
    size off the message so the two cannot drift apart."""
    g = _grid()
    g.info.resolution = 0.05
    g.info.width, g.info.height = 200, 200
    assert rcz.grid_window_metres(g) == pytest.approx(10.0)


# --- the two modes stay distinct -------------------------------------------

def test_follow_robot_z_differs_from_the_global_minimum_on_high_ground():
    """Regression guard for the bug this mode fixes: on the lake the local
    sheet must NOT collapse onto the global-minimum answer."""
    path = os.path.join(_MAPS, "lake_dtm.npy")
    if not os.path.exists(path):
        pytest.skip("no lake DTM checked out")
    z = np.load(path)
    res, ox, oy = rcz.load_dtm_meta(path)
    rows, cols = np.where(np.isfinite(z) & (z > np.nanpercentile(z, 95)))
    r, c = int(rows[0]), int(cols[0])
    local = rcz.window_min_z(z, res, ox, oy,
                             ox + (c + 0.5) * res, oy + (r + 0.5) * res, 10.0)
    assert local - rcz.dtm_min_z(z) > 0.5


def test_window_min_never_pokes_through_terrain_anywhere_on_the_lake():
    """Swept over the real map: for every sampled robot position, the sheet is
    at or below every finite cell of the window it is drawn over."""
    path = os.path.join(_MAPS, "lake_dtm.npy")
    if not os.path.exists(path):
        pytest.skip("no lake DTM checked out")
    z = np.load(path)
    res, ox, oy = rcz.load_dtm_meta(path)
    window, half = 10.0, 5.0
    checked = 0
    for r in range(0, z.shape[0], 17):
        for c in range(0, z.shape[1], 17):
            x, y = ox + (c + 0.5) * res, oy + (r + 0.5) * res
            got = rcz.window_min_z(z, res, ox, oy, x, y, window)
            if got is None:
                continue
            c0 = max(int(np.floor((x - half - ox) / res)), 0)
            c1 = min(int(np.floor((x + half - ox) / res)), z.shape[1] - 1)
            r0 = max(int(np.floor((y - half - oy) / res)), 0)
            r1 = min(int(np.floor((y + half - oy) / res)), z.shape[0] - 1)
            win = z[r0:r1 + 1, c0:c1 + 1]
            fin = win[np.isfinite(win)]
            assert (fin >= got).all()
            checked += 1
    assert checked > 50
