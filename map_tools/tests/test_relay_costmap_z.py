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
