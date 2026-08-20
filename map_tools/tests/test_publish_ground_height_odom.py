"""Tests for scripts/publish_ground_height_odom.py pure functions.

Mirrors the approach in test_publish_dtm_cloud.py: the script keeps its maths in
importable pure functions and its ROS wiring inside main(), so the maths is
testable without a running master.
"""
import importlib.util
import os

import numpy as np
import pytest

_SCRIPT = os.path.join(os.path.dirname(__file__), "..", "..",
                       "scripts", "publish_ground_height_odom.py")


def _load():
    spec = importlib.util.spec_from_file_location("ghodom", os.path.abspath(_SCRIPT))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ghodom = _load()


def test_clearance_comes_from_base_footprint_tf():
    """The clearance is chassis geometry, read as -(base_link->base_footprint).z.

    On this Husky the URDF puts that joint at wheel_vertical_offset - wheel_radius
    = 0.03282 - 0.1651 = -0.13228, so the clearance is 0.13228 m.
    """
    assert abs(ghodom.clearance_from_footprint_tf(-0.13228) - 0.13228) < 1e-9


def test_clearance_rejects_a_non_negative_footprint_offset():
    """base_footprint lies BELOW base_link. A zero or positive z means the frame
    is not the ground-projection frame this assumes, and silently accepting it
    would bury the robot in its own terrain."""
    for bad in (0.0, 0.13228):
        with pytest.raises(ValueError):
            ghodom.clearance_from_footprint_tf(bad)


def test_published_z_is_terrain_plus_clearance():
    """The whole point of the rewrite: z = terrain(x, y) + a CONSTANT clearance.

    Uses a known DTM cell so the two halves are checked together rather than in
    isolation.
    """
    dtm = np.array([[3.761, 5.000],
                    [7.250, np.nan]], dtype=float)   # row 0 = lowest y
    res, ox, oy = 1.0, 0.0, 0.0
    clearance = 0.13228

    elev = ghodom.dtm_elevation_at(dtm, res, ox, oy, 0.5, 0.5)
    assert elev == 3.761
    assert abs(ghodom.base_link_z(elev, clearance) - 3.89328) < 1e-9

    # A different cell -> different z, but the SAME clearance above its terrain.
    elev2 = ghodom.dtm_elevation_at(dtm, res, ox, oy, 0.5, 1.5)
    assert elev2 == 7.250
    assert abs(ghodom.base_link_z(elev2, clearance) - 7.38228) < 1e-9


def test_clearance_does_not_vary_with_terrain_slope():
    """Regression guard on the bug this rewrite fixes.

    The old plane-fit measured 0.08 m of clearance on gentle ground and 0.41 m on
    a slope -- a 0.33 m error that pushed every ground return onto the 0.40 m
    lower bound of filter_cloud_above_terrain.py's obstacle band. A constant
    cannot do that: the height above terrain is identical on flat and steep cells.
    """
    clearance = 0.13228
    flat, steep = 3.761, 11.940
    assert abs((ghodom.base_link_z(flat, clearance) - flat) -
               (ghodom.base_link_z(steep, clearance) - steep)) < 1e-12
    assert abs(ghodom.base_link_z(steep, clearance) - steep - clearance) < 1e-12


def test_off_dtm_publishes_nothing_rather_than_a_fabricated_height():
    """Over an uncovered cell there is no honest height, so base_link_z returns
    None and the node skips the publish. It must NOT fall back to 0.0, which the
    EKF would fuse as a real absolute elevation."""
    assert ghodom.base_link_z(None, 0.13228) is None
    # and the no-data path that produces that None
    dtm = np.array([[1.0, np.nan]], dtype=float)
    assert ghodom.dtm_elevation_at(dtm, 1.0, 0.0, 0.0, 1.5, 0.5) is None
    assert ghodom.base_link_z(
        ghodom.dtm_elevation_at(dtm, 1.0, 0.0, 0.0, 1.5, 0.5), 0.13228) is None


def test_no_ground_plane_fit_remains():
    """The slope-sensitive fit is gone, not merely unused: leaving it importable
    invites a caller to reintroduce the 0.33 m error."""
    for gone in ("fit_ground_plane", "sensor_height_from_fit", "smooth_median"):
        assert not hasattr(ghodom, gone), "%s should have been deleted" % gone


def test_dtm_elevation_lookup():
    z = np.array([[1.0, 2.0],
                  [3.0, np.nan]], dtype=float)   # row 0 = lowest y
    # cell centres are at origin + (i+0.5)*res; res=1.0, origin (0,0)
    assert ghodom.dtm_elevation_at(z, 1.0, 0.0, 0.0, 0.5, 0.5) == 1.0
    assert ghodom.dtm_elevation_at(z, 1.0, 0.0, 0.0, 1.5, 0.5) == 2.0
    assert ghodom.dtm_elevation_at(z, 1.0, 0.0, 0.0, 0.5, 1.5) == 3.0
    # NaN cell -> None (no data, never a fabricated 0.0)
    assert ghodom.dtm_elevation_at(z, 1.0, 0.0, 0.0, 1.5, 1.5) is None
    # outside the grid -> None
    assert ghodom.dtm_elevation_at(z, 1.0, 0.0, 0.0, 99.0, 0.5) is None
    assert ghodom.dtm_elevation_at(z, 1.0, 0.0, 0.0, -5.0, 0.5) is None


def test_covariance_makes_only_z_meaningful():
    cov = ghodom.covariance_for_z(0.05)
    assert len(cov) == 36
    assert cov[14] == 0.05          # z: the one we mean
    for i in (0, 7, 21, 28, 35):    # x, y, roll, pitch, yaw
        assert cov[i] >= 1e6


# --- dtm_elevation_at: negative-origin indexing (the int()-truncation bug) -----
# int() rounds TOWARD ZERO, so a query in the sub-cell strip just BELOW the origin
# gets index 0 instead of -1 and is answered with cell 0's terrain instead of being
# rejected as off-grid. Measured on the lake DTM (origin -49.75, -25.0): x = -49.85,
# y = -20.0 returned 4.130 m under truncation, None under floor. Inside the grid the
# two agree; np.floor is the fix, same as scripts/relay_path_z.py.

def test_negative_origin_uses_floor_not_truncation():
    # Distinct value per cell so a wrong index is unambiguous.
    z = np.arange(16, dtype=float).reshape(4, 4)
    res, ox, oy = 0.25, -49.75, -25.0

    # A point 0.1 m into cell 1 on both axes. Offsets are +0.35 m -> 1.4 cells.
    # floor(1.4) = 1 (correct). int(1.4) also = 1 here, so this is the control.
    assert ghodom.dtm_elevation_at(z, res, ox, oy, ox + 0.35, oy + 0.35) == z[1, 1]

    # The real bug bites when the *offset itself* is negative, i.e. the query is
    # below the origin: floor gives -1 (correctly rejected as off-grid) while
    # int() gives 0 (silently returns cell 0's terrain).
    below = ghodom.dtm_elevation_at(z, res, ox, oy, ox - 0.1, oy - 0.1)
    assert below is None, "point below the DTM origin must be off-grid, not cell 0"

    # And a point below the origin on ONE axis only must still be rejected.
    assert ghodom.dtm_elevation_at(z, res, ox, oy, ox + 0.35, oy - 0.1) is None
    assert ghodom.dtm_elevation_at(z, res, ox, oy, ox - 0.1, oy + 0.35) is None


def test_negative_world_coordinates_index_correctly():
    """Cells are selected by floor across a negative-origin grid, so each 0.25 m
    step advances exactly one cell rather than repeating cell 0."""
    z = np.arange(16, dtype=float).reshape(4, 4)
    res, ox, oy = 0.25, -49.75, -25.0
    # Walk x across the first four columns at a fixed row.
    got = [ghodom.dtm_elevation_at(z, res, ox, oy, ox + 0.125 + i * res, oy + 0.125)
           for i in range(4)]
    assert got == [z[0, 0], z[0, 1], z[0, 2], z[0, 3]]


def test_non_finite_coordinates_return_none():
    z = np.ones((4, 4), dtype=float)
    assert ghodom.dtm_elevation_at(z, 0.25, -49.75, -25.0, float("nan"), 0.0) is None
    assert ghodom.dtm_elevation_at(z, 0.25, -49.75, -25.0, 0.0, float("inf")) is None
