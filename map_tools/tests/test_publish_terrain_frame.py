"""Tests for the terrain-frame publisher's pure logic (no ROS master needed).

The module lives in scripts/, which is not a package, so it is loaded by
path, following map_tools/tests/test_publish_dtm_cloud.py's convention.
"""
import importlib.util
import math
import os

import numpy as np
import pytest

_SCRIPT = os.path.join(os.path.dirname(__file__), "..", "..", "scripts",
                       "publish_terrain_frame.py")

rospy = pytest.importorskip("rospy",
                            reason="ROS not on this interpreter's path")


def _load():
    spec = importlib.util.spec_from_file_location("publish_terrain_frame",
                                                   _SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


ptf = _load()


def _plane_points(a, b, c, n=200, seed=0):
    rng = np.random.default_rng(seed)
    x = rng.uniform(-5, 5, n)
    y = rng.uniform(-5, 5, n)
    z = a * x + b * y + c
    return np.column_stack([x, y, z])


def test_fit_ground_plane_recovers_known_slope():
    pts = _plane_points(0.1, -0.05, -1.4)
    a, b, c = ptf.fit_ground_plane(pts)
    assert a == pytest.approx(0.1, abs=1e-6)
    assert b == pytest.approx(-0.05, abs=1e-6)
    assert c == pytest.approx(-1.4, abs=1e-6)


def test_fit_ground_plane_flat_ground_zero_slope():
    pts = _plane_points(0.0, 0.0, -1.464)
    a, b, c = ptf.fit_ground_plane(pts)
    assert a == pytest.approx(0.0, abs=1e-9)
    assert b == pytest.approx(0.0, abs=1e-9)
    assert c == pytest.approx(-1.464, abs=1e-9)


def test_fit_ground_plane_rejects_too_few_points():
    with pytest.raises(ValueError):
        ptf.fit_ground_plane(np.array([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]))


def test_quat_roundtrips_roll_pitch():
    from landmark_loc import derotate

    for roll, pitch in [(0.0, 0.0), (math.radians(-3.78), math.radians(0.53)),
                        (math.radians(10.0), math.radians(-15.0))]:
        x, y, z, w = ptf.quat_from_roll_pitch(roll, pitch)
        # unit quaternion
        assert math.hypot(math.hypot(x, y), math.hypot(z, w)) == pytest.approx(1.0, abs=1e-9)
        r2, p2 = derotate.roll_pitch_from_quat(x, y, z, w)
        assert r2 == pytest.approx(roll, abs=1e-6)
        assert p2 == pytest.approx(pitch, abs=1e-6)


def test_ground_height_from_cloud_recovers_height_when_level():
    # Sensor at origin, flat ground 1.464 m below it, roll=pitch=0.
    pts = _plane_points(0.0, 0.0, -1.464, n=500)
    height = ptf.ground_height_from_cloud(
        pts, roll=0.0, pitch=0.0, fit_min_range=1.5, fit_max_range=8.0,
        ground_percentile=25)
    assert height == pytest.approx(1.464, abs=0.05)


def test_ground_height_from_cloud_raises_on_empty():
    with pytest.raises(ValueError):
        ptf.ground_height_from_cloud(
            np.zeros((0, 3)), roll=0.0, pitch=0.0, fit_min_range=1.5,
            fit_max_range=8.0, ground_percentile=25)


def test_rolling_median_returns_median_of_window():
    rm = ptf.RollingMedian(3)
    assert rm.push(1.0) == pytest.approx(1.0)
    assert rm.push(3.0) == pytest.approx(2.0)
    assert rm.push(2.0) == pytest.approx(2.0)
    # window is full (size 3); pushing a 4th value evicts the oldest (1.0)
    assert rm.push(10.0) == pytest.approx(3.0)


def test_rolling_median_empty_returns_none():
    rm = ptf.RollingMedian(5)
    assert rm.value() is None


def test_rolling_median_rejects_nonpositive_window():
    with pytest.raises(ValueError):
        ptf.RollingMedian(0)
