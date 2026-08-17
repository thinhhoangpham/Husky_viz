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


def _annulus_points(a, b, c, n=800, r_in=2.0, r_out=7.0, seed=0):
    """Points on the plane z = a*x + b*y + c, spread over an annulus."""
    rng = np.random.RandomState(seed)
    r = rng.uniform(r_in, r_out, n)
    th = rng.uniform(-np.pi, np.pi, n)
    x = r * np.cos(th)
    y = r * np.sin(th)
    z = a * x + b * y + c
    return np.c_[x, y, z]


def test_fit_recovers_known_slope_and_height():
    pts = _annulus_points(0.10, -0.05, -1.40)
    fit = ghodom.fit_ground_plane(pts)
    assert fit is not None
    a, b, c = fit
    assert abs(a - 0.10) < 0.02
    assert abs(b - (-0.05)) < 0.02
    assert abs(c - (-1.40)) < 0.05


def test_flat_ground_gives_zero_slope_and_exact_height():
    pts = _annulus_points(0.0, 0.0, -1.25)
    fit = ghodom.fit_ground_plane(pts)
    assert fit is not None
    a, b, c = fit
    assert abs(a) < 1e-6
    assert abs(b) < 1e-6
    assert abs(c - (-1.25)) < 1e-6
    # sensor height is the negated intercept
    assert abs(ghodom.sensor_height_from_fit(fit) - 1.25) < 1e-6


def test_fit_returns_none_when_too_few_points():
    assert ghodom.fit_ground_plane(np.zeros((3, 3))) is None
    # points all inside the min_range annulus hole -> nothing to fit
    close = _annulus_points(0.0, 0.0, -1.0, n=200, r_in=0.1, r_out=0.5)
    assert ghodom.fit_ground_plane(close) is None


def test_base_link_z_subtracts_mount_offset():
    # sensor 1.464 m above ground, mounted 0.826 m above base_link.
    # With no terrain elevation supplied this is CLEARANCE only.
    assert abs(ghodom.base_link_z(1.464, 0.826) - 0.638) < 1e-9
    assert ghodom.base_link_z(None, 0.826) is None


def test_base_link_z_adds_terrain_elevation():
    # The real case: terrain under the robot is at 3.761 m and the robot's
    # clearance above it is 0.125 m -> absolute z 3.886 (Gazebo truth).
    z = ghodom.base_link_z(0.951, 0.826, ground_elev=3.761)
    assert abs(z - 3.886) < 1e-3
    # regression guard on the bug this fixes: without the terrain term the
    # robot sank ~3.8 m and rendered beneath the map it was standing on
    assert abs(ghodom.base_link_z(0.951, 0.826) - 0.125) < 1e-3


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


def test_smooth_median_returns_window_median():
    assert ghodom.smooth_median([1.0, 5.0, 2.0]) == 2.0
    assert ghodom.smooth_median([]) is None
    # a single wild outlier must not move the median much
    assert ghodom.smooth_median([0.6, 0.62, 0.61, 0.59, 99.0]) == 0.61


def test_covariance_makes_only_z_meaningful():
    cov = ghodom.covariance_for_z(0.05)
    assert len(cov) == 36
    assert cov[14] == 0.05          # z: the one we mean
    for i in (0, 7, 21, 28, 35):    # x, y, roll, pitch, yaw
        assert cov[i] >= 1e6
