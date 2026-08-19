import numpy as np
import pytest
from map_tools.slope_costmap import slope_degrees


def test_flat_terrain_is_zero_slope():
    heights = np.full((10, 10), 3.0, dtype=np.float32)
    out = slope_degrees(heights, 0.25)
    assert np.allclose(out, 0.0)


def test_constant_ramp_matches_analytic_angle():
    # 1 m rise per 1 m run along +x => 45 degrees, regardless of resolution.
    res = 0.25
    cols = np.arange(20) * res
    heights = np.tile(cols, (20, 1)).astype(np.float32)
    out = slope_degrees(heights, res)
    # Interior cells only: np.gradient uses one-sided differences at edges.
    assert np.allclose(out[1:-1, 1:-1], 45.0, atol=1e-6)


def test_known_shallow_gradient():
    # 0.1 m rise per 1.0 m run => atan(0.1) = 5.7106 degrees
    res = 0.5
    cols = np.arange(12) * res * 0.1
    heights = np.tile(cols, (12, 1)).astype(np.float32)
    out = slope_degrees(heights, res)
    assert out[5, 5] == pytest.approx(np.degrees(np.arctan(0.1)), abs=1e-6)


def test_nan_is_preserved_not_zero():
    heights = np.full((8, 8), 2.0, dtype=np.float32)
    heights[4, 4] = np.nan
    out = slope_degrees(heights, 0.25)
    assert np.isnan(out[4, 4])


def test_slope_is_direction_agnostic():
    # Uphill and downhill of equal steepness get the SAME magnitude.
    res = 0.25
    up = np.tile(np.arange(16) * res, (16, 1)).astype(np.float32)
    down = up[:, ::-1].copy()
    a = slope_degrees(up, res)
    b = slope_degrees(down, res)
    assert np.allclose(a[1:-1, 1:-1], b[1:-1, 1:-1])
