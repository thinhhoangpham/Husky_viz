"""Rasteriser tests on synthetic meshes whose height is known analytically."""
import numpy as np
import pytest

from map_tools.dtm_raster import DtmGrid, grid_bounds, rasterize


def _quad(x0, y0, x1, y1, zfn):
    """Two triangles tiling the rectangle, with z from zfn(x, y)."""
    c = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    p = [(x, y, zfn(x, y)) for x, y in c]
    return np.array([[p[0], p[1], p[2]], [p[0], p[2], p[3]]], dtype=float)


def test_flat_plane_has_constant_height():
    tris = _quad(0.0, 0.0, 4.0, 2.0, lambda x, y: 3.5)
    g = rasterize(tris, resolution=0.5)
    assert g.width == 8 and g.height == 4
    assert np.isfinite(g.z).all()
    np.testing.assert_allclose(g.z, 3.5, atol=1e-6)
    n_valid, total, z_min, z_max = g.stats()
    assert n_valid == total == 32
    assert z_max - z_min == pytest.approx(0.0, abs=1e-6)


def test_known_slope_interpolates_at_cell_centers():
    """z = 2x on [0,4]x[0,2]: each cell center must get exactly 2 * center_x."""
    tris = _quad(0.0, 0.0, 4.0, 2.0, lambda x, y: 2.0 * x)
    g = rasterize(tris, resolution=0.5)
    X, _Y = g.cell_centers()
    np.testing.assert_allclose(g.z, 2.0 * X, atol=1e-6)
    # Centers run 0.25..3.75, so relief is 2*(3.75-0.25) = 7.0, not 2*4.
    _n, _t, z_min, z_max = g.stats()
    assert z_min == pytest.approx(0.5, abs=1e-6)
    assert z_max == pytest.approx(7.5, abs=1e-6)


def test_uncovered_cells_are_nan_not_zero():
    """A mesh in one corner must leave the rest NaN. Zero would forge a
    flat plateau, which is the whole reason NaN is used."""
    tris = _quad(0.0, 0.0, 1.0, 1.0, lambda x, y: 5.0)
    # Force a grid four times as wide/tall as the covered patch.
    g = rasterize(tris, resolution=0.5, origin=(0.0, 0.0), shape=(4, 4))
    assert g.width == 4 and g.height == 4
    covered = np.isfinite(g.z)
    assert covered[:2, :2].all()
    # Everything outside the patch is NaN, and specifically not 0.0.
    assert not covered[2:, :].any()
    assert not covered[:, 2:].any()
    assert np.isnan(g.z[3, 3])
    assert (g.z[covered] == 5.0).all()


def test_max_z_wins_when_two_surfaces_overlap():
    """Overlapping surfaces: the upper one must be reported, regardless of the
    order the triangles arrive in."""
    low = _quad(0.0, 0.0, 2.0, 2.0, lambda x, y: 1.0)
    high = _quad(0.0, 0.0, 2.0, 2.0, lambda x, y: 4.0)
    a = rasterize(np.concatenate([low, high]), resolution=0.5)
    b = rasterize(np.concatenate([high, low]), resolution=0.5)
    np.testing.assert_allclose(a.z, 4.0, atol=1e-6)
    np.testing.assert_allclose(a.z, b.z, atol=1e-6)


def test_negative_heights_survive_the_nan_seeding():
    """Cells are seeded with -inf before the max reduction; a genuinely
    negative surface must still come out with its real value."""
    tris = _quad(0.0, 0.0, 2.0, 2.0, lambda x, y: -7.25)
    g = rasterize(tris, resolution=0.5)
    assert np.isfinite(g.z).all()
    np.testing.assert_allclose(g.z, -7.25, atol=1e-6)


def test_chunking_does_not_change_the_result():
    rng = np.random.RandomState(0)
    tris = np.concatenate([
        _quad(float(i), 0.0, float(i) + 1.0, 3.0,
              lambda x, y, i=i: 0.3 * x + 0.1 * y + rng.rand())
        for i in range(12)])
    big = rasterize(tris, resolution=0.25, chunk=10000)
    small = rasterize(tris, resolution=0.25, chunk=3)
    np.testing.assert_allclose(big.z, small.z, atol=1e-6, equal_nan=True)


def test_degenerate_triangles_are_skipped_not_crashed():
    """Zero-xy-area faces (vertical walls edge-on) have no barycentric basis."""
    good = _quad(0.0, 0.0, 2.0, 2.0, lambda x, y: 1.0)
    vertical = np.array([[[0.5, 0.5, 0.0], [0.5, 0.5, 9.0],
                          [0.5, 1.5, 9.0]]], dtype=float)
    collapsed = np.array([[[1.0, 1.0, 3.0]] * 3], dtype=float)
    g = rasterize(np.concatenate([good, vertical, collapsed]), resolution=0.5)
    # The vertical wall is edge-on in plan view, so it contributes no height
    # even though its z reaches 9.0.
    np.testing.assert_allclose(g.z, 1.0, atol=1e-6)


def test_origin_snaps_to_resolution_multiple():
    """Two meshes at the same resolution must share cell boundaries so their
    grids can be compared cell-for-cell (terrain vs water)."""
    tris = _quad(-3.3, -1.1, 2.0, 2.0, lambda x, y: 0.0)
    ox, oy, _w, _h = grid_bounds(tris, 0.25)
    assert ox == pytest.approx(-3.5)
    assert oy == pytest.approx(-1.25)
    assert (ox / 0.25) == pytest.approx(round(ox / 0.25))


def test_forced_grid_geometry_is_honoured():
    tris = _quad(0.0, 0.0, 1.0, 1.0, lambda x, y: 2.0)
    g = rasterize(tris, resolution=0.5, origin=(-2.0, -2.0), shape=(10, 8))
    assert (g.height, g.width) == (10, 8)
    assert g.origin_x == -2.0 and g.origin_y == -2.0


def test_cell_centers_match_grid_geometry():
    g = DtmGrid(np.zeros((3, 4), dtype=np.float32), 0.5, 1.0, -2.0)
    X, Y = g.cell_centers()
    assert X.shape == (3, 4) and Y.shape == (3, 4)
    assert X[0, 0] == pytest.approx(1.25)
    assert Y[0, 0] == pytest.approx(-1.75)
    assert X[2, 3] == pytest.approx(2.75)
    assert Y[2, 3] == pytest.approx(-0.75)


def test_empty_stats_are_nan_not_an_exception():
    g = DtmGrid(np.full((2, 2), np.nan, dtype=np.float32), 0.25, 0.0, 0.0)
    n_valid, total, z_min, z_max = g.stats()
    assert n_valid == 0 and total == 4
    assert np.isnan(z_min) and np.isnan(z_max)


def test_bad_inputs_are_rejected():
    tris = _quad(0.0, 0.0, 1.0, 1.0, lambda x, y: 0.0)
    with pytest.raises(ValueError):
        rasterize(tris, resolution=0.0)
    with pytest.raises(ValueError):
        rasterize(np.zeros((5, 2, 3)), resolution=0.5)
    with pytest.raises(ValueError):
        rasterize(np.zeros((0, 3, 3)), resolution=0.5)
