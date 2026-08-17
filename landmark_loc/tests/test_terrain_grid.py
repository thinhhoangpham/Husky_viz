import numpy as np
from landmark_loc import terrain_grid


def test_bin_min_z_takes_lowest_point_per_cell():
    # two points in the same cell at different z -> min kept
    pts = np.array([[0.1, 0.1, 5.0],
                    [0.2, 0.2, 2.0]], dtype=float)
    g = terrain_grid.bin_min_z(pts, resolution=1.0,
                               origin_x=0.0, origin_y=0.0, width=1, height=1)
    assert g.shape == (1, 1)
    assert abs(g[0, 0] - 2.0) < 1e-9


def test_bin_min_z_empty_cell_is_nan():
    pts = np.array([[0.5, 0.5, 3.0]], dtype=float)
    g = terrain_grid.bin_min_z(pts, resolution=1.0,
                               origin_x=0.0, origin_y=0.0, width=2, height=2)
    assert not np.isnan(g[0, 0])   # cell with the point
    assert np.isnan(g[1, 1])       # empty cell -> NaN, never 0.0


def test_bin_min_z_row_zero_is_lowest_y():
    # a point at low y must land in row 0
    pts = np.array([[0.5, 0.5, 1.0]], dtype=float)
    g = terrain_grid.bin_min_z(pts, resolution=1.0,
                               origin_x=0.0, origin_y=0.0, width=1, height=2)
    assert not np.isnan(g[0, 0])
    assert np.isnan(g[1, 0])


def test_morphological_ground_removes_a_bump():
    # flat ground at z=0 with one tall object cell; opening should flatten it
    z = np.zeros((7, 7), dtype=float)
    z[3, 3] = 5.0  # a "tree"
    ground = terrain_grid.morphological_ground(z, window_cells=3)
    assert abs(ground[3, 3]) < 1e-6  # bump removed


def test_morphological_ground_keeps_a_broad_slope():
    # a slope wider than the window must survive (it is terrain, not an object)
    xs = np.arange(20)
    z = np.tile(xs.astype(float), (20, 1))  # ramp in x
    ground = terrain_grid.morphological_ground(z, window_cells=3)
    # interior values preserved to within the window's reach
    assert abs(ground[10, 10] - z[10, 10]) <= 3.0
