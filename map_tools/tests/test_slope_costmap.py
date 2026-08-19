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


from map_tools.slope_costmap import slope_to_occupancy, UNKNOWN_OCC


def test_below_warn_is_free():
    s = np.array([[0.0, 5.0, 9.9]])
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert list(out[0]) == [0, 0, 0]


def test_at_or_above_lethal_is_full_occupancy():
    s = np.array([[18.0, 18.1, 24.3, 90.0]])
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert list(out[0]) == [100, 100, 100, 100]


def test_graded_band_is_strictly_between():
    s = np.array([[10.0, 14.0, 17.9]])
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert all(1 <= v <= 99 for v in out[0]), list(out[0])


def test_graded_band_is_monotonic():
    s = np.linspace(10.0, 17.9, 40).reshape(1, -1)
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)[0]
    assert list(out) == sorted(out)


def test_band_midpoint_is_near_half():
    s = np.array([[14.0]])  # exact midpoint of 10..18
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert 45 <= out[0, 0] <= 55


def test_nan_becomes_unknown_never_lethal():
    s = np.array([[np.nan, 3.0]])
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert out[0, 0] == UNKNOWN_OCC == -1
    assert out[0, 1] == 0


def test_flat_world_produces_no_cost_at_all():
    # The park case: 0.87 deg max relief must yield a uniformly free layer.
    s = np.full((50, 50), 0.87)
    out = slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)
    assert out.max() == 0


def test_thresholds_are_honoured_not_hardcoded():
    s = np.array([[12.0]])
    assert slope_to_occupancy(s, warn_deg=10.0, lethal_deg=18.0)[0, 0] < 100
    assert slope_to_occupancy(s, warn_deg=5.0, lethal_deg=11.0)[0, 0] == 100


from map_tools.slope_costmap import GridSpec, resample_nearest


def test_identical_grids_roundtrip_unchanged():
    g = GridSpec(origin_x=-1.0, origin_y=-2.0, resolution=0.25, width=8, height=4)
    src = np.arange(32, dtype=np.int16).reshape(4, 8)
    out = resample_nearest(src, g, g)
    assert np.array_equal(out, src)


def test_upsample_preserves_world_position_of_a_marked_cell():
    # Source: 1 m cells, origin (0,0), 4x4. Mark the cell covering world (2.5, 1.5).
    src_grid = GridSpec(0.0, 0.0, 1.0, 4, 4)
    src = np.zeros((4, 4), dtype=np.int16)
    src[1, 2] = 100                      # row=1 -> y in [1,2), col=2 -> x in [2,3)
    dst_grid = GridSpec(0.0, 0.0, 0.5, 8, 8)   # same extent, finer cells
    out = resample_nearest(src, src_grid, dst_grid)
    # World (2.5, 1.5) must still be 100 in the destination.
    col = int((2.5 - dst_grid.origin_x) / dst_grid.resolution)
    row = int((1.5 - dst_grid.origin_y) / dst_grid.resolution)
    assert out[row, col] == 100
    # And a cell far away must not be.
    assert out[0, 0] == 0


def test_offset_origin_is_accounted_for():
    src_grid = GridSpec(-10.0, -10.0, 1.0, 20, 20)
    src = np.zeros((20, 20), dtype=np.int16)
    src[15, 12] = 100                    # world x in [2,3), y in [5,6)
    dst_grid = GridSpec(0.0, 0.0, 1.0, 10, 10)   # different origin
    out = resample_nearest(src, src_grid, dst_grid)
    assert out[5, 2] == 100
    assert out.sum() == 100              # exactly one cell carried over


def test_cells_outside_source_get_fill():
    src_grid = GridSpec(0.0, 0.0, 1.0, 2, 2)
    src = np.zeros((2, 2), dtype=np.int16)
    dst_grid = GridSpec(0.0, 0.0, 1.0, 4, 4)     # extends past the source
    out = resample_nearest(src, src_grid, dst_grid, fill=-1)
    assert out[0, 0] == 0        # inside source
    assert out[3, 3] == -1       # outside source


import os
from map_tools.slope_costmap import (
    occupancy_to_pixels, write_pgm, write_yaml, UNKNOWN_PIXEL,
)


def test_free_occupancy_becomes_white_pixel():
    assert occupancy_to_pixels(np.array([[0]], dtype=np.int16))[0, 0] == 255


def test_lethal_occupancy_becomes_black_pixel():
    assert occupancy_to_pixels(np.array([[100]], dtype=np.int16))[0, 0] == 0


def test_unknown_becomes_the_unknown_pixel():
    out = occupancy_to_pixels(np.array([[-1]], dtype=np.int16))
    assert out[0, 0] == UNKNOWN_PIXEL == 205


def test_inversion_roundtrips_through_map_server_formula():
    # map_server: occ = (255 - px) / 255 * 100
    for occ_in in [0, 1, 25, 50, 75, 99, 100]:
        px = int(occupancy_to_pixels(np.array([[occ_in]], dtype=np.int16))[0, 0])
        occ_back = round((255 - px) / 255.0 * 100)
        assert occ_back == occ_in, (occ_in, px, occ_back)


def test_graded_band_is_monotonically_darker():
    occ = np.array([[0, 25, 50, 75, 100]], dtype=np.int16)
    px = occupancy_to_pixels(occ)[0]
    assert list(px) == sorted(px, reverse=True)


def test_pgm_row_zero_is_highest_y(tmp_path):
    # occ row 0 = LOWEST y. In the file, the FIRST row must be the HIGHEST y.
    occ = np.array([[0, 0], [100, 100]], dtype=np.int16)   # row 1 = high y = lethal
    px = occupancy_to_pixels(occ)
    path = str(tmp_path / "t.pgm")
    write_pgm(path, px)
    with open(path, "rb") as fh:
        data = fh.read()
    body = data.split(b"255\n", 1)[1]
    assert body[0] == 0 and body[1] == 0        # first file row = high y = black
    assert body[2] == 255 and body[3] == 255    # last file row = low y = white


def test_pgm_header_is_binary_p5_with_right_dimensions(tmp_path):
    px = np.zeros((3, 7), dtype=np.uint8)
    path = str(tmp_path / "t.pgm")
    write_pgm(path, px)
    with open(path, "rb") as fh:
        head = fh.read(20)
    assert head.startswith(b"P5\n7 3\n255\n")


def test_yaml_carries_grid_and_thresholds(tmp_path):
    g = GridSpec(-55.4915, -30.9713, 0.15, 100, 80)
    path = str(tmp_path / "t.yaml")
    write_yaml(path, "t.pgm", g, {"warn_deg": 10.0, "lethal_deg": 18.0,
                                  "world": "lake"})
    text = open(path).read()
    assert "image: t.pgm" in text
    assert "resolution: 0.150000" in text
    assert "origin: [-55.491500, -30.971300, 0.0]" in text
    assert "warn_deg" in text and "18.0" in text


import pytest as _pytest  # already imported above; kept for clarity
from map_tools.slope_costmap import read_dtm_yaml, build, main


def _fake_world(tmp_path, name, heights, res=0.25, ox=-5.0, oy=-4.0):
    """Write a minimal <name>_dtm.{npy,yaml} + <name>_map.yaml pair."""
    maps = tmp_path / "maps"
    maps.mkdir(exist_ok=True)
    np.save(str(maps / ("%s_dtm.npy" % name)), heights.astype(np.float32))
    (maps / ("%s_dtm.yaml" % name)).write_text(
        "# comment line that must be skipped\n"
        "layer: terrain\n"
        "resolution: %f\norigin_x: %f\norigin_y: %f\n"
        "width: %d\nheight: %d\n"
        % (res, ox, oy, heights.shape[1], heights.shape[0]))
    (maps / ("%s_map.yaml" % name)).write_text(
        "image: %s_map.pgm\nresolution: 0.150000\n"
        "origin: [%f, %f, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n" % (name, ox, oy))
    return maps


def test_read_dtm_yaml_skips_comments_and_types_values(tmp_path):
    p = tmp_path / "d.yaml"
    p.write_text("# a comment\nlayer: terrain\nresolution: 0.250000\n"
                 "width: 400\nnote: free text here\n")
    got = read_dtm_yaml(str(p))
    assert got["layer"] == "terrain"
    assert got["resolution"] == 0.25
    assert got["width"] == 400
    assert got["note"] == "free text here"


def test_build_on_flat_world_produces_all_free(tmp_path):
    heights = np.full((40, 60), 3.0)
    maps = _fake_world(tmp_path, "flatland", heights)
    stats = build("flatland", str(maps), warn_deg=10.0, lethal_deg=18.0)
    assert stats["lethal_pct"] == 0.0
    assert stats["graded_pct"] == 0.0
    assert stats["free_pct"] == 100.0
    assert os.path.exists(str(maps / "flatland_slope.pgm"))
    assert os.path.exists(str(maps / "flatland_slope.yaml"))
    assert os.path.exists(str(maps / "flatland_slope.npy"))


def test_build_writes_degrees_not_cost_in_the_npy(tmp_path):
    res = 0.25
    heights = np.tile(np.arange(60) * res, (40, 1))   # 45 degree ramp
    maps = _fake_world(tmp_path, "ramp", heights, res=res)
    build("ramp", str(maps), warn_deg=10.0, lethal_deg=18.0)
    saved = np.load(str(maps / "ramp_slope.npy"))
    assert saved.dtype == np.float32
    assert saved[20, 30] == pytest.approx(45.0, abs=1e-4)


def test_build_marks_steep_ground_lethal(tmp_path):
    res = 0.25
    heights = np.tile(np.arange(60) * res, (40, 1))   # 45 deg everywhere
    maps = _fake_world(tmp_path, "steep", heights, res=res)
    stats = build("steep", str(maps), warn_deg=10.0, lethal_deg=18.0)
    assert stats["lethal_pct"] > 90.0


def test_build_propagates_nan_as_unknown(tmp_path):
    heights = np.full((40, 60), 3.0)
    heights[:20, :] = np.nan
    maps = _fake_world(tmp_path, "holey", heights)
    stats = build("holey", str(maps), warn_deg=10.0, lethal_deg=18.0)
    assert stats["unknown_pct"] > 0.0
    assert stats["lethal_pct"] == 0.0     # NaN must never become lethal


def test_main_returns_zero_on_success(tmp_path):
    maps = _fake_world(tmp_path, "cli", np.full((40, 60), 3.0))
    assert main(["cli", "--maps-dir", str(maps)]) == 0


def test_main_rejects_inverted_thresholds(tmp_path):
    maps = _fake_world(tmp_path, "cli2", np.full((40, 60), 3.0))
    assert main(["cli2", "--maps-dir", str(maps),
                 "--warn-deg", "20", "--lethal-deg", "10"]) != 0
