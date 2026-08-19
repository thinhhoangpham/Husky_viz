"""Tests for the DTM cloud publisher's pure logic (no ROS master needed).

The module lives in scripts/, which is not a package, so it is loaded by path.
"""
import importlib.util
import os

import numpy as np
import pytest

_SCRIPT = os.path.join(os.path.dirname(__file__), "..", "..", "scripts",
                       "publish_dtm_cloud.py")

rospy = pytest.importorskip("rospy",
                            reason="ROS not on this interpreter's path")


def _load():
    spec = importlib.util.spec_from_file_location("publish_dtm_cloud", _SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


pdc = _load()


def test_nan_cells_are_skipped_entirely():
    z = np.array([[1.0, np.nan], [np.nan, 2.0]], dtype=np.float32)
    pts = pdc.build_cloud_points(z, 0.5, 0.0, 0.0)
    assert len(pts) == 2
    # No point may sit at a NaN cell's centre, and none may be z=0 filler.
    assert sorted(pts[:, 2].tolist()) == [1.0, 2.0]


def test_point_positions_are_cell_centers_in_world_frame():
    z = np.array([[5.0]], dtype=np.float32)
    pts = pdc.build_cloud_points(z, 0.25, -10.0, 4.0)
    assert len(pts) == 1
    assert pts[0, 0] == pytest.approx(-10.0 + 0.125)
    assert pts[0, 1] == pytest.approx(4.0 + 0.125)
    assert pts[0, 2] == pytest.approx(5.0)


def test_row_zero_is_lowest_y():
    """Matches the extractor's row convention; a flip here would mirror the
    terrain north-south in RViz."""
    z = np.array([[1.0], [2.0]], dtype=np.float32)
    pts = pdc.build_cloud_points(z, 1.0, 0.0, 0.0)
    by_height = {round(float(p[2])): float(p[1]) for p in pts}
    assert by_height[1] < by_height[2]


def test_all_nan_grid_yields_no_points():
    z = np.full((4, 4), np.nan, dtype=np.float32)
    pts = pdc.build_cloud_points(z, 0.5, 0.0, 0.0)
    assert pts.shape == (0, 4)


def test_flat_layer_does_not_divide_by_zero():
    """The water layer is a constant plane, so the ramp's span is zero."""
    z = np.full((3, 3), 3.52, dtype=np.float32)
    pts = pdc.build_cloud_points(z, 0.5, 0.0, 0.0, colormap="water")
    assert len(pts) == 9
    assert np.isfinite(pts).all()


def test_rgb_is_bit_reinterpreted_not_value_cast():
    """PointCloud2 packs rgb as a float32 whose BITS are the uint32. A numeric
    cast would silently destroy the colour."""
    z = np.array([[0.0, 1.0]], dtype=np.float32)
    pts = pdc.build_cloud_points(z, 1.0, 0.0, 0.0)
    rgb_bits = pts[:, 3].astype(np.float32).view(np.uint32)
    # Every channel must be in range and the values must differ across the ramp.
    assert (rgb_bits <= 0xFFFFFF).all()
    assert rgb_bits[0] != rgb_bits[1]


def test_explicit_z_range_makes_two_grids_comparable():
    """Forcing z-min/z-max is how park and lake get a shared colour scale."""
    flat = np.array([[1.0, 1.0]], dtype=np.float32)
    auto = pdc.build_cloud_points(flat, 1.0, 0.0, 0.0)
    forced = pdc.build_cloud_points(flat, 1.0, 0.0, 0.0, z_min=0.0, z_max=10.0)
    # Auto-scaled, a flat grid lands mid-ramp; forced, it lands near the bottom.
    assert auto[0, 3] != forced[0, 3]


def _grid():
    return np.array([[3.5, 4.0, np.nan], [5.0, 5.9, 4.5]], dtype=np.float32)


def test_default_call_publishes_unshifted_z():
    """Regression guard: no flags must mean exactly the old behaviour."""
    z = _grid()
    pts = pdc.build_cloud_points(z, 0.5, 0.0, 0.0)
    assert sorted(pts[:, 2].tolist()) == pytest.approx(
        sorted(z[np.isfinite(z)].tolist()))


def test_z_offset_shifts_every_point_by_the_constant():
    z = _grid()
    base = pdc.build_cloud_points(z, 0.5, 0.0, 0.0)
    shifted = pdc.build_cloud_points(z, 0.5, 0.0, 0.0, z_offset=-5.0)
    assert shifted[:, 2] == pytest.approx(base[:, 2] - 5.0, abs=1e-5)
    # x and y must not move.
    assert shifted[:, :2] == pytest.approx(base[:, :2])


def test_align_min_puts_minimum_at_zero():
    z = _grid()
    off = pdc.compute_align_offset(z, "min")
    pts = pdc.build_cloud_points(z, 0.5, 0.0, 0.0, z_offset=off)
    assert pts[:, 2].min() == pytest.approx(0.0, abs=1e-5)


def test_align_median_puts_median_at_zero():
    z = _grid()
    off = pdc.compute_align_offset(z, "median")
    pts = pdc.build_cloud_points(z, 0.5, 0.0, 0.0, z_offset=off)
    assert float(np.median(pts[:, 2])) == pytest.approx(0.0, abs=1e-5)


def test_align_none_and_all_nan_compute_no_shift():
    assert pdc.compute_align_offset(_grid(), "none") == 0.0
    assert pdc.compute_align_offset(
        np.full((2, 2), np.nan, dtype=np.float32), "min") == 0.0


def test_align_composes_additively_with_z_offset():
    """--z-align min --z-offset 0.5 puts the minimum at +0.5."""
    z = _grid()
    total = 0.5 + pdc.compute_align_offset(z, "min")
    pts = pdc.build_cloud_points(z, 0.5, 0.0, 0.0, z_offset=total)
    assert pts[:, 2].min() == pytest.approx(0.5, abs=1e-5)


def test_colours_are_offset_invariant():
    """The key invariant: a colour means the same elevation at any offset."""
    z = _grid()
    base = pdc.build_cloud_points(z, 0.5, 0.0, 0.0)
    shifted = pdc.build_cloud_points(z, 0.5, 0.0, 0.0, z_offset=-5.0)
    assert (base[:, 3].view(np.uint32) == shifted[:, 3].view(np.uint32)).all()
    forced = pdc.build_cloud_points(z, 0.5, 0.0, 0.0, z_min=0.0, z_max=10.0)
    forced_shifted = pdc.build_cloud_points(z, 0.5, 0.0, 0.0, z_min=0.0,
                                            z_max=10.0, z_offset=-5.0)
    # z-min/z-max stay UNSHIFTED world-frame heights, so these match too.
    assert (forced[:, 3].view(np.uint32)
            == forced_shifted[:, 3].view(np.uint32)).all()


@pytest.mark.parametrize("offset", [-5.9, -1.0, 0.0, 2.5, 100.0])
def test_relief_is_unchanged_by_any_offset(offset):
    z = _grid()
    base = pdc.build_cloud_points(z, 0.5, 0.0, 0.0)[:, 2]
    shifted = pdc.build_cloud_points(z, 0.5, 0.0, 0.0, z_offset=offset)[:, 2]
    assert (shifted.max() - shifted.min()) == pytest.approx(
        base.max() - base.min(), abs=1e-4)


def test_cloud_message_layout():
    z = np.array([[1.0, 2.0], [3.0, np.nan]], dtype=np.float32)
    pts = pdc.build_cloud_points(z, 0.5, 0.0, 0.0)
    msg = pdc.make_cloud_msg(pts, "map", rospy.Time(0))
    assert msg.height == 1
    assert msg.width == 3
    assert msg.point_step == 16
    assert msg.row_step == 48
    assert len(msg.data) == 48
    assert [f.name for f in msg.fields] == ["x", "y", "z", "rgb"]
    assert msg.header.frame_id == "map"
    # Round-trip the buffer and confirm the heights survived.
    back = np.frombuffer(msg.data, dtype=np.float32).reshape(-1, 4)
    assert sorted(back[:, 2].tolist()) == [1.0, 2.0, 3.0]


def test_meta_loader_reads_the_extractor_yaml(tmp_path):
    npy = tmp_path / "x_dtm.npy"
    np.save(str(npy), np.zeros((2, 2), dtype=np.float32))
    (tmp_path / "x_dtm.yaml").write_text(
        "# comment\nlayer: terrain\nresolution: 0.250000\n"
        "origin_x: -50.000000\norigin_y: -26.750000\nwidth: 400\n")
    res, ox, oy = pdc.load_grid_meta(str(npy))
    assert res == pytest.approx(0.25)
    assert ox == pytest.approx(-50.0)
    assert oy == pytest.approx(-26.75)


def test_missing_yaml_raises_rather_than_guessing(tmp_path):
    """Without the .yaml there is no grid geometry; defaulting to (0,0) would
    place the cloud somewhere plausible but wrong."""
    npy = tmp_path / "y_dtm.npy"
    np.save(str(npy), np.zeros((2, 2), dtype=np.float32))
    with pytest.raises(IOError):
        pdc.load_grid_meta(str(npy))


def _slope_rgb(deg_value, other_values=()):
    """rgb888 int for a single slope-degree cell, in a grid that also
    contains `other_values` (to probe range-independence)."""
    z = np.array([[deg_value] + list(other_values)], dtype=np.float32)
    pts = pdc.build_cloud_points(z, 1.0, 0.0, 0.0, colormap="slope")
    rgb_bits = int(pts[0, 3].astype(np.float32).view(np.uint32))
    return (rgb_bits >> 16) & 0xFF, (rgb_bits >> 8) & 0xFF, rgb_bits & 0xFF


def test_slope_safe_band_is_green():
    r, g, b = _slope_rgb(5.0)
    assert g > r and g > b


def test_slope_middle_band_is_between_green_and_red():
    r, g, b = _slope_rgb(14.0)
    assert r > 0 and g > 0
    assert b == 0


def test_slope_lethal_band_is_red():
    r, g, b = _slope_rgb(20.0)
    assert r > g and r > b


def test_slope_ramp_is_absolute_not_range_dependent():
    """The test that would catch a percentile/relative ramp: a 5 deg cell
    must colour identically whether its grid spans nothing or 0-24 deg."""
    narrow = _slope_rgb(5.0, other_values=[5.0])
    wide = _slope_rgb(5.0, other_values=[0.0, 24.0])
    assert narrow == wide


def test_slope_ramp_handles_nan_without_crashing():
    z = np.array([[5.0, np.nan, 20.0]], dtype=np.float32)
    pts = pdc.build_cloud_points(z, 1.0, 0.0, 0.0, colormap="slope")
    assert len(pts) == 2
    assert np.isfinite(pts).all()


def test_grid_meta_override_reads_the_specified_file_not_the_sibling(tmp_path):
    """A slope .npy is on the DTM grid, so --grid-meta must point geometry at
    the DTM's yaml, ignoring whatever (possibly wrong-geometry) yaml sits
    beside the slope .npy itself."""
    npy = tmp_path / "world_slope.npy"
    np.save(str(npy), np.zeros((2, 2), dtype=np.float32))
    # Sibling yaml deliberately carries DIFFERENT geometry from the override,
    # so a passing test proves the override -- not the sibling -- was read.
    (tmp_path / "world_slope.yaml").write_text(
        "resolution: 999.0\norigin_x: 999.0\norigin_y: 999.0\n")
    dtm_yaml = tmp_path / "world_dtm.yaml"
    dtm_yaml.write_text(
        "resolution: 0.250000\norigin_x: -50.000000\norigin_y: -26.750000\n")
    res, ox, oy = pdc.load_grid_meta(str(npy), str(dtm_yaml))
    assert res == pytest.approx(0.25)
    assert ox == pytest.approx(-50.0)
    assert oy == pytest.approx(-26.75)


def test_grid_meta_default_still_reads_the_sibling_yaml(tmp_path):
    """Regression guard: omitting --grid-meta must mean exactly the old
    behaviour of reading the sibling yaml."""
    npy = tmp_path / "x_dtm.npy"
    np.save(str(npy), np.zeros((2, 2), dtype=np.float32))
    (tmp_path / "x_dtm.yaml").write_text(
        "resolution: 0.250000\norigin_x: -50.000000\norigin_y: -26.750000\n")
    res, ox, oy = pdc.load_grid_meta(str(npy))
    assert res == pytest.approx(0.25)
    assert ox == pytest.approx(-50.0)
    assert oy == pytest.approx(-26.75)


def test_map_server_style_sibling_yaml_error_mentions_grid_meta(tmp_path):
    """A map_server-style yaml (image/origin:[x,y,z]/negate/...) has no
    origin_x -- the error must name --grid-meta as the fix, not just list the
    missing keys, since a future caller hitting this needs to know what to do."""
    npy = tmp_path / "world_slope.npy"
    np.save(str(npy), np.zeros((2, 2), dtype=np.float32))
    (tmp_path / "world_slope.yaml").write_text(
        "image: world_slope.pgm\nresolution: 0.250000\n"
        "origin: [-50.0, -26.75, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\nmode: trinary\n")
    with pytest.raises(ValueError, match="--grid-meta"):
        pdc.load_grid_meta(str(npy))


def test_real_park_dtm_round_trips_if_present():
    npy = os.path.join(os.path.dirname(__file__), "..", "..", "maps",
                       "park_dtm.npy")
    if not os.path.exists(npy):
        pytest.skip("park_dtm.npy not generated yet")
    z = np.load(npy)
    res, ox, oy = pdc.load_grid_meta(npy)
    pts = pdc.build_cloud_points(z, res, ox, oy)
    assert len(pts) == int(np.isfinite(z).sum())
    assert np.isfinite(pts[:, :3]).all()
