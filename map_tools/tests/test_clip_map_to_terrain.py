import numpy as np
import pytest

from map_tools.clip_map_to_terrain import (
    compute_clip, read_pgm, write_pgm, build, main,
)


def _fake_world(tmp_path, name, map_pixels, map_res, map_ox, map_oy,
                 dtm_ox, dtm_oy, dtm_res, dtm_width, dtm_height):
    """Write a minimal <name>_map.{yaml,pgm} + <name>_dtm.yaml set.

    map_pixels: internal convention array, row 0 = LOWEST y.
    """
    maps = tmp_path / "maps"
    maps.mkdir(exist_ok=True)

    write_pgm(str(maps / ("%s_map.pgm" % name)), map_pixels)
    (maps / ("%s_map.yaml" % name)).write_text(
        "image: %s_map.pgm\nresolution: %f\n"
        "origin: [%f, %f, 0.0]\nnegate: 0\n"
        "occupied_thresh: 0.65\nfree_thresh: 0.196\n"
        % (name, map_res, map_ox, map_oy))

    (maps / ("%s_dtm.yaml" % name)).write_text(
        "# comment line that must be skipped\n"
        "layer: terrain\n"
        "resolution: %f\norigin_x: %f\norigin_y: %f\n"
        "width: %d\nheight: %d\n"
        % (dtm_res, dtm_ox, dtm_oy, dtm_width, dtm_height))

    return maps


def test_crop_is_exact_world_coordinates_preserved():
    # Object map: 10x10 cells at res=1.0, origin (0,0) -> extent [0,10]x[0,10].
    # DTM footprint: origin (2,2), 4x4 cells at res=1.0 -> [2,6]x[2,6].
    # A naive off-by-one in the crop start would shift world coords by 1 cell.
    map_pixels = np.full((10, 10), 254, dtype=np.uint8)
    # Occupied cell at world (3.5, 3.5) -> row=3, col=3 (inside footprint).
    map_pixels[3, 3] = 0

    clipped, result = compute_clip(
        map_pixels, map_ox=0.0, map_oy=0.0, map_res=1.0,
        dtm_ox=2.0, dtm_oy=2.0, dtm_res=1.0, dtm_width=4, dtm_height=4)

    assert result.clip_origin_x == 2.0
    assert result.clip_origin_y == 2.0
    # World (3.5, 3.5) in the clipped grid: col = (3.5-2)/1 = 1, row = 1.
    assert clipped[1, 1] == 0
    # Verify against original: only one occupied cell total, and it survived.
    assert (clipped == 0).sum() == 1


def test_cells_outside_footprint_are_discarded_and_counted():
    map_pixels = np.full((10, 10), 254, dtype=np.uint8)
    map_pixels[3, 3] = 0   # world (3.5, 3.5) -- inside DTM footprint [2,6]
    map_pixels[8, 8] = 0   # world (8.5, 8.5) -- outside DTM footprint

    clipped, result = compute_clip(
        map_pixels, map_ox=0.0, map_oy=0.0, map_res=1.0,
        dtm_ox=2.0, dtm_oy=2.0, dtm_res=1.0, dtm_width=4, dtm_height=4)

    assert result.orig_occupied_count == 2
    assert result.discarded_count == 1
    assert (clipped == 0).sum() == 1
    assert result.discarded_pct == pytest.approx(50.0)


def test_map_smaller_than_terrain_is_noop():
    # Object map entirely inside a larger DTM footprint.
    map_pixels = np.full((5, 5), 254, dtype=np.uint8)
    map_pixels[2, 2] = 0

    clipped, result = compute_clip(
        map_pixels, map_ox=1.0, map_oy=1.0, map_res=1.0,
        dtm_ox=-10.0, dtm_oy=-10.0, dtm_res=1.0, dtm_width=100, dtm_height=100)

    assert result.unchanged is True
    assert clipped.shape == map_pixels.shape
    assert np.array_equal(clipped, map_pixels)
    assert result.discarded_count == 0
    assert result.clip_origin_x == 1.0
    assert result.clip_origin_y == 1.0


def test_clipped_origin_stays_on_original_cell_grid():
    map_pixels = np.full((20, 20), 254, dtype=np.uint8)
    map_ox, map_oy, map_res = -5.0, -5.0, 0.25
    # DTM origin does NOT land exactly on a map cell boundary.
    dtm_ox, dtm_oy = -3.37, -3.12

    clipped, result = compute_clip(
        map_pixels, map_ox, map_oy, map_res,
        dtm_ox, dtm_oy, dtm_res=0.25, dtm_width=8, dtm_height=8)

    # (clip_origin - map_origin) / map_res must be a whole number.
    dx_cells = (result.clip_origin_x - map_ox) / map_res
    dy_cells = (result.clip_origin_y - map_oy) / map_res
    assert dx_cells == pytest.approx(round(dx_cells), abs=1e-9)
    assert dy_cells == pytest.approx(round(dy_cells), abs=1e-9)


def test_resolution_is_preserved(tmp_path):
    map_pixels = np.full((20, 20), 254, dtype=np.uint8)
    maps = _fake_world(tmp_path, "resworld", map_pixels,
                       map_res=0.15, map_ox=-5.0, map_oy=-5.0,
                       dtm_ox=-2.0, dtm_oy=-2.0, dtm_res=0.25,
                       dtm_width=8, dtm_height=8)
    result = build("resworld", str(maps))
    text = (maps / "resworld_map_clipped.yaml").read_text()
    assert "resolution: 0.150000" in text


def test_dry_run_writes_nothing(tmp_path):
    map_pixels = np.full((20, 20), 254, dtype=np.uint8)
    map_pixels[10, 10] = 0
    maps = _fake_world(tmp_path, "dryworld", map_pixels,
                       map_res=1.0, map_ox=0.0, map_oy=0.0,
                       dtm_ox=2.0, dtm_oy=2.0, dtm_res=1.0,
                       dtm_width=10, dtm_height=10)
    ret = main(["dryworld", "--maps-dir", str(maps), "--dry-run"])
    assert ret == 0
    assert not (maps / "dryworld_map_clipped.pgm").exists()
    assert not (maps / "dryworld_map_clipped.yaml").exists()


def test_written_pgm_roundtrips_occupied_cells(tmp_path):
    map_pixels = np.full((20, 20), 254, dtype=np.uint8)
    map_pixels[5, 5] = 0
    map_pixels[6, 7] = 0
    map_pixels[15, 15] = 0   # will be clipped out
    maps = _fake_world(tmp_path, "rtworld", map_pixels,
                       map_res=1.0, map_ox=0.0, map_oy=0.0,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=10, dtm_height=10)
    build("rtworld", str(maps))

    reread = read_pgm(str(maps / "rtworld_map_clipped.pgm"))
    expected = map_pixels[0:10, 0:10]
    assert np.array_equal(reread, expected)
    occ_rows, occ_cols = np.where(reread == 0)
    assert sorted(zip(occ_rows.tolist(), occ_cols.tolist())) == [(5, 5), (6, 7)]


def test_build_reports_expected_dims_and_discard_counts(tmp_path):
    map_pixels = np.full((10, 10), 254, dtype=np.uint8)
    map_pixels[1, 1] = 0
    map_pixels[9, 9] = 0
    maps = _fake_world(tmp_path, "reportworld", map_pixels,
                       map_res=1.0, map_ox=0.0, map_oy=0.0,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=5, dtm_height=5)
    result = build("reportworld", str(maps))
    assert (result.orig_width, result.orig_height) == (10, 10)
    assert (result.clip_width, result.clip_height) == (5, 5)
    assert result.discarded_count == 1
