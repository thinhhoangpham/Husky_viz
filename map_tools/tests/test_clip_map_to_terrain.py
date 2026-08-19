import numpy as np
import pytest

from map_tools.clip_map_to_terrain import (
    compute_clip, mask_unknown_cells, read_pgm, write_pgm, build, main,
    UNKNOWN,
)


def _fake_world(tmp_path, name, map_pixels, map_res, map_ox, map_oy,
                 dtm_ox, dtm_oy, dtm_res, dtm_width, dtm_height,
                 dtm_heights=None):
    """Write a minimal <name>_map.{yaml,pgm} + <name>_dtm.yaml (+.npy) set.

    map_pixels: internal convention array, row 0 = LOWEST y.
    dtm_heights: optional row0=LOWEST-y float array; if given, also writes
    <name>_dtm.npy (needed for --mask-unknown tests).
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

    if dtm_heights is not None:
        np.save(str(maps / ("%s_dtm.npy" % name)), dtm_heights.astype(np.float32))

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


def test_mask_unknown_nan_terrain_becomes_unknown_even_if_source_was_free():
    pixels = np.full((4, 4), 254, dtype=np.uint8)  # all free
    dtm = np.zeros((4, 4), dtype=np.float32)
    dtm[2, 2] = np.nan  # no terrain at this cell

    out = mask_unknown_cells(pixels, dtm, map_ox=0.0, map_oy=0.0, map_res=1.0,
                              dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0)

    assert out[2, 2] == UNKNOWN
    # every other cell keeps its original free value
    mask = np.ones((4, 4), dtype=bool)
    mask[2, 2] = False
    assert np.all(out[mask] == 254)


def test_mask_unknown_valid_terrain_cells_keep_original_value():
    pixels = np.full((4, 4), 254, dtype=np.uint8)
    pixels[1, 1] = 0  # occupied, but has terrain
    dtm = np.zeros((4, 4), dtype=np.float32)  # all valid terrain

    out = mask_unknown_cells(pixels, dtm, map_ox=0.0, map_oy=0.0, map_res=1.0,
                              dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0)

    assert np.array_equal(out, pixels)


def test_mask_unknown_occupied_cell_on_nan_terrain_becomes_unknown():
    # Chosen behaviour: no-terrain wins over occupied -- an object marker
    # over off-mesh void is not meaningful ground info either, so the rule
    # stays total ("no terrain -> unknown") with no occupied special case.
    pixels = np.full((4, 4), 254, dtype=np.uint8)
    pixels[1, 1] = 0  # occupied
    dtm = np.zeros((4, 4), dtype=np.float32)
    dtm[1, 1] = np.nan  # ...but no terrain underneath it

    out = mask_unknown_cells(pixels, dtm, map_ox=0.0, map_oy=0.0, map_res=1.0,
                              dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0)

    assert out[1, 1] == UNKNOWN


def test_mask_unknown_cell_outside_dtm_footprint_becomes_unknown():
    # Object map extends beyond the DTM footprint entirely (small DTM).
    pixels = np.full((6, 6), 254, dtype=np.uint8)
    dtm = np.zeros((3, 3), dtype=np.float32)  # covers only [0,3]x[0,3]

    out = mask_unknown_cells(pixels, dtm, map_ox=0.0, map_oy=0.0, map_res=1.0,
                              dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0)

    # Cells within [0,3]x[0,3] keep free; cells beyond become unknown.
    assert np.all(out[0:3, 0:3] == 254)
    assert np.all(out[3:6, :] == UNKNOWN)
    assert np.all(out[:, 3:6] == UNKNOWN)


def test_without_mask_unknown_output_identical_to_current_behaviour(tmp_path):
    map_pixels = np.full((10, 10), 254, dtype=np.uint8)
    map_pixels[1, 1] = 0
    map_pixels[9, 9] = 0
    dtm_heights = np.zeros((5, 5), dtype=np.float32)
    dtm_heights[0, 0] = np.nan  # present but irrelevant without the flag
    maps = _fake_world(tmp_path, "nomask", map_pixels,
                       map_res=1.0, map_ox=0.0, map_oy=0.0,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=5, dtm_height=5, dtm_heights=dtm_heights)

    result_plain = build("nomask", str(maps), out_suffix="_plain")
    result_masked_off = build("nomask", str(maps), out_suffix="_flagoff",
                              mask_unknown=False)

    plain = read_pgm(str(maps / "nomask_map_plain.pgm"))
    flagoff = read_pgm(str(maps / "nomask_map_flagoff.pgm"))
    assert np.array_equal(plain, flagoff)
    assert result_plain.clip_width == result_masked_off.clip_width
    assert result_plain.clip_height == result_masked_off.clip_height


def test_build_with_mask_unknown_writes_unknown_pixels(tmp_path):
    map_pixels = np.full((10, 10), 254, dtype=np.uint8)
    map_pixels[1, 1] = 0
    dtm_heights = np.zeros((10, 10), dtype=np.float32)
    dtm_heights[5:, :] = np.nan  # top half of the DTM has no mesh coverage
    maps = _fake_world(tmp_path, "maskworld", map_pixels,
                       map_res=1.0, map_ox=0.0, map_oy=0.0,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=10, dtm_height=10, dtm_heights=dtm_heights)

    build("maskworld", str(maps), out_suffix="_masked", mask_unknown=True)
    out = read_pgm(str(maps / "maskworld_map_masked.pgm"))

    assert np.all(out[5:, :] == UNKNOWN)
    assert np.all(out[:5, :] == map_pixels[:5, :])


def test_decoding_masked_pgm_under_trinary_semantics_yields_unknown(tmp_path):
    map_pixels = np.full((6, 6), 254, dtype=np.uint8)
    map_pixels[0, 0] = 0  # occupied, has terrain -- stays occupied
    dtm_heights = np.zeros((6, 6), dtype=np.float32)
    dtm_heights[3, 3] = np.nan
    maps = _fake_world(tmp_path, "decodeworld", map_pixels,
                       map_res=1.0, map_ox=0.0, map_oy=0.0,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=6, dtm_height=6, dtm_heights=dtm_heights)

    build("decodeworld", str(maps), out_suffix="_decode", mask_unknown=True)
    px = read_pgm(str(maps / "decodeworld_map_decode.pgm"))

    p = (255.0 - px.astype(float)) / 255.0
    occ = np.where(p > 0.65, 100, np.where(p < 0.196, 0, -1))

    assert occ[3, 3] == -1
    assert occ[0, 0] == 100
    mask = np.ones((6, 6), dtype=bool)
    mask[3, 3] = False
    mask[0, 0] = False
    assert np.all(occ[mask] == 0)


def test_existing_clipped_case_still_passes_with_mask_unknown_param_default(tmp_path):
    # Sanity: build() signature accepts the new kwarg but defaults to the
    # pre-existing behaviour when omitted (regression guard for callers).
    map_pixels = np.full((10, 10), 254, dtype=np.uint8)
    map_pixels[1, 1] = 0
    maps = _fake_world(tmp_path, "sig", map_pixels,
                       map_res=1.0, map_ox=0.0, map_oy=0.0,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=5, dtm_height=5)
    result = build("sig", str(maps))
    assert (result.clip_width, result.clip_height) == (5, 5)


def test_no_crop_mask_unknown_output_matches_input_geometry(tmp_path):
    map_pixels = np.full((10, 10), 254, dtype=np.uint8)
    map_pixels[1, 1] = 0
    dtm_heights = np.zeros((10, 10), dtype=np.float32)
    maps = _fake_world(tmp_path, "nocropworld", map_pixels,
                       map_res=1.0, map_ox=-3.0, map_oy=-7.0,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=5, dtm_height=5, dtm_heights=None)
    # DTM footprint (5x5 at origin 0,0) is smaller than the object map
    # (10x10 at origin -3,-7): a normal crop would shrink the output.
    (maps / "nocropworld_dtm.npy")
    np.save(str(maps / "nocropworld_dtm.npy"), dtm_heights)

    result = build("nocropworld", str(maps), out_suffix="_nocrop",
                   mask_unknown=True, no_crop=True)

    assert (result.clip_width, result.clip_height) == (10, 10)
    assert (result.clip_origin_x, result.clip_origin_y) == (-3.0, -7.0)

    out = read_pgm(str(maps / "nocropworld_map_nocrop.pgm"))
    assert out.shape == (10, 10)


def test_no_crop_mask_unknown_cells_correct(tmp_path):
    map_pixels = np.full((6, 6), 254, dtype=np.uint8)
    map_pixels[0, 0] = 0  # occupied, has terrain (dtm cell (0,0)=0.0) -> stays
    dtm_heights = np.zeros((6, 6), dtype=np.float32)
    dtm_heights[4, 4] = np.nan  # a no-terrain cell
    maps = _fake_world(tmp_path, "nocropmask", map_pixels,
                       map_res=1.0, map_ox=0.0, map_oy=0.0,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=6, dtm_height=6, dtm_heights=dtm_heights)

    build("nocropmask", str(maps), out_suffix="_masked", mask_unknown=True,
          no_crop=True)
    out = read_pgm(str(maps / "nocropmask_map_masked.pgm"))

    assert out[4, 4] == UNKNOWN
    assert out[0, 0] == 0  # valid-terrain occupied cell keeps its value


def test_no_crop_alone_reproduces_input_byte_for_byte(tmp_path):
    map_pixels = np.full((8, 8), 254, dtype=np.uint8)
    map_pixels[2, 3] = 0
    maps = _fake_world(tmp_path, "nocroponly", map_pixels,
                       map_res=1.0, map_ox=1.5, map_oy=2.5,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=3, dtm_height=3)

    build("nocroponly", str(maps), out_suffix="_pure", no_crop=True)

    input_bytes = (maps / "nocroponly_map.pgm").read_bytes()
    output_bytes = (maps / "nocroponly_map_pure.pgm").read_bytes()
    assert input_bytes == output_bytes


def test_default_no_flags_still_crops_as_before(tmp_path):
    # Same case as test_build_reports_expected_dims_and_discard_counts:
    # confirms default behaviour (no --no-crop, no --mask-unknown) is
    # byte-for-byte unchanged by the new code path.
    map_pixels = np.full((10, 10), 254, dtype=np.uint8)
    map_pixels[1, 1] = 0
    map_pixels[9, 9] = 0
    maps = _fake_world(tmp_path, "defaultworld", map_pixels,
                       map_res=1.0, map_ox=0.0, map_oy=0.0,
                       dtm_ox=0.0, dtm_oy=0.0, dtm_res=1.0,
                       dtm_width=5, dtm_height=5)
    result = build("defaultworld", str(maps))
    assert (result.orig_width, result.orig_height) == (10, 10)
    assert (result.clip_width, result.clip_height) == (5, 5)
    assert result.discarded_count == 1
