import numpy as np
from landmark_loc import terrain_match


def _synthetic_dtm(tmp_path):
    # a smooth ramp+bump prior, 40x40 at 0.5 m
    ys, xs = np.mgrid[0:40, 0:40].astype(float)
    z = 0.05 * xs + 0.03 * ys
    z[20:24, 20:24] += 1.0  # a distinctive bump
    npy = tmp_path / "p_dtm.npy"
    yml = tmp_path / "p_dtm.yaml"
    np.save(npy, z.astype(np.float32))
    yml.write_text(
        "resolution: 0.5\norigin_x: 0.0\norigin_y: 0.0\n"
        "width: 40\nheight: 40\n")
    return npy, yml, z


def test_load_dtm_reads_geometry(tmp_path):
    npy, yml, z = _synthetic_dtm(tmp_path)
    d = terrain_match.load_dtm(str(npy), str(yml))
    assert d.resolution == 0.5
    assert d.z.shape == (40, 40)
    assert d.origin_x == 0.0


def test_gradient_nan_aware():
    z = np.array([[0.0, 1.0, np.nan],
                  [0.0, 1.0, 2.0]], dtype=float)
    gx, gy = terrain_match.gradient(z)
    assert np.isfinite(gx[1, 0])         # both neighbours finite
    assert np.isnan(gx[0, 1])            # right neighbour is NaN


def test_match_finds_zero_offset_when_local_equals_prior_patch(tmp_path):
    npy, yml, z = _synthetic_dtm(tmp_path)
    d = terrain_match.load_dtm(str(npy), str(yml))
    # local patch = exact copy of prior rows 10:30, cols 10:30
    local = z[10:30, 10:30].astype(np.float32).copy()
    # its true map-frame origin corner:
    true_x = 10 * 0.5
    true_y = 10 * 0.5
    # prior guess a bit off; matcher should recover the true patch center
    cx = true_x + 20 * 0.5 * 0.5  # patch-center-ish prior
    cy = true_y + 20 * 0.5 * 0.5
    res = terrain_match.match_terrain(local, 0.5, d, (cx, cy), search_radius_m=3.0)
    assert res is not None
    x, y, score = res
    # recovered center should sit within a cell of the true patch center
    tcx = true_x + (local.shape[1] * 0.5) / 2.0
    tcy = true_y + (local.shape[0] * 0.5) / 2.0
    assert abs(x - tcx) <= 0.5
    assert abs(y - tcy) <= 0.5
    assert score > 0.9


def test_match_returns_none_on_too_few_valid_cells():
    z = np.full((40, 40), np.nan, dtype=np.float32)
    class _D:
        pass
    d = _D(); d.z = z; d.resolution = 0.5; d.origin_x = 0.0; d.origin_y = 0.0
    local = np.full((10, 10), np.nan, dtype=np.float32)
    assert terrain_match.match_terrain(local, 0.5, d, (5.0, 5.0), 3.0) is None


def test_match_is_offset_invariant(tmp_path):
    # Adding a constant to the local grid must NOT change the recovered match.
    # This is the core "absolute z never assumed" property.
    npy, yml, z = _synthetic_dtm(tmp_path)
    d = terrain_match.load_dtm(str(npy), str(yml))
    local = z[10:30, 10:30].astype(np.float32).copy()
    cx = 10 * 0.5 + (local.shape[1] * 0.5) / 2.0
    cy = 10 * 0.5 + (local.shape[0] * 0.5) / 2.0
    base = terrain_match.match_terrain(local, 0.5, d, (cx, cy), search_radius_m=3.0)
    shifted = terrain_match.match_terrain(local + 7.3, 0.5, d, (cx, cy), search_radius_m=3.0)
    assert base is not None and shifted is not None
    # recovered (x, y) identical; score identical to floating-point tolerance
    assert abs(base[0] - shifted[0]) < 1e-9
    assert abs(base[1] - shifted[1]) < 1e-9
    assert abs(base[2] - shifted[2]) < 1e-9


import os
import pytest

_LAKE = "/home/thinh/Documents/Husky_viz/maps/lake_dtm.npy"


@pytest.mark.skipif(not os.path.exists(_LAKE), reason="lake_dtm not generated")
def test_self_match_on_real_lake_dtm():
    d = terrain_match.load_dtm(_LAKE, _LAKE.replace(".npy", ".yaml"))
    # cut a 30x30 patch out of the middle and match it back to itself
    r, c = d.z.shape[0] // 2, d.z.shape[1] // 2
    local = d.z[r:r + 30, c:c + 30].astype(np.float32).copy()
    tcx = d.origin_x + (c + 15) * d.resolution
    tcy = d.origin_y + (r + 15) * d.resolution
    res = terrain_match.match_terrain(local, d.resolution, d, (tcx, tcy), 4.0)
    assert res is not None
    x, y, score = res
    assert abs(x - tcx) <= d.resolution
    assert abs(y - tcy) <= d.resolution
