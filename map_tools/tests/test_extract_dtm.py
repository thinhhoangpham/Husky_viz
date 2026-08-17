"""Extractor tests: scale and pose application, and the water box geometry."""
import os

import numpy as np
import pytest

from map_tools import extract_dtm
from map_tools.dtm_raster import DtmGrid, rasterize

MAPS = os.path.join(os.path.dirname(__file__), "..", "..", "maps")


def test_world_registry_matches_the_world_files():
    """The mesh path and scale are hardcoded from the .world; if the world is
    re-saved with a different scale this must fail rather than silently
    rasterise at the old one."""
    import re
    for key, spec in extract_dtm.WORLDS.items():
        assert os.path.exists(spec.mesh_path), spec.mesh_path
        text = open(spec.world_file).read()
        mesh_name = os.path.basename(spec.mesh_path)
        # Find the <scale> that accompanies this mesh's <uri> in the world.
        pat = re.compile(
            r"<uri>model://[^<]*%s</uri>\s*<scale>([^<]+)</scale>"
            % re.escape(mesh_name))
        found = {tuple(float(v) for v in m.group(1).split())
                 for m in pat.finditer(text)}
        assert found, "no <scale> beside %s in %s" % (mesh_name, key)
        assert found == {spec.scale}, (key, found, spec.scale)


def test_scale_and_pose_are_applied_to_terrain_triangles():
    """Rebuild the transform independently and require the extractor to match.

    Guards the whole chain: node matrices, the per-axis mesh <scale>, then the
    effective model <pose> (state block, not the definition).
    """
    from map_tools.mesh_bounds import _triangles
    from map_tools.model_pose import resolve_pose_from_file

    spec = extract_dtm.WORLDS["park"]
    tris, pose = extract_dtm.load_terrain_triangles(spec)

    raw = _triangles(spec.mesh_path, scale=1.0)
    expected_pose = resolve_pose_from_file(spec.world_file, spec.model_name)
    expected = raw * np.asarray(spec.scale) + np.array(
        [expected_pose.x, expected_pose.y, expected_pose.z])

    assert pose.source == "state"
    np.testing.assert_allclose(tris, expected, rtol=0, atol=1e-9)


def test_park_scale_flattens_z_by_a_hundredfold():
    """park's <scale> z is 0.01. The raw mesh has ~0.69 of z range; scaled it
    must be ~0.0069 -- this is the deliberate flattening the whole park-vs-lake
    comparison is about."""
    from map_tools.mesh_bounds import _triangles
    spec = extract_dtm.WORLDS["park"]
    raw = _triangles(spec.mesh_path, scale=1.0)
    raw_relief = raw[:, :, 2].max() - raw[:, :, 2].min()
    tris, _pose = extract_dtm.load_terrain_triangles(spec)
    scaled_relief = tris[:, :, 2].max() - tris[:, :, 2].min()
    assert scaled_relief == pytest.approx(raw_relief * 0.01, rel=1e-9)
    assert scaled_relief < 0.01


def test_pose_z_offset_lands_terrain_at_the_state_height():
    """park's state pose raises the terrain to z ~2.99; the DTM must sit there,
    not at 0."""
    spec = extract_dtm.WORLDS["park"]
    tris, pose = extract_dtm.load_terrain_triangles(spec)
    assert pose.z == pytest.approx(2.98891, abs=1e-5)
    mid = 0.5 * (tris[:, :, 2].max() + tris[:, :, 2].min())
    assert mid == pytest.approx(pose.z, abs=0.01)


def test_rotated_terrain_is_refused_not_silently_ignored():
    """Non-zero roll/pitch/yaw is unsupported; ignoring it would rasterise
    tilted terrain as if it were flat."""
    from map_tools.model_pose import Pose

    spec = extract_dtm.WORLDS["park"]
    real = extract_dtm.resolve_pose_from_file

    def fake(_path, _name):
        return Pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.5, "state")

    extract_dtm.resolve_pose_from_file = fake
    try:
        with pytest.raises(NotImplementedError):
            extract_dtm.load_terrain_triangles(spec)
    finally:
        extract_dtm.resolve_pose_from_file = real


def test_water_box_size_is_read_from_the_world():
    text = open(extract_dtm.WORLDS["lake"].world_file).read()
    size = extract_dtm._box_size(text, "lago")
    assert size == pytest.approx((75.0997, 37.8189, 8.6983))


def test_water_layer_is_flat_and_uses_the_state_pose():
    """Water top = state pose z + half the box height. With the definition
    pose (z=0) it would come out ~0.83 m too high."""
    grid = DtmGrid(np.full((200, 398), np.nan, dtype=np.float32),
                   0.25, -49.75, -25.0)
    wgrid, info = extract_dtm.water_layer("lake", grid)

    assert info["pose"].source == "state"
    assert info["pose"].z == pytest.approx(-0.828242, abs=1e-6)
    assert info["top_z"] == pytest.approx(-0.828242 + 8.6983 / 2.0, abs=1e-6)

    vals = wgrid.z[np.isfinite(wgrid.z)]
    assert vals.size > 0
    # Flat by construction: one distinct height everywhere it exists.
    assert np.ptp(vals) == pytest.approx(0.0, abs=1e-6)
    assert vals[0] == pytest.approx(info["top_z"], abs=1e-4)


def test_water_layer_shares_the_dtm_grid_geometry():
    """Both layers must be cell-for-cell comparable in RViz."""
    grid = DtmGrid(np.full((200, 398), np.nan, dtype=np.float32),
                   0.25, -49.75, -25.0)
    wgrid, _info = extract_dtm.water_layer("lake", grid)
    assert wgrid.z.shape == grid.z.shape
    assert wgrid.resolution == grid.resolution
    assert wgrid.origin_x == grid.origin_x
    assert wgrid.origin_y == grid.origin_y


def test_water_outside_the_box_footprint_is_nan():
    """No water beyond the box; 0.0 there would paint a sheet across the map."""
    grid = DtmGrid(np.full((200, 398), np.nan, dtype=np.float32),
                   0.25, -49.75, -25.0)
    wgrid, info = extract_dtm.water_layer("lake", grid)
    X, Y = wgrid.cell_centers()
    outside = (np.abs(X - info["pose"].x) > info["size"][0] / 2.0)
    assert outside.any()
    assert np.isnan(wgrid.z[outside]).all()


def test_park_terrain_uses_the_lowpoly_mesh():
    """park.world uses lowpoly for BOTH visual and collision, so it is the
    simulated surface."""
    assert extract_dtm.WORLDS["park"].mesh_path.endswith(
        "terreno_parque_lowpoly.dae")


def test_lake_terrain_uses_the_collision_mesh():
    """lake.world's collision is lago.dae, NOT the lowpoly visual. The robot's
    wheels and the lidar rays interact with collision geometry."""
    spec = extract_dtm.WORLDS["lake"]
    assert spec.mesh_path.endswith("lago.dae")
    assert "lowpoly" not in os.path.basename(spec.mesh_path)


# ------------------------------------------------- end-to-end on real output

def test_generated_park_dtm_is_flat_and_mostly_covered():
    npy = os.path.join(MAPS, "park_dtm.npy")
    if not os.path.exists(npy):
        pytest.skip("run: python3 -m map_tools.extract_dtm --world park")
    z = np.load(npy)
    assert z.dtype == np.float32
    finite = z[np.isfinite(z)]
    relief = finite.max() - finite.min()
    # The measured park relief, ~7 mm.
    assert relief == pytest.approx(0.0069, abs=0.001)
    assert np.isfinite(z).mean() > 0.95


def test_generated_lake_has_far_more_relief_than_park():
    """The headline comparison: lake relief must dwarf park's."""
    lake = os.path.join(MAPS, "lake_dtm.npy")
    park = os.path.join(MAPS, "park_dtm.npy")
    if not (os.path.exists(lake) and os.path.exists(park)):
        pytest.skip("generate both DTMs first")
    lz = np.load(lake)[np.isfinite(np.load(lake))]
    pz = np.load(park)[np.isfinite(np.load(park))]
    lake_relief = lz.max() - lz.min()
    park_relief = pz.max() - pz.min()
    assert lake_relief > 2.0
    assert lake_relief > 100 * park_relief
