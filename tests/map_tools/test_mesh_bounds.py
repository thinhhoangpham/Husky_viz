import os
from map_tools.mesh_bounds import footprint, footprint_dxdy

MODELS = os.path.join(os.path.dirname(__file__), "..", "..", "models_opt")


def test_bench_footprint_center_offset_from_node_transforms():
    # Regression guard for the node-transform bug: Bench_1.dae's
    # <visual_scene> <node> <matrix> elements translate the geometry ~1.4 m
    # from the mesh origin. Without applying them, the footprint center is
    # wrongly ~(0, 0) and the static-map box lands ~1.2 m off the physical
    # bench.
    _hx, _hy, cx, cy = footprint(os.path.join(MODELS, "bench", "Bench_1.dae"))
    assert abs(cx - 0.389) < 0.1
    assert abs(cy - (-1.407)) < 0.1


def test_bench_footprint_extents_order_of_magnitude():
    hx, hy, _cx, _cy = footprint(os.path.join(MODELS, "bench", "Bench_1.dae"))
    # Transformed-vertex extent (union of all node-transformed sub-meshes),
    # order ~0.83 x 1.78 m -- half extents ~0.4 x 0.89 m.
    assert abs(hx - 0.83 / 2.0) < 0.15
    assert abs(hy - 1.78 / 2.0) < 0.15


def test_trash_bin_footprint_small():
    hx, hy, _cx, _cy = footprint(os.path.join(MODELS, "trash_bin_1", "trash_bin.dae"))
    assert hx < 0.2 and hy < 0.2


def test_garden_table_footprint_at_true_scale():
    # garden_table's SDF mesh <scale> in park.world is 1 1 1, NOT 0.15 like
    # bench -- guards the scale-per-mesh bug (table was ~6.7x too small when
    # 0.15 was wrongly applied to every mesh). Its node transforms carry
    # ~zero xy offset, so the center stays near the mesh origin.
    hx, hy, cx, cy = footprint(
        os.path.join(MODELS, "garden_table", "garden_table.dae"), scale=1.0)
    assert abs(hx - 1.319 / 2.0) < 0.05
    assert abs(hy - 3.000 / 2.0) < 0.05
    assert abs(cx) < 0.05
    assert abs(cy) < 0.05


def test_footprint_dxdy_backward_compat_returns_positive_extents():
    dx, dy = footprint_dxdy(os.path.join(MODELS, "bench", "Bench_1.dae"))
    assert dx > 0 and dy > 0
