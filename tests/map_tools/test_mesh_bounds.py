import os
from map_tools.mesh_bounds import footprint_dxdy

MODELS = os.path.join(os.path.dirname(__file__), "..", "..", "models_opt")

def test_bench_footprint_is_long_and_scaled():
    dx, dy = footprint_dxdy(os.path.join(MODELS, "bench", "Bench_1.dae"))
    # union of all positions arrays, x0.15: ~0.829 x 1.782 m. Long axis ~1.78 m.
    assert abs(dx - 0.829) < 0.02
    assert abs(dy - 1.782) < 0.02

def test_trash_bin_footprint_small():
    dx, dy = footprint_dxdy(os.path.join(MODELS, "trash_bin_1", "trash_bin.dae"))
    assert dx < 0.2 and dy < 0.2

def test_garden_table_footprint_at_true_scale():
    # garden_table's SDF mesh <scale> in park.world is 1 1 1, NOT 0.15 like
    # bench -- guards the scale-per-mesh bug (table was ~6.7x too small when
    # 0.15 was wrongly applied to every mesh).
    dx, dy = footprint_dxdy(
        os.path.join(MODELS, "garden_table", "garden_table.dae"), scale=1.0)
    assert abs(dx - 1.319) < 0.05
    assert abs(dy - 3.000) < 0.05
