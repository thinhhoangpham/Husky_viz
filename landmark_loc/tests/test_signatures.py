import math
from map_tools import mesh_bounds
from landmark_loc import signatures


def test_bounds3d_returns_six_values_and_positive_extents():
    import os
    root = os.path.join(os.path.dirname(__file__), "..", "..", "models_opt")
    hx, hy, hz, cx, cy, cz = mesh_bounds.bounds3d(
        os.path.join(root, "bench", "Bench_1.dae"), 0.15)
    assert hx > 0 and hy > 0 and hz > 0
    # bench full-extents in metres are all sub-3m at 0.15 scale
    assert 2 * hx < 3.0 and 2 * hy < 3.0 and 2 * hz < 3.0


def test_signatures_cover_four_families_with_ordered_dims():
    for fam in signatures.SIGNATURE_FAMILIES:
        sig = signatures.MESH_SIGNATURES[fam]
        assert sig["major"] >= sig["minor"] > 0
        assert sig["height"] > 0


def test_bench_is_elongated_and_low_lamp_is_thin_and_tall():
    bench = signatures.MESH_SIGNATURES["bench"]
    lamp = signatures.MESH_SIGNATURES["lamp"]
    # bench: high horizontal aspect ratio (a bar)
    assert bench["major"] / bench["minor"] > 1.8
    # lamp: tall relative to its footprint
    assert lamp["height"] / max(lamp["major"], 0.01) > 3.0
