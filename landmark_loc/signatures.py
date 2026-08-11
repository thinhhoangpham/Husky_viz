"""Mesh-derived shape signatures for the four identifiable park landmark types.

Each signature is the object's true major/minor horizontal full-extent and full
height, read from its .dae via mesh_bounds.bounds3d. The live classifier
(classify.py) uses these as the CENTERS of its type bands; the +/- margins that
absorb partial lidar views are tuned in-sim and live in classify.py, not here.
Keeping the raw geometry here means the sim and the classifier agree by
construction.
"""
import os
from map_tools import mesh_bounds

_MODELS_ROOT = os.path.join(os.path.dirname(__file__), "..", "models_opt")

# family -> (relative mesh path, per-mesh scale). Scales: bench/table confirmed
# in extract_park_map.py; lamp/bin default 1.0, pinned against live lidar in the
# in-sim step of this task.
_MESHES = {
    "bench":        (("bench", "Bench_1.dae"), 0.15),
    "garden_table": (("garden_table", "garden_table.dae"), 1.0),
    "lamp":         (("lamp", "street_lamp.dae"), 1.0),
    "trash_bin_1":  (("trash_bin_1", "trash_bin.dae"), 1.0),
}

SIGNATURE_FAMILIES = ("bench", "garden_table", "lamp", "trash_bin_1")


def _signature(rel_parts, scale):
    hx, hy, hz, _, _, _ = mesh_bounds.bounds3d(
        os.path.join(_MODELS_ROOT, *rel_parts), scale)
    dx, dy = 2 * hx, 2 * hy
    return {"major": max(dx, dy), "minor": min(dx, dy), "height": 2 * hz}


MESH_SIGNATURES = {
    fam: _signature(parts, scale) for fam, (parts, scale) in _MESHES.items()
}
