"""Mesh-derived shape signatures for the four identifiable park landmark types.

Each signature is the object's true major/minor horizontal full-extent and full
height, read from its .dae via mesh_bounds.bounds3d. The live classifier
(classify.py) uses these as the CENTERS of its type bands; the +/- margins that
absorb partial lidar views are tuned in-sim and live in classify.py, not here.
Keeping the raw geometry here means the sim and the classifier agree by
construction.
"""
from map_tools.park_types import PARK_TYPES

# Registry views: the mesh signatures now live in the single type registry
# (map_tools.park_types). A "signature family" is any registry type with a mesh
# (bench, garden_table, lamp, trash_bin_1). Names kept for existing importers.
SIGNATURE_FAMILIES = tuple(t.world_prefix for t in PARK_TYPES if t.mesh is not None)

MESH_SIGNATURES = {
    t.world_prefix: t.signature for t in PARK_TYPES if t.mesh is not None
}
