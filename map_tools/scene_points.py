"""Assemble ONE map-frame point cloud from every catalog object in the park.

The unique-landmark-waypoint localizer identifies distinctive REGIONS of
space (not individual objects) by describing the points inside a spatial
window and matching against an offline map. Before any window can be cut, the
whole scene needs to exist as one combined point cloud in map-frame
coordinates -- that is what this module builds. Pure geometry: meshes + their
world poses in, one (N,3) array out. No landmark logic, no ROS.

Mesh resolution mirrors what the two extractors already do, but centralised
here instead of duplicated per-family:
  - Most catalog types carry `mesh=(rel_path_parts, scale)` directly in the
    registry (bench, garden_table, lamp, trash_bin_1) -- resolved the same
    way extract_park_map.BOX_MESHES does, under models_opt/.
  - tree_8 has mesh=None in the registry (see park_types.py: trees are
    identified by vertical profile, not a mesh signature) but its trunk asset
    is `models_opt/tree_8/bark8.obj`, a Wavefront mesh at scale 1.0
    (~7.89 m tall -- verified against the file).
  - postescable also has mesh=None (same reason: identified by profile) but
    its asset is `models_lake_opt/linea1/postes.dae` at scale 0.03 -- the
    same file extract_lake_map.py's POLE_OFFSETS were measured from.
    Unlike extract_lake_map, this module does NOT call _expand_poles or crop
    to a single pole: the scene cloud is the whole undivided scene, so the
    raw mesh (both poles + the cable span) is placed once at the model pose.
    Windowing/cropping is the later location-grid task's job, not this one.

Families with no resolvable mesh by either route (arbolpartes4, obstacle-only
in the registry) are skipped, with the reason logged -- never crashed on,
never silently dropped.
"""
import math
import os

import numpy as np

from map_tools.mesh_sample import sample_surface, sample_triangles
from map_tools.obj_read import read_obj_triangles
from map_tools.park_types import BY_PREFIX

_MAP_TOOLS_ROOT = os.path.dirname(__file__)
_REPO_ROOT = os.path.join(_MAP_TOOLS_ROOT, "..")
_MODELS_OPT_ROOT = os.path.join(_REPO_ROOT, "models_opt")

# Extra mesh resolution for families the registry marks mesh=None but which
# DO have a real asset on disk (see module docstring). (path, scale), same
# shape as ParkType.mesh so both routes can be handled uniformly below.
_EXTRA_MESHES = {
    "tree_8": (
        os.path.join(_MODELS_OPT_ROOT, "tree_8", "bark8.obj"), 1.0),
    "postescable": (
        os.path.join(_REPO_ROOT, "models_lake_opt", "linea1", "postes.dae"),
        0.03),
}


def _resolve_mesh(family):
    """Return (path, scale) for `family`, or None if it has no mesh asset."""
    ptype = BY_PREFIX.get(family)
    if ptype is not None and ptype.mesh is not None:
        parts, scale = ptype.mesh
        return os.path.join(_MODELS_OPT_ROOT, *parts), scale
    return _EXTRA_MESHES.get(family)


def sample_model(model, n, seed=0):
    """Return (n, 3) surface points for one Model, in MESH-LOCAL frame.

    Dispatches by the resolved mesh file's extension: .dae -> sample_surface,
    .obj -> read_obj_triangles + sample_triangles (same area-weighted core).
    Families with no resolvable mesh return an empty (0, 3) array, with the
    reason logged.
    """
    resolved = _resolve_mesh(model.family)
    if resolved is None:
        print("scene_points.sample_model: skipping %r (%s) -- no resolvable "
              "mesh asset" % (model.name, model.family))
        return np.zeros((0, 3))

    path, scale = resolved
    ext = os.path.splitext(path)[1].lower()
    if ext == ".dae":
        return sample_surface(path, scale, n=n, seed=seed)
    elif ext == ".obj":
        tris = read_obj_triangles(path, scale=scale)
        return sample_triangles(tris, n, seed=seed)
    else:
        print("scene_points.sample_model: skipping %r (%s) -- unrecognised "
              "mesh extension %r" % (model.name, model.family, ext))
        return np.zeros((0, 3))


def scene_cloud(models, per_object_n=2000, seed=0):
    """Return (N, 3) map-frame point cloud combining every catalog model's
    surface points.

    Each model is sampled in its mesh-local frame via sample_model, then
    rotated by the model's yaw and translated by (world_x, world_y). z is
    left as the mesh gives it (ground-relative, since these are static
    park-world placements with no per-model z offset).

    Deterministic for a fixed seed: every model uses the same `seed`, so
    re-running with the same `models`/`per_object_n`/`seed` reproduces the
    cloud exactly (order of `models` is preserved, nothing depends on dict
    iteration order).
    """
    clouds = []
    for m in models:
        pts = sample_model(m, per_object_n, seed=seed)
        if len(pts) == 0:
            continue
        cos_yaw, sin_yaw = math.cos(m.yaw), math.sin(m.yaw)
        x = pts[:, 0] * cos_yaw - pts[:, 1] * sin_yaw + m.world_x
        y = pts[:, 0] * sin_yaw + pts[:, 1] * cos_yaw + m.world_y
        z = pts[:, 2]
        clouds.append(np.column_stack([x, y, z]))

    if not clouds:
        return np.zeros((0, 3))
    return np.concatenate(clouds, axis=0)
