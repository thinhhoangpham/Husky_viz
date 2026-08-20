"""Offline: park.world -> ROS occupancy grid (park_map.pgm/.yaml) + a named
objects table (park_objects.yaml). Run once, or whenever park.world changes.

No simulator, no ground truth: reads the static world file only. Footprints use
per-family constant radii (all obstacle collisions in park.world are meshes, so
there is no SDF primitive radius to read). The costmap inflates these later --
we emit RAW footprints, no pre-inflation.
"""
import os
import sys
import math
import argparse

from map_tools.sdf_parse import parse_models
from map_tools.occupancy_grid import Grid
from map_tools.mesh_bounds import footprint
from map_tools.park_types import PARK_TYPES
from map_tools.dtm_raster import read_dtm_yaml

# Footprint radii in metres, BEFORE costmap inflation. See the plan for rationale.
# NOTE: the furniture entries here (bench/garden_table/lamp/trash_bin_1) are
# superseded by BOX_MESHES below -- build_grid boxes those families using the
# real mesh footprint instead. Left in place because test_radii still checks
# every family has a positive radius, and RADII has no other users to break.
# Sourced from the single type registry (map_tools.park_types), keyed by
# world_prefix.
RADII = {t.world_prefix: t.disc_radius for t in PARK_TYPES}

# Families stamped as yaw-oriented boxes: (collision .dae path relative to the
# models_opt/ root, mesh scale). Everything else (trees, lamp, trash_bin_1)
# stays a disc via RADII. lamp (0.095 m wide) and trash_bin_1 (~0.10x0.06 m)
# are EXCLUDED here on purpose: at 0.15 m grid resolution their box footprint
# is sub-cell, so stamp_box can find zero (or ~1) cell centers inside it and
# the object nearly or entirely vanishes from the map. Discs still mark them
# correctly.
#
# The scale is PER MESH, read from the <collision><geometry><mesh><scale> in
# park.world -- NOT the <state><scale>, which is always "1 1 1" (runtime
# state, not the model's authored scale) and NOT model.sdf's default scale
# (park.world's <scale> is the one actually applied at spawn time and can
# differ from model.sdf, as it does for garden_table: model.sdf says 1 1 1 but
# so does park.world here -- the two happen to agree for garden_table, but
# bench's park.world scale (0.15) also differs from its model.sdf default
# (0.1); park.world is the source of truth because it's the file actually
# loaded).
import os as _os
_MODELS_ROOT = _os.path.join(_os.path.dirname(__file__), "..", "models_opt")
# Sourced from the type registry: box_stamped types carry a mesh (rel-parts,
# scale); expand rel-parts to the absolute .dae path this module expects.
BOX_MESHES = {
    t.world_prefix: (_os.path.join(_MODELS_ROOT, *t.mesh[0]), t.mesh[1])
    for t in PARK_TYPES
    if t.box_stamped
}
# Cache footprints so each .dae is parsed once. Stores (half_dx, half_dy, cx,
# cy): the mesh-local center offset (cx, cy) accounts for COLLADA node
# transforms that translate geometry away from the mesh origin (e.g. the
# bench, whose footprint center is ~1.4 m from its origin) -- see
# mesh_bounds.footprint().
_footprint_cache = {}
def _box_extents(family):
    if family not in _footprint_cache:
        path, scale = BOX_MESHES[family]
        _footprint_cache[family] = footprint(path, scale)
    return _footprint_cache[family]

# Families that become named goal destinations. Sourced from the type registry:
# any type with is_object=True (keyed by world_prefix, so tree_8 not 'tree').
OBJECT_FAMILIES = tuple(t.world_prefix for t in PARK_TYPES if t.is_object)

# Pad around the extreme OBJECT positions when they extend the grid past the
# terrain. Model poses are points; their stamped footprints reach outward from
# them, so without a pad an edge object gets clipped. 5.0 m is the old margin
# default, comfortably larger than the biggest footprint half-extent (~1.5 m).
OBJECT_PAD = 5.0


def terrain_extent(dtm_yaml):
    """(min_x, min_y, max_x, max_y) of the DTM's FULL grid footprint, metres.

    FULL grid, deliberately -- NOT the bounding box of the finite (non-NaN)
    cells. A NaN cell means "this world position has no terrain under it"
    (open water in lake.world is ~30% of the DTM; off-mesh void elsewhere).
    Those are exactly the cells the map must represent as UNKNOWN so the
    planner refuses to route through them -- see clip_map_to_terrain.py. A cell
    can only be unknown if it is INSIDE the map; shrinking to the finite bbox
    would push the water back outside the map entirely, where the planner has
    no data at all rather than data saying do-not-go. That is the same class of
    bug as sizing from objects, so we keep the whole DTM footprint.
    """
    m = read_dtm_yaml(dtm_yaml)
    ox, oy = float(m["origin_x"]), float(m["origin_y"])
    res = float(m["resolution"])
    return ox, oy, ox + int(m["width"]) * res, oy + int(m["height"]) * res


def build_grid(models, resolution=0.15, margin=0.0, radii=None, dtm_yaml=None):
    """Stamp each model's footprint into a fresh occupancy grid.

    Extent is the TERRAIN footprint (`dtm_yaml`, padded by `margin`), UNIONed
    with the object extents so an object standing off the mesh is still
    contained rather than silently clipped. The terrain is what defines how far
    the map reaches -- objects merely sit on it. Passing dtm_yaml=None falls
    back to the old object-only sizing, with a warning, for callers that have
    no DTM.

    `margin` is metres of pad. It used to pad point-like object POSITIONS (so a
    footprint at the extreme edge was not clipped); around the terrain it has no
    such job, since the DTM footprint already is the drivable extent and padding
    it only adds cells that become unknown. Hence the default is now 0.0. The
    object half of the union still gets its own OBJECT_PAD so no footprint is
    clipped there.

    `radii` maps family -> disc radius; None means the park table, so existing
    callers are unchanged. extract_lake_map passes the lake registry's radii.
    """
    if radii is None:
        radii = RADII
    xs = [m.world_x for m in models]
    ys = [m.world_y for m in models]
    obj = (min(xs) - OBJECT_PAD, min(ys) - OBJECT_PAD,
           max(xs) + OBJECT_PAD, max(ys) + OBJECT_PAD)
    if dtm_yaml is None:
        sys.stderr.write(
            "WARNING: no DTM given -- sizing the grid from OBJECT POSITIONS "
            "only. The map's extent will not match the terrain.\n")
        bounds = obj
    else:
        tx0, ty0, tx1, ty1 = terrain_extent(dtm_yaml)
        terrain = (tx0 - margin, ty0 - margin, tx1 + margin, ty1 + margin)
        bounds = (min(terrain[0], obj[0]), min(terrain[1], obj[1]),
                  max(terrain[2], obj[2]), max(terrain[3], obj[3]))
        # An object outside the terrain footprint is a real anomaly (a prop
        # floating off the mesh), not something to quietly drop. Keep it in the
        # map AND name it.
        for m in models:
            if not (tx0 <= m.world_x <= tx1 and ty0 <= m.world_y <= ty1):
                sys.stderr.write(
                    "WARNING: object %r at (%.2f, %.2f) lies OUTSIDE the terrain "
                    "footprint x %.2f..%.2f y %.2f..%.2f -- grid extended to keep "
                    "it.\n" % (m.name, m.world_x, m.world_y, tx0, tx1, ty0, ty1))
    g = Grid(bounds[0], bounds[1], bounds[2], bounds[3], resolution)
    for m in models:
        if m.family in BOX_MESHES:
            hx, hy, cx, cy = _box_extents(m.family)
            # Rotate the mesh-local center offset by the model's yaw and
            # apply it on top of the link pose -- the box must be stamped at
            # the geometry's true (world-frame) center, not the link origin.
            sx = m.world_x + (cx * math.cos(m.yaw) - cy * math.sin(m.yaw))
            sy = m.world_y + (cx * math.sin(m.yaw) + cy * math.cos(m.yaw))
            g.stamp_box(sx, sy, m.yaw, hx, hy)
        else:
            g.stamp_disc(m.world_x, m.world_y, radii[m.family])
    return g


def build_objects(models, object_families=None):
    """Name -> map-frame pose for every model that is a named destination.

    `object_families` defaults to the park set, leaving existing callers alone.
    """
    if object_families is None:
        object_families = OBJECT_FAMILIES
    objects = {}
    for m in models:
        if m.family in object_families:
            objects[m.name] = {"x": round(m.world_x, 3), "y": round(m.world_y, 3),
                               "yaw": round(m.yaw, 4)}
    return objects


def _write_objects_yaml(objects, path, generator="extract_park_map.py"):
    with open(path, "w") as fh:
        fh.write("# Named goal destinations, map-frame metres. Generated by "
                 "%s.\n" % generator)
        for name in sorted(objects):  # sorted only for stable file diffs (not display)
            p = objects[name]
            fh.write("%s: {x: %.3f, y: %.3f, yaw: %.4f}\n"
                     % (name, p["x"], p["y"], p["yaw"]))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Extract park map + objects from park.world")
    ap.add_argument("--world", default=os.path.join(
        os.path.dirname(__file__), "..", "natural_environments_ros_opt",
        "natural_enviroment", "worlds", "park.world"))
    ap.add_argument("--out-dir", default=os.path.join(
        os.path.dirname(__file__), "..", "maps"))
    ap.add_argument("--resolution", type=float, default=0.15)
    ap.add_argument("--dtm", default=os.path.join(
        os.path.dirname(__file__), "..", "maps", "park_dtm.yaml"),
        help="DTM yaml whose footprint defines the grid extent. Pass an "
             "empty string to fall back to object-based sizing.")
    ap.add_argument("--margin", type=float, default=0.0,
                    help="metres of pad around the terrain footprint")
    args = ap.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    models = parse_models(args.world)
    grid = build_grid(models, resolution=args.resolution, margin=args.margin,
                      dtm_yaml=(args.dtm or None))
    objects = build_objects(models)

    grid.write_pgm(os.path.join(args.out_dir, "park_map.pgm"))
    grid.write_yaml(os.path.join(args.out_dir, "park_map.yaml"), "park_map.pgm")
    _write_objects_yaml(objects, os.path.join(args.out_dir, "park_objects.yaml"))
    print("wrote park_map.pgm/.yaml (%dx%d @ %.2f m) and park_objects.yaml (%d objects)"
          % (grid.width, grid.height, args.resolution, len(objects)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
