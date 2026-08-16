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
import time
import argparse

import numpy as np

from map_tools.sdf_parse import parse_models
from map_tools.occupancy_grid import Grid
from map_tools.mesh_bounds import footprint
from map_tools.park_types import PARK_TYPES
from map_tools.scene_points import scene_cloud
from landmark_loc.descriptor import window, describe_region, region_distance

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


def build_grid(models, resolution=0.15, margin=5.0, radii=None):
    """Stamp each model's footprint into a fresh occupancy grid.

    `radii` maps family -> disc radius; None means the park table, so existing
    callers are unchanged. extract_lake_map passes the lake registry's radii.
    """
    if radii is None:
        radii = RADII
    xs = [m.world_x for m in models]
    ys = [m.world_y for m in models]
    g = Grid(min(xs) - margin, min(ys) - margin,
             max(xs) + margin, max(ys) + margin, resolution)
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


# --- Per-region distinctiveness over a location grid ------------------------
#
# The runtime localizer describes the point cloud in a spatial WINDOW around the
# robot and matches it against an offline map of DISTINCTIVE locations -- spots
# whose region descriptor has no near twin. This path builds that offline map:
# walk a grid of candidate locations over the park, describe each window with the
# SAME describe_region the runtime side uses, measure each location's distance to
# its nearest OTHER location, and keep only the distinctive ones.

GRID_STEP = 5.0        # metres between candidate locations
# Window radius. describe_region's default is 12 m, but at 12 m the nearest-other
# distances form a smooth continuum with no usable gap and the distinctive tail
# is smeared across many spots that each see several structures at once. 8 m
# sharpens the arrangement signal: measured, the top-distinctive locations then
# sit cleanly next to the six poles (all within ~12 m of a pole) and drop off
# where the map interior begins. Same R passed to window() and describe_region().
WINDOW_RADIUS = 8.0
MIN_WINDOW_PTS = 30    # a window with almost nothing in it is open ground, skip

# Bounds on how many locations the threshold may admit. NOT a target and NOT
# tuned to the poles -- it only rules out the two degenerate cuts a raw
# largest-gap search falls into on this smooth tail: a threshold so high it keeps
# a single top outlier, or so low it keeps nearly everything. Within these
# bounds the largest gap is the genuine break between the distinctive tail and
# the interior bulk (see _choose_threshold).
MIN_DISTINCTIVE = 4
MAX_DISTINCTIVE = 25


def _grid_descriptors(cloud, step=GRID_STEP, radius=WINDOW_RADIUS):
    """Describe a `step`-metre grid over the cloud's x/y extent.

    Returns a list of (x, y, descriptor) for every grid cell whose window holds
    at least MIN_WINDOW_PTS points (empty open ground carries no structure and
    would only pollute the distribution). Extent read from the cloud, never
    hardcoded.
    """
    xmin, ymin = float(cloud[:, 0].min()), float(cloud[:, 1].min())
    xmax, ymax = float(cloud[:, 0].max()), float(cloud[:, 1].max())

    out = []
    y = ymin
    while y <= ymax:
        x = xmin
        while x <= xmax:
            win = window(cloud, x, y, radius)
            if win.shape[0] >= MIN_WINDOW_PTS:
                desc = describe_region(win, radius=radius)
                out.append((round(x, 3), round(y, 3), desc))
            x += step
        y += step
    return out


def _nearest_other_distances(descriptors):
    """For each region, its region_distance to the nearest OTHER region.

    Computed here (not via distinctiveness.nearest_distances) because that helper
    is hardcoded to descriptor_distance; regions need region_distance.
    """
    n = len(descriptors)
    nearest = []
    for i in range(n):
        best = float("inf")
        for j in range(n):
            if i == j:
                continue
            d = region_distance(descriptors[i], descriptors[j])
            if d < best:
                best = d
        nearest.append(best)
    return nearest


def _choose_threshold(nearest, lo=MIN_DISTINCTIVE, hi=MAX_DISTINCTIVE):
    """Place the threshold in the largest gap of the sorted nearest-distance
    distribution, among the cuts that admit between `lo` and `hi` locations.
    Returns (threshold, best_gap, sorted_list).

    Distinctive = nearest-other distance ABOVE the threshold: no near twin.

    The unconstrained largest gap on this data is a single top outlier standing
    above a wide void, which would keep only one location. Constraining the cut
    to admit a handful of locations skips that degenerate gap and finds the real
    break between the distinctive tail (windows next to a pole, no near twin) and
    the interior bulk (windows that resemble many others). The bounds are not a
    target count and carry no pole information -- they only exclude the two
    degenerate ends. If the gap is poor the window radius is the knob, not these.
    """
    s = sorted(nearest)
    if len(s) < 2:
        return (float("inf"), 0.0, s)
    best_gap = -1.0
    thr = s[-1]
    for i in range(len(s) - 1):
        count_above = len(s) - (i + 1)
        if not (lo <= count_above <= hi):
            continue
        gap = s[i + 1] - s[i]
        if gap > best_gap:
            best_gap = gap
            thr = 0.5 * (s[i] + s[i + 1])
    return (thr, best_gap, s)


def build_regions(models, step=GRID_STEP, radius=WINDOW_RADIUS):
    """Build the distinctive-locations map. Returns (regions_dict, report_str).

    regions_dict: {loc_id: {x, y, descriptor: [...168...], nearest: float}} for
    distinctive locations only. report_str: the measured distribution + chosen
    threshold, printed by main so the choice is auditable.
    """
    t0 = time.time()
    cloud = scene_cloud(models)
    t_cloud = time.time() - t0
    print("scene_cloud: %d points in %.1f s" % (cloud.shape[0], t_cloud))

    cells = _grid_descriptors(cloud, step=step, radius=radius)
    descriptors = [d for (_, _, d) in cells]
    print("grid: %d candidate locations with >= %d window points"
          % (len(cells), MIN_WINDOW_PTS))

    nearest = _nearest_other_distances(descriptors)
    threshold, gap, sorted_nd = _choose_threshold(nearest)

    lines = ["nearest-other region_distance distribution (sorted):"]
    lines.append("  " + ", ".join("%.3f" % v for v in sorted_nd))
    lines.append("chosen threshold %.3f (largest gap %.3f in the tail, admitting "
                 "%d-%d locations)" % (threshold, gap, MIN_DISTINCTIVE,
                                       MAX_DISTINCTIVE))
    report = "\n".join(lines)

    regions = {}
    for i, (x, y, desc) in enumerate(cells):
        if nearest[i] >= threshold:
            loc_id = "loc_%03d" % i
            regions[loc_id] = {
                "x": x, "y": y,
                "descriptor": [round(float(v), 6) for v in desc],
                "nearest": round(float(nearest[i]), 6),
            }
    return regions, report


def _write_regions_yaml(regions, path, generator="extract_park_map.py"):
    """Write park_regions.yaml deterministically (sorted loc_id for stable diffs)."""
    with open(path, "w") as fh:
        fh.write("# Distinctive locations for per-region localization, map-frame "
                 "metres.\n# Generated by %s --regions.\n" % generator)
        for loc_id in sorted(regions):
            r = regions[loc_id]
            desc = ", ".join("%.6f" % v for v in r["descriptor"])
            fh.write("%s:\n" % loc_id)
            fh.write("  x: %.3f\n  y: %.3f\n  nearest: %.6f\n"
                     % (r["x"], r["y"], r["nearest"]))
            fh.write("  descriptor: [%s]\n" % desc)


def main(argv=None):
    ap = argparse.ArgumentParser(description="Extract park map + objects from park.world")
    ap.add_argument("--world", default=os.path.join(
        os.path.dirname(__file__), "..", "natural_environments_ros_opt",
        "natural_enviroment", "worlds", "park.world"))
    ap.add_argument("--out-dir", default=os.path.join(
        os.path.dirname(__file__), "..", "maps"))
    ap.add_argument("--resolution", type=float, default=0.15)
    ap.add_argument("--regions", action="store_true",
                    help="also build park_regions.yaml (distinctive locations)")
    args = ap.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    models = parse_models(args.world)
    grid = build_grid(models, resolution=args.resolution)
    objects = build_objects(models)

    grid.write_pgm(os.path.join(args.out_dir, "park_map.pgm"))
    grid.write_yaml(os.path.join(args.out_dir, "park_map.yaml"), "park_map.pgm")
    _write_objects_yaml(objects, os.path.join(args.out_dir, "park_objects.yaml"))
    print("wrote park_map.pgm/.yaml (%dx%d @ %.2f m) and park_objects.yaml (%d objects)"
          % (grid.width, grid.height, args.resolution, len(objects)))

    if args.regions:
        regions, report = build_regions(models)
        print(report)
        _write_regions_yaml(regions, os.path.join(args.out_dir, "park_regions.yaml"))
        print("wrote park_regions.yaml (%d distinctive locations)" % len(regions))
    return 0


if __name__ == "__main__":
    sys.exit(main())
