"""Offline: lake.world -> ROS occupancy grid (lake_map.pgm/.yaml) + a named
objects table (lake_objects.yaml). Run once, or whenever lake.world changes.

Same machinery as extract_park_map.py (grid stamping, object table, no
simulator, no ground truth -- the static world file only). What differs is the
type registry it classifies against, and one exclusion.

WHAT IS IN THE MAP
  tree         13x  disc r=0.45 (trunk). Obstacle + catalog landmark.
  postescable   1x  disc r=0.25 (one pole). Obstacle + catalog landmark.

WHAT IS A LANDMARK BUT NOT AN OBSTACLE
  lago          1x  The lake. Written to lake_objects.yaml so it has a known
                    map-frame position for planning and matching, but NEVER
                    stamped into the grid -- see WATER below.

WHAT IS DELIBERATELY ABSENT
  altaniv_seca_d (68), altaniv (31), bush (30), arbusto3 (34): low vegetation,
  163 instances. Not in LAKE_TYPES at all, so classify_prefix returns 'skip'
  and they never reach the grid. The robot decides whether to dodge them from
  live lidar and the local costmap. Stamping them would carve the static map
  into something unnavigable while hiding ground that is actually passable.

WATER
  `lago` is a visual-only <box> with NO <collision>, so the lidar returns
  nothing from it -- live obstacle avoidance is blind to the water. It is not
  something to bump into; it is a region to plan away from using prior map
  knowledge. So it is a landmark with a position, not occupied cells. Marking
  it lethal would also be wrong: its 75.1 x 37.8 m footprint is ordinary
  walkable ground for most of its extent (the box sits about 1 m BELOW mean
  terrain and surfaces only where the ground bottoms out).

  A planner that must actively avoid the water wants a costmap layer seeded
  from lake_objects.yaml, not lethal cells in this .pgm.

KNOWN LIMITATION -- the power line
  `postescable` is ONE model holding BOTH poles plus the 29 m span between
  them. Measured from linea1/postes.dae at world scale 0.03: 58.97 x 3.60 m
  overall, with near-base geometry clustering into exactly two poles, each
  0.50 m wide, ~29 m apart. A single disc at the model origin therefore marks
  only ONE pole; the second is unmarked.

  Left as-is on purpose. Splitting it needs either two registry entries with
  per-pole offsets or box-stamping the whole 59 m span (which would wall off
  the ground under the cables, 8 m up and not an obstacle at robot height).
  With one power line in the world this is a known gap, not a silent one.
"""
import argparse
import math
import os
import sys

from map_tools.sdf_parse import Model, parse_models
from map_tools.extract_park_map import build_grid, build_objects, _write_objects_yaml
from map_tools.park_types import LAKE_TYPES, LAKE_PREFIXES_LONGEST_FIRST

# Families that are landmarks only: they get an entry in lake_objects.yaml but
# are never stamped into the occupancy grid. See WATER above.
NOT_STAMPED = ("lago",)

# Mesh-local (x, y) of each pole inside linea1/postes.dae, at the model's world
# scale of 0.03. MEASURED from the mesh: take the vertices in the lowest 1 m and
# cluster them in x -- two clusters fall out, each with a 0.49 x 0.49 m
# footprint, 28.8 m apart.
#
# This offset is why a single disc at the model pose is wrong. `postescable`'s
# link_0 sits at (36.818, -5.326) with yaw 0.8566 rad, but the geometry inside
# the mesh is NOT centred on that link: rotating these offsets out puts the
# poles at world (+18.52, -24.52) and (+37.39, -2.76). Neither is near the link.
# Confirmed against the live lidar: returns above z=12 m (higher than any tree
# in this world) cluster at (+18.34, -23.34), ~1.2 m from predicted pole A.
#
# Same class of bug as the park landmark map, which had to read the trunk LINK
# rather than the model pose. Here even the link pose is not enough, because the
# geometry is offset inside the mesh.
POLE_OFFSETS = ((-26.489, 1.251), (2.311, 1.251))

# Lake counterparts of extract_park_map's RADII / OBJECT_FAMILIES, sourced the
# same way from the registry so the two cannot drift.
LAKE_RADII = {t.world_prefix: t.disc_radius for t in LAKE_TYPES}
LAKE_OBJECT_FAMILIES = tuple(t.world_prefix for t in LAKE_TYPES if t.is_object)


def _expand_poles(models):
    """Replace each `postescable` model with one Model per real pole.

    build_grid stamps a disc at model.world_x/world_y, which for this family is
    the link pose -- a point in mid-span with no geometry on it. Rotate each
    measured mesh-local pole offset by the model yaw and emit a Model there
    instead, so both poles are marked and the empty midpoint is not.
    """
    out = []
    for m in models:
        if m.family != "postescable":
            out.append(m)
            continue
        for i, (lx, ly) in enumerate(POLE_OFFSETS):
            wx = m.world_x + (lx * math.cos(m.yaw) - ly * math.sin(m.yaw))
            wy = m.world_y + (lx * math.sin(m.yaw) + ly * math.cos(m.yaw))
            out.append(Model(name="%s_pole%d" % (m.name, i), family=m.family,
                             world_x=wx, world_y=wy, yaw=m.yaw))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Extract lake map + objects from lake.world")
    ap.add_argument("--world", default=os.path.join(
        os.path.dirname(__file__), "..", "natural_environments_ros_opt",
        "natural_enviroment", "worlds", "lake.world"))
    ap.add_argument("--out-dir", default=os.path.join(
        os.path.dirname(__file__), "..", "maps"))
    ap.add_argument("--resolution", type=float, default=0.15)
    args = ap.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    models = parse_models(args.world, prefixes=LAKE_PREFIXES_LONGEST_FIRST)

    # objects: every classified model, water included (it is a landmark). The
    # power line is expanded here too -- otherwise `goal postescable` sends the
    # robot to the empty midpoint between the poles rather than to a pole.
    objects = build_objects(_expand_poles(models),
                            object_families=LAKE_OBJECT_FAMILIES)
    # grid: everything except the landmark-only families, with the power line
    # expanded into its two real poles (see POLE_OFFSETS).
    stamped = _expand_poles([m for m in models if m.family not in NOT_STAMPED])
    grid = build_grid(stamped, resolution=args.resolution, radii=LAKE_RADII)

    grid.write_pgm(os.path.join(args.out_dir, "lake_map.pgm"))
    grid.write_yaml(os.path.join(args.out_dir, "lake_map.yaml"), "lake_map.pgm")
    _write_objects_yaml(objects, os.path.join(args.out_dir, "lake_objects.yaml"),
                        generator="extract_lake_map.py")

    by_family = {}
    for m in models:
        by_family[m.family] = by_family.get(m.family, 0) + 1
    print("wrote lake_map.pgm/.yaml (%dx%d @ %.2f m) and lake_objects.yaml "
          "(%d objects)" % (grid.width, grid.height, args.resolution, len(objects)))
    print("  classified: %s" % ", ".join(
        "%s x%d%s" % (f, n, " (landmark only)" if f in NOT_STAMPED else "")
        for f, n in sorted(by_family.items())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
