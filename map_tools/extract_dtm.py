"""Offline: park.world / lake.world -> a bare-earth Digital Terrain Model
(<world>_dtm.npy/.yaml in maps/), plus the lake's water surface layer.

DTM = TERRAIN ONLY. Trees, benches, lamps and every other object are excluded
on purpose: they are catalogued separately (park_objects.yaml / scene_points),
and mixing them in would turn a ground-height model into a canopy-height model.

No simulator and no ground truth: this reads the static .world file and the
mesh files it references. That is map data, not runtime state.

Three transforms have to be applied, in this order, or the terrain lands in the
wrong place. Each has already caused a real metre-scale bug in this repo:

  1. COLLADA <node> <matrix>  -- via mesh_bounds._triangles (see that module;
     ignoring node matrices once put a bench footprint ~1.2 m off).
  2. The world's mesh <scale>  -- e.g. park is 50 25 0.01, a deliberate ~100x
     vertical flattening; lake is 50 25 4.
  3. The model's EFFECTIVE <pose> -- via model_pose.resolve_pose, which
     prefers the <state> block over the <model> definition. Both worlds
     override their terrain this way (see model_pose.py for measured values).

Which mesh is the terrain differs per world, and NOT by the same rule:

  park.world  model 'parque'        visual AND collision are the same
                                    terreno_parque_lowpoly.dae, so lowpoly IS
                                    the simulated surface.
  lake.world  model 'terreno_lago'  visual is terreno_lago_lowpoly.dae but
                                    COLLISION is lago.dae -- a denser copy of
                                    the same surface. We rasterise the
                                    collision mesh: the wheels and the lidar
                                    rays interact with collision geometry, so
                                    that is the ground the robot actually
                                    experiences.

The lake's water is NOT a mesh. It is a Gazebo <box> (model 'lago', material
vrc/agua), so its surface is analytic -- the top face of the box at its
effective pose. No mesh parsing is needed or wanted for it.
"""
import argparse
import os
import sys

import numpy as np

from map_tools.dtm_raster import rasterize
from map_tools.mesh_bounds import _triangles
from map_tools.model_pose import resolve_pose_from_file

_REPO_ROOT = os.path.join(os.path.dirname(__file__), "..")
_WORLDS = os.path.join(_REPO_ROOT, "natural_environments_ros_opt",
                       "natural_enviroment", "worlds")


class TerrainSpec(object):
    """Everything needed to rasterise one world's terrain."""

    def __init__(self, world_file, model_name, mesh_path, scale, note):
        self.world_file = world_file
        self.model_name = model_name
        self.mesh_path = mesh_path
        self.scale = scale
        self.note = note


# scale is the world file's mesh <scale> (x y z), verified against the .world.
WORLDS = {
    "park": TerrainSpec(
        world_file=os.path.join(_WORLDS, "park.world"),
        # NOTE: the MODEL is named 'parque'. 'terreno_parque' is only the mesh
        # directory in the model:// URI -- there is no model by that name, and
        # looking one up returns nothing.
        model_name="parque",
        mesh_path=os.path.join(_REPO_ROOT, "models_opt", "terreno_parque",
                               "terreno_parque_lowpoly.dae"),
        scale=(50.0, 25.0, 0.01),
        note="lowpoly is both visual and collision in park.world"),
    "lake": TerrainSpec(
        world_file=os.path.join(_WORLDS, "lake.world"),
        model_name="terreno_lago",
        mesh_path=os.path.join(_REPO_ROOT, "models_lake_opt", "terreno_lago",
                               "lago.dae"),
        scale=(50.0, 25.0, 4.0),
        note="lago.dae is the COLLISION mesh (visual is the lowpoly copy)"),
}

# The lake's water surface: model 'lago', a visual-only <box> whose <size> is
# read here from the world definition. Only the lake has one.
WATER = {
    "lake": {"world_file": WORLDS["lake"].world_file, "model_name": "lago"},
}


def load_terrain_triangles(spec):
    """Triangles of the terrain mesh in WORLD coordinates.

    mesh_bounds._triangles applies the COLLADA node matrices and a single
    uniform scale. The world <scale> here is per-axis and non-uniform (z is
    0.01 vs 50 in x), so we pass scale=1.0 and apply the three axis scales
    ourselves, then the model's effective pose.
    """
    tris = _triangles(spec.mesh_path, scale=1.0)
    tris = tris * np.asarray(spec.scale, dtype=float)

    pose = resolve_pose_from_file(spec.world_file, spec.model_name)
    if pose is None:
        raise ValueError("no pose for model %r in %s"
                         % (spec.model_name, spec.world_file))
    # Terrain models in both worlds are axis-aligned (roll=pitch=yaw=0). A
    # rotated terrain would need a full rotation here; refuse rather than
    # silently ignore it.
    if any(abs(a) > 1e-9 for a in (pose.roll, pose.pitch, pose.yaw)):
        raise NotImplementedError(
            "model %r has a non-zero rotation (%.6g, %.6g, %.6g); the DTM "
            "extractor only handles axis-aligned terrain"
            % (spec.model_name, pose.roll, pose.pitch, pose.yaw))

    tris = tris + np.array([pose.x, pose.y, pose.z], dtype=float)
    return tris, pose


def _box_size(world_text, model_name):
    """The <size> of a model's visual <box>, as (sx, sy, sz)."""
    import re
    from map_tools.model_pose import _model_block
    block = _model_block(world_text, model_name)
    if block is None:
        raise ValueError("no model %r" % (model_name,))
    m = re.search(r"<box>\s*<size>([^<]+)</size>", block)
    if m is None:
        raise ValueError("model %r has no <box><size>" % (model_name,))
    vals = [float(v) for v in m.group(1).split()]
    if len(vals) != 3:
        raise ValueError("bad <size> %r" % (m.group(1),))
    return tuple(vals)


def water_layer(world_key, grid):
    """Water surface height on the SAME grid geometry as `grid`.

    The water is a box, so its top face is a horizontal plane at
    pose.z + size_z/2, covering the box's xy footprint. Cells outside that
    footprint are NaN -- there is no water there, and 0.0 would invent a sheet
    of water across the whole map.
    """
    cfg = WATER[world_key]
    with open(cfg["world_file"], "r") as fh:
        text = fh.read()
    size = _box_size(text, cfg["model_name"])
    pose = resolve_pose_from_file(cfg["world_file"], cfg["model_name"])
    if pose is None:
        raise ValueError("no pose for water model %r" % (cfg["model_name"],))

    top_z = pose.z + size[2] / 2.0
    half_x, half_y = size[0] / 2.0, size[1] / 2.0

    X, Y = grid.cell_centers()
    inside = ((np.abs(X - pose.x) <= half_x)
              & (np.abs(Y - pose.y) <= half_y))
    z = np.full(X.shape, np.nan, dtype=np.float32)
    z[inside] = top_z

    from map_tools.dtm_raster import DtmGrid
    return DtmGrid(z, grid.resolution, grid.origin_x, grid.origin_y), {
        "top_z": top_z, "size": size, "pose": pose,
    }


def _write_yaml(path, grid, meta):
    n_valid, total, z_min, z_max = grid.stats()
    with open(path, "w") as fh:
        fh.write("# Digital Terrain Model metadata. Generated by "
                 "map_tools/extract_dtm.py -- do not hand-edit.\n")
        fh.write("# Heights are metres in the world/map frame. The .npy is\n"
                 "# float32 [height][width], row 0 = LOWEST y. NaN = no mesh\n"
                 "# coverage (NOT zero height).\n")
        for key in ("layer", "world", "world_file", "source_mesh",
                    "model_name", "pose_source"):
            if key in meta:
                fh.write("%s: %s\n" % (key, meta[key]))
        fh.write("resolution: %.6f\n" % grid.resolution)
        fh.write("origin_x: %.6f\n" % grid.origin_x)
        fh.write("origin_y: %.6f\n" % grid.origin_y)
        fh.write("width: %d\n" % grid.width)
        fh.write("height: %d\n" % grid.height)
        fh.write("z_min: %.6f\n" % z_min)
        fh.write("z_max: %.6f\n" % z_max)
        fh.write("relief: %.6f\n" % (z_max - z_min))
        fh.write("valid_cells: %d\n" % n_valid)
        fh.write("total_cells: %d\n" % total)
        fh.write("valid_fraction: %.6f\n" % (float(n_valid) / total if total
                                             else 0.0))
        if "scale" in meta:
            fh.write("mesh_scale: [%.6g, %.6g, %.6g]\n" % tuple(meta["scale"]))
        if "pose" in meta:
            p = meta["pose"]
            fh.write("model_pose: [%.6g, %.6g, %.6g]\n" % (p.x, p.y, p.z))
        if "note" in meta:
            fh.write("note: %s\n" % meta["note"])


def _summarise(label, grid):
    n_valid, total, z_min, z_max = grid.stats()
    pct = 100.0 * n_valid / total if total else 0.0
    print("  %-6s grid %d x %d @ %.2f m  origin (%.3f, %.3f)"
          % (label, grid.width, grid.height, grid.resolution,
             grid.origin_x, grid.origin_y))
    print("         cells %d total, %d valid (%.1f%%)"
          % (total, n_valid, pct))
    print("         z_min %.4f  z_max %.4f  RELIEF %.4f m"
          % (z_min, z_max, z_max - z_min))
    return z_max - z_min


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Extract a bare-earth DTM from a Gazebo world")
    ap.add_argument("--world", required=True, choices=sorted(WORLDS),
                    help="which world's terrain to rasterise")
    ap.add_argument("--resolution", type=float, default=0.25,
                    help="grid cell size in metres (default 0.25)")
    ap.add_argument("--out", default=os.path.join(_REPO_ROOT, "maps"),
                    help="output directory (default maps/)")
    args = ap.parse_args(argv)

    if args.resolution <= 0:
        ap.error("--resolution must be positive")

    spec = WORLDS[args.world]
    os.makedirs(args.out, exist_ok=True)

    print("world   : %s" % os.path.normpath(spec.world_file))
    print("model   : %s" % spec.model_name)
    print("mesh    : %s" % os.path.normpath(spec.mesh_path))
    print("scale   : %s   (%s)" % (spec.scale, spec.note))

    tris, pose = load_terrain_triangles(spec)
    print("pose    : (%.6g, %.6g, %.6g) from the <%s> block"
          % (pose.x, pose.y, pose.z, pose.source))
    print("parsed  : %d triangles" % len(tris))

    grid = rasterize(tris, resolution=args.resolution)
    print("")
    relief = _summarise("dtm", grid)

    base = os.path.join(args.out, "%s_dtm" % args.world)
    np.save(base + ".npy", grid.z)
    _write_yaml(base + ".yaml", grid, {
        "layer": "terrain",
        "world": args.world,
        "world_file": os.path.normpath(spec.world_file),
        "source_mesh": os.path.normpath(spec.mesh_path),
        "model_name": spec.model_name,
        "pose_source": pose.source,
        "scale": spec.scale,
        "pose": pose,
        "note": spec.note,
    })
    print("         -> %s.npy / .yaml" % os.path.normpath(base))

    if args.world in WATER:
        wgrid, winfo = water_layer(args.world, grid)
        print("")
        print("water   : box size %.4f x %.4f x %.4f, pose z %.6g "
              "(<%s> block)"
              % (winfo["size"] + (winfo["pose"].z, winfo["pose"].source)))
        print("          top face z = %.4f m" % winfo["top_z"])
        _summarise("water", wgrid)
        wbase = os.path.join(args.out, "%s_water" % args.world)
        np.save(wbase + ".npy", wgrid.z)
        _write_yaml(wbase + ".yaml", wgrid, {
            "layer": "water",
            "world": args.world,
            "world_file": os.path.normpath(WATER[args.world]["world_file"]),
            "source_mesh": "(none -- analytic <box> top face)",
            "model_name": WATER[args.world]["model_name"],
            "pose_source": winfo["pose"].source,
            "pose": winfo["pose"],
            "note": "visual-only water box, material vrc/agua",
        })
        print("         -> %s.npy / .yaml" % os.path.normpath(wbase))

    print("")
    print("RELIEF (%s) = %.4f m" % (args.world, relief))
    return 0


if __name__ == "__main__":
    sys.exit(main())
