#!/usr/bin/env python3
"""Make low-poly versions of the lake world's meshes.

Quadric decimation, one output per mesh. Nothing else - it does not touch
lake.world, and it does not touch collision. Point the world's <visual> blocks at
the results yourself; leave <collision> on the originals.

Two settings are load-bearing. Change either and the output is visibly broken:

  1. SAVE IN THE SOURCE FORMAT.  DAE in -> DAE out, OBJ in -> OBJ out.
     Writing a DAE source out as .obj DESTROYS the UVs - pymeshlab's OBJ writer
     only emits `vt` lines when a texture is registered, so the mesh ends up with
     a single UV coordinate and renders as one flat blob of colour. OBJ output
     also needs save_wedge_texcoord=True.

  2. THE GROUND CANNOT TAKE A HARD CUT.  terreno_lago is a holed grid (742,619 of
     1,050,625 lattice sites) carrying 2.43 m of real relief. A 75% cut destroyed
     the surface outright. 50% keeps it - verified height span 2.435 m, identical
     to the original, shore dip intact.

Decimation is deliberate here rather than the grid subsample park used on its own
ground: park's terrain was flat (6.9 mm of relief) so uniform subsampling lost
nothing, while this one has shape worth spending triangles on. Quadric decimation
is content-aware - it collapses flat regions and keeps curvature.

Outputs land in models_lake_opt/<model>/. Meshes are built in a scratch dir first
because pymeshlab silently re-encodes texture PNGs sitting next to whatever it
writes.

Usage:
    python3 scripts/optimize_lake_meshes.py            # build every mesh
    python3 scripts/optimize_lake_meshes.py arbusto3   # build matching meshes only
"""
import os
import re
import shutil
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = "/media/thinh/Extreme Pro/Husky viz/models"
DST = os.path.join(REPO, "models_lake_opt")
SCRATCH = os.path.join(REPO, "artifacts", "lake_mesh_build")

# src_rel, model_dir, out_name, target_faces, relax_boundary
#
# relax_boundary=False keeps boundary edges pinned, which the ground needs.
# The leafy meshes floor out well above target with it on (their leaf cards are
# nearly all boundary edges), so they relax it to reach the target.
#
# tree_8_v/bark8.obj is absent on purpose: park already built a low-poly of that
# exact asset at models_opt/tree_8/bark8_lowpoly.obj (20,000 faces). Reuse it.
JOBS = [
    ("terreno_lago/lago.dae", "terreno_lago", "terreno_lago_lowpoly.dae", 740_000, False),
    ("arbusto3/model.dae", "arbusto3", "arbusto3_lowpoly.dae", 40_000, True),
    ("dry_bush/untitled.obj", "dry_bush", "dry_bush_lowpoly.obj", 30_000, True),
    ("tree_8_v/crown8.obj", "tree_8_v", "crown8_lowpoly.obj", 12_000, True),
    ("linea1/postes.dae", "linea1", "postes_lowpoly.dae", 30_000, True),
    ("linea1/cables2.dae", "linea1", "cables2_lowpoly.dae", 30_000, True),
]

# postes/cables carry no UVs in the source, so 0 is correct for them
NO_UV_EXPECTED = {"postes_lowpoly.dae", "cables2_lowpoly.dae"}


def uv_count(path):
    """UV entries in the output, so a UV-destroying save is caught immediately."""
    if path.endswith(".obj"):
        with open(path, errors="ignore") as fh:
            return sum(1 for line in fh if line.startswith("vt "))
    data = open(path, errors="ignore").read()
    return sum(int(x) for x in re.findall(r'-map-?\d*-array"\s+count="(\d+)"', data))


def build(rel, model_dir, out, target, relax):
    import pymeshlab as ml

    ms = ml.MeshSet()
    ms.load_new_mesh(os.path.join(SRC, rel))
    mesh = ms.current_mesh()
    before = mesh.face_number()
    textured = mesh.has_wedge_tex_coord()

    kw = dict(
        targetfacenum=target,
        preserveboundary=not relax,
        preservenormal=True,
        planarquadric=True,
        optimalplacement=True,
    )
    if not relax:
        kw["boundaryweight"] = 2.0

    if textured:
        ms.meshing_decimation_quadric_edge_collapse_with_texture(**kw)
    else:
        ms.meshing_decimation_quadric_edge_collapse(preservetopology=False, **kw)
    after = ms.current_mesh().face_number()

    os.makedirs(SCRATCH, exist_ok=True)
    scratch_path = os.path.join(SCRATCH, out)
    if out.endswith(".obj"):
        ms.save_current_mesh(scratch_path, save_wedge_texcoord=True)  # rule 1
    else:
        ms.save_current_mesh(scratch_path)

    uv = uv_count(scratch_path)
    broken = textured and out not in NO_UV_EXPECTED and uv < 100
    if broken:
        print(f"  {out:26s} UV LOSS ({uv}) - not installed. See rule 1.")
        return False

    target_dir = os.path.join(DST, model_dir)
    os.makedirs(target_dir, exist_ok=True)
    for name in (out, out + ".mtl"):
        src = os.path.join(SCRATCH, name)
        if os.path.exists(src):
            shutil.copy(src, target_dir)
    # pymeshlab drops a placeholder texture beside OBJ output; never ship it
    dummy = os.path.join(target_dir, "dummy.png")
    if os.path.exists(dummy):
        os.remove(dummy)

    print(
        f"  {out:26s} {before:>9,} -> {after:>8,} "
        f"({100 * (1 - after / before):4.1f}%)  uv={uv:,}"
    )
    return True


def main():
    if not os.path.isdir(SRC):
        sys.exit(f"source assets not mounted: {SRC}")
    only = sys.argv[1] if len(sys.argv) > 1 else None
    jobs = [j for j in JOBS if not only or only in j[2] or only in j[0]]
    if not jobs:
        sys.exit(f"no mesh matches {only!r}")

    print(f"Low-poly meshes -> {os.path.relpath(DST, REPO)}/")
    ok = all([build(*j) for j in jobs])
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
