#!/usr/bin/env python3
"""Build the low-poly lake world: reduce meshes for rendering, keep collision full.

Mirrors what park.world does:
  * VISUAL     -> hard-cut low-poly mesh
  * COLLISION  -> the ORIGINAL mesh, untouched
  * sole exception: tree bark, which gets its own separate collision mesh
    (park already built one at models_opt/tree_8/bark8_collision.obj, reused here)

Two rules that are load-bearing; breaking either produces a visibly broken world:

  1. SAVE IN THE SOURCE FORMAT.  DAE in -> DAE out, OBJ in -> OBJ out.
     Writing a DAE source out as .obj DESTROYS the UVs: pymeshlab's OBJ writer
     only emits `vt` lines when a texture is registered, so the mesh ends up with
     a single UV coordinate and renders as one flat blob of colour.  For OBJ
     output also pass save_wedge_texcoord=True.

  2. THE GROUND IS A HOLED GRID WITH REAL RELIEF.  terreno_lago has 742,619 of
     1,050,625 lattice sites occupied and 2.43 m of height variation.  Cutting it
     75% destroyed the surface outright.  50% keeps it intact.  Do not raise
     GROUND_TARGET without re-checking the height distribution afterwards
     (--verify does this).

Usage:
    python3 scripts/optimize_lake_meshes.py            # decimate + install + rewrite world
    python3 scripts/optimize_lake_meshes.py --verify   # re-check an existing build
    python3 scripts/optimize_lake_meshes.py --revert   # restore the pristine world

Requires the source assets on the external drive (see SRC below) and pymeshlab.
"""
import argparse
import os
import re
import shutil
import sys

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = "/media/thinh/Extreme Pro/Husky viz/models"
DST = os.path.join(REPO, "models_lake_opt")
WORLD = os.path.join(
    REPO, "natural_environments_ros_opt/natural_enviroment/worlds/lake.world"
)
BACKUP = WORLD + ".bak-before-opt"
# scratch: pymeshlab silently re-encodes texture PNGs sitting next to whatever it
# writes, so meshes are built here and only the mesh file is moved into place.
SCRATCH = os.path.join(REPO, "artifacts", "lake_mesh_build")

GROUND_TARGET = 740_000  # 50% of 1,481,096 - see rule 2 above

# src_rel, out_name, target_faces, relax_boundary
JOBS = [
    ("terreno_lago/lago.dae", "terreno_lago_lowpoly.dae", GROUND_TARGET, False),
    ("arbusto3/model.dae", "arbusto3_lowpoly.dae", 40_000, True),
    ("dry_bush/untitled.obj", "dry_bush_lowpoly.obj", 30_000, True),
    ("tree_8_v/crown8.obj", "crown8_lowpoly.obj", 12_000, True),
    ("linea1/postes.dae", "postes_lowpoly.dae", 30_000, True),
    ("linea1/cables2.dae", "cables2_lowpoly.dae", 30_000, True),
]

# model dir each output belongs in
INSTALL = {
    "terreno_lago_lowpoly.dae": "terreno_lago",
    "arbusto3_lowpoly.dae": "arbusto3",
    "dry_bush_lowpoly.obj": "dry_bush",
    "crown8_lowpoly.obj": "tree_8_v",
    "postes_lowpoly.dae": "linea1",
    "cables2_lowpoly.dae": "linea1",
}

# visual swaps: original uri -> low-poly uri
VISUAL = {
    "terreno_lago/lago.dae": "terreno_lago/terreno_lago_lowpoly.dae",
    "arbusto3/model.dae": "arbusto3/arbusto3_lowpoly.dae",
    "dry_bush/untitled.obj": "dry_bush/dry_bush_lowpoly.obj",
    "tree_8_v/crown8.obj": "tree_8_v/crown8_lowpoly.obj",
    "linea1/postes.dae": "linea1/postes_lowpoly.dae",
    "linea1/cables2.dae": "linea1/cables2_lowpoly.dae",
    # park already optimized this exact asset; reuse rather than rebuild
    "tree_8_v/bark8.obj": "tree_8/bark8_lowpoly.obj",
}
# collision swaps: only bark moves off its original, exactly as park did
COLLISION = {
    "tree_8_v/bark8.obj": "tree_8/bark8_collision.obj",
}


def uv_count(path):
    """Number of UV entries, so a UV-destroying save is caught immediately."""
    if path.endswith(".obj"):
        with open(path, errors="ignore") as fh:
            return sum(1 for line in fh if line.startswith("vt "))
    data = open(path, errors="ignore").read()
    return sum(int(x) for x in re.findall(r'-map-?\d*-array"\s+count="(\d+)"', data))


def decimate():
    import pymeshlab as ml

    os.makedirs(SCRATCH, exist_ok=True)
    print("Decimating (visual meshes only; collision keeps the original):")
    for rel, out, target, relax in JOBS:
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

        path = os.path.join(SCRATCH, out)
        if path.endswith(".obj"):
            ms.save_current_mesh(path, save_wedge_texcoord=True)  # rule 1
        else:
            ms.save_current_mesh(path)

        uv = uv_count(path)
        cut = 100 * (1 - after / before)
        warn = "  <-- UV LOSS, see rule 1" if textured and uv < 100 else ""
        print(f"  {out:26s} {before:>9,} -> {after:>8,} ({cut:4.1f}%)  uv={uv:,}{warn}")


def install():
    print("Installing into models_lake_opt/:")
    for out, model_dir in INSTALL.items():
        target_dir = os.path.join(DST, model_dir)
        os.makedirs(target_dir, exist_ok=True)
        for name in (out, out + ".mtl"):
            src = os.path.join(SCRATCH, name)
            if os.path.exists(src):
                shutil.copy(src, target_dir)
        print(f"  {model_dir}/{out}")
    # pymeshlab drops a placeholder texture next to OBJ output; never ship it
    for root, _dirs, files in os.walk(DST):
        for f in files:
            if f == "dummy.png":
                os.remove(os.path.join(root, f))


def rewrite_world():
    """Point <visual> at the low-poly mesh, leave <collision> on the original."""
    if not os.path.exists(BACKUP):
        shutil.copy(WORLD, BACKUP)
        print(f"  backed up -> {os.path.basename(BACKUP)}")
    # always rewrite from the pristine copy so this is idempotent
    data = open(BACKUP, encoding="utf-8", errors="ignore").read()

    chunks, pos, counts = [], 0, {}
    for m in re.finditer(r"<(visual|collision)(\s[^>]*)?>.*?</\1>", data, re.S):
        chunks.append(data[pos : m.start()])
        blk, kind = m.group(0), m.group(1)
        table = VISUAL if kind == "visual" else COLLISION
        for orig, rep in table.items():
            token = f"model://{orig}"
            if token in blk:
                blk = blk.replace(token, f"model://{rep}")
                counts[(kind, orig)] = counts.get((kind, orig), 0) + 1
        chunks.append(blk)
        pos = m.end()
    chunks.append(data[pos:])
    new = "".join(chunks)

    open(WORLD, "w", encoding="utf-8").write(new)
    print("Rewriting lake.world:")
    for (kind, orig), n in sorted(counts.items()):
        print(f"  {kind:9s} {orig:26s} x{n}")

    import xml.etree.ElementTree as ET

    ET.parse(WORLD)
    print(f"  {len(data):,} -> {len(new):,} bytes, XML OK")


def verify():
    """Re-check the two things that were silently broken on earlier attempts."""
    ok = True

    print("UV survival:")
    for out, model_dir in INSTALL.items():
        path = os.path.join(DST, model_dir, out)
        if not os.path.exists(path):
            print(f"  {out:26s} MISSING")
            ok = False
            continue
        uv = uv_count(path)
        # postes/cables have no UVs in the source, so 0 is correct for them
        expect_uv = out not in ("postes_lowpoly.dae", "cables2_lowpoly.dae")
        bad = expect_uv and uv < 100
        ok &= not bad
        print(f"  {out:26s} uv={uv:>9,}{'   <-- BROKEN' if bad else ''}")

    print("Ground height (must match the original):")
    for label, path in (
        ("original", os.path.join(SRC, "terreno_lago/lago.dae")),
        ("low-poly", os.path.join(DST, "terreno_lago/terreno_lago_lowpoly.dae")),
    ):
        if not os.path.exists(path):
            print(f"  {label}: MISSING")
            ok = False
            continue
        data = open(path, errors="ignore").read()
        m = re.search(
            r'<float_array[^>]*positions-array"[^>]*count="(\d+)"[^>]*>(.*?)</float_array>',
            data,
            re.S,
        )
        vals = [float(x) for x in m.group(2).split()]
        zs = vals[2::3]
        # ground is scaled 50 25 4 in the world; x4 gives world metres
        print(
            f"  {label}: verts={len(zs):>8,}  z span {(max(zs) - min(zs)) * 4:.3f} m"
            f"  (min {min(zs) * 4:+.3f}, max {max(zs) * 4:+.3f})"
        )

    print("Collision must still reference the ORIGINAL meshes:")
    data = open(WORLD, errors="ignore").read()
    col = set()
    for m in re.finditer(r"<collision[^>]*>(.*?)</collision>", data, re.S):
        col |= {
            u
            for u in re.findall(r"<uri>model://([^<]+)</uri>", m.group(1))
            if u.endswith((".dae", ".obj"))
        }
    for uri in sorted(col):
        lowpoly = "_lowpoly" in uri
        ok &= not lowpoly
        print(f"  {uri:44s}{'  <-- LOW-POLY IN COLLISION' if lowpoly else ''}")

    print("\nVERIFY:", "PASS" if ok else "FAIL")
    return ok


def revert():
    if not os.path.exists(BACKUP):
        sys.exit(f"no backup at {BACKUP}")
    shutil.copy(BACKUP, WORLD)
    print(f"restored {WORLD} from {os.path.basename(BACKUP)}")


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--verify", action="store_true", help="check an existing build")
    ap.add_argument("--revert", action="store_true", help="restore the pristine world")
    args = ap.parse_args()

    if args.revert:
        revert()
        return
    if args.verify:
        sys.exit(0 if verify() else 1)

    if not os.path.isdir(SRC):
        sys.exit(f"source assets not mounted: {SRC}")
    decimate()
    install()
    rewrite_world()
    print()
    verify()


if __name__ == "__main__":
    main()
