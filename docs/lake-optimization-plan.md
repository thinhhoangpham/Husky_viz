# Lake world — optimization plan

**DONE 2026-08-15 — see "What was actually done" at the top. The plan text below
is kept for its measurements, but several of its recommendations were NOT
followed and some were wrong.**

---

## What was actually done

Followed **park's structure exactly**: visual gets the hard cut, collision keeps
the ORIGINAL mesh, with tree bark the sole exception (its own separate collision
mesh, reusing park's existing file).

| Model | Visual | Collision |
|---|---|---|
| `terreno_lago` (ground) | `terreno_lago_lowpoly.dae` 740,000 (50%) | **original** 1,481,096 |
| `arbusto3` | `arbusto3_lowpoly.dae` 40,000 (84%) | **original** 249,731 |
| `dry_bush` | `dry_bush_lowpoly.obj` 30,000 (82%) | **original** 163,516 |
| `linea1/postes` | `postes_lowpoly.dae` 29,988 (92%) | **original** 351,336 |
| `linea1/cables2` | `cables2_lowpoly.dae` 29,991 (89%) | **original** 279,476 |
| `tree_8_v/crown8` | `crown8_lowpoly.obj` 17,792 (50%) | **original** 35,584 |
| `tree_8_v/bark8` | park's `tree_8/bark8_lowpoly.obj` 20,000 | park's `tree_8/bark8_collision.obj` 39,974 |
| `altaniv`, `altaniv_d` | untouched | untouched |

Outputs in `models_lake_opt/` (gitignored, like `models_opt/`). World edited at
`natural_environments_ros_opt/natural_enviroment/worlds/lake.world`; original
preserved as `lake.world.bak-before-opt`.

### Verified after the work

| Check | Result |
|---|---|
| RTF (robot + lidar, 60 s) | **0.9974** |
| Lidar | **10.00 Hz, 15,629 pts/scan** (was 16,026 → −2.5%, all from bark) |
| gzserver memory | 3.93 GB, **0 MB drift** |
| GPU | 53% (was **100%** unoptimized) |
| Ground height span | **2.435 m — identical to original** |
| Ground shore dip | present, ~3 cm (original ~3 cm) |
| Load errors | none |

### Mistakes made, so they are not repeated

1. **Save meshes in their SOURCE format.** Park did DAE→DAE, OBJ→OBJ. Writing a
   DAE source out as OBJ **destroys the UVs** — pymeshlab's OBJ writer only emits
   `vt` lines when a texture is registered, so the ground came out with a single
   UV coordinate and rendered as one flat blob of colour. For OBJ output also
   pass `save_wedge_texcoord=True`.
2. **Do not decimate the ground aggressively.** It is a holed grid (742,619 of
   1,050,625 lattice sites) with 2.43 m of relief. A 75% cut destroyed the
   surface — most of the terrain vanished. 50% keeps it intact.
3. **Do not delete the tree crowns.** Park's justification (canopy at 3.99 m,
   above the 2 m obstacle band) is about obstacle avoidance, not looks. Deleting
   them turned lake's trees into bare sticks. Reduce, do not remove.
4. **Do not point collision at the hard-cut visual mesh.** That is the opposite
   of what park did on every single model.

---

Original plan text follows. Written 2026-08-15 before any of the above.

Companion to `docs/map-optimization.md` (the park procedure) and
`park_optimization.md` (the park record). The two-track principle, the pymeshlab
traps, and the world-file editing rules all carry over unchanged and are not
repeated in full here — read the park procedure first.

Target world: `natural_environments_ros_opt/natural_enviroment/worlds/park.world`'s
sibling, `.../worlds/lake.world`. The `_tagged` variant is **out of scope** by
decision.

---

## Status of the numbers below

| Claim | Basis |
|---|---|
| Triangle counts, per-mesh | **Measured** — parsed from the source assets on the external drive |
| Instance counts (179 models) | **Measured** — `<state>` block excluded; see the counting trap |
| Visual/collision split | **Measured** — every mesh appears in both, so the tracks start identical |
| Material splits (leaf/bark/trunk) | **Measured** — per-`usemtl` / per-`<triangles>` face counts |
| Terrain lattice, relief, footprint | **Measured** — parsed the position array |
| Asset identity `tree_8_v` == park `tree_8` | **Measured** — 165,380 tris, matches park's documented figure |
| World inlines models, no `<include>` | **Verified** — 0 include tags, 179 body defs + 179 state entries |
| `laser_retro` per-species mapping | **Verified** — 192 tags, enumerated per model |
| RTF, lidar rate, memory, unoptimized | **Measured** 2026-08-15 — see Step 0. RTF 0.9982, lidar 9.98 Hz, no drift |
| ~~"Lake will not run acceptably as-is"~~ | **REFUTED by measurement.** Physics and lidar run fine unoptimized |
| Render FPS (~30 vs park's ~60) | **Reported by the user, not yet instrumented.** GPU measured at 100% |
| Post-optimization targets | **Proportional estimates**, not calibrated against a measured frame budget |

**Step 0 has now been run** (2026-08-15) — see its section for measured results.
The headline: physics and lidar are fine unoptimized; **render frame rate is the
real problem**, and the targets below were written before that was known.

---

## Assets and where they are

The lake models are **not in this repo**. They live on the external drive:

    /media/thinh/Extreme Pro/Husky viz/models/                  (97 model dirs)
    /media/thinh/Extreme Pro/Husky viz/natural_environments_ros/

`models_opt/` in the repo holds only the 8 park models. Nothing lake-related is
on the internal disk.

### Naming trap — read this before touching anything

Two different models have confusingly similar names:

| Model name in `lake.world` | What it is | Geometry |
|---|---|---|
| `terreno_lago` | **the ground** | mesh `terreno_lago/lago.dae`, 1,481,096 tris, scale `50 25 4` |
| `lago` | **the water** | a `<box>` 75.0997 x 37.8189 x 8.6983, visual only, **no collision** |

The ground model's *mesh file* is named `lago.dae`. The water model is *named*
`lago` and has no mesh at all. Do not conflate them.

Also note the model **instance** names in the world differ from the **folder**
names they resolve to:

| Instance name (in world) | Folder (on drive) |
|---|---|
| `altaniv_seca_d` | `altaniv_d` |
| `bush` | `dry_bush` |
| `tree` | `tree_8_v` |
| `postescable` | `linea1` |

---

## Measured census — the unoptimized world

**179 models total** (not 358 — see the counting trap below).

Every mesh is referenced by **both** a `<visual>` and a `<collision>` in its
model, so the two tracks start out identical:

| Mesh | Tris each | Inst. | Visual | Collision | Share |
|---|---|---|---|---|---|
| `arbusto3/model.dae` | 249,731 | 34 | **8,490,854** | 8,490,854 | 46.4% |
| `dry_bush/untitled.obj` | 163,516 | 30 | **4,905,480** | 4,905,480 | 26.8% |
| `tree_8_v/bark8.obj` | 165,380 | 13 | 2,149,940 | 2,149,940 | 11.8% |
| `terreno_lago/lago.dae` | 1,481,096 | 1 | 1,481,096 | 1,481,096 | 8.1% |
| `tree_8_v/crown8.obj` | 35,584 | 13 | 462,592 | 462,592 | 2.5% |
| `linea1/postes.dae` | 351,336 | 1 | 351,336 | 351,336 | 1.9% |
| `linea1/cables2.dae` | 279,476 | 1 | 279,476 | 279,476 | 1.5% |
| `altaniv/nivdisppeqdae.dae` | 4,153 | 31 | 128,743 | 128,743 | 0.7% |
| `altaniv_d/alta.dae` | 563 | 68 | 38,284 | 38,284 | 0.2% |
| | | **179** | **18,287,801** | **18,287,801** | |

**Lake is ~2.4x heavier than park was before optimization** (18.3 M vs 7.56 M).

### Counting trap — corrected 2026-08-15

An earlier draft of this plan claimed 358 models and ~35.1 M triangles. **Both
were exactly 2x too high.** `<model name=` appears **twice** for every model —
once inside the `<state>` block (pose/velocity only, no geometry) and once as the
body definition — so a naive `grep -c "<model name='"` double-counts everything.

`lake.world` has a `<state>` block spanning bytes 5867–83113 containing 179
model entries, plus 179 body definitions outside it. **Exclude the `<state>`
block before counting instances.**

The terrain is also **one** model, not two — it is referenced twice within its
own model (visual + collision), which is 1.48 M per track, not 2.96 M total.

The conclusions are unchanged in shape — the vegetation still dominates at 73%,
and the priority order is identical — but every absolute figure is halved, and
the world is less overweight than first stated.

**The cost is concentrated in the vegetation: `arbusto3` + `dry_bush` = 76.4%.**
That is where the work is. Park's expensive items (ground, path) are cheap here.

---

## Measured topology — what each asset needs

### `arbusto3` (68 inst, 16.98 M) — bark-dominant, park's tree profile

Three distinct bush meshes in one file, 9 `<triangles>` blocks each, three
materials:

| Material | Tris | Share |
|---|---|---|
| Bark | 221,931 | 88.9% |
| Leaf | 27,415 | 11.0% |
| Flowers | 385 | 0.2% |

Sub-meshes: `European_Cranberry_Bush_03` (119,426), `Style_3` (76,103),
`Style_1` (54,202). Height span 0 – 6.7 m.

**pymeshlab flattens sub-meshes on import and no setting prevents it.** Park got
away with this on the bench *only* because all its sub-meshes shared one
material. Here there are three. **Each geometry must be decimated separately**
and reassembled, or the bark/leaf/flower bindings are destroyed.

### `dry_bush` (60 inst, 9.81 M) — leaf-dominant, inside the obstacle band

163,516 tris, all triangles (no quads), Blender 2.82 export, 5 objects:

| Material | Tris | Share |
|---|---|---|
| `Physocarpus_opu_diab_l1..l5` (**leaves**) | 148,932 | 91.1% |
| `Physocarpus_opu_diab_tr` (trunk) | 14,568 | 8.9% |
| 4 stray `Material_#0*` groups | 16 | ~0% |

**Measured height: leaves span 0.02 – 1.80 m above base; trunk 0.00 – 1.80 m.**

### `terreno_lago` (1 inst, 1.48 M) — a holed grid with real relief

- Single geometry, named `Grid-mesh`. No polylist.
- Footprint **99.50 x 49.75 m** (local 1.99 x 1.99 at scale `50 25`).
- Dominant grid spacing 0.001953 = **1/512**, implying a **1025 x 1025 lattice** —
  the same lattice park's ground had.
- But only **742,619 of 1,050,625 sites are occupied (~71%)**. Every vertex has a
  unique (x,y); no duplicates. **It is a grid with holes.**
- **Relief 2.43 m** in world units (local z span 0.6086 x the 4x vertical scale).
  Distribution is a genuine bowl: p0 −1.51 m, p50 −0.52 m, p100 +0.93 m.

### `tree_8_v` (26 inst, 5.23 M) — literally park's tree

`bark8.obj` is 165,380 tris, matching park's documented figure exactly. Same
asset, different folder name.

### Everything else (~1.6 M, 4.5%)

`linea1` (postes + cables, 1.26 M over 2 instances), `altaniv` and `altaniv_d`
(0.33 M over 198 instances — individually tiny at 4,153 and 563 tris).

---

## The plan

### Step 0 — baseline: DONE 2026-08-15

Ran unoptimized, world + Husky + Ouster, via `create_lake.launch` then
`add_husky_lake_1.launch`.

| Metric | Measured |
|---|---|
| **RTF** | **0.9982** over 60 s (also 0.9985, 0.9989) |
| **`/os0_cloud_node/points`** | **9.98 Hz**, 16,026 points/scan |
| gzserver RSS | 3.83 GB, **0 MB drift** over 60 s |
| gzclient RSS | 2.25 GB, 828 MiB GPU |
| Controllers | both `( running )` |
| World load errors | zero |

**Physics and lidar are fine unoptimized.** Park's post-optimization RTF was
0.999; lake matches it before any work. The premise that lake "will not run
acceptably as-is" — inferred from park's history — **was wrong**. Park's failure
was never raw triangle count; it was specific pathological geometry, which lake
does not have.

**The open problem is render frame rate, which RTF does not measure.** RTF is
gzserver's physics keeping pace with wall clock; FPS is gzclient's render loop.
Reported observation: lake renders at ~30 fps against optimized park's ~60. The
GPU (Quadro P4000) sat at **100% utilization** during the run.

Because the Ouster is `gpu_ray`, the lidar's ray pass and the render share that
GPU, so the two costs are **not independent** — a render-side measurement must
be taken with the lidar both enabled and disabled to attribute the cost.

**Consequence for this plan: the visual track is the live problem; the collision
track has no measured problem to solve.** Steps below should be re-read with that
in mind — collision targets were written defensively against a laser bottleneck
that did not materialise.

**Still unmeasured:** actual FPS (no instrumented number yet, only the reported
~30), and the split between render cost and lidar cost.

#### Launch notes (learned the hard way)

- **The Ouster plugin is at `~/husky_overlay_ws/devel/lib/libgazebo_ros_ouster_gpu_laser.so`.**
  `load-park-world.sh` searches `~/catkin_ws`, `$SCRIPT_DIR/catkin_ws` and
  `$SCRIPT_DIR/../catkin_ws` — **none of which exist**. Its fallback would spawn a
  laser-less Husky.
- **There is a duplicate package tree.** `~/husky_overlay_ws/src/natural_environments`
  shadows the repo's `natural_environments_ros_opt/natural_enviroment`. Sourcing
  the overlay *and* prepending the repo to `ROS_PACKAGE_PATH` makes roslaunch die
  with "multiple files named [create_lake.launch]". Use a full path to the launch
  file. The two `lake.world` copies are **md5-identical** (`c2c9403b...`).
- **The model path has spaces** (`/media/thinh/Extreme Pro/Husky viz/models`).
  Symlink it somewhere space-free before setting `GAZEBO_MODEL_PATH`.
- Two non-fatal errors appear and can be ignored: a joystick error
  (`/dev/input/ps4` absent, even with `joystick:=false`) and
  `Spawn service failed. Exiting.` — the robot spawns correctly regardless.
  Verify with `/gazebo/model_states`.
- **Check the lidar by counting messages, not by `rostopic info`.** A publisher
  can exist while nothing is emitted; `rostopic hz` also returned nothing here
  while the topic was in fact healthy at 9.98 Hz.

### Step 1 — the free win: reuse park's tree outputs

`tree_8_v` is park's `tree_8`. Park already produced both tracks:

| Track | File | Tris |
|---|---|---|
| Visual | `models_opt/tree_8/bark8_lowpoly.obj` | 20,000 |
| Collision | `models_opt/tree_8/bark8_collision.obj` | 39,974 |

Point lake's 26 trees at these. **Delete `crown8.obj`** — same asset, same
justification park measured (lowest leaf 3.99 m, obstacle map discards above
2 m), both tracks.

Saves ~4.6 M visual and ~3.3 M collision for zero new decimation work. Do this
first: largest ratio of benefit to risk in the whole plan.

**Do not reuse the 20,000-tri visual mesh as collision.** Park measured it at
8.63 mm mean / 236 mm max ray-hit shift, biased outward, bounding box shrunk
13.98 -> 13.52 m. That is why a separate 39,974-tri collision mesh exists.

### Step 2 — `arbusto3`: the main event (48% of the world)

Bark is 88.9%, so this follows park's tree profile.

- **Visual:** decimate bark hard. Park cut tree bark 165,380 -> 20,000 (8.3x) with
  3.36 mm deviation against a 7.1 mm pixel. A similar ratio here suggests
  ~250 K -> ~30 K per mesh.
- **Collision:** decimate conservatively and *separately*, park-style. Park chose
  40,000 from 165,380 (4.1x).
- **Keep the leaves on both tracks.** Only 11% of the mesh — deletion saves little
  and degrades the object.

Handle the three sub-meshes individually (see the flattening trap above).

Estimated: 16.98 M -> ~2.0 M visual.

### Step 3 — `dry_bush`: decimate, do NOT delete the leaves (28% of the world)

**This is where park's precedent inverts, and the measurement says so.**

Park deleted tree leaves because the lowest hung **3.99 m** up and the obstacle
map discards above **2 m** — navigationally irrelevant. `dry_bush` foliage sits
at **0.02 – 1.80 m**: entirely inside the obstacle band, from ground level up.

Deleting it removes 91.1% of the object and leaves a 14,568-tri skeleton at the
same height as the bush that was there. The robot would see a thin stick where
there is a solid obstacle, at exactly the height its lidar sweeps.

Park's "deletion is the only option" reasoning also does not apply. That held
because the crown was 17,792 *separate 2-triangle cards* — a flat quad cannot go
below 2 triangles, so decimation had nothing to remove. `dry_bush` has 148,932
tris across 5 leaf groups: real geometry with real redundancy. Decimation is
available.

- **Visual:** decimate leaves hard — 91% of the mesh, the single biggest
  per-asset win available.
- **Collision:** keep the leaves; decimate lightly or not at all. Thin peripheral
  structure is what over-decimation destroys, and a bush is nothing but that.
  Park's named failure mode — *detail with no counterpart*, geometry deleted so
  rays stop returning anything — grows much faster than ray-hit shift, and
  dominates here.

Estimated: 9.81 M -> ~1.5 M visual.

### Step 4 — `terreno_lago`: light touch, or skip

**Park's ground recipe does not transfer.** Park subsampled 16x because its
ground held **6.9 mm** of relief across the same footprint. This holds **2.43 m**
— about 350x more, a real bowl. A 16x subsample would visibly flatten it.

This terrain behaves like park's **path**, not park's ground: a holed lattice
where every-4th-point subsampling erodes the outline, because a coarse cell needs
all four corners present. On the path that cost ~39 cm x / ~20 cm y. Here the
holes are the shoreline, so erosion eats the water's edge.

- **Visual:** subsample at most 4x (every 2nd point, ~370 K), not 16x. Verify
  deviation against the 7.1 mm pixel **with the 4x vertical scale applied** — the
  same conversion trap park hit with the bench's 0.15 scale.
- **Collision:** **leave it alone.** It is 4.2% of the world against 76% in the
  vegetation. Cutting the surface the robot drives on, with 2.43 m of real shape,
  to save 4% is a bad trade.

### Step 5 — `linea1`, and leave the rest alone

`postes.dae` (351,336) + `cables2.dae` (279,476), 2 instances = 1.26 M, 3.6%.
Worth a visual decimation pass if convenient. Cables are thin structure —
conservative on collision.

**Leave `altaniv` and `altaniv_d` alone.** 198 instances but only 0.33 M total
(0.9%); at 4,153 and 563 tris each they are already cheap.

### Step 6 — materials, world file, verification

All park rules apply unchanged — see `docs/map-optimization.md` Steps 4–6:

- **Repair materials after every pymeshlab export.** It renames materials,
  swaps shaders, drops texture maps. Budget for it per asset.
- **Build in a scratch dir**, move only the mesh — pymeshlab silently re-encodes
  PNGs sitting next to the output.
- **`lake.world` inlines everything — VERIFIED.** Zero `<include>` tags. 179
  body definitions, each also appearing in the `<state>` block (bytes
  5867–83113), exactly like `park.world`. **All edits go in the world file, and
  both copies must stay consistent.** It also has **no `<?xml?>` declaration** —
  it starts directly with `<sdf version='1.6'>`. Do not add one.
- **Preserve `laser_retro` — mapping VERIFIED.** 192 tags total. The values are a
  clean per-species code, which makes them a usable ground-truth label source:

  | Value | Model | n |
  |---|---|---|
  | 6 | `altaniv_d` (68) + `altaniv` (31) | 99 |
  | 4 | `arbusto3` (34) + `dry_bush` (30) | 64 |
  | 3 / 2 | `tree_8_v` | 13 each |
  | 1 | `terreno_lago` (ground) | 1 |
  | 12 / 13 | `linea1` (postes / cables) | 1 each |

  Note `tree_8_v` carries **two** values (2 and 3) across its 13 instances, and
  the two bush species **share** value 4 — so the code is not one-to-one with
  species in either direction. Count and re-check these after every edit; a
  dropped or reordered tag breaks classification silently.
- **Re-run the Step 0 baseline** and compare.

---

## Projection

| Asset | Visual now | Approach | Est. visual after |
|---|---|---|---|
| `arbusto3` x34 | 8.49 M | decimate bark hard, keep leaves | ~1.0 M |
| `dry_bush` x30 | 4.91 M | decimate leaves hard, keep them | ~0.75 M |
| `tree_8_v` x13 | 2.61 M | reuse park outputs, delete crown | 0.26 M |
| `terreno_lago` | 1.48 M | 4x subsample at most | ~0.37 M |
| `linea1` x1 | 0.63 M | decimate | ~0.08 M |
| `altaniv*` x99 | 0.17 M | leave alone | 0.17 M |
| **Total** | **18.29 M** | | **~2.6 M** |

Roughly an **86% visual cut**, comparable to park's 89.4%. Collision cut should
be far more conservative — park achieved 75%, and here the terrain and bush
collision are deliberately left near-full, so expect less.

**These are estimates from triangle ratios, not from a measured frame budget.**
Revise after Step 0.

---

## Open questions before starting

1. **Is lake actually the goal?** Everything downstream is park-only: the bag,
   `maps/park_map.pgm`, `park_objects.yaml`, the registry in
   `map_tools/park_types.py`, the landmark detector, all runbooks. Optimizing the
   world is the first step of a long chain — a lake demo also needs map
   extraction, a landmark type registry for lake's species, and a re-tuned
   detector. **The mesh work may be the cheapest part.**
2. **The water has no collision.** `lago` is visual-only, so the robot drives
   through the lake. Deliberate or not, decide before demoing navigation.
   (`lake_tagged.world` gives it a collision box; plain does not.)
3. **Where do outputs go?** Park used `models_opt/`. Lake needs its own tree, and
   the source assets are on an external drive that must stay mounted.
4. **Is there a bag or ground-truth map for lake?** None found in the repo.
