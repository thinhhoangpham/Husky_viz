# Map optimization

How the park map was made runnable: the procedure, in the order you would run it.

This is the **recipe**. The **record** — what was measured, why each triangle
target was picked, which alternatives were rejected, and the dated run logs — is
`park_optimization.md` at the repo root. Nothing here is new; it is that
document re-sequenced as instructions. When a step misbehaves, the reasoning
behind it is there.

---

## What the procedure is

Three operations:

1. **The ground** — a 2-million-triangle mesh for an almost perfectly flat
   surface. Subsampled to 1/16.
2. **The big trees** — leaves deleted outright, bark reduced.
3. **The remaining models** — meshes reduced to fewer, bigger triangles.

Everything else in this file is the handling those three operations need in
order to not break the simulation.

---

## The one fact that governs everything

The Ouster is declared `<sensor type="gpu_ray">` with plugin
`libgazebo_ros_ouster_gpu_laser.so`
(`husky_description/urdf/OS1-64.urdf.xacro:55` and `:80`). It casts against
**collision** geometry, never visual geometry. Visual and collision are separate
per model.

Because the ray pass runs on the GPU, the sensor needs a live GL context —
`gzclient` must be up on `:0` or `/os0_cloud_node/points` has zero publishers.
It also means the lidar and the render **share the GPU**, so collision cost and
render cost are not independent.

So:

- **Visual meshes** can be cut hard. The only budget is the camera: 1280 x 720
  at 104 degrees, so one pixel covers **7.1 mm at 5 m**. Deviation below that
  cannot appear on screen.
- **Collision meshes** should be left alone. Changing one changes the recorded
  laser point cloud. Only the bark was ever cut, and only because leaving it
  full-detail exhausted memory.

Whenever you are unsure whether a reduction is safe, the question is only ever
"is this visual or collision?"

---

## Step 0 — copy, and snapshot before each round

Never edit originals.

    models/                     ->  models_opt/
    natural_environments_ros/   ->  natural_environments_ros_opt/

Back up the world file before each round of edits. The existing backups sit in
`natural_environments_ros_opt/natural_enviroment/worlds/`:

    park.world.bak-before-trees
    park.world.bak-before-path
    park.world.bak-before-bench

Note: the pristine `models/` and `natural_environments_ros/` trees are **not in
this repo** — only the `_opt` copies are. Re-running any of Steps 1-3 requires
mounting the original source drive.

---

## Step 1 — the ground

The ground (`terreno_parque`) was **2,097,152 triangles in a 290 MB file**, for a
surface with **6.9 mm of height variation across the entire 100 x 50 m park**. It
had been authored as a small 176-shade grey image and inflated into two million
triangles by some tool. This step puts it back.

The mesh is a regular grid — exactly 1025 x 1025 vertices — so the reduction is a
**grid subsample, not a decimation**: keep every 4th grid point in each
direction, giving 257 x 257. That is 16x fewer cells, which is why the output is
a power of two.

**2,097,152 -> 131,072 triangles.**

Output: `models_opt/terreno_parque/terreno_parque_lowpoly.dae` (8.3 MB).

Preserve the grass texture and the laser reflectivity settings. The visible
ground is a true rectangle, exactly 100.000000 x 50.000000 m.

### Collision uses the same low-poly mesh

Point both the visual and the collision at
`terreno_parque_lowpoly.dae`. That is what `park.world` does today — verified: it
references the mesh for both, and the terrain collision is the 131,072-triangle
mesh.

This is why park collision totals 1,888,877 triangles and the collision cut is
75%.

---

## Step 2 — the big trees

The 23 `tree_8` trees were **61% of the whole park**: each 165,380 triangles of
trunk and branches plus 35,584 triangles of leaves.

### Delete the leaves

Both visual and collision. The canopy is **17,792 completely separate flat leaf
cards, each already just 2 triangles** — a flat quad cannot go below 2, so
deleting whole leaves is the only available reduction.

This **does change the recorded laser cloud**: the leaves were collision
geometry, the laser sits ~0.9 m up and looks upward to 45 degrees, so it caught
the canopy underside from ~3 m and swept through it from ~10 m. Accepted
knowingly.

It does **not** affect driving or obstacle avoidance: the lowest leaf hangs
**3.99 m** up, the robot is knee-high, and the obstacle map discards everything
above 2 m.

### Reduce the bark — visual

165,380 -> **20,000 triangles**, visual only.

Output: `models_opt/tree_8/bark8_lowpoly.obj` (3.5 MB, exactly 20,000 triangles,
17,096 vertices) plus `bark8_lowpoly.mtl`.

### Reduce the bark — collision, separately

Bark collision was 3,803,740 triangles — **82% of all collision geometry** in the
park (23 trees x 165,380, against 838,403 for everything else combined). This is
the memory problem that forced the cut.

165,380 -> **40,000 triangles** (39,974 actual).

Output: `models_opt/tree_8/bark8_collision.obj` (2.28 MB, 27,099 vertices), pure
`v`/`f` geometry with no UVs, normals, or material references. All 23
`<collision>` blocks point at it.

**Decimate this fresh from the original. Do not reuse the 20,000-triangle visual
mesh as collision.** Measured, it is worse than a fresh 9,974-triangle
decimation despite having twice the triangles: 8.63 mm mean ray-hit shift against
0.89 mm, 236 mm max against 31 mm, it is biased *outward*, and its bounding box
has shrunk from 13.98 m to 13.52 m, meaning it has lost extremity geometry.

There is a **hard floor at 8,966 triangles**: the bark has 8,964 boundary
(one-face) edges, which cannot collapse with boundary preservation on.

---

## Step 3 — the remaining models

Same operation, visual meshes only, target chosen per model.

### The path

`camino_parque` was **565,088 triangles in an 85 MB file** — 40% of everything on
screen once the ground and trees were handled.

It is a 1024 x 1024 lattice over the same 100 x 50 m footprint as the ground, but
with **only 27.2% of lattice sites occupied** — an X-shaped pair of wide bands
with grass between. Nearly flat: 82 mm of variation, sitting ~5 cm above the
grass.

Being a lattice, it takes the same every-4th-point subsample as the ground —
**visual only**.

| | Original | After |
|---|---|---|
| Visual | 565,088 | **34,380** |
| Collision | 565,088 | 565,088 (unchanged) |
| File | 85,378,897 bytes | **2,986,809 bytes** |

Output: `models_opt/camino_parque/camino_parque_lowpoly.dae` (17,850 vertices).

Accepted consequence: a coarse cell can only exist where all four corners exist,
so the drawn outline **erodes by up to 3 original grid cells — about 39 cm along
x, 20 cm along y**, 2.7% of path area, appearing as a thin grass fringe. The
collision shape keeps the exact original outline, so wheels and laser still see
the path at true width. `<laser_retro>7</laser_retro>` is untouched.

Two notes: a pure "all four corners present" reconstruction yields 565,314
triangles where the file declares 565,088, so **113 quads (0.04%) do not follow
the plain grid rule** and the rebuild does not reproduce them. And
`camino_parque.dae` has an empty `<library_images/>` with no material, effect, or
binding blocks at all — its texture comes entirely from the SDF side
(`vrc/terrain` -> `camtext.jpg`), consuming the mesh UVs. So Step 4 does not
apply to the path; only the UVs mattered, and they were preserved.

### The benches

The 16 benches were 131,264 triangles, 15% of what remained on screen.

`Bench_1.dae` is 8,204 triangles in 5 sub-meshes. The two heavy ones
(`NurbsPath_003` at 4,640 and `NurbsPath_001` at 2,888) are swept curves — the
slats — carrying redundant geometry along their length. **Not a lattice**, so the
subsample does not apply; this one needs true quadric decimation.

8,204 -> **2,000 triangles**, visual only. All 16 collision blocks still
reference the original mesh.

Output: `models_opt/bench/Bench_1_lowpoly.dae` (2,000 triangles, 1,074 vertices,
275,744 bytes, down from 1,061,982).

**pymeshlab flattens the 5 sub-meshes into 1 at import**, before any filter runs,
and no setting prevents it. Harmless here only because all 5 `instance_material`
bindings point at the same single material — the file has exactly one
`library_materials` and one `library_effects` entry. Check this before assuming
it is safe on another asset.

---

## Step 4 — repair the material after every pymeshlab export

pymeshlab rewrites materials on export. It has to be undone by hand, per model.
Left alone, the model renders untextured.

**Trees.** pymeshlab named the material `material_0`, dropped the bark
surface-detail map, and changed the shading values. Repair: rewrite the `.mtl` as
a byte-for-byte copy of the original `bark8.mtl`, then edit the `.obj` to
reference `bark8_lowpoly.mtl` and `usemtl Bark01_SHD`.

**Benches.** pymeshlab renamed the material to `material0`, the effect to
`material0-fx`, the image to `texture0`, replaced the original `<lambert>`
(emission `0 0 0 1`, ior `1.45`) with a generic `<blinn>` template, and changed
the texcoord name from `UVMap` to `UVSET0`. Repair: copy the original's blocks in
verbatim. Add a guard asserting none of pymeshlab's strings survive in the file.

### The texture trap

**Saving a mesh with pymeshlab into a model folder silently overwrites the
texture images next to it.** It happened twice before anyone noticed — once on
the trees, once on the benches.

pymeshlab does not copy the original image. It holds it in memory as raw pixels
and writes a fresh PNG with its own compression settings. Pixels unchanged, only
the encoding differs.

**Rule: build meshes in a scratch directory and move only the mesh file into
place.** Otherwise verify the model folder's textures afterwards.

The two files this already hit —
`models_opt/tree_8/JA02_Bark01_dif_su.png` and
`models_opt/bench/Bench_1_Base_Color.png` — were verified pixel-identical to
their originals (same dimensions, decompressed pixel stream byte-identical; only
the PNG row-filter choice differs) and were deliberately left as they are.

---

## Step 5 — write the edits into the world file

**`park.world` inlines everything.** It does not use `<include>`; all 94 models
are written out in full inside the world file. **Editing
`models_opt/tree_8/model.sdf` has no effect whatsoever on the park.**

All model edits go into:

    natural_environments_ros_opt/natural_enviroment/worlds/park.world

Each model name appears **exactly twice**: once in the `<state>` block near the
top (pose and velocity only, no geometry) and once in the body as a full
definition. Keep both consistent — a `<state>` entry for a link that no longer
exists makes Gazebo warn.

`park.world` has **no `<?xml?>` declaration**. It starts directly with
`<sdf version='1.6'>`. Do not add one.

---

## Step 6 — verify

- **Triangle census**, visual and collision separately.
- **`<laser_retro>` survived the edits**: `7` on the path, and `2` still
  appearing 38 times after the bark collision swap.
- **All 16 bench collision blocks** still reference the original mesh, not the
  low-poly one.
- **No pymeshlab strings** (`material_0`, `material0`, `material0-fx`,
  `texture0`, `UVSET0`) remain in any repaired file.
- **Load the world** and confirm zero errors with physics at real-time factor
  ~1.0.
- **Confirm the laser publishes.** `/os0_cloud_node/points` must have a
  publisher and actually emit at ~10 Hz with stable memory — check message count,
  not just publisher presence. Requires `gzclient` on `:0`: the gpu_ray sensor
  needs a live GL context.

---

## Result

| | Original | After | Cut |
|---|---|---|---|
| Visual triangles | 7,557,727 | **799,503** | **89.4%** |
| Collision triangles | 7,557,727 | **1,888,877** | **75.0%** |

---

## Known-unverified

Carried over from `park_optimization.md`; worth re-checking on any rebuild.

- **Tree bark material repair has not been confirmed by eye.**
- **Bench texcoord `set` attribute.** pymeshlab writes the geometry's texcoord
  input *without* a `set` attribute where the original has `set="0"`. An absent
  `set` is conventionally read as 0, and the restored `bind_vertex_input` does
  carry `input_set="0"`, so it should bind — but this is unverified. **If the
  benches render untextured or with scrambled UVs, adding `set="0"` to that line
  is the first thing to try.**

Inherited, not introduced: the bench texture reference is
`<init_from>/Bench_1_Base_Color.png</init_from>` — with a **leading slash**, an
absolute path to filesystem root. This is authored into the source asset; the
pristine `models/bench/Bench_1.dae` is md5-identical and carries the same line.
Kept as-is. If it is broken, it is broken for all 16 benches equally and predates
this work.
