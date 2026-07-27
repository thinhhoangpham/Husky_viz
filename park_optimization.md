# Park simulation optimization

Making `park.world` actually runnable in Docker on an Apple Silicon Mac.

**Status: it runs.** Loaded with zero errors, physics at full real-world speed
(real time factor 0.999). Previously it could not be run at all.

Everything below was measured, not estimated. Where an earlier figure turned out
to be wrong it is corrected and the wrong value is named, so old notes quoting it
can be recognised as stale.

---

## Some words used here

A 3D object in a computer is not solid. It is a hollow shell, like a papier-mache
model, built out of thousands of tiny flat **triangles** glued edge to edge. More
triangles means smoother and more detailed, but slower to draw. Fewer triangles
means chunkier, but faster.

Every object in Gazebo carries **two separate shapes**:

- the **look-at shape**, used only for drawing on screen
- the **bump-into shape**, used for physics and for the laser scanner

These are independent. In this park the two were originally the same file, which
is what made everything so expensive.

**Which one the laser uses matters.** The Ouster is declared as
`<sensor type="ray">` (`husky_description/urdf/OS1-64.urdf.xacro:55`), meaning it
casts rays against **bump-into shapes**. Visual quality has zero effect on laser
data. This is the single fact that made most of the work safe.

---

## Result

| | Original | Now | Cut |
|---|---|---|---|
| Triangles drawn on screen | 7,557,727 | **799,503** | **89.4%** |
| Triangles the laser measures against | 7,557,727 | **1,888,877** | **75.0%** |

Five changes got there: the ground, the trees, the path, the benches, and the
bark collision mesh. (The laser figure rose from 1,757,805 to 1,888,877 on
2026-07-23 when the terrain collision was switched from a heightmap to the
131,072-triangle low-poly mesh — see the fix below.)

> **The laser works (fixed 2026-07-23).** It was broken by the terrain's
> `<heightmap>` collision, which the CPU ray sensor cannot cast against — it froze
> the whole simulation and leaked memory to OOM without ever producing a scan.
> Fixed by switching the terrain collision to the low-poly trimesh; the laser now
> publishes at **~10 Hz** with stable memory. See "The laser was broken by the
> terrain heightmap".

---

## Safety: originals were never touched

All work happened in copies. The originals in `models/` and
`natural_environments_ros/` are pristine — this was re-verified on 2026-07-21
after the bench work.

| Copy | Contents | Status |
|---|---|---|
| `models_opt/` | all 97 model folders | 768 files, 13,681,668,904 bytes — identical to source **except two textures, see below** |
| `natural_environments_ros_opt/` | the simulation package | 376 files, 128,203,007 bytes, identical to source |

**Correction to an earlier claim in this file.** `models_opt/` is *not* a
byte-identical copy of `models/`. Two texture images differ:

| File | Changed | Cause |
|---|---|---|
| `models_opt/tree_8/JA02_Bark01_dif_su.png` | 2026-07-21 16:25 | pymeshlab re-save during the tree work |
| `models_opt/bench/Bench_1_Base_Color.png` | 2026-07-21 22:32 | pymeshlab re-save during the bench work |

Both were verified **pixel-identical** to their originals — same dimensions, and
the decompressed pixel stream is byte-identical (50,335,744 bytes for the bench
texture either way). Only the PNG row-filter choice differs, which changes the
compressed size by ~69 KB and nothing else. **Deliberately left as they are**;
no visual or functional difference exists. See "The pymeshlab texture trap".

947 macOS metadata stub files (`._*`) were deleted from `models_opt/`. The
external drive is ExFAT and cannot store macOS extended attributes, so `cp -a`
creates these sidecars. They regenerate whenever a file carrying a
`com.apple.provenance` attribute is rewritten; deleting a sidecar does not
prevent recurrence, because the attribute stays on the real file.

Backups of the world file, one per round of work, all in
`natural_environments_ros_opt/natural_enviroment/worlds/`:

- `park.world.bak-before-trees`
- `park.world.bak-before-path`
- `park.world.bak-before-bench`

---

## Change 1 — the ground

The park's ground was a single 2,097,152-triangle shape in a 290 MB file, for a
surface with only **6.9 mm of height variation across the entire 100 x 50 m
park**. It had originally been drawn as a small 176-shade grey image and inflated
into two million triangles by some authoring tool. It was put back.

| Purpose | New file | Size |
|---|---|---|
| What the robot drives on | `models_opt/terreno_parque/terreno_parque_heightmap.png` | 250 KB |
| What you see | `models_opt/terreno_parque/terreno_parque_lowpoly.dae` | 8.3 MB, 131,072 triangles |

**2,097,152 → 131,072 triangles.**

The reduction was a clean grid subsample, not a generic simplification. Both
meshes are square grids: the original has exactly 1025 x 1025 vertices, the
replacement 257 x 257 — every 4th grid point in each direction, so 16x fewer.
That is why the result is a power of two rather than a round number.

The visible ground was verified to be a true rectangle, exactly
100.000000 x 50.000000 m. Laser reflectivity settings and the grass texture were
preserved.

### Two traps found the hard way

**Gazebo cannot read 16-bit height images.** `ImageHeightmap.cc`'s
`FillHeightMap()` reads a single byte per pixel and divides by 255. A 16-bit
image yields the low byte, which is full-amplitude noise. It must be 8-bit. This
turned out to be finer than the source data anyway — the original height field
contains only **176 distinct levels**.

**Gazebo's heightmaps are always square.** `Heightmap.cc:816` sets the world size
from a single scalar (`terrainSize.X()`), and `:611-614` subtracts half of that
same value from both x and y. A rectangular park rendered as a square. This is
why the *look-at* shape is a mesh file and not a heightmap; only the *bump-into*
shape is a heightmap, where the squareness does not apply the same way.

---

## Change 2 — the trees

The 23 big trees were 61% of the whole park. Each carried 165,380 triangles of
trunk and branches plus 35,584 triangles of leaves.

**What was done:**

- **Leaves deleted entirely**, both look-at and bump-into. Deliberate decision.
- **Trunk and branches rebuilt from 165,380 down to 20,000 triangles, for looks
  only.** The bump-into version stays at full original detail, so what the laser
  measures is bit-for-bit unchanged.

New file: `models_opt/tree_8/bark8_lowpoly.obj` (3.5 MB, exactly 20,000
triangles, 17,096 vertices) plus `bark8_lowpoly.mtl`.

### Why 20,000 was chosen

Rebuilding with fewer triangles shifts the surface slightly. Measured deviation
from the original:

| Target | Typical deviation | Worst case (twig tips) |
|---|---|---|
| 80,000 | 1.67 mm | 46 mm |
| 40,000 | 2.52 mm | 46 mm |
| **20,000** | **3.36 mm** | 59 mm |
| 10,000 | 3.97 mm | 88 mm |

The simulation's camera is 1280 x 720 at 104 degrees. At 5 m, one pixel covers
**7.1 mm**. Every option is below that, so the shift cannot appear in a photo.
20,000 captures nearly all the speed gain; going to 10,000 adds only 8% more
speed while eating visibly more thin twigs.

### Why the leaves could not be simplified

The canopy is **17,792 completely separate flat leaf cards, each already just 2
triangles**. That is the floor — a flat quad cannot go below 2 triangles. The
only possible reduction is deleting whole leaves, which is what was done.

### Consequence of removing leaves — accepted knowingly

The leaves were bump-into shapes, so the laser did measure them. The laser sits
about 0.9 m up and looks upward to 45 degrees, so from ~3 m away it caught the
underside of the canopy and from 10 m away it swept through the middle of it.

**Removing the leaves therefore changes the recorded laser point cloud** relative
to the original dataset. It does not change driving or obstacle avoidance at all:
the lowest leaf hangs **3.99 m** above the ground, the robot is knee-high, and
the obstacle map discards everything above 2 m.

### The material file needed hand repair

pymeshlab wrote its own material file naming the material `material_0`, dropping
the bark surface-detail map, and changing the shading values. Left alone the
trees would have rendered untextured. It was rewritten as a byte-for-byte copy of
the original `bark8.mtl`, and the `.obj` was edited to reference
`bark8_lowpoly.mtl` and `usemtl Bark01_SHD`.

**This has not been confirmed by eye.** See "What remains".

---

## Change 3 — the path

Done 2026-07-21. The path (`camino_parque`) was the largest single item drawn
once the ground and trees were dealt with: **565,088 triangles in an 85 MB
file**, 40% of everything on screen.

**What it actually is.** A 1024 x 1024 lattice covering the same 100 x 50 m
footprint as the ground, but with **only 27.2% of the lattice sites occupied** —
an X-shaped pair of wide bands with grass showing through elsewhere. It is
almost perfectly flat: 82 mm of height variation across the whole park, sitting
as a ~5 cm raised surface over the grass.

**Why the ground's recipe could not be copied.** The ground's bump-into shape
became a heightmap. Gazebo heightmaps are always solid rectangles and cannot
have holes; the path is a shape *with* holes. A heightmap would have paved the
entire park including the grass.

**What was done instead:** the same every-4th-lattice-point subsample as the
ground, applied to the look-at shape only.

| | Original | Now |
|---|---|---|
| Look-at triangles | 565,088 | **34,380** |
| Bump-into triangles | 565,088 | 565,088 (unchanged) |
| File size | 85,378,897 bytes | **2,986,809 bytes** |

New file: `models_opt/camino_parque/camino_parque_lowpoly.dae`, 17,850 vertices.

**Accepted consequence.** A coarse cell can only exist where all four of its
corners exist, so the drawn outline **erodes by up to 3 original grid cells —
about 39 cm along x, 20 cm along y**, which is 2.7% of the path area. On bands
roughly 15 m wide that is a ~2.6% narrowing per edge, appearing as a thin grass
fringe. The bump-into shape keeps the exact original outline, so the laser and
the wheels still see the path at its true width, and `<laser_retro>7</laser_retro>`
is untouched.

**Two things worth recording:**

- A pure "all four corners present" reconstruction yields 565,314 triangles where
  the file declares 565,088 — **113 quads (0.04%) do not follow the plain grid
  rule**. The rebuild does not reproduce those exceptions.
- `camino_parque.dae` has an **empty `<library_images/>` and no material, effect
  or binding blocks at all**. Its texture comes entirely from the SDF side
  (`vrc/terrain` → `camtext.jpg`), which consumes the mesh's UV coordinates. So
  the material trap that hit the tree bark could not apply here; only the UVs
  mattered, and they were preserved.

---

## Change 4 — the benches

Done 2026-07-21. The 16 benches were 131,264 triangles, 15% of what remained on
screen.

`Bench_1.dae` is 8,204 triangles in 5 sub-meshes; the two heavy ones
(`NurbsPath_003` at 4,640 and `NurbsPath_001` at 2,888) are swept curves — the
slats — which carry a lot of redundant geometry along their length and decimate
well. This is **not** a lattice, so the ground/path subsample did not apply;
true quadric decimation via pymeshlab was needed.

Measured deviation, converted to world millimetres at the bench's 0.15 scale:

| Target | Mean | Worst case | Mean equals one pixel at |
|---|---|---|---|
| 6,000 | 0.03 mm | 3.2 mm | 2 cm |
| 4,000 | 0.16 mm | 5.8 mm | 11 cm |
| **2,000** | **0.60 mm** | **13.9 mm** | **42 cm** |
| 1,000 | 1.98 mm | 32.8 mm | 1.4 m |
| 500 | 7.99 mm | 76.5 mm | 5.6 m |

**2,000 was chosen** — the mean is still sub-millimetre against the 7.1 mm
pixel, and it saves 99,264 triangles across the 16 benches. New file:
`models_opt/bench/Bench_1_lowpoly.dae`, 2,000 triangles, 1,074 vertices,
275,744 bytes (down from 1,061,982). Look-at only; all 16 collision blocks still
reference the original mesh.

**pymeshlab flattens the 5 sub-meshes into 1 at import**, before any filter runs,
and no setting prevents it. This is harmless here because all 5
`instance_material` bindings pointed at the *same* single material — the file has
exactly one `library_materials` and one `library_effects` entry.

**The material needed hand repair again**, exactly as the trees did. pymeshlab
renamed the material to `material0`, the effect to `material0-fx`, the image to
`texture0`, replaced the original `<lambert>` (emission `0 0 0 1`, ior `1.45`)
with a generic `<blinn>` template, and changed the texcoord name from `UVMap` to
`UVSET0`. All of it was undone by copying the original's blocks in verbatim, and
a guard asserts none of pymeshlab's strings survive in the file.

**Two things deliberately not changed:**

- The texture reference is `<init_from>/Bench_1_Base_Color.png</init_from>` —
  **with a leading slash**, an absolute path to filesystem root. This is authored
  into the source asset; pymeshlab did not add it, and the pristine
  `models/bench/Bench_1.dae` is md5-identical and carries the same line. It was
  kept as-is so the new mesh behaves exactly as the old one. If it turns out to
  be broken it is broken for all 16 benches equally and predates this work.
- pymeshlab writes the geometry's texcoord input **without a `set` attribute**
  where the original has `set="0"`. An absent `set` is conventionally read as 0
  and the restored `bind_vertex_input` does carry `input_set="0"`, so it should
  bind — but this is unverified. **If the benches render untextured or with
  scrambled UVs, adding `set="0"` to that line is the first thing to try.**

---

## Change 5 — the bark collision mesh

Done 2026-07-21, and the only change so far that deliberately alters what the
laser measures. Motivated by the memory failure described in the next section.

Collision geometry was 4,642,143 triangles, of which **bark was 3,803,740 (82%)**
— 23 trees x 165,380. Everything else together is 838,403.

Measured trade-off before choosing. Deviations are Hausdorff, sampled both
directions, at the bark's world scale of 1.0 (verified from `park.world`):

| Bark mesh | Tris | Ray-hit shift mean / max | Detail with no counterpart | Bias | Park collision after | Cut |
|---|---|---|---|---|---|---|
| fresh 80,000 | 79,974 | 0.30 mm / 31 mm | 7.9 mm | none (−0.04) | 2,677,805 | 42% |
| **fresh 40,000** | **39,974** | **0.89 mm / 31 mm** | **24.9 mm** | **−0.33 mm inward** | **1,757,805** | **62%** |
| existing `bark8_lowpoly.obj` | 20,000 | 8.63 mm / 236 mm | 24.4 mm | +4.67 mm outward | 1,298,403 | 72% |
| fresh 10,000 | 9,974 | 3.82 mm / 77 mm | 61.4 mm | −3.13 mm inward | 1,067,805 | 77% |
| floor | 8,966 | 3.66 mm / 159 mm | 76.5 mm | −3.27 mm inward | 1,044,621 | 77.5% |

The "Park collision after" figures here are as of Change 5, when the ground
collision was still a heightmap (0 triangles). The later terrain-collision fix
(2026-07-23) added the 131,072-triangle ground mesh, so the current park
collision total is 1,888,877, not 1,757,805 — see the census and "The laser was
broken by the terrain heightmap".

**40,000 was chosen.** New file `models_opt/tree_8/bark8_collision.obj`, 39,974
triangles, 27,099 vertices, 2.28 MB, pure `v`/`f` geometry with no UVs, normals
or material references. All 23 `<collision>` blocks now point at it;
`<laser_retro>2</laser_retro>` is unchanged and still appears 38 times.

**Two error modes, and they must not be conflated.** *Ray-hit shift* is a ray
still hitting bark but at a slightly different distance. *Detail with no
counterpart* is original twig geometry deleted outright, so those rays stop
returning anything at all. The second grows much faster than the first. For
reference the physical Ouster OS1-64's range accuracy is about ±30 mm — a
datasheet figure, not a measurement, and the simulated laser has no such noise.

**The existing 20,000-triangle `bark8_lowpoly.obj` is unusable as collision.**
Measured 8.63 mm mean shift and 236 mm max — worse than a fresh 9,974-triangle
decimation despite twice the triangles — it is the only candidate biased
*outward*, and its bounding box has shrunk from 13.98 m to 13.52 m, so it has
lost extremity geometry. It stays visual-only. Do not reuse it for collision.

**A hard floor at 8,966 triangles.** The bark has 8,964 boundary (one-face)
edges; with boundary preservation on they cannot collapse. No bark-only change
can take park collision below 1,044,621 triangles.

Height distribution of the original bark, re-verified: **97.27% above 4 m**,
0.44% between 2–4 m, **2.29% below 2 m**. Median triangle edge 69.1 mm, 95th
percentile 132.2 mm.

---

## The laser was broken by the terrain heightmap — FIXED 2026-07-23

For most of this project the laser was the blocking problem. Spawn the robot into
the park and gzserver's memory climbed steadily until the kernel killed it
(`OOMKilled: true`), and **no scan was ever produced** — not on the ROS topic
`/os0_cloud_node/points`, not on the Gazebo-internal topic
`/gazebo/default/husky/base_link/os0_sensor-OS1-64/scan`. The root cause is now
found and the laser works. The account below is kept because the path to the
answer corrected several earlier guesses.

**Root cause: the CPU ray sensor cannot cast against a `<heightmap>` collision.**
The park's terrain collision was a Gazebo `<heightmap>` — introduced by the
optimization (the original used a trimesh; see the census note). The instant the
Ouster is activated, its first update tries to cast rays against that heightmap
and enters a non-terminating, memory-allocating path that **never returns**.
Because that runs on the world-update thread, it takes the whole simulation down:
physics stops, nothing publishes, and the stuck query allocates until OOM. On
real hardware the sensor runs on the **GPU**, which rasterises depth and never
does this physics ray query — which is why the dataset could be recorded live but
the CPU sim could not reproduce it.

**How it was proven — staged single-variable tests, 2026-07-23.**

*The sensor freezes physics.* Measuring sim_time (does the clock advance?)
alongside memory, adding one thing at a time to the same park:

| Stage | Running | sim_time over ~5 s wall | Memory |
|---|---|---|---|
| A | Bare park world | advances, RTF ~1.0 | flat, low |
| B | + robot + laser, **sensor idle** (no subscriber) | advances, RTF ~1.0 | flat 3.7 GB |
| C | **sensor activated** | one step, then **frozen** | climbs, no ceiling |

The clock froze the instant the sensor turned on, and zero scans came out.

*It is not the ray count.* Cutting the sensor 64x (32,768 → 512 rays) barely moved
the leak rate — 34 MB/s → 26 MB/s — while the idle footprint dropped 3,745 → 835
MB. So the leak is a fixed per-tick cost, not per-ray work, and even 512 tiny rays
still published nothing. This killed the earlier "sweep too slow to finish" idea.

*The culprit is the heightmap, not the trees.* Replacing only the terrain's
heightmap collision with a flat plane — every tree, path and bench collision left
in place — flipped the result completely:

| | Heightmap collision | Flat-plane collision |
|---|---|---|
| sim_time | frozen | advances, RTF ~0.33 |
| Memory | leaks to OOM | flat 3,737 MB |
| Publish rate | 0 | **10.005 Hz** |

The trees were still in the scene and the rays hit them fine, so trimesh
collisions are innocent — the heightmap alone is the trigger. This also explains
the old clue that the same sensor ran at **10.046 Hz** in `husky_empty_world`:
that world's ground is a flat plane, never a heightmap.

**The fix: terrain collision → the low-poly trimesh.** The terrain does not need a
heightmap collision. The optimization had already built a decimated 8.3 MB terrain
mesh (`terreno_parque_lowpoly.dae`) and wired it up only as the *visual*. Pointing
the **collision** at that same mesh (scale `50 25 0.01`, matching the visual)
gives a trimesh collision — which the laser handles — at 1/16th the triangles of
the original. One block changed in `park.world`, model `parque`, the collision
`<geometry>`: `<heightmap>…</heightmap>` → the lowpoly `<mesh>`. Backup:
`park.world.bak-before-mesh-collision`.

**Validated, 2026-07-23:**

| Check | Before (heightmap) | After (lowpoly mesh) |
|---|---|---|
| Laser publish rate | 0, never | **9.994 Hz** |
| Points per scan | none | **16,386**, dense |
| Memory | leaks to OOM | **flat 3,783 MB** |
| Physics (sim_time) | frozen | steps, RTF ~0.32 |
| Robot on terrain | — | rests, Z stable 3.119 m |

The one cost is real-time factor ~0.32 — the sim runs at ~1/3 wall-clock speed,
the inherent price of 32,768 CPU ray-vs-trimesh casts per tick with no GPU. It is
not the leak and not a fault: the data is correct at 10 Hz of *sim* time, which is
what a recorded dataset needs. Fewer rays or the GPU plugin would speed it up if
ever wanted.

**Collision accuracy vs. the original terrain.** The low-poly collision was
measured against the full-res original collision (`terreno_parque.dae`, 2,097,152
triangles), both scaled to world metres (`50 25 0.01`), Hausdorff both directions:
**mean deviation 0.02 mm, RMS 0.13 mm, worst-case 4.4 mm** at a single point.
Horizontally the match is sub-millimetre against 50–100 m extents; vertically the
terrain has only 6.9 mm of world-space relief, so the worst single point is ~64%
of that relief while the mean error is ~0.3%. In practice the robot and laser see
the same surface as the original terrain, at 1/16th the triangles.

**Superseded earlier guesses** (recorded so old notes are recognisable as stale):
the growth was once attributed to collision *sizing* (the 62% bark cut only
lowered the starting point, never the rate), and a live heap sample suggested
unit-direction-vector-shaped doubles — neither was the cause. The cause is the
heightmap collision, full stop.

---

## The pymeshlab texture trap

**Saving a mesh with pymeshlab into a model folder silently overwrites the
texture images sitting next to it.** This happened twice before it was noticed —
once during the tree work and once during the bench work.

Proven, not inferred: loading `Bench_1.dae` and calling `save_current_mesh()`
into an *empty* directory emits `Bench_1_Base_Color.png` alongside the mesh,
12,629,526 bytes, md5 `16e5ce0f455d17e483e3bf2211b0fba5` — byte-for-byte the
file now in `models_opt/bench/`, against `72e686a0…` for the pristine original.

pymeshlab does not copy the original file. It holds the image in memory as raw
pixels and writes a fresh PNG from them with its own compression settings. The
pixels are unchanged; only the encoding differs.

**Rule for future work: build meshes in a scratch directory and move only the
mesh file into place, or verify the model folder's textures afterwards.**

---

## Critical structural fact: the world file inlines everything

`park.world` does **not** use `<include>`. All 94 models are written out in full
inside the world file. Editing `models_opt/tree_8/model.sdf` has **no effect
whatsoever** on the park.

All model edits must be made in
`natural_environments_ros_opt/natural_enviroment/worlds/park.world`.

Each model name appears exactly twice: once in the `<state>` block near the top
(pose and velocity only, no geometry) and once in the body as a full definition.
Both copies must be kept consistent — a `<state>` entry for a link that no longer
exists makes Gazebo warn.

Also note: `park.world` has **no `<?xml?>` declaration**. It starts directly with
`<sdf version='1.6'>`. Do not add one.

### What is actually in the park

94 models plus a single directional light named `sun`:

| Object | Count | Model name in the file |
|---|---|---|
| Big trees | 23 | `tree_8*` |
| Benches | 16 | `bench*` |
| Lamp posts | 15 | `lamp*` |
| Small trees | 15 | `arbolpartes4*` |
| Rubbish bins | 11 | `trash_bin_1*` |
| Garden tables | 11 | `garden_table*` |
| Ground | 1 | `parque` |
| Path | 1 | `camino_parque` |
| Dead junk | 1 | `Untitled2` |

The Spanish names: `parque` is the ground, `camino_parque` the park path,
`arbolpartes4` the small trees. The `_clone` suffixes come from the author
copy-pasting one object repeatedly in the Gazebo editor.

`Untitled2` points at `/home/a/Desktop/modelos_mundo_dataset/terreno_dataset.dae`
— the original author's own machine — and sits at z ≈ −1.76 x 10^8 m, about
176,000 km below the world. Deliberately left broken; see `park_world_notes.md`.

---

## Verification of the world edits

### Trees

| Check | Expected | Measured |
|---|---|---|
| leaf shape references | 0 | **0** |
| full-detail bark (bump-into) | 23 | **23** |
| simplified bark (look-at) | 23 | **23** |
| models still present | 94 | **94**, each name exactly twice |
| small trees and their links | unchanged | **unchanged** |
| ground work still intact | yes | heightmap, lowpoly mesh and `vrc/parque` material all present |
| file parses as XML | yes | **OK** |

One expectation was wrong and is worth recording: `<laser_retro>2</laser_retro>`
appears **38** times, not 23. Twenty-three belong to the big trees, fifteen to the
small trees. The big trees' leaves used `<laser_retro>3</laser_retro>`, so
deleting them removed instances of `3`, not `2`.

### Path

| Check | Expected | Measured |
|---|---|---|
| lines changed in `park.world` | 1 | **1** (line 129, the visual URI) |
| collision URI | still `camino_parque.dae` | **unchanged**, `laser_retro 7` intact |
| new mesh triangles | 34,380 | **34,380**, 17,850 vertices |
| UV and normal array lengths | consistent | **consistent**; 500 random vertices spot-checked, 0 mismatches |
| non-geometry bytes vs original | identical | **byte-identical** |
| file parses as XML | yes | **OK** |

### Benches

| Check | Expected | Measured |
|---|---|---|
| lines changed in `park.world` | 16 | **16**, at an even 89-line stride |
| split of URIs | 16 collision original / 16 visual lowpoly | **exactly that**, no collision touched |
| new mesh triangles | 2,000 | **2,000**, 1,074 vertices |
| scales | unchanged | all 16 still `0.15 0.15 0.15` |
| `<state>` block | unchanged | **byte-identical to backup** |
| material blocks vs original | identical | **byte-identical** (`library_images`, `library_effects`, `library_materials`, binding) |

### Bark collision

| Check | Expected | Measured |
|---|---|---|
| lines changed in `park.world` | 23 | **23**, all collision URIs |
| `model://tree_8/bark8.obj` remaining | 0 | **0** |
| `bark8_collision.obj` | 23, all collision | **23**, context verified by walking the file |
| `bark8_lowpoly.obj` | 23, all visual | **23**, untouched |
| `<laser_retro>2</laser_retro>` | 38 | **38** |
| new mesh triangles | 39,974 | **39,974**, 27,099 vertices |
| files in `models_opt/tree_8/` | unchanged | **32/32 byte-identical**, md5 checked file by file |
| file parses as XML | yes | **OK** |

The build ran in a scratch directory precisely because of the texture trap
below — pymeshlab did emit `JA02_Bark01_dif_su.png` alongside the mesh, but it
landed in the scratchpad and only the `.obj` was moved into the project. This
is the containment that should be used every time.

---

## Test results

Run on 2026-07-21, **after** the path work but **before** the bench work. World
only — see the gaps below.

| Measurement | 21 Jul, ground + trees | 21 Jul, + path |
|---|---|---|
| Errors on load | zero | **zero** (only harmless TIFF/EXIF noise) |
| Real time factor | 0.999 | not re-measured |
| Container memory | 2.816 GiB | **2.518 GiB** of 9.703 GiB |
| gzserver | 10.7% CPU, 1.07 GB | **9.2% CPU, 1.02 GB** |
| gzclient | 306% CPU, 1.75 GB | **331% CPU, 1.41 GB** |

Memory fell ~300 MB, tracking the 82 MB the path mesh shed plus its in-memory
form. CPU is essentially unchanged, as expected — gzclient's cost here is
dominated by software-rendering the tree canopies, not the path.

Docker at time of test: **10.4 GB memory, 11 CPUs**.

### What the tests did NOT cover

- **The robot has still never been driven.** It has been spawned, but never
  given a velocity command, and no waypoint mission has been run.
- **The sim laser has not yet been recorded to a bag.** As of 2026-07-23 it
  produces live scans in the park at ~10 Hz (see the fix above); capturing them
  to a rosbag and comparing against `park_1.bag` is still to do.

### Runs with the robot, 2026-07-21

| Configuration | gzserver | Laser | Outcome |
|---|---|---|---|
| Empty world + robot | 3.13 GB | **10.046 Hz** | stable |
| Park (4.6M collision) + robot | 8.11 GB | none | OOM-killed |
| Park (1.76M collision) + robot | 3.72 GB rising to 7.68 GB | none | OOM-killed |
| Park (1.76M collision) + robot, **no laser** | **0.84 GB, flat** | n/a | stable |

The visual check was completed in this session and passed — see "What remains".

### Runs with the robot, 2026-07-23 (the laser fix)

| Configuration | gzserver | Laser | Outcome |
|---|---|---|---|
| Park, **heightmap** collision + robot + laser | frozen, leaks to OOM | none | OOM-killed |
| Park, **flat-plane** collision + robot + laser | 3.74 GB, flat | **10.005 Hz** | stable (isolation test) |
| Park, **lowpoly-mesh** collision + robot + laser | **3.78 GB, flat** | **9.994 Hz** | **stable — the fix** |

The lowpoly-mesh row is the shipped state: 16,386 points/scan, memory flat, RTF
~0.32, robot resting on the terrain (Z stable at 3.119 m).

---

## What we learned about this system

These findings redirected the work several times and remain relevant.

**The robot is blind.** `fixer_husky.py` subscribes only to compass and GPS. Its
entire control law is `linear.x = 0.3` and `angular.z = -5.5 * yaw_error`, aimed
at seven hard-coded GPS waypoints. Measured average speed in `park_1.bag`:
**0.303 m/s** — it never deviated, because nothing could make it.

**The camera is not a live sensor.** `get_real_cam.py` runs a second pass: after
the drive, the bag is replayed and every 40th pose teleports a free-floating
stereo camera to the recorded robot position and photographs the scene.
253,500 / 40 = 6,337, matching the bag's 6,337 image pairs exactly. Repeated
against `park_tagged.world`, where every object is painted a flat identity colour,
to produce segmentation labels.

**The stereo cameras on the robot are disabled** in `sensor_description.urdf`.

**The obstacle map is not part of the dataset.** `costmap_params.yaml` sits loose
at the project root and is read only by `park_rviz.launch` and
`replay_park.launch`. The dataset's own `add_husky_park_1.launch` runs no
navigation at all — only control, teleop and spawn. There is a *second*,
different costmap config inside `husky_navigation/config/` that uses a flat 2D
laser at 5 cm resolution with no height limit; do not confuse the two.

**`tree_8_v/` is a byte-identical twin of `tree_8/`**, used only by `lake.world`,
never by the park.

---

## Corrections to earlier notes

If you are reading an older summary, these figures in it are wrong.

| Claimed earlier | Measured truth |
|---|---|
| 8,400,000 triangles originally | **7,557,727** |
| Small trees: 68,778 each, 1,031,670 total | **8,455 each, 126,825 total** — off by 8x |
| "Trunk detail is finer than a hair" | average bark triangle is **32 mm** across |
| "About 1 GB of photos" is a memory problem | only 5 tree images are used, ~**67 MB**, shared. Never a problem |
| Docker has only 7.65 GB, needs raising | it has **10.4 GB**. **"Not a constraint" was measured without the robot and is wrong.** With the robot and laser, memory grows without bound and no ceiling is enough |
| "The laser keeps up" is the open question | it froze the sim in the park — the **terrain `<heightmap>` collision**, not the sensor, was the cause. Fixed 2026-07-23 by switching terrain collision to the low-poly trimesh; the laser now runs at ~10 Hz. See "The laser was broken by the terrain heightmap" |
| "Not worth doing: the small trees are 2% of the park" | true against the original 7.56M, **false now** — they are **16% of what is drawn** |
| `models_opt/` is byte-identical to source | **two textures differ**, see the Safety section |

Two smaller ones: benches, lamps, bins and tables total **146,490** triangles,
not ~90,000. And the "20 cm squares, everything above 2 m discarded" costmap
description is real but comes from the loose project-root file, not from the
dataset.

The "photos" and "Docker memory" to-do items are **dead** — both rested on wrong
numbers.

---

## Full triangle census

Measured from the actual mesh files, weighted by how many times each is used.

| What | Count | Each | Original total | Drawn now | Laser now |
|---|---|---|---|---|---|
| Big tree trunk/branches | 23 | 165,380 | 3,803,740 | 460,000 | **919,402** |
| Big tree leaves | 23 | 35,584 | 818,432 | **0** | **0** |
| Ground | 1 | 2,097,152 | 2,097,152 | 131,072 | **131,072** |
| The path | 1 | 565,088 | 565,088 | **34,380** | 565,088 |
| Benches | 16 | 8,204 | 131,264 | **32,000** | 131,264 |
| Small trees | 15 | 8,455 | 126,825 | 126,825 | 126,825 |
| Bins | 11 | 1,166 | 12,826 | 12,826 | 12,826 |
| Tables | 11 | 120 | 1,320 | 1,320 | 1,320 |
| Lamps | 15 | 72 | 1,080 | 1,080 | 1,080 |
| **Total** | | | **7,557,727** | **799,503** | **1,888,877** |

The ground's "Laser now" is 131,072 as of 2026-07-23: its collision was switched
from a `<heightmap>` (which counts as no triangles, but which the CPU ray sensor
could not cast against — see "The laser was broken by the terrain heightmap") to
the 131,072-triangle low-poly terrain mesh, the same one already used for the
visual. That switch is what raised the laser total from 1,757,805 to 1,888,877.
The original pre-optimization terrain collision was the full 2,097,152-triangle
`terreno_parque.dae`.

The big trees now use three different bark meshes: `bark8.obj` (165,380, the
untouched original, no longer referenced by `park.world`), `bark8_lowpoly.obj`
(20,000, drawn) and `bark8_collision.obj` (39,974, what the laser measures).

The small trees split into `copa4.dae` (canopy, 6,970 each, 104,550 total) and
`tronco4.dae` (trunk, 1,485 each, 22,275 total).

Bark triangle sizes, measured: 5th percentile 12.8 mm, median 32.4 mm, 95th
percentile 63.7 mm, largest 133 mm. Total bark surface 105 m² on a 14.0 m tree.

Bark triangles by height: **97.3% sit above 4 m.** Only 3,792 triangles (2.3%)
are below 2 m, which is all the robot and the obstacle map ever care about.

---

## What remains

**1. Fix the laser — DONE 2026-07-23.** The laser now runs in the park at ~10 Hz
with stable memory. The root cause was the terrain's `<heightmap>` collision,
which the CPU ray sensor cannot cast against; fixed by switching the terrain
collision to the low-poly trimesh. See "The laser was broken by the terrain
heightmap". This unblocks the dataset: the bag's `/os0_cloud_node/points` holds
2,535 scans recorded **live** during the drive (unlike the cameras, a second
pass), and there is no path where laser data comes from anything but a running
sensor — which now works. Still open: record the sim laser to a bag and compare
its scan density and geometry against `park_1.bag`, and drive the robot.

**2. Look at it — DONE 2026-07-21.** Confirmed by eye in noVNC. The tree bark is
textured, not flat grey; the path renders correctly as a distinct textured
strip; the benches show correct wood-grain texture, which also settles the
missing `set="0"` worry from Change 4 — it binds fine. Trees are bare, as
designed. Real time factor 0.99 with the empty park.

**3. Small tree canopies — 104,550 triangles, now the third largest item drawn.**
Investigated and deliberately deferred. `copa4.dae` is **697 completely separate
leaf clusters of exactly 10 triangles each**. Decimation cannot merge across
islands, so realistically 2–3x is available, not the 16x the path gave; the other
lever is deleting whole clusters as was done for the big trees. Judged not worth
mangling the leaves for a partial win.

**4. Not worth doing, measured:**

- **Small tree trunks.** `tronco4.dae` is an open, non-watertight shell — 700 of
  its 2,512 edges are boundary edges, 20 disconnected components, non-manifold.
  With boundary preservation on it **refuses to go below 1,019 triangles**. The
  one reachable target buys 1,485 → 1,099 for a **6.1 mm** mean deviation, which
  is 86% of a pixel at 5 m. Across 15 trees that is 5,790 triangles, 0.6% of the
  scene. Turning boundary preservation off reaches 185 triangles but triples the
  mean to 18.1 mm and eats the shell rims — the trunk's silhouette against the
  sky.
- **Bins** decimate cleanly (580 triangles at 0.92 mm mean) but are only 6,446
  triangles, 0.7% of the scene — and pymeshlab **deletes their material
  entirely**, since their whole appearance is a colour-only Lambert with no
  texture.
- **Tables and lamps** are already 120 and 72 triangles each. Together 2,400
  triangles, 0.27% of the scene. Nothing to win.

**5. Dead disk weight, no downside.**

- `models_opt/tree_8/uploads_files_2812146_BCY_JA02_AcerNikoense_8.blend` —
  **322 MB**, never opened by Gazebo
- `models_opt/tree_8_v/` — **874 MB**, byte-identical copy of `tree_8/`, used only
  by the lake world

The single biggest remaining item is the big tree trunks at 460,000 drawn, 58%
of the scene. Halving them to 10,000 each would save 230,000 triangles — more
than every small object combined — at the cost of visibly more thin twigs (see
Change 2). Not done; the 20,000 decision stands.

Nothing here is urgent. It already runs at full speed.

---

## How to run it

Docker build context lives at `~/husky-docker/` on the internal SSD, **not** in
the project folder.

### Starting up

**Do not use `docker compose up -d`.** This file previously recommended it; that
was wrong. Measured on 2026-07-21 with `--dry-run`, it reports
`Container husky-docker-husky-1 Recreate` — it rebuilds the image and **destroys
the container, taking `/root/catkin_ws` with it** (where the Ouster plugins are
built from source), and risks the stale mount entry that only a Docker Desktop
restart clears.

Use `start` instead:

```bash
cd ~/husky-docker
docker compose start
```

**Expect the display to be broken on the first start after a stop.** `stop`
preserves the container's filesystem, including `/tmp`, so Xvfb finds its own
stale lock from the previous run and aborts with
`(EE) Server is already active for display 1`. Because `entrypoint.sh:9`
backgrounds Xvfb with `&`, its `set -e` never fires and the script carries on —
so `fluxbox` and `x11vnc` fail too, while `websockify` and `roscore` start
normally. The symptom is noVNC loading the page and then showing
**"Failed to connect to server"**.

The fix, in this order:

```bash
docker compose exec -T husky rm -f /tmp/.X1-lock /tmp/.X11-unix/X1
docker compose restart
```

The container must be running before the lock can be deleted, and `restart`
re-runs the entrypoint so Xvfb, fluxbox, x11vnc and roscore all come up
properly. `restart` does **not** recreate the container.

Verify before continuing — `ss` and `netstat` are not installed, so probe the
port directly:

```bash
docker compose exec -T husky bash -lc 'exec 3<>/dev/tcp/127.0.0.1/5900 && head -c 12 <&3'
```

A healthy server answers `RFB 003.008`.

### Launching the park

Wait for noVNC at <http://localhost:6080/vnc.html>, then:

```bash
docker compose exec husky bash -lc \
  "source /workspace/park-env-opt.sh && roslaunch natural_environments create_park.launch"
```

**`start-sim.sh` does NOT launch the park.** It launches
`husky_gazebo husky_playpen.launch`. Use the manual command above for the park.

To run the **unmodified** original park instead, source `park-env.sh` rather than
`park-env-opt.sh`.

Expect a harmless Gazebo mesh error about the `Untitled2` model — see
`park_world_notes.md`.

### Shutting down

Stop the simulation cleanly first, then the container:

```bash
docker compose exec -T husky pkill -INT -f "roslaunch natural_environments"
cd ~/husky-docker && docker compose stop
```

Gazebo takes about 60 seconds to exit after the SIGINT.

**Use `stop`, never `down`.** `down` destroys the container, which takes
`/root/catkin_ws` with it, and leaves a stale mount entry in the Docker daemon
that only a Docker Desktop restart clears. `stop-sim.sh` ends with a full `down`
and therefore walks into this trap.
