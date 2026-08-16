# Unique-landmark + waypoint localization

**Date:** 2026-08-16
**Branch:** `feat/unique-landmark-waypoint-loc`
**Status:** design, pending review

## Problem

The localizer today identifies landmarks by **type**: `classify.py` collapses a
lidar cluster to one of five words (`bench`, `lamp`, `garden_table`,
`trash_bin_1`, `tree`), and `constellation.py` matches those labels against the
map by geometry. Every bench is interchangeable with every other bench, so no
single sighting can fix the pose — it takes a constellation of at least three
(`constellation.py:33`, `_MIN_INLIERS = 3`).

Two consequences:

1. **The classifier is load-bearing and expensive.** Every threshold in
   `classify.py` (`_TREE_CANOPY_MIN_Z`, `_LAMP_POST_MAX`, the `foot_major`
   bands) is a tuned number that must hold across viewing angle, range and
   occlusion. Extending the system means extending that cascade.
2. **The absolute anchor is captured once and never refreshed.**
   `localizer_node.py:333-353` latches a single GPS-converged map pose at
   startup; from then on the prior is that anchor plus accumulated odometry.
   CLAUDE.md records what this costs: a latched transform went 2.7 m, then
   7.7 m, then 13.5 m stale over one run.

This design replaces the type-based identification with **shape descriptors on
distinctive objects**, and replaces the one-time anchor with **operator
waypoints that re-anchor on arrival**.

## Approach

Two independent information sources that cross-check each other.

**Robot side — descriptors, no classification.** The robot computes a
shape descriptor per cluster (NDT-style voxel statistics over height bands, see
below) and matches it against a descriptor
map built offline from the world file. It never asks "is this a bench?" It asks
"have I seen this exact shape somewhere in the map, and only there?" Clusters
whose descriptor has no near twin anywhere in the map are **unique anchors** and
pin the pose on their own. Everything else is ignored.

**Operator side — waypoints.** The operator sends waypoints as navigation
targets. On arrival, a waypoint becomes the new anchor for the dead-reckoning
prior, replacing the one-time GPS anchor.

Neither source derives from the other, so disagreement between them is
meaningful: it means GPS is spoofed, odometry has drifted, or the robot is not
where it was sent. That is the anti-spoofing property the existing attack demos
(`attack_navsat.py`, `mode landmark`) are built around.

### Why not descriptor-matching on ordinary objects

Considered and rejected. The park's 76 objects are instances of **five identical
meshes**, so every bench's descriptor is not merely similar but *identical*.
Descriptor similarity would buy exactly what the type label already buys, minus
the ability to hash into buckets the way `_cat_pair_index` does today — a slower
reimplementation of type matching with thresholds to tune.

Worse, a descriptor able to separate a bench from a garden table from a bin,
from partial single-face views at varying range, is *the same hard problem the
classifier already solves*, re-solved in a new representation. Since the point
of this work is to stop depending on that classifier, taking it on again in
another form defeats the purpose.

Restricting the mechanism to genuinely distinctive objects avoids all of it. The
only question asked of a cluster is "is this the distinctive thing, or not?",
and for a 16.5 m pylon against a 3.15 m lamp that is a five-fold margin on a
single number.

## World change: power-line corridors

The park as it stands contains nothing distinctive — five repeated meshes,
nothing over 3.15 m except trees. A survey of every model available locally
found exactly one suitable structure.

**`linea1`** (`models_lake_opt/linea1/`) — a high-voltage transmission span,
steel pylons plus catenary cables. At the world scale of 0.03 used in
`lake.world` it is 59 m long, 3.6 m deep and **16.5 m tall**. Nothing else in
the park comes within 13 m of that height. Its vertical profile is strongly
bimodal: thin legs from 0–12 m, then a wide crossarm band from 12–16.5 m. That
is precisely what the descriptor below separates best — and its legs are an open
steel lattice, which produces a local-shape signature no solid object shares.

Rejected alternatives: `dumpster` (4.0 × 2.0 × 1.68 m) and `jersey_barrier`
(4.07 × 0.81 × 1.14 m) from `~/.gazebo/models/` — both sit in the same height
band as a bench, so they discriminate only by length. No building, tower,
gazebo, silo, statue or container exists anywhere on this machine; obtaining one
would mean fetching from Gazebo Fuel.

### Layout

The landmark unit is the **individual pole**, not the span. One `linea1` model
is *two* poles 28.8 m apart joined by a 59 m cable span (measured from
`linea1/postes.dae` at world scale 0.03, and already handled this way by
`extract_lake_map.py:76`, `POLE_OFFSETS`). The robot only ever sees one pole or
a stretch of cable at a time, so a pole — not the model, not the span — is the
anchor. This follows the precedent of commit 7091639, *"stamp the power line's
real poles, not its model origin"*, and the existing lake extractor's
`_expand_poles`.

Placement searched model link poses and bearings over the existing object map,
maximising the worst *pole's* clearance. The park is uniformly cluttered — the
best achievable worst-case clearance is 4.7 m, so these are sited, not swept.

**Three models → six poles**, in two corridors. Model link poses (the values
that go in the world file), each with its two derived pole positions:

| Corridor | Model link pose (x, y, yaw) | Derived pole (x, y) | Clearance |
|---|---|---|---|
| **A** — north band, yaw 0.2094 rad (12°) | (-16.330, 5.284, 0.2094) | (-42.50, 1.00) | 4.8 m |
| | | (-14.33, 6.99) | 5.2 m |
| | (40.012, 17.259, 0.2094) | (13.84, 12.98) | 4.7 m |
| | | (42.01, 18.96) | 5.3 m |
| **B** — south edge, yaw 0.0 rad | (-0.511, -24.251, 0.0000) | (-27.00, -23.00) | 5.6 m |
| | | (1.80, -23.00) | 4.9 m |

The two corridor-A models sit end-to-end (link poses 57.6 m apart along the
12° line) so their four poles form one continuous line across the open northern
band; corridor B runs one model along the southern boundary. Both lines are
straight, regularly spaced, and clear of tree stands — a plausible utility
layout.

Coverage over the park's ~98 × 49 m extent: median distance to the nearest pole
**12.9 m**, and **66% of the park lies within the localizer's 15 m gate**
(`max_range`) of at least one pole.

`linea1` lives in `models_lake_opt`, so `load-park-world.sh:240` must add that
directory to `GAZEBO_MODEL_PATH` for the park world, or the model must be copied
into `models_opt`.

## Descriptor: NDT-style shape statistics over height bands

The descriptor borrows the *representation* step from NDT (Normal Distributions
Transform) but not its registration step. NDT voxelizes a cloud and replaces the
points in each voxel with a Gaussian — a mean and a 3×3 covariance — then
registers scans by maximising likelihood under that field. Here the Gaussians
are used only to **characterise local shape**, never to match scan-to-scan.

### Per-voxel shape from the covariance

Decompose each voxel's covariance into eigenvalues λ₁ ≥ λ₂ ≥ λ₃ — the extent of
the local point distribution along its three principal axes. Their *ratios*
classify the material in that voxel:

| Eigenvalue pattern | Class | What produces it here |
|---|---|---|
| λ₁ ≫ λ₂ ≈ λ₃ | **linear** — a stick | pylon members, lamp post, tree trunk |
| λ₁ ≈ λ₂ ≫ λ₃ | **planar** — a sheet | bench seat, table top, ground |
| λ₁ ≈ λ₂ ≈ λ₃ | **volumetric** — a blob | foliage, bushes |

### The descriptor proper

Split the vertical range into fixed-width z-bands. For each band record:

1. the **distribution of voxel shape classes** in that band (primary signal), and
2. the horizontal extent of the band (secondary — coarse size).

A pylon reads as: dense **linear** voxels in many differing orientations from 0
to 12 m, a wide band of linear voxels at 12–16.5 m, nothing above. A bench: a
few **planar** voxels below 1 m, nothing above. Those cannot collide under any
viewing angle.

Note this is a strictly stronger signal than extent alone. Horizontal extent
cannot distinguish a 1.2 m solid slab from a 1.2 m open lattice — both measure
1.2 m across. The pylon's legs are a criss-crossing steel lattice, so they
produce *many linear voxels in differing orientations*, which no other object in
the park does. A tree trunk is linear too, but it is one vertical stick, not a
lattice.

### Why this form

**It survives partial views.** This is the decisive property. Lidar sees only an
object's near face, so *global* measurements like width are systematically
understated and vary with approach angle. Eigenvalue **ratios** are a local
property: a voxel on the near face of a steel member is linear whether or not
the far side is visible. This directly addresses what would otherwise be the
design's largest risk.

**It computes from both a mesh and a point cloud.** The map is built offline
from `park.world` meshes; the robot measures live returns. "What shape is the
material in this voxel?" is answerable from either, which is what puts map and
observation in the same space. See the mesh-sampling caveat below.

**Height remains the frame.** Where the crossarm band sits does not change with
viewing angle, so banding by height keeps the descriptor's structure stable
while the per-band statistics describe what the material actually is.

### Distinctiveness

Measured offline: for each map object, the distance from its descriptor to its
nearest neighbour's descriptor. Objects whose nearest match is far away are
unique anchors. This is a *measurement*, not an assumption — if a future world
contains two identical structures, they will correctly score as non-unique and
be excluded.

### Caveats this introduces

- **Voxel size is a real tuning knob.** Too large and everything reads
  volumetric; too small and there are too few points per voxel for a stable
  covariance (~5+ needed). At 15 m the Ouster's returns are sparse enough that a
  distant pylon may not fill enough voxels. This trades `classify.py`'s many
  thresholds for two knobs — voxel size and minimum points per voxel — which is
  fewer, but not none.
- **Mesh-side computation must sample the surface, not use raw vertices.** Mesh
  vertices cluster where the modeller added detail, not where a laser would
  strike, so eigenvalues from vertices do not correspond to eigenvalues from
  returns. The extractor must sample points across mesh faces.
- **Point density falls with range**, so any count-sensitive statistic must be
  normalised before comparison.

## Localization flow

**Anchor.** The prior is anchor + odometry displacement since the anchor, with
heading from `/compass/data` — the mechanism `compose_prior`
(`localizer_node.py:56-86`) already implements. The change is *what sets the
anchor*:

- today: a single GPS-converged pose, captured once at startup, never updated
- proposed: whichever of these two events happened **most recently**, since both
  reset the accumulated odometry error:
  - a **confirmed** waypoint arrival (confirmed in the sense below), or
  - a pylon sighting that passed the match gate

  A pylon sighting is the stronger of the two — it is a direct measurement of
  position against a known map object, where a waypoint arrival is an assertion
  corroborated by perception. So when both occur within the same tick, the
  pylon sighting sets the anchor.

**Fix.** A confident pylon match pins the pose directly. No three-inlier
constellation, because a unique object plus a measured range and bearing is
already a position.

**Between anchors.** Non-distinctive clusters are ignored entirely. The robot
coasts on anchor + odometry until the next pylon or the next waypoint arrival.

**Accepted tradeoff, stated explicitly:** pose quality degrades with odometry
drift between pylon sightings, bounded by pylon spacing (median 13.4 m) and
waypoint frequency. This is the cost of not rebuilding constellation matching in
descriptor space, and it is the reason the waypoint half of the design is not
optional.

## Waypoints

Waypoints remain navigation targets on the existing `move_base` path. The new
behaviour is re-anchoring.

**Arrival must be confirmed by descriptor match, never by `move_base` status or
the fused pose.** This is the load-bearing constraint of the whole design. The
project's own memory records that *move_base SUCCEEDED/dist and fused pose lie
about arrival*, and the fused pose is precisely the quantity a navsat attack
controls. Anchoring on an unconfirmed arrival would inject the attacker's error
into the pose belief as though it were truth, and every downstream estimate
would inherit it.

So: on reaching waypoint *k*, the robot anchors on waypoint *k*'s coordinates
only if it independently recognises the shapes it expects to see there.
Otherwise it holds the previous anchor and reports the discrepancy.

**Fault signal.** Knowing that waypoint *k+1* is a known distance and bearing
ahead gives a predicted pose at all times. Sustained disagreement between that
prediction and the descriptor-derived pose is surfaced to the operator as a
fault. It is not silently averaged into the estimate — the operator may force a
reset, but only as an explicit command.

This keeps waypoints as a *position assertion and consistency check*, never as a
pose measurement. The operator asserts intent; intent is not evidence of arrival.

## Components

| Unit | Responsibility |
|---|---|
| `map_tools/park_types.py` | add the `linea1`/pylon type. The dataclass documents (`:59-62`) that fields appended last with defaults are a zero-breakage change. |
| `map_tools/extract_park_map.py` | stamp pylons individually; emit the descriptor map |
| descriptor module (new, `landmark_loc/`) | points → voxel Gaussians → per-band shape statistics. Shared by extraction and runtime so the two cannot drift apart. |
| mesh surface sampler (new, `map_tools/`) | samples points across mesh faces so the map side sees a laser-like point set rather than modeller-placed vertices |
| distinctiveness (new) | offline nearest-neighbour scoring over the descriptor map; marks unique anchors |
| detector plugin (new) | registers in the existing `DETECTORS` table (`detector.py:225`); matches clusters to unique anchors only |
| `landmark_loc/localizer_node.py` | anchor source becomes waypoint/anchor-driven rather than one-shot GPS |
| `operator/operate.py` | waypoint sequence; arrival confirmation; fault reporting |

`classify.py` and `constellation.py` are **not deleted**. They stay selectable
via `~classifier` and `~matcher` and remain the documented demo path until the
descriptor path is proven in-sim.

## Testing

Unit-testable without a simulator, following the existing `landmark_loc/tests`
pattern:

- voxel shape classification returns *linear* for a synthetic stick, *planar*
  for a sheet, *volumetric* for an isotropic blob
- descriptor of a synthetic pylon-shaped point set (a lattice of sticks)
  separates from a bench/lamp/table/bin set by a wide margin
- the same descriptor computed from mesh-sampled points and from a *partial*
  (single-face, decimated) point set of the same object stays within the match
  threshold — the property the eigenvalue-ratio form exists to provide
- descriptor is stable under decimation, standing in for range-driven sparsity
- distinctiveness scoring marks pylons unique and the five repeated families
  non-unique, on the real extracted map
- arrival confirmation rejects a waypoint whose expected shapes are absent
- anchor update leaves the prior unchanged when arrival is unconfirmed

In-sim (run from the main conversation, never from a subagent, per
CLAUDE.md): pylons visible in the cloud at the stated ranges; a fix produced
from a single pylon sighting; drift between anchors bounded as predicted; and
the existing navsat-drift attack unable to move the descriptor-derived pose.

## Risks

1. **Sparsity at range, and voxel sizing.** The eigenvalue-ratio descriptor
   largely defuses the *partial view* problem — local shape is a local property,
   so a near-face voxel classifies correctly without the far side. What remains
   is **density**: a pylon at 15 m may not put enough points in enough voxels to
   yield stable covariances (~5+ points per voxel needed). Voxel size and
   minimum point count are the two knobs, and the decimation test above exists
   to bound them. This is now the most likely place to need tuning.
2. **Cable returns.** `linea1`'s collision geometry includes catenary cables
   spanning 59 m. Those return lidar points belonging to no pylon and may
   pollute clustering. Likely needs an extent or height filter in segmentation.
3. **Arrival confirmation strictness.** Too strict and the robot never
   re-anchors, leaving it on pure odometry; too loose and a false anchor injects
   error. Needs an explicit, tested gate.
4. **Coverage gaps.** 40% of the park is beyond the 15 m gate from any pylon.
   In those regions the design is pure dead reckoning until the next waypoint.
5. **`gzclient` dependency.** Unchanged from today: the GPU-ray lidar produces no
   cloud without a live GL context on `:0`.

## Verified facts behind this design

- **Lidar intensity is unusable.** The Ouster publishes an `intensity` field
  (offset 16, FLOAT32) and `park.world` assigns a distinct `<laser_retro>` per
  family (bench 11, lamp 8, path 7, tree 2…), but the GPU ray path does not
  propagate it. Measured in the running sim on 2026-08-16: all 15,587 finite
  returns in a single message carried `intensity = 0.0`, one distinct value.
  A material/semantic channel via intensity is **not available**, and recovering
  it would mean switching to CPU ray, which the project prohibits.
- **No camera is active.** RealSense (`HUSKY_REALSENSE_ENABLED`, default 0) and
  ZED stereo (commented out, `sensor_description.urdf:9-14`) both exist but are
  off, and nothing in the repo consumes an image topic. Simulated depth is a
  z-buffer readout and does not reproduce real RealSense failure modes
  (sunlight washout, untextured surfaces, holes), so building on it would
  repeat the ground-truth mistake in a subtler form.
- **No thermal sensor exists**, and Gazebo has no thermal sensor type. The
  original "hot reactor" example is not implementable here without a custom
  plugin and per-model temperature properties.
- **`Untitled2` in `park.world:6430` is a broken ghost** — its mesh path points
  at `/home/a/Desktop/...` on the original author's machine, so it has no
  geometry and has been free-falling since startup (z = -1.76e8 m, still
  accelerating). Not a landmark; noise in the world file.

## Future extensions

The descriptor is a vector; adding dimensions does not change the architecture.
A colour statistic from a camera, or a temperature band from a thermal sensor,
would append columns and make distinctiveness scoring strictly sharper. The
design deliberately does not depend on either, because neither sensor is
available and simulated versions of both diverge from hardware in ways that
would not transfer.
