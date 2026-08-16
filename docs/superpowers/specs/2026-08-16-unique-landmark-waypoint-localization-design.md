# Unique-landmark + waypoint localization

**Date:** 2026-08-16
**Branch:** `feat/unique-landmark-waypoint-loc`
**Status:** design, REVISED 2026-08-16 to per-region descriptors (see "Revision"
below), pending review

## Revision — from per-object to per-region descriptors

The first version of this spec identified distinctive *objects* by shape. That
broke on contact with the real map: the distinctive structures we added are
several byte-identical instances of one mesh, so their per-object descriptors are
distance 0 apart and none is distinctive — the mechanism had nothing to grip.
More fundamentally, "distinctive object" still smuggled in the idea of segmenting
and implicitly typing objects, which this project exists to avoid.

The revision makes the unit a **region of space at a chosen scale**, described
with no notion of object or type. A single distinctive structure and a
distinctive *arrangement* of ordinary structures become the same measurement at
different window sizes. This is the design below. Sections still describing
per-object work are marked; the descriptor and extractor need extending to
windows, which is where the remaining implementation goes.

## Revision 2 — waypoints decoupled from landmarks (2026-08-16)

The first two versions treated a waypoint as a *landmark to verify*: the operator
declared which distinctive region to expect at each point, and arrival was
confirmed only when the robot perceived it. That was wrong on two counts.

**It deadlocked.** The operator names places from `park_objects.yaml`
(`postescable_1`); the map identifies distinctive regions by grid id
(`loc_080`). Disjoint vocabularies, so the name check could never pass — and
because a route always published a non-empty expectation, the proximity fallback
was unreachable. Every route would stall at its first waypoint.

**It was the wrong model.** In real unmanned-ground-vehicle practice the operator
sends *coordinate* waypoints; landmark/map-relative localization is a separate
layer underneath whose job is surviving GPS denial; and pre-placed beacons are a
third, distinct mechanism. Conflating the mission layer with the localization
layer is not how fielded systems work.

Revision 2 separates them completely: **waypoints are bare coordinates; landmark
matching is the GPS-denied localization layer; they never exchange information.**
The distinctive regions are best understood as *natural* check-in points —
serving the role a placed beacon would, but discovered in the existing scene.

This also **strengthens** the anti-spoofing story rather than weakening it: with
waypoints removed from the anchor path, operator intent can no longer influence
the pose estimate at all. Only measurements do.

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

This design replaces the type-based identification with **distinctive-region
descriptors** — computed with no classification and no notion of "object" — and
replaces the never-refreshed anchor with **re-anchoring on every distinctive-
region match**, so the accumulated odometry error is reset each time the robot
passes a recognisable place.

## Approach

Two independent layers: a mission layer (coordinate waypoints) and a
localization layer (region matching). They do not exchange information — see
"The two layers are independent" below.

**Robot side — region descriptors, no classification, no object types.** The
robot describes the point cloud around a location — a *spatial region*, not a
segmented object — with an NDT-style descriptor (see below), and matches it
against a descriptor map built offline from the world file. It never asks "is
this a bench?" and never even asks "is this one object or several?" It asks
"have I seen *this configuration of space* in the map, and only there?" Regions
whose descriptor has no near twin anywhere in the map are **distinctive anchors**
and pin the pose on their own. Everything else is ignored.

**Why region, not object (the design's core decision).** Identifying a
*distinctive object* and identifying a *distinctive arrangement of ordinary
objects* are the same operation at different spatial scales. A single oddly
shaped structure is distinctive in a small window; a spot surrounded by a rare
configuration of common structures is distinctive in a large window. By making
the descriptor summarise a **region at a chosen scale** — rather than a
segmented object — both cases fall out of one measurement, and the system never
needs to know what any structure physically *is*.

This directly resolves the identical-instance problem. If a map contains several
byte-identical structures, their *object* descriptors are distance 0 apart and
none is distinctive — correct, and unhelpful. But each sits among different
neighbours at different spacings, so at a window scale wide enough to include
that neighbourhood, the *region* descriptors differ and distinctiveness
re-emerges. The scale is the knob; the algorithm and the measurement are
unchanged. Distinctiveness is never assumed for any structure — it is measured,
at whatever scale is chosen, over descriptors that carry no labels.

**Operator side — coordinate waypoints.** The operator sends a route of bare
map-frame `(x, y)` points. The robot drives them in order via `move_base` and
advances when its own position estimate reaches each point. Waypoints are
**navigation targets only** — they are not landmarks, they do not name
landmarks, and they never feed the pose estimate.

### The two layers are independent (revised 2026-08-16, see Revision 2)

This mirrors how fielded unmanned ground vehicles actually work, and it is the
decisive simplification of this design:

- **Waypoints are the mission**: where to drive, as coordinates, exactly as an
  operator drops points on a map.
- **Landmark localization is the GPS-denied layer underneath**: it keeps the
  position estimate honest while the robot drives, opportunistically, whenever
  the route happens to pass a distinctive region. It has no opinion about which
  waypoint is active or whether the robot has "arrived".

The two never exchange information at runtime. That is deliberate. An operator
asserting "drive to (30, -8)" is a statement of *intent*, not evidence the robot
*is* there — so operator input must never enter the pose estimate. Only region
matches (and the one-time GPS bootstrap) set the anchor. Keeping intent out of
the estimate is the correct anti-spoofing posture: a spoofed GPS, a drifting
odometer, and a mistaken operator all fail independently rather than
contaminating one another.

Framed against real practice, the distinctive regions this design discovers are
best understood as **natural check-in points** — the role a pre-placed beacon or
fiducial plays in a real deployment, except found in the existing scene rather
than installed as hardware. That is what makes them usable in terrain where
placing hardware is not an option.

### Why regions, not per-object matching

An earlier version of this design matched *per-object* descriptors and marked
individual objects distinctive. That fails on this map, and the failure is
instructive. The park's repeated structures are byte-identical instances of a
few meshes, so their per-object descriptors are distance 0 apart: none is
distinctive, correctly, and the added power-line structures are *also* repeated
instances of one mesh, so per-object matching cannot tell one from another
either. Per-object distinctiveness simply has nothing to grip on here.

The region descriptor grips where the object descriptor cannot. Two identical
structures in different neighbourhoods produce different *region* descriptors as
soon as the window is wide enough to include those neighbourhoods. So the
question asked is never "what type is this object" and never even "is this one
object" — it is "is the configuration of space at this location one that occurs
nowhere else in the map." That question is answered by measurement, at a chosen
scale, over label-free descriptors — and it degrades gracefully: where a single
structure is already distinctive, a small window suffices; where nothing is
distinctive alone, a larger window finds distinctiveness in the arrangement, or
correctly reports that this location is ambiguous.

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

These structures exist to give the map *something with a distinctive local
region* — they are tall and open-latticed, unlike anything else in the scene, so
a window centred near one of them has a region descriptor with no near twin. But
note the design does **not** treat "a pole" as the landmark unit: the distinctive
thing is the *region of space* around a pole, which the extractor discovers by
measuring distinctiveness over a location grid (below), not by declaring poles
special. One `linea1` model is two poles 28.8 m apart joined by a 59 m cable span
(`extract_lake_map.py:76`, `POLE_OFFSETS`); those poles are simply more objects
in the scene, and the location grid will find whichever windows around them turn
out to be distinctive.

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
describe the local structure in that voxel, with no reference to what object the
voxel belongs to:

| Eigenvalue pattern | Local structure |
|---|---|
| λ₁ ≫ λ₂ ≈ λ₃ | **linear** — points strung along one axis (a thin member) |
| λ₁ ≈ λ₂ ≫ λ₃ | **planar** — points spread on a surface (a sheet) |
| λ₁ ≈ λ₂ ≈ λ₃ | **volumetric** — points filling space isotropically (a blob) |

These are three numbers per voxel. They are properties of the points, not of any
category — the descriptor never names what produced them.

### The descriptor proper — a region, at a chosen scale

The descriptor summarises **all points within a spatial window** of a chosen
radius `R` around a location — not a segmented object. It has two parts, and the
second is what makes the region abstraction work:

1. **Vertical structure** (as before): split the window's height range into
   fixed z-bands; per band record the mean voxel shape-class distribution and
   the horizontal extent. This captures "what kind of structure exists at each
   height," which is view-stable.

2. **Horizontal arrangement**: split the window into an angular/radial grid of
   sectors around its centre, and per sector record how much occupied structure
   it contains and at what shape class. This captures "what sits *around* this
   location, and in which direction" — the signal that distinguishes two
   identical structures sitting in different neighbourhoods.

Part 1 alone makes a single distinctive structure distinctive. Part 2 is what
lets a location surrounded by a rare configuration of *ordinary, repeated*
structures also be distinctive — the same descriptor, now reading arrangement
rather than only local shape. Neither part references an object type; both are
statistics over the raw points in the window.

The window radius `R` is the **scale knob**. Small `R` describes essentially one
structure (the single-distinctive-object case). Large `R` takes in neighbours
(the distinctive-arrangement case). One descriptor, one measurement; the scale
chosen at extraction time decides which regime applies, and the same `R` is used
on the map side and the runtime side so the two remain comparable.

### Why this form

**Eigenvalue ratios survive partial views.** Lidar sees only the near face of
any structure, so *global* measurements like width are systematically understated
and vary with approach angle. Eigenvalue **ratios** are a local property: a voxel
on the near face reads the same whether or not the far side is visible. This is
the property the whole match rests on.

**A solid volume and an open lattice are separable** where a bare extent cannot
tell them apart: both may be 1.2 m across, but one fills its voxels
volumetrically and the other produces many linear voxels in differing
orientations. The descriptor sees that difference without knowing either is an
object of any kind.

**It computes identically from a mesh and from a point cloud.** The map is built
offline from `park.world` meshes; the robot measures live returns. "What is the
structure of the points in this window?" is answerable from either, which is what
puts map and observation in the same space. See the mesh-sampling caveat below.

**Height remains the frame.** Where the crossarm band sits does not change with
viewing angle, so banding by height keeps the descriptor's structure stable
while the per-band statistics describe what the material actually is.

### Distinctiveness

Measured offline over a grid of candidate locations (not over "objects"). At
each location, compute the region descriptor at the chosen scale `R`; the
distance from that descriptor to its nearest neighbour among all other locations
is the location's distinctiveness. Locations whose nearest match is far away —
above a threshold placed in the empty gap the measured distribution reveals, not
guessed in advance — are **distinctive anchors**. This is a measurement, not an
assumption: a location surrounded by an unremarkable, repeated configuration
scores low and is excluded; a location whose neighbourhood occurs nowhere else
scores high. Nothing here knows what physical structures produced either
outcome.

### Caveats this introduces

- **Voxel size is a real tuning knob.** Too large and everything reads
  volumetric; too small and there are too few points per voxel for a stable
  covariance (~5+ needed). At 15 m the Ouster's returns are sparse enough that a
  distant, sparsely-sampled window may not fill enough voxels. This trades `classify.py`'s many
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
- proposed: **a distinctive-region match that passed the match gate**, replacing
  the anchor each time one lands. The one-time GPS fix remains only as the
  bootstrap that seeds the first anchor before any region has been matched.

  Waypoints do **not** set the anchor. An earlier draft let a "confirmed
  waypoint arrival" re-anchor the prior; that was dropped when waypoints became
  bare coordinates (Revision 2). A waypoint is the operator's *intent*, not a
  measurement of where the robot is, and intent must never enter the pose
  estimate — otherwise a mistaken or spoofed instruction silently becomes
  "truth". Region matches are measurements; they alone move the anchor.

**Fix.** A confident region match pins the pose directly. The matched map
location has a known world position, and the descriptor's own centre gives the
robot's offset from it, so one match is already a position — no multi-object
constellation required. Because the map stores each distinctive location's
world coordinates, resolving *which* location was matched is a nearest-descriptor
lookup gated by the robot's prior (so two far-apart locations with similar
descriptors cannot be confused).

**Between anchors.** Non-distinctive regions are ignored entirely. The robot
coasts on anchor + odometry until the next distinctive-region match or the next
waypoint arrival.

**Accepted tradeoff, stated explicitly:** pose quality degrades with odometry
drift between region matches, bounded by distinctive-location spacing (median ~13 m) and
waypoint frequency. This is the cost of not rebuilding constellation matching in
descriptor space, and it is the reason the waypoint half of the design is not
optional.

## Waypoints

Waypoints are **bare map-frame `(x, y)` coordinates** — a route the operator
lays down, exactly as an operator drops points on a map in a real deployment.
They are navigation targets on the existing `move_base` path and nothing more.

`route x1 y1 x2 y2 ...` queues the points; the robot drives them in order and
advances to the next when its own position estimate reaches the current one.
Single-goal commands (`goal`, `goal xy`, `goal <name>`) are unchanged; `route`
is additive.

**Waypoints do not interact with localization.** They do not name landmarks,
they do not declare what the robot should perceive, and they never set the
anchor. The region localizer runs underneath, correcting the pose estimate
whenever the route happens to carry the robot through a distinctive region, and
is entirely unaware of which waypoint is active.

An earlier draft coupled the two: a waypoint declared which distinctive region
to expect, and arrival was confirmed only when the robot perceived it. That was
over-engineering — it conflated the mission layer with the localization layer,
and in practice it deadlocked (the operator's names and the map's region ids
were disjoint vocabularies, so arrival could never confirm and a route stalled
at its first point). Revision 2 removes the coupling.

**What arrival means now.** The robot reports arrival when its own
landmark-corrected position estimate reaches the waypoint. Between region
matches that estimate is dead-reckoned, so arrival carries the accumulated
drift — the same honest limitation any fielded vehicle has when it reports
"waypoint reached" from its own navigation solution.

**The anti-spoofing property is preserved, and is in fact cleaner.** It does not
live in an arrival handshake; it lives in the anchor rule: only region matches
move the anchor. A navsat attack can corrupt the fused pose, and a mistaken
operator can send the robot to the wrong coordinate, but neither can move the
descriptor-derived anchor, because neither is ever consulted for it.

**Fault signal.** Sustained disagreement between the dead-reckoned prior and the
descriptor-derived position is published on `/landmark_fault` for the operator to
see. It is never silently averaged into the estimate.

## Components

| Unit | Responsibility |
|---|---|
| `map_tools/park_types.py` | registry entry for the `linea1` model so its geometry is placed in the scene (done). No type is privileged in matching — the entry only puts the structure on the map. |
| descriptor module (`landmark_loc/descriptor.py`) | points-in-a-window → voxel Gaussians → per-band shape statistics **plus horizontal-arrangement statistics**. Shared by extraction and runtime so the two cannot drift. Built as per-object; **must be extended to per-region windows** (the pivot). |
| mesh surface sampler (`map_tools/mesh_sample.py`) | samples points across mesh faces so the map side sees a laser-like point set rather than modeller-placed vertices (done). |
| `.obj` triangle reader (new, `map_tools/`) | minimal Wavefront reader (v/f lines only, no materials) so `.obj` assets (e.g. `tree_8`) can be sampled too |
| scene-point assembler (new) | builds one combined map-frame point cloud from all placed meshes, so a region window can be cut from it at any location — the per-region analogue of today's per-object sampling |
| region extractor (`map_tools/extract_park_map.py`) | over a grid of candidate locations, cut each window, describe it, score distinctiveness, emit the descriptor map with each distinctive location's world coordinates |
| distinctiveness (`landmark_loc/distinctiveness.py`) | nearest-neighbour scoring over the descriptor map; threshold placed in the measured gap (done; operates on any `{name: descriptor}` map) |
| detector plugin (new) | registers in the existing `DETECTORS` table (`detector.py:225`); describes the region around the robot and matches it to distinctive map locations only, gated by the prior |
| `landmark_loc/localizer_node.py` | anchor source becomes region-match / confirmed-waypoint driven rather than one-shot GPS |
| `operator/operate.py` | `route` over bare (x,y) coordinates; advance on reaching each point. NO landmark coupling (Revision 2). |

`classify.py` and `constellation.py` are **not deleted**. They stay selectable
via `~classifier` and `~matcher` and remain the documented demo path until the
descriptor path is proven in-sim.

## Testing

Unit-testable without a simulator, following the existing `landmark_loc/tests`
pattern:

- voxel shape classification returns *linear* for a synthetic stick, *planar*
  for a sheet, *volumetric* for an isotropic blob (done)
- a region window containing a distinctive structure separates from a window
  over ordinary repeated structure by a wide margin
- **two windows over identical structures in different neighbourhoods are
  distinguished by the horizontal-arrangement part** — the property the region
  pivot exists to provide; a window over an identical structure in an *identical*
  neighbourhood is correctly not distinguished
- the same region descriptor computed from mesh-sampled points and from a
  *partial* (single-face, decimated) point set of the same window stays within
  the match threshold
- descriptor is stable under decimation, standing in for range-driven sparsity
- distinctiveness scoring, run over the real extracted location grid, yields a
  distance distribution with a clear gap; the chosen threshold sits in it
- the waypoint queue advances only on reaching the current coordinate, and a
  route of coordinates round-trips through parsing unchanged
- the anchor is moved by a region match and by nothing else (no waypoint path
  into the anchor at all, per Revision 2)

In-sim (run from the main conversation, never from a subagent, per CLAUDE.md):
the distinctive structures visible in the cloud at the stated ranges; a fix
produced from a single region match; the correct map location resolved (not a
far-apart look-alike); drift between anchors bounded as predicted; and the
existing navsat-drift attack unable to move the descriptor-derived pose.

## Risks

1. **Window scale `R` is the new central knob.** Too small and identical
   structures stay indistinguishable (the arrangement never enters the window);
   too large and every window overlaps its neighbours so *nothing* is distinctive
   and matching also needs more of the scene visible at once. `R` must be chosen
   from the measured distinctiveness distribution, and the same `R` must be used
   on both the map and runtime sides. This is the most likely place to need
   tuning and the biggest new risk the pivot introduces.
2. **Runtime windows are partial and off-centre.** The map cuts a clean window
   centred on a location; the robot cuts a window from a live cloud that sees
   only near faces and whose centre is wherever the robot currently thinks it is.
   The arrangement part is more sensitive to a mis-centred window than the
   vertical part is. The prior-gated match and the partial-view test exist to
   bound this, but it is real and must be validated in-sim.
3. **Sparsity at range, and voxel sizing.** A distant, sparsely-sampled window
   may not put enough points in enough voxels for stable covariances (~5+ per
   voxel). Voxel size and minimum point count are knobs; the decimation test
   bounds them.
4. **Cable returns.** `linea1`'s geometry includes catenary cables spanning
   59 m. Those points fall inside a wide region window and may distort its
   descriptor. Whether they help (part of the real arrangement) or hurt
   (mis-centre the statistics) is an empirical question for the in-sim task; may
   need a height filter.
5. **Arrival confirmation strictness.** Too strict and the robot never
   re-anchors; too loose and a false anchor injects error. Needs an explicit,
   tested gate.
6. **Coverage gaps.** Parts of the park may have no distinctive region within the
   15 m gate. There the design is pure dead reckoning until the next waypoint.
   The extraction's distinctiveness map reveals exactly where these gaps are.
7. **`gzclient` dependency.** Unchanged: the GPU-ray lidar produces no cloud
   without a live GL context on `:0`.

## Verified facts behind this design

- **The distinctiveness emerged from ordinary furniture, not the added
  structures.** Running the extractor on the real park produced 11 distinctive
  locations, each within ~12 m of a power-line pole by *coincidence of the
  search order* — but their nearest actual objects are garden tables, benches
  and trash bins (e.g. `loc_080` -> garden_table @ 2.8 m, `loc_116` ->
  trash_bin @ 0.9 m). At the 8 m window it is the *arrangement* of ordinary
  repeated furniture that is locally unique, while the poles' immediate
  surroundings are comparatively repetitive.

  Two consequences. First, this is the per-region method working exactly as
  intended: it is type-blind, so it finds distinctiveness wherever it genuinely
  exists rather than where we expected to plant it. Second, **the `linea1`
  structures added in the world change turned out to be unnecessary** — the park
  already contained distinctive arrangements. They are kept (harmless scenery,
  and they do contribute to some regions' arrangement context), but a future
  world would not need them. Do not repeat the add-a-distinctive-object step on
  the assumption that a repetitive map has nothing to find; measure first.

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

### Active re-localization: bias the planner toward distinctive regions

**The single most valuable follow-up.** Today the localizer is *passive*: the
robot gets a position fix only when its route happens to carry it through a
distinctive region. If a route never passes one, the estimate dead-reckons and
drifts with nothing to correct it — the accepted tradeoff recorded under Risks.

Making it *active* means giving the planner knowledge of where the distinctive
regions are, so it can deliberately route through one when the estimate has gone
stale: *"no fix in 40 m, there is a distinctive region 8 m off the path — detour
through it, re-anchor, resume."* The robot would then manage its own localization
confidence rather than hoping the mission takes it somewhere recognisable.

Shape of the work:

- A costmap **cost-bias layer** (not an obstacle layer) seeded from
  `maps/park_regions.yaml`, making cells near distinctive regions cheaper to
  traverse. The planner then prefers them at equal cost, for free.
- A **staleness signal** — time or distance since the last accepted region match
  — that scales the bias: irrelevant right after a fix, strong when drifting.
- Optionally an explicit *re-localize* behaviour that inserts a detour waypoint
  when staleness crosses a threshold.

**Note what this is NOT.** Distinctive regions must never be written into the
costmap as *obstacles*. The lidar already stamps every return into the obstacle
layer directly; a second recognition-driven path would double-stamp what the
sensor already sees and, worse, would stamp it at the wrong place whenever the
pose is off. Obstacle avoidance must trust raw sensor geometry; recognition
belongs in localization and, at most, in planner *preference*.

The correct existing coupling between landmarks and the costmap is already in
place and is indirect: a landmark-corrected pose keeps live sensor returns
landing in the right cells. CLAUDE.md records the failure this prevents — a
drifting pose "dragged the costmap off the static map", so the planner avoided
phantom obstacles and drove into real ones. That is landmarks serving the
costmap the right way round.
