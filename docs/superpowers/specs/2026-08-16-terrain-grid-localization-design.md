# Terrain-aware grid localization

**Date:** 2026-08-16
**Branch:** `feat/unique-landmark-waypoint-loc`
**Status:** DESIGN — pending review. No implementation yet.
**Supersedes/extends:** builds on
`2026-08-16-unique-landmark-waypoint-localization-design.md` (the region-descriptor
line). Keeps that spec's core insight — identity lives in region structure, not in
typed objects — and adds a terrain cue plus a sequence-based fusion, KEEPING the
existing object classification pipeline rather than replacing it.
**Evidence:** `docs/research/lidar-place-recognition-survey.md` (survey), plus live
measurements taken this session in the lake world (cited inline).

---

## 1. Problem

The current localizer (`landmark_loc/localizer_node.py`) segments the cloud into
blobs (`crop → cluster → classify`), matches a constellation of typed centroids,
and publishes a one-shot gated (x, y) fix. Three structural problems, each
confirmed this session:

1. **The front-end is manual and brittle.** ~13 hand-tuned constants (`z_min`,
   `z_max`, `max_range`, `link_dist`, `min_pts`, `max_extent`, `constellation_tol`,
   `dist_gate`, `max_prior_dist`, `residual_gate`, `fov_halfwidth`, `max_jump`,
   `anchor_min_dist`). Every new landmark type forces re-tuning that risks breaking
   the old ones (git history: `z_max`→7.0, `max_extent`→6.0, both "measured per
   tree-landmark spec").

2. **It cannot represent terrain.** Terrain is continuous, underlies everything,
   and runs past sensor range. `crop(z_min=-0.5)` deletes the ground at step one;
   Euclidean clustering cannot blob a continuous surface; no shape template exists
   for a shore or a bank. The shore dip the user asked about is invisible to this
   pipeline *by construction*, not by tuning (survey §0, §6).

3. **One-shot gating cannot cure aliasing.** With 23 identical trees / 16 benches /
   15 lamps, near-tie matches are the normal case. Every gate (`residual`, `_jump_ok`,
   median smoother) is a reject rule on the *final* answer; none can distinguish a
   plausible-but-wrong match from a correct one, because that requires evidence
   across a *sequence*, which a per-scan mechanism does not have (survey §4.3, §6).

**Goal:** (a) represent terrain as a first-class localization cue — the capability
the object pipeline structurally lacks (problem 2); (b) resolve aliasing across
scans (problem 3); while working on BOTH the flat park and terrain-rich maps
without a rewrite between them. Problem 1 (the object pipeline's manual constants)
is acknowledged but NOT solved here — the object pipeline is kept as-is; this design
adds terrain beside it rather than rebuilding it.

---

## 2. Deciding measurements (taken this session, lake world)

These numbers, not argument, drive the design. All are reproducible from the repo.

| Fact | Value | Source |
|---|---|---|
| Park terrain relief | **0.0069 m** (z-scale 0.01, flattened 100×) | `maps/park_dtm.npy` |
| Lake terrain relief | **2.4223 m** (z-scale 4) | `maps/lake_dtm.npy` |
| Relief ratio | **351×** | derived |
| Robot tilt on lake slope (lidar ground plane, base frame) | **pitch 17.46°, roll 2.74°** | live plane fit |
| False height ramp that tilt injects at 20 m | **~6 m** (> 2.4 m real relief) | 20·sin(17.46°) |
| Robot attitude per TF / EKF | **roll 0°, pitch 0°, z 0** (`two_d_mode: true`) | `localization_map.yaml:40` |
| Roll/pitch actually available | **yes**, on `/compass/data` (gravity on Z, 9.78) | live IMU read |
| Sensed ground vs DTM under robot | disagree by **~0.66 m**, position-dependent | live |
| Lake water lidar-visibility | **none** — `lago` has zero `<collision>` | `lake.world` |

**Two conclusions forced by the data:**

- **Terrain and object cues alternate, they do not overlap.** Park: 6.9 mm relief
  → terrain cue is dead, objects carry everything. Lake: 2.42 m relief → terrain is
  the strong cue, fewer catalog objects. "Both maps matter equally" (user) therefore
  makes the hybrid mandatory, not optional.
- **Gravity de-rotation is a prerequisite, not a refinement.** A 17° uncorrected
  tilt injects a false ramp larger than the entire real signal. Any terrain work
  built without it measures the robot, not the world.

---

## 3. Architecture

Add gravity de-rotation and a terrain grid cue; **keep the existing object
pipeline** (crop→cluster→classify→constellation); fuse both cues across scans with
a hypothesis tracker. The grid does NOT replace classification — rasterizing an
object into a height cell discards the very shape the classifier needs. The grid is
a TERRAIN representation. Objects and terrain are two independent front-ends that
share one input: the gravity-de-rotated cloud.

```
                         /compass/data (roll, pitch, yaw)
                                    │
/os0_cloud_node/points ──> de-rotate scan (gravity-align)
                                    │
                 ┌──────────────────┴───────────────────┐
                 │ (shared de-rotated cloud)             │
   crop → cluster → classify                    2.5D terrain grid
   → constellation match vs catalog       (per cell: max_z, min_z, count)
                 │  (Cue A: objects)                     │
                 │                          min_z → morphological ground
                 │                          → gradient-correlate vs DTM
                 │                                        │ (Cue B: terrain)
                 └──────────────────┬───────────────────┘
                        hypothesis tracker (top-K across scans)
                                    │
                     committed (x, y) fix ──> /odometry/landmark_fix ──> map-EKF
```

| Layer | What | Replaces / adds |
|---|---|---|
| De-rotation | rotate cloud by roll/pitch from `/compass/data` | NEW; gates everything below |
| Cue A (objects) | crop→cluster→classify→constellation, on the de-rotated cloud | KEPT ~as-is; input is now de-rotated |
| Cue B (terrain) | 2.5D grid → morphological ground → gradient-correlate vs DTM | NEW; terrain only |
| Fusion | top-K hypothesis tracker across scans | replaces one-shot gating |

The map-EKF is **unchanged**. It still receives an absolute (x, y) on
`odometry/abs_fix` and stays 2D (`two_d_mode: true`). Roll/pitch are consumed
INSIDE the localizer to form the measurement; they never enter the filter. This is
the same pattern the existing localizer already uses for yaw (subscribes to
`/compass/data` directly, `localizer_node.py:478`), so it needs no nav-stack change.

---

## 4. The terrain grid (Cue B front-end)

The grid is a TERRAIN representation, NOT an object detector. A single pass over
the de-rotated cloud bins points into a world-frame, axis-aligned Cartesian grid.
Per cell: `max_z`, `min_z`, `count`. This is what the object pipeline (crop→cluster
→classify) structurally cannot do — represent a continuous surface — and it is why
the grid is added rather than swapped in.

**Cell validity is three-state, never two:**

| State | Meaning | Used by matcher? |
|---|---|---|
| valid | returns received | yes |
| occluded / out-of-range | line of sight blocked or beyond range | no — excluded |
| clear-but-empty | line of sight open, nothing returned | (would be water on hardware — see §7) |

Uncovered cells are **NaN**, never 0.0 — a fabricated zero is a fake flat plateau
that dominates correlation. (The DTM extractor already enforces this;
`map_tools/dtm_raster.py`.)

**Derived channel:**

- `ground = morphological_opening(min_z)` — erode (min-filter) then dilate
  (max-filter) over a window WIDER than the widest object and NARROWER than the
  terrain feature scale (~10 m for this park/lake; objects ≤ 6 m, terrain features
  tens of m). Removes objects that poke up and leaves the local, slope-following
  ground surface — the answer to the user's "how do you define 0.0 on uneven
  terrain": you don't assume one, you compute it from the data. Same principle as
  Patchwork region-wise plane fitting (survey §5), done as a cheap grid filter.

`ground` is the terrain-matching channel (Cue B). Objects are handled separately by
the classifier (Cue A, §5) — the grid does not detect them. The `ground` surface
is, however, ALSO what the deferred traversability layer needs (§9), which is why
the grid is specified as a shared representation even though only the terrain cue
consumes it now.

---

## 5. Cue A — objects (KEPT: classification + constellation)

The existing object pipeline stays: `crop → cluster → classify → constellation
match` (`landmark_loc/segment.py`, `detector.py`, `solve.py`). It is NOT replaced
by the grid — a distinct object's shape is exactly the information the classifier
uses, and rasterizing it into a height cell would discard that. The grid detects no
objects.

**The one change to this pipeline: it consumes the gravity-de-rotated cloud** (§3)
instead of the raw cloud. On a tilted robot the raw `crop(z_min…z_max)` band cuts
the wrong slice of the world (a fixed height band is not level ground on a 17°
slope); de-rotating first makes the existing crop meaningful on terrain. Everything
downstream of the crop — clustering, classification, the typed/typeless
constellation matcher — is unchanged.

This is the always-available cue: on the flat park it carries the entire load. Its
~13 hand-tuned constants remain a known wart (§1), NOT resolved by this design;
addressing them is separate future work, out of scope here.

---

## 6. Cue B — terrain (gradient correlation)

North-aligned by compass (no rotation search), correlate the GRADIENT of the local
`ground` grid against the gradient of the prior DTM. Gradients, not raw heights,
so a constant elevation offset (unknown robot height, sloped ground) cancels
exactly — this is why the ~0.66 m sensed-vs-DTM discrepancy and the missing z-state
do NOT block terrain matching. The correlation peak gives (x, y) directly, in
metres. Restrict the search to a neighbourhood of the odom prior; FFT-correlate.

**Prior = DTM extracted from the world file** (`map_tools/extract_dtm.py`), NOT
built by driving (that would be SLAM, inheriting the drift we are correcting —
honors the "extract, don't build" rule). The DTM is bare terrain; objects are the
separate catalog. On real hardware the prior is a downloadable **DTM** (bare-earth;
NOT a DSM — a ground robot cannot observe the canopy-top surface a DSM records).

**This cue is the strong one on terrain maps and contributes ~nothing on the park.**
Judge it on the lake, never the park (survey §5.1, §8).

---

## 7. Fusion — hypothesis tracker

Replace reject-gates with a top-K pose-hypothesis tracker. Each scan, the matcher
returns the top-K candidate poses (not one). Hypotheses are carried across scans:
each is propagated by odom, and a new scan's candidates either reinforce an
existing hypothesis (agrees within tolerance) or seed a new one. Commit and publish
only when one hypothesis dominates for N consecutive scans.

This is a particle filter with 3–5 particles and no resampling — it buys the
sequence-based disambiguation that cures plausible-but-wrong aliasing (the dominant
failure in a repeated-object scene), without the noise models, weighting, and
degeneracy management of a full PF. It **upgrades cleanly**: when terrain likelihood
maps arrive, hypotheses become particles and the tracker becomes NDT-MCL-style
(survey §4.3).

DECISION (open, see §10): full particle filter now vs. hypothesis tracker first.
Recommendation: tracker first.

---

## 8. Decisions locked this session

- **Both environments matter equally** → hybrid, both cues first-class.
- **Uniqueness = arrangement + one-of-a-kind + terrain shape** (NOT whole-scan
  Scan Context: compass already supplies the yaw its column-shift exists to
  recover, so it buys nothing here — heightmap-gradient dominates it on every axis
  for this setup).
- **Grid is terrain-only; it does NOT replace classification.** crop/cluster/
  classify stays for objects (rasterizing an object discards its shape). Grid and
  object pipeline are two front-ends sharing the de-rotated cloud.
- **Prior DTM, not DSM.** Terrain matched as bare earth; objects as catalog.
- **`two_d_mode` stays `true`.** Roll/pitch consumed in the localizer, not fused.
  Nothing observes z, so relaxing it trades stable-and-wrong for drifting-and-wrong.
- **De-rotation is step one**, independently useful and verifiable.

---

## 9. Traversability — deferred second consumer (scope boundary)

The SAME grid, with slope-per-cell = gradient magnitude of `ground`, is exactly
what a traversability costmap needs. This matters because the lake is **invisible
to obstacle avoidance by construction** (`lago` has no collision; deliberately
absent from `lake_map.pgm`) — so today nothing stops the planner routing the robot
into the water. The localizer would SEE the shore dip and use it; the 2D planner
does NOT avoid it.

To keep that door open WITHOUT building it now, the grid is specified so a
traversability layer can consume it later. Constraints this imposes on the grid,
all of which localization wants anyway EXCEPT the last:

1. world-frame, axis-aligned;
2. resolution compatible with the `move_base` costmap (evenly divisible);
3. separate ground/object channels (slope must come from `ground`, not canopy);
4. gravity de-rotated (a tilted scan makes flat ground read as a cliff);
5. **accumulated (rolling window), not per-scan-discarded** — traversability needs
   slope for terrain about to be driven over, including currently-occluded cells.

Item 5 is the one genuine cost localization would not otherwise pay (a rolling grid
needs bounded memory + drift-aware placement). DECISION (open, see §10).

Building the traversability layer itself (costmap plugin, `move_base` config) is
**out of scope** for this design.

---

## 10. Open decisions for review

1. **Fusion depth:** hypothesis tracker first (recommended) vs. full particle
   filter now.
2. **Grid persistence:** per-scan (simpler, localization-only) vs. accumulated
   rolling window (serves traversability later, costs memory + drift handling).
   This is the one choice that most affects the deferred traversability consumer.
3. **De-rotation as a standalone first step?** It is independently useful (fixes
   the RViz tilt, enables terrain) and cheaply verifiable. Recommend building and
   testing it before the rest.
4. **Build order:** de-rotate → grid front-end → object cue on grid (park-testable)
   → terrain cue (lake-testable) → hypothesis tracker. Confirm.

---

## 11. Risks and honest limits

- **The sim's `/compass/data` is noise-free by construction** (`accelGaussianNoise
  0.00`, `headingDrift 0.00`). Its roll/pitch are perfect; real IMU attitude is
  noisy and needs filtering. A design validated only here will look better than it
  will on hardware. State this; do not tune to the noise-free case.
- **`/os0_cloud_node/imu` and `/compass/data` are not independent** — both read the
  same noise-free body pose, so they are byte-identical. Using the Ouster IMU adds
  no independent information over the compass here.
- **Water null-returns are untestable in this sim.** `lago` has no collision, so the
  "lake reads as a hole in the data" cue (real on hardware) does not exist here.
  In-sim shoreline detection must use terrain gradient alone.
- **The park cannot validate terrain matching.** 6.9 mm relief. Terrain cue will
  look broken while being correct. Judge on the lake.
- **The lake DTM has a ~30% coverage gap** (a diagonal wedge with no terrain,
  inherent to the asset, identical in visual and collision meshes). Those cells are
  "no prior data," distinct from occlusion and water; the matcher must treat them so.
- **Morphological ground window is now load-bearing.** Too large flattens real
  terrain; too small leaves object residue that pollutes the terrain match. Critical
  parameter, not a convenience.
- **Slope, not just height, must be handled on steep faces.** Height-above-ground
  along vertical vs. surface normal differs on a 30° face; matters for tall-object
  detection, not for terrain. Defer until a case demands it.

---

## 12. Components (implementation sketch, not built)

- `landmark_loc/derotate.py` — subscribe `/compass/data`, apply inverse roll/pitch
  to the cloud. Standalone, testable first. Feeds BOTH front-ends.
- `landmark_loc/terrain_grid.py` — 2.5D binning (max_z/min_z/count) + morphological
  ground. TERRAIN only; does not detect objects. Reuses `map_tools` conventions.
- terrain matcher — gradient FFT-correlation of `ground` vs DTM, odom-gated search.
- `map_tools/extract_dtm.py` — DONE this session (prior DTM from world file).
- object pipeline — `segment.py`/`detector.py`/`solve.py` KEPT as-is, input rewired
  to the de-rotated cloud. No new object code.
- hypothesis tracker — top-K carry across scans (both cues), commit on N-scan
  dominance.

Tests: per-module pytest under `landmark_loc/tests/` and `map_tools/tests/`, small
synthetic grids with known geometry, following existing test conventions. Sim
validation follows RUN-MAP-NAV.md in full (park for objects, lake for terrain),
judged by the robot's actual Gazebo position vs the goal — never by fused pose.
