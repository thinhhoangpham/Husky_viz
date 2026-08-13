# Shape-Fit Observations + Bad-Fix Rejection — Design

**Date:** 2026-08-13
**Branch:** `feat/constellation-matcher`
**Status:** design, pending user review

## Problem (measured, this session)

The landmark localizer estimates each observed object's position from the
**centroid of the visible lidar points** pushed out by a round "near-face" radius
(`classify.to_observations`). That model is wrong for elongated objects, and it
produces two failures we measured in-sim:

1. **Inaccurate observation centers.** The lidar sees only the near face of an
   object — for an elongated bench/table that is a near-edge *line* or *L-shape*,
   not the full footprint. Real captures (robot driven up to a bench/table, this
   session):
   - **bench** (true footprint 1.78 × 0.80 m): near broadside view = solid L-block
     (major ~1.6–1.8, minor ~0.8); angled/far view = thin near-edge line (minor
     collapses 0.8 → 0.3, length foreshortens). The visible-points centroid shifts
     by up to **~0.9 m** with viewpoint.
   - **garden_table** (true footprint 3.00 × 1.32 m, an elongated ~2:1 rectangle,
     NOT a circle): seen as a large irregular cluster (top edge + attached
     chairs), major ~2.9, minor ~1.4–1.5. Worst-case centroid error **~1.5 m**.
   - **lamp / trash_bin_1** (compact, aspect ~1.3–1.8): centroid ≈ true center,
     error ~0.3 m — already fine, no change needed.

   These 0.9–1.5 m errors distort the constellation *shape*, so RANSAC either
   fails to form a 3-inlier match (STALE) or forms a **wrong** one.

2. **Bad fixes that teleport the pose.** Observed in RViz during a landmark-mode
   drive: the pose jumped **backward**, and distance-to-goal grew. Confirmed in the
   diag by fixes like `FIX x=17.23 y=-6.21 rms=0.39` and `FIX x=-12.27 y=-4.69` —
   off the route line, yet passing the current rms residual gate (0.5 m). A wrong
   but internally-consistent constellation yields a low-rms fit at a **wrong**
   pose. That fix reaches the map-EKF and yanks the robot's belief backward, which
   is worse than no fix (STALE at least coasts smoothly on odom).

**Catalog positions are correct** (validated: the operator's `goal <name>` drives
the robot to the object, and it stores each object's true center — extracted
offline from `park.world`). The bug is entirely on the **observation** side and in
**accepting wrong fixes**.

## Goal

Two independent, composable fixes:

- **Part 1 — Shape-fit observations.** Estimate each object's true center **and
  yaw** by fitting its **actual footprint** (the real rectangle / circle) to the
  cluster points — no reduction to a point or a line. Add yaw to the observation,
  the catalog, and the matcher (as a frame-invariant pairwise yaw-difference), so
  orientation becomes matching signal.
- **Part 2 — Reject physically-impossible fixes.** Before publishing a landmark
  fix, reject it if it places the robot farther from where it can be (last
  published pose advanced by odom displacement) than the robot could have moved.
  This drops the backward-teleport bad fixes; that tick goes STALE (publishes
  nothing). NOT re-anchoring — a pure output filter, no feedback into the prior.

Trees keep the existing trunk-base position fix (separate, already working).

## Part 1 — Shape-fit observations

### Footprint model per type

Each identifiable type has a known 2D footprint from its mesh (`signatures.py`):

| type | footprint | fit |
|---|---|---|
| bench | rectangle 1.78 × 0.80 | ICP rectangle fit |
| garden_table | rectangle 3.00 × 1.32 | ICP rectangle fit |
| lamp | circle r≈0.24 | centroid + radius push-out (unchanged) |
| trash_bin_1 | circle r≈0.19 | centroid + radius push-out (unchanged) |
| tree | (trunk-base, existing) | unchanged |

Only the **elongated rectangles** (bench, table) get the new fit. Compact/round
types (lamp, bin) keep the current centroid + near-face push-out — measured error
~0.3 m, within the 0.5 m inlier tolerance, so YAGNI on changing them.

### The fit — ICP-style rectangle registration

For a bench/table cluster, register the known rectangle outline to the observed
points, solving for `(center_x, center_y, yaw)`:

1. **Initial guess.** PCA on the cluster's xy points gives the long-axis direction
   → initial `yaw`. Initial center = the visible-points centroid (rough).
2. **Iterate (ICP):** repeat until converged (or a small fixed iteration cap):
   - For each observed point, find the closest point on the **known rectangle
     outline** (the four edges at the current pose).
   - Solve the rigid `(dx, dy, dyaw)` that minimizes the sum of squared
     point-to-outline distances (a small linear least-squares step).
   - Apply it to the rectangle pose; repeat.
3. **Output:** the converged rectangle center = observation `(x, y)`; the
   rectangle long-axis angle = observation `yaw`.
4. **Robustness / fallback:** the known rectangle size fills in the unseen back
   face, so a partial near-edge view still constrains the center in the depth
   direction. If the cluster is too sparse to fit reliably (< a min point count,
   or the fit residual is large / doesn't converge), fall back to the current
   centroid + push-out so we never crash or emit garbage. A thin near-edge-only
   line is the weak case (the rectangle can slide along the edge); the fallback
   and the min-point guard cover it — measured impact assessed in-sim.

This lives in a new focused module (e.g. `landmark_loc/shapefit.py`) so it's
testable in isolation with synthetic point sets; `classify.to_observations` calls
it for bench/table.

### Yaw through the pipeline (frame-invariant pairwise)

- **Observation** gains a `yaw` field (robot-frame object orientation from the
  fit). Round types (lamp/bin) have no meaningful yaw → carry `None`.
- **Catalog** (`MapLandmark`) gains a `yaw` field. The extractor
  (`extract_park_map.py`) already reads each model's world yaw (`m.yaw`); it writes
  a `yaw:` field per entry in `park_places.yaml`; `catalog.load` reads it.
- **Matcher** (`constellation.py`): the pair feature becomes `(distance,
  yaw_diff)` where `yaw_diff = objA.yaw − objB.yaw`. A yaw-*difference* is
  frame-invariant — rotating the whole scene (drift/heading) leaves A-to-B angle
  difference unchanged — so drift-immunity is preserved. Seed/inlier matching
  additionally requires the observed pair's yaw-diff to match the catalog pair's
  yaw-diff within a tolerance. Pairs where either object has `yaw=None`
  (lamp/bin) fall back to distance-only matching (no yaw constraint), so
  round-only constellations still work exactly as today.
- Absolute yaw is **never** compared to the map directly (that would need the
  robot heading and re-introduce heading dependence). Only relative pairwise
  yaw-differences are used.

## Part 2 — Reject physically-impossible fixes

Before publishing a computed fix `(x, y)` in `on_cloud`:

- Maintain `last_pub_xy` (the last published fix) and the odom pose at that time.
- Compute `expected = last_pub_xy + (odom_now − odom_at_last_pub)` — where the
  robot should be now, per odom displacement (odom absolute drifts, but its
  *displacement* over a few seconds is trustworthy).
- If `hypot(fix − expected) > MAX_JUMP` → **reject**: do not publish, log as a
  rejected-jump STALE tick, return.
- `MAX_JUMP` is a few metres (initial ~3 m), larger than real per-tick motion +
  drift but far smaller than the backward teleports (~8–15 m) we saw. Tuned
  in-sim.
- On the **first** fix after startup (no `last_pub_xy` yet) the gate is skipped —
  the bootstrap fix is accepted (it establishes the reference).

This is NOT re-anchoring: the prior's spawn anchor is untouched, the fix is never
fed back into the prior/anchor, and a rejected fix simply isn't published. It only
filters output. The existing wide `_PRIOR_SANITY` (15 m) in the matcher stays;
this adds a tighter, odom-referenced physical-motion gate at the node level.

## What changes

- **New:** `landmark_loc/shapefit.py` (ICP rectangle fit) + its tests.
- **`classify.py`:** `to_observations` calls the shape fit for bench/table (round
  types + trees unchanged); `Observation` gains `yaw`.
- **`catalog.py`:** `MapLandmark` gains `yaw`; `load` reads `yaw:` from yaml.
- **`extract_park_map.py`:** write `yaw:` per place entry (uses existing `m.yaw`).
- **`maps/park_places.yaml`:** regenerate with yaw (worktree catalog).
- **`constellation.py`:** pair feature `(distance, yaw_diff)`; yaw-diff match with
  tolerance; `None`-yaw pairs fall back to distance-only.
- **`localizer_node.py`:** the motion-jump reject in `on_cloud`; track
  `last_pub_xy` + odom-at-last-pub.
- **Unchanged:** compass heading prior, buff_size, mode gate, tree trunk fix,
  solve_pose refit + residual gate, EKF wiring.

## Testing

### Unit
- **shapefit:** synthetic bench/table point sets at several viewpoints (full L,
  near-edge line, angled, partial) → fitted center within a few cm of truth and
  yaw within a few degrees; sparse/degenerate cluster → falls back cleanly.
- **catalog:** loads yaml with `yaw:`; missing yaw → `None` without crashing.
- **constellation:** pairwise yaw-diff is frame-invariant (rotate the whole scene
  → same match); a wrong constellation that matches on distance but NOT yaw-diff is
  rejected; `None`-yaw (lamp/bin) pairs match distance-only as before; drift-
  immunity tests still pass.
- **motion gate (localizer helper):** a fix within MAX_JUMP of expected is
  accepted; a backward-teleport fix beyond MAX_JUMP is rejected; first fix
  (no reference) is accepted.

### In-sim acceptance (main runs, full RUN-MAP-NAV Steps 0–3 verbatim, Gazebo-judged)
- Drive in landmark mode. Success = STALE fraction **drops** vs the ~32% baseline,
  bench/table observation distances shrink (via the stale-diag), **no backward-
  teleport jumps** in RViz, and the robot **reaches the goal marker in Gazebo**
  (judged by the Gazebo view, never by move_base SUCCEEDED/dist).

## Risks / honest caveats

1. **Thin near-edge-only views** under-constrain the rectangle fit (it can slide
   along the edge). Mitigated by the known-size depth constraint + sparse-cluster
   fallback; real impact measured in-sim.
2. **Table cluster includes chairs**, so its point set isn't a clean rectangle
   outline — the ICP may be pulled by chair points. May need outlier rejection in
   the fit or a looser table tolerance; assessed in-sim.
3. **ICP cost** per cluster per tick at 2 Hz — small clusters (tens–hundreds of
   points), few objects; expected tractable, watch CPU.
4. **MAX_JUMP tuning** trades bad-fix rejection against dropping good drift-
   corrected fixes (raising STALE). Start ~3 m, tune in-sim.
5. **Catalog regen** must not change existing (x, y) — only add yaw. Verify the
   positions are byte-identical after regeneration.

## Out of scope
- Changing lamp/bin/tree position logic (already adequate).
- Absolute-yaw matching (heading-dependent; rejected).
- Re-anchoring (fatally circular; rejected earlier).
