# RANSAC Whole-Set Constellation Matcher — Design

**Date:** 2026-08-13
**Branch:** `feat/constellation-matcher`
**Status:** design approved, pending spec review

## Problem

In landmark mode the robot goes STALE mid-drive and fails to reach the goal. Root
cause, measured in-sim over a long debugging session (not guessed):

**Transient odom-prior drift spikes to ~4m during turns** (skid-steer wheel scrub),
and ~4m is **comparable to the spacing between same-type landmarks** in this park.
The localizer's prior is `anchor + odom`; it is accurate (~0.5m) when the robot is
stopped/straight but spikes to ~3.9m on turns, then recovers.

Both simple matchers fail at this drift scale:
- **Nearest-neighbor** (`solve.associate`): matches each observation to the nearest
  same-type catalog landmark *under the prior*. At ~4m drift with `dist_gate=4`,
  observations project past the gate → no match → STALE. Widening the gate to 6m
  kept matches alive but then grabbed the *wrong* same-type neighbor → inconsistent
  fit → residual gate rejects → STALE. **No gate value wins because drift ≈ spacing.**
- **Old constellation matcher** (`_grow`): identifies by drift-invariant pairwise
  distances (right idea) but builds incrementally from ONE seed and SKIPS any
  observation with a tie (>1 candidate within tol). In dense same-type areas ties
  are constant → constellation never reaches 3 → `assoc=0` → STALE.

Measured invariants: prior position accurate at rest, cloud-odom timing negligible
(~0.1s), catalog correct, observations correct (center-offset fix applied). The
failure is specifically **prior drift ≈ landmark spacing defeating prior-dependent
association**.

## Goal

Replace `_grow` with a **RANSAC whole-set matcher** that identifies landmarks by
the shape of the whole observed set (drift-invariant), so it survives the ~4m
transient drift. Drop-in replacement for `constellation.match` (same signature and
return type); `solve_pose`, the localizer, catalog, classify, crop, and trees are
unchanged.

## Why RANSAC beats "drift ≈ spacing"

RANSAC derives the robot pose from a *shape seed* (a pair of observed landmarks
matched to a catalog pair by their frame-invariant pairwise distance — no prior),
then verifies that pose against the *whole observed set* by counting inliers. A
correct arrangement produces many inliers; a wrong seed scatters and produces few.
The prior never gates identification, so its drift cannot break the match. Ties on
any single landmark are irrelevant — the whole-set inlier count picks the correct
global arrangement.

## Design

### Algorithm — `constellation.match(observations, gated, prior_xyz, tol, max_prior_dist)`

Returns `[(Observation, MapLandmark)]` (unchanged contract).

1. If `len(observations) < 3` or `len(gated) < 3` → return `[]`.
2. Precompute (reuse existing helpers):
   - `obs_d = _obs_pair_dists(observations)` — distance between each observed pair.
   - `cat_idx = _cat_pair_index(gated)` — catalog pairs keyed by `frozenset(types)` →
     list of `(L_a, L_b, dist)`.
3. For each observed pair `(o_i, o_j)`:
   - Look up catalog pairs with `frozenset(o_i.identity, o_j.identity)` whose
     `|dist(L_a,L_b) − obs_d[i,j]| ≤ seed_tol`.
   - For each type-consistent orientation (`_seed_orientations`):
     - Compute the rigid transform `T` (via `rigid_transform_2d`) from the two seed
       correspondences: observed `(o_i, o_j)` (robot frame) → catalog `(L_a, L_b)`
       (map frame). `T` is the implied robot map pose.
     - Score: project every observation `o_k` through `T` to map position `p_k`;
       find the nearest same-type catalog landmark to `p_k`; if that distance
       `≤ inlier_tol` AND the landmark is not already claimed by a closer inlier,
       count it as an inlier `(o_k, landmark)`.
     - Track the `(T, inliers)` with the maximum inlier count.
4. `best` = max-inlier candidate. If `len(best.inliers) < 3` → return `[]`.
5. Sanity check: the implied robot pose (from `best`) must be within
   `max_prior_dist_sanity` (~15m, WIDE) of `prior_xyz`; else return `[]`. This is a
   final-only guard against a rare wrong-but-self-consistent solution; it is NOT a
   real gate (15m >> any real drift).
6. Return `best.inliers` as `[(Observation, MapLandmark)]`.

`solve_pose` (unchanged) then refits the pose from these inlier pairs via
`rigid_transform_2d` and applies the existing residual gate as the final quality
check.

### Inlier one-to-one

Within one candidate's scoring, a catalog landmark may be claimed by at most one
observation — the nearest. When two observations project near the same catalog
landmark, keep the closer as the inlier (drop the other from this candidate's
inlier set). This prevents a duplicate from inflating the inlier count.

### Parameters

| param | value | rationale |
|---|---|---|
| `seed_tol` | 1.0 m | pairwise-distance match for seeds; frame-invariant, tolerant of centroid noise, NOT of drift (drift doesn't affect pair distances) |
| `inlier_tol` | **0.5 m** (TIGHT) | the discriminator — a correct transform lands observations within 0.5m of their catalog landmarks; a wrong seed scatters. This tightness is what rejects wrong matches. |
| min inliers | 3 | 3 non-collinear correspondences pin position+heading (no reflection flip) and give a meaningful residual |
| `max_prior_dist_sanity` | 15 m (WIDE) | final-only sanity; tolerates ~4m drift with huge margin |

`seed_tol` reuses the existing `tol` parameter slot; `inlier_tol` and the sanity
distance are new module constants (or params). The localizer passes its existing
`constellation_tol` as `seed_tol`.

### What changes

- **`constellation.py`:** replace the `_grow` + candidate-size-selection +
  `_prior_dist` primary-filter block with the RANSAC loop. Reuse `_obs_pair_dists`,
  `_cat_pair_index`, `_dist`, `_seed_orientations`. Remove `_grow`, `_centroid`,
  `_prior_dist` (or keep only if still used). Import/use `rigid_transform_2d` (move
  it to a shared spot or import from solve — resolve the import direction cleanly to
  avoid a cycle; simplest: a small local 2-point transform helper, or import solve's
  at call time).
- **`solve.py`:** revert the nearest-neighbor experiment — `solve_pose` calls
  `constellation.match` again (not `associate` + dedup). Keep `associate`,
  `rigid_transform_2d`, the min-3 and residual-gate logic. The `_dedupe_one_to_one`
  helper is no longer needed by `solve_pose` (RANSAC handles one-to-one internally);
  keep or remove per cleanliness.
- **`localizer_node.py`:** the diag `_pairs` call reverts to
  `solve.constellation.match(...)`. `dist_gate` reverts to being unused by the match
  path (or removed); the match uses `constellation_tol` as `seed_tol`. Keep the
  center-offset fix, sync fix, crop, trees.
- **Unchanged:** catalog, classify (center-offset stays), segment, crop, trees,
  the EKF wiring.

### Import-cycle note

`rigid_transform_2d` lives in `solve.py`, which imports `constellation`. If
`constellation` needs `rigid_transform_2d`, importing solve at module load creates a
cycle. Resolve by either (a) moving `rigid_transform_2d` into a small shared module
(e.g. `geom.py`) imported by both, or (b) a local 2-point rigid-transform helper
inside `constellation` for the seed, and letting `solve_pose` do the final N-point
refit. Prefer (a) for cleanliness; (b) is acceptable if simpler.

## Complexity

Seed pairs: (observed pairs ≈ 21 for N=7) × (matching catalog pairs per type combo,
pruned to a handful by type+distance) × 2 orientations ≈ low hundreds. Each scores
N≈7 observations against ~15 gated. Sub-millisecond; fine for 2 Hz.

## Testing

### Unit (`test_constellation.py`)
- **Clean match:** 3+ landmark scene at the correct pose → correct inlier pairs and
  pose.
- **Drift-immunity (the key test):** same scene, prior 4m off → still returns the
  correct match (inliers from the seed transform, not the prior). A prior 10m off →
  still correct. This is the property the whole rewrite exists for.
- **Dense/tie:** multiple same-type landmarks within a few metres → RANSAC picks the
  max-inlier arrangement, not confused by per-landmark ties (the `_grow` failure
  case). Assert the correct, larger constellation wins.
- **Wrong-seed rejection:** a scene where no consistent arrangement exists → best
  inliers < 3 → returns `[]`.
- **Reflection/flip:** a 2-point-only consistent scene → cannot win (min-3) → `[]`.
- **Sanity guard:** a self-consistent constellation whose implied pose is >15m from
  the prior → rejected.

### In-sim acceptance (main runs, judged by Gazebo, NOT move_base SUCCEEDED)
Full demo per RUN-MAP-NAV.md: GPS → spoof → switch to landmark → the robot drives
the whole route **through its turns** and reaches the goal. Success = the robot's
actual Gazebo position reaches the goal marker, AND the diag shows fixes staying
alive during the drift spikes (no long assoc=0/STALE stretches on turns). Compare to
the pre-RANSAC baseline (STALE mid-drive, stalled ~6m short).

## Risks / honest caveats

1. **Two genuinely-identical landmark shapes within the gated region** → pure
   geometry could pick the wrong one (we chose no prior tiebreak). Rare in this
   park; if it bites, add a prior tiebreak among equal-inlier winners (a small,
   later change).
2. RANSAC is more code than a tweak — it is the real rewrite, but standard and
   self-contained in `constellation.py`.
3. Assumes ≥3 correctly-classified landmarks are visible; genuinely sparse spots
   still go STALE (acceptable — the robot coasts on odom briefly).

## Out of scope
- Re-anchoring (fatally circular — needs a fix to re-anchor to, which is exactly
  what STALE lacks; abandoned).
- Reducing odom drift itself.
- Any change to the observation pipeline, EKF, or mux.
