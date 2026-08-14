# Constellation Landmark Matcher — Design

**Date:** 2026-08-11
**Branch base:** `feat/live-pose-source-switch` @ `e402efe` (current tuned localizer)
**Status:** design approved, pending spec review

## Problem

The landmark localizer goes intermittently STALE in landmark mode, especially
after switching from GPS mid-attack. Measured root cause (this session), not
guessed: the matcher `associate()` in `landmark_loc/solve.py:19` identifies each
observed landmark by projecting it into the map **through the odom-anchored
prior** and grabbing the nearest catalog landmark. The prior is `anchor + odom
displacement`; odom drifts (skid-steer wheel scrub, uncorrected). Once drift
exceeds the landmark spacing, the projection lands nearest the WRONG catalog
landmark — or none — so association drops below 2 correspondences (or the wrong
matches inflate the Umeyama RMS above the residual gate). Either way `solve_pose`
returns `None`, the node publishes nothing, and the mux reports `:stale`.

Proven that no `dist_gate` value fixes this: 2 too tight (misses), 4 best
compromise (~half stale), 6 too loose (grabs wrong far matches → high RMS →
rejected). The failure is the prior-dependence of identification, upstream of any
gate.

### Measured target population (previous run, 3,571 paired diag ticks)

| Outcome | Ticks | Note |
|---|---|---|
| Good fix (assoc≥2, RMS≤1.0) | 2,823 (79%) | already working |
| obs≥3 but assoc<2 | 231 (6.5%) | **matcher fixes**: sees plenty, drifted prior mismatches |
| obs≥3, assoc≥2, RMS>1.0 rejected | 68 (2%) | **matcher fixes**: wrong matches → high RMS |
| obs<3 (genuinely landmark-lean) | ~12% | not fixable by any shape method; coast on odom |

Distribution of identifiable landmarks per tick: **81% of ticks see 3+**, so the
constellation approach has enough shape the large majority of the route.

## Fix — prior-free constellation matching

Identify each observation by the **robot-free shape** of the observed landmark
group (pairwise distances between landmarks), matched against the catalog using
**type + geometry**. A distance between two points is frame-invariant, so it does
not move when the prior drifts — breaking the chicken-and-egg where identification
needed a good pose to produce a good pose.

The prior is used in exactly ONE place: a coarse regional tiebreaker when two
catalog constellations fit equally well. Drift cannot break a "which part of the
park" decision.

## Architecture — single drop-in swap

Pipeline unchanged except one function:

```
cloud → crop → cluster → classify → catalog.gate → [associate] → rigid_transform_2d → publish
                                                     ^^^^^^^^^ only this changes
```

`associate()` is replaced by a constellation matcher with the **same signature
and return type**. `solve_pose`, `rigid_transform_2d`, `localizer_node.py`,
`catalog.gate`, the EKF, the mux, and the operator are all untouched.

- **Input (unchanged):** `observations` (typed, robot-frame x,y — `classify.Observation`),
  `gated_landmarks` (typed, map-frame x,y — `catalog.MapLandmark`), `prior_xyz`,
  plus one new tolerance param `constellation_tol`.
- **Output (unchanged):** `pairs` — list of `(observation, map_landmark)`.
  `solve_pose` consumes it exactly as today.

## Algorithm

1. **Build the observed shape.** Pairwise distances among observations (N×N table),
   each pair tagged with its two types. Robot-frame distances equal map-frame
   distances → directly comparable to the catalog.
2. **Build catalog candidate pairs.** Same pairwise distances among
   `gated_landmarks`, tagged with types.
3. **Match pairs by type + distance.** An observed pair `lamp↔bench = 6.2 m` can
   only correspond to a catalog pair that is also `lamp↔bench` and also ~6.2 m
   apart, within `constellation_tol`.
4. **Grow a consistent set.** For each candidate seed pair from step 3
   (deterministic order: iterate observation pairs by index, catalog matches by
   catalog order), grow it — add a third observation only if its distances to
   BOTH already-assigned landmarks match a catalog landmark's distances AND type.
   Grow until no more fit. A set of ≥2 mutually-consistent correspondences is a
   valid constellation; keep the largest such set found across seeds (ties →
   step 5).
5. **Resolve ambiguity with the prior.** If two catalog constellations both fit,
   pick the one whose landmarks are closest to the coarse odom prior.
6. **Return the pairs** from the winning constellation. `solve_pose` proceeds
   unchanged (Umeyama → pose → residual gate → publish).

The prior appears only in step 5.

## Error handling & edge cases

- **<3 observations (~19% of ticks).** With 2 distinct-type observations, if
  exactly one catalog pair matches on both types and distance, use it. With 0/1
  observations, or an ambiguous 2-obs pair, return empty pairs → `solve_pose`
  None → node silent → EKF coasts on odom. Same graceful degradation as today;
  no regression.
- **No consistent constellation.** Return empty pairs; never fabricate a fit.
- **Distance-tolerance collisions** (two same-type landmarks close together).
  Disambiguated by step 4's mutual consistency (the third landmark) and step 5's
  prior tiebreak; if still ambiguous, reject rather than guess.
- **Degenerate/collinear geometry.** A near-collinear triple is flip-symmetric;
  the existing reflection guard (`solve.py:43`) plus the residual gate reject a
  wrong flip downstream. No new handling.
- **Tolerance param.** One new ROS param `constellation_tol` (~0.3 m for
  lidar/centroid jitter). Gates on RELATIVE distance (stable), so far less
  sensitive than `dist_gate`.
- **Performance.** N small (obs ≤9 seen; gated catalog a handful). Pairwise
  tables tiny; negligible at 5 Hz.

## Testing

**Unit (`landmark_loc/tests/test_constellation.py`, pure Python, no ROS/sim):**

1. Clean 3-landmark match → 3 correct pairs.
2. **Drift-immunity** — same observations, `prior_xyz` 8 m off truth → still
   returns correct pairs. (Nearest-neighbor fails this; matcher must pass. This
   is the test that proves the fix.)
3. Too few landmarks — 1 obs → empty; 2 distinct-type with a unique catalog pair
   → that pair; 2 ambiguous → empty.
4. No match — shape fits nothing → empty.
5. Type constraint — observed lamp geometrically near a catalog bench must NOT
   pair.
6. Ambiguity + prior tiebreak — two identical catalog constellations; the one
   nearer the prior wins.
7. Collinear/degenerate — no wrong-flip pair survives (or is caught downstream).

**Regression:** existing `solve_pose` / `rigid_transform_2d` tests stay green
(their contract is unchanged; matcher output feeds them identically).

**In-sim acceptance (run by main):** full demo — GPS drive → spoof →
`mode landmark` mid-attack → robot reaches goal under active spoof, AND the
"sees 3+ but stale" ticks (231+68 measured) now produce fixes. Compare
stale-rate before/after on the same route from `[diag]` logging. Success =
markedly fewer stale ticks in landmark-rich stretches, robot reaches goal.

## Non-goals

- Not fixing the ~12% genuinely landmark-lean ticks (obs<3) — unwinnable for any
  shape method; odom coasts those short gaps.
- Not touching `compose_prior`, the anchor logic, the gates, the EKF, or the mux.
- Not a rewrite — one function (~30–50 lines) plus tests.

## Rollback

Current tuned state is `e402efe`. Work happens in an isolated worktree/branch; if
the matcher doesn't pan out, discard the branch and `e402efe` stands untouched.
