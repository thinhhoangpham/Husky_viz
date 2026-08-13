# Tree (tree_8) Landmark Type — Design

**Date:** 2026-08-12
**Branch:** `feat/constellation-matcher` (extends the constellation matcher; not yet merged)
**Status:** design approved, pending spec review

## Problem

In landmark mode the fused pose still **wanders multi-metre during the approach
drive** to a goal, even with the constellation matcher + median smoothing. Measured
cause (this session): near the goal the robot often sees only 2 same-type landmarks;
2 points give a single pairwise distance, which is not unique, so the matcher
locks onto ambiguous / wrong constellations for stretches, and the published fix
lurches (26 moves >1m in one drive, up to 11.6m). The median-of-5 smoothing hides
isolated outliers but not a *run* of consistent wrong fixes. move_base chases each
lurch, so the robot weaves and circles before settling.

Root lever: **too few landmarks in view.** The catalog has only 53 landmarks
(bench/lamp/bin/table), clustered, with gaps along the ~35 m approach route. The
park also contains **38 trees that are currently discarded** — a dense, well-spread
set of potential landmarks the matcher is blind to.

## Goal

Add **trees as a new `tree` landmark type** feeding the existing constellation
matcher, to give it more geometric anchors along the approach route and reduce the
wandering. Scope: **`tree_8` only** (23 tall bare trees). `arbolpartes4` (15 short
bushes) is **excluded** — see Decision below.

## Why this works (constellation matcher recap)

Every landmark type is *anonymous within its type* (the lidar sees "a tree", not
"tree #17"). The constellation matcher already identifies anonymous same-type
landmarks **as a group by their pairwise-distance fingerprint** (frame-invariant,
drift-immune), with a type constraint. Trees are not a new *kind* of problem — they
are *more of the same*: 23 more anonymous points, densely spread, so the matcher
sees 3-5 landmarks where it used to see 2, making the fingerprint unique. Trees are
valuable because they are **numerous and spread out**, not because they are
individually identifiable.

## Decision: tree_8 only, arbolpartes4 excluded (measured)

Two tree families exist. Measured live in-sim:

| | tree_8 (23) | arbolpartes4 (15) |
|---|---|---|
| Appearance | tall bare tree | short dense bush |
| Canopy height (lidar) | 4.0-7.75 m | ~2.3 m (cropped) |
| Canopy width | 3.7-4.75 m | 2.18 m, minor 0.59 |
| Structure | single link_0 = trunk+canopy | link_0 trunk **and** link_1 bush, **2.24 m apart** |
| Verdict | clean, view-robust landmark | overlaps lamp/bench size; visible part (bush) offset from trunk → catalog trap |

tree_8 is the high-value, low-risk lever. arbolpartes4 is deferred (add later only
if tree_8 is insufficient).

## Measured thresholds

All numbers below are measured live (not guessed). Basis: profile probe + a
13-tree sample across 4 viewpoints.

| Threshold | Value | Basis |
|---|---|---|
| Crop floor `z_min` | **-0.5 m** | ground return sits at z<-0.5 (3.5-4.5 m wide blob under every object); real object points start ≥ -0.5. Lifting the floor drops the ground blob, keeps the trunk. |
| Crop ceiling `z_max` | **7.0 m** | canopy wide-band spans z 2.5-7.75; 7.0 captures it. (old 3.5 cut the canopy mid-bloom) |
| Cluster `max_extent` | **6.0 m** | canopy max width measured **4.75 m** (n=13, min 3.69, p50 4.57); 6.0 clears it with margin. (old 3.5 discards every canopy) |
| Canopy-band test | **width ≥ 2.0 m in a z-band at z ≥ 2.5 m** | trees measured 2.9-4.75 m wide there; lamps <1 m at every height; trunk→canopy split measured z 1.75-2.75 (p50 2.25) |

## Design

### 1. Classifier — vertical-profile rule (`landmark_loc/classify.py`)

A cluster is a **`tree`** when it has a **wide canopy above a narrow trunk**: some
horizontal z-band at **z ≥ 2.5 m** has width **≥ 2.0 m**. This keys on the
*structural profile* (view-robust) rather than absolute canopy size (which varies
3.7-4.75 m).

- Replaces the current `_is_tree()` (which drops trees by trunk footprint) with a
  rule that **emits** a `tree` Observation.
- Runs before the four rigid-type signature matches and wins, same control shape as
  today's `_is_tree()`.
- Natural exclusivity (measured): a **lamp** is <1 m wide at every height (never a
  wide high band); a **bench/bin** has no high band at all. So the tree rule cannot
  fire on them, and the four rigid types are undisturbed. This is why raising the
  crop does not require re-pinning the four bands — but the build MUST re-verify it
  in-sim (below).
- The cluster must retain its points (not just bbox) so the classifier can slice
  z-bands. `segment.Cluster` already carries `points`.

### 2. Crop (`landmark_loc/localizer_node.py`)

`z_min`: -0.73 → **-0.5**. `z_max`: 3.5 → **7.0**. Two param defaults; no logic
change.

### 3. Clustering (`landmark_loc/segment.py` caller)

`max_extent`: 3.5 → **6.0** (localizer param). Canopies (max 4.75 m) now survive.
`link_dist` stays 0.3 (trees ≥3.6 m apart → canopies stay separate clusters,
confirmed).

### 4. Catalog (`map_tools/extract_park_map.py`, `landmark_loc/catalog.py`)

- `extract_park_map.py`: add `"tree_8"` to `PLACE_FAMILIES`. The extractor already
  parses tree_8 at link_0 (single-link → correct). Regenerates
  `maps/park_places.yaml` with 23 tree entries.
- `catalog.py`: add `"tree"` to `_IDENTITY_FAMILIES`; map the `tree_8` name-prefix
  → identity `"tree"`. Observed canopy centre aligns to link_0 within 0.08-0.68 m
  (measured), so the trunk position is a valid catalog coordinate.

### 5. Signatures (`landmark_loc/signatures.py`)

**Untouched.** Trees are classified by the profile rule, not a mesh signature.
`SIGNATURE_FAMILIES` stays the four rigid types.

### 6. Data flow downstream — no structural change

`classify.to_observations()` yields `tree` observations; `catalog.gate()` includes
tree landmarks; `constellation.match()` already handles any identity via its type
constraint + geometry (`tree` is just a new type value); `localizer_node` gets the
param changes only. The matcher, solve, and mux are unchanged.

## Testing

### Unit
- Classifier: synthetic wide-canopy-over-trunk cluster → `tree`; thin-all-the-way
  cluster → not tree (lamp-like); low-only cluster → not tree (bench-like).
- Catalog: `tree_8*` names load as identity `tree`; count = 23.

### In-sim acceptance (main runs it, from a clean kill)
Full demo per RUN-MAP-NAV.md: GPS → spoof → switch to landmark → robot reaches
goal under active spoof. **Success metric = reduced approach-route wandering vs the
pre-tree baseline**: fewer/smaller multi-metre jumps in the published fix during the
drive, robot drives a cleaner path to the goal. Also confirm the four existing types
still classify correctly at the raised crop (no regression).

## Risks

1. **Raised crop touches all types' input** — mitigated by measured natural
   exclusivity of the tree rule; MUST still re-verify the four types in-sim.
2. **Canopy merging** — measured low (3.6 m min spacing vs 4.75 m max width; tight
   link_dist=0.3 keeps them separate). Re-confirm in the acceptance run.
3. **max_extent=6.0 could admit a genuinely large non-tree blob** — no such object
   in this park (next largest real cluster is a tree); acceptable.

## Out of scope
- arbolpartes4 bushes (deferred).
- Any change to the matcher, solve, mux, or smoothing (this feature only supplies
  more landmarks).
- Jump-gate / min-3-landmarks matcher changes (separate, still-open ideas).
