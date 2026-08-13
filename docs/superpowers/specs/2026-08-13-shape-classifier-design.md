# Shape-Based Landmark Classifier — Design

**Date:** 2026-08-13
**Branch:** `feat/constellation-matcher` (worktree `constellation-matcher`)
**Status:** design, pending user review

## Problem (measured this session)

The current classifier (`classify.py::_matches`) decides a cluster's type by
comparing its **bounding-box extents** (`major`, `minor`, `height`) against each
object's **whole-mesh bounding box** (`signatures.py`), widened by loose margins
(`±0.8 / 0.6 / 1.0 m`). Measured on a live captured frame (15 clusters, robot
parked among park furniture):

- **12 of 15 clusters (80%) were dropped as `unknown`.** Among them, at least
  **four lamps** — clusters whose points form an unmistakable thin vertical post
  (e.g. cluster [10]: 238 points, footprint 0.15 m, height 2.09 m) — were dropped
  because their *observed* minor extent (0.06–0.25 m) does not match the lamp's
  *whole-mesh* minor (0.48 m). The whole-mesh box includes the lamp head, which
  the lidar usually does not see.
- The loose margins simultaneously **admit phantoms**: a 0.13×0.07 m pole fragment
  falls inside the trash-bin band because that band accepts major 0–1.48, minor
  0–0.98.

Root cause: **absolute-size-band matching is not viewpoint-invariant.** A partial
lidar view of an object has a different bounding box than the full mesh, so the
size band either misses the real object or admits the wrong one. This is the
failure the pole-landmark literature (arXiv 2305.06845) designs around by not
keying on absolute size.

## Key insight — classify by the object's REAL shape, not its bounding box

Each park object has a **distinctive multi-part 3-D shape** that is far more
viewpoint-stable than its bounding box. Measured directly from each `.dae` mesh
(`mesh_bounds` vertex extraction, applying node transforms):

| object | height | footprint (major×minor, aspect) | vertical structure | round? |
|---|---|---|---|---|
| **lamp** | **3.15 m** | post ~0.14×0.13 (≈round) low; head 0.63×0.48 at top | **thin round post** z 0→~1.3 m, **head flares** at z>2.5 m | post round; head not |
| **trash_bin_1** | 1.04 m | **0.68×0.38, aspect 1.79 (oblong box)** | uniform oblong column, full width at all z | **NOT round** |
| **bench** | 0.94 m | 1.78×0.80 aspect 2.2 low → **aspect 7.7** high | **wide seat/base low** (z<0.57), **thin back panel high** (z>0.57, minor→0.25) | no |
| **garden_table** | 1.09 m | 3.00×1.32, aspect 2.27 | wide flat top (3.0 m long); narrower legs/base low | no |
| **tree** | ~3–5 m | trunk ~0.9 m → canopy 2.9–4.75 m | **narrow trunk low, wide canopy high** (existing gate) | trunk round |

The discriminators that survive partial views (each measured, not assumed):

1. **lamp** — the only object with a **thin (~0.14 m) near-round post extending
   above ~1.3 m**. Nothing else is that skinny that high. Robust from any angle or
   range; the lidar sees the post even when it misses the head. (Captured lamps
   [0],[4],[10],[11],[14] all show this post; the current classifier caught only 1.)
2. **trash_bin** — a **short (≤~1.2 m) oblong box, ~2:1 footprint, no thin post.**
   Distinct from lamp by height (1.0 vs 3.15 m) AND post-width (0.38 vs 0.14 m);
   distinct from bench/table by footprint size (much smaller).
3. **bench vs table** — both low wide boxes; separated by **length** (bench 1.78 m,
   table 3.00 m) and bench's thin-back-panel-high signature.
4. **tree** — trunk-narrow-low / canopy-wide-high vertical profile (unchanged; the
   existing `_is_tree` gate already works and is the best of the current rules).

**Correction to an earlier (wrong) direction:** lamp and trash_bin are NOT
genuinely ambiguous. That impression came from the bounding-box view (which
collapses both to "smallish"). In real geometry they differ ~3× in height and ~4×
in post-width — widely separable. So the classifier keeps **all five fine labels**
(lamp, trash_bin_1, bench, garden_table, tree); nothing is merged into a coarse
class. The matcher (`constellation.py`) is **untouched** — it still receives
specific identities, preserving its discrimination.

## Goal

Replace the absolute-size-band decision in `classify.py` with a **shape-signature
match**: for each cluster, measure a small set of **viewpoint-stable shape
features** from its real points, and assign the fine label whose measured shape
signature the cluster fits — or `unknown` if it fits none. Keep every fine label.
Keep the matcher, the catalog, `shapefit.fit_rectangle` (pose refinement), and the
tree gate as they are.

## Shape features (per cluster, all viewpoint-stable)

Computed from the cluster's real `(N,3)` points. None depends on seeing the whole
object.

- **`height`** = z-extent (`max−min`). Already available on `Cluster`.
- **`post_width`** = the footprint diagonal of points in the **low band**
  (z_base → z_base + 0.8 m). This isolates the *post/base*, not the head/canopy.
  For a lamp this is ~0.14 m; for a bin ~0.7 m; for a bench/table ≥1.3 m.
- **`post_round`** = circle-fit radial-RMS / radius on that same low band
  (roundness ratio; < ~0.15 ⇒ round). Lamp post ≈ round (≈0.0–0.1); bin oblong,
  bench/table not round.
- **`tall_thin`** = does a **thin** (footprint diagonal < 0.4 m) band exist at
  **height > 1.3 m**? True only for lamp (its post reaches high). Directly
  separates lamp from the short bin. Measured from the point profile, not the
  bounding box.
- **`foot_major` / `foot_minor`** = PCA extents of the **full** footprint (all z).
  Used only to separate the two low boxes (bench 1.78 vs table 3.00 m major).
- Tree is decided first by the **existing `_is_tree` profile gate** (wide canopy
  band at z ≥ 2.5 m), which already works and needs no change.

## Classification rule (decision order)

Ordered most-specific-first; first match wins; no match ⇒ `unknown`.

1. **tree** — `_is_tree(cluster)` (existing gate, unchanged). A real trunk+canopy
   must win before any size rule, exactly as today.
2. **lamp** — `tall_thin` is True: a thin band (diag < ~0.4 m) exists above ~1.3 m
   AND the low-band `post_round` indicates a round-ish thin post (`post_width` <
   ~0.35 m). This is the lamp's signature and nothing else satisfies it.
3. **trash_bin_1** — short (`height` < ~1.4 m) AND compact oblong footprint
   (`foot_major` in ~[0.4, 1.0] m, aspect ~1.3–2.3) AND no tall-thin post. The
   short oblong box.
4. **garden_table** — low box, **long**: `foot_major` ≳ 2.3 m (toward 3.0),
   `height` < ~1.4 m, wide (`foot_minor` ≳ 0.9 m).
5. **bench** — low box, **medium length**: `foot_major` in ~[1.3, 2.3] m,
   `height` < ~1.2 m, with the wide-low / thin-high seat-plus-back profile.
6. else **unknown**.

Thresholds above are **seed values from the measured mesh + captured clusters**;
they are pinned in-sim (see Testing). Exact values live in `classify.py` constants,
documented with the measurement each came from — never a bare magic number.

### Why this is not just "new bands"

The current rule matches **absolute bounding-box size of the whole object**. This
rule matches **structural features that a partial view still exhibits**: a lamp is
"a thin round post that goes high" whether the lidar sees 20 % or 100 % of it; a
bin is "a short oblong box" from any side. The features are chosen so that the
*visible fraction* does not change the answer — which is precisely what the
size-band rule got wrong.

## What changes

- **`classify.py`:** replace `_matches` (size-band) with the shape-feature rule
  above and its helpers (`_post_width`, `_post_round`, `_tall_thin`,
  `_foot_extents`). `classify_cluster` keeps its signature and return contract
  (one of the five labels or `"unknown"`). `to_observations` is **unchanged** —
  it still calls `shapefit.fit_rectangle` for bench/table pose and the
  centroid+radius push-out for lamp/bin, keyed on the (now shape-derived) label.
- **`signatures.py`:** keep the mesh dimensions, but the classifier no longer uses
  the whole-mesh box as a band center. Repurpose/extend it to record the
  **measured shape signature per object** (post_width, height, foot_major,
  roundness) so the sim and classifier still agree by construction. The raw
  `MESH_SIGNATURES` numbers stay available for `KNOWN_RADIUS` and pose push-out.
- **Unchanged:** `constellation.py` (matcher), `catalog.py`, `park_places.yaml`,
  `shapefit.py`, `localizer_node.py` wiring, the tree gate, the motion-jump gate,
  compass prior, buff_size.

## Testing

### Unit (`landmark_loc/tests/`)
- **Synthetic shape sets** built from the measured signatures at several
  viewpoints (near post-only lamp, far lamp, angled bench near-edge, table with
  chairs, short oblong bin): each returns its correct fine label; a thin pole
  fragment does NOT return `trash_bin` (the phantom the old margins admitted).
- **Regression on the 15 captured clusters** (saved this session): assert the
  four dropped lamps [0],[4],[10],[11] now classify as `lamp`; bench [12] stays
  `bench`; bin [13] stays `trash_bin_1`; lamp [14] stays `lamp`. This is the
  concrete before/after the whole redesign exists to fix.
- Existing classify/constellation/shapefit tests continue to pass unchanged
  (matcher and catalog are untouched).

### In-sim acceptance (main runs, full RUN-MAP-NAV Steps 0–3 verbatim, Gazebo-judged)
Success =
- the `unknown` fraction **drops sharply** from the measured 80 % (target: most
  lamps/benches/bins/tables in view are labeled), captured via the stale-diag;
- **no phantom labels** on pole fragments / ground blobs;
- the constellation forms a correct ≥3-inlier match far more often (STALE fraction
  falls);
- the robot **reaches the goal marker in Gazebo** in landmark mode after a GPS
  spoof + switch (judged by the Gazebo view, never by move_base SUCCEEDED/dist).

### In-sim pinning (main runs, requires user go-ahead)

The thresholds in `classify.py` are seed values from object meshes plus one
session's captured clusters — PROVISIONAL, not yet pinned against a live run.
Pinning is main-conversation work, gated on explicit user consent, and is not
performed by an implementer subagent:

- Run RUN-MAP-NAV Steps 0–3 **verbatim**, from a **clean kill** (no reused or
  stacked processes).
- Capture the stale-diag `unknown` fraction during the landmark drive.
- Confirm **no phantom labels** are admitted (pole fragments / ground blobs
  classified as a real object type).
- Confirm the robot **reaches the goal marker in Gazebo** after the GPS spoof
  + landmark-mode switch, judged by the Gazebo view only — never by move_base
  SUCCEEDED/dist and never by fused pose.
- If a real object is dropped as `unknown`, or a phantom is admitted, adjust
  the relevant seed threshold(s) in `classify.py` and record the newly
  measured value (and the cluster/run it came from) in the constant's comment,
  the same way the Task 2/4 measurements are recorded there now.

## Risks / honest caveats

1. **lamp head vs post visibility.** The rule keys on the thin post above 1.3 m.
   If a lamp is seen only very close and low (post base only, < 1.3 m of it), the
   `tall_thin` test could miss it. Mitigation: the captured lamps all showed post
   up to ~1.6–2.0 m, so this is the uncommon case; measured in-sim.
2. **bench near-edge-only view** can foreshorten `foot_major` below 1.3 m and look
   bin-sized. The wide-low/thin-high profile and the shape-fit residual help
   disambiguate; assessed in-sim. This is the same weak view the shape-fit design
   already flagged.
3. **Threshold pinning.** Seed thresholds come from one captured frame + the
   meshes. They must be pinned against a full landmark drive before the branch is
   called done — the numbers here are a starting point, not final.
4. **Tree gate at range.** Far trees whose canopy foreshortens below the canopy
   width floor still drop to `unknown` (observed: clusters [5],[6],[7]). This
   redesign does not fix the tree gate; out of scope, noted.

## Out of scope
- Coarse/merged classes (POLE/BOX) — rejected; the real geometry separates the
  fine types, so merging would needlessly weaken the matcher.
- Changing the constellation matcher, catalog, or pose-refinement shape-fit.
- Improving the tree gate's range behavior.
- Full mesh-ICP classification — the earlier proof showed it does not discriminate
  on partial views (1.1×); the structural-feature approach here is what the
  captured-cluster numbers support.
