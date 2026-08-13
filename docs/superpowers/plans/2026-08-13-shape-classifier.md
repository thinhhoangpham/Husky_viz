# Shape-Based Landmark Classifier Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the classifier's absolute-size-band decision with a viewpoint-stable shape-signature match, so partial lidar views of park objects are labeled correctly instead of dropped as `unknown`.

**Architecture:** A new pure-geometry module `shapefeat.py` extracts viewpoint-stable shape features from a cluster's real points (post width/roundness in a low band, presence of a thin band up high, full footprint extents). `classify.py` replaces `_matches` with an ordered shape-signature rule that consumes those features. The constellation matcher, catalog, pose-refinement shape-fit, and tree gate are untouched; all five fine labels (lamp, trash_bin_1, bench, garden_table, tree) are preserved.

**Tech Stack:** Python 3.8, numpy, pytest. No ROS in the changed modules (pure numpy on `(N,3)` arrays), matching the existing `segment.py`/`classify.py`/`shapefit.py` style.

## Global Constraints

- **Keep all five fine labels.** Do NOT merge types into coarse classes. `classify_cluster` returns exactly one of `{"lamp","trash_bin_1","bench","garden_table","tree","unknown"}`.
- **Do NOT change** `constellation.py`, `catalog.py`, `maps/park_places.yaml`, `shapefit.py`, or `localizer_node.py` wiring. `to_observations`'s pose logic (shape-fit for bench/table, centroid+radius push-out for round types, trunk for tree) stays; only the *label decision* changes.
- **`classify_cluster(cluster, margins=DEFAULT_MARGINS)` keeps its signature** and return contract (a label string). Callers (`to_observations`, tests, `localizer_node`) must keep working.
- **The tree gate wins first** and is unchanged: `_is_tree(cluster)` decides `tree` before any other rule (a real trunk+canopy must be excluded from the other types).
- **Every threshold is a named constant with a comment citing its measurement** (mesh dimension or captured-cluster value). No bare magic numbers.
- **Features require real points.** A cluster with `points=None` or `< _MIN_SHAPE_PTS` points cannot be shape-classified → returns `unknown` (never crashes). Existing `points=None` size-only tests are migrated to point-based fixtures.
- Tests live in `landmark_loc/tests/`; run with `cd <worktree> && PYTHONPATH=. python3 -m pytest <path> -v`.

### Measured signatures (from meshes + 15 captured clusters, this session)

| object | height (m) | low-band post_width (m) | full foot_major (m) | thin band above 1.3 m? | roundness (low band) |
|---|---|---|---|---|---|
| lamp | 3.15 | ~0.14 | 0.63 (head) | **yes** (post rises tall) | round (post) |
| trash_bin_1 | 1.04 | ~0.68 (oblong) | 0.68 | no | not round (aspect 1.8) |
| bench | 0.94 | ~0.80 | 1.78 | no | not round |
| garden_table | 1.09 | ~1.32 | 3.00 | no | not round |

Captured lamps dropped by the OLD classifier that MUST now be `lamp`: clusters [0] (0.46×0.25×2.51), [4] (0.43×0.12×2.53), [10] (0.12×0.06×2.09, 238 pts), [11] (0.51×0.24×2.50), plus [14] (already correct). Fixture: `landmark_loc/tests/fixtures/captured_clusters.npz` + `.json` manifest.

---

## File Structure

- **Create** `landmark_loc/shapefeat.py` — pure shape-feature extraction from `(N,3)` points.
- **Create** `landmark_loc/tests/test_shapefeat.py` — unit tests for the features.
- **Modify** `landmark_loc/classify.py` — replace `_matches` with the shape rule; add feature-based helpers; keep `classify_cluster`/`to_observations` contracts.
- **Modify** `landmark_loc/tests/test_classify.py` — migrate size-only (`points=None`) tests to point-based fixtures; add the shape-rule cases.
- **Create** `landmark_loc/tests/test_captured_regression.py` — assert the redesign labels the 15 captured real clusters correctly (the concrete before/after).
- **Present (committed data):** `landmark_loc/tests/fixtures/captured_clusters.npz` + `captured_clusters.json` (already saved).

---

## Task 1: Shape-feature extraction module (`shapefeat.py`)

**Files:**
- Create: `landmark_loc/shapefeat.py`
- Test: `landmark_loc/tests/test_shapefeat.py`

**Interfaces:**
- Produces (consumed by Task 2's classifier):
  - `LOW_BAND = 0.8` (m), `HIGH_Z = 1.3` (m), `THIN_DIAG = 0.4` (m), `_MIN_SHAPE_PTS = 6`
  - `foot_diag(points_xy) -> float` — bbox diagonal of xy points.
  - `pca_extents(points_xy) -> (major, minor)` — PCA span (mirror of `segment._pca_extents`, reused here to avoid a cross-import; identical formula).
  - `circle_roundness(points_xy) -> (radius, ratio)` — Kasa circle fit; `ratio = radial_rms / radius`. `ratio < 0.15` ⇒ round.
  - `post_width(points) -> float` — `foot_diag` of points in the low band `[z_base, z_base+LOW_BAND)`; `0.0` if too few.
  - `has_thin_high_band(points) -> bool` — True if any 0.5 m z-band above `HIGH_Z` has `foot_diag < THIN_DIAG` with ≥3 points (the lamp post signature).
  - `foot_extents(points) -> (major, minor)` — PCA extents over the full footprint (all z).

- [ ] **Step 1: Write the failing tests**

```python
# landmark_loc/tests/test_shapefeat.py
import numpy as np
from landmark_loc import shapefeat as sf


def _lamp_post(height=2.5, w=0.14, n=40):
    """Thin near-round vertical post from z=0 to height."""
    pts = []
    for z in np.linspace(0.0, height, n):
        for a in np.linspace(0, 2 * np.pi, 6, endpoint=False):
            pts.append((0.5 * w * np.cos(a), 0.5 * w * np.sin(a), z))
    return np.array(pts, float)


def _low_box(major=1.78, minor=0.80, height=0.9, n=8):
    """Low wide rectangular box (bench-like)."""
    pts = []
    for z in np.linspace(0.0, height, 4):
        for x in np.linspace(-major / 2, major / 2, n):
            for y in (-minor / 2, minor / 2):
                pts.append((x, y, z))
    return np.array(pts, float)


def test_post_width_isolates_thin_post():
    p = _lamp_post()
    assert sf.post_width(p) < 0.25          # thin post, not the head/full box


def test_has_thin_high_band_true_for_lamp():
    assert sf.has_thin_high_band(_lamp_post(height=2.5)) is True


def test_has_thin_high_band_false_for_low_box():
    assert sf.has_thin_high_band(_low_box()) is False


def test_has_thin_high_band_false_for_short_post():
    # a post that stops below HIGH_Z has no thin band up high
    assert sf.has_thin_high_band(_lamp_post(height=1.0)) is False


def test_foot_extents_recovers_box_major():
    mj, mn = sf.foot_extents(_low_box(major=1.78, minor=0.80))
    assert abs(mj - 1.78) < 0.2 and abs(mn - 0.80) < 0.2


def test_circle_roundness_round_post_low_ratio():
    r, ratio = sf.circle_roundness(_lamp_post()[:, :2])
    assert ratio < 0.15                      # round


def test_circle_roundness_oblong_box_high_ratio():
    r, ratio = sf.circle_roundness(_low_box(major=0.68, minor=0.38)[:, :2])
    assert ratio > 0.15                      # not round


def test_too_few_points_safe():
    p = np.zeros((3, 3))
    assert sf.post_width(p) == 0.0
    assert sf.has_thin_high_band(p) is False
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd <worktree> && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_shapefeat.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'landmark_loc.shapefeat'`.

- [ ] **Step 3: Write `shapefeat.py`**

```python
"""Viewpoint-stable shape features for landmark classification.

Pure numpy on (N,3) point arrays in the lidar frame. Unlike a bounding box, these
features describe an object's STRUCTURE (a thin post that rises high, a low wide
box, an oblong column), which a partial lidar view still exhibits. Thresholds are
measured from the object meshes and live captured clusters (see the plan/spec).
"""
import numpy as np

LOW_BAND = 0.8      # m: height of the near-base band used to measure the post/base
HIGH_Z = 1.3        # m: a thin band ABOVE this height is the lamp-post signature
                    # (lamp post rises past 1.3 m; bin/bench/table do not)
THIN_DIAG = 0.4     # m: footprint diagonal below this is "thin" (lamp post ~0.14 m)
_BAND = 0.5         # m: z-band thickness for the high-band scan
_BAND_MIN_PTS = 3   # a band needs this many points to measure its width
_MIN_SHAPE_PTS = 6  # fewer points than this: shape is unmeasurable


def foot_diag(xy):
    if len(xy) == 0:
        return 0.0
    return float(np.hypot(xy[:, 0].max() - xy[:, 0].min(),
                          xy[:, 1].max() - xy[:, 1].min()))


def pca_extents(xy):
    if len(xy) < 2:
        return 0.0, 0.0
    c = xy.mean(axis=0)
    cen = xy - c
    cov = np.cov(cen.T)
    _, evec = np.linalg.eigh(cov)
    proj = cen @ evec
    span = proj.max(axis=0) - proj.min(axis=0)
    return float(max(span)), float(min(span))


def circle_roundness(xy):
    """Kasa circle fit. Returns (radius, radial_rms / radius). Low ratio => round."""
    if len(xy) < 3:
        return 0.0, 1.0
    x, y = xy[:, 0], xy[:, 1]
    A = np.c_[2 * x, 2 * y, np.ones(len(x))]
    b = x ** 2 + y ** 2
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    cx, cy, c = sol
    r = float(np.sqrt(max(c + cx ** 2 + cy ** 2, 1e-9)))
    d = np.hypot(x - cx, y - cy)
    rms = float(np.sqrt(np.mean((d - r) ** 2)))
    return r, rms / max(r, 1e-3)


def post_width(points):
    """Footprint diagonal of the near-base band [z0, z0+LOW_BAND). Isolates the
    post/base from a head or canopy above. 0.0 if too few points."""
    if points is None or len(points) < _MIN_SHAPE_PTS:
        return 0.0
    z0 = float(points[:, 2].min())
    band = points[(points[:, 2] >= z0) & (points[:, 2] < z0 + LOW_BAND)]
    if len(band) < _BAND_MIN_PTS:
        return 0.0
    return foot_diag(band[:, :2])


def has_thin_high_band(points):
    """True if a thin (foot_diag < THIN_DIAG) band exists above HIGH_Z. This is the
    lamp signature: a skinny post rising high. Nothing else in the park does this."""
    if points is None or len(points) < _MIN_SHAPE_PTS:
        return False
    top = float(points[:, 2].max())
    z = HIGH_Z
    while z < top:
        band = points[(points[:, 2] >= z) & (points[:, 2] < z + _BAND)]
        if len(band) >= _BAND_MIN_PTS and foot_diag(band[:, :2]) < THIN_DIAG:
            return True
        z += _BAND
    return False


def foot_extents(points):
    """PCA extents (major, minor) over the full footprint. 0,0 if too few points."""
    if points is None or len(points) < _MIN_SHAPE_PTS:
        return 0.0, 0.0
    return pca_extents(points[:, :2])
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd <worktree> && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_shapefeat.py -v`
Expected: PASS (8 tests).

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/shapefeat.py landmark_loc/tests/test_shapefeat.py
git commit -m "feat(shapefeat): viewpoint-stable shape features for classification"
```

---

## Task 2: Shape-signature classification rule in `classify.py`

**Files:**
- Modify: `landmark_loc/classify.py` (replace `_matches` + `classify_cluster` body; keep signatures)
- Test: `landmark_loc/tests/test_classify.py` (migrate + extend — Task 3 covers test migration; this task adds the minimal new-rule tests inline and keeps the module importable)

**Interfaces:**
- Consumes (from Task 1): `shapefeat.post_width`, `has_thin_high_band`, `foot_extents`, `circle_roundness`, `_MIN_SHAPE_PTS`.
- Produces: `classify_cluster(cluster, margins=DEFAULT_MARGINS) -> str` (one of the five labels or `"unknown"`), unchanged signature. `to_observations` unchanged.

**Decision rule (ordered, first match wins):**
1. `_is_tree(cluster)` → `"tree"` (unchanged gate, wins first).
2. no points / `< _MIN_SHAPE_PTS` → `"unknown"`.
3. `shapefeat.has_thin_high_band(points)` AND `post_width(points) < _LAMP_POST_MAX` → `"lamp"`.
4. `height < _BIN_MAX_H` AND `_BIN_FOOT_MIN <= foot_major < _BIN_FOOT_MAX` AND not thin-high → `"trash_bin_1"`.
5. `height < _BOX_MAX_H` AND `foot_major >= _TABLE_MAJOR_MIN` → `"garden_table"`.
6. `height < _BOX_MAX_H` AND `_BENCH_MAJOR_MIN <= foot_major < _TABLE_MAJOR_MIN` → `"bench"`.
7. else `"unknown"`.

Thresholds (seed values, measured; pinned in-sim per plan Testing):

```python
_LAMP_POST_MAX = 0.35   # m: lamp post foot_diag ~0.14; captured lamps 0.12-0.51 low
_BIN_MAX_H     = 1.4    # m: bin height 1.04; lamp 3.15 excluded by has_thin_high_band anyway
_BIN_FOOT_MIN  = 0.30   # m: bin foot_major ~0.68; keep off sub-0.3 noise fragments
_BIN_FOOT_MAX  = 1.20   # m: below bench 1.78 major, above bin 0.68 with margin
_BOX_MAX_H     = 1.40   # m: bench 0.94, table 1.09 both under this
_BENCH_MAJOR_MIN = 1.20 # m: bench major 1.78; near-edge foreshortening floor
_TABLE_MAJOR_MIN = 2.30 # m: table major 3.00; splits table (>=2.3) from bench (<2.3)
```

- [ ] **Step 1: Write the failing tests** (append to `test_classify.py`)

```python
# --- shape-rule tests (Task 2) ---
import numpy as np
from landmark_loc.segment import Cluster


def _pts_cluster(points):
    xy = points[:, :2]
    from landmark_loc.segment import _pca_extents
    major, minor = _pca_extents(xy)
    return Cluster(points=points, centroid_xy=(float(xy[:, 0].mean()), float(xy[:, 1].mean())),
                   major=float(major), minor=float(minor),
                   height=float(points[:, 2].max() - points[:, 2].min()))


def _lamp_post(height=2.5, w=0.14, n=40):
    pts = []
    for z in np.linspace(0.0, height, n):
        for a in np.linspace(0, 2 * np.pi, 6, endpoint=False):
            pts.append((0.5 * w * np.cos(a), 0.5 * w * np.sin(a), z))
    return np.array(pts, float)


def _box(major, minor, height, n=8):
    pts = []
    for z in np.linspace(0.0, height, 4):
        for x in np.linspace(-major / 2, major / 2, n):
            for y in (-minor / 2, minor / 2):
                pts.append((x, y, z))
    return np.array(pts, float)


def test_thin_tall_post_classifies_lamp():
    assert classify.classify_cluster(_pts_cluster(_lamp_post())) == "lamp"


def test_short_oblong_box_classifies_bin():
    # bin: 0.68 x 0.38 x 1.04, no tall thin post
    assert classify.classify_cluster(_pts_cluster(_box(0.68, 0.38, 1.04))) == "trash_bin_1"


def test_medium_low_box_classifies_bench():
    assert classify.classify_cluster(_pts_cluster(_box(1.78, 0.80, 0.9))) == "bench"


def test_long_low_box_classifies_table():
    assert classify.classify_cluster(_pts_cluster(_box(3.0, 1.32, 1.05))) == "garden_table"


def test_pole_fragment_is_not_bin():
    # a tiny pole fragment (the OLD phantom) must NOT be trash_bin now
    frag = _lamp_post(height=0.5, w=0.10, n=8)
    assert classify.classify_cluster(_pts_cluster(frag)) != "trash_bin_1"


def test_no_points_is_unknown():
    c = Cluster(points=None, centroid_xy=(1, 2), major=0.7, minor=0.4, height=1.0)
    assert classify.classify_cluster(c) == "unknown"
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd <worktree> && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_classify.py -k "post or box or fragment or no_points" -v`
Expected: FAIL (old size-band `_matches` mislabels or returns unknown).

- [ ] **Step 3: Rewrite the decision in `classify.py`**

Replace `_matches` and the `classify_cluster` body. Keep `_is_tree`, `_trunk_xy`, `_band_width`, `to_observations`, `Observation`, `KNOWN_RADIUS`, `_RECT_FOOTPRINT`, `DEFAULT_MARGINS` (still passed through for signature compat, even though the new rule keys on shape). Add the threshold constants above. New body:

```python
from landmark_loc import shapefeat

def classify_cluster(cluster, margins=DEFAULT_MARGINS):
    # Tree wins first: a real trunk+canopy must be excluded from the other types.
    if _is_tree(cluster, margins):
        return "tree"
    pts = cluster.points
    if pts is None or len(pts) < shapefeat._MIN_SHAPE_PTS:
        return "unknown"
    # lamp: the only object with a thin post rising above HIGH_Z.
    if shapefeat.has_thin_high_band(pts) and shapefeat.post_width(pts) < _LAMP_POST_MAX:
        return "lamp"
    foot_major, _foot_minor = shapefeat.foot_extents(pts)
    height = cluster.height
    # trash_bin: short compact oblong box, no tall thin post.
    if height < _BIN_MAX_H and _BIN_FOOT_MIN <= foot_major < _BIN_FOOT_MAX:
        return "trash_bin_1"
    # garden_table: low box, long.
    if height < _BOX_MAX_H and foot_major >= _TABLE_MAJOR_MIN:
        return "garden_table"
    # bench: low box, medium length.
    if height < _BOX_MAX_H and _BENCH_MAJOR_MIN <= foot_major < _TABLE_MAJOR_MIN:
        return "bench"
    return "unknown"
```

Note: `_matches` is deleted. If any test imports `_matches` directly, migrate it in Task 3.

- [ ] **Step 4: Run the new tests to verify they pass**

Run: `cd <worktree> && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_classify.py -k "post or box or fragment or no_points" -v`
Expected: PASS (6 tests). Note: OTHER pre-existing tests in this file may now fail — that is expected and fixed in Task 3.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/classify.py landmark_loc/tests/test_classify.py
git commit -m "feat(classify): shape-signature rule replaces size-band matching"
```

---

## Task 3: Migrate the existing `test_classify.py` size-band tests to point-based fixtures

**Files:**
- Modify: `landmark_loc/tests/test_classify.py`

**Why:** The old tests build `Cluster(points=None, ...)` and pass only major/minor/height (e.g. `test_classifies_each_type_from_ideal_dims`, `test_ideal_bin_is_still_bin`, `test_ideal_lamp_is_still_lamp`, `test_ambiguous_between_bands_is_unknown`, `test_to_observations_emits_tree_drops_unknown`). The new rule needs `points`, so these must be rebuilt as point clouds of the intended shape, or removed where they encode the OLD size-band contract that no longer holds. The tree tests (`_canopy_cluster`, `test_wide_canopy_over_trunk_is_tree`, `test_thin_tall_pole_is_not_tree`, `test_low_wide_object_is_not_tree`) already use `points` and MUST still pass unchanged.

**Interfaces:**
- Consumes: `_pts_cluster`, `_lamp_post`, `_box` helpers from Task 2's additions.

- [ ] **Step 1: Inventory which tests break**

Run: `cd <worktree> && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_classify.py -v`
Expected: the `points=None` size-only tests FAIL; the point-based tree tests and the Task-2 shape tests PASS. Record the failing names.

- [ ] **Step 2: Migrate each failing test to a point-based fixture**

For each `points=None` size-only test, rebuild the cluster from a point cloud of the intended shape using `_box(...)` / `_lamp_post(...)`:

```python
def test_classifies_each_type_from_shape():
    assert classify.classify_cluster(_pts_cluster(_lamp_post())) == "lamp"
    assert classify.classify_cluster(_pts_cluster(_box(0.68, 0.38, 1.04))) == "trash_bin_1"
    assert classify.classify_cluster(_pts_cluster(_box(1.78, 0.80, 0.9))) == "bench"
    assert classify.classify_cluster(_pts_cluster(_box(3.0, 1.32, 1.05))) == "garden_table"


def test_to_observations_emits_tree_drops_unknown():
    clusters = [
        _pts_cluster(_box(1.78, 0.80, 0.9)),     # bench -> emitted
        _canopy_cluster(),                        # tree  -> emitted
        _pts_cluster(_box(1.9, 1.3, 0.9)),        # ambiguous mid-size -> unknown, dropped
    ]
    obs = to_observations(clusters)
    idents = sorted(o.identity for o in obs)
    assert "tree" in idents and "bench" in idents
    assert "unknown" not in idents
```

Delete tests that assert the OLD size-band contract specifically (e.g. `test_ambiguous_between_bands_is_unknown` keyed to bench/table *aspect bands*) and REPLACE with the shape-equivalent (a mid-size box between bench and table `foot_major` bands → `unknown`). If any test imports `_matches`, remove that import and rewrite via `classify_cluster`.

- [ ] **Step 3: Run the full file to verify all pass**

Run: `cd <worktree> && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_classify.py -v`
Expected: PASS (all).

- [ ] **Step 4: Run the whole suite for regressions**

Run: `cd <worktree> && PYTHONPATH=. python3 -m pytest landmark_loc/tests/ -q`
Expected: PASS except the one known pre-existing unrelated failure (`test_launch.py::test_runbook_offers_both_modes`, present before this branch). Confirm no NEW failures.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/tests/test_classify.py
git commit -m "test(classify): migrate size-band tests to point-based shape fixtures"
```

---

## Task 4: Regression test against the 15 captured real clusters

**Files:**
- Create: `landmark_loc/tests/test_captured_regression.py`
- Uses: `landmark_loc/tests/fixtures/captured_clusters.npz` + `captured_clusters.json` (already committed data)

**Why:** This is the concrete before/after the whole redesign exists for. The old classifier dropped 12/15 as `unknown`, including 4 obvious lamps. This test asserts the shape rule recovers them.

**Interfaces:**
- Consumes: `segment.Cluster`, `classify.classify_cluster`, the fixture files.

**Expected labels (human-verified from shape this session):** cluster [10] → lamp; [0],[4],[11] → lamp; [14] → lamp; [12] → bench; [13] → trash_bin_1. Trees [2],[6],[8] and far/ambiguous fragments are NOT asserted positively (the tree gate at range is out of scope); they are only asserted to NOT be mislabeled as a *furniture* type (no phantoms).

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_captured_regression.py
import json
import os
import numpy as np
import pytest
from landmark_loc.segment import Cluster, _pca_extents
from landmark_loc import classify

_HERE = os.path.dirname(__file__)
_NPZ = os.path.join(_HERE, "fixtures", "captured_clusters.npz")
_JSON = os.path.join(_HERE, "fixtures", "captured_clusters.json")


def _load():
    arrs = np.load(_NPZ)
    manifest = {m["i"]: m for m in json.load(open(_JSON))}
    out = {}
    for key in arrs.files:
        i = int(key[1:])
        pts = arrs[key].astype(float)
        major, minor = _pca_extents(pts[:, :2])
        out[i] = Cluster(points=pts,
                         centroid_xy=(float(pts[:, 0].mean()), float(pts[:, 1].mean())),
                         major=float(major), minor=float(minor),
                         height=float(pts[:, 2].max() - pts[:, 2].min()))
    return out, manifest


# clusters the OLD classifier dropped that MUST now classify correctly
_MUST = {0: "lamp", 4: "lamp", 10: "lamp", 11: "lamp", 14: "lamp",
         12: "bench", 13: "trash_bin_1"}
# fragments/ground blobs that must NOT become a furniture phantom
_NO_PHANTOM = (1, 3, 9)
_FURNITURE = {"lamp", "trash_bin_1", "bench", "garden_table"}


@pytest.mark.parametrize("i,expected", sorted(_MUST.items()))
def test_captured_cluster_labeled_correctly(i, expected):
    clusters, _ = _load()
    assert classify.classify_cluster(clusters[i]) == expected


@pytest.mark.parametrize("i", _NO_PHANTOM)
def test_captured_fragment_not_phantom_furniture(i):
    clusters, _ = _load()
    got = classify.classify_cluster(clusters[i])
    assert got not in _FURNITURE, f"cluster {i} became phantom {got}"


def test_unknown_rate_dropped():
    clusters, _ = _load()
    labels = [classify.classify_cluster(c) for c in clusters.values()]
    unknown = sum(1 for l in labels if l == "unknown")
    # OLD: 12/15 unknown. New rule must do much better on this frame.
    assert unknown <= 6, f"{unknown}/15 still unknown"
```

- [ ] **Step 2: Run to verify current behavior**

Run: `cd <worktree> && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_captured_regression.py -v`
Expected AFTER Tasks 1-2: PASS. (If run before Task 2, it fails — that is the point.) If any `_MUST` cluster does not classify as expected, that is a real signal the thresholds need pinning — adjust the seed constants in `classify.py` (documenting the captured-cluster value that drove the change) and re-run. Do NOT loosen a threshold so far it readmits a `_NO_PHANTOM` fragment.

- [ ] **Step 3: Commit**

```bash
git add landmark_loc/tests/test_captured_regression.py
git commit -m "test(classify): regression on 15 captured real clusters (recovers dropped lamps)"
```

---

## Task 5: Documentation + in-sim pinning note

**Files:**
- Modify: `landmark_loc/classify.py` (module docstring), `docs/superpowers/specs/2026-08-13-shape-classifier-design.md` (mark implemented)

- [ ] **Step 1: Update the `classify.py` module docstring**

Rewrite the top docstring to describe the shape-signature approach (was: "Bands are centered on the mesh-derived signatures … widened by DEFAULT_MARGINS"). New text states: classification is by viewpoint-stable shape features (`shapefeat`), ordered tree→lamp→bin→table→bench→unknown, with thresholds measured from meshes + captured clusters and pinned in-sim.

- [ ] **Step 2: Add an in-sim pinning checklist to the design doc**

Under Testing, add a short "In-sim pinning (main runs, requires user go-ahead)" note: run RUN-MAP-NAV Steps 0-3 verbatim from a clean kill; capture the stale-diag `unknown` fraction; confirm no phantoms; confirm the robot reaches the goal in Gazebo after GPS spoof + landmark switch; if a real object is still dropped or a phantom admitted, adjust the seed thresholds and record the measured value. This step is NOT run by the implementer — it is main-conversation work gated on user consent.

- [ ] **Step 3: Commit**

```bash
git add landmark_loc/classify.py docs/superpowers/specs/2026-08-13-shape-classifier-design.md
git commit -m "docs(classify): document shape-signature classifier + in-sim pinning"
```

---

## Self-Review

**Spec coverage:** shape features (Task 1), the ordered shape rule keeping all 5 labels (Task 2), test migration off `points=None` (Task 3), captured-cluster regression proving the fix (Task 4), docs + pinning gate (Task 5). Matcher/catalog/shapefit/localizer untouched per Global Constraints. ✓

**Placeholder scan:** thresholds are concrete named constants with measured comments, not TBDs. Test code is complete, not "write tests for the above." ✓

**Type consistency:** `classify_cluster(cluster, margins=...)->str` unchanged; `shapefeat` function names match between Task 1 interfaces, Task 2 calls, and Task 4. `Cluster(points, centroid_xy, major, minor, height)` matches `segment.py`. `_pca_extents` reused from `segment.py` in tests; `shapefeat.pca_extents` is the module-internal mirror (documented as identical). ✓

**Known caveat surfaced, not hidden:** thresholds are seeds from ONE captured frame + meshes; Task 4 Step 2 and Task 5 make pinning an explicit, user-gated step. In-sim acceptance is Gazebo-judged and needs a real sim run the implementer does not perform.
