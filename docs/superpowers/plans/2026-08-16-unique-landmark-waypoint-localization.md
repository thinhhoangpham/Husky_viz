# Unique-landmark + waypoint localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Localize the Husky from *distinctive* landmarks (power-line poles) identified by an NDT-style shape descriptor with no classifier, re-anchoring the dead-reckoning prior on descriptor-confirmed operator-waypoint arrivals.

**Architecture:** An offline step adds `linea1` structures to `park.world`, assembles one map-frame point cloud from all placed meshes, and measures **per-region** distinctiveness over a grid of candidate locations — no object segmentation, no types. A descriptor module (pure numpy, shared by extraction and runtime) turns the points in a spatial window into voxel-shape statistics over height bands **plus horizontal-arrangement sectors**. A detector plugin describes the region around the robot's prior and matches it to distinctive map locations only, gated by the prior. The localizer's anchor becomes the most recent region match or descriptor-confirmed waypoint arrival.

> **REVISED 2026-08-16 — per-region pivot.** The original plan (Tasks 1–13 below) built *per-object* descriptors. That broke on the real map: the distinctive structures are byte-identical instances of one mesh, so per-object descriptors are distance 0 apart and none is distinctive. Per the revised spec, the unit is now a **region of space at a chosen scale**, with no notion of object or type. Tasks 1–7 and 10 stand as completed and are reused. Tasks 8, 9, 11, 12 are **superseded** by the per-region tasks **T14–T21** in the "Per-region tasks" section at the end of this plan — implement those, not the originals. Task 13 (in-sim) is replaced by T21.

**Tech Stack:** Python 3.8, numpy, ROS Noetic (rospy), pytest. No new third-party dependencies.

**Spec:** `docs/superpowers/specs/2026-08-16-unique-landmark-waypoint-localization-design.md` — read it alongside this plan.

## Global Constraints

- **No ground truth, ever.** No `/gazebo/model_states`, no `gazebo_msgs`, no constant measured from simulator internals. Pose comes from GPS/compass/odometry + landmarks only.
- **GPU-ray lidar only.** Never switch the Ouster to CPU ray (causes stepping). This is why lidar intensity is unusable (verified constant 0.0) — the descriptor is geometry-only.
- **Single type registry.** `map_tools/park_types.py` is the one source for both map extraction and runtime; never hardcode per-type numbers elsewhere.
- **Descriptor computed identically from mesh-sampled points and lidar points.** One function, shared, so the two sides cannot drift.
- **Files pinned to repo root do not move** (see CLAUDE.md). New standalone tools go in `scripts/`; package code stays in its package.
- **Subagents implement + pytest + commit only; they never run the simulator.** In-sim validation is done by the main conversation from a clean kill.
- **Arrival is confirmed by descriptor match, never by move_base status or fused pose.**
- **Pole = landmark unit**, not the model or the span. One `linea1` model = two poles via `POLE_OFFSETS = ((-26.489, 1.251), (2.311, 1.251))` at world scale 0.03.
- Reuse existing seams: detector plugin registers in `landmark_loc/detector.py:225` `DETECTORS`; selected via `~classifier`. Do not delete `classify.py` / `constellation.py`.

---

## File Structure

**New files:**
- `landmark_loc/descriptor.py` — points → NDT voxel-shape descriptor; descriptor distance. Pure numpy, no ROS. Shared by extraction and runtime.
- `map_tools/mesh_sample.py` — sample points across mesh faces (surface, not vertices).
- `landmark_loc/distinctiveness.py` — offline nearest-neighbour scoring over a descriptor map; marks unique anchors.
- `landmark_loc/anchor_detector.py` — detector plugin: cluster → descriptor → match to unique anchors only.
- `landmark_loc/waypoint_anchor.py` — pure logic: given predicted pose, a fresh pole sighting, and a pending arrival, decide the new anchor and any fault signal. No ROS.
- Tests: `landmark_loc/tests/test_descriptor.py`, `test_distinctiveness.py`, `test_anchor_detector.py`, `test_waypoint_anchor.py`; `map_tools/tests/test_mesh_sample.py`.

**Modified files:**
- `natural_environments_ros_opt/natural_enviroment/worlds/park.world` — add 3 `postescable` models (state block + model definitions).
- `load-park-world.sh:240` — add `models_lake_opt` to `GAZEBO_MODEL_PATH` for the park world (or copy `linea1` into `models_opt`).
- `map_tools/park_types.py` — add a `postescable` PARK_TYPES entry.
- `map_tools/extract_park_map.py` — pole expansion (reuse lake `_expand_poles`/`POLE_OFFSETS`); emit `park_descriptors.yaml`.
- `landmark_loc/localizer_node.py` — anchor source becomes pole-sighting / confirmed-waypoint driven.
- `operator/operate.py` — waypoint sequence + arrival confirmation + fault reporting.

**Build order rationale:** descriptor math first (pure, fully unit-testable, no sim, no world edit), then the map it consumes, then the world it describes, then the runtime wiring, then the operator. Each task leaves the repo green.

---

### Task 1: Voxel shape classification

**Files:**
- Create: `landmark_loc/descriptor.py`
- Test: `landmark_loc/tests/test_descriptor.py`

**Interfaces:**
- Consumes: nothing (numpy only).
- Produces:
  - `voxel_shape(points)` → `(linearity, planarity, sphericity)` floats in [0,1] summing to 1.0, from the covariance eigenvalues λ₁≥λ₂≥λ₃ of an (N,3) array. Degenerate (N<3 or zero-variance) → `(0.0, 0.0, 1.0)`.

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_descriptor.py
import numpy as np
from landmark_loc.descriptor import voxel_shape


def _rng(seed):
    return np.random.RandomState(seed)


def test_linear_points_classify_linear():
    # A thin stick along x: large spread in x, tiny in y/z.
    p = _rng(0).randn(200, 3) * np.array([1.0, 0.01, 0.01])
    lin, pla, sph = voxel_shape(p)
    assert lin > 0.8
    assert lin > pla and lin > sph
    assert abs((lin + pla + sph) - 1.0) < 1e-6


def test_planar_points_classify_planar():
    # A sheet in x-y: spread in x and y, tiny in z.
    p = _rng(1).randn(200, 3) * np.array([1.0, 1.0, 0.01])
    lin, pla, sph = voxel_shape(p)
    assert pla > 0.5
    assert pla > lin and pla > sph


def test_isotropic_points_classify_spherical():
    p = _rng(2).randn(200, 3)
    lin, pla, sph = voxel_shape(p)
    assert sph > 0.4
    assert sph >= lin and sph >= pla


def test_degenerate_is_spherical_default():
    assert voxel_shape(np.zeros((2, 3))) == (0.0, 0.0, 1.0)
    assert voxel_shape(np.zeros((0, 3))) == (0.0, 0.0, 1.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_descriptor.py -v`
Expected: FAIL — `ModuleNotFoundError` / `voxel_shape` undefined.

- [ ] **Step 3: Write minimal implementation**

```python
# landmark_loc/descriptor.py
"""NDT-style shape descriptor: points -> per-height-band voxel-shape statistics.

Pure numpy, no ROS. Shared by map extraction (mesh-sampled points) and the
runtime detector (lidar points) so the two sides cannot drift. See the design
doc's "Descriptor" section: local shape from covariance eigenvalue RATIOS is a
LOCAL property, so a near-face voxel classifies correctly without the far side.
"""
import numpy as np


def voxel_shape(points):
    """Return (linearity, planarity, sphericity) for an (N,3) point set.

    From the covariance eigenvalues l1>=l2>=l3 (Demantke et al.):
        linearity   = (l1 - l2) / l1
        planarity   = (l2 - l3) / l1
        sphericity  =  l3       / l1
    Normalised to sum to 1. Degenerate input -> (0, 0, 1).
    """
    p = np.asarray(points, dtype=float)
    if p.shape[0] < 3:
        return (0.0, 0.0, 1.0)
    cov = np.cov((p - p.mean(axis=0)).T)
    evals = np.linalg.eigvalsh(cov)  # ascending
    l3, l2, l1 = float(evals[0]), float(evals[1]), float(evals[2])
    if l1 <= 1e-12:
        return (0.0, 0.0, 1.0)
    lin = (l1 - l2) / l1
    pla = (l2 - l3) / l1
    sph = l3 / l1
    s = lin + pla + sph
    if s <= 1e-12:
        return (0.0, 0.0, 1.0)
    return (lin / s, pla / s, sph / s)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_descriptor.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/descriptor.py landmark_loc/tests/test_descriptor.py
git commit -m "feat(descriptor): voxel shape classification from covariance eigenvalues"
```

---

### Task 2: Height-band descriptor vector

**Files:**
- Modify: `landmark_loc/descriptor.py`
- Test: `landmark_loc/tests/test_descriptor.py`

**Interfaces:**
- Consumes: `voxel_shape` (Task 1).
- Produces:
  - `describe(points, band_height=1.0, voxel=0.5, min_voxel_pts=5, n_bands=18)` → a numpy array of shape `(n_bands, 4)`: per band `[mean_linearity, mean_planarity, mean_sphericity, horizontal_extent]`. Bands index height above the cluster's own minimum z. Empty bands are all-zero.
  - `descriptor_distance(a, b)` → float L2 distance over the flattened `(n_bands, 4)` arrays, with the extent column scaled by `EXTENT_WEIGHT = 0.25` so shape dominates size.

- [ ] **Step 1: Write the failing test**

```python
# append to landmark_loc/tests/test_descriptor.py
from landmark_loc.descriptor import describe, descriptor_distance


def _lattice_pole(seed, height=16.0):
    # Criss-crossing thin members over 0..height: many linear voxels.
    r = _rng(seed)
    zs = r.uniform(0, height, 4000)
    xs = r.choice([-0.25, 0.25], 4000) + r.randn(4000) * 0.02
    ys = r.randn(4000) * 0.02
    a = np.column_stack([xs, ys, zs])
    b = np.column_stack([ys, xs, zs])  # members in the other orientation
    return np.vstack([a, b])


def _bench():
    r = _rng(9)
    xs = r.uniform(-0.9, 0.9, 800)
    ys = r.uniform(-0.4, 0.4, 800)
    zs = r.uniform(0.0, 0.94, 800)
    return np.column_stack([xs, ys, zs])


def test_describe_shape_and_bands():
    d = describe(_lattice_pole(3))
    assert d.shape == (18, 4)
    # a tall pole has non-empty high bands; a bench would not
    assert d[14, :3].sum() > 0  # ~14-15 m band has material


def test_pole_far_from_bench():
    dp = describe(_lattice_pole(3))
    db = describe(_bench())
    dpp = describe(_lattice_pole(4))  # different sampling of the same shape
    # same-shape distance must be much smaller than cross-shape distance
    assert descriptor_distance(dp, dpp) < descriptor_distance(dp, db)
    assert descriptor_distance(dp, db) > 1.0


def test_empty_bands_are_zero():
    d = describe(_bench())
    # bench has no material above ~1 m -> bands 2..17 all zero
    assert np.allclose(d[2:], 0.0)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_descriptor.py -v`
Expected: FAIL — `describe` / `descriptor_distance` undefined.

- [ ] **Step 3: Write minimal implementation**

```python
# append to landmark_loc/descriptor.py

EXTENT_WEIGHT = 0.25


def describe(points, band_height=1.0, voxel=0.5, min_voxel_pts=5, n_bands=18):
    """Per-height-band voxel-shape descriptor. Shape (n_bands, 4).

    Bands are measured above the cluster's OWN minimum z (so ground offset
    does not matter). In each band, points are bucketed into `voxel`-sized
    x/y cells; each cell with >= min_voxel_pts contributes a voxel_shape, and
    the band records the MEAN shape over its voxels plus the band's horizontal
    extent. Bands with no qualifying voxel are all-zero.
    """
    out = np.zeros((n_bands, 4), dtype=float)
    p = np.asarray(points, dtype=float)
    if p.shape[0] == 0:
        return out
    z0 = p[:, 2].min()
    for bi in range(n_bands):
        lo = z0 + bi * band_height
        hi = lo + band_height
        band = p[(p[:, 2] >= lo) & (p[:, 2] < hi)]
        if band.shape[0] == 0:
            continue
        out[bi, 3] = float(np.hypot(
            band[:, 0].max() - band[:, 0].min(),
            band[:, 1].max() - band[:, 1].min()))
        cells = np.floor(band[:, :2] / voxel).astype(int)
        buckets = {}
        for i, key in enumerate(map(tuple, cells)):
            buckets.setdefault(key, []).append(i)
        shapes = [voxel_shape(band[idxs])
                  for idxs in buckets.values() if len(idxs) >= min_voxel_pts]
        if shapes:
            out[bi, :3] = np.mean(shapes, axis=0)
    return out


def descriptor_distance(a, b):
    """Weighted L2 over flattened (n_bands, 4) descriptors; extent down-weighted."""
    a = np.array(a, dtype=float, copy=True)
    b = np.array(b, dtype=float, copy=True)
    a[:, 3] *= EXTENT_WEIGHT
    b[:, 3] *= EXTENT_WEIGHT
    return float(np.linalg.norm(a.ravel() - b.ravel()))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_descriptor.py -v`
Expected: PASS (7 tests).

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/descriptor.py landmark_loc/tests/test_descriptor.py
git commit -m "feat(descriptor): per-height-band descriptor vector + weighted distance"
```

---

### Task 3: Descriptor is stable under decimation and partial views

**Files:**
- Test: `landmark_loc/tests/test_descriptor.py`

**Interfaces:**
- Consumes: `describe`, `descriptor_distance` (Task 2).
- Produces: nothing (characterization test; the property the design relies on).

This task adds no code — it *pins* the mesh-vs-partial and range-sparsity robustness the descriptor exists to provide. If it fails, Task 2's parameters (`voxel`, `min_voxel_pts`) need tuning, not the test.

- [ ] **Step 1: Write the test**

```python
# append to landmark_loc/tests/test_descriptor.py
def test_partial_view_matches_full():
    full = _lattice_pole(5)
    # keep only the near HALF (y < 0): a single-face view
    partial = full[full[:, 1] < 0]
    df = describe(full)
    dp = describe(partial)
    db = describe(_bench())
    # partial view still much closer to its full self than to a bench
    assert descriptor_distance(df, dp) < 0.5 * descriptor_distance(df, db)


def test_decimation_stable():
    full = _lattice_pole(6)
    decim = full[::4]  # 1/4 the density, standing in for range
    assert descriptor_distance(describe(full), describe(decim)) \
        < descriptor_distance(describe(full), describe(_bench()))
```

- [ ] **Step 2: Run and verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_descriptor.py -v`
Expected: PASS. If either fails, tune `voxel`/`min_voxel_pts` in `describe` (larger voxel or lower min tolerates sparsity) until both this task and Task 2 pass, then note the chosen values in a comment.

- [ ] **Step 3: Commit**

```bash
git add landmark_loc/tests/test_descriptor.py landmark_loc/descriptor.py
git commit -m "test(descriptor): pin partial-view and decimation robustness"
```

---

### Task 4: Mesh surface sampler

**Files:**
- Create: `map_tools/mesh_sample.py`
- Create: `map_tools/tests/__init__.py` (empty, if absent)
- Test: `map_tools/tests/test_mesh_sample.py`

**Interfaces:**
- Consumes: `map_tools.mesh_bounds` (existing COLLADA loader — reuse its triangle access; read `mesh_bounds.py` first to see how it loads faces).
- Produces:
  - `sample_surface(dae_path, scale, n=4000, seed=0)` → (n,3) numpy array of points sampled uniformly by triangle area over the mesh surface, in mesh-local metres at `scale`. Deterministic given `seed`.

**Note:** read `map_tools/mesh_bounds.py` before writing this — it already parses the COLLADA triangles and applies node transforms + scale. Reuse that face extraction; do NOT re-parse COLLADA from scratch. If `mesh_bounds` exposes only bounds, add a small internal `_triangles(dae_path, scale)` helper there and import it.

- [ ] **Step 1: Write the failing test**

```python
# map_tools/tests/test_mesh_sample.py
import os
import numpy as np
from map_tools.mesh_sample import sample_surface

LINEA1 = os.path.join(os.path.dirname(__file__), "..", "..",
                      "models_lake_opt", "linea1", "postes.dae")


def test_sample_count_and_determinism():
    a = sample_surface(LINEA1, 0.03, n=2000, seed=0)
    b = sample_surface(LINEA1, 0.03, n=2000, seed=0)
    assert a.shape == (2000, 3)
    assert np.array_equal(a, b)


def test_sample_within_mesh_bounds():
    from map_tools.mesh_bounds import bounds3d
    pts = sample_surface(LINEA1, 0.03, n=2000, seed=1)
    (xmin, xmax), (ymin, ymax), (zmin, zmax) = bounds3d(LINEA1, 0.03)
    assert xmin - 0.1 <= pts[:, 0].min() and pts[:, 0].max() <= xmax + 0.1
    assert zmin - 0.1 <= pts[:, 2].min() and pts[:, 2].max() <= zmax + 0.1
    # the pole mesh is ~16.5 m tall at this scale
    assert pts[:, 2].max() - pts[:, 2].min() > 12.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_mesh_sample.py -v`
Expected: FAIL — module/`sample_surface` undefined. (Confirm `bounds3d` signature against `map_tools/mesh_bounds.py` first; adjust the unpack if it returns a different shape.)

- [ ] **Step 3: Write minimal implementation**

```python
# map_tools/mesh_sample.py
"""Sample points across mesh SURFACES (area-weighted), not raw vertices.

The descriptor must see a laser-like point set. Raw COLLADA vertices cluster
where the modeller added detail; area-weighted surface samples approximate what
a lidar sweep returns. Reuses mesh_bounds' triangle extraction so scale and
COLLADA node transforms are applied exactly as the rest of map_tools does.
"""
import numpy as np

from map_tools.mesh_bounds import _triangles  # (M,3,3) triangles at scale


def sample_surface(dae_path, scale, n=4000, seed=0):
    tris = np.asarray(_triangles(dae_path, scale), dtype=float)  # (M,3,3)
    if len(tris) == 0:
        return np.zeros((0, 3))
    v0, v1, v2 = tris[:, 0], tris[:, 1], tris[:, 2]
    areas = 0.5 * np.linalg.norm(np.cross(v1 - v0, v2 - v0), axis=1)
    total = areas.sum()
    if total <= 0:
        return np.zeros((0, 3))
    rng = np.random.RandomState(seed)
    tri_idx = rng.choice(len(tris), size=n, p=areas / total)
    u = rng.rand(n, 1)
    w = rng.rand(n, 1)
    over = (u + w > 1).ravel()
    u[over] = 1 - u[over]
    w[over] = 1 - w[over]
    a = v0[tri_idx]
    return a + u * (v1[tri_idx] - a) + w * (v2[tri_idx] - a)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_mesh_sample.py -v`
Expected: PASS. If `_triangles` does not exist in `mesh_bounds`, add it there (extract the triangle list the existing `bounds3d`/`footprint` already walk) and commit that with this task.

- [ ] **Step 5: Commit**

```bash
git add map_tools/mesh_sample.py map_tools/tests/test_mesh_sample.py map_tools/mesh_bounds.py
git commit -m "feat(map): area-weighted mesh surface sampler for descriptors"
```

---

### Task 5: Distinctiveness scoring

**Files:**
- Create: `landmark_loc/distinctiveness.py`
- Test: `landmark_loc/tests/test_distinctiveness.py`

**Interfaces:**
- Consumes: `descriptor_distance` (Task 2).
- Produces:
  - `nearest_distances(descriptors)` where `descriptors` is `{name: (n_bands,4) array}` → `{name: float}` distance to the closest *other* descriptor.
  - `unique_names(descriptors, threshold)` → set of names whose nearest-other distance ≥ `threshold`.

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_distinctiveness.py
import numpy as np
from landmark_loc.distinctiveness import nearest_distances, unique_names


def _d(*rows):
    a = np.zeros((18, 4))
    for bi, val in rows:
        a[bi, 0] = val
    return a


def test_repeated_shapes_score_low():
    # three identical "benches", one tall "pole"
    descs = {
        "bench_a": _d((0, 1.0)),
        "bench_b": _d((0, 1.0)),
        "bench_c": _d((0, 1.0)),
        "pole": _d((0, 1.0), (14, 1.0)),
    }
    nd = nearest_distances(descs)
    assert nd["bench_a"] == 0.0  # identical twin exists
    assert nd["pole"] > 0.5      # nothing like it
    uniq = unique_names(descs, threshold=0.5)
    assert uniq == {"pole"}


def test_single_entry_has_no_neighbour():
    nd = nearest_distances({"only": _d((0, 1.0))})
    assert nd["only"] == float("inf")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_distinctiveness.py -v`
Expected: FAIL — module undefined.

- [ ] **Step 3: Write minimal implementation**

```python
# landmark_loc/distinctiveness.py
"""Offline: which map objects have a shape unlike any other object.

An object whose nearest-OTHER descriptor is far away is a unique anchor. This is
a measurement over the extracted descriptor map, not an assumption: two identical
structures correctly score as non-unique and are excluded.
"""
from landmark_loc.descriptor import descriptor_distance


def nearest_distances(descriptors):
    names = list(descriptors)
    out = {}
    for a in names:
        best = float("inf")
        for b in names:
            if a is b or a == b:
                continue
            d = descriptor_distance(descriptors[a], descriptors[b])
            if d < best:
                best = d
        out[a] = best
    return out


def unique_names(descriptors, threshold):
    nd = nearest_distances(descriptors)
    return {name for name, d in nd.items() if d >= threshold}
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_distinctiveness.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/distinctiveness.py landmark_loc/tests/test_distinctiveness.py
git commit -m "feat(landmark): distinctiveness scoring over a descriptor map"
```

---

### Task 6: `postescable` park type

**Files:**
- Modify: `map_tools/park_types.py` (add one `ParkType` to `PARK_TYPES`)
- Test: `landmark_loc/tests/test_catalog.py` OR a new `map_tools/tests/test_park_types_pole.py`

**Interfaces:**
- Consumes: existing `ParkType` dataclass (`park_types.py:48`), `classify_prefix`.
- Produces: `classify_prefix("postescable_...")` → `"postescable"`; the type is `is_object=True, is_catalog=True`.

Read `park_types.py:188-284` (the `PARK_TYPES` tuple) and the existing lake `postescable` entry (`:322-341`) first — copy that entry's fields, since the asset is identical.

- [ ] **Step 1: Write the failing test**

```python
# map_tools/tests/test_park_types_pole.py
from map_tools.park_types import classify_prefix, BY_PREFIX


def test_postescable_classifies():
    assert classify_prefix("postescable_pole0") == "postescable"
    assert classify_prefix("postescable") == "postescable"


def test_postescable_is_object_and_catalog():
    t = BY_PREFIX["postescable"]
    assert t.is_object and t.is_catalog
    assert t.identity == "postescable"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_park_types_pole.py -v`
Expected: FAIL — `"postescable"` not in `BY_PREFIX` (currently only in `LAKE_BY_PREFIX`).

- [ ] **Step 3: Add the type**

Append to the `PARK_TYPES` tuple in `map_tools/park_types.py` (copy the lake `postescable` entry verbatim — same asset, same numbers):

```python
    ParkType(
        # Power-line pole added to the park as a UNIQUE-shape landmark. Same
        # asset as lake's postescable (linea1/postes.dae). disc_radius 0.25 =
        # one pole; profile type, 0.35 m post-width margin like the lamp.
        world_prefix="postescable", identity="postescable",
        is_object=True, is_catalog=True, disc_radius=0.25,
        mesh=None,
        box_stamped=False, marker_color=(0.6, 0.3, 0.0, 1.0),
        score_family="profile", score_margin=0.35,
        score_floor=0.05,
    ),
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_park_types_pole.py landmark_loc/tests/test_catalog.py -v`
Expected: PASS (and existing catalog tests still green).

- [ ] **Step 5: Commit**

```bash
git add map_tools/park_types.py map_tools/tests/test_park_types_pole.py
git commit -m "feat(map): register postescable as a park landmark type"
```

---

### Task 7: Add power-line poles to park.world

**Files:**
- Modify: `natural_environments_ros_opt/natural_enviroment/worlds/park.world`
- Modify: `load-park-world.sh:240` (GAZEBO_MODEL_PATH)
- Test: `map_tools/tests/test_park_world_poles.py`

**Interfaces:**
- Consumes: `map_tools.sdf_parse.parse_models`.
- Produces: `parse_models(park.world)` returns 3 models with `family == "postescable"` at the three link poses below.

**Model link poses** (from the spec's layout table):
- A1: `(-16.330, 5.284, 0.2094)`
- A2: `(40.012, 17.259, 0.2094)`
- B1: `(-0.511, -24.251, 0.0000)`

Each `postescable` model needs (a) a full `<model>` definition block (copy the structure lake.world uses for `postescable` — two links `link_0`/`link_4`, mesh `model://linea1/postes.dae` + `model://linea1/cables2.dae`, scale 0.03) placed among the other model definitions, AND (b) a matching entry in the `<state world_name='default'>` block with `link_0`'s pose set to the link pose above. `parse_models` reads the STATE block's `link_0` pose, so the state entry is what the extractor sees.

- [ ] **Step 1: Write the failing test**

```python
# map_tools/tests/test_park_world_poles.py
import os
from map_tools.sdf_parse import parse_models

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment",
                     "worlds", "park.world")


def test_three_poles_present():
    models = parse_models(WORLD)
    poles = [m for m in models if m.family == "postescable"]
    assert len(poles) == 3
    got = sorted((round(m.world_x, 2), round(m.world_y, 2)) for m in poles)
    want = sorted([(-16.33, 5.28), (40.01, 17.26), (-0.51, -24.25)])
    assert got == want
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_park_world_poles.py -v`
Expected: FAIL — 0 postescable models found.

- [ ] **Step 3: Edit the world file**

Add the three models. Use the lake.world `postescable` blocks as the template (definition at `lake.world:19762`, state at `lake.world:1827`). For each, set both the model `<pose>` and the state `link_0` `<pose>` to the link pose above (x y z=0 0 0 roll pitch yaw). Add `models_lake_opt` to the park branch of `GAZEBO_MODEL_PATH` in `load-park-world.sh:240` so `model://linea1/...` resolves (or copy `models_lake_opt/linea1` to `models_opt/linea1` and skip the path edit — pick one and note it in the commit).

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_park_world_poles.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add natural_environments_ros_opt/natural_enviroment/worlds/park.world load-park-world.sh map_tools/tests/test_park_world_poles.py
git commit -m "feat(world): add three power-line poles to park.world"
```

---

### Task 8: Extract pole poses + descriptor map

**Files:**
- Modify: `map_tools/extract_park_map.py`
- Test: `map_tools/tests/test_extract_descriptors.py`

**Interfaces:**
- Consumes: `_expand_poles` + `POLE_OFFSETS` (move both from `extract_lake_map.py` to `extract_park_map.py` or a shared spot, since both extractors now need them — read `extract_lake_map.py:76-101`), `mesh_sample.sample_surface` (Task 4), `descriptor.describe` (Task 2), `distinctiveness.unique_names` (Task 5).
- Produces: `main()` additionally writes `maps/park_descriptors.yaml`: for each catalog object `name: {descriptor: [[..4..] x n_bands], unique: bool}`. Poles expanded so each pole is a separate entry.

The descriptor for a pole is computed once from `sample_surface(postes.dae, 0.03)` restricted to one pole's mesh-local x-window (reuse `POLE_OFFSETS` to split), so map-side descriptors come from the same `describe` the runtime uses. Furniture descriptors come from each family's mesh via `park_types` `mesh` where present; families without a mesh (tree, lamp) sample their `.dae` from the registry path.

- [ ] **Step 1: Write the failing test**

```python
# map_tools/tests/test_extract_descriptors.py
import os
import yaml
from map_tools import extract_park_map


def test_descriptor_map_marks_poles_unique(tmp_path):
    extract_park_map.main(["--out-dir", str(tmp_path)])
    with open(tmp_path / "park_descriptors.yaml") as fh:
        d = yaml.safe_load(fh)
    poles = [k for k in d if k.startswith("postescable")]
    assert len(poles) == 6  # 3 models x 2 poles
    assert all(d[p]["unique"] for p in poles)
    # at least one bench exists and is NOT unique (identical twins)
    benches = [k for k in d if k.startswith("bench")]
    assert benches and not any(d[b]["unique"] for b in benches)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_extract_descriptors.py -v`
Expected: FAIL — no `park_descriptors.yaml` written.

- [ ] **Step 3: Implement**

In `extract_park_map.py`: import `_expand_poles`/`POLE_OFFSETS` (relocated to be shared), `sample_surface`, `describe`, `unique_names`. After `build_objects`, build `{name: descriptor}` for every catalog object (poles expanded), compute `unique_names(descs, threshold=DESC_UNIQUE_THRESHOLD)` (set `DESC_UNIQUE_THRESHOLD = 0.5` as a module constant with a comment that it is tuned by Task 3's margins), and write `park_descriptors.yaml`. Keep the existing `.pgm`/`.yaml`/`park_objects.yaml` outputs unchanged, but run `build_grid`/`build_objects` on `_expand_poles(models)` so the poles are stamped and named per-pole.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest map_tools/tests/test_extract_descriptors.py map_tools/tests/test_park_world_poles.py -v`
Expected: PASS.

- [ ] **Step 5: Regenerate the committed maps and commit**

```bash
cd /home/thinh/Documents/Husky_viz && python3 -m map_tools.extract_park_map
git add map_tools/extract_park_map.py map_tools/extract_lake_map.py maps/park_map.pgm maps/park_map.yaml maps/park_objects.yaml maps/park_descriptors.yaml map_tools/tests/test_extract_descriptors.py
git commit -m "feat(map): emit park_descriptors.yaml with per-object uniqueness"
```

---

### Task 9: Anchor detector plugin

**Files:**
- Create: `landmark_loc/anchor_detector.py`
- Modify: `landmark_loc/detector.py` (register in `DETECTORS`)
- Test: `landmark_loc/tests/test_anchor_detector.py`

**Interfaces:**
- Consumes: `segment.Cluster` (has `.points`), `descriptor.describe`/`descriptor_distance`, the `Detector` contract (`detector.py:91`), `classify.Observation`.
- Produces:
  - `AnchorDetector(descriptors_path, match_threshold=...)` with `name = "anchor"`, implementing `detect(percepts, frame_id, stamp) -> (labels, observations)`. A cluster is labelled with the unique anchor name whose map descriptor is within `match_threshold` (nearest wins); otherwise `"unknown"` and dropped. Only `unique` anchors from `park_descriptors.yaml` are candidates.
  - Registered so `get_detector("anchor")` works.

Observations carry `identity=<anchor name>`, the cluster's pushed-out `x,y` (reuse the near-face pushout convention already in `classify.to_observations` — for a pole, centroid pushed out by `disc_radius`), `confidence` = `1 - dist/match_threshold`.

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_anchor_detector.py
import numpy as np
import yaml
from landmark_loc.segment import Cluster
from landmark_loc.anchor_detector import AnchorDetector
from landmark_loc.descriptor import describe


def _cluster(points):
    xy = points[:, :2]
    return Cluster(points=points, centroid_xy=(float(xy[:, 0].mean()),
                   float(xy[:, 1].mean())), major=1.0, minor=1.0,
                   height=float(points[:, 2].max() - points[:, 2].min()))


def _pole_points(seed):
    r = np.random.RandomState(seed)
    zs = r.uniform(0, 16, 3000)
    xs = r.choice([-0.25, 0.25], 3000) + r.randn(3000) * 0.02
    ys = r.randn(3000) * 0.02
    a = np.column_stack([xs, ys, zs]); b = np.column_stack([ys, xs, zs])
    return np.vstack([a, b])


def _write_descmap(path):
    d = {"postescable_A_pole0": {"descriptor": describe(_pole_points(1)).tolist(),
                                 "unique": True},
         "bench_1": {"descriptor": describe(
             np.random.RandomState(2).rand(500, 3) * [1.8, 0.8, 0.94]).tolist(),
             "unique": False}}
    with open(path, "w") as fh:
        yaml.safe_dump(d, fh)


def test_pole_matches_unique_anchor(tmp_path):
    dm = tmp_path / "d.yaml"; _write_descmap(dm)
    det = AnchorDetector(str(dm), match_threshold=1.5)
    labels, obs = det.detect([_cluster(_pole_points(9))], frame_id="os", stamp=1.0)
    assert labels == ["postescable_A_pole0"]
    assert len(obs) == 1 and obs[0].identity == "postescable_A_pole0"


def test_bench_shaped_cluster_rejected(tmp_path):
    dm = tmp_path / "d.yaml"; _write_descmap(dm)
    det = AnchorDetector(str(dm), match_threshold=1.5)
    bench = np.random.RandomState(3).rand(500, 3) * [1.8, 0.8, 0.94]
    labels, obs = det.detect([_cluster(bench)], frame_id="os", stamp=1.0)
    assert labels == ["unknown"]     # bench is not a UNIQUE anchor -> dropped
    assert obs == []
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_anchor_detector.py -v`
Expected: FAIL — module undefined.

- [ ] **Step 3: Implement + register**

Write `AnchorDetector` (load descriptor map, keep only `unique`, match by `descriptor_distance` under `match_threshold`, emit `classify.Observation`). Add to `detector.py`:

```python
from landmark_loc.anchor_detector import AnchorDetector  # near top
# ... in DETECTORS:
    AnchorDetector.name: AnchorDetector,
```

Note `AnchorDetector.__init__` needs `descriptors_path`, so `get_detector("anchor", descriptors_path=...)` is how the node builds it — the node already passes `~classifier`-specific kwargs is false today, so localizer_node Task 11 must pass this kwarg. Keep the `get_detector(**kwargs)` passthrough (already present at `detector.py:233`).

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_anchor_detector.py landmark_loc/tests/test_detector_seam.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/anchor_detector.py landmark_loc/detector.py landmark_loc/tests/test_anchor_detector.py
git commit -m "feat(landmark): anchor detector matching clusters to unique descriptors"
```

---

### Task 10: Waypoint-anchor logic

**Files:**
- Create: `landmark_loc/waypoint_anchor.py`
- Test: `landmark_loc/tests/test_waypoint_anchor.py`

**Interfaces:**
- Consumes: nothing (pure functions, numpy/math only).
- Produces:
  - `confirm_arrival(waypoint_xy, expected_names, sightings, radius)` → bool. `sightings` is `[(name, x, y), ...]` of pole observations this tick; arrival is confirmed iff at least one expected anchor is seen within `radius` of where the waypoint says it should be.
  - `choose_anchor(prev_anchor, pole_sighting, confirmed_waypoint)` → `(anchor_xy, source)` where `source ∈ {"pole","waypoint","hold"}`; pole wins over waypoint wins over holding the previous anchor (spec: pole is the stronger fix).
  - `fault_offset(predicted_xy, sighting_xy)` → float distance; the caller thresholds it.

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_waypoint_anchor.py
from landmark_loc.waypoint_anchor import confirm_arrival, choose_anchor, fault_offset


def test_arrival_confirmed_when_expected_pole_seen():
    assert confirm_arrival((10.0, 5.0), {"pole_A"},
                           [("pole_A", 10.3, 5.1)], radius=1.0) is True


def test_arrival_rejected_when_pole_absent_or_far():
    assert confirm_arrival((10.0, 5.0), {"pole_A"}, [], radius=1.0) is False
    assert confirm_arrival((10.0, 5.0), {"pole_A"},
                           [("pole_A", 30.0, 5.0)], radius=1.0) is False


def test_pole_beats_waypoint_beats_hold():
    assert choose_anchor((0, 0), (7, 7), (3, 3)) == ((7, 7), "pole")
    assert choose_anchor((0, 0), None, (3, 3)) == ((3, 3), "waypoint")
    assert choose_anchor((0, 0), None, None) == ((0, 0), "hold")


def test_fault_offset():
    assert abs(fault_offset((0, 0), (3, 4)) - 5.0) < 1e-9
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_waypoint_anchor.py -v`
Expected: FAIL — module undefined.

- [ ] **Step 3: Implement**

```python
# landmark_loc/waypoint_anchor.py
"""Pure decision logic for waypoint re-anchoring. No ROS.

Arrival is confirmed by DESCRIPTOR SIGHTING, never by move_base status or fused
pose (design doc: the fused pose is exactly what a navsat attack controls). A
pole sighting is a stronger fix than a waypoint assertion, so it wins.
"""
import math


def confirm_arrival(waypoint_xy, expected_names, sightings, radius):
    wx, wy = waypoint_xy
    for name, x, y in sightings:
        if name in expected_names and math.hypot(x - wx, y - wy) <= radius:
            return True
    return False


def choose_anchor(prev_anchor, pole_sighting, confirmed_waypoint):
    if pole_sighting is not None:
        return (pole_sighting, "pole")
    if confirmed_waypoint is not None:
        return (confirmed_waypoint, "waypoint")
    return (prev_anchor, "hold")


def fault_offset(predicted_xy, sighting_xy):
    return math.hypot(predicted_xy[0] - sighting_xy[0],
                      predicted_xy[1] - sighting_xy[1])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_waypoint_anchor.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/waypoint_anchor.py landmark_loc/tests/test_waypoint_anchor.py
git commit -m "feat(landmark): pure waypoint re-anchoring + fault-offset logic"
```

---

### Task 11: Wire anchor detector + re-anchoring into the localizer

**Files:**
- Modify: `landmark_loc/localizer_node.py`
- Test: `landmark_loc/tests/test_node_helpers.py` (extend) — node-helper level, no live ROS.

**Interfaces:**
- Consumes: `get_detector("anchor", descriptors_path=...)`, `waypoint_anchor.choose_anchor`/`confirm_arrival`/`fault_offset`, the existing `compose_prior`/`on_odom`/`on_compass` machinery.
- Produces: a localizer that (a) selects the anchor detector via `~classifier:=anchor` with a new `~descriptors_path` param, (b) sets the anchor from the most recent pole sighting or confirmed waypoint rather than the one-time GPS capture, (c) publishes a fault when `fault_offset` exceeds `~fault_gate`.

Read `localizer_node.py:56-86` (`compose_prior`), `:313-355` (odom/compass/map/mode handlers, the one-time anchor at `:333-353`), and `:358-447` (`on_cloud`). The anchor capture at `:333-353` becomes: seed the FIRST anchor from GPS as today (bootstrap), but thereafter `choose_anchor` updates it each tick from pole sightings. Subscribe to the operator's active-waypoint topic (Task 12 defines it: `/operator/active_waypoint`, `geometry_msgs/PointStamped`, map frame) to know the expected arrival point and its expected anchor names.

- [ ] **Step 1: Write the failing test** (extend `test_node_helpers.py`)

```python
# landmark_loc/tests/test_node_helpers.py  (add)
def test_anchor_updates_from_pole_sighting():
    # helper under test: localizer_node._update_anchor(prev, pole, wp) delegates
    # to waypoint_anchor.choose_anchor and returns (anchor, source).
    from landmark_loc import localizer_node as ln
    assert ln._update_anchor((0, 0), (5, 5), None) == ((5, 5), "pole")
    assert ln._update_anchor((0, 0), None, None) == ((0, 0), "hold")
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_node_helpers.py -v`
Expected: FAIL — `_update_anchor` undefined.

- [ ] **Step 3: Implement**

Add `_update_anchor` (thin wrapper over `choose_anchor`) and thread it through `on_cloud`: after `det.detect(...)`, extract pole-family observations as `sightings`, pick the nearest as `pole_sighting`, read the confirmed-waypoint anchor, call `_update_anchor`, and use the result as `anchor_map` in `compose_prior`. Add `~descriptors_path` and `~fault_gate` params; validate `~classifier` against the extended `DETECTORS`. Publish fault on `/landmark_fault` (`std_msgs/Float32`, the offset) when it exceeds `~fault_gate`. Keep `~classifier:=cascade` the default so existing runbooks are unchanged.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/ -v`
Expected: PASS (all, including the untouched cascade/constellation tests).

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/localizer_node.py landmark_loc/tests/test_node_helpers.py
git commit -m "feat(localizer): anchor from pole sightings + confirmed waypoints"
```

---

### Task 12: Operator waypoint sequence + arrival confirmation

**Files:**
- Modify: `operator/operate.py`
- Test: `operator/tests/test_waypoints.py`

**Interfaces:**
- Consumes: existing `move_base` client, `objects.py` name→(x,y) table, `waypoint_anchor.confirm_arrival`.
- Produces:
  - a `route <name> <name> ...` command that queues waypoints and drives them in order;
  - publishes the active waypoint on `/operator/active_waypoint` (`geometry_msgs/PointStamped`, map frame) plus its expected anchor names on `/operator/expected_anchors` (`std_msgs/String`, comma-separated) so the localizer (Task 11) can confirm arrival;
  - advances to the next waypoint only when the localizer reports descriptor-confirmed arrival (subscribe to a confirmation signal, e.g. `/landmark_fix` proximity or an explicit `/operator/arrival_confirmed`), NEVER on move_base `SUCCEEDED` alone.

Read `operate.py:352-394` (`_dispatch`), `:396-465` (`_do_goal*`), `:578-594` (help). Keep single-goal commands working; `route` is additive. The queue/advance logic should live in a small pure helper so it is unit-testable without ROS.

- [ ] **Step 1: Write the failing test**

```python
# operator/tests/test_waypoints.py
from operator.waypoint_queue import WaypointQueue


def test_queue_advances_only_on_confirmation():
    q = WaypointQueue(["pole_A", "bench_3", "pole_B"])
    assert q.current() == "pole_A"
    q.on_arrival(confirmed=False)          # move_base says done, perception does not
    assert q.current() == "pole_A"          # must NOT advance
    q.on_arrival(confirmed=True)
    assert q.current() == "bench_3"
    q.on_arrival(confirmed=True)
    assert q.current() == "pole_B"
    q.on_arrival(confirmed=True)
    assert q.current() is None and q.done()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest operator/tests/test_waypoints.py -v`
Expected: FAIL — module undefined.

- [ ] **Step 3: Implement**

Create `operator/waypoint_queue.py` with the pure `WaypointQueue` (`current`, `on_arrival(confirmed)`, `done`). Wire it into `operate.py`: `route` parses names → queue; on each localizer arrival-confirmation message call `on_arrival(confirmed=True)` and send the next goal; publish `active_waypoint` + `expected_anchors` when a waypoint becomes current. Add help text.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest operator/tests/test_waypoints.py operator/tests/ -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add operator/waypoint_queue.py operator/operate.py operator/tests/test_waypoints.py
git commit -m "feat(operator): waypoint route with descriptor-confirmed advance"
```

---

### Task 13: In-sim validation (main conversation only — NOT a subagent)

**Files:** none (validation task). Run per CLAUDE.md: clean kill first, launch in tracked shells, judge by the robot's actual position in Gazebo, never by fused pose.

- [ ] **Step 1:** `./load-park-world.sh` with `gzclient` up on `:0`; confirm the three poles are visible in Gazebo and `/os0_cloud_node/points` has a publisher.
- [ ] **Step 2:** Echo the cloud near a pole; confirm returns above z≈12 m exist (nothing else in the park reaches that height) — the descriptor's discriminating band.
- [ ] **Step 3:** Run the localizer with `_classifier:=anchor _descriptors_path:=maps/park_descriptors.yaml`; drive toward a pole; confirm a `/landmark_fix` is produced from a single pole sighting and the fix lands the robot near the true pole in Gazebo.
- [ ] **Step 4:** Run the operator `route` across two poles; confirm advance happens only on descriptor confirmation, and that drift between poles is bounded (does not reach the 13.5 m divergence the old single-anchor design hit).
- [ ] **Step 5:** With the run active, launch `attack.sh navsat --drift-rate 0.5 --max-offset 15 --duration 40`; confirm the descriptor-derived fix does not follow the spoof and a `/landmark_fault` fires. Record findings in the PR description; do not commit sim logs.

---

## Self-Review

**1. Spec coverage**

| Spec section | Task |
|---|---|
| NDT voxel shape (linear/planar/spherical) | 1 |
| Per-band descriptor + distance | 2 |
| Partial-view / decimation robustness | 3 |
| Mesh surface sampling (not vertices) | 4 |
| Distinctiveness scoring | 5 |
| `postescable` type registry entry | 6 |
| Power-line poles in park.world + GAZEBO_MODEL_PATH | 7 |
| Descriptor map extraction, poles expanded | 8 |
| Anchor detector (unique-only, classifier replaced) | 9 |
| Waypoint re-anchoring + fault logic | 10 |
| Localizer wiring (anchor source, fault publish) | 11 |
| Operator waypoint route + confirmed arrival | 12 |
| In-sim validation incl. navsat-attack resistance | 13 |
| Cable-return pollution (Risk 2) | handled in 13 Step 2 observation; if clusters include cable, add a height/extent filter in `on_cloud` — flagged, not pre-coded |
| Coverage gaps (Risk 4) | accepted; validated in 13 Step 4 |

**2. Placeholder scan:** no TBD/TODO; every code step carries real code. Risk 2 (cable filter) is deliberately left as a conditional in Task 13 because whether it is needed is an empirical question answered only in sim — the plan says exactly what to do if it triggers.

**3. Type consistency:** `describe`/`descriptor_distance`/`voxel_shape` (Tasks 1–2) used consistently in 3, 5, 8, 9. `_expand_poles`/`POLE_OFFSETS` reused from `extract_lake_map.py` in 7–8. `choose_anchor`/`confirm_arrival`/`fault_offset` (Task 10) consumed by 11–12. `AnchorDetector.name = "anchor"` selected via `~classifier` in 11. `WaypointQueue.on_arrival(confirmed)` (12) matches the localizer's confirmation signal (11).

## Notes for the executor

- **Descriptor parameters (`voxel`, `min_voxel_pts`, `n_bands`, `DESC_UNIQUE_THRESHOLD`, `match_threshold`) are tuned, not sacred.** Task 3 is the anchor: if real in-sim pole clouds don't match the mesh-sampled descriptor, adjust these and re-run Tasks 2–3 + 8 before touching anything downstream. Record the final values in `descriptor.py`.
- **Do not run the simulator inside Tasks 1–12.** They are pure-Python and pytest-only. Only Task 13 touches the sim, and only from the main conversation.

---

# Per-region tasks (T14–T21) — the pivot

These replace superseded Tasks 8, 9, 11, 12, 13. Tasks 1–7 and 10 are done and reused. Build order: pure infrastructure (T14–T16) → descriptor design (T17) → extraction (T18) → runtime (T19) → operator (T20) → in-sim (T21). Each of T14–T16 is deliberately self-contained so its implementer's context holds only that piece — an `.obj` parser implementer never sees descriptor code, and vice versa.

### Task T14: Wavefront .obj triangle reader

**Files:**
- Create: `map_tools/obj_read.py`
- Test: `map_tools/tests/test_obj_read.py`

**Interfaces:**
- Consumes: nothing (stdlib + numpy).
- Produces: `read_obj_triangles(obj_path, scale=1.0)` → `(M,3,3)` numpy array of triangles in metres. Reads only `v` (vertex) and `f` (face) lines; ignores `vn`/`vt`/`usemtl`/`o`/`g`/comments. Triangulates polygon faces by fan. Face indices are 1-based and may be `v`, `v/vt`, or `v/vt/vn` — take the vertex index (before the first `/`); negative (relative) indices supported.

- [ ] **Step 1: failing test**

```python
# map_tools/tests/test_obj_read.py
import os, numpy as np
from map_tools.obj_read import read_obj_triangles

BARK = os.path.join(os.path.dirname(__file__), "..", "..", "models_opt", "tree_8", "bark8.obj")

def test_reads_a_known_obj_height():
    tris = read_obj_triangles(BARK, scale=1.0)
    assert tris.ndim == 3 and tris.shape[1:] == (3, 3)
    zs = tris[:, :, 2]
    assert zs.max() - zs.min() > 5.0   # the trunk mesh is several metres tall

def test_face_formats_and_fan_triangulation():
    import tempfile, textwrap
    obj = textwrap.dedent('''\
        v 0 0 0
        v 1 0 0
        v 1 1 0
        v 0 1 0
        f 1/1/1 2/2/2 3/3/3 4/4/4
    ''')
    with tempfile.NamedTemporaryFile("w", suffix=".obj", delete=False) as fh:
        fh.write(obj); path = fh.name
    tris = read_obj_triangles(path, scale=2.0)
    assert tris.shape == (2, 3, 3)          # quad -> 2 triangles by fan
    assert np.isclose(np.abs(tris).max(), 2.0)  # scale applied

def test_negative_indices():
    import tempfile, textwrap
    obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n"
    with tempfile.NamedTemporaryFile("w", suffix=".obj", delete=False) as fh:
        fh.write(obj); path = fh.name
    tris = read_obj_triangles(path)
    assert tris.shape == (1, 3, 3)
```

- [ ] **Step 2: run, expect fail** — `python3 -m pytest map_tools/tests/test_obj_read.py -v`
- [ ] **Step 3: implement** `read_obj_triangles`: parse `v` lines into a vertex list; for each `f`, split tokens, take the substring before the first `/` as the (1-based, possibly negative) vertex index, resolve to 0-based, fan-triangulate (v0,vi,vi+1), stack, multiply by `scale`. No third-party libs.
- [ ] **Step 4: run, expect pass**
- [ ] **Step 5: commit** — `git add map_tools/obj_read.py map_tools/tests/test_obj_read.py && git commit -m "feat(map): minimal Wavefront .obj triangle reader"`

---

### Task T15: Scene-point assembler

**Files:**
- Create: `map_tools/scene_points.py`
- Test: `map_tools/tests/test_scene_points.py`

**Interfaces:**
- Consumes: `mesh_sample.sample_surface` (.dae), `obj_read.read_obj_triangles` + `mesh_sample` area sampling (.obj), `sdf_parse.parse_models`, `park_types` registry (mesh paths + scales).
- Produces: `scene_cloud(models, per_object_n=2000, seed=0)` → `(N,3)` map-frame point cloud combining every catalog model's surface points, each placed by its world (x,y,yaw). A shared helper `sample_model(model, n, seed)` returns one model's points in mesh-local frame (dispatching .dae vs .obj by extension), which `scene_cloud` rotates by yaw and translates by (world_x, world_y). z from the mesh.

**Note:** the registry gives mesh path + scale per family; `sample_surface` already area-samples a `.dae`. For `.obj`, area-sample `read_obj_triangles` output with the SAME area-weighted scheme (factor it out of `mesh_sample` if needed, or add `sample_triangles(tris, n, seed)` there and have both paths call it). Reuse, do not duplicate the sampler.

- [ ] **Step 1: failing test**

```python
# map_tools/tests/test_scene_points.py
import os, numpy as np
from map_tools.sdf_parse import parse_models
from map_tools.scene_points import scene_cloud

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment", "worlds", "park.world")

def test_scene_cloud_spans_the_park_and_is_tall():
    ms = parse_models(WORLD)
    cloud = scene_cloud(ms, per_object_n=500, seed=0)
    assert cloud.shape[1] == 3 and len(cloud) > 10000
    # park extent is roughly x in [-50,48], y in [-26,23]; poles reach ~16 m + ground z~3
    assert cloud[:, 0].min() < -40 and cloud[:, 0].max() > 40
    assert cloud[:, 2].max() > 15.0        # the tall added structures are present
    # determinism
    assert np.array_equal(cloud, scene_cloud(ms, per_object_n=500, seed=0))
```

- [ ] **Step 2: run, expect fail**
- [ ] **Step 3: implement** `sample_model` (dispatch by mesh extension via the registry; skip families with no catalog mesh AND no obj, logging which) and `scene_cloud` (place each model by yaw+translation). Poles: expand via `_expand_poles`? NO — for the scene cloud you want the RAW mesh points at the model pose, which already contains both poles and cables; do not pre-crop. The location grid (T18) handles windows.
- [ ] **Step 4: run, expect pass**
- [ ] **Step 5: commit** — `git add map_tools/scene_points.py map_tools/tests/test_scene_points.py map_tools/mesh_sample.py && git commit -m "feat(map): assemble one map-frame scene cloud from all placed meshes"`

---

### Task T16: Region-window cutter

**Files:**
- Modify: `landmark_loc/descriptor.py` (add `window` — it belongs with the descriptor since both are pure point-geometry and runtime needs it too)
- Test: `landmark_loc/tests/test_descriptor.py` (extend)

**Interfaces:**
- Consumes: numpy only.
- Produces: `window(cloud, cx, cy, radius)` → the subset of `(N,3)` `cloud` whose x,y is within `radius` of `(cx,cy)`, recentred so the window centre is at x=y=0 (z left absolute). Recentring is what makes a map window and a runtime window comparable regardless of world position.

- [ ] **Step 1: failing test**

```python
# append to landmark_loc/tests/test_descriptor.py
from landmark_loc.descriptor import window

def test_window_selects_and_recenters():
    pts = np.array([[10.,10.,1.],[10.5,10.,1.],[30.,30.,1.]])
    w = window(pts, 10.0, 10.0, 2.0)
    assert len(w) == 2                       # third point is 28 m away
    assert abs(w[:,0].mean()) < 1.0 and abs(w[:,1].mean()) < 1.0   # recentred near origin
    assert np.allclose(w[:,2], 1.0)          # z untouched
```

- [ ] **Step 2: run, expect fail**
- [ ] **Step 3: implement** `window`: mask by `hypot(x-cx,y-cy) <= radius`, subtract `(cx,cy)` from x,y, leave z.
- [ ] **Step 4: run, expect pass**
- [ ] **Step 5: commit** — `git add landmark_loc/descriptor.py landmark_loc/tests/test_descriptor.py && git commit -m "feat(descriptor): spatial window cutter with recentering"`

---

### Task T17: Horizontal-arrangement descriptor

**Files:**
- Modify: `landmark_loc/descriptor.py` (add `describe_region`)
- Test: `landmark_loc/tests/test_descriptor.py`

**Interfaces:**
- Consumes: `voxel_shape`, `describe` (the vertical part, T2), numpy.
- Produces: `describe_region(points, n_sectors=8, n_rings=3, radius=12.0, **describe_kwargs)` → a 1-D numpy vector concatenating (a) the flattened vertical descriptor `describe(points)` and (b) an arrangement grid: for each of `n_sectors` angular sectors × `n_rings` radial rings, the occupied-point mass and mean voxel-shape mix in that cell. `region_distance(a, b)` → weighted L2 with the arrangement block weighted by `ARRANGEMENT_WEIGHT = 1.0` (tunable) so arrangement and vertical shape both count.

**Design note (this is the real landmark reasoning):** the arrangement grid is what distinguishes two identical structures in different neighbourhoods. A sector/ring cell records "how much structure sits in this direction-and-distance from the window centre." Two windows over identical isolated structures with identical surroundings are identical (correct); two with different neighbours differ in the cells where the neighbours fall. Keep it rotation-SENSITIVE for now (the world has an absolute compass heading available at runtime); note in a comment that a rotation-invariant variant is a future option if heading proves unreliable.

- [ ] **Step 1: failing test**

```python
# append to landmark_loc/tests/test_descriptor.py
from landmark_loc.descriptor import describe_region, region_distance

def _tower(cx, cy, seed):
    r = _rng(seed); z = r.uniform(0,16,3000)
    x = r.choice([-0.25,0.25],3000)+r.randn(3000)*0.02 + cx
    y = r.randn(3000)*0.02 + cy
    return np.column_stack([x,y,z])

def test_identical_structure_same_empty_neighbourhood_matches():
    a = _tower(0,0,1); b = _tower(0,0,2)
    da, db = describe_region(a), describe_region(b)
    assert region_distance(da, db) < 1.0     # same shape, same (empty) surroundings

def test_neighbour_in_different_direction_separates():
    # same central structure; one has a neighbour to the +x side, one to +y
    base = _tower(0,0,1)
    east = np.vstack([base, _tower(8,0,3)])
    north = np.vstack([base, _tower(0,8,4)])
    d = region_distance(describe_region(east), describe_region(north))
    assert d > 1.0                            # arrangement differs by direction
```

- [ ] **Step 2: run, expect fail**
- [ ] **Step 3: implement** `describe_region` (vertical block from `describe`; arrangement block = per sector×ring occupied mass + mean voxel_shape) and `region_distance` (weighted L2). Choose `ARRANGEMENT_WEIGHT` so both tests pass; record the value + reasoning in a comment.
- [ ] **Step 4: run, expect pass**
- [ ] **Step 5: commit** — `git add landmark_loc/descriptor.py landmark_loc/tests/test_descriptor.py && git commit -m "feat(descriptor): horizontal-arrangement region descriptor"`

---

### Task T18: Location-grid distinctiveness extractor

**Files:**
- Modify: `map_tools/extract_park_map.py`
- Test: `map_tools/tests/test_extract_regions.py`

**Interfaces:**
- Consumes: `scene_points.scene_cloud` (T15), `descriptor.window` (T16), `descriptor.describe_region`/`region_distance` (T17), `distinctiveness` machinery (T5, generalised to `region_distance`).
- Produces: `main()` additionally writes `maps/park_regions.yaml`: for each grid location that is distinctive, `{x, y, descriptor: [...], nearest: float}`. Grid over the park extent at a chosen step; window radius R from the descriptor. The distinctiveness threshold is CHOSEN from the measured nearest-distance distribution (report the distribution; place the threshold in the empty gap), not hardcoded blindly.

- [ ] **Step 1: failing test**

```python
# map_tools/tests/test_extract_regions.py
import os, yaml
from map_tools import extract_park_map

def test_distinctive_locations_cluster_near_the_added_structures(tmp_path):
    extract_park_map.main(["--out-dir", str(tmp_path), "--regions"])
    with open(tmp_path / "park_regions.yaml") as fh:
        regs = yaml.safe_load(fh)
    assert len(regs) >= 4                     # several distinctive spots exist
    # every distinctive location should be near one of the six known pole positions
    poles = [(-42.5,1.0),(-14.33,6.99),(13.84,12.98),(42.01,18.96),(-27.0,-23.0),(1.8,-23.0)]
    import math
    for r in regs.values():
        assert min(math.hypot(r["x"]-px, r["y"]-py) for px,py in poles) < 20.0
```

- [ ] **Step 2: run, expect fail**
- [ ] **Step 3: implement** the `--regions` path: build `scene_cloud`, iterate a grid, `window`+`describe_region` each, compute each location's nearest-other `region_distance`, threshold, write `park_regions.yaml`. Print the distance distribution and chosen threshold. Keep the existing occupancy-grid/objects outputs intact.
- [ ] **Step 4: run, expect pass** (and report the distance distribution + threshold)
- [ ] **Step 5: commit** — `git add map_tools/extract_park_map.py map_tools/tests/test_extract_regions.py maps/park_regions.yaml && git commit -m "feat(map): per-region distinctiveness over a location grid"`

---

### Task T19: Region anchor detector

**Files:**
- Create: `landmark_loc/region_detector.py`
- Modify: `landmark_loc/detector.py` (register `"region"` in `DETECTORS`)
- Test: `landmark_loc/tests/test_region_detector.py`

**Interfaces:**
- Consumes: `descriptor.window`/`describe_region`/`region_distance`, `park_regions.yaml`, the prior (x,y).
- Produces: `RegionDetector(regions_path, match_threshold, prior_gate=25.0)` with `name="region"`. Method `match(cloud, prior_xy)` → `(map_x, map_y, confidence)` or `None`: describe the window around `prior_xy` in the live cloud, find the nearest distinctive map region whose stored (x,y) is within `prior_gate` of the prior, accept if `region_distance < match_threshold`. The prior gate is what stops a far-away look-alike being chosen.

- [ ] **Step 1: failing test**

```python
# landmark_loc/tests/test_region_detector.py
import numpy as np, yaml
from landmark_loc.region_detector import RegionDetector
from landmark_loc.descriptor import describe_region

def _tower(cx,cy,seed):
    r=np.random.RandomState(seed); z=r.uniform(0,16,3000)
    x=r.choice([-0.25,0.25],3000)+r.randn(3000)*0.02+cx; y=r.randn(3000)*0.02+cy
    return np.column_stack([x,y,z])

def _regmap(path):
    d={"loc0":{"x":10.0,"y":0.0,"descriptor":describe_region(_tower(0,0,1)).tolist()}}
    open(path,"w").write(yaml.safe_dump(d))

def test_matches_region_near_prior(tmp_path):
    p=tmp_path/"r.yaml"; _regmap(str(p))
    det=RegionDetector(str(p), match_threshold=1.0, prior_gate=25.0)
    cloud=_tower(10,0,2)                      # same structure, at the map loc
    out=det.match(cloud, (10.0,0.0))
    assert out is not None and abs(out[0]-10.0)<2.0

def test_prior_gate_rejects_far_lookalike(tmp_path):
    p=tmp_path/"r.yaml"; _regmap(str(p))
    det=RegionDetector(str(p), match_threshold=1.0, prior_gate=25.0)
    cloud=_tower(200,0,2)                     # identical shape but far from the map loc
    assert det.match(cloud, (200.0,0.0)) is None
```

- [ ] **Step 2: run, expect fail**
- [ ] **Step 3: implement** `RegionDetector.match` + register `"region"` in `DETECTORS`. Keep the `Detector` contract shape where sensible (this one is region- not percept-based, so `match` is its own entry point; document that in the module).
- [ ] **Step 4: run, expect pass**
- [ ] **Step 5: commit** — `git add landmark_loc/region_detector.py landmark_loc/detector.py landmark_loc/tests/test_region_detector.py && git commit -m "feat(landmark): prior-gated region anchor detector"`

---

### Task T20: Wire region detector + re-anchoring into the localizer

Same as the original Task 11, but the anchor fix comes from `RegionDetector.match(cloud, prior_xy)` instead of a per-cluster pole sighting. `_update_anchor` (delegating to `waypoint_anchor.choose_anchor`, T10) is unchanged. Params: `~classifier:=region`, `~regions_path`, `~match_threshold`, `~prior_gate`, `~fault_gate`. Publishes `/landmark_fault` and `/landmark_arrival_confirmed` (per ruling F3). Test at the node-helper level (`test_node_helpers.py`), no live ROS. Default `~classifier` stays `cascade` so existing runbooks are unchanged.

- [ ] Write the failing helper test for `_update_anchor` (reuse the T10 semantics), run, implement the wiring, run full `landmark_loc/tests/` + `map_tools/tests/`, commit.

---

### Task T21: Operator waypoint route + in-sim validation

T21a is the original Task 12 (operator `route` + `WaypointQueue` + descriptor-confirmed advance, bare-import per ruling F4), unchanged. T21b is the original Task 13 in-sim validation, main-conversation only, updated for region matching: distinctive structures visible; a fix from a single region match; the correct location resolved (not a far look-alike); drift bounded; navsat attack cannot move the descriptor pose.

- [ ] T21a: implement operator route (see original Task 12 steps), commit.
- [ ] T21b: MAIN CONVERSATION ONLY — run the sim per RUN-MAP-NAV Steps 0–3, then the region-localization and navsat-attack checks. Record findings in the PR; commit no sim logs.
