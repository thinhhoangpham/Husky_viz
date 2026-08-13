# Tree (tree_8) Landmark Type Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add trees (`tree_8`) as a new `tree` landmark type so the constellation matcher has more geometric anchors along the approach route, reducing landmark-mode wandering.

**Architecture:** A cluster is classified `tree` by a vertical-profile rule (a wide canopy band above a narrow trunk), replacing the old `_is_tree()` which dropped trees. Trees flow through the existing pipeline (gate → constellation.match → solve) as just another identity — no matcher/solve/mux change. The lidar crop is raised to see canopies, and the cluster extent cap is raised so canopies survive. The catalog gains 23 tree landmarks from the existing extractor.

**Tech Stack:** Python 3, ROS Noetic (rospy), numpy, pytest. Files in `landmark_loc/` and `map_tools/`, worktree `.worktrees/constellation-matcher` on branch `feat/constellation-matcher`.

## Global Constraints

- **Spec is authoritative:** `docs/superpowers/specs/2026-08-12-tree-landmark-type-design.md`. All thresholds below are measured live in-sim; do not change them without a measurement.
- **Measured thresholds (verbatim):** crop `z_min=-0.5`, `z_max=7.0`; `max_extent=6.0`; tree canopy test = a z-band centered at **z ≥ 2.5 m** with horizontal width **≥ 2.0 m**.
- **Scope: `tree_8` only.** `arbolpartes4` is explicitly EXCLUDED.
- **No ground truth** for pose anywhere (project hard rule): no `gazebo_msgs`, no `/gazebo/*` pose. Catalog positions come from the world-file `<state>` block (offline extraction), which is allowed — it is the known map, not runtime robot pose.
- **The tree rule keys on `cluster.points` (the actual point array), not the bbox** (`major/minor/height`), because a canopy's bbox size varies with view; the vertical profile is view-robust.
- **Do NOT touch** the matcher (`constellation.py`), `solve.py`, the mux (`abs_fix_selector.py`), smoothing, or `signatures.py`. This feature only supplies more landmarks.
- The four rigid types (bench/garden_table/lamp/trash_bin_1) must still classify correctly after the change. The tree rule is naturally exclusive (lamps <1 m wide at all heights; benches have no high band), but this MUST be re-verified in the in-sim task.

---

### Task 1: Vertical-profile tree classifier

**Files:**
- Modify: `landmark_loc/classify.py`
- Test: `landmark_loc/tests/test_classify.py` (rewrite tree-related tests)

**Interfaces:**
- Consumes: `landmark_loc.segment.Cluster` — has fields `points` (np.ndarray Nx3, lidar frame, may be `None` in synthetic tests), `centroid_xy` (tuple), `major`, `minor`, `height` (floats).
- Produces: `classify_cluster(cluster, margins=DEFAULT_MARGINS) -> str` — now returns `"tree"` for canopy clusters; `to_observations(clusters, margins) -> [Observation]` now EMITS `tree` observations (identity `"tree"`) and drops only `"unknown"`.

**Context:** Today `classify.py` has `_is_tree()` (trunk-footprint rule) that runs first and returns `"tree"`, and `to_observations` DROPS `"tree"` and `"unknown"`. This task replaces the trunk-footprint `_is_tree` with a canopy-profile rule and makes `to_observations` EMIT trees. The four rigid-type signature matches (`_matches`) are unchanged.

- [ ] **Step 1: Write the failing tests** (rewrite the tree tests; keep the rigid-type tests)

Replace the tree-specific tests in `landmark_loc/tests/test_classify.py`. Keep `test_classifies_each_type_from_ideal_dims`, `test_ideal_bin_is_still_bin`, `test_ideal_lamp_is_still_lamp`, `test_ambiguous_between_bands_is_unknown` unchanged. REMOVE `test_round_tall_trunk_is_tree_not_lamp`, `test_real_trunks_inside_lamp_height_band_are_tree_not_lamp`, `test_short_trunks_under_crop_are_tree_not_bin`, `test_to_observations_drops_tree_and_unknown` (they assert the OLD trunk-footprint + drop behavior). Add:

```python
import numpy as np
from landmark_loc.segment import Cluster
from landmark_loc import classify


def _canopy_cluster(trunk_w=0.4, canopy_w=4.0, top_z=6.0):
    """Synthetic tree: a narrow trunk (z 0..2.4) under a wide canopy (z 2.5..top).
    Returns a Cluster whose `points` produce the intended vertical profile."""
    pts = []
    # trunk: narrow column of points from z=0 to 2.4
    for z in np.linspace(0.0, 2.4, 12):
        pts += [(0.0, 0.0, z), (trunk_w, 0.0, z)]
    # canopy: wide disc of points from z=2.5 to top_z
    for z in np.linspace(2.5, top_z, 10):
        for a in np.linspace(0, 2 * np.pi, 12, endpoint=False):
            r = canopy_w / 2.0
            pts.append((r * np.cos(a), r * np.sin(a), z))
    arr = np.array(pts, dtype=float)
    major = arr[:, 0].max() - arr[:, 0].min()
    minor = arr[:, 1].max() - arr[:, 1].min()
    height = arr[:, 2].max() - arr[:, 2].min()
    return Cluster(points=arr, centroid_xy=(1.0, 2.0),
                   major=float(major), minor=float(minor), height=float(height))


def test_wide_canopy_over_trunk_is_tree():
    assert classify.classify_cluster(_canopy_cluster()) == "tree"


def test_min_canopy_width_is_tree():
    # width exactly at the 2.0 m threshold, high band -> tree
    assert classify.classify_cluster(_canopy_cluster(canopy_w=2.0, top_z=5.0)) == "tree"


def test_thin_tall_pole_is_not_tree():
    # a lamp-like column: narrow (<2 m) at every height -> NOT tree
    pts = []
    for z in np.linspace(0.0, 3.0, 30):
        pts += [(0.0, 0.0, z), (0.5, 0.0, z)]  # 0.5 m wide the whole way up
    arr = np.array(pts, dtype=float)
    c = Cluster(points=arr, centroid_xy=(1.0, 2.0), major=0.5, minor=0.5, height=3.0)
    assert classify.classify_cluster(c) != "tree"


def test_low_wide_object_is_not_tree():
    # a bench-like object: wide but only low (no band at z >= 2.5) -> NOT tree
    pts = []
    for z in np.linspace(0.0, 0.9, 6):
        for x in np.linspace(0.0, 2.5, 8):
            pts.append((x, 0.0, z))
    arr = np.array(pts, dtype=float)
    c = Cluster(points=arr, centroid_xy=(1.0, 2.0), major=2.5, minor=0.4, height=0.9)
    assert classify.classify_cluster(c) != "tree"


def test_to_observations_emits_tree_drops_unknown():
    bench_dims = _dims("bench")
    clusters = [
        Cluster(points=None, centroid_xy=(1.0, 2.0), major=bench_dims[0],
                minor=bench_dims[1], height=bench_dims[2]),
        _canopy_cluster(),                                            # tree -> emitted
        Cluster(points=None, centroid_xy=(3.0, 4.0), major=1.9,
                minor=1.3, height=0.9),                               # unknown -> dropped
    ]
    obs = classify.to_observations(clusters)
    idents = sorted(o.identity for o in obs)
    assert idents == ["bench", "tree"]
    tree_obs = [o for o in obs if o.identity == "tree"][0]
    assert (tree_obs.x, tree_obs.y) == (1.0, 2.0)


def _dims(fam):
    from landmark_loc.signatures import MESH_SIGNATURES as S
    s = S[fam]
    return s["major"], s["minor"], s["height"]
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd ~/Documents/Husky_viz/.worktrees/constellation-matcher && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_classify.py -v`
Expected: the new `*_is_tree`/`emits_tree` tests FAIL (old `_is_tree` uses footprint, `to_observations` drops trees); the four rigid-type tests still PASS.

- [ ] **Step 3: Implement the canopy-profile rule**

In `landmark_loc/classify.py`, replace the `_is_tree(...)` function and its trunk-constant block with a canopy-profile rule. Add constants near the top (remove the old `_TREE_*` / `_LAMP_BAND_BOTTOM` constants that the footprint rule used):

```python
# Tree = a wide canopy band ABOVE a narrow trunk. Keys on the vertical PROFILE
# (view-robust), not the canopy's absolute size (which varies 3.7-4.75 m in-sim).
# Thresholds measured live (13 trees, 4 viewpoints): canopy begins by z~2.75
# (p50 2.25), canopy width 2.9-4.75 m; lamps stay <1 m wide at every height.
_TREE_CANOPY_MIN_Z = 2.5      # a wide band at/above this height is a canopy
_TREE_CANOPY_MIN_WIDTH = 2.0  # canopy horizontal width floor (lamp head < 1 m)
_TREE_BAND = 0.5              # z-band thickness for the profile scan
_TREE_BAND_MIN_PTS = 3        # a band needs this many points to measure width


def _band_width(pts, z0, z1):
    """Horizontal bbox-diagonal width of points whose z is in [z0, z1)."""
    m = (pts[:, 2] >= z0) & (pts[:, 2] < z1)
    if int(m.sum()) < _TREE_BAND_MIN_PTS:
        return 0.0
    xy = pts[m][:, :2]
    return float(((xy[:, 0].max() - xy[:, 0].min()) ** 2
                  + (xy[:, 1].max() - xy[:, 1].min()) ** 2) ** 0.5)


def _is_tree(cluster, margins):
    """True when the cluster has a wide canopy band at z >= _TREE_CANOPY_MIN_Z.

    A lamp is narrow (<_TREE_CANOPY_MIN_WIDTH) at every height, a bench/bin has no
    band that high, so neither can satisfy this. Uses the cluster's raw points
    (the vertical profile), not its bbox. Synthetic clusters with points=None are
    never trees (the four rigid-type tests build those)."""
    pts = cluster.points
    if pts is None or len(pts) == 0:
        return False
    top = float(pts[:, 2].max())
    z = _TREE_CANOPY_MIN_Z
    while z < top:
        if _band_width(pts, z, z + _TREE_BAND) >= _TREE_CANOPY_MIN_WIDTH:
            return True
        z += _TREE_BAND
    return False
```

Keep `classify_cluster` calling `_is_tree(cluster, margins)` first and returning `"tree"` when true (unchanged control flow — only `_is_tree`'s body changed). Change `to_observations` to EMIT trees:

```python
def to_observations(clusters, margins=DEFAULT_MARGINS):
    out = []
    for c in clusters:
        ident = classify_cluster(c, margins)
        if ident == "unknown":
            continue
        out.append(Observation(identity=ident,
                               x=c.centroid_xy[0], y=c.centroid_xy[1]))
    return out
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd ~/Documents/Husky_viz/.worktrees/constellation-matcher && PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_classify.py -v`
Expected: ALL pass (rigid types + new tree tests).

- [ ] **Step 5: Run the full landmark_loc test suite (no regression)**

Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/ -v`
Expected: all pass except the pre-existing unrelated `test_launch.py::test_runbook_offers_both_modes` failure noted in the constellation-matcher ledger (confirm it is the SAME failure, not a new one).

- [ ] **Step 6: Commit**

```bash
git add landmark_loc/classify.py landmark_loc/tests/test_classify.py
git commit -m "feat(classify): tree = wide canopy over narrow trunk (emit, not drop)"
```

---

### Task 2: Add tree_8 to the catalog (map + identity)

**Files:**
- Modify: `map_tools/extract_park_map.py:68` (`PLACE_FAMILIES`)
- Modify: `landmark_loc/catalog.py` (`_IDENTITY_FAMILIES`, identity mapping)
- Modify/regenerate: `maps/park_places.yaml`
- Test: `landmark_loc/tests/test_catalog.py` (add tree cases; create the file if absent)

**Interfaces:**
- Consumes: `map_tools.sdf_parse.parse_models` (already classifies `tree_8` at link_0), `map_tools.sdf_parse.classify` (name → family; already returns `"tree_8"`).
- Produces: `catalog.load(places_path) -> [MapLandmark]` now includes entries with `identity == "tree"` for every `tree_8*` name; `MapLandmark(name, identity, x, y)`.

**Context:** `extract_park_map.py` already parses tree_8 (single link_0 = correct position) but filters it out of the places table. `catalog.py` maps names → identity via `map_tools.sdf_parse.classify` and keeps only `_IDENTITY_FAMILIES`. Add `tree_8`/`tree` to both.

- [ ] **Step 1: Write the failing catalog test**

Create/extend `landmark_loc/tests/test_catalog.py`:

```python
from landmark_loc import catalog


def test_tree8_names_load_as_tree_identity(tmp_path):
    places = tmp_path / "places.yaml"
    places.write_text(
        "bench_1: {x: 1.0, y: 2.0}\n"
        "tree_8: {x: 10.0, y: 20.0}\n"
        "tree_8_clone_3: {x: 11.0, y: 21.0}\n")
    lms = catalog.load(str(places))
    ids = {lm.name: lm.identity for lm in lms}
    assert ids["tree_8"] == "tree"
    assert ids["tree_8_clone_3"] == "tree"
    assert ids["bench_1"] == "bench"
```

- [ ] **Step 2: Run to verify it fails**

Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_catalog.py -v`
Expected: FAIL — `tree_8*` currently filtered out (not in `_IDENTITY_FAMILIES`).

- [ ] **Step 3: Add tree identity to the catalog**

In `landmark_loc/catalog.py`: add `"tree"` to `_IDENTITY_FAMILIES`, and map the `tree_8` family to identity `"tree"`. The current `load()` uses `_family_of(name)` (which returns `"tree_8"`); translate that family to identity `"tree"` before the `_IDENTITY_FAMILIES` check:

```python
_IDENTITY_FAMILIES = {"bench", "garden_table", "lamp", "trash_bin_1", "tree"}

# map raw world-file family -> matcher identity (tree_8 model -> generic 'tree')
_FAMILY_TO_IDENTITY = {"tree_8": "tree"}


def _identity_of(name):
    fam = _family_of(name)
    return _FAMILY_TO_IDENTITY.get(fam, fam)
```

Then in `load()` replace `fam = _family_of(name)` with `fam = _identity_of(name)` and keep the `if fam not in _IDENTITY_FAMILIES: continue` gate. `MapLandmark(name, fam, x, y)` now stamps identity `"tree"` for tree_8 names.

- [ ] **Step 4: Add tree_8 to the map extractor**

In `map_tools/extract_park_map.py:68`, change:

```python
PLACE_FAMILIES = ("bench", "garden_table", "lamp", "trash_bin_1", "tree_8")
```

(Leave `build_grid`/`RADII` alone — tree_8 already has `RADII["tree_8"]=0.45` for the occupancy disc.)

- [ ] **Step 5: Regenerate park_places.yaml and verify 23 trees**

Run:
```bash
cd ~/Documents/Husky_viz/.worktrees/constellation-matcher
PYTHONPATH=. python3 -m map_tools.extract_park_map \
  --world natural_environments_ros_opt/natural_enviroment/worlds/park.world \
  --out-dir maps
PYTHONPATH=. python3 -c "import yaml; d=yaml.safe_load(open('maps/park_places.yaml')); print('tree_8 entries:', sum(1 for k in d if k.startswith('tree_8')))"
```
Expected: `tree_8 entries: 23`. (If the extractor CLI flags differ, read `main()` in `extract_park_map.py` and use its actual arguments — do NOT hand-edit the yaml.)

- [ ] **Step 6: Run catalog tests + load the real catalog**

Run:
```bash
PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_catalog.py -v
PYTHONPATH=. python3 -c "from landmark_loc import catalog; lms=catalog.load('maps/park_places.yaml'); print('tree landmarks:', sum(1 for l in lms if l.identity=='tree'))"
```
Expected: tests pass; `tree landmarks: 23`.

- [ ] **Step 7: Commit**

```bash
git add map_tools/extract_park_map.py landmark_loc/catalog.py maps/park_places.yaml landmark_loc/tests/test_catalog.py
git commit -m "feat(catalog): add tree_8 as 'tree' identity (23 landmarks)"
```

---

### Task 3: Raise the crop and cluster extent in the localizer

**Files:**
- Modify: `landmark_loc/localizer_node.py` (param defaults `z_min`, `z_max`, `max_extent`)

**Interfaces:**
- Consumes: nothing new. `segment.crop(pts, z_min, z_max, max_range)`, `segment.cluster(pts, link_dist, min_pts, max_extent)` — signatures unchanged.
- Produces: no interface change; only default param values change.

**Context:** `localizer_node.py` builds a `p` dict of params. The tree rule needs canopies in the cloud (raise `z_max`), the ground blob out (raise `z_min`), and canopies to survive clustering (raise `max_extent`).

- [ ] **Step 1: Change the three param defaults**

In `landmark_loc/localizer_node.py`, in the `p = dict(...)` block:
- `z_min`: `rospy.get_param("~z_min", -0.73)` → default `-0.5`
- `z_max`: `rospy.get_param("~z_max", 3.5)` → default `7.0`
- `max_extent`: `rospy.get_param("~max_extent", 3.5)` → default `6.0`

Add a one-line comment on each citing the spec (measured). No other change.

- [ ] **Step 2: Verify the node imports and params read (smoke, no sim)**

Run:
```bash
PYTHONPATH=. python3 -c "import ast; ast.parse(open('landmark_loc/localizer_node.py').read()); print('parse ok')"
PYTHONPATH=. python3 -c "from landmark_loc import segment, classify, catalog, solve; print('imports ok')"
```
Expected: both print ok. (Full node run needs ROS + sim — that is Task 4, run by main.)

- [ ] **Step 3: Commit**

```bash
git add landmark_loc/localizer_node.py
git commit -m "feat(localizer): raise crop to z[-0.5,7.0], max_extent 6.0 for canopies"
```

---

### Task 4: In-sim acceptance (RUN BY MAIN, not a subagent)

**Files:** none (verification only).

**Context:** This task is run by the main conversation from a clean sim start, following RUN-MAP-NAV.md exactly, per the sim-run discipline (tracked shells, kill by exact PID, verify by reading). It is NOT dispatched to a subagent.

- [ ] **Step 1: Bring up the sim clean** (RUN-MAP-NAV.md Steps 0-3), localizer pointed at the worktree (`PYTHONPATH=~/Documents/Husky_viz/.worktrees/constellation-matcher`).

- [ ] **Step 2: Confirm trees are detected and the four types still classify.** Add a temporary diag (or reuse the localizer `[diag]` line) to log matched landmark identities; drive/observe near trees and confirm `tree` observations appear AND bench/lamp/bin/table still classify (no regression at the raised crop).

- [ ] **Step 3: Run the full demo** — GPS → spoof (attacker container) → `mode landmark` → re-send goal. Watch the localizer `[diag]` and the fused pose.

- [ ] **Step 4: Measure the acceptance criterion** — approach-route wandering vs the pre-tree baseline: count published-fix moves >1 m during the drive and their max magnitude; confirm the robot reaches the goal under active spoof with a cleaner path (fewer/smaller lurches than the baseline's 26 moves >1 m, up to 11.6 m).

- [ ] **Step 5: Tear the sim down clean** (kill by exact PID, relauncher first, verify master down by reading).

- [ ] **Step 6: Record the result in the SDD ledger** (numbers, pass/fail vs baseline).

---

## Self-Review

**Spec coverage:** classifier rule (Task 1), crop + max_extent (Task 3), catalog 23 trees (Task 2), signatures untouched (no task — correct, it is explicitly unchanged), data flow unchanged (no task — correct), in-sim acceptance with wandering metric (Task 4), arbolpartes4 excluded (Global Constraints + Task 2 only adds tree_8). All covered.

**Placeholder scan:** all code steps carry real code; all thresholds are concrete measured numbers; the extractor CLI step has a fallback instruction to read `main()` if flags differ. No TBDs.

**Type consistency:** `Cluster.points` used in Task 1 matches `segment.Cluster` (has `points`). `classify_cluster`/`to_observations` signatures unchanged. `catalog.load` returns `MapLandmark(name, identity, x, y)` — Task 2 keeps that shape, only adds the `tree` identity. `constellation.match`/`solve_pose` untouched (Global Constraints).

**Known conflict flagged for pre-flight:** Task 1 REWRITES existing tests in `test_classify.py` that assert the OLD "tree is dropped" behavior. This is intended (the feature inverts that behavior) and is called out explicitly in Task 1 Step 1 — it is not a defect, but the controller's pre-flight scan should note it so the reviewer does not treat the test rewrite as removing coverage.
