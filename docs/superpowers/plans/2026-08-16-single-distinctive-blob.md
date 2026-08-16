# Single distinctive blob — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Localize from **one uniquely-shaped object** recognised by its own shape alone — no arrangement, no window, no classifier — after adding a genuinely one-of-a-kind structure to the park.

**Architecture:** Cluster the live cloud (existing `segment.py`), compute a shape descriptor per *cluster* (the existing `describe()` vertical descriptor, already built and tested), and match against a tiny map of distinctive objects. A cluster matches only if its shape is close to a map object's *and* that map object's shape has no near twin. Position comes from the matched object's known map coordinates plus the measured range/bearing to it.

**Spec:** `docs/superpowers/specs/2026-08-16-unique-landmark-waypoint-localization-design.md`

---

## Why — what we learned the hard way

Two approaches have now been measured on this map:

**Per-object shape matching (v1) failed** because every candidate came in
identical copies: five repeated furniture meshes, and the six power-line poles we
added were six copies of one mesh. Shape alone had nothing to grip — descriptor
distance between identical instances is exactly 0.

**Region-arrangement descriptors (v2) worked offline and failed in-sim.** Offline
it found 11 distinctive locations with 163x separation. Live, parked 0.4 m from a
distinctive spot, the correct region ranked *third* (6.63) behind a region 25.8 m
away (5.75) — a meaningless ranking no threshold could fix. Measured cause: **98%
of the live 8 m window is ground** (12920 of 13237 points) versus 0% in the
mesh-built map. Recorded in the spec under "Region-arrangement descriptors —
built, measured, and SHELVED", along with the deeper reason it is parked: the
arrangement term is sensitive to the ground, to the robot's exact position, and to
heading, because it describes *space* rather than *a thing*.

**This plan takes the third path: make one object genuinely unique, and describe
only the object.** A single-cluster descriptor is not sensitive to what surrounds
it, does not need a window radius, and is far less affected by the ground —
because it describes the cluster's points, not a disc of space. The v1 failure was
never "shape matching does not work"; it was "this map contained nothing unique."
Fix the map, and the simplest mechanism becomes viable.

## The critical rule: exactly ONE

**Add exactly one instance.** Six identical poles is precisely the mistake that
sank v1. The whole premise is that the object's shape has no twin, so a second
copy destroys the property by construction. The extractor must additionally
*verify* uniqueness by measurement rather than assume it (Task 4).

## What to add, and why it will be unmistakable

Measured heights of everything currently in the park:

| object | height |
|---|---|
| bench | 0.94 m |
| trash_bin_1 | 1.04 m |
| garden_table | 1.09 m |
| lamp | 3.15 m |
| tree_8 (trunk) | 7.89 m |
| power-line pole | 16.49 m |

The gap to exploit is **tall AND wide**. Trees are tall but narrow; poles are tall
but a thin open lattice; everything else is under 3.2 m. A **water-tower-like
structure roughly 10–12 m tall and 4–6 m across** occupies a height/width
combination nothing else in this world has, and it is solid rather than latticed,
so its per-band shape statistics differ from a pole's as well.

**Build it from SDF primitives, not a downloaded mesh.** No suitable building,
tower or silo exists in `models_opt/`, `models_lake_opt/`, `~/.gazebo/models/`
(only dumpster, jersey_barrier, barrels, cones, hydrant, asphalt_plane) or
`/usr/share/gazebo-11/models/` (only ground_plane and sun). Primitives are better
here anyway: exact known geometry, no mesh parsing, no 300 MB file, trivially
reproducible, and the map side can compute the descriptor analytically instead of
sampling a mesh. Suggested form: a cylinder tank on a short cylindrical pedestal —
distinctive vertical profile (narrow base, wide top), unlike anything present.

## Global Constraints

- Python 3.8, numpy, PyYAML, pytest. NO new third-party dependencies.
- NEVER Gazebo ground truth. Parsing the static world file is fine.
- `map_tools/park_types.py` stays the single registry.
- Reuse `landmark_loc/descriptor.py`'s `describe()` / `descriptor_distance()`
  (the VERTICAL descriptor, Tasks 1–3 of the original plan — already built,
  tested, and unaffected by the v2 failure). Do NOT use `describe_region` /
  `region_distance` / `window` — that is the shelved arrangement path.
- Do NOT delete the shelved region code. It stays on the branch, unselected.
- `natural_environments_ros_opt/` is **git-ignored** — world edits are NOT
  version controlled. Record exact poses in the plan/report so the edit is
  reproducible, and take a backup before editing.
- Determinism: fixed seeds so the committed map artifact is stable.

---

### Task 1: Add ONE water-tower model to park.world

> **EXECUTE TASK 2 FIRST.** This task's test cannot pass before Task 2 exists.
> `map_tools/sdf_parse.py:78` drops any model whose prefix classifies as
> `skip`, and `classify_prefix("water_tower")` returns `skip` until Task 2
> registers the type. Run Task 2 (registry), then Task 1 (world edit), then
> Task 3. Same three commits, same contents — every commit green and
> bisectable. Task numbering is left as-is so cross-references still resolve.

**Files:**
- Modify: `natural_environments_ros_opt/natural_enviroment/worlds/park.world` (untracked — back it up first)
- Test: `map_tools/tests/test_park_world_tower.py`

**Placement: (20.0, 14.0)** — chosen by search, not by guess. It has **10.8 m
clearance** from the nearest existing object (a 5 m-wide tower needs real room)
and sits **0.8 m from the in-sim test route**, so the robot drives straight past
it. Ground is at z≈2.99, and the model's origin is at its base, so the pose is
`20.0 14.0 2.99 0 -0 0`.

(An earlier candidate at (20, 5) was rejected: only 3.8 m clearance, too tight
for a 5 m tank. The search covered the whole park for sites with >=8 m clearance
within 12 m of the route; (20,14) was the best.)

**Geometry (SDF primitives, `<static>1</static>`):**
- pedestal: cylinder, radius 1.0 m, length 4.0 m, centred at z_local 2.0
- tank: cylinder, radius 2.5 m, length 5.0 m, centred at z_local 8.5
- total height ≈ 11 m, max width 5 m
- give it both `<visual>` and `<collision>` — **collision is essential**, without
  it the lidar returns nothing and the whole feature is inert.

- [ ] **Step 1: failing test**

```python
# map_tools/tests/test_park_world_tower.py
import os
from map_tools.sdf_parse import parse_models

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment",
                     "worlds", "park.world")

def test_exactly_one_tower():
    towers = [m for m in parse_models(WORLD) if m.family == "water_tower"]
    assert len(towers) == 1, "the whole design depends on there being exactly ONE"
    assert round(towers[0].world_x, 1) == 20.0
    assert round(towers[0].world_y, 1) == 14.0
```

- [ ] **Step 2:** run, expect fail (0 towers).
- [ ] **Step 3:** back up the world file, then add the model to **both** halves of
      `park.world`: the `<state world_name='default'>` block (which
      `sdf_parse.parse_models` reads via `link_0`'s pose) **and** the model
      definition blocks after it (the geometry Gazebo loads). Use pose
      `20.0 14.0 2.99 0 -0 0`. The clearance (10.8 m) and route proximity (0.8 m)
      are already verified — do not re-site it without re-running the search.
- [ ] **Step 4:** run, expect pass. Also confirm `parse_models` family counts are
      otherwise unchanged (was 94 models: tree_8 23, bench 16, arbolpartes4 15,
      lamp 15, garden_table 11, trash_bin_1 11, postescable 3 → now +1 water_tower).
- [ ] **Step 5:** commit the test (the world file itself is git-ignored — say so in
      the commit message and record the exact pose there).

---

### Task 2: Register the water_tower type

**Files:**
- Modify: `map_tools/park_types.py`
- Test: `map_tools/tests/test_park_types_tower.py`

- [ ] **Step 1: failing test**

```python
# map_tools/tests/test_park_types_tower.py
from map_tools.park_types import classify_prefix, BY_PREFIX

def test_water_tower_registered():
    assert classify_prefix("water_tower") == "water_tower"
    t = BY_PREFIX["water_tower"]
    assert t.is_catalog is False
    assert t.is_object is True
```

- [ ] **Step 2:** run, expect fail.
- [ ] **Step 3:** add a `ParkType` entry: `world_prefix="water_tower"`,
      `identity="water_tower"`, `is_object=True`, `is_catalog=False`,
      `disc_radius=2.5` (the tank radius — its true footprint),
      `mesh=None` (primitives, no mesh file), `box_stamped=False`.
      **`is_catalog` MUST be `False`.** Every consumer of that flag is
      classifier machinery — `classify.py:117` (`KNOWN_RADIUS` size bands),
      `catalog.py:16` (matcher identity set), `localizer_node.py:174`
      (`_LABEL_COLOR`) — and this design does not classify the tower.
      `test_score.py:188` also asserts every `is_catalog` identity is
      scoreable, so `is_catalog=True` with no `score_family` fails the suite.
      Omit `marker_color` entirely; it defaults, and a non-classified type
      never enters `_LABEL_COLOR`. Leave the scoring fields at their defaults —
      **do not** give it a `score_family`; `score.py`'s `profile_type_score`
      raises on unknown profile identities (this bit us before), and this design
      does not use the score detector.
- [ ] **Step 4:** run, expect pass, plus the whole suite
      (`map_tools/tests/ landmark_loc/tests/ -q`). Watch specifically for
      `test_score.py` / `test_ab_compare.py` regressions from the registry change.
- [ ] **Step 5:** commit.

---

### Task 3: Analytic descriptor for a primitive object

**Files:**
- Create: `map_tools/primitive_sample.py`
- Test: `map_tools/tests/test_primitive_sample.py`

**Interfaces:**
- Produces: `sample_cylinder_stack(stack, n=4000, seed=0)` → (N,3) points sampled
  over the *surfaces* of a list of `(radius, length, z_centre)` cylinders, in
  metres, origin at the model base. Deterministic.

**Why:** the tower has no mesh, so `sample_surface` cannot describe it. Sampling
its primitive surfaces analytically gives the map-side point set, in the same form
`describe()` consumes — keeping map and observation in one space.

- [ ] **Step 1: failing test**

```python
# map_tools/tests/test_primitive_sample.py
import numpy as np
from map_tools.primitive_sample import sample_cylinder_stack

TOWER = [(1.0, 4.0, 2.0), (2.5, 5.0, 8.5)]   # pedestal, tank

def test_shape_and_determinism():
    a = sample_cylinder_stack(TOWER, n=2000, seed=0)
    b = sample_cylinder_stack(TOWER, n=2000, seed=0)
    assert a.shape == (2000, 3)
    assert np.array_equal(a, b)

def test_geometry_matches_the_spec():
    p = sample_cylinder_stack(TOWER, n=4000, seed=1)
    assert p[:, 2].min() > -0.1 and p[:, 2].max() < 11.1      # ~11 m tall
    top = p[p[:, 2] > 6.0]
    assert np.hypot(top[:, 0], top[:, 1]).max() > 2.3         # tank is wide
    low = p[p[:, 2] < 3.5]
    assert np.hypot(low[:, 0], low[:, 1]).max() < 1.2         # pedestal is narrow
```

- [ ] **Step 2:** run, expect fail.
- [ ] **Step 3:** implement — for each cylinder, sample its lateral surface and
      its two caps with probability proportional to their areas; stack them.
      numpy only.
- [ ] **Step 4:** run, expect pass.
- [ ] **Step 5:** commit.

---

### Task 4: Emit the distinctive-object map, verifying uniqueness by measurement

**Files:**
- Modify: `map_tools/extract_park_map.py`
- Test: `map_tools/tests/test_extract_objects_map.py`

**Interfaces:**
- Produces: `maps/park_landmarks.yaml` — for each catalog object family, one
  descriptor (all instances of a family share a mesh, so compute once), the family's
  instance count, its `nearest_other` descriptor distance, and a `distinctive` flag.
  Plus per-instance `(x, y)` for the distinctive ones.

**The uniqueness rule (this is the heart of the plan):** a family is `distinctive`
only if **(a)** it has exactly ONE instance in the world, and **(b)** its
descriptor's distance to every other family's descriptor exceeds a threshold
chosen from the measured distribution. Condition (a) is what v1 lacked. Report the
full family-to-family distance matrix.

- [ ] **Step 1: failing test**

```python
# map_tools/tests/test_extract_objects_map.py
import yaml
from map_tools import extract_park_map

def test_only_the_single_instance_family_is_distinctive(tmp_path):
    extract_park_map.main(["--out-dir", str(tmp_path), "--landmarks"])
    d = yaml.safe_load(open(tmp_path / "park_landmarks.yaml"))
    fams = {k: v for k, v in d.items() if not k.startswith("_")}
    distinctive = {k for k, v in fams.items() if v["distinctive"]}
    assert distinctive == {"water_tower"}, distinctive
    # the repeated families must be excluded BY COUNT, not by shape luck
    assert fams["bench"]["count"] > 1 and not fams["bench"]["distinctive"]
    assert fams["water_tower"]["count"] == 1
```

- [ ] **Step 2:** run, expect fail.
- [ ] **Step 3:** implement a `--landmarks` path: for each catalog family sample
      its geometry (mesh via `sample_surface` / `.obj` via `obj_read`, or
      `sample_cylinder_stack` for the tower), `describe()` it, count its instances
      from `parse_models`, compute family-to-family `descriptor_distance`, apply
      the two-part rule, and write `maps/park_landmarks.yaml` with a `_meta` block
      recording the descriptor parameters (same self-describing-map convention the
      region map used — the runtime must not guess them).
- [ ] **Step 4:** run, expect pass. **Report the full family-to-family distance
      matrix** and the threshold chosen.
- [ ] **Step 5:** regenerate and commit `maps/park_landmarks.yaml`.

---

### Task 5: Blob detector — match a cluster to the distinctive object

**Files:**
- Create: `landmark_loc/blob_detector.py`
- Modify: `landmark_loc/detector.py` (register `"blob"`)
- Test: `landmark_loc/tests/test_blob_detector.py`

**Interfaces:**
- Produces: `BlobDetector(landmarks_path, match_threshold)`, `name="blob"`,
  `percept_based = False` (it consumes clusters + returns a position fix, like
  `RegionDetector`; reuse that flag so the A/B percept harness skips it).
  `match(clusters, prior_xy) -> (family, map_x, map_y, confidence) or None`:
  1. `describe()` each cluster (its own points, self-centred — NOT a space window).
  2. Compare against the distinctive families' descriptors only.
  3. Accept the best if under `match_threshold`; resolve WHICH instance via the
     prior (there is only one, so this is a sanity check, not a disambiguation).
  4. Return that instance's map `(x, y)`.

- [ ] **Step 1: failing test** — build a synthetic tower-shaped cluster from
      `sample_cylinder_stack`, and a bench-shaped one; assert the tower matches
      `water_tower` with the correct map coordinates and the bench returns None.
      Assert exact family and coordinates, with a tight justified threshold.
- [ ] **Step 2:** run, expect fail.
- [ ] **Step 3:** implement + register in `DETECTORS`.
- [ ] **Step 4:** run, expect pass, plus the whole suite.
- [ ] **Step 5:** commit.

---

### Task 6: Wire the blob detector into the localizer

**Files:**
- Modify: `landmark_loc/localizer_node.py`
- Test: `landmark_loc/tests/test_node_helpers.py` (extend)

Mirror the existing region wiring, which is already correct and stays in place:
`~classifier:=blob`, `~landmarks_path`, `~match_threshold`. In `on_cloud`, crop +
cluster with `segment` (as the cascade path does), call
`blob_detector.match(clusters, prior_xy)`, and on a hit set the anchor via
`_update_anchor` and publish `/odometry/landmark_fix`. Keep `/landmark_fault`.
Default `~classifier` stays `cascade`. **Leave the pre-existing uncommitted
`od.header.stamp = msg.header.stamp` change alone.**

- [ ] Failing helper test → run → implement → run whole suite → commit.

---

### Task 7: In-sim validation — MAIN CONVERSATION ONLY

Not for a subagent; requires the simulator.

- [ ] **Step 1:** bring up per `RUN-MAP-NAV.md` Steps 0–3 with `_classifier:=blob`
      (restart the operator container if stale).
- [ ] **Step 2:** confirm the tower is visible in the live cloud — points above
      z≈9 m within a few metres of (20, 14). Nothing else in the park is both that
      tall and that wide.
- [ ] **Step 3:** park the robot near the tower and measure the live cluster's
      descriptor against the map's. **Acceptance: the tower matches, and no
      furniture cluster does.** Set `match_threshold` from that measurement.
- [ ] **Step 4:** drive the 5-waypoint route
      (`route 40.84 8.19 35.84 13.19 5.84 13.19 -9.16 13.19 -14.16 3.19`) — it
      passes within 0.8 m of (20, 14) — and confirm a fix lands as the tower comes
      into range.
- [ ] **Step 5:** run the navsat spoof during the drive; confirm the
      descriptor-derived pose does not follow it.

**If Step 3 fails**, the remaining suspect is the same mesh-vs-lidar gap: the map
describes a whole object, the lidar sees its near face only. The fallback is to
build the tower's map descriptor from a **recorded live scan** of it rather than
from analytic sampling — a one-object survey, far cheaper than the full survey
drive the region approach would have needed.

## Risks

1. **One object means sparse coverage.** A fix is available only near (20, 5).
   That is the accepted cost of the simplest mechanism; the robot dead-reckons
   elsewhere. Add more distinct shapes later if coverage matters — each must be
   a *different* shape, never a copy.
2. **Near-face vs whole-object.** The known gap. Step 3 measures it; the
   one-object survey fallback is named above.
3. **Ground contamination is reduced but not zero.** Clustering already separates
   objects from ground (`segment.crop` has a `z_min`), so the 98%-ground problem
   that killed the region approach does not arise in the same way — but verify in
   Step 3 rather than assuming.
4. **The tower is synthetic.** A primitive cylinder stack is not a real-world
   object; a real deployment would key on real structures. Acceptable for a
   demonstrator, and noted so nobody mistakes it for a general claim.
