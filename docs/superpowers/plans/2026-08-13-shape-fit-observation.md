# Shape-Fit Observations + Bad-Fix Rejection Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax.

**Goal:** Estimate elongated landmark (bench/table) observation center + yaw by fitting the real footprint rectangle to the lidar cluster (ICP), thread yaw through catalog/observation/matcher as a frame-invariant pairwise yaw-difference, and reject physically-impossible landmark fixes before they reach the EKF.

**Architecture:** A new `shapefit.py` does ICP registration of a known rectangle to cluster points. `classify.to_observations` uses it for bench/table (round types + trees unchanged), emitting yaw. The extractor writes catalog yaw; the matcher adds a pairwise yaw-difference constraint (drift-immune). The localizer node adds an odom-referenced motion-jump gate on the published fix.

**Tech Stack:** Python 3, numpy, pytest. Files in `landmark_loc/` and `map_tools/`.

## Global Constraints

- Spec is authoritative: `docs/superpowers/specs/2026-08-13-shape-fit-observation-design.md`.
- Real footprints (metres): bench 1.78 × 0.80, garden_table 3.00 × 1.32. Round types lamp/trash_bin_1 and trees are UNCHANGED (measured adequate).
- Yaw is used ONLY as a frame-invariant pairwise DIFFERENCE (`yawA - yawB`); absolute yaw is never compared to the map. Pairs where either yaw is `None` fall back to distance-only matching.
- The motion gate is a pure OUTPUT filter — it never feeds back into the prior/anchor (NOT re-anchoring). A rejected fix publishes nothing (STALE that tick).
- Catalog regeneration must NOT change existing (x, y) — only add `yaw`. Existing entries' x/y stay byte-identical.
- No ground truth for pose (project rule): no `gazebo_msgs`.
- Run tests: `cd ~/Documents/Husky_viz/.worktrees/constellation-matcher && PYTHONPATH=. python3 -m pytest <files> -v`. The known-unrelated `test_launch.py::test_runbook_offers_both_modes` failure is pre-existing and out of scope.
- Keep: compass heading prior, buff_size, mode gate, tree trunk fix, solve_pose refit + residual gate, EKF wiring.

---

### Task 1: `shapefit.py` — ICP rectangle registration

**Files:**
- Create: `landmark_loc/shapefit.py`
- Test: `landmark_loc/tests/test_shapefit.py`

**Interfaces:**
- Produces: `fit_rectangle(points_xy, length, width) -> (cx, cy, yaw, ok)` where `points_xy` is an Nx2 numpy array (robot-frame cluster xy), `length`/`width` are the known footprint dims (metres), returns fitted center `(cx, cy)`, long-axis angle `yaw` (radians), and `ok` (bool — False if too sparse / didn't fit, caller falls back).

**Context:** Register the known rectangle OUTLINE (4 edges) to the observed points. Initial guess: PCA long axis → yaw, centroid → center. Then iterate: assign each point to the nearest rectangle edge (as a line segment), solve the small rigid update minimizing squared point-to-segment distances, apply, repeat to a fixed cap. The known size fills in the unseen back face.

- [ ] **Step 1: Write the failing tests**
```python
# landmark_loc/tests/test_shapefit.py
import math
import numpy as np
from landmark_loc.shapefit import fit_rectangle

BENCH_L, BENCH_W = 1.78, 0.80

def _rect_edge_points(cx, cy, yaw, length, width, n_per_edge=15, sides="all"):
    """Sample points on the rectangle outline at pose (cx,cy,yaw)."""
    hl, hw = length / 2.0, width / 2.0
    corners = [(-hl, -hw), (hl, -hw), (hl, hw), (-hl, hw)]
    c, s = math.cos(yaw), math.sin(yaw)
    pts = []
    edges = [(0, 1), (1, 2), (2, 3), (3, 0)]
    use = edges if sides == "all" else [edges[0]]  # "near" = one long edge
    for a, b in use:
        for t in np.linspace(0, 1, n_per_edge):
            lx = corners[a][0] + t * (corners[b][0] - corners[a][0])
            ly = corners[a][1] + t * (corners[b][1] - corners[a][1])
            pts.append((cx + c * lx - s * ly, cy + s * lx + c * ly))
    return np.array(pts, float)

def test_full_outline_recovers_pose():
    pts = _rect_edge_points(5.0, 2.0, 0.6, BENCH_L, BENCH_W, sides="all")
    cx, cy, yaw, ok = fit_rectangle(pts, BENCH_L, BENCH_W)
    assert ok
    assert abs(cx - 5.0) < 0.1 and abs(cy - 2.0) < 0.1
    # yaw modulo pi (a rectangle is symmetric under 180deg)
    dyaw = (yaw - 0.6) % math.pi
    assert min(dyaw, math.pi - dyaw) < 0.1

def test_L_shape_two_edges_recovers_center():
    hl, hw = BENCH_L / 2, BENCH_W / 2
    # near long edge + one short end (an L), at pose (3,-1,0)
    pts = _rect_edge_points(3.0, -1.0, 0.0, BENCH_L, BENCH_W, sides="all")
    # keep only points on the near long edge and one end (simulate partial view)
    keep = (pts[:, 1] < -1.0 + 0.05) | (pts[:, 0] > 3.0 + hl - 0.05)
    cx, cy, yaw, ok = fit_rectangle(pts[keep], BENCH_L, BENCH_W)
    assert ok
    assert abs(cx - 3.0) < 0.2 and abs(cy - (-1.0)) < 0.2

def test_sparse_returns_not_ok():
    pts = np.array([[1.0, 1.0], [1.1, 1.0]])  # 2 points, too few
    cx, cy, yaw, ok = fit_rectangle(pts, BENCH_L, BENCH_W)
    assert not ok
```

- [ ] **Step 2: Run — expect fail** (module missing)
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_shapefit.py -v`

- [ ] **Step 3: Implement `landmark_loc/shapefit.py`**
```python
"""Fit a known rectangle footprint to a lidar cluster (robot-frame xy) by ICP.

The lidar sees only the near face(s) of an object; for an elongated bench/table
that is a near-edge line or L-shape, not the full footprint, so the visible-
points centroid is a biased estimate of the true center. We instead register the
object's KNOWN rectangle outline to the points and read off the true center and
orientation. Import-free of ROS.
"""
import math
import numpy as np

_MIN_PTS = 6
_MAX_ITERS = 20
_CONV_EPS = 1e-3   # metres; stop when the update translation is tiny


def _pca_yaw(xy):
    c = xy.mean(axis=0)
    u, s, vt = np.linalg.svd(xy - c)
    v = vt[0]                      # principal (long-axis) direction
    return math.atan2(v[1], v[0])


def _rect_segments(cx, cy, yaw, length, width):
    """Return the 4 edges as (p0, p1) segment endpoints in world (robot) frame."""
    hl, hw = length / 2.0, width / 2.0
    corners = np.array([[-hl, -hw], [hl, -hw], [hl, hw], [-hl, hw]], float)
    c, s = math.cos(yaw), math.sin(yaw)
    R = np.array([[c, -s], [s, c]])
    w = (R @ corners.T).T + np.array([cx, cy])
    return [(w[0], w[1]), (w[1], w[2]), (w[2], w[3]), (w[3], w[0])]


def _closest_on_segment(p, a, b):
    ab = b - a
    t = np.dot(p - a, ab) / max(np.dot(ab, ab), 1e-12)
    t = min(1.0, max(0.0, t))
    return a + t * ab


def _nearest_outline_pts(pts, cx, cy, yaw, length, width):
    segs = _rect_segments(cx, cy, yaw, length, width)
    out = np.empty_like(pts)
    for i, p in enumerate(pts):
        best, bd = None, 1e18
        for a, b in segs:
            q = _closest_on_segment(p, a, b)
            d = np.dot(p - q, p - q)
            if d < bd:
                bd, best = d, q
        out[i] = best
    return out


def fit_rectangle(points_xy, length, width):
    pts = np.asarray(points_xy, float).reshape(-1, 2)
    if len(pts) < _MIN_PTS:
        return 0.0, 0.0, 0.0, False
    cx, cy = pts.mean(axis=0)
    yaw = _pca_yaw(pts)
    for _ in range(_MAX_ITERS):
        targets = _nearest_outline_pts(pts, cx, cy, yaw, length, width)
        # Solve rigid (Umeyama, no scale) mapping current model->targets, i.e.
        # move the rectangle so its outline sits under the points. We compute the
        # transform that best moves pts onto targets, then apply the INVERSE to the
        # rectangle pose (equivalently move the rectangle toward the points).
        src_c = pts.mean(axis=0)
        dst_c = targets.mean(axis=0)
        H = (pts - src_c).T @ (targets - dst_c)
        U, _, Vt = np.linalg.svd(H)
        R = Vt.T @ U.T
        if np.linalg.det(R) < 0:
            Vt[-1, :] *= -1
            R = Vt.T @ U.T
        dyaw = math.atan2(R[1, 0], R[0, 0])
        t = dst_c - R @ src_c
        # apply this small motion to the rectangle center+yaw
        new_c = R @ np.array([cx, cy]) + t
        step = math.hypot(new_c[0] - cx, new_c[1] - cy)
        cx, cy = float(new_c[0]), float(new_c[1])
        yaw += dyaw
        if step < _CONV_EPS:
            break
    return cx, cy, yaw, True
```

- [ ] **Step 4: Run tests — expect pass**
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_shapefit.py -v`

- [ ] **Step 5: Commit**
```bash
git add landmark_loc/shapefit.py landmark_loc/tests/test_shapefit.py
git commit -m "feat(shapefit): ICP rectangle registration for elongated landmarks"
```

---

### Task 2: Observation carries yaw; bench/table use the shape fit

**Files:**
- Modify: `landmark_loc/classify.py`
- Test: `landmark_loc/tests/test_classify.py` (extend)

**Interfaces:**
- Consumes: `shapefit.fit_rectangle(points_xy, length, width) -> (cx, cy, yaw, ok)`.
- Produces: `Observation(identity, x, y, yaw)` — `yaw` is a float (radians) for bench/table, `None` for lamp/bin/tree. `to_observations(clusters, margins=DEFAULT_MARGINS)` unchanged signature, now emits yaw.

**Context:** Add `yaw: float | None = None` to `Observation`. Add a `_RECT_FOOTPRINT = {"bench": (1.78, 0.80), "garden_table": (3.00, 1.32)}`. In `to_observations`, for bench/table call `shapefit.fit_rectangle(c.points[:, :2], L, W)`; if `ok`, use `(cx, cy)` as the center and set `yaw`; if not `ok`, fall back to the current centroid + push-out and `yaw=None`. Round types (lamp/bin) and tree keep exactly current logic with `yaw=None`.

- [ ] **Step 1: Write the failing tests** (append to `test_classify.py`)
```python
import math
import numpy as np
from landmark_loc.classify import Observation, to_observations
from landmark_loc.segment import Cluster

def _bench_cluster_from_outline():
    # a bench outline at robot-frame (5,0), yaw 0 -> full rectangle points
    import numpy as np, math
    L, W = 1.78, 0.80
    hl, hw = L/2, W/2
    corners = [(-hl,-hw),(hl,-hw),(hl,hw),(-hl,hw)]
    edges=[(0,1),(1,2),(2,3),(3,0)]; pts=[]
    for a,b in edges:
        for t in np.linspace(0,1,15):
            lx=corners[a][0]+t*(corners[b][0]-corners[a][0])
            ly=corners[a][1]+t*(corners[b][1]-corners[a][1])
            pts.append((5.0+lx, 0.0+ly))
    xy=np.array(pts); z=np.full((len(xy),1),0.4)
    p3=np.hstack([xy,z])
    return Cluster(points=p3, centroid_xy=(float(xy[:,0].mean()),float(xy[:,1].mean())),
                   major=L, minor=W, height=0.4)

def test_bench_observation_has_yaw_and_fit_center():
    c = _bench_cluster_from_outline()
    obs = to_observations([c])
    assert len(obs) == 1
    o = obs[0]
    assert o.identity == "bench"
    assert o.yaw is not None
    # fit center near the true (5,0), not just the centroid
    assert abs(o.x - 5.0) < 0.2 and abs(o.y - 0.0) < 0.2

def test_lamp_observation_yaw_is_none():
    # a compact lamp cluster: tall, narrow (round type keeps centroid+pushout)
    xy = np.array([[3.0,0.0],[3.05,0.02],[2.98,-0.03],[3.02,0.01]])
    z = np.linspace(0, 3.15, len(xy)).reshape(-1,1)
    p3 = np.hstack([xy, z])
    c = Cluster(points=p3, centroid_xy=(3.0,0.0), major=0.63, minor=0.48, height=3.15)
    obs = to_observations([c])
    assert len(obs) == 1 and obs[0].identity == "lamp" and obs[0].yaw is None
```

- [ ] **Step 2: Run — expect fail** (`Observation` has no `yaw`; bench center is centroid, not fit)
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_classify.py -v`

- [ ] **Step 3: Implement.** In `classify.py`:
  Add import: `from landmark_loc import shapefit`.
  Change the dataclass:
```python
@dataclass
class Observation:
    identity: str
    x: float
    y: float
    yaw: float = None
```
  Add near `KNOWN_RADIUS`:
```python
# Real rectangle footprints (length, width) in metres, from the mesh signatures.
# Only elongated types get the ICP shape fit; round types keep centroid+pushout.
_RECT_FOOTPRINT = {"bench": (1.78, 0.80), "garden_table": (3.00, 1.32)}
```
  Rewrite the position section of `to_observations` so the loop body is:
```python
        ident = classify_cluster(c, margins)
        if ident == "unknown":
            continue
        yaw = None
        if ident in _RECT_FOOTPRINT:
            L, W = _RECT_FOOTPRINT[ident]
            fx, fy, fyaw, ok = shapefit.fit_rectangle(c.points[:, :2], L, W)
            if ok:
                out.append(Observation(identity=ident, x=fx, y=fy, yaw=fyaw))
                continue
            # fall through to centroid+pushout on a failed fit
        if ident == "tree":
            trunk = _trunk_xy(c.points)
            cx, cy = trunk if trunk is not None else c.centroid_xy
        else:
            cx, cy = c.centroid_xy
        r = math.hypot(cx, cy)
        radius = KNOWN_RADIUS.get(ident, 0.0)
        if r > 1e-6 and radius > 0.0:
            ux, uy = cx / r, cy / r
            ox, oy = cx + radius * ux, cy + radius * uy
        else:
            ox, oy = cx, cy
        out.append(Observation(identity=ident, x=ox, y=oy, yaw=yaw))
```

- [ ] **Step 4: Run tests — expect pass** (existing classify tests still pass; new yaw tests pass)
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_classify.py -v`

- [ ] **Step 5: Commit**
```bash
git add landmark_loc/classify.py landmark_loc/tests/test_classify.py
git commit -m "feat(classify): bench/table observation via shape fit + yaw; round types unchanged"
```

---

### Task 3: Catalog carries yaw (extractor writes it, loader reads it)

**Files:**
- Modify: `map_tools/extract_park_map.py`
- Modify: `landmark_loc/catalog.py`
- Modify: `maps/park_places.yaml` (regenerate)
- Test: `landmark_loc/tests/test_catalog.py` (extend)

**Interfaces:**
- Consumes: `sdf_parse.Model.yaw` (already parsed, world-frame radians).
- Produces: `MapLandmark(name, identity, x, y, yaw)` — `yaw` float radians or `None` if absent in yaml. `catalog.load(path) -> list[MapLandmark]` unchanged signature.

**Context:** The extractor already reads `m.yaw`. Write it into each place entry. `catalog.load` reads `yaw` if present (default `None`). Only bench/table need meaningful yaw for matching; writing it for all is harmless. CRITICAL: regenerating must not change existing x/y — verify.

- [ ] **Step 1: Write the failing tests** (append to `test_catalog.py`)
```python
import tempfile, os
from landmark_loc.catalog import load, MapLandmark

def test_load_reads_yaw_when_present():
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write("bench: {x: 1.0, y: 2.0, yaw: 0.5}\n")
        f.write("lamp_clone: {x: 3.0, y: 4.0}\n")   # no yaw -> None
        path = f.name
    lms = {l.name: l for l in load(path)}
    os.unlink(path)
    assert abs(lms["bench"].yaw - 0.5) < 1e-9
    assert lms["lamp_clone"].yaw is None
```

- [ ] **Step 2: Run — expect fail** (`MapLandmark` has no `yaw`)
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_catalog.py -v`

- [ ] **Step 3: Implement.** In `catalog.py`, change the dataclass and loader:
```python
@dataclass
class MapLandmark:
    name: str
    identity: str
    x: float
    y: float
    yaw: float = None
```
```python
        out.append(MapLandmark(name, fam, float(xy["x"]), float(xy["y"]),
                               float(xy["yaw"]) if "yaw" in xy else None))
```
  In `extract_park_map.py`, change `build_places` and `_write_places_yaml`:
```python
def build_places(models):
    places = {}
    for m in models:
        if m.family in PLACE_FAMILIES:
            places[m.name] = {"x": round(m.world_x, 3), "y": round(m.world_y, 3),
                              "yaw": round(m.yaw, 4)}
    return places
```
```python
        for name in sorted(places):
            p = places[name]
            fh.write("%s: {x: %.3f, y: %.3f, yaw: %.4f}\n"
                     % (name, p["x"], p["y"], p["yaw"]))
```

- [ ] **Step 4: Run tests — expect pass**
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_catalog.py -v`

- [ ] **Step 5: Regenerate the worktree catalog and verify x/y unchanged**

The extractor writes `park_places.yaml` (and the map pgm) into `--out-dir`. The
worktree catalog is `maps/park_places.yaml`, so regenerate into `maps/` and
verify only yaw was added.
```bash
cp maps/park_places.yaml /tmp/places_before.yaml
PYTHONPATH=. python3 map_tools/extract_park_map.py \
  --world natural_environments_ros_opt/natural_enviroment/worlds/park.world \
  --out-dir maps
# verify: every x and y is byte-identical to before (yaml has yaw added)
PYTHONPATH=. python3 -c "
import yaml
a=yaml.safe_load(open('/tmp/places_before.yaml')); b=yaml.safe_load(open('maps/park_places.yaml'))
assert set(a)==set(b), 'name set changed'
for k in a:
    assert abs(a[k]['x']-b[k]['x'])<1e-9 and abs(a[k]['y']-b[k]['y'])<1e-9, ('x/y changed for '+k)
    assert 'yaw' in b[k], ('yaw missing for '+k)
print('OK: x/y unchanged for all', len(a), 'entries; yaw added')
"
# NOTE: regenerating also rewrites maps/park_map.pgm/.yaml into maps/. If that
# changes files you don't want touched, restore them: `git checkout maps/park_map.*`
```
Expected: `OK: x/y unchanged ...`.

- [ ] **Step 6: Commit**
```bash
git add map_tools/extract_park_map.py landmark_loc/catalog.py maps/park_places.yaml landmark_loc/tests/test_catalog.py
git commit -m "feat(catalog): store per-landmark yaw (extractor writes, loader reads)"
```

---

### Task 4: Matcher uses pairwise yaw-difference (frame-invariant)

**Files:**
- Modify: `landmark_loc/constellation.py`
- Test: `landmark_loc/tests/test_constellation.py` (extend)

**Interfaces:**
- Consumes: `Observation.yaw`, `MapLandmark.yaw` (float or None).
- Produces: `match(...)` unchanged signature; additionally constrains seed pairs by yaw-difference when both objects in a pair have a yaw.

**Context:** Add a `_YAW_TOL` module constant (radians). In the seed loop, when the observed pair AND the chosen catalog orientation both have yaws (all four not None), require the observed yaw-difference to match the catalog yaw-difference within `_YAW_TOL` (comparing modulo pi, since a rectangle is 180deg-symmetric). If any of the four yaws is None, skip the yaw check (distance-only, as today). This is frame-invariant: rotating the whole scene changes each yaw by the same amount, so the DIFFERENCE is unchanged.

- [ ] **Step 1: Write the failing tests** (append to `test_constellation.py`)
```python
import math
from landmark_loc.constellation import match
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark

def _o(ident, x, y, yaw=None): return Observation(ident, x, y, yaw)
def _l(name, ident, x, y, yaw=None): return MapLandmark(name, ident, x, y, yaw)

def test_yaw_diff_rejects_wrong_orientation_pair():
    # Two benches whose catalog yaw-difference is ~0 (parallel). An observed pair
    # with the SAME distance but a 90deg yaw-difference must NOT seed-match them.
    cat = [_l("bA","bench",10.0,0.0,0.0), _l("bB","bench",13.0,0.0,0.0),
           _l("lC","lamp",10.0,5.0,None)]
    # observed: correct positions but bench yaws differ by 90deg (wrong)
    obs = [_o("bench",10.0,0.0,0.0), _o("bench",13.0,0.0,math.pi/2),
           _o("lamp",10.0,5.0,None)]
    pairs = match(obs, cat, (0.0,0.0,0.0), 1.0)
    names = sorted(l.name for _,l in pairs)
    # the bench-bench yaw mismatch blocks that seed; with only 1 usable
    # correspondence type left, cannot reach 3 -> []
    assert pairs == [] or "bB" not in names

def test_yaw_diff_frame_invariant_still_matches_when_rotated():
    # correct scene, benches parallel (yaw-diff 0) in both catalog and obs,
    # whole observed scene rotated by 0.7 rad (obs yaws all +0.7) -> yaw-DIFF
    # unchanged -> still matches.
    cat = [_l("bA","bench",10.0,0.0,0.2), _l("bB","bench",13.0,0.0,0.2),
           _l("tC","trash_bin_1",8.0,5.0,None)]
    rot=0.7
    obs = [_o("bench",10.0,0.0,0.2+rot), _o("bench",13.0,0.0,0.2+rot),
           _o("trash_bin_1",8.0,5.0,None)]
    pairs = match(obs, cat, (0.0,0.0,0.0), 1.0)
    assert len(pairs) >= 3

def test_none_yaw_pairs_match_distance_only():
    # all round types (no yaw) -> behaves exactly like distance-only today
    cat = [_l("l1","lamp",10.0,0.0), _l("l2","lamp",12.5,0.0),
           _l("t1","trash_bin_1",8.0,5.0)]
    obs = [_o(l.identity,l.x,l.y) for l in cat]
    pairs = match(obs, cat, (0.0,0.0,0.0), 1.0)
    assert len(pairs) >= 3
```

- [ ] **Step 2: Run — expect fail** (yaw not used yet; the wrong-orientation pair currently matches)
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_constellation.py -v`

- [ ] **Step 3: Implement.** In `constellation.py` add:
```python
_YAW_TOL = 0.35   # rad (~20deg): pairwise yaw-diff tolerance for seed matching


def _yaw_diff_ok(oi, oj, ci, cj):
    """True if all four yaws present AND the observed pair yaw-difference matches
    the catalog pair yaw-difference within _YAW_TOL (modulo pi, rectangle-
    symmetric). If any yaw is None, returns True (no yaw constraint)."""
    ys = (oi.yaw, oj.yaw, ci.yaw, cj.yaw)
    if any(y is None for y in ys):
        return True
    d_obs = (oi.yaw - oj.yaw) % math.pi
    d_cat = (ci.yaw - cj.yaw) % math.pi
    dd = abs(d_obs - d_cat) % math.pi
    return min(dd, math.pi - dd) <= _YAW_TOL
```
  In the `match` seed loop, inside the `_seed_orientations` loop, gate the seed:
```python
                for cat_i, cat_j in _seed_orientations(oi, oj, a, b):
                    if not _yaw_diff_ok(oi, oj, cat_i, cat_j):
                        continue
                    tx, ty, yaw, _ = rigid_transform_2d(
                        [[oi.x, oi.y], [oj.x, oj.y]],
                        [[cat_i.x, cat_i.y], [cat_j.x, cat_j.y]])
                    ...
```

- [ ] **Step 4: Run tests — expect pass** (new yaw tests pass; ALL existing constellation drift-immunity tests still pass)
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_constellation.py -v`

- [ ] **Step 5: Commit**
```bash
git add landmark_loc/constellation.py landmark_loc/tests/test_constellation.py
git commit -m "feat(constellation): frame-invariant pairwise yaw-diff seed constraint"
```

---

### Task 5: Motion-jump gate — reject physically-impossible fixes

**Files:**
- Modify: `landmark_loc/localizer_node.py`
- Test: `landmark_loc/tests/test_node_helpers.py` (extend)

**Interfaces:**
- Produces: pure helper `_jump_ok(fix_xy, last_pub_xy, odom_disp, max_jump) -> bool` — True if `fix_xy` is within `max_jump` metres of `last_pub_xy + odom_disp` (i.e. physically reachable), OR if `last_pub_xy is None` (bootstrap: always accept).

**Context:** Add the helper and use it in `on_cloud`: track `state["last_pub_xy"]` and the odom pose captured at last publish (`state["last_pub_odom"]`). When a fix is computed, compute `odom_disp = (odom_now.x - last_pub_odom.x, odom_now.y - last_pub_odom.y)` and reject via `_jump_ok`. On reject: log a rejected-jump STALE line, return (publish nothing). On accept: publish, then update `last_pub_xy` and `last_pub_odom`. Add `~max_jump` param (default 3.0). This is a pure OUTPUT filter — does NOT touch the anchor/prior.

- [ ] **Step 1: Write the failing tests** (append to `test_node_helpers.py`)
```python
from landmark_loc.localizer_node import _jump_ok

def test_jump_bootstrap_accepts():
    assert _jump_ok((5.0, 0.0), None, (0.0, 0.0), 3.0) is True

def test_jump_within_reach_accepts():
    # last pub (10,0); odom moved (-2,0); expected (8,0); fix (8.3,0.1) close -> ok
    assert _jump_ok((8.3, 0.1), (10.0, 0.0), (-2.0, 0.0), 3.0) is True

def test_backward_teleport_rejected():
    # last pub (10,0); odom moved (-2,0) forward; expected ~(8,0);
    # fix jumps BACKWARD to (18,0) -> 10m from expected -> reject
    assert _jump_ok((18.0, 0.0), (10.0, 0.0), (-2.0, 0.0), 3.0) is False
```

- [ ] **Step 2: Run — expect fail** (`_jump_ok` missing)
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_node_helpers.py -v`

- [ ] **Step 3: Implement.** In `localizer_node.py` add the helper (near `_is_landmark_mode`):
```python
def _jump_ok(fix_xy, last_pub_xy, odom_disp, max_jump):
    """Physical-motion gate: a fix must land within max_jump of where the robot
    can be (last published pose advanced by odom displacement). Bootstrap
    (no last_pub_xy) always accepts. Pure output filter; never re-anchors."""
    if last_pub_xy is None:
        return True
    ex = last_pub_xy[0] + odom_disp[0]
    ey = last_pub_xy[1] + odom_disp[1]
    return math.hypot(fix_xy[0] - ex, fix_xy[1] - ey) <= max_jump
```
  Add to the params dict: `max_jump=rospy.get_param("~max_jump", 3.0),`.
  Add to the state dict: `"last_pub_xy": None, "last_pub_odom": None,`.
  In `on_cloud`, AFTER `x, y, yaw, rms, n = result` and BEFORE publishing, insert:
```python
        # Physical-motion gate: reject a fix that teleports beyond reachable.
        if state["last_pub_odom"] is not None:
            odom_disp = (odom_synced[0] - state["last_pub_odom"][0],
                         odom_synced[1] - state["last_pub_odom"][1])
        else:
            odom_disp = (0.0, 0.0)
        if not _jump_ok((x, y), state["last_pub_xy"], odom_disp, p["max_jump"]):
            rospy.loginfo_throttle(0.5,
                "[diag] obs=%d assoc=%d prior=(%.1f,%.1f) REJECT-JUMP fix=(%.2f,%.2f)"
                % (len(obs), len(_pairs), prior[0], prior[1], x, y))
            return
```
  After `pub.publish(od)` (and `state["last_pub"] = now`), add:
```python
        state["last_pub_xy"] = (x, y)
        state["last_pub_odom"] = odom_synced
```
  (`odom_synced` is the interpolated odom already computed earlier in `on_cloud`.)

- [ ] **Step 4: Run tests — expect pass**
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/test_node_helpers.py -v`

- [ ] **Step 5: Run full suite**
Run: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/ -v` → all pass except the known `test_launch.py::test_runbook_offers_both_modes`.

- [ ] **Step 6: Commit**
```bash
git add landmark_loc/localizer_node.py landmark_loc/tests/test_node_helpers.py
git commit -m "feat(localizer): motion-jump gate rejects physically-impossible fixes"
```

---

### Task 6: Remove the temporary stale-diag; in-sim acceptance (RUN BY MAIN)

**Files:**
- Modify: `landmark_loc/localizer_node.py` (remove the TEMP STALE DIAGNOSTIC block from commit c0bac44)

- [ ] **Step 1: Remove the temp diagnostic.** Delete the block between
  `# TEMP STALE DIAGNOSTIC — remove after root-cause` and
  `# END TEMP STALE DIAGNOSTIC` in `on_cloud`. Run the suite to confirm nothing
  breaks: `PYTHONPATH=. python3 -m pytest landmark_loc/tests/ -v`.
- [ ] **Step 2: Commit**
```bash
git add landmark_loc/localizer_node.py
git commit -m "chore(localizer): remove temporary stale diagnostic"
```
- [ ] **Step 3 (MAIN, not a subagent): In-sim acceptance.** Full RUN-MAP-NAV Steps
  0–3 VERBATIM (each block with its own env line, from a clean kill). Drive in
  landmark mode. Judge by GAZEBO, not move_base SUCCEEDED:
  - STALE fraction drops vs the ~32% baseline.
  - No backward-teleport jumps in RViz (the motion gate fires — watch for
    REJECT-JUMP diag lines).
  - Bench/table observations track their catalog positions (fixes stay alive
    through the drive).
  - The robot's actual Gazebo position reaches the goal marker.
- [ ] **Step 4:** Tear down clean (kill by exact PID, verify master DOWN). Record
  the Gazebo-judged result in the SDD ledger.

---

## Self-Review

**Spec coverage:** shape-fit ICP (Task 1), bench/table observation + yaw + round-unchanged (Task 2), catalog yaw via extractor+loader with x/y-unchanged check (Task 3), pairwise yaw-diff frame-invariant matching with None fallback (Task 4), motion-jump reject not-re-anchoring (Task 5), temp-diag removal + Gazebo-judged in-sim (Task 6). All spec sections covered.

**Placeholder scan:** every step has real code + concrete constants (footprints 1.78×0.80 / 3.00×1.32, `_YAW_TOL=0.35`, `max_jump=3.0`, `_MIN_PTS=6`). No TBDs. Task 3 Step 5 flags to confirm the extractor's actual `--places` flag from its argparse.

**Type consistency:** `Observation(identity,x,y,yaw=None)` and `MapLandmark(name,identity,x,y,yaw=None)` defined once (Tasks 2, 3) and consumed consistently (Task 4 `_yaw_diff_ok`, Task 5). `fit_rectangle(points_xy,length,width)->(cx,cy,yaw,ok)` defined Task 1, called Task 2 with `c.points[:, :2]`. `_jump_ok(fix_xy,last_pub_xy,odom_disp,max_jump)->bool` defined and used Task 5. `_RECT_FOOTPRINT` keys (`bench`,`garden_table`) match identity strings used throughout.

**Known conflict flagged:** Task 2 and Task 4 extend existing tests (classify, constellation) — intended; the new tests assert the stronger properties (fit center, yaw-diff) and existing drift-immunity tests must continue to pass (explicit in Task 4 Step 4).
