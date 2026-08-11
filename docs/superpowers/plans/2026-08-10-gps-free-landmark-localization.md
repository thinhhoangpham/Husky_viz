# GPS-Free Landmark Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a GPS-free navigation mode where the robot localizes by recognizing typed park landmarks in the live lidar and matching them against a known catalog, publishing an absolute pose fix that the existing map-EKF fuses in place of GPS.

**Architecture:** One new node (`landmark_localizer`) segments `/os0_cloud_node/points`, classifies clusters into bench/table/lamp/bin by 3D shape (footprint + height), matches the typed constellation against `maps/park_places.yaml` seeded by the compass+odom prior, solves a 2D rigid transform for `(x,y,yaw)`, and publishes `nav_msgs/Odometry` on `/odometry/abs_fix`. The map-EKF's `odom1` input is renamed from the GPS-specific `odometry/gps` to the neutral `odometry/abs_fix`; GPS mode feeds that topic via navsat (unchanged behavior), landmark mode feeds it via the localizer (navsat not started). A sibling launch file `move_base_landmark.launch` selects the mode.

**Tech Stack:** Python 3, ROS Noetic (rospy), `sensor_msgs/PointCloud2`, `nav_msgs/Odometry`, `tf2_ros`, NumPy (Umeyama/Kabsch solve), existing `map_tools/mesh_bounds.py`, pytest.

## Global Constraints

- **No ground truth, ever.** No `/gazebo/*` topics, no `gazebo_msgs` import, no constant obtained by measuring simulator internal state. Verification is by the robot's own sensors and the operator's RViz/Gazebo view only. (CLAUDE.md standing rule.)
- **Do not modify the map-EKF internals** (`localization_map.yaml`) except the single `odom1` topic rename on line 93. No change to `two_d_mode`, `frequency`, compass (`imu1`), or wheel (`odom0`) config.
- **GPS mode must remain fully working** after all changes — the spoof demo depends on it. The `odom1` rename plus a navsat output remap must be behavior-preserving for GPS mode.
- **Cloud facts (from URDF):** topic `/os0_cloud_node/points`, frame `os0_lidar`, 10 Hz, `min_range 0.9`, `max_range 50.0`, `noise 0.0`. TF `os0_lidar`→`base_link` exists via the robot description.
- **Catalog is the 53 typed non-tree landmarks** already in `maps/park_places.yaml` (16 bench, 15 lamp, 11 garden_table, 11 trash_bin_1). Trees are excluded from identity (obstacle-only).
- **Meshes** (under `models_opt/`, per-mesh scale): `bench/Bench_1.dae` (0.15), `garden_table/garden_table.dae` (1.0), `lamp/street_lamp.dae` (1.0), `trash_bin_1/trash_bin.dae` (1.0). Scales confirmed for bench/table in `extract_park_map.py`; lamp/bin scales pinned in Task 1.
- **Landmark descriptor is `{identity, x, y}`** with `identity` an opaque string label — the matcher must read it only as an equality key, so a future camera/fiducial identity source can populate it without matcher changes.
- **Conservative classification:** a cluster ambiguous between two types is labeled `unknown` and dropped. A wrong label is worse than a missing one.
- **Fit-gated output:** publish a fix only when ≥2 identity-consistent correspondences solve with RMS residual below threshold; otherwise publish nothing (EKF coasts on odom — coast-and-recover only, no global re-localization in v1).
- **Position-only fusion:** the EKF keeps `imu1 = compass/data` for yaw. The solve's yaw is used internally to gate/validate the fit but is NOT fused.
- All new Python lives under `landmark_loc/` (new package dir) with a `tests/` subdir; pure-logic functions are unit-tested without ROS. Absolute config/launch paths per repo convention.

---

### Task 1: Mesh-derived landmark signatures + 3D bounds helper

**Files:**
- Modify: `map_tools/mesh_bounds.py` (add `bounds3d`)
- Create: `landmark_loc/__init__.py` (empty package marker)
- Create: `landmark_loc/signatures.py`
- Test: `landmark_loc/tests/test_signatures.py`

**Interfaces:**
- Consumes: `mesh_bounds.footprint(dae_path, scale) -> (half_dx, half_dy, cx, cy)` (existing).
- Produces:
  - `mesh_bounds.bounds3d(dae_path, scale) -> (half_dx, half_dy, half_dz, cx, cy, cz)` — same node-transform-aware walk as `footprint`, but also returns z half-extent and z center.
  - `signatures.MESH_SIGNATURES: dict[str, dict]` — for each of `bench|garden_table|lamp|trash_bin_1`, `{ "major": float, "minor": float, "height": float }` in metres, computed from the mesh (major = larger horizontal full-extent, minor = smaller, height = full z-extent).
  - `signatures.SIGNATURE_FAMILIES: tuple` — the four family keys, in the order above.

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_signatures.py
import math
from map_tools import mesh_bounds
from landmark_loc import signatures


def test_bounds3d_returns_six_values_and_positive_extents():
    import os
    root = os.path.join(os.path.dirname(__file__), "..", "..", "models_opt")
    hx, hy, hz, cx, cy, cz = mesh_bounds.bounds3d(
        os.path.join(root, "bench", "Bench_1.dae"), 0.15)
    assert hx > 0 and hy > 0 and hz > 0
    # bench full-extents in metres are all sub-3m at 0.15 scale
    assert 2 * hx < 3.0 and 2 * hy < 3.0 and 2 * hz < 3.0


def test_signatures_cover_four_families_with_ordered_dims():
    for fam in signatures.SIGNATURE_FAMILIES:
        sig = signatures.MESH_SIGNATURES[fam]
        assert sig["major"] >= sig["minor"] > 0
        assert sig["height"] > 0


def test_bench_is_elongated_and_low_lamp_is_thin_and_tall():
    bench = signatures.MESH_SIGNATURES["bench"]
    lamp = signatures.MESH_SIGNATURES["lamp"]
    # bench: high horizontal aspect ratio (a bar)
    assert bench["major"] / bench["minor"] > 1.8
    # lamp: tall relative to its footprint
    assert lamp["height"] / max(lamp["major"], 0.01) > 3.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_signatures.py -v`
Expected: FAIL — `mesh_bounds.bounds3d` and `landmark_loc.signatures` do not exist.

- [ ] **Step 3: Add `bounds3d` to `mesh_bounds.py`**

Add after `footprint` (mirror its node-transform walk, but track z):

```python
def bounds3d(dae_path, scale=0.15):
    """Like footprint() but also returns z half-extent and z center.

    Applies each visual_scene node's <matrix> to its geometry vertices before
    bounding (same as footprint), so translated/rotated geometry is handled.
    Returns (half_dx, half_dy, half_dz, cx, cy, cz), all scaled to metres.
    """
    txt = _strip_namespace(open(dae_path).read())
    root = ET.fromstring(txt)

    geom_positions = {}
    for geom in root.findall(".//geometry"):
        gid = geom.get("id")
        verts = []
        for arr in geom.findall(".//float_array"):
            arr_id = arr.get("id") or ""
            if "positions" not in arr_id.lower():
                continue
            vals = _parse_floats(arr.text or "")
            verts.extend(zip(vals[0::3], vals[1::3], vals[2::3]))
        if verts:
            geom_positions.setdefault(gid, []).extend(verts)

    xs, ys, zs = [], [], []
    for node in root.findall(".//visual_scene//node"):
        ig = node.find(".//instance_geometry")
        if ig is None:
            continue
        gid = ig.get("url", "").lstrip("#")
        verts = geom_positions.get(gid)
        if not verts:
            continue
        mat_el = node.find("matrix")
        if mat_el is not None:
            m = _parse_floats(mat_el.text)
            R = [m[0], m[1], m[2], m[4], m[5], m[6], m[8], m[9], m[10]]
            T = [m[3], m[7], m[11]]
        else:
            R, T = _identity()
        for vx, vy, vz in verts:
            xs.append(R[0] * vx + R[1] * vy + R[2] * vz + T[0])
            ys.append(R[3] * vx + R[4] * vy + R[5] * vz + T[1])
            zs.append(R[6] * vx + R[7] * vy + R[8] * vz + T[2])

    if not xs:
        raise ValueError("no positions arrays in %s" % dae_path)

    half_dx = (max(xs) - min(xs)) / 2.0 * scale
    half_dy = (max(ys) - min(ys)) / 2.0 * scale
    half_dz = (max(zs) - min(zs)) / 2.0 * scale
    cx = (min(xs) + max(xs)) / 2.0 * scale
    cy = (min(ys) + max(ys)) / 2.0 * scale
    cz = (min(zs) + max(zs)) / 2.0 * scale
    return half_dx, half_dy, half_dz, cx, cy, cz
```

- [ ] **Step 4: Create `landmark_loc/__init__.py`** (empty file).

- [ ] **Step 5: Create `landmark_loc/signatures.py`**

```python
"""Mesh-derived shape signatures for the four identifiable park landmark types.

Each signature is the object's true major/minor horizontal full-extent and full
height, read from its .dae via mesh_bounds.bounds3d. The live classifier
(classify.py) uses these as the CENTERS of its type bands; the +/- margins that
absorb partial lidar views are tuned in-sim and live in classify.py, not here.
Keeping the raw geometry here means the sim and the classifier agree by
construction.
"""
import os
from map_tools import mesh_bounds

_MODELS_ROOT = os.path.join(os.path.dirname(__file__), "..", "models_opt")

# family -> (relative mesh path, per-mesh scale). Scales: bench/table confirmed
# in extract_park_map.py; lamp/bin default 1.0, pinned against live lidar in the
# in-sim step of this task.
_MESHES = {
    "bench":        (("bench", "Bench_1.dae"), 0.15),
    "garden_table": (("garden_table", "garden_table.dae"), 1.0),
    "lamp":         (("lamp", "street_lamp.dae"), 1.0),
    "trash_bin_1":  (("trash_bin_1", "trash_bin.dae"), 1.0),
}

SIGNATURE_FAMILIES = ("bench", "garden_table", "lamp", "trash_bin_1")


def _signature(rel_parts, scale):
    hx, hy, hz, _, _, _ = mesh_bounds.bounds3d(
        os.path.join(_MODELS_ROOT, *rel_parts), scale)
    dx, dy = 2 * hx, 2 * hy
    return {"major": max(dx, dy), "minor": min(dx, dy), "height": 2 * hz}


MESH_SIGNATURES = {
    fam: _signature(parts, scale) for fam, (parts, scale) in _MESHES.items()
}
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_signatures.py -v`
Expected: PASS. If `test_bench_is_elongated_and_low_lamp_is_thin_and_tall` fails, the lamp/bin scale is wrong — print `MESH_SIGNATURES` and compare against `park_map.pgm` footprints; adjust the scale in `_MESHES` until dims are physically sane (lamp footprint < 0.3 m, bench major 1–2 m). Record the pinned scales in the report.

- [ ] **Step 7: Commit**

```bash
git add map_tools/mesh_bounds.py landmark_loc/__init__.py landmark_loc/signatures.py landmark_loc/tests/test_signatures.py
git commit -m "feat(landmark-loc): mesh-derived 3D landmark signatures + bounds3d helper"
```

**NOTE for the controller (in-sim, main conversation only, not the subagent):** after this task, in a clean sim, drive the robot near a bench/table/lamp/bin and dump one cluster per type (Task 2's node prints cluster dims) to confirm the mesh signatures match live lidar extents and to pin the classifier margins used in Task 3. This is a live check, not a unit test.

---

### Task 2: Cloud → clusters (segmentation)

**Files:**
- Create: `landmark_loc/segment.py`
- Test: `landmark_loc/tests/test_segment.py`

**Interfaces:**
- Consumes: nothing from prior tasks (pure geometry on point arrays).
- Produces:
  - `segment.crop(points, z_min, z_max, max_range) -> np.ndarray` — points is an `(N,3)` array in the **lidar frame**; returns the subset with `z_min <= z <= z_max` (z relative to the lidar; the caller passes a band already offset for ground) and horizontal range `<= max_range`.
  - `segment.cluster(points, link_dist, min_pts, max_extent) -> list[Cluster]` — Euclidean single-link-style clustering in 3D; drops clusters with `< min_pts` points or horizontal extent `> max_extent`.
  - `segment.Cluster` — dataclass `{ points: np.ndarray, centroid_xy: (float,float), major: float, minor: float, height: float }` where major/minor are the oriented (PCA) horizontal full-extents and height is the z full-extent.

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_segment.py
import numpy as np
from landmark_loc import segment


def test_crop_filters_height_and_range():
    pts = np.array([
        [0.0, 0.0, -5.0],   # below band
        [1.0, 0.0, 0.5],    # keep
        [30.0, 0.0, 0.5],   # out of range
        [2.0, 0.0, 1.9],    # keep
    ])
    out = segment.crop(pts, z_min=0.1, z_max=2.0, max_range=15.0)
    assert out.shape[0] == 2


def test_cluster_separates_two_blobs():
    a = np.random.default_rng(0).normal([0, 0, 0.5], 0.05, size=(60, 3))
    b = np.random.default_rng(1).normal([5, 5, 0.5], 0.05, size=(60, 3))
    clusters = segment.cluster(np.vstack([a, b]),
                               link_dist=0.3, min_pts=10, max_extent=3.0)
    assert len(clusters) == 2
    cents = sorted(c.centroid_xy[0] for c in clusters)
    assert abs(cents[0] - 0.0) < 0.5 and abs(cents[1] - 5.0) < 0.5


def test_cluster_drops_sparse_and_oversized():
    sparse = np.random.default_rng(2).normal([0, 0, 0.5], 0.05, size=(3, 3))
    wall = np.random.default_rng(3).uniform([-5, -5, 0.5], [5, 5, 0.6], size=(200, 3))
    clusters = segment.cluster(np.vstack([sparse, wall]),
                               link_dist=0.3, min_pts=10, max_extent=3.0)
    assert clusters == []


def test_cluster_reports_major_minor_height():
    # a bar 2m long (x), 0.3m wide (y), 0.5m tall (z)
    rng = np.random.default_rng(4)
    bar = np.column_stack([
        rng.uniform(-1.0, 1.0, 200),
        rng.uniform(-0.15, 0.15, 200),
        rng.uniform(0.0, 0.5, 200),
    ])
    c = segment.cluster(bar, link_dist=0.5, min_pts=10, max_extent=4.0)[0]
    assert c.major > 1.7 and c.minor < 0.6 and 0.4 < c.height < 0.6
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_segment.py -v`
Expected: FAIL — `landmark_loc.segment` does not exist.

- [ ] **Step 3: Write `landmark_loc/segment.py`**

```python
"""Segment a lidar point array into candidate landmark clusters.

Pure geometry on (N,3) numpy arrays in the lidar frame. No ROS. Clustering is a
grid-accelerated Euclidean single-link grouping; adequate for the sparse,
well-separated park furniture at ~15 m range and cheap enough for 10 Hz.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class Cluster:
    points: np.ndarray
    centroid_xy: tuple
    major: float
    minor: float
    height: float


def crop(points, z_min, z_max, max_range):
    if len(points) == 0:
        return points
    z = points[:, 2]
    rng = np.hypot(points[:, 0], points[:, 1])
    keep = (z >= z_min) & (z <= z_max) & (rng <= max_range)
    return points[keep]


def _grid_clusters(xy, link_dist):
    """Union-find over points whose cells are within link_dist (grid-bucketed)."""
    if len(xy) == 0:
        return []
    cell = np.floor(xy / link_dist).astype(int)
    buckets = {}
    for i, (cx, cy) in enumerate(map(tuple, cell)):
        buckets.setdefault((cx, cy), []).append(i)
    parent = list(range(len(xy)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    d2 = link_dist * link_dist
    for (cx, cy), idxs in buckets.items():
        neigh = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                neigh.extend(buckets.get((cx + dx, cy + dy), []))
        for i in idxs:
            for j in neigh:
                if j <= i:
                    continue
                if np.sum((xy[i] - xy[j]) ** 2) <= d2:
                    union(i, j)
    groups = {}
    for i in range(len(xy)):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def _pca_extents(xy):
    c = xy.mean(axis=0)
    centered = xy - c
    cov = np.cov(centered.T) if len(xy) > 1 else np.zeros((2, 2))
    evals, evecs = np.linalg.eigh(cov)
    proj = centered @ evecs
    spans = proj.max(axis=0) - proj.min(axis=0)
    major, minor = max(spans), min(spans)
    return major, minor


def cluster(points, link_dist, min_pts, max_extent):
    if len(points) == 0:
        return []
    xy = points[:, :2]
    out = []
    for idxs in _grid_clusters(xy, link_dist):
        if len(idxs) < min_pts:
            continue
        pts = points[idxs]
        cxy = pts[:, :2]
        major, minor = _pca_extents(cxy)
        if major > max_extent:
            continue
        height = float(pts[:, 2].max() - pts[:, 2].min())
        out.append(Cluster(
            points=pts,
            centroid_xy=(float(cxy[:, 0].mean()), float(cxy[:, 1].mean())),
            major=float(major), minor=float(minor), height=height))
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_segment.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/segment.py landmark_loc/tests/test_segment.py
git commit -m "feat(landmark-loc): lidar cloud cropping + Euclidean clustering"
```

---

### Task 3: Classifier (cluster → typed identity)

**Files:**
- Create: `landmark_loc/classify.py`
- Test: `landmark_loc/tests/test_classify.py`

**Interfaces:**
- Consumes: `segment.Cluster` (major, minor, height), `signatures.MESH_SIGNATURES`.
- Produces:
  - `classify.classify_cluster(cluster, margins=DEFAULT_MARGINS) -> str` — returns one of `"bench" | "garden_table" | "lamp" | "trash_bin_1" | "tree" | "unknown"`.
  - `classify.DEFAULT_MARGINS: dict` — per-dimension tolerance bands (metres / ratio) tuned to absorb partial lidar views; the in-sim NOTE from Task 1 pins these.
  - `classify.Observation` — dataclass `{ identity: str, x: float, y: float }` (the pluggable descriptor; `identity` is the type string here).
  - `classify.to_observations(clusters, margins=DEFAULT_MARGINS) -> list[Observation]` — classify each, drop `tree`/`unknown`, emit `{identity, x, y}` using the cluster centroid.

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_classify.py
from landmark_loc import classify
from landmark_loc.segment import Cluster


def _c(major, minor, height):
    return Cluster(points=None, centroid_xy=(1.0, 2.0),
                   major=major, minor=minor, height=height)


def test_classifies_each_type_from_ideal_dims():
    from landmark_loc.signatures import MESH_SIGNATURES as S
    for fam in ("bench", "garden_table", "lamp", "trash_bin_1"):
        sig = S[fam]
        got = classify.classify_cluster(_c(sig["major"], sig["minor"], sig["height"]))
        assert got == fam, f"{fam} misclassified as {got}"


def test_round_tall_trunk_is_tree_not_lamp():
    # tree trunk: small round footprint but tall with trunk radius > lamp pole
    got = classify.classify_cluster(_c(major=0.45, minor=0.42, height=4.0))
    assert got == "tree"


def test_ambiguous_between_bands_is_unknown():
    # deliberately between bench and table aspect/size
    got = classify.classify_cluster(_c(major=1.9, minor=1.3, height=0.9))
    assert got == "unknown"


def test_to_observations_drops_tree_and_unknown():
    clusters = [
        _c(*_dims("bench")),
        _c(major=0.45, minor=0.42, height=4.0),   # tree
        _c(major=1.9, minor=1.3, height=0.9),     # unknown
    ]
    obs = classify.to_observations(clusters)
    assert len(obs) == 1 and obs[0].identity == "bench"
    assert obs[0].x == 1.0 and obs[0].y == 2.0


def _dims(fam):
    from landmark_loc.signatures import MESH_SIGNATURES as S
    s = S[fam]
    return s["major"], s["minor"], s["height"]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_classify.py -v`
Expected: FAIL — `landmark_loc.classify` does not exist.

- [ ] **Step 3: Write `landmark_loc/classify.py`**

```python
"""Rule-based classification of a lidar cluster into a park landmark type.

Bands are centered on the mesh-derived signatures (signatures.MESH_SIGNATURES)
and widened by DEFAULT_MARGINS to absorb partial views. Deliberately
CONSERVATIVE: a cluster matching zero or more-than-one type is 'unknown' and is
dropped downstream. Trees (round, tall, trunk-radius footprint) are recognized
only to EXCLUDE them from identity; they remain obstacles for the costmap.
"""
from dataclasses import dataclass
from landmark_loc.signatures import MESH_SIGNATURES, SIGNATURE_FAMILIES

# Tolerances. major/minor in metres, aspect is a ratio, height in metres.
# Pinned against live lidar (Task 1 in-sim NOTE). Starting values below.
DEFAULT_MARGINS = {
    "major": 0.8,      # +/- m on the major horizontal extent
    "minor": 0.6,      # +/- m on the minor horizontal extent
    "height": 1.0,     # +/- m on height
    "aspect_split": 1.8,   # major/minor above this = elongated (bench-like)
}

# Tree exclusion: a roughly round footprint (aspect < aspect_split) with a
# trunk-scale minor extent AND tall. Distinguishes trunk from lamp pole by the
# larger trunk radius and from bins by height.
_TREE_MIN_HEIGHT = 2.0
_TREE_MIN_MINOR = 0.30   # trunk radius > lamp pole
_TREE_MAX_MINOR = 1.0


@dataclass
class Observation:
    identity: str
    x: float
    y: float


def _matches(cluster, fam, m):
    sig = MESH_SIGNATURES[fam]
    if abs(cluster.major - sig["major"]) > m["major"]:
        return False
    if abs(cluster.minor - sig["minor"]) > m["minor"]:
        return False
    if abs(cluster.height - sig["height"]) > m["height"]:
        return False
    aspect = cluster.major / max(cluster.minor, 1e-3)
    sig_aspect = sig["major"] / max(sig["minor"], 1e-3)
    elongated = aspect >= m["aspect_split"]
    sig_elongated = sig_aspect >= m["aspect_split"]
    return elongated == sig_elongated


def classify_cluster(cluster, margins=DEFAULT_MARGINS):
    aspect = cluster.major / max(cluster.minor, 1e-3)
    # tree gate first: round + tall + trunk-radius
    if (aspect < margins["aspect_split"]
            and cluster.height >= _TREE_MIN_HEIGHT
            and _TREE_MIN_MINOR <= cluster.minor <= _TREE_MAX_MINOR):
        return "tree"
    hits = [fam for fam in SIGNATURE_FAMILIES if _matches(cluster, fam, margins)]
    if len(hits) == 1:
        return hits[0]
    return "unknown"


def to_observations(clusters, margins=DEFAULT_MARGINS):
    out = []
    for c in clusters:
        ident = classify_cluster(c, margins)
        if ident in ("tree", "unknown"):
            continue
        out.append(Observation(identity=ident,
                               x=c.centroid_xy[0], y=c.centroid_xy[1]))
    return out
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_classify.py -v`
Expected: PASS. If a family's ideal dims don't self-classify (bands overlap so two hits → unknown), tighten `aspect_split` or narrow the overlapping margin; the four ideal signatures MUST each classify to themselves.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/classify.py landmark_loc/tests/test_classify.py
git commit -m "feat(landmark-loc): conservative rule-based cluster classifier"
```

---

### Task 4: Catalog loader + FoV gating

**Files:**
- Create: `landmark_loc/catalog.py`
- Test: `landmark_loc/tests/test_catalog.py`

**Interfaces:**
- Consumes: `maps/park_places.yaml` (`name: {x, y}`), the family classifier `map_tools.sdf_parse.classify` (name → family), `classify.Observation`.
- Produces:
  - `catalog.MapLandmark` — dataclass `{ name: str, identity: str, x: float, y: float }` (map frame).
  - `catalog.load(places_path) -> list[MapLandmark]` — load `park_places.yaml`, map each name to its identity via `sdf_parse.classify` (`arbolpartes4`/`tree_8` → skipped; the four families kept), skip anything classifying to `skip` or a tree.
  - `catalog.gate(landmarks, prior_xyz, max_range, fov_halfwidth) -> list[MapLandmark]` — transform each landmark into the robot frame implied by `prior_xyz=(x,y,yaw)` and keep those within `max_range` and within `+/- fov_halfwidth` radians of forward (the 360° lidar means fov_halfwidth can be π, but the arg lets callers restrict).

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_catalog.py
import math
from landmark_loc import catalog


def test_load_maps_names_to_identities(tmp_path):
    p = tmp_path / "places.yaml"
    p.write_text(
        "bench: {x: 1.0, y: 2.0}\n"
        "bench_clone_1: {x: 3.0, y: 4.0}\n"
        "lamp: {x: -5.0, y: 6.0}\n"
        "garden_table_clone_2: {x: 7.0, y: 8.0}\n"
        "trash_bin_1: {x: 9.0, y: 0.0}\n")
    lms = catalog.load(str(p))
    ids = sorted(l.identity for l in lms)
    assert ids == ["bench", "bench", "garden_table", "lamp", "trash_bin_1"]


def test_gate_keeps_only_in_range_and_fov():
    lms = [
        catalog.MapLandmark("near", "bench", 2.0, 0.0),    # 2m ahead
        catalog.MapLandmark("far", "lamp", 40.0, 0.0),     # out of range
        catalog.MapLandmark("behind", "lamp", -2.0, 0.0),  # behind (out of fov)
    ]
    kept = catalog.gate(lms, prior_xyz=(0.0, 0.0, 0.0),
                        max_range=15.0, fov_halfwidth=math.pi / 2)
    names = {l.name for l in kept}
    assert names == {"near"}


def test_gate_respects_prior_yaw():
    lm = [catalog.MapLandmark("left", "bench", 0.0, 2.0)]  # 2m to +y (world)
    # robot facing +y (yaw=pi/2): the landmark is straight ahead
    kept = catalog.gate(lm, prior_xyz=(0.0, 0.0, math.pi / 2),
                       max_range=15.0, fov_halfwidth=math.pi / 4)
    assert len(kept) == 1
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_catalog.py -v`
Expected: FAIL — `landmark_loc.catalog` does not exist.

- [ ] **Step 3: Write `landmark_loc/catalog.py`**

```python
"""Load the known landmark catalog and gate it to the robot's plausible view.

The catalog is maps/park_places.yaml (name -> map-frame x,y). Each name is
mapped to a landmark identity via map_tools.sdf_parse.classify. gate() prunes
the catalog to landmarks a robot at the prior pose could currently see, so
association is a local problem, not a global search.
"""
import math
from dataclasses import dataclass
import yaml
from map_tools.sdf_parse import classify as _family_of

_IDENTITY_FAMILIES = {"bench", "garden_table", "lamp", "trash_bin_1"}


@dataclass
class MapLandmark:
    name: str
    identity: str
    x: float
    y: float


def load(places_path):
    with open(places_path) as fh:
        data = yaml.safe_load(fh)
    out = []
    for name, xy in data.items():
        fam = _family_of(name)
        if fam not in _IDENTITY_FAMILIES:
            continue
        out.append(MapLandmark(name, fam, float(xy["x"]), float(xy["y"])))
    return out


def gate(landmarks, prior_xyz, max_range, fov_halfwidth):
    px, py, pyaw = prior_xyz
    c, s = math.cos(-pyaw), math.sin(-pyaw)
    kept = []
    for lm in landmarks:
        dx, dy = lm.x - px, lm.y - py
        # rotate world delta into robot frame (robot forward = +x)
        rx = c * dx - s * dy
        ry = s * dx + c * dy
        rng = math.hypot(rx, ry)
        if rng > max_range or rng < 1e-6:
            continue
        bearing = math.atan2(ry, rx)
        if abs(bearing) <= fov_halfwidth:
            kept.append(lm)
    return kept
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_catalog.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/catalog.py landmark_loc/tests/test_catalog.py
git commit -m "feat(landmark-loc): catalog loader + field-of-view gating"
```

---

### Task 5: Association + rigid-transform pose solve

**Files:**
- Create: `landmark_loc/solve.py`
- Test: `landmark_loc/tests/test_solve.py`

**Interfaces:**
- Consumes: `classify.Observation` (robot-frame `{identity, x, y}`), `catalog.MapLandmark` (map-frame), a prior `(x,y,yaw)`.
- Produces:
  - `solve.associate(observations, gated_landmarks, prior_xyz, dist_gate) -> list[tuple[Observation, MapLandmark]]` — for each observation, transform it to the map frame using the prior, match to the nearest gated landmark of the SAME identity, keep if within `dist_gate`.
  - `solve.rigid_transform_2d(src_xy, dst_xy) -> (x, y, yaw, rms)` — Umeyama/Kabsch: best rotation+translation mapping `src` (robot-frame observed points) onto `dst` (map-frame matched points); returns the robot pose in map frame and the RMS residual.
  - `solve.solve_pose(observations, gated_landmarks, prior_xyz, dist_gate, residual_gate) -> Optional[(x,y,yaw,rms,n)]` — associate then solve; returns `None` if `< 2` correspondences or `rms > residual_gate`; else the pose plus residual and correspondence count.

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_solve.py
import math
import numpy as np
from landmark_loc import solve
from landmark_loc.classify import Observation
from landmark_loc.catalog import MapLandmark


def _obs_from_truth(true_xyz, landmarks):
    """Project map landmarks into the robot frame at the TRUE pose (test only)."""
    x, y, yaw = true_xyz
    c, s = math.cos(-yaw), math.sin(-yaw)
    obs = []
    for lm in landmarks:
        dx, dy = lm.x - x, lm.y - y
        obs.append(Observation(lm.identity, c * dx - s * dy, s * dx + c * dy))
    return obs


def test_rigid_transform_recovers_known_pose():
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    true = (2.0, -1.0, 0.5)
    obs = _obs_from_truth(true, lms)
    src = np.array([[o.x, o.y] for o in obs])
    dst = np.array([[l.x, l.y] for l in lms])
    x, y, yaw, rms = solve.rigid_transform_2d(src, dst)
    assert abs(x - 2.0) < 1e-6 and abs(y + 1.0) < 1e-6
    assert abs((yaw - 0.5 + math.pi) % (2 * math.pi) - math.pi) < 1e-6
    assert rms < 1e-6


def test_solve_pose_rejects_when_too_few_matches():
    lms = [MapLandmark("a", "bench", 5.0, 1.0)]
    obs = _obs_from_truth((0, 0, 0), lms)
    out = solve.solve_pose(obs, lms, prior_xyz=(0, 0, 0),
                          dist_gate=1.0, residual_gate=0.5)
    assert out is None  # only 1 correspondence


def test_solve_pose_rejects_high_residual():
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    obs = _obs_from_truth((2.0, -1.0, 0.5), lms)
    obs[0].x += 3.0  # corrupt one observation badly
    out = solve.solve_pose(obs, lms, prior_xyz=(2.0, -1.0, 0.5),
                          dist_gate=5.0, residual_gate=0.3)
    assert out is None  # residual too high


def test_solve_pose_accepts_clean_fit():
    lms = [MapLandmark("a", "bench", 5.0, 1.0),
           MapLandmark("b", "lamp", 6.0, -2.0),
           MapLandmark("c", "garden_table", 3.0, 4.0)]
    obs = _obs_from_truth((2.0, -1.0, 0.5), lms)
    out = solve.solve_pose(obs, lms, prior_xyz=(2.0, -1.0, 0.5),
                          dist_gate=1.0, residual_gate=0.3)
    assert out is not None
    x, y, yaw, rms, n = out
    assert abs(x - 2.0) < 0.05 and abs(y + 1.0) < 0.05 and n == 3
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_solve.py -v`
Expected: FAIL — `landmark_loc.solve` does not exist.

- [ ] **Step 3: Write `landmark_loc/solve.py`**

```python
"""Associate observed landmarks to catalog landmarks and solve the robot pose.

Association is identity-consistent nearest-neighbor under the prior. The pose is
the 2D rigid transform (Umeyama/Kabsch) mapping observed (robot-frame) points
onto their matched (map-frame) points; that transform IS the robot's map pose.
A fit with < 2 correspondences or RMS residual above the gate is rejected
(returns None) so a bad scan cannot corrupt the downstream EKF.
"""
import math
import numpy as np


def _to_map(o, prior_xyz):
    x, y, yaw = prior_xyz
    c, s = math.cos(yaw), math.sin(yaw)
    return (x + c * o.x - s * o.y, y + s * o.x + c * o.y)


def associate(observations, gated_landmarks, prior_xyz, dist_gate):
    pairs = []
    for o in observations:
        mx, my = _to_map(o, prior_xyz)
        best, best_d = None, dist_gate
        for lm in gated_landmarks:
            if lm.identity != o.identity:
                continue
            d = math.hypot(lm.x - mx, lm.y - my)
            if d <= best_d:
                best, best_d = lm, d
        if best is not None:
            pairs.append((o, best))
    return pairs


def rigid_transform_2d(src_xy, dst_xy):
    src = np.asarray(src_xy, float)
    dst = np.asarray(dst_xy, float)
    cs, cd = src.mean(axis=0), dst.mean(axis=0)
    s0, d0 = src - cs, dst - cd
    H = s0.T @ d0
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:      # reflection guard
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    yaw = math.atan2(R[1, 0], R[0, 0])
    t = cd - R @ cs
    resid = (R @ s0.T).T + cd - dst
    rms = float(np.sqrt(np.mean(np.sum(resid ** 2, axis=1)))) if len(dst) else 0.0
    return float(t[0]), float(t[1]), yaw, rms


def solve_pose(observations, gated_landmarks, prior_xyz, dist_gate, residual_gate):
    pairs = associate(observations, gated_landmarks, prior_xyz, dist_gate)
    if len(pairs) < 2:
        return None
    src = np.array([[o.x, o.y] for o, _ in pairs])
    dst = np.array([[lm.x, lm.y] for _, lm in pairs])
    x, y, yaw, rms = rigid_transform_2d(src, dst)
    if rms > residual_gate:
        return None
    return (x, y, yaw, rms, len(pairs))
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_solve.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/solve.py landmark_loc/tests/test_solve.py
git commit -m "feat(landmark-loc): identity association + Umeyama pose solve with residual gate"
```

---

### Task 6: The ROS node (`landmark_localizer`)

**Files:**
- Create: `landmark_loc/localizer_node.py` (executable rospy node)
- Create: `landmark_loc/tests/test_node_helpers.py`

**Interfaces:**
- Consumes: all prior modules; `sensor_msgs/PointCloud2`, `nav_msgs/Odometry`, `tf2_ros`.
- Produces:
  - a runnable node publishing `/odometry/abs_fix` (`nav_msgs/Odometry`, `frame_id: map`, `child_frame_id: base_link`), x,y + yaw-from-solve-for-logging only, with a **covariance scaled by match count**.
  - `localizer_node.covariance_for(n_matches, base_var) -> list[36]` — pure helper: position variance shrinks with more matches (`base_var / n`), orientation/other entries large (unfused). Unit-tested.
  - `localizer_node.cloud_to_array(cloud_msg) -> np.ndarray` — `(N,3)` in the lidar frame (thin wrapper over `sensor_msgs.point_cloud2.read_points`); not unit-tested (ROS msg), exercised in-sim.

**Node behavior (documented, exercised in-sim not unit tests):**
- Subscribe `/os0_cloud_node/points`. On each cloud (throttle to ~5 Hz to bound CPU):
  1. `cloud_to_array` → crop (z-band offset for ground: ground is ~0.83 m below `os0_lidar`, so band `z ∈ [-0.73, 1.2]` ≈ 0.1–2.0 m above ground — pin in-sim) → cluster.
  2. `classify.to_observations`.
  3. prior `(x,y,yaw)` from the latest `/odometry/filtered_map` (EKF output) — the compass+odom fused pose; if none yet, skip this cloud.
  4. `catalog.gate` → `solve.solve_pose`.
  5. if a pose is returned, publish `/odometry/abs_fix` with `covariance_for(n)`; else publish nothing.
- Params (private `~`): `places_path`, `z_min`, `z_max`, `max_range`, `link_dist`, `min_pts`, `max_extent`, `dist_gate`, `residual_gate`, `fov_halfwidth`, `rate`, `base_var`. Defaults match the pinned in-sim values.

- [ ] **Step 1: Write the failing test (pure helper only)**

```python
# landmark_loc/tests/test_node_helpers.py
from landmark_loc import localizer_node as ln


def test_covariance_shrinks_with_matches():
    c2 = ln.covariance_for(2, base_var=1.0)
    c6 = ln.covariance_for(6, base_var=1.0)
    # index 0 = x var, index 7 = y var (row-major 6x6)
    assert c6[0] < c2[0] and c6[7] < c2[7]
    assert len(c2) == 36


def test_covariance_marks_orientation_unfused_large():
    c = ln.covariance_for(4, base_var=1.0)
    # yaw variance (index 35) must be large (orientation not fused from here)
    assert c[35] >= 1e3
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_node_helpers.py -v`
Expected: FAIL — module import fails / `covariance_for` missing.

- [ ] **Step 3: Write `landmark_loc/localizer_node.py`**

Guard ROS imports so the pure helper is importable without a ROS env:

```python
#!/usr/bin/env python3
"""ROS node: publish an absolute map-frame pose fix from lidar landmark matching.

Pipeline per cloud: cloud->array -> crop -> cluster -> classify -> gate catalog
by the EKF prior -> associate -> rigid-transform solve -> publish /odometry/abs_fix
(only on a fit that passes the residual+count gate; otherwise silent so the EKF
coasts on odom). Position-only: yaw from the solve is logged, not fused (the
map-EKF takes yaw from /compass/data).
"""
import math

import numpy as np

from landmark_loc import segment, classify, catalog, solve


def covariance_for(n_matches, base_var):
    cov = [0.0] * 36
    pos_var = base_var / max(n_matches, 1)
    cov[0] = pos_var      # x
    cov[7] = pos_var      # y
    cov[14] = 1e6         # z (unused, 2d)
    cov[21] = 1e6         # roll
    cov[28] = 1e6         # pitch
    cov[35] = 1e6         # yaw (NOT fused from here)
    return cov


def cloud_to_array(cloud_msg):
    from sensor_msgs import point_cloud2
    pts = point_cloud2.read_points(
        cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.array(list(pts), dtype=float)


def main():
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2

    rospy.init_node("landmark_localizer")
    places = rospy.get_param("~places_path",
                             "/home/thinh/Documents/Husky_viz/maps/park_places.yaml")
    p = dict(
        z_min=rospy.get_param("~z_min", -0.73),
        z_max=rospy.get_param("~z_max", 1.2),
        max_range=rospy.get_param("~max_range", 15.0),
        link_dist=rospy.get_param("~link_dist", 0.3),
        min_pts=rospy.get_param("~min_pts", 10),
        max_extent=rospy.get_param("~max_extent", 3.5),
        dist_gate=rospy.get_param("~dist_gate", 2.0),
        residual_gate=rospy.get_param("~residual_gate", 0.4),
        fov_halfwidth=rospy.get_param("~fov_halfwidth", math.pi),
        base_var=rospy.get_param("~base_var", 0.5),
        rate=rospy.get_param("~rate", 5.0),
    )
    landmarks = catalog.load(places)
    rospy.loginfo("landmark_localizer: %d catalog landmarks", len(landmarks))

    state = {"prior": None, "last_pub": rospy.Time(0)}
    pub = rospy.Publisher("/odometry/abs_fix", Odometry, queue_size=5)

    def on_prior(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        state["prior"] = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y, yaw)

    def on_cloud(msg):
        now = rospy.Time.now()
        if (now - state["last_pub"]).to_sec() < 1.0 / p["rate"]:
            return
        if state["prior"] is None:
            return
        pts = cloud_to_array(msg)
        if len(pts) == 0:
            return
        cropped = segment.crop(pts, p["z_min"], p["z_max"], p["max_range"])
        clusters = segment.cluster(cropped, p["link_dist"], p["min_pts"], p["max_extent"])
        obs = classify.to_observations(clusters)
        gated = catalog.gate(landmarks, state["prior"], p["max_range"], p["fov_halfwidth"])
        result = solve.solve_pose(obs, gated, state["prior"],
                                  p["dist_gate"], p["residual_gate"])
        if result is None:
            return
        x, y, yaw, rms, n = result
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.w = 1.0
        od.pose.covariance = covariance_for(n, p["base_var"])
        pub.publish(od)
        state["last_pub"] = now

    rospy.Subscriber("/odometry/filtered_map", Odometry, on_prior, queue_size=5)
    rospy.Subscriber("/os0_cloud_node/points", PointCloud2, on_cloud, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
```

Make it executable: `chmod +x landmark_loc/localizer_node.py`.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_node_helpers.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/localizer_node.py landmark_loc/tests/test_node_helpers.py
git commit -m "feat(landmark-loc): landmark_localizer ROS node publishing /odometry/abs_fix"
```

---

### Task 7: EKF topic rename + navsat remap (Option 1 wiring; GPS-mode-preserving)

**Files:**
- Modify: `natural_environments_ros_opt/husky/husky_control/config/localization_map.yaml:93`
- Modify: `natural_environments_ros_opt/husky/husky_control/launch/control.launch` (navsat node output remap)
- Test: `landmark_loc/tests/test_wiring.py`

**Interfaces:**
- Consumes: nothing (config/launch edits).
- Produces: the map-EKF reads its absolute-position input from `odometry/abs_fix`; navsat writes into that topic in GPS mode. GPS mode behavior is unchanged (navsat still supplies the fix, just under a neutral name).

**Constraint:** `localization_map.yaml` is gitignored but already tracked — `git commit` picks up the modification directly (already-tracked overrides .gitignore).

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_wiring.py
import re

CTRL = ("/home/thinh/Documents/Husky_viz/natural_environments_ros_opt/"
        "husky/husky_control/launch/control.launch")
EKF = ("/home/thinh/Documents/Husky_viz/natural_environments_ros_opt/"
       "husky/husky_control/config/localization_map.yaml")


def test_map_ekf_odom1_is_neutral_abs_fix():
    txt = open(EKF).read()
    assert re.search(r"^odom1:\s*odometry/abs_fix\s*$", txt, re.M)
    assert "odom1: odometry/gps" not in txt


def test_navsat_remaps_output_to_abs_fix():
    txt = open(CTRL).read()
    # navsat node must remap its odometry/gps output to the neutral topic so
    # GPS mode still feeds the EKF after the rename.
    assert re.search(r'from="odometry/gps"\s+to="odometry/abs_fix"', txt)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_wiring.py -v`
Expected: FAIL — the current `odom1: odometry/gps` and no navsat remap.

- [ ] **Step 3: Edit `localization_map.yaml` line 93**

Change:
```yaml
odom1: odometry/gps
```
to:
```yaml
odom1: odometry/abs_fix
```

- [ ] **Step 4: Edit `control.launch` navsat node**

Inside the `<node ... name="navsat_transform">` block, add an output remap so GPS mode keeps feeding the EKF under the neutral name (place beside the existing remaps):

```xml
      <remap from="odometry/gps" to="odometry/abs_fix"/>
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_wiring.py -v`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add natural_environments_ros_opt/husky/husky_control/launch/control.launch landmark_loc/tests/test_wiring.py
git commit -m "refactor(ekf): rename map-EKF odom1 to neutral odometry/abs_fix (GPS mode preserved)"
```

(The `localization_map.yaml` change is picked up by this commit automatically — it is already tracked.)

**NOTE for the controller (in-sim, main only):** after this task, run a clean GPS-mode sim (`load-park-world.sh` + `move_base_gps_map.launch`) and confirm a normal goal still drives and stops — this proves the rename didn't break GPS mode BEFORE landmark mode is even added.

---

### Task 8: Landmark-mode launch file + demo runbook

**Files:**
- Create: `launch/move_base_landmark.launch`
- Modify: `RUN-MAP-NAV.md` (Step 2 → two options)
- Test: `landmark_loc/tests/test_launch.py`

**Interfaces:**
- Consumes: `landmark_loc/localizer_node.py`, the neutral `odometry/abs_fix` topic (Task 7).
- Produces: a launch that starts map_server + move_base (identical to `move_base_gps_map.launch`) plus the `landmark_localizer` node; it does NOT start navsat (landmark mode has no GPS in the loop — the localizer fills `odometry/abs_fix` instead).

- [ ] **Step 1: Write the failing test**

```python
# landmark_loc/tests/test_launch.py
LAUNCH = "/home/thinh/Documents/Husky_viz/launch/move_base_landmark.launch"
RUNBOOK = "/home/thinh/Documents/Husky_viz/RUN-MAP-NAV.md"


def test_launch_starts_localizer_and_move_base():
    txt = open(LAUNCH).read()
    assert "localizer_node.py" in txt
    assert 'type="move_base"' in txt
    assert 'type="map_server"' in txt
    # landmark mode must NOT start navsat_transform
    assert "navsat_transform" not in txt


def test_runbook_offers_both_modes():
    txt = open(RUNBOOK).read()
    assert "move_base_gps_map.launch" in txt
    assert "move_base_landmark.launch" in txt
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_launch.py -v`
Expected: FAIL — launch file and runbook option absent.

- [ ] **Step 3: Create `launch/move_base_landmark.launch`**

Mirror `move_base_gps_map.launch` exactly (map_server + move_base with the same params/costmaps), then add the localizer. It reuses the SAME costmaps/planner configs — only the localization source differs.

```xml
<?xml version="1.0"?>
<!--
  GPS-FREE landmark-localization move_base, sibling of move_base_gps_map.launch.
  Identical map_server + move_base + costmaps + planner. The ONLY difference:
  instead of GPS (navsat_transform -> odometry/abs_fix), the landmark_localizer
  node fills odometry/abs_fix from lidar-vs-map matching. navsat is NOT started
  here; the map-EKF (already running from control.launch, odom1=odometry/abs_fix)
  fuses the landmark fix in place of GPS. Compass still supplies yaw.

  The robot, dual-EKF and TF chain are ASSUMED ALREADY RUNNING (load-park-world.sh).
-->
<launch>

  <node name="map_server" pkg="map_server" type="map_server"
        args="/home/thinh/Documents/Husky_viz/maps/park_map.yaml" output="screen"/>

  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="base_global_planner" value="navfn/NavfnROS"/>
    <param name="base_local_planner"  value="dwa_local_planner/DWAPlannerROS"/>
    <rosparam file="/home/thinh/Documents/Husky_viz/config/planner_gps.yaml" command="load"/>
    <rosparam file="/home/thinh/Documents/Husky_viz/config/costmap_common_gps.yaml" command="load" ns="global_costmap"/>
    <rosparam file="/home/thinh/Documents/Husky_viz/config/costmap_common_gps.yaml" command="load" ns="local_costmap"/>
    <rosparam file="/home/thinh/Documents/Husky_viz/config/costmap_local_gps.yaml"      command="load" ns="local_costmap"/>
    <rosparam file="/home/thinh/Documents/Husky_viz/config/costmap_global_gps_map.yaml" command="load" ns="global_costmap"/>
    <remap from="cmd_vel" to="/cmd_vel"/>
    <remap from="odom"    to="/odometry/filtered_map"/>
  </node>

  <node pkg="landmark_loc" type="localizer_node.py" name="landmark_localizer"
        output="screen"
        args="">
    <param name="places_path" value="/home/thinh/Documents/Husky_viz/maps/park_places.yaml"/>
  </node>

</launch>
```

If `landmark_loc` is not a catkin package on the ROS path, run the node by absolute path instead — replace the `<node pkg=... type=...>` with:
```xml
  <node name="landmark_localizer" pkg="rospy_tutorials" type="talker"
        launch-prefix="python3"
        args="/home/thinh/Documents/Husky_viz/landmark_loc/localizer_node.py" output="screen"/>
```
Prefer the clean `pkg` form if a lightweight `package.xml`/`setup.py` for `landmark_loc` can be added on the existing overlay path; otherwise the absolute-path form is acceptable and documented in the report. (Decide based on how other repo nodes like `operator/operate.py` are launched — match that convention.)

- [ ] **Step 4: Edit `RUN-MAP-NAV.md` Step 2**

Replace the single Step 2 launch with:

```markdown
## Step 2 — Navigation + map  (choose ONE localization mode)

### Option A — GPS mode (spoofable; used by the attacker demo in Step 6)
​```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
roslaunch launch/move_base_gps_map.launch
​```

### Option B — Landmark mode (GPS-free; recognizes park landmarks from lidar)
​```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
roslaunch launch/move_base_landmark.launch
​```

In landmark mode the GPS-spoof of Step 6 has nothing to attack (no navsat in the
loop) — the robot keeps localizing off the furniture it can see.
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_launch.py -v`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add launch/move_base_landmark.launch RUN-MAP-NAV.md landmark_loc/tests/test_launch.py
git commit -m "feat(landmark-loc): move_base_landmark.launch + dual-mode demo runbook"
```

**NOTE for the controller (in-sim, main only):** full end-to-end verification, from a clean sim, judged by the robot's own sensors + RViz (never ground truth):
1. `load-park-world.sh`, then `move_base_landmark.launch`.
2. Confirm `/odometry/abs_fix` publishes and the estimate sits where lidar clusters align with the static map in RViz.
3. `goal garden_table` → robot drives and STOPS at the table.
4. Drive a long leg → estimate stays anchored (no odom-style drift).
5. Run the GPS-spoof (Step 6) in landmark mode → navigation is unaffected (attack inert).

---

## Self-Review

**Spec coverage:**
- Catalog (identifiable landmarks, trees excluded) → Task 4. ✓
- Perception: crop keeping z, cluster, classify by footprint+height, 3D-identity/2D-geometry → Tasks 2, 3 (flatten to `{identity,x,y}` at classify output). ✓
- Pluggable identity descriptor → `classify.Observation`/`catalog.MapLandmark` carry an opaque `identity` string; matcher uses equality only (Task 5). ✓
- Compass+odom prior seeds & gates; rigid-transform solve; residual gate; coast-and-recover → Task 5 (`solve_pose` returns None to coast), Task 6 (publish-nothing path). ✓
- Position-only fusion, compass keeps yaw → Task 6 covariance marks yaw unfused; Task 7 leaves `imu1` untouched. ✓
- New launch, sibling of GPS, navsat dropped, EKF `odom1` repointed via neutral topic (Option 1) → Tasks 7, 8. ✓
- GPS mode preserved → Task 7 navsat output remap + in-sim check. ✓
- Demo runbook offers both modes → Task 8. ✓
- No-ground-truth in all verification NOTEs → every in-sim NOTE judges by sensors/RViz. ✓

**Placeholder scan:** thresholds are concrete starting values with a stated in-sim pinning step (Task 1 NOTE); no TBD/TODO; every code step has full code.

**Type consistency:** `Observation{identity,x,y}` and `MapLandmark{name,identity,x,y}` used consistently; `solve_pose` returns `(x,y,yaw,rms,n)` consistently; `covariance_for(n,base_var)` signature matches its test and node call; `bounds3d` 6-tuple matches `signatures._signature` unpacking.

## Execution note on in-sim tasks

Unit-testable logic (Tasks 1–5, and the `covariance_for` helper of Task 6, the wiring/launch string checks of Tasks 7–8) is implemented and pytest-verified by subagents. **All live-sim verification (the NOTEs after Tasks 1, 6, 7, 8) is run by the controller in the main conversation, from a clean sim kill, never by a subagent and never against Gazebo ground truth** — per the project's standing "agents implement, main runs sim" rule and the no-ground-truth constraint.
