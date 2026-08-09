# Landmark-Based GPS-Spoofing Fallback — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** When GPS is spoofed (Option B `attack_navsat_drift.py`), let the operator switch the robot's absolute-position source from GPS to lidar landmark localization against the known park map, so the robot still reaches the real goal.

**Architecture:** Dead reckoning (wheel odom + compass, already fused by the map EKF) is corrected by an absolute anchor. Today that anchor is `/odometry/gps`; this plan adds a landmark localizer that produces the same message on `/odometry/landmark`, and a gated relay that publishes whichever source is trusted onto a single topic `/odometry/absolute` that the map EKF's `odom1` now reads. The operator flips trust with a REPL command. Costmaps, planners, and move_base are untouched.

**Tech Stack:** ROS Noetic (Python `rospy`), `sensor_msgs/PointCloud2` + `sensor_msgs.point_cloud2`, `nav_msgs/Odometry`, `numpy` (1.17.4). No sklearn (not installed) — clustering is numpy-only. `std_srvs/SetBool` for the trust service.

## Global Constraints

- **No Gazebo ground truth, ever** — no `/gazebo/*`, no `gazebo_msgs`. Pose comes only from the robot's own lidar matched to the static `park.world` survey, plus wheel odom + `/compass/data`. Verification is judged by honest sensors + operator's Gazebo view, never against ground truth.
- **Map frame throughout** — the demo runs the dual-EKF map-frame stack: `/ekf_localization_map` publishes `map→odom`, pose topic `/odometry/filtered_map`, costmaps + goals in `map`. All landmark coords and the landmark pose are in the `map` frame.
- **Do not touch** costmaps, move_base, planners, the odom-frame EKF, or TF topology.
- **Landmark map is static survey data:** 91 point objects from `natural_environments_ros_opt/natural_enviroment/worlds/park.world` (38 tree, 16 bench, 15 lamp, 11 table, 11 trash_bin).
- **Lidar facts:** `/os0_cloud_node/points` (~10 Hz), frame `os0_lidar`, ground ≈ 0.83 m below the lidar. Trunk slab `z∈[-0.70,-0.40]` in `os0_lidar` yields ~410 stable pts/frame.
- **Python conventions:** match existing scripts (`operator/operate.py`, `operator/gcs_commands.py`) — `rospy`, module-level topic constants, small classes.
- **Commit after every task.** Branch: `feat/realistic-operator` (current).

---

## File Structure

- `landmark_localization/extract_landmarks.py` — offline: park.world → landmarks JSON (already written in scratchpad; move into repo).
- `landmark_localization/park_landmarks.json` — the 91-landmark map (generated).
- `landmark_localization/landmark_map.py` — loads the JSON into a numpy array + lookup helpers (pure, unit-testable).
- `landmark_localization/cluster_detect.py` — numpy-only: PointCloud2 → list of 2D cluster centroids at trunk height (pure, unit-testable).
- `landmark_localization/landmark_localizer_node.py` — ROS node: subscribes points, detects clusters, matches to map (seeded by dead-reckon prior), publishes `/odometry/landmark`.
- `landmark_localization/gated_relay_node.py` — ROS node: republishes GPS or landmark onto `/odometry/absolute`; `std_srvs/SetBool` service `/localization/trust_landmark`.
- `natural_environments_ros_opt/husky/husky_control/config/localization_map.yaml` — modify `odom1` topic.
- `operator/gcs_commands.py` — add `trust` verb parsing.
- `operator/operate.py` — add `trust landmark`/`trust gps` dispatch + divergence display.
- `tests/` — unit tests for the pure modules.

---

### Task 1: Move the extractor + landmark map into the repo

**Files:**
- Create: `landmark_localization/extract_landmarks.py` (copy from scratchpad)
- Create: `landmark_localization/park_landmarks.json` (regenerate)

**Interfaces:**
- Produces: `park_landmarks.json` shape `{frame:"map", count:91, landmarks:[{name,type,x,y},...]}`.

- [ ] **Step 1: Copy the proven extractor into the repo**

```bash
mkdir -p landmark_localization
cp "/tmp/claude-1000/-home-thinh-Documents-Husky-viz/30653e6a-f199-4935-b140-b15587a009a5/scratchpad/extract_landmarks.py" landmark_localization/extract_landmarks.py
```

- [ ] **Step 2: Regenerate the JSON into the repo**

Run:
```bash
cd /home/thinh/Documents/Husky_viz
python3 landmark_localization/extract_landmarks.py \
  natural_environments_ros_opt/natural_enviroment/worlds/park.world \
  landmark_localization/park_landmarks.json
```
Expected output ends with: `wrote 91 landmarks -> landmark_localization/park_landmarks.json`

- [ ] **Step 3: Verify the JSON**

Run:
```bash
python3 -c "import json,collections; d=json.load(open('landmark_localization/park_landmarks.json')); print(d['count'], d['frame']); print(collections.Counter(l['type'] for l in d['landmarks']))"
```
Expected: `91 map` and `Counter({'tree': 38, 'bench': 16, 'lamp': 15, 'table': 11, 'trash_bin': 11})`

- [ ] **Step 4: Commit**

```bash
git add landmark_localization/extract_landmarks.py landmark_localization/park_landmarks.json
git commit -m "feat(landmark): extract 91 park landmarks from park.world into repo"
```

---

### Task 2: Landmark map loader (`landmark_map.py`)

**Files:**
- Create: `landmark_localization/landmark_map.py`
- Test: `tests/test_landmark_map.py`

**Interfaces:**
- Consumes: `park_landmarks.json` from Task 1.
- Produces:
  - `class LandmarkMap` with:
    - `LandmarkMap.from_json(path) -> LandmarkMap`
    - `.xy` — numpy array shape `(N,2)` of landmark map coords
    - `.frame` — str, `"map"`
    - `.query_near(x, y, radius) -> np.ndarray` — subset of `.xy` within `radius` of `(x,y)`

- [ ] **Step 1: Write the failing test**

```python
# tests/test_landmark_map.py
import numpy as np
from landmark_localization.landmark_map import LandmarkMap

def test_loads_91_landmarks(tmp_path):
    m = LandmarkMap.from_json("landmark_localization/park_landmarks.json")
    assert m.frame == "map"
    assert m.xy.shape == (91, 2)

def test_query_near_filters_by_radius():
    m = LandmarkMap.from_json("landmark_localization/park_landmarks.json")
    # a large radius returns all; a tiny radius at an empty spot returns none
    assert m.query_near(0.0, 0.0, 1000.0).shape[0] == 91
    assert m.query_near(1e6, 1e6, 1.0).shape[0] == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest tests/test_landmark_map.py -v`
Expected: FAIL with `ModuleNotFoundError: landmark_localization.landmark_map`

- [ ] **Step 3: Write minimal implementation**

```python
# landmark_localization/landmark_map.py
import json
import numpy as np

class LandmarkMap(object):
    def __init__(self, xy, frame):
        self.xy = np.asarray(xy, dtype=float).reshape(-1, 2)
        self.frame = frame

    @classmethod
    def from_json(cls, path):
        with open(path) as f:
            d = json.load(f)
        xy = [[lm["x"], lm["y"]] for lm in d["landmarks"]]
        return cls(xy, d["frame"])

    def query_near(self, x, y, radius):
        if self.xy.shape[0] == 0:
            return self.xy
        d = np.hypot(self.xy[:, 0] - x, self.xy[:, 1] - y)
        return self.xy[d <= radius]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_landmark_map.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add landmark_localization/landmark_map.py tests/test_landmark_map.py
git commit -m "feat(landmark): LandmarkMap loader with radius query"
```

---

### Task 3: Cluster detection (`cluster_detect.py`)

**Files:**
- Create: `landmark_localization/cluster_detect.py`
- Test: `tests/test_cluster_detect.py`

**Interfaces:**
- Produces:
  - `detect_clusters(points_xyz, z_min=-0.70, z_max=-0.40, grid=0.5, min_pts=3) -> np.ndarray`
    - input: numpy `(M,3)` array of lidar points in the `os0_lidar` frame
    - output: `(K,2)` array of 2D cluster centroids (x,y in the lidar frame) for vertical objects in the trunk slab
    - Algorithm (numpy only, NO sklearn): keep points with `z_min <= z <= z_max`; bin (x,y) into `grid`-metre cells; for each cell with `>= min_pts` points, emit the mean (x,y). This is a deliberately simple grid-cluster — adequate because trunks/poles are compact.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_cluster_detect.py
import numpy as np
from landmark_localization.cluster_detect import detect_clusters

def test_finds_two_trunks_ignores_ground_and_canopy():
    rng = np.random.RandomState(0)
    # trunk A near (5,0), trunk B near (0,3), both in the slab z in [-0.7,-0.4]
    a = np.column_stack([5 + 0.05*rng.randn(20), 0 + 0.05*rng.randn(20), -0.55 + 0.05*rng.randn(20)])
    b = np.column_stack([0 + 0.05*rng.randn(20), 3 + 0.05*rng.randn(20), -0.55 + 0.05*rng.randn(20)])
    ground = np.column_stack([rng.uniform(-10,10,200), rng.uniform(-10,10,200), np.full(200,-0.83)])
    canopy = np.column_stack([rng.uniform(-10,10,200), rng.uniform(-10,10,200), np.full(200, 3.0)])
    pts = np.vstack([a,b,ground,canopy])
    c = detect_clusters(pts)
    assert c.shape[1] == 2
    # two clusters found, near the trunk truth
    assert c.shape[0] == 2
    found = sorted([tuple(np.round(p,0)) for p in c])
    assert (0.0,3.0) in found and (5.0,0.0) in found

def test_empty_when_no_points_in_slab():
    pts = np.column_stack([[0,1],[0,1],[3.0,3.0]])  # all canopy
    assert detect_clusters(pts).shape[0] == 0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_cluster_detect.py -v`
Expected: FAIL with `ModuleNotFoundError: landmark_localization.cluster_detect`

- [ ] **Step 3: Write minimal implementation**

```python
# landmark_localization/cluster_detect.py
import numpy as np

def detect_clusters(points_xyz, z_min=-0.70, z_max=-0.40, grid=0.5, min_pts=3):
    p = np.asarray(points_xyz, dtype=float).reshape(-1, 3)
    if p.shape[0] == 0:
        return np.empty((0, 2))
    slab = p[(p[:, 2] >= z_min) & (p[:, 2] <= z_max)]
    if slab.shape[0] == 0:
        return np.empty((0, 2))
    keys = np.floor(slab[:, :2] / grid).astype(np.int64)
    # group by cell key
    order = np.lexsort((keys[:, 1], keys[:, 0]))
    ks = keys[order]
    xy = slab[order, :2]
    centroids = []
    start = 0
    for i in range(1, len(ks) + 1):
        if i == len(ks) or (ks[i] != ks[start]).any():
            if i - start >= min_pts:
                centroids.append(xy[start:i].mean(axis=0))
            start = i
    return np.array(centroids) if centroids else np.empty((0, 2))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_cluster_detect.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add landmark_localization/cluster_detect.py tests/test_cluster_detect.py
git commit -m "feat(landmark): numpy grid-cluster trunk detector"
```

---

### Task 4: Pose solver (`pose_solve.py`)

**Files:**
- Create: `landmark_localization/pose_solve.py`
- Test: `tests/test_pose_solve.py`

**Interfaces:**
- Consumes: cluster centroids (lidar frame, from Task 3), `LandmarkMap` (Task 2).
- Produces:
  - `solve_pose(clusters_xy, map_xy, prior_xytheta, assoc_radius=2.0) -> (x, y, theta, n_used)`
    - `clusters_xy`: `(K,2)` observed centroids in the robot/base frame (x fwd, y left).
    - `map_xy`: `(N,2)` known landmark coords in `map` frame.
    - `prior_xytheta`: `(x,y,theta)` dead-reckoned prior (seed).
    - Transforms observed clusters into `map` frame using the prior, associates each to the nearest map landmark within `assoc_radius`, then computes the rigid 2D transform (Umeyama/Procrustes, no scale) from observed→matched-map points, and returns the corrected absolute pose. `n_used` = number of associated pairs. If `n_used < 2`, return the prior unchanged with `n_used` as-is (caller decides confidence).

- [ ] **Step 1: Write the failing test**

```python
# tests/test_pose_solve.py
import numpy as np
from landmark_localization.pose_solve import solve_pose

def _rot(t):
    return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])

def test_recovers_known_pose():
    # true robot pose in map
    tx, ty, tth = 10.0, -4.0, 0.6
    map_xy = np.array([[12.0, -3.0], [9.0, -6.0], [11.0, -1.0], [7.0, -4.0]])
    # observed = map points expressed in the robot frame at the TRUE pose
    R = _rot(tth)
    obs = (map_xy - [tx, ty]) @ R  # world->body
    # give a slightly wrong prior; solver should correct to truth
    prior = (10.5, -3.6, 0.5)
    x, y, th, n = solve_pose(obs, map_xy, prior)
    assert n >= 3
    assert abs(x - tx) < 0.2 and abs(y - ty) < 0.2
    assert abs((th - tth + np.pi) % (2*np.pi) - np.pi) < 0.05

def test_returns_prior_when_no_association():
    map_xy = np.array([[100.0, 100.0]])
    obs = np.array([[1.0, 0.0], [0.0, 1.0]])
    prior = (0.0, 0.0, 0.0)
    x, y, th, n = solve_pose(obs, map_xy, prior, assoc_radius=1.0)
    assert (x, y, th) == prior
    assert n < 2
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_pose_solve.py -v`
Expected: FAIL with `ModuleNotFoundError: landmark_localization.pose_solve`

- [ ] **Step 3: Write minimal implementation**

```python
# landmark_localization/pose_solve.py
import numpy as np

def _rot(t):
    return np.array([[np.cos(t), -np.sin(t)], [np.sin(t), np.cos(t)]])

def solve_pose(clusters_xy, map_xy, prior_xytheta, assoc_radius=2.0):
    px, py, pth = prior_xytheta
    obs = np.asarray(clusters_xy, dtype=float).reshape(-1, 2)
    mp = np.asarray(map_xy, dtype=float).reshape(-1, 2)
    if obs.shape[0] == 0 or mp.shape[0] == 0:
        return (px, py, pth, 0)
    # observed (body) -> map, using the prior
    obs_map = obs @ _rot(pth).T + np.array([px, py])
    # associate each observed to nearest map landmark within radius
    src, dst = [], []
    for i, o in enumerate(obs_map):
        d = np.hypot(mp[:, 0] - o[0], mp[:, 1] - o[1])
        j = int(np.argmin(d))
        if d[j] <= assoc_radius:
            src.append(obs[i])      # keep in BODY frame for the solve
            dst.append(mp[j])
    n = len(src)
    if n < 2:
        return (px, py, pth, n)
    src = np.array(src); dst = np.array(dst)
    # rigid 2D transform body->map (Umeyama, no scale)
    cs, cd = src.mean(0), dst.mean(0)
    H = (src - cs).T @ (dst - cd)
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, np.linalg.det(Vt.T @ U.T)])
    Rm = Vt.T @ D @ U.T
    theta = np.arctan2(Rm[1, 0], Rm[0, 0])
    t = cd - Rm @ cs
    return (float(t[0]), float(t[1]), float(theta), n)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_pose_solve.py -v`
Expected: PASS (2 tests)

- [ ] **Step 5: Commit**

```bash
git add landmark_localization/pose_solve.py tests/test_pose_solve.py
git commit -m "feat(landmark): rigid-transform pose solver from cluster associations"
```

---

### Task 5: Gated relay node (`gated_relay_node.py`)

**Files:**
- Create: `landmark_localization/gated_relay_node.py`
- Test: `tests/test_gated_relay.py` (logic-only test of the selection function)

**Interfaces:**
- Consumes: `/odometry/gps`, `/odometry/landmark` (`nav_msgs/Odometry`).
- Produces:
  - topic `/odometry/absolute` (`nav_msgs/Odometry`) — the trusted source, republished.
  - service `/localization/trust_landmark` (`std_srvs/SetBool`): `data=true` → pass landmark; `data=false` → pass GPS (default false).
  - Pure helper `select_source(trust_landmark, gps_msg, landmark_msg) -> msg|None` for unit testing.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_gated_relay.py
from landmark_localization.gated_relay_node import select_source

def test_selects_gps_by_default():
    assert select_source(False, "GPS", "LM") == "GPS"

def test_selects_landmark_when_trusted():
    assert select_source(True, "GPS", "LM") == "LM"

def test_none_when_selected_source_missing():
    assert select_source(True, "GPS", None) is None
    assert select_source(False, None, "LM") is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_gated_relay.py -v`
Expected: FAIL with `ModuleNotFoundError`

- [ ] **Step 3: Write minimal implementation**

```python
# landmark_localization/gated_relay_node.py
import rospy
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool, SetBoolResponse

def select_source(trust_landmark, gps_msg, landmark_msg):
    return landmark_msg if trust_landmark else gps_msg

class GatedRelay(object):
    def __init__(self):
        self._trust_landmark = False
        self._gps = None
        self._landmark = None
        self._pub = rospy.Publisher("/odometry/absolute", Odometry, queue_size=10)
        rospy.Subscriber("/odometry/gps", Odometry, self._on_gps, queue_size=10)
        rospy.Subscriber("/odometry/landmark", Odometry, self._on_landmark, queue_size=10)
        rospy.Service("/localization/trust_landmark", SetBool, self._on_trust)

    def _on_trust(self, req):
        self._trust_landmark = bool(req.data)
        src = "landmark" if self._trust_landmark else "gps"
        rospy.logwarn("gated_relay: trusting %s", src)
        return SetBoolResponse(success=True, message="trusting %s" % src)

    def _on_gps(self, msg):
        self._gps = msg
        if not self._trust_landmark:
            self._pub.publish(msg)

    def _on_landmark(self, msg):
        self._landmark = msg
        if self._trust_landmark:
            self._pub.publish(msg)

if __name__ == "__main__":
    rospy.init_node("gated_relay")
    GatedRelay()
    rospy.spin()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_gated_relay.py -v`
Expected: PASS (3 tests)

- [ ] **Step 5: Commit**

```bash
git add landmark_localization/gated_relay_node.py tests/test_gated_relay.py
git commit -m "feat(landmark): gated relay node with SetBool trust service"
```

---

### Task 6: Point the map EKF at the relay output

**Files:**
- Modify: `natural_environments_ros_opt/husky/husky_control/config/localization_map.yaml` (the `odom1:` line, currently `odom1: odometry/gps`)

**Interfaces:**
- Consumes: `/odometry/absolute` from Task 5.
- Produces: the map EKF now fuses whatever the relay passes (GPS by default, unchanged behaviour until a switch).

- [ ] **Step 1: Change the odom1 source**

In `natural_environments_ros_opt/husky/husky_control/config/localization_map.yaml`, change:
```yaml
odom1: odometry/gps
```
to:
```yaml
# odom1 now comes from the gated relay (/odometry/absolute), which republishes
# either /odometry/gps (default) or /odometry/landmark (after the operator
# switches trust). See landmark_localization/gated_relay_node.py.
odom1: odometry/absolute
```
Leave `odom1_config`, `odom1_differential`, `odom1_queue_size` exactly as they are.

- [ ] **Step 2: Verify the edit**

Run: `grep -n "odom1" natural_environments_ros_opt/husky/husky_control/config/localization_map.yaml`
Expected: `odom1: odometry/absolute` present; the `_config`/`_differential`/`_queue_size` lines unchanged.

- [ ] **Step 3: Commit**

```bash
git add natural_environments_ros_opt/husky/husky_control/config/localization_map.yaml
git commit -m "feat(localization): map EKF odom1 reads gated relay /odometry/absolute"
```

---

### Task 7: Landmark localizer node (`landmark_localizer_node.py`)

**Files:**
- Create: `landmark_localization/landmark_localizer_node.py`

**Interfaces:**
- Consumes: `/os0_cloud_node/points` (`PointCloud2`), `/odometry/filtered_map` (dead-reckon prior), `LandmarkMap` (Task 2), `detect_clusters` (Task 3), `solve_pose` (Task 4).
- Produces: `/odometry/landmark` (`nav_msgs/Odometry`, frame `map`), published each time a cloud is processed and `n_used >= 2`.
- Note: clusters from `detect_clusters` are in the `os0_lidar` frame; convert to base frame by translating for the lidar mount (x fwd offset ≈ 0.09 m from `park_1_topic_breakdown.md` frame tree; yaw of the lidar mount is 0). For v1 the small planar offset is applied as a constant; document it inline.

- [ ] **Step 1: Write the node**

```python
# landmark_localization/landmark_localizer_node.py
import os
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2
from tf.transformations import quaternion_from_euler, euler_from_quaternion

from landmark_localization.landmark_map import LandmarkMap
from landmark_localization.cluster_detect import detect_clusters
from landmark_localization.pose_solve import solve_pose

# lidar mount planar offset in base frame (os0_lidar is ~0.09 m fwd of base_link;
# see park_1_topic_breakdown.md frame tree). Yaw offset is 0.
LIDAR_DX = 0.09
LIDAR_DY = 0.0

class LandmarkLocalizer(object):
    def __init__(self):
        map_path = rospy.get_param(
            "~map_path",
            os.path.join(os.path.dirname(__file__), "park_landmarks.json"))
        self.map = LandmarkMap.from_json(map_path)
        self.prior = (0.0, 0.0, 0.0)
        self.have_prior = False
        self.pub = rospy.Publisher("/odometry/landmark", Odometry, queue_size=10)
        rospy.Subscriber("/odometry/filtered_map", Odometry, self._on_prior, queue_size=1)
        rospy.Subscriber("/os0_cloud_node/points", PointCloud2, self._on_cloud, queue_size=1)

    def _on_prior(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.prior = (p.x, p.y, yaw)
        self.have_prior = True

    def _on_cloud(self, msg):
        if not self.have_prior:
            return
        pts = np.array(list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)))
        if pts.shape[0] == 0:
            return
        clusters = detect_clusters(pts)             # os0_lidar frame
        if clusters.shape[0] == 0:
            return
        clusters = clusters + np.array([LIDAR_DX, LIDAR_DY])  # -> base frame
        x, y, th, n = solve_pose(clusters, self.map.xy, self.prior)
        if n < 2:
            return
        self._publish(x, y, th, msg.header.stamp)

    def _publish(self, x, y, th, stamp):
        o = Odometry()
        o.header.stamp = stamp
        o.header.frame_id = "map"
        o.child_frame_id = "base_link"
        o.pose.pose.position.x = x
        o.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_euler(0, 0, th)
        o.pose.pose.orientation.x = qx
        o.pose.pose.orientation.y = qy
        o.pose.pose.orientation.z = qz
        o.pose.pose.orientation.w = qw
        # modest covariance so the EKF trusts it like GPS (x,y strong; yaw weak)
        cov = [0.0] * 36
        cov[0] = 0.5; cov[7] = 0.5; cov[35] = 1.0
        o.pose.covariance = cov
        self.pub.publish(o)

if __name__ == "__main__":
    rospy.init_node("landmark_localizer")
    LandmarkLocalizer()
    rospy.spin()
```

- [ ] **Step 2: Syntax/import check (no ROS master needed)**

Run: `python3 -c "import ast; ast.parse(open('landmark_localization/landmark_localizer_node.py').read()); print('parse OK')"`
Expected: `parse OK`

- [ ] **Step 3: Commit**

```bash
git add landmark_localization/landmark_localizer_node.py
git commit -m "feat(landmark): localizer node publishing /odometry/landmark"
```

---

### Task 8: Operator `trust` command parsing

**Files:**
- Modify: `operator/gcs_commands.py` (extend `parse_command`)
- Test: `tests/test_gcs_commands.py`

**Interfaces:**
- Consumes: existing `parse_command(line) -> (verb, args)` (see `operator/gcs_commands.py`).
- Produces: `parse_command("trust landmark") -> ("trust", ["landmark"])`, `parse_command("trust gps") -> ("trust", ["gps"])`, `parse_command("trust foo") -> ("error", ["trust arg must be landmark or gps"])`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_gcs_commands.py
import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "operator"))
from gcs_commands import parse_command

def test_trust_landmark():
    assert parse_command("trust landmark") == ("trust", ["landmark"])

def test_trust_gps():
    assert parse_command("trust gps") == ("trust", ["gps"])

def test_trust_bad_arg():
    assert parse_command("trust foo") == ("error", ["trust arg must be landmark or gps"])

def test_existing_goal_still_parses():
    assert parse_command("goal 49.9 8.9") == ("goal", [49.9, 8.9])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `python3 -m pytest tests/test_gcs_commands.py -v`
Expected: FAIL on the three `trust` tests (`goal` test passes).

- [ ] **Step 3: Add the `trust` verb to `parse_command`**

In `operator/gcs_commands.py`, inside `parse_command`, add before the final `if verb in (...)` / fallthrough (keep existing `goal` handling intact):
```python
    if verb == "trust":
        if len(parts) != 2 or parts[1] not in ("landmark", "gps"):
            return ("error", ["trust arg must be landmark or gps"])
        return ("trust", [parts[1]])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `python3 -m pytest tests/test_gcs_commands.py -v`
Expected: PASS (4 tests)

- [ ] **Step 5: Commit**

```bash
git add operator/gcs_commands.py tests/test_gcs_commands.py
git commit -m "feat(operator): parse 'trust landmark'/'trust gps' commands"
```

---

### Task 9: Operator dispatch + divergence display

**Files:**
- Modify: `operator/operate.py` (dispatch the `trust` verb; subscribe to `/odometry/landmark`; show GPS-vs-landmark divergence)

**Interfaces:**
- Consumes: `("trust", ["landmark"|"gps"])` from Task 8; the `/localization/trust_landmark` service (Task 5); `/odometry/filtered_map` (already subscribed) and `/odometry/landmark` (Task 7).
- Produces: operator can flip trust from the REPL; the status row shows the divergence between the GPS-fused pose and the landmark pose (the spoof signal).

- [ ] **Step 1: Add the service proxy + landmark subscription in `Operator.__init__`**

After the existing subscribers block (near `operator/operate.py:141-151`), add:
```python
        from nav_msgs.msg import Odometry as _Odom
        self._landmark_pose = None
        rospy.Subscriber("/odometry/landmark", _Odom, self._on_landmark, queue_size=1)
        from std_srvs.srv import SetBool as _SetBool
        self._SetBool = _SetBool
        self._trust_srv_name = "/localization/trust_landmark"
```

- [ ] **Step 2: Add the landmark callback (near the other `_on_*` methods)**

```python
    def _on_landmark(self, msg):
        p = msg.pose.pose.position
        with self._lock:
            self._landmark_pose = (p.x, p.y)
```
(Use the same lock the other callbacks use; check the existing `_on_odom` for the lock attribute name and mirror it.)

- [ ] **Step 3: Dispatch the `trust` verb in the REPL loop**

Where the parsed `(verb, args)` is handled (the command dispatch in `run()`/the REPL), add a branch:
```python
        elif verb == "trust":
            want_landmark = (args[0] == "landmark")
            try:
                rospy.wait_for_service(self._trust_srv_name, timeout=5.0)
                proxy = rospy.ServiceProxy(self._trust_srv_name, self._SetBool)
                resp = proxy(want_landmark)
                print("trust -> %s (%s)" % (args[0], resp.message))
            except Exception as exc:
                print("trust switch failed: %s" % exc)
```

- [ ] **Step 4: Add divergence to the status output**

In `_write_row` (near `operator/operate.py:192`), compute and include the GPS-fused-vs-landmark divergence when both are available:
```python
        gx, gy = (self.state.map_pose or (None, None))   # existing fused map pose
        lm = self._landmark_pose
        if lm is not None and gx is not None:
            div = ((gx - lm[0])**2 + (gy - lm[1])**2) ** 0.5
            div_str = "%.2f" % div
        else:
            div_str = "n/a"
        # append div_str to the printed/written row (match the row's existing format)
```
(Adapt to the actual attribute holding the fused map pose — check `_on_odom`, which stores the `/odometry/filtered_map` pose; reuse that attribute rather than adding a new one.)

- [ ] **Step 5: Syntax check**

Run: `python3 -c "import ast; ast.parse(open('operator/operate.py').read()); print('parse OK')"`
Expected: `parse OK`

- [ ] **Step 6: Commit**

```bash
git add operator/operate.py
git commit -m "feat(operator): trust switch dispatch + GPS-vs-landmark divergence display"
```

---

### Task 10: Launch file for the fallback nodes

**Files:**
- Create: `launch/landmark_fallback.launch`

**Interfaces:**
- Consumes: the nodes from Tasks 5 and 7.
- Produces: one launch that starts `gated_relay` + `landmark_localizer`. Started alongside the demo (after the dual-EKF/navsat stack is up, before/with `move_base_gps.launch`).

- [ ] **Step 1: Write the launch**

```xml
<launch>
  <!-- GPS-spoofing fallback: gated relay + lidar landmark localizer.
       Start AFTER the dual-EKF + navsat stack is up (they publish
       /odometry/gps and /odometry/filtered_map). The map EKF's odom1 now
       reads /odometry/absolute (see localization_map.yaml). -->
  <node pkg="landmark_localization" type="gated_relay_node.py"
        name="gated_relay" output="screen"
        cwd="node" respawn="false"/>
  <node pkg="landmark_localization" type="landmark_localizer_node.py"
        name="landmark_localizer" output="screen"
        cwd="node" respawn="false">
    <param name="map_path"
           value="$(find landmark_localization)/park_landmarks.json"/>
  </node>
</launch>
```
Note: if `landmark_localization` is not a catkin package (`$(find ...)` fails), replace `pkg=`/`type=` with `<node ... command>` using absolute paths, or add a minimal `package.xml`+`CMakeLists.txt`. Decide based on how the repo's other nodes are launched (check whether `operator/` or root scripts are run by path vs `rosrun`). If run-by-path is the norm here, use absolute-path `<node>` entries and an absolute `map_path`.

- [ ] **Step 2: Validate the launch XML parses**

Run: `python3 -c "import xml.dom.minidom as m; m.parse('launch/landmark_fallback.launch'); print('xml OK')"`
Expected: `xml OK`

- [ ] **Step 3: Commit**

```bash
git add launch/landmark_fallback.launch
git commit -m "feat(landmark): launch file for gated relay + localizer"
```

---

### Task 11: Integration test — nominal agreement (live sim)

**Files:**
- Create: `landmark_localization/README.md` (run + verify steps)

**Interfaces:**
- Consumes: everything above + the running demo (per `RUN-GOAL-HIJACK.md` Step 2 world, dual-EKF, `move_base_gps.launch`).

- [ ] **Step 1: Document + run the nominal check**

Write `landmark_localization/README.md` describing:
1. Start the demo world + dual-EKF + move_base (RUN-GOAL-HIJACK.md Steps 2–3).
2. `roslaunch launch/landmark_fallback.launch`.
3. Verify the localizer publishes and AGREES with GPS while GPS is healthy:
```bash
# both should report similar map positions when GPS is honest
rostopic echo -n1 /odometry/landmark/pose/pose/position
rostopic echo -n1 /odometry/filtered_map/pose/pose/position
```
Expected: landmark (x,y) within a couple metres of the fused map pose (validates the localizer + warm-up before it is ever relied on). Record the observed numbers in the README.

- [ ] **Step 2: Commit**

```bash
git add landmark_localization/README.md
git commit -m "docs(landmark): nominal-agreement verification steps + results"
```

---

### Task 12: Integration test — under attack (the real test)

**Files:**
- Modify: `landmark_localization/README.md` (append the under-attack procedure + result)

- [ ] **Step 1: Run the attack + switch, record the outcome**

Append to `landmark_localization/README.md` and execute:
1. Send the real goal (RUN-GOAL-HIJACK.md Step 5).
2. Start the drift spoof: `python3 attack_navsat_drift.py --duration 60 --csv attack_navsat_drift_run.csv`.
3. Watch `operate.py` — the GPS-vs-landmark **divergence grows** (the spoof signal).
4. Operator types `trust landmark`.
5. Verify: `/odometry/absolute` now equals `/odometry/landmark`; the fused map pose snaps back toward truth; the robot **resumes toward the real goal** instead of thrashing.

Success (judged by honest sensors + Gazebo view, NOT ground truth): after the switch, wheel-odom/compass show real forward progress toward the goal and the robot reaches it. Record before/after divergence and the outcome in the README.

- [ ] **Step 2: Commit**

```bash
git add landmark_localization/README.md
git commit -m "docs(landmark): under-attack verification (switch defeats the spoof)"
```

---

## Self-Review Notes

- **Spec coverage:** offline map (T1), map loader (T2), cluster detect (T3), pose solve (T4), gated relay + trust service (T5), EKF odom1 swap (T6), localizer node (T7), operator parse (T8) + dispatch/display (T9), launch (T10), nominal test (T11), under-attack test (T12). Manual-trust policy ✓; automatic chi-squared explicitly deferred (spec References) — not a task. Costmaps/planner untouched ✓.
- **No sklearn** — clustering is numpy grid-cluster (T3), consistent everywhere.
- **Type consistency:** `/odometry/landmark` and `/odometry/absolute` are `nav_msgs/Odometry` throughout; `solve_pose` returns `(x,y,theta,n)` used consistently by T7; `select_source` signature matches T5 test.
- **Open risk flagged, not hidden:** T10 notes the catkin-package-vs-run-by-path decision must be resolved against repo convention at implementation time; T7 notes the constant lidar mount offset as a v1 simplification.
