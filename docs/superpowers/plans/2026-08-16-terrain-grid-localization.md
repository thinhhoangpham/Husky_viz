# Terrain-aware Grid Localization Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add terrain (ground-surface) matching as a second absolute-position cue to the landmark localizer, gravity-de-rotating the scan first, and resolve identical-object aliasing by tracking a few position hypotheses across scans.

**Architecture:** Two independent front-ends share one gravity-de-rotated cloud: the EXISTING crop→cluster→classify→constellation pipeline (objects, Cue A, kept as-is) and a NEW 2.5D terrain grid whose ground surface is gradient-correlated against a prior DTM (terrain, Cue B). A lightweight hypothesis tracker holds the top-K position guesses from either cue across scans and commits one only when it stays consistent for N scans. The map-EKF is untouched — it still receives an absolute (x, y) on the same interface.

**Tech Stack:** Python 3, ROS Noetic (rospy), NumPy, pytest. No new dependencies.

**Spec:** `docs/superpowers/specs/2026-08-16-terrain-grid-localization-design.md`

## Global Constraints

- **No Gazebo ground truth, ever.** No `/gazebo/*` topics/services, no `gazebo_msgs` import, no constant measured from simulator internal state — in code OR tests. (CLAUDE.md hard rule.)
- **NaN, never 0.0, for absent grid data.** A missing cell is `float("nan")`; a fabricated zero is a fake flat plateau that dominates correlation. (Spec §4.)
- **Roll/pitch consumed in the localizer, NOT fused into the EKF.** `two_d_mode: true` stays. Read `/compass/data` directly, as the localizer already does for yaw (`localizer_node.py:478`). (Spec §8.)
- **Grid is terrain-only.** It does NOT detect objects; the classifier is kept. (Spec §5.)
- **Per-scan grid** (no accumulation this iteration). (Spec §10.2, decided.)
- **Absolute z is never assumed.** Terrain matches GRADIENTS so a constant height offset cancels. (Spec §6.)
- **Repo tidiness (CLAUDE.md):** package code in `landmark_loc/`, tests in `landmark_loc/tests/`, outputs in `artifacts/`. Do not add loose files to repo root.
- **Judge sim results by the robot's actual Gazebo position vs the goal**, never by fused pose. (CLAUDE.md.)
- **Frame/row convention:** DTM `.npy` is `z[row, col]`, row 0 = LOWEST y, `origin_x/origin_y` = cell (0,0) corner, `resolution` m/cell. Match `map_tools/dtm_raster.py:DtmGrid` exactly.

---

## File Structure

**Phase 1 — de-rotation (park-testable):**
- Create `landmark_loc/derotate.py` — pure functions: quaternion→(roll,pitch), build the inverse-tilt rotation, apply to an (N,3) array.
- Create `landmark_loc/tests/test_derotate.py`.
- Modify `landmark_loc/localizer_node.py` — subscribe roll/pitch from `/compass/data` (already subscribed for yaw); de-rotate the cloud array once, right after `cloud_to_array`, before `segment.crop`.

**Phase 2 — terrain grid + matcher (lake-testable):**
- Create `landmark_loc/terrain_grid.py` — bin a de-rotated cloud into `min_z` per cell; morphological opening → `ground`. Terrain only.
- Create `landmark_loc/terrain_match.py` — load a prior DTM (`.npy`+`.yaml`), gradient-correlate a local `ground` grid against it within an odom-prior neighbourhood, return a map-frame (x, y) + a score.
- Create `landmark_loc/tests/test_terrain_grid.py`, `landmark_loc/tests/test_terrain_match.py`.
- Modify `landmark_loc/localizer_node.py` — build the terrain grid from the de-rotated cloud, call the matcher, emit its (x, y) as a candidate.

**Phase 3 — hypothesis tracker:**
- Create `landmark_loc/hypothesis_tracker.py` — hold top-K (x,y) candidates, propagate by odom displacement, reinforce/decay, commit on N-scan dominance.
- Create `landmark_loc/tests/test_hypothesis_tracker.py`.
- Modify `landmark_loc/localizer_node.py` — route BOTH cues' candidates through the tracker; publish only the committed pose to `/odometry/landmark_fix`.

---

## Phase 1 — Gravity de-rotation

### Task 1: Quaternion → roll/pitch, and the inverse-tilt rotation matrix

**Files:**
- Create: `landmark_loc/derotate.py`
- Test: `landmark_loc/tests/test_derotate.py`

**Interfaces:**
- Produces:
  - `roll_pitch_from_quat(x, y, z, w) -> (roll, pitch)` — radians, standard aerospace convention (roll about x, pitch about y).
  - `level_rotation(roll, pitch) -> np.ndarray` — a (3,3) matrix that, left-multiplied onto body-frame points `R @ p`, removes tilt (rotates the gravity-tilted frame back to gravity-aligned). Yaw is NOT touched.

- [ ] **Step 1: Write the failing tests**

```python
# landmark_loc/tests/test_derotate.py
import math
import numpy as np
from landmark_loc import derotate


def test_level_quat_gives_zero_roll_pitch():
    # identity quaternion => no tilt
    r, p = derotate.roll_pitch_from_quat(0.0, 0.0, 0.0, 1.0)
    assert abs(r) < 1e-9
    assert abs(p) < 1e-9


def test_pure_pitch_recovered():
    # quaternion for +0.3 rad pitch about y
    half = 0.15
    qx, qy, qz, qw = 0.0, math.sin(half), 0.0, math.cos(half)
    r, p = derotate.roll_pitch_from_quat(qx, qy, qz, qw)
    assert abs(r) < 1e-6
    assert abs(p - 0.3) < 1e-6


def test_level_rotation_is_identity_when_level():
    R = derotate.level_rotation(0.0, 0.0)
    assert np.allclose(R, np.eye(3))


def test_level_rotation_flattens_a_tilted_ground_plane():
    # a ground plane tilted by 0.3 rad pitch: points on z = tan(0.3)*x in the
    # BODY frame. After de-rotation their z should be ~constant (flat).
    pitch = 0.3
    xs = np.linspace(-10, 10, 50)
    pts = np.zeros((50, 3))
    pts[:, 0] = xs
    pts[:, 2] = math.tan(pitch) * xs
    R = derotate.level_rotation(0.0, pitch)
    out = (R @ pts.T).T
    assert out[:, 2].std() < 1e-6  # z is now flat
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_derotate.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'landmark_loc.derotate'`.

- [ ] **Step 3: Write the implementation**

```python
# landmark_loc/derotate.py
"""Gravity de-rotation: remove the robot's roll/pitch from a point cloud so a
tilted scan does not masquerade as sloped terrain. Yaw is left alone (the map
frame's heading comes from the compass yaw elsewhere).

Measured motivation: on the lake slope the robot pitches ~17 deg; at 20 m that
injects a ~6 m false height ramp, larger than the map's true 2.4 m relief. See
docs/superpowers/specs/2026-08-16-terrain-grid-localization-design.md section 2.
"""
import math

import numpy as np


def roll_pitch_from_quat(x, y, z, w):
    """(roll, pitch) in radians from a quaternion, aerospace convention.
    Roll is rotation about x, pitch about y. Yaw is intentionally not returned.
    """
    # roll (x-axis)
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # pitch (y-axis), clamped for numerical safety at the poles
    sinp = 2.0 * (w * y - z * x)
    sinp = max(-1.0, min(1.0, sinp))
    pitch = math.asin(sinp)
    return roll, pitch


def level_rotation(roll, pitch):
    """(3,3) matrix R such that R @ p removes the given roll and pitch from a
    body-frame point p, leaving a gravity-aligned frame. Yaw is untouched.

    We undo pitch then roll: R = Rx(-roll) @ Ry(-pitch).
    """
    cr, sr = math.cos(-roll), math.sin(-roll)
    cp, sp = math.cos(-pitch), math.sin(-pitch)
    rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]], dtype=float)
    ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]], dtype=float)
    return rx @ ry


def derotate_cloud(points, roll, pitch):
    """Apply level_rotation to an (N,3) array. Empty passes through as (0,3)."""
    p = np.asarray(points, dtype=float)
    if len(p) == 0:
        return p.reshape(-1, 3)
    return (level_rotation(roll, pitch) @ p.T).T
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_derotate.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Add a test for `derotate_cloud` empty + shape**

```python
# append to landmark_loc/tests/test_derotate.py
def test_derotate_cloud_empty():
    out = derotate.derotate_cloud(np.zeros((0, 3)), 0.1, 0.2)
    assert out.shape == (0, 3)


def test_derotate_cloud_preserves_count():
    pts = np.random.RandomState(0).randn(17, 3)
    out = derotate.derotate_cloud(pts, 0.1, -0.2)
    assert out.shape == (17, 3)
```

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_derotate.py -v`
Expected: PASS (6 tests).

- [ ] **Step 6: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add landmark_loc/derotate.py landmark_loc/tests/test_derotate.py
git commit -m "feat(landmark_loc): gravity de-rotation helpers for tilted scans"
```

### Task 2: Wire de-rotation into the localizer's cloud path

**Files:**
- Modify: `landmark_loc/localizer_node.py` — `on_compass` (store the full quaternion, not just yaw), and `on_cloud` (de-rotate right after `cloud_to_array`).

**Interfaces:**
- Consumes: `derotate.roll_pitch_from_quat`, `derotate.derotate_cloud` from Task 1.
- Produces: the object pipeline (`segment.crop` onward) now receives a gravity-de-rotated cloud. No signature changes to downstream functions.

- [ ] **Step 1: Read the current compass handler and cloud path**

Read `landmark_loc/localizer_node.py` around `on_compass` (line ~354) and `on_cloud` (line ~386, specifically the `pts = cloud_to_array(msg)` line ~402 and `cropped = segment.crop(...)` line ~405).

- [ ] **Step 2: Store roll/pitch in `on_compass`**

Change `on_compass` to keep roll and pitch alongside yaw. The `state` dict already has `"compass_yaw"`; add `"compass_roll"` and `"compass_pitch"` (initialise both to `None` in the `state = {...}` block near line 320).

```python
def on_compass(msg):
    q = msg.orientation
    state["compass_yaw"] = _yaw(q)
    roll, pitch = derotate.roll_pitch_from_quat(q.x, q.y, q.z, q.w)
    state["compass_roll"] = roll
    state["compass_pitch"] = pitch
```

Add `from landmark_loc import ... derotate` to the existing package import at the top (line ~17: `from landmark_loc import segment, catalog, solve, detector` → add `derotate`).

- [ ] **Step 3: De-rotate in `on_cloud`**

Immediately after `pts = cloud_to_array(msg)` and its empty check, insert de-rotation. Guard on roll/pitch availability (fall back to no de-rotation only if compass hasn't arrived — but note `on_cloud` already returns early if `compass_yaw is None` at line ~395, so roll/pitch are guaranteed present by then).

```python
        pts = cloud_to_array(msg)
        if len(pts) == 0:
            return
        pts = derotate.derotate_cloud(
            pts, state["compass_roll"], state["compass_pitch"])
        cropped = segment.crop(pts, p["z_min"], p["z_max"], p["max_range"])
```

- [ ] **Step 4: Add a focused unit test for the wiring seam**

Because `on_cloud` is a closure inside `main()`, test the seam at the function level instead: assert that a tilted synthetic ground cloud, after `derotate_cloud` with the measured tilt, has a flatter z spread than before. This proves the crop will now see level ground.

```python
# landmark_loc/tests/test_derotate_integration.py
import math
import numpy as np
from landmark_loc import derotate


def test_tilted_ground_is_flatter_after_derotation():
    # synthetic ground tilted 17 deg in pitch (the measured lake slope)
    pitch = math.radians(17.0)
    xs = np.linspace(0, 20, 200)
    pts = np.zeros((200, 3))
    pts[:, 0] = xs
    pts[:, 2] = math.tan(pitch) * xs  # ground rising with range
    before = pts[:, 2].std()
    out = derotate.derotate_cloud(pts, 0.0, pitch)
    after = out[:, 2].std()
    assert after < before * 0.01  # tilt essentially removed
```

- [ ] **Step 5: Run the new test + the full localizer test suite**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/ -v`
Expected: PASS, including all pre-existing localizer tests (the de-rotation is a no-op on level ground, so nothing regresses).

- [ ] **Step 6: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add landmark_loc/localizer_node.py landmark_loc/tests/test_derotate_integration.py
git commit -m "feat(landmark_loc): de-rotate cloud by compass roll/pitch before crop"
```

**PHASE 1 GATE — sim test (main conversation runs this, not a subagent).**
Follow `RUN-MAP-NAV.md` Steps 0–3 on the **park** (de-rotation is a no-op there, ~0 tilt: proves no regression) then on the **lake** (proves de-rotation engages on the slope). Judge object localization by the robot's actual Gazebo position vs the goal marker. Expected: park behaves exactly as before; lake object matching no longer degrades on slopes.

---

## Phase 2 — Terrain grid + gradient matcher

### Task 3: 2.5D terrain grid with morphological ground

**Files:**
- Create: `landmark_loc/terrain_grid.py`
- Test: `landmark_loc/tests/test_terrain_grid.py`

**Interfaces:**
- Consumes: a gravity-de-rotated (N,3) cloud in the MAP frame (points already transformed by the map-frame prior, as `cloud_to_map_frame` does in `localizer_node.py:20`).
- Produces:
  - `bin_min_z(points_map, resolution, origin_x, origin_y, width, height) -> np.ndarray` — (height, width) float32, `min_z` per cell, NaN where no point falls. Row 0 = lowest y (matches `DtmGrid`).
  - `morphological_ground(min_z, window_cells) -> np.ndarray` — opening (erode then dilate) that removes up-poking objects, leaving the ground surface. NaN-aware.

- [ ] **Step 1: Write the failing tests**

```python
# landmark_loc/tests/test_terrain_grid.py
import numpy as np
from landmark_loc import terrain_grid


def test_bin_min_z_takes_lowest_point_per_cell():
    # two points in the same cell at different z -> min kept
    pts = np.array([[0.1, 0.1, 5.0],
                    [0.2, 0.2, 2.0]], dtype=float)
    g = terrain_grid.bin_min_z(pts, resolution=1.0,
                               origin_x=0.0, origin_y=0.0, width=1, height=1)
    assert g.shape == (1, 1)
    assert abs(g[0, 0] - 2.0) < 1e-9


def test_bin_min_z_empty_cell_is_nan():
    pts = np.array([[0.5, 0.5, 3.0]], dtype=float)
    g = terrain_grid.bin_min_z(pts, resolution=1.0,
                               origin_x=0.0, origin_y=0.0, width=2, height=2)
    assert not np.isnan(g[0, 0])   # cell with the point
    assert np.isnan(g[1, 1])       # empty cell -> NaN, never 0.0


def test_bin_min_z_row_zero_is_lowest_y():
    # a point at low y must land in row 0
    pts = np.array([[0.5, 0.5, 1.0]], dtype=float)
    g = terrain_grid.bin_min_z(pts, resolution=1.0,
                               origin_x=0.0, origin_y=0.0, width=1, height=2)
    assert not np.isnan(g[0, 0])
    assert np.isnan(g[1, 0])


def test_morphological_ground_removes_a_bump():
    # flat ground at z=0 with one tall object cell; opening should flatten it
    z = np.zeros((7, 7), dtype=float)
    z[3, 3] = 5.0  # a "tree"
    ground = terrain_grid.morphological_ground(z, window_cells=3)
    assert abs(ground[3, 3]) < 1e-6  # bump removed


def test_morphological_ground_keeps_a_broad_slope():
    # a slope wider than the window must survive (it is terrain, not an object)
    xs = np.arange(20)
    z = np.tile(xs.astype(float), (20, 1))  # ramp in x
    ground = terrain_grid.morphological_ground(z, window_cells=3)
    # interior values preserved to within the window's reach
    assert abs(ground[10, 10] - z[10, 10]) <= 3.0
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_terrain_grid.py -v`
Expected: FAIL — module missing.

- [ ] **Step 3: Write the implementation**

```python
# landmark_loc/terrain_grid.py
"""2.5D terrain grid: bin a map-frame cloud into per-cell min_z, then extract a
local ground surface by morphological opening. TERRAIN ONLY -- this does not
detect objects (the classifier does that). See design spec sections 4-6.

Grid convention matches map_tools/dtm_raster.py:DtmGrid -- z[row, col], row 0 =
lowest y, origin_(x,y) = cell (0,0) corner, resolution metres/cell. Absent cells
are NaN, never 0.0 (a fake zero is a fabricated flat plateau).
"""
import numpy as np


def bin_min_z(points_map, resolution, origin_x, origin_y, width, height):
    """(height, width) float32 of min z per cell; NaN where no point falls."""
    z = np.full((height, width), np.nan, dtype=np.float32)
    p = np.asarray(points_map, dtype=float)
    if len(p) == 0:
        return z
    cols = ((p[:, 0] - origin_x) / resolution).astype(int)
    rows = ((p[:, 1] - origin_y) / resolution).astype(int)
    inb = (cols >= 0) & (cols < width) & (rows >= 0) & (rows < height)
    cols, rows, zz = cols[inb], rows[inb], p[inb, 2]
    # min per cell: sort so lowest z last-writes each (row,col)
    order = np.argsort(-zz)  # descending z; np assignment keeps the last write
    flat = rows[order] * width + cols[order]
    zf = z.reshape(-1)
    zf[flat] = zz[order]
    return z


def _min_filter(z, k):
    """NaN-aware min over a (2k+1) square window."""
    h, w = z.shape
    out = np.full_like(z, np.nan)
    for r in range(h):
        r0, r1 = max(0, r - k), min(h, r + k + 1)
        for c in range(w):
            c0, c1 = max(0, c - k), min(w, c + k + 1)
            win = z[r0:r1, c0:c1]
            v = win[np.isfinite(win)]
            if v.size:
                out[r, c] = v.min()
    return out


def _max_filter(z, k):
    """NaN-aware max over a (2k+1) square window."""
    h, w = z.shape
    out = np.full_like(z, np.nan)
    for r in range(h):
        r0, r1 = max(0, r - k), min(h, r + k + 1)
        for c in range(w):
            c0, c1 = max(0, c - k), min(w, c + k + 1)
            win = z[r0:r1, c0:c1]
            v = win[np.isfinite(win)]
            if v.size:
                out[r, c] = v.max()
    return out


def morphological_ground(min_z, window_cells):
    """Opening (erode=min then dilate=max) removes up-poking objects and leaves
    the slope-following ground surface. `window_cells` is the half-width k; it
    must exceed the widest object and stay under the terrain feature scale.
    """
    k = int(window_cells)
    return _max_filter(_min_filter(min_z, k), k)
```

Note: the double loop is O(cells × window²). Grids here are ≤ 80×80 with k≈4 → trivial. If a live profile shows it hot, swap to `scipy.ndimage.grey_opening` — but do NOT add scipy speculatively (YAGNI).

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_terrain_grid.py -v`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add landmark_loc/terrain_grid.py landmark_loc/tests/test_terrain_grid.py
git commit -m "feat(landmark_loc): 2.5D terrain grid with morphological ground"
```

### Task 4: DTM loading + gradient correlation matcher

**Files:**
- Create: `landmark_loc/terrain_match.py`
- Test: `landmark_loc/tests/test_terrain_match.py`

**Interfaces:**
- Consumes: `terrain_grid.morphological_ground` output (local ground grid); a prior DTM `.npy`+`.yaml` (keys verified: `resolution`, `origin_x`, `origin_y`, `width`, `height`; `.npy` is float32 `[row][col]`, row 0 = lowest y, NaN = no coverage).
- Produces:
  - `load_dtm(npy_path, yaml_path) -> Dtm` where `Dtm` has `.z, .resolution, .origin_x, .origin_y`.
  - `gradient(z) -> (gx, gy)` — NaN-aware finite differences; NaN where either neighbour is NaN.
  - `match_terrain(local_ground, local_res, prior, prior_xy, search_radius_m) -> (x, y, score) | None` — slide the local ground gradient over the prior gradient within `search_radius_m` of `prior_xy`, return the best-correlation map-frame (x, y) and a normalized score in [0,1]; None if too few valid overlapping cells.

- [ ] **Step 1: Write the failing tests**

```python
# landmark_loc/tests/test_terrain_match.py
import numpy as np
from landmark_loc import terrain_match


def _synthetic_dtm(tmp_path):
    # a smooth ramp+bump prior, 40x40 at 0.5 m
    ys, xs = np.mgrid[0:40, 0:40].astype(float)
    z = 0.05 * xs + 0.03 * ys
    z[20:24, 20:24] += 1.0  # a distinctive bump
    npy = tmp_path / "p_dtm.npy"
    yml = tmp_path / "p_dtm.yaml"
    np.save(npy, z.astype(np.float32))
    yml.write_text(
        "resolution: 0.5\norigin_x: 0.0\norigin_y: 0.0\n"
        "width: 40\nheight: 40\n")
    return npy, yml, z


def test_load_dtm_reads_geometry(tmp_path):
    npy, yml, z = _synthetic_dtm(tmp_path)
    d = terrain_match.load_dtm(str(npy), str(yml))
    assert d.resolution == 0.5
    assert d.z.shape == (40, 40)
    assert d.origin_x == 0.0


def test_gradient_nan_aware():
    z = np.array([[0.0, 1.0, np.nan],
                  [0.0, 1.0, 2.0]], dtype=float)
    gx, gy = terrain_match.gradient(z)
    assert np.isfinite(gx[1, 0])         # both neighbours finite
    assert np.isnan(gx[0, 1])            # right neighbour is NaN


def test_match_finds_zero_offset_when_local_equals_prior_patch(tmp_path):
    npy, yml, z = _synthetic_dtm(tmp_path)
    d = terrain_match.load_dtm(str(npy), str(yml))
    # local patch = exact copy of prior rows 10:30, cols 10:30
    local = z[10:30, 10:30].astype(np.float32).copy()
    # its true map-frame origin corner:
    true_x = 10 * 0.5
    true_y = 10 * 0.5
    # prior guess a bit off; matcher should recover the true patch center
    cx = true_x + 20 * 0.5 * 0.5  # patch-center-ish prior
    cy = true_y + 20 * 0.5 * 0.5
    res = terrain_match.match_terrain(local, 0.5, d, (cx, cy), search_radius_m=3.0)
    assert res is not None
    x, y, score = res
    # recovered center should sit within a cell of the true patch center
    tcx = true_x + (local.shape[1] * 0.5) / 2.0
    tcy = true_y + (local.shape[0] * 0.5) / 2.0
    assert abs(x - tcx) <= 0.5
    assert abs(y - tcy) <= 0.5
    assert score > 0.9


def test_match_returns_none_on_too_few_valid_cells():
    z = np.full((40, 40), np.nan, dtype=np.float32)
    class _D:
        pass
    d = _D(); d.z = z; d.resolution = 0.5; d.origin_x = 0.0; d.origin_y = 0.0
    local = np.full((10, 10), np.nan, dtype=np.float32)
    assert terrain_match.match_terrain(local, 0.5, d, (5.0, 5.0), 3.0) is None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_terrain_match.py -v`
Expected: FAIL — module missing.

- [ ] **Step 3: Write the implementation**

```python
# landmark_loc/terrain_match.py
"""Terrain localization by GRADIENT correlation of a local ground grid against a
prior DTM. Gradients (not raw heights) so a constant elevation offset -- unknown
robot height, sloped ground -- cancels exactly. North-aligned by compass upstream,
so no rotation search here. See design spec section 6.
"""
import numpy as np
import yaml


class Dtm(object):
    __slots__ = ("z", "resolution", "origin_x", "origin_y")

    def __init__(self, z, resolution, origin_x, origin_y):
        self.z = z
        self.resolution = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y


def load_dtm(npy_path, yaml_path):
    z = np.load(npy_path)
    m = yaml.safe_load(open(yaml_path))
    return Dtm(z.astype(np.float32), float(m["resolution"]),
               float(m["origin_x"]), float(m["origin_y"]))


def gradient(z):
    """NaN-aware forward differences. gx[i,j]=z[i,j+1]-z[i,j], gy[i,j]=
    z[i+1,j]-z[i,j]; NaN where either operand is NaN (last row/col also NaN)."""
    h, w = z.shape
    gx = np.full_like(z, np.nan)
    gy = np.full_like(z, np.nan)
    gx[:, :w - 1] = z[:, 1:] - z[:, :w - 1]
    gy[:h - 1, :] = z[1:, :] - z[:h - 1, :]
    return gx, gy


def _score_at(lgx, lgy, pgx, pgy, r0, c0):
    """Normalized gradient-match score for placing the local grid's (0,0) at
    prior cell (r0,c0). Correlation over cells finite in BOTH; None if <25 pairs.
    Score = 1 / (1 + mean squared gradient difference), in (0,1]."""
    h, w = lgx.shape
    ph, pw = pgx.shape
    if r0 < 0 or c0 < 0 or r0 + h > ph or c0 + w > pw:
        return None
    px = pgx[r0:r0 + h, c0:c0 + w]
    py = pgy[r0:r0 + h, c0:c0 + w]
    m = np.isfinite(lgx) & np.isfinite(px) & np.isfinite(lgy) & np.isfinite(py)
    if m.sum() < 25:
        return None
    d = (lgx[m] - px[m]) ** 2 + (lgy[m] - py[m]) ** 2
    return 1.0 / (1.0 + float(d.mean()))


def match_terrain(local_ground, local_res, prior, prior_xy, search_radius_m):
    """Slide the local ground gradient over the prior gradient within
    search_radius_m of prior_xy. Return (x, y, score): the map-frame CENTER of
    the best placement of the local grid, and its score. None if no placement
    has enough valid overlap. Assumes local_res == prior.resolution.
    """
    if abs(local_res - prior.resolution) > 1e-6:
        raise ValueError("local/prior resolution mismatch: %r vs %r"
                         % (local_res, prior.resolution))
    lgx, lgy = gradient(local_ground)
    pgx, pgy = gradient(prior.z)
    h, w = local_ground.shape
    res = prior.resolution
    # prior cell that the search centers on (place local-grid center at prior_xy)
    cx_cell = int(round((prior_xy[0] - prior.origin_x) / res - w / 2.0))
    cy_cell = int(round((prior_xy[1] - prior.origin_y) / res - h / 2.0))
    rad = int(round(search_radius_m / res))
    best = None
    for dr in range(-rad, rad + 1):
        for dc in range(-rad, rad + 1):
            s = _score_at(lgx, lgy, pgx, pgy, cy_cell + dr, cx_cell + dc)
            if s is None:
                continue
            if best is None or s > best[0]:
                best = (s, cy_cell + dr, cx_cell + dc)
    if best is None:
        return None
    s, r0, c0 = best
    # map-frame center of the placed local grid
    x = prior.origin_x + (c0 + w / 2.0) * res
    y = prior.origin_y + (r0 + h / 2.0) * res
    return (x, y, s)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_terrain_match.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Add a real-DTM smoke test (lake), skipped if the file is absent**

```python
# append to landmark_loc/tests/test_terrain_match.py
import os
import pytest

_LAKE = "/home/thinh/Documents/Husky_viz/maps/lake_dtm.npy"


@pytest.mark.skipif(not os.path.exists(_LAKE), reason="lake_dtm not generated")
def test_self_match_on_real_lake_dtm():
    d = terrain_match.load_dtm(_LAKE, _LAKE.replace(".npy", ".yaml"))
    # cut a 30x30 patch out of the middle and match it back to itself
    r, c = d.z.shape[0] // 2, d.z.shape[1] // 2
    local = d.z[r:r + 30, c:c + 30].astype(np.float32).copy()
    tcx = d.origin_x + (c + 15) * d.resolution
    tcy = d.origin_y + (r + 15) * d.resolution
    res = terrain_match.match_terrain(local, d.resolution, d, (tcx, tcy), 4.0)
    assert res is not None
    x, y, score = res
    assert abs(x - tcx) <= d.resolution
    assert abs(y - tcy) <= d.resolution
```

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_terrain_match.py -v`
Expected: PASS (5 tests; the last self-matches on the real lake DTM).

- [ ] **Step 6: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add landmark_loc/terrain_match.py landmark_loc/tests/test_terrain_match.py
git commit -m "feat(landmark_loc): DTM gradient-correlation terrain matcher"
```

### Task 5: Wire the terrain cue into the localizer

**Files:**
- Modify: `landmark_loc/localizer_node.py` — new params (`~dtm_path`, `~terrain_window`, `~terrain_search_radius`, `~terrain_score_gate`); build the local terrain grid from the de-rotated MAP-frame cloud; call `match_terrain`; emit its (x, y) as a candidate for the tracker (Phase 3). Until Phase 3 lands, log it under `[terrain]` and do NOT yet publish it.

**Interfaces:**
- Consumes: `terrain_grid.bin_min_z`, `terrain_grid.morphological_ground`, `terrain_match.load_dtm`, `terrain_match.match_terrain` from Tasks 3–4; the existing `cloud_to_map_frame` (line 20) and `compose_prior` (line 82).
- Produces: a per-tick terrain candidate `(x, y, score)` or None, available to Phase 3.

- [ ] **Step 1: Load the DTM once at startup (optional param)**

In `main()`, after the catalog load (~line 315), add:

```python
    dtm = None
    dtm_path = rospy.get_param("~dtm_path", "")
    if dtm_path:
        dtm = terrain_match.load_dtm(dtm_path, dtm_path.replace(".npy", ".yaml"))
        rospy.loginfo("[terrain] loaded DTM %s (%dx%d @ %.2fm)",
                      dtm_path, dtm.z.shape[1], dtm.z.shape[0], dtm.resolution)
    p["terrain_window"] = rospy.get_param("~terrain_window", 4)
    p["terrain_search_radius"] = rospy.get_param("~terrain_search_radius", 5.0)
    p["terrain_score_gate"] = rospy.get_param("~terrain_score_gate", 0.5)
```

Add `terrain_grid` and `terrain_match` to the package import (line ~17).

- [ ] **Step 2: Compute the terrain candidate in `on_cloud`**

After the object pipeline's `solve.solve_pose` block, and only if `dtm is not None`, build a local grid over a window around the prior and match. Use the DTM's own resolution/origin so the grids align.

```python
        terrain_cand = None
        if dtm is not None:
            map_pts = cloud_to_map_frame(pts, prior)   # pts already de-rotated
            g = terrain_grid.bin_min_z(
                map_pts, dtm.resolution, dtm.origin_x, dtm.origin_y,
                dtm.z.shape[1], dtm.z.shape[0])
            ground = terrain_grid.morphological_ground(g, p["terrain_window"])
            tm = terrain_match.match_terrain(
                ground, dtm.resolution, dtm, (prior[0], prior[1]),
                p["terrain_search_radius"])
            if tm is not None and tm[2] >= p["terrain_score_gate"]:
                terrain_cand = tm
                rospy.loginfo_throttle(1.0,
                    "[terrain] fix=(%.2f,%.2f) score=%.2f" % tm)
```

Note: binning the whole cloud into the full-map grid is wasteful but correct and simple; the search radius bounds the matcher's work. Optimize only if a live profile shows it hot (YAGNI).

- [ ] **Step 3: Regression-test that the object path is unchanged when no DTM is set**

Run the full existing suite; with `~dtm_path` unset, `dtm is None`, so `on_cloud` behaves exactly as after Phase 1.

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/ -v`
Expected: PASS, no regressions.

- [ ] **Step 4: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add landmark_loc/localizer_node.py
git commit -m "feat(landmark_loc): compute terrain candidate from DTM in on_cloud"
```

**PHASE 2 GATE — sim test (main conversation).**
Generate the lake DTM if absent (`python3 -m map_tools.extract_dtm --world lake`). Follow `RUN-MAP-NAV.md` Steps 0–3 on the **lake**, adding `_dtm_path:=/home/thinh/Documents/Husky_viz/maps/lake_dtm.npy` to the localizer (Terminal 3). Drive a goal; watch the `[terrain]` log line. Expected: terrain fixes appear with score above the gate and track the robot's actual Gazebo position as it crosses the relief. On the **park**, terrain score should be low/absent (6.9 mm relief) — confirming the cue correctly contributes nothing where there is no signal.

---

## Phase 3 — Hypothesis tracker

### Task 6: The tracker (hold top-K guesses, commit on dominance)

**Files:**
- Create: `landmark_loc/hypothesis_tracker.py`
- Test: `landmark_loc/tests/test_hypothesis_tracker.py`

**Interfaces:**
- Produces:
  - `Hypothesis` with `.x, .y, .support` (int scan-count of consecutive reinforcement).
  - `HypothesisTracker(k=5, merge_dist=2.0, commit_support=3)` with:
    - `predict(dx, dy)` — shift every hypothesis by the odom displacement since last update.
    - `update(candidates)` — `candidates` is a list of `(x, y)`; each reinforces the nearest hypothesis within `merge_dist` (support += 1) or seeds a new one (support = 1); unreinforced hypotheses decay (support -= 1) and are dropped at 0; keep only the top-`k` by support.
    - `committed()` — the single hypothesis with `support >= commit_support` and highest support, or None.

- [ ] **Step 1: Write the failing tests**

```python
# landmark_loc/tests/test_hypothesis_tracker.py
from landmark_loc.hypothesis_tracker import HypothesisTracker


def test_single_consistent_candidate_commits_after_n():
    t = HypothesisTracker(commit_support=3, merge_dist=1.0)
    assert t.committed() is None
    t.update([(10.0, 5.0)])
    assert t.committed() is None            # support 1
    t.update([(10.1, 5.0)])
    assert t.committed() is None            # support 2
    t.update([(9.9, 5.1)])
    c = t.committed()
    assert c is not None and abs(c.x - 10.0) < 0.5


def test_impostor_dies_when_it_stops_being_seen():
    t = HypothesisTracker(commit_support=3, merge_dist=1.0)
    # both seen once
    t.update([(10.0, 5.0), (40.0, 30.0)])
    # only the true one keeps being seen; impostor decays
    t.update([(10.0, 5.0)])
    t.update([(10.0, 5.0)])
    c = t.committed()
    assert c is not None and abs(c.x - 10.0) < 0.5
    # impostor should be gone (support decayed to 0)
    assert all(abs(h.x - 40.0) > 1.0 for h in t.hypotheses)


def test_predict_shifts_hypotheses():
    t = HypothesisTracker(merge_dist=0.5)
    t.update([(1.0, 1.0)])
    t.predict(2.0, 0.0)
    # after predict, a candidate at (3,1) reinforces the SAME hypothesis
    t.update([(3.0, 1.0)])
    assert len(t.hypotheses) == 1
    assert t.hypotheses[0].support == 2


def test_keeps_only_top_k():
    t = HypothesisTracker(k=2, merge_dist=0.5)
    t.update([(0, 0), (10, 0), (20, 0), (30, 0)])
    assert len(t.hypotheses) <= 2
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_hypothesis_tracker.py -v`
Expected: FAIL — module missing.

- [ ] **Step 3: Write the implementation**

```python
# landmark_loc/hypothesis_tracker.py
"""Lightweight multi-hypothesis position tracker: hold a few candidate (x, y)
guesses, carry them across scans, and commit the one that stays consistent.

This is the cure for identical-object aliasing (23 identical trees etc.): a
single scan cannot tell a correct match from a plausible-but-wrong one, but only
the correct hypothesis stays reinforced as the robot moves. It is a particle
filter with a handful of particles and no resampling (design spec section 7).
"""
import math


class Hypothesis(object):
    __slots__ = ("x", "y", "support")

    def __init__(self, x, y, support=1):
        self.x = x
        self.y = y
        self.support = support


class HypothesisTracker(object):
    def __init__(self, k=5, merge_dist=2.0, commit_support=3):
        self.k = k
        self.merge_dist = merge_dist
        self.commit_support = commit_support
        self.hypotheses = []

    def predict(self, dx, dy):
        for h in self.hypotheses:
            h.x += dx
            h.y += dy

    def update(self, candidates):
        reinforced = set()
        for (cx, cy) in candidates:
            best, bestd = None, self.merge_dist
            for h in self.hypotheses:
                d = math.hypot(h.x - cx, h.y - cy)
                if d <= bestd:
                    best, bestd = h, d
            if best is not None:
                # move toward the candidate a little; bump support
                best.x = 0.5 * (best.x + cx)
                best.y = 0.5 * (best.y + cy)
                best.support += 1
                reinforced.add(id(best))
            else:
                self.hypotheses.append(Hypothesis(cx, cy, 1))
                reinforced.add(id(self.hypotheses[-1]))
        # decay the unreinforced
        for h in self.hypotheses:
            if id(h) not in reinforced:
                h.support -= 1
        self.hypotheses = [h for h in self.hypotheses if h.support > 0]
        # keep top-k by support
        self.hypotheses.sort(key=lambda h: h.support, reverse=True)
        self.hypotheses = self.hypotheses[:self.k]

    def committed(self):
        best = None
        for h in self.hypotheses:
            if h.support >= self.commit_support:
                if best is None or h.support > best.support:
                    best = h
        return best
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_hypothesis_tracker.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add landmark_loc/hypothesis_tracker.py landmark_loc/tests/test_hypothesis_tracker.py
git commit -m "feat(landmark_loc): lightweight multi-hypothesis position tracker"
```

### Task 7: Route both cues through the tracker; publish the committed pose

**Files:**
- Modify: `landmark_loc/localizer_node.py` — instantiate the tracker in `state`; each tick `predict` by odom displacement, `update` with the object fix and the terrain candidate, and publish ONLY the committed pose to `/odometry/landmark_fix`.

**Interfaces:**
- Consumes: `HypothesisTracker` (Task 6); the object `solve_pose` result `(x, y, yaw, rms, n)`; the `terrain_cand` `(x, y, score)` (Task 5).
- Produces: `/odometry/landmark_fix` now carries the committed hypothesis, not a raw per-scan fix.

- [ ] **Step 1: Add tracker params + state**

In `main()` params: `p["tracker_k"] = rospy.get_param("~tracker_k", 5)`, `p["tracker_merge"] = rospy.get_param("~tracker_merge", 2.0)`, `p["tracker_commit"] = rospy.get_param("~tracker_commit", 3)`. In `state`: `"tracker": hypothesis_tracker.HypothesisTracker(p["tracker_k"], p["tracker_merge"], p["tracker_commit"])` and `"tracker_last_odom": None`. Add `hypothesis_tracker` to the import.

- [ ] **Step 2: Predict + update + commit in `on_cloud`**

Replace the direct publish of the object fix with: gather candidates, predict by odom displacement since last tick, update, and publish the committed hypothesis. Keep the existing `_jump_ok`, `fix_history` median, and covariance machinery applied to the COMMITTED pose (they remain a useful output smoother).

```python
        cands = []
        if result is not None:
            cands.append((result[0], result[1]))        # object fix (x, y)
        if terrain_cand is not None:
            cands.append((terrain_cand[0], terrain_cand[1]))  # terrain fix
        tr = state["tracker"]
        if state["tracker_last_odom"] is not None:
            dx = odom_synced[0] - state["tracker_last_odom"][0]
            dy = odom_synced[1] - state["tracker_last_odom"][1]
            tr.predict(dx, dy)
        state["tracker_last_odom"] = odom_synced
        tr.update(cands)
        committed = tr.committed()
        if committed is None:
            return
        x, y = committed.x, committed.y
        # ... existing _jump_ok gate, fix_history median, covariance, publish ...
```

The remaining publish block (build `Odometry`, stamp with `msg.header.stamp`, covariance, `pub.publish`) is unchanged except it now uses the committed `(x, y)`.

- [ ] **Step 3: Regression-test with no DTM (object-only through tracker)**

With `~dtm_path` unset, only object fixes feed the tracker. The existing localizer tests that exercise `on_cloud` behavior at the seam should still pass; run the full suite.

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/ -v`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
cd /home/thinh/Documents/Husky_viz
git add landmark_loc/localizer_node.py
git commit -m "feat(landmark_loc): commit fixes via hypothesis tracker over both cues"
```

**PHASE 3 GATE — sim test (main conversation).**
Follow `RUN-MAP-NAV.md` Steps 0–3 on the **park** (object cue only — verify the tracker commits and the robot still reaches goals by its actual Gazebo position) and the **lake** (both cues — verify terrain contributes and the robot localizes across the relief). The key qualitative check: a run that previously produced an occasional confident wrong fix near an identical-tree cluster should now NOT commit to the wrong cluster, because the impostor hypothesis decays before reaching `commit_support`.

---

## Self-Review

**Spec coverage:**
- §3 architecture (two front-ends, shared de-rotated cloud) → Tasks 2, 5.
- §4 terrain grid (min_z, morphological ground, NaN) → Task 3.
- §5 objects kept, input rewired → Task 2.
- §6 gradient correlation, offset-invariant → Task 4.
- §7 hypothesis tracker → Tasks 6, 7.
- §8 locked decisions (two_d_mode stays, roll/pitch in localizer, per-scan) → Global Constraints + Task 2, 5.
- §9 traversability → explicitly deferred; no task (correct — out of scope).
- §11 risks: noise-free IMU (documented, no code), park-can't-validate-terrain (Phase 2 gate note), coverage gap/NaN (Tasks 3, 4 NaN handling).
- §12 components → Tasks 1–7 map 1:1 (terrain_grid.py, terrain_match.py, derotate.py, hypothesis_tracker.py, extract_dtm already done).

**Placeholder scan:** no TBD/TODO; every code step has real code; every test step has real assertions.

**Type consistency:** `Dtm`/`DtmGrid` both expose `.z/.resolution/.origin_x/.origin_y` (Task 4 `Dtm` mirrors `map_tools.dtm_raster.DtmGrid`). `match_terrain` returns `(x, y, score)` — consumed as `terrain_cand` in Task 5 and `terrain_cand[0/1]` in Task 7. `solve_pose` returns `(x, y, yaw, rms, n)` — Task 7 uses `result[0], result[1]`. Tracker `committed()` returns a `Hypothesis` with `.x/.y` — used in Task 7. Consistent.

## Notes for the executor

- **Sim gates are run by the main conversation, never a subagent** (CLAUDE.md: agents implement + pytest + commit only; main runs the sim from a clean kill). A subagent finishing a phase should STOP at the gate and report; it does not run `roslaunch`/Gazebo/Docker.
- **Every sim test starts from the full teardown block** in `RUN-MAP-NAV.md` and follows Steps 0–3 verbatim — no skipped steps.
- **The lake DTM already exists** (`maps/lake_dtm.npy`, generated this session). Regenerate with `python3 -m map_tools.extract_dtm --world lake` if needed.
- **`/imu/data` is NOT the attitude source** — it is 90°-mis-mounted. Roll/pitch come from `/compass/data` (gravity on Z, verified). Do not switch sources.
