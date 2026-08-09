# Offline Map + Route Planning Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the park robot a preloaded offline map (occupancy grid + named places, extracted from `park.world`) so move_base plans a route around known obstacles at the start of a run, with live lidar still adding obstacles the map did not know about.

**Architecture:** Extend the EXISTING GPS-anchored move_base stack (`launch/move_base_gps.launch`, both costmaps already in the drift-free `map` frame). Add (1) an offline extractor that turns `park.world` into a ROS `map_server` grid + a `park_places.yaml` name→xy table, (2) a `map_server` + `StaticLayer` under the existing lidar `ObstacleLayer` so the static map is the baseline and lidar is additive, and (3) map-frame and named goal verbs in the operator prompt. No new planner is written — NavfnROS (global) + DWAPlannerROS (local) already exist and are tuned.

**Tech Stack:** Python 3 (ROS Noetic, rospy), pytest, ROS `map_server`/`costmap_2d`, PGM/YAML map format, SDF (Gazebo world XML), `move_base_msgs`, `actionlib`.

## Global Constraints

- **NO ground truth ever.** No `/gazebo/get_model_state`, `/gazebo/model_states`, `gazebo_msgs`, or any constant measured from simulator runtime state. Map data comes from the static `park.world` file (a design asset), never from a running sim. Verification is by the robot's own sensors + operator/RViz view.
- **Map/costmap frame is `map`** (GPS-anchored, drift-free). Everything the extractor emits and every goal is in the `map` frame.
- **Map resolution = 0.15 m/cell**, matching `config/costmap_global_gps.yaml:resolution: 0.15`. No resampling.
- **No pre-inflation in the extractor.** Emit raw obstacle footprints; the costmap's tuned `InflationLayer` (`inflation_radius: 0.5`, `cost_scaling_factor: 6.0` in `config/costmap_common_gps.yaml`) inflates.
- **Tree trunk = `link_0` for BOTH tree families** (verified in-sim 2026-08-09). `arbolpartes4*` is multi-link (trunk `link_0`/`tronco4.dae` offset ~1 m from model `<pose>`); `tree_8*` is single-link (`bark8`, no offset). Read geometry from model *definitions* (SDF after line ~4765), never the `<state>` snapshot (line ~146+).
- **All obstacle collisions are meshes** (`.dae`/`.obj`) — no SDF primitives. Footprints use per-family constant radii (below), not mesh parsing.
- **Config paths are absolute** in launch files (repo convention: `/home/thinh/Documents/Husky_viz/...`).
- **Do not modify** `drive_to_point_gps.py`, `move_base_gps.launch`, `move_base_park.launch`, `costmap_global_gps.yaml`, or any `attack_*.py`. New/parallel files only.

## Reference facts (from the codebase, for the implementer)

- **Tree models & counts** (model definitions, geometry-bearing): `arbolpartes4*` = 15 trees (trunk mesh `arbol4//tronco4.dae`, canopy `arbol4/copa4.dae`); `tree_8*` = 23 trees (trunk mesh `bark8_collision.obj`). Total 38 trees.
- **Furniture (obstacles):** `bench*` (16), `garden_table*` (22), `lamp*` (30), `trash_bin_1*` (22). Collisions are meshes.
- **Drivable (skip):** `parque` / `terreno_parque` (ground), `camino_parque` (trail), `Untitled2` (container — skip; verify no geometry).
- **World file:** `natural_environments_ros_opt/natural_enviroment/worlds/park.world`. Two blocks: `<state world_name='default'>` (line ~146, poses only) and model definitions (line ~4765+, poses + geometry). Model world pose lives in the `<state>` block per model as `<model name=..><pose>`; for `arbolpartes4` the trunk is the `<state>` model's `link_0` `<pose>`.
- **GPS→map datum:** `referenceLatitude 49.9`, `referenceLongitude 8.9` (`gps.urdf.xacro`); the map origin is this datum. World +x = NORTH, world +y = WEST.
- **Existing goal path:** `operator/gcs_commands.py:parse_command()` parses the prompt line; `operator/operate.py:_dispatch()` (line ~248) routes `goal` → `_do_goal(lat, lon)` (line ~275) → `latlon_to_map()` → `MoveBaseGoal` in `map` frame via `SimpleActionClient("move_base", MoveBaseAction)`.
- **Configs to mirror for the new launch:** `config/costmap_common_gps.yaml`, `config/costmap_local_gps.yaml`, `config/costmap_global_gps.yaml`, `config/planner_gps.yaml`, `launch/move_base_gps.launch`.

## Per-family footprint radii (constants, metres)

Conservative discs, chosen ≥ the visible trunk/object radius so the grid never under-marks. These are footprint radii BEFORE costmap inflation.

- `arbolpartes4*` trunk: **0.30 m**
- `tree_8*` trunk: **0.45 m** (visibly larger trees)
- `bench*`: **0.9 m** (benches are long; a disc is a conservative over-approximation — acceptable at 0.15 m grid, refine to a box only if it blocks a real route)
- `garden_table*`: **0.6 m**
- `lamp*`: **0.20 m**
- `trash_bin_1*`: **0.25 m**

## File Structure

- `map_tools/__init__.py` — package marker.
- `map_tools/sdf_parse.py` — pure SDF reading: model list, per-model world pose, per-family trunk/link pose. No grid logic.
- `map_tools/occupancy_grid.py` — a `Grid` class: world↔cell transforms, disc stamping, PGM/YAML writers. No SDF logic.
- `map_tools/extract_park_map.py` — CLI orchestrator: reads world via `sdf_parse`, classifies models, stamps footprints into a `Grid`, writes `park_map.pgm`/`.yaml` + `park_places.yaml`.
- `tests/map_tools/test_sdf_parse.py`, `test_occupancy_grid.py`, `test_extract_park_map.py` — unit tests.
- `maps/park_map.pgm`, `maps/park_map.yaml`, `maps/park_places.yaml` — generated artifacts (committed).
- `config/costmap_global_gps_map.yaml` — NEW global costmap config (static map, not rolling).
- `launch/move_base_gps_map.launch` — NEW launch (mirrors `move_base_gps.launch` + `map_server`).
- `operator/gcs_commands.py` — MODIFY: add `goal xy` and `goal <name>` parsing.
- `operator/operate.py` — MODIFY: add `_do_goal_xy()` + named-goal lookup dispatch.
- `operator/places.py` — NEW: load `park_places.yaml`, resolve name→(x,y) with free-cell offset.

---

## Task 1: SDF parsing — model list and world poses

**Files:**
- Create: `map_tools/__init__.py`
- Create: `map_tools/sdf_parse.py`
- Test: `tests/map_tools/__init__.py`, `tests/map_tools/test_sdf_parse.py`

**Interfaces:**
- Consumes: nothing.
- Produces:
  - `parse_models(world_path: str) -> list[Model]` where `Model` is a dataclass `Model(name: str, family: str, world_x: float, world_y: float)`. `family` is one of `"arbolpartes4"`, `"tree_8"`, `"bench"`, `"garden_table"`, `"lamp"`, `"trash_bin_1"`, or `"skip"`. `world_x/world_y` are the map-frame position of the obstacle (trunk for trees).
  - `classify(name: str) -> str` returning the family string.

- [ ] **Step 1: Write the failing test for classify**

```python
# tests/map_tools/test_sdf_parse.py
from map_tools.sdf_parse import classify

def test_classify_families():
    assert classify("arbolpartes4") == "arbolpartes4"
    assert classify("arbolpartes4_clone_10") == "arbolpartes4"
    assert classify("tree_8") == "tree_8"
    assert classify("tree_8_clone_2_clone_7_clone_1") == "tree_8"
    assert classify("bench_clone_0_clone_clone") == "bench"
    assert classify("garden_table") == "garden_table"
    assert classify("lamp") == "lamp"
    assert classify("trash_bin_1") == "trash_bin_1"
    assert classify("parque") == "skip"
    assert classify("camino_parque") == "skip"
    assert classify("Untitled2") == "skip"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_sdf_parse.py::test_classify_families -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'map_tools.sdf_parse'`

- [ ] **Step 3: Write classify + package markers**

```python
# map_tools/__init__.py
```
(empty file)

```python
# tests/map_tools/__init__.py
```
(empty file)

```python
# map_tools/sdf_parse.py
"""Read obstacle positions from a Gazebo SDF world file (park.world).

Reads the <state world_name=...> snapshot for POSES and the model-name prefix
for CLASSIFICATION. Trees: the trunk is link_0 (verified in-sim 2026-08-09) --
for the multi-link arbolpartes4 that link is offset ~1 m from the model <pose>,
so we read link_0's pose, not the model pose. tree_8 is single-link so link_0
== model pose; reading link_0 is correct for both.
"""
import re
from dataclasses import dataclass

# Ordered longest-prefix-first so "trash_bin_1" is not shadowed, etc.
_FAMILY_PREFIXES = (
    "arbolpartes4",
    "tree_8",
    "garden_table",
    "trash_bin_1",
    "bench",
    "lamp",
)


def classify(name):
    for fam in _FAMILY_PREFIXES:
        if name == fam or name.startswith(fam + "_"):
            return fam
    return "skip"


@dataclass
class Model:
    name: str
    family: str
    world_x: float
    world_y: float
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_sdf_parse.py::test_classify_families -v`
Expected: PASS

- [ ] **Step 5: Write the failing test for parse_models against the real world file**

```python
# add to tests/map_tools/test_sdf_parse.py
import os
from map_tools.sdf_parse import parse_models

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment",
                     "worlds", "park.world")

def test_parse_models_counts():
    models = parse_models(WORLD)
    fams = [m.family for m in models]
    assert fams.count("arbolpartes4") == 15
    assert fams.count("tree_8") == 23
    assert "skip" not in fams  # skipped models are dropped, not returned

def test_arbolpartes4_trunk_is_offset_from_model_pose():
    # The first arbolpartes4 model <pose> in the <state> block is
    # (36.8181, -20.8082); its link_0 (trunk) pose is (36.9169, -19.6925).
    # parse_models MUST return the LINK pose, not the model pose.
    models = parse_models(WORLD)
    tree = next(m for m in models if m.name == "arbolpartes4")
    assert abs(tree.world_x - 36.9169) < 0.01
    assert abs(tree.world_y - 19.6925) < 0.5  # y magnitude; sign per frame
    # Guard the bug: it must NOT be the model pose y = -20.8082
    assert abs(tree.world_y - (-20.8082)) > 0.5
```

- [ ] **Step 6: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_sdf_parse.py -k parse -v`
Expected: FAIL with `AttributeError`/`ImportError` (parse_models not defined)

- [ ] **Step 7: Implement parse_models**

```python
# add to map_tools/sdf_parse.py

# The <state world_name='default'> block holds each model's runtime pose and,
# nested, each link's pose. We parse THAT block. For arbolpartes4 we need
# link_0's pose (the trunk); for every other family the model pose == the link
# pose so either works and we use link_0 uniformly.
_MODEL_RE = re.compile(r"<model name='([^']+)'>")
_LINK_RE = re.compile(r"<link name='([^']+)'>")
_POSE_RE = re.compile(
    r"<pose[^>]*>\s*([-\d.eE]+)\s+([-\d.eE]+)\s+[-\d.eE]+\s+"
    r"[-\d.eE]+\s+[-\d.eE]+\s+[-\d.eE]+\s*</pose>")


def _state_block(text):
    start = text.index("<state world_name=")
    end = text.index("</state>", start)
    return text[start:end]


def parse_models(world_path):
    """Return the obstacle Models (family != 'skip') from park.world.

    Position is the trunk/link_0 world pose from the <state> block.
    """
    with open(world_path, "r") as fh:
        text = fh.read()
    state = _state_block(text)

    models = []
    # Split the state block into per-model chunks.
    idxs = [m.start() for m in _MODEL_RE.finditer(state)]
    idxs.append(len(state))
    for i in range(len(idxs) - 1):
        chunk = state[idxs[i]:idxs[i + 1]]
        name = _MODEL_RE.search(chunk).group(1)
        family = classify(name)
        if family == "skip":
            continue
        # Find link_0's pose within this model chunk.
        link_x = link_y = None
        for lm in _LINK_RE.finditer(chunk):
            if lm.group(1) == "link_0":
                after = chunk[lm.end():]
                pm = _POSE_RE.search(after)
                if pm:
                    link_x = float(pm.group(1))
                    link_y = float(pm.group(2))
                break
        if link_x is None:
            # Fallback: model pose (first pose in the chunk).
            pm = _POSE_RE.search(chunk)
            link_x = float(pm.group(1))
            link_y = float(pm.group(2))
        models.append(Model(name, family, link_x, link_y))
    return models
```

- [ ] **Step 8: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_sdf_parse.py -v`
Expected: PASS (all three tests). If the trunk-offset test fails, the parser is reading model pose not link_0 — fix before proceeding; this is the core correctness guard.

- [ ] **Step 9: Commit**

```bash
git add map_tools/__init__.py map_tools/sdf_parse.py tests/map_tools/
git commit -m "feat(map): SDF parser for park.world obstacle poses (trunk=link_0)"
```

---

## Task 2: Occupancy grid — transforms, disc stamping, PGM/YAML output

**Files:**
- Create: `map_tools/occupancy_grid.py`
- Test: `tests/map_tools/test_occupancy_grid.py`

**Interfaces:**
- Consumes: nothing (pure geometry/IO).
- Produces:
  - `class Grid(min_x, min_y, max_x, max_y, resolution)` with attrs `resolution`, `origin_x`, `origin_y`, `width`, `height`.
  - `Grid.world_to_cell(x, y) -> (col, row)`
  - `Grid.stamp_disc(x, y, radius) -> None` (marks cells within `radius` of world point occupied)
  - `Grid.is_occupied(x, y) -> bool`
  - `Grid.write_pgm(path) -> None` and `Grid.write_yaml(path, image_name) -> None` (ROS map_server format)

- [ ] **Step 1: Write the failing test for transforms and stamping**

```python
# tests/map_tools/test_occupancy_grid.py
from map_tools.occupancy_grid import Grid

def test_grid_dimensions_and_origin():
    g = Grid(min_x=-10.0, min_y=-10.0, max_x=10.0, max_y=10.0, resolution=0.5)
    assert g.width == 40
    assert g.height == 40
    assert g.origin_x == -10.0
    assert g.origin_y == -10.0

def test_stamp_disc_marks_center_and_within_radius():
    g = Grid(-5, -5, 5, 5, 0.5)
    g.stamp_disc(0.0, 0.0, 1.0)
    assert g.is_occupied(0.0, 0.0) is True
    assert g.is_occupied(0.8, 0.0) is True     # within radius
    assert g.is_occupied(3.0, 0.0) is False    # outside radius

def test_free_by_default():
    g = Grid(-5, -5, 5, 5, 0.5)
    assert g.is_occupied(2.0, 2.0) is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_occupancy_grid.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'map_tools.occupancy_grid'`

- [ ] **Step 3: Implement the Grid class**

```python
# map_tools/occupancy_grid.py
"""Occupancy grid with world<->cell transforms, disc stamping, and ROS
map_server PGM/YAML output. Pure geometry + IO; no SDF knowledge.

ROS map_server convention: PGM pixel value 0 = occupied (black), 254 = free
(white), 205 = unknown. Row 0 of the PGM is the TOP of the image, which maps to
the HIGHEST y in the world (origin is the bottom-left corner). We store cells
with row 0 = lowest y and flip vertically on write.
"""
import math


class Grid:
    OCC = 0
    FREE = 254

    def __init__(self, min_x, min_y, max_x, max_y, resolution):
        self.resolution = resolution
        self.origin_x = min_x
        self.origin_y = min_y
        self.width = int(math.ceil((max_x - min_x) / resolution))
        self.height = int(math.ceil((max_y - min_y) / resolution))
        # Row-major, row 0 = lowest y. Default free.
        self._cells = bytearray([self.FREE]) * (self.width * self.height)

    def world_to_cell(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return col, row

    def _in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def _set_occ(self, col, row):
        if self._in_bounds(col, row):
            self._cells[row * self.width + col] = self.OCC

    def is_occupied(self, x, y):
        col, row = self.world_to_cell(x, y)
        if not self._in_bounds(col, row):
            return False
        return self._cells[row * self.width + col] == self.OCC

    def stamp_disc(self, x, y, radius):
        r_cells = int(math.ceil(radius / self.resolution))
        c0, r0 = self.world_to_cell(x, y)
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                # Cell center distance check in metres.
                cx = self.origin_x + (c0 + dc + 0.5) * self.resolution
                cy = self.origin_y + (r0 + dr + 0.5) * self.resolution
                if math.hypot(cx - x, cy - y) <= radius:
                    self._set_occ(c0 + dc, r0 + dr)

    def write_pgm(self, path):
        # Flip vertically: PGM row 0 = top = highest y.
        with open(path, "wb") as fh:
            fh.write(b"P5\n%d %d\n255\n" % (self.width, self.height))
            for row in range(self.height - 1, -1, -1):
                fh.write(bytes(self._cells[row * self.width:(row + 1) * self.width]))

    def write_yaml(self, path, image_name):
        with open(path, "w") as fh:
            fh.write("image: %s\n" % image_name)
            fh.write("resolution: %.6f\n" % self.resolution)
            fh.write("origin: [%.6f, %.6f, 0.0]\n" % (self.origin_x, self.origin_y))
            fh.write("negate: 0\n")
            fh.write("occupied_thresh: 0.65\n")
            fh.write("free_thresh: 0.196\n")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_occupancy_grid.py -v`
Expected: PASS

- [ ] **Step 5: Write the failing test for PGM/YAML output**

```python
# add to tests/map_tools/test_occupancy_grid.py
def test_write_pgm_and_yaml(tmp_path):
    g = Grid(-5, -5, 5, 5, 0.5)
    g.stamp_disc(0.0, 0.0, 0.5)
    pgm = tmp_path / "m.pgm"
    yaml = tmp_path / "m.yaml"
    g.write_pgm(str(pgm))
    g.write_yaml(str(yaml), "m.pgm")
    header = pgm.read_bytes()[:15]
    assert header.startswith(b"P5\n20 20\n255\n")
    txt = yaml.read_text()
    assert "resolution: 0.500000" in txt
    assert "origin: [-5.000000, -5.000000, 0.0]" in txt
    assert "image: m.pgm" in txt
```

- [ ] **Step 6: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_occupancy_grid.py::test_write_pgm_and_yaml -v`
Expected: PASS

- [ ] **Step 7: Commit**

```bash
git add map_tools/occupancy_grid.py tests/map_tools/test_occupancy_grid.py
git commit -m "feat(map): occupancy grid with disc stamping + ROS PGM/YAML output"
```

---

## Task 3: Extractor CLI — park.world → map + places

**Files:**
- Create: `map_tools/extract_park_map.py`
- Create (generated): `maps/park_map.pgm`, `maps/park_map.yaml`, `maps/park_places.yaml`
- Test: `tests/map_tools/test_extract_park_map.py`

**Interfaces:**
- Consumes: `map_tools.sdf_parse.parse_models`, `map_tools.occupancy_grid.Grid`.
- Produces:
  - `RADII: dict[str, float]` (family → footprint radius, from the plan's constants).
  - `build_grid(models: list[Model], resolution=0.15, margin=5.0) -> Grid`
  - `build_places(models) -> dict[str, dict]` → `{name: {"x": float, "y": float}}` for named-place families (bench, garden_table, lamp, trash_bin_1; NOT trees).
  - `main(argv)` CLI writing the three files under `maps/`.

- [ ] **Step 1: Write the failing test for RADII and build_grid**

```python
# tests/map_tools/test_extract_park_map.py
import os
from map_tools.sdf_parse import parse_models, Model
from map_tools.extract_park_map import RADII, build_grid, build_places

WORLD = os.path.join(os.path.dirname(__file__), "..", "..",
                     "natural_environments_ros_opt", "natural_enviroment",
                     "worlds", "park.world")

def test_radii_cover_all_obstacle_families():
    for fam in ("arbolpartes4", "tree_8", "bench", "garden_table",
                "lamp", "trash_bin_1"):
        assert fam in RADII and RADII[fam] > 0

def test_build_grid_marks_a_known_tree_and_leaves_far_ground_free():
    models = parse_models(WORLD)
    g = build_grid(models, resolution=0.15, margin=5.0)
    tree = next(m for m in models if m.name == "arbolpartes4")
    assert g.is_occupied(tree.world_x, tree.world_y) is True
    # A point 50 m away from any obstacle is free. Use grid corner-ish empty.
    assert g.is_occupied(tree.world_x + 40.0, tree.world_y + 40.0) is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_extract_park_map.py -k "radii or build_grid" -v`
Expected: FAIL (`ModuleNotFoundError` for `extract_park_map`)

- [ ] **Step 3: Implement RADII, build_grid, build_places**

```python
# map_tools/extract_park_map.py
"""Offline: park.world -> ROS occupancy grid (park_map.pgm/.yaml) + a named
places table (park_places.yaml). Run once, or whenever park.world changes.

No simulator, no ground truth: reads the static world file only. Footprints use
per-family constant radii (all obstacle collisions in park.world are meshes, so
there is no SDF primitive radius to read). The costmap inflates these later --
we emit RAW footprints, no pre-inflation.
"""
import os
import sys

from map_tools.sdf_parse import parse_models
from map_tools.occupancy_grid import Grid

# Footprint radii in metres, BEFORE costmap inflation. See the plan for rationale.
RADII = {
    "arbolpartes4": 0.30,
    "tree_8": 0.45,
    "bench": 0.90,
    "garden_table": 0.60,
    "lamp": 0.20,
    "trash_bin_1": 0.25,
}

# Families that become named goal destinations (not trees).
PLACE_FAMILIES = ("bench", "garden_table", "lamp", "trash_bin_1")


def build_grid(models, resolution=0.15, margin=5.0):
    xs = [m.world_x for m in models]
    ys = [m.world_y for m in models]
    g = Grid(min(xs) - margin, min(ys) - margin,
             max(xs) + margin, max(ys) + margin, resolution)
    for m in models:
        g.stamp_disc(m.world_x, m.world_y, RADII[m.family])
    return g


def build_places(models):
    places = {}
    for m in models:
        if m.family in PLACE_FAMILIES:
            places[m.name] = {"x": round(m.world_x, 3), "y": round(m.world_y, 3)}
    return places
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_extract_park_map.py -k "radii or build_grid" -v`
Expected: PASS

- [ ] **Step 5: Write the failing test for build_places and main**

```python
# add to tests/map_tools/test_extract_park_map.py
def test_build_places_has_named_furniture_not_trees():
    models = parse_models(WORLD)
    places = build_places(models)
    assert any(n.startswith("bench") for n in places)
    assert any(n.startswith("lamp") for n in places)
    assert not any(n.startswith("arbolpartes4") for n in places)
    assert not any(n.startswith("tree_8") for n in places)
    # Each entry has numeric x and y.
    sample = next(iter(places.values()))
    assert isinstance(sample["x"], float) and isinstance(sample["y"], float)

def test_main_writes_three_files(tmp_path):
    from map_tools.extract_park_map import main
    out = tmp_path / "maps"
    rc = main(["--world", WORLD, "--out-dir", str(out)])
    assert rc == 0
    assert (out / "park_map.pgm").exists()
    assert (out / "park_map.yaml").exists()
    assert (out / "park_places.yaml").exists()
```

- [ ] **Step 6: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/test_extract_park_map.py -k "places or main" -v`
Expected: FAIL (`main` not defined)

- [ ] **Step 7: Implement main**

```python
# add to map_tools/extract_park_map.py
import argparse


def _write_places_yaml(places, path):
    with open(path, "w") as fh:
        fh.write("# Named goal destinations, map-frame metres. Generated by "
                 "extract_park_map.py.\n")
        for name in sorted(places):  # sorted only for stable file diffs (not display)
            p = places[name]
            fh.write("%s: {x: %.3f, y: %.3f}\n" % (name, p["x"], p["y"]))


def main(argv=None):
    ap = argparse.ArgumentParser(description="Extract park map + places from park.world")
    ap.add_argument("--world", default=os.path.join(
        os.path.dirname(__file__), "..", "natural_environments_ros_opt",
        "natural_enviroment", "worlds", "park.world"))
    ap.add_argument("--out-dir", default=os.path.join(
        os.path.dirname(__file__), "..", "maps"))
    ap.add_argument("--resolution", type=float, default=0.15)
    args = ap.parse_args(argv)

    os.makedirs(args.out_dir, exist_ok=True)
    models = parse_models(args.world)
    grid = build_grid(models, resolution=args.resolution)
    places = build_places(models)

    grid.write_pgm(os.path.join(args.out_dir, "park_map.pgm"))
    grid.write_yaml(os.path.join(args.out_dir, "park_map.yaml"), "park_map.pgm")
    _write_places_yaml(places, os.path.join(args.out_dir, "park_places.yaml"))
    print("wrote park_map.pgm/.yaml (%dx%d @ %.2f m) and park_places.yaml (%d places)"
          % (grid.width, grid.height, args.resolution, len(places)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 8: Run all extractor tests**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/map_tools/ -v`
Expected: PASS (all tasks 1–3 tests)

- [ ] **Step 9: Generate the real map artifacts and sanity-check**

Run: `cd /home/thinh/Documents/Husky_viz && python -m map_tools.extract_park_map`
Expected: prints grid dims + place count (≥ ~60 places from bench/table/lamp/bin). Then verify the YAML:
Run: `head -3 maps/park_map.yaml && wc -l maps/park_places.yaml`
Expected: `image: park_map.pgm`, `resolution: 0.150000`, and a nonzero place count.

- [ ] **Step 10: Commit**

```bash
git add map_tools/extract_park_map.py tests/map_tools/test_extract_park_map.py maps/park_map.pgm maps/park_map.yaml maps/park_places.yaml
git commit -m "feat(map): extractor CLI + generated park_map + park_places"
```

---

## Task 4: Static-map costmap config + launch

**Files:**
- Create: `config/costmap_global_gps_map.yaml`
- Create: `launch/move_base_gps_map.launch`
- Test: manual/in-sim (no pytest — this is ROS config). Verification steps below are the "test".

**Interfaces:**
- Consumes: `maps/park_map.yaml` (Task 3), existing `config/costmap_common_gps.yaml`, `config/costmap_local_gps.yaml`, `config/planner_gps.yaml`.
- Produces: a running `move_base` whose global costmap has a `StaticLayer` (park map) UNDER the `ObstacleLayer` (lidar), in the `map` frame.

- [ ] **Step 1: Create the static global costmap config**

```yaml
# config/costmap_global_gps_map.yaml
# Global costmap for the GPS-anchored, MAP-BACKED move_base
# (launch/move_base_gps_map.launch). Unlike costmap_global_gps.yaml (mapless,
# rolling window), this loads the preloaded park map as a StaticLayer and plans
# over the WHOLE park at once. Layer order is the additive-lidar rule:
#   static (preloaded trees) -> obstacles (live lidar, ADDS) -> inflation.
# Lidar can only ADD obstacles on top of the static map, never clear it.
global_frame: map
robot_base_frame: base_link

static_map: true
rolling_window: false
# track_unknown_space off: with a full static map the world is known free/occupied,
# not "unknown". (Contrast the mapless config, which tracks unknown space.)
track_unknown_space: false

resolution: 0.15

plugins:
  - {name: static,     type: "costmap_2d::StaticLayer"}
  - {name: obstacles,  type: "costmap_2d::ObstacleLayer"}
  - {name: inflation,  type: "costmap_2d::InflationLayer"}

static:
  map_topic: /map
  subscribe_to_updates: false
```

- [ ] **Step 2: Create the launch file**

```xml
<!-- launch/move_base_gps_map.launch -->
<?xml version="1.0"?>
<!--
  GPS-anchored, obstacle-aware move_base WITH a preloaded static park map.
  Identical to move_base_gps.launch except:
    (1) a map_server node publishes /map from maps/park_map.yaml, and
    (2) the global costmap is costmap_global_gps_map.yaml (StaticLayer +
        ObstacleLayer + Inflation, static_map true, not rolling).
  The robot, dual-EKF, navsat_transform and TF chain (map->odom->base_link) are
  ASSUMED ALREADY RUNNING (started by load-park-world.sh), same as
  move_base_gps.launch. Absolute config paths per repo convention.
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

</launch>
```

- [ ] **Step 3: Verify NavfnROS allow_unknown is still valid with a static map**

Read `config/planner_gps.yaml`: `NavfnROS.allow_unknown: true`. With `track_unknown_space: false` the static costmap marks everything known, so `allow_unknown` is harmless (nothing is unknown). No change needed. Note this in the launch comment if not already clear.

- [ ] **Step 4: In-sim smoke test — map loads and registers**

Start the sim, then this launch, per RUN-GOAL-HIJACK.md env:
```bash
# Terminal 1
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
./load-park-world.sh
# Terminal 2 (after robot is up)
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
roslaunch launch/move_base_gps_map.launch
```
Verify:
```bash
rostopic echo -n1 /map | head -20                       # OccupancyGrid published
rosparam get /move_base/global_costmap/static_map        # true
rosparam get /move_base/global_costmap/plugins           # static, obstacles, inflation
```
Expected: `/map` has data; static_map true; three plugins listed with `static` first.

- [ ] **Step 5: In-sim registration check (no ground truth)**

With RViz (`config/husky_planning.rviz`, fixed frame `map`), confirm the static-map tree blobs and the live lidar ObstacleLayer marks OVERLAP for a tree near the robot (they should coincide, proving the map-frame anchor). Judge visually in RViz; do NOT compare against `/gazebo/model_states`.

- [ ] **Step 6: Commit**

```bash
git add config/costmap_global_gps_map.yaml launch/move_base_gps_map.launch
git commit -m "feat(nav): static park map as StaticLayer under lidar in move_base_gps_map"
```

---

## Task 5: Named-place resolver

**Files:**
- Create: `operator/places.py`
- Test: `tests/operator/test_places.py`

**Interfaces:**
- Consumes: `maps/park_places.yaml` (Task 3).
- Produces:
  - `load_places(path) -> dict[str, tuple[float, float]]`
  - `resolve(name, places, offset=1.2) -> tuple[float, float]` — returns a goal point offset `offset` metres from the named object toward the map origin direction of travel is not known, so offset toward +x by default (a free-cell offset so the goal is beside, not inside, the obstacle). Raises `KeyError` with available names if `name` unknown.

- [ ] **Step 1: Write the failing test**

```python
# tests/operator/test_places.py
import os
import pytest
from operator_pkg_shim import load_places, resolve  # see Step 3 for import note

def test_load_and_resolve(tmp_path):
    y = tmp_path / "places.yaml"
    y.write_text("bench_1: {x: 10.0, y: 5.0}\nlamp_2: {x: -3.0, y: 2.0}\n")
    places = load_places(str(y))
    assert places["bench_1"] == (10.0, 5.0)
    gx, gy = resolve("bench_1", places, offset=1.0)
    # Offset by 1 m along +x from the object, so the goal is beside it.
    assert (gx, gy) == (11.0, 5.0)

def test_resolve_unknown_raises_with_names(tmp_path):
    places = {"bench_1": (1.0, 2.0)}
    with pytest.raises(KeyError) as exc:
        resolve("nope", places)
    assert "bench_1" in str(exc.value)
```

Import note: `operator` is a stdlib module name, so `import operator.places` is shadowed. The test imports via a shim. Create `tests/operator/__init__.py` and `tests/operator/operator_pkg_shim.py`:
```python
# tests/operator/operator_pkg_shim.py
import importlib.util, os
_spec = importlib.util.spec_from_file_location(
    "park_places_mod",
    os.path.join(os.path.dirname(__file__), "..", "..", "operator", "places.py"))
_m = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_m)
load_places = _m.load_places
resolve = _m.resolve
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/operator/test_places.py -v`
Expected: FAIL (`operator/places.py` missing → shim exec fails)

- [ ] **Step 3: Implement places.py**

```python
# operator/places.py
"""Load the named-places table (maps/park_places.yaml) and resolve a name to a
map-frame goal point offset just outside the object (so the goal is a free cell
beside it, not inside the obstacle the object also is in the costmap).

Deliberately NOT using pyyaml: the file is a flat 'name: {x: .., y: ..}' format
this parses directly, so operator containers need no extra dependency.
"""
import re

_LINE = re.compile(r"^([^:#\s]+):\s*\{x:\s*([-\d.]+),\s*y:\s*([-\d.]+)\}")


def load_places(path):
    places = {}
    with open(path, "r") as fh:
        for line in fh:
            m = _LINE.match(line.strip())
            if m:
                places[m.group(1)] = (float(m.group(2)), float(m.group(3)))
    return places


def resolve(name, places, offset=1.2):
    if name not in places:
        raise KeyError("unknown place '%s'; known: %s"
                       % (name, ", ".join(sorted(places))))
    x, y = places[name]
    # Offset along +x so the goal sits beside the object, not on it. The planner
    # + inflation handle final approach; this only needs to land in a free cell.
    return (x + offset, y)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/operator/test_places.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add operator/places.py tests/operator/
git commit -m "feat(operator): named-place resolver (park_places.yaml -> map xy)"
```

---

## Task 6: Operator prompt — `goal xy` and `goal <name>` verbs

**Files:**
- Modify: `operator/gcs_commands.py` (parse_command, around lines 4-18)
- Modify: `operator/operate.py` (_dispatch ~248, add `_do_goal_xy`)
- Test: `tests/operator/test_gcs_commands.py`

**Interfaces:**
- Consumes: `parse_command` (existing), `operator/places.py` (Task 5), existing `_do_goal(lat, lon)`.
- Produces: `parse_command` returns `("goal_xy", [x, y])` for `goal xy <x> <y>` and `("goal_name", [name])` for `goal <name>` (single non-numeric arg). Existing `("goal", [lat, lon])` unchanged.

- [ ] **Step 1: Write the failing test for parse_command**

```python
# tests/operator/test_gcs_commands.py
import importlib.util, os
_spec = importlib.util.spec_from_file_location(
    "gcs_commands_mod",
    os.path.join(os.path.dirname(__file__), "..", "..", "operator", "gcs_commands.py"))
_m = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_m)
parse_command = _m.parse_command

def test_goal_latlon_unchanged():
    assert parse_command("goal 49.9 8.9") == ("goal", [49.9, 8.9])

def test_goal_xy():
    assert parse_command("goal xy 12.5 -3.0") == ("goal_xy", [12.5, -3.0])

def test_goal_name():
    assert parse_command("goal bench_1") == ("goal_name", ["bench_1"])

def test_goal_xy_bad_args():
    cmd, args = parse_command("goal xy foo bar")
    assert cmd == "error"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/operator/test_gcs_commands.py -v`
Expected: FAIL (`goal xy 12.5 -3.0` currently returns `("error", ...)` because it expects 2 numeric args and gets "xy")

- [ ] **Step 3: Update parse_command**

Replace the `goal` branch in `operator/gcs_commands.py` (currently lines ~9-15) with:

```python
    if verb == "goal":
        rest = parts[1:]
        # goal xy <x> <y>  -- map-frame metres
        if rest and rest[0] == "xy":
            if len(rest) != 3:
                return ("error", ["goal xy needs <x> <y>"])
            try:
                return ("goal_xy", [float(rest[1]), float(rest[2])])
            except ValueError:
                return ("error", ["goal xy args must be numbers"])
        # goal <lat> <lon>  -- two numeric args (existing behaviour)
        if len(rest) == 2:
            try:
                return ("goal", [float(rest[0]), float(rest[1])])
            except ValueError:
                return ("error", ["goal args must be numbers"])
        # goal <name>  -- single non-numeric arg -> named place lookup
        if len(rest) == 1:
            return ("goal_name", [rest[0]])
        return ("error", ["goal needs <lat> <lon>, xy <x> <y>, or <name>"])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/operator/test_gcs_commands.py -v`
Expected: PASS

- [ ] **Step 5: Wire the new verbs into operate.py dispatch**

In `operator/operate.py`, add near the top (after existing imports):
```python
import os as _os
from places import load_places, resolve as resolve_place  # operator/ is on sys.path[0]

_PLACES_PATH = _os.path.join(_os.path.dirname(__file__), "..", "maps", "park_places.yaml")
```

In `_dispatch` (after the existing `if cmd == "goal":` branch, line ~251-252), add:
```python
        elif cmd == "goal_xy":
            self._do_goal_xy(args[0], args[1])
        elif cmd == "goal_name":
            try:
                places = load_places(_PLACES_PATH)
                gx, gy = resolve_place(args[0], places)
            except (KeyError, IOError) as exc:
                rospy.logwarn("named goal failed: %s", exc)
                return
            rospy.loginfo("named goal '%s' -> map (%.2f, %.2f)", args[0], gx, gy)
            self._do_goal_xy(gx, gy)
```

Add the `_do_goal_xy` method next to `_do_goal` (after line ~275). It is `_do_goal` with the `latlon_to_map` line removed — the input IS already map xy. The snippet below matches the REAL `_do_goal` API verified in source: frame constant `GOAL_FRAME`, quaternion orientation `gq` (identity here since a park goal has no required heading), and the real marker signature `place_goal_marker("goal_marker_real", gx, gy, "0 1 0", frame="map")`. Read `_do_goal` (operate.py ~275-305) and copy its exact tail; only skip the conversion:

```python
    def _do_goal_xy(self, gx, gy):
        """Send a goal already in map-frame metres (no lat/lon conversion).
        Identical to _do_goal from `self._goal_x = ...` onward; only the
        lat/lon -> map step is skipped because (gx, gy) is already map-frame."""
        self._goal_x, self._goal_y = gx, gy
        self.state.sent_goal = (gx, gy)
        gq = (0.0, 0.0, 0.0, 1.0)  # identity: no required heading at a park goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = GOAL_FRAME
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.orientation.x = gq[0]
        goal.target_pose.pose.orientation.y = gq[1]
        goal.target_pose.pose.orientation.z = gq[2]
        goal.target_pose.pose.orientation.w = gq[3]
        place_goal_marker("goal_marker_real", gx, gy, "0 1 0", frame="map")
        self.client.send_goal(goal)
        rospy.loginfo("sent map goal (%.2f, %.2f)", gx, gy)
```
This reuses `GOAL_FRAME`, `MoveBaseGoal`, `place_goal_marker` — all already imported/defined in operate.py (`place_goal_marker` at line 47, `GOAL_FRAME` is the module frame constant). No new imports beyond `places` (Step 5).

- [ ] **Step 6: Update the prompt help text**

In `operator/operate.py` `_print_help` (find it), add lines documenting `goal xy <x> <y>` and `goal <name>` alongside the existing `goal <lat> <lon>`.

- [ ] **Step 7: Re-run operator tests**

Run: `cd /home/thinh/Documents/Husky_viz && python -m pytest tests/operator/ -v`
Expected: PASS (places + gcs_commands)

- [ ] **Step 8: Commit**

```bash
git add operator/gcs_commands.py operator/operate.py tests/operator/test_gcs_commands.py
git commit -m "feat(operator): goal xy <x y> and goal <name> verbs (map-frame + place lookup)"
```

---

## Task 7: End-to-end in-sim validation + run doc

**Files:**
- Create: `RUN-MAP-NAV.md` (runbook for the map-based flow)
- Test: in-sim, judged by robot sensors + RViz (no ground truth).

- [ ] **Step 1: Plan-before-seeing test**

Bring up sim + `move_base_gps_map.launch` + operator (per RUN-GOAL-HIJACK.md env). From the `operator>` prompt send a goal on the far side of a known tree cluster the robot has NOT approached (e.g. `goal xy -15 -3`). In RViz watch the global plan: it should already bend around the mapped trees BEFORE lidar sees them (the static-map payoff). Robot follows; move_base reports SUCCEEDED. Record pass/fail from the RViz plan + arrival, not ground truth.

- [ ] **Step 2: Named-goal test**

From the prompt: `goal <one bench name from maps/park_places.yaml>` (e.g. `goal bench`). Confirm the log prints the resolved map xy, a marker appears, and the robot drives beside the bench and stops. Confirm via RViz/operator view.

- [ ] **Step 3: Additive-lidar re-plan test**

Place an obstacle NOT in the static map on the planned route (ask before using `/gazebo/set_model_state` to spawn/move one — repositioning is allowed with consent per CLAUDE.md; it is not a pose source). Confirm the global plan re-routes and DWA dodges. Judge via RViz.

- [ ] **Step 4: Write RUN-MAP-NAV.md**

Document the exact terminal sequence (mirror RUN-GOAL-HIJACK.md Steps 0-3, but Step 3 uses `move_base_gps_map.launch`), the three goal verbs, how to regenerate the map (`python -m map_tools.extract_park_map`), and the three validation checks above. Cross-reference RUN-GOAL-HIJACK.md for shared setup.

- [ ] **Step 5: Commit**

```bash
git add RUN-MAP-NAV.md
git commit -m "docs: RUN-MAP-NAV runbook for offline-map navigation"
```

---

## Self-Review notes (author)

- **Spec coverage:** extractor (§4.1) → Tasks 1-3; static-map wiring (§4.2) → Task 4; named-goal lookup (§4.3) → Tasks 5-6; testing (§6) → Tasks 1-7; two-tree-family handling → Task 1 + RADII in Task 3. All §7 open items resolved: (1) radii = constants; (2) map origin from obstacle spread + margin (Task 3 build_grid); (3) allow_unknown/track_unknown_space (Task 4 Steps 1,3); (4) named-goal wiring (Tasks 5-6) + free-cell offset (Task 5 resolve); (5) furniture footprints (RADII).
- **Frame/units:** everything map-frame metres; `goal xy` and named goals bypass lat/lon (Task 6). inflation_radius corrected to 0.5 (the real value in costmap_common_gps.yaml), not 0.35.
- **`operator` stdlib shadow:** handled via import shims in tests and `sys.path[0]` import in operate.py (operator/ is the script dir at runtime).
