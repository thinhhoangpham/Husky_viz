# Offline Map + Route Planning for the Park (Google-Maps-style) — Design

**Date:** 2026-08-09
**Status:** Approved design, ready for implementation planning
**Author:** brainstormed with the user

## 1. Goal

Give the park robot a **preloaded offline map** so it can plan a route around
obstacles *at the start of a run*, the way Google Maps plans a route from a map
before you drive — then follow it while live lidar refines the plan around
anything the map did not know about.

Today the robot has **no map**: `drive_to_point_gps.py` drives a hardcoded list
of world-frame (x, y) waypoints in straight lines, and the `move_base_gps`
navigation stack (see below) plans **mapless** — a rolling costmap window built
only from what the lidar has seen so far. Neither knows the park's obstacle
layout before it sees it. This project adds that prior knowledge.

## 2. Key discovery — most of the stack already exists

The move_base navigation stack described in `RUN-GOAL-HIJACK.md` is already
running and already does the hard parts of this feature:

- **`launch/move_base_gps.launch`** runs move_base with **both costmaps in the
  GPS-anchored `map` frame** (`config/costmap_global_gps.yaml:global_frame: map`),
  global planner `navfn/NavfnROS`, local planner `dwa_local_planner/DWAPlannerROS`,
  consuming the live Ouster `/os0_cloud_node/points` through an `ObstacleLayer`.
- The **map-frame anchoring is already solved**: the dataset robot's dual-EKF
  (`/ekf_localization_map` map->odom, `/ekf_localization` odom->base_link) plus
  `/navsat_transform` publishes the whole `map -> odom -> base_link` TF chain,
  GPS-anchored and drift-free. This is what a preloaded static map requires, and
  it exists today.
- **`config/husky_planning.rviz`** already visualizes the global plan bending
  around obstacles — the "Google Maps view".
- Goals already enter as GPS lat/lon converted to the `map` frame (operator
  prompt `goal <lat> <lon>`), and NavfnROS/DWA already route and dodge.

**The one thing missing is the preloaded map itself.** Both costmaps are
`static_map: false`, `rolling_window: true` — mapless. There is no
`park_map.pgm`, no `map_server`, no `StaticLayer`, and no named-places table.

Consequence for scope: we do **not** build a navigation stack or a planner. We
**extend the existing `move_base_gps` stack** with a static map, and add a small
map extractor and a named-goal lookup. Three additions, no re-architecture.

## 3. What we are NOT doing (rejected alternatives)

- **Not** writing a standalone A* planner or a new `nav_park_map.launch`
  independent of `move_base_gps.launch`. That would duplicate the map-frame
  move_base that already exists and risk two divergent stacks. (Considered and
  rejected during design.)
- **Not** building the map by SLAM / a lidar mapping run. The park layout is
  known ground-truth (`park.world`); we extract it, we do not build it.
- **Not** letting lidar *erase* map obstacles. Lidar is **additive** only (see
  §5.2). The preloaded map is authoritative.
- **Not** touching `drive_to_point_gps.py`, `move_base_park.launch`, the mapless
  demo, or any attack script. This feature is layered on; existing demos keep
  working unchanged.

## 4. Architecture — three new/changed pieces

```
park.world ──(extractor, offline, once)──> park_map.pgm + park_map.yaml
                                        └─> park_places.yaml (name -> x,y)

map_server(park_map.yaml) ──> /map (OccupancyGrid, map frame, latched)
                                        │
move_base global costmap: StaticLayer(/map) + ObstacleLayer(lidar) + Inflation
                                        │
goal (x y  OR  name->lookup) ──> move_base action ──> NavfnROS plan ──> DWA drive
```

### 4.1 Map extractor — `extract_park_map.py` (NEW, offline)

Run once (or whenever `park.world` changes). Parses the world file and writes
three files. No simulator involved; pure, deterministic, unit-testable.

**Resolution: 0.15 m/cell.** Chosen to **match the existing costmap resolution**
(`costmap_global_gps.yaml:resolution: 0.15`) so `/map` and the costmap share a
grid and never resample. The park is roughly 80–120 m across, giving a grid on
the order of 800x800 cells — trivial for NavfnROS. (The Husky is ~0.99 x 0.67 m,
so 0.15 m cells resolve sub-robot-width gaps.)

**Which obstacles (verified against `park.world`, 2026-08-09).** The world has
**two distinct tree models** plus furniture — the extractor must handle both tree
families, they are structured differently:

- **`arbolpartes4*` — 15 trees ("small").** Two links: `link_0` mesh
  `arbol4//tronco4.dae` (**trunk** — confirmed visually in-sim by recoloring),
  `link_1` mesh `arbol4/copa4.dae` (**canopy/leaves**). The trunk `link_0` pose
  is offset ~1 m from the model `<pose>` — this is the trap below.
- **`tree_8*` — 23 trees ("big").** **Single link** `link_0`, visual
  `bark8_lowpoly.obj` + collision `bark8_collision.obj`. Model `<pose>` and
  `link_0` pose share the same x/y (no offset), so either places it correctly.
- **Furniture (obstacles):** `bench*` (16), `garden_table*` (22), `lamp*` (30),
  `trash_bin_1*` (22). Box/disc footprints from their meshes.
- **Drivable (skipped):** ground `terreno_parque`/`parque`, trail `camino_parque`,
  and the container model `Untitled2` (verify it holds no real geometry).

Model counts come from the **model *definitions*** (SDF after line ~4765), which
carry the real collision geometry — NOT from the earlier `<state>` snapshot,
which lists many entries without geometry and inflated an earlier "~180 trees"
miscount. Actual trees: **15 + 23 = 38**.

**Footprints.** Trees rasterize as a disc (radius from the trunk collision mesh:
`tronco4.dae` for `arbolpartes4`, `bark8_collision.obj` for `tree_8`);
box-shaped objects as their bounding box. **No pre-inflation** — the extractor
emits raw footprints and lets the costmap's tuned `InflationLayer`
(`robot_radius: 0.55`, `inflation_radius: 0.35` in `costmap_common_gps.yaml`)
grow them. Pre-inflating here would double-inflate.

**The trunk-link / two-representation trap (RESOLVED for both families).**
`park.world` contains each model twice: a `<state world_name='default'>` snapshot
(line ~146+, poses only, no collision) and the real model *definitions*
(line ~4765+, with collision geometry). For `arbolpartes4` the model is
**multi-link** and its trunk `link_0` sits ~1 m from the model `<pose>`; reading
the **model pose** instead of the **trunk link pose** gave 1–5 m placement errors
and a jittery localizer in prior work (project memory). For `tree_8` (single
link) the offset is zero. The extractor MUST therefore:
  1. read collision geometry from the world model **definitions** (they have it),
  2. place each tree at its **`link_0`** world pose (trunk for both families) —
     confirmed `link_0` = trunk for `arbolpartes4` by in-sim recolor, and the
     only link for `tree_8`,
  3. have a regression test asserting a known `arbolpartes4` tree lands at its
     `link_0` (trunk) position, offset from the model pose.

**Outputs:**
- `park_map.pgm` + `park_map.yaml` — standard ROS `map_server` occupancy grid
  (free/occupied/unknown, resolution, origin). Standard format so `map_server`
  loads it and RViz displays it directly.
- `park_places.yaml` — `name -> {x, y}` for the named objects above, seeding the
  named-goal feature. Hand-added aliases/points may be appended later.

### 4.2 Static-map wiring — `move_base_gps_map.launch` + edited global costmap (NEW/CHANGED)

Add the preloaded map to the existing map-frame move_base:

- **New launch `move_base_gps_map.launch`** — mirrors `move_base_gps.launch`
  exactly, plus a `map_server` node loading `park_map.yaml` (publishes `/map`,
  latched, `map` frame). Kept as a separate launch so the existing mapless
  `move_base_gps.launch` demo continues to work untouched.
- **Global costmap changes** (a new `costmap_global_gps_map.yaml`, so the mapless
  one is preserved):
  - `static_map: false -> true`
  - `rolling_window: true -> false` (the global costmap now covers the whole
    park from the preloaded map, not a rolling window)
  - **layer stack becomes:**
    `StaticLayer(/map)` -> `ObstacleLayer(lidar)` -> `InflationLayer`.
    The `StaticLayer` (preloaded trees) is the baseline; the existing
    `ObstacleLayer` **adds** live lidar obstacles on top; inflation grows both.
    **This layer order IS the additive-lidar rule** — lidar can only add, never
    clear the static map.
  - `track_unknown_space` / NavfnROS `allow_unknown` reviewed for consistency:
    with a full static map the global costmap is no longer mostly-unknown, so
    the `allow_unknown` behavior tuned for the mapless case should be revisited
    (see §7).
- **Local costmap unchanged** — stays a short-range rolling window in the `map`
  frame off live lidar. Drift over its few-metre horizon is negligible and it
  must react to real, present obstacles, not the prior map.

### 4.3 Named-goal lookup (NEW, thin)

Coordinates-first with a name lookup layer:
- `goal <x> <y>` — send a map-frame point straight to move_base (existing path).
- `goal <name>` — look `<name>` up in `park_places.yaml`, get its (x, y).
- Because named objects are themselves obstacles in the grid, a named goal is
  **offset to a free cell just outside the object** (stand next to the bench,
  not on it), so move_base can actually reach it.
- Goals are sent through the **existing** move_base goal channel (the operator
  prompt / `SimpleActionClient`, `PoseStamped` in the `map` frame). Goal
  orientation defaults to "don't care" (approach-determined), overridable.
- Preferred surface: a new verb in the existing operator prompt (e.g.
  `goal bench`) rather than a separate script, reusing the goal machinery that
  already exists. Exact wiring point confirmed in planning (see §7).

## 5. Behavior

### 5.1 Plan at the start (the core feature)

On a goal, NavfnROS plans a global route over the **static map** immediately —
it routes around trees the robot **has not yet seen**, because the map already
knows they are there. This is the payoff over the current mapless stack, which
can only avoid obstacles after lidar observes them. This is the literal
"use the map to plan the path at the beginning".

### 5.2 Re-plan live (additive lidar)

While driving, the `ObstacleLayer` marks live lidar returns into the global
costmap on top of the static layer. When a newly-marked obstacle (one the map
did not have, or that moved) lands on the planned route, NavfnROS re-plans
around it and DWA dodges locally — this is the existing, tuned behavior
(`DWA reverse-escape` etc., commit `7ebb4d6`), now operating on top of a
static baseline instead of a blank one. Lidar never clears static-map cells.

## 6. Testing

Honors the project's **no-ground-truth** rule: results are judged by the robot's
own sensors and the operator's Gazebo/RViz view, never against
`/gazebo/model_states`.

- **Extractor (offline unit tests, fast, deterministic):**
  - Known trees mark blocked cells; ground/trail cells stay free.
  - **Trunk-link regression:** a known tree lands at its trunk link pose, not
    its model `<pose>` (guards the 1–5 m bug).
  - Reads collision geometry from world model definitions, not the `<state>`
    snapshot.
  - `park_places.yaml` contains the expected named objects with plausible coords.
- **Map registration (in sim):** with the robot beside a known tree, the
  static-map tree and the live lidar-marked tree **coincide** in the costmap
  (overlap, not offset). This proves the map-frame anchor and that the map is
  not drifting relative to the world.
- **Plan-before-seeing (in sim):** send a goal on the far side of a tree cluster
  the robot has not approached; verify the initial global plan already routes
  around it (the static-map payoff), the robot follows, and move_base reports
  `SUCCEEDED`.
- **Additive re-plan (in sim):** place an obstacle NOT in the static map on the
  planned route; verify the route re-plans around it (validates the existing DWA
  on top of the static baseline).

## 7. Open details to resolve during implementation planning

These are known unknowns, deliberately not guessed. (The trunk-link question —
formerly the top item — is now RESOLVED: `link_0` = trunk for both tree families,
confirmed by in-sim recolor; see §4.1.)

1. **Trunk collision radius per family.** How to read the footprint radius from
   each trunk mesh — `tronco4.dae` (`arbolpartes4`) and `bark8_collision.obj`
   (`tree_8`). The meshes are `.dae`/`.obj` files, so the extractor either parses
   mesh bounds or uses a per-family constant radius. Decide which.
2. **Map origin / extent.** Compute the `park_map.yaml` `origin` and grid bounds
   from the spread of obstacle positions so the whole park fits with margin, in
   the `map` frame (origin 49.9/8.9 datum, consistent with `fix_to_world`).
3. **`allow_unknown` / `track_unknown_space`** re-tuning now that the global
   costmap has a full static map rather than being mostly unknown.
4. **Named-goal wiring point** — confirm whether the lookup goes into the
   operator prompt, a small resolver script, or both; and the exact free-cell
   offset rule for a named (obstacle) destination.
5. **Furniture footprints** — box vs. disc per object type, and whether small
   items (`lamp`, `trash_bin_1`) are worth marking at 0.15 m resolution or are
   better left to live lidar.

## 8. Artifacts

- **New:** `extract_park_map.py`; `park_map.pgm` + `park_map.yaml`;
  `park_places.yaml`; `move_base_gps_map.launch`;
  `config/costmap_global_gps_map.yaml`; extractor tests; named-goal lookup
  (script and/or operator-prompt verb).
- **Changed:** possibly the operator prompt (add `goal <name>`); a run doc
  section for the map-based flow.
- **Untouched:** `drive_to_point_gps.py`, `move_base_gps.launch`,
  `move_base_park.launch`, `costmap_global_gps.yaml`, all attack scripts, the
  existing hijack/GPS-nav demos.
