# Terrain slope costmap layer

Date: 2026-08-18
Status: design, approved in chat, not implemented
Scope: **navigation only**. Terrain-for-localization is already built and is not touched.

## 1. Problem

`move_base` is terrain-blind. The global and local costmaps encode objects
(`<world>_map.pgm`) and live lidar returns, and nothing else. A slope steep enough to
bog or roll the Husky costs exactly as much to plan through as level ground.

The lake world has real relief; the park world does not. Measured from the extracted
DTMs (`maps/*_dtm.npy`), interior cells only, NaN coverage edges eroded:

| World | Relief | Max slope | p99 | p95 | p50 | >10° | >15° | >20° |
|---|---|---|---|---|---|---|---|---|
| park | 0.007 m | 0.9° | — | 0.1° | 0.03° | 0% | 0% | 0% |
| lake | 2.42 m | 22.1° | 17.1° | 13.2° | 3.6° | 14.1% | 2.4% | 0.24% |

Park's terrain mesh is scaled `z: 0.01` (a deliberate ~100x vertical flattening,
`maps/park_dtm.yaml:mesh_scale`), leaving a 7 mm plane. **Lake is the only terrain
testbed in scope.** Park's role is to prove the layer degrades to "uniformly free"
rather than breaking.

Note an earlier figure of 79° max slope for lake was a NaN-edge artifact of
differencing across the coverage boundary. The correct maximum is 22.1°.

## 2. Non-goals

Named explicitly because each was considered and rejected, not overlooked.

- **Not a localization change.** `landmark_loc/terrain_grid.py` and
  `terrain_match.py` are built, wired into `localizer_node.py:480-484`, and passing.
  Terrain-referenced localization has no standard ROS equivalent, so it stays
  first-party. This design does not touch it.
- **Not an attitude/TF fix.** The obstacle layer gates lidar on absolute z in a frame
  that reports the robot as level (§7). Correcting that means publishing real
  roll/pitch into TF, which changes every downstream consumer. Deliberately deferred;
  this design leaves the obstacle layer exactly as wrong as it is today, and no worse.
- **Not live elevation fusion.** The slope layer is derived from the static world
  file. Nothing at runtime senses terrain.
- **Not robot behaviour on slopes.** No speed moderation on descent, no rollover
  attitude limits. `dwa_local_planner` is 2D and has no notion of pitch. This design
  changes only where the robot is willing to plan, not how it drives once there.
- **Not `elevation_mapping` / `traversability_estimation`.** See §3.
- **Not a map regeneration.** `<world>_map.pgm` is untouched.

## 3. Package selection

Terrain-awareness is plumbing, not a contribution. The requirement is to use standard
packages and write as little as possible.

The mainstream ROS answer is the ANYbotics/ETH trio. Availability was verified on this
box against the Noetic apt repo:

| Package | Role | Noetic apt |
|---|---|---|
| `grid_map` | multi-layer 2.5D grid; filters; costmap conversion | **yes**, 1.6.4 |
| `elevation_mapping` | build elevation map from live sensing | **no** |
| `traversability_estimation` | slope/step/roughness → traversability | **no** |

The two unavailable packages are source-only and pull in `kindr` + `kindr_ros`, also
unreleased for Noetic. They are also the two least needed here: `elevation_mapping`
exists to *build* an elevation map, and this project already has a better one than it
would produce. `maps/<world>_dtm.npy` is rasterized offline from the world's
**collision** mesh by `map_tools/extract_dtm.py` — no simulator, no ground truth,
consistent with the standing "the map is known ground truth, never SLAM it" rule.
Running `elevation_mapping` would mean live-rebuilding, less accurately, a map already
held exactly.

**Decision: `grid_map` from apt, fed by the existing DTM.** Dry-run verified clean —
8 packages, 0 removals, 0 conflicts, no source builds:

```
ros-noetic-grid-map-core  -costmap-2d  -cv  -msgs  -ros  -filters
                          -rviz-plugin  -visualization        (all 1.6.4)
```

### Correction on filter names

`grid_map_filters` 1.6.4 ships **no `SlopeFilter`** — verified against
`filter_plugins.xml` in the package. Slope is obtained the way ETH's own
`traversability_estimation` and the upstream `grid_map_demos` chain do it:
`NormalVectorsFilter` → `MathExpressionFilter` computing `acos(normal_z)`. Two stock
filters instead of one; still pure YAML, still no math written here.

## 4. Architecture

One-way flow. No feedback, no runtime state.

```
maps/<world>_dtm.npy ──► DTM loader ──► grid_map "elevation" layer
                                              │
                        grid_map_filters chain (YAML, stock plugins)
                          NormalVectorsFilter   → normal_x/y/z
                          MathExpressionFilter  → slope = acos(normal_z)
                          MathExpressionFilter  → traversability (threshold + ramp)
                                              │
                        grid_map_costmap_2d ──► nav_msgs/OccupancyGrid (latched)
                                              │
                        move_base costmaps: an additional StaticLayer
```

Height is never cost. Absolute elevation is meaningless to a ground robot — 5.9 m is
no harder to drive than 3.5 m. Only **gradient** is. A hilltop plateau and a lakeside
flat both resolve to free; the shore drop between them resolves to lethal.

This matches §9 of `2026-08-16-terrain-grid-localization-design.md`, which specified
the traversability consumer as "a new additive costmap layer ... slotted into the
plugin list alongside the existing static → obstacles → inflation". `grid_map` now
supplies the slope computation and the costmap conversion instead of first-party code.

### Layer composition

```yaml
plugins:
  - {name: static,    type: "costmap_2d::StaticLayer"}     # <world>_map.pgm, objects
  - {name: slope,     type: "costmap_2d::StaticLayer"}     # NEW, terrain gradient
  - {name: obstacles, type: "costmap_2d::ObstacleLayer"}   # live lidar
  - {name: inflation, type: "costmap_2d::InflationLayer"}
```

`costmap_2d` runs each layer's `updateCosts` in sequence over the master grid;
`StaticLayer` maxes against what is already there. A cell is blocked if it holds an
object **or** exceeds the slope threshold — the union. Neither layer overwrites the
other.

**Both global and local costmaps** get the layer. The layer is static, so including it
twice costs nothing, and global-only would leave `dwa_local_planner` free to cut a
corner onto a bank while tracking a valid global path.

## 5. What is written vs configured

| Piece | Kind | Where |
|---|---|---|
| DTM → `grid_map` loader | **new, ~80 lines** | `map_tools/` |
| slope + traversability | **YAML only** | `config/` |
| DTM → costmap | **stock** | `grid_map_costmap_2d` |
| move_base wiring | **YAML only** | `config/costmap_*.yaml` |

The loader is the only real code: `np.load` the `.npy`, read `resolution` /
`origin_x` / `origin_y` from the sibling `.yaml`, populate a `GridMap` `elevation`
layer. NaN is preserved as NaN — `grid_map` is natively NaN-aware, and a fake zero
would fabricate a flat plateau over uncovered ground.

## 6. Thresholds

Grounded in the measured lake distribution (§1), not chosen freehand.

- **Lethal above 15°** — gates the steepest 2.4% of lake, including the shore drop.
- **Graded cost 8°–15°**, linear ramp — the planner prefers flat routes without being
  forbidden mild grade. 14.1% of lake lies above 10°, so the ramp meaningfully shapes
  routes rather than being decorative.
- Below 8° — free.

A Husky's practical limit is roughly 20–30°, so 15° is deliberately conservative.
Honest consequence: only 0.24% of lake exceeds 20° and nothing exceeds 25°, so a
threshold set at the true vehicle limit would gate almost nothing. The demo is
therefore about *route shaping over gradient*, not about a dramatic impassable wall.

Park computes to ~0° everywhere and the layer is uniformly free. That is correct
graceful degradation and doubles as the regression test.

## 7. Known defect this design does NOT fix

`config/costmap_common.yaml:32-33` gates lidar returns on **absolute z**
(`min_obstacle_height: 0.15`, `max_obstacle_height: 1.2`) in a frame where `base_link`
never tilts. On lake's slopes, over the 20 m `obstacle_range`, ground rises past
1.2 m uphill and reads as a solid wall, and falls below 0.15 m downhill so a real drop
reads as free space.

This is the same class of defect already recorded in project memory as "classifier
absolute-z breaks on terrain". `landmark_loc/derotate.py` documents the magnitude:
~17° pitch on the lake slope injects a **~6 m false height ramp at 20 m — larger than
the map's entire 2.4 m relief**.

Standard TF machinery does not fix this. `costmap_2d`'s `ObstacleLayer` already
transforms clouds via TF automatically; the error is in the TF tree's *contents*, not
in the transforming. The correct fix is publishing measured roll/pitch into TF so
every consumer de-rotates for free — the pieces exist
(`scripts/publish_terrain_frame.py` measures it, `two_d_mode` is unset so the EKF
already nominally estimates attitude) but are not in the load-bearing path.

Scope was deliberately kept tight. Recorded here so it is a known deferral, not a
surprise. The slope layer is unaffected: it is built from the static DTM and contains
no lidar at all.

## 8. Resolution alignment

The DTM is 0.25 m; `config/costmap_global_gps_map.yaml` runs at 0.15 m. The ratio is
non-integer, so `StaticLayer` resampling would smear a lethal slope cell across
neighbouring cells.

**Regenerate the DTM at 0.15 m** to match. `map_tools/extract_dtm.py` takes resolution
as a parameter, so this is a re-run, not a code change. Do it before wiring.

Two `StaticLayer` instances in one costmap both default to subscribing `/map`. The new
one needs its own `map_topic` and an explicit `use_maximum`, or it silently
half-works.

## 9. Multi-world support

`extract_dtm.py` holds a hardcoded `WORLDS` registry with a per-world `TerrainSpec`.
Its own docstring explains why this cannot be generic: which mesh is the terrain
differs per world by no consistent rule (park's visual and collision are the same
mesh; lake's collision `lago.dae` is a different, denser mesh than its visual). It
also hard-refuses rotated terrain models rather than silently mishandling them.

"Works with all maps" therefore means **one registry entry per world**, not
auto-discovery. Park and lake are both already registered. This is an accepted cost,
stated rather than hidden.

## 10. Testing

- **Unit** — loader round-trip (`.npy` → `GridMap` → identical values, NaN preserved,
  origin and resolution correct); slope from a synthetic plane of known gradient
  against the analytic value.
- **Park regression** — layer uniformly free; existing navigation behaviour unchanged.
- **Lake, RViz** — `grid_map_rviz_plugin` renders the slope layer over the terrain;
  confirm high cost follows the visible shore drop.
- **Lake, live** — a goal placed across the steep shore should route around it.

Judged by the robot's **actual position in Gazebo versus the goal marker**, per the
standing rule that `move_base` SUCCEEDED and the fused pose both lie. Sim runs are
executed from the main conversation off a clean kill, following `RUN-MAP-NAV.md` in
full.

## 11. Risks and honest limits

- **Thin margin.** Nothing in lake exceeds 25°, so the layer is validated only against
  mild-to-moderate grade. Behaviour at genuinely impassable slope is untested here.
- **The water is not modelled.** `lago` has no collision geometry and is deliberately
  absent from `lake_map.pgm`, so nothing stops the planner routing into the lake. The
  slope layer catches the *shore gradient*, which helps incidentally, but flat water
  beyond the drop reads as free. This is not a water hazard fix.
- **Static only.** Terrain the DTM does not know about is invisible to this layer.
- **`hill.world` and `forest.world`** reference meshes that currently exist only in
  `~/.local/share/Trash`, so neither world loads and neither has a DTM. Out of scope.

## 12. Open decisions

Both defaulted in this document; flagged for review.

1. **Local costmap inclusion** — defaulted to *yes, both global and local* (§4).
2. **DTM resolution** — defaulted to *regenerate at 0.15 m* (§8). The alternative is
   0.075 m, an exact 2x subdivision, at 4x the memory.
3. **Runtime node vs bake-ahead** — the filter chain is specified as a live node so it
   stays stock YAML and inspectable in RViz. Since the DTM is static, slope could
   instead be computed once offline and loaded by a plain `StaticLayer`, dropping the
   runtime node entirely. Simpler; less inspectable. Not yet decided.
