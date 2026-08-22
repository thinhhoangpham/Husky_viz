# DWA stalls with goal_cost = 40001 (unreachable local goal)

**RESOLVED 2026-08-22.** Two independent defects, both found and fixed. Jump to
"Resolution" at the end for what to change and how it was verified. The
investigation below is preserved as written on 2026-08-20 because its
measurements are sound, but **two of its conclusions were wrong** and are
corrected in place:

- The "unknown cells" hypothesis it could not confirm is DEAD: measured 0
  NO_INFORMATION cells in the local costmap across 516 samples, moving and
  stopped. `track_unknown_space` is unset on the local costmap, so untraced
  cells are FREE, not unknown.
- Its "central contradiction" (valid seed + unreachable cells) dissolves once
  you notice `setLocalGoal` only rejects NO_INFORMATION while `updatePathCell`
  also rejects LETHAL and INSCRIBED. A seed can be simultaneously VALID to the
  first and a WALL to the second. That is defect 1.

## The symptom

move_base drives normally for tens of metres, then stops dead and never recovers. Wheel
velocity goes to exactly 0.00 in a single control cycle (no deceleration, no oscillation).
move_base eventually aborts with status 4, "Failed to find a valid control. Even after
executing recovery behaviors." The robot is NOT physically stuck — it is in open,
obstacle-free terrain, and manual `cmd_vel` driving works (including uphill, confirmed in
an earlier session).

## The mechanism (confirmed end-to-end by measurement, not inference)

Chain: DWA's `goal_costs_` MapGrid wavefront never reaches the trajectory endpoints → every
trajectory scores as unreachable → 0 valid trajectories → zero velocity → recovery
behaviours fail → abort.

Evidence captured with DWA loggers at DEBUG (`rosservice call /move_base/set_logger_level`
on `ros.base_local_planner` and `ros.dwa_local_planner`, captured off `/rosout` — ROS_DEBUG
does NOT reach stdout):

- `Evaluated 300 trajectories, found 0 valid`
- `300 x "discarded by cost function 5 with cost: -2.000000"`
- Across the whole capture: 21,900 rejections, 100% gen_id 5; 73/73 scoring cycles found 0
  valid.

Critic registration order, read from `dwa_planner.cpp` lines 171-177:

| gen_id | Critic | Notes |
|---|---|---|
| 0 | oscillation | |
| 1 | obstacle | |
| 2 | goal_front | `setStopOnFailure(false)` — can never cause a rejection |
| 3 | alignment | `setStopOnFailure(false)` — can never cause a rejection |
| 4 | path | |
| 5 | goal | **the critic that rejected every trajectory in the capture** |
| 6 | twirling | |

Sentinel values, from `base_local_planner/map_grid.h`:

- `obstacleCosts()` = `map_.size()` = 40000 for a 200x200 grid — cell blocked by an obstacle
- `unreachableCellCosts()` = `map_.size() + 1` = 40001 — cell NEVER VISITED by the wavefront

And from `map_grid_cost_function.cpp` `scoreTrajectory`: returns -3.0 for the obstacle
sentinel, **-2.0 for the unreachable sentinel**, -4.0 for off-map. We observe -2.0, i.e.
never-visited, NOT obstacle-blocked. This distinction is the single most important fact in
this file.

> **CORRECTION 2026-08-22.** True but misleading as written. The cells scoring -2.0 are
> indeed never-visited, but the REASON they were never visited IS an obstacle: the one
> under the wavefront seed. Obstacles cause the never-visited state by blocking the flood
> at its origin. "Never-visited, NOT obstacle-blocked" reads as "obstacles are not
> involved", which sent this investigation away from the actual cause.

## The central contradiction (this is where the investigation stopped)

`setLocalGoal` was replicated exactly in a live instrumentation script and reports a VALID
SEED on every cycle right through the abort (seed index ~70, ~71 plan poses in bounds, none
on NO_INFORMATION). A valid seed means `computeTargetDistance` should flood the entire
reachable grid. Yet the cells the trajectories touch still hold 40001, the never-visited
value. Both cannot be true of the same grid — which suggests the MapGrid DWA actually scores
against is not the one being reconstructed from the published costmap. **Resolving this
contradiction is the next step.**

> **RESOLVED 2026-08-22 — and the grid was never the problem.** Both statements ARE true of
> the same grid. The instrumentation replicated `setLocalGoal`'s own acceptance rule, which
> rejects only NO_INFORMATION, so it correctly reported "VALID SEED" for a seed sitting on
> an INSCRIBED (253) cell. `updatePathCell` then refuses to expand from 253, so the flood
> visits ZERO cells and every cell keeps 40001. Measured at the stall: seed cell cost = 253,
> `flood reached_robot=False, cells_visited=0`. The check that was missing is whether the
> seed is LETHAL/INSCRIBED, not just unknown.

Note the exact `setLocalGoal` acceptance condition, because it is subtler than it looks: a
plan point counts only if `costmap.worldToMap(...)` succeeds AND
`costmap.getCost(map_x, map_y) != NO_INFORMATION`. It takes the LAST valid run of points and
`break`s at the first invalid point after one has been found.

## Eliminated hypotheses

Roughly a dozen hypotheses were tested and refuted before the DEBUG capture identified the
actual critic (5, goal). The ones below are the ones with recorded evidence.

| Hypothesis | How it was ruled out |
|---|---|
| Obstacles / inflation — **WRONGLY ELIMINATED, see below** | 0 rejections from the obstacle critic; local costmap histogram shows 0 cells of any nonzero cost within 3 m of the robot at the stall |
| Unknown (NO_INFORMATION) cells blocking the wavefront | All stall points sit in windows with 0.0% unknown, while spawn (42.2% unknown) drives away fine; the correlation runs backwards |
| `prune_plan: false` (seed left behind the robot) | Measured seed 68-70 poses AHEAD of the robot throughout |
| TF extrapolation errors / "Could not transform the global plan" | These are real and frequent but flat at 4-12 per 10 s across the ENTIRE run, including 40 m of healthy driving; they are not what changes at the stall |
| Goal z / robot z | Robot z matches ground height to the millimetre at the stall; terrain there varies 3 cm over 2 m |
| Location-specific terrain | Stalls occur at (24.9, 6.5), (-6.7, -6.5) and (-5.0, -15.4) on different headings |
| Velocity limits, slope, physics | Manual driving works, including uphill |

> **CORRECTION 2026-08-22 — "Obstacles / inflation" was eliminated on the wrong test.**
> Both facts in that row are true and both are irrelevant. The obstacle critic logging 0
> rejections is expected: the obstacle does its damage through the GOAL critic's wavefront,
> not through the obstacle critic. And measuring cost "within 3 m of the ROBOT" is the wrong
> place — the wavefront is seeded where the global plan LEAVES the local window, ~5 m out,
> and that is where the blocking cell was. Measure at the SEED, not at the robot.
>
> The "unknown cells" row stands, and was re-confirmed independently: 0 NO_INFORMATION cells
> in 516 samples. `track_unknown_space` is not set on the local costmap, so untraced cells
> are FREE (0), never 255 — unknown cannot block this wavefront at all.

## Run matrix

| Run | World | Distance | Elevation change | Result |
|---|---|---|---|---|
| lake uphill (x4 earlier runs) | lake | — | +0.84 m | ABORTED, stalled ~(24.9, 6.5) |
| lake uphill | lake | 92.5 m | +0.69 m | ABORTED at 41 m, stalled ~(-6.7, -6.5) |
| park | park | 90.6 m | ~0 | **SUCCEEDED**, 0.37 m final error |
| lake downhill (robot teleported to the map high point first) | lake | 58.3 m | -2.30 m | ABORTED at ~16 m, stalled ~(-5.0, -15.4) |

**Caveat:** park is a WEAK CONTROL. Park's DTM relief is 0.007 m (mesh z-scale 0.01) vs
lake's 2.422 m (mesh z-scale 4). Park is effectively a flat plane, so it never exercises
elevation change at all. Park succeeding does not prove the planner is healthy on non-flat
terrain.

A true downhill goal does not exist from the lake spawn — the robot spawns at the 3.4th
percentile of elevation (z=3.761, median 4.480), so only 3.4% of the map lies below it and
none of those cells have a clear footprint. The downhill run therefore required teleporting
the robot to the map's high point (11.12, -15.62, z=5.93) via `/gazebo/set_model_state`
(user-approved for repositioning between runs only — never as a pose source).

## Separate finding: terrain filter blind ring

`scripts/filter_cloud_above_terrain.py` keeps points 0.40-3.00 m above terrain. Near the
robot the ground sits only ~0.13 m below the sensor footprint, so nearby ground returns fall
under the 0.40 m floor and are dropped. Measured consequence: the filtered cloud
`/os0_cloud_node/points_above_terrain` contains ZERO points within ~4.4 m of the robot (0
pts in 0-2 m, 0 in 2-4 m, 326 in 4-6 m, 325 in 6-10 m). The local costmap therefore gets no
marking data at all inside a ~4.4 m ring.

This is a real defect worth fixing on its own merits, but it does NOT explain the stall — a
blind ring yields MISSING obstacles, not phantom ones, and the robot refuses to move through
space the costmap reports as entirely free. There is no radius parameter; the ring is a side
effect of the 0.40 m floor.

## How to reproduce

1. Run the full `RUN-MAP-NAV.md` Steps 0-3 on the lake world (all steps, no skipping — note
   that `DTM_WORLD` must be exported to match the world, see the Step 3 callout in
   `RUN-MAP-NAV.md`).
2. Confirm the robot is at spawn and both controllers read `( running )`.
3. Send a far goal (~90 m) into open terrain, e.g. (44.88, -4.62).
4. The robot drives ~40 m and stalls.

## Resolution (2026-08-22)

Two INDEPENDENT defects. Both were needed; neither alone is sufficient.

### Defect 1 — `setLocalGoal` seeds the wavefront on a cell `updatePathCell` treats as a wall

`setLocalGoal` accepts a seed if `worldToMap()` succeeds AND the cost is
`!= NO_INFORMATION` — it tests ONLY 255. `updatePathCell` refuses to expand through
`LETHAL_OBSTACLE` (254), `INSCRIBED_INFLATED_OBSTACLE` (253) **and** `NO_INFORMATION` (255).
So a 253 cell is simultaneously an acceptable SEED and an impassable WALL. When the global
plan exits the 10x10 m window through an obstacle's inflation band, the BFS is seeded on a
blocked cell, visits **0 cells**, every cell keeps `unreachableCellCosts()` (40001), all 300
trajectories score -2.0, and DWA commands exactly zero velocity.

Reproduced 3/3 on lake, stalling within ~1.3 m of the same spot, always with
`flood reached_robot=False, cells_visited=0, seed_blocked=True` and the seed cost read as
**253** directly from the saved costmap.

**Fix:** make the seed test agree with the expansion test. A shared
`isBlockedCost()` helper is now used by BOTH `setLocalGoal` and `updatePathCell` so they
cannot diverge again. Because the loop keeps the LAST valid point and breaks after, the seed
naturally walks back to the last usable point on the plan.

    ~/husky_overlay_ws/src/navigation/base_local_planner/src/map_grid.cpp

Vendored at tag **1.17.3**, matching the installed `ros-noetic-base-local-planner` exactly.
Only that one package is built. `libdwa_local_planner.so` links `libbase_local_planner.so`
dynamically, so DWA itself needs no rebuild.

**Deploying it — automatic, from the launch file:**

`launch/move_base_gps_map.launch` carries an `env` tag inside the move_base node:

    <env name="LD_LIBRARY_PATH"
         value="$(env HOME)/husky_overlay_ws/devel/lib:$(env LD_LIBRARY_PATH)"/>

That puts the patched build ahead of `/opt/ros/noetic/lib` on the loader path, scoped to
that node so nothing else on the machine is affected. No export is needed in any terminal.

Do NOT `source ~/husky_overlay_ws/devel/setup.bash` as an alternative — that clone carries
17 UNBUILT navigation packages which then shadow the working installed ones, and
`move_base`/`map_server` fail with "Cannot locate node of type". Only the library path is
wanted, which is why this is an env tag rather than a workspace overlay.

**Only `move_base_gps_map.launch` has the tag.** `move_base_landmark.launch`,
`move_base_gps.launch` and `move_base_park.launch` still load the STOCK library and remain
exposed to defect 1.

**NOT PORTABLE.** The patched `.so` lives outside this repo. On a machine with no
`~/husky_overlay_ws` build the path simply does not exist, the stock library loads, and the
bug returns SILENTLY. Verify against the LIVE process, not the environment — the library
loads lazily when DWA is constructed, so check after move_base is fully up:

    grep -o '[^ ]*libbase_local_planner.so' /proc/$(pgrep -f lib/move_base/move_base)/maps

It must print the `husky_overlay_ws` path.

A permanent, portable fix means either vendoring the patch into this repo with build
instructions, or upstreaming it to ros-planning/navigation.

### Defect 2 — the global plan routes through obstacles the global costmap cannot see

Fixing defect 1 stopped the total paralysis but the robot still ground against obstacles:
measured on lake, it thrashed 189 s in one 4.5 m stretch with 21 forward/reverse flips
(100% of all reversing in a 92 m drive happened there).

Cause: the global costmap was StaticLayer + InflationLayer only, so navfn planned straight
through anything absent from the static map — and vegetation is deliberately absent (163
instances excluded, or the map becomes unnavigable). The local planner was then asked to
resolve a plan running through solid geometry, which it cannot do.

**This was proven NOT to be a local-planner problem.** Swapping DWA for
`base_local_planner/TrajectoryPlannerROS` changed nothing: it also commanded zero velocity,
stopping 7 m short of a spawned box with 4.95 m clear ahead. Tuning was likewise disproved —
raising `occdist_scale` 0.4 -> 1.0 made it WORSE (189 s -> 270 s, 21 -> 27 flips), and the
known-working reference sim runs a FAR more extreme ratio (`path_distance_bias` 96 against
`occdist_scale` 0.02) while avoiding obstacles fine.

**Fix:** add an ObstacleLayer to the global costmap, between static and inflation.

    config/costmap_global_gps_map.yaml

    plugins:
      - {name: static,     type: "costmap_2d::StaticLayer"}
      - {name: obstacles,  type: "costmap_2d::ObstacleLayer"}     # <- added
      - {name: inflation,  type: "costmap_2d::InflationLayer"}

The layer instance MUST be named `obstacles` so it picks up the existing `obstacles:` block
already loaded from `costmap_common_gps.yaml`. No observation-source params are duplicated.

### This reverses commit ecb0634 — and why that is now safe

`ecb0634` removed this layer after measuring **2373 of 2760** global lethal cells (86%) came
from live lidar rather than the object map, swamping the static map. That failure did not
reproduce, measured over a full 92 m lake drive:

    global lethal over the run: 342 -> 364 -> 555 -> 547 -> 496 -> 605 -> 642 -> 470 -> 383
    final: global lethal 383 vs static map occupied 342  ->  ~41 cells (11%) from lidar

Marks rise near vegetation and **fall again** — raytrace clearing works, no runaway.

The likely reason it is safe now: `ecb0634` predates the terrain-relative cloud filter
(`7d4f7d7`) and the mark/clear source split (`9c27dae`). Back then the layer was fed raw
ground returns and marked terrain itself. It is now fed the FILTERED cloud for marking and
the RAW cloud for clearing, so the marks are mostly real objects.

**Watch for this regressing.** One long drive is not proof against accumulation. If global
lethal cells start climbing monotonically and dominating the static map, that is the
`ecb0634` symptom returning.

### Verification

| Test | Result |
|---|---|
| lake, goal (44.88, -4.62) — the 3/3 reproduction | **SUCCEEDED**, 0.283 m error, 0 stall latches |
| park, 1x1x1.5 m box spawned 8 m ahead in-path | **dodged**, SUCCEEDED, ~0.4 m error |
| box in global costmap (was 0 lethal / 0 inscribed / 1444 free) | **9 lethal, 49 inscribed** |

### Still open

- **The blind ring is real and unfixed** (see the section above), but it is NOT the stall.
  Measured directly: within 2 m of the robot **100% of raw returns are ground**, so nothing
  is being wrongly discarded — there is simply nothing else to see. It is sensor geometry
  (lidar 0.92 m up, +/-45 deg FOV: the -45/-30/-20 deg beams hit ground at 0.92/1.6/2.5 m),
  not a threshold bug. Lowering the 0.40 m floor would mark the ground as lethal, which is
  the bug that floor exists to prevent.
- **Neither fix is committed.** The overlay IS wired into `move_base_gps_map.launch`
  (env tag), but not into the other move_base launches.
- The `TrajectoryPlannerROS:` block in `config/planner_gps.yaml` remains INERT. Editing it
  looks like a change and does nothing — a trap that cost time in this investigation.
