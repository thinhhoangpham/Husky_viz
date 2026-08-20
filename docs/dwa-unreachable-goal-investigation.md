# DWA stalls with goal_cost = 40001 (unreachable local goal)

**UNRESOLVED — investigation paused 2026-08-20.**

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

## The central contradiction (this is where the investigation stopped)

`setLocalGoal` was replicated exactly in a live instrumentation script and reports a VALID
SEED on every cycle right through the abort (seed index ~70, ~71 plan poses in bounds, none
on NO_INFORMATION). A valid seed means `computeTargetDistance` should flood the entire
reachable grid. Yet the cells the trajectories touch still hold 40001, the never-visited
value. Both cannot be true of the same grid — which suggests the MapGrid DWA actually scores
against is not the one being reconstructed from the published costmap. **Resolving this
contradiction is the next step.**

Note the exact `setLocalGoal` acceptance condition, because it is subtler than it looks: a
plan point counts only if `costmap.worldToMap(...)` succeeds AND
`costmap.getCost(map_x, map_y) != NO_INFORMATION`. It takes the LAST valid run of points and
`break`s at the first invalid point after one has been found.

## Eliminated hypotheses

Roughly a dozen hypotheses were tested and refuted before the DEBUG capture identified the
actual critic (5, goal). The ones below are the ones with recorded evidence.

| Hypothesis | How it was ruled out |
|---|---|
| Obstacles / inflation | 0 rejections from the obstacle critic; local costmap histogram shows 0 cells of any nonzero cost within 3 m of the robot at the stall |
| Unknown (NO_INFORMATION) cells blocking the wavefront | All stall points sit in windows with 0.0% unknown, while spawn (42.2% unknown) drives away fine; the correlation runs backwards |
| `prune_plan: false` (seed left behind the robot) | Measured seed 68-70 poses AHEAD of the robot throughout |
| TF extrapolation errors / "Could not transform the global plan" | These are real and frequent but flat at 4-12 per 10 s across the ENTIRE run, including 40 m of healthy driving; they are not what changes at the stall |
| Goal z / robot z | Robot z matches ground height to the millimetre at the stall; terrain there varies 3 cm over 2 m |
| Location-specific terrain | Stalls occur at (24.9, 6.5), (-6.7, -6.5) and (-5.0, -15.4) on different headings |
| Velocity limits, slope, physics | Manual driving works, including uphill |

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

## Next steps (unverified — these are leads, not conclusions)

1. Enable `publish_cost_grid_pc` on the DWA block and read the real per-cell `goal_cost`
   from `/move_base/DWAPlannerROS/cost_cloud`. **Trap:** `publish_cost_grid_pc: true`
   currently sits in the `TrajectoryPlannerROS:` block of `config/planner_gps.yaml` (line
   61), which is INERT — the live planner is DWA (`DWAPlannerROS:`, around line 68). It must
   be moved/added to the DWA block. It is read once at construction, so it is NOT settable
   via dynamic_reconfigure and requires a move_base restart. Also note `getCellCosts`
   returns false (dropping the point entirely) for blocked/unreachable cells, so unreachable
   cells may be ABSENT from the cloud rather than present with 40001 — absence is itself the
   signal.
2. Re-run the downhill goal with DEBUG enabled to confirm whether that stall shares the
   critic-5 signature. The downhill stall had 938 inscribed (99) + 106 lethal (100) cells in
   the local window versus only 3 lethal at earlier stalls — real, live, sensor-supported
   obstacles at 4+ m — so it may have a different signature. This was NOT captured.
3. Investigate why the wavefront fails to propagate from a confirmed-valid seed (the central
   contradiction above).

## Useful commands

```bash
rosservice call /move_base/set_logger_level ros.base_local_planner debug
rosservice call /move_base/set_logger_level ros.dwa_local_planner debug
# ROS_DEBUG does NOT reach stdout — capture off /rosout instead:
rostopic echo /rosout > /tmp/rosout_capture.log
```

## Provenance

Findings measured in a live session on 2026-08-20; all numbers in this file came from direct
measurement against a running sim, not from inference.
