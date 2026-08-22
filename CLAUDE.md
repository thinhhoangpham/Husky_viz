# Husky viz

## What this project is — defensive robotics security research

This repository is a **self-contained simulation testbed for studying the resilience of
autonomous-robot localization and navigation**. The research question is defensive: *when a
mobile robot's pose estimate is corrupted — by sensor noise, drift, spoofed GPS, or a
compromised ROS graph — can the robot detect it and keep navigating safely?*

The work being built here is the **defense side**:
- `landmark_loc/` — an independent, map-based localizer that cross-checks the fused pose
  against lidar observations of known static landmarks, so a corrupted pose can be detected
  rather than blindly trusted.
- `map_tools/`, `maps/` — the known-map/DTM ground reference the defense is validated against.
- `operator/`, drift monitors, and the GPS/compass sanity checks — supervision and
  degradation detection during a live drive.

To measure whether a defense actually works, the testbed must be able to **reproduce the
fault conditions it is supposed to catch**. That is the sole purpose of the adversary-emulation
harness described in the "Defense evaluation" section below: it is a controlled fault
injector — a standard threat-model emulator, the robotics equivalent of a test fixture — used
to generate corrupted-pose scenarios on demand.

**Scope, and it is a hard boundary:** everything here runs against a **Gazebo simulation on
this single, locally-owned machine**, over a private Docker bridge, with no real robot and no
external network in scope. Nothing in this repo targets, or is intended to target, any system
not owned by this project. Any technique added here must be in service of measuring or
improving a defense; the repo is not a place to develop offensive capability for its own sake.

## Environment (native Linux)
- **This is a NATIVE Linux box with ROS Noetic installed directly** — the sim runs
  via `roslaunch` on the host, not in any container. The robot's real X display is
  **`:0`** (Xorg). There is no Xvfb, no `:1`, no noVNC.
- **The Gazebo GUI client `gzclient` must be running on `:0` for the lidar to
  produce its point cloud.** The Ouster is a GPU-ray sensor: it needs a live GL
  context to render the ray pass. Symptom of a missing GL context: `/os0_cloud_node/imu`
  publishes but `/os0_cloud_node/points` has zero publishers, so landmark
  localization and obstacle avoidance go inert. If `/points` has no publisher,
  check DISPLAY/gzclient FIRST — it is not a code or spawn problem.
- Project folder: `/home/thinh/Documents/Husky_viz`.
- Bag: `bag files/park_1_bag/park_1.bag` (≈44.6 GB), with `check_park_1.txt` sha256.
  `bag_file_explorer.ipynb` reads it with the `rosbags` Python library (pure Python).
- Syscall traces: `umdhusky-data_collection-syscall-*-processed.csv`; viz via
  `data_analysis.ipynb`, `timearcs.html/js`, `ridge-plot.html/js`.

## The bag — verified fact
- `park_1.bag` is a **ROS 1** bag: the file header is literally `#ROSBAG V2.0`, and its message types use ROS 1 naming (`gazebo_msgs/ModelStates`, `geometry_msgs/PoseWithCovariance`) with no `/msg/` infix.
- **Discrepancy:** `park_1_topic_breakdown.md` claims the bag is "a ROS 2 Gazebo simulation" and cites `sensor_msgs/msg/...` naming as the tell. That ROS 2 claim is incorrect — verified against the file bytes. Its other claim, that the data is simulator-generated rather than a real robot (based on `/gazebo/model_states`), is consistent with the contents.
- Because it is ROS 1, it plays natively under Noetic with no conversion.

## Usage
- Bring up the world + robot: `./load-park-world.sh` (two stages — park world first,
  then the Husky spawned into it once the world is genuinely up). Run it with the
  real display on `:0` and ensure `gzclient` is up so the lidar cloud publishes.
- The offline-map navigation demo (world → move_base → localizer → selector →
  operator → goals → fault-injection check) is in `RUN-MAP-NAV.md`.
- Control the robot manually: `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/kb_teleop/cmd_vel` — keys `i`/`,` forward/back, `j`/`l` turn, `k` stop, `q`/`z` speed.
- Use `/kb_teleop/cmd_vel` rather than `/cmd_vel`: Husky runs `twist_mux`, which arbitrates inputs by priority; `/kb_teleop/cmd_vel` is the keyboard slot. Available cmd_vel topics: `/cmd_vel`, `/husky_velocity_controller/cmd_vel`, `/joy_teleop/cmd_vel`, `/kb_teleop/cmd_vel`, `/twist_marker_server/cmd_vel`.
- Controller health check (a real recurring bug): `rosrun controller_manager controller_manager list` — both `husky_joint_publisher` and `husky_velocity_controller` must read `( running )`.

---

## Navigation — NO GROUND TRUTH (hard rule)

**Never use Gazebo ground truth for robot pose, in code or in verification.** That means no
`/gazebo/get_model_state`, no `/gazebo/model_states`, no `gazebo_msgs` import, and no constant
whose value was obtained by measuring simulator internal state. This is a standing project
constraint, not a per-task preference.

Why: ground truth does not exist on the real Husky. A driver built on it is a simulator demo that
cannot transfer to hardware, and it hides the actual problem (estimating pose from noisy,
drifting sensors) rather than solving it.

This applies to verification too. Checking a GPS-driven run against ground truth is partly
circular and is not an acceptable way to prove correctness. Judge results by what the robot's own
sensors report, and by what the operator sees in Gazebo.

`/gazebo/set_model_state` for repositioning the robot between test runs is a separate matter — ask
before using it, and never let it become a source of pose data.

### Sensors — what to use instead

| Quantity | Topic | Type | Notes |
|---|---|---|---|
| Position | `/navsat/fix` | `sensor_msgs/NavSatFix` | Absolute, does not drift. Check `status.status >= 0`. |
| Heading | `/compass/data` | `sensor_msgs/Imu` | Absolute world yaw, does not drift. |
| Position (drifting) | `/odometry/filtered` | `nav_msgs/Odometry` | EKF output in the **odom** frame. Drifts — see below. |

Do NOT use `/imu/data` as a heading source — it is mounted rotated 90 degrees.

### Why `/odometry/filtered` drifts

The EKF (`/ekf_localization`) is configured `world_frame: odom`, `imu0_differential: true`, with
`odom0: husky_velocity_controller/odom`. Nothing in that configuration observes **absolute**
position or heading — it fuses wheel encoders plus *relative* IMU yaw only. The Husky is a
skid-steer, so it turns by scrubbing its wheels sideways; the encoders count that scrub as travel.
The error is cumulative and never corrected.

Measured consequence: a driver that latched a WORLD->odom transform at startup put the robot
2.7 m, then 7.7 m, then **13.5 m** off route as distance accumulated — it drove off the mapped
terrain and flipped. Re-deriving the transform once per waypoint was NOT sufficient; it still went
stale within a single long leg.

The fix is not to tune the EKF. It is to use an absolute source (GPS + compass) and recompute
every tick, caching nothing.

### GPS -> world conversion

The GPS plugin's datum is declared in
`natural_environments_ros_opt/husky/husky_description/urdf/gps.urdf.xacro` (lines 33-36):
`referenceLatitude 49.9`, `referenceLongitude 8.9`, `referenceHeading 0`.

Derive the scale from those values plus WGS84 geodesy (equatorial radius 6378137.0, flattening
1/298.257223563) evaluated at the reference latitude — never by measuring the simulator:

    world_x =  (lat - REF_LAT) / deg_lat_per_metre
    world_y = -(lon - REF_LON) / deg_lon_per_metre

The minus sign on `world_y` is real: with `referenceHeading 0`, world +y points WEST, so longitude
decreases as y increases. If the xacro's reference values change, the constants must be updated to
match — nothing detects a mismatch at runtime.

### Working driver

`scripts/drive_to_point_gps.py` — drives to one world (x, y) point using GPS + compass only, closed-loop
at 10 Hz, nothing cached between ticks.

    python3 scripts/drive_to_point_gps.py X Y [--tolerance 0.5] [--max-lin 0.4] [--max-ang 0.8] [--timeout 180]

Control law: turn in place while `|heading_error| > 0.25 rad`, otherwise drive forward with
`linear.x = min(max_lin, 0.6 * dist)` and proportional heading correction. Sign convention:
positive `angular.z` is counter-clockwise (left); `heading_error > 0` means the target is left, so
`angular.z` is positive — no negation anywhere.

Publish to `/cmd_vel`. It is the lowest-priority `twist_mux` input (priority 1, slot `external`);
publishing anywhere else reaches zero subscribers and the robot silently never moves.

### Known-broken (do not copy their approach)

`drive-park-nav.sh` + `gps_goal_sender.py` (move_base) and `auto_drive_waypoints.py` +
`drive-park.sh` all navigate in the drifting odom frame. They are the source of the 13.5 m
divergence above. `drive_to_point.py` was deleted for using ground truth.

Also note: `/os0_cloud_node/points` publishes **only when `gzclient` is up on `:0`** (the GPU-ray sensor needs a live GL context — see the environment note above). When it is up, the cloud is live and the landmark localizer gets clusters; when it is not, `/points` has zero publishers, the move_base costmaps stay empty, and obstacle avoidance is inert. If you see no clusters, check `gzclient`/DISPLAY first — it is not a code problem.

---

## Defense evaluation — adversary emulation (Tier 2 network model, built 2026-07-30)

To measure whether the landmark localizer and drift monitors actually catch a corrupted
pose, this project needs a repeatable way to *cause* that corruption. `attacker/` is that
fault-injection harness: a **rogue-peer emulator**, implemented as a **separate Docker
container** with its own IP on the local Docker bridge (Tier 2 model: a peer that discovers
and reaches the ROS master, then publishes bad data), used purely to generate test scenarios
for the defenses under development. It targets only this machine's own simulated ROS graph —
there is no external network or third-party system in scope. Design and rationale (tiers, why
no Gazebo attacker model): `docs/attacker-network-simulation.md`; spec + plan under
`docs/superpowers/`.

- **Lives in `attacker/`** — self-contained. `Dockerfile` (base
  `ros:noetic-ros-core` + `nmap` + `iproute2`; **no Gazebo**), `entrypoint.sh`
  (derives the container's own IP → exports `ROS_IP`), `docker-compose.yml`
  (default docker0 bridge, `ROS_MASTER_URI` built from `ROBOT_HOST_IP`, repo
  bind-mounted **rw** at `/repo`), and three phase scripts.
- **The four `attack_*.py` run UNCHANGED** — bind-mounted from the repo root, not
  copied. `attack.sh` only validates a name (`cmd_vel|compass|odom|param`) and
  execs the existing script.
- **Runbook: `attacker/README.md`** — four phases: **0** host prep, **1**
  `scan.sh` (nmap reachability), **2** `enum.sh` (read-only graph read), **3**
  `attack.sh` (inject).
- **Sim runs NATIVELY on this Linux box** (`roslaunch`, not a container). The
  attacker container reaches the native master via the **docker0 gateway IP**.
- **The `ROS_IP` gotcha (bites once):** before `roslaunch`, the host must
  `export ROS_IP=<docker0 gw IP>` and `ROS_MASTER_URI=http://$ROS_IP:11311`.
  Without `ROS_IP` the master advertises `127.0.0.1`, so **nmap succeeds but the
  container's `rostopic` calls hang** — `enum.sh` prints this exact diagnosis.
- **Invocation quirk:** scripts are called as `docker compose run --rm attacker
  ./attacker/scan.sh` — the `attacker/` prefix is required because `working_dir`
  is `/repo` and the scripts are mounted one level down at `/repo/attacker/`.
- **Deferred:** Tier 3 (on-the-wire MITM) and the geofence/trigger layer — both
  intentionally out of scope, noted in the spec.

### Docker on this Linux box (installed 2026-07-30)

- Docker CE **28.1.1** + compose plugin **v2.35.1**, from Docker's official apt
  repo via `install-docker.sh` (repo root). Daemon enabled and running.
- `thinh` is in the `docker` group, **but** the group only activates in a **new
  login session** — after install, `docker` needs `sudo` until you log out/in or
  run `newgrp docker`.

---

## Repository organization — KEEP IT TIDY as development moves forward

The repo root was decluttered on 2026-08-14. **Maintain this structure**: when you
create or move files, put them in the right place and do not let the root accumulate
loose scripts, data, or junk again. This is a standing instruction, not a one-time
cleanup.

### Where things go
- **`scripts/`** — standalone driver/plot/audit/utility `.py` and `.sh` tools that
  are NOT container- or import-pinned (e.g. `drive_to_point_gps.py`, `plot_*.py`,
  non-dispatched `attack_*_*.py` variants). New one-off tools go here, not at root.
- **`artifacts/`** — all run OUTPUTS: `.csv`, `.png`, plot `.json`, logs. Never commit
  scratch outputs to root. Point script `--csv`/`--out` defaults or your invocation at
  `artifacts/` when practical.
- **`archive/`** — dead/superseded files kept for reference (`.bak`, `.gps-ekf`,
  old variants). Prefer deleting truly disposable files (git has history); archive only
  what is ambiguous.
- **`landmark_loc/` `map_tools/` `operator/`** — the package trees. Package code stays
  in its package; the type registry is `map_tools/park_types.py` (single source for
  both map-extraction and lidar-classification — see the detector section).
- **`launch/` `config/` `maps/` `docs/`** — as named. Runbooks (`RUN-*.md`) currently
  live at root by convention; leave them unless asked.
- **macOS `._*` files** are junk — delete on sight, never commit.

### Files PINNED to repo root — DO NOT MOVE (they break the sim/containers)
These are referenced by hardcoded container paths, bare-name exec, or cross-imports.
Moving any requires editing its caller too — do not relocate casually:
- **Container-dispatched attacks:** `attack_cmd_vel.py`, `attack_compass.py`,
  `attack_odom.py`, `attack_param.py`, `attack_goal.py`, `attack_navsat.py` —
  `attacker/attack.sh` execs `/repo/attack_<name>.py` in the bind-mounted container.
- **Import targets:** `goal_marker.py`, `send_mapless_goal.py` — imported by root
  scripts AND the operator container (mounts `../:/repo`).
- **Exec'd by name:** `spawn_robot_idle.py`, `reset-robot.py` (by
  `spawn-robot-idle.sh`); `monitor_navsat_drift.py` (by `RUN-GOAL-HIJACK.md`).
- **Primary entry point:** `load-park-world.sh` (invoked by all runbooks + this file).
- **`/workspace`-external:** `husky_teleop*.py`, `park-env*.sh` — resolved inside a
  separate Gazebo container, not this repo; leave in place.

### When you move a referenced file
Trace its callers first (launch files, `*.sh`, `attacker/`, `operator/`, `RUN-*.md`,
this CLAUDE.md, python imports) and update every reference in the same change. A file
referenced by bare name or absolute path breaks silently at runtime if moved without
fixing the caller.

---

## Obstacle avoidance — two fixes, both required (2026-08-22)

The robot used to drive ~40 m and stop dead in open terrain, or grind against bushes it
could see. Two INDEPENDENT defects; neither fix alone is sufficient. Full analysis and
evidence: `docs/dwa-unreachable-goal-investigation.md`.

**1. DWA seeded its goal-wavefront on a cell it also treats as a wall.**
`setLocalGoal` accepted a seed if the cost was `!= NO_INFORMATION` (255 only), while
`updatePathCell` refuses to expand through LETHAL (254), INSCRIBED (253) **and** 255. A 253
cell was therefore a valid seed AND an impassable wall: the flood visited 0 cells, every
cell kept 40001, all 300 trajectories scored -2.0, and DWA commanded exactly zero velocity.

Fixed in a vendored `base_local_planner` at `~/husky_overlay_ws` (tag 1.17.3, matching the
installed package; only that package is built). **Deployment is automatic**: an `env` tag
inside the move_base node in `launch/move_base_gps_map.launch` puts
`~/husky_overlay_ws/devel/lib` ahead of `/opt/ros/noetic/lib` on the loader path, scoped to
that node. No export is needed in any terminal.

**Never `source` that workspace's `setup.bash`** as an alternative — its clone carries 17
unbuilt navigation packages that then shadow the working installed ones, and
move_base/map_server fail with "Cannot locate node of type". Only the library path is wanted.

**The patched .so lives OUTSIDE this repo, so the fix is not portable.** On a machine with
no `~/husky_overlay_ws` build the path silently does not exist, the stock library loads, and
the bug returns with no warning. Verify against the LIVE process (the library loads lazily
when DWA is constructed, so check after move_base is up):

    grep -o '[^ ]*libbase_local_planner.so' /proc/$(pgrep -f lib/move_base/move_base)/maps

Only `launch/move_base_gps_map.launch` carries the env tag. `move_base_landmark.launch` and
the other move_base launches still load the STOCK library and remain exposed to the bug.

**2. The global plan routed through obstacles the global costmap could not see.**
`config/costmap_global_gps_map.yaml` now has an `obstacles` ObstacleLayer between static and
inflation. Without it navfn planned through vegetation (deliberately absent from the static
map) and the local planner was asked to resolve a plan running through solid geometry.

This REVERSES commit `ecb0634`, which removed that layer after measuring 2373/2760 (86%) of
global lethal cells came from lidar. That did not reproduce: over a full 92 m lake drive
global lethal rose and FELL (342→642→383), ending at 383 vs 342 static — ~11% from lidar.
Clearing works now because the layer is fed the FILTERED cloud for marking and the RAW cloud
for clearing (`9c27dae`), which post-dates `ecb0634`. **Watch for regression:** monotonically
climbing global lethal cells means the `ecb0634` symptom is back.

### What was DISPROVED — do not retry these

- **Not a local-planner choice.** Swapping DWA for `TrajectoryPlannerROS` changed nothing;
  it also froze, 7 m short of a box with 4.95 m clear ahead.
- **Not scoring tuning.** Raising `occdist_scale` 0.4→1.0 made it WORSE (189 s→270 s in the
  obstacle zone, 21→27 direction flips). Lowering `path_distance_bias` 34→15 did not help.
  The known-working reference sim runs a far more extreme ratio (96 vs 0.02) and avoids fine.
- **Not sensing.** Fails identically on sparse bushes, solid boxes, and a spawned box.
- **Not unknown cells, TF errors, or the EKF z change.** 0 NO_INFORMATION cells in 516
  samples; TF errors flat with no onset; the 2D stack discards z entirely
  (`goal_functions.cpp:130-132` prunes on x/y only).
- **`TrajectoryPlannerROS:` in `config/planner_gps.yaml` is INERT.** Editing it looks like a
  change and does nothing. The live block is `DWAPlannerROS:`.

---

## Running a sim test — MANDATORY PROCEDURE, NO EXCEPTIONS

**Every sim test follows this exact sequence. Skipping any part of it is not allowed,
and "it's optional / display-only / not relevant to what I'm testing" is NOT a reason
to skip.** This rule exists because skipping steps and reporting unverified guesses
has repeatedly burned the user's time and context.

### 1. Kill everything first
Run the full teardown block from `RUN-MAP-NAV.md` ("Stop everything"). Then **read the
survivor list** — do not trust exit code 0. The scan matches on the absolute repo path,
so nodes started with a **relative** path (`python3 scripts/foo.py`) are NOT matched and
survive silently. Kill those by explicit PID. Verify: no sim processes, port 11311 free,
operator container gone, `husky_lan` gone.

### 2. Start fresh — run EVERY terminal, in doc order
No reusing a running stack. No stacking a new run on an old one. Run Steps 0-3 of
`RUN-MAP-NAV.md` in order, **including the ones the doc calls optional**:
Terminal 0 (network), 1 (world+robot), 2 (map_server+move_base), 3 (localizer),
4 (selector), 5 (ground-height), **6 (costmap z relay GLOBAL)**, **6b (costmap z relay
LOCAL)**, 6c (path/goal z relay), 7 (cloud filter), then Step 3 (operator).

**Terminals 6 and 6b are NOT optional in practice.** `operator/operator.rviz` points its
Costmap displays at `/move_base/{global,local}_costmap/costmap_z`, which ONLY those two
relays publish. Skip them and RViz shows **"No map received"** on both costmaps — which
looks exactly like a broken costmap and will send you chasing a nonexistent bug.

For the **lake** world, three things change (see the table at the top of RUN-MAP-NAV.md):
`./load-park-world.sh --world lake`; append
`map:=/home/thinh/Documents/Husky_viz/maps/lake_map_terrain.yaml` to Terminal 2;
`_objects_path:=.../lake_objects.yaml`. Export BOTH `DTM_WORLD=lake` and
`OBJECTS_PATH=/repo/maps/lake_objects.yaml` before the operator.

### 3. Report every terminal to the user
State each terminal's status individually as you bring it up. Not a summary.

### 4. Verify EVERYTHING at the end, before sending a goal
Controllers both `( running )`; gzclient on `:0`; TF `map->odom->base_link` resolves;
and every topic checked. Only then send the goal.

### HOW TO CHECK A TOPIC CORRECTLY — three traps that produced FOUR false alarms

1. **`costmap` is LATCHED and published ONCE; the live data is on `costmap_updates`.**
   `costmap_2d` sends the full grid once (latched) and then streams incremental patches
   on `/move_base/<ns>/costmap_updates`. Measuring `/costmap` and finding "1 msg, 0 Hz"
   is **CORRECT AND EXPECTED — NOT A FAULT**. Measured: `/costmap` 1 msg/15 s vs
   `/costmap_updates` 22 msgs/15 s. The `costmap_z` relays mirror `/costmap`, so their
   output is legitimately static too.
2. **rospy callbacks need a real spin.** Counting messages with bare `time.sleep()` in
   the main thread silently under-counts and reports live topics as SILENT. Use
   `rospy.Rate().sleep()` in a loop, or `rospy.wait_for_message`.
3. **The odom EKF publishes `/odometry/filtered_odom`, NOT `/odometry/filtered`.**
   The map EKF publishes `/odometry/filtered_map`. `/odometry/filtered` does not exist in
   this configuration; subscribing to it reads 0 and looks like a dead EKF.

Also: `rostopic echo -n1 <topic>/info` proves NOTHING about whether a topic is live — it
returns the latched message even at 0 Hz. And a single startup
`Timed out waiting for transform ... map does not exist` from move_base is benign; it
recovers. `/odometry/landmark_fix` silent at idle is expected (needs motion for clusters).

**Before declaring any component broken: verify with a correct method, and check how the
component is DESIGNED to publish. Do not report a suspicion to the user as a finding.**
