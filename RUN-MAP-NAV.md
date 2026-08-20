# Offline-Map Navigation Demo

One terminal per step. Each block sets its own ROS env, so copy-paste as-is.

> **Park is the default.** To run the same demo in the **lake** world, change
> the things below — everything else is identical.
>
> | Step | Park (default) | Lake |
> |---|---|---|
> | 1 | `./load-park-world.sh` | `./load-park-world.sh --world lake` |
> | 2 | `roslaunch launch/move_base_gps_map.launch` | append `map:=$HOME/Documents/Husky_viz/maps/lake_map_terrain.yaml` |
> | 3 | `_objects_path:=…/maps/park_objects.yaml` | `…/maps/lake_objects.yaml` |
>
> Lake is self-contained in `models_lake_opt/` (low-poly visuals *and* the original
> collision meshes) — no external drive needed to run it.
> Its landmarks are `tree`, `postescable`, `lago` — and note **the lake itself is a
> landmark, not an obstacle**: `lago` has no `<collision>`, so the lidar cannot see
> the water and it is deliberately absent from `lake_map.pgm`. The water IS blocked
> for the planner, but as **unknown, not as an obstacle** — see "Terrain mask" below.
> Low vegetation is
> skipped in the static map too; the robot dodges it from live lidar.
> Step 3's operator (`operate.py`) still reads `maps/park_objects.yaml` — goal-by-name
> is park-only until that path is parameterised.

## Step 0 — Docker network

```bash
docker network create --subnet 172.20.0.0/16 husky_lan 2>/dev/null || true
```

## Step 1 — World + robot

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
./load-park-world.sh
```

Wait for Gazebo to show the park + robot, then ~30–60 s for the pose to settle.

## Step 2 — Navigation + map

### Terrain mask — how open water is made no-go

The map `map_server` loads is **not** the plain object map. It is
`maps/<world>_map_terrain.pgm`, the object map with every cell the world's DTM
has no terrain for — open water, and anything past the edge of the terrain mesh
— overwritten as **unknown** (pixel 205), plus a 1.0 m eroded keep-out margin
around that region.

**Unknown, deliberately not lethal.** Driving into the lake means *sinking*, not
hitting a wall, so the correct statement is "there is no ground information
here, do not plan through it" (`costmap_2d` `NO_INFORMATION`) rather than
"there is an obstacle here". Nothing marks water occupied, and nothing should.

Three settings make that stick, and all three are required together:

| Where | Setting | Value |
|---|---|---|
| `launch/move_base_gps_map.launch` | `map` arg | `maps/<world>_map_terrain.yaml` |
| `config/costmap_global_gps_map.yaml` | `track_unknown_space` | `true` |
| `config/planner_gps.yaml` | `NavfnROS/allow_unknown` | `false` |

With any one of them flipped back, unknown collapses to free and the planner
will route the robot across the lake and off the mesh, where it falls (measured
previously: GPS altitude −36401 m).

Regenerate after any change to the DTM or the object map:

```bash
cd ~/Documents/Husky_viz
python3 -m map_tools.clip_map_to_terrain park --mask-unknown --no-crop --erode 1.0 --out-suffix _terrain
python3 -m map_tools.clip_map_to_terrain lake --mask-unknown --no-crop --erode 1.0 --out-suffix _terrain
```

`--no-crop` is load-bearing: the masked map must keep the plain object map's
exact width, height, origin and resolution. `--erode 1.0` exists because
`costmap_2d`'s `InflationLayer` does **not** inflate `NO_INFORMATION` cells, so
without it the planner will happily park the robot's centre on the last terrain
cell and let its footprint (0.60 m circumscribed radius) overhang the void.

Note `*.pgm` is gitignored and the map PGMs are force-added, so a regenerated
mask will **not** appear in `git status` — commit it with
`git add -f maps/<world>_map_terrain.pgm`.

> **Known risk, expected not surprising:** commit e628992 recorded that the
> global costmap stopped publishing under exactly this
> `track_unknown_space: true` + `allow_unknown: false` pairing, and that symptom
> was never diagnosed. It may reappear. If `/move_base/global_costmap/costmap`
> goes silent after this change, that is the known issue, not a new one.

Start these **five** nodes, each in its own new terminal (Step 1 is Terminal 1). All five must be running. **Terminals 6 and 6b** are optional and display-only — skip them and navigation is unaffected, only the RViz costmap overlays (global and local respectively) go empty.

**Terminal 2 — map_server + move_base:**

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
roslaunch launch/move_base_gps_map.launch
```

**Terminal 3 — landmark localizer** (fills `/odometry/landmark_fix` from the lidar):

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
PYTHONPATH=~/Documents/Husky_viz:$PYTHONPATH python3 ~/Documents/Husky_viz/landmark_loc/localizer_node.py _objects_path:=/home/thinh/Documents/Husky_viz/maps/park_objects.yaml _matcher:=typed
```

> `_matcher:=typed` (default) runs the identity-aware RANSAC constellation matcher. `_matcher:=typeless` runs the geometry-only control matcher (ignores landmark identity; matches on pairwise-distance shape alone) — used to compare identity-vs-geometry. To demo the other mode, relaunch Terminal 3 with `_matcher:=typeless`; the localizer logs `[localizer] matcher mode: <mode>` at startup so you can confirm which is active.

**Terminal 4 — pose-source selector** (fills `/odometry/abs_fix`; starts in `gps` mode):

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
python3 ~/Documents/Husky_viz/landmark_loc/abs_fix_selector.py
```

**Terminal 5 — ground-height publisher** (fills `/odometry/ground_height`; this is what gives the robot its TRUE altitude):

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
PYTHONPATH=~/Documents/Husky_viz:$PYTHONPATH python3 ~/Documents/Husky_viz/scripts/publish_ground_height_odom.py _dtm_path:=/home/thinh/Documents/Husky_viz/maps/lake_dtm.npy
```

> Swap `lake_dtm.npy` for `park_dtm.npy` when running the park world — the DTM must match the world in Step 1.
>
> **`_dtm_path` is not optional.** Both EKFs fuse this topic as absolute z (`odom1` in `localization.yaml`, `odom2` in `localization_map.yaml`). Without the DTM the node publishes the robot's *clearance above the ground* (~0.13 m) instead of its absolute elevation, and the EKF reads that as altitude — sinking the robot ~3.75 m under the terrain in RViz. The node logs `no ~dtm_path: publishing CLEARANCE only` when this is wrong. Use an **absolute** path: `~` is not expanded inside `_dtm_path:=`.
>
> Check it with `rostopic echo -n1 /odometry/ground_height/pose/pose/position/z` (expect ~3.75 m on the lake) and `rosrun tf tf_echo map base_link` (z should match, not 0.000).

**Terminal 6 — costmap z relay, GLOBAL** (display only; publishes `/move_base/global_costmap/costmap_z`):

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
python3 ~/Documents/Husky_viz/scripts/relay_costmap_z.py _world:=lake
```

> Use `_world:=park` for the park world — the DTM must match the world in Step 1.
>
> **What it is for.** `costmap_2d` hardcodes the published `origin.position.z` to
> 0 (`Costmap2DPublisher` keeps only `saved_origin_x_/y_`), and there is no
> parameter that changes it — the `origin_z` that exists belongs to `VoxelLayer`
> and is a 3-D voxel column base, not a render offset. So the global costmap
> always arrives at z = 0, while the terrain it describes sits at 3.5–5.9 m
> (lake). Since Step 5 gives the robot its true altitude, RViz drew the terrain
> and robot hovering several metres above a grey sheet. This node republishes the
> grid at the DTM's **minimum** height on a **new** topic.
>
> **It is display-only and cannot affect navigation.** It never writes to
> `/move_base/global_costmap/costmap`; move_base and NavfnROS keep consuming that
> topic byte-for-byte unchanged. Occupancy values pass through untouched, so
> water stays **unknown**, never lethal. `operator/operator.rviz` points its
> "Costmap (global)" display at `…/costmap_z`; skip this terminal and that
> display is simply empty — nothing else degrades.
>
> **Why the minimum, and why that is approximate.** An `OccupancyGrid` is flat:
> one origin, no per-cell height, so it can only sit at a single z and cannot
> follow relief. At the minimum the sheet lies at or below the surface
> everywhere and never pokes through — a sheet that intersects terrain reads as
> a rendering fault, one beneath it reads as a projection. Cost: under the
> highest ground it sits ~2.4 m low in the lake world (relief 2.42 m). In the
> park world relief is 0.007 m, so the choice is immaterial. Values: park
> **2.986 m**, lake **3.505 m**.

**Terminal 6b — costmap z relay, LOCAL** (display only; publishes `/move_base/local_costmap/costmap_z`):

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
python3 ~/Documents/Husky_viz/scripts/relay_costmap_z.py _world:=lake _follow_robot:=true \
    _in_topic:=/move_base/local_costmap/costmap \
    _out_topic:=/move_base/local_costmap/costmap_z
```

> Use `_world:=park` for the park world. This is a SECOND instance of the same
> node as Terminal 6 — no new script — differing only in its parameters.
>
> **Why a second instance, and why `_follow_robot`.** The local costmap has the
> same z = 0 problem as the global one, but the fixed DTM-minimum height that
> Terminal 6 uses would be wrong for it. The local costmap is
> `rolling_window: true`, 10×10 m, in the `odom` frame, and it TRAVELS WITH THE
> ROBOT (`config/costmap_local_gps.yaml`); pinned to the map minimum, its patch
> would sink further below the ground the higher the robot climbed.
> `_follow_robot:=true` instead looks the robot up in tf (`map`→`base_link`) on
> every costmap message and samples the DTM over the window's own footprint,
> taking the **minimum** over that window.
>
> **Why the window minimum and not the single cell under the robot.** The
> "never poke through the terrain" rule still has to hold, and on the lake it is
> not a formality: a single 10 m window spans up to **2.007 m** of the map's
> 2.422 m total relief, and 79% of windows exceed 0.5 m (median 0.78 m). A sheet
> at the robot's own ground height would cut visibly through the slope ahead
> across most of the map. The window minimum keeps it underneath everywhere
> while still tracking the robot — mean gap under the robot **1.033 m → 0.430 m**
> versus the global minimum, worst case **2.409 m → 1.371 m**.
>
> **Why not the robot's fused z.** Since Step 5 the EKF fuses absolute z, so
> `base_link` z is a real altitude — but it is terrain *plus* clearance, so a
> sheet there floats above the ground and pokes through it. It would also wire
> the pose *estimate* into the display: under drift or a GPS spoof the sheet
> would move with the corrupted pose and make it look self-consistent. The
> display stays anchored to the offline DTM instead.
>
> **Display-only, exactly like Terminal 6.** It never writes to
> `/move_base/local_costmap/costmap`; move_base's own local planner (DWA) keeps
> consuming that topic byte-for-byte unchanged, and the node refuses to start if
> `_in_topic` and `_out_topic` are equal. `operator/operator.rviz` points its
> "Costmap (local)" display at `…/costmap_z`; skip this terminal and that display
> is simply empty.
>
> If tf is not yet up, or the robot wanders off the DTM, the node holds the last
> good z (logged, throttled) rather than dropping the sheet to 0 — and publishes
> nothing at all until the first valid sample, so the bug can never flash back.

**Terminal 7 — terrain-relative cloud filter** (fills `/os0_cloud_node/points_above_terrain`; **the local costmap's only obstacle source**):

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
python3 ~/Documents/Husky_viz/scripts/filter_cloud_above_terrain.py _world:=lake
```

> Use `_world:=park` for the park world — the DTM must match the world in Step 1.
>
> **This terminal is not optional.** `config/costmap_local_gps.yaml` points the
> local costmap's observation source at `/os0_cloud_node/points_above_terrain`.
> Skip this node and that topic has no publisher, so the local costmap sees **no
> obstacles at all** and close-range avoidance is inert. If the robot ignores an
> obstacle, check `rostopic info /os0_cloud_node/points_above_terrain` for a
> publisher before debugging anything else.
>
> **What it is for.** `costmap_2d` decides "is this point an obstacle?" with a
> scalar comparison against **absolute z** in the costmap frame
> (`min_obstacle_height`/`max_obstacle_height`), which is only correct if the
> ground is a horizontal plane at a fixed height. On relief it marks the
> **ground itself** as lethal — the crescent of false obstacles 2–5 m ahead of
> the robot (measured once: 6355 lethal cells all within 6 m, while the nearest
> real mapped landmark was 7.88 m away — every one of them false). Measured in
> the lake world on a 3.9° slope (n = 16894 returns): against **its own terrain
> cell** the ground spans just +0.083…+0.090 m and objects start at +1.2 m;
> in **absolute z** the same ground smears over 4.24…4.56 m and that smear moves
> as the robot drives. `costmap_2d` has no hook for a terrain-relative test, so
> this node makes the decision upstream: it looks the terrain height up from
> `maps/<world>_dtm.npy` at **each point's own (x, y)** and republishes only the
> returns **0.40–3.00 m above the ground beneath that point**. The costmap's own
> gate is opened to ±1000 m so it can never override that.
>
> **The band.** Ground tops out at +0.090 m with a 7 mm spread and objects start
> near +1.2 m, so 0.40 m sits roughly mid-gap — deliberately not marginal.
> 3.00 m clears the Husky with room for real overhangs while dropping canopy
> (p95 = +7.19 m), which must not be marked: the robot drives under trees.
> Override with `_min_height:=` / `_max_height:=`.
>
> **Points with no terrain under them are dropped** (open water, off-mesh void).
> There is no ground reference there, so no honest height test exists, and the
> global costmap already treats those cells as unknown keep-out. Pass
> `_keep_off_dtm:=true` to mark them instead.
>
> **On a TF failure the cloud passes through UNFILTERED**, with a throttled
> warning. Publishing an empty cloud would tell the costmap the way ahead is
> clear, which is the one failure mode that can drive the robot into something.
>
> **It is not spoof-resistant.** The terrain lookup is indexed by the point's
> map-frame position, which comes from the robot's own pose estimate; under a
> pose spoof it reads the wrong terrain cells. It is a geometry correction for
> the costmap, not a detector — the defences against pose corruption are the
> landmark localizer and the drift monitors.

Both pose feeders (GPS + landmark) run at once, publishing to separate topics. The selector forwards exactly one to the EKF, and the operator switches between them live with `mode gps` / `mode landmark` (Step 4) — no relaunch, no choosing a launch file.

## Step 3 — Operator

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz/operator
docker compose up -d
docker compose exec operator bash -lc "source /opt/ros/noetic/setup.bash && ./operator/operate.py"
```

Watch the robot in a browser: **http://localhost:6080/vnc.html**. For the full window (not cut off), use **http://localhost:6080/vnc.html?resize=scale**.

> **Display datum: true world elevation.** The DTM cloud is published unshifted
> (`DTM_OFF` is `0`), so terrain, the robot (true altitude since Step 5) and the
> relayed costmap all render at real-world height and share one datum. This
> reverses an earlier fix that pulled the terrain DOWN to z≈0 to meet a robot
> that was pinned there; now that the robot carries true z, that shift would
> leave it floating ~4 m above the ground. Set `DTM_Z_OFFSET=<metres>` in the
> operator environment to shift both DTM layers back if needed.

> **Rebuild after editing `operator/entrypoint.sh`:** the operator container bakes `entrypoint.sh` into its image. After changing it (resolution, RViz maximize, etc.), a plain `docker compose up` reuses the old image and your change will NOT apply — you must rebuild: `cd operator && docker compose build && docker compose up -d`.

## Step 4 — Send a goal (at the `operator>` prompt)

```
goal 49.9000094 8.9000327      # GPS lat/lon
goal xy 9.17 13.55             # map coordinates
goal garden_table              # landmark name
mode gps                       # switch absolute source to GPS (navsat)
mode landmark                  # switch absolute source to landmark localizer
```

Landmarks: `bench`, `garden_table`, `lamp`, `trash_bin_1`.

Switching is live — no relaunch needed. `status` shows `abs_fix=<mode>` (with
`:stale` appended if the selected source has gone silent).

## Step 5 — (optional) Drop an obstacle in its path

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
python3 - <<'PY'
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
rospy.init_node("spawn_box", anonymous=True)
sdf = """<?xml version="1.0"?><sdf version="1.6"><model name="surprise_box">
<static>true</static><link name="link">
<collision name="c"><geometry><box><size>1 1 1.5</size></box></geometry></collision>
<visual name="v"><geometry><box><size>1 1 1.5</size></box></geometry>
<material><ambient>1 0.3 0 1</ambient><diffuse>1 0.3 0 1</diffuse></material></visual>
</link></model></sdf>"""
rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=10)
sp = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
sp("surprise_box", sdf, "", Pose(position=Point(20.0, 0.0, 3.65),
                                 orientation=Quaternion(0,0,0,1)), "world")
PY
```

Edit `Point(20.0, 0.0, 3.65)` to a spot ahead of the robot (keep `z=3.65` — the
ground is at z≈2.9). Remove it:

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
rosservice call /gazebo/delete_model '{model_name: surprise_box}'
```

## Step 6 — (optional) Attacker: GPS spoof

While the robot is driving to a goal, an attacker on the network slowly fakes
its GPS. `navsat_transform` accepts the drifting fixes, so the map-EKF's fused
position is dragged off course — the robot chases a phantom and thrashes, while
the operator's display still looks nominal (it reads the corrupted pose).

Send a goal (Step 4) so the robot is en route, then run the attacker:

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz/attacker
docker compose run --rm attacker ./attacker/attack.sh navsat --drift-rate 0.5 --max-offset 15 --duration 40
```

Watch in RViz: the robot lurches/spins off its route as the fused pose drifts
(~15 m over 40 s). When the attack stops, genuine GPS reels the estimate back.

Stronger variant: `--drift-rate 1.5 --max-offset 40 --duration 40`.

The spoof only affects the fused pose while `abs_fix` is in `gps` mode; running
`mode landmark` at the `operator>` prompt removes navsat from the loop live, letting
the operator switch away from a spoofed source mid-attack.

## Step 7 — Full demo: GPS → spoof → switch to landmarks → recover

The headline flow: the robot navigates on GPS, an attacker drifts the GPS so the
robot is dragged off course, then the operator switches the absolute source to
landmarks live — GPS leaves the loop, and the robot reaches its goal on lidar
localization instead.

Assumes Steps 0–3 are up (world+robot, move_base, localizer, selector, operator).
The selector starts in `gps` mode by default.

1. **Confirm GPS mode and send the goal.** At the `operator>` prompt:

   ```
   status                         # expect: abs_fix=gps
   goal 49.9000094 8.9000327      # GPS lat/lon goal (known-good, free space)
   ```

   The robot should begin driving toward the goal on the GPS-anchored pose.

2. **Launch the GPS spoof** (separate terminal) while the robot is en route:

   ```bash
   export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
   cd ~/Documents/Husky_viz/attacker
   docker compose run --rm attacker ./attacker/attack.sh navsat --drift-rate 0.5 --max-offset 15 --duration 40
   ```

   Watch in RViz (**http://localhost:6080/vnc.html**): the fused pose drifts and
   the robot lurches off its route. Because the selector is in `gps` mode, the
   spoofed navsat fix is what `abs_fix` carries.

3. **Switch to landmarks live** — at the `operator>` prompt, mid-attack:

   ```
   mode landmark                  # abs_fix now = landmark localizer, navsat out of the loop
   status                         # expect: abs_fix=landmark
   ```

   The GPS spoof no longer reaches the EKF: `abs_fix` is now filled by the lidar
   landmark localizer, which the attacker cannot touch. The fused pose re-anchors
   to what the robot actually sees.

4. **Confirm recovery.** Re-send the same goal so the robot re-plans from the
   now-truthful pose (the drift during the attack may have left it mid-route):

   ```
   goal 49.9000094 8.9000327
   status                         # watch dist decrease; abs_fix should stay 'landmark'
   ```

   The robot should drive to the goal on landmark localization and arrive, with
   the attacker still running — demonstrating the switch defeats the spoof.

## Stop everything

**Read this before running it.** Two things about this block are deliberate and may
surprise you:

- **It kills EVERY ROS/Gazebo process on this machine**, not just the ones your
  current shell started — including orphaned nodes left behind by earlier sim runs.
  That is the point: ghost nodes accumulating on a shared master is the failure this
  block exists to prevent. If you are running an unrelated ROS process on this box,
  it will be killed too.
- **It exits non-zero and prints the survivors** if anything is still alive at the
  end. Silence + exit 0 is the only "everything is down" signal; never assume success
  without checking.

It does *not* use `pkill -f` for the kill — on this box `pkill -f` matches nothing
even when the pattern is literally the start of the cmdline. It enumerates PIDs from
`/proc/*/cmdline` and kills each by explicit PID (SIGTERM, then SIGKILL), which does
work.

Matching happens in **two tiers**, and the order matters:

1. **Unconditional.** Anything running out of the ROS install prefix
   `/opt/ros/noetic/`, or any `gzserver`/`gzclient`/`gzweb`/`rosmaster`/`roscore`, is
   killed with **no exclusions applied at all**. A node under the ROS prefix is a sim
   node by definition, so nothing may exempt it. (An earlier version checked
   exclusions first and let real nodes escape on a bare substring — a
   `velodyne_pointcloud/.../--decoder=hdl` node was spared because "decoder" contains
   "code".)
2. **Exclusion-filtered.** Processes matched only by the *repo path* or by a bare
   mention of `roslaunch` are the ambiguous ones — that is where a VS Code window with
   a repo file open, or a shell whose command line quotes a roslaunch invocation,
   shows up. Editor/tooling exclusions apply to this tier only, and they anchor on the
   executable name (`*/code`, `*/code *`) rather than a bare substring, so they cannot
   fire on an arbitrary substring. The residual gap is narrow but real: a sim node
   launched from the repo *outside* `/opt/ros/noetic/` whose executable is literally
   named `code`, `vim`, `cursor`, etc. would be spared. Nothing in the current stack
   is, and such a process would still be reported as a survivor with a non-zero exit
   rather than silently ignored.

```bash
set -u
REPO="/home/thinh/Documents/Husky_viz"

# Enumerate sim PIDs from /proc. Two tiers -- see the note above.
scan_sim_pids() {
  local d pid cmd
  for d in /proc/[0-9]*/; do
    pid=${d#/proc/}; pid=${pid%/}
    case "$pid" in "$$"|"$PPID"|1) continue ;; esac
    cmd=$(tr '\0' ' ' < "${d}cmdline" 2>/dev/null) || continue
    [ -n "$cmd" ] || continue

    # Tier 1: ROS install prefix + Gazebo/master. ALWAYS killed, never exempt.
    case "$cmd" in
      */opt/ros/noetic/*|*/gzserver|*/gzserver\ *|gzserver\ *|\
      */gzclient|*/gzclient\ *|gzclient\ *|*/gzweb|*/gzweb\ *|\
      */rosmaster|*/rosmaster\ *|rosmaster\ *|*/roscore|*/roscore\ *|roscore\ *)
        echo "$pid"; continue ;;
    esac

    # Tier 2: repo path or a bare 'roslaunch' mention -- ambiguous, so filter
    # out editors/IDEs and agent tooling by EXECUTABLE NAME (anchored on '/').
    case "$cmd" in
      *"$REPO"*|*/roslaunch|*/roslaunch\ *|roslaunch\ *|*\ roslaunch\ *)
        case "$cmd" in
          */code|*/code\ *|*/codium|*/codium\ *|*/cursor|*/cursor\ *|\
          */nvim|*/nvim\ *|*/vim|*/vim\ *|*/emacs|*/emacs\ *|\
          */sublime_text|*/sublime_text\ *|*/pycharm*|*/idea*|*/jetbrains*|\
          */claude|*/claude\ *|*claude-code*|*snapshot*) continue ;;
        esac
        echo "$pid" ;;
    esac
  done
}

# SIGTERM, wait, SIGKILL. Repeat: respawners and multi-generation residue can
# need several passes. Bounded at 4 so this can never spin forever.
for pass in 1 2 3 4; do
  pids=$(scan_sim_pids); [ -n "$pids" ] || break
  echo "pass $pass: SIGTERM -> $(echo $pids | tr '\n' ' ')"
  for p in $pids; do kill -15 "$p" 2>/dev/null; done
  sleep 3
  pids=$(scan_sim_pids); [ -n "$pids" ] || break
  echo "pass $pass: SIGKILL -> $(echo $pids | tr '\n' ' ')"
  for p in $pids; do kill -9 "$p" 2>/dev/null; done
  sleep 1
done

# Docker teardown: operator stack, attacker containers, the husky_lan network.
( cd "$REPO/operator" && docker compose down ) || echo "WARN: docker compose down failed"
att=$(docker ps -aq --filter name=attacker)
[ -n "$att" ] && { docker rm -f $att || echo "WARN: could not remove attacker containers"; }
docker network inspect husky_lan >/dev/null 2>&1 && \
  { docker network rm husky_lan || echo "WARN: could not remove husky_lan"; }

# Verify. Anything below that reports a problem means teardown did NOT succeed.
fail=0
survivors=$(scan_sim_pids)
if [ -n "$survivors" ]; then
  fail=1; echo "FAIL: sim processes still alive:"
  for p in $survivors; do echo "  $p  $(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)"; done
fi
if ss -ltn 2>/dev/null | grep -q ':11311'; then
  fail=1; echo "FAIL: port 11311 still LISTENing (rosmaster is up)"
fi
left=$(docker ps -aq --filter name=attacker)
[ -n "$left" ] && { fail=1; echo "FAIL: attacker containers remain: $left"; }
docker network inspect husky_lan >/dev/null 2>&1 && \
  { fail=1; echo "FAIL: docker network husky_lan still exists"; }

if [ "$fail" -ne 0 ]; then
  echo "TEARDOWN INCOMPLETE — kill the survivors above by PID and re-run."
  exit 1
fi
echo "PASS: no ROS/Gazebo processes, 11311 free, containers and husky_lan gone."
```

If it exits 1, kill the listed PIDs directly (`kill -9 <pid>`) and run the block
again. If a PID keeps coming back, something is respawning it — find its parent with
`ps -o ppid= -p <pid>` before killing again.
