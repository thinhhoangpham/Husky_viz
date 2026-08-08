# Run the Goal-Hijack Demo — Step by Step

> For the plain GPS-navigation demo (no attacker/hijack), see RUN-GPS-NAV.md.

The operator sends a **GPS goal** (lat/lon) to the **dataset robot**; a network
attacker overhears it, then injects a **false GPS goal** and the robot diverts to
the attacker's target instead. Open a separate terminal for each step. Do them in
order.

> **Reset between runs (read first):** the dataset robot has **no** ready-made
> teleport-reset helper yet. `reset-robot.py` is for the **stock** robot only (it
> teleports to `38.26, 1.25`, the stock spawn — **not** the dataset spawn
> `45.64, 0.02`), so it is **wrong** for this demo. Until a dataset reset helper
> exists, the simplest reliable reset is to **restart the world+robot** (Step 0 →
> Step 2). See "Reset the robot between runs" below.

---

## Setup (one-time, per machine)

Same as RUN-GPS-NAV.md — the demo uses the **DATASET robot**, not the stock Husky:

- Its **dual-EKF + GPS localization** comes from the dataset `husky_control`
  package, and its **full sensor suite** (Ouster OS1-64 GPU lidar + GPS) from the
  dataset `husky_description`.
- These resolve via the catkin overlay at `~/husky_overlay_ws`, which symlinks the
  dataset packages from `natural_environments_ros_opt` into `~/husky_overlay_ws/src`
  and must be built.

> **#1 GOTCHA — the lidar plugin (read this loudly):** the Ouster Gazebo plugin is
> **built from dataset source** into
> `~/husky_overlay_ws/devel/lib/libgazebo_ros_ouster_gpu_laser.so`. `gzserver`
> **must** have `GAZEBO_PLUGIN_PATH` pointing at `~/husky_overlay_ws/devel/lib`, or
> the lidar publishes **NO** `/os0_cloud_node/points` (only `/os0_cloud_node/imu`).
> With no point cloud, move_base sees no obstacles and cannot plan. Step 2 exports
> this — do not skip it.

---

### Step 0 — kill any old sim (only for a truly fresh start)

You only need this the very first time, or if the sim is in a broken state — a
normal reset does the same thing (see the note at the top).

```bash
# Kill the roslaunch parents FIRST so nothing respawns, then give them a moment.
pkill -9 -f 'bin/roslaunch' || true
sleep 1

# Every sim process pattern. On this box the ROS nodes are children of
# `systemd --user`, not of roslaunch, so we pattern-kill each one directly.
SIM_PATTERNS='gzserver gzclient gazebo rosmaster roscore move_base ekf_localization navsat robot_state_publisher twist_mux create_park add_husky load-park controller_manager spawn_model robot_localization'
for p in $SIM_PATTERNS; do pkill -9 -f "$p" || true; done

# Verify-and-retry loop: pattern-kills can miss systemd-user children (they get
# reparented / race the kill), so re-check with pgrep and kill any survivor by
# PID directly, which always works. Repeat up to 3 times, then report status.
for attempt in 1 2 3; do
  pat="$(echo "$SIM_PATTERNS" | tr ' ' '|')"
  alive="$(pgrep -f "$pat" | grep -v "^$$\$" || true)"
  [ -z "$alive" ] && break
  echo "sim procs still alive (attempt $attempt): $alive — killing by PID"
  for pid in $alive; do kill -9 "$pid" 2>/dev/null || true; done
  sleep 1
done
pat="$(echo "$SIM_PATTERNS" | tr ' ' '|')"
alive="$(pgrep -f "$pat" | grep -v "^$$\$" || true)"
if [ -z "$alive" ]; then echo "CLEAN: all sim procs down"; else echo "WARNING: still alive: $alive"; fi

# Remove leftover attacker/operator containers from previous runs
docker rm -f $(docker ps -aq --filter name=attacker --filter name=operator) 2>/dev/null || true
# Purge any leftover/ghost ROS nodes from killed attacker/operator runs
# (a ghost publisher on /move_base/goal preempts goals and makes markers vanish).
yes | rosnode cleanup 2>/dev/null || true
```

---

### Step 1 — create the network

```bash
# Force-disconnect any attached containers, then remove husky_lan (no-op if absent)
if docker network inspect husky_lan >/dev/null 2>&1; then
  for c in $(docker network inspect husky_lan --format '{{range .Containers}}{{.Name}} {{end}}'); do
    docker network disconnect -f husky_lan "$c" 2>/dev/null || true
  done
  docker network rm husky_lan 2>/dev/null || true
fi
docker network create --subnet 172.20.0.0/16 husky_lan
```

> **Why `--subnet` is required:** the rest of this guide hardcodes the gateway IP `172.20.0.1` (as `ROS_IP` / `ROS_MASTER_URI` / `ROBOT_HOST_IP`). Without pinning the subnet, Docker may assign `husky_lan` a different range on recreation (e.g. `172.21.0.0/16`, gateway `172.21.0.1`); the hardcoded `172.20.0.1` then no longer exists on any interface and `roslaunch` fails with `Unable to contact my own server at [http://172.20.0.1:...]`. Pinning `172.20.0.0/16` guarantees the gateway is always `172.20.0.1`.

---

### Step 2 — start the WORLD + dataset ROBOT (Terminal 1)

**Note:** `load-park-world.sh` now auto-discovers the lidar plugin (it searches
`GAZEBO_PLUGIN_PATH` first, then `~/husky_overlay_ws/devel/lib`), so you no longer
need to export `GAZEBO_PLUGIN_PATH` yourself — the commented line below is kept only
as an optional fallback.

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROBOT_HOST_IP=172.20.0.1
# Optional fallback — no longer required; the script finds the plugin itself:
# export GAZEBO_PLUGIN_PATH="$HOME/husky_overlay_ws/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
./load-park-world.sh
```

`load-park-world.sh` brings up the park world **and** spawns the dataset robot
(dual-EKF + GPS + GPU lidar). It **prepends** the overlay to `ROS_PACKAGE_PATH` so
`gzserver` resolves the dataset `husky_description` meshes. Wait until Gazebo shows
the park and the robot.

**Verify (Terminal 2, same `ROS_IP` / `ROS_MASTER_URI` env):**

```bash
# Lidar points — should show ~10 Hz (~15k pts). If NOTHING, the plugin was not
# found — confirm it exists at ~/husky_overlay_ws/devel/lib.
rostopic hz /os0_cloud_node/points

# Localization — shows /ekf_localization, /ekf_localization_map, /navsat_transform
rosnode list | grep -E "ekf_localization|navsat"

# Controllers — husky_joint_publisher and husky_velocity_controller both ( running )
rosrun controller_manager controller_manager list

# GPS-anchored pose — frame_id should be map
rostopic echo -n1 /odometry/filtered_map
```

---

### Step 3 — start move_base (GPS nav + obstacle avoidance) (Terminal 2)

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
roslaunch launch/move_base_gps.launch
```

This runs move_base with costmaps in the **MAP** frame (GPS-anchored, drift-free),
consuming the live 3D lidar `/os0_cloud_node/points` for obstacle marking. Global
planner `NavfnROS`, local planner `DWAPlannerROS`; `cmd_vel` → `/cmd_vel`;
odom → `/odometry/filtered_map`.

**Verify:**

```bash
rosparam get /move_base/global_costmap/global_frame       # = map
rosparam get /move_base/global_costmap/plugins            # includes the obstacles layer
```

---

### (Optional) Visualize the planning — RViz

This opens RViz preloaded to show the global route, both costmaps, the live lidar, and the goal, so you can watch the plan reshape as the robot drives.

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
rviz -d config/husky_planning.rviz
```

**Fixed Frame is `map`**; the bright green **Global plan** path is the route — watch it bend around obstacles as new lidar comes in.

---

### Step 4 — start the attacker in watch mode (Terminal 3)

```bash
cd ~/Documents/Husky_viz/attacker
export ROS_IP=172.20.0.1
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROBOT_HOST_IP=172.20.0.1
docker compose run --rm attacker ./attacker/attack.sh goal --watch
```

The attacker LISTENS — it subscribes to `/move_base/goal` to overhear the
operator's goal. Nothing is set in advance.
Wait for: `READY — now waiting for the operator's goal.`

> In `--watch` mode the attacker prints the overheard goal (e.g.
> `OVERHEARD operator goal: (1.05, -2.35)`) and **exits without injecting** — a
> two-step "see, then decide" flow. Both operator and attacker run in ros-core
> containers and convert GPS lat/lon → map via the navsat `/fromLL` service,
> falling back to local WGS84 geodesy (the normal case in the container, since
> `robot_localization` isn't installed there).

---

### Step 5 — operator sends the REAL GPS goal (Terminal 4)

```bash
cd ~/Documents/Husky_viz/operator
export ROS_IP=172.20.0.1
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROBOT_HOST_IP=172.20.0.1
docker compose up -d
docker compose exec operator bash -lc "source /opt/ros/noetic/setup.bash && ./operator/operate.py"
```

This starts the operator and waits at the `operator>` prompt — it sends **nothing** yet (RViz view at http://localhost:6080/vnc.html).

**Then, when you decide, send the goal** — at the `operator>` prompt type:

```
goal 49.9000094 8.9000327
```

Other prompt commands: `cancel`, `teleop`, `stop`, `estop`/`release`, `auto`, `status`, `quit`.

- `49.9000094 / 8.9000327` is **dataset waypoint 3** — world ≈ `(1.16, -2.40)`,
  down the trail from spawn.
- A **GREEN** marker floats above the real goal.
- The attacker (Terminal 3, from Step 4) prints
  `OVERHEARD operator goal: (1.05, -2.35)`, then exits.

Now you have SEEN the real goal. Go to Step 6 to inject the false goal.

---

### Step 6 — attacker injects the FALSE goal (Terminal 3)

```bash
cd ~/Documents/Husky_viz/attacker
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
docker compose run --rm attacker ./attacker/attack.sh goal --abs-lat 49.9001798 --abs-lon 8.9001114
```

- `49.9001798 / 8.9001114` is world ≈ `(20, -8)` — off to the side of the trail.
- A **RED** marker floats above the false goal.
- The robot **DIVERTS** from the green (real) goal to the RED (false) goal and
  drives there, avoiding obstacles. Verified: robot reached the fake goal within
  ~1 m.

> **The robot keeps "moving in place" at the goal:** the attacker **republishes**
> the fake goal continuously to stay the newest goal, so the robot keeps
> micro-adjusting once it arrives. Stop the attacker to let it settle —
> Ctrl-C its container, or:
> ```bash
> docker rm -f $(docker ps -q --filter name=attacker)
> ```

> **Injection alternatives to `--abs-lat/--abs-lon` (GPS):**
> - `--abs-x <x> --abs-y <y>` — inject a direct point in the **MAP** frame
>   (skips GPS conversion).
> - `--offset-x <dx> --offset-y <dy>` — inject a goal **offset** from the
>   overheard real goal (relative to what the attacker heard in Step 5).

**Option B — GPS slow-drift spoof (corrupts the position ESTIMATE — navigation denial, not a physical hijack)**

Instead of injecting a false *goal*, this attack fakes the robot's *GPS position*: it publishes `/navsat/fix` at the genuine ~2 Hz rate with a **slowly growing offset**, so `navsat_transform` accepts each fix as a plausible correction and the map-EKF's **fused position estimate** is dragged off-route. Critically, this corrupts only the *estimate* — the robot does **not** physically travel to the phantom location. Because move_base steers off a lie that keeps moving as the offset grows, the robot **thrashes in place** (spins and lurches) chasing a target it can never reach, and fails its mission. This is a **navigation-denial / disorientation** attack, not a "drive the robot to an attacker-chosen spot" hijack.

The operator's goal is never touched (`sent_goal == active_goal` stays consistent), and the operator does not subscribe to `/navsat/fix`. Worse for the operator: their `status`/`dist` readouts are computed from the **corrupted** fused pose (`/odometry/filtered_map`), so the display looks nominal — the robot appears to be making smooth progress while it is actually spinning in place. The spoof is stealthy on every channel the operator watches.

Like the goal-inject, the **attack runs in the attacker container** as a Tier-2 blind injector (`attack_navsat.py`): it takes one seed `/navsat/fix` sample and publishes a drifting `/navsat/fix` open-loop, reading **no** internal `robot_localization` topics. The fused-vs-anchor proof CSV is produced by a **separate host-side defender monitor** (`monitor_navsat_drift.py`), run alongside — that is the analyst observing effects, not part of the attacker.

Attacker (container-side):

```bash
cd ~/Documents/Husky_viz/attacker
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
docker compose run --rm attacker ./attacker/attack.sh navsat --drift-rate 0.5 --max-offset 15 --duration 40
```

Defender monitor (host-side, in a second terminal — legitimate, reads the robot's own estimator):

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
python3 monitor_navsat_drift.py --duration 40 --csv monitor_navsat_drift_run.csv
```

- Defaults: 2 Hz publish rate, 0.5 m/s westward drift, capped at 15 m (this cap is the **estimate** offset, not physical travel).
- Verified: the fused position **estimate** drifted ~15 m sideways as the offset ramped to the cap; `/odometry/gps` tracked the lie (it did **not** collapse to `(0,0)`), confirming `navsat_transform` accepted the drifted fixes. The robot itself did **not** travel there — it spun and lurched in place chasing the moving phantom target.
- **Why a slow drift and not a flood:** publishing fake fixes at 10–100 Hz overruns `navsat_transform`, which then emits `(0,0)` and the EKF rejects it — nothing happens. Matching the real ~2 Hz with a gradual offset keeps each fix plausible, so the lie propagates.
- Options: `--drift-rate <m/s>` (offset growth), `--max-offset <m>` (cap), `--drift-bearing <deg>` (default 90 = sideways/west), or `--drift-x <m/s> --drift-y <m/s>` (map-frame NORTH+/WEST+) to override bearing.

**Intense variant (larger, faster estimate drift):**

```bash
cd ~/Documents/Husky_viz/attacker
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
docker compose run --rm attacker ./attacker/attack.sh navsat --drift-rate 1.5 --max-offset 40 --duration 40
```

Run the host-side `monitor_navsat_drift.py` alongside (as above) to capture the fused-vs-anchor CSV.

- 1.5 m/s drift (3× the default), capped at 40 m of **estimate** offset.
- Verified on a clean sim recording ALL sensors: the fused position **estimate** climbed ~40 m sideways (e.g. `map_y` to ~+25 m in one run), tracking the injected offset ~1:1, with `/odometry/gps` following the whole way (no `(0,0)` collapse). Even at 1.5 m/s the fixes stayed plausible enough that `navsat_transform` accepted them. The robot's real down-range position (`map_x`) held roughly constant (~28 m) — **it did not translate to the phantom location.** Physically the robot spun and lurched in place: wheel odometry showed `wheel_step` near 0 with forward speed stuttering 0↔0.5, odom yaw rate spiking to ~1 rad/s repeatedly, and the compass heading swinging through all angles (−175° → −95° → +25° → +153° …). The robot **never arrives anywhere** — it thrashes in place until the mission fails. The 40 m figure is the ESTIMATE moving, not the robot.
- **Not permanent:** the corruption holds only while the attack runs. Once it stops, genuine 2 Hz GPS reels the fused estimate back to true within a few fixes — this is a live disorientation/denial, not a permanent relocation.
- **Detection:** the robot's **honest** sensors — wheel odometry and `/compass/data` — report the real physical motion (spinning and lurching in place), while the corrupted GPS-fused pose (`/odometry/filtered_map`) claims smooth sideways travel. The two **disagree**, and that contradiction is the tell. A monitor that compares wheel-odom/compass displacement + heading against the GPS-fused pose flags the spoof. Note: do **not** rely on `/imu/data` as a witness — its `angular_velocity` reads dead/~0 on this robot (confirmed even during a hard 1.2 rad/s turn), so IMU yaw-rate cannot witness the turns. Use wheel odometry + compass.

---

### Reset the robot between runs

The dataset robot (`load-park-world.sh`) has **no** ready-made teleport-reset
helper yet.

> **Do not use `reset-robot.py`** — it teleports to `38.26, 1.25`
> (`SPAWN_X`/`SPAWN_Y` from `send_mapless_goal.py`), which is the **stock** robot's
> spawn, **not** the dataset spawn `45.64, 0.02`. It is wrong for this demo.

A proper dataset reset would teleport husky to the dataset spawn pose (world
`45.64, 0.02`, yaw `2.6132`) via `/gazebo/set_model_state` and re-sync the map EKF
via the `/set_pose` service (frame `map`) — but that is **not yet scripted**. For
now, the simplest reliable reset is to **restart the world+robot**: run Step 0,
then Step 2 again, then re-run Steps 3–6.

---

### Stop everything

```bash
# Kill the roslaunch parents FIRST so nothing respawns, then give them a moment.
pkill -9 -f 'bin/roslaunch' || true
sleep 1

# Every sim process pattern. On this box the ROS nodes are children of
# `systemd --user`, not of roslaunch, so we pattern-kill each one directly.
SIM_PATTERNS='gzserver gzclient gazebo rosmaster roscore move_base ekf_localization navsat robot_state_publisher twist_mux create_park add_husky load-park controller_manager spawn_model robot_localization'
for p in $SIM_PATTERNS; do pkill -9 -f "$p" || true; done

# Verify-and-retry loop: pattern-kills can miss systemd-user children (they get
# reparented / race the kill), so re-check with pgrep and kill any survivor by
# PID directly, which always works. Repeat up to 3 times, then report status.
for attempt in 1 2 3; do
  pat="$(echo "$SIM_PATTERNS" | tr ' ' '|')"
  alive="$(pgrep -f "$pat" | grep -v "^$$\$" || true)"
  [ -z "$alive" ] && break
  echo "sim procs still alive (attempt $attempt): $alive — killing by PID"
  for pid in $alive; do kill -9 "$pid" 2>/dev/null || true; done
  sleep 1
done
pat="$(echo "$SIM_PATTERNS" | tr ' ' '|')"
alive="$(pgrep -f "$pat" | grep -v "^$$\$" || true)"
if [ -z "$alive" ]; then echo "CLEAN: all sim procs down"; else echo "WARNING: still alive: $alive"; fi

# Remove attacker/operator containers
docker rm -f $(docker ps -aq --filter name=attacker --filter name=operator) 2>/dev/null || true
# Force-disconnect any attached containers, then remove husky_lan (no-op if absent)
if docker network inspect husky_lan >/dev/null 2>&1; then
  for c in $(docker network inspect husky_lan --format '{{range .Containers}}{{.Name}} {{end}}'); do
    docker network disconnect -f husky_lan "$c" 2>/dev/null || true
  done
  docker network rm husky_lan 2>/dev/null || true
fi
```
</content>
</invoke>
