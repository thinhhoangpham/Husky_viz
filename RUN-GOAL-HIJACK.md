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
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f gazebo
pkill -9 -f rosmaster; pkill -9 -f roscore
pkill -9 -f move_base; pkill -9 -f ekf_localization; pkill -9 -f navsat
pkill -9 -f robot_state_publisher; pkill -9 -f twist_mux; pkill -9 -f create_park; pkill -9 -f load-park
sleep 2
# Remove leftover attacker/operator containers from previous runs
docker rm -f $(docker ps -aq --filter name=attacker --filter name=operator) 2>/dev/null
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

**CRITICAL:** export `GAZEBO_PLUGIN_PATH` so the lidar plugin loads (see the gotcha
above).

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROBOT_HOST_IP=172.20.0.1
export GAZEBO_PLUGIN_PATH="$HOME/husky_overlay_ws/devel/lib${GAZEBO_PLUGIN_PATH:+:$GAZEBO_PLUGIN_PATH}"
./load-park-world.sh
```

`load-park-world.sh` brings up the park world **and** spawns the dataset robot
(dual-EKF + GPS + GPU lidar). It **prepends** the overlay to `ROS_PACKAGE_PATH` so
`gzserver` resolves the dataset `husky_description` meshes. Wait until Gazebo shows
the park and the robot.

**Verify (Terminal 2, same `ROS_IP` / `ROS_MASTER_URI` env):**

```bash
# Lidar points — should show ~10 Hz (~15k pts). If NOTHING, GAZEBO_PLUGIN_PATH
# was not set — see the gotcha at the top.
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
docker compose run --rm operator ./operator/operate.py --lat 49.9000094 --lon 8.9000327
```

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
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f gazebo
pkill -9 -f rosmaster; pkill -9 -f roscore
pkill -9 -f move_base; pkill -9 -f ekf_localization; pkill -9 -f navsat
pkill -9 -f robot_state_publisher; pkill -9 -f twist_mux; pkill -9 -f create_park; pkill -9 -f load-park
# Remove attacker/operator containers
docker rm -f $(docker ps -aq --filter name=attacker --filter name=operator) 2>/dev/null
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
