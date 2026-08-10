# Run the Offline-Map Navigation Demo — Step by Step

> For the GPS-only navigation demo (live obstacles but no preloaded map), see
> RUN-GPS-NAV.md. For the goal-hijack attack demo (offline map + attacker
> injection), see RUN-GOAL-HIJACK.md.

The robot plans over a **preloaded static map** of the park (extracted from
`park.world`) and **drives to goals** using both the offline map and live
**lidar obstacle avoidance**. The flow is: world + dataset robot (Stage 1) →
move_base with the static map + live lidar (Stage 2) → send goals by name or
coordinates (Stage 3). Open a separate terminal for each step. Do them in order.

---

## Setup (one-time, per machine)

Same as RUN-GPS-NAV.md — the demo uses the **DATASET robot**, not the stock
Husky:

- Its **dual-EKF + GPS localization** comes from the dataset `husky_control`
  package, and its **full sensor suite** (Ouster OS1-64 GPU lidar + GPS) from the
  dataset `husky_description`.
- These resolve via the catkin overlay at `~/husky_overlay_ws`, which symlinks the
  dataset packages from `natural_environments_ros_opt` into `~/husky_overlay_ws/src`
  and must be built.

> **#1 GOTCHA — the lidar plugin (read this loudly):** the Ouster Gazebo plugin
> is **built from dataset source** into
> `~/husky_overlay_ws/devel/lib/libgazebo_ros_ouster_gpu_laser.so`. `gzserver`
> **must** have `GAZEBO_PLUGIN_PATH` pointing at `~/husky_overlay_ws/devel/lib`,
> or the lidar publishes **NO** `/os0_cloud_node/points` (only
> `/os0_cloud_node/imu`). With no point cloud, move_base sees no obstacles and
> cannot plan around them. Step 2 exports this — do not skip it.

---

### Step 0 — create the Docker network (once per boot)

The operator container attaches to a fixed-subnet Docker network `husky_lan` so
the gateway IP is always `172.20.0.1` — the value hardcoded as `ROS_IP` /
`ROS_MASTER_URI` / `ROBOT_HOST_IP` everywhere below. Create it before anything
else:

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

> **Why `--subnet` is required:** without pinning the subnet, Docker may assign a
> different range on recreation (e.g. `172.21.0.0/16`, gateway `172.21.0.1`); the
> hardcoded `172.20.0.1` then exists on no interface and `roslaunch` fails with
> `Unable to contact my own server at [http://172.20.0.1:...]`. Pinning
> `172.20.0.0/16` guarantees the gateway is `172.20.0.1`.

---

### Step 1 — start the world + dataset robot (Terminal 1)

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROBOT_HOST_IP=172.20.0.1
./load-park-world.sh
```

`load-park-world.sh` brings up the park world **and** spawns the dataset robot
(dual-EKF + GPS + GPU lidar). It **prepends** the overlay to `ROS_PACKAGE_PATH`
so `gzserver` resolves the dataset `husky_description` meshes. Wait until Gazebo
shows the park and the robot.

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

**Map-frame EKF convergence:** The dataset robot runs a **map-frame EKF** (not
the odom-frame one used in GPS-Nav) that fuses GPS (position) + compass (heading)
to anchor the robot's pose in the world frame. On startup, the
map→odom→base_link TF chain takes ~30–60 s to converge because GPS needs a valid
fix. Watch `/odometry/filtered_map` with `rostopic echo` or just observe RViz;
you will see the pose stabilize. Until then, move_base's global plan may be off.

> **Compass-heading fix (required for correct lidar registration):** the map EKF
> (`natural_environments_ros_opt/husky/husky_control/config/localization_map.yaml`)
> now fuses `/compass/data` yaw as its absolute heading anchor (and no longer
> fuses the 90°-mis-mounted `/imu/data` yaw). Without this, the map-frame heading
> drifts under wheel scrub — verified ~36° off — which **rotates the lidar point
> cloud in the map frame** (red points swing when the robot turns) and mis-places
> obstacles. With the fix, EKF heading tracks the compass within ~2° through a
> turn and lidar obstacles stay fixed on the objects. If you reset this config,
> re-apply the fix or turning will de-register the lidar.

> **CLI tool note (TF lookup):** Tools like `rostopic hz` and `rosrun tf tf_echo`
> can hang on this box's software-GL Gazebo setup if there is heavy rendering.
> Instead of those, use `tf2_ros` Python to check the TF chain: `python3 -c
> "import rospy; from tf2_ros import TransformListener, Buffer; rospy.init_node('test'); tf=Buffer(); l=TransformListener(tf); print(tf.lookup_transform('map','base_link',rospy.Time())).except:"` Or just watch RViz.

---

### Step 2 — start move_base with the preloaded static map (Terminal 2)

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
roslaunch launch/move_base_gps_map.launch
```

This launches **map_server** (publishing `/map` from `maps/park_map.yaml`, the
offline map extracted from `park.world`) **PLUS** move_base with a cost map
that layers the static map (as a `StaticLayer`) under live-lidar obstacles (as
an `ObstacleLayer`), both in the **GPS-anchored map frame**. The global planner
routes over the static map and knows about all mapped objects (trees, benches,
lamp posts, ground obstacles). The local planner (DWA) respects both the planned
route and live-detected obstacles.

**Verify:**

```bash
# Static map enabled (should be true)
rosparam get /move_base/global_costmap/static_map

# Cost map layers: should show [static_map, obstacles, inflation_layer]
rosparam get /move_base/global_costmap/plugins

# DWA obstacle penalty (tuned to safe clearance with inflated obstacles)
rosparam get /move_base/DWAPlannerROS/occdist_scale      # 0.4

# Map frame (should be map, not odom)
rosparam get /move_base/global_costmap/global_frame
```

---

### (Optional) Visualize the planning — RViz

This opens RViz preloaded to show the global route, both costmaps (static map +
live lidar), the live lidar points, and the goal markers, so you can watch the
plan reshape as the robot drives and obstacles appear.

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
rviz -d config/husky_planning.rviz
```

**Fixed Frame is `map`**; the bright **green Global plan** path is the route —
watch it bend around obstacles as new lidar comes in. The **gray costmap** shows
the preloaded map (ground level), and the **red/orange overlay** shows live
lidar-detected obstacles.

**To also see the raw static map layer only:**

- Add → By topic → `/map` → Map (set color invert to see white grid on gray).

**RViz launch gotcha on this box (non-desktop shell):** RViz needs the dataset
overlay sourced and the correct display:

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
source /opt/ros/noetic/setup.bash
source ~/husky_overlay_ws/devel/setup.bash
export ROS_PACKAGE_PATH="/home/thinh/Documents/Husky_viz/natural_environments_ros_opt:$ROS_PACKAGE_PATH"
export DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority
rviz -d config/husky_planning.rviz
```

---

### Step 3 — send goals (Terminal 3 or 4)

Three ways to send goals — all in the **map** frame (world coordinates, no frame
conversion):

#### Option A — RViz "2D Nav Goal"

Click the "2D Nav Goal" button (arrow icon) in RViz, then click a point on the
map. move_base receives it immediately as an action goal.

#### Option B — Operator prompt + container-side RViz (the intended operator view)

```bash
cd ~/Documents/Husky_viz/operator
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
docker compose up -d
docker compose exec operator bash -lc "source /opt/ros/noetic/setup.bash && ./operator/operate.py"
```

`docker compose up -d` starts the operator container, which (with the default
`OPERATOR_RVIZ=1`) launches its **own RViz inside the container** (Xvfb +
noVNC), preloaded with `operator/operator.rviz` — the robot model, lidar,
costmaps, global/local plan, and active goal. **Open the operator's view in a
browser at http://localhost:6080/vnc.html** (Fixed Frame `map`). This is the
operator-side visualization — you do NOT need the host RViz from the previous
section for the operator flow. Set `OPERATOR_RVIZ=0 docker compose up -d` if you
want the REPL only.

> The container reaches the native ROS master over `husky_lan` (Step 0). TF and
> all `/move_base/*` + `/os0_cloud_node/points` topics flow into the container,
> so the RViz view mirrors what the robot sees. The RobotModel display needs the
> `robot_description` URDF (published on the master by `load-park-world.sh`); if
> the robot mesh does not render, the pose axes / lidar / costmaps still do.

The `docker compose exec ... operate.py` line then opens the `operator>` prompt.
At the prompt, send goals using one of:

```
goal <lat> <lon>          # GPS lat/lon (in degrees) — converted to map frame
goal xy <x> <y>           # Direct map-frame metres (skips GPS conversion)
goal <name>               # Named place from maps/park_places.yaml
```

Examples:

```
goal 49.9000094 8.9000327   # Dataset waypoint 3
goal xy 5.0 -3.5            # Direct map metres
goal bench                  # Named location (must match a name in maps/park_places.yaml exactly)
```

**Named goal lookup and snapping:**

- `goal <name>` looks up the name in `maps/park_places.yaml` **exactly** (case
  and spelling must match).
- **Automatic snapping to free space:** if the looked-up goal coordinate falls
  on or inside an obstacle's inflated footprint (costmap cost ≥ some threshold),
  move_base **snaps** the goal to the nearest free cell within ~1 m. This allows
  you to use "Goal: bench" even if the exact bench location is technically
  inside the 3D mesh — the goal is moved to a reachable perimeter automatically.
- Available named goals: read `maps/park_places.yaml` directly (e.g.
  `grep -E '^[a-z]' maps/park_places.yaml`). There is no `goal list` command.

Other prompt commands: `cancel`, `teleop`, `stop`, `estop`/`release`, `auto`,
`status`, `quit`.

---

## Verified Behavior

All tests below were performed on a clean sim and confirmed working.

### Route planning over static map

- Sent goal to `bench` (world x ≈ -8 m, y ≈ 20 m). Global plan routed from spawn
  (~45 m, 0 m) toward the bench, negotiating static map obstacles (~44 m run).
- Plan converged in ~5 s, local plan followed the global route smoothly.

### Named goal lookup + auto-snap

- Sent `goal bench`. Operator resolved the name to map coordinates (−8.1, 20.3).
- Goal was within the bench mesh inflated footprint; move_base snapped it to a
  free cell ~1.5 m away.
- Robot drove to the snapped goal and stopped (within ±0.2 m).

### Live obstacle avoidance

- **Test:** spawned a 1 m × 1 m × 1.5 m box in the middle of the robot's path
  (not in the preloaded map).
- **Result:** lidar detected it within 1–2 s. move_base recomputed the global plan
  to route around it. DWA local planner kept **~2 m clearance** from the obstacle
  (due to `occdist_scale=0.4` and `inflation_radius: 0.5` m), with no contact.
  Robot continued to the goal via the new path.

  > This clearance required a fix: the `DWAPlannerROS` block was originally
  > missing all trajectory-scoring params, so DWA ran with the default
  > `occdist_scale=0.01` (near-zero obstacle avoidance) and **grazed** obstacles.
  > Adding `occdist_scale: 0.4` + the scoring/sim params gave it a safe berth.

---

## How to Regenerate the Static Map

If you modify `park.world` (add/remove/move obstacles), regenerate the map:

```bash
cd ~/Documents/Husky_viz
python3 -m map_tools.extract_park_map
```

This regenerates:
- `maps/park_map.pgm` — the occupancy grid image.
- `maps/park_map.yaml` — the map metadata (origin, resolution, etc.).
- `maps/park_places.yaml` — the named landmark positions extracted from
  `park.world` models.

Re-run `roslaunch launch/move_base_gps_map.launch` in Terminal 2 (or just
`rosnode kill map_server` then `rosrun map_server map_server maps/park_map.yaml`)
to reload the new map. The costmap will regenerate on the next publish cycle
(~1 s).

---

## Known Issues

### Bench location offset

The **`bench`** static-map footprint is ~1 m off the physical bench in the
Gazebo sim (visually, the drawn box is 1 m west of where the mesh is). Other
landmarks (trees, table, lamp post, trash bin) register correctly. Root cause
not yet identified; the issue may be in the mesh bounding-box, the sdf-to-yaml
coordinate transform, or a model-pose offset in `park.world`. The workaround is
to snap goals to free space (automatic during `goal bench`); if you need the
exact true position, inspect the mesh in Gazebo or add an offset to
`park_places.yaml`.

### Ground level offset in Gazebo

The **park ground is at world z ≈ 2.9 m, not z = 0.** The map frame flattens z
to 0 for the occupancy grid. If you spawn test obstacles in Gazebo and they
appear underground or invisible, check their z coordinate:

- Spawn at z ≈ 2.9 + `half_height` (e.g., z = 3.65 for a 1.5 m tall box).
- Use `/gazebo/spawn_sdf_model` or the RViz model spawner and verify with
  `/gazebo/get_model_state`.

### DWA local planner — slow final approach

On the final approach to a goal (last ~1 m), the DWA local planner can be slow
or cause minor spinny motion — a pre-existing tuning quirk of skid-steer robots.
It still arrives at the goal; the delay is ~10–30 s for the final meter,
acceptable for a lab demo. Tuning `DWAPlannerROS/scaling_speed` or
`holonomic_robot: false` parameters may help (not yet explored).

---

## Stop Everything

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

# Stop + remove the operator container (frees the 6080 noVNC port)
cd ~/Documents/Husky_viz/operator && docker compose down 2>/dev/null || true
docker rm -f $(docker ps -aq --filter name=operator) 2>/dev/null || true

# Remove the husky_lan network (disconnect any stragglers first)
if docker network inspect husky_lan >/dev/null 2>&1; then
  for c in $(docker network inspect husky_lan --format '{{range .Containers}}{{.Name}} {{end}}'); do
    docker network disconnect -f husky_lan "$c" 2>/dev/null || true
  done
  docker network rm husky_lan 2>/dev/null || true
fi

# Delete any test obstacles spawned live in Gazebo (only needed if the sim is
# still up; they do not persist across a gzserver restart). Example:
#   rosservice call /gazebo/delete_model '{model_name: obstacle_replan_test}'
```
