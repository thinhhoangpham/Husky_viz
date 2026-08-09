# Run the GPS-Navigation Demo — Step by Step

The operator sends a GPS goal (lat/lon); the dataset robot drives to it using
GPS-anchored localization and 3D-lidar obstacle avoidance. **No attacker, no
hijack** — for that demo see `RUN-GOAL-HIJACK.md`.

Open a separate terminal for each step. Do them in order.

> **VERIFIED WORKING end-to-end.** The robot drives ~44 m down the trail,
> weaving around trees (visible steering), and stops on the GPS goal within ~1 m.
> Position stays GPS-accurate throughout — the map frame tracks true world within
> ~0.1 m, with no odom drift.

---

## Setup (one-time, per machine)

The demo uses the **DATASET robot**, not the stock Husky:

- Its **dual-EKF + GPS localization** comes from the dataset `husky_control`
  package, and its **full sensor suite** (Ouster OS1-64 GPU lidar + GPS, sensor
  arch) from the dataset `husky_description`.
- These resolve via the catkin overlay at `~/husky_overlay_ws`, which symlinks the
  dataset packages — `husky_control`, `husky_description`, `natural_environments`,
  `ouster_gazebo_plugins` — from `natural_environments_ros_opt` into
  `~/husky_overlay_ws/src`. On a fresh machine those symlinks plus a catkin build
  are required. (Not a full build guide — just know they must exist and be built.)

> **#1 GOTCHA — the lidar plugin (read this loudly):** the Ouster Gazebo plugin
> is **built from dataset source** (`natural_environments_ros_opt/ouster_example`)
> into `~/husky_overlay_ws/devel/lib/libgazebo_ros_ouster_gpu_laser.so`.
> `gzserver` **must** have `GAZEBO_PLUGIN_PATH` pointing at that directory, or the
> lidar publishes **NO** `/os0_cloud_node/points` (only `/os0_cloud_node/imu`).
> With no point cloud, move_base sees no obstacles and cannot plan. Step 2 exports
> this — do not skip it.

---

### Step 0 — kill any old sim (only for a truly fresh start)

You only need this the very first time, or if the sim is in a broken state.

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f gazebo
pkill -9 -f rosmaster; pkill -9 -f roscore
pkill -9 -f move_base; pkill -9 -f ekf_localization; pkill -9 -f navsat
pkill -9 -f robot_state_publisher; pkill -9 -f twist_mux; pkill -9 -f create_park; pkill -9 -f load-park
sleep 2
yes | rosnode cleanup 2>/dev/null || true
```

> If you run these in a sandboxed shell, `pkill` may need to target PIDs directly;
> in your own terminal, `pkill -9 -f` works as written.

---

### Step 1 — create the network (only if using the operator container)

The operator talks to the master over `husky_lan`. Skip if you drive goals
another way.

```bash
docker network inspect husky_lan >/dev/null 2>&1 || docker network create --subnet 172.20.0.0/16 husky_lan
```

> **Why `--subnet` is required:** the rest of this guide hardcodes the gateway IP
> `172.20.0.1`. Without pinning the subnet, Docker may assign a different range on
> recreation and the hardcoded `172.20.0.1` then exists on no interface.

---

### Step 2 — start the WORLD + dataset ROBOT (Terminal 1)

**CRITICAL:** export `GAZEBO_PLUGIN_PATH` so the lidar plugin loads (see the
gotcha above).

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
`gzserver` resolves the dataset `husky_description` meshes (the lidar renders as a
real model, not a cube). Wait until Gazebo shows the park and the robot.

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

### Step 4 — operator sends a GPS goal (Terminal 3)

The operator sends its goal as a GPS **lat/lon** (like the dataset's
`/navigation/objetive_gps` waypoints). `operate.py` converts lat/lon → map via the
navsat `/fromLL` service, or a local WGS84 geodesy fallback when
`robot_localization` isn't in the container (the normal case for the ros-core
operator container).

```bash
cd ~/Documents/Husky_viz/operator
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
docker compose up -d
docker compose exec operator bash -lc "source /opt/ros/noetic/setup.bash && ./operator/operate.py"
```

This starts the operator and waits at the `operator>` prompt — it sends **nothing** yet (RViz view at http://localhost:6080/vnc.html).

**Then, when you decide, send the goal** — at the `operator>` prompt type:

```
goal 49.9000094 8.9000327
```

Other prompt commands: `cancel`, `teleop`, `stop`, `estop`/`release`, `auto`, `status`, `quit`.

> `49.9000094 / 8.9000327` is **dataset waypoint 3** — world ≈ `(1.16, -2.40)`,
> ~44 m down the trail from spawn. Substitute other lat/lon for other goals; keep
> the goal within the rolling costmap's reach.

- A **GREEN** marker floats above the goal (visible in Gazebo). It floats **above**
  the lidar height gate on purpose so the lidar does **not** scan it as an
  obstacle. (A ground-level marker gets marked as a lethal obstacle and blocks the
  goal — that bug is fixed in `goal_marker.py`.)
- The robot drives to the goal, planning a **curved path around** lidar-detected
  obstacles, and stops on arrival (move_base status **3**, "Goal reached").

---

## What you should see

- Robot drives ~44 m down the trail, **weaving around trees** (visible steering).
- Reaches the GPS goal within **~1 m**.
- Position stays **GPS-accurate** — the map frame tracks true world within ~0.1 m,
  no odom drift.

---

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| **No lidar points** (`/os0_cloud_node/points` silent, only `.../imu` publishes) | `GAZEBO_PLUGIN_PATH` was not exported before the world came up — see Step 2 and the top gotcha. Restart the world with it set. |
| **Robot stops a few metres short with an empty plan** | Something is marked as a **lethal obstacle at the goal** (e.g. an old ground-level marker). The floating green marker in `goal_marker.py` avoids this; check no stale ground-level marker is present. |
| **Operator `ModuleNotFoundError: robot_localization`** | Expected in the ros-core operator container — `operate.py` falls back to local WGS84 geodesy automatically. Not an error. |
| **Goal never reached / no plan, goal is far** | The goal may be **outside the rolling costmap**. Pick a closer waypoint. |

---

### Stop everything

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f gazebo
pkill -9 -f rosmaster; pkill -9 -f roscore
pkill -9 -f move_base; pkill -9 -f ekf_localization; pkill -9 -f navsat
pkill -9 -f robot_state_publisher; pkill -9 -f twist_mux; pkill -9 -f create_park; pkill -9 -f load-park
```
