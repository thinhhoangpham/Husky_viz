# Husky viz

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
  operator → goals → attacker) is in `RUN-MAP-NAV.md`.
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

`drive_to_point_gps.py` — drives to one world (x, y) point using GPS + compass only, closed-loop
at 10 Hz, nothing cached between ticks.

    python3 drive_to_point_gps.py X Y [--tolerance 0.5] [--max-lin 0.4] [--max-ang 0.8] [--timeout 180]

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

Also note: `/os0_cloud_node/points` does **not** publish on this machine (only
`/os0_cloud_node/imu` does), so the move_base costmaps stay empty and obstacle avoidance is inert.

---

## Security demo — Tier 2 network attacker (built 2026-07-30)

An "outside attacker" against the ROS graph, implemented as a **separate Docker
container** with its own IP (Tier 2: rogue peer that discovers + reaches the
master, then injects). Design and rationale (tiers, why no Gazebo attacker model):
`docs/attacker-network-simulation.md`; spec + plan under `docs/superpowers/`.

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
