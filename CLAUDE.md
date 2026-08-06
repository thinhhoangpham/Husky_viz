# Husky viz

## Project
- Project folder: `/Volumes/Extreme Pro/Husky viz` (external drive; the path contains spaces — always quote it).
- Contents (observed):
  - `bag files/park_1_bag/park_1.bag` (44,599,180,254 bytes ≈ 44.6 GB) with a `check_park_1.txt` sha256 checksum file
  - `bag_file_explorer.ipynb`, `data_analysis.ipynb`
  - `park_1_topic_breakdown.md`
  - several `umdhusky-data_collection-syscall-*-processed.csv` syscall trace files (columns: Epoch Time, Hours, Minutes, Seconds, Milliseconds, Node, PID, System Call, Unfinished Call, Resumed Call, Execution Time, Return Code, Arguments)
  - D3 visualization files `timearcs.html`/`timearcs.js`, `ridge-plot.html`/`ridge-plot.js`, `colors.json`, `arc_dot.txt`
  - `docs/` (includes `husky_rosbag_datasets.md` and student abstracts/reports)
  - `Husky-main/` and `HuskyA300-Dashboard-main/` source trees
  - `park_1_lidar_trajectory.gif`
- `dataset.txt` contains: https://www.uma.es/robotics-and-mechatronics/info/132852/negs-ugv-dataset
- macOS on Apple Silicon (arm64). The external drive generates AppleDouble `._*` metadata files.

## The bag — verified fact
- `park_1.bag` is a **ROS 1** bag: the file header is literally `#ROSBAG V2.0`, and its message types use ROS 1 naming (`gazebo_msgs/ModelStates`, `geometry_msgs/PoseWithCovariance`) with no `/msg/` infix.
- **Discrepancy:** `park_1_topic_breakdown.md` claims the bag is "a ROS 2 Gazebo simulation" and cites `sensor_msgs/msg/...` naming as the tell. That ROS 2 claim is incorrect — verified against the file bytes. Its other claim, that the data is simulator-generated rather than a real robot (based on `/gazebo/model_states`), is consistent with the contents.
- Because it is ROS 1, it plays natively under Noetic with no conversion.
- `bag_file_explorer.ipynb` reads the bag with the `rosbags` Python library (pure Python) — it does NOT need the Docker container or a ROS install.

## Docker simulation environment
- **Build context lives at `~/husky-docker/` (internal SSD), NOT in the project folder.** Reason: Docker's build-context sender fails on the external drive with `failed to xattr .../._Dockerfile: operation not permitted`. A `.dockerignore` does not fix this — the failure happens while loading the build definition, before ignore rules apply. This is why the context was moved.
- Files there: `Dockerfile`, `entrypoint.sh`, `docker-compose.yml`, `.dockerignore`, `README.md`.
- Image: `husky-docker-husky:latest`. Base `ros:noetic-robot`, native `linux/arm64` (no emulation). `ros-noetic-desktop-full` plus `ros-noetic-husky-simulator`, `husky-desktop`, `husky-viz` — all available as arm64 Focal debs.
- GUI: Xvfb + fluxbox + x11vnc + noVNC. Browser at http://localhost:6080/vnc.html. Chosen because macOS Docker has no GPU passthrough, so all rendering is software (llvmpipe); noVNC keeps GL inside the container and ships only pixels, which beats XQuartz/X11 forwarding for 3D.
- Mount: project folder bind-mounted read-write at `/workspace`. Bag path inside the container: `/workspace/bag files/park_1_bag/park_1.bag`.
- Docker Desktop file sharing must include `/Volumes/Extreme Pro`.
- **Avoid `docker compose down` — prefer `docker compose stop`/`start`.** `down` removes the container and leaves a stale mount entry in the Docker daemon; the next `up` then fails with `error while creating mount source path '/host_mnt/Volumes/Extreme Pro/Husky viz': mkdir /host_mnt/Volumes/Extreme Pro: file exists`, even though the drive is still mounted on the host.
- Recovery: **only a Docker Desktop restart clears it.** `docker compose rm -f` followed by `up` does NOT work — the stale entry lives in the daemon's mount table, not the container definition. Note a Docker restart also stops unrelated containers (e.g. `n8n`, `parquet-api`), which do not auto-restart.

## Performance fix (important)
- Symptom: Gazebo's GUI appeared frozen — menus did not respond to clicks, though the scene still rendered and VNC was healthy (1 ms latency).
- Cause: `gzclient` consumed **756% CPU** with load average **22.69** on the 8-core VM; llvmpipe spawns one render thread per core and starved Qt's event loop.
- Fix: `LP_NUM_THREADS=4` (set in the `environment:` block of `docker-compose.yml`) and Xvfb reduced to `1280x720` (in `entrypoint.sh`).
- Result, measured: gzclient 756% → **107%**, container 717% → **248%**, CPU idle 25.5% → **84.3%**, load 22.69 → **6.44**. Menus became responsive.
- Note: setting the env var in `entrypoint.sh` alone is NOT sufficient — `docker compose exec` shells do not inherit the entrypoint's exports, and that is where `roslaunch`/gzclient are started. It must be in the compose `environment:` block.

## Usage
- Start everything: `"/Volumes/Extreme Pro/Husky viz/start-sim.sh"` (brings up the container and launches the sim in the foreground; Ctrl-C stops it).
- Shut down everything: `"/Volumes/Extreme Pro/Husky viz/stop-sim.sh"` — SIGINTs the sim inside the container, then runs a full `docker compose down`, then probes whether the bind mount survived and warns if a Docker Desktop restart is needed. Because it uses `down`, expect to need that restart (see the Docker section above); for a lighter stop that avoids the trap entirely, use `docker compose stop` instead.
- **Do not run `start-sim.sh` while a sim is already running.** The duplicate nodes kill `robot_state_publisher` and the controller spawner dies mid-way, leaving `husky_velocity_controller` in state `initialized` rather than `running` — the robot then silently ignores all `cmd_vel` input and teleop appears dead. Check with `rosrun controller_manager controller_manager list`; both `husky_joint_publisher` and `husky_velocity_controller` must read `( running )`.
- Manual launch: `cd ~/husky-docker && docker compose exec husky bash`, then `source /opt/ros/noetic/setup.bash && export DISPLAY=:1 && roslaunch husky_gazebo husky_playpen.launch`. Lighter alternative world: `husky_empty_world.launch`.
- `roscore` is started by the container entrypoint, so `roslaunch` attaches to it rather than starting its own.
- Control the robot (second terminal): `rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=/kb_teleop/cmd_vel` — keys `i`/`,` forward/back, `j`/`l` turn, `k` stop, `q`/`z` speed. GUI alternative: `rosrun rqt_robot_steering rqt_robot_steering` with topic `/kb_teleop/cmd_vel`.
- Use `/kb_teleop/cmd_vel` rather than `/cmd_vel`: Husky runs `twist_mux`, which arbitrates inputs by priority; `/kb_teleop/cmd_vel` is the keyboard slot. Available cmd_vel topics: `/cmd_vel`, `/husky_velocity_controller/cmd_vel`, `/joy_teleop/cmd_vel`, `/kb_teleop/cmd_vel`, `/twist_marker_server/cmd_vel`.
- Play the bag: `rosbag play "/workspace/bag files/park_1_bag/park_1.bag"` (not yet exercised this session).

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
- **Sim runs NATIVELY on this Linux box** (`roslaunch`, not a container here —
  unlike the macOS `~/husky-docker` setup in the Docker section above). The
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
