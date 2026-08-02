# Operator Container — Design Spec

**Date:** 2026-08-02
**Status:** Approved design, not yet implemented.
**Parent design:** `docs/attacker-network-simulation.md` (§9 operator setups, §10 Tier 3 split).
**Sibling (built):** the Tier 2 attacker container, `attacker/` +
`docs/superpowers/specs/2026-07-30-tier2-attacker-container-design.md`.

---

## 1. Purpose

Build a **remote ROS operator** as a separate Docker container with its own IP,
against the **natively-run stock Husky sim**. It models §9 "operator setup 1" (a
ROS command node on a separate host — the most faithful Tier 3 target). It does
exactly three things:

1. **Commands** — sends **one `move_base` goal**: reach a target point, then stop.
2. **Watches** — subscribes to robot telemetry and logs each update to console.
3. **Records** — exports a **normal-baseline trajectory CSV** whose columns are the
   union of every signal the repo's attack scripts log, so each attack's plot can
   pick its columns and draw a normal-vs-attack comparison.

It contains **no attack code**. Its role in the larger plan is to establish the
genuine operator↔robot **wire** (goal outbound + telemetry inbound) that a future
Tier 3 attacker container will sit on. It is independently useful now as a
"remote operator drives the robot" demonstration and as the baseline generator
for the existing attack comparisons.

### Non-goals (explicitly deferred)
- Containerizing the robot / Gazebo (robot runs natively on the host).
- The Tier 3 attacker-in-the-middle container (C) and ARP-spoof tooling.
- The compass/navsat park-GPS robot topology (this uses the **stock** robot).
- Re-pointing `attack_param.py` at a move_base parameter (parked for later).

---

## 2. Topology (mirrors the Tier 2 attacker container)

Operator container ↔ native host sim over the **docker0 bridge**, exactly like
`attacker/`:

- Host runs the sim natively (`roslaunch`, not a container here).
- The master is rebound off `localhost`: host exports
  `ROS_IP=<docker0 gw IP>` and `ROS_MASTER_URI=http://$ROS_IP:11311` **before**
  launching the sim (the "ROS_IP gotcha" — without it nmap/connection succeeds
  but topic handshakes hang).
- The operator container derives **its own** IP → exports `ROS_IP` (verbatim
  from `attacker/entrypoint.sh`), and gets `ROS_MASTER_URI` from `ROBOT_HOST_IP`.
- Goals and telemetry cross docker0. This is the "real wire."

---

## 3. Three decoupled pieces

The world, the robot, and the operator are **separately started**, on purpose:

1. **World only** — the park world loads with **no robot in it**
   (`load-park-stock-husky.sh`, already exists; it is world-only).
2. **Robot spawn (NEW)** — a new host-side script spawns the **stock** Husky into
   the already-running world, brings up mapless `move_base`, and then **idles** —
   ready to receive a goal. The robot is spawned *separately from the map*, not
   "with" it.
3. **Operator (NEW, container)** — assumes the robot is already spawned and ready;
   sends one goal, watches, writes CSV. It does **NO** robot bring-up (doing so
   would make it co-located with the robot and defeat the remote-operator model).

The operator is a pure remote peer. The robot side is pure robot bring-up. Neither
does the other's job.

---

## 4. Components

### 4a. NEW host-side robot-spawn script — `spawn-robot-idle.sh`

Spawns the robot into the already-running world and leaves it ready + idle.

- **Reuses `send_mapless_goal.py`'s bring-up functions** (`start_robot()` →
  `wait_for_robot_description()` → `delete_existing_robot()` → `spawn_robot()` →
  `wait_for_controllers()`, then launch `launch/move_base_mapless_park.launch`).
- **Stops before the goal** — it idles instead of sending a goal.
- **Leaves `send_mapless_goal.py` untouched** — the new script reuses its
  functions (import or a thin extracted helper); it does not modify that file.
- Bring-up is verified working: a live test drove a 10 m odom goal to
  `SUCCEEDED`, straight, with ≤0.03 m lateral drift over ~21 s.

### 4b. NEW operator container — `operator/` (mirrors `attacker/`)

```
operator/
  Dockerfile          # FROM ros:noetic-ros-core + ros-noetic-move-base-msgs
                      #   + geometry_msgs/nav_msgs (goal + telemetry types).
                      #   NO gazebo. NO attack code.
  entrypoint.sh       # derive container IP -> export ROS_IP  (verbatim from attacker/)
  docker-compose.yml  # docker0 bridge; ROS_MASTER_URI from ROBOT_HOST_IP;
                      #   repo bind-mounted rw at /repo (operator writes its CSV).
  operator.py         # the operator node (see §5-6).
  README.md           # runbook: host prep -> spawn-robot-idle.sh -> operator.
```

- **Mount is `rw`** (not `ro`): the operator writes `operator_run.csv` into the
  repo, exactly like the attack scripts. (Sending a goal / watching are network
  operations and do not require a writable mount; the CSV export is why `rw`.)

### 4c. Untouched
`send_mapless_goal.py`, `husky_auto_drive.py` (remains the param-attack victim),
`attack_param.py`, `drive-straight.py`, and everything under `attacker/`.

---

## 5. Goal semantics

- Operator sends **one `move_base` goal**: **reach a target point (x, y)**, then stop.
- **Frame: `odom`.** The stock robot has **no GPS/compass**, so there is no world
  frame available; the target point is an **odom-frame (x, y)**, relative to where
  the robot spawned (odom resets to spawn each run). This matches the goal frame
  that drove to `SUCCEEDED` in the live test.
- Target point is a **CLI arg** (`--goal-x --goal-y`, odom-frame metres). Default:
  `--goal-x 10 --goal-y 0` (10 m straight ahead of spawn), the point verified to
  drive to `SUCCEEDED` in the live test.
- Goal-send flow (reuses `send_mapless_goal.py`'s proven approach): wait for the
  `move_base` action server → read current pose from `/odometry/filtered` → send
  the goal in odom → wait for terminal state.

---

## 6. Telemetry watch + baseline CSV

Each log tick until the goal reaches a terminal state, the operator writes **both**
a console line (same style as `send_mapless_goal.py`'s progress stream) and a CSV
row.

### Subscriptions
- `/odometry/filtered` — fused pose + yaw (the trajectory).
- `/cmd_vel` — the **planner output** (twist_mux `external` slot, what move_base publishes).
- `/husky_velocity_controller/cmd_vel` — the **controller input** (post-mux, drives the wheels).
- `/move_base/status` — goal state (for the console line; not a CSV column).

### CSV — `operator_run.csv` (path overridable via `--csv`, like the attack scripts)

**11 columns**, the union of every *baseline* signal any repo attack CSV logs,
keyed on a shared `elapsed_time`:

| column | source | serves comparison with |
|---|---|---|
| `elapsed_time` | since goal sent | all (shared clock) |
| `fused_x` | `/odometry/filtered` pos.x | odom, imu_derail/faithful, drive-straight; compass `true_x` (in-kind) |
| `fused_y` | `/odometry/filtered` pos.y | same as above |
| `fused_yaw` | `/odometry/filtered` yaw (rad) | imu×2, husky_auto_drive, drive-straight |
| `fused_yaw_deg` | same, degrees | compass `true_yaw_deg` |
| `planner_linear_x` | `/cmd_vel` linear.x | cmd_vel `planner_linear_x` |
| `planner_angular_z` | `/cmd_vel` angular.z | cmd_vel `planner_angular_z` |
| `ctrl_linear_x` | `/husky_velocity_controller/cmd_vel` linear.x | cmd_vel `ctrl_linear_x`; param/husky_auto_drive/drive-straight `cmd_linear_x` |
| `ctrl_angular_z` | `/husky_velocity_controller/cmd_vel` angular.z | cmd_vel `ctrl_angular_z`; `cmd_angular_z` |
| `ref_x` | goal point x | odom `ref_x` |
| `ref_y` | goal point y | odom `ref_y` |

### What is intentionally NOT logged
The **attack-injected / derived** columns have no baseline value by definition and
stay in the attack CSVs: `fake_yaw_deg` (compass), `value_written` (param),
`d_from_baseline_m` / `d_yaw_from_baseline_rad` (imu). Each attack computes these
against the operator's baseline.

### Comparability caveats (honest, from the topology)
- The operator runs the **stock** robot (odom EKF, **no compass, no GPS**). So its
  yaw is **EKF-odom yaw**, and its position is the **EKF fused pose** — not
  `/compass/data` or `/navsat/fix`. The compass attack targets the **park-GPS
  robot**, a *different* sim. Comparisons against compass/GPS attacks are therefore
  **comparable in kind** (heading vs. heading, position vs. position), not the same
  sensor. This is unavoidable given the stock topology and must not be papered over.
- The naming differences (`true_x/y`↔`fused_x/y`, `ctrl_*`↔`cmd_*`) are just each
  attack's label for the same underlying signal; the plot scripts map names.

---

## 7. Runbook (in `operator/README.md`)

1. **Host prep** — derive docker0 gw IP; `export ROS_IP` + `ROS_MASTER_URI`; load
   the world (`load-park-stock-husky.sh`).
2. **Spawn the robot** — `./spawn-robot-idle.sh` — spawns the stock Husky + mapless
   move_base, then idles (robot ready, no goal).
3. **Build** — `cd operator && export ROBOT_HOST_IP && docker compose build`.
4. **Operate** — `docker compose run --rm operator ./operator/operator.py
   --goal-x <x> --goal-y <y> [--csv operator_run.csv]`. Watch the robot drive in
   Gazebo; the console streams telemetry; `operator_run.csv` lands in the repo.

---

## 8. Verification

- Robot reaches the goal (`move_base` reports `SUCCEEDED`) driven **from the
  container**, over docker0 — proving the remote-operator wire works.
- `operator_run.csv` is written with all 11 columns populated over the run.
- At least one attack's existing plot script can consume `operator_run.csv` as the
  "normal" series and overlay its attack CSV (column mapping per §6).
- No file under §4c was modified.
