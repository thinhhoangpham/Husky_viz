# Realistic Operator (GCS) — Design Spec

**Date:** 2026-08-06
**Status:** Approved design, not yet implemented.
**Supersedes (in part):** `docs/superpowers/specs/2026-08-02-operator-container-design.md`
— that spec built a one-shot baseline generator; this one turns it into an
interactive ground control station (GCS). The container topology, the docker0 /
`husky_lan` wiring, and the baseline-CSV contract from that spec are **kept**;
the one-goal-then-exit behavior is **replaced** by an interactive session.
**Target sim:** the GPS park robot (`launch/move_base_gps.launch`, `map` frame),
the same robot `attack_goal.py` targets.

---

## 1. Purpose

Turn `operator/operate.py` from a fire-one-goal-and-exit baseline generator into
a **faithful ground control station**: the thing a real UGV operator sits at. A
real operator does three things — **command** a mission, **monitor** the robot,
and **intervene** when it goes wrong — and this operator does all three, at the
fidelity this simulation can honestly support.

This is NOT built around any specific attack demo. It is built to be a realistic
operator. That it happens to make the goal-hijack (`attack_goal.py`) visible, and
that some of its intervention controls genuinely override the attacker, are
*consequences* of building it faithfully — not features designed for a demo.

### What a real operator does (the reference the design is measured against)

- **Command:** send a goal / waypoints; cancel; re-send / re-task.
- **Monitor:** live map (robot pose, planned path, the goal actually being
  pursued, costmaps, sensors); mission status; health (comms link; battery/faults
  where the platform publishes them).
- **Intervene:** switch to manual driving (teleop); stop the robot; e-stop.

---

## 2. Fidelity to THIS simulation (honest mapping)

Every realistic operator action is listed here with whether the sim can represent
it faithfully. This section is the design's honesty contract — nothing is faked
silently.

| Real operator action | In this sim? | How |
|---|---|---|
| Send / re-send goal | Yes | move_base action client (GPS → `/fromLL` → map frame) |
| Cancel goal | Yes | action client `cancel_goal()` |
| Teleop (manual drive) | Yes | publish Twist to `joy_teleop/cmd_vel` (twist_mux priority 10) |
| Stop (hold still) | Yes | zero Twist on the teleop slot |
| **E-stop** | **Yes — real** | publish on the `e_stop` twist_mux **lock** (priority 255) → twist_mux blocks all cmd_vel output; release to restore. Lock message type (`std_msgs/Bool` by twist_mux convention) to be confirmed against the running graph at implementation. |
| Live map view | Yes | RViz (pose, active goal, plans, costmaps, laser) |
| Comms-link health | Yes | telemetry staleness (heartbeat age) |
| Battery / diagnostics | **No** | `/husky_status`, `/diagnostics` come from the real MCU, not Gazebo — not published here (same as `/os0_cloud_node/points`). NOT faked. Omitted. |

### The twist_mux priority ladder (verified fact — the intervention math)

From `natural_environments_ros_opt/husky/husky_control/config/twist_mux.yaml`:

```
topics:
  joy_teleop/cmd_vel            priority 10   (teleop)
  twist_marker_server/cmd_vel   priority  8   (interactive marker)
  cmd_vel                       priority  1   (move_base — and the attacker)
locks:
  e_stop                        priority 255  (lock, std_msgs/Bool)
```

Consequences the operator relies on:
- **Teleop / stop (priority 10) OUTRANK move_base and the attacker (priority 1).**
  When the operator drives, its commands win over whatever is on `/cmd_vel`.
- **E-stop (lock, priority 255) is decisive** — it blocks all cmd_vel output
  regardless of source; the robot freezes until released.
- **Goal / cancel are CONTESTED, not decisive.** They act on `/move_base/goal`,
  the *same* topic the attacker re-injects on at 2 Hz. Re-sending the true goal
  wins only momentarily (newest goal wins) before the attacker overwrites it. This
  is the real ROS 1 behavior and is not worked around.

**The honest security lesson this encodes:** an operator that sees the robot
diverge is not helpless — teleop and e-stop genuinely retake the robot via
twist_mux priority. But it cannot *cleanly* win the goal channel against an
unauthenticated peer (ROS 1 authenticates nobody; there is no way to lock the
attacker out of `/move_base/goal`). Real-world durable recovery is out-of-band
(physical e-stop, pull from network). Detection is real; clean in-band recovery of
the *goal channel* is not.

---

## 3. Architecture — ONE node, one terminal, one RViz tab

The operator is **a single ROS node** that holds all state (sent goal, current
mode, e-stop state, all subscriptions) and writes the CSV. It is NOT split into
separate nodes — there is no inter-node mode topic or pipe; mode is just a local
variable.

Two surfaces the human uses:

1. **RViz — the eyes (browser tab, always on).** Auto-launched by the container
   entrypoint (Xvfb + x11vnc + noVNC; view at `http://localhost:<port>`). A
   config file `operator.rviz` displays, in the `map` frame:
   - robot model + pose (TF / `/odometry/filtered_map`)
   - **active goal** — `/move_base/goal` (where a hijack becomes visible)
   - global plan + local plan
   - global costmap + local costmap
   - laser / pointcloud (if publishing)
   View-only. The operator never clicks-to-command (goal entry is console/CLI).

2. **Console — the hands (one terminal, plain REPL).** The node runs here. It
   prints a line **on events** (goal accepted / arrived / aborted, mode change,
   e-stop toggled) and a full snapshot **on demand** (`status`). No
   self-refreshing status line (plain mode, chosen for simplicity / low bug
   surface). The live *visual* is RViz; the console is for commanding + on-demand
   text status.

---

## 4. Console commands

Typed at the REPL in the operator terminal:

| Command | Action | Mechanism | vs. attacker |
|---|---|---|---|
| `goal <lat> <lon>` | send / re-send GPS goal | `/fromLL`→map→ action client `send_goal` | contested |
| `cancel` | stop navigating | action client `cancel_goal()` | contested |
| `teleop` | enter manual mode; drive with i/j/k/l keys | publish `joy_teleop/cmd_vel` (prio 10) | **wins** |
| `stop` | hold still | zero Twist on teleop slot (prio 10) | **wins** |
| `estop` | freeze all motion | engage `e_stop` lock (prio 255) | **decisive** |
| `release` | release e-stop | release `e_stop` lock | — |
| `auto` | resume autonomous | stop publishing teleop; move_base drives `/cmd_vel` | — |
| `status` | print a full snapshot | — | — |
| `quit` | end session, close CSV | — | — |

**Modes** (the node's `operator_mode` state): `AUTO`, `MANUAL` (teleop),
`STOPPED`, `ESTOP`. Transitions are driven by the commands above.

**Teleop key handling:** only while in `MANUAL` mode does the console read raw
keys (i/j/k/l forward/turn/back, k/space stop, q/z speed — matching
`teleop_twist_keyboard` conventions). Outside `MANUAL` the REPL is line-based.

---

## 5. Goal semantics (kept from the prior operator)

- Goal is a **GPS lat/lon** (CLI `--lat/--lon` for the initial goal, or the
  `goal <lat> <lon>` command), converted to the `map` frame via the running
  `navsat_transform` `/fromLL` service, with a **local WGS84 geodesy fallback**
  (datum 49.9 / 8.9 from `gps.urdf.xacro`) if `/fromLL` is absent — verbatim the
  existing conversion in `operate.py`.
- Sent as a `move_base` goal in the **`map`** frame (costmaps live in `map`).
- Pose read from `/odometry/filtered_map` (map frame), same as now.

---

## 6. Telemetry, health, and the CSV

### Subscriptions
- `/odometry/filtered_map` — fused pose + yaw (map frame)
- `/cmd_vel` — planner output (twist_mux `external`, priority 1)
- `/husky_velocity_controller/cmd_vel` — controller input (post-mux)
- `/move_base/goal` — the **active** goal being pursued
- `/move_base/status` — nav status
- `/move_base/feedback` — live progress (optional; status suffices)

### Comms heartbeat
`heartbeat_age` = seconds since the last message on the primary telemetry topic
(`/odometry/filtered_map`). A rising value = the operator has lost the link. This
is the only "health" signal the sim can honestly provide.

### CSV — `operator_run.csv` (append-only extension of the existing contract)

Columns **1–11 unchanged and in the same order** (the baseline contract every
attack plot depends on):

```
elapsed_time, fused_x, fused_y, fused_yaw, fused_yaw_deg,
planner_linear_x, planner_angular_z, ctrl_linear_x, ctrl_angular_z,
ref_x, ref_y,
```

Then **appended** (columns 12–16):

| # | column | source |
|---|---|---|
| 12 | `active_goal_x` | `/move_base/goal` target x (map) |
| 13 | `active_goal_y` | `/move_base/goal` target y (map) |
| 14 | `nav_status` | `/move_base/status` (e.g. ACTIVE/SUCCEEDED/ABORTED) |
| 15 | `heartbeat_age` | seconds since last telemetry |
| 16 | `operator_mode` | AUTO / MANUAL / STOPPED / ESTOP (local node state) |

`ref_x/ref_y` (cols 10–11) remain the goal the operator **sent**; `active_goal_x/y`
(12–13) are the goal move_base is **actually pursuing**. In a normal run they
match; under a goal-hijack they diverge — the hijack is thus captured in the
record without any special "attack" logic.

Because it is one node, `operator_mode` is a local variable it already holds — no
mode topic, no inter-process plumbing.

---

## 7. Session lifecycle

**Interactive: runs until `quit`** (this replaces the prior spec's "exit on
terminal move_base state / timeout"). A real operator's station stays up so it can
monitor and intervene; a one-shot exit cannot command teleop/e-stop after the
first goal resolves. An initial goal may be given at launch (`--lat/--lon`) or
sent later with `goal`. The CSV logs continuously from node start to `quit`.

---

## 8. Container & topology (kept from the prior spec, plus RViz)

- Operator container mirrors `attacker/`: own IP, joins `husky_lan` (docker0),
  `ROS_MASTER_URI` from `ROBOT_HOST_IP`, repo bind-mounted **rw** (writes CSV).
- **NEW:** the entrypoint launches the RViz visual stack (Xvfb + x11vnc + noVNC)
  so the map is available at `http://localhost:<port>` the moment the container is
  up. `operate.py` runs interactively in the container terminal.
- **Risk:** RViz under software GL (llvmpipe) is heavier than the current
  ros-core image; if CPU starves the Qt event loop, apply the same class of fix
  used for the Gazebo container (`LP_NUM_THREADS`, reduced Xvfb resolution — see
  the macOS Docker notes in CLAUDE.md). To be validated during implementation.
- The robot side is unchanged: host runs the GPS park sim natively; robot +
  move_base + EKFs + navsat_transform are already up before the operator connects.

### The `ROS_IP` gotcha (unchanged, still applies)
Before launching the native sim the host must `export ROS_IP=<husky_lan gw>` and
`ROS_MASTER_URI=http://$ROS_IP:11311`, else the master advertises 127.0.0.1 and
the container's topic handshakes hang.

---

## 9. Files

### Modified
- `operator/operate.py` — one-shot script → interactive single-node GCS
  (commands of §4, telemetry/CSV of §6, lifecycle of §7). Keeps the `/fromLL` +
  geodesy conversion and the map-frame goal logic.
- `operator/entrypoint.sh` — add the RViz + Xvfb + x11vnc + noVNC launch.
- `operator/Dockerfile` — add RViz + the VNC stack + `std_msgs` (e_stop lock msg).
- `operator/docker-compose.yml` — expose the noVNC port.
- `operator/README.md` — new runbook (open RViz tab; REPL commands).

### New
- `operator/operator.rviz` — the operator RViz config (§3 surface 1).

### Untouched
`attack_goal.py` and the other `attack_*.py`, `attacker/`, `send_gps_goal.py`,
`send_mapless_goal.py`, all launch files, all `config/*.yaml`.

---

## 10. Verification

- From the container, `goal <lat> <lon>` drives the GPS park robot to the point
  (`move_base` → `SUCCEEDED`), visible in the RViz browser tab, over `husky_lan`.
- `teleop` mode: keyboard drives the robot; verify (via `rostopic echo`) the
  Twist is on `joy_teleop/cmd_vel` and that it moves the robot even while a
  low-priority `/cmd_vel` publisher is active (proves the priority-10 override).
- `estop`: robot freezes (no motion) while `e_stop` is latched `true`, even with
  move_base/teleop commanding; `release` restores motion.
- `cancel`: move_base goal goes to a terminal/preempted state; robot stops
  navigating.
- `operator_run.csv` has all 16 columns; `active_goal_x/y` tracks the pursued
  goal; `operator_mode` reflects the commands issued; cols 1–11 byte-compatible
  with the existing baseline contract (an existing attack plot still consumes it).
- RViz shows pose, active goal, plans, costmaps updating live.
- No file listed under §9 "Untouched" is modified.

---

## 11. Explicitly out of scope

- Battery / diagnostics display (not published in sim — §2).
- Click-to-goal in RViz (goal entry is console/CLI; RViz is view-only).
- Self-refreshing TUI status line (plain REPL chosen; TUI is a possible later
  enhancement).
- Any automated "attack detected!" alarm — detection is emergent (the human sees
  the active goal / path change in RViz; the divergence is recorded in the CSV).
- Multi-robot, authentication/roles, waypoint sequences (single goal at a time).
- Out-of-band recovery modeling (physical e-stop hardware, network cut) beyond the
  real in-sim `e_stop` twist_mux lock.
