# Goal-Hijack Attack (`attack_goal.py`) — Design Spec

**Date:** 2026-08-02
**Status:** Approved design, not yet implemented.
**Parent design:** `docs/attacker-network-simulation.md` (Tier 2 rogue-publisher model; §9 realistic entry chains).
**Sibling attacks:** `attack_cmd_vel.py`, `attack_odom.py`, `attack_compass.py`, `attack_param.py`.

---

## 1. Purpose

Demonstrate **what real attackers actually do** to a ROS robot: reach the
unauthenticated ("trust-anyone") ROS graph and **publish**. Specifically, a
**mission-hijack** — the attacker overhears the operator's navigation goal and
injects its own, so `move_base` drives the robot to the *attacker's* target
instead of the operator's.

This is the realistic attack, not an academic one. Documented real-world ROS
attacks are rogue-publish/subscribe on an exposed or pivoted-into master (Shodan
finds thousands of open masters; dashboard/laptop pivots land on the same graph)
— **not** on-the-wire TCPROS sniffing or MITM, which are research/paper artifacts.
An earlier "wiretap/sniff/MITM" direction was explicitly **dropped** for this
reason (see §6).

`attack_goal.py` mirrors the existing `attack_*.py` family and runs from the
existing Tier 2 `attacker/` container.

### The attack in one line
The operator sends the robot to `(10, 0)`; the attacker overhears it, publishes
`(10, 12)`, and the robot drives *there* instead — the operator none the wiser.

---

## 2. Mechanics (informed offset-hijack)

Three steps, all pure ROS-graph access (the trust-anyone vulnerability):

1. **Recon by SUBSCRIBE (not sniff).** Subscribe to `/move_base/goal`
   (`move_base_msgs/MoveBaseActionGoal`). The master delivers every goal the
   operator publishes — plaintext, no auth. The callback caches the latest real
   target `(x, y)` and logs `overheard operator goal: (x, y)`.
   - **NOT packet sniffing.** No pcap, no TCPROS decode. A normal `rospy`
     subscriber; ROS deserializes the message. This is easier *and* more
     realistic than sniffing — the same graph access the attacker uses to publish.

2. **Wait, then derive.** A subscriber only receives goals published *after* it
   subscribes, and the operator's goal is one-shot. So the attacker **subscribes
   first and waits** until it overhears a real goal (bounded by `--timeout`). On
   timeout with nothing heard, exit with a clear message ("no operator goal seen
   — is the operator running?"). Once heard:
   `fake = (real_x + offset_x, real_y + offset_y)`.

3. **Inject by PUBLISH.** Build a `MoveBaseActionGoal` with the fake target
   (frame `odom`, orientation facing the fake target), and publish to
   `/move_base/goal`. Re-publish at `--rate` for `--duration` so the fake goal
   stays the newest goal `move_base` acts on.

### Timing / demo flow
Attacker runs **first** and lurks (subscribed, waiting); operator then sends its
mission; attacker overhears and injects. This matches the "lurking attacker
overhears the mission, then strikes" story and avoids missing the one-shot goal.

---

## 3. Components

**One new file**, repo root, alongside the other attacks:

```
attack_goal.py
```

Structure mirrors `attack_cmd_vel.py` (a class + `parse_args()` + `main()` with
`rospy.on_shutdown` cleanup):

- **Imports:** `rospy`, `move_base_msgs.msg.MoveBaseActionGoal`,
  `nav_msgs.msg.Odometry` (to log the robot's actual position),
  `math`, `argparse`, `csv`, `threading`, `time`.
  - **No `tf` dependency:** the goal only needs a yaw-only orientation quaternion,
    computed inline with `math` — `qz = sin(yaw/2)`, `qw = cos(yaw/2)`, `qx=qy=0`.
    This avoids pulling `ros-noetic-tf` into the attacker image.
- **`GoalHijackAttack` class:**
  - `__init__`: publisher to `--topic`; subscriber to `--topic` for recon;
    subscriber to `/odometry/filtered` for the robot's actual position; opens CSV.
  - `_on_real_goal(msg)`: cache `(real_x, real_y)` from
    `msg.goal.target_pose.pose.position`; log the overheard goal (once).
  - `_on_odom(msg)`: cache robot `(x, y)`.
  - `_build_goal(fx, fy)`: a `MoveBaseActionGoal`, `frame_id="odom"`,
    `header.stamp=now`, position `(fx, fy)`, orientation from
    `yaw = atan2(fy-real_y, fx-real_x)` → `orientation.z = sin(yaw/2)`,
    `orientation.w = cos(yaw/2)` (x=y=0), computed inline with `math`.
  - `run()`: wait (up to `--timeout`) for a real goal; compute fake; then loop at
    `--rate`, publishing the fake goal and writing a CSV row each tick, until
    `--duration` elapses (0 = until Ctrl-C).
  - `shutdown()`: flush/close CSV, guarded.
- **CLI (`parse_args`):**
  - `--offset-x` (float, default `0.0`)
  - `--offset-y` (float, default `12.0`) — visible sabotage; dial down for subtle
  - `--rate` (float, default `2.0` Hz)
  - `--duration` (float, default `0.0` = until Ctrl-C)
  - `--timeout` (float, default `60.0`) — max wait for a real goal
  - `--topic` (default `/move_base/goal`)
  - `--csv` (default `attack_goal_report.csv`)
  - validate `--rate > 0`, `--timeout > 0`.

### 3a. Attacker container dependency (REQUIRED change)

The existing `attacker/Dockerfile` is base `ros:noetic-ros-core` and installs only
`nmap`/`iproute2` — it **lacks `move_base_msgs`**. `attack_goal.py` imports
`move_base_msgs.msg.MoveBaseActionGoal`. So the attacker image must add:

```dockerfile
ros-noetic-move-base-msgs
```

to its `apt-get install` line (`nmap iproute2 ros-noetic-move-base-msgs`).
Analogous to the operator image's `iproute2` fix. `tf` is deliberately NOT needed
(orientation computed inline with `math`, see §3). `nav_msgs` ships with
`ros-noetic-ros-core`. Without `move_base_msgs`, `attack_goal.py` fails at import
inside the container; the other `attack_*.py` don't need it, so this is purely
additive.

---

## 4. CSV — `attack_goal_report.csv` (`--csv` overridable, like the siblings)

| column | source | meaning |
|---|---|---|
| `elapsed_time` | since attack start | shared clock |
| `real_goal_x` | overheard `/move_base/goal` | the operator's stolen target x |
| `real_goal_y` | overheard `/move_base/goal` | the operator's stolen target y |
| `fake_goal_x` | real_x + offset_x | the injected target x |
| `fake_goal_y` | real_y + offset_y | the injected target y |
| `robot_x` | `/odometry/filtered` | robot's actual position x over time |
| `robot_y` | `/odometry/filtered` | robot's actual position y over time |

Shows the injected goal vs. the operator's real goal, and the robot tracking
toward the **fake** one — the hijack, measurable. Cross-checks against the
operator's `operator_run.csv` (whose `ref_x/ref_y` is the operator's *believed*
goal `(10, 0)`), so the two files together prove operator-intent ≠ robot-path.

---

## 5. Verification (live-sim, per repo convention)

1. Host: world + `spawn-robot-idle.sh` (native robot + mapless move_base, master
   on the docker0 IP).
2. Start **`attack_goal.py`** from the `attacker/` container — it subscribes and
   waits (`overheard` not yet logged).
3. Run the **operator** (`operator/operate.py --goal-x 10 --goal-y 0`) → publishes
   the real goal.
4. **Success criteria:**
   - Attacker console: `overheard operator goal: (10.00, 0.00)` then
     `injecting fake goal: (10.00, 12.00)`.
   - Robot drives toward **(10, 12)**, NOT (10, 0) — visible in Gazebo.
   - `attack_goal_report.csv`: `robot_x/robot_y` diverge toward `fake_goal_x/y`;
     `real_goal_x/y` = (10,0), `fake_goal_x/y` = (10,12).
   - Cross-check: `operator_run.csv` shows the operator *believed* it sent (10,0)
     (`ref_x=10, ref_y=0`) while the robot went elsewhere.

---

## 6. Honest caveats (must appear in the writeup)

- **Detectable on the graph.** This is a rogue *publish*, so a defender running
  `rostopic info /move_base/goal` sees an **extra publisher** besides the operator.
  This is a true property of what real attackers do (they publish to the
  trust-anyone graph); the fully-*stealthy* variant — rewriting the operator's own
  goal packet **in flight** so no second publisher appears — is on-the-wire MITM,
  which is the **academic/research** path and was deliberately **not** built (real
  attackers reach the master and publish instead).
- **Entry is assumed, not exploited here.** `attack_goal.py` runs from the Tier 2
  `attacker/` container, which models "already reached the master." The realistic
  *entry* chain (exposed master via Shodan, or pivot from a breached
  `HuskyA300-Dashboard-main` web app, per §9) is **documented, not built** —
  standing up the dashboard as a live exploit target is a separate project.
- **Newest-goal race.** The hijack relies on `move_base` acting on the newest
  goal; the attacker re-publishes at `--rate` to stay newest. If the operator also
  re-published continuously (it does not — it sends once), the two would contend.

---

## 7. Out of scope (dropped or deferred)

- **Wiretap / sniff / pcap / TCPROS decode / MITM** — the earlier direction,
  **dropped** as academic (not what real attackers do). Recon here is a graph
  *subscribe*, not a packet sniff.
- **Dashboard-pivot exploit** — documented as the realistic entry (§6), not built.
- **Stealth (in-flight rewrite)** — the MITM upgrade; deferred, needs the
  containerized-robot topology (§10 of the parent doc).
