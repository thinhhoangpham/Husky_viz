# Operator Container Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a remote ROS operator — a Docker container with its own IP that sends one move_base goal to the natively-run stock Husky, watches telemetry, and exports a normal-baseline CSV — plus a new host-side script that spawns the robot into an already-running world and idles.

**Architecture:** Three decoupled pieces. (1) The world loads robot-less (existing `load-park-stock-husky.sh`). (2) A NEW `spawn-robot-idle.sh` spawns the stock Husky + mapless move_base by **reusing `send_mapless_goal.py`'s bring-up functions**, then idles (no goal). (3) A NEW `operator/` container (mirroring `attacker/`) sends one odom-frame goal, subscribes to telemetry, logs to console, and writes an 11-column baseline CSV. The operator does NO robot bring-up — it is a pure remote peer over docker0.

**Tech Stack:** ROS 1 Noetic, Python 3 (`rospy`, `actionlib`, `move_base_msgs`, `tf.transformations`), Docker + docker-compose, bash. Base image `ros:noetic-ros-core`.

## Global Constraints

- **No ground truth.** No `/gazebo/model_states`, no `gazebo_msgs`, no `/gazebo/get_model_state` for pose. Position/heading come from `/odometry/filtered` only. (Project hard rule, `CLAUDE.md`.)
- **Stock robot topology.** The robot is the STOCK Husky (odom-frame EKF, **no GPS `/navsat/fix`, no compass `/compass/data`**). Goals and CSV are odom-frame. GPS/compass comparability is deferred — do not add those columns now.
- **Leave these files UNTOUCHED:** `send_mapless_goal.py`, `husky_auto_drive.py`, `attack_param.py`, `drive-straight.py`, everything under `attacker/`. Reuse `send_mapless_goal.py` by **import**, never by editing it.
- **Goal frame is `odom`.** `frame_id = "odom"`, matching the mapless costmaps' `global_frame`.
- **cmd_vel wiring.** move_base publishes to `/cmd_vel` (twist_mux `external` slot, priority 1); the controller input is `/husky_velocity_controller/cmd_vel`. Both are logged.
- **Container mount is `rw`** (operator writes its CSV into the repo).
- **docker0 topology.** `ROS_MASTER_URI` built from `ROBOT_HOST_IP`; container derives its own `ROS_IP` in `entrypoint.sh` (the ROS_IP gotcha).
- **Testing is live-sim integration, not unit tests.** This repo has no pytest harness; every ROS script here is verified by running it against the running sim and inspecting behavior/output. Each task's "test" is a concrete run + observation. Requires the sim world up on the host (see Task 0 preconditions).

---

## Reference: exact reusable pieces (verified in the codebase)

From `send_mapless_goal.py` — importable, do not edit:
- Module constants: `PLANNER_LAUNCH`, `SPAWN_X/Y/Z/YAW`, `ODOM_TOPIC`, `DISTANCE`, `STATUS_TEXT`.
- `yaw_of(odom)` → float yaw from an `Odometry` msg.
- `bring_up_robot()` → runs `start_robot()` → `wait_for_robot_description()` → `delete_existing_robot()` → `spawn_robot()` → `wait_for_controllers()`; returns the `control.launch` Popen.
- `start_planner()` → launches `move_base_mapless_park.launch` in its own process group; returns Popen.
- `stop_planner(proc)`, `stop_robot(proc)` → guarded teardown.
- Teardown ordering (from its `main()`): planner first, then robot, each guarded in a `finally`.

Goal construction (from its `run()`), to replicate in the operator with an explicit target point instead of "distance ahead":
```python
goal = MoveBaseGoal()
goal.target_pose.header.frame_id = "odom"
goal.target_pose.header.stamp = rospy.Time.now()
goal.target_pose.pose.position.x = gx
goal.target_pose.pose.position.y = gy
gq = quaternion_from_euler(0.0, 0.0, yaw)   # yaw = heading toward the goal
goal.target_pose.pose.orientation.x/y/z/w = gq[0..3]
client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
client.wait_for_server(rospy.Duration(60.0))
client.send_goal(goal)
```

From `attacker/` — files to mirror in shape/style: `Dockerfile`, `entrypoint.sh`, `docker-compose.yml`, `attack.sh` (bash dispatch style), `README.md` (phase-runbook style).

---

## Task 0: Host-side spawn/idle script — `spawn-robot-idle.sh` + `spawn_robot_idle.py`

Spawns the stock Husky + mapless move_base into an ALREADY-RUNNING world, then idles until Ctrl-C. Reuses `send_mapless_goal.py`'s bring-up by import.

**Files:**
- Create: `spawn_robot_idle.py` (repo root)
- Create: `spawn-robot-idle.sh` (repo root)

**Interfaces:**
- Consumes (imported from `send_mapless_goal`): `bring_up_robot()`, `start_planner()`, `stop_planner(proc)`, `stop_robot(proc)`.
- Produces: a running robot + `move_base` action server on the host master, ready for the operator. No Python API consumed by later tasks.

**Preconditions for its test:** on the host, world is up and master rebound off localhost:
```bash
ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
export ROS_IP="${ROBOT_HOST_IP}" ROS_MASTER_URI="http://${ROS_IP}:11311"
./load-park-stock-husky.sh    # world only, no robot   (in its own terminal)
```

- [ ] **Step 1: Write `spawn_robot_idle.py`**

```python
#!/usr/bin/env python3
"""Spawn the STOCK Husky + mapless move_base into an ALREADY-RUNNING park world,
then IDLE (no goal) until Ctrl-C, tearing everything down on exit.

This is the ROBOT SIDE of the operator demo: it makes the robot ready so a
REMOTE operator (operator/operator.py, a separate container) can send a goal.
It deliberately does NOT send any goal itself — that is the operator's job.

Reuses send_mapless_goal.py's bring-up verbatim (imported, that file is not
modified). Teardown order mirrors send_mapless_goal.main(): planner then robot,
each guarded so one failure cannot leak the other.
"""
import signal
import rospy
from send_mapless_goal import (
    bring_up_robot, start_planner, stop_planner, stop_robot,
)


def main():
    rospy.init_node("spawn_robot_idle", anonymous=True)
    robot = None
    planner = None
    try:
        robot = bring_up_robot()
        planner = start_planner()
        rospy.loginfo("Robot spawned + mapless move_base up. IDLE — waiting for a "
                      "remote operator goal. Ctrl-C to tear down.")
        rospy.spin()          # idle until SIGINT/rospy shutdown
    finally:
        try:
            stop_planner(planner)
        finally:
            stop_robot(robot)


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Write `spawn-robot-idle.sh`**

```bash
#!/usr/bin/env bash
# Spawn the STOCK Husky + mapless move_base into an ALREADY-RUNNING park world,
# then idle. ROBOT SIDE of the operator demo. The world must already be up
# (./load-park-stock-husky.sh in another terminal). Ctrl-C tears it all down.
set -euo pipefail
source /opt/ros/noetic/setup.bash
cd "$(dirname "$0")"
exec python3 ./spawn_robot_idle.py
```

- [ ] **Step 3: Make both executable**

Run: `chmod +x spawn-robot-idle.sh spawn_robot_idle.py`

- [ ] **Step 4: Live test — spawn + idle**

With the world up and env rebound (preconditions above), in a new terminal:
```bash
export ROS_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
export ROS_MASTER_URI="http://${ROS_IP}:11311"
./spawn-robot-idle.sh
```
Expected: logs "Husky controllers are running", "Launching planner", then
"IDLE — waiting for a remote operator goal". The robot is visible in Gazebo.
In a second terminal, verify the action server exists and it idles (no goal):
```bash
rostopic list | grep move_base            # /move_base/goal, /move_base/status, ...
rosnode list | grep move_base             # /move_base present
rostopic echo -n1 /move_base/status       # status array present, empty (no goal)
```
Expected: move_base present; no goal active. Leave it running for Task 3's test.

- [ ] **Step 5: Verify clean teardown**

Ctrl-C the script. Expected: "Shutting down planner", "Shutting down robot",
process exits, no orphaned `move_base`/`roslaunch` (`pgrep -af move_base` empty).

- [ ] **Step 6: Commit**

```bash
git add spawn_robot_idle.py spawn-robot-idle.sh
git commit -m "feat(operator): spawn-robot-idle — robot side, spawn + move_base, no goal

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 1: Operator node — `operator/operator.py`

The operator: send ONE odom-frame move_base goal to a target point, watch
telemetry to console, and write the 11-column baseline CSV. No robot bring-up.

**Files:**
- Create: `operator/operator.py`

**Interfaces:**
- Consumes: a running `move_base` action server on the master (from Task 0's `spawn-robot-idle.sh`), reachable via `ROS_MASTER_URI`. Imports `yaw_of` from `send_mapless_goal` for consistent yaw extraction.
- Produces: `operator_run.csv` (default; `--csv` overrides). CSV header exactly:
  `elapsed_time, fused_x, fused_y, fused_yaw, fused_yaw_deg, planner_linear_x, planner_angular_z, ctrl_linear_x, ctrl_angular_z, ref_x, ref_y`
- CLI: `operator.py --goal-x <float> --goal-y <float> [--csv PATH] [--timeout SEC]`; defaults `--goal-x 10 --goal-y 0`, `--csv operator_run.csv`, `--timeout 180`.

- [ ] **Step 1: Write `operator/operator.py`**

```python
#!/usr/bin/env python3
"""Remote ROS operator: send ONE move_base goal to a target (x, y) in the odom
frame, watch telemetry, and write a normal-baseline CSV.

Runs in the operator container (own IP, docker0). Assumes the robot is ALREADY
spawned and move_base is up (host-side spawn-robot-idle.sh). Does NO robot
bring-up — it is a pure remote peer.

CSV columns are the union of every BASELINE signal the repo's attack CSVs log,
on a shared elapsed_time clock, so each attack's plot can overlay its series.
Attack-injected columns (fake_yaw_deg, value_written, d_*) have no baseline and
stay in the attack CSVs.
"""
import argparse
import csv
import math
import sys
import threading

import actionlib
import rospy
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

from send_mapless_goal import yaw_of  # consistent yaw extraction; file unmodified

ODOM_TOPIC = "/odometry/filtered"
PLANNER_CMD_TOPIC = "/cmd_vel"                              # move_base output
CTRL_CMD_TOPIC = "/husky_velocity_controller/cmd_vel"      # controller input
STATUS_TEXT = {
    GoalStatus.PENDING: "PENDING", GoalStatus.ACTIVE: "ACTIVE",
    GoalStatus.SUCCEEDED: "SUCCEEDED", GoalStatus.ABORTED: "ABORTED",
    GoalStatus.REJECTED: "REJECTED", GoalStatus.PREEMPTED: "PREEMPTED",
    GoalStatus.LOST: "LOST",
}
CSV_HEADER = ["elapsed_time", "fused_x", "fused_y", "fused_yaw", "fused_yaw_deg",
              "planner_linear_x", "planner_angular_z",
              "ctrl_linear_x", "ctrl_angular_z", "ref_x", "ref_y"]


class Operator(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._odom = None          # latest Odometry
        self._planner_cmd = (0.0, 0.0)  # (linear.x, angular.z) from /cmd_vel
        self._ctrl_cmd = (0.0, 0.0)     # from controller cmd_vel
        rospy.Subscriber(ODOM_TOPIC, Odometry, self._on_odom, queue_size=1)
        rospy.Subscriber(PLANNER_CMD_TOPIC, Twist, self._on_planner, queue_size=1)
        rospy.Subscriber(CTRL_CMD_TOPIC, Twist, self._on_ctrl, queue_size=1)
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(CSV_HEADER)
        self._csv_file.flush()

    def _on_odom(self, msg):
        with self._lock:
            self._odom = msg

    def _on_planner(self, msg):
        with self._lock:
            self._planner_cmd = (msg.linear.x, msg.angular.z)

    def _on_ctrl(self, msg):
        with self._lock:
            self._ctrl_cmd = (msg.linear.x, msg.angular.z)

    def _write_row(self, elapsed):
        with self._lock:
            odom = self._odom
            plx, paz = self._planner_cmd
            clx, caz = self._ctrl_cmd
        if odom is None:
            return None
        px = odom.pose.pose.position.x
        py = odom.pose.pose.position.y
        yaw = yaw_of(odom)
        self._csv.writerow(
            ["%.3f" % elapsed, "%.4f" % px, "%.4f" % py,
             "%.4f" % yaw, "%.4f" % math.degrees(yaw),
             "%.4f" % plx, "%.4f" % paz, "%.4f" % clx, "%.4f" % caz,
             "%.4f" % self.args.goal_x, "%.4f" % self.args.goal_y])
        self._csv_file.flush()
        return (px, py, yaw)

    def run(self):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server ...")
        if not client.wait_for_server(rospy.Duration(60.0)):
            rospy.logerr("move_base action server not available.")
            return 1

        # Heading toward the goal from the current pose, so the robot faces its
        # target on arrival (same convention as send_mapless_goal's goal yaw).
        start = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=30.0)
        sp = start.pose.pose.position
        gyaw = math.atan2(self.args.goal_y - sp.y, self.args.goal_x - sp.x)
        gq = quaternion_from_euler(0.0, 0.0, gyaw)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "odom"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.args.goal_x
        goal.target_pose.pose.position.y = self.args.goal_y
        goal.target_pose.pose.orientation.x = gq[0]
        goal.target_pose.pose.orientation.y = gq[1]
        goal.target_pose.pose.orientation.z = gq[2]
        goal.target_pose.pose.orientation.w = gq[3]

        rospy.loginfo("Sending goal (frame=odom): x=%.3f y=%.3f",
                      self.args.goal_x, self.args.goal_y)
        start_t = rospy.Time.now()
        client.send_goal(goal)

        rate = rospy.Rate(1.0)
        deadline = start_t + rospy.Duration(self.args.timeout)
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - start_t).to_sec()
            pose = self._write_row(elapsed)
            state = client.get_state()
            if pose is not None:
                dist = math.hypot(self.args.goal_x - pose[0],
                                  self.args.goal_y - pose[1])
                rospy.loginfo("state=%s pos=(%.2f, %.2f) dist_to_goal=%.2f m",
                              STATUS_TEXT.get(state, state), pose[0], pose[1], dist)
            else:
                rospy.loginfo("state=%s (no odom yet)", STATUS_TEXT.get(state, state))
            if state in (GoalStatus.SUCCEEDED, GoalStatus.ABORTED,
                         GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.LOST):
                rospy.loginfo("Final move_base state: %s", STATUS_TEXT.get(state, state))
                return 0 if state == GoalStatus.SUCCEEDED else 2
            if rospy.Time.now() > deadline:
                rospy.logwarn("Timed out after %ss; last state=%s",
                              self.args.timeout, STATUS_TEXT.get(state, state))
                return 3
            rate.sleep()
        return 0

    def shutdown(self):
        if self._csv_file and not self._csv_file.closed:
            self._csv_file.flush()
            self._csv_file.close()
            rospy.loginfo("CSV saved to %s", self.args.csv)


def main():
    p = argparse.ArgumentParser(description="Remote operator: send one move_base goal.")
    p.add_argument("--goal-x", type=float, default=10.0, dest="goal_x",
                   help="target x in the odom frame (m), default 10.0")
    p.add_argument("--goal-y", type=float, default=0.0, dest="goal_y",
                   help="target y in the odom frame (m), default 0.0")
    p.add_argument("--csv", default="operator_run.csv",
                   help="baseline CSV output path (default operator_run.csv)")
    p.add_argument("--timeout", type=float, default=180.0,
                   help="give up after this many seconds (default 180)")
    args = p.parse_args()

    rospy.init_node("operator", anonymous=True)
    op = Operator(args)
    try:
        return op.run()
    finally:
        op.shutdown()


if __name__ == "__main__":
    sys.exit(main())
```

- [ ] **Step 2: Make executable**

Run: `chmod +x operator/operator.py`

- [ ] **Step 3: Deferred to Task 3's live test**

`operator.py` runs inside the container against the master; it is verified end
to end in Task 3 (needs docker0 + a live robot). No standalone unit test exists
for this repo's ROS scripts. Confirm now only that it byte-compiles:

Run: `python3 -m py_compile operator/operator.py`
Expected: exit 0, no output.

- [ ] **Step 4: Commit**

```bash
git add operator/operator.py
git commit -m "feat(operator): operator node — one odom goal + telemetry + baseline CSV

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 2: Operator container scaffolding — Dockerfile, entrypoint, compose

Mirror `attacker/` so the operator gets its own IP on docker0 and the repo is
mounted rw for the CSV.

**Files:**
- Create: `operator/Dockerfile`
- Create: `operator/entrypoint.sh`
- Create: `operator/docker-compose.yml`

**Interfaces:**
- Consumes: `operator/operator.py` (Task 1), and the repo files (`send_mapless_goal.py`) via the `/repo` bind mount so the import in `operator.py` resolves.
- Produces: `docker compose run --rm operator ./operator/operator.py ...` runnable against the host master.

- [ ] **Step 1: Write `operator/Dockerfile`**

```dockerfile
FROM ros:noetic-ros-core

# move-base-msgs = MoveBaseAction/Goal for the operator; ros-noetic-actionlib
# is pulled in by it. tf for quaternion_from_euler; nav/geometry msgs for
# telemetry types. NO gazebo. NO attack code.
RUN apt-get update \
    && apt-get install -y --no-install-recommends \
        ros-noetic-move-base-msgs ros-noetic-actionlib ros-noetic-tf \
    && rm -rf /var/lib/apt/lists/*

# operator.py is bind-mounted with the repo at /repo at runtime.
WORKDIR /repo
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
```

- [ ] **Step 2: Write `operator/entrypoint.sh`** (verbatim shape from `attacker/entrypoint.sh`)

```bash
#!/usr/bin/env bash
# Derive this container's own IP and advertise it as ROS_IP so the remote
# master hands peers a reachable callback address (the ROS_IP gotcha).
set -euo pipefail
source /opt/ros/noetic/setup.bash

CONTAINER_IP="$(ip -4 -o addr show scope global | awk '{print $4}' | cut -d/ -f1 | head -n1)"
export ROS_IP="${CONTAINER_IP}"

echo "[operator] ROS_IP=${ROS_IP}"
echo "[operator] ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"

exec "$@"
```

- [ ] **Step 3: Write `operator/docker-compose.yml`**

```yaml
services:
  operator:
    build: .
    image: husky-operator
    # Default bridge (docker0): own IP; reaches the NATIVE host master via the
    # docker0 gateway IP passed in ROBOT_HOST_IP.
    environment:
      # Supply at runtime: ROBOT_HOST_IP=<docker0 gateway IP>. No hardcoding.
      ROS_MASTER_URI: "http://${ROBOT_HOST_IP}:11311"
    volumes:
      # Repo mounted rw so operator.py can write operator_run.csv, and so its
      # `from send_mapless_goal import ...` resolves from /repo.
      - ../:/repo
    working_dir: /repo
    stdin_open: true
    tty: true
```

- [ ] **Step 4: Make entrypoint executable + build**

```bash
chmod +x operator/entrypoint.sh
ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
export ROBOT_HOST_IP
cd operator && docker compose build
```
Expected: image `husky-operator` builds without error.

- [ ] **Step 5: Smoke-test container wiring (no robot needed)**

```bash
cd operator
docker compose run --rm operator python3 -c \
  "import rospy, actionlib, move_base_msgs.msg, nav_msgs.msg, tf.transformations; print('imports ok')"
```
Expected: prints `[operator] ROS_IP=...`, `[operator] ROS_MASTER_URI=http://<ip>:11311`, then `imports ok`.

- [ ] **Step 6: Commit**

```bash
git add operator/Dockerfile operator/entrypoint.sh operator/docker-compose.yml
git commit -m "feat(operator): container scaffolding — Dockerfile, entrypoint, compose

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 3: End-to-end live test + runbook — `operator/README.md`

Drive the robot from the container over docker0, confirm the CSV, and document
the four-step runbook.

**Files:**
- Create: `operator/README.md`

**Interfaces:**
- Consumes: everything from Tasks 0–2.
- Produces: a documented, verified end-to-end flow. No code consumed downstream.

- [ ] **Step 1: Full end-to-end run**

Terminal 1 (host): world up + env rebound:
```bash
export ROS_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
export ROS_MASTER_URI="http://${ROS_IP}:11311"
./load-park-stock-husky.sh
```
Terminal 2 (host): spawn the robot + move_base, idle:
```bash
export ROS_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
export ROS_MASTER_URI="http://${ROS_IP}:11311"
./spawn-robot-idle.sh
```
Terminal 3 (host): run the operator container:
```bash
cd operator
export ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
docker compose run --rm operator ./operator/operator.py --goal-x 10 --goal-y 0
```
Expected: console streams `state=ACTIVE pos=(...) dist_to_goal=...` decreasing,
ending `Final move_base state: SUCCEEDED`. Robot drives in Gazebo. Exit code 0.

- [ ] **Step 2: Verify the CSV**

```bash
head -1 ../operator_run.csv
wc -l ../operator_run.csv
```
Expected: header is exactly
`elapsed_time,fused_x,fused_y,fused_yaw,fused_yaw_deg,planner_linear_x,planner_angular_z,ctrl_linear_x,ctrl_angular_z,ref_x,ref_y`
and there are multiple data rows; `fused_x` climbs toward ~10 over the run; `ref_x`=10, `ref_y`=0 constant.

- [ ] **Step 3: Confirm untouched files are unchanged**

Run: `git status --porcelain send_mapless_goal.py husky_auto_drive.py attack_param.py drive-straight.py attacker/`
Expected: no output (none modified).

- [ ] **Step 4: Write `operator/README.md`**

```markdown
# Operator container (remote ROS operator)

A Docker container with its own IP that acts as a REMOTE operator against the
natively-run stock Husky: it sends ONE move_base goal (reach an odom-frame
point, then stop), watches telemetry to console, and writes a normal-baseline
CSV (`operator_run.csv`) whose columns are the union of every baseline signal
the repo's attack CSVs log — so each attack's plot can overlay a normal series.

Design: `docs/superpowers/specs/2026-08-02-operator-container-design.md`.
It contains NO attack code and does NO robot bring-up. It is the benign
counterpart of `attacker/`, and the operator↔robot wire it establishes is the
future Tier 3 target (`docs/attacker-network-simulation.md` §10).

## Phase 0 — Host prep + world (host terminal 1)

```bash
ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
echo "docker0 gateway = ${ROBOT_HOST_IP}"
export ROS_IP="${ROBOT_HOST_IP}"
export ROS_MASTER_URI="http://${ROS_IP}:11311"
./load-park-stock-husky.sh          # world only (no robot), master off localhost
```
Without `ROS_IP` the master advertises 127.0.0.1: the container connects but
topic handshakes hang.

## Phase 1 — Spawn the robot, idle (host terminal 2)

```bash
export ROS_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
export ROS_MASTER_URI="http://${ROS_IP}:11311"
./spawn-robot-idle.sh               # spawn stock husky + mapless move_base, then idle
```
Waits at "IDLE — waiting for a remote operator goal." Robot visible in Gazebo.

## Phase 2 — Build + operate (host terminal 3)

```bash
cd operator
export ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
docker compose build
docker compose run --rm operator ./operator/operator.py --goal-x 10 --goal-y 0
```
Watch the robot drive to the point in Gazebo; the console streams telemetry;
`operator_run.csv` lands in the repo root. Override output with `--csv path`.

## Normal-vs-attack comparison

`operator_run.csv` is the NORMAL baseline. Each attack CSV shares the
`elapsed_time` clock; a plot picks the columns it needs (e.g. odom attack uses
`fused_x/fused_y`; param uses `ctrl_linear_x`; cmd_vel uses `planner_*`/`ctrl_*`).
Attack-injected columns (`fake_yaw_deg`, `value_written`, `d_*`) have no baseline
and stay in the attack CSVs. NOTE: this stock robot has no GPS/compass, so its
position/heading are EKF-odom — comparable in kind, not the same sensor, to the
compass/GPS attacks (those target the park-GPS robot). GPS support is deferred.
```

- [ ] **Step 5: Commit**

```bash
git add operator/README.md
git commit -m "docs(operator): four-phase runbook + verified end-to-end flow

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 4: Link the operator into the parent design docs

Add a pointer from the attacker-network-simulation doc so the operator is discoverable.

**Files:**
- Modify: `docs/attacker-network-simulation.md` (append a section near the Tier 2 "Implemented" note)

**Interfaces:**
- Consumes: nothing. Produces: cross-reference only.

- [ ] **Step 1: Append the operator pointer**

After the "## Implemented: Tier 2 attacker container" section, add:

```markdown
---

## Implemented: operator container (Tier 3 groundwork)

The §10 **operator** (setup 1 — a remote ROS command node) is implemented under
`operator/`: its own IP on docker0, sends one move_base goal to the natively-run
stock Husky, watches telemetry, and exports a normal-baseline CSV for the attack
comparisons. Robot spawn is decoupled into the host-side `spawn-robot-idle.sh`
(the world stays robot-less; the robot is spawned separately, then idles ready
for the operator). Runbook: `operator/README.md`. Design:
`docs/superpowers/specs/2026-08-02-operator-container-design.md`. It establishes
the operator↔robot wire that a future Tier 3 attacker container (C) will sit on;
the attacker-in-the-middle itself remains deferred.
```

- [ ] **Step 2: Commit**

```bash
git add docs/attacker-network-simulation.md
git commit -m "docs(attacker): link operator container from parent design doc

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Self-Review

**Spec coverage** (each spec section → task):
- §1 Purpose (goal + watch + CSV, no attack) → Tasks 1, 3.
- §2 Topology (docker0, ROS_IP gotcha) → Task 2 (entrypoint/compose), Task 3 (runbook Phase 0).
- §3 Three decoupled pieces (world / spawn / operator) → Task 0 (spawn), Task 1 (operator); world reuses existing script.
- §4a spawn-robot-idle.sh reusing send_mapless_goal.py → Task 0.
- §4b operator/ container files (rw mount) → Tasks 1, 2.
- §4c untouched files → Global Constraints + Task 3 Step 3 verifies.
- §5 goal semantics (reach odom (x,y), CLI default 10/0) → Task 1.
- §6 subscriptions + 11-column CSV → Task 1 (exact header), Task 3 Step 2 verifies.
- §6 not-logged attack-injected columns + comparability caveat → Task 1 docstring, Task 3 README.
- §7 runbook → Task 3.
- §8 verification (SUCCEEDED from container, CSV populated, no untouched-file changes) → Task 3 Steps 1–3.

**Placeholder scan:** No TBD/TODO; every code step has complete content; no "handle edge cases" hand-waving; CLI defaults are concrete (10/0).

**Type consistency:** `yaw_of(odom)` imported (same source both scripts). CSV header string identical in Task 1 `CSV_HEADER`, Task 3 Step 2, and README. `bring_up_robot`/`start_planner`/`stop_planner`/`stop_robot` names match `send_mapless_goal.py` exactly (verified). Goal fields (`goal.target_pose.header.frame_id="odom"`, `.pose.position/.orientation`) match the reference. `--goal-x/--goal-y` (dest `goal_x`/`goal_y`) used consistently across `run()`, `_write_row()`, and CLI.

**Note on testing:** deliberately live-sim integration, not pytest — matches this repo's convention (no test harness exists; all ROS scripts are verified by running against the sim). Called out in Global Constraints.
