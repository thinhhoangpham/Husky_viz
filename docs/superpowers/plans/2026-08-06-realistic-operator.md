# Realistic Operator (GCS) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Turn `operator/operate.py` from a one-shot baseline generator into an interactive ground control station (GCS) that commands, monitors, and intervenes against the GPS park robot, with an always-on RViz view in the container.

**Architecture:** One ROS node (`operate.py`) holds all state (sent goal, mode, e-stop, subscriptions) and writes the CSV. It runs as a plain REPL in one terminal. A second surface — RViz — is auto-launched by the container entrypoint and viewed in a browser via noVNC. The node commands `move_base` via an action client (contested `/move_base/goal`), and intervenes via twist_mux slots that outrank the attacker (teleop priority 10, e-stop lock priority 255, vs. `/cmd_vel` priority 1).

**Tech Stack:** Python 2/3 rospy (ROS Noetic), `move_base_msgs` action client, `geometry_msgs/Twist`, `std_msgs`, twist_mux, RViz, Xvfb + x11vnc + noVNC, Docker Compose.

## Global Constraints

- Target sim: GPS park robot, `map` frame (`launch/move_base_gps.launch`). Pose from `/odometry/filtered_map`.
- NO ground truth anywhere (no `/gazebo/*`, no `gazebo_msgs` for pose). Standing project rule.
- Publish goals via the `move_base` action client; publish teleop to `joy_teleop/cmd_vel` (twist_mux priority 10); e-stop via the `e_stop` twist_mux lock (priority 255).
- CSV columns 1–11 unchanged and in the same order (baseline contract): `elapsed_time, fused_x, fused_y, fused_yaw, fused_yaw_deg, planner_linear_x, planner_angular_z, ctrl_linear_x, ctrl_angular_z, ref_x, ref_y`. New columns appended only (12–16).
- Keep the existing `/fromLL` + WGS84 geodesy fallback conversion verbatim (datum 49.9 / 8.9 from `gps.urdf.xacro`).
- Do NOT modify: `attack_goal.py`, other `attack_*.py`, `attacker/`, `send_gps_goal.py`, `send_mapless_goal.py`, launch files, `config/*.yaml`.
- Interactive session: node runs until `quit` (not exit-on-terminal-state).
- One ROS node only — no second node, no mode topic, no pipe.

---

## Pre-flight: verify two runtime facts (do first, no commit)

These two facts are assumed by the plan but only confirmable against a running graph. Verify before Task 3 and Task 5; if either differs, adjust that task's message type / topic accordingly.

- [ ] **P1: e_stop lock message type.** With the GPS park sim running:
  `rostopic info /e_stop` (and `rostopic type /e_stop`). Expected: `std_msgs/Bool`. If it is `Bool`, engage = publish `data: true`, release = `data: false`. Record the actual type.
- [ ] **P2: teleop + global-plan topic names.** Run: `rostopic list | grep -E "joy_teleop|move_base.*plan|/e_stop"`. Confirm `joy_teleop/cmd_vel` exists and note the exact global/local plan topic names for the RViz config (commonly `/move_base/NavfnROS/plan` and `/move_base/DWAPlannerROS/global_plan` / `local_plan`).

---

## Task 1: GcsState — the node's mode/goal state holder (pure, unit-testable)

Extract the operator's mutable state into a small pure class so mode transitions and CSV field derivation are testable without ROS.

**Files:**
- Create: `operator/gcs_state.py`
- Test: `operator/tests/test_gcs_state.py`

**Interfaces:**
- Produces:
  - `GcsState()` — holds `mode` (str, one of `"AUTO"|"MANUAL"|"STOPPED"|"ESTOP"`, initial `"AUTO"`), `sent_goal` (`(x,y)` map tuple or `None`), `active_goal` (`(x,y)` or `None`), `nav_status` (str, initial `"NONE"`), `estop_engaged` (bool, initial `False`).
  - `set_mode(m)` — sets `mode`; raises `ValueError` on unknown mode.
  - `engage_estop()` — sets `estop_engaged=True`, `mode="ESTOP"`.
  - `release_estop()` — sets `estop_engaged=False`, `mode="AUTO"`.
  - `MODES` — the tuple `("AUTO","MANUAL","STOPPED","ESTOP")`.

- [ ] **Step 1: Write the failing test**

```python
# operator/tests/test_gcs_state.py
import pytest
from operator_pkg_shim import GcsState  # see Step 3 note

def test_defaults():
    s = GcsState()
    assert s.mode == "AUTO"
    assert s.sent_goal is None
    assert s.active_goal is None
    assert s.nav_status == "NONE"
    assert s.estop_engaged is False

def test_set_mode_valid_and_invalid():
    s = GcsState()
    s.set_mode("MANUAL")
    assert s.mode == "MANUAL"
    with pytest.raises(ValueError):
        s.set_mode("FLYING")

def test_estop_engage_release():
    s = GcsState()
    s.engage_estop()
    assert s.estop_engaged is True and s.mode == "ESTOP"
    s.release_estop()
    assert s.estop_engaged is False and s.mode == "AUTO"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd operator && python3 -m pytest tests/test_gcs_state.py -v`
Expected: FAIL — `ModuleNotFoundError` / import error.

- [ ] **Step 3: Write minimal implementation**

Import shim so tests don't depend on ROS: create `operator/tests/conftest.py` adding the operator dir to `sys.path`, and have the test import from `gcs_state` directly. Replace the test's import line with `from gcs_state import GcsState` and delete the shim reference.

```python
# operator/gcs_state.py
class GcsState(object):
    MODES = ("AUTO", "MANUAL", "STOPPED", "ESTOP")

    def __init__(self):
        self.mode = "AUTO"
        self.sent_goal = None      # (x, y) map frame
        self.active_goal = None    # (x, y) map frame
        self.nav_status = "NONE"
        self.estop_engaged = False

    def set_mode(self, m):
        if m not in self.MODES:
            raise ValueError("unknown mode: %s" % m)
        self.mode = m

    def engage_estop(self):
        self.estop_engaged = True
        self.mode = "ESTOP"

    def release_estop(self):
        self.estop_engaged = False
        self.mode = "AUTO"
```

```python
# operator/tests/conftest.py
import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd operator && python3 -m pytest tests/test_gcs_state.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add operator/gcs_state.py operator/tests/test_gcs_state.py operator/tests/conftest.py
git commit -m "feat(operator): GcsState mode/goal state holder + tests"
```

---

## Task 2: CSV row builder (pure, extends the 11-col contract to 16)

Isolate CSV header + row formatting so the column contract is tested independently of ROS.

**Files:**
- Create: `operator/gcs_csv.py`
- Test: `operator/tests/test_gcs_csv.py`

**Interfaces:**
- Consumes: `GcsState` (Task 1) for `nav_status`, `mode`, `active_goal`.
- Produces:
  - `CSV_HEADER` — list of 16 column names, cols 1–11 exactly the baseline contract, then `active_goal_x, active_goal_y, nav_status, heartbeat_age, operator_mode`.
  - `build_row(elapsed, pose, planner, ctrl, sent_goal, active_goal, nav_status, heartbeat_age, mode)` → list of 16 stringified values. `pose=(x,y,yaw_rad)`, `planner=(lx,az)`, `ctrl=(lx,az)`, `sent_goal=(x,y)`, `active_goal=(x,y)|None`. Missing values (`None`) rendered as `"nan"`. Floats formatted `%.4f`, yaw_deg derived from yaw_rad.

- [ ] **Step 1: Write the failing test**

```python
# operator/tests/test_gcs_csv.py
import math
from gcs_csv import CSV_HEADER, build_row

def test_header_contract():
    assert CSV_HEADER[:11] == [
        "elapsed_time","fused_x","fused_y","fused_yaw","fused_yaw_deg",
        "planner_linear_x","planner_angular_z","ctrl_linear_x","ctrl_angular_z",
        "ref_x","ref_y"]
    assert CSV_HEADER[11:] == [
        "active_goal_x","active_goal_y","nav_status","heartbeat_age","operator_mode"]

def test_build_row_full():
    row = build_row(
        elapsed=5.0, pose=(1.0, 2.0, math.pi/2), planner=(0.4, 0.0),
        ctrl=(0.4, 0.0), sent_goal=(10.0, 0.0), active_goal=(10.0, 12.0),
        nav_status="ACTIVE", heartbeat_age=0.2, mode="AUTO")
    assert len(row) == 16
    assert row[0] == "5.0000"
    assert row[4] == "%.4f" % 90.0           # yaw_deg
    assert row[9] == "10.0000" and row[10] == "0.0000"   # ref_x/y = sent goal
    assert row[11] == "10.0000" and row[12] == "12.0000" # active goal
    assert row[13] == "ACTIVE"
    assert row[15] == "AUTO"

def test_build_row_missing_active_goal():
    row = build_row(
        elapsed=0.0, pose=(0.0,0.0,0.0), planner=(0.0,0.0), ctrl=(0.0,0.0),
        sent_goal=None, active_goal=None, nav_status="NONE",
        heartbeat_age=1.0, mode="AUTO")
    assert row[9] == "nan" and row[11] == "nan"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd operator && python3 -m pytest tests/test_gcs_csv.py -v`
Expected: FAIL — import error.

- [ ] **Step 3: Write minimal implementation**

```python
# operator/gcs_csv.py
import math

CSV_HEADER = [
    "elapsed_time", "fused_x", "fused_y", "fused_yaw", "fused_yaw_deg",
    "planner_linear_x", "planner_angular_z", "ctrl_linear_x", "ctrl_angular_z",
    "ref_x", "ref_y",
    "active_goal_x", "active_goal_y", "nav_status", "heartbeat_age",
    "operator_mode",
]

def _f(v):
    return "nan" if v is None else "%.4f" % v

def build_row(elapsed, pose, planner, ctrl, sent_goal, active_goal,
              nav_status, heartbeat_age, mode):
    px, py, yaw = pose
    plx, paz = planner
    clx, caz = ctrl
    sx, sy = (sent_goal if sent_goal else (None, None))
    ax, ay = (active_goal if active_goal else (None, None))
    return [
        "%.4f" % elapsed, _f(px), _f(py), _f(yaw),
        _f(math.degrees(yaw)) if yaw is not None else "nan",
        _f(plx), _f(paz), _f(clx), _f(caz),
        _f(sx), _f(sy), _f(ax), _f(ay),
        nav_status, _f(heartbeat_age), mode,
    ]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd operator && python3 -m pytest tests/test_gcs_csv.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add operator/gcs_csv.py operator/tests/test_gcs_csv.py
git commit -m "feat(operator): CSV row builder extending baseline to 16 cols"
```

---

## Task 3: Intervention publishers (teleop / stop / e-stop) — thin ROS wrappers

Wrap the twist_mux intervention channels behind a small class so command handlers stay simple. ROS calls are injected so the logic is testable with fakes.

**Files:**
- Create: `operator/gcs_intervene.py`
- Test: `operator/tests/test_gcs_intervene.py`

**Interfaces:**
- Consumes: two publisher-like objects with `.publish(msg)`; `Twist` and `Bool` message factories (injected).
- Produces:
  - `Intervene(teleop_pub, estop_pub, twist_cls, bool_cls)`.
  - `drive(linx, angz)` → publishes a `Twist` with those fields on `teleop_pub`.
  - `stop()` → publishes a zero `Twist` on `teleop_pub`.
  - `engage_estop()` → publishes `bool_cls(data=True)` on `estop_pub`.
  - `release_estop()` → publishes `bool_cls(data=False)` on `estop_pub`.
- Note: e-stop message type is `std_msgs/Bool` per pre-flight P1; if P1 found otherwise, change `bool_cls` usage accordingly.

- [ ] **Step 1: Write the failing test**

```python
# operator/tests/test_gcs_intervene.py
from gcs_intervene import Intervene

class FakePub:
    def __init__(self): self.sent = []
    def publish(self, m): self.sent.append(m)

class FakeTwist:
    def __init__(self):
        self.linear = type("L", (), {"x":0.0,"y":0.0,"z":0.0})()
        self.angular = type("A", (), {"x":0.0,"y":0.0,"z":0.0})()

class FakeBool:
    def __init__(self, data=False): self.data = data

def test_drive_sets_twist_fields():
    tp, ep = FakePub(), FakePub()
    iv = Intervene(tp, ep, FakeTwist, FakeBool)
    iv.drive(0.5, -0.2)
    assert tp.sent[-1].linear.x == 0.5 and tp.sent[-1].angular.z == -0.2

def test_stop_is_zero_twist():
    tp, ep = FakePub(), FakePub()
    iv = Intervene(tp, ep, FakeTwist, FakeBool)
    iv.stop()
    assert tp.sent[-1].linear.x == 0.0 and tp.sent[-1].angular.z == 0.0

def test_estop_engage_release_bool():
    tp, ep = FakePub(), FakePub()
    iv = Intervene(tp, ep, FakeTwist, FakeBool)
    iv.engage_estop(); iv.release_estop()
    assert ep.sent[0].data is True and ep.sent[1].data is False
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd operator && python3 -m pytest tests/test_gcs_intervene.py -v`
Expected: FAIL — import error.

- [ ] **Step 3: Write minimal implementation**

```python
# operator/gcs_intervene.py
class Intervene(object):
    def __init__(self, teleop_pub, estop_pub, twist_cls, bool_cls):
        self._teleop = teleop_pub
        self._estop = estop_pub
        self._Twist = twist_cls
        self._Bool = bool_cls

    def drive(self, linx, angz):
        t = self._Twist()
        t.linear.x = linx
        t.angular.z = angz
        self._teleop.publish(t)

    def stop(self):
        self._teleop.publish(self._Twist())

    def engage_estop(self):
        self._estop.publish(self._Bool(data=True))

    def release_estop(self):
        self._estop.publish(self._Bool(data=False))
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd operator && python3 -m pytest tests/test_gcs_intervene.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add operator/gcs_intervene.py operator/tests/test_gcs_intervene.py
git commit -m "feat(operator): twist_mux intervention wrappers (teleop/stop/estop)"
```

---

## Task 4: Command parser + dispatch table (pure)

Parse a REPL line into an action + args, so the interactive loop is a thin shell over tested logic.

**Files:**
- Create: `operator/gcs_commands.py`
- Test: `operator/tests/test_gcs_commands.py`

**Interfaces:**
- Produces:
  - `parse_command(line)` → `(cmd, args)` where `cmd` in `{"goal","cancel","teleop","stop","estop","release","auto","status","quit","help","unknown","noop"}`. `args` is a list. `goal` requires two float args else `("error", ["goal needs <lat> <lon>"])`. Blank line → `("noop", [])`. Unknown verb → `("unknown", [verb])`.

- [ ] **Step 1: Write the failing test**

```python
# operator/tests/test_gcs_commands.py
from gcs_commands import parse_command

def test_goal_ok():
    assert parse_command("goal 49.9 8.9") == ("goal", [49.9, 8.9])

def test_goal_bad_args():
    cmd, args = parse_command("goal 49.9")
    assert cmd == "error"

def test_simple_verbs():
    for v in ["cancel","teleop","stop","estop","release","auto","status","quit"]:
        assert parse_command(v) == (v, [])

def test_blank_and_unknown():
    assert parse_command("   ") == ("noop", [])
    assert parse_command("frobnicate") == ("unknown", ["frobnicate"])
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd operator && python3 -m pytest tests/test_gcs_commands.py -v`
Expected: FAIL — import error.

- [ ] **Step 3: Write minimal implementation**

```python
# operator/gcs_commands.py
SIMPLE = ("cancel", "teleop", "stop", "estop", "release", "auto",
          "status", "quit", "help")

def parse_command(line):
    parts = line.strip().split()
    if not parts:
        return ("noop", [])
    verb = parts[0].lower()
    if verb == "goal":
        if len(parts) != 3:
            return ("error", ["goal needs <lat> <lon>"])
        try:
            return ("goal", [float(parts[1]), float(parts[2])])
        except ValueError:
            return ("error", ["goal args must be numbers"])
    if verb in SIMPLE:
        return (verb, [])
    return ("unknown", [verb])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd operator && python3 -m pytest tests/test_gcs_commands.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add operator/gcs_commands.py operator/tests/test_gcs_commands.py
git commit -m "feat(operator): REPL command parser + dispatch table"
```

---

## Task 5: Rewrite operate.py into the interactive GCS node (wire it all together)

Replace the one-shot `run()` with an interactive node using Tasks 1–4. Keep the `/fromLL` + geodesy conversion, subscriptions, and map-frame goal logic. This is the integration task; it is verified live (no unit test — it is glue over already-tested units, exercised in Task 8).

**Files:**
- Modify: `operator/operate.py` (replace `class Operator.run` and `main`; keep `latlon_to_map`, `fix_to_world_fallback`, `yaw_of`, the datum constants; extend `__init__` subscriptions and CSV writer to use `gcs_csv.CSV_HEADER`).

**Interfaces:**
- Consumes: `GcsState` (T1), `gcs_csv.CSV_HEADER`/`build_row` (T2), `Intervene` (T3), `parse_command` (T4).
- Produces: an `operate.py` runnable as `./operator/operate.py [--lat L --lon O] [--csv path]` presenting the REPL of §4.

- [ ] **Step 1: Add new imports + publishers/subscribers in `__init__`**

Add near the existing imports:
```python
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalID
from gcs_state import GcsState
from gcs_csv import CSV_HEADER as GCS_CSV_HEADER, build_row
from gcs_intervene import Intervene
from gcs_commands import parse_command
```
In `Operator.__init__`, after the existing subscribers, add:
```python
self.state = GcsState()
self._active_goal = None      # (x,y) from /move_base/goal
self._last_odom_wall = None   # for heartbeat
rospy.Subscriber("/move_base/goal", MoveBaseActionGoal, self._on_active_goal, queue_size=1)
rospy.Subscriber("/move_base/status", GoalStatusArray, self._on_status, queue_size=1)
self._teleop_pub = rospy.Publisher("joy_teleop/cmd_vel", Twist, queue_size=1)
self._estop_pub = rospy.Publisher("e_stop", Bool, queue_size=1, latch=True)
self._intervene = Intervene(self._teleop_pub, self._estop_pub, Twist, Bool)
```
Add imports at top for the new message types:
```python
from move_base_msgs.msg import MoveBaseActionGoal
from actionlib_msgs.msg import GoalStatusArray
```
Change the CSV header write to use `GCS_CSV_HEADER`.

- [ ] **Step 2: Add the new callbacks + heartbeat**

```python
    def _on_active_goal(self, msg):
        p = msg.goal.target_pose.pose.position
        with self._lock:
            self._active_goal = (p.x, p.y)
            self.state.active_goal = (p.x, p.y)

    def _on_status(self, msg):
        # last status in the array is the current goal's
        if msg.status_list:
            self.state.nav_status = STATUS_TEXT.get(
                msg.status_list[-1].status, str(msg.status_list[-1].status))

    def _heartbeat_age(self):
        if self._last_odom_wall is None:
            return float("nan")
        return time.time() - self._last_odom_wall
```
In the existing `_on_odom`, set `self._last_odom_wall = time.time()`.

- [ ] **Step 3: Replace `run()` with the interactive REPL + a background CSV/telemetry thread**

```python
    def run(self, initial_goal=None):
        self.client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server ...")
        self.client.wait_for_server(rospy.Duration(60.0))
        self._start_wall = time.time()
        # background telemetry/CSV writer (~2 Hz)
        self._writer = threading.Thread(target=self._telemetry_loop)
        self._writer.daemon = True
        self._writer.start()
        if initial_goal is not None:
            self._do_goal(initial_goal[0], initial_goal[1])
        self._print_help()
        while not rospy.is_shutdown():
            try:
                line = input("operator> ")
            except (EOFError, KeyboardInterrupt):
                break
            cmd, args = parse_command(line)
            if cmd == "quit":
                break
            self._dispatch(cmd, args)
        return 0

    def _telemetry_loop(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown() and not self._stop.is_set():
            self._write_row((time.time() - self._start_wall))
            rate.sleep()
```
Add `self._stop = threading.Event()` in `__init__`.

- [ ] **Step 4: Implement `_dispatch` and the goal/teleop handlers**

```python
    def _dispatch(self, cmd, args):
        if cmd == "goal":
            self._do_goal(args[0], args[1])
        elif cmd == "cancel":
            self.client.cancel_all_goals(); self.state.set_mode("AUTO")
            rospy.loginfo("CANCELLED goal")
        elif cmd == "teleop":
            self.state.set_mode("MANUAL"); self._teleop_repl()
        elif cmd == "stop":
            self._intervene.stop(); self.state.set_mode("STOPPED")
            rospy.loginfo("STOP (zero velocity)")
        elif cmd == "estop":
            self._intervene.engage_estop(); self.state.engage_estop()
            rospy.logwarn("E-STOP ENGAGED")
        elif cmd == "release":
            self._intervene.release_estop(); self.state.release_estop()
            rospy.loginfo("E-STOP RELEASED")
        elif cmd == "auto":
            self.state.set_mode("AUTO"); rospy.loginfo("AUTO mode")
        elif cmd == "status":
            self._print_status()
        elif cmd in ("help", "unknown", "error"):
            self._print_help() if cmd == "help" else rospy.logwarn(" ".join(args) or "?")

    def _do_goal(self, lat, lon):
        gx, gy, path = latlon_to_map(lat, lon)
        self._goal_x, self._goal_y = gx, gy
        self.state.sent_goal = (gx, gy)
        start = rospy.wait_for_message(ODOM_TOPIC, Odometry, timeout=30.0)
        sp = start.pose.pose.position
        gyaw = math.atan2(gy - sp.y, gx - sp.x)
        gq = quaternion_from_euler(0.0, 0.0, gyaw)
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = GOAL_FRAME
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = gx
        goal.target_pose.pose.position.y = gy
        goal.target_pose.pose.orientation.x = gq[0]
        goal.target_pose.pose.orientation.y = gq[1]
        goal.target_pose.pose.orientation.z = gq[2]
        goal.target_pose.pose.orientation.w = gq[3]
        place_goal_marker("goal_marker_real", gx, gy, "0 1 0", frame="map")
        self.client.send_goal(goal)
        self.state.set_mode("AUTO")
        rospy.loginfo("SENT goal map=(%.2f, %.2f) via %s", gx, gy, path)
```
`_teleop_repl` reads raw keys (i/j/k/l/,) until `x`/Esc, calling `self._intervene.drive(...)`; on exit call `self._intervene.stop()` and `self.state.set_mode("AUTO")`. Use `termios`/`tty` raw mode (guard with try/except so a non-tty just returns).

- [ ] **Step 5: Update `_write_row` to build the 16-col row and add `_print_status`/`_print_help`**

Rewrite `_write_row` to gather pose/planner/ctrl under `self._lock` and call `build_row(...)` with `self.state`, `self._active_goal`, `self._heartbeat_age()`, `self.state.mode`. `_print_status` prints the one-line snapshot (`state | sent | active | dist | mode | estop | link`). `_print_help` lists the §4 commands.

- [ ] **Step 6: Update `main()` for the interactive lifecycle**

```python
def main():
    p = argparse.ArgumentParser(description="Interactive GCS operator.")
    p.add_argument("--lat", type=float, default=None)
    p.add_argument("--lon", type=float, default=None)
    p.add_argument("--csv", default="operator_run.csv")
    p.add_argument("--timeout", type=float, default=180.0)
    args = p.parse_args()
    rospy.init_node("operator", anonymous=True)
    op = Operator(args)
    rospy.on_shutdown(op.shutdown)
    initial = (args.lat, args.lon) if (args.lat is not None and args.lon is not None) else None
    try:
        return op.run(initial_goal=initial)
    finally:
        op.shutdown()
```

- [ ] **Step 7: Syntax check + import check (no ROS needed)**

Run: `cd operator && python3 -c "import ast; ast.parse(open('operate.py').read()); print('OK')"`
Expected: `OK`. (Full runtime verification is Task 8.)

- [ ] **Step 8: Commit**

```bash
git add operator/operate.py
git commit -m "feat(operator): interactive GCS node (goal/cancel/teleop/stop/estop) wiring T1-T4"
```

---

## Task 6: RViz operator config

Ship the operator RViz config showing pose, active goal, plans, costmaps, laser in the `map` frame.

**Files:**
- Create: `operator/operator.rviz`

**Interfaces:**
- Consumes: topic names confirmed in pre-flight P2 (global/local plan, costmap, laser). Uses `map` as fixed frame.

- [ ] **Step 1: Create the RViz config**

Create `operator/operator.rviz` with `Fixed Frame: map` and Displays: RobotModel, TF, a `Pose`/`Marker` for the active goal (`/move_base/goal` shown via move_base's own goal, plus the existing `goal_marker_*`), Path for global plan and local plan (topic names from P2), Map for global + local costmap, LaserScan/PointCloud2 for the lidar topic. Base it on a stock Husky RViz config if present under `natural_environments_ros_opt/husky/husky_viz/rviz/` to match conventions.

- [ ] **Step 2: Validate it is well-formed YAML**

Run: `cd operator && python3 -c "import yaml; yaml.safe_load(open('operator.rviz')); print('OK')"`
Expected: `OK`.

- [ ] **Step 3: Commit**

```bash
git add operator/operator.rviz
git commit -m "feat(operator): RViz operator view config (pose/goal/plan/costmap/laser)"
```

---

## Task 7: Container — RViz + noVNC stack in the operator image

Add the visual stack to the operator container and expose the noVNC port. Entrypoint launches Xvfb + RViz + x11vnc + noVNC on startup; the REPL still runs via `docker compose run`.

**Files:**
- Modify: `operator/Dockerfile`
- Modify: `operator/entrypoint.sh`
- Modify: `operator/docker-compose.yml`
- Modify: `operator/README.md`

**Interfaces:**
- Produces: `http://localhost:6080/vnc.html` serving the operator RViz; `operate.py` REPL unchanged in behavior.

- [ ] **Step 1: Dockerfile — add RViz + VNC stack + std_msgs**

Add to the `apt-get install` list: `ros-noetic-rviz ros-noetic-std-msgs xvfb x11vnc novnc websockify fluxbox`. Keep the existing packages.

- [ ] **Step 2: entrypoint.sh — launch the visual stack, then exec the command**

Before `exec "$@"`, add (guarded by an env flag `OPERATOR_RVIZ=${OPERATOR_RVIZ:-1}` so it can be disabled):
```bash
if [ "${OPERATOR_RVIZ:-1}" = "1" ]; then
  export DISPLAY=:1
  Xvfb :1 -screen 0 1280x720x24 &
  sleep 1
  fluxbox &
  x11vnc -display :1 -forever -nopw -quiet -bg
  websockify --web=/usr/share/novnc 6080 localhost:5900 &
  rosrun rviz rviz -d /repo/operator/operator.rviz &
  echo "[operator] RViz at http://localhost:6080/vnc.html"
fi
```
(Apply the CLAUDE.md software-GL note: if gzclient-style CPU starvation appears, set `LP_NUM_THREADS=4` in compose `environment:`.)

- [ ] **Step 3: docker-compose.yml — expose noVNC port + LP_NUM_THREADS**

Add under `operator:`:
```yaml
    ports:
      - "6080:6080"
    environment:
      ROS_MASTER_URI: "http://${ROBOT_HOST_IP}:11311"
      LP_NUM_THREADS: "4"
```
(Merge with the existing `environment:` block — do not duplicate the key.)

- [ ] **Step 4: README.md — new runbook**

Document: bring up the GPS park sim natively (with the ROS_IP export); `docker compose up -d` (RViz auto-starts → open `http://localhost:6080/vnc.html`); `docker compose exec operator ./operator/operate.py --lat <L> --lon <O>` for the REPL; list the commands (goal/cancel/teleop/stop/estop/release/auto/status/quit).

- [ ] **Step 5: Build the image**

Run: `cd operator && export ROBOT_HOST_IP=127.0.0.1 && docker compose build`
Expected: build succeeds.

- [ ] **Step 6: Commit**

```bash
git add operator/Dockerfile operator/entrypoint.sh operator/docker-compose.yml operator/README.md
git commit -m "feat(operator): RViz + noVNC visual stack in operator container"
```

---

## Task 8: Live end-to-end verification (against the running GPS park sim)

No new files. Exercises the whole operator on the real graph and confirms the §10 spec verification points. Requires the GPS park robot + move_base running natively with the ROS_IP export.

- [ ] **Step 1: Bring up sim + operator container**

Host: export `ROS_IP`/`ROS_MASTER_URI` to the `husky_lan` gateway; start the GPS park robot + `move_base_gps.launch`. Then `cd operator && docker compose up -d`; open `http://localhost:6080/vnc.html` → RViz shows the robot.

- [ ] **Step 2: goal — drives to point**

`docker compose exec operator ./operator/operate.py --lat 49.9007 --lon 8.9`.
Verify in RViz: green goal marker, global plan, robot drives; `status` shows `dist` shrinking; console logs `SENT goal`. Verify `move_base` reaches `SUCCEEDED`.

- [ ] **Step 3: teleop override**

In the REPL: `teleop`, drive with i/j/k/l. In another shell `rostopic echo joy_teleop/cmd_vel` shows the Twist; robot moves under manual control even if `/cmd_vel` has a publisher. Exit teleop → `stop`.

- [ ] **Step 4: e-stop is decisive**

Send a `goal`, then `estop`. Robot freezes (no motion) while a goal is active; `rostopic echo /e_stop` shows the latched engage. `release` → motion resumes. Confirms priority-255 lock.

- [ ] **Step 5: cancel**

Send a `goal`, then `cancel`; `move_base` goes to a preempted/terminal state; robot stops navigating.

- [ ] **Step 6: CSV contract**

`head -1 ../operator_run.csv` → 16 columns; cols 1–11 header byte-identical to the baseline contract. Confirm `operator_mode` reflects the modes used and `active_goal_x/y` populated. Confirm an existing attack plot script can still read cols 1–11.

- [ ] **Step 7: Commit any fixes found during verification**

```bash
git add -A && git commit -m "fix(operator): live-verification adjustments"
```
(Only if changes were needed; otherwise skip.)

---

## Self-Review Notes

- **Spec coverage:** §1 purpose → whole plan; §2 fidelity/twist_mux → T3, T5(dispatch), T8(3–4); §3 one-node + RViz → T5, T6, T7; §4 commands → T4, T5; §5 goal semantics → T5 `_do_goal` (keeps `latlon_to_map`); §6 telemetry/CSV → T2, T5(callbacks/`_write_row`); §7 lifecycle → T5(`run` loop, runs until quit); §8 container → T7; §9 files → matches T5–T7 modified/new lists; §10 verification → T8; §11 out-of-scope → nothing added (no battery, no click-to-goal, no TUI, no alarm).
- **Pre-flight P1/P2** de-risk the two runtime assumptions (e_stop type, plan/teleop topic names) before the tasks that depend on them.
- **Type consistency:** `GcsState` fields/methods, `CSV_HEADER`/`build_row` signature, `Intervene` methods, and `parse_command` return contract are used with identical names in T5.
- **No ground truth:** pose is `/odometry/filtered_map`; no `gazebo_msgs`. (The `ros-noetic-gazebo-msgs` already in the Dockerfile is unused legacy — left untouched per the do-not-modify constraint on scope; not imported by the new code.)
