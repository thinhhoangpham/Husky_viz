# Live Pose-Source Switch Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Let the operator switch live and manually between GPS and landmark absolute-position sources, guaranteeing exactly one source feeds the map-EKF's `odometry/abs_fix` at any moment.

**Architecture:** A new self-contained loose python node `abs_fix_selector` sits between the two sources (`odometry/gps_fix`, `odometry/landmark_fix`) and the map-EKF's `odometry/abs_fix` input. It strictly forwards the selected source, exposes a `topic_tools/MuxSelect` service to switch, publishes a latched friendly-name status topic with a `:stale` flag, and raises stale after 2s of selected-source silence — never substituting. The two feeders are renamed to distinct topics; the EKF input topic name is unchanged.

**Tech Stack:** ROS Noetic, rospy, `nav_msgs/Odometry`, stock `topic_tools/MuxSelect` service type, pytest for the ROS-free arbitration class.

## Global Constraints

- `landmark_loc` and the new selector are **loose script trees, not catkin packages**. No new `.srv`, no message generation, no build step. Use the **stock `topic_tools/MuxSelect`** service type (request `string topic`, response `string prev_topic`) via `rospy.ServiceProxy` / `rospy.Service`.
- Run loose python nodes by absolute path (`python3 <abs-path>`), matching `landmark_loc/localizer_node.py` and `operator/operate.py`. Not a `<node>` in a launch file.
- The map-EKF input topic name **`odometry/abs_fix` must not change** — `localization_map.yaml:97 odom1: odometry/abs_fix` stays as-is.
- All ROS imports go **inside** `main()` / ROS callbacks so the module imports without ROS present (same pattern as `landmark_loc/localizer_node.py`), keeping the arbitration class unit-testable.
- The selector **never substitutes** the unselected source. Strict forward only; stale is a flag, not a failover.
- Startup default mode = **`gps`** (preserves current behavior and the attacker demo).
- `stale_timeout` default = **2.0 s**.
- Forward messages **unchanged** (same header, pose, twist, covariance) — the selector is a relay, not a transformer.
- No ground truth anywhere (standing project rule).

## Topic and service names (authoritative)

- Inputs: `/odometry/gps_fix` (from navsat), `/odometry/landmark_fix` (from localizer). Both `nav_msgs/Odometry`.
- Output: `/odometry/abs_fix` (`nav_msgs/Odometry`) — read by the map-EKF.
- Switch service: `/set_abs_fix_mode`, type `topic_tools/MuxSelect`. Request `topic` = the **input topic name** of the desired source. Response `prev_topic` = the previously-selected input topic.
- Status topic: `/abs_fix_mode` (`std_msgs/String`, **latched**). Values: `gps`, `landmark`, `gps:stale`, `landmark:stale`.
- Friendly-name ↔ topic map (single source of truth, defined in the selector and mirrored in the operator):
  - `gps`      ↔ `/odometry/gps_fix`
  - `landmark` ↔ `/odometry/landmark_fix`

---

## File Structure

- **Create** `landmark_loc/abs_fix_selector.py` — the selector node. Two parts:
  - `AbsFixArbiter` (plain class, no ROS): holds selected source, last-seen timestamps, computes forward-decision and stale state. Unit-tested.
  - `main()` (ROS glue): subscribers, publisher, service, timer — all ROS imports inside.
- **Create** `tests/test_abs_fix_arbiter.py` — pytest for `AbsFixArbiter`.
- **Modify** `natural_environments_ros_opt/husky/husky_control/launch/control.launch:77` — navsat output remap `odometry/gps` → `odometry/gps_fix`.
- **Modify** `landmark_loc/localizer_node.py:61` — publish `/odometry/landmark_fix`.
- **Modify** `operator/gcs_commands.py` — parse `mode <gps|landmark>`.
- **Modify** `operator/operate.py` — `mode` command (calls the service), status line shows current mode, help text.
- **Modify** `RUN-MAP-NAV.md` — start the selector; document `mode` command; note both feeders run together.

---

## Task 1: The arbitration class (ROS-free) + tests

**Files:**
- Create: `landmark_loc/abs_fix_selector.py` (class only, this task)
- Test: `tests/test_abs_fix_arbiter.py`

**Interfaces:**
- Produces (consumed by Task 2's ROS glue and by tests):
  - `SOURCES = {"gps": "/odometry/gps_fix", "landmark": "/odometry/landmark_fix"}`
  - `TOPIC_TO_NAME = {v: k for k, v in SOURCES.items()}`
  - `class AbsFixArbiter:`
    - `__init__(self, stale_timeout=2.0, initial="gps")`
    - `select(self, name) -> str | None` — set selected source by friendly name; return the **previous friendly name**; return `None` and do not change state if `name` not in `SOURCES`.
    - `note_message(self, name, now)` — record that source `name` produced a message at time `now` (float seconds). Ignores unknown names.
    - `should_forward(self, name) -> bool` — True iff `name` is the currently selected source.
    - `status(self, now) -> str` — friendly name of the selected source, plus `:stale` suffix iff the selected source's last message is older than `stale_timeout` (or it has never produced one). Never references the unselected source.
    - `selected_name` property → current friendly name.
- Consumes: nothing.

- [ ] **Step 1: Write the failing tests**

```python
# tests/test_abs_fix_arbiter.py
import pytest
from landmark_loc.abs_fix_selector import AbsFixArbiter, SOURCES, TOPIC_TO_NAME


def test_defaults_to_gps():
    a = AbsFixArbiter()
    assert a.selected_name == "gps"
    assert a.should_forward("gps") is True
    assert a.should_forward("landmark") is False


def test_topic_maps_are_consistent():
    assert SOURCES["gps"] == "/odometry/gps_fix"
    assert SOURCES["landmark"] == "/odometry/landmark_fix"
    assert TOPIC_TO_NAME["/odometry/gps_fix"] == "gps"
    assert TOPIC_TO_NAME["/odometry/landmark_fix"] == "landmark"


def test_select_switches_and_returns_previous():
    a = AbsFixArbiter()
    prev = a.select("landmark")
    assert prev == "gps"
    assert a.selected_name == "landmark"
    assert a.should_forward("landmark") is True
    assert a.should_forward("gps") is False


def test_select_unknown_is_rejected_and_state_unchanged():
    a = AbsFixArbiter()
    assert a.select("galileo") is None
    assert a.selected_name == "gps"


def test_only_selected_source_forwards():
    a = AbsFixArbiter(initial="gps")
    assert a.should_forward("gps") is True
    assert a.should_forward("landmark") is False
    a.select("landmark")
    assert a.should_forward("gps") is False
    assert a.should_forward("landmark") is True


def test_status_fresh_when_selected_recently_published():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("gps", now=100.0)
    assert a.status(now=101.0) == "gps"


def test_status_stale_after_timeout_on_selected():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("gps", now=100.0)
    assert a.status(now=103.0) == "gps:stale"


def test_status_stale_when_selected_never_published():
    a = AbsFixArbiter(stale_timeout=2.0)
    assert a.status(now=100.0) == "gps:stale"


def test_unselected_source_silence_does_not_affect_status():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("gps", now=100.0)        # selected source fresh
    # landmark (unselected) never publishes -> must NOT make status stale
    assert a.status(now=101.0) == "gps"


def test_stale_clears_when_selected_resumes():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("gps", now=100.0)
    assert a.status(now=103.0) == "gps:stale"
    a.note_message("gps", now=103.5)
    assert a.status(now=104.0) == "gps"


def test_switching_uses_new_source_freshness():
    a = AbsFixArbiter(stale_timeout=2.0)
    a.note_message("landmark", now=50.0)     # landmark last seen long ago
    a.note_message("gps", now=100.0)
    a.select("landmark")                      # now selected: landmark, stale
    assert a.status(now=100.1) == "landmark:stale"
    a.note_message("landmark", now=100.2)
    assert a.status(now=100.3) == "landmark"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest tests/test_abs_fix_arbiter.py -v`
Expected: FAIL — `ModuleNotFoundError` / `AbsFixArbiter` not defined.

- [ ] **Step 3: Write the arbitration class**

```python
# landmark_loc/abs_fix_selector.py
"""Live pose-source selector: forwards exactly ONE absolute-position source onto
/odometry/abs_fix (the map-EKF's absolute anchor), switchable at runtime by the
operator. Strict forward -- never substitutes the unselected source; raises a
:stale flag when the selected source goes silent past stale_timeout.

This is the "twist_mux for the pose anchor": navsat_transform publishes
/odometry/gps_fix, landmark_localizer publishes /odometry/landmark_fix, and this
node passes only the selected one to /odometry/abs_fix. See
docs/superpowers/specs/2026-08-11-live-pose-source-switch-design.md.

ROS imports live inside main()/callbacks so AbsFixArbiter is unit-testable
without a running master (same pattern as landmark_loc/localizer_node.py).
"""

# friendly-name -> input topic. Single source of truth; the operator mirrors it.
SOURCES = {
    "gps": "/odometry/gps_fix",
    "landmark": "/odometry/landmark_fix",
}
TOPIC_TO_NAME = {v: k for k, v in SOURCES.items()}

OUTPUT_TOPIC = "/odometry/abs_fix"
STATUS_TOPIC = "/abs_fix_mode"
SELECT_SERVICE = "/set_abs_fix_mode"
DEFAULT_STALE_TIMEOUT = 2.0


class AbsFixArbiter(object):
    """Pure arbitration logic, no ROS. Decides which source forwards and whether
    the selected source is stale. Knows nothing about the unselected source's
    freshness -- silence there must never affect output or status."""

    def __init__(self, stale_timeout=DEFAULT_STALE_TIMEOUT, initial="gps"):
        if initial not in SOURCES:
            raise ValueError("unknown initial source: %s" % initial)
        self.stale_timeout = stale_timeout
        self._selected = initial
        self._last_seen = {}  # friendly name -> float seconds of last message

    @property
    def selected_name(self):
        return self._selected

    def select(self, name):
        """Switch selected source by friendly name. Return the PREVIOUS friendly
        name. Return None (state unchanged) if name is unknown."""
        if name not in SOURCES:
            return None
        prev = self._selected
        self._selected = name
        return prev

    def note_message(self, name, now):
        """Record that source `name` produced a message at time `now`."""
        if name in SOURCES:
            self._last_seen[name] = now

    def should_forward(self, name):
        """True iff `name` is the currently selected source."""
        return name == self._selected

    def status(self, now):
        """Friendly name of the selected source, with ':stale' iff its last
        message is older than stale_timeout (or it never published). Only the
        SELECTED source's freshness matters."""
        last = self._last_seen.get(self._selected)
        stale = last is None or (now - last) > self.stale_timeout
        return self._selected + (":stale" if stale else "")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest tests/test_abs_fix_arbiter.py -v`
Expected: PASS (all 10 tests).

- [ ] **Step 5: Commit**

```bash
git add landmark_loc/abs_fix_selector.py tests/test_abs_fix_arbiter.py
git commit -m "feat(selector): AbsFixArbiter arbitration class + tests"
```

---

## Task 2: The ROS node glue (main())

**Files:**
- Modify: `landmark_loc/abs_fix_selector.py` (append `main()` + `__main__`)

**Interfaces:**
- Consumes: `AbsFixArbiter`, `SOURCES`, `TOPIC_TO_NAME`, `OUTPUT_TOPIC`, `STATUS_TOPIC`, `SELECT_SERVICE`, `DEFAULT_STALE_TIMEOUT` from Task 1.
- Produces: a runnable node (`python3 landmark_loc/abs_fix_selector.py`) that:
  - forwards the selected input `Odometry` to `/odometry/abs_fix` unchanged,
  - advertises `/set_abs_fix_mode` (`topic_tools/MuxSelect`),
  - publishes latched `/abs_fix_mode` on every mode change and on stale transitions.

**Note:** This task has no ROS-free unit test (it is I/O glue). It is verified in-sim by the main conversation (see Task 6 acceptance). The task reviewer checks the glue against the spec statically.

- [ ] **Step 1: Append `main()` to `landmark_loc/abs_fix_selector.py`**

```python
def main():
    import rospy
    from nav_msgs.msg import Odometry
    from std_msgs.msg import String
    from topic_tools.srv import MuxSelect, MuxSelectResponse

    rospy.init_node("abs_fix_selector")
    stale_timeout = rospy.get_param("~stale_timeout", DEFAULT_STALE_TIMEOUT)
    initial = rospy.get_param("~initial_mode", "gps")
    if initial not in SOURCES:
        rospy.logwarn("initial_mode '%s' unknown; defaulting to gps", initial)
        initial = "gps"

    arb = AbsFixArbiter(stale_timeout=stale_timeout, initial=initial)
    out_pub = rospy.Publisher(OUTPUT_TOPIC, Odometry, queue_size=5)
    status_pub = rospy.Publisher(STATUS_TOPIC, String, queue_size=1, latch=True)
    last_status = {"value": None}

    def publish_status():
        s = arb.status(rospy.get_time())
        if s != last_status["value"]:
            last_status["value"] = s
            status_pub.publish(String(data=s))

    def on_source(msg, name):
        arb.note_message(name, rospy.get_time())
        if arb.should_forward(name):
            out_pub.publish(msg)          # forwarded UNCHANGED
        publish_status()

    for fname, topic in SOURCES.items():
        rospy.Subscriber(topic, Odometry, on_source, callback_args=fname,
                         queue_size=5)

    def on_select(req):
        # req.topic is an INPUT TOPIC NAME (topic_tools/MuxSelect convention).
        name = TOPIC_TO_NAME.get(req.topic)
        if name is None:
            rospy.logwarn("set_abs_fix_mode: unknown topic '%s'", req.topic)
            return MuxSelectResponse(prev_topic="")  # empty => rejected
        prev = arb.select(name)
        rospy.loginfo("abs_fix source: %s -> %s", prev, name)
        publish_status()
        return MuxSelectResponse(prev_topic=SOURCES[prev])

    rospy.Service(SELECT_SERVICE, MuxSelect, on_select)

    publish_status()  # latch the initial mode immediately

    # Stale watchdog: re-evaluate status ~2 Hz so :stale latches even when the
    # selected source is fully silent (no callback would otherwise fire).
    def on_timer(_evt):
        publish_status()
    rospy.Timer(rospy.Duration(0.5), on_timer)

    rospy.loginfo("abs_fix_selector up: initial=%s, stale_timeout=%.1fs",
                  initial, stale_timeout)
    rospy.spin()


if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Verify the module still imports without ROS and tests still pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -c "import landmark_loc.abs_fix_selector as m; print(m.SOURCES)" && python3 -m pytest tests/test_abs_fix_arbiter.py -q`
Expected: prints the SOURCES dict, tests PASS. (Imports at module level must NOT pull in rospy.)

- [ ] **Step 3: Byte-compile the node to catch syntax errors**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m py_compile landmark_loc/abs_fix_selector.py && echo OK`
Expected: `OK`.

- [ ] **Step 4: Commit**

```bash
git add landmark_loc/abs_fix_selector.py
git commit -m "feat(selector): ROS node glue (forward, MuxSelect service, latched status)"
```

---

## Task 3: Rename the two feeders' output topics

**Files:**
- Modify: `natural_environments_ros_opt/husky/husky_control/launch/control.launch:77`
- Modify: `landmark_loc/localizer_node.py:61`

**Interfaces:**
- Produces: navsat now publishes `/odometry/gps_fix`; localizer now publishes `/odometry/landmark_fix`. Neither writes `/odometry/abs_fix` anymore — only the selector does.
- Consumes: nothing (topic-name edits only).

- [ ] **Step 1: Repoint navsat output**

In `control.launch`, change the navsat remap (currently line 77):

```xml
    <remap from="odometry/gps" to="odometry/gps_fix"/>
```

(was `to="odometry/abs_fix"`).

- [ ] **Step 2: Repoint the localizer publisher**

In `landmark_loc/localizer_node.py` (currently line 61):

```python
    pub = rospy.Publisher("/odometry/landmark_fix", Odometry, queue_size=5)
```

(was `"/odometry/abs_fix"`). Update the module docstring line that mentions publishing `/odometry/abs_fix` to say `/odometry/landmark_fix`.

- [ ] **Step 3: Confirm nothing else still publishes abs_fix directly**

Run: `cd /home/thinh/Documents/Husky_viz && grep -rn "odometry/abs_fix" --include=*.py --include=*.launch --include=*.yaml . | grep -v abs_fix_selector`
Expected: only `localization_map.yaml` (the EKF **input**, correct) and doc/comment references. No `.py`/`.launch` still *publishing* it except the selector.

- [ ] **Step 4: Commit**

```bash
git add natural_environments_ros_opt/husky/husky_control/launch/control.launch landmark_loc/localizer_node.py
git commit -m "refactor(nav): feeders publish gps_fix/landmark_fix; selector fills abs_fix"
```

---

## Task 4: Operator command parsing for `mode`

**Files:**
- Modify: `operator/gcs_commands.py`
- Test: `operator/tests/test_gcs_commands_mode.py` (create; if `operator/tests/` does not exist, place at `tests/test_gcs_commands_mode.py` and adjust import)

**Interfaces:**
- Consumes: existing `parse_command(line)` contract — returns `(verb, args)`.
- Produces: `parse_command("mode gps") -> ("mode", ["gps"])`, `parse_command("mode landmark") -> ("mode", ["landmark"])`, `parse_command("mode") -> ("error", [...])`, `parse_command("mode foo") -> ("error", [...])`.

- [ ] **Step 1: Write the failing test**

```python
# tests/test_gcs_commands_mode.py
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "operator"))
from gcs_commands import parse_command


def test_mode_gps():
    assert parse_command("mode gps") == ("mode", ["gps"])

def test_mode_landmark():
    assert parse_command("mode landmark") == ("mode", ["landmark"])

def test_mode_case_insensitive_value():
    assert parse_command("mode GPS") == ("mode", ["gps"])

def test_mode_missing_arg_is_error():
    verb, _ = parse_command("mode")
    assert verb == "error"

def test_mode_unknown_value_is_error():
    verb, _ = parse_command("mode teleporter")
    assert verb == "error"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest tests/test_gcs_commands_mode.py -v`
Expected: FAIL — `mode` currently falls through to `("unknown", ["mode"])`.

- [ ] **Step 3: Add `mode` parsing to `gcs_commands.py`**

Insert, before the `if verb in SIMPLE:` line:

```python
    if verb == "mode":
        rest = parts[1:]
        if len(rest) != 1:
            return ("error", ["mode needs <gps|landmark>"])
        val = rest[0].lower()
        if val not in ("gps", "landmark"):
            return ("error", ["mode must be gps or landmark"])
        return ("mode", [val])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest tests/test_gcs_commands_mode.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add operator/gcs_commands.py tests/test_gcs_commands_mode.py
git commit -m "feat(operator): parse 'mode <gps|landmark>' command"
```

---

## Task 5: Operator `mode` command + status line

**Files:**
- Modify: `operator/operate.py`

**Interfaces:**
- Consumes: `parse_command` returning `("mode", [val])` from Task 4; the selector's `/set_abs_fix_mode` service (`topic_tools/MuxSelect`) and latched `/abs_fix_mode` topic from Tasks 1-2.
- Produces: `mode gps` / `mode landmark` at the REPL calls the service and prints the result; the status line shows the current mode; help lists it.

- [ ] **Step 1: Subscribe to the latched status topic**

In `Operator.__init__`, alongside the other subscribers, add a String subscriber that caches the latest mode:

```python
        from std_msgs.msg import String
        self._abs_fix_mode = None
        rospy.Subscriber("/abs_fix_mode", String, self._on_abs_fix_mode,
                          queue_size=1)
```

And the callback (method on `Operator`):

```python
    def _on_abs_fix_mode(self, msg):
        with self._lock:
            self._abs_fix_mode = msg.data
```

- [ ] **Step 2: Add the friendly-name→topic map and the service call**

Near the top of `operate.py` (module constants), mirror the selector's map:

```python
# Mirror of landmark_loc/abs_fix_selector.SOURCES (kept in sync by hand; the
# selector owns the authoritative copy). Operator sends the INPUT TOPIC NAME to
# the topic_tools/MuxSelect service.
ABS_FIX_SOURCES = {"gps": "/odometry/gps_fix",
                   "landmark": "/odometry/landmark_fix"}
SET_ABS_FIX_MODE_SRV = "/set_abs_fix_mode"
```

Add a dispatch branch in `_dispatch` (alongside the other `elif cmd == ...`):

```python
        elif cmd == "mode":
            self._do_mode(args[0])
```

And the method:

```python
    def _do_mode(self, name):
        from topic_tools.srv import MuxSelect
        topic = ABS_FIX_SOURCES.get(name)
        if topic is None:
            rospy.logwarn("mode: unknown source '%s'", name)
            return
        try:
            rospy.wait_for_service(SET_ABS_FIX_MODE_SRV, timeout=3.0)
        except rospy.ROSException:
            rospy.logwarn("mode: %s not available (is abs_fix_selector "
                          "running?)", SET_ABS_FIX_MODE_SRV)
            return
        try:
            select = rospy.ServiceProxy(SET_ABS_FIX_MODE_SRV, MuxSelect)
            resp = select(topic)
        except rospy.ServiceException as exc:
            rospy.logwarn("mode: service call failed: %s", exc)
            return
        if not resp.prev_topic:
            rospy.logwarn("mode: selector rejected '%s'", name)
            return
        rospy.loginfo("abs_fix source now '%s' (was '%s')", name,
                      ABS_FIX_SOURCES_INV.get(resp.prev_topic, resp.prev_topic))
```

Add the inverse map next to `ABS_FIX_SOURCES`:

```python
ABS_FIX_SOURCES_INV = {v: k for k, v in ABS_FIX_SOURCES.items()}
```

- [ ] **Step 3: Show the mode in the status line**

In `_print_status`, read the cached mode and append it:

```python
        with self._lock:
            abs_mode = self._abs_fix_mode
        abs_mode_str = abs_mode if abs_mode is not None else "n/a"
```

Append `abs_fix=%s` to the printed line (add `abs_mode_str` to the format args):

```python
        print("state=%s | sent=%s | active=%s | dist=%s | mode=%s | estop=%s | "
              "link_age=%s | abs_fix=%s" % (
            self.state.nav_status, sx_sy, ax_ay, dist, self.state.mode,
            self.state.estop_engaged, age_str, abs_mode_str))
```

- [ ] **Step 4: Update help text**

In `_print_help`, add a line after the `goal <name>` line:

```python
            "  mode <gps|landmark>  switch the absolute-position source live\n"
```

- [ ] **Step 5: Byte-compile and confirm command parsing tests still pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m py_compile operator/operate.py && python3 -m pytest tests/test_gcs_commands_mode.py -q && echo OK`
Expected: `OK` (compiles; parser tests pass). Full operate.py behavior is verified in-sim (Task 6).

- [ ] **Step 6: Commit**

```bash
git add operator/operate.py
git commit -m "feat(operator): 'mode gps|landmark' live switch + status line"
```

---

## Task 6: Runbook documentation

**Files:**
- Modify: `RUN-MAP-NAV.md`

**Interfaces:**
- Consumes: everything above (node run command, operator command).
- Produces: a runbook that starts the selector and both feeders, and documents `mode`.

- [ ] **Step 1: Rework Step 2 to start the selector + both feeders**

Replace the Option A / Option B split with a single flow that always runs the selector, since the switch is now live. Add, after `move_base` starts, a block to start the selector (loose python node, like the localizer):

```bash
# in its own terminal: start the pose-source selector (fills /odometry/abs_fix)
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
python3 ~/Documents/Husky_viz/landmark_loc/abs_fix_selector.py
```

Keep the existing localizer start block (Step 2 Option B), and note both the localizer and navsat can run together — the selector arbitrates. Note the selector starts in `gps` mode by default.

- [ ] **Step 2: Document the `mode` command under Step 4**

Add to the operator commands block:

```
mode gps                       # switch absolute source to GPS (navsat)
mode landmark                  # switch absolute source to landmark localizer
```

Add one sentence: switching is live; `status` shows `abs_fix=<mode>` (with `:stale` if the selected source has gone silent).

- [ ] **Step 3: Note the attack interaction**

Add a sentence to Step 6: the GPS spoof only affects the fused pose while `abs_fix` is in `gps` mode; `mode landmark` removes navsat from the loop live, so the operator can switch away from a spoofed source.

- [ ] **Step 4: Commit**

```bash
git add RUN-MAP-NAV.md
git commit -m "docs(runbook): start abs_fix_selector; document 'mode' live switch"
```

---

## In-sim acceptance (run by the main conversation, from a clean kill)

Not a subagent task. After all tasks merge, the main conversation verifies, per `agents-implement-main-runs-sim` (full teardown first, verify master down, then start fresh):

1. Bring up sim clean; start move_base, the localizer, and `abs_fix_selector`.
2. Confirm `/abs_fix_mode` latches `gps`; `rostopic echo /odometry/abs_fix` tracks `/odometry/gps_fix`; send a goal, robot navigates.
3. At `operator>`, `mode landmark` → prints the switch, `/abs_fix_mode` latches `landmark`, `abs_fix` now tracks `/odometry/landmark_fix`, robot keeps navigating.
4. Stop the selected feeder's input (e.g. pause the localizer while in landmark mode) → `/abs_fix_mode` shows `landmark:stale`, `abs_fix` stops updating, `/odometry/gps_fix` is **not** substituted.
5. `mode gps` → recovers.

## Self-Review notes

- **Spec coverage:** selector node (T1-2), rename (T3), operator command+status (T4-5), runbook (T6), stale flag (T1 `status` + T2 timer), service type stock MuxSelect (Global Constraints), startup default gps (T1/T2), never-substitute (T1 `status`/`should_forward` reference only the selected source). All covered.
- **Type consistency:** `AbsFixArbiter.select` returns previous friendly name (T1); T2 maps it back to a topic for `MuxSelectResponse.prev_topic`; T5 maps that topic back to a friendly name via `ABS_FIX_SOURCES_INV`. Consistent.
- **No placeholders:** all code blocks are concrete; every referenced symbol is defined in T1's Interfaces.
