# GPS-Anchored Dead-Reckoning Prior Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Give the landmark localizer a clean, attack-independent prior — the initial (pre-attack) GPS pose as an anchor, propagated by odom-frame dead reckoning and re-anchored on accepted landmark fixes — so landmark mode bootstraps and tracks even while GPS is spoofed.

**Architecture:** Add a pure `compose_prior()` function (ROS-free, unit-tested) that composes an immutable map-frame anchor with the odom-frame displacement since the anchor. Rewire `localizer_node.main()` to capture the anchor once at startup (when GPS is valid), cache the live odom-frame pose, compute the prior from anchor⊕odom in `on_cloud` instead of reading the poisonable map pose, and re-anchor to each accepted fix.

**Tech Stack:** ROS Noetic, rospy, `nav_msgs/Odometry`, `sensor_msgs/NavSatFix`, numpy, pytest.

## Global Constraints

- `landmark_loc` is a loose script tree, NOT a catkin package. No new `.srv`, no build step.
- ALL ROS imports stay **inside** `main()` / callbacks so the module imports without ROS (existing pattern). New pure functions (`compose_prior`) are module-level and ROS-free.
- The initial anchor is **GPS-derived, captured exactly once** at startup; after that it is advanced **only** by the localizer's own accepted landmark fixes — **never** re-read from `/odometry/filtered_map` (that would reintroduce the spoof).
- Re-anchoring happens **only** on a fix that passed the full count + residual gate — never on a coasted/None result. The `residual_gate` (0.4) and `≥ 2` correspondence rule are the load-bearing safeguard for the anchor.
- The odom-frame source is `/odometry/filtered_odom` (odom-frame EKF, `world_frame: odom`), verified to fuse only wheel odom + IMU — **no abs_fix/GPS/navsat**. Do not use `/odometry/filtered_map` for motion.
- Position-only fix (yaw logged, not fused) — unchanged from today.
- No Gazebo ground truth anywhere (standing project rule).
- GPS-clean-at-startup is an accepted assumption; prior-free global localization is out of scope (separate deferred design).

## Authoritative names / values

- New pure fn: `compose_prior(anchor_map, anchor_odom, odom_now) -> (x, y, yaw)`
  - `anchor_map = (ax, ay, ayaw)` map-frame anchor
  - `anchor_odom = (ox0, oy0, oyaw0)` odom-frame pose at anchor capture
  - `odom_now = (ox, oy, oyaw)` current odom-frame pose
- Topics: prior-motion `/odometry/filtered_odom`; anchor-capture source `/odometry/filtered_map`; GPS validity `/navsat/fix`; output unchanged `/odometry/landmark_fix`.
- `state` dict gains: `anchor_map`, `anchor_odom`, `odom_now` (all default `None`).

---

## File Structure

- **Modify** `landmark_loc/localizer_node.py`:
  - add module-level `compose_prior()` (Task 1)
  - rewire `main()`: odom + navsat subscribers, one-time anchor capture, prior from `compose_prior`, re-anchor on accepted fix (Task 2)
- **Modify** `landmark_loc/tests/test_node_helpers.py` (or new `test_compose_prior.py`): unit tests for `compose_prior` (Task 1)

---

## Task 1: `compose_prior` pure function + tests

**Files:**
- Modify: `landmark_loc/localizer_node.py` (add module-level function, no ROS)
- Test: `landmark_loc/tests/test_compose_prior.py` (create)

**Interfaces:**
- Produces: `compose_prior(anchor_map, anchor_odom, odom_now) -> (x, y, yaw)` — returns the map-frame prior obtained by applying the odom-frame displacement (`odom_now` relative to `anchor_odom`) starting from `anchor_map`. Pure, ROS-free, deterministic.
- Consumes: nothing.

- [ ] **Step 1: Write the failing tests**

```python
# landmark_loc/tests/test_compose_prior.py
import math
from landmark_loc.localizer_node import compose_prior


def _close(a, b, tol=1e-9):
    return abs(a - b) <= tol


def test_zero_motion_returns_anchor():
    anchor = (10.0, -5.0, 0.7)
    x, y, yaw = compose_prior(anchor, (3.0, 3.0, 0.2), (3.0, 3.0, 0.2))
    assert _close(x, 10.0) and _close(y, -5.0) and _close(yaw, 0.7)


def test_pure_translation_anchor_yaw_zero():
    # anchor yaw 0 -> odom displacement applies directly in map axes
    anchor = (10.0, -5.0, 0.0)
    x, y, yaw = compose_prior(anchor, (0.0, 0.0, 0.0), (2.0, 1.0, 0.0))
    assert _close(x, 12.0) and _close(y, -4.0) and _close(yaw, 0.0)


def test_pure_translation_anchor_yaw_90deg():
    # anchor facing +90deg: odom +x (forward) maps to map +y
    anchor = (0.0, 0.0, math.pi / 2)
    x, y, yaw = compose_prior(anchor, (0.0, 0.0, 0.0), (2.0, 0.0, 0.0))
    assert _close(x, 0.0, 1e-9) and _close(y, 2.0) and _close(yaw, math.pi / 2)


def test_pure_rotation_updates_yaw_only():
    anchor = (4.0, 7.0, 0.3)
    x, y, yaw = compose_prior(anchor, (1.0, 1.0, 0.0), (1.0, 1.0, 0.5))
    assert _close(x, 4.0) and _close(y, 7.0) and _close(yaw, 0.8)


def test_combined_matches_hand_computed():
    # anchor at (0,0,0); odom moves from (0,0,0) to (1,2, pi/2)
    anchor = (0.0, 0.0, 0.0)
    x, y, yaw = compose_prior(anchor, (0.0, 0.0, 0.0), (1.0, 2.0, math.pi / 2))
    assert _close(x, 1.0) and _close(y, 2.0) and _close(yaw, math.pi / 2)


def test_anchor_offset_and_odom_offset():
    # odom baseline non-zero: only the DELTA since baseline matters
    anchor = (100.0, 200.0, 0.0)
    x, y, yaw = compose_prior(anchor, (5.0, 5.0, 0.0), (8.0, 9.0, 0.0))
    assert _close(x, 103.0) and _close(y, 204.0) and _close(yaw, 0.0)
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_compose_prior.py -v`
Expected: FAIL — `compose_prior` not defined.

- [ ] **Step 3: Add `compose_prior` to `localizer_node.py`**

Add at module level (after `cloud_to_array`, before `main`):

```python
def compose_prior(anchor_map, anchor_odom, odom_now):
    """Map-frame prior = anchor_map advanced by the odom-frame displacement of
    odom_now relative to anchor_odom.

    anchor_map  = (ax, ay, ayaw)   immutable map-frame anchor (pre-attack GPS,
                                   or the last accepted landmark fix)
    anchor_odom = (ox0, oy0, oyaw0) odom-frame pose captured with the anchor
    odom_now    = (ox, oy, oyaw)    current odom-frame pose (attack-independent)

    The odom frame drifts but its relative motion is trustworthy, so the
    displacement since the anchor, applied from the anchor, tracks the robot's
    true pose without ever reading the (spoofable) map pose.
    """
    ax, ay, ayaw = anchor_map
    ox0, oy0, oyaw0 = anchor_odom
    ox, oy, oyaw = odom_now
    # displacement in the odom frame, rotated into the anchor-odom body frame
    dx_o, dy_o = ox - ox0, oy - oy0
    c0, s0 = math.cos(-oyaw0), math.sin(-oyaw0)
    rx = c0 * dx_o - s0 * dy_o
    ry = s0 * dx_o + c0 * dy_o
    # apply that body-frame displacement from the map-frame anchor
    ca, sa = math.cos(ayaw), math.sin(ayaw)
    px = ax + ca * rx - sa * ry
    py = ay + sa * rx + ca * ry
    pyaw = ayaw + (oyaw - oyaw0)
    return (px, py, pyaw)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m pytest landmark_loc/tests/test_compose_prior.py -v`
Expected: PASS (6 tests).

- [ ] **Step 5: Confirm module still imports without ROS**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -c "import sys; import landmark_loc.localizer_node as m; assert 'rospy' not in sys.modules; print('ok', m.compose_prior((0,0,0),(0,0,0),(1,0,0)))"`
Expected: prints `ok (1.0, 0.0, 0.0)` and rospy NOT imported.

- [ ] **Step 6: Commit**

```bash
git add landmark_loc/localizer_node.py landmark_loc/tests/test_compose_prior.py
git commit -m "feat(localizer): compose_prior — map anchor advanced by odom displacement"
```

---

## Task 2: Rewire `main()` — anchor capture, clean prior, re-anchor on fix

**Files:**
- Modify: `landmark_loc/localizer_node.py` (`main()` only)

**Interfaces:**
- Consumes: `compose_prior` (Task 1), existing pipeline (`segment`, `classify`, `catalog`, `solve`).
- Produces: a localizer whose prior comes from `compose_prior(anchor_map, anchor_odom, odom_now)` — never the raw map pose after startup — and that re-anchors on each accepted fix. Output topic/behaviour otherwise unchanged.

**Note:** I/O glue — no ROS-free unit test; verified in-sim (acceptance below). Reviewer checks statically against the spec.

- [ ] **Step 1: Extend the `state` dict**

Replace `state = {"prior": None, "last_pub": rospy.Time(0)}` with:

```python
    state = {
        "anchor_map": None,    # (ax, ay, ayaw) immutable-ish map anchor
        "anchor_odom": None,   # (ox0, oy0, oyaw0) odom pose captured with anchor
        "odom_now": None,      # (ox, oy, oyaw) latest odom-frame pose
        "gps_valid": False,    # /navsat/fix status.status >= 0 seen
        "last_pub": rospy.Time(0),
    }
```

- [ ] **Step 2: Add a yaw helper and the odom / navsat / map callbacks**

Add near the top of `main()` (after `state`), a local yaw extractor and the three callbacks:

```python
    def _yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def on_odom(msg):
        p_ = msg.pose.pose.position
        state["odom_now"] = (p_.x, p_.y, _yaw(msg.pose.pose.orientation))

    def on_navsat(msg):
        if msg.status.status >= 0:
            state["gps_valid"] = True

    def on_map(msg):
        # ONE-TIME anchor capture: only before an anchor exists, only when GPS
        # is valid and an odom pose is available. Never updates the anchor after.
        if state["anchor_map"] is not None:
            return
        if not state["gps_valid"] or state["odom_now"] is None:
            return
        p_ = msg.pose.pose.position
        state["anchor_map"] = (p_.x, p_.y, _yaw(msg.pose.pose.orientation))
        state["anchor_odom"] = state["odom_now"]
        rospy.loginfo("anchor captured: map=(%.2f,%.2f,%.2f) odom=(%.2f,%.2f,%.2f)",
                      state["anchor_map"][0], state["anchor_map"][1], state["anchor_map"][2],
                      state["anchor_odom"][0], state["anchor_odom"][1], state["anchor_odom"][2])
```

- [ ] **Step 3: Rewrite `on_cloud` to use the composed prior and re-anchor on fix**

Replace the body of `on_cloud` with:

```python
    def on_cloud(msg):
        now = rospy.Time.now()
        if (now - state["last_pub"]).to_sec() < 1.0 / p["rate"]:
            return
        if (state["anchor_map"] is None or state["anchor_odom"] is None
                or state["odom_now"] is None):
            return
        prior = compose_prior(state["anchor_map"], state["anchor_odom"],
                              state["odom_now"])
        pts = cloud_to_array(msg)
        if len(pts) == 0:
            return
        cropped = segment.crop(pts, p["z_min"], p["z_max"], p["max_range"])
        clusters = segment.cluster(cropped, p["link_dist"], p["min_pts"], p["max_extent"])
        obs = classify.to_observations(clusters)
        gated = catalog.gate(landmarks, prior, p["max_range"], p["fov_halfwidth"])
        result = solve.solve_pose(obs, gated, prior, p["dist_gate"], p["residual_gate"])
        if result is None:
            return
        x, y, yaw, rms, n = result
        # RE-ANCHOR: an accepted (gated) fix is a trustworthy landmark-derived
        # absolute position. Reset the dead-reckoning baseline to it so drift
        # only accumulates between fixes, never over the whole run.
        state["anchor_map"] = (x, y, prior[2])   # keep composed yaw (yaw not solved/fused)
        state["anchor_odom"] = state["odom_now"]
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.w = 1.0
        od.pose.covariance = covariance_for(n, p["base_var"])
        pub.publish(od)
        state["last_pub"] = now
```

Note: the fix is position-only (`solve` returns a `yaw` but it is not fused; the map-EKF takes yaw from `/compass/data`). The re-anchor keeps the composed `prior[2]` as the anchor yaw so heading continues to track odom, consistent with today's position-only behaviour.

- [ ] **Step 4: Replace the subscribers**

Replace the two existing `rospy.Subscriber(...)` lines with:

```python
    from sensor_msgs.msg import NavSatFix
    rospy.Subscriber("/odometry/filtered_odom", Odometry, on_odom, queue_size=5)
    rospy.Subscriber("/navsat/fix", NavSatFix, on_navsat, queue_size=5)
    rospy.Subscriber("/odometry/filtered_map", Odometry, on_map, queue_size=5)
    rospy.Subscriber("/os0_cloud_node/points", PointCloud2, on_cloud, queue_size=1)
```

Add `NavSatFix` to the imports at the top of `main()` (with `Odometry`, `PointCloud2`) instead of the inline import if preferred — either is fine as long as it stays inside `main()`. Remove the now-unused `on_prior` function.

- [ ] **Step 5: Update the module docstring**

Change the docstring's pipeline line from "gate catalog by the EKF prior" to reflect the clean prior:

```
Pipeline per cloud: cloud->array -> crop -> cluster -> classify -> gate catalog
by a GPS-anchored dead-reckoned prior (initial pre-attack GPS anchor advanced by
odom-frame motion, re-anchored on each accepted fix) -> associate ->
rigid-transform solve -> publish /odometry/landmark_fix ...
```

- [ ] **Step 6: Byte-compile, import check, run existing localizer tests**

Run: `cd /home/thinh/Documents/Husky_viz && python3 -m py_compile landmark_loc/localizer_node.py && python3 -c "import sys; import landmark_loc.localizer_node as m; assert 'rospy' not in sys.modules; print('import ok')" && python3 -m pytest landmark_loc/tests/test_node_helpers.py landmark_loc/tests/test_compose_prior.py -q`
Expected: compiles, `import ok` (no rospy at module load), tests PASS.

- [ ] **Step 7: Commit**

```bash
git add landmark_loc/localizer_node.py
git commit -m "feat(localizer): GPS-anchored dead-reckoned prior + re-anchor on fix"
```

---

## In-sim acceptance (run by the main conversation, from a clean kill, gzclient on :0)

Not a subagent task. Per project rules (full teardown first; the GPU-ray lidar cloud needs gzclient on display :0 or `/os0_cloud_node/points` never publishes):

1. Bring up on GPS; start move_base, localizer, selector. Confirm the localizer logs `anchor captured: map=(...) odom=(...)` once, near the true startup pose.
2. Send the goal `goal 49.9000094 8.9000327`; confirm the robot drives on GPS and the localizer publishes `/odometry/landmark_fix` (mode `gps` still, but the fix stream proves the clean prior matches).
3. Mid-route, start a strong spoof: `attack.sh navsat --drift-rate 1.5 --max-offset 40 --duration 90`.
4. Switch to landmark mode **while the spoof is active** (`mode landmark`).
5. Confirm: `/odometry/landmark_fix` keeps publishing (mode clears from `:stale`), the fused pose tracks the robot's TRUE position (verified against `/navsat/fix` — the honest sensor, NOT the spoofed abs_fix), and the robot reaches the goal on landmark localization with the attack running. Contrast with the prior 51 m abort.

## Self-Review notes

- **Spec coverage:** clean prior via `compose_prior` (T1); one-time GPS-valid anchor capture (T2 S2); prior from anchor⊕odom, not raw map pose (T2 S3); re-anchor on accepted fix only (T2 S3); odom source `/odometry/filtered_odom`, GPS-validity via `/navsat/fix` (T2 S4). All covered.
- **Type consistency:** `compose_prior` returns `(x,y,yaw)`; `state` stores 3-tuples; `on_cloud` passes the composed 3-tuple to `gate`/`solve_pose` exactly as the old prior tuple. Consistent.
- **No placeholders:** all code blocks concrete; every symbol defined.
