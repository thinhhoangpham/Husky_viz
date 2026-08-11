# Live Operator-Switchable Pose-Source Selector — Design

**Date:** 2026-08-11
**Status:** Approved (design phase)

## Goal

Let the operator switch, **live and manually**, between the two existing
absolute-position sources — GPS (`navsat_transform`) and landmark localization
(`landmark_localizer`) — without restarting the simulation, swapping launch
files, or killing any node. The switch guarantees that **exactly one** source
feeds the map-EKF's absolute-position input at any moment.

## Background — the two nav modes today, mechanically

Both nav modes run the **identical** `move_base` + dual-EKF stack. The map-frame
EKF (`husky_control/config/localization_map.yaml`) uses `odom1: odometry/abs_fix`
as its absolute (x, y) anchor. "GPS mode" vs "landmark mode" reduces to **which
process fills the single topic `/odometry/abs_fix`**:

- **GPS mode** — `navsat_transform` publishes it (its output `odometry/gps` is
  remapped to `odometry/abs_fix` in `control.launch:77`).
- **Landmark mode** — `landmark_localizer` (`landmark_loc/localizer_node.py`)
  publishes `odometry/abs_fix` directly from lidar-vs-map matching.

**Key runtime fact that motivates this design:** `navsat_transform` is started
by the robot bring-up (`control.launch`, via `load-park-world.sh`) and runs the
**whole time, in both modes**. The landmark localizer is an *extra* node started
on top. So today, in landmark mode, **both feeders are alive and both write the
same topic `odometry/abs_fix`** — they collide, and the EKF fuses whatever
arrives. The current separation between modes is really "which node did you
start," not a clean arbitration. A *live* switch cannot exist on top of that
collision; the sources must first be separated, then arbitrated by one valve.

## Why a mux/relay (chosen architecture)

A relay node in front of the filter is both the cleanest engineering and the
closest to how a real robotics team integrates a second absolute-pose source:

- `robot_localization` assumes each input topic has **one coherent publisher**;
  pointing two sources at one input topic is a documented anti-pattern.
- The robot already uses this exact idiom for velocity: `twist_mux` arbitrates
  multiple `cmd_vel` sources into one output. This design is "twist_mux for the
  pose anchor."
- On real robots, when GPS quality drops a supervisor switches the absolute
  anchor to an alternate source. That supervisor is structurally a
  selector/relay between sources and filter — exactly this node.

Rejected alternatives: gating each feeder (navsat is a stock, unmodifiable
`robot_localization` node — gating it needs a wrapper, i.e. a messier relay);
killing/spawning feeder processes live (process churn on the shared master is
the project's documented ghost-node failure mode).

## Architecture

New loose python node **`abs_fix_selector`** (run by absolute path, matching the
repo convention for `localizer_node.py` / `operate.py`), sitting between the two
sources and the map-EKF:

```
  navsat_transform ───► /odometry/gps_fix ──────┐
                                                 ├─► [abs_fix_selector] ─► /odometry/abs_fix ─► map-EKF
  landmark_localizer ─► /odometry/landmark_fix ──┘          │
                                                            ├─► /abs_fix_mode  (latched status)
  operator ──(service /set_abs_fix_mode)──────────────────► ┘
```

### Node interface

- **Subscribes:** `/odometry/gps_fix`, `/odometry/landmark_fix` — both
  `nav_msgs/Odometry`.
- **Publishes:** `/odometry/abs_fix` (`nav_msgs/Odometry`) — forwards **only**
  the currently selected source's messages, **unchanged** (same header, pose,
  covariance). The map-EKF config is untouched; it still reads `abs_fix`.
- **Latched topic `/abs_fix_mode`** (`std_msgs/String`) — the current mode,
  broadcast on every change and on staleness transitions. Values:
  `"gps"`, `"landmark"`, `"gps:stale"`, `"landmark:stale"`. Latched so any
  late-joining observer (operator status line, RViz, logger, future supervisor)
  immediately reads the current mode without issuing a call.
- **Service `/set_abs_fix_mode`** — stock `topic_tools/MuxSelect` type: request
  `topic` (the input topic name of the desired source, e.g. `/odometry/gps_fix`),
  response `prev_topic` (the previously-selected input topic; empty string means
  the request was rejected and the mode left unchanged). Using the stock type
  avoids a custom `.srv`/catkin package. The service is how the mode is
  *changed*; the latched topic is how it is *observed*. A future automatic
  supervisor is a second client of this service.

### Arbitration behavior (strict forward + stale flag)

- The selector forwards **only** the selected source to `abs_fix`. It **never**
  substitutes the other source on its own.
- If the selected source publishes nothing for longer than `stale_timeout`
  (default **2.0 s**), the selector appends `:stale` to the latched
  `/abs_fix_mode` status. Behavior does not change — nothing is substituted;
  the EKF coasts on wheel odom + compass, its normal drift behavior — but the
  operator (and a future supervisor) can now *see* the source has gone quiet.
  The flag clears (back to plain `"gps"`/`"landmark"`) when the selected source
  resumes.
- Rationale: keeps the valve honest and dumb (the operator stays in control),
  preserves the GPS-spoof attack demo (the mux does not defend itself by fleeing
  to landmark when GPS is corrupted), and leaves health-based automatic
  switching to be designed cleanly as a separate future feature.

### Startup default

The selector starts in **`gps`** mode. This preserves today's behavior exactly:
GPS is the current default and the attacker demo assumes GPS, so nothing
regresses for anyone who does not touch the new command.

## Changes to existing files (the topic rename)

The two feeders must publish **distinct** topics so the selector can arbitrate.
The map-EKF input topic name (`odometry/abs_fix`) does **not** change.

1. **`natural_environments_ros_opt/husky/husky_control/launch/control.launch`
   (line 77):** remap navsat output `odometry/gps` → **`odometry/gps_fix`**
   (was `odometry/abs_fix`).
2. **`landmark_loc/localizer_node.py`:** publish **`odometry/landmark_fix`**
   (was `odometry/abs_fix`).
3. **`localization_map.yaml`:** `odom1: odometry/abs_fix` — **unchanged**. The
   selector now fills it.

Consequence: both feeders may run continuously in both modes (navsat always
does). They publish to their own topics; the selector decides which reaches the
EKF. Starting the localizer node becomes independent of mode selection — this is
what makes the switch truly live (no process churn, no launch swap).

## Operator REPL additions (`operator/operate.py`)

- New command **`mode gps`** / **`mode landmark`** — maps the friendly name to
  its input topic and calls `/set_abs_fix_mode`; a non-empty `prev_topic` in the
  reply confirms the switch took effect (empty means rejected). An unknown/failed
  switch prints the failure.
- **Status line** (`_print_status`) gains the current mode + stale flag, read
  from the latched `/abs_fix_mode` topic.
- Help text lists the new command.

## Documentation (`RUN-MAP-NAV.md`)

- Note that `abs_fix_selector` must be running (a loose `python3 <abs-path>`
  node, like the localizer) and that both feeders can run together.
- Document `mode gps` / `mode landmark` under the operator commands.

## Testing

### Unit (pytest, no ROS)

Extract the arbitration into a plain class (no ROS imports) so it is testable
without a running master. Given a sequence of `(source, timestamp)` inputs and
`set_mode` calls, assert:

- Only the selected source's messages are forwarded; the other source's are
  dropped.
- Switching mode changes which source forwards, immediately.
- `stale` is raised after `stale_timeout` of silence on the selected source and
  cleared when it resumes.
- Silence on the *unselected* source never affects output or the stale flag.
- An unknown mode string is rejected (mode unchanged).

### In-sim (main conversation, from a clean kill)

1. Bring up the sim clean (per `agents-implement-main-runs-sim` — full teardown
   first). Start `abs_fix_selector` and both feeders.
2. GPS mode by default: confirm `abs_fix` tracks `gps_fix` and the robot
   navigates a goal.
3. Mid-drive `mode landmark`: confirm the reply's `prev_topic` is non-empty,
   `/abs_fix_mode` latches `landmark`, `abs_fix` now tracks `landmark_fix`, and
   the robot keeps navigating.
4. Silence the selected source (stop its feeder input): confirm `/abs_fix_mode`
   shows `:stale`, `abs_fix` stops updating, and the other source is **not**
   substituted.

## Out of scope (YAGNI)

- The **automatic** health-based supervisor — the next feature; this design
  makes it a drop-in second client of `/set_abs_fix_mode`.
- Collapsing `move_base_gps_map.launch` and `move_base_landmark.launch` into one.
- Blending/fusing both sources — strictly one at a time.
