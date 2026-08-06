# IMU Spoofing Attack — Design Spec

**Date:** 2026-07-28
**Context:** Authorized security research. Reproduces and extends "Attack #3: IMU Data
Spoofing" from the Raider Security Stage 3 Report (AVSVA, Army Research Lab capstone),
against the stock dead-reckoning Husky topology brought up by this repo's base scripts.
Everything runs in Gazebo simulation.

## Goal

Deliver **two separate standalone attack scripts** that spoof `sensor_msgs/Imu` on
`/imu/data` (the topic the stock EKF fuses):

1. `attack_imu_faithful.py` — a literal reproduction of the report's described attack.
2. `attack_imu_derail.py` — a version tuned to actually derail `move_base` navigation.

Neither modifies the base scripts. Each is launched by hand in a **third terminal**
alongside the existing two-terminal workflow.

## Grounding: the actual target topology (verified on disk)

Verified against the stock ROS Noetic install, not assumed:

- **Real IMU source** — `husky.urdf.xacro:377-388`: Gazebo plugin
  `libhector_gazebo_ros_imu.so`, `updateRate = 50.0` Hz, `bodyName = base_link`,
  `topicName = imu/data`. So genuine messages arrive at **50 Hz** with **`frame_id =
  base_link`**. Our spoofer must use `frame_id = base_link` to match, and can dominate
  the stream by publishing well above 50 Hz.
- **EKF fusion config** — `husky_control/config/localization.yaml`:
  - `world_frame: odom`, `two_d_mode: true`, `frequency: 50`.
  - `odom0: husky_velocity_controller/odom` (wheel odom).
  - `imu0: imu/data` with:
    - `imu0_config` fuses **orientation roll/pitch/yaw** (idx 3,4,5) and **angular
      velocity roll/pitch/yaw-rate** (idx 9,10,11).
    - **Does NOT fuse linear acceleration** (idx 12,13,14 = false).
    - `imu0_differential: true` — the EKF integrates the *change* in the fused
      orientation, not its absolute value.
    - `imu0_remove_gravitational_acceleration: true`.
- **Victim signal** — `send_mapless_goal.py:53` drives `move_base` off
  `/odometry/filtered`, the EKF output. Corrupting `/imu/data` corrupts
  `/odometry/filtered`, which corrupts planning/tracking. No GPS/compass exists in this
  topology to correct it.

### Decisive consequence for the attack design

The report's **headline linear-acceleration spoof (~50 m/s², >5g) is fused out** — this
EKF ignores IMU linear acceleration entirely. The IMU channel that actually moves
`/odometry/filtered` here is **orientation / yaw-rate**. Combined with
`imu0_differential: true`, a **coherent yaw-rate bias** is integrated by the filter and
walks the estimated heading away from truth, whereas **large impossible spikes** are
mostly self-cancelling under differential integration + 2D mode. This is precisely why
the two deliverables behave differently, and it is evidence-backed, not a guess.

## Deliverable 1 — `attack_imu_faithful.py`

Literal reproduction of Report §3, for fidelity to the documented attack.

- Publishes `sensor_msgs/Imu` to `/imu/data`.
- **Impossible values** as documented: angular velocity ~10 rad/s (~573°/s), linear
  acceleration ~50 m/s² (>5g).
- **Sinusoidal modulation** of those values ("persistent oscillating disturbances that
  confound sensor fusion" — report's language).
- Valid, well-formed message: `header.stamp = rospy.Time.now()`, `frame_id =
  base_link`, populated orientation quaternion, angular velocity, linear acceleration,
  and non-zero covariance diagonals so the EKF does not reject it as unset (`-1`).
- Publish rate: fixed **100 Hz** constant (report frames this as raw injection; rate is
  not a tunable here).
- Logs the injected values via `rospy.loginfo` (matches "instrumented with detailed
  logging").

**Honest property, documented in the script header:** because this EKF does not fuse
linear acceleration and integrates orientation differentially, the brute-force impossible
values will *not* reliably derail navigation. That is a truthful finding about the
report's exact attack against this exact topology — not a bug to hide. Making it
effective is Deliverable 2's job.

## Deliverable 2 — `attack_imu_derail.py`

Same topic and message type; attack **profile** designed for reliable navigation
derailment given the verified EKF config.

- **Coherent yaw-rate bias**: inject a steady angular-velocity yaw-rate offset (plus a
  slow drift term), consistent across messages, so the differential EKF integrates it
  into a growing heading error. This is what makes `/odometry/filtered` believe the
  robot is turning when it is not, so `move_base` steers to correct a phantom error and
  drives off the intended path.
- **Plausible magnitudes**, not impossible ones: values in a range the filter treats as
  real (so it is trusted and integrated), rather than obvious garbage.
- **High sustained rate** (default **200 Hz**) to dominate the genuine 50 Hz stream in
  each fusion cycle.
- **Tunables as top-of-file constants**: yaw-rate bias magnitude, drift rate, publish
  rate — so impact can be dialed in.
- Orientation quaternion is published consistently with the injected yaw-rate so the
  fused-orientation channel and angular-velocity channel agree (avoids the EKF averaging
  a self-contradictory message toward zero).

**Honest limitation, documented in the header:** starting magnitudes are chosen from the
verified EKF config, but exact values may need **one tuning pass** against a live run to
get a visibly decisive derailment. The script is built to make that trivial (constants at
top).

## Shared observability (both scripts)

Chosen level: "monitor the victim" (not a full exit-summary harness).

- Subscribe to `/odometry/filtered`.
- Capture a **pre-attack baseline** pose + heading during the first ~2 s before
  injecting (so there is a reference).
- During the attack, log at ~1 Hz the **divergence** of the current filtered
  pose/heading from the baseline, alongside the spoofed values being published. This
  quantifies the effect from the attacker's side, complementing
  `send_mapless_goal.py`'s own `dist_to_goal` output.
- Clean Ctrl-C shutdown: stop publishing, print a short final line.

## Workflow (three terminals)

| Terminal | Command | Role |
|---|---|---|
| 1 | `./load-park-stock-husky.sh` | park world (unchanged) |
| 2 | `./send_mapless_goal.py` | spawn Husky, plan, drive to goal (unchanged) |
| 3 | `./attack_imu_faithful.py` *or* `./attack_imu_derail.py` | the spoofer |

The attacker is an ordinary ROS node that joins the open graph and publishes to a topic
the EKF trusts — the exact "no authentication / unrestricted topic access" gap the report
demonstrates. It replaces nothing; it appears in `rosnode list` alongside the robot's
own nodes.

## Explicitly out of scope (YAGNI)

- No changes to `load-park-stock-husky.sh` or `send_mapless_goal.py`.
- No runner/sequencer script — the operator controls timing by when they launch
  terminal 3.
- No GUI, no bag recording, no detection/analysis side (that is the report's separate
  BagAnalyzer).
- No full drift-metric-at-exit comparison harness.
- No attacks on other topics/param server/XMLRPC (those are separate report items).

## Files to create

- `attack_imu_faithful.py` (repo root, executable, `#!/usr/bin/env python3`).
- `attack_imu_derail.py` (repo root, executable, `#!/usr/bin/env python3`).

Both follow `send_mapless_goal.py`'s established conventions: `rospy.init_node(...)`,
module-level constants, clear docstrings explaining *why*, clean signal-based shutdown.

## Open question deferred to tuning, not design

Exact yaw-rate bias magnitude / rate for `attack_imu_derail.py` to produce a decisive
visible derailment. Resolved empirically in one run after implementation; constants are
exposed for this.
