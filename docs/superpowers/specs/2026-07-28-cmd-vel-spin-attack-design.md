# CMD_VEL Spin Attack — Design

**Date:** 2026-07-28
**Status:** Approved for implementation

## Summary

A simulation-only security demonstration that reproduces the security report's
**"CMD_VEL Topic Injection"** attack (report §1, "spin attack") against the
stock playpen Husky. While the robot is autonomously driving to a move_base
goal, a standalone attacker node floods the wheel-command topic with a
spin-in-place command at a higher rate than the legitimate planner, so the
wheels obey the attacker and the robot abandons its goal and spins.

This is the third attack script in the repo and deliberately follows the exact
pattern of `attack_odom.py` and `attack_compass.py`.

## Faithfulness to the report

The report (§1, page 17) specifies this attack literally:

- **Topic:** `/husky_velocity_controller/cmd_vel` (the controller's direct input)
- **Rate:** 30 Hz, chosen to overwhelm the legitimate ~20 Hz driver
- **Payload:** `linear.x = 0.0` (stop) and `angular.z = 2.0 rad/s` (rapid spin)
- **Signature:** "spin attack" — zero linear velocity + high angular velocity
- **Intended behavior (report's words):** the robot "loses its intended forward
  trajectory and begins spinning in place," demonstrating "complete loss of
  motion control."

These exact values are the script's defaults.

### One documented deviation from the report's text

The report's **detection** paragraph for this attack claims detection via
"sudden position discontinuities exceeding 5 meters." That does not fit a pure
spin: with `linear.x = 0.0` the robot does not translate, so position should not
jump 5 m. That 5 m language appears to be copy-pasted from the Odometry Spoofing
attack (§2, which really does fabricate a position jump). The honest, correct
signature for THIS attack is the report's own first paragraph: **zero linear
velocity + high angular velocity.** The telemetry is built around that real
signature, and this deviation is noted in the script docstring rather than
silently reproducing an incorrect claim.

## Victim

`send_mapless_goal.py` — brings up the stock Husky and drives it to a move_base
goal ~15 m ahead. move_base is a real path planner publishing to `/cmd_vel`
(the twist_mux priority-1 `external` slot), and twist_mux remaps its output to
`/husky_velocity_controller/cmd_vel`, the controller's input.

Because move_base is a genuine closed-loop planner, this victim makes the
report's "path planner misled / loses motion control" impact directly
observable: the planner keeps commanding forward motion on `/cmd_vel` while the
wheels obey the attacker's spin on the controller-input topic. The result is a
last-message-wins tug-of-war that the attacker wins by out-rating the planner.

## The attack script — `attack_cmd_vel.py`

A standalone `rospy` node, same shape as the existing two attack scripts.

### What it does

- Publishes `geometry_msgs/Twist` to `/husky_velocity_controller/cmd_vel` at
  30 Hz with `linear.x = 0.0`, `angular.z = 2.0`.
- Runs until `Ctrl-C` or an optional `--duration`.
- On stop: simply ceases publishing. No corrective message is sent — once the
  faster stream stops, move_base's own commands resume dominating on their own.
  (Same clean-shutdown discipline as `attack_odom.py` / `attack_compass.py`.)

### Command-line knobs (defaults = the report)

| Flag | Default | Meaning |
|---|---|---|
| `--rate` | `30.0` | publish rate in Hz (report's 30 Hz; out-rates the ~20 Hz driver) |
| `--linear` | `0.0` | `linear.x` (report's stop) |
| `--angular` | `2.0` | `angular.z` rad/s (report's rapid spin) |
| `--duration` | `0.0` | seconds to run; 0 = until Ctrl-C |
| `--topic` | `/husky_velocity_controller/cmd_vel` | target topic (exposed for experiments, same as the other scripts; default = report) |
| `--csv` | `attack_cmd_vel_report.csv` | telemetry output path |

`--rate` must be `> 0` (argparse error otherwise), matching the other scripts.

### Telemetry / proof (NO ground truth — hard project rule)

The script never reads Gazebo ground truth. It records, once per second, to a
CSV so the attack can be proven after the fact:

- `elapsed_time`
- **The actual command reaching the wheels:** subscribe to
  `/husky_velocity_controller/cmd_vel` and log the latest `linear.x` /
  `angular.z`. During a successful attack these read ~`(0.0, 2.0)` — the spin
  signature — instead of the planner's forward command.
- **The planner still fighting:** subscribe to `/cmd_vel` (move_base's output,
  pre-twist_mux) and log its latest `linear.x` / `angular.z`. This shows the
  planner is still commanding forward motion even though the wheels ignore it —
  the report's "path planner misled / loss of control" story, made visible.

CSV columns:
`elapsed_time, ctrl_linear_x, ctrl_angular_z, planner_linear_x, planner_angular_z`

This answers, in one file: "Is the wheel command the attacker's spin while the
planner is still trying to go forward?" — i.e. did the attack take control away
from the planner.

Note on the self-publish loop: the attacker both publishes to and (for
telemetry) subscribes to `/husky_velocity_controller/cmd_vel`, so it will see
its own messages there. That is fine and expected — the whole point of the
`ctrl_*` columns is to confirm the attacker's spin is what's on that topic. No
sentinel/filtering is needed (unlike `attack_odom.py`, which needed to isolate
the real yaw rate); here we WANT to observe the winning command, whoever sent it.

## How it is run

Three terminals (matching the existing workflow):

1. `./load-park-stock-husky.sh` — loads the park world.
2. `source /opt/ros/noetic/setup.bash && ./send_mapless_goal.py` — brings up the
   robot and starts driving to the goal (the victim).
3. `python3 attack_cmd_vel.py` — runs the attack. Watch the robot stop tracking
   its goal and spin in place; `Ctrl-C` to stop and let it recover.

## Non-goals (YAGNI)

- No one-button launcher that starts victim + attack together. Run by hand, like
  the other attacks.
- No twist_mux priority-slot variant. The `--topic` flag technically allows
  pointing elsewhere, but the design and default are strictly the report's
  controller-input topic.
- No detection/analysis code. This script only performs and records the attack;
  analysis (the report's `BagAnalyzer` role) is out of scope here.

## Files

- **New:** `attack_cmd_vel.py` (repo root, alongside the other two attack scripts)
- **New:** this design doc.
