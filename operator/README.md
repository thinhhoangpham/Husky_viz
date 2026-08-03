# Operator container (remote ROS operator)

A Docker container with its own IP that acts as a REMOTE operator against the
natively-run stock Husky: it sends ONE move_base goal (reach an odom-frame
point, then stop), watches telemetry to console, and writes a normal-baseline
CSV (`operator_run.csv`) whose columns are the union of every baseline signal
the repo's attack CSVs log — so each attack's plot can overlay a normal series.

This container contains NO attack code and does NO robot bring-up. It is the
benign counterpart of `attacker/`, and the operator↔robot wire it establishes
is the future Tier 3 target. Design:
`docs/superpowers/specs/2026-08-02-operator-container-design.md`; parent doc:
`docs/attacker-network-simulation.md` §10.

Verified end-to-end on 2026-08-02: robot drove straight, console streamed
`state=ACTIVE ... dist_to_goal` decreasing, ended `Final move_base state:
SUCCEEDED`, and `operator_run.csv` was written to the repo root.

## Phase 0 — Host prep + world (host terminal 1)

```bash
ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
echo "docker0 gateway = ${ROBOT_HOST_IP}"
export ROS_IP="${ROBOT_HOST_IP}"
export ROS_MASTER_URI="http://${ROS_IP}:11311"
./load-park-stock-husky.sh          # world only (no robot), master off localhost
```
Without `ROS_IP` the master advertises a hostname/127.0.0.1: the container
connects (nmap-level reachability succeeds) but topic handshakes hang. Export
both `ROS_IP` and `ROS_MASTER_URI` BEFORE launching the sim.

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
docker compose run --rm operator ./operator/operate.py --goal-x 10 --goal-y 0
```
Watch the robot drive to the point in Gazebo; the console streams telemetry;
`operator_run.csv` lands in the repo root. Override output with `--csv path`.

The script is `operator/operate.py`, not `operator.py` — it was named to avoid
shadowing Python's stdlib `operator` module. Every invocation must use the
full path `./operator/operate.py`.

**`--help` gotcha:** `docker compose run --rm operator ./operator/operate.py
--help` does NOT work — `docker compose run` intercepts a bare `--help` as its
own flag rather than passing it through. To get help output, put it after a
`--` separator:
```bash
docker compose run --rm operator ./operator/operate.py -- --help
```
Normal arguments like `--goal-x`/`--goal-y` do not need the `--` separator.

## Verified CSV

```bash
head -1 ../operator_run.csv
wc -l ../operator_run.csv
```
Header is exactly:
```
elapsed_time,fused_x,fused_y,fused_yaw,fused_yaw_deg,planner_linear_x,planner_angular_z,ctrl_linear_x,ctrl_angular_z,ref_x,ref_y
```
11 columns, multiple data rows; `fused_x` climbs toward the goal over the run;
`ref_x`/`ref_y` stay constant at the requested goal.

## Normal-vs-attack comparison

`operator_run.csv` is the NORMAL baseline. Each attack CSV shares the
`elapsed_time` clock; a plot picks the columns it needs (e.g. odom attack uses
`fused_x`/`fused_y`; param uses `ctrl_linear_x`; cmd_vel uses
`planner_*`/`ctrl_*`). Attack-injected columns (`fake_yaw_deg`, `value_written`,
`d_*`) have no baseline and stay in the attack CSVs. NOTE: this stock robot has
no GPS/compass, so its position/heading are EKF-odom — comparable in kind, not
the same sensor, to the compass/GPS attacks (those target the park-GPS robot).
GPS support is deferred.
