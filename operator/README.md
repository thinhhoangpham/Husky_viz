# Operator container (remote ROS operator)

A Docker container with its own IP that acts as a REMOTE operator against the
natively-run GPS-enabled park Husky: an interactive REPL that sends GPS-goal
move_base actions, supports teleop/stop/e-stop intervention, watches telemetry
to console, and writes a normal-baseline CSV (`operator_run.csv`) whose columns
are the union of every baseline signal the repo's attack CSVs log — so each
attack's plot can overlay a normal series. The container also runs its own
RViz instance (served over noVNC) so the operator can watch the robot/plan
without touching the Gazebo GUI.

This container contains NO attack code and does NO robot bring-up. It is the
benign counterpart of `attacker/`, and the operator↔robot wire it establishes
is the future Tier 3 target. Design:
`docs/superpowers/specs/2026-08-02-operator-container-design.md`; parent doc:
`docs/attacker-network-simulation.md` §10.

## Phase 0 — Host prep + world (host terminal 1)

```bash
# One-time: create the shared LAN both the operator and attacker containers join.
docker network create husky_lan 2>/dev/null || true

# The gateway of husky_lan is how containers reach the NATIVE robot/master.
GW="$(docker network inspect husky_lan --format '{{(index .IPAM.Config 0).Gateway}}')"
echo "husky_lan gateway = ${GW}"

# Launch the native GPS park sim advertising on that gateway (NOT docker0).
export ROS_IP="${GW}"
export ROS_MASTER_URI="http://${ROS_IP}:11311"
./load-park-stock-husky.sh          # bring up the GPS park world + robot
```
Without `ROS_IP` the master advertises a hostname/127.0.0.1: the container
connects (nmap-level reachability succeeds) but topic handshakes hang. Export
both `ROS_IP` and `ROS_MASTER_URI` BEFORE launching the sim.

## Phase 1 — Bring up the operator container (host terminal 2)

```bash
cd operator
export ROBOT_HOST_IP="$(docker network inspect husky_lan --format '{{(index .IPAM.Config 0).Gateway}}')"
docker compose up -d
```
The container starts its own Xvfb + fluxbox + x11vnc + noVNC + RViz stack
(entrypoint, guarded by `OPERATOR_RVIZ=1` — default on). Open
**http://localhost:6080/vnc.html** in a browser to watch the operator's RViz
view of the robot and its plan. Disable the visual stack with
`OPERATOR_RVIZ=0 docker compose up -d` if only the REPL is needed.

## Phase 2 — Run the operator REPL (host terminal 3)

```bash
docker compose exec operator ./operator/operate.py --lat <LAT> --lon <LON>
```
The script is `operator/operate.py`, not `operator.py` — it was named to avoid
shadowing Python's stdlib `operator` module. Every invocation must use the
full path `./operator/operate.py`. `--lat`/`--lon` send an initial GPS goal;
omit them to start idle at the `operator>` prompt. Telemetry streams to
console and `operator_run.csv` lands in the repo root (override with
`--csv path`).

### REPL commands

| Command | Effect |
|---|---|
| `goal <lat> <lon>` | Send a new GPS goal as a move_base action. |
| `cancel` | Cancel the active goal, return to AUTO mode. |
| `teleop` | Enter manual teleop sub-mode (drive with keyboard-style input). |
| `stop` | Zero velocity immediately, mode -> STOPPED. |
| `estop` | Engage the e-stop (latched). |
| `release` | Release the e-stop. |
| `auto` | Return to AUTO mode. |
| `status` | Print current goal, nav status, mode, heartbeat age. |
| `quit` | Exit the REPL. |

## CSV output

```bash
head -1 ../operator_run.csv
wc -l ../operator_run.csv
```
Columns are the union of every baseline signal the repo's attack CSVs log
(e.g. `elapsed_time,fused_x,fused_y,fused_yaw,fused_yaw_deg,planner_linear_x,
planner_angular_z,ctrl_linear_x,ctrl_angular_z,ref_x,ref_y`).

## Normal-vs-attack comparison

`operator_run.csv` is the NORMAL baseline. Each attack CSV shares the
`elapsed_time` clock; a plot picks the columns it needs (e.g. odom attack uses
`fused_x`/`fused_y`; param uses `ctrl_linear_x`; cmd_vel uses
`planner_*`/`ctrl_*`). Attack-injected columns (`fake_yaw_deg`, `value_written`,
`d_*`) have no baseline and stay in the attack CSVs. NOTE: this stock robot has
no GPS/compass, so its position/heading are EKF-odom — comparable in kind, not
the same sensor, to the compass/GPS attacks (those target the park-GPS robot).
GPS support is deferred.
