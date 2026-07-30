# Tier 2 Network Attacker (separate container)

A Docker container with its own IP that discovers and reaches the **natively
run** Husky ROS master, then injects via the repo's `attack_*.py` unchanged.
Design: `docs/superpowers/specs/2026-07-30-tier2-attacker-container-design.md`.

## Phase 0 — Host prep (run on the host BEFORE the sim)

Find the docker0 gateway IP and rebind the master off localhost:

```bash
ROBOT_HOST_IP="$(ip -4 addr show docker0 | awk '/inet /{print $2}' | cut -d/ -f1)"
echo "docker0 gateway = ${ROBOT_HOST_IP}"
export ROS_IP="${ROBOT_HOST_IP}"
export ROS_MASTER_URI="http://${ROS_IP}:11311"
roslaunch <your husky sim>          # master now reachable off localhost
```

Without `ROS_IP` the master advertises `127.0.0.1`; nmap will pass but the
container's `rostopic` calls hang (see Phase 2 diagnostics).

## Build

```bash
cd attacker
export ROBOT_HOST_IP        # from Phase 0
docker compose build
```

## Phase 1 — Reachability

```bash
docker compose run --rm attacker ./scan.sh
```
Expect exactly the host on `11311/tcp open`. FAIL → fix Phase 0 / firewall.

## Phase 2 — Enumeration

```bash
docker compose run --rm attacker ./enum.sh
```
Expect node/topic/param lists (`/husky_velocity_controller/cmd_vel`,
`/compass/data`, `/navsat/fix`, …). Hang here after Phase 1 passed → the
host `ROS_IP` fix in Phase 0.

## Phase 3 — Exploitation

```bash
docker compose run --rm attacker ./attack.sh cmd_vel  --duration 8
docker compose run --rm attacker ./attack.sh compass  --yaw 1.5708
docker compose run --rm attacker ./attack.sh odom
docker compose run --rm attacker ./attack.sh param
```
Watch the robot obey in the Gazebo (noVNC) view. CSVs land under `/repo`
(the mounted repo) — pass `--csv /tmp/x.csv` to redirect.

## Recon vs. attack

Phases 1–2 are reconnaissance (port scan + graph read — discovery, not harm).
Phase 3 is the attack payload. Tier 3 (on-the-wire MITM) is out of scope.
