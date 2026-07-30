# Tier 2 Attacker Container Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a self-contained `attacker/` Docker entity that, from its own container IP, discovers and reaches the natively-run Husky ROS master and injects via the existing `attack_*.py` scripts unchanged.

**Architecture:** A `ros:noetic-ros-core` container joins the host's default `docker0` bridge and reaches the native master at the docker0 gateway IP. Four ordered phases — host prep (Phase 0, doc-only), reachability (`scan.sh`), enumeration (`enum.sh`), exploitation (`attack.sh`) — each independently verifiable so a failure names the broken layer. The repo's attack scripts are bind-mounted, never copied.

**Tech Stack:** Docker + docker-compose, ROS Noetic (`ros:noetic-ros-core`), `nmap`, `iproute2`, bash, existing `rospy` attack scripts.

## Global Constraints

- **Tier 2 only.** No Tier 3 (MITM/sniff), no trigger/geofence, no Gazebo attacker model. (Spec §7)
- **Attack scripts run UNCHANGED.** `attack_*.py` are bind-mounted read-only from the repo root; the plan never edits them. (Spec §1, §3)
- **No Gazebo in the container.** Base `ros:noetic-ros-core` only — `rospy` + tooling. (Spec §2)
- **All new files live in `attacker/`.** The only change outside it is one pointer note in `docs/attacker-network-simulation.md`. (Spec §3)
- **Native master, docker0 bridge.** The container reaches the host master via the docker0 gateway IP; `ROBOT_HOST_IP` is supplied at runtime, never hardcoded. (Spec §1, §2)
- **`ROS_IP` gotcha.** Both sides must set `ROS_IP`; the host advertises its docker0 gateway IP, the container advertises its own IP, or the remote TCPROS handshake fails silently after nmap succeeds. (Spec §2)
- **Verification is inspection + runbook, not a test suite.** Docker is not reachable from the authoring session; each task ends with a concrete inspection check and the host/container command + expected output that the user runs. (Spec §6)
- **Per project policy**, the actual file authoring (Dockerfile, compose, shell) is code — route each task's authoring to `senior-fullstack-dev`.

**Known attack-script CLI facts (for `attack.sh`, verified in-repo):**
- `attack_cmd_vel.py` — flags: `--rate --linear --angular --duration --topic --csv`; node `attack_cmd_vel`.
- `attack_compass.py` — flags: `--rate --yaw --yaw-offset --duration --topic --csv`; node `attack_compass`.
- `attack_odom.py` — flags: `--rate --duration --topic --csv --pose-x --pose-y --vx`; node `attack_odom`.
- `attack_param.py` — flags: `--param --sequence --dwell --loop --csv`; node `attack_param`.
- All four write a CSV (default in CWD) → the container's working dir must be writable, or `--csv /tmp/...`.

---

### Task 1: Container image — `attacker/Dockerfile` + `entrypoint.sh`

Builds the attacker host: ROS core + nmap, with `ROS_IP` auto-derived at start.

**Files:**
- Create: `attacker/Dockerfile`
- Create: `attacker/entrypoint.sh`

**Interfaces:**
- Consumes: nothing (first task).
- Produces: image build context under `attacker/`; an entrypoint that (a) exports `ROS_IP` = the container's own primary IP, (b) execs `"$@"` or an interactive `bash` if no command. Later tasks assume `ROS_IP` is already exported inside any container shell/command.

- [ ] **Step 1: Write `attacker/Dockerfile`**

```dockerfile
FROM ros:noetic-ros-core

# nmap = Phase 1 reachability scan; iproute2 = derive container IP for ROS_IP.
RUN apt-get update \
    && apt-get install -y --no-install-recommends nmap iproute2 \
    && rm -rf /var/lib/apt/lists/*

# Attack scripts are bind-mounted here at runtime (see docker-compose.yml).
WORKDIR /repo
COPY entrypoint.sh /usr/local/bin/entrypoint.sh
RUN chmod +x /usr/local/bin/entrypoint.sh

ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]
CMD ["bash"]
```

- [ ] **Step 2: Write `attacker/entrypoint.sh`**

```bash
#!/usr/bin/env bash
# Derive this container's own IP and advertise it as ROS_IP so the remote
# master hands peers a reachable callback address (the ROS_IP gotcha, spec §2).
set -euo pipefail

source /opt/ros/noetic/setup.bash

# First non-loopback IPv4 of the container.
CONTAINER_IP="$(ip -4 -o addr show scope global | awk '{print $4}' | cut -d/ -f1 | head -n1)"
export ROS_IP="${CONTAINER_IP}"

echo "[attacker] ROS_IP=${ROS_IP}"
echo "[attacker] ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}"

exec "$@"
```

- [ ] **Step 3: Verify by inspection**

Confirm: base is `ros:noetic-ros-core` (no Gazebo); only `nmap` + `iproute2` added; scripts are NOT `COPY`d (bind-mounted later); entrypoint exports `ROS_IP` from the container IP and `exec "$@"` so `CMD`/compose commands run under it.

- [ ] **Step 4: Build check (user runs)**

Run (host, in `attacker/`): `docker build -t husky-attacker .`
Expected: image builds; `docker run --rm husky-attacker bash -lc 'which nmap && which rosnode'` prints both paths.

- [ ] **Step 5: Commit**

```bash
git add attacker/Dockerfile attacker/entrypoint.sh
git commit -m "feat(attacker): ROS-core + nmap container image with ROS_IP auto-derive"
```

---

### Task 2: Compose wiring — `attacker/docker-compose.yml`

Puts the container on docker0 with its own IP, points it at the host master, mounts the repo.

**Files:**
- Create: `attacker/docker-compose.yml`

**Interfaces:**
- Consumes: the `husky-attacker` image from Task 1 (built here via `build:`).
- Produces: a service `attacker` where inside the container `ROS_MASTER_URI=http://${ROBOT_HOST_IP}:11311`, the repo is bind-mounted at `/repo`, and `/repo` is writable (for attack CSVs). Later tasks run `docker compose run --rm attacker <cmd>`.

- [ ] **Step 1: Write `attacker/docker-compose.yml`**

```yaml
services:
  attacker:
    build: .
    image: husky-attacker
    # Default bridge (docker0): the container gets its own IP and reaches the
    # NATIVE host master via the docker0 gateway IP passed in ROBOT_HOST_IP.
    environment:
      # Supply at runtime: ROBOT_HOST_IP=<docker0 gateway IP>. No hardcoding.
      ROS_MASTER_URI: "http://${ROBOT_HOST_IP}:11311"
    volumes:
      # Repo mounted rw so bind-mounted attack_*.py can write their CSVs.
      - ../:/repo
    working_dir: /repo
    # Keep STDIN/TTY for interactive `docker compose run` sessions.
    stdin_open: true
    tty: true
```

- [ ] **Step 2: Verify by inspection**

Confirm: no `network_mode`/custom network (uses default bridge → own IP); `ROS_MASTER_URI` built from `${ROBOT_HOST_IP}`; repo mounted rw at `/repo`; `working_dir: /repo` so CSVs land in a writable place; no ports published (attacker dials out, needs no inbound).

- [ ] **Step 3: Config check (user runs)**

Run (host, in `attacker/`): `ROBOT_HOST_IP=172.17.0.1 docker compose config`
Expected: rendered config shows `ROS_MASTER_URI: http://172.17.0.1:11311` and the `../:/repo` mount. (IP is illustrative.)

- [ ] **Step 4: Commit**

```bash
git add attacker/docker-compose.yml
git commit -m "feat(attacker): compose service on docker0 targeting native host master"
```

---

### Task 3: Phase 1 — `attacker/scan.sh` (reachability)

nmap sweep for the master; the first checkpoint, isolating network failure from ROS failure.

**Files:**
- Create: `attacker/scan.sh`

**Interfaces:**
- Consumes: runs inside the container (Tasks 1–2); reads `ROBOT_HOST_IP` from env.
- Produces: exit 0 + printed "open" line when a host answers on 11311; non-zero + a Phase-0 hint when none do. No ROS calls.

- [ ] **Step 1: Write `attacker/scan.sh`**

```bash
#!/usr/bin/env bash
# PHASE 1 — REACHABILITY. Pure network scan; no ROS calls (spec §4).
# Sweeps the docker0 subnet for an open ROS master port (11311).
set -euo pipefail

: "${ROBOT_HOST_IP:?set ROBOT_HOST_IP to the docker0 gateway IP (see README Phase 0)}"

# Scan the /24 the gateway lives on, e.g. 172.17.0.1 -> 172.17.0.0/24.
SUBNET="$(echo "${ROBOT_HOST_IP}" | awk -F. '{print $1"."$2"."$3".0/24"}')"

echo "[scan] nmap -p 11311 ${SUBNET}"
OUT="$(nmap -p 11311 --open "${SUBNET}")"
echo "${OUT}"

if echo "${OUT}" | grep -q "11311/tcp open"; then
  echo "[scan] OK: found an open ROS master on 11311."
  echo "[scan] Next: run ./enum.sh to read the graph (Phase 2)."
  exit 0
else
  echo "[scan] FAIL: no host answered on 11311." >&2
  echo "[scan] Fix Phase 0 on the host: export ROS_IP + ROS_MASTER_URI to the" >&2
  echo "[scan] docker0 gateway IP before roslaunch, and check the host firewall." >&2
  exit 1
fi
```

- [ ] **Step 2: Verify by inspection**

Confirm: derives `/24` from `ROBOT_HOST_IP`; uses only `nmap` (no `rostopic`); success gated on `11311/tcp open`; failure message points back to Phase 0 host rebind + firewall; `set -euo pipefail`.

- [ ] **Step 3: Runbook check (user runs, sim up + host rebound)**

Run (host, in `attacker/`): `ROBOT_HOST_IP=<docker0 gw> docker compose run --rm attacker ./scan.sh`
Expected: nmap lists exactly the host on `11311/tcp open`; prints `[scan] OK`. With the sim down: prints `[scan] FAIL` and exits non-zero.

- [ ] **Step 4: Commit**

```bash
git add attacker/scan.sh
git commit -m "feat(attacker): Phase 1 scan.sh — nmap reachability checkpoint"
```

---

### Task 4: Phase 2 — `attacker/enum.sh` (enumeration)

Reads the ROS graph the master volunteers; the second checkpoint, catching the ROS_IP gotcha.

**Files:**
- Create: `attacker/enum.sh`

**Interfaces:**
- Consumes: `ROS_MASTER_URI` (from compose) and `ROS_IP` (from entrypoint) already set in the container.
- Produces: prints topic/node/param listings; a targeted ROS_IP-gotcha message if a list call times out after scan succeeded.

- [ ] **Step 1: Write `attacker/enum.sh`**

```bash
#!/usr/bin/env bash
# PHASE 2 — ENUMERATION. Read-only ROS graph read; the master volunteers this
# to any caller (spec §4). Run only after scan.sh (Phase 1) passes.
set -uo pipefail
source /opt/ros/noetic/setup.bash

echo "[enum] ROS_MASTER_URI=${ROS_MASTER_URI:-<unset>}  ROS_IP=${ROS_IP:-<unset>}"

# Short timeout so the ROS_IP gotcha surfaces as a clear message, not a hang.
run() {
  echo "=== $* ==="
  if ! timeout 15 "$@"; then
    echo "[enum] '$*' failed/timed out." >&2
    echo "[enum] nmap passed but ROS calls hang => the ROS_IP callback gotcha" >&2
    echo "[enum] (spec §2): the HOST master must export ROS_IP=<docker0 gw IP>" >&2
    echo "[enum] before roslaunch, else it advertises 127.0.0.1 to this peer." >&2
    return 1
  fi
}

run rosnode list
run rostopic list
run rosparam list
echo "[enum] OK: graph read complete. Next: ./attack.sh <name> (Phase 3)."
```

- [ ] **Step 2: Verify by inspection**

Confirm: read-only (`list` only, no `pub`/`set`); `timeout` wraps each call so the gotcha is a message not a hang; the failure text names the HOST-side `ROS_IP` fix specifically; sources the ROS setup.

- [ ] **Step 3: Runbook check (user runs, sim up)**

Run (host, in `attacker/`): `ROBOT_HOST_IP=<docker0 gw> docker compose run --rm attacker ./enum.sh`
Expected: prints node/topic/param lists including `/husky_velocity_controller/cmd_vel`, `/compass/data`, `/navsat/fix`. If it times out here after scan passed → the printed ROS_IP hint is the fix.

- [ ] **Step 4: Commit**

```bash
git add attacker/enum.sh
git commit -m "feat(attacker): Phase 2 enum.sh — ROS graph read with ROS_IP-gotcha diagnostics"
```

---

### Task 5: Phase 3 — `attacker/attack.sh` (exploitation)

Dispatches a named, unchanged `attack_*.py` against the remote master.

**Files:**
- Create: `attacker/attack.sh`

**Interfaces:**
- Consumes: bind-mounted `/repo/attack_<name>.py`; `ROS_MASTER_URI`/`ROS_IP` from the container env.
- Produces: runs the chosen script with pass-through args; validates name and file existence first.

- [ ] **Step 1: Write `attacker/attack.sh`**

```bash
#!/usr/bin/env bash
# PHASE 3 — EXPLOITATION. Runs an existing attack_*.py UNCHANGED against the
# remote master (spec §1, §3). Usage: ./attack.sh <name> [script args...]
set -euo pipefail
source /opt/ros/noetic/setup.bash

NAME="${1:-}"; shift || true
case "${NAME}" in
  cmd_vel|compass|odom|param) ;;
  *)
    echo "usage: ./attack.sh <cmd_vel|compass|odom|param> [args...]" >&2
    exit 2 ;;
esac

SCRIPT="/repo/attack_${NAME}.py"
if [[ ! -f "${SCRIPT}" ]]; then
  echo "[attack] ${SCRIPT} not found — is the repo mounted at /repo?" >&2
  exit 1
fi

echo "[attack] firing ${SCRIPT} against ${ROS_MASTER_URI} (ROS_IP=${ROS_IP})"
exec python3 "${SCRIPT}" "$@"
```

- [ ] **Step 2: Verify by inspection**

Confirm: name validated against exactly the four scripts; existence checked before run; `exec python3` passes through all extra args unchanged; never edits the script; the mount-missing case is called out.

- [ ] **Step 3: Runbook check (user runs, sim up + driving)**

Run (host, in `attacker/`): `ROBOT_HOST_IP=<docker0 gw> docker compose run --rm attacker ./attack.sh cmd_vel --duration 8`
Expected: script prints its telemetry; the robot spins in place / abandons its goal in the Gazebo (noVNC) view. Bad name → usage + exit 2.

- [ ] **Step 4: Commit**

```bash
git add attacker/attack.sh
git commit -m "feat(attacker): Phase 3 attack.sh — dispatch unchanged attack_*.py"
```

---

### Task 6: Runbook — `attacker/README.md` + discoverability note

Ties the four phases together and records the host-side rebind; makes the entity findable.

**Files:**
- Create: `attacker/README.md`
- Modify: `docs/attacker-network-simulation.md` (append a short pointer)

**Interfaces:**
- Consumes: Tasks 1–5 (image, compose, three phase scripts).
- Produces: the end-to-end operator runbook; a pointer from the parent doc.

- [ ] **Step 1: Write `attacker/README.md`**

Content must include, verbatim and in order:

````markdown
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
````

- [ ] **Step 2: Append pointer to `docs/attacker-network-simulation.md`**

Add at the end of the doc:

```markdown
---

## Implemented: Tier 2 attacker container

Tier 2 (§2) is implemented as a separate container under `attacker/`. It
discovers and reaches the **natively run** master over docker0 and injects via
the existing `attack_*.py` unchanged. Runbook and host-side rebind steps:
`attacker/README.md`. Design spec:
`docs/superpowers/specs/2026-07-30-tier2-attacker-container-design.md`.
```

- [ ] **Step 3: Verify by inspection**

Confirm: README covers all four phases with copy-paste commands and expected
output; Phase 0 derives `docker0` IP at runtime (no hardcoded IP); the four
`attack.sh` examples use only real flags (`--duration`, `--yaw`, `--csv`); the
parent-doc note points to both `attacker/README.md` and the spec.

- [ ] **Step 4: End-to-end runbook check (user runs)**

Follow `attacker/README.md` top to bottom with the sim up: Phase 0 → build →
Phase 1 OK → Phase 2 lists topics → Phase 3 `cmd_vel` visibly disrupts the
robot. Each phase's checkpoint passes before the next.

- [ ] **Step 5: Commit**

```bash
git add attacker/README.md docs/attacker-network-simulation.md
git commit -m "docs(attacker): four-phase runbook + parent-doc pointer"
```

---

## Self-Review

**Spec coverage:**
- Spec §1 decisions → Global Constraints + Tasks 1–6. ✓
- §2 architecture (docker0, ROS_IP both sides, no Gazebo) → Task 1 (ROS_IP derive), Task 2 (docker0/master), Constraints. ✓
- §3 files (Dockerfile, compose, entrypoint, scan/enum/attack, README, parent note) → Tasks 1–6, one file each or grouped by responsibility. ✓
- §4 four phases → Phase 0 (Task 6 README), Phase 1 (Task 3), Phase 2 (Task 4), Phase 3 (Task 5). ✓
- §5 error handling (scan 0-hosts, enum ROS_IP gotcha, attack name/file) → Tasks 3/4/5 respectively. ✓
- §6 verification by inspection + runbook → every task's Steps 2–4. ✓
- §7 out of scope → Global Constraints (no Tier 3 / trigger / Gazebo model). ✓

**Placeholder scan:** No TBD/TODO; all scripts are complete; all commands concrete. Illustrative IPs are labeled illustrative and derived at runtime. ✓

**Type/name consistency:** Service name `attacker`, image `husky-attacker`, mount `/repo`, env `ROBOT_HOST_IP`, and attack names `cmd_vel|compass|odom|param` are used identically across Tasks 2–6. Attack-script flags match the verified in-repo CLIs. ✓
