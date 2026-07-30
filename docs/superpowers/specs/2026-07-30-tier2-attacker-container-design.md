# Tier 2 Attacker as a Separate Entity — Design Spec

**Date:** 2026-07-30
**Status:** Approved design, not yet implemented
**Parent doc:** `docs/attacker-network-simulation.md`

Implements the **Tier 2** attacker from the parent design doc: a network attacker
as a **separate host** (a Docker container with its own IP) that must **discover
and reach** the natively-running Husky ROS master, then inject using the existing
`attack_*.py` scripts **unchanged**. Tier 3 (on-the-wire MITM) is explicitly out
of scope and deferred.

---

## 1. Decisions locked in

| Decision | Choice | Why |
|---|---|---|
| Fidelity tier | **Tier 2** (rogue peer on the ROS graph) | Most realistic *threat* for this robot: low barrier (reach port 11311), documented in the wild (Shodan-visible masters). Existing scripts already embody it. Tier 3 deferred. |
| Sim host | **Native `roslaunch` on this Linux box** | ROS Noetic is installed natively here (`/opt/ros/noetic`); the sim is not containerized on Linux. |
| Attacker host | **Docker container on the same box, own IP** | A genuine second network peer. Reaches the native master via the default `docker0` bridge → host IP. |
| Network model | **Default `docker0` bridge → host IP** | Because the master is native on the host, the attacker must reach the *host*. A custom bridge network would only help if the sim were also a container. The container still gets its own IP and nmap still sweeps a real subnet. |
| Discovery step | **Included, as an explicit labeled recon phase** | Shows the full realistic chain: get on network → scan → enumerate → inject. |
| Recon structure | **Two separate scripts**: `scan.sh` (nmap reachability) then `enum.sh` (ROS graph read) | Isolates *network* failure from *ROS* failure. Test nmap connectivity first. |
| Host rebind delivery | **Doc + copy-paste commands** | Leaves the existing `roslaunch` flow untouched; no wrapper script to maintain. |
| Execution split | **Files authored in-repo; user runs Docker** | Docker is not reachable from the authoring session. Verification is by file inspection + a per-phase runbook with expected output. |

---

## 2. Architecture

```
This Linux host  (docker0 gateway IP, e.g. 172.17.0.1)
├── NATIVE: roscore + roslaunch <husky sim>
│     master must bind reachably, NOT localhost:
│       ROS_MASTER_URI=http://172.17.0.1:11311
│       ROS_IP=172.17.0.1        ← the gotcha: without this the master
│                                   advertises 127.0.0.1 as the callback
│                                   address and the remote TCPROS handshake
│                                   fails (nmap succeeds, rostopic hangs)
└── DOCKER: attacker container  (own IP on docker0, e.g. 172.17.0.2)
      ROS_MASTER_URI=http://172.17.0.1:11311
      ROS_IP=<attacker container IP>   (derived at start from the container)
      contains: rospy + nmap + iproute2; attack_*.py bind-mounted, unchanged
```

The attacker is **a process on the ROS graph** — never a Gazebo model. Network
attacks do not travel through simulated physical space (parent doc §5). The
container does **not** need Gazebo, only `rospy` to talk to the master.

---

## 3. Files

All new files live in a self-contained `attacker/` directory. Nothing outside it
changes except one discoverability note.

```
attacker/
├── Dockerfile            # ros:noetic-ros-core + nmap + iproute2; no Gazebo
├── docker-compose.yml    # attacker container on docker0, env-var driven
├── entrypoint.sh         # derive + export ROS_IP from container IP, then shell/cmd
├── scan.sh               # PHASE 1 — nmap reachability sweep
├── enum.sh               # PHASE 2 — ROS graph read (rostopic/rosnode/rosparam)
├── attack.sh             # PHASE 3 — run a chosen attack_*.py against remote master
└── README.md             # host-side rebind steps + full per-phase runbook
```

- **`Dockerfile`** — base `ros:noetic-ros-core` (has `rospy`, no Gazebo needed),
  adds `nmap` and `iproute2`. The repo's `attack_*.py` are **bind-mounted** at
  runtime, not copied, so the repo stays the single source of truth and the
  scripts run unchanged.
- **`docker-compose.yml`** — one service on the default bridge (own container
  IP). Reads `ROBOT_HOST_IP` from the environment and sets
  `ROS_MASTER_URI=http://${ROBOT_HOST_IP}:11311`. Bind-mounts the repo root
  read-only so the scripts are visible inside the container. (Implementation
  note: the mount is actually read-write, because the Phase 3 attack scripts
  write their output CSVs into the mounted repo tree.)
- **`entrypoint.sh`** — derives the container's own IP, exports `ROS_IP` so the
  attacker's callback address is correct, then runs the passed command or drops
  to an interactive shell.
- **`scan.sh`** (Phase 1) — `nmap -p 11311 <docker0 subnet>`. Exits non-zero with
  a clear message if zero hosts answer (points back to Phase 0). Pure network
  reachability — no ROS calls. **Run and verify this first.**
- **`enum.sh`** (Phase 2) — `rostopic list` / `rosnode list` / `rosparam list`
  against the found master. Read-only. If these hang after nmap succeeded, the
  script calls out the `ROS_IP` callback-address problem specifically.
- **`attack.sh`** (Phase 3) — takes an attack name
  (`cmd_vel` | `compass` | `odom` | `param`) plus its args, validates the name
  and that the mounted `attack_<name>.py` exists, then execs it against the
  remote master. Zero changes to the attack scripts.
- **`README.md`** — the runbook (see §4), including the host-side rebind commands.

**One note outside `attacker/`:** a short pointer added to
`docs/attacker-network-simulation.md` recording that Tier 2 is implemented in
`attacker/` and how it is launched, so the setup is discoverable later.

Per the project's global instructions, the file authoring (Dockerfile, compose,
shell scripts) is code and will be routed to the `senior-fullstack-dev` agent,
not written in the main conversation.

---

## 4. Data flow — four ordered phases

Each phase is a checkpoint you verify before the next, mirroring the real kill
chain (recon → recon → exploit → observe). Splitting reachability from
enumeration means a failed phase names the broken layer exactly.

```
PHASE 0 — HOST PREP  (host, before the sim)
  export ROS_IP=<docker0 gateway IP>          # e.g. 172.17.0.1
  export ROS_MASTER_URI=http://$ROS_IP:11311
  roslaunch <husky sim>                       # master reachable off localhost
        │
        ▼
PHASE 1 — REACHABILITY  (container: scan.sh)
  nmap -p 11311 <docker0 subnet>
  CHECKPOINT: exactly one host answers 11311 (the robot).
    0 hosts → Phase 0 rebind or a firewall is wrong. STOP here.
    Nothing ROS-level runs until this passes.
        │
        ▼
PHASE 2 — ENUMERATION  (container: enum.sh)
  rostopic list / rosnode list / rosparam list against the found master
  CHECKPOINT: full topic/node/param graph prints ("ROS volunteers everything").
    nmap passed but this hangs → the ROS_IP callback-address gotcha. Read-only.
        │
        ▼
PHASE 3 — EXPLOITATION  (container: attack.sh <name> [args])
  execs the mounted attack_<name>.py (cmd_vel|compass|odom|param), unchanged
  OBSERVE: robot obeys the spoofed input in Gazebo (noVNC / operator view)
```

**Reconnaissance vs. attack:** Phases 1–2 are reconnaissance (a port scan +
graph read — discovery, not harm; the robot's function is untouched). Phase 3 is
the actual attack payload. The nmap scan is how the attacker *finds* the robot,
not how it *harms* it.

---

## 5. Error handling

- **`scan.sh`** — non-zero exit + clear message if 0 hosts answer on 11311;
  message points back to Phase 0 (host rebind / firewall).
- **`enum.sh`** — if `rostopic list` hangs or fails *after* nmap succeeded, that
  is specifically the `ROS_IP` callback-address problem; the script says so
  rather than failing silently.
- **`attack.sh`** — validates the attack name against the known set and confirms
  the mounted `attack_<name>.py` exists before running; clear error otherwise.

---

## 6. Testing / verification

Docker is not reachable from the authoring session, so verification is:

1. **File inspection** — every authored file is reviewed for correctness.
2. **Per-phase runbook** — the README gives the exact command and the expected
   output at each checkpoint, so each layer is confirmed independently on the
   host: Phase 1 proves container→host reachability; Phase 2 proves ROS-level
   access; Phase 3 proves the takeover.

No automated test suite is in scope — this is infrastructure + a demo runbook,
verified by the checkpoints above.

---

## 7. Out of scope (explicitly deferred)

- **Tier 3** (sniff / replay / MITM against raw TCPROS) — a different vulnerability
  (unencrypted wire) needing the victim nodes split across hosts and `NET_ADMIN`.
  Saved for later if needed.
- **The trigger / geofence layer** (parent doc §6–7) — firing the attack on a
  spatial condition. Deliberately deferred; this spec sets up the attacker entity
  only.
- **A physical "attacker" model in Gazebo** — rejected by the parent doc (§5); a
  network attacker has no physical location.

---

## 8. References

- Parent design: `docs/attacker-network-simulation.md` (tiers, reachability chain,
  Gazebo/network boundary).
- Existing attack scripts (run unchanged): `attack_cmd_vel.py`, `attack_compass.py`,
  `attack_odom.py`, `attack_param.py`.
- No-ground-truth rule, sensor table, GPS→world constants: `CLAUDE.md`.
