# Simulating an Outside Attacker on the Husky ROS Network

This document captures the design discussion for simulating an *external network
attacker* against the native ROS 1 (Noetic) Husky simulation — how such an
attack works, how faithfully to reproduce the attacker's access, and how to wire
the **Gazebo physics layer** to the **network attack layer** so an attack fires
based on the robot's physical state.

It is a design/architecture record, not a runbook. No code has been written for
the trigger/geofence pieces yet — the final open decisions are listed at the end.

---

## 1. Why no special infrastructure is needed to model the attacker

The current setup is native ROS 1 Noetic with `ROS_MASTER_URI=http://localhost:11311`.

**ROS 1 has no authentication and no network boundary.** The master is an
unauthenticated XML-RPC service; any process that can open a TCP connection to
port `11311` becomes a full peer on the graph. Once connected it can:

- publish to *any* topic (as `attack_cmd_vel.py`, `attack_odom.py`, `attack_compass.py` do),
- overwrite *any* parameter (`attack_param.py`),
- read every topic and unregister other nodes.

So "an outside attacker sending an attack" and "a Python script publishing to a
topic" are **mechanically the same thing** — a rogue publisher on the ROS graph.
The existing `attack_*.py` scripts already *are* the attacker at the protocol
level. The only question is how faithfully to reproduce *how the attacker got
onto the network*.

---

## 2. Fidelity tiers for the attacker's access

| Tier | What it is | What it demonstrates | Extra infra |
|---|---|---|---|
| **Tier 1** *(dropped)* | Attack script as a separate process on the **same host** as the sim | The authentication/authorization flaw only. **Assumes the attacker already has network access** — skips the reachability/discovery step. | None |
| **Tier 2** | Attacker on a **separate host** (VM/container) on the same virtual network; must discover + reach the robot | The realistic remote-reachability chain: get on the network → scan → connect → attack | A second host (VM or container) |
| **Tier 3** | **Network-layer** attacks (sniff / replay / MITM) against the raw TCPROS traffic between nodes | A *different vulnerability*: ROS traffic is unencrypted/unauthenticated **on the wire** | A virtual network with nodes on *different* hosts |

**Tier 1 is dropped** for this effort because it assumes the attacker already has
access. We are committing to the realistic model where the attacker is a
**separate host that must discover and reach the robot.**

### Tier 1 vs. Tier 3 — the key conceptual split

- **Tier 1/2** abuse the fact that ROS **trusts any peer**. The attacker plays by
  ROS's rules: it registers with the master as a legitimate node and publishes.
- **Tier 3** abuses the fact that ROS **traffic is unencrypted and unauthenticated
  on the wire**. Once two nodes find each other via the master, they talk
  *directly* over a plain, unencrypted TCP stream (TCPROS); the master is not in
  the data path. An attacker positioned *on the path between two nodes* can
  **sniff** (read plaintext GPS/compass/cmd_vel), **replay** (re-inject captured
  packets), or **MITM/inject** (ARP-spoof between controller and publisher, then
  alter Twist values in flight). These use standard network tools
  (`tcpdump`, `ettercap`/`arpspoof`, `scapy`) — *not* ROS scripts — and require
  the attacker to sit between two machines, which only exists when nodes run on
  **different hosts**.

---

## 3. How an attacker reaches the ROS master in the real world

`ROS_MASTER_URI=http://<host>:11311` is **just an IP and a port — no secret,
no key, no token.** So "accessing the URI" reduces to two things:

1. **Network reachability** — can the attacker send packets to that host:port?
2. **Knowing the address** — trivial: `11311` is the well-known ROS default; a
   scan finds it in seconds.

Because part 2 is nearly free, the entire real-world problem is **part 1: getting
onto a network from which the robot is reachable.** There is **no discovery
broadcast in ROS 1** — the master does not advertise itself. A node connects only
because it was *configured* with the address (`ROS_MASTER_URI` env var). This is
"security by nobody-knowing-the-address" (obscurity), with no second line of
defense once the port is found.

How attackers actually get network reachability, roughly by frequency:

1. **Same LAN / WiFi** — robots are rarely air-gapped; they sit on lab/warehouse/
   campus WiFi. Anyone who joins (cracked WiFi, un-isolated guest net, compromised
   laptop, insider) can `nmap -p 11311 <subnet>` and find the robot.
2. **Internet-exposed master** — real and documented: internet scans (Shodan has a
   ROS fingerprint) have found thousands of live, publicly reachable ROS masters
   on open `11311`, caused by public IPs, "temporary" port-forwards, or
   misconfigured cloud security groups. No prior access needed — the open port is
   the front door.
3. **Pivot from another compromised device** — phished operator laptop, a
   compromised co-located IoT device, or an exploited web dashboard (this project
   contains `HuskyA300-Dashboard-main`; a compromised dashboard host sits directly
   on the ROS network).
4. **Supply-chain / physical** — malicious ROS package, tampered USB, or physical
   access to an Ethernet port.

Once on the network, discovery is free: `ROS_MASTER_URI` is in plaintext in env
vars / `~/.bashrc` / launch files, and the master itself answers `getSystemState`
to *any* caller, returning the entire node/topic graph including every node's IP
and port. ROS is designed to tell any caller everything about itself.

**Real-world chain:** get on the network (WiFi crack / exposed port / pivot) →
`nmap` for `11311` → connect and read the whole graph for free →
publish/inject. The existing attack scripts reproduce the *last* step.

---

## 4. How a deployed robot is legitimately commanded (the defensive context)

There is no single standard, but one near-universal fact: **raw ROS almost never
goes over the internet.** ROS 1's topic traffic is designed for a trusted LAN;
remote command uses a *secured wrapper* and keeps ROS local on the robot.

Patterns, local → remote:

1. **Local teleop, no internet** — RC radio link, or a laptop on the same WiFi
   running teleop (this mirrors the sim's gamepad / `teleop_twist_keyboard` model).
2. **Autonomous with local supervision** — autonomy runs *onboard* (like the
   `move_base` / GPS-waypoint drivers); the human sends high-level goals, not
   wheel commands, so the link tolerates slow/intermittent connectivity.
3. **Remote over the internet, wrapped** —
   - **VPN** (WireGuard/OpenVPN): robot + console join an encrypted private net;
     ROS flows as if on a LAN but the port is not internet-exposed. Most common
     "proper" way to remotely reach a ROS 1 robot.
   - **Cloud relay / fleet platform** (AWS IoT, Formant, Freedom Robotics, InOrbit,
     custom MQTT/gRPC): the robot **dials out** over TLS — no inbound port to scan.
   - **Web dashboard over HTTPS + WebSockets** (e.g. `rosbridge` behind TLS + login).
     `HuskyA300-Dashboard-main` is exactly this shape; the whole robot's security
     then hinges on the dashboard's, making it a prime pivot target.

**Security point:** every *safe* remote pattern adds the authentication +
encryption layer ROS 1 lacks, and keeps the naked master off the public internet.
The common real-world breach is that wrapper being **absent, weak, or bypassed**
("port-forward 11311 just for testing", VPN not required on the LAN, default
dashboard login) — which drops an attacker straight onto the trust-anyone ROS
layer. That is why Shodan-visible ROS masters exist.

---

## 5. The Gazebo / network-layer boundary — a critical clarification

**Gazebo simulates the *physical world* (pose, terrain, sensors, collisions).
A network attacker lives on the *ROS/TCP network*, which has no physical location.**

Consequences:

- **Do NOT add an "attacker entity/model" to the Gazebo scene.** A Gazebo model is
  just a physical object; it cannot publish to `cmd_vel` or set a parameter by
  being *near* the robot. Network attacks do not travel through simulated physical
  space. An attacker mesh would be theater, not a real demonstration.
- The attacker is **a process on the ROS graph**, period. Where you *run* that
  process (same host vs. separate host) is the only real variable, and it maps to
  the tiers in §2.

**Does the attacker need a separate laptop?** No. A **second VM or Docker
container on the same physical machine** gives a genuine second host with its own
IP — a real network peer that must discover and reach the robot. A physical
second laptop adds only *presentation* value (visceral "this other computer takes
over the robot"), not technical fidelity: the vulnerability, discovery, and
takeover are identical. For a network attacker, a **container is the practical
choice**.

### Is a VM/container "the same as this machine"?

For attack purposes, the attacker only needs to speak the same **ROS 1 wire
protocol**. Use **Ubuntu 20.04 + ROS Noetic** on both sides (Noetic is the one
official ROS 1 distro for 20.04) → the existing `rospy` attack scripts run
unchanged.

| | Docker container | VM |
|---|---|---|
| OS | Shares host kernel; Ubuntu 20.04 userspace from the ROS image | Full separate Ubuntu 20.04, own kernel |
| Weight | Light (hundreds of MB, seconds to start) | Heavy (GBs, full boot) |
| Network | Own IP on a Docker bridge → real second host | Own IP on bridged adapter → like a physical box |
| Setup | `docker pull ros:noetic-ros-core`, drop scripts in | Install Ubuntu, then ROS |

Both are equally valid for a *network* attacker because each gets its **own IP**.
Official prebuilt images (`ros:noetic-ros-core` has `rospy` + tooling; the sim's
Docker image already used `ros:noetic-robot` as base) mean no manual ROS install.
The attacker container does **not** need Gazebo — only `rospy` to talk to the
master.

### Env-var subtlety for any remote attacker (§2 Tier 2)

The master is currently bound to `localhost`. To let a remote attacker connect,
rebind it to a reachable IP on **both** sides:

- **Robot host:** `ROS_MASTER_URI=http://<robot-ip>:11311` and `ROS_IP=<robot-ip>`.
  `ROS_IP` matters because ROS hands out *callback addresses* to peers; if it
  advertises `127.0.0.1`, a remote node connects to the master but then cannot
  complete the topic handshake. (This bites everyone once.)
- **Attacker host:** `ROS_MASTER_URI=http://<robot-ip>:11311` and `ROS_IP=<attacker-ip>`.

Once set, `nmap` discovery + the existing scripts work with **zero code changes** —
the scripts do not care that the master is now remote.

---

## 6. Wiring the Gazebo physical layer to the network attack

Goal: fire the attack **only when the robot reaches a location in the attacker's
range, or physically touches something that activates the attack** — a spatial
attack *envelope*, not an always-on attack.

The bridge is a **separate "trigger" node** that watches the robot's physical
state on one side and enables/launches the network attack on the other. This is
in-character: a real attacker with a staging capability watches for a condition,
then strikes.

```
GAZEBO (physics)                    NETWORK / ROS layer
  robot pose ──► /navsat/fix ─────► [TRIGGER NODE]
  (or) contact sensor ─► /bumper ─►   • geofence / contact check
                                       • when in-range: enable ──► [ATTACK NODE]
                                                                     publishes spoofed
                                                                     cmd_vel / compass / odom
                                                                        │
                                                                        ▼
                                                                   robot obeys → moves in Gazebo
                                                                   (pose changes → feeds back to trigger)
```

The feedback loop is the point: the robot's motion changes its position, the
trigger re-evaluates, so the attack turns **on** entering the zone and **off**
leaving it — a real spatial envelope.

### 6a. Where the trigger gets "the robot's location" — two architectures

**Architecture A — trigger reads the robot's own sensors (recommended).**
The trigger subscribes to `/navsat/fix` (GPS), converts to world (x, y) using the
WGS84 constants already documented in `CLAUDE.md` (`REF_LAT 49.9`, `REF_LON 8.9`,
west-pointing +y sign), and fires when the robot enters a geofenced region.

- **Models a realistic attacker** who has compromised the graph and *eavesdrops on
  the robot's own telemetry* to time the strike. A real attacker cannot see Gazebo
  ground truth either — they watch the GPS/telemetry stream, exactly like this.
- **Respects the project's hard no-ground-truth rule** — in fact it is *more*
  faithful. This is the default choice.

**Architecture B — trigger reads Gazebo ground truth (`/gazebo/model_states`).**
The trigger uses the true pose to decide when the robot is "in range."

- `CLAUDE.md` forbids ground truth for robot pose in code *and* verification.
  Nuance: here ground truth would feed the *experimenter's trigger timing*, not the
  robot's navigation or the attack payload — a scenario-director's god's-eye view.
- **Defensible ONLY as a clearly-labeled scenario/experiment-control device**, and
  only if it never touches the robot's estimate or the attack payload — purely the
  on/off timing. Reach for it only if the trigger must be perfectly precise in
  world coordinates regardless of GPS noise (e.g. "fire at exactly x=10.0").
  Needs an explicit, narrow justification. **A is the default; B is the exception.**

### 6b. Two trigger *types*

1. **Geofence / proximity ("robot enters attacker's range").** Pure-software
   geofence in the trigger node — a distance/region check on the robot's position
   (from GPS, per Arch A). No Gazebo world changes. Models an "ambush zone" /
   attacker WiFi-or-jamming range. Easiest and most flexible.
2. **Physical contact ("robot touches something").** Reaches into the Gazebo
   world: add an object with a **Gazebo contact/bumper sensor** (or a trip region)
   that publishes a ROS message on collision; the trigger subscribes to that
   contact topic and fires on contact. Models a physical trap / tripwire / a
   malicious device the robot bumps. Requires adding a model + sensor plugin to the
   world file.

Both feed the **same** trigger node — the only difference is what it subscribes to
(a GPS-derived geofence vs. a contact-sensor topic).

### 6c. How the trigger *fires* the attack — two mechanisms

- **Gate an always-running attacker (preferred).** The attack node runs
  continuously but only publishes when an `enabled` flag is true; the trigger flips
  it via a ROS service/topic/param. Clean, fast, no process spawning. The existing
  scripts already re-read state each tick, so adding an `enabled` gate is a small,
  in-character change.
- **Launch on demand.** The trigger `subprocess`-launches an existing `attack_*.py`
  when the condition hits. Zero change to the scripts, but slower to start and
  messier to stop.

---

## 7. Proposed build (not yet implemented)

A single new **`attack_trigger.py`** node that:

1. Subscribes to `/navsat/fix`, converts to world (x, y) with the documented WGS84
   constants (`REF_LAT 49.9`, `REF_LON 8.9`, west-pointing +y).
2. Takes a geofence region as CLI args (center x, y + radius, or a box).
3. Enables the attack while the robot is inside the region; disables it outside.
4. Optionally, a `--contact-topic` mode that instead triggers on a Gazebo
   bumper-sensor message (the "touch something" flavor).

**Recommended starting point: Architecture A + geofence** — cleanest, fully
respects the no-ground-truth constraint, needs no world-file changes, and produces
the location-triggered attack envelope described above. Add the contact/tripwire
flavor as a second mode afterward.

---

## 8. Open decisions (to resolve before implementation)

1. **Trigger source:** Architecture **A** (robot's own GPS telemetry — realistic,
   respects no-ground-truth, *recommended*) or **B** (Gazebo ground truth as an
   experimenter's scenario director — needs the ground-truth exception)?
2. **First trigger type:** **geofence/proximity** (software only) or **physical
   contact** (add a bumper-sensor object to the Gazebo world)?
3. **Which attack the trigger fires:** compass-spoof, `cmd_vel` spin, odom spoof,
   param manipulation, or selectable per-run?
4. **(If pursuing Tier 2/3)** attacker host as a **Docker container** (recommended)
   or a **VM**; and whether to include the `nmap` discovery step in the demo.

---

## References in this repo

- Existing application-layer attack scripts: `attack_cmd_vel.py`, `attack_compass.py`,
  `attack_odom.py`, `attack_imu_derail.py`, `attack_imu_faithful.py`, `attack_param.py`.
- Victim/driver: `drive_to_point_gps.py`, `husky_auto_drive.py`, `send_mapless_goal.py`.
- Web control surface (pivot target): `HuskyA300-Dashboard-main/`.
- No-ground-truth rule, sensor table, and GPS→world constants: `CLAUDE.md`.

---

## Implemented: Tier 2 attacker container

Tier 2 (§2) is implemented as a separate container under `attacker/`. It
discovers and reaches the **natively run** master over docker0 and injects via
the existing `attack_*.py` unchanged. Runbook and host-side rebind steps:
`attacker/README.md`. Design spec:
`docs/superpowers/specs/2026-07-30-tier2-attacker-container-design.md`.

---

## Implemented: operator container (Tier 3 groundwork)

The §10 **operator** (setup 1 — a remote ROS command node) is implemented under
`operator/`: its own IP on docker0, sends one move_base goal to the natively-run
stock Husky, watches telemetry, and exports a normal-baseline CSV for the attack
comparisons. Robot spawn is decoupled into the host-side `spawn-robot-idle.sh`
(the world stays robot-less; the robot is spawned separately, then idles ready
for the operator). Runbook: `operator/README.md`. Design:
`docs/superpowers/specs/2026-08-02-operator-container-design.md`. It establishes
the operator↔robot wire that a future Tier 3 attacker container (C) will sit on;
the attacker-in-the-middle itself remains deferred.

---

## 9. Operator setup — what the robot↔operator link actually is

Expands §4 with the analysis needed to model a **realistic** operator boundary
(the only genuine cross-network wire on a field robot — see §10). There are
three real operator setups, and *which one* you model decides whether the
attacked wire even carries ROS.

| # | Operator setup | Wire protocol | ROS/network attacker can hit it? |
|---|---|---|---|
| **1** | **Another ROS machine** (laptop with `ROS_MASTER_URI` → robot; runs `teleop_twist_keyboard`/RViz/rqt) | raw **plaintext TCPROS** | **Yes, directly.** Both Tier 2 (it's a peer on the trust-anyone graph) AND Tier 3 (its link is plaintext ROS on the wire). Most faithful Tier 3 target. |
| **2** | **Web app / dashboard** (browser, no ROS; robot runs `rosbridge`/custom server — e.g. `HuskyA300-Dashboard-main`) | JSON/WebSocket, often TLS | Partially — attack the WS stream or the app's auth, not raw ROS. But a compromised dashboard host pivots **straight onto the onboard trust-anyone graph** → all Tier 2 attacks apply. |
| **3** | **Dedicated remote control** (RC radio / gamepad) | proprietary RF / USB HID — **not TCP/IP** | **No.** That's an RF-security problem; a `joy_node` translates it to `cmd_vel` *onboard*. Out of scope for a network attacker. |

**Which is more realistic — ROS laptop vs. web app?** Depends on stage:

- **ROS laptop (setup 1)** dominates **development / research / early field trials** —
  common, but mostly among the engineers who *build* the robot.
- **Web app (setup 2)** dominates **production deployment.** The operator is a
  non-engineer (warehouse worker, farmer) who will never install ROS; fleets need
  one console for many robots; remote operation must not put raw ROS on the
  internet, so a dashboard over HTTPS/WSS is used instead. Clearpath ships a web
  dashboard for exactly this reason.

For a **deployed field Husky**, the web app is the more truthful operator picture.
But note the nuance: **even with a web app, raw plaintext ROS still runs onboard**
— the dashboard is only a rosbridge translation layer bolted on top. So the
dashboard is both the *realistic* operator surface **and** the biggest risk: the
whole robot's security collapses to the dashboard's, and breaching it (weak
password, exposed rosbridge, XSS) lands the attacker directly on the onboard
trust-anyone graph where every Tier 2 attack already works.

**Two setups, two different demos (they don't conflict):**

- **Web app as the realistic *entry point*** → models *how the attacker gets onto
  the ROS network*: breach the dashboard → pivot to onboard ROS → run the existing
  `attack_*.py` (Tier 2). This is a complete, realistic attack chain using what is
  **already built** — no Tier 3 needed — and matches the repo's
  `HuskyA300-Dashboard-main`.
- **ROS laptop as the realistic Tier 3 *target*** → models *plaintext ROS on the
  operator wire* being sniffed/replayed/MITM'd (§10).

## 10. Tier 3 — realistic split: robot↔operator, NOT sensor↔controller

A Tier 3 (on-the-wire) demo needs two victim nodes on **different hosts** so
there is a real wire to intercept. The tempting split — *put a sensor node in its
own container* — is **contrived and must not be used**: on a real Husky the
sensors (GPS, IMU, compass, encoders) are mounted on the chassis and wired to the
robot's **own CPU**, so their driver nodes run onboard and talk to the controller
over **loopback**. There is no network wire between sensor and controller;
manufacturing one would demonstrate a vulnerability that does not exist (the same
"theater" failure §5 warns about for a Gazebo attacker model).

The only genuine cross-network wire on a field robot is **robot ↔ operator**
(and, less commonly, robot ↔ a second onboard compute box over an internal
Ethernet switch; or robot ↔ cloud backend, usually TLS-wrapped). So the faithful
Tier 3 split keeps **everything onboard together** and separates the operator:

```
Container A (the ROBOT):     roscore + gazebo + controller + ALL sensor drivers
                             — sensors talk to the controller over loopback
                               INSIDE this container, exactly as on real hardware
Container B (the OPERATOR):  a ROS teleop/command node + a telemetry subscriber
                             (operator setup 1), on a separate IP
        ↕  A↔B command+telemetry traffic crosses the bridge — the REAL wire
Container C (the ATTACKER):  sits between A and B (ARP-spoof), with
                             tcpdump / ettercap / scapy (a NEW image — the Tier 2
                             image deliberately has none of these)
```

Tier 3 attack types on that wire: **sniff** (read plaintext GPS/`cmd_vel`),
**replay** (re-inject captured packets), **MITM/inject** (rewrite Twist/heading in
flight). These are **new scripts**, not the Tier 2 `attack_*.py`. Effort is high
(the two-host split + containerized Gazebo + kernel-level ARP-spoof caveats +
NET_ADMIN/NET_RAW); deferred to its own design cycle. Sniffing is the achievable,
dramatic first step; full in-flight MITM of framed TCPROS is a real project.
