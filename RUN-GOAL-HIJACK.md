# Run the Goal-Hijack Demo — Step by Step

Open a separate terminal for each step. Do them in order.

> **Reset between runs (read first):** the world (Step 2) and the robot (Step 3)
> are each started **once** and left up. To return the robot to its start pose
> between attack runs, do **not** restart the world and do **not** kill/respawn
> any node. Just re-run `./spawn-robot-idle.sh` (it auto-resets if the robot is
> already up) or run `./reset-robot.py`. See "Reset the robot between runs" below.

---

### Step 0 — kill any old sim (only for a truly fresh start)

You only need this the very first time, or if the sim is in a broken state — a
normal reset between runs does **not** kill anything (see the note above).

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f gazebo
pkill -9 -f rosmaster; pkill -9 -f roscore
pkill -9 -f move_base; pkill -9 -f spawn_robot_idle
pkill -9 -f ekf_localization; pkill -9 -f robot_state_publisher
pkill -9 -f twist_mux; pkill -9 -f create_park; pkill -9 -f load-park
sleep 2
# Purge any leftover/ghost ROS nodes from killed attacker/operator runs
# (a ghost publisher on /move_base/goal preempts goals and makes markers vanish).
yes | rosnode cleanup 2>/dev/null || true
```

---

### Step 1 — create the network

```bash
docker network rm husky_lan 2>/dev/null
docker network create --subnet 172.20.0.0/16 husky_lan
```

> **Why `--subnet` is required:** the rest of this guide hardcodes the gateway IP `172.20.0.1` (as `ROS_IP` / `ROS_MASTER_URI` / `ROBOT_HOST_IP`). Without pinning the subnet, Docker may assign `husky_lan` a different range on recreation (e.g. `172.21.0.0/16`, gateway `172.21.0.1`); the hardcoded `172.20.0.1` then no longer exists on any interface and `roslaunch` fails with `Unable to contact my own server at [http://172.20.0.1:...]`. Pinning `172.20.0.0/16` guarantees the gateway is always `172.20.0.1`.

---

### Step 2 — start the world (Terminal 1)

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROBOT_HOST_IP=172.20.0.1
./load-park-stock-husky.sh
```

Wait until Gazebo shows the park.

---

### Step 3 — spawn the robot (Terminal 2)

```bash
cd ~/Documents/Husky_viz
export ROS_IP=172.20.0.1
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROBOT_HOST_IP=172.20.0.1
./spawn-robot-idle.sh
```

Wait for: `IDLE — waiting for a remote operator goal.`

> **Spawn-or-reset:** `spawn-robot-idle.sh` spawns the robot only if none is up.
> If a robot is already up, it instead **resets** it (teleport to init pose, zero
> velocity, re-sync the EKF) without killing any node — see below.
>
> **GPU lidar:** the robot mounts an Ouster OS1-64 that publishes
> `/os0_cloud_node/points` at 10 Hz. It uses the **GPU** ray sensor (`gpu_ray`);
> the CPU ray sensor's ~17,800 raycasts/scan starved Gazebo's physics loop and
> made motion jerky, so `gpu_ray` is required for smooth motion. Already wired via
> `HUSKY_URDF_EXTRAS` in `spawn-robot-idle.sh`.

---

### Step 4 — start the attacker (Terminal 3)

```bash
cd ~/Documents/Husky_viz/attacker
export ROS_IP=172.20.0.1
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROBOT_HOST_IP=172.20.0.1
docker compose run --rm attacker ./attacker/attack.sh goal --watch
```

The attacker LISTENS. Nothing is set in advance.
Wait for: `READY — now waiting for the operator's goal.`

---

### Step 5 — operator sends the real goal (Terminal 4)

```bash
cd ~/Documents/Husky_viz/operator
export ROS_IP=172.20.0.1
export ROS_MASTER_URI=http://172.20.0.1:11311
export ROBOT_HOST_IP=172.20.0.1
docker compose run --rm operator ./operator/operate.py --goal-x 15 --goal-y 0
```

- Green circle at (10, 0) = real goal.
- The attacker (Terminal 3) prints the real goal it heard:
  `OVERHEARD operator goal: (10.00, 0.00)`, then exits.

Now you have SEEN the real goal. Go to Step 6 to send the false goal.

---

### Step 6 — attacker sends the false goal (Terminal 3)

```bash
docker compose run --rm attacker ./attacker/attack.sh goal --abs-x 10 --abs-y 12
```

- Red circle at (10, 12) = false goal.
- Robot drives to the RED circle, not the green.

---

### Reset the robot between runs

To run the demo again, return the robot to its start pose — do **not** restart the
world and do **not** kill/respawn nodes. From Terminal 2 (or any terminal with the
same `ROS_*` env vars):

```bash
cd ~/Documents/Husky_viz
./spawn-robot-idle.sh   # auto-resets because the robot is already up
# — or, equivalently —
./reset-robot.py
```

Either one teleports the robot to the init pose, zeroes its velocity, and re-syncs
the EKF. It is instant and returns to the prompt. The world (Step 2) stays up the
whole time, so the ROS master stays clean (no ghost node registrations). Then
re-run the attacker (Step 4) and operator (Step 5) for the next run.

---

### Stop everything

```bash
pkill -9 -f gzserver; pkill -9 -f gzclient; pkill -9 -f gazebo
pkill -9 -f rosmaster; pkill -9 -f roscore
pkill -9 -f move_base; pkill -9 -f spawn_robot_idle
pkill -9 -f ekf_localization; pkill -9 -f robot_state_publisher
pkill -9 -f twist_mux; pkill -9 -f create_park; pkill -9 -f load-park
docker network rm husky_lan 2>/dev/null
```
