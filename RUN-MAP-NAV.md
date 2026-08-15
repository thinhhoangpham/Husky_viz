# Offline-Map Navigation Demo

One terminal per step. Each block sets its own ROS env, so copy-paste as-is.

> **Park is the default.** To run the same demo in the **lake** world, change
> three things — everything else is identical:
>
> | Step | Park (default) | Lake |
> |---|---|---|
> | 1 | `./load-park-world.sh` | `./load-park-world.sh --world lake` |
> | 2 | `roslaunch launch/move_base_gps_map.launch` | append `map:=$HOME/Documents/Husky_viz/maps/lake_map.yaml` |
> | 3 | `_objects_path:=…/maps/park_objects.yaml` | `…/maps/lake_objects.yaml` |
>
> Lake is self-contained in `models_lake_opt/` (low-poly visuals *and* the original
> collision meshes) — no external drive needed to run it.
> Its landmarks are `tree`, `postescable`, `lago` — and note **the lake itself is a
> landmark, not an obstacle**: `lago` has no `<collision>`, so the lidar cannot see
> the water and it is deliberately absent from `lake_map.pgm`. Low vegetation is
> skipped in the static map too; the robot dodges it from live lidar.
> Step 3's operator (`operate.py`) still reads `maps/park_objects.yaml` — goal-by-name
> is park-only until that path is parameterised.

## Step 0 — Docker network

```bash
docker network create --subnet 172.20.0.0/16 husky_lan 2>/dev/null || true
```

## Step 1 — World + robot

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
./load-park-world.sh
```

Wait for Gazebo to show the park + robot, then ~30–60 s for the pose to settle.

## Step 2 — Navigation + map

Start these **three** nodes, each in its own new terminal (Step 1 is Terminal 1). All three must be running.

**Terminal 2 — map_server + move_base:**

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
roslaunch launch/move_base_gps_map.launch
```

**Terminal 3 — landmark localizer** (fills `/odometry/landmark_fix` from the lidar):

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
PYTHONPATH=~/Documents/Husky_viz:$PYTHONPATH python3 ~/Documents/Husky_viz/landmark_loc/localizer_node.py _objects_path:=/home/thinh/Documents/Husky_viz/maps/park_objects.yaml _matcher:=typed
```

> `_matcher:=typed` (default) runs the identity-aware RANSAC constellation matcher. `_matcher:=typeless` runs the geometry-only control matcher (ignores landmark identity; matches on pairwise-distance shape alone) — used to compare identity-vs-geometry. To demo the other mode, relaunch Terminal 3 with `_matcher:=typeless`; the localizer logs `[localizer] matcher mode: <mode>` at startup so you can confirm which is active.

**Terminal 4 — pose-source selector** (fills `/odometry/abs_fix`; starts in `gps` mode):

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
source /opt/ros/noetic/setup.bash
python3 ~/Documents/Husky_viz/landmark_loc/abs_fix_selector.py
```

Both pose feeders (GPS + landmark) run at once, publishing to separate topics. The selector forwards exactly one to the EKF, and the operator switches between them live with `mode gps` / `mode landmark` (Step 4) — no relaunch, no choosing a launch file.

## Step 3 — Operator

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz/operator
docker compose up -d
docker compose exec operator bash -lc "source /opt/ros/noetic/setup.bash && ./operator/operate.py"
```

Watch the robot in a browser: **http://localhost:6080/vnc.html**. For the full window (not cut off), use **http://localhost:6080/vnc.html?resize=scale**.

> **Rebuild after editing `operator/entrypoint.sh`:** the operator container bakes `entrypoint.sh` into its image. After changing it (resolution, RViz maximize, etc.), a plain `docker compose up` reuses the old image and your change will NOT apply — you must rebuild: `cd operator && docker compose build && docker compose up -d`.

## Step 4 — Send a goal (at the `operator>` prompt)

```
goal 49.9000094 8.9000327      # GPS lat/lon
goal xy 9.17 13.55             # map coordinates
goal garden_table              # landmark name
mode gps                       # switch absolute source to GPS (navsat)
mode landmark                  # switch absolute source to landmark localizer
```

Landmarks: `bench`, `garden_table`, `lamp`, `trash_bin_1`.

Switching is live — no relaunch needed. `status` shows `abs_fix=<mode>` (with
`:stale` appended if the selected source has gone silent).

## Step 5 — (optional) Drop an obstacle in its path

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
python3 - <<'PY'
import rospy
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion
rospy.init_node("spawn_box", anonymous=True)
sdf = """<?xml version="1.0"?><sdf version="1.6"><model name="surprise_box">
<static>true</static><link name="link">
<collision name="c"><geometry><box><size>1 1 1.5</size></box></geometry></collision>
<visual name="v"><geometry><box><size>1 1 1.5</size></box></geometry>
<material><ambient>1 0.3 0 1</ambient><diffuse>1 0.3 0 1</diffuse></material></visual>
</link></model></sdf>"""
rospy.wait_for_service("/gazebo/spawn_sdf_model", timeout=10)
sp = rospy.ServiceProxy("/gazebo/spawn_sdf_model", SpawnModel)
sp("surprise_box", sdf, "", Pose(position=Point(20.0, 0.0, 3.65),
                                 orientation=Quaternion(0,0,0,1)), "world")
PY
```

Edit `Point(20.0, 0.0, 3.65)` to a spot ahead of the robot (keep `z=3.65` — the
ground is at z≈2.9). Remove it:

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
rosservice call /gazebo/delete_model '{model_name: surprise_box}'
```

## Step 6 — (optional) Attacker: GPS spoof

While the robot is driving to a goal, an attacker on the network slowly fakes
its GPS. `navsat_transform` accepts the drifting fixes, so the map-EKF's fused
position is dragged off course — the robot chases a phantom and thrashes, while
the operator's display still looks nominal (it reads the corrupted pose).

Send a goal (Step 4) so the robot is en route, then run the attacker:

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz/attacker
docker compose run --rm attacker ./attacker/attack.sh navsat --drift-rate 0.5 --max-offset 15 --duration 40
```

Watch in RViz: the robot lurches/spins off its route as the fused pose drifts
(~15 m over 40 s). When the attack stops, genuine GPS reels the estimate back.

Stronger variant: `--drift-rate 1.5 --max-offset 40 --duration 40`.

The spoof only affects the fused pose while `abs_fix` is in `gps` mode; running
`mode landmark` at the `operator>` prompt removes navsat from the loop live, letting
the operator switch away from a spoofed source mid-attack.

## Step 7 — Full demo: GPS → spoof → switch to landmarks → recover

The headline flow: the robot navigates on GPS, an attacker drifts the GPS so the
robot is dragged off course, then the operator switches the absolute source to
landmarks live — GPS leaves the loop, and the robot reaches its goal on lidar
localization instead.

Assumes Steps 0–3 are up (world+robot, move_base, localizer, selector, operator).
The selector starts in `gps` mode by default.

1. **Confirm GPS mode and send the goal.** At the `operator>` prompt:

   ```
   status                         # expect: abs_fix=gps
   goal 49.9000094 8.9000327      # GPS lat/lon goal (known-good, free space)
   ```

   The robot should begin driving toward the goal on the GPS-anchored pose.

2. **Launch the GPS spoof** (separate terminal) while the robot is en route:

   ```bash
   export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
   cd ~/Documents/Husky_viz/attacker
   docker compose run --rm attacker ./attacker/attack.sh navsat --drift-rate 0.5 --max-offset 15 --duration 40
   ```

   Watch in RViz (**http://localhost:6080/vnc.html**): the fused pose drifts and
   the robot lurches off its route. Because the selector is in `gps` mode, the
   spoofed navsat fix is what `abs_fix` carries.

3. **Switch to landmarks live** — at the `operator>` prompt, mid-attack:

   ```
   mode landmark                  # abs_fix now = landmark localizer, navsat out of the loop
   status                         # expect: abs_fix=landmark
   ```

   The GPS spoof no longer reaches the EKF: `abs_fix` is now filled by the lidar
   landmark localizer, which the attacker cannot touch. The fused pose re-anchors
   to what the robot actually sees.

4. **Confirm recovery.** Re-send the same goal so the robot re-plans from the
   now-truthful pose (the drift during the attack may have left it mid-route):

   ```
   goal 49.9000094 8.9000327
   status                         # watch dist decrease; abs_fix should stay 'landmark'
   ```

   The robot should drive to the goal on landmark localization and arrive, with
   the attacker still running — demonstrating the switch defeats the spoof.

## Stop everything

**Read this before running it.** Two things about this block are deliberate and may
surprise you:

- **It kills EVERY ROS/Gazebo process on this machine**, not just the ones your
  current shell started — including orphaned nodes left behind by earlier sim runs.
  That is the point: ghost nodes accumulating on a shared master is the failure this
  block exists to prevent. If you are running an unrelated ROS process on this box,
  it will be killed too.
- **It exits non-zero and prints the survivors** if anything is still alive at the
  end. Silence + exit 0 is the only "everything is down" signal; never assume success
  without checking.

It does *not* use `pkill -f` for the kill — on this box `pkill -f` matches nothing
even when the pattern is literally the start of the cmdline. It enumerates PIDs from
`/proc/*/cmdline` and kills each by explicit PID (SIGTERM, then SIGKILL), which does
work.

Matching happens in **two tiers**, and the order matters:

1. **Unconditional.** Anything running out of the ROS install prefix
   `/opt/ros/noetic/`, or any `gzserver`/`gzclient`/`gzweb`/`rosmaster`/`roscore`, is
   killed with **no exclusions applied at all**. A node under the ROS prefix is a sim
   node by definition, so nothing may exempt it. (An earlier version checked
   exclusions first and let real nodes escape on a bare substring — a
   `velodyne_pointcloud/.../--decoder=hdl` node was spared because "decoder" contains
   "code".)
2. **Exclusion-filtered.** Processes matched only by the *repo path* or by a bare
   mention of `roslaunch` are the ambiguous ones — that is where a VS Code window with
   a repo file open, or a shell whose command line quotes a roslaunch invocation,
   shows up. Editor/tooling exclusions apply to this tier only, and they anchor on the
   executable name (`*/code`, `*/code *`) rather than a bare substring, so they cannot
   fire on an arbitrary substring. The residual gap is narrow but real: a sim node
   launched from the repo *outside* `/opt/ros/noetic/` whose executable is literally
   named `code`, `vim`, `cursor`, etc. would be spared. Nothing in the current stack
   is, and such a process would still be reported as a survivor with a non-zero exit
   rather than silently ignored.

```bash
set -u
REPO="/home/thinh/Documents/Husky_viz"

# Enumerate sim PIDs from /proc. Two tiers -- see the note above.
scan_sim_pids() {
  local d pid cmd
  for d in /proc/[0-9]*/; do
    pid=${d#/proc/}; pid=${pid%/}
    case "$pid" in "$$"|"$PPID"|1) continue ;; esac
    cmd=$(tr '\0' ' ' < "${d}cmdline" 2>/dev/null) || continue
    [ -n "$cmd" ] || continue

    # Tier 1: ROS install prefix + Gazebo/master. ALWAYS killed, never exempt.
    case "$cmd" in
      */opt/ros/noetic/*|*/gzserver|*/gzserver\ *|gzserver\ *|\
      */gzclient|*/gzclient\ *|gzclient\ *|*/gzweb|*/gzweb\ *|\
      */rosmaster|*/rosmaster\ *|rosmaster\ *|*/roscore|*/roscore\ *|roscore\ *)
        echo "$pid"; continue ;;
    esac

    # Tier 2: repo path or a bare 'roslaunch' mention -- ambiguous, so filter
    # out editors/IDEs and agent tooling by EXECUTABLE NAME (anchored on '/').
    case "$cmd" in
      *"$REPO"*|*/roslaunch|*/roslaunch\ *|roslaunch\ *|*\ roslaunch\ *)
        case "$cmd" in
          */code|*/code\ *|*/codium|*/codium\ *|*/cursor|*/cursor\ *|\
          */nvim|*/nvim\ *|*/vim|*/vim\ *|*/emacs|*/emacs\ *|\
          */sublime_text|*/sublime_text\ *|*/pycharm*|*/idea*|*/jetbrains*|\
          */claude|*/claude\ *|*claude-code*|*snapshot*) continue ;;
        esac
        echo "$pid" ;;
    esac
  done
}

# SIGTERM, wait, SIGKILL. Repeat: respawners and multi-generation residue can
# need several passes. Bounded at 4 so this can never spin forever.
for pass in 1 2 3 4; do
  pids=$(scan_sim_pids); [ -n "$pids" ] || break
  echo "pass $pass: SIGTERM -> $(echo $pids | tr '\n' ' ')"
  for p in $pids; do kill -15 "$p" 2>/dev/null; done
  sleep 3
  pids=$(scan_sim_pids); [ -n "$pids" ] || break
  echo "pass $pass: SIGKILL -> $(echo $pids | tr '\n' ' ')"
  for p in $pids; do kill -9 "$p" 2>/dev/null; done
  sleep 1
done

# Docker teardown: operator stack, attacker containers, the husky_lan network.
( cd "$REPO/operator" && docker compose down ) || echo "WARN: docker compose down failed"
att=$(docker ps -aq --filter name=attacker)
[ -n "$att" ] && { docker rm -f $att || echo "WARN: could not remove attacker containers"; }
docker network inspect husky_lan >/dev/null 2>&1 && \
  { docker network rm husky_lan || echo "WARN: could not remove husky_lan"; }

# Verify. Anything below that reports a problem means teardown did NOT succeed.
fail=0
survivors=$(scan_sim_pids)
if [ -n "$survivors" ]; then
  fail=1; echo "FAIL: sim processes still alive:"
  for p in $survivors; do echo "  $p  $(tr '\0' ' ' < /proc/$p/cmdline 2>/dev/null)"; done
fi
if ss -ltn 2>/dev/null | grep -q ':11311'; then
  fail=1; echo "FAIL: port 11311 still LISTENing (rosmaster is up)"
fi
left=$(docker ps -aq --filter name=attacker)
[ -n "$left" ] && { fail=1; echo "FAIL: attacker containers remain: $left"; }
docker network inspect husky_lan >/dev/null 2>&1 && \
  { fail=1; echo "FAIL: docker network husky_lan still exists"; }

if [ "$fail" -ne 0 ]; then
  echo "TEARDOWN INCOMPLETE — kill the survivors above by PID and re-run."
  exit 1
fi
echo "PASS: no ROS/Gazebo processes, 11311 free, containers and husky_lan gone."
```

If it exits 1, kill the listed PIDs directly (`kill -9 <pid>`) and run the block
again. If a PID keeps coming back, something is respawning it — find its parent with
`ps -o ppid= -p <pid>` before killing again.
