# Offline-Map Navigation Demo

One terminal per step. Each block sets its own ROS env, so copy-paste as-is.

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
PYTHONPATH=~/Documents/Husky_viz/.worktrees/constellation-matcher:$PYTHONPATH python3 ~/Documents/Husky_viz/.worktrees/constellation-matcher/landmark_loc/localizer_node.py _places_path:=/home/thinh/Documents/Husky_viz/.worktrees/constellation-matcher/maps/park_places.yaml
```

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

Watch the robot in a browser: **http://localhost:6080/vnc.html**

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

```bash
pkill -9 -f 'bin/roslaunch' || true; sleep 1
for p in gzserver gzclient gazebo rosmaster move_base ekf_localization navsat \
         robot_state_publisher twist_mux controller_manager map_server rviz; do
  pkill -9 -f "$p" || true
done
pgrep -f 'gzserver|gazebo|move_base|ekf_localization|map_server' | xargs -r kill -9 2>/dev/null || true
cd ~/Documents/Husky_viz/operator && docker compose down 2>/dev/null || true
docker rm -f $(docker ps -aq --filter name=attacker) 2>/dev/null || true
docker network rm husky_lan 2>/dev/null || true
```
