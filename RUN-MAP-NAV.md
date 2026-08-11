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

## Step 2 — Navigation + map  (choose ONE localization mode)

### Option A — GPS mode (spoofable; used by the attacker demo in Step 6)

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
roslaunch launch/move_base_gps_map.launch
```

### Option B — Landmark mode (GPS-free; recognizes park landmarks from lidar)

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
cd ~/Documents/Husky_viz
roslaunch launch/move_base_landmark.launch
```

In landmark mode the GPS-spoof of Step 6 has nothing to attack (no navsat in the
loop) — the robot keeps localizing off the furniture it can see.

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
```

Landmarks: `bench`, `garden_table`, `lamp`, `trash_bin_1`.

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
