# Run the Offline-Map Navigation Demo

The robot plans over a **preloaded map** of the park (extracted from `park.world`)
and drives to goals, avoiding both mapped obstacles and live lidar obstacles.

Prerequisite: the dataset overlay at `~/husky_overlay_ws` is built (provides the
dual-EKF + GPS + Ouster lidar robot). One terminal per step.

**Every terminal needs these exports:**

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311 ROBOT_HOST_IP=172.20.0.1
```

---

## Step 0 — Docker network (once per boot)

```bash
docker network create --subnet 172.20.0.0/16 husky_lan 2>/dev/null || true
```

The subnet is pinned so the gateway is always `172.20.0.1` (the hardcoded
`ROS_IP`). If a stale `husky_lan` blocks recreation, remove it first:
`docker network rm husky_lan`.

---

## Step 1 — World + robot (Terminal 1)

```bash
cd ~/Documents/Husky_viz
./load-park-world.sh
```

Wait until Gazebo shows the park + robot, then **wait ~30–60 s** for the GPS/EKF
to converge (the `map` pose stabilizes). Quick check:

```bash
rosnode list | grep -E "ekf_localization|navsat"      # localization up
rostopic echo -n1 /odometry/filtered_map | grep frame_id   # frame_id: "map"
```

---

## Step 2 — move_base with the static map (Terminal 2)

```bash
cd ~/Documents/Husky_viz
roslaunch launch/move_base_gps_map.launch
```

Loads `map_server` (`/map` from `maps/park_map.yaml`) + move_base with the static
map layered under live lidar, in the `map` frame. Quick check:

```bash
rosparam get /move_base/global_costmap/static_map     # true
```

---

## Step 3 — Operator view (Terminal 3)

```bash
cd ~/Documents/Husky_viz/operator
docker compose up -d
docker compose exec operator bash -lc "source /opt/ros/noetic/setup.bash && ./operator/operate.py"
```

- The container runs its own RViz — open **http://localhost:6080/vnc.html**
  (Fixed Frame `map`) to watch the robot, lidar, costmap, and planned route.
- The `exec` line drops you at the `operator>` prompt (Step 4).

---

## Step 4 — Send goals (at the `operator>` prompt)

Three goal forms — all end up as a `map`-frame goal:

```
goal 49.9000094 8.9000327      # by GPS lat/lon (degrees)
goal xy 9.17 13.55             # by map coordinates (metres)
goal garden_table              # by landmark name (from maps/park_places.yaml)
```

Landmark names to try: `bench`, `garden_table`, `lamp`, `trash_bin_1` (exact
name only). A named goal is auto-snapped to the nearest free cell if it lands
inside an obstacle. List all names: `grep -E '^[a-z]' maps/park_places.yaml`.

Other prompt commands: `cancel`, `stop`, `status`, `quit`.

### Drop a surprise obstacle in the robot's path (Terminal 4)

Spawn a box the map doesn't know about, to watch the robot detect it with lidar
and re-route around it. **Set z to sit on the ground (park ground is at z ≈ 2.9,
not 0):** for a 1.5 m box use z = 3.65. Edit `x`/`y` to a point ahead of the
robot.

```bash
export ROS_IP=172.20.0.1 ROS_MASTER_URI=http://172.20.0.1:11311
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
# EDIT x, y to a spot on the robot's route; z=3.65 puts it on the ground.
sp("surprise_box", sdf, "", Pose(position=Point(20.0, 0.0, 3.65),
                                 orientation=Quaternion(0,0,0,1)), "world")
print("spawned surprise_box at (20,0)")
PY
```

Remove it:

```bash
rosservice call /gazebo/delete_model '{model_name: surprise_box}'
```

---

## Regenerate the map (after editing park.world)

```bash
cd ~/Documents/Husky_viz && python3 -m map_tools.extract_park_map
```

Writes `maps/park_map.{pgm,yaml}` + `maps/park_places.yaml`, then restart Step 2.

---

## Known issues

- **`bench` footprint is ~1 m off** the real bench in the map (bench-specific;
  other landmarks are correct). Snap-to-free hides it for goals.
- **Ground is at z ≈ 2.9**, not 0 — spawn test obstacles at `2.9 + half_height`
  or they end up underground/invisible.
- **DWA slow final approach** — the robot can crawl/spin the last ~1 m before it
  settles on the goal. Pre-existing skid-steer tuning; it does arrive.
- If you reset the map EKF config, re-apply the compass-heading fix in
  `natural_environments_ros_opt/husky/husky_control/config/localization_map.yaml`
  (fuse `/compass/data` yaw) or the lidar de-registers when the robot turns.

---

## Stop everything

```bash
pkill -9 -f 'bin/roslaunch' || true; sleep 1
for p in gzserver gzclient gazebo rosmaster move_base ekf_localization navsat \
         robot_state_publisher twist_mux controller_manager map_server rviz; do
  pkill -9 -f "$p" || true
done
# systemd-user children can dodge pattern-kills; kill survivors by PID:
pgrep -f 'gzserver|gazebo|move_base|ekf_localization|map_server' | xargs -r kill -9 2>/dev/null || true

cd ~/Documents/Husky_viz/operator && docker compose down 2>/dev/null || true
docker network rm husky_lan 2>/dev/null || true
```
