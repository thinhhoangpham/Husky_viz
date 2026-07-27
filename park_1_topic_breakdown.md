# park_1.bag — Topic Breakdown

**Bag:** `park_1.bag` — a **ROS 1 Gazebo simulation of a Husky robot** (NOT a real robot).

> **Correction:** an earlier version of this file called this a *ROS 2* bag and cited `sensor_msgs/msg/...` naming as the evidence. That was wrong — no such naming appears anywhere in the file. It is ROS 1, verified against the file bytes. All type names below have been corrected.

**Tells — it is ROS 1:**
- The first 13 bytes of the file are literally `#ROSBAG V2.0`, the ROS 1 bag magic. ROS 2 has no such magic string.
- It is a single 41.5 GB regular file. A ROS 2 bag is a *directory* holding `metadata.yaml` plus `.db3`/`.mcap` files.
- Header record fields are `chunk_count`, `conn_count`, `index_pos` — ROS 1 bag v2.0 fields.
- Every message type uses ROS 1 two-part naming (`sensor_msgs/Imu`, not `sensor_msgs/msg/Imu`). There is not one `/msg/` occurrence in the connection records.
- Consequence: it plays natively under Noetic, no conversion needed.

**Tells — it is simulated, not a real robot:**
- `/gazebo/model_states` and `/gazebo_client/*` topics come only from the Gazebo simulator, and `/gazebo/model_states` runs at 253500 msgs / 253 s = **1000 Hz**, Gazebo's default physics rate.
- Model names are GUI-cloned world objects: `lamp_clone_clone_clone_clone_2_clone_clone_0_clone_1`, `tree_8_clone_2_clone_7_clone_1`. The list also contains `Untitled2` and a bare `/` — artifacts of hand-editing a world file.
- Timestamps are **sim-clock, not wall-clock**: `rosbag info` reports start `Jan 01 1970 05:07:20`, i.e. t≈18440 s from epoch zero, which is Gazebo's `/clock` under `use_sim_time`. The file's own mtime is Dec 21 2021.
- `/imu/data/bias` and `/os0_cloud_node/imu/bias` exist at all. A simulated IMU plugin can publish its injected bias as ground truth; real IMU hardware cannot know its own bias, so no such topic exists on a real robot.
- The GPS is a plugin left on default settings — see the `/navsat/fix` note below.

> **This is not the NEGS-UGV dataset.** `dataset.txt` links the Universidad de Málaga NEGS-UGV field dataset. This bag is *not* that data — at most a simulated counterpart. Any analysis that treats these as real-world sensor readings is mis-framed, and the GPS track in particular is synthetic (see below).

## Ground-truth from simulator

| Topic | Message Type | Count | Notes |
|---|---|---|---|
| `/gazebo/model_states` | `gazebo_msgs/ModelStates` | 253500 | Simulator god's-eye truth: exact pose of every model. Only exists in sim. Highest volume. |
| `/gazebo_client/front_left_speed` | `std_msgs/Float32` | 2521 | Front-left wheel speed, sim's wheel encoder. |
| `/gazebo_client/front_right_speed` | `std_msgs/Float32` | 2521 | Front-right wheel speed, sim's wheel encoder. |
| `/gazebo_client/back_left_speed` | `std_msgs/Float32` | 2521 | Back-left wheel speed, sim's wheel encoder. |
| `/gazebo_client/back_right_speed` | `std_msgs/Float32` | 2521 | Back-right wheel speed, sim's wheel encoder. |

## LiDAR (Ouster OS0)

| Topic | Message Type | Count | Notes |
|---|---|---|---|
| `/os0_cloud_node/points` | `sensor_msgs/PointCloud2` | 2535 | 3D LiDAR point cloud, ~10 Hz. |
| `/os0_cloud_node/imu` | `sensor_msgs/Imu` | 25350 | IMU built into the Ouster LiDAR. |
| `/os0_cloud_node/imu/bias` | `sensor_msgs/Imu` | 25350 | Estimated bias of LiDAR-IMU. |

## Main IMU (body)

| Topic | Message Type | Count | Notes |
|---|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | 12675 | Primary body IMU, ~50 Hz. |
| `/imu/data/bias` | `sensor_msgs/Imu` | 12675 | Estimated bias of that IMU. |

## Localization / motion

| Topic | Message Type | Count | Notes |
|---|---|---|---|
| `/tf` | `tf2_msgs/TFMessage` | 25350 | Moving coordinate transforms over time. |
| `/tf_static` | `tf2_msgs/TFMessage` | 1 | Fixed sensor-mounting transforms, sent once. |
| `/odometry/filtered` | `nav_msgs/Odometry` | 12675 | Fused odometry (EKF). |
| `/navsat/fix` | `sensor_msgs/NavSatFix` | 507 | GPS/GNSS fix, ~2 Hz. |
| `/navigation/objetive_gps` | `std_msgs/Float64MultiArray` | 8 | GPS goal waypoints. |

## Cameras (stereo pair — this bag DOES have cameras)

| Topic | Message Type | Count | Notes |
|---|---|---|---|
| `stereo/camera/left/real/compressed` | `sensor_msgs/CompressedImage` | 6337 | Stereo camera pair, compressed, ~2.5 Hz. |
| `stereo/camera/right/real/compressed` | `sensor_msgs/CompressedImage` | 6337 | Stereo camera pair, compressed, ~2.5 Hz. |
| `/stereo/camera/left/tag/image_raw` | `sensor_msgs/Image` | 6336 | Raw stereo stream, likely AprilTag/fiducial detection. |
| `/stereo/camera/right/tag/image_raw` | `sensor_msgs/Image` | 6336 | Raw stereo stream, likely AprilTag/fiducial detection. |

## Configuration bookkeeping (count = 1 each, not sensor data)

All `.../parameter_descriptions` (`dynamic_reconfigure/ConfigDescription`) and `.../parameter_updates` (`dynamic_reconfigure/Config`) topics for `imu/data`, `os0_cloud_node/imu`, and `navsat/fix`. Published once at startup to record node parameter settings. **Safe to ignore for data analysis.**

| Topic | Message Type | Count |
|---|---|---|
| `/imu/data/accel/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | 1 |
| `/imu/data/accel/parameter_updates` | `dynamic_reconfigure/Config` | 1 |
| `/imu/data/rate/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | 1 |
| `/imu/data/rate/parameter_updates` | `dynamic_reconfigure/Config` | 1 |
| `/imu/data/yaw/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | 1 |
| `/imu/data/yaw/parameter_updates` | `dynamic_reconfigure/Config` | 1 |
| `/os0_cloud_node/imu/accel/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | 1 |
| `/os0_cloud_node/imu/accel/parameter_updates` | `dynamic_reconfigure/Config` | 1 |
| `/os0_cloud_node/imu/rate/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | 1 |
| `/os0_cloud_node/imu/rate/parameter_updates` | `dynamic_reconfigure/Config` | 1 |
| `/os0_cloud_node/imu/yaw/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | 1 |
| `/os0_cloud_node/imu/yaw/parameter_updates` | `dynamic_reconfigure/Config` | 1 |
| `/navsat/fix/status/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | 1 |
| `/navsat/fix/status/parameter_updates` | `dynamic_reconfigure/Config` | 1 |
| `/navsat/fix/velocity/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | 1 |
| `/navsat/fix/velocity/parameter_updates` | `dynamic_reconfigure/Config` | 1 |
| `/navsat/fix/position/parameter_descriptions` | `dynamic_reconfigure/ConfigDescription` | 1 |
| `/navsat/fix/position/parameter_updates` | `dynamic_reconfigure/Config` | 1 |

## Validation against the writeups

| Claim in writeups | Verdict |
|---|---|
| No syscall topic | **CONFIRMED** (none present) |
| `/tf` and `/tf_static` present | **CONFIRMED** (both present) |
| LiDAR, IMU, odom, GPS present | **CONFIRMED** |
| "no-cam" / cameras absent | **WRONG** for this bag: it has a full stereo camera pair |
| "a genuine Husky" | **PARTLY WRONG**: it *is* ROS 1, as the writeups said — but it is a Gazebo simulation of a Husky, not a real robot. The robot *model* is the genuine Clearpath Husky description (see frame tree). |
| `/husky_velocity_controller/*`, `/status` (`husky_msgs`), `/joint_states`, `/rosout`, `/diagnostics` | **ABSENT**; wheel speeds come via `/gazebo_client/*` instead |

## Bottom line

The core thesis (sensors + motion + transforms present, no syscall topic) holds. This specific bag is a **ROS 1** Gazebo simulation of a Husky **with** a stereo camera. So it differs from the writeups' description on two counts — it is simulated rather than a real robot, and it has cameras rather than being camera-less — but it matches them on being ROS 1.

## Sample messages — interpreted

These are the first message of each meaningful topic, read from `park_1.bag`, with what the values mean.

### Sensor / data topics

- **/gazebo/model_states** — 94 models in the world: `parque` (park), `camino_parque` (park path), and dozens of scene props (`trash_bin`, `lamp`, `garden_table`, `bench`, `tree_8`, `arbolpartes4` = "tree parts"). The `*_clone` names are the sim editor duplicating props. Confirms a Gazebo park simulation; Spanish names suggest a Spanish-language source environment.
- **/tf** — first message carries the 4 wheel transforms (`base_link -> front_left_wheel_link`, etc.). Wheel rotation quaternions are non-identity and changing = wheels spinning. Translations (±0.256 x, ±0.2854 y) are wheel positions on the chassis.
- **/os0_cloud_node/imu** — the LiDAR's built-in IMU. Values are non-physical: `angular_velocity.z = 244.5 rad/s` (~14,000 deg/s), impossible for a rolling robot. Raw, uncalibrated sim IMU noise, not real motion.
- **/imu/data** — body IMU, also non-physical: `angular_velocity.x = 122 rad/s`, `linear_acceleration.z = -25`. Likely sim IMU running unfiltered.
- **/os0_cloud_node/imu/bias & /imu/data/bias** — all zeros. Bias estimators publishing but no estimate yet (or nominally zero in sim). Placeholder streams.
- **/odometry/filtered** — the usable motion truth: robot at x=101.9, y=73.5 in the `odom` frame, driving 0.3 m/s forward (`twist.linear.x=0.3`), essentially straight (`angular.z ~ 0`). WARNING: covariance has huge values (~1.4e5) so position uncertainty estimate is enormous — trust the pose loosely.
- **/os0_cloud_node/points** — one LiDAR scan: 19,948 points, 32 bytes each, in frame `os0_lidar`. Fields `x, y, z, intensity, ring` (`ring` = which laser row; Ouster is multi-beam). `height=1, width=19948` = unstructured point list, not a 2D grid.
- **Cameras** — `stereo/.../real/compressed` are JPEG frames ~337 KB each. `stereo/.../tag/image_raw` are 1280x720, raw 8UC3 (3-channel BGR), 2.76 MB each. Both have `header.stamp = 0` — NO valid timestamp, which complicates time-syncing them to LiDAR/odom.
- **/gazebo_client/*_speed** — all four wheels at ~0.9267 (essentially identical) = driving straight, no turning. Consistent with odometry.
- **/navsat/fix** — GPS: lat `49.9004102588982`, lon `8.899999817227329`, alt `3.13`. **These coordinates are synthetic and carry no geographic meaning.** Three reasons: (a) 49.9°N 8.9°E is near Darmstadt, **Germany**, while the simulated world is a Spanish-named park (`parque`, `camino_parque`) from a Málaga-based group at 36.7°N −4.4°E — the world and the GPS disagree by ~1,500 km; (b) the longitude is `8.899999817`, a float-rounding artifact of a hardcoded `8.9`, i.e. a literal default rather than a measurement; (c) `49.9 / 8.9` are the stock reference coordinates shipped as defaults by the common Gazebo GNSS sensor plugin (`hector_gazebo_plugins`), so this is a sim GPS nobody re-configured. Also note `status=0` *and* `service=0` — a real receiver sets `service=1` (GPS constellation). **Do not plot this track on a map or treat it as a real route.**
- **/navigation/objetive_gps** — 9 GPS waypoints (`numero_puntos=9`, `numero_coordenadas=2`) as lat/lon pairs, clustered around 49.900/8.900 — the mission route through the park. These inherit the same synthetic origin as `/navsat/fix`: meaningful only *relative to each other*, not as absolute Earth positions.

### Frame tree (from /tf_static)

```
base_link
├─ base_footprint        (z −0.13)   ground-projection frame
├─ front_bumper_link     (x +0.48)   front of robot
├─ rear_bumper_link      (x −0.48)   back (rotated 180°)
├─ imu_link              (x 0.19, z 0.149, rotated ~90°)  body IMU
├─ inertial_link         (at origin) center of mass
├─ GNSS_link             (x 0.1,  z 0.92)   GPS antenna, high up
├─ os0_sensor            (x 0.09, z 0.79)   LiDAR, top of mast
│   ├─ os0_lidar         (z +0.036)  laser origin
│   └─ os0_imu           (small offset)  LiDAR's IMU
├─ left_camera           (x 0.15, y +0.06, z 0.72)  stereo left
├─ right_camera          (x 0.15, y −0.06, z 0.72)  stereo right  (12 cm baseline)
├─ top_plate_link → top_plate_front/rear_link
├─ top_chassis_link, user_rail_link
└─ sensor_arch_base_link → sensor_arch_mount_link (z +0.51)  the sensor mast
```

- LiDAR and cameras sit ~0.72–0.79 m up on a sensor arch; stereo baseline is 12 cm (left y=+0.06, right y=−0.06); GPS antenna highest at 0.92 m.
- Frame naming (`top_plate_link`, `user_rail_link`, `sensor_arch`) is the standard Clearpath Husky model — a genuine Husky model, simulated.

### Flags for the project

- Camera timestamps are zero (`header.stamp = 0`) — to fuse camera with LiDAR/odom by time, fall back on the bag's record-time, not the message header.
- Raw IMU values are non-physical (244 rad/s) — use `/odometry/filtered` for trustworthy motion; raw IMU streams look unusable as-is.
