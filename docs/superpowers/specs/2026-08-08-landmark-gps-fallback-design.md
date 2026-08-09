# Landmark-Based Fallback for GPS Spoofing (Option B attack) — Design

**Date:** 2026-08-08
**Status:** Design approved; ready for implementation plan.

## Problem

The Option B attack (`attack_navsat_drift.py`, see `RUN-GOAL-HIJACK.md` lines 252–282)
spoofs `/navsat/fix` with a slowly growing offset. `navsat_transform_node` accepts each
fake fix as plausible, so the **map-frame EKF's fused position estimate is dragged
off-route**. The robot does not physically travel to the phantom — it thrashes in place
chasing a target it can never reach, and the mission fails. This is a
**navigation-denial / disorientation** attack.

Crucially, the spoof corrupts exactly ONE thing: the **`map→odom` transform** (the
robot's position within the map frame). It does **not** corrupt:
- the costmap (built from live lidar `/os0_cloud_node/points` — obstacle data is honest),
- the operator's goal (a fixed point in `map`).

## Goal

When GPS is spoofed, the robot must still **reach the operator's true goal**, by taking
its absolute position from **lidar landmark localization against a known map** instead of
GPS. GPS becomes a rejectable input rather than the anchor.

## What already exists (verified this session)

- **Costmap / obstacle avoidance works.** `/os0_cloud_node/points` publishes (~10 Hz,
  ~15k–20k pts) once `GAZEBO_PLUGIN_PATH` includes `~/husky_overlay_ws/devel/lib`
  (`load-park-world.sh` now auto-discovers this). The global costmap is `static_map:
  false`, rolling-window, live-lidar — **not touched by this design**.
- **The current GPS localization is already Option C's structure.** The map-frame EKF
  (`natural_environments_ros_opt/husky/husky_control/config/localization_map.yaml`)
  fuses:
  - `odom0: husky_velocity_controller/odom` (wheel odometry, velocity) — dead reckoning
  - `imu0: imu/data` (heading)
  - `odom1: odometry/gps` (absolute x,y anchor) — **the spoofed input**

  i.e. dead reckoning corrected by a periodic absolute anchor. Today the anchor is GPS;
  the fallback swaps a landmark-derived anchor into the same `odom1` slot.
- **Lidar is NOT a localization input today** (confirmed: no pointcloud in either EKF
  config). It only feeds the costmap.
- **The map already exists as ground-truth data.** The park world
  `natural_environments_ros_opt/natural_enviroment/worlds/park.world` (the exact world
  Gazebo renders, via the overlay symlink) contains **91 point-object landmarks** with
  exact `(x, y)` in the `map` frame:

  | type | count |
  |---|---|
  | tree | 38 |
  | bench | 16 |
  | lamp | 15 |
  | trash_bin | 11 |
  | table | 11 |
  | **total** | **91** |

  Spatially well-distributed across the ~100×50 m park (x∈[−50,48], y∈[−28,22]).
  **No SLAM / no map-building is required** — the map is a list of known points.

## Approach — Option C: dead reckoning + lidar landmark-matching correction

This is the recognised GPS-denied / GPS-spoofed navigation pattern in the literature:
dead reckoning (INS/wheel odometry) whose unbounded drift is bounded by an external
reference — here, lidar matched against a known landmark map. See "References" below.

### Components

**1. Offline — landmark map extraction (DONE, proven)**
- `extract_landmarks.py` parses `park.world` → `park_landmarks.json`:
  `{frame: "map", count: 91, landmarks: [{name, type, x, y}, ...]}`.
- This is "the map the robot knows." Static survey data — legitimate under the
  no-ground-truth rule (a real deployment surveys its environment; pose still comes from
  the robot's own lidar, never Gazebo's live pose).

**2. Runtime — landmark localizer node (NEW)**
- Subscribes to live `/os0_cloud_node/points`.
- Detects vertical trunk/pole-like clusters at trunk height (verified: a slab
  `z∈[-0.70,-0.40]` in the `os0_lidar` frame yields ~410 stable points/frame — ground is
  ~0.83 m below the lidar).
- Matches the observed cluster pattern against the 91 known landmark coordinates → solves
  for the absolute pose in the `map` frame that makes them align.
- Publishes `/odometry/landmark` (`nav_msgs/Odometry`, absolute x,y in `map`), the same
  message *shape* the EKF's `odom1` slot already consumes from GPS.
- Uses dead-reckoned pose (odom + compass) as the prior/seed for the match each tick, so
  matching is a local association, not a global search.

**3. Runtime — gated relay node (NEW, the switch mechanism)**
- Sits between the two absolute sources and the EKF. The EKF's `odom1` is remapped (once,
  at launch) to consume the relay's output.
- Holds a "trust" flag:
  - **GPS trusted (default):** relay passes `/odometry/gps` → EKF navigates on GPS
    (normal operation, unchanged behaviour).
  - **Landmark trusted:** relay passes `/odometry/landmark`, blocks GPS → EKF navigates
    on landmarks. Pose **snaps** to truth (accepted transient); move_base replans.
- Exposes a ROS service to flip the flag. Reversible (`trust gps`).
- The EKF's own config is not changed at runtime — only which topic feeds `odom1`.

**4. Operator interface — `operate.py` extensions (NEW)**
- **Display:** show both the GPS-fused pose and the landmark pose (or their divergence),
  so the operator can *see* the spoof — this is the signal they act on. Extends the
  existing `operator/operate.py` / `plot_operator_status.py`.
- **Command:** REPL commands `trust landmark` / `trust gps` that call the gated-relay
  service.

### Trust policy — manual now, automatic later

**Now: manual operator switch.** The operator watches the divergence and types `trust
landmark`. Human-in-the-loop; simplest to demo the "see it, decide" story.

**Documented for later: automatic chi-squared rejection.** The realistic deployed
countermeasure is continuous fusion that auto-rejects the spoofed GPS via a residual /
deviation test (robot_localization already has a Mahalanobis outlier gate). This is the
recommended future upgrade — not built in v1. See References.

### Data flow

```
park.world ──(offline)──► park_landmarks.json (91 pts)
                                    │
live /os0_cloud_node/points ──► landmark_localizer ──► /odometry/landmark ──┐
                                                                            ├─► gated_relay ─► EKF odom1 ─► map→odom ─► move_base
/navsat/fix ──► navsat_transform ──► /odometry/gps ─────────────────────────┘        ▲              (costmaps + planner UNCHANGED)
                                                                    operator: `trust landmark`
                                                                    (sees divergence on operate.py)
```

## Explicitly NOT touched

- Global/local costmaps (stay rolling live-lidar; the landmark map is localization-only,
  never a costmap).
- move_base, global/local planners, the odom-frame EKF, TF tree topology.
- The EKF config at runtime (only the `odom1` source topic is relayed).
- The flat path (`camino_parque`): it is park-spanning geometry, not a point landmark, so
  it is excluded from v1. It could later serve as a reflectivity / on-path constraint.

## Testing / success criteria

Per the no-ground-truth rule: judged by the robot's own honest sensors + the operator's
Gazebo view. **Never** verified against Gazebo ground truth.

1. **Landmark map:** `park_landmarks.json` has 91 landmarks in the `map` frame. (DONE)
2. **Localizer nominal:** with GPS healthy, `/odometry/landmark` agrees with the
   GPS-fused pose (validates the localizer before it is ever relied on). This also proves
   the localizer is warm/converged before any switch.
3. **Under attack (the real test):** run `attack_navsat_drift.py` → operator sees
   divergence grow on the display → types `trust landmark` → GPS is cut, pose snaps to
   truth, and the **robot reaches the real goal** instead of thrashing in place.

## Known risks

- **Landmark association ambiguity in open stretches.** Mitigated by seeding the match
  with dead-reckoned pose and by the dense, well-distributed 91-landmark set. If a leg is
  landmark-sparse, the robot coasts on dead reckoning (bounded) until the next sighting.
- **Cluster detection tuning.** Distinguishing trunk/pole clusters from canopy/ground in
  the live cloud needs tuning (trunk-height band verified; detector still to build).
- **Compute:** landmark detection + matching at lidar rate on the Quadro P4000, alongside
  gzserver — verify it keeps up.
- **Warm-up:** the localizer must be converged before the operator switches, or the snap
  goes to a bad pose. Test #2 covers this.

## References (GPS-denied / GPS-spoofed navigation — for the automatic-rejection upgrade)

- LiDAR Scan Matching Aided INS in GNSS-Denied Environments (incl. dense forests):
  https://pmc.ncbi.nlm.nih.gov/articles/PMC4541902/
- GPS-Denied LiDAR-Based SLAM — A Survey (IET/Wiley, 2025):
  https://ietresearch.onlinelibrary.wiley.com/doi/full/10.1049/csy2.70031
- Spoofing-Resilient LiDAR-GPS Factor Graph Localization (chi-squared residual detector):
  https://arxiv.org/html/2307.04692
- Secure localization of land vehicles under GPS spoofing (deviation between GPS and
  DR-estimated location, thresholded): https://www.nature.com/articles/s41598-025-32863-5
- GPS Spoofing Attack Detection in AVs Using Adaptive DBSCAN:
  https://arxiv.org/html/2510.10766v1
