#!/usr/bin/env python3
"""Diagnose the DWA "goal critic rejects 100% of trajectories with cost -2.0" stall.

Background and the full evidence trail: docs/dwa-unreachable-goal-investigation.md.

WHAT THIS TESTS
---------------
cost -2.0 out of base_local_planner's MapGridCostFunction means the scored cell held
`unreachableCellCosts()` = map_.size()+1 -- i.e. the wavefront NEVER VISITED it. It is
NOT the obstacle case (-3.0) nor off-map (-4.0).

In base_local_planner/map_grid.cpp, `updatePathCell` treats NO_INFORMATION (255) exactly
like LETHAL_OBSTACLE (254) and INSCRIBED_INFLATED_OBSTACLE (253): it sets the cell to
obstacleCosts() and returns false, so the cell is never pushed onto the BFS queue. Unknown
cells are a WALL to the wavefront.

The investigation doc measured unknown-cell fraction AROUND THE ROBOT and found 0.0%, and
concluded unknown cells were not the cause. But `computeTargetDistance` floods FROM THE
SEED chosen by `setLocalGoal` -- which sits out at the edge of the 10x10 m rolling window --
BACK TOWARD the robot. What matters is therefore unknown coverage at the SEED and along the
seed->robot corridor, not around the robot. This script measures there.

The decisive measurement is D4: a 4-connected flood fill from the seed cell over cells whose
cost is not in {255, 254, 253}. If that flood does not reach the robot footprint, the
wavefront genuinely cannot deliver a finite goal_cost to the trajectories and the hypothesis
is confirmed. If it does reach, the grid DWA scores is not the grid being published, and the
hypothesis is dead.

MODES
-----
  * rolling time-series at ~2 Hz for the whole run (to find the ONSET, not just the end state)
  * a one-shot full dump (D1-D11) latched the first time a stall is detected

NO GROUND TRUTH: pose comes from TF and from sensor topics only. Nothing here reads
/gazebo/model_states, /gazebo/get_model_state, or imports gazebo_msgs.

Outputs land in artifacts/dwa_stall/:
  <ts>_timeseries.csv, <ts>_stall_dump.json, <ts>_costgrid.npy, <ts>_costgrid.png
"""

import argparse
import collections
import json
import math
import os
import re
import sys
import time

import numpy as np

import rospy
import tf2_ros
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rosgraph_msgs.msg import Log
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

try:
    from map_msgs.msg import OccupancyGridUpdate
except ImportError:  # map_msgs is normally present with Noetic's nav stack
    OccupancyGridUpdate = None


# --- costmap_2d sentinel values (costmap_2d/cost_values.h) --------------------
NO_INFORMATION = 255
LETHAL_OBSTACLE = 254
INSCRIBED_INFLATED_OBSTACLE = 253
FREE_SPACE = 0

# Cells the wavefront refuses to expand through (map_grid.cpp updatePathCell).
BLOCKING = (NO_INFORMATION, LETHAL_OBSTACLE, INSCRIBED_INFLATED_OBSTACLE)

# Geometry of the rolling window vs the sensor config (see the doc):
# the window reaches 5 m in +/-x and +/-y, so its CORNERS are 7.07 m out, but
# raytrace_range is only 6.0 m -- cells beyond 6 m can never be raytraced clear.
RAYTRACE_RANGE = 6.0
WINDOW_CORNER_RANGE = math.sqrt(5.0 ** 2 + 5.0 ** 2)

_REJECT_RE = re.compile(
    r"discarded by cost function\s+(-?\d+)\s+with cost:\s*(-?[\d.]+)")
_EVAL_RE = re.compile(r"Evaluated\s+(\d+)\s+trajectories,\s+found\s+(\d+)\s+valid")


# =============================================================================
# OccupancyGrid -> costmap_2d value remap
# =============================================================================
def occ_to_costmap(data):
    """Map published OccupancyGrid int8 values back to raw costmap_2d 0..255.

    THIS IS THE SINGLE EASIEST THING TO GET WRONG IN THIS SCRIPT -- every
    measurement below depends on it.

    Costmap2DPublisher (costmap_2d_publisher.cpp) builds a 256-entry lookup and
    publishes the LOSSY projection:

        NO_INFORMATION (255)              -> -1
        LETHAL_OBSTACLE (254)             -> 100
        INSCRIBED_INFLATED_OBSTACLE (253) ->  99
        everything else (0..252)          -> char(1 + (251 * c) / 251) roughly,
                                             i.e. a linear squeeze onto 0..98

    We invert that: -1/100/99 map back to their exact sentinels; the remaining
    0..98 band is rescaled to 0..252. The inverse of the linear band is not
    bit-exact (the forward map is many-to-one), but it does not need to be:
    every gate in this script keys off the three sentinels and off "is it zero",
    both of which round-trip exactly.
    """
    occ = np.asarray(data, dtype=np.int16)
    cost = np.zeros(occ.shape, dtype=np.uint8)

    unknown = occ < 0
    lethal = occ == 100
    inscribed = occ == 99
    other = ~(unknown | lethal | inscribed)

    # Linear band: 0..98 -> 0..252. occ==0 -> 0 exactly (FREE_SPACE round-trips).
    scaled = np.round(occ[other].astype(np.float32) * (252.0 / 98.0))
    cost[other] = np.clip(scaled, 0, 252).astype(np.uint8)
    cost[inscribed] = INSCRIBED_INFLATED_OBSTACLE
    cost[lethal] = LETHAL_OBSTACLE
    cost[unknown] = NO_INFORMATION
    return cost


# =============================================================================
# Live local costmap (full grid + incremental updates)
# =============================================================================
class LocalCostmap(object):
    """Holds the current local costmap, applying OccupancyGridUpdate patches."""

    def __init__(self):
        self.cost = None           # uint8 [h, w], raw costmap_2d values
        self.resolution = None
        self.origin_x = None
        self.origin_y = None
        self.frame_id = None
        self.stamp = None

    # -- callbacks --
    def on_grid(self, msg):
        w, h = msg.info.width, msg.info.height
        self.cost = occ_to_costmap(msg.data).reshape(h, w)
        self.resolution = msg.info.resolution
        self.origin_x = msg.info.origin.position.x
        self.origin_y = msg.info.origin.position.y
        self.frame_id = msg.header.frame_id
        self.stamp = msg.header.stamp

    def on_update(self, msg):
        if self.cost is None:
            return
        h, w = self.cost.shape
        if msg.x < 0 or msg.y < 0 or msg.x + msg.width > w or msg.y + msg.height > h:
            rospy.logwarn_throttle(
                10.0, "[dwa_probe] costmap_update out of bounds -- ignoring patch")
            return
        patch = occ_to_costmap(msg.data).reshape(msg.height, msg.width)
        self.cost[msg.y:msg.y + msg.height, msg.x:msg.x + msg.width] = patch
        self.stamp = msg.header.stamp
        # NOTE: rolling-window costmaps republish the FULL grid whenever the
        # window origin moves, so the origin stored above stays consistent with
        # the patched contents.

    # -- geometry --
    @property
    def ready(self):
        return self.cost is not None

    def world_to_map(self, wx, wy):
        """costmap_2d::Costmap2D::worldToMap -- returns None when out of bounds."""
        if wx < self.origin_x or wy < self.origin_y:
            return None
        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)
        h, w = self.cost.shape
        if mx < w and my < h:
            return (mx, my)
        return None

    def map_to_world(self, mx, my):
        return (self.origin_x + (mx + 0.5) * self.resolution,
                self.origin_y + (my + 0.5) * self.resolution)

    def get_cost(self, mx, my):
        return int(self.cost[my, mx])


# =============================================================================
# D1 -- setLocalGoal replication
# =============================================================================
def adjust_plan_resolution(plan_xy, resolution):
    """base_local_planner::adjustPlanResolution.

    Densifies the plan so no two consecutive points are further apart than the
    costmap resolution, inserting linearly interpolated points. Reproduced here
    because the seed choice below iterates the DENSIFIED plan, and a sparse plan
    can skip straight over the unknown cell that would have ended the valid run.
    """
    if len(plan_xy) < 2:
        return list(plan_xy)
    out = [plan_xy[0]]
    for i in range(1, len(plan_xy)):
        x0, y0 = plan_xy[i - 1]
        x1, y1 = plan_xy[i]
        dist = math.hypot(x1 - x0, y1 - y0)
        if dist > resolution:
            n = int(dist / resolution)
            for j in range(1, n + 1):
                t = float(j) / (n + 1)
                out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
        out.append((x1, y1))
    return out


def compute_local_goal(grid, plan_xy):
    """Replicate MapGrid::setLocalGoal's seed selection.

    Acceptance rule (map_grid.cpp): a plan point counts only if worldToMap()
    succeeds AND getCost() != NO_INFORMATION. The function keeps the LAST valid
    run of points and breaks at the first invalid point encountered AFTER at
    least one valid point has been seen. The wavefront then floods from that
    final accepted point.
    """
    result = {
        "seed_found": False,
        "seed_map_x": None, "seed_map_y": None,
        "seed_world_x": None, "seed_world_y": None,
        "seed_index_in_dense_plan": None,
        "dense_plan_len": 0,
        "plan_poses_in_bounds": 0,
        "plan_poses_on_unknown": 0,
        "raw_plan_len": len(plan_xy),
    }
    if not grid.ready or not plan_xy:
        return result

    dense = adjust_plan_resolution(plan_xy, grid.resolution)
    result["dense_plan_len"] = len(dense)

    local_goal_x = local_goal_y = -1
    seed_idx = None
    started = False
    for i, (wx, wy) in enumerate(dense):
        cell = grid.world_to_map(wx, wy)
        if cell is None:
            if started:
                break
            continue
        result["plan_poses_in_bounds"] += 1
        mx, my = cell
        if grid.get_cost(mx, my) == NO_INFORMATION:
            result["plan_poses_on_unknown"] += 1
            if started:
                break
            continue
        started = True
        local_goal_x, local_goal_y = mx, my
        seed_idx = i

    if local_goal_x < 0:
        return result

    wx, wy = grid.map_to_world(local_goal_x, local_goal_y)
    result.update(seed_found=True, seed_map_x=local_goal_x, seed_map_y=local_goal_y,
                  seed_world_x=wx, seed_world_y=wy, seed_index_in_dense_plan=seed_idx)
    return result


# =============================================================================
# D2/D3/D4/D11 -- grid analyses
# =============================================================================
def cost_histogram(values):
    """Counts at each sentinel plus free/other, from a flat array of raw costs."""
    values = np.asarray(values, dtype=np.uint8)
    return {
        "total": int(values.size),
        "no_information_255": int(np.count_nonzero(values == NO_INFORMATION)),
        "lethal_254": int(np.count_nonzero(values == LETHAL_OBSTACLE)),
        "inscribed_253": int(np.count_nonzero(values == INSCRIBED_INFLATED_OBSTACLE)),
        "free_0": int(np.count_nonzero(values == FREE_SPACE)),
        "other": int(np.count_nonzero(
            (values != NO_INFORMATION) & (values != LETHAL_OBSTACLE) &
            (values != INSCRIBED_INFLATED_OBSTACLE) & (values != FREE_SPACE))),
    }


def corridor_cells(grid, a, b, half_width_m):
    """Cells within `half_width_m` of the segment a->b (both cells), as a mask."""
    h, w = grid.cost.shape
    ys, xs = np.mgrid[0:h, 0:w]
    ax, ay = float(a[0]), float(a[1])
    bx, by = float(b[0]), float(b[1])
    dx, dy = bx - ax, by - ay
    seg_len_sq = dx * dx + dy * dy
    if seg_len_sq < 1e-9:
        t = np.zeros_like(xs, dtype=np.float32)
    else:
        t = ((xs - ax) * dx + (ys - ay) * dy) / seg_len_sq
        t = np.clip(t, 0.0, 1.0)
    px, py = ax + t * dx, ay + t * dy
    dist_cells = np.hypot(xs - px, ys - py)
    return dist_cells <= (half_width_m / grid.resolution)


def annulus_histogram(grid, robot_cell, r_inner, r_outer):
    """D3 -- cost counts in the r_inner..r_outer m annulus around the robot."""
    h, w = grid.cost.shape
    ys, xs = np.mgrid[0:h, 0:w]
    d = np.hypot(xs - robot_cell[0], ys - robot_cell[1]) * grid.resolution
    mask = (d >= r_inner) & (d <= r_outer)
    hist = cost_histogram(grid.cost[mask])
    hist["r_inner_m"] = r_inner
    hist["r_outer_m"] = r_outer
    return hist


def flood_from_seed(grid, seed_cell, footprint_cells):
    """D4 -- 4-connected flood from the seed over non-blocking cells.

    This is exactly MapGrid::computeTargetDistance's reachable set:
    that BFS expands each cell's 4 neighbours and calls updatePathCell, which
    refuses to enqueue any cell whose cost is NO_INFORMATION, LETHAL_OBSTACLE or
    INSCRIBED_INFLATED_OBSTACLE. So the set of cells that end up with a FINITE
    target_dist is precisely the 4-connected component of non-blocking cells
    containing the seed. Anything outside it keeps unreachableCellCosts()
    (40001) and scores -2.0 -- the value observed at the stall.
    """
    h, w = grid.cost.shape
    sx, sy = seed_cell
    out = {"reached_robot": False, "cells_visited": 0, "bbox": None,
           "seed_blocked": False}

    if grid.cost[sy, sx] in BLOCKING:
        # setLocalGoal only rejects NO_INFORMATION, so a seed on a lethal or
        # inscribed cell is possible and would stop the wavefront immediately.
        out["seed_blocked"] = True
        return out

    blocked = np.isin(grid.cost, BLOCKING)
    visited = np.zeros((h, w), dtype=bool)
    visited[sy, sx] = True
    queue = collections.deque([(sx, sy)])
    minx = maxx = sx
    miny = maxy = sy
    count = 0
    while queue:
        x, y = queue.popleft()
        count += 1
        if x < minx: minx = x
        if x > maxx: maxx = x
        if y < miny: miny = y
        if y > maxy: maxy = y
        for nx, ny in ((x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1)):
            if 0 <= nx < w and 0 <= ny < h and not visited[ny, nx] and not blocked[ny, nx]:
                visited[ny, nx] = True
                queue.append((nx, ny))

    out["cells_visited"] = count
    out["bbox"] = {"min_x": int(minx), "max_x": int(maxx),
                   "min_y": int(miny), "max_y": int(maxy)}
    out["reached_robot"] = bool(any(
        visited[fy, fx] for fx, fy in footprint_cells
        if 0 <= fx < w and 0 <= fy < h))
    out["_visited"] = visited
    return out


def footprint_cells_for(grid, rx, ry, yaw, half_len=0.5, half_wid=0.33):
    """Cells covered by the Husky footprint (config/costmap_common_gps.yaml box)."""
    cells = set()
    cs, sn = math.cos(yaw), math.sin(yaw)
    n = max(2, int(2 * max(half_len, half_wid) / grid.resolution) + 1)
    for u in np.linspace(-half_len, half_len, n):
        for v in np.linspace(-half_wid, half_wid, n):
            cell = grid.world_to_map(rx + u * cs - v * sn, ry + u * sn + v * cs)
            if cell is not None:
                cells.add(cell)
    if not cells:
        cell = grid.world_to_map(rx, ry)
        if cell is not None:
            cells.add(cell)
    return sorted(cells)


def nearest_lethal(grid, robot_cell):
    """D11 -- distance to the nearest lethal cell, plus lethal/inscribed totals."""
    lethal_idx = np.argwhere(grid.cost == LETHAL_OBSTACLE)
    inscribed_count = int(np.count_nonzero(grid.cost == INSCRIBED_INFLATED_OBSTACLE))
    out = {"lethal_count": int(lethal_idx.shape[0]),
           "inscribed_count": inscribed_count,
           "nearest_lethal_m": None}
    if lethal_idx.size:
        d = np.hypot(lethal_idx[:, 1] - robot_cell[0],
                     lethal_idx[:, 0] - robot_cell[1]) * grid.resolution
        out["nearest_lethal_m"] = float(d.min())
    return out


# =============================================================================
# Point cloud range bins (D8/D9)
# =============================================================================
def range_bins(cloud_msg, bins=((0, 2), (2, 4), (4, 6), (6, 10))):
    """Count cloud points per range bin, measured in the SENSOR frame.

    The cloud is in the lidar frame, whose origin is the sensor -- close enough
    to the robot centre (a fraction of a metre) for the blind-ring question,
    and it avoids a TF transform of ~100k points at 2 Hz.
    """
    out = {"%d-%d" % b: 0 for b in bins}
    out["total"] = 0
    if cloud_msg is None:
        return None
    for x, y, z in point_cloud2.read_points(
            cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
        r = math.sqrt(x * x + y * y + z * z)
        out["total"] += 1
        for lo, hi in bins:
            if lo <= r < hi:
                out["%d-%d" % (lo, hi)] += 1
                break
    return out


# =============================================================================
# The probe node
# =============================================================================
class StallProbe(object):
    def __init__(self, args):
        self.args = args
        self.grid = LocalCostmap()

        self.cmd = None
        self.cmd_time = None
        self.plan = []              # [(x, y)] in the plan's own frame
        self.plan_frame = None
        self.goal = None            # (x, y, frame)
        self.status = None
        self.raw_cloud = None
        self.filtered_cloud = None
        self.cost_cloud = None
        self.ground_z = None        # from /odometry/ground_height_odom (no ground truth)

        self.rejections = collections.Counter()     # gen_id -> count
        self.rejection_costs = collections.defaultdict(collections.Counter)
        self.eval_cycles = 0
        self.eval_zero_valid = 0

        self.stall_since = None
        self.latched = False

        self.tf_buf = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buf)

        self.seen_topics = set()
        self._subscribe()

        os.makedirs(args.outdir, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S")
        self.ts_path = os.path.join(args.outdir, "%s_timeseries.csv" % stamp)
        self.dump_path = os.path.join(args.outdir, "%s_stall_dump.json" % stamp)
        self.npy_path = os.path.join(args.outdir, "%s_costgrid.npy" % stamp)
        self.png_path = os.path.join(args.outdir, "%s_costgrid.png" % stamp)
        self.csv = open(self.ts_path, "w")
        self.csv.write("stamp,robot_x,robot_y,robot_z,dist_to_goal,cmd_lin_x,cmd_ang_z,"
                       "unknown_cells,lethal_cells,inscribed_cells,seed_map_x,seed_map_y,"
                       "flood_reaches_robot\n")
        self.csv.flush()

    # -- subscriptions ------------------------------------------------------
    def _sub(self, topic, msg_type, cb):
        self.seen_topics.add(topic)
        return rospy.Subscriber(topic, msg_type, cb, queue_size=1)

    def _subscribe(self):
        a = self.args
        self._sub(a.costmap_topic, OccupancyGrid, self.grid.on_grid)
        if OccupancyGridUpdate is not None:
            self._sub(a.costmap_topic + "_updates", OccupancyGridUpdate, self.grid.on_update)
        else:
            rospy.logwarn("[dwa_probe] map_msgs not importable -- costmap_updates "
                          "will be ignored; the full grid is still tracked")
        self._sub(a.plan_topic, Path, self.on_plan)
        self._sub(a.plan_fallback_topic, Path, self.on_plan_fallback)
        self._sub(a.cmd_vel_topic, Twist, self.on_cmd)
        self._sub("/move_base/status", GoalStatusArray, self.on_status)
        self._sub("/move_base/current_goal", PoseStamped, self.on_goal)
        self._sub(a.raw_cloud_topic, PointCloud2, self.on_raw_cloud)
        self._sub(a.filtered_cloud_topic, PointCloud2, self.on_filtered_cloud)
        self._sub(a.cost_cloud_topic, PointCloud2, self.on_cost_cloud)
        self._sub("/odometry/ground_height_odom", Odometry, self.on_ground_height)
        self._sub("/rosout", Log, self.on_rosout)

    # -- callbacks ----------------------------------------------------------
    def on_plan(self, msg):
        if msg.poses:
            self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.plan_frame = msg.header.frame_id
            self._plan_from_fallback = False

    def on_plan_fallback(self, msg):
        # Only used if the DWA-transformed plan never arrives.
        if msg.poses and not self.plan:
            self.plan = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
            self.plan_frame = msg.header.frame_id
            self._plan_from_fallback = True

    def on_cmd(self, msg):
        self.cmd = msg
        self.cmd_time = rospy.Time.now()

    def on_status(self, msg):
        self.status = msg.status_list[-1].status if msg.status_list else None

    def on_goal(self, msg):
        self.goal = (msg.pose.position.x, msg.pose.position.y, msg.header.frame_id)

    def on_raw_cloud(self, msg):
        self.raw_cloud = msg

    def on_filtered_cloud(self, msg):
        self.filtered_cloud = msg

    def on_cost_cloud(self, msg):
        self.cost_cloud = msg

    def on_ground_height(self, msg):
        self.ground_z = msg.pose.pose.position.z

    def on_rosout(self, msg):
        """D7 -- tally DWA's DEBUG rejection lines (only present at DEBUG level)."""
        m = _REJECT_RE.search(msg.msg)
        if m:
            gen_id = int(m.group(1))
            self.rejections[gen_id] += 1
            self.rejection_costs[gen_id]["%.3f" % float(m.group(2))] += 1
            return
        m = _EVAL_RE.search(msg.msg)
        if m:
            self.eval_cycles += 1
            if int(m.group(2)) == 0:
                self.eval_zero_valid += 1

    # -- pose ---------------------------------------------------------------
    def robot_pose(self, frame):
        """Robot pose in `frame` from TF. Returns (x, y, z, roll, pitch, yaw) or None."""
        try:
            tr = self.tf_buf.lookup_transform(
                frame, self.args.base_frame, rospy.Time(0), rospy.Duration(0.3))
        except Exception as exc:  # tf2 raises several distinct exception types
            rospy.logwarn_throttle(5.0, "[dwa_probe] TF %s->%s failed: %s"
                                   % (frame, self.args.base_frame, exc))
            return None
        t = tr.transform.translation
        q = tr.transform.rotation
        # quaternion -> RPY (avoids a tf.transformations import)
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        return (t.x, t.y, t.z, roll, pitch, yaw)

    def plan_in_grid_frame(self):
        """The plan, expressed in the local costmap frame.

        DWA's own global_plan is already published in the local costmap frame,
        so the common path is a no-op. The NavfnROS fallback is in `map` and
        needs a transform; if that transform is unavailable we return nothing
        rather than silently mixing frames.
        """
        if not self.plan or not self.grid.ready:
            return []
        if not self.plan_frame or self.plan_frame.lstrip("/") == \
                (self.grid.frame_id or "").lstrip("/"):
            return self.plan
        try:
            tr = self.tf_buf.lookup_transform(
                self.grid.frame_id, self.plan_frame, rospy.Time(0), rospy.Duration(0.3))
        except Exception as exc:
            rospy.logwarn_throttle(5.0, "[dwa_probe] cannot transform plan %s->%s: %s"
                                   % (self.plan_frame, self.grid.frame_id, exc))
            return []
        t = tr.transform.translation
        q = tr.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        cs, sn = math.cos(yaw), math.sin(yaw)
        return [(t.x + x * cs - y * sn, t.y + x * sn + y * cs) for x, y in self.plan]

    def dist_to_goal(self):
        if self.goal is None:
            return None
        pose = self.robot_pose(self.goal[2] or "map")
        if pose is None:
            return None
        return math.hypot(self.goal[0] - pose[0], self.goal[1] - pose[1])

    # -- stall detection ----------------------------------------------------
    def check_stall(self, dist):
        a = self.args
        now = rospy.Time.now()
        if self.cmd is None or self.cmd_time is None:
            return False
        if (now - self.cmd_time).to_sec() > a.cmd_stale_sec:
            # No cmd_vel at all is also a stall symptom, but we cannot
            # distinguish it from "publisher down", so treat it as zero.
            zero = True
        else:
            zero = (abs(self.cmd.linear.x) < a.lin_thresh and
                    abs(self.cmd.angular.z) < a.ang_thresh)
        if not zero or dist is None or dist <= a.min_goal_dist:
            self.stall_since = None
            return False
        if self.stall_since is None:
            self.stall_since = now
            return False
        return (now - self.stall_since).to_sec() >= a.stall_hold_sec

    # -- the tick -----------------------------------------------------------
    def tick(self, _evt=None):
        if not self.grid.ready:
            rospy.logwarn_throttle(10.0, "[dwa_probe] waiting for %s"
                                   % self.args.costmap_topic)
            return
        pose = self.robot_pose(self.grid.frame_id)
        if pose is None:
            return
        rx, ry, rz = pose[0], pose[1], pose[2]
        robot_cell = self.grid.world_to_map(rx, ry)
        dist = self.dist_to_goal()

        unknown = int(np.count_nonzero(self.grid.cost == NO_INFORMATION))
        lethal = int(np.count_nonzero(self.grid.cost == LETHAL_OBSTACLE))
        inscribed = int(np.count_nonzero(self.grid.cost == INSCRIBED_INFLATED_OBSTACLE))

        plan = self.plan_in_grid_frame()
        seed = compute_local_goal(self.grid, plan)

        flood_flag = ""
        if self.args.flood_every_tick and seed["seed_found"] and robot_cell:
            fp = footprint_cells_for(self.grid, rx, ry, pose[5])
            res = flood_from_seed(self.grid, (seed["seed_map_x"], seed["seed_map_y"]), fp)
            flood_flag = "1" if res["reached_robot"] else "0"

        self.csv.write("%.3f,%.3f,%.3f,%.3f,%s,%.4f,%.4f,%d,%d,%d,%s,%s,%s\n" % (
            rospy.Time.now().to_sec(), rx, ry, rz,
            "" if dist is None else "%.3f" % dist,
            0.0 if self.cmd is None else self.cmd.linear.x,
            0.0 if self.cmd is None else self.cmd.angular.z,
            unknown, lethal, inscribed,
            "" if seed["seed_map_x"] is None else seed["seed_map_x"],
            "" if seed["seed_map_y"] is None else seed["seed_map_y"],
            flood_flag))
        self.csv.flush()

        if not self.latched and self.check_stall(dist):
            self.latched = True
            try:
                self.dump(pose, robot_cell, seed, dist)
            except Exception as exc:
                rospy.logerr("[dwa_probe] stall dump failed: %s" % exc)
            if self.args.exit_after_dump:
                rospy.signal_shutdown("dump complete")

    # -- the payload --------------------------------------------------------
    def dump(self, pose, robot_cell, seed, dist):
        a = self.args
        grid = self.grid
        rx, ry, rz, roll, pitch, yaw = pose
        d = {
            "stall_time": rospy.Time.now().to_sec(),
            "wall_time": time.strftime("%Y-%m-%d %H:%M:%S"),
            "robot": {"x": rx, "y": ry, "z": rz, "roll": roll, "pitch": pitch,
                      "yaw": yaw, "frame": grid.frame_id,
                      "map_x": None if robot_cell is None else robot_cell[0],
                      "map_y": None if robot_cell is None else robot_cell[1]},
            "goal": None if self.goal is None else
                    {"x": self.goal[0], "y": self.goal[1], "frame": self.goal[2]},
            "dist_to_goal": dist,
            "move_base_status": self.status,
            "costmap": {"width": int(grid.cost.shape[1]),
                        "height": int(grid.cost.shape[0]),
                        "resolution": grid.resolution,
                        "origin_x": grid.origin_x, "origin_y": grid.origin_y,
                        "frame": grid.frame_id},
            "grid_histogram": cost_histogram(grid.cost.ravel()),
            "D1_seed": seed,
            "notes": [],
        }
        if getattr(self, "_plan_from_fallback", False):
            d["notes"].append("plan came from the %s fallback, not %s"
                              % (a.plan_fallback_topic, a.plan_topic))

        seed_cell = None
        if seed["seed_found"]:
            seed_cell = (seed["seed_map_x"], seed["seed_map_y"])
        else:
            d["notes"].append("NO VALID SEED: setLocalGoal replication found no "
                              "acceptable plan point (empty plan, plan out of the "
                              "window, or every in-bounds point on NO_INFORMATION)")

        # D2 -- seed->robot corridor histogram
        if seed_cell and robot_cell:
            mask = corridor_cells(grid, seed_cell, robot_cell, a.corridor_half_width)
            hist = cost_histogram(grid.cost[mask])
            hist["half_width_m"] = a.corridor_half_width
            d["D2_corridor_histogram"] = hist
        else:
            d["D2_corridor_histogram"] = None

        # D3 -- geometrically unclearable annulus (6.0 m .. 7.07 m)
        d["D3_annulus_histogram"] = (
            annulus_histogram(grid, robot_cell, a.annulus_inner, a.annulus_outer)
            if robot_cell else None)

        # D4 -- the decisive flood fill
        verdict = "INCONCLUSIVE"
        if seed_cell and robot_cell:
            fp = footprint_cells_for(grid, rx, ry, yaw)
            flood = flood_from_seed(grid, seed_cell, fp)
            visited = flood.pop("_visited", None)
            flood["footprint_cell_count"] = len(fp)
            d["D4_flood"] = flood
            if flood["seed_blocked"]:
                verdict = ("SEED CELL IS ITSELF BLOCKED (lethal/inscribed) -> the "
                           "wavefront cannot start at all")
            elif flood["reached_robot"]:
                verdict = ("FLOOD FROM SEED REACHES ROBOT -> hypothesis DEAD, the "
                           "scored grid is not the published costmap")
            else:
                verdict = ("FLOOD FROM SEED DOES NOT REACH ROBOT -> unknown cells "
                           "block the wavefront (hypothesis CONFIRMED)")
        else:
            d["D4_flood"] = None
            visited = None
        d["D4_verdict"] = verdict

        # D5 -- cost_cloud coverage
        d["D5_cost_cloud"] = self.cost_cloud_coverage(robot_cell, seed_cell)

        # D6 -- full grid dump
        np.save(self.npy_path, grid.cost)
        d["D6_costgrid_npy"] = self.npy_path
        png = self.write_png(grid, robot_cell, seed_cell, visited)
        d["D6_costgrid_png"] = png

        # D7 -- critic rejection breakdown off /rosout
        if self.rejections:
            d["D7_rejections"] = {
                "by_gen_id": {str(k): v for k, v in sorted(self.rejections.items())},
                "costs_by_gen_id": {str(k): dict(v)
                                    for k, v in sorted(self.rejection_costs.items())},
                "scoring_cycles": self.eval_cycles,
                "cycles_with_zero_valid": self.eval_zero_valid,
            }
        else:
            d["D7_rejections"] = None
            d["notes"].append(
                "no DWA rejection lines seen on /rosout -- set the loggers to DEBUG: "
                "rosservice call /move_base/set_logger_level ros.base_local_planner debug")

        # D8/D9 -- cloud range bins
        d["D8_filtered_cloud_bins"] = range_bins(self.filtered_cloud)
        if d["D8_filtered_cloud_bins"] is None:
            d["notes"].append("no message on %s" % a.filtered_cloud_topic)
        d["D9_raw_cloud_bins"] = range_bins(self.raw_cloud)
        if d["D9_raw_cloud_bins"] is None:
            d["notes"].append("no message on %s" % a.raw_cloud_topic)

        # D10 -- robot z vs terrain z, plus attitude
        d["D10_height"] = {
            "robot_z": rz,
            "terrain_z": self.ground_z,
            "z_above_terrain": None if self.ground_z is None else rz - self.ground_z,
            "roll_rad": roll, "pitch_rad": pitch,
            "roll_deg": math.degrees(roll), "pitch_deg": math.degrees(pitch),
        }
        if self.ground_z is None:
            d["notes"].append("no message on /odometry/ground_height_odom -- "
                              "terrain_z unavailable (is publish_ground_height_odom.py up?)")

        # D11 -- lethal proximity
        d["D11_lethal"] = nearest_lethal(grid, robot_cell) if robot_cell else None

        with open(self.dump_path, "w") as fh:
            json.dump(d, fh, indent=2, sort_keys=False)
        self.print_summary(d)

    def cost_cloud_coverage(self, robot_cell, seed_cell):
        """D5 -- which grid cells appear in /move_base/DWAPlannerROS/cost_cloud.

        getCellCosts() returns false for blocked/unreachable cells, so those
        points are DROPPED from the cloud entirely: the HOLE is the signal, not
        a 40001 value. An absent topic is expected unless move_base was restarted
        with publish_cost_grid_pc in the DWAPlannerROS block.
        """
        grid = self.grid
        if self.cost_cloud is None:
            return {"available": False,
                    "reason": ("no message on %s -- publish_cost_grid_pc must be set "
                               "in the DWAPlannerROS block of config/planner_gps.yaml "
                               "and move_base RESTARTED (it is read at construction, "
                               "not settable via dynamic_reconfigure)"
                               % self.args.cost_cloud_topic)}
        h, w = grid.cost.shape
        present = np.zeros((h, w), dtype=bool)
        for x, y, _z in point_cloud2.read_points(
                self.cost_cloud, field_names=("x", "y", "z"), skip_nans=True):
            cell = grid.world_to_map(x, y)
            if cell is not None:
                present[cell[1], cell[0]] = True
        out = {"available": True,
               "cells_present": int(np.count_nonzero(present)),
               "cells_missing": int(present.size - np.count_nonzero(present)),
               "total_cells": int(present.size)}
        if robot_cell and seed_cell:
            mask = corridor_cells(grid, seed_cell, robot_cell,
                                  self.args.corridor_half_width)
            out["corridor_cells"] = int(np.count_nonzero(mask))
            out["corridor_cells_present"] = int(np.count_nonzero(present & mask))
            out["corridor_cells_missing"] = int(np.count_nonzero(~present & mask))
            out["drive_corridor_is_missing"] = bool(out["corridor_cells_missing"] > 0)
        return out

    def write_png(self, grid, robot_cell, seed_cell, visited):
        """Optional visualisation: cost grid with robot, seed and flood extent."""
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except Exception as exc:
            rospy.logwarn("[dwa_probe] no matplotlib, skipping PNG: %s" % exc)
            return None
        fig, ax = plt.subplots(figsize=(7, 7))
        ax.imshow(grid.cost, origin="lower", cmap="viridis", vmin=0, vmax=255)
        if visited is not None:
            ax.contour(visited.astype(float), levels=[0.5], colors="white",
                       linewidths=0.8)
        if robot_cell:
            ax.plot(robot_cell[0], robot_cell[1], "r+", markersize=14,
                    label="robot")
        if seed_cell:
            ax.plot(seed_cell[0], seed_cell[1], "wo", markersize=8,
                    markerfacecolor="none", label="seed")
        if robot_cell and seed_cell:
            ax.plot([seed_cell[0], robot_cell[0]], [seed_cell[1], robot_cell[1]],
                    "r--", linewidth=0.8, label="corridor")
        ax.legend(loc="upper right", fontsize=8)
        ax.set_title("local costmap at stall (white outline = flood from seed)")
        fig.savefig(self.png_path, dpi=130, bbox_inches="tight")
        plt.close(fig)
        return self.png_path

    def print_summary(self, d):
        p = sys.stdout.write
        p("\n" + "=" * 78 + "\n")
        p("DWA STALL LATCHED  %s\n" % d["wall_time"])
        p("=" * 78 + "\n")
        p("D4 VERDICT: %s\n\n" % d["D4_verdict"])
        r = d["robot"]
        p("robot        (%.2f, %.2f, %.2f) in %s, cell (%s, %s)\n"
          % (r["x"], r["y"], r["z"], r["frame"], r["map_x"], r["map_y"]))
        if d["goal"]:
            p("goal         (%.2f, %.2f) in %s   dist %s   move_base status %s\n"
              % (d["goal"]["x"], d["goal"]["y"], d["goal"]["frame"],
                 "n/a" if d["dist_to_goal"] is None else "%.2f m" % d["dist_to_goal"],
                 d["move_base_status"]))
        s = d["D1_seed"]
        if s["seed_found"]:
            p("D1 seed      cell (%d, %d)  world (%.2f, %.2f)  dense idx %d/%d  "
              "in-bounds %d  on-unknown %d\n"
              % (s["seed_map_x"], s["seed_map_y"], s["seed_world_x"], s["seed_world_y"],
                 s["seed_index_in_dense_plan"], s["dense_plan_len"],
                 s["plan_poses_in_bounds"], s["plan_poses_on_unknown"]))
        else:
            p("D1 seed      NONE FOUND (raw plan len %d)\n" % s["raw_plan_len"])
        g = d["grid_histogram"]
        p("grid         unknown %d  lethal %d  inscribed %d  free %d  other %d  "
          "(of %d)\n" % (g["no_information_255"], g["lethal_254"],
                         g["inscribed_253"], g["free_0"], g["other"], g["total"]))
        c = d["D2_corridor_histogram"]
        if c:
            p("D2 corridor  unknown %d  lethal %d  inscribed %d  free %d  (of %d)\n"
              % (c["no_information_255"], c["lethal_254"], c["inscribed_253"],
                 c["free_0"], c["total"]))
        an = d["D3_annulus_histogram"]
        if an:
            p("D3 annulus   %.1f-%.2f m: unknown %d / %d cells\n"
              % (an["r_inner_m"], an["r_outer_m"], an["no_information_255"],
                 an["total"]))
        f = d["D4_flood"]
        if f:
            p("D4 flood     reached_robot=%s  visited %d cells  bbox %s\n"
              % (f["reached_robot"], f["cells_visited"], f["bbox"]))
        cc = d["D5_cost_cloud"]
        if cc.get("available"):
            p("D5 cost_cloud present %d / %d  (corridor missing %s)\n"
              % (cc["cells_present"], cc["total_cells"],
                 cc.get("corridor_cells_missing")))
        else:
            p("D5 cost_cloud UNAVAILABLE: %s\n" % cc["reason"])
        if d["D7_rejections"]:
            p("D7 rejects   by gen_id %s  (0-valid cycles %d/%d)\n"
              % (d["D7_rejections"]["by_gen_id"],
                 d["D7_rejections"]["cycles_with_zero_valid"],
                 d["D7_rejections"]["scoring_cycles"]))
        p("D8 filtered  %s\n" % d["D8_filtered_cloud_bins"])
        p("D9 raw       %s\n" % d["D9_raw_cloud_bins"])
        h = d["D10_height"]
        p("D10 height   robot_z %.3f  terrain_z %s  roll %.2f deg  pitch %.2f deg\n"
          % (h["robot_z"], "n/a" if h["terrain_z"] is None else "%.3f" % h["terrain_z"],
             h["roll_deg"], h["pitch_deg"]))
        if d["D11_lethal"]:
            p("D11 lethal   count %d  inscribed %d  nearest %s\n"
              % (d["D11_lethal"]["lethal_count"], d["D11_lethal"]["inscribed_count"],
                 "none" if d["D11_lethal"]["nearest_lethal_m"] is None
                 else "%.2f m" % d["D11_lethal"]["nearest_lethal_m"]))
        for n in d["notes"]:
            p("NOTE: %s\n" % n)
        p("\nwrote %s\n      %s\n" % (self.dump_path, self.npy_path))
        if d.get("D6_costgrid_png"):
            p("      %s\n" % d["D6_costgrid_png"])
        p("=" * 78 + "\n\n")
        sys.stdout.flush()

    def close(self):
        try:
            self.csv.close()
        except Exception:
            pass


def parse_args(argv):
    repo = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--outdir", default=os.path.join(repo, "artifacts", "dwa_stall"))
    ap.add_argument("--rate", type=float, default=2.0,
                    help="time-series sample rate, Hz (default 2)")
    # stall latch
    ap.add_argument("--lin-thresh", type=float, default=0.01)
    ap.add_argument("--ang-thresh", type=float, default=0.01)
    ap.add_argument("--stall-hold-sec", type=float, default=2.0)
    ap.add_argument("--min-goal-dist", type=float, default=5.0,
                    help="only latch while further than this from the goal")
    ap.add_argument("--cmd-stale-sec", type=float, default=1.0,
                    help="cmd_vel older than this counts as zero velocity")
    ap.add_argument("--exit-after-dump", action="store_true")
    # analysis geometry
    ap.add_argument("--corridor-half-width", type=float, default=1.0)
    ap.add_argument("--annulus-inner", type=float, default=RAYTRACE_RANGE)
    ap.add_argument("--annulus-outer", type=float, default=WINDOW_CORNER_RANGE)
    ap.add_argument("--flood-every-tick", action="store_true",
                    help="compute D4 on every time-series tick (slower)")
    # topics / frames
    ap.add_argument("--costmap-topic", default="/move_base/local_costmap/costmap")
    ap.add_argument("--plan-topic", default="/move_base/DWAPlannerROS/global_plan")
    ap.add_argument("--plan-fallback-topic", default="/move_base/NavfnROS/plan")
    ap.add_argument("--cmd-vel-topic", default="/cmd_vel")
    ap.add_argument("--raw-cloud-topic", default="/os0_cloud_node/points")
    ap.add_argument("--filtered-cloud-topic",
                    default="/os0_cloud_node/points_above_terrain")
    ap.add_argument("--cost-cloud-topic", default="/move_base/DWAPlannerROS/cost_cloud")
    ap.add_argument("--base-frame", default="base_link")
    return ap.parse_args(rospy.myargv(argv)[1:])


def main(argv=None):
    args = parse_args(sys.argv if argv is None else argv)
    rospy.init_node("dwa_stall_probe", anonymous=True)
    probe = StallProbe(args)
    rospy.on_shutdown(probe.close)
    rospy.loginfo("[dwa_probe] time-series -> %s", probe.ts_path)
    rospy.Timer(rospy.Duration(1.0 / max(0.1, args.rate)), probe.tick)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
