#!/usr/bin/env python3
"""Republish the global costmap at the terrain's elevation, for RViz.

DISPLAY ONLY. This node subscribes to the global costmap and publishes a copy
whose only difference is `info.origin.position.z`. It publishes to a NEW topic;
the original is never touched, never remapped, and move_base / NavfnROS keep
consuming exactly the bytes they always did. Nothing here can change a plan.

WHY THIS NODE EXISTS
--------------------
`costmap_2d` offers no way to set the published origin z. Costmap2DPublisher
tracks only `saved_origin_x_, saved_origin_y_` (costmap_2d_publisher.h) and
emits `origin.position.z` as a hardcoded 0. The `origin_z` parameter that does
exist belongs to VoxelLayer (voxel_layer.h, VoxelPluginConfig.h) and is the
base of a 3-D voxel COLUMN used for marking/clearing -- it is not a render
offset and does not reach the published OccupancyGrid. So the grid always
arrives at z = 0, and there is no configuration that changes that. A relay is
the only option.

That z = 0 became visible once the robot was put at true altitude (commit
07cd63c fused absolute z, so base_link now sits at ~4.2 m in the lake world).
The costmap stayed at 0 while the terrain it describes spans 3.505..5.927 m,
so RViz drew the terrain hovering several metres above a grey sheet.

THE HEIGHT CHOSEN, AND WHY IT IS AN APPROXIMATION
-------------------------------------------------
An OccupancyGrid is a FLAT plane. It has one origin and no per-cell height, so
it can only sit at a SINGLE z -- it physically cannot follow terrain relief.
Whatever we pick is therefore an approximation, and this is the honest
statement of it:

    z = the MINIMUM finite height in the world's DTM.

Rationale: at the minimum, the sheet lies at or below the terrain surface
EVERYWHERE, so it never pokes through the ground. A sheet that intersects
terrain reads as a rendering fault; one sitting cleanly beneath it reads
correctly as a projection of the ground onto a plane. The cost is vertical
error under high ground: in the lake world (relief 2.42 m) the sheet sits up to
~2.4 m below the highest terrain. In the park world (relief 0.007 m) the choice
is irrelevant -- the terrain is flat to within a centimetre.

Rejected alternatives, for the record: the MEDIAN height minimises average
error but makes the sheet visibly cut through every hill; the height UNDER THE
ROBOT agrees best where you are looking but would have to move as the robot
drives, and a display plane that slides up and down underfoot is disorienting.

THE LOCAL COSTMAP NEEDS A DIFFERENT ANSWER (~follow_robot)
----------------------------------------------------------
Everything above is correct for the GLOBAL costmap, which is static, in the
`map` frame, and covers the whole world. The LOCAL costmap is none of those:
it is `rolling_window: true`, 10x10 m, in the `odom` frame, and it TRAVELS
WITH THE ROBOT. Pinning it to the global minimum would be wrong nearly
everywhere -- as the robot climbs, its little patch would sink further and
further beneath the ground it is actually driving on.

So `~follow_robot:=true` recomputes z per message from the terrain NEAR THE
ROBOT: look the robot up in tf, then take the minimum DTM height over the
window footprint centred on it.

Why the minimum OVER THE WINDOW rather than the single cell under the robot:
the "never poke through" invariant still has to hold, and on the lake it is
not a formality. Measured on maps/lake_dtm.npy, a single 10x10 m window spans
up to 2.007 m of the map's 2.422 m total relief, and 79% of windows exceed
0.5 m (median 0.78 m). A sheet at the robot's own ground height would visibly
cut through the slope ahead of it across most of the map. Taking the window
minimum keeps the sheet under the terrain everywhere it is drawn while still
tracking the robot: mean gap under the robot drops from 1.033 m (global
minimum) to 0.430 m, worst case from 2.409 m to 1.371 m.

Why tf + DTM and NOT the robot's fused z (which, since 07cd63c, is a real
altitude): two reasons. The fused z is terrain PLUS clearance, so a sheet at
that height floats above the ground and pokes through it -- the exact fault
this node exists to avoid. And it would wire the pose ESTIMATE into the
display: under a spoof or drift the sheet would move with the corrupted pose
and make it look self-consistent. This repo's purpose is to make pose
corruption visible, so the display stays anchored to the offline map.

Usage (no launch file; see RUN-MAP-NAV.md):

    # global costmap (fixed z, the original behaviour)
    python3 scripts/relay_costmap_z.py _world:=lake

    # local costmap (z follows the robot)
    python3 scripts/relay_costmap_z.py _world:=lake _follow_robot:=true \
        _in_topic:=/move_base/local_costmap/costmap \
        _out_topic:=/move_base/local_costmap/costmap_z

Params (all private, all optional):
    ~world         park | lake      (default park) -- picks maps/<world>_dtm.npy
    ~dtm_path      explicit .npy path, overrides ~world
    ~in_topic      default /move_base/global_costmap/costmap
    ~out_topic     default /move_base/global_costmap/costmap_z
    ~z             explicit metres, overrides the DTM entirely (escape hatch)
    ~follow_robot  track the terrain under the robot, default False
    ~map_frame     frame the DTM is expressed in, default map
    ~base_frame    robot frame to look up, default base_link
    ~tf_timeout    seconds to wait for the transform, default 0.1
"""
import os
import sys
from copy import deepcopy

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid

DEFAULT_IN_TOPIC = "/move_base/global_costmap/costmap"
DEFAULT_OUT_TOPIC = "/move_base/global_costmap/costmap_z"


def dtm_min_z(heights):
    """The minimum FINITE height in a DTM array, as a float.

    NaN means "no mesh covered this cell" -- not a height of zero -- so NaNs
    must be excluded rather than treated as low ground, or the sheet would be
    dragged down to 0 by cells that have no terrain at all.

    Returns 0.0 for an all-NaN (or empty) grid: there is no terrain to sit on,
    so the only defensible fallback is the nav stack's own datum, which leaves
    the display exactly as it behaved before this node existed.
    """
    z = np.asarray(heights, dtype=np.float64)
    finite = z[np.isfinite(z)]
    if finite.size == 0:
        return 0.0
    return float(finite.min())


def window_min_z(heights, resolution, origin_x, origin_y, robot_x, robot_y,
                 window):
    """Lowest finite DTM height within a `window`-metre square around the robot.

    `heights` is the array from maps/<world>_dtm.npy: [row][col], row 0 =
    LOWEST y, NaN = no mesh coverage (NOT a height of zero). `origin_x/y` and
    `resolution` come from the sibling .yaml. `robot_x/robot_y` are in the same
    world frame as that origin.

    Returns a float, or None when there is no answer -- the window lies off the
    grid, covers only NaN, or the robot position itself is not finite. None is
    "no terrain data here", which is a different statement from "height zero"
    and is left for the caller to act on; folding it to 0.0 would drop the
    sheet several metres underground for a frame.

    The window is CLIPPED to the grid, never wrapped: negative indices would
    silently read terrain from the far side of the map.
    """
    z = np.asarray(heights)
    if z.ndim != 2 or z.size == 0:
        return None
    if not (np.isfinite(robot_x) and np.isfinite(robot_y)):
        return None

    n_rows, n_cols = z.shape
    half = float(window) / 2.0

    # np.floor, not int() truncation: truncation rounds toward zero, which on
    # the negative side of the map folds the cell below the origin onto cell 0
    # and reads the wrong terrain.
    c0 = int(np.floor((robot_x - half - origin_x) / resolution))
    c1 = int(np.floor((robot_x + half - origin_x) / resolution))
    r0 = int(np.floor((robot_y - half - origin_y) / resolution))
    r1 = int(np.floor((robot_y + half - origin_y) / resolution))

    c0, c1 = max(c0, 0), min(c1, n_cols - 1)
    r0, r1 = max(r0, 0), min(r1, n_rows - 1)
    if c0 > c1 or r0 > r1:
        return None

    win = z[r0:r1 + 1, c0:c1 + 1]
    finite = win[np.isfinite(win)]
    if finite.size == 0:
        return None
    return float(finite.min())


def grid_window_metres(msg):
    """The larger side of an OccupancyGrid's footprint, in metres.

    The relay reads the window size off the message rather than taking it as a
    parameter so it cannot drift out of sync with costmap_local_gps.yaml. The
    larger side is used so the sampled window always covers the whole sheet.
    """
    return max(msg.info.width, msg.info.height) * msg.info.resolution


def load_dtm_meta(npy_path):
    """(resolution, origin_x, origin_y) from the .yaml beside `npy_path`.

    The .npy carries no geometry of its own, so the sibling .yaml written by
    map_tools/extract_dtm.py is required to locate its cells in the world.

    Parsed line-by-line rather than with PyYAML, matching how
    scripts/filter_cloud_above_terrain.py and scripts/publish_dtm_cloud.py read
    the same generated file: it is strictly `key: value`.
    """
    yaml_path = os.path.splitext(npy_path)[0] + ".yaml"
    if not os.path.exists(yaml_path):
        raise IOError(
            "missing %s -- the DTM .npy carries no geometry of its own, so "
            "the .yaml written beside it by map_tools/extract_dtm.py is "
            "required to locate its cells." % yaml_path)
    needed = ("resolution", "origin_x", "origin_y")
    meta = {}
    with open(yaml_path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, _, val = line.partition(":")
            key = key.strip()
            if key in needed:
                try:
                    meta[key] = float(val.strip())
                except ValueError:
                    pass
    missing = [k for k in needed if k not in meta]
    if missing:
        raise ValueError("%s is missing %s" % (yaml_path, ", ".join(missing)))
    return meta["resolution"], meta["origin_x"], meta["origin_y"]


def resolve_dtm_path(world, dtm_path=None, maps_dir=None):
    """Absolute path to the DTM .npy to read.

    An explicit `dtm_path` wins outright. Otherwise the path is built from
    `world`, which is validated against the files actually present rather than
    a hardcoded list, so adding a world needs no edit here.
    """
    if dtm_path:
        return os.path.abspath(os.path.expanduser(dtm_path))
    if maps_dir is None:
        maps_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "maps")
    return os.path.abspath(os.path.join(maps_dir, "%s_dtm.npy" % world))


def shift_grid_z(msg, z):
    """A copy of OccupancyGrid `msg` with origin.position.z set to `z`.

    Everything else -- data, resolution, width, height, x/y origin,
    orientation, frame_id -- is carried over untouched. The input message is
    never mutated: it is shared with any other subscriber in this process and
    with rospy's own buffers.
    """
    out = OccupancyGrid()
    out.header = msg.header
    # info is DEEP-copied, never assigned by reference: mutating origin.position.z
    # through a shared reference would rewrite the costmap that move_base and
    # every other subscriber in this process see. data is large and is not
    # modified, so it is shared deliberately.
    out.info = deepcopy(msg.info)
    out.info.origin.position.z = z
    out.data = msg.data
    return out


def main(argv=None):
    argv = sys.argv if argv is None else argv
    rospy.init_node("relay_costmap_z", anonymous=True)

    world = rospy.get_param("~world", "park")
    dtm_path_param = rospy.get_param("~dtm_path", "")
    in_topic = rospy.get_param("~in_topic", DEFAULT_IN_TOPIC)
    out_topic = rospy.get_param("~out_topic", DEFAULT_OUT_TOPIC)
    z_override = rospy.get_param("~z", None)
    follow_robot = bool(rospy.get_param("~follow_robot", False))
    map_frame = rospy.get_param("~map_frame", "map")
    base_frame = rospy.get_param("~base_frame", "base_link")
    tf_timeout = float(rospy.get_param("~tf_timeout", 0.1))

    if out_topic == in_topic:
        rospy.logfatal("[relay_costmap_z] ~out_topic must differ from "
                       "~in_topic (%s) -- republishing onto the costmap "
                       "move_base consumes is exactly what this node must "
                       "never do.", in_topic)
        return 1

    # ~z short-circuits everything, in both modes: it is the escape hatch for
    # "I know the number, do not read anything".
    if z_override is not None:
        z = float(z_override)
        rospy.loginfo("[relay_costmap_z] z = %.4f m (from ~z, DTM not read)", z)
        if follow_robot:
            rospy.logwarn("[relay_costmap_z] ~z was given, so ~follow_robot is "
                          "IGNORED -- the sheet is pinned at %.4f m and will "
                          "not track the robot.", z)
    elif follow_robot:
        z = None  # recomputed per message below
    else:
        path = resolve_dtm_path(world, dtm_path_param)
        if not os.path.exists(path):
            rospy.logfatal("[relay_costmap_z] no DTM at %s -- set ~world to a "
                           "world that has maps/<world>_dtm.npy, or pass "
                           "~dtm_path / ~z explicitly.", path)
            return 1
        z = dtm_min_z(np.load(path))
        rospy.loginfo("[relay_costmap_z] z = %.4f m (minimum terrain height "
                      "in %s)", z, path)

    pub = rospy.Publisher(out_topic, OccupancyGrid, queue_size=1, latch=True)

    if z is not None:
        def on_costmap(msg):
            pub.publish(shift_grid_z(msg, z))
    else:
        # Follow-robot mode. Imported here, not at module scope, so the fixed-z
        # path -- and the pure-function tests -- never need tf2 present.
        import tf2_ros

        path = resolve_dtm_path(world, dtm_path_param)
        if not os.path.exists(path):
            rospy.logfatal("[relay_costmap_z] no DTM at %s -- set ~world to a "
                           "world that has maps/<world>_dtm.npy, or pass "
                           "~dtm_path / ~z explicitly.", path)
            return 1
        heights = np.load(path)
        resolution, origin_x, origin_y = load_dtm_meta(path)
        rospy.loginfo("[relay_costmap_z] follow_robot: z tracks the terrain "
                      "under %s, sampled from %s", base_frame, path)

        tf_buffer = tf2_ros.Buffer()
        tf2_ros.TransformListener(tf_buffer)

        # Held across messages so a momentary tf gap or an off-mesh excursion
        # holds the sheet where it was instead of snapping it to 0 -- a flat
        # display plane that flickers underground is worse than a stale one.
        state = {"z": None}

        def on_costmap(msg):
            try:
                tf = tf_buffer.lookup_transform(
                    map_frame, base_frame, rospy.Time(0),
                    rospy.Duration(tf_timeout))
            except Exception as exc:      # noqa: BLE001 - tf2 raises 4 types
                rospy.logwarn_throttle(
                    5.0, "[relay_costmap_z] no %s->%s transform (%s); holding "
                    "last z", map_frame, base_frame, exc)
                tf = None

            if tf is not None:
                sampled = window_min_z(
                    heights, resolution, origin_x, origin_y,
                    tf.transform.translation.x, tf.transform.translation.y,
                    grid_window_metres(msg))
                if sampled is None:
                    rospy.logwarn_throttle(
                        5.0, "[relay_costmap_z] robot is off the DTM; holding "
                        "last z")
                else:
                    state["z"] = sampled

            if state["z"] is None:
                # Nothing valid has ever been sampled. Publishing at 0 would
                # reproduce the very bug this node fixes, so publish nothing
                # and let the RViz display stay empty until tf comes up.
                return
            pub.publish(shift_grid_z(msg, state["z"]))

    rospy.Subscriber(in_topic, OccupancyGrid, on_costmap, queue_size=1)
    rospy.loginfo("[relay_costmap_z] %s -> %s (display only; %s is not "
                  "modified)", in_topic, out_topic, in_topic)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
