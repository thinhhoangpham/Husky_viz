#!/usr/bin/env python3
"""Republish plans, goals and markers draped over the terrain, for RViz.

DISPLAY ONLY. Every handler here subscribes to a topic move_base owns and
publishes a COPY to a new topic. The originals are never written, never
remapped, and move_base / NavfnROS / DWAPlannerROS keep consuming exactly the
bytes they always did. Nothing in this node can change a plan or a goal.

WHY THIS NODE EXISTS
--------------------
The nav stack is 2-D. NavfnROS, DWAPlannerROS and move_base's goal echo all
emit poses with `position.z` hardcoded to 0, because a 2-D planner has no
concept of height. Once the robot was put at true altitude the terrain spans
3.505..5.927 m in the lake world, so every one of those layers renders about
four metres UNDERGROUND -- the route line, the local plan and the goal arrow
all disappear beneath the DTM point cloud.

Measured on the live system before this node:
    /move_base/NavfnROS/plan            z = 0
    /move_base/DWAPlannerROS/local_plan z = 0
    /move_base/current_goal             z = 0

WHY A SIBLING OF relay_costmap_z.py AND NOT AN EXTENSION OF IT
--------------------------------------------------------------
The two nodes look similar but do genuinely different geometry, and merging
them would make both harder to read.

An OccupancyGrid is ONE FLAT SHEET. It has a single origin and no per-cell
height, so it can only ever sit at a single z, and relay_costmap_z.py spends
its length justifying which single z is least wrong (the minimum over the
window, so the sheet never pokes through the ground). That reasoning is a
compromise forced by the message type.

A Path, a PoseStamped and a MarkerArray are LISTS OF POINTS. Each point can be
given the terrain height at ITS OWN (x, y), so the route DRAPES over the
relief instead of lying on one plane. That is strictly better than anything
the costmap relay can do, and it is a different policy, not a parameter of the
same one. Folding both into one node would mean carrying two incompatible
sampling rules plus a message-type switch, and would leave the costmap
script's careful single-plane argument sitting next to code that does not
obey it.

So: one sibling node, three message types, one sampling rule -- per-point.

FRAMES: WHY EACH POSE IS TRANSFORMED BEFORE SAMPLING
-----------------------------------------------------
The DTM is expressed in `map`. The plans are NOT all in `map`:
    /move_base/NavfnROS/plan             map   (costmap_global_gps.yaml)
    /move_base/DWAPlannerROS/local_plan  odom  (costmap_local_gps.yaml)
Sampling an odom-frame x,y straight into a map-frame DTM would read the wrong
terrain by exactly the map->odom offset. So each message is transformed into
`map` to find the height, and the height is then converted BACK into the
message's own frame before being written, so the output stays in the frame its
header advertises and RViz needs no special handling.

The transform is looked up ONCE per message, not once per pose: a plan is a
single instant, and using a different transform for different points of the
same path would bend it.

WHY tf + THE OFFLINE DTM, AND NOT THE ROBOT'S FUSED z
------------------------------------------------------
Same reason as relay_costmap_z.py. The fused z is an ESTIMATE; under a spoof or
drift it moves with the corrupted pose and the display would follow it,
rendering the corruption self-consistent and invisible. This repo exists to
make pose corruption visible, so the display height is anchored to the offline
map. (The map->odom lookup needed to place an odom-frame plan is unavoidable --
that is a frame relationship, not a height source.)

OFF-DTM AND NaN POINTS
----------------------
A DTM cell is NaN where no terrain mesh covered it, and a plan can run past the
edge of the grid entirely. NaN is "no terrain here", which is NOT "height
zero" -- folding it to zero would drop that point four metres underground and
put a spike in the line.

The choice: a point with no terrain data KEEPS THE LAST VALID HEIGHT along the
path (and, for the first points, the first valid height found anywhere in the
message). A short off-mesh excursion therefore renders as a flat bridge at the
height of the ground it left, which reads correctly as "unknown ground here"
and keeps the line continuous. If a message has NO valid sample at all, it is
published UNCHANGED rather than dropped, so a plan is never silently missing
from RViz -- it just renders at z = 0 as it did before this node existed, and
the node says so once every five seconds.

MARKERS
-------
`~marker_topic` is provided for completeness but is OFF by default and is not
needed for /landmark_observed_markers. Those markers are published by
landmark_loc/localizer_node.py in the LIDAR's own frame with
z = cluster_top + 0.5, so RViz already transforms them into 3-D at true
altitude through tf. They are correct as they stand and must not be relayed --
doing so would add terrain height to a height that already includes it.

Usage (no launch file; see RUN-MAP-NAV.md):

    python3 scripts/relay_path_z.py _world:=lake

That single invocation relays all three broken topics at once. Params (all
private, all optional):
    ~world          park | lake   (default park) -- picks maps/<world>_dtm.npy
    ~dtm_path       explicit .npy path, overrides ~world
    ~z_offset       metres added above the terrain, default 0.15, so the line
                    sits just clear of the ground instead of z-fighting it
    ~path_topics    comma-separated nav_msgs/Path inputs
    ~pose_topics    comma-separated geometry_msgs/PoseStamped inputs
    ~marker_topics  comma-separated visualization_msgs/MarkerArray inputs
                    (empty by default -- see MARKERS above)
    ~out_suffix     appended to each input topic name, default _z
    ~map_frame      frame the DTM is expressed in, default map
    ~tf_timeout     seconds to wait for a transform, default 0.1
"""
import os
import sys
from copy import deepcopy

import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

DEFAULT_PATH_TOPICS = ("/move_base/NavfnROS/plan,"
                       "/move_base/DWAPlannerROS/local_plan")
DEFAULT_POSE_TOPICS = "/move_base/current_goal"
DEFAULT_MARKER_TOPICS = ""
DEFAULT_Z_OFFSET = 0.15


def resolve_dtm_path(world, dtm_path=None, maps_dir=None):
    """Absolute path to the DTM .npy to read. Explicit `dtm_path` wins."""
    if dtm_path:
        return os.path.abspath(os.path.expanduser(dtm_path))
    if maps_dir is None:
        maps_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "maps")
    return os.path.abspath(os.path.join(maps_dir, "%s_dtm.npy" % world))


def load_dtm_meta(npy_path):
    """(resolution, origin_x, origin_y) from the .yaml beside `npy_path`.

    The .npy carries no geometry of its own. Parsed line-by-line rather than
    with PyYAML, matching scripts/relay_costmap_z.py and
    scripts/filter_cloud_above_terrain.py: the generated file is strictly
    `key: value`.
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


def dtm_height_at(heights, resolution, origin_x, origin_y, x, y):
    """Terrain height at world (x, y), or None where there is no terrain.

    `heights` is maps/<world>_dtm.npy: [row][col], row 0 = LOWEST y, NaN = no
    mesh coverage (NOT a height of zero).

    None means "no terrain data at this point" -- off the grid, a NaN cell, or
    a non-finite input coordinate. That is a different statement from "height
    zero" and is left for the caller to act on; see OFF-DTM in the module
    docstring.

    np.floor, not int() truncation: truncation rounds toward zero, which on the
    negative side of the map folds the cell below the origin onto cell 0 and
    reads the wrong terrain. The lake DTM's origin is (-49.75, -25.0), so every
    plan in the western half of that world hits this.
    """
    z = np.asarray(heights)
    if z.ndim != 2 or z.size == 0:
        return None
    if not (np.isfinite(x) and np.isfinite(y)):
        return None
    col = int(np.floor((x - origin_x) / resolution))
    row = int(np.floor((y - origin_y) / resolution))
    if row < 0 or col < 0 or row >= z.shape[0] or col >= z.shape[1]:
        return None
    v = z[row, col]
    if not np.isfinite(v):
        return None
    return float(v)


def drape_heights(xys, sample, z_offset=0.0):
    """Per-point display heights for the world-frame points `xys`.

    `sample` is a callable (x, y) -> height or None. Returns a list the same
    length as `xys`, or None when NOT ONE point had terrain data -- which the
    caller treats as "publish the message unchanged" rather than inventing
    heights.

    Gaps (off-DTM / NaN cells) inherit the LAST valid height, and any leading
    gap inherits the FIRST valid height found. So an off-mesh excursion renders
    as a flat bridge at the level of the ground the path left, keeping the line
    continuous instead of spiking it to zero. `z_offset` lifts the whole result
    clear of the ground so the line does not z-fight the terrain cloud.
    """
    raw = [sample(x, y) for x, y in xys]
    first = next((v for v in raw if v is not None), None)
    if first is None:
        return None

    out = []
    last = first
    for v in raw:
        if v is not None:
            last = v
        out.append(last + z_offset)
    return out


def _apply(points, heights):
    """Write `heights` into the `position.z` of each point in `points`."""
    for p, z in zip(points, heights):
        p.z = z


def drape_path(msg, sample, z_offset=0.0):
    """A copy of nav_msgs/Path `msg` with each pose sitting on the terrain.

    The input is never mutated: it is shared with any other subscriber in this
    process and with rospy's own buffers. Returns the original object when
    there is no terrain data anywhere along it (see `drape_heights`), so the
    caller can publish it unchanged.

    `sample` takes coordinates already in the DTM's frame -- transforming into
    that frame is the caller's job, because the transform is looked up once per
    MESSAGE, not once per pose.
    """
    if not msg.poses:
        return msg
    heights = drape_heights(
        [(p.pose.position.x, p.pose.position.y) for p in msg.poses],
        sample, z_offset)
    if heights is None:
        return msg
    out = deepcopy(msg)
    _apply([p.pose.position for p in out.poses], heights)
    return out


def drape_pose(msg, sample, z_offset=0.0):
    """A copy of geometry_msgs/PoseStamped `msg` sitting on the terrain.

    Same contract as `drape_path`: never mutates the input, returns the
    original when the point has no terrain data under it.
    """
    heights = drape_heights(
        [(msg.pose.position.x, msg.pose.position.y)], sample, z_offset)
    if heights is None:
        return msg
    out = deepcopy(msg)
    out.pose.position.z = heights[0]
    return out


def drape_marker_array(msg, sample, z_offset=0.0):
    """A copy of visualization_msgs/MarkerArray with each marker on the terrain.

    Each marker's own pose is draped, and any marker carrying a `points` list
    (LINE_STRIP, POINTS, ...) has every entry of that list draped too, since
    those are what actually get drawn for those types.

    A marker whose pose has no terrain under it keeps its original z rather
    than inheriting a neighbour's: markers are independent objects, not a
    connected line, so the continuity argument that justifies bridging gaps in
    a Path does not apply here.
    """
    out = deepcopy(msg)
    for m in out.markers:
        heights = drape_heights(
            [(m.pose.position.x, m.pose.position.y)], sample, z_offset)
        if heights is not None:
            m.pose.position.z = heights[0]
        if getattr(m, "points", None):
            pt_heights = drape_heights(
                [(p.x, p.y) for p in m.points], sample, z_offset)
            if pt_heights is not None:
                _apply(m.points, pt_heights)
    return out


def out_topic_for(in_topic, suffix="_z"):
    """The republish topic name for `in_topic`. Never equal to the input."""
    if not suffix:
        raise ValueError(
            "~out_suffix must not be empty -- an empty suffix would make the "
            "output topic identical to the input, and republishing onto a "
            "topic move_base consumes is exactly what this node must never do.")
    return in_topic + suffix


def split_topics(raw):
    """Parse a comma-separated topic list, dropping blanks and whitespace."""
    return [t.strip() for t in (raw or "").split(",") if t.strip()]


def main(argv=None):
    argv = sys.argv if argv is None else argv
    rospy.init_node("relay_path_z", anonymous=True)

    world = rospy.get_param("~world", "park")
    dtm_path_param = rospy.get_param("~dtm_path", "")
    z_offset = float(rospy.get_param("~z_offset", DEFAULT_Z_OFFSET))
    suffix = rospy.get_param("~out_suffix", "_z")
    map_frame = rospy.get_param("~map_frame", "map")
    tf_timeout = float(rospy.get_param("~tf_timeout", 0.1))

    path_topics = split_topics(
        rospy.get_param("~path_topics", DEFAULT_PATH_TOPICS))
    pose_topics = split_topics(
        rospy.get_param("~pose_topics", DEFAULT_POSE_TOPICS))
    marker_topics = split_topics(
        rospy.get_param("~marker_topics", DEFAULT_MARKER_TOPICS))

    try:
        outs = {t: out_topic_for(t, suffix)
                for t in path_topics + pose_topics + marker_topics}
    except ValueError as exc:
        rospy.logfatal("[relay_path_z] %s", exc)
        return 1
    # Belt and braces alongside the empty-suffix check above: whatever the
    # suffix, an output must never collide with ANY input we subscribe to.
    clash = set(outs.values()) & set(outs)
    if clash:
        rospy.logfatal("[relay_path_z] output topic(s) %s collide with an "
                       "input -- refusing to publish onto a topic move_base "
                       "consumes.", sorted(clash))
        return 1

    path = resolve_dtm_path(world, dtm_path_param)
    if not os.path.exists(path):
        rospy.logfatal("[relay_path_z] no DTM at %s -- set ~world to a world "
                       "that has maps/<world>_dtm.npy, or pass ~dtm_path.",
                       path)
        return 1
    heights = np.load(path)
    resolution, origin_x, origin_y = load_dtm_meta(path)
    rospy.loginfo("[relay_path_z] terrain from %s (+%.2f m clearance)",
                  path, z_offset)

    import tf2_ros
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer)

    def sampler_for(frame_id, stamp):
        """A (x, y) -> height callable for points expressed in `frame_id`.

        Returns (sampler, dz) where `dz` converts a map-frame height back into
        `frame_id`, so the value written into the message stays in the frame
        its header advertises. Returns (None, None) when the transform is
        unavailable.

        The lookup happens HERE, once per message -- not per pose -- so every
        point of one plan is placed with the same transform and the path
        cannot be bent by a mid-message tf update.
        """
        if frame_id == map_frame:
            return (lambda x, y: dtm_height_at(
                heights, resolution, origin_x, origin_y, x, y)), 0.0
        try:
            tf = tf_buffer.lookup_transform(
                map_frame, frame_id, stamp, rospy.Duration(tf_timeout))
        except Exception as exc:      # noqa: BLE001 - tf2 raises 4 types
            rospy.logwarn_throttle(
                5.0, "[relay_path_z] no %s->%s transform (%s); republishing "
                "unchanged", map_frame, frame_id, exc)
            return None, None
        t = tf.transform.translation
        r = tf.transform.rotation
        # Yaw-only planar transform. The plans are 2-D and the DTM is indexed
        # by x,y alone, so roll/pitch cannot change WHICH cell is sampled --
        # only a full 3-D rotation of the path would, and these messages have
        # no meaningful z to rotate. Yaw from the quaternion, no tf_conversions
        # dependency.
        yaw = np.arctan2(2.0 * (r.w * r.z + r.x * r.y),
                         1.0 - 2.0 * (r.y * r.y + r.z * r.z))
        cos_y, sin_y = np.cos(yaw), np.sin(yaw)

        def sample(x, y):
            mx = t.x + cos_y * x - sin_y * y
            my = t.y + sin_y * x + cos_y * y
            return dtm_height_at(heights, resolution, origin_x, origin_y,
                                 mx, my)

        # A map-frame height of h is h - t.z in the child frame.
        return sample, -float(t.z)

    def make_handler(pub, drape, in_topic):
        def handler(msg):
            sample, dz = sampler_for(msg.header.frame_id, msg.header.stamp)
            if sample is None:
                pub.publish(msg)
                return
            pub.publish(drape(msg, sample, z_offset + dz))
        return handler

    subs = []
    for topics, msg_type, drape in (
            (path_topics, Path, drape_path),
            (pose_topics, PoseStamped, drape_pose)):
        for t in topics:
            pub = rospy.Publisher(outs[t], msg_type, queue_size=1, latch=True)
            subs.append(rospy.Subscriber(
                t, msg_type, make_handler(pub, drape, t), queue_size=1))
            rospy.loginfo("[relay_path_z] %s -> %s (display only; %s is not "
                          "modified)", t, outs[t], t)

    if marker_topics:
        # Imported lazily: the default configuration relays no markers, and the
        # pure-function tests must not need visualization_msgs present.
        from visualization_msgs.msg import MarkerArray
        for t in marker_topics:
            pub = rospy.Publisher(outs[t], MarkerArray, queue_size=1,
                                  latch=True)
            subs.append(rospy.Subscriber(
                t, MarkerArray, make_handler(pub, drape_marker_array, t),
                queue_size=1))
            rospy.loginfo("[relay_path_z] %s -> %s (display only)", t, outs[t])

    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
