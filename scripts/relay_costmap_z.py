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

Usage (no launch file; see RUN-MAP-NAV.md):

    python3 scripts/relay_costmap_z.py _world:=lake
    python3 scripts/relay_costmap_z.py _world:=park

Params (all private, all optional):
    ~world       park | lake        (default park) -- picks maps/<world>_dtm.npy
    ~dtm_path    explicit .npy path, overrides ~world
    ~in_topic    default /move_base/global_costmap/costmap
    ~out_topic   default /move_base/global_costmap/costmap_z
    ~z           explicit metres, overrides the DTM entirely (escape hatch)
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

    if out_topic == in_topic:
        rospy.logfatal("[relay_costmap_z] ~out_topic must differ from "
                       "~in_topic (%s) -- republishing onto the costmap "
                       "move_base consumes is exactly what this node must "
                       "never do.", in_topic)
        return 1

    if z_override is not None:
        z = float(z_override)
        rospy.loginfo("[relay_costmap_z] z = %.4f m (from ~z, DTM not read)", z)
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

    def on_costmap(msg):
        pub.publish(shift_grid_z(msg, z))

    rospy.Subscriber(in_topic, OccupancyGrid, on_costmap, queue_size=1)
    rospy.loginfo("[relay_costmap_z] %s -> %s (display only; %s is not "
                  "modified)", in_topic, out_topic, in_topic)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
