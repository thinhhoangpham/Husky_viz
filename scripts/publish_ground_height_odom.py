#!/usr/bin/env python3
"""Publish the robot's ABSOLUTE height as a nav_msgs/Odometry the EKFs fuse for z.

    z = dtm_elevation_at(robot x, y) + clearance

WHY THIS EXISTS
---------------
The odom EKF used to run `two_d_mode: true`, which forced z/roll/pitch to zero.
With that off (so the robot's estimate can reflect real terrain), NOTHING in the
filter observes absolute z: the wheel odometry's vz is structurally zero on a
skid-steer, and no other input carries height. An unobserved state in an EKF is an
unconstrained random walk, so turning two_d_mode off WITHOUT this node would make z
drift without bound. This node supplies the missing measurement.

WHY THE CLEARANCE IS A CONSTANT, NOT A MEASUREMENT
--------------------------------------------------
This node used to MEASURE its clearance above ground by fitting a plane to the low
lidar returns in a 1.5-8 m annulus. That was wrong in principle: the clearance is a
fixed geometric property of the chassis, and the fit's error scaled with terrain
slope. On a slope the annulus spans ground that rises on one side and falls on the
other; the fit keys on the LOWEST returns, so it latched onto the downhill part of
the ring -- ground genuinely lower than what is under the wheels -- and inferred too
much clearance.

Measured, same robot, same run:

    parked on gentle ground   clearance read ~0.08 m above terrain
    driven onto a slope       clearance read ~0.41 m above terrain   (0.33 m error)

That error propagated: the fused z came out ~0.41 m too high, so every ground return
appeared 0.41 m above the DTM. scripts/filter_cloud_above_terrain.py marks 0.40..3.00 m
above terrain as an obstacle, so the ground landed exactly on the lower bound and 79%
of returns were marked -- 24816 lethal cells in the local costmap starting 0.51 m from
the robot, only 7% of which was the real tree. The rest was false ground.

WHERE THE CONSTANT COMES FROM -- and how to change it for another robot
-----------------------------------------------------------------------
From TF, at startup, as `-(base_link -> base_footprint).z`.

`base_footprint` is by definition the ground-projection frame: the URDF places it at
the point directly beneath base_link where the wheels touch the ground. On this Husky
(husky.urdf.xacro:105-109) that joint is

    origin z = wheel_vertical_offset - wheel_radius = 0.03282 - 0.1651 = -0.13228

so the lookup yields clearance = 0.13228 m. That agrees with the ~0.13 m a Husky's
true clearance is, and with the ~0.08 m the old fit measured when parked on flat
ground.

Using base_footprint rather than reconstructing the number from an axle height plus a
wheel radius is deliberate. The radius is a collision/visual property, not a
transform, so it is not in TF at all -- sourcing it would mean parsing
/robot_description or carrying a 0.1651 default that silently goes stale on a
different chassis. base_footprint needs neither: it is a standard REP-120 frame that
any mobile base publishes, it already encodes exactly the quantity we want, and it
follows automatically if the wheels or ride height change.

FOR A ROBOT WITH NO base_footprint: pass `_clearance:=<metres>` and the TF lookup is
skipped entirely. That param also overrides the lookup for testing.

THE FRAME MATTERS -- ~frame_id, and why the wrong one silently corrupts the map EKF
-----------------------------------------------------------------------------------
What we publish is an ABSOLUTE elevation (DTM terrain height + clearance), not an
odom-relative one. The `~frame_id` param says so; it must match the frame the number
is actually expressed in.

This is not cosmetic, because of how robot_localization consumes a pose measurement.
In RosFilter::preparePose it looks up `world_frame <- msg.header.frame_id` from tf and
applies that transform to the measurement before fusing (ros_filter.cpp: the
`poseTmp.mult(targetFrameTrans, poseTmp)` step). So:

  * stamped "odom", read by the ODOM filter (world_frame: odom)
        transform is odom->odom = identity. Harmless no-op.
  * stamped "odom", read by the MAP filter (world_frame: map)
        transform is map->odom -- THE MAP FILTER'S OWN OUTPUT. The filter applies its
        own estimate to its own input, so its output is added back to its measurement
        every cycle and settles at a nonzero offset. Measured: map->odom z drifted to
        +1.228 m, putting the robot 1.23 m above where it stood.
  * stamped "map" (correct for this quantity)
        the MAP filter's transform is map->map = identity, so no feedback. The ODOM
        filter's transform is odom->map, whose z is -(map->odom).z -- and the map
        filter now drives map->odom to ~0, so that correction vanishes. Both filters
        converge on the same true height.

Hence: run this with `_frame_id:=map` and fuse it in BOTH filters. Fusing it in
NEITHER is not an alternative -- the map filter does not pass z through, it estimates
it, so with no z input it estimates 0 and emits map->odom z = -(odom->base_link).z,
cancelling the odom filter's correct height and putting the robot back underground.

OFF-DTM BEHAVIOUR
-----------------
The DTM is now the ONLY terrain source; there is no fit to fall back on. When the
robot is over a cell the DTM does not cover -- off the mesh, or a no-data cell such as
open water -- this node PUBLISHES NOTHING for that scan and warns (throttled). It does
not hold the last good z and it does not substitute 0.

Holding the last good z was rejected: a stale height fused as a fresh absolute
measurement is indistinguishable from a real one to the EKF, so the longer the robot
stays off-DTM the more confidently wrong its altitude becomes -- exactly the silent
fabrication this node exists to avoid. Publishing nothing degrades honestly instead:
z simply goes unobserved again and the EKF's own covariance grows to say so. Note the
robot should not be off-DTM during normal operation anyway -- the planner is already
constrained to the terrain mask.

No Gazebo ground truth is used anywhere: only the robot's own URDF-derived TF, its
filtered x/y, and the offline DTM.
"""
import sys


def dtm_elevation_at(dtm_z, resolution, origin_x, origin_y, x, y):
    """Terrain elevation from a DTM array at world (x, y); None if outside the
    grid or that cell has no data (NaN). Row 0 = lowest y, matching DtmGrid.

    np.floor, not int() truncation: truncation rounds toward zero, so a query in
    the sub-cell strip just BELOW the origin gets index 0 instead of -1 and is
    silently answered with cell 0's terrain instead of being rejected as
    off-grid. Measured on the lake DTM (origin -49.75, -25.0): x = -49.85,
    y = -20.0 returned 4.130 m under truncation and correctly returns None under
    floor. Only the strip within one cell (0.25 m) outside the west/south edge
    differs -- inside the grid both agree -- but answering an off-grid point with
    a wrong elevation is exactly the silent failure this must not do. Same fix as
    scripts/relay_path_z.py.
    """
    import numpy as np
    if not (np.isfinite(x) and np.isfinite(y)):
        return None
    col = int(np.floor((x - origin_x) / resolution))
    row = int(np.floor((y - origin_y) / resolution))
    if row < 0 or col < 0 or row >= dtm_z.shape[0] or col >= dtm_z.shape[1]:
        return None
    v = dtm_z[row, col]
    if not np.isfinite(v):
        return None
    return float(v)


def base_link_z(ground_elev, clearance):
    """base_link's ABSOLUTE z in the map frame: terrain elevation plus the
    chassis's fixed clearance above the ground it stands on.

    `ground_elev` is None when the robot is over a cell the DTM does not cover,
    and this returns None so the caller publishes nothing. See OFF-DTM BEHAVIOUR
    in the module docstring: there is no honest height to report there, and a
    fabricated one is fused as gospel.
    """
    if ground_elev is None:
        return None
    return ground_elev + clearance


def clearance_from_footprint_tf(dz):
    """Chassis clearance from the (base_link -> base_footprint) translation z.

    base_footprint sits ON the ground beneath base_link, so its z is NEGATIVE and
    the clearance is its magnitude. A non-negative dz means the frame is not the
    ground-projection frame this assumes (or the sign convention was flipped), so
    raise rather than publish a robot buried in its own terrain.
    """
    if not (dz < 0.0):
        raise ValueError(
            "base_link->base_footprint z = %.4f m, expected negative "
            "(base_footprint must lie below base_link, on the ground)" % dz)
    return -dz


def covariance_for_z(z_variance):
    """6x6 row-major covariance that makes ONLY z meaningful.

    The EKF is configured to fuse z from this message alone; the huge variances on
    the other five DOF are what stop it from also believing our (meaningless) x, y
    and orientation fields.
    """
    cov = [0.0] * 36
    cov[0] = 1e6        # x
    cov[7] = 1e6        # y
    cov[14] = z_variance
    cov[21] = 1e6       # roll
    cov[28] = 1e6       # pitch
    cov[35] = 1e6       # yaw
    return cov


def main(argv=None):
    import numpy as np
    import rospy
    import tf2_ros
    from nav_msgs.msg import Odometry

    rospy.init_node("ground_height_odom")
    rate_hz = rospy.get_param("~rate", 10.0)
    z_var = rospy.get_param("~z_variance", 0.05)
    dtm_path = rospy.get_param("~dtm_path", "")
    # Explicit override for a robot with no base_footprint frame, or for testing.
    # <= 0 means "not set": look it up from TF instead.
    clearance_param = float(rospy.get_param("~clearance", 0.0))
    # See THE FRAME MATTERS in the module docstring. Default "odom" preserves the
    # historical behaviour for any existing caller; the dual-EKF setup wants "map".
    frame_id = rospy.get_param("~frame_id", "odom")

    # The DTM is the ONLY terrain source now that the ground-plane fit is gone, so
    # without it there is no absolute elevation to publish at all.
    if not dtm_path:
        rospy.logfatal("[ground_height] ~dtm_path is required: it is the only "
                       "terrain source, so without it there is no absolute "
                       "elevation to publish")
        return 1
    import yaml
    dtm_z = np.load(dtm_path)
    meta = yaml.safe_load(open(dtm_path.replace(".npy", ".yaml")))
    dtm_res = float(meta["resolution"])
    dtm_ox = float(meta["origin_x"])
    dtm_oy = float(meta["origin_y"])
    rospy.loginfo("[ground_height] DTM loaded %s (%dx%d @ %.2fm)",
                  dtm_path, dtm_z.shape[1], dtm_z.shape[0], dtm_res)

    state = {"xy": None}

    def on_pose(msg):
        # x,y only -- used solely to index the DTM. We never read its z (that is
        # the quantity we are producing, so consuming it would be circular).
        state["xy"] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    rospy.Subscriber("/odometry/filtered_map", Odometry, on_pose, queue_size=5)
    pub = rospy.Publisher("/odometry/ground_height", Odometry, queue_size=5)

    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf)

    clearance = clearance_param if clearance_param > 0.0 else None
    if clearance is not None:
        rospy.loginfo("[ground_height] clearance = %.4f m (from ~clearance param)",
                      clearance)

    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        # Fixed chassis geometry: look it up once from TF rather than hardcoding a
        # wheel radius. See WHERE THE CONSTANT COMES FROM in the module docstring.
        if clearance is None:
            try:
                tr = tf_buf.lookup_transform("base_link", "base_footprint",
                                             rospy.Time(0), rospy.Duration(1.0))
            except Exception as exc:
                rospy.logwarn_throttle(
                    5.0, "[ground_height] waiting for base_link->base_footprint "
                    "(%s); pass _clearance:=<metres> if this robot has no "
                    "base_footprint frame", exc)
                rate.sleep()
                continue
            clearance = clearance_from_footprint_tf(tr.transform.translation.z)
            rospy.loginfo("[ground_height] clearance = %.4f m "
                          "(from TF base_link->base_footprint)", clearance)

        if state["xy"] is None:
            rospy.logwarn_throttle(
                5.0, "[ground_height] no pose on /odometry/filtered_map yet, "
                "cannot index the DTM")
            rate.sleep()
            continue

        x, y = state["xy"]
        ground_elev = dtm_elevation_at(dtm_z, dtm_res, dtm_ox, dtm_oy, x, y)
        z = base_link_z(ground_elev, clearance)
        if z is None:
            # Off the mesh or a no-data cell. Publish NOTHING rather than a stale
            # or fabricated height -- see OFF-DTM BEHAVIOUR in the docstring.
            rospy.logwarn_throttle(
                2.0, "[ground_height] robot at (%.2f, %.2f) is off the DTM "
                "(off-mesh or no-data cell): publishing nothing, z is "
                "unobserved until it returns to covered terrain", x, y)
            rate.sleep()
            continue

        od = Odometry()
        od.header.stamp = rospy.Time.now()
        od.header.frame_id = frame_id
        od.child_frame_id = "base_link"
        od.pose.pose.position.z = z
        od.pose.pose.orientation.w = 1.0
        od.pose.covariance = covariance_for_z(z_var)
        pub.publish(od)
        rospy.loginfo_throttle(
            1.0, "[ground_height] z=%.3f m (terrain %.3f + clearance %.4f) "
            "at (%.2f, %.2f)" % (z, ground_elev, clearance, x, y))
        rate.sleep()
    return 0


if __name__ == "__main__":
    sys.exit(main())
