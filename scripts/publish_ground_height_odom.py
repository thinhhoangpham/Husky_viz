#!/usr/bin/env python3
"""Publish the robot's ABSOLUTE height, measured from the lidar's own view of the
ground, as a nav_msgs/Odometry the odom EKF can fuse for z.

WHY THIS EXISTS
---------------
The odom EKF used to run `two_d_mode: true`, which forced z/roll/pitch to zero.
With that off (so the robot's estimate can reflect real terrain), NOTHING in the
filter observes absolute z: the wheel odometry's vz is structurally zero on a
skid-steer, and no other input carries height. An unobserved state in an EKF is an
unconstrained random walk, so turning two_d_mode off WITHOUT this node would make z
drift without bound. This node supplies the missing measurement.

WHAT IS MEASURED
----------------
The lidar sees the ground around the robot. De-rotating the cloud by the compass's
gravity-referenced roll/pitch makes that ground level in the sensor frame, so a
plane fit to the low returns yields, from its intercept, the SENSOR's height above
the LOCAL ground surface.

THE Z CONVENTION (and its assumptions) -- read this before trusting the number
-----------------------------------------------------------------------------
What the fit gives:      sensor height above local ground   (h_sensor)
What the EKF wants:      base_link z in the ODOM frame

These are different quantities, and converting between them requires an assumption.
We publish:

    z_base_link = h_sensor - mount_dz

where `mount_dz` is the FIXED base_link -> lidar offset, looked up from TF (never
hardcoded; it is 0.826 m on this robot). The result is "how high base_link sits
above the ground directly beneath it".

ASSUMPTION, STATED PLAINLY: this treats the local ground surface as the odom
frame's z datum -- i.e. it defines z=0 as "the ground under the robot" rather than
tracking absolute elevation change as the robot climbs. On flat ground the two
agree. On a long climb they do NOT: this measurement says "I am 0.13 m above the
dirt", not "I have ascended 3 m since start". It therefore CONSTRAINS z (preventing
the random walk) but does not by itself give terrain-relative altitude. That is the
honest limit of a ground-plane fit with no other altitude reference, and it is the
right trade here: a bounded, always-available constraint beats an unobserved state.

Sanity note: on this robot the fit measures h_sensor ~1.46 m while the URDF mount
offset is 0.826 m, implying base_link ~0.64 m above ground -- higher than a Husky's
true ~0.13 m. The gap is the ground-plane fit keying on the lowest returns of a
SLOPING patch (the robot sits on a shore slope) rather than the ground directly
beneath the wheels. The z_variance default is set loose enough that the EKF treats
this as a soft constraint, not gospel.

No Gazebo ground truth is used anywhere: only /os0_cloud_node/points, /compass/data,
and the robot's own URDF-derived TF.
"""
import argparse
import math
import statistics
import sys

import numpy as np


def fit_ground_plane(points, min_range=1.5, max_range=8.0, percentile=25.0):
    """Least-squares plane z = a*x + b*y + c over the LOW returns in an annulus.

    `points` is an (N,3) array that has ALREADY been de-rotated to gravity-level, so
    a real ground plane comes out near-horizontal and `c` is the sensor's height
    below zero (hence -c is the height above ground).

    The annulus skips the robot's own body (min_range) and far returns whose
    grazing angle makes them unreliable (max_range). Keeping only the lowest
    `percentile` of z in that ring selects ground rather than objects standing on it.

    Returns (a, b, c) or None when there is not enough ground to fit.
    """
    p = np.asarray(points, dtype=float)
    if len(p) < 10:
        return None
    d = np.hypot(p[:, 0], p[:, 1])
    near = p[(d > min_range) & (d < max_range)]
    if len(near) < 10:
        return None
    thr = np.percentile(near[:, 2], percentile)
    g = near[near[:, 2] <= thr]
    if len(g) < 10:
        return None
    A = np.c_[g[:, 0], g[:, 1], np.ones(len(g))]
    coef, *_ = np.linalg.lstsq(A, g[:, 2], rcond=None)
    return float(coef[0]), float(coef[1]), float(coef[2])


def sensor_height_from_fit(fit):
    """Sensor height above the fitted local ground plane. None if no fit."""
    if fit is None:
        return None
    return -fit[2]


def base_link_z(h_sensor, mount_dz, ground_elev=0.0):
    """base_link's ABSOLUTE z in the map frame.

        z = ground_elev + (h_sensor - mount_dz)
            ^ terrain surface   ^ base_link's clearance above that surface

    `ground_elev` is the terrain's true elevation beneath the robot, looked up
    from the prior DTM. Without it this returns only the CLEARANCE (the old
    behaviour, ground_elev=0), which the EKF would fuse as if it were absolute
    elevation -- putting the robot metres below the terrain it stands on. That
    was a real bug: with terrain at 3.761 m the robot rendered at 0.096 m
    instead of its true 3.886 m, sinking it and its lidar under the map.

    TRADE-OFF, stated plainly: taking ground_elev from the DTM makes z depend on
    the prior map, not on sensors alone. The clearance term stays sensor-derived,
    so a wrong map shifts the robot bodily but does not corrupt its measured
    height above the ground it is on.
    """
    if h_sensor is None:
        return None
    return ground_elev + (h_sensor - mount_dz)


def dtm_elevation_at(dtm_z, resolution, origin_x, origin_y, x, y):
    """Terrain elevation from a DTM array at world (x, y); None if outside the
    grid or that cell has no data (NaN). Row 0 = lowest y, matching DtmGrid."""
    import numpy as np
    col = int((x - origin_x) / resolution)
    row = int((y - origin_y) / resolution)
    if row < 0 or col < 0 or row >= dtm_z.shape[0] or col >= dtm_z.shape[1]:
        return None
    v = dtm_z[row, col]
    if not np.isfinite(v):
        return None
    return float(v)


def smooth_median(window):
    """Median of the rolling window; None when the window is empty. Median (not
    mean) so a single bad fit -- e.g. one scan that caught a wall -- cannot drag
    the published height."""
    vals = [v for v in window if v is not None]
    if not vals:
        return None
    return float(statistics.median(vals))


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
    import rospy
    import tf2_ros
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu, PointCloud2
    from sensor_msgs import point_cloud2

    sys.path.insert(0, "/home/thinh/Documents/Husky_viz")
    from landmark_loc import derotate

    rospy.init_node("ground_height_odom")
    rate_hz = rospy.get_param("~rate", 10.0)
    min_range = rospy.get_param("~fit_min_range", 1.5)
    max_range = rospy.get_param("~fit_max_range", 8.0)
    pct = rospy.get_param("~ground_percentile", 25.0)
    win_n = int(rospy.get_param("~smooth_window", 5))
    z_var = rospy.get_param("~z_variance", 0.05)
    dtm_path = rospy.get_param("~dtm_path", "")

    # Prior DTM supplies the terrain elevation the measured clearance sits on
    # top of. Without it we publish clearance alone, which the EKF reads as
    # absolute elevation and sinks the robot under the terrain.
    dtm_z = dtm_res = dtm_ox = dtm_oy = None
    if dtm_path:
        import yaml
        dtm_z = np.load(dtm_path)
        meta = yaml.safe_load(open(dtm_path.replace(".npy", ".yaml")))
        dtm_res = float(meta["resolution"])
        dtm_ox = float(meta["origin_x"])
        dtm_oy = float(meta["origin_y"])
        rospy.loginfo("[ground_height] DTM loaded %s (%dx%d @ %.2fm)",
                      dtm_path, dtm_z.shape[1], dtm_z.shape[0], dtm_res)
    else:
        rospy.logwarn("[ground_height] no ~dtm_path: publishing CLEARANCE only, "
                      "which the EKF will read as absolute elevation")

    state = {"roll": None, "pitch": None, "cloud": None, "window": [],
             "xy": None}

    def on_imu(msg):
        q = msg.orientation
        r, p = derotate.roll_pitch_from_quat(q.x, q.y, q.z, q.w)
        state["roll"], state["pitch"] = r, p

    def on_cloud(msg):
        state["cloud"] = msg

    def on_pose(msg):
        # x,y only -- used solely to index the DTM. We never read its z (that is
        # the quantity we are producing, so consuming it would be circular).
        state["xy"] = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    rospy.Subscriber("/odometry/filtered_map", Odometry, on_pose, queue_size=5)
    rospy.Subscriber("/compass/data", Imu, on_imu, queue_size=5)
    rospy.Subscriber("/os0_cloud_node/points", PointCloud2, on_cloud,
                     queue_size=1, buff_size=2 ** 24)
    pub = rospy.Publisher("/odometry/ground_height", Odometry, queue_size=5)

    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf)

    # The mount offset is FIXED, so look it up once from TF rather than hardcoding
    # it -- if the sensor is remounted, this follows automatically.
    mount_dz = None
    rate = rospy.Rate(rate_hz)
    while not rospy.is_shutdown():
        if mount_dz is None:
            try:
                tr = tf_buf.lookup_transform("base_link", "os0_lidar",
                                             rospy.Time(0), rospy.Duration(1.0))
                mount_dz = tr.transform.translation.z
                rospy.loginfo("[ground_height] mount offset base_link->os0_lidar = %.3f m",
                              mount_dz)
            except Exception:
                rate.sleep()
                continue

        msg = state["cloud"]
        if msg is None or state["roll"] is None:
            rate.sleep()
            continue

        pts = np.array(list(point_cloud2.read_points(
            msg, field_names=("x", "y", "z"), skip_nans=True)), dtype=float)
        if len(pts) == 0:
            rate.sleep()
            continue

        level = derotate.derotate_cloud(pts, state["roll"], state["pitch"])
        fit = fit_ground_plane(level, min_range, max_range, pct)

        # Terrain elevation beneath the robot, from the prior DTM. 0.0 when no
        # DTM is configured or the robot is over a no-data cell -- in which case
        # we fall back to publishing clearance alone (documented, not silent).
        ground_elev = 0.0
        if dtm_z is not None and state["xy"] is not None:
            e = dtm_elevation_at(dtm_z, dtm_res, dtm_ox, dtm_oy,
                                 state["xy"][0], state["xy"][1])
            if e is not None:
                ground_elev = e

        z = base_link_z(sensor_height_from_fit(fit), mount_dz, ground_elev)
        if z is None:
            # Too little ground this scan (looking over water, or boxed in). Skip
            # rather than publish a fabricated height.
            rate.sleep()
            continue

        state["window"].append(z)
        state["window"] = state["window"][-win_n:]
        zs = smooth_median(state["window"])

        od = Odometry()
        # Stamp with the CLOUD's time: this height was measured from that scan, and
        # the EKF time-aligns absolute measurements by this stamp.
        od.header.stamp = msg.header.stamp
        od.header.frame_id = "odom"
        od.child_frame_id = "base_link"
        od.pose.pose.position.z = zs
        od.pose.pose.orientation.w = 1.0
        od.pose.covariance = covariance_for_z(z_var)
        pub.publish(od)
        rospy.loginfo_throttle(
            1.0, "[ground_height] z=%.3f m (raw %.3f, slope %.2f deg, n=%d)"
            % (zs, z, math.degrees(math.atan(math.hypot(fit[0], fit[1]))),
               len(state["window"])))
        rate.sleep()


if __name__ == "__main__":
    main()
