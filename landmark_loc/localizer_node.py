#!/usr/bin/env python3
"""ROS node: publish an absolute map-frame pose fix from lidar landmark matching.

Pipeline per cloud: cloud->array -> crop -> cluster -> classify -> gate catalog
by a GPS-anchored dead-reckoned prior (initial pre-attack GPS anchor advanced by
odom-frame motion, re-anchored on each accepted fix) -> associate ->
rigid-transform solve -> publish /odometry/landmark_fix
(only on a fit that passes the residual+count gate; otherwise silent so the EKF
coasts on odom). Position-only: yaw from the solve is logged, not fused (the
map-EKF takes yaw from /compass/data).
"""
import math

import numpy as np

from landmark_loc import segment, classify, catalog, solve


def covariance_for(n_matches, base_var):
    cov = [0.0] * 36
    pos_var = base_var / max(n_matches, 1)
    cov[0] = pos_var      # x
    cov[7] = pos_var      # y
    cov[14] = 1e6         # z (unused, 2d)
    cov[21] = 1e6         # roll
    cov[28] = 1e6         # pitch
    cov[35] = 1e6         # yaw (NOT fused from here)
    return cov


def cloud_to_array(cloud_msg):
    from sensor_msgs import point_cloud2
    pts = point_cloud2.read_points(
        cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.array(list(pts), dtype=float)


def compose_prior(anchor_map, anchor_odom, odom_now):
    """Map-frame prior = anchor_map advanced by the odom-frame displacement of
    odom_now relative to anchor_odom.

    anchor_map  = (ax, ay, ayaw)   immutable map-frame anchor (pre-attack GPS,
                                   or the last accepted landmark fix)
    anchor_odom = (ox0, oy0, oyaw0) odom-frame pose captured with the anchor
    odom_now    = (ox, oy, oyaw)    current odom-frame pose (attack-independent)

    The odom frame drifts but its relative motion is trustworthy, so the
    displacement since the anchor, applied from the anchor, tracks the robot's
    true pose without ever reading the (spoofable) map pose.
    """
    ax, ay, ayaw = anchor_map
    ox0, oy0, oyaw0 = anchor_odom
    ox, oy, oyaw = odom_now
    # displacement in the odom frame, rotated into the anchor-odom body frame
    dx_o, dy_o = ox - ox0, oy - oy0
    c0, s0 = math.cos(-oyaw0), math.sin(-oyaw0)
    rx = c0 * dx_o - s0 * dy_o
    ry = s0 * dx_o + c0 * dy_o
    # apply that body-frame displacement from the map-frame anchor
    ca, sa = math.cos(ayaw), math.sin(ayaw)
    px = ax + ca * rx - sa * ry
    py = ay + sa * rx + ca * ry
    pyaw = ayaw + (oyaw - oyaw0)
    return (px, py, pyaw)


def main():
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.msg import NavSatFix

    rospy.init_node("landmark_localizer")
    places = rospy.get_param("~places_path",
                             "/home/thinh/Documents/Husky_viz/maps/park_places.yaml")
    p = dict(
        z_min=rospy.get_param("~z_min", -0.73),
        z_max=rospy.get_param("~z_max", 1.2),
        max_range=rospy.get_param("~max_range", 15.0),
        link_dist=rospy.get_param("~link_dist", 0.3),
        min_pts=rospy.get_param("~min_pts", 10),
        max_extent=rospy.get_param("~max_extent", 3.5),
        dist_gate=rospy.get_param("~dist_gate", 2.0),
        residual_gate=rospy.get_param("~residual_gate", 0.5),
        fov_halfwidth=rospy.get_param("~fov_halfwidth", math.pi),
        base_var=rospy.get_param("~base_var", 0.5),
        rate=rospy.get_param("~rate", 5.0),
        anchor_min_dist=rospy.get_param("~anchor_min_dist", 5.0),
    )
    landmarks = catalog.load(places)
    rospy.loginfo("landmark_localizer: %d catalog landmarks", len(landmarks))

    state = {
        "anchor_map": None,    # (ax, ay, ayaw) immutable-ish map anchor
        "anchor_odom": None,   # (ox0, oy0, oyaw0) odom pose captured with anchor
        "odom_now": None,      # (ox, oy, oyaw) latest odom-frame pose
        "gps_valid": False,    # /navsat/fix status.status >= 0 seen
        "last_pub": rospy.Time(0),
    }
    pub = rospy.Publisher("/odometry/landmark_fix", Odometry, queue_size=5)

    def _yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def on_odom(msg):
        p_ = msg.pose.pose.position
        state["odom_now"] = (p_.x, p_.y, _yaw(msg.pose.pose.orientation))

    def on_navsat(msg):
        if msg.status.status >= 0:
            state["gps_valid"] = True

    def on_map(msg):
        # ONE-TIME anchor capture: only before an anchor exists, only when GPS
        # is valid and an odom pose is available. Never updates the anchor after.
        if state["anchor_map"] is not None:
            return
        if not state["gps_valid"] or state["odom_now"] is None:
            return
        # Wait for the map-EKF to CONVERGE: before GPS is fused, filtered_map
        # sits at the origin (0,0); once anchored it jumps to the true pose
        # (robot spawns ~45m from the datum). Capturing while still near origin
        # would record a ~45m-wrong anchor that never self-corrects. Requiring
        # the pose to be clearly away from origin proves GPS has converged.
        if math.hypot(msg.pose.pose.position.x,
                      msg.pose.pose.position.y) < p["anchor_min_dist"]:
            return
        p_ = msg.pose.pose.position
        state["anchor_map"] = (p_.x, p_.y, _yaw(msg.pose.pose.orientation))
        state["anchor_odom"] = state["odom_now"]
        rospy.loginfo("anchor captured: map=(%.2f,%.2f,%.2f) odom=(%.2f,%.2f,%.2f)",
                      state["anchor_map"][0], state["anchor_map"][1], state["anchor_map"][2],
                      state["anchor_odom"][0], state["anchor_odom"][1], state["anchor_odom"][2])

    def on_cloud(msg):
        now = rospy.Time.now()
        if (now - state["last_pub"]).to_sec() < 1.0 / p["rate"]:
            return
        if (state["anchor_map"] is None or state["anchor_odom"] is None
                or state["odom_now"] is None):
            return
        rospy.loginfo_throttle(1.0, "[diag] anchor_map=%s anchor_odom=%s odom_now=%s"
                               % (state["anchor_map"], state["anchor_odom"], state["odom_now"]))
        prior = compose_prior(state["anchor_map"], state["anchor_odom"],
                              state["odom_now"])
        rospy.loginfo_throttle(1.0, "[diag] prior=(%.2f,%.2f,%.2f)" % prior)
        pts = cloud_to_array(msg)
        if len(pts) == 0:
            return
        cropped = segment.crop(pts, p["z_min"], p["z_max"], p["max_range"])
        clusters = segment.cluster(cropped, p["link_dist"], p["min_pts"], p["max_extent"])
        obs = classify.to_observations(clusters)
        gated = catalog.gate(landmarks, prior, p["max_range"], p["fov_halfwidth"])
        rospy.loginfo_throttle(1.0, "[diag] obs=%d types=%s gated=%d"
                               % (len(obs), [o.identity for o in obs], len(gated)))
        result = solve.solve_pose(obs, gated, prior, p["dist_gate"], p["residual_gate"])
        if result is None:
            rospy.loginfo_throttle(1.0, "[diag] solve=None (no fix this tick)")
            return
        rospy.loginfo_throttle(1.0, "[diag] FIX x=%.2f y=%.2f rms=%.3f n=%d"
                               % (result[0], result[1], result[3], result[4]))
        x, y, yaw, rms, n = result
        # RE-ANCHOR: an accepted (gated) fix is a trustworthy landmark-derived
        # absolute position. Reset the dead-reckoning baseline to it so drift
        # only accumulates between fixes, never over the whole run.
        state["anchor_map"] = (x, y, prior[2])   # keep composed yaw (yaw not solved/fused)
        state["anchor_odom"] = state["odom_now"]
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = x
        od.pose.pose.position.y = y
        od.pose.pose.orientation.w = 1.0
        od.pose.covariance = covariance_for(n, p["base_var"])
        pub.publish(od)
        state["last_pub"] = now

    rospy.Subscriber("/odometry/filtered_odom", Odometry, on_odom, queue_size=5)
    rospy.Subscriber("/navsat/fix", NavSatFix, on_navsat, queue_size=5)
    rospy.Subscriber("/odometry/filtered_map", Odometry, on_map, queue_size=5)
    rospy.Subscriber("/os0_cloud_node/points", PointCloud2, on_cloud, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
