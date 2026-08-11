#!/usr/bin/env python3
"""ROS node: publish an absolute map-frame pose fix from lidar landmark matching.

Pipeline per cloud: cloud->array -> crop -> cluster -> classify -> gate catalog
by the EKF prior -> associate -> rigid-transform solve -> publish /odometry/landmark_fix
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
        residual_gate=rospy.get_param("~residual_gate", 0.4),
        fov_halfwidth=rospy.get_param("~fov_halfwidth", math.pi),
        base_var=rospy.get_param("~base_var", 0.5),
        rate=rospy.get_param("~rate", 5.0),
    )
    landmarks = catalog.load(places)
    rospy.loginfo("landmark_localizer: %d catalog landmarks", len(landmarks))

    state = {"prior": None, "last_pub": rospy.Time(0)}
    pub = rospy.Publisher("/odometry/landmark_fix", Odometry, queue_size=5)

    def on_prior(msg):
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        state["prior"] = (msg.pose.pose.position.x,
                          msg.pose.pose.position.y, yaw)

    def on_cloud(msg):
        now = rospy.Time.now()
        if (now - state["last_pub"]).to_sec() < 1.0 / p["rate"]:
            return
        if state["prior"] is None:
            return
        pts = cloud_to_array(msg)
        if len(pts) == 0:
            return
        cropped = segment.crop(pts, p["z_min"], p["z_max"], p["max_range"])
        clusters = segment.cluster(cropped, p["link_dist"], p["min_pts"], p["max_extent"])
        obs = classify.to_observations(clusters)
        gated = catalog.gate(landmarks, state["prior"], p["max_range"], p["fov_halfwidth"])
        result = solve.solve_pose(obs, gated, state["prior"],
                                  p["dist_gate"], p["residual_gate"])
        if result is None:
            return
        x, y, yaw, rms, n = result
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

    rospy.Subscriber("/odometry/filtered_map", Odometry, on_prior, queue_size=5)
    rospy.Subscriber("/os0_cloud_node/points", PointCloud2, on_cloud, queue_size=1)
    rospy.spin()


if __name__ == "__main__":
    main()
