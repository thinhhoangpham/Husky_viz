#!/usr/bin/env python3
"""Publish BLUE RViz markers at the centroids of the lake map's duplicate
constellation pair, plus the trees forming each group.

VISUALIZATION ONLY. Reads maps/lake_objects.yaml (the static map) and
maps/lake_dtm.npy for terrain height. Reads NO robot pose and no ground truth.

Background: the lake catalog has exactly one pair of fully-disjoint tree triples
that are congruent within the constellation matcher's _INLIER_TOL (0.5 m) --
max per-point residual 0.329 m, rms 0.294 m -- so they are indistinguishable by
shape alone. Their centroids are 59.4 m apart, which the matcher's
_PRIOR_SANITY (15 m) guard rejects.

Publishes latched on ~topic (default /duplicate_constellations) in frame `map`.
Add a MarkerArray display on that topic in RViz.
"""
import math

import numpy as np
import rospy
import yaml
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# The one duplicate pair, from the congruence search over all 286 tree triples.
GROUP_A = ["tree_8_clone_clone", "tree_8_clone_clone_2",
           "tree_8_clone_clone_clone_clone_1"]
GROUP_B = ["tree_8_clone_clone_1", "tree_8_clone_clone_clone_clone_3",
           "tree_8_clone_clone_clone_clone_4"]

BLUE = (0.15, 0.45, 1.0)
Z_LIFT = 0.5   # metres above terrain, so the marker clears the ground cloud


def terrain_z(dtm, meta, x, y, default=4.5):
    res = meta["resolution"]
    i = int(round((x - meta["origin_x"]) / res))
    j = int(round((y - meta["origin_y"]) / res))
    if 0 <= j < dtm.shape[0] and 0 <= i < dtm.shape[1]:
        z = dtm[j, i]
        if not np.isnan(z):
            return float(z)
    return default


def main():
    rospy.init_node("mark_duplicate_constellations")
    repo = rospy.get_param("~repo", "/home/thinh/Documents/Husky_viz")
    topic = rospy.get_param("~topic", "/duplicate_constellations")

    objs = yaml.safe_load(open("%s/maps/lake_objects.yaml" % repo))
    dtm = np.load("%s/maps/lake_dtm.npy" % repo)
    meta = yaml.safe_load(open("%s/maps/lake_dtm.yaml" % repo))

    pub = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    arr = MarkerArray()
    mid = 0

    for label, names in (("A", GROUP_A), ("B", GROUP_B)):
        pts = [(objs[n]["x"], objs[n]["y"]) for n in names]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        cz = terrain_z(dtm, meta, cx, cy) + Z_LIFT

        # 1) big sphere at the centroid
        m = Marker()
        m.header.frame_id = "map"
        m.ns = "dup_centroid"
        m.id = mid; mid += 1
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = cx
        m.pose.position.y = cy
        m.pose.position.z = cz
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 2.5
        m.color.r, m.color.g, m.color.b = BLUE
        m.color.a = 0.9
        arr.markers.append(m)

        # 2) text label above it
        t = Marker()
        t.header.frame_id = "map"
        t.ns = "dup_label"
        t.id = mid; mid += 1
        t.type = Marker.TEXT_VIEW_FACING
        t.action = Marker.ADD
        t.pose.position.x = cx
        t.pose.position.y = cy
        t.pose.position.z = cz + 2.5
        t.pose.orientation.w = 1.0
        t.scale.z = 1.8
        t.color.r, t.color.g, t.color.b = BLUE
        t.color.a = 1.0
        t.text = "constellation %s" % label
        arr.markers.append(t)

        # 3) triangle outline through the three trees
        ln = Marker()
        ln.header.frame_id = "map"
        ln.ns = "dup_shape"
        ln.id = mid; mid += 1
        ln.type = Marker.LINE_STRIP
        ln.action = Marker.ADD
        ln.pose.orientation.w = 1.0
        ln.scale.x = 0.35
        ln.color.r, ln.color.g, ln.color.b = BLUE
        ln.color.a = 0.85
        for x, y in pts + [pts[0]]:
            ln.points.append(Point(x, y, terrain_z(dtm, meta, x, y) + Z_LIFT))
        arr.markers.append(ln)

        rospy.loginfo("[dup] constellation %s centroid=(%.3f, %.3f, %.3f)",
                      label, cx, cy, cz)

    pub.publish(arr)
    rospy.loginfo("[dup] published %d markers on %s (latched)",
                  len(arr.markers), topic)
    rospy.spin()


if __name__ == "__main__":
    main()
