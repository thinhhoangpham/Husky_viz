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
import statistics

import numpy as np

from landmark_loc import segment, catalog, solve, detector


def _is_landmark_mode(mode_str):
    """True when mode_str is "landmark" or "landmark:stale" (the localizer
    should run its pipeline); False for "gps", None, or empty (dormant)."""
    return str(mode_str or "").startswith("landmark")


def _jump_ok(fix_xy, last_pub_xy, odom_disp, max_jump):
    """Physical-motion gate: a fix must land within max_jump of where the robot
    can be (last published pose advanced by odom displacement). Bootstrap
    (no last_pub_xy) always accepts. Pure output filter; never re-anchors."""
    if last_pub_xy is None:
        return True
    ex = last_pub_xy[0] + odom_disp[0]
    ey = last_pub_xy[1] + odom_disp[1]
    return math.hypot(fix_xy[0] - ex, fix_xy[1] - ey) <= max_jump


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


def compose_prior(anchor_map, anchor_odom, odom_now, heading_yaw):
    """Map-frame prior = anchor_map advanced by the odom-frame displacement of
    odom_now relative to anchor_odom, with heading taken from the ABSOLUTE
    compass yaw (never from odom/fused yaw).

    anchor_map  = (ax, ay, ayaw)   immutable map-frame anchor (pre-attack GPS,
                                   or the last accepted landmark fix)
    anchor_odom = (ox0, oy0, oyaw0) odom-frame pose captured with the anchor
    odom_now    = (ox, oy, oyaw)    current odom-frame pose (attack-independent)
    heading_yaw = current absolute compass yaw (/compass/data), drift-free

    The odom frame drifts but its relative motion is trustworthy, so the
    displacement since the anchor, applied from the anchor, tracks the robot's
    true position without ever reading the (spoofable) map pose. Odom yaw is
    NOT trustworthy while turning (skid-steer wheel scrub under-reports
    rotation), so the prior's heading comes from the compass instead.
    """
    ax, ay, ayaw = anchor_map
    ox0, oy0, oyaw0 = anchor_odom
    ox, oy, _oyaw = odom_now
    # displacement in the odom frame, rotated into the anchor-odom body frame
    dx_o, dy_o = ox - ox0, oy - oy0
    c0, s0 = math.cos(-oyaw0), math.sin(-oyaw0)
    rx = c0 * dx_o - s0 * dy_o
    ry = s0 * dx_o + c0 * dy_o
    # apply that body-frame displacement from the map-frame anchor
    ca, sa = math.cos(ayaw), math.sin(ayaw)
    px = ax + ca * rx - sa * ry
    py = ay + sa * rx + ca * ry
    pyaw = heading_yaw
    return (px, py, pyaw)


def odom_at(buf, t):
    """Interpolated (x, y, yaw) odom pose at time t from buf, a list of
    (t, x, y, yaw) sorted ascending by t. Returns None if buf is empty.
    Clamps to the oldest/newest sample if t falls outside the buffer's span.
    Yaw is interpolated via shortest-angle wrap.
    """
    if not buf:
        return None
    if t <= buf[0][0]:
        return buf[0][1], buf[0][2], buf[0][3]
    if t >= buf[-1][0]:
        return buf[-1][1], buf[-1][2], buf[-1][3]
    for i in range(len(buf) - 1):
        t_i, x_i, y_i, yaw_i = buf[i]
        t_j, x_j, y_j, yaw_j = buf[i + 1]
        if t_i <= t <= t_j:
            if t_j == t_i:
                return x_i, y_i, yaw_i
            frac = (t - t_i) / (t_j - t_i)
            x = x_i + frac * (x_j - x_i)
            y = y_i + frac * (y_j - y_i)
            dyaw = math.atan2(math.sin(yaw_j - yaw_i), math.cos(yaw_j - yaw_i))
            yaw = yaw_i + frac * dyaw
            return x, y, yaw
    # unreachable given the clamps above, but keep a safe fallback
    return buf[-1][1], buf[-1][2], buf[-1][3]


# RGBA color per classifier identity, for the observed-cluster text labels
# (helps the operator spot misclassifications and 'unknown' clusters at a
# glance). Purely a visualization constant; does not affect classification.
# Marker label colors by classifier identity. Sourced from the single type
# registry (map_tools.park_types): every catalog identity contributes its
# marker_color (lamp yellow, bench green, garden_table cyan, trash_bin_1 orange,
# tree dark green). "unknown" is not a registry type -- it is the classifier's
# drop label -- so its red is kept as a literal here.
from map_tools.park_types import PARK_TYPES as _PARK_TYPES

_LABEL_COLOR = {t.identity: t.marker_color for t in _PARK_TYPES if t.is_catalog}
_LABEL_COLOR["unknown"] = (1.0, 0.0, 0.0, 1.0)  # red


def marker_text(ident, confidence):
    """Label text for one observed cluster: identity, plus confidence if any.

    `confidence` is None when the detector reports no meaningful score for
    this cluster, in which case the text is the bare identity -- which is what
    keeps this sane for BOTH detectors. The cascade has no score (it reports a
    constant 1.0), and printing "bench 1.00" on every label would be noise
    dressed up as information; the caller passes None for it. A scoring
    detector passes its real number and the operator sees "bench 0.75".

    Rejected clusters still read "unknown", and still show their score when
    one exists -- "unknown 0.04" tells the operator the percept ALMOST made
    its floor, which is exactly the case a too-tight floor produces and is
    invisible from the label alone.
    """
    if confidence is None:
        return ident
    return "%s %.2f" % (ident, confidence)


#: detector modes whose confidence is a REAL score worth showing. The cascade
#: is excluded on purpose: it emits a constant 1.0, which is not evidence.
_SCORING_CLASSIFIERS = ("score",)


def cluster_confidences(labels, observations, classifier):
    """Per-cluster confidence list parallel to `labels`, or None.

    Returns None for a non-scoring detector (the cascade), which makes the
    markers and the [diag] line fall back to bare identities -- see
    `marker_text` for why a constant 1.0 is worth hiding.

    For a scoring detector, the score is read back off the emitted
    Observations rather than by re-classifying: `detect` returns accepted
    percepts IN INPUT ORDER (the seam contract), so the i-th non-unknown label
    owns the i-th Observation. Rejected clusters get None -- their below-floor
    score is not carried on an Observation, because no Observation was
    emitted for them.

    Guarded, not assumed: if the two sequences do not line up, this returns
    None (no confidence shown) rather than mislabelling every cluster with
    somebody else's score.
    """
    if classifier not in _SCORING_CLASSIFIERS:
        return None
    accepted = [i for i, l in enumerate(labels) if l != "unknown"]
    if len(accepted) != len(observations):
        return None
    by_index = dict(zip(accepted, observations))
    return [None if i not in by_index else by_index[i].confidence
            for i in range(len(labels))]


def confidence_summary(observations, classifier):
    """One-line per-detection confidence summary for the [diag] log, or "".

    Empty string for a non-scoring detector, so the cascade's [diag] line is
    byte-for-byte what it was. For a scoring detector it reads e.g.
    " conf=[lamp:0.94,bench:0.75] min=0.75" -- per-detection, because an
    average would hide the ONE weak detection that is about to be associated
    to a catalog landmark, which is the thing worth seeing in a live run.
    """
    if classifier not in _SCORING_CLASSIFIERS or not observations:
        return ""
    per = ",".join("%s:%.2f" % (o.identity, o.confidence) for o in observations)
    return " conf=[%s] min=%.2f" % (
        per, min(o.confidence for o in observations))


def build_observed_markers(clusters, frame_id, stamp, labels, confidences=None):
    """Build a MarkerArray of TEXT_VIEW_FACING labels, one per cluster, showing
    the classifier's identity string for that cluster (including 'unknown').
    `labels` is the precomputed per-cluster identity list from
    `Detector.detect` -- passed in rather than recomputed here so the tick
    classifies exactly once (see landmark_loc.detector, ONE PASS PER TICK).
    `confidences`, when given, is a parallel per-cluster score list (None
    entries allowed) appended to the label text by `marker_text`; omit it for
    a detector with no meaningful score.
    First element is a DELETEALL so stale labels from a tick with more
    clusters than the current tick don't linger."""
    import rospy
    from visualization_msgs.msg import Marker, MarkerArray

    arr = MarkerArray()
    delete_all = Marker()
    delete_all.action = Marker.DELETEALL
    arr.markers.append(delete_all)

    for i, c in enumerate(clusters):
        ident = labels[i]
        cx, cy = c.centroid_xy
        if c.points is not None and len(c.points) > 0:
            z_top = float(c.points[:, 2].max())
        else:
            z_top = 0.0
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = stamp
        m.ns = "observed"
        m.id = i
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.position.x = cx
        m.pose.position.y = cy
        m.pose.position.z = z_top + 0.5
        m.pose.orientation.w = 1.0
        m.scale.z = 0.6
        r, g, b, a = _LABEL_COLOR.get(ident, _LABEL_COLOR["unknown"])
        m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, a
        m.lifetime = rospy.Duration(0.5)
        m.text = marker_text(
            ident, None if confidences is None else confidences[i])
        arr.markers.append(m)
    return arr


def main():
    import rospy
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from sensor_msgs.msg import NavSatFix
    from sensor_msgs.msg import Imu
    from std_msgs.msg import String
    from visualization_msgs.msg import MarkerArray

    rospy.init_node("landmark_localizer")
    objects = rospy.get_param("~objects_path",
                              "/home/thinh/Documents/Husky_viz/maps/park_objects.yaml")
    p = dict(
        z_min=rospy.get_param("~z_min", -0.5),  # measured per tree-landmark spec: raised to drop ground blob
        z_max=rospy.get_param("~z_max", 7.0),  # measured per tree-landmark spec: raised to include tree canopies
        max_range=rospy.get_param("~max_range", 15.0),
        link_dist=rospy.get_param("~link_dist", 0.3),
        min_pts=rospy.get_param("~min_pts", 10),
        max_extent=rospy.get_param("~max_extent", 6.0),  # measured per tree-landmark spec: raised so canopies survive clustering
        constellation_tol=rospy.get_param("~constellation_tol", 1.0),
        dist_gate=rospy.get_param("~dist_gate", 6.0),  # nearest-neighbor match radius (map metres); must exceed odom drift, stay under same-type landmark spacing
        max_prior_dist=rospy.get_param("~max_prior_dist", 5.0),
        residual_gate=rospy.get_param("~residual_gate", 1.0),
        fov_halfwidth=rospy.get_param("~fov_halfwidth", math.pi),
        base_var=rospy.get_param("~base_var", 0.5),
        rate=rospy.get_param("~rate", 2.0),
        anchor_min_dist=rospy.get_param("~anchor_min_dist", 5.0),
        smooth_window=rospy.get_param("~smooth_window", 5),
        max_jump=rospy.get_param("~max_jump", 3.0),
        matcher=rospy.get_param("~matcher", "typed"),
        classifier=rospy.get_param("~classifier", detector.DEFAULT_DETECTOR),
    )
    if p["matcher"] not in ("typed", "typeless"):
        rospy.logwarn("landmark_localizer: unknown ~matcher %r, defaulting to 'typed'",
                      p["matcher"])
        p["matcher"] = "typed"
    if p["classifier"] not in detector.DETECTORS:
        rospy.logwarn("landmark_localizer: unknown ~classifier %r, defaulting to %r",
                      p["classifier"], detector.DEFAULT_DETECTOR)
        p["classifier"] = detector.DEFAULT_DETECTOR
    det = detector.get_detector(p["classifier"])
    landmarks = catalog.load(objects)
    rospy.loginfo("landmark_localizer: %d catalog landmarks", len(landmarks))
    rospy.loginfo("[localizer] matcher mode: %s", p["matcher"])
    rospy.loginfo("[localizer] classifier mode: %s", p["classifier"])

    state = {
        "anchor_map": None,    # (ax, ay, ayaw) immutable-ish map anchor
        "anchor_odom": None,   # (ox0, oy0, oyaw0) odom pose captured with anchor
        "odom_now": None,      # (ox, oy, oyaw) latest odom-frame pose
        "odom_buf": [],        # [(t, ox, oy, oyaw)] last ~2.0s, for cloud-stamp sync
        "compass_yaw": None,   # latest absolute yaw from /compass/data
        "gps_valid": False,    # /navsat/fix status.status >= 0 seen
        "last_pub": rospy.Time(0),
        "fix_history": [],   # last N accepted (x, y) fixes for median smoothing
        "abs_fix_mode": "gps",  # dormant until /abs_fix_mode says otherwise
        "last_pub_xy": None,    # (x, y) of last accepted/published fix
        "last_pub_odom": None,  # odom pose captured at last published fix
    }
    pub = rospy.Publisher("/odometry/landmark_fix", Odometry, queue_size=5)
    markers_pub = rospy.Publisher("/landmark_observed_markers", MarkerArray, queue_size=1)

    def _yaw(q):
        return math.atan2(2 * (q.w * q.z + q.x * q.y),
                          1 - 2 * (q.y * q.y + q.z * q.z))

    def on_odom(msg):
        p_ = msg.pose.pose.position
        state["odom_now"] = (p_.x, p_.y, _yaw(msg.pose.pose.orientation))
        t = msg.header.stamp.to_sec()
        state["odom_buf"].append((t, p_.x, p_.y, state["odom_now"][2]))
        cutoff = t - 2.0
        buf = state["odom_buf"]
        i = 0
        while i < len(buf) and buf[i][0] < cutoff:
            i += 1
        if i:
            state["odom_buf"] = buf[i:]

    def on_compass(msg):
        state["compass_yaw"] = _yaw(msg.orientation)

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

    def on_mode(msg):
        state["abs_fix_mode"] = msg.data

    def on_cloud(msg):
        if not _is_landmark_mode(state["abs_fix_mode"]):
            return
        now = rospy.Time.now()
        if (now - state["last_pub"]).to_sec() < 1.0 / p["rate"]:
            return
        if (state["anchor_map"] is None or state["anchor_odom"] is None
                or state["odom_now"] is None):
            return
        if state["compass_yaw"] is None:
            return
        odom_synced = odom_at(state["odom_buf"], msg.header.stamp.to_sec())
        if odom_synced is None:
            return
        prior = compose_prior(state["anchor_map"], state["anchor_odom"],
                              odom_synced, state["compass_yaw"])
        pts = cloud_to_array(msg)
        if len(pts) == 0:
            return
        cropped = segment.crop(pts, p["z_min"], p["z_max"], p["max_range"])
        clusters = segment.cluster(cropped, p["link_dist"], p["min_pts"], p["max_extent"])
        # One classification pass per tick: detect() returns the labels the
        # markers need (all clusters, including 'unknown') AND the accepted
        # observations, stamped with THIS cloud's frame/time so downstream
        # never has to assume where/when they came from.
        labels, obs = det.detect(clusters, frame_id=msg.header.frame_id,
                                 stamp=msg.header.stamp.to_sec())
        confs = cluster_confidences(labels, obs, p["classifier"])
        markers_pub.publish(build_observed_markers(
            clusters, msg.header.frame_id, msg.header.stamp, labels, confs))
        gated = catalog.gate(landmarks, prior, p["max_range"], p["fov_halfwidth"])
        _match_mod = (solve.constellation_typeless if p["matcher"] == "typeless"
                      else solve.constellation)
        _pairs = _match_mod.match(obs, gated, prior, p["constellation_tol"],
                                   p["max_prior_dist"])
        result = solve.solve_pose(obs, gated, prior, p["constellation_tol"], p["residual_gate"],
                                   p["max_prior_dist"], matcher=p["matcher"])
        if result is None:
            _matched = ",".join(lm.name for _o, lm in _pairs)
            rospy.loginfo_throttle(0.5,
                "[diag] obs=%d assoc=%d prior=(%.1f,%.1f) matched=[%s] %s%s"
                % (len(obs), len(_pairs), prior[0], prior[1], _matched, "STALE",
                   confidence_summary(obs, p["classifier"])))
            return
        x, y, yaw, rms, n = result
        # Physical-motion gate: reject a fix that teleports beyond reachable.
        if state["last_pub_odom"] is not None:
            odom_disp = (odom_synced[0] - state["last_pub_odom"][0],
                         odom_synced[1] - state["last_pub_odom"][1])
        else:
            odom_disp = (0.0, 0.0)
        if not _jump_ok((x, y), state["last_pub_xy"], odom_disp, p["max_jump"]):
            rospy.loginfo_throttle(0.5,
                "[diag] obs=%d assoc=%d prior=(%.1f,%.1f) REJECT-JUMP fix=(%.2f,%.2f)%s"
                % (len(obs), len(_pairs), prior[0], prior[1], x, y,
                   confidence_summary(obs, p["classifier"])))
            return
        # Anchor stays FIXED at the initial spawn pose (no re-anchoring). The
        # prior is always initial-anchor + odom/compass motion; landmarks
        # correct the published fix but never move the dead-reckoning baseline.
        state["fix_history"].append((x, y))
        state["fix_history"] = state["fix_history"][-p["smooth_window"]:]
        sx = statistics.median(h[0] for h in state["fix_history"])
        sy = statistics.median(h[1] for h in state["fix_history"])
        _matched = ",".join(lm.name for _o, lm in _pairs)
        rospy.loginfo_throttle(0.5,
            "[diag] obs=%d assoc=%d prior=(%.1f,%.1f) matched=[%s] "
            "FIX x=%.2f y=%.2f rms=%.2f n=%d pub=(%.2f,%.2f)%s"
            % (len(obs), len(_pairs), prior[0], prior[1], _matched, x, y, rms, n, sx, sy,
               confidence_summary(obs, p["classifier"])))
        od = Odometry()
        od.header.stamp = now
        od.header.frame_id = "map"
        od.child_frame_id = "base_link"
        od.pose.pose.position.x = sx
        od.pose.pose.position.y = sy
        od.pose.pose.orientation.w = 1.0
        od.pose.covariance = covariance_for(n, p["base_var"])
        pub.publish(od)
        state["last_pub"] = now
        state["last_pub_xy"] = (x, y)
        state["last_pub_odom"] = odom_synced

    rospy.Subscriber("/odometry/filtered_odom", Odometry, on_odom, queue_size=5)
    rospy.Subscriber("/compass/data", Imu, on_compass, queue_size=5)
    rospy.Subscriber("/navsat/fix", NavSatFix, on_navsat, queue_size=5)
    rospy.Subscriber("/odometry/filtered_map", Odometry, on_map, queue_size=5)
    rospy.Subscriber("/abs_fix_mode", String, on_mode, queue_size=1)
    rospy.Subscriber("/os0_cloud_node/points", PointCloud2, on_cloud, queue_size=1,
                     buff_size=2**24)  # 16 MB: large ~1–2 MB OS0 clouds need buffer > default 64 KB to avoid stale-frame fragmentation
    rospy.spin()


if __name__ == "__main__":
    main()
