#!/usr/bin/env python3
"""RViz demo: why terrain tilt eliminates a duplicate constellation.

Draws BOTH tied constellation hypotheses (A and B) in the map frame, each with:
  - its tree triangle and centroid
  - a TILE showing the attitude the DTM predicts at that hypothesis's (x,y,yaw)
  - a second TILE showing the attitude the robot ACTUALLY measures
  - a text readout of the attitude residual and the verdict

The wrong hypothesis renders with its two tiles visibly disagreeing; the correct
one renders with them flush. This is a VISUALIZATION of an analysis result --
it does not modify the localizer, which today rejects the wrong hypothesis via
its 15 m prior sanity check, not via tilt.

Pose truth for the measured attitude comes from /compass/data (absolute, does
not drift). No Gazebo ground truth is read.

Publishes latched MarkerArray on ~topic (default /tilt_disambiguation).
"""
import math

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Point, Quaternion
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray

GROUP_A = ["tree_8_clone_clone", "tree_8_clone_clone_2",
           "tree_8_clone_clone_clone_clone_1"]
GROUP_B = ["tree_8_clone_clone_1", "tree_8_clone_clone_clone_clone_3",
           "tree_8_clone_clone_clone_clone_4"]
YAW_A, YAW_B = 0.0, math.radians(76.0)

TRUE_RGB = (0.12, 0.66, 0.82)     # cool  - consistent
WRONG_RGB = (0.85, 0.31, 0.18)    # warm  - eliminated
MEAS_RGB = (0.95, 0.95, 0.92)     # measured attitude
LIFT = 1.2                        # metres above terrain for the tiles


class Dtm(object):
    def __init__(self, npy, yml):
        self.z = np.load(npy)
        m = yaml.safe_load(open(yml))
        self.res = m["resolution"]; self.ox = m["origin_x"]; self.oy = m["origin_y"]

    def z_at(self, x, y):
        i = int(round((x - self.ox) / self.res)); j = int(round((y - self.oy) / self.res))
        if 0 <= j < self.z.shape[0] and 0 <= i < self.z.shape[1]:
            v = self.z[j, i]
            if np.isfinite(v):
                return float(v)
        return None

    def plane_gradient(self, x, y, half_m=0.6):
        r = int(round(half_m / self.res))
        i = int(round((x - self.ox) / self.res)); j = int(round((y - self.oy) / self.res))
        j0, j1 = max(0, j - r), min(self.z.shape[0], j + r + 1)
        i0, i1 = max(0, i - r), min(self.z.shape[1], i + r + 1)
        w = self.z[j0:j1, i0:i1]
        ok = np.isfinite(w)
        if ok.sum() < 6:
            return None
        jj, ii = np.nonzero(ok)
        A = np.c_[ii * self.res, jj * self.res, np.ones(int(ok.sum()))]
        sol, *_ = np.linalg.lstsq(A, w[ok], rcond=None)
        return float(sol[0]), float(sol[1])


def predict_rp(gx, gy, yaw):
    g_fwd = gx * math.cos(yaw) + gy * math.sin(yaw)
    g_left = -gx * math.sin(yaw) + gy * math.cos(yaw)
    return math.atan(g_left), -math.atan(g_fwd)


def _base(ns, mid, frame="map"):
    m = Marker()
    m.header.frame_id = frame
    m.ns = ns; m.id = mid
    m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    return m


def tile(ns, mid, x, y, z, roll, pitch, yaw, rgb, alpha, size=2.6):
    m = _base(ns, mid)
    m.type = Marker.CUBE
    q = quaternion_from_euler(roll, pitch, yaw)
    m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z
    m.pose.orientation = Quaternion(*q)
    m.scale.x = size; m.scale.y = size * 0.72; m.scale.z = 0.07
    m.color.r, m.color.g, m.color.b = rgb; m.color.a = alpha
    return m


def text(ns, mid, x, y, z, s, rgb, h=0.85):
    m = _base(ns, mid)
    m.type = Marker.TEXT_VIEW_FACING
    m.pose.position.x = x; m.pose.position.y = y; m.pose.position.z = z
    m.scale.z = h
    m.color.r, m.color.g, m.color.b = rgb; m.color.a = 1.0
    m.text = s
    return m


def main():
    rospy.init_node("show_tilt_disambiguation")
    repo = rospy.get_param("~repo", "/home/thinh/Documents/Husky_viz")
    topic = rospy.get_param("~topic", "/tilt_disambiguation")
    objs = yaml.safe_load(open("%s/maps/lake_objects.yaml" % repo))
    dtm = Dtm("%s/maps/lake_dtm.npy" % repo, "%s/maps/lake_dtm.yaml" % repo)

    # Measured attitude. Live from /compass/data when the sim is up; otherwise
    # supplied as ~roll_deg/~pitch_deg so the demo layer runs with NO simulator
    # (a bare roscore is enough -- the DTM and landmark map are files on disk).
    roll_p = rospy.get_param("~roll_deg", None)
    pitch_p = rospy.get_param("~pitch_deg", None)
    if roll_p is not None and pitch_p is not None:
        m_roll, m_pitch = math.radians(float(roll_p)), math.radians(float(pitch_p))
        rospy.loginfo("[tilt-viz] measured attitude from PARAMS: "
                      "roll %.3f deg pitch %.3f deg (offline mode)",
                      math.degrees(m_roll), math.degrees(m_pitch))
    else:
        rospy.loginfo("[tilt-viz] waiting for /compass/data ...")
        try:
            imu = rospy.wait_for_message("/compass/data", Imu, timeout=10)
        except Exception:
            rospy.logfatal("[tilt-viz] no /compass/data and no ~roll_deg/~pitch_deg. "
                           "With the sim down, pass the recorded attitude, e.g. "
                           "_roll_deg:=0.143 _pitch_deg:=2.469")
            return
        o = imu.orientation
        m_roll, m_pitch, _ = euler_from_quaternion((o.x, o.y, o.z, o.w))
        rospy.loginfo("[tilt-viz] measured roll %.3f deg pitch %.3f deg (live)",
                      math.degrees(m_roll), math.degrees(m_pitch))

    # With the sim down nothing publishes the `map` frame, and RViz's Fixed
    # Frame is `map` -- every display errors and the markers never draw. Publish
    # a static map->world identity so the layer stands on its own.
    if rospy.get_param("~publish_map_frame", False):
        import tf2_ros
        from geometry_msgs.msg import TransformStamped
        _static = tf2_ros.StaticTransformBroadcaster()
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "tilt_demo_origin"
        t.transform.rotation.w = 1.0
        _static.sendTransform(t)
        rospy.loginfo("[tilt-viz] publishing static map frame (offline mode)")

    pub = rospy.Publisher(topic, MarkerArray, queue_size=1, latch=True)
    arr = MarkerArray(); mid = 0

    for key, names, yaw_h in (("A", GROUP_A, YAW_A), ("B", GROUP_B, YAW_B)):
        pts = [(objs[n]["x"], objs[n]["y"]) for n in names]
        cx = sum(p[0] for p in pts) / 3.0
        cy = sum(p[1] for p in pts) / 3.0
        gz = dtm.z_at(cx, cy) or 4.5
        g = dtm.plane_gradient(cx, cy)
        p_roll, p_pitch = predict_rp(g[0], g[1], yaw_h)
        resid = math.degrees(math.hypot(m_roll - p_roll, m_pitch - p_pitch))
        correct = resid < 3.0
        rgb = TRUE_RGB if correct else WRONG_RGB

        # constellation outline
        ln = _base("constellation", mid); mid += 1
        ln.type = Marker.LINE_STRIP; ln.scale.x = 0.3
        ln.color.r, ln.color.g, ln.color.b = rgb; ln.color.a = 0.85
        for x, y in pts + [pts[0]]:
            ln.points.append(Point(x, y, (dtm.z_at(x, y) or gz) + 0.5))
        arr.markers.append(ln)
        for x, y in pts:
            s = _base("trees", mid); mid += 1
            s.type = Marker.SPHERE
            s.pose.position.x = x; s.pose.position.y = y
            s.pose.position.z = (dtm.z_at(x, y) or gz) + 0.5
            s.scale.x = s.scale.y = s.scale.z = 1.0
            s.color.r, s.color.g, s.color.b = rgb; s.color.a = 0.9
            arr.markers.append(s)

        # PREDICTED attitude tile (what the DTM says at this hypothesis)
        arr.markers.append(tile("predicted", mid, cx, cy, gz + LIFT,
                                p_roll, p_pitch, yaw_h, rgb, 0.95)); mid += 1
        # MEASURED attitude tile, same spot, floating just above
        arr.markers.append(tile("measured", mid, cx, cy, gz + LIFT + 0.55,
                                m_roll, m_pitch, yaw_h, MEAS_RGB, 0.55)); mid += 1

        verdict = "CONSISTENT" if correct else "ELIMINATED"
        arr.markers.append(text("verdict", mid, cx, cy, gz + LIFT + 2.6,
                                "%s  %s\nresidual %.2f deg" % (key, verdict, resid),
                                rgb)); mid += 1
        rospy.loginfo("[tilt-viz] %s: predicted roll %.2f pitch %.2f -> residual %.2f deg (%s)",
                      key, math.degrees(p_roll), math.degrees(p_pitch), resid, verdict)

    pub.publish(arr)
    rospy.loginfo("[tilt-viz] published %d markers on %s (latched)", len(arr.markers), topic)
    rospy.spin()


if __name__ == "__main__":
    main()
