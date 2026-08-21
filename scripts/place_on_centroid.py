#!/usr/bin/env python3
"""Place the Husky on one of the duplicate-constellation centroids, ALIGNED to
the local terrain slope, for the tilt disambiguation demo (RUN-TILT-DEMO.md).

    python3 scripts/place_on_centroid.py A|B

WHY ALIGNED, AND WHY THE TWIST IS ZEROED
----------------------------------------
Dropping the robot flat onto a slope makes it slide and roll -- a 6.3 deg site
tipped it to 46.8 deg. So the roll/pitch are set from the DTM plane at the
target and the model is placed exactly at ground + chassis clearance.

`set_model_state` also KEEPS the model's previous velocity: re-placing an
already-tumbling robot let it keep tumbling from the new pose (measured 84 deg
tilt). The twist is therefore zeroed explicitly, and the state is set twice
either side of a physics tick.

GROUND-TRUTH RULE: this only ever SETS pose. It never reads a Gazebo pose as
data -- repositioning between test runs is the sanctioned use.
"""
import math
import sys

import numpy as np
import rospy
import yaml
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
from robot_localization.srv import SetPose
from sensor_msgs.msg import Imu
from tf.transformations import quaternion_from_euler, euler_from_quaternion

REPO = "/home/thinh/Documents/Husky_viz"
CLEARANCE = 0.1323          # base_link above ground, from TF
CENTROIDS = {"A": (-12.540, -20.623, 0.0),
             "B": (46.078, -11.246, math.radians(76.0))}


def plane_gradient(z, res, ox, oy, x, y, half_m=0.6):
    r = int(round(half_m / res))
    i = int(round((x - ox) / res)); j = int(round((y - oy) / res))
    j0, j1 = max(0, j - r), min(z.shape[0], j + r + 1)
    i0, i1 = max(0, i - r), min(z.shape[1], i + r + 1)
    w = z[j0:j1, i0:i1]
    ok = np.isfinite(w)
    if ok.sum() < 6:
        return None
    jj, ii = np.nonzero(ok)
    A = np.c_[ii * res, jj * res, np.ones(int(ok.sum()))]
    sol, *_ = np.linalg.lstsq(A, w[ok], rcond=None)
    return float(sol[0]), float(sol[1])


def main():
    key = (sys.argv[1].upper() if len(sys.argv) > 1 else "A")
    if key not in CENTROIDS:
        print("usage: place_on_centroid.py A|B"); return 1
    x, y, yaw = CENTROIDS[key]

    rospy.init_node("place_on_centroid", anonymous=True)
    z = np.load("%s/maps/lake_dtm.npy" % REPO)
    m = yaml.safe_load(open("%s/maps/lake_dtm.yaml" % REPO))
    res, ox, oy = m["resolution"], m["origin_x"], m["origin_y"]

    g = plane_gradient(z, res, ox, oy, x, y)
    if g is None:
        rospy.logfatal("no terrain under centroid %s", key); return 1
    g_fwd = g[0] * math.cos(yaw) + g[1] * math.sin(yaw)
    g_left = -g[0] * math.sin(yaw) + g[1] * math.cos(yaw)
    roll, pitch = math.atan(g_left), -math.atan(g_fwd)

    i = int(round((x - ox) / res)); j = int(round((y - oy) / res))
    gz = float(z[j, i])

    q = quaternion_from_euler(roll, pitch, yaw)
    ms = ModelState()
    ms.model_name = "husky"; ms.reference_frame = "world"
    ms.pose.position.x = x; ms.pose.position.y = y; ms.pose.position.z = gz + CLEARANCE
    ms.pose.orientation.x, ms.pose.orientation.y = q[0], q[1]
    ms.pose.orientation.z, ms.pose.orientation.w = q[2], q[3]
    ms.twist.linear.x = ms.twist.linear.y = ms.twist.linear.z = 0.0
    ms.twist.angular.x = ms.twist.angular.y = ms.twist.angular.z = 0.0

    rospy.wait_for_service("/gazebo/set_model_state", timeout=20)
    sms = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(1.0)

    sms(ms); rospy.sleep(0.3); sms(ms)     # twice, either side of a tick
    for _ in range(5):
        pub.publish(Twist()); rospy.sleep(0.05)
    rospy.loginfo("[place] centroid %s: (%.3f, %.3f, %.3f) roll %.2f pitch %.2f yaw %.1f deg",
                  key, x, y, gz + CLEARANCE, math.degrees(roll),
                  math.degrees(pitch), math.degrees(yaw))
    rospy.sleep(5.0)

    # Re-sync the EKF: after a teleport the filter still believes the old pose.
    try:
        imu = rospy.wait_for_message("/compass/data", Imu, timeout=10)
        o = imu.orientation
        mr, mp, my = euler_from_quaternion((o.x, o.y, o.z, o.w))
        rospy.wait_for_service("/set_pose", timeout=10)
        sp = rospy.ServiceProxy("/set_pose", SetPose)
        pc = PoseWithCovarianceStamped()
        pc.header.frame_id = "map"; pc.header.stamp = rospy.Time.now()
        pc.pose.pose.position.x = x; pc.pose.pose.position.y = y; pc.pose.pose.position.z = gz
        qq = quaternion_from_euler(mr, mp, my)
        pc.pose.pose.orientation.x, pc.pose.pose.orientation.y = qq[0], qq[1]
        pc.pose.pose.orientation.z, pc.pose.pose.orientation.w = qq[2], qq[3]
        pc.pose.covariance = [0.0] * 36
        for k in (0, 7, 14, 21, 28, 35):
            pc.pose.covariance[k] = 0.05
        sp(pc)
        rospy.loginfo("[place] EKF re-synced; settled tilt %.2f deg",
                      math.degrees(math.hypot(mr, mp)))
    except Exception as e:
        rospy.logwarn("[place] EKF re-sync skipped (%s)", e)
    return 0


if __name__ == "__main__":
    sys.exit(main())
