#!/usr/bin/env python3
"""Publish a DISPLAY-ONLY TF frame that shows the robot sitting on the terrain
at the terrain's true tilt, for RViz.

WHY: the map EKF runs two_d_mode=true, so base_link is pinned to z=0 with
roll=pitch=0. That is correct for navigation (a skid-steer Husky has no
absolute pitch/roll sensing the EKF should trust for control), but it makes
the lidar scan render as a flat disc in RViz, visually mismatched against the
DTM terrain layer, which carries true world elevations.

This node changes NOTHING about navigation, the EKF, or localization. It is a
pure spectator: it reads /compass/data and /os0_cloud_node/points, and
broadcasts one extra TF frame purely for visualization.

FRAME CONVENTION
-----------------
Child frame (default "base_link_terrain") is a child of `parent_frame`
(default "base_link"). The published transform is:

    rotation    = quaternion built from the MEASURED (roll, pitch), yaw = 0
    translation = (0, 0, dz)

where `dz` is chosen so that, when something is displayed in the child frame,
it appears at the measured ground height beneath the sensor rather than at
base_link's pinned z=0. Concretely:

    dz = -(sensor_height_above_ground)

sensor_height_above_ground is estimated by fitting a plane z = a*x + b*y + c
to a de-rotated (gravity-levelled) annulus of the lidar cloud around the
robot, and taking sensor_height_above_ground = -c (see fit_ground_plane).

So: rotate anything shown in base_link_terrain by the robot's true roll/pitch,
then push it down by the sensor's measured height above the ground directly
below it. A marker/mesh drawn at the origin of base_link_terrain therefore
lands on the real ground, tilted at the real slope -- exactly what Gazebo
shows, without altering base_link or anything the nav stack consumes.

This node subscribes only; it never publishes cmd_vel, odometry, or anything
that could influence navigation, and it never touches Gazebo ground truth --
only /compass/data (IMU) and /os0_cloud_node/points (lidar), both real
sensors.
"""
import math
import statistics
import sys
from collections import deque

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import Imu, PointCloud2

from landmark_loc import derotate


def fit_ground_plane(points):
    """Least-squares fit z = a*x + b*y + c to an (N,3) array of points.

    Returns (a, b, c). Raises ValueError if fewer than 3 points are given
    (an underdetermined fit) -- callers should catch this and reuse the last
    good estimate rather than propagate garbage.
    """
    pts = np.asarray(points, dtype=float)
    if pts.ndim != 2 or pts.shape[0] < 3 or pts.shape[1] < 3:
        raise ValueError("need at least 3 (x, y, z) points to fit a plane")
    a_mat = np.column_stack([pts[:, 0], pts[:, 1], np.ones(len(pts))])
    coeffs, _, _, _ = np.linalg.lstsq(a_mat, pts[:, 2], rcond=None)
    a, b, c = coeffs
    return float(a), float(b), float(c)


def ground_height_from_cloud(pts_xyz, roll, pitch, fit_min_range, fit_max_range,
                             ground_percentile):
    """Sensor height above the local ground plane, from a raw (N,3) lidar
    cloud in the sensor's own (tilted) frame.

    Levels the cloud with the measured (roll, pitch), keeps an annulus
    between fit_min_range and fit_max_range (close-in points are the robot
    body / near-field noise; far points are unreliable and may not be
    ground), keeps the lowest `ground_percentile` of that annulus by height
    (the ground, not overhanging canopy/obstacles), fits a plane, and returns
    -c (the vertical drop from the sensor origin to the fitted plane).

    Raises ValueError (propagated from fit_ground_plane or thrown directly)
    if there are not enough ground points to trust -- callers should catch
    this and reuse the last good value.
    """
    pts = np.asarray(pts_xyz, dtype=float)
    if pts.ndim != 2 or pts.shape[0] == 0:
        raise ValueError("empty cloud")

    leveled = derotate.derotate_cloud(pts, roll, pitch)
    d = np.hypot(leveled[:, 0], leveled[:, 1])
    near = leveled[(d > fit_min_range) & (d < fit_max_range)]
    if len(near) < 3:
        raise ValueError("too few points in the fit annulus")

    thr = np.percentile(near[:, 2], ground_percentile)
    ground = near[near[:, 2] <= thr]
    if len(ground) < 3:
        raise ValueError("too few ground points below the percentile threshold")

    a, b, c = fit_ground_plane(ground)
    return -c


def quat_from_roll_pitch(roll, pitch):
    """(x, y, z, w) quaternion for the given roll (about x) and pitch (about
    y), with yaw = 0. Aerospace convention, matching
    landmark_loc.derotate.roll_pitch_from_quat's inverse."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    # yaw = 0 => cy = 1, sy = 0
    x = sr * cp
    y = cr * sp
    z = -sr * sp
    w = cr * cp
    return x, y, z, w


class RollingMedian(object):
    """Fixed-size rolling window that reports the median of its contents.

    Used to smooth the height estimate frame-to-frame so small per-cloud
    fitting noise does not make the terrain frame visibly jitter.
    """

    def __init__(self, window):
        if window < 1:
            raise ValueError("window must be >= 1")
        self._buf = deque(maxlen=window)

    def push(self, value):
        self._buf.append(float(value))
        return self.value()

    def value(self):
        if not self._buf:
            return None
        return statistics.median(self._buf)

    def __len__(self):
        return len(self._buf)


def cloud_msg_to_xyz(cloud_msg):
    """(N,3) float array of x,y,z from a sensor_msgs/PointCloud2, NaNs
    dropped. Mirrors landmark_loc.localizer_node.cloud_to_array."""
    from sensor_msgs import point_cloud2
    pts = point_cloud2.read_points(
        cloud_msg, field_names=("x", "y", "z"), skip_nans=True)
    return np.array(list(pts), dtype=float)


def main(argv=None):
    rospy.init_node("terrain_frame_publisher", anonymous=True)

    parent_frame = rospy.get_param("~parent_frame", "base_link")
    child_frame = rospy.get_param("~child_frame", "base_link_terrain")
    rate_hz = rospy.get_param("~rate", 10.0)
    fit_min_range = rospy.get_param("~fit_min_range", 1.5)
    fit_max_range = rospy.get_param("~fit_max_range", 8.0)
    ground_percentile = rospy.get_param("~ground_percentile", 25)
    smooth_window = rospy.get_param("~smooth_window", 5)

    if rate_hz <= 0:
        rospy.logerr("~rate must be positive, got %s", rate_hz)
        return 1

    state = {"roll": None, "pitch": None}
    height_smoother = RollingMedian(smooth_window)
    last_good_height = [None]  # mutable cell so the cloud callback can write it

    def on_compass(msg):
        q = msg.orientation
        roll, pitch = derotate.roll_pitch_from_quat(q.x, q.y, q.z, q.w)
        state["roll"] = roll
        state["pitch"] = pitch

    def on_cloud(msg):
        if state["roll"] is None or state["pitch"] is None:
            return  # no orientation yet -- nothing to level the cloud with
        try:
            pts = cloud_msg_to_xyz(msg)
            height = ground_height_from_cloud(
                pts, state["roll"], state["pitch"],
                fit_min_range, fit_max_range, ground_percentile)
        except ValueError as exc:
            rospy.logwarn_throttle(
                5.0, "terrain frame: ground fit failed (%s), reusing last "
                     "good height", exc)
            return
        last_good_height[0] = height_smoother.push(height)

    rospy.Subscriber("/compass/data", Imu, on_compass, queue_size=5)
    rospy.Subscriber("/os0_cloud_node/points", PointCloud2, on_cloud,
                     queue_size=1)

    broadcaster = tf2_ros.TransformBroadcaster()
    rate = rospy.Rate(rate_hz)
    rospy.loginfo("terrain_frame_publisher: broadcasting %s -> %s at %.1f Hz "
                  "(display-only, does not affect navigation)",
                  parent_frame, child_frame, rate_hz)

    while not rospy.is_shutdown():
        roll, pitch = state["roll"], state["pitch"]
        height = last_good_height[0]
        if roll is not None and pitch is not None and height is not None:
            rospy.loginfo_throttle(
                1.0, "terrain frame: roll=%.3f rad pitch=%.3f rad "
                     "sensor_height_above_ground=%.3f m",
                roll, pitch, height)

            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = parent_frame
            t.child_frame_id = child_frame
            t.transform.translation.x = 0.0
            t.transform.translation.y = 0.0
            t.transform.translation.z = -height
            qx, qy, qz, qw = quat_from_roll_pitch(roll, pitch)
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            broadcaster.sendTransform(t)
        # else: no compass yet, or no good ground fit yet -- publish nothing
        # rather than a frame with fabricated values.

        try:
            rate.sleep()
        except rospy.ROSInterruptException:
            break
    return 0


if __name__ == "__main__":
    sys.exit(main())
