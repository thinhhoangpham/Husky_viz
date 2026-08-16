#!/usr/bin/env python3
"""Drop ground returns from the Ouster cloud before the costmap sees them.

WHY THIS EXISTS
---------------
The costmap's ObstacleLayer gates returns by height (`min_obstacle_height`)
measured in the costmap's global_frame -- `odom` for the local costmap, `map`
for the global one. Both EKFs run with `two_d_mode: true`
(husky_control/config/localization.yaml, localization_map.yaml), so the pose
they publish is FLAT: measured live, odom->base_link is z=+0.000 with
roll=+0.00deg pitch=-0.00deg, while the robot is physically sitting on sloped
terrain at world z~3.9.

That tilt is real but unmodelled. When a ground return is transformed into
`odom` through a transform that claims the robot is level, ground on the uphill
side is rotated ABOVE the height gate and marked as a lethal obstacle. Measured
at the lake spawn: 367 lethal cells, nearest 2.37 m away at bearing +-175..180deg
(i.e. straight behind, which is uphill there), with nothing in the world or in
lake_map.pgm at that spot -- the static map has ZERO occupied cells within 5 m of
spawn. They survive /move_base/clear_costmaps because they are re-marked every
cycle, so they are not stale residue.

Park never showed this: its ground has 6.9 mm of relief across the whole world,
so "robot is level" is effectively true there. Lake carries 2.43 m.

WHY FILTER IN THE SENSOR FRAME
------------------------------
This node works in the LIDAR's own frame, where "below the robot" is
unambiguous and no EKF pose is involved -- so the flat-pose bug cannot reach it.
A return is ground if it lies below a cone that opens downward from the sensor:

    z < -(SENSOR_HEIGHT - GROUND_MARGIN) + r * tan(SLOPE_TOLERANCE)

r is horizontal range. The `r * tan()` term is what makes this work on a hill:
it lets the accepted ground surface rise with distance at up to SLOPE_TOLERANCE,
so genuinely sloping terrain is still recognised as ground, while a tree trunk
or a bench -- which rises steeply over a short horizontal run -- is not.

Anything above the cone is passed through untouched, so real obstacles keep
their full 3D shape and the costmap's own gate still applies afterwards.

This does NOT fix the underlying flat-pose problem; it stops that problem from
manufacturing phantom obstacles. Making the EKF publish real pitch/roll (drop
two_d_mode) would be the deeper fix, but that changes pose estimation for
navigation, GPS and the landmark localizer all at once -- far more blast radius
than a costmap false positive warrants.

USAGE
    rosrun landmark_loc ground_filter_node.py            # or python3 <path>
    _input:=/os0_cloud_node/points  _output:=/os0_cloud_node/points_nogroud
    _sensor_height:=0.826 _slope_tolerance_deg:=20 _ground_margin:=0.15

Then point the costmap's observation source at the OUTPUT topic.
"""
import math

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2


class GroundFilter(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input", "/os0_cloud_node/points")
        self.output_topic = rospy.get_param("~output",
                                            "/os0_cloud_node/points_noground")
        # Lidar height above base_link. From sensor_description.urdf the OS1-64
        # is mounted at xyz="0.09 0 0.79" and the sensor's own link adds 0.03618,
        # giving 0.826 -- confirmed live via tf (base_link->os0_lidar z=+0.826).
        self.sensor_height = float(rospy.get_param("~sensor_height", 0.826))
        # How fast the accepted ground surface may rise with range.
        #
        # MEASURED, not guessed. Sampling the live lake cloud away from any
        # obstacle, ground height in base_link over the costmap's marking range
        # is nearly flat:
        #     r=0-1 m  p50 -0.098  max -0.092
        #     r=1-2 m  p50 -0.092  max -0.025
        #     r=2-3 m  p50 -0.076  max +0.150
        #     r=3-4 m  p50 -0.074  max +0.158
        # so the worst ground return sits ~+0.16 m at 4 m, i.e. under 4 deg of
        # apparent rise. 8 deg doubles that for margin and still climbs only
        # 0.14 m per metre.
        #
        # An earlier 20 deg default was a GUESS and it was badly wrong: at r=4 m
        # it put the ground ceiling at +0.78 m, so a real 1 x 1 x 1 m box spawned
        # 4 m ahead was entirely classified as ground and vanished from the
        # filtered cloud. Verified by spawning one. Keep this small -- the cost
        # of too large is a blind robot, which is far worse than a phantom blob.
        self.slope_tol = math.radians(
            float(rospy.get_param("~slope_tolerance_deg", 8.0)))
        # Extra clearance above the nominal ground plane at r=0. Ground measures
        # about -0.09 m near the robot, so 0.15 covers it with room to spare.
        self.ground_margin = float(rospy.get_param("~ground_margin", 0.15))
        # Returns closer than this are dropped outright: at very short range the
        # cone is thin and a single stray return dominates. The Ouster's own
        # min_range is 0.9 m, so this normally does nothing.
        self.min_range = float(rospy.get_param("~min_range", 0.5))

        self.pub = rospy.Publisher(self.output_topic, PointCloud2, queue_size=2)
        self.sub = rospy.Subscriber(self.input_topic, PointCloud2, self.cb,
                                    queue_size=2, buff_size=2 ** 24)
        self.n_in = self.n_out = 0
        self.last_log = rospy.Time.now()
        rospy.loginfo("[ground_filter] %s -> %s  (h=%.3f m, slope<=%.1f deg, "
                      "margin=%.2f m)", self.input_topic, self.output_topic,
                      self.sensor_height, math.degrees(self.slope_tol),
                      self.ground_margin)

    def is_ground(self, x, y, z):
        r = math.hypot(x, y)
        if r < self.min_range:
            return True
        # Cone opening downward from the sensor: ground may rise with range.
        ceiling = -(self.sensor_height - self.ground_margin) + r * math.tan(self.slope_tol)
        return z < ceiling

    def cb(self, msg):
        kept = [p for p in pc2.read_points(msg, field_names=("x", "y", "z"),
                                           skip_nans=True)
                if not self.is_ground(p[0], p[1], p[2])]
        out = pc2.create_cloud_xyz32(msg.header, kept)
        self.pub.publish(out)

        self.n_in += msg.width * msg.height
        self.n_out += len(kept)
        now = rospy.Time.now()
        if (now - self.last_log).to_sec() >= 10.0:
            pct = 100.0 * self.n_out / self.n_in if self.n_in else 0.0
            rospy.loginfo("[ground_filter] kept %d/%d (%.1f%%) over last %ds",
                          self.n_out, self.n_in, pct, 10)
            self.n_in = self.n_out = 0
            self.last_log = now


def main():
    rospy.init_node("ground_filter")
    GroundFilter()
    rospy.spin()


if __name__ == "__main__":
    main()
