#!/usr/bin/env python3
"""Filter a lidar cloud by height above the TERRAIN, not by absolute z.

WHY THIS NODE EXISTS
--------------------
`costmap_2d`'s obstacle gate is a scalar comparison: a point is an obstacle if
`min_obstacle_height <= z <= max_obstacle_height`, where z is measured in the
costmap frame. That test assumes the ground is a horizontal plane at a known,
constant height. On sloped or elevated terrain it is simply wrong, and there is
no hook in costmap_2d to make it terrain-relative -- ObstacleLayer stores two
doubles and compares them, so the decision has to be made BEFORE the cloud
reaches the costmap. Hence a filter node in front of it.

MEASURED, lake world, robot parked on a 3.9 deg slope at terrain z = 4.47 m
(n = 16894 returns, terrain sampled per-point from maps/lake_dtm.npy):

    height above ITS OWN terrain cell   p1 +0.083  p50 +0.090  p75 +1.233  p95 +7.191
    ABSOLUTE z (what costmap_2d sees)   p1 +4.237  p50 +4.562  p75 +6.542  p95 +11.683

Terrain-relative, the ground is a 7 MILLIMETRE band (+0.083..+0.090) and the
nearest object starts at +1.2 m -- a separation so wide the threshold is barely
a choice. In absolute z the same ground smears over 4.24..4.56 m, and that
smear MOVES as the robot drives across relief, so no fixed pair of numbers can
separate ground from object world-wide. The observed symptom was a crescent of
false lethal cells 2-5 m from the robot (measured once: 6355 lethal cells all
within 6 m while the nearest real mapped landmark was 7.88 m away -- every one
of them false), which is precisely sloped ground crossing an absolute-z
threshold.

THE BAND, AND WHY
-----------------
Default 0.40 .. 3.00 m above terrain.

  * Lower bound 0.40 m. Ground tops out at +0.090 m with a 7 mm spread, and the
    lowest real object return sits near +1.2 m. 0.40 is ~4.4x above the top of
    ground and ~3x below the bottom of objects -- roughly the middle of an
    unusually wide dead zone, in log terms. It is deliberately NOT marginal:
    picking 0.15 would "work" on this measurement but leaves only 6 cm of
    headroom for DTM interpolation error, pose error, and suspension pitch.
  * Upper bound 3.00 m. Above the Husky (0.39 m tall) with room for overhanging
    structure it genuinely cannot pass under, and well below the canopy returns
    at +7.2 m (p95) which must NOT be marked -- the robot drives under trees.

NOT SPOOF-RESISTANT -- READ THIS BEFORE RELYING ON IT
-----------------------------------------------------
The terrain lookup is indexed by the point's map-frame (x, y), which is derived
from the robot's own pose estimate. If that pose is wrong -- drift, or a
deliberate spoof -- this node reads the WRONG terrain cells and its height test
degrades accordingly. It is a geometry correction for the costmap, and nothing
more. It is not a detector, it does not validate pose, and it must not be cited
as a defence against pose corruption. The defences for that are the landmark
localizer and the drift monitors.

Usage (no launch file; see RUN-MAP-NAV.md):

    python3 scripts/filter_cloud_above_terrain.py _world:=lake
    python3 scripts/filter_cloud_above_terrain.py _world:=park

Params (all private, all optional):
    ~world            park | lake   (default park) -- picks maps/<world>_dtm.npy
    ~dtm_path         explicit .npy path, overrides ~world
    ~in_topic         default /os0_cloud_node/points
    ~out_topic        default /os0_cloud_node/points_above_terrain
    ~min_height       metres above terrain, default 0.40
    ~max_height       metres above terrain, default 3.00
    ~map_frame        frame the DTM is expressed in, default map
    ~keep_off_dtm     keep points with no terrain data under them, default False
    ~tf_timeout       seconds to wait for the transform, default 0.1
"""
import os
import sys

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2

DEFAULT_IN_TOPIC = "/os0_cloud_node/points"
DEFAULT_OUT_TOPIC = "/os0_cloud_node/points_above_terrain"

# Ground sits at ~+0.09 m above its own terrain cell and objects start at
# ~+1.2 m (measured, see module docstring). These are the middle of that gap,
# not its edges.
DEFAULT_MIN_HEIGHT = 0.40
DEFAULT_MAX_HEIGHT = 3.00


def height_above_ground(points, dtm, resolution, origin_x, origin_y):
    """Height of each point above the terrain cell beneath ITS OWN (x, y).

    `points` is (N, 3) of map-frame x, y, z. `dtm` is the float array from
    maps/<world>_dtm.npy: [row][col], row 0 = LOWEST y, NaN = no mesh coverage
    (NOT a height of zero). Returns a float64 (N,) array.

    NaN is returned for any point whose (x, y) falls outside the grid or lands
    on a NaN cell -- "no terrain data here", which is a distinct answer from
    "height zero" and is left for the caller to act on. NaN in a point's own
    x/y/z propagates to NaN for the same reason.

    Nearest-cell lookup, not bilinear. The DTM is 0.25 m and terrain relief is
    smooth at that scale, so interpolation would move the answer by far less
    than the 0.31 m of margin the default band carries on its tight side; the
    simpler lookup is not the limiting error term.
    """
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return np.zeros(0, dtype=np.float64)
    if pts.ndim != 2 or pts.shape[1] < 3:
        raise ValueError("points must be (N, 3), got shape %r" % (pts.shape,))

    dtm = np.asarray(dtm)
    n_rows, n_cols = dtm.shape
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]

    # np.floor, not int() truncation: truncation rounds toward zero, which for
    # negative offsets folds the cell below the origin onto cell 0 and silently
    # reads the wrong terrain on the negative side of the map.
    with np.errstate(invalid="ignore"):
        col = np.floor((x - origin_x) / resolution)
        row = np.floor((y - origin_y) / resolution)

    inside = (np.isfinite(col) & np.isfinite(row)
              & (col >= 0) & (col < n_cols)
              & (row >= 0) & (row < n_rows))

    heights = np.full(pts.shape[0], np.nan, dtype=np.float64)
    if not inside.any():
        return heights

    ri = row[inside].astype(np.intp)
    ci = col[inside].astype(np.intp)
    terrain = dtm[ri, ci].astype(np.float64)
    # NaN terrain propagates through the subtraction on its own, so no mask is
    # needed here: no-data cells come out NaN, which is the intended answer.
    heights[inside] = z[inside] - terrain
    return heights


def band_mask(heights, min_height, max_height, keep_off_dtm=False):
    """Boolean keep-mask for `heights` (output of height_above_ground).

    Points inside [min_height, max_height] are kept. NaN heights mean "no
    terrain data under this point" and are governed by `keep_off_dtm`.
    """
    h = np.asarray(heights, dtype=np.float64)
    known = np.isfinite(h)
    keep = np.zeros(h.shape, dtype=bool)
    keep[known] = (h[known] >= min_height) & (h[known] <= max_height)
    if keep_off_dtm:
        keep |= ~known
    return keep


def load_dtm_meta(npy_path):
    """(resolution, origin_x, origin_y) from the .yaml beside `npy_path`.

    Parsed line-by-line rather than with PyYAML: the file is generated by
    map_tools/extract_dtm.py and is strictly `key: value`, matching how
    scripts/publish_dtm_cloud.py reads the same file.
    """
    yaml_path = os.path.splitext(npy_path)[0] + ".yaml"
    if not os.path.exists(yaml_path):
        raise IOError(
            "missing %s -- the DTM .npy carries no geometry of its own, so "
            "the .yaml written beside it by map_tools/extract_dtm.py is "
            "required to locate its cells." % yaml_path)
    needed = ("resolution", "origin_x", "origin_y")
    meta = {}
    with open(yaml_path) as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#") or ":" not in line:
                continue
            key, _, val = line.partition(":")
            key = key.strip()
            if key in needed:
                try:
                    meta[key] = float(val.strip())
                except ValueError:
                    pass
    missing = [k for k in needed if k not in meta]
    if missing:
        raise ValueError("%s is missing %s" % (yaml_path, ", ".join(missing)))
    return meta["resolution"], meta["origin_x"], meta["origin_y"]


def resolve_dtm_path(world, dtm_path=None, maps_dir=None):
    """Absolute path to the DTM .npy to read. Explicit `dtm_path` wins."""
    if dtm_path:
        return os.path.abspath(os.path.expanduser(dtm_path))
    if maps_dir is None:
        maps_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "maps")
    return os.path.abspath(os.path.join(maps_dir, "%s_dtm.npy" % world))


def _xyz_offsets(msg):
    """Byte offsets of the x, y, z fields, or None if any is missing/not float32.

    Only float32 xyz is supported, which is what every ROS lidar driver emits
    (and what the OS0 gazebo plugin emits here). Anything else is reported
    rather than silently misread as float32 garbage.
    """
    from sensor_msgs.msg import PointField
    found = {}
    for f in msg.fields:
        if f.name in ("x", "y", "z"):
            if f.datatype != PointField.FLOAT32 or f.count != 1:
                return None
            found[f.name] = f.offset
    if len(found) != 3:
        return None
    return found["x"], found["y"], found["z"]


def cloud_xyz_view(msg):
    """(N, 3) float32 xyz read straight out of the message buffer.

    A strided VIEW over `msg.data`, not a copy and not a Python generator:
    `sensor_msgs.point_cloud2.read_points` yields one tuple per point in
    Python, which at ~17k points and 10 Hz is far too slow to sit in the
    lidar path. Returns None if the cloud is not float32 xyz.
    """
    offs = _xyz_offsets(msg)
    if offs is None:
        return None
    raw = np.frombuffer(msg.data, dtype=np.uint8)
    n_points = raw.size // msg.point_step
    if n_points == 0:
        return np.zeros((0, 3), dtype=np.float32)
    # Trim any trailing padding so the reshape is exact.
    raw = raw[:n_points * msg.point_step].reshape(n_points, msg.point_step)
    ox, oy, oz = offs
    cols = [raw[:, o:o + 4].copy().view(np.float32).reshape(-1) for o in (ox, oy, oz)]
    return np.column_stack(cols)


def filter_cloud(msg, keep):
    """A copy of PointCloud2 `msg` holding only the points where `keep` is True.

    The point RECORDS are copied byte-for-byte, so every field the sensor
    emitted (intensity, ring, timestamp, ...) survives untouched -- nothing is
    re-encoded and no field layout is assumed beyond the point stride.

    header (stamp AND frame_id) is carried over unchanged: the points are still
    in the SENSOR frame. They must not be published in the map frame, because
    costmap_2d applies its own sensor->costmap transform and would otherwise
    transform them a second time.

    The result is an unordered cloud (height 1): removing arbitrary points
    destroys the row/column structure of an organised cloud, so claiming to
    still be organised would be a lie about the geometry.
    """
    out = PointCloud2()
    out.header = msg.header
    out.fields = msg.fields
    out.is_bigendian = msg.is_bigendian
    out.point_step = msg.point_step

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    n_points = raw.size // msg.point_step
    if n_points:
        raw = raw[:n_points * msg.point_step].reshape(n_points, msg.point_step)
        kept = raw[np.asarray(keep, dtype=bool)]
    else:
        kept = np.zeros((0, msg.point_step), dtype=np.uint8)

    out.height = 1
    out.width = int(kept.shape[0])
    out.row_step = out.point_step * out.width
    out.data = kept.tobytes()
    # is_dense TRUE: every surviving point passed a finite height test, so by
    # construction none of them has a NaN coordinate.
    out.is_dense = True
    return out


def transform_to_matrix(tr):
    """4x4 homogeneous matrix from a geometry_msgs/TransformStamped.

    Quaternion -> rotation written out directly rather than pulling in
    tf.transformations, so the pure-numpy core stays importable without a
    running ROS stack.
    """
    t = tr.transform.translation
    q = tr.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n <= 0.0:
        raise ValueError("zero-norm quaternion in transform")
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    m = np.eye(4, dtype=np.float64)
    m[0, 0] = 1.0 - (yy + zz)
    m[0, 1] = xy - wz
    m[0, 2] = xz + wy
    m[1, 0] = xy + wz
    m[1, 1] = 1.0 - (xx + zz)
    m[1, 2] = yz - wx
    m[2, 0] = xz - wy
    m[2, 1] = yz + wx
    m[2, 2] = 1.0 - (xx + yy)
    m[0, 3], m[1, 3], m[2, 3] = t.x, t.y, t.z
    return m


def apply_transform(points, matrix):
    """(N, 3) points through a 4x4 homogeneous matrix."""
    pts = np.asarray(points, dtype=np.float64)
    if pts.size == 0:
        return np.zeros((0, 3), dtype=np.float64)
    return pts.dot(matrix[:3, :3].T) + matrix[:3, 3]


def main(argv=None):
    argv = sys.argv if argv is None else argv
    rospy.init_node("filter_cloud_above_terrain", anonymous=True)

    world = rospy.get_param("~world", "park")
    dtm_path_param = rospy.get_param("~dtm_path", "")
    in_topic = rospy.get_param("~in_topic", DEFAULT_IN_TOPIC)
    out_topic = rospy.get_param("~out_topic", DEFAULT_OUT_TOPIC)
    min_height = float(rospy.get_param("~min_height", DEFAULT_MIN_HEIGHT))
    max_height = float(rospy.get_param("~max_height", DEFAULT_MAX_HEIGHT))
    map_frame = rospy.get_param("~map_frame", "map")
    keep_off_dtm = bool(rospy.get_param("~keep_off_dtm", False))
    tf_timeout = float(rospy.get_param("~tf_timeout", 0.1))

    if out_topic == in_topic:
        rospy.logfatal("[terrain_filter] ~out_topic must differ from ~in_topic "
                       "(%s) -- republishing onto the raw lidar topic would "
                       "feed this node its own output.", in_topic)
        return 1
    if min_height > max_height:
        rospy.logfatal("[terrain_filter] ~min_height (%.3f) is above "
                       "~max_height (%.3f); nothing could ever pass.",
                       min_height, max_height)
        return 1

    path = resolve_dtm_path(world, dtm_path_param)
    if not os.path.exists(path):
        rospy.logfatal("[terrain_filter] no DTM at %s -- set ~world to a world "
                       "that has maps/<world>_dtm.npy, or pass ~dtm_path.",
                       path)
        return 1
    dtm = np.load(path)
    resolution, origin_x, origin_y = load_dtm_meta(path)
    rospy.loginfo("[terrain_filter] DTM %s (%dx%d @ %.3f m), band %.2f..%.2f m "
                  "above terrain, off-DTM points are %s",
                  path, dtm.shape[1], dtm.shape[0], resolution,
                  min_height, max_height,
                  "KEPT" if keep_off_dtm else "DROPPED")

    tf_buf = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buf)

    pub = rospy.Publisher(out_topic, PointCloud2, queue_size=1)

    def on_cloud(msg):
        xyz = cloud_xyz_view(msg)
        if xyz is None:
            rospy.logwarn_throttle(
                10.0, "[terrain_filter] %s is not float32 x/y/z -- passing "
                "through UNFILTERED.", in_topic)
            pub.publish(msg)
            return
        if xyz.shape[0] == 0:
            pub.publish(filter_cloud(msg, np.zeros(0, dtype=bool)))
            return

        try:
            tr = tf_buf.lookup_transform(map_frame, msg.header.frame_id,
                                         msg.header.stamp,
                                         rospy.Duration(tf_timeout))
        except Exception as exc:
            # PASS THROUGH, do not drop. A missing transform means we cannot
            # judge height, and publishing an empty cloud would tell the
            # costmap "the world ahead is clear" -- the one failure mode that
            # can drive the robot into something. Unfiltered data is noisy;
            # absent data is unsafe.
            rospy.logwarn_throttle(
                5.0, "[terrain_filter] no %s <- %s transform (%s) -- passing "
                "the cloud through UNFILTERED.", map_frame,
                msg.header.frame_id, exc)
            pub.publish(msg)
            return

        pts_map = apply_transform(xyz, transform_to_matrix(tr))
        heights = height_above_ground(pts_map, dtm, resolution,
                                      origin_x, origin_y)
        keep = band_mask(heights, min_height, max_height, keep_off_dtm)
        pub.publish(filter_cloud(msg, keep))

    rospy.Subscriber(in_topic, PointCloud2, on_cloud, queue_size=1,
                     buff_size=2 ** 24)
    rospy.loginfo("[terrain_filter] %s -> %s", in_topic, out_topic)
    rospy.spin()
    return 0


if __name__ == "__main__":
    sys.exit(main())
