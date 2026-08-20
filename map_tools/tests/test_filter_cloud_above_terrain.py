"""Tests for the terrain-relative cloud filter's pure logic (no roscore).

The module lives in scripts/, which is not a package, so it is loaded by path
-- same approach as test_relay_costmap_z.py.
"""
import importlib.util
import os

import numpy as np
import pytest

_SCRIPT = os.path.join(os.path.dirname(__file__), "..", "..", "scripts",
                       "filter_cloud_above_terrain.py")

rospy = pytest.importorskip("rospy",
                            reason="ROS not on this interpreter's path")


def _load():
    spec = importlib.util.spec_from_file_location("filter_cloud_above_terrain",
                                                  _SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


fcat = _load()

# A 4x4 grid of 1 m cells whose origin is NEGATIVE on both axes, so the
# floor-vs-truncate distinction is actually exercised.
RES = 1.0
OX = -2.0
OY = -2.0


def _flat(height, shape=(4, 4)):
    return np.full(shape, height, dtype=np.float32)


# --- the core: height is measured against the point's OWN terrain cell -------

def test_ground_on_flat_terrain_reads_about_zero():
    """Ground lying on terrain at 4.47 m reads ~0, not 4.47."""
    dtm = _flat(4.47)
    pts = np.array([[0.0, 0.0, 4.47], [1.5, -1.5, 4.47]])
    h = fcat.height_above_ground(pts, dtm, RES, OX, OY)
    assert h == pytest.approx([0.0, 0.0], abs=1e-6)


def test_obstacle_reads_its_own_height_whatever_the_terrain_elevation():
    """THE CORE PROPERTY, and exactly what an absolute-z gate gets wrong.

    The same 0.5 m obstacle must read 0.5 m whether it stands on terrain at
    0 m or at 4.47 m. An absolute-z band would see 0.5 in one case and 4.97
    in the other and could not accept both.
    """
    pts = np.array([[0.0, 0.0, 0.0]])
    for elevation in (0.0, 3.5, 4.47, 5.93, -12.0):
        dtm = _flat(elevation)
        obstacle = pts + np.array([0.0, 0.0, elevation + 0.5])
        h = fcat.height_above_ground(obstacle, dtm, RES, OX, OY)
        assert h[0] == pytest.approx(0.5, abs=1e-5), elevation


def test_sloped_terrain_ground_still_reads_zero():
    """The crescent case: ground on a slope must NOT look like an obstacle.

    Terrain rises 0.5 m per cell across x. Ground points follow it. Every
    one must read ~0 -- under an absolute-z gate they span 4.0..5.5 m and the
    upslope ones cross any fixed threshold, which is the false-lethal arc.
    """
    dtm = np.array([[4.0, 4.5, 5.0, 5.5]] * 4, dtype=np.float32)
    xs = np.array([-1.5, -0.5, 0.5, 1.5])
    pts = np.column_stack([xs, np.zeros(4), np.array([4.0, 4.5, 5.0, 5.5])])
    h = fcat.height_above_ground(pts, dtm, RES, OX, OY)
    assert h == pytest.approx([0.0] * 4, abs=1e-6)

    # ...and the same slope with a 0.6 m obstacle on the highest cell reads 0.6.
    obstacle = np.array([[1.5, 0.0, 5.5 + 0.6]])
    assert fcat.height_above_ground(obstacle, dtm, RES, OX, OY)[0] == \
        pytest.approx(0.6, abs=1e-6)


def test_row_is_y_and_column_is_x():
    """Row 0 = lowest y (the DTM's documented layout). A transposed lookup
    would pass every symmetric test above, so it is pinned explicitly."""
    dtm = np.zeros((4, 4), dtype=np.float32)
    dtm[3, 0] = 9.0  # highest y, lowest x
    h = fcat.height_above_ground(np.array([[-1.5, 1.5, 9.0]]), dtm, RES, OX, OY)
    assert h[0] == pytest.approx(0.0)


# --- no-data handling -------------------------------------------------------

def test_points_off_the_grid_read_nan():
    dtm = _flat(4.0)
    pts = np.array([[-99.0, 0.0, 4.0], [0.0, 99.0, 4.0], [0.0, 0.0, 4.0]])
    h = fcat.height_above_ground(pts, dtm, RES, OX, OY)
    assert np.isnan(h[0]) and np.isnan(h[1])
    assert h[2] == pytest.approx(0.0)


def test_points_just_outside_the_negative_edge_are_off_grid_not_folded():
    """floor vs int() truncation: x = -2.5 is one cell BELOW the origin.

    Truncating would give column 0 and silently return a height from the
    wrong cell instead of reporting no data.
    """
    dtm = _flat(4.0)
    assert np.isnan(fcat.height_above_ground(
        np.array([[-2.5, 0.0, 4.0]]), dtm, RES, OX, OY)[0])


def test_nan_terrain_cell_reads_nan_not_a_height_of_zero():
    dtm = _flat(4.0)
    dtm[2, 2] = np.nan
    assert np.isnan(fcat.height_above_ground(
        np.array([[0.5, 0.5, 4.0]]), dtm, RES, OX, OY)[0])


def test_nan_point_coordinate_reads_nan():
    dtm = _flat(4.0)
    pts = np.array([[np.nan, 0.0, 4.0], [0.0, 0.0, np.nan]])
    assert np.all(np.isnan(fcat.height_above_ground(pts, dtm, RES, OX, OY)))


def test_empty_input_returns_empty():
    dtm = _flat(4.0)
    assert fcat.height_above_ground(np.zeros((0, 3)), dtm, RES, OX, OY).shape \
        == (0,)


def test_wrong_shape_is_rejected():
    with pytest.raises(ValueError):
        fcat.height_above_ground(np.zeros((4, 2)), _flat(4.0), RES, OX, OY)


# --- band_mask --------------------------------------------------------------

def test_band_keeps_only_heights_inside_the_band():
    h = np.array([0.09, 0.39, 0.40, 1.2, 3.0, 3.01, 7.2])
    keep = fcat.band_mask(h, 0.40, 3.00)
    assert list(keep) == [False, False, True, True, True, False, False]


def test_band_drops_off_dtm_points_by_default():
    """Off-DTM is open water or off-mesh void. There is no ground reference,
    so no honest height test exists; the default is to drop."""
    keep = fcat.band_mask(np.array([np.nan, 1.0]), 0.40, 3.00)
    assert list(keep) == [False, True]


def test_band_can_keep_off_dtm_points_when_asked():
    keep = fcat.band_mask(np.array([np.nan, 1.0]), 0.40, 3.00,
                          keep_off_dtm=True)
    assert list(keep) == [True, True]


def test_band_on_empty_input():
    assert fcat.band_mask(np.zeros(0), 0.4, 3.0).shape == (0,)


# --- measured percentiles from the live lake run ----------------------------

def test_default_band_separates_the_measured_ground_from_the_measured_objects():
    """Terrain-relative percentiles measured in-sim (lake, 3.9 deg slope,
    n=16894): ground p1..p50 = +0.083..+0.090, objects from p75 = +1.233,
    canopy p95 = +7.191."""
    ground = np.array([0.083, 0.086, 0.088, 0.090])
    objects = np.array([1.233, 2.0])
    canopy = np.array([7.191, 9.780])
    lo, hi = fcat.DEFAULT_MIN_HEIGHT, fcat.DEFAULT_MAX_HEIGHT
    assert not fcat.band_mask(ground, lo, hi).any()
    assert fcat.band_mask(objects, lo, hi).all()
    assert not fcat.band_mask(canopy, lo, hi).any()


def test_the_same_returns_cannot_be_separated_by_an_absolute_z_band():
    """Why this node exists: in ABSOLUTE z the measured ground (4.237..4.562)
    and the measured objects (from 6.542) are separable only for THIS terrain
    elevation. Shift the robot onto ground 2 m lower -- inside the lake's own
    2.42 m relief -- and object returns land squarely in the old ground band,
    so no fixed absolute band works world-wide."""
    ground_abs = np.array([4.237, 4.422, 4.533, 4.562])
    lo, hi = 4.6, 6.4  # a band tuned perfectly for this one spot
    assert not fcat.band_mask(ground_abs, lo, hi).any()
    object_on_lower_ground = np.array([6.542 - 2.0])
    assert not fcat.band_mask(object_on_lower_ground, lo, hi).any(), \
        "a real obstacle on lower terrain is missed by the absolute band"


# --- transform helpers ------------------------------------------------------

def test_identity_transform_is_a_no_op():
    m = np.eye(4)
    pts = np.array([[1.0, 2.0, 3.0]])
    assert fcat.apply_transform(pts, m) == pytest.approx(pts)


def test_transform_applies_rotation_then_translation():
    # 90 deg about z, then +10 in x.
    m = np.eye(4)
    m[:3, :3] = [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    m[:3, 3] = [10.0, 0.0, 0.0]
    out = fcat.apply_transform(np.array([[1.0, 0.0, 0.5]]), m)
    assert out[0] == pytest.approx([10.0, 1.0, 0.5])


def test_transform_of_empty_points():
    assert fcat.apply_transform(np.zeros((0, 3)), np.eye(4)).shape == (0, 3)


def test_quaternion_to_matrix_matches_a_known_rotation():
    from geometry_msgs.msg import TransformStamped
    tr = TransformStamped()
    # 90 deg about z
    tr.transform.rotation.z = np.sin(np.pi / 4)
    tr.transform.rotation.w = np.cos(np.pi / 4)
    tr.transform.translation.x = 2.0
    tr.transform.translation.z = 4.47
    m = fcat.transform_to_matrix(tr)
    out = fcat.apply_transform(np.array([[1.0, 0.0, 0.0]]), m)
    assert out[0] == pytest.approx([2.0, 1.0, 4.47], abs=1e-9)


def test_zero_quaternion_is_rejected():
    from geometry_msgs.msg import TransformStamped
    tr = TransformStamped()  # all-zero rotation, an invalid quaternion
    with pytest.raises(ValueError):
        fcat.transform_to_matrix(tr)


# --- PointCloud2 read / rewrite --------------------------------------------

def _make_cloud(xyz, extra=None, frame="os0_lidar"):
    """A PointCloud2 with float32 x,y,z plus a float32 'intensity' field."""
    from sensor_msgs.msg import PointCloud2, PointField
    xyz = np.asarray(xyz, dtype=np.float32)
    n = xyz.shape[0]
    if extra is None:
        extra = np.arange(n, dtype=np.float32)
    msg = PointCloud2()
    msg.header.frame_id = frame
    msg.header.stamp = rospy.Time(123, 456)
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12,
                   datatype=PointField.FLOAT32, count=1),
    ]
    msg.point_step = 16
    msg.height = 1
    msg.width = n
    msg.row_step = msg.point_step * n
    msg.is_bigendian = False
    msg.is_dense = True
    rec = np.column_stack([xyz, np.asarray(extra, dtype=np.float32)])
    msg.data = rec.astype(np.float32).tobytes()
    return msg


def test_cloud_xyz_view_reads_the_coordinates():
    xyz = np.array([[1.0, 2.0, 3.0], [-4.0, 5.5, 6.25]])
    got = fcat.cloud_xyz_view(_make_cloud(xyz))
    assert got == pytest.approx(xyz)


def test_cloud_xyz_view_on_an_empty_cloud():
    assert fcat.cloud_xyz_view(_make_cloud(np.zeros((0, 3)))).shape == (0, 3)


def test_cloud_xyz_view_rejects_a_cloud_without_float32_xyz():
    msg = _make_cloud(np.array([[1.0, 2.0, 3.0]]))
    msg.fields = [f for f in msg.fields if f.name != "z"]
    assert fcat.cloud_xyz_view(msg) is None


def test_filter_cloud_keeps_the_selected_points_and_all_their_fields():
    xyz = np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0], [3.0, 0.0, 0.0]])
    msg = _make_cloud(xyz, extra=[10.0, 20.0, 30.0])
    out = fcat.filter_cloud(msg, np.array([True, False, True]))
    assert out.width == 2 and out.height == 1
    assert out.row_step == out.point_step * 2
    rec = np.frombuffer(out.data, dtype=np.float32).reshape(-1, 4)
    assert rec[:, 0] == pytest.approx([1.0, 3.0])
    # the non-xyz field rode along untouched
    assert rec[:, 3] == pytest.approx([10.0, 30.0])


def test_filter_cloud_preserves_stamp_and_the_ORIGINAL_frame():
    """The points stay in the SENSOR frame. Publishing them in the map frame
    would make costmap_2d transform them a second time."""
    msg = _make_cloud(np.array([[1.0, 0.0, 0.0]]), frame="os0_lidar")
    out = fcat.filter_cloud(msg, np.array([True]))
    assert out.header.frame_id == "os0_lidar"
    assert out.header.stamp == msg.header.stamp


def test_filter_cloud_dropping_everything_yields_an_empty_cloud():
    msg = _make_cloud(np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]))
    out = fcat.filter_cloud(msg, np.array([False, False]))
    assert out.width == 0 and out.data == b""


def test_filter_cloud_on_an_empty_input_does_not_crash():
    msg = _make_cloud(np.zeros((0, 3)))
    out = fcat.filter_cloud(msg, np.zeros(0, dtype=bool))
    assert out.width == 0


def test_filter_cloud_does_not_mutate_the_input():
    msg = _make_cloud(np.array([[1.0, 0.0, 0.0], [2.0, 0.0, 0.0]]))
    before = msg.data
    fcat.filter_cloud(msg, np.array([True, False]))
    assert msg.data == before and msg.width == 2


# --- end to end over the pure functions ------------------------------------

def test_end_to_end_ground_dropped_obstacle_kept_on_sloped_terrain():
    """The whole chain on the crescent case, with no ROS master.

    Terrain slopes 0.5 m per cell. Four ground returns follow it (must be
    dropped) and one 1.5 m obstacle stands on the highest cell (must be kept).
    """
    dtm = np.array([[4.0, 4.5, 5.0, 5.5]] * 4, dtype=np.float32)
    xs = np.array([-1.5, -0.5, 0.5, 1.5])
    ground = np.column_stack([xs, np.zeros(4), [4.0, 4.5, 5.0, 5.5]])
    obstacle = np.array([[1.5, 0.0, 5.5 + 1.5]])
    pts = np.vstack([ground, obstacle])

    msg = _make_cloud(pts)
    xyz = fcat.cloud_xyz_view(msg)
    h = fcat.height_above_ground(xyz, dtm, RES, OX, OY)
    keep = fcat.band_mask(h, fcat.DEFAULT_MIN_HEIGHT, fcat.DEFAULT_MAX_HEIGHT)
    out = fcat.filter_cloud(msg, keep)

    assert out.width == 1
    rec = np.frombuffer(out.data, dtype=np.float32).reshape(-1, 4)
    assert rec[0, 2] == pytest.approx(7.0)
