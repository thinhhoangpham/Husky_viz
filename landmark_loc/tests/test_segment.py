# landmark_loc/tests/test_segment.py
import numpy as np
from landmark_loc import segment


def test_crop_filters_height_and_range():
    pts = np.array([
        [0.0, 0.0, -5.0],   # below band
        [1.0, 0.0, 0.5],    # keep
        [30.0, 0.0, 0.5],   # out of range
        [2.0, 0.0, 1.9],    # keep
    ])
    out = segment.crop(pts, z_min=0.1, z_max=2.0, max_range=15.0)
    assert out.shape[0] == 2


def test_cluster_separates_two_blobs():
    a = np.random.default_rng(0).normal([0, 0, 0.5], 0.05, size=(60, 3))
    b = np.random.default_rng(1).normal([5, 5, 0.5], 0.05, size=(60, 3))
    clusters = segment.cluster(np.vstack([a, b]),
                               link_dist=0.3, min_pts=10, max_extent=3.0)
    assert len(clusters) == 2
    cents = sorted(c.centroid_xy[0] for c in clusters)
    assert abs(cents[0] - 0.0) < 0.5 and abs(cents[1] - 5.0) < 0.5


def test_cluster_drops_sparse_and_oversized():
    sparse = np.random.default_rng(2).normal([0, 0, 0.5], 0.05, size=(3, 3))
    wall = np.random.default_rng(3).uniform([-5, -5, 0.5], [5, 5, 0.6], size=(200, 3))
    clusters = segment.cluster(np.vstack([sparse, wall]),
                               link_dist=0.3, min_pts=10, max_extent=3.0)
    assert clusters == []


def test_cluster_reports_major_minor_height():
    # a bar 2m long (x), 0.3m wide (y), 0.5m tall (z)
    rng = np.random.default_rng(4)
    bar = np.column_stack([
        rng.uniform(-1.0, 1.0, 200),
        rng.uniform(-0.15, 0.15, 200),
        rng.uniform(0.0, 0.5, 200),
    ])
    c = segment.cluster(bar, link_dist=0.5, min_pts=10, max_extent=4.0)[0]
    assert c.major > 1.7 and c.minor < 0.6 and 0.4 < c.height < 0.6
