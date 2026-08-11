"""Segment a lidar point array into candidate landmark clusters.

Pure geometry on (N,3) numpy arrays in the lidar frame. No ROS. Clustering is a
grid-accelerated Euclidean single-link grouping; adequate for the sparse,
well-separated park furniture at ~15 m range and cheap enough for 10 Hz.
"""
from dataclasses import dataclass
import numpy as np


@dataclass
class Cluster:
    points: np.ndarray
    centroid_xy: tuple
    major: float
    minor: float
    height: float


def crop(points, z_min, z_max, max_range):
    if len(points) == 0:
        return points
    z = points[:, 2]
    rng = np.hypot(points[:, 0], points[:, 1])
    keep = (z >= z_min) & (z <= z_max) & (rng <= max_range)
    return points[keep]


def _grid_clusters(xy, link_dist):
    """Union-find over points whose cells are within link_dist (grid-bucketed)."""
    if len(xy) == 0:
        return []
    cell = np.floor(xy / link_dist).astype(int)
    buckets = {}
    for i, (cx, cy) in enumerate(map(tuple, cell)):
        buckets.setdefault((cx, cy), []).append(i)
    parent = list(range(len(xy)))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    def union(a, b):
        ra, rb = find(a), find(b)
        if ra != rb:
            parent[rb] = ra

    d2 = link_dist * link_dist
    for (cx, cy), idxs in buckets.items():
        neigh = []
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                neigh.extend(buckets.get((cx + dx, cy + dy), []))
        for i in idxs:
            for j in neigh:
                if j <= i:
                    continue
                if np.sum((xy[i] - xy[j]) ** 2) <= d2:
                    union(i, j)
    groups = {}
    for i in range(len(xy)):
        groups.setdefault(find(i), []).append(i)
    return list(groups.values())


def _pca_extents(xy):
    c = xy.mean(axis=0)
    centered = xy - c
    cov = np.cov(centered.T) if len(xy) > 1 else np.zeros((2, 2))
    evals, evecs = np.linalg.eigh(cov)
    proj = centered @ evecs
    spans = proj.max(axis=0) - proj.min(axis=0)
    major, minor = max(spans), min(spans)
    return major, minor


def cluster(points, link_dist, min_pts, max_extent):
    if len(points) == 0:
        return []
    xy = points[:, :2]
    out = []
    for idxs in _grid_clusters(xy, link_dist):
        if len(idxs) < min_pts:
            continue
        pts = points[idxs]
        cxy = pts[:, :2]
        major, minor = _pca_extents(cxy)
        if major > max_extent:
            continue
        height = float(pts[:, 2].max() - pts[:, 2].min())
        out.append(Cluster(
            points=pts,
            centroid_xy=(float(cxy[:, 0].mean()), float(cxy[:, 1].mean())),
            major=float(major), minor=float(minor), height=height))
    return out
