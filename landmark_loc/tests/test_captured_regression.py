import json
import os
import numpy as np
import pytest
from landmark_loc.segment import Cluster, _pca_extents
from landmark_loc import classify

_HERE = os.path.dirname(__file__)
_NPZ = os.path.join(_HERE, "fixtures", "captured_clusters.npz")
_JSON = os.path.join(_HERE, "fixtures", "captured_clusters.json")


def _load():
    arrs = np.load(_NPZ)
    manifest = {m["i"]: m for m in json.load(open(_JSON))}
    out = {}
    for key in arrs.files:
        i = int(key[1:])
        pts = arrs[key].astype(float)
        major, minor = _pca_extents(pts[:, :2])
        out[i] = Cluster(points=pts,
                         centroid_xy=(float(pts[:, 0].mean()), float(pts[:, 1].mean())),
                         major=float(major), minor=float(minor),
                         height=float(pts[:, 2].max() - pts[:, 2].min()))
    return out, manifest


# clusters the OLD classifier dropped that MUST now classify correctly
# [13] (would have been trash_bin_1) is EXCLUDED from _MUST: its captured
# height is 1.679 m vs the trash_bin mesh's 1.041 m because the segmenter
# fused overhanging tree foliage onto the bin cluster (verified from the raw
# points -- a clean vertical scan line from z=0.09-1.60, then the footprint
# spreads sideways sharply above z=1.69). That's a segmentation artifact, not
# a classifier failure, and no clean bin capture exists this session to pin
# a height threshold against. [13] is allowed to read unknown.
_MUST = {0: "lamp", 4: "lamp", 10: "lamp", 11: "lamp", 14: "lamp",
         12: "bench"}
# fragments/ground blobs that must NOT become a furniture phantom
_NO_PHANTOM = (1, 3, 9)
_FURNITURE = {"lamp", "trash_bin_1", "bench", "garden_table"}


@pytest.mark.parametrize("i,expected", sorted(_MUST.items()))
def test_captured_cluster_labeled_correctly(i, expected):
    clusters, _ = _load()
    assert classify.classify_cluster(clusters[i]) == expected


@pytest.mark.parametrize("i", _NO_PHANTOM)
def test_captured_fragment_not_phantom_furniture(i):
    clusters, _ = _load()
    got = classify.classify_cluster(clusters[i])
    assert got not in _FURNITURE, f"cluster {i} became phantom {got}"


def test_unknown_rate_dropped():
    clusters, _ = _load()
    labels = [classify.classify_cluster(c) for c in clusters.values()]
    unknown = sum(1 for l in labels if l == "unknown")
    # OLD: 12/15 unknown. New rule recovers 5 lamps + 1 bench; [13] (would-be
    # bin) stays unknown due to foliage contamination (see _MUST comment
    # above), not a classifier gap. Measured exact count: 7/15.
    assert unknown == 7, f"{unknown}/15 still unknown (expected exactly 7)"
