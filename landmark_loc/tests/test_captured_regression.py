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
_MUST = {0: "lamp", 4: "lamp", 10: "lamp", 11: "lamp", 14: "lamp",
         12: "bench", 13: "trash_bin_1"}
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
    # OLD: 12/15 unknown. New rule must do much better on this frame.
    assert unknown <= 6, f"{unknown}/15 still unknown"
