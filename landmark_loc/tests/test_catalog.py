# landmark_loc/tests/test_catalog.py
import math
import tempfile
import os
from landmark_loc import catalog
from landmark_loc.catalog import load, MapLandmark


def test_load_maps_names_to_identities(tmp_path):
    p = tmp_path / "objects.yaml"
    p.write_text(
        "bench: {x: 1.0, y: 2.0}\n"
        "bench_clone_1: {x: 3.0, y: 4.0}\n"
        "lamp: {x: -5.0, y: 6.0}\n"
        "garden_table_clone_2: {x: 7.0, y: 8.0}\n"
        "trash_bin_1: {x: 9.0, y: 0.0}\n")
    lms = catalog.load(str(p))
    ids = sorted(l.identity for l in lms)
    assert ids == ["bench", "bench", "garden_table", "lamp", "trash_bin_1"]


def test_gate_keeps_only_in_range_and_fov():
    lms = [
        catalog.MapLandmark("near", "bench", 2.0, 0.0),    # 2m ahead
        catalog.MapLandmark("far", "lamp", 40.0, 0.0),     # out of range
        catalog.MapLandmark("behind", "lamp", -2.0, 0.0),  # behind (out of fov)
    ]
    kept = catalog.gate(lms, prior_xyz=(0.0, 0.0, 0.0),
                        max_range=15.0, fov_halfwidth=math.pi / 2)
    names = {l.name for l in kept}
    assert names == {"near"}


def test_gate_respects_prior_yaw():
    lm = [catalog.MapLandmark("left", "bench", 0.0, 2.0)]  # 2m to +y (world)
    # robot facing +y (yaw=pi/2): the landmark is straight ahead
    kept = catalog.gate(lm, prior_xyz=(0.0, 0.0, math.pi / 2),
                       max_range=15.0, fov_halfwidth=math.pi / 4)
    assert len(kept) == 1


def test_tree8_names_load_as_tree_identity(tmp_path):
    objects = tmp_path / "objects.yaml"
    objects.write_text(
        "bench_1: {x: 1.0, y: 2.0}\n"
        "tree_8: {x: 10.0, y: 20.0}\n"
        "tree_8_clone_3: {x: 11.0, y: 21.0}\n")
    lms = catalog.load(str(objects))
    ids = {lm.name: lm.identity for lm in lms}
    assert ids["tree_8"] == "tree"
    assert ids["tree_8_clone_3"] == "tree"
    assert ids["bench_1"] == "bench"


def test_load_reads_yaw_when_present():
    with tempfile.NamedTemporaryFile("w", suffix=".yaml", delete=False) as f:
        f.write("bench: {x: 1.0, y: 2.0, yaw: 0.5}\n")
        f.write("lamp_clone: {x: 3.0, y: 4.0}\n")   # no yaw -> None
        path = f.name
    lms = {l.name: l for l in load(path)}
    os.unlink(path)
    assert abs(lms["bench"].yaw - 0.5) < 1e-9
    assert lms["lamp_clone"].yaw is None
