"""Unit tests for the camera detector's decode, letterbox and NMS.

These tests carry more weight than usual: there is NO offline corpus of park
imagery in either repo (the sibling's `output/vision_dataset/` is gitignored and
gone), so nothing but synthetic tensors pins the decode maths until the sim
runs. They therefore assert exact pixel coordinates and exact detection counts,
not merely that "something came back".

Nothing here imports rospy, onnxruntime or needs a master: `decode`, `letterbox`
and `nms` are pure numpy, which is exactly why they live in a ROS-free module.
"""
import numpy as np
import pytest

from landmark_loc.camera_detect import (
    CLASS_IDENTITIES, INPUT_SIZE, YOLO_CLASS_NAMES, decode, letterbox, nms)
from map_tools.park_types import PARK_TYPES

N_ANCHORS = 8400
N_CLASSES = len(YOLO_CLASS_NAMES)


def make_raw(planted, n_anchors=N_ANCHORS):
    """Build a [1, 9, 8400] head with boxes planted at known anchor slots.

    `planted` is [(anchor_idx, cls_idx, score, cx, cy, w, h)] in 640-space.
    Every other anchor is left at score 0, i.e. firmly below any threshold.
    """
    raw = np.zeros((1, 4 + N_CLASSES, n_anchors), dtype=np.float32)
    for idx, cls_idx, score, cx, cy, w, h in planted:
        raw[0, 0, idx] = cx
        raw[0, 1, idx] = cy
        raw[0, 2, idx] = w
        raw[0, 3, idx] = h
        raw[0, 4 + cls_idx, idx] = score
    return raw


# --------------------------------------------------------------------------
# class name -> registry identity
# --------------------------------------------------------------------------

def test_every_yolo_class_resolves_to_a_registry_identity():
    known = {t.identity for t in PARK_TYPES}
    assert len(CLASS_IDENTITIES) == len(YOLO_CLASS_NAMES)
    for name, identity in zip(YOLO_CLASS_NAMES, CLASS_IDENTITIES):
        assert identity in known, (
            "%r -> %r is not a park_types identity" % (name, identity))


def test_trash_bin_alias_is_applied():
    # The one name that is NOT an identity verbatim: trained label 'trash_bin',
    # registry identity 'trash_bin_1'. Downstream must see repo vocabulary.
    assert CLASS_IDENTITIES[YOLO_CLASS_NAMES.index("trash_bin")] == "trash_bin_1"


def test_other_four_classes_pass_through_unaliased():
    for name in ("bench", "lamp", "tree", "garden_table"):
        assert CLASS_IDENTITIES[YOLO_CLASS_NAMES.index(name)] == name


# --------------------------------------------------------------------------
# decode: exact coordinates
# --------------------------------------------------------------------------

def test_decode_square_image_recovers_exact_box():
    """640x640 source: scale 1, no padding, so 640-space == pixel space."""
    raw = make_raw([(0, 0, 0.9, 100.0, 200.0, 40.0, 60.0)])
    dets = decode(raw, scale=1.0, pad_x=0, pad_y=0, orig_wh=(640, 640),
                  conf=0.30)
    assert len(dets) == 1
    identity, conf, x1, y1, x2, y2 = dets[0]
    assert identity == "bench"
    assert conf == pytest.approx(0.9)
    # cx,cy,w,h = 100,200,40,60 -> xyxy = 80,170,120,230
    assert (x1, y1, x2, y2) == pytest.approx((80.0, 170.0, 120.0, 230.0))


def test_decode_undoes_letterbox_on_non_square_image():
    """512x384 (the real camera size) exercises the pad in BOTH axes.

    letterbox scale = 640/512 = 1.25, so the image renders 640x480 and is
    padded (640-480)//2 = 80 rows top and bottom; pad_x = 0.
    """
    src = np.zeros((384, 512, 3), dtype=np.uint8)
    _, scale, pad_x, pad_y = letterbox(src, INPUT_SIZE)
    assert scale == pytest.approx(1.25)
    assert (pad_x, pad_y) == (0, 80)

    # A box occupying source pixels x 100..200, y 50..150. Forward-transform it
    # into 640-space by hand, plant it, and require decode to return the source
    # pixels exactly.
    sx1, sx2 = 100.0, 200.0        # source columns
    sy1, sy2 = 50.0, 150.0         # source rows
    gx1, gx2 = sx1 * scale + pad_x, sx2 * scale + pad_x
    gy1, gy2 = sy1 * scale + pad_y, sy2 * scale + pad_y
    cx, cy = (gx1 + gx2) / 2.0, (gy1 + gy2) / 2.0
    w, h = gx2 - gx1, gy2 - gy1

    # Sanity-check the hand-computed forward transform: pad_y=80 must shift the
    # rows but not the columns, so the test cannot pass by self-consistency.
    assert (gx1, gx2) == pytest.approx((125.0, 250.0))
    assert (gy1, gy2) == pytest.approx((142.5, 267.5))

    raw = make_raw([(7, 2, 0.77, cx, cy, w, h)])
    dets = decode(raw, scale, pad_x, pad_y, orig_wh=(512, 384), conf=0.30)
    assert len(dets) == 1
    identity, conf, x1, y1, x2, y2 = dets[0]
    assert identity == "tree"
    assert (x1, y1, x2, y2) == pytest.approx((100.0, 50.0, 200.0, 150.0),
                                             abs=1e-4)


def test_decode_clips_boxes_to_image_bounds():
    """A box straddling the frame edge must be clipped, never negative."""
    # cx,cy,w,h in 640-space, scale 1: xyxy = -20,-30,80,70 before clipping.
    raw = make_raw([(3, 1, 0.8, 30.0, 20.0, 100.0, 100.0)])
    dets = decode(raw, scale=1.0, pad_x=0, pad_y=0, orig_wh=(512, 384),
                  conf=0.30)
    assert len(dets) == 1
    _, _, x1, y1, x2, y2 = dets[0]
    assert (x1, y1) == pytest.approx((0.0, 0.0))
    assert (x2, y2) == pytest.approx((80.0, 70.0))
    assert 0.0 <= x1 <= x2 <= 512.0
    assert 0.0 <= y1 <= y2 <= 384.0


def test_decode_clips_boxes_to_the_far_edges_too():
    """The opposite overflow: a box running off the right/bottom of the frame.

    The network predicts in 640-space and has no idea the source is 512x384, so
    it readily emits boxes past the far edges; unclipped they would index
    outside the image when drawn or cropped.
    """
    # scale 1: xyxy = 462,334,662,534, i.e. past both far edges of a 512x384.
    raw = make_raw([(4, 0, 0.8, 562.0, 434.0, 200.0, 200.0)])
    dets = decode(raw, scale=1.0, pad_x=0, pad_y=0, orig_wh=(512, 384),
                  conf=0.30)
    assert len(dets) == 1
    _, _, x1, y1, x2, y2 = dets[0]
    assert (x1, y1) == pytest.approx((462.0, 334.0))
    assert (x2, y2) == pytest.approx((512.0, 384.0))     # clipped, not 662/534


def test_decode_returns_multiple_distinct_boxes_with_right_classes():
    raw = make_raw([
        (10, 0, 0.90, 100.0, 100.0, 20.0, 20.0),   # bench
        (20, 1, 0.80, 300.0, 100.0, 20.0, 20.0),   # lamp
        (30, 4, 0.70, 500.0, 400.0, 20.0, 20.0),   # garden_table
        (40, 3, 0.60, 100.0, 400.0, 20.0, 20.0),   # trash_bin -> trash_bin_1
    ])
    dets = decode(raw, 1.0, 0, 0, (640, 640), conf=0.30)
    assert len(dets) == 4
    # decode sorts by descending confidence.
    assert [d[0] for d in dets] == ["bench", "lamp", "garden_table",
                                    "trash_bin_1"]
    assert [round(d[1], 2) for d in dets] == [0.90, 0.80, 0.70, 0.60]


# --------------------------------------------------------------------------
# the reject path
# --------------------------------------------------------------------------

def test_all_below_threshold_yields_zero_detections():
    """The reject path -- required by the detector contract, not optional."""
    raw = make_raw([
        (5, 0, 0.29, 100.0, 100.0, 20.0, 20.0),
        (6, 2, 0.10, 300.0, 300.0, 40.0, 40.0),
    ])
    assert decode(raw, 1.0, 0, 0, (640, 640), conf=0.30) == []


def test_empty_tensor_yields_zero_detections():
    assert decode(make_raw([]), 1.0, 0, 0, (640, 640), conf=0.30) == []


def test_threshold_is_inclusive_and_only_drops_what_is_below():
    planted = [(1, 0, 0.31, 100.0, 100.0, 20.0, 20.0),
               (2, 1, 0.29, 300.0, 300.0, 20.0, 20.0)]
    dets = decode(make_raw(planted), 1.0, 0, 0, (640, 640), conf=0.30)
    assert len(dets) == 1
    assert dets[0][0] == "bench"


def test_raising_conf_progressively_rejects_more():
    planted = [(1, 0, 0.95, 100.0, 100.0, 20.0, 20.0),
               (2, 1, 0.60, 300.0, 300.0, 20.0, 20.0),
               (3, 2, 0.35, 500.0, 500.0, 20.0, 20.0)]
    raw = make_raw(planted)
    assert len(decode(raw, 1.0, 0, 0, (640, 640), conf=0.30)) == 3
    assert len(decode(raw, 1.0, 0, 0, (640, 640), conf=0.50)) == 2
    assert len(decode(raw, 1.0, 0, 0, (640, 640), conf=0.90)) == 1
    assert len(decode(raw, 1.0, 0, 0, (640, 640), conf=0.99)) == 0


# --------------------------------------------------------------------------
# class-wise NMS
# --------------------------------------------------------------------------

def test_nms_suppresses_duplicates_of_the_same_class():
    # Two near-identical benches: only the higher-scoring one survives.
    raw = make_raw([(1, 0, 0.90, 100.0, 100.0, 50.0, 50.0),
                    (2, 0, 0.70, 102.0, 101.0, 50.0, 50.0)])
    dets = decode(raw, 1.0, 0, 0, (640, 640), conf=0.30, iou=0.45)
    assert len(dets) == 1
    assert dets[0][0] == "bench"
    assert dets[0][1] == pytest.approx(0.90)


def test_nms_is_class_wise_overlapping_different_classes_both_survive():
    """A lamp in front of a tree overlaps heavily but is a separate object.

    Identical boxes, different classes: cross-class suppression would wrongly
    delete one, so BOTH must come back.
    """
    raw = make_raw([(1, 1, 0.90, 100.0, 100.0, 50.0, 50.0),   # lamp
                    (2, 2, 0.85, 100.0, 100.0, 50.0, 50.0)])  # tree
    dets = decode(raw, 1.0, 0, 0, (640, 640), conf=0.30, iou=0.45)
    assert len(dets) == 2
    assert sorted(d[0] for d in dets) == ["lamp", "tree"]


def test_nms_keeps_same_class_boxes_that_barely_overlap():
    # Two benches side by side, sharing only a sliver: both are real.
    raw = make_raw([(1, 0, 0.90, 100.0, 100.0, 50.0, 50.0),
                    (2, 0, 0.80, 145.0, 100.0, 50.0, 50.0)])
    dets = decode(raw, 1.0, 0, 0, (640, 640), conf=0.30, iou=0.45)
    assert len(dets) == 2


def test_nms_helper_ranks_by_score_not_input_order():
    boxes = [[0, 0, 10, 10], [1, 1, 11, 11], [100, 100, 110, 110]]
    keep = nms(boxes, [0.5, 0.9, 0.4], iou_thresh=0.45)
    assert keep[0] == 1                # highest score first
    assert sorted(keep) == [1, 2]      # box 0 suppressed by box 1
    assert nms([], [], 0.45) == []


# --------------------------------------------------------------------------
# letterbox
# --------------------------------------------------------------------------

def test_letterbox_output_shape_and_normalisation():
    src = np.full((384, 512, 3), 255, dtype=np.uint8)
    batch, scale, pad_x, pad_y = letterbox(src, INPUT_SIZE)
    assert batch.shape == (1, 3, INPUT_SIZE, INPUT_SIZE)
    assert batch.dtype == np.float32
    assert 0.0 <= batch.min() and batch.max() <= 1.0
    assert batch.max() == pytest.approx(1.0)     # 255 -> 1.0


def test_letterbox_swaps_bgr_to_rgb():
    # A pure-blue BGR image (255, 0, 0) must land in the LAST channel of the
    # CHW RGB tensor. Sample the image centre, which is never padding.
    src = np.zeros((384, 512, 3), dtype=np.uint8)
    src[:, :, 0] = 255
    batch, _, _, _ = letterbox(src, INPUT_SIZE)
    c = INPUT_SIZE // 2
    assert batch[0, 0, c, c] == pytest.approx(0.0)   # R
    assert batch[0, 2, c, c] == pytest.approx(1.0)   # B


def test_letterbox_pads_a_tall_image_horizontally():
    src = np.zeros((640, 320, 3), dtype=np.uint8)
    _, scale, pad_x, pad_y = letterbox(src, INPUT_SIZE)
    assert scale == pytest.approx(1.0)
    assert (pad_x, pad_y) == (160, 0)


def test_letterbox_roundtrip_through_decode_for_a_tall_image():
    """The other padding axis: a box in a tall image maps back exactly."""
    src = np.zeros((640, 320, 3), dtype=np.uint8)
    _, scale, pad_x, pad_y = letterbox(src, INPUT_SIZE)

    sx1, sy1, sx2, sy2 = 40.0, 100.0, 240.0, 500.0
    gx1, gx2 = sx1 * scale + pad_x, sx2 * scale + pad_x
    gy1, gy2 = sy1 * scale + pad_y, sy2 * scale + pad_y
    raw = make_raw([(11, 4, 0.88, (gx1 + gx2) / 2.0, (gy1 + gy2) / 2.0,
                     gx2 - gx1, gy2 - gy1)])

    dets = decode(raw, scale, pad_x, pad_y, orig_wh=(320, 640), conf=0.30)
    assert len(dets) == 1
    identity, _, x1, y1, x2, y2 = dets[0]
    assert identity == "garden_table"
    assert (x1, y1, x2, y2) == pytest.approx((sx1, sy1, sx2, sy2), abs=1e-4)


def test_decode_undoes_pad_before_scale_in_both_axes():
    """Pins the ORDER of the letterbox undo: subtract pad, THEN divide by scale.

    This needs an image where pad_x AND pad_y are both nonzero and scale != 1 --
    otherwise `(v - pad)/scale` and `v/scale - pad` give the same answer and the
    ordering bug hides. A 200x100 source scales by 3.2 to 640x320, padding 160
    rows; pad_x is forced nonzero here to exercise the x axis too.
    """
    src = np.zeros((100, 200, 3), dtype=np.uint8)
    _, scale, _, pad_y = letterbox(src, INPUT_SIZE)
    assert scale == pytest.approx(3.2)
    assert pad_y == 160

    pad_x = 24                       # forced, so both axes have pad and scale
    sx1, sx2, sy1, sy2 = 20.0, 120.0, 10.0, 60.0
    gx1, gx2 = sx1 * scale + pad_x, sx2 * scale + pad_x
    gy1, gy2 = sy1 * scale + pad_y, sy2 * scale + pad_y
    raw = make_raw([(9, 1, 0.8, (gx1 + gx2) / 2.0, (gy1 + gy2) / 2.0,
                     gx2 - gx1, gy2 - gy1)])

    dets = decode(raw, scale, pad_x, pad_y, orig_wh=(200, 100), conf=0.30)
    assert len(dets) == 1
    _, _, x1, y1, x2, y2 = dets[0]
    assert (x1, y1, x2, y2) == pytest.approx((sx1, sy1, sx2, sy2), abs=1e-4)


def test_letterbox_rejects_an_empty_image():
    with pytest.raises(ValueError):
        letterbox(np.zeros((0, 10, 3), dtype=np.uint8), INPUT_SIZE)


def test_decode_rejects_a_wrong_shaped_head():
    with pytest.raises(ValueError):
        decode(np.zeros((1, 7, 100), dtype=np.float32), 1.0, 0, 0, (640, 640))
