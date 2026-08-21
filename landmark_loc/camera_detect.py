"""YOLOv8 park-object detector: raw ONNX inference plus decode, ROS-free.

WHY THIS MODULE IMPORTS NO rospy
--------------------------------
Everything here is pure numpy + onnxruntime so the decode maths can be unit
tested with no master, no sim and no camera -- the pattern
`landmark_loc/abs_fix_selector.py` already uses (its ROS imports live inside
`main()`). The ROS wrapper is `scripts/camera_detect_node.py`.

WHY THE DECODE IS HAND-WRITTEN
------------------------------
`landmark_loc/weights/park_yolo.onnx` exports the RAW YOLOv8 head:

    output0  [1, 9, 8400]  =  4 box (cx, cy, w, h in 640-space)
                            + 5 class scores, one row per class

**No NMS is baked into the graph.** Ultralytics -- which would normally do the
decode -- is not installed and must stay uninstalled (it drags in torch, which
does not install cleanly on this host's Python 3.8). So box decoding, letterbox
inversion and class-wise NMS are implemented below in numpy.

CLASS NAMES ARE NOT THE REPO'S VOCABULARY
-----------------------------------------
The network was trained with its own five names; this repo names landmarks by
the `identity` field of the `map_tools.park_types` registry, which is the single
source of truth shared by the map-building and lidar-detection sides. The
mapping is derived FROM that registry (see `_build_class_identities`) rather
than duplicated into a competing table here, and is asserted at import, so a
future registry rename fails loudly instead of silently mislabelling.

THE REJECT PATH IS PART OF THE CONTRACT
---------------------------------------
Anything scoring below `conf` is DROPPED and nothing is emitted for it. See the
"`unknown` IS PART OF THE CONTRACT" section of `landmark_loc/detector.py`: a
detector that always assigns some class turns every scrap of background into
phantom furniture, those phantoms associate to real catalog landmarks, and the
pose fix is corrupted. This module is not yet wired into that seam, but it
honours the contract now so it can be.
"""
import os

import numpy as np

from map_tools.park_types import PARK_TYPES

# The order the network was trained in. Index == the class channel in output0.
# Fixed by the weights; see landmark_loc/weights/README.md. Do not reorder.
YOLO_CLASS_NAMES = ("bench", "lamp", "tree", "trash_bin", "garden_table")

# The one YOLO name that is not a registry identity. Four of the five names are
# identities verbatim ('bench', 'lamp', 'tree' -- the registry already collapses
# world family tree_8 -> identity 'tree' -- and 'garden_table'). The fifth is
# not: the training labels used the bare noun 'trash_bin' while the registry
# identity carries the park.world model suffix, 'trash_bin_1'. Aliasing here is
# deliberate and explicit rather than fuzzy prefix matching, so that if the
# registry ever gains a second bin family the ambiguity surfaces as a failure.
_YOLO_NAME_ALIASES = {"trash_bin": "trash_bin_1"}

INPUT_SIZE = 640


def _build_class_identities():
    """Resolve each trained YOLO class name to a registry identity.

    Derived from PARK_TYPES so the registry stays the single source of truth.
    Raises at import if any class fails to resolve -- a rename in park_types.py
    must break loudly here, not quietly mislabel every detection downstream.
    """
    known = {t.identity for t in PARK_TYPES}
    resolved = []
    for name in YOLO_CLASS_NAMES:
        identity = _YOLO_NAME_ALIASES.get(name, name)
        if identity not in known:
            raise ImportError(
                "YOLO class %r resolves to identity %r, which is not in the "
                "map_tools.park_types registry (known: %s). The registry was "
                "renamed without updating landmark_loc/camera_detect.py."
                % (name, identity, sorted(known)))
        resolved.append(identity)
    return tuple(resolved)


#: YOLO class index -> this repo's landmark identity.
CLASS_IDENTITIES = _build_class_identities()

DEFAULT_WEIGHTS = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                               "weights", "park_yolo.onnx")


def letterbox(bgr, size=INPUT_SIZE):
    """Resize preserving aspect ratio and pad to `size` x `size`.

    Returns (batch, scale, pad_x, pad_y) where `batch` is the [1,3,size,size]
    float32 RGB /255 network input, and the scale/pad triple is everything
    `decode` needs to map boxes back to source pixels.
    """
    h, w = bgr.shape[:2]
    if h <= 0 or w <= 0:
        raise ValueError("empty image: shape %r" % (bgr.shape,))

    scale = min(float(size) / w, float(size) / h)
    new_w, new_h = int(round(w * scale)), int(round(h * scale))
    # Guard the degenerate case of an extreme aspect ratio rounding to zero.
    new_w, new_h = max(1, min(size, new_w)), max(1, min(size, new_h))

    # cv2 is imported lazily: the pure decode/NMS maths is testable without it.
    import cv2
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    resized = cv2.resize(bgr, (new_w, new_h), interpolation=interp)

    pad_x = (size - new_w) // 2
    pad_y = (size - new_h) // 2
    canvas = np.full((size, size, 3), 114, dtype=np.uint8)   # YOLO's grey pad
    canvas[pad_y:pad_y + new_h, pad_x:pad_x + new_w] = resized

    rgb = canvas[:, :, ::-1]                                  # BGR -> RGB
    chw = np.ascontiguousarray(rgb.transpose(2, 0, 1), dtype=np.float32) / 255.0
    return chw[None, ...], scale, pad_x, pad_y


def nms(boxes, scores, iou_thresh):
    """Greedy NMS over one class. Returns kept indices, highest score first."""
    if len(boxes) == 0:
        return []
    boxes = np.asarray(boxes, dtype=np.float64)
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = np.argsort(np.asarray(scores, dtype=np.float64))[::-1]

    keep = []
    while order.size > 0:
        i = order[0]
        keep.append(int(i))
        if order.size == 1:
            break
        rest = order[1:]
        ix1 = np.maximum(x1[i], x1[rest])
        iy1 = np.maximum(y1[i], y1[rest])
        ix2 = np.minimum(x2[i], x2[rest])
        iy2 = np.minimum(y2[i], y2[rest])
        inter = np.maximum(0.0, ix2 - ix1) * np.maximum(0.0, iy2 - iy1)
        union = areas[i] + areas[rest] - inter
        # A zero-area box cannot overlap anything meaningfully; keep it rather
        # than divide by zero.
        iou = np.where(union > 0.0, inter / np.maximum(union, 1e-12), 0.0)
        order = rest[iou <= iou_thresh]
    return keep


def decode(raw, scale, pad_x, pad_y, orig_wh, conf=0.30, iou=0.45):
    """Turn the raw [1,9,8400] head into boxes in ORIGINAL image pixels.

    Returns [(identity, conf, x1, y1, x2, y2)] sorted by descending confidence.
    Anything below `conf` is dropped (the reject path) -- an empty list is a
    valid and expected result.
    """
    arr = np.asarray(raw, dtype=np.float32)
    if arr.ndim == 3:
        arr = arr[0]
    n_cls = len(YOLO_CLASS_NAMES)
    if arr.shape[0] != 4 + n_cls:
        raise ValueError(
            "expected a [1, %d, N] head, got %r" % (4 + n_cls, np.shape(raw)))

    preds = arr.T                                   # [N, 4 + n_cls]
    scores_all = preds[:, 4:]
    cls_idx = np.argmax(scores_all, axis=1)
    cls_conf = scores_all[np.arange(len(preds)), cls_idx]

    keep_mask = cls_conf >= conf                    # REJECT PATH
    if not np.any(keep_mask):
        return []
    preds = preds[keep_mask]
    cls_idx = cls_idx[keep_mask]
    cls_conf = cls_conf[keep_mask]

    # xywh (centre form, 640-space) -> xyxy, then undo the letterbox: subtract
    # the pad the letterbox added, then divide out the resize scale.
    cx, cy, bw, bh = preds[:, 0], preds[:, 1], preds[:, 2], preds[:, 3]
    x1 = (cx - bw / 2.0 - pad_x) / scale
    y1 = (cy - bh / 2.0 - pad_y) / scale
    x2 = (cx + bw / 2.0 - pad_x) / scale
    y2 = (cy + bh / 2.0 - pad_y) / scale

    orig_w, orig_h = orig_wh
    x1 = np.clip(x1, 0.0, orig_w)
    x2 = np.clip(x2, 0.0, orig_w)
    y1 = np.clip(y1, 0.0, orig_h)
    y2 = np.clip(y2, 0.0, orig_h)
    boxes = np.stack([x1, y1, x2, y2], axis=1)

    # CLASS-WISE NMS: suppress only within a class. A lamp standing in front of
    # a tree overlaps heavily but they are two real, separately useful
    # detections; cross-class suppression would delete one of them.
    out = []
    for c in np.unique(cls_idx):
        sel = np.nonzero(cls_idx == c)[0]
        for k in nms(boxes[sel], cls_conf[sel], iou):
            j = sel[k]
            out.append((CLASS_IDENTITIES[int(c)], float(cls_conf[j]),
                        float(boxes[j, 0]), float(boxes[j, 1]),
                        float(boxes[j, 2]), float(boxes[j, 3])))

    out.sort(key=lambda d: d[1], reverse=True)
    return out


class ObjectDetector(object):
    """Runs park_yolo.onnx over BGR frames and yields identified boxes."""

    def __init__(self, weights_path=DEFAULT_WEIGHTS, conf=0.30, iou=0.45):
        self.weights_path = weights_path
        self.conf = float(conf)
        self.iou = float(iou)
        self._session = None
        self._input_name = None

    @property
    def session(self):
        """The onnxruntime session, created on first use.

        Lazy so that constructing a detector (and importing this module) costs
        nothing, and so a node can report a missing weights file as a clean
        fatal rather than dying inside __init__.
        """
        if self._session is None:
            if not os.path.isfile(self.weights_path):
                raise IOError("detector weights not found: %s"
                              % self.weights_path)
            import onnxruntime
            self._session = onnxruntime.InferenceSession(
                self.weights_path, providers=["CPUExecutionProvider"])
            self._input_name = self._session.get_inputs()[0].name
        return self._session

    def detect(self, bgr):
        """BGR image -> [(identity, conf, x1, y1, x2, y2)] in image pixels."""
        batch, scale, pad_x, pad_y = letterbox(bgr, INPUT_SIZE)
        session = self.session
        raw = session.run(None, {self._input_name: batch})[0]
        h, w = bgr.shape[:2]
        return decode(raw, scale, pad_x, pad_y, (w, h),
                      conf=self.conf, iou=self.iou)
