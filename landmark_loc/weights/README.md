# Detector weights

## `park_yolo.onnx`

YOLOv8n fine-tuned on the park world, 5 classes in this **exact** order:

    0 bench   1 lamp   2 tree   3 trash_bin   4 garden_table

- input  `images`  `[1, 3, 640, 640]` float32, RGB, `/255`, letterboxed
- output `output0` `[1, 9, 8400]` — raw YOLOv8 head: 4 box (xywh, 640-space)
  + 5 class scores. **No NMS is baked in**; the caller must decode and
  suppress (see `landmark_loc/camera_detect.py`).
- sha256 `554e5f22cfeeeff1458cbd6bb38a06601b4ecbeeb541c1c1efc32f5150bc266d`

Runs on CPU via `onnxruntime==1.16.3` (the last release with a `cp38` wheel,
and ROS Noetic pins Python 3.8) at ~68 ms/frame on this host — well inside the
2 Hz the camera publishes at. ONNX is used in preference to the original
`.pt` because that would drag in torch + ultralytics, neither of which
installs cleanly on Python 3.8.

## Provenance — WHY THIS IS COMMITTED

Copied from `Husky_new_simulation/research/vision/weights/park_yolo.onnx`.

**These weights cannot be regenerated from anything in either repo.** Retraining
would need all of:

- `output/vision_dataset/` — the 225 recorded frames + camera poses. Gitignored,
  and no longer on disk.
- `park_models/` — the meshes `autolabel_mesh.py` projects to make pixel-tight
  boxes. Gitignored, and built from an external ~12.7 GB model dataset that is
  not on this machine.
- a GPU training run (the original was done on Google Colab).

So this file is treated as an irreplaceable binary artifact and is committed
despite its size, rather than referenced from the sibling repo — which is not
version-controlled together with this one. If you ever do retrain, record the
new dataset here and update this file.

### Training geometry — DO NOT CHANGE THE CAMERAS WITHOUT RETRAINING

The detector was trained on images from two **mono** cameras
(`sensor type="camera"`, `libgazebo_ros_camera.so`), NOT a stereo pair:

| | value |
|---|---|
| mount | `(0.32, ±0.10, 0.60)` on `base_link` |
| yaw | `±0.436 rad` (±25°, diverged to widen coverage) |
| hfov | `1.2113 rad` (69.4°) |
| image | 512 × 384 |

`natural_environments_ros_opt/.../rgb_cams.urdf.xacro` replicates this exactly.
Changing FOV, resolution or mount height changes the apparent scale and image
position of every object, which is precisely what a small fine-tuned detector is
sensitive to — the repo's separate `stereo_camera.urdf.xacro` (multicamera,
1280×720, hfov 1.815, both forward) is a **different sensor** and these weights
should not be expected to work with it.
