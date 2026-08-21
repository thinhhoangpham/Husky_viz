#!/usr/bin/env python3
"""ROS wrapper around the YOLO park-object detector.

Subscribes to both forward RGB cameras, runs `landmark_loc.camera_detect` on
each frame, and publishes what it found. All the decode maths lives in that
ROS-free module so it can be unit tested without a master; this file is only
plumbing.

PUBLISHED MESSAGE LAYOUT
------------------------
`/perception/cam_detections` is a `std_msgs/Float32MultiArray` -- a STANDARD
message on purpose, so nothing has to build or source a custom msg package to
read it. `data` is flat, 7 floats per detection:

    [cls_idx, conf, x1, y1, x2, y2, cam] * N

    cls_idx  index into landmark_loc.camera_detect.CLASS_IDENTITIES, which maps
             it to this repo's landmark identity (e.g. 3 -> 'trash_bin_1')
    conf     0..1
    x1,y1,x2,y2   bounding box in ORIGINAL image pixels (512x384), top-left
             origin, x right, y down
    cam      WHICH CAMERA saw it: 0 = left, 1 = right. The two cameras are
             yawed +/-25 deg apart, so a bearing cannot be recovered from the
             pixel column alone -- the source camera is required, not optional.

`layout` is filled in (one 7-wide `detections` dim by one `fields` dim) so a
consumer can confirm the stride rather than hardcoding 7. N == 0 is a normal,
frequent result: it means nothing was seen with enough confidence.

VERIFICATION SURFACE
--------------------
With `~save` set, annotated JPEGs are written to `~save_dir`, throttled to one
per `~save_period` seconds PER CAMERA. `~save_raw` additionally writes the
unannotated frame beside each one: if detections look wrong in sim, the raw
frame is what distinguishes "the camera is mounted or aimed wrong" from "the
detector is misbehaving". There is no offline image corpus to fall back on, so
these files are the primary evidence.
"""
import os
import sys

import cv2

# The node is exec'd from scripts/, so make the repo root importable.
_REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _REPO_ROOT not in sys.path:
    sys.path.insert(0, _REPO_ROOT)

from landmark_loc.camera_detect import (  # noqa: E402
    CLASS_IDENTITIES, DEFAULT_WEIGHTS, ObjectDetector)

FIELDS_PER_DETECTION = 7
CAMERAS = (("left", "/rgb_cam_left/image_raw", 0),
           ("right", "/rgb_cam_right/image_raw", 1))

# Distinct BGR colours per class so a glance at a saved frame is enough.
_COLORS = ((0, 200, 255), (255, 160, 0), (0, 220, 0),
           (200, 0, 200), (0, 90, 255))


def annotate(bgr, detections):
    """Draw boxes with class name and confidence. Returns a new image."""
    out = bgr.copy()
    for identity, conf, x1, y1, x2, y2 in detections:
        try:
            color = _COLORS[CLASS_IDENTITIES.index(identity)]
        except ValueError:
            color = (255, 255, 255)
        p1 = (int(round(x1)), int(round(y1)))
        p2 = (int(round(x2)), int(round(y2)))
        cv2.rectangle(out, p1, p2, color, 2)

        label = "%s %.2f" % (identity, conf)
        (tw, th), base = cv2.getTextSize(
            label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        # Put the label inside the image when the box is against the top edge,
        # otherwise it is drawn off-frame and lost.
        top = p1[1] - th - base
        if top < 0:
            top = p1[1] + 2
        cv2.rectangle(out, (p1[0], top), (p1[0] + tw + 4, top + th + base),
                      color, -1)
        cv2.putText(out, label, (p1[0] + 2, top + th),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1,
                    cv2.LINE_AA)
    return out


class CameraDetectNode(object):

    def __init__(self):
        import rospy
        from cv_bridge import CvBridge

        self._rospy = rospy
        self.bridge = CvBridge()

        weights = rospy.get_param("~weights", DEFAULT_WEIGHTS)
        self.conf = float(rospy.get_param("~conf", 0.30))
        self.save = bool(rospy.get_param("~save", True))
        self.save_raw = bool(rospy.get_param("~save_raw", True))
        self.save_period = float(rospy.get_param("~save_period", 1.0))
        save_dir = rospy.get_param(
            "~save_dir", os.path.join(_REPO_ROOT, "artifacts", "camera_detect"))
        # CLAUDE.md: '~' is NOT expanded inside a ROS param, and a literal '~'
        # in a path yields a confusing FileNotFoundError. Expand it defensively
        # and resolve relative paths against the repo root, not the cwd roslaunch
        # happened to start in.
        save_dir = os.path.expanduser(str(save_dir))
        if not os.path.isabs(save_dir):
            save_dir = os.path.join(_REPO_ROOT, save_dir)
        self.save_dir = save_dir

        weights = os.path.expanduser(str(weights))
        if not os.path.isfile(weights):
            rospy.logfatal(
                "camera_detect: weights file not found: %s -- set the "
                "~weights param to an ABSOLUTE path to park_yolo.onnx "
                "('~' is not expanded by ROS params).", weights)
            raise SystemExit(1)

        self.detector = ObjectDetector(weights, conf=self.conf)
        try:
            self.detector.session          # load now, so failure is immediate
        except Exception as exc:           # noqa: BLE001 - reported, not hidden
            rospy.logfatal("camera_detect: could not load %s: %s", weights, exc)
            raise SystemExit(1)

        if self.save:
            try:
                os.makedirs(self.save_dir)
            except OSError:
                if not os.path.isdir(self.save_dir):
                    rospy.logfatal("camera_detect: cannot create save_dir %s",
                                   self.save_dir)
                    raise SystemExit(1)
            rospy.loginfo("camera_detect: saving annotated frames to %s",
                          self.save_dir)

        from std_msgs.msg import Float32MultiArray
        from sensor_msgs.msg import Image
        self._Float32MultiArray = Float32MultiArray

        self.pub = rospy.Publisher("/perception/cam_detections",
                                   Float32MultiArray, queue_size=1)

        self._last_save = {}
        self._seen = set()
        for name, topic, cam_id in CAMERAS:
            rospy.Subscriber(topic, Image, self._on_image,
                             callback_args=(name, cam_id),
                             queue_size=1, buff_size=2 ** 24)
            rospy.loginfo("camera_detect: subscribed to %s", topic)
        rospy.loginfo(
            "camera_detect: ready (conf=%.2f). Either camera publishing alone "
            "is fine; nothing here waits for both.", self.conf)

    def _on_image(self, msg, args):
        name, cam_id = args
        rospy = self._rospy

        if name not in self._seen:
            self._seen.add(name)
            rospy.loginfo("camera_detect: first frame from %s camera", name)

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as exc:          # noqa: BLE001 - one bad frame only
            rospy.logerr_throttle(
                5.0, "camera_detect: cannot decode %s image: %s" % (name, exc))
            return

        try:
            detections = self.detector.detect(bgr)
        except Exception as exc:          # noqa: BLE001 - one bad frame only
            rospy.logerr_throttle(
                5.0, "camera_detect: inference failed on %s: %s" % (name, exc))
            return

        self._publish(detections, cam_id)

        tally = {}
        for det in detections:
            tally[det[0]] = tally.get(det[0], 0) + 1
        # Sorted by count, not by name: the dominant class is the informative
        # part of the line.
        summary = ", ".join(
            "%s x%d" % (k, v)
            for k, v in sorted(tally.items(), key=lambda kv: -kv[1]))
        rospy.loginfo_throttle(
            2.0, "camera_detect[%s]: %d detection(s)%s"
            % (name, len(detections), (" -- " + summary) if summary else ""))

        if self.save:
            self._maybe_save(name, bgr, detections)

    def _publish(self, detections, cam_id):
        from std_msgs.msg import MultiArrayDimension

        msg = self._Float32MultiArray()
        data = []
        for identity, conf, x1, y1, x2, y2 in detections:
            data.extend([float(CLASS_IDENTITIES.index(identity)), float(conf),
                         float(x1), float(y1), float(x2), float(y2),
                         float(cam_id)])
        msg.data = data

        n = len(detections)
        msg.layout.dim = [
            MultiArrayDimension(label="detections", size=n,
                                stride=n * FIELDS_PER_DETECTION),
            MultiArrayDimension(label="fields", size=FIELDS_PER_DETECTION,
                                stride=FIELDS_PER_DETECTION),
        ]
        msg.layout.data_offset = 0
        self.pub.publish(msg)

    def _maybe_save(self, name, bgr, detections):
        rospy = self._rospy
        now = rospy.get_time()
        # Throttled PER CAMERA, so a quiet camera never starves the busy one.
        if now - self._last_save.get(name, 0.0) < self.save_period:
            return
        self._last_save[name] = now

        stamp = "%.2f" % now
        base = os.path.join(self.save_dir, "%s_%s" % (stamp, name))
        try:
            cv2.imwrite(base + ".jpg", annotate(bgr, detections))
            if self.save_raw:
                # The unannotated frame is what tells "camera aimed wrong" apart
                # from "detector wrong" when a sim result looks surprising.
                cv2.imwrite(base + "_raw.jpg", bgr)
        except Exception as exc:          # noqa: BLE001 - saving is diagnostic
            rospy.logerr_throttle(
                10.0, "camera_detect: could not write %s: %s" % (base, exc))


def main():
    import rospy
    rospy.init_node("camera_detect")
    CameraDetectNode()
    rospy.spin()


if __name__ == "__main__":
    main()
