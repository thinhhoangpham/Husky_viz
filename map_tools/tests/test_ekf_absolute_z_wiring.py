"""Guards the absolute-z wiring across the dual-EKF setup.

THE MECHANISM being guarded (robot_localization ros_filter.cpp, noetic-devel):
a pose measurement is transformed into the receiving filter's `world_frame`
before it is fused. odometryCallback passes `targetFrame = worldFrameId_`
(~1786); preparePose sets `finalTargetFrame = targetFrame` and, for a
non-differential input, `poseTmp.frame_id_ = msg->header.frame_id` (~2666);
it looks up `finalTargetFrame <- poseTmp.frame_id_` (~2713) and applies it with
`poseTmp.mult(targetFrameTrans, poseTmp)` (~3021).

So the applied transform is `lookup(world_frame <- header.frame_id)`. It is the
identity EXACTLY when the stamp equals that filter's world_frame. The two
filters have different world_frames, so ONE topic cannot serve both -- whichever
filter the stamp does not match applies a live tf lookup (one of the filters'
own outputs) to its own input.

Writing M = (map->base_link).z, O = (odom->base_link).z, T = the true height,
and noting the map filter emits map->odom = M - O (~2025), all three wirings
that have been live in this repo:

  1. one topic stamped "odom", fused in both.
     Map filter applies map<-odom = +(M-O): M := T + (M-O), runaway.
     Measured map->odom z = +1.228, robot 1.23 m high.

  1b. dropped from the map filter entirely.
     That filter ESTIMATES z rather than passing it through, so with no z input
     it sat at its initial 0 and emitted the negated height.
     Measured map->odom z = -3.839, robot back underground.

  2. one topic stamped "map", fused in both.
     Map filter correct (identity). Odom filter applies odom<-map = -(M-O) and
     fuses T - (M-O), which with M = T is exactly O -- its own estimate. The
     measurement cancels itself, so the odom filter's z is UNOBSERVED and freezes
     wherever it drifted. Measured: ground_height 3.893, filtered_odom 5.927,
     map->odom -2.034. The TOTAL map->base_link was right (3.893) while the SPLIT
     was 2.034 m wrong -- and the local costmap runs in the odom frame
     (config/costmap_local_gps.yaml, global_frame: odom), so it consumed the
     wrong half.

  3. TWO topics carrying the SAME number, each stamped in its own filter's
     world_frame.  <- CURRENT, what these tests pin.
     Both transforms are the identity: M := T and O := T, map->odom = 0.

The odom-stamped copy is deliberately NOT compensated by the current map->odom.
It is the identical raw number; `odom <- odom` is already the identity, so there
is nothing to compensate, and computing it from map->odom would rebuild the very
output-into-input feedback that broke wirings 1 and 2.
"""
import os
import re

import yaml

_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
_MAP_YAML = os.path.join(_ROOT, "natural_environments_ros_opt", "husky",
                         "husky_control", "config", "localization_map.yaml")
_ODOM_YAML = os.path.join(_ROOT, "natural_environments_ros_opt", "husky",
                          "husky_control", "config", "localization.yaml")
_SCRIPT = os.path.join(_ROOT, "scripts", "publish_ground_height_odom.py")
_RUNBOOK = os.path.join(_ROOT, "RUN-MAP-NAV.md")

_Z_INDEX = 2  # position of the z flag in robot_localization's 15-element config

# The map filter's world_frame is "map", the odom filter's is "odom", so each
# must read the copy stamped to match. Swapping these two is the whole bug.
_MAP_TOPIC = "odometry/ground_height"
_ODOM_TOPIC = "odometry/ground_height_odom"


def _load(path):
    with open(path) as fh:
        return yaml.safe_load(fh)


def _input_for_topic(cfg, topic):
    """(prefix, config-list) for whichever odomN input reads `topic`."""
    for key, value in cfg.items():
        if re.fullmatch(r"odom\d+", key) and value == topic:
            return key, cfg[key + "_config"]
    return None, None


def _z_input(cfg):
    """(prefix, topic) for whichever odomN input asserts z, whatever it reads."""
    for key, value in cfg.items():
        if re.fullmatch(r"(odom|pose)\d+", key) and cfg[key + "_config"][_Z_INDEX]:
            return key, value
    return None, None


def test_map_filter_observes_absolute_z_from_the_map_stamped_topic():
    """The map filter must have a z input, and it must be the map-stamped copy.

    Without any z input it does not inherit the odom filter's height -- it
    estimates z at 0 and negates it into map->odom (wiring 1b)."""
    cfg = _load(_MAP_YAML)
    assert cfg["world_frame"] == "map", "premise of this whole file"
    key, z_cfg = _input_for_topic(cfg, _MAP_TOPIC)
    assert key is not None, "map EKF has no %s input" % _MAP_TOPIC
    assert z_cfg[_Z_INDEX] is True, "%s must fuse z" % key
    assert cfg[key + "_differential"] is False, "absolute elevation, not a delta"


def test_odom_filter_observes_absolute_z_from_the_ODOM_stamped_topic():
    """The odom filter must read the odom-stamped copy, NOT the map-stamped one.

    This is the regression that motivated the second topic: reading the
    map-stamped copy here makes the measurement cancel itself (wiring 2), so z
    goes unobserved and the odom frame silently freezes metres off."""
    cfg = _load(_ODOM_YAML)
    assert cfg["world_frame"] == "odom", "premise of this whole file"
    key, z_cfg = _input_for_topic(cfg, _ODOM_TOPIC)
    assert key is not None, "odom EKF has no %s input" % _ODOM_TOPIC
    assert z_cfg[_Z_INDEX] is True, "%s must fuse z" % key
    assert cfg[key + "_differential"] is False, "absolute elevation, not a delta"


def test_neither_filter_reads_the_other_filters_copy():
    """Belt and braces: the wrong topic must not appear in either file at all.

    Catches the case where someone adds the right input but leaves the old one
    behind, which would fuse both and reintroduce the feedback term."""
    assert _input_for_topic(_load(_ODOM_YAML), _MAP_TOPIC) == (None, None), \
        "odom EKF must not read the map-stamped %s" % _MAP_TOPIC
    assert _input_for_topic(_load(_MAP_YAML), _ODOM_TOPIC) == (None, None), \
        "map EKF must not read the odom-stamped %s" % _ODOM_TOPIC


def test_exactly_one_z_input_per_filter_and_it_is_the_ground_height():
    """Only the ground-height input may assert z, in EITHER filter. abs_fix in
    particular carries z = 0.0, so fusing its z would drag the robot to zero."""
    for path, expected in ((_MAP_YAML, _MAP_TOPIC), (_ODOM_YAML, _ODOM_TOPIC)):
        cfg = _load(path)
        z_keys = [k for k, v in cfg.items()
                  if re.fullmatch(r"(odom|pose)\d+", k)
                  and cfg[k + "_config"][_Z_INDEX]]
        assert len(z_keys) == 1, \
            "%s: exactly one input may assert z, got %s" % (path, z_keys)
        assert cfg[z_keys[0]] == expected, \
            "%s: the z input must be %s, got %s" % (path, expected, cfg[z_keys[0]])


def test_both_filters_keep_two_d_mode_off():
    """two_d_mode forces z to zero. Either filter having it on flattens the
    composed map->base_link regardless of what the other one estimates."""
    assert _load(_MAP_YAML)["two_d_mode"] is False
    assert _load(_ODOM_YAML)["two_d_mode"] is False


def test_publisher_defaults_to_the_correct_frame_pair():
    """The two stamps default to map and odom respectively, so the runbook does
    not have to pass either. The old "odom" default for ~frame_id was a
    backwards-compatibility concession that silently produced wiring 1."""
    src = open(_SCRIPT).read()
    assert 'rospy.get_param("~frame_id", "map")' in src
    assert 'rospy.get_param("~frame_id_odom", "odom")' in src
    assert 'rospy.get_param("~topic", "/%s")' % _MAP_TOPIC in src
    assert 'rospy.get_param("~topic_odom", "/%s")' % _ODOM_TOPIC in src


def test_publisher_stamps_from_the_params_not_from_literals():
    """Both messages must take their frame from the params, so the guard below
    and the config above actually govern what is published."""
    src = open(_SCRIPT).read()
    assert "od.header.frame_id = frame_id" in src
    assert "pub.publish(_height_msg(Odometry, stamp, frame_id, z, cov))" in src
    assert ("pub_odom.publish(_height_msg(Odometry, stamp, frame_id_odom, z, cov))"
            in src)
    assert 'od.header.frame_id = "odom"' not in src
    assert 'od.header.frame_id = "map"' not in src


def test_publisher_refuses_two_identical_stamps():
    """If both stamps were the same frame, one filter would be applying its own
    output to its own input again -- the exact bug. The node must not start."""
    src = open(_SCRIPT).read()
    assert "if frame_id == frame_id_odom:" in src


def test_publisher_does_not_compensate_the_odom_copy_with_map_to_odom():
    """The odom-stamped value must be the SAME raw number, not one corrected by
    the current map->odom. Deriving an EKF input from the EKFs' own output is
    the feedback loop this whole design exists to avoid, and `odom <- odom` is
    already the identity so there is nothing to correct."""
    src = open(_SCRIPT).read()
    # one z, published twice; no second height variable, no map->odom lookup
    assert src.count("od.pose.pose.position.z = z") == 1
    assert "lookup_transform(\"map\", \"odom\"" not in src
    assert "lookup_transform('map', 'odom'" not in src


def test_runbook_starts_the_publisher_without_a_frame_override():
    """The defaults are the correct pair now, and passing _frame_id:=map is what
    produced wiring 2 -- the map filter right, the odom filter 2 m high."""
    text = open(_RUNBOOK).read()
    lines = [l for l in text.splitlines()
             if "publish_ground_height_odom.py" in l
             and l.strip().startswith(("PYTHON", "python"))]
    assert lines, "runbook no longer launches the ground-height publisher"
    for l in lines:
        assert "_frame_id:=" not in l, \
            "runbook must not override the frame any more: %s" % l
        assert "_dtm_path:=" in l


def test_runbook_documents_the_odom_frame_check():
    """A correct map->base_link total hid a 2 m wrong split for a whole commit,
    so the runbook has to tell the operator to check the odom frame too."""
    text = open(_RUNBOOK).read()
    assert "tf_echo odom base_link" in text
    assert "filtered_odom" in text
