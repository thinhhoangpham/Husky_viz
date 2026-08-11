import os
import re

# Repo root = three levels up from this file
# (landmark_loc/tests/test_wiring.py -> repo root). Derive from __file__ so the
# test validates the tree it actually lives in (worktree or main), not a
# hardcoded absolute path into some other checkout.
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
CTRL = os.path.join(_REPO, "natural_environments_ros_opt", "husky",
                    "husky_control", "launch", "control.launch")
EKF = os.path.join(_REPO, "natural_environments_ros_opt", "husky",
                   "husky_control", "config", "localization_map.yaml")
SELECTOR = os.path.join(_REPO, "landmark_loc", "abs_fix_selector.py")
LOCALIZER = os.path.join(_REPO, "landmark_loc", "localizer_node.py")


def test_map_ekf_odom1_is_neutral_abs_fix():
    txt = open(EKF).read()
    assert re.search(r"^odom1:\s*odometry/abs_fix\s*$", txt, re.M)
    assert "odom1: odometry/gps" not in txt


def test_navsat_remaps_output_to_gps_fix():
    # navsat now outputs to /odometry/gps_fix (a distinct source topic); the
    # selector -- not navsat -- fills /odometry/abs_fix. The old
    # odometry/gps -> odometry/abs_fix remap must be gone.
    txt = open(CTRL).read()
    assert re.search(r'from="odometry/gps"\s+to="odometry/gps_fix"', txt)
    assert 'to="odometry/abs_fix"' not in txt


def test_selector_is_sole_abs_fix_publisher():
    # The selector publishes /odometry/abs_fix; the localizer no longer does
    # (it publishes /odometry/landmark_fix).
    selector = open(SELECTOR).read()
    localizer = open(LOCALIZER).read()
    assert '"/odometry/abs_fix"' in selector or "OUTPUT_TOPIC" in selector
    assert '"/odometry/landmark_fix"' in localizer
    assert '"/odometry/abs_fix"' not in localizer
