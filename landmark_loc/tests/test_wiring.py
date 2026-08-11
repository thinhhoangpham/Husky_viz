import re

CTRL = ("/home/thinh/Documents/Husky_viz/natural_environments_ros_opt/"
        "husky/husky_control/launch/control.launch")
EKF = ("/home/thinh/Documents/Husky_viz/natural_environments_ros_opt/"
       "husky/husky_control/config/localization_map.yaml")


def test_map_ekf_odom1_is_neutral_abs_fix():
    txt = open(EKF).read()
    assert re.search(r"^odom1:\s*odometry/abs_fix\s*$", txt, re.M)
    assert "odom1: odometry/gps" not in txt


def test_navsat_remaps_output_to_abs_fix():
    txt = open(CTRL).read()
    # navsat node must remap its odometry/gps output to the neutral topic so
    # GPS mode still feeds the EKF after the rename.
    assert re.search(r'from="odometry/gps"\s+to="odometry/abs_fix"', txt)
