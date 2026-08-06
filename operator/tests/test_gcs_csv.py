import math
from gcs_csv import CSV_HEADER, build_row

def test_header_contract():
    assert CSV_HEADER[:11] == [
        "elapsed_time","fused_x","fused_y","fused_yaw","fused_yaw_deg",
        "planner_linear_x","planner_angular_z","ctrl_linear_x","ctrl_angular_z",
        "ref_x","ref_y"]
    assert CSV_HEADER[11:] == [
        "active_goal_x","active_goal_y","nav_status","heartbeat_age","operator_mode"]

def test_build_row_full():
    row = build_row(
        elapsed=5.0, pose=(1.0, 2.0, math.pi/2), planner=(0.4, 0.0),
        ctrl=(0.4, 0.0), sent_goal=(10.0, 0.0), active_goal=(10.0, 12.0),
        nav_status="ACTIVE", heartbeat_age=0.2, mode="AUTO")
    assert len(row) == 16
    assert row[0] == "5.0000"
    assert row[4] == "%.4f" % 90.0           # yaw_deg
    assert row[9] == "10.0000" and row[10] == "0.0000"   # ref_x/y = sent goal
    assert row[11] == "10.0000" and row[12] == "12.0000" # active goal
    assert row[13] == "ACTIVE"
    assert row[15] == "AUTO"

def test_build_row_missing_active_goal():
    row = build_row(
        elapsed=0.0, pose=(0.0,0.0,0.0), planner=(0.0,0.0), ctrl=(0.0,0.0),
        sent_goal=None, active_goal=None, nav_status="NONE",
        heartbeat_age=1.0, mode="AUTO")
    assert row[9] == "nan" and row[11] == "nan"
