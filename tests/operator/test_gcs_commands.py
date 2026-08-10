import importlib.util, os
_spec = importlib.util.spec_from_file_location(
    "gcs_commands_mod",
    os.path.join(os.path.dirname(__file__), "..", "..", "operator", "gcs_commands.py"))
_m = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_m)
parse_command = _m.parse_command

def test_goal_latlon_unchanged():
    assert parse_command("goal 49.9 8.9") == ("goal", [49.9, 8.9])

def test_goal_xy():
    assert parse_command("goal xy 12.5 -3.0") == ("goal_xy", [12.5, -3.0])

def test_goal_name():
    assert parse_command("goal bench_1") == ("goal_name", ["bench_1"])

def test_goal_xy_bad_args():
    cmd, args = parse_command("goal xy foo bar")
    assert cmd == "error"
