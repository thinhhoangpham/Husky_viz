from gcs_commands import parse_command

def test_goal_ok():
    assert parse_command("goal 49.9 8.9") == ("goal", [49.9, 8.9])

def test_goal_bad_args():
    cmd, args = parse_command("goal 49.9")
    assert cmd == "error"

def test_simple_verbs():
    for v in ["cancel","teleop","stop","estop","release","auto","status","quit"]:
        assert parse_command(v) == (v, [])

def test_blank_and_unknown():
    assert parse_command("   ") == ("noop", [])
    assert parse_command("frobnicate") == ("unknown", ["frobnicate"])
