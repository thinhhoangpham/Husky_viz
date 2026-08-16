from gcs_commands import parse_command

def test_goal_ok():
    assert parse_command("goal 49.9 8.9") == ("goal", [49.9, 8.9])

def test_goal_single_arg_is_named_object():
    # a single non-lat/lon arg is a named-object lookup, not an error
    assert parse_command("goal 49.9") == ("goal_name", ["49.9"])

def test_goal_xy_wrong_arity_is_error():
    # 'goal xy' needs exactly two numbers; one is an error
    cmd, _ = parse_command("goal xy 1")
    assert cmd == "error"

def test_simple_verbs():
    for v in ["cancel","teleop","stop","estop","release","auto","status","quit"]:
        assert parse_command(v) == (v, [])

def test_route_names():
    assert parse_command("route pole_A bench_3 pole_B") == (
        "route", ["pole_A", "bench_3", "pole_B"])

def test_route_single_name():
    assert parse_command("route pole_A") == ("route", ["pole_A"])

def test_route_no_names_is_error():
    cmd, _ = parse_command("route")
    assert cmd == "error"

def test_blank_and_unknown():
    assert parse_command("   ") == ("noop", [])
    assert parse_command("frobnicate") == ("unknown", ["frobnicate"])
