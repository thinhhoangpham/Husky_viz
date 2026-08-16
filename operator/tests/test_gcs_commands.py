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

def test_route_coordinate_pairs():
    assert parse_command("route 10 -8 30 -8 30 12") == (
        "route", [(10.0, -8.0), (30.0, -8.0), (30.0, 12.0)])

def test_route_two_pairs_is_minimum():
    assert parse_command("route 1.5 2.5 -3 -4") == (
        "route", [(1.5, 2.5), (-3.0, -4.0)])

def test_route_odd_arg_count_is_error():
    assert parse_command("route 1 2 3") == (
        "error", ["route needs an even number of args, at least two "
                  "<x> <y> pairs"])

def test_route_single_pair_is_error():
    # one pair is a single goal -- `goal xy` -- not a route
    cmd, _ = parse_command("route 1 2")
    assert cmd == "error"

def test_route_non_numeric_is_error():
    # object NAMES are no longer accepted: waypoints are bare coordinates
    assert parse_command("route pole_A bench_3 pole_B lamp_2") == (
        "error", ["route args must be numbers"])
    # a name mixed in among coordinates is rejected too
    assert parse_command("route 10 -8 pole_A -8") == (
        "error", ["route args must be numbers"])

def test_route_no_args_is_error():
    cmd, _ = parse_command("route")
    assert cmd == "error"

def test_blank_and_unknown():
    assert parse_command("   ") == ("noop", [])
    assert parse_command("frobnicate") == ("unknown", ["frobnicate"])
