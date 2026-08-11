import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "operator"))
from gcs_commands import parse_command


def test_mode_gps():
    assert parse_command("mode gps") == ("mode", ["gps"])

def test_mode_landmark():
    assert parse_command("mode landmark") == ("mode", ["landmark"])

def test_mode_case_insensitive_value():
    assert parse_command("mode GPS") == ("mode", ["gps"])

def test_mode_missing_arg_is_error():
    verb, _ = parse_command("mode")
    assert verb == "error"

def test_mode_unknown_value_is_error():
    verb, _ = parse_command("mode teleporter")
    assert verb == "error"
