import os
import sys
import pytest

# Add the tests/operator directory to sys.path so we can import operator_pkg_shim
sys.path.insert(0, os.path.dirname(__file__))
from operator_pkg_shim import load_places, resolve

def test_load_and_resolve(tmp_path):
    y = tmp_path / "places.yaml"
    y.write_text("bench_1: {x: 10.0, y: 5.0}\nlamp_2: {x: -3.0, y: 2.0}\n")
    places = load_places(str(y))
    assert places["bench_1"] == (10.0, 5.0)
    gx, gy = resolve("bench_1", places, offset=1.0)
    # Offset by 1 m along +x from the object, so the goal is beside it.
    assert (gx, gy) == (11.0, 5.0)

def test_resolve_unknown_raises_with_names(tmp_path):
    places = {"bench_1": (1.0, 2.0)}
    with pytest.raises(KeyError) as exc:
        resolve("nope", places)
    assert "bench_1" in str(exc.value)
