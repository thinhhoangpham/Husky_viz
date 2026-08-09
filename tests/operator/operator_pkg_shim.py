import importlib.util, os
_spec = importlib.util.spec_from_file_location(
    "park_places_mod",
    os.path.join(os.path.dirname(__file__), "..", "..", "operator", "places.py"))
_m = importlib.util.module_from_spec(_spec); _spec.loader.exec_module(_m)
load_places = _m.load_places
resolve = _m.resolve
