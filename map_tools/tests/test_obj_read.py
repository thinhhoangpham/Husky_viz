import os, numpy as np
from map_tools.obj_read import read_obj_triangles

BARK = os.path.join(os.path.dirname(__file__), "..", "..", "models_opt", "tree_8", "bark8.obj")

def test_reads_a_known_obj_height():
    tris = read_obj_triangles(BARK, scale=1.0)
    assert tris.ndim == 3 and tris.shape[1:] == (3, 3)
    zs = tris[:, :, 2]
    assert zs.max() - zs.min() > 5.0   # the trunk mesh is several metres tall

def test_face_formats_and_fan_triangulation():
    import tempfile, textwrap
    obj = textwrap.dedent('''\
        v 0 0 0
        v 1 0 0
        v 1 1 0
        v 0 1 0
        f 1/1/1 2/2/2 3/3/3 4/4/4
    ''')
    with tempfile.NamedTemporaryFile("w", suffix=".obj", delete=False) as fh:
        fh.write(obj); path = fh.name
    tris = read_obj_triangles(path, scale=2.0)
    assert tris.shape == (2, 3, 3)          # quad -> 2 triangles by fan
    assert np.isclose(np.abs(tris).max(), 2.0)  # scale applied

def test_negative_indices():
    import tempfile, textwrap
    obj = "v 0 0 0\nv 1 0 0\nv 0 1 0\nf -3 -2 -1\n"
    with tempfile.NamedTemporaryFile("w", suffix=".obj", delete=False) as fh:
        fh.write(obj); path = fh.name
    tris = read_obj_triangles(path)
    assert tris.shape == (1, 3, 3)
