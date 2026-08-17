import math
import numpy as np
from landmark_loc import derotate


def test_tilted_ground_is_flatter_after_derotation():
    # synthetic ground tilted 17 deg in pitch (the measured lake slope)
    pitch = math.radians(17.0)
    xs = np.linspace(0, 20, 200)
    pts = np.zeros((200, 3))
    pts[:, 0] = xs
    pts[:, 2] = math.tan(pitch) * xs  # ground rising with range
    before = pts[:, 2].std()
    out = derotate.derotate_cloud(pts, 0.0, pitch)
    after = out[:, 2].std()
    assert after < before * 0.01  # tilt essentially removed
