"""Occupancy grid with world<->cell transforms, disc stamping, and ROS
map_server PGM/YAML output. Pure geometry + IO; no SDF knowledge.

ROS map_server convention: PGM pixel value 0 = occupied (black), 254 = free
(white), 205 = unknown. Row 0 of the PGM is the TOP of the image, which maps to
the HIGHEST y in the world (origin is the bottom-left corner). We store cells
with row 0 = lowest y and flip vertically on write.
"""
import math


class Grid:
    OCC = 0
    FREE = 254

    def __init__(self, min_x, min_y, max_x, max_y, resolution):
        self.resolution = resolution
        self.origin_x = min_x
        self.origin_y = min_y
        self.width = int(math.ceil((max_x - min_x) / resolution))
        self.height = int(math.ceil((max_y - min_y) / resolution))
        # Row-major, row 0 = lowest y. Default free.
        self._cells = bytearray([self.FREE]) * (self.width * self.height)

    def world_to_cell(self, x, y):
        col = int((x - self.origin_x) / self.resolution)
        row = int((y - self.origin_y) / self.resolution)
        return col, row

    def _in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    def _set_occ(self, col, row):
        if self._in_bounds(col, row):
            self._cells[row * self.width + col] = self.OCC

    def is_occupied(self, x, y):
        col, row = self.world_to_cell(x, y)
        if not self._in_bounds(col, row):
            return False
        return self._cells[row * self.width + col] == self.OCC

    def stamp_disc(self, x, y, radius):
        r_cells = int(math.ceil(radius / self.resolution))
        c0, r0 = self.world_to_cell(x, y)
        for dr in range(-r_cells, r_cells + 1):
            for dc in range(-r_cells, r_cells + 1):
                # Cell center distance check in metres.
                cx = self.origin_x + (c0 + dc + 0.5) * self.resolution
                cy = self.origin_y + (r0 + dr + 0.5) * self.resolution
                if math.hypot(cx - x, cy - y) <= radius:
                    self._set_occ(c0 + dc, r0 + dr)

    def write_pgm(self, path):
        # Flip vertically: PGM row 0 = top = highest y.
        with open(path, "wb") as fh:
            fh.write(b"P5\n%d %d\n255\n" % (self.width, self.height))
            for row in range(self.height - 1, -1, -1):
                fh.write(bytes(self._cells[row * self.width:(row + 1) * self.width]))

    def write_yaml(self, path, image_name):
        with open(path, "w") as fh:
            fh.write("image: %s\n" % image_name)
            fh.write("resolution: %.6f\n" % self.resolution)
            fh.write("origin: [%.6f, %.6f, 0.0]\n" % (self.origin_x, self.origin_y))
            fh.write("negate: 0\n")
            fh.write("occupied_thresh: 0.65\n")
            fh.write("free_thresh: 0.196\n")
