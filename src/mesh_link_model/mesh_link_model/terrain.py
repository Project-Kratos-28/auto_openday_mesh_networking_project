"""Terrain height map + line-of-sight check.

Loads the crater mesh (crater.obj) that generate_crater.py makes, turns it into a
grid of heights, and answers: "is the straight line between A and B above the ground?"
"""
import math
import os
import warnings

import numpy as np


class Terrain:
    def __init__(self, grid, x0, y0, cell, source):
        self.grid = grid          # grid[i, j] = ground height at (x0 + i*cell, y0 + j*cell)
        self.x0, self.y0, self.cell = x0, y0, cell
        self.source = source      # where the heights came from (printed at start)

    # -------------------------------------------------------------- loading
    @classmethod
    def from_obj(cls, path):
        """Read the 'v x y z' lines of an OBJ mesh into a height grid."""
        verts = []
        with open(path) as f:
            for line in f:
                if line.startswith("v "):
                    p = line.split()
                    verts.append((float(p[1]), float(p[2]), float(p[3])))
        if len(verts) < 4:
            raise ValueError(f"{path}: no vertices found")
        v = np.array(verts)
        # the "up" axis is the one with the smallest spread (crater is ~33 m tall, 480 m wide)
        up = int(np.argmin(v.max(0) - v.min(0)))
        if up == 2:
            x, y, z = v[:, 0], v[:, 1], v[:, 2]      # Z-up mesh (normal for Gazebo)
        elif up == 1:
            x, y, z = v[:, 0], -v[:, 2], v[:, 1]     # Y-up mesh -> convert
        else:
            x, y, z = v[:, 1], v[:, 2], v[:, 0]
        ux, uy = np.unique(np.round(x, 2)), np.unique(np.round(y, 2))
        if len(ux) * len(uy) == len(v) and len(ux) > 1:          # a regular grid (normal case)
            cell = float(np.median(np.diff(ux)))
        else:                                                    # unknown layout: guess square grid
            n = max(2, int(round(math.sqrt(len(v)))))
            cell = max(x.max() - x.min(), y.max() - y.min()) / (n - 1)
        nx = int(round((x.max() - x.min()) / cell)) + 1
        ny = int(round((y.max() - y.min()) / cell)) + 1
        grid = np.full((nx, ny), np.nan)
        ix = np.clip(np.round((x - x.min()) / cell).astype(int), 0, nx - 1)
        iy = np.clip(np.round((y - y.min()) / cell).astype(int), 0, ny - 1)
        np.fmax.at(grid, (ix, iy), z)                # highest vertex in each cell
        grid = _fill_holes(grid)
        return cls(grid, float(x.min()), float(y.min()), cell, path)

    @classmethod
    def from_generator(cls, script):
        """Backup: build the grid from generate_crater.py's height() formula."""
        import importlib.util
        spec = importlib.util.spec_from_file_location("generate_crater", script)
        gen = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(gen)
        cell = 2.5
        xs = np.arange(-gen.SIZE_X / 2, gen.SIZE_X / 2 + cell / 2, cell)
        ys = np.arange(-gen.SIZE_Y / 2, gen.SIZE_Y / 2 + cell / 2, cell)
        grid = np.array([[gen.height(x, y) for y in ys] for x in xs])
        return cls(grid, float(xs[0]), float(ys[0]), cell, f"formula in {script}")

    @classmethod
    def load(cls, path=None):
        """Use crater.obj if we can find it, otherwise the built-in crater."""
        for p in [path] + candidate_paths():
            if p and os.path.isfile(p):
                return cls.from_obj(p)
        script = os.path.expanduser("~/mars_ws/src/mars_sim/scripts/generate_crater.py")
        if os.path.isfile(script):
            return cls.from_generator(script)
        raise FileNotFoundError("crater.obj not found - run generate_crater.py and colcon build")

    # -------------------------------------------------------------- queries
    def height_at(self, x, y):
        """Ground height at (x, y), bilinear between grid points. Works on arrays too."""
        g = self.grid
        fx = np.clip((np.asarray(x, float) - self.x0) / self.cell, 0, g.shape[0] - 1.0001)
        fy = np.clip((np.asarray(y, float) - self.y0) / self.cell, 0, g.shape[1] - 1.0001)
        i, j = fx.astype(int), fy.astype(int)
        tx, ty = fx - i, fy - j
        return (g[i, j] * (1 - tx) * (1 - ty) + g[i + 1, j] * tx * (1 - ty)
                + g[i, j + 1] * (1 - tx) * ty + g[i + 1, j + 1] * tx * ty)

    def ground(self, x, y):
        """Ground height at one point, as a plain float."""
        return float(self.height_at(x, y))

    def line_of_sight(self, a, b, clearance=0.5, step=1.0):
        """True if the straight line a->b (x, y, z) stays `clearance` m above the ground."""
        a, b = np.asarray(a, float), np.asarray(b, float)
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        n = max(2, int(d / step))
        t = np.arange(1, n) / n                      # sample points, not the two ends
        pts = a[None, :] + t[:, None] * (b - a)[None, :]
        ground = self.height_at(pts[:, 0], pts[:, 1])
        return bool(np.all(ground < pts[:, 2] - clearance))

    def describe(self):
        g = self.grid
        return (f"{self.source}: {g.shape[0]}x{g.shape[1]} points, "
                f"x {self.x0:.0f}..{self.x0 + (g.shape[0] - 1) * self.cell:.0f} m, "
                f"height {np.min(g):.1f}..{np.max(g):.1f} m")


def _fill_holes(grid):
    """Give empty grid cells the height of a neighbour (in case the mesh isn't a perfect grid)."""
    for _ in range(200):
        holes = np.isnan(grid)
        if not holes.any():
            break
        p = np.pad(grid, 1, constant_values=np.nan)
        nb = np.stack([p[:-2, 1:-1], p[2:, 1:-1], p[1:-1, :-2], p[1:-1, 2:]])
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", RuntimeWarning)
            fill = np.nanmax(nb, axis=0)
        grid[holes] = fill[holes]
    return np.nan_to_num(grid, nan=0.0)


def candidate_paths():
    """Places crater.obj can be (installed share folder first, then the source folder)."""
    rel = os.path.join("models", "mars_crater", "meshes", "crater.obj")
    paths = []
    try:
        from ament_index_python.packages import get_package_share_directory
        paths.append(os.path.join(get_package_share_directory("mars_sim"), rel))
    except Exception:
        pass
    paths.append(os.path.join(os.path.expanduser("~/mars_ws/src/mars_sim"), rel))
    return paths
