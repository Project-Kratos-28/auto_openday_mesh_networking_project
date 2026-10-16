#!/usr/bin/env python3
"""
generate_crater.py - makes the Mars crater terrain as a 3D mesh (OBJ file) for Gazebo.

Run from the package folder:   python3 scripts/generate_crater.py
Writes:                        models/mars_crater/meshes/crater.obj
Then rebuild the workspace:    colcon build --symlink-install

Coordinates (metres):
  - crater centre = (0, 0); crater FLOOR is at z = 0 (so the rover spawns at the origin)
  - the flat plain outside the crater is at z = DEPTH (25 m)
  - the rim sticks up RIM_HEIGHT (8 m) above the plain
Change the numbers below to resize the crater, then re-run.
"""
import os
import numpy as np

# ---------------- settings you can change ----------------
SIZE = 480.0          # terrain is SIZE x SIZE metres, centred on the crater
GRID = 161            # points per side (more = smoother but heavier). 161 -> 3 m spacing
RIM_RADIUS = 100.0    # crater radius to the top of the rim (m)  -> 200 m wide crater
DEPTH = 25.0          # floor depth below the outside plain (m)
RIM_HEIGHT = 8.0      # rim height above the plain (m)
EJECTA_RADIUS = 170.0 # where the raised ground around the rim fades back to the plain (m)
BUMPS = 0.4           # height of small random Mars bumps (m)
BASE_XY = (200.0, 0.0)  # base station spot: kept flat so the station sits level
SEED = 7              # change for different bumps
# ----------------------------------------------------------


def smoothstep(a, b, x):
    """0 when x<=a, 1 when x>=b, smooth S-curve in between."""
    t = np.clip((x - a) / (b - a), 0.0, 1.0)
    return t * t * (3 - 2 * t)


def crater_height(x, y):
    """Height of the crater shape (no bumps), measured from the PLAIN level."""
    r = np.hypot(x, y)
    inside = -DEPTH + (DEPTH + RIM_HEIGHT) * (r / RIM_RADIUS) ** 2.5          # bowl rising to the rim
    outside = RIM_HEIGHT * (1 - smoothstep(RIM_RADIUS, EJECTA_RADIUS, r))     # rim slope fading to plain
    return np.where(r <= RIM_RADIUS, inside, outside)


def bumps(x, y):
    """Small rolling bumps, flattened near the rover spawn and the base station."""
    rng = np.random.default_rng(SEED)
    x, y = np.asarray(x, dtype=float), np.asarray(y, dtype=float)
    z = np.zeros(np.shape(x))
    for _ in range(12):                                   # add 12 random gentle waves
        kx, ky = rng.uniform(-1, 1, 2) * 2 * np.pi / rng.uniform(15, 60)
        z += np.sin(kx * x + ky * y + rng.uniform(0, 2 * np.pi))
    z *= BUMPS / 3.0
    z *= smoothstep(12, 30, np.hypot(x, y))                                   # flat around rover
    z *= smoothstep(15, 35, np.hypot(x - BASE_XY[0], y - BASE_XY[1]))         # flat around base
    return z


def terrain_z(x, y):
    """Final height used by Gazebo (crater floor = 0)."""
    return crater_height(x, y) + DEPTH + bumps(x, y)


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    out = os.path.join(here, '..', 'models', 'mars_crater', 'meshes', 'crater.obj')
    os.makedirs(os.path.dirname(out), exist_ok=True)

    xs = np.linspace(-SIZE / 2, SIZE / 2, GRID)
    X, Y = np.meshgrid(xs, xs, indexing='ij')             # X[i, j], Y[i, j]
    Z = terrain_z(X, Y)

    # surface normals (for smooth lighting) from the slope in x and y
    step = xs[1] - xs[0]
    dzdx, dzdy = np.gradient(Z, step, step)
    N = np.dstack([-dzdx, -dzdy, np.ones_like(Z)])
    N /= np.linalg.norm(N, axis=2, keepdims=True)

    with open(out, 'w') as f:
        f.write('# Mars crater terrain, made by generate_crater.py\n')
        f.write('o crater\n')
        for i in range(GRID):
            for j in range(GRID):
                f.write(f'v {X[i, j]:.2f} {Y[i, j]:.2f} {Z[i, j]:.3f}\n')
        for i in range(GRID):
            for j in range(GRID):
                n = N[i, j]
                f.write(f'vn {n[0]:.4f} {n[1]:.4f} {n[2]:.4f}\n')
        idx = lambda i, j: i * GRID + j + 1               # OBJ counts from 1
        for i in range(GRID - 1):
            for j in range(GRID - 1):
                a, b, c, d = idx(i, j), idx(i + 1, j), idx(i + 1, j + 1), idx(i, j + 1)
                f.write(f'f {a}//{a} {b}//{b} {c}//{c}\n')   # two triangles per square,
                f.write(f'f {a}//{a} {c}//{c} {d}//{d}\n')   # counter-clockwise = facing up

    print(f'Wrote {out}')
    print(f'  crater floor z = {terrain_z(np.array(0.0), np.array(0.0)):.2f} m, '
          f'rim top z = {DEPTH + RIM_HEIGHT:.1f} m, plain z = {DEPTH:.1f} m')
    print(f'  base station ground z = {terrain_z(np.array(BASE_XY[0]), np.array(BASE_XY[1])):.2f} m')


if __name__ == '__main__':
    main()
