#!/usr/bin/env python3
"""Convert a FAST-LIO-saved PCD into a Nav2 occupancy grid (.pgm + .yaml).

The M5-b PCD includes scattered points from FAST-LIO drift segments,
so this script:

  1. Reads the PCD's binary float32 XYZ data.
  2. Crops to a configurable XY axis-aligned bounding box around the
     map origin (default ±20 m) to throw away the diverged outliers.
  3. Slices Z to the chair-relevant obstacle band (default 0.1 m to
     1.5 m above the LiDAR origin) so the floor and ceiling don't
     get baked into the costmap.
  4. Rasterises the surviving points into a 2D grid at a fixed
     resolution (default 0.05 m / cell).
  5. Marks every grid cell within line-of-sight from the world origin
     as free space — this is a cheap ray-cast (Bresenham) that gives
     Nav2 something to plan through. Cells the ray never reaches
     stay "unknown".
  6. Writes a PGM (binary P5) and the matching nav2_map_server YAML.

Usage:
    python3 scripts/pcd_to_occupancy_grid.py docs/m5-maps/lab.pcd \\
        docs/m5-maps/lab           # → docs/m5-maps/lab.pgm + lab.yaml

Optional flags:
    --xy-range METRES        crop XY to ±METRES around origin (default 20)
    --z-min METRES, --z-max METRES   chair-relevant Z slice
    --resolution METRES      grid resolution (default 0.05)
    --origin-frame STR       value placed in yaml frame_id (default "map")
"""

import argparse
import os
import struct
import sys

import numpy as np


PIX_OCCUPIED = 0       # black
PIX_FREE = 254         # almost white
PIX_UNKNOWN = 205      # the convention for unknown


def read_pcd_xyz(path):
    """Return an (N, 3) float32 numpy array of XYZ points from a binary PCD."""
    with open(path, 'rb') as f:
        header = []
        while True:
            line = f.readline().decode('ascii', errors='ignore').strip()
            header.append(line)
            if line.startswith('DATA'):
                fmt = line.split()[1]
                break
        if fmt != 'binary':
            raise ValueError(f'Only binary PCDs supported (got DATA {fmt!r}).')

        fields = next(h.split()[1:] for h in header if h.startswith('FIELDS'))
        sizes = [int(s) for s in next(h.split()[1:] for h in header if h.startswith('SIZE'))]
        counts = [int(s) for s in next(h.split()[1:] for h in header if h.startswith('COUNT'))]
        points = int(next(h.split()[1] for h in header if h.startswith('POINTS')))

        step = sum(s * c for s, c in zip(sizes, counts))
        raw = f.read()

    arr = np.frombuffer(raw, dtype=np.uint8).reshape(points, step)
    xyz = np.zeros((points, 3), dtype=np.float32)
    offset = 0
    for name, s, c in zip(fields, sizes, counts):
        if name == 'x':
            xyz[:, 0] = np.frombuffer(arr[:, offset:offset + 4].tobytes(), dtype=np.float32)
        elif name == 'y':
            xyz[:, 1] = np.frombuffer(arr[:, offset:offset + 4].tobytes(), dtype=np.float32)
        elif name == 'z':
            xyz[:, 2] = np.frombuffer(arr[:, offset:offset + 4].tobytes(), dtype=np.float32)
        offset += s * c
    return xyz


def bresenham_free_cells(grid, origin_px, hit_pixels):
    """For each hit pixel, mark cells along the ray from origin to (just before) the hit as free.

    grid is mutated in-place. origin_px is (col, row), hit_pixels is (N, 2) of (col, row).
    """
    ox, oy = origin_px
    H, W = grid.shape
    for hx, hy in hit_pixels:
        # Bresenham's line from (ox, oy) to (hx, hy), excluding endpoint
        dx = abs(hx - ox)
        dy = -abs(hy - oy)
        sx = 1 if ox < hx else -1
        sy = 1 if oy < hy else -1
        err = dx + dy
        x, y = ox, oy
        while True:
            if (x, y) == (hx, hy):
                break
            if 0 <= x < W and 0 <= y < H and grid[y, x] == PIX_UNKNOWN:
                grid[y, x] = PIX_FREE
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('pcd_path', help='input PCD (binary, with x/y/z fields)')
    p.add_argument('out_base', help='output basename without extension; .pgm and .yaml are written')
    p.add_argument('--xy-range', type=float, default=20.0,
                   help='crop XY to ±this many metres around origin (default 20)')
    p.add_argument('--z-min', type=float, default=0.1,
                   help='lower Z slice bound, m (default 0.1)')
    p.add_argument('--z-max', type=float, default=1.5,
                   help='upper Z slice bound, m (default 1.5)')
    p.add_argument('--resolution', type=float, default=0.05,
                   help='metres per grid cell (default 0.05)')
    p.add_argument('--no-free-cast', action='store_true',
                   help="skip Bresenham free-cell marking (only occupied + unknown)")
    args = p.parse_args()

    xyz = read_pcd_xyz(args.pcd_path)
    print(f'Loaded {len(xyz)} points from {args.pcd_path}', file=sys.stderr)
    print(f'  X: [{xyz[:, 0].min():+.2f}, {xyz[:, 0].max():+.2f}] m', file=sys.stderr)
    print(f'  Y: [{xyz[:, 1].min():+.2f}, {xyz[:, 1].max():+.2f}] m', file=sys.stderr)
    print(f'  Z: [{xyz[:, 2].min():+.2f}, {xyz[:, 2].max():+.2f}] m', file=sys.stderr)

    mask = (
        (np.abs(xyz[:, 0]) <= args.xy_range) &
        (np.abs(xyz[:, 1]) <= args.xy_range) &
        (xyz[:, 2] >= args.z_min) &
        (xyz[:, 2] <= args.z_max)
    )
    keep = xyz[mask]
    print(f'After XY ±{args.xy_range} m + Z [{args.z_min}, {args.z_max}] m: {len(keep)} points',
          file=sys.stderr)

    if len(keep) == 0:
        print('ERROR: no points survived filtering', file=sys.stderr)
        return 1

    # Grid spans ±xy_range; image origin (col=0, row=0) is bottom-left in world coords.
    res = args.resolution
    side = int(np.ceil(2 * args.xy_range / res))
    grid = np.full((side, side), PIX_UNKNOWN, dtype=np.uint8)
    origin_world = (-args.xy_range, -args.xy_range)  # lower-left corner in world

    # World origin (0, 0) maps to pixel:
    origin_px = (
        int(round((0 - origin_world[0]) / res)),       # col
        side - 1 - int(round((0 - origin_world[1]) / res)),  # row (image flipped)
    )

    # Hit pixels: each filtered point goes to one cell.
    cols = ((keep[:, 0] - origin_world[0]) / res).astype(int)
    rows = side - 1 - ((keep[:, 1] - origin_world[1]) / res).astype(int)
    valid = (cols >= 0) & (cols < side) & (rows >= 0) & (rows < side)
    cols, rows = cols[valid], rows[valid]

    # Free space first (so OCCUPIED stamps win the overlay).
    if not args.no_free_cast:
        unique_hits = np.unique(np.stack([cols, rows], axis=1), axis=0)
        print(f'Ray-casting from origin to {len(unique_hits)} unique hit cells '
              f'(Bresenham, can take a moment)...', file=sys.stderr)
        bresenham_free_cells(grid, origin_px, unique_hits)

    grid[rows, cols] = PIX_OCCUPIED

    n_occ = int((grid == PIX_OCCUPIED).sum())
    n_free = int((grid == PIX_FREE).sum())
    n_unk = int((grid == PIX_UNKNOWN).sum())
    total = grid.size
    print(f'Grid: {side}x{side} cells ({side*res:.1f} m / side), '
          f'occupied={n_occ} ({100*n_occ/total:.1f}%), '
          f'free={n_free} ({100*n_free/total:.1f}%), '
          f'unknown={n_unk} ({100*n_unk/total:.1f}%)', file=sys.stderr)

    # Write PGM (binary P5).
    pgm_path = args.out_base + '.pgm'
    with open(pgm_path, 'wb') as f:
        f.write(f'P5\n{side} {side}\n255\n'.encode('ascii'))
        f.write(grid.tobytes())
    print(f'Wrote {pgm_path}')

    # Write Nav2 map_server YAML.
    yaml_path = args.out_base + '.yaml'
    pgm_basename = os.path.basename(pgm_path)
    yaml_text = (
        f'image: {pgm_basename}\n'
        f'resolution: {res}\n'
        f'origin: [{origin_world[0]:.3f}, {origin_world[1]:.3f}, 0.0]\n'
        f'negate: 0\n'
        f'occupied_thresh: 0.65\n'
        f'free_thresh: 0.25\n'
    )
    with open(yaml_path, 'w') as f:
        f.write(yaml_text)
    print(f'Wrote {yaml_path}')

    return 0


if __name__ == '__main__':
    raise SystemExit(main())
