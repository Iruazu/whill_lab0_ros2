#!/usr/bin/env python3
"""Downscale an occupancy pgm into a web-servable png + metadata for the UI.

The demo map (docs/maps/campus/occupancy_cleaned.pgm) is 6640x6295 @
0.05 m/px = ~41 MB — far too heavy to hand a tablet browser as a map
background. This script rasterises it down by an integer factor and writes:

  src/whill_dispatch/web/map.png        the downscaled background
  src/whill_dispatch/web/map_meta.json  origin + effective resolution + size

app.js reads map_meta.json and converts map-frame metres to png pixels with
the standard occupancy-grid transform (origin at the pgm's bottom-left, y
flipped for image coordinates):

    px = (mx - origin_x) / res_eff
    py = height_px - (my - origin_y) / res_eff       # y flip

Downscaling by an integer factor F leaves the origin unchanged and scales
the effective resolution to res * F, so the transform above works on the
small png with res_eff written into the json — no separate scale needed.

Idempotent: re-running overwrites both outputs. Safe to re-run whenever the
cleaned map is regenerated (Task #14 v2 map, etc.).

Usage:
    python3 scripts/m7_make_web_map.py
    python3 scripts/m7_make_web_map.py --factor 8
    python3 scripts/m7_make_web_map.py \\
        --pgm docs/maps/campus/occupancy_cleaned.pgm \\
        --yaml docs/maps/campus/occupancy_cleaned.yaml \\
        --out-dir src/whill_dispatch/web
"""

import argparse
import json
import os

import yaml
from PIL import Image

# The cleaned pgm is larger than PIL's decompression-bomb guard. It is a
# trusted local artifact, so lift the cap for this script only.
Image.MAX_IMAGE_PIXELS = None

_REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..'))


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument(
        '--pgm',
        default=os.path.join(
            _REPO_ROOT, 'docs', 'maps', 'campus', 'occupancy_cleaned.pgm'))
    ap.add_argument(
        '--yaml',
        default=os.path.join(
            _REPO_ROOT, 'docs', 'maps', 'campus', 'occupancy_cleaned.yaml'))
    ap.add_argument(
        '--out-dir',
        default=os.path.join(_REPO_ROOT, 'src', 'whill_dispatch', 'web'))
    ap.add_argument(
        '--factor', type=int, default=8,
        help='integer downscale factor (default 8: 6640x6295 -> 830x786, '
             'res 0.05 -> 0.4 m/px)')
    args = ap.parse_args()

    with open(args.yaml) as f:
        meta = yaml.safe_load(f)
    res = float(meta['resolution'])
    origin = [float(v) for v in meta['origin']]

    img = Image.open(args.pgm)
    w, h = img.size
    new_size = (max(1, w // args.factor), max(1, h // args.factor))
    # BOX averaging keeps thin occupied lines visible after shrink better
    # than NEAREST, which would drop most single-pixel walls at factor 8.
    small = img.resize(new_size, Image.BOX)

    os.makedirs(args.out_dir, exist_ok=True)
    png_path = os.path.join(args.out_dir, 'map.png')
    small.save(png_path, optimize=True)

    res_eff = res * args.factor
    meta_out = {
        'image': 'map.png',
        'width': new_size[0],
        'height': new_size[1],
        'resolution': res_eff,
        'origin': origin,
        'source_pgm': os.path.relpath(args.pgm, _REPO_ROOT),
        'downscale_factor': args.factor,
    }
    json_path = os.path.join(args.out_dir, 'map_meta.json')
    with open(json_path, 'w') as f:
        json.dump(meta_out, f, indent=2)

    png_kib = os.path.getsize(png_path) / 1024.0
    print(f'wrote {png_path} ({new_size[0]}x{new_size[1]}, {png_kib:.0f} KiB)')
    print(f'wrote {json_path} (res_eff {res_eff:.3f} m/px, origin {origin})')


if __name__ == '__main__':
    main()
