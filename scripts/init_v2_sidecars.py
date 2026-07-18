#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Initialise empty sidecar PNGs for v2 pipeline (keepout_mask, free_mask).

Both files are fully-transparent RGBA at the exact pixel dimensions of the
Stage 1 grid (read from v2_layers.yaml). Existing files are NOT overwritten
unless --force is passed — the operator's paint work is precious.

Usage:
    scripts/init_v2_sidecars.py --layers-yaml docs/maps/campus/v2/v2_layers.yaml
"""

import argparse
import pathlib
import sys

import numpy as np
import yaml
from PIL import Image


SIDECAR_NAMES = ('keepout_mask.png', 'free_mask.png')


def main():
    p = argparse.ArgumentParser(description=__doc__.split('\n\n', 1)[0])
    p.add_argument('--layers-yaml', required=True, type=pathlib.Path,
                   help='Stage 1 v2_layers.yaml manifest — resolves the pixel grid '
                        'AND the output directory (sidecars are placed as siblings).')
    p.add_argument('--force', action='store_true',
                   help='Overwrite existing sidecar files. Off by default so a '
                        'partially-painted mask is never wiped by mistake.')
    args = p.parse_args()

    if not args.layers_yaml.is_file():
        raise SystemExit(f'--layers-yaml not found: {args.layers_yaml}')
    with args.layers_yaml.open() as f:
        m = yaml.safe_load(f)
    W = int(m['grid']['width'])
    H = int(m['grid']['height'])
    out_dir = args.layers_yaml.parent
    print(f'grid    : {W} x {H} @ {m["grid"]["resolution"]} m/px')
    print(f'out_dir : {out_dir}')

    empty = np.zeros((H, W, 4), dtype=np.uint8)   # fully transparent RGBA
    for name in SIDECAR_NAMES:
        path = out_dir / name
        if path.exists() and not args.force:
            print(f'[skip] {path} exists (use --force to overwrite)')
            continue
        Image.fromarray(empty, 'RGBA').save(path)
        print(f'[out ] {path}  (empty transparent RGBA {W}x{H})')

    return 0


if __name__ == '__main__':
    sys.exit(main())
