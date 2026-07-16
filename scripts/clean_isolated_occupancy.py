#!/usr/bin/env python3
"""Remove isolated occupied cells ("salt") from an OccupancyGrid PGM.

Rationale (2026-07-16 field, Task #13, [[project-campus-map-salt]]):
    docs/maps/campus/occupancy.pgm has ground-noise "salt" baked in from
    the M5-R mapping run (pre-Patchwork++). RViz overlay of /map + local
    costmap confirmed the salt positions match the fixed lethal cells
    seen during Nav2 operation. This script strips the salt by removing
    small isolated connected components without eroding continuous
    walls / buildings / curbs.

Method:
    1. Threshold the pgm to a binary "occupied" mask (pgm == 0 under the
       nav2 default with negate=0, occupied_thresh=0.65).
    2. Connected-components analysis (8-connectivity). This preserves
       corners and does not depend on wall thickness alone — a 1-cell
       wall that is 100 cells long survives with area=100.
    3. Drop every component whose area is below --min-blob-size. Real
       building walls at 5 cm resolution have hundreds of connected
       cells; salt is by definition isolated 1-3 px clusters.
    4. Optional closing (--closing-kernel K, K>=3, default off) to fill
       one-cell gaps in walls introduced by measurement noise. Left off
       by default because it can bridge across narrow openings.

Why connected-components instead of a straight morphological opening:
    Opening (erode+dilate) with a 3x3 kernel would erase any 2-px-wide
    wall along with the salt. Connected-components with a size threshold
    keeps a 1-px-wide but long wall (area >> min) while killing genuinely
    isolated dots (area <= 3).

Outputs:
    - Cleaned pgm at --output
    - Sibling yaml at <output-stem>.yaml with the `image:` field
      swapped to the cleaned pgm filename. Every other field is copied
      verbatim from --input-yaml (so origin/resolution/thresholds
      stay identical).
    - Diff PNG at --diff, RGB: original where unchanged, red where
      cells were removed.

Usage:
    scripts/clean_isolated_occupancy.py \\
        --input docs/maps/campus/occupancy.pgm \\
        --input-yaml docs/maps/campus/occupancy.yaml \\
        --output docs/maps/campus/occupancy_cleaned.pgm \\
        --diff docs/maps/campus/cleaning_diff.png

Add `--dry-run` to report counts without writing files.
"""

import argparse
import pathlib
import sys

import cv2
import numpy as np
import yaml
from PIL import Image


OCCUPIED_PGM_VALUE = 0  # matches nav2 map_server default with negate=0
FREE_PGM_VALUE = 254    # matches the value nav2's map_saver writes for free


def parse_args():
    p = argparse.ArgumentParser(
        description='Remove isolated occupied cells from an OccupancyGrid PGM.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--input', required=True, type=pathlib.Path,
                   help='Source pgm.')
    p.add_argument('--input-yaml', required=True, type=pathlib.Path,
                   help='Source yaml. Fields are copied to the output yaml '
                        'unchanged except `image:` which is set to the '
                        '--output basename.')
    p.add_argument('--output', required=True, type=pathlib.Path,
                   help='Cleaned pgm output path. A sibling yaml is written '
                        'at the same stem.')
    p.add_argument('--diff', required=True, type=pathlib.Path,
                   help='Diff PNG output path. Removed cells are painted '
                        'red on top of the original grid.')
    p.add_argument('--min-blob-size', type=int, default=4,
                   help='Drop connected components with fewer cells than '
                        'this (default 4 — kills every 1/2/3-cell isolated '
                        'blob, keeps everything larger).')
    p.add_argument('--closing-kernel', type=int, default=0,
                   help='If >= 3, run a morphological closing on the kept '
                        'occupied mask with a square kernel of this size. '
                        'Default off: closing can bridge narrow gaps '
                        'unintentionally.')
    p.add_argument('--dry-run', action='store_true',
                   help='Report counts, do not write any files.')
    return p.parse_args()


def load_pgm(path):
    with Image.open(path) as im:
        if im.mode != 'L':
            raise SystemExit(f'{path}: expected mode L (8-bit gray), '
                             f'got {im.mode!r}')
        return np.array(im)


def build_diff(original, removed_mask):
    """RGB diff: grayscale copy of original with removed cells in red."""
    rgb = np.stack([original, original, original], axis=-1).astype(np.uint8)
    rgb[removed_mask] = np.array([255, 0, 0], dtype=np.uint8)
    return rgb


def main():
    args = parse_args()

    if not args.input.is_file():
        raise SystemExit(f'--input not found: {args.input}')
    if not args.input_yaml.is_file():
        raise SystemExit(f'--input-yaml not found: {args.input_yaml}')

    pgm = load_pgm(args.input)
    occ_mask = (pgm == OCCUPIED_PGM_VALUE).astype(np.uint8)

    total_occ = int(occ_mask.sum())
    if total_occ == 0:
        raise SystemExit(f'{args.input}: no occupied cells (value=0). '
                         f'Nothing to clean.')

    # Connected components. Background = label 0.
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        occ_mask, connectivity=8)

    kept_mask = np.zeros_like(occ_mask)
    n_components = n_labels - 1  # exclude background
    n_removed_components = 0
    n_removed_cells = 0
    n_kept_components = 0

    for lbl in range(1, n_labels):
        area = int(stats[lbl, cv2.CC_STAT_AREA])
        if area < args.min_blob_size:
            n_removed_components += 1
            n_removed_cells += area
        else:
            kept_mask[labels == lbl] = 1
            n_kept_components += 1

    if args.closing_kernel >= 3:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT,
            (args.closing_kernel, args.closing_kernel))
        kept_mask = cv2.morphologyEx(kept_mask, cv2.MORPH_CLOSE, kernel)

    removed_mask = occ_mask.astype(bool) & (~kept_mask.astype(bool))

    cleaned = pgm.copy()
    cleaned[removed_mask] = FREE_PGM_VALUE

    # Summary — always print, regardless of dry-run.
    all_areas = stats[1:, cv2.CC_STAT_AREA]
    print(f'Input pgm         : {args.input}')
    print(f'  size            : {pgm.shape[1]} x {pgm.shape[0]}')
    print(f'  occupied cells  : {total_occ:,}')
    print(f'  components      : {n_components:,}')
    print(f'  size distribution (of occupied components, all-areas):')
    for lo, hi in [(1, 1), (2, 3), (4, 10), (11, 100),
                   (101, 1000), (1001, 10 ** 9)]:
        band = (all_areas >= lo) & (all_areas <= hi)
        n_band = int(band.sum())
        cells_band = int(all_areas[band].sum())
        label = (f'{lo:>5}..{hi:>10}' if hi < 10 ** 9
                 else f'{lo:>5}..     ∞')
        print(f'    area [{label}] : {n_band:>7,} comps, '
              f'{cells_band:>10,} cells')
    largest = int(all_areas.max()) if len(all_areas) else 0
    print(f'  largest comp    : {largest:,} cells '
          f'({"salt-only map — no continuous features" if largest < 20 else "may include real features"})')
    print(f'Threshold         : min area = {args.min_blob_size} cells')
    print(f'  removed comps   : {n_removed_components:,}')
    print(f'  removed cells   : {n_removed_cells:,} '
          f'({100 * n_removed_cells / total_occ:.2f}% of occupied)')
    print(f'  kept comps      : {n_kept_components:,}')
    if args.closing_kernel >= 3:
        print(f'Closing kernel    : {args.closing_kernel}x{args.closing_kernel}')

    if args.dry_run:
        print('(dry-run: no files written)')
        return 0

    # Write cleaned pgm.
    args.output.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(cleaned, mode='L').save(args.output)
    print(f'Wrote {args.output}')

    # Write sibling yaml. Copy every field from --input-yaml, set image.
    with args.input_yaml.open() as f:
        yaml_doc = yaml.safe_load(f)
    yaml_doc['image'] = args.output.name
    out_yaml = args.output.with_suffix('.yaml')
    with out_yaml.open('w') as f:
        yaml.safe_dump(yaml_doc, f, sort_keys=False)
    print(f'Wrote {out_yaml}')

    # Write diff PNG.
    diff = build_diff(pgm, removed_mask)
    args.diff.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(diff, mode='RGB').save(args.diff)
    print(f'Wrote {args.diff}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
