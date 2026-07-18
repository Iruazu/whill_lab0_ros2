#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""gen_auto_free_mask.py — auto-generate a free_mask sidecar for 1-cell
strictly-isolated OCCUPIED cells inside the operator's roadway_mask.

Rule 2 application (INPUT, not OUTPUT modification): the composer treats
this sidecar exactly like a human-painted free_mask, i.e. it erases
machine_occ where painted (audit trail via erased_by_free.png). No
morphological operation is applied to the OUTPUT pgm — Rule 1 preserved.
This script's job is only to identify which cells the operator would
have painted themselves under Rule 2's "auto-remove is 1-cell strictly
isolated dust only" rule, and to save that painted intent as a PNG the
composer consumes.

Semantics:
  A cell qualifies for free_mask_auto if
    (i) it is machine_occ (union of Stage 1 occupied_step + occupied_structure)
   (ii) it is INSIDE the operator's roadway_mask (any painted cell)
  (iii) its 8-neighbourhood contains NO OTHER machine_occ cell

Also outputs salt_candidates_2to3.csv — coordinates of the 2-3 cell
blobs INSIDE roadway_mask. These are NOT auto-deleted (Rule 2 strict:
only truly isolated 1-cell). The operator inspects the CSV later and
decides case-by-case whether to paint free_mask or leave occupied.

Usage:

    scripts/gen_auto_free_mask.py \\
        --layers-yaml            docs/maps/campus/v2/v2_layers.yaml \\
        --roadway-mask           docs/maps/campus/v2/roadway_mask.png \\
        --output-free-mask-auto  docs/maps/campus/v2/free_mask_auto.png \\
        --output-salt-csv        docs/maps/campus/v2/salt_candidates_2to3.csv

Then pass free_mask_auto.png to compose_occupancy.py via
--free-mask-auto (see compose docstring for OR semantics with human
free_mask).
"""

import argparse
import csv
import pathlib
import sys

import cv2
import numpy as np
import yaml
from PIL import Image


COLOR_AUTO_FREE = (255, 255, 255, 255)   # opaque white = painted


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__.split('\n\n', 1)[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--layers-yaml', required=True, type=pathlib.Path,
                   help='Stage 1 v2_layers.yaml (locates step + structure).')
    p.add_argument('--roadway-mask', required=True, type=pathlib.Path,
                   help='Operator-painted roadway whitelist PNG. Only cells '
                        'INSIDE this mask are auto-freed. If the mask is empty '
                        '(fresh sidecar), the script produces empty outputs '
                        'with a warning.')
    p.add_argument('--output-free-mask-auto', required=True, type=pathlib.Path,
                   help='RGBA PNG output. White opaque at auto-free cells, '
                        'transparent elsewhere. Fed to compose_occupancy.py '
                        'via --free-mask-auto.')
    p.add_argument('--output-salt-csv', required=True, type=pathlib.Path,
                   help='CSV output listing 2-3 cell blob centroids inside '
                        'roadway (not auto-deleted; for operator review).')
    p.add_argument('--mask-threshold', type=int, default=128,
                   help='Grayscale threshold on the roadway_mask (0-255). '
                        'Default 128 — matches composer default.')
    return p.parse_args()


def load_layers_manifest(path):
    with path.open() as f:
        return yaml.safe_load(f)


def load_rgba_opaque(path, expected_hw):
    im = Image.open(path)
    a = np.array(im)
    if a.shape[:2] != expected_hw:
        raise SystemExit(f'{path}: shape {a.shape[:2]} != expected {expected_hw}')
    if im.mode != 'RGBA':
        raise SystemExit(f'{path}: expected RGBA, got {im.mode}')
    return a[..., 3] == 255


def load_sidecar_mask(path, expected_hw, threshold):
    im = Image.open(path)
    if im.mode == 'RGBA':
        alpha = np.array(im.getchannel('A'))
        gray = np.array(im.convert('L'))
        painted = (gray >= threshold) & (alpha > 0)
    else:
        gray = np.array(im.convert('L'))
        painted = gray >= threshold
    if painted.shape != expected_hw:
        raise SystemExit(f'{path}: shape {painted.shape} != expected {expected_hw}')
    return painted


def main():
    args = parse_args()
    manifest = load_layers_manifest(args.layers_yaml)
    W = int(manifest['grid']['width'])
    H = int(manifest['grid']['height'])
    resolution = float(manifest['grid']['resolution'])
    origin = manifest['grid']['origin']
    hw = (H, W)
    layers_dir = args.layers_yaml.parent

    # machine_occ = union of step + structure opaque cells.
    machine_occ = np.zeros(hw, dtype=bool)
    for key in ('occupied_step', 'occupied_structure'):
        if key not in manifest['layers']:
            print(f'[warn] {key} missing from manifest')
            continue
        path = layers_dir / manifest['layers'][key]
        machine_occ |= load_rgba_opaque(path, hw)
    print(f'[in ] machine_occ         : {int(machine_occ.sum()):>10,} cells')

    roadway = load_sidecar_mask(args.roadway_mask, hw, args.mask_threshold)
    n_roadway = int(roadway.sum())
    print(f'[in ] roadway_mask        : {n_roadway:>10,} cells '
          f'(L≥{args.mask_threshold})')
    if n_roadway == 0:
        print(f'[warn] roadway_mask is EMPTY. free_mask_auto and salt CSV '
              f'will be empty. Paint the roadway_mask first.')

    # 8-connected labeling on machine_occ (raw — Rule 2 strict, no
    # pre-dilation). Then filter by component size.
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(
        machine_occ.astype(np.uint8), connectivity=8)
    areas = stats[:, cv2.CC_STAT_AREA]

    # Build per-cell area lookup (int32 with 0 for background).
    per_cell_area = np.zeros(hw, dtype=np.int32)
    per_cell_area[machine_occ] = areas[labels[machine_occ]]

    # (1) 1-cell isolated INSIDE roadway → free_mask_auto.
    auto_free = machine_occ & roadway & (per_cell_area == 1)
    n_auto = int(auto_free.sum())
    print(f'[out] 1-cell isolated (in roadway): {n_auto:>10,} cells → free_mask_auto')

    # (2) 2-3 cell blobs INSIDE roadway → CSV (not deleted).
    blob23_cell = machine_occ & roadway & ((per_cell_area == 2) | (per_cell_area == 3))
    # Component ids of these cells:
    blob23_labels_in_roadway = np.unique(labels[blob23_cell])
    blob23_labels_in_roadway = blob23_labels_in_roadway[blob23_labels_in_roadway != 0]
    n_blob23_components = int(blob23_labels_in_roadway.size)
    n_blob23_cells = int(blob23_cell.sum())
    print(f'[out] 2-3 cell blobs (in roadway) : {n_blob23_components:>10,} '
          f'components / {n_blob23_cells:,} cells → CSV (NOT auto-deleted)')

    # ---- Write free_mask_auto.png (RGBA) ----
    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    rgba[auto_free] = COLOR_AUTO_FREE
    args.output_free_mask_auto.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(rgba, 'RGBA').save(args.output_free_mask_auto)
    print(f'[out] {args.output_free_mask_auto}')

    # ---- Write salt_candidates_2to3.csv ----
    ox, oy = float(origin[0]), float(origin[1])
    args.output_salt_csv.parent.mkdir(parents=True, exist_ok=True)
    with args.output_salt_csv.open('w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['component_id', 'size_cells', 'centroid_px', 'centroid_py',
                    'centroid_map_x', 'centroid_map_y'])
        for lbl in blob23_labels_in_roadway:
            i = int(lbl)
            cx = float(stats[i, cv2.CC_STAT_LEFT] + stats[i, cv2.CC_STAT_WIDTH] / 2.0)
            cy = float(stats[i, cv2.CC_STAT_TOP] + stats[i, cv2.CC_STAT_HEIGHT] / 2.0)
            size = int(areas[i])
            x_map = ox + cx * resolution
            y_map = oy + (H - 1 - cy) * resolution
            w.writerow([i, size, f'{cx:.1f}', f'{cy:.1f}',
                        f'{x_map:.3f}', f'{y_map:.3f}'])
    print(f'[out] {args.output_salt_csv}')

    return 0


if __name__ == '__main__':
    sys.exit(main())
