#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""compose_occupancy.py — Stage 2 of the PCD → OccupancyGrid v2 pipeline.

Composes Stage 1's layer-separated evidence (occupied_step,
occupied_structure, free_evidence) with two optional hand-painted
sidecar masks (keepout_mask, free_mask) into a single Nav2-ready
pgm + yaml. Reads scripts/pcd_to_occupancy_v2.py's v2_layers.yaml
manifest so the caller only needs to point at the Stage 1 output
directory + sidecars.

Composition rule (Rule 3 of the v2 design, protecting machine
occupied from accidental overpaint):

    occupied_final = machine_occupied ∪ (keepout_mask ∩ ¬free_mask)
    free_final     = (machine_free ∪ free_mask) ∩ ¬occupied_final
    pgm[occupied_final] = 0     (OCCUPIED)
    pgm[free_final]     = 254   (FREE)
    everywhere else     = 205   (UNKNOWN)

Where:
  * machine_occupied  = union of the opaque cells in occupied_step +
                        occupied_structure PNGs.
  * machine_free      = union of the non-transparent cells in
                        free_evidence PNG.
  * keepout_mask      = optional PNG; cells with grayscale ≥ threshold
                        AND (alpha > 0 if RGBA) become human-added
                        occupied.
  * free_mask         = optional PNG; same threshold rule; adds free
                        AND is allowed to override keepout_mask, but
                        NEVER overrides machine_occupied.

Key properties:
  * OCCUPIED never grows beyond raw evidence positions (Rule 1) —
    the composer never dilates.
  * Machine-called occupied cannot be turned into free by any human
    mask (Rule 3 protection) — misclick-safe.
  * free_mask CAN undo keepout_mask, so operators can iterate.
  * All masks share the pgm's exact pixel grid; a size mismatch is
    a hard error, not silently resampled.
  * When both sidecars are absent this script produces a "machine-
    only" pgm — useful for A/B comparison against v1 output.

Usage:

    # machine-only (no human input yet)
    scripts/compose_occupancy.py \\
        --layers-yaml docs/maps/campus/v2/v2_layers.yaml \\
        --output-pgm  docs/maps/campus/v2/final.pgm

    # with human sidecars once GIMP work is done
    scripts/compose_occupancy.py \\
        --layers-yaml docs/maps/campus/v2/v2_layers.yaml \\
        --keepout-mask docs/maps/campus/v2/keepout_mask.png \\
        --free-mask    docs/maps/campus/v2/free_mask.png \\
        --output-pgm   docs/maps/campus/v2/final.pgm

The sibling <output-pgm>.yaml is written automatically with the same
resolution / origin / thresholds as the Stage 1 grid.
"""

import argparse
import pathlib
import sys
import time

import numpy as np
import yaml
from PIL import Image


OCCUPIED = 0
UNKNOWN = 205
FREE = 254


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__.split('\n\n', 1)[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--layers-yaml', required=True, type=pathlib.Path,
                   help='Path to Stage 1 v2_layers.yaml manifest.')
    p.add_argument('--output-pgm', required=True, type=pathlib.Path,
                   help='Final pgm output. Sibling yaml is written at the '
                        'same stem with .yaml suffix.')
    p.add_argument('--output-yaml', type=pathlib.Path,
                   help='Explicit output yaml path (default: --output-pgm '
                        'with .yaml suffix).')

    p.add_argument('--keepout-mask', type=pathlib.Path,
                   help='Grayscale / RGBA PNG (same pixel grid as Stage 1 '
                        'layers). Bright pixels add OCCUPIED, subject to '
                        '--free-mask override.')
    p.add_argument('--free-mask', type=pathlib.Path,
                   help='Grayscale / RGBA PNG. Bright pixels add FREE and '
                        'can undo --keepout-mask, but never override '
                        'machine-called occupied (Rule 3 protection).')
    p.add_argument('--mask-threshold', type=int, default=128,
                   help='Grayscale threshold on sidecar masks above which a '
                        'pixel counts as painted (0-255). Default 128 — '
                        'tolerant to GIMP brush anti-aliasing edges.')

    p.add_argument('--occupied-thresh', type=float, default=0.65,
                   help='Emitted in the output yaml. Nav2 map_server default.')
    p.add_argument('--free-thresh', type=float, default=0.196,
                   help='Emitted in the output yaml. Nav2 map_server default.')
    p.add_argument('--negate', type=int, default=0,
                   help='Emitted in the output yaml. Nav2 map_server default.')

    p.add_argument('--include-free-evidence', action='store_true', default=True,
                   help='(default on) Treat free_evidence.png as machine_free. '
                        'Turn off with --no-include-free-evidence to require '
                        'the human to paint every FREE cell explicitly.')
    p.add_argument('--no-include-free-evidence', dest='include_free_evidence',
                   action='store_false')

    p.add_argument('--dry-run', action='store_true',
                   help='Compute + report counts, do not write pgm/yaml.')
    return p.parse_args()


def load_layers_manifest(path):
    with path.open() as f:
        doc = yaml.safe_load(f)
    grid = doc['grid']
    W = int(grid['width'])
    H = int(grid['height'])
    resolution = float(grid['resolution'])
    origin = list(grid['origin'])
    return doc, W, H, resolution, origin


def load_rgba_opaque(path, expected_hw):
    """Load PNG and return a bool mask of opaque pixels (alpha == 255).

    For an RGBA PNG produced by pcd_to_occupancy_v2.py, opaque == 255
    marks the machine's occupied cells and non-zero-but-not-255 marks
    free_evidence semi-transparent cells. Callers pick which they want.
    """
    im = Image.open(path)
    a = np.array(im)
    if a.shape[:2] != expected_hw:
        raise SystemExit(f'{path}: shape {a.shape[:2]} != expected {expected_hw}. '
                         f'Stage 1 layers and sidecars must share exact pixel dims.')
    if im.mode != 'RGBA':
        raise SystemExit(f'{path}: expected RGBA, got {im.mode}. '
                         f'Not a Stage 1 layer PNG?')
    return a[..., 3] == 255


def load_rgba_nonempty(path, expected_hw):
    """Return a bool mask of ANY non-transparent pixel (alpha > 0)."""
    im = Image.open(path)
    a = np.array(im)
    if a.shape[:2] != expected_hw:
        raise SystemExit(f'{path}: shape {a.shape[:2]} != expected {expected_hw}.')
    if im.mode != 'RGBA':
        raise SystemExit(f'{path}: expected RGBA, got {im.mode}.')
    return a[..., 3] > 0


def load_sidecar_mask(path, expected_hw, threshold):
    """Load a hand-painted mask PNG and return a bool array of painted cells.

    Supported modes:
      L / RGB / P → painted = L ≥ threshold (any bright pixel wins)
      RGBA        → painted = L ≥ threshold AND alpha > 0

    L is computed via PIL.Image.convert('L') which uses the ITU-R 601-2
    luma formula (0.299R + 0.587G + 0.114B). Recommend WHITE (255,255,255)
    for the paint colour — pure red at L=76 falls below the default
    threshold of 128 and would be silently ignored.
    """
    im = Image.open(path)
    if im.mode == 'RGBA':
        alpha = np.array(im.getchannel('A'))
        gray = np.array(im.convert('L'))
        painted = (gray >= threshold) & (alpha > 0)
    else:
        gray = np.array(im.convert('L'))
        painted = gray >= threshold
    if painted.shape != expected_hw:
        raise SystemExit(f'{path}: shape {painted.shape} != expected {expected_hw}. '
                         f'Sidecar masks must share exact pixel dims with Stage 1.')
    return painted


def main():
    args = parse_args()
    if not args.layers_yaml.is_file():
        raise SystemExit(f'--layers-yaml not found: {args.layers_yaml}')

    t0 = time.time()
    print(f'== compose_occupancy ==')
    print(f'layers-yaml : {args.layers_yaml}')
    print(f'keepout-mask: {args.keepout_mask}')
    print(f'free-mask   : {args.free_mask}')
    print(f'output-pgm  : {args.output_pgm}')

    manifest, W, H, resolution, origin = load_layers_manifest(args.layers_yaml)
    hw = (H, W)
    print(f'grid        : {W} x {H}, origin={origin}, resolution={resolution}')

    layers_dir = args.layers_yaml.parent
    layers = manifest['layers']

    # ---- Machine-occupied = union of step + structure opaque cells ----
    machine_occ = np.zeros(hw, dtype=bool)
    for key in ('occupied_step', 'occupied_structure'):
        if key not in layers:
            print(f'[warn] {key} missing from manifest — skipping')
            continue
        path = layers_dir / layers[key]
        if not path.is_file():
            raise SystemExit(f'layer file not found: {path}')
        mask = load_rgba_opaque(path, hw)
        n = int(mask.sum())
        print(f'[layer] {key:20s}: {n:>10,} cells (opaque)')
        machine_occ |= mask
    n_machine_occ = int(machine_occ.sum())
    print(f'[layer] machine_occ union: {n_machine_occ:>10,} cells')

    # ---- Machine-free = free_evidence non-transparent cells ----
    machine_free = np.zeros(hw, dtype=bool)
    if args.include_free_evidence and 'free_evidence' in layers:
        path = layers_dir / layers['free_evidence']
        if not path.is_file():
            raise SystemExit(f'layer file not found: {path}')
        machine_free = load_rgba_nonempty(path, hw)
        print(f'[layer] free_evidence     : {int(machine_free.sum()):>10,} cells (α>0)')

    # ---- Sidecars ----
    keepout = np.zeros(hw, dtype=bool)
    free_mask = np.zeros(hw, dtype=bool)
    if args.keepout_mask is not None:
        if not args.keepout_mask.is_file():
            raise SystemExit(f'--keepout-mask not found: {args.keepout_mask}')
        keepout = load_sidecar_mask(args.keepout_mask, hw, args.mask_threshold)
        print(f'[side ] keepout_mask      : {int(keepout.sum()):>10,} cells (L≥{args.mask_threshold})')
    if args.free_mask is not None:
        if not args.free_mask.is_file():
            raise SystemExit(f'--free-mask not found: {args.free_mask}')
        free_mask = load_sidecar_mask(args.free_mask, hw, args.mask_threshold)
        print(f'[side ] free_mask         : {int(free_mask.sum()):>10,} cells (L≥{args.mask_threshold})')

    # ---- Compose ----
    # Rule 3 protection built into the algebra: machine_occ is always
    # OR-ed in unconditionally (line 1), so no sidecar can flip a
    # machine-occupied cell to FREE.
    occupied_final = machine_occ | (keepout & ~free_mask)
    free_final = (machine_free | free_mask) & ~occupied_final

    # ---- Diagnostics ----
    n_occ = int(occupied_final.sum())
    n_free = int(free_final.sum())
    n_unknown = W * H - n_occ - n_free
    n_keepout_effective = int((keepout & ~free_mask & ~machine_occ).sum())
    n_keepout_vetoed_by_free_mask = int((keepout & free_mask).sum())
    n_free_mask_over_machine_occ = int((free_mask & machine_occ).sum())
    print()
    print(f'== composition ==')
    print(f'  machine_occupied       : {n_machine_occ:>12,}')
    print(f'  keepout effective (+)  : {n_keepout_effective:>12,}  (added to OCCUPIED)')
    print(f'  keepout vetoed by free : {n_keepout_vetoed_by_free_mask:>12,}  (free_mask won)')
    print(f'  free_mask over occ (X) : {n_free_mask_over_machine_occ:>12,}  '
          f'(REJECTED — Rule 3 protection)')
    print(f'  ---')
    print(f'  OCCUPIED (final)       : {n_occ:>12,}  ({100*n_occ/(W*H):6.2f}%)')
    print(f'  FREE     (final)       : {n_free:>12,}  ({100*n_free/(W*H):6.2f}%)')
    print(f'  UNKNOWN  (final)       : {n_unknown:>12,}  ({100*n_unknown/(W*H):6.2f}%)')

    if args.dry_run:
        print(f'\n(dry-run: no files written)')
        print(f'== total wall time: {time.time() - t0:.1f} s ==')
        return 0

    # ---- Write pgm ----
    pgm = np.full(hw, UNKNOWN, dtype=np.uint8)
    pgm[free_final] = FREE
    pgm[occupied_final] = OCCUPIED
    args.output_pgm.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pgm, mode='L').save(args.output_pgm)
    print(f'[out] {args.output_pgm}')

    # ---- Write yaml ----
    out_yaml_path = args.output_yaml or args.output_pgm.with_suffix('.yaml')
    yaml_out = {
        'image': args.output_pgm.name,
        'resolution': resolution,
        'origin': origin,
        'negate': args.negate,
        'occupied_thresh': args.occupied_thresh,
        'free_thresh': args.free_thresh,
    }
    with out_yaml_path.open('w') as f:
        yaml.safe_dump(yaml_out, f, sort_keys=False)
    print(f'[out] {out_yaml_path}')

    print(f'\n== total wall time: {time.time() - t0:.1f} s ==')
    return 0


if __name__ == '__main__':
    sys.exit(main())
