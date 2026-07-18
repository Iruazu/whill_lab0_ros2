#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""compose_occupancy.py — Stage 2 of the PCD → OccupancyGrid v2 pipeline.

Composes Stage 1's layer-separated evidence (occupied_step,
occupied_structure, free_evidence) with two hand-painted sidecar
masks (keepout_mask, free_mask) into a Nav2-ready pgm + yaml.

Composition rule (2026-07-18 PR #91 review — priority reordered so
the human can erase machine noise, which is the whole point of v2;
2026-07-18 review round 3 added the roadway_mask whitelist):

    conflict         = keepout & free_mask            [warn + record]
    erased_by_free   = machine_occ & free_mask & ¬keepout [audit]
    occupied_final   = keepout | (machine_occ & ¬free_mask)
    free_final       = (free_mask | machine_free) & ¬occupied_final
                       ∩ roadway_mask                 [fail-closed if given]

Priority (highest → lowest):
    keepout  >  free_mask  >  machine_occ  >  machine_free

--roadway-mask (optional): fail-closed whitelist for FREE. Only cells
    within the operator-painted roadway can become FREE in the final
    pgm. Cells that WOULD have been FREE but sit outside the roadway
    are clipped and recorded in clipped_by_roadway.png (magenta) with a
    count in the diagnostic log. If no --roadway-mask is passed, the
    composer prints a 1-line warning and falls back to un-clipped
    behaviour (previous v2 default). Recommended for campus operation:
    Nav2 planner should never entertain FREE cells outside the
    driveable corridor.

Rationale:
  * keepout wins over free_mask — if the human explicitly painted
    "this IS an obstacle", nothing else may erase it. Conflicts (both
    painted) resolve to keepout AND are logged so the operator sees
    the intent collision.
  * free_mask wins over machine_occ — v2's whole point is that the
    human is the truth verdict. Road-surface salt that the mechanical
    pass caught MUST be erasable, otherwise Stage 2 has no way to
    remove the false positives Stage 1 admits. An earlier draft made
    machine_occ unconditionally win; that broke the design intent and
    was called out in PR #91 review.
  * machine_occ wins over machine_free — if the mechanical pass saw
    a step or a wall AND raycast free crossed the same cell (rare),
    trust the geometry over the ray.
  * machine_free wins over nothing — cells with no evidence stay
    UNKNOWN.

The composer never dilates and never smooths — Stage 1's Rule 1
"output cells are the raw evidence positions" carries through to
final.pgm unchanged.

Audit outputs (always written next to --output-pgm when non-zero):
  conflict.png       cells where keepout AND free_mask both painted
                     (keepout wins; magenta, RGBA opaque). Zero cells
                     is the healthy state; non-zero means the operator
                     needs to reconcile intent.
  erased_by_free.png cells that were machine_occ, went to FREE via
                     free_mask (yellow, RGBA opaque). NOT an error —
                     this is exactly the salt-removal channel v2
                     depends on. Count is the audit of how much
                     human erasure happened.

Usage:

    # machine-only baseline (no human input yet)
    scripts/compose_occupancy.py \\
        --layers-yaml docs/maps/campus/v2/v2_layers.yaml \\
        --output-pgm  docs/maps/campus/v2/final.pgm

    # with human sidecars once GIMP work is done
    scripts/compose_occupancy.py \\
        --layers-yaml docs/maps/campus/v2/v2_layers.yaml \\
        --keepout-mask docs/maps/campus/v2/keepout_mask.png \\
        --free-mask    docs/maps/campus/v2/free_mask.png \\
        --output-pgm   docs/maps/campus/v2/final.pgm
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

# Audit-layer colours (opaque so they read clearly over any GIMP base).
COLOR_CONFLICT = (255, 0, 255, 255)          # magenta — keepout ∩ free_mask
COLOR_ERASED_HUMAN = (255, 220, 0, 255)      # yellow  — erased by human free_mask only
COLOR_ERASED_AUTO = (0, 220, 220, 255)       # cyan    — erased by free_mask_auto only
COLOR_ERASED_BOTH = (255, 255, 255, 255)     # white   — erased by both sources (rare)
COLOR_CLIPPED = (255, 0, 255, 255)           # magenta — free cells clipped by roadway_mask
                                              # (distinct location from conflict so the two
                                              # never overlap visually)


def parse_args():
    p = argparse.ArgumentParser(
        description=__doc__.split('\n\n', 1)[0],
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    p.add_argument('--layers-yaml', required=True, type=pathlib.Path,
                   help='Path to Stage 1 v2_layers.yaml manifest.')
    p.add_argument('--output-pgm', required=True, type=pathlib.Path,
                   help='Final pgm output. Sibling yaml + audit PNGs are written '
                        'next to it.')
    p.add_argument('--output-yaml', type=pathlib.Path,
                   help='Explicit output yaml path (default: --output-pgm '
                        'with .yaml suffix).')

    p.add_argument('--keepout-mask', type=pathlib.Path,
                   help='Bright pixels add OCCUPIED. Wins over free_mask on '
                        'the same cell (conflict logged + conflict.png).')
    p.add_argument('--free-mask', type=pathlib.Path,
                   help='Bright pixels add FREE and CAN erase machine_occ. '
                        'Erased cells recorded in erased_by_free.png (yellow '
                        'for human-source cells).')
    p.add_argument('--free-mask-auto', type=pathlib.Path,
                   help='Auto-generated free mask (typically from '
                        'scripts/gen_auto_free_mask.py — 1-cell isolated '
                        'occupied inside roadway_mask). OR-ed with '
                        '--free-mask; erased cells contributed by this '
                        'source render as CYAN in erased_by_free.png so the '
                        'operator sees which fixes were automatic vs human.')
    p.add_argument('--roadway-mask', type=pathlib.Path,
                   help='Optional fail-closed whitelist for FREE. Cells that '
                        'would have been FREE but sit OUTSIDE this mask are '
                        'clipped and recorded in clipped_by_roadway.png. When '
                        'omitted, the composer prints a warning and falls back '
                        'to un-clipped behaviour. Recommended for campus '
                        'operation — Nav2 should not plan outside the '
                        'operator-defined driveable corridor.')
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

    p.add_argument('--no-audit-png', action='store_true',
                   help='Skip conflict.png and erased_by_free.png even when '
                        'their counts are non-zero. Not recommended.')

    p.add_argument('--dry-run', action='store_true',
                   help='Compute + report counts, do not write pgm/yaml/audit PNGs.')
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
    """Bool mask of pixels with alpha == 255 in an RGBA PNG."""
    im = Image.open(path)
    a = np.array(im)
    if a.shape[:2] != expected_hw:
        raise SystemExit(f'{path}: shape {a.shape[:2]} != expected {expected_hw}. '
                         f'Stage 1 layers and sidecars must share exact pixel dims.')
    if im.mode != 'RGBA':
        raise SystemExit(f'{path}: expected RGBA, got {im.mode}.')
    return a[..., 3] == 255


def load_rgba_nonempty(path, expected_hw):
    """Bool mask of any non-transparent (alpha > 0) pixel."""
    im = Image.open(path)
    a = np.array(im)
    if a.shape[:2] != expected_hw:
        raise SystemExit(f'{path}: shape {a.shape[:2]} != expected {expected_hw}.')
    if im.mode != 'RGBA':
        raise SystemExit(f'{path}: expected RGBA, got {im.mode}.')
    return a[..., 3] > 0


def load_sidecar_mask(path, expected_hw, threshold):
    """Hand-painted PNG → bool of painted cells.

    L / RGB / P mode: painted = L ≥ threshold.
    RGBA mode: painted = L ≥ threshold AND alpha > 0 (transparent pixels
    are never painted regardless of RGB channels).

    L via PIL ITU-R 601-2 luma (0.299R + 0.587G + 0.114B). Recommend
    painting in WHITE — pure red (L=76) falls below the default
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


def save_audit_layer(path, mask, colour):
    """Write an opaque-colour RGBA PNG at `mask == True`, transparent elsewhere."""
    H, W = mask.shape
    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    rgba[mask, 0] = colour[0]
    rgba[mask, 1] = colour[1]
    rgba[mask, 2] = colour[2]
    rgba[mask, 3] = colour[3]
    Image.fromarray(rgba, 'RGBA').save(path)


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

    # machine_occupied = union of Stage 1 step + structure opaque cells.
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

    # machine_free = free_evidence non-transparent cells (raycast or footprint).
    machine_free = np.zeros(hw, dtype=bool)
    if args.include_free_evidence and 'free_evidence' in layers:
        path = layers_dir / layers['free_evidence']
        if not path.is_file():
            raise SystemExit(f'layer file not found: {path}')
        machine_free = load_rgba_nonempty(path, hw)
        print(f'[layer] free_evidence     : {int(machine_free.sum()):>10,} cells (α>0)')

    # Sidecars.
    keepout = np.zeros(hw, dtype=bool)
    free_mask = np.zeros(hw, dtype=bool)
    if args.keepout_mask is not None:
        if not args.keepout_mask.is_file():
            raise SystemExit(f'--keepout-mask not found: {args.keepout_mask}')
        keepout = load_sidecar_mask(args.keepout_mask, hw, args.mask_threshold)
        print(f'[side ] keepout_mask      : {int(keepout.sum()):>10,} cells '
              f'(L≥{args.mask_threshold})')
    if args.free_mask is not None:
        if not args.free_mask.is_file():
            raise SystemExit(f'--free-mask not found: {args.free_mask}')
        free_mask = load_sidecar_mask(args.free_mask, hw, args.mask_threshold)
        print(f'[side ] free_mask         : {int(free_mask.sum()):>10,} cells '
              f'(L≥{args.mask_threshold})')
    free_mask_auto = np.zeros(hw, dtype=bool)
    if args.free_mask_auto is not None:
        if not args.free_mask_auto.is_file():
            raise SystemExit(f'--free-mask-auto not found: {args.free_mask_auto}')
        free_mask_auto = load_sidecar_mask(args.free_mask_auto, hw, args.mask_threshold)
        print(f'[side ] free_mask_auto    : {int(free_mask_auto.sum()):>10,} cells '
              f'(L≥{args.mask_threshold})')
    # Merge for composition (composer treats both sources identically —
    # audit distinguishes them via colour).
    free_mask_effective = free_mask | free_mask_auto
    # roadway_mask: fail-closed whitelist. If absent, all cells whitelisted
    # (equivalent to legacy behaviour) but WITH a warning so the operator
    # knows planning space is not fenced.
    roadway_provided = args.roadway_mask is not None
    if roadway_provided:
        if not args.roadway_mask.is_file():
            raise SystemExit(f'--roadway-mask not found: {args.roadway_mask}')
        roadway_mask = load_sidecar_mask(args.roadway_mask, hw, args.mask_threshold)
        print(f'[side ] roadway_mask      : {int(roadway_mask.sum()):>10,} cells '
              f'(L≥{args.mask_threshold})  ← fail-closed FREE whitelist')
    else:
        roadway_mask = np.ones(hw, dtype=bool)   # accept everything
        print(f'[warn ] no --roadway-mask supplied. FREE cells are NOT clipped '
              f'to a driveable corridor; Nav2 may plan into off-limits areas.')

    # Compose. Priority: keepout > free_mask > machine_occ > machine_free.
    # Roadway whitelist applies AFTER the free/occupied resolution — a
    # cell painted by keepout stays OCCUPIED even outside roadway (safe:
    # obstacles do not shrink based on whitelist).
    conflict = keepout & free_mask_effective
    erased_by_free = machine_occ & free_mask_effective & ~keepout
    # Per-source erasure attribution for the audit PNG.
    erased_by_free_human = erased_by_free & free_mask & ~free_mask_auto
    erased_by_free_auto  = erased_by_free & free_mask_auto & ~free_mask
    erased_by_free_both  = erased_by_free & free_mask & free_mask_auto
    occupied_final = keepout | (machine_occ & ~free_mask_effective)
    free_would_be = (free_mask_effective | machine_free) & ~occupied_final
    free_final = free_would_be & roadway_mask
    clipped_by_roadway = free_would_be & ~roadway_mask

    # Diagnostics.
    n_occ = int(occupied_final.sum())
    n_free = int(free_final.sum())
    n_unknown = W * H - n_occ - n_free
    n_conflict = int(conflict.sum())
    n_erased = int(erased_by_free.sum())
    n_erased_human = int(erased_by_free_human.sum())
    n_erased_auto = int(erased_by_free_auto.sum())
    n_erased_both = int(erased_by_free_both.sum())
    n_keepout_new = int((keepout & ~machine_occ).sum())
    n_keepout_reinforcing = int((keepout & machine_occ).sum())
    n_clipped = int(clipped_by_roadway.sum())

    print()
    print(f'== composition ==')
    print(f'  machine_occupied         : {n_machine_occ:>12,}')
    print(f'  keepout new  (adds occ)  : {n_keepout_new:>12,}')
    print(f'  keepout reinforcing      : {n_keepout_reinforcing:>12,}  '
          f'(already machine_occ)')
    print(f'  free_mask erased occ     : {n_erased:>12,}  '
          f'(audit → erased_by_free.png)')
    if args.free_mask_auto is not None or args.free_mask is not None:
        print(f'    ↳ by human only        : {n_erased_human:>10,}  (yellow)')
        print(f'    ↳ by auto only         : {n_erased_auto:>10,}  (cyan)')
        print(f'    ↳ by both              : {n_erased_both:>10,}  (white)')
    print(f'  conflict keepout∩free    : {n_conflict:>12,}  '
          f'(keepout wins → conflict.png)')
    if roadway_provided:
        print(f'  clipped_by_roadway       : {n_clipped:>12,}  '
              f'(would-be-FREE outside whitelist → clipped_by_roadway.png)')
    if n_conflict > 0:
        print(f'  [warn] {n_conflict:,} cells have BOTH keepout and free_mask painted. '
              f'keepout wins per priority rule; inspect conflict.png to reconcile intent.')
    print(f'  ---')
    print(f'  OCCUPIED (final)         : {n_occ:>12,}  ({100*n_occ/(W*H):6.2f}%)')
    print(f'  FREE     (final)         : {n_free:>12,}  ({100*n_free/(W*H):6.2f}%)')
    print(f'  UNKNOWN  (final)         : {n_unknown:>12,}  ({100*n_unknown/(W*H):6.2f}%)')

    if args.dry_run:
        print(f'\n(dry-run: no files written)')
        print(f'== total wall time: {time.time() - t0:.1f} s ==')
        return 0

    # Write pgm.
    pgm = np.full(hw, UNKNOWN, dtype=np.uint8)
    pgm[free_final] = FREE
    pgm[occupied_final] = OCCUPIED
    args.output_pgm.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pgm, mode='L').save(args.output_pgm)
    print(f'[out] {args.output_pgm}')

    # Write yaml.
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

    # Audit PNGs. Written when non-zero (unless --no-audit-png).
    if not args.no_audit_png:
        if n_conflict > 0:
            conflict_path = args.output_pgm.parent / 'conflict.png'
            save_audit_layer(conflict_path, conflict, COLOR_CONFLICT)
            print(f'[out] {conflict_path}  ({n_conflict:,} magenta cells)')
        if n_erased > 0:
            erased_path = args.output_pgm.parent / 'erased_by_free.png'
            # Compose 3-colour RGBA: yellow = human, cyan = auto, white = both.
            H_, W_ = erased_by_free.shape
            rgba = np.zeros((H_, W_, 4), dtype=np.uint8)
            for mask, colour in (
                (erased_by_free_human, COLOR_ERASED_HUMAN),
                (erased_by_free_auto,  COLOR_ERASED_AUTO),
                (erased_by_free_both,  COLOR_ERASED_BOTH),
            ):
                if mask.any():
                    rgba[mask, 0] = colour[0]
                    rgba[mask, 1] = colour[1]
                    rgba[mask, 2] = colour[2]
                    rgba[mask, 3] = colour[3]
            Image.fromarray(rgba, 'RGBA').save(erased_path)
            print(f'[out] {erased_path}  '
                  f'({n_erased:,} cells: {n_erased_human:,} yellow / '
                  f'{n_erased_auto:,} cyan / {n_erased_both:,} white)')
        if n_clipped > 0:
            clipped_path = args.output_pgm.parent / 'clipped_by_roadway.png'
            save_audit_layer(clipped_path, clipped_by_roadway, COLOR_CLIPPED)
            print(f'[out] {clipped_path}  ({n_clipped:,} magenta cells)')

    print(f'\n== total wall time: {time.time() - t0:.1f} s ==')
    return 0


if __name__ == '__main__':
    sys.exit(main())
