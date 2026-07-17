#!/usr/bin/env python3
"""pcd_to_traversability.py — 2D traversability occupancy grid from static.pcd.

Task #22 prototype (2026-07-17). Motivation: cleaned OccupancyGrid's "free"
means "LiDAR ray passed through", not "chair can drive over" (2026-07-16 field,
ADR-0009 §Consequences). This script rebuilds a occupancy grid where free
requires actual local geometry to be flat and step-free.

Algorithm (per --input-yaml grid, all thresholds CLI-overridable):

  1. XY-bin all points into cells of `resolution` m.
  2. Cell ground_z = per-cell lower percentile of z (default 5%).
     Robust to building tops / tree canopy returns.
  3. For each cell, keep points within [ground_z, ground_z + roughness_band].
     Fit a local plane z = ax + by + c by least squares (batched 3x3 normal
     equations, closed form). Roughness = RMS of residuals.
  4. Step check: 8-neighbor abs(ground_z diff). If any neighbor differs by
     more than step_threshold, both cells count as a step edge → occupied.
  5. Classification (per cell):
       count < min_points        → unknown (205)
       (step OR roughness > thr)  → occupied (0)
       otherwise                  → free (254)
  6. Free inheritance (if --diff-vs given): unknown cells that are (a) not
     occupied on this pass, (b) free in the reference pgm (typically
     occupancy_cleaned.pgm), and (c) within --inherit-radius m of a
     trav-known cell get promoted to free. Rescues the 97 %-unknown gap
     caused by static.pcd's ~3 pts/cell density without blindly
     re-importing salt from far-away regions the reference map has no
     support for.
  7. Intermediate arrays (count, ground_z, roughness) cached to --cache-npz
     for fast rerun on threshold sweeps.

Output pgm/yaml use --input-yaml's origin/resolution/negate/thresholds so the
result overlays the existing map exactly.

Usage:
    scripts/pcd_to_traversability.py \\
        --input-pcd  docs/maps/campus/static.pcd \\
        --input-yaml docs/maps/campus/occupancy.yaml \\
        --output-pgm docs/maps/campus/occupancy_trav.pgm \\
        --cache-npz  docs/maps/campus/.trav_cache.npz \\
        --diff-vs    docs/maps/campus/occupancy_cleaned.pgm \\
        --output-diff docs/maps/campus/trav_vs_cleaned_diff.png
"""

import argparse
import pathlib
import sys
import time

import cv2
import numpy as np
import open3d as o3d
import pandas as pd
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
    p.add_argument('--input-pcd', required=True, type=pathlib.Path)
    p.add_argument('--input-yaml', required=True, type=pathlib.Path,
                   help='Existing occupancy yaml for origin / resolution / size / thresholds.')
    p.add_argument('--output-pgm', required=True, type=pathlib.Path)
    p.add_argument('--output-yaml', type=pathlib.Path,
                   help='Sibling yaml path (default: output-pgm with .yaml suffix).')

    p.add_argument('--step-threshold', type=float, default=0.03,
                   help='8-neighbor ground_z diff (m) that flags a step. Default 0.03 (= 3 cm).')
    p.add_argument('--roughness-threshold', type=float, default=0.015,
                   help='Plane-residual RMS (m, i.e. std) that flags rough terrain. Default 0.015 '
                        '(= 15 mm std ≈ 2.25e-4 m² variance). Reference: asphalt ~2 mm, '
                        'concrete seams ~5 mm, short grass ~10 mm, tall grass ~25 mm.')
    p.add_argument('--min-points', type=int, default=3,
                   help='Cells with fewer points are classified as unknown (205). Default 3 (plane-fit minimum).')
    p.add_argument('--ground-percentile', type=float, default=5.0,
                   help='Per-cell z lower percentile used as ground_z (%%). Default 5.')
    p.add_argument('--roughness-band', type=float, default=0.3,
                   help='Only points in [ground_z, ground_z + this] contribute to roughness (m). '
                        'Filters building tops / tree canopy from the plane fit. Default 0.3.')

    p.add_argument('--cache-npz', type=pathlib.Path,
                   help='Save / reuse per-cell intermediates (count, ground_z, roughness). '
                        'When present + valid, PCD is not re-loaded — threshold sweeps take ~15 s.')
    p.add_argument('--force-recompute', action='store_true',
                   help='Ignore an existing --cache-npz and rebuild from PCD.')

    p.add_argument('--diff-vs', type=pathlib.Path,
                   help='Reference pgm (e.g. occupancy_cleaned.pgm). If given, unknown cells '
                        'get free-inheritance (see --inherit-radius), and with --output-diff an '
                        'RGB overlay is written (red = new occupied vs ref, green = new free).')
    p.add_argument('--output-diff', type=pathlib.Path)
    p.add_argument('--inherit-radius', type=float, default=0.5,
                   help='Free-inheritance radius (m). Unknown cells within this distance of a '
                        'trav-known cell that are marked free in --diff-vs, and are not otherwise '
                        'occupied on this pass, inherit "free". Set to 0 to disable inheritance. '
                        'Default 0.5.')

    p.add_argument('--dry-run', action='store_true',
                   help='Report the classification counts without writing any output files.')
    return p.parse_args()


def load_pcd(path):
    pcd = o3d.io.read_point_cloud(str(path))
    pts = np.asarray(pcd.points, dtype=np.float32)
    return pts


def load_map_ref(yaml_path):
    with yaml_path.open() as f:
        doc = yaml.safe_load(f)
    ref_pgm = np.array(Image.open(yaml_path.parent / doc['image']))
    H, W = ref_pgm.shape
    return doc, H, W


def per_cell_stats(pts, origin, resolution, W, H,
                   ground_percentile, roughness_band):
    """Compute per-cell (count, ground_z, roughness) as flat arrays of length W*H.

    Cells with count < 3 have roughness = NaN (plane fit not possible).
    """
    origin_x, origin_y = float(origin[0]), float(origin[1])
    ix = ((pts[:, 0] - origin_x) / resolution).astype(np.int32)
    iy = ((pts[:, 1] - origin_y) / resolution).astype(np.int32)
    in_bounds = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    ix = ix[in_bounds]
    iy = iy[in_bounds]
    xyz = pts[in_bounds]
    print(f'    in-bounds points: {len(xyz):,} of {len(pts):,}')

    # Flat cell index. int64 avoids overflow when W*H > 2^31.
    cell = iy.astype(np.int64) * W + ix.astype(np.int64)

    df = pd.DataFrame({
        'cell': cell,
        'x':    xyz[:, 0],
        'y':    xyz[:, 1],
        'z':    xyz[:, 2],
    })

    # Step A: count + ground_z per cell.
    grp = df.groupby('cell', sort=False)
    q = ground_percentile / 100.0
    count_per = grp.size().rename('count')
    ground_per = grp['z'].quantile(q).rename('ground_z').astype(np.float32)

    # Step B: broadcast ground_z to per-point, keep only near-ground points.
    df = df.join(ground_per, on='cell')
    near_mask = df['z'] <= (df['ground_z'] + roughness_band)
    near = df[near_mask]

    # Step C: per-cell moments for the batched 3x3 plane fit.
    # Fit z = a*x + b*y + c. RSS from moments: szz - (a*sxz + b*syz + c*sz).
    near = near.assign(
        xx=near['x'] * near['x'],
        xy=near['x'] * near['y'],
        yy=near['y'] * near['y'],
        xz=near['x'] * near['z'],
        yz=near['y'] * near['z'],
        zz=near['z'] * near['z'],
    )
    mom = near.groupby('cell', sort=False).agg(
        n=('x', 'size'),
        sx=('x', 'sum'), sy=('y', 'sum'), sz=('z', 'sum'),
        sxx=('xx', 'sum'), sxy=('xy', 'sum'), syy=('yy', 'sum'),
        sxz=('xz', 'sum'), syz=('yz', 'sum'), szz=('zz', 'sum'),
    )
    m3 = mom[mom['n'] >= 3]

    # Batched 3x3 solve. Regularise with a tiny diagonal so single-line
    # (colinear x,y) cells do not blow up — the regularisation shifts the
    # answer by less than 1 nm and never dominates real signal.
    n = len(m3)
    A = np.empty((n, 3, 3), dtype=np.float64)
    A[:, 0, 0] = m3['sxx']; A[:, 0, 1] = m3['sxy']; A[:, 0, 2] = m3['sx']
    A[:, 1, 0] = m3['sxy']; A[:, 1, 1] = m3['syy']; A[:, 1, 2] = m3['sy']
    A[:, 2, 0] = m3['sx'];  A[:, 2, 1] = m3['sy'];  A[:, 2, 2] = m3['n']
    A[:, 0, 0] += 1e-10
    A[:, 1, 1] += 1e-10
    A[:, 2, 2] += 1e-10
    b = np.empty((n, 3), dtype=np.float64)
    b[:, 0] = m3['sxz']; b[:, 1] = m3['syz']; b[:, 2] = m3['sz']
    coefs = np.linalg.solve(A, b)   # (n, 3)

    a_c = coefs[:, 0]; b_c = coefs[:, 1]; c_c = coefs[:, 2]
    sxz = m3['sxz'].to_numpy(); syz = m3['syz'].to_numpy()
    sz = m3['sz'].to_numpy(); szz = m3['szz'].to_numpy()
    n_arr = m3['n'].to_numpy().astype(np.float64)
    rss = szz - (a_c * sxz + b_c * syz + c_c * sz)
    rss = np.clip(rss, 0.0, None)   # numerical noise floor
    residual_rms = np.sqrt(rss / n_arr).astype(np.float32)

    # Step D: assemble flat output arrays over the full grid.
    n_cells = W * H
    count_flat = np.zeros(n_cells, dtype=np.int32)
    ground_flat = np.full(n_cells, np.nan, dtype=np.float32)
    rough_flat = np.full(n_cells, np.nan, dtype=np.float32)

    count_flat[count_per.index.to_numpy()] = count_per.to_numpy()
    ground_flat[ground_per.index.to_numpy()] = ground_per.to_numpy()
    rough_flat[m3.index.to_numpy()] = residual_rms

    return count_flat, ground_flat, rough_flat


def compute_step_mask(ground_z_2d, valid_2d, step_threshold):
    """8-neighbor abs-diff on ground_z. Both cells must be valid; edges skipped.

    Returns bool mask (H, W).
    """
    H, W = ground_z_2d.shape
    # Mask out the outermost 1-cell ring so np.roll's wraparound never
    # produces a valid comparison.
    interior = np.zeros((H, W), dtype=bool)
    interior[1:-1, 1:-1] = True

    step = np.zeros((H, W), dtype=bool)
    for dy in (-1, 0, 1):
        for dx in (-1, 0, 1):
            if dy == 0 and dx == 0:
                continue
            shifted_z = np.roll(ground_z_2d, (dy, dx), axis=(0, 1))
            shifted_v = np.roll(valid_2d, (dy, dx), axis=(0, 1))
            diff = np.abs(ground_z_2d - shifted_z)
            both = valid_2d & shifted_v & interior
            step |= both & (diff > step_threshold)
    return step


def build_diff_overlay(pgm, cleaned):
    """RGB overlay. Grayscale base is pgm. Red = new occupied, green = new free."""
    rgb = np.stack([pgm, pgm, pgm], axis=-1).astype(np.uint8)
    new_occ = (pgm == OCCUPIED) & (cleaned != OCCUPIED)
    new_free = (pgm == FREE) & (cleaned != FREE)
    rgb[new_occ] = np.array([255, 0, 0], dtype=np.uint8)
    rgb[new_free] = np.array([0, 255, 0], dtype=np.uint8)
    return rgb


def main():
    args = parse_args()
    if not args.input_pcd.is_file():
        raise SystemExit(f'--input-pcd not found: {args.input_pcd}')
    if not args.input_yaml.is_file():
        raise SystemExit(f'--input-yaml not found: {args.input_yaml}')

    t0 = time.time()
    print(f'== pcd_to_traversability ==')
    print(f'input-pcd :  {args.input_pcd}')
    print(f'input-yaml:  {args.input_yaml}')

    ref_doc, H, W = load_map_ref(args.input_yaml)
    origin = ref_doc['origin']
    resolution = ref_doc['resolution']
    print(f'grid     :   {W} x {H}, origin={origin}, resolution={resolution}')
    print(f'thresholds:  step={args.step_threshold} m | rough_std={args.roughness_threshold} m | '
          f'min_pts={args.min_points} | ground_p={args.ground_percentile}% | '
          f'rough_band={args.roughness_band} m')
    print()

    use_cache = (args.cache_npz is not None
                 and args.cache_npz.is_file()
                 and not args.force_recompute)

    if use_cache:
        t1 = time.time()
        print(f'[cache] loading {args.cache_npz} ...')
        cache = np.load(args.cache_npz)
        count = cache['count']
        ground_z = cache['ground_z']
        roughness = cache['roughness']
        print(f'[cache] {time.time() - t1:.1f} s')
    else:
        t1 = time.time()
        print(f'[pcd] loading ...')
        pts = load_pcd(args.input_pcd)
        print(f'[pcd] {len(pts):,} points, {time.time() - t1:.1f} s')

        t1 = time.time()
        print(f'[stats] per-cell count / ground_z / roughness ...')
        count, ground_z, roughness = per_cell_stats(
            pts, origin, resolution, W, H,
            args.ground_percentile, args.roughness_band,
        )
        print(f'[stats] {time.time() - t1:.1f} s')

        if args.cache_npz is not None:
            args.cache_npz.parent.mkdir(parents=True, exist_ok=True)
            np.savez(args.cache_npz, count=count, ground_z=ground_z, roughness=roughness)
            print(f'[cache] wrote {args.cache_npz}')

    count2d = count.reshape(H, W)
    ground_z2d = ground_z.reshape(H, W)
    roughness2d = roughness.reshape(H, W)

    known_2d = count2d >= args.min_points
    n_known = int(known_2d.sum())

    # Roughness classification (NaN → not rough).
    rough_mask = np.nan_to_num(roughness2d, nan=0.0) > args.roughness_threshold

    # Step classification.
    t1 = time.time()
    print(f'[step] 8-neighbor diff ...')
    step_mask = compute_step_mask(
        np.nan_to_num(ground_z2d, nan=0.0),
        known_2d,
        args.step_threshold,
    )
    print(f'[step] {time.time() - t1:.1f} s')

    # Trav-only classification. Step + rough imply occupied; the rest of
    # `known_2d` is free. Unknown cells stay unknown until inheritance runs.
    occupied_2d = known_2d & (rough_mask | step_mask)
    trav_free_2d = known_2d & ~occupied_2d

    pgm = np.full((H, W), UNKNOWN, dtype=np.uint8)
    pgm[trav_free_2d] = FREE
    pgm[occupied_2d] = OCCUPIED

    # Free-inheritance from --diff-vs (option C改). Rescues unknown cells
    # that are within --inherit-radius of a known cell AND read free in the
    # reference map AND are not occupied on this pass. All three
    # conjuncts must hold — inheritance never overrides a fresh occupied
    # cell, and it never manufactures free where the reference has none.
    inherited_2d = np.zeros((H, W), dtype=bool)
    ref_free_2d = None
    if args.diff_vs is not None and args.inherit_radius > 0.0:
        if not args.diff_vs.is_file():
            print(f'[inherit] --diff-vs not found: {args.diff_vs} (skipping)')
        else:
            ref = np.array(Image.open(args.diff_vs))
            if ref.shape != pgm.shape:
                print(f'[inherit] shape mismatch: pgm={pgm.shape} vs ref={ref.shape} (skipping)')
            else:
                t1 = time.time()
                radius_cells = int(round(args.inherit_radius / resolution))
                k = 2 * radius_cells + 1
                print(f'[inherit] radius={args.inherit_radius} m = {radius_cells} cells, '
                      f'dilation kernel {k}x{k} ...')
                # cv2.dilate wants uint8; the kernel is a rectangle here.
                # Ellipse would round the reach, but with r=10 the corners
                # only reach r*sqrt(2)=14 cells (0.71 m) — over the nominal
                # 0.5 m budget by 40 %. Use ellipse (=disk) so the "within
                # 0.5 m" statement is honest.
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
                dilated_known = cv2.dilate(known_2d.astype(np.uint8), kernel).astype(bool)
                ref_free_2d = (ref == FREE)
                inherited_2d = (~occupied_2d) & ref_free_2d & dilated_known & ~known_2d
                pgm[inherited_2d] = FREE
                print(f'[inherit] {int(inherited_2d.sum()):,} cells inherited '
                      f'in {time.time() - t1:.1f} s')

    n_cells = W * H
    n_occ = int(occupied_2d.sum())
    n_free_trav = int(trav_free_2d.sum())
    n_free_inherit = int(inherited_2d.sum())
    n_free_total = n_free_trav + n_free_inherit
    n_unknown = n_cells - n_known - n_free_inherit
    n_step = int((step_mask & known_2d).sum())
    n_rough = int((rough_mask & known_2d).sum())
    n_both = int((step_mask & rough_mask & known_2d).sum())
    print()
    print(f'== classification ==')
    print(f'  total          : {n_cells:>12,}')
    print(f'  known (trav)   : {n_known:>12,}  ({100*n_known/n_cells:6.2f}%)')
    print(f'  unknown        : {n_unknown:>12,}  ({100*n_unknown/n_cells:6.2f}%)')
    print(f'  free (total)   : {n_free_total:>12,}  ({100*n_free_total/n_cells:6.2f}%)')
    print(f'    from trav    : {n_free_trav:>12,}')
    print(f'    inherited    : {n_free_inherit:>12,}')
    print(f'  occupied       : {n_occ:>12,}  ({100*n_occ/n_cells:6.2f}%)')
    print(f'    step-only    : {n_step - n_both:>12,}')
    print(f'    rough-only   : {n_rough - n_both:>12,}')
    print(f'    both         : {n_both:>12,}')

    if args.dry_run:
        print(f'\n(dry-run: no files written)')
        print(f'== total wall time: {time.time() - t0:.1f} s ==')
        return 0

    args.output_pgm.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(pgm, mode='L').save(args.output_pgm)
    print(f'[out] wrote {args.output_pgm}')

    out_yaml_path = args.output_yaml or args.output_pgm.with_suffix('.yaml')
    yaml_out = {
        'image': args.output_pgm.name,
        'resolution': resolution,
        'origin': origin,
        'negate': ref_doc.get('negate', 0),
        'occupied_thresh': ref_doc.get('occupied_thresh', 0.65),
        'free_thresh': ref_doc.get('free_thresh', 0.196),
    }
    with out_yaml_path.open('w') as f:
        yaml.safe_dump(yaml_out, f, sort_keys=False)
    print(f'[out] wrote {out_yaml_path}')

    if args.diff_vs is not None and args.output_diff is not None:
        if not args.diff_vs.is_file():
            print(f'[diff] --diff-vs not found: {args.diff_vs} (skipping)')
        else:
            cleaned = np.array(Image.open(args.diff_vs))
            if cleaned.shape != pgm.shape:
                print(f'[diff] shape mismatch: pgm={pgm.shape} vs cleaned={cleaned.shape} (skipping)')
            else:
                diff = build_diff_overlay(pgm, cleaned)
                args.output_diff.parent.mkdir(parents=True, exist_ok=True)
                Image.fromarray(diff, mode='RGB').save(args.output_diff)
                print(f'[out] wrote {args.output_diff}')

    print(f'\n== total wall time: {time.time() - t0:.1f} s ==')
    return 0


if __name__ == '__main__':
    sys.exit(main())
