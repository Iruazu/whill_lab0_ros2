#!/usr/bin/env python3
"""pcd_to_traversability.py — 2D traversability occupancy grid from static.pcd.

Task #22 (2026-07-17 initial; 2026-07-18 h-filter + manual free-mask mode).
Motivation: cleaned OccupancyGrid's "free" means "LiDAR ray passed through",
not "chair can drive over" (2026-07-16 field, ADR-0009 §Consequences).
This script rebuilds a occupancy grid where free requires actual local
geometry to be flat and step-free.

2026-07-18 revision: free-space auto-detection has proven too fragile on the
campus point cloud (97 %-unknown gap even with inheritance), so we pivot to
"trav gives us OCCUPIED, a human paints FREE on top". The script still auto-
computes traversability occupancy, but the FREE side is now sourced from a
hand-painted mask via --free-mask. Occupied always beats hand-paint, so a
human accidentally painting over a curb does not open the curb.

Algorithm (per --input-yaml grid, all thresholds CLI-overridable):

  1. XY-bin all points into cells of `resolution` m.
  2. Cell ground_z = per-cell lower percentile of z (default 5%).
     Robust to building tops / tree canopy returns.
  3. h-filter (default 2.0 m): drop any point sitting more than
     --max-height-above-ground metres above the LOCAL ground. Local ground
     is per-cell ground_z eroded with a small disk so canopy-only cells
     inherit their neighbours' true ground level rather than trusting their
     own canopy z. A wheelchair + rider is ~1.3-1.5 m tall; 2.0 m gives
     safe headroom without keeping tree branches that would otherwise
     phantom-occupy the map. Applied BEFORE any classification.
  4. For each cell, keep points within [ground_z, ground_z + roughness_band].
     Fit a local plane z = ax + by + c by least squares (batched 3x3 normal
     equations, closed form). Roughness = RMS of residuals.
  5. Step check: 8-neighbor abs(ground_z diff). If any neighbor differs by
     more than step_threshold, both cells count as a step edge → occupied.
  6. Trav classification (per cell):
       count < min_points        → unknown
       (step OR roughness > thr)  → OCCUPIED
       otherwise                  → (free-candidate, may become FREE below)
  7. Final pgm assembly. Two modes:
     (a) --free-mask <PNG> (manual mode, preferred as of 2026-07-18):
         pixel value = OCCUPIED where trav says occupied
                     = FREE where mask pixel ≥ --mask-threshold AND not
                       trav-occupied
                     = UNKNOWN otherwise
         Rule: occupied > free > unknown. Trav-occupied always wins.
     (b) no --free-mask (legacy auto mode): free-candidate cells become
         FREE. If --diff-vs is given, unknown cells within --inherit-radius
         of a trav-known cell that are FREE in the reference pgm also
         become FREE.
  8. Intermediate arrays (count, ground_z, roughness) cached to --cache-npz
     for fast rerun on threshold sweeps. Cache is invalidated when
     --max-height-above-ground changes.

Auxiliary outputs (always written when --output-pgm is written):
  * <output-pgm-dir>/trav_occupied_only.png — RGBA, same pixel dims as the
    input map; trav-occupied cells opaque red (255,0,0,255), everything
    else fully transparent. This is the layer to load over cleaned.pgm in
    GIMP so a human can paint FREE without overpainting occupied.
  * <output-pgm-dir>/trav_occupied_over_cleaned.png — RGB, cleaned.pgm as
    grayscale base with red overlay where trav is occupied. Semi-
    transparent (RED_ALPHA) so map features remain visible. Only written
    when --diff-vs is given (needs the base grayscale image).

Output pgm/yaml use --input-yaml's origin/resolution/negate/thresholds so the
result overlays the existing map exactly.

Usage — pass 1 (produce the red layer to paint on):
    scripts/pcd_to_traversability.py \\
        --input-pcd  docs/maps/campus/static.pcd \\
        --input-yaml docs/maps/campus/occupancy_cleaned.yaml \\
        --output-pgm docs/maps/campus/occupancy_trav.pgm \\
        --cache-npz  docs/maps/campus/.trav_cache.npz \\
        --diff-vs    docs/maps/campus/occupancy_cleaned.pgm

Then in GIMP: open occupancy_cleaned.pgm, place trav_occupied_only.png as a
top layer, paint FREE (white) on a new middle layer, export that middle
layer as e.g. campus_manual_free.png at the SAME pixel dimensions.

Usage — pass 2 (composite the human-painted map for Nav2):
    scripts/pcd_to_traversability.py \\
        --input-pcd  docs/maps/campus/static.pcd \\
        --input-yaml docs/maps/campus/occupancy_cleaned.yaml \\
        --output-pgm docs/maps/campus/occupancy_trav_manual.pgm \\
        --cache-npz  docs/maps/campus/.trav_cache.npz \\
        --free-mask  docs/maps/campus/campus_manual_free.png
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

# Radius (m) of the disk used to erode per-cell ground_z before the h-filter.
# A canopy-only cell inherits the min ground_z of any cell within this radius,
# so tree crowns whose LiDAR shadow left ~1-3 stray cells with only canopy
# points get rescued to the true ground level of adjacent open ground. Small
# enough not to blur real terrain (buildings, walls) whose ground_z varies
# meaningfully over longer distances.
GROUND_SMOOTH_RADIUS_M = 0.25

# Alpha for the red overlay in trav_occupied_over_cleaned.png. Semi-
# transparent so the underlying cleaned.pgm features (curbs, edges, salt)
# remain visible for human verification of coverage.
RED_ALPHA = 0.6


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
    p.add_argument('--max-height-above-ground', type=float, default=2.0,
                   help='Drop points more than this many metres above the LOCAL ground before '
                        'any classification (metres). Default 2.0 — safe headroom for a '
                        'wheelchair + rider (~1.5 m) while removing tree branches at 3 m+ that '
                        'would otherwise phantom-occupy the map. Set to 0 to disable.')

    p.add_argument('--free-mask', type=pathlib.Path,
                   help='Hand-painted PNG (same pixel dimensions as --input-yaml grid) where '
                        'white ≥ --mask-threshold marks cells that should be FREE in the '
                        'output. Composite rule: OCCUPIED(trav) > FREE(mask) > UNKNOWN — trav '
                        'occupied always wins even if a human accidentally painted over it. '
                        'When given, --diff-vs inheritance is skipped for the main pgm output.')
    p.add_argument('--mask-threshold', type=int, default=128,
                   help='Grayscale threshold on --free-mask above which a pixel counts as '
                        'painted (0-255). Default 128 — tolerant to GIMP brush anti-aliasing.')

    p.add_argument('--cache-npz', type=pathlib.Path,
                   help='Save / reuse per-cell intermediates (count, ground_z, roughness). '
                        'When present + valid, PCD is not re-loaded — threshold sweeps take ~15 s.')
    p.add_argument('--force-recompute', action='store_true',
                   help='Ignore an existing --cache-npz and rebuild from PCD.')

    p.add_argument('--diff-vs', type=pathlib.Path,
                   help='Reference pgm (e.g. occupancy_cleaned.pgm). If given AND --free-mask is '
                        'NOT given, unknown cells get free-inheritance (see --inherit-radius). '
                        'Always used, when given, as the grayscale base for '
                        'trav_occupied_over_cleaned.png. With --output-diff an additional RGB '
                        'diff overlay is written (red = new occupied vs ref, green = new free).')
    p.add_argument('--output-diff', type=pathlib.Path)
    p.add_argument('--inherit-radius', type=float, default=0.5,
                   help='Free-inheritance radius (m). Unknown cells within this distance of a '
                        'trav-known cell that are marked free in --diff-vs, and are not otherwise '
                        'occupied on this pass, inherit "free". Set to 0 to disable inheritance. '
                        'Ignored when --free-mask is given. Default 0.5.')

    p.add_argument('--output-occupied-png', type=pathlib.Path,
                   help='RGBA PNG with trav-occupied cells opaque red and everything else '
                        'fully transparent. Same pixel dimensions as --input-yaml grid. '
                        'Default: <output-pgm-dir>/trav_occupied_only.png. Use --no-occupied-png '
                        'to skip.')
    p.add_argument('--output-overlay-png', type=pathlib.Path,
                   help='RGB PNG with --diff-vs as grayscale base and trav-occupied cells '
                        'blended in red (alpha=%.2f). Written only when --diff-vs is provided. '
                        'Default: <output-pgm-dir>/trav_occupied_over_cleaned.png.' % RED_ALPHA)
    p.add_argument('--no-occupied-png', action='store_true',
                   help='Skip trav_occupied_only.png output.')
    p.add_argument('--no-overlay-png', action='store_true',
                   help='Skip trav_occupied_over_cleaned.png output even when --diff-vs is given.')

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
                   ground_percentile, roughness_band,
                   max_height_above_ground=0.0):
    """Compute per-cell (count, ground_z, roughness) as flat arrays of length W*H.

    Cells with count < 3 have roughness = NaN (plane fit not possible).

    When max_height_above_ground > 0, points sitting more than that many
    metres above the LOCAL ground are dropped before count / ground_z /
    roughness are computed. "Local ground" = per-cell ground_z eroded by a
    small disk so canopy-only cells inherit their neighbours' true ground.
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

    q = ground_percentile / 100.0

    if max_height_above_ground > 0.0:
        # First pass: coarse ground_z from ALL in-bounds points so we know
        # roughly where the ground is under every cell that has points.
        initial_ground_per = df.groupby('cell', sort=False)['z'].quantile(q).astype(np.float32)

        # Erode ground_z with a disk kernel so canopy-only cells adopt the
        # min ground_z of nearby cells. Cells that have no valid neighbour
        # within the radius keep +inf, which means the h-filter accepts
        # nothing → any points there fail and the cell becomes unknown.
        initial_ground_flat = np.full(W * H, np.inf, dtype=np.float32)
        initial_ground_flat[initial_ground_per.index.to_numpy()] = initial_ground_per.to_numpy()
        initial_ground_2d = initial_ground_flat.reshape(H, W)
        r_cells = max(1, int(round(GROUND_SMOOTH_RADIUS_M / resolution)))
        k = 2 * r_cells + 1
        smooth_kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
        # cv2.erode on float32 = per-pixel minimum in the kernel window.
        # borderValue=+inf so image edges do not become artificial low ground.
        smoothed_ground_2d = cv2.erode(
            initial_ground_2d, smooth_kernel,
            borderType=cv2.BORDER_CONSTANT, borderValue=float(np.inf),
        )
        smoothed_ground_flat = smoothed_ground_2d.ravel()

        smoothed_per_point = smoothed_ground_flat[df['cell'].to_numpy()]
        z_arr = df['z'].to_numpy()
        keep = z_arr <= (smoothed_per_point + max_height_above_ground)
        n_dropped = int((~keep).sum())
        print(f'    h-filter: dropped {n_dropped:,} of {len(df):,} points '
              f'> {max_height_above_ground} m above local ground '
              f'(smooth radius {GROUND_SMOOTH_RADIUS_M} m = {r_cells} cells)')
        df = df.iloc[keep].reset_index(drop=True)

    # Step A: count + ground_z per cell (from post-filter points).
    grp = df.groupby('cell', sort=False)
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


def build_occupied_only_rgba(occupied_2d):
    """RGBA array (H, W, 4) — opaque red where occupied, fully transparent elsewhere.

    Pixel dimensions match occupied_2d exactly, so overlaying this PNG on
    occupancy_cleaned.pgm in GIMP requires no scaling and cannot drift. The
    alpha channel is a hard 0/255 mask; nothing is anti-aliased.
    """
    H, W = occupied_2d.shape
    rgba = np.zeros((H, W, 4), dtype=np.uint8)
    rgba[occupied_2d, 0] = 255   # R
    rgba[occupied_2d, 3] = 255   # A
    return rgba


def build_occupied_over_grayscale(base_gray, occupied_2d, alpha=RED_ALPHA):
    """RGB overlay of a semi-transparent red on a grayscale base image.

    Where occupied_2d is True, pixel = (1-alpha) * base + alpha * red.
    Where False, pixel = base repeated across channels. Same pixel dims
    as base_gray. alpha=0 means unchanged base, alpha=1 means fully opaque
    red on occupied cells.
    """
    if base_gray.shape != occupied_2d.shape:
        raise ValueError(
            f'base_gray shape {base_gray.shape} != occupied_2d shape {occupied_2d.shape}')
    rgb = np.stack([base_gray, base_gray, base_gray], axis=-1).astype(np.float32)
    red = np.array([255.0, 0.0, 0.0], dtype=np.float32)
    rgb[occupied_2d] = (1.0 - alpha) * rgb[occupied_2d] + alpha * red
    return np.clip(rgb, 0.0, 255.0).astype(np.uint8)


def load_free_mask(mask_path, expected_shape, threshold):
    """Load a hand-painted PNG and return a bool array of the 'painted' region.

    Reads with PIL. If the image is RGB or RGBA, converts to L (luminance)
    first — a user might paint in whatever mode GIMP defaults to, and we
    want any bright pixel to count as 'painted' regardless of channel
    layout. Alpha, if present, gates the mask: a fully transparent pixel
    cannot be considered painted no matter its RGB.
    """
    im = Image.open(mask_path)
    if im.mode == 'RGBA':
        alpha = np.array(im.getchannel('A'))
        gray = np.array(im.convert('L'))
        painted = (gray >= threshold) & (alpha > 0)
    else:
        gray = np.array(im.convert('L'))
        painted = gray >= threshold
    if painted.shape != expected_shape:
        raise ValueError(
            f'--free-mask shape {painted.shape} != map shape {expected_shape}. '
            f'The mask PNG must be exported at the exact pixel dimensions of the '
            f'reference pgm (no scaling, no cropping).')
    return painted


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
          f'rough_band={args.roughness_band} m | max_h={args.max_height_above_ground} m')
    if args.free_mask is not None:
        print(f'mode      :  MANUAL — free from --free-mask (occupied > free > unknown)')
    else:
        print(f'mode      :  AUTO — free from trav-known cells' +
              (f' + inheritance vs {args.diff_vs}' if args.diff_vs is not None else ''))
    print()

    # Cache validity depends on max-height: the intermediates (count,
    # ground_z, roughness) are all computed AFTER the h-filter, so a
    # different threshold demands a rebuild. Everything else is applied
    # post-cache (step/roughness thresholds) and does not invalidate.
    use_cache = (args.cache_npz is not None
                 and args.cache_npz.is_file()
                 and not args.force_recompute)

    if use_cache:
        t1 = time.time()
        print(f'[cache] loading {args.cache_npz} ...')
        cache = np.load(args.cache_npz)
        cached_h = float(cache['max_height_above_ground'][0]) if 'max_height_above_ground' in cache.files else -1.0
        if abs(cached_h - args.max_height_above_ground) > 1e-6:
            print(f'[cache] max_height changed ({cached_h} → {args.max_height_above_ground}), recomputing')
            use_cache = False
        else:
            count = cache['count']
            ground_z = cache['ground_z']
            roughness = cache['roughness']
            print(f'[cache] {time.time() - t1:.1f} s')

    if not use_cache:
        t1 = time.time()
        print(f'[pcd] loading ...')
        pts = load_pcd(args.input_pcd)
        print(f'[pcd] {len(pts):,} points, {time.time() - t1:.1f} s')

        t1 = time.time()
        print(f'[stats] per-cell count / ground_z / roughness ...')
        count, ground_z, roughness = per_cell_stats(
            pts, origin, resolution, W, H,
            args.ground_percentile, args.roughness_band,
            max_height_above_ground=args.max_height_above_ground,
        )
        print(f'[stats] {time.time() - t1:.1f} s')

        if args.cache_npz is not None:
            args.cache_npz.parent.mkdir(parents=True, exist_ok=True)
            np.savez(
                args.cache_npz,
                count=count, ground_z=ground_z, roughness=roughness,
                max_height_above_ground=np.array([args.max_height_above_ground], dtype=np.float64),
            )
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

    # Trav-occupied cells drive the red PNG AND always win in the final pgm,
    # regardless of mode. This is the whole reason we split occupied
    # detection (mechanical) from free assignment (human or heuristic):
    # a human accidentally painting over a curb never opens the curb.
    occupied_2d = known_2d & (rough_mask | step_mask)
    trav_free_2d = known_2d & ~occupied_2d

    # Compose the output pgm. Two paths.
    pgm = np.full((H, W), UNKNOWN, dtype=np.uint8)
    inherited_2d = np.zeros((H, W), dtype=bool)
    manual_free_2d = np.zeros((H, W), dtype=bool)

    if args.free_mask is not None:
        # MANUAL mode. Free comes entirely from the hand-painted PNG.
        # Composite rule: OCCUPIED > FREE(painted) > UNKNOWN.
        if not args.free_mask.is_file():
            raise SystemExit(f'--free-mask not found: {args.free_mask}')
        t1 = time.time()
        painted_2d = load_free_mask(args.free_mask, (H, W), args.mask_threshold)
        # Free where painted AND not trav-occupied.
        manual_free_2d = painted_2d & ~occupied_2d
        pgm[manual_free_2d] = FREE
        pgm[occupied_2d] = OCCUPIED
        n_paint = int(painted_2d.sum())
        n_paint_over_occ = int((painted_2d & occupied_2d).sum())
        print(f'[mask] painted cells: {n_paint:,} '
              f'({n_paint_over_occ:,} landed on trav-occupied and were kept as OCCUPIED) '
              f'in {time.time() - t1:.1f} s')
    else:
        # AUTO mode (legacy). Trav-known non-occupied cells are FREE, and
        # optionally --diff-vs inheritance rescues nearby unknown cells
        # that read FREE in the reference map.
        pgm[trav_free_2d] = FREE
        pgm[occupied_2d] = OCCUPIED
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
                    # Ellipse (=disk) kernel so the "within R metres"
                    # statement is honest at the corners.
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
    n_free_manual = int(manual_free_2d.sum())
    n_step = int((step_mask & known_2d).sum())
    n_rough = int((rough_mask & known_2d).sum())
    n_both = int((step_mask & rough_mask & known_2d).sum())
    # Break out FREE by source so the mode is obvious in the log.
    n_free_in_pgm = int((pgm == FREE).sum())
    n_unknown_in_pgm = int((pgm == UNKNOWN).sum())
    print()
    print(f'== classification ==')
    print(f'  total          : {n_cells:>12,}')
    print(f'  known (trav)   : {n_known:>12,}  ({100*n_known/n_cells:6.2f}%)')
    print(f'  unknown (pgm)  : {n_unknown_in_pgm:>12,}  ({100*n_unknown_in_pgm/n_cells:6.2f}%)')
    print(f'  free (pgm)     : {n_free_in_pgm:>12,}  ({100*n_free_in_pgm/n_cells:6.2f}%)')
    if args.free_mask is not None:
        print(f'    from mask    : {n_free_manual:>12,}')
        print(f'    trav-free    : {n_free_trav:>12,}  (dropped: manual mode ignores auto free)')
    else:
        print(f'    from trav    : {n_free_trav:>12,}')
        print(f'    inherited    : {n_free_inherit:>12,}')
    print(f'  occupied (pgm) : {n_occ:>12,}  ({100*n_occ/n_cells:6.2f}%)')
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

    # trav_occupied_only.png — the layer a human loads into GIMP to see
    # exactly which cells the mechanical algorithm called occupied, so
    # they can paint FREE anywhere the red is NOT.
    if not args.no_occupied_png:
        occ_only_path = args.output_occupied_png or (
            args.output_pgm.parent / 'trav_occupied_only.png')
        rgba = build_occupied_only_rgba(occupied_2d)
        occ_only_path.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgba, mode='RGBA').save(occ_only_path)
        print(f'[out] wrote {occ_only_path}  '
              f'(RGBA, {int(occupied_2d.sum()):,} opaque red cells)')

    # trav_occupied_over_cleaned.png — visual confirmation of coverage
    # with the cleaned map showing through in the background.
    if args.diff_vs is not None and not args.no_overlay_png:
        if not args.diff_vs.is_file():
            print(f'[overlay] --diff-vs not found: {args.diff_vs} (skipping overlay)')
        else:
            base = np.array(Image.open(args.diff_vs))
            if base.shape != occupied_2d.shape:
                print(f'[overlay] shape mismatch: base={base.shape} vs '
                      f'grid={occupied_2d.shape} (skipping overlay)')
            else:
                overlay_path = args.output_overlay_png or (
                    args.output_pgm.parent / 'trav_occupied_over_cleaned.png')
                over = build_occupied_over_grayscale(base, occupied_2d, alpha=RED_ALPHA)
                overlay_path.parent.mkdir(parents=True, exist_ok=True)
                Image.fromarray(over, mode='RGB').save(overlay_path)
                print(f'[out] wrote {overlay_path}  '
                      f'(RGB, red α={RED_ALPHA} over cleaned base)')

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
