#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""audit_free_leaks.py — Two independent leak metrics for a v2 final pgm.

Runs against a Nav2 pgm (values 0/205/254) and reports:

  (a) 建物リーク metric
        FREE cells whose 15x15 pixel neighbourhood is > 70 % UNKNOWN.
        Interpretation: this FREE cell sits inside an information void
        (typical of a raycast that penetrated a building interior),
        surrounded by cells the LiDAR never observed. A healthy corridor
        FREE cell has almost all its neighbours also FREE or OCCUPIED
        (edge context), NOT UNKNOWN.
        Metric: count of such cells. Ideal = 0.
        Also lists the top-N cell coordinates (map frame) for manual
        inspection when > 0.

  (b) 段差越境 free metric — CAVEAT: NOT IMPLEMENTED PER ORIGINAL SPEC
        Original spec (2026-07-18 PR #91 review round 3): count FREE
        cells on the 敷地側 (property side, opposite of Rule-3 chair-
        accessible) of a step/structure occupied line, with maximum
        intrusion depth past the barrier. Would require identifying,
        for each occupied cell, its 敷地-side direction (from Rule 3
        accessibility) then flood-filling FREE cells outward on that
        side.

        This script implements a WEAKER PROXY: FREE cells that are not
        reachable from any trajectory anchor via the traversable graph
        (traversable = ¬OCCUPIED). A leak that FINGERS past a wall
        but stays CONNECTED to the traj-reachable region (via a gap
        elsewhere) is missed. On the campus map this proxy has always
        returned 0 both before and after P0 fix, indicating it is
        insensitive to the phenomenon of interest.

        Impact is contained by --roadway-mask (fail-closed FREE
        whitelist) since a proper roadway_mask clips ALL out-of-
        corridor FREE cells regardless of their reachability, which
        is the operational safety we care about. Proper metric_b
        implementation is tracked as a backlog issue (see PR #91
        review round 4). Do NOT rely on the current metric_b=0 as
        evidence of "no cross-barrier leak".

Usage:

    scripts/audit_free_leaks.py \\
        --pgm-yaml docs/maps/campus/v2/final_rayon.yaml \\
        --traj     docs/maps/campus/traj_lidar.txt

  Optional:
    --top N           dump up to N leaked-cell coordinates (default 20)
    --window N        neighbourhood size for metric (a), default 15
    --unknown-ratio X threshold for metric (a), default 0.70
"""

import argparse
import pathlib
import sys

import cv2
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
    p.add_argument('--pgm-yaml', required=True, type=pathlib.Path,
                   help='Nav2 pgm yaml (final_rayon.yaml). Loads pgm + origin '
                        '+ resolution from here.')
    p.add_argument('--traj', required=True, type=pathlib.Path,
                   help='TUM trajectory file. Used for metric (b) to identify '
                        'the traj-reachable component.')
    p.add_argument('--window', type=int, default=15,
                   help='Neighbourhood window size (px) for metric (a). Default 15.')
    p.add_argument('--unknown-ratio', type=float, default=0.70,
                   help='UNKNOWN fraction in the window that flags a FREE cell '
                        'as building-leak candidate for metric (a). Default 0.70.')
    p.add_argument('--top', type=int, default=20,
                   help='Print up to this many leaked-cell coords per metric.')
    p.add_argument('--label', default='',
                   help='Optional label emitted in the header (e.g. "before" / "after").')
    p.add_argument('--refined-min-occ-dist-m', type=float, default=3.0,
                   help='For the refined (a\') metric: only count leak cells whose '
                        'distance to the NEAREST OCCUPIED is > this many metres. '
                        'Excludes thin raycast rays that hug walls (not building '
                        'interior leakage) from the count. Default 3.0 m.')
    p.add_argument('--fragment-erode-m', type=float, default=0.35,
                   help='Metric (c) FREE connectivity: erode FREE by this radius '
                        '(m) then count 8-connected components. Smaller = tighter '
                        'connectivity gate. Default 0.35 m ≈ wheelchair half-width. '
                        'Lower fragment count = better planning connectivity for '
                        'Nav2.')
    return p.parse_args()


def load_pgm_with_yaml(path):
    with path.open() as f:
        y = yaml.safe_load(f)
    pgm_path = path.parent / y['image']
    pgm = np.array(Image.open(pgm_path))
    return pgm, float(y['resolution']), y['origin']


def load_traj_pixels(path, origin, resolution, H, W):
    ox, oy = float(origin[0]), float(origin[1])
    pxs, pys = [], []
    with path.open() as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#'):
                continue
            p = line.split()
            if len(p) < 4:
                continue
            x, y = float(p[1]), float(p[2])
            px = int(round((x - ox) / resolution))
            py = H - 1 - int(round((y - oy) / resolution))
            if 0 <= px < W and 0 <= py < H:
                pxs.append(px); pys.append(py)
    return np.asarray(pxs, dtype=int), np.asarray(pys, dtype=int)


def metric_a_building_leak(pgm, window, unknown_ratio):
    """Return (count, cell_coords_top). See file docstring."""
    H, W = pgm.shape
    is_unknown = (pgm == UNKNOWN).astype(np.uint8)
    is_free = pgm == FREE
    # Window sum of UNKNOWN using boxFilter (normalises = averages).
    unknown_frac = cv2.boxFilter(
        is_unknown.astype(np.float32),
        ddepth=-1,
        ksize=(window, window),
        borderType=cv2.BORDER_REPLICATE,
    )
    leak_mask = is_free & (unknown_frac > unknown_ratio)
    n_leak = int(leak_mask.sum())
    ys, xs = np.where(leak_mask)
    return n_leak, (xs, ys), leak_mask


def metric_b_step_bypass(pgm, traj_px, traj_py):
    """PROXY implementation — see file docstring "CAVEAT" note.

    Returns (count, max_depth_m_or_None, leak_mask). This connected-
    components approach detects only FREE regions that are COMPLETELY
    isolated from the traj-reachable component, missing the more
    common case of a FREE finger that extends past a barrier but
    stays connected to the main free region via a gap. Proper
    implementation per the original spec is a backlog issue.
    """
    is_occ = pgm == OCCUPIED
    is_free = pgm == FREE
    traversable = (~is_occ).astype(np.uint8)   # FREE + UNKNOWN are traversable
    n_labels, labels = cv2.connectedComponents(traversable, connectivity=4)
    if len(traj_px) == 0:
        return 0, None, np.zeros_like(is_free)
    # Find the set of traj labels (usually one giant component).
    traj_labels = set(int(x) for x in labels[traj_py, traj_px] if int(x) != 0)
    if not traj_labels:
        return 0, None, np.zeros_like(is_free)
    reachable = np.zeros(n_labels, dtype=bool)
    for lbl in traj_labels:
        reachable[lbl] = True
    reachable_mask = reachable[labels]
    leak_mask = is_free & ~reachable_mask
    n_leak = int(leak_mask.sum())
    if n_leak == 0:
        return 0, 0.0, leak_mask
    # Max intrusion depth: for each leaked FREE cell, distance to nearest
    # OCCUPIED cell. Approximates "how deep behind the wall we leaked".
    dt_occ = cv2.distanceTransform(
        (~is_occ).astype(np.uint8), cv2.DIST_L2, 5)
    max_depth_px = float(dt_occ[leak_mask].max())
    return n_leak, max_depth_px, leak_mask


def print_top_cells(name, xs, ys, top, pgm, origin, resolution):
    H = pgm.shape[0]
    ox, oy = float(origin[0]), float(origin[1])
    n_show = min(top, len(xs))
    if n_show == 0:
        return
    # Sample evenly across the leak set so we get spatial spread.
    sample_idx = np.linspace(0, len(xs) - 1, n_show).astype(int)
    print(f'  top-{n_show} leaked-cell coords ({name}):')
    print(f'  {"pixel(x,y)":<20} {"map(x_m, y_m)":<26}')
    for i in sample_idx:
        px, py = int(xs[i]), int(ys[i])
        x_map = ox + px * resolution
        y_map = oy + (H - 1 - py) * resolution
        print(f'  pixel({px:>5}, {py:>5})  map({x_map:>8.3f}, {y_map:>8.3f}) m')


def main():
    args = parse_args()
    if not args.pgm_yaml.is_file():
        raise SystemExit(f'--pgm-yaml not found: {args.pgm_yaml}')
    if not args.traj.is_file():
        raise SystemExit(f'--traj not found: {args.traj}')

    pgm, resolution, origin = load_pgm_with_yaml(args.pgm_yaml)
    H, W = pgm.shape
    n_cells = W * H
    n_free = int((pgm == FREE).sum())
    n_occ = int((pgm == OCCUPIED).sum())
    n_unk = int((pgm == UNKNOWN).sum())

    tag = f' [{args.label}]' if args.label else ''
    print(f'== audit_free_leaks{tag} ==')
    print(f'pgm      : {args.pgm_yaml.parent / yaml.safe_load(args.pgm_yaml.open())["image"]}')
    print(f'grid     : {W} x {H} @ {resolution} m/px, origin={origin}')
    print(f'FREE     : {n_free:>10,} ({100*n_free/n_cells:.2f}%)')
    print(f'OCCUPIED : {n_occ:>10,} ({100*n_occ/n_cells:.2f}%)')
    print(f'UNKNOWN  : {n_unk:>10,} ({100*n_unk/n_cells:.2f}%)')
    print()

    print(f'--- (a) 建物リーク: FREE cells with {args.window}x{args.window} '
          f'window UNKNOWN > {args.unknown_ratio:.0%} ---')
    n_a, (xs_a, ys_a), leak_a = metric_a_building_leak(
        pgm, args.window, args.unknown_ratio)
    print(f'  count: {n_a:,}')
    # Refined (a\'): subset of (a) that is > refined_min_occ_dist_m from any
    # OCC. Excludes thin raycast rays that hug walls — those are legitimate
    # ray-added FREE, not building interior leakage.
    is_occ = pgm == OCCUPIED
    dt_occ = cv2.distanceTransform((~is_occ).astype(np.uint8), cv2.DIST_L2, 5)
    depth_m = dt_occ * resolution
    leak_a_refined = leak_a & (depth_m > args.refined_min_occ_dist_m)
    n_a_refined = int(leak_a_refined.sum())
    print(f'  refined (a\', > {args.refined_min_occ_dist_m} m from any OCC): '
          f'{n_a_refined:,}  ← 「真の building interior leak」の目安')
    print_top_cells('metric a', xs_a, ys_a, args.top, pgm, origin, resolution)
    if n_a_refined > 0:
        ys_ar, xs_ar = np.where(leak_a_refined)
        print(f'  refined leak cells (a\', top-{min(args.top, n_a_refined)}):')
        print_top_cells('metric a\'', xs_ar, ys_ar, args.top, pgm, origin, resolution)
    print()

    print(f'--- (b) 段差越境 free: FREE cells NOT reachable from traj via ¬OCC ---')
    traj_px, traj_py = load_traj_pixels(args.traj, origin, resolution, H, W)
    n_b, max_depth_px, leak_b = metric_b_step_bypass(pgm, traj_px, traj_py)
    print(f'  count: {n_b:,}')
    if n_b > 0 and max_depth_px is not None:
        max_depth_m = max_depth_px * resolution
        print(f'  max intrusion depth: {max_depth_px:.2f} px = {max_depth_m:.3f} m '
              f'(nearest-OCC distance for the deepest leaked cell)')
    ys_b, xs_b = np.where(leak_b)
    print_top_cells('metric b', xs_b, ys_b, args.top, pgm, origin, resolution)
    print()

    # (c) FREE connectivity: erode-and-count.
    print(f'--- (c) FREE connectivity: erode {args.fragment_erode_m} m then 8-conn count ---')
    r_cells = max(1, int(round(args.fragment_erode_m / resolution)))
    k = 2 * r_cells + 1
    kern = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (k, k))
    free_bool = (pgm == FREE).astype(np.uint8)
    eroded = cv2.erode(free_bool, kern)
    n_frag_labels, _ = cv2.connectedComponents(eroded, connectivity=8)
    n_fragments = n_frag_labels - 1
    n_eroded = int(eroded.sum())
    print(f'  erosion radius: {r_cells} cells ({args.fragment_erode_m} m)')
    print(f'  eroded FREE cells: {n_eroded:,}')
    print(f'  connected fragments: {n_fragments:,}  ← lower is better for Nav2 planning')
    print()

    # Machine-readable one-liner for scripting
    print(f'RESULT{tag}: metric_a={n_a} metric_a_refined={n_a_refined} '
          f'metric_b={n_b} max_depth_m='
          f'{max_depth_px * resolution if (max_depth_px is not None and n_b > 0) else 0:.3f} '
          f'metric_c_fragments={n_fragments}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
