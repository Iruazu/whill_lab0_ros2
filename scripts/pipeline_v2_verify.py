#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""pipeline_v2_verify.py — M1/M2/M3 verification for v2 occupancy pipeline.

Reads a cross_sections.yaml, evaluates each section against the pgm(s)
it references, and prints a tabular summary. Sections with
`status: placeholder` are skipped with a warning — user fills in real
map-frame coordinates before verification is meaningful.

Metrics:

  M1 断面照合 (curb + ditch position, ±1 cell = 5 cm)
    For each section: rasterise segment start_xy → end_xy into pixels,
    find the OCCUPIED cell closest to expected_step_xy, report distance
    in cells. PASS if ≤ 1 cell. Handles both curb (positive step) and
    ditch (negative step). Fails LOUDLY if no OCCUPIED cell on the
    segment — that means Rule 3 placement is off, not near.

  M2 縁石線連続率 (v1 vs v2 comparative)
    For each section: rasterise segment, compute OCCUPIED cell ratio +
    longest gap. Runs against BOTH v1 (occupancy_cleaned.pgm, via
    --v1-pgm) AND v2 (final pgm from cross_sections.yaml). PASS if
    v2_ratio >= v1_ratio — i.e. v2 did not break v1's curb continuity.
    Longest-gap comparison is reported for information but not part of
    the pass/fail gate.

  M3 建物幅 (壁+庇, ±1 cell)
    For each section: rasterise segment across a building, find first
    and last OCCUPIED cells along the segment, compute their linear
    distance. PASS if |measured - expected_width_m| ≤ resolution.
    Rule 3 (h=2.2m walkable clearance) admits eaves 2.0-2.2m, so
    expected_width should be the wall+eave real measurement.

Usage:

    scripts/pipeline_v2_verify.py \\
        --cross-sections docs/maps/campus/v2/cross_sections.yaml \\
        --v1-pgm         docs/maps/campus/occupancy_cleaned.pgm

The Stage 2 grid pgm is auto-loaded from cross_sections.yaml's
`grid_yaml:` field (relative to the yaml file's dir).

Output formatted as GitHub-flavored markdown table for direct paste
into PR comments (add --markdown to switch from text to md).
"""

import argparse
import pathlib
import sys

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
    p.add_argument('--cross-sections', required=True, type=pathlib.Path,
                   help='YAML with m1/m2/m3 section definitions + grid_yaml pointer.')
    p.add_argument('--v1-pgm', type=pathlib.Path,
                   help='Optional v1 pgm (occupancy_cleaned.pgm) for M2 A/B '
                        'comparison. If omitted, M2 reports v2 ratio only and '
                        'skips the v2>=v1 gate.')
    p.add_argument('--markdown', action='store_true',
                   help='Emit GitHub-flavored markdown tables (default: plain text).')
    return p.parse_args()


def map_to_pixel(map_x, map_y, origin_x, origin_y, resolution, H):
    """Nav2 map_server convention: image row 0 = highest map_y."""
    px = int(round((map_x - origin_x) / resolution))
    py = H - 1 - int(round((map_y - origin_y) / resolution))
    return px, py


def line_pixels(x0, y0, x1, y1, W, H):
    """DDA line rasterisation. Returns (xs, ys) inside grid bounds."""
    steps = max(abs(x1 - x0), abs(y1 - y0)) + 1
    xs = np.linspace(x0, x1, steps).round().astype(int)
    ys = np.linspace(y0, y1, steps).round().astype(int)
    valid = (xs >= 0) & (xs < W) & (ys >= 0) & (ys < H)
    return xs[valid], ys[valid]


def load_pgm_with_yaml(yaml_path):
    with yaml_path.open() as f:
        y = yaml.safe_load(f)
    pgm_path = yaml_path.parent / y['image']
    pgm = np.array(Image.open(pgm_path))
    return pgm, float(y['resolution']), y['origin']


def evaluate_m1(pgm, resolution, origin, section):
    H, W = pgm.shape
    ox, oy = float(origin[0]), float(origin[1])
    x0, y0 = map_to_pixel(*section['start_xy'], ox, oy, resolution, H)
    x1, y1 = map_to_pixel(*section['end_xy'], ox, oy, resolution, H)
    esxp, esyp = map_to_pixel(*section['expected_step_xy'], ox, oy, resolution, H)
    xs, ys = line_pixels(x0, y0, x1, y1, W, H)
    if xs.size == 0:
        return {'passed': False, 'delta_cells': None, 'reason': 'segment out of grid'}
    vals = pgm[ys, xs]
    occ_idx = np.where(vals == OCCUPIED)[0]
    if occ_idx.size == 0:
        return {'passed': False, 'delta_cells': None,
                'reason': 'no OCCUPIED on segment (Rule 3 misplaced?)'}
    dists = np.hypot(xs[occ_idx] - esxp, ys[occ_idx] - esyp)
    min_dist_px = float(dists.min())
    delta_m = min_dist_px * resolution
    return {
        'passed': min_dist_px <= 1.0,
        'delta_cells': min_dist_px,
        'delta_m': delta_m,
        'n_occ_on_segment': int(occ_idx.size),
    }


def evaluate_m2(pgm_v2, pgm_v1, resolution, origin, section):
    H, W = pgm_v2.shape
    ox, oy = float(origin[0]), float(origin[1])
    x0, y0 = map_to_pixel(*section['start_xy'], ox, oy, resolution, H)
    x1, y1 = map_to_pixel(*section['end_xy'], ox, oy, resolution, H)
    xs, ys = line_pixels(x0, y0, x1, y1, W, H)
    if xs.size == 0:
        return {'passed': False, 'reason': 'segment out of grid'}

    def stats(pgm):
        if pgm is None:
            return None, None
        v = pgm[ys, xs]
        occ = v == OCCUPIED
        ratio = float(occ.mean()) if occ.size else 0.0
        cur, max_gap = 0, 0
        for b in occ:
            if not b:
                cur += 1
                if cur > max_gap:
                    max_gap = cur
            else:
                cur = 0
        return ratio, max_gap

    v2_ratio, v2_gap = stats(pgm_v2)
    v1_ratio, v1_gap = stats(pgm_v1)
    if v1_ratio is None:
        return {
            'passed': None,   # gate skipped without v1
            'v2_ratio': v2_ratio, 'v2_gap': v2_gap,
            'v1_ratio': None,  'v1_gap': None,
            'n_cells': int(xs.size),
        }
    return {
        'passed': v2_ratio >= v1_ratio,
        'v2_ratio': v2_ratio, 'v2_gap': v2_gap,
        'v1_ratio': v1_ratio, 'v1_gap': v1_gap,
        'n_cells': int(xs.size),
    }


def evaluate_m3(pgm, resolution, origin, section):
    H, W = pgm.shape
    ox, oy = float(origin[0]), float(origin[1])
    x0, y0 = map_to_pixel(*section['start_xy'], ox, oy, resolution, H)
    x1, y1 = map_to_pixel(*section['end_xy'], ox, oy, resolution, H)
    xs, ys = line_pixels(x0, y0, x1, y1, W, H)
    if xs.size == 0:
        return {'passed': False, 'measured_width_m': None, 'reason': 'segment out of grid'}
    vals = pgm[ys, xs]
    occ_idx = np.where(vals == OCCUPIED)[0]
    if occ_idx.size < 2:
        return {'passed': False, 'measured_width_m': None,
                'reason': f'need ≥2 OCCUPIED cells on segment (got {int(occ_idx.size)})'}
    first, last = int(occ_idx[0]), int(occ_idx[-1])
    seg_dist_px = float(np.hypot(xs[last] - xs[first], ys[last] - ys[first]))
    measured_m = seg_dist_px * resolution
    expected_m = float(section['expected_width_m'])
    delta_m = abs(measured_m - expected_m)
    return {
        'passed': delta_m <= resolution,
        'measured_width_m': measured_m,
        'expected_width_m': expected_m,
        'delta_m': delta_m,
    }


def format_verdict(result):
    if result.get('passed') is True:
        return 'PASS'
    if result.get('passed') is False:
        return 'FAIL'
    return 'N/A'


def print_m1(rows, markdown):
    header = ['section', 'kind', 'verdict', 'Δ cells', 'Δ m', 'n_occ']
    if markdown:
        print('| ' + ' | '.join(header) + ' |')
        print('|' + '|'.join(['---'] * len(header)) + '|')
    else:
        print(f'{"section":<28} {"kind":<6} {"verdict":<8} {"Δ cells":<10} {"Δ m":<10} {"n_occ":<6}')
        print('-' * 78)
    for name, kind, r in rows:
        v = format_verdict(r)
        dc = f'{r["delta_cells"]:.2f}' if r.get('delta_cells') is not None else '—'
        dm = f'{r["delta_m"]:.3f}' if r.get('delta_m') is not None else '—'
        no = str(r.get('n_occ_on_segment', '—'))
        if markdown:
            note = f' ({r["reason"]})' if r.get('reason') else ''
            print(f'| `{name}` | {kind} | **{v}**{note} | {dc} | {dm} | {no} |')
        else:
            print(f'{name:<28} {kind:<6} {v:<8} {dc:<10} {dm:<10} {no:<6}')
            if r.get('reason'):
                print(f'    reason: {r["reason"]}')


def print_m2(rows, markdown):
    header = ['section', 'verdict', 'v2 ratio', 'v1 ratio', 'v2 gap', 'v1 gap', 'cells']
    if markdown:
        print('| ' + ' | '.join(header) + ' |')
        print('|' + '|'.join(['---'] * len(header)) + '|')
    else:
        print(f'{"section":<28} {"verdict":<8} {"v2 ratio":<10} {"v1 ratio":<10} '
              f'{"v2 gap":<8} {"v1 gap":<8} {"cells":<8}')
        print('-' * 90)
    for name, r in rows:
        v = format_verdict(r)
        v2r = f'{r["v2_ratio"]:.3f}' if r.get('v2_ratio') is not None else '—'
        v1r = f'{r["v1_ratio"]:.3f}' if r.get('v1_ratio') is not None else '—'
        v2g = str(r.get('v2_gap', '—'))
        v1g = str(r.get('v1_gap', '—'))
        nc = str(r.get('n_cells', '—'))
        if markdown:
            print(f'| `{name}` | **{v}** | {v2r} | {v1r} | {v2g} | {v1g} | {nc} |')
        else:
            print(f'{name:<28} {v:<8} {v2r:<10} {v1r:<10} {v2g:<8} {v1g:<8} {nc:<8}')


def print_m3(rows, markdown):
    header = ['section', 'verdict', 'measured m', 'expected m', 'Δ m']
    if markdown:
        print('| ' + ' | '.join(header) + ' |')
        print('|' + '|'.join(['---'] * len(header)) + '|')
    else:
        print(f'{"section":<28} {"verdict":<8} {"measured m":<12} {"expected m":<12} {"Δ m":<8}')
        print('-' * 76)
    for name, r in rows:
        v = format_verdict(r)
        mm = f'{r["measured_width_m"]:.3f}' if r.get('measured_width_m') is not None else '—'
        em = f'{r["expected_width_m"]:.3f}' if r.get('expected_width_m') is not None else '—'
        dm = f'{r["delta_m"]:.3f}' if r.get('delta_m') is not None else '—'
        if markdown:
            note = f' ({r["reason"]})' if r.get('reason') else ''
            print(f'| `{name}` | **{v}**{note} | {mm} | {em} | {dm} |')
        else:
            print(f'{name:<28} {v:<8} {mm:<12} {em:<12} {dm:<8}')
            if r.get('reason'):
                print(f'    reason: {r["reason"]}')


def main():
    args = parse_args()
    if not args.cross_sections.is_file():
        raise SystemExit(f'--cross-sections not found: {args.cross_sections}')

    with args.cross_sections.open() as f:
        cs = yaml.safe_load(f)

    grid_yaml_path = args.cross_sections.parent / cs['grid_yaml']
    if not grid_yaml_path.is_file():
        raise SystemExit(f'grid_yaml not found: {grid_yaml_path}')
    pgm_v2, resolution, origin = load_pgm_with_yaml(grid_yaml_path)
    print(f'v2 grid  : {pgm_v2.shape[1]} x {pgm_v2.shape[0]} @ {resolution} m/px, '
          f'origin={origin}')

    pgm_v1 = None
    if args.v1_pgm is not None:
        if not args.v1_pgm.is_file():
            raise SystemExit(f'--v1-pgm not found: {args.v1_pgm}')
        pgm_v1 = np.array(Image.open(args.v1_pgm))
        if pgm_v1.shape != pgm_v2.shape:
            raise SystemExit(f'v1 shape {pgm_v1.shape} != v2 shape {pgm_v2.shape}')
        print(f'v1 grid  : {args.v1_pgm} (loaded for M2 A/B)')
    else:
        print(f'v1 grid  : (not provided — M2 will report v2-only)')

    n_pl = 0
    m1_rows, m2_rows, m3_rows = [], [], []

    for section in cs.get('m1_sections', []):
        if section.get('status') == 'placeholder':
            n_pl += 1
            continue
        r = evaluate_m1(pgm_v2, resolution, origin, section)
        m1_rows.append((section['id'], section.get('expected_side_kind', '?'), r))

    for section in cs.get('m2_sections', []):
        if section.get('status') == 'placeholder':
            n_pl += 1
            continue
        r = evaluate_m2(pgm_v2, pgm_v1, resolution, origin, section)
        m2_rows.append((section['id'], r))

    for section in cs.get('m3_sections', []):
        if section.get('status') == 'placeholder':
            n_pl += 1
            continue
        r = evaluate_m3(pgm_v2, resolution, origin, section)
        m3_rows.append((section['id'], r))

    print()
    if n_pl:
        print(f'⚠ {n_pl} placeholder sections SKIPPED. Fill real coords + '
              f'set status: to something other than "placeholder" to enable them.')
        print()

    sep = '\n\n' if args.markdown else '\n'

    if m1_rows:
        print(f'{"### " if args.markdown else "== "}M1 断面照合 (±1 cell = 5 cm) ==')
        print_m1(m1_rows, args.markdown)
        print(sep, end='')
    if m2_rows:
        print(f'{"### " if args.markdown else "== "}M2 縁石線連続率 (v2 ≥ v1) ==')
        print_m2(m2_rows, args.markdown)
        print(sep, end='')
    if m3_rows:
        print(f'{"### " if args.markdown else "== "}M3 建物幅 (壁+庇, ±1 cell) ==')
        print_m3(m3_rows, args.markdown)
        print(sep, end='')

    if not (m1_rows or m2_rows or m3_rows):
        print('No non-placeholder sections evaluated. Nothing to report.')
        return 0

    # Overall verdict
    all_ok = (
        all(r.get('passed') is True for _, _, r in m1_rows)
        and all(r.get('passed') in (True, None) for _, r in m2_rows)
        and all(r.get('passed') is True for _, r in m3_rows)
    )
    print()
    if all_ok:
        print('OVERALL: ✅ all evaluated sections PASS')
    else:
        print('OVERALL: ❌ at least one section FAILED — inspect details above')
    return 0 if all_ok else 1


if __name__ == '__main__':
    sys.exit(main())
