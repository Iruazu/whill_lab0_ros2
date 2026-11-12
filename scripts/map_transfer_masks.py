#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Carry the hand-painted v2 sidecars of an old map onto a rebuilt map (#154).

The hand paint of the campus map lives in two places:
  1. the sidecar masks (roadway_mask / free_mask / keepout_mask.png), which the
     composer applies on top of the machine layers;
  2. edits made directly in final.pgm after composing (2026-07-20..24 field
     fixes: 30dbfb1, 0c903bc, 54ce32b, e741e9a), which no mask contains.
(2) is recovered as the difference between the committed final.pgm and a fresh
compose of the same masks (--old-final vs --old-recomposed).

Everything is resampled into the new grid with T_new_old (map_align.py), nearest
neighbour, no morphology (Rule 1 of the v2 pipeline: do not move edges). The
result is a CANDIDATE: the global transform does not remove local differences
between the two maps (map_align.py reports them), and a curb or a wall that
moved by 10-30 cm makes a carried-over brush stroke land on the wrong side.
The outputs are therefore written as candidate_*.png plus a review overlay, and
never under the sidecar names the composer reads by default; a person accepts
them by copying (or repainting) into roadway_mask.png etc.

Inputs are only read; existing outputs are not overwritten without --force.
"""
import argparse
import json
import math
from pathlib import Path
import sys

import numpy as np
from PIL import Image
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from compose_occupancy import load_sidecar_mask  # noqa: E402

FREE_MIN = 250  # map_server free pixels are 254 (composer) / 255 (hand paint)
OCC_MAX = 5


def grid(layers_yaml):
    g = yaml.safe_load(Path(layers_yaml).read_text())['grid']
    return int(g['width']), int(g['height']), float(g['resolution']), g['origin'][:2]


def pix_to_map(W, H, res, origin):
    # col, row (image, row 0 at top) -> map x, y at the pixel centre
    return np.array([[res, 0, origin[0] + 0.5 * res],
                     [0, -res, origin[1] + (H - 0.5) * res],
                     [0, 0, 1]])


def warp(mask_old, A_old_from_new, W, H):
    rows, cols = np.mgrid[0:H, 0:W]
    pts = np.stack([cols.ravel(), rows.ravel(), np.ones(W * H)])
    src = A_old_from_new @ pts
    c = np.rint(src[0]).astype(np.int64)
    r = np.rint(src[1]).astype(np.int64)
    Ho, Wo = mask_old.shape
    inside = (c >= 0) & (c < Wo) & (r >= 0) & (r < Ho)
    out = np.zeros(W * H, bool)
    out[inside] = mask_old[r[inside], c[inside]]
    return out.reshape(H, W)


def save_mask(path, mask):
    rgba = np.zeros(mask.shape + (4,), np.uint8)
    rgba[mask] = (255, 255, 255, 255)
    Image.fromarray(rgba, 'RGBA').save(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--old-layers-yaml', required=True)
    ap.add_argument('--new-layers-yaml', required=True)
    ap.add_argument('--transform', required=True, help='T_new_old.txt (4x4) from map_align.py')
    ap.add_argument('--mask', action='append', default=[],
                    help='name=path of an old sidecar mask (repeatable)')
    ap.add_argument('--old-final', help='committed final.pgm (with direct edits)')
    ap.add_argument('--old-recomposed', help='compose_occupancy.py output of the same masks')
    ap.add_argument('--new-final', help='new composed pgm, for the review overlay')
    ap.add_argument('--mask-threshold', type=int, default=128)
    ap.add_argument('--out-dir', required=True, type=Path)
    ap.add_argument('--force', action='store_true')
    a = ap.parse_args()
    a.out_dir.mkdir(parents=True, exist_ok=True)

    Wo, Ho, reso, oo = grid(a.old_layers_yaml)
    Wn, Hn, resn, on = grid(a.new_layers_yaml)
    T = np.loadtxt(a.transform)
    yaw = math.atan2(T[1, 0], T[0, 0])
    # 2D part of T: the z tilt between the two frames is a fraction of a degree
    # and shifts horizontal positions by < 1 cm over the map, below one cell.
    S = np.array([[math.cos(yaw), -math.sin(yaw), T[0, 3]],
                  [math.sin(yaw), math.cos(yaw), T[1, 3]], [0, 0, 1]])
    A = np.linalg.inv(pix_to_map(Wo, Ho, reso, oo)) @ np.linalg.inv(S) @ pix_to_map(Wn, Hn, resn, on)

    report = {'transform_se2': {'x': T[0, 3], 'y': T[1, 3], 'yaw_deg': math.degrees(yaw)},
              'old_grid': [Wo, Ho, reso, oo], 'new_grid': [Wn, Hn, resn, on], 'outputs': {}}
    outputs = {}
    for spec in a.mask:
        name, path = spec.split('=', 1)
        m = load_sidecar_mask(Path(path), (Ho, Wo), a.mask_threshold)
        outputs[f'candidate_{name}.png'] = (warp(m, A, Wn, Hn), int(m.sum()))

    if a.old_final and a.old_recomposed:
        fin = np.array(Image.open(a.old_final))
        rec = np.array(Image.open(a.old_recomposed))
        if fin.shape != (Ho, Wo) or rec.shape != (Ho, Wo):
            sys.exit('old final / recomposed pgm do not match the old grid')
        added_free = (fin >= FREE_MIN) & (rec < FREE_MIN)
        removed_free = (fin < FREE_MIN) & (rec >= FREE_MIN)
        added_occ = (fin <= OCC_MAX) & (rec > OCC_MAX)
        report['old_direct_edits_cells'] = {
            'free_added': int(added_free.sum()), 'free_removed': int(removed_free.sum()),
            'occ_added': int(added_occ.sum()),
            'occ_to_free': int(((rec <= OCC_MAX) & (fin >= FREE_MIN)).sum())}
        # Direct edits widened the drivable area (UNKNOWN/OCC -> FREE). In the
        # sidecar model that is both a roadway extension and a free override.
        outputs['candidate_direct_free.png'] = (warp(added_free, A, Wn, Hn), int(added_free.sum()))
        if removed_free.any() or added_occ.any():
            outputs['candidate_direct_occ.png'] = (warp(added_occ | removed_free, A, Wn, Hn),
                                                   int((added_occ | removed_free).sum()))

    # Composer inputs for a candidate final.pgm: a direct UNKNOWN->FREE edit is,
    # in the sidecar model, a roadway extension AND a free override (the
    # composer only frees cells that are inside roadway and free-evidenced or
    # free-masked), so it is merged into both.
    direct = outputs.get('candidate_direct_free.png')
    merged = {}
    if direct is not None:
        for name in ('roadway_mask', 'free_mask'):
            if f'candidate_{name}.png' in outputs:
                merged[f'candidate_{name.split("_")[0]}_merged.png'] = (
                    outputs[f'candidate_{name}.png'][0] | direct[0], -1)

    for fname, (m, n_old) in {**outputs, **merged}.items():
        p = a.out_dir / fname
        if p.exists() and not a.force:
            sys.exit(f'{p} exists; use --force')
        save_mask(p, m)
        report['outputs'][fname] = {'old_cells': n_old, 'new_cells': int(m.sum())}

    if a.new_final:
        base = np.array(Image.open(a.new_final).convert('L'))
        rgb = np.stack([base] * 3, -1).astype(np.float32) * 0.6
        colours = {'candidate_roadway_mask.png': (0, 160, 255), 'candidate_free_mask.png': (255, 220, 0),
                   'candidate_keepout_mask.png': (255, 0, 255), 'candidate_direct_free.png': (0, 255, 0),
                   'candidate_direct_occ.png': (255, 0, 0)}
        for fname, (m, _) in outputs.items():
            c = np.array(colours.get(fname, (255, 255, 255)), np.float32)
            rgb[m] = rgb[m] * 0.4 + c * 0.6
        Image.fromarray(rgb.clip(0, 255).astype(np.uint8)).save(a.out_dir / 'candidate_review_overlay.png')
        report['outputs']['candidate_review_overlay.png'] = 'full resolution; colours: ' + json.dumps(colours)
    (a.out_dir / 'candidate_transfer.json').write_text(json.dumps(report, indent=2, default=float))
    print(json.dumps(report, indent=2, default=float))


if __name__ == '__main__':
    main()
