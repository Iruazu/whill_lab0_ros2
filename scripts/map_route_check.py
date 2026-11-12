#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Check that every dispatch waypoint lies on FREE and that all of them are
connected through FREE space wide enough for the chair (#154, #155).

This is an offline stand-in for "Nav2 can plan the whole route": the global
planner runs with allow_unknown:false, so a route exists only through FREE
cells, and the chair needs a corridor at least its inscribed radius wide. The
FREE set is eroded by --radius and connected components are compared. It does
not run Nav2 (costmap inflation cost, smoother and controller are not modelled);
passing this is necessary, not sufficient.

A waypoint whose own cell becomes non-FREE after the erosion is snapped to the
nearest eroded-FREE cell within --snap, and the snap distance is reported:
a waypoint at a road edge is still reachable by the planner's goal tolerance.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
from PIL import Image
import yaml


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('map_yaml')
    ap.add_argument('waypoints_yaml')
    ap.add_argument('--radius', type=float, default=0.35,
                    help='erosion radius [m] (same as audit_free_leaks.py metric c)')
    ap.add_argument('--snap', type=float, default=0.5, help='max snap distance [m]')
    a = ap.parse_args()

    m = yaml.safe_load(Path(a.map_yaml).read_text())
    img = np.array(Image.open(Path(a.map_yaml).parent / m['image']).convert('L'))
    res, (ox, oy) = float(m['resolution']), m['origin'][:2]
    H, W = img.shape
    # map_server trinary with negate 0: p = (255 - v) / 255, free if p < free_thresh.
    free = ((255 - img.astype(np.float32)) / 255.0) < float(m.get('free_thresh', 0.196))
    r = max(1, int(round(a.radius / res)))
    k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2 * r + 1, 2 * r + 1))
    eroded = cv2.erode(free.astype(np.uint8), k).astype(bool)
    n, lab = cv2.connectedComponents(eroded.astype(np.uint8), connectivity=8)
    dist, (iy, ix) = None, (None, None)
    wps = yaml.safe_load(Path(a.waypoints_yaml).read_text())['waypoints']
    out = []
    for w in wps:
        c = int(np.floor((w['x'] - ox) / res))
        rr = int(H - 1 - np.floor((w['y'] - oy) / res))
        rec = {'name': w['name'], 'pixel': [c, rr], 'value': None, 'free': False,
               'component': 0, 'snap_m': None}
        if 0 <= c < W and 0 <= rr < H:
            rec['value'] = int(img[rr, c])
            rec['free'] = bool(free[rr, c])
            s = int(round(a.snap / res))
            win = eroded[max(0, rr - s):rr + s + 1, max(0, c - s):c + s + 1]
            ys, xs = np.nonzero(win)
            if len(ys):
                d = np.hypot(ys + max(0, rr - s) - rr, xs + max(0, c - s) - c)
                j = int(np.argmin(d))
                rec['snap_m'] = round(float(d[j] * res), 3)
                rec['component'] = int(lab[ys[j] + max(0, rr - s), xs[j] + max(0, c - s)])
        out.append(rec)
    comps = {o['component'] for o in out}
    result = {'map': a.map_yaml, 'radius_m': a.radius, 'free_cells': int(free.sum()),
              'eroded_free_components': int(n - 1), 'waypoints': out,
              'all_connected': len(comps) == 1 and 0 not in comps}
    print(json.dumps(result, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
