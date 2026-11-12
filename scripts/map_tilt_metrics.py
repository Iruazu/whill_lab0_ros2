#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Tilt and size metrics of a GLIM/DUFOMap map (#153).

The campus map (docs/maps/campus/README.md, "1.81° tilt") was judged by a plane
fitted to the trajectory. This script repeats exactly that fit so a rebuilt map
can be compared with the same number, and adds two measures the trajectory fit
cannot give:

* trajectory plane z = a*x + b*y + c (least squares over all poses): tilt of
  the plane against the map z axis, its steepest azimuth, and the residual RMS.
  A sloped campus also gives a non-zero value, so this alone does not separate
  "map is tilted" from "ground is sloped".
* local ground normal under the start and the end pose, from the static PCD:
  points within --radius of the pose, below the sensor, and a RANSAC-free
  robust plane (iteratively re-weighted on the lowest band). Under hypothesis
  A of the campus README (the start pose stood on a tilted surface) the start
  ground normal is tilted against map z by about the same angle; under B (GLIM
  gravity alignment error) the whole map is tilted, so a level-looking start
  surface is tilted in map coordinates as well. The start and end values must
  agree for a closed loop; a gap shows map warp rather than a frame tilt.
* point count of the PCD.

Inputs are only read. Output is JSON on stdout.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np


def load_traj(path):
    rows = [list(map(float, ln.split())) for ln in Path(path).read_text().splitlines()
            if ln.strip() and not ln.startswith('#')]
    return np.array(rows)


def read_pcd_xyz(path):
    # Minimal PCD reader (ascii / binary, x y z float32 first): open3d is not
    # guaranteed on this host and the DUFOMap output is x y z only.
    with open(path, 'rb') as f:
        header = {}
        while True:
            raw = f.readline()
            if not raw:
                sys.exit(f'{path}: end of file before the DATA line; not a PCD')
            line = raw.decode('ascii', 'replace').strip()
            if not line or line.startswith('#'):
                continue
            key, *vals = line.split()
            header[key.upper()] = vals
            if key.upper() == 'DATA':
                break
        fields = header['FIELDS']
        sizes = list(map(int, header['SIZE']))
        types = header['TYPE']
        counts = list(map(int, header.get('COUNT', ['1'] * len(fields))))
        n = int(header['POINTS'][0])
        kind = header['DATA'][0]
        if kind == 'ascii':
            arr = np.loadtxt(f, dtype=np.float64, max_rows=n)
            idx = [fields.index(c) for c in 'xyz']
            return arr[:, idx]
        if kind != 'binary':
            sys.exit(f'unsupported PCD DATA {kind}')
        tmap = {('F', 4): 'f4', ('F', 8): 'f8', ('U', 1): 'u1', ('U', 2): 'u2',
                ('U', 4): 'u4', ('I', 1): 'i1', ('I', 2): 'i2', ('I', 4): 'i4'}
        dt = np.dtype([(fld, tmap[(t, s)]) if c == 1 else (fld, tmap[(t, s)], (c,))
                       for fld, s, t, c in zip(fields, sizes, types, counts)])
        data = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        return np.stack([data['x'], data['y'], data['z']], axis=1).astype(np.float64)


def tilt_of_normal(nrm):
    nrm = nrm / np.linalg.norm(nrm)
    if nrm[2] < 0:
        nrm = -nrm
    tilt = math.degrees(math.acos(min(1.0, nrm[2])))
    # azimuth of the downhill direction is irrelevant; report the uphill one,
    # same convention as the README ("steepest azimuth from +x").
    az = math.degrees(math.atan2(-nrm[1], -nrm[0])) % 360
    return tilt, az


def traj_plane(traj):
    x, y, z = traj[:, 1], traj[:, 2], traj[:, 3]
    A = np.c_[x, y, np.ones_like(x)]
    (a, b, c), *_ = np.linalg.lstsq(A, z, rcond=None)
    resid = z - A @ np.array([a, b, c])
    tilt, az = tilt_of_normal(np.array([-a, -b, 1.0]))
    return {'a': float(a), 'b': float(b), 'c': float(c), 'tilt_deg': round(tilt, 3),
            'steepest_azimuth_deg': round(az, 1), 'resid_rms_m': round(float(np.sqrt(np.mean(resid ** 2))), 3),
            'z_span_m': round(float(np.ptp(z)), 3)}


def local_ground(pts, center, radius, sensor_height_max):
    d = np.hypot(pts[:, 0] - center[0], pts[:, 1] - center[1])
    sel = pts[(d < radius) & (pts[:, 2] < center[2] - 0.3)
              & (pts[:, 2] > center[2] - sensor_height_max)]
    if len(sel) < 50:
        return None
    # Start from the lowest 30 % (ground, not curbs or cars), then keep points
    # within 5 cm of the current plane until the plane stops moving.
    keep = sel[sel[:, 2] < np.percentile(sel[:, 2], 30) + 0.2]
    for _ in range(20):
        A = np.c_[keep[:, 0] - center[0], keep[:, 1] - center[1], np.ones(len(keep))]
        coef, *_ = np.linalg.lstsq(A, keep[:, 2], rcond=None)
        r = sel[:, 2] - np.c_[sel[:, 0] - center[0], sel[:, 1] - center[1], np.ones(len(sel))] @ coef
        new = sel[np.abs(r) < 0.05]
        if len(new) == len(keep):
            break
        keep = new
    tilt, az = tilt_of_normal(np.array([-coef[0], -coef[1], 1.0]))
    return {'tilt_deg': round(tilt, 3), 'uphill_azimuth_deg': round(az, 1),
            'ground_z_m': round(float(coef[2]), 3),
            'sensor_above_ground_m': round(float(center[2] - coef[2]), 3),
            'inliers': int(len(keep))}


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('traj', help='TUM trajectory (GLIM traj_lidar.txt)')
    ap.add_argument('--pcd', help='static PCD for the local ground planes and the point count')
    ap.add_argument('--radius', type=float, default=6.0)
    ap.add_argument('--sensor-height-max', type=float, default=2.0,
                    help='ignore points more than this below the sensor (ditches, slopes)')
    a = ap.parse_args()
    traj = load_traj(a.traj)
    out = {'traj': a.traj, 'poses': int(len(traj)), 'traj_plane': traj_plane(traj)}
    if a.pcd:
        pts = read_pcd_xyz(a.pcd)
        out['pcd'] = a.pcd
        out['points'] = int(len(pts))
        out['ground_start'] = local_ground(pts, traj[0, 1:4], a.radius, a.sensor_height_max)
        out['ground_end'] = local_ground(pts, traj[-1, 1:4], a.radius, a.sensor_height_max)
    print(json.dumps(out, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
