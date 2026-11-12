#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Find the transform from an old map frame to a rebuilt map frame (#154, #155).

Both maps come from GLIM runs on the same bag, so their trajectories share
timestamps: pose i of the old run and the pose of the new run at the same stamp
are the same physical sensor position. A least-squares rigid fit over all
matched positions (Umeyama, no scale) therefore gives T_new_old without any
point cloud registration and without a hand-picked initial guess.

The fit is global. The two maps also differ locally (different drift, the old
one built from mis-calibrated clouds), so the script reports the per-pose
residual after the fit and, for every query point (waypoints), the residual of
the nearest old pose. That residual is how far a point placed by hand in the
old map would be off in the new map after applying T: use it to decide whether
a waypoint can be carried over or must be re-picked.

Optional --old-pcd/--new-pcd refine T with point-to-plane ICP (open3d) and
report the fitness, as an independent check of the trajectory fit.

Outputs (in --out): T_new_old.txt (4x4), se2.json (x, y, yaw for 2D grids),
align.json (statistics), waypoints_transformed.yaml if --waypoints is given.
Inputs are only read.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import yaml


def load_tum(path):
    rows = [list(map(float, ln.split())) for ln in Path(path).read_text().splitlines()
            if ln.strip() and not ln.startswith('#')]
    return np.array(rows)


def umeyama(src, dst):
    mu_s, mu_d = src.mean(0), dst.mean(0)
    H = (src - mu_s).T @ (dst - mu_d)
    U, _, Vt = np.linalg.svd(H)
    D = np.eye(3)
    D[2, 2] = np.sign(np.linalg.det(Vt.T @ U.T))
    R = Vt.T @ D @ U.T
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = mu_d - R @ mu_s
    return T


def apply(T, p):
    return p @ T[:3, :3].T + T[:3, 3]


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--old-traj', required=True)
    ap.add_argument('--new-traj', required=True)
    ap.add_argument('--old-pcd')
    ap.add_argument('--new-pcd')
    ap.add_argument('--waypoints', help='waypoints.yaml of the old map')
    ap.add_argument('--max-dt', type=float, default=0.02,
                    help='max stamp difference for a pose pair [s]')
    ap.add_argument('--out', required=True, type=Path)
    a = ap.parse_args()
    a.out.mkdir(parents=True, exist_ok=True)

    old, new = load_tum(a.old_traj), load_tum(a.new_traj)
    idx = np.clip(np.searchsorted(new[:, 0], old[:, 0]), 1, len(new) - 1)
    idx -= (old[:, 0] - new[idx - 1, 0]) < (new[idx, 0] - old[:, 0])
    ok = np.abs(new[idx, 0] - old[:, 0]) < a.max_dt
    P, Q = old[ok, 1:4], new[idx[ok], 1:4]
    T = umeyama(P, Q)
    res = np.linalg.norm(apply(T, P) - Q, axis=1)
    res_xy = np.linalg.norm((apply(T, P) - Q)[:, :2], axis=1)
    yaw = math.degrees(math.atan2(T[1, 0], T[0, 0]))
    tilt = math.degrees(math.acos(min(1.0, T[2, 2])))
    stats = {'pairs': int(ok.sum()), 'old_poses': int(len(old)), 'new_poses': int(len(new)),
             'T_xyz_m': np.round(T[:3, 3], 4).tolist(), 'yaw_deg': round(yaw, 4),
             'z_axis_tilt_deg': round(tilt, 4),
             'residual_3d_m': {k: round(float(np.percentile(res, q)), 3)
                               for k, q in (('p50', 50), ('p95', 95), ('max', 100))},
             'residual_xy_m': {k: round(float(np.percentile(res_xy, q)), 3)
                               for k, q in (('p50', 50), ('p95', 95), ('max', 100))}}

    if a.old_pcd and a.new_pcd:
        import open3d as o3d
        src = o3d.io.read_point_cloud(a.old_pcd).voxel_down_sample(0.3)
        dst = o3d.io.read_point_cloud(a.new_pcd).voxel_down_sample(0.3)
        dst.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30))
        before = o3d.pipelines.registration.evaluate_registration(src, dst, 0.3, T)
        icp = o3d.pipelines.registration.registration_icp(
            src, dst, 0.5, T, o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
        D = np.linalg.inv(T) @ icp.transformation
        stats['icp'] = {
            'fitness_traj_T_0p3m': round(before.fitness, 4),
            'rmse_traj_T_0p3m': round(before.inlier_rmse, 4),
            'fitness_icp_0p5m': round(icp.fitness, 4), 'rmse_icp_0p5m': round(icp.inlier_rmse, 4),
            'icp_minus_traj_xyz_m': np.round(D[:3, 3], 3).tolist(),
            'icp_minus_traj_rot_deg': round(math.degrees(math.acos(
                max(-1.0, min(1.0, (np.trace(D[:3, :3]) - 1) / 2)))), 4)}
        np.savetxt(a.out / 'T_new_old_icp.txt', icp.transformation)

    np.savetxt(a.out / 'T_new_old.txt', T)
    (a.out / 'se2.json').write_text(json.dumps(
        {'x': float(T[0, 3]), 'y': float(T[1, 3]), 'yaw_rad': math.radians(yaw)}, indent=2))
    np.savetxt(a.out / 'residual_per_pose.txt',
               np.c_[old[ok, 0], P, res_xy], header='t old_x old_y old_z residual_xy_m')

    if a.waypoints:
        wp = yaml.safe_load(Path(a.waypoints).read_text())
        out = []
        for w in wp['waypoints']:
            p = np.array([w['x'], w['y'], 0.0])
            # z of the old map at that place: take the nearest old pose's z so
            # the 3D transform is applied at ground level, not at z=0.
            j = int(np.argmin(np.linalg.norm(P[:, :2] - p[:2], axis=1)))
            p[2] = P[j, 2]
            q = apply(T, p)
            out.append({'name': w['name'], 'old_xy': [w['x'], w['y']],
                        'new_xy': [round(float(q[0]), 3), round(float(q[1]), 3)],
                        'shift_m': round(float(np.linalg.norm(q[:2] - p[:2])), 3),
                        'nearest_old_pose_m': round(float(np.linalg.norm(P[j, :2] - p[:2])), 2),
                        'local_residual_xy_m': round(float(res_xy[j]), 3),
                        'new_yaw': round(float(w.get('yaw', 0.0)) + math.radians(yaw), 4)})
        stats['waypoints'] = out
        (a.out / 'waypoints_transformed.yaml').write_text(yaml.safe_dump(out, allow_unicode=True,
                                                                         sort_keys=False))
    (a.out / 'align.json').write_text(json.dumps(stats, indent=2, ensure_ascii=False))
    print(json.dumps(stats, indent=2, ensure_ascii=False))


if __name__ == '__main__':
    main()
