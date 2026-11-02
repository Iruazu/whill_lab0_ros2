#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Test IMU against IMU-free LiDAR odometry (from lidar_icp_odom.py).

1. time offset tau (IMU stamp + tau = LiDAR time) by maximising the fit between
   scan-to-scan LiDAR angular velocity and gyro averaged over the same interval;
2. gyro-to-LiDAR rotation (hand-eye, Kabsch with joint gyro-bias), gyro scale;
3. accelerometer: LiDAR-derived specific force at the IMU vs measured, as a
   3x3 regression (sign / axis / scale check) and per-frame Delta-v errors in
   the style of GLIM's IMU validation.
Plots go to --plot-dir.
"""
import argparse
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from scipy.signal import butter, filtfilt  # noqa: E402
from scipy.spatial.transform import Rotation as Rot  # noqa: E402

Q_CFG = np.array([-0.035606, -0.066319, -0.002368, 0.997163])   # GLIM audit T_lidar_imu (x,y,z,w)
T_CFG = np.array([-0.05, -0.40, -0.35])                           # IMU origin in LiDAR frame


def kabsch(A, B):
    """R minimising sum |B - R A|^2 (rows are vectors)."""
    H = A.T @ B
    U, S, Vt = np.linalg.svd(H)
    D = np.diag([1, 1, np.sign(np.linalg.det(Vt.T @ U.T))])
    return Vt.T @ D @ U.T, S


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('imu_npz')
    ap.add_argument('odom_npz')
    ap.add_argument('--plot-dir', default='plots')
    ap.add_argument('--tag', default='')
    ap.add_argument('--up-lidar0', nargs=3, type=float, default=None,
                    help='gravity-up unit vector in the first LiDAR frame (floor normal at static start)')
    a = ap.parse_args()
    imu = np.load(a.imu_npz)
    od = np.load(a.odom_npz)
    ti, acc, gyr = imu['stamp'], imu['acc'], imu['gyro']
    ts, T = od['stamp'], od['T']
    R = T[:, :3, :3]
    p = T[:, :3, 3]
    good = od['fitness'] > 0.5
    print(f'[{a.tag}] scans={len(ts)} fitness<0.5: {(~good).sum()}')

    # cumulative gyro integral for exact interval averages
    dti = np.diff(ti, prepend=ti[0])
    cum = np.cumsum(gyr * dti[:, None], axis=0)

    def gyro_avg(t_a, t_b, tau):
        ca = np.stack([np.interp(t_a - tau, ti, cum[:, j]) for j in range(3)], 1)
        cb = np.stack([np.interp(t_b - tau, ti, cum[:, j]) for j in range(3)], 1)
        return (cb - ca) / (t_b - t_a)[:, None]

    dR = np.einsum('nji,njk->nik', R[:-1], R[1:])
    dtl = np.diff(ts)
    wL = Rot.from_matrix(dR).as_rotvec() / dtl[:, None]
    ok = good[1:] & good[:-1]
    taus = np.arange(-0.15, 0.1501, 0.0025)
    cost = []
    for tau in taus:
        wI = gyro_avg(ts[:-1], ts[1:], tau)
        Rk, _ = kabsch(wI[ok] - wI[ok].mean(0), wL[ok] - wL[ok].mean(0))
        cost.append(np.sqrt(np.mean(np.sum((wL[ok] - (wI[ok] - (wI[ok].mean(0) - Rk.T @ wL[ok].mean(0))) @ Rk.T) ** 2, 1))))
    cost = np.array(cost)
    tau_best = taus[np.argmin(cost)]
    # parabolic refinement
    i = np.argmin(cost)
    if 0 < i < len(taus) - 1:
        c0, c1, c2 = cost[i - 1:i + 2]
        tau_best = taus[i] + 0.5 * (c0 - c2) / (c0 - 2 * c1 + c2) * (taus[1] - taus[0])
    print(f'[{a.tag}] time offset tau (LiDAR stamp = IMU stamp + tau): {tau_best * 1e3:+.1f} ms '
          f'(rms omega residual {cost.min():.4f} rad/s; at tau=0: {cost[np.argmin(np.abs(taus))]:.4f})')

    wI = gyro_avg(ts[:-1], ts[1:], tau_best)
    # joint rotation + gyro bias: wL = R (wI - b)
    b = np.zeros(3)
    for _ in range(20):
        Rh, S = kabsch(wI[ok] - b, wL[ok])
        b = (wI[ok] - wL[ok] @ Rh).mean(0)
    Rh, S = kabsch(wI[ok] - b, wL[ok])
    res = wL[ok] - (wI[ok] - b) @ Rh.T
    rpy = Rot.from_matrix(Rh).as_euler('xyz', degrees=True)
    print(f'[{a.tag}] hand-eye R_lidar_imu RPY (deg) = {np.round(rpy, 2)}  singular values {np.round(S, 2)} '
          f'(yaw about the vertical is weakly observable if S[1],S[2] << S[0])')
    print(f'[{a.tag}] gyro bias (rad/s) = {np.round(b, 4)}  residual rms = {np.sqrt((res ** 2).sum(1).mean()):.4f}')
    Rc = Rot.from_quat(Q_CFG).as_matrix()
    print(f'[{a.tag}] GLIM cfg R_lidar_imu RPY = {np.round(Rot.from_matrix(Rc).as_euler("xyz", degrees=True), 2)}; '
          f'angle(cfg, hand-eye) = {np.degrees(Rot.from_matrix(Rc.T @ Rh).magnitude()):.2f} deg')
    # Dominant axis comparison (2-DoF, robust to yaw-only motion)
    U, Sv, Vt = np.linalg.svd(wL[ok], full_matrices=False)
    axL = Vt[0] * np.sign(Vt[0][2])
    U, Sv2, Vt2 = np.linalg.svd(wI[ok] - b, full_matrices=False)
    axI = Vt2[0] * np.sign(Vt2[0][2])
    print(f'[{a.tag}] dominant rotation axis: LiDAR {np.round(axL, 4)}  IMU {np.round(axI, 4)}')
    for nm, Rm in (('cfg', Rc), ('hand-eye', Rh)):
        print(f'[{a.tag}]   angle(R_{nm} * axis_IMU, axis_LiDAR) = {np.degrees(np.arccos(np.clip((Rm @ axI) @ axL, -1, 1))):.2f} deg')
    # gyro scale along the dominant axis
    sL = wL[ok] @ axL
    sI = (wI[ok] - b) @ axI
    k = np.polyfit(sI, sL, 1)
    print(f'[{a.tag}] gyro scale (LiDAR/IMU along yaw axis) = {k[0]:.4f}')

    # ---------- accelerometer ----------
    tm = 0.5 * (ts[:-1] + ts[1:])
    # world-frame velocity of LiDAR origin, then IMU point with lever arm
    r_li = T_CFG
    pI = p + np.einsum('nij,j->ni', R, r_li)
    # smooth positions/orientation-derived quantities with a 1.5 Hz low-pass at scan rate
    fs = 1.0 / np.median(dtl)
    bl, al = butter(2, 1.5 / (fs / 2))
    pIs = filtfilt(bl, al, pI, axis=0)
    vW = np.gradient(pIs, ts, axis=0)
    aW = np.gradient(vW, ts, axis=0)
    if a.up_lidar0 is not None:
        up0 = np.array(a.up_lidar0) / np.linalg.norm(a.up_lidar0)
    else:
        up0 = Rc @ (acc[:500].mean(0) / np.linalg.norm(acc[:500].mean(0)))
    upW = R[0] @ up0  # world = first lidar frame
    fW = aW + 9.80665 * upW
    fL = np.einsum('nji,nj->ni', R, fW)
    fI_pred_cfg = fL @ Rc          # = Rc^T fL
    fI_pred_he = fL @ Rh
    # measured, low-passed identically and sampled at scan times (shifted by tau)
    bi, ai_ = butter(2, 1.5 / 50.0)
    accs = filtfilt(bi, ai_, acc, axis=0)
    fI_meas = np.stack([np.interp(ts - tau_best, ti, accs[:, j]) for j in range(3)], 1)
    w = good.copy()
    w[:20] = False
    w[-20:] = False
    # 3x3 regression meas = M pred + c on the dynamic part
    X = np.c_[fI_pred_cfg[w], np.ones(w.sum())]
    coef, *_ = np.linalg.lstsq(X, fI_meas[w], rcond=None)
    M = coef[:3].T
    print(f'[{a.tag}] accel regression meas = M * pred(cfg) + c :\n{np.round(M, 3)}\n  c = {np.round(coef[3], 3)}')
    for j, n in enumerate('xyz'):
        dyn_m = fI_meas[w, j] - fI_meas[w, j].mean()
        dyn_p = fI_pred_cfg[w, j] - fI_pred_cfg[w, j].mean()
        print(f'[{a.tag}]   axis {n}: corr(meas, pred_cfg) = {np.corrcoef(dyn_m, dyn_p)[0, 1]:+.3f}  '
              f'std meas {dyn_m.std():.3f} pred {dyn_p.std():.3f}; mean meas-pred = {np.mean(fI_meas[w, j] - fI_pred_cfg[w, j]):+.3f}')
    err = fI_meas - fI_pred_cfg
    print(f'[{a.tag}] |meas - pred_cfg| (1.5 Hz LP) median {np.median(np.linalg.norm(err[w], axis=1)):.3f} m/s^2; '
          f'with hand-eye R: {np.median(np.linalg.norm((fI_meas - fI_pred_he)[w], axis=1)):.3f}')

    Path(a.plot_dir).mkdir(parents=True, exist_ok=True)
    fig, ax = plt.subplots(5, 1, figsize=(13, 14), sharex=False)
    ax[0].plot(taus * 1e3, cost)
    ax[0].axvline(tau_best * 1e3, color='k', ls='--')
    ax[0].set_xlabel('tau [ms] (LiDAR stamp = IMU stamp + tau)')
    ax[0].set_ylabel('rms omega resid [rad/s]')
    ax[0].set_title(f'{a.tag}: time-offset search')
    t_rel = tm - ts[0]
    ax[1].plot(t_rel, wL[:, 2], lw=0.6, label='LiDAR yaw rate')
    ax[1].plot(t_rel, (wI - b) @ Rh.T[:, 2], lw=0.6, label='gyro (hand-eye R, bias removed)')
    ax[1].legend()
    ax[1].set_ylabel('rad/s')
    for j, n in enumerate('xy'):
        ax[2 + j].plot(ts[w] - ts[0], fI_meas[w, j], lw=0.6, label=f'IMU acc {n} (1.5 Hz LP)')
        ax[2 + j].plot(ts[w] - ts[0], fI_pred_cfg[w, j], lw=0.6, label=f'LiDAR-derived f_{n} in IMU frame (cfg R)')
        ax[2 + j].legend()
        ax[2 + j].set_ylabel('m/s^2')
    ax[4].plot(ts[w] - ts[0], fI_meas[w, 2], lw=0.6, label='IMU acc z')
    ax[4].plot(ts[w] - ts[0], fI_pred_cfg[w, 2], lw=0.6, label='LiDAR-derived f_z')
    ax[4].legend()
    ax[4].set_xlabel('s from segment start')
    fig.tight_layout()
    out = Path(a.plot_dir) / f'imu_vs_lidar_{a.tag}.png'
    fig.savefig(out, dpi=90)
    print(f'[{a.tag}] plot: {out}')


if __name__ == '__main__':
    main()
