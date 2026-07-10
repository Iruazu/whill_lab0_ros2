#!/usr/bin/env python3
# calibration-ledger §更新プロトコル で参照される frame_audit ツール。
#
# 静止 IMU の /imu/data_rep145 実測 (ax, ay, az) から base_link -> imu_link
# の RPY (yaw=0 前提) と quaternion を逆算する。static_tf_launch.py に
# コピペできる形で出力する。
#
# 前提:
#  - imu_link の +x は base_link +x (= 前進方向) と axis-aligned
#    (IMU 物理再マウントで確保)
#  - 従って yaw = 0。roll (x 軸周り) と pitch (y 軸周り) だけを解く
#  - /imu/data_rep145 は REP-145 で specific force (= -gravity) を返す
#  - 静止 IMU では ax = -g*sin(p), ay = +g*sin(r)*cos(p),
#    az = +g*cos(r)*cos(p) となる (g は IMU スケール込みの |g|)
#
# 使い方:
#   python3 scratchpad/frame_audit.py --ax 1.397 --ay -0.699 --az 9.782

import argparse
import math


def solve(ax, ay, az):
    g = math.sqrt(ax * ax + ay * ay + az * az)
    # ax = -g * sin(p) → p = arcsin(-ax/g)
    p = math.asin(-ax / g)
    cp = math.cos(p)
    # ay = g * sin(r) * cos(p) → r = arcsin(ay / (g*cos(p)))
    r = math.asin(ay / (g * cp))
    return r, p, g


def rpy_to_quat_zyx(roll, pitch, yaw):
    # ROS 標準の Rz(yaw) * Ry(pitch) * Rx(roll) 分解 (静的軸 XYZ 順に等価)
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ax", type=float, required=True, help="mean linear_acceleration.x [m/s^2]")
    ap.add_argument("--ay", type=float, required=True, help="mean linear_acceleration.y [m/s^2]")
    ap.add_argument("--az", type=float, required=True, help="mean linear_acceleration.z [m/s^2]")
    args = ap.parse_args()

    r, p, g = solve(args.ax, args.ay, args.az)
    qx, qy, qz, qw = rpy_to_quat_zyx(r, p, 0.0)

    # 検算 (predicted vs input)
    ax_pred = -g * math.sin(p)
    ay_pred = g * math.sin(r) * math.cos(p)
    az_pred = g * math.cos(r) * math.cos(p)

    print("Input (stationary IMU, /imu/data_rep145 mean):")
    print(f"  ax = {args.ax:+.4f}")
    print(f"  ay = {args.ay:+.4f}")
    print(f"  az = {args.az:+.4f}")
    print(f"  |g| effective = {g:.4f}")
    print()
    print("Derived RPY (base_link -> imu_link, yaw fixed to 0):")
    print(f"  roll  = {r:+.4f} rad  ({math.degrees(r):+.3f}°)")
    print(f"  pitch = {p:+.4f} rad  ({math.degrees(p):+.3f}°)")
    print(f"  yaw   =  0.0000 rad  ( 0.000°)")
    print()
    print("Quaternion (x, y, z, w) — Rz(0)*Ry(pitch)*Rx(roll):")
    print(f"  ({qx:+.6f}, {qy:+.6f}, {qz:+.6f}, {qw:+.6f})")
    print()
    print("Roundtrip check (predicted from RPY vs measured):")
    print(f"  ax: pred {ax_pred:+.4f}  meas {args.ax:+.4f}  Δ {ax_pred - args.ax:+.4f}")
    print(f"  ay: pred {ay_pred:+.4f}  meas {args.ay:+.4f}  Δ {ay_pred - args.ay:+.4f}")
    print(f"  az: pred {az_pred:+.4f}  meas {args.az:+.4f}  Δ {az_pred - args.az:+.4f}")
    print()
    print("Copy-paste into static_tf_launch.py:")
    print(f"  _static_tf('static_tf_imu',")
    print(f"             0.38, -0.03, 0.47,")
    print(f"             {r:+.4f}, {p:+.4f}, 0.0,   # roll={math.degrees(r):+.2f}°, pitch={math.degrees(p):+.2f}°, yaw=0")
    print(f"             'base_link', 'imu_link'),")


if __name__ == "__main__":
    main()
