#!/usr/bin/env python3
# calibration-ledger §0.2 / §0.5 の 5 秒静止 IMU チェック。
# /imu/data_rep145 を 500 サンプル (100 Hz × 5 s) 拾って平均を出し、
# ledger の期待値と tolerance で PASS/FAIL する。
#
# 期待値 (2026-07-10 朝 3 連続測定の平均、WHILL 静止時):
#   ax = +1.397  ± 0.05
#   ay = -0.699  ± 0.05
#   az = +9.782  ± 0.05
#   gx = -0.0184 ± 0.005
# 対応 static_tf: base_link -> imu_link roll=-4.09°, pitch=-8.11°, yaw=0
#
# 使い方 (bringup 起動済み、WHILL 完全静止で):
#   python3 scratchpad/imu_live_check.py

import sys
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

N_SAMPLES = 500
EXPECT = {
    "ax": (+1.397, 0.05),
    "ay": (-0.699, 0.05),
    "az": (+9.782, 0.05),
    "gx": (-0.0184, 0.005),
}


class ImuChecker(Node):
    def __init__(self):
        super().__init__("imu_live_check")
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
        )
        self.sub = self.create_subscription(Imu, "/imu/data_rep145", self.cb, qos)
        self.ax = []
        self.ay = []
        self.az = []
        self.gx = []
        self.get_logger().info(f"collecting {N_SAMPLES} samples ({N_SAMPLES/100:.1f} s)...")

    def cb(self, msg: Imu):
        if len(self.ax) >= N_SAMPLES:
            return
        self.ax.append(msg.linear_acceleration.x)
        self.ay.append(msg.linear_acceleration.y)
        self.az.append(msg.linear_acceleration.z)
        self.gx.append(msg.angular_velocity.x)
        n = len(self.ax)
        if n % 100 == 0:
            self.get_logger().info(f"  {n}/{N_SAMPLES}")

    def done(self):
        return len(self.ax) >= N_SAMPLES


def mean(xs):
    return sum(xs) / len(xs)


def main():
    rclpy.init()
    node = ImuChecker()
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        got = {
            "ax": mean(node.ax),
            "ay": mean(node.ay),
            "az": mean(node.az),
            "gx": mean(node.gx),
        }
        node.destroy_node()
        rclpy.shutdown()

    print()
    print(f"{'key':>4} {'measured':>10} {'expected':>10} {'tol':>7} {'delta':>8}  verdict")
    print("-" * 60)
    fail = False
    for k in ("ax", "ay", "az", "gx"):
        m = got[k]
        exp, tol = EXPECT[k]
        delta = m - exp
        ok = abs(delta) <= tol
        fail = fail or not ok
        mark = "OK  " if ok else "FAIL"
        print(f"{k:>4} {m:>+10.4f} {exp:>+10.4f} {tol:>7.4f} {delta:>+8.4f}  {mark}")
    print()
    if fail:
        print("VERDICT: FAIL — マウントが動いている可能性。走行に進まないこと。")
        print("  次アクション: 物理再固定 → scratchpad/frame_audit.py で TF 再計算 → static_tf 更新 → ledger 更新。")
        sys.exit(1)
    else:
        print("VERDICT: PASS — 2026-07-10 の T_lidar_imu と整合。走行に進んで良い。")


if __name__ == "__main__":
    main()
