#!/usr/bin/env python3
# Copyright (c) 2026 Iruazu / Utsunomiya University Systems Lab
# SPDX-License-Identifier: BSD-3-Clause
"""Republish /imu/data_raw with linear_acceleration negated (Issue #56).

なぜこのノードが要るか:
  RT 9 軸 IMU の中身は PCMK-G3X (MPU-9250 + LPC1343F の USB ファーム)。
  この LPC1343F ファームウェアが accel を「重力加速度ベクトルそのもの」
  (静止時 +Z up で z ≈ -9.81) として出してくる。REP-145 の specific
  force 規約 (静止時 z ≈ +9.81) とは符号が反転している。
  上流ドライバ (rt_usb_9axisimu_driver) は USB バイト列を 1:1 で
  msgs にコピーするだけで、この反転を補正していない。

  本来は M5R-3 の bag リライト (scripts/m5r3_fix_imu_bag.py) と同じ
  処理だが、それは「過去 bag を後追いで直す」用途。本ノードは
  runtime で /imu/data_raw → /imu/data_rep145 にリパブリッシュし、
  EKF / 将来の scan-to-map localizer / 新規録画 bag が REP-145 準拠の
  topic を直接購読できるようにする。/imu/data_raw 自体には触れない
  (後方互換、および「生 topic を見たい」デバッグ用途を保つ)。

  処理は accel の符号反転のみ。共分散、orientation、angular_velocity、
  header、linear_acceleration_covariance は一切いじらない (MPU-9250 の
  gyro/mag は REP-103 準拠で、ファームでも反転されていないため)。

QoS:
  上流ドライバは create_publisher(..., 1) すなわち rclcpp デフォルト
  プロファイル (RELIABLE / VOLATILE / KEEP_LAST / depth=1) で publish
  する。sub/pub の契約を対称にするため本ノードも同じ depth=1 の
  デフォルトプロファイルを使う。robot_localization 側も IMU sub は
  デフォルトプロファイルなので接続性は変わらない。
"""

from __future__ import annotations

import copy

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import Imu

INPUT_TOPIC = '/imu/data_raw'
OUTPUT_TOPIC = '/imu/data_rep145'


class ImuSignCorrector(Node):
    def __init__(self) -> None:
        super().__init__('imu_sign_corrector')
        # depth=1 は上流ドライバ (rt_usb_9axisimu_driver_component.cpp:115)
        # と完全一致させる。100 Hz publish に対して 1:1 リパブリッシュなので
        # キューを伸ばす意味がなく、伸ばすと逆に古いサンプルが残る。
        qos = QoSProfile(depth=1)
        self._pub = self.create_publisher(Imu, OUTPUT_TOPIC, qos)
        self._sub = self.create_subscription(
            Imu, INPUT_TOPIC, self._on_imu, qos,
        )
        self.get_logger().info(
            f'republishing {INPUT_TOPIC} -> {OUTPUT_TOPIC} '
            'with linear_acceleration negated (REP-145 specific force).',
        )

    def _on_imu(self, msg: Imu) -> None:
        # copy.copy ではなく deepcopy する: Imu は配列 (covariance) を持つ
        # ため、shallow copy だと共有参照が残り、将来の改修で配列を
        # 書き換えたときに sub 側のメッセージを壊す事故になりうる。
        # 100 Hz の Imu (~360 B) なので deepcopy のコストは無視できる。
        out = copy.deepcopy(msg)
        out.linear_acceleration.x = -msg.linear_acceleration.x
        out.linear_acceleration.y = -msg.linear_acceleration.y
        out.linear_acceleration.z = -msg.linear_acceleration.z
        self._pub.publish(out)


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    node = ImuSignCorrector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
