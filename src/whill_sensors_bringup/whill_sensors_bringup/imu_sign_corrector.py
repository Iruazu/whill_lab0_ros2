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
import signal

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import Imu

INPUT_TOPIC = '/imu/data_raw'
OUTPUT_TOPIC = '/imu/data_rep145'
# 終了要求に気づくまでの上限 (main() の説明を参照)。launch は SIGINT から
# SIGTERM まで 5 秒待つので、0.1 秒なら十分に短い。空のコールバックが
# 10 Hz で走るだけで、100 Hz の IMU 処理に比べて負荷は無視できる。
SHUTDOWN_POLL_S = 0.1


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
        # 何もしないタイマー。spin を定期的に C の wait から戻し、Python の
        # シグナルハンドラを走らせるためだけにある (main() の説明を参照)。
        # これが無いと、IMU が止まっているときに SIGINT で終われない。
        self.create_timer(SHUTDOWN_POLL_S, lambda: None)
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


def _raise_keyboard_interrupt(*_) -> None:
    raise KeyboardInterrupt()


def main(args: list[str] | None = None) -> None:
    # 終了処理 (Issue #169)。
    #
    # rclpy 既定のシグナルハンドラ (Humble) は、別スレッドで context を
    # shutdown する。そのため主スレッドの spin と後始末が、そのスレッドと
    # 競合する。2026-11-12 に SIGINT / SIGTERM を各 150 回送って、次を再現した:
    #   - spin から ExternalShutdownException、または wait set 生成時の
    #     RCLError (context is not valid) が上がり、トレースバックで exit 1
    #   - 後始末の try_shutdown が「rcl_shutdown already called」で失敗
    #   - destroy_node と shutdown の競合とみられる SIGSEGV (約 2%)
    # 例外を捕まえるだけでは後の2つが残る。
    #
    # そこで failsafe_node / dispatch_node と同じ方式を拡張する。rclpy の
    # ハンドラを外し、SIGINT / SIGTERM で KeyboardInterrupt を上げる。
    # shutdown するのは主スレッドだけになり、競合が無くなる。両ノードに
    # 無い追加は4つ: (1) Python のハンドラは spin が C の wait から戻った
    # ときにしか走らないので、SHUTDOWN_POLL_S のタイマーで定期的に戻す、
    # (2) executor をノードより先に作る、(3) ノード生成を try の中に置く、
    # (4) 後始末の間は2回目のシグナルを無視する。
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    signal.signal(signal.SIGINT, _raise_keyboard_interrupt)
    signal.signal(signal.SIGTERM, _raise_keyboard_interrupt)

    executor = None
    node = None
    try:
        # executor はノード (準備完了ログ) より先に明示的に作る。rclpy.spin()
        # に任せると spin の入口で executor を作るので、起動直後のシグナルが
        # その生成を途中で断ち、未完成の executor の __del__ が「Exception
        # ignored」のトレースバックを出す (2026-11-12、200 回中 約 35%)。
        executor = SingleThreadedExecutor()
        # ノード生成も try の中に置く。起動直後の Ctrl-C でも後始末を通す。
        node = ImuSignCorrector()
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        # 後始末の途中で2回目のシグナルが来てもトレースバックを出さない。
        # launch は SIGINT の後に SIGTERM を送ることがある。
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        signal.signal(signal.SIGTERM, signal.SIG_IGN)
        # 後始末は executor -> node -> context の順。
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        # shutdown は主スレッドしか呼ばないので1回だけになる。try_ を使うのは、
        # rclpy.init の直後に失敗した場合なども同じ経路で抜けるため。
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
