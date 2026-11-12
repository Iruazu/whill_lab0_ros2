"""Static TF chain rooted at base_link.

All three sensor frames are wired as direct children of `base_link`:

    base_link
    ├── imu_link
    ├── velodyne
    └── camera_link
        ├── camera_depth_frame -> camera_depth_optical_frame
        └── camera_color_frame -> camera_color_optical_frame

The RealSense subtree below `camera_link` is published by the
`realsense2_camera` driver itself; only the three top-level
`base_link -> sensor_*` transforms are emitted here.

`base_link` is the rear-axle midpoint projected to the ground plane
(x = rear axle, y = chassis centerline, z = ground). Every value below is
owned by `docs/calibration-ledger.md` (value, date, method, uncertainty,
status); change the ledger and this file together. The comments here only
name the ledger section and say why a number looks the way it does; the
derivations live in the ledger.

The three transforms are measured independently against `base_link`.
Until #145 the velodyne TF was the IMU TF plus the noetic LiDAR<->IMU
extrinsic; it no longer depends on the IMU TF.
"""

import math

from launch import LaunchDescription
from launch_ros.actions import Node


def _static_tf(name, x, y, z, roll, pitch, yaw, parent, child):
    return Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name=name,
        arguments=[
            '--x', str(x), '--y', str(y), '--z', str(z),
            '--roll', str(roll), '--pitch', str(pitch), '--yaw', str(yaw),
            '--frame-id', parent,
            '--child-frame-id', child,
        ],
    )


def generate_launch_description():
    return LaunchDescription([
        # base_link -> imu_link (台帳 §4.1)
        # 並進は PR #61 の巻尺、姿勢は 2026-07-10 朝の静止 gravity 逆算。
        # 2026-07-08 に IMU を axis-aligned に付け直したので yaw = 0。
        # roll / pitch が大きいのは座面とマウント溝の傾きによる。溝への
        # 再固定で ±2-3° 動くため、走行前後の静止確認 (台帳 §0) で再現性を見る。
        _static_tf('static_tf_imu',
                   0.38, -0.03, 0.47,
                   -0.0713, -0.1415, 0.0,   # roll=-4.09°, pitch=-8.11°, yaw=0 (2026-07-10 morning)
                   'base_link', 'imu_link'),

        # base_link -> velodyne (台帳 §2.1、#145)
        # x, y は巻尺 2 回 (旧 URDF と一致)。z / roll / pitch は Hi-Res の
        # 校正表 (sensors_launch の velodyne_calibration、#144) で投影した点群
        # の床面フィット。yaw は床面からは観測できないので、noetic
        # extrinsic_R 由来の -0.317° を残す。
        #   roll = +0.24°、pitch = +0.97°、yaw = -0.317°
        # 旧値 (0.484, 0.382, 0.794, pitch +9.0°) は IMU TF に noetic の
        # LiDAR<->IMU extrinsic を足したもの。+9° は「IMU が約 -8° 傾き、
        # LiDAR はほぼ水平」という両者の相対姿勢で、IMU を base_link と
        # axis-aligned とみなしてそれを LiDAR の対 base_link 姿勢に流用した
        # のが誤り (docs/imu-coordinate-audit.md §4.4 シナリオ B)。
        # 値を変えると base_link 系の点群が動き、旧地図と NDT が合わなく
        # なるので、地図の作り直し (#153) と同時にマージする。
        _static_tf('static_tf_velodyne',
                   0.42, 0.36, 0.777,
                   math.radians(0.24), math.radians(0.97),
                   math.radians(-0.317),
                   'base_link', 'velodyne'),

        # base_link -> camera_link
        # RealSense D435 は LiDAR にリジッドにサポートフレームで共締めされて
        # いる。本 Issue (M4R-2) は M4R-3 EKF 配線に必要な「ゼロでない概算」
        # を入れる段階に留め、本格的な extrinsic 再キャリブは M6-R で行う
        # (旧 M5-R で予定だった target-based キャリブは M6-R に繰り越し)。
        #
        # 簡易測定: LiDAR よりわずかに前方 (D435 は前向き)、y/z は LiDAR と
        # 同等。Issue #61 (IMU 実測) で IMU と LiDAR が共締めユニットごと
        # (+0.18, -0.03, -0.03) m スライドしたため、camera_link も同じ
        # 相対位置を維持するためスライド: (0.36, 0.412, 0.82) ->
        # (0.54, 0.382, 0.79)。姿勢は LiDAR と同方位 (RPY=0) で簡易化。
        #
        # TODO(M6-R): カメラ extrinsic を target-based キャリブ
        # (chessboard / AprilTag) で再校正し、回転成分も入れる。
        _static_tf('static_tf_camera',
                   0.54, 0.382, 0.79, 0.0, 0.0, 0.0,
                   'base_link', 'camera_link'),
    ])
