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

M4R-2: replaced the original identity placeholders with measurement-
based extrinsics. `base_link` is provisionally defined as the rear-axle
midpoint projected to the ground plane (x = rear axle, y = chassis
centerline, z = ground). The derivation and the values below — together
with the noetic-inherited LiDAR<->IMU extrinsic that anchors them — are
documented in `docs/ja/m3-extrinsics-from-noetic.md`. Re-check this
definition against the URDF / Nav2 footprint / saved-map origin when
M5-R wires up the map pipeline; the chassis mount has not changed since
the noetic era so the noetic extrinsic is reused as-is.
"""

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
        # base_link -> imu_link
        # IMU は座面クッション下、椅子の左右中央付近にマウントされている (RT
        # 9 軸 USB IMU)。base_link (後輪車軸中点・地面高さ) を基準にすると、
        # 座面下中央は概ね (x, y, z) = (0.20, 0.00, 0.50) m。URDF の seat box
        # 中央 (base_floor 系で x = -0.10、絶対系で base_link x = +0.20) と、
        # 標準的車椅子座面高 0.45-0.55 m から見積もった概算値。
        #
        # 姿勢は base_link と axis-aligned (REP-103, x=前, y=左, z=上) を仮定。
        # RT 9 軸 IMU はケースを水平にマウントしており、ロール/ピッチのずれは
        # M4R-3 の EKF バイアス推定で吸収可能な範囲。
        #
        # TODO(M4R-3 以降): 目視・メジャー実測で z をピン留めする (±5 cm 程度
        # の不確実性。EKF が吸収するが、地図補正導入時に小さい方が安全)。
        _static_tf('static_tf_imu',
                   0.20, 0.00, 0.50, 0.0, 0.0, 0.0,
                   'base_link', 'imu_link'),

        # base_link -> velodyne
        # noetic 引き継ぎの LiDAR<->IMU 外部パラメータを base_link に合成。
        # 並進: imu_link (base_link 系で 0.20, 0.00, 0.50) に、noetic
        # `extrinsic_T` (IMU 系で表した LiDAR 原点 = 0.104136, 0.411548,
        # 0.323704) を足す。IMU と base_link が axis-aligned 前提で単純加算
        # で済む。LiDAR は椅子の左 (+y, +0.412 m) かつ IMU より上 (+z,
        # +0.324 m)。session-2026-05-08.md の「~30 cm 下」表記は誤記であり、
        # +z=up 規約で +0.324 は LiDAR が IMU の上にあることを意味する。
        #
        # 回転: noetic `extrinsic_R` (3x3, LiDAR -> IMU) を Rz(yaw)*Ry(pitch)
        # *Rx(roll) 固定軸表現に分解した値を使う。分解式と数値の検算は
        # docs/ja/m3-extrinsics-from-noetic.md の追補節を参照。
        #   roll  = -0.035342 rad  (-2.025 deg)
        #   pitch = +0.156983 rad  (+8.995 deg)
        #   yaw   = -0.005527 rad  (-0.317 deg)
        # IMU と base_link が axis-aligned 前提のため、imu_R_lidar をそのまま
        # base_link -> velodyne の回転として使える。
        _static_tf('static_tf_velodyne',
                   0.304136, 0.411548, 0.823704,
                   -0.035342, 0.156983, -0.005527,
                   'base_link', 'velodyne'),

        # base_link -> camera_link
        # RealSense D435 は LiDAR にリジッドにサポートフレームで共締めされて
        # いる。本 Issue (M4R-2) は M4R-3 EKF 配線に必要な「ゼロでない概算」
        # を入れる段階に留め、本格的な extrinsic 再キャリブは M5-R で行う。
        #
        # 簡易測定: LiDAR よりわずかに前方 (D435 は前向き)、y/z は LiDAR と
        # 同等。base_link 系で (0.36, 0.412, 0.82) を仮置き。姿勢は LiDAR と
        # 同方位 (RPY=0) で簡易化。
        #
        # TODO(M5-R): カメラ extrinsic を target-based キャリブ
        # (chessboard / AprilTag) で再校正し、回転成分も入れる。
        _static_tf('static_tf_camera',
                   0.36, 0.412, 0.82, 0.0, 0.0, 0.0,
                   'base_link', 'camera_link'),
    ])
