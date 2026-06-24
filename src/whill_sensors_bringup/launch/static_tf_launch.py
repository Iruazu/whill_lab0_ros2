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
        # 9 軸 USB IMU)。base_link (後輪車軸中点・地面高さ) を基準とする実測値
        # (2026-06-24、Issue #61):
        #   x = +0.38 m  (後輪車軸線から前方 38 cm)
        #   y = -0.03 m  (車体中心から右 3 cm)
        #   z = +0.47 m  (地面から 47 cm。後輪ハブ高 17 cm + ハブから IMU まで
        #                 鉛直 30 cm、WHILL CR2 後輪 直径 ~34 cm と整合)
        #
        # 姿勢は base_link と axis-aligned (REP-103, x=前, y=左, z=上)。
        # IMU 本体の x 軸が車椅子の前進方向、z 軸が上を向いていることを目視確認済
        # (Issue #61)。ロール/ピッチの微小ずれは M4R-3 の EKF バイアス推定が吸収。
        #
        # M4R-2 (#36) で設定された placeholder (0.20, 0.00, 0.50) からの補正:
        # x +18 cm、y -3 cm、z -3 cm。±5 cm の見積もり想定を超えていたため
        # 本番マップ録画前に実測値で置換した (詳細は m3-extrinsics-from-noetic.md)。
        _static_tf('static_tf_imu',
                   0.38, -0.03, 0.47, 0.0, 0.0, 0.0,
                   'base_link', 'imu_link'),

        # base_link -> velodyne
        # noetic 引き継ぎの LiDAR<->IMU 外部パラメータを base_link に合成。
        # 並進: imu_link (base_link 系で 0.38, -0.03, 0.47) に、noetic
        # `extrinsic_T` (IMU 系で表した LiDAR 原点 = 0.104136, 0.411548,
        # 0.323704) を足す。IMU と base_link が axis-aligned 前提で単純加算
        # で済む。LiDAR は IMU と物理的に同じ機構に共締めされており動いていない
        # ため、IMU 再測定 (#61) では LiDAR↔IMU の相対は変えず、IMU の base_link
        # 系移動分だけスライドさせる。LiDAR は椅子の左 (+y, +0.382 m) かつ
        # IMU より上 (+z, +0.324 m)。
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
                   0.484136, 0.381548, 0.793704,
                   -0.035342, 0.156983, -0.005527,
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
