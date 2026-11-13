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
        # x, y は巻尺 2 回。z / roll / pitch は Hi-Res の校正表
        # (sensors_launch の velodyne_calibration、#144) で投影した点群の
        # 床面フィット (2026-10-28 計画書の値)。2026-11-12 の bag 再フィット
        # (z 0.771、roll -0.2°、pitch 0.3〜0.4°) とは合わなかったが、
        # 2026-11-13 の実機静止確認 (無人、同じ場所で 180° 回した 2 本の
        # 平均で床の勾配を打ち消し) が 0.78 m / +0.09° / +0.87° で計画書の
        # 値と 0.15° 以内に一致したので、計画書の値で確定 (台帳 §2.1)。
        # 人が座ると車体が沈んで床に対する pitch が約 -0.75° 変わるが、
        # それは base_link の姿勢の変化で、この TF は変えない。
        # yaw は未測定: 床面からは観測できず、-0.317° は noetic の LiDAR 対
        # IMU の yaw の流用。IMU 自体を 2026-07-08 に 90° 付け直して yaw 0 を
        # 目視で決めているので、base_link に対する測定値ではない (0 と同等)。
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

        # base_link -> camera_link (2026-10-28 実測、#146)
        # 値の正本: docs/calibration-ledger.md の camera_link 節 (#151 後は §3)。
        # D435 は VLP-16 の真下に 3D プリントホルダで共締め、水平・前向き。
        # x, y は LiDAR の巻尺値 (0.42, 0.36) にホルダとデータシートの寸法を
        # 足したもので、LiDAR の x, y が変われば一緒に動く。y の +17.5 mm は、
        # realsense2_camera が camera_link を筐体中心ではなく左 IR (深度) の
        # 原点に置くため。z はレンズ中心の巻尺。回転は水平の申告どおり 0
        # (床面フィットで ~1° 上向きの疑いあり、#149 で再評価)。
        # 導出・誤差・旧値 (0.54, 0.382, 0.79) は台帳。
        _static_tf('static_tf_camera',
                   0.47, 0.378, 0.73, 0.0, 0.0, 0.0,
                   'base_link', 'camera_link'),
    ])
