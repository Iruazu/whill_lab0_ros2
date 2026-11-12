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
documented in `docs/m3-extrinsics-from-noetic.md`. Re-check this
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
        # 9 軸 USB IMU)。base_link (後輪車軸中点・地面高さ) を基準とする実測値:
        #   x = +0.38 m  (後輪車軸線から前方 38 cm、PR #61)
        #   y = -0.03 m  (車体中心から右 3 cm、PR #61)
        #   z = +0.47 m  (地面から 47 cm、PR #61)
        #
        # 姿勢 (2026-07-08 セッションでの経過):
        #  1. 元 PR #61 は「IMU の +x が前進方向」と記載していたが、目視確認で
        #     実際は IMU の +y が前進方向 (90° yaw ズレ) であることが判明。
        #  2. GLIM の T_lidar_imu を 90° yaw 込みで再計算して config に反映
        #     したところ、GLIM が 6 秒で発散した (2 回試行、詳細は
        #     scripts/m5r3_run_glim.sh のコメント)。GLIM/imu_sign_corrector
        #     が axis-aligned 想定で組まれている疑い。
        #  3. マジックテープ固定を活かし IMU を物理的に反時計回りに 90° 回転
        #     させて axis-aligned に付け直し。これで noetic T_lidar_imu が
        #     本来の想定条件で機能し、GLIM の drift 大幅改善見込み。
        #
        # 再マウント後の姿勢:
        #   IMU の +x 軸 = base_link +x (前進方向)
        #   IMU の +y 軸 = base_link +y (左)
        #   IMU の +z 軸 = base_link +z (上)
        # つまり yaw = 0。ただし座面 (-5°) + マウント溝の追加傾きは物理的に
        # 残るため、IMU 全体は「後方低・前方高」に約 8° 傾いたまま
        # (Gemini + Claude 写真解析で 7.5-8°)。IMU +x が前方に向いた今、
        # この傾きは純粋に pitch = -8° として表される (tf2 の R_y(-8°) が
        # +x を斜め上に向ける、gravity 検算で確認)。
        #
        # 2026-07-09 audit (docs/imu-coordinate-audit.md §3): 2026-07-08
        # campus-outer bag 冒頭 10 秒 (WHILL 停止) の /imu/data_rep145 を
        # 1000 サンプル取得し gravity 逆算した結果:
        #   pitch 実測 = -7.66°  (元の -8° 設定と 0.34° 差、実質一致)
        #   roll  実測 = -5.77°  (元の 0° 設定と 5.77° 差 → audit で判明)
        # roll の未検出は、マウント溝が「後方低・前方高」だけでなく「右側低・
        # 左側高」も併せ持っていたことを示す (溝が横方向にも斜めに切ってある)。
        #
        # 2026-07-09 10:41-10:52 現地再測定: GRIL-Calib 校正 bag 撮影直前に
        # IMU 目視 → マジックテープに微スライドが疑われ、手押しで再固定を
        # 2 回試行。各固定で pitch/roll が ± 2-3° 動くことが判明 (溝の
        # 再現性が低い)。ただし 30 秒静置テスト (scratchpad/
        # imu_stability_check.py) で「触らなければ 0.03° 未満の drift」を
        # 確認 → 5 分録画中は動かない見込み。
        #
        # 2026-07-10 朝の再測定 (走行前 sanity check):
        # 走行日を跨いだ状態で scratchpad/imu_live_check.py を 3 連続実行。
        # 各軸ばらつき 0.003 m/s² 未満で完全に安定した実測平均:
        #   ax = +1.397  ay = -0.699  az = +9.782
        # frame_audit.py で逆算 (roundtrip Δ=0 で厳密):
        #   pitch = -8.11°  (-0.1415 rad)
        #   roll  = -4.09°  (-0.0713 rad)
        # 昨日値 (-3.52° / -7.76°) との差は roll -0.57°、pitch -0.35° で
        # 溝の ± 2-3° 再固定バラツキ範囲。物理再固定はしていない (触らずに
        # 一晩置いただけ) が、締結の応力緩和で微沈み込みしたものと解釈。
        # 走行はこの新値の commit と紐づく。
        _static_tf('static_tf_imu',
                   0.38, -0.03, 0.47,
                   -0.0713, -0.1415, 0.0,   # roll=-4.09°, pitch=-8.11°, yaw=0 (2026-07-10 morning)
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
        # docs/m3-extrinsics-from-noetic.md の追補節を参照。
        #   roll  = -0.035342 rad  (-2.025 deg)
        #   pitch = +0.156983 rad  (+8.995 deg)
        #   yaw   = -0.005527 rad  (-0.317 deg)
        # IMU と base_link が axis-aligned 前提のため、imu_R_lidar をそのまま
        # base_link -> velodyne の回転として使える。
        _static_tf('static_tf_velodyne',
                   0.484136, 0.381548, 0.793704,
                   -0.035342, 0.156983, -0.005527,
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
