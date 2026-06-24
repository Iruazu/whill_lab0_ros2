# noetic スタックから引き継ぐ LiDAR ↔ IMU 外部パラメータ

Language: [日本語](m3-extrinsics-from-noetic.md) | [English](../en/m3-extrinsics-from-noetic.md)

noetic 側の `whill_lab0` リポは校正済みの LiDAR-IMU 外部パラメータを `FAST_LIO/config/velodyne.yaml` に持っていた。M4 (humble 上の FAST-LIO) がゼロから再キャリブせずに既知の良好な設定から始められるよう、正確なポーズを下に記す。

## 出典

[`whill_lab0/FAST_LIO/config/velodyne.yaml`](https://github.com/Iruazu/whill_lab0/blob/main/FAST_LIO/config/velodyne.yaml) の `mapping:` セクション、`extrinsic_T` / `extrinsic_R` フィールド。

## 値

並進 `extrinsic_T` (IMU フレームで表した LiDAR 原点、メートル):

```
[ 0.104136, 0.411548, 0.323704 ]
```

回転 `extrinsic_R` (3×3、row-major、LiDAR → IMU):

```
[  0.987688,  0.000000,  0.156434,
  -0.005459,  0.999391,  0.034470,
  -0.156339, -0.034900,  0.987087 ]
```

これは Y 軸まわりにほぼ +9.0° の pitch (`-asin(R[2][0]) = -asin(-0.156339) ≈ +8.99°`) と、小さな ~2° の roll/yaw 成分の組み合わせ (RPY 分解の数値は本ファイル末尾の M4R-2 追補節参照)。

同 yaml の関連 FAST-LIO 入力:

- `lid_topic: /velodyne_points`
- `imu_topic: /imu/data_raw`
- `lidar_type: 2` (Velodyne)
- `scan_line: 16`, `scan_rate: 10` (VLP-16 の 10 Hz と一致)
- IMU ノイズ: `acc_cov: 0.1`, `gyr_cov: 0.1`, `b_acc_cov: 1e-4`, `b_gyr_cov: 1e-4`

## M4 での適用方法

FAST-LIO の ROS 2 fork を `whill_lab.repos` に追加したら、これらの値をそのまま humble 側の同等 config ファイルにコピーする。椅子で小さなループを走らせて mapping させ、ドリフトを確認することで検証する。引き継いだ外部パラメータが間違っている (例えば noetic 時代と humble 時代の間にセンサが物理的にマウントし直された) 場合は、LI-Init スタイルのキャリブレーションを再実行する。

センサマウントが noetic 時代から変わっていなければ、本キャリブはまだ有効なはず。

## base_link の物理定義 (M4R-2 追補)

M4R-2 (Issue #36) で `whill_sensors_bringup/launch/static_tf_launch.py` の 3 つの identity placeholder を実測ベースに置換するにあたり、`base_link` の物理位置を以下のとおり仮置きする。

- **位置**: 後輪車軸の左右中点を地面高さに射影した点
  - x = 0 が後輪車軸 (前向きを +x)
  - y = 0 が左右中点 (左を +y、REP-103)
  - z = 0 が地面 (上を +z)
- **姿勢**: REP-103 準拠 (x = 前, y = 左, z = 上)

### URDF (`whill_model_cr2.urdf`) との関係

`src/third_party/ros2_whill/whill_description/urdf/whill_model_cr2.urdf` には別概念の `base_link` (こちらは原点が床下シャシ前方寄り、後輪は `base_floor` 経由で `base_link x = 0, y = ±0.260, z = 0.015` にぶら下がる) が定義されているが、本 Issue で扱う **ナビ用 `base_link` は URDF 版とは独立に新規定義する**。URDF 改訂は本 Issue のスコープ外 (M5-R 以降で URDF を実車寸法に合わせて改訂する際に整合させる)。

### 仮置きである理由 / 再定義の条件

以下のいずれかが発生した時点で `base_link` の物理定義を再評価する。

1. **Nav2 footprint との整合**: M6-R で Nav2 を本格運用する際、footprint 多角形の原点は `base_link` 基準で定義する。前輪キャスタを含めた前方クリアランスを取りやすくするため、原点を前方にずらす判断が出る可能性がある。
2. **保存地図原点との整合**: M5-R で `docs/maps/<site>/` に保存する 2D 占有格子の原点座標は localization 起動時に `base_link` を介して解釈される。マッピング bag 開始時の車椅子姿勢が「後輪車軸を原点とする」前提と矛盾する場合 (例: 旧マップを引き継ぐ場合) は再定義する。
3. **URDF の改訂**: 上記の URDF 版 `base_link` と一致させる場合。

再定義した際は本節と `static_tf_launch.py` のコメントを同時に更新する。

## base_link 基準 extrinsic の算出 (M4R-2 追補)

`static_tf_launch.py` の 3 個の静的 transform の値を、上記 `base_link` 定義から算出した経緯を以下に残す。これらは Issue #36 受け入れ基準 1 (3 個とも非 identity で publish される) を満たすための値であり、`extrinsic_R` の RPY 分解以外は M4R-3 以降で精度を上げる余地がある。

### 3 個まとめ

| 親 → 子 | 並進 [m] | 回転 (RPY rad) | 由来 |
|---------|---------|---------------|------|
| `base_link → imu_link` | (0.38, -0.03, 0.47) | (0, 0, 0) | **実測** (2026-06-24, Issue #61) |
| `base_link → velodyne` | (0.484136, 0.381548, 0.793704) | (-0.035342, +0.156983, -0.005527) | **計算** (上記 imu + noetic 引き継ぎ `extrinsic_T`/`extrinsic_R`) |
| `base_link → camera_link` | (0.54, 0.382, 0.79) | (0, 0, 0) | **仮置き** (M6-R で target-based 校正)。Issue #61 で LiDAR/IMU の共締めユニットが平行移動したため同じ delta だけスライド済 |

### base_link → imu_link

| 項目 | 値 / 根拠 |
|------|----------|
| 何の値か | IMU (RT 9 軸 USB IMU = MPU-9250 + LPC1343F USB) の取付位置を `base_link` 基準で表したもの |
| 由来 | **実測** (Issue #61、2026-06-24)。IMU 本体は座面クッション下に水平マウント |
| x | **+0.38 m** — 後輪車軸 (左右タイヤ中心を結ぶ床上の直線) から前方 38 cm |
| y | **-0.03 m** — 車体中心から右 3 cm |
| z | **+0.47 m** — 地面から 47 cm。後輪ハブ高 17 cm (= タイヤ半径、WHILL CR2 後輪 ~34 cm 直径と整合) + ハブから IMU まで鉛直 30 cm |
| 不確実性 | 実測の手作業精度 (±1〜2 cm)。M4R-2 placeholder (±5 cm) より精度向上 |
| 姿勢 | (0, 0, 0)。IMU ケースの x 軸が車椅子前進方向、z 軸が上を向くマウント (REP-103 と一致)、目視確認済 (Issue #61) |
| 再評価のタイミング | (a) WHILL の物理改造 (シート交換、IMU マウント移設) (b) GLIM の「IMU prediction is not good」警告が解消しない場合の LiDAR↔IMU 精密校正 (kalibr 等。本値ではなく noetic 由来の `extrinsic_T`/`extrinsic_R` を再校正することになる) |

### M4R-2 placeholder からの変更 (Issue #61)

| 軸 | placeholder (M4R-2) | 実測 (M5-R prep) | 差分 |
|---|---|---|---|
| x | 0.20 m | **0.38 m** | +0.18 m |
| y | 0.00 m | **-0.03 m** | -0.03 m |
| z | 0.50 m | **0.47 m** | -0.03 m |

x の +18 cm はかなり大きく、M4R-2 で「±5 cm 程度の不確実性」と見積もった範囲を超えていた。本番マップ録画 (M5-R 完了直後) の前に実測値で置換することで、SLAM の IMU lever arm 計算と Nav2 footprint の精度を改善する。

### base_link → velodyne

並進と回転の両方を、上記 `imu_link` 値と noetic 引き継ぎ extrinsic から計算する。

**並進**: IMU と `base_link` が axis-aligned 前提のため、`base_link → imu_link` の並進に noetic `extrinsic_T` (IMU フレームで表した LiDAR 原点) を単純加算できる。

```
base_link → velodyne (translation)
  = base_link → imu_link (translation) + extrinsic_T
  = (0.38, -0.03, 0.47) + (0.104136, 0.411548, 0.323704)
  = (0.484136, 0.381548, 0.793704) [m]
```

物理解釈 (Issue #61 後):
- LiDAR は椅子の左 (+y, +0.382 m = IMU の左 0.412 m から車体中心の右 0.03 m を引いた値)
- LiDAR は IMU から上 (+z, +0.324 m。`extrinsic_T` は本文冒頭の通り「IMU フレームで表した LiDAR 原点」であり、IMU フレームは REP-103 で +z=up を採用する。したがって `extrinsic_T[2] = +0.324 > 0` は LiDAR が IMU の +z 方向 = 上方にあることを直接意味する。`session-2026-05-08.md:26` の「~30 cm 下」表記は符号を見落とした誤記、上方が正しい (= 座面上のセンサポール先端))
- LiDAR は IMU からわずかに前方 (+x, +0.104 m)
- Issue #61 で IMU が再測定により placeholder から (+0.18, -0.03, -0.03) m 動いたため、LiDAR も共締めの仮定で同じ delta だけスライド (相対位置 `extrinsic_T` は変えない)

**回転**: noetic `extrinsic_R` (3×3, LiDAR → IMU = `imu_R_lidar`) を `R = Rz(yaw) · Ry(pitch) · Rx(roll)` の固定軸表現に分解する。一般式:

```
R[2][0] = -sin(pitch)
R[2][1] = cos(pitch) · sin(roll)
R[2][2] = cos(pitch) · cos(roll)
R[1][0] = cos(pitch) · sin(yaw)
R[0][0] = cos(pitch) · cos(yaw)

⇒ pitch = -asin(R[2][0])
   roll  = atan2(R[2][1], R[2][2])
   yaw   = atan2(R[1][0], R[0][0])
```

本ファイル冒頭の `extrinsic_R` 数値を代入:

```
pitch = -asin(-0.156339)      = +0.156983 rad  (+8.9945 deg)
roll  = atan2(-0.034900, 0.987087) = -0.035342 rad  (-2.0249 deg)
yaw   = atan2(-0.005459, 0.987688) = -0.005527 rad  (-0.3167 deg)
```

検算: 上記 RPY から `R = Rz(yaw)·Ry(pitch)·Rx(roll)` を再構成すると元の `extrinsic_R` と最大 5.6e-7 で一致する。M4R-2 実装時に以下の Python スニペットで確認:

```python
import math
roll, pitch, yaw = -0.035342, +0.156983, -0.005527
cr, sr = math.cos(roll),  math.sin(roll)
cp, sp = math.cos(pitch), math.sin(pitch)
cy, sy = math.cos(yaw),   math.sin(yaw)
R = [
  [cy*cp,            cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
  [sy*cp,            sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
  [-sp,              cp*sr,            cp*cr           ],
]
# R を冒頭の extrinsic_R と element-wise diff すると max |Δ| < 6e-7
```

IMU と `base_link` が axis-aligned 前提のため、`imu_R_lidar` をそのまま `base_link → velodyne` の回転として使える。

### base_link → camera_link

| 項目 | 値 / 根拠 |
|------|----------|
| 何の値か | RealSense D435 の取付位置を `base_link` 基準で表したもの |
| 由来 | **仮置き** — D435 は LiDAR にリジッドに共締めされている (`static_tf_launch.py` 旧コメント、`whill_sensors_bringup/README.md` 参照)。M4R-2 は「ゼロでない概算で M4R-3 EKF 配線を通す」段階にとどめ、本格的な extrinsic 再キャリブは M6-R で行う (旧 M5-R 予定からスライド) |
| x | 0.54 m — LiDAR (x=0.484) より +0.05 m 程度前方 (D435 は前向き)。Issue #61 で +0.18 m スライド |
| y | 0.382 m — LiDAR と同じ左寄り (共締めのため)。Issue #61 で -0.03 m スライド |
| z | 0.79 m — LiDAR (z=0.794) と同等。Issue #61 で -0.03 m スライド |
| 姿勢 | (0, 0, 0) — 簡易化。実機ではカメラ筐体と LiDAR 筐体の取り付け角に数度のずれがある可能性が高いが、本値は visualization 用途と Nav2 footprint 計算の概算に十分 |
| 再校正のタイミング | M5-R で chessboard / AprilTag による target-based extrinsic キャリブを実施。それまでは本値で `view_frames` と RViz 表示が成立すれば良い |

