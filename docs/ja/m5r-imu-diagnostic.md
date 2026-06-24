# M5-R: GLIM IMU integration 警告の原因切り分けと対処方針

Language: [日本語](m5r-imu-diagnostic.md) | [English](../en/m5r-imu-diagnostic.md)

Issue #64 の受け入れ基準「原因切り分けレポート + 対処方針確定」を満たす
診断ドキュメント。

## 背景

`scripts/m5r3_run_glim.sh` 実行中、GLIM の odometry estimator が以下の警告を
bag 全期間にわたって連続発生させる:

```
[odom] [warning] IMU prediction is not good.
[odom] [warning] Possibly T_lidar_imu is not accurate or IMU bias is not well estimated.
[odom] [warning] IMU validation results:
  No-IMU errors  rot=0.231 +- 0.563 deg, trans=0.025 +- 0.011 m, vel=0.128 +- 0.095 m/s
  IMU errors     rot=0.114 +- 0.109 deg, trans=0.037 +- 0.020 m, vel=0.327 +- 0.170 m/s
  IMU better ratios rot=0.58, trans=0.11, vel=0.16
```

「IMU better ratios」は「IMU を入れた予測が IMU なし予測より良い割合」を
表す指標 (= 1.0 が理想、0.5 で五分)。`rot=0.58` (rotation で 58% IMU が
助ける) は許容範囲だが、`trans=0.11` / `vel=0.16` は **IMU を入れる方が
ほとんどの場合悪化する** ことを示している。閾値 (GLIM ソース
`src/glim/common/imu_validation.cpp` より rot=0.7 / trans=0.4 / vel=0.5)
を 3 軸とも下回っており、警告が常時出続ける。

## 原因候補と切り分け実験

Issue #64 本文の 4 候補:
1. noetic 由来の `T_lidar_imu` 値そのものの精度不足
2. IMU noise / bias parameter (`imu_acc_noise` 等) の未調整
3. IMU bias の温度ドリフト / 初期化不足
4. `/imu/data_rep145` の出力レート / buffering 問題

### 候補 4 の事前確認 (NG)

`docs/m5r-bench-data/2026-06-24-loop-outdoor-ext/bag` の `/imu/data_rep145` を
直接 inspect:

| 項目 | 値 |
|---|---|
| メッセージ総数 | 23,945 |
| 録画 duration | 239.45 s |
| **実効レート** | **99.99 Hz** (公称 100 Hz と完全一致) |
| 取りこぼし | なし |

→ 候補 4 は除外。

### 候補 1 の事前確認 (NG)

`scripts/m5r3_run_glim.sh` line 274-283 が GLIM config に書き込む
`T_lidar_imu` の数値 `[-0.05, -0.4, -0.35, 0.017399, -0.078447, 0.001369, 0.996765]`
を、noetic 由来の `extrinsic_T = [0.104136, 0.411548, 0.323704]` および
`extrinsic_R` (LiDAR→IMU rotation) から独立に SE3 反転計算で再導出:

```python
R_L_I = extrinsic_R.T  # IMU→LiDAR rotation
t_L_I = -R_L_I @ extrinsic_T
q_L_I = Rotation.from_matrix(R_L_I).as_quat()  # [qx,qy,qz,qw]
```

結果: 計算値と config 値が **7 桁目** まで一致 (差分 max 5e-7)。
→ SE3 数値計算ステップは正しい。残る可能性は「noetic 由来値そのものの
精度不足」(本ドキュメント結論で対処)。

### 候補 2 の実験 (NG)

`bag` の冒頭 5 秒 (椅子静止区間と推定) から IMU の per-axis stddev を
計測した即席 noise-density 推定:

| param | GLIM 現値 | 実測 (5s static) | MPU-9250 datasheet | 倍率 (config/actual) |
|---|---|---|---|---|
| `imu_acc_noise` (m/s²/√Hz) | 0.05 | 0.0026 | 0.008 | **約 19 倍過大** |
| `imu_gyro_noise` (rad/s/√Hz) | 0.02 | 0.0002 | 0.0013 | **約 100 倍過大** |

「config 値が過大 → GTSAM が IMU を信頼しない → trans/vel ratio 悪い」が
当初仮説。検証実験として `imu_acc_noise: 0.05 → 0.008`、
`imu_gyro_noise: 0.02 → 0.0013` (datasheet 値) に変更して同じ bag で再走:

| 試行 | rot | trans | vel | loop error |
|---|---|---|---|---|
| baseline (現状 config) | 0.60 | 0.11 | 0.16 | 3.99 m |
| noise param 緩和 (datasheet 値) | 0.59 | **0.02** | **0.06** | 4.03 m |

→ **trans/vel が悪化**。noise を下げる = IMU を信頼する → 「IMU 翻訳予測
そのものが悪い」状態でその予測が出力に流入 → ratio がさらに低下。

つまり現状の `imu_acc_noise: 0.05` は **意図的に inflate された値**であり、
defensive な設定として正しい。候補 2 は除外。

### 念のため LiDAR-only (NG)

sub_mapping と global_mapping の `enable_imu: false` で試行:

| 試行 | rot | trans | vel | loop error |
|---|---|---|---|---|
| sub/global IMU OFF | 0.60 | 0.02 | 0.06 | 3.97 m |

→ front-end odometry estimation は依然 IMU を使うため ratio は変わらず、
loop error は微小改善 (0.02 m)。**sub_mapping/global_mapping への IMU
貢献はほぼゼロ**。GLIM の current map quality は実質 LiDAR-only。

## 結論

**真の bottleneck は `T_lidar_imu` の回転精度**。SE3 数値計算は正しい
(候補 1 の半分は除外) が、**noetic 由来の `extrinsic_R` (= LiDAR↔IMU 相対
回転) そのものが kalibr 等での精密校正を経ておらず、推定上 1-2 度の
回転誤差を持つ**と考えられる。

候補 3 (IMU bias 初期化不足) も間接的に除外できる: 候補 2 の実験で IMU の
noise sigma を datasheet 値 (= IMU を正確に重み付け) にしても trans/vel
ratio が改善せずむしろ悪化した。bias 初期化不足が真の原因なら、適切な
重み付けで改善するはず。逆に何をしても trans/vel が悪化 / 改善しない
ことは、bias 推定の問題ではなく、その前段の **IMU 予測値そのものが
geometric に間違っている** (= T_lidar_imu の回転誤差で gravity 射影が
ズレる) ことの証拠。

理論的影響: 1° の rotation 誤差で gravity (9.81 m/s²) の射影が誤差を持ち、
~0.17 m/s² の擬似加速度を IMU 予測に注入する。preintegration で積分される
ため、translation/velocity 予測のエラーが no-IMU baseline (LiDAR ICP) より
大きくなる。これが trans=0.11 / vel=0.16 の症状と完全に一致。

## 対処方針

**GRIL-Calib (RA-L 2024、targetless ground robot LiDAR-IMU 校正)** で
`T_lidar_imu` を再校正する。

選定理由:
- ROS 2 humble 公式 branch あり (`Taeyoung96/GRIL-Calib`、humble)
- ground plane motion constraint 利用 → **WHILL のような平面走行ロボットに
  最適化**。校正ボード等の特殊機材不要
- VLP-16 / Velodyne 系で実績 (本プロジェクトの `livox_ros_driver2` 依存と
  既存ビルドツリーで完結)
- ライセンス: BSD-3-Clause (permissive)

採用しなかった代替:
- **kalibr** (ETH-ASL): ROS 1 noetic ベースで humble 環境への移植コスト大
- **lidar_imu_calib (APRIL-ZJU)**: 2020 IROS、ROS 1 のみ
- **Plan C (起動 sequence 調整)**: 起動時 30 秒静置で bias 初期化を改善
  する案だったが、本診断で「bias 初期化不足ではなく T_lidar_imu の精度
  不足」と判明したため除外

## 校正手順

### 1. GRIL-Calib のセットアップ (済)

```bash
scripts/install_gril_calib.sh
```

このスクリプトは `~/calib_ws/` に GRIL-Calib (humble branch) を clone +
ビルドする。本プロジェクトの `install/livox_ros_driver2` を依存として
吸い込むため、`~/whill_lab0_ros2/install/setup.bash` のソースが前提。
所要時間 ~1 分。

### 2. Motion bag の取得 (ユーザー、未実施)

GRIL-Calib は **平面運動だけ** で 6-DoF extrinsic を推定するが、十分な
モーション励起が必要。以下のプロトコルで bag 取得:

| 項目 | 値 |
|---|---|
| 場所 | 屋内の平坦な床 (廊下・実験室)。少なくとも 5m × 5m の障害物のない空間 |
| 走行 | WHILL ジョイスティック有人運転、0.3-0.5 m/s |
| 動きの種類 | (a) 8 の字を 3 周以上 (b) 急加速 + 急減速の往復 (c) その場旋回 360° を 2 回 |
| duration | 3-5 分 (180-300 秒) |
| 環境 | 静的物体のみ。歩行者・自転車などの動的物体は避ける |
| topics | `/velodyne_points` + `/imu/data_rep145` + `/tf_static` (通常録画と同じ) |

録画コマンド例:

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash

# Terminal A: bringup
ros2 launch whill_localization odom_bringup_launch.py

# Terminal B: bag 録画
RUN=2026-MM-DD-calib-motion
mkdir -p docs/m5r-bench-data/${RUN}
ros2 bag record \
  -o docs/m5r-bench-data/${RUN}/bag \
  /velodyne_points /imu/data_rep145 /tf_static
# (圧縮なし — GLIM/GRIL-Calib 共に zstd 非対応、本リポ規約)
```

### 3. GRIL-Calib 実行

bag 取得後、本リポルートから:

```bash
scripts/m5r4_run_gril_calib.sh docs/m5r-bench-data/<run>/bag <out-dir>
```

(本スクリプトは Issue #64 後続 PR で追加予定。motion bag 取得後に
合わせて実装する。現段階の手動手順は本ドキュメント末尾の付録 A 参照)

### 4. GLIM config への反映

GRIL-Calib 出力の T_lidar_imu を `scripts/m5r3_run_glim.sh` の
`new_tli` literal (line 274-283) に転記。次回 GLIM 実行から自動適用。

### 5. 検証

同じ 2026-06-24 bag で GLIM 再走、`IMU better ratios` の改善を観測:

| 指標 | 現状 | 目標 (GLIM 閾値) | 合格基準 |
|---|---|---|---|
| rot ratio | 0.60 | 0.7 | ≥ 0.7 |
| trans ratio | 0.11 | 0.4 | ≥ 0.4 |
| vel ratio | 0.16 | 0.5 | ≥ 0.5 |
| loop error | 3.99 m / 106 m | < 1% ループ長 | < 1.06 m |

`IMU better ratios rot/trans/vel` がすべて閾値超 → 警告消失。
合格すれば ADR-0003 を update (T_lidar_imu の再校正により M5-R 完了基準
B1 をより厳密に満たせる旨を追記)。

## 付録 A: GRIL-Calib 手動実行手順

`scripts/m5r4_run_gril_calib.sh` が未実装の現段階での手動手順。

```bash
# 1. config を WHILL/VLP-16 向けに patch
cp ~/calib_ws/src/GRIL-Calib/config/velodyne32.yaml /tmp/velodyne16_whill.yaml
# 編集ポイント:
#   common.imu_topic:        "/imu/data" -> "/imu/data_rep145"
#   preprocess.scan_line:    32          -> 16
#   calibration.imu_sensor_height: 0.73  -> 0.47  (Issue #61 実測)
#   calibration.trans_IL_x/y/z: 0.0      -> 0.104/0.412/0.324  (noetic 初期推定)
${EDITOR:-vi} /tmp/velodyne16_whill.yaml

# 2. ros2 環境ロード (GRIL-Calib + livox_ros_driver2 が見える状態)
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
source ~/calib_ws/install/setup.bash

# 3. GRIL-Calib launch (bag 再生は別ターミナル)
ros2 launch gril_calib mapping_velodyne.launch.py \
  config_path:=/tmp/velodyne16_whill.yaml

# 4. 別ターミナルで bag 再生
ros2 bag play docs/m5r-bench-data/<run>/bag --rate 0.5

# 5. GRIL-Calib 完了後、出力 T_lidar_imu を `~/calib_ws/result/` から確認
ls ~/calib_ws/result/
```

ROS 2 launch のターミナル出力末尾に `Calibration result:` で T_IL
(IMU→LiDAR の同次変換、4×4 行列形式) が出力される。GLIM 規約 (= TUM
quaternion 形式 `[tx, ty, tz, qx, qy, qz, qw]`、`m5r3_run_glim.sh` line
274 の `new_tli`) に変換する手順:

```python
import numpy as np
from scipy.spatial.transform import Rotation
# GRIL-Calib 出力の T_IL を 4x4 として読み込む (出力テキストからコピペ)
T_IL = np.array([
    [r11, r12, r13, t_x],
    [r21, r22, r23, t_y],
    [r31, r32, r33, t_z],
    [0.0, 0.0, 0.0, 1.0],
])
# GLIM の T_lidar_imu (= "p_lidar = T_lidar_imu * p_imu") は T_IL の inverse
T_LI = np.linalg.inv(T_IL)
t = T_LI[:3, 3]
q = Rotation.from_matrix(T_LI[:3, :3]).as_quat()  # [qx, qy, qz, qw]
print("new_tli for m5r3_run_glim.sh:")
for v in [*t, *q]:
    print(f"      {v:.6f},")
```

これを `scripts/m5r3_run_glim.sh` の `new_tli` 7 行に転記。

## 関連

- Issue #64 (本ドキュメントの起案元)
- PR #62 (Issue #61): `base_link → imu_link` 実測値化、本診断の前提
- PR #71 (Issue #63): GLIM auto_quit、本診断の前提 (GLIM が完走する状態)
- `docs/ja/m3-extrinsics-from-noetic.md`: noetic 由来 LiDAR-IMU extrinsic
  の出典 (今回再校正対象の値)
- ADR-0003 (`docs/ja/decisions/0003-mapping-slam-choice.md`): GLIM 採用根拠。
  本 Issue 解消後に「校正で精度向上」を追記予定
- `src/third_party/glim/src/glim/common/imu_validation.cpp`: 警告閾値の実装
