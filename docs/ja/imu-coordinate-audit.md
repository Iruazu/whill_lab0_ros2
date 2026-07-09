# IMU 座標系整合性 audit (2026-07-08)

- 日付: 2026-07-08
- 状態: 調査完了。コード変更なし。GRIL-Calib (2026-07-09 予定) に引き渡すための入力材料
- きっかけ: `2026-07-08-campus-outer` bag (47 分) の GLIM 出力で Z 軸ドリフト
  (dz = +7.5 m / 1770 m) が観測され、`fix_imu_bias: true` で追試したところ
  dy = -25 m と発散。「bias 発散が根本原因ではなく、extrinsic の姿勢誤差が
  bias に吸収されているだけ」の仮説を検証する
- 対象コミット: `b4468e5` (branch `chore/76-nvidia-suspend-uvm-reload`)

## 0. 3 行結論

1. `imu_sign_corrector` は REP-145 sign 反転のみ。座標変換ではない。
2. **静止時 gravity 実測から、IMU の物理姿勢は pitch = -7.66°、roll = -5.77°**。
   現行 static TF (`base_link → imu_link`) は pitch = -8° 相当だが **roll = 0**。
   → roll ~5.8° 不足。
3. **GLIM の `T_lidar_imu` は現行 TF chain と 8° pitch ずれている**。
   `base_link → velodyne` static TF は「IMU が base_link と axis-aligned」を
   前提に組まれたままで、2026-07-08 の IMU pitch = -8° 更新に追従していない。

## 1. 座標系フレーム図 (2026-07-08 時点)

```
world (重力 -Z)
   │
   ├─ base_link    後輪車軸中点・地面高さ (chassis level 前提)
   │   ├─ imu_link          (0.38, -0.03, 0.47)    RPY (0, -8°, 0)
   │   ├─ velodyne          (0.484, 0.382, 0.794)  RPY (-2.03°, +9.00°, -0.32°)
   │   └─ camera_link       (0.54, 0.382, 0.79)    RPY (0, 0, 0)
   │       └─ (RealSense subtree, realsense2_camera が publish)
   │
   └─ 各センサの物理軸 (再マウント後):
        IMU  : +x = base_link +x (前進),  +y = base_link +y (左),  +z = base_link +z (上)
        Velo : ROS 標準  +x = 前, +y = 左, +z = 上
        RS   : 光学慣習, camera_link は ROS 標準

出典: src/whill_sensors_bringup/launch/static_tf_launch.py:78-104
```

## 2. `imu_sign_corrector` の変換表

出典: `src/whill_sensors_bringup/whill_sensors_bringup/imu_sign_corrector.py`

`/imu/data_raw` → `/imu/data_rep145` は **linear_acceleration の 3 軸符号反転
のみ**。座標軸のリマップは行っていない。

| フィールド                     | 入力 (raw)   | 出力 (rep145)  | 説明                                                                 |
|-------------------------------|--------------|----------------|----------------------------------------------------------------------|
| `linear_acceleration.x`       | `a_x`        | **`-a_x`**     | REP-145 specific force 変換                                          |
| `linear_acceleration.y`       | `a_y`        | **`-a_y`**     | 同上                                                                 |
| `linear_acceleration.z`       | `a_z`        | **`-a_z`**     | 同上 (静止時 `raw z ≈ -g` → `rep145 z ≈ +g`)                         |
| `linear_acceleration_covariance` | そのまま  | そのまま       | 変更なし                                                             |
| `angular_velocity.{x,y,z}`    | そのまま     | そのまま       | MPU-9250 gyro は REP-103 準拠、ファーム反転なし                      |
| `angular_velocity_covariance` | そのまま     | そのまま       | 変更なし                                                             |
| `orientation`                 | そのまま     | そのまま       | ドライバはゼロ埋め (未使用)                                          |
| `orientation_covariance`      | そのまま     | そのまま       | 変更なし                                                             |
| `header`                      | そのまま     | そのまま       | frame_id/stamp 保持                                                  |

**含意**: sign_corrector は座標系ではなく「規約」の変換。IMU の物理軸方位
の議論は raw / rep145 のどちらでも同じ結論になる。

## 3. 静止時 gravity 実測 (bag `2026-07-08-campus-outer`)

WHILL 停止 (bag 冒頭 10 秒、bringup 起動直後、走行開始前) の `/imu/data_rep145`
を 1000 サンプル取得。

抽出スクリプト: `scratchpad/imu_static_sample.py` (sqlite3 + rclpy.serialization
で bag_0.db3 を直読)。

| 量                     | 実測値                        | 期待値 (axis-aligned)   | 判定                        |
|------------------------|-------------------------------|-------------------------|-----------------------------|
| `linear_accel.x`       | +1.322 m/s² (std 0.021)       | 0                       | **NG (+1.32 の常時オフセット)** |
| `linear_accel.y`       | -0.989 m/s² (std 0.025)       | 0                       | **NG (-0.99 の常時オフセット)** |
| `linear_accel.z`       | +9.785 m/s² (std 0.027)       | +9.807 (REP-145 の +g)  | OK (誤差 0.2%)              |
| \|accel\|              | 9.923 m/s²                    | 9.807                   | 誤差 +1.2% (弱いスケール誤差) |
| `angular_vel.x`        | -0.01904 rad/s                | 0                       | **NG (バイアス -1.09 °/s)** |
| `angular_vel.y`        | +0.00058 rad/s                | 0                       | OK                          |
| `angular_vel.z`        | +0.00125 rad/s                | 0                       | OK                          |

### 3.1 姿勢の逆算

`f_spec` (静止時) = -`g_imu` = `R_base_imu^T * (0, 0, +g)`。RPY 分解
(yaw = 0 前提、`R = R_z * R_y * R_x`) で解くと:

```
world +Z, IMU 系で = (+0.1332, -0.0997, +0.9861)
  → pitch = arcsin(-0.1332) = -7.66°
  → roll  = arcsin(-0.0997 / cos pitch) = -5.77°
```

計算スクリプト: `scratchpad/frame_audit.py`。

### 3.2 TF 記述との差分

| 軸    | 実測      | static TF (`static_tf_imu`)  | 差分       | 判定 |
|-------|-----------|------------------------------|------------|------|
| pitch | **-7.66°** | -8.00° (`-0.1396 rad`)      | +0.34°     | 十分一致 (± 0.5°) |
| roll  | **-5.77°** | **0°**                       | **-5.77°** | **矛盾** |

**結論**: `static_tf_launch.py:78` の `base_link → imu_link` は roll 成分を
落としている。物理的に IMU はマウント溝で **後方低・前方高** (pitch -8°) の
うえに **右側低・左側高 (roll -5.8°)** の傾きがある。

なお static_tf_launch.py:75 のコメントに「未確認: pitch の符号は光学的推測
に依存。... 実測して確認する。」とあり、まさに実測待ちの状態だった。今回
実測により pitch は正しく (-8° 実測 vs TF -8°)、しかし別の未検出な roll
成分 (-5.8°) が判明した。

## 4. `T_lidar_imu` の整合性

出典: `docs/m5r-bench-data/2026-07-08-campus-outer/glim-out-fixbias/config/config_sensors.json`

```json
"T_lidar_imu": [-0.05, -0.4, -0.35, 0.017399, -0.078447, 0.001369, 0.996765]
```

GLIM の convention は `p_lidar = T_lidar_imu * p_imu` (TUM [x,y,z,qx,qy,qz,qw])。

### 4.1 並進成分

| 由来                                    | 値 (m, LiDAR 系での IMU 原点)          |
|-----------------------------------------|----------------------------------------|
| static TF chain (`base->velodyne`^-1 * `base->imu`) | (-0.0500, -0.4000, -0.3500)  |
| GLIM 設定値                              | (-0.0500, -0.4000, -0.3500)            |
| **差分**                                | **< 1 µm (完全一致)**                  |

**判定**: 並進は矛盾なし。

### 4.2 回転成分

Static TF chain から導出:
`R_lidar_imu = R_base_lidar^T · R_base_imu = R_y(-9°)·R_x(2°) · R_y(-8°)`

| 由来                                    | RPY (roll, pitch, yaw) [deg]            |
|-----------------------------------------|-----------------------------------------|
| **static TF chain が示す期待値**         | **(+2.07, -17.00, -0.29)**              |
| **GLIM 設定値** (quaternion → RPY)      | **(+2.00,  -9.00,   0.00)**             |
| 差分 (TF - GLIM)                         | **(+0.07,  -8.00,  -0.29)**             |

**判定**: **pitch が 8° ずれている**。

### 4.3 由来の解剖

`static_tf_launch.py:87-100` のコメントより:
- `base_link → velodyne` の RPY は「**IMU と base_link が axis-aligned 前提で**、
  noetic の `imu_R_lidar` (RPY -2°, +9°, -0.3°) をそのまま流用」して設定された。
- ところが同じファイル :78 で `base_link → imu_link` は **後に** pitch = -8° に
  更新された (2026-07-08 の物理再マウント記録)。
- しかし `base_link → velodyne` はそのまま。
- 結果: 「axis-aligned 前提」が成立しなくなったのに、その前提で組んだ数値が
  そのまま残る。TF chain 全体としては LiDAR と IMU が **互いに 17° pitch
  傾いた** ことになる (物理的に一つのマウントに共締めされているとする本
  リポの前提と矛盾)。

GLIM の `T_lidar_imu` は noetic 由来の imu-lidar 相対姿勢 (RPY ~(+2°, -9°, 0°))
そのものを保持しているだけで、**GLIM だけを見れば内部整合**している。TF chain
との 8° 差分は **`base_link → velodyne` static TF 側にある**。

### 4.4 起こりうる 3 通りのシナリオ

| # | 仮説                                       | 修正対象                             | 検証方法                                             |
|---|--------------------------------------------|--------------------------------------|------------------------------------------------------|
| A | LiDAR も IMU と同じく物理的に -8° pitch    | `base_link → velodyne` を pitch=+1° に | LiDAR 点群で地面平面を fit し tilt を測る            |
| B | LiDAR は本当に +9° pitch (別マウント)      | GLIM の `T_lidar_imu` を pitch=-17° に | GRIL-Calib で LiDAR-IMU 相対姿勢を再校正            |
| C | ~~両方の TF が別々に間違っている~~        | 両方とも GRIL-Calib で再導出         | 明日の GRIL-Calib 一発で決着                        |

`static_tf_launch.py:87-95` のコメントは A の解釈で書かれている
(「LiDAR は IMU と物理的に同じ機構に共締めされており動いていない」)。
であれば選ぶべきは A。しかし A なら GLIM `T_lidar_imu` は現状のままで OK
ではなく、pitch~0 (LiDAR-IMU がほぼ同姿勢) が正解になるはず。noetic 由来の
+9° pitch は「LiDAR-IMU が別マウントで +9° 傾いていた」時代の値ということに
なり、現在の実装 (共締め) に照らせば古い値。→ GRIL-Calib で更新すべき。

## 5. 今夜の GLIM 実験結果との突き合わせ

`docs/m5r-bench-data/2026-07-08-campus-outer/glim-out-{baseline,fixbias}/traj_lidar.txt`
の end-to-start 誤差:

| 設定                            | end-to-start | dx     | dy      | dz     | yaw     |
|---------------------------------|-------------:|-------:|--------:|-------:|--------:|
| baseline (`fix_imu_bias: false`) | 11.53 m     | -8.01  | -3.54   | **+7.49** | +18.17° |
| fixbias  (`fix_imu_bias: true`)  | 27.51 m     | -0.26  | **-25.36** | **+10.66** | +19.93° |

- `fix_imu_bias` で dx が改善したのは、bias が今まで dx 側の TF 誤差を吸収
  していたことの証左。
- 一方 dy が発散し dz も悪化 = **bias 自由度が消えたことで、TF 側の姿勢誤差
  (§4 の pitch 8° 差分 + §3 の roll 5.8° 差分) が LiDAR 残差に直接転写された**。
- 結論: **bias 制御は対症療法ではなく問題を隠す機構だった**。根本は
  extrinsic (LiDAR-IMU 相対姿勢) と、IMU-base_link 相対姿勢の両方の校正誤差。

## 6. GRIL-Calib (2026-07-09) への引き渡し

明日 GRIL-Calib を回すにあたり、以下の前提条件を明示しておく。

### 6.1 GRIL-Calib が推定するもの

- `T_lidar_imu` の相対姿勢 (rotation + translation)
- IMU accel bias, gyro bias

### 6.2 校正 bag 取得時の条件 (docs/ja/m6r-indoor-calib-bag-protocol.md §3)

- WHILL は静止 → 直進 → 8 の字 → 静止 の順で 5 分程度
- 静止パートを最低 30 秒確保 (bias 収束用)

### 6.3 校正前に確認したいベースライン数値 (§3.1 と一致するはず)

- 停止時 `linear_accel`: 期待 (+1.32, -0.99, +9.78) m/s²
- 停止時 gyro bias `gx`: 約 -0.019 rad/s (WHILL 特有の常時オフセット)

上記から乖離する場合は再マウント / 温度異常 / ハーネス緩みを疑う。

### 6.4 GRIL-Calib 結果適用時のチェック手順

1. 出力 `T_lidar_imu` の並進部分と、現行 static TF chain の
   `p_imu_in_lidar_frame = (-0.05, -0.40, -0.35)` の差が ± 2 cm 以内か
2. 出力回転を RPY 分解し、以下と比較:
   - GRIL-Calib 版 RPY と、GLIM 現行値 `(+2.00°, -9.00°, 0.00°)` の差
   - GRIL-Calib 版 RPY と、TF chain 期待値 `(+2.07°, -17.00°, -0.29°)` の差
   どちらに近いかで §4.4 の A/B シナリオを決着させる
3. `static_tf_launch.py:78` の `base_link → imu_link` に **roll = -0.101 rad
   (-5.77°) を追加** (§3.2)。物理再マウントなしにこの値を反映してよい (実測)
4. §4.4 のシナリオ A を採る場合、`static_tf_launch.py:101` の
   `base_link → velodyne` を GRIL-Calib で得た `T_base_lidar` に置換

### 6.5 追加提案: gyro bias の恒久補正

gx の -0.019 rad/s (-1.09 °/s) は 47 分走行で **-51° の heading 誤差**に相当。
`config_sensors.json` の `imu_bias_noise: 1e-05` は「1e-5 rad/s² per √Hz」相当
の bias random walk。1 時間走行で bias が √3600 × 1e-5 ≈ 6e-4 rad/s まで
成長し得るので、常時 -0.019 rad/s の "既知バイアス" は GRIL-Calib で estimate
するにしても、EKF (robot_localization) 側でも bias 除去した IMU を購読する
ほうが安全。次のイテレーションで `imu_sign_corrector` に gyro bias 減算オプ
ションを足すか、別ノードにするかを検討する。

## 7. 未検証項目 (今回の audit で埋められなかったもの)

- **LiDAR の base_link に対する物理姿勢**: 今回 IMU 姿勢は gravity 測定で
  出したが、LiDAR は点群平面フィットが必要で今夜は実施していない。GRIL-Calib
  の結果と、点群からの地面 fit を突合して整合を確認するのが確実
- **camera_link の姿勢**: RealSense extrinsic は M4R-2 の暫定値 (RPY=0)
  のまま。M6-R でチェスボード校正予定 (`static_tf_launch.py:118`)
- **base_link 自体が水平か**: 実測は「base_link 水平・地面平面」を前提。
  タイヤの空気圧不均一、重量偏重、路面傾斜で base_link が微傾斜している
  可能性は排除できない。厳密には水準器で確認したい

## 8. 参照

- 静止 IMU サンプリング: `scratchpad/imu_static_sample.py`
- TF chain 数値計算: `scratchpad/frame_audit.py`
- `imu_sign_corrector` ソース: `src/whill_sensors_bringup/whill_sensors_bringup/imu_sign_corrector.py`
- static TF 定義: `src/whill_sensors_bringup/launch/static_tf_launch.py`
- GLIM `T_lidar_imu` 由来コメント: `scripts/m5r3_run_glim.sh:262-286`
- noetic 校正値の由来: `docs/ja/m3-extrinsics-from-noetic.md`
- 今夜の比較実験結果: `docs/m5r-bench-data/2026-07-08-campus-outer/glim-out-{baseline,fixbias}/`
