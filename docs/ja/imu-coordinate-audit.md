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

## 7. 2026-07-09 追記: LiDAR 姿勢実測と GLIM 検証

### 7.1 LiDAR の地面平面 fit — シナリオ B 確定

2026-07-09 収録 `2026-07-09-indoor-calib` bag の静止 30 スキャンに対して
ring 別解析 (scratchpad/lidar_ground_fit_multi.py、lidar_roll_from_ring0.py):

- Ring 0..15 の効果角度 (atan2(-z_med, xy_med)) は **全リングで 14.83-14.87°**
  = -15° 校正値と完全一致 → **LiDAR は base_link に対してほぼ完全に水平**
  (pitch ≈ -0.5°、roll ≈ 0°、plane fit std 0.4°)
- 4 象限で「見える面の高さ」は違うが (front 1.05m、back 1.27m、left 0.98m、
  right 1.05m)、これは LiDAR 傾きではなく屋内の物体/床高低差

→ §4.4 の **シナリオ B 確定** (LiDAR と IMU は別マウント。noetic の +9° pitch は
「IMU が -8° 傾いて LiDAR が水平だから見かけ上 imu-lidar 相対 +9° に見える」の
正体)。

### 7.2 audit 反映 T_lidar_imu で GLIM 4 者比較

LiDAR 水平 + IMU pitch -7.76° roll -3.52° → **R_lidar_imu ≈ RPY(-3.52°, -7.26°, 0°)**。
quaternion (qx, qy, qz, qw) = (-0.030651, -0.063283, -0.001945, 0.997523)。
`scripts/m5r3_run_glim.sh` に env `GLIM_TLI_FROM_AUDIT=1` で追加パッチ実装。

campus-outer bag (2813 秒、baseline commit `80af31f`) に対して 4 パターン
比較 (`docs/m5r-bench-data/2026-07-08-campus-outer/glim-out-*` および
`campus-outer-debiased/glim-out-audit-tli`):

| 実験                       | dx     | dy      | **dz**       | end-to-start | yaw drift |
|----------------------------|--------|---------|--------------|--------------|-----------|
| baseline (noetic T_lidar_imu) | -8.01  | -3.54   | **+7.49**    | 11.53 m      | +18.17°   |
| fix_imu_bias (§5)          | -0.26  | -25.36  | **+10.66**   | 27.51 m      | +19.93°   |
| **audit T_lidar_imu**      | -3.82  | -23.39  | **+1.28 (▼85%)** | 23.74 m  | +16.60°   |
| **debiased + audit T_lidar_imu** | -7.78 | -28.33 | **+0.39 (▼95%)** | 29.38 m | +18.76°   |

### 7.3 判明した事実

- **Z ドリフトの主犯は T_lidar_imu の roll 校正誤差** (5.5° 逆向き)。
  audit の実測姿勢を反映しただけで dz が 7.5m → 1.3m まで解消
- gyro bias (audit §3 の gx = -0.019 rad/s) を bag 冒頭 500 サンプル
  静止時 mean で事前減算しても **XY drift は改善せず、むしろ悪化**。
  GLIM 内蔵の bias 推定器と衝突した可能性大 (`IMU bias estimation seems
  inaccurate` 警告が 268 → 287 に増加)。
- **XY ドリフトの主因は gyro bias ではない**。残る候補:
  - T_lidar_imu の yaw 成分未計測 (今日の audit では 0 と仮定)
  - GLIM の loop closure が長時間走行で有効に働いていない
- yaw drift そのものは 47 分で 16-19° と大きくない (0.35°/min)。しかし
  47 分の間に「同じ廊下を通っても座標系が徐々に回転」して建物の複製が
  3-4 個生成されている (bag viewer 目視で確認済)

### 7.4 gyro debias 手法の再評価

事前減算による debias は本 audit の実験では **有害** と判明。理由推定:

1. **静的 bias 値が動作中の実 bias と異なる** (温度、加速度、機械振動で変化)
2. **GLIM の推定器を混乱** — GLIM は各サンプルで bias 状態を更新するが、
   事前減算されたデータに対しては bias state が 0 近くに拘束され、
   ロバスト性が下がる

代替策 (今後の候補):
- `imu_bias_noise` を config 側で 1e-5 → 1e-6 に絞って推定器を stiff にする
- または EKF (robot_localization) 側で bias 事前減算 (GLIM とは分離)
- 本命は GRIL-Calib で 6-DoF 校正 (bias も同時推定)

### 7.5 残る未検証項目

- **T_lidar_imu の yaw 成分**: 今日は 0 と仮定。真値との差が XY drift の
  主因である可能性高。GRIL-Calib 完走で実測すべき
- **GLIM の loop closure 効き**: 47 分 bag で建物複製が発生している。
  `config_global_mapping_gpu.json` の閾値調整で改善する余地あり
- **LiDAR の base_link に対する物理姿勢**: 今回 IMU 姿勢は gravity 測定で
  出したが、LiDAR は点群平面フィットが必要で今夜は実施していない。GRIL-Calib
  の結果と、点群からの地面 fit を突合して整合を確認するのが確実

### 7.6 追加発見: bag 内の 4.77 秒 IMU/LiDAR gap

2026-07-09 夜、audit-tli 結果を viewer で目視確認したところ、user が
「6:48 頃に全体が突然がくんと揺れ、その後 drift 由来のズレが出現」を報告。
run.log と bag timestamp を突き合わせた結果、bag 自体に **4.77 秒間の完全な
IMU/LiDAR 停止** が入っていることが判明:

```
last good IMU  : bag time 1783513647.598  (bag 冒頭から 1325.06 秒 = 22 分 05 秒地点)
next IMU       : bag time 1783513652.368  (4.77 秒後、diff = 4.769995)
LiDAR も同時に 3.96 秒停止 (diff = 3.956308)
```

run.log の該当行:
```
[glim] [warning] large time gap between consecutive IMU data!!
[glim] [warning] current=1783513652.368347 last=1783513647.598352 diff=4.769995
[glim] [warning] large time gap between consecutive LiDAR frames!!
[glim] [warning] current=1783513651.394380 last=1783513647.438072 diff=3.956308
[odom] [warning] insufficient number of IMU data between LiDAR scans!! num_imu=0
```

baseline / fix_imu_bias / audit-tli / debiased+audit の **4 実験すべてで同じ
gap を通過** し、いずれも通過後の pose graph に不定性が残る。**現在の
XY drift の実質下限は、この gap がハードコードしている**可能性が高い。

**原因の推定** (優先順):

1. **RMW/DDS 瞬断**: campus-outer 収録時のログでは 21:05 開始直後の
   ~ネットワーク切替タイミングに DDS の `ddsi_udp_conn_write` エラーが
   多発 (audit §runtime env RMW メモ参照)。同種の 4-5 秒級ハングが
   録画中に紛れ込んだ可能性が最有力
2. **rt_usb_9axisimu_driver の USB 再接続**: dmesg 未確認だが要らしさは中
3. **CPU governor throttle**: performance に設定していたが電源管理 IRQ で
   一瞬 powersave に落ちた可能性は低め

### 7.7 明日以降のプラン (Z drift 検証の完結条件)

user 合意 (2026-07-09):

1. **新規 bag 再録画**: 明日、DDS 瞬断リスクを最小化した状態 (WiFi 完全 OFF、
   デザリング切替なし、governor performance 確認) で campus 外周走行を再録。
   `ros2 bag info` で count が理論値と乖離しないこと + run.log で
   `large time gap` 警告ゼロを確認
2. **現 bag を gap で 2 分割して GLIM 検証**: 現在の campus-outer bag を
   bag time 1325.06 秒でカット → gap 前 (0-1325s ≒ 22 分) と gap 後
   (1330s-2813s ≒ 25 分) の 2 セグメントで独立に GLIM を回す
   - `ros2 bag record` の `--split-duration` は使えないので Python + rosbag2 API か
     `ros2 bag filter` (ROS 2 humble には未搭載) を検討。最悪 sqlite で
     bag_0.db3 を直接分割してもよい
3. **判定基準**: 2 セグメントとも drift が顕著でなければ「gap がなければ
   本番品質の bag を作れる」= 明日の再録画で本番マップ収録の目処が立つ

### 7.9 追試: bag を gap で 2 分割して GLIM (2026-07-09 23:00 頃)

user の判断で現 bag を bag time 1325.06s (gap 直前) でカットし、seg-A
(0-22min)、seg-B (22-47min) の 2 セグメントを独立に GLIM (`GLIM_TLI_FROM_AUDIT=1`)
にかけた。分割スクリプト: `scratchpad/split_bag_at_gap.py`。tf_static は
`scratchpad/` の後補填で 4 個入れ直し。

| run | duration | samples | end-start | dz (endpoint) |
|---|---|---|---|---|
| 47min audit-tli (full) | 47 min | 27,684 | 23.7 m | +1.28 |
| seg-A (0-22min, pre-gap) | 22 min | 13,051 | 60.3 m* | +4.05 |
| seg-B (22-47min, post-gap) | 25 min | 14,622 | 65.6 m* | +6.37 |

\* 各セグメント末での「起点からの直線距離」= WHILL がその時点でいた場所。
loop でないので loop error 指標としては無意味。

**目視判定 (offline_viewer で user が確認、2026-07-09 23:20)**:

- **seg-A**: 明確な単線ループ、drift 無し、Z 平坦 (色の帯はキャンパスの
  実高低差を表現)。**SLAM tracking 良好、audit T_lidar_imu は正しく機能**
- **seg-B**: 建物や地面が複数の高さで重複描画されている。**明確な Z drift**

**判定** — user の当初仮説通り、**bag 中の 4.77 秒 gap が主犯**:

- seg-A (gap 前だけ) では audit T_lidar_imu で **Z drift 完全解消**
- seg-B (gap 後) では復帰時のジャンプで Z 拘束が崩れ、以降累積
- 47min 全体で見た "Z drift 主犯 = T_lidar_imu roll 校正誤差 (audit で解消)" と
  "残る drift = gap 起因" が両立している

**私 (Claude) の誤診記録**: 当初 `traj_lidar.txt` の Z 全域 min-max span
(21m) だけ見て「Z が walk 中に 20m 振れている」と誤判定した。実際は
キャンパスの本物の高低差 (坂、階段、建物 2 階部分の壁点群) を全部「drift」
として計算に含めていたため。目視 or 「同じ物理場所での Z 一致性」を
まず確認すべきだった。数値だけで断定した点は反省項目として残す
(次回同種の判定では offline_viewer 目視を必ず併用する)。

**明日以降の本番マップ生成の道筋**:

1. **再録画 (DDS 瞬断リスクを最小化)**: WiFi OFF、デザリング切替なし、
   governor performance 確認。ros2 bag info で count が理論値通り、
   run.log で `large time gap` 警告ゼロを保証
2. 再録画 bag に対して `GLIM_TLI_FROM_AUDIT=1` で GLIM → seg-A クラスの
   クリーンな結果が期待できる
3. DUFOMap で動的物体除去 → `docs/maps/campus/` に成果物規約 (ADR-0004,
   ADR-0005) で格納 → M5-R 完了

### 7.10 練り込むべき follow-up (明日以降)

- gap 検知の bringup 側フック (7.6 の再発防止): 録画中の IMU 100Hz 未達を
  監視して即警告する node の追加 (`imu_sign_corrector` の副作用として
  組み込むのが自然か)
- 現 bag に対して IMU 線形補間で gap を埋める debias スクリプト流用テスト
  (7.7 の検証と独立に、`debias_gyro_bag.py` の骨組みを流用して 15 分程度で
  作れる)
- **camera_link の姿勢**: RealSense extrinsic は M4R-2 の暫定値 (RPY=0)
  のまま。M6-R でチェスボード校正予定 (`static_tf_launch.py:118`)
- **base_link 自体が水平か**: 実測は「base_link 水平・地面平面」を前提。
  タイヤの空気圧不均一、重量偏重、路面傾斜で base_link が微傾斜している
  可能性は排除できない。厳密には水準器で確認したい

## 8. 参照

- 静止 IMU サンプリング: `scratchpad/imu_static_sample.py`
- TF chain 数値計算: `scratchpad/frame_audit.py`
- 07-09 追記: LiDAR ring 別解析: `scratchpad/lidar_ground_fit_multi.py`、
  `scratchpad/lidar_roll_from_ring0.py`
- 07-09 追記: gyro bias 事前減算 bag rewrite: `scratchpad/debias_gyro_bag.py`
- 07-09 追記: bag を gap で分割: `scratchpad/split_bag_at_gap.py`
- `imu_sign_corrector` ソース: `src/whill_sensors_bringup/whill_sensors_bringup/imu_sign_corrector.py`
- static TF 定義: `src/whill_sensors_bringup/launch/static_tf_launch.py`
- GLIM `T_lidar_imu` 由来コメント: `scripts/m5r3_run_glim.sh:262-286`
- GLIM T_lidar_imu env-gated 追加パッチ: `scripts/m5r3_run_glim.sh` (`GLIM_TLI_FROM_AUDIT`)
- noetic 校正値の由来: `docs/ja/m3-extrinsics-from-noetic.md`
- 07-08 の比較実験結果: `docs/m5r-bench-data/2026-07-08-campus-outer/glim-out-{baseline,fixbias}/`
- 07-09 の追加実験結果: `docs/m5r-bench-data/2026-07-08-campus-outer/glim-out-audit-tli/`
  および `docs/m5r-bench-data/2026-07-08-campus-outer-debiased/glim-out-audit-tli/`
