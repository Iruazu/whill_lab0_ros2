# 校正台帳

車体とセンサの実測値の**唯一の正本**。launch・設定・文書に書かれた値がこの台帳と食い違う場合は、台帳を正とし、食い違いを §6 に載せて直す。

- 値を変えるときは、台帳の該当表と、表の「反映先」に書いた file:line を同じコミットで変える。旧値は §7 の履歴に1行で残し、消さない。
- launch のコメントは導出を繰り返さず、この台帳の節を指す。
- 状態の区分：
  - **未測定**：仮の値、または値がない。
  - **実測済み**：測定方法つきの値があるが、実機の静止確認・走行で検証していない。
  - **実機受入済み**：受入試験（静止確認 #149 など）で許容内と確認した。
- 座標系：`base_link` は後輪車軸の中点を地面に下ろした点（x 前、y 左、z 上）。並進は m、角度は tf2 の RPY（固定軸 X→Y→Z）。

## 0. 走行前後の IMU 静止確認

IMU のマウント（マジックテープ + 溝）は再固定で ±2〜3° 動くため、地図用の bag を撮るときは走行の前と後に、WHILL を止めて確認する。

1. `python3 scratchpad/imu_live_check.py`（静止 5 秒）。
2. 期待値（§4.1 の値を測った 2026-07-10 朝の実測）と比べる。

| 項目 | 期待値 | 許容 |
| --- | --- | --- |
| ax | +1.397 m/s² | ±0.05 |
| ay | −0.699 m/s² | ±0.05 |
| az | +9.782 m/s² | ±0.05 |
| gx | −0.0184 rad/s | ±0.005（WHILL 固有の gyro バイアス。除去せず GLIM に推定させる） |

3. 許容外ならマウントが動いている。走行せず、物理的に再固定 → `scratchpad/frame_audit.py` で RPY を逆算 → §4.1 と launch を更新する。走行後だけ外れた場合は、その bag 用の T_lidar_imu を実測値から計算し直す。
4. 起動地点の路面の傾きは IMU から分離できない（base_link を水平と仮定しているため）。水準器で前後・左右とも ±1° 以内の場所で行う。2026-07-10 の地図の傾き 1.81° はこの確認をしなかったことが一因と考えられる（`docs/maps/campus/README.md`）。

DDS・governor・bag 録画の手順はこの台帳では扱わない。現行手順は `docs/runbook-live-demo.md`。

## 1. footprint

| 項目 | 値 | 測定日 | 方法 | 誤差 | 状態 |
| --- | --- | --- | --- | --- | --- |
| Nav2 の外形 | 半径 0.45 m の円（中心 base_link） | — | 実測なし | 前端は x 約 0.75〜0.8 m と見られ、前の約 0.3 m がはみ出す | **未測定**（#143） |
| 車幅 | 0.6 m（failsafe Layer D の扇形の前提） | — | 出典なし | — | **未測定**（#143） |
| 前端・後端・最大幅・出っ張り | — | — | 巻尺で測る予定 | — | **未測定**（#143） |

- 反映先：`src/whill_navigation/config/nav2_params.yaml:231`（local）、`:282`（global）の `robot_radius`。`src/whill_safety/whill_safety/failsafe_node.py:59`（車幅 0.6 m の前提）、`:72`（半角 30°）。
- 多角形への変更と全系統での統一は #157。

## 2. LiDAR（VLP-16 Puck Hi-Res）

### 2.1 base_link → velodyne

| 成分 | 値 | 測定日 | 方法 | 誤差 |
| --- | --- | --- | --- | --- |
| x | 0.42 m | 2026-10-28 | 巻尺 2 回（旧 URDF と一致） | 未評価。静止物の位置で x に +2〜+8 cm のずれがあり、テープの基準か TF かを #149 で切り分ける |
| y | 0.36 m | 2026-10-28 | 巻尺 2 回（旧 URDF と一致） | 未評価 |
| z | 0.777 m | 2026-10-28 | Hi-Res の校正表で投影した点群の床面フィット | 未評価。旧記録の巻尺 0.80 m とは 2.3 cm 差（巻尺で測った位置が光学中心か不明） |
| roll | +0.24° | 2026-10-28 | 同上（床面フィット） | 路面の横断勾配を含む。#149 で道路の勾配と比べる |
| pitch | +0.97° | 2026-10-28 | 同上（床面フィット） | 路面の勾配を含む |
| yaw | −0.317° | 2026-07（noetic） | noetic `extrinsic_R` の分解（`docs/m3-extrinsics-from-noetic.md`） | 床面からは観測できない。未検証 |

- 状態：**実測済み**。屋外静止確認（#149）は未了。
- 反映先：`src/whill_sensors_bringup/launch/static_tf_launch.py:73`（`static_tf_velodyne`。角度は `math.radians` で変換）。
- 地図との関係：この TF は base_link 系の点群の位置を変える。旧地図（campus）とは NDT が合わなくなるので、地図の作り直し（#153）と同時にマージする。
- 旧値の誤り：旧 TF は IMU TF に noetic の LiDAR↔IMU extrinsic を足したもの。pitch +9° は「IMU が約 −8° 傾き、LiDAR はほぼ水平」という相対姿勢を、IMU を base_link と axis-aligned とみなして対 base_link の姿勢に流用した誤り（`docs/imu-coordinate-audit.md` §4.4 シナリオ B）。

### 2.2 校正ファイル（仰角表）

| 項目 | 値 | 確認日 | 方法 | 状態 |
| --- | --- | --- | --- | --- |
| 機種 | VLP-16 Puck Hi-Res（上下 ±10°） | 2026-10-28 | 本体の銘板 | 確認済み |
| 校正ファイル | `VLP16_hires_db.yaml`（velodyne_pointcloud 同梱） | 2026-10-28 | 床面フィット：VLP16db.yaml では高さ 1.15 m、Hi-Res で再計算すると 0.777 m | **実測済み**。屋外静止確認（#149）は未了 |

- 反映先：`src/whill_sensors_bringup/launch/sensors_launch.py:151`（`velodyne_calibration` 引数、既定値は `:153`）。旧 `VLP16db.yaml`（±15°）は引数で選べる。
- 旧 bag（`/velodyne_points` だけのもの）の補正は `scripts/velodyne_reproject_bag.py`（#152）。GLIM の IMU 不良の主因がこのファイルの誤りだったことは `docs/reports/2026-11-02-glim-imu-root-cause.md`。

## 3. カメラ（RealSense D435）base_link → camera_link

| 成分 | 値 | 測定日 | 方法 | 誤差 |
| --- | --- | --- | --- | --- |
| x | 0.47 m | 2026-10-28 | 後輪車軸→LiDAR 回転軸 0.42 m（巻尺）+ ホルダ奥行きの半分 約 0.05 m − 前面→camera_link 約 0.004 m（データシート、`_d435.urdf.xacro`） | 未評価 |
| y | 0.378 m | 2026-10-28 | 車体中心→LiDAR 軸 0.36 m（巻尺）+ 左 IR 原点の筐体中心からのずれ +0.0175 m（`d435_cam_depth_py`） | 未評価 |
| z | 0.73 m | 2026-10-28 | レンズ中心（巻尺） | 静止物 bag で物体最大高の誤差 +0.3〜+0.8 cm |
| roll / pitch / yaw | 0 | 2026-10-28 | 水平・前向き取付の申告 | 床面フィットで約 1° 上向きの疑い |

- 状態：**実測済み**。実機静止確認は未了（#149）。
- 反映先：`src/whill_sensors_bringup/launch/static_tf_launch.py:93`（`static_tf_camera`）。launch への反映は #146（ブランチ `fix/146-camera-tf`）で行う。このブランチ単体では launch は旧値のまま（§6）。

## 4. IMU（RT 9 軸 USB IMU）

### 4.1 base_link → imu_link

| 成分 | 値 | 測定日 | 方法 | 誤差 |
| --- | --- | --- | --- | --- |
| x | 0.38 m | 2026-06-24（#61） | 巻尺 | 未評価 |
| y | −0.03 m | 2026-06-24（#61） | 巻尺 | 未評価 |
| z | 0.47 m | 2026-06-24（#61） | 巻尺 | 未評価 |
| roll | −0.0713 rad（−4.09°） | 2026-07-10 | 静止 gravity の逆算（3 回連続、各軸 0.003 m/s² 未満で安定） | 再固定で ±2〜3° 動く |
| pitch | −0.1415 rad（−8.11°） | 2026-07-10 | 同上 | 同上 |
| yaw | 0 | 2026-07-08 | axis-aligned に付け直し（目視） | 未評価 |

- 状態：**実測済み**（2026-07-12 の M6R-2 実機受入はこの値の構成で PASS。ただし TF 単体の受入試験はしていない）。
- 反映先：`src/whill_sensors_bringup/launch/static_tf_launch.py:55`（`static_tf_imu`）。
- roll / pitch が大きいのは座面とマウント溝の傾きによる。
- 依存関係：IMU TF は LiDAR TF から導出していない（逆に、#145 以前の LiDAR TF が IMU TF から導出されていた）。LiDAR TF を変えても IMU TF は変わらない。

### 4.2 加速度の符号

| 項目 | 値 | 方法 | 状態 |
| --- | --- | --- | --- |
| `/imu/data_raw` の加速度 | 静止時 z ≈ −9.8（REP-145 と逆符号） | 静止時の実測（#56） | 確認済み |
| 補正 | 加速度 3 軸の符号を反転して `/imu/data_rep145` に再配信。gyro・mag・orientation はそのまま | — | 実装済み |

- 反映先：`src/whill_sensors_bringup/whill_sensors_bringup/imu_sign_corrector.py:68-70`。EKF・GLIM は `/imu/data_rep145` を使う。

### 4.3 GLIM の T_lidar_imu（地図作成のみ。runtime は使わない）

| モード | 並進 [m] | 回転 (qx, qy, qz, qw) | RPY | 由来 | 状態 |
| --- | --- | --- | --- | --- | --- |
| baseline（既定） | (−0.05, −0.4, −0.35) | (0.017399, −0.078447, 0.001369, 0.996765) | (+2°, −9°, 0°) | noetic | 旧値 |
| audit（`GLIM_TLI_FROM_AUDIT=1`） | 同上 | (−0.035606, −0.066319, −0.002368, 0.997163) | (−4.09°, −7.61°, 0°) | 2026-07-10 の IMU 静止値から導出（LiDAR を pitch −0.5° と仮定。#144 以前の点群で推定） | 2026-07-10 の地図（campus）で目視 PASS |

- 反映先：`scripts/m5r3_run_glim.sh:294-301`（baseline）、`:335-342`（audit）。
- **新しい TF との食い違い**：§2.1 と §4.1 から計算した T_lidar_imu は、並進 (−0.033, −0.392, −0.306)、RPY (−4.33°, −9.08°, +0.36°)。audit 値との差は roll 0.24°、pitch 1.47°、yaw 0.36°、並進 (1.7, 0.9, 4.4) cm。audit 値は VLP16db.yaml で見た LiDAR の pitch（−0.5°）を前提にしている。#153 で使う値は #147 の結論とあわせて決める（§6）。

## 5. D435 の深度の校正

| 項目 | 値 | 日付 | 状態 |
| --- | --- | --- | --- |
| FW | 5.17.3.10（5.12.11 から更新） | 2026-10-30 | 更新済み |
| 校正表 | FW 更新の前後でバイト単位で一致 | 2026-10-30 | 工場出荷値のまま |
| 深度の誤差（校正前） | 4 m で約 +0.26 m 奥。床の高さが 2 m で −0.1 cm、3 m で −2.7 cm、4 m で −7.9 cm | 2026-10-28 | 静止物 bag で測定 |
| On-Chip / Tare 校正 | — | — | **未測定**（#148） |

- 手順：`docs/plans/2026-10-28-avoidance-baseline.md` §4.4（ブランチ `feat/avoidance-baseline`）。
- バックアップ（削除しない）：`~/rs_backup_938422070073/`（FW 更新前のフラッシュ全体、校正表 `calib_table_fw5.17.3.10_20261030_095735.bin`、`rs-enumerate-devices -c` の出力）。
- 反映先：カメラ本体のフラッシュ（リポジトリの設定ファイルではない）。

## 6. 同じ値を持つ他の場所と食い違い

この台帳の値を写している場所。食い違いは直すか、担当の Issue を書く。履歴として残す文書（bench-data、レポート、監査記録）は当時の値のまま残し、ここには載せない。

| 場所 | 書かれている値 | 台帳 | 状況 |
| --- | --- | --- | --- |
| `src/whill_navigation/config/nav2_params.yaml:231`、`:282` | robot_radius 0.45 | §1 未測定 | #143（実測）→ #157（多角形） |
| `src/whill_safety/whill_safety/failsafe_node.py:59` | 車幅 0.6 m の前提 | §1 未測定 | #143 → #157 |
| `src/whill_sensors_bringup/launch/static_tf_launch.py:93` | camera_link (0.54, 0.382, 0.79) | §3 (0.47, 0.378, 0.73) | **食い違い**。#146 のマージで一致 |
| `src/whill_sensors_bringup/README.md` の TF 表 | camera_link (0.54, 0.382, 0.79) | §3 | **食い違い**。#146 のマージで一致 |
| `src/whill_perception/config/patchworkpp.yaml:25` | sensor_height 0.79 | §2.1 z 0.777 | **食い違い**（1.3 cm）。担当 Issue なし |
| `src/whill_navigation/config/pointcloud_to_laserscan.yaml:48-49` | 高さの帯 0.05〜1.6 m（ADR-0009、旧 TF・旧校正表で調整） | §2 | 値は台帳の写しではないが、旧 TF 前提で調整した値。見直しの担当 Issue なし |
| `scripts/m5r_pcd_to_occupancy.py:486` | `--lidar-mount-height` 既定 0.79 | §2.1 z 0.777 | **食い違い**（旧スクリプト。現用は v2 か要確認） |
| `scripts/m5r3_run_glim.sh:294-301`、`:335-342` | T_lidar_imu baseline / audit | §4.3 | **食い違い**（pitch 1.47° ほか）。#147 → #153 |
| `scripts/m5r4_run_gril_calib.sh:150` | imu_sensor_height 0.47 | §4.1 z 0.47 | 一致 |
| `docs/m3-extrinsics-from-noetic.md:78-79`、`:114-171` | velodyne・camera_link の旧値と導出 | §2.1、§3 | 旧値の導出記録。冒頭に「過去の記録」の注記あり。維持 |
| `docs/imu-coordinate-audit.md:27-29` | 当時の TF ツリー | — | 監査記録（履歴）として維持 |
| `docs/repository-refresh-2026-10-30.md:62-64` | 当時の食い違い一覧 | — | 2026-10-30 時点の記録として維持 |

## 7. 履歴

旧値はここに残す。経緯の詳細は各コミットと参照先の文書。

| 日付 | 項目 | 旧値 → 新値 | 理由 | コミット / Issue |
| --- | --- | --- | --- | --- |
| 2026-10-28 | LiDAR 校正ファイル | VLP16db.yaml → VLP16_hires_db.yaml | 実機は Puck Hi-Res | #144 |
| 2026-10-28 | base_link→velodyne | (0.484136, 0.381548, 0.793704) / RPY (−0.035342, +0.156983, −0.005527) rad → (0.42, 0.36, 0.777) / RPY (0.24°, 0.97°, −0.317°) | 巻尺 + 床面フィット。旧値は IMU TF + noetic extrinsic | #145 |
| 2026-10-28 | base_link→camera_link | (0.54, 0.382, 0.79) → (0.47, 0.378, 0.73)、RPY 0 | 巻尺 + データシート。旧値は M4R-2 の概算を #61 で IMU と一緒にずらしたもの | #146 |
| 2026-07-10 | GLIM audit T_lidar_imu | RPY 07-09 audit → (−4.09°, −7.61°, 0°) | 07-10 朝の IMU 静止値に追随 | `39bf794` |
| 2026-07-10 | base_link→imu_link | roll −3.52° / pitch −7.76° → −4.09° / −8.11° | 一晩で締結が緩んだ分（再固定はしていない） | `9cc4be2` |
| 2026-07-09 | GLIM audit T_lidar_imu | 追加（環境変数で切替） | IMU 監査の反映 | `494ea77` |
| 2026-07-09 | base_link→imu_link | 2 回目の再固定 | マジックテープの微スライド | `aed1e4d` |
| 2026-07-09 | base_link→imu_link | 1 回目の再固定 | 同上 | `171b01d` |
| 2026-07-09 | base_link→imu_link | roll 0 → −5.77° | 静止 gravity 監査で roll を発見 | `80af31f` |
| 2026-07-08 | imu_link | IMU を物理的に 90° 回して axis-aligned に付け直し、yaw 0 | GLIM の T_lidar_imu を 90° 込みにすると発散したため | #61 後の作業 |
| 2026-06-24 | base_link→imu_link 並進 | M4R-2 の値 → (0.38, −0.03, 0.47) | 巻尺（このとき LiDAR・カメラも同量ずらした） | `b2cb7df`（#61） |
| 2026-06-20 | base_link→各センサ | identity → noetic 由来の値（M4R-2） | noetic の LiDAR↔IMU extrinsic を流用 | `78f6658`（#36）、`docs/m3-extrinsics-from-noetic.md` |

旧版の台帳にあった「走行日の朝一チェックリスト」（lo-only DDS 前提、2026-07-09）と 2026-07-10 の地図採用の記録は、それぞれ現行 runbook と `docs/maps/campus/README.md` に移っている。旧版は `git show 14208d0:docs/calibration-ledger.md` で読める。

## 8. 更新の手順

1. 測る。方法・日付・誤差を記録する。
2. この台帳の該当表を書き換え、旧値を §7 に1行で残す。
3. 表の「反映先」の file:line を同じコミットで変える。launch を変えたら `ros2 launch <pkg> <launch>.py --show-args` と、static TF なら `tf2_echo` で値を確かめる。
4. §6 の写しを確かめ、食い違いを直すか担当 Issue を書く。
5. 実機で確かめたら状態を「実機受入済み」にし、確認した試験（Issue・日付）を書く。
