# Calibration Ledger — 現時点の「正」記録

このドキュメントは、`base_link` からの各センサ姿勢と `T_lidar_imu` の
**現時点で有効な値** と **その根拠** を 1 箇所にまとめた台帳。

一日で複数回書き換わる場合があるため、GLIM / GRIL-Calib / Nav2 のいずれかで
値を消費する前に必ずここを参照する。**必ず最新の commit hash と対応させる**。

## 0. 走行日の朝一チェックリスト (本番録画前 15 分)

### 0.1 環境変数と DDS 設定

```bash
# 1. bashrc に 2 行あることを確認 (無ければ追加して source):
#      export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
#      export CYCLONEDDS_URI=file:///home/systemlab/whill_lab0_ros2/configs/cyclonedds-lo-only.xml

# 2. daemon 再起動 (CLI 側の反映):
ros2 daemon stop && ros2 daemon start

# 3. bringup を起動する前に、そのターミナルで確認 (安全ネット):
echo $CYCLONEDDS_URI     # ← file:///.../cyclonedds-lo-only.xml が返ること
echo $RMW_IMPLEMENTATION # ← rmw_cyclonedds_cpp
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor  # ← performance
```

**注意**: `CYCLONEDDS_URI` は各プロセス起動時に読まれる env。既存シェルで
`source ~/.bashrc` し忘れた別ターミナルから bringup を起動すると、そのプロセスは
古い設定 (=lo 限定でない) で走る。**bringup 起動前の `echo $CYCLONEDDS_URI`
は必ず 1 行挟む**。

**bringup が participant index エラーで一部ノード死亡した場合**: lo 単一 IF は
CycloneDDS 既定 `MaxAutoParticipantIndex=9` を超えるため、`Failed to find a
free participant index for domain 0` で遅く起動したノードが落ちる。lo-only xml
の `<Discovery>` に `<MaxAutoParticipantIndex>100</MaxAutoParticipantIndex>` が
入っているか確認 (2026-07-10 追加、commit で対応)。

### 0.2 センサ健全性 — 静止 IMU 5 秒 (走行**前**)

Bringup 後 (IMU 静止状態):

```bash
python3 scratchpad/imu_live_check.py
```

期待値 (§base_link → imu_link ref 2026-07-10 朝の再測定後):

| 項目 | 期待 | tolerance | 判定 |
|------|------|-----------|------|
| ax   | +1.397 | ± 0.05 | ✓ or 中断 |
| ay   | -0.699 | ± 0.05 | ✓ or 中断 |
| az   | +9.782 | ± 0.05 | ✓ or 中断 |
| gx   | -0.0184 | ± 0.005 | ✓ or 中断 |

**中断条件**: いずれか tolerance 外 = IMU マウントが動いている or ハーネス緩み。
その場合は物理再固定 → `frame_audit.py` で TF 再計算 → static_tf 更新 →
ledger 更新 → 再開。走行は絶対にしない。

### 0.3 DDS 検証 bag (10 分)

**正常系** と **異常系** の両方を見る (片方だけだと lo 限定ミスで無音状態を
gap ゼロと誤読するリスクあり):

```bash
# ターミナル A: bringup 起動 (Ctrl+C しない)
ros2 launch whill_localization odom_bringup_launch.py

# ターミナル B: 検証 bag 録画開始
cd docs/m5r-bench-data/2026-07-10-dds-verify  # (mkdir しておく)
ros2 bag record -o bag /velodyne_points /imu/data_rep145 /tf_static

# ターミナル C: 意図的に外乱を発生 (5 分程度)
#   - WiFi ON/OFF 切替
#   - デザリング接続/切断
#   - Ethernet 抜き差し (差してあれば)
# → CycloneDDS が外部 discovery で振り回されないかテスト

# 5 分経ったら B で Ctrl+C
```

**判定** (両方満たすこと):

1. **正常系**: `ros2 bag info bag` で count が理論値通り
   - `/velodyne_points`: 秒数 × 10 の 90% 以上
   - `/imu/data_rep145`: 秒数 × 100 の 95% 以上
   - **これが満たされないと lo 限定の設定ミスで無音状態の可能性大 → 検証失敗**
2. **異常系**: bringup のログ (ターミナル A) で `large time gap` 警告ゼロ
   - もしくは `IMU loop back`、`ddsi_udp_conn_write ... failed` の類が出ていないこと

両方 ✓ → 本番録画に進む。片方でも ✗ → 原因調査 (本番は次回に)。

### 0.4 本番録画

- run-id: `2026-07-10-campus-outer-final` (仮)
- run-dir: `docs/m5r-bench-data/<run-id>/`
- コマンド: `ros2 bag record -o bag /velodyne_points /imu/data_rep145 /tf_static`
- 走行完了 → Ctrl+C
- `ros2 bag info` で count 再確認 (0.3 と同じ基準)
- run.log で `large time gap` ゼロ確認

### 0.5 走行**後** の静止 IMU 5 秒 (0.2 と同じ)

物理振動でマウントがズレていないかの保険。走行前 (0.2) と一致していれば
「この bag は commit hash `aed1e4d` の T_lidar_imu 値と整合」が保証される。
ズレていたら、この bag 用の T_lidar_imu を実測値で再計算 (`frame_audit.py`)。

### 0.6 GLIM 実行

```bash
GLIM_TLI_FROM_AUDIT=1 ./scripts/m5r3_run_glim.sh \
  docs/m5r-bench-data/<run-id>/bag \
  docs/m5r-bench-data/<run-id>/glim-out-audit-tli
```

判定: `traj_lidar.txt` の loop_error + offline_viewer 目視で seg-A 級品質を確認。
両方 ✓ → CloudCompare B1 → DUFOMap → `docs/maps/campus/` → M5-R 完了。

---


## 現時点の正 (as of 2026-07-10)

### base_link → imu_link
| 成分 | 値 | 根拠 |
|------|-----|------|
| x    | +0.38 m  | PR #61 実測 (後輪車軸線から前方 38 cm) |
| y    | -0.03 m  | PR #61 実測 (車体中心から右 3 cm) |
| z    | +0.47 m  | PR #61 実測 (地面から 47 cm、base_link は「地面」定義) |
| roll | -0.0713 rad (-4.09°) | 2026-07-10 朝 gravity 実測 (3 連続で ±0.003 内安定を確認) |
| pitch| -0.1415 rad (-8.11°) | 同上 |
| yaw  | 0 | axis-aligned re-mount 前提 |

- 対応 commit: **(this commit)** (`fix(sensors_bringup): re-audit base_link->imu_link for 07-10 recording`)
- 実測手順: `scratchpad/imu_live_check.py` (静止 5 秒) → 逸脱時 `scratchpad/frame_audit.py --ax ... --ay ... --az ...` で RPY 逆算
- 実測時の期待値 (500 サンプル、5 秒、WHILL 静止時):
  - ax ≈ +1.397 m/s²
  - ay ≈ -0.699 m/s²
  - az ≈ +9.782 m/s²
  - gx ≈ -0.0184 rad/s (WHILL 固有の gyro bias、除去しない — GLIM 内蔵推定に任せる)
- **昨日値との差**: roll -0.57°、pitch -0.35° (溝の ± 2-3° 再固定バラツキ範囲)。
  物理再固定はせず一晩置いただけで発生 → 締結の応力緩和による微沈み込みと解釈。
  以後同傾向が続くなら固定機構の見直し検討 (今日の走行はこの新値のまま進める)
- **走行前後で必ずチェック**: マウント再現性が ± 2° あるため、
  bag 収録前と収録後の両方でこの数値を再現しているか確認する。ズレたら
  その bag の T_lidar_imu を実測値ベースで再計算する必要あり (`scratchpad/frame_audit.py`)

### base_link → velodyne
| 成分 | 値 | 根拠 |
|------|-----|------|
| x    | +0.484136 m | noetic 由来 (imu 位置 + noetic extrinsic_T) |
| y    | +0.381548 m | 同上 |
| z    | +0.793704 m | 同上 (地面から 79.4 cm、user 実測 80 cm と一致) |
| roll | -0.035342 rad (-2.03°)  | 未更新 (noetic 由来) |
| pitch| +0.156983 rad (+9.00°)  | 未更新 (noetic 由来) |
| yaw  | -0.005527 rad (-0.32°)  | 未更新 (noetic 由来) |

- 未検証事項: LiDAR の実測姿勢は **ほぼ水平** (`docs/ja/imu-coordinate-audit.md`
  §7.1 の ring 別解析)。現行 TF の pitch = +9° は誤り。ただし GLIM は
  ROS TF を見ないので影響は Nav2 / rviz のみ。M6-R 開始前に更新するべし
- 影響範囲: Nav2 costmap、rviz 表示。GLIM/GRIL-Calib には影響なし

### GLIM の `T_lidar_imu` (config_sensors.json)
| Mode | 並進 [m] | 回転 (qx, qy, qz, qw) | 由来 |
|------|---------|----------------------|------|
| baseline (デフォルト) | (-0.05, -0.4, -0.35) | (0.017399, -0.078447, 0.001369, 0.996765) | noetic (RPY +2°, -9°, 0°) |
| **audit** (`GLIM_TLI_FROM_AUDIT=1`) | 同上 | (-0.035606, -0.066319, -0.002368, 0.997163) | **2026-07-10 pre-run audit** (RPY -4.09°, -7.61°, 0°) |

- 対応 commit: **`39bf794`** (07-10 pre-run 追随。前回 `494ea77` は 07-09 audit)
- 導出式: `R_lidar_imu.roll = imu.roll`, `R_lidar_imu.pitch = imu.pitch + 0.5°`
  (LiDAR は base_link 系で概ね水平、pitch -0.5° の残留のみを補正)
- 目視実証:
  - 2026-07-09 23:20: 07-09 audit 版で seg-A 単線・drift 無しを確認 (`docs/ja/imu-coordinate-audit.md` §7.9)
  - **2026-07-10 15:xx: 07-10 pre-run 版で本番 bag `2026-07-10-campus-outer-final` を処理 →
    viewer 3 視点すべて PASS**。詳細は下記「本番マップ採用結果」節
- **本番マップ生成時は `GLIM_TLI_FROM_AUDIT=1` 必須**
- 未計測: `yaw` 成分 (現在 0 仮定)。47 分フルループで yaw 起因の残留 drift が
  数 m 級で出る可能性が残る。今回の run は yaw -0.16° で収まり、GRIL-Calib
  優先度は当面「必要になれば」レベルに下げる

## 本番マップ採用結果 (2026-07-10)

M5-R 山越え記録。本番マップ候補として正式採用。

### 対象 run
- run-id: `2026-07-10-campus-outer-final`
- bag: `docs/m5r-bench-data/2026-07-10-campus-outer-final/bag` (2162 s / 12.8 GiB)
- glim-out: `docs/m5r-bench-data/2026-07-10-campus-outer-final/glim-out-audit-tli/`
- git commit: **`39bf794`** (GLIM audit quat = 07-10 pre-run)

### 定量指標

| 項目 | 値 | 判定 |
|------|-----|------|
| loop_error (end-to-start) | 1.317 m / 1310.098 m = **0.10%** | ✅ (基準 0.1-0.3% の最良側) |
| dx / dy / **dz** (per-axis) | +0.107 / -0.161 / **+1.303** | dz は視認不可レベル |
| yaw drift | -0.16° | ✅ (数 m 級 yaw 起因 drift 懸念は不発) |
| GLIM 実行時間 | 691.8 s | bag 2162 s の 32% |
| Peak VRAM | 3297 MiB | 参考値 |
| B1 数値代替 (ground z-gap) | **1.394 m** (traj dz と 7% 差) | ✅ 独立測定で SLAM 忠実性を確認 |
| CloudCompare B1 (壁 3 点) | 未実施 (GUI 難航 → 数値代替で代用) | 上記で代替済 |

### 目視判定 (3 視点すべて PASS)

- 真上: 建物・柱の輪郭一重、単線ループ (前日の複製ゴースト消失)
- 3D 俯瞰: Z レイヤー化なし、色グラデーションは実高低差のみ
- 地上レベル拡大: 壁面単線、二重壁なし、街路樹の個体分離

### 既知残差 (M6-R 引き継ぎ)

- **map tilt 1.81°** (追記 2026-07-10 19:xx、占有格子調査で発見):
  `traj_lidar.txt` (x, y, z) の平面フィットで
  ```
  z = -0.0155·x + +0.0276·y + 1.815
  tilt vs vertical = 1.81°  (azimuth 119° from +x)
  residual RMS = 1.32 m
  z_span 9.79 m のうち  平面成分 7.10 m (72.5%)  残差 1.32 m
  ```
  GLIM 出力の world z 軸が真の gravity と 1.81° ズレている。実地形
  起伏は残差 RMS 1.3 m 級。原因は 2 候補:
  1. IMU audit を実施した WHILL 静止位置 (start pose 付近) の路面が
     実際に 1.81° 傾いていた (§7.10「base_link 水平性は水準器で確認
     したい」未処理のツケ)
  2. GLIM の初期 gravity alignment が瞬間 IMU 値だけで走り、ズレが
     残った
  M6-R の localizer の gravity-aware factor 設計 or Nav2 costmap の
  垂直面判定に影響しうる。M6-R 開始前に「マップを de-tilt」or
  「localizer 側で許容」を判断する
- **IMU better ratios が低いまま**: trans=0.03 / vel=0.07。bag 47 分の
  大半で LiDAR 主導、IMU 予測寄与が薄い。マップ品質には影響しない
- **bias_acc が最後まで未収束**。localization で IMU 予測に頼る設計を
  組む場合はここが弱点になる。M6-R の localizer 選定時に評価軸へ

### 校正プロトコル改善案 (次回本番録画時に必ず取り入れる)

07-10 の追試で発見された map tilt 1.81° の再発防止として、§0 の朝一
チェックリストに以下を追加する:

1. **§0.2 に前置き: 起動地点の路面水準チェック**
   - 手順: 静止させた WHILL に手のひらサイズの水準器を base_link 面
     (座面 or 底部フレーム) に置き、前後・左右の両軸で気泡が中央にある
     ことを目視確認
   - 判定: ± 1° 以内 (気泡が水準器の 1 目盛り内) なら OK。それ以上なら
     path を数十 cm ずらして再試行、あるいはその場所での録画をあきらめる
   - 不合格時: base_link の傾きは IMU audit で自動測定できない (audit は
     gravity vs IMU の相対で、base_link は仮定として「水平」を入れて
     いるため)。この確認が抜けると本 audit で言う "base_link → imu_link
     の RPY" が正しくても、真の gravity vs map の関係がズレて 1.81°
     tilt が map に焼きつく (2026-07-10 の実例)
2. **§0.2 の代替: 複数地点 gravity 平均**
   - 上記水準器チェックが物理的に難しい場合 (水準器を持ち込めない、屋外
     で振動が大きい等)、以下で近似:
   - 走行予定経路上の 3-5 箇所で WHILL を停めて 5 秒静止 IMU サンプリング
     (`scratchpad/imu_live_check.py` を各地点で実行)
   - 各地点の gravity vector を回転させて世界座標での平均を取り、
     残差 RMS が小さいことを確認 (RMS < 0.3° なら「経路の路面がおおむね
     水平」と言える)
   - 大きければその方向 (steepest ascent 方位) がわかるので、audit で
     推定した base_link → imu_link RPY からその成分を差し引くことで
     真の imu_link → gravity 方向が復元できる。GLIM 側では
     `T_lidar_imu` の rotation にこの補正を追加する

どちらも 2026-07-10 の本番録画時には未実施。M6-R 用の再録画または
補助マップ取得時に導入する。

### 3 日間の追い込み総括

| 指標 | 07-08 seg-A/B (fixbias 実験) | **07-10 pre-run 本番** | 改善率 |
|------|-----------------------------|------------------------|-------|
| loop_error | 11.53 m | 1.317 m | 1/9 |
| yaw drift | 18° | 0.16° | 1/100 |
| 建物ゴースト | 3-4 個複製 | 消失 | ✅ |
| Z レイヤー | 多層パンケーキ | 単一面 | ✅ |

犯人リスト (すべて特定 + 修正 + 検証プロトコル整備済):
- DDS gap (テザリング IF): CycloneDDS lo-only 恒久設定 + MaxAutoParticipantIndex=100
- 座標系の層問題: base_link → imu_link の RPY 実測反映 (2 日連続で微更新)
- マウント管理: `imu_live_check.py` + `frame_audit.py` で走行前後の再現性を機械判定

### base_link → camera_link
- 未更新 (M4R-2 の仮置き RPY=0)。M6-R でチェスボード校正予定
- 影響範囲: RealSense を使う下流タスク (現在なし)

## 更新履歴

| Date       | Change                                      | Commit |
|------------|---------------------------------------------|--------|
| 2026-07-10 | **M5-R 完了**: `docs/maps/campus/` README + metadata + tilt 1.81° 記録 + 校正プロトコル改善案 (§本節「校正プロトコル改善案」) | (this commit) |
| 2026-07-10 | 占有格子に relative z-slice + anchor-free-radius 導入 (tilt/starved 問題を解消) | `7a9924a` |
| 2026-07-10 | 占有格子に trajectory-anchor free マーキング導入 | `a180c8b` |
| 2026-07-10 | B1 数値代替 `m5r3_b1_numeric.py` + campus map 適用 (ground z-gap 1.394 m) | `71b6407` |
| 2026-07-10 | 本番マップ採用: `2026-07-10-campus-outer-final` viewer PASS (loop_error 0.10%、複製・Z ゴーストなし) | `4ca6704` |
| 2026-07-10 | GLIM audit T_lidar_imu を 07-10 pre-run 導出値に更新 (RPY -4.09°/-7.61°) | `39bf794` |
| 2026-07-10 | base_link → imu_link 再測定 (roll -4.09°, pitch -8.11°、昨日値から -0.57°/-0.35° の応力緩和分) | `9cc4be2` |
| 2026-07-10 | lo-only xml に MaxAutoParticipantIndex=100 追加 (bringup で participant 枯渇 → 4 ノード死亡) | `d5c6eff` |
| 2026-07-09 | 朝一チェックリスト §0 追加                   | `f81d6c1` |
| 2026-07-09 | initial ledger作成 (今日の反省を受けて)      | `deb317d` |
| 2026-07-09 | GLIM audit T_lidar_imu 追加 (env-gated)     | `494ea77` |
| 2026-07-09 | base_link → imu_link 2nd remount           | `aed1e4d` |
| 2026-07-09 | base_link → imu_link 1st remount           | `171b01d` |
| 2026-07-09 | base_link → imu_link に roll -5.77° 追加    | `80af31f` |

## 更新プロトコル

1. IMU 姿勢を変えた/マウントを触ったら:
   - `scratchpad/imu_live_check.py` で 5 秒サンプリング
   - `scratchpad/frame_audit.py` の該当行を実測値に更新して RPY 逆算
   - `static_tf_launch.py` を更新 → commit
   - **この台帳の対応行 + 更新履歴を同時に更新**
2. GRIL-Calib を回した/結果を採用したら:
   - `docs/m5r-bench-data/<run>/gril-calib-out/GRIL_Calib_result.txt` を根拠に
   - `scripts/m5r3_run_glim.sh` の quaternion を更新 (env-gated or デフォルト)
   - **この台帳の GLIM T_lidar_imu 行を更新**
3. どこかで値を消費した (Nav2 起動、GLIM 実行、DUFOMap 等):
   - この台帳の該当値と commit hash を README や manifest に転記して再現性確保
