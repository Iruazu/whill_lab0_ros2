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

期待値 (§base_link → imu_link ref 2026-07-09 10:52 最終再固定後):

| 項目 | 期待 | tolerance | 判定 |
|------|------|-----------|------|
| ax   | +1.340 | ± 0.05 | ✓ or 中断 |
| ay   | -0.604 | ± 0.05 | ✓ or 中断 |
| az   | +9.815 | ± 0.05 | ✓ or 中断 |
| gx   | -0.01845 | ± 0.005 | ✓ or 中断 |

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


## 現時点の正 (as of 2026-07-09)

### base_link → imu_link
| 成分 | 値 | 根拠 |
|------|-----|------|
| x    | +0.38 m  | PR #61 実測 (後輪車軸線から前方 38 cm) |
| y    | -0.03 m  | PR #61 実測 (車体中心から右 3 cm) |
| z    | +0.47 m  | PR #61 実測 (地面から 47 cm、base_link は「地面」定義) |
| roll | -0.0614 rad (-3.52°) | 2026-07-09 10:52 gravity 実測 (2 回目マウント再固定後) |
| pitch| -0.1354 rad (-7.76°) | 同上 |
| yaw  | 0 | axis-aligned re-mount 前提 |

- 対応 commit: **`aed1e4d`** (`fix(sensors_bringup): finalize base_link->imu_link after 07-09 2nd remount`)
- 実測手順: `scratchpad/imu_live_check.py`
- 実測時の期待値 (500 サンプル、5 秒、WHILL 静止時):
  - ax ≈ +1.34 m/s²
  - ay ≈ -0.60 m/s²
  - az ≈ +9.82 m/s²
  - gx ≈ -0.019 rad/s (WHILL 固有の gyro bias、除去しない — GLIM 内蔵推定に任せる)
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
| **audit** (`GLIM_TLI_FROM_AUDIT=1`) | 同上 | (-0.030651, -0.063283, -0.001945, 0.997523) | 2026-07-09 audit (RPY -3.52°, -7.26°, 0°) |

- 対応 commit: **`494ea77`** (`feat(m5r3): env-gated GLIM_TLI_FROM_AUDIT + 07-09 4-way GLIM comparison`)
- 目視実証: 2026-07-09 23:20 の offline_viewer で seg-A が単線・drift 無しを確認
  (`docs/ja/imu-coordinate-audit.md` §7.9)
- **本番マップ生成時は `GLIM_TLI_FROM_AUDIT=1` 必須**
- 未計測: `yaw` 成分 (現在 0 仮定)。47 分フルループで yaw 起因の残留 drift が
  数 m 級で出る可能性が残る。出たら GRIL-Calib 6-DoF 校正を優先度上げ

### base_link → camera_link
- 未更新 (M4R-2 の仮置き RPY=0)。M6-R でチェスボード校正予定
- 影響範囲: RealSense を使う下流タスク (現在なし)

## 更新履歴

| Date       | Change                                      | Commit |
|------------|---------------------------------------------|--------|
| 2026-07-10 | lo-only xml に MaxAutoParticipantIndex=100 追加 (bringup で participant 枯渇 → 4 ノード死亡) | (this commit) |
| 2026-07-09 | initial ledger作成 (今日の反省を受けて)      | (this commit) |
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
