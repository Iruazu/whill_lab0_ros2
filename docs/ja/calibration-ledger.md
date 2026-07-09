# Calibration Ledger — 現時点の「正」記録

このドキュメントは、`base_link` からの各センサ姿勢と `T_lidar_imu` の
**現時点で有効な値** と **その根拠** を 1 箇所にまとめた台帳。

一日で複数回書き換わる場合があるため、GLIM / GRIL-Calib / Nav2 のいずれかで
値を消費する前に必ずここを参照する。**必ず最新の commit hash と対応させる**。

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
