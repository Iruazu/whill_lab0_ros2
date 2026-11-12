# `campus-m8`（M8-2 で作り直す campus 地図）

M8-2（#152〜#155）で、07-10 の地図用 bag から作り直す campus 地図。旧 `campus` は残し、別の site として置く。**採用は未定**。実機・屋外での確認（#156）と、ユーザーによる手塗りの確定（#154）が済むまで、デモは旧 `campus` を使う。

旧地図との違いは、点群の校正だけを正しくしたこと（Hi-Res の実機に VLP16db.yaml を使っていた。#144）。経路と元の記録は同じ。

## 状態

| 段階 | Issue | 状態 |
| --- | --- | --- |
| 1. 元 bag の点群を Hi-Res に補正 | #152 | 済（オフライン検証まで） |
| 2. GLIM → DUFOMap で static.pcd | #153 | 下の節を参照 |
| 3. 占有格子と手塗りの候補 | #154 | 下の節を参照 |
| 4. waypoint とローカライザの再生確認 | #155 | 下の節を参照 |

## 1. 元 bag の補正（#152）

元の bag（`~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag/`、2162 秒、点群 21311 個）にはパケットが無く、`/velodyne_points` だけがある。点に `ring` があるので、VLP16db.yaml で計算された点を、距離と方位を保ったまま Hi-Res の仰角へ解析的に戻せる（式は `scripts/velodyne_reproject_bag.py` の冒頭）。元の bag は読むだけで、変えていない。

### 実行したコマンド

```bash
cd ~/whill_lab0_ros2   # worktree でもよい
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
P=$(ros2 pkg prefix velodyne_pointcloud)/share/velodyne_pointcloud/params
B=~/bags/m5r-bench-data/2026-07-10-campus-outer-final
python3 scripts/velodyne_reproject_bag.py $B/bag $B/bag-hires \
  --src-calib $P/VLP16db.yaml --dst-calib $P/VLP16_hires_db.yaml
```

- 出力：`~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires/`（13 GB、sqlite3）。点群以外の話題は、受信時刻と QoS ごとそのまま写す。
- 所要：14 分（1 スレッド）。出力先があると止まるので、やり直すときは出力先を変える。
- 結果：点群 21311 個を補正、他のメッセージ 216195 個を複写。合計 237506 個で、元の bag と一致する。読んだ点から距離を計算し直したときの z の食い違いは最大 3.96 µm（float32 の丸め）。

### 検証

1. **ドライバそのものとの一致**（`scripts/velodyne_reproject_verify.sh`）。パケットと点群の両方がある段階Aの bag で、(a) 記録された点群を本ツールで補正したものと、(b) パケットを `velodyne_transform_node` に VLP16_hires_db.yaml で流した出力を、点ごとに比べた。点の並び（organized、NaN の位置）、ring、座標を見る。

   | bag | 比べた点群 / 点 | 最大の差 | p99.9 | NaN・ring の不一致 |
   | --- | --- | --- | --- | --- |
   | `sensor_check_20261028/chk_empty_0m_160726` | 113 / 2,276,593 | 0.015 mm | 0.004 mm | 0 |
   | `sensor_check_20261028/chk_chair_3m_160533` | 108 / 2,263,144 | 0.015 mm | 0.004 mm | 0 |

   対照として、補正していない記録を同じ基準と比べると最大 3.2 m ずれ、FAIL になる（比較が校正の違いを見分けられることの確認）。
   Issue の「0.00 mm で一致」は、小数第2位までなら 0.02 mm であり、厳密な 0 ではない。差は float32 の丸め（130 m で 1 回 0.008 mm、入力・補正・ドライバの出力でそれぞれ丸める）で、判定の許容は 0.05 mm にした。

   ```bash
   scripts/velodyne_reproject_verify.sh ~/bags/sensor_check_20261028/chk_empty_0m_160726 <新しい作業ディレクトリ>
   ```

2. **全行程の出力と、#147 の区間の一致**。#147 で別に作った先頭 300 秒（`~/bags/imu_investigation_20261030/seg_0_300_hires`）と、今回の全行程の先頭 2957 個は、6184 万点すべてで差 0.000 mm（同じ式なので当然だが、全行程を通しても壊れていないことの確認）。

   ```bash
   python3 scripts/velodyne_compare_clouds.py ~/bags/imu_investigation_20261030/seg_0_300_hires \
     ~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires
   ```

確かめていないこと：補正後の点群が実物の形に合うこと（真値が無い）。これは地図の品質（下の節）と屋外の静止確認（#149）で見る。

## 2. GLIM → DUFOMap（#153）

### 設定

#147 の結論（`docs/reports/2026-11-02-glim-imu-root-cause.md`）に従い、補正した bag を使い、`fix_imu_bias` は `false`。残る選択肢（IMU の雑音、T_lidar_imu）は、全行程で実際に回して比べてから決めた。どの実行も GPU（RTX 3080 Ti、driver 595.99.02）で終了コード 0、保存まで完了した。

| 実行 | T_lidar_imu | 雑音 acc / gyro | better ratio 回転 / 並進 / 速度 | 警告の回数 | 加速度バイアスの最大 [m/s²] | 所要 |
| --- | --- | --- | --- | --- | --- | --- |
| 旧 campus（補正前の点群） | audit | 不明（#147） | 0.85 / 0.03 / 0.07 | — | 1.06 | 692 s |
| **A：採用** | audit | 0.008 / 0.0013 | 0.90 / 0.31 / 0.51 | 190 | 0.385 | 468 s |
| B | audit | 0.05 / 0.02（上流） | 0.88 / 0.37 / 0.44 | 302 | 0.171 | 506 s |
| C | TF から計算し直した値 | 0.008 / 0.0013 | 0.90 / 0.15 / 0.33 | 295 | 0.709 | 487 s |

- better ratio は GLIM が警告を出すときにだけ表示される累積値で、上は最後の表示（`scripts/imu_investigation/summarize_glim_runs.py`）。警告の閾値は回転 0.7、並進 0.4、速度 0.5。どの実行も並進は閾値に届かず、IMU の予測が等速の予測より良いとは言い切れない。ただし旧地図の 0.03 / 0.07 からは大きく改善した。#147 の 300 秒の結果（並進 0.39〜0.40、速度 0.62〜0.64）より全行程では低い。
- audit 値：`[-0.05, -0.4, -0.35]`、RPY (−4.09°, −7.61°, 0°)（`scripts/m5r3_run_glim.sh`）。
- C の値：#145 の新しい base_link→velodyne（0.42, 0.36, 0.777, RPY 0.24°, 0.97°, −0.317°）と、今の base_link→imu_link（0.38, −0.03, 0.47, RPY −4.085°, −8.107°, 0）から計算した `[-0.0326, -0.3915, -0.306]`、RPY (−4.33°, −9.08°, +0.36°)、四元数 `[-0.037443, -0.079177, 0.000105, 0.996157]`。センサ校正の作業班の値と独立に計算して一致を確かめた。

**A を採用した理由**：C は IMU の指標がすべて悪化し（並進 0.15、速度 0.33、バイアス 0.71）、地図の傾きも 2.04° に増えた（下の表）。A と B はループの閉じと傾きがほぼ同じで、A の方が警告が少なく、速度の比が閾値を超える。B の方がバイアスが小さく並進の比は高いので、差は決定的ではない。#147 の 300 秒の実験と同じ雑音である A を選んだ。

**C の結果から言えること**：IMU と LiDAR の相対の向きは、TF の鎖から計算した値（pitch −9.08°）よりも audit 値（−7.61°）の方が、この bag の IMU と点群に合う。どちらかの TF（imu_link 側か velodyne 側）、または両方が実物とずれている可能性がある。この差は地図の傾きに直接出る。校正台帳（#151）と屋外の静止確認（#149）で扱う課題で、この地図では決めていない。

### 実行したコマンド

```bash
cd ~/whill_lab0_ros2   # このブランチ
B=~/bags/m5r-bench-data/2026-07-10-campus-outer-final
GLIM_TIMEOUT=3600s GLIM_DOMAIN_ID=152 scripts/imu_investigation/glim_control.sh \
  $B/bag-hires ~/bags/imu_investigation_20261030/gpu135-post-reboot/config $B/glim-m8-hires-noise-local \
  config_odometry_gpu.json:odometry_estimation.fix_imu_bias=false
scripts/m5r_run_dufomap.sh $B/glim-m8-hires-noise-local/glim $B/dufomap-m8-hires-noise-local   # 57 s
cp $B/dufomap-m8-hires-noise-local/static.pcd docs/maps/campus-m8/
cp $B/glim-m8-hires-noise-local/glim/traj_lidar.txt docs/maps/campus-m8/
cp $B/glim-m8-hires-noise-local/config/*.json docs/maps/campus-m8/glim_config/
```

- `glim_control.sh` は設定を `cp -rL` で実ファイルとして固定し、`glim_rosbag` を stdin なし（`</dev/null`）、localhost の専用 domain で動かす。全行程は 300 秒を超えるので `GLIM_TIMEOUT` を延ばした。
- B は雑音の2行（`config_sensors.json:sensors.imu_acc_noise=0.05`、`imu_gyro_noise=0.02`）、C は `config_sensors.json:sensors.T_lidar_imu=[...]` を足しただけ。出力は同じディレクトリの `glim-m8-hires-noise-upstream/`、`glim-m8-hires-tli-recomputed/`。
- `static.pcd` は Git 管理外。sha256 と点数は `metadata.yaml`。

### 品質

| 指標 | 旧 campus | A（採用） | B | C | 出所 |
| --- | --- | --- | --- | --- | --- |
| 経路長 | 1310.1 m | 1301.1 m | 1304.4 m | 1301.6 m | `m5r3_loop_error.py` |
| 始点と終点の差（軌跡） | 1.317 m | **0.118 m** | 0.125 m | 0.129 m | 同 |
| うち dx / dy / dz | +0.107 / −0.161 / +1.303 | +0.022 / −0.116 / **+0.006** | +0.030 / −0.121 / +0.003 | +0.035 / −0.123 / +0.012 | 同 |
| yaw のずれ | −0.16° | −0.23° | −0.24° | −0.23° | 同 |
| 軌跡の平面の傾き | 1.81°（方位 119°） | **0.72°**（152°） | 0.69° | 2.04°（164°） | `map_tilt_metrics.py` |
| 平面からの残差 RMS | 1.32 m | 0.34 m | 0.31 m | 0.34 m | 同 |
| 軌跡の高さの幅 | 9.79 m | 3.89 m | 3.79 m | 9.40 m | 同 |
| 始点の地面の傾き / 高さ | 1.15° / −0.969 m | 0.77° / −0.397 m | 0.77° / −0.396 m | 未計算 | 同（半径 6 m） |
| 終点の地面の傾き / 高さ | 1.25° / +0.550 m | 0.77° / −0.397 m | 0.78° / −0.399 m | 未計算 | 同 |
| 地面からの LiDAR の高さ（始点） | 1.32 m | 0.75 m | 0.75 m | — | 同 |
| 点数 | 9,649,880 | 7,614,632 | 7,668,165 | — | PCD |

```bash
scripts/m5r3_loop_error.py docs/maps/campus-m8/traj_lidar.txt
scripts/map_tilt_metrics.py docs/maps/campus-m8/traj_lidar.txt --pcd docs/maps/campus-m8/static.pcd
scripts/imu_investigation/summarize_glim_runs.py ~/bags/m5r-bench-data/2026-07-10-campus-outer-final/glim-m8-hires-*
```

読み取れること：

- **ループの閉じ**：始点と終点の差は 1.317 m → 0.118 m。旧地図の差の大半だった高さ（dz 1.30 m）がほぼ消え、始点と終点の地面の高さが 1 mm 以内で一致する（旧地図は 1.52 m 食い違っていた）。
- **傾き（campus README の仮説 A/B）**：旧地図の 1.81° は、点群の校正を直しただけで 0.72° に減り、T_lidar_imu の pitch を 1.5° 変えただけで 2.04° に増えた。地形は同じなので、**1.81° の大半は校正の誤りから生じた地図の傾き**で、「始点の路面が 1.81° 傾いていた」（仮説 A）では説明できない。仮説 B（GLIM の重力合わせ）は、校正の誤りを通じて起きたという意味で近いが、原因は初期化ではなく校正だった。残る 0.72〜0.77° が実際の地形か地図の傾きかは、真値が無いので決められない（水準器での確認が要る。calibration-ledger §0）。
- 地面からの LiDAR の高さ 0.75 m は、#145 の実測の z 0.777 m（base_link から）と近い。旧地図は 1.32 m で、仰角の誤りが高さに出ていた。
- 点数は約 21% 減った。同じ DUFOMap の設定なので、点群の歪み（上下に広がった点）が減ったためと考えるが、確かめていない。

目視（`start_side_view.png`）：始点を通る幅 2 m の帯を横から見ると、旧地図は地面が約 1.4 m 離れた2層に割れている（往路と復路の高さのずれ）。新しい地図は1層で、ゴーストが無い。下段は新しい地図の上面図。

確かめていないこと：壁などで測る B1（CloudCompare）、offline_viewer での全体の目視、実物との寸法の一致。
