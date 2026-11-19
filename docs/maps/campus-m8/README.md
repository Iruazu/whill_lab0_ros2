# `campus-m8`（M8-2 で作り直した campus 地図）

M8-2（#152〜#155）で、07-10 の地図用 bag から作り直した campus 地図。旧 `campus` は残し、別の site として置く。

**2026-11-13 から既定の site**（ユーザー判断、#190）。3 つの launch・`start_dispatch_demo.sh`・`check_map_sync.py`・Web の背景の既定は campus-m8 で、Nav2 は手塗りの候補（`map_variant:=candidate`、3 節）を読む。旧 `campus` は fallback として `site:=campus`（Nav2 は `map_variant:=v2`）で選べる（[runbook](../../runbook-live-demo.md)「旧地図（campus）に戻す」）。屋外の受入（#156）は既定にするための条件から外し、既定にした後の検証として行う。手塗りの確定（#154）もまだ。

**2026-11-19 から `candidate` は、手塗りを補正後の地理院の縁まで広げた版（154w）**（ユーザー判断 R7、R6 は gap 1 の版）。広げる前の手塗りのままの版は `map_variant:=candidate_hand`（`occupancy_candidate_hand.yaml` → `v2/candidate_hand_final.pgm`）で選べる。戻し方は 3 節 154w の「採用」と [runbook](../../runbook-live-demo.md)「手塗りのままの地図（candidate_hand）に戻す」。

経緯：この README の当初の版（2026-11-12）は「採用は未定。#156 と #154 が済むまで、デモは旧 `campus` を使う」としていた。2026-11-13 に、#186（LiDAR の TF と校正台帳）を main に入れて基準にすること、campus-m8 を #190 のマージで既定にすること、#156 はマージの後に検証することをユーザーが決めた。main には戻し先として `m8-pre-calib-20261113` のタグがある。

旧地図との違いは、点群の校正を正しくしたこと（Hi-Res の実機に VLP16db.yaml を使っていた。#144）と、GLIM の設定（#147 の結論）。経路と元の記録は同じ。

この地図は #145 の新しい LiDAR の TF と Hi-Res の校正（#144）を前提にする。#144・#145 の前の launch（VLP16db.yaml、#145 の前の TF）のまま使うと NDT の当てはまりが旧地図より悪い（4 節）。

**前提：#144 と #145（#186）を入れた構成で使う。** #144・#145 の前の launch の点群と TF では、この地図に対する NDT の当てはまりが旧地図の 15 倍悪い（4 節）。当初の版はこの理由で「#144 と #145 が main にマージされるまで、この地図で自律走行しない」としていた。#186 は 2026-11-13 に main に入り（TF 0.42, 0.36, 0.771 m、roll −0.05°、pitch +0.40°、yaw −0.3167°）、この条件は満たされた。#186 より前の版（タグ `m8-pre-calib-20261113` など）の launch とこの地図を組み合わせない。#156 の屋外受入も、#186 を入れた構成で行う。

### TF・校正が変わったときに作り直しが要るか

- この地図は、GLIM に渡した T_lidar_imu（audit 値、2 節）と、Hi-Res の校正で補正した点群だけから作った。GLIM・DUFOMap は velodyne フレームの点群と IMU で動き、**base_link→velodyne の TF は使っていない**。
- したがって、#151（校正台帳）や #149（屋外の静止確認）で base_link→velodyne の値が #145 から変わっても、**地図の作り直しは要らない**。ただし NDT の当てはまりは LiDAR の TF に依存するので、**4 節の NDT の再生比較は新しい TF でやり直す**（`--lidar-tf` を変えるだけ）。2026-11-13 の TF の変更（LiDAR のテープ固定）では、4 節の「2026-11-13 の再比較」でやり直した。
- 作り直しが要るのは、GLIM に渡す T_lidar_imu を変えるとき（2 節の実行 C のように、地図の傾きが変わる）と、点群の校正（VLP16_hires_db.yaml）を変えるとき。

## 状態

| 段階 | Issue | 状態 |
| --- | --- | --- |
| 1. 元 bag の点群を Hi-Res に補正 | #152 | 済（オフライン検証まで） |
| 2. GLIM → DUFOMap で static.pcd | #153 | 済（オフラインの品質指標まで。屋外の確認は未） |
| 3. 占有格子と手塗りの候補 | #154 | 候補まで。**手塗りの確定はユーザー**。既定の Nav2 の地図は候補（`candidate`）で、2026-11-19 から地理院の縁まで広げた版（154w）。手塗りのままの版は `candidate_hand`。広げた版の屋外の走行は未 |
| 4. waypoint とローカライザの再生確認 | #155 | 再生まで。**屋外の受入（#156）は未**（既定にした後の検証） |
| 5. 既定の site にする | #190 | 済（2026-11-13）。旧 `campus` は fallback |

| ファイル | 中身 | Git |
| --- | --- | --- |
| `static.pcd` | DUFOMap 後の点群（7,614,632 点、91 MB） | 管理外。正本と複製の手順は 2 節の「static.pcd の置き場」 |
| `traj_lidar.txt` | GLIM の軌跡（TUM、21300 姿勢） | 管理 |
| `glim_config/` | 実行した GLIM の設定（実ファイル） | 管理 |
| `metadata.yaml` | 出典と設定 | 管理 |
| `align_from_campus/` | 旧 campus からの座標変換と waypoint の写像 | 管理 |
| `v2/` | v2 の機械レイヤ、`candidates/`（手塗りの候補。`candidate_*_merged.png` は 154w で広げた版、`candidate_*_merged_hand.png` は広げる前）、`candidate_final.pgm`（広げた版）、`candidate_hand_final.pgm`（広げる前） | 管理 |
| `v2/experiments/202/`, `202b/`, `202c/`, `202d/` | 縁石からの自動抽出の試作（#202、3 節。202b は DUFOMap の元の点、202c は bag のスキャンを軌跡で重ねた密な面、202d は GSI の道路縁を縁ごとに証拠へ寄せる試作）。既定の地図には使わない | 管理（小さい画像と数値だけ） |
| `v2/experiments/154w/` | 手塗りを 202d の補正後の地理院の縁（段・構造物で寄せた区間だけ）まで広げた確定版の候補（#154、3 節）。2026-11-19 に既定の `candidate` に採用した | 管理（小さい画像と数値だけ） |
| `georef.yaml`, `reference-gsi.md` | 国土地理院の建物に合わせた map → 平面直角座標系第IX系・緯度経度の変換と、GSI の道路との比較（#205） | 管理 |
| `v2/experiments/205/` | 上の照合の重ね絵と数値 | 管理（小さい画像と数値だけ） |
| `occupancy_candidate.yaml` | `map_variant:=candidate` で読む候補の地図（`v2/candidate_final.pgm`、2026-11-19 から 154w で広げた版）。既定の site で variant を省いたときもこれ | 管理 |
| `occupancy_candidate_hand.yaml` | `map_variant:=candidate_hand` で読む、広げる前の手塗りのままの地図（`v2/candidate_hand_final.pgm`、2026-11-19 までの既定）。戻し先として残す | 管理 |
| `waypoints.yaml` | 配車地点（候補） | 管理 |
| `start_side_view.png` | 始点付近の横からの図（旧と新） | 管理 |

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
   cd ~/whill_lab0_ros2   # このブランチの checkout
   scripts/velodyne_reproject_verify.sh ~/bags/sensor_check_20261028/chk_empty_0m_160726 ~/reproject_verify_$(date +%Y%m%d_%H%M%S)
   ```

   パケットを再生するので、呼び出し元の `ROS_DOMAIN_ID` は引き継がない（runbook の端末は 0 で、動いているローカライザに届く）。domain は `VERIFY_DOMAIN_ID`（既定 152）、localhost のみで、その domain にノードがあれば何も起動せずに止まる。PASS には、基準の点群の 99% 以上に同じ stamp の補正後の点群があることも要る。

2. **全行程の出力と、#147 の区間の一致**。#147 で別に作った先頭 300 秒（`~/bags/imu_investigation_20261030/seg_0_300_hires`）と、今回の全行程の先頭 2957 個は、6184 万点すべてで差 0.000 mm（同じ式なので当然だが、全行程を通しても壊れていないことの確認）。

   ```bash
   cd ~/whill_lab0_ros2   # このブランチの checkout
   source /opt/ros/humble/setup.bash
   python3 scripts/velodyne_compare_clouds.py ~/bags/imu_investigation_20261030/seg_0_300_hires \
     ~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires
   ```

   メモリ：1つ目の引数（基準）は全部を float64 でメモリに持ち、2つ目は1個ずつ読む。300 秒の区間を基準にすると約 2 GB で済む。全行程の bag（約 4.5 億点）を基準にすると 14〜20 GB 要るので、小さい方を先に渡す。

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
- C の値：実行した時点（2026-11-12）の #145 の base_link→velodyne（0.42, 0.36, 0.777, RPY 0.24°, 0.97°, −0.317°。2026-11-13 に LiDAR をテープで固定した後の実測で z・roll・pitch が (0.771, −0.05°, +0.40°) に変わった。4 節の「2026-11-13 の再比較」）と、当時の base_link→imu_link（0.38, −0.03, 0.47, RPY −4.085°, −8.107°, 0）から計算した `[-0.0326, -0.3915, -0.306]`、RPY (−4.33°, −9.08°, +0.36°)、四元数 `[-0.037443, -0.079177, 0.000105, 0.996157]`。センサ校正の作業班の値と独立に計算して一致を確かめた。

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
- 実行した版：GLIM の開始時の HEAD は 14208d0 で、`glim_control.sh` の `GLIM_TIMEOUT` の変更は未コミットだった（15a350b で記録）。詳細と glim / glim_ros の版、GPU ドライバは `metadata.yaml`。

### static.pcd の置き場

`static.pcd` は Git 管理外（`.gitignore` の `docs/maps/**/*.pcd`）。

| 置き場 | 役割 |
| --- | --- |
| `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/dufomap-m8-hires-noise-local/static.pcd` | **正本**。DUFOMap の出力そのもの。変えない・消さない |
| この作業の worktree の `docs/maps/campus-m8/static.pcd` | 正本の複製（sha256 一致を確認済み）。worktree を消すと無くなる |

- sha256 は `d244865866e3eacdea6e26851dd3ed61ba626029fdc6dd4f2dabfb6d720d03b5`、点数 7,614,632（`metadata.yaml`）。
- **作り直しでは同じファイルにならない**。GLIM の GPU 実行は非決定的なので、同じ bag・同じ設定・同じコマンドで回しても軌跡と点群が少し変わり、sha256 は一致しない。品質の数値（2 節の表）も少し変わる。この地図を使うなら、作り直さずに正本を複製する。
- main の checkout（launch が `docs/maps/<site>/` を読む場所）に置く手順。マージの後に実行する：

  ```bash
  cd ~/whill_lab0_ros2
  cp ~/bags/m5r-bench-data/2026-07-10-campus-outer-final/dufomap-m8-hires-noise-local/static.pcd docs/maps/campus-m8/static.pcd
  echo "d244865866e3eacdea6e26851dd3ed61ba626029fdc6dd4f2dabfb6d720d03b5  docs/maps/campus-m8/static.pcd" | sha256sum -c
  ```

  `docs/maps/campus-m8/static.pcd: OK` と出れば正しい。`FAILED` なら使わない。
- **危険：正本は 1 か所にしかない**。worktree の複製も同じディスク（`~` と同じ NVMe）にあり、ディスクの故障や `~/bags` の整理で両方失われうる。作り直しても同じ地図にならないので、外付けの媒体か研究室の共有ストレージに、sha256 を添えて複製を置くことを勧める（この作業では置いていない。91 MB をリポジトリに入れることはしない）。

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
- 地面からの LiDAR の高さ 0.75 m は、#145 の実測の z（base_link から。この比較をした時点では 0.777 m、2026-11-13 のテープ固定後は 0.771 m）と近い。旧地図は 1.32 m で、仰角の誤りが高さに出ていた。
- 点数は約 21% 減った。同じ DUFOMap の設定なので、点群の歪み（上下に広がった点）が減ったためと考えるが、確かめていない。

目視（`start_side_view.png`）：始点を通る幅 2 m の帯を横から見ると、旧地図は地面が約 1.4 m 離れた2層に割れている（往路と復路の高さのずれ）。新しい地図は1層で、ゴーストが無い。下段は新しい地図の上面図。

確かめていないこと：壁などで測る B1（CloudCompare）、offline_viewer での全体の目視、実物との寸法の一致。

## 旧地図からの座標変換

両地図は同じ bag から作ったので、GLIM の軌跡の同じ時刻の点は同じ場所を指す。21300 組の対応から剛体変換 T_new_old を最小二乗で求めた（`scripts/map_align.py`、結果は `align_from_campus/`）。

| 項目 | 値 |
| --- | --- |
| 並進 x / y / z | −0.005 / +0.029 / −1.241 m |
| yaw | −0.104° |
| z 軸の傾き | 1.27°（旧地図の傾きを打ち消す分） |
| 変換後の残差（水平） | 中央値 0.081 m、p95 0.266 m、最大 0.387 m |
| 変換後の残差（3次元） | 中央値 0.81 m、p95 1.86 m（旧地図の高さのゆがみは剛体では吸収できない） |
| 点群の ICP との差 | 水平 0.02 m、z −0.34 m、回転 0.27° |

水平の差は数 cm〜数十 cm なので、2次元の地図（格子・手塗り・waypoint）は、この変換で旧地図と重ねられる。ただし p95 で 0.27 m の局所差が残る。

```bash
scripts/map_align.py --old-traj docs/maps/campus/traj_lidar.txt --new-traj docs/maps/campus-m8/traj_lidar.txt \
  --old-pcd ~/whill_lab0_ros2/docs/maps/campus/static.pcd --new-pcd docs/maps/campus-m8/static.pcd \
  --waypoints docs/maps/campus/waypoints.yaml --out docs/maps/campus-m8/align_from_campus
```

## 3. 占有格子と手塗りの候補（#154）

**手塗りは確定していない。** ここにあるのは、旧地図の塗りを機械的に写した候補と、その候補で合成した地図だけ。確定はユーザーが行う。

### 作ったもの

- `v2/` の機械レイヤ（`occupied_step.png` など）：`pcd_to_occupancy_v2.py` を新しい PCD で実行。格子は旧 campus v2 と同じ（origin [−128.25, −90.35]、6640×6295、0.05 m）にした。旧地図の塗りと GIMP で同じ画素に重ねられ、Web の背景の meta とも形が揃う。
- `v2/candidates/`：旧地図の塗りを T_new_old で写した候補。最近傍で写すだけで、形は変えない（v2 の Rule 1）。

  | ファイル | 中身 | 旧のセル数 → 写したセル数 |
  | --- | --- | --- |
  | `candidate_roadway_mask.png` | 旧 `roadway_mask.png`（走行可能域の白リスト） | 1,793,532 → 1,793,532 |
  | `candidate_free_mask.png` | 旧 `free_mask.png`（塩の消去） | 351,895 → 351,895 |
  | `candidate_keepout_mask.png` | 旧 `keepout_mask.png` | 0 → 0 |
  | `candidate_direct_free.png` | `final.pgm` に直接塗った拡幅（30dbfb1、0c903bc の正門前、54ce32b・e741e9a の x −66〜−42 など）。どのマスクにも入っていないので、`final.pgm` と、同じマスクで合成し直した結果の差から取り出した | 419,092 → 419,090 |
  | `candidate_roadway_merged.png`, `candidate_free_merged.png` | 上の roadway / free に直接の拡幅を足したもの（合成の入力。直接の拡幅は roadway と free の両方に入れないと FREE にならない） | — |
  | `candidate_review_overlay.png` | 候補の合成結果に色を重ねた確認用（青=roadway、黄=free、緑=直接の拡幅） | — |
  | `candidate_transfer.json` | 変換と各セル数の記録 | — |

  旧 `final.pgm` の直接の塗りは、FREE を増やしただけ（FREE を減らした・障害物を足したセルは 0）。そのうち 4,357 セルは機械の障害物を FREE にしていた。合成し直した結果は、b36b204 時点の `final.pgm` と 1 セルも違わない（バイト単位で同じ）ことを確かめた。b36b204 以後、旧 v2 のマスク・`v2_layers.yaml`・合成スクリプトは変わっておらず、`final.pgm` だけが直接の塗りで変わった。確かめるコマンド（下の「再現」の旧マスクでの合成の後）：

  ```bash
  cd ~/whill_lab0_ros2   # このブランチの checkout
  git show b36b204:docs/maps/campus/v2/final.pgm | cmp - /tmp/old/final_recomposed.pgm && echo "b36b204 と同一"
  ```
- `v2/candidate_final.pgm`、`occupancy_candidate.yaml`：候補のマスクで合成した地図。`nav_launch.py site:=campus-m8 map_variant:=candidate` で読める。`occupancy.yaml`（variant 無指定）は、確定するまで置かない。2026-11-13 に既定の site にしたので、variant を省いたときも nav_launch はこの候補を読む（`_DEFAULT_VARIANT_BY_SITE`）。確定して `occupancy.yaml` を置いたら、その対応と `start_dispatch_demo.sh`・`check_map_sync.py` の対応を消す。
- 2026-11-19 に、この節の候補を地理院の縁まで広げた版（154w）を `candidate` に採用した（下の 154w 節「採用」）。ここで作った手塗りのままの地図は `v2/candidate_hand_final.pgm`・`occupancy_candidate_hand.yaml`（`map_variant:=candidate_hand`）、その入力のマスクは `v2/candidates/candidate_{roadway,free}_merged_hand.png` として残した。下の「再現」の最後の合成は、出力を `candidate_final.pgm` に書くと広げた版を手塗りのままで上書きするので、作り直すときは `--output-pgm $D/v2/candidate_hand_final.pgm` に読み替え、入力も `*_hand.png` にする（`map_transfer_masks.py` も `candidate_*_merged.png` を上書きするので、作り直したら 154w 節の「採用」をやり直す）。

### 候補の合成の結果

この表は手塗りのままの版（2026-11-19 からは `candidate_hand`）の値。今の `candidate`（154w で広げた版）の値は 154w 節の「結果」の表。

| 項目 | 値 |
| --- | --- |
| 機械の障害物 | 401,771 セル（旧 v2 は 435,489） |
| 候補の free で消えた障害物 | 8,336 セル（旧の人手の塗りだけ 126、自動だけ 5,650、両方 2,560）。`v2/erased_by_free.png`（Git 管理外。合成のたびに作られる） |
| 最終の OCC / FREE / UNKNOWN | 0.94% / 4.94% / 94.11% |
| 配車4地点が FREE か | 4点とも FREE（新しい座標、下の waypoint 節） |
| 4地点が車体の幅でつながるか | つながる（FREE を 0.35 m 削った後の同じ連結成分。`scripts/map_route_check.py`） |
| 参考：手塗りなし（機械だけ） | 4点が別々の成分（1714 成分）、正門前は FREE の外。塗りが要ることの確認 |

```bash
scripts/map_route_check.py docs/maps/campus-m8/occupancy_candidate_hand.yaml docs/maps/campus-m8/waypoints.yaml
```

つながらない（`all_connected: false`）と終了コード 1 で終わる。これは「全体の経路が計画できる」の代わりの必要条件で、Nav2 の planner は動かしていない（コストの膨張、smoother、controller は見ていない）。Nav2 の global planner を地図だけで動かした結果は 154w 節の「Nav2 の planner での確認」（`scripts/nav2_plan_check.py`）。

### ユーザーが確かめること

1. **障害物を消していないか**：`v2/erased_by_free.png` の 8,336 セル。旧地図で塩として消した場所でも、新しい点群では本物の縁石・柱かもしれない。特に旧の人手の塗りで消えたもの（黄と白）。
2. **塗りの縁がずれていないか**：地図間の局所差は水平 p95 0.27 m。縁石・壁ぎわの roadway の縁が、新しい `occupied_step.png` / `occupied_structure.png` の外側に出ていないか。`candidate_review_overlay.png` を GIMP で機械レイヤと重ねて見る。
3. **直接の拡幅（緑）**：旧地図で場当たり的に足した帯（seimon 付近、x −66〜−42 の北側など）を、正式にマスクとして残すか。残すなら roadway と free の両方に入れる。
4. 確定したら、確定版を `v2/roadway_mask.png`、`v2/free_mask.png`、`v2/keepout_mask.png` として置き、`gen_auto_free_mask.py` と `compose_occupancy.py` で `v2/final.pgm` を作る（手順は [v2 塗り手順](../v2-paint-guide.md)。パスを campus-m8 に読み替える）。その後 `occupancy.yaml` と Web の背景（`scripts/m7_make_web_map.py`）を作る。

### 再現

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout
D=docs/maps/campus-m8; OV=docs/maps/campus/v2
mkdir -p /tmp/old /tmp/new
scripts/pcd_to_occupancy_v2.py --input-pcd $D/static.pcd --input-yaml docs/maps/campus/occupancy_cleaned.yaml \
  --traj $D/traj_lidar.txt --output-dir $D/v2 --cache-npz $D/.v2_cache.npz
# 旧地図の直接の塗りを取り出すため、旧マスクで合成し直す
scripts/gen_auto_free_mask.py --layers-yaml $OV/v2_layers.yaml --roadway-mask $OV/roadway_mask.png \
  --output-free-mask-auto /tmp/old/free_mask_auto.png --output-salt-csv /tmp/old/salt.csv
scripts/compose_occupancy.py --layers-yaml $OV/v2_layers.yaml --roadway-mask $OV/roadway_mask.png \
  --free-mask $OV/free_mask.png --free-mask-auto /tmp/old/free_mask_auto.png --keepout-mask $OV/keepout_mask.png \
  --output-pgm /tmp/old/final_recomposed.pgm
scripts/map_transfer_masks.py --old-layers-yaml $OV/v2_layers.yaml --new-layers-yaml $D/v2/v2_layers.yaml \
  --transform $D/align_from_campus/T_new_old.txt \
  --mask roadway_mask=$OV/roadway_mask.png --mask free_mask=$OV/free_mask.png --mask keepout_mask=$OV/keepout_mask.png \
  --old-final $OV/final.pgm --old-recomposed /tmp/old/final_recomposed.pgm \
  --out-dir $D/v2/candidates --force
# 上の map_transfer_masks.py が merged（roadway ∪ 直接の拡幅、free ∪ 直接の拡幅）も書く
scripts/gen_auto_free_mask.py --layers-yaml $D/v2/v2_layers.yaml --roadway-mask $D/v2/candidates/candidate_roadway_merged.png \
  --output-free-mask-auto /tmp/new/free_mask_auto.png --output-salt-csv /tmp/new/salt.csv
scripts/compose_occupancy.py --layers-yaml $D/v2/v2_layers.yaml \
  --roadway-mask $D/v2/candidates/candidate_roadway_merged.png --free-mask $D/v2/candidates/candidate_free_merged.png \
  --free-mask-auto /tmp/new/free_mask_auto.png --keepout-mask $D/v2/candidates/candidate_keepout_mask.png \
  --output-pgm $D/v2/candidate_final.pgm
# 確認用の重ね絵：同じ map_transfer_masks.py を --new-final $D/v2/candidate_final.pgm を足してもう一度
```

### 縁石からの自動抽出の試作（#202）

**結論：手塗りの白リストは置き換えない（手塗りを使い続ける）。** 自動の走行可能域は、軌跡の近くの手塗りを 94% 覆い、配車4地点のうち3点を手塗りなしでつなげた。一方で、自動だけの所が手塗りと同じくらいある（precision 0.49）。原因は縁石の線が途切れることで、その元は static.pcd の z が 10 cm 刻みなこと。点群を細かくしないと、パラメータの調整では詰めきれない。既定の地図（`candidate_final.pgm`）は変えていない。

`scripts/map_roadway_from_curbs.py` で作った。入力は static.pcd と traj_lidar.txt、v2 の構造物層だけで、実機は使わない。手順は次の5つ。

1. 地面の高さ：各セルについて、局所最小（半径 0.5 m）から 0.5 m 以内の点の下位 5% を地面とする。点の無いセルは 0.3 m 以内なら最も近い値で埋め、中央値（5×5）でならす。
2. 縁石：0.5 m の窓の中で高さの幅が 8〜25 cm、勾配が 0.4 以上、0.8 m の平均の面でも幅が 5 cm 以上のセルを縁石とする。構造物層（地面から 0.1〜2.2 m）のあるセルは除く。幅が 25 cm を超える所は「高い段」とし、縁石の線には入れないが、通れない所に入れる。
3. 走行可能域：軌跡（1 m おき）の種から、地面が見えていて縁石・高い段・構造物でないセルを、4 近傍で領域成長する。軌跡から 8 m までに限る。成長は 0.25 m 削った域で行い、後で戻す（縁石の線の 0.5 m 未満の切れ目から漏らさないため）。最後に closing 0.6 m、opening 0.2 m、2 m² 未満の穴埋めで整える。
4. 手塗りの候補（`candidate_roadway_mask.png`）と比べる。
5. README §3 の「再現」と同じ合成（`gen_auto_free_mask.py` → `compose_occupancy.py`）を、手塗りの roadway の代わりに自動の roadway で行い、`map_route_check.py` で4地点を確かめる。

#### 分かったこと：static.pcd は 10 cm のボクセルの中心

static.pcd の 7,614,632 点は、x・y・z のどれも 0.05 + 0.1k の値しか取らない（全点で確認）。DUFOMap が 0.1 m のボクセルの中心を出しているため。この性質から次の3つが起きる。

- 5 cm 格子のセルには、高さ 10 cm ごとに高々 1 点しか入らない。v2 の `min_points` 3 で数えると、地面が見えたセルは 118,202 しかない。そこで 1 点で「見えた」とした（1,650,306 セル）。
- 縁石の高さも 10 cm 刻みになる。10 cm 前後の縁石は、ちょうど 1 段に見える。
- 路面の高さが量子化の境目にあると、隣のセルと 10 cm ずつ上下にばらばらになる。発進点〜bldg3 で確かめた（例：x 25 m、y 2 m 付近では −0.65 と −0.75 が入り混じる）。この揺れと 1 段の縁石は、どちらも隣と 10 cm 違うだけで形が同じ。平均の面の条件（手順 2）でこの揺れの多くは落ちるが、現物では揺れも平均して 4〜5 cm 残る所がある。一方、10 cm の縁石は 5.9 cm になる。両者がこれだけ近いので、縁石の線が途切れるか、道の中に偽の縁石が残るかのどちらかになる。

#### 結果（既定のパラメータ）

| 項目 | 値 |
| --- | --- |
| 自動 / 手塗り / 両方のセル数 | 3,106,791 / 1,793,532 / 1,533,620 |
| IoU / precision / recall（全体） | **0.456 / 0.494 / 0.855** |
| recall（軌跡から 8 m 以内の手塗りだけ。1,639,378 セル） | 0.936 |
| 発進点〜bldg3（x −10〜70、y −15〜15） | IoU 0.470、precision 0.482、recall 0.948 |
| seimon（x −100〜−75、y −20〜5） | IoU 0.411、precision 0.411、recall 1.000 |
| 北の道（x −125〜75、y 145〜175） | IoU 0.630、precision 0.676、recall 0.902 |
| 縁石 / 高い段 / 構造物のセル数 | 1,138,806 / 1,893,764 / 401,771 |
| 種のうち通れたもの | 1,250 / 1,252 |
| 所要時間（合成と経路の確認を含む） | 41 秒（地面 3 秒、縁石 2 秒、成長 3 秒、合成 3 回で 24 秒）。最大メモリ 3.0 GB |

手塗りの 154,154 セル（8.6%）は軌跡から 8 m より遠い。主に中央の南北の道で、今回の bag では走っていない。軌跡を種にする限り、ここは取れない。

合成と経路の確認（`map_route_check.py`：0.35 m 削った FREE の連結）：

| 変種 | roadway | free（塩の消去） | FREE のセル数 | 4地点 |
| --- | --- | --- | --- | --- |
| 参考：今の候補 | 手塗り（merged） | 手塗り（merged） | 2,066,589 | つながる |
| A | 自動 | なし（完全に自動） | 3,028,723 | **つながらない**。seimon は FREE だが、0.35 m 削ると 0.5 m 以内に FREE が残らない。B で手塗りの free を足すとつながるので、機械の障害物（塩）で狭まっていると見られる（未確認）。他の3点は同じ成分 |
| B | 自動 | 手塗り（merged） | 3,034,428 | つながる |
| C | 自動 ∪ 手塗り | 手塗り（merged） | 3,251,688 | つながる |

B がつながるのは、手塗りの free（旧地図で seimon 前の塩を消した塗り）があるから。roadway だけを自動にすれば、経路の条件は満たす。ただし B の FREE は今の候補の 1.47 倍あり、増えた分は確かめていない。

#### 効いたこと

- 地面の面は 3 秒で作れる。塊ごとに読み、セルごとの百分位は並べ替えと添字で取るので、7.6 M 点でも最大メモリは 3 GB に収まる。
- 1/4 の図（`curbs_over_maxheight_q.png`）では、多くの道で両側の縁に沿って縁石の線が出る。#202 の発端の「段差層は縁石を線として取れていない」に対しては、線としては取れている。
- 軌跡から 8 m 以内なら、手塗りの 94% を覆う。北の道は IoU 0.63 で、手塗りに近い。
- 軌跡の真下は LiDAR の死角で地面の点が無い。そこで、種を 1 m 以内の通れるセルに寄せた。これで種はほぼ全部（1,250/1,252）が使える。

#### 効かなかったこと

- **縁石の線が途切れ、そこから漏れる。** 発進点〜bldg3 の南側（`start_crop_1to1.png`）は、縁石の切れ目から歩道・植栽の手前まで広がる。漏れを止めているのは 8 m の上限で、縁石ではない。precision が 0.5 前後にとどまるのはこのため。
- **坂の等高線は縁石と区別できない。** 2% の坂でも 5 m ごとに 10 cm の段が出る。局所的には縁石と同じ形なので、平均の面の条件では落ちない（`scripts/tests/test_roadway_curbs.py` の冒頭に記録）。道を横切る向きの線になることが多く、種が 1 m おきにあるので成長はあまり妨げない。それでも縁石の図には残る。
- **漏れ止めと覆う率の両立。** 掃引の数値は下表。切れ目の幅（gap）を大きくすると、道の中の偽の縁石で道が切れる。平均の面の条件を緩めると、偽の縁石が増える。
- 手塗りの縁は旧地図から写したもの（局所差 p95 0.27 m）。自動の縁のほうが 1 m ほど外の所もある（発進点の北側。自動は縁石の線で止まっている）。どちらが正しいかは、地図だけでは決まらない。

掃引（同じ地面の面で、縁石と成長だけを変えた。IoU / precision / recall、「近」は軌跡から 8 m 以内の recall、「4点」は roadway を 0.35 m 削った連結で見た目安）：

| hole-fill | trend 窓 / 幅 | gap | max-dist | close | IoU | P | R | 近 | 4点 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0.15 | 1.0 / 0.04 | 0.35 | 12 | 0.3 | 0.430 | 0.736 | 0.509 | — | — |
| 0.15 | 1.0 / 0.04 | 0.5 | 12 | 0.3 | 0.293 | 0.807 | 0.316 | — | — |
| 0.15 | 1.0 / 0.04 | 1.0 | 12 | 0.3 | 0.064 | 0.881 | 0.065 | — | — |
| 0.15 | 1.0 / 0.05 | 0.35 | 12 | 0.6 | 0.458 | 0.615 | 0.643 | 0.696 | — |
| 0.15 | 1.0 / 0.05 | 0.35 | 8 | 0.6 | 0.471 | 0.639 | 0.642 | 0.702 | 2点（seimon・bldg3 が外） |
| 0.15 | 1.0 / 0.05 | 0.35 | 6 | 0.6 | 0.485 | 0.670 | 0.638 | — | — |
| 0.3 | 1.0 / 0.05 | 0.35 | 8 | 0.6 | 0.438 | 0.482 | 0.829 | 0.907 | 3点（bldg3 が外） |
| 0.3 | 1.0 / 0.05 | 0.25 | 8 | 0.6 | 0.430 | 0.457 | 0.878 | 0.961 | 4点 |
| 0.3 | 1.0 / 0.04 | 0.25 | 8 | 0.6 | 0.459 | 0.506 | 0.832 | 0.910 | 4点 |
| **0.3** | **0.8 / 0.05** | **0.25** | **8** | **0.6** | **0.456** | **0.494** | **0.855** | **0.936** | **4点**（合成後は上の表） |
| 0.3 | 0.8 / 0.06 | 0.25 | 8 | 0.6 | 0.426 | 0.452 | 0.880 | 0.963 | 4点 |
| 0.3 | 平均の面の条件なし | 0.25 | 8 | 0.6 | 0.336 | 0.774 | 0.373 | 0.408 | ばらばら |

既定を trend 窓 0.8 m / 幅 0.05 m にしたのは、10 cm の段を合成データで縁石として検出できる組み合わせだから。1.0 m / 0.05 m では幅が 4.8 cm になり、1 段の縁石が一つも出ない。IoU は 1.0 m / 0.04 m とほぼ同じ。

#### 推奨

1. **手塗りを使い続ける。** 自動の roadway で置き換えると、手塗りより FREE が 1.47 倍に増える。増えた所は縁石の切れ目からの漏れで、歩道・植栽の手前・広場を含む。この増えた所を人が確かめないまま Nav2 の地図にするのは、手塗りより危ない。併用（自動 ∪ 手塗り、変種 C）も同じ理由で地図には使わない。
2. 自動の結果は、手塗りを確定するとき（3 節「ユーザーが確かめること」の 2）の点検に使える。軌跡から 8 m 以内で手塗りだけの所（`roadway_auto_vs_hand_q.png` の青）は、手塗りが縁石・構造物を越えている候補になる。
3. 置き換えを再び考えるなら、まず 10 cm の量子化を外す。GLIM の出力を DUFOMap にかける前の点、または DUFOMap を細かいボクセルで回した点から、同じスクリプト（`--pcd`）で作り直す。そのうえで、同じ3つの数（IoU、4地点、縁石の線の目視）で比べる。今の static.pcd のままパラメータを詰めても、IoU は 0.43〜0.49 の幅から出なかった。（2026-11-16 に試した。下の「DUFOMap の出力解像度を変えた再試行」）

外部の参照（国土地理院の道路）との比較は [reference-gsi.md](reference-gsi.md)（#205）。自動の roadway の 3 分の 1 が GSI の道路の外にあり（precision 0.67）、手塗りは 93% が中にある。上の推奨と食い違わない。

#### 置き場

| ファイル | 中身 | Git |
| --- | --- | --- |
| `v2/experiments/202/roadway_auto.png` | 自動の roadway（原寸、手塗りの候補と同じ形式。`compose_occupancy.py --roadway-mask` にそのまま渡せる） | 管理 |
| `v2/experiments/202/curb_mask.png` | 縁石の候補（原寸） | 管理 |
| `v2/experiments/202/curbs_over_maxheight_q.png` | 1/4。最大高さの下絵に縁石（赤）、高い段（暗い紫） | 管理 |
| `v2/experiments/202/roadway_auto_vs_hand_q.png` | 1/4。両方（白）、自動だけ（赤）、手塗りだけ（青） | 管理 |
| `v2/experiments/202/start_crop_1to1.png` | 原寸。x −10〜70、y −15〜15。自動（緑）、手塗りの縁（青）、縁石（赤）、高い段（紫）、軌跡（黄） | 管理 |
| `v2/experiments/202/summary.json` | パラメータ、数値、経路の結果、所要時間 | 管理 |
| `~/bags/m8-experiments/202/{A_auto,B_auto_handfree,C_union_handfree}/final.{pgm,yaml}` | 合成した地図（各 42 MB）と監査の PNG | 管理外 |
| `~/bags/m8-experiments/202/diff_auto_vs_hand.png`, `roadway_union.png` | 原寸の差分、変種 C の roadway | 管理外 |

再現（約 40 秒。何度でも同じ結果になる。PNG のバイト列が同じことを確かめた）：

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout。static.pcd は main 側の正本を自動で探す
scripts/map_roadway_from_curbs.py --compose
# 合成した地図の経路の確認だけをやり直す
scripts/map_route_check.py ~/bags/m8-experiments/202/B_auto_handfree/final.yaml docs/maps/campus-m8/waypoints.yaml
```

#### DUFOMap の出力解像度を変えた再試行（2026-11-16）

**結論：手塗りを使い続ける（推奨は変わらない）。** 10 cm の格子を外すと、道の中の偽の縁石はほぼ消え、発進点〜bldg3 の南の縁石が線としてつながった（目視）。ただし数値の改善は小さい（IoU 0.456 → 0.471、precision 0.494 → 0.524、B の FREE は手塗りの 1.47 倍 → 1.34 倍）。配車4地点は今回も手塗りの free が無いとつながらない（A は seimon が外）。縁石の線の点検には、今回の調整版（R1 調整）を 10 cm 版の代わりに使える。

上の「推奨」の 3 を試した。同じ GLIM の実行（`glim-m8-hires-noise-local/glim/`）を、DUFOMap にもう一度かけた。`m5r_run_dufomap.sh` に2つの引数を足した。既定（0.1 m、ボクセルの中心）は変えていないので、正本の手順は今までと同じ呼び出しになる。

- `--output-points raw`：DUFOMap の `outputMap(..., voxel_map=False)`。静的と判定されたボクセルに入った元の点をそのまま書く。消すボクセルの判定は voxel と同じで、書く点だけが変わる。
- `--resolution M`：DUFOMap のボクセルの大きさ。

| 変種 | resolution | 出す点 | 点数 | 格子に乗る点の割合 | 所要（変換 + DUFOMap） | 最大メモリ |
| --- | --- | --- | --- | --- | --- | --- |
| 正本（2 節） | 0.1 | ボクセルの中心 | 7,614,632 | 100%（x・y・z とも 0.05 + 0.1k） | 57 s | — |
| R1 | 0.1 | 元の点 | 13,170,437（入力 13,208,521 の 99.71%） | 0.20%（一様なら 0.20%） | 40 s | 2.4 GB |
| R2 | 0.05 | 元の点 | 13,184,845（同 99.82%） | 0.20%（0.025 + 0.05k でも 0.40%、一様の値） | 56 s | 4.6 GB |

- 「格子に乗る点の割合」は、座標が 0.05 + 0.1k から 0.1 mm 以内の点の割合。一様に散らばっていれば 0.2% になる。0.1 で割った余りは、0.1 mm 刻みで 1,000 通りすべてに出る（正本は 1 通り）。R1・R2 とも格子から外れている。
- R1 の点を 0.1 m のボクセルに戻すと、正本の 7,614,632 ボクセルと 1 つ残らず一致した。動的除去の結果は正本と同じで、違うのは点の出し方だけ。
- R2（0.05 m）は 20 GB の上限に遠く届かなかった（最大 4.6 GB）。

**分かったこと：元の点も疎い。** GLIM のサブマップは 0.1 m で間引かれている（`config_sub_mapping_gpu.json` の `submap_downsample_resolution` 0.1）。元の点でも、地面の帯に入る点は 365 万点、地面が見えた 5 cm のセルは 264 万で、1 セルに平均 1.4 点しかない。そのため `min_points` は 1 のままにした（3 では 19.5 万セル、2 でも 62 万セルしか残らず、道が切れる。下の掃引）。また、R1 と R2 の結果はほぼ同じになった。DUFOMap の解像度は消すボクセルの判定にしか効かず、書かれる点はどちらも同じ GLIM の点だから。

##### 結果（10 cm 版と並べて）

「調整」は `--trend-dz-min 0.03 --curb-dz-min 0.05`（他は #202 の既定）。量子化の揺れ（隣のセルと 10 cm ずつ上下する）が無くなったので、平均の面の条件を 0.05 → 0.03 m に緩めても道の中に偽の縁石が出ない。縁石の下限 0.08 → 0.05 m は、5〜8 cm の低い縁石（10 cm 版では 1 段に丸められていた）を拾うため。

| 項目 | 10 cm 版（上の結果） | R1 既定 | **R1 調整** | R2 既定 | R2 調整 |
| --- | --- | --- | --- | --- | --- |
| IoU / precision / recall（全体） | 0.456 / 0.494 / 0.855 | 0.419 / 0.444 / 0.881 | **0.471 / 0.524 / 0.822** | 0.419 / 0.444 / 0.881 | 0.471 / 0.524 / 0.826 |
| recall（軌跡から 8 m 以内） | 0.936 | 0.963 | 0.899 | 0.963 | 0.903 |
| 発進点〜bldg3 の IoU / precision | 0.470 / 0.482 | 0.434 / 0.442 | **0.559 / 0.572** | 0.434 / 0.442 | 0.559 / 0.572 |
| seimon / 北の道の IoU | 0.411 / 0.630 | 0.391 / 0.586 | 0.411 / 0.601 | 0.391 / 0.586 | 0.409 / 0.600 |
| 自動の roadway のセル数 | 3,106,791 | 3,554,447 | 2,812,871 | 3,554,327 | 2,828,455 |
| 縁石 / 高い段のセル数 | 1,138,806 / 1,893,764 | 1,014,497 / 1,994,832 | 1,590,234 / 2,115,500 | 1,014,652 / 1,995,116 | 1,590,069 / 2,116,434 |
| A（自動だけ）の4地点 | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） |
| B（自動 + 手塗りの free）の4地点 | つながる | つながる | つながる | つながる | つながる |
| B の FREE のセル数（今の候補 2,066,589 の何倍か） | 3,034,428（1.47） | 3,419,850（1.65） | **2,761,562（1.34）** | 3,419,706（1.65） | 2,777,136（1.34） |
| C（自動 ∪ 手塗り + 手塗りの free） | つながる | つながる | つながる | つながる | つながる |
| 所要 / 最大メモリ（抽出 + 合成） | 41 s / 3.0 GB | 41 s / 3.0 GB | 45 s / 3.0 GB | 41 s / 3.0 GB | 42 s / 3.0 GB |

- A で seimon が外れるのは 10 cm 版と同じ。seimon の点は FREE だが、0.35 m 削ると 0.5 m 以内に FREE が残らない。
- 既定のまま元の点にすると、かえって悪くなる（IoU 0.419）。既定の平均の面の条件（0.05 m）は 10 cm の量子化に合わせた値で、低い縁石まで落としてしまい、南の広場への漏れが大きくなる。
- **対照**：同じ調整を 10 cm の正本にかけると、道の中の偽の縁石で道が切れる（IoU 0.493、precision 0.672、recall 0.649、roadway を 0.35 m 削った連結で seimon が外、他の3点も 2 成分に割れる）。調整が効くのは、点が格子から外れたから。

掃引（R1、合成なし。「4点」は roadway を 0.35 m 削った連結で見た目安、数字は各地点の成分の番号で、0 は 0.5 m 以内に通れるセルが無い）：

| 変えたもの | IoU | P | R | 近 | 発進点の IoU / P | 4点 |
| --- | --- | --- | --- | --- | --- | --- |
| 既定 | 0.419 | 0.444 | 0.881 | 0.963 | 0.434 / 0.442 | つながる |
| min-points 3 | 0.160 | 0.785 | 0.168 | 0.184 | 0.431 / 0.703 | [91, 0, 0, 90] |
| min-points 5 | 0.005 | 0.882 | 0.005 | 0.005 | 0.006 / 0.811 | 全部 0 |
| trend 窓 0.5 | 0.444 | 0.475 | 0.871 | 0.953 | 0.440 / 0.448 | つながる |
| trend 幅 0.03 | 0.465 | 0.507 | 0.849 | 0.928 | 0.538 / 0.549 | つながる |
| trend 幅 0.025 | 0.463 | 0.519 | 0.813 | 0.889 | 0.541 / 0.553 | つながる |
| trend 幅 0.02 | 0.460 | 0.528 | 0.781 | 0.854 | 0.542 / 0.554 | つながる |
| 平均の面の条件なし | 0.460 | 0.547 | 0.743 | 0.812 | 0.539 / 0.558 | [42, 42, 42, 0]（bldg3 が外） |
| **trend 幅 0.03 + 縁石の下限 0.05** | **0.471** | **0.524** | **0.822** | **0.899** | **0.559 / 0.572** | **つながる** |
| trend 幅 0.03 + gap 0.35 | 0.465 | 0.537 | 0.777 | 0.850 | 0.560 / 0.582 | [32, 32, 32, 0]（bldg3 が外） |
| trend 幅 0.03 + 中央値 3×3 | 0.485 | 0.581 | 0.745 | 0.815 | 0.581 / 0.596 | つながる |
| trend 幅 0.03 + hole-fill 0.2 | 0.477 | 0.524 | 0.842 | 0.921 | 0.541 / 0.553 | つながる |
| trend 幅 0.03 + max-dist 6 | 0.524 | 0.582 | 0.841 | 0.928 | 0.617 / 0.640 | [2, 2, 0, 2]（seimon が外） |
| 縁石の下限 0.05（trend 幅は既定） | 0.423 | 0.449 | 0.880 | 0.963 | 0.435 / 0.443 | つながる |

R2 でも同じ傾向（既定 0.419、trend 幅 0.03 で 0.464、trend 幅 0.03 + 縁石の下限 0.05 で 0.471、min-points 2 は 0.453 だが seimon が外れる）。max-dist 6 は IoU が最も高いが、seimon（軌跡から 7.2 m）が届かなくなるので採らない（10 cm 版と同じ理由）。中央値 3×3 は precision が上がるが recall が 0.745 に下がるので、既定（5×5）のままにした。

##### 目視（`202b/R1_tuned/start_crop_1to1.png`、x −10〜70、y −15〜15）

- **道の中の偽の縁石が消えた。** 10 cm 版の crop では車道の中に短い赤い線が散っていた。R1 では既定でも調整でも、軌跡の周りの車道に赤い線はほぼ無い。
- **南の縁石が線としてつながった（調整）。** x −10〜30 では、車道の南の縁（手塗りの縁の青い線と同じ所、y ≈ −1.5）に赤い線がほぼ切れ目なく出る。10 cm 版と R1 既定で南へ 6〜7 m 広がっていた漏れ（x −5〜30、y −2〜−8）は止まった。北側の縁、北へ延びる道（x ≈ 23〜27）の両側も、手塗りの縁に沿う。
- **漏れは残る。** x 30〜45 で、植栽の間（y −2〜−7）に入り込む。左上（x −10〜−2、y 3〜15）の広場にも広がる（10 cm 版と同じ）。
- **段の無い縁は点群の高さでは取れない。** R1 既定で南へ漏れた所の断面を元の点で見た（0.2 m 幅の帯の下位 5% の高さ）。x 15 では、y −4 から −1.2 まで 12 cm ゆるやかに下がるだけで段が無い。x 35 では、y −2.8 付近に幅 0.3 m・高さ 0.1〜0.25 m の細い出っ張りがあり、その南はまた車道と同じ高さ。手塗りの南の縁（y ≈ 0〜−1）の所には段が無い。手塗りの縁の一部は、高さの段ではない境目（舗装の違い、白線など）で引かれている。
- 全体の 1/4 の図（`roadway_auto_vs_hand_q.png`）では、多くの道で自動の帯が手塗りの両側に少しずつはみ出し（赤の縁）、出入口・駐車場の所で横へ張り出す。手塗りだけの所（青）は、主に軌跡から遠い中央の南北の道。

##### 推奨（更新）

1. **手塗りを使い続ける。** 自動の roadway で置き換えると、FREE が手塗りの 1.34 倍になる。増えた所は確かめておらず、植栽の間・広場・出入口を含む。A（完全に自動）は配車4地点をつながない。
2. **併用はしない。** 自動 ∪ 手塗り（C）は4地点をつなぐが、FREE が 1.48 倍（3,054,796）になり、上と同じ理由で地図には使わない。
3. **点検には R1 調整版を使う。** 手塗りを確定するとき（3 節「ユーザーが確かめること」の 2）、`202b/R1_tuned/curb_mask.png` の縁石の線を手塗りの縁と重ねると、手塗りが縁石を越えている所を見つけられる。10 cm 版（`202/`）より線が連続し、道の中の偽の線が少ない。
4. **さらに進めるなら**、次の2つ。どちらもこの試作の範囲を超える。
   - 点の密度：GLIM の `submap_downsample_resolution` を 0.1 より小さくして回し直すか、bag の各スキャンを GLIM の軌跡で直接重ねる。GLIM の GPU 実行は非決定的（2 節）なので、回し直した地図は正本と少し違う。マスクだけを作るのに使い、正本との位置合わせを確かめる必要がある。
   - 段の無い縁：高さだけでは取れない。反射強度（GLIM の `intensities_compact.bin`）で舗装の違い・白線を見るか、そこは手塗りに任せる。
5. 正本の static.pcd・既定の DUFOMap の出力（ボクセルの中心）は変えない。localizer と占有格子の入力は今のまま。

##### 置き場

| ファイル | 中身 | Git |
| --- | --- | --- |
| `v2/experiments/202b/R1_tuned/` | R1 調整：`roadway_auto.png`、`curb_mask.png`（原寸）、確認用の3枚（`curbs_over_maxheight_q.png`、`roadway_auto_vs_hand_q.png`、`start_crop_1to1.png`）、`summary.json` | 管理 |
| `v2/experiments/202b/{R1_default,R2_default,R2_tuned}/` | 同じ。ただし `curb_mask.png` は省いた（R1 調整とほぼ同じで、各 1.3〜1.8 MB あるため。下の再現で作り直せる） | 管理 |
| `~/bags/m8-experiments/202b/{R1_res0.1_raw,R2_res0.05_raw}/static.pcd` | DUFOMap の元の点（各 158 MB）と `staging/` | 管理外 |
| `~/bags/m8-experiments/202b/{R1,R2}*/roadway_{default,tuned}/` | 合成した地図（A・B・C）と原寸の差分 | 管理外 |
| `~/bags/m8-experiments/202b/{R1,R2}.log` | DUFOMap の実行記録（`/usr/bin/time -v` つき） | 管理外 |

再現（DUFOMap 各 1 分弱、抽出と合成は各 45 秒）：

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout
B=~/bags/m5r-bench-data/2026-07-10-campus-outer-final
W=~/bags/m8-experiments/202b
E=docs/maps/campus-m8/v2/experiments/202b
scripts/m5r_run_dufomap.sh $B/glim-m8-hires-noise-local/glim $W/R1_res0.1_raw --resolution 0.1 --output-points raw
scripts/m5r_run_dufomap.sh $B/glim-m8-hires-noise-local/glim $W/R2_res0.05_raw --resolution 0.05 --output-points raw
scripts/map_roadway_from_curbs.py --pcd $W/R1_res0.1_raw/static.pcd --out-dir $E/R1_default --work-dir $W/R1_res0.1_raw/roadway_default --compose
scripts/map_roadway_from_curbs.py --pcd $W/R1_res0.1_raw/static.pcd --out-dir $E/R1_tuned --work-dir $W/R1_res0.1_raw/roadway_tuned --compose --trend-dz-min 0.03 --curb-dz-min 0.05
scripts/map_roadway_from_curbs.py --pcd $W/R2_res0.05_raw/static.pcd --out-dir $E/R2_default --work-dir $W/R2_res0.05_raw/roadway_default --compose
scripts/map_roadway_from_curbs.py --pcd $W/R2_res0.05_raw/static.pcd --out-dir $E/R2_tuned --work-dir $W/R2_res0.05_raw/roadway_tuned --compose --trend-dz-min 0.03 --curb-dz-min 0.05
```

`m5r_run_dufomap.sh` は出力先に static.pcd があると止まる（作り直すときは `--force`）。GLIM の出力は読むだけで、正本の static.pcd には触れない。

#### bag のスキャンを軌跡で直接重ねた密な地面の面（2026-11-16）

**結論：手塗りを使い続ける（推奨は変わらない）。** 点の密度は 202b の数十倍になり、道の中の偽の縁石は消えた。しかし数値は良くならなかった（IoU 0.40〜0.44。202b の R1 調整は 0.471）。原因は密度ではなかった。手塗りの縁の多くには高さの段が無く、路面の高さだけでは縁を引けない。代わりに、**反射強度で舗装と芝・植栽の境がはっきり見える**（`202c/start_intensity_1to1.png`）。縁石の抽出を続けるなら、次は強度を使う。

202b の「推奨（更新）」の 4 の「bag の各スキャンを GLIM の軌跡で直接重ねる」を試した。GLIM は回し直していない。正本と同じ GLIM の実行の軌跡（`traj_lidar.txt`）を使うので、結果は正本の static.pcd・v2 の格子にそのまま揃う。

`scripts/map_ground_raster_from_bag.py` で作った。

- **フレームと時刻**：`traj_lidar.txt` は TUM 形式（t x y z qx qy qz qw）で、GLIM の world（= static.pcd と v2 の格子の map 座標）での LiDAR の姿勢。bag の `/velodyne_points` は header の時刻がスキャンの最後の点の時刻で、点ごとの `time` 欄は header からの相対（−0.100〜+0.0013 s）。GLIM の TimeKeeper は、`time` が負なら frame の時刻を header + min(time)（最初の点）にずらす（glim のソースで確認）。軌跡の範囲にあるスキャン 21,301 個のうち 21,300 個で、header + min(time) が軌跡の時刻と 1 ms 以内で一致した（差は 0.000 ms）。残る 1 個は軌跡の始まりにまたがるスキャン。
- **補間**：点の絶対時刻（header + time）で軌跡を補間する。並進は線形、回転は slerp。スキャン内を 50 の時間の区切りに分け、区切りの中央の姿勢を使う（2 ms。1 m/s で 2 mm）。これで走りながらのスキャンの歪みも取れる（GLIM は IMU で補正するが、ここでは隣り合う姿勢の間を等速とみなす）。軌跡の外のスキャン 10 個と、軌跡の外の点 24,775 は捨てた。軌跡の間隔は 0.101〜0.102 s で、0.25 s を超える穴は無い。
- **点の選び方**：センサから 0.5〜15 m、センサより 3 m 以上高くない点。さらに、正本の DUFOMap の static.pcd の 0.1 m ボクセルに入る点だけを残す（動的な物を地図と同じ判定で除く）。
- **集計**：v2 の 5 cm の格子で、セルごとに点数、最小、最大、下位 5%（セルの最小から 1 cm × 64 ビンの度数分布、ビンの中は一様とみなす）、地面の強度の平均（セルの最小から 0.10 m 以内の点）。

**静的ボクセルを 1 つ太らせた理由。** static.pcd のボクセルにそのまま入る点だけにすると、センサより 0.5 m 以上低い点（ほぼ地面）の 34% が落ちる。落ちた点の 6 割は、上下 ±1 m に静的ボクセルが 1 つも無い柱にあった。GLIM のサブマップは 0.1 m に間引いた後、`submap_target_num_points`（5 万点）へ無作為に減らすので、地面のボクセルが抜けた穴になる。各軸 ±1 ボクセル（3×3×3）太らせると、地面の点は 99.6% 残り、地面より上の点は 1.4% が落ちる。落ちるのは静的な物から 10 cm より離れた点、つまり人や動く車の胴体。足元の 10 cm は戻るが、地面の高さは下位百分位で取るので効かない（スキャン 95 個、全行程から 200 個おきで測った）。太らせない版（`--static-dilate 0`）も作り、点数だけを比べた（下表）。

| 項目 | 202b R1（DUFOMap の元の点） | **202c（±1 ボクセル、既定）** | 202c（太らせない） |
| --- | --- | --- | --- |
| 地面の面に使った点 | 365 万（地面の帯） | 2 億 9,513 万（距離内 2 億 9,817 万の 99.0%）。地面の帯の中は 1 億 9,359 万 | 2 億 2,088 万 |
| 点のあるセル | 264 万（地面） | 845 万 | 526 万 |
| 3 点以上のセル（最小から 0.64 m 以内の点で数える） | 19.5 万 | **709 万** | 449 万 |
| 10 点以上のセル | — | **480 万** | 313 万 |
| 所要 / 最大メモリ | DUFOMap 40 s / 2.4 GB | **2 分 48 秒 / 4.1 GB**（読み込みと変換 121 s、集計 33 s） | 2 分 8 秒 / 3.6 GB |

キャッシュ（`~/bags/m8-experiments/202c/cache/`、2.5 GB、500 スキャンごと）があれば、読み込みは飛ばして集計からやり直す。1 プロセス、numpy だけ。npz は 143 MB。下位 5% とセルの最小の差は、10 点以上のセルの中央値で 6 mm。

##### 結果（`--ground-npz` で縁石の抽出と合成）

`map_roadway_from_curbs.py --ground-npz` で、PCD の代わりにこの面を読む。下位 5% と点数（最小から 0.64 m 以内の点）をそのまま使い、局所最小からの帯（0.5 m）はセルの下位 5% に掛ける。「既定」は #204 の既定、「調整」は 202b の `--trend-dz-min 0.03 --curb-dz-min 0.05`。

| 項目 | 10 cm 版 | 202b R1 調整 | 3 点・既定 | 3 点・調整 | 10 点・既定 | **10 点・調整** |
| --- | --- | --- | --- | --- | --- | --- |
| IoU / precision / recall（全体） | 0.456 / 0.494 / 0.855 | **0.471 / 0.524 / 0.822** | 0.401 / 0.424 / 0.883 | 0.412 / 0.437 / 0.876 | 0.425 / 0.451 / 0.882 | 0.436 / 0.465 / 0.873 |
| recall（軌跡から 8 m 以内） | 0.936 | 0.899 | 0.966 | 0.959 | 0.965 | 0.956 |
| 発進点〜bldg3 の IoU / precision | 0.470 / 0.482 | **0.559 / 0.572** | 0.426 / 0.433 | 0.448 / 0.456 | 0.435 / 0.442 | 0.454 / 0.462 |
| seimon / 北の道の IoU | 0.411 / 0.630 | 0.411 / 0.601 | 0.384 / 0.572 | 0.388 / 0.563 | 0.402 / 0.587 | 0.403 / 0.591 |
| 地面が見えたセル / 埋めた後 | 1.65 M / — | — | 6.75 M / 9.98 M | 6.75 M / 9.98 M | 4.61 M / 8.28 M | 4.61 M / 8.28 M |
| 縁石 / 高い段のセル | 1,138,806 / 1,893,764 | 1,590,234 / 2,115,500 | 276,438 / 368,943 | 388,052 / 392,163 | 226,979 / 281,562 | 309,226 / 301,891 |
| 自動の roadway のセル | 3,106,791 | 2,812,871 | 3,734,374 | 3,595,860 | 3,508,494 | 3,369,418 |
| A（自動だけ）の4地点 | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） |
| B（自動 + 手塗りの free） | つながる | つながる | つながる | つながる | つながる | つながる |
| B の FREE のセル（今の候補の何倍か） | 3,034,428（1.47） | **2,761,562（1.34）** | 3,569,272（1.73） | 3,467,283（1.68） | 3,440,038（1.66） | 3,310,540（1.60） |
| C（自動 ∪ 手塗り） | つながる | つながる | つながる（1.80） | つながる（1.75） | つながる（1.74） | つながる（1.68） |
| 所要 / 最大メモリ（抽出 + 合成） | 41 s / 3.0 GB | 45 s / 3.0 GB | 41 s / 3.0 GB | 41 s / 3.0 GB | 41 s / 3.0 GB | 39 s / 3.0 GB |

- 縁石のセルは 202b の 1/5 以下になった。202b の縁石の大半は、疎らな点が芝・植栽の上で作る凹凸だったと見られる。密な面ではそれが無くなり、本当の段だけが残る。その分、段の無い縁から漏れて precision が下がる。
- 10 点以上のほうが 3 点以上より少し良い。点の少ないセル（15 m 近くの遠い芝）を埋めた値で扱うほうが、凹凸が出にくい。

##### 目視（`202c/mp10_tuned/start_crop_1to1.png`、x −10〜70、y −15〜15。比べる相手は `202b/R1_tuned/start_crop_1to1.png`）

- **道の中の偽の縁石は無い。** 202b R1 では芝・植栽の上に赤い短い線が一面に散っていた。202c では、段のある所（建物の縁、北へ延びる道 x ≈ 23〜27 の両側、北側の縁 x −1〜22）にだけ、切れ目の少ない線が出る。
- **x 15 付近の南の縁（坂）：つながらない。** 縁石の線は出ず、自動の roadway は x 7〜37 で南へ y ≈ −7.5 まで広がる。密な面の断面（x 15、0.25 m × 0.45 m の中央値）では、路面 −0.574 m（y −1.5）から芝 −0.511 m（y −2.0）へ 0.5 m で 6 cm、その先 2.5 m で 8 cm ゆるやかに上がる。勾配が 0.12 で縁石の条件（0.4）に届かない。202b の R1 調整でこの縁に出ていた線は、疎らな点の凹凸が並んだものと見られ、段ではない。同じ所の強度は 12 → 34 にはっきり変わる（舗装 → 芝）。
- **x 30〜45 の植栽の間：つながらない。** 自動の roadway は植栽の島の間を南へ y ≈ −6 まで入る。x 35 の断面では、y −2〜−5 の強度が 11〜17 で、路面（8〜13）と同じ舗装。植栽の間の斜めの舗装路に入っていて、高さでも強度でも路面とつながっている。手塗りはここを roadway に入れていない（通る道ではない、という人の判断）。
- **左上の広場（x −10〜−2、y 3〜15）：つながらない（10 cm 版・202b と同じ）。** 段が無く、強度でも舗装なので、高さでも強度でも区切れない。軌跡からの距離の上限（8 m）だけが止めている。
- x −10〜6 の南の縁（y ≈ −2.3）では止まる。x 0 の断面で、路面 −0.43 m が y −2.0〜−2.5 で 5 cm 上がり始める所に縁石の短い線が出る。

##### 反射強度（`202c/intensity_q.png`、`202c/start_intensity_1to1.png`）

bag の `/velodyne_points` には `intensity` がある（0〜255）。地面の点（セルの最小から 0.10 m 以内）の平均を見ると、舗装（暗い）と芝・植栽（明るい）の境が線としてはっきり出る。マンホール、舗装の継ぎ目、舗装の種類の違い（発進点の東の舗装の色の違い）も見える。白線は、この区間ではっきりしたものは見当たらなかった。

手塗りの roadway の中と、自動（10 点・調整）だけの所の強度（10 点以上のセル）：

| 区分 | 全体の中央値（p10〜p90） | 発進点〜bldg3 の中央値（p10〜p90） |
| --- | --- | --- |
| 手塗りと自動の両方 | 10.1（5.9〜15.0） | 10.1（6.9〜12.8） |
| 自動だけ（漏れ） | 13.2（7.0〜42.1） | 13.5（8.5〜37.2） |

強度 20 未満を舗装とすると、手塗りのセルの 98.3%（発進点〜bldg3 は 100%）が残り、漏れのセルの 27%（同 31%）が落ちる。漏れの残り 7 割は舗装の上（広場、斜めの舗装路、駐車場）で、強度でも区切れない。そこは「舗装だが通る道ではない」という人の判断。

##### 推奨（更新）

1. **手塗りを使い続ける。** 密な面でも、自動の roadway の FREE は手塗りの 1.60 倍以上になり、A は配車4地点をつながない。点検に使う縁石の線は、202b の R1 調整版より 202c（10 点・調整）の方が偽の線が少なく、段のある縁だけを示す。手塗りが**段**を越えている所を探すなら 202c の `mp10_tuned/curb_mask.png`、**舗装の縁**を越えている所を探すなら `start_intensity_1to1.png`・`intensity_q.png` を使う。
2. **高さだけで縁石を詰める作業はここで止める。** 密度を上げた結果、残る漏れの原因が「段の無い縁」だと分かった。パラメータでは直らない。
3. 自動化を続けるなら、領域成長の壁に「強度 20 以上（芝・植栽）」を足す試作が次の候補。上の数字から、漏れの 3 割を止め、手塗りの 98% を残す見込み（未試行）。残る広場・舗装路は人の判断なので、自動化の対象にしない。
4. 正本の static.pcd・既定の地図は変えていない。

##### 置き場

| ファイル | 中身 | Git |
| --- | --- | --- |
| `v2/experiments/202c/summary.json` | 密な面のパラメータ、点数、セル数、所要時間 | 管理 |
| `v2/experiments/202c/count_q.png` | 1/4。点数（対数、黒 = 0、白 = 1 万点） | 管理 |
| `v2/experiments/202c/intensity_q.png` | 1/4。地面の強度の平均（明るいほど強い。表示は p1〜p99 の 2.0〜72.8） | 管理 |
| `v2/experiments/202c/start_intensity_1to1.png`, `start_ground_1to1.png` | 原寸。x −10〜70、y −15〜15 の強度と地面の高さ（下位 5%、p2〜p98 で濃淡） | 管理 |
| `v2/experiments/202c/{mp3,mp10}_{default,tuned}/` | `--ground-npz` の抽出の結果（202b と同じ画像と `summary.json`）。`curb_mask.png` は調整の 2 つだけ（既定は下の再現で作り直せる） | 管理 |
| `~/bags/m8-experiments/202c/ground_raster.npz` | 密な面（count、count_low、z_min、z_p、z_max、intensity_mean、intensity_count、meta） | 管理外 |
| `~/bags/m8-experiments/202c/cache/` | 地図に置いた点（セル番号・z・強度、500 スキャンごと、2.5 GB） | 管理外 |
| `~/bags/m8-experiments/202c/count_log.png`, `run.log`, `roadway.log` | 原寸の点数、実行記録（`/usr/bin/time -v` つき） | 管理外 |
| `~/bags/m8-experiments/202c/strict_dilate0/` | 太らせない版の npz と記録 | 管理外 |
| `~/bags/m8-experiments/202c/roadway/*/` | 合成した地図（A・B・C）と原寸の差分 | 管理外 |

再現（密な面 約 3 分、抽出と合成は各 40 秒）：

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout
W=~/bags/m8-experiments/202c
E=docs/maps/campus-m8/v2/experiments/202c
scripts/map_ground_raster_from_bag.py                      # 既定で bag-hires・traj_lidar.txt・正本の static.pcd を読む
scripts/map_ground_raster_from_bag.py --static-dilate 0 --work-dir $W/strict_dilate0 --preview-dir $W/strict_dilate0/preview
scripts/map_roadway_from_curbs.py --ground-npz $W/ground_raster.npz --min-points 3 --out-dir $E/mp3_default --work-dir $W/roadway/mp3_default --compose
scripts/map_roadway_from_curbs.py --ground-npz $W/ground_raster.npz --min-points 3 --out-dir $E/mp3_tuned --work-dir $W/roadway/mp3_tuned --compose --trend-dz-min 0.03 --curb-dz-min 0.05
scripts/map_roadway_from_curbs.py --ground-npz $W/ground_raster.npz --min-points 10 --out-dir $E/mp10_default --work-dir $W/roadway/mp10_default --compose
scripts/map_roadway_from_curbs.py --ground-npz $W/ground_raster.npz --min-points 10 --out-dir $E/mp10_tuned --work-dir $W/roadway/mp10_tuned --compose --trend-dz-min 0.03 --curb-dz-min 0.05
```

`map_ground_raster_from_bag.py` は、キャッシュがあれば読み込みを飛ばす（中断しても 500 スキャン単位で続きから）。キャッシュを作ったときと引数が違うと止まる（作り直すときは `--force`）。bag と static.pcd は読むだけ。

##### 反射強度の壁を足した領域成長（2026-11-16）

**結論：強度の壁で、自動の roadway は初めて手塗りに近づいた（IoU 0.436 → 0.517、発進点〜bldg3 0.454 → 0.566）。GSI の道路の面（#205）に対しては、手塗りより IoU が高い（軌跡から 8 m 以内で 0.634 対 0.548）。それでも Nav2 の地図には手塗りを使い続ける。** 自動だけの所の 99% は舗装で、残る差は「舗装だが通る道ではない所（広場・構内の小道）を roadway に入れるか」という塗りの規則の違いになった。点群の処理の問題ではもう無い。A（完全に自動）は今回も seimon をつながない。

上の「推奨（更新）」の 4 を試した。`map_roadway_from_curbs.py --intensity-npz` で、密な面の地面の強度（`intensity_mean`）が `--max-intensity` 以上のセルを、芝・植栽として領域成長で通さない（縁石・高い段・構造物の壁に足す）。

- 点の少ないセル（地面の点 3 未満）は 0.3 m 以内なら近い値で埋める。それより遠い、強度の分からないセルは壁にしない。
- 中央値 5×5 でならし、壁は半径 0.2 m で opening する。幅 0.4 m より細い明るい線（白線・横断歩道の帯）で道が横に切れないようにするため。芝・植栽は面なので残る（単体テストで確認）。
- 他は 202c の「10 点・調整」（`--min-points 10 --trend-dz-min 0.03 --curb-dz-min 0.05`）と同じ。種は軌跡、上限は 8 m。
- GSI との比較は `scripts/map_roadway_vs_gsi.py`。#205 の `map_georef_gsi.py report` と同じ道路の面（GSI の道路縁と中心線から作る。0.1 m の格子）を、#205 の関数をそのまま呼んで作る。#205 の `metrics.json` の手塗りと 10 cm 版の数値（全体 IoU 0.2055 と 0.2273 など）と一致することを確かめた。

手塗りの候補と比べる（「近」は軌跡から 8 m 以内の手塗りの recall）：

| 項目 | 202b R1 調整 | 202c 10 点・調整 | 強度 < 15 | **強度 < 20** | 強度 < 25 |
| --- | --- | --- | --- | --- | --- |
| IoU / precision / recall（全体） | 0.471 / 0.524 / 0.822 | 0.436 / 0.465 / 0.873 | **0.534 / 0.612** / 0.806 | **0.517 / 0.560 / 0.872** | 0.505 / 0.545 / 0.873 |
| 近 | 0.899 | 0.956 | 0.882 | 0.954 | 0.955 |
| 発進点〜bldg3 の IoU / precision | 0.559 / 0.572 | 0.454 / 0.462 | **0.666 / 0.686** | 0.566 / 0.579 | 0.561 / 0.574 |
| seimon / 北の道の IoU | 0.411 / 0.601 | 0.403 / 0.591 | 0.431 / 0.680 | 0.410 / 0.611 | 0.408 / 0.595 |
| 強度の壁 / 自動の roadway のセル | — / 2,812,871 | — / 3,369,418 | 4,845,144 / 2,362,752 | 3,761,804 / 2,794,609 | 3,021,918 / 2,873,972 |
| A（自動だけ）の4地点 | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） | つながらない（seimon） |
| B（自動 + 手塗りの free） | つながる | つながる | つながる | つながる | つながる |
| B の FREE（今の候補 2,066,589 の何倍か） | 1.34 | 1.60 | **1.14** | 1.34 | 1.38 |
| C（自動 ∪ 手塗り） | つながる | つながる | つながる（1.29） | つながる（1.42） | つながる（1.46） |

GSI の道路の面と比べる（IoU / precision / recall。#205 と同じ区切り）：

| マスク | 全体 | 軌跡から 8 m 以内 | 発進点〜bldg3 | seimon | 北の道 |
| --- | --- | --- | --- | --- | --- |
| 手塗りの候補 | 0.206 / 0.933 / 0.209 | 0.548 / **0.927** / 0.573 | 0.636 / 1.000 / 0.636 | 0.157 / 1.000 / 0.157 | 0.561 / 0.998 / 0.561 |
| 10 cm 版（202） | 0.227 / 0.667 / 0.256 | 0.559 / 0.667 / 0.776 | 0.624 / 0.693 / 0.862 | 0.377 / 0.995 / 0.378 | 0.583 / 0.862 / 0.644 |
| 202b R1 調整 | 0.222 / 0.702 / 0.245 | 0.564 / 0.702 / 0.741 | 0.737 / 0.823 / 0.876 | 0.377 / 0.996 / 0.378 | 0.575 / 0.853 / 0.638 |
| 202c 10 点・調整 | 0.241 / 0.659 / 0.275 | 0.581 / 0.659 / 0.832 | 0.626 / 0.678 / 0.891 | 0.374 / 0.976 / 0.378 | 0.643 / 0.864 / 0.715 |
| 強度 < 15 | 0.219 / 0.793 / 0.232 | 0.594 / 0.793 / 0.703 | **0.809 / 0.954 / 0.842** | 0.362 / 1.000 / 0.362 | 0.587 / 0.912 / 0.622 |
| **強度 < 20** | 0.242 / 0.758 / 0.263 | **0.634** / 0.758 / 0.794 | 0.738 / 0.829 / 0.871 | 0.369 / 0.978 / 0.372 | 0.629 / 0.869 / 0.695 |
| 強度 < 25 | 0.246 / 0.751 / 0.267 | **0.638** / 0.751 / 0.809 | 0.741 / 0.827 / 0.877 | 0.371 / 0.978 / 0.374 | 0.642 / 0.867 / 0.712 |

- GSI の道路縁は歩道を含む幅で引かれ、手塗りはその 1 m ほど内側（#205）。手塗りは GSI に対して precision が高く recall が低い。強度の壁の自動は、縁が舗装の境（芝の手前）まで出るので recall が上がり、軌跡の近くの IoU は手塗りより高い。
- 閾値は 20 と 25 がほぼ同じ。15 にすると、少し明るい舗装（広場など）も壁になり、precision が上がって recall が下がる。手塗りに対する IoU は 15 が最も高いが、軌跡の近くの手塗りの recall が 0.882 に落ちる。手塗りのセルのうち強度 15 未満は 93.8%（20 未満は 99.7%）で、手塗りの道の一部も壁になる。

**舗装だが手塗りに入っていない所**（軌跡から 8 m 以内、強度で舗装と判定した面積。強度の分かる面は 16,509 m²）：

| 舗装の閾値 | 舗装 | 手塗り（うち舗装） | 舗装で手塗りでない | うち GSI の道路の面の中 | 強度 < 20 の自動が手塗りより広い所（うち舗装） |
| --- | --- | --- | --- | --- | --- |
| < 15 | 9,048 m² | 4,098 m²（93.8%） | 5,203 m² | 1,894 m² | 3,077 m²（2,632 m²） |
| **< 20** | **10,838 m²** | 4,098 m²（**99.7%**） | **6,751 m²** | **2,389 m²** | 3,077 m²（**3,056 m²、99.3%**） |
| < 25 | 11,925 m² | 4,098 m²（99.9%） | 7,832 m² | 2,560 m² | 3,077 m²（3,072 m²） |

- 軌跡の近くの舗装は手塗りの 2.6 倍ある。舗装で手塗りでない 6,751 m² のうち、2,389 m² は GSI の道路の面の中（道路の端・歩道の側）、残る約 4,360 m² は GSI でも道路ではない舗装（広場、駐車場、構内の小道）。
- 強度 < 20 の自動が手塗りより広い 3,077 m² は、ほぼ全部（99.3%）が舗装。芝・植栽への漏れは止まり、残る差は「舗装をどこまで roadway にするか」の規則の差。

名前を付けた 2 か所（箱の中の割合）：

| 箱 | 舗装（< 20） | GSI の道路 | 手塗り | 10 点・調整 | 強度 < 15 | 強度 < 20 | 強度 < 25 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 斜めの舗装路（x 30〜45、y −8〜−2） | 45.6% | 0% | 0% | 25.3% | 8.5% | 15.4% | 16.4% |
| 左上の広場（x −10〜−2、y 3〜15） | 99.6% | 3.8% | 0% | 67.0% | 3.3% | 67.0% | 67.0% |

- **斜めの舗装路**：舗装だが、手塗りにも GSI の道路の面にも入っていない（箱の中の舗装は 45.6%。残りは植栽の島）。強度 < 20 の自動は、車道から下りる入口の部分（x 33〜38、y −2〜−6）だけを含む。芝・植栽で区切られた先へは広がらない（`intensity_gate/mp10_tuned_i20/start_crop_1to1.png`）。
- **左上の広場**：ほぼ全面が舗装で段も無いので、強度 < 20・< 25 は軌跡から 8 m の上限まで全部を含む（67.0% は上限で切れた形）。手塗りにも GSI にも入っていない。強度 < 15 では外れる。広場の舗装の強度は中央値 12.5、上位 1/4 が 14.5 以上で、< 15 では広場の中に壁が散らばり、0.25 m 削った成長が入れない。
- 発進点〜bldg3 の南の縁（x 7〜37 の芝への漏れ）は、強度 < 20 で止まった。自動の縁は芝の手前（y ≈ −2.3）にあり、手塗りの縁（y ≈ −1.5）より 0.5〜0.8 m 外。#205 の「手塗りは GSI の道路縁の 1 m 内側」と同じ向きのずれ。

##### 推奨（2026-11-16、強度の壁の後）

1. **Nav2 の地図は手塗りのまま。** 理由は2つ。A（完全に自動）は配車4地点をつながない（seimon。手塗りの free が要る）。また、自動は手塗りより舗装を広く取る（B の FREE が 1.34 倍）。広く取った所は芝ではなく舗装だが、広場・構内の小道を走ってよいかは人が決める規則で、点群からは決まらない。
2. **点検には強度の壁の版（`intensity_gate/mp10_tuned_i20/`）を使う。** 縁石の線（202c）より、縁の位置の根拠が強い。手塗りの確定（3 節「ユーザーが確かめること」の 2）で、次の2つを見る。
   - 手塗りの縁を、舗装の縁（`start_crop_1to1.png` の暗い所の端）まで広げるか。今は GSI の道路縁・舗装の縁より 0.5〜1 m 内側。
   - 舗装で手塗りでない所（軌跡の近くで 6,751 m²。左上の広場、斜めの舗装路の入口、駐車場）を roadway に入れるか。入れないなら、それは「舗装か」ではなく「通る道か」という規則なので、手塗りで持つ。
3. 自動の roadway を地図に使う方向に進めるなら、規則を自動で表す必要がある。例：強度の壁の自動 ∩ GSI の道路の面（道路だけ、広場は除く）。未試行。seimon の接続（A）は roadway ではなく塩（機械の障害物）の側の問題で、別に扱う。
4. 閾値は 20 を既定にした。25 とほぼ同じ結果で、15 は少し明るい舗装も壁にしてしまう。

##### 置き場（強度の壁）

| ファイル | 中身 | Git |
| --- | --- | --- |
| `v2/experiments/202c/intensity_gate/mp10_tuned_i{15,20,25}/` | `roadway_auto.png`（原寸）、`start_crop_1to1.png`（原寸、強度の下絵：自動 = 緑、手塗りの縁 = 青、縁石 = 赤、高い段 = 紫、軌跡 = 黄）、`roadway_auto_vs_hand_q.png`（1/4、手塗りとの重ね）、`summary.json`。縁石の図は `202c/mp10_tuned/` と同じなので省いた | 管理 |
| `v2/experiments/202c/intensity_gate/gsi_vs_{gate20,gate15,hand_candidate}_q.png` | 1/4。GSI の道路の面だけ = 薄い緑、マスクだけ = 赤、両方 = 灰、GSI の道路縁 = 水色、中心線 = 黄 | 管理 |
| `v2/experiments/202c/intensity_gate/gsi_and_paved.json` | GSI に対する数値、舗装の面積、名前を付けた箱の割合 | 管理 |
| `~/bags/m8-experiments/202c/intensity_gate/*/` | 合成した地図（A・B・C）と原寸の差分 | 管理外 |
| `~/bags/m8-experiments/202c/gsi_ref/` | GSI の道路の面のキャッシュ（`gsi_road_area_k2.npz`）。記録の数値を出したときの #205 の写し（`map_georef_gsi.py`・`georef.yaml`。main と同一）と `metrics_205.json` も置いてある | 管理外 |

再現（抽出と合成は各 42 秒、GSI の比較は 20 秒。GSI のタイルは #205 で取得済みの `~/bags/m8-experiments/205/raw/` を読むだけ）：

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout
W=~/bags/m8-experiments/202c
E=docs/maps/campus-m8/v2/experiments
for th in 15 20 25; do
  scripts/map_roadway_from_curbs.py --ground-npz $W/ground_raster.npz --min-points 10 --trend-dz-min 0.03 --curb-dz-min 0.05 \
    --intensity-npz $W/ground_raster.npz --max-intensity $th \
    --out-dir $E/202c/intensity_gate/mp10_tuned_i$th --work-dir $W/intensity_gate/mp10_tuned_i$th --compose
done
# GSI の変換と道路の面は main の scripts/map_georef_gsi.py と georef.yaml を既定で読む（#205）
scripts/map_roadway_vs_gsi.py \
  --mask auto_202=$E/202/roadway_auto.png --mask r1_tuned_202b=$E/202b/R1_tuned/roadway_auto.png \
  --mask mp10_tuned_202c=$E/202c/mp10_tuned/roadway_auto.png \
  --mask gate15=$E/202c/intensity_gate/mp10_tuned_i15/roadway_auto.png \
  --mask gate20=$E/202c/intensity_gate/mp10_tuned_i20/roadway_auto.png \
  --mask gate25=$E/202c/intensity_gate/mp10_tuned_i25/roadway_auto.png \
  --overlay hand_candidate --overlay gate20 --overlay gate15
```

#### GSI の道路縁を縁ごとに証拠へ寄せる試作（202d、2026-11-17）

**結論：軌跡の近くの GSI の道路縁は、舗装の縁から多くの所で 0.3 m 程度（補正量の中央値）しかずれていない。発進点付近はさらに小さく（中央値 0.13 m、p90 0.37 m）、寄せても形はほとんど変わらない（ユーザーの目視「GSI はかなり良好」と一致）。補正が効いたのは、GSI の縁が芝の島・駐車場の側へ入り込み、道幅が 1 m 前後広く描かれていた所（北の道 y ≈ 158〜163 の x −9〜52 など）。ここでは段（縁石）へ寄せた縁が、反射強度の暗い（舗装）/ 明るい（芝）の境と重なった（目視）。効かないのは、縁の外も舗装が続く所（駐輪・駐車の帯、構内の小道の入口、広場）と、GSI の角の面取りと現物の丸い角の形が違う所。既定の地図には入れない。**

#205 の剛体の位置合わせは、全体の平行移動と回転だけを直す。電子国土基本図の道路縁は線ごとに図化の誤差（地図情報レベル 2500、標準偏差 1.75 m 以内）を持つので、左右の縁が別々にずれ、道幅が変わって見える可能性がある。全体の剛体の誤差（原点 0.1〜1 m、yaw 0.1°）は幅を変えないので、ここでは扱わない。`scripts/map_gsi_edge_snap.py` で作った。読むのは #205 の取得済みのタイル、`georef.yaml`、202c の密な面（`ground_raster.npz`）、v2 の構造物層・段差層、軌跡だけ。

**方法**

1. GSI の道路縁（ftCode // 100 == 22）を 1 m 以下の区間に分ける。元の頂点は残すので、寄せる量が 0 なら形は変わらない。地理院ベクトルのタイルは縁を重ねて持ち、同じ線が 2 回入る。別の線の区間で、中点が 0.5 m 以内、外向きの法線がほぼ同じものを重なりとみなし、数えるときは後の方を除いた（軌跡 8 m 以内で 284 区間）。重なった組の状態・寄せる量の食い違いは実測で、8 m 以内の 284 組のうち状態の食い違い 0、寄せる量の差 0.1 m 超が 4 組（全体の 3,884 組でも同じ 0 と 4）。補正後の面には両方の線が入る（道路の面は、どちらの線も越えない所）。
2. どちら側が道か：区間の法線の ±0.5〜1.5 m で、GSI の道路の面（#205 と同じ作り方）を見る。両側とも道か、両側とも道でなければ、最も近い GSI の中心線（10 m 以内、法線の向きとはっきり合うもの）の向きで決める。それでも決まらなければ「向き不明」とし、動かさない（`decide_side`）。
3. 区間の中点から法線方向 ±1.5 m の断面を、区間に沿って 5 本取る（5 cm 刻み）。証拠と、それぞれの縁の位置：
   - **段**（202c の「10 点・調整」の縁石と高い段、v2 の段差層。1 セル太らせる）：道の側から来て最初に当たる段の帯の内側の境。5 本のうち 4 本以上（`--min-support 0.8`）で同じ位置にあるものだけを線とみなす。窓の端から始まる帯は使わない。
   - **構造物**（v2 の `occupied_structure`）：段と同じ。
   - **反射強度の段**（202c の強度 20 以上の壁 = 芝・植栽）：内側 0.4 m の舗装の割合と、外側 0.4 m の芝の割合がともに 0.7 以上の境。
4. 優先は 段 > 構造物 > 強度。ただし、元の位置より内側（道の側）に段・構造物の候補があれば、外側の候補（段・構造物・強度とも）は使わない。道の端は「道の側から来て最初に当たる段」なので、内側に段があれば、その外の候補は道の端になりえない（レビュー SF-1。以前は内 −0.8 m の縁石と外 +0.5 m の段があると +0.5 を選び、縁石を道に含めえた）。内外の両側に候補があるときに未補正にする案もあるが、内側の段は根拠としてそのまま使えるので、内側を取る方にした。強度の段は内側にあっても外の段を止めない。そのうえで、同じ種類の中では元の位置に最も近い候補を取る（GSI は概ね良好なので、寄せる量の小さい解を取る）。信頼度は段 0.8、構造物 0.6、強度だけ 0.3。強度の段が 0.3 m 以内で一致すれば +0.15、同じ種類の候補が複数あれば 1 つにつき −0.1。
5. 誤った寄せを防ぐ 2 つの条件。どちらも目視で見つけた誤りから足した（下の「目視」）：
   - 外へ寄せる候補は、道に足す帯（元の位置から候補の手前 0.1 m まで）の構造物が 3% を超えるか、段が 10% を超えるなら使わない。
   - 段・構造物の候補のすぐ内側 0.4 m の半分以上が芝なら、その段は道の端ではない（植栽の島・生垣の縁）として使わない。強度は「暗い = 舗装」の側では背の低い植栽で誤るが、「明るい = 舗装でない」の側は誤りにくい。そこで明るい側だけを使う。
6. 上限：段・構造物は 1.5 m、強度だけは 1.0 m。超えたら寄せない（`rejected_cap`）。
7. 連続性：線ごとに、近傍 5 区間の中央値から 0.5 m より飛ぶ区間を却下する（`rejected_jump`）。近傍に寄せた区間が自分を含めて 2 つ未満なら、孤立として却下する（`rejected_isolated`）。区間が 1 つしかない線は近傍を数えられないので寄せない（`rejected_short_line`）。残りは近傍の中央値でならし、その区間の根拠の上限（強度だけなら 1.0 m）で切る。頂点は両隣の区間の移動の平均だけ動かすので、未補正の区間との境は半分ずつでつながる。
8. 補正後の縁と GSI の中心線から、`map_georef_gsi.gsi_road_area` で道路の面（0.1 m）を作る。補正前の面も同じ手順（1 m に分けた線）で作り直した。202c のキャッシュとの違いは 10,637 セル（0.5%、線を分けて描いた差）。手塗りの縁→GSI の縁の距離は、#205 の値（候補 1.2 / 2.3 m、merged 0.8 / 1.8 m）を再現した。
9. 幅の検算：GSI の中心線に沿って 1 m ごとに、中心線の法線の両向きに最初に当たる道路縁までの距離の和を、補正前・補正後で測る。参考に、同じ断面で反射強度の舗装が続く長さも測る（舗装でない所が 0.3 m 続いたら止め、止まった所が芝・段・構造物のときだけ数える）。軌跡から 8 m 以内に限り、差が 0.5 m を超える所を一覧にする。
10. 安全側の版（参考）：補正後の面から、段・構造物で寄せられなかった縁（未補正と強度だけの区間）の 0.5 m 以内を削り、段・構造物のセルを除く。

**区間の状態**（重なりを除いた数。「探した」は軌跡から 20 m 以内）：

| 範囲 | 区間（長さ） | 寄せた | 証拠なし | 上限超え | 飛び | 孤立 | 向き不明 | 見ていない |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 発進点〜bldg3（x −10〜70、y −15〜15） | 164（161 m） | **122（74.4%）** | 39 | 2 | 0 | 1 | 0 | 0 |
| 軌跡から 8 m 以内 | 2,008（1,930 m） | **1,410（70.2%）** | 437 | 64 | 19 | 13 | 64 | 1 |
| 探した全区間（20 m 以内） | 2,612（2,496 m） | 1,553（59.5%） | 578 | 72 | 30 | 24 | 180 | 175 |

区間が 1 つしかない線（`rejected_short_line`）は、どの範囲にも無かった。

**レビュー（SF-1〜SF-4、重なりの検出）の前後**（軌跡 8 m 以内。「前」は 2026-11-17 の 1 回目のコミット 6b103e7、重なりは位置を 0.1 m に丸めた一致で除いていた）：

| 項目 | 前 | 後 |
| --- | --- | --- |
| 区間（重なりを除いた数） | 2,020 | 2,008 |
| 寄せた / 割合 | 1,410 / 69.8% | 1,410 / 70.2% |
| 飛びで却下 | 31 | 19 |
| 補正量の中央値 / p90 | 0.30 / 0.78 m | 0.33 / 0.78 m |
| 手塗り（merged）に対する IoU（補正後） | 0.680 | 0.681 |
| 幅の差 0.5 m 超（か所 / 区間 / 補正後が狭い） | 234 / 60 / 196 | 243 / 58 / 205 |

- 内側の段・構造物で外の候補を落とした区間は 43（8 m 以内）。うち結果が変わったのは 14 区間で、残りは前から内側の候補を選んでいた。寄せた区間が SF-1 で未補正に落ちたのは 1 区間（飛びで却下）だけ。
- 変わった主な所は北の道の東（x 74〜87、y ≈ 160 の南の縁）。前は内 −0.6 m と外の候補が交互に選ばれて飛びで却下されていた 7 区間が、内へ 0.63 m に揃って寄った（下の「目視」）。
- 発進点〜bldg3 の区間の状態・補正量は前と同じ。

**補正量**（寄せた区間。符号は外向きが正。外向きの割合は、道を広げた区間の割合）：

| 範囲 / 根拠 | 区間 | 絶対値の中央値 / p90 | 符号つきの中央値 | 外向きの割合 |
| --- | --- | --- | --- | --- |
| 8 m 以内 / 全部 | 1,410 | 0.33 / 0.78 m | — | — |
| 8 m 以内 / 段 | 891 | 0.28 / 0.73 m | −0.23 m | 36% |
| 8 m 以内 / 構造物 | 14 | 0.25 / 0.70 m | −0.08 m | 43% |
| 8 m 以内 / 強度だけ | 505 | 0.35 / 0.88 m | −0.18 m | 34% |
| 発進点〜bldg3 / 全部 | 122 | **0.13 / 0.37 m** | — | — |
| 発進点〜bldg3 / 段 | 52 | 0.08 / 0.42 m | −0.08 m | 42% |
| 発進点〜bldg3 / 強度だけ | 70 | 0.18 / 0.28 m | −0.13 m | 24% |

- 寄せる向きは、内向き（道を狭める）が 6 割強。GSI の縁は道を少し広めに描く傾向がある。#205 の「GSI の道路の面には歩道・側溝・縁石の上も入る」と同じ向き。
- 構造物を根拠にした区間はわずか（14）。構造物層（建物・生垣）は縁から離れた所にあり、その手前に段か強度の段がある所がほとんど。

**道路の面の比較**（0.1 m の格子。GSI の面を評価するマスク、手塗り・自動を参照にした IoU / precision / recall）。**手塗りも正解ではない**。手塗りは人の規則で GSI の縁より 1 m ほど内側に塗られ、自動は舗装の面を取る：

| 参照 | GSI の版 | 軌跡から 8 m 以内 | 発進点〜bldg3 | 北の道 | 全体 | 参照の縁→GSI の縁（中央値 / p90） |
| --- | --- | --- | --- | --- | --- | --- |
| 手塗り（merged） | 補正前 | 0.657 / 0.695 / 0.922 | 0.810 / 0.810 / 1.000 | 0.597 / 0.598 / 0.998 | 0.249 / 0.254 / 0.929 | 0.80 / 1.84 m |
| | 補正後 | **0.681 / 0.722 / 0.925** | 0.834 / 0.834 / 1.000 | **0.638 / 0.638 / 1.000** | 0.253 / 0.258 / 0.931 | **0.63 / 1.70 m** |
| | 安全側 | 0.719 / 0.803 / 0.873 | 0.896 / 0.947 / 0.944 | 0.689 / 0.714 / 0.951 | 0.274 / 0.284 / 0.882 | — |
| 手塗り（候補） | 補正前 | 0.547 / 0.572 / 0.927 | 0.636 / 0.636 / 1.000 | 0.558 / 0.558 / 0.998 | 0.205 / 0.209 / 0.933 | 1.20 / 2.33 m |
| | 補正後 | 0.566 / 0.593 / 0.927 | 0.655 / 0.655 / 1.000 | 0.595 / 0.595 / 1.000 | 0.208 / 0.211 / 0.933 | 0.99 / 2.21 m |
| 202c 強度 < 20 の自動 | 補正前 | 0.633 / 0.793 / 0.758 | 0.738 / 0.871 / 0.829 | 0.627 / 0.692 / 0.870 | 0.242 / 0.262 / 0.758 | 0.78 / 3.40 m |
| | 補正後 | **0.664 / 0.832 / 0.767** | 0.761 / 0.900 / 0.832 | **0.679 / 0.747 / 0.882** | 0.249 / 0.269 / 0.767 | **0.40 / 3.42 m** |
| | 安全側 | 0.633 / 0.883 / 0.691 | 0.697 / 0.944 / 0.726 | 0.692 / 0.817 / 0.819 | 0.250 / 0.282 / 0.691 | — |

- 補正後は、手塗り・自動のどちらに対しても IoU と precision が上がり、recall は表の範囲ではほぼ変わらない（下がっていない）。狭めた所が、手塗り・自動の外だったことになる。自動の縁→GSI の縁の距離の中央値は 0.78 → 0.40 m。ただし、自動は補正の証拠（強度・段）と同じ点群から作ったので、自動との一致は独立な検証ではない。手塗りとの一致の改善（0.80 → 0.63 m）も、手塗りが同じ点群の下絵で塗られている点で、完全には独立でない。
- 道路の面の面積：補正前 20,308 m² → 補正後 20,047 m² → 安全側 17,231 m²（軌跡 8 m 以内は 6,717 → 6,487 → 5,504 m²）。

**幅の検算**：軌跡から 8 m 以内の中心線の上で、1 m ごとに標本を取った。重なった中心線の標本（中心線を区間と同じ方法で検出、162 か所）を除いて 1,055 か所。下の表の「標本」は、そのうち両側の縁が 10 m 以内に見つかった数：

| 量 | 標本 | 中央値 | p10 / p90 |
| --- | --- | --- | --- |
| GSI の幅 | 883 | 5.94 m | 5.03 / 9.51 m |
| 補正後の幅 | 891 | 5.85 m | 4.54 / 9.52 m |
| 舗装の幅（強度、参考） | 654 | 6.68 m | 4.60 / 10.89 m |
| 補正後 − GSI | 880 | −0.16 m | −0.96 / +0.18 m |
| 舗装 − GSI（参考） | 627 | +0.01 m | −0.99 / +3.90 m |

- 補正後と GSI の幅の差が 0.5 m を超えた所は 243 か所、58 区間（`width_diff.csv`、`width_diff_sections.json`、`width_diff_q.png`）。うち 205 か所は補正後が狭い。長い区間は北の道（x 22〜52、y ≈ 160 で 31 m・−1.11 m、x −9〜13、y ≈ 158 で 23 m・−1.20 m、x 74〜94 で 21 m・−1.11 m、x 112〜122 で 11 m・−1.11 m）と、東の南北の道（x ≈ 141〜144、y 46〜52 と 118〜127 で −0.8〜−0.9 m）。
- 重なりの検出は 0.5 m 以内・法線がほぼ平行なものに限るので、半径より大きくずれて重なった中心線の標本は二重に数えうる（数は確かめていない）。
- 交差点では、中心線の法線が交わる道の縁に当たるので、幅が大きく出る（`width_diff_q.png` の交差点の横線）。一覧のうち交差点の区間は、幅の差ではない。
- 舗装の幅は、道の脇の駐車場・広場・小道へ舗装が続くと広く出る（p90 +3.9 m）。縁の位置の検算には使えないので、参考にとどめる。

**目視**（`start_crop_{photo,intensity}_1to1.png` は x −10〜70、y −15〜15。`north_crop_*` は x −10〜70、y 150〜172。強度の版の色：白 = 補正前の GSI の縁、赤 = 段で寄せた縁、橙 = 構造物、青 = 強度だけ、桃 = 探したが動かさなかった縁、黄 = 中心線と軌跡。下絵の赤い点・橙の点は段・構造物の層。写真は約 0.48 m/画素でぼやけるので、縁の判断は主に強度の版で行った）：

- **発進点の東（x 10〜70）**：GSI の縁は、写真のアスファルトの縁、強度の暗い / 明るいの境とほぼ重なる。寄せは 0.1〜0.2 m で、寄せ先は強度の境か縁石の線。植栽・木の上への寄せは見当たらない。
- **北へ延びる道（x 23〜27）との交差点**：GSI の西の角は面取り（直線）で、現物の縁石は丸い角。段へ寄せた縁（赤）は丸い角に沿った（内へ 0.4〜0.6 m）。東の角では、GSI の面取りが芝の三角を道に含めている。断面に平行な縁が無いので、ここは動かせない（桃）。**局所的な法線方向の平行移動では、角の形の違いは直せない。**
- **1 回目の誤り 1（x −10〜0 の南の縁）**：`--min-support 0.6`（5 本のうち 3 本）では、道の外の駐輪・駐車の帯の奥にある段の点の並びへ、1 m 外へ寄った。この帯は強度では舗装で、構造物の点が並ぶ。写真でもアスファルトに見え、道か帯かは点群からは決まらない。0.8 にして、ここは未補正になった。その時点の数値はほぼ変わらなかった（8 m 以内の寄せた区間 74% → 70%、手塗り（merged）に対する IoU 0.675 → 0.675。どちらも重なりを除く前の値）。
- **北の道の東（x 74〜87、y ≈ 160 の南の縁。レビューの修正で変わった所、切り出しは画像に残していない）**：前は内外の候補が交互に選ばれ、飛びで却下されていた。内側の段を優先する修正の後は、内へ 0.63 m に揃って寄った。寄せ先は、路肩に沿って明るい点が並ぶ帯（側溝の格子と推測）のすぐ外の段。その外は建物までの舗装の帯で、GSI の縁はその帯の中ほどを通る。「道の側から最初に当たる段が道の端」という規則には合うが、その帯（歩道か）を道に含めるかは人の判断。
- **1 回目の誤り 2（北へ延びる道の東の縁、y 10〜15）**：舗装の端の 0.6 m 外にある生垣の段へ寄った（間は芝の帯）。「段の内側が芝なら道の端ではない」の条件を足し、舗装の端の段に寄るようになった。
- **斜めの舗装路の入口（x 30〜37 の南）**：舗装が続くので縁が無く、未補正になった（正しい）。
- **北の道（x −10〜18、y ≈ 158〜163）**：**GSI の南の縁が、芝の島の中へ約 0.6 m 入っていた。** 寄せた縁は芝の島の縁石の上で、強度の暗い / 明るいの境と重なる。北の縁も 0.2 m ほど縁石へ寄った。幅は GSI より約 1.2 m 狭くなる。x 18〜44 では、GSI の南の縁が駐車場の中を通る。寄せた縁は、駐車場との境の縁石の線（強度の明るい細い線）にある。駐車場の入口の扱いは人の判断。
- **背の低い植栽**：強度ではアスファルトと同じく暗い。そのため、強度だけの寄せはその外側（芝との境）まで行きうる。2 か所の切り出しでは、強度だけの寄せは小さく（発進点〜bldg3 で p90 0.28 m、外向き 24%）、植栽の上への寄せは見当たらなかった。他の場所は見ていない（未確認）。安全側の版では、強度だけの区間を未補正と同じに扱う。
- **木の影**：写真は証拠に使っていない。強度は LiDAR の反射なので、影の影響を受けない。木の幹・生垣は構造物・段に出るが、2 か所の切り出しには、それが寄せ先になった所は無かった。

**推奨**

1. **既定の地図には入れない。** 補正の主な効果は GSI の道路の面を狭めることで、手塗りの roadway（GSI の縁より内側）は変わらない。
2. **点検に使う。** 幅の差 0.5 m 超の一覧（`width_diff_sections.json`、GIMP の 18 番）は、GSI の縁を信じてはいけない所の一覧になる。GIMP の 17 番で、寄せた縁と根拠（色）を見る。赤（段）の寄せは採用してよい候補、青（強度だけ）と桃（未補正）は人が見る、と分けられる。
3. 202c の推奨 3（「強度の壁の自動 ∩ GSI の道路の面」で規則を自動で表す）を試すなら、GSI の面は補正後（`~/bags/m8-experiments/202d/gsi_snapped_road.png`）を使う。補正前は、北の道などで芝・駐車場を 1 m 前後含む。
4. 占有地図に GSI を使う場合の参考は、安全側の版（`gsi_safe_road.png`）。段・構造物で確かめられない縁は 0.5 m 内側へ削ってある。ただし角の形の違い（面取り）は、削っても残る。
5. 未確認：発進点付近と北の道以外の目視。背の低い植栽の場所の特定（強度では区別できないので、現地か高解像度の写真が要る）。パラメータの掃引（`--search-m`、`--jump-m`、`--uncert-m`）。

**置き場（202d）**

| ファイル | 中身 | Git |
| --- | --- | --- |
| `v2/experiments/202d/summary.json` | 区間の状態と補正量、道路の面の比較、幅の統計、タイルの重なり（除いた数・組の食い違い）、証拠のセル数、パラメータ、所要時間 | 管理 |
| `v2/experiments/202d/edge_segments_near.csv` | 軌跡 8 m 以内の区間ごとの offset・根拠・信頼度・状態（map 座標、外向きの法線、候補の数、落とした候補の数、内側の段で外の候補を落とした数 `outward_blocked`） | 管理 |
| `v2/experiments/202d/width_diff.csv`, `width_diff_sections.json` | 幅の差が 0.5 m を超えた標本（map 座標・法線・左右の距離）と、続いた標本をまとめた区間 | 管理 |
| `v2/experiments/202d/overview_q.png` | 1/4。補正後の道路の面（薄緑）、補正で外れた面（薄赤）、加わった面（緑）、補正後の縁（赤 = 段、橙 = 構造物、青 = 強度、桃 = 未補正、灰 = 探していない）、補正前の縁（灰の細線）、中心線（黄）、軌跡（黒） | 管理 |
| `v2/experiments/202d/width_diff_q.png` | 1/4。幅の差 0.5 m 超の断面（赤 = 補正後が狭い、青 = 広い、橙の細線 = 舗装の幅との差） | 管理 |
| `v2/experiments/202d/{start,north}_crop_{photo,intensity}_1to1.png` | 原寸。上の「目視」の画像 | 管理 |
| `~/bags/m8-experiments/202d/edge_segments.csv`, `width_samples.csv` | 全区間・全標本の表 | 管理外 |
| `~/bags/m8-experiments/202d/gsi_road_versions_k2.npz` | 0.1 m の道路の面（補正前・補正後・安全側）、縁（補正前・補正後）、中心線 | 管理外 |
| `~/bags/m8-experiments/202d/gsi_{snapped,safe}_road.png` | 原寸（5 cm）の補正後・安全側の面（白・不透明 = 道。compose_occupancy の形式） | 管理外 |
| `~/bags/m8-experiments/202d/run.log` | 実行記録（`/usr/bin/time -v`） | 管理外 |
| `~/bags/m8-experiments/gimp-layers/17_gsi_edge_snapped.png`, `18_width_diff.png` | GIMP 用の原寸 RGBA（17 = 補正後の縁を根拠で色分け、18 = 幅の差）。`campus-m8_layers.xcf` に入れた（17 は表示、18 は非表示で開く） | 管理外 |
| `~/bags/m8-experiments/gimp-layers/build_xcf.sh` | `layers.txt` から `build.scm` を作り、GIMP で `campus-m8_layers.xcf` を作り直す（Git 外。消えたら、`layers.txt` の各行から `gimp-file-load-layer` を並べる同じ形の build.scm を作る） | 管理外 |

再現（約 52 秒、最大メモリ 3.0 GB。xcf の作り直しは約 1 分）。入力の `~/bags/m8-experiments/202c/ground_raster.npz` は、202c の再現（上の「bag のスキャンを軌跡で直接重ねた密な地面の面」）で作る。タイルは #205 で取得済みの `~/bags/m8-experiments/205/raw/` を読むだけ：

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout
mkdir -p ~/bags/m8-experiments/202d
/usr/bin/time -v python3 scripts/map_gsi_edge_snap.py > ~/bags/m8-experiments/202d/run.log 2>&1
bash ~/bags/m8-experiments/gimp-layers/build_xcf.sh   # Git 外のスクリプト（このリポジトリには無い）
python3 -m pytest scripts/tests/test_gsi_edge_snap.py -q
```

`map_gsi_edge_snap.py` は既定で、docs の `v2/experiments/202d/`、`~/bags/m8-experiments/202d/`、`~/bags/m8-experiments/gimp-layers/` の 17・18 番を上書きする。パラメータを試すときは `--out-dir`・`--work-dir` を変え、`--no-gimp` を付ける。`build_xcf.sh` は `layers.txt`（`ファイル名の幹|レイヤー名|表示 1/0`）の各行から作る。17・18 番の 2 行は 2026-11-17 に足した。

### 手塗りを補正後の地理院の縁まで広げた候補（154w、2026-11-17、2026-11-19 に R1〜R6 を反映）

**結論：手塗りの roadway を、202d で段・構造物へ寄せた地理院の縁（赤・橙）まで外へ広げた「確定版の候補」を作った。2026-11-19 の判断（R1〜R6）を既定に入れた版で、FREE は全体で 375.4 m²（+7.3%）増え、FREE が減った所は無い。OCC は、帯の中の孤立した 3 セル以下の点（R6）の 789 セルだけ減った。waypoint の 4 地点は今の既定の地図と同じく、4点とも FREE で、車体の幅で1つの成分につながる。広げた縁はほぼ全部（774 区間のうち 763）で OCC まで inflation の半径（0.6 m）より近い。車体の中心が置ける面（OCC から 0.45 m 以上）として増えたのは 152.9 m²、inflation の外は 87.9 m²。R6 は、生垣・柵の 2 セルごと（間に 1 セル）の点の格子を障害物として残す版（`--band-salt-gap-cells 1`）を既定にした。手塗りの内側と同じ生の 8 連結（gap 0）では格子の前の列が消えて帯がその奥まで入るためで、gap 0 の版（FREE +391.3 m²）との採否はユーザーが決め直す（下の「R6 の 2 つの版」）。**

**2026-11-19 に、この既定の版（gap 1）を既定の地図 `candidate` に採用した**（ユーザー判断 R7。下の「採用」）。広げる前の手塗りのままの版は `candidate_hand` として残し、ファイルかパラメータの切り替えで戻せる。Nav2 の planner を地図だけで動かすと、配車 4 地点の 12 組はどちらの地図でも全て計画でき、12 組のうち 9 組は経路が変わらない（下の「Nav2 の planner での確認」）。屋外ではまだ走らせていない。

向き：georef.yaml では map の +x はほぼ南（北から 185°）、+y はほぼ東を向く。この節の「北の道」「南の縁」などは、前の版から使っている画像の上下（上 = map の +y）の呼び名で、方位ではない。R1〜R6 の所は方位か「map の ±y 側」で書いた。

`scripts/map_widen_hand_roadway.py` で作った。読むのは、手塗りの候補（`candidate_roadway_merged.png`、`candidate_free_merged.png`、`candidate_keepout_mask.png`）、v2 の機械レイヤ、202d の補正後の道路の面（`~/bags/m8-experiments/202d/gsi_snapped_road.png`）と全区間の表（`edge_segments.csv`）だけ。この節の数字は、断りが無ければ `v2/experiments/154w/summary.json` の値（キーを括弧に書く）。

#### ユーザーの判断（2026-11-17、判断票の回答）

- Q1：手塗りの縁は、**赤・橙の区間（status=snapped かつ kind が step または structure）だけ**、補正後の地理院の縁まで広げた候補を作る。
- Q2：左上の広場（x −10〜−2、y 3〜15）、斜めの舗装路の入口（x 30〜45、y −8〜−2）、発進点西の駐輪・駐車の帯（x −10〜0 の南の縁）、北の道の駐車場の入口（x 18〜44 の南）は**含めない**。
- Q3：北の道の東の路肩の帯（x 74〜87、y ≈ 160）は補正後の縁のまま（帯は含めない）。
- Q4：強度だけで寄せた区間（kind=intensity）は**採らない**（未補正と同じ扱い＝広げない）。
- Q5：補正後の地理院の面は、手塗りの確定版の候補づくりに使う。
- Q6：地理院の要調整箇所の追加指定はなし。
- 前提：自動抽出（202c の強度 < 20、緑）は背の低い植栽を含むとユーザーが目視で判断したので、根拠に使わない。

#### 2 回目の判断（2026-11-19、判断票の回答 R1〜R6）

前の版（2026-11-17、コミット 672bd81）の「要判断の所」への回答。すべてスクリプトの既定の振る舞いにした（前の振る舞いはオプションで再現できる。下の表）。

| 回答 | 内容 | 実装 | 前の版の再現 |
| --- | --- | --- | --- |
| R1 除く | 発進点から正門前へ向かう道の西（map の −y 側）の駐輪・駐車の帯の全体 | 除外の箱 `west_parking_strip`（x −77〜+6、y −9〜−1）。Q2 の発進点西の箱（x −10〜0）を置き換えた | `--exclude-box` で前の 4 つを渡す |
| R2 残す | 北の道の南（x 44〜60、y 150〜160.5）の出っ張り | 変更なし | — |
| R3 広げない | 上限 1.5 m まで広がった区間 | その区間に割り当てた帯を全部落とす（`--capped-policy drop`、既定） | `--capped-policy clip` |
| R4 残す | 北へ延びる道の両側（x 21〜27、y 5〜16） | 変更なし | — |
| R5 除く | 点の密な帯（x ≈ −95） | 除外の箱 `dot_field`（x −97〜−93.5、y 73〜94.5。判断票の箱から広げた。下の 6.） | 箱を外す |
| R6 消す | 帯の中の 3 セル以下の障害物 | 手塗りの内側の塩と同じ規則で消す（`--band-salt-max-cells 3`、既定）。ただし点の格子は残す（`--band-salt-gap-cells 1`、既定。2026-11-19 のレビュー MF-1、下の「R6 の 2 つの版」） | `--band-salt-max-cells 0` |

#### 方法

1. 広げてよい帯：補正後の地理院の道路の面の中で、手塗りの roadway の外、手塗りからの距離が `--max-widen-m`（1.5 m。202d の寄せる量の上限と同じ）以内のセル。
2. そのセルに最も近い補正後の縁の区間が赤・橙で、最も近い青・桃・灰の区間より `--tie-m`（0.05 m）以上近いもの。区間は 202d の表の中点を `offset_m` × 外向きの法線だけ動かし、2.5 cm ごとの点にして最近傍を取る（全 69,775 区間、うち赤・橙 999）。境目で迷うセルは広げない側に倒す。
3. その最も近い赤・橙の区間の外向きの法線で見て、縁の外側にあるセルは落とす（`stages_cells.outside_snapped_edge`：25,465 セル）。補正後の面は 202d で 0.1 m の格子で作ったので、縁が階段になり、縁のすぐ外のセルが面に入る。また、交わる道の口では、面が縁の延長の外（交わる道の中）へ続く。
4. keepout の塗りを除く（今の候補は空。keepout は free より優先なので明示した）。
5. R3：上限まで広がった区間の帯を落とす。帯を一度作り（下の 6.〜9. まで）、帯のセルを最も近い赤・橙の区間に割り当て（集計と同じ割り当て）、帯のセルが 4 以上（`--min-seg-cells`）で最大の幅が上限 − 半セル（1.475 m）以上の区間を「上限まで広がった区間」とする（前の版の `derived.capped_*` と同じ定義）。その区間に割り当てたセルを候補から除いて作り直す。候補が減るだけなので、作り直した帯は 1 回目の部分集合で、上限まで広がる区間は新たにはできない（`checks.capped_policy_drop_leaves_no_capped_segment`）。1 回目の数字は `capped`。
6. R6：段 ∪ 構造物の成分で 3 セル以下のもののうち、成分の全セルが「ここまでの候補（除外の箱の後）∪ 手塗り」に入り、しかも 7. で残す障害物を太らせた所に掛からないものを、帯へ戻す。成分は、1 セル以下の隙間で並ぶ点を 1 つとみなして取る（`--band-salt-gap-cells 1`、既定。各点を自分を角とする 2×2 の正方形に広げて 8 連結を取ると、隙間が 1 セル以下の点どうしだけが繋がる）。こうすると 10 cm ボクセルの生垣・柵の 2 セルごとの点の格子は大きい塊として残る。`--band-salt-gap-cells 0` は `gen_auto_free_mask.py`（手塗りの内側の塩を消す規則、3 節）と同じ生の 8 連結。どちらで選んだ点も生の成分は 3 セル以下なので、消すのは `gen_auto_free_mask.py` の同じ規則。帯の縁にまたがる点（補正後の縁の外に一部が出る）と大きい障害物のすぐ脇の点は戻さない（縁そのものか障害物の一部とみなす、安全側）。戻した点は **roadway にだけ**足し、free の塗りには足さない。合成の `gen_auto_free_mask.py` が手塗りの内側と同じく `free_mask_auto.png` に入れて消し、`erased_by_free.png` では自動の消去（水色）として残る（free にも塗ると人の塗りとの「両方」の扱いになり、どちらの規則で消えたか追えない）。
7. 残した段・構造物（`occupied_step` ∪ `occupied_structure` から 6. で戻した点を除いた物。compose_occupancy の machine_occ と同じ集合から）を 1 セル（0.05 m、`--barrier-dilate-m`）**正方形**で太らせて除く。帯は free の塗りにも足すので、除かないと機械の障害物を消す。太らせるのは、static.pcd が 10 cm のボクセルで、植栽・生垣の構造物が 2 セルごと（間に 1 セル）の点の並びになるため（太らせない初回は、x 104〜106、y 18〜19 で帯が点の間を通って植え込みへ入った）。正方形にするのは、楕円の 3×3 は十字で、斜めに 2 セルごと（間に 1 セル）に並ぶ点の間を 8 連結の帯がすり抜けるため（レビュー MF-1）。`salt_candidates` は段 ∪ 構造物と同じ集合（2026-11-17 に実測で 401,771 セルが一致、`checks.salt_candidates_equal_step_or_structure`）なので、別には足さない。
8. 除外の箱（`--exclude-box`、既定は下の 5 つ）の中を除く。
9. 残りの 8 連結の成分のうち、手塗りに接するものだけを残す（飛び地は足さない。`stages_cells.dropped_islands`：755 セル）。
10. 内側へ（手塗りを削る方向）は変えない。
11. 合成：3 節の「再現」と同じ順（`gen_auto_free_mask.py` → `compose_occupancy.py`）。roadway = 手塗り ∪ 帯（6. の点を含む）、free = 手塗りの free ∪ 帯（6. の点を除く）、keepout は手塗りの候補のまま。帯を free にも足すのは、旧地図の直接の拡幅と同じ扱い。roadway だけに足すと、free_evidence（レイの通過）の無い 2,903 セル（7.3 m²、帯の 1.9%、`derived.band_cells_without_free_evidence`）が UNKNOWN のまま残り、帯がまだらになる（`--free-policy evidence` で比べられる）。
12. 確かめること（外れたら終了コード 1、`checks`）：手塗りのまま同じ手順で合成した地図が今の既定の地図とバイト単位で同じ、増えた FREE が帯と一致（`--free-policy evidence` では帯の部分集合）、変わったセルが帯の中だけ、帯が手塗り・keepout・残した段・構造物と重ならない、合成の `free_mask_auto.png` の手塗りのままとの差分が 6. で戻した点と 1 セルも違わない（`band_salt_erased_by_auto_rule`）、drop のとき、落とした区間のセルが帯に 1 つも無い（`capped_segment_cells_absent_from_band`）、上限まで広がった区間が残らない（`capped_policy_drop_leaves_no_capped_segment`）。2026-11-19 の実行では、既定（gap 1）・gap 0 ともすべて成り立った。
    - 抜け穴：帯のセルが `--min-seg-cells`（4）未満の区間は「上限まで広がった区間」に数えないので、上限まで届いても R3 で落ちない。その数を `capped.small_segments_reaching_cap` に残す（2026-11-19 は両方の版で 0 区間）。
13. 二重に広げない止め：入力の手塗りが、前回の出力の `roadway_widened.png` と同じ（`summary.json` の `outputs_sha256.roadway_widened` と sha256 が一致、または `~/bags/m8-experiments/154w/roadway_widened.png` と塗りが一致）なら、何も書かずに止まる。入力の sha256 は `inputs_sha256` に残す。

除外の箱（Q2 の x の範囲と、広場・入口の y はユーザーの回答のまま。回答に y の無い所は、手塗りの map の −y 側の縁の外を覆い、道の反対側の帯に掛からない値を原寸の切り出しで決めた）と、その中の実測（`q2_boxes`。キーの名前は前の版のまま、R1・R5 の箱も入る。「箱が無いときの帯」は 6. の点の戻しを箱の後の候補で判定した値なので、箱の中の点は戻していない）：

| 箱 | 範囲（x / y） | 補正後の面のうち手塗りの外 | 箱が無いときの帯 | 箱の後 |
| --- | --- | --- | --- | --- |
| 左上の広場 | −10〜−2 / 3〜15 | 3.46 m² | **1.49 m²**（広場の map の −y 側の縁の段の手前の細い帯） | 0 |
| 斜めの舗装路の入口 | 30〜45 / −8〜−2 | 0 | 0（入口の縁は未補正＝桃） | 0 |
| 発進点〜正門前の道の西の駐輪・駐車の帯（R1、`west_parking_strip`） | −77〜+6 / −9〜−1 | 77.71 m² | **16.87 m²** | 0 |
| 北の道の駐車場の入口 | 18〜44 / 150〜160.5 | 32.37 m² | **20.73 m²**（駐車場との境の縁石の手前、x 18〜44 の map の −y 側の縁） | 0 |
| 点の密な帯（R5、`dot_field`） | −97〜−93.5 / 73〜94.5 | 31.72 m² | **5.10 m²** | 0 |

広場と駐車場の入口は、補正後の面に入り、赤の区間に面していたので、箱が要った。斜めの入口は、縁が未補正なので、箱が無くても入らなかった。Q3 の路肩の帯（x 74〜87、y ≈ 160）は補正後の縁（内へ 0.63 m）の外なので入らない。

R1・R5 の箱の決め方（2026-11-19、前の版の帯の原寸で実測）：

- R1：前の版の帯の、x −77〜+6 の map の −y 側のセルは y −4.87〜−1.42、+y 側のセルは y 0.52 以上。y −9〜−1 は −y 側だけを覆い、道の反対側の帯は切らない（`west_crop_*` で確かめた）。y −9 は、手塗りの縁が最も −y へ出る x −77（y −4.2）から上限 1.5 m を引いても余る値。x −80〜−77 の −y 側の帯（正門前の手前、1.05 m²、419 セル）は回答の範囲の外なので残した。中身（2026-11-19 の原寸の目視）：手塗りの縁と、その −y 側の植え込みの角の縁石（強度の明るい線）の間の細い帯で、強度は暗い舗装（中央値 12.4、≥ 20 は 3%。隣の手塗りの内側は 8.4）。x −77 より +x 側の駐車・駐輪の区画（強度の四角い模様）には掛からず、区画の前の路肩の舗装に見える。写真（約 0.48 m/画素）では区別できない。
- R5：判断票の箱（x −96.5〜−93.5、y 76〜92）では、点の密な所（x −99〜−95.5 で段 ∪ 構造物のセルが 3% 以上の、y 0.5 m 刻みの帯）が y 73〜94.5 まで続き、前の版の帯は箱の外の y 75〜76（0.49 m²）と y 92〜94.5（0.96 m²）にも同じ点の間の隙間として残っていた。y 94 付近の帯は x −96.68 まで出る。そこで y を密な所の端へ、x を −97 へ広げた。x −93.5 は道の反対側（x ≈ −91）の帯に届かない。y 94.5 より +y 側の帯（点が 1% 未満の舗装の脇、3.03 m²）は切らない。

#### 結果（手塗りのままの地図との比較）

「手塗りのまま」は採用前の既定の地図（2026-11-19 からは `occupancy_candidate_hand.yaml`）。この節で「今の既定の地図」「今の縁」と書いた所は、採用前に書いたもので、手塗りのままの版を指す。

| 項目 | 手塗りのまま（`candidate_hand`） | 154w の候補（2026-11-19。採用して今の `candidate`） |
| --- | --- | --- |
| FREE（全体、`areas.whole`） | 2,066,589 セル（5,166.5 m²） | 2,216,748 セル（5,541.9 m²、**+375.4 m²、+7.3%**） |
| FREE（発進点〜bldg3：x −10〜70、y −15〜15） | 439.2 m² | 459.7 m²（+20.5） |
| FREE（北の道：x −125〜96、y 145〜175） | 878.7 m² | 1,012.1 m²（+133.4） |
| FREE が減ったセル | — | 0 |
| OCC | 393,435 セル | 392,646 セル（−789。R6 で消した点。消した 792 セルのうち 3 セルは手塗りの free の塗りの中で、今の地図でも OCC ではなかった） |
| waypoint の 4 地点が FREE か | 4点とも FREE | 4点とも FREE |
| 4地点が車体の幅でつながるか（`map_route_check.py`、0.35 m 削り、`routes`） | つながる（成分 7 個） | つながる（成分 13 個。増えたのは帯の小さな出っ張りが削った後に残る分で、4点はどちらも同じ成分） |

R1〜R6 ごとの数字（前の版は 2026-11-17 のコミット 672bd81 の `summary.json`。「要判断の所」の箱の帯の面積は `watch_band_m2`）：

| 回答 | 前の版 | 今の版 | 内訳 |
| --- | --- | --- | --- |
| R1 駐輪・駐車の帯（x −75〜−10、y −6〜−1） | 15.49 m² | 0 | 箱 `west_parking_strip` の中の帯は箱が無ければ 16.87 m²（`q2_boxes`） |
| R1 同じ帯の東（x 0〜5、y −4〜−1） | 0.48 m² | 0 | |
| R2 北の道の南（x 44〜60） | 1.97 m² | 2.44 m² | 増えた 0.47 m² は R6（孤立した点とその周りの 1 セルが帯に戻った）。R6 を外す（`--band-salt-max-cells 0`）と 1.97 m² で前の版と同じ。gap 0 では 2.63 m² |
| R3 上限で切れる所（x −96〜−93、y 12〜48） | 36.80 m² | 0.09 m² | 残るのは上限に達しない隣の区間のセル |
| R4 北へ延びる道の両側（x 21〜27、y 5〜16） | 6.40 m² | 6.52 m² | 増えた 0.12 m² は R6。R6 を外すと 6.40 m²、gap 0 では 6.89 m² |
| R5 点の密な帯（x −96.5〜−93.5、y 76〜92） | 3.65 m² | 0 | 広げた箱 `dot_field` の中は箱が無ければ 5.10 m² |

R3（`capped`）：1 回目の帯（R1・R5・R6 を入れた後、478.3 m²）で上限まで広がった区間は **65 区間、102.8 m²**（41,134 セル）。これを全部落とした。前の版の定義と数え方は同じ。前の版の 65 区間・98.3 m² より面積が多いのは R6 のため：孤立した点とその周りが帯に戻ると、同じ区間の帯が少し広がる。R6 を外して作ると 65 区間・98.32 m² で前の版と一致する。`--capped-policy clip`（上限で切って広げる）なら帯は 478.3 m²。

R6（`band_salt`）：帯へ戻して消した点は **792 セル、766 成分**（1 セル 740、2 セル 26。1.98 m²）。合成の `free_mask_auto.png` の差分と 1 セルも違わない。1 セルの隙間で繋がる点の格子と 4 セル以上の塊は残り、その周りの 1 セルも帯に入らない。帯の 0.1 m 以内に残る OCC は 4,062 セル（3,619 成分、`derived.occ_within_0_1m_of_band`。前の版は 5,783 セル、5,191 成分）。

#### R6 の 2 つの版（2026-11-19 のレビュー MF-1）

9f8cc95 では R6 を手塗りの内側と同じ生の 8 連結（gap 0）で入れた。レビューで、生垣・柵の 2 セルごと（間に 1 セル）の点の格子が 1 点ずつ「3 セル以下」として消え、残す障害物から外れて太らせた壁も無くなるので、格子の奥の候補のセルが帯につながり補正後の縁まで FREE になることが分かった（原寸の切り出しで、帯の外の縁で消えた水色の点は生垣の前の列だった。上限まで届く区間が 65 から 68 に増えたのもこのため）。R6 の回答は「大きい塊は残るので本物の障害物を消すおそれは小さい」という説明に基づいていたが、格子ではそれが成り立たない。そこで、格子を残す版（`--band-salt-gap-cells 1`）を既定にし、gap 0 は選べるように残した。ユーザーが決め直す。

なお 9f8cc95 の `--band-salt-gap-cells g` は、中心に置いた半径 g の正方形で太らせて連結を取っていたので、隙間 2g まで繋がっていた（レビュー SF-1）。各点を自分を角とする (g+1)×(g+1) の正方形に広げる方法に直し、隙間 g 以下だけが繋がるようにした（単体テストで隙間 1 と 2 を区別）。9f8cc95 の報告にあった gap 1 の一時の数字（帯 370.6 m² など）と、gap 0 の `cells_in_dot_lattice` 1,720 は、この誤りの入った値で、下の表で置き換える。

| 項目 | 既定（gap 1、格子を残す） | gap 0（手塗りの内側と同じ生の 8 連結） |
| --- | --- | --- |
| 帯 = 増えた FREE | 375.4 m²（+7.27%） | 391.3 m²（+7.57%） |
| 消した点 | 792 セル、766 成分 | 2,292 セル、2,093 成分（うち 1 セルの隙間で格子になる点 1,423 セル、`band_salt.cells_in_dot_lattice`） |
| OCC の減り | 789 セル | 2,288 セル |
| R3 で落とした区間 | 65 区間、102.8 m² | 68 区間、110.1 m² |
| 増えた FREE のうち OCC から 0.45 m 以上 / 0.6 m 以上 | 152.9 / 87.9 m² | 177.8 / 105.2 m² |
| 今は 0.6 m 以上で、広げて 0.6 m 未満になる区間（全 OCC まで） | 136 区間、58 か所 | 136 区間、58 か所 |
| waypoint の 4 地点（`map_route_check.py`） | all_connected、成分 13 個 | all_connected、成分 10 個 |
| `checks` | すべて成り立つ | すべて成り立つ |
| 出力 | `v2/experiments/154w/`、`~/bags/m8-experiments/154w/` | `~/bags/m8-experiments/154w_gap0/`（Git 外） |

数字の出所は、既定の版の `summary.json` の `key` と `compare.key`（gap 0 の `summary.json` から写した物）。gap 0 で多い 1,500 セルは、ほとんどが格子の点（1,423 セル）で、多いのは x 60〜100、y 0〜20、x 60〜120、y 160〜180、x −100〜−80、y 120〜160（9f8cc95 の一時の集計、20 m の升目）。

| 項目 | 値 |
| --- | --- |
| 広げた区間（帯のセルが 4 以上、`widened_segments`） | 774（段 762、構造物 12）。赤・橙の区間にタイルの重なり（dup）の区間は無かった（`segments.trusted_non_dup` = 999）ので、重なりへ割り当てたセルは 0（`derived.cells_reassigned_from_dup_segments`） |
| 区間ごとの最大の広げた幅（手塗りの縁から） | 中央値 0.54 m、p90 0.90 m、最大 1.47 m（上限まで広がった区間は R3 で落とした） |
| 上限 1.5 m に達した区間（`derived.capped_*`） | 0（R3） |
| 増えた FREE のうち OCC から 0.45 m 以上（車体の中心が置ける、`usable`） | 152.9 m² |
| 増えた FREE のうち OCC から 0.6 m 以上（inflation の外） | 87.9 m² |
| 帯の中で強度 ≥ 20（明るい）のセル（`derived.bright_*`） | 45.7 m²（帯の 12.2%。手塗りの内側は 1.9%）。縁石・側溝のコンクリートも明るく、芝と区別できないので参考 |
| 広げた roadway に新たにできる穴（1 m² 以下、`derived.new_holes_in_roadway_mask`） | 44 個、732 セル（そのうち OCC 104 セル）。前の版は 575 個、7,650 セル。R6 で孤立した点が roadway に入ったため（gap 0 では 24 個、78 セル） |
| 所要時間・最大メモリ | 約 48 秒、3.6 GB（R3 で帯を 2 回作る） |

#### レビューの修正（MF-1、SF-5）の前後（2026-11-17 の版の記録）

この小節の数字は R1〜R6 を入れる前の 2026-11-17 の版（コミット 672bd81）のもので、今の `summary.json` には無い。前の版（2026-11-17 のコミット e5dbf39。楕円 3×3 で太らせ、縁の外側のセルを落としていなかった）と比べた。前の版の帯を、同じコードの十字・法線なしの設定で作り直すと 1 セルも違わない（215,058 セル）ことを確かめてから分けた（この表は一時の比較で、`summary.json` には無い。今の版の数字だけが `summary.json` にある）：

| 段階 | 帯 | 減った面積 | 減った所 |
| --- | --- | --- | --- |
| 前の版 | 537.6 m² | — | — |
| MF-1（正方形で太らせる）だけ | 510.9 m² | −26.8 m² | 9,328 か所に散る。ほとんどは点の斜めの隣の 1 セル。点の間を抜けて奥へつながっていた所は x −95.4、y 80.4（0.25 m²）、x −95.3、y 77.7（0.16 m²）、x 26.3、y 12.2（0.10 m²）など。発進点〜bldg3 で −1.8 m²、北の道で −10.5 m² |
| ＋SF-5（縁の外側を落とす）＝今の版 | 480.1 m² | −30.7 m² | 763 か所。大きい所は x −13.4、y 75.4 と x −19.7、y 75.2（各 1.1 m²、南北の道の交差点で、交わる道の口の中）、x 51.1、y 159.6（0.8 m²、北の道の南の縁。補正後の面が赤い縁より 0.4 m 外まで続いていた）、x 17.3、y 147.1（0.7 m²）、x 146.5、y 79〜81（各 0.6 m²）。発進点〜bldg3 で −2.0 m²、北の道で −8.4 m² |

目視（原寸の切り出し）：x −95、y 76〜82 では、構造物の点が密に並ぶ帯（点群では高さ 0.1〜2.2 m の物の並び。植栽か駐輪かは区別できない）の中へ、前の版の帯が点の斜めの隙間を通って入り込んでいた。修正後もこの帯の中に、点の間の 2 セル以上の隙間の帯が残った（x −96.5〜−93.5、y 76〜92 で 3.65 m²。2026-11-19 に R5 で箱を置いて除いた）。SF-5 で落ちた交差点の 2 か所は、南北の道の赤い縁の延長の外、交わる道の中で、落とすのが正しい。x 51.1、y 159.6 は、帯が赤い縁で止まるようになった。

#### inflation の余裕

`src/whill_navigation/config/nav2_params.yaml`：global / local とも `robot_radius` 0.45 m（円）、`inflation_radius` 0.6 m、`cost_scaling_factor` 4.0。global は static_layer + inflation_layer、`allow_unknown: false`、`track_unknown_space: true`。

縁は、小さい穴（1 m² 以下、`--hole-max-m2`。帯の中に残る点とその周り）を埋めた FREE の外の縁で取る。新しい縁は区間に割り当てた帯の外の縁、今の縁はその帯のセルに 8 近傍で接する今の FREE の縁（`clearance`）：

| 距離の相手 | 新しい縁：0.6 m 未満 / 0.45 m 未満 | 新しい縁：中央値 / p90 | 今の縁：0.6 m 未満 | 今の縁：中央値 | 今は 0.6 m 以上で、広げて 0.6 m 未満になる区間 |
| --- | --- | --- | --- | --- | --- |
| 全 OCC | 763 / 759（774 区間中） | 0.10 / 0.16 m | 622 / 769（80.9%） | 0.35 m | **136** |
| 段だけ | 449 / 416 | 0.39 / 3.49 m | 206 / 769（27%） | 0.90 m | 242 |

（前の版は全 OCC で 850 / 839（870 区間中）、今は 0.6 m 以上で広げて 0.6 m 未満になる区間 169・72 か所。）

- 帯の外の縁は、段の内側の境か、その手前の構造物の点（太らせた 1 セルの外）で止まるので、全 OCC までの距離はほぼ 0.1〜0.2 m。段だけまでの中央値が 0.39 m なのは、202d が寄せた先の段（202c の縁石と v2 の段差層を 1 セル太らせた物）と、v2 の段差層の OCC のセルがずれる所、縁石の手前に構造物の点が並ぶ所があるため。段だけで p90 が 3.49 m なのは、構造物で寄せた区間と、点の格子・構造物の点で止まって段に届かない区間。
- 帯のうち OCC から 0.45 m 以内は、Nav2 では車体の中心が入れない（inscribed）。使える面として増えるのは 152.9 m²（inflation の外は 87.9 m²）。前の版（165.3 m²、99.7 m²）より少ないのは、R1・R3・R5 で帯が減った（全体で 104.7 m²）ため。gap 0 では帯の中の格子の点も消えるので 177.8 m²、105.2 m²。
- 今の縁でも 80.9% はすでに OCC まで 0.6 m 未満（中央値 0.35 m）。手塗りの縁の外の段・構造物が、すでに inflation に掛かっていた。
- **今は 0.6 m 以上あり、広げて 0.6 m 未満になる区間（全 OCC まで）：136**。同じ線で続く区間をまとめて 58 か所（`clearance.occ.sites`。線・区間の番号、x・y の範囲、前後の最短距離、最大の広げた幅）。多いのは、北の道の西（x −76〜−67、y 154、11 区間）、x ≈ −97 の南北の道（y 95〜104 で 10 区間、y 118〜125 で 7 区間）、x ≈ −13.5、y 33〜42（9 区間）、東の道（x 132〜136、y 167〜170 と y 44）。
- 推測（この節を書いた時点では Nav2 は動かしていない）：今の地図の手塗りの縁の外は UNKNOWN で、inflation は UNKNOWN を膨らませないので、経路は手塗りの縁ぎりぎりを通りうる。広げると、縁の外は段・構造物の OCC になり、その手前に inflation のコストが付く。経路が縁から離れる向きに効く可能性がある。2026-11-19 に planner だけで確かめた結果は下の「Nav2 の planner での確認」：手塗りのままでは経路が UNKNOWN の縁に 0.25 m まで寄る所があることは当たっていた。広げた版で離れたのは正門前の手前の 1 か所だけで、R1 の箱の中の 1 か所は広げていないので残る。

#### 目視（`v2/experiments/154w/start_crop_{intensity,photo}_1to1.png` は x −15〜45、y −9〜4、`north_crop_*` は x −12〜96、y 151〜168、`west_crop_*` は x −82〜−12、y −8〜4（R1）、`dotfield_crop_*` は x −101〜−89、y 70〜98（R5）。色：青の薄塗り = 手塗りの roadway、桃 = 広げた帯（縁を濃く）、赤の点 = 段、橙の点 = 構造物、水色の点 = R6 で消した点、線 = 補正後の縁（赤 = 段、橙 = 構造物、青 = 強度、桃 = 未補正、灰 = 探していない）、水色の箱 = 除外の箱。補正後の面は 0.1 m の格子なので、帯の外の縁は 2 画素の階段になる）

2026-11-19 の版で見たこと：`west_crop_*` では、箱が道の map の −y 側だけを覆い、+y 側の帯は残る。x −80〜−77 の −y 側の帯は箱の外に残る。`dotfield_crop_*` では、点の密な所が箱の中に収まり、y 94.5 より +y 側の帯は残る。下の箇条は 2026-11-17 の版の目視で、R1・R3・R5・R6 で変わった所は括弧に書いた。

- **発進点付近**：南の縁は x 13〜20 などで、芝の手前の赤い縁石まで 0.3〜0.5 m 広がる。帯は強度の暗い（舗装）側にある。広場・斜めの入口・駐輪の帯（箱の中）には帯が無い。
- **北の道（x −12〜18）**：北・南とも、芝の島の縁石（赤）まで 0.5〜1 m 広がり、強度の暗い側に収まる。202d で GSI が芝へ入り込んでいた所は、補正後の縁（縁石）で止まる。帯の中に点の穴（小さい丸）が多い（R6 で 3 セル以下の点は消えた）。
- **北の道（x 18〜44）**：北の縁は芝の縁石まで。南（駐車場の入口）は箱で除いた。
- **北の道（x 60〜96）**：南の路肩（Q3）は補正後の縁（側溝の外の段）までで止まり、路肩の帯には入らない。北の x 66〜70 と x 92〜96 は、交わる道の口・角の縁石まで三角形に広がる（形は最近傍の区間の境で決まり、角の丸みには沿わない）。
- **植栽への入り込み**：x 104〜106、y 18〜19 は植え込みの手前で止まる。x −95、y 76〜92 の点の密な帯の中には、上のとおり 3.65 m² が残る。

**要判断だった所**（2026-11-17 の版の値と内容。2026-11-19 に R1〜R5 で決まった。今の値は上の「R1〜R6 ごとの数字」）：

| 所 | 帯（11-17） | 内容 | 回答 |
| --- | --- | --- | --- |
| 駐輪・駐車の帯の箱の外（x −75〜−10、y −6〜−1） | 15.49 m² | 発進点の道の map の −y 側（西）の駐輪の帯は x −75〜+5 まで続く。孤立した赤・橙の区間に面した出っ張りが残っていた：x −16〜−13（構造物＝駐輪の列の前面）、x −35〜−32（段、駐輪の列の前に入る）、x −74〜−48（段まで 0.5〜0.8 m） | R1 除く |
| 同じ帯の東（x 0〜5） | 0.48 m² | 箱のすぐ +x 側の小さな出っ張り | R1 除く |
| 北の道の南（x 44〜60） | 1.97 m² | 駐車場の入口の箱の +x 側のすぐ外。駐車場・建物の前庭との境の段までの出っ張り | R2 残す |
| 上限で切れる所（x −96〜−93、y 12〜48） | 36.80 m² | 補正後の縁（x ≈ −98.5 の縁石）が手塗りの縁から 5 m 離れ、間は舗装で、手塗りの縁の 1 m 外に白線がある。帯は白線を越えて 1.5 m の所で直線で切れていた | R3 広げない |
| 北へ延びる道の両側（x 21〜27、y 5〜16） | 6.40 m² | 強度ではやや明るく、写真（約 0.48 m/画素）では舗装より暗い。側溝か背の低い植栽か区別できない（未確認） | R4 残す |
| 点の密な帯（x −96.5〜−93.5、y 76〜92） | 3.65 m² | 構造物の点の間の隙間に残る帯 | R5 除く |

#### 採用（2026-11-19、R7）

ユーザーの判断（R7）：既定の地図をこの広げた版に差し替え、次の屋外の試験で使う。手塗りのままの地図は消さず、ファイルかパラメータの切り替えで戻せるように残す。R6 は gap 1（スクリプトの既定）の版。ブランチ `feat/154-adopt-widened-map`（#210 の上に積む）で行い、マージはユーザーが行う。

runtime が読む物と変えた物：

- Nav2（map_server）が読むのは `nav_launch.py` が解決する `occupancy_<variant>.yaml` と、その `image` の pgm だけ。campus-m8 の既定の variant は `candidate`（`_DEFAULT_VARIANT_BY_SITE`、`start_dispatch_demo.sh` の `demo_variant_for`、`check_map_sync.py` の `DEMO_VARIANT_BY_SITE`）。`occupancy_candidate.yaml` は `v2/candidate_final.pgm` を指したまま変えず、pgm の中身を広げた版に差し替えた。launch・パラメータ・variant の対応は変えていない。
- 配車 Web の背景（`src/whill_dispatch/web/map.png`、`map_meta.json`）は広げた版で作り直した（`source_pgm` は同じ `v2/candidate_final.pgm`）。
- NDT（localizer）は `static.pcd` を読み、占有格子は読まない。`static.pcd`・waypoint・TF・校正値は変えていない。

| ファイル | 中身 | sha256 |
| --- | --- | --- |
| `v2/candidate_final.pgm` | 広げた版（既定の `candidate`）。`~/bags/m8-experiments/154w/compose/final.pgm` とバイト単位で同じ | `fca2abb47a4af7ae987784f976a14b7d51ccf5adfeae1e9190a3b7b959fcaa9e` |
| `v2/candidate_hand_final.pgm` | 手塗りのまま（`candidate_hand`）。1a4f327 の `v2/candidate_final.pgm` と同じ | `5106c0bca46650a40a491ca611cabdb0eed470ec96cfca5db5778813fc1c41d9` |
| `v2/candidates/candidate_roadway_merged.png` | 広げた roadway（`outputs_sha256.roadway_widened`） | `abcc68ba4cd0814625046a06b9d0a9191168da9e44f9345a6059cf34be0acdf4` |
| `v2/candidates/candidate_free_merged.png` | 広げた free（`outputs_sha256.free_widened`） | `456d692205a58f129192a93b7ade81b39de5bbf0ec54de07de8e983d391f0826` |
| `v2/candidates/candidate_roadway_merged_hand.png` | 広げる前の roadway（`inputs_sha256.hand_roadway`） | `7441d3d19ba875e3c04b2548ec0cd508e56369c5e2b01b9a56920d974448861d` |
| `v2/candidates/candidate_free_merged_hand.png` | 広げる前の free（`inputs_sha256.hand_free`） | `ba775314a8df0980fefc5e856640accd833805272377acc006bff02a16c95207` |
| `occupancy_candidate_hand.yaml` | `occupancy_candidate.yaml` と同じ値で、`image: v2/candidate_hand_final.pgm` | — |

`candidate_keepout_mask.png` は空のままで変えていない。`*_hand` の 3 つは 1a4f327 の版とバイト単位で同じなので、Git の中身は増えない。`map_widen_hand_roadway.py` の入力と `--baseline-map` の既定は `*_hand` に変えた（入力の既定が広げた版のままだと、既定の実行が二重に広げる止めで止まる）。既定のまま実行して、出力の sha256 が 3 つとも `outputs_sha256` と同じで、`checks` が全て成り立つことを確かめた（`summary.json` の数字は、入力のパスの記録以外は変わらない。`summary.json` は差し替えていないので、`params.hand_roadway` などは採用前のパスのまま）。

実行したコマンド（2026-11-19）：

```bash
cd ~/whill_lab0_ros2   # 採用用のブランチの checkout
D=docs/maps/campus-m8
PRE=1a4f327            # 採用前の手塗りの版
IN=/tmp/154w_in; OUT=/tmp/154w_out; WORK=/tmp/154w_work; T=/tmp/154w_compose
mkdir -p $IN $OUT $WORK $T
# 1. 採用前の手塗りと、比べる相手の地図を取り出し、入力の sha256 が inputs_sha256 と同じことを確かめる
git show $PRE:$D/v2/candidates/candidate_roadway_merged.png > $IN/candidate_roadway_merged.png
git show $PRE:$D/v2/candidates/candidate_free_merged.png > $IN/candidate_free_merged.png
git show $PRE:$D/v2/candidates/candidate_keepout_mask.png > $IN/candidate_keepout_mask.png
git show $PRE:$D/v2/candidate_final.pgm > $IN/candidate_final.pgm
git show $PRE:$D/occupancy_candidate.yaml | sed 's|^image: v2/candidate_final.pgm|image: candidate_final.pgm|' > $IN/occupancy_candidate.yaml
sha256sum $IN/candidate_roadway_merged.png $IN/candidate_free_merged.png   # 7441d3d1… / ba775314…
# 2. 154w を採用前の版から作り直す。docs の summary.json と ~/bags の出力を上書きしないよう、出力先は一時の所
python3 scripts/map_widen_hand_roadway.py --hand-roadway $IN/candidate_roadway_merged.png \
  --hand-free $IN/candidate_free_merged.png --keepout $IN/candidate_keepout_mask.png \
  --baseline-map $IN/occupancy_candidate.yaml --out-dir $OUT --work-dir $WORK --no-gimp
sha256sum $WORK/roadway_widened.png $WORK/free_widened.png $WORK/compose/final.pgm   # abcc68ba… / 456d6922… / fca2abb4…
# 3. 手塗りのままの版を *_hand の名前で残す
cp $D/v2/candidate_final.pgm $D/v2/candidate_hand_final.pgm
cp $D/v2/candidates/candidate_roadway_merged.png $D/v2/candidates/candidate_roadway_merged_hand.png
cp $D/v2/candidates/candidate_free_merged.png $D/v2/candidates/candidate_free_merged_hand.png
sed 's|^image: v2/candidate_final.pgm|image: v2/candidate_hand_final.pgm|' $D/occupancy_candidate.yaml > $D/occupancy_candidate_hand.yaml
# 4. 手塗りの候補を広げた版に差し替え、3 節の「再現」の最後の 2 段と同じ合成で candidate_final.pgm を作り直す
cp $WORK/roadway_widened.png $D/v2/candidates/candidate_roadway_merged.png
cp $WORK/free_widened.png $D/v2/candidates/candidate_free_merged.png
scripts/gen_auto_free_mask.py --layers-yaml $D/v2/v2_layers.yaml --roadway-mask $D/v2/candidates/candidate_roadway_merged.png \
  --output-free-mask-auto $T/free_mask_auto.png --output-salt-csv $T/salt.csv
scripts/compose_occupancy.py --layers-yaml $D/v2/v2_layers.yaml \
  --roadway-mask $D/v2/candidates/candidate_roadway_merged.png --free-mask $D/v2/candidates/candidate_free_merged.png \
  --free-mask-auto $T/free_mask_auto.png --keepout-mask $D/v2/candidates/candidate_keepout_mask.png \
  --output-pgm $D/v2/candidate_final.pgm
# 5. 154w の地図と 1 セルも違わないこと、残した版が採用前と同じこと、配車 4 地点
cmp $D/v2/candidate_final.pgm ~/bags/m8-experiments/154w/compose/final.pgm && echo "154w と同一"
cmp $D/v2/candidate_hand_final.pgm $IN/candidate_final.pgm && echo "手塗りのまま = 1a4f327"
scripts/map_route_check.py $D/occupancy_candidate.yaml $D/waypoints.yaml
scripts/map_route_check.py $D/occupancy_candidate_hand.yaml $D/waypoints.yaml
# 6. 二重に広げる止めが効くこと（差し替えた後の candidate_roadway_merged.png を渡すと何も書かずに止まる）
python3 scripts/map_widen_hand_roadway.py --hand-roadway $D/v2/candidates/candidate_roadway_merged.png \
  --work-dir /tmp/154w_guard --no-gimp
# 7. Web の背景を作り直し、Nav2 の地図との同期を確かめる
python3 scripts/m7_make_web_map.py --yaml $D/occupancy_candidate.yaml --out-dir src/whill_dispatch/web
python3 scripts/check_map_sync.py --web-dir src/whill_dispatch/web
```

結果：5. は両方とも同一、`map_route_check.py` は両方とも `all_connected: true`（広げた版は成分 13 個、手塗りのままは 7 個）。6. は「前回の広げた版（summary.json の sha256）と同じ。二重に広げないよう止める」で止まり、何も書かなかった。7. は ok。

- `compose_occupancy.py` は `candidate_final.yaml` も書く（3 節の再現と同じ）が、中身は変わらない。`erased_by_free.png`・`clipped_by_roadway.png` は Git 管理外。
- 差し替えた手塗りの roadway には、R6 で戻した 3 セル以下の点が入るので、手順 4. の `gen_auto_free_mask.py` が手塗りの内側と同じく消す（`compose/` の合成と同じ）。手塗りの free には入れていない。残る穴は 44 個、732 セル（`derived.new_holes_in_roadway_mask`。帯の中に残した点の格子・塊とその周りの 1 セル、うち OCC 104 セル）で、穴の OCC は合成でも OCC、残りは UNKNOWN になる。
- 手塗りの確定（`v2/roadway_mask.png` 等、3 節の「ユーザーが確かめること」4.）とは別の段階。確定するときは、広げた版と手塗りのままのどちらを元にするかを決める。

戻し方：

- その場で一時的に戻す（ファイルは変えない）：Nav2 を `map_variant:=candidate_hand` で起動し、Web の背景を手塗りのままの版で作り直す。コマンドは [runbook](../../runbook-live-demo.md)「手塗りのままの地図（candidate_hand）に戻す」。
- 既定そのものを戻す（ファイルを差し替える。ブランチと PR で行う）：

```bash
cd ~/whill_lab0_ros2   # 戻す用のブランチの checkout
source /opt/ros/humble/setup.bash
D=docs/maps/campus-m8
cp $D/v2/candidate_hand_final.pgm $D/v2/candidate_final.pgm
cp $D/v2/candidates/candidate_roadway_merged_hand.png $D/v2/candidates/candidate_roadway_merged.png
cp $D/v2/candidates/candidate_free_merged_hand.png $D/v2/candidates/candidate_free_merged.png
sha256sum $D/v2/candidate_final.pgm   # 5106c0bca46650a40a491ca611cabdb0eed470ec96cfca5db5778813fc1c41d9
python3 scripts/m7_make_web_map.py --yaml $D/occupancy_candidate.yaml --out-dir src/whill_dispatch/web
colcon build --symlink-install --packages-select whill_dispatch
python3 scripts/check_map_sync.py --web-dir install/whill_dispatch/share/whill_dispatch/web
scripts/map_route_check.py $D/occupancy_candidate.yaml $D/waypoints.yaml
```

  この採用のコミットを `git revert` しても同じ（`*_hand` と `occupancy_candidate_hand.yaml` も消える。`map_widen_hand_roadway.py` の既定の入力も元に戻る）。

#### Nav2 の planner での確認（2026-11-19、オフライン）

`scripts/nav2_plan_check.py` で、地図ごとに map_server と planner_server（global_costmap を内に持つ）だけを起動し、配車 4 地点の順序つきの 12 組で `ComputePathToPose`（`use_start: true`）を呼んだ。パラメータは `src/whill_navigation/config/nav2_params.yaml` をそのまま読む（NavFn、`tolerance` 0.5、`allow_unknown: false`、global_costmap は static_layer + inflation_layer、`robot_radius` 0.45 m、`inflation_radius` 0.6 m、`cost_scaling_factor` 4.0）。外した層は無い（global_costmap にはもともと obstacle_layer が無い）。map → base_link は static TF の恒等。controller・velocity_smoother・twist_mux は起動していないので cmd_vel は出ない。domain 87、`rmw_cyclonedds_cpp`、`configs/cyclonedds-runtime-min.xml`。smoother・controller（RPP）・local costmap・実機の自己位置は見ていない。

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
python3 scripts/nav2_plan_check.py \
  --map docs/maps/campus-m8/occupancy_candidate_hand.yaml --map docs/maps/campus-m8/occupancy_candidate.yaml \
  --waypoints docs/maps/campus-m8/waypoints.yaml --out-dir /tmp/nav2_plan_check
```

列：長さは経路の点列の折れ線の長さ。OCC まで・非 FREE までは経路の点から最も近い OCC / FREE でない（UNKNOWN か OCC の）セルまでの距離の最小（地図の pgm から。中心どうしなので ±0.05 m）。ずれは、広げた版の経路の各点から手塗りのままの経路への最短距離の平均 / 最大（最大は両向きの Hausdorff）。

| 組（始点 → 終点） | 成功（手塗り / 広げた） | 長さ [m]（手塗り → 広げた） | OCC まで最小 [m] | 非 FREE まで最小 [m] | ずれ 平均 / 最大 [m] |
| --- | --- | --- | --- | --- | --- |
| kounoken → bldg7 | ○ / ○ | 38.49 → 38.49 | 1.85 → 1.85 | 1.60 → 1.60 | 0 / 0 |
| kounoken → seimon | ○ / ○ | 88.87 → 88.85 | 0.68 → 0.76 | **0.29 → 0.56** | 0.025 / 0.38 |
| kounoken → bldg3 | ○ / ○ | 68.22 → 68.22 | 1.25 → 1.25 | 1.20 → 1.20 | 0 / 0 |
| bldg7 → kounoken | ○ / ○ | 38.53 → 38.53 | 1.80 → 1.80 | 1.55 → 1.55 | 0 / 0 |
| bldg7 → seimon | ○ / ○ | 50.37 → 50.34 | 0.68 → 0.67 | **0.25 → 0.50** | 0.024 / 0.26 |
| bldg7 → bldg3 | ○ / ○ | 106.62 → 106.62 | 0.62 → 0.62 | 0.50 → 0.50 | 0 / 0 |
| seimon → kounoken | ○ / ○ | 89.19 → 89.19 | 0.89 → 0.89 | **0.25 → 0.25** | 0 / 0 |
| seimon → bldg7 | ○ / ○ | 50.70 → 50.70 | 0.90 → 0.90 | **0.25 → 0.25** | 0 / 0 |
| seimon → bldg3 | ○ / ○ | 157.26 → 157.26 | 0.70 → 0.70 | **0.25 → 0.25** | 0 / 0 |
| bldg3 → kounoken | ○ / ○ | 68.03 → 68.03 | 1.45 → 1.66 | 1.30 → 1.43 | 0 / 0 |
| bldg3 → bldg7 | ○ / ○ | 106.51 → 106.51 | 0.66 → 0.66 | 0.60 → 0.60 | 0 / 0 |
| bldg3 → seimon | ○ / ○ | 156.85 → 156.83 | 0.66 → 0.66 | **0.29 → 0.50** | 0.007 / 0.17 |

- 12 組とも両方の地図で計画できた。経路が OCC から 0.6 m（inflation の半径）より近づく所は、どの組にも無い（最小 0.62 m）。経路上の global costmap のコストはどの組も 0。
- 経路が変わったのは正門前（seimon）へ向かう 3 組だけで、ずれは最大 0.38 m、長さの差は 0.02〜0.03 m。他の 9 組は 1 点も変わらない（bldg3 → kounoken は経路は同じで、その脇の縁が広がって OCC・非 FREE までの距離が増えた）。NavFn はコスト 0 の面で最短の経路を取るので、広げた帯がカーブの内側に無ければ経路は動かない。
- **手塗りのままでは、経路が UNKNOWN の縁に 0.25〜0.29 m（`robot_radius` 0.45 m より近い）まで寄る所が 2 か所ある**。inflation は UNKNOWN を膨らませない（Humble の InflationLayer は LETHAL だけを膨らませる）ので、global の経路は車体の中心を手塗りの縁の 0.25 m まで寄せる。
  - x −80.0〜−80.9、y −4.6〜−5.0（正門前の手前、map の −y 側、約 1 m）：seimon へ向かう 3 組。広げた版では 0.50〜0.56 m に離れた（R1 の箱 x −77〜+6 の外で、帯が広がった所）。
  - x −67.1〜−64.6、y ≈ −3.0（正門前から発進点への道、map の −y 側、約 2.5 m）：seimon から出る 3 組。広げた版でも 0.25 m のまま。R1 の箱（駐輪・駐車の帯）の中で広げていないため。
- 残る 1 か所は、global の経路としては計画できるが、車体の円（0.45 m）が UNKNOWN に 0.2 m 掛かる。local costmap の static_layer と RPP の衝突検知がこれをどう扱うかは、オフラインでは確かめていない（屋外で、seimon から出発する組で見る）。
- `nav2_plan_check.py` は経路を重ねた原寸の切り出し（`plans_crop.png`、手塗りのまま = 青、広げた版 = 赤）と全組の点列（`plans.json`）を `--out-dir` に書く（Git 管理外）。

#### 置き場（154w）

| ファイル | 中身 | Git |
| --- | --- | --- |
| `v2/experiments/154w/summary.json` | パラメータ（パスはリポジトリ相対・`~/`）、入力・出力の sha256、確かめたこと（`checks`）、除外の箱、帯の各段階のセル数、除外の箱ごとの実測（`q2_boxes`）、R3 の 1 回目で上限まで広がった区間（`capped`、区間ごとの一覧つき）、R6 で消した点（`band_salt`）、比べるときの主な数字（`key`）と gap 0 の版の同じ数字（`compare`）、面積、要判断だった所の帯の面積、広げた区間の数と幅、この節の数字の出所（`derived`）、Nav2 の値、inflation の余裕（`clearance`）、`map_route_check.py` の結果 | 管理 |
| `v2/experiments/154w/widened_segments.csv` | 広げた区間ごと（線・区間の番号、補正後の中点、根拠、寄せた量、帯のセル数、広げた幅の最大・中央値、明るいセルの割合、新しい縁・今の縁のセル数と、全 OCC・段だけまでの最短距離） | 管理 |
| `v2/experiments/154w/{start,north,west,dotfield}_crop_{intensity,photo}_1to1.png` | 原寸。上の「目視」の画像（west・dotfield は 2026-11-19 に足した） | 管理 |
| `v2/experiments/154w/overview_band_8.png` | 1/8。今の FREE（薄青）、OCC（灰）、帯（桃、見えるように 1 画素太らせた） | 管理 |
| `~/bags/m8-experiments/154w/band.png`, `roadway_widened.png`, `free_widened.png` | 原寸（白・不透明が塗り。compose_occupancy の形式）。帯だけ（R6 の点を含む）、広げた roadway（R6 の点を含む）、広げた free（R6 の点を含まない） | 管理外 |
| `~/bags/m8-experiments/154w/compose/final.pgm`, `final.yaml` | 合成した候補の地図（と `free_mask_auto.png`、`salt.csv`、`erased_by_free.png`、`clipped_by_roadway.png`） | 管理外 |
| `~/bags/m8-experiments/154w_gap0/` | R6 を gap 0（生の 8 連結）で作った比べる版。`summary.json`、切り出し、`compose/final.{pgm,yaml}` など、上と同じ物（GIMP の 19 番は書かない） | 管理外 |
| `~/bags/m8-experiments/154w/compose_hand/` | 手塗りのまま同じ手順で合成した地図（`--baseline-map` と同じことの確認用。採用後の既定は `occupancy_candidate_hand.yaml`） | 管理外 |
| `~/bags/m8-experiments/154w/run.log` | 実行記録（`/usr/bin/time -v`） | 管理外 |
| `~/bags/m8-experiments/gimp-layers/19_widened_candidate.png` | GIMP 用の原寸 RGBA（帯 = 桃、R6 で消した点 = 水色）。`campus-m8_layers.xcf` に表示で入れた | 管理外 |

再現（1 回約 48 秒、最大メモリ 3.6 GB。gap 0 の版を先に作り、既定の版に `--compare-summary` で数字を写す。xcf の作り直しは約 15 秒）。入力の `~/bags/m8-experiments/202d/` は上の 202d の再現で作る。`build_xcf.sh` の `layers.txt` には 2026-11-17 に 19 番の 1 行を足し、2026-11-19 にレイヤー名を `19_widened_candidate|手塗りを補正後の地理院の縁まで広げた帯（桃、消した小さい点＝水色、154w の候補）|1` に変えた。R1〜R6 の前の版を作るときは `--exclude-box` で前の 4 つの箱を渡し、`--capped-policy clip --band-salt-max-cells 0` を付ける（2026-11-19 に、672bd81 の `outputs_sha256` の 3 つと sha256 が一致することを確かめた）：

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout
mkdir -p ~/bags/m8-experiments/154w
mkdir -p ~/bags/m8-experiments/154w_gap0
/usr/bin/time -v python3 scripts/map_widen_hand_roadway.py --band-salt-gap-cells 0 \
  --out-dir ~/bags/m8-experiments/154w_gap0 --work-dir ~/bags/m8-experiments/154w_gap0 --no-gimp \
  > ~/bags/m8-experiments/154w_gap0/run.log 2>&1
/usr/bin/time -v python3 scripts/map_widen_hand_roadway.py --compare-summary ~/bags/m8-experiments/154w_gap0/summary.json \
  > ~/bags/m8-experiments/154w/run.log 2>&1
bash ~/bags/m8-experiments/gimp-layers/build_xcf.sh   # Git 外のスクリプト（このリポジトリには無い）
scripts/run_tests.sh --scripts-only
```

`map_widen_hand_roadway.py` は既定で、docs の `v2/experiments/154w/`、`~/bags/m8-experiments/154w/`、`~/bags/m8-experiments/gimp-layers/19_widened_candidate.png` を上書きする。パラメータを試すときは `--out-dir`・`--work-dir` を変え、`--no-gimp` を付ける。採用（2026-11-19）の後は、入力と `--baseline-map` の既定が `*_hand` を指すので、上のコマンドはそのまま採用前の版から作る。採用の後に上のコマンドで `summary.json` を書き直すと、`params` の入力のパスだけが `*_hand` に変わる。

#### 確かめていないこと

- controller（RPP）・local costmap・実機の自己位置での走り、帯の中に残る OCC の点の影響。global の planner だけは 2026-11-19 にオフラインで確かめた（上の「Nav2 の planner での確認」）。特に、seimon から出る組で経路が UNKNOWN の縁に 0.25 m まで寄る所（x −67〜−65、y ≈ −3、R1 の箱の中）を、屋外で RPP がどう扱うか。
- 北へ延びる道の帯（R4 で残した）が側溝か植栽か、点の密な帯（R5 で除いた）の中身。現地か高解像度の写真が要る。
- R6 を gap 1（既定、格子を残す）と gap 0 のどちらにするか（ユーザーが決め直す）。gap 0 で余分に消える格子の点（1,423 セル）が、生垣・柵か塩かは現地で確かめていない。
- 発進点付近・北の道・上の点検箇所以外の目視。
- `--max-widen-m`、`--tie-m`、`--barrier-dilate-m`、`--hole-max-m2` の掃引。

## 4. waypoint とローカライザの再生確認（#155、オフラインの部分）

### waypoint・初期位置・配車地点

旧 `campus/waypoints.yaml` の4点を T_new_old で写し、`waypoints.yaml` に置いた（出所と丸めはファイルの冒頭）。

| 地点 | 旧 (x, y) | 新 (x, y) | 移動量 | 局所差の目安 | 新しい候補地図で |
| --- | --- | --- | --- | --- | --- |
| kounoken（工農研横、発進点） | (0.00, 0.00) | (−0.01, 0.06) | 0.066 m | 0.06 m | FREE（254） |
| bldg7（7号館前） | (−38.42, −0.47) | (−38.43, −0.30) | 0.169 m | 0.08 m | FREE（254） |
| seimon（正門前） | (−87.72, −7.72) | (−87.76, −7.40) | 0.314 m | 0.23 m | FREE（254、直接の拡幅を写した部分） |
| bldg3（3号館前） | (67.77, 2.37) | (67.76, 2.27) | 0.098 m | 0.02 m | FREE（254） |

- 「局所差の目安」は、その点に最も近い旧軌跡の点で、剛体変換で吸収しきれなかった両地図の差（`align_from_campus/align.json`）。seimon は最寄りの軌跡から 7.2 m 離れているので目安の信頼性が低い。
- **初期位置**：運用では発進点（kounoken、map の原点付近）で RViz の 2D Pose Estimate を打つ。新しい地図でも原点は発進点付近（ずれ 0.07 m、yaw −0.10°）で、手順は変わらない。`m6r_smoke_test.sh` の初期位置 (0, 0, 0) もそのまま使える。
- **配車の Web 背景**：`src/whill_dispatch/web/map_meta.json` の origin（[−128.25, −90.35]）と、格子との関係（0.05 m の格子を 8 分の 1 に縮めた 0.4 m、830×786）は、新しい格子にもそのまま当てはまる（旧 v2 と同じ格子にしたため）。解像度そのものは Web が 0.4 m、地図が 0.05 m で、同じではない。当初は「地図を確定した後に作り直す」としていたが、2026-11-13 に既定の site にしたので、候補の `occupancy_candidate.yaml` から `scripts/m7_make_web_map.py` で作り直した（`source_pgm` は `docs/maps/campus-m8/v2/candidate_final.pgm`、`scripts/check_map_sync.py` は ok）。手塗りを確定したら、もう一度作り直す。
- 地点の名前の変更（#170）はこの作業に含まない。

### NDT の再生

07-24 の走行 bag（`~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26`。回帰チェックの基準 bag）の点群を #152 と同じ方法で補正し（`...-19_05_26-hires/`、8309 個）、デモと同じローカライザの設定で再生した。地図・TF・点群の組み合わせを変えて比べる。初期位置は bag に記録された最初の `/pcl_pose`（旧地図の座標）を T_new_old で写したもの。

#### 07-24 の bag の補正（所要 6 分 29 秒、出力 5.3 GB）

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout（worktree でもよい）
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
P=$(ros2 pkg prefix velodyne_pointcloud)/share/velodyne_pointcloud/params
D=~/bags/m7-bench-data/2026-07-24-live-campus
python3 scripts/velodyne_reproject_bag.py $D/rosbag2_2026_07_24-19_05_26 $D/rosbag2_2026_07_24-19_05_26-hires \
  --src-calib $P/VLP16db.yaml --dst-calib $P/VLP16_hires_db.yaml
```

結果：点群 8309 個を補正、他のメッセージ 191113 個を複写、z の再計算の食い違いは最大 3.48 µm。元の bag は読むだけ。

#### 再生の6本

実際の実行は、下と同じ引数を渡す一時的なラッパーで行った。3本（`new_map_old_tf_orig`、`full_*` の2本）はラッパーの呼び出しの記録から、残る3本は各実行の `localizer.log` / `play.log`（開いた bag と static.pcd のパス、TF の出し方）から引数を戻した**再構成**。どれも今の `ndt_replay_compare.py`（引数は当時と同じ）で同じ条件になる。旧地図は main の checkout（`~/whill_lab0_ros2/docs/maps`）から、新しい地図はこのブランチの `docs/maps` から読んだ。表の「#145 の前の launch」の TF は、当時の `~/whill_lab0_ros2/install` の `static_tf_launch.py`（このブランチに install が無いため main 側のビルド。#145 を入れる前の値）。表の「#145（11-12）」は、下のコマンドの `NEW_TF`（2026-11-12 時点の #145 の値）。

```bash
cd ~/whill_lab0_ros2   # このブランチの checkout（worktree なら、そのディレクトリ）
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
# 車載 PC の runbook の domain（0）と分ける。ndt_replay_compare.py は domain が空でなければ止まる。
export ROS_DOMAIN_ID=153 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
B=~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26
T=docs/maps/campus-m8/align_from_campus/T_new_old.txt
OLD_MAPS=~/whill_lab0_ros2/docs/maps   # 旧 campus（main の checkout）
NEW_TF="0.42 0.36 0.777 0.24 0.97 -0.317"   # 2026-11-12 時点の #145 の値。2026-11-13 に変わった（下の「2026-11-13 の再比較」）
O=~/ndt_replay_$(date +%Y%m%d_%H%M%S); mkdir -p $O   # 出力先は毎回新しくする（既にあると止まる）
# 240 秒（bag の 73〜313 秒）
python3 scripts/regression/ndt_replay_compare.py --bag $B       --site campus    --maps-root $OLD_MAPS --start 73 --duration 240 --out $O/old_map_old_tf_orig
python3 scripts/regression/ndt_replay_compare.py --bag $B-hires --site campus    --maps-root $OLD_MAPS --lidar-tf "$NEW_TF" --start 73 --duration 240 --out $O/old_map_new_tf_hires
python3 scripts/regression/ndt_replay_compare.py --bag $B       --site campus-m8 --maps-root docs/maps --map-transform $T --start 73 --duration 240 --out $O/new_map_old_tf_orig
python3 scripts/regression/ndt_replay_compare.py --bag $B-hires --site campus-m8 --maps-root docs/maps --lidar-tf "$NEW_TF" --map-transform $T --start 73 --duration 240 --out $O/new_map_new_tf_hires
# 全区間（bag の 73〜814 秒）
python3 scripts/regression/ndt_replay_compare.py --bag $B       --site campus    --maps-root $OLD_MAPS --start 73 --duration 741 --out $O/full_old_map_old_tf_orig
python3 scripts/regression/ndt_replay_compare.py --bag $B-hires --site campus-m8 --maps-root docs/maps --lidar-tf "$NEW_TF" --map-transform $T --start 73 --duration 741 --out $O/full_new_map_new_tf_hires
```

- 1本あたり 240 秒の区間で約 5 分、全区間で約 13 分。6本は順に回す（同じ domain を使うので並べられない）。
- `ndt_replay/*.json` は各実行の `result.json` の写し。bag・maps_root・版（`git`）とパスの書き方は、再生の後に手で直した（各 JSON の `provenance_note`）。数値は実行したときのまま。

`--lidar-tf` は `replay_localizer_launch.py` の新しい引数で、base_link→velodyne だけをその値で出す（#145 の launch を変える前に比べるため。既定の空なら従来どおりブランチの `static_tf_launch.py`）。

以下の 2 つの表と「読み取れること」は 2026-11-12 の実行で、TF は当時の値。記録として残す。

走行中の 240 秒（bag の 73〜313 秒。回帰チェックと同じ区間）：

| 地図 | TF | 点群 | fitness 中央値 / p95 / 最大 | 閾値超えの警告 | 姿勢の間隔の最大 | 記録との差 xy 中央値 / p95 |
| --- | --- | --- | --- | --- | --- | --- |
| 旧 campus | #145 の前の launch | 元 | 0.0225 / 0.131 / 8.16 | 4 | 1.42 s | 0.000 / 0.052 m |
| 旧 campus | #145（11-12） | Hi-Res | 0.0976 / 0.195 / 0.71 | 0 | 1.52 s | 0.059 / 0.123 m |
| campus-m8 | #145 の前の launch | 元 | 0.343 / 0.496 / 3.20 | 0 | 0.91 s | 0.052 / 0.139 m |
| **campus-m8** | **#145（11-12）** | **Hi-Res** | **0.0163 / 0.038 / 0.62** | **0** | **0.81 s** | 0.079 / 0.156 m |

走行の全区間（bag の 73〜814 秒、741 秒）：

| 地図 | TF | 点群 | fitness 中央値 / p95 / 最大 | 閾値超えの警告 | 姿勢の間隔の最大 | 記録との差 xy 中央値 / p95 / 最大 |
| --- | --- | --- | --- | --- | --- | --- |
| 旧 campus | #145 の前の launch | 元 | 0.0306 / 0.199 / 10.47 | 6 | 1.12 s | 0.000 / 0.123 / 0.683 m |
| **campus-m8** | **#145（11-12）** | **Hi-Res** | **0.0154 / 0.062 / 0.66** | **0** | **0.81 s** | 0.063 / 0.164 / 0.966 m |

全区間でも、閾値超え 0 回、収束しなかった回 0 回、姿勢 7299 個（記録は 7298 個）。

読み取れること：

- 新しい地図・#145 の TF・Hi-Res の点群の組み合わせが、fitness が最も小さい（旧の組み合わせに対して、240 秒で中央値 0.7 倍・p95 0.3 倍、全区間で中央値 0.5 倍・p95 0.3 倍）。閾値（6.0）を超えた回はなく、収束しなかった回もない。旧の組み合わせは同じ bag で閾値超えが 4〜6 回ある。各実行の結果は `ndt_replay/*.json`。
- 旧の組み合わせ（1行目）から変えた要素の数で見ると：
  - **地図だけを変えた**（campus-m8・#145 の前の launch・元の点群）：fitness 中央値が 15 倍（0.0225 → 0.343）、p95 は 3.8 倍に悪化。閾値超えは 4 → 0 回、最大は 8.16 → 3.20 で、外れ値は減ったが全体の当てはまりは大きく悪い。
  - **TF と点群の2つを変えた**（旧 campus・#145・Hi-Res）：中央値は約 4 倍（0.0225 → 0.0976）、p95 は 1.5 倍に悪化。一方で閾値超えは 4 → 0 回、最大は 8.16 → 0.71 になった。良し悪しは指標によって分かれる。
  - TF だけ、点群だけを変えた組み合わせは回していない。
- 地図だけを変えた行から、**#153 だけを先にマージすると当てはまりが大きく悪くなる**。#144・#145 だけを先に入れた場合（2行目）も、中央値と p95 は旧の組み合わせより悪い。3つをそろえた行だけが中央値・p95・最大・閾値超えのすべてで旧より良いので、ロードマップの方針どおり、#144・#145・#153 は一緒にマージする必要がある。
- **各条件は1回ずつしか回しておらず、実行ごとのばらつきは評価していない**。同じ条件の 240 秒の結果（記録との差 xy の最大 1.081 m）が、その区間を含む全区間の結果（0.966 m）より大きいことから、実行ごとに結果が変わることは分かっている（決定的なら全区間の最大は区間の最大以上になる。原因は調べていない）。上の倍率は1回ずつの比較であり、小さい差（例えば p95 の 1.5 倍）は、ばらつきの範囲かもしれない。
- 「記録との差」は旧地図での姿勢を新しい座標に写したものとの差で、地図の違いと TF の違い（LiDAR の位置が 6 cm 変わる）を含む。正しさの指標ではない。
- fitness は TF を 10 cm ずらしても変わらなかった記録がある（[回帰チェック](../../regression-check.md)）。fitness が小さいことは地図と点群がよく重なることを示すが、位置が正しいことの証明ではない。

#### 2026-11-13 の再比較（LiDAR をテープで固定した後の TF）

2026-11-13 に LiDAR をテープで固定し、固定後の実機静止確認で base_link→velodyne が (0.42, 0.36, **0.771**, RPY **−0.05°, +0.40°**, −0.3167°) に変わった（上の表の (0.777, +0.24°, +0.97°) から。出典は #186 の `docs/calibration-ledger.md` §2.1 の C）。冒頭の「TF・校正が変わったときに作り直しが要るか」のとおり地図は base_link→velodyne を使っていないので作り直さず、`--lidar-tf` だけを変えて回し直した。あわせて、ビルドの違いで数値が変わらないことを確かめるため、2026-11-12 の TF で新しい地図の 240 秒を 1 本回し直した（d）。

条件：#186 と #190 を main（f35fe86）に merge したローカルの統合ブランチ（b74bdf2。push していない）の worktree で `colcon build --symlink-install` し、その install を source した。他は上と同じ（domain 153、localhost、CycloneDDS、`CYCLONEDDS_URI` なし、07-24 の bag の Hi-Res 補正版、`--start 73`、旧地図は main の checkout、新しい地図は worktree の `docs/maps` に正本の static.pcd を複製して sha256 一致を確認）。結果は `ndt_replay/2026-11-13/*.json`。

```bash
cd ~/whill_lab0_ros2/.claude/worktrees/m8-integration   # #186 と #190 を merge した worktree（ビルド済み）
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=153 ROS_LOCALHOST_ONLY=1 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI
B=~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26
T=docs/maps/campus-m8/align_from_campus/T_new_old.txt
OLD_MAPS=~/whill_lab0_ros2/docs/maps
NEW_TF="0.42 0.36 0.771 -0.05 0.40 -0.3167"   # 2026-11-13（テープ固定後）
PREV_TF="0.42 0.36 0.777 0.24 0.97 -0.317"    # 2026-11-12（上の表と同じ文字列）
O=~/bags/m5r-bench-data/2026-07-10-campus-outer-final/ndt-compare-20261113; mkdir -p $O
python3 scripts/regression/ndt_replay_compare.py --bag $B-hires --site campus    --maps-root $OLD_MAPS --lidar-tf "$NEW_TF" --start 73 --duration 240 --out $O/a_old_map_new_tf_hires_240
python3 scripts/regression/ndt_replay_compare.py --bag $B-hires --site campus-m8 --maps-root docs/maps --lidar-tf "$NEW_TF" --map-transform $T --start 73 --duration 240 --out $O/b_new_map_new_tf_hires_240
python3 scripts/regression/ndt_replay_compare.py --bag $B-hires --site campus-m8 --maps-root docs/maps --lidar-tf "$PREV_TF" --map-transform $T --start 73 --duration 240 --out $O/d_new_map_prev_tf_hires_240
python3 scripts/regression/ndt_replay_compare.py --bag $B-hires --site campus-m8 --maps-root docs/maps --lidar-tf "$NEW_TF" --map-transform $T --start 73 --duration 741 --out $O/c_full_new_map_new_tf_hires_741
```

各実行の前に domain 153 にノードが無いことを確かめた（スクリプト自身も確かめる）。4 本とも終了コード 0。

走行中の 240 秒（bag の 73〜313 秒）。比べやすいよう、上の表の 11-12 の行を並べた：

| 実行 | 地図 | TF | fitness 中央値 / p95 / 最大 | 閾値超えの警告 | 収束しなかった回 | 姿勢の間隔の最大 | 記録との差 xy 中央値 / p95 / 最大 |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 11-12（上の表） | 旧 campus | #145（11-12） | 0.0976 / 0.195 / 0.71 | 0 | 0 | 1.52 s | 0.059 / 0.123 / 0.915 m |
| **a（11-13）** | 旧 campus | **11-13** | 0.0981 / 0.169 / 0.73 | 0 | 0 | 1.12 s | 0.063 / 0.126 / 0.905 m |
| 11-12（上の表） | campus-m8 | #145（11-12） | 0.0163 / 0.038 / 0.62 | 0 | 0 | 0.81 s | 0.079 / 0.156 / 1.081 m |
| d（11-13、TF は 11-12） | campus-m8 | #145（11-12） | 0.0163 / 0.038 / 0.66 | 0 | 0 | 0.91 s | 0.079 / 0.156 / 0.966 m |
| **b（11-13）** | campus-m8 | **11-13** | **0.0163 / 0.038 / 0.62** | **0** | **0** | **0.81 s** | 0.084 / 0.154 / 1.082 m |

走行の全区間（bag の 73〜814 秒、741 秒）：

| 実行 | 地図 | TF | fitness 中央値 / p95 / 最大 | 閾値超えの警告 | 収束しなかった回 | 姿勢の間隔の最大 | 記録との差 xy 中央値 / p95 / 最大 | 姿勢の数（記録） |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 11-12（上の表） | campus-m8 | #145（11-12） | 0.0154 / 0.062 / 0.66 | 0 | 0 | 0.81 s | 0.063 / 0.164 / 0.966 m | 7299（7298） |
| **c（11-13）** | campus-m8 | **11-13** | **0.0154 / 0.065 / 0.62** | **0** | **0** | **0.91 s** | 0.066 / 0.161 / 1.082 m | 7298（7298） |

読み取れること：

- **d は 11-12 の結果を再現した**。fitness の中央値・p95 と記録との差 xy の中央値・p95 は一致し、違うのは最大値（fitness 0.62 → 0.66、xy 1.081 → 0.966 m）と姿勢の間隔の最大（0.81 → 0.91 s）だけ。最大値が実行ごとに変わることは上の「ばらつき」の項のとおりで、統合ブランチのビルドで当てはまりが変わったとは言えない。
- **新しい地図では、TF の変更（z −6 mm、roll −0.29°、pitch −0.57°）で当てはまりはほぼ変わらない**。b と 11-12・d は中央値・p95 が同じ、全区間（c）も中央値が同じで p95 は 0.062 → 0.065（d と 11-12 の最大値の差と同程度で、ばらつきの範囲と見る）。閾値超え・収束しなかった回は 0 のまま。fitness が TF の小さな変化に鈍いことは上の最後の項と同じで、TF の正しさをこの比較で判定することはできない。
- **旧地図では p95 がやや良くなった**（0.195 → 0.169）が、中央値は同じ（0.098）で、旧の組み合わせ（0.0225）より約 4 倍悪いままである。「#144・#145 と #153 は一緒にマージする」という上の結論は変わらない。
- 各条件は 1 回ずつ（d を除く）。小さい差はばらつきの範囲かもしれない点は 11-12 と同じ。

### 確かめていないこと（屋外が要る）

- 新しい地図での実走行と、各地点に椅子を置いたときの `/pcl_pose` との一致（#156）。
- Nav2 の経路計画と走行（再生では Nav2 を動かしていない）。
- 回帰チェックの基準（`scripts/regression/baseline.json`）の更新。基準は旧地図で記録した姿勢との差なので、地図を変えると失敗するのが正常。2026-11-13 に既定の site を変えたときも、回帰チェックは site・基準とも旧 campus のまま残した（数値は変えていない。`baseline.json` の `site_note`）。上の 2026-11-13 の再比較は Hi-Res に補正した bag と T_new_old を使う別の測り方で、そのまま閾値にはできない。campus-m8 の基準は #156 の走行の bag で決め直す。
