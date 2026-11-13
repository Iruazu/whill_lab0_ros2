# `campus-m8`（M8-2 で作り直した campus 地図）

M8-2（#152〜#155）で、07-10 の地図用 bag から作り直した campus 地図。旧 `campus` は残し、別の site として置く。

**2026-11-13 から既定の site**（ユーザー判断、#190）。3 つの launch・`start_dispatch_demo.sh`・`check_map_sync.py`・Web の背景の既定は campus-m8 で、Nav2 は手塗りの候補（`map_variant:=candidate`、3 節）を読む。旧 `campus` は fallback として `site:=campus`（Nav2 は `map_variant:=v2`）で選べる（[runbook](../../runbook-live-demo.md)「旧地図（campus）に戻す」）。屋外の受入（#156）は既定にするための条件から外し、既定にした後の検証として行う。手塗りの確定（#154）もまだ。

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
| 3. 占有格子と手塗りの候補 | #154 | 候補まで。**手塗りの確定はユーザー**。既定の Nav2 の地図は候補（`candidate`） |
| 4. waypoint とローカライザの再生確認 | #155 | 再生まで。**屋外の受入（#156）は未**（既定にした後の検証） |
| 5. 既定の site にする | #190 | 済（2026-11-13）。旧 `campus` は fallback |

| ファイル | 中身 | Git |
| --- | --- | --- |
| `static.pcd` | DUFOMap 後の点群（7,614,632 点、91 MB） | 管理外。正本と複製の手順は 2 節の「static.pcd の置き場」 |
| `traj_lidar.txt` | GLIM の軌跡（TUM、21300 姿勢） | 管理 |
| `glim_config/` | 実行した GLIM の設定（実ファイル） | 管理 |
| `metadata.yaml` | 出典と設定 | 管理 |
| `align_from_campus/` | 旧 campus からの座標変換と waypoint の写像 | 管理 |
| `v2/` | v2 の機械レイヤ、`candidates/`（手塗りの候補）、`candidate_final.pgm` | 管理 |
| `occupancy_candidate.yaml` | `map_variant:=candidate` で読む候補の地図。既定の site で variant を省いたときもこれ | 管理 |
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

### 候補の合成の結果

| 項目 | 値 |
| --- | --- |
| 機械の障害物 | 401,771 セル（旧 v2 は 435,489） |
| 候補の free で消えた障害物 | 8,336 セル（旧の人手の塗りだけ 126、自動だけ 5,650、両方 2,560）。`v2/erased_by_free.png`（Git 管理外。合成のたびに作られる） |
| 最終の OCC / FREE / UNKNOWN | 0.94% / 4.94% / 94.11% |
| 配車4地点が FREE か | 4点とも FREE（新しい座標、下の waypoint 節） |
| 4地点が車体の幅でつながるか | つながる（FREE を 0.35 m 削った後の同じ連結成分。`scripts/map_route_check.py`） |
| 参考：手塗りなし（機械だけ） | 4点が別々の成分（1714 成分）、正門前は FREE の外。塗りが要ることの確認 |

```bash
scripts/map_route_check.py docs/maps/campus-m8/occupancy_candidate.yaml docs/maps/campus-m8/waypoints.yaml
```

つながらない（`all_connected: false`）と終了コード 1 で終わる。これは「全体の経路が計画できる」の代わりの必要条件で、Nav2 の planner は動かしていない（コストの膨張、smoother、controller は見ていない）。

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
