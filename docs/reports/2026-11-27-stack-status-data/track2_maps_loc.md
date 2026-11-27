# Track 2 監査：地図・地図作成パイプライン・runtime 自己位置（NDT）・EKF

- 対象：`/home/systemlab/whill_lab0_ros2`、main `64f7a49`（2026-11-19 15:04:52 +0900、#211 マージ）。作業ツリーは clean。
- 監査日：2026-11-27。読み取りのみ。リポジトリのファイルは変えていない。
- 証拠の分類：**[A]** 日付・条件・数値の付いた測定記録がある（生データの有無も書く）／**[B]** 今のコード・設定を読んだだけ（実機での挙動は示していない）／**[C]** 文書に書いてあるだけで、生データが見つからない／**[D]** 証拠が無い、または食い違う（両方を書く）。
- 中断前の実行についての注記：PC が再起動する前の段階（重い処理を禁じる指示が来る前）に、次の 3 つを実行した。結果はこの文書で使う。再起動の後は実行していない。
  - `campus-m8/static.pcd`・`campus/static.pcd`・`~/bags/.../dufomap-m8-hires-noise-local/static.pcd` の sha256 と、`campus-m8/v2/candidate_final.pgm`・`candidate_hand_final.pgm`・`campus/v2/final.pgm` の sha256
  - PCD の先頭 600 バイトのヘッダ（POINTS）
  - `scripts/check_map_sync.py` を 3 通り（読むだけのスクリプトであることを先に確かめた）

## 前提：校正・TF が変わった日時（以下の「前か後か」の基準）

| 項目 | #144/#145 の前（〜2026-11-13 16:24） | 今（main） | 出典 |
| --- | --- | --- | --- |
| Velodyne の仰角表 | `VLP16db.yaml`（Hi-Res の実機に対して誤り、#144） | `VLP16_hires_db.yaml` | `src/whill_sensors_bringup/launch/sensors_launch.py:151-153`、`src/whill_safety/launch/m6r_bringup_launch.py:292-298`。導入 7165750（2026-11-12）、main へのマージ #186 = 5392200（2026-11-13 16:24:02） |
| base_link→velodyne | (0.484136, 0.381548, 0.793704) m、roll −0.035342 rad、pitch 0.156983 rad、yaw −0.005527 rad（2026-06-20 の 78f6658 から） | (0.42, 0.36, 0.771) m、roll −0.05°、pitch +0.40°、yaw −0.317° | `git show m8-pre-calib-20261113:src/whill_sensors_bringup/launch/static_tf_launch.py:124-127`、今の `static_tf_launch.py:85-89`。11-13 の値は 9f6ad27 |
| base_link→imu_link | (0.38, −0.03, 0.47) m、roll −0.0713 rad、pitch −0.1415 rad、yaw 0 | 同じ（変わっていない） | `static_tf_launch.py:55-58`。2026-07-10 の 9cc4be2 から |
| 元 bag の点群の補正（#152） | — | `scripts/velodyne_reproject_bag.py`（VLP16db → Hi-Res、ring を使う） | `docs/maps/campus-m8/README.md` 1 節 |
| 戻し先のタグ | `m8-pre-calib-20261113` = f35fe86 | — | `git tag -l -n1` |

#186 の後の TF の値は 2 つある：2026-11-12 版 (0.42, 0.36, 0.777, RPY 0.24°, 0.97°, −0.317°)、2026-11-13 版（LiDAR をテープで固定した後）(0.42, 0.36, 0.771, −0.05°, +0.40°, −0.3167°)。今の main は 11-13 版。[B]

---

## (1) 地図の一覧

### 1.1 runtime の既定の site

既定は **campus-m8**。Nav2 の variant は **candidate**。[B]

| 場所 | file:line | 値 |
| --- | --- | --- |
| m6r bringup（NDT の static.pcd） | `src/whill_safety/launch/m6r_bringup_launch.py:116`（`_DEFAULT_SITE`）、`:261`（DeclareLaunchArgument の既定） | `'campus-m8'` |
| Nav2 | `src/whill_navigation/launch/nav_launch.py:117`（`_DEFAULT_SITE`）、`:125-126`（`_DEFAULT_VARIANT_BY_SITE = {'campus-m8': 'candidate'}`）、`:353-354` | campus-m8 / candidate |
| dispatch（waypoints） | `src/whill_dispatch/launch/dispatch_launch.py:63`、`:203` | `'campus-m8'` |
| 統合起動 | `scripts/start_dispatch_demo.sh:8`（`SITE=campus-m8`）、`:44-49`（`demo_variant_for`：campus-m8→candidate、それ以外→v2） | — |
| 同期の確認 | `scripts/check_map_sync.py:41`（`DEFAULT_SITE`）、`:43-46`（`DEMO_VARIANT_BY_SITE`：campus-m8→candidate、campus→v2） | — |
| Web の背景を作る | `scripts/m7_make_web_map.py:76`（既定の `--yaml` = `docs/maps/campus-m8/occupancy_candidate.yaml`） | — |

fallback：`site:=campus`（Nav2 は `map_variant:=v2` → `docs/maps/campus/occupancy_v2.yaml` → `v2/final.pgm`）。`docs/maps/campus/README.md:3`、`docs/runbook-live-demo.md:243`。[B]

### 1.2 ディレクトリごと

サイズと mtime は `ls -la --time-style=long-iso`。mtime が 2026-11-27 15:35 のファイルは、その時刻の `git pull --ff-only`（reflog `HEAD@{2026-11-27 15:35:43}`）で書かれたもので、作った日ではない。`.pcd` は Git 管理外（`.gitignore:133` `docs/maps/**/*.pcd`）。

| site | 主なファイル（ディスク上） | static.pcd | 元 bag・作った日 | パイプライン | #144/#145/#152 の前か後か | 根拠 |
| --- | --- | --- | --- | --- | --- | --- |
| **campus-m8**（既定） | README.md 181,001 B、metadata.yaml、georef.yaml、reference-gsi.md、traj_lidar.txt 1,887,386 B、waypoints.yaml、glim_config/（12 ファイル、実ファイル）、align_from_campus/（5）、ndt_replay/（6 + 2026-11-13/ に 4）、occupancy_candidate.yaml、occupancy_candidate_hand.yaml、v2/（candidate_final.pgm 41,798,817 B、candidate_hand_final.pgm 41,798,817 B、candidate_final.yaml、機械レイヤ PNG 7 枚、v2_layers.yaml、candidates/ 10、experiments/154w・202・202b・202c・202d・205）、start_side_view.png。`occupancy.yaml` は**無い** | **ある**。91,375,760 B、mtime 2026-11-13 17:00、ヘッダ POINTS 7,614,632。sha256 `d244865866e3eacdea6e26851dd3ed61ba626029fdc6dd4f2dabfb6d720d03b5`（metadata.yaml の `static_pcd_sha256` と一致。正本 `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/dufomap-m8-hires-noise-local/static.pcd` も同じ値） | 元の記録は旧 campus と同じ 2026-07-10 15:07:43 の bag。それを #152 で補正した `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag-hires/`。GLIM の採用した実行 A は 2026-11-12 17:12:44〜17:20:32、DUFOMap は 17:23〜17:37 | 補正した bag → `scripts/imu_investigation/glim_control.sh`（GLIM GPU、fix_imu_bias=false、雑音 0.008/0.0013、T_lidar_imu = audit 値）→ `scripts/m5r_run_dufomap.sh`（resolution 0.1、inflate_hits_dist 0.2、inflate_unknown 2）→ `pcd_to_occupancy_v2.py` → 旧手塗りを `map_transfer_masks.py` で写す → `map_widen_hand_roadway.py`（154w）→ `gen_auto_free_mask.py`・`compose_occupancy.py`。GLIM 1.2.0（88b3833）/ glim_ros 1.2.1（a62811d）、RTX 3080 Ti Laptop、driver 595.99.02 | 点群：**#144・#152 の後**（Hi-Res に補正した点群）。TF：README によると GLIM・DUFOMap は base_link→velodyne を使わないので、地図の中身は #145 の TF に依存しない。地図を作ったのは #186 の main マージ（11-13 16:24）より前の 11-12 で、HEAD は 14208d0 に未コミットの変更を足した状態（実行 A の途中で 2fa22ef が入った） | `campus-m8/metadata.yaml`、`campus-m8/README.md:17-21`、`:50-194`。[A]（品質の数値は README 2 節。生の GLIM 出力は `~/bags/.../glim-m8-hires-*` にある） |
| **campus**（fallback） | README.md、metadata.yaml、occupancy.{pgm,yaml}、occupancy_cleaned.{pgm,yaml}、occupancy_cleaned.xcf、occupancy_cleaned１.xcf（全角の「１」）、occupancy_trav.yaml、occupancy_v2.yaml、occupancy_v2.pgm → v2/final.pgm（シンボリックリンク）、traj_lidar.txt、trav_*.png 4 枚、cleaning_diff.png、waypoints.yaml、.v2_cache.npz 668,782,098 B、v2/（final.pgm ほか。build/・install/・log/ という名前の Git 管理外のディレクトリもあり、build/ には final_before_paint_20260720.pgm が入っている） | **ある**。115,798,736 B、mtime 2026-07-10 17:45、ヘッダ POINTS 9,649,880。sha256 `9f5ac62b48464034aefb35b60854761d6b03e7d1fafbe53bec8f895840b0e8e4`（照合する値はどの文書にも無い） | 2026-07-10 15:07:43 の bag（`metadata.yaml` の `source_bag` は `docs/m5r-bench-data/.../bag/` だが、今はそこに無い。bag は `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/bag/`） | GLIM（audit T_lidar_imu、`m5r3_run_glim.sh`）→ DUFOMap → `m5r_pcd_to_occupancy.py`（commit 7a9924a）→ v2（`pcd_to_occupancy_v2.py` と手塗りのマスク、2026-07-18〜07-24。final.pgm の最後の変更は e741e9a、2026-07-24） | **前**。点群は VLP16db で計算されたもの（#144 の誤り）。#152 の補正もしていない。GLIM の設定の記録は、書き換わる上流の設定へのシンボリックリンク（`docs/m5r-bench-data/2026-07-10-campus-outer-final/glim-out-audit-tli/config/*.json -> src/third_party/glim/config/...`）で、実行したときの値は残っていない（#147 の報告「別に見つかった問題 2」） | `campus/metadata.yaml`、`campus/README.md`、`docs/reports/2026-11-02-glim-imu-root-cause.md`。[A]（品質の数値は README）／設定の記録は [D] |
| campus-demo | metadata.yaml、occupancy.{pgm,yaml}（pgm 39,523,613 B、2026-07-14） | ある。123,261,682 B、mtime 2026-07-08 20:32、POINTS 10,271,792 | 2026-07-08 18:52:24 の bag の、内部の欠落より前の 44 分（pregap） | GLIM + DUFOMap、`m5r_pcd_to_occupancy.py`。`commit: "TBD-registered-post-hoc"` | **前**（VLP16db、旧 TF の時期） | metadata.yaml。git の履歴は b361493（2026-07-10）の 1 件だけ。[C] |
| campus-half-v3 | metadata.yaml、occupancy.{pgm,yaml}（pgm 11,875,446 B、2026-07-10） | ある。39,962,684 B、mtime 2026-07-08 09:32、POINTS 3,330,209 | 2026-07-07 19:30:40 | GLIM + DUFOMap。`commit: "TBD-after-commit"` | **前** | metadata.yaml（「drift 14.5%、Z warp 45 m」）。[C] |
| campus-outdoor-corrected | metadata.yaml、occupancy.{pgm,yaml}（pgm 28,181,315 B） | ある。35,162,444 B、mtime 2026-07-08 13:23、POINTS 2,930,189 | 2026-07-08 12:22:55 | GLIM + DUFOMap。`commit: "TBD-after-commit"` | **前** | metadata.yaml（「drift 0.19 m / 457.5 m」）。[C] |
| lab-legacy-m5b | README.md、lab.pcd 8,207,547 B（2026-05-30、POINTS 256,478）、global_2026-06-04_10min.pcd 79,128,528 B（POINTS 4,945,521）、lab.pgm、lab.yaml | （static.pcd という名前は無い） | 旧 M5-b、FAST-LIO | FAST-LIO（runtime の設定は削除済み） | **前**。使っていない | README.md。[C] |
| _template | metadata.yaml、occupancy.yaml、README.md | 無し（テンプレート） | — | — | — | [B] |

### 1.3 campus-m8 の占有格子の版

| variant | YAML → 画像 | sha256（README 154w「採用」の記録 = 2026-11-27 に読んだ値） | 中身 | 検証 |
| --- | --- | --- | --- | --- |
| **candidate（既定）** | `occupancy_candidate.yaml` → `v2/candidate_final.pgm` | `fca2abb47a4af7ae987784f976a14b7d51ccf5adfeae1e9190a3b7b959fcaa9e`（一致） | 旧 campus v2 の手塗りを T_new_old で写した候補を、202d で補正した地理院の縁（段・構造物で寄せた区間）まで広げた版（154w、R6 は gap 1）。2026-11-19 から既定（537ad73、PR #211）。FREE 2,216,748 セル（5,541.9 m²、手塗りのままより +375.4 m²、+7.3%）、OCC 392,646 セル | オフラインのみ：`map_route_check.py`（4 地点が FREE で、車体の幅で 1 成分につながる。成分 13）、`nav2_plan_check.py`（2026-11-19、12 組すべて計画できる。OCC までの最小 0.62 m。seimon から出る 3 組は非 FREE の縁まで 0.25 m）、`check_map_sync.py` ok。屋外は 2026-11-27 の走行（3 節の行 13）が唯一 [A]。手塗りの確定（#154）はまだで OPEN |
| candidate_hand | `occupancy_candidate_hand.yaml` → `v2/candidate_hand_final.pgm` | `5106c0bca46650a40a491ca611cabdb0eed470ec96cfca5db5778813fc1c41d9`（一致） | 広げる前（2026-11-13〜11-19 の既定。1a4f327 の candidate_final.pgm と同じ）。FREE 2,066,589 セル、OCC 393,435 セル（154w の表）。この版自体の値：機械の障害物 401,771、free で消えた障害物 8,336、OCC / FREE / UNKNOWN = 0.94% / 4.94% / 94.11% | `map_route_check.py`（成分 7）、`nav2_plan_check.py`（同じ 12 組） |
| （無い）occupancy.yaml | — | — | 手塗りが確定するまで置かない（README 3 節） | — |
| v2/experiments/202・202b・202c・202d・205 | — | — | 縁石からの自動抽出の試作（#202、OPEN）と GSI との照合（#205）。既定の地図には使わない | 試作の数値だけ（README 3 節） |

数値の出典：`docs/maps/campus-m8/README.md` の 3 節（候補の合成の結果、154w の結果、採用、Nav2 の planner での確認）と `v2/experiments/154w/summary.json`（summary.json そのものは再起動後に開いていない）。[A]（オフライン）

関連 Issue：#153 CLOSED 2026-11-13、#154 OPEN、#155 OPEN、#156 OPEN、#170 CLOSED、#171 CLOSED、#202 OPEN（コメント 0）。PR #210（cf46559）・#211（64f7a49）はマージ済み。

### 1.4 Web の背景と Nav2 の地図

- `src/whill_dispatch/web/map_meta.json`：`source_pgm: docs/maps/campus-m8/v2/candidate_final.pgm`、830×786、resolution 0.4、origin [−128.25, −90.35, 0.0]、downscale_factor 8。[B]
- `scripts/check_map_sync.py`（読むだけ）が確かめること（`:3-17`）：meta の `source_pgm` と YAML の `image` が同じファイルか、origin が同じか、resolution = YAML の resolution × factor か、map.png の寸法 = 元画像 ÷ factor か、map.png の画素が同じ縮小方法の結果と合うか（差 > 2 の画素を数える、`PIXEL_TOLERANCE = 2`）、negate・free_thresh が app.js の FREE_THRESHOLD と矛盾しないか。終了コードは一致 0、食い違い 1、入力が読めない 2。
- 実行の結果（再起動の前に実行）：
  - 既定（campus-m8 / candidate、`src/whill_dispatch/web`）：`地図の同期: ok`、rc=0。注意として「元画像の高さ 6295 が factor 8 で割り切れません。Web の座標は原点と反対の端で最大 0.35 m ずれます」。
  - `install/whill_dispatch/share/whill_dispatch/web`：ok、rc=0。
  - `--site campus`（fallback）：rc=1。出典が違う、map.png の 124175 画素が違う（最大差 109）。Web の背景は 1 組だけなので、fallback に戻すときは作り直しが要る（`docs/maps/README.md` に書かれている運用と合う）。[A]（2026-11-27 の実行）
- NDT は `static.pcd` を読み、占有格子は読まない（README 154w「採用」）。[B]

---

## (2) 今の自己位置推定・EKF の設定

### 2.1 NDT（lidar_localization_ros2）

パッケージは `rsasaki0109/lidar_localization_ros2`。`whill_lab.repos` で `1dbcf3a655328d79a52252d11c8836502011ba5f` に固定。checkout も 1dbcf3a で、変更は無い。`ndt_omp_ros2` は `version: humble`（ブランチ。checkout は ef8a349）。[B]

設定ファイル：`src/whill_safety/config/m6r_lidar_localization.yaml`（パラメータの値は 2026-07-20 の 2d3e983 / 01f2fd8 から変わっていない。11-13 の ac397ca はコメントだけ。`git diff 01f2fd8 HEAD` で、コメントでない行の差は 0）。[B]

| パラメータ | 値 | 行 |
| --- | --- | --- |
| registration_method | NDT_OMP | :41 |
| score_threshold | 6.0 | :42 |
| ndt_resolution | 1.0 | :43 |
| ndt_step_size | 0.1 | :44 |
| ndt_num_threads | 8 | :45 |
| ndt_max_iterations | 25 | :46 |
| transform_epsilon | 0.01 | :47 |
| voxel_leaf_size | 0.5 | :48 |
| enable_scan_voxel_filter | true | :49 |
| scan_max_range / scan_min_range | 80.0 / 1.0 | :50 / :51 |
| scan_period | 0.1 | :52 |
| cloud_queue_depth | 1 | :53 |
| use_pcd_map | true | :54 |
| map_path | ""（launch が埋める） | :55 |
| set_initial_pose | false（RViz の 2D Pose Estimate） | :56 |
| use_imu | false | :57 |
| enable_map_odom_tf | true | :58 |
| use_odom | true（#108、2026-07-20） | :71 |
| predict_pose_from_previous_delta | false | :86 |
| use_twist_prediction | false | :93 |
| enable_timer_publishing | 設定していない（理由は :95-105） | — |

launch が上書き・追加する値（`src/whill_safety/launch/m6r_bringup_launch.py`）：[B]

| 項目 | 値 | 行 |
| --- | --- | --- |
| map_path | `<maps_root>/<site>/static.pcd` を一時 yaml の `/**` に書く | :140-179（:173） |
| odom の remap | `SetRemap('/odom' → '/odometry/filtered')` | :203 |
| cloud_topic | `/velodyne_points` | :209 |
| imu_topic | `/imu/data_rep145`（use_imu_preintegration が false なので使われない） | :216 |
| use_imu_preintegration | `'false'`（上流の既定は true） | :227 |
| publish_lidar_tf | `'false'`（上流の恒等 TF を止める） | :232 |
| use_sim_time | launch 引数 | :233、:268 |
| velodyne_calibration | 既定 `VLP16_hires_db.yaml` | :292-298 |

上流の launch（`src/third_party/lidar_localization_ros2/launch/lidar_localization.launch.py`）で既定のまま残る値：global/odom/base frame = map/odom/base_link、`use_continuous_time_deskew` false、`imu_preintegration_use_base_frame_transform` false。[B]

failsafe の Layer B が自己位置を見る閾値（`src/whill_safety/whill_safety/failsafe_node.py`）：`FITNESS_MAX = 1.0`（:64）、`FITNESS_WINDOW_S = 2.0`（:65）、`PCL_POSE_TIMEOUT_S = 1.0`（:66）。NDT 自身の reject の閾値は 6.0。[B]

ADR-0006（`docs/decisions/0006-localizer-choice.md`）：Status は **proposed**（:4）。M6R-5 で accepted にする 4 条件（:106-114）はすべて未チェック。2026-07-19 の追補（odometry 拘束、:116-）の Status は proposal。決定 6 の NDT の初期値（:47-55）は今の yaml と同じ値。[B]／食い違いは 5 節の D-1。

### 2.2 EKF（robot_localization）

設定：`src/whill_localization/config/ekf_odom.yaml`。起動：`src/whill_localization/launch/ekf_odom_launch.py:49-58`（`ekf_node`、パラメータはこの yaml と use_sim_time だけ）。`odom_bringup_launch.py` から include され、それを `m6r_bringup_launch.py` が include する。[B]

| 項目 | 値 | 行 |
| --- | --- | --- |
| frequency | 30.0 | :55 |
| sensor_timeout | 0.5 | :62 |
| two_d_mode | true | :64 |
| publish_tf / publish_acceleration | true / false | :69 / :70 |
| map / odom / base_link / world frame | map / odom / base_link / **odom** | :74-77 |
| odom0 | `/whill/odom`、fuse するのは vx・vy・vyaw だけ | :84、:94-98 |
| odom0_differential / relative / queue | false / false / 10 | :101 / :102 / :106 |
| imu0 | `/imu/data_rep145`、fuse するのは vroll・vpitch・vyaw・ax・ay（orientation は使わない） | :109、:115-119 |
| imu0_remove_gravitational_acceleration | true | :129 |
| imu0_queue_size | 20 | :133 |
| process_noise / initial_estimate の共分散 | yaml に書いていない = robot_localization の既定（理由はコメント :45-50） | — |

入力の共分散：[B]
- `/whill/odom`：`src/third_party/ros2_whill/whill_driver/src/whill_node.cpp:234-239` で pose の x・y・yaw、twist の vx・vy・vyaw がすべて 0.01（固定値）。fork は `humble-with-odom-2026-06-18`（d2d1218）に固定。
- `/imu/data_rep145`：ドライバが `linear_acceleration_stddev`・`angular_velocity_stddev` の 2 乗を対角に入れる（`rt_usb_9axisimu_driver.cpp:390-404`）。orientation_covariance[0] = −1。このリポジトリの launch がこの 2 つのパラメータを渡しているか、どの値で動いているかは確かめていない（6 節）。`imu_sign_corrector.py` は共分散を変えない（:22）。

EKF の測定：
- M4-R の受入（Issue #37、PR #41、2026-06-20）：PR #41 の本文では AC1（30 Hz）・AC2（TF の publisher）・AC3（手押し 10 m で終端誤差 ≤ 0.5 m）・AC4（静止 30 秒で yaw drift ≤ 0.1 rad）にチェックが付いているが、**数値・bag・解析の出力はどこにも無い**。`docs/m4r-bench-data/README.md` の Run index は空（「— | — | …」）で、ディレクトリには README.md しか無い。Issue #37 のコメントは 0 件。PR #42 には「M4R-3 verification で観測された AC3 軌跡の 35° 横方向流れ … は別 Issue 起案推奨」とだけある。→ **[C]**
- その受入は、今と違う構成で行った：当時の `imu0` は `/imu/data_raw`（PR #41 の本文。`/imu/data_rep145` に変えたのは #56、2026-06-21）。IMU を付け直したのは 2026-07-08、base_link→imu_link が今の値になったのは 2026-07-10（9cc4be2）。どちらも受入の後。**今の EKF の構成で終端誤差・yaw drift を測った記録は無い。** [D]
- 車輪の単位（`docs/m4r-whill-units.md`、2026-06-14）：motor_angle = rad、motor_speed = km/h（上流のソース + 約 2 m の粗い走行、巻尺なし）。タイヤ径 0.265 m・tread 0.496 m は公称値で、「(未実測)」。[C]（`/tmp/cr2_full.log` は残っていない）
- 入力のレート：2026-07-14 の manifest で `/whill/odom` 1569 件 / 627.7 s = 2.5 Hz、`/imu/data_rep145` 62760 件 = 100.0 Hz。bag は `~/bags/m6r-bench-data/2026-07-14-verify-campus/bag` にある。[A]
- 仮定のままのもの：yaml のコメント (5)「upstream のデフォルトで満たせる見込み」、two_d_mode の理由「学内は実質平面」。[C]
- IMU の TF と EKF：EKF は base_link→imu_link の TF で加速度を回してから重力を引く（yaml のコメント :38-43、:121-129）。campus-m8 README 2 節には、TF の鎖から計算した T_lidar_imu（pitch −9.08°）が audit 値（−7.61°）よりこの bag に合わない、「どちらかの TF（imu_link 側か velodyne 側）、または両方が実物とずれている可能性がある」とある。IMU の TF の誤差が EKF に与える影響を測った記録は無い。[D]
- 文書の古い記述：`ekf_odom_launch.py:5` と `odom_bringup_launch.py:20` の docstring は「`/imu/data_raw` を fuse」と書いているが、yaml（:109）は `/imu/data_rep145`。[D]（コードは yaml が正しい）

---

## (3) 自己位置の測定結果と、今の構成に当てはまるか

今の構成 = 地図 campus-m8（static.pcd sha256 d2448658…）・Hi-Res の仰角表・TF 2026-11-13 版・NDT の設定は 2026-07-20 以降と同じ（use_odom true）・Nav2 の地図は candidate（154w）。

| # | 日付 | 種類 | 地図 | 点群の校正 / TF | NDT の設定 | 数値（原文のまま） | 分類・生データ | 今の構成に当てはまるか |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 2026-07-08 | bag 再生の smoke test（地図を作った bag そのものを再生） | campus-outdoor-corrected | VLP16db / 旧 TF | 今の yaml の元になった値。use_odom・predict 系の設定は無い（`run_config.yaml`） | /alignment_status の 1 回の記録：message ok、fitness_score 0.011374、alignment_time 0.374848 s。out.log で「fitness score is over 6.0」は 0 回 | [A]。`docs/m6r-bench-data/2026-07-08-smoke-campus-outdoor-corrected/`。記録は 1 回分だけで、時系列ではない | **当てはまらない**。地図・校正・TF・odom の設定がすべて違う。地図を作った bag で再生しているので、自分自身との一致を見ているだけ |
| 2 | 2026-07-08 | 同上 | campus-half-v3 | 同上 | 同上 | 1 回の記録：message fitness_score_over_threshold_rejected、fitness 8.752737。out.log で 6.0 超えの警告 2229 回 | [A] | 当てはまらない |
| 3 | 2026-07-08 | 同上 | campus-demo | 同上 | 同上 | alignment_status.txt は 0 行。out.log で 6.0 超えの警告 0 回、pcl_pose.txt 48 行 | [D]（fitness の値が無い。この結果を書いた文書も無い） | 当てはまらない |
| 4 | 2026-07-12 | 屋外の実機、M6R-2 の受入（launch は使わず、localizer を直接起動） | campus（9,649,880 点） | VLP16db / 旧 TF | 当時の yaml + `enable_debug: true` + `use_imu_preintegration: false`。use_odom は未導入 | localizer のログの 521 フレーム：fitness 最小 0.0629518、最大 0.0827226、p50 0.0709437、p95 0.0759160、p99 0.0789292、has_converged は 521 件すべて（manifest）。ログから数え直した値：521 件、最小 0.0629518、最大 0.0827226、p50 0.0709459、p95 0.0759301（百分位の取り方の違い）。「静止 1 分 fitness 0.0186〜0.0193」「12 分走行で reject 0、最悪 ~0.3」は「操作者目視」 | ログの部分は [A]（`terminal_b_localizer.log`）。静止・12 分の値は [C]（bag は `/tf_static` 1 件だけ、24.9 KiB） | **当てはまらない** |
| 5 | 2026-07-14 | 屋外の実機、§10.4 の確認（m6r_bringup_launch） | campus | VLP16db / 旧 TF | use_odom は未導入 | 手動で約 10 分（627.7 s）走って reject_count 0。fitness は「良好」とだけあって数値は無い。トピックのレートは 1.3 節・2.2 節のとおり | reject 0 は [C]（manifest だけ。bag は `~/bags/m6r-bench-data/2026-07-14-verify-campus/bag` にあるが、解析は残っていない）。レートは [A] | 当てはまらない |
| 6 | 2026-07-19 | 屋外の実機、配車の自律走行（#108） | campus（v2） | VLP16db / 旧 TF | use_odom 0、use_twist_prediction 0、predict_pose_from_previous_delta 1（Issue に書かれた起動ログの値） | 28 回 reject、map→odom が最大約 50 秒止まる、job 1 は 165 秒で abort、job 2 は 53 秒で abort | [C]（Issue #108 の本文。`~/.ros/log/...` のログは今回見ていない） | 当てはまらない（この後 use_odom を入れた） |
| 7 | 2026-07-20 | 屋外の実機（#108 のコメント） | campus（v2） | VLP16db / 旧 TF | use_odom true（PR #109）。今の NDT の設定と同じ | bag 15_59_27：fitness n=6065、最小 0.012、最大 10.03、6.0 超え 5 回。bag 18_09_28（73 m を完走）：最大 0.855、6.0 超え 0 回 | [A]（Issue のコメントに日付と数値。bag は `~/bags/m7-bench-data/2026-07-20-live-campus/` にある。解析スクリプトは残っておらず、数値は今回計算し直していない） | NDT の設定は今と同じ。**地図・校正・TF が違う** |
| 8 | 2026-07-20 | 同上、controller の A/B（#101） | campus（v2） | 同上 | 同上 | angular.z RMS 0.462 → 0.125、cross-track 中央値 5.0 cm → 1.6 cm、最大 14.1 cm → 9.7 cm、fitness > 6.0 は 5 回 → 0 回 | [A]（同じ条件） | 当てはまらない（Nav2 の設定は別の Track） |
| 9 | 2026-11-02 | 回帰チェックの基準（07-24 の bag を再生） | campus | 07-24 の bag の点群（VLP16db）/ 旧 TF（main 81f3ec7） | 今の設定 | `observed.localizer`：「xy p50 0.000 m, p95 0.053-0.080 m, max 0.683 m, yaw p95 0.13-0.18 度, 姿勢 2350-2354 個（3回）」。閾値 xy_err_p50 ≤ 0.02 m など | [C]（`scripts/regression/baseline.json` の文字列だけで、各実行の出力は見つけていない） | **当てはまらない**。`site_note` に「demo_regression.sh --level replay は、#156 の走行の bag で campus-m8 の基準を取るまで main で失敗する」とある |
| 10 | 2026-11-12 | 07-24 の bag の再生 6 本（`ndt_replay_compare.py`） | 旧 campus / campus-m8 | 元の点群または Hi-Res に補正した点群 / 旧 TF または 11-12 版の TF | 今の設定（bag に記録された `/odometry/filtered` を使う） | 240 秒（bag の 73〜313 秒）：旧地図・旧 TF・元の点群 0.0225 / 0.1307 / 8.1647（中央値 / p95 / 最大、警告 4）。旧地図・11-12 版 TF・Hi-Res 0.0976 / 0.1953 / 0.7115（0）。campus-m8・旧 TF・元の点群 0.3427 / 0.4956 / 3.1979（0）。campus-m8・11-12 版 TF・Hi-Res 0.0163 / 0.038 / 0.6188（0、xy の差 p50 0.079 m）。741 秒：旧の組み合わせ 0.0306 / 0.1985 / 10.4661（6）。campus-m8・11-12 版 TF・Hi-Res 0.0154 / 0.0616 / 0.6563（0） | [A]（`docs/maps/campus-m8/ndt_replay/*.json`。git の欄は `2fa22ef-dirty`・`70042db-dirty`・`15a350b-dirty`） | 地図は今と同じ。**TF は 11-12 版で今（11-13 版）と違う**。bag は補正した 07-24 の記録で、新しい記録ではない。各条件 1 回だけ |
| 11 | 2026-11-13 | 同じ再生を 4 本（11-13 版 TF） | 旧 campus / campus-m8 | Hi-Res に補正 / 11-13 版 TF（d だけ 11-12 版） | 今の設定。統合ブランチ b74bdf2（push していない） | a（旧地図）0.0981 / 0.1694 / 0.7276。b（campus-m8）0.0163 / 0.0378 / 0.6218、警告 0、xy の差 p50 0.084 m。d（campus-m8・11-12 版 TF）0.0163 / 0.038 / 0.6563。c（campus-m8、741 秒）0.0154 / 0.065 / 0.6218、警告 0、xy の差 p50 / p95 / 最大 0.066 / 0.161 / 1.082 m | [A]（`ndt_replay/2026-11-13/*.json`） | 地図・TF・校正は今と同じ（b・c）。ただし**再生**で、bag は 07-24（記録したときは VLP16db・旧 TF・旧地図）を補正したもの。odom は記録された EKF の出力。README は「fitness は TF を 10 cm ずらしても変わらなかった記録がある」「位置が正しいことの証明ではない」と書いている |
| 12 | 2026-11-16 | 屋外の静止確認（#149）。LiDAR の TF と物の位置の確認で、自己位置の測定ではない | — | Hi-Res / 11-13 版 TF（`results.json` の `lidar_tf` [0.42, 0.36, 0.771, −0.05, 0.4, −0.317]） | — | 床の平面（LiDAR）slope_x 0.38°、slope_y −0.24°、z0 −0.024 m。コーン 7 本は全部検出。未検出・失敗の項目は空 | [A]（`docs/calibration/2026-11-16-outdoor-static/`、bag は `~/bags/2026-11-16-static/`） | TF の確認としては今の構成。自己位置の数値は含まない |
| **13** | **2026-11-27** | **屋外の実機の自律走行**（RViz の Nav2 Goal、配車 Web の job は 0 件）。`start_dispatch_demo.sh --speed slow`（0.30 m/s）、failsafe・twist_mux 有効 | **campus-m8 / candidate**（README に candidate_final.pgm `fca2abb4…`、static.pcd `d2448658…`） | 今と同じ（main `64f7a49`、`scripts_used/git_main_head.txt`） | 今と同じ | `analysis/summary.json`（下の表 13a） | **[A]**。bag は mcap（`record_avoidance_bag.sh --profile drive`）。**リポジトリには入っていない**（`~/bags/2026-11-27-m8-drive/`）。判定に使った `check_localization.py` は**マージしていないブランチ**の 692686a（`scripts_used/check_localization_commit.txt`） | **今の構成での唯一の測定**。ただし絶対位置の誤差（真値との比較）は測っていない（README：「同じ目印に戻って比べる走行はしていない」）。配車 4 地点に椅子を置いて照合する #155/#156 の手順もしていない |

### 13a. 2026-11-27 の走行（`~/bags/2026-11-27-m8-drive/analysis/summary.json` の値そのまま）

| bag | 長さ [s] | 判定 | fitness 件数 | fitness p50 / p95 / 最大 | 1.0 超え | 収束しない | 連続 reject の最大 | failure_category | 姿勢の間隔の最大 [s] | ndt_path_m / odom_path_m | goals | cmd_vel_safety のゼロ指令 | error_logs |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| start_static_155338 | 59.9 | PASS | 591 | 0.019826 / 0.0316165 / 0.038853 | 0 | 0 | 0 | healthy 591 | 0.13075757026672363 | 2.2 / 0.0 | — | 0 | なし |
| drive01_155523 | 54.1 | PASS | 534 | 0.02242 / 0.04376395 / 0.06027 | 0 | 0 | 0 | healthy 534 | 0.12904882431030273 | 10.5 / 8.7 | 51555aa5 SUCCEEDED | 23 | 「Failed to get result for compute_path_to_pose in node halt!」1 |
| drive02_155728 | 53.4 | PASS | 527 | 0.023781 / 0.05982529999999999 / 0.099119 | 0 | 0 | 0 | healthy 527 | 0.1334540843963623 | 14.4 / 13.1 | SUCCEEDED ×2 | 0 | なし |
| drive03_155854 | 239.5 | PASS | 2361 | 0.063106 / 0.091789 / 0.159595 | 0 | 0 | 0 | healthy 2361 | 0.13727784156799316 | 60.5 / 43.2 | a5b9f089 ABORTED、c25fc20d EXECUTING | 1760 | 「controller_server: Failed to make progress」5 |
| drive04_161045 | 214.7 | PASS | 2117 | 0.043196 / 0.07637640000000001 / 0.133884 | 0 | 0 | 0 | healthy 2117 | 0.14042043685913086 | 51.6 / 47.1 | be23b8ef SUCCEEDED、1fef57b6 EXECUTING | 1068 | 「RegulatedPurePursuitController detected collision ahead!」4、「Failed to make progress」2 |

- 止まっている時と走っている時の fitness（`fitness_by_motion`）：drive03 は stationary p50 0.085523（925 件）、moving p50 0.0339975（1432 件）。drive04 は stationary 0.0442975、moving 0.040727。
- `cmd_vel_teleop_n` は全 bag で 0。
- README（同じディレクトリ、2026-11-27 16:39）の解析：止まった 2 か所（drive03 (64.5, 5.1)、drive04 (−84.0, −0.4)）は Layer D の扇形の遮断で、反応した点は地図の OCC の縁から 0.05 m 以内。README は「自己位置は合っている」と書いているが、根拠は点と OCC が重なることと fitness で、真値との比較ではない。drive03 では 0.1 s の間に 0.28〜0.40 m・3.7〜4.9° 動いた姿勢が 6 回ある。車体の長方形から OCC までの最小は drive03 で 0.20 m、drive04 で 0.30 m。
- 記録していない区間：16:06:21〜16:10:02 の goal（約 66 m、成功）は bag が無い（README）。
- 食い違い：README の表の「NDT の移動（1 s 間引き）」8.7 / 13.0 / 45.1 / 46.3 m は、summary.json の `ndt_path_m`（10.5 / 14.4 / 60.5 / 51.6）とも `odom_path_m`（8.7 / 13.1 / 43.2 / 47.1）とも一致しない（drive01 だけ odom と同じ値）。間引いた計算は summary.json に無い。[D]

---

## (4) オフラインのパイプライン：今の手順と再現できるか

### 4.1 campus-m8 を作った手順（今の手順）[A]（実行の記録は campus-m8 README の 1〜3 節）

1. `scripts/velodyne_reproject_bag.py`：VLP16db で記録した bag の点群を Hi-Res に補正する（新しい bag は今の既定が Hi-Res なので要らない）。検証は `velodyne_reproject_verify.sh`（ドライバの出力との差が最大 0.015 mm）と `velodyne_compare_clouds.py`。
2. `scripts/imu_investigation/glim_control.sh`：GLIM を GPU で動かす。設定は `cp -rL` で実ファイルにして残す。campus-m8 の設定は `docs/maps/campus-m8/glim_config/`（**`configs/` に GLIM の設定は無い**。`configs/` にあるのは CycloneDDS の XML と `legacy/` だけ）。
3. `scripts/m5r_run_dufomap.sh` → `m5r_run_dufomap_core.py`（DUFOMap、pip）。
4. `scripts/pcd_to_occupancy_v2.py`。格子の型は `--input-yaml docs/maps/campus/occupancy_cleaned.yaml`（旧 campus の格子を使う）。
5. `map_align.py` → `map_transfer_masks.py`（旧手塗りを写す）→ `map_widen_hand_roadway.py`（154w。入力に `~/bags/m8-experiments/202d/` が要る）→ `gen_auto_free_mask.py` → `compose_occupancy.py`。
6. `m7_make_web_map.py` → `check_map_sync.py`、`map_route_check.py`、`nav2_plan_check.py`。

### 4.2 古い手順・使わなくなった物 [B]／[C]

- `scripts/pcd_to_occupancy_grid.py`：FAST-LIO の M5-b の PCD 向け（冒頭 :2-14）。`m5r_pcd_to_occupancy.py` はその兄弟で、GLIM + DUFOMap 向け（:9-13）。`pcd_to_occupancy_v2.py`（2026-07-18）が「older pipeline (m5r_pcd_to_occupancy.py + clean_isolated_occupancy.py + GIMP manual salt cleanup)」を置き換えると書いている（:4-10）。ただし `docs/scripts.md` は 3 つとも区別なく並べている。
- `scripts/m5r3_run_glim.sh`：`docs/scripts.md` では今も「GLIMのオフライン実行」。設定は `cp -r`（-L なし）。旧 campus の実行の config はシンボリックリンクとして残っている。campus-m8 では使っていない。
- `docs/m5r-pipeline.md`：冒頭で「過去の計画・調査・検証記録」と明記している。
- #147 の結論（`docs/reports/2026-11-02-glim-imu-root-cause.md`）：主な原因は VLP16db（300 秒の区間、各条件 3 回）。並進の better ratio は元の点群 0.03 → Hi-Res 0.39〜0.40、速度の誤差は IMU / 等速 = 0.094〜0.097 / 0.026 → 0.019 / 0.023〜0.024 m/s。全行程（campus-m8 の実行 A）では並進 0.31、速度 0.51 で、並進は閾値 0.4 に届いていない（README 2 節）。[A]（`docs/reports/2026-11-02-glim-imu-data/control_summary.txt`、生の出力は `~/bags/imu_investigation_20261030/glim_control_20261102/`）

### 4.3 固定されている物・されていない物

| 物 | 固定のされ方 | 実際の checkout | 分類 |
| --- | --- | --- | --- |
| lidar_localization_ros2 | `whill_lab.repos`：SHA 1dbcf3a | 1dbcf3a、変更なし | [B] |
| ndt_omp_ros2 | ブランチ `humble` | ef8a349 | [B] 固定されていない |
| velodyne | タグ 2.5.1 | 0f2a3bb（2.5.1）、変更なし | [B] |
| ros2_whill | タグ humble-with-odom-2026-06-18 | d2d1218 | [B] |
| ros2_whill_interfaces | ブランチ humble | 6617352 | [B] 固定されていない |
| rt_usb_9axisimu_driver | ブランチ humble-devel | f1bdaf4（タグ 2.1.0） | [B] 固定されていない |
| patchwork_plusplus | タグ v1.4.1 | 3e6903a、`ros/COLCON_IGNORE` が未追跡 | [B] |
| realsense-ros | タグ 4.55.1 | 8a86cb88、変更 1 件 | [B] |
| **GLIM / glim_ros2** | **`whill_lab.repos` に無い**。`scripts/install_glim.sh` は `GLIM_REF="master"`、`GLIM_ROS2_REF="master"`、`GTSAM_REF="4.3a0"`、`GTSAM_POINTS_REF="master"`、`IRIDESCENCE_REF="master"` | glim 88b3833（v1.2.1-3）で **config/ の 8 ファイルに未コミットの変更**、glim_ros2 a62811d。版は campus-m8 の metadata.yaml に記録されている | [B]。repos では固定されていない。記録はある |
| ローカルの GLIM 設定と campus-m8 の設定の違い | — | `src/third_party/glim/config/config_odometry_gpu.json` は `fix_imu_bias: true`、`docs/maps/campus-m8/glim_config/` は false。12 ファイルのうち 6 つが違う（config.json、config_global_mapping_cpu、config_odometry_cpu/gpu、config_ros、config_sub_mapping_cpu）。config_sensors.json は同じ | [B] |
| DUFOMap | metadata.yaml に「dufomap 1.1.1 (pip)」。このマシンの `pip show dufomap` は 1.1.1。リポジトリの中で版を固定している物は見つけていない（m5r_run_dufomap.sh の setup の節は読み切っていない） | 1.1.1 | [C] |
| 入力の bag | `~/bags/`（Git 管理外）。2026-07-10 の bag・bag-hires・07-24 の bag はディスクにある（ディレクトリを確認。bag-inventory のブランチの表にも載っている） | — | [B] |
| static.pcd | Git 管理外。正本は `~/bags/.../dufomap-m8-hires-noise-local/static.pcd` の 1 か所だけ（README：「危険：正本は 1 か所にしかない」）。外部への複製は「この作業では置いていない」 | sha 一致 | [A] |
| 作り直したときの一致 | README：「GLIM の GPU 実行は非決定的なので … sha256 は一致しない」 | — | [C]（README の記述。同じ設定で 2 回回して比べた記録は見ていない） |
| 占有格子の作り直し | 154w は入力・出力の sha256 を summary.json に記録し、手塗りのままで作り直した結果が既定の地図とバイト単位で同じことを確かめている（README） | — | [A]（オフライン） |

結論（事実だけ）：static.pcd は**リポジトリの中身だけからは作り直せない**（bag は `~/bags`、GLIM は repos で固定されていない、GPU の実行は非決定的）。占有格子は、static.pcd・`~/bags/m8-experiments/202d/`・追跡されているマスクがあれば手順が書かれている。

### 4.4 マージしていないブランチ（今の状態として扱わない）

`git branch -a --no-merged main` の結果。指定されたうち `feat/152-153-hires-map`、`exp/202-*`（4 本）、`exp/205-gsi-reference`、`feat/154-adopt-widened-map` は `git log main..<branch>` が空で、**マージ済み**。

| ブランチ | 中身 | この Track との関係 |
| --- | --- | --- |
| `feat/localization-check-script` | 692686a（2026-11-27）`check_localization.py` を足す。/alignment_status・/pcl_pose・/whill/odom を受けるだけ。Layer B と同じ条件（fitness > 1.0 が 2 s、未収束が 2 s、間隔 > 1 s）で FAIL、p95 > 0.065×3 で WARN。コミットの文面は「実機では未検証」 | 2026-11-27 の走行の判定（13a）に使った |
| `integration/186-190`（ローカル） | b74bdf2・e77c78f（2026-11-13）。#186 と #190 を統合した、push していないブランチ | 2026-11-13 の再生（行 11）はこの worktree でビルドした |
| `docs/bag-inventory-20261127` | 4bd9619：`docs/reports/2026-11-27-bag-inventory.md`（4453 行） | 地図の元 bag の場所と長さが載っている（2026-07-10 の bag 2162.0 s、237506 件など）。今回は grep で見ただけ |
| `fix/193-scan-silent` | failsafe の D:scan_silent | 別の Track |

---

## (5) 食い違い

- **D-1 ADR-0006 の状態**：`whill_lab.repos` のコメントは「ADR-0006 records this as the accepted version」、`m6r_lidar_localization.yaml:9` は「These are what ADR-0006 pins for the demo」と書いている。一方、`docs/decisions/0006-localizer-choice.md:4` は「Status: **proposed**」で、:106-114 の accepted にする 4 条件はすべて `[ ]`。ADR の決定 2（:29-31）は「`version: main`」と書いており、repos の SHA 固定と合わない。
- **D-2 07-14 の fitness**：`m6r_lidar_localization.yaml:6-8` は「M6R-2 §10.4 verify走行 2026-07-14, fitness range 0.02-0.3, reject count 0」と書いている。一方、`docs/m6r-bench-data/2026-07-14-verify-campus/manifest.yaml` の fitness は「良好 (静止時、閾値 6.0 に対して十分低い)」とだけあって数値が無い。0.0186〜0.0193 と ~0.3 は 07-12 の manifest の「操作者目視」の値。
- **D-3 旧 campus の点数・サイズ**：`campus/README.md` の表は「13M 点、208 MB」。ディスクのファイルは 115,798,736 B、ヘッダ POINTS 9,649,880（07-12・07-14 の manifest、campus-m8 README の 9,649,880 と一致）。
- **D-4 旧 campus の録画時間**：README の冒頭は「47 分」、概要は「2162 s / 36 分 2 秒」（README 自身が食い違いを注記している）。bag-inventory のブランチの表は 2162.0 s。
- **D-5 DUFOMap のパラメータ**：`campus/metadata.yaml` などの `erasor_params`（voxel_size_m 0.2、pr_threshold 0.8、rr_threshold 0.2）。一方、地図を作った頃の `m5r_run_dufomap_core.py`（7a9924a の時点）の既定は `--resolution 0.1`、`--d-s 0.2`、`--d-p 2`。campus-m8 の metadata.yaml は「旧フィールド名。中身は DUFOMap（ERASOR ではない）」と書いている。旧地図で実際に使った値は記録から決められない。
- **D-6 `campus/occupancy_trav.yaml`** は `image: occupancy_trav.pgm` を指すが、そのファイルはディスクにも Git にも無い（02f9aaf、2026-07-18）。
- **D-7 旧 campus と TF の関係**：`docs/runbook-live-demo.md:243` は「旧 campus は … #186 の前の TF で作った」。一方 `campus-m8/README.md:19` は「GLIM・DUFOMap は velodyne フレームの点群と IMU で動き、base_link→velodyne の TF は使っていない」。旧地図が依存しているのは誤った仰角表（VLP16db）と T_lidar_imu で、base_link→velodyne ではないという書き方と、runbook の書き方が合わない。
- **D-8 EKF の docstring**：`ekf_odom_launch.py:5`、`odom_bringup_launch.py:20` は `/imu/data_raw`。yaml の :109 は `/imu/data_rep145`。
- **D-9 M4-R の受入**：PR #41 は AC3・AC4 に `[x]`。数値は無く、`docs/m4r-bench-data` の Run index は空。
- **D-10 2026-11-27 の README と summary.json** の移動距離の列が合わない（13a の最後）。
- **D-11 metadata の `source_bag`**：旧 site の metadata.yaml は `docs/m5r-bench-data/.../bag/` を指すが、bag は `~/bags/m5r-bench-data/...` に移されている（`docs/m5r-bench-data/2026-07-10-campus-outer-final/` に `bag/` は無い）。
- **D-12 campus-m8 の「屋外の受入」**：#156 は OPEN。#156 のコメント（2026-11-16）は受入走行で「地点での /pcl_pose と椅子の位置の照合（#155）」と「回帰の基準の取り直し」をすると書いている。2026-11-27 の走行の README は、それらをしていない（絶対位置の照合なし、配車 Web の job は 0 件）。

## (6) 決められないことと、決めるのに要る物

1. **今の構成での自己位置の絶対誤差**：記録が無い。既知の地点（配車 4 地点）に椅子を置いて `/pcl_pose` と照合すること（#155/#156 の手順）が要る。fitness は TF の 10 cm のずれに鈍い（`docs/regression-check.md:39`）ので、fitness だけでは決められない。
2. **今の構成での EKF の性能**：今の IMU の取り付け・TF・`/imu/data_rep145` での 10 m 直進と静止 30 秒の bag と、`m4r3_ekf_bench.py` の出力が要る。M4-R の受入（2026-06-20）は構成が違い、数値も残っていない。
3. **IMU の入力共分散**：`linear_acceleration_stddev`・`angular_velocity_stddev` の実際の値。このリポジトリの launch（`imu_launch.py`）からドライバへ渡しているかを確かめていない。動いているノードのパラメータか launch を読む必要がある。
4. **IMU の TF と LiDAR の TF の整合**：TF の鎖の pitch −9.08° と audit −7.61° の差 1.47°（`calibration-ledger.md:217`）のうち、どちらがずれているか。README も台帳も決めていない。
5. **旧 campus の GLIM の実行時の設定**：記録はシンボリックリンクだけなので、値は分からない（#147 の報告）。
6. **campus-m8 の static.pcd を作り直したときの再現性**：同じ設定で 2 回回して比べた記録は見ていない。
7. **2026-11-27 の各数値の再計算**：この監査では bag を開いていない（指示による）。summary.json の値はそのまま写した。
8. **07-20 の Issue コメントの数値**：解析スクリプトが残っておらず、計算し直していない。
9. **campus-demo の smoke test の結果**：alignment_status.txt が空で、fitness が分からない。
10. **DUFOMap の版の固定**：リポジトリの中に固定があるかは、m5r_run_dufomap.sh の setup の節を読み切らないと言えない。

## (7) 読んだもの

**ファイル**：`docs/maps/*/metadata.yaml`（6 つ）、`docs/maps/*/occupancy*.yaml`、`docs/maps/README.md`、`docs/maps/campus/README.md`（1-60 行と見出し）、`docs/maps/campus-m8/README.md`（1-300、871-1000、1050-1080、1116-1378 行）、`docs/maps/campus-m8/{georef.yaml, waypoints.yaml（先頭）, glim_config/*.json（比較）, ndt_replay/**/*.json}`、`docs/maps/lab-legacy-m5b/README.md`、`docs/maps/_template/README.md`、`src/whill_safety/config/m6r_lidar_localization.yaml`、`src/whill_safety/launch/m6r_bringup_launch.py`（grep と 186-300 行）、`src/whill_navigation/launch/nav_launch.py`（grep）、`src/whill_dispatch/launch/dispatch_launch.py`（grep）、`scripts/start_dispatch_demo.sh`（grep、44-49 行）、`scripts/check_map_sync.py`（1-60 行と grep）、`scripts/m7_make_web_map.py`（grep）、`src/whill_dispatch/web/map_meta.json`、`src/third_party/lidar_localization_ros2/launch/lidar_localization.launch.py`、`src/whill_sensors_bringup/launch/static_tf_launch.py`（45-100 行）と `sensors_launch.py`（grep）、`src/whill_localization/config/ekf_odom.yaml`、`src/whill_localization/launch/ekf_odom_launch.py`、`odom_bringup_launch.py`（grep）、`src/third_party/ros2_whill/whill_driver/src/whill_node.cpp:234-239`、`src/third_party/rt_usb_9axisimu_driver/src/rt_usb_9axisimu_driver.cpp:380-404`、`src/whill_safety/whill_safety/failsafe_node.py`（定数）、`whill_lab.repos`、`.gitignore`（grep）、`docs/decisions/0006-localizer-choice.md`、`docs/m4r-bench-data/README.md`、`docs/m4r-whill-units.md`（211-260 行）、`docs/calibration-ledger.md`（13-26、184-200 行と grep）、`docs/m6r-bench-data/*/{run_config.yaml, alignment_status.txt, out.log（末尾・grep）, manifest.yaml}`、`docs/m6r-bench-data/2026-07-12-acceptance-campus/{terminal_b_localizer.log（集計）, bag/metadata.yaml}`、`docs/m7-bench-data/2026-07-22-live-campus/seimon-stall/README.md`、`docs/m7-bench-data/2026-07-19-mock/README.md`（grep）、`docs/calibration/2026-11-16-outdoor-static/{notes.csv, results.json}`、`docs/reports/2026-11-02-glim-imu-root-cause.md`、`docs/reports/2026-10-30-imu-handoff-audit.md`（先頭）、`docs/reports/2026-10-14-dispatch-demo.md`（先頭と grep）、`docs/scripts.md`、`docs/m5r-pipeline.md`（先頭）、`docs/regression-check.md`（grep）、`docs/runbook-live-demo.md`（grep）、`scripts/regression/baseline.json`、`scripts/regression/replay_localizer_launch.py`（先頭と grep）、`scripts/{pcd_to_occupancy_grid.py, m5r_pcd_to_occupancy.py, pcd_to_occupancy_v2.py, m5r3_run_glim.sh, imu_investigation/glim_control.sh, m5r_run_dufomap.sh, install_glim.sh}`（冒頭と grep）、`scripts/m5r_run_dufomap_core.py`（grep、7a9924a の版も）、`~/bags/2026-11-27-m8-drive/{README.md, analysis/summary.json, scripts_used/git_main_head.txt, scripts_used/check_localization_commit.txt}`。

**Issue / PR**（gh）：#37（本文・コメント 0）、PR #41・#42（本文）、#101・#108（本文・コメント）、#144・#145・#147・#149・#151・#152・#153・#154・#155・#156・#170・#171・#202（状態）、#149・#153・#154・#155・#156（コメント）、#190・#205（状態）。

**git**：`git log --first-parent main`（2026-07-22〜11-19）、地図ディレクトリごとの `git log`、`git log -S`（TF の値・VLP16_hires）、`git show m8-pre-calib-20261113:...static_tf_launch.py`、`git tag -l -n1`、`git branch -a --no-merged main`、`git log main..<branch>`、`git show --stat 692686a`、`git show docs/bag-inventory-20261127:docs/reports/2026-11-27-bag-inventory.md`（grep）、`git reflog`、`git diff 01f2fd8 HEAD -- m6r_lidar_localization.yaml`、`git -C src/third_party/* rev-parse/describe/status`。

**実行したもの**（すべて再起動の前）：PCD の sha256 3 つ（campus-m8、campus、`~/bags` の正本）と pgm の sha256 3 つ、PCD のヘッダ、`check_map_sync.py` を 3 通り、`pip show dufomap`、07-12 のログの fitness の集計（python）。
