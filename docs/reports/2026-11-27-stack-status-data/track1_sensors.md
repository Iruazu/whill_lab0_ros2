# Track 1 監査：センサ・校正値・静的 TF（main 64f7a49、2026-11-27）

読むだけの監査。リポジトリのファイルは変えていない。ROS ノードは起動していない。
証拠の区分：
- **[A] 実測**：日付・条件・数値つきの測定記録がある（生データの所在も書く）
- **[B] コード・設定から読んだだけ**（実機での挙動は示されていない）
- **[C] 文書に書いてあるだけ**（生データは見つからない）
- **[D] 証拠なし、または食い違い**（両方を原文のまま書く）

前提の事実（git で確認）：
- 下の §5 に挙げた関連ブランチ（feat/144-145-lidar-calib、docs/149-outdoor-static-results、fix/146-camera-tf、docs/143-footprint-spec-sources、investigate/147-imu-audit、feat/149-field-tools、feat/149-lidar-yaw-method、feat/152-153-hires-map、fix/141-preflight-device-aliases、fix/192-patchwork-sensor-height）は、どれも `git rev-list --count main..<branch>` が 0。**未マージのコミットは無い**。
- 例外：worktree `.claude/worktrees/agent-aecfa03d91f4cdcc6`（ブランチ feat/144-145-lidar-calib、HEAD 3b9d250 は main に含まれる）に、**コミットされていないステージ済みの変更**がある（21 ファイル、+34 / −1296 行）。中身は台帳 §2.1 を古い版（「z 0.777 m（launch の値）」「候補が 2 つある…ユーザー判断、#149 で確定」）に戻す差分、`imu_sign_corrector.py` の #169 の変更と `test_imu_sign_corrector_shutdown.py` の削除などで、**現行ではない**。この監査の値はすべて main のもの。
- `src/third_party/` は `.gitignore`（8 行目）で git 管理外。版は `whill_lab.repos` で固定し、作業ツリーの実物を `git -C` で確認した（velodyne 2.5.1 / 0f2a3bb、rt_usb_9axisimu_driver 2.1.0 / f1bdaf4、ros2_whill humble-with-odom-2026-06-18 / d2d1218、realsense-ros 4.55.1 / 8a86cb88 だが後述のとおり使われていない）。

---

## 1. 現在の値の一覧

### 1.1 VLP-16（ドライバと点群）

| 項目 | 現在の値（main） | 場所 | 由来 | 区分 |
| --- | --- | --- | --- | --- |
| 機種 | 「VLP-16 Puck Hi-Res（上下 ±10°）」 | `docs/calibration-ledger.md:162`。`sensors_launch.py:32-33` | 「本体の銘板」2026-10-28。銘板の写真・記録はリポジトリに無い（`find docs -iname '*plate*'` などで 0 件） | [C] |
| ドライバの model / rpm / port / device_ip | `model: VLP16`、`rpm: 600.0`、`port`・`device_ip`（値はネットワーク設定のため省略。上流の既定）、`gps_time: false`、`timestamp_first_packet: false`、`frame_id: velodyne` | 上流 `src/third_party/velodyne/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml:1-14`（`install/` の複製と同一）。`sensors_launch.py:97-102` が上書きなしで読む | 上流の既定。`whill_lab.repos` のコメント「Defaults inherited from the noetic stack — no IP override」 | [B] |
| 校正ファイル（仰角表） | 既定 `<velodyne_pointcloud share>/params/VLP16_hires_db.yaml` | `sensors_launch.py:150-158`（引数 `velodyne_calibration`）、ノードへの上書き `:110-114`。上位でも同じ既定：`src/whill_localization/launch/odom_bringup_launch.py:72-74,101-107`、`src/whill_safety/launch/m6r_bringup_launch.py:254,293-313` | #144（PR #186、2026-11-13 マージ） | [B]（値）／[A]（効果の測定は §2.1） |
| 仰角表の中身 | `VLP16_hires_db.yaml` の `vert_correction` は ±0.17453292519943295 rad（±10°）から 16 本、`vert_offset_correction` は全部 `0.0` | `install/velodyne_pointcloud/share/velodyne_pointcloud/params/VLP16_hires_db.yaml:4-49` | 上流の汎用の表。**個体の工場校正（db.xml など）ではない**。個体の校正ファイルはリポジトリに無い（grep で 0 件） | [B] |
| （旧）VLP16db.yaml | `vert_offset_correction` が ±0.0007〜±0.0112（16 本、全部違う値） | `install/.../params/VLP16db.yaml` | 上流 | [B] |
| transform node | `min_range: 0.9`、`max_range: 130.0`、`organize_cloud: true`、`view_direction: 0.0`、`fixed_frame: ""`、`target_frame: ""`、`model: VLP16`（`calibration: VLP16db.yaml` は launch で上書き） | `src/third_party/velodyne/velodyne_pointcloud/config/VLP16-velodyne_transform_node-params.yaml:15-24` | 上流の既定 | [B] |
| 点ごとの時刻 | 点の `time` 欄がある。「最小値の中央値は −0.100122 s、最大値は +0.001306 s」 | `docs/reports/2026-10-30-imu-handoff-audit.md`「時刻・単位・静止区間」 | 07-10 の bag の再集計（生データ `~/bags/imu_investigation_20261030/`） | [A]（07-10 の bag について） |
| laserscan node | `ring: -1`、`resolution: 0.007`。出力は `/scan` → `/scan_raw` に remap | 上流 `default-velodyne_laserscan_node-params.yaml:25-28`、`sensors_launch.py:115-124` | 上流、#80 | [B] |
| 有線ネットワーク | ホストの固定アドレス（値は省略）、テンプレ `network/01-velodyne-static.yaml.template`、`scripts/install_velodyne_network.sh <NIC名>` | `docs/m3-sensors.md:112-123` | M3（2026-05-07） | [B] |
| レート | 11-16 の静止確認で「/velodyne_packets と /velodyne_points は各本 115〜117 通（9.7〜9.9 Hz、公称 10 Hz）」 | Issue #150 のコメント | 2026-11-16、12 秒 × 8 本 | [C]（数え方の記録・出力ファイルはリポジトリに無い） |

### 1.2 静的 TF（`src/whill_sensors_bringup/launch/static_tf_launch.py`）

| TF | 現在の値 | 場所 | 由来 | 区分 |
| --- | --- | --- | --- | --- |
| base_link → imu_link 並進 | `0.38, -0.03, 0.47` m | `:56` | 巻尺 2026-06-24（#61、`b2cb7df`）。台帳 §4.1 `:188-190`、誤差「未評価」 | [C]（巻尺の記録は数値のみ） |
| base_link → imu_link 回転 | `-0.0713, -0.1415, 0.0` rad（コメント「roll=-4.09°, pitch=-8.11°, yaw=0 (2026-07-10 morning)」） | `:57` | 2026-07-10 朝の静止 gravity 逆算 `ax = +1.397 ay = -0.699 az = +9.782`（`docs/imu-coordinate-audit.md:428-431`、`scratchpad/imu_live_check.py`・`scratchpad/frame_audit.py`、両方 git 管理）。yaw 0 は「axis-aligned に付け直し（目視）」2026-07-08 | roll/pitch [A]（ただし数値は文書の転記のみで、サンプルの生ファイルは無い。数値の出所は 3 回の live check）／yaw [C] |
| base_link → velodyne x, y | `0.42, 0.36` m | `:86` | 巻尺 2 回 2026-10-28（台帳 `:65-66`、計画書 `docs/plans/2026-10-28-avoidance-baseline.md:16`）。x は 2026-11-16 の屋外静止確認 D でも確認（§2.3） | x [A]／y [C]（台帳 `:66` 誤差「未評価」） |
| base_link → velodyne z, roll, pitch | `0.771` m、`math.radians(-0.05)`、`math.radians(0.40)` | `:86-87` | 2026-11-13 屋内静止確認 C（台帳 `:124-138`、bag `~/bags/m8-static-check-20261113/fixed_*` が実在） | [A] |
| base_link → velodyne yaw | `math.radians(-0.317)` | `:88` | noetic の LiDAR↔IMU extrinsic の yaw（`docs/m3-extrinsics-from-noetic.md` の `yaw = atan2(-0.005459, 0.987688) = -0.005527 rad (-0.3167 deg)`）。台帳 `:70`「**未測定（旧値の流用、0 と同等）**」 | [D]（試行 2 つが約 2° 食い違う。§3） |
| base_link → camera_link | `0.47, 0.378, 0.73, 0.0, 0.0, 0.0` | `:101` | 2026-10-28 巻尺 + データシート（台帳 §3 `:170-175`）。PR #185（2026-11-12 マージ） | x,y,z [C]（導出は台帳のみ）。回転 0 は「水平・前向き取付の申告」[C]、「床面フィットで約 1° 上向きの疑い」 |
| （参考）camera_link 以下 | realsense2_camera が出す。11-16 の bag の `/tf_static` で `camera_link→camera_color_frame` = (−0.0003, 0.0148, 0.0002)、q (0.006933, −0.001237, 0.000214, 0.999975) | 11-16 bag（前回の作業で読んだ。今回は bag を開いていない） | カメラ本体の工場値 | [B] |

記録された bag の `/tf_static`（前回の作業で直接読んだ値）：
- `~/bags/m8-static-check-20261113/{flat_1,fixed_2_place2,fixed_5_place3_rot180}`：base_link→velodyne = (0.4841, 0.3815, 0.7937)、q (−0.017399, 0.078447, −0.001369, 0.996765) → **#145 より前の TF で記録**（PR #186 のマージ 2026-11-13 16:24 JST より前）。床面フィットは velodyne 系で行うので、記録時の TF はフィット結果に入らない（台帳の方法の記述による）。C の bag の記録時の校正表は台帳に書かれていない（B は「校正表は旧 VLP16db のまま」と明記、台帳 `:112`）。
- `~/bags/2026-11-16-static/empty_start_140915`：base_link→velodyne (0.42, 0.36, 0.771)、camera_link (0.47, 0.378, 0.73)、imu_link (0.38, −0.03, 0.47) → 現行の TF で記録。
- `~/bags/2026-11-27-m8-drive/`：README によれば main `64f7a49` で記録（`scripts_used/git_main_head.txt`）、カメラなし。したがって現行の TF・Hi-Res の表で動いたと読めるが、bag の `/tf_static` は今回は開いていない（機械を軽く保つ指示のため）。[C]

### 1.3 IMU（RT USB 9 軸）

| 項目 | 現在の値 | 場所 | 由来 | 区分 |
| --- | --- | --- | --- | --- |
| port / frame | `/dev/imu`（udev `2b72:0003`）、`imu_link` | `src/whill_sensors_bringup/launch/imu_launch.py:27-38`、`udev/99-whill-stack.rules` | — | [B] |
| ドライバのパラメータ | launch は `port`・`frame_id` だけ渡す。上流の既定 `linear_acceleration_stddev 0.023145`、`angular_velocity_stddev 0.0010621`、`magnetic_field_stddev 0.00000080786` | `imu_launch.py:46-49`、`src/third_party/rt_usb_9axisimu_driver/src/rt_usb_9axisimu_driver_component.cpp:59-63` | 上流の既定（本機で測った値ではない） | [B] |
| 共分散 | 対角 = stddev²。`orientation_covariance[0] = -1` | 同 `rt_usb_9axisimu_driver.cpp:390-404` | 上流 | [B] |
| 加速度の単位 | g → m/s²（`CONVERTOR_G2A(9.80665)`） | `include/rt_usb_9axisimu_driver/rt_usb_9axisimu.hpp:130` | 上流 | [B] |
| 符号の補正 | `/imu/data_raw` の加速度 3 軸を反転して `/imu/data_rep145`。gyro・mag・orientation・共分散はそのまま | `src/whill_sensors_bringup/whill_sensors_bringup/imu_sign_corrector.py:79-81` | #56。根拠「M5R-3 (#48) で実測 (`x=-0.46, y=-2.16, z=-9.71` at rest)」（Issue #56 本文） | 加速度の反転 [C]（数値は Issue のみ）。gyro が REP-103 準拠という点は Issue #56 で「暫定判定。本 Issue 実装中に rotate 試験で再確認する」とあり、再確認の記録は見つからない → [D] |
| EKF の入力 | `imu0: /imu/data_rep145`、`imu0_config` は角速度 3 軸と ax, ay | `src/whill_localization/config/ekf_odom.yaml:108-118` | — | [B] |
| 期待値（走行前後の確認） | ax +1.397 / ay −0.699 / az +9.782 m/s²（許容 ±0.05）、gx −0.0184 rad/s（±0.005） | 台帳 `:20-25` | 2026-07-10 朝 | [A]（§1.2 と同じ測定） |

### 1.4 WHILL ドライバ（ros2_whill fork）

| 項目 | 現在の値 | 場所 | 由来 | 区分 |
| --- | --- | --- | --- | --- |
| port | `/dev/whill`（launch で上書き）。上流 yaml は `/dev/ttyUSB0` | `odom_bringup_launch.py:125-134`、`src/third_party/ros2_whill/whill_bringup/config/params.yaml:3` | 2026-07-22 の不具合（docstring） | [B] |
| publish_interval_ms | `400`（上流 params.yaml）。コードの既定は 500 | 同 `params.yaml:4`、`whill_node.cpp`「kDefaulPpublishIntervalMs = 500」 | 上流 | [B] |
| 車輪半径 | `kWheelRadius = 0.1325` m | `src/third_party/ros2_whill/whill_driver/src/whill_node.cpp:40` | 「WHILL Model CR2 URDF」の公称値。台帳 §1 `:44` は「ROS ドライバ `wheel_radius`、FDA K261175」で「公式」 | [C]（本機での実測なし。`docs/m4r-whill-units.md:236`「(未実測)」） |
| トレッド | `kTread = 0.496` m | `whill_node.cpp:41` | 上流のコメント（`cmd_vel` 換算）。台帳 `:43`「URDF は 0.52、ROS1 xacro は 0.49。運動学に使う 0.496 を採る」 | [C]（`m4r-whill-units.md:237`「(未実測)」） |
| odom の単位 | motor_angle `0.001` rad/LSB、motor_speed `0.004` km/h/LSB。odom は角度の差分から計算、右輪を反転 | `src/third_party/ros2_whill/whill_driver/src/model_cr2/whill.cpp:62-69`、`whill_node.cpp:130-200` | 単位は上流の宣言 + 2026-06-14 の粗い裏取り（「約 2 m 前進、1 回試行、メジャー計測なしの粗い裏取り」、`m4r-whill-units.md:210-245`） | [B]（単位）／距離スケールは [D]：「1 m 直進試験…数値合格判定は M4R-3 に持ち越し」。M4R-3（Issue #37）の受入基準「手押し 10 m 直進…≤ 0.5 m」は Issue にチェックの記録が無く、`docs/m4r-bench-data/` は README（手順）だけで実行記録が無い |
| odom の共分散 | pose/twist の x, y, yaw に `0.01`（「precision unknown」） | `whill_node.cpp`（odom.pose.covariance[0]… 行） | 仮置き | [B] |
| cmd_vel 換算 | `linear.x * 900`、`angular.z * -446.4` | `whill_node.cpp` `OnControllerCmdVel` | 上流 | [B] |

### 1.5 RealSense D435（opt-in）

| 項目 | 現在の値 | 場所 | 区分 |
| --- | --- | --- | --- |
| 起動 | 既定 `realsense:=false`。真のとき `realsense2_camera/launch/rs_launch.py` を引数なしで include | `sensors_launch.py:140-148,161-165` | [B] |
| 使われるドライバ | `src/third_party/realsense-ros/COLCON_IGNORE`（git 管理外、2026-10-28 17:41 作成）により、apt の `ros-humble-realsense2-camera 4.58.4` が使われる（`ros2 pkg prefix realsense2_camera` → `/opt/ros/humble`） | 計画書 `docs/plans/2026-10-28-avoidance-baseline.md:111-114` | [B] |
| FW | 5.17.3.10（2026-10-30 更新）、校正表は FW 更新の前後でバイト単位で一致 | 台帳 §5 `:224-225`。バックアップ `~/rs_backup_938422070073/` | [C] |
| 深度の校正（OCC / Tare） | 未実施 | 台帳 `:227`、Issue #148 OPEN | [D]（値なし） |
| GLIM 設定の T_lidar_camera | `[-0.0032, -0.05292, 0.00108, 0.00837, 0.72559, -0.00328, 0.68807]`、画像 752×480 | `docs/maps/campus-m8/glim_config/config_sensors.json:3-11,33-36` | 上流 GLIM の既定とみられる値で D435 とは無関係（カメラは GLIM で使っていない）。出所の記録なし → [D] |

### 1.6 footprint

| 項目 | 現在の値 | 場所 | 区分 |
| --- | --- | --- | --- |
| Nav2 の外形 | `robot_radius: 0.45`（local・global） | `src/whill_navigation/config/nav2_params.yaml:231`、`:282` | [B]。出典なし（台帳 `:51`「実測なし」「要変更（#157）」） |
| failsafe の車幅の前提 | 「the chair's 0.6 m width」、半角 30° | `src/whill_safety/whill_safety/failsafe_node.py:72`（コメント）、`:85` | [B]／幅 0.6 m の出典なし（台帳 `:52`） |
| 公式寸法 | 全幅 0.554 m、全長 0.985 m、トレッド 0.496 m、後輪半径 0.1325 m、前輪径 0.257 m、最小回転半径 0.760 m | 台帳 `:40-46`（PR #200） | [C]（外部の公式資料の転記。URL つき） |
| 車軸 → 前端 | 0.72〜0.75 m（±2 cm） | 台帳 `:47`、Issue #143 コメント | [C]（2026-11-16 屋外の巻尺、数値のみ） |
| 車軸 → 後端・最大幅・出っ張り | 未測定（後端は「約 0.24〜0.26 m（全長 0.985 − 前端から推定）」） | 台帳 `:48-50` | [D]（値なし） |

### 1.7 GLIM（地図作成のみ）の T_lidar_imu

| モード | 値 [x, y, z, qx, qy, qz, qw] | 場所 | 由来 | 区分 |
| --- | --- | --- | --- | --- |
| baseline（既定） | `-0.05, -0.4, -0.35, 0.017399, -0.078447, 0.001369, 0.996765`（台帳では RPY (+2°, −9°, 0°)） | `scripts/m5r3_run_glim.sh:293-302` | noetic の `extrinsic_T`/`extrinsic_R` の逆（`b5bc6ce` 2026-06-21、#48）。noetic 時代の取り付けの値 | [C] |
| audit（`GLIM_TLI_FROM_AUDIT=1`） | `-0.05, -0.4, -0.35, -0.035606, -0.066319, -0.002368, 0.997163`（RPY (−4.09°, −7.61°, 0°)） | `scripts/m5r3_run_glim.sh:333-342` | 07-10 朝の IMU の roll/pitch と「LiDAR を pitch −0.5° と仮定」（`m5r3_run_glim.sh:317-327`、`39bf794`）。並進は noetic のまま | 回転は [C]（下の §2.5 の注意を参照）、並進は [C]（noetic 流用） |
| campus-m8 で実際に使った値 | audit と同じ `[-0.05, -0.4, -0.35, -0.035606, -0.066319, -0.002368, 0.997163]`、`imu_acc_noise 0.008`、`imu_gyro_noise 0.0013`、`imu_bias_noise 1e-05` | `docs/maps/campus-m8/glim_config/config_sensors.json:12-20,37-39`、`docs/maps/campus-m8/metadata.yaml:17` | — | [B] |
| 現在の TF の鎖から計算した値（この監査で計算） | 並進 (−0.0357, −0.39, −0.3016)、RPY (−4.037°, −8.508°, +0.31°) | §1.2 の imu_link と velodyne の値から `R_bv⁻¹·R_bi`、`R_bv⁻¹(t_bi − t_bv)`（scipy、固定軸 xyz） | 計算値。どの bag でも使っていない | [B] |

---

## 2. 変更履歴：何が違っていて、いつ直ったか

コミットの時刻は `git log --date=iso`、PR のマージ時刻は `gh pr view`（UTC。JST は +9 h）。

### 2.1 LiDAR の校正ファイル（#144）— main にマージ済み、Issue CLOSED

- **違っていた値**：`VLP16db.yaml`（±15°）。上流 `velodyne-all-nodes-VLP16-launch.py` の include で固定（`git show 02007d5:src/whill_sensors_bringup/launch/sensors_launch.py:40`、`git show 4bd9139:...:93`）。
- **期間**：パッケージ作成 `02007d5`（2026-05-07）から PR #186 のマージ `5392200`（2026-11-13T07:24:03Z = 16:24 JST）まで。この期間の main の bringup は全部 VLP16db。
- **正しいとする値**：`VLP16_hires_db.yaml`（`7165750`、2026-11-12。上位 launch への引き渡しは `7de4718`）。
- **根拠**：
  - 機種 = Hi-Res は「本体の銘板」のみ（台帳 `:162`）。写真・記録なし [C]。
  - 床面フィット：VLP16db のままでは z 1.144〜1.147 m、Hi-Res で 0.771〜0.773 m（台帳 `:93-106`、bag `~/bags/sensor_check_20261028/` の 5 本、`scripts/lidar_floor_fit.py`）[A]。巻尺の旧記録は「0.80 m」（stale worktree の台帳差分にのみ。main の台帳では「旧記録の巻尺 0.80 m」の行は消えている）。
  - 2026-11-16 屋外のコーン：最上 ring の高さと予測の差 `dz_ring` は 7 本で 0.004 / 0.002 / 0.001 / 0.001 / 0.0 / −0.0 / −0.0 m（`docs/calibration/2026-11-16-outdoor-static/results.json`、`calibration` は全 bag `"hires"`）[A]。これは ring の仰角が Hi-Res の表どおりであることと整合する。
  - 解析的な補正とドライバの一致：段階 A の bag 2 本で最大の差 0.015 mm、p99.9 0.004 mm、NaN・ring の不一致 0（`docs/maps/campus-m8/README.md:72-78`）。Issue #144 の完了条件「0 mm で一致」は README で「小数第2位までなら 0.02 mm であり、厳密な 0 ではない」と書き直されている [A]。
  - GLIM の IMU の改善（§2.6）は副次の裏付け [A]（300 秒の区間）。
- **未検証**：仰角表は上流の汎用値で、`vert_offset_correction` は全部 0（§1.1）。**個体の工場校正値との比較は無い**。Issue #144 の題「実機と違う」は機種（±15° → ±10°）の違いのことで、個体差の補正はしていない。

### 2.2 base_link → velodyne（#145）— main にマージ済み、Issue CLOSED（屋外は #149 で継続）

| 期間（main） | 値 | コミット |
| --- | --- | --- |
| 2026-05-07〜06-20 | identity（0, 0, 0, 0, 0, 0） | `02007d5` |
| 2026-06-20〜06-24 | (0.304136, 0.411548, 0.823704) / RPY (−0.035342, 0.156983, −0.005527) rad | `78f6658`（#36） |
| 2026-06-24〜2026-11-13 16:24 JST | (0.484136, 0.381548, 0.793704) / RPY (−0.035342, +0.156983, −0.005527) rad（pitch +8.9945°） | `b2cb7df`（#61） |
| main には入らなかった中間値（ブランチのみ） | (0.42, 0.36, 0.777) / (0.24°, 0.97°, −0.317°) | `7165750`（2026-11-12、feat/144-145）。`9f6ad27`（2026-11-13 15:46）で置換 |
| 2026-11-13 16:24 JST〜現在 | (0.42, 0.36, 0.771) / (−0.05°, +0.40°, −0.317°) | `9f6ad27` → PR #186 `5392200` |

- **違っていた理由**（台帳 `:156`、`static_tf_launch.py:78-82`）：旧値は「IMU TF に noetic の LiDAR↔IMU extrinsic を足したもの」。`docs/m3-extrinsics-from-noetic.md` の並進 `(0.38, -0.03, 0.47) + (0.104136, 0.411548, 0.323704) = (0.484136, 0.381548, 0.793704)` と RPY 分解がその計算。pitch +9° は相対姿勢で、対 base_link ではなかった。
- **旧値が誤りだったことの証拠**：2026-07-09 の時点で `docs/imu-coordinate-audit.md:237-248`（§7.1）が「LiDAR は base_link に対してほぼ完全に水平 (pitch ≈ -0.5°、roll ≈ 0°)」「シナリオ B 確定」と書いている。**それでも static TF は 2026-11-13 まで +8.99° のまま**だった（git の値の履歴で確認）。注意：§7.1 の根拠「Ring 0..15 の効果角度…全リングで 14.83-14.87° = -15° 校正値と完全一致」は VLP16db で計算した点群で行われていて、仰角表が ±15° なら点の仰角は作り方の上で表の角度になる（`vert_offset_correction` が小さいため）。この根拠は取り付け角の独立な測定とは読めない [D]。
- **正しいとする値の根拠**：
  - z / roll / pitch：屋内静止確認 C、2026-11-13、テープ固定後、180° 回した 2 本 × 2 組（台帳 `:124-138`）。1 組目平均 (0.773 m, −0.03°, +0.41°)、2 組目平均 (0.770 m, −0.07°, +0.39°)、採用 (0.771, −0.05, +0.40)。bag は `~/bags/m8-static-check-20261113/fixed_1..5`（実在）[A]。
  - x：屋外 D でコーンの形を補正した残差「+0.6 / +0.2 / +0.6 / +0.8 / +0.8 / −0.5 / −1.2 cm」（台帳 `:145`）。**生の `dx` は results.json にある**（−0.013 / −0.029 / −0.037 / −0.047 / −0.059 / −0.083 / −0.114 m）が、**コーンの半径を引く計算は results.json にも `scripts/check_object_placement.py` にも無い**（`radius` の文字列 0 件）。この監査で「円錐 φ0.21 m が台座 0.37 m + 円柱 0.04 m の上から 0.725 m まで」と仮定し、`x_front_z` から床高を引いた高さで半径を出して足すと +2.4 / +3.5 / +5.4 / +7.1 / +8.6 / +8.8 / +11.1 cm になり、**台帳の残差は再現しなかった**（仮定した形・高さの取り方が台帳と違う可能性があり、どちらが正しいかは決められない）。→ x の「1 cm 以内」は [C]、生の dx は [A]。
  - y：屋外 D で「両センサの y は 3 cm 以内で一致」（台帳 `:146`）。LiDAR と深度は同じ y 0.36 系から出ているので（camera の y は LiDAR の巻尺値から導出、台帳 `:178`）、独立な検証ではない。台帳 `:66` は「未評価」のまま [C]。
  - yaw：§3 の D を参照。
- **未検証**：yaw。「水平器 + テープ」の復帰手順の再現性（台帳 `:82`「**未測定**」）。着座時（台帳 `:120` `flat_3_seated` 1 回のみ、インライア 68）。

### 2.3 base_link → camera_link（#146）— PR #185 は main にマージ済み、Issue は OPEN（#148 待ち）

| 期間（main） | 値 | コミット |
| --- | --- | --- |
| 〜2026-06-20 | identity | `02007d5` |
| 2026-06-20〜06-24 | (0.36, 0.412, 0.82) / 0 | `78f6658` |
| 2026-06-24〜2026-11-12 18:14 JST | (0.54, 0.382, 0.79) / 0 | `b2cb7df`（「Issue #61 で LiDAR/IMU の共締めユニットが平行移動したため同じ delta だけスライド済」、`m3-extrinsics-from-noetic.md`） |
| 2026-11-12〜現在 | (0.47, 0.378, 0.73) / 0 | `c00e527` → PR #185 `a1689bc`（2026-11-12T09:14:09Z） |

- 旧値の誤りの説明：「旧 z 0.79 では深度の物体高が常に +6 cm 高く出た」（台帳 `:261`）[C]。
- 新値の根拠：x = 0.42 + 約 0.05 − 約 0.004、y = 0.36 + 0.0175、z はレンズ中心の巻尺（台帳 `:172-174`）[C]。2026-11-16 の再測は「車軸→レンズ約 0.46 m」（台帳 `:172`）で、TF の 0.47 と 1 cm 違う（台帳は TF を変えていない）。
- 屋外 D の深度の結果：深度の `dx`（生）は 1.5 m −0.006、2.0 m 0.0、2.5 m +0.013、3.0 m +0.078、3.5 m +0.086、4.0 m +0.188、5.0 m +0.492 m（results.json）[A]。台帳の「コーンの半径を引いた残差 +5.2 / +6.5 / +5.5 cm」は半径の補正後の値で、計算は repo に無い [C]。
- 回転 0 は申告のみ、「約 1° 上向きの疑い」は未評価 [D]。
- **未検証**：Issue #146 の完了条件「静止確認で深度の物の位置が許容内」は未判定（Issue コメント「#148 の校正の後に判定する」）。

### 2.4 base_link → imu_link — 変更は 2026-07-10 が最後

| コミット | 日時 | 回転 (roll, pitch, yaw) rad | 注記 |
| --- | --- | --- | --- |
| `78f6658` | 2026-06-20 | (0, 0, 0)、並進 (0.20, 0.00, 0.50) | M4R-2 |
| `b2cb7df` | 2026-06-24 | (0, 0, 0)、並進 (0.38, −0.03, 0.47) | #61 巻尺 |
| `d9edac9` | 2026-07-08 11:53 | (0.1396, 0.0, −1.5708) | 「roll=+8°, pitch=0, yaw=-90°」 |
| `6c70d9b` | 2026-07-08 13:28 | (0.0, −0.1396, 0.0) | 物理的に 90° 付け直し後 |
| `80af31f` | 2026-07-09 09:46 | (−0.1007, −0.1396, 0.0) | roll −5.77° 追加（07-08 bag 1000 サンプル） |
| `171b01d` | 2026-07-09 16:12 | (−0.1089, −0.1268, 0.0) | 再固定 1 回目 |
| `aed1e4d` | 2026-07-09 16:19 | (−0.0614, −0.1354, 0.0) | 再固定 2 回目 |
| `9cc4be2` | 2026-07-10 15:03 | (−0.0713, −0.1415, 0.0) | 07-10 朝の値。現行 |

- 07-10 の地図 bag（`2026-07-10-campus-outer-final`、15:07 開始）は `9cc4be2`（15:03）の直後。
- **走行後の値は許容外**：「ax = +1.331  ay = -0.577  az = +9.768」、逆算「roll = -3.38°, pitch = -7.75°」（`docs/imu-coordinate-audit.md:453-462`）。台帳 `:218` も「§0 の許容 ±0.05 を超える…計算し直すべきだったが、していない」と書く [A]（数値は文書の転記）。
- IMU の取り付けは 2026-07-10 以降測り直した記録が無い。**2026-11 の時点の IMU の取り付け角は未測定**（11-13 の静止 bag に `/imu/data_rep145` は記録されているが、台帳はそれを IMU の確認に使っていない）[D]。

### 2.5 GLIM の T_lidar_imu

| コミット | 日時 | 内容 |
| --- | --- | --- |
| `b5bc6ce` | 2026-06-21 | baseline（noetic の逆）を追加（#48） |
| `494ea77` | 2026-07-09 | audit（q −0.030651, −0.063283, −0.001945, 0.997523）を環境変数で追加 |
| `39bf794` | 2026-07-10 | audit を (−0.035606, −0.066319, −0.002368, 0.997163) に更新。現行 |

- audit の回転は「LiDAR ≈ level in base_link (pitch -0.5°, roll ~0°)」と IMU の 07-10 朝の値から（`m5r3_run_glim.sh:317-327`）。LiDAR の −0.5° は 2026-07-09 の VLP16db の点群での推定（台帳 `:214`「#144 以前の点群で推定」）。Hi-Res での LiDAR の対 base_link pitch は +0.40°（11-13、テープ固定後）または +0.87°（11-13、固定前 B）で、どちらも −0.5° と違う。ただし 07-10 時点の LiDAR の取り付けは測っていない [D]。
- campus-m8 の作成時の比較（`docs/maps/campus-m8/README.md:113-121`）：実行 C（当時の TF の鎖から計算した RPY (−4.33°, −9.08°, +0.36°)）は IMU の指標が悪化し地図の傾き 2.04°。audit（A）を採用、傾き 0.72°。README の結論「どちらかの TF（imu_link 側か velodyne 側）、または両方が実物とずれている可能性がある…この地図では決めていない」。
- 台帳 §4.3 `:217` の「§2.1 と §4.1 から計算した T_lidar_imu は…RPY (−4.33°, −9.08°, +0.36°)」は **11-12 の TF（pitch 0.97°）での値**。今の TF（pitch 0.40°）で計算すると RPY (−4.037°, −8.508°, +0.31°)、並進 (−0.0357, −0.39, −0.3016)（この監査の計算、§1.7）。台帳は更新されていない。
- **未検証**：audit の並進 (−0.05, −0.4, −0.35) は noetic 由来のままで、TF の鎖の並進（z −0.3016）と z で約 4.8 cm 違う。どちらが実物かの測定は無い。

### 2.6 #147（GLIM の IMU）— CLOSED（2026-11-02）、PR #173 マージ済み

- 結論（`docs/reports/2026-11-02-glim-imu-root-cause.md`）：「主な原因は、LiDAR の校正ファイルの誤り（Hi-Res の実機に VLP16db.yaml）」。同じ 300 秒・各 3 回：元の点群で better ratio 並進 0.03・速度 0.06、Hi-Res で 0.39–0.40・0.62–0.64（fix_imu_bias false）。速度の誤差 IMU 0.094–0.097 → 0.019 m/s。加速度バイアスの最大 1.01–1.07 → 0.11–0.14 m/s²。生データ `~/bags/imu_investigation_20261030/glim_control_20261102/`、要約 `docs/reports/2026-11-02-glim-imu-data/control_summary.txt` [A]（300 秒区間、GPU 実行）。
- 2026-10-30 の監査（`docs/reports/2026-10-30-imu-handoff-audit.md`）の「IMU が効いていない見方は支持されない」は 11-02 に訂正済み（同レポート冒頭の追記と Issue #147 のコメント）。
- 報告書自身が「仕組み（推測、確かめていない）」「高さの幅が小さくなったことは、精度の証明ではない」と書いている。全行程での確認は #153（campus-m8 README の better ratio の表）。
- 未解決：GLIM の IMU 対 LiDAR の相対姿勢が TF の鎖と合わない件（Issue #153 のクローズコメント「#147 で扱う」と書かれているが、#147 は既に CLOSED）[D]。

### 2.7 #149（屋外静止確認）— Issue OPEN。結果の記録 PR #198 はマージ済み

- 屋内前段 2026-11-13（§2.2）と屋外 D 2026-11-16（キャンパスのアスファルト、無人、コーン 7 本 + 空 1 本、各 12 秒）。生 bag `~/bags/2026-11-16-static/`（実在）、解析結果の写し `docs/calibration/2026-11-16-outdoor-static/results.json` と `notes.csv` は `~/bags/2026-11-16-static/analysis/results.json`・`notes.csv` と `cmp` で一致 [A]。
- `~/bags/2026-11-14-static/` は**空のディレクトリ**。台帳・Issue に 11-14 の測定の記録は無い。
- 残り（Issue のコメント 2026-11-16）：「残りは yaw だけ」。低い物の深度チェックは #148 に移した。
- 路面：`slope_x_deg 0.38`、`slope_y_deg -0.24`（results.json `floor.lidar.plane`）。

### 2.8 #150（D435 と USB-LAN のハブ共有）— CLOSED 2026-11-16

- 根拠は Issue のコメントの件数のみ（「各本 115〜117 通」「深度は 352〜355 枚（29.7〜30 Hz）」）。数えたスクリプト・出力はリポジトリに無い [C]。静止・12 秒の条件のみ。走行中の長時間は未確認（コメント「#156 の記録 bag で同じ数え方で確認」）。11-27 の走行はカメラなし（README）なので、この確認にはならない。

### 2.9 #151（台帳を唯一の正本に）— CLOSED、PR #186

- 完了条件「台帳と launch・設定の値が一致し、食い違いが無い」。値は launch と一致するが、台帳の中の file:line と記述に古い所が残る（§3）。

### 2.10 #143（footprint）— Issue OPEN。PR #200（公式仕様の転記）はマージ済み

- 実測は「車軸 → 前端 0.72〜0.75 m」だけ。後端・最大幅・出っ張りは未測定。Nav2 は半径 0.45 m の円のまま（#157 OPEN）。前端 0.72〜0.75 m は半径 0.45 m の円の外に出る（台帳 `:51`「前の約 0.3 m がはみ出す」）[B]+[C]。

### 2.11 #141（/dev/imu の別名）— CLOSED、PR #182

- 起動前チェック（`check_device_aliases.sh`、preflight 0b）で検出するだけ。原因は未特定（クローズコメント）。センサ値には影響しない。

### 2.12 #148（D435 深度の校正）— OPEN。OCC / Tare は未実施。

### 2.13 #152 / #153（地図用 bag の補正と地図の作り直し）— CLOSED、PR #190

- campus-m8 は 2026-07-10 の bag（VLP16db・旧 TF・07-10 の IMU 取り付け）の点群を Hi-Res に**解析的に**直して作った。GLIM・DUFOMap は base_link→velodyne を使わない（campus-m8 README `:19-21`）。T_lidar_imu は audit（§2.5）。

### 2.14 #192（Patchwork++ sensor_height）— PR #201 マージ済み

- `src/whill_perception/config/patchworkpp.yaml:32` `sensor_height: 0.771`。0.79 → 0.771。出力が全フレームで変わらなかった記録（台帳 `:241`）[A]。

### 2.15 作られた物と変更の前後

| 物 | 作った時期 | 校正表 | velodyne TF | 根拠 |
| --- | --- | --- | --- | --- |
| `docs/maps/campus-half-v3`、`campus-demo`、`campus-outdoor-corrected`、`campus`（07-07〜07-10 の bag） | 2026-07 | VLP16db | 旧（pitch +8.99°）。GLIM は TF を使わないが、点群が ±15° の表 | 各 `metadata.yaml` の `acquired_at`、校正表は当時の launch（§2.1 の期間） |
| `docs/maps/campus-m8` | 2026-11-12 作成 | 07-10 の点群を Hi-Res に補正 | 地図は TF 非依存。NDT の再生比較は 11-13 の TF でやり直し済み（README `:20`） | metadata.yaml |
| `docs/bags-manifest.tsv` の全 54 本 | 2026-05-07〜2026-07-31 | VLP16db | 旧 TF（07-24 の bag の `/tf_static` は旧 TF と `docs/lidar-yaw-measurement.md` §3 が明記） | manifest の `started` と §2.1・§2.2 の期間 |
| `~/bags/sensor_check_20261028/` | 2026-10-28 | VLP16db | 旧 | 台帳 `:86`、manifest に無い |
| `~/bags/m8-static-check-20261113/` | 2026-11-13 11:42〜15:43 | B は VLP16db（台帳）。C は台帳に記載なし | 旧（bag の `/tf_static` で確認） | §1.2 |
| `~/bags/2026-11-16-static/` | 2026-11-16 | Hi-Res（results.json） | 現行（bag の `/tf_static`） | §1.2 |
| `~/bags/2026-11-27-m8-drive/` | 2026-11-27 | main 64f7a49 なので既定は Hi-Res | 現行（README の記載から。bag は今回開いていない） | README |

旧 TF・旧校正表の上で調整したパラメータ（台帳 `:152-155`、`:242-243`）：
- `src/whill_navigation/config/pointcloud_to_laserscan.yaml:48-49` `min_height: 0.05`、`max_height: 1.6`（「2026-07-15 の A/B で調整、ADR-0009」）。**新しい TF・Hi-Res で再検証した記録なし、担当 Issue なし** [D]。
- failsafe の Layer D（`/scan`）・Layer C（no_ground）のしきい値：同上 [D]。11-27 の走行では Layer D（`D:forward_blocked`）で 2 回止まり、反応した点は no_ground の高さ中央値 0.55 m・0.66 m、地図の OCC の縁から 0.05 m（`~/bags/2026-11-27-m8-drive/README.md`「止まった 2 か所の切り分け」）。センサ値の誤りを示す記録ではなく、停止域の形（±30° の扇形）の問題として README は書いている。
- 台帳 `:152` の「pitch が +8.995° → +0.97° に変わるので…2 m 先で約 0.28 m、10 m 先で約 1.4 m 変わる」は 11-12 の値（0.97°）で書かれたまま。今の値は +0.40°。

---

## 3. 食い違い（原文のまま、両方の場所）

### 3.1 文書の中の数値・状態の食い違い

| # | 一方 | 他方 | 区分 |
| --- | --- | --- | --- |
| 1 | 台帳 `:67` z の誤差「2 組の差 6 mm」 | 台帳 `:136`「2 組の差 z 3 mm・roll 0.04°・pitch 0.02°」、`static_tf_launch.py:67`「2 組の差は z 3 mm・0.05° 以内」、Issue #149「2 組の差は 3 mm / 0.05°」。表の値 0.773 と 0.770 の差は 3 mm | [D] |
| 2 | 台帳 `:68` roll「2 組の差 0.05°」 | 台帳 `:136`「roll 0.04°」（表の −0.03 と −0.07 の差は 0.04°） | [D] |
| 3 | 台帳 `:80`「x・y・z・roll・pitch は**実機受入済み**」 | 台帳 `:66` y「未評価」。屋外 D の y は深度との一致のみ（同じ y 0.36 から導出、§2.2） | [D] |
| 4 | 台帳 §2.2 `:163` 校正ファイル「**実測済み**。屋外静止確認（#149）は未了」 | 台帳 `:144` と results.json：屋外 D の dz_ring で確認済み（§2.1） | [D]（状態の書き忘れ） |
| 5 | 台帳 §3 `:177` カメラ「実機静止確認は未了（#149）」 | 台帳 `:148`・§5 `:226`：屋外 D で深度の結果あり。Issue #146 は「#148 の校正の後に判定」 | [D] |
| 6 | 台帳 §2.2 `:163`「Hi-Res で再計算すると 0.77 m」 | 計画書 `docs/plans/2026-10-28-avoidance-baseline.md:15`「Hi-Resで再計算すると0.777 m」、Issue #145 本文「z 0.777」 | [D]（0.777 の元のスクリプトは「未特定」、台帳 `:84`） |
| 7 | 台帳 `:152`「pitch が +8.995° → +0.97° に変わるので」 | 現行 +0.40°（`static_tf_launch.py:87`） | [D]（古い） |
| 8 | 台帳 §4.3 `:217` TF の鎖の T_lidar_imu「RPY (−4.33°, −9.08°, +0.36°)」「audit 値との差は…pitch 1.47°」 | 現行の TF から計算すると (−4.037°, −8.508°, +0.31°)、audit との pitch の差 0.898°（この監査の計算） | [D]（11-12 の TF での値のまま） |
| 9 | 台帳 §6 `:244` `m5r_pcd_to_occupancy.py` の台帳側「§2.1 z 0.777」 | 台帳 §2.1 は 0.771。スクリプト `scripts/m5r_pcd_to_occupancy.py:486` は `default=0.79`、help「default 0.79 per calibration-ledger's base_link->velodyne z」 | [D] |
| 10 | `src/whill_sensors_bringup/README.md:26` velodyne「2026-11-13 テープ固定後の実測（台帳 §2.1 C）」 | 同 README `:29`「LiDARは2026-10-28の実測（屋外静止確認 #149 は未了）」「カメラは…（#149 未了）」 | [D] |
| 11 | `odom_bringup_launch.py:20`「`ekf_node` fusing `/whill/odom` + `/imu/data_raw`」 | `ekf_odom.yaml:109` `imu0: /imu/data_rep145` | [D]（docstring が古い） |
| 12 | 台帳 §1 `:43` トレッドの出典「WHILL ROS ドライバ（`wheel_tread`、オドメトリの定数）」「公式コード」、§1 `:44` 半径「公式」 | `docs/m4r-whill-units.md:236-237` 半径・トレッド「(未実測)」。`whill_node.cpp:30-37`「Issue #30 adopted both as the working constants pending the M4R-3 real-machine 1 m straight-line measurement」。ROS 2 の fork では 0.1325 は fork 側で足した定数（`whill_node.cpp:30-41`）。台帳 `:56` の出典「ROS1 ドライバ `WHILL.h`（tread 0.496、wheel_radius 0.1325）」はこの監査では確かめていない（外部） | [D]（「公式」の値でも本機の実測ではない、という点が文書で書き分けられていない） |
| 13 | `whill_node.cpp:32` 半径の出典「WHILL Model CR2 URDF (whill_model_cr2.urdf…)」 | `docs/m4r-whill-units.md:235` 「0.265 m (URDF `whill_modelc.urdf`)」 | [D]（URDF の名前が違う。値は同じ） |
| 14 | 台帳 §0 `:20-25` の期待値 ay −0.699 m/s²（07-10 朝の live check） | `docs/reports/2026-10-30-imu-handoff-audit.md`：同じ 07-10 の bag の「IMU先頭から2〜27秒の2500件の平均加速度は `[1.335711, -0.525529, 9.794122]`」。差 ay 0.17 m/s²（許容 ±0.05 を超える）。ただし同レポートは「静止とみなした場合」と断っていて、静止の確認は無い | [D] |
| 15 | `scripts/m5r3_run_glim.sh:271-284` のコメント：「T_lidar_imu…physically-correct orientation…caused GLIM to diverge…Keep the noetic value」 | 台帳 §4.3 と campus-m8：audit 値を使い、発散せず地図を作った | [D]（コメントが古い。2026-07-08 時点の記述） |

### 3.2 file:line の参照が今のファイルとずれている所（台帳 §6 など）

| 台帳の記載 | 今の位置 | 区分 |
| --- | --- | --- |
| `imu_sign_corrector.py:68-70`（台帳 `:207`） | 反転は `:79-81`（#169 で行がずれた） | [D] |
| `m6r_bringup_launch.py:275`（台帳 `:165`） | `velodyne_calibration` の宣言は `:293`、既定の組み立ては `:254` | [D] |
| `failsafe_node.py:59`（車幅 0.6 m）、`:72`（半角 30°）（台帳 `:54`、`:240`） | 0.6 m はコメント `:72`、半角は `:85`。`:59` は `FITNESS_MAX = 1.0` | [D] |
| `m5r3_run_glim.sh:294-301`、`:335-342`（台帳 `:216`） | `:293-302`、`:333-342`（ほぼ一致） | [B] |
| `static_tf_launch.py:85`、`:100`、`:55`（台帳 `:83`、`:179`、`:196`） | 一致 | [B] |
| `sensors_launch.py:151`、`:153`（台帳 `:165`） | 一致 | [B] |
| `odom_bringup_launch.py:102`（台帳 `:165`） | 一致 | [B] |
| `patchworkpp.yaml:32`（台帳 `:241`） | 一致 | [B] |
| `nav2_params.yaml:231`、`:282` | 一致 | [B] |

### 3.3 yaw の 2 つの試行

- 台帳 `:70`、`docs/lidar-yaw-measurement.md` §3：「テープ固定**前**の 07-24 の走行 bag で 約 +1.0°（…+1.02°、…+1.16°、95 % 区間 +0.7〜+1.4°）」。
- 同 `:70`：「今の取り付けでは、11-16 のコーンの並びを『テープ ∥ 車体の x』と仮定すると約 −1.0° だが、テープの平行は未確認」。台帳 `:150`「テープは base_link 系で +0.68° ± 0.07°（LiDAR）・+0.41° ± 0.13°（深度）左に回って見える」。
- 台帳の結論：「2 つの試行は約 2° 食い違い、原因（テープの平行か取り付けの変化か）は未特定」。launch は −0.317°（noetic 流用）のまま [D]。

### 3.4 IMU の取り付けの説明の食い違い

- 台帳 `:268`（07-08）「90° 回して axis-aligned に付け直し、yaw 0・pitch −8°」「GLIM の T_lidar_imu を 90° 込みにすると発散したため」。
- `scripts/m5r3_run_glim.sh:266-270`：「the IMU is actually rotated relative to base_link (IMU +y points forward, +x points right = 90 deg yaw)」→ その後に付け直したことはこのコメントに書かれていない。現行の static TF は yaw 0。yaw 0 の根拠は「目視」のみ [C]。

---

## 4. リポジトリから決められないこと（必要なもの）

| 項目 | 何が無いか | 決めるのに要るもの |
| --- | --- | --- |
| LiDAR の yaw | 測定値なし。試行 2 つが約 2° 食い違う | `docs/lidar-yaw-measurement.md` §5 の手順（停止の組、往復）で今の取り付けを測る（#156） |
| LiDAR の個体の仰角 | 汎用の Hi-Res 表（offset 0）のみ。個体の工場校正ファイルが無い | Velodyne の個体の校正ファイル、または ring ごとの床・壁フィット |
| 機種 = Hi-Res の一次記録 | 銘板の写真・型番の記録が無い | 銘板の写真か型番・シリアルの記録 |
| 「水平器 + テープ」の再現性 | 未測定（台帳 `:82`） | 同じ場所で合わせ直し → 静止確認 C を複数回 |
| 2026-11 時点の IMU の取り付け角 | 07-10 以降の記録なし。07-10 の走行後で既に許容外 | `scratchpad/imu_live_check.py` の静止確認（台帳 §0）、または 11-13・11-16・11-27 の bag の `/imu/data_rep145` 静止区間の集計（重い bag の読み込みが要るので今回はしていない） |
| imu_link の yaw | 目視のみ | 回転させたときの gyro 軸の確認、または走行での推定 |
| gyro の符号が REP-103 か | #56 で「暫定判定」、rotate 試験の記録なし | その場旋回で `angular_velocity.z` の符号と `/whill/odom` の yaw rate を比べる |
| T_lidar_imu の真値 | audit（並進は noetic 流用、回転は VLP16db 時代の LiDAR −0.5° 仮定）と TF の鎖が約 0.9°（今の TF で）食い違う | 今の取り付けでの LiDAR-IMU 校正（GRIL-Calib など）と、それで作った地図の傾き・NDT の比較 |
| 車輪半径・トレッドの本機での値 | 「(未実測)」。10 m 直進の受入記録なし | 巻尺の直線で `/whill/odom` の距離、その場旋回で yaw |
| camera_link の回転 | 申告 0、「約 1° 上向きの疑い」 | 深度の床面フィット（11-16 の results.json の `floor.depth.plane` は slope_x −0.2°、slope_y 0.14° で、これは camera_link 0 の TF の下での値。解釈の記録なし） |
| 深度の距離 bias | 校正前の値のみ | #148 の OCC / Tare と撮り直し |
| footprint | 後端・最大幅・出っ張りが未測定 | 巻尺（#143） |
| コーンの半径補正の計算 | 計算式・コードが repo に無く、この監査の仮定では台帳の残差を再現しない | 補正に使った式（コーンの形、高さの基準） |
| `/scan` の高さ帯・Layer C/D の閾値 | 旧 TF・VLP16db で調整、再検証記録なし | 新 TF での再生比較か、低い物を置いた静止確認 |
| 07-10 以前の地図（campus など）での sensor 値の影響の大きさ | campus-m8 README の比較（傾き 1.81° → 0.72°）以外は無い | — |
| D435 と LiDAR のハブ共有の走行中の欠落 | 静止 12 秒のみ | 走行 bag での件数（11-27 はカメラなしなので不可） |

---

## 5. 実際に読んだもの

### ファイル（main 64f7a49）
- `src/whill_sensors_bringup/launch/static_tf_launch.py`（全文）、`sensors_launch.py`（全文）、`imu_launch.py`（全文）、`README.md`（全文）、`package.xml`（依存）、`whill_sensors_bringup/imu_sign_corrector.py:1-110`
- `src/whill_localization/launch/odom_bringup_launch.py:1-139`、`src/whill_localization/config/ekf_odom.yaml:60-140`
- `src/whill_safety/launch/m6r_bringup_launch.py`（grep）、`src/whill_safety/whill_safety/failsafe_node.py:55-85`（と grep）
- `src/whill_navigation/config/pointcloud_to_laserscan.yaml:40-52`、`nav2_params.yaml`（grep）、`src/whill_perception/config/patchworkpp.yaml`（grep）
- `docs/calibration-ledger.md`（全 280 行）
- `docs/calibration/2026-11-16-outdoor-static/notes.csv`、`results.json`（python で要約）
- `docs/m3-extrinsics-from-noetic.md`（1-60 と数値の grep）、`docs/imu-coordinate-audit.md`（目次、§2・§3・§4.4・§7.1・§8.1-8.3）、`docs/reports/2026-10-30-imu-handoff-audit.md`（全文）、`docs/reports/2026-11-02-glim-imu-root-cause.md`（全文）、`docs/lidar-yaw-measurement.md:1-40`、`docs/perception-blind-zone.md`（数値の grep）
- `docs/m4r-whill-units.md:210-260,380-398`（と grep）、`docs/m4r-bench-data/README.md`（冒頭と grep）、`docs/m3-sensors.md:112-124`（と grep）
- `docs/maps/campus-m8/README.md`（grep、`:66-100`）、`docs/maps/campus-m8/metadata.yaml:1-60`、`docs/maps/campus-m8/glim_config/config_sensors.json:1-40`、`docs/maps/*/metadata.yaml`（acquired_at など）
- `docs/plans/2026-10-28-avoidance-baseline.md`（grep）、`docs/bags-manifest.tsv`（列 1-3）、`docs/bags.md`（grep、校正の記述なし）、`docs/development.md`（grep）
- `scripts/m5r3_run_glim.sh:220-360`、`scripts/m5r_pcd_to_occupancy.py:480-490`、`scripts/m5r4_run_gril_calib.sh:145-152`、`scripts/check_object_placement.py`（grep）
- `whill_lab.repos`（前半）、`udev/99-whill-stack.rules`、`.gitignore`（grep）
- 上流（git 管理外）：`src/third_party/velodyne/velodyne_driver/config/VLP16-velodyne_driver_node-params.yaml`、`velodyne_pointcloud/config/VLP16-velodyne_transform_node-params.yaml`、`velodyne_laserscan/config/default-velodyne_laserscan_node-params.yaml`、`install/velodyne_pointcloud/share/velodyne_pointcloud/params/VLP16_hires_db.yaml`・`VLP16db.yaml`（grep）、`src/third_party/rt_usb_9axisimu_driver/src/*.cpp`・`include/*.hpp`（grep）、`src/third_party/ros2_whill/whill_driver/src/whill_node.cpp:20-60,130-270`、`model_cr2/whill.cpp:55-70`、`whill_bringup/config/params.yaml`、`src/third_party/realsense-ros/COLCON_IGNORE`
- `~/bags/2026-11-27-m8-drive/README.md`
- bag の `/tf_static`（前回の作業で読んだ。今回は bag を開いていない）：`~/bags/m8-static-check-20261113/{flat_1,fixed_2_place2,fixed_5_place3_rot180}`、`~/bags/2026-11-16-static/empty_start_140915`。bag の `metadata.yaml`（話題名）とディレクトリの一覧

### Issue（`gh issue view`、本文とコメント）
#37、#56、#141、#143、#144、#145、#146、#147、#148、#149、#150、#151、#152、#153。一覧の状態（#141〜#157）

### PR（`gh pr view`）
#182、#185、#186、#187、#188、#190、#197、#198、#200、#201（#191・#192 は PR ではない）。#173（検索）

### コミット・git
- `git log --follow` / 値の抽出：`static_tf_launch.py` の 02007d5、78f6658、b2cb7df、d9edac9、6c70d9b、80af31f、171b01d、aed1e4d、9cc4be2、c00e527、7165750、ac49849、9f6ad27
- `sensors_launch.py` の 02007d5、4bd9139、7165750。`imu_launch.py`、`ekf_odom.yaml`、`docs/calibration-ledger.md`、`failsafe_node.py` の log
- `m5r3_run_glim.sh` の `-S` 検索：b5bc6ce、494ea77、39bf794（494ea77 の差分）
- ブランチ：`git rev-list --count main..<branch>`（§冒頭の 10 本）、`git worktree list`、3 つの worktree の `git status`、`agent-aecfa03d91f4cdcc6` の `git diff --cached --stat` と台帳の差分、`git merge-base --is-ancestor`（3b9d250、11be463、c7417a7）
- third_party の `git describe`・`status`（velodyne、realsense-ros、rt_usb_9axisimu_driver、ros2_whill）

### この監査でした計算
- TF の鎖からの T_lidar_imu（11-12 の TF・現行の TF・旧 TF）と、audit・baseline の四元数の RPY（scipy、固定軸 xyz）
- results.json の dx に、仮定したコーンの半径を足す試算（台帳の残差は再現しなかった）
