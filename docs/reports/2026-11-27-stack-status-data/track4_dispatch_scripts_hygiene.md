# Track 4 監査: dispatch/Web・scripts・テスト・文書整合・リポジトリ衛生

- 対象: `/home/systemlab/whill_lab0_ros2`、main `64f7a49`、作業ツリー clean（2026-11-27 監査時点）
- 方法: 読み取りのみ（cat / grep / git / gh / bash -n / py_compile を一時領域へ）。ノード起動・トピック発行・ビルド・テスト実行はしていない（`run_tests.sh` の結果は主セッションの実行値を引用）。
- 証拠の区分: **[A]** 日時・条件・数値つきの実測記録 / **[T]** 自動テストあり（名前を記す） / **[B]** 現行コード・設定を読んだだけ / **[C]** 文書の記述だけ / **[D]** 証拠なし、または食い違い（両方を引用）
- 削除・残置の判断はしない。参照元・最終更新・依存を事実として並べる。

---

## A. whill_dispatch

### A-1. 中身 [B]

| ファイル | 内容 |
| --- | --- |
| `src/whill_dispatch/whill_dispatch/dispatch_node.py`（736 行） | 1 ノード。地点解決（waypoints.yaml）、FIFO キュー、NavigateToPose action client、状態配信、手動救出の String→Twist 変換 |
| `src/whill_dispatch/launch/dispatch_launch.py` | rosbridge_websocket(+rosapi) の include、dispatch_node、静的配信（`python3 -m http.server` または TLS 時 `scripts/m7_https_server.py`）、任意で `scripts/mock_navigate_to_pose.py` |
| `src/whill_dispatch/web/` | `index.html`(152 行)、`app.js`(588 行)、`style.css`(267 行)、`map.png`、`map_meta.json`、`vendor/roslib.min.js` + `LICENSE.roslibjs` |
| `src/whill_dispatch/.gitignore` | `web/dispatch.crt`（「TLS cert staged at launch for iPad trust — never commit」） |
| `CMakeLists.txt` | ament_cmake + ament_cmake_python。`launch` `web` を share へ、`dispatch_node.py` を `lib/whill_dispatch/dispatch_node` に RENAME で install（symlink ではなくコピー。install 側と src は監査時点で同一内容） |
| テスト | **なし**（`src/whill_dispatch/test/` 無し、CMakeLists に `ament_add_*` 無し） |

dispatch_node のインターフェース（`dispatch_node.py:185-215`）:
- pub: `/dispatch/state`（String, 既定 5 Hz）、`/dispatch/waypoints`（String, 既定 1 Hz）、**`/cmd_vel_teleop`（geometry_msgs/Twist）**
- sub: `/dispatch/submit`、`/dispatch/teleop`（String）、`/pcl_pose`、`/alignment_status`、`/whill/states/model_cr2`（whill_msgs が import できる時だけ。`dispatch_node.py:76-79` で try/except）
- service: `/dispatch/cancel`（std_srvs/Trigger）
- action client: `action_name` 既定 `/navigate_to_pose`
- 手動救出定数（`dispatch_node.py:101-117`）: `TELEOP_VX_MAX = 0.3`、`TELEOP_WZ_MAX = 0.6`、`TELEOP_WATCHDOG_S = 0.4`、`TELEOP_IDLE_OFF_S = 20.0`
- Web 側 `app.js:386-387`: ページが https なら `wss`、そうでなければ `ws`、ポート `9090` 固定。`app.js:450-451` で `/plan`（nav_msgs/Path）を購読。

launch 引数（主セッションの `--show-args` と同じ。コード `dispatch_launch.py` 末尾）: `site`=`campus-m8`、`use_mock`=`false`、`use_rosbridge`=`true`、`rosbridge_port`=`9090`、`http_port`=`8000`、`action_name`=`/navigate_to_pose`、`use_tls`=`false`、`tls_cert`=`~/.whill_dispatch_tls/dispatch.crt`、`tls_key`=`~/.whill_dispatch_tls/dispatch.key`。
- launch は `scripts/m7_https_server.py` と `scripts/mock_navigate_to_pose.py` を **リポジトリの scripts/ から** 解決する（install 先から 4 階層上、`_repo_root_from_pkg_share`）。両スクリプトは install されない。
- `use_tls:=true` で cert/key が無いと RuntimeError（`dispatch_launch.py` `_setup`）。
- TLS 時、`m7_https_server.py:43-48` は cert を配信ディレクトリへコピーする。symlink-install のため実体は `install/whill_dispatch/share/whill_dispatch/web/dispatch.crt`（監査時点で存在、2026-11-27 15:50 更新）。

### A-2. コード内の記述と実装の食い違い [D]

| 記述 | 実装 |
| --- | --- |
| `package.xml` description「dispatch adds no TF and no cmd_vel」 | `dispatch_node.py:192-193` `self._cmd_vel_teleop_pub = self.create_publisher(Twist, '/cmd_vel_teleop', 10)` |
| `dispatch_launch.py` docstring「dispatch adds no TF and no /cmd_vel*, so it does not conflict…」 | 同上（`/cmd_vel_teleop` を出す）。パッケージ README:11 は「手動救出では `/cmd_vel_teleop` に速度指令を出す」と正しく書いている |
| `m7_https_server.py:9-11`「cert ファイル (dispatch.crt) を配信ディレクトリに --cert-serve-dir で見せると」 | 引数 `--cert-serve-dir` は存在しない（argparse は `--port --dir --cert --key --bind` のみ）。コピーは常に行う |
| `package.xml`「rosbridge_server 2.0.7 is apt-installed」 | dpkg: `ros-humble-rosbridge-server 2.0.7-1jammy.20260605.145055`（一致） |
| ADR-0012 `Status: **proposed**`（実機配線確認後に accepted 化） | 2026-10-14 に実機ジョブ成功の記録あり（A-4）。ADR の状態は proposed のまま [C] |

### A-3. 他からの結合（何が dispatch を必要とするか）

パッケージ依存（package.xml）として whill_dispatch を要求する whill_* パッケージは **無い** [B]。トピック・ファイル・スクリプト経由の結合は次のとおり。

| 依存元 | 何に依存するか | 区分 |
| --- | --- | --- |
| `scripts/start_dispatch_demo.sh`（現用の統合起動。2026-11-27 の実機走行もこれで起動） | `ros2 pkg prefix whill_dispatch` が失敗すると `fail 'whill_dispatchが見つかりません'`。`$HOME/.whill_dispatch_tls/dispatch.crt`/`.key` が無いと `必要なファイルがありません`。`docs/maps/<site>/waypoints.yaml` 必須。ポート 8000/9090 を bind できないと停止。`check_map_sync.py --web-dir <install>/share/whill_dispatch/web` が失敗すると停止。起動列の最後に `dispatch_launch.py site:=… use_mock:=false use_tls:=true` を必ず起動（オプションで外せない）。既存ノード検査に `/dispatch_node` を含む | [B] |
| `scripts/demo_regression.sh` | `check_map_sync.py`（`src/whill_dispatch/web` を照合）、`start_dispatch_demo.sh --dry-run`、`dispatch_launch.py --show-args` | [B] |
| `scripts/check_map_sync.py:193` | 既定 `--web-dir` = `src/whill_dispatch/web` | [B] |
| `scripts/m7_make_web_map.py` | Web 背景（map.png / map_meta.json）を作る | [B] |
| `scripts/record_avoidance_bag.sh:113` | 録画トピックに `/plan /dispatch/state` | [B] |
| `scripts/map_route_check.py`、`scripts/nav2_plan_check.py` | パッケージではなく `waypoints.yaml`（配車地点）を入力にする | [B] |
| `scripts/import_upstream.sh:79`、README・development.md のビルド手順 | `--packages-up-to … whill_dispatch` | [B] |
| テスト | `scripts/tests/test_dispatch_demo_launcher.py`、`test_default_site_consistency.py`（dispatch_launch.py の既定 site を照合）、`test_map_sync.py`（Web 背景） | [T] |

安全・ナビゲーション側の結合 [B]:
- `src/whill_safety/config/twist_mux.yaml`: `teleop: topic /cmd_vel_teleop, timeout 0.5, priority 50`（safety 100、navigation 10）。コメントで dispatch_node を供給元と記す。twist_mux 自体はトピックを待つだけで dispatch が無くても動く。
- `failsafe_node.py`: `layer_d_rescue_enabled`（既定 `False`、`m6r_bringup_launch.py`/`safety_launch.py` の `failsafe_layer_d_rescue` 既定 `'false'`）が true の時だけ `/cmd_vel_teleop` を購読（`failsafe_node.py:352-355`）。`RESCUE_MAX_WZ = 0.6` のコメントに「dispatch_node's TELEOP_WZ_MAX」。テスト `test_failsafe_rescue.py::test_shipped_twist_mux_matches_the_assumptions` が twist_mux.yaml の teleop timeout を固定 [T]。
- `/cmd_vel_teleop` の唯一の実装上の供給元は dispatch_node（リポジトリ内 grep）。ジョイスティック等の他の teleop 供給元は無い [B]。
- whill_navigation・whill_localization・whill_perception・whill_sensors_bringup は dispatch をコメントで言及するのみ（`odom_bringup_launch.py:124`、`imu_sign_corrector.py:101`）[B]。
- `m6r_preflight.sh` は dispatch に依存しない（check 0〜5 に dispatch/8000/9090 の項目なし）[B]。

### A-4. 実測記録: `docs/reports/2026-10-14-dispatch-demo.md` [A]（一部 [C]）

- 条件（同ファイル:5-8）: 実行 PC `systemlab-Alienware-x15-R2`、`scripts/start_dispatch_demo.sh`「既定: campus、v2、normal、domain 0、TLS」。**当時の既定は site `campus` / variant `v2`**（現在の既定は campus-m8 / candidate なので、この記録は現行既定の地図ではない）。
- 根拠: ユーザー報告「起動完了し、7号館から工農研究所までの配車成功」[C] と、ログ `log/dispatch-demo-20261014-140220-LhNDRE/` の `Managed nodes are active`、`Reached the goal!`、`Goal succeeded`、`job 1 SUCCEEDED` [A]。ログは監査時点で `log/` に現存（Git 管理外）。
- 表（:20-23）: 13:58:42 起動 → `kounoken` 宛て、38 秒後 RPP `collision ahead` ×2、14:00:56 `/dispatch/cancel` で `CANCELED`、未到着。14:02:20 起動 → `bldg7`(-38.42,-0.47) 宛て 14:03:19 送信、14:04:43 `SUCCEEDED`。
- 食い違い（#170、:16-18）: 報告の向き「7号館→工農研」と成功ジョブの向き（工農研付近→`bldg7`）が逆 [D]（文書自身が記載）。
- 明示的に未確認（:32-34）: 終了後の全プロセス消失・直後の再起動、preflight 全項目、手動中の Layer D 停止、停止距離、他経路の受入。「この記録は1回の配車成功」。

### A-5. 実測記録: `docs/m7-bench-data/`

| 記録 | 内容 | 区分 |
| --- | --- | --- |
| `2026-07-19-mock/README.md` + PNG 2 枚 | 実機なし。`dispatch_launch.py use_mock:=true` で AC1〜7 を PASS。地点は当時の `kounoken/gate/bldg7/library`（現在の地点名 `kounoken/bldg7/seimon/bldg3` と違う）。:99-100「waypoints.yaml の x/y/yaw はプレースホルダ」。mock feedback は線形の理想化（:94-97） | [A]（mock） |
| `2026-07-22-live-campus/seimon-stall/README.md` | 実機。bag `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/rosbag2_2026_07_22-18_01_15`（875.9 MiB, 132.5 s、監査時点で存在）。`map_variant:=v2 speed:=normal`。停滞の原因を塗り境界の inflation と Spin リカバリと分析し、塗り拡幅・rotate 0.8→0.4・カスタム BT（Spin/BackUp 除去）・`rotate_to_heading_min_angle 0.785→1.57`・`movement_time_allowance 10→15` を記す。再走 bag 2 は雨で中断。「実機確認手順 (次回)」の結果記録は無い | [A]（旧 campus 地図） |

2026-07-24 の走行 bag（`~/bags/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26`、5.3G、存在）は回帰チェックの基準 bag。対応する m7-bench-data の README は repo に無い [D: 記録なし]。

### A-6. dispatch を使わない現行の実機記録

- `~/bags/2026-11-27-m8-drive/README.md`（Git 管理外、リポジトリ外）[A]: campus-m8 / candidate（154w、`candidate_final.pgm` sha256 `fca2abb4…`）、main `64f7a49`、`start_dispatch_demo.sh --speed slow`、「走行の指示: RViz の Nav2 Goal（配車 Web の job は 0 件）」、`cmd_vel_teleop は全区間 0 件`。goal 成功 3 件（うち 1 件は bag 無し）、停止 2 件（drive03 `Failed to make progress ×5`・`D:forward_blocked`、drive04 `collision ahead ×5`）。
- 監査時点の `log/dispatch-demo-20261127-154932-08Z01t/dispatch.log` は `dispatch_node ready: 4 waypoint(s) from …/campus-m8/waypoints.yaml` のみで job 行なし [A]。
- `log/dispatch-demo-20261113-113616-3PwMqG/`、`…-20261113-153041-KEE1HM/` にも dispatch.log が無い（bringup/nav2/rviz のみ）。どの段階で止めた起動かは記録なし [D]。
- この 2026-11-27 走行は repo 内の文書には未反映（runbook:7 の記述と食い違う。D 節参照）。

---

## B. scripts/ 全ファイル

列の意味: **doc** = `docs/scripts.md` に名前がある（`dir` は `imu_investigation/`・`regression/` の行でディレクトリとしてのみ記載）/ **最終 commit** = `git log -1 --format=%ad --date=short -- <file>` / **テスト** = `scripts/tests` または `src/*/test` でその名前を参照するテストファイル（「対象のテスト」とは限らず、使っているだけのものも含む）/ **参照元** = basename の stem を `git grep -F` し、自身と `docs/scripts.md` を除外した件数と、bench-data・experiments を除く先頭最大 8 件 / **位置づけ** = スクリプト自身の冒頭コメントに書かれた状態（DEPRECATED・SUPERSEDED・マイルストーン名など）。区分はすべて [B]（コードと git を読んだだけ）。

注意: stem の部分一致を使っているため、`check_static.py` は `start_dispatch_demo.sh` 内の関数名 `check_static_pcd` に、`regression/compare.py`（stem `compare`）・`baseline.json`（stem `baseline`）は一般語に誤って一致し、件数が過大。

| ファイル | 用途（冒頭コメント） | doc | 最終 commit | テスト | 参照元（件数: 例） | 位置づけ（自己記述） |
| --- | --- | --- | --- | --- | --- | --- |
| audit_free_leaks.py | v2 final pgm の free 漏れ 2 指標 | yes | 2026-07-18 | なし | 2: reports/2026-07-18-pipeline-v2-report.md, map_route_check.py | v2 地図（2026-07） |
| check_dds_nic.py | DDS XML の NIC と LiDAR の NIC の一致（#140） | yes | 2026-11-02 | test_preflight.py | 4: m6r_preflight.sh, record_avoidance_bag.sh, test_preflight.py, plans/avoidance-baseline | 現用（preflight check 0） |
| check_device_aliases.sh | /dev/whill /dev/imu の別名と VID:PID（#141） | yes | 2026-11-12 | test_preflight.py | 4: runbook, m6r_preflight.sh, start_dispatch_demo.sh, test_preflight.py | 現用（preflight 0b） |
| check_failsafe_inputs.sh | failsafe 入力の受信を同じ QoS で（#168） | yes | 2026-11-02 | なし（preflight テストからは偽物で呼ばれる可能性あり、名前参照なし） | 2: m6r_preflight.sh, record_avoidance_bag.sh | 現用（preflight check 4） |
| check_map_sync.py | Web 背景と Nav2 地図 YAML の同期（#171） | yes | 2026-11-13 | test_map_sync.py, test_default_site_consistency.py | 11: architecture-overview, regression-check, runbook, demo_regression.sh, start_dispatch_demo.sh, m7_make_web_map.py ほか | 現用（dispatch Web 用） |
| check_object_placement.py | 置いた物と床の base_link 位置（#149, M8-4） | yes | 2026-11-13 | test_check_object_placement.py | 5: calibration-ledger, lidar-yaw-measurement, plans/avoidance-baseline, record_avoidance_bag.sh | 現用（M8） |
| check_static.py | IMU bag の静止区間の確認 | yes | 2026-05-07 | なし | 5（誤一致含む）: calibration-ledger, m5r_run_dufomap.sh | 初期（M3 期） |
| clean_isolated_occupancy.py | 孤立 occupied セル除去（Task #13, 2026-07-16） | yes | 2026-07-15 | なし | 4: maps/campus/README, pcd_to_occupancy_v2.py, nav_launch.py（`map_variant:=cleaned` の説明） | 旧 campus |
| compose_occupancy.py | v2 Stage 2 合成 | yes | 2026-07-19 | test_nav2_plan_check.py（使用） | 12: campus-m8 README, v2-paint-guide, gen_auto_free_mask.py, map_* ほか | 現用（v2 パイプライン） |
| configure_proxy.sh | 宇都宮大 proxy 用 apt/SSH 設定 | yes | 2026-05-06 | なし | 4: m5r-cuda-setup, install_cuda.sh, install_ros2_humble.sh | セットアップ |
| cpu_governor.py | #136 の performance governor サービスの install/check/remove | **no** | 2026-11-02 | なし | 1: reports/2026-11-02-cpu-governor.md | 現用ホスト設定（記載漏れ） |
| demo_regression.sh | マージ前のデモ回帰チェック（#142） | yes | 2026-11-12 | なし | 5: PR テンプレート, bags.md, regression-check, baseline.json, whill_dispatch README | 現用 |
| gen_auto_free_mask.py | roadway 内の ≤3 セル OCC の free_mask 自動生成 | yes | 2026-07-18 | test_widen_hand_roadway.py（使用） | 7: campus-m8 README, compose_occupancy.py, map_widen_hand_roadway.py ほか | 現用（v2） |
| grant_serial_access.sh | dialout 追加 | yes | 2026-05-06 | なし | 3: m2-whill-core, check_device_aliases.sh, install_udev_rules.sh | セットアップ |
| import_upstream.sh | whill_lab.repos の vcs import・Patchwork++ COLCON_IGNORE・rosdep | yes | 2026-11-12 | なし | 3: development.md, m2-whill-core, m3-sensors | 現用セットアップ |
| imu_investigation/audit_imu_investigation.py | #147 引き継ぎ検査を NPZ から再現 | dir | 2026-11-02 | なし | 1: reports/2026-10-30-imu-handoff-audit.md | #147 調査 |
| imu_investigation/extract_bag_stats.py | bag から IMU と LiDAR メタを抽出 | dir | 2026-11-02 | なし | **0** | #147 調査 |
| imu_investigation/glim_control.sh | #147 GLIM 対照実験 | dir | 2026-11-12 | なし | 3: campus-m8 README, campus-m8 metadata.yaml, reports/glim-imu-root-cause | #147 調査（campus-m8 地図の出典に記載） |
| imu_investigation/imu_vs_lidar.py | IMU と LiDAR odom の比較 | dir | 2026-11-02 | なし | 6: reports/2026-10-30-imu-audit-data/* | #147 調査 |
| imu_investigation/lidar_icp_odom.py | IMU 無しの LiDAR ICP odom | dir | 2026-11-02 | なし | 2: imu_vs_lidar.py, static_gravity_vs_floor.py | #147 調査 |
| imu_investigation/static_gravity_vs_floor.py | 静止窓で重力と床法線の比較 | dir | 2026-11-02 | なし | **0** | #147 調査 |
| imu_investigation/summarize_glim_runs.py | GLIM run の要約 | dir | 2026-11-02 | なし | 2: campus-m8 README, reports/glim-imu-root-cause | #147 調査 |
| init_v2_sidecars.py | v2 sidecar PNG の初期化 | yes | 2026-07-18 | なし | 2: v2-paint-guide, reports/pipeline-v2 | v2 |
| install_cuda.sh | CUDA 12.4 + cuDNN 8（GLIM 用） | yes | 2026-06-13 | なし | 5: m5r-cuda-setup, m5r-glim-setup, install_glim.sh, m5r3_run_glim.sh | セットアップ（M5-R） |
| install_glim.sh | GLIM ソース導入（`GLIM_REF="master"`, `GLIM_ROS2_REF="master"`） | yes | 2026-10-30 | なし | 6: ADR-0003, development.md, m5r-glim-setup ほか | 現用（オフライン地図作成） |
| install_gril_calib.sh | GRIL-Calib 導入 | yes | 2026-10-30 | なし | 2: m5r-imu-diagnostic, repository-refresh | M5-R 校正 |
| install_nvidia_suspend_fix.sh | NVIDIA suspend 対策（#76） | yes | 2026-07-08 | なし | 1: host-setup-nvidia-suspend.md | ホスト設定 |
| install_ros2_humble.sh | ROS 2 Humble Desktop 導入 | yes | 2026-05-06 | なし | 3: development.md, import_upstream.sh | セットアップ |
| install_udev_rules.sh | udev 規則の導入 | yes | 2026-05-07 | なし | 4: development.md, m3-sensors, check_device_aliases.sh, udev rules | セットアップ |
| install_velodyne_network.sh | 有線 NIC の netplan | yes | 2026-05-07 | なし | 3: development.md, m3-sensors, network template | セットアップ |
| lidar_floor_fit.py | 床面から z・roll・pitch | yes | 2026-11-12 | test_lidar_yaw_from_drive.py（参照） | 5: calibration-ledger, lidar-yaw-measurement, check_object_placement.py ほか | 現用（M8-1 校正） |
| lidar_yaw_from_drive.py | 走行 NDT 姿勢から yaw（#149） | yes | 2026-11-16 | test_lidar_yaw_from_drive.py | 4: calibration-ledger, lidar-yaw-measurement, ndt_replay_compare.py | 現用（M8-1） |
| m4r3_ekf_bench.py | bag で EKF 誤差（M4R-3, #37） | yes | 2026-06-20 | なし | 2: m5r3_loop_error.py | M4-R |
| m5r3_b1_numeric.py | B1 ループ閉合誤差 | yes | 2026-07-10 | なし | 6: ADR-0003, m5r3-comparison-protocol, maps/campus/README | M5-R3 |
| m5r3_export_merged_ply.py | 結合点群出力 | yes | 2026-07-10 | なし | 3: m5r3-comparison-protocol, maps/campus/README | M5-R3 |
| m5r3_fix_imu_bag.py | IMU 加速度符号反転 bag | yes | 2026-11-02 | なし | 5: ADR-0003, m5r-pipeline, m5r3_run_glim.sh, imu_sign_corrector.py | **「DEPRECATED (Issue #56)」「retained only for historical bags」**（:4,:11） |
| m5r3_loop_error.py | 軌跡のループ誤差（#48） | yes | 2026-10-30 | なし | 26: ADR-0003, campus-m8 README ほか | M5-R3。冒頭が存在しない `scripts/m5r3_run_fastlio_sam.sh` を「使う側」として挙げる [D] |
| m5r3_run_glim.sh | GLIM オフライン実行 | yes | 2026-11-12 | なし | 40 | 現用（地図作成） |
| m5r4_run_gril_calib.sh | GRIL-Calib 実行（#64） | yes | 2026-10-30 | なし | 5: calibration-ledger ほか | M5-R4 |
| m5r_dufomap_diff.py | DUFOMap 前後差分（#49） | yes | 2026-06-21 | なし | 3: ADR-0004, m5r-pipeline | M5R-4 |
| m5r_glim_to_pcd.py | GLIM keyframe → PCD | yes | 2026-06-21 | なし | 6: ADR-0004, m5r-pipeline, m5r_run_dufomap.sh ほか | 現用（地図作成の一段） |
| m5r_pcd_to_occupancy.py | DUFOMap PCD → 占有格子（M5R-6） | yes | 2026-07-10 | なし | 15: maps/campus-demo 等の yaml/pgm | v2 冒頭で「older pipeline」と記載（`pcd_to_occupancy_v2.py` 冒頭） |
| m5r_run_dufomap.sh | DUFOMap 一式（#49） | yes | 2026-11-16 | なし | 9: campus-m8 README/metadata ほか | 現用（地図作成） |
| m5r_run_dufomap_core.py | DUFOMap Python API ラッパ | yes | 2026-11-16 | なし | 3 | 現用 |
| m6r_preflight.sh | 走行前ゲート check 0〜5 | yes | 2026-11-12 | test_preflight.py | 8: runbook, ADR-0007, start_dispatch_demo.sh ほか | 現用 |
| m6r_record_calib_bag.sh | 屋内校正 bag（#64） | yes | 2026-11-02 | なし | 4 | M6-R |
| m6r_smoke_test.sh | lidar_localization の bag 再生スモーク（M6R-1） | yes | 2026-11-02 | なし | 8: ADR-0006, whill_lab.repos（SHA 更新時に再実行と記載）, m6r_bringup_launch.py | M6-R（whill_lab.repos が更新手順として参照） |
| m7_https_server.py | dispatch Web の HTTPS 配信 | yes | 2026-07-20 | なし | 2: runbook, dispatch_launch.py | dispatch（M7） |
| m7_make_tls_cert.sh | 自己署名証明書 | yes | 2026-07-20 | なし | 3: runbook, m7_https_server.py, dispatch_launch.py | dispatch（M7） |
| m7_make_web_map.py | 占有 pgm → Web 背景 + meta | yes | 2026-11-13 | test_map_sync.py | 7: runbook, maps README, check_map_sync.py, whill_dispatch README, app.js | dispatch（M7） |
| m7_ws_smoke.py | rosbridge 経由の投入・取消 E2E | yes | 2026-07-21 | なし | 3: whill_dispatch README, dispatch_node.py | dispatch（M7）。README:50 で「読み取り専用の接続確認ではない」 |
| map_align.py | 2 地図間の変換（#154,#155） | yes | 2026-11-12 | なし | 5: campus-m8 README, waypoints.yaml, map_transfer_masks.py, ndt_replay_compare.py | M8 |
| map_georef_gsi.py | GSI 位置合わせ（#205） | yes | 2026-11-19 | test_georef_gsi.py | 8 | M8 実験 |
| map_ground_raster_from_bag.py | 密な地面ラスタ（#202） | yes | 2026-11-16 | test_ground_raster_from_bag.py | 4 | M8 実験「既定の地図は変えない」 |
| map_gsi_edge_snap.py | GSI 縁の補正（#202 202d） | yes | 2026-11-19 | test_gsi_edge_snap.py | 3 | M8 実験 |
| map_roadway_from_curbs.py | 縁石から roadway 試作（#202） | yes | 2026-11-19 | test_roadway_curbs.py, test_ground_raster_from_bag.py | 7 | 「試作」 |
| map_roadway_vs_gsi.py | roadway と GSI の比較（#202c） | yes | 2026-11-17 | **なし** | 2: campus-m8 README, map_gsi_edge_snap.py | M8 実験 |
| map_route_check.py | 配車地点が FREE で連結か | yes | 2026-11-12 | test_nav2_plan_check.py（参照） | 5 | M8（入力は配車地点 waypoints.yaml） |
| map_tilt_metrics.py | 地図の傾き（#153） | yes | 2026-11-12 | なし | 1: campus-m8 README | M8 |
| map_transfer_masks.py | 旧地図の手塗りを新格子へ（#154） | yes | 2026-11-12 | なし | 1: campus-m8 README | M8 |
| map_widen_hand_roadway.py | 154w 拡幅候補 | yes | 2026-11-19 | test_widen_hand_roadway.py | 2 | M8（現既定地図の生成元） |
| mock_navigate_to_pose.py | 模擬 NavigateToPose | yes | 2026-07-19 | なし | 4: plans/m7-dispatch, runbook, dispatch_launch.py | dispatch（M7）開発用 |
| move_bags.py | docs の bag を ~/bags へ（#139） | yes | 2026-11-02 | test_move_bags.py | 2: bags.md | 現用（一度きりの移行用） |
| nav2_plan_check.py | 配車地点の全組で planner（#154） | yes | 2026-11-19 | test_nav2_plan_check.py | 2 | M8 |
| pcd_to_occupancy_grid.py | 「FAST-LIO-saved PCD」→ 占有格子（M5-b） | yes | 2026-10-30 | なし | 10: legacy-findings, maps/_template, .claude/agents/ros2-implementer.md ほか | 旧 M5-b（FAST-LIO 期） |
| pcd_to_occupancy_v2.py | v2 Stage 1 | yes | 2026-07-18 | test_step_accessible.py | 8 | 現用（v2） |
| pcd_to_traversability.py | traversability 格子 | yes | 2026-07-18 | なし | 1: .gitignore | **「SUPERSEDED by v2 (PR #91)」**（:4） |
| pipeline_v2_verify.py | v2 の M1〜M3 検証 | yes | 2026-07-18 | なし | 1: reports/pipeline-v2 | v2（2026-07） |
| record_avoidance_bag.sh | 屋外試験 bag を mcap 記録 | yes | 2026-11-16 | test_record_avoidance_bag.py | 5 | 現用（2026-11-27 走行で使用、~/bags README 記載） |
| regression/baseline.json | 回帰閾値 | dir | 2026-11-13 | test_regression_compare.py | （誤一致多数） | 現用だが `site: campus`・旧 TF で測定（C 節） |
| regression/compare.py | 結果と閾値の比較 | dir（名前は無い） | 2026-11-02 | test_regression_compare.py | （誤一致多数） | 現用 |
| regression/ndt_replay_compare.py | 地図・TF を選んで NDT 再生（#155） | yes | 2026-11-16 | なし | 12 | M8 |
| regression/patchwork_height_compare.py | sensor_height 違いの比較（#192） | yes | 2026-11-16 | なし | 3 | M8 |
| regression/replay_check.py | 基準 bag で failsafe・localizer 再生（#142） | dir（名前は無い） | 2026-11-13 | test_regression_compare.py（compare のみ） | 6 | 現用 |
| regression/replay_localizer_launch.py | localizer と静的 TF だけの launch | dir（名前は無い） | 2026-11-13 | なし | 9 | 現用 |
| run_tests.sh | 自作パッケージ + scripts/tests | yes | 2026-11-02 | なし | 5 | 現用 |
| setup_workspace.sh | **`~/ros2_ws`** に ws 作成し **`.bashrc` へ追記**（:2, :8, :29-33） | yes | 2026-05-06 | なし | 1: repository-refresh-2026-10-30 | 初期。対象 ws がこのリポジトリではない |
| start_dispatch_demo.sh | 統合起動 | yes | 2026-11-13 | test_dispatch_demo_launcher.py, test_default_site_consistency.py, test_map_sync.py | 16 | 現用（名前は dispatch だが bringup 全体の入口） |
| velodyne_compare_clouds.py | 2 bag の点群を点ごと比較（#152） | yes | 2026-11-12 | なし | 2 | M8 |
| velodyne_reproject_bag.py | 誤校正 bag の再投影 | yes | 2026-11-02 | test_check_object_placement.py（ROS があれば一致確認） | 13 | M8 |
| velodyne_reproject_verify.sh | 上の検証（#152） | yes | 2026-11-12 | なし | 2 | M8 |

scripts/tests の各ファイル（doc 列）: `docs/scripts.md` に載るのは `test_step_accessible`・`test_check_object_placement`・`test_lidar_yaw_from_drive`・`test_record_avoidance_bag`・`test_dispatch_demo_launcher`・`test_default_site_consistency` の 6 本。**載っていない 10 本**: `test_georef_gsi`（2026-11-16）、`test_ground_raster_from_bag`（11-16）、`test_gsi_edge_snap`（11-17）、`test_map_sync`（11-13）、`test_move_bags`（11-02）、`test_nav2_plan_check`（11-19）、`test_preflight`（11-12）、`test_regression_compare`（11-02）、`test_roadway_curbs`（11-16）、`test_widen_hand_roadway`（11-19）。

### B-1. scripts.md との差分 [B]

- scripts.md に名前があって実在しないファイル: **なし**。
- 実在して scripts.md に無い: `cpu_governor.py`、上の tests 10 本、`regression/` 配下の `baseline.json`・`compare.py`・`replay_check.py`・`replay_localizer_launch.py`（ディレクトリ行のみ）、`imu_investigation/` 配下 7 本（ディレクトリ行のみ）。
- `scripts.md:93` `tests/test_step_accessible.py` の説明「段差処理のテスト」。実体は `pcd_to_occupancy_v2.py` の `compute_step_accessible_side`（Rule 3）の合成テスト 4 件。
- 存在しないスクリプトへの参照（履歴文書・bench-data を除く）: `scripts/m5r3_run_fastlio_sam.sh`（`docs/decisions/0003-mapping-slam-choice.md`、`scripts/m5r3_loop_error.py` 冒頭）、`scripts/m5r3_mapping_velodyne_for_fastlio_sam.launch.py`（ADR-0003）。他に `clone_fastlio_sam_for_eval.sh`、`m5r_erasor_diff.py`、`m5r_pcd_to_occupancy.sh`、`m5r_run_erasor.sh`、`m6r4_cmd_vel_stop_latency.py`、`m6r4_costmap_stats.py`、`m6r_tf_jump_check.py`、`nav_to_waypoint.py` は bench-data・plans・legacy の中だけで参照。

### B-2. その他の機械的事実 [B]

- `bash -n`: 追跡中の全 `.sh` が通過。`py_compile`（出力は一時領域）: `scripts/`・`src/whill_*`・`scratchpad/*.py` の全 .py が通過。shellcheck はこのホストに無い。
- 実行ビット: 全 `.sh` は 100755。shebang があるのに 100644 の .py: `cpu_governor.py`、`imu_investigation/audit_imu_investigation.py`、`lidar_yaw_from_drive.py`、`m5r3_b1_numeric.py`、`m5r3_export_merged_ply.py`、`m7_make_web_map.py`、`m7_ws_smoke.py`、`mock_navigate_to_pose.py`、tests 6 本（launch は `python3 <path>` で呼ぶので動作には影響しない）。
- `scripts/__pycache__/`: **Git 管理外**（`.gitignore` の `__pycache__/`、`git ls-files scripts/__pycache__` は 0 件）。中に、もう存在しない `m5r3_mapping_velodyne_for_fastlio_sam.launch.cpython-310.pyc` が残る。`scripts/regression/__pycache__`、`scripts/tests/__pycache__` も同様に ignored。
- `src/whill_sensors_bringup/scripts/` は `__pycache__/` だけを含むディレクトリ（2026-06-11）で、中身は ignored。追跡ファイル 0。

---

## C. 自動テストと回帰チェック

### C-1. テスト一覧（件数は `def test` の数。parametrize の展開前）[T]

主セッションの実行結果（2026-11-27, main 64f7a49）: `scripts/run_tests.sh` 通過、colcon 54 tests 0 failures、scripts/tests 217 passed [A]。

colcon（`ament_add_pytest_test`、`run_tests.sh` が `.test_ws/` でビルドして実行）:

| テスト | 件数 | 範囲 |
| --- | --- | --- |
| `src/whill_safety/test/test_failsafe_never_arrived.py` | 12 | #159 未着入力（`startup_grace_sec`）。既定 0 で止めない、grace 後に止める、sim clock、不正値拒否 |
| `src/whill_safety/test/test_failsafe_rescue.py` | 17 | #161 Layer D 中の救援中継（既定 off、後退・回転のみ中継、前進は即ゼロ、古い指令でゼロ、他 Layer でゼロ、twist_mux.yaml の teleop timeout 0.5 との整合） |
| `src/whill_sensors_bringup/test/test_imu_sign_corrector_shutdown.py` | 2 | #169 終了処理 |

**テストが無い自作パッケージ**: whill_dispatch、whill_navigation、whill_localization、whill_perception（CMakeLists に `ament_add_*` 無し）[B]。failsafe の Layer A〜D の判定そのもの（fitness 閾値、Layer D 扇形 ±30°・1.0〜2.0 m など）の単体テストは無く、検出は bag 再生の回帰チェック（下記）だけ [B]。

scripts/tests（pytest、ROS ノードは起動しない）:

| テスト | 件数 | 範囲（docstring） |
| --- | --- | --- |
| test_check_object_placement.py | 37 | check_object_placement.py の計算、TF 既定値と launch・台帳の一致 |
| test_default_site_consistency.py | 3 | 既定 site/variant が 3 launch・起動スクリプト・check_map_sync.py で一致 |
| test_dispatch_demo_launcher.py | 20 | start_dispatch_demo.sh の引数、static.pcd の有無と sha256、子プロセス失敗、初期位置の順序、INT/TERM/HUP/PIPE での後始末（無害なプロセスで） |
| test_georef_gsi.py | 9 | map_georef_gsi の変換・復号（通信なし） |
| test_ground_raster_from_bag.py | 14 | map_ground_raster_from_bag の合成データ |
| test_gsi_edge_snap.py | 20 | map_gsi_edge_snap の合成データ |
| test_lidar_yaw_from_drive.py | 10 | yaw 推定の符号・蛇行・後退・停止 |
| test_map_sync.py | 17 | check_map_sync.py（合成地図と現用 campus-m8・fallback campus） |
| test_move_bags.py | 10 | bag 移動の安全側 |
| test_nav2_plan_check.py | 7 | nav2_plan_check の地図・経路計算 |
| test_preflight.py | 22 | m6r_preflight.sh・check_dds_nic.py・check_device_aliases.sh を偽の ros2/ip/udevadm で |
| test_record_avoidance_bag.py | 10 | record_avoidance_bag.sh の事前・事後確認 |
| test_regression_compare.py | 4 | 閾値が正常を通し、壊した場合（TF x+10 cm、扇形 ±60°）を落とす |
| test_roadway_curbs.py | 4 | detect_curbs |
| test_step_accessible.py | 4 | compute_step_accessible_side（段差 Rule 3） |
| test_widen_hand_roadway.py | 24 | map_widen_hand_roadway |

CI: `.github/` には ISSUE_TEMPLATE と `pull_request_template.md` のみ。workflow は無い [B]。`docs/development.md:45`「CI は無い」と一致。

### C-2. 「デモ回帰チェック」が実際に確かめること [B]

`scripts/demo_regression.sh`:
- 静的（既定）: ① `scripts/run_tests.sh`、② `check_map_sync.py --maps-root <branch>/docs/maps`、③ `start_dispatch_demo.sh --dry-run`（コマンド表示だけ）、④ `m6r_bringup_launch.py`・`nav_launch.py`・`dispatch_launch.py` の `--show-args`。`ROS_DOMAIN_ID=142 ROS_LOCALHOST_ONLY=1`、`CYCLONEDDS_URI` を unset。
- 再生（`--level replay`）: `scripts/regression/replay_check.py` で基準 bag `${WHILL_BAGS_ROOT:-$HOME/bags}/m7-bench-data/2026-07-24-live-campus/rosbag2_2026_07_24-19_05_26`（監査時点で存在、5.3G）を failsafe と localizer に流し `baseline.json` と比較。出力 `.test_ws/regression/<日時>/`。
- 実機は不要、走行指令は出さない（`docs/regression-check.md:13`）。ロボットが要るのは段階 3「runbook のとおりデモのルートを1周」だけ（同:9）。
- 確かめないこと（同:33-40）: Nav2 の計画・制御、Layer C（bag に `/velodyne_points_no_ground` が無い）、入力途絶検出、速度指令と停止距離、NDT fitness、地図と入力点群の正しさ。

`baseline.json` の状態 [B]: `"site": "campus"`、`"measured_on": "2026-11-02, main 81f3ec7"`、旧 TF（base_link→velodyne 0.484, 0.382, 0.794 m、roll −2.03°、pitch +9.00°）。`site_note` 自身が「demo_regression.sh --level replay は、#156 の走行の bag で campus-m8 の基準を取るまで main で失敗する」「それまでは --level replay --only failsafe を使う」と書く（regression-check.md:44-53 も同じ）。閾値: failsafe `stops_while_driving max 0`・`engaged_s 28.0–38.0`・`engagements 34–48`、localizer `xy_err_p50_m max 0.02`・`xy_err_p95_m max 0.11` ほか。

### C-3. 実行記録

| 日付 | 内容 | 区分 |
| --- | --- | --- |
| 2026-11-02 | PR #176（merged 2026-11-02T08:19:03Z）本文: 正常「localizer 4回、failsafe 3回」で xy 中央値 0.000 m、p95 0.053〜0.080 m、走行中の停止 0 回、33 秒 → 合格。TF x+10 cm で p50 0.099 m・p95 0.132 m → 不合格。扇形 ±60° で走行中停止 9 回・43.3 秒 → 不合格 | [A]（PR 本文） |
| 2026-11-02 | PR #177 本文「`scripts/demo_regression.sh`：合格。`scripts/tests` は 26 件」 | [A]（静的段階） |
| 2026-11-09 | PR #178 本文「静的チェック OK（scripts のテスト 36 件）」。replay は bag 移動後に通すと記載、その結果の記録は PR 本文に無い | [A] / [D] |
| 2026-11-16 | worktree `.claude/worktrees/agent-a6be4559bf67aff21/.test_ws/regression/fix193-20261116-153021/result.json`: `{"failsafe": {"engagements": 41, "engaged_s": 33.48, "stops_while_driving": 0}}`、`…-155044`: `engagements 40, engaged_s 33.0, stops_while_driving 0, params ["scan_silent_sec:=0.5"]`（未マージ PR #203 のブランチ、failsafe のみ） | [A]（Git 管理外） |
| main `.test_ws/regression/` | ディレクトリ自体が無い（main 側で replay を実行した出力は残っていない） | [D: 記録なし] |
| PR #210・#211 | 検証欄は `run_tests.sh --scripts-only`（199 件・217 passed）と `check_map_sync.py ok`。`demo_regression.sh` の記載は無い | [A]/[D] |

`.github/pull_request_template.md` は「デモ回帰チェック」3 段のチェック欄を持つが、PR 本文での記入状況は上表以外確認していない。

---

## D. 文書とコードの整合

主セッションの `--show-args` 結果と launch のコードを根拠にした。launch 名・既定 site・ポート・DDS XML 名・速度・優先度は、下の食い違い以外は README / architecture-overview / runbook / development / scripts.md / docs/README / CLAUDE.md とコードで一致した [B]。

一致を確認した主な値 [B]: 既定 site `campus-m8`（dispatch_launch / m6r_bringup / nav_launch の 3 つ、start_dispatch_demo.sh の `SITE=campus-m8`）。nav_launch の `map_variant` 既定 `''`→campus-m8 は `occupancy_candidate.yaml`、それ以外は `occupancy.yaml`。`speed` 既定 `slow`、`speed_caps = {'slow': 0.30, 'normal': 0.45, 'fast': 0.60}`（`nav_launch.py:212`）、start_dispatch_demo.sh は `SPEED=normal`。twist_mux 優先度 100/50/10・timeout 0.5。EKF `frequency: 30.0`。localizer `use_odom: true`、`predict_pose_from_previous_delta: false`、`use_imu: false`、`registration_method: NDT_OMP`。RPP `use_collision_detection: true`。global costmap `[static_layer, inflation_layer]`、local `[static_layer, obstacle_layer, inflation_layer]`、behavior_server `[spin, backup, wait]`。velocity_smoother 出力 `/whill/controller/cmd_vel`。DDS XML `configs/cyclonedds-runtime-min.xml`（start_dispatch_demo.sh・m6r_preflight.sh・runbook・`~/.bashrc:140` が同じ）。preflight の番号 0, 0b, 1〜5。Layer D `FORWARD_SECTOR_HALF_ANGLE_RAD = math.radians(30.0)`、`MIN_M = 1.0`、`MAX_M = 2.0`。campus-m8 `static.pcd` sha256 は metadata.yaml・runbook:55 と実ファイルが一致（`d244865866e3…d03b5`）。

### D-1. 食い違い [D]

| # | 文書側 | 根拠側 | 内容 |
| --- | --- | --- | --- |
| 1 | `docs/runbook-live-demo.md:7`「広げた版は屋外ではまだ走らせていない」、同:5「屋外の受入（#156）はまだで」 | `~/bags/2026-11-27-m8-drive/README.md`:3-5「地図: campus-m8 / variant candidate（154w で広げた版…）」「start_dispatch_demo.sh --speed slow」、goal 成功・停止の表 | 2026-11-27 に広げた版で実機走行した記録が repo 外にある。runbook は未更新（その走行が屋外かどうか、#156 の受入に当たるかは、記録の冒頭に場所の明記がなく本監査では判断しない） |
| 2 | `docs/runbook-live-demo.md:280-281` `mkdir -p docs/m7-bench-data` / `ros2 bag record -o "docs/m7-bench-data/…-live-campus"` | `docs/bags.md:3`「走行 bag は `~/bags/` に置く。`docs/` には置かない」、`scripts/record_avoidance_bag.sh`（`~/bags/<run>/`、mcap） | 録画先と方法が二通り。runbook の録画節は record_avoidance_bag.sh に触れない |
| 3 | `src/whill_dispatch/package.xml` description「dispatch adds no TF and no cmd_vel」、`dispatch_launch.py` docstring「dispatch adds no TF and no /cmd_vel*」 | `dispatch_node.py:192-193` が `/cmd_vel_teleop` を publish | A-2 と同じ |
| 4 | `scripts/m7_https_server.py:9-11`「--cert-serve-dir で見せると」 | 同ファイル argparse に該当引数なし | A-2 と同じ |
| 5 | `README.md:30`「Web は HTTPS 8000、rosbridge は WSS 9090」、`runbook:3` 同旨 | `dispatch_launch.py` 既定 `use_tls`=`false`（HTTP / WS）。HTTPS/WSS になるのは start_dispatch_demo.sh が `use_tls:=true` を渡す時 | 条件付きの値。パッケージ README:7 は「既定はHTTP 8000 / WS 9090、`use_tls:=true` でHTTPS / WSS」と正しい |
| 6 | `README.md:30`・`runbook:3`・`whill_dispatch/README.md:46` の `whill_console`（gateway 8765）、`runbook:113` の `whill_platform` の `env.sh` | リポジトリ内にコード・設定なし（`git grep` は文書のみ）。`~/whill_console*` も無い | 外部構成への言及だけ [C] |
| 7 | `docs/development.md:3`「上流の取得元と版は `whill_lab.repos`」 | `src/third_party/glim`・`glim_ros2` は whill_lab.repos に無く、`scripts/install_glim.sh:66,68` が `GLIM_REF="master"`・`GLIM_ROS2_REF="master"` で取得。realsense-ros は whill_lab.repos で 4.55.1 だが、このホストでは `src/third_party/realsense-ros/COLCON_IGNORE`（2026-10-28、Git 管理外）で無効化し apt の `ros-humble-realsense2-camera 4.58.4` を使用（`ros2 pkg prefix realsense2_camera` → `/opt/ros/humble`）。経緯は `docs/plans/2026-10-28-avoidance-baseline.md:111-114` だけに記載。`import_upstream.sh` は realsense に COLCON_IGNORE を置かない | 新規 clone の手順ではソースビルドの 4.55.1 になる |
| 8 | `docs/scripts.md:12` setup_workspace.sh「既定workspaceは `~/ros2_ws`」 | 記述はコードと一致。ただし対象がこのリポジトリ（`~/whill_lab0_ros2`）ではなく、`.bashrc` に追記する（`CLAUDE.md:25`「bashrcは自動編集しない」） | 文書とコードは一致、運用規則との関係のみ |
| 9 | `scripts/m5r3_loop_error.py` 冒頭「Used by scripts/m5r3_run_glim.sh and scripts/m5r3_run_fastlio_sam.sh」、ADR-0003 | `scripts/m5r3_run_fastlio_sam.sh` は存在しない | B-1 |
| 10 | `docs/regression-check.md:9`・PR テンプレート:29「デモのルートを1周」 | 「デモのルート」を定義する現行文書は無い（runbook 9 節は配車手順のみ） | [C] |
| 11 | `.github/pull_request_template.md` 「方針適合チェック (docs/plans/2026-06-11-platform-pivot.md 5 章)」: `tf_bridge_launch.py` の identity 構成、`use_collision_detection: false` のまま等 | `tf_bridge_launch.py` はリポジトリに無い。`use_collision_detection: true`（nav2_params.yaml:184）。PR #176 本文も「旧計画（FAST-LIO、identity TF など）を前提にしている」と記載 | テンプレートが撤廃前の計画を基準にする |
| 12 | `docs/m7-bench-data/2026-07-19-mock/README.md:37,77` 地点 `kounoken/gate/bldg7/library` | 現 `waypoints.yaml`（campus・campus-m8）は `kounoken/bldg7/seimon/bldg3` | 履歴記録（当時の値）。現行手順としては使えない |
| 13 | `m6r_preflight.sh:13-14`（ヘッダ）「/cmd_vel_safety not publishing at the expected 20 Hz」 | 実装は 30 s 以内の最初の 1 件と、その後 6 s で 60 件以上（`:137-147`） | 表現の差のみ（矛盾ではない） |

`docs/README.md` が挙げるリンク先（research の HTML 4 本、plans/roadmap-m8、maps/README、calibration-ledger、perception-blind-zone）は実在する [B]。

### D-2. 撤廃した配車／デモの目的を現在の目的として示す文書

`git grep VLA` で目的の更新を書いているのは `CLAUDE.md:5`、`docs/plans/2026-10-30-roadmap-m8.md:3-9`、`docs/decisions/0013-rescue-while-stopped.md:24`、`docs/perception-blind-zone.md:3` だけ [B]。次の現行文書は配車／デモを主目的・既定として書いたまま [C]:

| 文書:行 | 記述 |
| --- | --- |
| `README.md:3` | 「…フェイルセーフ、タブレットからの配車を含む」（スタックの説明に研究基盤の記述なし） |
| `README.md:5` | 「統合起動と実機配車は 2026-10-14 に成功した」（最新の実機記録として掲示。2026-11-27 走行は未掲載） |
| `README.md:27,36-39` | 「配車・Web」の行。起動は `scripts/start_dispatch_demo.sh`「bringup・Nav2・配車 Web・RViz を起動する」 |
| `docs/runbook-live-demo.md:1` | 表題「実機配車デモの起動・停止手順」。:3「対象は…配車 Web」、:45「## 9. 配車と手動救出」、:85-89 デモ用 AP |
| `docs/regression-check.md:1,3` | 「デモ回帰チェック」「main を常にデモができる状態に保つための」、:46「デモの既定 site」 |
| `docs/README.md:83,97` | 「デモ回帰チェック」、実機検証記録は「2026-10-14 統合起動・配車成功」の 1 件のみ |
| `docs/development.md:42,47` | 「デモが使う `install/`」「デモ回帰チェック」 |
| `docs/scripts.md:25` | 節名「実機・配車・検証」 |
| `.github/pull_request_template.md:11-17,25-29` | platform-pivot 計画の方針適合チェック、デモ回帰チェック |
| `src/whill_dispatch/package.xml` description | 「M7 dispatch (配車) API boundary… at demo time」 |
| `CLAUDE.md:23` | 「実機起動はm6r_bringup + nav_launch + dispatch_launch」— コードとは一致するが、同:5 の目的（配車を目的として扱わない）と並ぶ |
| `docs/plans/2026-10-30-roadmap-m8.md:24` | 目的更新後の文書だが方針 2 で「デモ回帰チェック（#142）を通す」 |

名前の上でも、現用の統合起動 `start_dispatch_demo.sh`、ログ `log/dispatch-demo-*`、ロック `log/dispatch-demo.lock`、証明書 `~/.whill_dispatch_tls/` が「dispatch/demo」を冠する [B]。

---

## E. リポジトリ衛生（事実のみ）

### E-1. 残骸に見えるもの [B]

| パス | 追跡 | 大きさ | 最終 commit | 参照・内容 |
| --- | --- | --- | --- | --- |
| `configs/cyclonedds-runtime.xml.bak` | **ignored**（`*.bak`） | 8.0K（4782 B、2026-07-24 17:00） | — | `configs/legacy/cyclonedds-runtime.xml` と比べ、legacy 側の DEPRECATED ヘッダ（11 行）が無いだけの差。旧 NIC `enx00e04c6808dc` を列挙する旧 runtime XML |
| `configs/legacy/cyclonedds-runtime.xml` | 追跡 | 8.0K | 2026-10-30 | ヘッダ「DEPRECATED (2026-10-30 に configs/legacy/ へ移動) 使わない。現用は configs/cyclonedds-runtime-min.xml」。参照: cyclonedds-bag-record.xml、runtime-min.xml、m6r-bench-data manifest、plans 2 本、repository-refresh、`scripts/tests/test_record_avoidance_bag.py` |
| `configs/cyclonedds-bag-record.xml` | 追跡 | — | — | 参照: legacy XML、m6r-demo-prep-checklist、plans/m6r-localization、repository-refresh（現用文書からの参照なし。CLAUDE.md:25 は「旧runtime / bag-record XMLとの混在を避ける」） |
| `imgui.ini` | **ignored**（`/imgui.ini`） | 4.0K（417 B、2026-07-12） | — | GLIM viewer の Dear ImGui 窓配置 |
| `scratchpad/` | `.py` 3 本を追跡、PNG 4 枚は ignored | 508K | 2026-11-12 | `frame_audit.py`・`imu_live_check.py`（calibration-ledger、imu-coordinate-audit が参照）、`investigate_thin_corridor.py`（ADR-0003、maps/campus/README が参照）。PNG 4 枚（2026-07-10） |
| `network/01-velodyne-static.yaml.template` | 追跡 | 8.0K | 2026-10-30 | 参照: m3-sensors、check_dds_nic.py、install_velodyne_network.sh |
| `udev/99-whill-stack.rules` | 追跡 | 8.0K | 2026-05-07 | 参照: runbook、check_device_aliases.sh、install_udev_rules.sh、test_preflight.py、odom_bringup_launch.py、imu_launch ほか（現用） |
| `docs/maps/campus/v2/build/`・`install/`・`log/` | ignored | 40M / 80K / 44K | — | v2 ディレクトリ内で colcon が走った跡 |
| `docs/m3-bench-data/build/`・`install/`・`log/` | ignored | 8.0K / 80K / 28K | — | 同上 |
| `src/whill_sensors_bringup/scripts/` | ignored の `__pycache__/` のみ | — | — | 2026-06-11 のディレクトリ |

### E-2. ignored / untracked [B]

`git status --short --untracked-files=all` は空（untracked なし）。`git status --ignored --short` は 3065 行。主なもの（GLIM keyframe ディレクトリは集約）:
- `build/` 1.9G、`install/` 258M、`log/` 206M、`.test_ws/` 11M、`src/third_party/` 238M、`.claude/worktrees/` 8.2G、`.pytest_cache/`、`.claude/settings.local.json`
- `docs/m5r-bench-data/<run>/…`（GLIM・DUFOMap 出力、19 run ディレクトリ）、`docs/m6r-bench-data/2026-07-08-smoke-campus-demo/…`
- 地図 PCD: `docs/maps/{campus,campus-m8,campus-demo,campus-half-v3,campus-outdoor-corrected}/static.pcd`、`docs/maps/lab-legacy-m5b/{lab.pcd,global_2026-06-04_10min.pcd}`
- GIMP: `docs/maps/campus/occupancy_cleaned.xcf`、`docs/maps/campus/occupancy_cleaned１.xcf`（全角数字の名前）、`docs/maps/campus/v2/final.xcf`
- v2 自動生成物: `docs/maps/campus/.v2_cache.npz`、`v2/clipped_by_roadway.png`、`erased_by_free.png`、`free_mask_auto.png`、`salt_candidates_2to3.csv`
- `__pycache__`: scripts、scripts/regression、scripts/tests、whill_dispatch、whill_safety/test、whill_sensors_bringup/launch・test
- `log/` 内: `dispatch-demo-20261014-{134410,134722,135842,140220}-*`、`-20261113-{113616,153041}-*`、`-20261127-{154932,160516}-*`、`dispatch-demo.lock`、colcon の `build_*`

### E-3. ブランチ・worktree [B]

ローカルブランチ 53 本（main を含む）。main = origin/main = `64f7a49`。
**main に未マージ**（`git merge-base --is-ancestor` が偽）:

| ブランチ | main より先 | 最終 commit | PR |
| --- | --- | --- | --- |
| `docs/bag-inventory-20261127` | 1 | 2026-11-27 | なし（`4bd9619 docs(reports): このPCの bag の棚卸し`） |
| `feat/localization-check-script` | 1 | 2026-11-27 | なし（`692686a … check_localization.py`、2026-11-27 走行の解析に使用と ~/bags README に記載） |
| `fix/193-scan-silent` | 3 | 2026-11-16 | #203 OPEN |
| `feat/avoidance-baseline` | 7 | 2026-10-30 | なし |
| `feat/avoidance-c2` | 11 | 2026-10-30 | なし（STVL、replay_depth_to_points） |
| `feat/avoidance-c3` | 9 | 2026-10-30 | なし（Collision Monitor、Layer D 矩形） |
| `integration/186-190` | 2（merge commit のみ） | 2026-11-13 | なし。取り込んだ 2 本の第 2 親（`9f6ad27`、`5be4da8`）はいずれも main に含まれる |
| `m5e/velodyne-self-filter` | 12 | 2026-06-04 | なし。upstream 設定が壊れている（`m5e/velodyne-self-filter@{upstream}`）。内容は fast_lio_sam の RViz 設定・NEO-M8P（GNSS）など |

マージ済み: `docs/143…`、`docs/149…`、`docs/166…`、`exp/154…`、`exp/202-*`（4 本）、`exp/205…`、`feat/144-145…`、`feat/149-*`（2）、`feat/152-153…`、`feat/154…`、`feat/171…`、`fix/141…`、`fix/146…`、`fix/159…`、`fix/161…`、`fix/169…`、`fix/170…`、`fix/192…`、`fix/194…`、`fix/record-bag-foreground`、および `worktree-agent-*` 20 本（全てマージ済み・ahead 0、upstream なし、最終 2026-11-09〜11-19）。`docs/143…`・`docs/149…`・`fix/record-bag-foreground`・`integration/186-190` の upstream は `origin/main` に設定されている。

`git worktree list`: 本体 + `.claude/worktrees/` 下 14 本（`agent-a2c7f7e…`[docs/bag-inventory-20261127]、`agent-a5ff978…`[feat/154-adopt-widened-map]、`agent-a60c1dd…`[feat/localization-check-script]、`agent-a6be455…`[fix/193-scan-silent]、`agent-aea1688…`[exp/202-gsi-edge-snap]、`agent-aecfa03…`[feat/144-145-lidar-calib]、`agent-aefeb92…`[exp/154-widen-hand-to-gsi]、`avoidance-baseline`、`avoidance-c2`、`avoidance-c3`、`footprint-143`、`m8-integration`、`outdoor-149`、`record-fg`）。うち 7 本はマージ済みブランチを checkout している。**prunable 2 本**: `/tmp/claude-1000/…/42aa6711…/scratchpad/review/docs_166`（`358d427` detached）、`…/review/fix_193`（`caab732` detached）。`.claude/worktrees/` 合計 8.2G。

リモートブランチ 38 本（`origin/main` 含む）。ローカルに無いもの: `origin/chore/139-bag-location`、`origin/docs/138-research-docs`、`origin/docs/goal-pivot-research`、`origin/docs/roadmap-m8`、`origin/fix/138-import-upstream-colcon-ignore`、`origin/fix/web-map-source-pgm`、`origin/investigate/147-imu-audit`。

### E-4. whill_lab.repos と src/third_party [B]

| エントリ | version（引用） | 固定か | checkout の HEAD | 一致 | ローカル変更 |
| --- | --- | --- | --- | --- | --- |
| third_party/ros2_whill（Iruazu fork） | `humble-with-odom-2026-06-18` | 注釈付きタグ | `d2d12189…` | タグの commit と一致 | なし |
| third_party/ros2_whill_interfaces | `humble` | **ブランチ（浮動）** | `66173522…`（branch humble、upstream と同じ。FETCH_HEAD 2026-07-15） | — | なし |
| third_party/velodyne | `2.5.1` | タグ | `0f2a3bb1…` | 一致 | なし |
| third_party/realsense-ros | `4.55.1` | タグ | `8a86cb88…` | 一致 | untracked `COLCON_IGNORE`（2026-10-28）。実行時は apt 4.58.4 |
| third_party/rt_usb_9axisimu_driver | `humble-devel` | **ブランチ（浮動）** | `f1bdaf4a…`（describe `2.1.0`、FETCH_HEAD 2026-07-15） | — | なし |
| third_party/lidar_localization_ros2 | `1dbcf3a655328d79a52252d11c8836502011ba5f  # 2026-07-04 G3 refactor` | commit | `1dbcf3a6…` | 一致 | なし |
| third_party/ndt_omp_ros2 | `humble` | **ブランチ（浮動）** | `ef8a3498…`（FETCH_HEAD 2026-07-15） | — | なし |
| third_party/patchwork_plusplus | `v1.4.1` | タグ | `3e6903a1…` | 一致 | untracked `ros/COLCON_IGNORE`（import_upstream.sh が作る。cpp/・python/ の COLCON_IGNORE は status に出ない） |
| （repos に無い）glim | install_glim.sh `GLIM_REF="master"` | **ブランチ（浮動）** | `88b38332…`（`v1.2.1-3-g88b3833`） | — | **8 ファイル変更**（`config/config.json`、`config_global_mapping_gpu.json`、`config_logging.json`、`config_odometry_gpu.json`、`config_preprocess.json`、`config_viewer.json` ほか、124 insertions / 287 deletions） |
| （repos に無い）glim_ros2 | `GLIM_ROS2_REF="master"` | **ブランチ（浮動）** | `a62811dc…`（`v1.2.1-2-ga62811d`） | — | なし |

### E-5. package.xml と実装の依存の差（具体的なものだけ）[B]

| パッケージ | 宣言されていないが使っている | 宣言しているが使っていない |
| --- | --- | --- |
| whill_localization | `ament_index_python`（`ekf_odom_launch.py:20`、`odom_bringup_launch.py:52`） | `rviz2`（`package.xml:30`。launch に rviz の起動なし） |
| whill_navigation | Nav2 パラメータのプラグイン提供元: `nav2_navfn_planner`（NavfnPlanner）、`nav2_regulated_pure_pursuit_controller`（RPP）、`nav2_costmap_2d`（Static/Obstacle/InflationLayer、costmap ノード）。BT XML（`navigate_to_pose_no_spin_bt.xml`: ComputePathToPose、FollowPath、ClearEntireCostmap、Wait ほか）の提供元 `nav2_behavior_tree` | — |
| whill_perception | `ament_index_python`（`ground_removal_launch.py:20`） | — |
| whill_sensors_bringup | `ament_index_python`（`sensors_launch.py:67` ほか） | — |
| whill_dispatch | 実行時にリポジトリの `scripts/m7_https_server.py`・`scripts/mock_navigate_to_pose.py` を読む（install されない。依存宣言の対象外） | — |
| whill_safety | 差なし（`lidar_localization_ros2`・`twist_mux`・`whill_localization`・`whill_sensors_bringup` などを宣言） | — |

（`/opt/ros/humble` に該当パッケージはすべて入っているため、このホストでは実害は出ていない。rosdep で新規環境を作る場合にだけ効く差。）

### E-6. install/ の鮮度 [B]

`install/whill_dispatch/lib/whill_dispatch/dispatch_node`（コピー、2026-07-22）、`install/whill_safety/lib/whill_safety/failsafe_node`（2026-11-13）、`install/whill_sensors_bringup/lib/whill_sensors_bringup/imu_sign_corrector`（2026-11-13）は、いずれも src と内容が同一（`diff -q`）。launch・web は symlink。
