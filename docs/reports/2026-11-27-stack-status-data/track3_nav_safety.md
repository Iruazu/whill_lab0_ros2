# Track 3 監査：navigation / safety / perception（main 64f7a49、2026-11-27）

読み取りのみ。リポジトリのファイルは変更していない。ROS ノードの起動・topic の publish・build・test 実行はしていない。

## 証拠の区分

- **[A]** 実機の測定記録（日付・条件・数値）がある
- **[T]** リポジトリの自動テストが扱う（純ロジックか、記録データを使うかを書く）
- **[B]** 現在のコード・設定から読んだだけ（実機の挙動は示されていない）
- **[C]** 文書に書かれているだけ。生データは見つからない
- **[D]** 証拠なし、または食い違い（両方を原文で示す）

## 基準となる日時（下層の修正）

| 事象 | main に入った日時 | 根拠 |
| --- | --- | --- |
| LiDAR 校正表 VLP16db → VLP16_hires_db（#144）＋ base_link→velodyne TF（#145） | PR #186、2026-11-13 16:24 JST（merge 5392200） | `git log --merges` |
| campus-m8 地図・既定 site（#190） | PR #190、2026-11-13 16:57 JST（c39beed） | 同上 |
| Patchwork++ sensor_height 0.79→0.771（#192） | PR #201、2026-11-16 15:55 JST（4fdd81e） | 同上 |
| 旧 TF | base_link→velodyne (0.484136, 0.381548, 0.793704) / RPY (−0.035342, **+0.156983 rad = +8.99°**, −0.005527)。2026-06-20（78f6658）〜2026-11-12 | `git log -L` static_tf_launch.py |
| 現 TF | (0.42, 0.36, 0.771) / roll −0.05°、pitch +0.40°、yaw −0.317°（yaw は未測定、旧値の流用） | `src/whill_sensors_bringup/launch/static_tf_launch.py:85-89` |

以下「修正前」＝ 2026-11-13 16:24 より前に最後に設定された値。

## 現構成での唯一の走行記録（2026-11-27）

`~/bags/2026-11-27-m8-drive/README.md` と `logs/*/nav2.log`・`bringup.log`（リポジトリ外、未コミット）。main 64f7a49、site campus-m8、map_variant candidate、`start_dispatch_demo.sh --speed slow`（0.30 m/s）、failsafe / twist_mux 有効、カメラなし、goal は RViz から。**[A]**

| goal（nav2.log） | 結果（nav2.log） |
| --- | --- |
| (-0.96,-0.52)→(7.95,-0.16) | Goal succeeded（run1 l.322） |
| (7.58,-0.08)→(23.06,0.31) | Goal succeeded（run1 l.380） |
| (22.74,0.45)→(102.07,5.87) | Failed to make progress ×4（l.528-583）→ 16:02:44 preempt で (-68.05,-2.04) へ → Failed to make progress ×2（l.603,621）→ Goal canceled（l.637、16:03:15） |
| (28.89,0.46)→(-36.99,-0.84) | Goal succeeded（run2 l.430。bag なし） |
| (-36.61,-0.99)→(-91.32,29.49) | RPP "detected collision ahead!" ×6（run2 l.436,440,446,485,491,647）、Failed to make progress ×2（l.614,633）→ Goal canceled（l.665、16:14:25） |

- 数の食い違い：README は drive03 を「Failed to make progress ×5（bag の /rosout）」、drive04 を「collision ahead ×5」と書く。nav2.log ではそれぞれ 6 回（preempt 前 4＋後 2）と 6 回。README は bag の範囲、ログは全区間を数えたためと読めるが、確かめていない。**[D]（数え方の違い）**
- failsafe（bringup.log）：起動ログ `forward_blocked > 1 pts in ±30° @ 1.0-2.0 m, hysteresis 0.5s | ... | Layer D rescue off | never-arrived inputs: warning only`。両起動で起動 10 s 後に `DEAD INPUT after 10s: ['B:/pcl_pose']`。ENGAGED：run1 計 15（D:forward_blocked 13、B:pcl_pose_silent 1＝15:50:29、C:perception_silent 1＝16:03:19＝取消の後）。run2 計 26（D 単独 8。B/C を含むものは 16:05:56 の B 1 件を除き 16:15:42 以降＝README が「機器を外した後」とする区間）。
- README の数値：drive03 で Layer D 遮断 758/878 scan、drive04 で 376/589 scan。反応点は base_link で前 1.4〜1.8 m・左 0.88〜0.98 m（drive03）、前 1.55〜1.73 m・右 0.96〜0.98 m（drive04）、点の高さ中央値 0.55 m / 0.66 m、地図 OCC まで 0.05 m。経路から OCC まで最小 0.70 m / 0.67 m。車体長方形（後端 −0.25、前端 +0.75、幅 0.554 m の仮定）→ OCC 最小 **0.20 m（drive03 左前角）/ 0.30 m（drive04 右前角）**、Nav2 の円（0.45 m）→ OCC 最小 0.11 m / 0.20 m。fitness 中央値 0.020〜0.063、p95 ≤ 0.092、最大 0.160。
- **README の解釈（README の主張であり本監査は検証していない）**：「止めたのは地図にある静的な構造物」「原因は Layer D の扇形と Nav2 の経路の幅の食い違い」「矩形の Layer D なら 1〜2 scan しか遮断しない（オフラインの見積もり）」「154w で広げたことは原因ではない」「自己位置は合っている」。
- この記録に**無いもの**：停止距離、停止までの時間、smoother 経由の減速の測定、人・低い物に対する Layer D、teleop（`cmd_vel_teleop` は全区間 0 件）、speed normal/fast。

---

## (1) Navigation パラメータ（下層依存と修正前後）

ファイル：`src/whill_navigation/launch/nav_launch.py`、`config/nav2_params.yaml`（以下 np）、`config/pointcloud_to_laserscan.yaml`（以下 p2ls）、`config/navigate_to_pose_no_spin_bt.xml`（以下 BT）、`src/whill_perception/config/patchworkpp.yaml`（以下 pw）。日付は `git blame` の commit 日。

### 1.1 launch 引数（nav_launch.py）

| 引数 | 既定 | 行 | 内容 | 区分 |
| --- | --- | --- | --- | --- |
| site | `campus-m8` | :117, :352-361 | `docs/maps/<site>/occupancy[_<variant>].yaml` | [B] |
| map_variant | `''` → campus-m8 は `candidate` | :125-127, :159-160, :362-380 | 2026-11-19 以降 154w 拡幅版 | [B] |
| speed | `slow` | :211-221, :382-388 | slow 0.30 / normal 0.45 / fast 0.60 m/s。RPP `desired_linear_vel` と smoother `max_velocity[0]` だけを上書き、accel/decel は不変。説明文「現地で段階的に上げて各段の停止性能を実測すること」 | [B]。各段の停止性能の測定記録は見つからない [D]。runbook は `speed:=normal`（docs/runbook-live-demo.md:128）、2026-11-27 走行は slow |
| BT | `navigate_to_pose_no_spin_bt.xml` を per-run yaml に焼き込み | :226-227 | `default_nav_to_pose_bt_xml` のみ。through_poses は未設定 | [B] |
| remap | controller/behavior `/cmd_vel→/cmd_vel_nav`（:301, :312）、smoother `/cmd_vel_smoothed→/whill/controller/cmd_vel`（:333） | | | [B] |
| 同梱 | patchworkpp（ground_removal_launch）、pointcloud_to_laserscan（`cloud_in=/velodyne_points_no_ground`、`scan=/scan`）、map/planner/controller/behavior/bt/smoother、lifecycle_manager（bond_timeout 4.0） | :262-346 | | [B] |

### 1.2 パラメータ表

「依存」＝下層（footprint、TF、校正表、センサ高さ、地図）への依存。

| パラメータ | 値 | file:line | 最終設定 | 依存 | 修正前/後 | 修正後の再検証 | 調整理由の記録 | 区分 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| local `robot_radius` | 0.45 m | np:231 | 262772fa 2026-05-20 | footprint | 前 | なし。11-27 の README は車体前角 → OCC 0.20/0.30 m を算出 | コメント「1.0 m × 0.6 m を 0.45 m の円で覆う」。#157 は「この コメントは誤り」、台帳 §1 は前端 0.72〜0.75 m（実測 2026-11-16）、後端・最大幅は未測定 | [B]、前端は [A]、#157 未着手 |
| global `robot_radius` | 0.45 m | np:282 | 262772fa 2026-05-20 | footprint | 前 | 同上 | 同上 | [B] |
| `footprint`（多角形） | 未設定 | — | — | footprint | — | — | #157 OPEN | [B] |
| inflation_radius（local/global） | 0.6 m | np:270, :300 | 6ba656bc 2026-07-21 | footprint、地図 | 前（campus v2 で調整） | 11-27 走行で使用。README は「経路を OCC から 0.67〜0.70 m 離す」と算出 | コメントに 07-21 field の理由（0.4 < 0.45 の死角、端 0.25 m）。生データ所在の記載なし | [B]、理由は [C] |
| cost_scaling_factor（local/global） | 4.0 | np:267, :298 | 6ba656bc 2026-07-21 | 地図 | 前 | なし | 07-21 の理由あり。「左偏りは実機で再確認」→ 記録なし | [C] |
| local costmap 窓 | 10×10 m、0.05 m | np:224-226 | 25d9b993 2026-07-16 | /scan の clearing 幾何 | 前 | なし | 07-16 field「ghost lethals」 | [C] |
| local plugins | static, obstacle, inflation | np:232 | 9370f8ed 2026-07-14 | | 前 | | | [B] |
| obstacle_layer 入力 | `/scan`、`sensor_frame: base_link`、LaserScan、max_obstacle_height 2.0、obstacle_max_range 20.0、raytrace_max_range 25.0、min 0.0 | np:240-254 | 9370f8ed 2026-07-14 | /scan（TF・校正表・p2ls の帯） | 前 | なし | | [B] |
| global plugins | static, inflation（obstacle_layer なし） | np:291 | 1fe651e3 2026-07-20 | | 前 | | 07-20 field の残像の理由 | [C] |
| p2ls `target_frame` | base_link | p2ls:46 | 9370f8ed 2026-07-14 | TF に直接依存 | 前（値は同じだが TF の中身が 11-13 に変わった） | #191 OPEN | | [B] |
| p2ls `min_height` | 0.05 m | p2ls:48 | 5c8162a4 2026-07-14（ADR-0009 の A/B は 2026-07-15） | TF・校正表 | **前**（旧 TF pitch +8.99°、VLP16db） | **なし**。#191（2026-11-13 起票、M8-3、コメント 0）が再検証待ち。perception-blind-zone.md §4 は新 TF の計算で「Layer D にも 3.9 m より手前にも効かない、0.05 は変えなくてよい」と書く（計算、実測なし） | ADR-0009 §検証結果 2026-07-15（脚 ~4 点、偽 lethal 0）。生データの所在なし | [C]。新構成では [D]（未検証） |
| p2ls `max_height` | 1.6 m | p2ls:49 | 9370f8ed 2026-07-14 | TF | 前 | なし | 「立位の人の胴・頭、天井」。測定記録なし | [B] |
| p2ls `range_min` | 0.5 m | p2ls:56 | 9370f8ed 2026-07-14 | TF（距離の原点） | 前 | なし | コメント「LiDAR 原点から 0.5 m の自己反射」。target_frame が base_link なので距離は base_link 原点から（perception-blind-zone.md §1 も同旨）→ §6 の食い違い | [D] |
| p2ls `range_max`、角度 | 25.0 m、−π〜π、0.008726646 rad（0.5°）、scan_time 0.1、use_inf true | p2ls:50-62 | 9370f8ed 2026-07-14 | | 前 | | | [B] |
| Patchwork++ `sensor_height` | 0.771 m | pw:32 | 554be414 **2026-11-16** | LiDAR z | **後** | bag 再生で 0.79 と 0.771 の no_ground 差 0 点（07-24 走行 1774 フレーム、11-16 静止 117・116 フレーム）。着座の bag は未確認 | `src/whill_perception/README.md`、台帳 §6。結果ファイルはリポジトリに無い（入力 bag は `~/bags` に有り） | [C]（記録データの再生、自動テストではない） |
| Patchwork++ `min_range` | 1.0 m | pw:33 | e8241b05 2026-07-14 | LiDAR 原点基準 | 前 | なし | コメント「WHILL body self-return standoff (~0.5 m from LiDAR)」。挙動は §3・§6 の食い違いを参照 | [B]／[D] |
| Patchwork++ `max_range` | 25.0 m | pw:34 | e8241b05 2026-07-14 | | 前 | | | [B] |
| Patchwork++ その他 | struct の既定（yaml に書かない） | pw:36-38 | | | | | 「field data で必要が出たら」 | [B] |
| RPP `desired_linear_vel` | 0.3（speed で上書き） | np:137 | 9370f8ed 2026-07-14 | | | | | [B] |
| RPP `max_linear_accel` / `decel` | 0.3 / 0.6 | np:138-139 | 2026-05-20 | | | | コメント「documentation only」 | [B] |
| RPP lookahead | 0.8 / min 0.6 / max 0.9、time 1.5 | np:146, :156-158 | 25d9b993 07-16、645332f5 07-20 | 自己位置（旧 TF・旧地図の NDT） | 前 | 「次回実機で単独 A/B で効果確認する」→ 記録なし。11-27 README は drive03 で「角速度の指令が ±0.5〜0.8 rad/s で左右に振れた」と記す | 07-19 bag の数値（angular.z RMS 0.462、周期 2.1 s、波長 0.58 m、cross-track RMS 5 cm） | [C] |
| `rotate_to_heading_angular_vel` | 0.4 rad/s | np:166 | bb0666c4 2026-07-22 | | 前 | なし | seimon-stall README（07-22 bag） | [C]（bag は `~/bags/m7-bench-data/2026-07-22-live-campus/seimon-stall/` に有り） |
| `rotate_to_heading_min_angle` | 1.57 rad | np:199 | b77804d3 2026-07-22 | | 前 | なし | 同上 | [C] |
| `use_collision_detection` / `max_allowed_time_to_collision_up_to_carrot` | true / 1.0 s | np:184-185 | d8ab2664 07-16 / 05-20 | footprint（円 0.45）、/scan | 前 | 11-27 run2 で "collision ahead" 6 回（nav2.log） | ADR-0007 は射程 = 1.0 × 0.3 = 0.3 m（normal 0.45 m、fast 0.60 m は計算値） | [B]、作動は [A] |
| regulated scaling | use true、cost_regulated false、min_radius 0.9、min_speed 0.25 | np:186-189 | 2026-05-20 | | | | | [B] |
| `allow_reversing` | false | np:191 | 2026-05-20 | 後方センサなし | | | | [B] |
| progress checker | radius 0.5 m、allowance 15.0 s | np:104, :110 | b77804d3 2026-07-22 | | 前 | 11-27 で "Failed to make progress" 計 8 回 | 07-22 | [C]、作動は [A] |
| goal checker | xy 0.5 m、yaw 3.15、stateful | np:122-124 | 93655fb9 2026-07-21 | | 前 | | 理由は「配車は … 到着時の向きは指定しない」（撤廃した配車目的に基づく） | [C] |
| planner NavFn | tolerance 0.5、use_astar false、allow_unknown false | np:317-326 | 2026-05-20 / 9370f8ed 07-14 | 地図 | 前 | | コメントは M5-R campus の外周地図を前提に「v2 で内側が埋まったら見直す」 | [B] |
| bt_navigator | transform_tolerance 0.3、bt_loop 10、server_timeout 20 | np:29-35 | 07-14 | | | | | [B] |
| velocity_smoother | 20 Hz、OPEN_LOOP、max_velocity [0.3,0,0.5]（vx は speed で上書き）、min [−0.1,0,−0.8]、max_accel [0.3,0,1.0]、max_decel [−0.5,0,−1.0]、velocity_timeout 1.0 | np:356-377 | 8cd7fe5b 05-20、25d9b993 07-16（vyaw 0.5） | 停止経路（§2） | 前 | 停止の測定なし | | [B] |
| behavior_server | plugins [spin, backup, wait]、simulate_ahead 2.0、max_rot 0.8、min_rot 0.2、acc 1.0 | np:328-347 | 2026-05-20 | | | | spin/backup は BT から呼ばれない（§5） | [B] |

### 1.3 旧 TF・旧校正表に対して調整され、修正後に再検証されていないもの

旧 TF は pitch +8.99°。#191 本文は「base_link 系での点の高さは 2 m 先で約 0.3 m 変わる」、台帳 §6 は「約 0.28 m」（+8.995°→+0.97° の差）、perception-blind-zone.md §4 は「+8.995°→+0.40° の差は 0.30 m」。どれも計算値。

| 項目 | 調整日 | 調整に使ったデータ | 再検証 |
| --- | --- | --- | --- |
| p2ls min_height 0.05 / max 1.6 / range_min 0.5 | 2026-07-14〜15 | 旧 TF・VLP16db の /scan | #191 OPEN。未実施 |
| Layer D 点数閾値 1（5→3→1） | 2026-07-19（4f908584） | 旧 TF の /scan（band_probe 149 scan、データはリポジトリに無い） | なし |
| Layer D 扇形 ±30°、1.0〜2.0 m、ヒステリシス 0.5 s | 2026-07-16/17 | 机上（ADR-0007）。V6.4 の幾何の実測は記録なし | なし。11-27 走行で 2 goal が D:forward_blocked で停止（README は扇形と経路幅の食い違いと解釈） |
| Layer C 2.0 s | 2026-07-16 | 机上（10 Hz の 20 フレーム） | なし（再生基準 bag に `/velodyne_points_no_ground` が無いので回帰でも作動しない、docs/regression-check.md） |
| inflation 0.6 / cost_scaling 4.0 / robot_radius 0.45 | 07-21 / 05-20 | 旧 campus v2 地図、旧 TF の NDT | 154w の地図作成（scripts/map_widen_hand_roadway.py:545-555）と nav2_plan_check.py がこの値を読む。footprint 実測後に地図評価も変わる |
| RPP lookahead / rotate 系 / progress 15 s | 07-16〜07-22 | 旧 TF・旧地図の走行 bag | なし |
| failsafe 再生回帰の基準（40〜42 回、走行中停止 0） | 2026-11-02 | 2026-07-24 bag に記録された（旧 TF の）/scan | docs/regression-check.md は「failsafe の項目は TF に依存しない（記録時の /scan を流す）」とする。新構成の /scan による基準はない |

調整理由の記録がないもの：p2ls max_height 1.6、Patchwork++ min_range 1.0 の実測根拠、Layer D 半角 30° の実測、velocity_smoother の加減速値の実測、twist_mux timeout 0.5 の実測。

---

## (2) 安全層

### 2.1 cmd_vel の経路 [B]

```
controller_server / behavior_server ─(/cmd_vel_nav, prio 10)─┐
dispatch_node（/dispatch/teleop→/cmd_vel_teleop, prio 50）───┼─ twist_mux ─(/cmd_vel)─ velocity_smoother ─(/whill/controller/cmd_vel)─ WHILL driver
failsafe_node（/cmd_vel_safety, prio 100、層が作動中のみ 20 Hz）─┘
```

- twist_mux：`src/whill_safety/config/twist_mux.yaml:27-41`。safety /cmd_vel_safety timeout 0.5 prio 100（0b9085e6 2026-07-14）、teleop /cmd_vel_teleop 0.5 / 50（baab83f2 2026-07-21）、navigation /cmd_vel_nav 0.5 / 10。lock topic は無い。出力 `cmd_vel_out→/cmd_vel`（safety_launch.py:119）。
- **停止指令も velocity_smoother を通る**：README.md:26「停止指令も velocity_smoother を通る」、docs/repository-maintenance.md:11「safety のゼロ指令も velocity_smoother を通る。停止距離は実機で確かめる」、src/whill_navigation/README.md:30。smoother は OPEN_LOOP・max_decel −0.5 m/s²（np:361, :373）。計算上（v/0.5、v²/(2·0.5)）の減速：0.30 m/s → 0.6 s・0.09 m、0.45 → 0.9 s・0.20 m、0.60 → 1.2 s・0.36 m。**計算値であり測定ではない**（反応遅れ・WHILL 側の応答を含まない）。ADR-0007:296 は 0.3 m/s の場合だけ「停止距離 0.09 m」と書く。[B]（計算）
- Layer D の扇形と車体の位置関係（計算、[B]）：/scan は base_link（後輪車軸中点）原点の距離。前端は 0.72〜0.75 m（台帳 §1 実測）なので、扇形の手前端 1.0 m は正面で前端の 0.25〜0.28 m 先、奥端 2.0 m は 1.25〜1.28 m 先。
- 停止距離・反応時間の測定：**リポジトリ・Issue・bench-data・2026-10-14 report・2026-11-27 README のいずれにも無い**。計画されたが存在しないもの：`scripts/m6r4_cmd_vel_stop_latency.py`（docs/plans/2026-07-14-m6r4-nav2-obstacle-layer.md:292、`git ls-files` に無い）、ADR-0007 G4（`/reinitialization_requested` → cmd_vel zero < 200 ms）、avoidance-baseline 計画 C3「slow → normal の順に停止距離を実測」（docs/plans/2026-10-28-avoidance-baseline.md:104、:228）。docs/reports/2026-10-14-dispatch-demo.md:34「この記録は1回の配車成功であり、preflight全項目・手動中のLayer D停止・停止距離・他経路の受入完了を意味しない」。docs/regression-check.md:38「速度の指令と停止距離：車体を動かさないので測れない」。**[D]（証拠なし）**

### 2.2 failsafe 層（`src/whill_safety/whill_safety/failsafe_node.py`、main）

既定の起動は rescue off、grace 0.0（safety_launch.py:73-90、m6r_bringup_launch.py:277-305）。2026-11-27 の起動ログがこれを示す [A]。

| 挙動 | 条件・閾値（file:line） | (i) テスト | (ii) 実機記録 | 区分 |
| --- | --- | --- | --- | --- |
| A:reinit | `/reinitialization_requested`（std_msgs/Bool、depth 10）が data=True → 1.0 s 保持（:117, :262, :430-434, :499-501）。発信元は lidar_localization_ros2（third_party lidar_localization_component.cpp:974-976、transient_local reliable） | 直接のテストなし。test_failsafe_rescue.py `test_other_layer_switches_relay_to_zero` が `_last_reinit_time` を直接代入して救援中継がゼロに変わることだけ確かめる（純ロジック） | なし。ADR-0007 G4（<200 ms）は未計測、ADR-0007 は **proposed**（:4）。PR #76 の mock 検証（「reinit 発火と 1s 復帰」）は ADR-0007:240 の記述のみ | [B]、mock は [C] |
| B:fitness | `/alignment_status` の fitness_score > 1.0 が 2.0 s 連続（:64-65, :436-464, :502-504） | なし | 11-27 起動 2 で 16:16:24 以降（README は機器を外した後と説明）。走行中の作動なし（README「全 bag で PASS」） | [B] |
| B:converged | has_converged false が 2.0 s（:505-507） | なし | なし | [B] |
| B:pcl_pose_silent | 初回受信後 `/pcl_pose`（PoseWithCovarianceStamped）が 1.0 s 途絶（:66, :508-512） | ラベル名のみ（rescue テスト）。途絶の判定自体のテストなし | 11-27：15:50:29、16:05:56（起動直後）、16:15:42 以降（機器取り外し後） | [B]、作動は [A] |
| C:perception_silent | 初回受信後 `/velodyne_points_no_ground` が 2.0 s 途絶（:70, :280-282, :517-519） | test_failsafe_never_arrived.py `test_late_silence_still_uses_the_existing_watchdog`（純ロジック、時刻を差し替え）が 2.1 s 後に `('C:perception_silent',)` を確かめる | 11-27：16:03:19（取消の後）、16:15:43 以降 | [T]、作動は [A] |
| D:forward_blocked | `/scan`（sensor QoS）で base_link 前方 \|angle\| ≤ 30°、1.0 ≤ r ≤ 2.0 m の有限値が **1 点以上**（:85-96, :472-494）→ 最後の遮断から 0.5 s 未満の間作動（:523-525） | test_failsafe_rescue.py `test_default_zero_and_no_subscription`：正面 1.5 m の 1 点でゼロ 1 回、0.51 s 後に沈黙（純ロジック）。扇形の境界（30°、1.0/2.0 m）のテストは main に無い（`test_forward_zone.py` は未マージブランチのみ） | 11-27：drive03 で遮断 758/878 scan、drive04 で 376/589（README）、ENGAGED D 単独 run1 13・run2 8（bringup.log）。2026-07-19：#102 コメント「preflight PASS (PR #107)」「走行中 ENGAGED 0 回」（Issue コメントのみ）。2026-07-22 seimon-stall README：bag2 で 819 発報（手動退避中）。停止距離・反応時間の記録はない | [T]（一部）、作動は [A] |
| D 点数閾値の根拠 | 1（4f908584 2026-07-19） | — | band_probe 149 scan で ≥3 点は 6/149（ADR-0007:490-504、failsafe_node.py:88-94）。band_probe のスクリプト・データはリポジトリに無い | [C] |
| DEAD INPUT 警告 | 起動 10 s 後に未受信の B/C/D 入力をログ ERROR 1 回（:104, :547-564）。停止はしない | なし（scripts/tests/test_preflight.py は preflight スクリプトを偽 ros2 で検査する純ロジック） | 11-27 の両起動で `DEAD INPUT after 10s: ['B:/pcl_pose']`（bringup.log） | [B]、出力は [A] |
| never-arrived（#159） | `startup_grace_sec` > 0 のとき、未着の /pcl_pose・/velodyne_points_no_ground・/scan を `*_never_arrived` 層にする。既定 0.0（無効）（:116, :145-157, :528-533） | test_failsafe_never_arrived.py 13 件（純ロジック）：既定で止めない、grace 後に 3 ラベル、初回受信で解除、sim clock 待ち、負・NaN・整数の拒否 | なし（#159 コメント「既定を 10 秒で有効にするかは実機…で決める」） | [T] |
| Layer D 救援中継（#161、ADR-0013 proposed） | `layer_d_rescue_enabled` 既定 false。有効時、作動層が D 単独で teleop が 0.3 s 以内・後退 ≤ 0.2 m/s・\|wz\| ≤ 0.6 rad/s・他成分 0 なら /cmd_vel_safety に中継（:119-142, :178-220, :390-425） | test_failsafe_rescue.py（純ロジック、17 関数＋パラメタ化）：形の判定、D 単独のみ、鮮度、タイミング検査、twist_mux.yaml の timeout/優先度順の照合（`test_shipped_twist_mux_matches_the_assumptions`）、前進はゼロ、時計の戻り等 | なし。whill_safety/README.md:72「実機では未検証」 | [T] |
| D:scan_silent（#193） | **main に無い**。failsafe_node.py:110-112 が「Layer D has NO silence check after its first /scan (pre-existing gap …)」と明記。PR #203（OPEN、fix/193-scan-silent）に `scan_silent_sec`（既定 0.0） | main：なし。ブランチに test_failsafe_scan_silent.py（340 行、PR 本文「新規 29 件」） | なし | [B]（穴の存在）。PR は §4 |
| twist_mux 優先度（safety > teleop > nav） | yaml の値 | `test_shipped_twist_mux_matches_the_assumptions`（設定ファイルを読む純ロジック） | 手動操縦中に safety が勝つ実機記録はない（§2.3） | [T] |
| SAFE_HOLD 3.0 s、jump 検知、`/failsafe_status` | ADR-0007 §2-3 の設計。main の実装に無い（`git grep failsafe_status` → コード 0 件） | — | — | [B]（不在） |
| failsafe_node 自身の停止の検知 | 無い（ADR-0007:160-162「ノード自身のクラッシュを検知する層は持たない」） | — | — | [B] |

### 2.3 文書が明示する未検証事項（main、`git grep` で確認）

1. docs/repository-maintenance.md:13「手動救出中に safety が優先して止まることは、実機の証跡がまだない」
2. docs/repository-maintenance.md:11「safety のゼロ指令も velocity_smoother を通る。停止距離は実機で確かめる」
3. src/whill_dispatch/README.md:42「手動中のLayer D停止は実機で未検証」
4. src/whill_safety/README.md:72（救援中継）「実機では未検証」
5. src/whill_safety/README.md:47（startup_grace_sec）「停止と解除が起動手順のとおりに起こることを実機で確かめてから既定を変える」
6. src/whill_navigation/README.md:30「衝突検知や設定読み込みが有効でも、実機の検知範囲・停止距離を検証したことにはならない」
7. docs/architecture-overview.md:60「速度上限は実測速度・停止距離を保証しない」
8. docs/reports/2026-10-14-dispatch-demo.md:34（上記引用）
9. docs/regression-check.md:36-41：Nav2、Layer C、入力途絶の検出、停止距離は「確かめていない」
10. ADR-0007:587-592（実機バックログ）：救出フロー全体、dead-man の実際の駆動停止レイテンシ、手動操縦中の Layer D 優先
11. ADR-0007:518-529 accepted 化条件（V2/V3、V6.4、30 min 走行で false-trip 0、preflight exit 0）：達成記録なし。ADR-0007 は proposed のまま
12. perception-blind-zone.md 冒頭「実機で低い物を置いて確かめたのはまだない」
13. #191（高さ帯・Layer C/D の再検証）、#156（屋外 2 回目、安全系の回帰）OPEN

### 2.4 関連 Issue / ADR の状態

| 番号 | 状態 | 要点 |
| --- | --- | --- |
| ADR-0007 | proposed | §2 の表（A=Empty、C=/pcl_pose、SAFE_HOLD）は現コードと違う（§6） |
| ADR-0013 | proposed（2026-11-12、採択待ち） | 救援中継。既定無効 |
| #158 | OPEN | smoother OPEN_LOOP の速度段差（Collision Monitor 前提、C3） |
| #159 | OPEN | PR #188 merge 済（既定 0）。既定値の決定待ち |
| #160 | OPEN | Spin とその場旋回。#157 の後 |
| #161 | OPEN | PR #189 merge 済（既定 false）。#193 と実機確認待ち |
| #191 | OPEN、コメント 0 | 高さ帯・Layer C/D の再検証 |
| #193 / PR #203 | OPEN / OPEN | /scan 途絶。PR は既定 0.0、推奨 0.5。実機確認は #156 |
| #102 | CLOSED 2026-09-10（棄却、新リポジトリへ移管と記載） | Layer D 実機受入。完了ではなく棄却 |
| #143 / #157 | OPEN / OPEN | 前端 0.72〜0.75 m のみ実測。後端・幅・出っ張り未測定 |

---

## (3) Perception（whill_perception）

### 3.1 中身 [B]

- ノードは `patchworkpp_node` 1 つ（src/whill_perception/src/patchworkpp_node.cpp、232 行、BSD-3 の自作 wrapper）。上流 Patchwork++ C++ core（src/third_party/patchwork_plusplus/cpp、3e6903a）を直接リンク。sub `cloud_in=/velodyne_points`、pub `cloud_no_ground=/velodyne_points_no_ground`（ground_removal_launch.py:80-83）。入力の header frame をそのまま使い、**座標変換はしない**（velodyne 系で処理）。intensity が無い・空・0/0 の分割のときは空の点群を出して WARN（:160-195）。
- 設定は pw:27-41 の 5 項目のみ（verbose、sensor_height 0.771、min_range 1.0、max_range 25.0、stats_log_period_frames 100）。他は struct の既定。ADR-0011:72-74「VLP-16 で既定値の性能検証は未実施」。
- pointcloud→LaserScan は whill_navigation 側（apt の pointcloud_to_laserscan、nav_launch.py:270-278）。
- **self-filter（車体の点の除去）ノードは無い**。車体対策として書かれているのは Patchwork++ min_range 1.0 と p2ls range_min 0.5 の 2 つ。
- ADR-0011：accepted（2026-07-14、bag replay で AC1-AC3）。sensor_height の記述は 0.79 のまま（ADR-0011:72、現値 0.771）。ADR-0009：accepted（2026-07-15 A/B）。

### 3.2 Patchwork++ の min_range の実際の挙動（コードから）[B]

上流 core `patchworkpp.cpp:603-628`（`pc2czm`）：`r = xy2radius(x,y)` が `(r <= max_range) && (r > min_range)` の点だけ同心ゾーンに入れ、**それ以外（r ≤ 1.0 m と r > 25 m）は `cloud_nonground_.push_back(...)`**（:627）。`getNonground()`（patchworkpp.h:164）がそれを返し、wrapper が publish する（patchworkpp_node.cpp:184）。つまり LiDAR から水平 1.0 m 以内の点は捨てられず「非地面」として /velodyne_points_no_ground に出る。これは §6 の文書の記述と食い違う。1.0 m 以内に実際に点がどれだけ出ているかの測定は見つからない（`scripts/regression/patchwork_height_compare.py` は 1 < r ≤ 25 m だけを数える、src/whill_perception/README.md）。

### 3.3 低い物の死角（docs/perception-blind-zone.md、#166、PR #199 2026-11-16 merge）

- 使った値：**新 TF**（z 0.771、pitch +0.40°、roll −0.05°、x 0.42、y 0.36）、`VLP16_hires_db.yaml` の仰角 ±10°、D435 は (0.47, 0.378, 0.73)、回転 0（§1 表）。旧 TF ではない。
- 主な数値（計算）：最下 ring は前方で LiDAR から 4.20 m で床に当たる（後方 4.56 m）。Layer D 扇形内で LiDAR が見える最も低い高さ 0.43〜0.69 m（「高さ約 0.43 m 未満の物は停止域内で LiDAR に映らない」）。高さ 20 cm の物は LiDAR から 3.11 m（正面 base_link 約 3.5 m）より近いと LiDAR から消える。D435 は高さ 20 cm を光軸方向 0.96 m（base_link 約 1.43 m）より手前で見ない。base_link 1.0〜1.4 m はどちらにも映らない。**[C]（計算）**
- 実測で確かめた部分：2026-11-16 屋外コーン 7 本の LiDAR 点の最低高さ z_min が予測 h_min と 1.5〜4.0 m で 0.9〜1.3 cm 低く一致（docs/calibration/2026-11-16-outdoor-static/results.json、リポジトリ内）。**[A]**（コーン＝高 0.725 m。低い物では未測定）
- 未確認（同 §6）：低い物の実測、Patchwork++ が遠方の低い物を地面と判定しないか、2D obstacle_layer が近づいたときに低い物のセルを消すか、着座での z。§5 の扱い（a/b/c、推奨 b）は「提案で、未決定」。
- 古い記述：§6「sensor_height は 0.79 m のまま」→ #201（同日 15:55、#199 の 8 分後）で 0.771 に変更済み。

### 3.4 測定 vs 仮定の整理

| 項目 | 状態 | 区分 |
| --- | --- | --- |
| LiDAR z・pitch・x | 2026-11-13 屋内、2026-11-16 屋外コーンで許容内（台帳 §2.1、#149） | [A] |
| LiDAR yaw | 未測定（−0.317° は noetic の流用） | [D] |
| 着座の pitch −0.75° | 1 回のみ（台帳 §2.1 B） | [A]（1 回） |
| 着座の z | 「不安定で使えない」（blind-zone §2.3） | [D] |
| 停止域の死角の高さ | 計算 | [C] |
| Patchwork++ の地面除去（床の漏れ 9.1〜10.6 %、物の保持 82.4〜88.1 %） | bag 再生の結果を README に記載、結果ファイルなし | [C] |
| 車体の自己反射の範囲（0.5 m／1.0 m） | 測定記録なし。ADR-0007:508-512 自身が「矛盾する、実測して」と backlog に書く | [D] |
| 2026-11-27 で Layer D が反応した点の高さ 0.55 / 0.66 m | README の解析（stop_probe.py） | [A]（README の解析値） |

---

## (4) 未マージのブランチ

基準：`git branch -a --merged main`、`git rev-list --count main..<b>`。内容は**現行ではない**。

| ブランチ | merged? | main..b | 最終 commit | 分岐点 | 中身（1〜2 行） | LiDAR 修正（11-13）より前に開始？ |
| --- | --- | --- | --- | --- | --- | --- |
| feat/avoidance-baseline（origin も同じ） | 未 | 7 | f7b2de0 2026-10-30 | dfbf86d 2026-10-15 | C0：failsafe の全閾値を ROS パラメータ化（F1）、`layer_d_shape` sector/rectangle（F2）、`extra_perception_topics`（F3）、`/failsafe/state`（F4）、test_forward_zone.py、realsense.yaml、camera TF、C1 の記録・校正ツール。計画書・記録スクリプトの一部は既に main へ移植済み（計画書冒頭の注記） | 前。roadmap は「載せ直す」（#162 #163） |
| feat/avoidance-c2 | 未 | 11 | 52457ef 2026-10-30 | dfbf86d | baseline ＋ STVL（`nav2_avoidance_stvl.yaml`、`avoidance:=true` 時）、`perception_3d_launch.py`、`cloud_height_filter_node.cpp`、`patchworkpp_hires.yaml`（sensor_height **0.777**、`frame_id_override: velodyne_hires`＝旧 TF を残したまま Hi-Res の並行フレームを作る設計） | 前。計画書 §2 の注記「velodyne_hires の二重化は不要になった可能性が高い」 |
| feat/avoidance-c3 | 未 | 9 | 9b22443 2026-10-30 | dfbf86d | baseline ＋ Collision Monitor を smoother の後ろ（`collision_monitor.yaml`：PolygonStop [0.80,±0.45]、PolygonSlow [1.65,±0.65] ratio 0.5、FootprintApproach 円 0.45 m・2.0 s、入力 `/velodyne_points_hires_no_ground` と D435 点群、min_height 0.10）、Layer D 矩形を `avoidance:=true` に接続、07-24/07-22 bag の作動解析、test_collision_monitor_config.py、test_safety_launch_params.py。#163 コメント「実機の停止距離は未検証」 | 前。footprint 0.45 前提（#157 で決め直し） |
| integration/186-190 | 未（merge commit 2 本のみ） | 2 | b74bdf2 2026-11-13 | f35fe86 | #186 と #190 を統合した merge commit だけ。両親（#186・#190 の先端）はともに main の祖先（`git merge-base --is-ancestor` で確認）。固有の非 merge commit は無い | 修正そのもの |
| fix/193-scan-silent（origin も同じ、PR #203 OPEN） | 未 | 3 | 4d59cce 2026-11-16 | 2aa6c1c 2026-11-16 | `scan_silent_sec`（既定 0.0、推奨 0.5）で D:scan_silent、純関数 `silence_step()`、launch 引数 `failsafe_scan_silent_sec`、replay_check.py の `--failsafe-param`、ADR-0007/0013 追記、テスト 29 件（PR 本文） | 後 |
| fix/159-failsafe-never-arrived-inputs | **merged**（PR #188） | 0 | 90c29a0 2026-11-12 | — | main に入っている | — |
| fix/161-rescue-while-stopped | **merged**（PR #189） | 0 | 4ae62a2 2026-11-13 | — | 同上 | — |
| fix/192-patchwork-sensor-height | **merged**（PR #201） | 0 | 554be41 2026-11-16 | — | 同上 | — |
| docs/166-low-object-blind-zone | **merged**（PR #199） | 0 | 39b5af3 2026-11-16 | — | 同上 | — |

- avoidance 3 本は main に対して 130 commit 遅れ。3 本とも旧 TF（pitch +8.99°）・VLP16db の main から分岐。
- 食い違い：#163 の 2026-11-02 コメント「HEAD 9b22443、未 push」だが、現在 `origin/feat/avoidance-c3` は 9b22443 で存在する。**[D]**
- 補足：`.claude/worktrees/` 配下に多数の worktree（avoidance-c2、avoidance-c3、footprint-143、outdoor-149、m8-integration、agent-* 等）がある。本監査では中身を評価していない。

---

## (5) 使われていない設定（main）

grep の範囲：`git grep` を main の `src/`、`scripts/`（tests 除く）、`*.py`/`*.sh`、`*.js` に対して実行。

| 項目 | 場所 | 「使われていない」の根拠 | 区分 |
| --- | --- | --- | --- |
| behavior_server の `spin`・`backup` プラグイン | np:334-338 | 既定の NavigateToPose BT（nav_launch.py:226-227 で設定）BT:82-109 には `<Wait>` と `<ClearEntireCostmap>` しかない（`git grep "<Spin\|<BackUp"` は BT の XML コメント以外 0 件）。dispatch は NavigateToPose だけを使う（src/whill_dispatch/whill_dispatch/dispatch_node.py:67, :215）。`NavigateThroughPoses` の client は src/scripts に 0 件。ただし `default_nav_through_poses_bt_xml` は未設定なので、through-poses の goal が送られた場合は Nav2 の既定 BT が使われる。その既定 BT が Spin/BackUp を含むかはローカルで確認していない | [B]（through-poses は [D]） |
| behavior_server の `simulate_ahead_time` 2.0、`max/min_rotational_vel`、`rotational_acc_lim` | np:344-347 | spin/backup 用。上と同じ理由で NavigateToPose 経路では呼ばれない | [B] |
| bt_navigator `plugin_lib_names` の spin/back_up/drive_on_heading/assisted_teleop 等 | np:42-89 | カスタム BT は使わない。コメント（np:36-41）は「削ると既定 BT の読み込みで落ちる」として nav2_bringup と同じ一覧を残す方針 | [B]（意図的に残置） |
| RPP `max_linear_accel` 0.3 | np:138 | コメント自体が「documentation only」 | [B] |
| RPP `cost_scaling_dist` 0.3 / `cost_scaling_gain` 1.0 | np:202-203 | `use_cost_regulated_linear_velocity_scaling: false`（np:187）。Nav2 RPP ではこの 2 つはコストによる減速にだけ使われる（Nav2 の仕様からの判断。ローカルのソースでは未確認） | [B] |
| smoother `min_velocity` −0.1 のコメント「small reverse band for backup recovery」 | np:371 | BackUp は BT に無い。値自体は救援 teleop の後退（ADR-0013 背景：後退 −0.2 → smoother で −0.1 に制限）で使われるので、値は死んでいない。コメントの理由が古い | [B] |
| Collision Monitor | — | main の src/scripts に `collision_monitor` は 0 件（`git grep -n "collision_monitor\|CollisionMonitor" main -- src scripts`）。`feat/avoidance-c3` にのみ存在 | [B]（不在） |
| `/failsafe_status`・`/failsafe/state` | — | コードに 0 件（README の言及のみ） | [B]（不在） |
| 未参照の yaml | — | 本トラックの全 yaml/xml（nav2_params、p2ls、BT、patchworkpp、twist_mux、m6r_lidar_localization）は launch から参照されている（nav_launch.py:187-189, :226、ground_removal_launch.py:27、safety_launch.py:22、m6r_bringup_launch.py:144）。未参照ファイルは見つからない | [B] |
| 既定で無効の機能 | `layer_d_rescue_enabled` false、`startup_grace_sec` 0.0 | 死んではいないが既定構成では動かない（2026-11-27 起動ログで確認） | [B]、[A] |

---

## (6) 文書とコードの食い違い

| # | 文書側（原文） | コード／他文書側 | 区分 |
| --- | --- | --- | --- |
| 1 | failsafe_node.py:73-77「Patchwork++ filters points closer than 1.0 m as WHILL body self-return … so /scan is silent below 1.0 m by design」。ADR-0007:295「0.5-1.0 m の点は上流が self-return として捨てるため /scan に元々来ない」、:479-484（Fix 4）。ADR-0013:78「この範囲（1 m 未満）は Patchwork++ の `min_range: 1.0` により `/scan` に元々点が無く」 | 上流 core patchworkpp.cpp:603-628：r ≤ min_range の点は `cloud_nonground_` に入れて出力する（捨てない）。さらに Patchwork++ の r は LiDAR 原点基準、Layer D・/scan の r は base_link 原点基準（LiDAR は base_link の x 0.42・左 0.36 m）で、基準点が違う。blind-zone §3.1 は扇形の角が LiDAR から 0.47 m と算出 | [D] |
| 2 | ADR-0009 §3「p2ls の publish `/scan`: **reliable** … QoS override は不要」、nav_launch.py:66「/scan (reliable)」、:71-73、:265-269 | ADR-0007:404-415（2026-07-16 incident）「p2ls は `/scan` を **BEST_EFFORT** で publish（`ros2 topic info /scan --verbose` … で実証）」、failsafe_node.py:283-291 | [D] |
| 3 | ADR-0007 §2 表：A=`std_msgs/Empty`、C=`/pcl_pose`（PoseStamped）、`SAFE_HOLD_S` 3.0、`/failsafe_status` 1 Hz | コード：A=Bool data=True（:262, :430-434）、C=/velodyne_points_no_ground、/pcl_pose は B（PoseWithCovarianceStamped）、SAFE_HOLD と /failsafe_status は無い（docstring :43-47 が「意図的に省略」と記す） | [D]（ADR の本文が古い） |
| 4 | ADR-0007:395 運用ゲートの期待値「forward_blocked > 5 pts in ±30° @ 0.5-2.0 m」 | 実際の起動ログ（2026-11-27）「forward_blocked > 1 pts in ±30° @ 1.0-2.0 m」。またログ文は「>」だが判定は `count >= FORWARD_POINT_COUNT_MIN`（:491） | [D] |
| 5 | ADR-0007:309-321「movement_time_allowance: 10.0 s」「recovery (spin / backup / wait)」 | np:110 は 15.0、BT は Spin/BackUp を除去 | [D] |
| 6 | np:362-367「rotate_to_heading_angular_vel stays 0.8」、np:165「progress checker の 10 s 猶予内」 | np:166 は 0.4、np:110 は 15.0 | [D]（コメントが古い） |
| 7 | np:227-230「1.0 m long × 0.6 m wide; a 0.45 m circle conservatively covers it」 | 台帳 §1：車軸→前端 0.72〜0.75 m（実測）、全幅 0.554 m（公式）。#157「このコメントは誤り」 | [D] |
| 8 | p2ls:31「velodyne sits ~0.9 m above base_link」 | TF z 0.771（static_tf_launch.py:86） | [D] |
| 9 | p2ls:34-42「range_min 0.5 m avoids WHILL chassis self-return (within 0.5 m of the LiDAR origin)」、ADR-0009 §1 同旨 | target_frame base_link のため距離は base_link 原点から（blind-zone §1 の表も同旨）。ADR-0007:508-512 も「pointcloud_to_laserscan.yaml の range_min 0.5 の comment … と矛盾する」と記す | [D] |
| 10 | ADR-0009 §検証結果 2026-07-15「人 (立位、前方 2 m) の脚捕捉数 ~4 点常時」 | ADR-0007:490-504（2026-07-19）「人が帯内に立っても >= 3 点になるのは 6/149 のみ … ADR-0009 の『脚 ~4 点』自体が成立せず」。#102 コメント「/scan 段階では 0-3 点まで痩せる (129 bin)」。一方 p2ls の角度設定（0.008726646 rad、±π）からは 720 bin になる（計算）。「129 bin」の出所は確認できない | [D] |
| 11 | perception-blind-zone.md §6「`patchworkpp.yaml` の sensor_height は 0.79 m のまま」、ADR-0011:72「0.79 m」 | pw:32 は 0.771（#201、2026-11-16） | [D]（古い） |
| 12 | 台帳 §6（p2ls 帯・Layer C/D の行）「担当 Issue なし（起票案あり）」 | #191 が 2026-11-13 に起票済み（M8-3） | [D]（古い） |
| 13 | 台帳 §1・§6「failsafe_node.py:59（車幅 0.6 m）」「:72（半角 30°）」 | 現在の該当行は :71-72（コメント）、:85（半角） | [D]（行番号ずれ） |
| 14 | ADR-0007 Layer D 表「WHILL 幅 0.6 m」 | 公式全幅 0.554 m（台帳 §1）、後付け物込みは未測定 | [D] |
| 15 | nav_launch.py:77-78「`use_collision_detection` stays false through M6R4-c and flips to true in M6R4-3」 | np:184 true（歴史的記述として残存） | [B]（古い記述） |
| 16 | 2026-11-27 README「Failed to make progress ×5」「collision ahead ×5」 | nav2.log：run1 6 回、run2 collision ahead 6 回 | [D]（数え方） |
| 17 | #163 コメント「未 push」 | `origin/feat/avoidance-c3` が存在 | [D] |

---

## (7) 判断できないこと

1. 停止距離・停止までの時間（slow/normal/fast、Layer A〜D のどれでも）。測定がない。
2. 手動操縦中に safety（Layer D）が teleop に勝って止まるか（実機）。
3. 新 TF・Hi-Res 校正表での p2ls 高さ帯 0.05〜1.6 m、Layer D の点数閾値 1、扇形 ±30°・1.0〜2.0 m が妥当か（#191 未実施、着座の bag なし）。2026-11-27 の走行は扇形が静的構造物で停止したことを記録しているが、閾値の妥当性の検証ではない。
4. LiDAR から 1.0 m 以内の点（Patchwork++ が非地面として出す点）が実際に /scan・Layer D に入っているか、車体・乗員の自己反射が出るか。
5. p2ls の /scan の QoS（reliable か best-effort か）。文書が食い違い、インストール済みバイナリのソースは確認していない。
6. 「129 bin」の出所と、band_probe（2026-07-19）の生データ。リポジトリに無い。
7. ADR-0009 の 2026-07-15 A/B と 2026-07-14 Phase B の生データの所在（ADR は 2026-07-14 verify-campus の manifest を参照するが、その manifest は localizer の受入で、p2ls の A/B の数値を含まない）。
8. Nav2 既定の through-poses BT の中身（Spin/BackUp を含むか）。ローカルで確認していない。
9. failsafe 再生回帰の基準（07-24 bag の旧 TF の /scan）が新構成でどう変わるか。新構成の基準 bag がない（#156 で取得予定）。
10. 2026-11-27 README の解釈（構造物が原因、矩形なら通過できる、自己位置は合っている）の独立検証。本監査では bag を開いていない（指示により）。
11. LiDAR yaw（未測定）が Layer D の扇形の左右と footprint の評価に与える影響。
12. 2026-11-27 の両起動で起動 10 s 後に出た `DEAD INPUT ['B:/pcl_pose']` の原因（初期位置の指定前か等）。ログからは判断していない。

---

## (8) 実際に読んだもの

**ファイル（main 64f7a49）**：src/whill_navigation/launch/nav_launch.py、config/nav2_params.yaml、config/pointcloud_to_laserscan.yaml、config/navigate_to_pose_no_spin_bt.xml；src/whill_safety/whill_safety/failsafe_node.py、config/twist_mux.yaml、launch/safety_launch.py、launch/m6r_bringup_launch.py（grep）、test/test_failsafe_rescue.py、test/test_failsafe_never_arrived.py、CMakeLists.txt；src/whill_perception/config/patchworkpp.yaml、launch/ground_removal_launch.py、src/patchworkpp_node.cpp、README.md、CMakeLists.txt（grep）；src/third_party/patchwork_plusplus/cpp/patchworkpp/src/patchworkpp.cpp（:121-183, :585-628）、include/patchwork/patchworkpp.h（:164）；src/third_party/lidar_localization_ros2/src/lidar_localization_component.cpp（:970-980 grep）；src/whill_sensors_bringup/launch/static_tf_launch.py；src/whill_dispatch/whill_dispatch/dispatch_node.py（grep）；scripts/map_widen_hand_roadway.py（:540-555）、scripts/nav2_plan_check.py（grep）、scripts/m6r_preflight.sh（冒頭）、scripts/tests/*（grep）。
**文書**：docs/decisions/0007（:1-175, :236-601）、0009（全体の主要部）、0011（:1-8, :66-80）、0013（:1-20）；docs/perception-blind-zone.md（全文）；docs/calibration-ledger.md（:36-60, :70-82, :138, :228-280）；docs/plans/2026-10-30-roadmap-m8.md（全文）、2026-10-28-avoidance-baseline.md（:1-40 と grep）；docs/regression-check.md（:1-60）；docs/reports/2026-10-14-dispatch-demo.md；docs/m6r-bench-data/2026-07-14-verify-campus/manifest.yaml；docs/m7-bench-data/2026-07-22-live-campus/seimon-stall/README.md；README.md・docs/repository-maintenance.md・各 README の grep 該当行。
**リポジトリ外**：~/bags/2026-11-27-m8-drive/README.md、logs/*/nav2.log（grep）、logs/*/bringup.log（failsafe 行の grep）。bag 本体は開いていない。
**Issue / PR（gh）**：#101 #102 #108 #143 #144 #145 #149 #156 #157 #158 #159 #160 #161 #163 #166 #191 #192 #193（本文とコメント）、PR #203（本文・commit）。
**git**：`git log --merges`、`git log -L`（static_tf_launch.py velodyne）、`git blame`（nav2_params.yaml :91-377、pointcloud_to_laserscan.yaml、patchworkpp.yaml、failsafe_node.py :64-142、twist_mux.yaml）、commit 262772fa 9370f8ed 8cd7fe5b 25d9b993 645332f5 bb0666c4 b77804d3 d8ab2664 6ba656bc 1fe651e3 93655fb9 5c8162a4 e8241b05 554be414 0b9085e6 ffad5276 dc567cf2 4f908584 13a81ecb 13ee1310 79391774 64941456 94e442a fc7347a、ブランチ比較（§4）。
