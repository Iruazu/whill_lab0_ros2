# Plan: キャンパス内自律走行スタックの ROS 2 humble 移植

## ユーザー要件の理解

旧 noetic `~/whill_lab0/` の学内自律走行スタック (loader_kiban + slam_localization
+ pedestrian_flow_navigator + autoware_tracker) を、本リポの Nav2 標準スタックに
**作り直す**。盲目的コピーではなく、自前 A\* / Pure Pursuit / PCL NDT は捨てて
Nav2 標準 plugin + 現代 OSS (FASTLIO2_SAM_LC, patchwork++, ndt_omp,
social_costmap_plugin) で再構成する。

範囲は「ループクロージャ付き LIO + 事前点群地図リローカライゼーション + 車輪
オドメ統合 + 歩行者反映 costmap + Nav2 planner 切替」までの一式。M5-e / M6 の
基盤となる。

## 解決すべき問題

1. **FAST-LIO 単体で 60 秒 18% ドリフト**。長距離 (キャンパス 600 m スケール)
   では破綻する。旧 noetic は PCL NDT (`slam_localization/`) で `map -> odom` を
   補正していたが、本リポにはこの層が無い。
2. **車輪オドメトリ未統合**。M2 で動いている `/whill/odom` が Nav2 に流れていない
   ため、`map -> odom -> base_link` という Nav2 標準 TF tree が崩れている (現状
   は `map -> camera_init` identity でごまかしている)。
3. **動的障害物が costmap に反映されない**。M5-b のゴースト障害物対策で
   `use_collision_detection: false` にしているため、歩行者を含む動的物体に対し
   無防備。
4. **Nav2 の global planner が NavFn のまま**。600 m × 600 m 級のマップで
   wavefront 展開が遅くパス品質も劣る。
5. **旧スタックの「学内自律走行」相当機能が無い**。ゴール送信 → 経路計画 → 追従
   → 歩行者回避 の一連が、現状の `whill_navigation` (M5-a〜c) では
   小スケール (数 m) でしか実証されていない。

## 前提条件

- 旧スタック調査は `docs/legacy-findings/campus-autonomous-navigation.md` を
  一次ソースとする (2026-05-28 に保存済み)。ユーザー承認により Phase 0 は **skip**。
- 技術選定は `docs/research/2026-05-28-campus-autonomous-navigation-tech-survey.md`
  に従う。SmacPlanner2D / RPP 継続 / FASTLIO2_SAM_LC + ndt_omp / patchwork++ +
  lidar_cluster_ros2 / nav2_social_costmap_plugin。
- M3 (sensors), M4 (FAST-LIO 単体), M5-a〜c (Nav2 lifecycle 起動) は完了済み。
- M5-d (`velodyne_self_filter`) は本計画の前提として完了している扱い (本ブランチで進行中)。
- 旧 loader_kiban の A\* / Pure Pursuit / 独自 OccupancyGrid 生成は移植**しない**。
  Nav2 標準で代替する。
- 旧 pedestrian_flow_navigator の Potential Field + LJ 斥力ロジックは移植**しない**。
  歩行者は costmap layer に反映する方針に統一する (Nav2 controller 側で吸収)。
- 旧 autoware_tracker (IMM-UKF-PDA) は移植**しない**。検出は patchwork++ +
  クラスタリングで代替し、トラッキングは Nav2 obstacle layer の time-decay に
  任せる。proxemics が必要になった時点で `nav2_social_costmap_plugin` 追加。
- 事前点群地図は M5-b で作成済みの PCD を再利用する (再キャプチャはゴースト
  障害物問題と一緒に Phase B で実施)。
- 旧スタックのハードコード値 (ゴール (199.4, 311.4) など) は再利用しない。
  ゴールは Nav2 BT navigator + RViz / Nav2 waypoint follower で外部化する。

## 受け入れ基準 (Acceptance Criteria)

完了したと言えるための観測可能な条件:

- [ ] `ros2 launch whill_bringup campus_autonomous.launch.py` 一発で
      sensors + localization + navigation + perception が立ち上がる
- [ ] TF tree が Nav2 標準 (`map -> odom -> base_link`、`map -> odom` は LIO/SAM、
      `odom -> base_link` は車輪オドメ) になっている
- [ ] 600 m スケールの事前点群地図でリローカライゼーション成功
      (`/slam_reloc` サービス成功 + 初回 fix から 30 秒以内に Nav2 が active)
- [ ] 200 m 以上の直線/カーブ混在経路を SmacPlanner2D で 5 秒以内に計画
- [ ] 歩行者 1 体が経路上を横断するシナリオで、costmap に歩行者が現れて
      停止 or 回避できる
- [ ] FAST-LIO 単体時の 60 秒 18% ドリフトが、ループクロージャ + 車輪オドメ
      統合で 5% 以下に改善
- [ ] 既存 M5 acceptance (5 m / 10 m 短距離ゴール) が回帰しない
- [ ] ADR が 3 件作成され accepted ステータス
- [ ] `docs/legacy-findings/campus-autonomous-navigation.md` が成果物として残る

## Phase 分解

phase は依存関係順 (A → F)。各 phase 単独で merge 可能。

```
Phase 0 (legacy 整理)
  └─ Phase A (車輪オドメ + EKF)
       ├─ Phase B (FASTLIO2_SAM_LC + 地図リローカライゼーション)
       │    └─ Phase C (SmacPlanner2D + 二層 costmap)
       │         └─ Phase D (歩行者検出 → costmap)
       │              └─ Phase E (social costmap)
       │                   └─ Phase F (統合 launch & 実機検証)
```

---

### Phase 0: 旧実装の正式な archeology 成果物化

- **目的**: context で受け取っている旧スタック情報 (loader_kiban / slam_localization
  / pedestrian_flow_navigator の構造・既知バグ・地雷) を `docs/legacy-findings/`
  に正式ファイル化し、以降の phase が参照可能な単一ソースにする
- **担当 agent**: `legacy-archaeologist`
- **入力**: `~/whill_lab0/loader_kiban/`, `~/whill_lab0/slam_localization/`,
  `~/whill_lab0/pedestrian_flow_navigator/`, `~/whill_lab0/autoware_tracker/`
- **出力**:
  - `docs/legacy-findings/campus-autonomous-navigation.md` (機能ブロック・トピック
    フロー図・既知バグ・地雷の一覧)
  - `docs/legacy-index.md` の「既に詳細調査済み」セクションに追記
- **検証方法**: 以下の質問に成果物だけで答えられること
  - mapping_node ↔ path_planning_node の型不整合の正確な場所と影響範囲
  - loam_velodyne 依存 (`/integrated_to_init`) の置換ポイント
  - Joy axes 食い違いの現物確認 (axes[0]=旋回 / axes[1]=直進 の根拠)
  - 旧 TF tree の欠落範囲 (どこからどこまでが無いか)
- **触るパッケージ**: なし (docs のみ)
- **リスク**: 旧リポが整理されていないため調査時間が嵩む
  - **フォールバック**: M5-e ブロッカーになる情報 (車輪オドメと FAST-LIO の
    時刻同期方法、TF root frame) のみに絞って先行公開
- **工数**: S (1-2 日)
- **依存先**: なし

---

### Phase A: 車輪オドメトリ統合 + robot_localization EKF

- **目的**: `/whill/odom` (M2 で稼働中) と FAST-LIO odometry を `robot_localization`
  の EKF で融合し、Nav2 標準の `map -> odom -> base_link` TF tree に正式移行する。
  これは旧 M5-e の積み残しと完全に重なる本計画の足回り
- **担当 agent**: `ros2-implementer` (実装) → `code-reviewer` (レビュー)
- **入力**: Phase 0 の成果物、`whill_localization/`, `whill_navigation/launch/tf_bridge_launch.py`
- **出力**:
  - 新パッケージ `whill_state_estimation/` (または `whill_localization/` 内に
    config 追加。ADR-0001 で決定)
  - `robot_localization` の `ekf.yaml` 2 個
    - `ekf_odom`: `/whill/odom` + IMU → `odom -> base_link`
    - `ekf_map`: `ekf_odom 出力` + FAST-LIO Odometry → `map -> odom`
  - 現行の `tf_bridge_launch.py` (identity `map -> camera_init` を吐いている層)
    を削除 or 無効化
- **主要パラメータ初期値**:
  - `ekf_odom.frequency: 30`
  - `ekf_odom.two_d_mode: true` (車椅子は平面前提)
  - `ekf_odom.odom0`: `/whill/odom` (vx, vyaw を信頼)
  - `ekf_odom.imu0`: `/imu/data_raw` (yaw rate のみ信頼、加速度は捨てる)
  - `ekf_map.odom0`: `ekf_odom` 出力 (vx, vyaw)
  - `ekf_map.odom1`: `/Odometry` (FAST-LIO の絶対位置 x, y, yaw)
  - `ekf_map.transform_time_offset: 0.05` (時刻同期マージン)
- **検証方法**:
  - `ros2 run tf2_tools view_frames.py` で `map -> odom -> base_link` チェーンが
    一本に通る
  - 静止状態で `map -> base_link` が固定 (FAST-LIO 単体時の odom 揺らぎが消える)
  - 60 秒ドライブで `odom -> base_link` の積算と `/whill/odom` 積算が
    1% 以内で一致
- **リスク**:
  - `/whill/odom` と FAST-LIO の時刻同期 (sim_time vs system_time) で TF
    extrapolation エラー
    - **フォールバック**: `ekf_map` から FAST-LIO を一旦外して `ekf_odom` 単体で
      `odom -> base_link` だけ確立し、`map -> odom` は当面 identity に戻す
  - `/whill/odom` covariance が WHILL SDK で書かれていない可能性
    - **フォールバック**: `ros2_whill` 側で covariance を経験値 (vx: 0.05^2,
      vyaw: 0.1^2) で上書きするラッパーノードを書く
- **工数**: M (3-5 日)
- **依存先**: Phase 0
- **生成 ADR**: ADR-0001 (EKF パッケージ配置と二段 EKF 構成)

---

### Phase B: FASTLIO2_SAM_LC への置換 + 事前点群地図リローカライゼーション

- **目的**: 現行 `whill_localization` の FAST-LIO を liangheming 版
  `FASTLIO2_SAM_LC` (GTSAM ループクロージャ + `/slam_reloc` ICP relocalization)
  に置換し、`map -> odom` を pose graph 出力で駆動。事前点群地図に対する初期化
  も同じ stack で行う
- **担当 agent**: `ros2-implementer` (実装) → `debugger` (TF/時刻同期) →
  `code-reviewer`
- **入力**: Phase A の EKF 構成、M5-b の事前点群 PCD、`whill_localization/config/velodyne_whill.yaml`
- **出力**:
  - `src/third_party/FASTLIO2_SAM_LC/` (`whill_lab.repos` に追加)
  - `whill_localization/config/fastlio2_sam_lc.yaml` (extrinsic, GTSAM パラメータ,
    loop closure search radius)
  - `whill_localization/launch/fastlio2_sam_lc_launch.py`
  - 事前点群地図の格納場所規約: `maps/<env>/cloud.pcd` + `maps/<env>/origin.yaml`
  - 起動シーケンス: bringup → `/slam_reloc` 自動 call (初回 fix が降りるまで
    Nav2 lifecycle inactive)
  - Phase A の暫定物の撤去:
    - `state_estimation_launch.py` 内の `map -> camera_init` identity static TF
      (`tf2_ros/static_transform_publisher`) を削除。FASTLIO2_SAM_LC が `map`
      フレームに直接 publish するようになるため不要
    - `whill_navigation/launch/tf_bridge_launch.py` 本体と、
      `nav_launch.py` からの `IncludeLaunchDescription` を削除
- **主要パラメータ初期値**:
  - `loop_closure.search_radius: 5.0`
  - `loop_closure.icp_fitness_threshold: 0.3`
  - `relocalization.initial_guess`: param で初期 pose 指定可能に
  - `map_frame: map`, `odom_frame: odom`, `body_frame: base_link`
    (Phase A の TF tree と整合させる。FAST-LIO 既定の `camera_init` は使わない)
- **検証方法**:
  - 既存 M5-b 環境で `/slam_reloc` 成功率 5 回中 5 回
  - 同じ 60 秒ドライブで FAST-LIO 単体ドリフトと比較し、ループクロージャ有効時
    に終端誤差が 1/3 以下
  - リローカライズ後の `map -> odom` jump が 30 cm 未満
- **リスク**:
  - GTSAM の依存解決でビルド失敗 (旧 noetic 系 PCL/Eigen バージョン衝突)
    - **フォールバック**: GTSAM を apt の `libgtsam-dev` から取らずソースビルド
      して isolated install
  - ループクロージャが「位置近傍 ICP」のため開放経路 (キャンパス長い直線) で
    効かない (research 文書の論点3 地雷)
    - **フォールバック**: 段階 B (research 推奨) の `lidar_localization_ros2`
      (ndt_omp pure localization) に Phase B+ として後続切替
  - リローカライゼーション中の `map -> camera_init` 旧 identity 接続が残ると
    TF が二重定義になる
    - **フォールバック**: Phase A で削除を完了させてから Phase B に入る厳格な順序
- **工数**: L (5-8 日)
- **依存先**: Phase A
- **生成 ADR**: ADR-0002 (LIO スタック選定: FASTLIO2_SAM_LC 採用)

---

### Phase C: SmacPlanner2D 切替 + 二層 costmap

- **目的**: Nav2 global planner を NavFn から SmacPlanner2D に切替え、global
  costmap を 10-20 cm/pixel の低解像、local costmap を 5 cm/pixel の高解像に
  分離する。600 m スケール対応
- **担当 agent**: `ros2-implementer` → `code-reviewer`
- **入力**: Phase B の `map -> odom` が安定動作する状態、`whill_navigation/config/nav2_params.yaml`
- **出力**:
  - `whill_navigation/config/nav2_params.yaml` 改修
  - 大マップ用の `whill_navigation/config/nav2_params_campus.yaml` 別ファイル
    (lab 用と campus 用を環境変数で切替)
- **主要パラメータ初期値**:
  - `planner_server.GridBased.plugin: "nav2_smac_planner::SmacPlanner2D"`
  - `planner_server.GridBased.max_iterations: 10000000` (大マップ向け増量)
  - `planner_server.GridBased.tolerance: 0.5`
  - `global_costmap.resolution: 0.15` (キャンパス用)
  - `local_costmap.resolution: 0.05` (現状維持)
  - `global_costmap.rolling_window: false`
  - `local_costmap.rolling_window: true`
  - `planner_server.expected_planner_frequency: 1.0` (大マップでも 1Hz は出す)
- **検証方法**:
  - 200 m 経路で計画時間 5 秒以内 (NavFn 比較)
  - 既存 M5-c の 5 m / 10 m goal で回帰しない (`bt_navigator: Goal succeeded`)
  - global costmap のメモリ使用量が 1 GB 未満
- **リスク**:
  - 大マップで `max_iterations` 不足
    - **フォールバック**: SmacPlanner2D を `ThetaStarPlanner` に切替 (ADR-0003 内で
      代替案として明記)
  - 低解像 global costmap で細い通路 (1m 幅以下) が膨張で塞がる
    - **フォールバック**: `inflation_radius` を local costmap だけ手厚くし、
      global は最小限にする
- **工数**: M (2-4 日)
- **依存先**: Phase B
- **生成 ADR**: ADR-0003 (Global planner 選定: SmacPlanner2D 採用 + 二層 costmap)

---

### Phase D: LiDAR 歩行者検出 → costmap obstacle layer

- **目的**: VLP-16 点群から地面除去 (patchwork++) + Euclidean clustering
  (lidar_cluster_ros2) で歩行者候補を抽出し、`people_msgs/People` または
  `PointCloud2` (filtered) として costmap_2d の obstacle layer に流す
- **担当 agent**: `ros2-implementer` (perception) → `code-reviewer`
- **入力**: Phase C の動作する Nav2、M5-d で完成済みの `velodyne_self_filter`
  (車体除去) 後の点群
- **出力**:
  - 新パッケージ `whill_perception/`
  - `src/third_party/patchwork-plusplus-ros/`, `src/third_party/lidar_cluster_ros2/`
    を `whill_lab.repos` に追加
  - `whill_perception/launch/pedestrian_detection_launch.py`
  - `whill_perception/src/cluster_to_people.cpp` (BBox + サイズフィルタで歩行者
    だけ抽出して `people_msgs/People` 化)
  - `whill_navigation/config/nav2_params.yaml` の `local_costmap` に obstacle
    layer 追加 (filtered cloud を読む)
- **主要パラメータ初期値**:
  - `patchwork.sensor_height: 0.7` (車椅子 LiDAR マウント高)
  - `patchwork.max_range: 20.0`
  - `cluster.min_cluster_size: 5`
  - `cluster.max_cluster_size: 200`
  - `pedestrian.bbox_size_min: [0.2, 0.2, 0.8]`
  - `pedestrian.bbox_size_max: [1.0, 1.0, 2.0]`
  - `local_costmap.obstacle_layer.observation_sources: pedestrian_scan`
  - `local_costmap.obstacle_layer.pedestrian_scan.observation_persistence: 0.5`
- **検証方法**:
  - 1 体の歩行者が 3 m 前方を横断する bag で、検出再現率 90%+
  - 歩行者通過後 1 秒以内に obstacle layer から消える (residual ゴースト無し)
  - 旧 M5-b ゴースト障害物地図でも本実装が悪化させない (`use_collision_detection:
    true` で再開可能)
- **リスク**:
  - patchwork++ がキャンパスの段差 (縁石・スロープ) で過検出 (research 地雷)
    - **フォールバック**: `sensor_height` と `max_range` を現地調整 + 段差箇所
      は static map で no-go zone を手動指定
  - クラスタリングが樹木の幹を歩行者と誤認
    - **フォールバック**: 速度推定を追加してから people 化 (静止物は除外)
- **工数**: L (5-8 日)
- **依存先**: Phase C, Phase 0 (旧 autoware_tracker / pedestrian_flow_navigator
  の入出力規約理解)

---

### Phase E: nav2_social_costmap_plugin (proxemics)

- **目的**: 歩行者の進行方向と速度を考慮した Gaussian costmap layer を追加し、
  歩行者の後ろを通る、横を 1 m 開けるなどの社会的距離行動を獲得
- **担当 agent**: `ros2-implementer` → `code-reviewer`
- **入力**: Phase D の `people_msgs/People`
- **出力**:
  - `src/third_party/nav2_social_costmap_plugin/` (`whill_lab.repos` 追加、必要なら fork)
  - `whill_navigation/config/nav2_params.yaml` の `local_costmap` に social layer 追加
- **主要パラメータ初期値**:
  - `social_layer.amplitude: 100.0`
  - `social_layer.sigma: 0.6` (proxemics personal zone)
  - `social_layer.covar_front_amplifier: 1.5` (進行方向側を強める)
  - `local_costmap.plugins: [obstacle_layer, social_layer, inflation_layer]`
- **検証方法**:
  - 歩行者の後ろを通る経路が生成される (Phase D 状態だと正面突破を試みる挙動と
    比較)
  - 既存 acceptance が回帰しない
- **リスク**:
  - upstream commit 9 件と少ない (research 地雷)
    - **フォールバック**: fork して本リポ管理下に置く。または `inflation_layer`
      の半径手動増で代用 (proxemics 精度は劣る)
  - 歩行者の後ろを延々追従し続けるバグ (research 地雷)
    - **フォールバック**: `planner_server.replanning_frequency` を 2 Hz に上げる
- **工数**: M (2-4 日)
- **依存先**: Phase D
- **生成 ADR**: なし (Phase D の ADR に proxemics 採用を併記する程度)

---

### Phase F: 統合 launch (whill_bringup) + 実機検証

- **目的**: M6 として、sensors + state_estimation + localization + perception
  + navigation を 1 つの top-level launch で起動し、キャンパス内実機検証を
  記録 (bag + 走行ログ + RViz スクリーンキャプチャ)
- **担当 agent**: `ros2-implementer` (launch 構築) → `code-reviewer`
- **入力**: Phase A〜E の成果物
- **出力**:
  - 新パッケージ `whill_bringup/`
  - `whill_bringup/launch/campus_autonomous.launch.py`
  - `whill_bringup/config/<env>.yaml` (lab / campus 切替)
  - `docs/m6-campus-validation.md` (検証手順 + 結果)
  - `docs/m6-bench-data/` (bag + 走行ログ)
- **検証方法**:
  - 受け入れ基準の全項目を実機で確認
  - 50 m / 100 m / 200 m goal を各 3 回ずつ成功
  - 歩行者シナリオ (1 体横断、2 体追従) で停止 or 回避成功
- **リスク**:
  - lifecycle node の起動順依存で `map -> odom` 確立前に Nav2 active になる
    - **フォールバック**: `lifecycle_manager` の `autostart: false` + リローカライ
      ゼーション成功を待ってから手動 transition
  - 屋外環境特有の問題 (GPS なし → 初期位置をどう与えるか、直射日光下の D435
    深度品質)
    - **フォールバック**: 初期位置は RViz `2D Pose Estimate` 経由、D435 は本 phase
      では使わず VLP-16 単独に限定
- **工数**: L (5-10 日、実機検証込み)
- **依存先**: Phase A〜E すべて

---

## 全体工数見積もり

| Phase | 工数 | 累積 |
|-------|-----|-----|
| 0     | S (1-2d) | 1-2d |
| A     | M (3-5d) | 4-7d |
| B     | L (5-8d) | 9-15d |
| C     | M (2-4d) | 11-19d |
| D     | L (5-8d) | 16-27d |
| E     | M (2-4d) | 18-31d |
| F     | L (5-10d) | 23-41d |

合計 23-41 営業日 (実機検証時間込み)。最大の不確実性は Phase B (FASTLIO2_SAM_LC
の humble ビルド) と Phase D (patchwork++ の現地調整)。

## リスクと既知の不確実性

- **リスク1**: Phase B で FASTLIO2_SAM_LC が humble で素直にビルドできない
  (GTSAM/PCL/Eigen 依存衝突) — 緩和策: 早期に Phase B の build smoke test を
  Phase A と並列で走らせる
- **リスク2**: Phase A の `/whill/odom` covariance が WHILL SDK 側で未設定の
  可能性 — 緩和策: Phase 0 と並行して `ros2_whill` を 1 度実機で吐かせて確認
- **リスク3**: Phase D の patchwork++ がキャンパス段差で過検出 — 緩和策: 現地で
  bag を 1 本撮ってから param tuning する phase を Phase D 内に組み込む
- **リスク4**: Phase F 実機検証時の天候・歩行者出現タイミングが再現不能 —
  緩和策: bag 記録を取り、後日 offline replay できる構成にする
- **不確実性1**: 大マップ (600m × 600m) で SmacPlanner2D が実用速度を出すか —
  解消するのに必要な情報: Phase C で実マップを使ったベンチ
- **不確実性2**: `nav2_social_costmap_plugin` upstream の humble 完全互換性 —
  解消するのに必要な情報: Phase E の build smoke test
- **不確実性3**: Phase B の段階 B (ndt_omp pure localization) への切替が
  必要になるかは Phase B 完了まで判断不能

## ADR の候補

このプランで生まれる重要な技術判断:

- [ ] ADR-0001: 車輪オドメ + LIO 融合の EKF 構成 (二段 EKF / robot_localization)
- [ ] ADR-0002: LiDAR-Inertial Odometry のスタック選定 (FASTLIO2_SAM_LC 採用)
- [ ] ADR-0003: Nav2 global planner 選定 (SmacPlanner2D 採用 + 二層 costmap)

ADR-0004 以降は Phase D / E 着手時に発生したら都度追加 (例: 歩行者検出の
patchwork++ vs Autoware lidar_centerpoint の最終判断、social costmap fork 方針)。

## 次のアクション

ユーザーが何をすればこのプランが動き出すか:

1. 本計画を読み、後述の質問 3 件に回答する
2. ADR-0001〜0003 のドラフトに目を通し、`proposed` → `accepted` への昇格判断
3. Phase 0 の起動 (`legacy-archaeologist` invoke) を承認する
