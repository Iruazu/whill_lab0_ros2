# Research: キャンパス内自律走行スタック — noetic → ROS 2 Humble 移植技術選定

## 調査日
2026-05-28

## TL;DR

推奨スタック: SmacPlanner2D / RPP継続 / FASTLIO2_SAM_LC + ndt_omp localization / social_costmap_plugin + people_msgs / patchwork++ + euclidean-cluster。最大のボトルネックはローカライゼーション (論点3) で、FAST-LIO単体18%ドリフトを解消しない限り他論点の改善効果は薄い。ここだけ先行着手を推奨する。

---

## 論点 1: グローバルパスプランナー

### 評価軸

- 600 m × 600 m スケールで 5 cm/pixel コストマップを張ると 12000 × 12000 セル。全域 wavefront 展開コストが現実的か
- 車椅子は差動2輪で切り返し不可。kinematic feasibility の価値は限定的
- 16 ゴール切替え時に毎回再計画が走るため、単発計画のレイテンシより再計画頻度とコスト合計が重要

### 候補比較表

| 候補 | ROS 2 humble 対応 | 最終更新 | ライセンス | 計算負荷 | Nav2 統合 | 本案件適合 |
|------|------------------|---------|-----------|---------|----------|-----------|
| NavFn (Dijkstra/A*) | 公式同梱 | Nav2 releases に追随 | Apache-2.0 | 大 (≈66-88 ms/中規模マップ) | ネイティブ | 低 — 大マップで遅く、パス品質に artifacts あり |
| SmacPlanner2D | 公式同梱 | Nav2 releases に追随 | Apache-2.0 | 中 (≈50 ms、NavFn より約38%速い) | ネイティブ | 高 — 差動2輪に最適、パス品質優秀 |
| SmacPlannerHybrid (Hybrid-A*) | 公式同梱 | Nav2 releases に追随 | Apache-2.0 | 低 (≈39-42 ms、最速) | ネイティブ | 中 — 切り返し前提、差動2輪には過剰 |
| ThetaStarPlanner | 公式同梱 | Nav2 releases に追随 | Apache-2.0 | 中 (≈46 ms/87.5 m パス) | ネイティブ | 中 — open area での斜め経路は有利、コーナー品質は未検証 |

### 推奨: SmacPlanner2D

- NavFn 比で約 38% 高速かつ gradient wavefront artifacts なし
- 差動2輪 (= 車椅子) のホロノミック近似に最適。HybridA* の切り返し計画は不要
- 600 m × 600 m を 5 cm 解像度で張ると計算負荷が問題になる可能性があるため、global costmap は 10-20 cm/pixel での運用を推奨

不採用理由:
- NavFn: 計算コスト高・パス artifacts の既知問題
- SmacPlannerHybrid: minimum turning radius 制約を差動2輪に強制すると計画失敗リスク
- ThetaStarPlanner: any-angle は open area で有利だが、キャンパス歩道 (障害物多・細い経路) でのコーナー品質は SmacPlanner2D に劣る可能性

地雷:
- 600 m × 600 m を fine resolution で張ると global planner メモリが数 GB に達する。`global_costmap` の `resolution: 0.2` 以上を推奨し、local costmap だけ fine にする二層構成
- SmacPlanner2D の `max_iterations` パラメータは大マップでデフォルト値 (1000000) では不足する場合あり

---

## 論点 2: ローカルコントローラ

### 候補比較表

| 候補 | ROS 2 humble | 最終更新 | ライセンス | 計算負荷 | Nav2 統合 | 適合 |
|------|------|---------|-----------|---------|----------|------|
| RPP (既採用) | 公式同梱 | Nav2 releases | Apache-2.0 | 低 | ネイティブ | 高 |
| MPPI Controller | 公式同梱 | Nav2 releases | Apache-2.0 | 高 (GPU 推奨) | ネイティブ | 中 |
| DWB | 公式同梱 | Nav2 releases | Apache-2.0 | 中 | ネイティブ | 中 |
| Graceful Controller | 公式同梱 | Nav2 releases | Apache-2.0 | 低-中 | ネイティブ | 中 |

### 推奨: RPP を継続 (現状維持)

WHILL CR2 の最大速度 1.6 m/s は MPPI の恩恵を最大化できる速度帯ではなく、計算コストと見合わない。動的歩行者回避は論点 4 の costmap layer 側で吸収するのが正しい役割分担。

不採用理由:
- MPPI: Nav2 issues #5375 で差動2輪での近距離劣化が報告
- DWB: RPP との差が小さく切替の必然性なし
- GracefulController: 動的障害物への応答が costmap 依存のみで RPP+velocity_smoother と差が小さい

---

## 論点 3: Map-based Localization (★最重要)

### 候補比較表

| 候補 | ROS 2 humble | 最終更新 | ライセンス | 計算負荷 | Nav2 統合 | 適合 |
|------|------|---------|-----------|---------|----------|------|
| AMCL (2D MCL) | 公式同梱 | Nav2 releases | Apache-2.0 | 低 | ネイティブ | 低 |
| ndt_omp / lidar_localization_ros2 | 対応 (2024-05) | 2025-05 (v0.2.1) | BSD | 中 | 中 | 高 |
| autoware_ndt_scan_matcher | 対応 | 2024 active | Apache-2.0 | 中-高 | 低 | 低 |
| FASTLIO2_SAM_LC (liangheming) | 対応 | 2024-2025 active | MIT | 中 (GTSAM) | 中 | 高 |
| robot_localization EKF | 公式同梱 | 2024 active | BSD | 低 | ネイティブ | 中 (補助) |
| HDL Localization (ROS2) | 対応 (2024-10) | 不明 | BSD | 中 | 中 | 中 |

### 推奨: 2段階戦略

**段階 A (短期, M5-e): FASTLIO2_SAM_LC (liangheming版)**
- ROS 2 humble 対応、GTSAM によるポーズグラフ最適化でループクロージャ
- ICP ベースの relocalization (`/slam_reloc` サービス)
- 既存 `whill_localization` の FAST-LIO2 コアをドロップイン置換可能

**段階 B (将来, M6 安定化後): lidar_localization_ros2 (ndt_omp系) との二段構え**
- マップ構築後は軽量な NDT pure localization に切替えで計算負荷削減

不採用理由:
- AMCL: VLP-16 を LaserScan 変換すると屋外の高低差で投影不安定
- autoware_ndt_scan_matcher: Autoware エコシステム依存が深く切り出し困難
- FAST-LIO-SAM (engcang版): ROS 1 のみ
- robot_localization EKF 単体: ドリフト補正機能を持たない (車輪オドメ統合用として別途必要)

地雷:
- FASTLIO2_SAM_LC のループクロージャは「位置近傍」ICP マッチング。開放経路ではドリフト補正効果限定的。初回マップ構築はループを意識したルートで
- `map -> camera_init` identity 接続を pose graph 出力で上書きする際、costmap 初期化前の ancient TF 参照に注意

---

## 論点 4: 歩行者回避戦略

### 候補比較表

| 候補 | ROS 2 humble | 最終更新 | ライセンス | 計算負荷 | Nav2 統合 | 適合 |
|------|------|---------|-----------|---------|----------|------|
| (a) costmap_2d dynamic obstacle layer | ネイティブ | Nav2 releases | Apache-2.0 | 低 | 最高 | 高 |
| (b) MPPI social critic | ネイティブ | Nav2 releases | Apache-2.0 | 高 | 中 | 低-中 |
| (c) nav2_social_costmap_plugin (upo版) | 確認済 | 9 commits | Apache-2.0 | 低 | 中 | 中 |
| (d) people_msgs + spencer_tracking 互換 | 断片的 | 不明 | Apache-2.0/LGPL | 中 | 低 | 低 |
| (e) potential_field を Nav2 plugin 再実装 | 自作 | — | BSD | 低 | 中 | 中 |

### 推奨: (a) 近期 + (c) 中期

- 近期: 検出ノードが出す BBox を `people_msgs/People` または MarkerArray に変換して `costmap_2d` obstacle layer に流す
- 中期: 人道的配慮 (proxemics) が必要になった時点で (c) を追加 layer として載せる

地雷:
- `nav2_social_costmap_plugin` は commit 9 件と少なく、fork 運用が現実的
- 歩行者の後ろに計画されたパスを延々フォローし続ける現象が起きるため `planner_server` の `replanning_frequency` を上げる

---

## 論点 5: LiDAR 歩行者検出

### 候補比較表

| 候補 | ROS 2 humble | 最終更新 | ライセンス | 計算負荷 | 適合 |
|------|------|---------|-----------|---------|------|
| patchwork++ + euclidean-cluster | CI確認 2024 | 2024 active | MIT + BSD | 低 | 高 |
| autoware lidar_centerpoint (DNN) | 対応 | 2024-12 | Apache-2.0 | 高 (GPU必須) | 低 |
| lidar_cluster_ros2 (jkk) | 対応 | 2024 active | MIT | 低 (100 Hz+) | 高 |
| RealSense D435 + YOLO + depth | 対応 | 2024 active | Apache-2.0 | 中-高 | 中 (補助) |
| 旧 lidar_obstacle_detector ROS2 fork | 非公式 | 2022 停滞 | MIT | 低 | 低 |

### 推奨: patchwork++ + lidar_cluster_ros2

- patchwork++ は IROS 2022、ROS 2 公式リポで humble/jazzy CI 通過
- VLP-16 で動作実証 (Patchwork2 後継ドキュメント記載)
- クラスタを BBox サイズ + 速度ヒューリスティックで歩行者ラベリング → `people_msgs` 化 → costmap layer

不採用理由:
- lidar_centerpoint: 16 beam での精度未保証 + GPU 必須
- RealSense D435 + YOLO: 屋外直射日光で深度品質劣化
- 旧 lidar_obstacle_detector fork: 2022 停滞

地雷:
- patchwork++ はキャンパス段差 (縁石・スロープ) で過検出可能性。`sensor_height` と `max_range` の現地調整必要

---

## 総合推奨スタック

```
VLP-16 点群
  |
  +-- [patchwork++]  地面除去
  |      |
  |   [lidar_cluster_ros2]  euclidean clustering
  |      |
  |   [BBox filter + people_msgs 変換]
  |      |
  |   +--> costmap_2d obstacle layer (近期)
  |   +--> nav2_social_costmap_plugin layer (中期)
  |
  +-- [FASTLIO2_SAM_LC]  LIO + loop closure + relocalization
         |
      [robot_localization EKF]  /whill/odom + LIO odom fusion
         |
      map -> odom tf  (Nav2 標準)
         |
      [Nav2 lifecycle]
         ├── PlannerServer:  SmacPlanner2D  (NavFn から変更)
         ├── ControllerServer: RPP + velocity_smoother  (現状継続)
         └── CostmapServer: obstacle layer + social layer
```

## 優先アクション

1. **M5-e 最優先**: FASTLIO2_SAM_LC を `whill_localization` に統合 (ループクロージャ + relocalization)
2. **M5-e 並行**: `robot_localization` EKF で `/whill/odom` + FAST-LIO Odometry 統合
3. **M5-b 修正後**: `use_collision_detection: true` 復活 + patchwork++ + lidar_cluster_ros2 を obstacle layer に流す
4. **M6 前**: `nav2_social_costmap_plugin` を local costmap に追加
5. **global planner**: NavFn → SmacPlanner2D (param 変更のみ)

## 不確実性

1. SmacPlanner2D vs ThetaStarPlanner の大マップ性能比較は単一論文ソースで未検証
2. MPPI vs RPP の動的環境性能は Systems 誌 1 件のみ
3. FASTLIO2_SAM_LC の relocalization 定量ベンチは公式未公開
4. patchwork++ の VLP-16 キャンパス環境での ground truth 比較未実施

## Sources

- Nav2 Smac Planner: https://docs.nav2.org/configuration/packages/configuring-smac-planner.html
- arXiv 2401.13078 (Smac Planner論文): https://arxiv.org/html/2401.13078v1
- Black Coffee Robotics navigation review: https://www.blackcoffeerobotics.com/blog/ros-and-ros2-navigation-stacks-a-performance-review
- Nav2 RPP: https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
- Systems 2026 (RPP vs MPPI): https://doi.org/10.3390/systems14030228
- Nav2 MPPI: https://docs.nav2.org/configuration/packages/configuring-mppic.html
- FASTLIO2_ROS2 (liangheming): https://github.com/liangheming/FASTLIO2_ROS2
- FASTLIO2_SAM_LC (liangheming): https://github.com/liangheming/FASTLIO2_SAM_LC
- lidar_localization_ros2 (rsasaki0109): https://github.com/rsasaki0109/lidar_localization_ros2
- autoware_ndt_scan_matcher: https://github.com/autowarefoundation/autoware_core/tree/main/localization/autoware_ndt_scan_matcher
- nav2_social_costmap_plugin: https://github.com/robotics-upo/nav2_social_costmap_plugin
- patchwork-plusplus: https://github.com/url-kaist/patchwork-plusplus
- patchwork-plusplus-ros: https://github.com/url-kaist/patchwork-plusplus-ros
- Patchwork++ arXiv 2207.11919: https://arxiv.org/abs/2207.11919
- lidar_cluster_ros2 (jkk-research): https://github.com/jkk-research/lidar_cluster_ros2
- Nav2 setup_guides robot_localization: https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html
- Nav2 issue #5375 (MPPI differential): https://github.com/ros-navigation/navigation2/issues/5375
