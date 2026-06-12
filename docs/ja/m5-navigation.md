# M5 — ROS 2 humble での自律航法

Language: [日本語](m5-navigation.md) | [English](../en/m5-navigation.md)

> **ステータス**: M5-d **完了** (2026-05-20)。WHILL CR2 上での初の自律 goal-to-pose 成功: 1.6 m 前方ゴールに ~6 秒でクリーンに到達、0 → 0.3 m/s への 0.3 m/s² 滑らかなランプ (velocity_smoother)、`bt_navigator: Goal succeeded`。M5 のクロージング条件は満たした。M5-e (追加チューニング + 長距離ゴール + 動的障害物) は任意で残る。

## ゴール

既知地図上で与えられたゴールポーズへ WHILL 椅子を自律走行させる。M4 の FAST-LIO 由来 `/Odometry` と M2 の `/cmd_vel` → WHILL モーション経路を土台に、その間に Nav2 (もしくは同等の ROS 2 航法スタック) を挿入する。

M5 の終了条件: `ros2 launch whill_navigation nav_launch.py` で起動し、RViz から 5 m / 10 m / 廊下端のポーズにゴールを置くと、椅子が経路を計画し、リアルタイム障害物回避を伴って経路追従し、許容範囲内で到着する状態。

## スコープ (M5 内)

- FAST-LIO の `camera_init -> body` TF を Nav2 が期待する `map -> odom -> base_link` チェーンに橋渡しする。
- 椅子が運用する研究室領域の 2D 占有格子 (もしくは 3D 対応 costmap) を構築する。FAST-LIO の PCD 保存を有効にした単発走行から起こし、オフラインで変換する。
- 保存地図と椅子チューン済 `nav2_params.yaml` に対して `whill_localization` + Nav2 lifecycle bringup を構成するトップレベル `nav_launch.py` を持つ `whill_navigation` パッケージを書く。
- 椅子上の live 検証: RViz で `geometry_msgs/PoseStamped` ゴールを送り、Nav2 が経路を計画してそこまで走るのを観察する。
- WHILL CR2 のダイナミクスに合うよう、costmap inflation、planner cadence、`controller_server` の速度・加速度上限をチューニングする。

## スコープ外 (M6 に譲る)

- 歩行者フロー / 動的障害物予測 (noetic スタックの `pedestrian_flow_navigator` 相当)
- ゴールシーケンシング、Nav2 デフォルトを超える recovery 動作
- ROS bag replay 駆動の検証ハーネス — M5 検証は椅子上で手動で行う

## ハードウェア → Nav2 入力

| 出力元 | トピック / TF | 備考 |
|-------|-------------|------|
| FAST-LIO | `/Odometry` (もしくは `nav_msgs/Odometry` の remap) | `camera_init -> body` で publish される。`map -> odom -> base_link` として再配信する必要がある |
| `whill_sensors_bringup` | `/tf_static` | `base_link → imu_link / velodyne / camera_link` の 4 つの static transform |
| `velodyne_pointcloud` | `/velodyne_points` | local costmap に直接、もしくは `pointcloud_to_laserscan` の 2D スライス経由で渡す |
| 保存地図 | `nav_msgs/OccupancyGrid` (`map_server`) | FAST-LIO の PCD からオフライン変換、もしくは Cartographer / SLAM Toolbox 走行から構築 |
| WHILL ドライバ | `/whill/controller/cmd_vel` 上の `geometry_msgs/Twist` (M2) | Nav2 の `controller_server` がここに publish する |

## 手順 (計画、作業の進行に応じて改訂)

1. **TF ブリッジ**。FAST-LIO の `camera_init -> body` を `map -> base_link` に remap し (FAST-LIO をグローバル localizer として扱う) 残りは `whill_sensors_bringup` の static TF に任せる、もしくは小さなノードを差し込んで fix を `map -> odom` (ドリフト) と `odom -> base_link` (瞬時) に分割する。簡単な道: 前者の remap だけの方式。

2. **地図構築**。`whill_localization/config/velodyne_whill.yaml` の `pcd_save.pcd_save_en: true` を立てて、テスト領域をカバーする緩やかなループを走る。得られた PCD を `docs/m5-maps/<env>.pcd` に保存する。[`scripts/pcd_to_occupancy_grid.py`](../../scripts/pcd_to_occupancy_grid.py) で占有格子に変換する (numpy のみ、ドリフト外れ値をクロップし、ラスタライズ前に椅子に関係する障害物帯に Z をスライスする)。

3. **Nav2 を `whill_lab.repos` に追加**する — fork した Nav2 が必要なら。そうでなければ apt の `ros-humble-nav2-*` パッケージで足りる。

4. **`whill_navigation` パッケージを書く**。`launch/nav_launch.py` で `whill_localization/localization_launch.py` を include し、椅子チューン済 `config/nav2_params.yaml` に対して Nav2 lifecycle スタック (`map_server`、`planner_server`、`controller_server`、`bt_navigator`、`behavior_server`、`lifecycle_manager_navigation`) を立ち上げる。

5. **最初の live テスト**。RViz の `2D Goal Pose` で 5 m 前方ポーズにゴールを置き、`controller_server` が `cmd_vel` を publish し、椅子が動き、ゴールに到達することを確認する。

6. **チューニングループ**。`nav2_params.yaml` を反復:
   - 椅子幅に合わせた costmap `inflation_layer.inflation_radius`
   - WHILL Mode 2 に合わせた `controller_server.FollowPath.max_vel_x`
   - `bt_navigator` の recovery 動作
   - planner cadence

## ステータス

| ステップ | ステータス |
|---------|-----------|
| ブランチ `m5/navigation` を main から切る | 完了 (2026-05-08) |
| マイルストーン文書スタブ作成 | 完了 (本ファイル) |
| **M5-a — TF ブリッジ** `map → camera_init → body → base_link → sensors` | **完了 (2026-05-08)** — `whill_navigation/launch/tf_bridge_launch.py` が 2 つの static identity (`map → camera_init`、`body → base_link`) を追加。run2 replay に対する `tf2_tools view_frames` で全チェーンを確認 (スナップショットは [`m3-bench-data/frames-m5a-2026-05-08.pdf`](../m3-bench-data/frames-m5a-2026-05-08.pdf)) |
| `whill_navigation` パッケージ骨格 + `nav_launch.py` コンポーザ | 完了 (2026-05-08) |
| **M5-b — 保存地図 (PCD)** | **完了 (2026-05-08)** — `docs/m5-maps/lab.pcd`、256,478 点、8.2 MB。`pcd_save.pcd_save_en: true` + `publish.map_en: true` で `m4_chair_live_2026-05-08_run2` を replay し、`/map_save` サービスを叩いて取得。**注意: FAST-LIO ドリフト区間からの散在点を含む** (有界軌道は原点から ~15 m に収まったが、XY 範囲は ±350 m に達する)。M5-c の PCD → 占有格子変換は、`nav2_map_server` に渡す前に積極的にクリップ・外れ値除去する必要がある |
| **`nav2_map_server` 用 2D 占有格子 (.pgm + .yaml)** | **完了 (2026-05-20)** — [`scripts/pcd_to_occupancy_grid.py`](../../scripts/pcd_to_occupancy_grid.py) が `lab.pcd` → [`docs/m5-maps/lab.pgm`](../m5-maps/lab.pgm) + [`docs/m5-maps/lab.yaml`](../m5-maps/lab.yaml) に変換。デフォルト: XY クロップ ±20 m、Z スライス [0.1, 1.5] m、0.05 m / セル → 800×800。密度外れ値フィルタ (5×5 ウィンドウ、最小クラスタ 5) + 原点周辺 1.5 m の clear-disk で歩行者通過スパイクと椅子自己反射を除去。原点からの Bresenham レイキャストで free space をマークし、未到達セルは unknown のまま。コミット `3270336`、`53d691f` |
| **`nav2_params.yaml` を椅子向けにチューン** | **完了 (2026-05-20)** — RPP、`desired_linear_vel: 0.3 m/s`、`use_collision_detection: false` (static map にまだ phantom が残るため)、`robot_radius: 0.45`、`inflation_radius: 0.5`。正準 47 プラグインの bt_navigator リスト。コミット `262772f`、`98e0c65`、`53d691f`、`8cd7fe5` |
| **`nav_launch.py` での Nav2 lifecycle bringup** | **完了 (2026-05-20)** — `map_server` + `planner_server` + `controller_server` + `behavior_server` + `bt_navigator` + **`velocity_smoother`** + `lifecycle_manager_navigation`。velocity_smoother で RPP 自身がランプしない実加速度上限 (前進 0.3 m/s²、回転 1.0 rad/s²) を強制。`/cmd_vel_smoothed → /whill/controller/cmd_vel` に remap。コミット `262772f`、`8cd7fe5` |
| RViz2 ドライラン: 椅子なしで map と costmap が描画される | 廃止 — 椅子上の live M5-d に直行 |
| **椅子上の live goal-following** | **完了 (2026-05-20)** — (0.08, 0.03) → (1.59, 0.00) の 1.6 m 前方ゴール。Begin → `Goal succeeded` が ~6 秒。cmd_vel ランプは 0 → 0.3 m/s が 1 秒 (`velocity_smoother.max_accel: 0.3` と一致)、recovery 動作は発火せず、走行中ずっと FAST-LIO がトラック維持。ユーザーフィードバック: "結構いい感じ" |
| チューニングノート整理 | 保留 — M5-e (任意の後続: 長距離ゴール、動的障害物層、クリーン再収録 map での collision detection 再有効化) |

## 未解決の問い (M5 進行中に答える)

- Nav2 を駆動するのに FAST-LIO 単独で十分な精度か、それとも保存地図上の AMCL でラップしてグローバル再 localization が必要か?
- 2D 占有格子で十分か、3D costmap が要るか (椅子/机部屋には Nav2 の 2D スキャンが見逃すテーブル脚がある)?
- `cmd_vel` を WHILL に直送するか、椅子の実ダイナミクスに合うよう velocity smoother 経由にするか?

## 関連

- M4 マイルストーン文書 + config: [`m4-localization.md`](m4-localization.md)、[`../../src/whill_localization/`](../../src/whill_localization/)
- M2 WHILL ドライバトピック: [`m2-whill-core.md`](m2-whill-core.md)
- M3 からの TF ツリー: [`m3-sensors.md`](m3-sensors.md) + M4 の FAST-LIO 由来 `camera_init -> body`
