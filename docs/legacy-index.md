# Legacy Repo Index (`~/whill_lab0/`)

旧 noetic 実装 `~/whill_lab0/` の **エントリポイント・マップ**。
`legacy-archaeologist` エージェントが調査の起点として参照する。

毎回旧リポを全 Glob するのは context window の浪費なので、ここに「どこに何があるか」の概略を貯める。
新しい調査で得た知見は `docs/legacy-findings/<topic>.md` に詳細を、ここには 1-2 行の見出しを追記する。

## 旧リポの位置

```
~/whill_lab0/
```

(配置が違う場合は本ファイル・`.claude/agents/legacy-archaeologist.md` 冒頭・`CLAUDE.md` の該当箇所・`.claude/settings.json` を書き換える)

---

## 機能マップ

### 駆動 / 車輪オドメトリ
- パッケージ: `ros_whill/`
- 主要ノード: `ros_whill` (`ros_whill/src/ros_whill.cpp`)
- noetic 移植元: `whill-labs/ros_whill`
- メモ:
  - シリアル接続は環境変数 `TTY_WHILL` で指定
  - sub: `/whill/controller/joy` (`sensor_msgs/Joy`, axes[0]=旋回, axes[1]=直進, 範囲 ±100)
  - pub: `/whill/states/jointState`, `/odom`, `sensor_msgs/BatteryState`, tf `odom → base_link`
  - 速度プロファイル: `ros_whill/params/initial_speedprofile.yaml`

### LiDAR / IMU / カメラドライバ
- パッケージ:
  - LiDAR: `velodyne-mast/`, `velodyne_pcl/` (Velodyne → `/velodyne_points`)
  - IMU: `rt_usb_9axisimu_driver/` (RT 9軸 IMU, `/dev/ttyACM0` → `/imu/data_raw`)
  - IMU tf: `tf_imus/` (`imu_link ↔ world`)
  - カメラ: `realsense-ros/` (Intel RealSense)
  - キャリブレーション: `velodyne_camera_calibration/`
  - 統合ドライバまとめ: `sensor/`
- ROS 2 側で対応する移植先: `whill_sensors_bringup`
- メモ:
  - IMU 標準偏差は `linear_acceleration_stddev`, `angular_velocity_stddev`, `magnetic_field_stddev` パラメータで設定

### LiDAR-Inertial Odometry (FAST-LIO)
- パッケージ: `whill_lab0/FAST_LIO/`
- 設定: `whill_lab0/FAST_LIO/config/velodyne.yaml`
- ROS 2 側: `whill_localization`
- メモ:
  - 校正済み extrinsic 値は本リポの `docs/m3-extrinsics-from-noetic.md` に転記済み
  - 主要ノード: `laserMapping` (`FAST_LIO/src/laserMapping.cpp`)
  - sub: `/velodyne_points`, `/imu/data` ← **ROS 2 側は `/imu/data_raw`。トピック名差分に注意**
  - pub: `/integrated_to_init` (`nav_msgs/Odometry`)
  - 構造: iKD-Tree + EKF、IMU 前処理は `IMU_Processing.hpp`
  - 主 launch: `mapping_velodyne.launch` (`filter_size_surf=0.5`, `filter_size_map=0.5`, `cube_side_length=1000`)
    - ※ `cube_side_length=1000` は本リポでは VoxelGrid int32 overflow を起こすため 200 に下げ済み
  - `loam_velodyne/` も同梱されているが現行運用は FAST-LIO 系統

### 自律走行 (キャンパス内)
- パッケージ: `loader_kiban/` (トップレベルオーケストレーション)
- 主要ノード:
  - `mapping_node` (`loader_kiban/src/mapping_node.cpp`)
    - in: `/integrated_to_init` → out: `/map` (`OccupancyGrid` 3000×3000, scale=20), `/localization/pose2d`
  - `path_planning_node` (`loader_kiban/src/pathplanning_node.cpp`)
    - **A\*** (対角コスト考慮) → `/path_planning/route` (`std_msgs/Float32MultiArray` `[x0,y0,x1,y1,...]`)
  - `motion_execution_node` (`loader_kiban/src/motion_execution_node.cpp`)
    - Pure Pursuit 風追従 → `/whill/controller/joy`
    - パラメータ: `goal_tolerance=5.0`, `linear_speed=0.2`, `angular_gain=0.4`
- launch: `loader_kiban/launch/autonomous_navigation.launch`
- 関連: グローバル位置補正用 `slam_localization/` (PCL NDT, `slam_localization_auto.cpp` / `ndt.cpp` / `gupndt.cpp` / `new_ndt.cpp`)
  - sub: `/velodyne_points`, `/map_cloud`, `/first_localization`, `/startslam`, `/endslam`
  - pub: `/second_localization`
  - パラメータ: `ndt_leaf`, `ndt_epsilon`, `ndt_step_size`, `ndt_iteration`、スコアは `ndt_final_score`, `ndt_final_accuracy`
- メモ: NDT 周りはバックアップソース（`*コピー.cpp`, `*バックアップ.cpp`）が複数残っており開発途中。本流は `slam_localization_auto.cpp`

### 運転アシスト
- パッケージ: (確認できる範囲では独立した「アシスト」パッケージは未設置)
- 主要ノード: `pedestrian_flow_navigator/` がポテンシャル法で同等機能（衝突回避）を担当
- メモ:
  - `relative_velocity/`, `position_to_velocity/` が周辺機能（速度推定・Pose→Velocity 変換）として存在

### 歩行者フロー判定 / 人検出
- パッケージ:
  - `pedestrian_flow_navigator/` (回避制御本体, **Potential Field** + **Lennard-Jones 型斥力**)
    - sub: `/autoware_tracker/tracker/objects_world`, `/integrated_to_init`
    - pub: `/whill/controller/joy`, 可視化 `/potential_field` (`visualization_msgs/MarkerArray`)
    - 主要パラメータ: 引力 `kv_=0.5`, 斥力 `p_=2.0, q_=1.0, w_=7e-6`, 感応半径 `rc_=7.0m`, ロボット半径 0.35m, 歩行者半径 0.30m, 旋回平滑化 `sigma_WN_=π/20`
    - ゴール座標は (199.4, 311.4) でハードコード（移植時は要外部化）
  - `autoware_tracker/` (IMM-UKF-PDA トラッキング → `/autoware_tracker/tracker/objects_world`)
    - `gating_thres=9.22`, `detection_probability=0.9`, `static_velocity_thres=0.5`
  - `ros_pede_movement/` (歩行者運動関連の補助)
- メモ:
  - 検出前段は `lidar_obstacle_detector/` (Euclidean クラスタリング + BBox) と `linefit_ground_segmentation/` (RANSAC 地面分離)

### Navigation (move_base / navfn / 等)
- パッケージ: **標準スタック未使用**。`loader_kiban/` 内で独自 A\* + Pure Pursuit を実装
- ROS 2 側: `whill_navigation` (Nav2 lifecycle)
- メモ:
  - 移植時は `loader_kiban` のロジックを Nav2 の global planner / controller plugin に置き換える方向で検討
  - 既存 `/map` は OccupancyGrid なので Nav2 costmap への流し込みは比較的容易
  - `motion_execution_node` と `pedestrian_flow_navigator` の両方が `/whill/controller/joy` に publish しており排他起動が前提（移植時は behavior tree で切替）

### Bringup / 統合 launch
- パッケージ:
  - `loader_kiban/launch/autonomous_navigation.launch` — mapping + planning + motion 一括起動
  - `FAST_LIO/launch/mapping_velodyne.launch` — LiDAR-Inertial Odometry
  - `ros_whill/launch/*` — WHILL ドライバ
  - `slam_localization/launch/*` — NDT グローバル位置推定
  - rviz: トップレベル `localiza_config.rviz`
- メモ: 完全統合の単一トップレベル launch は未確認。各サブシステムを個別に起動する運用と思われる

### その他 / 未分類
- `route/` — 直線ルート補間ツール
- `position_to_velocity/` — Pose → Velocity 変換（Mocap/カメラベース推定用と思われる）
- `relative_velocity/` — 相対速度算出
- `image_fps/` — 画像配信 FPS 測定ユーティリティ
- `ddynamic_reconfigure/` — dynamic_reconfigure 補助ライブラリ
- `catkin_simple/` — catkin マクロライブラリ
- `reef_msgs/` — 共通メッセージ定義
- `Autoware/` — Autoware の一部パッケージ群（`autoware_tracker` 等の依存）

---

## 移植時の横断シグナル（合成・要約）

上の機能マップから読み取れる、移植計画に直結する事実:

1. **トピック規約の差分**: 旧スタックは odometry を `/integrated_to_init`、joy 指令を `/whill/controller/joy`、FAST-LIO IMU 入力を `/imu/data` で運用。ROS 2 側は `/Odometry` / `/cmd_vel` / `/imu/data_raw`。移植時に remap か作り直しが必要。
2. **グローバル位置推定が存在する**: `slam_localization/` の PCL NDT が、事前点群地図 (`/map_cloud`) に対する map-based localization を担っている。現 ROS 2 スタックが持たない「FAST-LIO ドリフトを地図で補正する層」がここにある。Nav2 移行時の `map → odom` 補正源の有力候補。
3. **Navigation は完全自前**: move_base/Nav2 不使用。A\* + Pure Pursuit を `loader_kiban` に直書き。Nav2 の planner/controller plugin への置換が移植の主作業。
4. **排他起動の前提**: `motion_execution_node`(経路追従)と `pedestrian_flow_navigator`(回避)が両方 `/whill/controller/joy` に出力。Nav2 移行時は behavior tree で排他切替に整理するのが自然。
5. **ハードコード値の外部化が必須**: ゴール座標 (199.4, 311.4)、`goal_tolerance=5.0`、ポテンシャル場の各係数などが直書き。移植時に param 化する。
6. **地図表現**: 旧 `/map` は OccupancyGrid (3000×3000, scale=20)。Nav2 costmap へは比較的素直に流せる。

---

## 既に詳細調査済みの項目

(`legacy-archaeologist` 完了時にここへ追記)

| 機能 | 詳細ファイル | 調査日 |
|------|------------|--------|
| 学内自律走行 (キャンパス) | `docs/legacy-findings/campus-autonomous-navigation.md` | 2026-05-28 |
| WHILL 車輪オドメトリ計算 | `docs/legacy-findings/whill-wheel-odometry.md` | 2026-05-28 |

---

## 移植不要 / 廃棄推奨

- `slam_localization/` 内のバックアップソース（`*コピー.cpp`, `*バックアップ.cpp` 等）— 開発過程の遺物
- `loam_velodyne/` — FAST-LIO に置き換わっているため移植不要の可能性大（要確認）
- `catkin_simple/`, `ddynamic_reconfigure/` — ROS 2 では不要（ament / ros2 標準で代替）
