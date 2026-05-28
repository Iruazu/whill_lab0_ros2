# Legacy Investigation: Campus Autonomous Navigation (学内自律走行)

## 調査日
2026-05-28 (legacy-archaeologist)

## TL;DR

学内自律走行は `loader_kiban/` の 3 ノード (mapping, path_planning, motion_execution) が中核で、グローバル位置補正に `slam_localization/` (PCL NDT)、歩行者回避に `pedestrian_flow_navigator/` が独立して動く。ROS 2 移植の最大障壁は (1) 地図データが絶対パスにハードコードされた外部テキストファイル依存、(2) `mapping_node` と `path_planning_node` 間のトピック型不一致バグ (OccupancyGrid vs UInt8MultiArray) が未修正のまま残っていること、(3) NDT の初期位置推定と FAST-LIO が完全に疎結合で、`slam_localization_auto` は FAST-LIO ではなく **loam_velodyne の `/integrated_to_init`** を入力として設計されている点。

---

## A. 実行エントリポイント

### Launch 構成

`/home/systemlab/whill_lab0/loader_kiban/launch/autonomous_navigation.launch` (行 1-16):
- `mapping_node` (loader_kiban パッケージ)
- `path_planning_node` (loader_kiban パッケージ)
- `motion_execution_node` (loader_kiban パッケージ)
- `localization_manager` はコメントアウト (行 11-12)
- RViz もコメントアウト (行 14-15)
- **remap は 1 件もない**。全ノードがトピック名をハードコード

このランチには `slam_localization`, `pedestrian_flow_navigator`, `FAST_LIO`, センサドライバは含まれない。それぞれ別ランチで独立起動する運用。

### ノード起動順序・依存

ランチ側に順序制御はなく全ノード同時起動。`mapping_node` が `/integrated_to_init` を受信して初めて地図を生成するため、FAST-LIO (または loam_velodyne) が先に動いていることが実質的前提。

---

## B. データフロー

```
/velodyne_points ──> FAST_LIO (laserMapping) ──> /Odometry
                                                       │
              loam_velodyne (TransformMaintenance) ──> /integrated_to_init
                                                       │
                              ┌────────────────────────┘
                              ▼
                        mapping_node
                    (OccupancyGrid生成, scale変換)
                              │
                    /map (OccupancyGrid)
                    /localization/pose2d (Pose2D)
                    /map/update_trigger (Bool)
                              │
          ┌───────────────────┤
          ▼                   ▼
  path_planning_node    (pose2d を start 座標に使用)
  (A* 探索)
          │
  /path_planning/route (Float32MultiArray [x0,y0,x1,y1,...])
          │
          ▼
  motion_execution_node
  (Pure Pursuit 風)
          │
  /whill/controller/joy (sensor_msgs/Joy)

[独立系統]
/velodyne_points ──> lidar_obstacle_detector ──> /autoware_msgs/DetectedObjectArray
                                                        │
                                              autoware_tracker ──> /autoware_tracker/tracker/objects_world
                                                                          │
                                                              pedestrian_flow_navigator
                                                                          │
                                                              /whill/controller/joy (排他起動前提)
```

**重要な不整合**: `path_planning_node` の `map_sub` は `std_msgs/UInt8MultiArray` 型で定義されているが (`pathplanning_node.cpp:395`)、`mapping_node` が publish する `/map` は `nav_msgs/OccupancyGrid` 型 (`mapping_node.cpp:108`)。`synchronizedCallback` は呼ばれず、現行コードでは動作しないバグ。

### mapping_node: 座標変換ロジック

- `/integrated_to_init` の position (m) に `scale = 100 / SCALE = 5.0` を乗算 (`mapping_node.cpp:128-135`)
- `MAP_SIZE = 3000`, `SCALE = 20` マクロ定義 (`mapping_node.cpp:28-29`)
- OccupancyGrid の `resolution = 0.2 m/grid` (`mapping_node.cpp:256`)
- **地図原点**: `origin.position = (0.0, 0.0, 0.0)` ハードコード (`mapping_node.cpp:259-261`)
- 地図データは外部 txt ファイル (`mapping_node.cpp:282`) `/home/systemlab-f/catkin_ws/src/kiban_localization/src/map_data1.txt`
- セル値: `'0'`=自由, `'1'`=静的障害物, `'2'`=禁止領域, `'3'`=動的障害物, `'4'`=膨張障害物, `'5'`=ロボット位置

### path_planning_node: A* 詳細

- ヒューリスティック: スタート-ゴール直線からの垂直距離 (`pathplanning_node.cpp:74-86`)
- 移動コスト: 直線=1.0, 対角=√2 (`pathplanning_node.cpp:124`)
- ペナルティ: セル値 `'4'` (膨張) に +100, `'3'` (動的) に +1000 (`pathplanning_node.cpp:126-131`)
- 出力: `[x0,y0,x1,y1,...]` 平坦化配列 (`pathplanning_node.cpp:213-218`)
- ゴール: ランチパラメータ `goal_num` (デフォルト 1)、16 ゴールをハードコード (`pathplanning_node.cpp:190,260-330`)

### motion_execution_node: Pure Pursuit 詳細

- オドメ: `/localization/pose` (`motion_execution_node.cpp:38`)。`localization_manager` 経由 (launch でコメントアウト = 実質起動しない)
- グリッド変換: `position.x * 5.0` (`motion_execution_node.cpp:54-55`)
- ルックアヘッド: ウェイポイント順次消化、`goal_tolerance` 以内で次へ (`motion_execution_node.cpp:85-93`)
- 速度指令: `v = linear_speed` 定速, `w = angular_gain * angle_diff` P制御 (`motion_execution_node.cpp:95-98`)
- joy: `axes[0] = linear, axes[1] = angular` (`motion_execution_node.cpp:110-111`)

**注意**: `pedestrian_flow_navigator` は逆: `axes[0] = angular, axes[1] = linear` (`pedestrian_flow_navigator.cpp:210-211`)

### slam_localization: NDT 詳細

- トリガー:
  - `/startslam` → `slam=0, slam_phase=0` → 次の velodyne CB で PCD ロード + 初期位置設定 (`slam_localization_auto.cpp:607-609, 282-325`)
  - `/endslam` → map.pcd 保存 + `ros::shutdown()` (`slam_localization_auto.cpp:613-621`)
  - `/first_localization` → 前段ローカライザの初期推定を `fl` に格納 (`slam_localization_auto.cpp:262-270`)
- `/second_localization` は 5Hz publish (`slam_localization_auto.cpp:202-207, 224-250`)
- PCD: `/home/systemlab-kurihara/catkin_ws/src/slam_localization/map/map_local.pcd` (ハードコード)
- オドメ予測: 車輪オドメを共有メモリ (`libodo.so`, `ODO_SHM_ID`) から取得 → NDT initial guess (`slam_localization_auto.cpp:674-711`)

### pedestrian_flow_navigator

- ゴール `(199.4, 311.4)` ハードコード (`pedestrian_flow_navigator.cpp:41-42`)
- `motion_execution_node` との切替: **コード上の排他機構なし**。両ノードが `/whill/controller/joy` に同時 publish。kill/launch で切り替えていた運用と推測
- センサ→重心オフセット: `dx=-0.20m, dy=-0.35m` ハードコード (`pedestrian_flow_navigator.cpp:274-282`)

---

## C. パラメータ一覧

### loader_kiban

| パラメータ | 値 | 設定箇所 |
|-----------|-----|---------|
| `goal_tolerance` | 5.0 (グリッド = ≈1.0m) | `motion_execution_node.cpp:32` |
| `linear_speed` | 0.2 | `motion_execution_node.cpp:33` |
| `angular_gain` | 0.4 | `motion_execution_node.cpp:34` |
| `goal_num` | 1 | `pathplanning_node.cpp:190` |
| `MAP_SIZE` | 3000 | `mapping_node.cpp:28` |
| `SCALE` | 20 | `mapping_node.cpp:29` |
| 障害物膨張半径 | ±3 grid (≈0.6m) | `mapping_node.cpp:521-529` |
| センサ視野角 | 260° | `mapping_node.cpp:464` |
| センサ検出範囲 | 50 grid (10m) | `mapping_node.cpp:464` |

### NDT (slam_localization_auto.launch)

| パラメータ | 値 |
|-----------|-----|
| `dist_th` | 0.0 |
| `yaw_th` | 0.0 |
| `map_leaf` | 0.5 m |
| `vld_leaf` | 0.5 m |
| `h_vlp16` | 0.50 m |
| `ndt_leaf` | 1.0 m |
| `ndt_epsilon` | 0.003 |
| `ndt_step_size` | 0.5 |
| `ndt_iteration` | 100 |

### Potential Field 係数

| 変数 | 値 |
|------|-----|
| `kv_` | 0.5 |
| `p_, q_` | 2.0, 1.0 |
| `w_` | 7e-6 |
| `s_` | 0.1 |
| `rc_` | 7.0 m |
| `sigma_WN_` | π/20 |
| `r_robot, r_human` | 0.35 m, 0.30 m |
| 速度上限 | 1.6 m/s |
| 速度スムージング α_v | 0.2 |
| 角速度スムージング α_w | 0.3 |

### ハードコード値 (param 化必須)

| 値 | 場所 | 内容 |
|----|------|------|
| `/home/systemlab-f/...map_data1.txt` | `mapping_node.cpp:282` | 占有格子ファイル |
| `/home/systemlab-kurihara/...map_local.pcd` | `slam_localization_auto.cpp:284` | 事前点群地図 |
| `(199.4, 311.4)` | `pedestrian_flow_navigator.cpp:41-42` | 歩行者回避ゴール |
| `(44.9, 398.4)` | `slam_localization_auto.cpp:296-297` | NDT 初期位置 |
| 16 ゴール座標 | `pathplanning_node.cpp:260-330` | A* ゴール群 |
| `dx=-0.20, dy=-0.35` | `pedestrian_flow_navigator.cpp:274-275` | センサ→重心オフセット |

---

## D. 座標系・トピック規約の差分

### TF tree (旧)

- FAST-LIO: `camera_init → body` (`laserMapping.cpp:591-592`)
- loam_velodyne: `/integrated_to_init` (`camera_init` frame と推定、TransformMaintenance.cpp:60)
- slam_localization: `/second_localization` の `frame_id = "map", child = "base_footprint"` (`slam_localization_auto.cpp:234-235`)
- **`map → odom → base_link` の連続 TF チェーンは存在しない**。tf broadcaster コード自体がコメントアウト

### トピック対応 (旧 noetic → ROS 2)

| 旧 noetic | ROS 2 | 差分 |
|-----------|-------|------|
| `/integrated_to_init` | `/Odometry` | 同型 (nav_msgs/Odometry) |
| `/whill/controller/joy` | `/cmd_vel` | sensor_msgs/Joy → geometry_msgs/Twist |
| `/imu/data` | `/imu/data_raw` | 名前のみ |
| `/second_localization` | (なし) | map-based localization 未実装 |
| `/localization/pose2d` | (なし) | mapping_node 固有 |
| `/path_planning/route` | `/plan` | 独自 → nav_msgs/Path |
| `/map` (OccupancyGrid) | `/map` | 同型、解像度・原点異なる |

---

## E. 地図資産

### 占有格子 (2D)

- `/home/systemlab-f/catkin_ws/src/kiban_localization/src/map_data1.txt`
- フォーマット: 1 行目 `x_max y_max` + 以降 ASCII 2D 配列 (`mapping_node.cpp:287-295`)
- セル値: `'0'`=free, `'1'`=static, `'2'`=restricted
- サイズ: 3000×3000 grid × 0.2 m/grid = 600m × 600m

### 事前点群地図 (3D)

- `/home/systemlab-kurihara/catkin_ws/src/slam_localization/map/map_local.pcd`
- 生成: `/startslam` → マッピング走行 → `/endslam` (`slam_localization_auto.cpp:483,613-618`)
- 座標系: FAST-LIO の `camera_init` 基準

---

## F. 既知のハック・バグ・遺物

1. **mapping_node↔path_planning_node 型不整合** (`mapping_node.cpp:108` vs `pathplanning_node.cpp:395`): OccupancyGrid pub vs UInt8MultiArray sub。callback 呼ばれない
2. **motion_execution_node のオドメ不整合** (`motion_execution_node.cpp:38`): `localization_manager` がコメントアウトされている = `/localization/pose` 未配信 = `odomCallback` 呼ばれない
3. **slam=2 初期化** (`slam_localization_auto.cpp:201-203`): `velodyne_Callback` の switch に `case 2` なし。`/startslam` 待ち
4. **コメントアウト大ブロック** (`slam_localization_auto.cpp:382-479`): MCL 比較ロジックが死蔵
5. **`slam_localization/src/` のバックアップ氾濫**: `*コピー.cpp` `*バックアップ.cpp` 等。本流は CMakeLists で `slam_localization_auto` ターゲットが指す `slam_localization_auto.cpp`

---

## G. 移植上の最大の地雷 (5 件)

### 地雷 1: loam_velodyne への暗黙依存

`/integrated_to_init` は FAST-LIO ではなく loam_velodyne の `TransformMaintenance` が publish。ROS 2 側 FAST-LIO は `/Odometry`。旧スタック全体が `/integrated_to_init` 前提のため remap またはブリッジが必要。

### 地雷 2: 地図データが外部ファイル・絶対パス依存

`map_data1.txt` (`mapping_node.cpp:282`) が `systemlab-f` のホームディレクトリにあり、内容物のドキュメントなし。ROS 2 では Nav2 map_server (YAML+PGM) への変換が必要。

### 地雷 3: mapping_node↔path_planning_node の型不整合バグ

現行コードでは `path_planning_node` の中核 callback が呼ばれない。旧 noetic で動いていた構成は別 launch かコメントアウト内の旧版である可能性が高い。

### 地雷 4: Joy axes マッピングの食い違い

`motion_execution_node` (axes[0]=linear) と `pedestrian_flow_navigator` (axes[0]=angular) で逆。実機での正動作版を確認してから `/cmd_vel` (Twist) に統一。

### 地雷 5: TF チェーン未構築 + 座標系混在

旧スタックには `map → odom → base_link` が存在しない。`camera_init`, `map`, `base_footprint` が接続されておらず Nav2 要求と根本的に異なる。座標も `camera_init` メートル系とグリッド系 (×5) が混在。

---

## 開いている疑問

1. `map_data1.txt` の実物が何のキャンパスエリアか、座標原点がどこか
2. 旧スタックで最後に実機動作した構成 (型不整合バグを考えると別 launch の可能性)
3. WHILL ドライバが axes をどう解釈するか (`motion_execution` と `pedestrian_flow_navigator` で逆のため)
4. `/startslam` / `/endslam` を誰が publish していたか

---

## 主要 file:line リスト

1. `loader_kiban/launch/autonomous_navigation.launch:1-16`
2. `loader_kiban/src/mapping_node.cpp:28-29` (MAP_SIZE/SCALE)
3. `loader_kiban/src/mapping_node.cpp:101` (sub /integrated_to_init)
4. `loader_kiban/src/mapping_node.cpp:108-109` (pub /map, /localization/pose2d)
5. `loader_kiban/src/mapping_node.cpp:282` (地図ファイルパス ハードコード)
6. `loader_kiban/src/pathplanning_node.cpp:74-86` (直線ヒューリスティック)
7. `loader_kiban/src/pathplanning_node.cpp:120-144` (8 方向 A* + ペナルティ)
8. `loader_kiban/src/pathplanning_node.cpp:180-193` (型不整合の起点)
9. `loader_kiban/src/pathplanning_node.cpp:260-330` (16 ゴールハードコード)
10. `loader_kiban/src/motion_execution_node.cpp:32-34` (パラメータデフォルト)
11. `loader_kiban/src/motion_execution_node.cpp:107-112` (joy axes: axes[0]=linear)
12. `slam_localization/lanuch/slam_localization_auto.launch:11-25` (NDT 全パラメータ)
13. `slam_localization/src/slam_localization_auto.cpp:169-197` (sub/pub)
14. `slam_localization/src/slam_localization_auto.cpp:282-325` (slam=0 PCD ロード+初期位置)
15. `slam_localization/src/slam_localization_auto.cpp:549-603` (ndt_scan_matching)
16. `slam_localization/src/slam_localization_auto.cpp:607-621` (startslam/endslam CB)
17. `pedestrian_flow_navigator/src/pedestrian_flow_navigator.cpp:22-31` (Potential Field 係数)
18. `pedestrian_flow_navigator/src/pedestrian_flow_navigator.cpp:40-42` (ゴール座標ハードコード)
19. `pedestrian_flow_navigator/src/pedestrian_flow_navigator.cpp:207-212` (joy: axes[0]=angular, motion と逆)
20. `loam_velodyne/src/lib/TransformMaintenance.cpp:59-60` (/integrated_to_init の出所)
21. `FAST_LIO/src/laserMapping.cpp:857-858` (旧スタック未使用の /Odometry)
