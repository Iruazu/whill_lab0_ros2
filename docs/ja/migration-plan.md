# 移行計画: noetic → humble

Language: [日本語](migration-plan.md) | [English](../en/migration-plan.md)

移行元: `Iruazu/whill_lab0` (24 パッケージ、ROS noetic)。移行先: `Iruazu/whill_lab0_ros2`
(本リポ、ROS 2 humble)。

## パッケージ棚卸しと移植戦略

各パッケージは移植方式によって 3 群に分類する。**Group A** は ROS 2 公式上流パッケージで置換、
**Group B** は手作業で移植、**Group C** は本リポでは扱わない別プロジェクト扱いとする。

### Group A — ROS 2 公式上流で置換

| noetic パッケージ | ROS 2 置換先 | 備考 |
|------------------|-------------|------|
| `ros_whill` | [`whill-labs/ros2_whill`](https://github.com/whill-labs/ros2_whill) | WHILL Inc. 公式 ROS 2 ドライバ |
| `realsense-ros` | [`IntelRealSense/realsense-ros`](https://github.com/IntelRealSense/realsense-ros) (`ros2` ブランチ) | Intel 公式 ROS 2 ラッパ |
| `velodyne-mast`, `velodyne_pcl` | [`ros-drivers/velodyne`](https://github.com/ros-drivers/velodyne) (`ros2` ブランチ) | Velodyne 公式 ROS 2 ドライバ |
| `rt_usb_9axisimu_driver` | 上流の `ros2` ブランチ | 同じベンダの ROS 2 ブランチ |
| `FAST_LIO` | [`hku-mars/FAST_LIO`](https://github.com/hku-mars/FAST_LIO) (`ros2` ブランチ) | 同じ著者陣 |
| `linefit_ground_segmentation` | コミュニティ ROS 2 fork | `colcon test` で要検証 |
| `catkin_simple` | (廃棄) | `ament_cmake` で代替 |
| `ddynamic_reconfigure` | (廃棄) | ROS 2 の dynamic parameters API で代替 |

### Group B — 手作業で移植する独自コード

引き継ぎが行われていないため、各パッケージの実運用上の使用状況は不明。
方針: 下流パッケージ (例えば bringup launch ファイル) が参照したタイミングで初めて移植に着手し、本表に記録する。

| パッケージ | 想定される役割 | 移植優先度 |
|-----------|---------------|-----------|
| `autoware_tracker` | WHILL 周辺の物体トラッキング | 低 (Autoware-AI 依存) |
| `pedestrian_flow_navigator` | 歩行者周りの航法 | M5 |
| `ros_pede_movement` | 歩行者運動関連ユーティリティ | M5 |
| `slam_localization` | キャンパス特化の自己位置推定 | M4 |
| `route` | ルート生成・追従 | M5 |
| `sensor` | センサユーティリティ (TBD) | TBD |
| `tf_imus` | IMU 用 TF publish | M3 |
| `loader_kiban` | ベース / ローダ (TBD) | TBD |
| `position_to_velocity` | 速度導出 | M5 |
| `relative_velocity` | 障害物との相対速度 | M5 |
| `image_fps` | 画像 FPS 計測 | 低 |
| `reef_msgs` | メッセージ定義 (REEF) | 必要に応じて |
| `loam_velodyne` | 旧 LiDAR オドメトリ | 廃棄 (FAST-LIO で代替) |
| `lidar_obstacle_detector` | 障害物検出 | M5 |
| `velodyne_camera_calibration` | 外部パラメータキャリブレーション | ユーティリティ (単発) |

### Group C — 直接移植の対象外

- `Autoware/` — noetic 時代の Autoware AI 同梱版。ROS 2 側の後継は Autoware Universe で、実質別プロジェクト。本リポでは移植しない。

## ブランチ / PR 戦略

```
main
├─ m1/env-setup           ← ROS 2 humble インストール、スクリプト、ドキュメント
├─ m2/whill-core          ← whill_ros2 ドライバ + teleop bringup
├─ m3/sensors             ← Velodyne + RealSense + IMU
├─ m4/localization        ← FAST-LIO + slam_localization 移植
├─ m5/navigation          ← 歩行者フロー、ルート、障害物検出
└─ m6/bringup-integration ← トップレベル launch + 車載検証
```

各マイルストーンは最新の `main` から派生し、単一 PR として届ける。実機 WHILL 上で
(M1 はホスト上で) 該当部分が動作する状態になって初めてマージする。
