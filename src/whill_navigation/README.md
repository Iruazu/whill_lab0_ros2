# whill_navigation

Nav2 と障害物前処理を起動する。2026-10-14 に現用の `nav_launch.py` と設定を照合。実機手順は [runbook](../../docs/ja/runbook-live-demo.md)。

## 起動と責務

`nav_launch.py` は Patchwork++、pointcloud_to_laserscan、map_server、planner、controller、behavior、bt_navigator、velocity_smoother、lifecycle manager を起動する。車体・EKF・localizer・failsafe・twist_mux は別の統合 bringup が担当する。

地図は `site` と `map_variant` で選択する。既定siteはcampus、variantが空だと `occupancy.yaml`。デモでは `map_variant:=v2` を明示する。`WHILL_MAPS_ROOT` で地図の親ディレクトリを変更できる。

## 現用設定

| 項目 | 内容 |
| --- | --- |
| planner | NavFn、`allow_unknown: false` |
| controller | Regulated Pure Pursuit、`use_collision_detection: true` |
| local costmap | static / obstacle / inflation、rolling window 10m四方 |
| global costmap | static / inflation。動的obstacle層は含まない |
| scan入力 | `/velodyne_points_no_ground` → `/scan` |
| scanの高さ | base_link基準 0.05〜1.6m |
| speed | slow=0.30、normal=0.45、fast=0.60m/s。既定slow |
| BT | `config/navigate_to_pose_no_spin_bt.xml`。Spin/BackUpを含まない |

速度選択はRPPとsmootherの前進上限だけを変更する。加減速や実測速度は別。設定の詳細は `config/nav2_params.yaml` と `pointcloud_to_laserscan.yaml` を参照。

## 速度指令

`avoidance:=false`（既定）：controller / behavior → `/cmd_vel_nav` → twist_mux → `/cmd_vel` → velocity_smoother → `/whill/controller/cmd_vel` → WHILL driver。

`avoidance:=true`：… → velocity_smoother → `/cmd_vel_smoothed` → collision_monitor → `/whill/controller/cmd_vel` → WHILL driver。smoother の出力先を remap で変えるだけで、twist_mux と smoother の設定は同じ。collision_monitor は `launch/collision_monitor_launch.py` から専用の lifecycle manager で起動する。active にならなければ driver に指令が届かないので、止まる側に倒れる。

Collision Monitor（`config/collision_monitor.yaml`、Humble 1.1.x、2026-10-30 に合成入力だけで確認）：

| 名前 | 範囲（base_link） | 動作 |
| --- | --- | --- |
| PolygonStop | x 0〜0.80 m、\|y\| ≤ 0.45 m | 4点以上で停止（回転も0） |
| PolygonSlow | x 0〜1.65 m、\|y\| ≤ 0.65 m | 4点以上で速度を0.5倍 |
| FootprintApproach | 半径0.45 mの円 | 衝突まで2.0 s未満なら減速 |

- 寸法は `nav2_params.yaml` の `robot_radius` 0.45 m から決めた（`test/test_collision_monitor_config.py` で照合する）。
- 入力は `/velodyne_points_hires_no_ground`（C2）と `/camera/camera/depth/color/points`。どちらも高さ0.10〜1.6 mの点を使う。トピックは `cm_lidar_topic:=` と `cm_depth_topic:=` で変えられる。
- Humble の Collision Monitor は、`source_timeout`（3.5 s）より古い入力を**無視する**（止めない）。入力が途絶えたときは、failsafe の `extra_perception_topics` が 2.0 s で止める（`whill_safety/launch/safety_launch.py`）。
- 停止は smoother の後ろでかかるので、平滑化されない。

failsafeもteleopも平滑化の前に調停する。安全停止指令も平滑化を通る。衝突検知や設定読み込みが有効でも、実機の検知範囲・停止距離を検証したことにはならない。

## 検証

launch変更時は `ros2 launch whill_navigation nav_launch.py --show-args`。実機ではlifecycle、地図、初期位置、入力受信を確認する。旧READMEの「collision detectionはfalse」「両costmapに動的層」「まだ走行しない段階」は過去の開発計画で、現用コードの説明ではない。
