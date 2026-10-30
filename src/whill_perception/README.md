# whill_perception

Patchwork++のC++ coreを使って点群から地面を除去する自作ROS 2 wrapper。2026-10-14 に構成を照合。

## パイプライン

`/velodyne_points` → `patchworkpp_node` → `/velodyne_points_no_ground` → pointcloud_to_laserscan → `/scan`。

`ground_removal_launch.py` は `whill_navigation/nav_launch.py` に内包される。Nav2起動後に別途重複起動しない。地面除去後の点群はfailsafeのLayer Cも監視する。

設定は `config/patchworkpp.yaml`。斜面・路面の誤検知を減らすための前処理だが、段差や人をすべて検知する保証ではない。現用設定の近距離除外は1mで、failsafe Layer Dの1〜2m帯にも関係する。

## ビルド

上流C++ coreは `src/third_party/patchwork_plusplus/cpp/`。`whill_lab.repos` から取得する。現用の `scripts/import_upstream.sh` は上流の `ros/` / `cpp/` / `python/` をcolconの個別パッケージとして拾わないよう `COLCON_IGNORE` を置く。自作wrapperがcoreを直接組み込む。

上流wrapperのライセンス表記の不整合を避ける設計判断は [ADR-0011](../../docs/ja/decisions/0011-ground-removal-choice.md) に保存している。車載ビルド手順は [開発環境](../../docs/ja/development.md)。

## 回避モード用の検知点群（`perception_3d_launch.py`）

`nav_launch.py avoidance:=true` のときだけ起動する（計画 `docs/ja/plans/2026-10-28-avoidance-baseline.md` §2.1）。既存の driver・transform・`velodyne` TF・上の Patchwork++ には触らない。

| ノード | 入力 → 出力 | 役割 |
| --- | --- | --- |
| `velodyne_transform_node_hires` | `/velodyne_packets` → `/velodyne_points_hires` | `VLP16_hires_db.yaml`（±10°）で変換し直す。今の driver は ±15° の表で変換している |
| `patchworkpp_node_hires` | → `/perception/lidar_obstacles` | `sensor_height` 0.777、`frame_id_override: velodyne_hires` |
| `lidar_obstacle_filter` | → `/perception/lidar_obstacles_filtered` | base_link で 0.05〜1.6 m |
| `depth_obstacle_filter` | `/camera/camera/depth/color/points` → `/perception/depth_obstacles` | base_link で 0.10〜1.6 m、0.3〜3.0 m（暫定） |
| `static_tf_velodyne_hires` | | base_link → velodyne_hires（0.42, 0.36, 0.777、roll 0.24°、pitch 0.97°、yaw −0.317°） |

`patchworkpp_node` の `frame_id_override` は既定が空で、空なら入力の frame をそのまま使う（今と同じ）。`cloud_height_filter_node` は sensor → base_link を最新の static TF で引く（センサは固定取付のため）。

設定は `config/patchworkpp_hires.yaml` と `config/perception_3d.yaml`。深度だけの bag（点群なし）は `scripts/tools/replay_depth_to_points.py` で点群に戻して再生できる。根本対応（計画 §8）が済んだら、この経路は廃止する。
