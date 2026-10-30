# whill_perception

Patchwork++のC++ coreを使って点群から地面を除去する自作ROS 2 wrapper。

## パイプライン

`/velodyne_points` → `patchworkpp_node` → `/velodyne_points_no_ground` → pointcloud_to_laserscan → `/scan`。

`ground_removal_launch.py` は `whill_navigation/nav_launch.py` に内包される。Nav2起動後に別途重複起動しない。地面除去後の点群はfailsafeのLayer Cも監視する。

設定は `config/patchworkpp.yaml`。斜面・路面の誤検知を減らすための前処理だが、段差や人をすべて検知する保証ではない。現用設定の近距離除外は1mで、failsafe Layer Dの1〜2m帯にも関係する。

## ビルド

上流C++ coreは `src/third_party/patchwork_plusplus/cpp/`（`whill_lab.repos` から取得）。自作wrapperがcoreを直接組み込むため、上流の `ros/` / `cpp/` / `python/` はcolconで個別にビルドしない（`COLCON_IGNORE`）。

上流wrapperのライセンス表記の不整合を避ける設計判断は [ADR-0011](../../docs/ja/decisions/0011-ground-removal-choice.md) に保存している。車載ビルド手順は [開発環境](../../docs/ja/development.md)。
