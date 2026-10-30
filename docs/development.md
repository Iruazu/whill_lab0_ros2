# 開発環境とビルド

Ubuntu 22.04、ROS 2 Humble、colcon、rosdep、vcstool。上流の取得元と版は `whill_lab.repos`（ブランチ指定はコミット固定ではない）。

新しい PC では `scripts/install_ros2_humble.sh`、`install_udev_rules.sh`、`install_velodyne_network.sh` を中身を確認してから使う。sudo とネットワーク設定を伴う。構築済みの PC では不要。

## 上流取得と依存導入

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
scripts/import_upstream.sh
```

`vcs import` と `rosdep install` を行う。Patchwork++ の C++ core は `whill_perception` が直接組み込むため、上流の `ros/` `cpp/` `python/` は colcon で個別にビルドしない（`COLCON_IGNORE`）。

## ビルド

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  whill_sensors_bringup whill_localization whill_navigation \
  whill_perception whill_safety whill_dispatch
source install/setup.bash
```

GLIM はオフライン処理用で、`scripts/install_glim.sh` で入れる。地図の PCD は Git に無いため別途配置する。

## 変更の検証

launch を変えたら `--show-args` で引数と import を確かめる（実機通信の検証ではない）。

```bash
ros2 launch whill_safety m6r_bringup_launch.py --show-args
ros2 launch whill_navigation nav_launch.py --show-args
ros2 launch whill_dispatch dispatch_launch.py --show-args
```

テストがあるパッケージは `colcon test --packages-select <package>` と `colcon test-result --verbose`。

## 文書の置き場所

起動手順は runbook、責務・入出力・設定はパッケージ README、過去の証跡は計画・ベンチ記録に置く。同じ手順を複数の文書に書かない。

作業はブランチと PR で出す。main への直接 commit / push と自動マージはしない。
