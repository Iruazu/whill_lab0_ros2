# 開発環境とビルド

## 前提

Ubuntu 22.04、ROS 2 Humble、colcon、rosdep、vcstool。実機 runtime の自作パッケージは6つ。上流の取得元・版は `whill_lab.repos` を参照し、ブランチ指定とコミット固定を同じものと扱わない。

新規環境では `scripts/install_ros2_humble.sh`、`install_udev_rules.sh`、`install_velodyne_network.sh` の内容を確認してから使用する。sudo やネットワーク設定を伴う。既に構築済みの実機PCで毎回実行する必要はない。

## 上流取得と依存導入

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
scripts/import_upstream.sh
```

このスクリプトは `vcs import` と `rosdep install` を行う。現用ローカル版は Patchwork++ の `ros/`、`cpp/`、`python/` に `COLCON_IGNORE` を作成する。使用する C++ core は自作 `whill_perception` が組み込む。`scripts/import_upstream.sh` には今回以前から未コミットの変更がある。

## 車載スタックのビルド

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-up-to \
  whill_sensors_bringup whill_localization whill_navigation \
  whill_perception whill_safety whill_dispatch
source install/setup.bash
```

GLIM はオフライン処理用。導入は `scripts/install_glim.sh` と歴史記録 `m5r-glim-setup.md` を照合する。地図の PCD は Git からは取得できないため、必要な地図成果物を別途配置する。

## 変更の検証

launch を変更したら、環境を読み込んだうえで該当 launch の `--show-args` を実行する。これは引数・import の確認であり、USB・DDS・実機通信の検証ではない。

```bash
ros2 launch whill_safety m6r_bringup_launch.py --show-args
ros2 launch whill_navigation nav_launch.py --show-args
ros2 launch whill_dispatch dispatch_launch.py --show-args
```

該当パッケージのテストがある場合は変更範囲に応じて `colcon test --packages-select <package>` と `colcon test-result --verbose` を使う。文書だけの変更で実機を起動する必要はない。

README、runbook、パッケージ README に重複した起動手順を増やさない。手順は runbook、責務・入力・設定はパッケージ README、過去の証跡は計画・ベンチ記録に置く。英語版の同期は不要。

作業ブランチと PR を使う。main への直接 commit / push と自動マージは行わない。
