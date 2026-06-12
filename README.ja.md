# whill_lab0_ros2

Language: [日本語](README.ja.md) | [English](README.en.md)

[whill_lab0](https://github.com/Iruazu/whill_lab0) (元 ROS noetic) の ROS 2 humble 移植版。

本リポジトリは、WHILL モビリティロボットのスタック (ドライバ・センサ・自己位置推定・ナビゲーション) を ROS noetic から ROS 2 humble に移植し、宇都宮大学キャンパスで稼働する WHILL 実機での動作検証までを到達点とする。

## ステータス

完了済みマイルストーン (初期ロードマップ M1〜M5):

| Milestone | Title | Status |
|-----------|-------|--------|
| M1 | ROS 2 humble environment setup on host | 完了 |
| M2 | WHILL core driver on real hardware (Model CR2 / USB) | 完了 |
| M3 | Sensor stack (Velodyne / RealSense / IMU) | 完了 (PR #4, #5 マージ済み) |
| M4 | Localization baseline (FAST-LIO) | 完了 (PR #6 マージ済み) |
| M5-a/b/c/d | Nav2 bringup + first autonomous goal on the chair (2026-05-20) | 完了 (PR #7 マージ済み) |
| M5-d (continued) / M5-e | Long-distance goals, dynamic obstacles, tuning | 2026-06-11 のプラットフォーム転換決定により凍結 |

進行中のロードマップ (転換後。正となる表は [`docs/plans/2026-06-11-platform-pivot.md`](docs/plans/2026-06-11-platform-pivot.md) §4 を参照):

| Phase | Title | Status |
|-------|-------|--------|
| M4-R | Odom 基盤再構築: robot_localization EKF (`/whill/odom` + IMU)、TF 再配線、`tf_bridge_launch.py` の廃止 | 未着手 |
| M5-R | マップパイプライン: オフライン SLAM (GLIM または FAST-LIO SAM) + 動的物体除去 (ERASOR)、`docs/maps/<site>/` 成果物規約 | 未着手 |
| M6-R | 運用 localization + Nav2 再統合: scan-to-map localizer、初期位置 UX、フェイルセーフノード、obstacle layer の復活 | 未着手 |
| M7 | 配車 API 層 (`whill_dispatch`): 名前付き地点、ジョブキュー、`NavigateToPose` ラッパー、状態 publish、rosbridge | 未着手 |
| M8 | タブレット Web アプリ: 地図表示、目的地指定、配車呼び出し | 未着手 (別リポジトリでもよい) |
| M9 | 統合検証: 無人呼び出し走行、物理 E-stop および遠隔停止 | 未着手 |

フェーズ ID (`M4-R` 等) と受け入れ基準はプラットフォーム転換文書の §4 および §6 で定義する。`tf_bridge_launch.py` の identity 構成、およびそれに依存する新機能の追加は同文書 §5 により禁止する。

## 開発ワークフロー

日々の作業は issue 駆動。1 つの Issue は 1 ブランチ・1 PR で完結させ、マージはユーザーが行う。現行の方針文書は [`docs/plans/2026-06-11-platform-pivot.md`](docs/plans/2026-06-11-platform-pivot.md)。

- 1 Issue = 1 ブランチ = 1 PR。受け入れ基準は観測可能なチェック 3 つ程度に抑え、1 セッションで実装とレビューが収まる粒度にする。
- ブランチ名: `<phase>/<issue-number>-<slug>` (例: `m4-r/12-add-ekf`)。雑務は `chore/<issue-number>-<slug>`。
- `main` は PR レビュー必須で保護 (approvals 0 — レビューもマージもユーザーが担う)。
- PR のマージは常にユーザーの作業。ADR (`docs/decisions/NNNN-*.md`) の起案も同様で、エージェントは下書きまで、`accepted` 行はユーザーが書く。
- Claude Code セッションから使えるスラッシュコマンド:
  - `/issue <phase or topic>` — 方針文書との対応 (M\*/R\*/P\*) と前提を埋めた、適切な粒度の GitHub Issue を起案する。
  - `/work <N>` — Issue N を引き取り、ブランチ作成、`ros2-implementer` での実装、`code-reviewer` の実行、コミット・push、ドラフト PR の作成までを行う。マージはユーザー。
  - `/status` — issue / PR / ブランチ / フェーズのダッシュボードを表示。ユーザー対応待ちの項目を先頭に並べる。

## ディレクトリ構成

```
whill_lab0_ros2/
├── src/         # colcon ソース空間 — ROS 2 パッケージ群
├── docs/        # 移行計画、マイルストーン別のメモ
└── scripts/     # 一発実行のセットアップ・ユーティリティ
```

## ビルド

ROS 2 humble をインストール済みで `source /opt/ros/humble/setup.bash` を読み込んだ状態で:

```bash
cd ~/whill_lab0_ros2
./scripts/install_udev_rules.sh      # /dev/whill, /dev/imu stable symlinks (one-time)
./scripts/import_upstream.sh         # vcs import + rosdep install
colcon build --packages-up-to whill --symlink-install
source install/setup.bash
```

上流パッケージは [`whill_lab.repos`](whill_lab.repos) に列挙してあり、`src/third_party/` 配下に clone される (gitignore 対象)。バージョンを固定する場合はこのファイルを編集する。

udev ルール ([`udev/99-whill-stack.rules`](udev/99-whill-stack.rules) で管理) は WHILL と RT 9 軸 IMU を USB VID:PID で識別する。どのポートに挿しても `/dev/whill` と `/dev/imu` に出てくる。

Velodyne VLP-16 は USB ではなく Ethernet 経由で接続するため、ホスト側の USB-Ethernet アダプタを LiDAR のサブネットに載せる:

```bash
ip -br link show | grep -E '^(enx|eth|enp)'              # find your iface
./scripts/install_velodyne_network.sh enxAABBCCDDEEFF    # substitute your iface name
```

これは [`network/01-velodyne-static.yaml.template`](network/01-velodyne-static.yaml.template) を `/etc/netplan/` に展開して適用する。経緯と、Velodyne 本体が再プログラムされていた場合のサブネット切替方法は [docs/m3-sensors.md](docs/m3-sensors.md) を参照。

## ドキュメント

プロジェクト文書一式は [`docs/`](docs/README.md) 配下に配置している。マイルストーン別メモ、移行計画、セッションログを含む。

## 参考リンク

- ソースリポ (noetic): https://github.com/Iruazu/whill_lab0
- ドキュメント目次: [docs/README.md](docs/README.md)
- 現行の方針文書: [docs/plans/2026-06-11-platform-pivot.md](docs/plans/2026-06-11-platform-pivot.md)
- 初期移行計画 (M1〜M3 の実行記録。今後の計画立案は方針転換後の文書が正):
  [docs/migration-plan.md](docs/migration-plan.md)
