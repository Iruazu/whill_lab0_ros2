# 文書一覧

2026-10-14 以降は日本語を正本とする。現在使う手順と過去の実装経緯を分けて読む。

## 現用文書

| 文書 | 用途 |
| --- | --- |
| [リポジトリ概要](../README.md) | 実装範囲・入口 |
| [実機 runbook](ja/runbook-live-demo.md) | 準備、起動、初期位置、検査、停止、録画 |
| [構成](ja/architecture-overview.md) | ノード、トピック、TF、速度指令 |
| [スクリプト一覧](ja/scripts.md) | 実行ファイルの用途と使い分け |
| [開発環境](ja/development.md) | 上流取得、ビルド、文書検証 |
| [整備課題](ja/repository-maintenance.md) | 未解決事項・検証範囲 |
| [地図一覧](maps/README.md) | 地図成果物の規約 |
| [校正台帳](ja/calibration-ledger.md) | 校正履歴。現用値はコードと照合する |
| [設計判断](ja/decisions/README.md) | 採択時点の判断と追加記録 |

## パッケージの説明

[センサ](../src/whill_sensors_bringup/README.md)、[オドメトリ](../src/whill_localization/README.md)、[障害物前処理](../src/whill_perception/README.md)、[Nav2](../src/whill_navigation/README.md)、[安全層](../src/whill_safety/README.md)、[配車・Web](../src/whill_dispatch/README.md)。

## 実機検証記録

- [2026-10-14 統合起動・配車成功](ja/reports/2026-10-14-dispatch-demo.md)

## 履歴と成果物

- [過去の計画・調査・検証の索引](ja/history.md)
- `m*-bench-data/` は実測記録、`maps/` は地図成果物。

日常の起動は実機runbookを参照する。履歴文書の旧設定・未着手表記を現在の状態と読み替えない。
