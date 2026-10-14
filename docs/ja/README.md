# 文書一覧

2026-10-14 以降は日本語を正本とする。現在使う手順と過去の実装経緯を分けて読む。

## 現用文書

| 文書 | 用途 |
| --- | --- |
| [リポジトリ概要](../../README.md) | 実装範囲・入口 |
| [実機 runbook](runbook-live-demo.md) | 準備、起動、初期位置、検査、停止、録画 |
| [構成](architecture-overview.md) | ノード、トピック、TF、速度指令 |
| [スクリプト一覧](scripts.md) | 実行ファイルの用途と使い分け |
| [開発環境](development.md) | 上流取得、ビルド、文書検証 |
| [整備課題](repository-maintenance.md) | 未解決事項・検証範囲 |
| [地図一覧](../maps/README.md) | 地図成果物の規約 |
| [校正台帳](calibration-ledger.md) | 校正履歴。現用値はコードと照合する |
| [設計判断](decisions/README.md) | 採択時点の判断と追加記録 |

## パッケージの説明

[センサ](../../src/whill_sensors_bringup/README.md)、[オドメトリ](../../src/whill_localization/README.md)、[障害物前処理](../../src/whill_perception/README.md)、[Nav2](../../src/whill_navigation/README.md)、[安全層](../../src/whill_safety/README.md)、[配車・Web](../../src/whill_dispatch/README.md)。

## 過去の記録

`plans/` は当時の実行計画、`research/` は調査、`reports/` は解析結果、`session-*.md` はセッション記録。`m1`〜`m6r` 文書と `migration-plan.md` は移植・検証の経緯として保存する。未着手などの表記は執筆当時の状態を示す。

- [2026-06-11 の構成転換](plans/2026-06-11-platform-pivot.md)
- [M6-R 自己位置推定計画](plans/2026-06-24-m6r-localization.md)
- [Nav2 障害物層の復帰計画](plans/2026-07-14-m6r4-nav2-obstacle-layer.md)
- [配車の実装計画](plans/2026-07-19-m7-dispatch.md)
- [旧 noetic 実装の索引](legacy-index.md)

旧計画にある FAST-LIO runtime、identity TF、旧DDS設定を現用手順として復活させない。最新の起動手順は runbook、挙動と数値はコード・設定を優先する。
