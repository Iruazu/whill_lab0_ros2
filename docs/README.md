# 文書一覧

現在使う手順と過去の記録を分けて読む。

## 現用文書

| 文書 | 用途 |
| --- | --- |
| [リポジトリ概要](../README.md) | 実装範囲・入口 |
| [実機 runbook](runbook-live-demo.md) | 準備、起動、初期位置、検査、停止、録画 |
| [構成](architecture-overview.md) | ノード、トピック、TF、速度指令 |
| [スクリプト一覧](scripts.md) | 実行ファイルの用途 |
| [開発環境](development.md) | 上流取得、ビルド、検証 |
| [デモ回帰チェック](regression-check.md) | マージ前に通す確認 |
| [未解決事項と制約](repository-maintenance.md) | 残課題と実装の制約 |
| [M8 ロードマップ](plans/2026-10-30-roadmap-m8.md) | 課題の順番。課題の正本は GitHub Issues |
| [地図一覧](maps/README.md) | 地図成果物の規約 |
| [校正台帳](calibration-ledger.md) | 校正履歴。現用値はコードと照合する |
| [設計判断](decisions/README.md) | ADR |

## パッケージ

[センサ](../src/whill_sensors_bringup/README.md)、[オドメトリ](../src/whill_localization/README.md)、[障害物前処理](../src/whill_perception/README.md)、[Nav2](../src/whill_navigation/README.md)、[安全層](../src/whill_safety/README.md)、[配車・Web](../src/whill_dispatch/README.md)。

## 実機検証記録

- [2026-10-14 統合起動・配車成功](reports/2026-10-14-dispatch-demo.md)

## 研究テーマの検討

- [研究の系譜と未解決性を確かめる（HTML、2026-10-16）](research/whill-research-evidence.html)：信頼性・診断・適応・学習制御の14文献。到達点、仮定、比較条件、専門性を整理。

- [選んだ5件の到達点と残る問いを読む（HTML）](research/whill-research-selected.html)：研究名で比較。実験の到達点・未検証事項・後続研究・拡張仮説と判断メモ。

- [26件の研究をカードで読む（HTML）](research/whill-research-papers.html)：具体的な問い・方法・成果・限界。分野の絞り込みと「気になる」選択。

- [WHILLを実験基盤とする研究課題のサーベイ（2026-10-15改訂）](research/2026-10-15-whill-research-topics.md)：先行研究25件の到達点・検証範囲、既存案の再評価、研究上の問い5案と実験設計。

## 過去の記録

- [過去の計画・調査・検証の索引](history.md)
- `m*-bench-data/` は実測記録、`maps/` は地図成果物。走行 bag は `~/bags/`（[bag の置き場](bags.md)）

過去の記録にある旧設定や「未着手」は当時の状態。
