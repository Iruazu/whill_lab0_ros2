# Architecture Decision Records (ADR)

Language: [日本語](README.md) | [English](../../en/decisions/README.md)

このディレクトリは本リポジトリのアーキテクチャ・統治・運用に関わる意思決定を ADR 形式で記録する。

## 命名規約

`NNNN-<short-kebab-case-slug>.md` (例: `0001-docs-i18n.md`、`0002-localizer-choice.md`)。連番は ADR 追加順で重複させない。slug はそのまま英小文字ハイフンつなぎでよい (日本語版のスラグでも可だが既存実装と揃える)。

## 必須節

1. ヘッダ — タイトル、Status (proposed / accepted / superseded by NNNN / deprecated)、Date、Deciders
2. 背景 (Context) — なぜこの決定が必要になったか。技術的・組織的・履歴的事実
3. 決定 (Decision) — 何を採択したか。観測可能なルール・構造として書く
4. 採用しなかった案 (Alternatives) — 検討した他案と棄却理由。最低 1 案は記録する
5. 結果 (Consequences) — 採択により得るもの・失うもの・後続作業

## Status の更新フロー

- `proposed` で agent または人間が起案する
- ユーザーがレビュー後、本ファイルを編集して `accepted` 行を確定する (agent は自分で `accepted` を書かない)
- 後続 ADR で覆る場合は `Status: superseded by NNNN` に書き換える

## Index

次の ADR 番号を採る前にこの表で最大番号を確認し、追加時はここに 1 行足す。

| 番号 | タイトル | Status |
|------|---------|--------|
| 0001 | docs の日英二言語化方針 | accepted |
| 0002 | (欠番・未使用) | — |
| 0003 | M5-R マップ作成 SLAM の最終選定 | accepted |
| 0004 | M5-R 動的物体除去ツールの選定 | accepted |
| 0005 | `docs/maps/<site>/` マップ成果物規約 | proposed |
| 0006 | 運用 localizer の選定 (M6-R) | proposed |
| 0007 | フェイルセーフノード + twist_mux 設計 (M6-R) | proposed |
| 0008 | (欠番・未使用。m6r4 計画書の候補番号ズレの名残) | — |
| 0009 | pointcloud_to_laserscan の高さ帯選定と QoS bridge (M6-R) | accepted |
| 0010 | Nav2 planner + controller の継続採用 + `allow_unknown: false` (M6-R) | proposed |
| 0011 | 地面除去前処理 — Patchwork++ core + 自前 ROS 2 wrapper (M6-R) | accepted |

## 関連

- 開発方針 (active policy): [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §7 に ADR 候補リストあり
- `CLAUDE.md`: ファイル所在の規約節
