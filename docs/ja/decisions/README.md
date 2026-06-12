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

## 関連

- 開発方針 (active policy): [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §7 に ADR 候補リストあり
- `CLAUDE.md`: ファイル所在の規約節
