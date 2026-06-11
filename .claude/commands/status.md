---
description: 進行状況ダッシュボード。Issue / PR / ブランチ / フェーズ進捗を 1 画面に要約し、ユーザーの判断待ち事項を先頭に提示する。
argument-hint: (引数なし)
---

現在の開発状況を以下の手順で要約してください。

## 手順

1. 次を実行して状態を収集する:
   - `gh issue list --state open`
   - `gh pr list --state open`
   - `git branch -a`
   - `git log --oneline -10 origin/main`
2. **「ユーザーの判断待ち」を先頭に列挙する**: レビュー / マージ待ちの PR、
   実機検証待ちの項目、未回答の質問、未決の ADR
   (`docs/plans/2026-06-11-platform-pivot.md` 7 章)
3. 方針文書 4 章のフェーズ (M4-R〜M9) ごとに状態を表で示す:
   未着手 / 計画済 / 実装中 (Issue 番号) / PR 中 (PR 番号) / 完了
4. 滞留 (7 日以上動きのない open Issue / draft PR) があれば指摘する

## 出力の制約

- 全体 30 行以内。詳細は Issue / PR の番号参照で済ませる
- 提案や講釈は不要。事実と判断待ち事項のみ
- gh が使えない環境ではその旨を 1 行で報告し、git ローカル情報のみで要約する
