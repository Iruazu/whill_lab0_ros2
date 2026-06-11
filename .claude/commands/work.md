---
description: 指定 Issue の実装から PR 起票までを一気通貫で回す (branch → 実装 → レビュー → コミット → PR draft)。マージは常にユーザーが行う。
argument-hint: <Issue 番号>
---

Issue #$ARGUMENTS を完了させてください。以下の順序を厳守:

## 手順

1. `gh issue view $ARGUMENTS` で Issue を読む。「方針文書との対応」または
   「前提とする仮定」が空なら、作業せず差し戻して終了する
2. main を最新化してからブランチを作成する:
   `git fetch origin && git switch main && git pull && git switch -c <フェーズ小文字>/$ARGUMENTS-<slug>`
   (例: `m4r/12-ekf-odom`)
3. `ros2-implementer` agent に実装を delegate する。入力として Issue の
   受け入れ基準・スコープ外・前提仮定を渡す
4. 実装完了後、`code-reviewer` agent を起動する。重大 (Must fix) があれば
   3 に戻して修正させる。重大 0 になるまで繰り返す
5. 受け入れ基準のうち Claude が検証可能なものを実行し、結果を記録する
6. コミットを作成する (作業ブランチ上のみ)。メッセージ 1 行目に
   `#$ARGUMENTS` を含める。論理単位ごとに分けてよいが、無意味な細切れにしない
7. `git push -u origin <ブランチ>` し、`.github/pull_request_template.md` の
   全項目 (方針適合チェック、code-reviewer 結果の貼付を含む) を埋めて
   `gh pr create --draft` する。実機検証が必要な項目は PR に明記する
8. PR の URL と「ユーザーがやること」(実機検証 / レビュー / マージ) を
   報告して終了する

## 禁止事項

- main への直接 push、`gh pr merge`、force push、rebase は行わない
- Issue のスコープ外に手を広げない。気付いた別問題は新 Issue の起案を提案する
- commit / push の権限が無い環境では、手順 6 以降を実行せず、
  ユーザーが実行すべきコマンド列と PR 本文の全文を提示して引き渡す
