---
description: research-analyst を明示的に起動して技術調査・選定を行う
argument-hint: <調査したい技術領域 / 比較したい候補>
---

`research-analyst` agent を起動し、以下の技術調査を行ってください。

調査対象:
$ARGUMENTS

research-analyst の system prompt 通り、以下を厳守:
1. 最低 3 つの独立したソースを参照
2. 一次情報を優先 (公式ドキュメント、論文、GitHub)
3. 対立する見解は両方提示
4. 本プロジェクトでの fit を必ず添える
5. 推奨を理由付きで断定する。中立を装って情報を並べるだけにしない

調査結果が後続の実装計画に直結する場合は、完了後に自動で `pm-orchestrator` に渡せる形でまとめてください。
