---
description: debugger agent を明示的に起動して問題を切り分け修正する
argument-hint: <観測されている問題 / エラー>
---

`debugger` agent を起動し、以下の問題を解決してください。

問題:
$ARGUMENTS

debugger の system prompt 通り、以下を厳守すること:
1. 再現できないバグは修正しない
2. 仮説 → 観測で証拠を集めてから修正
3. 修正は最小スコープで
4. 修正後は元の再現コマンドで検証
5. 診断ログは削除してから引き渡す

修正が複数のパッケージに跨がる場合は、debugger 完了後に `code-reviewer` を起動して整合性を確認してください。
