# SETUP — 配置手順（エディタ手作業版）

zip を展開し、エディタ（VS Code 等）の GUI で各ファイルをリポ内の所定の位置へ置くだけ。
使い方は `USAGE.md`。Claude Code は導入済みの前提。旧リポは `~/whill_lab0/`（あなたの環境はこれ）の前提。

---

## 完成形ツリー（このとおりに配置すれば完了）

`★NEW` が今回追加するもの。それ以外（`README.md`, `whill_lab.repos`, `src/` 等）は既存。**既存ファイルは触らない**（`.gitignore` だけ 1 行追記）。

```
whill_lab0_ros2/
├── CLAUDE.md                              ★NEW  ← リポ直下に置く
├── .gitignore                             既存  ← 末尾に1行だけ追記（下記）
├── README.md                              既存  ← 触らない
├── whill_lab.repos                        既存
├── .claude/                               ★NEW  ← フォルダごと置く
│   ├── settings.json                      ★NEW
│   ├── agents/                            ★NEW
│   │   ├── pm-orchestrator.md             ★NEW
│   │   ├── legacy-archaeologist.md        ★NEW
│   │   ├── ros2-implementer.md            ★NEW
│   │   ├── research-analyst.md            ★NEW
│   │   ├── debugger.md                    ★NEW
│   │   └── code-reviewer.md               ★NEW
│   └── commands/                          ★NEW
│       ├── plan.md                        ★NEW
│       ├── port-feature.md                ★NEW
│       ├── debug.md                       ★NEW
│       ├── research.md                    ★NEW
│       └── review.md                      ★NEW
├── docs/                                  既存フォルダ
│   ├── legacy-index.md                    ★NEW  ← 記入済みの最新版を置く
│   ├── SETUP.md                           ★NEW  ← 本ファイル（参照用・任意）
│   ├── USAGE.md                           ★NEW  ← 使い方（参照用・任意）
│   └── （既存の m3-*.md などはそのまま）
└── src/                                   既存  ← 触らない
```

zip 内のフォルダ構成は完成形ツリーと同じなので、**展開した中身をリポ直下にそのままドラッグ＆ドロップ**すれば位置が一致する（`docs/` と `.claude/` はマージ）。

---

## 配置後にやる 3 つ

### 1. `.gitignore` に 1 行追記

個人用の上書き設定を git 管理外にする。エディタで `.gitignore` を開き、末尾に追記:

```
# Claude Code: 個人上書き設定はローカルに留める
.claude/settings.local.json
```

### 2. 旧リポパスの確認

あなたの旧リポは `~/whill_lab0/` なので **編集不要**。
もし別の場所に移した場合のみ、以下 4 ファイル内の `~/whill_lab0` を実パスに置換（エディタの一括置換でよい）:
`CLAUDE.md` / `docs/legacy-index.md` / `.claude/agents/legacy-archaeologist.md` / `.claude/settings.json`

### 3. Claude Code を再起動して確認

```
cd ~/whill_lab0_ros2
claude
```

プロンプトで `/agents` を実行し、6 体が出れば完了:

```
pm-orchestrator   legacy-archaeologist   ros2-implementer
research-analyst  debugger               code-reviewer
```

→ 以降は `USAGE.md`。

---

## 配置トラブル

| 症状 | 対処 |
|------|------|
| `/agents` に 6 体出ない | エディタで `.claude/agents/` に 6 ファイルあるか確認。Claude Code を起動した場所がリポ直下（`pwd` で確認）か確認。`/exit` → `claude` で完全再起動 |
| CLAUDE.md が効かない（一般論しか返らない） | `CLAUDE.md` がリポ**直下**にあるか確認。`/memory` で project memory 認識を確認 |
| 旧リポを読めない | `.claude/settings.json` の `Read(~/whill_lab0/**)` のパスが実際の旧リポと一致するか確認 |

---

## git コミット（チーム共有する場合・任意）

```
git switch -c chore/claude-team-setup
git add CLAUDE.md .claude docs/legacy-index.md docs/SETUP.md docs/USAGE.md .gitignore
git commit -m "chore: Claude Code チーム体制を導入"
```

`.claude/settings.local.json` はコミットしない（手順1で gitignore 済み）。
