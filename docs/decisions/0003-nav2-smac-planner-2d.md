# ADR-0003: Nav2 global planner に SmacPlanner2D を採用し global/local costmap を二層構成にする

- 日付: 2026-05-28
- 状態: accepted (2026-05-28, ユーザー承認: 切り返し禁止確定、SmacPlanner2D 採用)

## 文脈

現行 `whill_navigation/config/nav2_params.yaml` は global planner に NavFn
(Nav2 デフォルト) を使用している。これは M5-a〜c の小スケール (5-10 m 級) 検証
には十分だったが、キャンパス 200 m 以上の経路では以下が問題化する。

- **NavFn の計算負荷**: 600 m × 600 m を 5 cm/pixel で張ると 12000 × 12000 セル、
  wavefront 展開が中規模 (88 ms 級) から大規模では数百 ms に達する
- **NavFn の paths artifacts**: gradient ベース wavefront で生成される経路に
  グリッド由来のジグザグが残ることが知られている
- **single-resolution costmap の限界**: 全域を fine resolution (5 cm) で張ると
  メモリが数 GB に達し、global planner の探索空間も比例して膨らむ。一方で
  local controller (RPP) は fine resolution が必要

旧 noetic は loader_kiban で独自 A\* を実装していたが、これは Nav2 標準 plugin
で十分代替可能 (research 文書の論点 1)。

## 検討した選択肢

### 選択肢 A: NavFn 継続 + global costmap だけ低解像化
- パラメータ変更のみで実装コストゼロ
- メリット: 最小変更
- デメリット:
  - NavFn の artifacts 問題は解消しない
  - 計算負荷削減は costmap 解像度のみで頭打ち

### 選択肢 B: SmacPlanner2D + 二層 costmap (本案採用)
- Nav2 公式 plugin、Apache-2.0
- NavFn 比 38% 高速 (公式ベンチ)、artifacts 無し
- 差動 2 輪 (車椅子) のホロノミック近似に最適
- global costmap 15 cm / local costmap 5 cm の二層構成
- メリット:
  - 公式 plugin で保守責任が upstream
  - 大マップでも計画時間が現実的
  - パス品質が NavFn より明確に高い
- デメリット:
  - `max_iterations` を大マップ向けに増量必要 (default 1000000 → 10000000)
  - 既存 acceptance test を再走させる必要

### 選択肢 C: SmacPlannerHybrid (Hybrid-A*)
- 切り返し含む経路計画 (Reeds-Shepp / Dubins)
- メリット: 最速 (39-42 ms)
- デメリット:
  - 差動 2 輪に minimum turning radius 制約を強制すると本来不要な迂回が発生
  - 切り返し計画は車椅子の挙動として不自然 (利用者酔いの懸念)
  - 過剰機能

### 選択肢 D: ThetaStarPlanner
- any-angle 計画で open area の斜め経路に強い
- メリット: open area の経路品質
- デメリット:
  - キャンパス歩道 (細い経路 + 障害物多) でのコーナー品質は SmacPlanner2D に
    劣る可能性 (research 文書)
  - 大マップでの実証少

## 決定

**選択肢 B (SmacPlanner2D + 二層 costmap)** を採用する。

具体的設定:
- `planner_server.GridBased.plugin: nav2_smac_planner::SmacPlanner2D`
- `planner_server.GridBased.max_iterations: 10000000`
- `global_costmap.resolution: 0.15` (キャンパス用)
- `local_costmap.resolution: 0.05` (現状維持)
- `global_costmap.rolling_window: false`, `local_costmap.rolling_window: true`

lab 用 (小スケール) と campus 用 (大スケール) で param ファイルを分離し、
launch 引数で切替える。

理由:
- NavFn の既知問題 (artifacts + 計算負荷) を一度に解消
- 公式 plugin で導入コスト最小、ライセンスも Apache-2.0 で問題なし
- 差動 2 輪に対する適合性が最も高い (SmacPlannerHybrid 過剰、ThetaStar 細道
  劣化リスク)
- 二層 costmap は Nav2 公式 docs にも記載のある標準パターン

## 帰結

良い側面:
- 200 m 経路でも 5 秒以内に計画完了の見込み
- パス品質向上で RPP の追従が滑らかになり、velocity_smoother の苦労が減る
- メモリ使用量が二層化で大幅削減 (global を 15 cm にすると単純計算で約 1/9)

悪い側面:
- M5-c までの acceptance を再走させる必要 (小スケール回帰確認)
- 低解像 global costmap で 1 m 幅以下の細い通路が inflation で塞がる可能性
  (キャンパスの細い歩道で要確認)
- `nav2_params.yaml` の lab/campus 切替がパッケージ複雑性を増やす

将来見直すべき条件:
- 細い通路でのプランニング失敗が頻発する場合、global_costmap を 10 cm に
  上げる、または ThetaStarPlanner との A/B test
- 600 m スケールで `max_iterations` 不足が出た場合、SmacPlannerHybrid への
  切替 (車椅子に切り返しを許容するかは別途 ADR)
- MPPI controller への移行が必要になった場合、global planner との相性を
  再評価
