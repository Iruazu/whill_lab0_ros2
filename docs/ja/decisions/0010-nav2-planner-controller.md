# ADR 0010: Nav2 planner + controller の継続採用 + `allow_unknown: false` (M6-R)

Language: [日本語](0010-nav2-planner-controller.md) | [English](../../en/decisions/0010-nav2-planner-controller.md)

- Status: **proposed** (M6R4-1+2 起草、M6R4-3 V4 走行データで accepted 化)
- Date: 2026-07-14
- Deciders: Iruazu (承認待ち)

## 背景

Nav2 の global planner と local controller は M5-c 期に「conservative first-cut」として選定した (NavfnPlanner + RegulatedPurePursuit)。当時のコメントには「M5-e で見直す」旨があるが、M5-e は
[`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §5 で凍結された。M6-R では選定の再確認と、M5-R の `campus` マップに特有の
`allow_unknown` 判断を併せて記録する必要がある。

`campus` マップは M5-R 本番で生成した外周ループ 1310 m の 2D 占有格子。中央
の約 200 m × 200 m は未走行 = unknown 領域として残っている
([`../../maps/campus/README.md`](../../maps/campus/README.md) §2)。この構造は
デモ本番の運用 (外周走行のみ) と整合するが、`planner_server` の `allow_unknown`
既定 (true) のままだと planner が unknown 領域を通行可として最短経路を切って
しまうため、外周を大回りせずに中央を突っ切る挙動が出うる。

## 決定

M6R4-1 の `nav2_params.yaml` に以下を確定する:

### 1. Global planner: `nav2_navfn_planner/NavfnPlanner`

- グリッド Dijkstra 実装 (Nav2 標準)。`use_astar: false` = Dijkstra 通し
- `tolerance: 0.5` (goal 半径 0.5 m 以内に到達可能な経路がなければ拒否)
- `allow_unknown: false` (下記 §3)

### 2. Local controller: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`

- 曲率で線速度を調整する Pure Pursuit 派生。差動駆動の WHILL CR2 に適合
- 加減速は controller 自身では制限せず (`max_linear_accel` は documentation
  のみ)、実際のランプは下流の `velocity_smoother` が担当
- 主要パラメータ:
  - `desired_linear_vel: 0.3` (m/s)
  - `lookahead_dist: 0.6` / `min_lookahead_dist: 0.3` / `max_lookahead_dist: 0.9`
  - `use_velocity_scaled_lookahead_dist: true`
  - `transform_tolerance: 0.3` (M4-R EKF 30 Hz + M6R-2 localizer 10 Hz に合わせて緩め設定を撤回)

### 3. `planner_server.allow_unknown: false`

- 既定 true → **false** に反転
- 理由: `campus` マップ中央の unknown 領域を経路計画に含めないことで、
  外周のみを走行するデモ経路を planner レベルで担保する
- 弊害: unknown 領域を「通れば近道」となる目的地が来た場合、planner は
  経路を返さず `NavigateToPose` が失敗する。デモ経路は外周のみなのでこの
  弊害は発生しない前提。仮に発生した場合は M6R4-3 の V1 で観測される

## 代替案

### 代替 A: `nav2_smac_planner/SmacPlanner2D` (A\*)

- 経路品質は向上 (footprint-aware、diagonal 移動あり)
- ただし chair の走行速度 0.3 m/s では 10 m の経路の質差が体感に出にくい
- `campus` マップは 6640×6295 セルと大きく、SmacPlanner2D の初期化コストが
  NavfnPlanner より高い可能性 (実測未実施)
- 却下理由: M6-R デモの経路品質要件を NavfnPlanner で満たせる見込みが強く、
  未実測の性能差のためにパラメータ空間を増やさない。`campus-v2` (内側補完
  map) が来た時点で再評価する

### 代替 B: `nav2_dwb_controller/DWBLocalPlanner`

- 動的環境で強い (trajectory rollout でスコアリング)
- ただしパラメータ数が RPP より圧倒的に多く、屋外広域の tuning に時間がかかる
- RPP の失敗モードは「carrot が届かない」の単純なもので debug しやすい。
  DWB の失敗は「score が振れて振動」等、原因追跡が難しい
- 却下理由: demo scope で DWB のメリットが必要なほど動的環境ではない
  (歩行者はランダム横断であり、複数障害物の中を織り縫うわけではない)。
  DWB 移行は M7 (`whill_dispatch`) 以降で判断

### 代替 C: `nav2_mppi_controller/MPPI Controller`

- 高性能だが計算コスト高。Alienware x15 R2 の CPU 余力を localizer + Nav2 で
  ほぼ使い切っている現状で追加負荷は入れたくない
- 却下理由: demo scope で不要

### 代替 D: `allow_unknown: true` (現行凍結物)

- unknown を通行可として扱う。M5-b 期は lab.pgm の unknown 領域が「未観測
  だが物理的に通れる」ものだったので合理的だった
- しかし `campus` の中央 unknown は「未観測かつ物理的にも通行可否不明」で、
  planner が経路を返しても実際には塀・建物で不通の可能性が高い
- 却下理由: デモ経路は外周のみ、内側 unknown 経由の経路は原理的に不要

## 結果

- 走行速度上限は 0.3 m/s で継続。demo スケジュール的にもチューニング時間なし
- `campus-v2` (内側補完 map) が来たら §3 の `allow_unknown` を再評価
- DWB / MPPI 移行、SmacPlanner2D への移行は M7 (`whill_dispatch`) 以降で判断
- 本 ADR は M6R4-3 の V4 (30 分連続走行) を経て accepted に昇格。V4 で経路
  暴走 (§3 の弊害) が観測されたら proposed に留め、`allow_unknown: true`
  への戻しを検討する

## 関連

- [`../plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../plans/2026-07-14-m6r4-nav2-obstacle-layer.md) §3 M6R4-1 (params 差分の一次記録)
- [`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md) §3.3 (Nav2 選定の親方針)
- [`../../maps/campus/README.md`](../../maps/campus/README.md) §2 (中央 unknown の由来)
- ADR-0008 (提案予定): Nav2 costmap 構成 (static + obstacle + inflation)
- ADR-0009 (提案予定): pointcloud_to_laserscan パラメータ選定
