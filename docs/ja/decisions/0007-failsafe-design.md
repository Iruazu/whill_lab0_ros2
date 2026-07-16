# ADR 0007: フェイルセーフノード + twist_mux 設計 (M6-R)

Language: [日本語](0007-failsafe-design.md) | [English](../../en/decisions/0007-failsafe-design.md)

- Status: **proposed** (M6R-3 着手時起案、M6R-5 完了時 accepted 化予定)
- Date: 2026-07-14 起案 / 2026-07-14 dessin-scope 縮小反映
- Deciders: Iruazu (承認待ち)

> **2026-07-14 スコープ縮小メモ**: 本 ADR の §Decision は「フル版」の設計。
> 2026-08-01 デモに向けて **M6R-3 の実装は下記 §Demo-scope reduction の
> lite 版に一時縮小**する (デモ運用条件で lite が成立するため)。lite 版
> の restore path は §Demo-scope reduction 末尾に記載。

## 背景

親方針 ([`../plans/2026-06-11-platform-pivot.md`](../plans/2026-06-11-platform-pivot.md))
の要件 R4 (無人走行の安全: 発散検知・自動/遠隔停止) は、単独では成立しない
運用 localization (ADR-0006 で採用した lidar_localization_ros2) の障害モードを
検知して cmd_vel を遮断する層を必要とする。

M6-R 実行計画 ([`../plans/2026-06-24-m6r-localization.md`](../plans/2026-06-24-m6r-localization.md))
§3.C は遮断方式として `twist_mux` を採用済 (優先度切替の設定 yaml が
そのまま「優先度の文書」になる利点)。§6 M6R-3 はこの上に `failsafe_node`
(3 層購読 + 判定) を新設し、`docs/ja/m6r-failsafe-design.md` に閾値根拠と
判定ロジックを記録することを求めている。

本 ADR は M6R-3 着手時点で **proposed** として起案し、G4 手動試験 (「/reinitialization_requested
手動 publish → cmd_vel zero 化 < 200 ms」) の pass と M6R-5 の受入テスト
(G1-G3) 通過をもって **accepted** に昇格させる。

## 決定

### 1. パッケージ配置

`whill_safety` (2026-07-12 M6R-2 で作成済) 内に以下を追加:

```
src/whill_safety/
├── whill_safety/
│   └── failsafe_node.py         (新規、本 ADR)
├── config/
│   └── twist_mux.yaml            (新規、本 ADR)
├── launch/
│   ├── m6r_bringup_launch.py    (既存)
│   └── safety_launch.py          (新規、本 ADR。m6r_bringup から include)
├── CMakeLists.txt                (failsafe_node の PROGRAMS install 追加)
└── package.xml                   (twist_mux, twist_mux_msgs exec_depend 追加)
```

理由: `whill_safety` パッケージは M6R-2 で作成した時点から M6R-3 の failsafe
と M9 の物理 E-stop を受け入れる想定 (README.md 冒頭に明記済)。既存 package.xml
にコメントアウトで dep 予約行がある。

### 2. failsafe_node の 3 層購読と判定ロジック

`failsafe_node` は以下の 3 topic を購読し、いずれかが「異常」と判定された時点で
`/cmd_vel_safety` に zero twist を publish して継続する。

| 層 | 購読 topic | 型 | 異常判定条件 | 根拠 |
|----|-----------|-----|-------------|------|
| A | `/reinitialization_requested` | `std_msgs/Empty` | 受信 = 即異常 | 手動遮断 (RViz ボタン / 運用者判断) |
| B | `/alignment_status` | `diagnostic_msgs/DiagnosticArray` | `fitness_score > FITNESS_MAX` または `has_converged: false` が `WINDOW_S` 秒間連続 | localizer 発散検知 |
| C | `/pcl_pose` | `geometry_msgs/PoseStamped` | 最終受信から `PCL_POSE_TIMEOUT_S` 秒経過 | localizer サイレント停止検知 |

判定は OR (どれか 1 つでも異常 → 遮断)。1 つの層が正常復帰しても、直近
`SAFE_HOLD_S` 秒間は遮断を維持 (瞬断抑制)。復帰は「全 3 層が同時に正常」
かつ `SAFE_HOLD_S` 経過が条件。

**publish**:
- `/cmd_vel_safety` (`geometry_msgs/Twist`, 20 Hz)
  - 正常時: 何も publish しない (twist_mux が nav 側を通す)
  - 異常時: zero twist を 20 Hz で publish (twist_mux が優先度で拾う)
- `/failsafe_status` (`diagnostic_msgs/DiagnosticArray`, 1 Hz)
  - 各層の状態、直近の遮断原因、SAFE_HOLD 残時間

### 3. 閾値 (2026-07-12 実測根拠)

`docs/m6r-bench-data/2026-07-12-acceptance-campus/manifest.yaml` および
`docs/m6r-bench-data/2026-07-14-verify-campus/manifest.yaml` の実測を根拠とする:

| パラメータ | 値 | 単位 | 根拠 |
|-----------|-----|------|------|
| `FITNESS_MAX` | 1.0 | (無次元 NDT score) | 実測レンジ 0.02-0.3 の上限に対して余裕 3x。localizer 内部の `score_threshold: 6.0` (reject 閾値) より十分厳しく、reject 前に遮断が入る |
| `WINDOW_S` | 2.0 | 秒 | 20 Hz publish の 40 サンプル。単発の外れ値では発火しない。10 分走行で reject 0 なので false positive は無い想定 |
| `PCL_POSE_TIMEOUT_S` | 1.0 | 秒 | 正常時 ~10 Hz publish の 10 周期分。DDS 短時間停滞は許容、完全停止は 1 秒で捕捉 |
| `SAFE_HOLD_S` | 3.0 | 秒 | 遮断→復帰が短周期で振動しないようヒステリシス |

閾値の再チューニングが必要になった場合は本 ADR を修正して記録する。

### 4. twist_mux 優先度

`config/twist_mux.yaml`:

```yaml
twist_mux:
  ros__parameters:
    topics:
      safety:
        topic:    /cmd_vel_safety
        timeout:  0.5
        priority: 100          # 最優先
      navigation:
        topic:    /cmd_vel_nav
        timeout:  0.5
        priority: 10
      # M9 で teleop を追加する場合 priority: 50 想定
    output_topic: /cmd_vel
```

**理由**:
- 優先度差 90 (100 vs 10) は将来 teleop (priority: 50) を挟む余地
- timeout: 0.5 s は publisher 側が 1 Hz でも維持できる緩めの設定
  (失敗時に safety が失効すると nav 側が復活してしまうため、safety は
  「発火後は 20 Hz で publish 継続」が仕様)

### 5. launch 統合

`launch/safety_launch.py` を新設し、`failsafe_node` + `twist_mux` を起動。
`m6r_bringup_launch.py` から `IncludeLaunchDescription` で include。既存の
mutual exclusion (odom_bringup と m6r_bringup の並行禁止) は継承。

Nav2 の cmd_vel 出力は `/cmd_vel` → `/cmd_vel_nav` に remap (M6R-4 の
`nav_launch.py` 側で対応)。ここが本 ADR のインターフェース境界。

## 採用しなかった案

- **LiDAR フレームレート監視を C 層に加える**:
  `/velodyne_points` の hz を fail 検知したい欲求はあるが、これは
  localizer よりも上流の故障。今回は `/pcl_pose` の停止で間接的に捕捉できる
  (`/velodyne_points` が来なければ `/pcl_pose` も来ない)。C 層の tightening は
  M9 で判断する。
- **速度制限 (soft limit)**:
  「遮断」だけでなく「減速」のオプションを持たせる案。実装工数が増える割に
  8/1 デモの安全網としては zero twist で十分。M9 以降で追加検討。
- **失敗時に stop_motion (Nav2 behavior) を呼ぶ**:
  Nav2 の behavior tree に組み込む方式は疎結合が崩れる。twist_mux による
  外側ゲートの方が failsafe 責務が明確。
- **`ros2_control` の emergency_stop を利用**:
  `whill_driver` は `ros2_control` を通していない (M2 実装時に採用しなかった)。
  経路が違うため今回は twist_mux 側で完結させる。
- **判定を moving average / Kalman filter で smooth 化**:
  実測 fitness 分布 (0.02-0.3、中央値 ~0.07) が既に安定していて、WINDOW_S=2s
  の連続判定で十分。overengineering を避ける。

## 結果

得るもの:

- R4 (無人走行の安全) の最小成立: 手動遮断 + localizer 発散検知 + サイレント
  停止検知の 3 層で cmd_vel を止められる状態
- G4 の acceptance が観測可能: `/reinitialization_requested` を publish して
  200 ms 以内に cmd_vel が zero 化する試験が単独で成立
- 閾値の由来が本 ADR + manifest で追跡可能。将来「fitness 0.5 で振動する」
  等の苦情が来た場合、根拠に立ち返って議論できる
- M9 の物理 E-stop / 遠隔停止を「A 層の別チャネル」として追加できる余地

失うもの:

- twist_mux のレイテンシ (~10-20 ms 実測想定) が cmd_vel パスに乗る。ただし
  WHILL の応答時間 (>100 ms) に比べれば無視できる
- failsafe_node が単一障害点になる。ノード自身のクラッシュを検知する層は
  持たない (systemd や ros2 launch のノード監視で最低限は担保)。強化は
  M9 で

## M6R-5 accepted 化条件

以下がすべて満たされた時点で本 ADR を **proposed** → **accepted** に昇格する:

1. G4 (`/reinitialization_requested` publish → cmd_vel zero < 200 ms) が
   実機で計測 pass
2. G1 (経路 1 周で失探しない) の実走行中に false positive の遮断が発火しない
   ことを bag で確認
3. `docs/ja/m6r-failsafe-design.md` に閾値の再測定手順 (どの bag を再生して
   どの CLI で確認するか) が書かれている
4. code-reviewer による priority-sorted findings が resolve 済み

## Demo-scope reduction (2026-07-14 追記)

### 決定

2026-08-01 オープンキャンパスデモに向けた **M6R-3 の実装は本 ADR §Decision
の「lite サブセット」に一時縮小**する。上位互換 (=フル版) の実装は本 ADR
に記載のまま残し、デモ後の M6-R フォローアップとして復元する。

### 何をやり、何をやらないか (Lite vs Full の差分)

| 項目 | Full (本 ADR §Decision) | **Lite (M6R-3 lite で実装、Issue #67 lite)** |
|------|------------------------|--------------------------------------------|
| A 層 手動遮断 | ✓ | ✓ (実装: `LAYER_A_HOLD_S=1.0s` の再ラッチ方式) |
| B 層 fitness 閾値 | ✓ (WINDOW_S=2.0s) | ✓ (同一) |
| B 層 has_converged=false 継続 | ✓ | ✓ (同一) |
| B 層 pcl_pose silence | ✓ (`PCL_POSE_TIMEOUT_S=1.0s`) | ✓ (同一) |
| **C 層 jump 検知** (連続 3 フレーム差分 > 0.5m) | ✓ | **✗ 未実装** |
| **SAFE_HOLD ヒステリシス** (3s) | ✓ | **✗ 未実装** (条件解除で即復帰) |
| **G4 実機 3 試験** (手動 / ノイズ点群 / 視野遮蔽) | 必須 | **バックログ** (デモ後) |
| BBS_2D 収束まで cmd_vel_safety 活性化 | ✓ | **未実装** (BBS_2D 自体を運用でオフ) |
| watchdog 各 topic 1s | ✓ | 部分的 (pcl_pose silence で相当機能) |
| twist_mux 優先度 | safety=100 > nav=10 | **同一** (据え置き) |
| 検証 | bag replay + 実機 G4 3 試験 | **bag 相当 mock 検証のみ** (実測 fitness 分布 0.02-0.3 で false positive 0、reinit 発火、fitness=5.0 で発火。詳細 PR #76 本文) |

### 縮小の理由 (受け入れ可能な risk trade-off)

デモ運用の実態が「操作者が椅子の横に立ち随伴、必要時に WHILL 物理ジョイス
ティックで即介入」という有人監視モードであり、以下が既に成立している:

- 物理ジョイスティックは WHILL ドライバの入力層で処理され、`cmd_vel` 経路
  とは独立に即遮断できる。つまり **人が横に立っていれば failsafe より速い
  hard override が常に存在**する
- SAFE_HOLD が無いことで生じる懸念 = 閾値付近での ON/OFF 振動は、操作者が
  目視で「振動 = 発散リスク」と読み取り、その場でジョイスティックに切替
  できる。飛び越えの被害はない
- jump 検知が無いことで生じる懸念 = 1 フレームの偶発的な localizer ジャン
  プを見逃す。ただし M4-R EKF が `odom -> base_link` の連続性を吸収する層
  として既に入っており、pcl_pose silence (1s) と fitness 閾値 (2s 継続) で
  実害のあるジャンプは間接的に捕捉される
- G4 3 試験は「無人走行時に事故を起こさない」保証のためのハードルであり、
  操作者随伴条件では緩められる。デモ後 (無人配車を目指す段階) で必須

### 復元経路 (Lite → Full)

デモ後、いずれかの状況で Full 版に昇格する:

1. **反復デモ / 無人試験** — 操作者随伴条件を外す予定が出た時点で G4 3 試験
   を実施、jump 検知 + SAFE_HOLD を実装
2. **failsafe が想定外に発火 or 見逃した** — バックログ Issue を起点に
   Full 版の該当層を先行実装
3. **M7 (dispatch) の無人配車機能を実装する時点** — 呼び出し先までの空車
   移動は無人走行なので Full 版が前提条件

Full 復元時の変更対象:
- `src/whill_safety/whill_safety/failsafe_node.py` に jump 検知 (`/pcl_pose`
  連続 3 フレーム差分 > 0.5m) と SAFE_HOLD (3s) を追加
- `docs/ja/m6r-failsafe-design.md` を新設、G4 実機 3 試験の手順と閾値
  再測定手順を記録
- 本 ADR を `accepted` 化 (§M6R-5 accepted 化条件を満たす)

### Lite 版の受け入れ (デモ merge 条件)

以下 3 点で M6R-3 lite の merge を認める:

1. `colcon build --packages-select whill_safety` PASS
2. Mock 検証 3 phase 全 PASS: baseline 15s = false positive 0 / reinit 発火
   と 1s 復帰 / fitness=5.0 で 2s 後発火 (詳細 PR #76 本文)
3. 本 §Demo-scope reduction が ADR-0007 に追記済み

上記が満たされば **Issue #67 は close せず**、本セクションで指定した
バックログ項目 (jump 検知 / SAFE_HOLD / G4 3 試験 / BBS_2D 自動停止) を
「M6R-3 follow-up」として残す。

## Layer D — 前方扇形 perception gate (2026-07-16 追記、proposed)

### 動機

2026-07-16 field で V2 (人が chair 前方 3-4 m 静止 → 停止) が **fail**。
`use_collision_detection: true` + obstacle_layer の人 lethal 化 + salt-
cleaned map の 3 条件を満たしていても停止しない事象を確認。

原因の確定:

- RPP `collision_check` の実効射程 = `max_allowed_time_to_collision_up_to_carrot × desired_linear_vel = 1.0 × 0.3 = 0.3 m` に過ぎない
- 加えて評価対象は **carrot (lookahead 0.8 m) 経路上のみ**
- planner が人を避ける経路を引き直せば「経路上の障害物」条件自体が
  成立しない

つまり **「障害物で停止 → 退去で再開」の要件は Nav2 のどの層にも
実装されていない**。demo 要件を満たすには専用の停止判定を入れる必要が
ある。

### 却下された代案 (A: RPP 側射程拡大)

- `max_allowed_time_to_collision_up_to_carrot` を数秒に拡大 → 実効
  射程は伸びるが、依然として carrot 経路 (lookahead 依存) 上のみ
- 拡大するほど「回避」と「停止」の責務が RPP に混在。planner 側は
  「経路を作る」責務に純化するのが Nav2 の設計思想
- lookahead を変えるとチューニング (蛇行) が再発する脆さ

### 採用 (B: failsafe Layer D)

停止を **safety 層の責務**として、RPP は回避に専念させる。Layer A/B/C
と同居する自然な拡張。cmd_vel ゲートは Nav2 の内部状態と独立に働く
ため確実。

**購読**: `/scan` (`sensor_msgs/LaserScan`, reliable QoS, ~10 Hz)。
p2ls_node の出力なので frame は `base_link`、angle は +x 前方の 0
基準。追加 subscription 1 本のみ (existing PointCloud2 layer C とは
別 topic)。

**判定**: base_link 前方扇形内の scan 点数が閾値以上 → 遮断。パラメータ:

| パラメータ | 値 | 根拠 |
|-----------|-----|------|
| `FORWARD_SECTOR_HALF_ANGLE_RAD` | 30° | WHILL 幅 0.6 m を 1.15 m 距離でカバー。cone 60° は反応の必要な前方視野に十分 |
| `FORWARD_SECTOR_MIN_M` | 0.5 m | p2ls `range_min` と一致、WHILL 車体上面の自己反射を除外 |
| `FORWARD_SECTOR_MAX_M` | 2.0 m | `desired_linear_vel = 0.3 m/s` で 6.7 s 反応余裕。velocity_smoother `max_decel = 0.5 m/s²` で停止距離 0.09 m ≪ 2.0 m |
| `FORWARD_POINT_COUNT_MIN` | 5 点 | /scan `angle_increment = 0.5°` で ±30° = 120 beam、人 0.5 m 幅 @ 2 m ≈ 15° = 30 beam、5 点はその 1/6 で単発 ghost では発火しない |
| `FORWARD_CLEAR_HYSTERESIS_S` | 0.5 s | 10 Hz scan の 5 連続クリアで解放、Layer A の再ラッチ方式と同構造。瞬断で ON/OFF 振動しない |

**発火 / 解放パターン** (Layer A の再ラッチ方式と同):
- scan callback で sector 内点数 ≥ 閾値 → `_forward_last_blocked_time = now`
- `_active_layers` で `now - _forward_last_blocked_time < HYSTERESIS_S` の間 `D:forward_blocked` 出力
- 継続クリア HYSTERESIS_S 経過で自動解放

**起動時 arming**: Layer C と同じ「first-message-arm」で
`_forward_last_blocked_time is None` の間は発火しない。起動直後の
false trip を避ける。

### BT / Nav2 との相互作用

Layer D 発火中の Nav2 側挙動:

1. `/cmd_vel = 0` (twist_mux が Layer D を通す)
2. `/odometry/filtered` velocity ≈ 0
3. `nav2_controller::SimpleProgressChecker` (`required_movement_radius: 0.5 m / movement_time_allowance: 10.0 s`) が 10 秒経過で `IsStuck` トリガー
4. BT が recovery (spin / backup / wait) に遷移。spin の回転 cmd_vel も Layer D で遮断されるので chair は動かない。allow_reversing=false で backup は無効。wait のみ実効。
5. Recovery タイムアウト後、BT が Goal aborted を返す可能性

**リスク**: 人が > 10-30 s 静止だと Goal fail。デモ経路では人退避は
数秒想定で問題なしと判断。field で誤 Goal aborted が観測されたら
`movement_time_allowance` を 30-60 s に拡大する。

### V2/V3 の再定義 (Layer D 基準)

| # | 従来判定 | Layer D 基準 |
|---|---------|-------------|
| V2 | 走行中の人横断 → 1 s 以内に `/cmd_vel_nav.linear.x < 0.05` | 前方 1.5-2 m 内に人立位 → **1 s 以内**に `/cmd_vel = 0` (twist_mux 出力)、failsafe log `D:forward_blocked` |
| V3 | 退避 5 s 以内に `/cmd_vel_nav > 0.1` | sector 外退避 → **1 s 以内** (0.5s hysteresis + scan 遅延) に Layer D 解放、`/cmd_vel_nav` 復活で走行再開 |
| V6.4 (追加) | — | (a) 静止状態で人を左右 30° 境界と距離 1.5 / 2.0 / 2.5 m に立たせて発火有無を確認 (geometry 実測)、(b) **経路脇 1 m 立位観客の false-trip 有無** (下記の観客導線リスクを field で判定) |

**V6.4 (b) の背景 — 観客導線リスク**: デモはオープンキャンパスで沿道
に見物客が立つ。±30° @ 2.0 m の扇は先端半幅が `2.0 × tan30° ≈ 1.155 m`
(先端全幅 ≈ 2.31 m) で、経路脇 1 m 強に立つ観客は sector 内。V6.4 (b)
の測定 (経路脇 1 m / 1.5 m 立位でそれぞれ発火するか) が判断材料:

| 発火状況 | 対処選択肢 |
|---------|-----------|
| 1 m 立位で発火、1.5 m 立位で無し | (a) 運用ルール = 観客導線を経路から **≥ 2 m** 離す (デモ準備チェックリスト §経路整備に追記して踏襲) |
| 1 m 立位でも 1.5 m でも発火 | (b) sector を **±25° へ絞る** (`FORWARD_SECTOR_HALF_ANGLE_RAD = math.radians(25.0)`)。±25° @ 2 m の先端半幅 = `2.0 × tan25° ≈ 0.933 m` で、1 m 立位はぎりぎり外れる。ただしマージン薄なので運用 (a) も併用推奨 |
| 1 m 立位でも 1.5 m でも無し | 現行 ±30° で十分。運用ルール不要 |

**注**: ±25° へ絞る場合、正面の人検知範囲も僅かに狭まる (V2/V3 の
「前方の人」検知は 2 m 距離では変わらない、幅方向で狭くなるだけ)。
安全側判定は変わらない。field 実測後に決定、決定は本 ADR §Layer D の
更新で追記。

### 夜間残像所見 (2026-07-16 late)

夜間の人通り増で **local_costmap の残像が planner 経路に迂回を発生**
させる頻度が上昇。原因は 2D raytracing のジオメトリ限界:

- **別の障害物の陰**: 手前の実在物でビームが残像まで届かない
- **高さの問題**: raytracing は `/scan` の 2D 平面で行われるため、遠方
  の床近くにはビームが物理的に存在しない (VLP-16 の下向きビームは
  近距離で地面に当たる)。人の足元 (z 低め) の残像は距離が離れるほど
  掃除ビーム不在

これは設定でなくジオメトリの限界。デモ対策の役割分担:

- **安全 (chair を止める)**: Layer D (生 scan 直視) が担う。costmap は
  見ない → 幽霊で誤停止しない、実在の人だけを止める
- **経路品質 (幽霊で迂回しない)**: `cost_scaling_factor` を 3.0 → 5.0
  で inflation 裾を圧縮 (radius=0.5 は robot_radius=0.45 との差 +0.05
  のギリギリなので radius は下げない)。幽霊 1 セルの経路への影響が
  減少
- **完全解決の見送り**: `spatio_temporal_voxel_layer` (時間減衰プラグ
  イン) 差し替えはデモ前に検証時間を捻出できず却下、post-demo backlog

### V2/V3 追加観察項目 (夜間残像との切り分け)

field で以下を確認 (Layer D が costmap でなく生 scan を見ていることの
実証):

- 人が sector 外へ退避 → 1 s 以内に Layer D 解放 → chair 走行再開
- 退避後、costmap には**残像が紫のまま残っていてよい**
- **残像で Layer D が再発火しないこと** (再発火すれば Layer D が誤って
  costmap 参照している = 実装バグ)

### post-demo backlog

- `spatio_temporal_voxel_layer` (voxel + temporal decay) への差し替え
  検討 (ADR 別立て)。3D voxel 表現で近似的に高さ問題も緩和される可能性

### 運用ゲート (デモ手順に必須)

走行前 (bringup ~20 秒後):

```bash
# collision_detection の effective 値
ros2 param get /controller_server FollowPath.use_collision_detection
# 期待: Boolean value is: true

# Layer D armed の startup log 確認
ros2 topic echo /rosout | grep -E "failsafe_node ready|forward_blocked"
# 期待: "forward_blocked > 5 pts in ±30° @ 0.5-2.0 m, hysteresis 0.5s"

# 動作テスト (手を前方に翳して 2 秒待つ)
ros2 topic hz /cmd_vel_safety
# 期待: 遮断中は 20 Hz publish
```

デモ準備チェックリストに追記済 (`docs/ja/m6r-demo-prep-checklist.md`)。

### Accepted 化条件

以下 3 点満了で proposed → accepted:

1. 明朝 field で V2/V3 (Layer D 基準) PASS
2. V6.4 (sector geometry 実測) PASS
3. 30 min 連続走行 (V4) で false-trip 0 (path 沿いの静止建物 / 木で発火しないこと)
