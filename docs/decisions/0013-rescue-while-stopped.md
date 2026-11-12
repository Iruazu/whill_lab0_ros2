# ADR 0013: 停止中の救援操作（Layer D 作動中の teleop）

- Status: **proposed**（提案。採択はユーザーが行う）
- Date: 2026-11-12
- Deciders: ユーザー（承認待ち）
- 課題: #161

## 背景

twist_mux の優先度は safety 100 > teleop 50 > navigation 10（`src/whill_safety/config/twist_mux.yaml`、ADR-0007 §teleop スロット有効化）。failsafe はどれかの層が作動している間、`/cmd_vel_safety` にゼロを 20 Hz で出す。teleop は safety より下なので、作動中は手動救出の指令も通らない。

ADR-0007 はこれを「手動操縦中でも Layer D の前方歩行者停止は生きたまま」という安全機構として採った。一方で、Layer D（前方 ±30°、1〜2 m に点が1点以上）は、前に物がある限り解けない。物が動かない場合（壁、置かれた物、地図に無い障害物）、後退や回転で離れる操作まで止まり、車体を動かすには WHILL 本体のジョイスティックしか無い。

現状の経路（コードで確認）:

- Web（`web/app.js`）は押している間だけ `/dispatch/teleop` に `{vx, wz}` を 10 Hz で送る。前進 0.2 m/s、後退 −0.2 m/s、回転 ±0.4 rad/s
- `dispatch_node` は ±0.3 m/s・±0.6 rad/s に制限して `/cmd_vel_teleop` に出す。指令が 0.4 秒途絶えるとゼロを1回出して止める。無操作 20 秒で手動モードを解除する
- twist_mux の teleop のタイムアウトは 0.5 秒
- `/cmd_vel` の後段の velocity_smoother は後退を −0.1 m/s、回転を ±0.5 rad/s に制限する（`nav2_params.yaml`）

Collision Monitor（C3、`feat/avoidance-c3`、main には未マージ）は smoother の後ろに置く計画で、twist_mux の優先度とは無関係に、停止域に点があれば全指令を止める。本 ADR の実装は main の failsafe だけを対象とする。

研究基盤への方針変更（2026-11-12、CLAUDE.md）後も、方策（VLA / VA）を実機で動かすときに同じ問題が起こる。前方を塞がれた状態から人が車体を離す手段は要る。

## 決定（推奨案。採択されるまで既定は無効）

**案 A: Layer D だけが作動しているとき、前進を含まない teleop 指令に限って failsafe がゼロ指令を控える。** パラメータ `layer_d_rescue_enabled`（既定 false）で有効にする。既定では購読も増えず、今と同じ動作になる。

有効時に、ゼロ指令を控える条件（すべて満たすとき）:

1. 作動している層が `D:forward_blocked` だけ。A・B・C（と #159 の未着入力）のどれかがあれば今どおり止める
2. 最後の `/cmd_vel_teleop` が `rescue_cmd_timeout_s`（既定 0.3 秒）以内に届いている
3. その指令が前進を含まない: `linear.x <= 0`、`linear.x >= -rescue_max_reverse_mps`（既定 0.2）、`|angular.z| <= rescue_max_wz`（既定 0.6）、y・z の並進と x・y の回転が 0。範囲外は救援と見なさず止める（制限ではなく拒否）
4. 救援中に、twist_mux の出力 `/cmd_vel` に前進（`linear.x > 0`）が現れていない。現れたら Layer D が解けるまで救援を止める（ラッチ）

条件の根拠:

- 2 の 0.3 秒は twist_mux の teleop タイムアウト 0.5 秒より短い。teleop が途絶えると、mux の teleop 枠が切れる前に failsafe がゼロを再開するので、Nav2（優先度 10）に切り替わる隙間ができない。10 Hz の送信なら2回の欠落まで許す
- 4 は、failsafe と twist_mux の受信が食い違ったとき（failsafe は teleop を受けたが mux は受けていない、など）に Nav2 の前進が通るのを止める。食い違いを直接は見られないので、mux の出力を見る
- 前進の指令が来たら、tick を待たず受信時にゼロを出す。mux が先に前進を1回通す余地は残る（下記）

実装: `src/whill_safety/whill_safety/failsafe_node.py`（`rescue_command_ok()`、`rescue_passes()`）。launch 引数 `failsafe_layer_d_rescue`（`m6r_bringup_launch.py`、`safety_launch.py`、既定 false）。救援の開始・終了はログに `RESCUE PASS` / `RESCUE BLOCKED` / `RESCUE LOCKED` / `RESCUE ended` と出る。

## 検証（実機なし、2026-11-12）

- 単体テスト `src/whill_safety/test/test_failsafe_rescue.py`（24件）: 指令の判定、Layer D 単独のときだけ通ること、鮮度、ラッチと解除、既定で購読しないこと、不正なパラメータの拒否。`config/twist_mux.yaml` を読み、teleop のタイムアウトが `rescue_cmd_timeout_s` より長いことと優先度の順も固定する
- 本物の failsafe_node と twist_mux（`safety_launch.py`、domain 159、localhost）に、前方 1.5 m を塞ぐ `/scan`、Nav2 の代わりの `/cmd_vel_nav` 前進 0.3 m/s（20 Hz）、teleop（10 Hz）を流し、`/cmd_vel` を記録した
  - 既定（false）: 後退・回転・前進のどれも `/cmd_vel` はゼロだけ
  - 有効（true）: 後退中は −0.2、回転中は 0.4 rad/s が出た。前進に切り替えると、上記の競合で前進 0.2 が1回出たあと、ゼロだけになった。teleop を止めた後もゼロだけで、Nav2 の前進は出なかった
  - どちらの試行でも、開始直後（最初の `/scan` より前）に Nav2 の前進 0.3 が2回出た。救援とは無関係な、この試験の立ち上げ順によるもの

## 採用しなかった案

**案 B: 救援専用の topic を twist_mux の最上位（例: `/cmd_vel_rescue`、優先度 110）に置き、速度の制限と dead-man を持つ中継ノードを通す。** 仕組みは単純で、Layer B（自己位置の喪失）や C（認識の途絶）の停止中にも救援できる。しかし failsafe の全層を上書きするので、中継ノードの不具合や誤送信がそのまま安全層の無効化になる。前方に人がいても前進できてしまうので、少なくとも前進の拒否を中継ノード側に作り直す必要がある。将来の遠隔停止・E-stop を twist_mux の topic で入れる場合は、その優先度を 110 より上にする制約も増える。

**案 C: 停止中は後退だけを許す（回転は止める）。** 案 A の条件 3 を `angular.z == 0` に狭めたもの。回転で前方の物を扇形の外に出せないので、壁際での向き直しができない。案 A でも `rescue_max_wz` を 0 にすれば同じ動作になる。

**案 D: twist_mux の lock 機能で safety 枠を止める。** lock は下位の全入力を止める仕組みで、上位の safety を外す用途に使えない。

## 結果

得るもの:

- 前方を塞がれた停止から、Web の手動救出で後退・回転して離れられる（有効化した場合）
- 前進は今どおり Layer D が止める。A・B・C の停止は今どおり teleop も止める

失うもの・残る危険:

- 後方の監視は無い。Layer D は前方だけを見る。後退中に後ろの人や物に当たる危険は、Layer D が作動していないときの teleop と同じで、並走者の監視に頼る
- 回転中も前方以外は見ていない。その場回転は並進しないので、1 m 以上先の物には当たらない。1 m 未満は Patchwork++ の `min_range` で `/scan` に元々無い
- 後退から前進へ切り替えた瞬間、failsafe と twist_mux の受信順によっては、mux が前進を1回（〜50 ms）通し得る。smoother の加速 0.3 m/s² で約 0.015 m/s。条件 4 で救援はラッチされる
- failsafe が `/cmd_vel_teleop` と `/cmd_vel` を購読する。救援の可否が failsafe の受信に依存する
- 救援中は failsafe の作動ログ（ENGAGED）が出たまま `/cmd_vel_safety` が止まる。ログに `RESCUE PASS` / `RESCUE BLOCKED` を出して区別する

後続:

- 実機での確認（未実施）: 前方に物を置いて Layer D を作動させ、(1) 前進が止まる、(2) 後退・回転が通る、(3) 指を離すと 0.5 秒以内にゼロに戻り Nav2 に戻らない、(4) 配車中（Nav2 が指令を出している）でも Nav2 の前進が通らない
- Collision Monitor（C3）をマージするときは別に決める。Humble の Collision Monitor は停止域に点があれば方向によらず止める。案としては、停止多角形を前方だけに置いて後退は `FootprintApproach`（指令の軌跡で判定）に任せる、救援中は停止多角形を無効にする、など。どちらも Humble での可否を確かめていない
- Layer B・C の停止中の救援（自己位置を使わない手動操作）は、本 ADR の範囲外。必要なら案 B と合わせて別に決める
