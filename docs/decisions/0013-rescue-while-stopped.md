# ADR 0013: 停止中の救援操作（Layer D 作動中の teleop）

- Status: **proposed**（提案。採択はユーザーが行う）
- Date: 2026-11-12
- Deciders: ユーザー（承認待ち）
- 課題: #161
- 改める決定: ADR-0007 §teleop スロット有効化「優先度の含意」の不変条件「手動操縦中でも Layer D の前方歩行者停止は生きたまま」。本 ADR を採択して有効にすると、Layer D 単独の停止中は、前進を含まない teleop 指令が failsafe 経由で通る。前進は今どおり止まる

## 背景

twist_mux の優先度は safety 100 > teleop 50 > navigation 10（`src/whill_safety/config/twist_mux.yaml`、ADR-0007 §teleop スロット有効化）。failsafe はどれかの層が作動している間、`/cmd_vel_safety` にゼロを 20 Hz で出す。teleop は safety より下なので、作動中は手動救出の指令も通らない。

ADR-0007 はこれを安全機構として採った。一方で Layer D（前方 ±30°、1〜2 m に点が1点以上）は、前に物がある限り解けない。物が動かない場合（壁、置かれた物、地図に無い障害物）、後退や回転で離れる操作まで止まり、車体を動かすには WHILL 本体のジョイスティックしか無い。

現状の経路（コードで確認）:

- Web（`web/app.js`）は押している間だけ `/dispatch/teleop` に `{vx, wz}` を 10 Hz で送る。前進 0.2 m/s、後退 −0.2 m/s、回転 ±0.4 rad/s
- `dispatch_node` は ±0.3 m/s・±0.6 rad/s に制限して `/cmd_vel_teleop` に出す。指令が 0.4 秒途絶えるとゼロを1回出して止める。無操作 20 秒で手動モードを解除する
- twist_mux の teleop と safety のタイムアウトは 0.5 秒
- `/cmd_vel` の後段の velocity_smoother は、並進を [−0.1, 0.3] m/s、回転を [−0.8, 0.5] rad/s に制限する（`src/whill_navigation/config/nav2_params.yaml` の `min_velocity` / `max_velocity`）

Collision Monitor（C3、`feat/avoidance-c3`、main には未マージ）は smoother の後ろに置く計画で、twist_mux の優先度とは無関係に、停止域に点があれば全指令を止める。本 ADR の実装は main の failsafe だけを対象とする。

研究基盤への方針変更（2026-11-12、CLAUDE.md）後も、方策（VLA / VA）を実機で動かすときに同じ問題が起こる。前方を塞がれた状態から人が車体を離す手段は要る。

## 決定（推奨案 E。採択されるまで既定は無効）

**案 E: Layer D だけが作動しているとき、前進を含まない teleop 指令を failsafe 自身が `/cmd_vel_safety`（優先度 100）に中継する。** 条件を1つでも満たさなくなったら、その時点でゼロを出す。パラメータ `layer_d_rescue_enabled`（既定 false）で有効にする。既定では購読も増えず、今と同じ動作になる。

中継する条件（すべて満たすとき）:

1. 作動している層が `D:forward_blocked` だけ。A・B・C と、#159 の `*_never_arrived` のどれかがあればゼロ
2. 最後の `/cmd_vel_teleop` が `rescue_cmd_timeout_s`（既定 0.3 秒）以内に届いている
3. その指令が前進を含まない: `-rescue_max_reverse_mps (0.2) <= linear.x <= 0`、`|angular.z| <= rescue_max_wz (0.6)`、他の成分が 0、すべて有限。範囲外は救援と見なさずゼロ（制限して通すのではなく拒否）

中継の内容は、検証した teleop 指令そのもの。前進など条件3を満たさない指令が来たら、tick を待たず受信時にゼロを出す。

Layer D が中継中に解けたとき: teleop が条件2・3を満たす間は中継を続け、満たさなくなったらゼロを1回出してから止める。すぐ黙ると、twist_mux の safety 枠に最後の中継指令が 0.5 秒残り、その間に来る操作者の離しのゼロ（teleop 枠で、safety より下）が落ちるため。

時間の条件: `rescue_cmd_timeout_s + 1/publish_hz + 余裕 0.1 秒 < twist_mux の teleop タイムアウト 0.5 秒` を起動時に確かめ、満たさなければ起動しない。中継は最後の teleop から最大でこの時間だけ続くので、twist_mux が teleop 自体を受け付ける時間より短く保つ。

この方式なら safety 枠は作動中ずっと生きているので、Nav2（優先度 10）も前進の teleop（50）も、構成上 `/cmd_vel` に出られない。

実装: `src/whill_safety/whill_safety/failsafe_node.py`（`rescue_command_ok()`、`rescue_relay_allowed()`、`check_rescue_timing()`、`_rescue_output()`）。launch 引数 `failsafe_layer_d_rescue`（`m6r_bringup_launch.py`、`safety_launch.py`、既定 false）。ログに `RESCUE relay` / `RESCUE ended — <理由>` を出す。C3 の `/failsafe/state` は main に無いので、C3 の取り込み時に中継中の状態をそこへ載せる。

## 検証（実機なし、2026-11-12）

- 単体テスト `src/whill_safety/test/test_failsafe_rescue.py`（27件）: 指令の判定、Layer D 単独のときだけ中継すること（#159 の層も含む）、中継の内容が teleop と同じこと、前進・鮮度切れ・他の層でゼロになること、Layer D が解けた後の中継と最後のゼロ1回、tick を含む時間の条件、既定で購読しないこと、不正なパラメータの拒否。`config/twist_mux.yaml` を読み、teleop のタイムアウト・優先度の順・safety のタイムアウトが tick より長いことも固定する
- 本物の failsafe_node と twist_mux（`safety_launch.py`、domain 159、localhost）に、前方 1.5 m を塞ぐ `/scan`、Nav2 の代わりの `/cmd_vel_nav` 前進 0.3 m/s（20 Hz）、teleop（10 Hz）を流し、`/cmd_vel` を記録した
  - 既定（false）: 塞いでいる間は、後退・回転・前進のどれも `/cmd_vel` はゼロだけ
  - 有効（true）、塞いだまま: 後退は −0.2、回転は 0.4 rad/s が出た。前進に切り替えると前の回転が2回出てからゼロだけになり、前進は1回も出なかった。指を離すと後退 −0.2 が6回（約 0.3 秒）出てからゼロだけになり、Nav2 の前進は出なかった
  - 有効、後退中に塞ぎを外す: 後退は途切れずに続き、指を離すと後退5回・ゼロ1回の後、約 0.5 秒で Nav2 の前進 0.3 に戻った（Layer D が解けているので正常）
  - どちらの試行でも、開始直後（最初の `/scan` より前）に Nav2 の前進 0.3 が2回出た。救援とは無関係な、この試験の立ち上げ順によるもの
- 再現できていないもの: failsafe の executor の遅れ（案 A の不採用理由）を人為的に起こす試験はしていない。案 E は safety 枠が切れないことを構成で保つので、遅れは中継の継続時間とゼロへの切替の遅れにだけ効く

## 採用しなかった案

**案 A: 条件1〜3を満たす間、failsafe がゼロ指令を控え、teleop を twist_mux の teleop 枠で通す（最初の実装）。** レビューで不採用（MF-2）。ゼロを控えている間の安全は、failsafe が最後の teleop を処理した時刻と twist_mux が受けた時刻が 0.3 秒 + tick 以内に揃うことに頼る。failsafe の executor が 0.2 秒遅れる（GIL、GC、CPU）だけで、failsafe が黙ったまま twist_mux の teleop 枠（0.5 秒）が切れ、`/cmd_vel_nav` の前進 0.3 m/s（20 Hz）が検知した障害物へ向けて通る。対策の `/cmd_vel` 監視によるラッチは事後で、同じ遅れた executor で動く。前進の teleop が漏れる量も1回に限られない（実測では切替時に1回出た）。

**案 B: 救援専用の topic を twist_mux の最上位（例: `/cmd_vel_rescue`、優先度 110）に置き、速度の制限と dead-man を持つ中継ノードを通す。** Layer B・C の停止中にも救援できる。しかし failsafe の全層を上書きするので、中継ノードの不具合や誤送信がそのまま安全層の無効化になる。前方に人がいても前進できるので、前進の拒否を中継ノード側に作り直す必要がある。将来の遠隔停止・E-stop を twist_mux の topic で入れる場合、その優先度を 110 より上にする制約も増える。

**案 C: 停止中は後退だけを許す（回転は止める）。** 回転で前方の物を扇形の外に出せないので、壁際での向き直しができない。案 E でも `rescue_max_wz` を 0 にすれば同じ動作になる。

**案 D: twist_mux の lock 機能で safety 枠を止める。** lock は下位の全入力を止める仕組みで、上位の safety を外す用途に使えない。

## 結果

得るもの:

- 前方を塞がれた停止から、Web の手動救出で後退・回転して離れられる（有効化した場合）
- 前進は今どおり止まる。A・B・C の停止は今どおり teleop も止める
- Nav2 と前進の teleop は、failsafe の処理遅れがあっても safety 枠に阻まれる

残る危険（安全の根拠ではなく、受け入れるかどうかを決める項目）:

- **その場回転の掃引**: 回転すると車体の外形が回転半径の内側を掃く。この範囲（1 m 未満）は Patchwork++ の `min_range: 1.0` により `/scan` に元々点が無く、Layer D も見ていない。回転中にすぐ横の人や物に当たる危険は残る
- **後方の無監視**: Layer D は前方だけを見る。後退中に後ろの人や物に当たる危険は、並走者の監視に頼る（Layer D が作動していないときの teleop と同じ）
- **離した後の残り**: 中継は最後の teleop から最大 `rescue_cmd_timeout_s` + 1 tick（既定 0.35 秒）続く。実測で、指を離した後に後退 −0.2 が 6 回（約 0.3 秒）出た。smoother が後退を 0.1 m/s に制限するので、余分な移動は数 cm
- **救援の可否が failsafe の受信に依存する**: failsafe が `/cmd_vel_teleop` を受け損ねるとゼロになる（安全側）
- **通信の揺らぎ**: Web → rosbridge → dispatch の経路で 0.3 秒を超える遅れ・欠落があると、中継とゼロが交互になり、救援がぎくしゃくする（安全側）

実機確認項目（未実施）:

- 前方に物を置いて Layer D を作動させ、(1) 前進が止まる、(2) 後退・回転が通る、(3) 指を離すと約 0.35 秒以内にゼロに戻り Nav2 に戻らない、(4) 配車中（Nav2 が指令を出している）でも Nav2 の前進が通らない、(5) 回転時の車体の掃引範囲と周囲の距離、(6) Wi-Fi 越しの Web 操作で救援が途切れないか

後続:

- 初回受信後の `/scan` 途絶を Layer D が監視しない既存の欠陥（`D:scan_silent` の追加）は、本 ADR を有効にする前に直す。`/scan` が途絶えると Layer D 自体が作動しなくなり、救援以前に停止の前提が崩れる
- Collision Monitor（C3）をマージするときは別に決める。Humble の Collision Monitor は停止域に点があれば方向によらず止める。案としては、停止多角形を前方だけに置いて後退は `FootprintApproach`（指令の軌跡で判定）に任せる、救援中は停止多角形を無効にする、など。どちらも Humble での可否を確かめていない
- Layer B・C の停止中の救援（自己位置を使わない手動操作）は、本 ADR の範囲外。必要なら案 B と合わせて別に決める
