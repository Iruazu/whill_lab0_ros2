# ADR 0012: whill_dispatch の Web 境界インターフェース方式 (M7)

Language: [日本語](0012-dispatch-web-interface.md) | (English 版は demo 後に必要になれば起こす)

- Status: **proposed** (2026-07-19 M7 最小実装と同時起草。実機での実配線確認後に accepted 化)
- Date: 2026-07-19
- Deciders: ユーザー + pm-orchestrator (計画: `docs/ja/plans/2026-07-19-m7-dispatch.md`)

## 背景 (Context)

platform-pivot §3.5 は Web / タブレット UI と ROS 2 の間に `whill_dispatch` を
境界として置くと定めたが、境界を流れるメッセージの型付け方式は未決だった
(§7 ADR 候補「Web 接続方式」)。2026-07-20 の実機検証で web app からの配車を
使うため、本日中に動く最小構成を選ぶ必要がある。

制約:

- 本日実機なしで mock E2E まで完成させる (rosidl ビルドや独立 API サーバの
  導入・デバッグに割ける時間がない)
- タブレットのブラウザで URL を開くだけで動く (npm/webpack ビルド無し)
- 認証・複数台管理は将来要件であり、今日は載せないが、載せられる境界を保つ

## 決定 (Decision)

**方式 A: JSON-over-`std_msgs/String` + `std_srvs/Trigger` + rosbridge_suite 直結**
を採用する。

- Web → ROS: `/dispatch/submit` (`std_msgs/String`, JSON payload)、
  `/dispatch/cancel` (`std_srvs/srv/Trigger`)
- ROS → Web: `/dispatch/state` (~5 Hz)、`/dispatch/waypoints` (1 Hz 再送)。
  いずれも `std_msgs/String` の JSON
- latched (transient_local) には頼らず定期再送で roslibjs の volatile 既定と
  折り合いを付ける
- Web は Nav2 の action や `/pcl_pose` を直接触らない。境界は
  `/dispatch/*` のみ (行動規範 #4 の帰結)

## 採用しなかった案 (Alternatives)

- **方式 B: 型付き custom interface (`whill_dispatch_interfaces` + rosidl)**。
  型安全と補完が得られるが、rosidl パッケージの新設・ビルド検証に時間を要し、
  rosbridge 越しでは結局 JSON 化されるため本日の利得が薄い。認証・複数台の
  M7 本実装で再評価する
- **方式 C: 独立ゲートウェイ API (FastAPI 等) + ROS ノード**。認証・セッション
  管理を載せる最終形に近いが、プロセス・依存が増え本日中に収まらない。
  platform-pivot §3.5 の通り「認証・複数台管理が視野に入った時点で ADR で判断」
  に委ねる

## 結果 (Consequences)

- 得るもの: 本日中の E2E 完成。Web 側は roslibjs のみで依存が閉じる。
  境界 topic が `/dispatch/*` に一元化され、後日方式 B/C へ置換しても
  Web 側の概念モデル (submit/cancel/state/waypoints) は保存される
- 失うもの: 型検査。payload schema は `src/whill_dispatch/README.md` に
  文書として固定し、dispatch_node 側で受信 JSON を検証して不正 payload を
  reject する (schema 崩れの検知をランタイムに移す)
- 後続作業: 認証・複数台管理の要件が具体化した時点で方式 B/C を再評価する
  ADR を起こす。JSON schema が肥大し始めたらそれが移行シグナル

## 追補: `/plan` の可視化目的 read-only 購読を許可する (提案, 2026-07-20)

- 位置づけ: 本 ADR「Web は `/dispatch/*` のみに接触する」の**例外を 1 件明示する**
  提案。Status は変更しない (本 ADR 全体は依然 proposed)。M7 web app v2
  (issue #110, 地図タップ配車 + 経路可視化) の実装と同時に起草。
- 背景: 経路 (plan) を UI に描くには Nav2 global planner の `/plan`
  (`nav_msgs/Path`) が必要。dispatch_node が中継する案もあったが、rosbridge は
  既に全 topic に接続でき、中継はノードに状態と遅延を足すだけなので、UI から
  `/plan` を直接購読する方針をユーザーが承認した。
- 判断: `/plan` 購読は **read-only の可視化**であり、コマンドは一切逆流しない
  (submit/cancel は従来どおり `/dispatch/*` のみ)。行動規範 #4 が禁じるのは
  「配車・Web 層のロジックを Nav2/localization ノードに密結合で書くこと」= 操作
  境界の越境であって、状態の観測ではない。したがって操作境界としての
  `/dispatch/*` 一元化は保たれる。
- 制約: この例外は `/plan` の**購読**に限る。Web から Nav2 の action・service・
  パラメータ・`/cmd_vel*` を触ることは引き続き禁止。可視化のために別の Nav2
  topic を増やす場合は、都度この追補に列挙して read-only であることを明記する。
- 将来: 方式 B/C (型付き interface / 独立ゲートウェイ API) へ移行する際は、
  `/plan` を含む可視化フィードを dispatch/ゲートウェイ側で束ねて再 publish し、
  Web の接触面を再び境界内に閉じる余地がある。今日はやらない。
