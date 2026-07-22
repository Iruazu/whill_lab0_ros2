# 2026-07-22 seimon 方向 立ち往生の bag 解析

- bag: `rosbag2_2026_07_22-18_01_15/` (875.9 MiB, 132.5 s, 本ディレクトリ・未 commit)
- 症状: bldg7 付近から seimon へ配車中、(-63.2, -0.9) 付近でその場左右回頭を
  繰り返して前進しない (現地報告は「道の狭さ故に立ち往生」)
- 起動構成: `map_variant:=v2 speed:=normal`、#120-#124 適用後
- 解析スクリプト: セッション scratchpad の `analyze_seimon_stall.py` /
  `analyze_stall_window.py` / `analyze_obstacle.py` (使い捨て。手法は下記に転記)

## タイムライン (pcl_pose 10 s ビン)

| 区間 | 移動距離 | yaw 振れ幅 | 状態 |
|------|---------|-----------|------|
| 30-80 s | 4.5-6.0 m / 10 s | 9-33° | 正常走行 (~0.5 m/s) |
| 80-130 s | 1.8-3.5 m / 10 s | 66-96° | (-63.1, -0.9) で停滞・その場回頭 |

- cmd_vel_nav: 1041 msgs。停滞窓では vx=0 が支配的 (全区間で zero 率 41%)、
  wz は +0.6 ⇔ -0.65 を ~5 s 周期で交互反転、計 78 回 / 105 s
- `/cmd_vel_safety`: **0 件** — Layer D は一度も作動していない (安全層は無関係)
- `/plan`: 83 回発行され続け、残長 25.6 m で一定 — global 計画は常に成功
- mux 出力 /cmd_vel は nav とほぼ同数 (1019 vs 1041) — 遮断なし

## 原因の切り分け

「道の狭さ (実障害物)」説は棄却:

1. 停滞地点の local_costmap 通行幅は lethal 基準 8.5-10 m、経路上の cost は全点 0
2. cost≥90 セルの最近傍は robot から 1.2 m だが、**同時刻の生 /scan は同方向で
   最短 3.1-3.6 m** — 現実の障害物に対応しない
3. その cost≥90 位置は静的地図 (v2/final.pgm) では FREE (254) だが、**塗り境界
   (y=+0.60、以遠 UNKNOWN 205) から 0.27-0.42 m** = robot_radius 0.45 の
   inscribed 帯。正体は「塗り境界の inflation スカート」

環境側の根本原因: **塗り漏斗**。塗り幅が x=-58 の 4.45 m から x=-65 の 3.75 m へ
窄まり、両側 inscribed (0.45×2) を引いた有効走行帯は ~2.9 m。一方、生の観測地図
(occupancy.pgm) は同区間で北側 y=+4.0 まで FREE — 実道路は塗りより ~3 m 広い
(live /scan の 3 m クリアとも整合)。塗りが保守的すぎて仮想の狭路を作っていた。

制御側の増幅要因: 蛇行で北縁 inscribed 帯に頭が向く → RPP collision 検知で vx=0
→ 方位誤差 > 45° で rotate-to-heading (wz 0.8 指令) → WHILL の yaw 応答遅れで
オーバーシュート (指令 0.65 rad/s×4 s=150° 相当に対し実 yaw ~50°) → 逆回頭。
10 s ごとに progress checker abort → spin/backup リカバリが回頭を追加、のループ。

## 対策 (本 PR)

1. **塗り拡幅** (根本): x∈[-66,-57] の北縁を、生地図で現塗り縁から連続 FREE が
   観測されている列に限り (生 FREE 上端 − 0.3 m マージン) と y=+1.5 の小さい方
   まで 205→254。OCC (≤100) は不変更 (検証で flip 0 を確認)。2896 px = 7.2 m²。
   拡幅後の通行幅 4.7-5.25 m。web map.png / map_meta.json も再生成済
2. **rotate_to_heading_angular_vel 0.8 → 0.4** (増幅抑制): オーバーシュート半減。
   45° 回頭 ~2 s で progress 10 s 猶予内

## 再走 (bag 2: rosbag2_2026_07_22-18_23_13, 174 s, 雨で中断)

塗り拡幅 + rotate 0.4 適用後の再走。「あまり改善なし」— (-61.4, -0.8) で再停滞。
120 s 以降は雨退避のジョイスティック手動走行 (1.5 m/s、解析対象外)。

bag 2 で確定した事実:

- 新塗りは反映済み (costmap 北縁 y=+1.23 — 旧塗りなら +0.15)。通行幅 ~4.9 m
- rotate 0.4 も反映済み (rotate 中 wz median 0.30)
- Layer D 819 発報はほぼ全て 130 s 以降の手動退避中 (前を歩く人に正常反応)。
  停滞窓 (85-120 s) は 34 発のみ。雨クラッタも scan 345 本中 7 本で否定
- **決定打**: t=111.0 s、方位誤差 -1.2° (完全整列) の瞬間を wz 0.8 のまま
  素通りして -42° まで回転継続 — これは経路を見ない固定 1.57 rad の
  **Spin リカバリ**。停止ループの支配項は rotate ではなく Spin だった

改訂した因果連鎖: 蛇行で誤差 45° 超 → rotate_to_heading (vx=0) → 並進 0 の
まま progress checker 10 s が abort → Spin が整列を無視して 90° 回転 →
大誤差を再生産 → 以降ループ。

## 追加対策 (bag 2 反映)

3. **カスタム BT** (`whill_navigation/config/navigate_to_pose_no_spin_bt.xml`):
   リカバリから Spin (整列破壊を実測) と BackUp (後方センサなしの盲目後退)
   を除去し、ClearingActions + Wait のみ残す。nav_launch が per-run yaml に
   `default_nav_to_pose_bt_xml` を焼き込む
4. **rotate_to_heading_min_angle 0.785 → 1.57**: 蛇行由来の 40-50° 誤差では
   vx=0 の rotate に入らず、前進しながら曲率で戻す (ループの入口を塞ぐ)
5. **movement_time_allowance 10 → 15 s**: 0.4 rad/s の正当な U ターン回頭
   (~8 s + ramp) を abort しないマージン。停止検知の遅れは Layer D が別途
   担保するため安全性への影響なし

## 実機確認手順 (次回)

同じ配車 (bldg7 → seimon) を再実行し:

- (-59〜-66) 区間を停止せず通過すること (bag1 は -63.2、bag2 は -61.4 で停滞)
- 停止しても spin (その場一回転) が発生しないこと
- 万一まだ停滞する場合は同じ topic 構成で bag を再取得。次の容疑は蛇行の
  根本 (lookahead / 速度 0.45 での再較正) と wet 路面のスリップ
