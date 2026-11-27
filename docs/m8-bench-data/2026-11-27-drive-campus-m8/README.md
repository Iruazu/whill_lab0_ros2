# 2026-11-27 campus-m8 実機走行（自己位置と走行指示の確認）

> この README と `summary.json` は `~/bags/2026-11-27-m8-drive/` からの写し（2026-11-27）。bag 本体・ログ・解析スクリプト・姿勢の CSV は `~/bags/2026-11-27-m8-drive/` にあり、リポジトリには入れていない。本文中の `analysis/`・`logs/`・`scripts_used/` はその場所からの相対パス。`summary.json` はこのディレクトリに置いた。

- 地図: campus-m8 / variant candidate（154w で広げた版、`candidate_final.pgm` sha256 `fca2abb4…`）。NDT の点群 `static.pcd` sha256 `d2448658…`
- コード: main `64f7a49`（#211 マージ後）。`scripts_used/git_main_head.txt`
- 構成: `start_dispatch_demo.sh --speed slow`（bringup + Nav2 + 配車 Web + RViz、failsafe / twist_mux 有効、0.30 m/s）。カメラなし
- 走行の指示: RViz の Nav2 Goal（配車 Web の job は 0 件。`logs/*/dispatch.log`）
- 記録: `scripts/record_avoidance_bag.sh --profile drive`（mcap）。`cmd_vel_teleop` は全区間 0 件で、ジョイスティックの介入は記録されていない
- 起動は 2 回（15:49 と 16:05）。ログは `logs/dispatch-demo-20261127-154932-08Z01t/`（1 回目）、`logs/dispatch-demo-20261127-160516-ASO8ww/`（2 回目）。`log/` からの複製

## bag

| bag | 時刻（JST） | 長さ | 大きさ | 起動 | 内容 |
| --- | --- | --- | --- | --- | --- |
| `start_static_155338` | 15:53:38〜 | 60 s | 2.2 GiB | 1 回目 | 発進点で停止したまま。初期位置の後 |
| `drive01_155523` | 15:55:23〜 | 54 s | 2.1 GiB | 1 回目 | goal (-0.96,-0.52)→(7.95,-0.16)：成功（15:56:04） |
| `drive02_155728` | 15:57:28〜 | 53 s | 2.0 GiB | 1 回目 | goal →(23.06,0.31)：成功（15:58:12） |
| `drive03_155854` | 15:58:54〜 | 240 s | 8.4 GiB | 1 回目 | goal →(102.07,5.87)：(64.5,5.1) 付近で止まる。Failed to make progress ×5（bag の /rosout）、D:forward_blocked。16:02:44 に (-68.05,-2.04) へ goal を差し替えたが進まず、16:03:15 に取消（bag の終わりの後） |
| `drive04_161045` | 16:10:45〜 | 215 s | 7.8 GiB | 2 回目 | goal (-36.61,-0.99)→(-91.32,29.49)：(-84.0,-0.4) 付近で止まる。RPP の collision ahead ×5、Failed to make progress ×2、D:forward_blocked。16:14:25 に取消 |

**記録していない区間:** 2 回目の起動の直後、16:06:21〜16:10:02 の goal (28.89,0.46)→(-36.99,-0.84)（約 66 m）は成功したが bag が無い（`logs/…160516…/nav2.log` にだけ残る）。1 回目の終点 (64.5,5.1) から 2 回目の始点 (28.89,0.46) までの移動も記録なし。

**終わりの後のログ:** 16:15:07 から WHILL の serial の write() error、16:15:42 から LiDAR の poll() timeout、16:17:35 から IMU の readSensorData FAILURE が出る。機器を外した後で、走行中の現象ではない。その後の `B:pcl_pose_silent` / `B:fitness` もこれに続くもの。

## 自己位置（`analysis/summary.json`）

`check_localization.py`（ブランチ feat/localization-check-script、`scripts_used/check_localization_commit.txt`）の判定を、bag の記録時刻で掛けた。全 bag で PASS（failsafe の Layer B に掛かる状態なし）。

| bag | fitness 中央値 / p95 / 最大 | 収束しない | 却下の連続 | failure_category | 姿勢の間隔の最大 | NDT の移動（1 s 間引き） |
| --- | --- | --- | --- | --- | --- | --- |
| start_static | 0.020 / 0.032 / 0.039 | 0 | 0 | healthy 全件 | 0.13 s | 停止中のばらつき σ x 0.2 cm・y 0.3 cm |
| drive01 | 0.022 / 0.044 / 0.060 | 0 | 0 | healthy 全件 | 0.13 s | 8.7 m |
| drive02 | 0.024 / 0.060 / 0.099 | 0 | 0 | healthy 全件 | 0.13 s | 13.0 m |
| drive03 | 0.063 / 0.092 / 0.160 | 0 | 0 | healthy 全件 | 0.14 s | 45.1 m |
| drive04 | 0.043 / 0.076 / 0.134 | 0 | 0 | healthy 全件 | 0.14 s | 46.3 m |

- 参考（bag の再生、README 4 節）: 中央値 0.015〜0.016、p95 0.038〜0.065。発進点付近（static・drive01・02）は同じ水準。drive03・04 は中央値で 3〜4 倍だが、failsafe の閾値 1.0 の 1/6 以下
- drive03 の停止中の fitness（中央値 0.086）は走行中（0.034）より高い。止まった場所（x 64 付近）の当てはまりが悪いと見られる（推測、未確認）
- drive03 で、0.1 s の間に 0.28〜0.40 m・3.7〜4.9° 動いた姿勢が 6 回ある（t=14.4・15.0・21.3・57.0・57.4 s ほか）。どれも Nav2 の角速度の指令が ±0.5〜0.8 rad/s で左右に振れた直後で、fitness が 0.13〜0.16 に上がってすぐ戻る。旋回の始まりで NDT の補正が大きく出ている。蛇行（#101）との関係は未確認
- 絶対位置の誤差（真値との比較）はこの記録からは出せない。同じ目印に戻って比べる走行はしていない

## 走行の指示

| goal | 結果 | 止まった理由（ログ） |
| --- | --- | --- |
| →(7.95,-0.16) | 成功 | — |
| →(23.06,0.31) | 成功 | — |
| →(102.07,5.87) | 止まる→差し替え→取消 | (64.5,5.1) 付近。D:forward_blocked で `/cmd_vel_safety` のゼロ指令 1760 件、controller の Failed to make progress ×5 |
| (28.89,0.46)→(-36.99,-0.84) | 成功（bag なし） | — |
| →(-91.32,29.49) | 止まる→取消 | (-84.0,-0.4) 付近。RPP の collision ahead ×5（goal の直後 16:10:36〜38 に 3 回）、D:forward_blocked、Failed to make progress ×2 |

- D:forward_blocked は前方 1.0〜2.0 m・±30° に LiDAR の点があると止める層。何に反応したかは bag の `/velodyne_points_no_ground` と `/scan` で確かめる（未確認）
- (-91.32,29.49) は正門（seimon、(-87.76,-7.40)）から北へ約 37 m。広げた roadway の内か外か、経路が通れる幅かは未確認

## 再解析

```bash
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
python3 ~/bags/2026-11-27-m8-drive/analysis/analyze_bags.py ~/bags/2026-11-27-m8-drive /tmp/reanalysis
```

`analyze_bags.py` は `~/whill_lab0_ros2/.claude/worktrees/agent-a60c1dd6bad14ec8f/scripts/check_localization.py` を import する（main にマージした後はパスを直す）。

## 止まった 2 か所の切り分け（2026-11-27 解析、`analysis/stop_probe.py`）

failsafe の Layer D と同じ条件（`/scan` の 1.0〜2.0 m・±30°、1 点で遮断）で前方の点を拾い、`/pcl_pose` で map に置いて地図のセルと照らした。

| | drive03（140〜229 s） | drive04（155〜215 s） |
| --- | --- | --- |
| 止まった位置・向き | (64.5, 5.1)、東向き | (−84.0, −0.4)、西向き（北へ右折する手前） |
| Layer D で遮断した scan | 758 / 878 | 376 / 589 |
| 反応した点の map の位置 | (66.25, 6.0) にほぼ全部（1942 / 1950） | (−85.5, 0.75) にほぼ全部（380 / 381） |
| base_link での位置 | 前 1.4〜1.8 m、左 0.88〜0.98 m（中央値 0.89） | 前 1.55〜1.73 m、右 0.96〜0.98 m（中央値 0.97） |
| 点の高さ（no_ground） | 中央値 0.55 m（p90 0.85） | 中央値 0.66 m |
| 地図のセル | 最寄りの OCC まで 0.05 m（OCC の縁と、その外の UNK） | 同じく 0.05 m |
| 経路からその点まで | 0.91 m（経路は y 5.1 で直進） | 0.70 m（経路は右折の内側を回る） |
| 経路から OCC まで（6 m 以内） | 最小 0.70 m | 最小 0.67 m |
| 手塗りの地図（candidate_hand）との違い | 経路のセル・反応した点のセルとも同じ | 同じ |
| 矩形の Layer D（#163、x 1〜2 m・\|y\| ≤ 0.5 m）なら | 2 / 877 scan だけ遮断 | 1 / 588 scan だけ遮断 |

読み取れること:

- **止めたのは地図にある静的な構造物**（高さ 0.55〜0.66 m）。人や置き物ではない。反応した点は地図の OCC の縁から 0.05 m 以内で、走っている間も止まっている間も同じ位置に出続ける。
- **自己位置は合っている**。LiDAR の点が地図の OCC にぴったり重なるので、ずれていればこうはならない。fitness も正常（上の表）。
- **154w で広げたことは原因ではない**。経路と点のセルは手塗りの地図でも同じ。
- **原因は Layer D の扇形と Nav2 の経路の幅の食い違い**。Nav2 は OCC から 0.67〜0.70 m 離して経路を引く（inflation 0.6 m、robot_radius 0.45 m）。一方、扇形 ±30° は 1.6 m 先で横 ±0.92 m まで届く。経路の横 0.9〜1.0 m の構造物が扇形に入り、構造物は動かないので遮断が解けない。Nav2 は進めず「Failed to make progress」を繰り返す。
- 矩形の Layer D（#163 の案）なら、どちらの場所でも 1〜2 scan しか遮断しない。0.5 s のヒステリシスで解けるので、通り抜けられる見込み（オフラインでの見積もり、実機では未確認）。
- drive04 の goal の直後（16:10:36〜38、bag の前）と t=33・38 s の RPP「collision ahead」は local costmap による判定で、25〜45 s には Layer D の扇形に点が無い。走りは続いていたので（odom の平均 0.22 m/s）、停止の原因ではない。何に反応したかは未確認。

## 車体の角と地図の構造物の距離（コーナーでの接触の懸念への検算）

車体を長方形（後端 −0.25 m・前端 +0.75 m・幅 0.554 m、base_link 基準。前端は台帳の実測 0.72〜0.75 m、幅は公式値で後付け物は含まない）とし、各 `/pcl_pose` で地図（candidate）の OCC までの最短距離を求めた。

| bag | 車体の長方形 → OCC の最小 | どこで | Nav2 の円（半径 0.45 m）→ OCC の最小 |
| --- | --- | --- | --- |
| drive01 | 1.00 m | — | 0.87 m |
| drive02 | 1.32 m | — | 1.20 m |
| drive03 | **0.20 m** | t=57.0 s、(37.0, 4.4)、**左前の角** | 0.11 m（t=98.5 s） |
| drive04 | **0.30 m** | t=154.1 s、(−81.9, −0.4)、**右前の角**（止まった右折の手前） | 0.20 m（t=157.0 s） |

- 経路は OCC から 0.67 m 以上離して引かれているが、実際の走りは経路より構造物に寄った（RPP の追従のずれ・左右の振れ）。drive03 の t=57 s は、角速度の指令が左右に振れ、姿勢が 0.1 s で 0.16 m 飛んだ時刻と重なる。
- Nav2 の外形は後輪車軸を中心とする半径 0.45 m の円で、前の約 0.3 m がはみ出す（台帳 §1「要変更」#157）。経路・RPP の衝突判定・costmap は前の角を知らない。
