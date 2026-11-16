# whill_safety

実機統合bringup、failsafe、twist_muxを提供する。運用手順は [runbook](../../docs/runbook-live-demo.md)。

## 起動単位

`m6r_bringup_launch.py` は odom bringup（センサ・WHILL・EKF）、NDT localizer、`safety_launch.py`（failsafe・twist_mux）を含む。内包するlaunchを別途起動しない。Nav2 / dispatch は別ターミナルで併用する。

既定の `site:=campus-m8` は `docs/maps/campus-m8/static.pcd` を読む（2026-11-13、#190）。旧地図の fallback は `site:=campus`（`docs/maps/campus/static.pcd`）。`WHILL_MAPS_ROOT` で親ディレクトリを変更できる。PCDはGit対象外で、campus-m8 は正本を複製して sha256 を確かめる（`docs/maps/campus-m8/README.md` 2 節）。RealSenseは既定false。

## Localizer

`config/m6r_lidar_localization.yaml` を一時YAMLに展開し地図パスを設定する。NDT_OMP、`use_odom: true`、`predict_pose_from_previous_delta: false`。odomを `/odometry/filtered` にremapする。直接IMU入力は使用しない。初期位置は `/initialpose`。

localizerの受理閾値 `score_threshold: 6.0` と、failsafeのfitness閾値1.0は別の値。

## 監視層

| 層 | 入力 | 作動条件 |
| --- | --- | --- |
| A | `/reinitialization_requested` | true受信後1秒保持 |
| B | `/alignment_status`, `/pcl_pose` | fitness>1.0または非収束が2秒継続、pose受信後の途絶>1秒 |
| C | `/velodyne_points_no_ground` | 初回受信後の途絶>2秒 |
| D | `/scan` | 前方±30°、1〜2mの有限点が1点以上。最後の検知から0.5秒保持。初回受信後の途絶≥0.5秒（`D:scan_silent`、#193） |

Dは点の検知で、人の分類ではない。`DEAD INPUT` watchdogは開始10秒後に未受信入力をエラー通知するが、ログを出すだけで起動禁止を保証するものではない。pose / perception / scan の途絶監視は初回受信で開始する。

### 初回受信後の `/scan` 途絶（#193、既定は有効）

#193 より前は、`/scan` が一度届いた後に止まると（LiDARの抜け、velodyne_driver や pointcloud_to_laserscan の停止）、Layer D は最後の判定のまま残り、車体は止まらなかった。パラメータ `scan_silent_sec`（launch引数 `failsafe_scan_silent_sec`、`m6r_bringup_launch.py` と `safety_launch.py` にある、既定 `0.5`）を超えて `/scan` が届かないと `D:scan_silent` が作動し、他の層と同じくゼロを20Hzで出す。受信が戻ってから0.5秒（Layer D の解除と同じ値）途絶が再発しなければ解く。0.6秒おきのような断続的な受信では解けない。`0.0` で無効（#193 以前の動作）。負の値やNaNは起動時に拒否する（`startup_grace_sec` と同じ）。

0.5秒の理由：

- `/scan` は10Hz。2026-07-22〜31 の走行 bag 11本（`/scan` 21,206通）の記録時刻の間隔は中央値0.101秒、最大0.213秒。例外は `/velodyne_points` も同時に約40秒途切れた2か所で、LiDAR側か記録側の途絶なので止まるのが正しい。0.5秒は4 scan 連続の欠落で、計測した最大の2倍以上の余裕がある
- Layer D の保持0.5秒と同じ値にすると、最後の scan で前方が塞がっていた場合でも、`D:forward_blocked` が解ける時刻に `D:scan_silent` が作動し、停止が途切れない（判定は `>=`）
- 0.3m/sで0.5秒と1 tick は約0.17m。扇形の手前端1.0mより短い

既定を有効にした理由（failsafe の変更なので、PR のマージがユーザーの承認になる）：

- 起動の手順では作動しえない。最初の `/scan` までは何もしない（`m6r_bringup` が Nav2 より先に起動しても止まらない）。最初の `/scan` より前は #159 の `D:scan_never_arrived`（`startup_grace_sec > 0` のときだけ）の担当で、両方が同時に出ることはない。`use_sim_time` で `/clock` より前に届いた scan（時刻0）は途絶の基準にしない
- 新しい停止の場面はほぼ増えない。`/scan` は `/velodyne_points_no_ground` から作るので、Nav2（nav_launch）の停止・再起動や LiDAR の停止では、既存の `C:perception_silent`（2秒）がもともと止める。増えるのは pointcloud_to_laserscan だけが止まった場合と、途絶から2秒までの間
- 単体テスト（`test/test_failsafe_scan_silent.py`）と、基準 bag の再生（`scripts/regression/replay_check.py --only failsafe`）で確かめた。結果は ADR-0007 の 2026-11-16 追記

実機の通常起動で誤停止しないことは未確認（#156 の機会に確かめる）。止まる場合は `failsafe_scan_silent_sec:=0.0` で従来の動作に戻せる。

救援（#161）との関係：救援の中継は `D:forward_blocked` が単独のときだけなので、`D:scan_silent` の作動中は中継せずゼロを出す。`/scan` が無いと、後退や回転の先を Layer D が見ていないため。

他の入力の途絶監視（#193 で確かめた）：

- C（`/velodyne_points_no_ground`）：初回受信後の途絶>2秒 で `C:perception_silent`。仕組みは既にある。解除は受信の再開で即時（ヒステリシス無し）のまま変えていない
- B（`/pcl_pose`）：初回受信後の途絶>1秒 で `B:pcl_pose_silent`。`/alignment_status` には途絶監視が無いが、`/pcl_pose` と同じ localizer ノード（`lidar_localization_component.cpp`）が出すので、ノードの停止は `/pcl_pose` 側で止まる。`/alignment_status` だけが止まると最後の fitness の判定が残る（悪い判定なら止まり続け、良い判定なら止まらない）。この片方だけの停止は起きにくいとして追加しなかった
- 深度（D435）：main の failsafe は深度を購読していない（RealSense は既定 false）。C3 の `extra_perception_topics` を取り込むときは、同じ途絶監視を加える

### 最初から届かない入力（#159、既定は無効）

パラメータ `startup_grace_sec`（launch引数 `failsafe_startup_grace_sec`、`m6r_bringup_launch.py` と `safety_launch.py` にある）で扱いを選ぶ。failsafeの変更なので、既定を変えるにはユーザーの承認が要る。

| 値 | 動作 |
| --- | --- |
| `0.0`（既定） | 従来どおり。10秒後の `DEAD INPUT` ログだけで、止めない |
| `> 0` | 開始からこの秒数までに1件も届かないB / C / D入力（`/pcl_pose`、`/velodyne_points_no_ground`、`/scan`）ごとに、`B:pcl_pose_never_arrived`・`C:perception_never_arrived`・`D:scan_never_arrived` として停止する。他の層と同じく `/cmd_vel_safety` にゼロを20Hzで出す。初回受信でその入力の停止は解け、以後は途絶監視に移る（B / C は既存、D は #193 の `D:scan_silent`） |

負の値やNaNは起動時に拒否する。このときfailsafe_nodeは起動直後に終了するが、twist_muxは動き続けるので、safetyの無い状態（fail-open）になる。preflightのcheck 2（`/failsafe_node` の存在）で検出される。failsafeが落ちたらlaunch全体を止める `on_exit=Shutdown` は、挙動の変更なので承認待ちの選択肢として残す（未実装）。

`use_sim_time` では最初の `/clock` から数える。既存の `DEAD INPUT` 判定は `_start_time=0` のまま数えるので、bag再生では最初のtickで出る（既存の動作。変えていない）。

選択肢と影響：

- **警告のみ（現状）**：起動順に影響しない。D435が起動しない場合などに、未監視のまま走れてしまう。
- **猶予後に停止、初回受信で解除（実装済み）**：入力がそろうまで走れない。m6r_bringupは単独でも起動するので、Nav2（`/scan`、`/velodyne_points_no_ground` の出所）の起動前と初期位置の指定前（`/pcl_pose`）は停止が出続ける。この間は手動救出（teleop）もtwist_muxで止まる。そろえば自動で解ける。preflightの手かざし試験（check 5）は、この停止でも `/cmd_vel_safety` が出るので通ってしまう。入力の受信を見るcheck 4が先に通っていることが前提になる。
- **停止してラッチ（未実装）**：一度作動したら再起動まで解けない。起動の遅いノードで毎回再起動が要るので採らなかった。

推奨は `10.0`（`DEAD INPUT` の通知と同じ）での有効化。停止と解除が起動手順のとおりに起こることを実機で確かめてから既定を変える。C3（`feat/avoidance-c3`）の `extra_perception_topics` を取り込むときは、同じ判定に加える。

作動中は `/cmd_vel_safety` にゼロ指令を20Hzで出す。無発報は正常の証明ではなく、未受信や未起動も切り分ける。

## 調停と制約

| 入力 | 優先度 | タイムアウト |
| --- | --- | --- |
| `/cmd_vel_safety` | 100 | 0.5秒 |
| `/cmd_vel_teleop` | 50 | 0.5秒 |
| `/cmd_vel_nav` | 10 | 0.5秒 |

設定は `config/twist_mux.yaml`。最高優先度の有効入力が勝ち、出力は `/cmd_vel`。後段にNav2のvelocity_smootherがあるため、ゼロ指令も平滑化される。物理E-stopとの統合や全周監視をこのパッケージが完成済みとは扱わない。

### 停止中の救援（#161、ADR-0013 提案、既定は無効）

既定では、どの層の作動中もteleopは通らない（safetyが優先）。パラメータ `layer_d_rescue_enabled`（launch引数 `failsafe_layer_d_rescue`）をtrueにすると、作動しているのがLayer Dだけのとき、0.3秒以内に届いた前進を含まない `/cmd_vel_teleop`（後退0.2m/s以内、回転0.6rad/s以内、他の成分0）を、failsafeが `/cmd_vel_safety` にそのまま中継する。条件を外れたら（前進・範囲外・途絶、A / B / C や未着入力の作動）その時点でゼロを出す。safety枠は作動中ずっと生きているので、Nav2と前進のteleopは `/cmd_vel` に出られない。中継中にLayer Dが解けたら、teleopが途絶えるまで中継を続け、ゼロを1回出してから止める。有効時だけ `/cmd_vel_teleop` を購読する。

| パラメータ | 既定 | 内容 |
| --- | --- | --- |
| `layer_d_rescue_enabled` | false | 救援の有効化 |
| `rescue_cmd_timeout_s` | 0.3 | teleop指令の鮮度。`rescue_cmd_timeout_s + 1/20Hz + 0.1秒` がtwist_muxのteleopタイムアウト0.5秒未満でないと起動しない（有効時のみ確かめる） |
| `rescue_max_reverse_mps` | 0.2 | 救援と見なす後退速度の上限 |
| `rescue_max_wz` | 0.6 | 救援と見なす回転速度の上限 |

後方とその場回転の掃引範囲（1m未満）は監視しない。UIの離しのゼロは中継されるので通常はすぐ止まる。ゼロが届かないと、最後の指令から約0.35秒（名目値。executorの遅れで伸びる）中継が続く。鮮度はsteady clockで測る。設計の比較と残る危険はADR-0013。実機では未検証。中継中の状態はログ（`RESCUE relay` / `RESCUE ended`）だけに出る。C3の `/failsafe/state` を取り込むときにそこへ載せる。

preflightは `scripts/m6r_preflight.sh`。check 4は各入力の受信を、failsafeと同じ型とQoSで確かめる。
