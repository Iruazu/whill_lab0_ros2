# 実機配車デモの起動・停止手順

2026-10-14 改訂。ローカルの現用コードを根拠に整理した手順。7〜8月の実測記録は履歴として参照するが、統合起動と実機配車の成功は2026-10-14にユーザー報告とログで確認した。確認範囲と地点名の不一致は [検証記録](reports/2026-10-14-dispatch-demo.md) を参照。

対象は `whill_lab0_ros2` の campus 地図 v2、実機 WHILL、Nav2、dispatch Web UI。`whill_console` の gateway 8765 は使用しない。この構成の画面は HTTPS 8000、rosbridge は WSS 9090。

## 統合起動（通常はこちら）

初回のビルド・ネットワーク・証明書準備を済ませたら、1ターミナルで以下を実行する。

```bash
cd ~/whill_lab0_ros2
scripts/start_dispatch_demo.sh
```

bringup・Nav2・TLS配車Web・RVizを起動し、共通のRMW・DDS・domainを設定する。既定はcampus / 地図v2 / speed normal / domain 0。地図や接続先を変更する場合は準備手順も照合する。

- 実行内容だけを見る: `scripts/start_dispatch_demo.sh --dry-run`
- RVizを別に起動する: `scripts/start_dispatch_demo.sh --no-rviz`
- 低速で起動する: `scripts/start_dispatch_demo.sh --speed slow`
- 別domain: `scripts/start_dispatch_demo.sh --domain 21`（診断側も同じ値にする）

スクリプトは機器・地図・証明書・DDSのNIC名・ポート・既存ノードを確認する。RVizを先に起動し、localizerとmap_serverのactive後に初期位置設定を促す。RVizで設定して起動ターミナルのEnterを押すと、Nav2のactiveを待って配車Webを起動する。起動成功は入力受信・自己位置収束・Web接続の保証ではない。ログは `log/dispatch-demo-<日時>-<識別子>/` の各 `.log` に保存する。

RVizのFixed Frameをmapに設定し、2D Pose Estimateで初期位置を指定して、起動ターミナルのEnterを押す。`--no-rviz` の場合も別途同じ環境で初期位置を指定する。Nav2がactiveになるにはmap TFが必要なため、初期位置設定前にcontrollerのactiveを待たない。配車Web起動後、別ターミナルから同じ環境でpreflightを実行できる。

```bash
cd ~/whill_lab0_ros2
scripts/start_dispatch_demo.sh --preflight
```

受信・収束の確認とpreflightの制約は以下のT5を参照。preflight失敗でも統合起動中のスタックは自動終了しない。結果を確認して配車せずに原因を調べる。Web画面に検査結果を連動するゲートは未実装。

統合起動ターミナルのCtrl-Cで、このshが起動したプロセスを停止する。dispatch停止後にNav2へgoal取消を要求し、Nav2・bringupを終了する。返信やプロセス終了だけで車体停止と判断せず現物も確認する。スクリプトはAP切替・sudo設定・証明書生成・初期位置送信・走行goal投入を自動実行しない。

以下は準備と個別起動・診断の詳細。統合起動中にT1〜T4のlaunchを重ねて実行しない。

## 1. 開始前の確認

既存の ROS スタックが動いている場合は、その起動ターミナルで停止してから切り替える。`git pull`、再ビルド、証明書生成、ネットワーク変更は準備作業として済ませ、デモ起動のたびに実施しない。未コミット変更がある状態で自動的に checkout / pull しない。

```bash
cd ~/whill_lab0_ros2
git status --short
ls -l /dev/whill /dev/imu
ls -l install/setup.bash configs/cyclonedds-runtime-min.xml
ls -l docs/maps/campus/static.pcd docs/maps/campus/occupancy_v2.yaml docs/maps/campus/v2/final.pgm
ls -l docs/maps/campus/waypoints.yaml
ls -l ~/.whill_dispatch_tls/dispatch.crt ~/.whill_dispatch_tls/dispatch.key
ip -brief link show
ip -brief address show
ss -ltnp | grep -E ':(8000|9090)\b'
```

`/dev/whill` と `/dev/imu` のリンク先番号は固定でなくてよい。リンク先が存在し、ユーザーが読み書きできることを確認する。`static.pcd` は Git 管理対象外で、別途配置が必要。

現用の `cyclonedds-runtime-min.xml` は lo と `enx00e04c680ec9` を列挙する。実際の LiDAR NIC と一致するか確認する。NIC は optional のため、名前が違っても DDS が起動することがある。起動成功だけで有線接続の確認を済ませない。旧 `cyclonedds-runtime.xml` と bag-record 用 XML は、この runbook では使用しない。

残存プロセスを調べる場合:

```bash
pgrep -af 'ros2 launch|dispatch_node|mock_navigate_to_pose|m7_https_server|http.server'
```

PID の所有者と起動元を確認し、起動元の Ctrl-C で終了する。無関係なプロセスを一括終了しない。

### ホスト設定

過去のデモでは CPU governor を performance、lo を multicast 有効にしている。現在の状態を確認し、必要なら設定する。システム設定を変えるため sudo が必要。

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
ip link show lo
sudo cpupower frequency-set -g performance
sudo ip link set lo multicast on
```

## 2. タブレット接続と証明書

同じ LAN で接続する。過去のデモ用 AP は NetworkManager の `Hotspot`、PC アドレス `10.42.0.1`。現在の接続設定で確認する。AP を有効にすると、使用中の Wi-Fi 接続が切り替わる場合がある。

```bash
nmcli connection show
# デモ用 AP を使用する場合だけ実行
nmcli connection up Hotspot
ip -4 address show wlo1
```

証明書は `~/.whill_dispatch_tls/dispatch.crt` / `dispatch.key`。初回または接続先 IP が変わった場合は、生成スクリプトの内容を確認して作成する。

```bash
cd ~/whill_lab0_ros2
scripts/m7_make_tls_cert.sh 10.42.0.1
```

既存証明書を再生成するとタブレット側の信頼設定も更新が必要になる。実際に接続する IP を指定する。

## 3. 各ターミナルの共通環境

起動・診断・録画の各ターミナルで次を実行する。他 workspace の環境を読み込んだシェルを使い回さず、新しいターミナルで始める。

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${PWD}/configs/cyclonedds-runtime-min.xml"
export ROS_DOMAIN_ID=0
```

この手順では domain 0 に統一する。別の domain を使う場合は全ターミナルで同じ値にする。`whill_platform` 側の domain 設定を混在させない。

## 4. T1: 車体・センサ・自己位置推定・安全層

共通環境を読み込んだターミナル1で:

```bash
ros2 launch whill_safety m6r_bringup_launch.py site:=campus
```

センサ、WHILL driver、EKF、localizer、failsafe、twist_mux が起動する。このターミナルは開いたままにする。sensors / odom / safety launch を追加で起動しない。RealSense は既定で起動しない。

初期位置や Nav2 未起動により `DEAD INPUT` が出ることがある。このメッセージは入力未受信を示す。後で入力が復旧したことを確認する必要があり、起動時の正常メッセージとして無条件に無視しない。

## 5. T2: Nav2 と障害物前処理

共通環境を読み込んだターミナル2で:

```bash
ros2 launch whill_navigation nav_launch.py site:=campus map_variant:=v2 speed:=normal
```

`map_variant:=v2` を明示する。無指定だと旧 `occupancy.yaml` を読む。`speed` の既定は slow=0.30、normal=0.45、fast=0.60 m/s。ここでは従来デモの normal を指定する。

`Managed nodes are active` を確認する。これは lifecycle 起動の確認であり、入力・自己位置・安全層の正常性は次の検査で確認する。

## 6. T3: 配車・Web

共通環境を読み込んだターミナル3で:

```bash
ros2 launch whill_dispatch dispatch_launch.py site:=campus use_mock:=false use_tls:=true
```

HTTPS 8000、rosbridge WSS 9090、dispatch が起動する。`use_mock:=false` は実 Nav2 を使用する。PC自身は `https://localhost:8000/`、タブレットは `https://<PCのIP>:8000/` を開く。

この Web UI に gateway トークン入力はない。TLS は通信を暗号化するが、認証は実装されていない。

タブレットに証明書を初めて導入する場合:

1. デモ用 LAN に接続する。
2. `https://<PCのIP>:8000/dispatch.crt` を開く。
3. 設定のプロファイル管理から証明書をインストールする。
4. 証明書信頼設定で対象の証明書を信頼する。
5. `https://<PCのIP>:8000/` を開き、WebSocket が接続済みになることを確認する。

OS の画面名は端末の版によって異なる。証明書が更新されている場合は古いプロファイルとの不一致を確認する。HTTPS画面が開けても、WSS 9090への接続成功は別途確認する。

## 7. T4: RViz と初期位置

共通環境を読み込んだターミナル4で:

```bash
rviz2
```

Fixed Frame を `map` にする。`/map`、`/velodyne_points`、`/scan`、`/plan` を追加し、2D Pose Estimate で実際の車体位置と向きを指定する。常に原点を送るのではなく、地図と実車を合わせる。

## 8. T5: 入力確認と preflight

共通環境を読み込んだターミナル5で、まずノード重複と入力を確認する。

```bash
ros2 node list | sort | uniq -c
ros2 lifecycle get /lidar_localization
ros2 lifecycle get /controller_server
for topic in /whill/odom /imu/data_rep145 /velodyne_points /velodyne_points_no_ground /scan /pcl_pose; do
  printf '\n確認: %s\n' "$topic"
  timeout 5 ros2 topic echo "$topic" --once >/dev/null
  printf '終了コード: %s\n' "$?"
done
timeout 5 ros2 topic echo /alignment_status --once
```

重複ノードを解消し、各入力が到着し、初期位置が収束していることを確認する。`timeout` の終了コード124は時間内に受信できなかったことを示す。トピック一覧に名前があるだけでは受信確認にならない。

続いて検査スクリプトを実行する。check 0〜5 の6項目で、Layer D の検査では指示に従い人が前方約1.5mに立つ。

```bash
timeout 180 scripts/m6r_preflight.sh
result=$?
printf 'preflight 終了コード: %s\n' "$result"
```

0以外は未完了または失敗。スクリプトの check 4 は `/rosout` の最初の一件だけを見るため、過去の `DEAD INPUT` や現在の入力欠落を漏れなく検出できるとは限らない。終了コード0だけで全安全機能の確認済みとせず、上記の受信確認と起動ログの確認を併用する。外側の timeout は無期限待ちを防ぐためであり、検査ロジックを修正するものではない。

`/cmd_vel_safety` が無発報でも、正常とは断定できない。入力未受信で監視が開始されていない場合もある。前方に物がない状態で停止指令が継続する場合は、ログで作動中の層を特定する。

## 9. 配車と手動救出

検査を終えたら Web から目的地を選択し、実行する。地図タップも実装されている。Nav2 の計画失敗と正常到着を区別して状態を確認する。

並走者が車体のジョイスティックに介入できる体制で行う。手動救出はボタンを押している間だけ指令を送る。指令途絶で0.4秒のwatchdog、20秒の無操作で手動モード解除が働く。teleop入力が途絶えるとmuxはNav2入力に戻り得るため、手動救出前に進行中の配車を取消する。

Layer D は前方±30°・1〜2mの有効点を監視する。後方や全周の検知、人の分類、物理E-stopとの統合を保証する機能ではない。手動中の safety 優先制御について、過去文書に未検証の注記が残るため、完了を主張するには実機の証跡が必要。

## 10. 停止と撤収

Web で進行中の配車を取消し、車体停止を確認する。その後 T3 dispatch → T2 Nav2 → T1 bringup の順に Ctrl-C。RViz と録画も終了する。dispatch終了だけではNav2のgoal取消を保証しない。

```bash
ss -ltnp | grep -E ':(8000|9090)\b'
```

APを使用した場合は、開始前の接続名を確認して `nmcli connection up <接続名>` で戻す。`uu-w` など環境固有の名前を固定の終了手順にしない。

## 録画

共通環境を読み込んだ別ターミナルで、録画先を明示して実行する。

```bash
mkdir -p docs/m7-bench-data
ros2 bag record -o "docs/m7-bench-data/$(date +%Y-%m-%d_%H-%M-%S)-live-campus" \
  /velodyne_points /scan /odometry/filtered /whill/odom /imu/data_rep145 \
  /pcl_pose /alignment_status /cmd_vel /cmd_vel_nav /cmd_vel_safety /cmd_vel_teleop \
  /whill/controller/cmd_vel /dispatch/state /plan \
  /local_costmap/costmap /global_costmap/costmap \
  /local_costmap/published_footprint /tf /tf_static
```

Ctrl-Cで録画停止後、`ros2 bag info <実際の録画先>` で確認する。点群10Hz・IMU100Hzなどは目安。点群数が録画時間に比べ著しく少ない場合は、解析前に受信と録画の状態を確認する。大容量bagをそのままGitに追加しない。

## トラブル時の確認順

| 症状 | 確認すること |
| --- | --- |
| 車体 odom が来ない | `/dev/whill` のリンク先、権限、電源、driverログ。ttyUSB番号を固定しない |
| 入力が来ない / DEAD INPUT | 全端末の RMW・domain・DDS XML、NIC名、lo multicast、publisherとsubscriberのQoS |
| 地図が違う | Nav2の `map_variant` とWeb背景画像の出典 |
| Webが開けない | 8000の待ち受け、LAN、証明書、HTTPS応答 |
| Webだけ未接続 | WSS 9090、証明書信頼、rosbridgeログ |
| goalが失敗する | 自己位置、global地図、local障害物、Nav2結果とログ。DDSだけを原因に決めつけない |
| 古いWeb背景 | 下記の再生成とdispatch再起動。ブラウザの再読込 |

Web背景を変更する場合:

```bash
cd ~/whill_lab0_ros2
python3 scripts/m7_make_web_map.py \
  --yaml docs/maps/campus/occupancy_v2.yaml --out-dir src/whill_dispatch/web
```

Nav2地図とは別の成果物を更新する。既存の `install` に反映されていることも確認する。生のPGMを別指定してYAMLと食い違わせない。
