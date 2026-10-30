# 実機配車デモの起動・停止手順

対象は campus 地図 v2、実機 WHILL、Nav2、配車 Web。画面は HTTPS 8000、rosbridge は WSS 9090（`whill_console` の gateway 8765 は使わない）。

## 統合起動（通常）

準備（1〜3 節）が済んでいれば 1 ターミナルで:

```bash
cd ~/whill_lab0_ros2
scripts/start_dispatch_demo.sh
```

既定は campus / 地図 v2 / speed normal / domain 0。RMW・DDS・domain はスクリプトが設定する。

| オプション | 用途 |
| --- | --- |
| `--dry-run` | 実行内容だけ表示 |
| `--no-rviz` | RViz を起動しない（初期位置は別途指定） |
| `--speed slow` | 低速 |
| `--domain 21` | 別 domain（診断側も同じ値に） |
| `--preflight` | 起動中のスタックに preflight を実行（別ターミナルで） |

流れ:

1. 機器・地図・証明書・NIC・ポート・既存ノードを確認し、bringup・Nav2・RViz を起動する
2. localizer と map_server が active になったら、RViz（Fixed Frame `map`）の 2D Pose Estimate で初期位置を指定し、起動ターミナルで Enter
3. Nav2 の active を待って配車 Web を起動する
4. 別ターミナルで `scripts/start_dispatch_demo.sh --preflight`（8 節の確認も併用）

- ログは `log/dispatch-demo-<日時>-<識別子>/`
- 起動成功は入力受信・自己位置収束・Web 接続の保証ではない。preflight が失敗してもスタックは止まらないので、配車せずに原因を調べる
- Ctrl-C で dispatch → Nav2 goal 取消 → Nav2 → bringup の順に止まる。車体の停止は現物で確かめる
- AP 切替・sudo 設定・証明書生成・初期位置・走行 goal は自動実行しない
- 統合起動中に 4〜7 節の launch を重ねて起動しない

## 1. 開始前の確認

前の ROS スタックは起動したターミナルで止めておく。pull・ビルド・証明書・ネットワーク変更は準備として済ませ、デモのたびにはしない。

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
pgrep -af 'ros2 launch|dispatch_node|mock_navigate_to_pose|m7_https_server|http.server'
```

- `/dev/whill` `/dev/imu` はリンク先が存在し読み書きできればよい（番号は固定でない）
- `static.pcd` は Git 管理外。別途配置する
- `cyclonedds-runtime-min.xml` は lo と `enx00e04c680ec9` を列挙する。実際の LiDAR NIC 名と一致するか確かめる（NIC は optional なので、違っていても起動してしまう）
- 残っているプロセスは起動元で Ctrl-C する。一括 kill しない

ホスト設定（sudo）:

```bash
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor
ip link show lo
sudo cpupower frequency-set -g performance
sudo ip link set lo multicast on
```

## 2. タブレット接続と証明書

PC とタブレットを同じ LAN につなぐ。デモ用 AP は NetworkManager の `Hotspot`（PC は `10.42.0.1`）。有効にすると使用中の Wi-Fi が切り替わることがある。

```bash
nmcli connection show
nmcli connection up Hotspot          # デモ用 AP を使うときだけ
ip -4 address show wlo1
```

証明書は `~/.whill_dispatch_tls/dispatch.crt` / `dispatch.key`。初回と接続先 IP が変わったときに作る（作り直すとタブレット側も入れ直し）:

```bash
cd ~/whill_lab0_ros2
scripts/m7_make_tls_cert.sh 10.42.0.1
```

## 3. 各ターミナルの共通環境

個別に起動・診断・録画するターミナルは、新しいターミナルで毎回これを実行する。

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI="file://${PWD}/configs/cyclonedds-runtime-min.xml"
export ROS_DOMAIN_ID=0
```

全ターミナルで同じ domain にする。`whill_platform` の `env.sh` を読んだシェルと混ぜない。

## 4. T1: 車体・センサ・自己位置・安全層

```bash
ros2 launch whill_safety m6r_bringup_launch.py site:=campus
```

センサ、WHILL driver、EKF、localizer、failsafe、twist_mux が起動する。sensors / odom / safety の launch を追加で起動しない。RealSense は既定で起動しない。

起動直後の `DEAD INPUT` は入力未受信の通知。後で入力が来たことを確かめる。

## 5. T2: Nav2 と障害物前処理

```bash
ros2 launch whill_navigation nav_launch.py site:=campus map_variant:=v2 speed:=normal
```

`map_variant:=v2` を省くと旧 `occupancy.yaml` を読む。speed は slow 0.30 / normal 0.45 / fast 0.60 m/s。`Managed nodes are active` は lifecycle の確認であり、入力や自己位置の確認ではない。

## 6. T3: 配車 Web

```bash
ros2 launch whill_dispatch dispatch_launch.py site:=campus use_mock:=false use_tls:=true
```

PC は `https://localhost:8000/`、タブレットは `https://<PC の IP>:8000/`。TLS で暗号化するが認証はない。

タブレットへの証明書導入（初回）:

1. `https://<PC の IP>:8000/dispatch.crt` を開く
2. 設定のプロファイル管理からインストール
3. 証明書信頼設定で信頼する
4. `https://<PC の IP>:8000/` を開き、WebSocket が接続済みになることを確かめる（HTTPS が開けても WSS 9090 は別に確かめる）

## 7. T4: RViz と初期位置

```bash
rviz2
```

Fixed Frame を `map` にし、`/map` `/velodyne_points` `/scan` `/plan` を追加。2D Pose Estimate で実際の車体の位置と向きを指定する。

## 8. T5: 入力確認と preflight

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

重複ノードが無く、全入力が届き（終了コード 124 は未受信）、初期位置が収束していることを確かめる。

```bash
timeout 180 scripts/m6r_preflight.sh
result=$?
printf 'preflight 終了コード: %s\n' "$result"
```

check 0〜5。Layer D の検査では指示に従って人が前方約 1.5 m に立つ。0 以外は失敗。check 4 は `/rosout` を 1 件見るだけなので、0 でも上の受信確認と起動ログを併用する。

`/cmd_vel_safety` が出ていないのは、入力未受信で監視が始まっていないだけのこともある。

## 9. 配車と手動救出

Web で目的地を選ぶか地図をタップして実行する。計画失敗と正常到着を区別して状態を見る。

並走者がジョイスティックに介入できる体制で行う。手動救出はボタンを押している間だけ指令を送る（途絶 0.4 秒でゼロ、無操作 20 秒で解除）。teleop が途絶えると Nav2 の指令に戻り得るので、救出の前に進行中の配車を取り消す。

## 10. 停止と撤収

Web で配車を取り消し、車体の停止を確かめる。T3 → T2 → T1 の順に Ctrl-C。RViz と録画も止める（dispatch を止めても Nav2 の goal は取り消されない）。

```bash
ss -ltnp | grep -E ':(8000|9090)\b'
```

AP を使ったら、元の接続名で `nmcli connection up <接続名>` に戻す。

## 録画

共通環境を読んだ別ターミナルで:

```bash
mkdir -p docs/m7-bench-data
ros2 bag record -o "docs/m7-bench-data/$(date +%Y-%m-%d_%H-%M-%S)-live-campus" \
  /velodyne_points /scan /odometry/filtered /whill/odom /imu/data_rep145 \
  /pcl_pose /alignment_status /cmd_vel /cmd_vel_nav /cmd_vel_safety /cmd_vel_teleop \
  /whill/controller/cmd_vel /dispatch/state /plan \
  /local_costmap/costmap /global_costmap/costmap \
  /local_costmap/published_footprint /tf /tf_static
```

止めたら `ros2 bag info <録画先>` で件数を確かめる（点群 10 Hz・IMU 100 Hz が目安）。bag を Git に追加しない。

## トラブル時の確認

| 症状 | 確認すること |
| --- | --- |
| 車体 odom が来ない | `/dev/whill` のリンク先、権限、電源、driver ログ |
| 入力が来ない / DEAD INPUT | 全ターミナルの RMW・domain・DDS XML、NIC 名、lo multicast、QoS |
| 地図が違う | Nav2 の `map_variant` と Web 背景の出典 |
| Web が開けない | 8000 の待ち受け、LAN、証明書 |
| Web だけ未接続 | WSS 9090、証明書の信頼、rosbridge ログ |
| goal が失敗する | 自己位置、global 地図、local 障害物、Nav2 の結果とログ |
| Web 背景が古い | 下記で再生成し、dispatch を再起動してブラウザを再読込 |

```bash
cd ~/whill_lab0_ros2
python3 scripts/m7_make_web_map.py \
  --yaml docs/maps/campus/occupancy_v2.yaml --out-dir src/whill_dispatch/web
```

Nav2 の地図とは別の成果物。`install` に反映されたかも確かめる。
