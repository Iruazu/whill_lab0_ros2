# 実機配車デモの起動・停止手順

対象は `campus-m8` 地図（候補 `candidate`）、実機 WHILL、Nav2、配車 Web。画面は HTTPS 8000、rosbridge は WSS 9090（`whill_console` の gateway 8765 は使わない）。

既定の地図は 2026-11-13 から `campus-m8`（#190。#186 の LiDAR の TF・Hi-Res 校正を前提に作り直した地図。経緯と再生の結果は [campus-m8 の README](maps/campus-m8/README.md)）。屋外の受入（#156）はまだで、この手順で行う。旧 `campus`（地図 v2）は fallback として残しており、「旧地図（campus）に戻す」の手順で戻せる。

## 統合起動（通常）

準備（1〜3 節）が済んでいれば 1 ターミナルで:

```bash
cd ~/whill_lab0_ros2
scripts/start_dispatch_demo.sh
```

既定は campus-m8 / 地図 candidate / speed normal / domain 0。RMW・DDS・domain はスクリプトが設定する。

| オプション | 用途 |
| --- | --- |
| `--dry-run` | 実行内容だけ表示 |
| `--no-rviz` | RViz を起動しない（初期位置は別途指定） |
| `--speed slow` | 低速 |
| `--site campus` | 旧地図（fallback）。Web 背景の作り直しが要る（「旧地図（campus）に戻す」） |
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
scripts/check_device_aliases.sh
ls -l install/setup.bash configs/cyclonedds-runtime-min.xml
ls -l docs/maps/campus-m8/static.pcd docs/maps/campus-m8/occupancy_candidate.yaml docs/maps/campus-m8/v2/candidate_final.pgm
ls -l docs/maps/campus-m8/waypoints.yaml
echo "d244865866e3eacdea6e26851dd3ed61ba626029fdc6dd4f2dabfb6d720d03b5  docs/maps/campus-m8/static.pcd" | sha256sum -c
ls -l ~/.whill_dispatch_tls/dispatch.crt ~/.whill_dispatch_tls/dispatch.key
ip -brief link show
ip -brief address show
ss -ltnp | grep -E ':(8000|9090)\b'
pgrep -af 'ros2 launch|dispatch_node|mock_navigate_to_pose|m7_https_server|http.server'
```

- `check_device_aliases.sh` は `/dev/whill` `/dev/imu` の有無、リンク先の VID:PID（`udev/99-whill-stack.rules` と照合）、読み書きを確かめる。番号は固定でない。別名だけ無いときは、記録を残すコマンドと戻すコマンド（`sudo udevadm trigger --action=change --name-match=/dev/ttyACM0` など）を表示する（#141）
- `static.pcd` は Git 管理外。campus-m8 の正本は `~/bags/m5r-bench-data/2026-07-10-campus-outer-final/dufomap-m8-hires-noise-local/static.pcd`。無ければ次で複製し、上の `sha256sum -c` が `OK` になることを確かめる（`FAILED` なら使わない）。作り直すと同じ地図にならないので、作り直さない（[campus-m8 の README](maps/campus-m8/README.md) 2 節）。`start_dispatch_demo.sh` は起動前に、static.pcd が無ければ下の複製コマンドを表示して止まり、`metadata.yaml` に `static_pcd_sha256` があれば照合して、違えば止まる

  ```bash
  cd ~/whill_lab0_ros2
  cp ~/bags/m5r-bench-data/2026-07-10-campus-outer-final/dufomap-m8-hires-noise-local/static.pcd docs/maps/campus-m8/static.pcd
  echo "d244865866e3eacdea6e26851dd3ed61ba626029fdc6dd4f2dabfb6d720d03b5  docs/maps/campus-m8/static.pcd" | sha256sum -c
  ```
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
ros2 launch whill_safety m6r_bringup_launch.py site:=campus-m8
```

センサ、WHILL driver、EKF、localizer、failsafe、twist_mux が起動する。sensors / odom / safety の launch を追加で起動しない。RealSense は既定で起動しない。

起動直後の `DEAD INPUT` は入力未受信の通知。後で入力が来たことを確かめる。

## 5. T2: Nav2 と障害物前処理

```bash
ros2 launch whill_navigation nav_launch.py site:=campus-m8 map_variant:=candidate speed:=normal
```

campus-m8 は手塗りが確定していない（#154）ので、variant なしの `occupancy.yaml` を置いていない。`map_variant` を省くと、nav_launch は site ごとの既定（campus-m8 は `candidate`）を使い、読んだ yaml を起動ログの `nav_launch: site=... map_yaml=...` に出す。旧 campus で `map_variant:=v2` を省くと旧 `occupancy.yaml` を読む。speed は slow 0.30 / normal 0.45 / fast 0.60 m/s。`Managed nodes are active` は lifecycle の確認であり、入力や自己位置の確認ではない。

## 6. T3: 配車 Web

```bash
ros2 launch whill_dispatch dispatch_launch.py site:=campus-m8 use_mock:=false use_tls:=true
```

配車地点の名前（`kounoken` / `bldg7` / `seimon` / `bldg3`）と表示名は旧 campus と同じ。座標は旧地図の点を写した候補で、現地での照合は #156 で行う。

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

check 0〜5。Layer D の検査では指示に従って人が前方約 1.5 m に立つ。0 以外は失敗。check 0 は DDS の XML と LiDAR の NIC の一致を、check 0b は `/dev/whill` `/dev/imu` の別名を、check 4 は failsafe の入力の受信を確かめる。初期位置を与える前は `/pcl_pose` が届かないので、check 4 は失敗する。

`/cmd_vel_safety` が出ていないのは、入力未受信で監視が始まっていないだけのこともある。

## 9. 配車と手動救出

Web で目的地を選ぶか地図をタップして実行する。計画失敗と正常到着を区別して状態を見る。

並走者がジョイスティックに介入できる体制で行う。手動救出はボタンを押している間だけ指令を送る（途絶 0.4 秒でゼロ、無操作 20 秒で解除）。teleop が途絶えると Nav2 の指令に戻り得るので、救出の前に進行中の配車を取り消す。

## 10. 停止と撤収

Web で配車を取り消し、車体の停止を確かめる。T3 → T2 → T1 の順に Ctrl-C。RViz と録画も止める（dispatch を止めても Nav2 の goal は取り消されない）。

T3 の Ctrl-C で `rosapi_node` が `ExternalShutdownException` のトレースバックと `process has died [... exit code 1 ...]` を出す。上流 rosapi の終了処理の不具合で、プロセスは終わっており再起動も妨げない（#169）。これ以外のノードのトレースバックや、終了後に残るプロセスは異常として扱う。

```bash
ss -ltnp | grep -E ':(8000|9090)\b'
```

AP を使ったら、元の接続名で `nmcli connection up <接続名>` に戻す。

## 旧地図（campus）に戻す

campus-m8 で自己位置が合わない、経路が引けないなど、その場で直せないときの fallback。旧 campus は 2026-07 の受入（M6R-2）とデモに使った地図で、#186 の前の TF で作った。#186 の TF と Hi-Res の点群で旧地図を使うと、NDT の当てはまりは campus-m8 より悪い（07-24 の bag の再生で fitness の中央値が約 6 倍。[campus-m8 の README](maps/campus-m8/README.md) 4 節）。戻すのは一時的な手段として扱い、戻した理由を記録する。

配車地点の名前は同じだが、座標は地図ごとに違う（各 site の `waypoints.yaml`）。Web の背景は 1 組しか置けないので、旧地図で作り直してビルドする（作り直さないと `start_dispatch_demo.sh` の地図の同期検査で止まる）。

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
ls -l docs/maps/campus/static.pcd docs/maps/campus/occupancy_v2.yaml docs/maps/campus/v2/final.pgm docs/maps/campus/waypoints.yaml
python3 scripts/m7_make_web_map.py --yaml docs/maps/campus/occupancy_v2.yaml --out-dir src/whill_dispatch/web
colcon build --symlink-install --packages-select whill_dispatch
python3 scripts/check_map_sync.py --site campus --web-dir install/whill_dispatch/share/whill_dispatch/web
scripts/start_dispatch_demo.sh --site campus
```

個別に起動するときは、3 節の共通環境の後に 3 つとも `site:=campus` にし、Nav2 は `map_variant:=v2`:

```bash
ros2 launch whill_safety m6r_bringup_launch.py site:=campus
ros2 launch whill_navigation nav_launch.py site:=campus map_variant:=v2 speed:=normal
ros2 launch whill_dispatch dispatch_launch.py site:=campus use_mock:=false use_tls:=true
```

campus-m8 に戻すときは、Web の背景を作り直してビルドする（`git checkout -- src/whill_dispatch/web/map.png src/whill_dispatch/web/map_meta.json` で Git の版に戻してもよい）:

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash
python3 scripts/m7_make_web_map.py --yaml docs/maps/campus-m8/occupancy_candidate.yaml --out-dir src/whill_dispatch/web
colcon build --symlink-install --packages-select whill_dispatch
python3 scripts/check_map_sync.py --web-dir install/whill_dispatch/share/whill_dispatch/web
```

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
| 地図が違う | 3 つの launch の `site`、Nav2 の `map_variant`（起動ログの `map_yaml=`）、Web 背景の出典（`map_meta.json` の `source_pgm`） |
| Web が開けない | 8000 の待ち受け、LAN、証明書 |
| Web だけ未接続 | WSS 9090、証明書の信頼、rosbridge ログ |
| goal が失敗する | 自己位置、global 地図、local 障害物、Nav2 の結果とログ |
| Web 背景が古い | 下記で再生成し、dispatch を再起動してブラウザを再読込 |

```bash
cd ~/whill_lab0_ros2
python3 scripts/m7_make_web_map.py \
  --yaml docs/maps/campus-m8/occupancy_candidate.yaml --out-dir src/whill_dispatch/web
```

Nav2 の地図とは別の成果物。`start_dispatch_demo.sh` は起動前に、install 側の背景と Nav2 の地図を `scripts/check_map_sync.py` で照合し、食い違えば起動しない。手で確かめるとき:

```bash
cd ~/whill_lab0_ros2
python3 scripts/check_map_sync.py --web-dir install/whill_dispatch/share/whill_dispatch/web
```
