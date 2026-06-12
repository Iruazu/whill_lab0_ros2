# M2 — ROS 2 humble での WHILL コアドライバ

Language: [日本語](m2-whill-core.md) | [English](../en/m2-whill-core.md)

## ゴール

公式ドライバを使い、実機 USB で接続した WHILL Model CR2 を ROS 2 humble 上で立ち上げる。終了条件: WHILL 本体ジョイスティックで通常通り車椅子が動き、かつ ROS 2 側で `/whill/states/model_cr2` にステートテレメトリが見える状態。

実機ハードウェアを叩く最小のマイルストーン。独自ロジックを持たず、上流ドライバを実機 CR2 に繋ぐだけ。

## ハードウェア

| | |
|--|--|
| 椅子 | WHILL Model CR2 |
| 接続 | 椅子の通信ポートから RS232C を引き、自作の赤/白/黒 3 線ハーネス経由で Prolific PL2303 USB-シリアルケーブルに繋ぐ → ホストの `/dev/ttyUSB0` |
| Baud / フレーミング | 38400 / 8N2 (CSTOPB) — WHILL 仕様で固定 |
| 操作入力 | WHILL 本体ジョイスティック (M2 では外部 teleop なし) |

CR2 の公式通信インタフェースは
[WHILL Control System Protocol Specification](https://github.com/WHILL/whill_control_system_protocol_specification) §8.1.2 によれば D-sub 9pin の RS232C (Pin2 TXD, Pin3 RXD, Pin5 GND)。本リポのラボハーネスは D-sub を経由せず、JST 側配線で同じ線を引き出している。機能的に等価で、2026-05-06 時点で動作確認済み。

## 上流パッケージ

本マイルストーンは [whill-labs](https://github.com/whill-labs) の公式 2 パッケージを使う:

- [`ros2_whill`](https://github.com/whill-labs/ros2_whill) — ドライバ、bringup、description、サンプル一式 (`whill`、`whill_bringup`、`whill_description`、`whill_driver`、`whill_examples` の 5 パッケージ)。
- [`ros2_whill_interfaces`](https://github.com/whill-labs/ros2_whill_interfaces) — メッセージ・サービス定義 (`whill_msgs` パッケージ)。`ModelCr2State`、`SpeedProfile`、`SetPower` / `SetSpeedProfile` / `SetBatterySaving` サービスを定義する。

`ros2_whill` は個人 fork [`Iruazu/ros2_whill`](https://github.com/Iruazu/ros2_whill) の `humble` ブランチに固定する。この fork は上流 `whill-labs/ros2_whill` を追跡しつつ、後述するコールドブートの癖を直す [PR #1: Send SetPower(ON) and re-enable body joystick during Initialize()](https://github.com/Iruazu/ros2_whill/pull/1) を当ててある。`ros2_whill_interfaces` は上流 `humble` ブランチそのまま。`humble` ブランチは CR シリーズ共通の機能のみを露出する系統で、CR 固有の追加機能を持つ `crystal-devel` は本リポでは使わない。

## 手順

### 1. 上流を import し依存解決する

リポルートで:

```bash
./scripts/import_upstream.sh
```

`vcs import src < whill_lab.repos` の後に、import 済みツリーに対して `rosdep install` を走らせる。clone 先は `src/third_party/` で、本リポの git 管理からは除外している (`.gitignore`)。再実行すると既存 clone を fast-forward する。

### 2. シリアルポートのアクセス権を付ける (初回のみ)

ドライバは `/dev/ttyUSB0` を開く。所有者は `root:dialout`。現ユーザーを `dialout` に追加する:

```bash
./scripts/grant_serial_access.sh
# 一度ログアウトしてログインし直すか:
newgrp dialout
```

### 3. ビルド

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to whill --symlink-install
source install/setup.bash
```

ラボ PC でクリーンビルド検証済 (6 パッケージ / 13.7 秒)。

### 4. WHILL を繋いでデバイスノードを確認する

1. WHILL の電源を入れる。
2. CR2 マニュアル指定 (USB-A → USB-mini など) のケーブルで椅子の USB ポートをホストに繋ぐ。
3. デバイスが見えるか確認する:
   ```bash
   ls /dev/ttyUSB*
   # → /dev/ttyUSB0
   dmesg | tail | grep ttyUSB
   ```
   `/dev/ttyUSB0` 以外で見える場合は `src/third_party/ros2_whill/whill_bringup/config/params.yaml` を編集する (またはステップ 5 で `port_name:=/dev/ttyUSBN` を渡す)。

### 5. ドライバを起動する

```bash
ros2 launch whill_bringup whill_launch.py
```

デバイスパスがデフォルトでない場合:

```bash
ros2 run whill_driver whill --ros-args -p port_name:=/dev/ttyUSB1
```

別ターミナルでテレメトリを確認:

```bash
ros2 topic list | grep whill
ros2 topic echo /whill/states/model_cr2
```

WHILL を本体ジョイスティックで動かす。`ModelCr2State` の値 (バッテリ、モータ状態、ジョイスティック傾き等) が変化するはず。

## コールドブートの癖とパッチ

電源を入れた直後の Model CR2 は、`StartSendingData` を受け取っただけでは Dataset1 フレームの送出を**開始しない**。仕様書 §5 によれば、椅子の通信サブシステムは `SetPower(ON)` を受け取ったあとに起き上がる。上流 `whill-labs/ros2_whill` の `Initialize()` は `StartSendingData` しか送っていないため、コールドブート時はドライバが `/dev/ttyUSB0` を開いて健康そうに見えるが何も publish しない。`ros2 topic echo /whill/states/model_cr2` も `ros2 topic hz` も無反応のままになる。

fork の [PR #1](https://github.com/Iruazu/ros2_whill/pull/1) は `WhillNode::Initialize()` の `StartSendingData` 直前に 2 つのコマンドを追加する:

1. `SetPowerOn()` を 10 ms / 2 ms 間隔で 2 回。これは `OnSetPowerSrv` が既に使っているリトライパターンと同形で、仕様の「5 ms 以内に応答がなければ SetPower を再送」を満たす。
2. `SendSetJoystickCommandWithLocal()`。`SetPower(ON)` は椅子をホスト制御モードに切り替え、本体ジョイスティックを暗黙のうちに無効化する。このコマンドで制御を本体側に戻し、ROS 側コントローラが何も publish していなくても本体ジョイスティックが効くようにする。

ROS 側制御が後で `/whill/controller/cmd_vel` や `/whill/controller/joy` に publish すると、既存の `SendSetVelocityCommand` / `SendSetJoystickCommand` 経路がオンデマンドで椅子をホスト制御に戻す。リグレッションはない。

### LCD 表示の解釈

CR2 本体 LCD は通常運用中 `BATTERY_POWER` (0–100 %) を表示する。`93` のような 2 桁数字は**バッテリ残量**であってエラーコードではない。電源 ON 直後に LED が一瞬赤くなるのも通常のブートシーケンスの一部でエラーではない。

### 「ドライバは動くがトピックが出ない」のデバッグ

`ros2 launch whill_bringup whill_launch.py` がエラーなくポートを開いたのに `ros2 topic echo /whill/states/model_cr2` が無音の場合、優先度順に:

1. **パッチ済 fork が実際に使われているか確認**。`git -C src/third_party/ros2_whill log --oneline -3` に `Send SetPower(ON) and re-enable body joystick during Initialize()` が含まれていればよい。含まれていなければ `./scripts/import_upstream.sh` を再実行してリビルドする。
2. **椅子を電源 OFF→ON** してから再起動する。既知状態からの `SetPower → Dataset` ハンドシェイクを確認できる。
3. **ケーブルが上がっているか確認**。`ls /dev/serial/by-id/` に PL2303 エントリが見えるか。`ls -l /dev/ttyUSB0` が `crw-rw---- root dialout` か。
4. **起動シェルで dialout が効いているか確認**。`groups | grep dialout`。なければ `newgrp dialout` (またはログインし直し) のあとに起動する。

## 日々の運用手順 (WHILL ルーム到着後)

ラボ PC はネットワーク間を移動する。WHILL ルームのネットワークに繋いだあと、ドライバを起動するシェルで:

```bash
# 1. ローカル USB / ROS 経路に干渉しないようキャンパス HTTP プロキシを切る
unset HTTP_PROXY HTTPS_PROXY FTP_PROXY http_proxy https_proxy ftp_proxy

# 2. このシェルで dialout が効いていることを確認
groups | grep dialout || newgrp dialout

# 3. WHILL の電源を入れ (LCD にバッテリ % が出る、例えば "93")、USB を繋ぐ
ls /dev/ttyUSB0   # 出ているはず

# 4. オーバーレイを source して起動
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
ros2 launch whill_bringup whill_launch.py
```

別ターミナルでも同じ `unset` + `source` を済ませた上で:

```bash
ros2 topic hz /whill/states/model_cr2     # publish_interval_ms=400 で ~2.5 Hz を期待
ros2 topic echo /whill/states/model_cr2 --once
```

## トピック

`humble` ブランチの上流ドライバ:

| 向き | トピック | 型 |
|-----|---------|---|
| pub | `/whill/states/model_cr2` | `whill_msgs/ModelCr2State` |
| sub | `/whill/controller/joy` | `sensor_msgs/Joy` |
| sub | `/whill/controller/cmd_vel` | `geometry_msgs/Twist` |

標準的な `/odom` や `/battery_state` は `humble` ブランチの上流ドライバでは**提供されない**。下流パッケージが要求するなら後続マイルストーンでブリッジノードを追加する余地はある。

## ステータス

| ステップ | ステータス |
|---------|-----------|
| 上流の `vcs import` | 完了 |
| `colcon build --packages-up-to whill` | 完了 (6 パッケージ、クリーン) |
| シリアルアクセス用の `dialout` 所属 | 完了 |
| WHILL 接続時に `/dev/ttyUSB0` が見える | 完了 (Prolific PL2303、`usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0`) |
| `whill_launch.py` が椅子に届き state を publish する | 完了 — コールドブートパッチ適用後、2026-05-06 に動作確認 |
| ジョイスティックで椅子を動かしつつ ROS 2 がテレメトリを受ける | 完了 — `/tmp/m2_test.log` の 10 秒キャプチャで `right/left_motor_speed` が ±1.5、`battery_current` が 0–122 で振れることを確認 |

M2 の受け入れ基準はすべて満たした。コールドブートパッチは fork PR #1 にある。調査の物語形式の記録は [Session log: 2026-05-06](session-2026-05-06.md) を参照。次は M3 (センサ)。詳細は [`m3-sensors.md`](m3-sensors.md)。
