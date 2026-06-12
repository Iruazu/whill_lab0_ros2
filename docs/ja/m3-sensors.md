# M3 — ROS 2 humble でのセンサスタック

Language: [日本語](m3-sensors.md) | [English](../en/m3-sensors.md)

## ゴール

WHILL に載っている知覚センサ (Velodyne LiDAR、Intel RealSense カメラ、9 軸 IMU) を ROS 2 humble 上で立ち上げ、M4 (自己位置推定) と M5 (ナビゲーション) がセンサごとの remap なしで消費できる正しい TF ツリーを持たせる。

M3 の終了条件: 各センサを単独で起動できる、それぞれが期待される ROS 2 トピックを publish する、そして `base_link → <sensor>_link` の static TF チェーンが整っている状態。融合・costmap・自己位置推定はまだ行わない (それらは M4/M5)。

## スコープ (M3 内)

- 各 Group A センサパッケージを公式 ROS 2 上流で置換し、`whill_lab.repos` に固定する。
- `tf_imus` (Group B) を移植する — M3 で必要になる唯一の Group B パッケージで、IMU フレームを正しく publish させるため。
- 新しい `whill_sensors_bringup` パッケージの下にセンサごとの launch ファイルを書き、それらをまとめて起動するトップレベル `sensors_launch.py` を用意する。
- ベンチでも椅子上でも各トピックを検証し、エビデンスとして単発の rosbag を取る。

## スコープ外 (後続マイルストーンに譲る)

- センサ融合 / EKF — M4
- Camera-LiDAR 外部パラメータキャリブ — マウントが変わった時だけ走らせるユーティリティ
- `velodyne_camera_calibration` — 単発ツールとして残し、ランタイム依存にはしない

## ハードウェア

椅子に載っているのは noetic スタックと同じ物理センサ — 2026-05-07 にユーザーが確認済 — なのでモデルとトポロジは確定:

| センサ | モデル | インタフェース | 備考 |
|-------|-------|---------------|------|
| LiDAR | Velodyne **VLP-16** | UDP 2368、10 Hz (RPM 600)、unicast | `frame_id: velodyne`、noetic 側で IP 上書きなし |
| デプスカメラ | Intel RealSense **D435** | USB 3 (VID:PID `8086:0b07`) | 2026-05-07 に本体側面ラベルと `dmesg` の製品名で確定。引き継ぎ時の "D455" は誤り — 実機は D435。`realsense-ros` ドライバは両方サポートするので上流の変更は不要。noetic 側にはプロジェクト固有 launch なし — 上流デフォルトで動かしていた |
| IMU | RT 9 軸 (`rt_usb_9axisimu_driver`) | USB CDC-ACM (VID:PID `2b72:0003`) | `/dev/ttyACM*` として enumerate し、`/dev/ttyUSB*` にはならない — WHILL とは**ナンバリングを共有しない**。安定パス `/dev/imu` はリポ追跡 udev ルールで提供 (後述) |

noetic 値の出典:
[`whill_lab0/FAST_LIO/config/velodyne.yaml`](https://github.com/Iruazu/whill_lab0/blob/main/FAST_LIO/config/velodyne.yaml)
と [`whill_lab0/velodyne-mast/velodyne_pointcloud/launch/VLP16_points.launch`](https://github.com/Iruazu/whill_lab0/blob/main/velodyne-mast/velodyne_pointcloud/launch/VLP16_points.launch)。

noetic スタックから引き継ぐトピック規約 (M4/M5 が前提とする):

- `/velodyne_points` — VLP-16 点群
- `/imu/data_raw` — RT 9 軸の生 IMU (FAST-LIO 入力)
- RealSense トピック: 上流デフォルト (`/camera/...`)

noetic スタックから引き継ぐ LiDAR↔IMU 外部パラメータは [m3-extrinsics-from-noetic.md](m3-extrinsics-from-noetic.md) にまとめて M4 で再利用する。

## 上流パッケージ (Group A)

[`whill_lab.repos`](../../whill_lab.repos) に固定:

| パッケージ | URL | 固定 ref | 備考 |
|-----------|-----|---------|------|
| `velodyne` | `ros-drivers/velodyne` | tag `2.5.1` | `ros2` 系列の最新 2.x リリース |
| `realsense-ros` | `IntelRealSense/realsense-ros` | tag `4.55.1` | 成熟した humble 基準。`librealsense2` システムパッケージ必要 |
| `rt_usb_9axisimu_driver` | `rt-net/rt_usb_9axisimu_driver` | branch `humble-devel` | ベンダ公式の humble ブランチ |

### システム依存

初回 `colcon build` の前に必要な apt パッケージ。どれも標準 ROS 2 / Ubuntu アーカイブから引けるので、Intel 公開の apt リポは **不要**:

```bash
sudo apt install -y \
  ros-humble-xacro \
  ros-humble-diagnostic-updater \
  ros-humble-librealsense2 \
  ros-humble-launch-pytest \
  python3-tqdm \
  libpcap0.8-dev
```

`ros-humble-librealsense2` (現行 2.57.7) は Intel の `librealsense2` を ROS 側でパッケージ化したもの。D435 ドライバがビルド時にヘッダ・ライブラリを見つけるには十分。将来 ROS パッケージが提供するより新しい librealsense が必要になれば、その時点で Intel の apt リポに切り替える。それまでは標準リポで足りる。

## 独自パッケージ (本マイルストーン内の Group B)

| パッケージ | 出典 | 対応 |
|-----------|-----|------|
| `tf_imus` | noetic `whill_lab0/tf_imus` | ament 化、`imu_link` 静的トランスフォームの公開 |

その他の Group B (`sensor` 等) は、本マイルストーンの launch ファイルからランタイム依存が露見しない限り M4/M5 に持ち越す。

## 手順 (計画)

1. 上流 3 つの URL を `whill_lab.repos` に追加し、`./scripts/import_upstream.sh` を走らせる。
2. `colcon build --packages-up-to whill_sensors_bringup`。
3. ベンチ上の単発 smoke テスト (LiDAR は壁向き、RealSense はテクスチャ面、IMU は机に静置):
   - `ros2 topic hz /velodyne_points`
   - `ros2 topic hz /camera/depth/image_rect_raw`
   - `ros2 topic hz /imu/data`
4. TF チェック: 全部起動した状態で `ros2 run tf2_tools view_frames`。`base_link` が 3 つのセンサフレーム全ての親になっていることを確認。
5. (3) と (4) を実機椅子上、USB / 電源も期待通りに繋いだ状態で繰り返す。
6. センサごとの単発 rosbag を `docs/m3-bench-data/` に保存 (大きければ gitignore)。

## ステータス

| ステップ | ステータス |
|---------|-----------|
| ハードウェア構成確定 (noetic スタックと同一) | 完了 |
| `velodyne` を `whill_lab.repos` に固定 | 完了 |
| `realsense-ros` を `whill_lab.repos` に固定 | 完了 |
| `rt_usb_9axisimu_driver` を `whill_lab.repos` に固定 | 完了 |
| ホストにシステム apt 依存をインストール | 完了 (2026-05-07) |
| センサパッケージの `vcs import` + `colcon build` | 完了 — 15/15 パッケージクリーン (2026-05-07) |
| リポ追跡 udev ルールによる安定デバイスパス (`/dev/whill`、`/dev/imu`) | 完了 (2026-05-07) |
| Velodyne 用ホスト側静的 IP (`192.168.1.100/24`) を netplan テンプレ経由で設定 | 完了 (2026-05-07) — 設定済、リンク検証はハードウェア待ち |
| `tf_imus` の ament 移植 | 廃止 — `whill_sensors_bringup/launch/static_tf_launch.py` に置換 (プレースホルダ identity TF、キャリブ値は TODO) |
| `whill_sensors_bringup` パッケージ作成 | 完了 (2026-05-07) — `ros2 launch whill_sensors_bringup sensors_launch.py` で 3 センサと `base_link` 起点の static TF が全部立ち上がる。IMU lifecycle ノードの自動 `configure → activate` も含む。[`src/whill_sensors_bringup/README.md`](../../src/whill_sensors_bringup/README.md) を参照 |
| ベンチで各センサのトピック検証 | 完了 (2026-05-07) — [`m3-bench-data/README.md`](../m3-bench-data/README.md) を参照 |
| TF ツリー検証 | 部分的 (2026-05-07) — RealSense サブツリーは [`m3-bench-data/frames-2026-05-07.pdf`](../m3-bench-data/frames-2026-05-07.pdf) に。`velodyne` と `imu_link` は static parent がまだ必要 (`whill_sensors_bringup` で対応) |
| 椅子上でセンサごとの rosbag 取得 | 完了 (2026-05-07) — `m3_chair_static_2026-05-07/` (1.1 GiB、19.85 秒、静止、0/99 モーションバースト) と `m3_chair_motion_2026-05-07/` (5.3 GiB、96.85 秒、5 秒静止ウォームアップ + 廊下と椅子/机部屋の走行・ループクロージャ試行、先頭 5 秒も 0/25 モーションバーストで検証済)。両方 gitignore。検証手法と椅子 bringup 中に判明した IMU レースコンディション修正は [`m3-bench-data/README.md`](../m3-bench-data/README.md) を参照 |

## Velodyne ネットワーク設定

VLP-16 は USB ではなくイーサネット経由で繋ぐ。LiDAR 工場出荷 IP は `192.168.1.201` なので、ホスト側 USB-Ethernet アダプタを同じ `/24` サブネットに置く必要がある。リポ追跡の netplan テンプレ
[`network/01-velodyne-static.yaml.template`](../../network/01-velodyne-static.yaml.template)
が選んだインタフェースに `192.168.1.100/24` を割り当て、その NIC では DHCP / gateway / DNS を無効化する (一般のインターネットは Wi-Fi 側で維持)。

```bash
ip -br link show | grep -E '^(enx|eth|enp)'         # USB-Ethernet iface を探す
./scripts/install_velodyne_network.sh enx00e04c6808dc   # 置き換えて適用
```

スクリプトはテンプレから `${IFACE}` を展開し、`/etc/netplan/01-velodyne-static.yaml` にモード 600・root:root で配置し (最近の netplan は緩いパーミッションを拒否する)、`netplan apply` を実行する。`ping -c 3 192.168.1.201` で確認。

VLP-16 が別サブネットに書き換えられている場合 (LiDAR 自身の IP に Web UI でアクセスして変更可能)、インストーラを走らせる前にテンプレの addresses ブロックを編集する。

## 安定デバイスパス (2026-05-07 で解決)

IMU は CDC-ACM デバイス (`/dev/ttyACM*`) として enumerate し、USB-serial デバイス (`/dev/ttyUSB*`) ではないので、カーネルナンバリングが WHILL USB-serial ポートと衝突することはない。それでも `ttyUSB0/ttyACM0` のナンバリングから launch ファイルを切り離すため — そしてクリーン clone 後にスタックを再現可能にするため — udev ルール [`udev/99-whill-stack.rules`](../../udev/99-whill-stack.rules) が VID:PID で両デバイスを固定 symlink にマップする:

| デバイス | VID:PID | カーネル名 | 安定 symlink |
|---------|---------|-----------|-------------|
| WHILL CR2 (PL2303) | `067b:2303` | `/dev/ttyUSB0` | `/dev/whill` |
| RT 9 軸 IMU | `2b72:0003` | `/dev/ttyACM0` | `/dev/imu` |

`./scripts/install_udev_rules.sh` でインストール (冪等)。`whill_sensors_bringup` 配下の launch ファイルは生 tty パスではなく `/dev/whill` と `/dev/imu` を指すこと。

## 未解決事項

過去にリストしていた未解決事項は 2026-05-07 にすべて解消。ホスト上での smoke テスト全キャプチャ (トピックレート、frame_id、TF スナップショット、rosbag2 詳細) は
[`m3-bench-data/README.md`](../m3-bench-data/README.md) にある。

クローズ:

- ~~ホスト USB コントローラ上で WHILL USB-serial ケーブルと RealSense D435 が利用可能な USB 3 帯域に収まるか。~~ — D435 は本ホストの Bus 2 で Genesys Logic USB3.1 ハブ経由で SuperSpeed (5 Gbps) として enumerate する。WHILL は Bus 3 の Full-speed (12 Mbps) ポートしか使わないので帯域競合はない。
- ~~IMU の実 publish トピックを確認し、上流ドライバが別の場所に publish するなら `/imu/data_raw` に remap する。~~ — 上流ドライバはそのまま `/imu/data_raw` に publish するので FAST-LIO の期待する入力と一致。remap 不要。**重要な留意点:** ドライバは `LifecycleNode` で、素の `ros2 run` だと `unconfigured` のままトピックを出さない。Bringup が `configure → activate` を駆動しないと subscriber にデータが届かない。

## smoke テストで分かった新事実

- **RealSense トピックプレフィクスは `/camera/camera/...`** (親 namespace `camera`、ノード名 `camera`、どちらも `rs_launch.py` のデフォルト)。M4/M5 の下流 launch は素の `/camera/` ではなくこのプレフィクスを受け入れるか remap すること。
- **Velodyne ドライバは sensor-data QoS で publish する** (best-effort)。humble の `ros2 topic hz` は自動検出してくれないので、Velodyne の生存確認には `ros2 topic echo --once /velodyne_points` か、コード側で sensor-data QoS の subscriber を書く。
