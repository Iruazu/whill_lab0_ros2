# M4 — ROS 2 humble での自己位置推定

Language: [日本語](m4-localization.md) | [English](../en/m4-localization.md)

## ゴール

椅子上で LiDAR-Inertial Odometry を動かし、リアルタイムの `/Odometry` と、次のマイルストーン (M5 / Nav2) が経路計画に使える `map` (もしくは `camera_init`) フレームを出力する。物理マウントが変わっていないので、noetic スタックから引き継いだ校正済 LiDAR↔IMU 外部パラメータを再利用する。

M4 の終了条件: クリーンな 60 秒の椅子走行から、FAST-LIO が記録経路をおおむねなぞる有界軌道と、下流パッケージが消費・remap できる `camera_init -> body` の TF チェーンを生成する状態。

## スコープ (M4 内)

- `hku-mars/FAST_LIO@ROS2` (ブランチ名は大文字注意) と、ハード依存である `Livox-SDK/livox_ros_driver2` を `whill_lab.repos` に固定し、本ワークスペースでビルドする。
- `fastlio_mapping` を椅子向けパラメータで包む `whill_localization` パッケージを書き、offline replay と live 運用の 2 つの launch エントリポイントを用意する。
- M3 で取った椅子搭載モーション bag に対して offline で、そして 3 つの新規 live 椅子走行で検証する。

## スコープ外 (M5 に譲る)

- ゴールベース航法のための 2D / 3D 地図構築
- Nav2 用の `map -> odom -> base_link` TF 配線
- `cmd_vel` の消費 — WHILL ドライバは既に M2 で運動を担当するが、自己位置推定 → コントローラのループは M5 スコープ

## ビルド前提

M4 で取り込む 2 つの上流は素の `vcs import + colcon build` でクリーンビルドはできない。新規ホストごとに 3 つの追加ステップが必要 (各ホストで 1 回):

1. **Livox SDK 2.x をシステムレベルで初期化する**。`livox_ros_driver2` は Velodyne で走る場合でも `/usr/local/lib/liblivox_lidar_sdk_shared.so` にハードリンクする。SDK は別系統でビルドする:

   ```bash
   git clone --depth 1 https://github.com/Livox-SDK/Livox-SDK2.git /tmp/Livox-SDK2
   cd /tmp/Livox-SDK2 && mkdir build && cd build
   cmake .. && make -j$(nproc) && sudo make install
   sudo ldconfig
   ```

2. **Livox ドライバの ROS バージョン別ファイルを差し替える**。上流リポは `package_ROS1.xml` / `package_ROS2.xml` と `launch_ROS1/` / `launch_ROS2/` を同梱しており、付属の `build.sh` スクリプトが colcon 呼び出し前に ROS2 版を所定位置にコピーする。そのスクリプトを 1 回走らせるか、手動でやる:

   ```bash
   cd src/third_party/livox_ros_driver2
   cp -f package_ROS2.xml package.xml
   [ -d launch ] || cp -r launch_ROS2 launch
   ```

3. **FAST_LIO の `ikd-Tree` git submodule を初期化する**。`vcs import` は submodule に再帰しない:

   ```bash
   git -C src/third_party/FAST_LIO submodule update --init --recursive
   ```

その後、Livox が期待する cmake フラグでビルドする:

```bash
colcon build --symlink-install \
    --cmake-args -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
```

これらのブートストラップ手順は、新規 fork を一発でビルドできるよう、後続コミットで `scripts/import_upstream.sh` に畳み込む。

## ハードウェア → FAST-LIO 入力

| 出力元 | トピック | レート | 備考 |
|-------|---------|-------|------|
| Velodyne VLP-16 | `/velodyne_points` | 10 Hz | per-point time フィールドは秒単位 (スキャン終端基準なので負の値になる)。`timestamp_unit: 0` と一致 |
| RT 9 軸 IMU | `/imu/data_raw` | 100 Hz | 生 — orientation フィールドの covariance[0]=-1 で "not provided" を示しており、これが FAST-LIO の望む形 |
| `whill_sensors_bringup` | `/tf_static` | latched | `base_link → imu_link / velodyne / camera_link` の 4 つの static transform。FAST-LIO は使わないが Nav2 が使う |

LiDAR↔IMU **外部パラメータ**は noetic スタック由来で変更なし: LiDAR は椅子の左側、IMU は座面クッションの下、約 30 cm 下。値とその由来は [`m3-extrinsics-from-noetic.md`](m3-extrinsics-from-noetic.md) を参照。

## 設定上の判断と理由

[`src/whill_localization/config/velodyne_whill.yaml`](../../src/whill_localization/config/velodyne_whill.yaml) は上流デフォルトをいくつか上書きしている:

| フィールド | 上流デフォルト | M4 の値 | 理由 |
|-----------|--------------|--------|------|
| `common.imu_topic` | `/imu/data` | `/imu/data_raw` | RT 9 軸ドライバは生 IMU を `/imu/data_raw` に publish するため |
| `preprocess.scan_line` | 32 | 16 | VLP-16 は 16 リング |
| `preprocess.timestamp_unit` | 2 (μs) | 0 (s) | velodyne ROS2 ドライバの per-point time は秒単位 |
| `preprocess.blind` | 2.0 | 0.5 | noetic から引き継ぎ — 椅子フレーム上で 0.5〜2 m の戻りが出るが、これを落としたくない |
| `mapping.fov_degree` | 360 | 180 | LiDAR の後ろ半球は椅子本体と着座者を見る。両方剛体的に取り付いており、世界地図を汚すため |
| `mapping.gyr_cov` | 0.1 | **0.5** | 上流値はジョイスティック駆動の鋭い旋回には締まりすぎ。0.1 だと最初の ~30 秒で旋回時に発散。0.5 で安定 |
| `mapping.extrinsic_T/R` | identity | noetic 引き継ぎ値 | 校正済み、noetic から不変。IMU は座面クッションの下に ~14 度傾けて装着、LiDAR は椅子左側で ~30 cm 上 |
| `cube_side_length` | 1000.0 | **200.0** | 1000³/0.5³ ボクセルは pcl::VoxelGrid の int32 インデックスをオーバーフローし、"No Effective Points!" でストールする |
| `publish.path_en` | false | true | RViz 軌跡可視化を有効化 |

## Replay プロトコル

オフライン専用テスト (椅子は電源 OFF、センサ非接続):

```bash
ros2 launch whill_localization fast_lio_launch.py rviz:=false
# 別ターミナルで:
ros2 bag play <bag_dir> --clock \
    --topics /velodyne_points /imu/data_raw /imu/mag /tf_static
```

椅子上の live テスト:

```bash
ros2 launch whill_localization localization_launch.py
```

各 run は最初の 5 秒は完全静止で始めること。最初の椅子モーション前に iESKF が IMU バイアスに収束できるようにするため。

## ステータス

| ステップ | ステータス |
|---------|-----------|
| `hku-mars/FAST_LIO` (ROS2 ブランチ) を固定 + ビルド | 完了 |
| `Livox-SDK/livox_ros_driver2` を固定 + ビルド (SDK インストール含む) | 完了 |
| `whill_localization` パッケージ作成 (config + launch 2 本) | 完了 |
| `IncludeLaunchDescription` の config パスバグ修正 | 完了 |
| クリーン bag で offline replay が有界 `/Odometry` を出力 | 完了 — `m4_chair_live_2026-05-08_run2`、経路長 40 m、ループクロージャ誤差 18 % |
| 椅子上の live 運用が E2E | 完了 — `localization_launch.py` で全ノードがクリーンに立ち上がる |
| 再現性の定量化 | 完了 — 2026-05-08 の 3-run 試験 (`docs/m3-bench-data/README.md`) |
| Config ブートストラップ (Livox SDK、submodule、package.xml) を `scripts/import_upstream.sh` に畳み込み | 保留 — M5 wrap-up に畳み込む |
| `map → odom → base_link` TF 配線 | M5 スコープ |
| Nav2 用の 2D / 3D 保存地図 | M5 スコープ |

## 既知の制約 / TODO

- **収録品質が支配的**。静止ウィンドウの汚染や前方 180° 視野内の歩行者は registration を不可逆に壊す。今後の走行は明示的な「ゴー」キューと静かな環境が必要。
- **長時間走行は依然発散する**。96.85 秒の `m3_chair_motion` replay はこの設定でも ~30 秒後に外れていく。収録時の旋回を緩める、環境ごとに LI-Init でより精密な外部パラメータを取る、あるいは M5 テストで必要性が見えるまで M4 成果物は ≤ 60 秒走行で受け入れる、のいずれか。
- **live `/Odometry` は CPU 律速**。`fastlio_mapping` が RViz、3 センサドライバ、`ros2 bag record` と同居する場合、FAST-LIO 公称 10 Hz に対して ~1.5 Hz。軌跡形状は正しく、サンプリングが粗いだけ。M5 のコントローラが live `/Odometry` を subscribe する前に、live ランタイムを削るか FAST-LIO を別ホストに分離する。
- **FAST-LIO は非決定的** — 同じ bag を 2 回 replay すると終端ポーズが ~10 m ずれる。M4 では許容するが、上流に issue を立てる価値はある。
