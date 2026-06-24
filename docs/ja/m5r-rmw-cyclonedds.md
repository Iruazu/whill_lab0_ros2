# M5-R: `/velodyne_points` 1 Hz 病の原因切り分けと RMW 選定

Language: [日本語](m5r-rmw-cyclonedds.md) | [English](../en/m5r-rmw-cyclonedds.md)

2026-06-24 の録画セッション中に発生した「`/velodyne_points` が公称
10 Hz から 1 Hz 前後に落ちる」事象の診断レポート。実測値の根拠と恒久対策
(RMW=CycloneDDS) を確定する 1 次史料。CLAUDE.md の「ランタイム環境の前提」
節と `docs/ja/m5r-pipeline.md` §録画 はここを参照する。

## 背景

`docs/m5r-bench-data/2026-06-24-loop-outdoor-ext/` の追加録画 take2 を試みた
際、`/velodyne_points` の rate が以下のように崩壊した:

```
$ timeout 8 ros2 topic hz /velodyne_points
WARNING: topic [/velodyne_points] does not appear to be published yet
average rate: 0.941
    min: 0.915s max: 1.210s std dev: 0.14732s
```

この状態のままでは GLIM への入力レートが 1/10 になり、後段の SLAM/M6-R
ベンチが破綻する。同日の最初の bag (`2026-06-24-loop-outdoor-ext/bag`、
13:49 録画) は 2361 messages / 239.45 s = 9.86 Hz で健全だったため、
ハードウェアではなく**実行時環境**の問題と当たりをつけて切り分けた。

## 切り分け手順 (消去法)

すべて launch を Terminal A で前面実行、別 Terminal で観測する分離構成
で取得。同じバイナリ・同じ launch・同じ機材で、状態だけが違う比較。

| # | 確認項目 | 結果 | 判定 |
|---|---------|------|------|
| 1 | `/velodyne_packets` (driver_node 出力) の rate | 9.857 Hz, std dev 0.00025s | driver / LiDAR / network すべて健全 |
| 2 | `transform_node` プロセス状態 (`top`) | CPU 6.2%, status S (sleeping) | 内部無限ループは否定 |
| 3 | `ros2 topic info -v /velodyne_packets` の Subscription count | 1 (transform_node) | subscribe 不成立は否定 |
| 4 | Publisher / Subscriber QoS | 両者 RELIABLE / VOLATILE | QoS ミスマッチは否定 |
| 5 | CPU governor / 周波数 | 既定 `powersave` (一部 484 MHz) → `performance` (一部 3.8 GHz) に切替 | governor 変更後も `/velodyne_points` 0–7 Hz と不安定。CPU 周波数は主因ではない |
| 6 | システム全体の CPU 使用率 (`top -b`) | 97–98% idle、競合プロセスなし | リソース競合は否定 |

ここまでで「入力健全・購読成立・QoS 一致・CPU 暇」が確定。残る経路は
RMW (DDS) ミドルウェアによるメッセージ配送そのもの。

### 決定的な実験: RMW 切替

`rmw_cyclonedds_cpp` を apt で導入し、launch / 観測の両ターミナルで
`export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` を設定して再測定:

```
$ timeout 12 ros2 topic hz /velodyne_points
average rate: 9.477   min: 0.101s max: 0.304s std dev: 0.02872s window: 49
average rate: 9.277   min: 0.101s max: 0.304s std dev: 0.03548s window: 95
```

スパイク (FastDDS では max 1.4–2.9 s) が完全に消え、std dev は 0.03 秒台に
収束。1 回の切り替えで再現性 100% で症状が解消した。

## 根本原因

**FastDDS (humble 既定 RMW) が `velodyne_msgs/VelodyneScan` のような
大きなメッセージ (1 scan = 76 packets) の配送で間欠的に詰まる。**

切り分け表 #1 が示す通り、`/velodyne_packets` も同じ FastDDS 配送経路を
通っているが、こちらは小さい packet なので影響が出ない。`VelodyneScan`
コンテナを 1 メッセージとして渡す段で SHM (shared memory) 転送の何か
(buffer / mutex) が間欠遅延を生み、`transform_node` の callback が 1 秒
以上呼ばれない期間が発生する。CycloneDDS は同じ条件で問題なく流れる。

経路としては既知問題で、FastDDS の v2.x には大メッセージ SHM の遅延
スパイク報告が複数ある。本リポは ROS 2 humble (FastDDS 2.6 系) のため
該当する。FastDDS 側の `transport descriptors` を弄って UDP loopback に
強制する選択肢もあるが、CycloneDDS への切替が最も低コストで確実。

## 副次的に確定した事項

- **CPU governor は再起動で `powersave` に戻る** (Alienware x15 R2)。
  録画/SLAM ではコア周波数のばらつきが直接ベンチの揺れに化けるため、
  毎セッション `sudo cpupower frequency-set -g performance` 必須。
- **既存 bag `2026-06-24-loop-outdoor-ext/bag` は健全**: 9.86 Hz / 100 Hz
  で記録済み (FastDDS 詰まり発生**前**のセッションで取得)。
  再録不要。`NOTES.md` に経緯を残す。

## 恒久対策

### 必須 (本機 = Alienware x15 R2)

```bash
# RMW: ~/.bashrc に永続化 (1 回だけ)
echo 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' >> ~/.bashrc

# governor: 毎セッション (録画 / SLAM / M6-R 検証の前に)
sudo cpupower frequency-set -g performance
```

### セッション開始時の sanity check

新しいターミナルで bringup する前に必ず:

```bash
echo $RMW_IMPLEMENTATION                                # rmw_cyclonedds_cpp
cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor   # performance
ros2 pkg list | grep rmw_cyclonedds_cpp                 # インストール確認
```

3 つすべて期待値であることを確認してから launch する。録画前は録画
ターミナルでも同じ 3 行を確認する (`~/.bashrc` 反映済みでも別シェルで
は環境が違う場合がある)。

### 録画後の検証 (mandatory)

```bash
ros2 bag info <bag-dir> | grep -E "velodyne_points|imu/data_rep145"
```

期待 (200 s 走行を例に):
- `/velodyne_points` count: ~2000 (200 s × 10 Hz)
- `/imu/data_rep145` count: ~20000 (200 s × 100 Hz)

`/velodyne_points` が公称の半分以下ならその bag は破棄して再録。
本症状 (1 Hz 病) は録画中に静かに発生するため、走行終了後にここで
気付く構造を作っておく。

## 影響範囲

- M5-R 録画ワークフロー (`docs/ja/m5r-pipeline.md` §録画)
- M6-R bringup (`whill_localization/launch/odom_bringup_launch.py` 起動時)
- M6-R フェイルセーフ検証 (`/velodyne_points` レートを安全閾値として
  使う設計が出てくる場合、その閾値は CycloneDDS 前提で設定する)
- GLIM オフライン処理 (bag 再生時、再生側のシェルでも RMW を揃える)

## 影響を**受けない**もの

- FAST-LIO のビルド時メッセージ (`-- Using RMW implementation
  'rmw_fastrtps_cpp' as default`) — これはビルド時の typesupport 既定
  解決であって、ランタイム RMW とは別系統。実行時に
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` が立っていれば Cyclone で動く。

## 未決の検討事項 (今は手を入れない)

- ADR 化: 比較・選定議論ではなく実測由来の制約なので、現状は本書 +
  CLAUDE.md の前提節で十分。将来 M7 以降でデプロイ計画を立てるときに
  ADR-0006 として正式化する選択肢を残す。
- launch / script への `export` 埋め込み: セッション側で環境を握る
  原則を崩したくないため見送り。`sanity check` 1 行を pipeline doc に
  足すことで運用カバーする。

## 参考

- 既存の関連診断: `docs/ja/m5r-imu-diagnostic.md` (Issue #64、GLIM IMU 警告)
- M5-R パイプライン本文: `docs/ja/m5r-pipeline.md`
- CLAUDE.md「ランタイム環境の前提」節
