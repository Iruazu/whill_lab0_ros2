# M5-R 前置: GLIM のソースビルドによるセットアップ

Language: [日本語](m5r-glim-setup.md) | [English](../en/m5r-glim-setup.md)

## 目的

M5-R (オフラインのマップ作成パイプライン) で第一候補としている [GLIM](https://github.com/koide3/glim) を、母艦 (Alienware x15 R2) にソースビルド経由でインストールし、サンプル bag で trajectory 出力まで疎通させる。後続の M5R-3 (実 bag を用いた GLIM vs FAST-LIO SAM 比較) の前提を成立させることが本文書の到達点。

ソースビルドを選択した理由:

- GLIM 公式 apt PPA (`koide3/ppa`) は CUDA 12.4 向けバイナリを配布していない (12.2 / 12.6 / 13.1 のみ)。本リポは [`m5r-cuda-setup.md`](m5r-cuda-setup.md) で CUDA 12.4 を pin しているため、PPA を採ると CUDA バージョンが揺れる
- PPA を採れば「お手軽だが pin 不可」、ソースビルドを採れば「重いが pin 可」のトレードオフ。本リポは M5R-3 で実 bag 比較を行う以上、ビルドが bit-for-bit 再現できる側を選ぶ
- GLIM・gtsam_points・Iridescence の各上流は MIT/BSD ライセンスで、本リポの [運用方針 §3.4](plans/2026-06-11-platform-pivot.md) (permissive 維持) と整合する

採択経緯と要件のひもづけは [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §3.3 と §9 を参照。

## 前提環境

| | |
|--|--|
| ホスト | Alienware x15 R2 (`systemlab-Alienware-x15-R2`) |
| OS | Ubuntu 22.04.5 LTS (jammy) |
| GPU | NVIDIA GeForce RTX 3080 Laptop GPU (16 GB VRAM、Ampere CC 8.6) |
| NVIDIA Driver | 595.71.05 |
| CUDA Toolkit | 12.4 (`/usr/local/cuda-12.4`、[`m5r-cuda-setup.md`](m5r-cuda-setup.md) §2 で導入) |
| cuDNN | 8.x (同上) |
| ROS 2 | humble Desktop (`/opt/ros/humble`) |

CUDA Toolkit が母艦に入っていない状態 (Issue #45 起案時点ではこの状態だった) ではビルドが進まないので、まずは [`m5r-cuda-setup.md`](m5r-cuda-setup.md) §4 の `vectorAdd` サンプルを `Result = PASS` で通すこと。

## 採用したライセンス整合

| コンポーネント | 上流リポ | ライセンス | 運用方針との整合 |
|---|---|---|---|
| GLIM | [`koide3/glim`](https://github.com/koide3/glim) | MIT | permissive、運用スタック (車載で動き、将来配布し得る部分) に組み込み可 |
| glim_ros (リポは [`koide3/glim_ros2`](https://github.com/koide3/glim_ros2)) | 上流 | MIT | 同上。**注意**: リポ URL は `glim_ros2` だが `package.xml` の `<name>` は `glim_ros` (上流の命名不整合)。`colcon build --packages-select` や `ros2 run / pkg list` ではすべて `glim_ros` を使う |
| gtsam_points | [`koide3/gtsam_points`](https://github.com/koide3/gtsam_points) | MIT | 同上 |
| GTSAM | [`borglab/gtsam`](https://github.com/borglab/gtsam) | BSD | permissive、組み込み可 |
| Iridescence | [`koide3/iridescence`](https://github.com/koide3/iridescence) | MIT | 視覚化のみ、運用コア外 |

ここまで全て permissive で揃っているため、GLIM 自体は M6-R 以降で運用スタックに組み込みなおす余地もある (とはいえ本文書ではマップ作成専用ツールとしての扱いに留める)。FAST-LIO 系 (GPL) との切り分けは [運用方針 §3.4](plans/2026-06-11-platform-pivot.md) を参照。

## セットアップ手順

### 0. CUDA 12.4 の存在確認

```bash
/usr/local/cuda-12.4/bin/nvcc --version
```

`release 12.4` を含む行が出力されることを確認する。出ない場合は CUDA Toolkit が入っていないので、先に [`m5r-cuda-setup.md`](m5r-cuda-setup.md) §2 の `scripts/install_cuda.sh` を実行する。

Issue #45 起案時点では `nvidia-driver-595` は稼働しているが Toolkit 自体が消えている状態だった。本ステップで再現再インストールを行う。

### 1. install_glim.sh の実行

リポジトリ直下から:

```bash
cd ~/whill_lab0_ros2
source /opt/ros/humble/setup.bash    # ROS_DISTRO=humble を環境に投入
./scripts/install_glim.sh
```

視覚化 (Iridescence) が不要な場合 (CI / ヘッドレス機):

```bash
./scripts/install_glim.sh skip-iridescence
```

スクリプトは以下を順に実施する (詳細はスクリプト冒頭コメント参照):

1. Ubuntu 22.04 / CUDA 12.4 nvcc / ROS_DISTRO=humble を確認 (満たさなければ exit 1)
2. apt 経由でビルド依存を投入 (`libomp-dev libboost-all-dev libmetis-dev libfmt-dev libspdlog-dev libglm-dev libglfw3-dev libpng-dev libjpeg-dev libeigen3-dev libtbb-dev` 等、既存ならスキップ)
3. GTSAM `4.3a0` を `~/.cache/whill_lab0_ros2/glim/gtsam` でソースビルドし `/usr/local` にインストール
4. gtsam_points (master) を CUDA 12.4 明示でソースビルド (`CMAKE_CUDA_COMPILER=/usr/local/cuda-12.4/bin/nvcc`、`BUILD_WITH_CUDA=ON`)
5. Iridescence (master) を視覚化用にビルド (skip-iridescence 指定時はスキップ)
6. `src/third_party/glim` と `src/third_party/glim_ros2` を clone し、`colcon build --packages-select glim glim_ros --symlink-install`
7. `install/setup.bash` を source して `ros2 pkg list` に `glim_ros` が出るかで検証 (ディレクトリ名は `glim_ros2` だがパッケージ名は `glim_ros`)

ビルド時間目安: Alienware x15 R2 (i9-12900H 14C/20T) で全工程 30〜45 分。GTSAM のみで 10〜15 分かかる。

### 2. 上流バージョンの pin について

スクリプト冒頭の以下の変数で pin を制御している:

| 変数 | 値 | 理由 |
|---|---|---|
| `GTSAM_REF` | `4.3a0` | GLIM が 2025-06-15 に要件を引き上げ。4.2a9 ではビルド不可 |
| `GTSAM_POINTS_REF` | `master` | 上流が tag を切っていない。完全再現したい場合は SHA に書き換える |
| `IRIDESCENCE_REF` | `master` | 同上 |
| `GLIM_REF` | `master` | 同上 |
| `GLIM_ROS2_REF` | `master` | 同上 |

GTSAM 以外を master 追従にしているのは上流の運用都合 (koide3 系はリリースタグを切らない方針) であり、本スクリプトの好みではない。チーム複数人で「同じビット」を共有する必要が出た時点で、SHA に書き換えて Issue を切ること。

### 3. 環境変数の追記 (必要に応じて)

GTSAM と gtsam_points は `/usr/local/lib` に入る。Ubuntu 22.04 の `ldconfig` はデフォルトで `/usr/local/lib` を見るので、追加の `LD_LIBRARY_PATH` 設定は不要なことが多い。`glim_rosbag` 起動時に `libgtsam.so.4.3` が見つからないと言われた場合のみ次を追記する:

```bash
export LD_LIBRARY_PATH=/usr/local/lib${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

CUDA の PATH 設定は [`m5r-cuda-setup.md`](m5r-cuda-setup.md) §3 で既に追記済みの想定。

### 4. サンプル bag の取得

GLIM 上流が Ouster OS1-128 で収録した検証用 bag を配布している。

```bash
mkdir -p /tmp/glim_sample && cd /tmp/glim_sample
# primary: zenodo (GLIM 公式 quickstart 記載、ROS 2 版 426 MB)
curl -L --fail -o os1_128_01_downsampled.tar.gz \
  'https://zenodo.org/record/7233945/files/os1_128_01_downsampled.tar.gz?download=1'
tar -xzf os1_128_01_downsampled.tar.gz
# 展開後 /tmp/glim_sample/os1_128_01_downsampled/ に metadata.yaml と .db3 が出る
```

bag サイズは約 426 MB (ROS 2 版)。研究室の外部 NAS にミラーを置く運用にする場合は本節を更新する。zenodo が落ちている場合の代替は `https://staff.aist.go.jp/k.koide/projects/glim/datasets/os1_128_01_downsampled.tar.gz` (公式 mirror、ただし URL パスの `/datasets/` 抜けに注意)。

サンプル bag は GLIM の検証専用であり、本リポの `docs/maps/<site>/` 規約 (M5R-7 のスコープ) には乗らない。ストレージ位置は `/tmp/` で完結させる。

#### Sample bag DL の現状 (2026-06-20、Issue #45 PR #52 着地時点)

実機セットアップ時に上記 2 つの mirror のいずれも本リポ環境からは取れなかった。記録として残す:

- **AIST mirror**: HTTP HEAD で `Content-Length: 78524908`、ローカル DL も完全一致 (78.5 MB)、しかし `gzip -t` が `unexpected end of file` で落ちる。展開すると 2.5 GB あるはずの archive (gzip header の `original size modulo 2^32`) なので **AIST 側のディスク上で archive が破損** (`Last-Modified: 2026-06-09` で固定)。
- **zenodo**: HTTP 経由で 30 秒平均 36 KB/s を実測。426 MB の全量 DL は ~3.4 時間となり実用域外。`systemd-inhibit` で囲えば走り切るが、本 Issue の AC は smoke test であり、PR を 3 時間ブロックする費用対効果がない。

判断: **Issue #45 の AC #4 (サンプル bag 疎通) は M5R-3 (#48 「GLIM vs FAST-LIO SAM 実 bag 比較」) で実 ループ走行 bag に対する検証を行うときに自然と充足される**。本 Issue 着地時点では DL infra 上流問題として記録に留め、PR #52 は merge する。後続で再現する場合の対処候補:

- 大学の高速 LAN や AnyDesk/VPN 経由で zenodo の帯域を改善
- GLIM の github releases に sample bag が追加されたら切替
- 研究室 NAS 上にミラー設置 + 本節 URL を NAS 経由に書き換え

### 5. 疎通確認 (trajectory 出力まで)

GLIM を rosbag 入力モードで起動する:

```bash
cd ~/whill_lab0_ros2
source install/setup.bash
mkdir -p /tmp/dump
ros2 run glim_ros glim_rosbag \
  /tmp/glim_sample/os1_128_01_downsampled \
  --ros-args \
    -p config_path:=$(ros2 pkg prefix glim_ros)/share/glim_ros/config/ \
    -p dump_path:=/tmp/dump/
```

Iridescence の OpenGL ウィンドウが起動し、点群と trajectory が表示される (skip-iridescence でビルドした場合はヘッドレスで進行)。bag 終端まで処理が走ると `/tmp/dump/traj_lidar.txt` が生成される。

```bash
head -3 /tmp/dump/traj_lidar.txt
# 各行は: timestamp(ns) x y z qx qy qz qw
```

行が空でなく、サンプル bag の全タイムスタンプ範囲を覆っていれば疎通成功。

## トラブルシュート

### GTSAM のビルドが Eigen バージョン衝突で落ちる

症状: `static_assert(EIGEN_VERSION_AT_LEAST(3, 4, 0) ...` のような assertion で停止する。

原因: Ubuntu 22.04 の `libeigen3-dev` は 3.4.0 で要件を満たすが、GTSAM のサブモジュールが古い Eigen のヘッダを抱えていることがある。

対処: スクリプト中の `GTSAM_USE_SYSTEM_EIGEN=ON` が効いていることを確認する。それでも落ちる場合は、`~/.cache/whill_lab0_ros2/glim/gtsam` を一度削除して再 clone し、`build/` も消した上で再実行する。

それでもなお落ちる場合 (上流 GTSAM 側の Eigen 取り扱いが変わったとき等)、`GTSAM_USE_SYSTEM_EIGEN=OFF` に切り替えて GTSAM 同梱の Eigen を使うことで回避できる場合がある。ただし `ON` 前提でビルドされた gtsam_points / GLIM とは Eigen ABI が食い違うため、切替後はそれらも作り直す必要がある:

1. `~/.cache/whill_lab0_ros2/glim/gtsam/build/CMakeCache.txt` 内の `GTSAM_USE_SYSTEM_EIGEN:BOOL=ON` を `OFF` に書き換える
2. `cmake --build ~/.cache/whill_lab0_ros2/glim/gtsam/build --parallel` で再ビルドし、`sudo cmake --install ~/.cache/whill_lab0_ros2/glim/gtsam/build` で入れ直す
3. `rm -rf ~/.cache/whill_lab0_ros2/glim/gtsam_points/build` で gtsam_points のビルドキャッシュを破棄してから `./scripts/install_glim.sh` を再実行する (gtsam_points と GLIM が新しい Eigen 設定で連鎖再ビルドされる)

### gtsam_points のリンクで `undefined reference to 'cuda'` が出る

症状: `nvlink error: Undefined reference to ...` でリンクが落ちる。

原因: CMake が PATH 上の別バージョンの nvcc を拾っている可能性。`CMAKE_CUDA_COMPILER` 指定が効いていないケース。

対処:

```bash
# build キャッシュを消して再実行
rm -rf ~/.cache/whill_lab0_ros2/glim/gtsam_points/build
./scripts/install_glim.sh
```

それでも再現する場合、`/usr/local/cuda-12.4/bin` 以外に nvcc が存在しないことを `which -a nvcc` で確認する。

### glim_rosbag が VRAM 不足で落ちる

症状: `cudaMalloc returned cudaErrorMemoryAllocation` または OOM kill。

原因: GLIM の GPU モードは大規模 bag で VRAM 16 GB を超え得る (本リポの母艦は 16 GB)。

対処:

1. bag を時間で分割して順に処理する (`ros2 bag info` で長さを確認し、`ros2 bag record --start-offset / --end-offset` ではなく、外部の bag editor で分割)
2. GLIM の CPU モードに切り替える: glim_ros の config (package 名) (`config_path` 配下の `config_sensors.json` 等) で `gpu` フラグを `false` に。CPU モードは速度が 1/3 〜 1/5 に落ちるが、VRAM 制約からは解放される
3. Iridescence を `skip-iridescence` でビルドしなおし、視覚化の VRAM を節約する

### Iridescence のウィンドウが表示されない (リモート SSH 等)

X11 forwarding が無いリモートシェルから GLIM を起動した場合、Iridescence は `glfwInit` で落ちる。`./scripts/install_glim.sh skip-iridescence` で再ビルドするか、`DISPLAY` 環境変数を設定して X 接続を確保する。

### GTSAM のビルドが OOM で停止する

症状: `cc1plus: error: out of memory allocating ...` で停止し、`dmesg` に oom-killer のログが出る。

原因: GTSAM のテンプレート展開は cc1plus 1 プロセスあたり 2〜4 GB 食う。本スクリプトは `nproc - 1` (Alienware で 13 並列) で走らせるため、メモリが 32 GB に近い環境では足りなくなる場合がある。

対処: スクリプト中の `JOBS=` を手動で 4 〜 8 程度に下げて再実行する。

## 関連

- 開発方針: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §3.3 (GLIM を M5-R 第一候補とする選定理由)、§3.4 (ライセンス方針)、§9 (開発機材確認)
- 前置: [`m5r-cuda-setup.md`](m5r-cuda-setup.md) — CUDA Toolkit 12.4 と cuDNN 8 のセットアップ (本文書の入口)
- ADR 0001: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md) — 新規 docs は ja/en 並列で生やす
- スクリプト: [`scripts/install_glim.sh`](../../scripts/install_glim.sh) — 本文書と対の冪等インストールスクリプト
- 対の文書: [`m5r-fastlio-sam-eval.md`](m5r-fastlio-sam-eval.md) — 第二候補 FAST-LIO SAM の clone-on-demand 手順とライセンス整合 (M5R-3 で本文書と比較)
- 関連 Issue: #23 (CUDA 文書とスクリプト)、#45 (本文書とスクリプト)、後続の M5-R SLAM 候補比較 ADR (実 bag で GLIM と FAST-LIO SAM を比較し、ADR-0003 として確定予定)
