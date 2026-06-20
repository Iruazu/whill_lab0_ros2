# M5-R 前置: CUDA Toolkit 12.4 と cuDNN 8 のセットアップ

Language: [日本語](m5r-cuda-setup.md) | [English](../en/m5r-cuda-setup.md)

## 目的

M5-R (オフラインのマップ作成パイプライン) で第一候補としている [GLIM](https://github.com/koide3/glim) は、GPU モードでビルド・実行するために CUDA Toolkit (公式検証バージョンは 12.x 系) と cuDNN を要求する。本リポではキャンパス bag からの後処理マッピングを母艦 PC で行う想定なので、運用車載機ではなく開発機側 (Alienware x15 R2) でこの前提を満たす必要がある。

CUDA Toolkit を「ROS 側パッケージ」と独立に立てる理由はもう一つある。Toolkit のインストール経路は (a) NVIDIA 公式 apt repo、(b) runfile、(c) conda、と複数あり、過去の研究室作業ではこの差分が手戻りの大きな原因になってきた。本文書とスクリプトは (a) の apt repo に統一し、手順を冪等化する。

採択経緯と要件のひもづけは [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §3.3 と §9 を参照。

## 前提環境

| | |
|--|--|
| ホスト | Alienware x15 R2 (`systemlab-Alienware-x15-R2`) |
| OS | Ubuntu 22.04.5 LTS (jammy) |
| カーネル | 6.8.0-124-generic |
| GPU | NVIDIA GeForce RTX 3080 Laptop GPU (16 GB VRAM、Ampere CC 8.6) |
| NVIDIA Driver | 595.71.05 (`nvidia-driver-595`、driver 報告 CUDA Version 13.2) |
| 動作確認日 | 2026-06-13 |

ドライバ 595 系は CUDA 12.x 全域 (12.0〜12.6) で前方互換が保証されているため、Toolkit を 12.4 に固定しても問題ない。最低ラインは driver 525 (CUDA 12.0 リリース時の同期版) で、これより古い場合は先にドライバを更新する。

スクリプトは Ubuntu 22.04 専用にしてある。CUDA の apt repo は ubuntu2204 と ubuntu2404 で URL も鍵の貼り方も別物で、本リポは ROS 2 humble の制約により 22.04 に固定されている (CLAUDE.md と方針文書 §9)。混在検出をサイレントにせず早期 exit する設計にしている。

## セットアップ手順

### 1. 事前確認

ドライバが正しくロードされているか確認する。

```bash
nvidia-smi
```

`Driver Version` が 525 以上、`CUDA Version` 欄 (これは「ドライバが対応可能な最大ランタイム」であり、Toolkit のインストール状態とは別物) が 12.0 以上であることを確認する。GPU 名と VRAM 容量も期待通りであること。

### 2. install_cuda.sh の実行

リポジトリ直下から:

```bash
cd ~/whill_lab0_ros2
./scripts/install_cuda.sh
```

スクリプトは以下を順に実施する (詳細はスクリプト冒頭コメント参照):

1. Ubuntu 22.04 / NVIDIA ドライバの存在を確認 (満たさなければ exit 1)
2. NVIDIA 公式 apt repo の鍵 (`cuda-keyring`) を登録 (既にあれば skip)
3. `cuda-toolkit-12-4` を apt 経由でインストール (既存なら skip)
4. `libcudnn8` / `libcudnn8-dev` をインストール (既存なら skip)
5. `nvcc --version` と `cudnn_version.h` のメジャー番号で検証
6. PATH 追記例を stderr に案内 (rc ファイルへの自動追記はしない)

途中で apt キャッシュが古いまま参照される事故を避けるため、`cuda-keyring` 登録直後に `apt-get update` が走るようになっている。

### 3. PATH 設定 (シェル rc に追記)

スクリプトは意図的に rc ファイルを触らない。理由は (a) ユーザーが複数の CUDA バージョンを使い分けるケースがあり、シェル起動時の自動 PATH 投入が衝突するため、(b) `configure_proxy.sh` と同じ運用方針 (シェル env はユーザーが書く) で揃えるため。

bash の場合 (`~/.bashrc` に追記):

```bash
export PATH=/usr/local/cuda-12.4/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

zsh の場合 (`~/.zshrc` に追記):

```zsh
export PATH=/usr/local/cuda-12.4/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

追記後、新しいシェルを開いて `which nvcc` が `/usr/local/cuda-12.4/bin/nvcc` を返すことを確認する。

### 4. 動作確認 (vectorAdd minimal sample)

CUDA Toolkit 12.x にはサンプルが同梱されていない。11.7 までは `/usr/local/cuda/samples/` に展開されていたが、12.0 以降は外部リポ ([NVIDIA/cuda-samples](https://github.com/NVIDIA/cuda-samples)) に分離された。本リポでは GLIM 以前の段階での疎通確認に外部 clone を要求したくないので、最小の vectorAdd を本文書に直書きする。

`/tmp/vectorAdd.cu` を以下の内容で作成する:

```cuda
// vectorAdd.cu — CUDA Toolkit 疎通確認用の最小サンプル
#include <cstdio>
#include <cuda_runtime.h>

// 戻り値を握り潰すと、ドライバと Toolkit のランタイム不整合が起きたときに
// セグフォルトかゴミ値で落ち、原因が分からない。疎通確認の目的のため
// 全ての CUDA 呼び出しはここで検査する。
#define CUDA_CHECK(call) do { \
  cudaError_t err__ = (call); \
  if (err__ != cudaSuccess) { \
    fprintf(stderr, "CUDA error at %s:%d: %s\n", __FILE__, __LINE__, cudaGetErrorString(err__)); \
    return 1; \
  } \
} while (0)

__global__ void vectorAdd(const float *a, const float *b, float *c, int n) {
  int i = blockDim.x * blockIdx.x + threadIdx.x;
  if (i < n) c[i] = a[i] + b[i];
}

int main() {
  const int n = 1 << 16;
  size_t bytes = n * sizeof(float);
  float *h_a = (float*)malloc(bytes), *h_b = (float*)malloc(bytes), *h_c = (float*)malloc(bytes);
  for (int i = 0; i < n; i++) { h_a[i] = (float)i; h_b[i] = (float)(2*i); }

  float *d_a, *d_b, *d_c;
  CUDA_CHECK(cudaMalloc(&d_a, bytes));
  CUDA_CHECK(cudaMalloc(&d_b, bytes));
  CUDA_CHECK(cudaMalloc(&d_c, bytes));
  CUDA_CHECK(cudaMemcpy(d_a, h_a, bytes, cudaMemcpyHostToDevice));
  CUDA_CHECK(cudaMemcpy(d_b, h_b, bytes, cudaMemcpyHostToDevice));

  int threads = 256, blocks = (n + threads - 1) / threads;
  vectorAdd<<<blocks, threads>>>(d_a, d_b, d_c, n);
  CUDA_CHECK(cudaGetLastError());        // カーネル launch の失敗を捕捉
  CUDA_CHECK(cudaDeviceSynchronize());   // カーネル実行中の失敗を捕捉
  CUDA_CHECK(cudaMemcpy(h_c, d_c, bytes, cudaMemcpyDeviceToHost));

  bool ok = true;
  for (int i = 0; i < n; i++) {
    if (h_c[i] != h_a[i] + h_b[i]) { ok = false; break; }
  }
  printf("Result = %s\n", ok ? "PASS" : "FAIL");

  cudaFree(d_a); cudaFree(d_b); cudaFree(d_c);
  free(h_a); free(h_b); free(h_c);
  return ok ? 0 : 1;
}
```

ビルドして実行する:

```bash
cd /tmp
/usr/local/cuda-12.4/bin/nvcc vectorAdd.cu -o vectorAdd
./vectorAdd
# 期待出力: Result = PASS
```

`Result = PASS` が出れば、nvcc + ランタイム + ドライバの整合が取れている。これが失敗する場合は次節のトラブルシュートを参照。

## トラブルシュート

### Secure Boot 有効環境

UEFI で Secure Boot が有効だと、NVIDIA カーネルモジュールが MOK (Machine Owner Key) 署名待ちで読み込まれず、`nvidia-smi` が `NVIDIA-SMI has failed because it couldn't communicate with the NVIDIA driver` を返すことがある。

確認: `mokutil --sb-state` で `SecureBoot enabled` と出るか見る。対処は以下のいずれか:

- 一時的な切り分けとして、UEFI 設定で Secure Boot を無効化して再起動
- 恒久対応として、`sudo mokutil --import /var/lib/shim-signed/mok/MOK.der` で MOK を登録し、次回再起動時の青画面で `Enroll MOK` を選んで完了する

研究室の本機 (Alienware x15 R2) は購入時点で Secure Boot 無効。デフォルトでこの問題は出ない。

### kernel module 競合 (Nouveau)

Ubuntu のオープンソースドライバ Nouveau がロードされていると、NVIDIA 公式ドライバが load 直前に失敗する。

確認:

```bash
lsmod | grep nouveau
```

何も出なければ問題なし。出る場合は `/etc/modprobe.d/blacklist-nouveau.conf` を作って次を書き、`sudo update-initramfs -u` 後に再起動する:

```
blacklist nouveau
options nouveau modeset=0
```

`nvidia-driver-*` を apt で入れた直後の再起動でこの設定は自動で投入されることが多いが、手動セットアップ時は明示的に確認する。

### PRIME profile (Optimus 機での GPU 切替)

ハイブリッド機 (Intel iGPU + NVIDIA dGPU) では、`prime-select` で GPU の優先度が切り替わる。dGPU が省電力モードで止まっていると `nvidia-smi` が応答するのに CUDA カーネルが起動しない、という状態になり得る。

確認:

```bash
prime-select query
# nvidia / intel / on-demand のいずれか
```

CUDA を常用するなら `sudo prime-select nvidia` 後に再起動する。バッテリ駆動時間とトレードオフになるので、開発用途以外では `on-demand` でもよい (この場合 `__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` を都度付けるか、`CUDA_VISIBLE_DEVICES=0` を環境変数で固定する)。

### ドライバ-Toolkit 互換表

NVIDIA は driver と CUDA Toolkit の互換表を [CUDA Compatibility ドキュメント](https://docs.nvidia.com/deploy/cuda-compatibility/index.html) で公開している。概要を抜粋:

| Driver 最小バージョン | 対応 CUDA Toolkit 最大版 |
|---------------------|--------------------------|
| 525.60.13 | CUDA 12.0 |
| 545.23.06 | CUDA 12.3 |
| 550.54.14 | CUDA 12.4 |
| 555.42.02 | CUDA 12.5 |
| 595.x (本環境) | CUDA 12.x 系全域で動作 |

driver 595 は CUDA 12.x のいずれにも対応できる「上位互換」位置にいるので、Toolkit 12.4 ピン留めは安全側の選択。Toolkit のバージョンを将来上げるとき (例: GLIM が 12.6 要件になる) は、本表に照らしてドライバの追加更新が必要かを再評価する。

### apt repo の鍵切替

`cuda-keyring` パッケージは NVIDIA 側で版が更新される (`1.0-1` → `1.1-1` → ...)。スクリプトは現行の `1.1-1` を固定で取得しているが、将来 NVIDIA が 1.1-1 deb を取り下げた場合は次のいずれかで対処する:

- スクリプトの `CUDA_KEYRING_DEB` 変数を新版に書き換える
- 手動で [CUDA repo の本家ページ](https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/) から最新 deb を確認・取得する

鍵切替の通知は通常 NVIDIA Developer Blog に出る。

## 関連

- 開発方針: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md) §3.3 (M5-R で GLIM を第一候補とする選定理由)、§9 (開発機材確認)
- ADR 0001: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md) — 新規 docs は ja/en 並列で生やす
- スクリプト: [`scripts/install_cuda.sh`](../../scripts/install_cuda.sh) — 本文書と対の冪等インストールスクリプト
- 後段: [`m5r-glim-setup.md`](m5r-glim-setup.md) — 本文書を入口とした GLIM (M5-R 第一候補 SLAM) のソースビルド手順
- 関連 Issue: #23 (本文書とスクリプトの整備)、#45 (後段の GLIM ソースビルド)、後続の M5-R SLAM 候補比較 ADR (FAST-LIO SAM との実 bag 比較で確定予定)
