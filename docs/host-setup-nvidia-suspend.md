# ホスト設定: NVIDIA サスペンド/レジューム CUDA 修正

- 対象機: Alienware x15 R2 (RTX 3080 Ti / driver 595 / kernel 6.8 / Ubuntu 22.04)
- 対応 Issue: [#76](https://github.com/Iruazu/whill_lab0_ros2/issues/76)
- スクリプト: `scripts/install_nvidia_suspend_fix.sh`

## 何を直しているか

サスペンド → レジューム直後、GPU を使う ROS 2 プロセス (GLIM の
`glim_rosbag` 等) を新規に起動すると、`libodometry_estimation_gpu.so` を
load した段階で `cudaErrorUnknown` が連発し、0.6 秒程度で abort する。
`nvidia-smi` は正常に応答するので一見 GPU は生きて見えるが、実際は
`nvidia_uvm` (CUDA 用の unified memory kernel module) が resume 後の状態
不整合で新規 CUDA コンテキストを作れない。

## なぜ 2 段構成か

現行 (2026-07-08 時点) の `/proc/driver/nvidia/params`:

```
PreserveVideoMemoryAllocations: 2   ← NVIDIA driver パッケージが既定で設定
UseKernelSuspendNotifiers:      0   ← ここが抜けている
```

`PreserveVideoMemoryAllocations=2` だけでは kernel 6.x + driver 595 の組合せ
では UVM が復帰しきらない。追加で:

1. **`NVreg_UseKernelSuspendNotifiers=1`** で driver に `pm_notifier` を
   登録させ、サスペンド前に UVM を明示的に quiesce
2. **resume 時に `nvidia_uvm` を reload** することで、UVM state machine を
   確実にクリーンな状態から開始

の 2 段で確実に直る。1. だけでは効かないケースが上流 forum でも報告されて
いる。2. だけでも当面動くが、driver が自前で pm_notifier を持たないため
suspend 中の GPU アクセスが不定になる。両方入れる。

## インストール

```bash
sudo ./scripts/install_nvidia_suspend_fix.sh
```

書き込まれるファイル:

| ファイル | 目的 |
|---------|------|
| `/etc/modprobe.d/whill-nvidia-uvm.conf` | `NVreg_UseKernelSuspendNotifiers=1` |
| `/lib/systemd/system-sleep/whill-nvidia-uvm-reload` | resume 時に `nvidia_uvm` を reload |

**再起動** が必要 (nvidia.ko は boot 時に modprobe.d を読むため)。
systemd sleep hook 側は再起動なしで即有効。

## 検証

再起動後:

```bash
./scripts/install_nvidia_suspend_fix.sh --verify
```

期待出力:

```
== installed files ==
  present   /etc/modprobe.d/whill-nvidia-uvm.conf
  present   /lib/systemd/system-sleep/whill-nvidia-uvm-reload
== live driver params ==
  PreserveVideoMemoryAllocations: 2
  UseKernelSuspendNotifiers: 1
```

手動 suspend/resume テスト:

```bash
systemctl suspend
# 蓋を開ける / power ボタンで復帰
./scripts/m5r3_run_glim.sh \
  docs/m5r-bench-data/2026-07-08-outdoor-corrected/bag \
  /tmp/glim-verify --force
grep -c cudaErrorUnknown /tmp/glim-verify/run.log
# 期待値: 0
```

修正前の 2026-07-08 セッションでは同じコマンドで 39 件検出、0.66 秒 abort。

## アンインストール

```bash
sudo ./scripts/install_nvidia_suspend_fix.sh --uninstall
```

modprobe.d の変更は次回再起動まで有効なまま (nvidia.ko が既に load 済みで
新しい設定を読まないため)。それでも問題は起きないので特に対応不要。

## スコープと制限

- **本設定は開発機 1 台のみ対象**。将来 M9 で車載機を分離する際は、その
  機体の driver / kernel 版に応じて再評価する
- `nvidia_uvm` の reload は「suspend 前の CUDA 状態を捨てる」動作。
  suspend 前に走っていた CUDA プロセスが suspend 中に固まっていた場合、
  それらは resume 時に確実に殺す事になる。GLIM や DUFOMap の run 途中で
  蓋を閉じた場合は run が失敗する (これは元々そうであるべき挙動)
- driver 596 系以降で上流に恒久修正が入ったら、この対処は不要になる可能性
  がある。その時点で本ファイルと Issue #76 を再評価する
