# M5-R prerequisite: CUDA Toolkit 12.4 and cuDNN 8 setup

Language: [日本語](../ja/m5r-cuda-setup.md) | [English](m5r-cuda-setup.md)

## Goal

[GLIM](https://github.com/koide3/glim), the first-candidate map-building SLAM
for M5-R (the offline map-production pipeline), requires CUDA Toolkit (the
upstream-validated line is 12.x) and cuDNN to build and run in GPU mode. In
this repository the campus bag is post-processed on a development host rather
than on the chair-mounted PC, so the prerequisite must be satisfied on the
dev machine (the Alienware x15 R2) rather than on the onboard machine.

There is a second reason to stand the CUDA Toolkit up independently of the
ROS-side packages. There are several ways to install it — (a) the NVIDIA
official apt repo, (b) the runfile installer, (c) a conda environment — and
in the lab's past, the difference between these has been a recurring source
of rework. This document and its companion script standardise on (a) and
make the procedure idempotent.

The selection rationale and how it traces back to the project requirements
is in [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
§3.3 and §9.

## Host environment

| | |
|--|--|
| Host | Alienware x15 R2 (`systemlab-Alienware-x15-R2`) |
| OS | Ubuntu 22.04.5 LTS (jammy) |
| Kernel | 6.8.0-124-generic |
| GPU | NVIDIA GeForce RTX 3080 Laptop GPU (16 GB VRAM, Ampere CC 8.6) |
| NVIDIA Driver | 595.71.05 (`nvidia-driver-595`; driver reports CUDA Version 13.2) |
| Verified | 2026-06-13 |

The 595 driver series is forward-compatible with the entire CUDA 12.x line
(12.0 through 12.6), so pinning the Toolkit at 12.4 is safe. The minimum
required driver is 525 (the CUDA 12.0 contemporary); if the host is older,
upgrade the driver first.

The script targets Ubuntu 22.04 only. The CUDA apt repo for `ubuntu2204` and
`ubuntu2404` differ both in URL and in keyring layout, and this repository
is pinned to 22.04 by the ROS 2 humble requirement (see CLAUDE.md and the
platform-pivot policy §9). We make the mismatch loud rather than silent —
the script exits early in `require_jammy`.

## Setup procedure

### 1. Pre-check

Confirm that the driver is loaded correctly:

```bash
nvidia-smi
```

Check that `Driver Version` is 525 or higher and that the `CUDA Version`
field (note: this is the *maximum runtime the driver can support*, not the
state of any installed Toolkit) is 12.0 or higher. The GPU name and VRAM
size should also match expectations.

### 2. Run install_cuda.sh

From the repository root:

```bash
cd ~/whill_lab0_ros2
./scripts/install_cuda.sh
```

The script runs the following in order (see the header comment in the script
for the details and rationale of each step):

1. Verify the host is Ubuntu 22.04 with an NVIDIA driver present (exit 1
   otherwise).
2. Register the NVIDIA apt repo keyring (`cuda-keyring`) — skipped if
   already installed.
3. Install `cuda-toolkit-12-4` via apt — skipped if already installed.
4. Install `libcudnn8` and `libcudnn8-dev` — skipped if already installed.
5. Verify by running `nvcc --version` and reading the major version from
   `cudnn_version.h`.
6. Print PATH guidance to stderr (the script does *not* edit any rc file).

To avoid touching a stale apt cache, the script runs `apt-get update`
immediately after registering the `cuda-keyring`.

### 3. Configure PATH (append to your shell rc)

The script deliberately does not touch your rc file. The reasons are
(a) several CUDA versions may co-exist under `/usr/local/`, and silently
exporting one of them on shell startup can collide with other workflows,
and (b) this matches the convention already established by
`configure_proxy.sh` (the user owns their shell environment).

For bash (append to `~/.bashrc`):

```bash
export PATH=/usr/local/cuda-12.4/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

For zsh (append to `~/.zshrc`):

```zsh
export PATH=/usr/local/cuda-12.4/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-12.4/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
```

After appending, open a fresh shell and verify that `which nvcc` returns
`/usr/local/cuda-12.4/bin/nvcc`.

### 4. Smoke test (a minimal vectorAdd sample)

CUDA Toolkit 12.x does not ship sample programs. Through 11.7 they lived
under `/usr/local/cuda/samples/`, but starting with 12.0 they were split
out into a separate repository
([NVIDIA/cuda-samples](https://github.com/NVIDIA/cuda-samples)). Since we
do not want to depend on an external clone for a basic smoke test that
runs before we even start touching GLIM, the minimal `vectorAdd` is
inlined here.

Create `/tmp/vectorAdd.cu` with the following contents:

```cuda
// vectorAdd.cu — minimal CUDA Toolkit smoke-test
#include <cstdio>
#include <cuda_runtime.h>

// Swallowing the return codes makes a driver / Toolkit runtime mismatch
// surface as a segfault or silent garbage instead of a clear message. For
// a smoke test that is exactly the failure mode we want to catch, so check
// every CUDA call.
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
  CUDA_CHECK(cudaGetLastError());        // catch kernel launch failure
  CUDA_CHECK(cudaDeviceSynchronize());   // catch failures inside the kernel
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

Build and run:

```bash
cd /tmp
/usr/local/cuda-12.4/bin/nvcc vectorAdd.cu -o vectorAdd
./vectorAdd
# expected: Result = PASS
```

If you see `Result = PASS`, the nvcc + runtime + driver triplet is
consistent. If it fails, consult the troubleshooting notes below.

## Troubleshooting

### Secure Boot enabled

When UEFI Secure Boot is enabled, the NVIDIA kernel module may fail to
load because it is waiting for a MOK (Machine Owner Key) signature, and
`nvidia-smi` returns `NVIDIA-SMI has failed because it couldn't communicate
with the NVIDIA driver`.

Check: `mokutil --sb-state` — look for `SecureBoot enabled`. Choose one of:

- Quick triage: disable Secure Boot in UEFI and reboot.
- Permanent fix: register the MOK with
  `sudo mokutil --import /var/lib/shim-signed/mok/MOK.der`, then select
  `Enroll MOK` from the blue screen on the next reboot.

The lab host (Alienware x15 R2) ships with Secure Boot disabled, so this
issue does not show up by default.

### Kernel module conflict (Nouveau)

If Ubuntu's open-source `nouveau` driver is loaded, the NVIDIA proprietary
driver fails to load.

Check:

```bash
lsmod | grep nouveau
```

If empty, you are fine. Otherwise, create `/etc/modprobe.d/blacklist-nouveau.conf`
with:

```
blacklist nouveau
options nouveau modeset=0
```

then run `sudo update-initramfs -u` and reboot. The apt install of
`nvidia-driver-*` usually does this for you, but verify explicitly when
doing the install by hand.

### PRIME profile (GPU switching on Optimus laptops)

On hybrid machines (Intel iGPU + NVIDIA dGPU), `prime-select` switches the
default GPU. If the dGPU is parked in a power-save state, `nvidia-smi` may
respond while CUDA kernels still fail to launch.

Check:

```bash
prime-select query
# returns nvidia / intel / on-demand
```

If you use CUDA all the time, run `sudo prime-select nvidia` and reboot.
The tradeoff is battery life — for non-CUDA workloads `on-demand` may be
preferable (in that case prepend
`__NV_PRIME_RENDER_OFFLOAD=1 __GLX_VENDOR_LIBRARY_NAME=nvidia` per command,
or pin `CUDA_VISIBLE_DEVICES=0` in the environment).

### Driver / Toolkit compatibility table

NVIDIA publishes the driver / Toolkit compatibility matrix in the
[CUDA Compatibility documentation](https://docs.nvidia.com/deploy/cuda-compatibility/index.html).
Excerpt:

| Minimum driver | Maximum supported CUDA Toolkit |
|----------------|-------------------------------|
| 525.60.13 | CUDA 12.0 |
| 545.23.06 | CUDA 12.3 |
| 550.54.14 | CUDA 12.4 |
| 555.42.02 | CUDA 12.5 |
| 595.x (this host) | Compatible across the full CUDA 12.x line |

Driver 595 sits comfortably above all of the 12.x Toolkit minimums, so
pinning the Toolkit at 12.4 is the safe-side choice. When the Toolkit
version is bumped in the future (for example, if GLIM moves to requiring
12.6), re-check this table to decide whether the driver also needs an
upgrade.

### apt repo keyring rotation

The `cuda-keyring` package is versioned by NVIDIA (`1.0-1` → `1.1-1` → …).
The script currently pins `1.1-1`. If NVIDIA retires the 1.1-1 deb in the
future, do one of:

- Bump the `CUDA_KEYRING_DEB` variable in the script to the new version.
- Manually check the [CUDA repo index](https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/x86_64/)
  to find the current deb.

Keyring rotation is typically announced on the NVIDIA Developer Blog.

## Related

- Strategy: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
  §3.3 (rationale for GLIM as the M5-R first candidate) and §9 (development
  hardware check).
- ADR 0001: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md) —
  new documents are authored in parallel under `docs/ja/` and `docs/en/`.
- Script: [`scripts/install_cuda.sh`](../../scripts/install_cuda.sh) — the
  idempotent installer that this document is paired with.
- Downstream: [`m5r-glim-setup.md`](m5r-glim-setup.md) — source-build
  procedure for GLIM (the M5-R first-candidate SLAM) that takes this
  document as its entry point.
- Related issues: #23 (this document and script), #45 (the downstream
  GLIM source build), and the upcoming M5-R SLAM candidate comparison
  ADR (to be settled by an empirical comparison against FAST-LIO SAM on
  real bags).
