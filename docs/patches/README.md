# Upstream patches

Patches against packages imported via `vcs import` into `src/third_party/`.
The third-party directory is gitignored to keep this repo close to the
upstream baselines, so any local modifications live here as patch files
and get reapplied after re-importing.

## Apply

```sh
cd src/third_party/<pkg>
git apply ../../../docs/patches/<patch-file>.patch
colcon build --packages-select <pkg> --symlink-install
```

## Reverse

```sh
cd src/third_party/<pkg>
git apply -R ../../../docs/patches/<patch-file>.patch
```

## Patches

### 2026-05-29-fastlio-self-exclusion.patch

Adds WHILL chair-body self-return exclusion inside FAST-LIO's
`velodyne_handler`. The cylinder + forward-arc sector mask runs
in-process so the IMU↔LiDAR motion-compensation timing stays intact —
a Python republish on `/velodyne_points_filtered` immediately broke
FAST-LIO with "No Effective Points" and VoxelGrid overflow on the
2026-05-29 live test.

Adds three params (yaml: `preprocess.self_radius`, `self_z_min`,
`self_z_max`, `self_sectors`). Defaults are no-op so a stock build with
this patch but no yaml change behaves identically to upstream
FAST-LIO. The matching yaml lives in
`src/whill_localization/config/velodyne_whill.yaml`.

Files touched: `src/preprocess.h`, `src/preprocess.cpp`,
`src/laserMapping.cpp`.

The 2026-05-29 bisect on a static chair found that the third sector
(outer right, r=1.85–2.15 m) starves FAST-LIO of registration anchors.
Cylinder + the two inner sectors is the holding configuration; the
outer-right arc remains in the cloud until Phase B brings
FASTLIO2_SAM_LC with loop closure.

This patch should be folded into the M5-e Phase B fork of FAST-LIO
when that lands.
