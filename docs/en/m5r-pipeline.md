# M5-R map-building pipeline

Language: [日本語](../ja/m5r-pipeline.md) | [English](m5r-pipeline.md)

Reference doc for the M5-R map-building pipeline. Issue #49 added the
dynamic-removal (DUFOMap) section, #50 added the occupancy-grid
conversion section, and #51 (M5R-7) wired in bag capture / SLAM
execution / `docs/maps/<site>/` placement / the M6-R handoff contract
for the M5-R-complete version.

Intended reader: someone generating a new map (= an offline workflow
done by the user themselves). M6-R consumers only need the
"Handoff to M6-R" section at the end for the input contract.

## End-to-end pipeline

```
[1] Bag recording (M4-R bringup)
        | /velodyne_points + /imu/data_rep145 + /tf_static
        v
    docs/m5r-bench-data/<run-id>/bag/         (gitignored)

[2] SLAM (ADR-0003: GLIM)
        | scripts/m5r3_run_glim.sh
        v
    docs/m5r-bench-data/<run-id>/glim-out/    (gitignored)
        - NNNNNN/{points,intensities,...}_compact.bin
        - traj_lidar.txt, manifest.yaml

[3] Dynamic-object removal (ADR-0004: DUFOMap)
        | scripts/m5r_run_dufomap.sh
        v
    docs/m5r-bench-data/<run-id>/dufomap-out/static.pcd   (gitignored)

[4] Occupancy-grid conversion (Nav2-compatible)
        | scripts/m5r_pcd_to_occupancy.py <static.pcd> <output-dir>
        v
    <output-dir>/occupancy.pgm + occupancy.yaml
    (typically write into docs/m5r-bench-data/<run-id>/dufomap-out/
     and move in step 5, or point <output-dir> straight at
     docs/maps/<site>/ to skip the mv)

[5] Placement under docs/maps/<site>/ (registry)
        - static.pcd        (gitignored — PCDs are large)
        - occupancy.pgm     (tracked)
        - occupancy.yaml    (tracked)
        - metadata.yaml     (tracked, per ADR-0005)
        - README.md         (tracked, optional)
```

Per-site wall-time budget (Alienware x15 R2, 200 s bag): GLIM ~10 min
(GPU, Iridescence on) + DUFOMap ~3 s + occupancy ~1 s. Bottleneck is
bag capture and GLIM post-processing.

## Bag recording (step 1)

### Preconditions
- M4-R bringup (`whill_localization/launch/odom_bringup_launch.py`)
  starts cleanly.
- Issue #56 `imu_sign_corrector` is running and `/imu/data_rep145` is
  publishing at 100 Hz (`ros2 topic hz /imu/data_rep145`).
- Route: a closed loop is preferred (start/end at the same wall makes
  SLAM loop-closure measurable). M5R-3's reference bag was ~50 m /
  200 s — see ADR-0003 §evaluation conditions.

### Record commands

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash

# 1. bringup (sensors + driver + EKF)
ros2 launch whill_localization odom_bringup_launch.py
```

Second terminal:

```bash
mkdir -p docs/m5r-bench-data/$(date +%Y-%m-%d)-<run-id>
ros2 bag record \
  -o docs/m5r-bench-data/$(date +%Y-%m-%d)-<run-id>/bag \
  /velodyne_points /imu/data_rep145 /tf_static
# drive, Ctrl-C when done
```

Only 3 topics are recorded:
- `/velodyne_points` (10 Hz) — VLP-16 raw
- `/imu/data_rep145` (100 Hz) — REP-145-fixed IMU (Issue #56)
- `/tf_static` — sensor extrinsics

`/imu/data_raw` is also published by the driver but the SLAM-facing
topic is `/imu/data_rep145`. EKF outputs like `/odometry/filtered` are
M4-R verification artifacts; GLIM only uses IMU + LiDAR so we skip
them to keep bag size down.

Naming and gitignore policy for `docs/m5r-bench-data/<run-id>/` lives
in [`../../m5r-bench-data/README.md`](../../m5r-bench-data/README.md).

### Post-record sanity check

```bash
ros2 bag info docs/m5r-bench-data/<run-id>/bag
```

Expected (using the M5R-3 outdoor-loop bag as an example):
- Duration: ~200 s
- `/velodyne_points` Count: ~2000 (200 s × 10 Hz)
- `/imu/data_rep145` Count: ~20000 (200 s × 100 Hz)
- `/tf_static` Count: ≥ 1

If `/imu/data_rep145` rate is much below 100 Hz, the bringup CPU was
saturated during recording. Don't run RViz or rqt alongside the
recording terminal.

## SLAM execution (step 2): GLIM

ADR-0003 selected **GLIM**
([`decisions/0003-mapping-slam-choice.md`](decisions/0003-mapping-slam-choice.md)).
FAST-LIO SAM was rejected for missing an upstream LICENSE file.

### Preconditions
- M5R-1 (#45) CUDA 12.4 + cuDNN setup complete
  ([`m5r-cuda-setup.md`](m5r-cuda-setup.md)).
- GLIM host build complete (`ros2 pkg list | grep glim_ros`)
  ([`m5r-glim-setup.md`](m5r-glim-setup.md)).

### Run command

```bash
scripts/m5r3_run_glim.sh \
  docs/m5r-bench-data/<run-id>/bag \
  docs/m5r-bench-data/<run-id>/glim-out
```

The wrapper:
- Samples peak VRAM every 0.5 s into `vram.log`.
- Writes `manifest.yaml` at exit (SLAM method, bag, start/end times,
  duration, exit code, CUDA version, git commit, ADR-0003 result
  placeholders).
- Idempotent: an existing `<out-dir>` aborts unless `--force` is
  passed.

### Handling old bags (pre-#56)

Bags recorded before Issue #56 carry `/imu/data_raw` in the
gravity-vector convention; GLIM mis-initialises with a known 171°
flip. Rewrite the bag first:

```bash
python3 scripts/m5r3_fix_imu_bag.py \
  docs/m5r-bench-data/<old-run-id>/bag \
  docs/m5r-bench-data/<old-run-id>/bag-imu-fixed
# then pass bag-imu-fixed to m5r3_run_glim.sh
```

See that script's DEPRECATED docstring and ADR-0003 notes for the
full backstory. **New recordings don't need this step** (Issue #56
fixed it at the source).

### Outputs

`<out-dir>/`:
- `NNNNNN/` keyframe dirs × N (e.g. ~18 for a 200 s drive):
  - `points_compact.bin` (binary, Eigen::Vector3f, keyframe-local)
  - `data.txt` (`T_world_origin` 4×4 + other transforms)
  - `intensities_compact.bin`, `normals_compact.bin`, `covs_compact.bin`
- `traj_lidar.txt` (TUM: timestamp tx ty tz qx qy qz qw)
- `manifest.yaml` (ADR-0003 consumer)
- `run.log`, `vram.log`

To get the loop-closure error in ADR-0003 format:

```bash
python3 scripts/m5r3_loop_error.py <out-dir>/traj_lidar.txt
# emits end-to-start distance / loop length / yaw drift
```

## Dynamic-object removal (DUFOMap)

### Why DUFOMap

See [ADR-0004](decisions/0004-dynamic-removal-choice.md). Summary:
ERASOR-family (GPL-3.0 + ROS 1 only) and Removert (no LICENSE file)
were rejected in favour of DUFOMap (BSD-3-Clause, `pip install
dufomap`, ROS-independent).

### Setup

Once on the dev host (Alienware x15 R2, Ubuntu 22.04 + Python 3.10):

```bash
# Ubuntu 22.04 ships python3 but not pip; install it first.
sudo apt install -y python3-pip
pip install dufomap
python3 -c 'import dufomap; print(dufomap)'   # sanity check
```

Verified on Ubuntu 22.04.5 LTS + Python 3.10.x. The DUFOMap wheel
bundles the UFO library; no extra native install is needed.

### Run

Produce a static PCD in one command from a GLIM output directory
(e.g. `docs/m5r-bench-data/<run>/glim-out/`):

```bash
scripts/m5r_run_dufomap.sh <glim-out-dir> <output-dir>
```

Example:

```bash
scripts/m5r_run_dufomap.sh \
  docs/m5r-bench-data/2026-06-21-loop-outdoor/glim-out \
  /tmp/m5r49_dufomap
# -> /tmp/m5r49_dufomap/static.pcd
```

Running the stages individually:

```bash
# 1. GLIM keyframes -> per-keyframe PCDs (VIEWPOINT header embedded)
scripts/m5r_glim_to_pcd.py \
  --glim-out <glim-out-dir> \
  --out-dir <staging-dir>

# 2. DUFOMap proper
scripts/m5r_run_dufomap_core.py \
  --data-dir <staging-dir> \
  --output <static.pcd>
```

Idempotency: every script aborts by default on existing output;
pass `--force` to overwrite (same convention as
`scripts/m5r3_run_glim.sh`).

### Parameters

Defaults come from upstream `KTH-RPL/dufomap/assets/config.toml`.
Tuning guidance is in the rightmost column. When changing, record
the value in `docs/maps/<site>/metadata.yaml`'s `dufomap_params`
field (field name finalised in M5R-7 / #51).

| Parameter | CLI flag | Default | Description | Tuning guidance |
|---|---|---|---|---|
| resolution | `--resolution` | 0.1 m | voxel size | 0.05 for dense indoor maps; 0.2 for wide outdoor. 0.1 is fine for VLP-16 outdoor loops |
| inflate_hits_dist (d_s) | `--d-s` | 0.2 m | inflation around hits | 0.3–0.5 for noisier sensors. VLP-16 range accuracy (~3 cm at 100 m) is fine at 0.2 |
| inflate_unknown (d_p) | `--d-p` | 2 voxels | unknown-region inflation | DUFOMap paper recommendation; rarely needs tuning |
| min_range | (not exposed) | 0.2 m | ego exclusion | Fixed in DUFOMap config.toml; only revisit if the ego bbox is large |
| max_range | (not exposed) | -1 (unbounded) | far-cut | Only revisit if outdoor returns past ~50 m are unreliable |
| num_threads | `--num-threads` | 12 | DUFOMap worker threads | Match host physical core count (12–14 for i9-12900H) |

### Input / output

Input: GLIM keyframe dirs (`<glim-out>/NNNNNN/`):

- `points_compact.bin` — Nx3 raw `Eigen::Vector3f` dump in the
  keyframe-local frame.
- `data.txt` — text dump whose first labelled block is
  `T_world_origin:` followed by a 4x4 row-major float matrix giving
  the keyframe origin's pose in the world frame.

`m5r_glim_to_pcd.py` converts the pair into per-keyframe PCDs. The
output is ASCII (the volumes are small enough that binary's space
gain does not justify losing readability), and the `VIEWPOINT`
header carries the `T_world_origin` translation + quaternion in
PCL's order (`qw qx qy qz`, which is the opposite of scipy's
`(x, y, z, w)`). The points stay in the keyframe-local frame —
DUFOMap applies the supplied pose internally with
`cloud_transform=False`.

Output: a single static PCD (`<output-dir>/static.pcd`). This feeds
M5R-6 (#50) occupancy-grid conversion and, later, the M6-R
scan-to-map localizer (ADR-0005 convention).

### Visual verification

Overlay before / after clouds with `scripts/m5r_dufomap_diff.py` and
confirm that pedestrian trails are gone:

```bash
# interactive viewer
scripts/m5r_dufomap_diff.py --before <raw>.pcd --after <static>.pcd

# screenshot (for CI / documentation)
scripts/m5r_dufomap_diff.py --before <raw>.pcd --after <static>.pcd \
  --screenshot diff.png
```

- before = red, after = blue.
- A bag containing dynamic objects (pedestrians, etc.) is required
  to produce evidence — a separate recording with the M4-R bringup
  launch active is needed (M5R-4 acceptance B2).
- This script needs `open3d` (`pip install open3d`). The wheel is
  large (~100 MB), so we do not pull it in CI; this is an
  interactive-only tool.

### Known concerns

- DUFOMap's `outputMap` output filename varies across versions
  (`dufomap_output.pcd` vs `dufomap_output_voxel_map.pcd`).
  `m5r_run_dufomap_core.py` probes both and moves whichever it
  finds to the user-specified `--output`. We deliberately do not
  patch upstream sources to keep the LICENSE story clean.
- `min_range` and `max_range` are not exposed in DUFOMap's Python
  API. If you need to change them, edit upstream
  `assets/config.toml` and `pip install --force-reinstall .`; not
  supported by this pipeline today.

## Occupancy-grid conversion (Nav2-compatible)

Turn DUFOMap's `static.pcd` into the `occupancy.pgm` + `occupancy.yaml`
pair that Nav2 `map_server` reads directly.
`scripts/m5r_pcd_to_occupancy.py` (Issue #50 / M5R-6) drives this stage.

The legacy `scripts/pcd_to_occupancy_grid.py` (M5-b era) is kept as-is
because `docs/maps/lab-legacy-m5b/` still feeds `nav_launch.py`. The
new script is a sibling, not a refactor; see its module docstring for
the rationale. Highlights:

- The XY range is computed from the input PCD's bbox (the legacy
  script hard-codes ±20 m).
- The ray-cast anchor defaults to the occupied-cell centroid (legacy:
  world (0, 0)).
- Output YAML matches `docs/maps/_template/occupancy.yaml` exactly
  (`free_thresh: 0.196`, the Nav2 default).
- The legacy outlier filter and clear-radius hacks are dropped —
  DUFOMap replaces the former and the chair-starts-at-origin premise
  is gone.

### Usage

```bash
scripts/m5r_pcd_to_occupancy.py <input.pcd> <output-dir> [options]
```

Example (converting the static.pcd from #49):

```bash
scripts/m5r_pcd_to_occupancy.py \
  /tmp/m5r49_dufomap/static.pcd \
  docs/maps/lab-loop \
  --force
# -> docs/maps/lab-loop/occupancy.pgm
# -> docs/maps/lab-loop/occupancy.yaml
```

Idempotency: existing `occupancy.pgm` / `occupancy.yaml` aborts the
run unless `--force` is passed (same convention as the rest of the
M5-R script suite).

### Parameters

| Parameter | CLI flag | Default | Description |
|---|---|---|---|
| resolution | `--resolution` | 0.05 m | Cell size. Matches `_template/occupancy.yaml`. |
| Z slice lower | `--z-min` | 0.1 m | Drop floor noise and shallow slopes. |
| Z slice upper | `--z-max` | 1.5 m | Drop ceilings / lintels / signage above chair height. |
| Ray-cast anchor | `--anchor-x`, `--anchor-y` | auto | Defaults to the occupied centroid; specify explicitly for U-shaped routes. |
| Disable ray-cast | `--no-raycast` | off | Stamp occupied only, leave everything else unknown. Debug aid. |
| occupied_thresh | `--occupied-thresh` | 0.65 | YAML output value. |
| free_thresh | `--free-thresh` | 0.196 | YAML output value (Nav2 default). |
| Padding | `--padding` | 2.0 m | Extra metres outside the bbox so edge obstacles aren't clipped. |
| Force overwrite | `--force` | off | Permit overwriting existing pgm/yaml. |

### Output

- `occupancy.pgm`: P5 binary, 1 byte/cell, header
  `P5\n# m5r_pcd_to_occupancy.py output\n<W> <H>\n255\n` followed by
  the pixel payload.
- `occupancy.yaml`: same field shape as
  `docs/maps/_template/occupancy.yaml` (`image` / `resolution` /
  `origin` / `negate` / `occupied_thresh` / `free_thresh`), with a
  one-line generator comment that records the source PCD name.

Pixel-value convention (ROS `map_server`):

| Value | Meaning |
|---|---|
| 0 | OCCUPIED (black) |
| 254 | FREE |
| 205 | UNKNOWN |

### Known concerns

- The auto anchor uses the occupied-point centroid; on U- or
  L-shaped routes the centroid can land outside the navigable area
  (e.g. inside a wall). Override with `--anchor-x` / `--anchor-y`.
- Very long, narrow bags blow the grid size up. A safety cap aborts
  if the grid exceeds 100 M cells; lower the resolution or pre-crop
  the PCD if you hit it.
- Ray-casting from a single anchor is an approximation; per-scan
  ray-cast (octomap / UFO style) is not implemented. This is enough
  for an offline pipeline that consumes an already-static cloud. If
  per-scan is ever needed, the keyframe poses must be carried into
  this stage — that is a separate Issue.

## Site registration (step 5): placement under `docs/maps/<site>/`

After step 4 you have `dufomap-out/static.pcd` + `occupancy.pgm` +
`occupancy.yaml`. Promote these into the registry at
`docs/maps/<site>/`:

### Procedure

```bash
# 1. clone the template into the new site
cp -r docs/maps/_template docs/maps/<site>
# e.g. docs/maps/lab-loop, docs/maps/utsunomiya-yoto-east

# 2. drop the artifacts in (PCD gitignored; pgm/yaml/metadata tracked)
cp docs/m5r-bench-data/<run-id>/dufomap-out/static.pcd docs/maps/<site>/
mv path/to/occupancy.pgm  docs/maps/<site>/
mv path/to/occupancy.yaml docs/maps/<site>/

# 3. Or: just point m5r_pcd_to_occupancy.py's <output-dir> directly at
#    docs/maps/<site>/ so step 2's mv is unnecessary.
scripts/m5r_pcd_to_occupancy.py \
  docs/maps/<site>/static.pcd \
  docs/maps/<site>/ \
  --force

# 4. fill in metadata.yaml (next section)
${EDITOR:-vi} docs/maps/<site>/metadata.yaml
```

### Filling out `metadata.yaml`

Schema is owned by [`../../maps/README.md`](../../maps/README.md)
§"`metadata.yaml` schema" (ADR-0005,
[`decisions/0005-maps-spec.md`](decisions/0005-maps-spec.md)). M5-R-
derived values:

| Field | Where it comes from |
|---|---|
| `acquired_at` | `ros2 bag info <bag>` Start, in ISO8601 + timezone |
| `route_summary` | One-line route description (e.g. `"lab 50m loop, start/end at NE corner of room 2F"`) |
| `weather` | `"indoor"` for indoor, weather + temperature for outdoor |
| `slam_method` | `glim` (ADR-0003) |
| `slam_params` | Omit if default config; otherwise repo-relative path to `docs/m5r-bench-data/<run-id>/glim-out/config/` |
| `erasor_params` | DUFOMap parameters inline (only non-default `resolution`, `d_s`, `d_p`) |
| `source_bag` | repo-relative path to `docs/m5r-bench-data/<run-id>/bag/` |
| `commit` | `git rev-parse HEAD` (40-char SHA) |

The key name `erasor_params` is kept from ADR-0005 for historical
reasons; the actual values are DUFOMap's (ADR-0004).

### Post-placement check

```bash
# Optional: confirm Nav2 map_server reads it
ros2 run nav2_map_server map_server \
  --ros-args -p yaml_filename:=docs/maps/<site>/occupancy.yaml
# Second terminal:
ros2 lifecycle set /map_server configure
ros2 lifecycle set /map_server activate
ros2 topic echo /map --once --field info
# resolution / width / height / origin should match the yaml
```

Visual confirmation in RViz follows the [occupancy-grid section](#occupancy-grid-conversion-nav2-compatible)
(Fixed Frame `map`, Map display Durability Policy = Transient Local).

## Handoff to M6-R (B6)

The final M5-R artifact set is the M6-R input contract:

```
docs/maps/<site>/
├── static.pcd        ← M6-R scan-to-map localizer input (NDT / MCL map)
├── occupancy.pgm     ← Nav2 map_server / obstacle layer input
├── occupancy.yaml    ← same (resolution / origin / threshold spec)
├── metadata.yaml     ← acquisition date / route / SLAM params audit trail
└── README.md         (optional)
```

What M6-R can assume:
- `static.pcd` is in the world frame (GLIM's map frame) and is
  dynamic-object-removed (pedestrians and cyclists are gone).
- `occupancy.yaml`'s `origin` is the world-frame lower-left corner of
  the map, in the same coordinate system as `static.pcd`.
- The origin of `static.pcd` is whatever GLIM picked as its map
  origin (not the chair's start pose); `metadata.yaml.route_summary`
  describes the spatial relationship.

What M6-R cannot assume yet (out of scope for M5-R; tracked as
follow-up Issues if needed):
- Accurate occlusion via per-scan ray-casting. The current pipeline
  uses a single-anchor approximation with the documented "starburst"
  artifact (see `scripts/m5r_pcd_to_occupancy.py` docstring).
- Outdoor continuity. M5-R verification covers indoor + one campus
  site; full-campus mapping is to be re-recorded when M6-R starts.

## ADR-0003 status

ADR-0003 (SLAM choice) was created in #48 and promoted to
**accepted** after Phase A / B measurement
([`decisions/0003-mapping-slam-choice.md`](decisions/0003-mapping-slam-choice.md)).
At the start of this Issue the Status is already `accepted`, so no
extra approval step is needed. Future SLAM re-selection should be a
new ADR (e.g. ADR-00NN: SLAM re-selection) rather than editing this
one in place — per ADR convention, supersede via a new doc, do not
mutate.
