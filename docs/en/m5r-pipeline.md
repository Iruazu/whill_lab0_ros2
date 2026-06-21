# M5-R map-building pipeline

Language: [日本語](../ja/m5r-pipeline.md) | [English](m5r-pipeline.md)

This is the reference doc for the M5-R map-building pipeline. The
file was created in Issue #49 as a skeleton covering the
dynamic-removal (DUFOMap) section; Issue #51 (M5R-7, pipeline
integration) will expand it to the full pipeline.

Current coverage: dynamic-removal stage only. Bag capture, SLAM
execution, occupancy-grid conversion, and `docs/maps/<site>/`
placement are added in #51.

## Dynamic-object removal (DUFOMap)

### Why DUFOMap

See [ADR-0004](decisions/0004-dynamic-removal-choice.md). Summary:
ERASOR-family (GPL-3.0 + ROS 1 only) and Removert (no LICENSE file)
were rejected in favour of DUFOMap (BSD-3-Clause, `pip install
dufomap`, ROS-independent).

### Setup

Once on the dev host (Alienware x15 R2, Ubuntu 22.04 + Python 3.10):

```bash
pip install dufomap
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

## Follow-up (to be written in Issue #51)

- Bag capture procedure (what to record with the M4-R bringup
  launch active).
- SLAM run commands (per-bag config patch for the adopted SLAM =
  GLIM).
- Occupancy-grid conversion command (M5R-6 / #50,
  `scripts/m5r_pcd_to_occupancy.py`).
- How to fill in `metadata.yaml` (adopted SLAM / DUFOMap parameters
  / acquisition date / route / weather).
- Final artifact placement (`docs/maps/<site>/...`).
- User-side procedure for promoting ADR-0003 to `accepted`.
