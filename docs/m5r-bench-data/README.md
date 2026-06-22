# M5-R bench data

Interim artifacts from the M5-R map-building pipeline (bag recording →
GLIM SLAM → DUFOMap dynamic removal). The pipeline itself is documented
in [`../ja/m5r-pipeline.md`](../ja/m5r-pipeline.md) (English mirror at
[`../en/m5r-pipeline.md`](../en/m5r-pipeline.md)). This README owns the
**storage convention** for this directory: what lives here, what is
tracked in git, what is not.

The **final-deliverable** maps live under `docs/maps/<site>/` (ADR-0005,
[`../maps/README.md`](../maps/README.md)). The contents of
`docs/m5r-bench-data/` are interim artifacts used to *produce* those
maps; they are regenerable from the bag + the pinned git commit and are
not part of the M6-R input contract.

The gitignore policy mirrors `docs/m4r-bench-data/`: keep small
descriptive files tracked, drop everything heavy and regenerable.

## Directory layout

```
docs/m5r-bench-data/
├── README.md                              (this file — protocol + index)
├── <YYYY-MM-DD>-<run-id>/                 (one directory per recording)
│   ├── bag/                               (rosbag2 dir — IGNORED)
│   ├── bag-imu-fixed/                     (Issue #56 IMU rewrite of pre-#56
│   │                                       bags only — IGNORED)
│   ├── glim-out/                          (GLIM dump — selective track)
│   │   ├── NNNNNN/                        (one keyframe dir)
│   │   │   ├── points_compact.bin         (IGNORED, regenerable)
│   │   │   ├── intensities_compact.bin    (IGNORED, regenerable)
│   │   │   ├── normals_compact.bin        (IGNORED, regenerable)
│   │   │   ├── covs_compact.bin           (IGNORED, regenerable)
│   │   │   ├── data.txt                   (IGNORED, regenerable)
│   │   │   └── imu_rate.txt               (IGNORED, regenerable)
│   │   ├── traj_lidar.txt                 (IGNORED, TUM trajectory)
│   │   ├── traj_imu.txt                   (IGNORED)
│   │   ├── odom_lidar.txt                 (IGNORED)
│   │   ├── odom_imu.txt                   (IGNORED)
│   │   ├── graph.bin                      (IGNORED, large pose graph)
│   │   ├── graph.txt                      (IGNORED)
│   │   ├── values.bin                     (IGNORED)
│   │   ├── manifest.yaml                  (TRACKED, ADR-0003 input)
│   │   ├── run.log                        (TRACKED, per-run summary)
│   │   ├── vram.log                       (IGNORED, 0.5 s VRAM samples)
│   │   ├── slam.log                       (IGNORED, SLAM node stdout)
│   │   ├── rss.log                        (IGNORED)
│   │   ├── gtsam_env.log                  (IGNORED, ldconfig snapshot)
│   │   └── config/                        (TRACKED, GLIM config patch)
│   ├── dufomap-out/                       (DUFOMap dump)
│   │   └── static.pcd                     (IGNORED)
│   ├── fastlio-sam-out/                   (only present for ADR-0003
│   │                                       comparison runs, not for
│   │                                       production maps)
│   │   ├── manifest.yaml                  (TRACKED)
│   │   └── run.log                        (TRACKED)
│   └── README.md                          (TRACKED, per-run notes)
└── <YYYY-MM-DD>-<run-id>.md               (optional: lifted-out copy of
                                            the per-run README so git
                                            tracks it; see m4r-bench
                                            README for the convention)
```

## Naming convention

`<YYYY-MM-DD>-<run-id>` where:

- `<YYYY-MM-DD>` is the **bag recording date** (not the SLAM run date).
  The date the chair was driven; multiple SLAM runs on the same bag
  share this prefix.
- `<run-id>` is a short kebab-case slug describing the route:
  - `loop-indoor`, `loop-outdoor` — research-lab interior / Yoto campus exterior loop
  - `<site>-<variant>` — for production captures, e.g. `lab-loop`,
    `utsunomiya-yoto-east`

If the same route is re-recorded on the same day, append `-r2`, `-r3`
etc. (e.g. `2026-06-22-loop-outdoor-r2`).

## What gets tracked in git

The `.gitignore` rules are file-pattern based (not directory based)
because gitignore cannot re-include a file inside an ignored directory.

| Pattern | Status |
|---|---|
| `bag/`, `bag-imu-fixed/` | IGNORED — rosbag2 directories, hundreds of MB each |
| `*.pcd` | IGNORED — point clouds, tens of MB each |
| `*.bin` | IGNORED — GLIM compact binaries (per-keyframe) and pose-graph dumps |
| `*.txt` | IGNORED — GLIM trajectory / per-keyframe text dumps |
| `*.log` | IGNORED — wrapper-script logs (vram, rss, slam, gtsam_env) |
| `run.log` | TRACKED (explicit allow) — per-run summary the wrapper tees on top of all other logs |
| `manifest.yaml` | TRACKED (explicit allow) — ADR-0003 / ADR-0004 audit input |
| `*.md` | TRACKED (explicit allow) — per-run READMEs and notes |

This pattern keeps git tree size bounded at well under 100 KB per run
(handful of small YAML / Markdown files) while preserving every audit
artifact ADR-0003 and ADR-0004 reference.

## Per-run README template

Drop a `README.md` into each `<YYYY-MM-DD>-<run-id>/` directory.
Minimum fields:

```markdown
# <YYYY-MM-DD> — <run-id>

## Environment
- Host: <hostname>, Ubuntu 22.04, ROS 2 humble
- Chair: WHILL Model CR2
- Sensors: VLP-16, RT 9-axis USB IMU (PCMK-G3X)
- Drivers: M4-R unified bringup (`odom_bringup_launch.py`) with #56
  IMU sign-correction active
- Operator: <name>

## Route
- <one-line description: site / loop length / start point>
- Weather (outdoor): <conditions>

## Bag
- Path: `docs/m5r-bench-data/<YYYY-MM-DD>-<run-id>/bag/`
- Duration: <s>
- Topics: /velodyne_points, /imu/data_rep145, /tf_static

## SLAM
- Command: `scripts/m5r3_run_glim.sh <bag> <out>`
- Loop-closure error: <m> (from `scripts/m5r3_loop_error.py`)
- Wall time: <s> (from manifest.yaml)
- Peak VRAM: <MiB> (from manifest.yaml)

## Dynamic removal
- Command: `scripts/m5r_run_dufomap.sh <glim-out> <out>`
- Static-point ratio: <%>

## Site registration
- Promoted to: `docs/maps/<site>/` (link if applicable)
- Final occupancy.pgm size: <W>×<H>

## Observations
- <anything unusual — drift, GLIM divergence, dynamic-removal artifacts>
```

## See also

- [`../ja/m5r-pipeline.md`](../ja/m5r-pipeline.md) — pipeline procedure
- [`../maps/README.md`](../maps/README.md) — registry convention (ADR-0005)
- [`../ja/decisions/0003-mapping-slam-choice.md`](../ja/decisions/0003-mapping-slam-choice.md) — why GLIM
- [`../ja/decisions/0004-dynamic-removal-choice.md`](../ja/decisions/0004-dynamic-removal-choice.md) — why DUFOMap
- [`../m4r-bench-data/README.md`](../m4r-bench-data/README.md) — sibling convention this README is modelled on
