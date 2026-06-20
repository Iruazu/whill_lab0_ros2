# M4-R bench data

Captured artifacts from the M4-R odom-layer verification runs (the
`odom -> base_link` EKF wired up by Issues #35 / #36 / #37 / #38).
Same gitignore policy as `docs/m3-bench-data/`: bag directories
themselves are too large to keep in git, only the descriptive files
(per-run READMEs, `frames.pdf` snapshots, analysis PDFs) are tracked.

The `.gitignore` rule is:

```
docs/m4r-bench-data/*/
!docs/m4r-bench-data/*.md
!docs/m4r-bench-data/*.pdf
```

i.e. every subdirectory is ignored, top-level Markdown / PDF is kept.

## Directory layout convention

```
docs/m4r-bench-data/
├── README.md                           (this file — protocol + index)
├── frames-m4r4-<date>.pdf               (optional: view_frames snapshot)
├── <YYYY-MM-DD>-<run-id>/               (one directory per run, gitignored)
│   ├── README.md                        (run metadata — committed by
│   │                                     copying out of the dir, see below)
│   ├── bag/                             (rosbag2 directory)
│   └── analysis.csv | analysis.pdf      (optional outputs of bench scripts)
└── <YYYY-MM-DD>-<run-id>.md             (the per-run README, lifted to the
                                          top level so git tracks it; the
                                          original inside the run dir is a
                                          working copy)
```

Why the per-run README lives twice: the directory itself is gitignored
(to drop the bag), so any text file inside the directory is also
ignored. Convention is to keep the human-readable summary alongside
the bag for self-contained archives, then copy / symlink it out to
`docs/m4r-bench-data/<YYYY-MM-DD>-<run-id>.md` so git tracks it.

## Per-run README template

Each run's README captures the conditions and the bench-script output
in enough detail that the same protocol can be re-run later. Minimum
fields:

```markdown
# <YYYY-MM-DD> — <run-id>

## Environment
- Host: <hostname>, Ubuntu 22.04, ROS 2 humble
- Chair: WHILL Model CR2 (serial / firmware if relevant)
- Floor: <surface — vinyl, carpet, painted concrete; affects wheel slip>
- Operator: <name>
- Drivers used: M4-R unified bringup (`odom_bringup_launch.py`)

## Protocol
- Test: <10 m straight push | 30 s static | other>
- Initial pose: <where the chair started on the marked line>

## Bag
- Path: `docs/m4r-bench-data/<dir>/bag/`
- Duration: <s>
- Topics: see "Topics recorded" below.

## Results
- `scripts/m4r3_ekf_bench.py <bag>` output:
  - End distance from start: <m> (acceptance: ≤ 0.5 m for 10 m push)
  - Yaw drift: <rad> (acceptance: ≤ 0.1 rad for 30 s static)

## Observations
- <Anything unusual — wheel slip, IMU bias jump, EKF re-init, etc.>
```

## Topics recorded

The bench scripts (`scripts/m4r3_ekf_bench.py`, etc.) assume the bag
contains the following six streams. Recording more is fine, recording
less will break analysis.

| Topic | Type | Why it is in the bag |
|-------|------|----------------------|
| `/whill/states/model_cr2` | `whill_msgs/ModelCr2State` | Raw chair state, traceable back to firmware ticks. Lets us re-derive `/whill/odom` if the driver's integration logic changes. |
| `/whill/odom` | `nav_msgs/Odometry` | M4R-1 wheel odometry — one of the two EKF inputs. |
| `/imu/data_raw` | `sensor_msgs/Imu` | RT 9-axis IMU at 100 Hz — the other EKF input. `orientation_covariance[0] == -1` (REP-145 "unknown") so orientation is unfused. |
| `/odometry/filtered` | `nav_msgs/Odometry` | M4R-3 EKF output — the trajectory that the bench script analyses. |
| `/tf` | `tf2_msgs/TFMessage` | Captures the live `odom -> base_link` edge for cross-checking against `/odometry/filtered`. |
| `/tf_static` | `tf2_msgs/TFMessage` | M4R-2 `base_link -> {imu_link, velodyne, camera_link}`. Latched, so the bag must record it to be replayable later. |

## Protocol — 10 m straight push (Issue #37 AC3)

This is the canonical M4-R EKF verification run. Acceptance: end
distance from start ≤ 0.5 m (5 % of path).

1. Lay a 10 m measuring tape on the floor. Mark start and end with
   tape; mark a separate "stopping" line ~30 cm past the 10 m mark so
   the operator does not have to brake on the measurement point.
2. Position the chair so the rear axle midpoint (the M4R-2 `base_link`
   origin) sits over the start mark, with the chair facing along the
   tape. Operator on the chair, joystick neutral.
3. In one terminal:
   ```bash
   ros2 launch whill_localization odom_bringup_launch.py
   ```
   Wait for the IMU lifecycle activation and the first
   `/odometry/filtered` message before continuing (`ros2 topic hz
   /odometry/filtered` should report ~30 Hz).
4. In a second terminal, pick a run ID and start recording:
   ```bash
   RUN_ID=2026-06-22-push10m-r1
   mkdir -p docs/m4r-bench-data/$RUN_ID
   ros2 bag record -o docs/m4r-bench-data/$RUN_ID/bag \
       /whill/states/model_cr2 /whill/odom /imu/data_raw \
       /odometry/filtered /tf /tf_static
   ```
5. Operator pushes (do not joystick — wheel slip from torqued motors
   contaminates the test) the chair along the tape to the stopping
   line, then holds the chair still for 2 s. `Ctrl-C` the bag
   recorder, then `Ctrl-C` the launch.
6. Analyse:
   ```bash
   python3 scripts/m4r3_ekf_bench.py docs/m4r-bench-data/$RUN_ID/bag
   ```
   Note the `End distance from start` and `Yaw drift` numbers.
7. Write the per-run README into the directory; copy / symlink to
   `docs/m4r-bench-data/$RUN_ID.md` so it survives the `.gitignore`.

## Protocol — 30 s static (Issue #37 AC4)

Acceptance: yaw drift ≤ 0.1 rad over 30 s of stillness.

1. Park the chair somewhere it will not be jostled (no foot traffic,
   no nearby HVAC vents). Operator seated, hands off the joystick.
2. Launch + record (same commands as steps 3–4 above, just replace
   the run ID).
3. Hold still for 35 s (the extra 5 s is leeway around start/stop
   transients that the bench script trims).
4. Stop bag, stop launch, run `m4r3_ekf_bench.py` as above.

## Run index

(Populated as runs land. Date / run-id / protocol / outcome.)

| Date | Run ID | Protocol | End dist | Yaw drift | Verdict |
|------|--------|----------|---------:|----------:|---------|
| —    | —      | —        |        — |         — | —       |
