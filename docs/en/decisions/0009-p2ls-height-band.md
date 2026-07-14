# ADR 0009: pointcloud_to_laserscan height band + QoS bridge (M6-R)

Language: [日本語](../../ja/decisions/0009-p2ls-height-band.md) | [English](0009-p2ls-height-band.md)

- Status: **proposed** (drafted 2026-07-14; promoted to accepted after the M6R4-b (ADR-0011) landing lets `min_height` be re-tuned)
- Date: 2026-07-14
- Deciders: Iruazu (pending approval)

## Context

Nav2's `obstacle_layer` subscribes to `sensor_msgs/LaserScan` at
reliable QoS. VLP-16's `/velodyne_points` is a `sensor_msgs/PointCloud2`
at best-effort SensorData QoS. Direct connection fails on QoS mismatch;
`pointcloud_to_laserscan` (p2ls) has to sit between them. The M5-c
comment "no obstacle_layer" traces to exactly this mismatch.

p2ls slices the 3D cloud with a `base_link`-flat horizontal band
`[min_height, max_height]` and republishes as 2D LaserScan. **This ADR
fixes the range of shapes that slice can handle**, and splits duties
with the ground-removal preprocessor (ADR-0011, Patchwork++).

## Decision

Committed in `src/whill_navigation/config/pointcloud_to_laserscan.yaml`.

### 1. Frame + filter band

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `target_frame` | `base_link` | Matches Nav2 costmap `robot_base_frame` |
| `transform_tolerance` | 0.1 s | Lower bound that admits the M4-R EKF (30 Hz) + M6R-2 localizer (10 Hz) |
| `min_height` | **0.25 m** (interim; re-tuned after M6R4-b lands) | 2026-07-14 Phase B measured -0.2 painting manholes / ruts lethal. 0.25 clears flat sections but leaves spikes on sloped ground (see §Verification below) |
| `max_height` | 1.6 m | Keeps a standing person's torso + head, drops indoor ceiling returns |
| `range_min` | 0.5 m | Removes WHILL chassis self-reflections (within 0.5 m of the LiDAR origin) |
| `range_max` | 25.0 m | Matches `obstacle_layer.raytrace_max_range`. A shorter range here leaves cells uncleared as the chair passes |

### 2. Angular resolution

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| `angle_min` / `angle_max` | -π / +π | Full sweep |
| `angle_increment` | 0.00873 rad (0.5°) | Matches VLP-16 horizontal resolution at 10 Hz; finer bins would split single rings across multiple entries |
| `scan_time` | 0.1 s | Inverse of the 10 Hz output |
| `use_inf` | true | Emit `+inf` for distant clearing so `obstacle_layer` raytraces past them correctly |
| `inf_epsilon` | 1.0 | Nav2 default for handling `+inf` |

### 3. QoS

- p2ls subscribe `/velodyne_points`: best-effort (SensorDataQoS). Even
  when upstream driver publishes reliable, downgrade is accepted
- p2ls publish `/scan`: **reliable** (matches the humble
  `ObstacleLayer` default expectation). No override needed — p2ls's
  own output is reliable by default

### 4. Single-publisher on `/scan`

The upstream VLP-16 launch (`velodyne-all-nodes-VLP16-launch.py`) also
spawns `velodyne_laserscan_node`, which dumps the raw 3D cloud collapsed
to 2D onto `/scan` with **no height filter**. That is not what
`obstacle_layer` should see. Adding p2ls in M6R4-2 without doing anything
else creates a `/scan` dual publisher, and both costmaps subscribe to
the mixed stream (measured 19.7 Hz = 9.86 × 2 in 2026-07-14 Phase B).

`whill_sensors_bringup/launch/sensors_launch.py` uses
`GroupAction + SetRemap('/scan' → '/scan_raw')` to rename the velodyne
output. Only p2ls's `/scan` reaches `obstacle_layer` (`2f26d0b`).

## Alternatives

### Alternative A: lower `min_height` to capture curbs (~0.15 m)

- **Rejected**: 2026-07-14 Phase B measured -0.2 producing false
  positives on undulating ground. As long as the slice is a
  single-threshold cut of a terrain-flat world, `min_height` is a
  trade-off, not a fix
- **Path forward**: ADR-0011 (Patchwork++ ground removal) removes the
  ground upstream, which lets this ADR's `min_height` relax back down

### Alternative B: use `velodyne_laserscan_node`'s `/scan` directly

- **Rejected**: no height filter. Would pick up ceiling and ground
  returns unconditionally and paint massive false-lethal cells
- The upstream `/scan` is renamed to `/scan_raw` for diagnostic use
  (see §4)

### Alternative C: switch to `nav2_costmap_2d::VoxelLayer` (3D)

- **Rejected for now**: swapping obstacle_layer for voxel_layer replaces
  the 2D slice with a 3D grid that could follow terrain. But compute
  cost rises (10 Hz not confirmed on the Alienware x15 R2), and the
  parameter surface expands substantially. Defer to M7 (`whill_dispatch`)
  or later, in its own ADR

### Alternative D: write our own QoS bridge

- **Rejected**: p2ls ships as BSD-3-Clause apt
  (`ros-humble-pointcloud-to-laserscan`). Rewriting is wheel
  reinvention. If a "Patchwork++ output (best-effort) → obstacle_layer
  (reliable)" bridge is needed later, we either use voxel_layer or a
  second p2ls hop — this ADR does not preclude either

## Consequences

- **Curbs (~0.15 m) and crouched children (< 0.25 m) are not captured**
  under this ADR's interim value. After M6R4-b (ADR-0011) lands,
  `min_height` gets relaxed toward ~0.05 m — at which point this ADR
  gets promoted to accepted
- **Operator-in-the-loop** stays required: paired with ADR-0007
  §Demo-scope reduction, the demo needs an operator with joystick
  override available
- **Single publisher on `/scan`** is committed here
  (velodyne_laserscan_node → `/scan_raw`). M6R4-b uses the same route,
  so the contract "`/scan` = p2ls output" stays firm
- **Downstream integration**: alongside ADR-0011's acceptance, a
  follow-up PR flips `pointcloud_to_laserscan_node.cloud_in` from
  `/velodyne_points` to `/velodyne_points_no_ground` — out of scope
  here

## Verification (2026-07-14 Phase B, 工農研横)

Outdoor measurements on `f110f2f` (min_height = 0.25):

| Item | Result |
|------|--------|
| Phase A/T1 | PASS: 6 nodes active (costmap on_activate completed **27 s** after initialpose, measured) |
| Phase A/T3 | PASS: `/alignment_status` fitness 0.0124-0.027, has_converged: true, 200-480× below the 6.0 threshold |
| `/scan` single publisher | PASS: 9.86 Hz (confirms the 2f26d0b SetRemap) |
| 3-layer costmap live | PASS: static + obstacle + inflation displayed in RViz |
| RViz static view | flat sections show clearing donuts correctly |
| U3-U6 (person standing) | **Suspended**: sloped patches still spike at min_height 0.25 and swamp the person-obstacle signal. Resume after ADR-0011's Patchwork++ ground removal lands |

**U3-U6's suspension is a limit of this ADR's interim value, not a
bridge-integration failure.** With ADR-0011 accepted (2026-07-14) and a
follow-up PR flipping p2ls's input to Patchwork++'s non-ground output,
U3-U6 becomes tractable alongside a `min_height` re-tune.

## Acceptance conditions

- **AC1** (build & QoS): `colcon build --packages-select whill_navigation
  whill_sensors_bringup` passes (currently PASS). `ros2 topic info /scan`
  shows publisher count = 1 (SetRemap effective). `ros2 topic hz /scan`
  holds 9-11 Hz for 30 s
- **AC2** (Phase A/T1-T5): 2026-07-14 measurement PASS
- **AC3** (Phase B/T3-T4): 2026-07-14 measurement PASS
- **AC4** (U3-U6, person-standing obstacle): revisit after ADR-0011
  lands and the p2ls remap flip follow-up. RViz must render the
  person's obstacle on local_costmap and clear when they step aside
- **AC5** (min_height re-tune): after AC4 passes, relaxing min_height
  to ~0.05 m must not resurrect the Phase B 2026-07-14 false-lethal
  baseline (judged alongside ADR-0011 AC4)

AC1-AC3 pass; AC4-AC5 are the ADR-0011-coupled follow-up.

## Related

- [`../../ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../../ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md)
  §3 M6R4-2 (primary p2ls parameter record)
- ADR-0011 (accepted 2026-07-14): ground removal preprocessing;
  precondition for this ADR's `min_height` re-tune
- Upstream `ros-humble-pointcloud-to-laserscan` (BSD-3-Clause, apt)
- 2026-07-14 measurements: `docs/m6r-bench-data/2026-07-14-verify-campus/`
