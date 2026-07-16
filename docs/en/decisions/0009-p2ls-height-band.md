# ADR 0009: pointcloud_to_laserscan height band + QoS bridge (M6-R)

Language: [日本語](../../ja/decisions/0009-p2ls-height-band.md) | [English](0009-p2ls-height-band.md)

- Status: **accepted** (2026-07-15 field A/B confirmed min_height = 0.05; AC1-AC5 all PASS)
- Date: 2026-07-14 (drafted) / 2026-07-15 (accepted)
- Deciders: Iruazu

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
| `min_height` | **0.05 m** (2026-07-15 A/B) | With Patchwork++ (ADR-0011) removing the ground upstream, a parallel 0.05 vs 0.10 field A/B showed zero false lethals on slopes / raised manholes at either value. 0.05 keeps ~4 sustained returns on a person's legs (2 m ahead); 0.10 sheds those returns without any safety-margin gain (see §Verification 2026-07-15 A/B) |
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

### Alternative A: lower `min_height` to capture ~5 cm curbs

- **Rejected (pre-Patchwork++)**: 2026-07-14 Phase B measured -0.2
  producing false positives on undulating ground. As long as the slice
  is a single-threshold cut of a terrain-flat world, `min_height` is a
  trade-off, not a fix
- **Path forward**: ADR-0011 (Patchwork++ ground removal) removes the
  ground upstream, which lets this ADR's `min_height` relax back down —
  confirmed in the 2026-07-15 A/B
- **Post-Patchwork++ limit (2026-07-15 A/B)**: the real campus-loop
  curbs measured ~5 cm, not the assumed 12-15 cm. That is below the
  0.05 cut line, and lowering further collapses onto the same
  ground-vs-obstacle ambiguity — Patchwork++ classifies a 5 cm rise as
  "ground". This step class (~5 cm) is intentionally out of scope for
  this layer; the chair traverses them, and routing side (map
  annotation / operator judgement) covers avoidance where needed

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

- **~5 cm steps (including real campus curbs) are out of scope for
  this layer**. Ground removal (ADR-0011) plus a 0.05 m slice cannot
  separate them from ground undulation by construction; routing side
  handles those
  - **2026-07-16 field addendum**: on the demo route, a few centimetre
    steps (tile seams, curb bottoms) sit on the path free in occupancy
    and the chair drives into them. The ADR-time "routing side handles
    it" assumption has to be turned into an **explicit route
    constraint** — passive avoidance by physical layout is not enough
    on this course
  - **Short-term mitigation (P0, demo-critical)**: paint no-go bands
    into `docs/maps/campus/occupancy_cleaned.pgm` with GIMP so those
    step locations read as occupied. Route walkthrough and paint
    procedure captured in the [demo prep checklist](../m6r-demo-prep-checklist.md)
  - **Medium-term mitigation (post-demo, under evaluation)**: adopt
    Nav2 Keepout Filter so route constraints are declared as a filter
    layer instead of baked into the pgm — allowing constraint updates
    without rerunning the map generation pipeline
- **People (standing / walking)**: 0.05 m keeps ~4 sustained returns
  on the legs and paints them lethal on `local_costmap` — enough
  signal for the intended costmap use (2026-07-15 A/B measurement)
- **Tall weeds** paint lethal at both values. Not resolvable in code;
  pre-demo route grooming (mow / route around) is captured in the
  [demo prep checklist](../m6r-demo-prep-checklist.md)
- **Operator-in-the-loop** stays required: paired with ADR-0007
  §Demo-scope reduction, the demo needs an operator with joystick
  override available
- **Single publisher on `/scan`** is committed here
  (velodyne_laserscan_node → `/scan_raw`). M6R4-b uses the same route,
  so the contract "`/scan` = p2ls output" stays firm

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

**U3-U6's suspension was a limit of this ADR's interim value, not a
bridge-integration failure.** With ADR-0011 accepted (2026-07-14) and
M6R4-c flipping p2ls's input to Patchwork++'s non-ground output, the
2026-07-15 A/B closes both `min_height` and U3-U6 (see below).

## Verification (2026-07-15 A/B, outdoor)

Two p2ls instances running side-by-side on Patchwork++'s
`/velodyne_points_no_ground`, one at `min_height = 0.05`, one at
`0.10`. Observed simultaneously against the same scene:

| Metric | 0.05 | 0.10 | Rationale |
|--------|------|------|-----------|
| False lethals on slopes + raised manholes | **0** | **0** | Phase B false obstacles gone. Patchwork++ ground removal absorbs the lower-bound margin at either value |
| Sustained returns on legs (person, 2 m ahead) | **~4** | fewer | Enough signal for costmap use. 0.10 drops returns without any safety-margin gain |
| ~5 cm curb detection | not detected | not detected | By design (see §Alternative A). Below the 0.05 cut, Patchwork++ classifies as "ground" |
| Tall weeds | lethal | lethal | Not resolvable in code; route-grooming issue |

**Decision**: adopt `min_height = 0.05`.

## Acceptance conditions — all PASS

- **AC1** (build & QoS): `colcon build --packages-select whill_navigation
  whill_sensors_bringup` PASS. `ros2 topic info /scan` publisher
  count = 1 (SetRemap effective). `ros2 topic hz /scan` holds
  9-11 Hz for 30 s (PASS)
- **AC2** (Phase A/T1-T5): 2026-07-14 measurement PASS
- **AC3** (Phase B/T3-T4): 2026-07-14 measurement PASS
- **AC4** (U3-U6, person-standing obstacle): 2026-07-15 A/B PASS. Legs
  visible on local_costmap and clearing tracks the person's departure
- **AC5** (min_height re-tune): 2026-07-15 A/B PASS. 0.05 drops the
  Phase B 2026-07-14 false-lethal spike count to 0 while keeping the
  person-obstacle signal

## Related

- [`../../ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../../ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md)
  §3 M6R4-2 (primary p2ls parameter record)
- ADR-0011 (accepted 2026-07-14): ground removal preprocessing;
  precondition for this ADR's `min_height` re-tune
- Upstream `ros-humble-pointcloud-to-laserscan` (BSD-3-Clause, apt)
- 2026-07-14 measurements: `docs/m6r-bench-data/2026-07-14-verify-campus/`
