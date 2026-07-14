# ADR 0010: Keep Nav2 planner + controller and set `allow_unknown: false` (M6-R)

Language: [日本語](../../ja/decisions/0010-nav2-planner-controller.md) | [English](0010-nav2-planner-controller.md)

- Status: **proposed** (drafted at M6R4-1+2, promoted to accepted after
  M6R4-3 V4 field run)
- Date: 2026-07-14
- Deciders: Iruazu (pending approval)

## Context

Nav2's global planner and local controller were picked in M5-c as a
"conservative first cut" (NavfnPlanner + RegulatedPurePursuit). The
comment at the time said "revisit in M5-e", but M5-e was frozen by
[`../plans/2026-06-11-platform-pivot.md`](../../ja/plans/2026-06-11-platform-pivot.md) §5.
M6-R needs to re-confirm the selection and record the `allow_unknown`
decision specific to the M5-R `campus` map.

The `campus` map is the M5-R production 2D occupancy grid of a 1310 m
outer-loop drive. Roughly 200 m × 200 m in the middle is unknown because
that area was never driven ([`../../maps/campus/README.md`](../../maps/campus/README.md)
§2). This shape is consistent with the demo operation (outer loop only),
but with `planner_server.allow_unknown` at its default (true) the
planner will happily cut through unknown cells to shorten a route —
producing paths that plunge through the middle instead of going around.

## Decision

Committed in `nav2_params.yaml` at M6R4-1:

### 1. Global planner: `nav2_navfn_planner/NavfnPlanner`

- Grid Dijkstra (Nav2 standard). `use_astar: false` keeps Dijkstra
- `tolerance: 0.5` (reject if no path reaches within 0.5 m of goal)
- `allow_unknown: false` (see §3)

### 2. Local controller: `nav2_regulated_pure_pursuit_controller::RegulatedPurePursuitController`

- Pure Pursuit derivative that regulates linear velocity by curvature.
  A natural fit for the differential-drive WHILL CR2
- Acceleration is not enforced by the controller itself
  (`max_linear_accel` is documentation only); the actual ramp lives in
  the downstream `velocity_smoother`
- Key parameters:
  - `desired_linear_vel: 0.3` (m/s)
  - `lookahead_dist: 0.6` / `min_lookahead_dist: 0.3` / `max_lookahead_dist: 0.9`
  - `use_velocity_scaled_lookahead_dist: true`
  - `transform_tolerance: 0.3` (tightened from the M5-c value of 0.5,
    which was sized for a live FAST-LIO at ~7 Hz)

### 3. `planner_server.allow_unknown: false`

- Flipped from the default true to **false**
- Rationale: keep the outer-loop-only demo route on-rails at planner
  level by refusing to route through the unknown centre of the `campus`
  map
- Downside: if a goal is set where the shortest path would cross
  unknown cells, the planner returns no path and `NavigateToPose`
  fails. The demo route is outer-loop only so this downside should not
  materialise; if it does, M6R4-3 V1 will observe it

## Alternatives

### Alternative A: `nav2_smac_planner/SmacPlanner2D` (A\*)

- Better path quality (footprint-aware, allows diagonal motion)
- At 0.3 m/s the qualitative difference over a 10 m path is not
  perceivable; the `campus` map is 6640×6295 cells and SmacPlanner2D
  probably has a higher init cost than NavfnPlanner (not measured)
- Rejected: NavfnPlanner is enough to meet demo path-quality
  requirements. Do not grow the parameter surface for an unmeasured
  performance delta. Re-evaluate when `campus-v2` (interior-filled map)
  lands

### Alternative B: `nav2_dwb_controller/DWBLocalPlanner`

- Robust in dynamic environments (trajectory-rollout scoring)
- Many more parameters than RPP and hard to tune outdoors
- RPP failure modes are simple ("carrot didn't reach") and easy to
  debug; DWB failure modes ("scoring oscillates and the chair
  vibrates") are harder to diagnose
- Rejected: not enough dynamic obstacles in the demo to justify DWB's
  cost. Random human crossings are single-obstacle, not weaving-through
  scenarios. Defer to M7 (`whill_dispatch`) or later

### Alternative C: `nav2_mppi_controller/MPPI Controller`

- Higher-performance but compute-hungry. The Alienware x15 R2 CPU
  budget is already committed to localizer + Nav2 lifecycle nodes
- Rejected: unnecessary at demo scope

### Alternative D: `allow_unknown: true` (the incumbent frozen value)

- Treats unknown as traversable. Made sense in M5-b when the lab.pgm
  unknown region was "not observed but physically passable"
- On `campus` the central unknown is "not observed and physically of
  unknown passability" (fences, buildings). A planned path through it
  will likely be blocked at execution time
- Rejected: demo route is outer-loop only, no route through the middle
  is ever needed

## Consequences

- Max speed stays at 0.3 m/s. No time to retune before the demo anyway
- If `campus-v2` (interior-filled map) lands, revisit `allow_unknown`
- DWB / MPPI migration and the SmacPlanner2D question are deferred to
  M7 (`whill_dispatch`) or later
- This ADR is promoted to accepted after M6R4-3 V4 (30-minute continuous
  run) passes. If V4 shows path runaway (the downside in §3), the ADR
  stays proposed and a revert to `allow_unknown: true` is considered

## Related

- [`../../ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md`](../../ja/plans/2026-07-14-m6r4-nav2-obstacle-layer.md) §3 M6R4-1 (primary params diff record)
- [`../../ja/plans/2026-06-11-platform-pivot.md`](../../ja/plans/2026-06-11-platform-pivot.md) §3.3 (parent-plan Nav2 rationale)
- [`../../maps/campus/README.md`](../../maps/campus/README.md) §2 (origin of the central unknown)
- ADR-0008 (proposed, upcoming): Nav2 costmap layout (static + obstacle + inflation)
- ADR-0009 (proposed, upcoming): pointcloud_to_laserscan parameter selection
