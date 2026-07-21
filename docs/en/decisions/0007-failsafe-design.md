# ADR 0007: Failsafe node + twist_mux design (M6-R)

Language: [日本語](../../ja/decisions/0007-failsafe-design.md) | [English](0007-failsafe-design.md)

- Status: **proposed** (drafted at M6R-3 kickoff, to be accepted on M6R-5 completion)
- Date: 2026-07-14
- Deciders: Iruazu (approval pending)

> **Translation status**: this English version is a stub during the proposed
> phase. The authoritative text is [`../../ja/decisions/0007-failsafe-design.md`](../../ja/decisions/0007-failsafe-design.md).
> Full English translation will be produced when the ADR is promoted to
> `accepted` at M6R-5.

> **2026-07-14 scope-reduction note**: M6R-3 is being implemented as a
> "lite" subset for the 2026-08-01 demo (operator walks beside the chair
> with physical joystick override, so hysteresis / jump detection / G4
> hardware trials are deferred). See §Demo-scope reduction in the
> Japanese version for the full lite-vs-full delta table and the
> restoration path.

## Summary

Adds a `failsafe_node` to the `whill_safety` package (created at M6R-2) plus
a `twist_mux` gate on `/cmd_vel`. The failsafe subscribes to three signals —
manual `/reinitialization_requested`, localizer `/alignment_status` (NDT
divergence), and `/pcl_pose` continuity (silent-stall detection) — and
publishes zero twist on `/cmd_vel_safety` whenever any of them trips. The
mux gives safety priority 100 and navigation priority 10 so a zero-twist
publication wins.

Thresholds derive from the 2026-07-12 M6R-2 live acceptance and the
2026-07-14 verify走行 measurements (fitness range 0.02-0.3, /pcl_pose ~10 Hz):
`FITNESS_MAX=1.0`, `WINDOW_S=2.0`, `PCL_POSE_TIMEOUT_S=1.0`, `SAFE_HOLD_S=3.0`.

See the Japanese version for the full decision, rejected alternatives, and
promotion criteria.

## Layer D — Forward sector perception gate (2026-07-16 addendum, proposed)

**Trigger**: 2026-07-16 field found V2 (stop for person 3-4 m ahead)
failing even with `use_collision_detection: true` + person visible on
`local_costmap`. RPP's collision reach is
`max_allowed_time_to_collision_up_to_carrot × desired_linear_vel =
1.0 × 0.3 = 0.3 m`, evaluated only along the carrot (lookahead 0.8 m).
Once the planner replans around the person the "obstacle on path"
condition no longer holds. The "stop for obstacle → resume on clear"
demo requirement is not implemented in any Nav2 layer.

**Rejected**: extending RPP's own reach (mixes stop/avoid
responsibilities, still coupled to carrot geometry).

**Adopted**: add Layer D to `failsafe_node`. Subscribes to `/scan`
(reliable, ~10 Hz, base_link frame from p2ls), counts points inside a
forward sector (±30°, 0.5-2.0 m). ≥ 5 points → gate `/cmd_vel` via
`/cmd_vel_safety`; continuous clear for 0.5 s releases (same
re-latch pattern as Layer A). Rationale for every value + BT
interaction (progress_checker / recovery) is in the Japanese version.

**V2 / V3 redefinition**: V2 = "person enters 1.5-2 m forward sector
→ `/cmd_vel = 0` within 1 s + `D:forward_blocked` in log"; V3 =
"person exits sector → Layer D released within 1 s + Nav2 resumes";
V6.4 added = (a) sector geometry check at ±30° boundaries, distances
1.5 / 2.0 / 2.5 m, plus (b) **path-side spectator false-trip check**
(open campus demo has onlookers along the route; ±30° @ 2 m has a tip
half-width of `2.0 × tan30° ≈ 1.155 m`, so a spectator 1 m off the
path edge sits inside the sector). Field decision from V6.4 (b) —
(A) operational rule = keep onlookers ≥ 2 m off the path, or (B)
tighten sector to ±25° (half-width @ 2 m ≈ 0.93 m so 1 m off-path
just barely clears). Decision recorded via ADR update after the field
run.

**Operational gate** (mandatory pre-drive check): documented in the
demo prep checklist.

**Promotion criteria**: V2 + V3 + V6.4 PASS on the field, plus V4 30-
min drive with zero false trips against static path-side buildings /
trees, plus `scripts/m6r_preflight.sh` exits 0 during real bringup.

## Incident 2026-07-16 late (silent QoS mismatch)

**What happened**: during a V2-preparation blocking-in test, Layer D
failed to fire and the chair drove into a person (no injury, test
setting).

**Root cause**: `failsafe_node.py:132-133` subscribed to `/scan` with
depth `10` and default reliability = RELIABLE. p2ls publishes /scan
BEST_EFFORT. QoS mismatch: subscription succeeds, zero messages
delivered, `_forward_last_blocked_time` stays `None`, Layer D never
arms, `_active_layers` treats it as clear. The startup log line
`failsafe_node ready: ... forward_blocked > 5 pts ...` reflects that
the subscription was CREATED, not that any message was RECEIVED — an
easy misread as "Layer D is ready". The template for the correct QoS
was seven lines above in the file: Layer C's `_on_perception`
subscription already used `qos_profile_sensor_data`.

**Lessons** (apply to every subscription going forward):

- Sensor-data topics (`/scan`, `/velodyne_points*`, `/camera/*`)
  should default to `qos_profile_sensor_data`. A best-effort
  subscriber is compatible with either publisher policy; ADR docs
  that claim "reliable" cannot save you from what upstream actually
  ships.
- Every first-message-arm layer needs a **dead-input watchdog**.
- The startup "ready" log is a subscription log, not a data-arrival
  log. Operational gates must verify data arrival.

**Fixes shipped in the same commit**:

1. Line 133 flipped to `qos_profile_sensor_data`.
2. `STARTUP_DEAD_INPUT_TIMEOUT_S = 10.0` module constant; `_tick()`
   checks each first-message-arm layer's `_last_*_time` after the
   startup budget and shouts one ERROR line naming the missing
   inputs.
3. `scripts/m6r_preflight.sh` — blocking pre-drive script. Verifies
   `use_collision_detection: true`, `/failsafe_node` alive, no
   `DEAD INPUT` on `/rosout` after 12 s, and a hand-in-front live
   fire test with `/cmd_vel_safety >= 15 Hz`. Exit 1 anywhere = do
   not drive.
4. `FORWARD_SECTOR_MIN_M` raised from 0.5 to 1.0 to match
   Patchwork++'s `min_range: 1.0` filter. The 0.5 draft value was a
   lie: /scan is silent below 1.0 m by construction (Patchwork++
   drops those points as self-return). Post-demo item = revisit
   Patchwork++'s `min_range` down to 0.5 to widen actual coverage
   (the pointcloud_to_laserscan comment already documents 0.5 m as
   the real self-return radius).

## Recalibration 2026-07-19 (field): `FORWARD_POINT_COUNT_MIN` 5 → 3 → 1

The draft value of 5 came from beam-count arithmetic (a person at 2 m
spans ≈ 30 of the 120 beams in the ±30° sector) that field data
contradicted twice. First 5 → 3 (`eae455f`): ADR-0009's A/B run showed
a person's legs return only ~4 points, so 5 never latched on a single
pedestrian and engagement flickered (0-108 msg over a 10 s window).
Then 3 → 1 (`4f90858`): a band_probe capture (149 scans) showed >= 3
in-band points in only 6/149 scans even with a person standing inside
the band — the current /scan (129 bins) is far sparser than the
arithmetic assumed. Current behavior: any single in-band point trips
the gate, released by the existing 0.5 s clear hysteresis. More
spurious stops are accepted as the safe direction for an accompanied
demo. The root cause of the sparse /scan is tracked in #102; revisit
the threshold once resolved.

Full narrative and the Japanese version's post-demo backlog are in
[`../../ja/decisions/0007-failsafe-design.md`](../../ja/decisions/0007-failsafe-design.md).
