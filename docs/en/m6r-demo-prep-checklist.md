# M6-R Demo Prep Checklist

Language: [日本語](../ja/m6r-demo-prep-checklist.md) | [English](m6r-demo-prep-checklist.md)

On-site operational steps to run **before** a demo. Absorbs
route-dependent issues that cannot be handled in code by pre-walking
the course and grooming the physical route.

Re-walk this list whenever the demo shape changes (venue, season,
course). Current scope: M6-R integration demo (campus outer loop).

## Route grooming

### Mow or route around tall weeds

- **Why**: With `min_height = 0.05 m` (ADR-0009 accepted) and
  Patchwork++ ground removal (ADR-0011 accepted), anything rising off
  the ground gets painted lethal. There is no signal separating
  people from weeds
- **Steps**:
  - 1-2 days before the demo, walk the course and mark waist-high
    (~50 cm+) weed patches
  - Options to keep the path clear: (a) mow, (b) map-annotate a
    detour, (c) route the path around within `raytrace_max_range`
- **Record**: log handled patches and the chosen mitigation in
  `docs/m6r-bench-data/<demo-date>-<site>/route-grooming.md`

### ~5 cm road-surface steps

- **Spec behaviour**: out of scope for the obstacle layer (ADR-0009
  §Consequences). The chair traverses these
- **Walk-through item**: visually confirm no new step above the WHILL's
  traversable limit (~10 cm+) has appeared. If it has, map-annotate
  a detour

## Pre-launch checks on demo day

Environment preconditions (RMW, CPU governor, sysctl, NVIDIA suspend
fix) live in [CLAUDE.md §Runtime environment](../../CLAUDE.md). This
section lists only items specific to the M6-R integration demo.

### Bringup — single terminal only

```
ros2 launch whill_safety m6r_bringup_launch.py site:=campus
```

Starts sensor drivers + WHILL driver + M4-R EKF + M6-R localizer +
safety layer. Do NOT launch `sensors_launch.py` or
`odom_bringup_launch.py` in parallel (2026-07-16 field: every node
doubled, `/velodyne_points` at 39.4 Hz, RealSense USB contention loop,
AC4 aborted). See `src/whill_safety/README.md` §Mutual exclusion.

### Verification (~20 s after bringup)

- [ ] **Zero duplicate nodes** (mandatory):
      `ros2 node list | sort | uniq -c | sort -rn | head` — every count
      MUST be 1. A `2 /velodyne_driver_node` line means a duplicate
      bringup is running; kill it before proceeding to AC
- [ ] **/velodyne_points at 10 Hz**: `ros2 topic hz /velodyne_points`
      should sit at 9-11 Hz. Anywhere near 20 or 40 Hz means a doubled
      bringup
- [ ] `map -> odom -> base_link` TF chain is a single chain
      (`ros2 run tf2_tools view_frames`)
- [ ] `/alignment_status.has_converged: true` with `fitness < 1.0`
      at rest after initial pose alignment
- [ ] `/scan` publisher count = 1 (velodyne_laserscan's `/scan_raw`
      remap is active): `ros2 topic info /scan`
- [ ] Operator in-the-loop with joystick override available
      (ADR-0007 §Demo-scope reduction)

### Pre-drive gate — `scripts/m6r_preflight.sh` (blocking, mandatory)

Since the 2026-07-16 late incident (silent QoS mismatch left Layer D
mute, chair contacted a person during blocking-in test), the pre-drive
gate must run **through the blocking script**:

```bash
scripts/m6r_preflight.sh
```

Do **not** issue any goal until the script exits 0. On exit 1, chase
the reported cause (failsafe_node not up / DEAD INPUT / no
`/cmd_vel_safety` publish) before anything else. The script covers:

1. `use_collision_detection: true` effective value
2. `/failsafe_node` alive
3. Dead-input watchdog path: wait 12 s, verify no `DEAD INPUT` line on
   `/rosout` (means every subscription arrived at least once)
4. Live-fire hand test: hold a hand 1.5 m in front of the chair,
   `/cmd_vel_safety >= 15 Hz` publish

### Pre-drive gate: use_collision_detection + Layer D armed (mandatory)

Layer D (forward sector perception gate, ADR-0007 §Layer D proposed)
must be active and `use_collision_detection: true` must actually reach
the controller before any drive:

```bash
# effective value of collision_detection
ros2 param get /controller_server FollowPath.use_collision_detection
# expect: Boolean value is: true

# Layer D armed startup log
ros2 topic echo /rosout | grep -E "failsafe_node ready|forward_blocked"
# expect: "forward_blocked > 5 pts in ±30° @ 0.5-2.0 m, hysteresis 0.5s"

# behavioural test (hold a hand ~1 m in front of the chair for 2 s)
ros2 topic hz /cmd_vel_safety
# expect: 20 Hz while blocked, publishing stops when the hand is removed
```

Do not start the demo if any of the three checks fails (V2 stop
requirement is not being met).

### Map variant selection (Task #13 salt cleanup)

`docs/maps/campus/occupancy.pgm` has baked-in ground-noise salt from
the pre-Patchwork++ M5-R pipeline (verified in the field 2026-07-16).
For demo runs, use the salt-cleaned variant:

```
ros2 launch whill_navigation nav_launch.py site:=campus map_variant:=cleaned
```

On first launch, display `/map` as OccupancyGrid in RViz and verify
the traversed-path salt is gone (compare with `cleaning_diff.png`).

### RealSense (opt-in, normally off)

The D435 is not consumed by the M6-R runtime stack, and USB 2.1
enumeration issues have burned launch cycles when the camera is
plugged in but unused — so `sensors_launch.py`'s `realsense` arg
defaults to false. Pass `realsense:=true` only for camera-specific
tests, and add a USB inspection step to this checklist for those
runs (`lsusb` shows D435, `/dev/bus/usb/` permissions).

## Related

- [ADR-0009: p2ls height band + QoS bridge](decisions/0009-p2ls-height-band.md)
- [ADR-0011: ground removal choice](decisions/0011-ground-removal-choice.md)
- [ADR-0007: failsafe / twist_mux](decisions/0007-failsafe-design.md)
  §Demo-scope reduction
- [`../maps/campus/README.md`](../maps/campus/README.md) §3 (map salt
  origin and mitigation)
