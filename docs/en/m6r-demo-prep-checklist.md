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

- [ ] `map -> odom -> base_link` TF chain is a single chain
      (`ros2 run tf2_tools view_frames`)
- [ ] `/alignment_status.has_converged: true` with `fitness < 1.0`
      at rest after initial pose alignment
- [ ] `/scan` publisher count = 1 (velodyne_laserscan's `/scan_raw`
      remap is active)
- [ ] Operator in-the-loop with joystick override available
      (ADR-0007 §Demo-scope reduction)

## Related

- [ADR-0009: p2ls height band + QoS bridge](decisions/0009-p2ls-height-band.md)
- [ADR-0011: ground removal choice](decisions/0011-ground-removal-choice.md)
- [ADR-0007: failsafe / twist_mux](decisions/0007-failsafe-design.md)
  §Demo-scope reduction
