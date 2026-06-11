# whill_lab0_ros2

ROS 2 humble port of [whill_lab0](https://github.com/Iruazu/whill_lab0) (originally ROS noetic).

The goal of this repository is to migrate the WHILL mobility robot stack — driver, sensors,
localization, and navigation — from ROS noetic to ROS 2 humble, and to validate the result
on the actual WHILL hardware running on the Utsunomiya University campus.

## Status

Completed milestones (initial roadmap M1–M5):

| Milestone | Title | Status |
|-----------|-------|--------|
| M1 | ROS 2 humble environment setup on host | done |
| M2 | WHILL core driver on real hardware (Model CR2 / USB) | done |
| M3 | Sensor stack (Velodyne / RealSense / IMU) | done (PR #4, #5 merged) |
| M4 | Localization baseline (FAST-LIO) | done (PR #6 merged) |
| M5-a/b/c/d | Nav2 bringup + first autonomous goal on the chair (2026-05-20) | done (PR #7 merged) |
| M5-d (continued) / M5-e | Long-distance goals, dynamic obstacles, tuning | frozen by the 2026-06-11 platform-pivot decision |

Active roadmap (post-pivot; see [`docs/plans/2026-06-11-platform-pivot.md`](docs/plans/2026-06-11-platform-pivot.md) §4 for the authoritative table):

| Phase | Title | Status |
|-------|-------|--------|
| M4-R | Odom foundation rebuild: robot_localization EKF (`/whill/odom` + IMU), TF rewire, retire `tf_bridge_launch.py` | not started |
| M5-R | Map pipeline: offline SLAM (GLIM or FAST-LIO SAM) + dynamic-object removal (ERASOR), `docs/maps/<site>/` artifact spec | not started |
| M6-R | Operational localization + Nav2 re-integration: scan-to-map localizer, initial-pose UX, failsafe node, obstacle layer restored | not started |
| M7 | Dispatch API layer (`whill_dispatch`): named waypoints, job queue, `NavigateToPose` wrapper, status publish, rosbridge | not started |
| M8 | Tablet web app: map view, destination selection, vehicle call | not started (may live in a separate repo) |
| M9 | Integrated validation: unmanned-call runs, physical and remote E-stop | not started |

Phase IDs (`M4-R` etc.) and acceptance criteria are defined in the platform-pivot document
§4 and §6. The previous `tf_bridge_launch.py` identity construction and any new feature
that depends on it are prohibited under §5 of that document.

## Development workflow

Day-to-day work is issue-driven. Each Issue is delivered as one branch and one PR; the human
merges. The active policy document is
[`docs/plans/2026-06-11-platform-pivot.md`](docs/plans/2026-06-11-platform-pivot.md).

- 1 Issue = 1 branch = 1 PR. Acceptance criteria are kept to roughly three observable checks
  per Issue so that implementation and review fit in one session.
- Branch name: `<phase>/<issue-number>-<slug>` (e.g. `m4-r/12-add-ekf`). Chores use
  `chore/<issue-number>-<slug>`.
- `main` is protected by required PR review (approvals 0 — the human is reviewer and merger).
- Merging a PR is always a human action. Authoring an ADR (`docs/decisions/NNNN-*.md`) is also
  a human action; agents may draft, but the human writes the `accepted` line.
- Slash commands available to Claude Code sessions:
  - `/issue <phase or topic>` — draft a GitHub Issue at the right granularity, with the
    policy correspondence (M\*/R\*/P\*) and assumptions filled in.
  - `/work <N>` — pick up Issue N, branch, implement via `ros2-implementer`, run
    `code-reviewer`, commit, push, open a draft PR. The human merges.
  - `/status` — issue / PR / branch / phase dashboard, with items waiting on the human at
    the top.

## Layout

```
whill_lab0_ros2/
├── src/         # colcon source space — ROS 2 packages
├── docs/        # migration plan, per-milestone notes
└── scripts/     # one-shot setup / utility scripts
```

## Build

After ROS 2 humble is installed and `source /opt/ros/humble/setup.bash` is in effect:

```bash
cd ~/whill_lab0_ros2
./scripts/install_udev_rules.sh      # /dev/whill, /dev/imu stable symlinks (one-time)
./scripts/import_upstream.sh         # vcs import + rosdep install
colcon build --packages-up-to whill --symlink-install
source install/setup.bash
```

Upstream packages declared in [`whill_lab.repos`](whill_lab.repos) are cloned
into `src/third_party/` (gitignored). Edit that file to pin different versions.

The udev rule (tracked at [`udev/99-whill-stack.rules`](udev/99-whill-stack.rules))
identifies WHILL and the RT 9-axis IMU by their USB VID:PIDs, so they always
appear at `/dev/whill` and `/dev/imu` regardless of which port they are plugged
into.

For the Velodyne VLP-16 (reached over Ethernet, not USB), put the host-side
USB-Ethernet adapter on the LiDAR subnet:

```bash
ip -br link show | grep -E '^(enx|eth|enp)'              # find your iface
./scripts/install_velodyne_network.sh enxAABBCCDDEEFF    # substitute your iface name
```

This renders [`network/01-velodyne-static.yaml.template`](network/01-velodyne-static.yaml.template)
into `/etc/netplan/` and applies it. See [docs/m3-sensors.md](docs/m3-sensors.md)
for the rationale and how to retarget the subnet if your unit was reprogrammed.

## Documentation

Full project documentation lives under [`docs/`](docs/README.md) — milestone
notes, the migration plan, and session logs.

## Reference

- Source repo (noetic): https://github.com/Iruazu/whill_lab0
- Docs index: [docs/README.md](docs/README.md)
- Active policy document: [docs/plans/2026-06-11-platform-pivot.md](docs/plans/2026-06-11-platform-pivot.md)
- Initial migration plan (M1–M3 execution record, superseded for forward planning):
  [docs/migration-plan.md](docs/migration-plan.md)
