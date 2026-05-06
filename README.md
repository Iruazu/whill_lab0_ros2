# whill_lab0_ros2

ROS 2 humble port of [whill_lab0](https://github.com/Iruazu/whill_lab0) (originally ROS noetic).

The goal of this repository is to migrate the WHILL mobility robot stack — driver, sensors,
localization, and navigation — from ROS noetic to ROS 2 humble, and to validate the result
on the actual WHILL hardware running on the Utsunomiya University campus.

## Status

| Milestone | Title | Status |
|-----------|-------|--------|
| M1 | ROS 2 humble environment setup on host | done |
| M2 | WHILL core driver on real hardware (Model CR2 / USB) | done |
| M3 | Sensor stack (Velodyne / RealSense / IMU) | pending |
| M4 | Localization (FAST-LIO + custom localization) | pending |
| M5 | Navigation (pedestrian-flow navigator etc.) | pending |
| M6 | Bringup integration + on-vehicle validation | pending |

Each milestone is delivered through its own `mN/...` branch and a pull request into `main`.

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
./scripts/import_upstream.sh         # vcs import + rosdep install
colcon build --packages-up-to whill --symlink-install
source install/setup.bash
```

Upstream packages declared in [`whill_lab.repos`](whill_lab.repos) are cloned
into `src/third_party/` (gitignored). Edit that file to pin different versions.

## Documentation

Full project documentation lives under [`docs/`](docs/README.md) — milestone
notes, the migration plan, and session logs.

## Reference

- Source repo (noetic): https://github.com/Iruazu/whill_lab0
- Docs index: [docs/README.md](docs/README.md)
- Migration plan: [docs/migration-plan.md](docs/migration-plan.md)
