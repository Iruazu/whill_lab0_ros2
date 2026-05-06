# M2 — WHILL core driver on ROS 2 humble

## Goal

Bring up the WHILL Model CR2 on ROS 2 humble using the official driver, with a
real USB connection to the chair. The end state is: the WHILL onboard joystick
drives the chair as normal, and ROS 2 sees state telemetry on
`/whill/states/model_cr2`.

This is the smallest viable real-hardware milestone — no custom logic, just the
upstream driver hooked up to the real CR2.

## Hardware

| | |
|--|--|
| Chair | WHILL Model CR2 |
| Connection | USB serial (chair's USB port → host `/dev/ttyUSB0`) |
| Operator input | WHILL onboard joystick (no external teleop in M2) |

## Upstream packages

This milestone uses two official packages from [whill-labs](https://github.com/whill-labs):

- [`ros2_whill`](https://github.com/whill-labs/ros2_whill) — the driver, bringup,
  description, and examples (5 packages: `whill`, `whill_bringup`,
  `whill_description`, `whill_driver`, `whill_examples`).
- [`ros2_whill_interfaces`](https://github.com/whill-labs/ros2_whill_interfaces)
  — message and service definitions (`whill_msgs` package). Defines
  `ModelCr2State`, `SpeedProfile`, and the `SetPower` / `SetSpeedProfile` /
  `SetBatterySaving` services.

Both are pinned to their `humble` branch in [`whill_lab.repos`](../whill_lab.repos).
The `humble` branch only exposes features common across all CR-series models;
`crystal-devel` adds CR-specific extras and is not used here.

## Procedure

### 1. Import upstream and resolve deps

From this repo's root:

```bash
./scripts/import_upstream.sh
```

This runs `vcs import src < whill_lab.repos` and then `rosdep install` against
the imported tree. The clones land in `src/third_party/` and are excluded from
this repo's git tracking (`.gitignore`). Re-running fast-forwards existing
clones.

### 2. Grant serial port access (one-time)

The driver opens `/dev/ttyUSB0`, which is owned by `root:dialout`. Add the
current user to `dialout`:

```bash
./scripts/grant_serial_access.sh
# log out / log back in, or:
newgrp dialout
```

### 3. Build

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to whill --symlink-install
source install/setup.bash
```

Verified clean build on the lab PC: 6 packages in 13.7 s.

### 4. Connect the WHILL and verify the device node

1. Power the WHILL on.
2. Connect the chair's USB port to the host with a USB-A→USB-mini (or whatever
   the CR2 manual specifies) cable.
3. Confirm the device appears:
   ```bash
   ls /dev/ttyUSB*
   # → /dev/ttyUSB0
   dmesg | tail | grep ttyUSB
   ```
   If the device is something other than `/dev/ttyUSB0`, edit
   `src/third_party/ros2_whill/whill_bringup/config/params.yaml` (or pass
   `port_name:=/dev/ttyUSBN` on the command line — see step 5).

### 5. Launch the driver

```bash
ros2 launch whill_bringup whill_launch.py
```

If the device path is not the default:

```bash
ros2 run whill_driver whill --ros-args -p port_name:=/dev/ttyUSB1
```

In a second terminal verify telemetry:

```bash
ros2 topic list | grep whill
ros2 topic echo /whill/states/model_cr2
```

Move the WHILL with its onboard joystick — `ModelCr2State` should report changing values (battery, motor state, joystick deflection, etc.).

## Topics

The upstream driver on the `humble` branch:

| Direction | Topic | Type |
|-----------|-------|------|
| pub | `/whill/states/model_cr2` | `whill_msgs/ModelCr2State` |
| sub | `/whill/controller/joy` | `sensor_msgs/Joy` |
| sub | `/whill/controller/cmd_vel` | `geometry_msgs/Twist` |

Standard `/odom` and `/battery_state` are **not** provided by the upstream driver
on `humble`. A bridge node may be added in a later milestone if downstream
packages need them.

## Status

| Step | Status |
|------|--------|
| `vcs import` of upstream | done |
| `colcon build --packages-up-to whill` | done (6 packages, clean) |
| `dialout` membership for serial access | pending (script ready; needs re-login) |
| `/dev/ttyUSB0` enumerated when WHILL connected | pending (real-hardware test) |
| `whill_launch.py` reaches the chair and publishes state | pending (real-hardware test) |
| Joystick drives the chair while ROS 2 receives telemetry | pending (real-hardware test) |

The first two are reproducible from any clean clone. The remaining four require
the actual CR2 connected over USB and will be marked off in the PR comment once
verified.
