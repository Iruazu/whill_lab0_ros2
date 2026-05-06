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
| Connection | RS232C from the chair's communication port through a custom red/white/black 3-wire harness terminating in a Prolific PL2303 USB-serial cable → host `/dev/ttyUSB0` |
| Baud / framing | 38400 / 8N2 (CSTOPB) — fixed by the WHILL spec |
| Operator input | WHILL onboard joystick (no external teleop in M2) |

The official CR2 communication interface (per
[WHILL Control System Protocol Specification](https://github.com/WHILL/whill_control_system_protocol_specification),
section 8.1.2) is RS232C on a D-sub 9pin connector — Pin2 TXD, Pin3 RXD, Pin5 GND.
The lab harness in this repo bypasses the D-sub and exposes the same lines
through a JST-side wiring; functionally equivalent and verified working as of
2026-05-06.

## Upstream packages

This milestone uses two official packages from [whill-labs](https://github.com/whill-labs):

- [`ros2_whill`](https://github.com/whill-labs/ros2_whill) — the driver, bringup,
  description, and examples (5 packages: `whill`, `whill_bringup`,
  `whill_description`, `whill_driver`, `whill_examples`).
- [`ros2_whill_interfaces`](https://github.com/whill-labs/ros2_whill_interfaces)
  — message and service definitions (`whill_msgs` package). Defines
  `ModelCr2State`, `SpeedProfile`, and the `SetPower` / `SetSpeedProfile` /
  `SetBatterySaving` services.

`ros2_whill` is pinned to a personal fork
[`Iruazu/ros2_whill`](https://github.com/Iruazu/ros2_whill) on its `humble`
branch — the fork tracks upstream `whill-labs/ros2_whill` plus
[PR #1: Send SetPower(ON) and re-enable body joystick during Initialize()](https://github.com/Iruazu/ros2_whill/pull/1),
which fixes the cold-boot quirk described below. `ros2_whill_interfaces` stays
on its upstream `humble` branch. The `humble` branches expose only features
common across CR-series models; `crystal-devel` adds CR-specific extras and is
not used here.

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

## Cold-boot quirk and patch

A freshly powered-on Model CR2 does **not** start streaming Dataset1 frames
just because the host sent `StartSendingData` — the chair's communication
subsystem only wakes up after it receives `SetPower(ON)` (per spec section 5).
Upstream `whill-labs/ros2_whill` `Initialize()` only sends `StartSendingData`,
so on a cold boot the driver opens `/dev/ttyUSB0` cleanly, looks healthy, and
publishes nothing — `ros2 topic echo /whill/states/model_cr2` and
`ros2 topic hz` both stay silent.

The fork's [PR #1](https://github.com/Iruazu/ros2_whill/pull/1) adds two
commands to `WhillNode::Initialize()` immediately before `StartSendingData`:

1. `SetPowerOn()` × 2 with a 10 ms / 2 ms gap — mirrors the retry pattern
   already used by `OnSetPowerSrv`, satisfies the spec's
   "re-issue SetPower if no response within 5 ms".
2. `SendSetJoystickCommandWithLocal()` — `SetPower(ON)` switches the chair
   into host-control mode and silently disables the body joystick; this hands
   control back so the body stick keeps working when no ROS controller is
   publishing.

When ROS-side control later publishes to `/whill/controller/cmd_vel` or
`/whill/controller/joy`, the existing `SendSetVelocityCommand` /
`SendSetJoystickCommand` paths flip the chair back to host-control on demand —
no regression.

### LCD display interpretation

The CR2's onboard LCD shows `BATTERY_POWER` (0–100 %) during normal
operation. A two-digit number like `93` is the battery percentage, **not** an
error code. The startup LED is briefly red on power-on as part of the normal
boot sequence — also not an error.

### Diagnosing "driver runs but no topic"

If `ros2 launch whill_bringup whill_launch.py` opens the port without errors
but `ros2 topic echo /whill/states/model_cr2` is silent, in priority order:

1. **Check that the patched fork is actually in use.** `git -C
   src/third_party/ros2_whill log --oneline -3` should include
   `Send SetPower(ON) and re-enable body joystick during Initialize()`. If
   not, re-run `./scripts/import_upstream.sh` and rebuild.
2. **Power-cycle the chair**, then re-launch — confirms the
   `SetPower → Dataset` handshake from a known state.
3. **Confirm the cable came up.** `ls /dev/serial/by-id/` should list the
   PL2303 entry; `ls -l /dev/ttyUSB0` should be `crw-rw---- root dialout`.
4. **Confirm dialout membership in the launching shell.** `groups | grep
   dialout` — if missing, `newgrp dialout` (or re-login) before launching.

## Daily runbook (after WHILL room arrival)

The lab PC moves between networks. After plugging into the WHILL-room
network, in the shell that will launch the driver:

```bash
# 1. Drop the campus HTTP proxy so the local USB / ROS path is unaffected
unset HTTP_PROXY HTTPS_PROXY FTP_PROXY http_proxy https_proxy ftp_proxy

# 2. Confirm dialout is effective in this shell
groups | grep dialout || newgrp dialout

# 3. Power the WHILL on (LCD shows battery %, e.g. "93"), then plug USB
ls /dev/ttyUSB0   # should exist

# 4. Source overlays and launch
source /opt/ros/humble/setup.bash
source ~/whill_lab0_ros2/install/setup.bash
ros2 launch whill_bringup whill_launch.py
```

In a second terminal (same `unset` + `source` prelude):

```bash
ros2 topic hz /whill/states/model_cr2     # expect ~2.5 Hz at publish_interval_ms=400
ros2 topic echo /whill/states/model_cr2 --once
```

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
| `dialout` membership for serial access | done |
| `/dev/ttyUSB0` enumerated when WHILL connected | done (Prolific PL2303, `usb-Prolific_Technology_Inc._USB-Serial_Controller-if00-port0`) |
| `whill_launch.py` reaches the chair and publishes state | done — verified 2026-05-06, after applying the cold-boot patch |
| Joystick drives the chair while ROS 2 receives telemetry | done — 10 s capture in `/tmp/m2_test.log` shows `right/left_motor_speed` swinging across ±1.5, `battery_current` 0–122 |

All M2 acceptance criteria met. The cold-boot patch lives in fork PR #1; see
[Session log: 2026-05-06](session-2026-05-06.md) for the full investigation
narrative. M3 (sensors) is the next milestone — see
[`m3-sensors.md`](m3-sensors.md).
