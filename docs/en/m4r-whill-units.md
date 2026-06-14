# M4-R prerequisite: pin down the ModelCr2State unit ambiguity on real hardware

Language: [日本語](../ja/m4r-whill-units.md) | [English](m4r-whill-units.md)

## Goal

M4-R (the odom-foundation rebuild) replaces the current TF wiring with a
wheel-odometry + IMU EKF that emits `/odometry/filtered`. The wheel-odometry
input — `/whill/odom` (`nav_msgs/Odometry`) — will be produced inside the
upstream `ros2_whill` driver itself in our chosen approach (case 1: add the
publisher to the Iruazu/ros2_whill fork). For case 1 to land cleanly, the
numeric fields of `whill_msgs/ModelCr2State` must be converted to SI units
(m, m/s, rad, rad/s) before being assembled into a differential-drive
odometry message. The msg definition
(`src/third_party/ros2_whill_interfaces/whill_msgs/msg/ModelCr2State.msg`)
carries no unit annotation:

```
int32 battery_power
float32 battery_current
float32 right_motor_angle   # unit: unknown (deg / rad / encoder count)
float32 left_motor_angle    # same
float32 right_motor_speed   # unit: unknown (m/s / km/h / rpm)
float32 left_motor_speed    # same
int32 power_on
int32 speed_mode_indicator
int32 error
```

Without nailing this down first, the publisher we land in the fork can be off
by a sign or by an order of magnitude, and the downstream EKF + Nav2 debug
loop becomes unable to isolate the cause. This issue exists so that the units
are pinned on the real chair *before* M4-R starts, and recorded here as a
fixed-value table that the fork patch later transcribes.

The four items to pin in this issue:

- The unit of `right_motor_angle` and `left_motor_angle`.
- The unit of `right_motor_speed` and `left_motor_speed`.
- Rear (drive) wheel tyre diameter — needed for distance conversion and
  odometry integration.
- Tread width (the centre-to-centre distance between the rear wheels) —
  needed for the differential-drive angular-rate calculation.

The pinned values feed into M4R-1 (the fork patch). They appear inside
`whill_node.cpp::OnStatesModelCr2Timer()` as `WHEEL_RADIUS`, `TREAD`,
`ANGLE_TO_RAD`, `SPEED_TO_MPS` (either hard-coded constants or launch
parameters — that choice is for M4R-1). This document is the *procedure*,
not the patch; the patch itself lives in M4R-1.

## Host environment

- A real WHILL Model CR2 chair (the one M2 brought up against the driver).
- The `whill_driver` node is bringup-ready and publishes
  `/whill/states/model_cr2` (`whill_msgs/ModelCr2State`).
  - Bringup command: `ros2 launch whill_bringup whill_launch.py`
    (resolves to
    `src/third_party/ros2_whill/whill_bringup/launch/whill_launch.py`).
- A joystick (`/whill/controller/joy`) or velocity command
  (`/whill/controller/cmd_vel`) for driving the chair.
- A 5 m-plus measuring tape and masking tape (or chalk) for floor marks.
- A flat floor with a 1 m straight run, and enough space to jack the rear
  wheels up off the floor (blocks, bricks, or a stable jack).
- Two operators are recommended (one drives, one runs the stopwatch / log
  capture in parallel).

## Procedure 1: one-revolution test — pin the motor_angle unit

Goal: decide whether `right_motor_angle` and `left_motor_angle` are reported
in deg, in rad, or in encoder counts.

The reason for jacking the rear wheels up: if the chair rolls along the
floor during the test, a small drive-wheel slip leaks into the angle delta
and the threshold between deg / rad / encoder becomes ambiguous. Spinning
the wheel in free air removes that confound.

Steps:

1. Jack the chair up so that the left and right drive wheels (rear) are
   fully off the floor. Visually confirm the frame is not rocking.
2. Stick one piece of masking tape on the right tyre sidewall as a fiducial
   (12 o'clock). It will serve as the physical "one revolution" marker.
   Repeat for the left wheel.
3. Terminal 1: launch the driver:

   ```bash
   ros2 launch whill_bringup whill_launch.py
   ```

4. Terminal 2: start logging:

   ```bash
   ros2 topic echo /whill/states/model_cr2 | tee /tmp/cr2_rot.log
   ```

5. Terminal 3 (or the joystick): drive the right wheel slowly. ros2_whill
   has no per-wheel command, so send a small `angular.z` on
   `/whill/controller/cmd_vel`. With the wheels in free air this spins one
   wheel positive and the other negative. Keep the rate slow enough to
   track the tape mark by eye (about 0.5 rad/s is fine).
6. Stop the moment the tape mark returns to 12 o'clock (one full revolution
   completed).
7. From the log, read `right_motor_angle` just before the wheel started
   moving (A_start) and immediately after the stop (A_end). Take the delta:

   ```
   ΔA = A_end - A_start
   ```

8. Repeat the same for the left wheel (`left_motor_angle`). Upstream
   ros2_whill is expected to treat the two wheels symmetrically, so both
   should land on the same unit scale. If they do not, repeat the test (you
   probably misread the tape mark, or stopped between log samples).

Decision table:

| Value of ΔA | Unit | Notes |
|-------------|------|-------|
| `ΔA ≈ 360` | **deg** | rad conversion is `× π/180` |
| `ΔA ≈ 6.283` (= 2π) | **rad** | already SI |
| `ΔA` is a large integer (e.g. 4096, 500, 1000, …) | **encoder count** | record N (pulses / rev) as a fixed value |

For the encoder case, once N is known the rad conversion is `× 2π / N`. If
N is not a "round" number (a power of two, or a clean multiple of 100), an
upstream gear ratio is likely baked in; note in the results table whether
the count is post-gear or directly on the wheel.

Cross-check via upstream code (informational, not authoritative): starting
from `whill_node.cpp::OnStatesModelCr2Timer()` you can follow into
`ReceiveDataset1()` to see where in the byte stream `motor_angle` is parsed
and what scale factor (if any) is applied. Use this if the measured values
disagree with the table.

## Procedure 2: 1 m forward run — pin the motor_speed unit and cross-check motor_angle

Goal: decide whether `right/left_motor_speed` is reported in m/s, km/h, or
rpm, and at the same time confirm that the `motor_angle` unit pinned in
Procedure 1 produces the correct distance over 1 m of straight driving.

Steps:

1. Lower the chair off the jack so the drive wheels are on the floor. The
   tyre pressure / hardness should be the same as during real operation
   (the loaded effective diameter differs from the free-spin one).
2. Mark a start line and a goal line on the floor with masking tape.
   Distance: exactly 1.000 m by the tape.
3. Terminal 1: start logging:

   ```bash
   ros2 topic echo /whill/states/model_cr2 | tee /tmp/cr2_drive.log
   ```

4. A second operator readies a stopwatch and places the chair at the start.
   Drive forward at as constant a speed as you can manage, using either the
   joystick or `/whill/controller/cmd_vel`. Measure T [s] — the time from
   when the rear wheel crosses the start line to when it crosses the goal
   line.
5. From the log, compute the average `right_motor_speed` V during the
   constant-speed mid-section (exclude the accel / decel ends). awk or a
   pocket calculator is fine.

Theoretical speed: V_theory = 1 [m] / T [s] = 1/T [m/s]

Decision table:

| Value of V | Unit | Notes |
|------------|------|-------|
| `V ≈ 1/T` | **m/s** | scale 1.0 |
| `V ≈ 3.6/T` | **km/h** | m/s conversion is `÷ 3.6` |
| `V is large` (matches the value back-computed from the tyre circumference) | **rpm** | m/s conversion is `× WHEEL_RADIUS × 2π / 60` |

Distance cross-check:

1. Read `right_motor_angle` at the start and goal of the same run; take ΔA
   (in the unit pinned in Procedure 1).
2. Normalise ΔA to radians:

   - If deg: `ΔA_rad = ΔA × π / 180`
   - If rad: `ΔA_rad = ΔA`
   - If encoder count (N pulses/rev): `ΔA_rad = ΔA × 2π / N`

3. Compute the distance: `distance_calc = WHEEL_RADIUS × ΔA_rad`
   (where WHEEL_RADIUS is the value pinned in Procedure 3).
4. The error against 1.000 m must be within 5 %. If it exceeds 5 %, the
   suspect list is:

   - Procedure 1 mis-identified the unit (deg vs rad vs encoder).
   - WHEEL_RADIUS does not match the nominal value (pressure or wear has
     moved the effective diameter).
   - The drive wheels slipped more than expected on the floor (drive speed
     too high).

   To isolate, redo the run at half the speed and recompute.

## Procedure 3: tyre diameter and tread width

Goal: capture the vehicle geometry the differential-drive odometry
calculation needs — translating per-wheel velocities into linear and
angular rates — from both the URDF nominal values and the real-hardware
measurement, and record the gap.

Nominal values, where to find them:

- `src/third_party/ros2_whill/whill_description/urdf/whill_model_cr2.urdf`:
  the `left_wheel` / `right_wheel` rear-drive links carry
  `radius="0.1325"` (= 0.265 m diameter).
- Tread width is derivable from the URDF joint origins, but the most
  explicit primary source is the inline comment in
  `whill_node.cpp::OnControllerCmdVel()` (`wheel_tread: 0.496`), which
  states 0.496 m.

Measured values:

- Tyre diameter: measure the outer diameter of the rear wheel with the tape
  while it sits loaded on the floor. The point is to capture the *loaded
  effective* diameter (i.e. with at least an empty-chair load applied),
  because the running WHEEL_RADIUS is sensitive to load and pressure — and
  if you measure unloaded you will see the discrepancy surface as a 1 m run
  error.
- Tread width: measure the centre-to-centre distance between the two rear
  tyres with the tape. The accurate way is to measure both the inner and
  outer distances and average them ((inner + outer) / 2) to get the centre
  line.

How to pick the adopted value:

- Prefer the measured value (it reflects this individual chair's wear and
  inflation).
- If the value also needs to match the URDF (e.g. to keep rviz and
  odometry consistent), use the nominal value but record the gap to the
  measured one separately.
- If the measured-vs-nominal gap is 2 % or more, write it explicitly into
  the notes column of the results table.

## Results (filled in by the user after the hardware test)

- Measurement date:
- Operator:
- whill_driver git SHA (`git -C src/third_party/ros2_whill log -1 --format=%H`):

### Units

| Field | Unit (deg / rad / encoder / m/s / km/h / rpm) | Value per revolution or per 1 m | Notes |
|-------|---------|--------|------|
| `right_motor_angle` |         |        |      |
| `left_motor_angle`  |         |        |      |
| `right_motor_speed` |         |        |      |
| `left_motor_speed`  |         |        |      |

### Geometry

odometry uses the radius (`WHEEL_RADIUS`), so if you measure the
diameter, halve it before writing it into the radius row. Pasting the
diameter directly into `WHEEL_RADIUS` makes odometry report roughly
twice the actual distance.

| Item | Nominal (source: URDF etc.) | Measured | Adopted (for odometry) |
|------|---------|--------|---------------|
| Tyre diameter |        |       | (record the radius in the row below) |
| Tyre radius (= `WHEEL_RADIUS`) |        |       |                 |
| Tread width (= `TREAD`)   |        |       |                 |

### 1 m forward-run error

- Computed distance: `<formula and value>`
- Difference against measured 1 m: ` % `
- Pass / fail (pass if < 5 %):

## Transcribing into M4R-1

The pinned values land in M4R-1 (case 1) — the fork of `ros2_whill` — by
extending `whill_driver/src/whill_node.cpp::OnStatesModelCr2Timer()` to
publish `/whill/odom`. A skeleton is shown below; the full implementation
belongs to the M4R-1 patch, so this is only the differential-drive
outline:

```cpp
// Additions to whill_node.cpp::OnStatesModelCr2Timer().
// The numbers come from the results section in docs/en/m4r-whill-units.md.
//
// Whether to pin these as compile-time constants or expose them as ROS
// parameters is a decision for M4R-1. If we want to swap calibration per
// chair, prefer parameters; if we are confident they are upstream spec,
// constants are fine.

// Include <cmath> for M_PI. Note that M_PI is a POSIX extension, not
// part of standard C++; on environments where it is not defined, either
// add `constexpr double M_PI = 3.14159265358979323846;` yourself or pass
// `-D_USE_MATH_DEFINES` to the compiler.
#include <cmath>

// Vehicle geometry (the adopted values from Procedure 3).
constexpr double WHEEL_RADIUS = ???;   // [m] loaded effective radius of the rear wheels
constexpr double TREAD        = ???;   // [m] centre-to-centre distance between rear wheels

// motor_angle → rad conversion (pinned in Procedure 1).
//   if deg:     M_PI / 180.0
//   if rad:     1.0
//   if encoder: 2.0 * M_PI / N   (N = counts per revolution)
constexpr double ANGLE_TO_RAD = ???;

// motor_speed → m/s conversion (pinned in Procedure 2).
//   if m/s:  1.0
//   if km/h: 1.0 / 3.6
//   if rpm:  WHEEL_RADIUS * 2.0 * M_PI / 60.0
constexpr double SPEED_TO_MPS = ???;

void WhillNode::OnStatesModelCr2Timer()
{
  auto msg = std::make_shared<whill_msgs::msg::ModelCr2State>();
  if (whill_->ReceiveDataset1(msg) < 1) {return;}
  states_model_cr2_pub_->publish(*msg);

  // Differential-drive odometry sketch. This document is the unit-pinning
  // procedure, so the implementation details — time keeping, init, covariance,
  // frame_id — are deferred to M4R-1.
  const double v_right = msg->right_motor_speed * SPEED_TO_MPS;
  const double v_left  = msg->left_motor_speed  * SPEED_TO_MPS;
  const double v_lin   = (v_right + v_left) / 2.0;        // [m/s] linear velocity
  const double w_ang   = (v_right - v_left) / TREAD;      // [rad/s] angular velocity
  // Integrate (x, y, yaw) against dt, build a nav_msgs/Odometry, publish.
  // (Details: M4R-1.)
}
```

The `???` placeholders must be filled with the pinned values by the time
the M4R-1 patch reaches review. The reviewer should treat "the results
table in this document is populated" as a precondition for accepting the
fork patch.

## Related

- M4R-1 (case 1): add the `/whill/odom` publisher to the Iruazu/ros2_whill
  fork. The pinned values from this document are transcribed into the C++
  patch in that PR.
- Strategy: [`plans/2026-06-11-platform-pivot.md`](plans/2026-06-11-platform-pivot.md)
  §4 (M4-R positioning) and §6 (M4-R acceptance criterion: `/odometry/filtered`
  emitted from wheel + IMU, terminal error within tolerance on a 10 m
  hand-push straight run).
- Upstream source: `src/third_party/ros2_whill/whill_driver/src/whill_node.cpp`
  (`OnStatesModelCr2Timer()` and `ReceiveDataset1()` are where motor_angle
  and motor_speed are parsed out of the byte stream — useful for the
  cross-check noted in the procedures).
- Upstream geometry: `src/third_party/ros2_whill/whill_description/urdf/whill_model_cr2.urdf`
  (rear wheels at `radius="0.1325"`) and the
  `wheel_tread: 0.496` comment in `whill_node.cpp::OnControllerCmdVel()`.
- ADR 0001: [`decisions/0001-docs-i18n.md`](decisions/0001-docs-i18n.md) —
  new documents are authored in parallel under `docs/ja/` and `docs/en/`.
