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

## Results (hardware test completed 2026-06-14)

- Measurement date: 2026-06-14
- Operator: Iruazu
- whill_driver git SHA: `ceebd45` (Iruazu/ros2_whill humble fork)
- Verification log: `/tmp/cr2_full.log` (~2 m forward run, single trial,
  a rough sanity-check without tape-measured distance)

#### Units (upstream source declaration + hardware sanity check)

| Field | Unit | Observed value over the 1 m run | Notes |
|-------|------|---------------------------------|-------|
| `right_motor_angle` | rad | cumulative -27.6 rad after wrap correction (~2 m run) = -4.39 revolutions | wraps at ±π; decreases when driving forward |
| `left_motor_angle`  | rad | same as above (forward run only) | increases when driving forward (opposite sign to right) |
| `right_motor_speed` | km/h | peak 1.036 km/h ≈ 0.288 m/s | negative when driving forward |
| `left_motor_speed`  | km/h | peak 1.052 km/h ≈ 0.292 m/s | positive when driving forward |

Source evidence: `src/third_party/ros2_whill/whill_driver/src/model_cr2/whill.cpp:62-69`
- L63: `// The value for converting [0.001rad] to [rad]` (`kMotorAngleFactor = 0.001`)
- L68: `// The value for converting [0.004km/h] to [km/h]` (`kMotorSpeedFactor = 0.004`)

#### Geometry

| Item | Nominal (source) | Measured | Adopted |
|------|------------------|----------|---------|
| Tyre diameter | 0.265 m (URDF `whill_modelc.urdf`) | (not measured) | 0.265 m |
| Tyre radius (= `WHEEL_RADIUS`) | 0.1325 m | (not measured) | 0.1325 m |
| Tread width (= `TREAD`) | 0.496 m (`whill_node.cpp:115` comment) | (not measured) | 0.496 m |

#### 1 m forward-run error

- Distance verification with a tape measure was not performed, so the
  numeric pass / fail judgement is **deferred to M4R-3 (after EKF
  integration)**
- The goal of this issue — pinning the units (motor_angle = rad,
  motor_speed = km/h) — is met
- Peak motor_speed = 1.036 km/h = 0.288 m/s sits inside the expected
  joystick-forward range and is consistent with the upstream source
  declaration

#### Additional notes for M4R-1 (surfaced by the hardware test)

1. **motor_angle wraps at ±π**: observed 3 wrap events over ~2 m of
   driving. An angle-based odometry must process the deltas in a
   wrap-aware way; we use the ROS 2 standard
   `angles::shortest_angular_distance()` (see the next section).
2. **Left/right sign inversion**: when driving forward,
   `right_motor_speed` is negative, `left_motor_speed` is positive,
   `right_motor_angle` is decreasing, and `left_motor_angle` is
   increasing. Under the
   `angles::shortest_angular_distance(prev, curr) = (curr - prev)` sign
   convention, **the right wheel delta is negated** to make "forward is
   positive" hold (so that d_right_raw < 0 and d_left_raw > 0 both
   become positive after the flip).
3. **Upstream WHILL packages ship no odometry implementation**: none of
   `whill-labs/ros2_whill`, `whill-labs/ros2_whill_applications`, or
   `whill-labs/whill_visualization` contains any odometry code. M4R-1
   must implement it locally.

## Transcribing into M4R-1

The pinned values land in M4R-1 (case 1) — the fork of `ros2_whill` — by
extending `whill_driver/src/whill_node.cpp::OnStatesModelCr2Timer()` to
publish `/whill/odom`. A skeleton is shown below; the full implementation
belongs to the M4R-1 patch, so this is only the differential-drive
outline:

```cpp
// Code to add to whill_node.cpp (implemented as part of the M4R-1 fork patch).
//
// Design choices (pinned 2026-06-14):
// - Wrap handling: ROS 2 standard angles package (ros-humble-angles)
//   - Avoids reinventing WrappedAngleDiff(); the ROS community maintains it
// - Odometry method: angle-based (derive velocity from motor_angle deltas)
//   - Robust against the low ~3 Hz publish rate (a velocity-based variant
//     is sensitive to the speed quantisation)
//   - Rejected alternative: velocity-based odometry
//     (consumes the 0.004 km/h motor_speed quantum directly)
// - Left/right sign: negate the right wheel so "forward is positive"
//   - Hardware log (2026-06-14): forward driving makes right_motor_angle
//     decrease and left_motor_angle increase.
//     angles::shortest_angular_distance(prev, curr) returns (curr - prev),
//     so d_right_raw < 0 and d_left_raw > 0 during forward motion.
//     Negating the right wheel makes both positive, and
//     v_angular = (v_right - v_left) / TREAD then matches the ROS REP-103
//     convention (left turn = positive).

#include <angles/angles.h>
#include <cmath>

// Unit conversion factors (whill.cpp:62-69 source comments + 2026-06-14 hardware sanity check)
constexpr double WHEEL_RADIUS = 0.1325;  // [m] URDF whill_modelc.urdf nominal
constexpr double TREAD        = 0.496;   // [m] whill_node.cpp:115 comment

// Previous sample held for the angle-based delta calculation
whill_msgs::msg::ModelCr2State::SharedPtr prev_state_;
rclcpp::Time prev_stamp_;

// Accumulated pose (relative to the map origin; initialised to 0 by the launch-time reset)
double x_ = 0.0, y_ = 0.0, yaw_ = 0.0;

void WhillNode::OnStatesModelCr2Timer()
{
  auto msg = std::make_shared<whill_msgs::msg::ModelCr2State>();
  if (whill_->ReceiveDataset1(msg) < 1) {return;}

  const auto now = this->now();

  // First sample: nothing to diff against, so just stash it and return
  if (!prev_state_) {
    prev_state_ = msg;
    prev_stamp_ = now;
    states_model_cr2_pub_->publish(*msg);
    return;
  }

  const double dt = (now - prev_stamp_).seconds();
  // Guard against bad dt values (dropped publishes / clock rewind)
  if (dt <= 0.0 || dt > 1.0) {
    prev_state_ = msg;
    prev_stamp_ = now;
    states_model_cr2_pub_->publish(*msg);
    return;
  }

  // Wrap-aware angle deltas; the right wheel is sign-flipped so "forward
  // is positive" (hardware log 2026-06-14: forward drives right_motor_angle
  // down, left_motor_angle up).
  const double d_right = -angles::shortest_angular_distance(
      prev_state_->right_motor_angle, msg->right_motor_angle);
  const double d_left  =  angles::shortest_angular_distance(
      prev_state_->left_motor_angle,  msg->left_motor_angle);

  // Angular rate (rad/s) -> tyre-contact-point linear velocity (m/s)
  const double v_right_mps = (d_right / dt) * WHEEL_RADIUS;
  const double v_left_mps  = (d_left  / dt) * WHEEL_RADIUS;

  // Differential-drive odometry. v_angular = (right - left) / TREAD matches
  // the ROS REP-103 convention (positive angular.z = left turn): during a
  // left turn the right wheel spins faster, so v_right > v_left and ω > 0.
  const double v_linear  = 0.5 * (v_right_mps + v_left_mps);
  const double v_angular = (v_right_mps - v_left_mps) / TREAD;

  // Pose integration (midpoint method, yaw integrated independently)
  yaw_ += v_angular * dt;
  // Normalise yaw to [-π, π] (prevents unbounded accumulation)
  yaw_ = angles::normalize_angle(yaw_);
  x_   += v_linear * std::cos(yaw_) * dt;
  y_   += v_linear * std::sin(yaw_) * dt;

  // Assemble and publish a nav_msgs/Odometry
  // (frame_id: "odom", child_frame_id: "base_link", quaternion built from yaw_)
  // Details are deferred to M4R-1.

  states_model_cr2_pub_->publish(*msg);
  prev_state_ = msg;
  prev_stamp_ = now;
}
```

### Dependency additions for the M4R-1 fork patch

The fork patch on `Iruazu/ros2_whill` must also include:

1. Additions to `whill_driver/package.xml`:
   ```xml
   <depend>angles</depend>
   <depend>nav_msgs</depend>
   <depend>tf2</depend>
   <depend>tf2_geometry_msgs</depend>
   ```

2. Additions to `whill_driver/CMakeLists.txt`:
   ```cmake
   find_package(angles REQUIRED)
   find_package(nav_msgs REQUIRED)
   find_package(tf2 REQUIRED)
   find_package(tf2_geometry_msgs REQUIRED)

   target_link_libraries(whill_node
     # ... existing ...
     angles::angles
   )
   ament_target_dependencies(whill_node
     # ... existing ...
     nav_msgs tf2 tf2_geometry_msgs
   )
   ```

3. Add the `/whill/odom` publisher (`whill_node.hpp` and `whill_node.cpp::Initialize()`):
   ```cpp
   odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/whill/odom", 10);
   ```

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
